import hashlib
import json
import math
from pathlib import Path

MODEL = Path(__file__).with_name('early_abort.json')
Q1_MODEL = Path(__file__).with_name('early_abort_q1.json')
VERSION = 'early_abort-v1'
ROUNDS = (8, 12, 16, 20, 24)
REST_MAX = 40
MIN_ROUND = 12
MIN_CONFLICTS = 80
MIN_THRESHOLD = .95
FEATURES = ('round_no', 'log_conflicts', 'trend4', 'remaining_share', 'conflicts_nets',
            'conflicts_pads', 'log_pads', 'layers4', 'layers6', 'layers8',
            'pad_density', 'factor', 'bga', 'level', 'cpu_share')


def active(amount):
    return str(amount) in ('1', 'q1')


def model_path(cfg):

    if str(cfg.get('early_abort')) == 'q1':
        if cfg.get('early_abort_model'):
            raise ValueError('q1 has a fixed model; replacement model only with early_abort=1')
        return Q1_MODEL
    return cfg.get('early_abort_model') or MODEL


def aborted(z):
    info = z.get('learning_data') or {}
    return bool(z.get('cur_state') == 'early_abort' or z.get('early_abort')
                or info.get('cur_state') == 'early_abort' or info.get('cause') == 'early_abort')


def context_verify(context):
    if not isinstance(context, dict):
        raise ValueError('Early-abort context must be an object')
    keys = ('pads', 'net_list', 'area_mm2', 'cu_layers', 'factor', 'level', 'bga')
    if any(type(context.get(k)) not in (int, float) or not math.isfinite(context[k]) for k in keys):
        raise ValueError('incomplete/non-finite early-abort context')
    if (min(context[k] for k in ('pads', 'net_list', 'area_mm2', 'factor')) <= 0
            or context['cu_layers'] not in (4, 6, 8) or context['level'] < 0
            or context['bga'] not in (0, 1)):
        raise ValueError('invalid early-abort geometry')
    return context


def feats(rows, context, budget):
    if not isinstance(rows, list) or any(not isinstance(r, dict) for r in rows): return None
    rr = [r for r in rows if r.get('phase') == 'negotiation']
    if not rr or rr[-1].get('round_no') not in ROUNDS or len(rr) < 4:
        return None
    try:

        for r in rr:
            if any(type(r.get(k)) not in (int, float) or not math.isfinite(r[k])
                   or r[k] < 0 for k in ('round_no', 'conflict_list', 'cpu_s')):
                return None
            if r['round_no'] != int(r['round_no']): return None
        context_verify(context)
        pads, nets = float(context['pads']), float(context['net_list'])
        area = float(context['area_mm2'])
        layers = int(context['cu_layers'])
        factor = float(context['factor'])
        level = float(context['level'])
        bga = float(context['bga'])
        cpu = float(rr[-1]['cpu_s'])
        if (min(pads, nets, area, budget) <= 0 or not math.isfinite(budget)
                or layers not in (4, 6, 8) or factor <= 0 or level < 0 or bga not in (0., 1.)):
            return None
        vals = [float(r['conflict_list']) for r in rr]
        c = vals[-1]
        x = [rr[-1]['round_no'], math.log1p(c), (c - vals[-4]) / max(1., vals[-4]),
             c / max(1., vals[0]), c / nets, c / pads, math.log1p(pads),
             float(layers == 4), float(layers == 6), float(layers == 8),
             pads / area, factor, bga, level, cpu / budget]
        return x if all(math.isfinite(v) for v in x) and cpu > 0 else None
    except (KeyError, ValueError, TypeError, OverflowError): return None


def model_load(path=MODEL):
    data = Path(path).read_bytes()
    m = json.loads(data)
    comparison = (isinstance(m, dict) and m.get('version') == 'early_abort-v2'
                 and m.get('usage') == 'loss_comparison'
                 and type(m.get('loss_rate_percent')) is int
                 and m['loss_rate_percent'] in (0, 1, 2, 3)
                 and m.get('min_round') == 8 and m.get('min_conflicts') == MIN_CONFLICTS)
    if (not isinstance(m, dict) or m.get('version') not in (VERSION, 'early_abort-v2')
            or (m.get('version') == 'early_abort-v2' and not comparison) or m.get('feats') != list(FEATURES)
            or m.get('status') != 'experimental' or m.get('rounds_n') != list(ROUNDS)
            or m.get('rest_max') != REST_MAX or m.get('min_round') != (8 if comparison else MIN_ROUND)
            or m.get('min_conflicts') != MIN_CONFLICTS):
        raise ValueError('invalid early-abort model contract')
    for k in ('medium', 'scale', 'weights'):
        if (not isinstance(m.get(k), list) or len(m[k]) != len(FEATURES)
                or not all(type(v) in (int, float) and math.isfinite(v) for v in m[k])):
            raise ValueError('invalid model coefficients')
    if (any(v <= 0 for v in m['scale'])
            or type(m.get('threshold')) not in (int, float)
            or not (0. if comparison else MIN_THRESHOLD) <= m['threshold'] <= 1
            or type(m.get('bias')) not in (int, float) or not math.isfinite(m['bias'])):
        raise ValueError('invalid model scale/threshold')
    m = dict(m, sha256=hashlib.sha256(data).hexdigest())
    return m


def decision(rows, context, budget, model):
    x = feats(rows, context, budget)
    if x is None: return None
    rr = [r for r in rows if r.get('phase') == 'negotiation']
    if model.get('comparison_rule') == 'Round 8, >80 conflicts' and rr[-1]['round_no'] != 8:
        return None

    if rr[-1]['conflict_list'] <= REST_MAX: return None
    if rr[-1]['round_no'] < model['min_round'] or rr[-1]['conflict_list'] <= model['min_conflicts']:
        return None

    if any(b['round_no'] <= a['round_no'] or b['cpu_s'] < a['cpu_s'] for a, b in zip(rr, rr[1:])):
        return None
    logit = model['bias'] + sum(w * (v - mu) / s for w, v, mu, s in
                               zip(model['weights'], x, model['medium'], model['scale']))
    score = 1 / (1 + math.exp(-max(-700., min(700., logit))))
    if score < model['threshold']: return None
    return dict(version=model['version'], model_sha256=model['sha256'], score=score,
                threshold=model['threshold'], round_no=rr[-1]['round_no'],
                conflict_list=rr[-1]['conflict_list'], cpu_s=rr[-1]['cpu_s'],
                cpu_budget_s=budget, feats=dict(zip(FEATURES, x)),
                goal='no valid result within this budget', censored=True)


def context_task(a):
    return dict(pads=a['pads'], net_list=a['net_list'], area_mm2=a['area_mm2'],
                cu_layers=a['cu_layers'], factor=a['factor'], level=a.get('level', 0),
                bga=int(any('BGA' in b for b in a['blocks'])))


def generator_board_verify(board):
    src = board.get('src') if isinstance(board, dict) else None
    if not isinstance(src, str) or not src.startswith('task seed=') or '(learning/tasks.py v' not in src:
        raise ValueError('native boards: predictor locked until own pair measurements')


def context_board(board, cu_layers):

    raise ValueError('native boards: predictor locked until own pair measurements')


def options(cfg, context=None):
    if not active(cfg.get('early_abort', '0')): return []
    model_path(cfg)
    out = ['early_abort=' + str(cfg['early_abort'])]
    if cfg.get('early_abort_model'): out.append('early_abort_model=' + str(cfg['early_abort_model']))
    if context is not None:
        out.append('early_abort_context=' + json.dumps(context, sort_keys=True, separators=(',', ':')))
    return out
