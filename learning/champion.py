import hashlib
import json
import math
import os
import random
import re
import tempfile
from collections import defaultdict
from statistics import median
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERSION = 'champion-cost-v1'
REWARD_VERSION = 'champion-dense-native-v3'
KINDS = ('signal', 'fast', 'analog', 'current_val', 'supply', 'sw', 'gnd')

SPACE = {
    'pfmul': (1.1, 1.2, 1.25, 1.3, 1.4, 1.5),
    'pf0': (.4, .6, .8, 1., 1.5, 2.), 'hinc': (.4, .7, 1., 1.4, 2.),
    'goal_from': (0, 5, 10, 20, 30, 40), 'all_close': (0., 1., 2., 3., 5., 8.),
    'chain_detour': (1.3, 1.5, 2., 2.5, 3.),
    'cross': (0., 1., 2., 3., 4., 6.), 'separate': (2., 2.5, 3., 3.5, 4.),
    'via_cost': (.3, .45, .6, .8, 1., 1.2),
    'dense': (0., .05, .1, .2, .4), 'dense_inner': (0., .05, .1, .2),
    'separate_w': (.5, .75, 1., 1.5, 2.),
    'fine': (0, 1),
    'cost_tight': (0., .1, .3, .6, 1., 2.),
    'shaping': ('45r','organic','per_net_kind'),
    'placer_style': ('prior','island'), 'router_style': ('prior','convergence'),
}
for _kind in KINDS:
    for _prefix in ('cost_via_', 'cost_inner_'):
        SPACE[_prefix + _kind] = (.5, .7, .85, 1., 1.15, 1.4, 1.8, 2.)

    SPACE['cost_rank_' + _kind] = (.5, .75, 1., 1.25, 1.5, 2.)
CATEGORIES={'shaping','placer_style','router_style'}
PLACER = {'cross', 'separate', 'via_cost', 'dense', 'dense_inner', 'separate_w', 'fine','placer_style'}
ROUTER = set(SPACE) - PLACER
DEFAULT = dict(pfmul=1.25, pf0=1., hinc=1., goal_from=20, all_close=0., chain_detour=2.,
               cross=2., separate=2., via_cost=.6, dense=0., dense_inner=0., separate_w=1., fine=1,cost_tight=0.)
DEFAULT.update({key: 1. for key in SPACE if key.startswith(('cost_via_','cost_inner_','cost_rank_'))})
DEFAULT.update(shaping='45r',placer_style='prior',router_style='prior')


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def file():
    return Path(os.environ.get('AUTOROUTER_CHAMPION') or
                Path(os.environ.get('LEARNING_CURVE_MAIN') or ROOT) / 'learning/model/champion_cfg.json')


def parameter(cfg):
    out = {}
    for key, value in cfg.items():
        if key not in SPACE:
            raise ValueError('Unknown champion cost: ' + key)
        if key in CATEGORIES:
            if value not in SPACE[key]:raise ValueError('Unknown champion style: '+key)
            out[key]=value;continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Non-finite champion cost: ' + key)
        if not min(SPACE[key]) <= value <= max(SPACE[key]):
            raise ValueError('Champion cost outside search space: ' + key)
        if key in ('goal_from', 'fine') and int(value) != value:
            raise ValueError(key+' must be an integer')
        out[key] = value
    return out


def cli(key, value):
    return str(int(value)) if key in ('goal_from', 'fine') else str(value)


def check_cli(args):
    parameter({k: float(v) for k,v in args.items() if k.startswith('cost_')})


def tool_options(values, tool):
    keys=ROUTER if tool=='router' else PLACER
    options={k:cli(k,v) for k,v in values.items() if k in keys and k not in CATEGORIES}
    if tool=='router':
        if 'shaping' in values:options['style']='45' if values['shaping']=='45r' else values['shaping']
        if values.get('router_style')=='convergence':
            options.update(conflict_site='exact',source_free='1',convergence='local',pfmax='1000',short_via='8')
    elif values.get('placer_style')=='island':
        options.update(island_physics='1',pin_post_legalize='1',bga_against='1',operate_new_free='1')
    return options


def net_class(A):
    v = str(A.get('version', '10')).split('+')[0]
    bga = any('BGA' in b.upper() for b in A.get('blocks', []))
    return 'v%s:L%d_%s_s%d' % (v, A['cu_layers'], 'bga' if bga else 'qfn',
                              int(round(float(A.get('level', 0)) / 2)) * 2)


def board_class(B):
    layers = B.get('cu_layers', 4)
    if isinstance(layers, list): layers = len(layers)
    bga = any('BGA' in str(f).upper() for f in B.get('fps', []))
    pads = B.get('pads', [])
    outline = B.get('outline') or []
    def points(x):
        if isinstance(x, (list, tuple)):
            if len(x) == 2 and all(isinstance(v, (int, float)) for v in x): yield x
            else:
                for y in x: yield from points(y)
        elif isinstance(x, dict):
            for y in x.values(): yield from points(y)
    ps = list(points(outline)) or [[p['x'], p['y']] for p in pads]
    area = max(1., (max(p[0] for p in ps)-min(p[0] for p in ps)) *
               (max(p[1] for p in ps)-min(p[1] for p in ps))) if ps else 1.
    st = min(8, max(0, 2 * int(len(pads) / area * 2)))
    return 'v11:L%d_%s_s%d' % (int(layers), 'bga' if bga else 'qfn', st)


def read_in(path=None):
    path = Path(path or file())
    if not path.exists(): return dict(version=VERSION, generation=0, netclasses={}, history=[])
    doc = json.loads(path.read_text())
    if doc.get('version') != VERSION: raise ValueError('Champion version invalid')
    for key, entry in doc['netclasses'].items():
        if not re.fullmatch(r'v\d+:L\d+_(bga|qfn)_s\d+', key): raise ValueError('ChampionClass')
        parameter(entry['parameter'])
        if not entry.get('evidence_sha'): raise ValueError('Champion without gate evidence')
    return doc


def freeze(path):
    doc=read_in();path=Path(path).resolve();path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix=path.name+'.',suffix='.new',delete=False) as f:
        json.dump(doc,f,sort_keys=True,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
        tmp=Path(f.name)
    os.replace(tmp,path)
    return dict(fpath=str(path),model_sha=sha(doc),generation=doc['generation'])


def selection(doc, nc, exact=False):
    if nc in doc['netclasses']: return nc, doc['netclasses'][nc]
    if exact: return nc, None
    v, suffix = nc.split(':'); family, stage = suffix.rsplit('_s', 1)
    candidates = [k for k in doc['netclasses'] if k.split(':')[1].rsplit('_s', 1)[0] == family]
    if not candidates: return nc, None
    key = min(candidates, key=lambda k: (k.split(':')[0] != v,
                                         abs(int(k.rsplit('_s', 1)[1])-int(stage)), k))
    return key, doc['netclasses'][key]


def apply(args, B, tool):
    if str(args.get('std', '1')) == '0' or str(args.get('champion', '1')) == '0': return None
    path = Path(args.get('champion_cfg') or file())
    doc = read_in(path)
    requested = args.get('champion_net_class') or board_class(B)
    key, entry = selection(doc, requested, exact='champion_net_class' in args)
    if entry is None: return None
    keys = ROUTER if tool == 'router' else PLACER
    values = {k: v for k, v in entry['parameter'].items() if k in keys}
    args.update(tool_options(values,tool))
    info = dict(version=VERSION, generation=doc['generation'], net_class=key,
                requested=requested, parameter=values, model_sha=sha(doc), evidence_sha=entry['evidence_sha'])
    print('Champion ' + json.dumps(info, sort_keys=True), flush=True)
    return info


def numbers(z):
    k = z.get('metrics') or z.get('outcome') or {}
    out = []
    for key in ('conflict_list', 'open_items'):
        value = k.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            return None
        out.append(value)
    native=z.get('native_champion')
    if native is not None:


        opened=native.get('open_items')
        if isinstance(opened,bool) or not isinstance(opened,(int,float)) or not math.isfinite(opened) or opened<0:
            return None
        out[1]=max(out[1],opened)
    return out


def native_error(z):
    value = (z.get('native_champion') or {}).get('faults',
        (z.get('outcome') or {}).get('drc_error', z.get('drc_error')))
    return value if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None


def total_remainder(z):
    rest, drc = numbers(z), native_error(z)
    return sum(rest) + drc if rest is not None and drc is not None else None


def reward(z):
    if z.get('not_routed') or z.get('generator_failure'): return None
    if z.get('is_valid') is None:


        curve = z.get('conflict_history') or []
        if z.get('cur_state') != 'early_abort' or not curve: return None
        values = curve[-1][:2]
        if len(values) != 2 or any(type(v) not in (int,float) or not math.isfinite(v) or v < 0 for v in values): return None
        return (1., sum(math.log1p(v) for v in values), 1., *map(math.log1p, values))
    rest = numbers(z)
    if rest is None:


        collisions=(z.get('placer_metrics') or {}).get('collisions')
        if z.get('cur_state')=='placer_collision' and isinstance(collisions,(int,float)) and math.isfinite(collisions) and collisions>0:
            return (1., 1e6+collisions, 1e6, 1e6, 0.)
        return None
    if z['is_valid'] is True:
        native=z.get('native_champion')
        if not native or native_error(z)!=0 or native.get('open_items')!=0 or any(rest):return None
        rating = z.get('score_v3')
        if not isinstance(rating, (int, float)) or not math.isfinite(rating): return None
        return (0., 0., 0., 0., math.log1p(max(0., rating)))
    drc = native_error(z)

    dense = sum(math.log1p(v) for v in rest) + (math.log1p(drc) if drc is not None else 0.)
    return (1., dense, float(drc is None), math.log1p(rest[0]), math.log1p(rest[1]))


def row_class(z):
    try:
        return net_class(dict(version=z.get('version', '10'), cu_layers=z['cu_layers'], blocks=z['blocks'],
                           level=z.get('level_task', z.get('level', 0))))
    except (KeyError, TypeError, ValueError): return None

def family(nc):
    return nc.split(':', 1)[1].rsplit('_s', 1)[0]


def effect(rows):
    data = []
    seen = set()
    for r in rows:
        nc = row_class(r); costs = r.get('cfg', {}).get('cost')
        rest = total_remainder(r)
        if not nc or not costs or rest is None or r.get('cur_state') != 'ok': continue
        try: costs = parameter(costs)
        except ValueError: continue
        cfg = r['cfg']; design = (nc, r.get('design_family', r.get('task')))
        if design[1] is None: continue
        identity = (design, sha(costs), r.get('code_sha256'), cfg.get('seed'))
        if identity in seen: continue
        seen.add(identity)
        stratum = (nc, r.get('code_sha256'), cfg.get('budget', 'cpu'), cfg.get('work_max'),
                   (r.get('budget') or {}).get('iter'))
        data.append((stratum, design, cfg, costs, math.log1p(rest)))
    centers = defaultdict(list)
    for st, _, _, _, value in data: centers[st].append(value)
    centers = {st: median(vs) for st, vs in centers.items()}
    result = []
    for key in sorted(SPACE):
        groups = defaultdict(dict); pairs = defaultdict(dict)
        for st, design, cfg, costs, value in data:
            if key not in costs: continue
            groups[costs[key]][design] = value - centers[st]
            fixed = dict(cfg); fixed['cost'] = {k:v for k,v in costs.items() if k != key}
            pairs[(design, sha(fixed), st)][costs[key]] = value
        means = [sum(g.values())/len(g) for g in groups.values() if len(g) >= 2]
        paired = [max(v.values())-min(v.values()) for v in pairs.values() if len(v) >= 2]
        effect = max(means)-min(means) if len(means) >= 2 else None
        result.append(dict(knob=key, effect=effect, designs=sum(len(v) for v in groups.values()),
                           exact_pairs=len(paired), pair_effect=median(paired) if paired else None))
    return sorted(result, key=lambda x: (x['effect'] is None, -(x['effect'] or 0.), x['knob']))



START_KNOBS = ('hinc', 'pfmul', 'pf0', 'goal_from', 'all_close',
                 'cost_via_signal', 'cost_tight', 'chain_detour')


def active_knobs(rows):
    ranked = [x['knob'] for x in effect(rows) if x['effect'] is not None]
    return tuple((ranked + [k for k in START_KNOBS if k not in ranked])[:8])


def merged(rows, nc):
    exact, prior, seen = [], [], set()
    for r in rows[-10000:]:
        rk = row_class(r)
        if not rk or rk.split(':')[0] not in ('v10','v11') or family(rk) != family(nc) or r.get('cur_state') == 'code_changed': continue
        b = reward(r)
        if b is None or r.get('cur_state') == 'placer_collision': continue
        cfg = r.get('cfg') or {}
        identity = (rk, r.get('design_family', r.get('task')), sha(cfg.get('cost') or {}))
        if identity in seen: continue
        seen.add(identity)
        (exact if rk == nc else prior).append((r, b))
    weight = min(1., 8./len(prior)) if prior else 0.
    return [(r,b,1.) for r,b in exact] + [(r,b,weight) for r,b in prior]


def suggestion(rows, nc, rnd=None, champion=None):
    rnd = rnd or random.Random()
    examples = merged(rows, nc)
    base = dict(DEFAULT, **(champion or {}))
    key = rnd.choice(active_knobs(rows))
    values = SPACE[key]
    if rnd.random() < .25:
        old = base[key]
        i = min(range(len(values)), key=lambda j: 0 if values[j] == old else
                (abs(values[j]-old) if key not in CATEGORIES else 1))
        alternatives = [v for v in (i-1, i+1) if 0 <= v < len(values)]
        base[key] = values[rnd.choice(alternatives)]
    else:
        samples = []
        for value in values:
            hits = [(r,b,w) for r,b,w in examples if
                    r.get('cfg', {}).get('cost', {}).get(key, r.get('cfg', {}).get(key)) == value]
            observed = [(r,b,w) for r,b,w in hits if type(r.get('is_valid')) is bool]
            good = sum(w for r,b,w in observed if b[0] == 0)
            total = sum(w for r,b,w in observed)
            probability = rnd.betavariate(1+good, 1+total-good)
            n = sum(w for r,b,w in hits)
            mean = sum((0. if b[0] == 0 else b[1])*w for r,b,w in hits)/n if n else 4.


            samples.append(((1-probability)*rnd.gauss(mean, 1/math.sqrt(n+1)), value))
        base[key] = min(samples)[1]
    return parameter(base)


def challenger(rows, nc, excluded=()):
    pooled = [(r,b,w) for r,b,w in merged(rows,nc)
              if type(r.get('is_valid')) is bool and total_remainder(r) is not None]
    centers = defaultdict(list)
    for r,b,w in pooled:
        design = (row_class(r),r.get('design_family',r.get('task')))
        centers[design].append(0. if b[0] == 0 else b[1])
    centers = {k:sum(v)/len(v) for k,v in centers.items()}
    groups = defaultdict(list); params = {}; exact = set()
    for r,b,w in pooled:
        costs = r.get('cfg', {}).get('cost')
        if not costs: continue
        key = sha(costs)
        if key in excluded: continue
        params[key] = parameter(costs)
        if row_class(r) == nc: exact.add(key)
        design = (row_class(r),r.get('design_family',r.get('task')))
        groups[key].append((b[0],(0. if b[0] == 0 else b[1])-centers[design],w))
    def rank(key):
        hits=groups[key];n=sum(w for _,_,w in hits)
        return (sum(f*w for f,_,w in hits)/n,sum(v*w for _,v,w in hits)/n,key)
    return params[min(exact, key=rank)] if exact else None


def quality(a, b):
    from analysis.rule_metrics import gate_fields, amount
    ra, rb = a.get('rule') or {}, b.get('rule') or {}
    errors = []
    if not ra.get('version') or ra.get('version') != rb.get('version'): errors.append('MeasurementVersion')
    for key in ('score_v3', 'coupling'):
        x, y = amount(ra, key), amount(rb, key)
        if x is None or y is None or y > x+1e-8: errors.append(key)
    for section, keys in (('design_rules', set(gate_fields(ra.get('design_rules') or {})) |
                          set(gate_fields(rb.get('design_rules') or {}))),
                          ('topology', {'parallel_rest_mm', 'branch_middle', 'sharp', 'tap_close_mm'})):
        for key in keys:
            x, y = amount(ra.get(section), key), amount(rb.get(section), key)
            if x is None or y is None or y > x+1e-8: errors.append(section+'.'+key)
    if ra.get('design_rules', {}).get('faults') or rb.get('design_rules', {}).get('faults'): errors.append('RuleError')
    return errors


def significance(deltas):
    wins = sum(d > 0 for d in deltas); losses = sum(d < 0 for d in deltas)
    n = wins + losses
    p = sum(math.comb(n, k) for k in range(wins, n+1))/2**n if n else 1.
    return dict(test='sign_single_sided_exact', gains=wins, losses=losses,
                bindings=len(deltas)-n, p=p, alpha=.05)


def same_placement(a, b):
    if any(z.get('cur_state') != 'placer_collision' or z.get('is_valid') is not False for z in (a,b)): return False
    ma, mb = a.get('champion_measurement') or {}, b.get('champion_measurement') or {}
    digest = ma.get('placement_sha')
    return (isinstance(digest, str) and len(digest) == 64 and digest == mb.get('placement_sha')
            and bool(ma.get('input_')) and ma.get('input_') == mb.get('input_'))


def gate_check(before, after):
    reasons = []
    if not before or set(before) != set(after): return dict(taken=False, reasons=['Set incomplete'])
    lost = []; counts = [0,0]; rest = [0,0]; complete = True; excluded = []; deltas = []
    for name in sorted(before):
        a,b = before[name],after[name]
        skip = same_placement(a,b)
        if skip: excluded.append(name)
        pair = []
        for i,z in enumerate((a,b)):
            collision = (z.get('placer_metrics') or {}).get('collisions')
            known_place = (z.get('cur_state') == 'placer_collision' and z.get('is_valid') is False
                           and type(collision) in (int,float) and math.isfinite(collision) and collision > 0)
            if type(z.get('is_valid')) is not bool or (z.get('cur_state') != 'ok' and not known_place):
                reasons.append(name+': unknown completion')
            r = total_remainder(z)
            if z.get('is_valid') is True:
                native = z.get('native_champion') or {}
                if native.get('faults') != 0 or native.get('open_items') != 0 or r != 0:
                    reasons.append(name+': contradictory/missing validity evidence')
            counts[i] += z.get('is_valid') is True
            if skip: continue
            if r is None:
                complete = False
                if not known_place: reasons.append(name+': residual measurement missing')
            else: rest[i] += r
            pair.append(r)
        if not skip and len(pair) == 2 and all(v is not None for v in pair): deltas.append(pair[0]-pair[1])
        if a.get('is_valid') is True:
            if b.get('is_valid') is not True: lost.append(name)
            else: reasons.extend(name+': '+e for e in quality(a,b))
    if lost: reasons.append('Valid boards lost: '+','.join(lost))
    statistic = significance(deltas)
    improved = counts[1] > counts[0] or (complete and counts[1] == counts[0]
                  and rest[1] < rest[0] and statistic['p'] <= statistic['alpha'])
    if not improved: reasons.append('No validity gain or significant paired total residual gain')
    return dict(taken=not reasons, reasons=reasons, is_valid=counts,
                rest=rest if complete else None, lost=lost,
                rest_pairs=len(deltas), placement_excluded=excluded, statistics=statistic)
