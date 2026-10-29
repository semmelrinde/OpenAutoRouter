import collections
import hashlib
import json
import math
import os
import re
from pathlib import Path
import sys
import threading
import time
import zlib

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
import numpy as np
from threadpoolctl import ThreadpoolController
from functools import wraps
from learning.features import EXTRA
from learning.model import FEATURES

_CONTROLLER = None
_CONTROLLER_LOCK = threading.Lock()


def one_thread(fn):
    @wraps(fn)
    def call(*args, **kwargs):
        global _CONTROLLER
        if _CONTROLLER is None:
            with _CONTROLLER_LOCK:
                if _CONTROLLER is None:

                    import sklearn.ensemble
                    _CONTROLLER = ThreadpoolController()
        with _CONTROLLER.limit(limits=1): return fn(*args, **kwargs)
    return call


VERSION = 2
SEED = 20261007
GATES = ['score_v3', 'coupling', 'parallel_rest_mm', 'branch_middle', 'sharp', 'tap_close_mm']
CFG = ['cross', 'separate', 'via_cost', 'via_f', 'pfmul', 'short_via', 'single_test', 'critique', 'fine']
NAMES = FEATURES + EXTRA + ['cfg_' + k for k in CFG] + ['budget_cpu_s', 'budget_iter', 'style_organic']


def finite(v):
    try:
        n = float(v)
        return n if math.isfinite(n) and n >= 0 else None
    except (ValueError, TypeError):
        return None


def family(z):

    if 'task' in z: return 'selfplay_seed_%s' % z['task']
    if z.get('design_family'): return str(z['design_family']).lower()
    from learning.families import family as source_family
    q = source_family(z)
    if '/' in q: return q


    f = re.sub(r'^\d+_', '', str(z.get('family') or z.get('board') or '')).lower().removesuffix('_tight')
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]


def role(f):
    n = zlib.crc32(f.encode()) % 5
    return {0: 'test', 1: 'calibration', 2: 'choice'}.get(n, 'fit')

def test_families():
    out = set()
    for p in (R / 'examples').glob('corpus*test*.json'):
        for b in json.loads(p.read_text()).get('boards', []):
            out.update(v for k, v in b.items() if k in ('name', 'pcbench') and isinstance(v, str))
    return out


def actual_test(f, banned):

    return f in {family(dict(board=t)) for t in banned}


def task(z):

    return json.dumps([z.get('board'), z.get('task'), z.get('version'),
                       z.get('code_sha256') or z.get('code'), z.get('router_sha256'), z.get('netclasses_sha256'),
                       z.get('budget'), z.get('gnd_layers'), z.get('factor'), z.get('level_task'),
                       (z.get('value_selection') or {}).get('pool_id')], sort_keys=True)


def evaluation_status(z):
    r = z.get('rule') or {}
    version = str(r.get('version') or z.get('evaluation_version') or '')


    rev = tuple((1, int(t)) if t.isdigit() else (0, t) for t in re.findall(r'\d+|\D+', version))
    return bool(r), rev, sum(rule(z, k) is not None for k in GATES)


def profile_data(z):
    r = (z.get('rule') or {}).get('profile_data')
    p = z.get('profile_data')
    if r and p and r != p: raise ValueError('contradictory evaluation profile context')
    return str(r or p or 'balanced')


def profile_verify(M, Z):
    if not M.get('profile_data'): raise ValueError('Model without profile contract: retrain')
    if any(profile_data(z) != M['profile_data'] for z in Z):
        raise ValueError('Evaluation profile does not match the frozen model')


def model_contract(M):
    return M.get('version') == VERSION and M.get('names') == NAMES and bool(M.get('profile_data'))


def read_in(paths):
    from learning.early_abort import aborted
    unique = {}; state = collections.Counter(); banned = test_families()
    for p in paths:
        if not Path(p).exists(): continue
        with open(p) as fh:
            for line in fh:
                state['raw_data'] += 1
                try: z = json.loads(line)
                except ValueError:
                    state['incomplete'] += 1; continue
                if aborted(z):
                    state['censored'] += 1; continue
                if not z.get('feats'):
                    state['without_features'] += 1; continue
                f = family(z)
                if not f or actual_test(f, banned):
                    state['testboard_or_without_family'] += 1; continue
                if z.get('code_changed') or z.get('faults') or z.get('cur_state') in ('faults', 'wall_limit', 'placer_wall_limit', 'placer_error'):
                    state['technical_error'] += 1; continue

                if not z.get('cur_state') and z.get('router_rc') not in (None, 0):
                    state['old_timeout'] += 1; continue
                if 'is_valid' not in z or not isinstance(z['is_valid'], bool):
                    state['without_validity'] += 1; continue
                z['_family'], z['_task'] = f, task(z)
                key = (z['_task'], json.dumps(z.get('cfg'), sort_keys=True), z.get('candidate'))

                if key in unique: state['doubled'] += 1
                if key not in unique or evaluation_status(z) >= evaluation_status(unique[key]): unique[key] = z
    Z = list(unique.values()); state['lines'] = len(Z)
    state['families'] = len({z['_family'] for z in Z})
    state['score_v3'] = sum(rating(z) is not None for z in Z)
    state['cpu'] = sum(cpu(z) is not None for z in Z)
    state['rule'] = sum(all(rule(z, k) is not None for k in GATES) for z in Z)
    return Z, dict(state)

def rating(z):
    return finite((z.get('rule') or {}).get('score_v3', z.get('score_v3')))


def minimax(z):

    return finite((z.get('rule') or {}).get('minimax'))


def cpu(z):
    v = finite((z.get('router') or {}).get('cpu_s'))
    if v is None and z.get('format') == 'v2': v = finite(z.get('route_s'))
    return v

def rule(z, k):
    if k == 'score_v3': return rating(z)
    r = z.get('rule') or {}
    return finite(r.get('coupling') if k == 'coupling' else (r.get('topology') or {}).get(k))


def input_(z):
    m = dict(z.get('feats') or {})
    m.update({'cfg_' + k: (z.get('cfg') or {}).get(k) for k in CFG})
    m.update(budget_cpu_s=(z.get('budget') or {}).get('cpu_s'), budget_iter=(z.get('budget') or {}).get('iter'),
             style_organic=int((z.get('cfg') or {}).get('style', (z.get('cfg') or {}).get('shaping')) == 'organic'))
    return [np.nan if finite(m.get(k)) is None else float(m[k]) for k in NAMES]


def matrix(Z):
    return np.asarray([input_(z) for z in Z], float).reshape(len(Z), len(NAMES))


def transform(M, Z):
    X = matrix(Z)[:, M['indices']]
    return np.where(np.isnan(X), M['median'], X)


@one_thread
def fit(Z, category='extra'):
    from sklearn.ensemble import ExtraTreesClassifier, ExtraTreesRegressor, HistGradientBoostingClassifier, HistGradientBoostingRegressor
    profiles = {profile_data(z) for z in Z}
    if len(profiles) != 1: raise ValueError('several evaluation profiles: use separate data/model folders')
    X = matrix(Z)

    indices = [i for i in range(len(NAMES)) if np.isfinite(X[:, i]).sum() >= 5]
    if not indices: raise ValueError('too few observed features')
    X = X[:, indices]; median = np.nanmedian(X, axis=0); X = np.where(np.isnan(X), median, X)
    M = dict(version=VERSION, profile_data=next(iter(profiles)), category=category, indices=indices, median=median, names=NAMES,
             fit_families=sorted({z['_family'] for z in Z}))
    if category == 'extra':
        clf = ExtraTreesClassifier(n_estimators=64, min_samples_leaf=4, max_depth=12, n_jobs=1, random_state=SEED)
        reg = lambda: ExtraTreesRegressor(n_estimators=64, min_samples_leaf=4, max_depth=12, n_jobs=1, random_state=SEED)
    elif category == 'boost':
        clf = HistGradientBoostingClassifier(max_iter=80, max_leaf_nodes=7, min_samples_leaf=8, random_state=SEED)
        reg = lambda: HistGradientBoostingRegressor(max_iter=80, max_leaf_nodes=7, min_samples_leaf=8, random_state=SEED)
    else: raise ValueError('unknown model type')
    y = [int(z['is_valid']) for z in Z]
    M['clf'] = clf.fit(X, y) if len(set(y)) > 1 else float(np.mean(y))
    for key, target in [('rating', rating), ('cpu', cpu), ('minimax', minimax)]:
        ok = [i for i, z in enumerate(Z) if target(z) is not None and (key != 'rating' or z['is_valid'])]

        M[key] = reg().fit(X[ok], np.log1p([target(Z[i]) for i in ok])) if len(ok) >= 8 else None
        M[key + '_n'] = len(ok)
    M['calibrator'] = None
    return M


def raw_p(M, X):
    c = M['clf']
    return c.predict_proba(X)[:, list(c.classes_).index(1)] if hasattr(c, 'classes_') else np.full(len(X), c)


def logits(p):
    p = np.clip(p, 1e-6, 1 - 1e-6)
    return np.log(p / (1 - p)).reshape(-1, 1)


@one_thread
def calibrate(M, Z):
    profile_verify(M, Z)
    from sklearn.linear_model import LogisticRegression
    y = [int(z['is_valid']) for z in Z]
    if len(Z) >= 20 and len(set(y)) > 1 and len({z['_family'] for z in Z}) >= 3:
        M['calibrator'] = LogisticRegression(C=1., random_state=SEED).fit(logits(raw_p(M, transform(M, Z))), y)
    M['calibration_families'] = sorted({z['_family'] for z in Z})
    return M


@one_thread
def prediction(M, Z):
    if M['version'] != VERSION or M['names'] != NAMES: raise ValueError('unsuitable model version')
    profile_verify(M, Z)
    X = transform(M, Z); raw = raw_p(M, X)
    p = M['calibrator'].predict_proba(logits(raw))[:, 1] if M['calibrator'] is not None else raw
    out = []
    values = {k: np.maximum(0, np.expm1(np.clip(M[k].predict(X), 0, 25))) if M[k] is not None else [None] * len(Z)
              for k in ('rating', 'cpu', 'minimax')}
    for i in range(len(Z)):
        out.append(dict(p_valid=float(p[i]), p_raw=float(raw[i]),
                        score_v3=None if values['rating'][i] is None else float(values['rating'][i]),
                        cpu_s=None if values['cpu'][i] is None else float(values['cpu'][i]),
                        minimax=None if values['minimax'][i] is None else float(values['minimax'][i])))
    return out


def heuristic(z):
    m = z['feats']
    return m['mst'] * (1 + .02 * m['crossings']) * (1 + max(0., m['rudy_max'] - 1))


def expectation(p):
    n = p['minimax'] if p['minimax'] is not None else p['score_v3']
    return 2 * (1 - p['p_valid']) + p['p_valid'] * (n / (1 + n) if n is not None else .5)


def arrange(M, Z):
    P = prediction(M, Z)
    return sorted(range(len(Z)), key=lambda i: (expectation(P[i]), heuristic(Z[i]), i)), P


def group_list(Z):
    d = collections.defaultdict(list)
    for z in Z: d[z['_task']].append(z)
    return d


def rank(z):
    n = rating(z)
    return (not z['is_valid'], n if n is not None else math.inf, z.get('candidate', 0))


def bootstrap(pairs, value, seed=SEED):
    ds = collections.defaultdict(list)
    for a, b in pairs:
        va, vb = value(a), value(b)
        if va is None or vb is None: return dict(n=len(pairs), diff=None, ci95=None)
        ds[a['_family']].append(va - vb)
    if not ds: return dict(n=0, diff=None, ci95=None)
    vals = np.array([np.mean(ds[f]) for f in sorted(ds)])
    rng = np.random.default_rng(seed)
    boot = vals[rng.integers(0, len(vals), (2000, len(vals)))].mean(1)
    return dict(n=len(pairs), families=len(ds), diff=float(vals.mean()), ci95=np.percentile(boot, [2.5, 97.5]).tolist())


def gate_check(pairs):
    valid = bootstrap(pairs, lambda z: int(z['is_valid']))
    absolute = dict(model=sum(a['is_valid'] for a, _ in pairs), basis=sum(b['is_valid'] for _, b in pairs))
    absolute['delta'] = absolute['model'] - absolute['basis']
    common = [(a, b) for a, b in pairs if a['is_valid'] and b['is_valid']]
    metrics = {k: bootstrap(common, lambda z, k=k: rule(z, k)) for k in GATES}
    ok = bool(absolute['delta'] >= 0 and valid['ci95'] and valid['ci95'][0] >= 0 and valid['families'] >= 3 and len(pairs) >= 20)
    ok &= all(v['ci95'] is not None and v.get('families', 0) >= 3 and v['ci95'][1] <= 1e-9 for v in metrics.values())
    strict = (valid['ci95'] is not None and valid['ci95'][0] > 0) or any(v['ci95'] and v['ci95'][1] < -1e-9 for v in metrics.values())
    return dict(better=bool(ok and strict), validity=valid, validity_absolute=absolute, metrics=metrics, common=len(common), tasks=len(pairs))


def thruput(details, basis='heuristic'):
    rate = collections.defaultdict(lambda: np.zeros(4))
    for v in details:
        if v['model_cpu_s'] is not None and v[basis + '_cpu_s'] is not None:
            rate[v['family']] += [v['model_valid'], v['model_cpu_s'], v[basis + '_valid'], v[basis + '_cpu_s']]
    out = dict(diff=None, ci95=None, families=len(rate), tasks=sum(
        v['model_cpu_s'] is not None and v[basis + '_cpu_s'] is not None for v in details))
    if rate:
        values = np.array([rate[f] for f in sorted(rate)]); totals = values.sum(0)
        if totals[1] > 0 and totals[3] > 0:
            rng = np.random.default_rng(SEED)
            sums = values[rng.integers(0, len(values), (2000, len(values)))].sum(1)
            ok = (sums[:, 1] > 0) & (sums[:, 3] > 0)
            diffs = 3600 * (sums[ok, 0] / sums[ok, 1] - sums[ok, 2] / sums[ok, 3])
            out.update(diff=float(3600 * (totals[0]/totals[1]-totals[2]/totals[3])), ci95=np.percentile(diffs,[2.5,97.5]).tolist())
    return out


def release_gate(t, d):


    quality = any(v['ci95'] and v['ci95'][1] < -1e-9 for v in t['metrics'].values())
    return bool(t['better'] and quality and d['tasks'] == t['tasks']
                and d['families'] >= 3 and d['ci95'] and d['ci95'][0] > 0)


def pool_cpu(ks, chosen):
    costs = [cpu(z) for z in chosen]
    placing = [finite((z.get('placer') or {}).get('cpu_s')) for z in ks]
    if any(v is None for v in costs + placing): return None
    return sum(costs) + sum(placing)


def measurement(M, Z, route=1, prior=None):
    pairs = []; details = []; pred = prediction(M, Z) if Z else []
    cal = {}
    if Z:
        y = np.array([z['is_valid'] for z in Z], float); p = np.array([v['p_valid'] for v in pred]); raw = np.array([v['p_raw'] for v in pred])
        bins = []
        for lo in np.arange(0, 1, .2):
            mask = (p >= lo) & (p < lo + .2 + (1e-9 if lo > .79 else 0))
            if mask.any(): bins.append(dict(n=int(mask.sum()), p=float(p[mask].mean()), is_valid=float(y[mask].mean())))
        cal = dict(n=len(Z), brier=float(np.mean((p-y)**2)), brier_raw=float(np.mean((raw-y)**2)), bins=bins,
                   calibrated=M['calibrator'] is not None)
    for task, ks in sorted(group_list(Z).items()):
        if len(ks) < 2: continue
        if len({(z.get('rule') or {}).get('version') or z.get('evaluation_version') for z in ks}) > 1: continue

        if any((z.get('value_selection') or {}).get('pool_n', len(ks)) != len(ks) for z in ks): continue

        if any(z['is_valid'] and rating(z) is None for z in ks): continue
        mi, _ = arrange(M, ks); fi = sorted(range(len(ks)), key=lambda i: (heuristic(ks[i]), i))
        selected = [ks[i] for i in mi[:route]]; baseline = [ks[i] for i in fi[:route]]
        a, b, oracle = min(selected, key=rank), min(baseline, key=rank), min(ks, key=rank)
        pairs.append((a, b))
        details.append(dict(family=a['_family'], task=task, candidates=len(ks), model_valid=a['is_valid'],
                            heuristic_valid=b['is_valid'], model_score=rating(a) if a['is_valid'] else None,
                            heuristic_score=rating(b) if b['is_valid'] else None, hit_model=rank(a)[:2] == rank(oracle)[:2] if oracle['is_valid'] else None,
                            hit_heuristic=rank(b)[:2] == rank(oracle)[:2] if oracle['is_valid'] else None, model_cpu_s=pool_cpu(ks, selected), heuristic_cpu_s=pool_cpu(ks, baseline)))
    def together(category):
        d = [v for v in details if v[category + '_valid']]
        timed = [v for v in details if v[category + '_cpu_s'] is not None]
        seconds = sum(v[category + '_cpu_s'] for v in timed)
        return dict(is_valid=len(d), tasks=len(details), score_medium=float(np.mean([v[category + '_score'] for v in d])) if d else None,
                    hit=float(np.mean([v['hit_' + category] for v in details if v['hit_' + category] is not None])) if any(v['hit_' + category] is not None for v in details) else None,
                    hit_tasks=sum(v['hit_' + category] is not None for v in details),
                    cpu_s=seconds, cpu_tasks=len(timed),
                    cpu_per_valid_s=seconds / sum(v[category + '_valid'] for v in timed) if any(v[category + '_valid'] for v in timed) else None,
                    valid_per_cpu_h=3600 * sum(v[category + '_valid'] for v in timed) / seconds if seconds else None)
    faults = {}
    for key, target in [('score_v3', rating), ('cpu_s', cpu)]:
        es = [abs(p[key] - target(z)) for z, p in zip(Z, pred) if p[key] is not None and target(z) is not None and (key != 'score_v3' or z['is_valid'])]
        faults[key] = dict(n=len(es), mae=float(np.mean(es)) if es else None)
    throughput = thruput(details)
    out = dict(throughput_delta=throughput, prediction_error=faults, gate_check=gate_check(pairs), model=together('model'), heuristic=together('heuristic'), calibration=cal, details=details,
               cpu_hint='Retrospective: router + all known placer CPU-s; export/DRC/shape/training not included; no live throughput.')
    if prior is not None:
        ap = []; ad = []
        for ks in group_list(Z).values():
            if len(ks) < 2 or any(z['is_valid'] and rating(z) is None for z in ks) or any((z.get('value_selection') or {}).get('pool_n', len(ks)) != len(ks) for z in ks): continue
            if len({(z.get('rule') or {}).get('version') or z.get('evaluation_version') for z in ks}) > 1: continue
            ai, _ = arrange(prior, ks); ni, _ = arrange(M, ks)
            new, old = [ks[i] for i in ni[:route]], [ks[i] for i in ai[:route]]
            a, b = min(new, key=rank), min(old, key=rank); ap.append((a, b))
            ad.append(dict(family=a['_family'], model_valid=a['is_valid'], champion_valid=b['is_valid'],
                           model_cpu_s=pool_cpu(ks, new), champion_cpu_s=pool_cpu(ks, old)))
        out['champion_gate'] = gate_check(ap)
        out['champion_throughput_delta'] = thruput(ad, 'champion')
    return out


def atomic(p, obj):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + '.new')
    tmp.write_text(json.dumps(obj, indent=1, allow_nan=False)); os.replace(tmp, p)


def save(M, path):
    import joblib
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.new'); joblib.dump(M, tmp); os.replace(tmp, path)


def train(paths, model_dir, route=1):
    Z, state = read_in(paths)
    profiles = {profile_data(z) for z in Z}
    if len(profiles) > 1: raise ValueError('several evaluation profiles: use separate data/model folders')
    parts = {r: [z for z in Z if role(z['_family']) == r] for r in ('fit', 'choice', 'calibration', 'test')}
    if len(parts['fit']) < 20 or len({z['_family'] for z in parts['fit']}) < 3:
        raise ValueError('under 20 fit rows / 3 fit families')
    pilot = {}; scores = {}
    for category in ('extra', 'boost'):
        M = calibrate(fit(parts['fit'], category), parts['calibration'])
        pilot[category] = measurement(M, parts['choice'], route)
        s = pilot[category]['model']
        scores[category] = (-s['is_valid'], s['score_medium'] if s['score_medium'] is not None else math.inf, category)
    category = min(scores, key=scores.get)
    M = calibrate(fit(parts['fit'] + parts['choice'], category), parts['calibration'])
    M['profile_data'] = next(iter(profiles), 'balanced')
    M['choice_families'] = sorted({z['_family'] for z in parts['choice']})
    M['known_families'] = sorted({z['_family'] for z in Z})
    M['id'] = hashlib.sha256(json.dumps([VERSION, M['profile_data'], category, SEED, [(z['_task'], z.get('cfg'), z['is_valid'], rating(z), minimax(z), cpu(z), input_(z))
                                                            for z in Z if role(z['_family']) != 'test']], sort_keys=True).encode()).hexdigest()[:16]
    report = dict(id=M['id'], profile_data=M['profile_data'], state=state, category=category, seed=SEED, route=route,
                  split={r: dict(lines=len(v), families=sorted({z['_family'] for z in v})) for r, v in parts.items()},
                  model_choice=pilot, test=measurement(M, parts['test'], route), released=False,
                  active_features=[NAMES[i] for i in M['indices']], targets={k: M[k+'_n'] for k in ('rating','cpu','minimax')})
    save(M, Path(model_dir) / 'candidate.joblib'); atomic(Path(model_dir) / 'candidate.json', report)
    return M, report


def verify(paths, model_dir, route=1):
    import joblib
    model_dir = Path(model_dir); Z, state = read_in(paths)
    path = model_dir / 'candidate.joblib'
    if not path.exists():
        report = train(paths, model_dir, route)[1]
        atomic(model_dir / 'last_training.json', dict(state=state, id=report['id'], released=False))
        return report
    M = joblib.load(path)
    if not model_contract(M):


        save(M, model_dir / ('old_contract_' + str(M.get('id', 'without_id')) + '.joblib'))
        report = train(paths, model_dir, route)[1]
        report['migration'] = 'old model contract: retrained, no release'
        atomic(model_dir / 'last_training.json', report)
        return report

    new = [z for z in Z if z['_family'] not in M['known_families'] and role(z['_family']) == 'test']
    profile_verify(M, new)
    master_path = model_dir / 'champion.joblib'
    prior = joblib.load(master_path) if master_path.exists() else None
    if prior is not None and not model_contract(prior): prior = None
    m = measurement(M, new, route, prior)
    ok = release_gate(m['gate_check'], m['throughput_delta']) and (prior is None or release_gate(m['champion_gate'], m['champion_throughput_delta']))

    report = dict(id=M['id'], profile_data=M['profile_data'], state=state, prospective=m, route=route, released=ok)
    if ok:
        save(M, model_dir / ('champion_' + M['id'] + '.joblib'))
        atomic(model_dir / 'champion.json', dict(id=M['id'], file='champion_' + M['id'] + '.joblib', released=True, application='pool', profile_data=M['profile_data'], route=route))
        save(M, master_path)
    atomic(model_dir / 'last_training.json', report)

    if ok or (m['gate_check']['tasks'] >= 20 and m['gate_check']['validity'].get('families', 0) >= 3):
        save(M, model_dir / ('candidate_' + M['id'] + '.joblib'))
        train(paths, model_dir, route)
    return report


class Service:
    def __init__(self, data, model, new=100, cpu_s=300, route=4):
        self.data = list(data); self.model = Path(model); self.new = max(1, new); self.cpu_s = cpu_s; self.route = route
        self.lock = threading.Lock(); self.thread = None; self.seen = -1; self.check_time = 0
        self.M = None; self.meta = {}; self.stamp = None
        p = self.model / 'last_training.json'
        if p.exists():
            try: self.seen = json.loads(p.read_text())['state']['raw_data']
            except (ValueError, KeyError): pass

    def load_data(self, pilot=False):
        import joblib
        path = self.model / ('candidate.json' if pilot else 'champion.json')
        if not path.exists(): return None, {}
        stamp = (str(path), path.stat().st_mtime_ns)
        if stamp != self.stamp:
            meta = json.loads(path.read_text())
            file = 'candidate.joblib' if pilot else meta['file']
            M = joblib.load(self.model / file)
            if M['id'] != meta['id']: raise ValueError('Model/metadata not atomic')
            self.M, self.meta, self.stamp = M, meta, stamp
        return self.M, self.meta

    def selection(self, Z, category='heuristic', route=None, explore=0):
        if not Z: return [], dict(status='empty')
        route = self.route if route is None else route
        order = sorted(range(len(Z)), key=lambda i: (heuristic(Z[i]), i)); info = dict(status='heuristic')
        if category in ('amount', 'value_auto'):
            with self.lock:
                try:
                    M, meta = self.load_data(pilot=category == 'amount')
                    code = {(z.get('code_sha256') or z.get('code'), z.get('router_sha256'), z.get('netclasses_sha256'), z.get('version')) for z in Z}
                    if len(code) > 1: raise ValueError('Pool with different code states')
                    if M is not None and (category == 'amount' or (meta.get('released') and meta.get('application') == 'pool' and meta.get('route') == route)):
                        order, P = arrange(M, Z); info = dict(status=category, id=M['id'], predictions=P)
                except Exception as e: info = dict(status='heuristic_error', faults=repr(e)[:200])
        take = order[:max(1, route)]

        rest = [i for i in range(len(Z)) if i not in take]
        if rest and explore:
            rng = np.random.default_rng(int((Z[0].get('cfg') or {}).get('seed', SEED)))
            take += rng.choice(rest, min(int(explore), len(rest)), replace=False).tolist()
        info['pool_n'] = len(Z)
        info['pool_id'] = hashlib.sha256(json.dumps([(z.get('board'), z.get('candidate'), z.get('cfg')) for z in Z], sort_keys=True).encode()).hexdigest()[:16]
        info['chosen'] = take
        return take, info
    def verify(self):
        with self.lock:
            if time.monotonic() - self.check_time < 30 or (self.thread and self.thread.is_alive()): return
            self.check_time = time.monotonic()
            n = 0
            for p in self.data:
                if Path(p).exists():
                    with open(p) as f: n += sum(1 for _ in f)
            if n == 0 or (self.seen >= 0 and n - self.seen < self.new): return
            from common import slots as PL
            if PL.tight(): return
            self.seen = n
            def train():
                import fcntl
                self.model.mkdir(parents=True, exist_ok=True)
                with open(self.model / 'training.lock', 'a') as lock:
                    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError: return
                    cmd = [str(R / '.venv/bin/python'), str(R / 'learning/value_model.py'), 'verify',
                           'data=' + ','.join(map(str, self.data)), 'model=' + str(self.model), 'route=%d' % self.route]
                    r = PL.job(cmd, cpu_s=self.cpu_s, log=str(self.model / 'training.log'), cwd=str(R))
                    atomic(self.model / 'training_run.json', r)
                    if r['cur_state'] != 'ok':
                        with self.lock: self.seen = -1
            self.thread = threading.Thread(target=train, daemon=True); self.thread.start()


def main():
    kw = dict(s.split('=', 1) for s in sys.argv[2:] if '=' in s)
    paths = kw['data'].split(','); model_dir = kw['model']; t = time.process_time()
    if sys.argv[1] == 'train': report = train(paths, model_dir, int(kw.get('route', '1')))[1]
    elif sys.argv[1] == 'verify': report = verify(paths, model_dir, int(kw.get('route', '1')))
    else: raise ValueError('train|verify')
    report['cpu_s'] = time.process_time() - t; report['load'] = os.getloadavg()[0]
    atomic(Path(model_dir) / 'run.json', report)
    print(json.dumps({k: v for k, v in report.items() if k not in ('test', 'model_choice', 'split')}, indent=1))


if __name__ == '__main__': main()
