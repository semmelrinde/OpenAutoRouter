"""Move ranker for the improvement loop.

postprocessing/improve.py tries what a human does while routing (shift, rotate or swap a part, move a
pull-up) as what-if experiments and logs one line per experiment. This module
  1. collects such lines on validly routed boards,
  2. trains a small linear ranking model (conditional logit per decision with L2; L2 logistic regression and
     histogram gradient boosting for comparison) that predicts per move the chance of being accepted, and measures it
     honestly with one board family held out at a time (hit rate among the first k moves against the fixed heuristic
     order and against random),
  3. provides the ranking for postprocessing/improve.py ranker=<model.json>.
Features: move kind, trigger (bottleneck, conflict, detour), heuristic gain estimate, distance, rotation, part role,
pads, local density, nearness to conflicts, affected nets and the start score.

usage: python3 learning/moves.py generate <train_folder> <work_folder> [n=20] ...
       python3 learning/moves.py collect <candidate_folder> ...
       python3 learning/moves.py train [model=model.json] [k=4] [l2=10]
       python3 learning/moves.py comparison <train_folder> <work_folder> ranker=<model.json>
"""
import collections, fcntl, glob, hashlib, json, math, os, random, shutil, sys, time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
VPY = os.path.join(R, '.venv', 'bin', 'python')
DATA = os.path.join(R, 'learning', 'data', 'moves.jsonl')
START = os.path.join(R, 'learning', 'data', 'moves_start.jsonl')
READ = DATA + ',' + START
MODEL = os.path.join(R, 'learning', 'model', 'moves.json')
KINDS = ('shift', 'rotate', 'swap', 'offset')
REASONS = ('bottleneck', 'conflict', 'detour', 'offset')
FUNCTIONS = ('decoupling', 'pull', 'series_r', 'slow', 'sensitive', 'misc')
NET_KINDS = ('signal', 'current_val', 'supply', 'analog', 'fast', 'sw')
WHOLE_MM = 50.0



def _f(x, d=0.0):
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else d


def function_group(f):
    f = (f or '').lower()
    if f.startswith('decoupling'): return 'decoupling'
    if f.startswith('pull'): return 'pull'
    if f == 'series_r': return 'series_r'
    if f in ('divider', 'rc_element'): return 'slow'
    if f in ('bootstrap', 'coil_sw', 'crystal_load', 'feedback', 'feedback_c', 'setting_r', 'shunt'): return 'sensitive'
    return 'misc'


def feats(before, rank=None, gain_max=None):
    b = before or {}; m = {}
    mv = b.get('move') or {}; category = mv.get('type'); mem = mv.get('members') or {}
    for a in KINDS: m['kind_' + a] = 1.0 if category == a else 0.0
    rs = b.get('reason') or {}
    for g in REASONS: m['occasion_' + g] = 1.0 if rs.get('type') == g else 0.0
    gs = _f(rs.get('gain_estimate'))
    m['gain_log'] = math.log1p(max(gs, 0.0))
    m['rank_log'] = math.log1p(rank or 0); m['rank_first'] = 1.0 if not rank else 0.0
    m['gain_rel'] = gs / gain_max if gain_max and gain_max > 0 else 1.0
    paths = [math.hypot(_f(v.get('dx_mm')), _f(v.get('dy_mm'))) for v in mem.values()] or [0.0]
    m['way_mm'] = max(paths); m['way_log'] = math.log1p(max(paths))
    rot = [(_f(v.get('rotation_deg')) % 360) for v in mem.values()] or [0.0]
    m['rotate_90'] = 1.0 if any(r in (90.0, 270.0) for r in rot) else 0.0
    m['rotate_180'] = 1.0 if any(r == 180.0 for r in rot) else 0.0
    m['parts_n'] = float(len(mem))
    T = b.get('part_list') or [{}]
    t0 = T[0]
    m['decoupling'] = 1.0 if any(t.get('decoupling') for t in T) else 0.0
    m['pads_n'] = max(_f(t.get('n_pads')) for t in T)
    m['side_B'] = 1.0 if t0.get('board_side') == 'B' else 0.0
    ww = b.get('travel_range') or {}
    w0 = ww.get(t0.get('ref')) or (next(iter(ww.values())) if ww else {})
    fg = function_group(t0.get('function') or (w0 or {}).get('keyname'))
    for g in FUNCTIONS: m['function_' + g] = 1.0 if fg == g else 0.0
    m['ww_known'] = 1.0 if w0 else 0.0
    m['ww_whole'] = 1.0 if (w0 or {}).get('whole') else 0.0
    m['ww_max'] = WHOLE_MM if (w0 or {}).get('whole') else min(_f((w0 or {}).get('max')), WHOLE_MM)
    m['ww_importance_log'] = math.log1p(_f((w0 or {}).get('importance')))
    m['price'] = _f(b.get('price'))
    m['price_rel'] = m['price'] / (1.0 + gs)

    lo = ((b.get('local_occupancy') or {}).get('part_list') or {}).get(t0.get('ref')) or {}
    board_side = t0.get('board_side') or 'F'
    for h in ('2', '5'):
        e = lo.get(h) or {}
        ca = e.get('courtyard_share') or {}
        m['occupied_%s' % h] = _f(ca.get(board_side)); m['occupied_other_%s' % h] = _f(ca.get('B' if board_side == 'F' else 'F'))
        m['neighbors_%s' % h] = _f(e.get('neighbors'))
    um = b.get('surroundings') or {}
    ut = (um.get('part_list') or {}).get(t0.get('ref')) or {}
    m['surroundings_known'] = 1.0 if ut else 0.0
    for h in ('1.5', '4'):
        e = (ut.get('window') or {}).get(h) or {}
        m['copper_%s' % h] = math.log1p(_f(e.get('copper_mm'))); m['copper_foreign_%s' % h] = math.log1p(_f(e.get('copper_foreign_mm')))
        m['vias_%s' % h] = _f(e.get('vias'))
    m['conflict_at_part'] = _f(ut.get('conflict_nets_at_part'))
    ka = ut.get('conflict_spacing_mm')
    m['conflict_close'] = 1.0 / (1.0 + _f(ka)) if ka is not None else 0.0

    N = b.get('affected_nets') or []
    m['nets_n'] = float(len(N))
    m['net_pads_max_log'] = math.log1p(max([_f(n.get('pads')) for n in N] or [0.0]))
    m['net_gnd'] = 1.0 if any(n.get('gnd') for n in N) else 0.0
    for a in NET_KINDS: m['net_' + a] = 1.0 if any(n.get('category') == a for n in N) else 0.0
    m['net_wide'] = max([_f(n.get('w')) for n in N] or [0.0])

    sc = b.get('score') or {}
    m['start_valid'] = 1.0 if sc.get('is_valid') else 0.0
    m['start_conflicts_log'] = math.log1p(_f(sc.get('conflict_list')))
    m['start_detour_log'] = math.log1p(max(_f(sc.get('detour_mm')), 0.0))
    m['start_vias_log'] = math.log1p(_f(sc.get('vias')))

    m['gain_x_shift'] = m['gain_log'] * m['kind_shift']
    m['gain_x_rotate'] = m['gain_log'] * m['kind_rotate']
    return m


NAMES = list(feats({}).keys())

SMALL = ['kind_shift', 'kind_rotate', 'kind_swap', 'kind_offset', 'gain_log', 'rank_log', 'rank_first', 'gain_rel', 'way_log', 'rotate_180', 'ww_whole', 'price_rel',
         'function_pull', 'function_decoupling', 'function_sensitive', 'occupied_2', 'copper_foreign_1.5', 'vias_1.5',
         'net_pads_max_log', 'nets_n', 'start_detour_log']
SETS = dict(all_items=NAMES, small=SMALL)


def vector(before, rank=None, gain_max=None):
    m = feats(before, rank, gain_max)
    return [m[n] for n in NAMES]

def _gs(z): return _f(((z.get('before') or {}).get('reason') or {}).get('gain_estimate'))


def context(Z):
    G = collections.defaultdict(list)
    for i, z in enumerate(Z):
        ids = z.get('ids') or {}; G[(ids.get('run_id'), ids.get('decision_id'))].append(i)
    out = [None] * len(Z)
    for idx in G.values():
        idx = sorted(idx, key=lambda i: (Z[i]['ids'].get('candidate_index') or 0))
        gm = max(_gs(Z[i]) for i in idx)
        for r, i in enumerate(idx): out[i] = (r, gm)
    return out


def matrix(Z):
    import numpy as np
    return np.array([vector(z.get('before'), *c) for z, c in zip(Z, context(Z))], dtype=float)



def _paths(fpath):
    return [p for p in (fpath.split(',') if isinstance(fpath, str) else list(fpath)) if p]


def read_in(fpath=READ):
    Z = []
    for line in (l for p in _paths(fpath) if os.path.exists(p) for l in open(p)):
        try: z = json.loads(line)
        except ValueError: continue
        if z.get('schema_version') != 'attempt.v1': continue
        st = (z.get('result') or {}).get('status')
        if st not in ('evaluated', 'router_error') or (z.get('decision') or {}).get('phase') != 'primary': continue
        Z.append(z)
    return Z


def goal(z):
    return 1 if (z.get('label') or {}).get('success') is True else 0


def gain_eff(z):
    g = (z.get('label') or {}).get('gain_if_valid')
    return max(_f(g) - _f((z.get('before') or {}).get('price')), 0.0) if g is not None else 0.0


def family(z): return (z.get('ids') or {}).get('board_family_id') or 'unknown'

def group_list(Z):
    G = collections.defaultdict(list)
    for i, z in enumerate(Z):
        ids = z.get('ids') or {}
        if (z.get('decision') or {}).get('phase') != 'primary': continue
        G[(ids.get('run_id'), ids.get('decision_id'))].append(i)
    return {g: sorted(v, key=lambda i: (Z[i]['ids'].get('candidate_index') or 0)) for g, v in G.items() if len(v) >= 2}



def _fit(X, y, l2=1.0, it=50):
    import numpy as np
    mu = X.mean(0); sd = X.std(0); sd[sd < 1e-9] = 1.0
    Xs = np.hstack([(X - mu) / sd, np.ones((len(X), 1))])
    w = np.zeros(Xs.shape[1]); P = np.eye(Xs.shape[1]) * l2; P[-1, -1] = 1e-6
    for _ in range(it):
        p = 1 / (1 + np.exp(-np.clip(Xs @ w, -30, 30)))
        g = Xs.T @ (p - y) + P @ w
        H = (Xs * (p * (1 - p))[:, None]).T @ Xs + P
        dw = np.linalg.solve(H, g); w -= dw
        if np.abs(dw).max() < 1e-8: break
    return dict(medium=mu.tolist(), scatter=sd.tolist(), weights=w[:-1].tolist(), axis=float(w[-1]))


def _fit_cl(X, y, grp, l2=10.0):
    import numpy as np
    from scipy.optimize import minimize
    mu = X.mean(0); sd = X.std(0); sd[sd < 1e-9] = 1.0; Xs = (X - mu) / sd
    G = collections.defaultdict(list)
    for i, g in enumerate(grp): G[g].append(i)
    G = [np.array(v) for v in G.values() if 0 < y[v].sum() < len(v)]
    if not G: return dict(medium=mu.tolist(), scatter=sd.tolist(), weights=[0.0] * X.shape[1], axis=0.0)
    def f(w):
        L = 0.5 * l2 * w @ w; g = l2 * w
        for v in G:
            s = Xs[v] @ w; m = s.max(); e = np.exp(s - m); p = e / e.sum()
            pos = y[v] > 0; ep = e[pos]; q = ep / ep.sum()
            L -= np.log(ep.sum()) - np.log(e.sum())
            g -= Xs[v][pos].T @ q - Xs[v].T @ p
        return L, g
    w = minimize(f, np.zeros(X.shape[1]), jac=True, method='L-BFGS-B').x
    return dict(medium=mu.tolist(), scatter=sd.tolist(), weights=w.tolist(), axis=0.0)


def _group_id(z):
    ids = z.get('ids') or {}
    return (ids.get('run_id'), ids.get('decision_id'))


def _logit(M, X):
    import numpy as np
    return ((np.asarray(X) - np.asarray(M['medium'])) / np.asarray(M['scatter'])) @ np.asarray(M['weights']) + M['axis']


def prediction(M, befores):
    import numpy as np
    if not befores: return []
    names = M['feats']
    gm = max(_f(((b or {}).get('reason') or {}).get('gain_estimate')) for b in befores)
    X = [[feats(b, r, gm).get(n, 0.0) for n in names] for r, b in enumerate(befores)]
    return (1 / (1 + np.exp(-np.clip(_logit(M, X), -30, 30)))).tolist()


def load_data(fpath):
    return json.load(open(fpath))



def _chance_hits(n, s, k):
    if s == 0: return 0.0
    if k >= n: return 1.0
    return 1.0 - math.comb(n - s, k) / math.comb(n, k)


def rate_groups(Z, G, amount, k):
    a = dict(decisions=0, with_success=0, model=0.0, today=0.0, chance=0.0, g_model=0.0, g_today=0.0, g_chance=0.0, move_records=0, successes=0,
             large=0, gr_model=0.0, gr_today=0.0, gr_chance=0.0)
    for g, idx in G.items():
        a['decisions'] += 1; a['move_records'] += len(idx)
        s = sum(goal(Z[i]) for i in idx); a['successes'] += s
        if s == 0: continue
        a['with_success'] += 1
        today = idx[:k]
        model = sorted(idx, key=lambda i: -amount[i])[:k]
        a['model'] += 1.0 if any(goal(Z[i]) for i in model) else 0.0
        a['today'] += 1.0 if any(goal(Z[i]) for i in today) else 0.0
        a['chance'] += _chance_hits(len(idx), s, k)
        if len(idx) > k:
            a['large'] += 1; a['gr_model'] += 1.0 if any(goal(Z[i]) for i in model) else 0.0
            a['gr_today'] += 1.0 if any(goal(Z[i]) for i in today) else 0.0; a['gr_chance'] += _chance_hits(len(idx), s, k)
        best = max(gain_eff(Z[i]) for i in idx)
        if best > 0:
            a['g_model'] += max(gain_eff(Z[i]) for i in model) / best
            a['g_today'] += max(gain_eff(Z[i]) for i in today) / best
            ge = sorted((gain_eff(Z[i]) for i in idx), reverse=True); n = len(idx); kk = min(k, n)
            a['g_chance'] += sum(ge[j] * math.comb(n - 1 - j, kk - 1) for j in range(n - kk + 1)) / math.comb(n, kk) / best
    return a


def pair_comparison(Z, G, amount, k, n_boot=2000, seed=0):
    per = collections.defaultdict(list)
    for g, idx in G.items():
        if len(idx) <= k or not any(goal(Z[i]) for i in idx): continue
        m = 1.0 if any(goal(Z[i]) for i in sorted(idx, key=lambda i: -amount[i])[:k]) else 0.0
        h = 1.0 if any(goal(Z[i]) for i in idx[:k]) else 0.0
        per[family(Z[idx[0]])].append(m - h)
    d = [x for v in per.values() for x in v]
    if not d: return None
    rnd = random.Random(seed); fams = sorted(per); boot = []
    for _ in range(n_boot):
        w = [x for f in (rnd.choice(fams) for _ in fams) for x in per[f]]
        boot.append(sum(w) / len(w))
    boot.sort()
    return dict(decisions=len(d), families=len(fams), siege=sum(1 for x in d if x > 0), defeats=sum(1 for x in d if x < 0),
                difference=round(sum(d) / len(d), 3), range95=[round(boot[int(0.025 * n_boot)], 3), round(boot[int(0.975 * n_boot) - 1], 3)])


def _quotas(a):
    n = a['with_success'] or 1
    return dict(decisions=a['decisions'], with_success=a['with_success'], move_records=a['move_records'], successes=a['successes'],
                model=round(a['model'] / n, 3), today=round(a['today'] / n, 3), chance=round(a['chance'] / n, 3),
                gain_share_model=round(a['g_model'] / n, 3), gain_share_today=round(a['g_today'] / n, 3),
                gain_share_chance=round(a['g_chance'] / n, 3), large=a['large'],
                large_model=round(a['gr_model'] / (a['large'] or 1), 3), large_today=round(a['gr_today'] / (a['large'] or 1), 3),
                large_chance=round(a['gr_chance'] / (a['large'] or 1), 3))


def _auc(y, p):
    pos = [q for t, q in zip(y, p) if t]; neg = [q for t, q in zip(y, p) if not t]
    if not pos or not neg: return None
    s = sum((1.0 if a > b else 0.5 if a == b else 0.0) for a in pos for b in neg)
    return round(s / (len(pos) * len(neg)), 3)


def predictions_cv(Z, l2=1.0, category='lin', dataset='all_items'):
    import numpy as np
    fam = sorted({family(z) for z in Z})
    X = matrix(Z); y = np.array([goal(z) for z in Z], dtype=float)
    X = X[:, [NAMES.index(n) for n in SETS[dataset]]]
    amount = np.zeros(len(Z)); with_ = []
    for f in fam:
        te = [i for i, z in enumerate(Z) if family(z) == f]; tr = [i for i, z in enumerate(Z) if family(z) != f]
        if not tr or y[tr].sum() == 0: continue
        if category == 'cl':
            amount[te] = _logit(_fit_cl(X[tr], y[tr], [_group_id(Z[i]) for i in tr], l2), X[te])
        elif category == 'hgb':
            from sklearn.ensemble import HistGradientBoostingClassifier
            h = HistGradientBoostingClassifier(max_depth=3, max_iter=150, learning_rate=0.05, min_samples_leaf=10, random_state=0)
            amount[te] = h.fit(X[tr], y[tr]).predict_proba(X[te])[:, 1]
        else:
            amount[te] = _logit(_fit(X[tr], y[tr], l2), X[te])
        with_.append(f)
    return amount, with_


def measure(Z, k=4, l2=1.0, category='lin', cv=None, dataset='all_items'):
    import numpy as np
    fam = sorted({family(z) for z in Z}); y = np.array([goal(z) for z in Z], dtype=float)
    amount, with_ = cv or predictions_cv(Z, l2, category, dataset); G = group_list(Z)
    per = {}
    for f in with_:
        Gf = {g: v for g, v in G.items() if family(Z[v[0]]) == f}
        per[f] = _quotas(rate_groups(Z, Gf, amount, k))
    total = _quotas(rate_groups(Z, G, amount, k)); total['pair'] = pair_comparison(Z, {g: v for g, v in G.items() if family(Z[v[0]]) in with_}, amount, k)
    today_value = np.array([_f(((z.get('before') or {}).get('reason') or {}).get('gain_estimate')) for z in Z])
    return dict(k=k, l2=l2, category=category, dataset=dataset, lines=len(Z), successes=int(y.sum()), families=len(fam), total=total, per_family=per,
                auc_model=_auc(y.tolist(), amount.tolist()), auc_today=_auc(y.tolist(), today_value.tolist()),
                families_model_ahead=sum(1 for v in per.values() if v['with_success'] and v['model'] > v['today']),
                families_today_ahead=sum(1 for v in per.values() if v['with_success'] and v['model'] < v['today']))


def model_id(M):
    fields = ('feats', 'medium', 'scatter', 'weights', 'axis', 'families', 'ranker_k', 'data_sha')
    return 'move_records-' + hashlib.sha256(json.dumps({k: M.get(k) for k in fields}, sort_keys=True).encode()).hexdigest()[:16]


def prospective(M, Z, k=4):
    import numpy as np
    prior = set(M.get('families') or [])
    new = [i for i, z in enumerate(Z) if family(z) not in prior]
    if not prior or not new: return []
    X = matrix(Z)[:, [NAMES.index(n) for n in M['feats']]]
    amount = np.zeros(len(Z)); amount[new] = _logit(M, X[new])
    out = []
    for g, idx in sorted(group_list(Z).items(), key=lambda kv: str(kv[0])):
        if family(Z[idx[0]]) in prior or len(idx) <= k or not any(goal(Z[i]) for i in idx): continue
        m = 1.0 if any(goal(Z[i]) for i in sorted(idx, key=lambda i: -amount[i])[:k]) else 0.0
        h = 1.0 if any(goal(Z[i]) for i in idx[:k]) else 0.0
        out.append(dict(decision='%s/%s' % g, family=family(Z[idx[0]]), model_id=M.get('id'), k=k, model=m, today=h, difference=m - h))
    return out


def prospective_state(lines, n_boot=2000, seed=0, model_id=None):
    if model_id is not None: lines = [z for z in lines if z.get('model_id') == model_id]
    per = collections.defaultdict(list)
    for z in lines: per[z['family']].append(z['difference'])
    d = [x for v in per.values() for x in v]
    if not d: return dict(decisions=0, model_id=model_id)
    rnd = random.Random(seed); fams = sorted(per); boot = []
    for _ in range(n_boot):
        w = [x for f in (rnd.choice(fams) for _ in fams) for x in per[f]]
        boot.append(sum(w) / len(w))
    boot.sort()
    return dict(model_id=model_id, decisions=len(d), families=len(fams), siege=sum(1 for x in d if x > 0), defeats=sum(1 for x in d if x < 0),
                difference=round(sum(d) / len(d), 3), range95=[round(boot[int(0.025 * n_boot)], 3), round(boot[int(0.975 * n_boot) - 1], 3)])


def better_than_today(meas, min_decisions=20, min_families=4, prosp=None):
    g = meas['total']
    fam_with = sum(1 for v in meas['per_family'].values() if v['with_success'])
    if g['with_success'] < min_decisions or fam_with < min_families: return False, 'too little data (%d decisions with success, %d families)' % (g['with_success'], fam_with)
    pa = g.get('pair') or {}
    if g['model'] > g['today'] and g['gain_share_model'] >= g['gain_share_today'] and (pa.get('range95') or [0])[0] > 0:
        if prosp is not None:
            if not prosp.get('model_id'): return False, 'look-ahead supports no single model'
            if prosp.get('families', 0) < min_families: return False, 'look-ahead too few new families'
            if prosp.get('decisions', 0) < min_decisions:
                return False, 'ahead by family split, look-ahead still too little (%d decisions)' % prosp.get('decisions', 0)
            if prosp['range95'][0] <= 0:
                return False, 'ahead by family split, look-ahead not safe (difference %+.3f, 95 %% %s)' % (prosp['difference'], prosp['range95'])
        return True, 'Model ahead (difference %+.3f, 95 %% %s%s)' % (pa['difference'], pa['range95'],
                                                                 ', look-ahead %s' % prosp['range95'] if prosp else '')
    return False, 'Model not safely ahead of the current ordering (%.3f vs %.3f, 95 %% of the difference %s)' % (
        g['model'], g['today'], pa.get('range95'))


def train(data=READ, model=None, k=4, l2=10.0, write_out=True, dataset='all_items', category='cl'):
    import numpy as np
    Z = read_in(data)
    if not Z or sum(goal(z) for z in Z) == 0: return None
    meas = measure(Z, k, l2, category=category, dataset=dataset)
    X = matrix(Z); y = np.array([goal(z) for z in Z], dtype=float)
    X = X[:, [NAMES.index(n) for n in SETS[dataset]]]
    fit = _fit_cl(X, y, [_group_id(z) for z in Z], l2) if category == 'cl' else _fit(X, y, l2)
    M = dict(version='move_records-v1', category='conditional-logit-l2' if category == 'cl' else 'logistic-l2', dataset=dataset, l2=l2, feats=SETS[dataset], **fit,
             ranker_k=k, measurement=meas,
             data=[os.path.abspath(p) for p in _paths(data)],
             data_sha=hashlib.sha256(b''.join(open(p, 'rb').read() for p in _paths(data) if os.path.exists(p))).hexdigest()[:16],
             created=time.strftime('%Y-%m-%d %H:%M'))
    M['families'] = sorted({family(z) for z in Z})
    M['id'] = model_id(M)
    if write_out and model:
        os.makedirs(os.path.dirname(os.path.abspath(model)), exist_ok=True)
        json.dump(M, open(model, 'w'), indent=1)
    return M



def append(fpath, lines):
    os.makedirs(os.path.dirname(os.path.abspath(fpath)), exist_ok=True)
    with open(fpath, 'a') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            for z in lines: f.write(json.dumps(z, ensure_ascii=False, allow_nan=False) + '\n')
            f.flush(); os.fsync(f.fileno())
        finally: fcntl.flock(f, fcntl.LOCK_UN)


def workdir(c, design_rules_from=None, goal_d=None):
    goal_d = goal_d or os.path.join(c, 'move_records')
    os.makedirs(goal_d, exist_ok=True)
    for f in ('board.json', 'routes.json', 'netclasses.json'):
        shutil.copy(os.path.join(c, f), os.path.join(goal_d, f))
    rg = os.path.join(c, 'design_rules.json')
    if not os.path.exists(rg): rg = os.path.join(design_rules_from or os.path.dirname(c.rstrip('/')), 'design_rules.json')
    shutil.copy(rg, os.path.join(goal_d, 'design_rules.json'))
    for f in ('improve_attempts.jsonl', 'improve.log'):
        p = os.path.join(goal_d, f)
        if os.path.exists(p): os.remove(p)
    return goal_d


def collect(c, design_rules_from=None, family_=None, data=DATA, cpu_s=900, rounds_n=2, cand=12, iter_=10, parallel=2,
            move_records='shift,rotate,swap,offset', ir='1', ranker=None, with_slot=True, keep=False, extra=()):
    from common import slots as PL
    zd = workdir(c, design_rules_from)
    fam = family_ or os.path.basename(os.path.dirname(c.rstrip('/')))
    cmd = [VPY, os.path.join(R, 'postprocessing', 'improve.py'), zd, 'rounds_n=%d' % rounds_n, 'cand=%d' % cand,
           'iter=%s' % iter_, 'parallel=%d' % parallel, 'move_records=' + move_records, 'ir=' + str(ir), 'family=' + fam,
           'budget_cpu=%d' % int(0.7 * cpu_s)] + list(extra)
    if ranker: cmd.append('ranker=' + ranker)
    r = PL.job(cmd, cpu_s=cpu_s, wall_s=6 * cpu_s, log=os.path.join(zd, 'improve.out'),
                env=dict(os.environ, PYTHONHASHSEED='0'), with_slot=with_slot)
    Z = []
    p = os.path.join(zd, 'improve_attempts.jsonl')
    if os.path.exists(p):
        for line in open(p):
            try: z = json.loads(line)
            except ValueError: continue
            z['collection'] = dict(source='learning/moves.py', candidate=os.path.abspath(c), cur_state=r['cur_state'], cpu_s=r['cpu_s'])
            Z.append(z)
    if Z: append(data, Z)
    if not keep: shutil.rmtree(os.path.join(zd, 'improve'), ignore_errors=True)
    return dict(lines=len(Z), successes=sum(1 for z in Z if (z.get('label') or {}).get('success')), cur_state=r['cur_state'],
                cpu_s=r['cpu_s'], wall_s=r['wall_s'])


def _pads(d):
    try: return open(os.path.join(d, 'processed.kicad_pcb'), errors='replace').read().count('(pad ')
    except OSError: return 0


def generate(tr, out, kw):
    from concurrent.futures import ThreadPoolExecutor
    from learning import collect as S
    from learning import endurance_run as DL
    from analysis import corpus as KO
    n = int(kw.get('n', '20')); NET_COPPER_ = int(kw.get('candidates', '1')); P = int(kw.get('parallel', '6'))
    time_s = int(kw.get('time_s', '240')); it = int(kw.get('iter', '40')); maxp = int(kw.get('max_pads', '150'))
    data = os.path.abspath(kw.get('data', DATA)); keep = kw.get('keep', '0') == '1'; tight = kw.get('tight', '0') == '1'
    rnd = random.Random(int(kw.get('seed', '1')))
    S.V2['an'] = True; KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % time_s,)
    if kw.get('boards'): dirs = [os.path.join(tr, b) for b in kw['boards'].split(',')]
    else:
        already = {family(z) for z in read_in([data, START])}
        for pj in glob.glob(os.path.join(out, '*.jsonl')) + glob.glob(os.path.join(os.path.dirname(out.rstrip('/')), '*', 'generate.jsonl')):
            for line in open(pj):
                try: already.add(json.loads(line).get('board'))
                except ValueError: pass
        already |= {os.path.basename(x.rstrip('/')) for x in glob.glob(os.path.join(os.path.dirname(out.rstrip('/')), '*', '*', ''))}
        dirs = [d for d in sorted(glob.glob(os.path.join(tr, '*'))) if os.path.isdir(d) and os.path.basename(d) not in already and DL.board_ok(d, maxp)[0]
                and _pads(d) >= int(kw.get('min_pads', '0'))]
        rnd.shuffle(dirs); dirs = dirs[:n]
    os.makedirs(out, exist_ok=True)
    proto = os.path.join(out, 'generate.jsonl')
    sk = dict(cpu_s=int(kw.get('cpu', '900')), rounds_n=int(kw.get('rounds_n', '2')), cand=int(kw.get('cand', '12')),
              iter_=kw.get('nb_iter', '10'), parallel=int(kw.get('nb_parallel', '2')), ir=kw.get('ir', '1'),
              move_records=kw.get('move_records', 'shift,rotate,swap,offset'), keep=keep)

    def board(d):
        name = os.path.basename(d); o = os.path.join(out, name); t0 = time.time(); erg = []
        try:
            info = S.prepare(d, o)
            if not info or info['features_human']['n_pads'] > maxp:
                erg.append(dict(board=name, cur_state='unsuitable')); return erg
            info['family'] = name
            states = [(d, o, info, k, '') for k in range(NET_COPPER_)]
            if tight:
                q, f = S.variant_tight(d, o, random.Random('%s-%s-tight' % (kw.get('seed', '1'), name)))
                if q:
                    o2 = o + '_tight'; info2 = S.prepare(q, o2)
                    if info2:
                        info2['family'] = name; info2['variant'] = dict(category='tight', factor=f)
                        states.append((q, o2, info2, 0, 'tight'))
            for dq, oq, iq, k, var in states:
                cfg = S.chance_cfg(random.Random('%s-%s-%d%s' % (kw.get('seed', '1'), name, k, var)))
                z = S.candidate(dq, oq, iq, k, cfg, it, time_s, False)
                g = bool((z.get('outcome') or {}).get('is_valid'))
                e = dict(board=name, variant=var or None, candidate=k, is_valid=g, cur_state=z.get('cur_state'), s=z.get('s'))
                if g: e['collect'] = collect(os.path.join(oq, 'k%d' % k), oq, family_=name, data=data, **sk)
                erg.append(e); print(json.dumps(e), flush=True)
        except Exception as ex:
            erg.append(dict(board=name, faults=repr(ex)[:300]))
        finally:
            for e in erg: e['s_total'] = round(time.time() - t0, 1)
            append(proto, erg)
            if not keep:
                for x in (o, o + '_tight', os.path.join(out, '_sources', name + '_tight')): shutil.rmtree(x, ignore_errors=True)
        return erg

    with ThreadPoolExecutor(P) as ex:
        all_items = [e for es in ex.map(board, dirs) for e in es]
    n_z = sum((e.get('collect') or {}).get('lines', 0) for e in all_items)
    print('generate: %d boards, %d valid states, %d move rows -> %s' % (len(dirs), sum(1 for e in all_items if e.get('is_valid')), n_z, data), flush=True)


def later(Z, fam_prev, ks=(1, 2, 3, 4), l2=10.0, dataset='all_items', category='cl'):
    import numpy as np
    prior = [i for i, z in enumerate(Z) if family(z) in fam_prev]; new = [i for i, z in enumerate(Z) if family(z) not in fam_prev]
    if not prior or not new: return None
    X = matrix(Z)[:, [NAMES.index(n) for n in SETS[dataset]]]; y = np.array([goal(z) for z in Z], dtype=float)
    M = _fit_cl(X[prior], y[prior], [_group_id(Z[i]) for i in prior], l2) if category == 'cl' else _fit(X[prior], y[prior], l2)
    amount = np.zeros(len(Z)); amount[new] = _logit(M, X[new])
    G = {g: v for g, v in group_list(Z).items() if family(Z[v[0]]) not in fam_prev}
    return dict(category=category, dataset=dataset, l2=l2, lines_prev=len(prior), lines_new=len(new), families_new=len({family(Z[i]) for i in new}), successes_new=int(y[new].sum()),
                auc_model=_auc(y[new].tolist(), amount[new].tolist()), auc_today=_auc(y[new].tolist(), [_gs(Z[i]) for i in new]),
                hits={'@%d' % k: dict(_quotas(rate_groups(Z, G, amount, k)), pair=pair_comparison(Z, G, amount, k)) for k in ks})


def report(data=READ, ks=(1, 2, 3, 4), development=None):
    Z = read_in(data); fam = collections.Counter(family(z) for z in Z)
    out = dict(lines=len(Z), successes=sum(goal(z) for z in Z), families=len(fam),
               families_with_success=len({family(z) for z in Z if goal(z)}), decisions=len(group_list(Z)),
               status=dict(collections.Counter(z['result']['status'] for z in Z)),
               start_valid=sum(1 for z in Z if ((z.get('before') or {}).get('score') or {}).get('is_valid')))
    def quote(key):
        c = collections.defaultdict(lambda: [0, 0])
        for z in Z: c[key(z)][0] += 1; c[key(z)][1] += goal(z)
        return {str(a): '%d/%d' % (v[1], v[0]) for a, v in sorted(c.items(), key=lambda kv: str(kv[0]))}
    out['per_move_kind'] = quote(lambda z: z['before']['move']['type'])
    out['per_occasion'] = quote(lambda z: (z['before'].get('reason') or {}).get('type'))
    out['per_function'] = quote(lambda z: function_group(((z['before'].get('part_list') or [{}])[0]).get('function')))
    out['hits'] = {}
    for name, category, l2, dataset in (('lin_all_l2=1', 'lin', 1.0, 'all_items'), ('lin_all_l2=10', 'lin', 10.0, 'all_items'),
                                ('lin_small_l2=10', 'lin', 10.0, 'small'), ('lin_small_l2=100', 'lin', 100.0, 'small'), ('hgb_all', 'hgb', 1.0, 'all_items'),
                                ('cl_small_l2=1', 'cl', 1.0, 'small'), ('cl_small_l2=10', 'cl', 10.0, 'small'), ('cl_all_l2=10', 'cl', 10.0, 'all_items')):
        cv = predictions_cv(Z, l2, category, dataset)
        for k in ks:
            try: m = measure(Z, k, l2, category=category, cv=cv, dataset=dataset)
            except Exception as e: out['hits']['%s@%d' % (name, k)] = repr(e)[:100]; continue
            g = m['total']
            out['hits']['%s@%d' % (name, k)] = dict(model=g['model'], today=g['today'], chance=g['chance'],
                                                     gain_model=g['gain_share_model'], gain_today=g['gain_share_today'],
                                                     gain_chance=g['gain_share_chance'], with_success=g['with_success'],
                                                     decisions=g['decisions'], auc=m['auc_model'], auc_today=m['auc_today'],
                                                     fam_ahead=m['families_model_ahead'], fam_back=m['families_today_ahead'],
                                                     large=g['large'], large_model=g['large_model'], large_today=g['large_today'], large_chance=g['large_chance'],
                                                     pair=g.get('pair'))
    if development:
        out['later'] = later(Z, set(json.load(open(development))['families']), ks)
    return out


def _score(log, what):
    import re
    for line in reversed(open(log).read().splitlines()) if os.path.exists(log) else []:
        m = re.match(r'%s \((v1|v2)\): Score ([-0-9.]+).*?(valid|INVALID)' % what, line)
        if m: return float(m.group(2)), m.group(3) == 'is_valid'
    return None, None


def comparison(tr, out, kw):
    from concurrent.futures import ThreadPoolExecutor
    from learning import collect as S
    from learning import endurance_run as DL
    from analysis import corpus as KO
    from common import slots as PL
    k = int(kw.get('k', '4')); model = os.path.abspath(kw['ranker']); P = int(kw.get('parallel', '4'))
    time_s = int(kw.get('time_s', '240')); it = int(kw.get('iter', '40')); maxp = int(kw.get('max_pads', '220'))
    S.V2['an'] = True; KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % time_s,)
    avoid = set(json.load(open(kw['development']))['families']) if kw.get('development') else set()
    dirs = [os.path.join(tr, b) for b in kw['boards'].split(',')] if kw.get('boards') else []
    if not dirs:
        all_items = [d for d in sorted(glob.glob(os.path.join(tr, '*'))) if os.path.isdir(d) and os.path.basename(d) not in avoid
                and DL.board_ok(d, maxp)[0] and _pads(d) >= int(kw.get('min_pads', '70'))]
        random.Random(int(kw.get('seed', '5'))).shuffle(all_items); dirs = all_items[:int(kw.get('n', '8'))]
    os.makedirs(out, exist_ok=True); proto = os.path.join(out, 'comparison.jsonl')
    variants = [('entire', ['cand=12']), ('today_%d' % k, ['cand=%d' % k]), ('ranker_%d' % k, ['cand=12', 'ranker=' + model, 'ranker_k=%d' % k])]

    def board(d):
        name = os.path.basename(d); o = os.path.join(out, name)
        try:
            info = S.prepare(d, o)
            if not info: return
            info['family'] = name
            z = S.candidate(d, o, info, 0, S.chance_cfg(random.Random('v-%s' % name)), it, time_s, False)
            if not (z.get('outcome') or {}).get('is_valid'):
                append(proto, [dict(board=name, is_valid=False)]); return
            c = os.path.join(o, 'k0'); erg = dict(board=name, is_valid=True)
            for vn, ex in variants:
                zd = workdir(c, o, os.path.join(c, vn))
                r = PL.job([VPY, os.path.join(R, 'postprocessing', 'improve.py'), zd, 'rounds_n=%s' % kw.get('rounds_n', '2'),
                             'iter=10', 'parallel=2', 'move_records=shift,rotate,swap,offset', 'family=' + name] + ex,
                            cpu_s=int(kw.get('cpu', '900')), log=os.path.join(zd, 'improve.out'), env=dict(os.environ, PYTHONHASHSEED='0'))
                n0, g0 = _score(os.path.join(zd, 'improve.log'), 'Basis'); n1, g1 = _score(os.path.join(zd, 'improve.log'), 'Result')
                vs = os.path.join(zd, 'improve_attempts.jsonl')
                erg[vn] = dict(score0=n0, score1=n1, valid1=g1, attempts=sum(1 for _ in open(vs)) if os.path.exists(vs) else 0,
                               cpu_s=r['cpu_s'], cur_state=r['cur_state'])
            append(proto, [erg]); print(json.dumps(erg), flush=True)
        except Exception as e:
            append(proto, [dict(board=name, faults=repr(e)[:300])])
        finally:
            if kw.get('keep', '0') != '1': shutil.rmtree(o, ignore_errors=True)

    with ThreadPoolExecutor(P) as ex: list(ex.map(board, dirs))


def main():
    if len(sys.argv) < 2: print(__doc__); return
    cmd = sys.argv[1]
    pos = [a for a in sys.argv[2:] if '=' not in a]; kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    if cmd == 'generate': generate(pos[0], pos[1], kw)
    elif cmd == 'comparison': comparison(pos[0], pos[1], kw)
    elif cmd == 'collect':
        print(json.dumps(collect(os.path.abspath(pos[0]), kw.get('design_rules_from'), kw.get('family'), os.path.abspath(kw.get('data', DATA)),
                                 int(kw.get('cpu', '900')), int(kw.get('rounds_n', '2')), int(kw.get('cand', '12')), kw.get('nb_iter', '10'),
                                 int(kw.get('nb_parallel', '2')), kw.get('move_records', 'shift,rotate,swap,offset'), kw.get('ir', '1'),
                                 kw.get('ranker'))))
    elif cmd == 'report': print(json.dumps(report(kw.get('data', READ), development=kw.get('development')), indent=1, ensure_ascii=False))
    elif cmd in ('train', 'measure'):
        M = train(kw.get('data', READ), kw.get('model', MODEL), int(kw.get('k', '4')), float(kw.get('l2', '10')),
                       write_out=cmd == 'train' and kw.get('write_out', '1') == '1', dataset=kw.get('dataset', 'all_items'), category=kw.get('category', 'cl'))
        if M is None: print('no data with success'); return
        m = M['measurement']; ok, reason = better_than_today(m)
        print(json.dumps(dict(id=M['id'], lines=m['lines'], successes=m['successes'], families=m['families'], total=m['total'],
                              auc_model=m['auc_model'], auc_today=m['auc_today'], families_model_ahead=m['families_model_ahead'],
                              families_today_ahead=m['families_today_ahead'], accept=ok, reason=reason), indent=1))
    else: print(__doc__)


if __name__ == '__main__':
    main()
