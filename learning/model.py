import json, os, sys, math, collections, random
os.environ.setdefault('OMP_NUM_THREADS', '2')
import numpy as np

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FEATURES = ['cu_layers', 'clr', 'track', 'surface', 'n_parts', 'n_pads', 'n_nets', 'pads_per_cm2', 'occupancy', 'mst', 'mst_gnd', 'hpwl',
            'mst_per_area', 'mst_rel_diag', 'longest_edge', 'crossings', 'cross_per_net', 'double_sided', 'rudy_max', 'rudy_p95',
            'rudy_medium', 'rudy_over', 'pads_max_4mm2', 'border_pads', 'share_current', 'via_f']
STANDARD = dict(via_f=1.0)


def load_data(fpath):
    from learning.early_abort import aborted
    Z = []
    for z in open(fpath):
        try: j = json.loads(z)
        except Exception: continue
        if not j.get('feats') or aborted(j): continue
        Z.append(j)
    return Z


def X_from(m): return np.array([float(m.get(k, STANDARD.get(k, 0.0)) or 0.0) for k in FEATURES], float)


def goal(z):
    return bool(z.get('is_valid')), (z.get('cost') if z.get('outcome') else None)


def rate_selection(Z, choice):
    pro = collections.defaultdict(list)
    for z in Z: pro[z['board']].append(z)
    w = []
    for b, ks in pro.items():
        k = choice(ks); g, c = goal(k); m = k.get('human') or {}
        mk = (m.get('length_mm') or 0) + 3 * (m.get('vias') or 0)
        w.append(min(2.0, c / mk) if g and c and mk > 0 else 2.0)
    return float(np.mean(w)) if w else None, len(w)


def train(Z):
    from sklearn.ensemble import HistGradientBoostingClassifier, HistGradientBoostingRegressor
    X = np.array([X_from(z['feats']) for z in Z]); yv = np.array([goal(z)[0] for z in Z], int)
    mc = np.array([goal(z)[1] is not None for z in Z]); yc = np.array([math.log(max(1.0, goal(z)[1] or 1.0)) for z in Z])
    clf = HistGradientBoostingClassifier(max_iter=200, learning_rate=0.05, max_leaf_nodes=15, min_samples_leaf=8, random_state=0)
    reg = HistGradientBoostingRegressor(max_iter=200, learning_rate=0.05, max_leaf_nodes=15, min_samples_leaf=8, random_state=0)
    if len(set(yv)) > 1: clf.fit(X, yv)
    else: clf = float(yv.mean())
    reg.fit(X[mc], yc[mc])
    return clf, reg


def relative(Z):
    pro = collections.defaultdict(list)
    for i, z in enumerate(Z): pro[z['board']].append(i)
    X = np.array([X_from(z['feats']) for z in Z]); Xr = np.zeros_like(X)
    yc = np.full(len(Z), np.nan)
    for b, idx in pro.items():
        Xr[idx] = X[idx] - X[idx].mean(0)
        lc = [math.log(goal(Z[i])[1]) if goal(Z[i])[1] else None for i in idx]
        ok = [v for v in lc if v is not None]
        if ok:
            m = float(np.mean(ok))
            for i, v in zip(idx, lc):
                if v is not None: yc[i] = v - m
    return Xr, np.array([goal(z)[0] for z in Z], int), yc


def train_rel(Z):
    from sklearn.ensemble import HistGradientBoostingClassifier, HistGradientBoostingRegressor
    Xr, yv, yc = relative(Z); mc = ~np.isnan(yc)
    clf = HistGradientBoostingClassifier(max_iter=150, learning_rate=0.05, max_leaf_nodes=7, min_samples_leaf=10, random_state=0)
    reg = HistGradientBoostingRegressor(max_iter=150, learning_rate=0.05, max_leaf_nodes=7, min_samples_leaf=10, random_state=0)
    clf.fit(Xr, yv) if len(set(yv)) > 1 else None
    reg.fit(Xr[mc], yc[mc])
    return ('rel', clf, reg)


def prediction_rel(mod, ms):
    _, clf, reg = mod; X = np.array([X_from(m) for m in ms]); Xr = X - X.mean(0)
    P = clf.predict_proba(Xr)[:, 1] if hasattr(clf, 'classes_') else np.full(len(ms), 0.5)
    return P, np.exp(reg.predict(Xr))


def prediction(mod, ms):
    if isinstance(mod, tuple) and len(mod) == 3 and mod[0] == 'rel': return prediction_rel(mod, ms)
    clf, reg = mod; X = np.array([X_from(m) for m in ms])
    P = clf.predict_proba(X)[:, 1] if hasattr(clf, 'predict_proba') else np.full(len(ms), float(clf))
    C = np.exp(reg.predict(X))
    return P, C


def choice_model(mod):
    def w(ks):
        P, C = prediction(mod, [k['feats'] for k in ks]); med = float(np.median(C))
        E = P * C + (1 - P) * 2 * med
        return ks[int(np.argmin(E))]
    return w


def choice_heuristic(ks):
    return min(ks, key=lambda k: k['feats']['mst'] * (1 + 0.02 * k['feats']['crossings']) * (1 + max(0.0, k['feats']['rudy_max'] - 1)))


def design_family(z):
    from learning.families import family
    return family(z)


def cross_validation(Z, fold=5, seed=0, split='family'):
    group = design_family if split == 'family' else (lambda z: z['board'])
    boards = sorted({group(z) for z in Z}); rnd = random.Random(seed); rnd.shuffle(boards)
    part = {b: i % fold for i, b in enumerate(boards)}
    erg = collections.defaultdict(list); n = 0
    for f in range(fold):
        tr = [z for z in Z if part[group(z)] != f]; te = [z for z in Z if part[group(z)] == f]
        if not te or len({goal(z)[0] for z in tr}) < 2: continue
        mod = train(tr); modr = train_rel(tr)
        def top2r(ks, mod=modr):
            P, C = prediction(mod, [k['feats'] for k in ks]); E = P * C + (1 - P) * 2 * float(np.median(C))
            two = [ks[i] for i in np.argsort(E)[:2]]
            return min(two, key=lambda k: (not goal(k)[0], goal(k)[1] or 1e9))
        def top2(ks, mod=mod):
            P, C = prediction(mod, [k['feats'] for k in ks]); med = float(np.median(C)); E = P * C + (1 - P) * 2 * med
            two = [ks[i] for i in np.argsort(E)[:2]]
            return min(two, key=lambda k: (not goal(k)[0], goal(k)[1] or 1e9))
        def heuristic2(ks):
            two = sorted(ks, key=lambda k: k['feats']['mst'] * (1 + 0.02 * k['feats']['crossings']) * (1 + max(0.0, k['feats']['rudy_max'] - 1)))[:2]
            return min(two, key=lambda k: (not goal(k)[0], goal(k)[1] or 1e9))
        def chance2(ks):
            two = sorted(ks, key=lambda k: k['candidate'])[:2]
            return min(two, key=lambda k: (not goal(k)[0], goal(k)[1] or 1e9))
        for name, w in (('model', choice_model(mod)), ('model_top2', top2), ('relative', choice_model(modr)), ('relative_top2', top2r), ('heuristic_top2', heuristic2), ('first2', chance2),
                        ('rule_of_thumb', choice_heuristic), ('first', lambda ks: min(ks, key=lambda k: k['candidate'])),
                        ('oracle', lambda ks: min(ks, key=lambda k: (not goal(k)[0], goal(k)[1] or 1e9)))):
            v, m = rate_selection(te, w); erg[name].append((v, m))
        n += 1
    return {k: round(sum(v * m for v, m in L) / max(1, sum(m for _, m in L)), 4) for k, L in erg.items()}, n


def settings(Z):
    pro = collections.defaultdict(list)
    for z in Z: pro[z['board']].append(z)
    best = {b: min([goal(k)[1] for k in ks if goal(k)[0] and goal(k)[1]] or [None], key=lambda x: x or 1e18) for b, ks in pro.items()}
    out = {}
    for key in ('cross', 'separate', 'via_cost', 'via_f'):
        d = collections.defaultdict(lambda: [0, 0, []])
        for z in Z:
            v = z['cfg'].get(key); d[v][0] += 1; d[v][1] += bool(goal(z)[0])
            if goal(z)[0] and best[z['board']]: d[v][2].append(goal(z)[1] / best[z['board']])
        out[key] = {str(v): dict(n=a, is_valid=round(g / a, 3), cost_rel=round(float(np.mean(c)), 3) if c else None) for v, (a, g, c) in sorted(d.items(), key=lambda kv: str(kv[0]))}
    return out


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    fpath = kw.get('data', os.path.join(R, 'learning/data/attempts.jsonl'))
    Z = load_data(fpath); print('Attempts %d, boards %d, valid %.0f %%' % (len(Z), len({z['board'] for z in Z}), 100 * np.mean([goal(z)[0] for z in Z]) if Z else 0))
    if sys.argv[1] == 'settings':
        print(json.dumps(settings(Z), indent=1)); return
    split = kw.get('split', 'family')
    cv, n = cross_validation(Z, split=split)
    print('Cross-validation by %s (%d folds), test bench value per selection (smaller is better):' % ('DesignFamilies' if split == 'family' else 'Boards', n), json.dumps(cv))
    import joblib
    off = kw.get('off', os.path.join(R, 'learning/model')); os.makedirs(off, exist_ok=True)
    category = kw.get('category', 'rel')
    mod = train_rel(Z) if category == 'rel' else train(Z)
    joblib.dump(mod, os.path.join(off, 'selection.joblib'))
    json.dump(dict(category=category, feats=FEATURES, attempts=len(Z), boards=len({z['board'] for z in Z}), cross_validation=cv, data=os.path.relpath(fpath, R),
                   settings=settings(Z)), open(os.path.join(off, 'selection.json'), 'w'), indent=1)
    print('Model saved ->', off)


if __name__ == '__main__':
    main()
