""
import json, os, sys, math, collections, random
os.environ.setdefault('OMP_NUM_THREADS', '2')
import numpy as np

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MERKMALE = ['lagen', 'clr', 'bahn', 'flaeche', 'n_teile', 'n_pads', 'n_netze', 'pads_je_cm2', 'belegung', 'mst', 'mst_gnd', 'hpwl',
            'mst_je_flaeche', 'mst_rel_diag', 'laengste_kante', 'kreuzungen', 'kreuz_je_netz', 'beidseitig', 'rudy_max', 'rudy_p95',
            'rudy_mittel', 'rudy_ueber', 'pads_max_4mm2', 'rand_pads', 'anteil_strom', 'via_f']
STANDARD = dict(via_f=1.0)


def laden(pfad):
    from lernen.fruehabbruch import abgebrochen
    Z = []
    for z in open(pfad):
        try: j = json.loads(z)
        except Exception: continue
        if not j.get('merkmale') or abgebrochen(j): continue
        Z.append(j)
    return Z


def X_von(m): return np.array([float(m.get(k, STANDARD.get(k, 0.0)) or 0.0) for k in MERKMALE], float)


def ziel(z):
    ""
    return bool(z.get('gueltig')), (z.get('kosten') if z.get('ergebnis') else None)


def bewerte_auswahl(Z, wahl):
    ""
    pro = collections.defaultdict(list)
    for z in Z: pro[z['board']].append(z)
    w = []
    for b, ks in pro.items():
        k = wahl(ks); g, c = ziel(k); m = k.get('mensch') or {}
        mk = (m.get('laenge_mm') or 0) + 3 * (m.get('vias') or 0)
        w.append(min(2.0, c / mk) if g and c and mk > 0 else 2.0)
    return float(np.mean(w)) if w else None, len(w)


def trainieren(Z):
    from sklearn.ensemble import HistGradientBoostingClassifier, HistGradientBoostingRegressor
    X = np.array([X_von(z['merkmale']) for z in Z]); yv = np.array([ziel(z)[0] for z in Z], int)
    mc = np.array([ziel(z)[1] is not None for z in Z]); yc = np.array([math.log(max(1.0, ziel(z)[1] or 1.0)) for z in Z])
    clf = HistGradientBoostingClassifier(max_iter=200, learning_rate=0.05, max_leaf_nodes=15, min_samples_leaf=8, random_state=0)
    reg = HistGradientBoostingRegressor(max_iter=200, learning_rate=0.05, max_leaf_nodes=15, min_samples_leaf=8, random_state=0)
    if len(set(yv)) > 1: clf.fit(X, yv)
    else: clf = float(yv.mean())
    reg.fit(X[mc], yc[mc])
    return clf, reg


def relativ(Z):
    ""
    pro = collections.defaultdict(list)
    for i, z in enumerate(Z): pro[z['board']].append(i)
    X = np.array([X_von(z['merkmale']) for z in Z]); Xr = np.zeros_like(X)
    yc = np.full(len(Z), np.nan)
    for b, idx in pro.items():
        Xr[idx] = X[idx] - X[idx].mean(0)
        lc = [math.log(ziel(Z[i])[1]) if ziel(Z[i])[1] else None for i in idx]
        ok = [v for v in lc if v is not None]
        if ok:
            m = float(np.mean(ok))
            for i, v in zip(idx, lc):
                if v is not None: yc[i] = v - m
    return Xr, np.array([ziel(z)[0] for z in Z], int), yc


def trainieren_rel(Z):
    from sklearn.ensemble import HistGradientBoostingClassifier, HistGradientBoostingRegressor
    Xr, yv, yc = relativ(Z); mc = ~np.isnan(yc)
    clf = HistGradientBoostingClassifier(max_iter=150, learning_rate=0.05, max_leaf_nodes=7, min_samples_leaf=10, random_state=0)
    reg = HistGradientBoostingRegressor(max_iter=150, learning_rate=0.05, max_leaf_nodes=7, min_samples_leaf=10, random_state=0)
    clf.fit(Xr, yv) if len(set(yv)) > 1 else None
    reg.fit(Xr[mc], yc[mc])
    return ('rel', clf, reg)


def vorhersage_rel(mod, ms):
    _, clf, reg = mod; X = np.array([X_von(m) for m in ms]); Xr = X - X.mean(0)
    P = clf.predict_proba(Xr)[:, 1] if hasattr(clf, 'classes_') else np.full(len(ms), 0.5)
    return P, np.exp(reg.predict(Xr))


def vorhersage(mod, ms):
    if isinstance(mod, tuple) and len(mod) == 3 and mod[0] == 'rel': return vorhersage_rel(mod, ms)
    clf, reg = mod; X = np.array([X_von(m) for m in ms])
    P = clf.predict_proba(X)[:, 1] if hasattr(clf, 'predict_proba') else np.full(len(ms), float(clf))
    C = np.exp(reg.predict(X))
    return P, C


def wahl_modell(mod):
    def w(ks):
        P, C = vorhersage(mod, [k['merkmale'] for k in ks]); med = float(np.median(C))
        E = P * C + (1 - P) * 2 * med
        return ks[int(np.argmin(E))]
    return w


def wahl_faust(ks):
    return min(ks, key=lambda k: k['merkmale']['mst'] * (1 + 0.02 * k['merkmale']['kreuzungen']) * (1 + max(0.0, k['merkmale']['rudy_max'] - 1)))


def design_familie(z):
    ""
    from lernen.familien import familie
    return familie(z)


def kreuzvalidierung(Z, falten=5, seed=0, split='familie'):
    ""
    gruppe = design_familie if split == 'familie' else (lambda z: z['board'])
    boards = sorted({gruppe(z) for z in Z}); rnd = random.Random(seed); rnd.shuffle(boards)
    teil = {b: i % falten for i, b in enumerate(boards)}
    erg = collections.defaultdict(list); n = 0
    for f in range(falten):
        tr = [z for z in Z if teil[gruppe(z)] != f]; te = [z for z in Z if teil[gruppe(z)] == f]
        if not te or len({ziel(z)[0] for z in tr}) < 2: continue
        mod = trainieren(tr); modr = trainieren_rel(tr)
        def top2r(ks, mod=modr):
            P, C = vorhersage(mod, [k['merkmale'] for k in ks]); E = P * C + (1 - P) * 2 * float(np.median(C))
            zwei = [ks[i] for i in np.argsort(E)[:2]]
            return min(zwei, key=lambda k: (not ziel(k)[0], ziel(k)[1] or 1e9))
        def top2(ks, mod=mod):
            P, C = vorhersage(mod, [k['merkmale'] for k in ks]); med = float(np.median(C)); E = P * C + (1 - P) * 2 * med
            zwei = [ks[i] for i in np.argsort(E)[:2]]
            return min(zwei, key=lambda k: (not ziel(k)[0], ziel(k)[1] or 1e9))
        def faust2(ks):
            zwei = sorted(ks, key=lambda k: k['merkmale']['mst'] * (1 + 0.02 * k['merkmale']['kreuzungen']) * (1 + max(0.0, k['merkmale']['rudy_max'] - 1)))[:2]
            return min(zwei, key=lambda k: (not ziel(k)[0], ziel(k)[1] or 1e9))
        def zufall2(ks):
            zwei = sorted(ks, key=lambda k: k['kandidat'])[:2]
            return min(zwei, key=lambda k: (not ziel(k)[0], ziel(k)[1] or 1e9))
        for name, w in (('modell', wahl_modell(mod)), ('modell_top2', top2), ('relativ', wahl_modell(modr)), ('relativ_top2', top2r), ('faust_top2', faust2), ('erste2', zufall2),
                        ('faustregel', wahl_faust), ('erster', lambda ks: min(ks, key=lambda k: k['kandidat'])),
                        ('orakel', lambda ks: min(ks, key=lambda k: (not ziel(k)[0], ziel(k)[1] or 1e9)))):
            v, m = bewerte_auswahl(te, w); erg[name].append((v, m))
        n += 1
    return {k: round(sum(v * m for v, m in L) / max(1, sum(m for _, m in L)), 4) for k, L in erg.items()}, n


def einstellungen(Z):
    ""
    pro = collections.defaultdict(list)
    for z in Z: pro[z['board']].append(z)
    best = {b: min([ziel(k)[1] for k in ks if ziel(k)[0] and ziel(k)[1]] or [None], key=lambda x: x or 1e18) for b, ks in pro.items()}
    out = {}
    for key in ('kreuz', 'trenn', 'via_kosten', 'via_f'):
        d = collections.defaultdict(lambda: [0, 0, []])
        for z in Z:
            v = z['cfg'].get(key); d[v][0] += 1; d[v][1] += bool(ziel(z)[0])
            if ziel(z)[0] and best[z['board']]: d[v][2].append(ziel(z)[1] / best[z['board']])
        out[key] = {str(v): dict(n=a, gueltig=round(g / a, 3), kosten_rel=round(float(np.mean(c)), 3) if c else None) for v, (a, g, c) in sorted(d.items(), key=lambda kv: str(kv[0]))}
    return out


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    pfad = kw.get('daten', os.path.join(R, 'lernen/daten/versuche.jsonl'))
    Z = laden(pfad); print('Versuche %d, Boards %d, gueltig %.0f %%' % (len(Z), len({z['board'] for z in Z}), 100 * np.mean([ziel(z)[0] for z in Z]) if Z else 0))
    if sys.argv[1] == 'einstellungen':
        print(json.dumps(einstellungen(Z), indent=1)); return
    split = kw.get('split', 'familie')
    cv, n = kreuzvalidierung(Z, split=split)
    print('Kreuzvalidierung nach %s (%d Falten), Pruefstandswert je Auswahl (kleiner ist besser):' % ('Designfamilien' if split == 'familie' else 'Boards', n), json.dumps(cv))
    import joblib
    aus = kw.get('aus', os.path.join(R, 'lernen/modell')); os.makedirs(aus, exist_ok=True)
    art = kw.get('art', 'rel')
    mod = trainieren_rel(Z) if art == 'rel' else trainieren(Z)
    joblib.dump(mod, os.path.join(aus, 'auswahl.joblib'))
    json.dump(dict(art=art, merkmale=MERKMALE, versuche=len(Z), boards=len({z['board'] for z in Z}), kreuzvalidierung=cv, daten=os.path.relpath(pfad, R),
                   einstellungen=einstellungen(Z)), open(os.path.join(aus, 'auswahl.json'), 'w'), indent=1)
    print('Modell gespeichert ->', aus)


if __name__ == '__main__':
    main()
