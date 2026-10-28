""
import collections, fcntl, glob, hashlib, json, math, os, random, shutil, sys, time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
VPY = os.path.join(R, '.venv', 'bin', 'python')
DATEN = os.path.join(R, 'lernen', 'daten', 'zuege.jsonl')
START = os.path.join(R, 'lernen', 'daten', 'zuege_start.jsonl')
LESEN = DATEN + ',' + START
MODELL = os.path.join(R, 'lernen', 'modell', 'zuege.json')
ARTEN = ('schieben', 'drehen', 'tauschen', 'versetzen')
GRUENDE = ('engstelle', 'konflikt', 'umweg', 'versetzen')
FUNKTIONEN = ('entkopplung', 'pull', 'serien_r', 'langsam', 'empfindlich', 'sonstig')
NETZARTEN = ('signal', 'strom', 'versorgung', 'analog', 'schnell', 'sw')
GANZ_MM = 50.0



def _f(x, d=0.0):
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else d


def funktion_gruppe(f):
    f = (f or '').lower()
    if f.startswith('entkopplung'): return 'entkopplung'
    if f.startswith('pull'): return 'pull'
    if f == 'serien_r': return 'serien_r'
    if f in ('teiler', 'rc_glied'): return 'langsam'
    if f in ('bootstrap', 'spule_sw', 'quarz_last', 'rueckkopplung', 'rueckkopplung_c', 'einstell_r', 'shunt'): return 'empfindlich'
    return 'sonstig'


def merkmale(before, rang=None, gewinn_max=None):
    ""
    b = before or {}; m = {}
    mv = b.get('move') or {}; art = mv.get('type'); mem = mv.get('members') or {}
    for a in ARTEN: m['art_' + a] = 1.0 if art == a else 0.0
    rs = b.get('reason') or {}
    for g in GRUENDE: m['anlass_' + g] = 1.0 if rs.get('type') == g else 0.0
    gs = _f(rs.get('gewinn_schaetzung'))
    m['gewinn_log'] = math.log1p(max(gs, 0.0))
    m['rang_log'] = math.log1p(rang or 0); m['rang_erster'] = 1.0 if not rang else 0.0
    m['gewinn_rel'] = gs / gewinn_max if gewinn_max and gewinn_max > 0 else 1.0
    wege = [math.hypot(_f(v.get('dx_mm')), _f(v.get('dy_mm'))) for v in mem.values()] or [0.0]
    m['weg_mm'] = max(wege); m['weg_log'] = math.log1p(max(wege))
    rot = [(_f(v.get('rotation_deg')) % 360) for v in mem.values()] or [0.0]
    m['dreh_90'] = 1.0 if any(r in (90.0, 270.0) for r in rot) else 0.0
    m['dreh_180'] = 1.0 if any(r == 180.0 for r in rot) else 0.0
    m['teile_n'] = float(len(mem))
    T = b.get('teile') or [{}]
    t0 = T[0]
    m['entkopplung'] = 1.0 if any(t.get('entkopplung') for t in T) else 0.0
    m['pads_n'] = max(_f(t.get('n_pads')) for t in T)
    m['seite_B'] = 1.0 if t0.get('seite') == 'B' else 0.0
    ww = b.get('wanderweite') or {}
    w0 = ww.get(t0.get('ref')) or (next(iter(ww.values())) if ww else {})
    fg = funktion_gruppe(t0.get('funktion') or (w0 or {}).get('schluessel'))
    for g in FUNKTIONEN: m['funktion_' + g] = 1.0 if fg == g else 0.0
    m['ww_bekannt'] = 1.0 if w0 else 0.0
    m['ww_ganz'] = 1.0 if (w0 or {}).get('ganz') else 0.0
    m['ww_max'] = GANZ_MM if (w0 or {}).get('ganz') else min(_f((w0 or {}).get('max')), GANZ_MM)
    m['ww_wichtigkeit_log'] = math.log1p(_f((w0 or {}).get('wichtigkeit')))
    m['preis'] = _f(b.get('preis'))
    m['preis_rel'] = m['preis'] / (1.0 + gs)

    lo = ((b.get('local_occupancy') or {}).get('teile') or {}).get(t0.get('ref')) or {}
    seite = t0.get('seite') or 'F'
    for h in ('2', '5'):
        e = lo.get(h) or {}
        ca = e.get('courtyard_anteil') or {}
        m['belegt_%s' % h] = _f(ca.get(seite)); m['belegt_andere_%s' % h] = _f(ca.get('B' if seite == 'F' else 'F'))
        m['nachbarn_%s' % h] = _f(e.get('nachbarn'))
    um = b.get('umfeld') or {}
    ut = (um.get('teile') or {}).get(t0.get('ref')) or {}
    m['umfeld_bekannt'] = 1.0 if ut else 0.0
    for h in ('1.5', '4'):
        e = (ut.get('fenster') or {}).get(h) or {}
        m['kupfer_%s' % h] = math.log1p(_f(e.get('kupfer_mm'))); m['kupfer_fremd_%s' % h] = math.log1p(_f(e.get('kupfer_fremd_mm')))
        m['vias_%s' % h] = _f(e.get('vias'))
    m['konflikt_am_teil'] = _f(ut.get('konflikt_netze_am_teil'))
    ka = ut.get('konflikt_abstand_mm')
    m['konflikt_nah'] = 1.0 / (1.0 + _f(ka)) if ka is not None else 0.0

    N = b.get('affected_nets') or []
    m['netze_n'] = float(len(N))
    m['netz_pads_max_log'] = math.log1p(max([_f(n.get('pads')) for n in N] or [0.0]))
    m['netz_gnd'] = 1.0 if any(n.get('gnd') for n in N) else 0.0
    for a in NETZARTEN: m['netz_' + a] = 1.0 if any(n.get('art') == a for n in N) else 0.0
    m['netz_breit'] = max([_f(n.get('w')) for n in N] or [0.0])

    sc = b.get('score') or {}
    m['start_gueltig'] = 1.0 if sc.get('gueltig') else 0.0
    m['start_konflikte_log'] = math.log1p(_f(sc.get('konflikte')))
    m['start_umweg_log'] = math.log1p(max(_f(sc.get('umweg_mm')), 0.0))
    m['start_vias_log'] = math.log1p(_f(sc.get('vias')))

    m['gewinn_x_schieben'] = m['gewinn_log'] * m['art_schieben']
    m['gewinn_x_drehen'] = m['gewinn_log'] * m['art_drehen']
    return m


NAMEN = list(merkmale({}).keys())

KLEIN = ['art_schieben', 'art_drehen', 'art_tauschen', 'art_versetzen', 'gewinn_log', 'rang_log', 'rang_erster', 'gewinn_rel', 'weg_log', 'dreh_180', 'ww_ganz', 'preis_rel',
         'funktion_pull', 'funktion_entkopplung', 'funktion_empfindlich', 'belegt_2', 'kupfer_fremd_1.5', 'vias_1.5',
         'netz_pads_max_log', 'netze_n', 'start_umweg_log']
SAETZE = dict(alle=NAMEN, klein=KLEIN)


def vektor(before, rang=None, gewinn_max=None):
    m = merkmale(before, rang, gewinn_max)
    return [m[n] for n in NAMEN]

def _gs(z): return _f(((z.get('before') or {}).get('reason') or {}).get('gewinn_schaetzung'))


def kontext(Z):
    ""
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
    return np.array([vektor(z.get('before'), *c) for z, c in zip(Z, kontext(Z))], dtype=float)



def _pfade(pfad):
    return [p for p in (pfad.split(',') if isinstance(pfad, str) else list(pfad)) if p]


def lesen(pfad=LESEN):
    ""
    Z = []
    for line in (l for p in _pfade(pfad) if os.path.exists(p) for l in open(p)):
        try: z = json.loads(line)
        except ValueError: continue
        if z.get('schema_version') != 'versuch.v1': continue
        st = (z.get('result') or {}).get('status')
        if st not in ('evaluated', 'router_error') or (z.get('decision') or {}).get('phase') != 'primary': continue
        Z.append(z)
    return Z


def ziel(z):
    ""
    return 1 if (z.get('label') or {}).get('success') is True else 0


def gewinn_eff(z):
    ""
    g = (z.get('label') or {}).get('gain_if_valid')
    return max(_f(g) - _f((z.get('before') or {}).get('preis')), 0.0) if g is not None else 0.0


def familie(z): return (z.get('ids') or {}).get('board_family_id') or 'unbekannt'

def gruppen(Z):
    ""
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
    return dict(mittel=mu.tolist(), streu=sd.tolist(), gewichte=w[:-1].tolist(), achse=float(w[-1]))


def _fit_cl(X, y, grp, l2=10.0):
    ""
    import numpy as np
    from scipy.optimize import minimize
    mu = X.mean(0); sd = X.std(0); sd[sd < 1e-9] = 1.0; Xs = (X - mu) / sd
    G = collections.defaultdict(list)
    for i, g in enumerate(grp): G[g].append(i)
    G = [np.array(v) for v in G.values() if 0 < y[v].sum() < len(v)]
    if not G: return dict(mittel=mu.tolist(), streu=sd.tolist(), gewichte=[0.0] * X.shape[1], achse=0.0)
    def f(w):
        L = 0.5 * l2 * w @ w; g = l2 * w
        for v in G:
            s = Xs[v] @ w; m = s.max(); e = np.exp(s - m); p = e / e.sum()
            pos = y[v] > 0; ep = e[pos]; q = ep / ep.sum()
            L -= np.log(ep.sum()) - np.log(e.sum())
            g -= Xs[v][pos].T @ q - Xs[v].T @ p
        return L, g
    w = minimize(f, np.zeros(X.shape[1]), jac=True, method='L-BFGS-B').x
    return dict(mittel=mu.tolist(), streu=sd.tolist(), gewichte=w.tolist(), achse=0.0)


def _gruppe_id(z):
    ids = z.get('ids') or {}
    return (ids.get('run_id'), ids.get('decision_id'))


def _logit(M, X):
    import numpy as np
    return ((np.asarray(X) - np.asarray(M['mittel'])) / np.asarray(M['streu'])) @ np.asarray(M['gewichte']) + M['achse']


def vorhersage(M, befores):
    ""
    import numpy as np
    if not befores: return []
    names = M['merkmale']
    gm = max(_f(((b or {}).get('reason') or {}).get('gewinn_schaetzung')) for b in befores)
    X = [[merkmale(b, r, gm).get(n, 0.0) for n in names] for r, b in enumerate(befores)]
    return (1 / (1 + np.exp(-np.clip(_logit(M, X), -30, 30)))).tolist()


def laden(pfad):
    return json.load(open(pfad))



def _zufall_treffer(n, s, k):
    ""
    if s == 0: return 0.0
    if k >= n: return 1.0
    return 1.0 - math.comb(n - s, k) / math.comb(n, k)


def bewerte_gruppen(Z, G, wert, k):
    ""
    a = dict(entscheidungen=0, mit_erfolg=0, modell=0.0, heute=0.0, zufall=0.0, g_modell=0.0, g_heute=0.0, g_zufall=0.0, zuege=0, erfolge=0,
             gross=0, gr_modell=0.0, gr_heute=0.0, gr_zufall=0.0)
    for g, idx in G.items():
        a['entscheidungen'] += 1; a['zuege'] += len(idx)
        s = sum(ziel(Z[i]) for i in idx); a['erfolge'] += s
        if s == 0: continue
        a['mit_erfolg'] += 1
        heute = idx[:k]
        modell = sorted(idx, key=lambda i: -wert[i])[:k]
        a['modell'] += 1.0 if any(ziel(Z[i]) for i in modell) else 0.0
        a['heute'] += 1.0 if any(ziel(Z[i]) for i in heute) else 0.0
        a['zufall'] += _zufall_treffer(len(idx), s, k)
        if len(idx) > k:
            a['gross'] += 1; a['gr_modell'] += 1.0 if any(ziel(Z[i]) for i in modell) else 0.0
            a['gr_heute'] += 1.0 if any(ziel(Z[i]) for i in heute) else 0.0; a['gr_zufall'] += _zufall_treffer(len(idx), s, k)
        best = max(gewinn_eff(Z[i]) for i in idx)
        if best > 0:
            a['g_modell'] += max(gewinn_eff(Z[i]) for i in modell) / best
            a['g_heute'] += max(gewinn_eff(Z[i]) for i in heute) / best
            ge = sorted((gewinn_eff(Z[i]) for i in idx), reverse=True); n = len(idx); kk = min(k, n)
            a['g_zufall'] += sum(ge[j] * math.comb(n - 1 - j, kk - 1) for j in range(n - kk + 1)) / math.comb(n, kk) / best
    return a


def paarvergleich(Z, G, wert, k, n_boot=2000, seed=0):
    ""
    je = collections.defaultdict(list)
    for g, idx in G.items():
        if len(idx) <= k or not any(ziel(Z[i]) for i in idx): continue
        m = 1.0 if any(ziel(Z[i]) for i in sorted(idx, key=lambda i: -wert[i])[:k]) else 0.0
        h = 1.0 if any(ziel(Z[i]) for i in idx[:k]) else 0.0
        je[familie(Z[idx[0]])].append(m - h)
    d = [x for v in je.values() for x in v]
    if not d: return None
    rnd = random.Random(seed); fams = sorted(je); boot = []
    for _ in range(n_boot):
        w = [x for f in (rnd.choice(fams) for _ in fams) for x in je[f]]
        boot.append(sum(w) / len(w))
    boot.sort()
    return dict(entscheidungen=len(d), familien=len(fams), siege=sum(1 for x in d if x > 0), niederlagen=sum(1 for x in d if x < 0),
                differenz=round(sum(d) / len(d), 3), bereich95=[round(boot[int(0.025 * n_boot)], 3), round(boot[int(0.975 * n_boot) - 1], 3)])


def _quoten(a):
    n = a['mit_erfolg'] or 1
    return dict(entscheidungen=a['entscheidungen'], mit_erfolg=a['mit_erfolg'], zuege=a['zuege'], erfolge=a['erfolge'],
                modell=round(a['modell'] / n, 3), heute=round(a['heute'] / n, 3), zufall=round(a['zufall'] / n, 3),
                gewinnanteil_modell=round(a['g_modell'] / n, 3), gewinnanteil_heute=round(a['g_heute'] / n, 3),
                gewinnanteil_zufall=round(a['g_zufall'] / n, 3), gross=a['gross'],
                gross_modell=round(a['gr_modell'] / (a['gross'] or 1), 3), gross_heute=round(a['gr_heute'] / (a['gross'] or 1), 3),
                gross_zufall=round(a['gr_zufall'] / (a['gross'] or 1), 3))


def _auc(y, p):
    pos = [q for t, q in zip(y, p) if t]; neg = [q for t, q in zip(y, p) if not t]
    if not pos or not neg: return None
    s = sum((1.0 if a > b else 0.5 if a == b else 0.0) for a in pos for b in neg)
    return round(s / (len(pos) * len(neg)), 3)


def vorhersagen_cv(Z, l2=1.0, art='lin', satz='alle'):
    ""
    import numpy as np
    fam = sorted({familie(z) for z in Z})
    X = matrix(Z); y = np.array([ziel(z) for z in Z], dtype=float)
    X = X[:, [NAMEN.index(n) for n in SAETZE[satz]]]
    wert = np.zeros(len(Z)); mit = []
    for f in fam:
        te = [i for i, z in enumerate(Z) if familie(z) == f]; tr = [i for i, z in enumerate(Z) if familie(z) != f]
        if not tr or y[tr].sum() == 0: continue
        if art == 'cl':
            wert[te] = _logit(_fit_cl(X[tr], y[tr], [_gruppe_id(Z[i]) for i in tr], l2), X[te])
        elif art == 'hgb':
            from sklearn.ensemble import HistGradientBoostingClassifier
            h = HistGradientBoostingClassifier(max_depth=3, max_iter=150, learning_rate=0.05, min_samples_leaf=10, random_state=0)
            wert[te] = h.fit(X[tr], y[tr]).predict_proba(X[te])[:, 1]
        else:
            wert[te] = _logit(_fit(X[tr], y[tr], l2), X[te])
        mit.append(f)
    return wert, mit


def messen(Z, k=4, l2=1.0, art='lin', cv=None, satz='alle'):
    ""
    import numpy as np
    fam = sorted({familie(z) for z in Z}); y = np.array([ziel(z) for z in Z], dtype=float)
    wert, mit = cv or vorhersagen_cv(Z, l2, art, satz); G = gruppen(Z)
    je = {}
    for f in mit:
        Gf = {g: v for g, v in G.items() if familie(Z[v[0]]) == f}
        je[f] = _quoten(bewerte_gruppen(Z, Gf, wert, k))
    gesamt = _quoten(bewerte_gruppen(Z, G, wert, k)); gesamt['paar'] = paarvergleich(Z, {g: v for g, v in G.items() if familie(Z[v[0]]) in mit}, wert, k)
    heute_wert = np.array([_f(((z.get('before') or {}).get('reason') or {}).get('gewinn_schaetzung')) for z in Z])
    return dict(k=k, l2=l2, art=art, satz=satz, zeilen=len(Z), erfolge=int(y.sum()), familien=len(fam), gesamt=gesamt, je_familie=je,
                auc_modell=_auc(y.tolist(), wert.tolist()), auc_heute=_auc(y.tolist(), heute_wert.tolist()),
                familien_modell_vorn=sum(1 for v in je.values() if v['mit_erfolg'] and v['modell'] > v['heute']),
                familien_heute_vorn=sum(1 for v in je.values() if v['mit_erfolg'] and v['modell'] < v['heute']))


def modell_id(M):
    ""
    felder = ('merkmale', 'mittel', 'streu', 'gewichte', 'achse', 'familien', 'ranker_k', 'daten_sha')
    return 'zuege-' + hashlib.sha256(json.dumps({k: M.get(k) for k in felder}, sort_keys=True).encode()).hexdigest()[:16]


def prospektiv(M, Z, k=4):
    ""
    import numpy as np
    alt = set(M.get('familien') or [])
    neu = [i for i, z in enumerate(Z) if familie(z) not in alt]
    if not alt or not neu: return []
    X = matrix(Z)[:, [NAMEN.index(n) for n in M['merkmale']]]
    wert = np.zeros(len(Z)); wert[neu] = _logit(M, X[neu])
    out = []
    for g, idx in sorted(gruppen(Z).items(), key=lambda kv: str(kv[0])):
        if familie(Z[idx[0]]) in alt or len(idx) <= k or not any(ziel(Z[i]) for i in idx): continue
        m = 1.0 if any(ziel(Z[i]) for i in sorted(idx, key=lambda i: -wert[i])[:k]) else 0.0
        h = 1.0 if any(ziel(Z[i]) for i in idx[:k]) else 0.0
        out.append(dict(entscheidung='%s/%s' % g, familie=familie(Z[idx[0]]), model_id=M.get('id'), k=k, modell=m, heute=h, differenz=m - h))
    return out


def prospektiv_stand(zeilen, n_boot=2000, seed=0, model_id=None):
    ""
    if model_id is not None: zeilen = [z for z in zeilen if z.get('model_id') == model_id]
    je = collections.defaultdict(list)
    for z in zeilen: je[z['familie']].append(z['differenz'])
    d = [x for v in je.values() for x in v]
    if not d: return dict(entscheidungen=0, model_id=model_id)
    rnd = random.Random(seed); fams = sorted(je); boot = []
    for _ in range(n_boot):
        w = [x for f in (rnd.choice(fams) for _ in fams) for x in je[f]]
        boot.append(sum(w) / len(w))
    boot.sort()
    return dict(model_id=model_id, entscheidungen=len(d), familien=len(fams), siege=sum(1 for x in d if x > 0), niederlagen=sum(1 for x in d if x < 0),
                differenz=round(sum(d) / len(d), 3), bereich95=[round(boot[int(0.025 * n_boot)], 3), round(boot[int(0.975 * n_boot) - 1], 3)])


def besser_als_heute(mess, min_entscheidungen=20, min_familien=4, prosp=None):
    ""
    g = mess['gesamt']
    fam_mit = sum(1 for v in mess['je_familie'].values() if v['mit_erfolg'])
    if g['mit_erfolg'] < min_entscheidungen or fam_mit < min_familien: return False, 'zu wenig Daten (%d Entscheidungen mit Erfolg, %d Familien)' % (g['mit_erfolg'], fam_mit)
    pa = g.get('paar') or {}
    if g['modell'] > g['heute'] and g['gewinnanteil_modell'] >= g['gewinnanteil_heute'] and (pa.get('bereich95') or [0])[0] > 0:
        if prosp is not None:
            if not prosp.get('model_id'): return False, 'vorausschauend kein einzelnes Modell belegt'
            if prosp.get('familien', 0) < min_familien: return False, 'vorausschauend zu wenige neue Familien'
            if prosp.get('entscheidungen', 0) < min_entscheidungen:
                return False, 'familiengetrennt vorn, vorausschauend noch zu wenig (%d Entscheidungen)' % prosp.get('entscheidungen', 0)
            if prosp['bereich95'][0] <= 0:
                return False, 'familiengetrennt vorn, vorausschauend nicht sicher (Differenz %+.3f, 95 %% %s)' % (prosp['differenz'], prosp['bereich95'])
        return True, 'Modell vorn (Differenz %+.3f, 95 %% %s%s)' % (pa['differenz'], pa['bereich95'],
                                                                 ', vorausschauend %s' % prosp['bereich95'] if prosp else '')
    return False, 'Modell nicht sicher vor der heutigen Reihenfolge (%.3f gegen %.3f, 95 %% der Differenz %s)' % (
        g['modell'], g['heute'], pa.get('bereich95'))


def trainieren(daten=LESEN, modell=None, k=4, l2=10.0, schreiben=True, satz='alle', art='cl'):
    ""
    import numpy as np
    Z = lesen(daten)
    if not Z or sum(ziel(z) for z in Z) == 0: return None
    mess = messen(Z, k, l2, art=art, satz=satz)
    X = matrix(Z); y = np.array([ziel(z) for z in Z], dtype=float)
    X = X[:, [NAMEN.index(n) for n in SAETZE[satz]]]
    fit = _fit_cl(X, y, [_gruppe_id(z) for z in Z], l2) if art == 'cl' else _fit(X, y, l2)
    M = dict(version='zuege-v1', art='bedingtes-logit-l2' if art == 'cl' else 'logistisch-l2', satz=satz, l2=l2, merkmale=SAETZE[satz], **fit,
             ranker_k=k, messung=mess,
             daten=[os.path.abspath(p) for p in _pfade(daten)],
             daten_sha=hashlib.sha256(b''.join(open(p, 'rb').read() for p in _pfade(daten) if os.path.exists(p))).hexdigest()[:16],
             erstellt=time.strftime('%Y-%m-%d %H:%M'))
    M['familien'] = sorted({familie(z) for z in Z})
    M['id'] = modell_id(M)
    if schreiben and modell:
        os.makedirs(os.path.dirname(os.path.abspath(modell)), exist_ok=True)
        json.dump(M, open(modell, 'w'), indent=1)
    return M



def anhaengen(pfad, zeilen):
    ""
    os.makedirs(os.path.dirname(os.path.abspath(pfad)), exist_ok=True)
    with open(pfad, 'a') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            for z in zeilen: f.write(json.dumps(z, ensure_ascii=False, allow_nan=False) + '\n')
            f.flush(); os.fsync(f.fileno())
        finally: fcntl.flock(f, fcntl.LOCK_UN)


def arbeitsordner(c, regeln_von=None, ziel_d=None):
    ""
    ziel_d = ziel_d or os.path.join(c, 'zuege')
    os.makedirs(ziel_d, exist_ok=True)
    for f in ('board.json', 'routes.json', 'klassen.json'):
        shutil.copy(os.path.join(c, f), os.path.join(ziel_d, f))
    rg = os.path.join(c, 'regeln.json')
    if not os.path.exists(rg): rg = os.path.join(regeln_von or os.path.dirname(c.rstrip('/')), 'regeln.json')
    shutil.copy(rg, os.path.join(ziel_d, 'regeln.json'))
    for f in ('nachbessern_versuche.jsonl', 'nachbessern.log'):
        p = os.path.join(ziel_d, f)
        if os.path.exists(p): os.remove(p)
    return ziel_d


def sammeln(c, regeln_von=None, familie_=None, daten=DATEN, cpu_s=900, runden=2, kand=12, iter_=10, parallel=2,
            zuege='schieben,drehen,tauschen,versetzen', ir='1', ranker=None, mit_platz=True, behalten=False, extra=()):
    ""
    from gemeinsam import plaetze as PL
    zd = arbeitsordner(c, regeln_von)
    fam = familie_ or os.path.basename(os.path.dirname(c.rstrip('/')))
    cmd = [VPY, os.path.join(R, 'nachbearbeitung', 'nachbessern.py'), zd, 'runden=%d' % runden, 'kand=%d' % kand,
           'iter=%s' % iter_, 'parallel=%d' % parallel, 'zuege=' + zuege, 'ir=' + str(ir), 'familie=' + fam,
           'budget_cpu=%d' % int(0.7 * cpu_s)] + list(extra)
    if ranker: cmd.append('ranker=' + ranker)
    r = PL.lauf(cmd, cpu_s=cpu_s, wand_s=6 * cpu_s, log=os.path.join(zd, 'nachbessern.out'),
                env=dict(os.environ, PYTHONHASHSEED='0'), mit_platz=mit_platz)
    Z = []
    p = os.path.join(zd, 'nachbessern_versuche.jsonl')
    if os.path.exists(p):
        for line in open(p):
            try: z = json.loads(line)
            except ValueError: continue
            z['sammlung'] = dict(quelle='lernen/zuege.py', kandidat=os.path.abspath(c), zustand=r['zustand'], cpu_s=r['cpu_s'])
            Z.append(z)
    if Z: anhaengen(daten, Z)
    if not behalten: shutil.rmtree(os.path.join(zd, 'nachbessern'), ignore_errors=True)
    return dict(zeilen=len(Z), erfolge=sum(1 for z in Z if (z.get('label') or {}).get('success')), zustand=r['zustand'],
                cpu_s=r['cpu_s'], wand_s=r['wand_s'])


def _pads(d):
    try: return open(os.path.join(d, 'processed.kicad_pcb'), errors='replace').read().count('(pad ')
    except OSError: return 0


def erzeugen(tr, out, kw):
    ""
    from concurrent.futures import ThreadPoolExecutor
    from lernen import sammeln as S
    from lernen import dauerlauf as DL
    from analyse import korpus as KO
    n = int(kw.get('n', '20')); NK_ = int(kw.get('kandidaten', '1')); P = int(kw.get('parallel', '6'))
    zeit = int(kw.get('zeit', '240')); it = int(kw.get('iter', '40')); maxp = int(kw.get('max_pads', '150'))
    daten = os.path.abspath(kw.get('daten', DATEN)); behalten = kw.get('behalten', '0') == '1'; eng = kw.get('eng', '0') == '1'
    rnd = random.Random(int(kw.get('seed', '1')))
    S.V2['an'] = True; KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % zeit,)
    if kw.get('boards'): dirs = [os.path.join(tr, b) for b in kw['boards'].split(',')]
    else:
        schon = {familie(z) for z in lesen([daten, START])}
        for pj in glob.glob(os.path.join(out, '*.jsonl')) + glob.glob(os.path.join(os.path.dirname(out.rstrip('/')), '*', 'erzeugen.jsonl')):
            for line in open(pj):
                try: schon.add(json.loads(line).get('board'))
                except ValueError: pass
        schon |= {os.path.basename(x.rstrip('/')) for x in glob.glob(os.path.join(os.path.dirname(out.rstrip('/')), '*', '*', ''))}
        dirs = [d for d in sorted(glob.glob(os.path.join(tr, '*'))) if os.path.isdir(d) and os.path.basename(d) not in schon and DL.board_ok(d, maxp)[0]
                and _pads(d) >= int(kw.get('min_pads', '0'))]
        rnd.shuffle(dirs); dirs = dirs[:n]
    os.makedirs(out, exist_ok=True)
    proto = os.path.join(out, 'erzeugen.jsonl')
    sk = dict(cpu_s=int(kw.get('cpu', '900')), runden=int(kw.get('runden', '2')), kand=int(kw.get('kand', '12')),
              iter_=kw.get('nb_iter', '10'), parallel=int(kw.get('nb_parallel', '2')), ir=kw.get('ir', '1'),
              zuege=kw.get('zuege', 'schieben,drehen,tauschen,versetzen'), behalten=behalten)

    def board(d):
        name = os.path.basename(d); o = os.path.join(out, name); t0 = time.time(); erg = []
        try:
            info = S.vorbereiten(d, o)
            if not info or info['merkmale_mensch']['n_pads'] > maxp:
                erg.append(dict(board=name, zustand='ungeeignet')); return erg
            info['familie'] = name
            staende = [(d, o, info, k, '') for k in range(NK_)]
            if eng:
                q, f = S.variante_eng(d, o, random.Random('%s-%s-eng' % (kw.get('seed', '1'), name)))
                if q:
                    o2 = o + '_eng'; info2 = S.vorbereiten(q, o2)
                    if info2:
                        info2['familie'] = name; info2['variante'] = dict(art='eng', faktor=f)
                        staende.append((q, o2, info2, 0, 'eng'))
            for dq, oq, iq, k, var in staende:
                cfg = S.zufall_cfg(random.Random('%s-%s-%d%s' % (kw.get('seed', '1'), name, k, var)))
                z = S.kandidat(dq, oq, iq, k, cfg, it, zeit, False)
                g = bool((z.get('ergebnis') or {}).get('gueltig'))
                e = dict(board=name, variante=var or None, kandidat=k, gueltig=g, zustand=z.get('zustand'), s=z.get('s'))
                if g: e['sammeln'] = sammeln(os.path.join(oq, 'k%d' % k), oq, familie_=name, daten=daten, **sk)
                erg.append(e); print(json.dumps(e), flush=True)
        except Exception as ex:
            erg.append(dict(board=name, fehler=repr(ex)[:300]))
        finally:
            for e in erg: e['s_gesamt'] = round(time.time() - t0, 1)
            anhaengen(proto, erg)
            if not behalten:
                for x in (o, o + '_eng', os.path.join(out, '_quellen', name + '_eng')): shutil.rmtree(x, ignore_errors=True)
        return erg

    with ThreadPoolExecutor(P) as ex:
        alle = [e for es in ex.map(board, dirs) for e in es]
    n_z = sum((e.get('sammeln') or {}).get('zeilen', 0) for e in alle)
    print('erzeugen: %d Boards, %d gueltige Staende, %d Zug-Zeilen -> %s' % (len(dirs), sum(1 for e in alle if e.get('gueltig')), n_z, daten), flush=True)


def spaeter(Z, fam_alt, ks=(1, 2, 3, 4), l2=10.0, satz='alle', art='cl'):
    ""
    import numpy as np
    alt = [i for i, z in enumerate(Z) if familie(z) in fam_alt]; neu = [i for i, z in enumerate(Z) if familie(z) not in fam_alt]
    if not alt or not neu: return None
    X = matrix(Z)[:, [NAMEN.index(n) for n in SAETZE[satz]]]; y = np.array([ziel(z) for z in Z], dtype=float)
    M = _fit_cl(X[alt], y[alt], [_gruppe_id(Z[i]) for i in alt], l2) if art == 'cl' else _fit(X[alt], y[alt], l2)
    wert = np.zeros(len(Z)); wert[neu] = _logit(M, X[neu])
    G = {g: v for g, v in gruppen(Z).items() if familie(Z[v[0]]) not in fam_alt}
    return dict(art=art, satz=satz, l2=l2, zeilen_alt=len(alt), zeilen_neu=len(neu), familien_neu=len({familie(Z[i]) for i in neu}), erfolge_neu=int(y[neu].sum()),
                auc_modell=_auc(y[neu].tolist(), wert[neu].tolist()), auc_heute=_auc(y[neu].tolist(), [_gs(Z[i]) for i in neu]),
                treffer={'@%d' % k: dict(_quoten(bewerte_gruppen(Z, G, wert, k)), paar=paarvergleich(Z, G, wert, k)) for k in ks})


def bericht(daten=LESEN, ks=(1, 2, 3, 4), entwicklung=None):
    ""
    Z = lesen(daten); fam = collections.Counter(familie(z) for z in Z)
    out = dict(zeilen=len(Z), erfolge=sum(ziel(z) for z in Z), familien=len(fam),
               familien_mit_erfolg=len({familie(z) for z in Z if ziel(z)}), entscheidungen=len(gruppen(Z)),
               status=dict(collections.Counter(z['result']['status'] for z in Z)),
               start_gueltig=sum(1 for z in Z if ((z.get('before') or {}).get('score') or {}).get('gueltig')))
    def quote(key):
        c = collections.defaultdict(lambda: [0, 0])
        for z in Z: c[key(z)][0] += 1; c[key(z)][1] += ziel(z)
        return {str(a): '%d/%d' % (v[1], v[0]) for a, v in sorted(c.items(), key=lambda kv: str(kv[0]))}
    out['je_zugart'] = quote(lambda z: z['before']['move']['type'])
    out['je_anlass'] = quote(lambda z: (z['before'].get('reason') or {}).get('type'))
    out['je_funktion'] = quote(lambda z: funktion_gruppe(((z['before'].get('teile') or [{}])[0]).get('funktion')))
    out['treffer'] = {}
    for name, art, l2, satz in (('lin_alle_l2=1', 'lin', 1.0, 'alle'), ('lin_alle_l2=10', 'lin', 10.0, 'alle'),
                                ('lin_klein_l2=10', 'lin', 10.0, 'klein'), ('lin_klein_l2=100', 'lin', 100.0, 'klein'), ('hgb_alle', 'hgb', 1.0, 'alle'),
                                ('cl_klein_l2=1', 'cl', 1.0, 'klein'), ('cl_klein_l2=10', 'cl', 10.0, 'klein'), ('cl_alle_l2=10', 'cl', 10.0, 'alle')):
        cv = vorhersagen_cv(Z, l2, art, satz)
        for k in ks:
            try: m = messen(Z, k, l2, art=art, cv=cv, satz=satz)
            except Exception as e: out['treffer']['%s@%d' % (name, k)] = repr(e)[:100]; continue
            g = m['gesamt']
            out['treffer']['%s@%d' % (name, k)] = dict(modell=g['modell'], heute=g['heute'], zufall=g['zufall'],
                                                     gewinn_modell=g['gewinnanteil_modell'], gewinn_heute=g['gewinnanteil_heute'],
                                                     gewinn_zufall=g['gewinnanteil_zufall'], mit_erfolg=g['mit_erfolg'],
                                                     entscheidungen=g['entscheidungen'], auc=m['auc_modell'], auc_heute=m['auc_heute'],
                                                     fam_vorn=m['familien_modell_vorn'], fam_hinten=m['familien_heute_vorn'],
                                                     gross=g['gross'], gross_modell=g['gross_modell'], gross_heute=g['gross_heute'], gross_zufall=g['gross_zufall'],
                                                     paar=g.get('paar'))
    if entwicklung:
        out['spaeter'] = spaeter(Z, set(json.load(open(entwicklung))['familien']), ks)
    return out


def _note(log, was):
    import re
    for line in reversed(open(log).read().splitlines()) if os.path.exists(log) else []:
        m = re.match(r'%s \((v1|v2)\): Note ([-0-9.]+).*?(gueltig|UNGUELTIG)' % was, line)
        if m: return float(m.group(2)), m.group(3) == 'gueltig'
    return None, None


def vergleich(tr, out, kw):
    ""
    from concurrent.futures import ThreadPoolExecutor
    from lernen import sammeln as S
    from lernen import dauerlauf as DL
    from analyse import korpus as KO
    from gemeinsam import plaetze as PL
    k = int(kw.get('k', '4')); modell = os.path.abspath(kw['ranker']); P = int(kw.get('parallel', '4'))
    zeit = int(kw.get('zeit', '240')); it = int(kw.get('iter', '40')); maxp = int(kw.get('max_pads', '220'))
    S.V2['an'] = True; KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % zeit,)
    meiden = set(json.load(open(kw['entwicklung']))['familien']) if kw.get('entwicklung') else set()
    dirs = [os.path.join(tr, b) for b in kw['boards'].split(',')] if kw.get('boards') else []
    if not dirs:
        alle = [d for d in sorted(glob.glob(os.path.join(tr, '*'))) if os.path.isdir(d) and os.path.basename(d) not in meiden
                and DL.board_ok(d, maxp)[0] and _pads(d) >= int(kw.get('min_pads', '70'))]
        random.Random(int(kw.get('seed', '5'))).shuffle(alle); dirs = alle[:int(kw.get('n', '8'))]
    os.makedirs(out, exist_ok=True); proto = os.path.join(out, 'vergleich.jsonl')
    varianten = [('voll', ['kand=12']), ('heute_%d' % k, ['kand=%d' % k]), ('ranker_%d' % k, ['kand=12', 'ranker=' + modell, 'ranker_k=%d' % k])]

    def board(d):
        name = os.path.basename(d); o = os.path.join(out, name)
        try:
            info = S.vorbereiten(d, o)
            if not info: return
            info['familie'] = name
            z = S.kandidat(d, o, info, 0, S.zufall_cfg(random.Random('v-%s' % name)), it, zeit, False)
            if not (z.get('ergebnis') or {}).get('gueltig'):
                anhaengen(proto, [dict(board=name, gueltig=False)]); return
            c = os.path.join(o, 'k0'); erg = dict(board=name, gueltig=True)
            for vn, ex in varianten:
                zd = arbeitsordner(c, o, os.path.join(c, vn))
                r = PL.lauf([VPY, os.path.join(R, 'nachbearbeitung', 'nachbessern.py'), zd, 'runden=%s' % kw.get('runden', '2'),
                             'iter=10', 'parallel=2', 'zuege=schieben,drehen,tauschen,versetzen', 'familie=' + name] + ex,
                            cpu_s=int(kw.get('cpu', '900')), log=os.path.join(zd, 'nachbessern.out'), env=dict(os.environ, PYTHONHASHSEED='0'))
                n0, g0 = _note(os.path.join(zd, 'nachbessern.log'), 'Basis'); n1, g1 = _note(os.path.join(zd, 'nachbessern.log'), 'Ergebnis')
                vs = os.path.join(zd, 'nachbessern_versuche.jsonl')
                erg[vn] = dict(note0=n0, note1=n1, gueltig1=g1, versuche=sum(1 for _ in open(vs)) if os.path.exists(vs) else 0,
                               cpu_s=r['cpu_s'], zustand=r['zustand'])
            anhaengen(proto, [erg]); print(json.dumps(erg), flush=True)
        except Exception as e:
            anhaengen(proto, [dict(board=name, fehler=repr(e)[:300])])
        finally:
            if kw.get('behalten', '0') != '1': shutil.rmtree(o, ignore_errors=True)

    with ThreadPoolExecutor(P) as ex: list(ex.map(board, dirs))


def main():
    if len(sys.argv) < 2: print(__doc__); return
    cmd = sys.argv[1]
    pos = [a for a in sys.argv[2:] if '=' not in a]; kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    if cmd == 'erzeugen': erzeugen(pos[0], pos[1], kw)
    elif cmd == 'vergleich': vergleich(pos[0], pos[1], kw)
    elif cmd == 'sammeln':
        print(json.dumps(sammeln(os.path.abspath(pos[0]), kw.get('regeln_von'), kw.get('familie'), os.path.abspath(kw.get('daten', DATEN)),
                                 int(kw.get('cpu', '900')), int(kw.get('runden', '2')), int(kw.get('kand', '12')), kw.get('nb_iter', '10'),
                                 int(kw.get('nb_parallel', '2')), kw.get('zuege', 'schieben,drehen,tauschen,versetzen'), kw.get('ir', '1'),
                                 kw.get('ranker'))))
    elif cmd == 'bericht': print(json.dumps(bericht(kw.get('daten', LESEN), entwicklung=kw.get('entwicklung')), indent=1, ensure_ascii=False))
    elif cmd in ('trainieren', 'messen'):
        M = trainieren(kw.get('daten', LESEN), kw.get('modell', MODELL), int(kw.get('k', '4')), float(kw.get('l2', '10')),
                       schreiben=cmd == 'trainieren' and kw.get('schreiben', '1') == '1', satz=kw.get('satz', 'alle'), art=kw.get('art', 'cl'))
        if M is None: print('keine Daten mit Erfolg'); return
        m = M['messung']; ok, grund = besser_als_heute(m)
        print(json.dumps(dict(id=M['id'], zeilen=m['zeilen'], erfolge=m['erfolge'], familien=m['familien'], gesamt=m['gesamt'],
                              auc_modell=m['auc_modell'], auc_heute=m['auc_heute'], familien_modell_vorn=m['familien_modell_vorn'],
                              familien_heute_vorn=m['familien_heute_vorn'], uebernehmen=ok, grund=grund), indent=1))
    else: print(__doc__)


if __name__ == '__main__':
    main()
