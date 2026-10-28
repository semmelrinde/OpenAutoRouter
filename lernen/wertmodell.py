""
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
from lernen.merkmale import ZUSATZ
from lernen.modell import MERKMALE

_CONTROLLER = None
_CONTROLLER_LOCK = threading.Lock()


def ein_thread(fn):
    ""
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
TORE = ['note_v3', 'kopplung', 'parallel_rest_mm', 'abzweig_mitte', 'spitz', 'abgriff_nah_mm']
CFG = ['kreuz', 'trenn', 'via_kosten', 'via_f', 'pfmul', 'kurz_via', 'einzeltest', 'kritik', 'fein']
NAMEN = MERKMALE + ZUSATZ + ['cfg_' + k for k in CFG] + ['budget_cpu_s', 'budget_iter', 'stil_organisch']


def endlich(v):
    try:
        n = float(v)
        return n if math.isfinite(n) and n >= 0 else None
    except (ValueError, TypeError):
        return None


def familie(z):

    if 'aufgabe' in z: return 'selbstspiel_seed_%s' % z['aufgabe']
    if z.get('designfamilie'): return str(z['designfamilie']).lower()
    from lernen.familien import familie as quellfamilie
    q = quellfamilie(z)
    if '/' in q: return q


    f = re.sub(r'^\d+_', '', str(z.get('familie') or z.get('board') or '')).lower().removesuffix('_eng')
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]


def rolle(f):
    n = zlib.crc32(f.encode()) % 5
    return {0: 'test', 1: 'kalibrierung', 2: 'wahl'}.get(n, 'fit')

def testfamilien():
    out = set()
    for p in (R / 'beispiele').glob('korpus*test*.json'):
        for b in json.loads(p.read_text()).get('boards', []):
            out.update(v for k, v in b.items() if k in ('name', 'pcbench') and isinstance(v, str))
    return out


def ist_test(f, verboten):

    return f in {familie(dict(board=t)) for t in verboten}


def aufgabe(z):

    return json.dumps([z.get('board'), z.get('aufgabe'), z.get('version'),
                       z.get('code_sha256') or z.get('code'), z.get('router_sha256'), z.get('klassen_sha256'),
                       z.get('budget'), z.get('gndlagen'), z.get('faktor'), z.get('stufe_aufgabe'),
                       (z.get('wert_auswahl') or {}).get('pool_id')], sort_keys=True)


def bewertungsstand(z):
    r = z.get('regel') or {}
    version = str(r.get('version') or z.get('bewertung_version') or '')


    rev = tuple((1, int(t)) if t.isdigit() else (0, t) for t in re.findall(r'\d+|\D+', version))
    return bool(r), rev, sum(regel(z, k) is not None for k in TORE)


def profil(z):
    r = (z.get('regel') or {}).get('profil')
    p = z.get('profil')
    if r and p and r != p: raise ValueError('widerspruechlicher Bewertungsprofil-Kontext')
    return str(r or p or 'ausgewogen')


def profil_pruefen(M, Z):
    if not M.get('profil'): raise ValueError('Modell ohne Profilvertrag: neu trainieren')
    if any(profil(z) != M['profil'] for z in Z):
        raise ValueError('Bewertungsprofil passt nicht zum eingefrorenen Modell')


def modellvertrag(M):
    return M.get('version') == VERSION and M.get('namen') == NAMEN and bool(M.get('profil'))


def lesen(pfade):
    from lernen.fruehabbruch import abgebrochen
    unique = {}; stand = collections.Counter(); verboten = testfamilien()
    for p in pfade:
        if not Path(p).exists(): continue
        with open(p) as fh:
            for line in fh:
                stand['roh'] += 1
                try: z = json.loads(line)
                except ValueError:
                    stand['unvollstaendig'] += 1; continue
                if abgebrochen(z):
                    stand['zensiert'] += 1; continue
                if not z.get('merkmale'):
                    stand['ohne_merkmale'] += 1; continue
                f = familie(z)
                if not f or ist_test(f, verboten):
                    stand['testboard_oder_ohne_familie'] += 1; continue
                if z.get('code_geaendert') or z.get('fehler') or z.get('zustand') in ('fehler', 'wandlimit', 'placer_wandlimit', 'placer_fehler'):
                    stand['technischer_fehler'] += 1; continue

                if not z.get('zustand') and z.get('router_rc') not in (None, 0):
                    stand['alter_timeout'] += 1; continue
                if 'gueltig' not in z or not isinstance(z['gueltig'], bool):
                    stand['ohne_gueltigkeit'] += 1; continue
                z['_familie'], z['_aufgabe'] = f, aufgabe(z)
                key = (z['_aufgabe'], json.dumps(z.get('cfg'), sort_keys=True), z.get('kandidat'))

                if key in unique: stand['doppelt'] += 1
                if key not in unique or bewertungsstand(z) >= bewertungsstand(unique[key]): unique[key] = z
    Z = list(unique.values()); stand['zeilen'] = len(Z)
    stand['familien'] = len({z['_familie'] for z in Z})
    stand['note_v3'] = sum(note(z) is not None for z in Z)
    stand['cpu'] = sum(cpu(z) is not None for z in Z)
    stand['regel'] = sum(all(regel(z, k) is not None for k in TORE) for z in Z)
    return Z, dict(stand)

def note(z):
    return endlich((z.get('regel') or {}).get('note_v3', z.get('note_v3')))


def minimax(z):

    return endlich((z.get('regel') or {}).get('minimax'))


def cpu(z):
    v = endlich((z.get('router') or {}).get('cpu_s'))
    if v is None and z.get('format') == 'v2': v = endlich(z.get('route_s'))
    return v

def regel(z, k):
    if k == 'note_v3': return note(z)
    r = z.get('regel') or {}
    return endlich(r.get('kopplung') if k == 'kopplung' else (r.get('topologie') or {}).get(k))


def eingang(z):
    m = dict(z.get('merkmale') or {})
    m.update({'cfg_' + k: (z.get('cfg') or {}).get(k) for k in CFG})
    m.update(budget_cpu_s=(z.get('budget') or {}).get('cpu_s'), budget_iter=(z.get('budget') or {}).get('iter'),
             stil_organisch=int((z.get('cfg') or {}).get('stil', (z.get('cfg') or {}).get('form')) == 'organisch'))
    return [np.nan if endlich(m.get(k)) is None else float(m[k]) for k in NAMEN]


def matrix(Z):
    return np.asarray([eingang(z) for z in Z], float).reshape(len(Z), len(NAMEN))


def transform(M, Z):
    X = matrix(Z)[:, M['indices']]
    return np.where(np.isnan(X), M['median'], X)


@ein_thread
def fit(Z, art='extra'):
    from sklearn.ensemble import ExtraTreesClassifier, ExtraTreesRegressor, HistGradientBoostingClassifier, HistGradientBoostingRegressor
    profiles = {profil(z) for z in Z}
    if len(profiles) != 1: raise ValueError('mehrere Bewertungsprofile: getrennte Daten/Modellordner verwenden')
    X = matrix(Z)

    indices = [i for i in range(len(NAMEN)) if np.isfinite(X[:, i]).sum() >= 5]
    if not indices: raise ValueError('zu wenige beobachtete Merkmale')
    X = X[:, indices]; median = np.nanmedian(X, axis=0); X = np.where(np.isnan(X), median, X)
    M = dict(version=VERSION, profil=next(iter(profiles)), art=art, indices=indices, median=median, namen=NAMEN,
             fit_familien=sorted({z['_familie'] for z in Z}))
    if art == 'extra':
        clf = ExtraTreesClassifier(n_estimators=64, min_samples_leaf=4, max_depth=12, n_jobs=1, random_state=SEED)
        reg = lambda: ExtraTreesRegressor(n_estimators=64, min_samples_leaf=4, max_depth=12, n_jobs=1, random_state=SEED)
    elif art == 'boost':
        clf = HistGradientBoostingClassifier(max_iter=80, max_leaf_nodes=7, min_samples_leaf=8, random_state=SEED)
        reg = lambda: HistGradientBoostingRegressor(max_iter=80, max_leaf_nodes=7, min_samples_leaf=8, random_state=SEED)
    else: raise ValueError('unbekannte Modellart')
    y = [int(z['gueltig']) for z in Z]
    M['clf'] = clf.fit(X, y) if len(set(y)) > 1 else float(np.mean(y))
    for key, target in [('note', note), ('cpu', cpu), ('minimax', minimax)]:
        ok = [i for i, z in enumerate(Z) if target(z) is not None and (key != 'note' or z['gueltig'])]

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


@ein_thread
def kalibrieren(M, Z):
    profil_pruefen(M, Z)
    from sklearn.linear_model import LogisticRegression
    y = [int(z['gueltig']) for z in Z]
    if len(Z) >= 20 and len(set(y)) > 1 and len({z['_familie'] for z in Z}) >= 3:
        M['calibrator'] = LogisticRegression(C=1., random_state=SEED).fit(logits(raw_p(M, transform(M, Z))), y)
    M['kalibrier_familien'] = sorted({z['_familie'] for z in Z})
    return M


@ein_thread
def vorhersage(M, Z):
    if M['version'] != VERSION or M['namen'] != NAMEN: raise ValueError('unpassende Modellversion')
    profil_pruefen(M, Z)
    X = transform(M, Z); raw = raw_p(M, X)
    p = M['calibrator'].predict_proba(logits(raw))[:, 1] if M['calibrator'] is not None else raw
    out = []
    values = {k: np.maximum(0, np.expm1(np.clip(M[k].predict(X), 0, 25))) if M[k] is not None else [None] * len(Z)
              for k in ('note', 'cpu', 'minimax')}
    for i in range(len(Z)):
        out.append(dict(p_gueltig=float(p[i]), p_roh=float(raw[i]),
                        note_v3=None if values['note'][i] is None else float(values['note'][i]),
                        cpu_s=None if values['cpu'][i] is None else float(values['cpu'][i]),
                        minimax=None if values['minimax'][i] is None else float(values['minimax'][i])))
    return out


def faust(z):
    m = z['merkmale']
    return m['mst'] * (1 + .02 * m['kreuzungen']) * (1 + max(0., m['rudy_max'] - 1))


def erwartung(p):
    n = p['minimax'] if p['minimax'] is not None else p['note_v3']
    return 2 * (1 - p['p_gueltig']) + p['p_gueltig'] * (n / (1 + n) if n is not None else .5)


def ordnen(M, Z):
    P = vorhersage(M, Z)
    return sorted(range(len(Z)), key=lambda i: (erwartung(P[i]), faust(Z[i]), i)), P


def gruppen(Z):
    d = collections.defaultdict(list)
    for z in Z: d[z['_aufgabe']].append(z)
    return d


def rang(z):
    n = note(z)
    return (not z['gueltig'], n if n is not None else math.inf, z.get('kandidat', 0))


def bootstrap(paare, value, seed=SEED):
    ds = collections.defaultdict(list)
    for a, b in paare:
        va, vb = value(a), value(b)
        if va is None or vb is None: return dict(n=len(paare), diff=None, ci95=None)
        ds[a['_familie']].append(va - vb)
    if not ds: return dict(n=0, diff=None, ci95=None)
    vals = np.array([np.mean(ds[f]) for f in sorted(ds)])
    rng = np.random.default_rng(seed)
    boot = vals[rng.integers(0, len(vals), (2000, len(vals)))].mean(1)
    return dict(n=len(paare), familien=len(ds), diff=float(vals.mean()), ci95=np.percentile(boot, [2.5, 97.5]).tolist())


def tor(paare):
    valid = bootstrap(paare, lambda z: int(z['gueltig']))
    absolute = dict(modell=sum(a['gueltig'] for a, _ in paare), basis=sum(b['gueltig'] for _, b in paare))
    absolute['delta'] = absolute['modell'] - absolute['basis']
    common = [(a, b) for a, b in paare if a['gueltig'] and b['gueltig']]
    metrics = {k: bootstrap(common, lambda z, k=k: regel(z, k)) for k in TORE}
    ok = bool(absolute['delta'] >= 0 and valid['ci95'] and valid['ci95'][0] >= 0 and valid['familien'] >= 3 and len(paare) >= 20)
    ok &= all(v['ci95'] is not None and v.get('familien', 0) >= 3 and v['ci95'][1] <= 1e-9 for v in metrics.values())
    strict = (valid['ci95'] is not None and valid['ci95'][0] > 0) or any(v['ci95'] and v['ci95'][1] < -1e-9 for v in metrics.values())
    return dict(besser=bool(ok and strict), gueltigkeit=valid, gueltigkeit_absolut=absolute, kennzahlen=metrics, gemeinsam=len(common), aufgaben=len(paare))


def durchsatz(details, basis='faust'):
    rate = collections.defaultdict(lambda: np.zeros(4))
    for v in details:
        if v['modell_cpu_s'] is not None and v[basis + '_cpu_s'] is not None:
            rate[v['familie']] += [v['modell_gueltig'], v['modell_cpu_s'], v[basis + '_gueltig'], v[basis + '_cpu_s']]
    out = dict(diff=None, ci95=None, familien=len(rate), aufgaben=sum(
        v['modell_cpu_s'] is not None and v[basis + '_cpu_s'] is not None for v in details))
    if rate:
        values = np.array([rate[f] for f in sorted(rate)]); totals = values.sum(0)
        if totals[1] > 0 and totals[3] > 0:
            rng = np.random.default_rng(SEED)
            sums = values[rng.integers(0, len(values), (2000, len(values)))].sum(1)
            ok = (sums[:, 1] > 0) & (sums[:, 3] > 0)
            diffs = 3600 * (sums[ok, 0] / sums[ok, 1] - sums[ok, 2] / sums[ok, 3])
            out.update(diff=float(3600 * (totals[0]/totals[1]-totals[2]/totals[3])), ci95=np.percentile(diffs,[2.5,97.5]).tolist())
    return out


def freigabetor(t, d):


    qualitaet = any(v['ci95'] and v['ci95'][1] < -1e-9 for v in t['kennzahlen'].values())
    return bool(t['besser'] and qualitaet and d['aufgaben'] == t['aufgaben']
                and d['familien'] >= 3 and d['ci95'] and d['ci95'][0] > 0)


def pool_cpu(ks, chosen):
    costs = [cpu(z) for z in chosen]
    placing = [endlich((z.get('placer') or {}).get('cpu_s')) for z in ks]
    if any(v is None for v in costs + placing): return None
    return sum(costs) + sum(placing)


def messung(M, Z, routen=1, alt=None):
    pairs = []; details = []; pred = vorhersage(M, Z) if Z else []
    cal = {}
    if Z:
        y = np.array([z['gueltig'] for z in Z], float); p = np.array([v['p_gueltig'] for v in pred]); raw = np.array([v['p_roh'] for v in pred])
        bins = []
        for lo in np.arange(0, 1, .2):
            mask = (p >= lo) & (p < lo + .2 + (1e-9 if lo > .79 else 0))
            if mask.any(): bins.append(dict(n=int(mask.sum()), p=float(p[mask].mean()), gueltig=float(y[mask].mean())))
        cal = dict(n=len(Z), brier=float(np.mean((p-y)**2)), brier_roh=float(np.mean((raw-y)**2)), bins=bins,
                   kalibriert=M['calibrator'] is not None)
    for task, ks in sorted(gruppen(Z).items()):
        if len(ks) < 2: continue
        if len({(z.get('regel') or {}).get('version') or z.get('bewertung_version') for z in ks}) > 1: continue

        if any((z.get('wert_auswahl') or {}).get('pool_n', len(ks)) != len(ks) for z in ks): continue

        if any(z['gueltig'] and note(z) is None for z in ks): continue
        mi, _ = ordnen(M, ks); fi = sorted(range(len(ks)), key=lambda i: (faust(ks[i]), i))
        selected = [ks[i] for i in mi[:routen]]; baseline = [ks[i] for i in fi[:routen]]
        a, b, oracle = min(selected, key=rang), min(baseline, key=rang), min(ks, key=rang)
        pairs.append((a, b))
        details.append(dict(familie=a['_familie'], aufgabe=task, kandidaten=len(ks), modell_gueltig=a['gueltig'],
                            faust_gueltig=b['gueltig'], modell_note=note(a) if a['gueltig'] else None,
                            faust_note=note(b) if b['gueltig'] else None, hit_modell=rang(a)[:2] == rang(oracle)[:2] if oracle['gueltig'] else None,
                            hit_faust=rang(b)[:2] == rang(oracle)[:2] if oracle['gueltig'] else None, modell_cpu_s=pool_cpu(ks, selected), faust_cpu_s=pool_cpu(ks, baseline)))
    def zusammen(art):
        d = [v for v in details if v[art + '_gueltig']]
        timed = [v for v in details if v[art + '_cpu_s'] is not None]
        seconds = sum(v[art + '_cpu_s'] for v in timed)
        return dict(gueltig=len(d), aufgaben=len(details), note_mittel=float(np.mean([v[art + '_note'] for v in d])) if d else None,
                    hit=float(np.mean([v['hit_' + art] for v in details if v['hit_' + art] is not None])) if any(v['hit_' + art] is not None for v in details) else None,
                    hit_aufgaben=sum(v['hit_' + art] is not None for v in details),
                    cpu_s=seconds, cpu_aufgaben=len(timed),
                    cpu_je_gueltig_s=seconds / sum(v[art + '_gueltig'] for v in timed) if any(v[art + '_gueltig'] for v in timed) else None,
                    gueltig_je_cpu_h=3600 * sum(v[art + '_gueltig'] for v in timed) / seconds if seconds else None)
    fehler = {}
    for key, target in [('note_v3', note), ('cpu_s', cpu)]:
        es = [abs(p[key] - target(z)) for z, p in zip(Z, pred) if p[key] is not None and target(z) is not None and (key != 'note_v3' or z['gueltig'])]
        fehler[key] = dict(n=len(es), mae=float(np.mean(es)) if es else None)
    throughput = durchsatz(details)
    out = dict(durchsatz_delta=throughput, vorhersage_fehler=fehler, tor=tor(pairs), modell=zusammen('modell'), faust=zusammen('faust'), kalibrierung=cal, details=details,
               cpu_hinweis='Retrospektiv: Router + alle bekannten Placer-CPU-s; Export/DRC/Form/Training nicht enthalten; kein Live-Durchsatz.')
    if alt is not None:
        ap = []; ad = []
        for ks in gruppen(Z).values():
            if len(ks) < 2 or any(z['gueltig'] and note(z) is None for z in ks) or any((z.get('wert_auswahl') or {}).get('pool_n', len(ks)) != len(ks) for z in ks): continue
            if len({(z.get('regel') or {}).get('version') or z.get('bewertung_version') for z in ks}) > 1: continue
            ai, _ = ordnen(alt, ks); ni, _ = ordnen(M, ks)
            neu, old = [ks[i] for i in ni[:routen]], [ks[i] for i in ai[:routen]]
            a, b = min(neu, key=rang), min(old, key=rang); ap.append((a, b))
            ad.append(dict(familie=a['_familie'], modell_gueltig=a['gueltig'], meister_gueltig=b['gueltig'],
                           modell_cpu_s=pool_cpu(ks, neu), meister_cpu_s=pool_cpu(ks, old)))
        out['meister_tor'] = tor(ap)
        out['meister_durchsatz_delta'] = durchsatz(ad, 'meister')
    return out


def atomar(p, obj):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + '.neu')
    tmp.write_text(json.dumps(obj, indent=1, allow_nan=False)); os.replace(tmp, p)


def speichern(M, path):
    import joblib
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.neu'); joblib.dump(M, tmp); os.replace(tmp, path)


def trainieren(pfade, mdir, routen=1):
    Z, stand = lesen(pfade)
    profiles = {profil(z) for z in Z}
    if len(profiles) > 1: raise ValueError('mehrere Bewertungsprofile: getrennte Daten/Modellordner verwenden')
    parts = {r: [z for z in Z if rolle(z['_familie']) == r] for r in ('fit', 'wahl', 'kalibrierung', 'test')}
    if len(parts['fit']) < 20 or len({z['_familie'] for z in parts['fit']}) < 3:
        raise ValueError('unter 20 Fitzeilen / 3 Fitfamilien')
    pilot = {}; scores = {}
    for art in ('extra', 'boost'):
        M = kalibrieren(fit(parts['fit'], art), parts['kalibrierung'])
        pilot[art] = messung(M, parts['wahl'], routen)
        s = pilot[art]['modell']
        scores[art] = (-s['gueltig'], s['note_mittel'] if s['note_mittel'] is not None else math.inf, art)
    art = min(scores, key=scores.get)
    M = kalibrieren(fit(parts['fit'] + parts['wahl'], art), parts['kalibrierung'])
    M['profil'] = next(iter(profiles), 'ausgewogen')
    M['wahl_familien'] = sorted({z['_familie'] for z in parts['wahl']})
    M['bekannte_familien'] = sorted({z['_familie'] for z in Z})
    M['id'] = hashlib.sha256(json.dumps([VERSION, M['profil'], art, SEED, [(z['_aufgabe'], z.get('cfg'), z['gueltig'], note(z), minimax(z), cpu(z), eingang(z))
                                                            for z in Z if rolle(z['_familie']) != 'test']], sort_keys=True).encode()).hexdigest()[:16]
    report = dict(id=M['id'], profil=M['profil'], stand=stand, art=art, seed=SEED, routen=routen,
                  split={r: dict(zeilen=len(v), familien=sorted({z['_familie'] for z in v})) for r, v in parts.items()},
                  modellwahl=pilot, test=messung(M, parts['test'], routen), freigegeben=False,
                  aktive_merkmale=[NAMEN[i] for i in M['indices']], ziele={k: M[k+'_n'] for k in ('note','cpu','minimax')})
    speichern(M, Path(mdir) / 'kandidat.joblib'); atomar(Path(mdir) / 'kandidat.json', report)
    return M, report


def pruefen(pfade, mdir, routen=1):
    import joblib
    mdir = Path(mdir); Z, stand = lesen(pfade)
    path = mdir / 'kandidat.joblib'
    if not path.exists():
        report = trainieren(pfade, mdir, routen)[1]
        atomar(mdir / 'letztes_training.json', dict(stand=stand, id=report['id'], freigegeben=False))
        return report
    M = joblib.load(path)
    if not modellvertrag(M):


        speichern(M, mdir / ('altvertrag_' + str(M.get('id', 'ohne_id')) + '.joblib'))
        report = trainieren(pfade, mdir, routen)[1]
        report['migration'] = 'alter Modellvertrag: neu trainiert, keine Freigabe'
        atomar(mdir / 'letztes_training.json', report)
        return report

    new = [z for z in Z if z['_familie'] not in M['bekannte_familien'] and rolle(z['_familie']) == 'test']
    profil_pruefen(M, new)
    master_path = mdir / 'meister.joblib'
    alt = joblib.load(master_path) if master_path.exists() else None
    if alt is not None and not modellvertrag(alt): alt = None
    m = messung(M, new, routen, alt)
    ok = freigabetor(m['tor'], m['durchsatz_delta']) and (alt is None or freigabetor(m['meister_tor'], m['meister_durchsatz_delta']))

    report = dict(id=M['id'], profil=M['profil'], stand=stand, prospektiv=m, routen=routen, freigegeben=ok)
    if ok:
        speichern(M, mdir / ('meister_' + M['id'] + '.joblib'))
        atomar(mdir / 'meister.json', dict(id=M['id'], datei='meister_' + M['id'] + '.joblib', freigegeben=True, anwendung='pool', profil=M['profil'], routen=routen))
        speichern(M, master_path)
    atomar(mdir / 'letztes_training.json', report)

    if ok or (m['tor']['aufgaben'] >= 20 and m['tor']['gueltigkeit'].get('familien', 0) >= 3):
        speichern(M, mdir / ('kandidat_' + M['id'] + '.joblib'))
        trainieren(pfade, mdir, routen)
    return report


class Dienst:
    ""
    def __init__(self, daten, modell, neu=100, cpu_s=300, routen=4):
        self.daten = list(daten); self.modell = Path(modell); self.neu = max(1, neu); self.cpu_s = cpu_s; self.routen = routen
        self.lock = threading.Lock(); self.thread = None; self.gesehen = -1; self.pruefzeit = 0
        self.M = None; self.meta = {}; self.stamp = None
        p = self.modell / 'letztes_training.json'
        if p.exists():
            try: self.gesehen = json.loads(p.read_text())['stand']['roh']
            except (ValueError, KeyError): pass

    def laden(self, pilot=False):
        import joblib
        path = self.modell / ('kandidat.json' if pilot else 'meister.json')
        if not path.exists(): return None, {}
        stamp = (str(path), path.stat().st_mtime_ns)
        if stamp != self.stamp:
            meta = json.loads(path.read_text())
            file = 'kandidat.joblib' if pilot else meta['datei']
            M = joblib.load(self.modell / file)
            if M['id'] != meta['id']: raise ValueError('Modell/Metadaten nicht atomar')
            self.M, self.meta, self.stamp = M, meta, stamp
        return self.M, self.meta

    def auswahl(self, Z, art='faust', routen=None, erkunden=0):
        if not Z: return [], dict(status='leer')
        routen = self.routen if routen is None else routen
        order = sorted(range(len(Z)), key=lambda i: (faust(Z[i]), i)); info = dict(status='faust')
        if art in ('wert', 'wert_auto'):
            with self.lock:
                try:
                    M, meta = self.laden(pilot=art == 'wert')
                    code = {(z.get('code_sha256') or z.get('code'), z.get('router_sha256'), z.get('klassen_sha256'), z.get('version')) for z in Z}
                    if len(code) > 1: raise ValueError('Pool mit unterschiedlichen Codezustaenden')
                    if M is not None and (art == 'wert' or (meta.get('freigegeben') and meta.get('anwendung') == 'pool' and meta.get('routen') == routen)):
                        order, P = ordnen(M, Z); info = dict(status=art, id=M['id'], vorhersagen=P)
                except Exception as e: info = dict(status='faust_fehler', fehler=repr(e)[:200])
        take = order[:max(1, routen)]

        rest = [i for i in range(len(Z)) if i not in take]
        if rest and erkunden:
            rng = np.random.default_rng(int((Z[0].get('cfg') or {}).get('seed', SEED)))
            take += rng.choice(rest, min(int(erkunden), len(rest)), replace=False).tolist()
        info['pool_n'] = len(Z)
        info['pool_id'] = hashlib.sha256(json.dumps([(z.get('board'), z.get('kandidat'), z.get('cfg')) for z in Z], sort_keys=True).encode()).hexdigest()[:16]
        info['gewaehlt'] = take
        return take, info
    def pruefen(self):
        with self.lock:
            if time.monotonic() - self.pruefzeit < 30 or (self.thread and self.thread.is_alive()): return
            self.pruefzeit = time.monotonic()
            n = 0
            for p in self.daten:
                if Path(p).exists():
                    with open(p) as f: n += sum(1 for _ in f)
            if n == 0 or (self.gesehen >= 0 and n - self.gesehen < self.neu): return
            from gemeinsam import plaetze as PL
            if PL.knapp(): return
            self.gesehen = n
            def train():
                import fcntl
                self.modell.mkdir(parents=True, exist_ok=True)
                with open(self.modell / 'training.lock', 'a') as lock:
                    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError: return
                    cmd = [str(R / '.venv/bin/python'), str(R / 'lernen/wertmodell.py'), 'pruefen',
                           'daten=' + ','.join(map(str, self.daten)), 'modell=' + str(self.modell), 'routen=%d' % self.routen]
                    r = PL.lauf(cmd, cpu_s=self.cpu_s, log=str(self.modell / 'training.log'), cwd=str(R))
                    atomar(self.modell / 'training_lauf.json', r)
                    if r['zustand'] != 'ok':
                        with self.lock: self.gesehen = -1
            self.thread = threading.Thread(target=train, daemon=True); self.thread.start()


def main():
    kw = dict(s.split('=', 1) for s in sys.argv[2:] if '=' in s)
    pfade = kw['daten'].split(','); mdir = kw['modell']; t = time.process_time()
    if sys.argv[1] == 'trainieren': report = trainieren(pfade, mdir, int(kw.get('routen', '1')))[1]
    elif sys.argv[1] == 'pruefen': report = pruefen(pfade, mdir, int(kw.get('routen', '1')))
    else: raise ValueError('trainieren|pruefen')
    report['cpu_s'] = time.process_time() - t; report['last'] = os.getloadavg()[0]
    atomar(Path(mdir) / 'lauf.json', report)
    print(json.dumps({k: v for k, v in report.items() if k not in ('test', 'modellwahl', 'split')}, indent=1))


if __name__ == '__main__': main()
