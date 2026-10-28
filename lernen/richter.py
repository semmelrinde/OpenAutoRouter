""
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
import zlib

os.environ.setdefault('OMP_NUM_THREADS', '1')
R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
VERSION = 'richter.v1'
SEED = 7
MIN_NOTEN = 100
MIN_DOPPEL = 10


def _hash(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def atomar(p, z):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    t = p.with_name(p.name + '.tmp.%s.%s' % (os.getpid(), threading.get_ident()))
    with t.open('w') as f:
        json.dump(z, f, indent=1, allow_nan=False); f.flush(); os.fsync(f.fileno())
    os.replace(t, p)


def familie(z):
    if z.get('designfamilie'): return str(z['designfamilie']).lower()

    f = str(z.get('designfamilie') or z.get('familie') or z.get('board') or '')
    f = re.sub(r'^\d+_', '', f).lower().removesuffix('_eng')
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]


def zurueck(f):
    return zlib.crc32(f.encode()) % 5 == 0


def merkmale(z):
    ""
    from lernen.form_merkmale import gruppenfehler, kette_ok, r39_ok
    regel = z.get('regel') or {}
    quellen = [z, regel, regel.get('regeln') or {}]
    out = {}
    def flach(v, pf, parents=(), target=None, raw=False):
        if target is None: target = out
        if isinstance(v, dict):
            if 'fehler' in v and pf not in ('', 'regel','regel.regeln'): return
            if v.get('vollstaendig') is False or v.get('ausgelassen'): return
            errors = dict(v.get('diagnose_fehler') or {})
            if isinstance(v.get('fehler'),dict): errors.update(v['fehler'])
            elif 'fehler' in v: return
            sources = (*quellen, *parents, v)
            if gruppenfehler([v]): return
            for k, x in sorted(v.items()):
                if k in ('bsp_knoten', 'bsp_parallel'): continue
                if k in errors or k+'_fehler' in v: continue
                if k == 'strom_engstellen' and isinstance(x, dict) and not r39_ok(sources) and 'fehler' not in x and not x.get('nicht_messbar'):


                    from analyse.strom_engstellen import lernwerte
                    gp = (x if raw else lernwerte(x)).get('gemessene_pfade')
                    if isinstance(gp, dict) and not gruppenfehler([gp]):
                        flach(gp, pf + '.strom_engstellen.gemessene_pfade', (), target, raw)
                    continue
                if k not in ('regel', 'regeln') and gruppenfehler(sources, k): continue
                if k == 'breitenwechsel' and gruppenfehler(sources, 'kennzahlen', 'breitenwechsel'): continue
                if k == 'strom_kette' and not kette_ok(x): continue
                if (k in ('netze', 'parallel_netze') or (k == 'gruppen' and pf == 'regel.regeln.r11')) and isinstance(x, dict):
                    sums = collections.defaultdict(list)
                    for q in x.values():
                        values = {}
                        flach(q, '', sources, values, True)
                        for path, value in values.items(): sums[path].append(value)
                    for path, vals in sums.items():

                        if len(vals) != len(x): continue


                        axis = 'gruppen' if k == 'gruppen' else 'netze'
                        flach(dict(mittel=sum(vals)/len(vals), maximum=max(vals), summe=sum(vals)), pf + '.' + axis + path, sources, target, raw)
                else:
                    if k == 'strom_engstellen':
                        if not r39_ok(sources): continue
                        if not raw:
                            from analyse.strom_engstellen import lernwerte
                            x = lernwerte(x)
                    if k in ('strom_engstellen_mm2', 'strom_engstellen_n') and not r39_ok(sources):
                        continue
                    flach(x, pf + '.' + str(k), sources, target, raw)
        elif isinstance(v, (int, float)) and math.isfinite(v):
            target[pf] = v if raw else math.copysign(math.log1p(abs(v)), v)
            if not raw: target[pf + '.vorhanden'] = 1.0
        elif isinstance(v, list) and not raw:

            nums = [x for x in v if isinstance(x, (int, float)) and math.isfinite(x)]
            if nums: flach(dict(mittel=sum(nums) / len(nums), maximum=max(nums), anzahl=len(nums)), pf, parents, target, raw)
    QUELLEN = ('kennzahlen', 'merkmale', 'regel', 'r27', 'topologie', 'kopplung', 'note_v3', 'v3_teile',
               'note_v3_teile', 'metriken_teile', 'v3_rel', 'lagen', 'dichte', 'gueltig')
    for k in QUELLEN:
        flach(z.get(k), k)
    from lernen.form_merkmale import werte as formwerte
    for k, v in formwerte(z).items(): flach(v, 'form.' + k)

    out['klasse=' + str(z.get('klasse') or (z.get('dauer') or {}).get('klasse') or '?')] = 1.0


    r27 = (z.get('regel') or {}).get('r27') or z.get('r27') or {}
    if isinstance(r27, dict) and r27.get('version') in ('topologie-v5-stamm', 'topologie-v6-stamm', 'topologie-v7-stamm', 'topologie-v8-stamm'):
        out['r27_version=' + r27['version']] = 1.0


    kupfer, pfade = set(), set()
    def r39(v, tiefe=0):
        if not isinstance(v, dict) or tiefe > 4: return
        for kk, x in v.items():
            if kk == 'r39_version' and isinstance(x, str): kupfer.add(x)
            elif kk == 'strom_engstellen' and isinstance(x, dict):
                if isinstance(x.get('version'), str): kupfer.add(x['version'])
                gp = x.get('gemessene_pfade')
                if isinstance(gp, dict) and isinstance(gp.get('version'), str): pfade.add(gp['version'])
            r39(x, tiefe + 1)
    for k in QUELLEN: r39(z.get(k))
    for v in sorted(kupfer):
        if re.fullmatch(r'r39-kupfer-v\d+(\.\d+)?', v): out['r39_version=' + v] = 1.0
    for v in sorted(pfade):
        if re.fullmatch(r'r39-pfade-v\d+(\.\d+)?', v): out['r39_pfade_version=' + v] = 1.0
    return out


def lesen(pfade, max_n=5000):
    ""
    G = collections.OrderedDict(); roh = 0; verworfen = 0
    for p in pfade:
        if not Path(p).exists(): continue
        with Path(p).open() as stream:
            for l in stream:
                try:
                    z = json.loads(l); n = z['note']; f = familie(z)
                    if isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n) or not 1 <= n <= 10 or not f:
                        raise ValueError('Note/Familie')
                except (ValueError, KeyError, TypeError): verworfen += 1; continue
                roh += 1
                sid = z.get('sample_id') or hashlib.sha256((str(z.get('board')) + ':' + str(z.get('bild'))).encode()).hexdigest()
                if sid in G and G[sid]['_familie'] != f: raise ValueError('Widerspruechliche Familie fuer Bild ' + sid)
                g = G.setdefault(sid, dict(z, sample_id=sid, _familie=f, _noten={}))
                g['_noten'][str(z.get('richter') or 'unbekannt')] = float(n)
                if z.get('richter') == 'nutzer': g.update({k: z.get(k) for k in ('rang', 'gruppe', 'gruppe_n')})

                g['note'] = g['_noten']['nutzer'] if 'nutzer' in g['_noten'] else sum(g['_noten'].values()) / len(g['_noten'])
                if len(G) > max_n: G.popitem(last=False)
    Z = list(G.values())[-max_n:]
    return Z, dict(roh=roh, kandidaten=len(Z), verworfen=verworfen, fenster=max_n, begrenzt=roh > max_n)


def messen(y, p, familien):
    import numpy as np
    from scipy.stats import spearmanr
    if not len(y): return dict(n=0, familien=0, spearman=None, mae=None, pro_familie={})
    y, p = np.array(y), np.array(p)
    def rho(a, b):
        if len(a) < 4 or len(set(a)) < 2 or len(set(b)) < 2: return None
        v = float(spearmanr(a, b).statistic)
        return v if math.isfinite(v) else None
    F = {}
    for f in sorted(set(familien)):
        ix = np.array([v == f for v in familien])
        F[f] = dict(n=int(sum(ix)), mae=float(np.mean(abs(y[ix] - p[ix]))), spearman=rho(y[ix], p[ix]))
    rhos = [v['spearman'] for v in F.values() if v['spearman'] is not None]
    return dict(n=len(y), familien=len(F), spearman=rho(y, p), mae=float(np.mean(abs(y - p))),
                mae_familien=float(np.mean([v['mae'] for v in F.values()])),
                spearman_familien=float(np.mean(rhos)) if rhos else None, pro_familie=F)


def kennzahlen_lernen(Z, seed=SEED):
    from sklearn.feature_extraction import DictVectorizer
    from sklearn.ensemble import ExtraTreesRegressor
    from sklearn.dummy import DummyRegressor
    V = DictVectorizer(sparse=False); X = V.fit_transform([merkmale(z) for z in Z])
    M = ExtraTreesRegressor(n_estimators=48, max_depth=10, min_samples_leaf=2, random_state=seed, n_jobs=1) if len(Z) >= 8 else DummyRegressor()
    M.fit(X, [z['note'] for z in Z])
    return dict(art='kennzahlen', vektor=V, modell=M, version=VERSION)


def bild_array(p, groesse=96):
    ""
    import numpy as np
    from PIL import Image, ImageOps
    with Image.open(p) as im:
        im = im.convert('RGB'); im = im.crop((0, min(22, im.height - 1), im.width, im.height))
        im = ImageOps.pad(im, (groesse, groesse), color=(250, 250, 247))
        return np.asarray(im, dtype=np.float32).transpose(2, 0, 1).copy() / 255.0


def netz(n):
    import torch.nn as nn
    class CNN(nn.Module):
        def __init__(self):
            super().__init__()
            self.bild = nn.Sequential(nn.Conv2d(3, 8, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
                                      nn.Conv2d(8, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
                                      nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.AdaptiveAvgPool2d((3, 3)), nn.Flatten())
            self.kopf = nn.Sequential(nn.Linear(288 + n, 32), nn.ReLU(), nn.Linear(32, 1), nn.Sigmoid())
        def forward(self, im, x):
            import torch
            return 1 + 9 * self.kopf(torch.cat([self.bild(im), x], dim=1)).flatten()
    return CNN()


def bild_lernen(Z, seed=SEED, epochen=12):
    import numpy as np
    import torch
    from sklearn.feature_extraction import DictVectorizer
    from sklearn.preprocessing import StandardScaler
    if not torch.backends.mps.is_available(): raise RuntimeError('Bildtraining braucht MPS; kein GPU-Rueckfall')
    torch.set_num_threads(1); torch.manual_seed(seed)
    V = DictVectorizer(sparse=False); sc = StandardScaler()
    X = sc.fit_transform(V.fit_transform([merkmale(z) for z in Z])).astype('float32')

    M = netz(X.shape[1]).to('mps'); opt = torch.optim.Adam(M.parameters(), lr=0.002)
    rnd = np.random.default_rng(seed); M.train()
    for _ in range(epochen):
        ids = rnd.permutation(len(Z))
        for a in range(0, len(ids), 16):
            ix = ids[a:a + 16]
            im = torch.from_numpy(np.stack([bild_array(Z[i]['bild_pfad']) for i in ix])).to('mps')
            x = torch.from_numpy(X[ix]).to('mps'); y = torch.tensor([Z[i]['note'] for i in ix], device='mps')
            opt.zero_grad(); loss = ((M(im, x) - y) ** 2).mean(); loss.backward(); opt.step()
    torch.mps.synchronize()
    state = {k: v.detach().cpu() for k, v in M.state_dict().items()}
    del M, opt; torch.mps.empty_cache()
    return dict(art='bild', vektor=V, skala=sc, state=state, version=VERSION, epochen=epochen, seed=seed, geraet='mps')


def vorhersage(M, Z):
    import numpy as np
    X = M['vektor'].transform([merkmale(z) for z in Z])
    if M['art'] == 'kennzahlen': return np.clip(M['modell'].predict(X), 1, 10).tolist()
    import torch
    if not torch.backends.mps.is_available(): raise RuntimeError('Bildvorhersage braucht MPS')
    if '_netz' not in M:
        M['_netz'] = netz(X.shape[1]).to('mps'); M['_netz'].load_state_dict(M['state']); M['_netz'].eval()
    X = M['skala'].transform(X).astype('float32'); out = []
    with torch.inference_mode():
        for a in range(0, len(Z), 16):
            im = torch.from_numpy(np.stack([bild_array(z['bild_pfad']) for z in Z[a:a + 16]])).to('mps')
            x = torch.from_numpy(X[a:a + 16]).to('mps'); out.extend(M['_netz'](im, x).cpu().tolist())
    return out


def doppel_messen(Z, pred):
    y, a, p, f = [], [], [], []
    for z, v in zip(Z, pred):
        N = [(k, n) for k, n in sorted(z['_noten'].items()) if k != 'unbekannt']
        if len(N) < 2: continue

        y.append(N[1][1]); a.append(N[0][1]); p.append(v); f.append(z['_familie'])
    return dict(gutachter=messen(y, a, f), ki=messen(y, p, f), referenz='lexikalisch zweiter Gutachter',
                kandidaten=[z['sample_id'] for z in Z if len(z['_noten']) >= 2])


def freigabe(m, n, echt=True):
    ""
    g, k = m['doppel']['gutachter'], m['doppel']['ki']; ges = m['test']
    gruende = []
    if not echt: gruende.append('synthetisch oder nicht dauerhaft archiviert')
    if n < MIN_NOTEN: gruende.append('unter 100 unterschiedliche Kandidaten')
    if m.get('split') != 'fest': gruende.append('keine festen Testfamilien')
    if ges.get('n', 0) < 20 or ges.get('familien', 0) < 3: gruende.append('unter 20 Testbilder / 3 Testfamilien')
    if g.get('n', 0) < MIN_DOPPEL or g.get('familien', 0) < 2: gruende.append('unter 10 Doppelnoten / 2 Testfamilien')
    for rk in ('spearman', 'spearman_familien'):
        if g.get(rk) is None or k.get(rk) is None or g[rk] <= 0 or k[rk] < 0.8 * g[rk]:
            gruende.append('80-Prozent-Tor ' + rk)
    if g.get('mae_familien') is None or k.get('mae_familien') is None or k['mae_familien'] > g['mae_familien'] / 0.8:
        gruende.append('80-Prozent-Tor mittlerer Fehler')
    return not gruende, gruende


def meister_tor(Z, neu, alt):
    ""
    import numpy as np
    a = messen([z['note'] for z in Z], alt, [z['_familie'] for z in Z])
    b = messen([z['note'] for z in Z], neu, [z['_familie'] for z in Z])
    ds = np.array([b['pro_familie'][f]['mae'] - a['pro_familie'][f]['mae'] for f in sorted(a['pro_familie'])])
    if not len(ds): return dict(besser=False, grund='keine Testfamilien')
    rnd = np.random.default_rng(SEED)
    ci = np.quantile([np.mean(rnd.choice(ds, len(ds))) for _ in range(1000)], [0.025, 0.975]).tolist()
    ok = len(ds) >= 3 and ci[1] < 0 and b['spearman'] is not None and a['spearman'] is not None and b['spearman'] >= a['spearman']
    return dict(besser=bool(ok), mae_diff95=ci, meister=a, herausforderer=b)


def trainieren(pfade, mdir, bilder=True, epochen=12, seed=SEED):

    import fcntl
    d = Path(mdir); d.mkdir(parents=True, exist_ok=True)
    with (d / '.training.lock').open('a') as f:
        try: fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return dict(ergebnis='training_laeuft', freigegeben=False)
        return _trainieren(pfade, mdir, bilder, epochen, seed)


def _trainieren(pfade, mdir, bilder=True, epochen=12, seed=SEED):
    import joblib
    from sklearn.model_selection import GroupKFold
    from threadpoolctl import threadpool_limits
    t0 = time.process_time(); Z, daten = lesen(pfade); d = Path(mdir); d.mkdir(parents=True, exist_ok=True)
    if not Z:
        E = dict(version=VERSION, ergebnis='keine_noten', daten=daten, freigegeben=False)
        atomar(d / 'letztes_training.json', E); return E
    tr = [z for z in Z if not zurueck(z['_familie'])]; te = [z for z in Z if zurueck(z['_familie'])]
    split = 'fest'
    if not tr or not te:
        tr, te = Z, []; split = 'pilot_cv'
    arten = ['kennzahlen']
    bild_ok = all(z.get('bild_pfad') and Path(z['bild_pfad']).is_file() for z in Z)
    if bilder and bild_ok: arten.append('bild')
    ergebnisse = {}; modelle = {}; fehler = {}
    with threadpool_limits(limits=1):
        for art in arten:
            fit = lambda T: kennzahlen_lernen(T, seed) if art == 'kennzahlen' else bild_lernen(T, seed, epochen)
            try:

                trainfs = sorted({z['_familie'] for z in tr})
                cv = dict(n=0, familien=0, mae_familien=None)
                if len(trainfs) >= 2:
                    P = [None] * len(tr)
                    folds = GroupKFold(n_splits=min(3, len(trainfs)))
                    for a, b in folds.split(tr, groups=[z['_familie'] for z in tr]):
                        mm = fit([tr[i] for i in a]); pp = vorhersage(mm, [tr[i] for i in b])
                        for i, v in zip(b, pp): P[i] = v
                        del mm
                    cv = messen([z['note'] for z in tr], P, [z['_familie'] for z in tr])
                M = fit(tr)
                if te: testz, p = te, vorhersage(M, te)
                elif cv['n']: testz, p = tr, P
                else: testz, p = [], []
                test = messen([z['note'] for z in testz], p, [z['_familie'] for z in testz])
                mess = dict(split=split, training_cv=cv, test=test, doppel=doppel_messen(testz, p),
                            training_familien=sorted({z['_familie'] for z in tr}), test_familien=sorted({z['_familie'] for z in te}))
                echt = all(z.get('sample_id') and z.get('archiv') and Path(z['archiv'], 'kennzahlen.json').is_file()
                           and not z.get('synthetisch') for z in Z)
                ok, grund = freigabe(mess, len(Z), echt)
                ergebnisse[art] = dict(mess, freigegeben=ok, gruende=grund); modelle[art] = M
            except Exception as e:
                fehler[art] = repr(e)
    if not modelle: raise RuntimeError('Kein Modell: ' + str(fehler))
    art = min(modelle, key=lambda a: (ergebnisse[a]['training_cv'].get('mae_familien') if ergebnisse[a]['training_cv'].get('mae_familien') is not None else 999, a != 'kennzahlen'))
    M = modelle[art]; M.pop('_netz', None)
    identity = hashlib.sha256(json.dumps(dict(version=VERSION, seed=seed, art=art, epochen=epochen,
                            daten=[(z['sample_id'], z['note'], sorted(z['_noten'].items())) for z in Z]), sort_keys=True).encode()).hexdigest()[:20]
    mp = d / (identity + '.joblib')
    tmp = mp.with_suffix('.tmp.%d' % os.getpid()); joblib.dump(M, tmp); os.replace(tmp, mp)
    E = dict(version=VERSION, noten_einheiten=daten['roh'], id=identity, datei=mp.name, sha256=_hash(mp), art=art, daten=daten,
             messung=ergebnisse[art], varianten=ergebnisse, fehler=fehler, n=len(Z), seed=seed,
             freigegeben=ergebnisse[art]['freigegeben'], training_familien=ergebnisse[art]['training_familien'],
             test_familien=ergebnisse[art]['test_familien'], cpu_s=round(time.process_time() - t0, 3), last=os.getloadavg()[0])
    ersetzt = True; meisterp = d / 'meister.json'
    if meisterp.exists():
        alt = json.loads(meisterp.read_text())
        if alt.get('freigegeben'):
            if not E['freigegeben']: ersetzt = False
            elif set(alt['training_familien']) & {z['_familie'] for z in te}: ersetzt = False
            else:
                am = laden_modell(d, alt)
                E['meister_tor'] = meister_tor(te, vorhersage(M, te), vorhersage(am, te))
                ersetzt = E['meister_tor']['besser']
    E['ersetzt'] = ersetzt
    atomar(d / 'herausforderer.json', E)
    if ersetzt: atomar(meisterp, E)
    atomar(d / 'letztes_training.json', E)
    return E


def laden_modell(d, meta):
    import joblib
    p = Path(d) / meta['datei']
    if p.parent.resolve() != Path(d).resolve() or _hash(p) != meta['sha256']: raise ValueError('Modellhash/Pfad stimmt nicht')
    return joblib.load(p)


class Dienst:
    ""
    def __init__(self, daten, modell, neu=20, bilder=True, epochen=12, cpu=300):
        self.daten, self.modell = Path(daten), Path(modell)
        self.neu, self.bilder, self.epochen, self.cpu = max(1, neu), bilder, epochen, cpu
        self.lock = threading.Lock(); self.thread = None; self.M = None; self.meta = {}; self.stamp = None
        self.pruefzeit = 0; self.gesehen = -1
        p = self.modell / 'letztes_training.json'
        if p.exists():
            try: self.gesehen = json.loads(p.read_text()).get('noten_einheiten', -1)
            except ValueError: pass

    def laden(self):
        p = self.modell / 'meister.json'
        if not p.exists(): return
        stamp = p.stat().st_mtime_ns
        if stamp == self.stamp: return
        meta = json.loads(p.read_text()); M = laden_modell(self.modell, meta)
        self.M, self.meta, self.stamp = M, meta, stamp

    def anwenden(self, z, bild=None):
        with self.lock:
            z.update(note_ki=None, note_ki_modell=None, note_ki_freigegeben=False, note_ki_status='keine_noten')
            try:
                self.laden()
                if self.M:
                    zz = dict(z, bild_pfad=str(bild)) if bild else z
                    z['note_ki'] = float(vorhersage(self.M, [zz])[0]); z['note_ki_modell'] = self.meta['id']
                    z['note_ki_freigegeben'] = self.meta.get('freigegeben', False)
                    z['note_ki_status'] = 'freigegeben' if z['note_ki_freigegeben'] else 'pilot'
            except Exception as e: z['note_ki_status'] = 'fehler'; z['note_ki_fehler'] = repr(e)[:200]
        return z

    def belohnung(self, z, basis, gewicht=0.2):
        if basis is None or basis >= 2 or z.get('code_geaendert') or not z.get('gueltig'): return basis
        with self.lock:
            try: self.laden()
            except Exception: return basis
            n = z.get('note_ki')
            if not self.meta.get('freigegeben') or z.get('note_ki_modell') != self.meta.get('id') or not isinstance(n, (int, float)) or not math.isfinite(n) or not 1 <= n <= 10:
                return basis
        return (1 - gewicht) * basis + gewicht * (10 - n) / 9

    def pruefen(self):

        with self.lock:
            if time.monotonic() - self.pruefzeit < 30 or (self.thread and self.thread.is_alive()): return
            self.pruefzeit = time.monotonic()
            if not self.daten.exists(): return
            from gemeinsam import plaetze as PL

            if PL.knapp() or os.getloadavg()[0] > 3.0 * (os.cpu_count() or 8): return
            Z, stand = lesen([self.daten]); n = stand['roh']
            if not n or (self.gesehen >= 0 and n - self.gesehen < self.neu): return
            self.gesehen = n
            def train():
                cmd = [str(R / '.venv/bin/python'), str(R / 'lernen/richter.py'), 'trainieren',
                       'daten=' + str(self.daten), 'modell=' + str(self.modell), 'bilder=%d' % self.bilder, 'epochen=%d' % self.epochen]
                self.modell.mkdir(parents=True, exist_ok=True)
                r = PL.lauf(cmd, cpu_s=self.cpu, log=str(self.modell / 'training.log'), cwd=str(R))
                atomar(self.modell / 'training_lauf.json', r)
                try: gesperrt = 'training_laeuft' in (self.modell / 'training.log').read_text()[-2000:]
                except OSError: gesperrt = False
                if r['zustand'] != 'ok' or gesperrt:
                    with self.lock: self.gesehen = -1
            self.thread = threading.Thread(target=train, daemon=True); self.thread.start()


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    d = Path(kw.get('modell', str(R / 'lernen/modell/richter'))).expanduser()
    if sys.argv[1] == 'trainieren':
        E = trainieren([Path(p).expanduser() for p in kw.get('daten', str(R / 'lernen/daten/gutachten.jsonl')).split(',')],
                       d, kw.get('bilder', '1') == '1', int(kw.get('epochen', '12')), int(kw.get('seed', SEED)))
        print(json.dumps(E, indent=1)); return
    if sys.argv[1] == 'vorhersagen':
        z = json.loads(Path(kw['zeile']).read_text()); D = Dienst('', d); D.anwenden(z, kw.get('bild')); print(json.dumps(z)); return
    raise SystemExit(__doc__)


if __name__ == '__main__': main()


def physik_vorhersage(samples, modellordner=None):
    ""
    from lernen.ki.routbarkeit_dienst import Dienst as PhysikDienst
    return PhysikDienst(modellordner, einsatz='richter').physik(samples)
