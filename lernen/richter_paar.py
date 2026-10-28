""
import collections
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
from lernen.richter import atomar, familie, zurueck
from analyse import vertrag as VT
from lernen.paar_vertrag import messsignatur as messsignatur

VERSION = 'richter-paar-v1'
MESSSCHEMA = 'richter-paar-mess-v3'
SEED = 7
MODELL = R / 'lernen/modell/richter_paar'
NUM = ('vias', 'signal_vias', 'knicke', 'mini', 'fast_stoesse', 'breitenwechsel',
       'laenge', 'laenge_aussen', 'belegung', 'dichte', 'packungsverlust',
       'parallel_mm', 'regelpruefer', 'rueckweg', 'uebersprechen', 'verlust',
       'verlust_mW', 'auge', 'auge_vertikal_verlust', 'auge_horizontal_verlust', 'stoerbudget', 'stoerbudget_max', 'kopplung', 'strom_x_max', 'topologie', 'strom_engstellen_mm2')


def zahl(x):
    return float(x) if not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x) else None


def note_v3(z):
    for d in (z, z.get('regel') or {}, z.get('kennz') or {}, z.get('regeln') or {}):
        if not isinstance(d, dict): continue
        v = zahl(d.get('note_v3'))
        if v is not None and v >= 0: return v
    return None


def basis(z):
    v = note_v3(z)
    return -math.log1p(v) if v is not None else None


def komponenten(M):
    ""
    out = collections.defaultdict(float)
    for v in (M.get('netze') or {}).values():
        for k, x in (v.get('teile') or {}).items():
            if zahl(x) is not None: out[v['klasse'] + '.' + k] += x * v.get('gewicht', 1)
    for k, x in (M.get('board_teile') or {}).items():
        if zahl(x) is not None:
            out['board.' + ('g_max' if k == 'kanal_max' else 'g' if k.startswith('kanal:') else k)] += x
    if M.get('si_note_aktiv'):
        value = (M.get('signalintegritaet') or {}).get('note_zusatz')
        if zahl(value) is not None: out['board.si_zusatz'] += value
    return dict(sorted(out.items()))


def komponentenbeleg(M):
    ""
    if not str(M.get('version', '')).startswith('v3') or not isinstance(M.get('netze'), dict) or not isinstance(M.get('board_teile'), dict): return {}
    for v in M['netze'].values():
        if not isinstance(v.get('klasse'), str) or not isinstance(v.get('teile'), dict): return {}
        if any(zahl(x) is None for x in v['teile'].values()) or zahl(v.get('gewicht', 1)) is None: return {}
    if any(zahl(x) is None for x in M['board_teile'].values()): return {}
    if M.get('si_note_aktiv') and zahl((M.get('signalintegritaet') or {}).get('note_zusatz')) is None: return {}
    c = komponenten(M)
    if zahl(M.get('note_v3')) is None or abs(sum(c.values()) - M['note_v3']) > .011: return {}
    groups = _komponentengruppen(c, M.get('version'))
    return dict(version='v3-komponenten-v2', note_version=M['version'], note_v3=M['note_v3'],
                sha256=hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest(), gruppen=sorted(groups))


def _komponentengruppen(c, version):
    from analyse.regelkennzahlen import TEILE_V3, TEILE_V3_SI, SI_VERSIONS
    mapping = TEILE_V3_SI if version in SI_VERSIONS else TEILE_V3
    out = dict.fromkeys(mapping, 0.0)

    terms = {t:k for k,v in TEILE_V3_SI.items() for t in v}
    for key,value in c.items():
        if not isinstance(key,str) or '.' not in key or zahl(value) is None: return {}
        cls,term = key.split('.',1)
        group = ('verlust' if cls == 'board' and term in ('g','g_max') else
                 'signal' if cls == 'board' and term == 'si_zusatz' else terms.get(term,'sonst'))
        out[group] = out.get(group,0.0) + value
    return out

def komponentengruppen(z):
    ""
    from analyse.regelkennzahlen import TEILE_V3, TEILE_V3_SI, SI_VERSIONS
    r = z.get('regel') or {}
    c = z.get('v3_komponenten') if 'v3_komponenten' in z else r.get('v3_komponenten')
    if c is not None:
        if not isinstance(c,dict): return {}
        proof = z.get('v3_komponenten_beleg') or r.get('v3_komponenten_beleg') or {}
        if proof.get('sha256') != hashlib.sha256(json.dumps(c,sort_keys=True).encode()).hexdigest(): return {}
        version = proof.get('note_version','v3.1')
        out = _komponentengruppen(c,version)
        if proof.get('gruppen') != sorted(out): return {}
        if proof.get('version') == 'v3-komponenten-v2':
            if zahl(proof.get('note_v3')) is None or abs(sum(out.values())-proof['note_v3']) > .011: return {}
        elif proof.get('version') != 'v3-komponenten-v1': return {}
        return out
    rr = r.get('regeln') or z.get('regeln') or {}
    version = z.get('note_v3_version') or r.get('note_v3_version') or r.get('version')
    groups = TEILE_V3_SI if version in SI_VERSIONS else TEILE_V3
    if not isinstance(rr,dict) or any(zahl(rr.get(k)) is None for k in groups): return {}
    return {k:float(rr[k]) for k in groups}


def board_id(z):


    fingerprints = [v for v in (z.get('board_kennung'),
                    (z.get('regel') or {}).get('board_kennung'),
                    z['regeln'].get('board_kennung') if isinstance(z.get('regeln'),dict) else None)
                    if isinstance(v, str) and v.strip()]
    if fingerprints:
        return fingerprints[0] if len(set(fingerprints)) == 1 else None
    for v in (z.get('board'), z.get('aufgabe')):
        if isinstance(v, (str, int)) and not isinstance(v, bool) and str(v).strip():
            return str(v)
    return None


def referenzzeile(z):
    ""
    return {k:z[k] for k in ('board','board_kennung','aufgabe','faktor','gueltig','note_v3','regel','tor',
                              'kennzahlen','kennz','merkmale','dichte','strom_kette','form_messung','geometrie_sha256','v3_komponenten','paar_messversion','v3_komponenten_beleg','note_v3_version') if k in z}



def torzeile(z):
    r = z.get('regel') or z.get('regeln') or {}
    declared = z.get('note_v3_version') or r.get('note_v3_version') or r.get('version')
    row = dict(z, regel=r)
    if isinstance(declared, str) and declared.startswith('v3'):
        row['ergebnis'] = dict(z.get('ergebnis') or {}, note_v3_version=declared)
    t = VT.von_zeile(row)
    sig = z.get('paar_messversion') or (z.get('regel') or z.get('regeln') or {}).get('paar_messversion')
    if not t.get('messversion') and isinstance(sig,str) and len(sig)==64:
        t['messversion'] = sig
    declared = z.get('note_v3_version') or (z.get('regel') or {}).get('note_v3_version') or (z.get('regel') or {}).get('version')
    if not str(t.get('note_version') or '').startswith('v3') and isinstance(declared,str) and declared.startswith('v3'):
        t['note_version'] = declared
    return t


def messvertrag(z):
    ""
    t = torzeile(z)
    sig = z.get('paar_messversion') or (z.get('regel') or z.get('regeln') or {}).get('paar_messversion')
    if not isinstance(sig, str) or len(sig) != 64 or not all(c in '0123456789abcdef' for c in sig): return None
    if not isinstance(t.get('messversion'), str) or not t['messversion'].strip(): return None
    if not isinstance(t.get('note_version'), str) or not t['note_version'].startswith('v3'): return None
    return dict(schema=MESSSCHEMA, vertrag=t['version'], note=t['note_version'], auswerter=t['messversion'], paar_auswerter=sig)


def trainingsvertrag(P):
    ""
    contracts = [messvertrag(z) for p in P for z in (p['a'], p['b'])]
    return contracts[0] if contracts and contracts[0] and all(c == contracts[0] for c in contracts) else None


def merkmale(z):
    ""
    from lernen.form_merkmale import gruppenfehler, kette_ok, r39_ok
    r = z.get('regel') or {}
    rr = r.get('regeln') or z.get('regeln') or {}
    if not isinstance(rr, dict): rr = {}
    sources = (z,r,rr)
    eng_ok = r39_ok(sources)
    out = {}
    def add(k, v):
        v = zahl(v)
        if v is not None: out[k] = math.copysign(math.log1p(abs(v)), v)
    for d in (z.get('kennz') or {}, z.get('kennzahlen') or {}, z.get('merkmale') or {}, rr, r, z):
        if gruppenfehler([d]): continue
        for k in NUM:
            if gruppenfehler((*sources,d), k): continue
            if k == 'breitenwechsel' and gruppenfehler((*sources,d), 'kennzahlen', 'breitenwechsel'): continue
            if k.startswith('strom_engstellen') and not r39_ok((*sources,d)): continue
            if zahl(d.get(k)) is not None: add(k, d[k])
    for name, data in [('r27', r.get('r27') or rr.get('r27') or z.get('r27') or {}),
                       ('eng', r.get('strom_engstellen') or rr.get('strom_engstellen') or {}),
                       ('kette', z.get('strom_kette') or r.get('strom_kette') or rr.get('strom_kette') or {}),
                       ('dichte', ((rr.get('p26') or {}).get('innen') or {}))]:
        if 'fehler' in data or data.get('nicht_messbar') or data.get('ausgelassen') or data.get('vollstaendig') is False: continue
        if name == 'eng' and not eng_ok: continue
        if name == 'r27' and gruppenfehler(sources, 'r27'): continue
        if name == 'kette' and (not kette_ok(data) or gruppenfehler(sources, 'strom_kette')): continue
        for k in ('spitz','stern','knoten','parallel_rest_mm','parallel_abgriff_mm',
                  'unnoetig_mm2','unnoetig_n','wirkungsgrad','glied_max','kupfer_mm2','packungsverlust'):
            add(name + '.' + k, data.get(k))
    from lernen.form_merkmale import werte as formwerte
    for k, v in formwerte(z).items(): add('form.' + k, v)

    for k, v in sorted((z.get('v3_komponenten') or r.get('v3_komponenten') or {}).items()):
        add('v3.' + k, v)
    for k,v in komponentengruppen(z).items(): add('v3_gruppe.'+k,v)
    return out


def _sid(z):
    return str(z.get('sample_id') or z.get('bild') or '')

def lesen(pfade, nutzer=None):
    ""
    G = {}; stats = collections.Counter(); users = {}
    for p in pfade:
        if not Path(p).is_file(): raise FileNotFoundError(p)
        for line in Path(p).read_text().splitlines():
            try: z = json.loads(line)
            except ValueError: stats['defekt'] += 1; continue
            stats['urteile'] += 1
            if not isinstance(z, dict) or not z.get('board') or not _sid(z) or not familie(z):
                stats['identitaet_fehlt'] += 1; continue
            z = dict(z, _familie=familie(z))
            key = (str(z['board']), str(z.get('bild') or _sid(z)), str(z.get('richter') or '?'))
            G[key] = z
    if nutzer and not Path(nutzer).is_file(): raise FileNotFoundError(nutzer)
    if nutzer:
        for line in Path(nutzer).read_text().splitlines():
            try: u = json.loads(line)
            except ValueError: stats['nutzer_defekt'] += 1; continue
            stats['nutzer_abgaben'] += 1
            for e in u.get('eintraege', []):
                if not e.get('bild'): continue

                key=(str(u['board']),str(e['bild']),'nutzer')
                z = dict(G.get(key) or {}, **e)
                z.update(board=u['board'], richter='nutzer', gueltig=(e.get('kennz') or {}).get('gueltig',z.get('gueltig')),
                         gruppe=u['board'], nutzer_bild=str(Path.home() / 'Library/Caches/Autorouter/nutzer_noten' / e['bild']))
                z['_familie'] = familie(z)
                users[key] = z
        G.update(users)
    stats['nutzer_kandidaten'] = len(users)
    Z = list(G.values())


    parents = {}
    def root(f):
        parents.setdefault(f, f)
        while parents[f] != f:
            parents[f] = parents[parents[f]]; f = parents[f]
        return f
    bases = {}
    for z in Z:
        board = str(z['board'])
        base = z['_familie'] if board.startswith('erf_') else familie(dict(board=board))
        bases[board] = base
        a,b = root(base),root(z['_familie'])
        if a != b: parents[b] = a
    stems = collections.defaultdict(set)
    for b in bases.values(): stems[root(b)].add(b)
    aliases = collections.defaultdict(set)
    for a in parents: aliases[root(a)].add(a)
    for z in Z:
        group = root(bases[str(z['board'])])
        z['_familie'] = min(stems[group]); z['_familie_aliases'] = sorted(aliases[group])
    stats['kandidaten_gutachter'] = len(Z)
    return Z, dict(stats)


def feste_familien(Z,d):
    ""
    p=Path(d)/'familien.json';reg=json.loads(p.read_text()) if p.is_file() else {}
    groups=collections.defaultdict(list)
    for z in Z: groups[z['_familie']].append(z)
    for f,zs in sorted(groups.items()):
        keys=sorted(set(a for z in zs for a in z.get('_familie_aliases',[f])) | {'board:'+str(z['board']) for z in zs})
        known={reg[k]['familie'] for k in keys if k in reg}
        conflict=len({zurueck(k) for k in known})>1 or any(reg[k].get('konflikt') for k in keys if k in reg)
        target=min((k for k in known if zurueck(k)),default=min(known,default=f))
        for k in keys:reg[k]=dict(familie=target,konflikt=bool(conflict))
        for z in zs:z['_familie']=target;z['_split_konflikt']=bool(conflict)
    atomar(p,reg)
    return Z


def paare(Z, ziel='note'):
    groups = collections.defaultdict(list); out = []
    for z in Z:
        if note_v3(z) is None: continue
        if z.get('gueltig') is not True and not (z.get('richter') == 'nutzer' and z.get('gueltig') is False): continue
        identity = board_id(z)
        if identity is None: continue
        groups[(identity, str(z.get('richter') or '?'))].append(z)
    for (board, judge), zs in sorted(groups.items()):
        for a, b in itertools.combinations(sorted(zs, key=_sid), 2):
            if _sid(a) == _sid(b) or a['_familie'] != b['_familie']: continue
            if a.get('gueltig') != b.get('gueltig'): continue
            ra, rb = zahl(a.get('rang')), zahl(b.get('rang'))
            na, nb = zahl(a.get(ziel)), zahl(b.get(ziel))
            if judge == 'nutzer' and ziel == 'note' and ra is not None and rb is not None:
                y = 1 if ra < rb else 0 if rb < ra else None
            elif na is not None and nb is not None and 1 <= na <= 10 and 1 <= nb <= 10:
                y = 1 if na > nb else 0 if nb > na else None
            else: y = None
            if y is None: continue
            out.append(dict(a=a,b=b,y=y,familie=a['_familie'],board=board,richter=judge,
                            gewicht=5.0 if judge == 'nutzer' else 1.0))
    return out


def differenzen(P, keys):
    import numpy as np
    X = []; B = []
    for p in P:
        a,b = merkmale(p['a']),merkmale(p['b'])

        X.append([a[k]-b[k] if k in a and k in b else 0.0 for k in keys])
        B.append(basis(p['a'])-basis(p['b']))
    return np.asarray(X, dtype=float).reshape(len(P), len(keys)), np.asarray(B)


def paargewichte(P):
    ""
    import numpy as np
    w = np.array([5.0 if p.get('richter') == 'nutzer' else 1.0 if p.get('richter') else p.get('gewicht',1.0) for p in P])
    return w / w.sum() if len(w) else w


def lernen(P, staerke=1.0):
    import numpy as np
    from scipy.optimize import minimize
    from scipy.special import expit
    keys = sorted(set().union(*(set(merkmale(p['a'])) & set(merkmale(p['b'])) for p in P)))
    X,b = differenzen(P,keys)

    w = paargewichte(P)
    scale = np.sqrt(w @ (X*X)) if len(P) else np.ones(len(keys))
    scale[scale < 1e-8] = 1
    X = X/scale
    y = np.array([p['y'] for p in P])
    def loss(theta):
        t = b + X@theta
        return float(w @ (np.logaddexp(0,t)-y*t) + staerke*.5*(theta@theta)), X.T@(w*(expit(t)-y))+staerke*theta
    fit = minimize(loss,np.zeros(len(keys)),jac=True,method='L-BFGS-B') if len(P) and keys else None
    if fit is not None and not fit.success: raise RuntimeError('Paaroptimierung: ' + fit.message)
    return dict(version=VERSION, keys=keys, scale=scale.tolist(), koeff=fit.x.tolist() if fit is not None else [],
                staerke=staerke, feste_effekte={j:0.0 for j in sorted({p['richter'] for p in P})},
                effekt_hinweis='Additive Gutachtereffekte fallen im bedingten Same-Board/Same-Judge-Vergleich exakt weg.')


def prognose(M,P,alpha=None):
    import numpy as np
    X,B = differenzen(P,M['keys'])
    return B + (M.get('alpha',1) if alpha is None else alpha)*(X/np.array(M['scale']))@np.array(M['koeff'])


def messen(M,P,bootstrap=2000,seed=SEED,scores=None):
    import numpy as np
    if not P: return dict(n=0,familien=0,treffer=None,v3=None,differenz95=None,pro_familie={})
    pp = prognose(M,P) if scores is None else np.asarray(scores); _,bb = differenzen(P,[])

    y = np.array([p['y'] for p in P])
    hit = lambda scores: np.where(abs(scores)<1e-12,.5,((scores>0)==y).astype(float))
    h,v=hit(pp),hit(bb); all_weights=paargewichte(P); fs=sorted({p['familie'] for p in P}); F={}
    for f in fs:
        ix=np.array([p['familie']==f for p in P]); w=all_weights[ix]
        F[f]=dict(n=int(ix.sum()),treffer=float(np.average(h[ix],weights=w)),v3=float(np.average(v[ix],weights=w)))
    w = paargewichte(P)

    masses = np.array([w[np.array([p['familie']==f for p in P])].sum() for f in fs])
    dh = np.array([F[f]['treffer']-F[f]['v3'] for f in fs])
    sample = np.random.default_rng(seed).integers(len(fs),size=(bootstrap,len(fs)))
    ci = np.quantile((dh[sample]*masses[sample]).sum(axis=1)/masses[sample].sum(axis=1),[.025,.975]).tolist()
    return dict(n=len(P),familien=len(F),treffer=float(w@h),v3=float(w@v),
                treffer_familien=float(np.mean([q['treffer'] for q in F.values()])),
                v3_familien=float(np.mean([q['v3'] for q in F.values()])),
                treffer_paare=float(h.mean()),v3_paare=float(v.mean()),differenz95=ci,
                pro_familie=F,gewicht='Je Paar KI 1, Nutzer 5; KI mit Familien-Clustern')


def freigabe(m, blind=False):
    reasons=[]
    if m['n'] < 60 or m['familien'] < 3: reasons.append('unter 60 Testpaare / 3 Testfamilien')
    if not blind: reasons.append('keine unabhaengige blinde Kontrollmenge')
    if m['treffer'] is None or m['v3'] is None: reasons.append('keine auswertbare Kontrollmessung')
    elif m['treffer'] < m['v3'] or m.get('treffer_paare',m['treffer']) < m.get('v3_paare',m['v3']): reasons.append('schlechter als v3')
    families = m.get('pro_familie') or {}
    if len(families) != m['familien']: reasons.append('Familiennachweise unvollstaendig')
    for f, q in sorted(families.items()):
        a, b = zahl(q.get('treffer')), zahl(q.get('v3'))
        if a is None or b is None or a < b: reasons.append('Familienregression/unbekannt: ' + f)
    if not (m['differenz95'] and m['differenz95'][0] > 0):
        reasons.append('kein positives Familien-KI des Unterschieds zu v3')
    return not reasons,reasons


def eichung(P, zielpfad, gewichte=None):
    ""
    import numpy as np
    from scipy.optimize import minimize
    from scipy.special import expit
    src=Path(gewichte or R/'analyse/metriken_gewichte.json'); G=json.loads(src.read_text())
    if Path(zielpfad).resolve()==src.resolve(): raise ValueError('Eichung darf Standardgewichte nicht ueberschreiben')
    components=komponentengruppen
    def darstellbar(z):
        groups=components(z)
        c=z.get('v3_komponenten') or (z.get('regel') or {}).get('v3_komponenten') or {}
        version=(z.get('v3_komponenten_beleg') or (z.get('regel') or {}).get('v3_komponenten_beleg') or {}).get('note_version')
        return bool(groups) and not groups.get('sonst') and not c.get('board.si_zusatz') and (version is None or version==G.get('version'))
    pp=[p for p in P if darstellbar(p['a']) and darstellbar(p['b'])]
    keys=sorted(set().union(*(set(components(p['a'])) | set(components(p['b'])) for p in pp))) if pp else []
    X=np.array([[-(components(p['a']).get(k,0)-components(p['b']).get(k,0))/max(1,note_v3(p['a'])+note_v3(p['b'])) for k in keys] for p in pp])
    multipliers={}
    if keys:
        y=np.array([p['y'] for p in pp]); w=paargewichte(pp)
        def objective(t):
            v=X@t
            return float(w@(np.logaddexp(0,v)-y*v)+.1*np.sum((t-1)**2)), X.T@(w*(expit(v)-y))+.2*(t-1)
        fit=minimize(objective,np.ones(len(keys)),jac=True,bounds=[(0,5)]*len(keys),method='L-BFGS-B')
        if not fit.success: raise RuntimeError('Eichung: '+fit.message)
        multipliers=dict(zip(keys,fit.x.tolist()))
        for key,factor in multipliers.items():
            from analyse.regelkennzahlen import TEILE_V3_SI
            for cls in G['klassen'].values():
                for term in TEILE_V3_SI.get(key,()):
                    if term in cls['g']: cls['g'][term] *= factor
            if key == 'verlust':
                for term in ('g','g_max'): G['gruppen'][term] *= factor
    G['_paar_eichung']=dict(version=VERSION,paare=len(pp),nicht_darstellbar=len(P)-len(pp),multiplikatoren=multipliers,freigegeben=False,
                           quelle_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
                           grund='Separate H6-/Familienpruefung erforderlich' if pp else 'Vollstaendige H12-Komponentensummen fehlen')
    atomar(zielpfad,G); return G['_paar_eichung']


def trainieren(pfade,ordner,nutzer=None,seed=SEED):
    ""
    import numpy as np
    from sklearn.model_selection import GroupKFold
    from threadpoolctl import threadpool_limits
    import fcntl
    d=Path(ordner); d.mkdir(parents=True,exist_ok=True)
    with (d/'.training.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        t0=time.process_time(); Z,stats=lesen(pfade,nutzer)
        if not Z: raise ValueError('Keine Eingabeurteile')
        Z=feste_familien(Z,d); result={}
        with threadpool_limits(limits=1):
            for ziel in ('note','strom_note'):
                P=paare(Z,ziel); historical_n=len(P)
                contracts=collections.defaultdict(list); current_sig=messsignatur()
                for p in P:
                    c=trainingsvertrag([p])
                    if c and c['paar_auswerter']==current_sig:
                        contracts[json.dumps(c,sort_keys=True)].append(p)
                if contracts:
                    P=max(sorted(contracts.items()),key=lambda kv:len(kv[1]))[1]
                tr=[p for p in P if not zurueck(p['familie'])]; te=[p for p in P if zurueck(p['familie'])]
                fs=sorted({p['familie'] for p in tr}); configs=[]
                if len(fs)>=3:
                    folds=list(GroupKFold(min(4,len(fs))).split(tr,groups=[p['familie'] for p in tr]))
                    for strength in (.01,.1,1.0):
                        predictions={a:[] for a in (0,.1,.25,.5,1)}
                        truth=[]; families=[]; weights=[]
                        for it,iv in folds:
                            model=lernen([tr[i] for i in it],strength)
                            pv=[tr[i] for i in iv]
                            for a in predictions: predictions[a].extend(prognose(model,pv,a).tolist())
                            truth.extend(p['y'] for p in pv); families.extend(p['familie'] for p in pv); weights.extend(p['gewicht'] for p in pv)
                        for a,scores in predictions.items():
                            h=np.where(np.abs(scores)<1e-12,.5,(np.array(scores)>0)==truth)
                            value=float(np.average(h,weights=weights))
                            configs.append((value,-a,strength,a))
                chosen=max(configs,default=(0,0,1,0),key=lambda q:(q[0],q[1],q[2]))
                M=lernen(tr,chosen[2]); M['alpha']=chosen[3]; M['ziel']=ziel
                test=messen(M,te,seed=seed)
                from lernen.paar_blind import kontrollpaare
                control, blind_audit = kontrollpaare(te)
                control_test=messen(M,control,seed=seed)
                ok,reasons=freigabe(control_test,bool(control))
                contract = trainingsvertrag(tr + control)
                if contract is None:
                    ok = False; reasons.append('Messschema/Auswertersignatur fehlt oder gemischt')
                M['messvertrag'] = contract
                coverage=collections.Counter(k for z in Z for k in merkmale(z))
                meta=dict(version=VERSION,messvertrag=contract,art='paar',ziel=ziel,freigegeben=ok,gruende=reasons,test=test,kontrolle=control_test,
                          blind_audit=blind_audit, training_cv=dict(treffer=chosen[0],alpha=M['alpha'],staerke=M['staerke']),
                          training_familien=fs,test_familien=sorted({p['familie'] for p in te}),paare=len(P),
                          training_boards=sorted({p['board'] for p in tr}),test_boards=sorted({p['board'] for p in te}),
                          historische_paare=historical_n, messmodus='aktueller_Vertrag' if contracts else 'historischer_Pilot',
                          training_paare=len(tr),test_paare=len(te),nutzer_paare=sum(p['richter']=='nutzer' for p in P),
                          coverage=dict(coverage),daten=stats,seed=seed)
                ident=hashlib.sha256(json.dumps(dict(M=M,labels=[(_sid(p['a']),_sid(p['b']),p['richter'],p['y'],p['gewicht']) for p in P]),sort_keys=True).encode()).hexdigest()[:20]
                mp=d/(ziel+'_'+ident+'.json'); atomar(mp,M)
                meta.update(id=ident,datei=mp.name,sha256=hashlib.sha256(mp.read_bytes()).hexdigest(),cpu_s=round(time.process_time()-t0,3),last=os.getloadavg()[0])
                atomar(d/(ziel+'_herausforderer.json'),meta)

                meister=d/(ziel+'_meister.json'); ersetzt=True
                if meister.exists():
                    old=json.loads(meister.read_text())
                    if old.get('freigegeben'):
                        ersetzt=False
                        if ok and old.get('messvertrag') == contract and not set(old['training_familien']) & {p['familie'] for p in te} and not set(old.get('training_boards',[])) & {p['board'] for p in te}:
                            am=laden_modell(d,old); alt=messen(am,control,seed=seed)
                            ds=np.array([control_test['pro_familie'][f]['treffer']-alt['pro_familie'][f]['treffer'] for f in control_test['pro_familie']])
                            ci=np.quantile(np.random.default_rng(seed).choice(ds,(2000,len(ds))).mean(axis=1),[.025,.975]).tolist()
                            meta['meister_differenz95']=ci; ersetzt=ci[0]>0
                meta['ersetzt']=ersetzt
                atomar(d/(ziel+'_herausforderer.json'),meta)
                if ersetzt: atomar(meister,meta)
                result[ziel]=meta
                if ziel=='note':
                    calibration=eichung(tr,d/'metriken_gewichte.json')
                    cp=[p for p in control if komponentengruppen(p['a']) and komponentengruppen(p['b'])]
                    weights=calibration['multiplikatoren']
                    scores=[-sum(weights.get(k,1)*(komponentengruppen(p['a']).get(k,0)-komponentengruppen(p['b']).get(k,0)) for k in set(komponentengruppen(p['a'])) | set(komponentengruppen(p['b'])))/max(1,note_v3(p['a'])+note_v3(p['b'])) for p in cp]
                    calibration['test']=messen(None,cp,seed=seed,scores=np.asarray(scores))
                    calibration['paar_tor'],calibration['paar_gruende']=freigabe(calibration['test'],bool(cp))

                    result['eichung']=calibration
                    candidate=json.loads((d/'metriken_gewichte.json').read_text());candidate['_paar_eichung']=calibration
                    atomar(d/'metriken_gewichte.json',candidate)
        atomar(d/'letztes_training.json',result); return result


def laden_modell(d,meta):
    p=Path(d)/meta['datei']
    if p.parent.resolve()!=Path(d).resolve() or hashlib.sha256(p.read_bytes()).hexdigest()!=meta['sha256']:
        raise ValueError('Paarmodell-Pfad/Hash stimmt nicht')
    M=json.loads(p.read_text())
    if M.get('version')!=VERSION: raise ValueError('Paarmodell-Version')
    keys,scale,coef=M.get('keys'),M.get('scale'),M.get('koeff')
    if not all(isinstance(v,list) for v in (keys,scale,coef)) or len(keys)!=len(scale) or len(keys)!=len(coef):
        raise ValueError('Paarmodell-Dimensionen')
    if any(not isinstance(k,str) for k in keys) or len(set(keys))!=len(keys): raise ValueError('Paarmodell-Merkmalsliste')
    if any(zahl(v) is None or v<=0 for v in scale) or any(zahl(v) is None for v in coef):
        raise ValueError('Paarmodell-Koeffizienten/Skala')
    if zahl(M.get('alpha')) is None or not 0<=M['alpha']<=1: raise ValueError('Paarmodell-Korrekturfaktor')
    if M.get('messvertrag') != meta.get('messvertrag'): raise ValueError('Paarmodell-Messvertrag')
    return M


class Dienst:
    def __init__(self,daten=None,modell=MODELL,neu=20,bilder=False,epochen=0,cpu=300,nutzer=None,ziel='note'):
        self.daten=Path(daten or R/'lernen/daten/gutachten.jsonl'); self.modell=Path(modell)
        self.nutzer=Path(nutzer or self.daten.with_name('nutzer_noten.jsonl'))
        self.neu=neu; self.cpu=cpu; self.ziel=ziel; self.lock=threading.RLock()
        self.meta={}; self.M=None; self.stamp=None; self.thread=None; self.pruefzeit=0; self.gesehen=None
        self.urteile = -1; self.nutzerstand = None
        last = self.modell/'letztes_training.json'
        if last.is_file():
            try: self.urteile = json.loads(last.read_text())['note']['daten']['urteile']
            except (ValueError,KeyError): pass

    def laden(self):
        p=self.modell/(self.ziel+'_meister.json')
        if not p.is_file(): self.M=None; self.meta={}; self.stamp=None; return
        stamp=p.stat().st_mtime_ns
        if stamp!=self.stamp:
            meta=json.loads(p.read_text()); M=laden_modell(self.modell,meta)
            self.M,self.meta,self.stamp=M,meta,stamp

    def anwenden(self,z,bild=None):
        with self.lock:
            z.update(paar_freigegeben=False,paar_status='keine_noten',paar_modell=None,paar_score=None,
                     paar_modell_pfad=str(self.modell.resolve()), paar_modell_datei=None)
            try:
                self.laden()
                if self.M and basis(z) is not None:
                    features = merkmale(z)
                    score = basis(z) + self.M.get('alpha',1) * sum(features.get(k,0)/s*c for k,s,c in
                                zip(self.M['keys'],self.M['scale'],self.M['koeff']))
                    applicable = self.anwendbar(z)
                    z.update(paar_modell=self.meta['id'], paar_modell_datei=str((self.modell/self.meta['datei']).resolve()),
                             paar_freigegeben=applicable, paar_status='freigegeben' if applicable else 'pilot', paar_score=score)
            except (OSError,ValueError,KeyError): z['paar_status']='fehler'
        return z

    def anwendbar(self, z):
        contract = messvertrag(z)
        return bool(board_id(z) is not None and self.M and self.meta.get('freigegeben') is True and contract and
                    self.M.get('messvertrag') == self.meta.get('messvertrag') == contract)

    def vergleich(self,a,b):
        ""
        if board_id(a) is None or board_id(a) != board_id(b):
            return dict(besser=False,nicht_schlechter=False,grund='Board fehlt/verschieden',paar=False)
        gate=VT.vergleich(torzeile(a),torzeile(b))
        answer=dict(besser=False,nicht_schlechter=gate['nicht_schlechter'],grund=gate['grund'],paar=False)
        if not a.get('gueltig') or not b.get('gueltig') or not gate['nicht_schlechter']: return answer
        na,nb=note_v3(a),note_v3(b)
        if na is None or nb is None or nb>na+1e-9: return answer
        try:
            with self.lock:
                self.laden()
                if self.anwendbar(a) and self.anwendbar(b) and set(merkmale(a)) == set(merkmale(b)):
                    score=float(prognose(self.M,[dict(a=b,b=a)])[0])
                    answer.update(besser=score>1e-12,grund='Paarmodell nach hartem Tor',paar=True,differenz=score)
                    return answer
        except (OSError,ValueError,KeyError): pass
        answer.update(besser=gate['besser'],grund='v3/Vertrag (Paarmodell nicht freigegeben)')
        return answer

    def zusatz(self,z,referenz,gewicht=.03):
        ""
        if not z.get('gueltig') or not referenz or not referenz.get('gueltig'): return 0.0
        if board_id(z) is None or board_id(z) != board_id(referenz): return 0.0
        try:
            with self.lock:
                self.laden()
                if not self.anwendbar(z) or not self.anwendbar(referenz) or basis(z) is None or basis(referenz) is None: return 0.0

                if not VT.vergleich(torzeile(referenz), torzeile(z))['nicht_schlechter']: return 0.0
                if set(merkmale(z))!=set(merkmale(referenz)): return 0.0
                score=float(prognose(self.M,[dict(a=z,b=referenz)])[0])
                return -min(.05,max(0,gewicht))*math.tanh(score/2)
        except (OSError,ValueError,KeyError): return 0.0

    def belohnung(self,z,basis,gewicht=.03):
        if basis is None or basis>=2 or not z.get('gueltig') or z.get('code_geaendert'): return basis
        ref=z.get('paar_referenz')
        extra = self.zusatz(z,ref,gewicht)
        if extra == 0.0: return basis
        value = min(1.9999,max(0,basis+extra))

        if z.get('aufgabe') is not None and (z.get('regel') or {}).get('bewertungsziel') != 'minimax':
            sx = zahl((z.get('regel') or {}).get('strom_x_max'))
            if sx is None: return basis
            value = min(1.4999,max(1,value)) if sx <= 1 else min(1.9999,max(1.5001,value))
        return value

    def pruefen(self):
        with self.lock:
            if time.monotonic()-self.pruefzeit<30 or self.thread and self.thread.is_alive(): return
            self.pruefzeit=time.monotonic()
            from gemeinsam import plaetze as PL
            if PL.knapp(): return
            stamp=tuple((p.stat().st_size,p.stat().st_mtime_ns) if p.is_file() else None for p in (self.daten,self.nutzer))
            if stamp==self.gesehen or not self.daten.is_file(): return
            count=sum(1 for line in self.daten.open() if line.strip())
            if self.urteile>=0 and count-self.urteile<self.neu and stamp[1]==self.nutzerstand: return
            def train():
                self.modell.mkdir(parents=True,exist_ok=True)
                cmd=[str(R/'.venv/bin/python'),str(R/'lernen/richter_paar.py'),'trainieren','daten='+str(self.daten),
                     'modell='+str(self.modell)]
                if self.nutzer.is_file(): cmd.append('nutzer='+str(self.nutzer))
                run=PL.lauf(cmd,cpu_s=self.cpu,log=str(self.modell/'training.log'),cwd=str(R))
                atomar(self.modell/'training_lauf.json',run)
                if run['zustand']=='ok':
                    self.gesehen=stamp; self.urteile=count; self.nutzerstand=stamp[1]
            self.thread=threading.Thread(target=train,daemon=True); self.thread.start()


def qualitaets_belohnung(z,basiswert):
    ""
    if basiswert is None or basiswert>=2 or not z.get('gueltig'): return basiswert
    r=z.get('regel') or {}; chain=z.get('strom_kette') or r.get('strom_kette') or {}
    from lernen.form_merkmale import kette_ok, gruppenfehler
    if not kette_ok(chain) or gruppenfehler((z, r, r.get('regeln') or {}, z.get('regeln') or {}), 'strom_kette', 'strom_x_max'):
        return basiswert
    eta,glied=zahl(chain.get('wirkungsgrad')),zahl(chain.get('glied_max'))
    sx=zahl(r.get('strom_x_max'))
    if chain.get('ausgelassen') or 'fehler' in chain or eta is None or eta<=0 or glied is None or sx is None: return basiswert
    worst=max(sx,1/eta,glied,1)
    severity=max(0,1-1/worst)

    n=note_v3(z); tail=n/(1+n) if n is not None else 1
    if sx>1: return 1.5001+.4599*severity+.03*tail
    return 1+.4599*severity+.03*tail


if __name__=='__main__':
    kw=dict(a.split('=',1) for a in sys.argv[2:] if '=' in a)
    if sys.argv[1:2]!=['trainieren']: raise SystemExit(__doc__)
    nutzerpfad=Path(kw.get('nutzer',R/'lernen/daten/nutzer_noten.jsonl')).expanduser()
    result=trainieren([Path(p).expanduser() for p in kw.get('daten',str(R/'lernen/daten/gutachten.jsonl')).split(',')],
                      Path(kw.get('modell',MODELL)).expanduser(),nutzerpfad if 'nutzer' in kw or nutzerpfad.is_file() else None,int(kw.get('seed',SEED)))
    print(json.dumps({k:{kk:vv for kk,vv in v.items() if kk not in ('coverage','test','training_familien','test_familien')} for k,v in result.items()},indent=1))
