""
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
VERSION = 'meister-kosten-v1'
BELOHNUNG_VERSION = 'meister-dicht-native-v3'
ARTEN = ('signal', 'schnell', 'analog', 'strom', 'versorgung', 'sw', 'gnd')

RAUM = {
    'pfmul': (1.1, 1.2, 1.25, 1.3, 1.4, 1.5),
    'pf0': (.4, .6, .8, 1., 1.5, 2.), 'hinc': (.4, .7, 1., 1.4, 2.),
    'ziel_ab': (0, 5, 10, 20, 30, 40), 'alle_nah': (0., 1., 2., 3., 5., 8.),
    'kette_umweg': (1.3, 1.5, 2., 2.5, 3.),
    'kreuz': (0., 1., 2., 3., 4., 6.), 'trenn': (2., 2.5, 3., 3.5, 4.),
    'via_kosten': (.3, .45, .6, .8, 1., 1.2),
    'dicht': (0., .05, .1, .2, .4), 'dicht_innen': (0., .05, .1, .2),
    'trenn_w': (.5, .75, 1., 1.5, 2.),
    'fein': (0, 1),
    'kosten_eng': (0., .1, .3, .6, 1., 2.),
    'form': ('45r','organisch','je_netzart'),
    'placer_stil': ('alt','insel'), 'router_stil': ('alt','konvergenz'),
}
for _art in ARTEN:
    for _prefix in ('kosten_via_', 'kosten_innen_'):
        RAUM[_prefix + _art] = (.5, .7, .85, 1., 1.15, 1.4, 1.8, 2.)

    RAUM['kosten_rang_' + _art] = (.5, .75, 1., 1.25, 1.5, 2.)
KATEGORIEN={'form','placer_stil','router_stil'}
PLACER = {'kreuz', 'trenn', 'via_kosten', 'dicht', 'dicht_innen', 'trenn_w', 'fein','placer_stil'}
ROUTER = set(RAUM) - PLACER
DEFAULT = dict(pfmul=1.25, pf0=1., hinc=1., ziel_ab=20, alle_nah=0., kette_umweg=2.,
               kreuz=2., trenn=2., via_kosten=.6, dicht=0., dicht_innen=0., trenn_w=1., fein=1,kosten_eng=0.)
DEFAULT.update({key: 1. for key in RAUM if key.startswith(('kosten_via_','kosten_innen_','kosten_rang_'))})
DEFAULT.update(form='45r',placer_stil='alt',router_stil='alt')


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def datei():
    return Path(os.environ.get('AUTOROUTER_MEISTER') or
                Path(os.environ.get('LERNKURVE_HAUPT') or ROOT) / 'lernen/modell/meister_cfg.json')


def parameter(cfg):
    ""
    out = {}
    for key, value in cfg.items():
        if key not in RAUM:
            raise ValueError('Unbekannte Meisterkosten: ' + key)
        if key in KATEGORIEN:
            if value not in RAUM[key]:raise ValueError('Unbekannter Meisterstil: '+key)
            out[key]=value;continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Nichtendliche Meisterkosten: ' + key)
        if not min(RAUM[key]) <= value <= max(RAUM[key]):
            raise ValueError('Meisterkosten ausserhalb Suchraum: ' + key)
        if key in ('ziel_ab', 'fein') and int(value) != value:
            raise ValueError(key+' muss ganzzahlig sein')
        out[key] = value
    return out


def cli(key, value):
    return str(int(value)) if key in ('ziel_ab', 'fein') else str(value)


def pruefe_cli(args):
    ""
    parameter({k: float(v) for k,v in args.items() if k.startswith('kosten_')})


def werkzeugoptionen(values, tool):
    ""
    keys=ROUTER if tool=='router' else PLACER
    options={k:cli(k,v) for k,v in values.items() if k in keys and k not in KATEGORIEN}
    if tool=='router':
        if 'form' in values:options['stil']='45' if values['form']=='45r' else values['form']
        if values.get('router_stil')=='konvergenz':
            options.update(konfliktort='exakt',quellfrei='1',konvergenz='lokal',pfmax='1000',kurz_via='8')
    elif values.get('placer_stil')=='insel':
        options.update(insel_physik='1',pin_nachlegal='1',bga_gegen='1',bedien_neu_frei='1')
    return options


def klasse(A):
    v = str(A.get('version', '10')).split('+')[0]
    bga = any('BGA' in b.upper() for b in A.get('bloecke', []))
    return 'v%s:L%d_%s_s%d' % (v, A['lagen'], 'bga' if bga else 'qfn',
                              int(round(float(A.get('stufe', 0)) / 2)) * 2)


def boardklasse(B):
    ""
    layers = B.get('lagen', 4)
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


def lesen(path=None):
    path = Path(path or datei())
    if not path.exists(): return dict(version=VERSION, generation=0, klassen={}, verlauf=[])
    doc = json.loads(path.read_text())
    if doc.get('version') != VERSION: raise ValueError('Meisterversion ungueltig')
    for key, entry in doc['klassen'].items():
        if not re.fullmatch(r'v\d+:L\d+_(bga|qfn)_s\d+', key): raise ValueError('Meisterklasse')
        parameter(entry['parameter'])
        if not entry.get('beleg_sha'): raise ValueError('Meister ohne Torbeleg')
    return doc


def einfrieren(path):
    ""
    doc=lesen();path=Path(path).resolve();path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix=path.name+'.',suffix='.neu',delete=False) as f:
        json.dump(doc,f,sort_keys=True,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
        tmp=Path(f.name)
    os.replace(tmp,path)
    return dict(pfad=str(path),modell_sha=sha(doc),generation=doc['generation'])


def auswahl(doc, kl, exakt=False):
    if kl in doc['klassen']: return kl, doc['klassen'][kl]
    if exakt: return kl, None
    v, suffix = kl.split(':'); family, stage = suffix.rsplit('_s', 1)
    candidates = [k for k in doc['klassen'] if k.split(':')[1].rsplit('_s', 1)[0] == family]
    if not candidates: return kl, None
    key = min(candidates, key=lambda k: (k.split(':')[0] != v,
                                         abs(int(k.rsplit('_s', 1)[1])-int(stage)), k))
    return key, doc['klassen'][key]


def anwenden(args, B, werkzeug):
    ""
    if str(args.get('std', '1')) == '0' or str(args.get('meister', '1')) == '0': return None
    path = Path(args.get('meister_cfg') or datei())
    doc = lesen(path)
    requested = args.get('meister_klasse') or boardklasse(B)
    key, entry = auswahl(doc, requested, exakt='meister_klasse' in args)
    if entry is None: return None
    keys = ROUTER if werkzeug == 'router' else PLACER
    values = {k: v for k, v in entry['parameter'].items() if k in keys}
    args.update(werkzeugoptionen(values,werkzeug))
    info = dict(version=VERSION, generation=doc['generation'], klasse=key,
                angefragt=requested, parameter=values, modell_sha=sha(doc), beleg_sha=entry['beleg_sha'])
    print('Meister ' + json.dumps(info, sort_keys=True), flush=True)
    return info


def zahlen(z):
    k = z.get('kennzahlen') or z.get('ergebnis') or {}
    out = []
    for key in ('konflikte', 'offen'):
        value = k.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            return None
        out.append(value)
    native=z.get('native_meister')
    if native is not None:


        opened=native.get('offen')
        if isinstance(opened,bool) or not isinstance(opened,(int,float)) or not math.isfinite(opened) or opened<0:
            return None
        out[1]=max(out[1],opened)
    return out


def native_fehler(z):
    value = (z.get('native_meister') or {}).get('fehler',
        (z.get('ergebnis') or {}).get('drc_fehler', z.get('drc_fehler')))
    return value if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None


def gesamtrest(z):
    ""
    rest, drc = zahlen(z), native_fehler(z)
    return sum(rest) + drc if rest is not None and drc is not None else None


def belohnung(z):
    ""
    if z.get('nicht_geroutet') or z.get('generatorfehler'): return None
    if z.get('gueltig') is None:


        curve = z.get('konflikt_verlauf') or []
        if z.get('zustand') != 'fruehabbruch' or not curve: return None
        values = curve[-1][:2]
        if len(values) != 2 or any(type(v) not in (int,float) or not math.isfinite(v) or v < 0 for v in values): return None
        return (1., sum(math.log1p(v) for v in values), 1., *map(math.log1p, values))
    rest = zahlen(z)
    if rest is None:


        collisions=(z.get('placer_kennzahlen') or {}).get('kollisionen')
        if z.get('zustand')=='placer_kollision' and isinstance(collisions,(int,float)) and math.isfinite(collisions) and collisions>0:
            return (1., 1e6+collisions, 1e6, 1e6, 0.)
        return None
    if z['gueltig'] is True:
        native=z.get('native_meister')
        if not native or native_fehler(z)!=0 or native.get('offen')!=0 or any(rest):return None
        note = z.get('note_v3')
        if not isinstance(note, (int, float)) or not math.isfinite(note): return None
        return (0., 0., 0., 0., math.log1p(max(0., note)))
    drc = native_fehler(z)

    dense = sum(math.log1p(v) for v in rest) + (math.log1p(drc) if drc is not None else 0.)
    return (1., dense, float(drc is None), math.log1p(rest[0]), math.log1p(rest[1]))


def zeilenklasse(z):
    try:
        return klasse(dict(version=z.get('version', '10'), lagen=z['lagen'], bloecke=z['bloecke'],
                           stufe=z.get('stufe_aufgabe', z.get('stufe', 0))))
    except (KeyError, TypeError, ValueError): return None

def familie(kl):
    ""
    return kl.split(':', 1)[1].rsplit('_s', 1)[0]


def wirkung(rows):
    ""
    data = []
    seen = set()
    for r in rows:
        kl = zeilenklasse(r); costs = r.get('cfg', {}).get('kosten')
        rest = gesamtrest(r)
        if not kl or not costs or rest is None or r.get('zustand') != 'ok': continue
        try: costs = parameter(costs)
        except ValueError: continue
        cfg = r['cfg']; design = (kl, r.get('designfamilie', r.get('aufgabe')))
        if design[1] is None: continue
        identity = (design, sha(costs), r.get('code_sha256'), cfg.get('seed'))
        if identity in seen: continue
        seen.add(identity)
        stratum = (kl, r.get('code_sha256'), cfg.get('budget', 'cpu'), cfg.get('arbeit_max'),
                   (r.get('budget') or {}).get('iter'))
        data.append((stratum, design, cfg, costs, math.log1p(rest)))
    centers = defaultdict(list)
    for st, _, _, _, value in data: centers[st].append(value)
    centers = {st: median(vs) for st, vs in centers.items()}
    result = []
    for key in sorted(RAUM):
        groups = defaultdict(dict); pairs = defaultdict(dict)
        for st, design, cfg, costs, value in data:
            if key not in costs: continue
            groups[costs[key]][design] = value - centers[st]
            fixed = dict(cfg); fixed['kosten'] = {k:v for k,v in costs.items() if k != key}
            pairs[(design, sha(fixed), st)][costs[key]] = value
        means = [sum(g.values())/len(g) for g in groups.values() if len(g) >= 2]
        paired = [max(v.values())-min(v.values()) for v in pairs.values() if len(v) >= 2]
        effect = max(means)-min(means) if len(means) >= 2 else None
        result.append(dict(knopf=key, effekt=effect, designs=sum(len(v) for v in groups.values()),
                           exakte_paare=len(paired), paar_effekt=median(paired) if paired else None))
    return sorted(result, key=lambda x: (x['effekt'] is None, -(x['effekt'] or 0.), x['knopf']))



START_KNOEPFE = ('hinc', 'pfmul', 'pf0', 'ziel_ab', 'alle_nah',
                 'kosten_via_signal', 'kosten_eng', 'kette_umweg')


def aktive_knoepfe(rows):
    ranked = [x['knopf'] for x in wirkung(rows) if x['effekt'] is not None]
    return tuple((ranked + [k for k in START_KNOEPFE if k not in ranked])[:8])


def gepoolt(rows, kl):
    ""
    exact, prior, seen = [], [], set()
    for r in rows[-10000:]:
        rk = zeilenklasse(r)
        if not rk or rk.split(':')[0] not in ('v10','v11') or familie(rk) != familie(kl) or r.get('zustand') == 'code_geaendert': continue
        b = belohnung(r)
        if b is None or r.get('zustand') == 'placer_kollision': continue
        cfg = r.get('cfg') or {}
        identity = (rk, r.get('designfamilie', r.get('aufgabe')), sha(cfg.get('kosten') or {}))
        if identity in seen: continue
        seen.add(identity)
        (exact if rk == kl else prior).append((r, b))
    weight = min(1., 8./len(prior)) if prior else 0.
    return [(r,b,1.) for r,b in exact] + [(r,b,weight) for r,b in prior]


def vorschlag(rows, kl, rnd=None, meister=None):
    ""
    rnd = rnd or random.Random()
    examples = gepoolt(rows, kl)
    base = dict(DEFAULT, **(meister or {}))
    key = rnd.choice(aktive_knoepfe(rows))
    values = RAUM[key]
    if rnd.random() < .25:
        old = base[key]
        i = min(range(len(values)), key=lambda j: 0 if values[j] == old else
                (abs(values[j]-old) if key not in KATEGORIEN else 1))
        alternatives = [v for v in (i-1, i+1) if 0 <= v < len(values)]
        base[key] = values[rnd.choice(alternatives)]
    else:
        samples = []
        for value in values:
            hits = [(r,b,w) for r,b,w in examples if
                    r.get('cfg', {}).get('kosten', {}).get(key, r.get('cfg', {}).get(key)) == value]
            observed = [(r,b,w) for r,b,w in hits if type(r.get('gueltig')) is bool]
            good = sum(w for r,b,w in observed if b[0] == 0)
            total = sum(w for r,b,w in observed)
            probability = rnd.betavariate(1+good, 1+total-good)
            n = sum(w for r,b,w in hits)
            mean = sum((0. if b[0] == 0 else b[1])*w for r,b,w in hits)/n if n else 4.


            samples.append(((1-probability)*rnd.gauss(mean, 1/math.sqrt(n+1)), value))
        base[key] = min(samples)[1]
    return parameter(base)


def herausforderer(rows, kl, ausgeschlossen=()):
    ""
    pooled = [(r,b,w) for r,b,w in gepoolt(rows,kl)
              if type(r.get('gueltig')) is bool and gesamtrest(r) is not None]
    centers = defaultdict(list)
    for r,b,w in pooled:
        design = (zeilenklasse(r),r.get('designfamilie',r.get('aufgabe')))
        centers[design].append(0. if b[0] == 0 else b[1])
    centers = {k:sum(v)/len(v) for k,v in centers.items()}
    groups = defaultdict(list); params = {}; exact = set()
    for r,b,w in pooled:
        costs = r.get('cfg', {}).get('kosten')
        if not costs: continue
        key = sha(costs)
        if key in ausgeschlossen: continue
        params[key] = parameter(costs)
        if zeilenklasse(r) == kl: exact.add(key)
        design = (zeilenklasse(r),r.get('designfamilie',r.get('aufgabe')))
        groups[key].append((b[0],(0. if b[0] == 0 else b[1])-centers[design],w))
    def rank(key):
        hits=groups[key];n=sum(w for _,_,w in hits)
        return (sum(f*w for f,_,w in hits)/n,sum(v*w for _,v,w in hits)/n,key)
    return params[min(exact, key=rank)] if exact else None


def qualitaet(a, b):
    ""
    from analyse.regelkennzahlen import torfelder, wert
    ra, rb = a.get('regel') or {}, b.get('regel') or {}
    errors = []
    if not ra.get('version') or ra.get('version') != rb.get('version'): errors.append('Messversion')
    for key in ('note_v3', 'kopplung'):
        x, y = wert(ra, key), wert(rb, key)
        if x is None or y is None or y > x+1e-8: errors.append(key)
    for section, keys in (('regeln', set(torfelder(ra.get('regeln') or {})) |
                          set(torfelder(rb.get('regeln') or {}))),
                          ('topologie', {'parallel_rest_mm', 'abzweig_mitte', 'spitz', 'abgriff_nah_mm'})):
        for key in keys:
            x, y = wert(ra.get(section), key), wert(rb.get(section), key)
            if x is None or y is None or y > x+1e-8: errors.append(section+'.'+key)
    if ra.get('regeln', {}).get('fehler') or rb.get('regeln', {}).get('fehler'): errors.append('Regelfehler')
    return errors


def signifikanz(deltas):
    ""
    wins = sum(d > 0 for d in deltas); losses = sum(d < 0 for d in deltas)
    n = wins + losses
    p = sum(math.comb(n, k) for k in range(wins, n+1))/2**n if n else 1.
    return dict(test='vorzeichen_einseitig_exakt', gewinne=wins, verluste=losses,
                bindungen=len(deltas)-n, p=p, alpha=.05)


def gleiche_platzierung(a, b):
    if any(z.get('zustand') != 'placer_kollision' or z.get('gueltig') is not False for z in (a,b)): return False
    ma, mb = a.get('meister_messung') or {}, b.get('meister_messung') or {}
    digest = ma.get('placement_sha')
    return (isinstance(digest, str) and len(digest) == 64 and digest == mb.get('placement_sha')
            and bool(ma.get('eingang')) and ma.get('eingang') == mb.get('eingang'))


def tor(vorher, nachher):
    ""
    reasons = []
    if not vorher or set(vorher) != set(nachher): return dict(angenommen=False, gruende=['Satz unvollstaendig'])
    lost = []; counts = [0,0]; rest = [0,0]; complete = True; excluded = []; deltas = []
    for name in sorted(vorher):
        a,b = vorher[name],nachher[name]
        skip = gleiche_platzierung(a,b)
        if skip: excluded.append(name)
        pair = []
        for i,z in enumerate((a,b)):
            collision = (z.get('placer_kennzahlen') or {}).get('kollisionen')
            known_place = (z.get('zustand') == 'placer_kollision' and z.get('gueltig') is False
                           and type(collision) in (int,float) and math.isfinite(collision) and collision > 0)
            if type(z.get('gueltig')) is not bool or (z.get('zustand') != 'ok' and not known_place):
                reasons.append(name+': unbekannter Abschluss')
            r = gesamtrest(z)
            if z.get('gueltig') is True:
                native = z.get('native_meister') or {}
                if native.get('fehler') != 0 or native.get('offen') != 0 or r != 0:
                    reasons.append(name+': widerspruechliche/fehlende Gueltigkeitsbelege')
            counts[i] += z.get('gueltig') is True
            if skip: continue
            if r is None:
                complete = False
                if not known_place: reasons.append(name+': Restmessung fehlt')
            else: rest[i] += r
            pair.append(r)
        if not skip and len(pair) == 2 and all(v is not None for v in pair): deltas.append(pair[0]-pair[1])
        if a.get('gueltig') is True:
            if b.get('gueltig') is not True: lost.append(name)
            else: reasons.extend(name+': '+e for e in qualitaet(a,b))
    if lost: reasons.append('Gueltige Boards verloren: '+','.join(lost))
    statistic = signifikanz(deltas)
    improved = counts[1] > counts[0] or (complete and counts[1] == counts[0]
                  and rest[1] < rest[0] and statistic['p'] <= statistic['alpha'])
    if not improved: reasons.append('Kein Gueltigkeitsgewinn oder signifikanter gepaarter Gesamtrestgewinn')
    return dict(angenommen=not reasons, gruende=reasons, gueltig=counts,
                rest=rest if complete else None, verloren=lost,
                rest_paare=len(deltas), placement_ausgeschlossen=excluded, statistik=statistic)
