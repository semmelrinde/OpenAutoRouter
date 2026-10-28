""
import collections, json, math, os, random, sys
os.environ.setdefault('OMP_NUM_THREADS', '2')
import numpy as np

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from lernen import modell as LM



ROH = collections.OrderedDict([
    ('mst', lambda m: math.log(max(m['mst'], 1e-6))),
    ('kreuz', lambda m: math.log(1 + 0.02 * m['kreuzungen'])),
    ('rudy', lambda m: math.log(1 + max(0.0, m['rudy_max'] - 1))),
    ('hpwl', lambda m: math.log(max(m['hpwl'], 1e-6))),
    ('mst_gnd', lambda m: math.log(1 + m['mst_gnd'])),
    ('laengste', lambda m: math.log(1 + m['laengste_kante'])),
    ('rudy_p95', lambda m: math.log(0.05 + m['rudy_p95'])),
    ('dichte', lambda m: math.log(1 + m['pads_max_4mm2'])),
    ('rand', lambda m: math.log(1 + m['rand_pads'])),
])
FAUST3 = ['mst', 'kreuz', 'rudy']
KLEIN = ['mst', 'kreuz', 'rudy', 'hpwl', 'laengste', 'dichte']
ALLE = list(ROH)


def laden(pfade, max_zeilen=20000):
    ""
    Z = collections.deque(maxlen=max_zeilen)
    for p in pfade:
        q = os.path.basename(p)
        for z in open(p):
            try: j = json.loads(z)
            except Exception: continue
            if not j.get('merkmale') or j.get('code_geaendert'): continue
            j['_quelle'] = q
            j['_aufgabe'] = str((j.get('regel') or {}).get('bewertungsziel', 'v3')) + ':' + str((j.get('regel') or {}).get('minimax_version', '')) + ':' + str((j.get('regel') or {}).get('minimax_profil', '')) + ':' + os.path.abspath(p) + ':' + j['board'] + ':' + str(j.get('code_sha256') or j.get('code') or 'alt') + ':' + json.dumps(j.get('budget') or {}, sort_keys=True) + ':' + ','.join(j.get('gndlagen') or []) + ':' + str(j.get('bewertung_version', 'alt'))
            if (j.get('regel') or {}).get('bewertungsziel') == 'minimax':
                j['_aufgabe'] += ':' + json.dumps(j['regel'].get('minimax_quellen'), sort_keys=True)
            j['_familie'] = LM.design_familie(j)
            if j.get('zustand'):
                zs = j['zustand']
                j['_zustand'] = ('zeitlimit_wand' if zs in ('wandlimit', 'placer_wandlimit') else
                                  'fehler' if zs in ('fehler', 'placer_fehler') else 'zeitlimit_cpu' if zs == 'zeitlimit' else zs)
            else:
                rc = j.get('router_rc')
                j['_zustand'] = 'zeitlimit_wand' if rc in ('zeitlimit', 'wandlimit') else 'fehler' if (rc not in (0, None) or j.get('fehler')) else 'ok'

            if not messbar(j): continue
            j['_gueltig'] = gueltig(j)
            v = regelwert(j)
            j['_wert'] = 2.0 if not j['_gueltig'] else (v / (1.0 + v) if v is not None else min(1.0, wert(j) / 2.0))
            j['_regel'] = v
            Z.append(j)
    return list(Z)


def wert(z):
    ""
    g, c = LM.ziel(z); m = z.get('mensch') or {}
    mk = (m.get('laenge_mm') or 0) + 3 * (m.get('vias') or 0)
    return min(2.0, c / mk) if g and c and mk > 0 else 2.0


def gueltig(z):
    from analyse.vertrag import von_zeile
    e = z.get('regel') or {}
    geo = von_zeile(z)['gueltig']
    return geo and bool(e.get('minimax_gueltig')) if e.get('bewertungsziel') == 'minimax' else geo


def regelwert(z):
    ""
    e = z.get('regel') or {}
    if e.get('bewertungsziel') == 'minimax':
        from analyse import minimax as MM
        return e.get('minimax') if MM.aktuell(e) and MM.vollstaendig(e) else None
    v = z.get('note_v3')
    if v is None: v = (z.get('regel') or {}).get('note_v3')
    try:
        v = float(v)
        if not math.isfinite(v) or v < 0: return None
        m = float(z.get('note_v3_mensch') or 0)
        return v / m if m > 1e-6 else v
    except (ValueError, TypeError): return None


def messbar(z):
    from lernen.fruehabbruch import abgebrochen
    if abgebrochen(z): return False
    e = z.get('regel') or {}
    if e.get('bewertungsziel') != 'minimax': return True
    from analyse import minimax as MM
    return MM.belohnung(z) is not None


def rang(z):
    ""
    v = regelwert(z); e = z.get('regel') or {}
    if not messbar(z): return (2, ((), float('inf')))
    if e.get('bewertungsziel') == 'minimax':
        from analyse import minimax as MM
        return (0 if gueltig(z) else 1, MM.rang(e))
    return (0 if gueltig(z) else 1, ((v if v is not None else float('inf'),), 0.0))


def _vertrag(z):
    e = z.get('regel') or {}
    return (e.get('bewertungsziel', 'v3'), e.get('version', z.get('bewertung_version', 'alt')),
            e.get('minimax_version'), e.get('minimax_profil'))


def _tor_fehler(grund, **werte):
    ""
    return dict(dict(besser=False, nicht_schlechter=False, vollstaendig=False,
                     gueltig=[0, 0], gemeinsam=0, kennzahlen={}, grund=grund), **werte)


def tor_messen(Z, kandidat, bezug, B=2000):
    ""
    gruppen = collections.defaultdict(list)
    for ks in aufgaben(Z, nur_messbar=False).values():
        a, b = kandidat(ks)[0], bezug(ks)[0]
        if _vertrag(a) != _vertrag(b):
            return _tor_fehler('verschiedene Bewertungsvertraege')
        gruppen[_vertrag(a)].extend(ks)
    if len(gruppen) <= 1: return _tor_vertrag_messen(Z, kandidat, bezug, B)
    tore = {repr(v): _tor_vertrag_messen(z, kandidat, bezug, B) for v, z in sorted(gruppen.items(), key=lambda q: repr(q[0]))}
    return dict(besser=all(t.get('nicht_schlechter', False) for t in tore.values()) and any(t['besser'] for t in tore.values()),
                vollstaendig=all(t.get('vollstaendig', False) for t in tore.values()),
                nicht_schlechter=all(t.get('nicht_schlechter', False) for t in tore.values()),
                gueltig=[sum(t['gueltig'][i] for t in tore.values()) for i in (0, 1)],
                gemeinsam=sum(t['gemeinsam'] for t in tore.values()), tore=tore,
                fehler={v: t['grund'] for v, t in tore.items() if t.get('grund')})


def _tor_vertrag_messen(Z, kandidat, bezug, B=2000):
    ""
    paare = []
    for _, ks in aufgaben(Z, nur_messbar=False).items():
        a, b = kandidat(ks)[0], bezug(ks)[0]
        ea, eb = a.get('regel') or {}, b.get('regel') or {}
        if _vertrag(a) != _vertrag(b):
            return _tor_fehler('verschiedene Bewertungsvertraege')
        if ea.get('bewertungsziel') == 'minimax':
            from analyse import minimax as MM
            if not messbar(a) or not messbar(b): return _tor_fehler('fehlende/veraltete Minimax-Messung')
            if not MM.aktuell(ea) or not MM.aktuell(eb): return _tor_fehler('veraltete Minimax-Version')
            if gueltig(a):
                tor = MM.annahme(eb, ea)
                if not tor['nicht_schlechter']: return _tor_fehler(tor['grund'])
        paare.append((a, b))
    ga = sum(gueltig(a) for a, b in paare); gb = sum(gueltig(b) for a, b in paare)
    common = [(a, b) for a, b in paare if gueltig(a) and gueltig(b)]
    out = dict(gueltig=[ga, gb], gemeinsam=len(common), kennzahlen={}, besser=False, nicht_schlechter=False, vollstaendig=False)
    from analyse import vertrag as VT
    vertrags_tore=[VT.vergleich(VT.von_zeile(b),VT.von_zeile(a)) for a,b in paare]
    out['vertrag']=vertrags_tore
    contract_ok=all(t['nicht_schlechter'] for t in vertrags_tore)
    elektrische_sperre=any(gueltig(a) and VT.von_zeile(a)['elektrisch']=='verletzt' for a,b in paare)
    if elektrische_sperre:contract_ok=False
    if not contract_ok:
        out['grund']='; '.join(t['grund'] for t in vertrags_tore if not t['nicht_schlechter'])
        if elektrische_sperre:out['grund']+=' Elektrisch verletzter Auswahlkandidat'
    if ga < gb or not common: return out
    flat=all(not isinstance((z.get('regel') or {}).get('topologie'),dict) for pair in common for z in pair)
    keys = list(VT.TOR) if flat else ['note_v3', 'kopplung', 'parallel_rest_mm', 'abzweig_mitte', 'spitz', 'abgriff_nah_mm']
    if any((a.get('regel') or {}).get('bewertungsziel') == 'minimax' for a, b in common):
        from analyse.regelkennzahlen import torfelder
        keys += ['regel:' + k for k in torfelder(common[0][0]['regel'])]
    si = any((z.get('regel') or {}).get('version') == 'v3.2' for a, b in common for z in (a, b))
    si_keys = ('auge', 'auge_vertikal_verlust', 'auge_horizontal_verlust', 'stoerbudget', 'stoerbudget_max',
               'verlust', 'uebersprechen', 'rueckweg', 'topologie', 'regelpruefer')
    if si: keys += list(si_keys)
    def value(z, key):
        if flat:return VT.zahl((z.get('regel') or {}).get(key))
        if key in si_keys:
            r = z.get('regel') or {}
            return (r.get('regeln') or r).get(key)
        if key.startswith('regel:'): return (z.get('regel') or {}).get('regeltor', {}).get(key[6:])
        if key == 'note_v3':
            if (z.get('regel') or {}).get('bewertungsziel') == 'minimax':
                return (z.get('regel') or {}).get('note_v3')
            return regelwert(z)
        r = z.get('regel') or {}
        return r.get('kopplung') if key == 'kopplung' else (r.get('topologie') or {}).get(key)
    families = sorted({a['_familie'] for a, b in common}); rnd = random.Random(0)
    draws = [rnd.choices(families, k=len(families)) for _ in range(B)]
    strict = ga > gb or any((a.get('regel') or {}).get('bewertungsziel') == 'minimax' and
                             MM.annahme(b['regel'], a['regel'])['besser'] for a, b in common)
    complete = contract_ok
    for key in keys:
        ds = collections.defaultdict(list)
        for a, b in common:
            va, vb = value(a, key), value(b, key)
            if va is None or vb is None or not math.isfinite(va) or not math.isfinite(vb): complete = False; continue
            ds[a['_familie']].append(va-vb)
        if len(ds) != len(families) or sum(map(len, ds.values())) != len(common):
            complete = False
            out['kennzahlen'][key] = dict(n=sum(map(len, ds.values())), ci=None); continue
        diff = float(np.mean([d for v in ds.values() for d in v]))
        boot = [np.mean([d for f in draw for d in ds.get(f, [])]) for draw in draws]
        ci = [float(np.percentile(boot, q)) for q in (2.5, 97.5)]
        einzel_max = max(d for v in ds.values() for d in v)
        out['kennzahlen'][key] = dict(n=sum(map(len, ds.values())), diff=diff, ci=ci, einzel_max=einzel_max)
        if diff < -1e-9: strict = True
        if diff > 1e-9 or ci[1] > 1e-9: complete = False


        if si and einzel_max > 1e-9: complete = False
    out['besser'] = complete and strict and len(families) >= 3
    out['nicht_schlechter'] = complete and ga >= gb
    out['vollstaendig'] = complete; out['familien'] = len(families)
    return out


def aufgaben(Z, max_k=64, nur_messbar=True):
    pro = collections.OrderedDict()
    for z in Z:
        if nur_messbar and not messbar(z): continue
        pro.setdefault(z['_aufgabe'], []).append(z)
    return collections.OrderedDict((a, ks[-max_k:]) for a, ks in pro.items())


def relativ(ks, namen):
    ""
    X = np.array([[ROH[n](k['merkmale']) for n in namen] for k in ks], float)
    return X - X.mean(0)


def paare(Z, namen, ohne_rauschen=True, gewichtet=True):
    ""
    X, y, w = [], [], []
    for a, ks in aufgaben(Z).items():
        Xr = relativ(ks, namen)
        for i in range(len(ks)):
            for j in range(len(ks)):
                if i == j: continue
                if ohne_rauschen and (ks[i]['_zustand'] in ('zeitlimit_wand', 'fehler') or ks[j]['_zustand'] in ('zeitlimit_wand', 'fehler')): continue
                gi, gj = gueltig(ks[i]), gueltig(ks[j])
                ei, ej = ks[i].get('regel') or {}, ks[j].get('regel') or {}
                if ei.get('bewertungsziel', 'v3') != ej.get('bewertungsziel', 'v3'): continue
                if ei.get('bewertungsziel') == 'minimax':
                    from analyse import minimax as MM
                    c = MM.annahme(ej, ei)
                    if not (gi and c['besser']) and not (gj and MM.annahme(ei, ej)['besser']): continue
                    d = (-1.0 if gi else 1.0) if gi != gj else c['delta'] / (1.0 + abs(c['delta']))
                elif gi != gj: d = -1.0 if gi else 1.0
                elif not gi: continue
                else:
                    vi, vj = regelwert(ks[i]), regelwert(ks[j])
                    if vi is None or vj is None: continue
                    d = (vi - vj) / (1.0 + abs(vi) + abs(vj))
                if abs(d) < 1e-9: continue
                X.append(Xr[i] - Xr[j]); y.append(1 if d > 0 else 0); w.append(abs(d) if gewichtet else 1.0)
    return np.array(X), np.array(y), np.array(w)


def trainieren(Z, namen=KLEIN, C=1.0, ohne_rauschen=True, gewichtet=True):
    ""
    from sklearn.linear_model import LogisticRegression
    X, y, w = paare(Z, namen, ohne_rauschen, gewichtet)
    if len(X) < 8 or len(set(y)) < 2: return dict(ziel='gueltig_note_v3', namen=list(namen), skala=[1.0] * len(namen), gewichte=[1.0 if n in FAUST3 else 0.0 for n in namen], paare=len(X))
    s = X.std(0); s[s < 1e-9] = 1.0
    lr = LogisticRegression(fit_intercept=False, C=C, max_iter=1000).fit(X / s, y, sample_weight=w)
    return dict(ziel='gueltig_note_v3', namen=list(namen), skala=[float(v) for v in s], gewichte=[float(v) for v in lr.coef_[0]], paare=int(len(X) // 2))


def punkte(mod, ks):
    Xr = relativ(ks, mod['namen'])
    return Xr @ (np.array(mod['gewichte']) / np.array(mod['skala']))


def ordnung_rang(mod):
    return lambda ks: [ks[i] for i in np.argsort(punkte(mod, ks), kind='stable')]


def faust_wert(k):
    m = k['merkmale']; return m['mst'] * (1 + 0.02 * m['kreuzungen']) * (1 + max(0.0, m['rudy_max'] - 1))


def ordnung_faust(ks): return sorted(ks, key=faust_wert)
def ordnung_erster(ks): return sorted(ks, key=lambda k: k['kandidat'])
def ordnung_orakel(ks): return sorted(ks, key=rang)


def ordnung_alt(mod):
    ""
    def o(ks):
        P, C = LM.vorhersage(mod, [k['merkmale'] for k in ks]); E = P * C + (1 - P) * 2 * float(np.median(C))
        return [ks[i] for i in np.argsort(E, kind='stable')]
    return o


def zufall(ks):
    ""
    w = [k['_wert'] for k in ks]; n = len(w)
    if n < 2: return float(np.mean(w)), float(np.mean(w))
    return float(np.mean(w)), float(np.mean([min(w[i], w[j]) for i in range(n) for j in range(i + 1, n)]))


def messen(Z_pool, Z_test, modelle, alt=False):
    ""
    T = aufgaben(Z_test); fam = collections.OrderedDict()
    for a, ks in T.items(): fam.setdefault(ks[0]['_familie'], []).append(a)
    erg = {}
    for f, auf in fam.items():
        tr = [z for z in Z_pool if z['_familie'] != f]
        ordn = dict(erster=ordnung_erster, faust=ordnung_faust, orakel=ordnung_orakel)
        for name, kw in modelle.items(): ordn[name] = ordnung_rang(trainieren(tr, **kw))
        if alt and len({LM.ziel(z)[0] for z in tr}) > 1:
            ordn['alt_abs'] = ordnung_alt(LM.trainieren(tr)); ordn['alt_rel'] = ordnung_alt(LM.trainieren_rel(tr))
        for a in auf:
            ks = T[a]; e = {}
            for name, o in ordn.items():
                r = o(ks); e[name] = (r[0]['_wert'], min(k['_wert'] for k in r[:2]))
            e['zufall'] = zufall(ks); e['_familie'] = f; e['_n'] = len(ks)
            erg[a] = e
    return erg


def fest_messen(Z_test, ordnungen):
    ""
    erg = {}
    for a, ks in aufgaben(Z_test).items():
        e = {name: ((r := o(ks))[0]['_wert'], min(k['_wert'] for k in r[:2])) for name, o in ordnungen.items()}
        e['zufall'] = zufall(ks); e['_familie'] = ks[0]['_familie']; e['_n'] = len(ks)
        erg[a] = e
    return erg


def tabelle(erg, bezug='faust', B=4000, seed=0):
    ""
    auf = sorted(erg); fams = sorted({erg[a]['_familie'] for a in auf}); namen = [k for k in erg[auf[0]] if not k.startswith('_')]
    idx = {f: [i for i, a in enumerate(auf) if erg[a]['_familie'] == f] for f in fams}
    rnd = random.Random(seed); out = collections.OrderedDict()
    zieh = [[i for f in rnd.choices(fams, k=len(fams)) for i in idx[f]] for _ in range(B)]
    for n in namen:
        z = dict()
        for t, tag in ((0, 'eins'), (1, 'zwei')):
            v = np.array([erg[a][n][t] for a in auf]); b = np.array([erg[a][bezug][t] for a in auf]); d = v - b
            bs = np.array([d[s].mean() for s in zieh])
            z[tag] = dict(wert=round(float(v.mean()), 4), gueltig=int((v < 2.0).sum()), diff=round(float(d.mean()), 4),
                          ci=[round(float(np.percentile(bs, 2.5)), 4), round(float(np.percentile(bs, 97.5)), 4)],
                          besser=int((d < -1e-9).sum()), schlechter=int((d > 1e-9).sum()))
        out[n] = z
    return dict(aufgaben=len(auf), familien=len(fams), strategien=out)


def drucke(titel, tab):
    print('\n%s  (%d Aufgaben, %d Familien)' % (titel, tab['aufgaben'], tab['familien']))
    print('  %-14s | %-28s | %-28s' % ('Strategie', 'ein Kandidat', 'bester von zwei'))
    print('  %-14s | %6s %4s %8s %15s %5s | %6s %4s %8s %15s %5s' % ('', 'Wert', 'gült', 'Δ Faust', '95 %', '+/−', 'Wert', 'gült', 'Δ Faust', '95 %', '+/−'))
    for n, z in tab['strategien'].items():
        print('  %-14s | %s | %s' % (n, *['%6.4f %4d %+8.4f [%+6.3f,%+6.3f] %2d/%-2d' % (z[t]['wert'], z[t]['gueltig'], z[t]['diff'], z[t]['ci'][0], z[t]['ci'][1],
                                                                                              z[t]['besser'], z[t]['schlechter']) for t in ('eins', 'zwei')]))


def merkmal_rang(Z):
    ""
    from scipy.stats import spearmanr
    out = {}
    for n in ALLE + ['faust']:
        r_alle, r_gueltig = [], []
        for a, ks in aufgaben(Z).items():
            x = [math.log(max(faust_wert(k), 1e-9)) if n == 'faust' else ROH[n](k['merkmale']) for k in ks]; w = [k['_wert'] for k in ks]
            if len(ks) >= 3 and len(set(w)) > 1 and len(set(x)) > 1: r_alle.append(spearmanr(x, w)[0])
            g = [(xi, wi) for xi, wi in zip(x, w) if wi < 2.0]
            if len(g) >= 3 and len({q[1] for q in g}) > 1 and len({q[0] for q in g}) > 1: r_gueltig.append(spearmanr(*zip(*g))[0])
        out[n] = dict(alle=round(float(np.mean(r_alle)), 3) if r_alle else None, n_alle=len(r_alle),
                      gueltige=round(float(np.mean(r_gueltig)), 3) if r_gueltig else None, n_gueltige=len(r_gueltig))
    return out


def stand(Z):
    A = aufgaben(Z); c = collections.Counter(z['_zustand'] for z in Z)
    gem = sum(1 for ks in A.values() if 0 < sum(k['_wert'] < 2.0 for k in ks) < len(ks))
    return dict(versuche=len(Z), aufgaben=len(A), familien=len({z['_familie'] for z in Z}), gueltig=sum(z['_wert'] < 2.0 for z in Z),
                zustand=dict(c), aufgaben_gemischt=gem, aufgaben_alle_gueltig=sum(1 for ks in A.values() if all(k['_wert'] < 2.0 for k in ks)),
                aufgaben_keiner_gueltig=sum(1 for ks in A.values() if all(k['_wert'] >= 2.0 for k in ks)))


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    pfade = kw.get('daten', os.path.join(R, 'lernen/daten/versuche.jsonl')).split(',')
    Z = laden(pfade); print('Daten:', json.dumps(stand(Z)))
    if sys.argv[1] == 'messen':
        mod = dict(rang=dict(namen=KLEIN), rang_faust3=dict(namen=FAUST3), rang_alle=dict(namen=ALLE))
        erg = messen(Z, Z, mod, alt=kw.get('alt', '0') == '1'); tab = tabelle(erg); drucke('Jede Familie einmal ausgelassen', tab)
        print('\nRangkorrelation Merkmal ↔ Wert innerhalb einer Aufgabe:', json.dumps(merkmal_rang(Z), indent=1))
        if kw.get('aus'): json.dump(dict(daten=pfade, stand=stand(Z), tabelle=tab), open(kw['aus'], 'w'), indent=1)
    elif sys.argv[1] == 'uebertragen':
        T = laden(kw['test'].split(',')); print('Test:', json.dumps(stand(T)))
        erg = messen(Z, T, dict(rang=dict(namen=KLEIN), rang_faust3=dict(namen=FAUST3)), alt=kw.get('alt', '0') == '1')
        drucke('Training auf daten=, Test auf test=', tabelle(erg))
    elif sys.argv[1] == 'trainieren':
        m = trainieren(Z, KLEIN); m.update(daten=[os.path.relpath(p, R) for p in pfade], stand=stand(Z), art='rang_linear_paarweise')
        aus = kw.get('aus', os.path.join(R, 'lernen/modell/rang_v2.json')); json.dump(m, open(aus, 'w'), indent=1)
        print('Gewichte (je Standardabweichung):', dict(zip(m['namen'], [round(g, 3) for g in m['gewichte']])), '->', aus)


if __name__ == '__main__':
    main()
