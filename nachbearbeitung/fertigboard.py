""
import collections, json, math, os, re, shutil, subprocess, sys, time

A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from gemeinsam.drc_ereignisse import ereignis, neue_ereignisse
from gemeinsam.kupfer_netz import Hindernisse, Netzkupfer, ZonenWaechter, geo_hash, zonen_fuellung as _zonen_fuellung, zonen_umrisse
from kicad.sexpr_tauschen import tauschen
from nachbearbeitung.sperrnetze import netz_abstaende, sperrnetze, sperrumfang

KPY = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
CLI = shutil.which('kicad-cli') or '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
VPY = sys.executable


def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def kopie(src_pcb, dst_pcb, pro, dru):
    shutil.copy(src_pcb, dst_pcb)
    b = dst_pcb[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')


def drc(pcb, pro, dru):
    ""
    b = pcb[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru'); bj = b + '_drc.json'
    if os.path.exists(bj): os.remove(bj)
    r = subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', '--refill-zones', '--save-board', '-o', bj, pcb],
                       capture_output=True, text=True)
    shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')
    if r.returncode != 0 or not os.path.exists(bj): raise RuntimeError('kicad-cli DRC fehlgeschlagen (%d): %s' % (r.returncode, r.stderr[-500:]))
    d = json.load(open(bj))
    if 'violations' not in d or 'unconnected_items' not in d: raise RuntimeError('DRC-Bericht ohne violations/unconnected_items: ' + bj)
    c = collections.Counter((v['severity'], v['type']) for v in d['violations'])
    ev = collections.Counter(ereignis(v) for v in d['violations']) + collections.Counter(ereignis(v, 'offen') for v in d['unconnected_items'])
    return dict(E=collections.Counter({k[1]: n for k, n in c.items() if k[0] == 'error'}), U=len(d['unconnected_items']),
                W=collections.Counter({k[1]: n for k, n in c.items() if k[0] == 'warning'}), ev=ev, d=d)


def hind(pcb, out, box):
    if os.path.exists(out): os.remove(out)
    r = subprocess.run([KPY, os.path.join(A, 'kicad', 'hind_export.py'), pcb] + [str(x) for x in box] + [out], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(out): raise RuntimeError('hind_export fehlgeschlagen: ' + r.stderr[-500:])
    return json.load(open(out))


def rahmen(H):
    xs = [p[0] for e in H['edges'] for p in e]; ys = [p[1] for e in H['edges'] for p in e]
    return [math.floor(min(xs)) - 1, math.floor(min(ys)) - 1, math.ceil(max(xs)) + 1, math.ceil(max(ys)) + 1]


def zonen_fuellung(text):
    ""
    return [(f['net'], f['layer'], f['poly']) for f in _zonen_fuellung(text)]


def netze_der_meldung(neu_ev, aend_netze, D=None, text_vorher=None):
    ""
    netze, fremd, orte = set(), set(), []
    def aus(items):
        eigen = False
        for i in items:
            ds = i.get('description', '')
            for teil in ds.split('[')[1:]:
                n = teil.split(']')[0]
                if n in aend_netze: netze.add(n); eigen = True
                else: fremd.add(n)
        if not eigen: orte.extend((i['pos']['x'], i['pos']['y']) for i in items if i.get('pos'))
        return eigen
    for v in neu_ev: aus(v['items'])
    if netze or D is None: return netze
    from shapely.geometry import LineString, Point
    neu = [(s['net'], s['layer'], LineString([s['a'], s['b']]).buffer(s['w'] / 2 + 0.02)) for s in D['add'] if s['a'] != s['b']]
    if text_vorher and fremd:
        for zn, L, g in zonen_fuellung(text_vorher):
            if zn not in fremd: continue
            netze |= {n for n, L2, b in neu if L2 == L and b.intersects(g)}
    if not netze and orte:
        netze |= {n for n, _, b in neu if any(b.distance(Point(o)) < 1.5 for o in orte)}
    return netze


def gut(R, ref):
    ""
    return not neue_ereignisse(R, ref) and R['U'] <= ref['U']


def anwenden_pruefen(cur, D, ziel, pro, dru, ref, sieb):
    ""
    text = open(cur, encoding='utf-8').read()
    net_of = {}
    H_uuid = D.get('_uuid_netz', {})
    for u in D['remove']: net_of[u] = H_uuid.get(u)
    raus = set()
    for runde in range(sieb + 1):
        Dk = dict(remove=[u for u in D['remove'] if net_of.get(u) not in raus], add=[s for s in D['add'] if s['net'] not in raus],
                  add_generated=[g for g in D.get('add_generated', []) if g.get('netz') not in raus])
        if not Dk['remove'] and not Dk['add']: return False, Dk, dict(grund='alles zurückgenommen', zurueck=sorted(raus))
        neu, info = tauschen(text, Dk)
        if info['fehlend']: return False, Dk, dict(grund='Bahnen nicht gefunden (kein halbes Delta): %s' % info['fehlend'][:5])
        if os.path.exists(ziel) and os.path.samefile(ziel, cur): raise RuntimeError('Ziel ist die Eingabe: ' + ziel)
        open(ziel, 'w', encoding='utf-8').write(neu)
        R = drc(ziel, pro, dru); nev = neue_ereignisse(R, ref)
        log('   DRC: Fehler %s, offen %d, Warnungen %d, neue Meldungen %d' % (dict(R['E']) or 0, R['U'], sum(R['W'].values()), len(nev)),
            '(zurückgenommen: %d Netze)' % len(raus) if raus else '')
        if gut(R, ref): return True, Dk, dict(info, zurueck=sorted(raus), drc=dict(fehler=dict(R['E']), offen=R['U'], warnungen=sum(R['W'].values())), _R=R)
        aend = {s['net'] for s in Dk['add']} | {net_of.get(u) for u in Dk['remove']} - {None}
        neu_raus = netze_der_meldung(nev, aend, Dk, text)
        if not neu_raus: return False, Dk, dict(grund='DRC-Meldung keinem geänderten Netz zuzuordnen', fehler=dict(R['E']), offen=R['U'])
        raus |= neu_raus
        log('   nehme zurück:', ', '.join(sorted(n.split('/')[-1] for n in neu_raus)))
    return False, Dk, dict(grund='nach %d Sieb-Runden nicht sauber' % sieb, zurueck=sorted(raus))




def schritt_aufraeumen(H, S, hin, o, cur, tmp):
    from nachbearbeitung.aufraeumen import aufraeumen
    return aufraeumen(H, sperren=S, netze=o.get('_netze'), hin=hin, mikro=float(o.get('mikro', 0.03)), fix=o.get('_fix', ()))


def schritt_mitte(H, S, hin, o, cur, tmp):
    from nachbearbeitung.mitte_anflug import mitte
    he = float(o.get('hmax_ebene', 0.3))
    hm = {(n, L): he for n, Ls in (o.get('_hmax_lagen') or {}).items() for L in Ls} if he > 0 else {}
    return mitte(H, sperren=S, netze=o.get('_netze'), hin=hin, fix=o.get('_fix', ()), hmax=hm)


def schritt_abkuerzen(H, S, hin, o, cur, tmp):
    hj = os.path.join(tmp, 'abk_h.json'); json.dump(H, open(hj, 'w'))
    box = rahmen(H); zonen = o['_zonennetze']
    netze = sorted(n for n in ({t['net'] for t in H['tracks'] if t['net']} if o.get('_netze') is None else set(o['_netze'])) if n not in S)
    D = dict(remove=[], add=[], stat={})
    fj = os.path.join(tmp, 'abk_fix.json'); json.dump(sorted(o.get('_fix', ())), open(fj, 'w'))
    zj = os.path.join(tmp, 'abk_ohne_tote.json'); json.dump(sorted(n for n in netze if n in zonen), open(zj, 'w'))
    if not netze: return D


    oj = os.path.join(tmp, 'abk_o.json')
    hl = os.path.join(tmp, 'abk_hmax_lagen.json'); json.dump(o.get('_hmax_lagen') or {}, open(hl, 'w'))
    bj = os.path.join(tmp, 'abk_budget.json'); json.dump(laengen_budget(o.get('_L0'), H, o.get('_tor_netze')), open(bj, 'w'))
    opt = ['drc=' + o.get('drc', '0.09'), 'drcpad=' + o.get('drcpad', '0.10'), 'drcvia=' + o.get('drcvia', o.get('drc', '0.09')),
           'rand=' + o.get('kante', '0.2'), 'loch=' + o.get('loch', '0.25'), 'hmax=' + o.get('hmax', '0'), 'tote=1', 'ohne_tote=' + zj,
           'hmax_lagen=' + hl, 'hmax_ebene=' + o.get('hmax_ebene', '0.3'), 'budget=' + bj,
           'pcb=' + cur, 'fix=' + fj] + (['klassen=' + o['klassen']] if o.get('klassen') else []) + (['sperren=' + o['sperren']] if o.get('sperren') else [])
    r = subprocess.run([VPY, os.path.join(A, 'nachbearbeitung', 'abkuerzen.py'), hj] + [str(x) for x in box] + [';'.join(netze), oj] + opt,
                       capture_output=True, text=True)
    if r.returncode != 0: raise RuntimeError('abkuerzen: ' + r.stderr[-800:])
    O = json.load(open(oj)); D['remove'] += O['remove']; D['add'] += O['add']
    D['stat']['abkuerzen'] = r.stdout.strip().split('\n')[-1][:400]
    return D


def laengen_budget(L0, H, tor_netze, prozent=2.0, mm=1.0):
    ""
    if not L0 or not tor_netze: return {}
    from analyse.fertig_kennzahlen import laengen
    L = laengen(H)
    return {n: max(0.0, min(mm, prozent / 100.0 * L0.get(n, 0.0)) - abs(L.get(n, 0.0) - L0.get(n, 0.0)) - 1e-6) for n in sorted(tor_netze)}


def schritt_glaetten(H, S, hin, o, cur, tmp):
    ""
    from shapely.geometry import LineString, Point, Polygon
    from nachbearbeitung.glaetten45 import glaetten
    clr = max(float(o.get('drc', 0.09)), float(o.get('drcpad', 0.10)), float(o.get('drcvia', o.get('drc', 0.09))))
    loch, kante = float(o.get('loch', 0.25)), float(o.get('kante', 0.2))
    ringe = sorted((Polygon(e).buffer(0) for e in H['edges'] if len(e) > 2), key=lambda g: -g.area)
    aussen = ringe[0]; schlitze = [g for g in ringe[1:] if aussen.contains(g.representative_point())]
    pads = collections.defaultdict(lambda: dict(polys=collections.defaultdict(list)))
    for p in H['pads']:
        k = (p.get('ref'), p.get('num'), p['net']); pads[k]['net'] = p['net']; pads[k]['polys'][p['layer']].append(p['poly'])
        pads[k]['x'], pads[k]['y'] = p.get('x', 0), p.get('y', 0)
    lagen = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']})
    Bp = [dict(net=v['net'], polys=dict(v['polys']), x=v['x'], y=v['y']) for v in pads.values()]
    for h in H.get('holes', []):
        c = list(Point(h['x'], h['y']).buffer(h['d'] / 2 + max(0.0, loch - clr), 16).exterior.coords)
        Bp.append(dict(net=h.get('net') or '#LOCH', polys={L: [c] for L in lagen}, x=h['x'], y=h['y']))
    sperr = [dict(verbote=dict(bahnen=True), polys=[list(g.buffer(kante, 8).exterior.coords)]) for g in schlitze]
    sperr += [dict(verbote=dict(bahnen=True), polys=[k['poly']]) for k in H.get('keepouts', []) if k.get('tracks')]
    B = dict(pads=Bp, outline=[list(aussen.exterior.coords)], keepouts=sperr,
             flaechen=[(n, L, g.buffer(max(0.0, c - clr), 8)) for n, L, g, c in o.get('_flaechen', ())])
    na = netz_abstaende(o.get('_dru'), {t['net'] for t in H['tracks']})
    tracks = []; wahl = set(o['_netze']) if o.get('_netze') is not None else None
    for t in H['tracks']:
        w = t['w'] + (2 * max(0.0, na.get(t['net'], 0) - clr) if t['net'] in S else 0.0)
        tracks.append(dict(net=t['net'], layer=t['layer'], pts=[list(t['a']), list(t['b'])], w=[w],
                           **({'fix': True} if t.get('arc') or t['net'] in S or (wahl is not None and t['net'] not in wahl)
                              or t.get('uuid') in o.get('_fix', ()) else {})))
    dmax = max([v['d'] for v in H['vias']] + [0.45])
    R = dict(tracks=tracks, vias=[dict(net=v['net'], x=v['x'], y=v['y'], d=v['d']) for v in H['vias']],
             optionen=dict(via='%g/0.2' % dmax, clr=clr, edge=kante))

    bogen = {(t['net'], t['layer']) for t in H['tracks'] if t.get('arc')}
    for t in R['tracks']:
        if (t['net'], t['layer']) in bogen: t['fix'] = True

    from nachbearbeitung.glaetten45 import implizite_anker
    rd = lambda p: (round(p[0], 4), round(p[1], 4))
    R['tracks'], anker = implizite_anker(R, B)
    vorher = collections.defaultdict(set)
    for t in R['tracks']:
        vorher[(t['net'], t['layer'])].add(tuple(sorted([rd(t['pts'][0]), rd(t['pts'][-1])])))
    he = float(o.get('hmax_ebene', 0.3))
    hm = {(n, L): he for n, Ls in (o.get('_hmax_lagen') or {}).items() for L in Ls} if he > 0 else {}
    info = glaetten(B, R, clr=clr, rand=kante, sperren=set(S), anker=anker, alt_hindernis=True, hmax=hm, ecken_schutz=True)
    nachher = collections.defaultdict(list)
    for t in R['tracks']: nachher[(t['net'], t['layer'])].append(t)
    D = dict(remove=[], add=[], stat=info)
    for key, ts in sorted(nachher.items()):
        if key[0] in S or key in bogen or (wahl is not None and key[0] not in wahl): continue
        segs = {tuple(sorted([rd(a), rd(b)])) for t in ts for a, b in zip(t['pts'][:-1], t['pts'][1:]) if a != b}
        if segs == vorher[key]: continue
        D['remove'] += sorted({t['uuid'] for t in H['tracks'] if (t['net'], t['layer']) == key})
        for t in ts:
            for (a, b), w in zip(zip(t['pts'][:-1], t['pts'][1:]), t['w']):
                if a != b: D['add'].append(dict(net=key[0], layer=key[1], a=list(a), b=list(b), w=w))

    budget = laengen_budget(o.get('_L0'), H, o.get('_tor_netze'))
    if budget:
        u_l = {t['uuid']: math.dist(t['a'], t['b']) for t in H['tracks']}; u_n = {t['uuid']: t['net'] for t in H['tracks']}
        dl = collections.defaultdict(float)
        for u in D['remove']: dl[u_n[u]] -= u_l[u]
        for s_ in D['add']: dl[s_['net']] += math.dist(s_['a'], s_['b'])
        zu_lang = sorted(n for n, d in dl.items() if n in budget and abs(d) > budget[n])
        if zu_lang:
            D['remove'] = [u for u in D['remove'] if u_n[u] not in zu_lang]; D['add'] = [s_ for s_ in D['add'] if s_['net'] not in zu_lang]
            info = dict(info, verworfen_laengentor=zu_lang)

    NK = Netzkupfer(H); rm = set(D['remove']); kaputt = set()
    for net in sorted({s['net'] for s in D['add']}):
        alt = NK.tracks.get(net, [])
        neu = [t for t in alt if t['uuid'] not in rm] + [s for s in D['add'] if s['net'] == net]
        if NK.trennt(net, alt, neu): kaputt.add(net)
    if kaputt:
        log('   glaetten: trennt, zurück:', ', '.join(sorted(kaputt)))
        u_net = {t['uuid']: t['net'] for t in H['tracks']}
        D['remove'] = [u for u in D['remove'] if u_net.get(u) not in kaputt]; D['add'] = [s for s in D['add'] if s['net'] not in kaputt]
    D['stat'] = dict(info, netze_lagen=len({(s['net'], s['layer']) for s in D['add']}), verworfen_trennt=sorted(kaputt))
    return D

def form_tor(H, H_vor=None, L0=None, tor_netze=()):
    ""
    from analyse.fertig_kennzahlen import laengen_verstoss, mitte_verlust, tor
    t = tor(H)
    t['mitte_verloren'] = sum(mitte_verlust(H_vor, H).values()) if H_vor is not None and H_vor is not H else 0
    t['laengentor'] = len(laengen_verstoss(L0, H, tor_netze)) if L0 is not None and tor_netze else 0
    return t


def form_netze(H_neu, H_vor, L0, tor_netze):
    ""
    from analyse.fertig_kennzahlen import laengen_verstoss, mitte_verlust
    return set(mitte_verlust(H_vor, H_neu)) | set(laengen_verstoss(L0, H_neu, tor_netze) if tor_netze else {})


def netze_schlechter(H, H_neu, L0, tor_netze, geaendert):
    ""
    return set(form_gruende(H, H_neu, L0, tor_netze, geaendert))


FORM_GRUENDE = ('winkel', 'r32', 'haarnadel', 'ecke90', 'enden', 'ziel', 'mitte', 'laenge')


def form_gruende(H, H_neu, L0, tor_netze, geaendert, fuell=None, fuell_neu=None):
    ""
    from analyse.fertig_kennzahlen import ecken90_neu, enden_neu, laengen_verstoss, mitte_verlust, tor, ziel_verlassen
    geaendert = sorted(n for n in geaendert if n)
    if not geaendert: return {}
    out = collections.defaultdict(set)
    def je(Hx, n):
        Hn = dict(Hx, tracks=[t for t in Hx['tracks'] if t['net'] == n], vias=[v for v in Hx['vias'] if v['net'] == n],
                  pads=[p for p in Hx['pads'] if p['net'] == n])
        t = tor(Hn); return (t['winkel'], t['r32'], t['haarnadeln'])
    for n in geaendert:
        for g, a, b in zip(('winkel', 'r32', 'haarnadel'), je(H_neu, n), je(H, n)):
            if a > b: out[n].add(g)
    for (n, L) in ecken90_neu(H, H_neu, netze=geaendert): out[n].add('ecke90')
    for n in enden_neu(H, H_neu, fuell, fuell_neu, netze=geaendert): out[n].add('enden')
    for n in ziel_verlassen(H, H_neu, netze=geaendert): out[n].add('ziel')
    for n in mitte_verlust(H, H_neu): out[n].add('mitte')
    for n in (laengen_verstoss(L0, H_neu, tor_netze) if tor_netze else {}): out[n].add('laenge')
    return {n: sorted(g) for n, g in sorted(out.items())}


def _lagen_union(fuell):
    from shapely.ops import unary_union
    z = collections.defaultdict(list)
    for f in fuell: z[(f['net'], f['layer'])].append(f['poly'])
    return {k: unary_union(v) for k, v in z.items()}


def _teile(g, amin=0.0):
    return [p for p in (g.geoms if hasattr(g, 'geoms') else [g]) if not p.is_empty and p.area > amin]


def zonen_tor(fuell_ref, fuell_neu, H_ref, H_neu, geaendert, weiss_max=0.1, streifen=0.1, streifen_min=0.01, flaeche_max=0.1, clr=0.15,
              rand=0.5, gesamt=False, umrisse=None, fuell_start=None, H_start=None):
    ""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    geaendert = set(n for n in geaendert if n)
    if not geaendert or not (fuell_ref or fuell_neu): return {}, []
    r6 = lambda p: (round(p[0], 6), round(p[1], 6))
    key = lambda t: (t['net'], t['layer'], tuple(sorted([r6(t['a']), r6(t['b'])])), round(t['w'], 6))
    alt = collections.Counter(key(t) for t in H_ref['tracks'] if t['net'] in geaendert)
    neu = collections.Counter(key(t) for t in H_neu['tracks'] if t['net'] in geaendert)
    diff = collections.defaultdict(list)
    for (n, L, (a, b), w) in list((alt - neu).elements()) + list((neu - alt).elements()):
        diff[L].append((n, (LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8)))
    Z0, Z1 = _lagen_union(fuell_ref), _lagen_union(fuell_neu)
    ZS = _lagen_union(fuell_start) if fuell_start is not None else None
    schuld = collections.defaultdict(set); befunde = []
    if gesamt:
        for (zn, L) in sorted(set(Z0) | set(Z1)):
            a0 = Z0[(zn, L)].area if (zn, L) in Z0 else 0.0; a1 = Z1[(zn, L)].area if (zn, L) in Z1 else 0.0
            if a1 < a0 - flaeche_max: befunde.append(dict(art='flaeche_gesamt', netz=zn, lage=L, vorher=round(a0, 3), nachher=round(a1, 3)))
    if not diff: return {}, befunde
    def schuldige(L, g, r, ohne=None):
        return sorted({n for n, s_ in diff.get(L, []) if n != ohne and s_.distance(g) < r})
    for L in sorted(diff):
        fenster = unary_union([s_ for _, s_ in diff[L]]).buffer(2.0, 8); fb = fenster.buffer(0.5, 8)
        zl = sorted({n for n, L2 in set(Z0) | set(Z1) if L2 == L})

        je_netz = collections.defaultdict(list)
        for n, s_ in diff[L]: je_netz[n].append(s_)
        kostet = collections.defaultdict(list)
        for n in sorted(je_netz):
            umk = unary_union(je_netz[n]).buffer(rand, 8)
            for zn in zl:
                if zn == n: continue
                a0 = Z0[(zn, L)].intersection(umk).area if (zn, L) in Z0 else 0.0
                a1 = Z1[(zn, L)].intersection(umk).area if (zn, L) in Z1 else 0.0
                aS = (ZS[(zn, L)].intersection(umk).area if (zn, L) in ZS else 0.0) if ZS is not None else a0
                if a1 < a0 - 1e-6: kostet[zn].append(n)
                if a1 < a0 - flaeche_max or (a1 < a0 - 1e-6 and a1 < aS - flaeche_max):
                    befunde.append(dict(art='flaeche', netz=zn, lage=L, schuld=[n], vorher=round(a0, 3), nachher=round(a1, 3),
                                        start=round(aS, 3)))
                    schuld[n].add('flaeche')
        if ZS is not None and H_start is not None:
            ks = collections.Counter(key(t) for t in H_start['tracks'] if t['layer'] == L)
            kn = collections.Counter(key(t) for t in H_neu['tracks'] if t['layer'] == L)
            seit = collections.defaultdict(list)
            for (m, L2, (a, b), w) in list((ks - kn).elements()) + list((kn - ks).elements()):
                seit[m].append((LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8))
            jetzt = {n: unary_union(g) for n, g in je_netz.items()}
            for m in sorted(seit):
                umk = unary_union(seit[m]).buffer(rand, 8)
                nahe = [n for n, g in jetzt.items() if g.intersects(umk)]
                if not nahe: continue
                for zn in zl:
                    if zn == m: continue
                    a0 = Z0[(zn, L)].intersection(umk).area if (zn, L) in Z0 else 0.0
                    a1 = Z1[(zn, L)].intersection(umk).area if (zn, L) in Z1 else 0.0
                    aS = ZS[(zn, L)].intersection(umk).area if (zn, L) in ZS else 0.0
                    if a1 < aS - flaeche_max and a1 < a0 - 1e-6:
                        befunde.append(dict(art='flaeche', netz=zn, lage=L, umkreis=m, schuld=sorted(nahe), vorher=round(a0, 3),
                                            nachher=round(a1, 3), start=round(aS, 3)))
                        for n in nahe: schuld[n].add('flaeche')
        if ZS is not None:
            for zn in zl:
                aS = ZS[(zn, L)].area if (zn, L) in ZS else 0.0; a1 = Z1[(zn, L)].area if (zn, L) in Z1 else 0.0
                a0 = Z0[(zn, L)].area if (zn, L) in Z0 else 0.0
                if a1 < aS - flaeche_max and a1 < a0 - 1e-6:


                    sch = sorted(kostet.get(zn) or je_netz)
                    befunde.append(dict(art='flaeche_gesamt', netz=zn, lage=L, schuld=sch, start=round(aS, 3), vorher=round(a0, 3), nachher=round(a1, 3)))
                    for n in sch: schuld[n].add('flaeche')

        cu0, cu1 = [], []
        for Hx, cu in ((H_ref, cu0), (H_neu, cu1)):
            for t in Hx['tracks']:
                if t['layer'] != L: continue
                g = (LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a']))
                if g.distance(fenster) > 0.5: continue
                cu.append(g.buffer(t['w'] / 2, 8))
            for v in Hx['vias']:
                if (not v.get('layers') or L in v['layers']) and Point(v['x'], v['y']).distance(fenster) <= 0.5:
                    cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2, 16))
            for p in Hx['pads']:
                if p['layer'] == L and len(p.get('poly') or []) >= 3:
                    g = Polygon(p['poly']).buffer(0)
                    if g.distance(fenster) <= 0.5: cu.append(g)
        board = unary_union([Polygon(e).buffer(0) for e in H_neu.get('edges', []) if len(e) > 2])
        innen = board.buffer(-0.5).intersection(fenster) if not board.is_empty else fenster

        gruppen = collections.defaultdict(list)
        for u in umrisse or ():
            if u['layer'] == L: gruppen[round(max(clr, u.get('clr', 0.0)), 4)].append(u['poly'])
        bereiche = [(c_, innen.intersection(unary_union(gs))) for c_, gs in sorted(gruppen.items())]
        bereiche = [(c_, b_) for c_, b_ in bereiche if not b_.is_empty]
        innen = unary_union([b_ for _, b_ in bereiche] or [Polygon()])
        sperr = unary_union([Polygon(k['poly']).buffer(0) for k in H_neu.get('keepouts', []) if L in (k.get('layers') or [])] or [Polygon()])
        def weiss(cu, Z):
            zf = [g.intersection(fb) for (n, L2), g in Z.items() if L2 == L]
            k = unary_union(cu + zf)
            return unary_union([b_.difference(k.buffer(c_ + 0.01, 8)).difference(sperr.buffer(c_)) for c_, b_ in bereiche] or [Polygon()])
        w0, w1 = (weiss(cu0, Z0), weiss(cu1, Z1)) if not innen.is_empty else (Polygon(), Polygon())
        r_w = max([0.4] + [c_ + 0.25 for c_, _ in bereiche])
        for g in _teile(w1.difference(w0.buffer(0.01)), weiss_max):
            c = g.representative_point(); sch = schuldige(L, g, r_w)
            befunde.append(dict(art='weiss', lage=L, mm2=round(g.area, 3), bei=[round(c.x, 2), round(c.y, 2)], schuld=sch))
            for n in sch: schuld[n].add('weiss')
        def duenn(g):
            return g.difference(g.buffer(-streifen / 2, 8).buffer(streifen / 2, 8)) if not g.is_empty else g
        for zn in sorted({n for n, L2 in Z1 if L2 == L}):
            g1 = Z1[(zn, L)].intersection(fb); g0 = Z0[(zn, L)].intersection(fb) if (zn, L) in Z0 else Polygon()
            s1, s0 = duenn(g1), duenn(g0)
            for g in _teile(s1.difference(s0.buffer(0.01)).intersection(fenster), streifen_min):
                c = g.representative_point(); sch = schuldige(L, g, 0.4, ohne=zn)
                befunde.append(dict(art='streifen', netz=zn, lage=L, mm2=round(g.area, 4), bei=[round(c.x, 2), round(c.y, 2)], schuld=sch))
                for n in sch: schuld[n].add('streifen')
    return {n: sorted(g) for n, g in sorted(schuld.items())}, befunde


VERSORGUNG = re.compile(r'(?i)(^|/)[+]|(^|[/_.])(vcc|vdd|vee|vbus|vbat|vio|vin|vsys|vref|v_dram)|(^|[/_.])p_?\d|(^|[/_.])\d+v\d*($|[_ ])|(^|[/_.])\d+v\d+')


def ebenen_hmax(H, fuell, anteil=0.3):
    ""
    from shapely.geometry import Polygon
    from nachbearbeitung.sperrnetze import MASSE
    ringe = [Polygon(e).buffer(0) for e in H.get('edges', []) if len(e) > 2]
    if not ringe or not fuell: return {}
    flaeche = max(g.area for g in ringe)
    je = collections.defaultdict(float)
    for f in fuell: je[f['layer']] += f['poly'].area
    ebenen = {L for L, a in je.items() if a >= anteil * flaeche}
    versorgung = {f['net'] for f in fuell if f['net'] and not MASSE.search(f['net'])}
    versorgung |= {t['net'] for t in H['tracks'] if t['net'] and VERSORGUNG.search(t['net']) and not MASSE.search(t['net'])
                   and not re.search(r'(?i)_EN$|ENABLE', t['net'])}
    out = collections.defaultdict(set)
    for t in H['tracks']:
        if t['net'] in versorgung and t['layer'] in ebenen: out[t['net']].add(t['layer'])
    return {n: sorted(v) for n, v in sorted(out.items())}


def fix_hash(H, fix):
    ""
    import hashlib
    r = lambda v: round(float(v), 6)
    T = sorted((t['uuid'], t['layer'], r(t['a'][0]), r(t['a'][1]), r(t['b'][0]), r(t['b'][1]), r(t['w'])) for t in H['tracks'] if t.get('uuid') in fix)
    return hashlib.sha256(json.dumps([len(fix), T]).encode()).hexdigest()


def via_sieb(H, D):
    ""
    from gemeinsam.kupfer_netz import anwenden
    u_net = {t['uuid']: t['net'] for t in H['tracks']}
    netze = sorted({a_['net'] for a_ in D['add']} | {u_net.get(u) for u in D['remove']} - {None})
    pads = collections.defaultdict(list)
    for p in H.get('pads', []):
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(p['poly'])
    def lagen(v, ts, stummel=False):
        c = (v['x'], v['y']); r = v['d'] / 2; out = set()
        for t in ts:
            if t['layer'] in out or (v.get('layers') and t['layer'] not in v['layers']): continue
            a, b = t['a'], t['b']
            if not stummel and math.dist(a, c) < r - 1e-6 and math.dist(b, c) < r - 1e-6: continue
            if math.dist(a, c) <= r + t['w'] / 2 + 1e-6 or math.dist(b, c) <= r + t['w'] / 2 + 1e-6 or \
                    (a != b and _strecke_abstand(c, a, b) <= r + t['w'] / 2 + 1e-6):
                out.add(t['layer'])
        return out
    raus = []
    for n in netze:
        alt = [t for t in H['tracks'] if t['net'] == n]; neu = anwenden(alt, D['remove'], [a_ for a_ in D['add'] if a_['net'] == n])
        def am_pad(v, L):
            from shapely.geometry import Point, Polygon
            return any(Polygon(q).buffer(0).distance(Point(v['x'], v['y'])) <= v['d'] / 2 + 1e-6 for q in pads.get((n, L), ()))
        def lose(v):
            if any(not am_pad(v, L) for L in lagen(v, alt) - lagen(v, neu)): return True
            pl = {L for L in (v.get('layers') or []) if am_pad(v, L)}
            return len(lagen(v, neu, True) | pl) < 2 <= len(lagen(v, alt, True) | pl)
        if any(lose(v) for v in H['vias'] if v['net'] == n): raus.append(n)
    if not raus: return D, []
    st = dict(D.get('stat') or {}, via_zurueck=raus)
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in raus], add=[a_ for a_ in D['add'] if a_['net'] not in raus], stat=st), raus


def _strecke_abstand(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _sehnen(D):
    ""
    if not any('m' in a_ for a_ in D.get('add', [])): return D
    add = []
    for a_ in D['add']:
        if 'm' not in a_: add.append(a_); continue
        b_ = {k_: v_ for k_, v_ in a_.items() if k_ not in ('m', 'uuid')}
        add += [dict(b_, b=a_['m']), dict(b_, a=a_['m'])]
    return dict(D, add=add)

def _ohne_netze(D, netze, H):
    if not netze: return D
    raus = set(netze); u_net = {t['uuid']: t['net'] for t in H['tracks']}
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in raus], add=[a_ for a_ in D['add'] if a_['net'] not in raus],
                add_generated=[g for g in D.get('add_generated', []) if (g.get('netz') if isinstance(g, dict) else None) not in raus])


def zonen_sieb(H, D, fuell, drc_=0.09):
    ""
    if not fuell or not D.get('add'): return D, []
    ZW = ZonenWaechter(H, fuell, drc=drc_); u_net = {t['uuid']: t['net'] for t in H['tracks']}
    rm = collections.defaultdict(list); ad = collections.defaultdict(list)
    for u in D['remove']: rm[u_net.get(u)].append(u)
    for a_ in D['add']: ad[a_['net']].append(a_)
    raus = []
    for n in sorted(set(rm) | set(ad), key=lambda x: x or ''):
        if ad.get(n) and ZW.pruefen(n, ad[n]): raus.append(n); continue
        ZW.commit(n, ad.get(n, []), rm.get(n, []))
    if not raus: return D, []
    st = dict(D.get('stat') or {}, zonen_zurueck=raus)
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in raus], add=[a_ for a_ in D['add'] if a_['net'] not in raus], stat=st), raus


def laufvertrag(ein, pro, dru, o):
    ""
    import hashlib, platform, shapely
    def sha(p): return hashlib.sha256(open(p, 'rb').read()).hexdigest()
    git = lambda *x: subprocess.run(['git', '-C', A] + list(x), capture_output=True, text=True).stdout.strip()
    cli = subprocess.run([CLI, '--version'], capture_output=True, text=True).stdout.strip()
    return dict(commit=git('rev-parse', 'HEAD'), arbeitsbaum_diff_sha256=hashlib.sha256(git('diff', 'HEAD').encode()).hexdigest(),
                optionen={k: (sorted(v) if isinstance(v, set) else v) for k, v in o.items() if not k.startswith('_')},
                eingabe_sha256=dict(pcb=sha(ein), pro=sha(pro), dru=sha(dru)), python=platform.python_version(),
                shapely=shapely.__version__, kicad_cli=cli)


def schritt_ecken45(H, S, hin, o, cur, tmp):
    ""
    from nachbearbeitung.ecken45 import ecken45
    fl = [(f['net'], f['layer'], f['poly']) for f in (o.get('_fuell') or [])]
    return ecken45(H, sperren=S, netze=o.get('_netze'), fix=o.get('_fix', ()), hin=hin, fuell=fl, cmax=float(o.get('ecken_cmax', 0.4)))


def schritt_maeander(H, S, hin, o, cur, tmp):
    ""
    from nachbearbeitung.maeander import maeander_aenderung
    zurueck = {n for n, g in S.items() if g == 'zurückgenommen'}
    D = maeander_aenderung(H, hin, o.get('_paare') or [], o, sperren=zurueck)
    return D



GR_POLY = re.compile(r'\n\t\(gr_poly\n.*?\n\t\)', re.S)


def formen_generiert(text, max_mm2=0.2):
    ""
    out = []
    for m in GR_POLY.finditer(text):
        b = m.group(0)
        n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', b); L = re.search(r'\(layer "([^"]+\.Cu)"\)', b)
        if not n or not n.group(1) or not L: continue
        pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-0-9.]+) ([-0-9.]+)\)', b)]
        if len(pts) < 3: continue
        A = abs(sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))) / 2
        if A < max_mm2:
            u = re.search(r'\(uuid "([^"]+)"\)', b)
            out.append((m.start(), m.end(), u.group(1) if u else None, n.group(1), L.group(1), pts, b))
    return out


def formen_entfernen(text, max_mm2=0.2, nur=None):
    ""
    teile = []; pos = 0; n = 0
    for a, e, u, *_r in formen_generiert(text, max_mm2):
        if nur is not None and u not in nur: continue
        teile.append(text[pos:a]); pos = e; n += 1
    teile.append(text[pos:])
    return ''.join(teile), n


def tropfen_neu(src, dst, pro, dru, o, tmp):
    ""
    from gemeinsam.kupfer_netz import zonen_fuellung as zf
    clr = max([f.get('clr') or 0.0 for f in zf(open(src, encoding='utf-8').read())] + [0.0]) or 0.2
    frei = o.get('tropfen_frei') or '%g' % round(max(float(o.get('drc', 0.09)), float(o.get('drcpad', 0.10))) + 0.012, 3)
    freiz = o.get('tropfen_freiz') or '%g' % round(clr + 0.005, 3)
    rand, loch = o.get('kante', '0.2'), o.get('loch', '0.25'); kl = ['klassen=' + o['klassen']] if o.get('klassen') else []
    topt = ['r=' + o.get('tropfen_r', '0.4'), 'frei=' + freiz, 'frei_hart=' + freiz, 'rand=' + rand, 'loch=' + loch] + kl
    def lauf(cmd, name):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0: raise RuntimeError('%s: %s' % (name, r.stderr[-600:]))
        return (r.stdout.strip().split('\n') or [''])[-1]
    def ex(pcb, k):
        j = lambda x: os.path.join(tmp, '%s_%s.json' % (x, k))
        lauf([KPY, os.path.join(A, 'kicad', 'fillet_nachlauf.py'), 'export', pcb, j('k')], 'fillet_nachlauf export')
        lauf([KPY, os.path.join(A, 'kicad', 'kupfer_export.py'), pcb, j('cu')], 'kupfer_export')
        lauf([KPY, os.path.join(A, 'kicad', 'sperr_export.py'), pcb, j('sperr')], 'sperr_export')
        return j('k'), j('sperr'), j('cu')
    def anh(pcb, formen, aus):
        x = lauf([KPY, os.path.join(A, 'kicad', 'formen_anhaengen.py'), pcb, formen, aus], 'formen_anhaengen')
        b = aus[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru'); return x
    log_ = {}
    z = [os.path.join(tmp, 'tr_z%d.kicad_pcb' % i) for i in range(4)]
    k, sp, cu = ex(src, 0)
    log_['signal'] = lauf([VPY, os.path.join(A, 'nachbearbeitung', 'weiss_tropfen.py'), k, sp, os.path.join(tmp, 'tr_t1.json')] + topt +
                          ['netze=signal', 'zonen=weich', 'pads=' + cu], 'weiss_tropfen signal')
    anh(src, os.path.join(tmp, 'tr_t1.json'), z[1])
    k, sp, cu = ex(z[1], 1)
    log_['flaeche'] = lauf([VPY, os.path.join(A, 'nachbearbeitung', 'weiss_tropfen.py'), k, sp, os.path.join(tmp, 'tr_t2.json')] + topt +
                           ['netze=flaeche', 'pads=' + cu], 'weiss_tropfen flaeche')
    anh(z[1], os.path.join(tmp, 'tr_t2.json'), z[2])
    k, sp, cu = ex(z[2], 2)
    log_['kerben'] = lauf([VPY, os.path.join(A, 'nachbearbeitung', 'kerben_fuellen.py'), k, sp, os.path.join(tmp, 'tr_t3.json'),
                           'frei_hart=' + frei, 'frei=' + freiz, 'rand=' + rand, 'loch=' + loch, 'pads=' + cu] + kl, 'kerben_fuellen')
    anh(z[2], os.path.join(tmp, 'tr_t3.json'), z[3])
    log_['lose'] = lauf([KPY, os.path.join(A, 'kicad', 'tropfen_lose_weg.py'), z[3], dst], 'tropfen_lose_weg')
    b = dst[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')
    return dict(log_, frei=frei, freiz=freiz, rand=rand, loch=loch)


def formen_passend(alte, H, text, r=0.5, rest=0.002):
    ""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    from gemeinsam.kupfer_netz import zonen_fuellung as zf
    Lk = lambda L: L[:-3] if L.endswith('.Cu') else L
    cu = collections.defaultdict(list)
    for t in H['tracks']: cu[(t['net'], t['layer'])].append(LineString([t['a'], t['b']]).buffer(t['w'] / 2, 16) if t['a'] != t['b'] else Point(t['a']).buffer(t['w'] / 2))
    for p_ in H.get('pads', []):
        if len(p_.get('poly') or []) >= 3: cu[(p_['net'], p_['layer'])].append(Polygon(p_['poly']).buffer(0))
    lagen = {t['layer'] for t in H['tracks']} | {p_['layer'] for p_ in H.get('pads', [])}
    for v in H.get('vias', []):
        for L in v.get('layers') or lagen: cu[(v['net'], L)].append(Point(v['x'], v['y']).buffer(v['d'] / 2, 32))
    fuell = collections.defaultdict(list)
    for f in zf(text): fuell[(f['net'], f['layer'])].append(f['poly'])
    umr = collections.defaultdict(list)
    for z in zonen_umrisse(text): umr[(z['net'], z['layer'])].append(z['poly'])
    cache = {}
    def geo(n, L):
        if (n, L) not in cache:
            c = unary_union(cu[(n, L)]) if cu[(n, L)] else None
            cache[(n, L)] = (c, c.buffer(r, 16).buffer(-r, 16) if c is not None else None,
                             unary_union(fuell[(n, L)]) if fuell[(n, L)] else None, unary_union(umr[(n, L)]) if umr[(n, L)] else None)
        return cache[(n, L)]
    passend, lose, verschoben = [], [], []
    for x in alte:
        n, L, pts = x[3], Lk(x[4]), x[5]
        F = Polygon(pts).buffer(0)
        c, zu, fu, um = geo(n, L)
        an = [g for g in (c, fu) if g is not None]
        if not an or not any(g.buffer(1e-4).intersects(F) for g in an): lose.append(x); continue
        innen = unary_union([g for g in (zu, um) if g is not None])
        if F.difference(innen.buffer(1e-4)).area > rest: verschoben.append(x); continue
        passend.append(x)
    return passend, lose, verschoben


def formen_einsetzen(text, bloecke):
    i = text.find('\n\t(zone')
    if i < 0: i = text.rstrip().rfind(')')
    return text[:i] + ''.join(bloecke) + text[i:]


def tropfen_phase(cur, ziel, pro, dru, ref, o, tmp, runden=3, alte=None, H=None):
    ""
    info0 = {}
    if alte:
        text = open(cur, encoding='utf-8').read()
        passend, lose, versch = formen_passend(alte, H, text)
        info0 = dict(alte=len(alte), alte_behalten=len(passend), alte_lose=len(lose), alte_verschoben=len(versch),
                     lose_bei=[[round(sum(q[0] for q in x[5]) / len(x[5]), 2), round(sum(q[1] for q in x[5]) / len(x[5]), 2), x[3]] for x in lose][:30],
                     verschoben_bei=[[round(sum(q[0] for q in x[5]) / len(x[5]), 2), round(sum(q[1] for q in x[5]) / len(x[5]), 2), x[3]] for x in versch][:30])
        log('   alte Tropfen: %d passen, %d lose, %d verschoben (nur die passenden kommen zurück)' % (len(passend), len(lose), len(versch)))
        mit = os.path.join(tmp, 'tr_alt.kicad_pcb'); kopie(cur, mit, pro, dru)
        open(mit, 'w', encoding='utf-8').write(formen_einsetzen(text, [x[6] for x in passend]))
        R_ = drc(mit, pro, dru); cur = mit
    info = dict(info0, **tropfen_neu(cur, ziel, pro, dru, o, tmp))
    text = open(ziel, encoding='utf-8').read(); info['formen'] = len(formen_generiert(text)); info['zurueck'] = 0
    for runde in range(runden + 1):
        R = drc(ziel, pro, dru); nev = neue_ereignisse(R, ref)
        log('   Tropfen: %d Formen, DRC Fehler %s, offen %d, neue Meldungen %d' % (info['formen'] - info['zurueck'], dict(R['E']) or 0, R['U'], len(nev)))
        if gut(R, ref): return True, R, info
        if runde == runden: break
        from shapely.geometry import Point, Polygon
        orte = [Point(i['pos']['x'], i['pos']['y']) for v in nev for i in v.get('items', []) if i.get('pos')]
        text = open(ziel, encoding='utf-8').read()
        weg = {u for _, _, u, n, L, pts, _b in formen_generiert(text) if u and any(Polygon(pts).buffer(0).distance(q) < 0.3 for q in orte)}
        if not weg: break
        text, n = formen_entfernen(text, nur=weg); info['zurueck'] += n
        open(ziel, 'w', encoding='utf-8').write(text)
        log('   Tropfen: %d Formen an neuen Meldungen zurück' % n)
    return False, R, info


STANDARD_SCHRITTE = 'aufraeumen,abkuerzen,glaetten,mitte,ecken45,aufraeumen,maeander'
FORM_AUSNAHMEN = dict(maeander=('winkel', 'haarnadel'))
ZONEN_AUSNAHMEN = dict(maeander=('flaeche', 'weiss'))
SCHRITTE = dict(aufraeumen=schritt_aufraeumen, abkuerzen=schritt_abkuerzen, glaetten=schritt_glaetten, mitte=schritt_mitte,
                ecken45=schritt_ecken45, maeander=schritt_maeander)


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    ein = os.path.abspath(a[0]); aus = os.path.abspath(a[1])
    if os.path.realpath(os.path.dirname(ein)) == os.path.realpath(aus):
        raise SystemExit('Ausgabeordner muss ein anderer Ordner als der der Eingabe sein')
    os.makedirs(aus, exist_ok=True)
    pro, dru = ein[:-10] + '.kicad_pro', ein[:-10] + '.kicad_dru'
    if not (os.path.exists(pro) and os.path.exists(dru)): raise SystemExit('.kicad_pro/.kicad_dru fehlen neben ' + ein)
    o['_dru'] = dru
    if o.get('netze'): o['_netze'] = [n for n in o['netze'].split(';') if n]
    schritte = o.get('schritte', STANDARD_SCHRITTE).split(',')
    sieb = int(o.get('sieb', 3)); runden = int(o.get('runden', 3))
    tmp = os.path.join(aus, 'tmp'); os.makedirs(tmp, exist_ok=True)
    cur = os.path.join(aus, 'f00_start.kicad_pcb'); kopie(ein, cur, pro, dru)


    text_ein = open(ein, encoding='utf-8').read(); tmax = float(o.get('tropfen_max', 0.2))
    n_gen = len(formen_generiert(text_ein, tmax))
    tropfen_an = o.get('tropfen', 'auto') == '1' or (o.get('tropfen', 'auto') == 'auto' and n_gen > 0)
    R_ein = None; alte_formen = []
    if tropfen_an:
        if n_gen:
            vor = os.path.join(aus, 'f00_eingabe.kicad_pcb'); kopie(ein, vor, pro, dru)
            log('Eingabe mit %d generierten Kupferformen (Tropfen): Zonen neu füllen, DRC' % n_gen, vor)
            R_ein = drc(vor, pro, dru)
            log('Eingabe: Fehler %s, offen %d, Warnungen %d' % (dict(R_ein['E']) or 0, R_ein['U'], sum(R_ein['W'].values())))
            alte_formen = formen_generiert(text_ein, tmax)
            text0, n_weg = formen_entfernen(text_ein, tmax)
            open(cur, 'w', encoding='utf-8').write(text0)
            log('Tropfen: %d generierte Formen entfernt (unter %g mm²), am Ende neu' % (n_weg, tmax))
        else:
            log('Tropfen: Eingabe ohne generierte Formen, am Ende neu (tropfen=1)')
    log('Start: Zonen neu füllen, DRC', cur)
    R0 = drc(cur, pro, dru); ref = R0; E0, U0, W0 = R0['E'], R0['U'], R0['W']
    log('Ausgangsboard: Fehler %s, offen %d, Warnungen %d %s' % (dict(E0) or 0, U0, sum(W0.values()), dict(W0)))
    H0 = hind(cur, os.path.join(aus, 'h_start.json'), [0, 0, 1000, 1000]); box = rahmen(H0)

    U = sperrumfang(H0, pcb=cur, pro=pro, dru=dru, klassen=o.get('klassen'), sperren=o.get('sperren'))
    S = U['netze']; o['_fix'] = set(U['bahnen'])
    tor_netze = sorted((set(U['gruppen']) | set(U['paare'])) - set(S))
    o['_tor_netze'] = tor_netze
    if 'maeander' in schritte:
        from gemeinsam.paarnetze import paare as _paare
        from nachbearbeitung.maeander import skew_regeln, toleranz
        _sr = skew_regeln(open(dru, encoding='utf-8').read())
        o['_paare'] = [(p_, n_, toleranz(p_, n_, _sr, float(o.get('skew_tol', 0.1))))
                       for p_, n_ in _paare(sorted({t['net'] for t in H0['tracks'] if t['net']}), o.get('klassen'))]
        log('Mäander: %d Paare:' % len(o['_paare']), ', '.join('%s (±%g)' % (p_.split('/')[-1], t_) for p_, n_, t_ in o['_paare']))
    json.dump(dict(S, **{'#bahnen': U['bahnen'], '#laengengruppen': U['gruppen']}), open(os.path.join(aus, 'sperrnetze.json'), 'w'),
              indent=1, ensure_ascii=False)
    log('gesperrt:', len(S), 'Netze:', ', '.join('%s (%s)' % (n.split('/')[-1], g.split(' ')[0]) for n, g in S.items()))
    log('feste Bahnen (Kelvin):', len(U['bahnen']), '; Längentor für', len(tor_netze), 'Netze:', ', '.join(n.split('/')[-1] for n in tor_netze)[:400])
    from nachbearbeitung.sperrnetze import _pcb_lesen
    o['_zonennetze'] = _pcb_lesen(cur)[2]
    for n_ in U.get('ohne_pfad', []): log('WARNUNG: Kelvin-Messleitung ohne Kupferweg vom IC zum Shunt, Netz ganz gesperrt:', n_)
    hash0 = geo_hash(H0, S); fix0 = fix_hash(H0, o['_fix'])
    from analyse.fertig_kennzahlen import laengen
    L0 = laengen(H0); o['_L0'] = L0

    o['_hmax_lagen'] = ebenen_hmax(H0, _zonen_fuellung(open(cur, encoding='utf-8').read())) if float(o.get('hmax_ebene', 0.3)) > 0 else {}
    if o['_hmax_lagen']: log('Versorgung auf Ebenenlagen (hmax %s):' % o.get('hmax_ebene', '0.3'),
                             ', '.join('%s %s' % (n.split('/')[-1], '/'.join(Ls)) for n, Ls in o['_hmax_lagen'].items())[:400])
    protokoll = []; k = 0; H = H0; maeander_netze = set()
    zurueck = collections.defaultdict(set)
    fehler = collections.defaultdict(lambda: collections.defaultdict(set))
    zonentor_an = o.get('zonentor', '1') == '1'; schritt_nr = 0
    fuell_start = _zonen_fuellung(open(cur, encoding='utf-8').read())
    for name in schritte:
        for rnd in range(runden if name in ('abkuerzen', 'mitte') else 1):
            text_cur = open(cur, encoding='utf-8').read()
            fuell = _zonen_fuellung(text_cur)
            umrisse = zonen_umrisse(text_cur)
            o['_flaechen'] = [(f['net'], f['layer'], f['poly'], f['clr']) for f in fuell if f['net'] in S]
            o['_fuell'] = fuell
            hin = Hindernisse(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', 0.09)),
                              drcvia=float(o.get('drcvia', o.get('drc', 0.09))), loch=float(o.get('loch', 0.25)), kante=float(o.get('kante', 0.2)),
                              netzabstand=netz_abstaende(dru, {t['net'] for t in H['tracks']}), flaechen=o['_flaechen'])
            schritt_nr += 1; sname = '%02d %s R%d' % (schritt_nr, name, rnd + 1)
            log('Schritt %s%s' % (name, ' Runde %d' % (rnd + 1) if rnd else ''))
            D = SCHRITTE[name](H, dict(S, **{n: 'zurückgenommen' for n in zurueck[name]}), hin, o, cur, tmp)
            zk = []
            if name != 'abkuerzen':
                _, zk = zonen_sieb(H, _sehnen(D), fuell, float(o.get('drc', 0.09))); D = _ohne_netze(D, zk, H)
                if zk: log('   Zonen: zurück (zerschneidet fremde Zone oder kostet Füllung):', ', '.join(sorted(n.split('/')[-1] for n in zk)))
                for n_ in zk: fehler[sname]['zonen_sieb'].add(n_)
            _, vk = via_sieb(H, _sehnen(D)); D = _ohne_netze(D, vk, H)
            if vk: log('   Vias: zurück (Via verlöre eine Lage):', ', '.join(sorted(n.split('/')[-1] for n in vk)))
            for n_ in vk: fehler[sname]['via_sieb (loses Via)'].add(n_)
            D['_uuid_netz'] = {t['uuid']: t['net'] for t in H['tracks']}
            log('   Vorschlag: entfernt %d, neu %d' % (len(D['remove']), len(D['add'])), json.dumps(D.get('stat'), ensure_ascii=False)[:600])
            if not D['remove'] and not D['add']:
                if zk or vk: protokoll.append(dict(schritt=name, runde=rnd + 1, nr=schritt_nr, uebernommen=False, grund='alles ausgesiebt',
                                                   stat=D.get('stat')))
                break
            ziel = os.path.join(aus, 'f%02d_%s.kicad_pcb' % (k + 1, name))
            ok, Dk, info = anwenden_pruefen(cur, D, ziel, pro, dru, ref, sieb)
            R_neu = info.pop('_R', None)
            protokoll.append(dict(schritt=name, runde=rnd + 1, nr=schritt_nr, vorschlag=dict(entfernt=len(D['remove']), neu=len(D['add'])), stat=D.get('stat'),
                                  uebernommen=ok, uebernommen_aenderung=dict(entfernt=len(Dk['remove']), neu=len(Dk['add'])) if ok else None, **info))
            zurueck[name] |= set(info.get('zurueck') or [])
            for n_ in info.get('zurueck') or []: fehler[sname]['drc_sieb'].add(n_)
            if not ok:
                log('   verworfen:', info.get('grund')); break
            H_neu = hind(ziel, os.path.join(tmp, 'h_%02d.json' % (k + 1)), [0, 0, 1000, 1000])


            t0 = form_tor(H, H, L0, tor_netze)
            gruende_alle = {}; zonen_befunde = []
            for versuch in range(3):
                geaendert = {s_['net'] for s_ in Dk['add']} | {D['_uuid_netz'].get(u) for u in Dk['remove']} - {None}
                fuell_neu = _zonen_fuellung(open(ziel, encoding='utf-8').read())
                gr = {n_: ['formtor:' + g_ for g_ in v_ if g_ not in FORM_AUSNAHMEN.get(name, ())]
                      for n_, v_ in form_gruende(H, H_neu, L0, tor_netze, geaendert, fuell, fuell_neu).items()}
                gr = {n_: g_ for n_, g_ in gr.items() if g_}
                if zonentor_an:
                    zt, bef = zonen_tor(fuell, fuell_neu, H, H_neu, geaendert, weiss_max=float(o.get('weiss_max', 0.1)),
                                        streifen=float(o.get('streifen', 0.1)), flaeche_max=float(o.get('flaeche_max', 0.1)), umrisse=umrisse,
                                        fuell_start=fuell_start, H_start=H0)
                    zonen_befunde += bef
                    for n_, g_ in zt.items():
                        g_ = [x for x in g_ if x not in ZONEN_AUSNAHMEN.get(name, ())]
                        if g_: gr.setdefault(n_, []).extend('zonentor:' + x for x in g_)
                if not gr: break
                for n_, g_ in gr.items():
                    gruende_alle.setdefault(n_, []).extend(x for x in g_ if x not in gruende_alle.get(n_, []))
                    for x in g_: fehler[sname][x].add(n_)
                log('   Formtor/Zonentor: nehme zurück:', '; '.join('%s (%s)' % (n_.split('/')[-1], ', '.join(g_)) for n_, g_ in sorted(gr.items()))[:900])
                zurueck[name] |= set(gr)
                u_net = D['_uuid_netz']
                D2 = dict(D, remove=[u for u in Dk['remove'] if u_net.get(u) not in gr], add=[a_ for a_ in Dk['add'] if a_['net'] not in gr],
                          add_generated=[g for g in Dk.get('add_generated', []) if g.get('netz') not in gr])
                ok2 = bool(D2['remove'] or D2['add'])
                if ok2:
                    ok2, Dk, info2 = anwenden_pruefen(cur, D2, ziel, pro, dru, ref, sieb); R_neu = info2.pop('_R', None)
                    for n_ in info2.get('zurueck') or []: fehler[sname]['drc_sieb'].add(n_); zurueck[name].add(n_)
                if not ok2: break
                H_neu = hind(ziel, os.path.join(tmp, 'h_%02d.json' % (k + 1)), [0, 0, 1000, 1000])
            else:
                ok2 = False
            if gruende_alle: protokoll[-1].update(formtor_zurueck=sorted(gruende_alle), formtor_gruende=gruende_alle)
            if zonen_befunde: protokoll[-1]['zonentor_befunde'] = zonen_befunde[:50]
            if gruende_alle and not ok2:
                log('   verworfen: Formtor/Zonentor'); protokoll[-1].update(uebernommen=False, grund='Formtor/Zonentor'); break
            t1 = form_tor(H_neu, H, L0, tor_netze)
            if gruende_alle: protokoll[-1].update(uebernommen_aenderung=dict(entfernt=len(Dk['remove']), neu=len(Dk['add'])))
            if name == 'maeander':
                maeander_netze.update(s_['net'] for s_ in Dk['add']); maeander_netze.update(D['_uuid_netz'].get(u) for u in Dk['remove'])
                maeander_netze.discard(None)
            k += 1; cur = ziel; H = H_neu; ref = R_neu
            log('   übernommen ->', os.path.basename(cur), 'Formtor', t0, '->', t1)
    tropfen_info = None; vor_tropfen = cur
    if tropfen_an:
        log('Tropfen neu (zonen.sh Schritt 6)')
        ziel = os.path.join(aus, 'f%02d_tropfen.kicad_pcb' % (k + 1))
        ok_t, R_t, tropfen_info = tropfen_phase(cur, ziel, pro, dru, ref, o, tmp, alte=alte_formen,
                                                H=hind(cur, os.path.join(tmp, 'h_vor_tropfen.json'), [0, 0, 1000, 1000]) if alte_formen else None)
        tropfen_info['uebernommen'] = ok_t
        protokoll.append(dict(schritt='tropfen', runde=1, nr=schritt_nr + 1, uebernommen=ok_t, stat=tropfen_info))
        if ok_t: k += 1; cur = ziel; ref = R_t; log('   übernommen ->', os.path.basename(cur))
        else: log('   verworfen: Tropfen brachten neue DRC-Meldungen; Ergebnis ohne generierte Tropfen')
    fertig = os.path.join(aus, 'fertig.kicad_pcb'); kopie(cur, fertig, pro, dru)
    R1 = drc(fertig, pro, dru); E1, U1, W1 = R1['E'], R1['U'], R1['W']
    H1 = hind(fertig, os.path.join(aus, 'h_fertig.json'), [0, 0, 1000, 1000])

    NK0, NK1 = Netzkupfer(H0), Netzkupfer(H1)
    getrennt = {}
    for net in sorted({t['net'] for t in H0['tracks'] if t['net']}):
        k0, k1 = NK0.komponenten(net), NK1.komponenten(net)
        g = collections.defaultdict(set)
        for pid, c in k0.items(): g[c].add(pid)
        sch = [sorted(ms) for ms in g.values() if len({k1.get(m) for m in ms}) > 1]
        if sch: getrennt[net] = sch
    S_hash = {n: g for n, g in S.items() if n not in maeander_netze}
    hash0 = geo_hash(H0, S_hash); hash1 = geo_hash(H1, S_hash); fix1 = fix_hash(H1, o['_fix'])
    from analyse.fertig_kennzahlen import laengen_verstoss, mitte_verlust
    mv = mitte_verlust(H0, H1); lv = laengen_verstoss(L0, H1, tor_netze)

    from analyse.fertig_kennzahlen import ecken90_neu, enden_neu, ziel_verlassen


    mit_eingabe = bool(R_ein and tropfen_info and tropfen_info.get('uebernommen'))
    fuell0 = _zonen_fuellung(open(os.path.join(aus, 'f00_eingabe.kicad_pcb' if mit_eingabe else 'f00_start.kicad_pcb'), encoding='utf-8').read())
    schluss = vor_tropfen if tropfen_info and tropfen_info.get('uebernommen') and not mit_eingabe else fertig
    fuell1 = _zonen_fuellung(open(schluss, encoding='utf-8').read())
    geaendert_alle = {t['net'] for t in H0['tracks']} | {t['net'] for t in H1['tracks']}
    en = enden_neu(H0, H1, fuell0, fuell1)
    e90 = {'%s %s' % k_: z_ for k_, z_ in sorted(ecken90_neu(H0, H1).items())}
    zv = ziel_verlassen(H0, H1)
    _, zbef = zonen_tor(fuell0, fuell1, H0, H1, geaendert_alle, weiss_max=float(o.get('weiss_max', 0.1)), streifen=float(o.get('streifen', 0.1)),
                        flaeche_max=float(o.get('flaeche_max', 0.1)), gesamt=True,
                        umrisse=zonen_umrisse(open(fertig, encoding='utf-8').read())) if o.get('zonentor', '1') == '1' else ({}, [])


    def _mae(b):
        if not maeander_netze: return False
        sch = set(b.get('schuld') or [])
        if b['art'] in ('weiss', 'flaeche') and sch and sch <= maeander_netze: return True
        if b['art'] == 'flaeche_gesamt' and not sch:
            fl = [x for x in zbef if x['art'] == 'flaeche' and (x.get('netz'), x.get('lage')) == (b.get('netz'), b.get('lage'))]
            return bool(fl) and all(set(x.get('schuld') or []) and set(x['schuld']) <= maeander_netze for x in fl)
        return False
    zbef_mae = [b for b in zbef if _mae(b)]; zbef = [b for b in zbef if not _mae(b)]
    ende = dict(enden_neu=en, ecken90_neu=e90, ziel_verlassen=zv, zonen_befunde=zbef, maeander_befunde=zbef_mae,
                streifen_neu=sum(1 for b in zbef if b['art'] == 'streifen'), weiss_neu=sum(1 for b in zbef if b['art'] == 'weiss'),
                flaeche_verlust=[b for b in zbef if b['art'] in ('flaeche', 'flaeche_gesamt')])
    sieb_netze = sorted({n for p_ in protokoll for n in (p_.get('zurueck') or [])})
    zaehler = dict(sieb_eingriffe=sum(1 for p_ in protokoll if p_.get('zurueck')), sieb_zurueck_netze=len(sieb_netze), sieb_netze=sieb_netze,
                   formtor_zurueck_netze=sorted({n for p_ in protokoll for n in (p_.get('formtor_zurueck') or [])}),
                   zonen_zurueck_netze=sorted({n for gg in fehler.values() for n in gg.get('zonen_sieb', ())}),
                   via_zurueck_netze=sorted({n for gg in fehler.values() for n in gg.get('via_sieb (loses Via)', ())}),
                   mitte_verloren=sum(mv.values()), mitte_verloren_je_netz=mv, laengentor_verstoesse=lv,
                   werkzeugfehler={st: {g: sorted(ns) for g, ns in sorted(gg.items())} for st, gg in fehler.items() if gg},
                   werkzeugfehler_summe=dict(collections.Counter(g.split(':')[0] if g.startswith('zonentor') else g
                                                                 for gg in fehler.values() for g, ns in gg.items() for _ in ns)),
                   formtor_eingriffe_je_grund=dict(collections.Counter(g for gg in fehler.values() for g, ns in gg.items()
                                                                       if g.startswith(('formtor', 'zonentor')) for _ in ns)))

    def flaeche(fl, je_lage=False):
        z = collections.defaultdict(float)
        for f in fl: z[(f['net'], f['layer']) if je_lage else f['net']] += f['poly'].area
        return z
    z0, z1 = flaeche(fuell0), flaeche(fuell1)
    zonen_diff = {n: dict(vorher_mm2=round(z0[n], 2), nachher_mm2=round(z1[n], 2), diff_mm2=round(z1[n] - z0[n], 2))
                  for n in sorted(set(z0) | set(z1)) if abs(z1[n] - z0[n]) > 0.05}
    zl0, zl1 = flaeche(fuell0, True), flaeche(fuell1, True)
    zonen_lage = {'%s %s' % k_: dict(vorher_mm2=round(zl0[k_], 3), nachher_mm2=round(zl1[k_], 3), diff_mm2=round(zl1[k_] - zl0[k_], 3))
                  for k_ in sorted(set(zl0) | set(zl1)) if abs(zl1[k_] - zl0[k_]) > 0.01}
    from analyse.fertig_kennzahlen import messen
    kz = dict(vorher=dict(alle=messen(H0), ohne_gesperrte=messen(H0, S)), nachher=dict(alle=messen(H1), ohne_gesperrte=messen(H1, S)))
    bericht = dict(eingabe=ein, ausgabe=fertig, schritte=protokoll, gesperrt=S,
                   drc_vorher=dict(fehler=dict(E0), offen=U0, warnungen=dict(W0)), drc_nachher=dict(fehler=dict(E1), offen=U1, warnungen=dict(W1)),
                   drc_ok=gut(R1, R0), drc_sauber=U1 == 0 and not E1, drc_neue_meldungen=len(neue_ereignisse(R1, R0)), verbindung_getrennt=getrennt,
                   drc_eingabe=dict(fehler=dict(R_ein['E']), offen=R_ein['U'], warnungen=dict(R_ein['W'])) if R_ein else None,
                   drc_neu_gegen_eingabe=len(neue_ereignisse(R1, R_ein)) if R_ein else None, tropfen=tropfen_info,
                   maeander=dict(netze=sorted(maeander_netze), skew_fehler_vorher=E0.get('skew_out_of_range', 0),
                                 skew_fehler_nachher=E1.get('skew_out_of_range', 0),
                                 paare=next((p_['stat'].get('paare') for p_ in protokoll if p_['schritt'] == 'maeander' and p_.get('uebernommen')), None)),
                   gesperrt_hash=dict(vorher=hash0, nachher=hash1, gleich=hash0 == hash1),
                   feste_bahnen=dict(anzahl=len(o['_fix']), vorher=fix0, nachher=fix1, gleich=fix0 == fix1), laengentor_netze=tor_netze,
                   zaehler=zaehler, zonenflaeche=zonen_diff, zonenflaeche_je_lage=zonen_lage, ende=ende, kennzahlen=kz,
                   lauf=laufvertrag(ein, pro, dru, o))
    json.dump(bericht, open(os.path.join(aus, 'bericht.json'), 'w'), indent=1, ensure_ascii=False)
    log('FERTIG: DRC Fehler %s, offen %d, Warnungen %d (vorher %d), neue Meldungen %d; getrennt %d Netze; gesperrte unverändert: %s, '
        'feste Bahnen unverändert: %s; Sieb nahm %d Netze zurück (%d Eingriffe); Mitte verloren %d; Längentor-Verstösse %d'
        % (dict(E1) or 0, U1, sum(W1.values()), sum(W0.values()), bericht['drc_neue_meldungen'], len(getrennt), hash0 == hash1, fix0 == fix1,
           zaehler['sieb_zurueck_netze'], zaehler['sieb_eingriffe'], zaehler['mitte_verloren'], len(lv)))
    log('  Werkzeugfehler je Grund:', json.dumps(zaehler['werkzeugfehler_summe'], ensure_ascii=False),
        '; neu gegenüber Start: Versätze/lose Enden %d, 90°-Ecken %d, Pad/Via verlassen %d, Streifen %d, Weiss %d, Flächenverlust %d'
        % (sum(en.values()), sum(e90.values()), sum(zv.values()), ende['streifen_neu'], ende['weiss_neu'], len(ende['flaeche_verlust'])))
    for teil in ('alle', 'ohne_gesperrte'):
        v, n = kz['vorher'][teil], kz['nachher'][teil]
        log('  %s: Länge %.1f -> %.1f, Segmente %d -> %d, Züge %d -> %d, Knicke %d -> %d, Treppen %s -> %s, Mikro<0,08 %d -> %d, R37 Via %s -> %s, Pad %s -> %s'
            % (teil, v['laenge_mm'], n['laenge_mm'], v['segmente'], n['segmente'], v['wackel']['zuege'], n['wackel']['zuege'], v['r18']['knicke'],
               n['r18']['knicke'], v['treppen'].get('treppen'), n['treppen'].get('treppen'), v['mikro']['unter_0_08'], n['mikro']['unter_0_08'],
               v['r37'].get('via_neben_mitte', 0), n['r37'].get('via_neben_mitte', 0), v['r37'].get('pad_neben_mitte', 0), n['r37'].get('pad_neben_mitte', 0)))
    form_ok = not en and not e90 and not zv and not ende['streifen_neu'] and not ende['weiss_neu'] and not ende['flaeche_verlust']
    return 0 if bericht['drc_ok'] and form_ok and not getrennt and hash0 == hash1 and fix0 == fix1 and not mv and not lv else 1


if __name__ == '__main__':
    sys.exit(main())
