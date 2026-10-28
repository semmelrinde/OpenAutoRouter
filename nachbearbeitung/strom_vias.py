""
import json, math, os, sys, collections
R0 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R0)
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree
from gemeinsam.fluss import analyse as fluss_analyse


def main():
    B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); Z = json.load(open(sys.argv[3])); K = json.load(open(sys.argv[4]))
    OUT = sys.argv[5]; a = dict(x.split('=', 1) for x in sys.argv[6:] if '=' in x)
    VD, VDR = [float(v) for v in a.get('via', '0.45/0.20').split('/')]; VR = VD / 2
    CLR = float(a.get('clr', '0.2')); H2H = float(a.get('h2h', '0.2')); EDGE = float(a.get('edge', '0.3'))
    PITCH = max(float(a.get('abstand', '0.7')), VDR + H2H + 0.02)
    I03 = float(a.get('strom_03', '1.0')); RES = float(a.get('reserve', '1.3')); IMIN = float(a.get('mindest_a', '0.5'))
    VIP = a.get('vip', '0') == '1'; REL = a.get('relativ', '0') == '1'
    I_VIA = I03 * VDR / 0.3
    board = unary_union([Polygon(o) for o in B['outline'] if len(o) >= 3]).buffer(-(EDGE + VR))
    GND = set(K.get('gnd') or [])

    geo = collections.defaultdict(list)
    for t in R['tracks']:
        for p, q, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            geo[t['layer']].append((t['net'], LineString([p, q]).buffer(w / 2, cap_style='round')))
    for p in B['pads']:
        for L, polys in p['polys'].items():
            for q in polys:
                if len(q) >= 3: geo[L].append((p['net'], Polygon(q).buffer(0)))
    smd = [(p['net'], Polygon(q).buffer(0)) for p in B['pads'] if not p.get('drill') for L, polys in p['polys'].items() for q in polys if len(q) >= 3]
    lagen = sorted(geo)
    baum = {L: (STRtree([g for _, g in geo[L]]), [n for n, _ in geo[L]]) for L in lagen}
    zonen = collections.defaultdict(list)
    fremdzone = collections.defaultdict(list)
    for z in Z:
        if z.get('gnd_signal') or (z.get('prio', 1) == 0 and z['net'] in GND) or len(z['pts']) < 3: continue
        g = Polygon(z['pts'], z.get('holes') or []).buffer(0)
        zonen[(z['net'], z['layer'])].append(g); fremdzone[z['layer']].append((z['net'], g))
    bohr = [(v['x'], v['y']) for v in R['vias']] + [(p['x'], p['y']) for p in B['pads'] if p.get('drill', 0) > 0]

    def frei(x, y, netz):
        pt = Point(x, y)
        if not board.contains(pt): return False
        if any(math.hypot(x - bx, y - by) < VDR + H2H for bx, by in bohr): return False
        if not VIP and any(n == netz and g.distance(pt) < VR for n, g in smd): return False
        kreis = pt.buffer(VR + CLR)
        for L in lagen:
            tr, nets = baum[L]
            for i in tr.query(kreis):
                n = nets[int(i)]; g = geo[L][int(i)][1]
                if n == netz: continue
                if g.distance(pt) < VR + CLR: return False
            for n, g in fremdzone[L]:
                if n != netz and g.distance(pt) < VR + CLR: return False
        return True

    def eigen(x, y, netz, L):
        pt = Point(x, y)
        if any(g.buffer(-VR * 0.5).contains(pt) for g in zonen[(netz, L)]): return True
        tr, nets = baum[L]
        return any(nets[int(i)] == netz and geo[L][int(i)][1].contains(pt) for i in tr.query(pt))

    neu = []; bericht = []
    tracks_n = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        if not t.get('sig'): tracks_n[t['net']].append(t)
    for netz, k in sorted(K['netze'].items()):
        if k.get('art') not in ('strom', 'versorgung', 'sw') or netz in GND: continue
        fl = k.get('fluss') or {}
        I_netz = float(fl.get('gesamt') or 0.0)
        if I_netz <= 0 or (fl.get('einheit') != 'A' and not REL): continue
        tr = tracks_n.get(netz) or []
        vs = [v for v in R['vias'] if v['net'] == netz]
        if not tr or not vs: continue
        info = fluss_analyse([dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr], vs,
                             [p for p in B['pads'] if p['net'] == netz], fl)
        anteil = (info or {}).get('anteil') or {}
        for v in vs:

            seg = []; ant = 0.0; vp = Point(v['x'], v['y'])
            for ti, t in enumerate(tr):
                for si, (p, q) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if LineString([p, q]).distance(vp) < 0.06:
                        far = q if math.dist(q, (v['x'], v['y'])) > math.dist(p, (v['x'], v['y'])) else p
                        seg.append((t['layer'], far, t['w'][si])); ant = max(ant, anteil.get((ti, si), 1.0 if not anteil else 0.0))
            lag = {L for L, _, _ in seg}
            for pp in B['pads']:
                if pp['net'] == netz:
                    for L, polys in pp['polys'].items():
                        if any(len(q) >= 3 and Polygon(q).buffer(0.05).contains(vp) for q in polys): lag.add(L)
            if not seg or len(lag) < 2: continue
            I = ant * I_netz
            if I < IMIN: continue
            n_soll = max(2, math.ceil(RES * I / I_VIA))
            schon = sum(1 for w in vs + neu if w['net'] == netz and math.hypot(w['x'] - v['x'], w['y'] - v['y']) < 2.5 * PITCH)
            fehlt = n_soll - schon
            if fehlt <= 0: continue
            wmin = min(w for _, _, w in seg)

            kand = []
            for L, far, w in seg:
                d = math.dist(far, (v['x'], v['y']))
                for k_ in (1, 2, 3):
                    r = k_ * PITCH
                    if r > d - 0.05: break
                    f = r / d; kand.append((r, L, v['x'] + f * (far[0] - v['x']), v['y'] + f * (far[1] - v['y'])))
            for r in (PITCH, 1.4 * PITCH, 2 * PITCH):
                for j in range(12):
                    ph = 2 * math.pi * j / 12; kand.append((r + 0.01, None, v['x'] + r * math.cos(ph), v['y'] + r * math.sin(ph)))
            gesetzt = 0; grund = collections.Counter(); stege = []
            for r, LA, x, y in sorted(kand):
                if gesetzt >= fehlt: break
                if any(math.hypot(x - w['x'], y - w['y']) < PITCH * 0.95 for w in vs + neu if w['net'] == netz): grund['zu nah an Via'] += 1; continue
                if not frei(x, y, netz): grund['fremdes Kupfer/Bohrung/Rand'] += 1; continue

                st = []; ok = True
                for L in sorted(lag - {LA}):
                    if eigen(x, y, netz, L): continue
                    linie = LineString([(x, y), (v['x'], v['y'])]); band = linie.buffer(wmin / 2 + CLR)
                    tr_, nets_ = baum.get(L, (None, []))
                    stoer = tr_ is not None and any(nets_[int(i)] != netz and geo[L][int(i)][1].intersects(band) for i in tr_.query(band))
                    stoer = stoer or any(n != netz and g.intersects(band) for n, g in fremdzone[L])
                    if stoer: ok = False; break
                    st.append(dict(net=netz, layer=L, pts=[[round(x, 4), round(y, 4)], [v['x'], v['y']]], w=[wmin], sig=False, strom_steg=True))
                if not ok: grund['Steg blockiert'] += 1; continue
                nv = dict(net=netz, x=round(x, 4), y=round(y, 4), strom_via=True); neu.append(nv); bohr.append((x, y)); stege += st; gesetzt += 1
            R['tracks'] += stege
            bericht.append(dict(netz=netz, x=v['x'], y=v['y'], strom_A=round(I, 3), soll=n_soll, vorher=schon, neu=gesetzt,
                                lagen=sorted(lag), stege=len(stege), abgelehnt=dict(grund)))
    R['vias'] = R['vias'] + neu
    json.dump(R, open(OUT, 'w'))
    json.dump(dict(i_via_A=round(I_VIA, 3), reserve=RES, abstand=PITCH, stellen=bericht), open(OUT.replace('.json', '_strom_vias.json'), 'w'), indent=1)
    kurz = [b for b in bericht if b['vorher'] + b['neu'] < b['soll']]
    print('Strom-Vias: %d Stellen, %d Vias neu (I_via %.2f A, Reserve %.1f); zu wenig Platz an %d Stellen' % (
        len(bericht), len(neu), I_VIA, RES, len(kurz)))
    for b in sorted(bericht, key=lambda b: -b['strom_A'])[:12]:
        print('  %-14s %5.2f A  soll %d  vorher %d  neu %d  %s  %s' % (b['netz'][-14:], b['strom_A'], b['soll'], b['vorher'], b['neu'], ','.join(b['lagen']), b['abgelehnt']))


if __name__ == '__main__':
    main()
