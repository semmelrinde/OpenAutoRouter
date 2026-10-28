""
import json, sys, math, collections, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shapely.geometry import LineString, Point, Polygon, box
from shapely.strtree import STRtree
H = json.load(open(sys.argv[1])); X0, Y0, X1, Y1 = map(float, sys.argv[2:6]); NETS = [n for n in sys.argv[6].split(';') if n]
OUT = sys.argv[7]; A = dict(a.split('=', 1) for a in sys.argv[8:] if '=' in a)
GESPERRT = {}; FIX = set()
if A.get('sperrauto', '1') == '1' or A.get('sperren'):
    from nachbearbeitung.sperrnetze import sperrnetze, sperrumfang
    auto = A.get('sperrauto', '1') == '1'
    GESPERRT = sperrnetze(pcb=A.get('pcb') if auto else None, pro=A.get('pro') if auto else None, dru=A.get('dru') if auto else None,
                          klassen=A.get('klassen') if auto else None, sperren=A.get('sperren'), netze={t['net'] for t in H['tracks']},
                          pads=H.get('pads') if auto else None, net_ties=H.get('net_ties') if auto else None,
                          tracks=H['tracks'] if auto else None, namen=auto)
    if auto:
        _U = sperrumfang(H, pcb=A.get('pcb'), pro=A.get('pro'), dru=A.get('dru'), klassen=A.get('klassen'), sperren=A.get('sperren'))
        FIX = set(_U['bahnen'])
        for _n in _U.get('ohne_pfad', []): GESPERRT.setdefault(_n, _U['netze'].get(_n, 'Kelvin ohne Weg'))
    if A.get('fix'): FIX |= set(json.load(open(A['fix'])))
    if any(n in GESPERRT for n in NETS):
        print('gesperrt (unveraendert):', ', '.join('%s [%s]' % (n, GESPERRT[n]) for n in NETS if n in GESPERRT))
    NETS = [n for n in NETS if n not in GESPERRT]
LAG = A.get('lagen', 'alle').split(',')
if LAG == ['alle']:
    LAG = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']})
MODUS = A.get('modus', 'voll')
TOTE = A.get('tote', '1') == '1'


TOTE_AUS = set(json.load(open(A['ohne_tote']))) if A.get('ohne_tote') else set()
CL_T, CL_P, CL_V, EDGE, MARG = 0.09, 0.10, 0.10, 0.2, 0.01
LOCH = 0.25
if 'regeln' in A:
    rg = json.load(open(A['regeln'])).get('router', {})
    CL_T = float(rg.get('drc', CL_T))
    CL_P = float(rg.get('drcpad', CL_P))
    CL_V = float(rg.get('drcvia', rg.get('drc', CL_V)))
    EDGE = float(rg.get('edge', EDGE)); LOCH = float(rg.get('loch_clr', LOCH))
if 'drc' in A: CL_T = CL_P = CL_V = float(A['drc'])
if 'drcpad' in A: CL_P = float(A['drcpad'])
if 'drcvia' in A: CL_V = float(A['drcvia'])
if 'rand' in A: EDGE = float(A['rand'])
if 'loch' in A: LOCH = float(A['loch'])
PADMITTE = A.get('padmitte', '1') == '1'
MITTE = A.get('mitte', '1') == '1'
HMAX_NL = {}
if A.get('hmax_lagen') and float(A.get('hmax_ebene', '0.3')) > 0:
    HMAX_NL = {(n, L): float(A.get('hmax_ebene', '0.3')) for n, Ls in json.load(open(A['hmax_lagen'])).items() for L in Ls}
HMAX = float(A.get('hmax', '0'))

BUDGET = json.load(open(A['budget'])) if A.get('budget') else {}
WORK = box(X0, Y0, X1, Y1)
def inw(p): return X0 <= p[0] <= X1 and Y0 <= p[1] <= Y1
def R(p): return (round(p[0], 4), round(p[1], 4))

obst = {L: [] for L in LAG}
for t in H['tracks']:
    if t['layer'] in obst:
        g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
        obst[t['layer']].append((t['net'], g.buffer(t['w'] / 2, 8), CL_T))
for p in H['pads']:
    if p['layer'] in obst: obst[p['layer']].append((p['net'], Polygon(p['poly']), CL_P))
for v in H['vias']:
    c = Point(v['x'], v['y']).buffer(v['d'] / 2, 16)
    for L in LAG: obst[L].append((v['net'], c, CL_V))

for h in H.get('holes', []):
    c = Point(h['x'], h['y']).buffer(h['d'] / 2, 16)
    for L in LAG: obst[L].append((h['net'] or '#LOCH', c, LOCH))
for k in H.get('keepouts', []):
    if k.get('tracks'):
        for L in k['layers']:
            if L in obst: obst[L].append(('#SPERR', Polygon(k['poly']), 0.0))
for e in H.get('edges', []):
    g = Polygon(e) if len(e) > 2 else LineString(e)
    for L in LAG: obst[L].append(('#KANTE', g.boundary if g.geom_type == 'Polygon' else g, EDGE))

ZW = None
if A.get('pcb') and A.get('zonen', '1') == '1':
    from gemeinsam.kupfer_netz import ZonenWaechter, zonen_fuellung
    FUELL = zonen_fuellung(open(A['pcb'], encoding='utf-8').read())
    for f in FUELL:
        if f['net'] in GESPERRT and f['layer'] in obst: obst[f['layer']].append((f['net'], f['poly'], max(0.0, f['clr'] - MARG - 1e-4)))
    ZW = ZonenWaechter(H, FUELL, drc=CL_T)
trees = {L: STRtree([o[1] for o in obst[L]]) for L in LAG}
DMAX = max([o[2] for L in LAG for o in obst[L]] + [0.25])
NEU = collections.defaultdict(list); NEU_RASTER = collections.defaultdict(list)
def _zellen(b, r=1.0):
    return [(i, j) for i in range(math.floor(b[0] / r), math.floor(b[2] / r) + 1) for j in range(math.floor(b[1] / r), math.floor(b[3] / r) + 1)]
def neu_hindernis(net, L, a, b, w):
    ""
    g = LineString([a, b]) if tuple(a) != tuple(b) else Point(a); k = len(NEU[L]); NEU[L].append((net, g, w))
    for z in _zellen(g.buffer(w / 2).bounds): NEU_RASTER[(L, z)].append(k)
def frei(seg, w, L, net):
    ""
    for i in trees[L].query(seg.buffer(w / 2 + DMAX + MARG)):
        n, g, d = obst[L][int(i)]
        if n == net: continue
        if seg.distance(g) < w / 2 + d + MARG: return False
    for k in sorted({k for z in _zellen(seg.buffer(w / 2 + CL_T + MARG).bounds) for k in NEU_RASTER.get((L, z), ())}):
        n, g, wn = NEU[L][k]
        if n != net and seg.distance(g) < (w + wn) / 2 + CL_T + MARG: return False
    return True
def okt(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs(dx) < 1e-6 or abs(dy) < 1e-6 or abs(abs(dx) - abs(dy)) < 1e-6
def kandidaten(a, b):
    if okt(a, b): return [[a, b]]
    dx, dy = b[0] - a[0], b[1] - a[1]; m = min(abs(dx), abs(dy)); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    d1 = (a[0] + sx * m, a[1] + sy * m)
    d2 = (b[0] - sx * m, b[1] - sy * m)
    return [[a, R(d1), b], [a, R(d2), b]]
def plen(pts): return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
def winkel_ok(p_prev, p, p_next):
    ""
    if p_prev is None or p_next is None: return True
    v1 = (p_prev[0] - p[0], p_prev[1] - p[1]); v2 = (p_next[0] - p[0], p_next[1] - p[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-9 or l2 < 1e-9: return True
    return (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2) <= 1e-6
def rechtwinklig(p_prev, p, p_next):
    ""
    if p_prev is None or p_next is None: return False
    v1 = (p_prev[0] - p[0], p_prev[1] - p[1]); v2 = (p_next[0] - p[0], p_next[1] - p[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-9 or l2 < 1e-9: return False
    return abs((v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)) <= math.sin(math.radians(0.5))
remove, add = [], []; stat = collections.Counter(); aend = {}
gebucht = collections.defaultdict(float)
for net in NETS:
    for L in LAG:
        segs = [t for t in H['tracks'] if t['net'] == net and t['layer'] == L]


        arc_ends = {R(t[e]) for t in segs if t.get('arc') for e in ('a', 'b')}
        inner = [t for t in segs if inw(t['a']) and inw(t['b']) and not t.get('arc') and t.get('uuid') not in FIX
                 and not any(R(t[e]) in arc_ends for e in ('a', 'b'))]
        if not inner: continue
        w = max(t["w"] for t in inner); TOL = min(t["w"] for t in inner) / 2 - 0.005
        own_v = [(Point(v['x'], v['y']), v['d'] / 2) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])]
        own_p = [Polygon(p['poly']).buffer(1e-4) for p in H['pads'] if p['net'] == net and p['layer'] == L]
        own_pm = [(p['x'], p['y']) if 'x' in p else None for p in H['pads'] if p['net'] == net and p['layer'] == L]
        inner_ids = {id(t) for t in inner}
        outer = [LineString([t['a'], t['b']]).buffer(t['w'] / 2 + 1e-4) for t in segs if id(t) not in inner_ids]


        pw = collections.defaultdict(float)
        for t in segs:
            for e in ('a', 'b'): pw[R(t[e])] = max(pw[R(t[e])], t['w'])
        ends = [(tuple(t['a']), t['w']) for t in segs] + [(tuple(t['b']), t['w']) for t in segs] + [((v[0].x, v[0].y), None) for v in own_v]
        pieces = []; breite = {}; stoss = []
        piece_w = []; pw_seen = {}; dup_breite = set(); kante_w = {}; dup_w = collections.defaultdict(set)
        for t in inner:
            a, b = tuple(t['a']), tuple(t['b']); ln = LineString([a, b]); cuts = []
            for q, wq in ends:
                dq = ln.distance(Point(q))
                if dq <= (TOL if wq is None else max(TOL, (wq + t['w']) / 2 - 1e-6)):
                    s_ = ln.project(Point(q))
                    if 0.02 < s_ < ln.length - 0.02:
                        cuts.append(s_)
                        if wq is not None and dq > 1e-6: stoss.append((R(q), R(ln.interpolate(round(s_, 5)).coords[0])))


            for pm in own_pm:
                if MITTE and pm is not None and ln.distance(Point(pm)) <= 0.005:
                    s_ = ln.project(Point(pm))
                    if 0.02 < s_ < ln.length - 0.02: cuts.append(s_)
            ds = [0.0] + sorted(set(round(c, 5) for c in cuts)) + [ln.length]
            pts = [R(ln.interpolate(d).coords[0]) for d in ds]; pts[0], pts[-1] = R(a), R(b)
            for p, q in zip(pts, pts[1:]):
                if p != q:
                    kq = tuple(sorted([p, q]))
                    if kq in pw_seen and abs(pw_seen[kq] - t['w']) > 1e-9: dup_breite.add(kq)
                    dup_w[kq].add(round(t['w'], 6))
                    pw_seen.setdefault(kq, t['w'])
                    pieces.append((p, q)); breite[(p, q)] = t['w']; piece_w.append(t['w'])


        allp = sorted({p for pq in pieces for p in pq})
        cl = []; owner = {}
        for p in allp:
            for k, ms in enumerate(cl):
                if all(math.dist(p, m) <= TOL for m in ms): ms.append(p); owner[p] = k; break
            else: cl.append([p]); owner[p] = len(cl) - 1
        upar = list(range(len(cl)))
        def uf(k):
            while upar[k] != k: upar[k] = upar[upar[k]]; k = upar[k]
            return k
        for q, x in stoss:
            if q in owner and x in owner: upar[uf(owner[q])] = uf(owner[x])
        if any(uf(k) != k for k in range(len(cl))):
            neu_cl = collections.defaultdict(list)
            for k, ms in enumerate(cl): neu_cl[uf(k)] += ms
            cl = [sorted(ms) for _, ms in sorted(neu_cl.items())]
            owner = {p: k for k, ms in enumerate(cl) for p in ms}
        def f(p): return owner[p]
        members = {k: ms for k, ms in enumerate(cl)}
        def an_via(c): return any(v.distance(Point(p)) <= r for v, r in own_v for p in members[c])
        def an_pad(c): return any(g.contains(Point(p)) for g in own_p for p in members[c])
        def an_aussen(c): return any(g.contains(Point(p)) for g in outer for p in members[c])
        rep = {}
        for c, ms in members.items():
            vs = [(v.x, v.y) for v, r in own_v if any(v.distance(Point(p)) <= r for p in ms)]
            rep[c] = R(vs[0]) if vs else collections.Counter(ms).most_common(1)[0][0]
        kanten = []
        def in_via(p, q): return any(v.distance(Point(p)) <= r - 0.005 and v.distance(Point(q)) <= r - 0.005 for v, r in own_v)
        def in_pad(p, q): return any(g.buffer(-0.005).contains(Point(p)) and g.buffer(-0.005).contains(Point(q)) for g in own_p)
        mitten = [(v.x, v.y) for v, r in own_v] + [(p['x'], p['y']) if 'x' in p else Polygon(p['poly']).centroid.coords[0]
                                                    for p in H['pads'] if p['net'] == net and p['layer'] == L]
        def ueberfluessig(p, q):
            if not (in_via(p, q) or in_pad(p, q)): return False


            if MITTE and any((LineString([p, q]) if p != q else Point(p)).distance(Point(m)) <= 0.005 and min(math.dist(p, m), math.dist(q, m)) >= 1e-3
                             for m in mitten): return False
            return not (MITTE and any(min(math.dist(p, m), math.dist(q, m)) < 1e-3 for m in mitten))
        def erreicht_m(stuecke):
            gs = [LineString([p, q]) if p != q else Point(p) for p, q in stuecke]
            return {m for m in mitten if any(g.distance(Point(m)) <= 0.005 for g in gs)}
        behalten = []
        def zur_mitte_(p, q):
            g = LineString([p, q]) if p != q else Point(p)
            return MITTE and any(g.distance(Point(m)) <= 0.005 for m in mitten)
        verloren = collections.defaultdict(list)
        for idx_, (p, q) in enumerate(pieces):
            c1, c2 = f(p), f(q)
            if MODUS == 'voll' and c1 == c2 and zur_mitte_(p, q): behalten.append((p, q)); stat['Mikrostueck zur Mitte bleibt'] += 1; continue
            if MODUS == 'voll' and c1 == c2: stat['Mikrostuecke'] += 1; verloren[c1].append((p, q)); continue
            if MODUS == 'voll' and ueberfluessig(p, q):
                stat['im Via/Pad'] += 1; verloren[c1].append((p, q)); verloren[c2].append((p, q)); continue
            if tuple(sorted([p, q])) in dup_breite: kante_w[len(kanten)] = piece_w[idx_]
            kanten.append((c1, c2, (p, q)))
        adj = collections.defaultdict(list)
        for k, (c1, c2, _) in enumerate(kanten): adj[c1].append(k); adj[c2].append(k)
        weg = set()
        def schnitt(p1, p2, q1, q2):
            d1 = (p2[0] - p1[0], p2[1] - p1[1]); d2 = (q2[0] - q1[0], q2[1] - q1[1]); den = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(den) < 1e-12: return None
            t_ = ((q1[0] - p1[0]) * d2[1] - (q1[1] - p1[1]) * d2[0]) / den
            return (p1[0] + t_ * d1[0], p1[1] + t_ * d1[1])






        fest_knoten = {owner[x] for kq in dup_breite for x in kq if x in owner}; geschlossen = [0]
        def einheit(a, b):
            d_ = math.dist(a, b)
            return None if d_ < 1e-9 else ((b[0] - a[0]) / d_, (b[1] - a[1]) / d_)
        def enden_an(c):
            out = []
            for k in adj[c]:
                if k in weg: continue
                c1, c2, (p, q) = kanten[k]
                if c1 == c: out.append((k, 'p', p, q))
                if c2 == c: out.append((k, 'q', q, p))
            return out
        def setze(k, seite, e_neu, o_neu=None):
            c1, c2, (p, q) = kanten[k]; ww = breite.get((p, q), w)
            np_ = (e_neu, o_neu if o_neu is not None else q) if seite == 'p' else (o_neu if o_neu is not None else p, e_neu)
            kanten[k] = (c1, c2, np_); breite[np_] = ww
        def bw_(k): return breite.get(kanten[k][2], w)
        def im_ziel(p): return any(v.distance(Point(p)) <= r for v, r in own_v) or any(g.contains(Point(p)) for g in own_p)
        def vereinen(c):
            if c in fest_knoten: return False
            es = enden_an(c)
            if len(es) != 2 or an_aussen(c): return False
            (k1, s1, e1, o1), (k2, s2, e2, o2) = es
            if k1 == k2: return False
            wa, wb = bw_(k1), bw_(k2)
            d1, d2 = einheit(o1, e1), einheit(o2, e2)
            if d1 is None or d2 is None: return False
            vor = lambda X, o, d: (X[0] - o[0]) * d[0] + (X[1] - o[1]) * d[1] > 1e-3
            X = schnitt(o1, e1, o2, e2)
            if X is not None:
                X = R(X)
                if math.dist(X, e1) > max(wa, wb) + TOL or math.dist(X, e2) > max(wa, wb) + TOL: return False
                if not (vor(X, o1, d1) and vor(X, o2, d2) and winkel_ok(o1, X, o2)): return False
                if rechtwinklig(o1, X, o2) and not im_ziel(X): return False
                if not (bleibt_im_ziel(e1, X) and bleibt_im_ziel(e2, X)): return False
                if not (frei(LineString([o1, X]), wa, L, net) and frei(LineString([o2, X]), wb, L, net)): return False
                setze(k1, s1, X); setze(k2, s2, X); stat['Knotenschluss (Schnittpunkt)'] += 1; return True
            quer = (e2[0] - e1[0]) * d1[1] - (e2[1] - e1[1]) * d1[0]
            if abs(quer) <= 1e-4:
                X = R(((e1[0] + e2[0]) / 2, (e1[1] + e2[1]) / 2))
                if not (vor(X, o1, d1) and vor(X, o2, d2)): return False
                if not (bleibt_im_ziel(e1, X) and bleibt_im_ziel(e2, X)): return False
                if not (frei(LineString([o1, X]), wa, L, net) and frei(LineString([o2, X]), wb, L, net)): return False
                setze(k1, s1, X); setze(k2, s2, X); stat['Knotenschluss (Linie)'] += 1; return True

            for (ka, sa, ea, oa, da, wa_), (kb, sb, eb, ob, db, wb_) in (((k1, s1, e1, o1, d1, wa), (k2, s2, e2, o2, d2, wb)),
                                                                     ((k2, s2, e2, o2, d2, wb), (k1, s1, e1, o1, d1, wa))):
                ca = kanten[ka][1] if sa == 'p' else kanten[ka][0]
                if ca in fest_knoten or an_via(ca) or an_pad(ca) or an_aussen(ca) or not inw(rep[ca]): continue
                nb3 = [x for x in enden_an(ca) if x[0] != ka]
                if len(nb3) != 1 or len(enden_an(ca)) != 2 or nb3[0][2] != oa: continue
                k3, s3, _, o3 = nb3[0]; d3 = einheit(o3, oa)
                if d3 is None or abs(d3[0] * da[1] - d3[1] * da[0]) < 1e-9: continue
                oa2 = schnitt(eb, (eb[0] + db[0], eb[1] + db[1]), o3, oa)
                if oa2 is None: continue
                oa2 = R(oa2); X = eb
                if math.dist(oa2, oa) > 2 * max(wa_, wb_) or not (vor(X, oa2, da) and vor(oa2, o3, d3)): continue
                if not (winkel_ok(o3, oa2, X) and winkel_ok(oa2, X, ob)): continue
                if not (bleibt_im_ziel(ea, X) and bleibt_im_ziel(oa, oa2)): continue
                hm_ = HMAX_NL.get((net, L), 0.0)
                if hm_ and (math.dist(oa2, oa) > hm_ or math.dist(X, ea) > hm_): continue
                w3 = bw_(k3)
                if not (frei(LineString([oa2, X]), wa_, L, net) and frei(LineString([o3, oa2]), w3, L, net)): continue
                setze(ka, sa, X, oa2); setze(k3, s3, oa2); stat['Knotenschluss (auf eine Linie gelegt)'] += 1; return True
            return False
        def bleibt_im_ziel(e, X):
            return ziele_von([e]) <= ziele_von([X])
        def ziele_von(pts):
            return {('v', i) for i, (v, r) in enumerate(own_v) for p in pts if v.distance(Point(p)) <= r} | \
                   {('p', i) for i, g in enumerate(own_p) for p in pts if g.contains(Point(p))}
        for c in sorted(verloren):
            es = enden_an(c); pkt = sorted({e for _, _, e, _ in es})


            verlassen = bool(pkt) and bool(ziele_von([x for pq in verloren[c] for x in pq]) - ziele_von(pkt))
            if not verlassen:
                if len(pkt) <= 1: continue
                if vereinen(c): geschlossen[0] += 1; continue

            for pq in verloren[c]:
                if f(pq[0]) == f(pq[1]):
                    if pq not in behalten: behalten.append(pq)
                elif not any(kk[2] == pq for kk in kanten):
                    kanten.append((f(pq[0]), f(pq[1]), pq)); k_ = len(kanten) - 1; adj[f(pq[0])].append(k_); adj[f(pq[1])].append(k_)
                    fest_knoten.add(f(pq[0])); fest_knoten.add(f(pq[1]))
            fest_knoten.add(c); stat['Stueck bleibt (Enden lagen sonst nicht aufeinander)'] += len(verloren[c])
        weggelassen = {pq for c in verloren for pq in verloren[c]} - set(behalten) - {kk[2] for kk in kanten}
        geaendert = MODUS == 'voll' and (bool(weggelassen) or geschlossen[0] > 0)
        for c in list(members):
            ks_ = adj[c]
            if len(ks_) != 2 or len(set(ks_)) != 2 or an_via(c) or an_pad(c) or an_aussen(c) or c in fest_knoten: continue
            if any(kanten[k][0] == kanten[k][1] for k in ks_): continue
            ends = []
            for k in ks_:
                c1, c2, (p, q) = kanten[k]
                ends.append((k, 'p', p, q) if c1 == c else (k, 'q', q, p))
            (k1, s1, e1, o1), (k2, s2, e2, o2) = ends
            if e1 == e2: continue
            X = schnitt(o1, e1, o2, e2)
            if X is None:
                d = (e1[0] - o1[0], e1[1] - o1[1]); ld = math.hypot(*d)
                if ld < 1e-9 or abs((e2[0] - o1[0]) * d[1] - (e2[1] - o1[1]) * d[0]) / ld > 1e-4: continue
                X = ((e1[0] + e2[0]) / 2, (e1[1] + e2[1]) / 2)
            X = R(X); wa, wb = breite.get(kanten[k1][2], w), breite.get(kanten[k2][2], w)
            if math.dist(X, e1) > max(wa, wb) or math.dist(X, e2) > max(wa, wb): continue
            if math.dist(X, o1) < 1e-3 or math.dist(X, o2) < 1e-3: continue
            neu1 = (X, o1) if s1 == 'p' else (o1, X); neu2 = (X, o2) if s2 == 'p' else (o2, X)
            if not (frei(LineString(neu1), wa, L, net) and frei(LineString(neu2), wb, L, net)): continue
            for k, nk, ww in ((k1, neu1, wa), (k2, neu2, wb)):
                c1, c2, _ = kanten[k]; kanten[k] = (c1, c2, nk); breite[nk] = ww
            stat['Stoesse ausgerichtet'] += 1; geaendert = True
        def grad(c): return sum(1 for k in adj[c] if k not in weg)
        def fest(c): return grad(c) != 2 or c in fest_knoten or an_via(c) or an_pad(c) or an_aussen(c) or not inw(rep[c])
        def lebt(c):
            ""
            if an_via(c) or an_pad(c) or an_aussen(c) or not inw(rep[c]): return True
            for m in members[c]:
                hw = (pw.get(m) or w) / 2
                if any(v.distance(Point(m)) <= r + hw for v, r in own_v): return True
                if any(g.distance(Point(m)) <= hw for g in own_p) or any(g.distance(Point(m)) <= hw for g in outer): return True
            for k, (c1, c2, (p, q)) in enumerate(kanten):
                if k in weg or c in (c1, c2): continue
                wk = breite.get((p, q), breite.get((q, p), w))
                if any(LineString([p, q]).distance(Point(m)) <= ((pw.get(m) or w) + wk) / 2 for m in members[c]): return True
            return False

        while MODUS == 'voll' and TOTE and net not in TOTE_AUS:
            tot = [c for c in members if grad(c) == 1 and not lebt(c)]
            if not tot: break
            for c in tot:
                ks = [k for k in adj[c] if k not in weg]
                if len(ks) == 1: weg.add(ks[0]); stat['tote Stuecke'] += 1; geaendert = True

        gesehen = set(); ketten = []
        for c in members:
            if not fest(c) or grad(c) == 0: continue
            for k in adj[c]:
                if k in weg or k in gesehen: continue
                kn = [c]; ks = []; cur = c; kk = k
                while True:
                    gesehen.add(kk); ks.append(kk); c1, c2, _ = kanten[kk]; nxt = c2 if c1 == cur else c1; kn.append(nxt); cur = nxt
                    if fest(cur): break
                    kk = [x for x in adj[cur] if x not in weg and x != kk][0]
                ketten.append((kn, ks))
        ersetzt = set(); neu = []
        def exakt_nb(p, ohne):
            ""
            out = []
            for k, (c1, c2, (a, b)) in enumerate(kanten):
                if k in weg or k in ohne or k in ersetzt: continue
                if a == p: out.append(b)
                elif b == p: out.append(a)
            out += [q for p_, q, _ in neu if p_ == p] + [p_ for p_, q, _ in neu if q == p]
            out += [b for a, b in behalten if a == p] + [a for a, b in behalten if b == p]
            return sorted(set(out))
        def nachbarn(c, k_eigen):
            ""
            out = []
            for k in adj[c]:
                if k in weg or k == k_eigen or k in ersetzt: continue
                c1, c2, _ = kanten[k]; out.append(rep[c2 if c1 == c else c1])
            return out
        def padmitte(c, p):


            if not PADMITTE or grad(c) != 1 or an_via(c) or an_aussen(c): return p

            mitte = {R(pm or g.centroid.coords[0]) for g, pm in zip(own_p, own_pm) if g.contains(Point(p))
                     and g.contains(Point(pm or g.centroid.coords[0])) and inw(pm or g.centroid.coords[0])}
            return next(iter(mitte)) if len(mitte) == 1 else p
        for kn, ks in (ketten if MODUS == 'voll' else []):

            seq = []
            for idx, k in enumerate(ks):
                c1, c2, (p, q) = kanten[k]
                seq.append((p, q) if c1 == kn[idx] else (q, p))
            n = len(seq); out = []; i = 0; anders = False
            while i < n:
                gef = None
                for j in range(n, i + 1, -1):
                    a, b = seq[i][0], seq[j - 1][1]
                    alt = sum(math.dist(p, q) for p, q in seq[i:j])
                    if math.dist(a, b) <= TOL:


                        if any(lebt(kn[x]) for x in range(i + 1, j)): continue
                        if net in BUDGET and alt > BUDGET[net] + 1e-9: continue
                        if MITTE and erreicht_m(seq[i:j]) - erreicht_m([(a, b)]): continue
                        gef = (j, [a, b], w); break
                    if i == 0: a = padmitte(kn[0], a)
                    if j == n: b = padmitte(kn[-1], b)
                    def bwe(pq): return breite.get(pq, breite.get((pq[1], pq[0]), w))
                    bs = sorted({bwe(pq) for pq in seq[i:j]}, reverse=True)
                    for c in kandidaten(a, b):
                        nseg = sum(1 for p, q in zip(c, c[1:]) if p != q)
                        if not (plen(c) < alt - 1e-3 or (plen(c) <= alt + 1e-4 and nseg < j - i)): continue
                        hm_ = min([h_ for h_ in (HMAX, HMAX_NL.get((net, L), 0.0)) if h_ > 0] or [0.0])
                        if hm_ and len(set(map(tuple, c))) > 1 and LineString(c).hausdorff_distance(LineString([seq[i][0]] + [q for _, q in seq[i:j]])) > hm_: continue

                        segs_c = [(p, q) for p, q in zip(c, c[1:]) if p != q]; wl = []
                        for si, (p, q) in enumerate(segs_c):
                            want = bwe(seq[i]) if si == 0 else (bwe(seq[j - 1]) if si == len(segs_c) - 1 else min(bs))
                            ok = next((bw for bw in [want] + [x for x in bs if x < want] if frei(LineString([p, q]), bw, L, net)), None)
                            if ok is None: break
                            wl.append(ok)
                        if len(wl) != len(segs_c): continue
                        wk = wl
                        prevs = [out[-1][0]] if out else nachbarn(kn[0], ks[0])
                        nxts = [seq[j][1]] if j < n else nachbarn(kn[-1], ks[-1])
                        if len(c) > 1 and math.dist(c[0], c[-1]) > TOL and (not all(winkel_ok(pv, c[0], c[1]) for pv in prevs) or not all(winkel_ok(c[-2], c[-1], nx) for nx in nxts)): continue

                        if len(c) > 1 and math.dist(c[0], c[-1]) > TOL:

                            pv1 = [out[-1][0]] if out else exakt_nb(c[0], set(ks[i:j]))
                            nx1 = [seq[j][1]] if j < n else exakt_nb(c[-1], set(ks[i:j]))
                            if ((len(pv1) == 1 and not im_ziel(c[0]) and rechtwinklig(pv1[0], c[0], c[1]) and not rechtwinklig(pv1[0], seq[i][0], seq[i][1]))
                                    or (len(nx1) == 1 and not im_ziel(c[-1]) and rechtwinklig(c[-2], c[-1], nx1[0]) and not rechtwinklig(seq[j - 1][0], seq[j - 1][1], nx1[0]))):
                                stat['Kandidat verworfen (neue 90-Grad-Ecke)'] += 1; continue
                        if net in BUDGET and abs(alt - plen(c)) > BUDGET[net] + 1e-9:
                            stat['Kandidat verworfen (Laengentor)'] += 1; continue


                        if MITTE and erreicht_m(seq[i:j]) - erreicht_m(list(zip(c, c[1:]))):
                            stat['Kandidat verworfen (Mitte ginge verloren)'] += 1; continue
                        gef = (j, c, wk); break
                    if gef: break
                if gef:
                    j, c, wk = gef; stat['abgekuerzt'] += 1
                    gespart_ = sum(math.dist(p, q) for p, q in seq[i:j]) - plen(c)
                    stat['mm gespart (vorgeschlagen)'] += round(gespart_, 4)
                    if net in BUDGET: BUDGET[net] -= abs(gespart_); gebucht[(net, L)] += abs(gespart_)
                    if c[0] != seq[i][0] or c[-1] != seq[j - 1][1]: stat['bis Padmitte'] += 1
                    if math.dist(c[0], c[-1]) > TOL:
                        segs_c = [(p, q) for p, q in zip(c, c[1:]) if p != q]
                        for si, (p, q) in enumerate(segs_c):
                            out.append((p, q)); breite[(p, q)] = wk[si] if isinstance(wk, list) else wk
                    i = j; anders = True
                else: out.append(seq[i]); i += 1
            if not anders: continue

            def bw(pq): return breite.get(pq, breite.get((pq[1], pq[0]), w))
            z = []
            for p, q in out:
                if z and z[-1][1] == p and abs(bw(z[-1]) - bw((p, q))) < 1e-6:
                    a, b = z[-1]
                    if abs((b[0] - a[0]) * (q[1] - b[1]) - (b[1] - a[1]) * (q[0] - b[0])) < 1e-6 and (b[0] - a[0]) * (q[0] - b[0]) + (b[1] - a[1]) * (q[1] - b[1]) > 0:
                        ww = bw(z[-1]); z[-1] = (a, q); breite[(a, q)] = ww; continue
                z.append((p, q))
            ersetzt.update(ks); neu += [(p, q, bw((p, q))) for p, q in z]; geaendert = True
        if not geaendert: continue
        rm_, add_ = [t['uuid'] for t in inner], []
        for k, (c1, c2, (p, q)) in enumerate(kanten):
            if k in weg or k in ersetzt: continue
            add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=kante_w.get(k, breite.get((p, q), w))))
        for p, q, ww in neu:
            if p != q: add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=ww))
        for p, q in behalten:
            kq = tuple(sorted([p, q]))
            for ww in (sorted(dup_w[kq]) if kq in dup_breite else [breite.get((p, q), w)]):
                add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=ww))


        if MITTE:
            for s_ in add_:
                for e, o in (('a', 'b'), ('b', 'a')):
                    x, y = tuple(s_[e]), tuple(s_[o])
                    for m in mitten:
                        d_ = math.dist(x, m)
                        if not (1e-6 < d_ <= TOL): continue
                        cr = (x[0] - y[0]) * (m[1] - y[1]) - (x[1] - y[1]) * (m[0] - y[0])

                        if abs(cr) <= 1e-6 * max(math.dist(x, y), 1e-9) and (x[0] - y[0]) * (m[0] - y[0]) + (x[1] - y[1]) * (m[1] - y[1]) > math.dist(x, y) ** 2 + 1e-12:
                            s_[e] = [m[0], m[1]]; stat['bis Mitte verlaengert'] += 1; break
        add_ = [s_ for s_ in add_ if R(s_['a']) != R(s_['b'])]
        if ZW is not None:
            kaputt = ZW.pruefen(net, add_)
            if kaputt:
                stat['verworfen (zerschneidet Zone)'] += 1; print('verworfen, zerschneidet die Zone von', ', '.join(kaputt), ':', net, L)
                if net in BUDGET: BUDGET[net] += gebucht.pop((net, L), 0.0)
                continue
            ZW.commit(net, add_, rm_)
        for s_ in add_: neu_hindernis(net, L, s_['a'], s_['b'], s_['w'])
        aend[(net, L)] = (rm_, add_)

if A.get('verbindung', '1') == '1' and aend:
    from gemeinsam.kupfer_netz import Netzkupfer, anwenden
    NK = Netzkupfer(H)
    for net in sorted({n for n, _ in aend}):
        lag_ = sorted(L for n, L in aend if n == net); alt = NK.tracks.get(net, [])


        fl = [(f['layer'], f['poly']) for f in FUELL if f['net'] == net] if ZW is not None else None
        def mit(Ls):
            rm_ = [u for L in Ls for u in aend[(net, L)][0]]; ad_ = [x for L in Ls for x in aend[(net, L)][1]]
            return anwenden(alt, rm_, ad_)
        if not NK.trennt(net, alt, mit(lag_), fl, fl): continue
        gut = []
        for L in lag_:
            if not NK.trennt(net, alt, mit(gut + [L]), fl, fl): gut.append(L)
        for L in lag_:
            if L not in gut:
                del aend[(net, L)]; stat['verworfen (trennt Netz)'] += 1
                if net in BUDGET: BUDGET[net] += gebucht.pop((net, L), 0.0)
                print('verworfen, trennt das Netz:', net, L)
laenge_alt = {t['uuid']: math.dist(t['a'], t['b']) for t in H['tracks']}
for (net, L) in sorted(aend):
    stat['Netz-Lagen geaendert'] += 1
    remove += aend[(net, L)][0]; add += aend[(net, L)][1]

    stat['mm gespart'] += round(sum(laenge_alt[u] for u in aend[(net, L)][0]) - sum(math.dist(a_['a'], a_['b']) for a_ in aend[(net, L)][1]), 4)
json.dump(dict(remove=remove, add=add), open(OUT, 'w'))
print(dict(stat), 'entfernt', len(remove), 'neu', len(add))
