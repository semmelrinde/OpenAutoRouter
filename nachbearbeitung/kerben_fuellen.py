""
import json, os, sys
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.paarnetze import paarnetze, norm
from gemeinsam.kupferform_anker import anker_laden, anbinden
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
B = float(opt.get('b', '0.1')); FH = float(opt.get('frei_hart', '0.138')); FZ = float(opt.get('frei', '0.155'))
RAND = float(opt.get('rand', '0.5')); LOCH = float(opt.get('loch', '0.27')); AMIN = float(opt.get('amin', '0.00003'))
UMSCHL = float(opt.get('umschluss', '0.6'))
BREITEN = [B] + sorted({float(x) for x in opt.get('breiten', '0.08,0.06,0.045,0.03').split(',') if x and float(x) < B}, reverse=True)
EPS = 2e-6
AUS = [a for a in opt.get('aus', '').split(',') if a]
PADS = {}; ANKER = {}
if opt.get('pads'):
    ANKER = anker_laden(json.load(open(opt['pads'])))
    for p in json.load(open(opt['pads']))['pads']:
        for L_, qs in p['polys'].items(): PADS.setdefault(L_, []).extend(Polygon(q['aussen'], q['loecher']).buffer(0) for q in qs)
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board_poly = poly(D['outline']); board_in = board_poly.buffer(-RAND - 0.06); board_ok = board_poly.buffer(-RAND - 0.005)
loecher = unary_union([Point(x, y).buffer(r + LOCH, 24) for x, y, r in D['loecher']]) if D['loecher'] else Polygon()
hart, zf = {}, {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): hart.setdefault((net, L), []).append(poly(ps))
for f in D.get('formen', []): hart.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
for z in D['zonen']: zf.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
hart = {k: unary_union(v) for k, v in hart.items()}; zf = {k: unary_union(v) for k, v in zf.items()}
PAARE = paarnetze({n for n, _ in list(hart) + list(zf) if n}, opt.get('klassen'))
out = []; stat = {}; neu = {}; bstat = {}; verworfen = 0
from gemeinsam.abstand_tor import zu_nah, ausgabe_form
for L in sorted({l for _, l in list(hart) + list(zf)}):
    netcu = {}
    for (n, l), g in list(hart.items()) + list(zf.items()):
        if l == L and n: netcu[n] = unary_union([netcu.get(n, Polygon()), g])
    alle = unary_union(list(netcu.values()))
    sperr = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['lagen'] and (s['fuellen'] or s['bahnen'])])
    W = board_in.difference(alle).difference(sperr).difference(loecher)
    def schmal(Wx, b): return Wx.difference(Wx.buffer(-b / 2, quad_segs=8).buffer(b / 2, quad_segs=8))
    def stuecke(g): return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= AMIN]
    offen = [(0, q) for q in stuecke(schmal(W, BREITEN[0]))]
    while offen:
        k, q = offen.pop()
        q = q.buffer(-0.001, quad_segs=4).buffer(0.001, quad_segs=4)
        if q.geom_type != 'Polygon': offen += [(k, p) for p in stuecke(q)]; continue
        if q.area < AMIN: continue
        nah = [n for n, g in netcu.items() if g.distance(q) < 0.003]
        if len(nah) != 1: continue
        n = nah[0]
        if q.boundary.intersection(netcu[n].buffer(0.003)).length < UMSCHL * q.boundary.length: continue
        if (norm(n) in PAARE or any(a in n for a in AUS)) and not any(pg.distance(q) < 0.003 for pg in PADS.get(L, [])): continue
        fh = unary_union([g for (m, l), g in hart.items() if l == L and m and m != n] + [g for (m, l), g in neu.items() if l == L and m != n])
        fz = unary_union([g for (m, l), g in zf.items() if l == L and m and m != n])
        verbot = unary_union([fh.buffer(FH), fz.buffer(FZ)])
        if q.difference(board_ok).area > EPS or (q.intersects(verbot) and q.intersection(verbot).area > EPS):
            if k + 1 < len(BREITEN):
                b2 = BREITEN[k + 1]; Wloc = W.intersection(q.buffer(b2 + 0.01))
                offen += [(k + 1, q2) for q2 in stuecke(schmal(Wloc, b2).intersection(q.buffer(0.001)))]
            continue
        g = q.buffer(0.02, quad_segs=8).intersection(unary_union([netcu[n], q]).buffer(1e-7))
        for gg in getattr(g, 'geoms', [g]):
            if gg.geom_type == 'Polygon' and gg.area >= AMIN and gg.distance(netcu[n]) < 0.002:
                gg = anbinden(gg, netcu[n], ANKER.get((n, L), []))
                pts = [list(c) for c in gg.simplify(0.0005).exterior.coords][:-1]
                if zu_nah(ausgabe_form(pts), fh, FH, fz, FZ): verworfen += 1; continue
                out.append(dict(net=n, layer=L, b=BREITEN[k], pts=pts)); stat[(L, n)] = stat.get((L, n), 0) + 1
                bstat[BREITEN[k]] = bstat.get(BREITEN[k], 0) + 1
                neu[(n, L)] = unary_union([neu.get((n, L), Polygon()), gg])
json.dump(out, open(OUT, 'w'))
print('Kerben gefuellt:', len(out), 'verworfen (Abstand):', verworfen, 'je Breite', dict(sorted(bstat.items(), reverse=True)), {'%s %s' % k: v for k, v in sorted(stat.items(), key=lambda kv: -kv[1])[:10]})
