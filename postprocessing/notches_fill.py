import json, os, sys
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.pair_nets import pair_nets, norm
from common.copper_shape_anchor import anchor_load, connect
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
B = float(opt.get('b', '0.1')); FH = float(opt.get('free_hard', '0.138')); FZ = float(opt.get('vacant', '0.155'))
BORDER = float(opt.get('border', '0.5')); HOLE = float(opt.get('hole_size', '0.27')); AMIN = float(opt.get('amin', '0.00003'))
ENCLOSE = float(opt.get('enclosure', '0.6'))
WIDTHS = [B] + sorted({float(x) for x in opt.get('width_list', '0.08,0.06,0.045,0.03').split(',') if x and float(x) < B}, reverse=True)
EPS = 2e-6
OFF = [a for a in opt.get('off', '').split(',') if a]
PADS = {}; ANCHOR = {}
if opt.get('pads'):
    ANCHOR = anchor_load(json.load(open(opt['pads'])))
    for p in json.load(open(opt['pads']))['pads']:
        for L_, qs in p['polys'].items(): PADS.setdefault(L_, []).extend(Polygon(q['outer'], q['hole_list']).buffer(0) for q in qs)
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board_poly = poly(D['outline']); board_in = board_poly.buffer(-BORDER - 0.06); board_ok = board_poly.buffer(-BORDER - 0.005)
hole_list = unary_union([Point(x, y).buffer(r + HOLE, 24) for x, y, r in D['hole_list']]) if D['hole_list'] else Polygon()
hard, zf = {}, {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): hard.setdefault((net, L), []).append(poly(ps))
for f in D.get('shapes', []): hard.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
for z in D['zone_list']: zf.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
hard = {k: unary_union(v) for k, v in hard.items()}; zf = {k: unary_union(v) for k, v in zf.items()}
PAIRS = pair_nets({n for n, _ in list(hard) + list(zf) if n}, opt.get('netclasses'))
out = []; stat = {}; new = {}; bin_stats = {}; rejected = 0
from common.spacing_gate import too_close, export_shape
for L in sorted({l for _, l in list(hard) + list(zf)}):
    netcu = {}
    for (n, l), g in list(hard.items()) + list(zf.items()):
        if l == L and n: netcu[n] = unary_union([netcu.get(n, Polygon()), g])
    all_items = unary_union(list(netcu.values()))
    blocked = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['cu_layers'] and (s['fill'] or s['track_list'])])
    W = board_in.difference(all_items).difference(blocked).difference(hole_list)
    def narrow(Wx, b): return Wx.difference(Wx.buffer(-b / 2, quad_segs=8).buffer(b / 2, quad_segs=8))
    def pieces(g): return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= AMIN]
    open_items = [(0, q) for q in pieces(narrow(W, WIDTHS[0]))]
    while open_items:
        k, q = open_items.pop()
        q = q.buffer(-0.001, quad_segs=4).buffer(0.001, quad_segs=4)
        if q.geom_type != 'Polygon': open_items += [(k, p) for p in pieces(q)]; continue
        if q.area < AMIN: continue
        close = [n for n, g in netcu.items() if g.distance(q) < 0.003]
        if len(close) != 1: continue
        n = close[0]
        if q.boundary.intersection(netcu[n].buffer(0.003)).length < ENCLOSE * q.boundary.length: continue
        if (norm(n) in PAIRS or any(a in n for a in OFF)) and not any(pg.distance(q) < 0.003 for pg in PADS.get(L, [])): continue
        fh = unary_union([g for (m, l), g in hard.items() if l == L and m and m != n] + [g for (m, l), g in new.items() if l == L and m != n])
        fz = unary_union([g for (m, l), g in zf.items() if l == L and m and m != n])
        ban = unary_union([fh.buffer(FH), fz.buffer(FZ)])
        if q.difference(board_ok).area > EPS or (q.intersects(ban) and q.intersection(ban).area > EPS):
            if k + 1 < len(WIDTHS):
                b2 = WIDTHS[k + 1]; Wloc = W.intersection(q.buffer(b2 + 0.01))
                open_items += [(k + 1, q2) for q2 in pieces(narrow(Wloc, b2).intersection(q.buffer(0.001)))]
            continue
        g = q.buffer(0.02, quad_segs=8).intersection(unary_union([netcu[n], q]).buffer(1e-7))
        for gg in getattr(g, 'geoms', [g]):
            if gg.geom_type == 'Polygon' and gg.area >= AMIN and gg.distance(netcu[n]) < 0.002:
                gg = connect(gg, netcu[n], ANCHOR.get((n, L), []))
                pts = [list(c) for c in gg.simplify(0.0005).exterior.coords][:-1]
                if too_close(export_shape(pts), fh, FH, fz, FZ): rejected += 1; continue
                out.append(dict(net=n, layer=L, b=WIDTHS[k], pts=pts)); stat[(L, n)] = stat.get((L, n), 0) + 1
                bin_stats[WIDTHS[k]] = bin_stats.get(WIDTHS[k], 0) + 1
                new[(n, L)] = unary_union([new.get((n, L), Polygon()), gg])
json.dump(out, open(OUT, 'w'))
print('Notches filled:', len(out), 'rejected (spacing):', rejected, 'per width', dict(sorted(bin_stats.items(), reverse=True)), {'%s %s' % k: v for k, v in sorted(stat.items(), key=lambda kv: -kv[1])[:10]})
