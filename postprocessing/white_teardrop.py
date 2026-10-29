import json, os, sys
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.pair_nets import pair_nets, norm
from common.copper_shape_anchor import anchor_load, connect
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
PADS = {}
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
ANCHOR = {}
if opt.get('pads'):
    _C = json.load(open(opt['pads']))
    ANCHOR = anchor_load(_C)
    for p in _C['pads']:
        for L, qs in p['polys'].items():
            for q in qs: PADS.setdefault(L, []).append(Polygon(q['outer'], q['hole_list']).buffer(0))
R = float(opt.get('r', '0.4')); FREE = float(opt.get('vacant', '0.155')); BORDER = float(opt.get('border', '0.5'))
FREE_HARD = float(opt.get('free_hard', '0.155'))
RADII = [R] + sorted({float(x) for x in opt.get('radii', '0.3,0.2,0.15,0.1,0.07,0.05').split(',') if x and float(x) < R}, reverse=True)
ZONES_HARD = opt.get('zone_list', 'hard') == 'hard'
EPS = 2e-6
HAIR = 0.001
THICKNESS = float(opt.get('thickness', '0.008'))
HOLE = float(opt.get('hole_size', '0.27')); AMIN = float(opt.get('amin', '0.00002'))
OFF = [a for a in opt.get('off', '').split(',') if a]
NETS = opt.get('net_list', 'surface' if opt.get('only_areas', '1') == '1' else 'all_items')


def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board_in = poly(D['outline']).buffer(-BORDER)
hole_list = unary_union([Point(x, y).buffer(r + HOLE, 24) for x, y, r in D['hole_list']]) if D['hole_list'] else Polygon()
copper_map, hard, zfill = {}, {}, {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): copper_map.setdefault((net, L), []).append(poly(ps)); hard.setdefault((net, L), []).append(poly(ps))
for z in D['zone_list']: copper_map.setdefault((z['net'], z['layer']), []).append(poly(z['polys'])); zfill.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
for f in D.get('shapes', []): copper_map.setdefault((f['net'], f['layer']), []).append(poly(f['polys'])); hard.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
PAIRS = pair_nets({n for n, _ in copper_map}, opt.get('netclasses'))
copper_map = {k: unary_union(v) for k, v in copper_map.items()}; hard = {k: unary_union(v) for k, v in hard.items()}; zfill = {k: unary_union(v) for k, v in zfill.items()}
out = []; stat = {}; rstat = {}; rejected = 0; new = {}
from common.spacing_gate import too_close, export_shape
for L in sorted({L for _, L in copper_map}):
    all_items = {n: g for (n, l), g in copper_map.items() if l == L and n}
    blocked = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['cu_layers'] and (s['fill'] or s['track_list'])])
    with_zone = {z['net'] for z in D['zone_list'] if z['layer'] == L}
    for n, U in sorted(all_items.items()):
        pair = norm(n) in PAIRS
        if (pair and not PADS) or any(a in n for a in OFF) or (NETS == 'surface' and n not in with_zone) or (NETS == 'signal' and n in with_zone): continue


        foreign_h = unary_union([g for (m, l), g in hard.items() if l == L and m and m != n] + [g for (m, l), g in new.items() if l == L and m != n])
        foreign_z = unary_union([g for (m, l), g in zfill.items() if l == L and m and m != n]) if ZONES_HARD else Polygon()
        ban = unary_union([foreign_h.buffer(FREE_HARD, quad_segs=16), foreign_z.buffer(FREE, quad_segs=16), blocked, hole_list])
        def growth(Ux, r): return Ux.buffer(r, quad_segs=16).buffer(-r, quad_segs=16).difference(Ux)
        def pieces(g): return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= AMIN]
        def fits(q):
            if q.difference(board_in).area > EPS or (q.intersects(ban) and q.intersection(ban).area > EPS): return False
            an = [pg for pg in PADS.get(L, []) if pg.distance(q) < 0.01]
            if len(an) >= 2: return False
            if pair and (not an or not an[0].buffer(1.5 * R, quad_segs=16).contains(q)): return False
            return True
        open_items = [(0, q) for q in pieces(growth(U, RADII[0]))]; n_add = 0
        while open_items:
            k, q = open_items.pop()
            q = q.buffer(-HAIR, quad_segs=4).buffer(HAIR, quad_segs=4)
            if q.geom_type != 'Polygon':
                open_items += [(k, p) for p in pieces(q)]; continue
            if q.area < AMIN or q.buffer(-THICKNESS / 2).is_empty or q.distance(U) > 0.002: continue
            if not fits(q):
                if k + 1 < len(RADII):
                    r2 = RADII[k + 1]; Uloc = U.intersection(q.buffer(2 * r2 + 0.01))
                    open_items += [(k + 1, q2) for q2 in pieces(growth(Uloc, r2).intersection(q.buffer(0.001)))]
                continue
            g = q.buffer(0.02, quad_segs=8).intersection(unary_union([U, q]).buffer(1e-7))

            for gg in getattr(g, 'geoms', [g]):
                if gg.geom_type == 'Polygon' and gg.area >= AMIN:
                    gg = connect(gg, U, ANCHOR.get((n, L), []))
                    pts = [list(c) for c in gg.simplify(0.0005).exterior.coords][:-1]
                    if too_close(export_shape(pts), foreign_h, FREE_HARD, foreign_z if ZONES_HARD else None, FREE):
                        rejected += 1; continue
                    out.append(dict(net=n, layer=L, r=RADII[k], pts=pts)); n_add += 1
                    new[(n, L)] = unary_union([new.get((n, L), Polygon()), export_shape(pts)])
                    rstat[RADII[k]] = rstat.get(RADII[k], 0) + 1
        if n_add: stat[(L, n)] = n_add
json.dump(out, open(OUT, 'w'))
print('Teardrop-Shapes:', len(out), 'rejected (spacing):', rejected, 'per radius', dict(sorted(rstat.items(), reverse=True)), {'%s %s' % k: v for k, v in sorted(stat.items(), key=lambda kv: -kv[1])[:8]})
