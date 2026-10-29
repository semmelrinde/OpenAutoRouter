import json, sys, collections
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
D = json.load(open(sys.argv[1])); opt = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
SP = json.load(open(sys.argv[2])) if len(sys.argv) > 2 and '=' not in sys.argv[2] else []
B = float(opt.get('b', '0.1')); AMIN = float(opt.get('amin', '0.0003')); N = int(opt.get('list_', '20'))
SPR = float(opt.get('keepout_border', '0.12')); HOLE = float(opt.get('hole_size', '0.3'))
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board = poly(D['outline']).buffer(-0.3)
per = collections.defaultdict(list)
for net, lay in D['items'].items():
    for L, ps in lay.items(): per[L].append(poly(ps))
for z in D['zone_list']: per[z['layer']].append(poly(z['polys']))
for f in D.get('shapes', []): per[f['layer']].append(poly(f['polys']))
ges = 0; all_items = []
for L in sorted(per):
    hole_list = unary_union([Point(x, y).buffer(r + HOLE) for x, y, r in D['hole_list']]) if D['hole_list'] else Polygon()
    blocked = unary_union([Polygon(q['pts']).buffer(0) for q in SP if L in q['cu_layers'] and q['fill']])
    forced = blocked.buffer(SPR).union(hole_list)
    cu = unary_union(per[L]); W = board.difference(cu).difference(blocked)
    k = W.difference(W.buffer(-B / 2, quad_segs=8).buffer(B / 2, quad_segs=8))
    part_list = [g for g in getattr(k, 'geoms', [k]) if g.geom_type == 'Polygon' and g.area > AMIN and g.distance(cu) < 0.001 and not forced.intersects(g)]
    ges += len(part_list); all_items += [(L, g) for g in part_list]
    print('%-4s notches %3d  area %.4f mm²' % (L, len(part_list), sum(g.area for g in part_list)))
print('total', ges)
for L, g in sorted(all_items, key=lambda t: -t[1].area)[:N]: print('   %-4s %.5f mm² at (%.3f, %.3f)' % (L, g.area, g.centroid.x, g.centroid.y))
