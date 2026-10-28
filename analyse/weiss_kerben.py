""
import json, sys, collections
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
D = json.load(open(sys.argv[1])); opt = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
SP = json.load(open(sys.argv[2])) if len(sys.argv) > 2 and '=' not in sys.argv[2] else []
B = float(opt.get('b', '0.1')); AMIN = float(opt.get('amin', '0.0003')); N = int(opt.get('liste', '20'))
SPR = float(opt.get('sperr_rand', '0.12')); LOCH = float(opt.get('loch', '0.3'))
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board = poly(D['outline']).buffer(-0.3)
per = collections.defaultdict(list)
for net, lay in D['items'].items():
    for L, ps in lay.items(): per[L].append(poly(ps))
for z in D['zonen']: per[z['layer']].append(poly(z['polys']))
for f in D.get('formen', []): per[f['layer']].append(poly(f['polys']))
ges = 0; alle = []
for L in sorted(per):
    loecher = unary_union([Point(x, y).buffer(r + LOCH) for x, y, r in D['loecher']]) if D['loecher'] else Polygon()
    sperr = unary_union([Polygon(q['pts']).buffer(0) for q in SP if L in q['lagen'] and q['fuellen']])
    erzwungen = sperr.buffer(SPR).union(loecher)
    cu = unary_union(per[L]); W = board.difference(cu).difference(sperr)
    k = W.difference(W.buffer(-B / 2, quad_segs=8).buffer(B / 2, quad_segs=8))
    teile = [g for g in getattr(k, 'geoms', [k]) if g.geom_type == 'Polygon' and g.area > AMIN and g.distance(cu) < 0.001 and not erzwungen.intersects(g)]
    ges += len(teile); alle += [(L, g) for g in teile]
    print('%-4s Kerben %3d  Flaeche %.4f mm²' % (L, len(teile), sum(g.area for g in teile)))
print('gesamt', ges)
for L, g in sorted(alle, key=lambda t: -t[1].area)[:N]: print('   %-4s %.5f mm² bei (%.3f, %.3f)' % (L, g.area, g.centroid.x, g.centroid.y))
