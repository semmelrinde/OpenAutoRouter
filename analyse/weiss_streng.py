""
import json, sys, collections
try:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
except ImportError:
    plt = None
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); png = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
CLR = float(opt.get('clr', '0.15')); TOL = float(opt.get('tol', '0.03'))
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-0.5)
erg = {}
axs = plt.subplots(1, 3, figsize=(30, 13))[1] if plt else [None] * 3
for ax, L in zip(axs, ('F', 'In2', 'B')):
    cu = []
    for t in C['tracks']:
        if t['layer'] == L: cu.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
    for v in C['vias']: cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
    for p in C['pads']:
        cu += [poly(q) for q in p['polys'].get(L, [])]
        if p.get('drill', 0) > 0: cu.append(Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.255 - CLR))
    for z in C['zones']:
        if z['layer'] == L: cu += [poly(q) for q in z['polys']]
    for f in C.get('formen', []):
        if f['layer'] == L: cu += [poly(q) for q in f['polys']]
    kupfer = unary_union(cu)
    sperr = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['lagen'] and s['fuellen']])
    weiss = inner.difference(kupfer.buffer(CLR + TOL)).difference(sperr.buffer(CLR))
    teile = [g for g in getattr(weiss, 'geoms', [weiss]) if g.area > 0.002]
    erg[L] = teile
    if ax is None: continue
    for q in getattr(board, 'geoms', [board]): ax.plot(*q.exterior.xy, 'k-', lw=0.6)
    for g in getattr(kupfer, 'geoms', [kupfer]):
        if g.geom_type == 'Polygon': ax.fill(*g.exterior.xy, color='0.85', lw=0)
    for g in teile: ax.fill(*g.buffer(0.15).exterior.xy, color='r', lw=0)
    ax.set_title('%s: %d unnoetig weisse Stellen, zusammen %.2f mm²' % (L, len(teile), sum(g.area for g in teile)))
    ax.set_aspect('equal'); ax.invert_yaxis()
if plt: plt.tight_layout(); plt.savefig(png, dpi=80)
for L, ts in erg.items():
    h = collections.Counter('<0.02' if g.area < 0.02 else '<0.1' if g.area < 0.1 else '<0.5' if g.area < 0.5 else '>=0.5' for g in ts)
    print(L, len(ts), 'Stellen, %.2f mm²' % sum(g.area for g in ts), dict(h))
json.dump({L: [dict(a=round(g.area, 4), x=round(g.centroid.x, 3), y=round(g.centroid.y, 3), w=round(2 * g.area / max(g.length, 1e-9), 3)) for g in ts] for L, ts in erg.items()}, open(png.replace('.png', '.json'), 'w'))
