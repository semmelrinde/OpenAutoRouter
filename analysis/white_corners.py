import json, math, os, sys
import numpy as np
import shapely
from shapely.geometry import Polygon, Point
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.pair_nets import pair_nets, norm
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2]))
opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
DW = float(opt.get('d', '0.015')); BEND = float(opt.get('bend', '35')); SHARP = float(opt.get('sharp', '80'))
SR = float(opt.get('keepout_border', '0.12')); HOLE = float(opt.get('hole_size', '0.3')); BORDER = float(opt.get('border', '0.55')); LIST = int(opt.get('list_', '10'))
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board = poly(D['outline']); board_in = board.buffer(-BORDER)
hole_list = unary_union([Point(x, y).buffer(r + HOLE, 24) for x, y, r in D['hole_list']]) if D['hole_list'] else Polygon()
contour = {}
if opt.get('zone_list'):
    for z in json.load(open(opt['zone_list'])):
        if not z.get('net'): continue
        g = unary_union([Polygon(p['outer'], p['hole_list']).buffer(0) for p in z['polys']])
        for L in z['cu_layers']: contour.setdefault((z['net'], L.replace('.Cu', '')), []).append(g.boundary)
contour = {k: unary_union(v) for k, v in contour.items()}
cu = {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): cu.setdefault((net, L), []).append(poly(ps))
for z in D['zone_list']: cu.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
for f in D.get('shapes', []): cu.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
cu = {k: unary_union(v).buffer(5e-4, quad_segs=2).buffer(-5e-4, quad_segs=2) for k, v in cu.items() if k[0]}

PAIRS = pair_nets({n for n, _ in cu}, opt.get('netclasses')) if (opt.get('netclasses') or opt.get('pads')) else set()
PADS = {}
if opt.get('pads'):
    for p_ in json.load(open(opt['pads']))['pads']:
        for L_, qs in p_['polys'].items(): PADS.setdefault((p_['net'], L_), []).extend(Polygon(q['outer'], q['hole_list']).buffer(0) for q in qs)
def corners_ring(ring):
    r = shapely.LineString(ring.coords).simplify(0.0005); n = len(r.coords) - 1; Lr = r.length
    if n < 3 or Lr < 4 * DW: return []
    xy = np.array(r.coords[:-1]); s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(np.array(r.coords), axis=0).T))])[:-1]
    pm = shapely.get_coordinates(shapely.line_interpolate_point(r, (s - DW) % Lr)); pp = shapely.get_coordinates(shapely.line_interpolate_point(r, (s + DW) % Lr))
    a = xy - pm; b = pp - xy
    t = np.degrees(np.arctan2(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0], (a * b).sum(1)))
    return [(xy[i, 0], xy[i, 1], -t[i]) for i in np.nonzero(t <= -BEND)[0]]
erg = []; keepout_l = {}
for (net, L), g in sorted(cu.items()):
    if L not in keepout_l:
        keepout_l[L] = unary_union([Polygon(s_['pts']).buffer(0) for s_ in S if L in s_['cu_layers']]).buffer(SR)
    raw_data = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon': continue
        p = orient(p, 1.0)
        for ring in [p.exterior] + list(p.interiors): raw_data += corners_ring(ring)
    raw_data.sort(key=lambda e: -e[2]); wt = []
    for x, y, k in raw_data:
        if any(math.hypot(x - u, y - v) < 0.03 for u, v, _ in wt): continue
        wt.append((x, y, k))
    um = contour.get((net, L))
    for x, y, k in wt:
        P = Point(x, y)
        if not board_in.contains(P) or hole_list.contains(P) or keepout_l[L].contains(P): continue
        if norm(net) in PAIRS and not any(pg.distance(P) < 0.05 for pg in PADS.get((net, L), [])): category = 'pair'
        else: category = 'contour' if um is not None and um.distance(P) < 0.01 else ('sharp' if k >= SHARP else 'bend')
        erg.append(dict(layer=L, net=net, x=round(x, 3), y=round(y, 3), bend=round(k, 1), category=category))
for L in ('F', 'In1', 'In2', 'B'):
    e = [x for x in erg if x['layer'] == L]
    print('%-4s sharp %3d  bend %3d  outline %3d  pair %3d' % (L, sum(x['category'] == 'sharp' for x in e), sum(x['category'] == 'bend' for x in e), sum(x['category'] == 'contour' for x in e), sum(x['category'] == 'pair' for x in e)))
print('total sharp+bend', sum(x['category'] in ('sharp', 'bend') for x in erg))
for x in sorted((x for x in erg if x['category'] in ('sharp', 'bend')), key=lambda x: -x['bend'])[:LIST]:
    print('  %-4s %-28s (%.3f|%.3f) bend %5.1f° %s' % (x['layer'], x['net'][:28], x['x'], x['y'], x['bend'], x['category']))
if opt.get('json'): json.dump(erg, open(opt['json'], 'w'))
