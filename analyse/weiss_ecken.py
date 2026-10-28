""
import json, math, os, sys
import numpy as np
import shapely
from shapely.geometry import Polygon, Point
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.paarnetze import paarnetze, norm
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2]))
opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
DW = float(opt.get('d', '0.015')); KNICK = float(opt.get('knick', '35')); SPITZ = float(opt.get('spitz', '80'))
SR = float(opt.get('sperr_rand', '0.12')); LOCH = float(opt.get('loch', '0.3')); RAND = float(opt.get('rand', '0.55')); LISTE = int(opt.get('liste', '10'))
def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board = poly(D['outline']); board_in = board.buffer(-RAND)
loecher = unary_union([Point(x, y).buffer(r + LOCH, 24) for x, y, r in D['loecher']]) if D['loecher'] else Polygon()
umriss = {}
if opt.get('zonen'):
    for z in json.load(open(opt['zonen'])):
        if not z.get('net'): continue
        g = unary_union([Polygon(p['aussen'], p['loecher']).buffer(0) for p in z['polys']])
        for L in z['lagen']: umriss.setdefault((z['net'], L.replace('.Cu', '')), []).append(g.boundary)
umriss = {k: unary_union(v) for k, v in umriss.items()}
cu = {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): cu.setdefault((net, L), []).append(poly(ps))
for z in D['zonen']: cu.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
for f in D.get('formen', []): cu.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
cu = {k: unary_union(v).buffer(5e-4, quad_segs=2).buffer(-5e-4, quad_segs=2) for k, v in cu.items() if k[0]}

PAARE = paarnetze({n for n, _ in cu}, opt.get('klassen')) if (opt.get('klassen') or opt.get('pads')) else set()
PADS = {}
if opt.get('pads'):
    for p_ in json.load(open(opt['pads']))['pads']:
        for L_, qs in p_['polys'].items(): PADS.setdefault((p_['net'], L_), []).extend(Polygon(q['aussen'], q['loecher']).buffer(0) for q in qs)
def ecken_ring(ring):
    ""
    r = shapely.LineString(ring.coords).simplify(0.0005); n = len(r.coords) - 1; Lr = r.length
    if n < 3 or Lr < 4 * DW: return []
    xy = np.array(r.coords[:-1]); s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(np.array(r.coords), axis=0).T))])[:-1]
    pm = shapely.get_coordinates(shapely.line_interpolate_point(r, (s - DW) % Lr)); pp = shapely.get_coordinates(shapely.line_interpolate_point(r, (s + DW) % Lr))
    a = xy - pm; b = pp - xy
    t = np.degrees(np.arctan2(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0], (a * b).sum(1)))
    return [(xy[i, 0], xy[i, 1], -t[i]) for i in np.nonzero(t <= -KNICK)[0]]
erg = []; sperr_l = {}
for (net, L), g in sorted(cu.items()):
    if L not in sperr_l:
        sperr_l[L] = unary_union([Polygon(s_['pts']).buffer(0) for s_ in S if L in s_['lagen']]).buffer(SR)
    roh = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon': continue
        p = orient(p, 1.0)
        for ring in [p.exterior] + list(p.interiors): roh += ecken_ring(ring)
    roh.sort(key=lambda e: -e[2]); gew = []
    for x, y, k in roh:
        if any(math.hypot(x - u, y - v) < 0.03 for u, v, _ in gew): continue
        gew.append((x, y, k))
    um = umriss.get((net, L))
    for x, y, k in gew:
        P = Point(x, y)
        if not board_in.contains(P) or loecher.contains(P) or sperr_l[L].contains(P): continue
        if norm(net) in PAARE and not any(pg.distance(P) < 0.05 for pg in PADS.get((net, L), [])): art = 'paar'
        else: art = 'umriss' if um is not None and um.distance(P) < 0.01 else ('spitz' if k >= SPITZ else 'knick')
        erg.append(dict(layer=L, net=net, x=round(x, 3), y=round(y, 3), knick=round(k, 1), art=art))
for L in ('F', 'In1', 'In2', 'B'):
    e = [x for x in erg if x['layer'] == L]
    print('%-4s spitz %3d  Knick %3d  Umriss %3d  Paar %3d' % (L, sum(x['art'] == 'spitz' for x in e), sum(x['art'] == 'knick' for x in e), sum(x['art'] == 'umriss' for x in e), sum(x['art'] == 'paar' for x in e)))
print('gesamt spitz+Knick', sum(x['art'] in ('spitz', 'knick') for x in erg))
for x in sorted((x for x in erg if x['art'] in ('spitz', 'knick')), key=lambda x: -x['knick'])[:LISTE]:
    print('  %-4s %-28s (%.3f|%.3f) Knick %5.1f° %s' % (x['layer'], x['net'][:28], x['x'], x['y'], x['knick'], x['art']))
if opt.get('json'): json.dump(erg, open(opt['json'], 'w'))
