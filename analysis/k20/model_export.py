import json, sys, os
sys.path.insert(0, '.')
from shapely.geometry import Polygon, mapping
from shapely.ops import unary_union
from learning.copper import _via_ground
from common.zones import board_geometry
from analysis.copper_model import settings, areas, baseline, geometry, drills, keepout_area, attached, on_layer, via_ground
bj, kj, rj, gj, out = sys.argv[1:6]
B, K, R = json.load(open(bj)), json.load(open(kj)), json.load(open(rj))
design_rules = json.load(open(gj)) if os.path.exists(gj) else None
B = board_geometry(B, R)
nl = int(B.get('cu_layers') or 2)
cu_layers = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
clr, border = settings(K, design_rules, None, None)
min_width = 0.2
planes, gl, gnd = areas(B, R, K)
d_via, b_via = _via_ground(R, K)

all_items = {}
for net in sorted(set(planes.values()) | gnd):
    candidates = {}
    for L in cu_layers:
        if not (planes.get(L) == net or (L not in gl and net in gnd)): continue
        f = baseline(B, R, L, {net}, clr=clr, border=border, via_d=d_via, drill=b_via)
        if L not in gl: f = f.buffer(-min_width / 2, 8).buffer(min_width / 2, 8)
        candidates[L] = f
    all_items[net] = candidates
from shapely import voronoi_polygons
from shapely.geometry import MultiPoint
for L in cu_layers:
    active = [n for n, fs in all_items.items() if L in fs and not fs[L].is_empty]
    if len(active) < 2: continue
    points = {}
    for n, g in geometry(B, R, L, d_via):
        if n in active and not g.is_empty:
            q = g.representative_point(); points[(q.x, q.y)] = n
    if len(set(points.values())) < 2:
        for n in active:
            if n not in points.values(): all_items[n][L] = Polygon()
        continue
    coords = sorted(points)
    grid_cells = voronoi_polygons(MultiPoint(coords), extend_to=unary_union([all_items[n][L] for n in active]).envelope, ordered=True)
    roles = {n: [] for n in active}
    for xy, cell in zip(coords, grid_cells.geoms): roles[points[xy]].append(cell)
    for n in active: all_items[n][L] = all_items[n][L].intersection(unary_union(roles[n]).buffer(-clr / 2, join_style=2))
fills = {}
for net, candidates in all_items.items():
    joined = attached(B, R, candidates, {L: net for L in candidates}, via_d=d_via, drill=b_via)
    for L, f in joined.items():
        if not f.is_empty: fills[(net, L)] = f

def polys(g):
    res = []
    for p in getattr(g, 'geoms', [g]):
        if p.is_empty or p.geom_type != 'Polygon': continue
        res.append([list(p.exterior.coords), [list(i.coords) for i in p.interiors]])
    return res
cu = dict(layers=cu_layers, nets={}, vias=[], pads=[])
extra = dict(layers=cu_layers, outline=[], keepouts=[], zones=[], tracks=[], vias=[], min_clearance=clr, source='model', gnd_layers=sorted(gl))
outl = unary_union([Polygon(q, (B.get('outline_holes') or [[]] * len(B['outline']))[i] if i < len(B.get('outline_holes') or []) else []).buffer(0) for i, q in enumerate(B['outline']) if len(q) >= 3])
extra['outline'] = polys(outl)
for k in B.get('keepouts', []):
    extra['keepouts'].append(dict(layers=k.get('cu_layers') or cu_layers, no_copper=bool((k.get('prohibitions') or {}).get('areas')), polys=polys(keepout_area(k))))
for L in cu_layers:
    part_list = geometry(B, R, L, d_via)
    jen = {}
    for n, g in part_list: jen.setdefault(n, []).append(g)
    for (n, LL), f in fills.items():
        if LL == L: jen.setdefault(n, []).append(f)
    bore = drills(B, R, b_via, L)
    for n, gs in jen.items():
        g = unary_union(gs)
        for k in B.get('keepouts', []):
            if (k.get('prohibitions') or {}).get('areas') and L in (k.get('cu_layers') or cu_layers):
                pass
        cu['nets'].setdefault(n, {})[L] = polys(g)

    zg = {}
    for z in R.get('zone_list', []):
        if z['layer'] == L:
            zg.setdefault((z['net'], z.get('source', '')), []).append(unary_union([Polygon(q['outer'], q.get('inner') or []).buffer(0) for q in z['polys']]))
    for (n, LL), f in fills.items():
        if LL == L: zg.setdefault((n, 'model_fill'), []).append(f)
    for (n, q), gs in sorted(zg.items()):
        extra['zones'].append(dict(net=n, layer=L, source=q, prio=None, clearance=clr, min_width=None, fill=polys(unary_union(gs))))
for t in R.get('tracks', []):
    pts = t['pts']; ws = t['w'] if isinstance(t['w'], list) else [t['w']] * (len(pts) - 1)
    for a, b, w in zip(pts[:-1], pts[1:], ws):
        extra['tracks'].append(dict(net=t['net'], layer=t['layer'], a=a, b=b, w=w, arc=False))
for v in R.get('vias', []):
    d, dr = via_ground(v, d_via, b_via)
    vl = [L for L in cu_layers if on_layer(v, L)]
    extra['vias'].append(dict(net=v.get('net', ''), x=v['x'], y=v['y'], d=d, drill=dr, layers=vl))
extra['drills'] = [dict(x=p['x'], y=p['y'], d=p['drill'], dmax=p['drill'], npth=not p.get('net'), net=p.get('net', '')) for p in B['pads'] if p.get('drill')]
cu['vias'] = [dict(net=v['net'], x=v['x'], y=v['y'], d=v['d'], drill=v['drill']) for v in extra['vias']]
cu['pads'] = [dict(ref=p.get('ref'), num=p.get('num'), net=p.get('net'), x=p.get('x'), y=p.get('y'), layers=p.get('layers')) for p in B['pads']]
os.makedirs(out, exist_ok=True)
json.dump(cu, open(os.path.join(out, 'cu.json'), 'w')); json.dump(extra, open(os.path.join(out, 'extra.json'), 'w'))
print('Layers', cu_layers, 'gnd_layers', sorted(gl), 'planes', planes, 'clr', clr, 'border', border, 'Fills', len(fills), 'Zones', len(extra['zones']), 'TrackSegments', len(extra['tracks']))
