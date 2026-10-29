import json, sys
from shapely.geometry import Polygon
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); Z = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
GROW = float(opt.get('grow', '0.45')); CLR = float(opt.get('clr', '0.15'))
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
board = unary_union([poly(p) for p in C['outline']])
fills = {}
for z in C['zones']: fills.setdefault((z['layer'], z['net']), []).extend(poly(q) for q in z['polys'])
for f in C.get('shapes', []): fills.setdefault((f['layer'], f['net']), []).extend(poly(q) for q in f['polys'])
plan = dict(keepouts_way=[], zone_list={}); n = 0; growth = 0.0
for z in Z:
    if z['rule'] or z['net'] == 'GND': continue
    L = z['cu_layers'][0]; O = unary_union([poly(p) for p in z['polys']])
    foreign = unary_union([g for (l, net), gs in fills.items() if l == L and net != z['net'] for g in gs]).buffer(CLR + 0.005)
    g = O.buffer(GROW, join_style=2).intersection(board).difference(foreign)
    part_list = [t for t in getattr(g, 'geoms', [g]) if t.intersects(O.buffer(0.01))]
    new = unary_union([O] + part_list).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)
    d = new.area - O.area
    if d < 0.002: continue
    n += 1; growth += d
    plan['zone_list'][z['uuid']] = [dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                                for gg in getattr(new, 'geoms', [new]) if gg.area > 0.01]


act = {}
for z in Z:
    if z['rule'] or z['net'] == 'GND': continue
    g = unary_union([poly(p) for p in plan['zone_list'][z['uuid']]]) if z['uuid'] in plan['zone_list'] else unary_union([poly(p) for p in z['polys']])
    act[z['uuid']] = (z, g)
plan['zones_way'] = []; together = 0
uu = list(act)
for i, u in enumerate(uu):
    if u in plan['zones_way']: continue
    z, g = act[u]
    changed = True
    while changed:
        changed = False
        for v in uu[i + 1:]:
            if v in plan['zones_way']: continue
            z2, g2 = act[v]
            if (z2['cu_layers'], z2['net'], z2['prio']) == (z['cu_layers'], z['net'], z['prio']) and g.intersects(g2):
                g = g.union(g2); plan['zones_way'].append(v); together += 1; changed = True
    act[u] = (z, g)
    if g.area > 0:
        plan['zone_list'][u] = [dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                            for gg in getattr(g, 'geoms', [g]) if gg.area > 0.01]
for v in plan['zones_way']: plan['zone_list'].pop(v, None)
json.dump(plan, open(OUT, 'w')); print('docked: %d zones, outline area +%.1f mm², merged: %d' % (n, growth, together))
