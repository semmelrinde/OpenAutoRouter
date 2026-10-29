import json, sys
from shapely.geometry import Polygon
from shapely.ops import unary_union
Z = json.load(open(sys.argv[1])); OUT = sys.argv[2]
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
zs = [z for z in Z if not z['rule'] and z['net'] != 'GND']
geo = {z['uuid']: unary_union([poly(p) for p in z['polys']]) for z in zs}
plan = dict(keepouts_way=[], zone_list={}, zones_way=[]); n = 0
for i, z in enumerate(zs):
    if z['uuid'] in plan['zones_way']: continue
    g = geo[z['uuid']]; changed = True; part = False
    while changed:
        changed = False
        for z2 in zs[i + 1:]:
            if z2['uuid'] in plan['zones_way']: continue
            if (z2['cu_layers'], z2['net'], z2['prio']) == (z['cu_layers'], z['net'], z['prio']) and g.buffer(0.001).intersects(geo[z2['uuid']]):
                g = g.union(geo[z2['uuid']]); plan['zones_way'].append(z2['uuid']); changed = True; part = True; n += 1
    if part:
        plan['zone_list'][z['uuid']] = [dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                                    for gg in getattr(g, 'geoms', [g]) if gg.area > 0.005]
json.dump(plan, open(OUT, 'w')); print('merged:', n, 'Zones in', len(plan['zone_list']))
