""
import json, sys
from shapely.geometry import Polygon
from shapely.ops import unary_union
Z = json.load(open(sys.argv[1])); OUT = sys.argv[2]
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
zs = [z for z in Z if not z['regel'] and z['net'] != 'GND']
geo = {z['uuid']: unary_union([poly(p) for p in z['polys']]) for z in zs}
plan = dict(sperren_weg=[], zonen={}, zonen_weg=[]); n = 0
for i, z in enumerate(zs):
    if z['uuid'] in plan['zonen_weg']: continue
    g = geo[z['uuid']]; changed = True; teil = False
    while changed:
        changed = False
        for z2 in zs[i + 1:]:
            if z2['uuid'] in plan['zonen_weg']: continue
            if (z2['lagen'], z2['net'], z2['prio']) == (z['lagen'], z['net'], z['prio']) and g.buffer(0.001).intersects(geo[z2['uuid']]):
                g = g.union(geo[z2['uuid']]); plan['zonen_weg'].append(z2['uuid']); changed = True; teil = True; n += 1
    if teil:
        plan['zonen'][z['uuid']] = [dict(aussen=[list(c) for c in gg.exterior.coords[:-1]], loecher=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                                    for gg in getattr(g, 'geoms', [g]) if gg.area > 0.005]
json.dump(plan, open(OUT, 'w')); print('verschmolzen:', n, 'Zonen in', len(plan['zonen']))
