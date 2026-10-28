""
import json, sys
from shapely.geometry import Polygon
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); Z = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
GROW = float(opt.get('grow', '0.45')); CLR = float(opt.get('clr', '0.15'))
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
board = unary_union([poly(p) for p in C['outline']])
fills = {}
for z in C['zones']: fills.setdefault((z['layer'], z['net']), []).extend(poly(q) for q in z['polys'])
for f in C.get('formen', []): fills.setdefault((f['layer'], f['net']), []).extend(poly(q) for q in f['polys'])
plan = dict(sperren_weg=[], zonen={}); n = 0; zuwachs = 0.0
for z in Z:
    if z['regel'] or z['net'] == 'GND': continue
    L = z['lagen'][0]; O = unary_union([poly(p) for p in z['polys']])
    fremd = unary_union([g for (l, net), gs in fills.items() if l == L and net != z['net'] for g in gs]).buffer(CLR + 0.005)
    g = O.buffer(GROW, join_style=2).intersection(board).difference(fremd)
    teile = [t for t in getattr(g, 'geoms', [g]) if t.intersects(O.buffer(0.01))]
    neu = unary_union([O] + teile).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)
    d = neu.area - O.area
    if d < 0.002: continue
    n += 1; zuwachs += d
    plan['zonen'][z['uuid']] = [dict(aussen=[list(c) for c in gg.exterior.coords[:-1]], loecher=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                                for gg in getattr(neu, 'geoms', [neu]) if gg.area > 0.01]


akt = {}
for z in Z:
    if z['regel'] or z['net'] == 'GND': continue
    g = unary_union([poly(p) for p in plan['zonen'][z['uuid']]]) if z['uuid'] in plan['zonen'] else unary_union([poly(p) for p in z['polys']])
    akt[z['uuid']] = (z, g)
plan['zonen_weg'] = []; zusammen = 0
uu = list(akt)
for i, u in enumerate(uu):
    if u in plan['zonen_weg']: continue
    z, g = akt[u]
    changed = True
    while changed:
        changed = False
        for v in uu[i + 1:]:
            if v in plan['zonen_weg']: continue
            z2, g2 = akt[v]
            if (z2['lagen'], z2['net'], z2['prio']) == (z['lagen'], z['net'], z['prio']) and g.intersects(g2):
                g = g.union(g2); plan['zonen_weg'].append(v); zusammen += 1; changed = True
    akt[u] = (z, g)
    if g.area > 0:
        plan['zonen'][u] = [dict(aussen=[list(c) for c in gg.exterior.coords[:-1]], loecher=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                            for gg in getattr(g, 'geoms', [g]) if gg.area > 0.01]
for v in plan['zonen_weg']: plan['zonen'].pop(v, None)
json.dump(plan, open(OUT, 'w')); print('angedockt: %d Zonen, Umrissflaeche +%.1f mm², zusammengefasst: %d' % (n, zuwachs, zusammen))
