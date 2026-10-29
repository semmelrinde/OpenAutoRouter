import json, sys
from shapely.geometry import Polygon, LineString, Point, mapping
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); Z = json.load(open(sys.argv[2])); OUT = sys.argv[3]
SP = json.load(open(sys.argv[4])) if len(sys.argv) > 4 and not '=' in sys.argv[4] else []
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
CLR = float(opt.get('clr', '0.15')); BORDER = float(opt.get('border', '0.5')); AMIN = float(opt.get('min', '0.3'))
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-BORDER)
pinned = [z for z in Z if z['rule'] and (z['fill'] and unary_union([poly(p) for p in z['polys']]).area >= 20)]
splitter = [z for z in Z if z['rule'] and z['fill'] and unary_union([poly(p) for p in z['polys']]).area < 20]
JTAG = []
plan = dict(keepouts_way=[], zone_list={})
for L in ('F', 'In2', 'B'):
    foreign = []
    for t in C['tracks']:
        if t['layer'] == L: foreign.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2 + CLR))
    for v in C['vias']: foreign.append(Point(v['x'], v['y']).buffer(v['d'] / 2 + CLR))
    for p in C['pads']:
        for q in p['polys'].get(L, []): foreign.append(poly(q).buffer(CLR))
        if p.get('drill', 0) > 0: foreign.append(Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.255))
    for f in C.get('shapes', []):
        if f['layer'] == L: foreign += [poly(q).buffer(CLR) for q in f['polys']]
    fills = {}
    for z in C['zones']:
        if z['layer'] == L: fills.setdefault(z['net'], []).extend(poly(q) for q in z['polys'])
    all_fills = unary_union([g.buffer(CLR) for gs in fills.values() for g in gs])
    pinned_list = unary_union([poly(p) for z in pinned if L in z['cu_layers'] for p in z['polys']] + [Polygon(q['pts']).buffer(0) for q in SP if q['ref'] and L in q['cu_layers']])
    vacant = inner.difference(unary_union(foreign)).difference(all_fills).difference(pinned_list).buffer(-0.075).buffer(0.075)
    regs = [g for g in getattr(vacant, 'geoms', [vacant]) if g.area >= AMIN]
    zones_L = [z for z in Z if not z['rule'] and L in z['cu_layers'] and z['net'] != 'GND']
    for g in regs:
        best = None
        for z in zones_L:
            fz = unary_union([fg for fg in fills.get(z['net'], [])]).intersection(unary_union([poly(p) for p in z['polys']]).buffer(0.01))
            if fz.is_empty: continue
            contact = g.buffer(CLR + 0.06).intersection(fz).area
            if contact > 1e-4 and (best is None or contact > best[0]): best = (contact, z)
        if best is None: continue
        z = best[1]
        prior = plan['zone_list'].get(z['uuid'], unary_union([poly(p) for p in z['polys']]))
        plan['zone_list'][z['uuid']] = prior.union(g.buffer(CLR + 0.05))
        for s in splitter:
            if L in s['cu_layers'] and unary_union([poly(p) for p in s['polys']]).intersects(g) and s['uuid'] not in plan['keepouts_way']:
                plan['keepouts_way'].append(s['uuid'])
        print('%-3s %.2f mm² at (%.2f, %.2f) -> zone %s (prio %d)' % (L, g.area, g.centroid.x, g.centroid.y, z['net'], z['prio']))
def rings(g):
    out = []
    for gg in getattr(g, 'geoms', [g]):
        gg = gg.simplify(0.002)
        out.append(dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors]))
    return out
json.dump(dict(keepouts_way=plan['keepouts_way'], zone_list={u: rings(g) for u, g in plan['zone_list'].items()}), open(OUT, 'w'))
print('Zones extended:', len(plan['zone_list']), '| sliver keepouts deleted:', len(plan['keepouts_way']))
