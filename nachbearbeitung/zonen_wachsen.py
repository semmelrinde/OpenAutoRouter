""
import json, sys
from shapely.geometry import Polygon, LineString, Point, mapping
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); Z = json.load(open(sys.argv[2])); OUT = sys.argv[3]
SP = json.load(open(sys.argv[4])) if len(sys.argv) > 4 and not '=' in sys.argv[4] else []
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
CLR = float(opt.get('clr', '0.15')); RAND = float(opt.get('rand', '0.5')); AMIN = float(opt.get('min', '0.3'))
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-RAND)
fest = [z for z in Z if z['regel'] and (z['fuellen'] and unary_union([poly(p) for p in z['polys']]).area >= 20)]
splitter = [z for z in Z if z['regel'] and z['fuellen'] and unary_union([poly(p) for p in z['polys']]).area < 20]
JTAG = []
plan = dict(sperren_weg=[], zonen={})
for L in ('F', 'In2', 'B'):
    fremd = []
    for t in C['tracks']:
        if t['layer'] == L: fremd.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2 + CLR))
    for v in C['vias']: fremd.append(Point(v['x'], v['y']).buffer(v['d'] / 2 + CLR))
    for p in C['pads']:
        for q in p['polys'].get(L, []): fremd.append(poly(q).buffer(CLR))
        if p.get('drill', 0) > 0: fremd.append(Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.255))
    for f in C.get('formen', []):
        if f['layer'] == L: fremd += [poly(q).buffer(CLR) for q in f['polys']]
    fills = {}
    for z in C['zones']:
        if z['layer'] == L: fills.setdefault(z['net'], []).extend(poly(q) for q in z['polys'])
    alle_fills = unary_union([g.buffer(CLR) for gs in fills.values() for g in gs])
    feste = unary_union([poly(p) for z in fest if L in z['lagen'] for p in z['polys']] + [Polygon(q['pts']).buffer(0) for q in SP if q['ref'] and L in q['lagen']])
    frei = inner.difference(unary_union(fremd)).difference(alle_fills).difference(feste).buffer(-0.075).buffer(0.075)
    regs = [g for g in getattr(frei, 'geoms', [frei]) if g.area >= AMIN]
    zonen_L = [z for z in Z if not z['regel'] and L in z['lagen'] and z['net'] != 'GND']
    for g in regs:
        best = None
        for z in zonen_L:
            fz = unary_union([fg for fg in fills.get(z['net'], [])]).intersection(unary_union([poly(p) for p in z['polys']]).buffer(0.01))
            if fz.is_empty: continue
            kontakt = g.buffer(CLR + 0.06).intersection(fz).area
            if kontakt > 1e-4 and (best is None or kontakt > best[0]): best = (kontakt, z)
        if best is None: continue
        z = best[1]
        alt = plan['zonen'].get(z['uuid'], unary_union([poly(p) for p in z['polys']]))
        plan['zonen'][z['uuid']] = alt.union(g.buffer(CLR + 0.05))
        for s in splitter:
            if L in s['lagen'] and unary_union([poly(p) for p in s['polys']]).intersects(g) and s['uuid'] not in plan['sperren_weg']:
                plan['sperren_weg'].append(s['uuid'])
        print('%-3s %.2f mm² bei (%.2f, %.2f) -> Zone %s (prio %d)' % (L, g.area, g.centroid.x, g.centroid.y, z['net'], z['prio']))
def ringe(g):
    out = []
    for gg in getattr(g, 'geoms', [g]):
        gg = gg.simplify(0.002)
        out.append(dict(aussen=[list(c) for c in gg.exterior.coords[:-1]], loecher=[[list(c) for c in h.coords[:-1]] for h in gg.interiors]))
    return out
json.dump(dict(sperren_weg=plan['sperren_weg'], zonen={u: ringe(g) for u, g in plan['zonen'].items()}), open(OUT, 'w'))
print('Zonen erweitert:', len(plan['zonen']), '| Splitter-Sperren geloescht:', len(plan['sperren_weg']))
