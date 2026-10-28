""
import json, sys, os, math
import numpy as np
from shapely.prepared import prep
from shapely.strtree import STRtree
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from gemeinsam.paarnetze import norm
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); Z = json.load(open(sys.argv[3])); OUT = sys.argv[4]
opt = dict(a.split('=', 1) for a in sys.argv[5:] if '=' in a)
BMIN = float(opt.get('bmin', '0.13'))
NETZE = set(opt['netze'].split(',')) if opt.get('netze') else {z['net'] for z in Z if not z['regel'] and z['net'] and z['net'] != opt.get('gnd', 'GND')}
VERS = {norm(n) for n, k in json.load(open(opt['klassen']))['netze'].items() if k.get('art') == 'versorgung'} - NETZE if opt.get('klassen') else set()
CLR = float(opt.get('clr', '0.15')); TOL = float(opt.get('tol', '0.01')); AMIN = float(opt.get('amin', '0.03'))
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-0.5)
plan = dict(sperren_weg=[], zonen={}, neu=[])

VD, VDR = [float(q) for q in opt.get('via', '0.45/0.2').split('/')]; VR = VD / 2
DRC = float(opt.get('drc', '0.128')) + 0.01; H2H = float(opt.get('h2h', '0.25')) + 0.01; RAND = float(opt.get('rand', '0.5')) + 0.01
LOCH = float(opt.get('loch', '0.255')) + 0.01
innerv = prep(board.buffer(-(RAND + VR)))
hart = [(LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2) for t in C['tracks'] if t['net'] != 'GND']
hart += [poly(q) for p in C['pads'] if p['net'] != 'GND' for qs in p['polys'].values() for q in qs]
hart += [Point(v['x'], v['y']).buffer(v['d'] / 2) for v in C['vias'] if v['net'] != 'GND']
hart += [poly(q) for f in C.get('formen', []) if f['net'] != 'GND' for q in f['polys']]
hart = [g for g in hart if not g.is_empty]; tree = STRtree(hart)
bohr = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p.get('drill', 0) > 0]
bohr_pads = [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p.get('drill', 0) > 0 and p['net'] != 'GND']
sperrv = prep(unary_union([Polygon(s['pts']).buffer(VR + 0.05) for s in S if s['fuellen'] or s['vias']]))
def via_ok(x, y):
    pt = Point(x, y)
    if not innerv.contains(pt) or sperrv.contains(pt): return False
    if any(hart[int(j)].distance(pt) < VR + DRC for j in tree.query(pt.buffer(VR + DRC))): return False
    if any(math.hypot(bx - x, by - y) < br + VDR / 2 + H2H for bx, by, br in bohr): return False
    return not any(math.hypot(bx - x, by - y) < br + VR + LOCH for bx, by, br in bohr_pads)
def via_passt(g, L):
    bx = g.buffer(VR).bounds; gi = prep(g.buffer(VR - 0.05))
    return any(gi.contains(Point(x, y)) and via_ok(x, y) for x in np.arange(bx[0], bx[2], 0.025) for y in np.arange(bx[1], bx[3], 0.025))
for L in ('F', 'In2', 'B'):
    items = []
    for t in C['tracks']:
        if t['layer'] == L: items.append(((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2), t['net']))
    for v in C['vias']: items.append((Point(v['x'], v['y']).buffer(v['d'] / 2), v['net']))
    for p in C['pads']:
        items += [(poly(q), p['net']) for q in p['polys'].get(L, [])]
        if p.get('drill', 0) > 0: items.append((Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.105), '#Bohrung'))
    for z in C['zones']:
        if z['layer'] == L: items += [(poly(q), z['net']) for q in z['polys']]
    for f in C.get('formen', []):
        if f['layer'] == L: items += [(poly(q), f['net']) for q in f['polys']]
    sperr = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['lagen'] and s['fuellen']])
    weiss = inner.difference(unary_union([g for g, _ in items]).buffer(CLR + TOL)).difference(sperr.buffer(CLR))
    for g in getattr(weiss, 'geoms', [weiss]):
        if g.area < AMIN or g.buffer(-BMIN / 2).is_empty: continue
        rand = g.buffer(CLR + TOL + 0.04)
        kontakt = {}
        for it, net in items:
            if (net in NETZE or norm(net) in VERS) and it.intersects(rand): kontakt[net] = kontakt.get(net, 0) + it.intersection(rand).area
        if not kontakt: continue
        net = max(kontakt, key=kontakt.get)
        if net not in NETZE and via_passt(g, L):
            print('%-3s Tasche %.3f mm² bei (%.2f, %.2f) an Bahn %s: GND-Via passt, bleibt fuer luecken_vias' % (L, g.area, g.centroid.x, g.centroid.y, net)); continue
        umriss = g.buffer(CLR + 0.05, join_style=2)

        ziel = None
        for z in Z:
            if not z['regel'] and z['net'] == net and L in z['lagen']:
                zo = unary_union([poly(p) for p in plan['zonen'].get(z['uuid'], z['polys'])]) if z['uuid'] in plan['zonen'] else unary_union([poly(p) for p in z['polys']])
                if zo.intersects(umriss): ziel = (z['uuid'], zo); break
        if ziel:
            u, zo = ziel; ng = zo.union(umriss)
            plan['zonen'][u] = [dict(aussen=[list(c) for c in gg.exterior.coords[:-1]], loecher=[[list(c) for c in h.coords[:-1]] for h in gg.interiors]) for gg in getattr(ng, 'geoms', [ng])]
            print('%-3s Tasche %.3f mm² bei (%.2f, %.2f) -> Zone %s erweitert' % (L, g.area, g.centroid.x, g.centroid.y, net))
        else:
            plan['neu'].append(dict(net=net, lage=L, prio=6, polys=[dict(aussen=[list(c) for c in umriss.exterior.coords[:-1]], loecher=[])]))
            print('%-3s Tasche %.3f mm² bei (%.2f, %.2f) -> neue Zone %s' % (L, g.area, g.centroid.x, g.centroid.y, net))
json.dump(plan, open(OUT, 'w')); print('neue Zonen:', len(plan['neu']), '| erweitert:', len(plan['zonen']))
