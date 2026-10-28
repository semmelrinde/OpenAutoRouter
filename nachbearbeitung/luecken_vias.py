""
import json, sys, math
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.strtree import STRtree
C = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
VD, VDR = [float(q) for q in opt.get('via', '0.5/0.2').split('/')]; VR = VD / 2
DRC = float(opt.get('drc', '0.128')) + 0.01; H2H = float(opt.get('h2h', '0.25')) + 0.01; RAND = float(opt.get('rand', '0.5')) + 0.01
CLR = float(opt.get('clr', '0.15')); TOL = float(opt.get('tol', '0.01'))
def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-0.5); innerv = prep(board.buffer(-(RAND + VR)))
hart = []
for t in C['tracks']:
    if t['net'] != 'GND': hart.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
for p in C['pads']:
    if p['net'] != 'GND': hart += [poly(q) for L, qs in p['polys'].items() for q in qs]
for v in C['vias']:
    if v['net'] != 'GND': hart.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
for f in C.get('formen', []):
    if f['net'] != 'GND': hart += [poly(q) for q in f['polys']]
hart = [g for g in hart if not g.is_empty]; tree = STRtree(hart)
bohr = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0]
sperr = prep(unary_union([Polygon(s['pts']).buffer(VR + 0.05) for s in S if s['fuellen'] or s['vias']]))
LOCH = float(opt.get('loch', '0.255')) + 0.01
bohr_pads = [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0 and p['net'] != 'GND']
def ok(x, y, neu):
    pt = Point(x, y)
    if not innerv.contains(pt) or sperr.contains(pt): return False
    for j in tree.query(pt.buffer(VR + DRC)):
        if hart[int(j)].distance(pt) < VR + DRC: return False
    for bx, by, br in bohr + [(a, b, VDR / 2) for a, b in neu]:
        if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
    for bx, by, br in bohr_pads:
        if math.hypot(bx - x, by - y) < br + VR + LOCH: return False
    return True
neu = []
for L in ('F', 'In2', 'B'):
    cu = []
    for t in C['tracks']:
        if t['layer'] == L: cu.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
    for v in C['vias']: cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
    for p in C['pads']:
        cu += [poly(q) for q in p['polys'].get(L, [])]
        if p.get('drill', 0) > 0: cu.append(Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.105))
    for z in C['zones']:
        if z['layer'] == L: cu += [poly(q) for q in z['polys']]
    for f in C.get('formen', []):
        if f['layer'] == L: cu += [poly(q) for q in f['polys']]
    weiss = inner.difference(unary_union(cu).buffer(CLR + TOL)).difference(unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['lagen'] and s['fuellen']]).buffer(CLR))
    for g in getattr(weiss, 'geoms', [weiss]):
        if g.area < 0.01: continue
        if any(g.distance(Point(a, b)) < VR + 0.2 for a, b in neu): continue
        bx = g.buffer(VR).bounds
        kand = [(round(x, 3), round(y, 3)) for x in np.arange(bx[0], bx[2], 0.025) for y in np.arange(bx[1], bx[3], 0.025)
                if g.buffer(VR - 0.05).contains(Point(x, y))]
        kand = [c for c in kand if ok(c[0], c[1], neu)]
        if not kand: continue
        best = min(kand, key=lambda c: g.centroid.distance(Point(c)))
        neu.append(best); print('%-3s Luecke %.3f mm² bei (%.2f, %.2f) -> GND-Via (%.3f, %.3f)' % (L, g.area, g.centroid.x, g.centroid.y, best[0], best[1]))
        zweite = [c for c in kand if math.hypot(c[0] - best[0], c[1] - best[1]) >= VD + H2H and ok(c[0], c[1], neu)]
        if zweite:
            c2 = max(zweite, key=lambda c: math.hypot(c[0] - best[0], c[1] - best[1])); neu.append(c2)
            print('    zweites GND-Via (%.3f, %.3f)' % c2)
json.dump(dict(vias=[dict(x=x, y=y, net='GND') for x, y in neu]), open(OUT, 'w')); print('GND-Vias in Luecken:', len(neu))
