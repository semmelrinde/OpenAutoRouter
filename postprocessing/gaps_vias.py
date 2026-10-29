import json, sys, math
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.strtree import STRtree
C = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
VD, VDR = [float(q) for q in opt.get('via', '0.5/0.2').split('/')]; VR = VD / 2
DRC = float(opt.get('drc', '0.128')) + 0.01; H2H = float(opt.get('h2h', '0.25')) + 0.01; BORDER = float(opt.get('border', '0.5')) + 0.01
CLR = float(opt.get('clr', '0.15')); TOL = float(opt.get('tol', '0.01'))
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-0.5); innerv = prep(board.buffer(-(BORDER + VR)))
hard = []
for t in C['tracks']:
    if t['net'] != 'GND': hard.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
for p in C['pads']:
    if p['net'] != 'GND': hard += [poly(q) for L, qs in p['polys'].items() for q in qs]
for v in C['vias']:
    if v['net'] != 'GND': hard.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
for f in C.get('shapes', []):
    if f['net'] != 'GND': hard += [poly(q) for q in f['polys']]
hard = [g for g in hard if not g.is_empty]; tree = STRtree(hard)
bore = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0]
blocked = prep(unary_union([Polygon(s['pts']).buffer(VR + 0.05) for s in S if s['fill'] or s['vias']]))
HOLE = float(opt.get('hole_size', '0.255')) + 0.01
bore_pads = [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0 and p['net'] != 'GND']
def ok(x, y, new):
    pt = Point(x, y)
    if not innerv.contains(pt) or blocked.contains(pt): return False
    for j in tree.query(pt.buffer(VR + DRC)):
        if hard[int(j)].distance(pt) < VR + DRC: return False
    for bx, by, br in bore + [(a, b, VDR / 2) for a, b in new]:
        if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
    for bx, by, br in bore_pads:
        if math.hypot(bx - x, by - y) < br + VR + HOLE: return False
    return True
new = []
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
    for f in C.get('shapes', []):
        if f['layer'] == L: cu += [poly(q) for q in f['polys']]
    white = inner.difference(unary_union(cu).buffer(CLR + TOL)).difference(unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['cu_layers'] and s['fill']]).buffer(CLR))
    for g in getattr(white, 'geoms', [white]):
        if g.area < 0.01: continue
        if any(g.distance(Point(a, b)) < VR + 0.2 for a, b in new): continue
        bx = g.buffer(VR).bounds
        cand = [(round(x, 3), round(y, 3)) for x in np.arange(bx[0], bx[2], 0.025) for y in np.arange(bx[1], bx[3], 0.025)
                if g.buffer(VR - 0.05).contains(Point(x, y))]
        cand = [c for c in cand if ok(c[0], c[1], new)]
        if not cand: continue
        best = min(cand, key=lambda c: g.centroid.distance(Point(c)))
        new.append(best); print('%-3s gap %.3f mm² at (%.2f, %.2f) -> GND via (%.3f, %.3f)' % (L, g.area, g.centroid.x, g.centroid.y, best[0], best[1]))
        second = [c for c in cand if math.hypot(c[0] - best[0], c[1] - best[1]) >= VD + H2H and ok(c[0], c[1], new)]
        if second:
            c2 = max(second, key=lambda c: math.hypot(c[0] - best[0], c[1] - best[1])); new.append(c2)
            print('    second GND via (%.3f, %.3f)' % c2)
json.dump(dict(vias=[dict(x=x, y=y, net='GND') for x, y in new]), open(OUT, 'w')); print('GND vias in gaps:', len(new))
