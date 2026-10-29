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
NMIN = int(opt.get('min', '2')); AMIN = float(opt.get('amin', '0.3')); SPLIT = float(opt.get('splitter', '3.0')); ONE = opt.get('one') == '1'
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = prep(board.buffer(-(BORDER + VR)))
hard = []
for t in C['tracks']:
    if t['net'] != 'GND': hard.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
for p in C['pads']:
    if p['net'] != 'GND': hard += [poly(q) for L, qs in p['polys'].items() for q in qs]
for v in C['vias']:
    if v['net'] != 'GND': hard.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
for f in C.get('shapes', []):
    if f['net'] != 'GND': hard += [poly(q) for q in f['polys']]
hard = [g for g in hard if not g.is_empty]
tree = STRtree(hard)
bore = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0]
blocked = prep(unary_union([Polygon(s['pts']).buffer(VR + 0.05) for s in S if s['fill'] or s['vias']]))
gnd_vias = [(v['x'], v['y']) for v in C['vias'] if v['net'] == 'GND'] + [(p['x'], p['y']) for p in C['pads'] if p['net'] == 'GND' and p['drill'] > 0]
def ok(x, y, new):
    pt = Point(x, y)
    if not inner.contains(pt) or blocked.contains(pt): return False
    for j in tree.query(pt.buffer(VR + DRC)):
        if hard[int(j)].distance(pt) < VR + DRC: return False
    for bx, by, br in bore + [(a, b, VDR / 2) for a, b in new]:
        if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
    return True
area_board = board.area; new = []; report = []
for z in C['zones']:
    if z['net'] != 'GND' or z['layer'] == 'In1': continue
    for q in z['polys']:
        P = poly(q)
        if P.is_empty or P.area < AMIN or P.area > 0.6 * area_board: continue
        Pp = prep(P.buffer(0.01))
        da = [v for v in gnd_vias + new if Pp.contains(Point(v))]
        if len(da) >= NMIN: continue
        Pin = prep(P.buffer(-VR - 0.02))
        bx = P.bounds
        cand = [(round(x, 3), round(y, 3)) for x in np.arange(bx[0], bx[2], 0.05) for y in np.arange(bx[1], bx[3], 0.05) if Pin.contains(Point(x, y))]
        cand = [c for c in cand if ok(c[0], c[1], new)]
        vs = list(da); assigned = []; new0 = len(new)
        while len(vs) < NMIN and cand:
            if vs: best = max(cand, key=lambda c: min(math.hypot(c[0] - a, c[1] - b) for a, b in vs))
            else:
                best = max(cand, key=lambda c: P.exterior.distance(Point(c)))
            if vs and min(math.hypot(best[0] - a, best[1] - b) for a, b in vs) < VD + H2H: break
            vs.append(best); assigned.append(best); new.append(best)
            cand = [c for c in cand if ok(c[0], c[1], new)]
        if P.area < SPLIT and len(vs) < NMIN and not ONE:
            del new[new0:]; assigned = []
        report.append(dict(layer_name=z['layer'], surface=round(P.area, 2), x=round(P.centroid.x, 2), y=round(P.centroid.y, 2), before=len(da), new=len(assigned)))
json.dump(dict(vias=[dict(x=x, y=y, net='GND') for x, y in new]), open(OUT, 'w'))
for b in sorted(report, key=lambda b: -b['surface']): print(b)
print('new GND vias:', len(new))
