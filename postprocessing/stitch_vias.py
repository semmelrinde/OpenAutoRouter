import json, sys, math
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union
from shapely.strtree import STRtree

B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); Z = json.load(open(sys.argv[3])); OUT = sys.argv[4]
a = dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x)
GRID = float(a.get('raster', '1.5')); EDGE = float(a.get('edge', '0.9')); DIST = float(a.get('dist', '0.2'))
VR = float(a.get('via', '0.45')) / 2; MINC = float(a.get('h2h', '0.2')) + float(a.get('drill', '0.2')) + 0.03

board = unary_union([Polygon(o) for o in B['outline']])
area = board.buffer(-(0.2 + VR + 0.1))
if 'tongue' in a:
    zx0, zx1, zy0 = [float(v) for v in a['tongue'].split(',')]
    area = area.difference(box(-50, zy0, 50, 200)).union(area.intersection(box(zx0 + 0.6, zy0, zx1 - 0.6, 200)))

foreign = []
for t in R['tracks']:
    if t['net'] == '/GND': continue
    for p0, p1, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
        foreign.append(LineString([p0, p1]).buffer(w / 2) if p0 != p1 else Point(p0).buffer(w / 2))
for v in R['vias']:
    if v['net'] != '/GND': foreign.append(Point(v['x'], v['y']).buffer(VR))
for p in B['pads']:
    if p['net'] == '/GND': continue
    for L, qs in p['polys'].items():
        for q in qs: foreign.append(Polygon(q))
for z in Z:
    if z['net'] != '/GND' and z.get('prio', 1) > 0: foreign.append(Polygon(z['pts']))

for p in B['pads']:
    if p['net'] == '/GND':
        for L, qs in p['polys'].items():
            for q in qs: foreign.append(Polygon(q))
T = STRtree(foreign)
vias = [(v['x'], v['y']) for v in R['vias']]


def ok(q):
    if not area.contains(Point(q)): return False
    c = Point(q).buffer(VR + DIST)
    for j in T.query(c):
        if c.intersects(foreign[int(j)]): return False
    return all(math.dist(q, v) >= MINC for v in vias)


cand = []

ring = board.buffer(-EDGE).exterior
n = int(ring.length / float(a.get('edge_raster', GRID)))
for i in range(n):
    p = ring.interpolate(i / n, normalized=True); cand.append((round(p.x, 3), round(p.y, 3)))
x0, y0, x1, y1 = board.bounds
y = y0 + GRID / 2
row = 0
while y < y1:
    x = x0 + GRID / 2 + (GRID / 2 if row % 2 else 0)
    while x < x1:
        cand.append((round(x, 3), round(y, 3))); x += GRID
    y += GRID * 0.866; row += 1
added = 0
for q in cand:
    if ok(q):
        R['vias'].append(dict(net='/GND', x=q[0], y=q[1])); vias.append(q); added += 1
json.dump(R, open(OUT, 'w'))
print('Stitching-Vias', added, 'from_', len(cand), 'Candidates')
