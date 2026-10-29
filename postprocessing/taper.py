import json, sys, math, collections
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
H = json.load(open(sys.argv[1])); X0, Y0, X1, Y1 = map(float, sys.argv[2:6]); NETS = [n for n in sys.argv[6].split(';') if n]
OUT = sys.argv[7]; A = dict(a.split('=', 1) for a in sys.argv[8:] if '=' in a)
FACTOR = float(A.get('factor', 4)); LMAX = float(A.get('lmax', 1.0)); CL = 0.10; MARG = 0.005; EDGE = 0.2
def inw(p): return X0 <= p[0] <= X1 and Y0 <= p[1] <= Y1
LAG = ('F', 'In2', 'In5', 'B')
obstacles = {L: [] for L in LAG}
for t in H['tracks']:
    if t['layer'] in obstacles: obstacles[t['layer']].append((t['net'], (LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])).buffer(t['w'] / 2, 12), CL))
for p in H['pads']:
    if p['layer'] in obstacles: obstacles[p['layer']].append((p['net'], Polygon(p['poly']), CL))
for v in H['vias']:
    for L in LAG: obstacles[L].append((v['net'], Point(v['x'], v['y']).buffer(v['d'] / 2, 16), CL))
for k in H.get('keepouts', []):
    if k.get('tracks'):
        for L in k['layers']:
            if L in obstacles: obstacles[L].append(('#KEEPOUT', Polygon(k['poly']), 0.0))
for e in H.get('edges', []):
    for L in LAG: obstacles[L].append(('#EDGE', LineString(list(e) + [e[0]]), EDGE))
trees = {L: STRtree([o[1] for o in obstacles[L]]) for L in LAG}
def vacant(g, L, net):
    for i in trees[L].query(g.buffer(0.3)):
        n, o, d = obstacles[L][int(i)]
        if n != net and g.distance(o) < d + MARG: return False
    return True
shapes = []; stat = collections.Counter()
for net in NETS:
    own_v = [Point(v['x'], v['y']).buffer(v['d'] / 2 - 0.01) for v in H['vias'] if v['net'] == net]
    for L in LAG:
        own_p = [Polygon(p['poly']) for p in H['pads'] if p['net'] == net and p['layer'] == L]
        segs = [t for t in H['tracks'] if t['net'] == net and t['layer'] == L and not t.get('arc') and t['a'] != t['b']]
        ends = collections.defaultdict(list)
        for t in segs:
            for e, o in ((t['a'], t['b']), (t['b'], t['a'])): ends[(round(e[0], 3), round(e[1], 3))].append((t, tuple(o)))
        for J, lst in ends.items():
            if len(lst) != 2 or not inw(J): continue
            (t1, o1), (t2, o2) = lst
            if abs(t1['w'] - t2['w']) < 0.02: continue
            P = Point(J)
            if any(g.contains(P) for g in own_v) or any(g.contains(P) for g in own_p): stat['in the pad/via'] += 1; continue
            (wb, ob), (wn, on) = sorted(((t1['w'], o1), (t2['w'], o2)), reverse=True)
            ln = math.dist(J, on)
            if ln < 1e-3: continue
            u = ((on[0] - J[0]) / ln, (on[1] - J[1]) / ln)
            l0 = min(FACTOR * (wb - wn), LMAX, 0.9 * ln)
            for f in (1.0, 0.75, 0.5, 0.3):
                l = l0 * f
                if l < 0.05: break
                Q = (J[0] + u[0] * l, J[1] + u[1] * l)
                g = unary_union([P.buffer(wb / 2, 24), Point(Q).buffer(wn / 2, 24)]).convex_hull
                if vacant(g, L, net):
                    shapes.append(dict(net=net, layer=L, pts=[[round(x, 4), round(y, 4)] for x, y in g.exterior.coords[:-1]]))
                    stat['Teardrop'] += 1; stat['Length full' if f == 1.0 else 'shortened'] += 1; break
            else: stat['no room'] += 1
json.dump(shapes, open(OUT, 'w'))
print(dict(stat))
