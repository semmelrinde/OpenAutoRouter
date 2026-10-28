""
import json, sys, math, heapq
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely import prepared
H = json.load(open(sys.argv[1])); NET = sys.argv[2]; X0, Y0, X1, Y1 = map(float, sys.argv[3:7])
A = dict(a.split('=', 1) for a in sys.argv[7:-1]); OUT = sys.argv[-1]
LAG = A.get('lagen', 'B,In5,In2').split(','); VLAG = LAG + [L for L in A.get('vialagen', 'F').split(',') if L and L not in LAG]; IGN = set(A.get('ignore', '').split(','))
VIAK = float(A.get('via', 1.2)); KNICK = float(A.get('knick', 0.25)); G = 0.025
W = float(A.get('w', 0.1)); CL_T = float(A.get('clt', 0.09)); CL_P = float(A.get('clp', 0.10)); MARG = 0.02; VM = float(A.get('viamarge', 0.005))
VD = float(A.get('vd', 0.45)); VDR = float(A.get('vdr', 0.2)); HH = float(A.get('hh', 0.2)); HCL = 0.2
def own(n): return n == NET or n.endswith('/' + NET.split('/')[-1]) and NET.split('/')[-1] in n
obs_t = {L: [] for L in VLAG}; obs_v = {L: [] for L in VLAG}; holes = []
for t in H['tracks']:
    if own(t['net']) and t['uuid'] not in IGN: continue
    if t['uuid'] in IGN: continue
    if t['layer'] not in VLAG: continue
    g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
    obs_t[t['layer']].append(g.buffer(t['w'] / 2 + CL_T + W / 2 + MARG, 8))
    obs_v[t['layer']].append(g.buffer(t['w'] / 2 + CL_T + VD / 2 + VM, 8))
for p in H['pads']:
    if own(p['net']) and A.get('viainpad') == '0' and p['layer'] in VLAG:
        obs_v[p['layer']].append(Polygon(p['poly']).buffer(VD / 2 + VM, 8))
    if own(p['net']) or p['layer'] not in VLAG: continue
    g = Polygon(p['poly'])
    obs_t[p['layer']].append(g.buffer(CL_P + W / 2 + MARG, 8)); obs_v[p['layer']].append(g.buffer(CL_P + VD / 2 + VM, 8))
for v in H['vias']:
    if v.get('uuid') in IGN: continue
    c = Point(v['x'], v['y'])
    holes.append(c.buffer(v['drill'] / 2 + HH + VDR / 2 + VM, 16))
    if own(v['net']): continue
    for L in VLAG:
        obs_t[L].append(c.buffer(v['d'] / 2 + CL_T + W / 2 + MARG, 16)); obs_v[L].append(c.buffer(v['d'] / 2 + CL_T + VD / 2 + VM, 16))
for k in H['keepouts']:
    g = Polygon(k['poly'])
    for L in k['layers']:
        if L not in VLAG: continue
        if k['tracks']: obs_t[L].append(g.buffer(W / 2 + MARG, 4))
        if k['vias']: obs_v[L].append(g.buffer(VD / 2 + VM, 4))
if A.get('zonen') == '1':
    FREI = set(A.get('zonen_frei', 'GND').split(','))
    for z in H.get('zonen', []):
        if own(z['net']) or z['net'] in FREI or z['layer'] not in VLAG: continue
        g = Polygon(z['outer'], z.get('holes', []))
        obs_t[z['layer']].append(g.buffer(CL_T + W / 2 + MARG, 8))
        if A.get('zonen_via') == '1': obs_v[z['layer']].append(g.buffer(CL_T + VD / 2 + VM, 8))
if float(A.get('rand', 0)) > 0:
    RD = float(A['rand'])
    for e in H.get('edges', []):
        g = LineString(e + e[:1])
        for L in VLAG: obs_t[L].append(g.buffer(RD + W / 2 + MARG, 8)); obs_v[L].append(g.buffer(RD + VD / 2 + VM, 8))
nx = int(round((X1 - X0) / G)) + 1; ny = int(round((Y1 - Y0) / G)) + 1
xs = X0 + np.arange(nx) * G; ys = Y0 + np.arange(ny) * G
from shapely import contains_xy
XX, YY = np.meshgrid(xs, ys, indexing='ij')
free_t = {}; free_v = np.ones((nx, ny), bool)
hu = unary_union(holes) if holes else None
if hu is not None: free_v &= ~contains_xy(hu, XX, YY)
for L in VLAG:
    if L in LAG: u = unary_union(obs_t[L]); free_t[L] = ~contains_xy(u, XX, YY)
    uv = unary_union(obs_v[L]); free_v &= ~contains_xy(uv, XX, YY)
def node(x, y): return int(round((x - X0) / G)), int(round((y - Y0) / G))
sx, sy, sL = A['start'].split(','); s = node(float(sx), float(sy))
zz = A['ziel'].split(','); zL = zz[-1]
if len(zz) == 5:
    zs = LineString([(float(zz[0]), float(zz[1])), (float(zz[2]), float(zz[3]))])
    ZSET = {(i, j) for i in range(nx) for j in range(ny) if zs.distance(Point(xs[i], ys[j])) <= 0.03}
    z = min(ZSET, key=lambda q: Point(xs[q[0]], ys[q[1]]).distance(Point(xs[s[0]], ys[s[1]])))
else:
    z = node(float(zz[0]), float(zz[1])); zL = zz[2] if len(zz) > 2 else '*'; ZSET = {z}
for L in (LAG if sL == '*' else [sL]): free_t[L][s] = True
for q in ZSET:
    for L in (LAG if zL == '*' else [zL]): free_t[L][q] = True
print('frei', {L: int(free_t[L].sum()) for L in LAG}, 'via-frei', int(free_v.sum()))
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
LI = {L: i for i, L in enumerate(LAG)}
def h(i, j): return 0.0 if len(ZSET) > 1 else ((lambda dx, dy: (max(dx, dy) - min(dx, dy) + math.sqrt(2) * min(dx, dy)) * G)(abs(i - z[0]), abs(j - z[1])))
starts = [(s[0], s[1], LI[L], -1) for L in (LAG if sL == '*' else [sL])]; start = starts[0]; dist = {q: 0.0 for q in starts}; prev = {}; pq = [(h(*s), 0.0, q) for q in starts]; goal = None
while pq:
    f, d, st = heapq.heappop(pq)
    if d > dist.get(st, 1e9) + 1e-12: continue
    i, j, l, dr = st
    if (i, j) in ZSET and (zL == '*' or LAG[l] == zL): goal = st; break
    for k, (di, dj) in enumerate(DIRS):
        a, c = i + di, j + dj
        if not (0 <= a < nx and 0 <= c < ny) or not free_t[LAG[l]][a, c]: continue
        if dr >= 0:
            turn = min((k - dr) % 8, (dr - k) % 8)
            if turn > 1: continue
            pen = KNICK if turn else 0.0
        else: pen = 0.0
        nd = d + G * (math.sqrt(2) if di and dj else 1) + pen
        ns = (a, c, l, k)
        if nd < dist.get(ns, 1e9) - 1e-12: dist[ns] = nd; prev[ns] = st; heapq.heappush(pq, (nd + h(a, c), nd, ns))
    if free_v[i, j]:
        for l2 in range(len(LAG)):
            if l2 == l or not free_t[LAG[l2]][i, j]: continue
            ns = (i, j, l2, -1); nd = d + VIAK
            if nd < dist.get(ns, 1e9) - 1e-12: dist[ns] = nd; prev[ns] = st; heapq.heappush(pq, (nd + h(i, j), nd, ns))
if goal is None:
    import collections; cnt = collections.Counter(LAG[k[2]] for k in dist); print("erreicht", cnt, "vias", sum(1 for k in dist if k[3] == -1 and k != start))
    reach = np.zeros((nx, ny), bool)
    for k in dist: reach[k[0], k[1]] = True
    np.save("reach.npy", reach); np.save("fv.npy", free_v); [np.save("ft_%s.npy" % L, free_t[L]) for L in LAG]
    print("kein Weg"); sys.exit(1)
path = [goal]
while path[-1] in prev: path.append(prev[path[-1]])
path.reverse()
segs, vias = [], []
cur = [path[0]]
for p in path[1:]:
    if p[2] != cur[-1][2]:
        vias.append([round(X0 + p[0] * G, 4), round(Y0 + p[1] * G, 4)])
        if len(cur) > 1: segs.append((LAG[cur[0][2]], cur))
        cur = [p]; continue
    cur.append(p)
if len(cur) > 1: segs.append((LAG[cur[0][2]], cur))
out = []
for L, pts in segs:
    poly = [pts[0]]
    for a_, b_, c_ in zip(pts, pts[1:], pts[2:]):
        if (b_[0] - a_[0], b_[1] - a_[1]) != (c_[0] - b_[0], c_[1] - b_[1]): poly.append(b_)
    poly.append(pts[-1])
    out.append(dict(layer=L, pts=[[round(X0 + q[0] * G, 4), round(Y0 + q[1] * G, 4)] for q in poly]))
json.dump(dict(net=NET, tracks=out, vias=vias), open(OUT, 'w'))
print('Laenge %.2f mm' % dist[goal], 'Vias', len(vias)); [print(' ', t['layer'], t['pts']) for t in out]; print(' vias', vias)
