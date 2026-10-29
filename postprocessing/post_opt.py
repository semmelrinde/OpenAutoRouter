import json, sys, math, heapq, collections
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely import contains_xy, distance as sdist
H = json.load(open(sys.argv[1])); NET = sys.argv[2]; RX0, RY0, RX1, RY1 = map(float, sys.argv[3:7])
A = dict(a.split('=', 1) for a in sys.argv[7:-1]); OUT = sys.argv[-1]
_LO = A.get('cu_layers', 'track_layers')
_ORD = lambda L: (0 if L == 'F' else 2 if L == 'B' else 1, int(L[2:]) if L.startswith('In') and L[2:].isdigit() else 0)
if _LO == 'track_layers': LAG = sorted({t['layer'] for t in H['tracks']}, key=_ORD)
elif _LO == 'all_items': LAG = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']}, key=_ORD)
else: LAG = _LO.split(',')
VIAK = float(A.get('via', 1.0)); BEND = float(A.get('bend', 0.3))
BORDER = float(A.get('border', 1.5)); G = 0.025
CL_T = float(A.get('drc', 0.09)); CL_P = float(A.get('drcpad', 0.10)); CL_BGA = float(A.get('drcbga', CL_P)); HOLE = float(A.get('hole_size', 0.25))
HOLE_VIA = float(A.get('hole_via', HOLE))
MARG = 0.02; VM = 0.005; HH = 0.2; EDGE = float(A.get('edge', 0.2))
RIP = box(RX0, RY0, RX1, RY1)
def circle(c, r, q=16):
    return c.buffer(r / math.cos(math.pi / (4 * q)), q)
def inr(p): return RX0 <= p[0] <= RX1 and RY0 <= p[1] <= RY1
own_t = [t for t in H['tracks'] if t['net'] == NET]; own_v = [v for v in H['vias'] if v['net'] == NET]; own_p = [p for p in H['pads'] if p['net'] == NET]
rip_t = [t for t in own_t if inr(t['a']) and inr(t['b']) and t['layer'] in LAG]

ALL_L = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']} | set(LAG))
def span(v): return set(v.get('layers') or ALL_L)
def _hangs_foreign_layer(v):
    c = Point(v['x'], v['y']); r = v['d'] / 2; sp = span(v)
    if any(t['layer'] not in LAG and t['layer'] in sp and t not in rip_t and LineString([t['a'], t['b']]).distance(c) <= r + t['w'] / 2
           for t in own_t if t['a'] != t['b']): return True
    return any(p['layer'] not in LAG and p['layer'] in sp and Polygon(p['poly']).distance(c) <= r for p in own_p)
rip_v = [v for v in own_v if inr((v['x'], v['y'])) and not _hangs_foreign_layer(v)]
RIPID = {t['uuid'] for t in rip_t} | {v['uuid'] for v in rip_v}

def _mode(pairs):
    z = collections.Counter()
    for k, g in pairs: z[k] += g
    return max(sorted(z), key=lambda k: z[k]) if z else None
if A.get('w', 'auto') == 'auto':
    W = _mode([(round(t['w'], 4), math.dist(t['a'], t['b']) + 1e-6) for t in (rip_t or own_t)]) or 0.1
else: W = float(A['w'])
if A.get('via_size', 'auto') == 'auto':
    VD, VDR = _mode([((round(v['d'], 4), round(v['drill'], 4)), 1) for v in own_v]) or _mode([((round(v['d'], 4), round(v['drill'], 4)), 1) for v in H['vias']]) or (0.45, 0.2)
else: VD, VDR = [float(x) for x in A['via_size'].split('/')]
old_len = sum(math.dist(t['a'], t['b']) for t in rip_t)

pieces = []
for t in own_t:
    if t['uuid'] in RIPID: continue
    g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
    pieces.append(({t['layer']}, g.buffer(t['w'] / 2, 8), ('t', g, t['w'])))
for v in own_v:
    if v['uuid'] in RIPID: continue

    pieces.append((span(v), Point(v['x'], v['y']).buffer(v['d'] / 2, 16), ('v', Point(v['x'], v['y']), v['d'])))
for p in own_p:
    pieces.append(({p['layer']}, Polygon(p['poly']), ('p', Polygon(p['poly']), 0)))
par = list(range(len(pieces)))
def f(i):
    while par[i] != i: par[i] = par[par[i]]; i = par[i]
    return i
for i in range(len(pieces)):
    for j in range(i + 1, len(pieces)):
        if (pieces[i][0] & pieces[j][0]) and pieces[i][1].distance(pieces[j][1]) < 1e-4: par[f(i)] = f(j)
_pad0 = {}
for i, p in enumerate(own_p):
    k = (p.get('ref'), p.get('num')); i += len(pieces) - len(own_p)
    if k[0] is None: continue
    if k in _pad0: par[f(i)] = f(_pad0[k])
    else: _pad0[k] = i
comp = collections.defaultdict(list)
for i in range(len(pieces)): comp[f(i)].append(i)
comps = list(comp.values())

near = [c for c in comps if any(pieces[i][1].distance(RIP) < 0.3 for i in c)]
if len(near) < 2:
    print('nothing to connect', len(near), 'Components'); json.dump(dict(net=NET, tracks=[], vias=[], ripped=sorted(RIPID), old_len=old_len), open(OUT, 'w')); sys.exit(0)

bb = unary_union([RIP] + [pieces[i][1] for c in near for i in c]).bounds
X0 = math.floor((max(bb[0], RX0) - BORDER) / G) * G; Y0 = math.floor((max(bb[1], RY0) - BORDER) / G) * G
X1 = math.ceil((min(bb[2], RX1) + BORDER) / G) * G; Y1 = math.ceil((min(bb[3], RY1) + BORDER) / G) * G
nx = int(round((X1 - X0) / G)) + 1; ny = int(round((Y1 - Y0) / G)) + 1
xs = X0 + np.arange(nx) * G; ys = Y0 + np.arange(ny) * G; XX, YY = np.meshgrid(xs, ys, indexing='ij')

COPPER_L = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']} | set(LAG), key=_ORD)
obs_t = {L: [] for L in LAG}; obs_v = {L: [] for L in COPPER_L}; holes = []; hole_t = []
for t in H['tracks']:
    if t['net'] == NET: continue
    g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
    if t['layer'] in obs_t: obs_t[t['layer']].append(g.buffer(t['w'] / 2 + CL_T + W / 2 + MARG, 8))
    obs_v[t['layer']].append(g.buffer(t['w'] / 2 + CL_T + VD / 2 + VM, 8))
for p in H['pads']:
    if p['net'] == NET: continue
    cp = CL_BGA if any(k in (p.get('fp') or '').upper() for k in ('BGA', 'WLCSP')) else CL_P
    g = Polygon(p['poly'])
    if p['layer'] in obs_t: obs_t[p['layer']].append(g.buffer(cp + W / 2 + MARG, 8))
    obs_v[p['layer']].append(g.buffer(cp + VD / 2 + VM, 8))
HOLES = []
for h in H.get('holes', []):
    if h.get('net') and h['net'] == NET: continue
    c = Point(h['x'], h['y']); holes.append(circle(c, h['d'] / 2 + HH + VDR / 2 + VM)); HOLES.append((c, h['d'] / 2, HOLE))
    hole_t.append(circle(c, h['d'] / 2 + HOLE + W / 2 + MARG))
    for L in LAG: obs_t[L].append(hole_t[-1])
    for L in COPPER_L: obs_v[L].append(circle(c, h['d'] / 2 + HOLE + VD / 2 + VM))
for v in H['vias']:
    if v['uuid'] in RIPID: continue
    c = Point(v['x'], v['y']); holes.append(circle(c, v['drill'] / 2 + HH + VDR / 2 + VM))
    if v['net'] == NET: continue
    HOLES.append((c, v['drill'] / 2, HOLE_VIA)); hole_t.append(circle(c, v['drill'] / 2 + HOLE_VIA + W / 2 + MARG))
    for L in LAG: obs_t[L].append(circle(c, max(v['d'] / 2 + CL_T, v['drill'] / 2 + HOLE_VIA) + W / 2 + MARG))
    for L in COPPER_L: obs_v[L].append(circle(c, max(v['d'] / 2 + CL_T, v['drill'] / 2 + HOLE_VIA) + VD / 2 + VM))
for k in H['keepouts']:
    g = Polygon(k['poly'])
    for L in k['layers']:
        if k['tracks'] and L in obs_t: obs_t[L].append(g.buffer(W / 2 + MARG, 4))
        if k['vias'] and L in obs_v: obs_v[L].append(g.buffer(VD / 2 + VM, 4))
for e in H.get('edges', []):
    g = LineString(list(e) + [e[0]]) if len(e) > 2 else LineString(e)
    for L in LAG: obs_t[L].append(g.buffer(EDGE + W / 2 + MARG, 4))
    for L in COPPER_L: obs_v[L].append(g.buffer(EDGE + VD / 2 + VM, 4))
free_t = {}; free_v = np.ones((nx, ny), bool)
if holes: free_v &= ~contains_xy(unary_union(holes), XX, YY)
for L in LAG:
    free_t[L] = ~contains_xy(unary_union(obs_t[L]), XX, YY) if obs_t[L] else np.ones((nx, ny), bool)
for L in COPPER_L:
    if obs_v[L]: free_v &= ~contains_xy(unary_union(obs_v[L]), XX, YY)
in_hole_spacing = contains_xy(unary_union(hole_t), XX, YY) if hole_t else np.zeros((nx, ny), bool)

cost_mul = {L: np.ones((nx, ny)) for L in LAG}
for spec in [s for s in A.get('expensive', '').split(';') if s]:
    bx, L, m = spec.split(':'); bx = list(map(float, bx.split(',')))
    if L in cost_mul: cost_mul[L][(XX >= bx[0]) & (XX <= bx[2]) & (YY >= bx[1]) & (YY <= bx[3])] = float(m)
LI = {L: i for i, L in enumerate(LAG)}
def comp_nodes(c):
    S = set()
    for i in c:
        Ls, geo, (kind, g, w) = pieces[i]
        if kind == 't': inner = g.buffer(max(w / 2 - 0.02, 0.005), 8)
        elif kind == 'v': inner = g.buffer(w / 2 - 0.03, 16)
        else: inner = g.buffer(-0.02) if g.buffer(-0.02).area > 0 else g
        b0 = inner.bounds
        i0 = max(0, int(math.floor((b0[0] - X0) / G))); i1 = min(nx - 1, int(math.ceil((b0[2] - X0) / G)))
        j0 = max(0, int(math.floor((b0[1] - Y0) / G))); j1 = min(ny - 1, int(math.ceil((b0[3] - Y0) / G)))
        if i0 > i1 or j0 > j1: continue
        sub = contains_xy(inner, XX[i0:i1 + 1, j0:j1 + 1], YY[i0:i1 + 1, j0:j1 + 1])
        for a, b_ in zip(*np.nonzero(sub)):
            for L in [L_ for L_ in Ls if L_ in LI]:
                S.add((i0 + a, j0 + b_, LI[L]))
    return S


CN = [{(i, j, l) for (i, j, l) in comp_nodes(c) if not in_hole_spacing[i, j]} for c in near]
if any(not s for s in CN): print('Component without grid node'); sys.exit(2)
for s in CN:
    for (i, j, l) in s: free_t[LAG[l]][i, j] = True
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
tree = set(CN[0]); rest = list(range(1, len(CN))); paths = []
total = 0.0
while rest:
    goal_of = {}
    for k in rest:
        for q in CN[k]: goal_of[q] = k
    dist = {}; prev = {}; pq = []
    for (i, j, l) in tree:
        st = (i, j, l, -1); dist[st] = 0.0; pq.append((0.0, st))
    heapq.heapify(pq); goal = None
    while pq:
        d, st = heapq.heappop(pq)
        if d > dist.get(st, 1e9) + 1e-12: continue
        i, j, l, dr = st
        if (i, j, l) in goal_of: goal = st; break
        for k, (di, dj) in enumerate(DIRS):
            a, c = i + di, j + dj
            if not (0 <= a < nx and 0 <= c < ny) or not free_t[LAG[l]][a, c]: continue
            pen = 0.0
            if dr >= 0:
                turn = min((k - dr) % 8, (dr - k) % 8)
                if turn > 1: continue
                pen = BEND if turn else 0.0
            nd = d + G * (1.41421356 if di and dj else 1) * 0.5 * (cost_mul[LAG[l]][i, j] + cost_mul[LAG[l]][a, c]) + pen
            ns = (a, c, l, k)
            if nd < dist.get(ns, 1e9) - 1e-12: dist[ns] = nd; prev[ns] = st; heapq.heappush(pq, (nd, ns))
        if free_v[i, j]:
            for l2 in range(len(LAG)):
                if l2 == l or not free_t[LAG[l2]][i, j]: continue
                ns = (i, j, l2, -1); nd = d + VIAK
                if nd < dist.get(ns, 1e9) - 1e-12: dist[ns] = nd; prev[ns] = st; heapq.heappush(pq, (nd, ns))
    if goal is None: print('no path to components', rest); sys.exit(1)
    path = [goal]
    while path[-1] in prev: path.append(prev[path[-1]])
    path.reverse(); paths.append(path)
    k = goal_of[goal[:3]]; rest.remove(k); tree |= CN[k]
    for (i, j, l, _) in path: tree.add((i, j, l))
segs, vias = [], []
for path in paths:
    cur = [path[0]]
    for p in path[1:]:
        if p[2] != cur[-1][2]:
            vias.append([round(X0 + p[0] * G, 4), round(Y0 + p[1] * G, 4)])
            if len(cur) > 1: segs.append((LAG[cur[0][2]], cur))
            cur = [p]; continue
        cur.append(p)
    if len(cur) > 1: segs.append((LAG[cur[0][2]], cur))
out = []; new_len = 0.0
for L, pts in segs:
    poly = [pts[0]]
    for a_, b_, c_ in zip(pts, pts[1:], pts[2:]):
        if (b_[0] - a_[0], b_[1] - a_[1]) != (c_[0] - b_[0], c_[1] - b_[1]): poly.append(b_)
    poly.append(pts[-1])
    P = [[round(X0 + q[0] * G, 4), round(Y0 + q[1] * G, 4)] for q in poly]
    new_len += sum(math.dist(a, b) for a, b in zip(P, P[1:]))
    out.append(dict(layer=L, pts=P, w=W))

for tr in out:
    for a_, b_ in zip(tr['pts'], tr['pts'][1:]):
        g = LineString([a_, b_]) if a_ != b_ else Point(a_)
        for c, r, lo in HOLES:
            if g.distance(c) < r + lo + W / 2 - 1e-6:
                print('Drill spacing violated: %.4f < %.4f at %s' % (g.distance(c) - r - W / 2, lo, a_)); sys.exit(3)
for p in vias:
    for c, r, lo in HOLES:
        if Point(p).distance(c) < r + lo + VD / 2 - 1e-6: print('Drill spacing via violated at', p); sys.exit(3)
json.dump(dict(net=NET, tracks=out, vias=vias, ripped=sorted(RIPID), old_len=old_len, new_len=new_len, old_vias=len(rip_v), w=W, via=[VD, VDR],
               cu_layers=LAG), open(OUT, 'w'))
print('%s: old %.2f mm / %d vias  ->  new %.2f mm / %d vias  (components %d, width %.4g, via %.3g/%.3g, layers %s)' % (
    NET.split('/')[-1], old_len, len(rip_v), new_len, len(vias), len(near), W, VD, VDR, ','.join(LAG)))
