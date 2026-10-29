import json, sys, math, collections, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shapely.geometry import LineString, Point, Polygon, box
from shapely.strtree import STRtree
H = json.load(open(sys.argv[1])); X0, Y0, X1, Y1 = map(float, sys.argv[2:6]); NETS = [n for n in sys.argv[6].split(';') if n]
OUT = sys.argv[7]; A = dict(a.split('=', 1) for a in sys.argv[8:] if '=' in a)
BLOCKED = {}; FIX = set()
if A.get('auto_keepout', '1') == '1' or A.get('keepout_list'):
    from postprocessing.keepout_nets import keepout_nets, keepout_extent
    auto = A.get('auto_keepout', '1') == '1'
    BLOCKED = keepout_nets(pcb=A.get('pcb') if auto else None, pro=A.get('pro') if auto else None, dru=A.get('dru') if auto else None,
                          netclasses=A.get('netclasses') if auto else None, keepout_list=A.get('keepout_list'), net_list={t['net'] for t in H['tracks']},
                          pads=H.get('pads') if auto else None, net_ties=H.get('net_ties') if auto else None,
                          tracks=H['tracks'] if auto else None, names=auto)
    if auto:
        _U = keepout_extent(H, pcb=A.get('pcb'), pro=A.get('pro'), dru=A.get('dru'), netclasses=A.get('netclasses'), keepout_list=A.get('keepout_list'))
        FIX = set(_U['track_list'])
        for _n in _U.get('without_path', []): BLOCKED.setdefault(_n, _U['net_list'].get(_n, 'Kelvin without path'))
    if A.get('fix'): FIX |= set(json.load(open(A['fix'])))
    if any(n in BLOCKED for n in NETS):
        print('locked (unchanged):', ', '.join('%s [%s]' % (n, BLOCKED[n]) for n in NETS if n in BLOCKED))
    NETS = [n for n in NETS if n not in BLOCKED]
LAG = A.get('cu_layers', 'all_items').split(',')
if LAG == ['all_items']:
    LAG = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']})
MODE = A.get('mode', 'entire')
DEAD = A.get('dead', '1') == '1'


DEAD_OFF = set(json.load(open(A['without_dead']))) if A.get('without_dead') else set()
CL_T, CL_P, CL_V, EDGE, MARG = 0.09, 0.10, 0.10, 0.2, 0.01
HOLE = 0.25
if 'design_rules' in A:
    rg = json.load(open(A['design_rules'])).get('router', {})
    CL_T = float(rg.get('drc', CL_T))
    CL_P = float(rg.get('drcpad', CL_P))
    CL_V = float(rg.get('drcvia', rg.get('drc', CL_V)))
    EDGE = float(rg.get('edge', EDGE)); HOLE = float(rg.get('hole_clr', HOLE))
if 'drc' in A: CL_T = CL_P = CL_V = float(A['drc'])
if 'drcpad' in A: CL_P = float(A['drcpad'])
if 'drcvia' in A: CL_V = float(A['drcvia'])
if 'border' in A: EDGE = float(A['border'])
if 'hole_size' in A: HOLE = float(A['hole_size'])
PAD_CENTER = A.get('pad_center', '1') == '1'
MIDDLE = A.get('middle', '1') == '1'
HMAX_NL = {}
if A.get('hmax_layers') and float(A.get('hmax_plane', '0.3')) > 0:
    HMAX_NL = {(n, L): float(A.get('hmax_plane', '0.3')) for n, Ls in json.load(open(A['hmax_layers'])).items() for L in Ls}
HMAX = float(A.get('hmax', '0'))

BUDGET = json.load(open(A['budget'])) if A.get('budget') else {}
WORK = box(X0, Y0, X1, Y1)
def inw(p): return X0 <= p[0] <= X1 and Y0 <= p[1] <= Y1
def R(p): return (round(p[0], 4), round(p[1], 4))

obstacles = {L: [] for L in LAG}
for t in H['tracks']:
    if t['layer'] in obstacles:
        g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
        obstacles[t['layer']].append((t['net'], g.buffer(t['w'] / 2, 8), CL_T))
for p in H['pads']:
    if p['layer'] in obstacles: obstacles[p['layer']].append((p['net'], Polygon(p['poly']), CL_P))
for v in H['vias']:
    c = Point(v['x'], v['y']).buffer(v['d'] / 2, 16)
    for L in LAG: obstacles[L].append((v['net'], c, CL_V))

for h in H.get('holes', []):
    c = Point(h['x'], h['y']).buffer(h['d'] / 2, 16)
    for L in LAG: obstacles[L].append((h['net'] or '#HOLE', c, HOLE))
for k in H.get('keepouts', []):
    if k.get('tracks'):
        for L in k['layers']:
            if L in obstacles: obstacles[L].append(('#KEEPOUT', Polygon(k['poly']), 0.0))
for e in H.get('edges', []):
    g = Polygon(e) if len(e) > 2 else LineString(e)
    for L in LAG: obstacles[L].append(('#EDGE', g.boundary if g.geom_type == 'Polygon' else g, EDGE))

ZW = None
if A.get('pcb') and A.get('zone_list', '1') == '1':
    from common.copper_net import ZonesGuard, zones_fill
    FILL = zones_fill(open(A['pcb'], encoding='utf-8').read())
    for f in FILL:
        if f['net'] in BLOCKED and f['layer'] in obstacles: obstacles[f['layer']].append((f['net'], f['poly'], max(0.0, f['clr'] - MARG - 1e-4)))
    ZW = ZonesGuard(H, FILL, drc=CL_T)
trees = {L: STRtree([o[1] for o in obstacles[L]]) for L in LAG}
DMAX = max([o[2] for L in LAG for o in obstacles[L]] + [0.25])
NEW = collections.defaultdict(list); NEW_RASTER = collections.defaultdict(list)
def _cells(b, r=1.0):
    return [(i, j) for i in range(math.floor(b[0] / r), math.floor(b[2] / r) + 1) for j in range(math.floor(b[1] / r), math.floor(b[3] / r) + 1)]
def new_obstacle(net, L, a, b, w):
    g = LineString([a, b]) if tuple(a) != tuple(b) else Point(a); k = len(NEW[L]); NEW[L].append((net, g, w))
    for z in _cells(g.buffer(w / 2).bounds): NEW_RASTER[(L, z)].append(k)
def vacant(seg, w, L, net):
    for i in trees[L].query(seg.buffer(w / 2 + DMAX + MARG)):
        n, g, d = obstacles[L][int(i)]
        if n == net: continue
        if seg.distance(g) < w / 2 + d + MARG: return False
    for k in sorted({k for z in _cells(seg.buffer(w / 2 + CL_T + MARG).bounds) for k in NEW_RASTER.get((L, z), ())}):
        n, g, wn = NEW[L][k]
        if n != net and seg.distance(g) < (w + wn) / 2 + CL_T + MARG: return False
    return True
def oct_(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs(dx) < 1e-6 or abs(dy) < 1e-6 or abs(abs(dx) - abs(dy)) < 1e-6
def candidates(a, b):
    if oct_(a, b): return [[a, b]]
    dx, dy = b[0] - a[0], b[1] - a[1]; m = min(abs(dx), abs(dy)); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    d1 = (a[0] + sx * m, a[1] + sy * m)
    d2 = (b[0] - sx * m, b[1] - sy * m)
    return [[a, R(d1), b], [a, R(d2), b]]
def plen(pts): return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
def angle_ok(p_prev, p, p_next):
    if p_prev is None or p_next is None: return True
    v1 = (p_prev[0] - p[0], p_prev[1] - p[1]); v2 = (p_next[0] - p[0], p_next[1] - p[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-9 or l2 < 1e-9: return True
    return (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2) <= 1e-6
def right_angled(p_prev, p, p_next):
    if p_prev is None or p_next is None: return False
    v1 = (p_prev[0] - p[0], p_prev[1] - p[1]); v2 = (p_next[0] - p[0], p_next[1] - p[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-9 or l2 < 1e-9: return False
    return abs((v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)) <= math.sin(math.radians(0.5))
remove, add = [], []; stat = collections.Counter(); change = {}
booked = collections.defaultdict(float)
for net in NETS:
    for L in LAG:
        segs = [t for t in H['tracks'] if t['net'] == net and t['layer'] == L]


        arc_ends = {R(t[e]) for t in segs if t.get('arc') for e in ('a', 'b')}
        inner = [t for t in segs if inw(t['a']) and inw(t['b']) and not t.get('arc') and t.get('uuid') not in FIX
                 and not any(R(t[e]) in arc_ends for e in ('a', 'b'))]
        if not inner: continue
        w = max(t["w"] for t in inner); TOL = min(t["w"] for t in inner) / 2 - 0.005
        own_v = [(Point(v['x'], v['y']), v['d'] / 2) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])]
        own_p = [Polygon(p['poly']).buffer(1e-4) for p in H['pads'] if p['net'] == net and p['layer'] == L]
        own_pm = [(p['x'], p['y']) if 'x' in p else None for p in H['pads'] if p['net'] == net and p['layer'] == L]
        inner_ids = {id(t) for t in inner}
        outer = [LineString([t['a'], t['b']]).buffer(t['w'] / 2 + 1e-4) for t in segs if id(t) not in inner_ids]


        pw = collections.defaultdict(float)
        for t in segs:
            for e in ('a', 'b'): pw[R(t[e])] = max(pw[R(t[e])], t['w'])
        ends = [(tuple(t['a']), t['w']) for t in segs] + [(tuple(t['b']), t['w']) for t in segs] + [((v[0].x, v[0].y), None) for v in own_v]
        pieces = []; width = {}; joint = []
        piece_w = []; pw_seen = {}; dup_width = set(); edge_w = {}; dup_w = collections.defaultdict(set)
        for t in inner:
            a, b = tuple(t['a']), tuple(t['b']); ln = LineString([a, b]); cuts = []
            for q, wq in ends:
                dq = ln.distance(Point(q))
                if dq <= (TOL if wq is None else max(TOL, (wq + t['w']) / 2 - 1e-6)):
                    s_ = ln.project(Point(q))
                    if 0.02 < s_ < ln.length - 0.02:
                        cuts.append(s_)
                        if wq is not None and dq > 1e-6: joint.append((R(q), R(ln.interpolate(round(s_, 5)).coords[0])))


            for pm in own_pm:
                if MIDDLE and pm is not None and ln.distance(Point(pm)) <= 0.005:
                    s_ = ln.project(Point(pm))
                    if 0.02 < s_ < ln.length - 0.02: cuts.append(s_)
            ds = [0.0] + sorted(set(round(c, 5) for c in cuts)) + [ln.length]
            pts = [R(ln.interpolate(d).coords[0]) for d in ds]; pts[0], pts[-1] = R(a), R(b)
            for p, q in zip(pts, pts[1:]):
                if p != q:
                    kq = tuple(sorted([p, q]))
                    if kq in pw_seen and abs(pw_seen[kq] - t['w']) > 1e-9: dup_width.add(kq)
                    dup_w[kq].add(round(t['w'], 6))
                    pw_seen.setdefault(kq, t['w'])
                    pieces.append((p, q)); width[(p, q)] = t['w']; piece_w.append(t['w'])


        allp = sorted({p for pq in pieces for p in pq})
        cl = []; owner = {}
        for p in allp:
            for k, ms in enumerate(cl):
                if all(math.dist(p, m) <= TOL for m in ms): ms.append(p); owner[p] = k; break
            else: cl.append([p]); owner[p] = len(cl) - 1
        upar = list(range(len(cl)))
        def uf(k):
            while upar[k] != k: upar[k] = upar[upar[k]]; k = upar[k]
            return k
        for q, x in joint:
            if q in owner and x in owner: upar[uf(owner[q])] = uf(owner[x])
        if any(uf(k) != k for k in range(len(cl))):
            new_cl = collections.defaultdict(list)
            for k, ms in enumerate(cl): new_cl[uf(k)] += ms
            cl = [sorted(ms) for _, ms in sorted(new_cl.items())]
            owner = {p: k for k, ms in enumerate(cl) for p in ms}
        def f(p): return owner[p]
        members = {k: ms for k, ms in enumerate(cl)}
        def on_via(c): return any(v.distance(Point(p)) <= r for v, r in own_v for p in members[c])
        def on_pad(c): return any(g.contains(Point(p)) for g in own_p for p in members[c])
        def on_outer(c): return any(g.contains(Point(p)) for g in outer for p in members[c])
        rep = {}
        for c, ms in members.items():
            vs = [(v.x, v.y) for v, r in own_v if any(v.distance(Point(p)) <= r for p in ms)]
            rep[c] = R(vs[0]) if vs else collections.Counter(ms).most_common(1)[0][0]
        edges = []
        def in_via(p, q): return any(v.distance(Point(p)) <= r - 0.005 and v.distance(Point(q)) <= r - 0.005 for v, r in own_v)
        def in_pad(p, q): return any(g.buffer(-0.005).contains(Point(p)) and g.buffer(-0.005).contains(Point(q)) for g in own_p)
        mid = [(v.x, v.y) for v, r in own_v] + [(p['x'], p['y']) if 'x' in p else Polygon(p['poly']).centroid.coords[0]
                                                    for p in H['pads'] if p['net'] == net and p['layer'] == L]
        def superfluous(p, q):
            if not (in_via(p, q) or in_pad(p, q)): return False


            if MIDDLE and any((LineString([p, q]) if p != q else Point(p)).distance(Point(m)) <= 0.005 and min(math.dist(p, m), math.dist(q, m)) >= 1e-3
                             for m in mid): return False
            return not (MIDDLE and any(min(math.dist(p, m), math.dist(q, m)) < 1e-3 for m in mid))
        def reached_m(pieces):
            gs = [LineString([p, q]) if p != q else Point(p) for p, q in pieces]
            return {m for m in mid if any(g.distance(Point(m)) <= 0.005 for g in gs)}
        keep = []
        def to_middle_(p, q):
            g = LineString([p, q]) if p != q else Point(p)
            return MIDDLE and any(g.distance(Point(m)) <= 0.005 for m in mid)
        lost = collections.defaultdict(list)
        for idx_, (p, q) in enumerate(pieces):
            c1, c2 = f(p), f(q)
            if MODE == 'entire' and c1 == c2 and to_middle_(p, q): keep.append((p, q)); stat['Micro piece to the center stays'] += 1; continue
            if MODE == 'entire' and c1 == c2: stat['MicroPieces'] += 1; lost[c1].append((p, q)); continue
            if MODE == 'entire' and superfluous(p, q):
                stat['in the via/pad'] += 1; lost[c1].append((p, q)); lost[c2].append((p, q)); continue
            if tuple(sorted([p, q])) in dup_width: edge_w[len(edges)] = piece_w[idx_]
            edges.append((c1, c2, (p, q)))
        adj = collections.defaultdict(list)
        for k, (c1, c2, _) in enumerate(edges): adj[c1].append(k); adj[c2].append(k)
        way = set()
        def cut(p1, p2, q1, q2):
            d1 = (p2[0] - p1[0], p2[1] - p1[1]); d2 = (q2[0] - q1[0], q2[1] - q1[1]); the = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(the) < 1e-12: return None
            t_ = ((q1[0] - p1[0]) * d2[1] - (q1[1] - p1[1]) * d2[0]) / the
            return (p1[0] + t_ * d1[0], p1[1] + t_ * d1[1])






        fixed_nodes = {owner[x] for kq in dup_width for x in kq if x in owner}; closed = [0]
        def unit(a, b):
            d_ = math.dist(a, b)
            return None if d_ < 1e-9 else ((b[0] - a[0]) / d_, (b[1] - a[1]) / d_)
        def ends_on(c):
            out = []
            for k in adj[c]:
                if k in way: continue
                c1, c2, (p, q) = edges[k]
                if c1 == c: out.append((k, 'p', p, q))
                if c2 == c: out.append((k, 'q', q, p))
            return out
        def assign(k, board_side, e_new, o_new=None):
            c1, c2, (p, q) = edges[k]; ww = width.get((p, q), w)
            np_ = (e_new, o_new if o_new is not None else q) if board_side == 'p' else (o_new if o_new is not None else p, e_new)
            edges[k] = (c1, c2, np_); width[np_] = ww
        def bw_(k): return width.get(edges[k][2], w)
        def in_goal(p): return any(v.distance(Point(p)) <= r for v, r in own_v) or any(g.contains(Point(p)) for g in own_p)
        def merge(c):
            if c in fixed_nodes: return False
            es = ends_on(c)
            if len(es) != 2 or on_outer(c): return False
            (k1, s1, e1, o1), (k2, s2, e2, o2) = es
            if k1 == k2: return False
            wa, wb = bw_(k1), bw_(k2)
            d1, d2 = unit(o1, e1), unit(o2, e2)
            if d1 is None or d2 is None: return False
            before = lambda X, o, d: (X[0] - o[0]) * d[0] + (X[1] - o[1]) * d[1] > 1e-3
            X = cut(o1, e1, o2, e2)
            if X is not None:
                X = R(X)
                if math.dist(X, e1) > max(wa, wb) + TOL or math.dist(X, e2) > max(wa, wb) + TOL: return False
                if not (before(X, o1, d1) and before(X, o2, d2) and angle_ok(o1, X, o2)): return False
                if right_angled(o1, X, o2) and not in_goal(X): return False
                if not (remains_in_goal(e1, X) and remains_in_goal(e2, X)): return False
                if not (vacant(LineString([o1, X]), wa, L, net) and vacant(LineString([o2, X]), wb, L, net)): return False
                assign(k1, s1, X); assign(k2, s2, X); stat['node closure (intersection)'] += 1; return True
            cross = (e2[0] - e1[0]) * d1[1] - (e2[1] - e1[1]) * d1[0]
            if abs(cross) <= 1e-4:
                X = R(((e1[0] + e2[0]) / 2, (e1[1] + e2[1]) / 2))
                if not (before(X, o1, d1) and before(X, o2, d2)): return False
                if not (remains_in_goal(e1, X) and remains_in_goal(e2, X)): return False
                if not (vacant(LineString([o1, X]), wa, L, net) and vacant(LineString([o2, X]), wb, L, net)): return False
                assign(k1, s1, X); assign(k2, s2, X); stat['Node junction (line)'] += 1; return True

            for (ka, sa, ea, oa, da, wa_), (kb, sb, eb, ob, db, wb_) in (((k1, s1, e1, o1, d1, wa), (k2, s2, e2, o2, d2, wb)),
                                                                     ((k2, s2, e2, o2, d2, wb), (k1, s1, e1, o1, d1, wa))):
                ca = edges[ka][1] if sa == 'p' else edges[ka][0]
                if ca in fixed_nodes or on_via(ca) or on_pad(ca) or on_outer(ca) or not inw(rep[ca]): continue
                nb3 = [x for x in ends_on(ca) if x[0] != ka]
                if len(nb3) != 1 or len(ends_on(ca)) != 2 or nb3[0][2] != oa: continue
                k3, s3, _, o3 = nb3[0]; d3 = unit(o3, oa)
                if d3 is None or abs(d3[0] * da[1] - d3[1] * da[0]) < 1e-9: continue
                oa2 = cut(eb, (eb[0] + db[0], eb[1] + db[1]), o3, oa)
                if oa2 is None: continue
                oa2 = R(oa2); X = eb
                if math.dist(oa2, oa) > 2 * max(wa_, wb_) or not (before(X, oa2, da) and before(oa2, o3, d3)): continue
                if not (angle_ok(o3, oa2, X) and angle_ok(oa2, X, ob)): continue
                if not (remains_in_goal(ea, X) and remains_in_goal(oa, oa2)): continue
                hm_ = HMAX_NL.get((net, L), 0.0)
                if hm_ and (math.dist(oa2, oa) > hm_ or math.dist(X, ea) > hm_): continue
                w3 = bw_(k3)
                if not (vacant(LineString([oa2, X]), wa_, L, net) and vacant(LineString([o3, oa2]), w3, L, net)): continue
                assign(ka, sa, X, oa2); assign(k3, s3, oa2); stat['Node junction (laid onto a line)'] += 1; return True
            return False
        def remains_in_goal(e, X):
            return targets_from([e]) <= targets_from([X])
        def targets_from(pts):
            return {('v', i) for i, (v, r) in enumerate(own_v) for p in pts if v.distance(Point(p)) <= r} | \
                   {('p', i) for i, g in enumerate(own_p) for p in pts if g.contains(Point(p))}
        for c in sorted(lost):
            es = ends_on(c); pkt = sorted({e for _, _, e, _ in es})


            leave = bool(pkt) and bool(targets_from([x for pq in lost[c] for x in pq]) - targets_from(pkt))
            if not leave:
                if len(pkt) <= 1: continue
                if merge(c): closed[0] += 1; continue

            for pq in lost[c]:
                if f(pq[0]) == f(pq[1]):
                    if pq not in keep: keep.append(pq)
                elif not any(kk[2] == pq for kk in edges):
                    edges.append((f(pq[0]), f(pq[1]), pq)); k_ = len(edges) - 1; adj[f(pq[0])].append(k_); adj[f(pq[1])].append(k_)
                    fixed_nodes.add(f(pq[0])); fixed_nodes.add(f(pq[1]))
            fixed_nodes.add(c); stat['Piece stays (ends otherwise did not coincide)'] += len(lost[c])
        omitted = {pq for c in lost for pq in lost[c]} - set(keep) - {kk[2] for kk in edges}
        changed = MODE == 'entire' and (bool(omitted) or closed[0] > 0)
        for c in list(members):
            ks_ = adj[c]
            if len(ks_) != 2 or len(set(ks_)) != 2 or on_via(c) or on_pad(c) or on_outer(c) or c in fixed_nodes: continue
            if any(edges[k][0] == edges[k][1] for k in ks_): continue
            ends = []
            for k in ks_:
                c1, c2, (p, q) = edges[k]
                ends.append((k, 'p', p, q) if c1 == c else (k, 'q', q, p))
            (k1, s1, e1, o1), (k2, s2, e2, o2) = ends
            if e1 == e2: continue
            X = cut(o1, e1, o2, e2)
            if X is None:
                d = (e1[0] - o1[0], e1[1] - o1[1]); ld = math.hypot(*d)
                if ld < 1e-9 or abs((e2[0] - o1[0]) * d[1] - (e2[1] - o1[1]) * d[0]) / ld > 1e-4: continue
                X = ((e1[0] + e2[0]) / 2, (e1[1] + e2[1]) / 2)
            X = R(X); wa, wb = width.get(edges[k1][2], w), width.get(edges[k2][2], w)
            if math.dist(X, e1) > max(wa, wb) or math.dist(X, e2) > max(wa, wb): continue
            if math.dist(X, o1) < 1e-3 or math.dist(X, o2) < 1e-3: continue
            new1 = (X, o1) if s1 == 'p' else (o1, X); new2 = (X, o2) if s2 == 'p' else (o2, X)
            if not (vacant(LineString(new1), wa, L, net) and vacant(LineString(new2), wb, L, net)): continue
            for k, net_copper, ww in ((k1, new1, wa), (k2, new2, wb)):
                c1, c2, _ = edges[k]; edges[k] = (c1, c2, net_copper); width[net_copper] = ww
            stat['Joints aligned'] += 1; changed = True
        def deg(c): return sum(1 for k in adj[c] if k not in way)
        def pinned(c): return deg(c) != 2 or c in fixed_nodes or on_via(c) or on_pad(c) or on_outer(c) or not inw(rep[c])
        def alive(c):
            if on_via(c) or on_pad(c) or on_outer(c) or not inw(rep[c]): return True
            for m in members[c]:
                hw = (pw.get(m) or w) / 2
                if any(v.distance(Point(m)) <= r + hw for v, r in own_v): return True
                if any(g.distance(Point(m)) <= hw for g in own_p) or any(g.distance(Point(m)) <= hw for g in outer): return True
            for k, (c1, c2, (p, q)) in enumerate(edges):
                if k in way or c in (c1, c2): continue
                wk = width.get((p, q), width.get((q, p), w))
                if any(LineString([p, q]).distance(Point(m)) <= ((pw.get(m) or w) + wk) / 2 for m in members[c]): return True
            return False

        while MODE == 'entire' and DEAD and net not in DEAD_OFF:
            tot = [c for c in members if deg(c) == 1 and not alive(c)]
            if not tot: break
            for c in tot:
                ks = [k for k in adj[c] if k not in way]
                if len(ks) == 1: way.add(ks[0]); stat['dead pieces'] += 1; changed = True

        seen = set(); chain_list = []
        for c in members:
            if not pinned(c) or deg(c) == 0: continue
            for k in adj[c]:
                if k in way or k in seen: continue
                kn = [c]; ks = []; cur = c; kk = k
                while True:
                    seen.add(kk); ks.append(kk); c1, c2, _ = edges[kk]; nxt = c2 if c1 == cur else c1; kn.append(nxt); cur = nxt
                    if pinned(cur): break
                    kk = [x for x in adj[cur] if x not in way and x != kk][0]
                chain_list.append((kn, ks))
        replaced = set(); new = []
        def exact_nb(p, without):
            out = []
            for k, (c1, c2, (a, b)) in enumerate(edges):
                if k in way or k in without or k in replaced: continue
                if a == p: out.append(b)
                elif b == p: out.append(a)
            out += [q for p_, q, _ in new if p_ == p] + [p_ for p_, q, _ in new if q == p]
            out += [b for a, b in keep if a == p] + [a for a, b in keep if b == p]
            return sorted(set(out))
        def neighbors(c, k_owned):
            out = []
            for k in adj[c]:
                if k in way or k == k_owned or k in replaced: continue
                c1, c2, _ = edges[k]; out.append(rep[c2 if c1 == c else c1])
            return out
        def pad_center(c, p):


            if not PAD_CENTER or deg(c) != 1 or on_via(c) or on_outer(c): return p

            middle = {R(pm or g.centroid.coords[0]) for g, pm in zip(own_p, own_pm) if g.contains(Point(p))
                     and g.contains(Point(pm or g.centroid.coords[0])) and inw(pm or g.centroid.coords[0])}
            return next(iter(middle)) if len(middle) == 1 else p
        for kn, ks in (chain_list if MODE == 'entire' else []):

            seq = []
            for idx, k in enumerate(ks):
                c1, c2, (p, q) = edges[k]
                seq.append((p, q) if c1 == kn[idx] else (q, p))
            n = len(seq); out = []; i = 0; different = False
            while i < n:
                found = None
                for j in range(n, i + 1, -1):
                    a, b = seq[i][0], seq[j - 1][1]
                    prior = sum(math.dist(p, q) for p, q in seq[i:j])
                    if math.dist(a, b) <= TOL:


                        if any(alive(kn[x]) for x in range(i + 1, j)): continue
                        if net in BUDGET and prior > BUDGET[net] + 1e-9: continue
                        if MIDDLE and reached_m(seq[i:j]) - reached_m([(a, b)]): continue
                        found = (j, [a, b], w); break
                    if i == 0: a = pad_center(kn[0], a)
                    if j == n: b = pad_center(kn[-1], b)
                    def bwe(pq): return width.get(pq, width.get((pq[1], pq[0]), w))
                    bs = sorted({bwe(pq) for pq in seq[i:j]}, reverse=True)
                    for c in candidates(a, b):
                        nseg = sum(1 for p, q in zip(c, c[1:]) if p != q)
                        if not (plen(c) < prior - 1e-3 or (plen(c) <= prior + 1e-4 and nseg < j - i)): continue
                        hm_ = min([h_ for h_ in (HMAX, HMAX_NL.get((net, L), 0.0)) if h_ > 0] or [0.0])
                        if hm_ and len(set(map(tuple, c))) > 1 and LineString(c).hausdorff_distance(LineString([seq[i][0]] + [q for _, q in seq[i:j]])) > hm_: continue

                        segs_c = [(p, q) for p, q in zip(c, c[1:]) if p != q]; wl = []
                        for si, (p, q) in enumerate(segs_c):
                            want = bwe(seq[i]) if si == 0 else (bwe(seq[j - 1]) if si == len(segs_c) - 1 else min(bs))
                            ok = next((bw for bw in [want] + [x for x in bs if x < want] if vacant(LineString([p, q]), bw, L, net)), None)
                            if ok is None: break
                            wl.append(ok)
                        if len(wl) != len(segs_c): continue
                        wk = wl
                        prevs = [out[-1][0]] if out else neighbors(kn[0], ks[0])
                        nxts = [seq[j][1]] if j < n else neighbors(kn[-1], ks[-1])
                        if len(c) > 1 and math.dist(c[0], c[-1]) > TOL and (not all(angle_ok(pv, c[0], c[1]) for pv in prevs) or not all(angle_ok(c[-2], c[-1], nx) for nx in nxts)): continue

                        if len(c) > 1 and math.dist(c[0], c[-1]) > TOL:

                            pv1 = [out[-1][0]] if out else exact_nb(c[0], set(ks[i:j]))
                            nx1 = [seq[j][1]] if j < n else exact_nb(c[-1], set(ks[i:j]))
                            if ((len(pv1) == 1 and not in_goal(c[0]) and right_angled(pv1[0], c[0], c[1]) and not right_angled(pv1[0], seq[i][0], seq[i][1]))
                                    or (len(nx1) == 1 and not in_goal(c[-1]) and right_angled(c[-2], c[-1], nx1[0]) and not right_angled(seq[j - 1][0], seq[j - 1][1], nx1[0]))):
                                stat['Candidate rejected (new 90-degree corner)'] += 1; continue
                        if net in BUDGET and abs(prior - plen(c)) > BUDGET[net] + 1e-9:
                            stat['Candidate rejected (length gate)'] += 1; continue


                        if MIDDLE and reached_m(seq[i:j]) - reached_m(list(zip(c, c[1:]))):
                            stat['Candidate rejected (center would be lost)'] += 1; continue
                        found = (j, c, wk); break
                    if found: break
                if found:
                    j, c, wk = found; stat['shortened'] += 1
                    saved_ = sum(math.dist(p, q) for p, q in seq[i:j]) - plen(c)
                    stat['mm saved (proposed)'] += round(saved_, 4)
                    if net in BUDGET: BUDGET[net] -= abs(saved_); booked[(net, L)] += abs(saved_)
                    if c[0] != seq[i][0] or c[-1] != seq[j - 1][1]: stat['to pad center'] += 1
                    if math.dist(c[0], c[-1]) > TOL:
                        segs_c = [(p, q) for p, q in zip(c, c[1:]) if p != q]
                        for si, (p, q) in enumerate(segs_c):
                            out.append((p, q)); width[(p, q)] = wk[si] if isinstance(wk, list) else wk
                    i = j; different = True
                else: out.append(seq[i]); i += 1
            if not different: continue

            def bw(pq): return width.get(pq, width.get((pq[1], pq[0]), w))
            z = []
            for p, q in out:
                if z and z[-1][1] == p and abs(bw(z[-1]) - bw((p, q))) < 1e-6:
                    a, b = z[-1]
                    if abs((b[0] - a[0]) * (q[1] - b[1]) - (b[1] - a[1]) * (q[0] - b[0])) < 1e-6 and (b[0] - a[0]) * (q[0] - b[0]) + (b[1] - a[1]) * (q[1] - b[1]) > 0:
                        ww = bw(z[-1]); z[-1] = (a, q); width[(a, q)] = ww; continue
                z.append((p, q))
            replaced.update(ks); new += [(p, q, bw((p, q))) for p, q in z]; changed = True
        if not changed: continue
        rm_, add_ = [t['uuid'] for t in inner], []
        for k, (c1, c2, (p, q)) in enumerate(edges):
            if k in way or k in replaced: continue
            add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=edge_w.get(k, width.get((p, q), w))))
        for p, q, ww in new:
            if p != q: add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=ww))
        for p, q in keep:
            kq = tuple(sorted([p, q]))
            for ww in (sorted(dup_w[kq]) if kq in dup_width else [width.get((p, q), w)]):
                add_.append(dict(net=net, layer=L, a=list(p), b=list(q), w=ww))


        if MIDDLE:
            for s_ in add_:
                for e, o in (('a', 'b'), ('b', 'a')):
                    x, y = tuple(s_[e]), tuple(s_[o])
                    for m in mid:
                        d_ = math.dist(x, m)
                        if not (1e-6 < d_ <= TOL): continue
                        cr = (x[0] - y[0]) * (m[1] - y[1]) - (x[1] - y[1]) * (m[0] - y[0])

                        if abs(cr) <= 1e-6 * max(math.dist(x, y), 1e-9) and (x[0] - y[0]) * (m[0] - y[0]) + (x[1] - y[1]) * (m[1] - y[1]) > math.dist(x, y) ** 2 + 1e-12:
                            s_[e] = [m[0], m[1]]; stat['extended to the center'] += 1; break
        add_ = [s_ for s_ in add_ if R(s_['a']) != R(s_['b'])]
        if ZW is not None:
            broken = ZW.verify(net, add_)
            if broken:
                stat['rejected (cuts zone)'] += 1; print('rejected, cuts the zone of', ', '.join(broken), ':', net, L)
                if net in BUDGET: BUDGET[net] += booked.pop((net, L), 0.0)
                continue
            ZW.commit(net, add_, rm_)
        for s_ in add_: new_obstacle(net, L, s_['a'], s_['b'], s_['w'])
        change[(net, L)] = (rm_, add_)

if A.get('connection', '1') == '1' and change:
    from common.copper_net import NetCopper, apply
    NET_COPPER = NetCopper(H)
    for net in sorted({n for n, _ in change}):
        lag_ = sorted(L for n, L in change if n == net); prior = NET_COPPER.tracks.get(net, [])


        fl = [(f['layer'], f['poly']) for f in FILL if f['net'] == net] if ZW is not None else None
        def with_(Ls):
            rm_ = [u for L in Ls for u in change[(net, L)][0]]; ad_ = [x for L in Ls for x in change[(net, L)][1]]
            return apply(prior, rm_, ad_)
        if not NET_COPPER.separates(net, prior, with_(lag_), fl, fl): continue
        good = []
        for L in lag_:
            if not NET_COPPER.separates(net, prior, with_(good + [L]), fl, fl): good.append(L)
        for L in lag_:
            if L not in good:
                del change[(net, L)]; stat['rejected (splits net)'] += 1
                if net in BUDGET: BUDGET[net] += booked.pop((net, L), 0.0)
                print('rejected, splits the net:', net, L)
length_prev = {t['uuid']: math.dist(t['a'], t['b']) for t in H['tracks']}
for (net, L) in sorted(change):
    stat['Net layers changed'] += 1
    remove += change[(net, L)][0]; add += change[(net, L)][1]

    stat['mm saved'] += round(sum(length_prev[u] for u in change[(net, L)][0]) - sum(math.dist(a_['a'], a_['b']) for a_ in change[(net, L)][1]), 4)
json.dump(dict(remove=remove, add=add), open(OUT, 'w'))
print(dict(stat), 'removed', len(remove), 'new', len(add))
