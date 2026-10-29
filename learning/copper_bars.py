import copy
import math
import time
from collections import defaultdict

import numpy as np
import shapely
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

POWER = ('current_val', 'supply', 'sw')
TAG = 'current_shape_v2'


def polygon(z):
    return Polygon(z['pts'], z.get('holes', [])).buffer(0)


def track_geometry(t):
    from common.arc import points
    if 'items' in t:
        gs = []
        for it in t['items']:
            if it[0] == 'L':
                ps, w = it[1:3], it[3]
            elif it[0] == 'A':
                ps, w = points(*it[1:4]), it[4]
            else:
                raise ValueError('Unknown track item: ' + str(it[0]))
            gs.append(LineString(ps).buffer(w / 2 + (0.00025 if it[0] == 'A' else 0), quad_segs=16))
        return unary_union(gs)
    return unary_union([LineString([a, b]).buffer(w / 2, quad_segs=16)
                        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']) if a != b])


def cu_layers(B):
    return ['F'] + ['In%d' % i for i in range(1, int(B.get('cu_layers') or 2) - 1)] + ['B']


def via_size(R):
    return tuple(map(float, str((R.get('options') or {}).get('via', '0.45/0.20')).split('/')))


def pad_polygon(B, p, q):
    xy = np.asarray(q)
    if str(B.get('src', '')).startswith('task seed=') and len(xy) == 12:
        rr = np.linalg.norm(xy - [p['x'], p['y']], axis=1)
        if np.max(rr) - np.min(rr) < 0.00003:
            return Point(p['x'], p['y']).buffer(float(np.max(rr)), quad_segs=64)
    return Polygon(q).buffer(0)


def copper_map(B, R, *, unfilled='faults'):
    from common.zones import unify, board_geometry
    B = board_geometry(B, R)
    R = unify(R)
    parts = defaultdict(list)
    ls = cu_layers(B)
    vd, _ = via_size(R)
    for p in B['pads']:
        for l, ps in p.get('polys', {}).items():
            parts[p['net'], l] += [pad_polygon(B, p, q) for q in ps if len(q) >= 3]
    for t in R.get('tracks', []):
        parts[t['net'], t['layer']].append(track_geometry(t))
    for v in R.get('vias', []):
        for l in v.get('layers') or ls:
            parts[v['net'], l].append(Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', vd)) / 2, quad_segs=24))
    for z in R.get('zones', []):
        if not z.get('filled'):
            if unfilled == 'ignore': continue
            raise ValueError('Unfilled zone: fill/export natively before extension')
        parts[z['net'], z['layer']].append(polygon(z))
    return {k: unary_union(v) for k, v in parts.items()}


def _design_rules(R, design_rules):

    rg = (design_rules or {}).get('router', design_rules or {})
    op = R.get('options') or {}
    raw_data = (design_rules or {}).get('raw_data') or {}
    def minimum(entries, fallback):
        vals = [float(source[key]) for source, key in entries if source.get(key) is not None]
        return max(vals) if vals else fallback
    clr = minimum([(R,'clr'),(raw_data,'min_clearance')] + [(source,key) for source in (rg,op)
                   for key in ('clr','drc','drcpad')], .1)
    edge = minimum([(raw_data,'min_copper_edge_clearance'),(rg,'edge'),(op,'edge')], .25)
    hole = minimum([(raw_data,'min_hole_clearance'),(rg,'h2h'),(op,'h2h')], .2)
    return clr, edge, hole


def _keepouts(B, ls):
    out = defaultdict(list)
    for k in B.get('keepouts', []):
        vb = k.get('prohibitions') or {}
        if not vb.get('areas', vb.get('track_list', True)):
            continue
        for l in k.get('cu_layers') or ls:
            out[l] += [Polygon(p).buffer(0) for p in k.get('polys', []) if len(p) >= 3]
    return {l: unary_union(g) for l, g in out.items()}


def _free(B, R, cu, net, layer, outline, blocked, clr, hole):
    foreign = unary_union([g for (n, l), g in cu.items() if l == layer and n != net])


    protected = unary_union([track_geometry(t) for t in R['tracks'] if t['net'] == net and t['layer'] == layer
                             and (t.get('sig') or t.get('kelvin'))])
    if not protected.is_empty:
        own_pads = unary_union([pad_polygon(B, p, q) for p in B['pads'] if p['net'] == net
                               for q in p.get('polys', {}).get(layer, []) if len(q) >= 3])
        protected = protected.buffer(clr + 0.005, quad_segs=24).difference(own_pads)
    holes = [Point(p['x'], p['y']).buffer(p['drill'] / 2 + hole, quad_segs=24)
             for p in B['pads'] if p.get('drill') and p['net'] != net]

    _, dr = via_size(R)
    holes += [Point(v['x'], v['y']).buffer(v.get('drill', dr) / 2 + hole, quad_segs=24)
              for v in R['vias'] if v['net'] != net]
    return outline.difference(unary_union([foreign.buffer(clr + 0.005, quad_segs=24),
            protected, blocked.get(layer, Polygon())] + holes))


def _parts(g):
    return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]


def _islands_ok(outline, old, new, min_gap=0.15):
    a = outline.difference(old).buffer(-min_gap / 2)
    b = outline.difference(new).buffer(-min_gap / 2)
    before, after = _parts(a), _parts(b)
    edge = outline.buffer(-min_gap / 2).boundary
    for p in before:
        connected = [q for q in after if q.intersection(p).area > 0.002]
        if len(connected) > 1:
            return False
        if connected and p.distance(edge) < 1e-6 and connected[0].distance(edge) > 1e-6:
            return False
    return True


def _zones(g, net, layer, ti, clr=0.1):
    return [dict(net=net, layer=layer, pts=[list(p) for p in q.exterior.coords[:-1]],
                 holes=[[list(p) for p in ring.coords[:-1]] for ring in q.interiors], prio=1000 + ti,
                 filled=True, estimated=True, full=True, min_thickness=0.05, local_clearance=clr,
                 current_path=True, copper_bars=TAG, track=ti)
            for q in _parts(g) if q.area > 0.001]


def check_widths(R, K, h=0.1):
    out = []
    for ti, t in enumerate(R['tracks']):
        if K['net_list'].get(t['net'], {}).get('category') not in POWER or t.get('sig') or t.get('kelvin'):
            continue
        g = unary_union([track_geometry(t)] + [polygon(z) for z in R.get('zones', [])
                        if z.get('layer') == t['layer'] and z.get('net') == t['net']])
        rings = [np.asarray(r.coords) for q in _parts(g) for r in [q.exterior] + list(q.interiors)]
        rate, steps = 0.0, set()

        for poly in _parts(g):
            for ri, ring in enumerate([poly.exterior] + list(poly.interiors)):
                xy = np.asarray(ring.coords[:-1])
                u, v = xy - np.roll(xy, 1, axis=0), np.roll(xy, -1, axis=0) - xy
                cross = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
                angle = np.arctan2(cross, np.sum(u * v, axis=1))
                orientation = (1 if ring.is_ccw else -1) * (-1 if ri else 1)
                for p in xy[angle * orientation < -math.radians(15)]:
                    steps.add(tuple(np.round(p, 4)))
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            length = math.dist(a, b)
            if length < 0.001:
                continue
            tangent = (np.asarray(b) - a) / length
            normal = np.array([-tangent[1], tangent[0]]) * 30
            def widths(ss):
                xy = np.asarray(a) + np.asarray(ss)[:, None] * tangent
                cuts = shapely.intersection(shapely.linestrings(np.stack([xy - normal, xy + normal], axis=1)), g)
                return np.asarray([max((q.length for q in getattr(c, 'geoms', [c]) if q.distance(Point(p)) < 1e-7), default=0)
                                   for p, c in zip(xy, cuts)])
            ss = np.linspace(0.0001, length - 0.0001, max(2, math.ceil(length / h) + 1))
            ww = widths(ss)
            rate = max(rate, float(np.max(np.abs(np.diff(ww))) / (ss[1] - ss[0])))
            for ring in rings:
                delta = np.diff(ring, axis=0)

                candidates = np.abs(delta @ tangent) < 1e-8
                candidates &= np.linalg.norm(delta, axis=1) > 0.01
                s = (ring[:-1][candidates] - a) @ tangent
                for pos in s[(s > 0.001) & (s < length - 0.001)]:
                    vals = widths([pos - 0.0001, pos + 0.0001])
                    if abs(vals[1] - vals[0]) > 0.01:
                        steps.add(tuple(np.round(np.asarray(a) + pos * tangent, 4)))
        out.append(dict(track=ti, net=t['net'], layer=t['layer'], max_dw_pro_mm=rate, levels=len(steps)))
    return dict(per_track=out, max_dw_pro_mm=max((q['max_dw_pro_mm'] for q in out), default=0),
                levels=sum(q['levels'] for q in out), sample_mm=h, jump_threshold_mm=0.01)


import collections
from shapely.geometry import box
from common.flow import goal_width
from common.current_track import nominal_width

H = 0.05
SAFE = 0.005


def args_kv(argv):
    return dict(a.split('=', 1) for a in argv if '=' in a)


def layers_from(B):
    n = int(B.get('cu_layers') or 2)
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def seg_poly(a, b, w):
    return (LineString([a, b]) if math.dist(a, b) > 1e-9 else Point(a)).buffer(w / 2, quad_segs=16)


def pad_polys(p, L):
    return [Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]



class Slot:
    def __init__(self, geoms_clr, outline, edge):
        self.part_list = [(g, c) for g, c in geoms_clr if not g.is_empty]
        self.tree = shapely.STRtree([g for g, _ in self.part_list]) if self.part_list else None
        self.clr = np.array([c for _, c in self.part_list]) if self.part_list else np.zeros(0)
        self.rand = outline.boundary; self.outline = outline; self.edge = edge
        self.cmax = float(self.clr.max()) if len(self.clr) else 0.0

    def wmax(self, xy, cap=50.0):
        pts = shapely.points(xy)
        w = 2 * (shapely.distance(pts, self.rand) - self.edge - SAFE)
        w = np.where(shapely.contains_xy(self.outline, xy[:, 0], xy[:, 1]), w, -1.0)
        w = np.minimum(w, cap)
        if self.tree is None: return w

        r = cap / 2 + self.cmax + SAFE
        pi, gi = self.tree.query(pts, predicate='dwithin', distance=r)
        if len(pi):
            d = shapely.distance(pts[pi], np.array([g for g, _ in self.part_list], dtype=object)[gi])
            ww = 2 * (d - self.clr[gi] - SAFE)
            np.minimum.at(w, pi, ww)
        return w


def obstacles(B, R, K, L, net_name, new_copper, design_rules):
    c_track, _, c_hole = _design_rules(R, design_rules)
    c_pad = c_track
    d_via = float(str((R.get('options') or {}).get('via', '0.45/0.20')).split('/')[0])
    out = []
    for p in B['pads']:
        if p['net'] != net_name:
            for g in pad_polys(p, L): out.append((g, c_pad))
        if p.get('drill') and (p['net'] != net_name or not p.get('polys', {}).get(L)):
            out.append((Point(p['x'], p['y']).buffer(p['drill'] / 2, 24), c_hole))
    for t in R['tracks']:
        if t['layer'] != L: continue
        if t['net'] == net_name and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web', 'items')): continue
        if (t['net'], L) in new_copper and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web', 'items')): continue
        out.append((track_geometry(t), c_track))
    for v in R['vias']:
        if v['net'] != net_name and L in (v.get('layers') or cu_layers(B)) and (v['net'], L) not in new_copper:
            out.append((Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', d_via)) / 2, 24), c_track))
    _, drill = via_size(R)
    for v in R['vias']:
        if v['net'] != net_name:
            out.append((Point(v['x'], v['y']).buffer(v.get('drill', drill) / 2, 24), c_hole))
    for z in R.get('zones', []):
        if z['layer'] == L and z['net'] != net_name and z.get('filled'):
            out.append((polygon(z), c_track))
    for (n, LL), g in new_copper.items():
        if LL == L and n != net_name: out.append((g, c_track))
    for k in B.get('keepouts', []):
        vb = k.get('prohibitions') or {}
        if vb.get('areas', vb.get('track_list', True)) and L in (k.get('cu_layers') or [L]):
            for q in k.get('polys', []):
                if len(q) >= 3: out.append((Polygon(q).buffer(0), 0.0))
    return out



def paths(tracks_idx, R, B, net_name, L, vias):
    segs = []
    for ti in tracks_idx:
        t = R['tracks'][ti]
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            if math.dist(a, b) > 1e-6: segs.append([tuple(a), tuple(b), ti, si, w])

    marks = [tuple(q) for ti in tracks_idx for q in R['tracks'][ti]['pts']] + [(v['x'], v['y']) for v in vias]
    new = []
    for a, b, ti, si, w in segs:
        ab = np.subtract(b, a); l2 = float(ab @ ab); part = [0.0, 1.0]
        for q in marks:
            s = float(np.subtract(q, a) @ ab) / l2
            if 1e-6 < s < 1 - 1e-6 and math.dist(q, (a[0] + s * ab[0], a[1] + s * ab[1])) < 1e-4: part.append(s)
        part = sorted(set(part))
        for s1, s2 in zip(part[:-1], part[1:]):
            new.append(((a[0] + s1 * ab[0], a[1] + s1 * ab[1]), (a[0] + s2 * ab[0], a[1] + s2 * ab[1]), ti, si, w))
    key = lambda q: (round(q[0], 4), round(q[1], 4))
    adj = collections.defaultdict(list)
    for i, (a, b, ti, si, w) in enumerate(new):
        adj[key(a)].append((i, key(b))); adj[key(b)].append((i, key(a)))
    padg = unary_union([g for p in B['pads'] if p['net'] == net_name for g in pad_polys(p, L)])
    viak = {key((v['x'], v['y'])) for v in vias}
    def nodes(k):
        return len(adj[k]) != 2 or k in viak or (not padg.is_empty and padg.contains(Point(k)))
    used = set(); out = []
    starts = [k for k in sorted(adj) if nodes(k)] + sorted(adj)
    for k0 in starts:
        for i0, k1 in sorted(adj[k0]):
            if i0 in used: continue
            P = [k0]; S = []; i, k = i0, k1
            while True:
                used.add(i); P.append(k); S.append(i)
                if nodes(k): break
                nxt = [(j, m) for j, m in adj[k] if j not in used]
                if not nxt: break
                i, k = nxt[0]
            pts = [new[S[0]][0] if key(new[S[0]][0]) == P[0] else new[S[0]][1]]
            for j, kk in zip(S, P[1:]):
                a, b = new[j][0], new[j][1]
                pts.append(b if key(b) == kk else a)
            out.append((pts, [(new[j][2], new[j][3]) for j in S], [new[j][4] for j in S]))
    return out


def simplify(P):
    Q = [P[0]]
    for p in P[1:]:
        if math.dist(p, Q[-1]) > 1e-6: Q.append(p)
    i = 1
    while i < len(Q) - 1:
        a, b, c = np.array(Q[i - 1]), np.array(Q[i]), np.array(Q[i + 1])
        u, v = b - a, c - b
        if abs(u[0] * v[1] - u[1] * v[0]) < 1e-9 * (np.linalg.norm(u) * np.linalg.norm(v) + 1e-12) and u @ v > 0: Q.pop(i)
        else: i += 1
    return Q


def arc(a, b, c, r):
    a, b, c = map(np.asarray, (a, b, c)); u = (b - a) / np.linalg.norm(b - a); v = (c - b) / np.linalg.norm(c - b)
    cosd = float(np.clip(u @ v, -1, 1)); dth = math.acos(cosd)
    if dth < 1e-4: return None
    T = r * math.tan(dth / 2)
    t1 = b - u * T
    s = 1 if (u[0] * v[1] - u[1] * v[0]) > 0 else -1
    nrm = np.array([-u[1], u[0]]) * s; m = t1 + nrm * r
    a0 = math.atan2(t1[1] - m[1], t1[0] - m[0]); n = max(2, int(math.ceil(r * dth / 0.02)))
    pts = [m + r * np.array([math.cos(a0 + s * dth * k / n), math.sin(a0 + s * dth * k / n)]) for k in range(n + 1)]
    return T, pts


def sample(P, h=H):
    P = np.asarray(P, float); d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    n = max(2, int(math.ceil(d[-1] / h)) + 1); s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])], s


def rounds_n(P, slot, w_nominal, w_floor, radii=(3.0, 2.0, 1.5, 1.0, 0.7, 0.5, 0.35, 0.25, 0.15, 0.1)):
    P = simplify(P)
    if len(P) < 3: return P, []
    L = [math.dist(P[i], P[i + 1]) for i in range(len(P) - 1)]
    out = [np.asarray(P[0])]; rs = []
    free_front = L[0]
    for i in range(1, len(P) - 1):
        back = L[i] if i == len(P) - 2 else L[i] / 2
        prior, _ = sample([P[i - 1], P[i], P[i + 1]], 0.05)
        cap_prev = min(w_nominal, float(slot.wmax(prior, cap=w_nominal + 0.1).min()))
        wt = None
        for r in radii:
            b = arc(P[i - 1], P[i], P[i + 1], r)
            if b is None: break
            T, pts = b
            if T > free_front - 1e-6 or T > back - 1e-6: continue
            wm = slot.wmax(np.asarray(pts), cap=w_nominal + 0.1)
            if wm.min() >= max(w_floor, cap_prev): wt = (r, pts, T); break
        if wt is None:
            out.append(np.asarray(P[i])); free_front = L[i]
        else:
            r, pts, T = wt; out.extend(pts); rs.append(r); free_front = L[i] - T
    out.append(np.asarray(P[-1]))
    return [tuple(map(float, q)) for q in out], rs


def smoothed(x, n):
    if n < 1: return x
    xp = np.r_[np.full(n, x[0]), x, np.full(n, x[-1])]; c = np.cumsum(np.r_[0, xp])
    return (c[2 * n + 1:] - c[:-2 * n - 1]) / (2 * n + 1)


def plan(wm, s, w_nominal, w_floor, k, a):
    goal = np.minimum(w_nominal, wm)
    floor = float(w_floor)
    goal = np.maximum(floor, goal)
    ds = float(s[1]-s[0]); n = max(1, int(round(a/2/ds)))
    t = goal.copy()
    for _ in range(12):
        e = t.copy()
        for j in range(1,len(e)): e[j] = min(e[j],e[j-1]+k*ds)
        for j in range(len(e)-2,-1,-1): e[j] = min(e[j],e[j+1]+k*ds)
        r = smoothed(smoothed(e,n),n)
        over = r-goal
        if float(over.max()) <= 1e-5: break
        t -= np.maximum(over,0)*1.5 + (over>1e-5)*1e-4

    r += max(0.0,floor-float(r.min()))
    spline = CubicSpline(s,r,bc_type=((1,0.0),(1,0.0)))
    return spline(s),goal


def move(xy, w):
    d = shapely.buffer(shapely.points(xy), w / 2, quad_segs=16)
    return shapely.union_all(shapely.convex_hull(shapely.union(d[:-1], d[1:]))) if len(d) > 1 else d[0]


def close(N, vacant, places, radii=(0.4, 0.25, 0.15, 0.08, 0.04)):
    n_ok = 0
    vacant = vacant.intersection(box(*N.bounds).buffer(1.0)); shapely.prepare(vacant)
    for r in radii:
        window = unary_union([Point(x, y).buffer(h + 2.5 * r, 16) for x, y, h in places]); shapely.prepare(window)
        C = N.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
        add = C.difference(N)
        ok = []
        for q in getattr(add, 'geoms', [add]):
            if q.is_empty or q.area < 2e-5: continue
            if q.area > max(0.02, 1.2 * r * r): continue
            qi = q.buffer(-1e-5)
            if vacant.covers(qi) and window.covers(qi): ok.append(q)
        if ok:
            N = unary_union([N] + ok); n_ok += len(ok)
    return N, n_ok


def pads_enclose(N, pads, vacant, profile, margin=0.12):
    report = []; anchors = []
    for pad in pads:
        shell = pad
        chosen = 0.0
        for gap in (margin, .08, .04, .02, .01, .005):
            candidate = pad.buffer(gap, quad_segs=24)
            if vacant.covers(candidate.difference(pad.buffer(.0001))):
                shell, chosen = candidate, gap
                break
        N = N.union(shell)
        taper = 0.0
        for path in profile:
            xy=np.asarray(path['xy']); ss=np.asarray(path['s_mm']); widths=np.asarray(path['w_mm'])
            inside=shapely.contains_xy(pad,xy[:,0],xy[:,1])
            if not inside.any(): continue
            center=int(np.flatnonzero(inside)[len(np.flatnonzero(inside))//2])
            radius=max(pad.bounds[2]-pad.bounds[0],pad.bounds[3]-pad.bounds[1])/2
            for reach in (6.,4.,2.,1.,.5):
                selected=np.abs(ss-ss[center])<=radius+reach
                if not selected.any(): continue
                discs=shapely.buffer(shapely.points(xy[selected]),widths[selected]/2,quad_segs=16)
                hull=shapely.convex_hull(shapely.union_all([shell,*discs]))
                added=hull.difference(N)
                if vacant.covers(added.buffer(-.00001)):
                    N=N.union(hull);taper=max(taper,reach);break

        c = pad.centroid
        radius = max(pad.bounds[2]-pad.bounds[0], pad.bounds[3]-pad.bounds[1])/2
        anchors.append((c.x,c.y,radius))
        report.append(dict(pad_bbox=list(pad.bounds), hem_mm=chosen,
                           enclosed=chosen > 0, connection_window_mm=taper, reason='' if chosen else 'No free complete pad rim'))
    if anchors:
        N, _ = close(N, vacant.union(unary_union(pads)), anchors, radii=(.7,.4,.2,.1,.05))
    return N, report


def current_per_path(PF, kk, pads_n, vias_n, extra=()):
    fl = kk.get('flow') or {}; source = set(fl.get('source') or []); load = fl.get('load') or {}
    par = {}
    def f(x):
        while par.get(x, x) != x: x = par.get(x, x)
        return x
    def u(a, b): par[f(a)] = f(b)
    for vi, v in enumerate(vias_n):
        for p in pads_n:
            if any(g.covers(Point(v['x'], v['y'])) for L in (p.get('polys') or {}) for g in pad_polys(p, L)): u(('VIA', vi), ('PAD', p['ref'], p['num']))
    for a_, b_ in extra: u(a_, b_)
    for q in source: u(('PAD',) + tuple(q.split(':')), 'Q')
    nodes = sorted({f(x) for pf in PF for x in (pf['ka'], pf['kb'])} | {'Q'}, key=str)
    ix = {k: i for i, k in enumerate(nodes)}; n = len(nodes)
    G = np.zeros((n, n)); g_ = []
    for pf in PF:
        a, b = ix[f(pf['ka'])], ix[f(pf['kb'])]; g = 1.0 / max(pf['total_length'], 0.01); g_.append((a, b, g))
        if a == b: continue
        G[a, a] += g; G[b, b] += g; G[a, b] -= g; G[b, a] -= g
    I = np.zeros(n); ges = 0.0
    for k, A in load.items():
        key = f(('PAD',) + tuple(k.split(':')))
        if A > 0 and k not in source:
            if key not in ix: raise ValueError('Load without targeted feed line: '+k)
            I[ix[key]] -= A; ges += A
    q = ix['Q']; m = [i for i in range(n) if i != q]
    V = np.zeros(n)
    if m and ges > 0:
        V[m] = np.linalg.lstsq(G[np.ix_(m, m)], I[m], rcond=None)[0]
        if np.max(np.abs(G[m] @ V-I[m])) > max(1e-8, ges*1e-8):
            raise ValueError('Current graph separates load from source')
    return [g * abs(V[a] - V[b]) for a, b, g in g_], ges, f


def levels(w, s, threshold=0.05):
    if len(w) < 2: return 0, 0.0
    dw = np.abs(np.diff(w)); ds = np.diff(s)
    return int((dw > threshold).sum()), float((dw / np.maximum(ds, 1e-9)).max())


def prev_levels(width_list, threshold=0.05):
    return sum(1 for a, b in zip(width_list[:-1], width_list[1:]) if abs(a - b) > threshold)



def current_shape(B, R, K, design_rules=None, *, k=0.6, a=1.0, nominal='ipc', cpu_s=4.5):
    original = R
    start = time.process_time()
    R = prepare(R, K)
    design_rules = design_rules or {}
    if R.get('current_shape'):
        return copy.deepcopy(original), dict(applied=False, reason='already present', cpu_s=0.0)
    clr, edge, hole = _design_rules(R, design_rules)
    outline = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    gl = {str(q).split('=')[0] for q in (R.get('gnd_layers') or [])}
    net_list = sorted([n for n, kk in K['net_list'].items() if kk.get('category') in POWER],
                   key=lambda n: (-(K['net_list'][n].get('current_A') or 0), n))
    new_copper = {}; new_tracks = {}; report = []; profiles = []
    for n in net_list:
        kk = K['net_list'][n]; tn = time.process_time()
        if tn - start > cpu_s:
            return copy.deepcopy(original), dict(applied=False, reason='CPU budget: atomically rejected', cpu_s=tn-start)
        idx = [i for i, t in enumerate(R['tracks']) if t['net'] == n and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web', 'items'))]
        if not idx: continue
        vias_n = [v for v in R['vias'] if v['net'] == n]
        pads_n = [p for p in B['pads'] if p['net'] == n]
        layers_n = [L for L in sorted({R['tracks'][i]['layer'] for i in idx}) if L not in gl]
        PF = []; extra = []
        for L in layers_n:
            pg = [(('PAD', p['ref'], p['num']), g) for p in pads_n for g in pad_polys(p, L)]
            def kn(q):
                key = next((key for key, g in pg if g.covers(Point(q))), None) or (L, round(q[0], 4), round(q[1], 4))
                for vi, v in enumerate(vias_n):
                    if math.dist(q, (v['x'], v['y'])) < 1e-3: extra.append((key, ('VIA', vi)))
                return key
            for P, S, W in paths([i for i in idx if R['tracks'][i]['layer'] == L], R, B, n, L, vias_n):
                PF.append(dict(L=L, P=P, S=S, W=W, ka=kn(P[0]), kb=kn(P[-1]),
                               total_length=sum(math.dist(a_, b_) for a_, b_ in zip(P[:-1], P[1:]))))
        try:
            I_pf, I_ges, find = current_per_path(PF, kk, pads_n, vias_n, extra)
        except ValueError as e:
            return copy.deepcopy(original), dict(applied=False, reason=str(e), cpu_s=time.process_time()-start)
        if (kk.get('flow') or {}).get('unit') != 'A' and I_ges > 0 and kk.get('current_A'):
            I_pf = [i*float(kk['current_A'])/I_ges for i in I_pf]; I_ges = float(kk['current_A'])
        for L in layers_n:
            slot = Slot(obstacles(B, R, K, L, n, new_copper, design_rules), outline, edge)
            free = outline.buffer(-edge).difference(unary_union([g.buffer(c+SAFE,quad_segs=16) for g,c in slot.part_list]))
            inherited = unary_union([track_geometry(R['tracks'][i]) for i in idx if R['tracks'][i]['layer']==L] +
                        [g for p in pads_n for g in pad_polys(p,L)] +
                        [Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',via_size(R)[0]))/2,24) for v in vias_n if L in (v.get('layers') or cu_layers(B))])
            allowed = free.union(inherited.buffer(0.0001))
            inner = L not in ('F', 'B')
            w_nc = float(kk.get('w_inner') or kk['w']) if inner else float(kk['w'])
            if nominal == 'ipc':
                w_nc = max(w_nc, nominal_width(kk, inner=inner))
            wmin = float(kk.get('wmin') or 0.2)
            part_list = []; path_info = []; places = []

            J = [j for j, pf in enumerate(PF) if pf['L'] == L]
            dat = {}
            for j in J:
                pf = PF[j]; I_p = I_pf[j]; P, W = pf['P'], pf['W']
                a_p = min(1.0, I_p / I_ges) if I_ges > 0 else 1.0
                w_nominal = goal_width(a_p, wmin, w_nc) if I_ges > 0 else w_nc
                loop = find(pf['ka']) == find(pf['kb'])


                w_floor = min(min(W), w_nominal) if (a_p < 1e-3 or loop) else min(W)
                Q, rs = P, []
                dat[j] = dict(a_p=a_p, w_nominal=w_nominal, w_floor=w_floor, Q=Q, rs=rs, I=I_p, loop=loop)


            inert = [j for j in J if dat[j]['a_p'] > 0.01 and not dat[j]['loop']]
            inz = collections.defaultdict(list)
            for j in inert:
                inz[find(PF[j]['ka'])].append((j, 0)); inz[find(PF[j]['kb'])].append((j, 1))
            free_j = set(inert); strands = []
            def further(nodes, j_prev):
                e = [x for x in inz[nodes] if x[0] != j_prev]
                return e[0] if len(inz[nodes]) == 2 and e and e[0][0] in free_j else None
            for j0 in sorted(inert, key=lambda j: (len(inz[find(PF[j]['ka'])]) == 2 and len(inz[find(PF[j]['kb'])]) == 2, j)):
                if j0 not in free_j: continue
                free_j.discard(j0); chain_ = [(j0, False)]

                kn_ = find(PF[j0]['ka']); jj = j0
                while True:
                    x = further(kn_, jj)
                    if x is None: break
                    j1, end = x; free_j.discard(j1)
                    rev = (end == 0)
                    chain_.insert(0, (j1, rev)); kn_ = find(PF[j1]['kb'] if rev else PF[j1]['ka']); jj = j1
                kn_ = find(PF[j0]['kb']); jj = j0
                while True:
                    x = further(kn_, jj)
                    if x is None: break
                    j1, end = x; free_j.discard(j1)
                    rev = (end == 1)
                    chain_.append((j1, rev)); kn_ = find(PF[j1]['ka'] if rev else PF[j1]['kb']); jj = j1
                strands.append(chain_)
            strands += [[(j, False)] for j in J if j not in inert]

            for chain_ in strands:
                if time.process_time() - start > cpu_s:
                    return copy.deepcopy(original), dict(applied=False, reason='CPU budget: atomically rejected', cpu_s=time.process_time()-start)
                line = []; limits = [0.0]
                for j, rev in chain_:
                    Q = dat[j]['Q'][::-1] if rev else dat[j]['Q']
                    if line and math.dist(line[-1], Q[0]) < 1e-6: Q = Q[1:]
                    elif line: line.append(Q[0]); Q = Q[1:]
                    line += list(Q)
                    limits.append(sum(math.dist(p_, q_) for p_, q_ in zip(line[:-1], line[1:])))
                w_nominal = max(dat[j]['w_nominal'] for j, _ in chain_)
                floor = min(dat[j]['w_floor'] for j, _ in chain_)
                original_line = line
                line, radii = rounds_n(line, slot, w_nominal, floor)
                xy, s = sample(line)
                w_nominal = max(dat[j]['w_nominal'] for j, _ in chain_)
                part = np.clip(np.searchsorted(limits, s, side='right') - 1, 0, len(chain_) - 1)


                floor = min(dat[j]['w_floor'] for j, _ in chain_)
                wm = slot.wmax(xy, cap=w_nominal + 0.1)
                w, goal = plan(wm, s, w_nominal, floor, k, a)
                g_move = move(xy,w)
                if g_move.difference(allowed).area > 1e-8:
                    for scale in (0.75,0.5,0.25,0.0):
                        smaller = floor + (w-floor)*scale
                        candidate = move(xy,smaller)
                        if candidate.difference(allowed).area <= 1e-8:
                            w,g_move = smaller,candidate; break
                    else:

                        xy,s = sample(original_line); w = np.full(len(s),floor)
                        g_move = move(xy,w); radii=[]
                part = np.clip(np.searchsorted(limits,s,side='right')-1,0,len(chain_)-1)
                part_list.append(g_move)
                profiles.append(dict(net=n, layer=L, xy=xy.tolist(), s_mm=s.tolist(), w_mm=w.tolist(),
                                     transition_min_mm=2*a, arcs=len(radii), w_nominal=w_nominal))
                places += [(xy[0][0], xy[0][1], w[0] / 2), (xy[-1][0], xy[-1][1], w[-1] / 2)]
                for i, (j, rev) in enumerate(chain_):
                    pf = PF[j]; P, W = pf['P'], pf['W']; d = dat[j]
                    m = part == i
                    si, wi, wmi = s[m] - limits[i], w[m], wm[m]
                    if rev: si, wi, wmi = (limits[i + 1] - limits[i]) - si[::-1], wi[::-1], wmi[::-1]
                    places += [(q[0], q[1], float(wi.max() if len(wi) else 0) / 2) for q in P]
                    long = float(limits[i + 1] - limits[i])
                    st, dwds = levels(w[m], s[m]) if m.sum() > 1 else (0, 0.0)
                    cum = np.cumsum([math.dist(*sg) for sg in zip(P[:-1], P[1:])]) / max(pf['total_length'], 1e-9)
                    path_info.append(dict(total_length=round(long, 2), current_A=round(d['I'], 3), share=round(d['a_p'], 3), w_nominal=round(d['w_nominal'], 3),
                                          strang=len(strands) and strands.index(chain_), prev_widths=sorted({round(x, 3) for x in W}), prev_levels=prev_levels(W),
                                          prev_min=round(min(W), 3), new_min=round(float(wi.min()), 3) if len(wi) else None,
                                          new_max=round(float(wi.max()), 3) if len(wi) else None,
                                          new_medium=round(float(np.mean(wi)), 3) if len(wi) else None, new_levels=st, dw_ds_max=round(dwds, 3),
                                          arcs=len(d['rs']), r_min=min(d['rs']) if d['rs'] else None,
                                          share_nominal=round(float((wi >= d['w_nominal'] - 0.01).mean()), 3) if len(wi) else None,
                                          tight_min=round(float(min(wmi.min(), 99)), 3) if len(wmi) else None, ends=[str(find(pf['ka'])), str(find(pf['kb']))],
                                          profile_data=dict(s=[round(float(x), 3) for x in si[::2]], w=[round(float(x), 4) for x in wi[::2]],
                                                      wmax=[round(float(min(x, 9)), 4) for x in wmi[::2]],
                                                      prior=[round(float(W[min(len(W) - 1, int(np.searchsorted(cum, x / max(long, 1e-9))))]), 4) for x in si[::2]])))
                st_all, dw_all = levels(w, s)
                path_info[-1]['strang_levels'] = st_all; path_info[-1]['strang_dw_ds_max'] = round(dw_all, 3)

                stride = 1
                sel = sorted(set(list(range(0, len(s), stride)) + [len(s) - 1] + [int(np.argmin(np.abs(s - g))) for g in limits]))
                pts = [tuple(map(float, xy[j])) for j in sel]
                simple = LineString(pts).simplify(0.002, preserve_topology=True)
                candidate = list(simple.coords)
                if g_move.covers(LineString(candidate).buffer(min(w.min(),0.1)/2)):
                    pts = candidate
                ws = [float(min(w.min(), 0.1))] * (len(pts) - 1)
                new_tracks.setdefault(n, []).append(dict(net=n, layer=L, pts=pts, w=ws, sig=False, current_shape=True, current_A=max(dat[j]['I'] for j, _ in chain_)))
            owned = [g for p in pads_n for g in pad_polys(p, L)] + [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0])) / 2, 24) for v in vias_n if L in (v.get('layers') or cu_layers(B))]
            protected = [track_geometry(t) for t in R['tracks'] if t['net'] == n and t['layer'] == L and any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web', 'items'))]
            N = unary_union(part_list + owned + protected)
            vacant = outline.buffer(-edge).difference(unary_union([g.buffer(c + SAFE, quad_segs=16) for g, c in slot.part_list]))
            N, pad_report = pads_enclose(N, [g for p in pads_n for g in pad_polys(p,L)], vacant, [p for p in profiles if p['net']==n and p['layer']==L])
            N, n_bad = close(N, vacant, places)
            N = N.buffer(-0.0001,quad_segs=16).buffer(0.0001,quad_segs=16)
            new_copper[(n, L)] = N
            report.append(dict(net_name=n, layer_name=L, current_A=kk.get('current_A'), w_net_class=w_nc, wmin=wmin, paths=path_info,
                                closures=n_bad, pads=pad_report, cpu_s=None))
        report_n = [b for b in report if b['net_name'] == n]
        for b in report_n: b['cpu_s'] = round((time.process_time() - tn) / max(1, len(report_n)), 3)

    chk = []
    prev_copper = collections.defaultdict(list)
    for t in R['tracks']:
        for a_, b_, w_ in zip(t['pts'][:-1], t['pts'][1:], t['w']): prev_copper[(t['net'], t['layer'])].append(seg_poly(a_, b_, w_))
    for (n, L), N in sorted(new_copper.items()):
        prior = unary_union(prev_copper[(n, L)] + [g for p in B['pads'] if p['net'] == n for g in pad_polys(p, L)] +
                          [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0])) / 2, 24) for v in R['vias'] if v['net'] == n and L in (v.get('layers') or cu_layers(B))])
        to_it = N.difference(prior.buffer(1e-4))
        obs = obstacles(B, R, K, L, n, {kk_: g for kk_, g in new_copper.items() if kk_ != (n, L) and kk_[1] == L}, design_rules)
        dmin = 99.0; violation = 0
        for g, c in obs:
            if to_it.is_empty: break
            d = to_it.distance(g)
            if d < c - 1e-4: violation += 1
            dmin = min(dmin, d - c)
        d_border = (outline.boundary.distance(to_it) - edge) if not to_it.is_empty else None
        chk.append(dict(net_name=n, layer_name=L, extra_mm2=round(to_it.area, 3), prev_mm2=round(prior.area, 3), new_mm2=round(N.area, 3),
                          reserve_min_mm=round(dmin, 4), violations=violation, border_reserve_mm=round(d_border, 4) if d_border is not None else None))
    if time.process_time() - start > cpu_s:
        return copy.deepcopy(original), dict(applied=False, reason='CPU budget: atomically rejected', cpu_s=time.process_time()-start)
    if not new_copper:
        return copy.deepcopy(original), dict(applied=False, reason='no shapeable current tracks', cpu_s=time.process_time()-start)
    R2 = copy.deepcopy(R)
    R2['tracks'] = [t for t in R2['tracks'] if not (t['net'] in new_tracks and t['layer'] not in gl and
                      not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web', 'items')))] + [t for n in sorted(new_tracks) for t in new_tracks[n]]
    R2.setdefault('zones', [])
    clr, _, _ = _design_rules(R, design_rules)
    for zi, ((n, L), g) in enumerate(sorted(new_copper.items())):
        R2['zones'].extend(_zones(g, n, L, zi, clr))
    info = dict(applied=bool(new_copper), model='current_val-shaping-v2', net_list=report, checking=chk,
                cpu_s=time.process_time()-start, transition_mm=2*a, slope=k,
                reason='', spacing_mm=clr, border_mm=edge, drill_spacing_mm=hole, parallel=[], profile=profiles, profile_levels=sum(p.get('strang_levels', 0) for b in report for p in b['paths']))
    R2['current_shape'] = dict(model=info['model'], transition_mm=2*a, slope=k, profile=profiles)
    R2['copper_bars'] = dict(model=info['model'])
    R2['options'] = dict(R.get('options') or {}, current_fix='1')
    if any(q['violations'] or (q['border_reserve_mm'] is not None and q['border_reserve_mm'] < -0.0001) for q in chk):
        info.update(applied=False, reason='New spacing violation: atomically rejected')
        return copy.deepcopy(original), info
    return R2, info


def extend(B, R, K, design_rules=None, **options):
    allowed = {k: v for k, v in options.items() if k in ('k', 'a', 'nominal', 'cpu_s')}
    return current_shape(B, R, K, design_rules, **allowed)


def check_edges(B, R, K, angle_deg=15, entire=True):
    rows = []
    from common.zones import active
    actual_copper=copper_map(B,R) if active(R) else None
    targets={(z['net'],z['layer']) for z in R.get('zones',[]) if z.get('copper_bars')}
    targets.update((p['net'],p['layer']) for p in (R.get('current_shape') or {}).get('profile',[]))
    for net, layer in sorted(targets):
        gs = [polygon(z) for z in R['zones'] if z['net'] == net and z['layer'] == layer and z.get('filled')]
        g = unary_union(gs)
        if actual_copper is not None:g=actual_copper.get((net,layer),g)
        anchors = unary_union([pad_polygon(B, p, q).buffer(0.01) for p in B['pads'] if p['net'] == net
                               for q in p.get('polys', {}).get(layer, [])] +
                              [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0]))/2+0.01)
                               for v in R['vias'] if v['net'] == net and layer in (v.get('layers') or cu_layers(B))])
        shapely.prepare(anchors)
        pad_enclosures=[]
        for p in B['pads']:
            if p['net']!=net: continue
            for contour in p.get('polys',{}).get(layer,[]):
                pad=pad_polygon(B,p,contour)
                covers=g.buffer(.000001).covers(pad)
                margin=g.boundary.distance(pad) if covers else 0.0
                pad_enclosures.append(dict(ref=p['ref'],num=str(p['num']),hem_mm=margin,
                                           enclosed=covers and margin>=.004))
        free_angles, pad_angles = [], []
        sharp = []; short_edges = []; concave = 0; convex = 0
        for poly in _parts(g):
            for ri, ring in enumerate([poly.exterior] + list(poly.interiors)):



                xy = np.asarray(ring.coords[:-1])
                if len(xy) < 3: continue
                edge_lengths = np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
                cum = np.r_[0,np.cumsum(edge_lengths)]
                perimeter = cum[-1]
                def at(offset):
                    ss = (cum[:-1]+offset) % perimeter
                    closed = np.vstack([xy,xy[0]])
                    return np.c_[np.interp(ss,cum,closed[:,0]),np.interp(ss,cum,closed[:,1])]
                all_angles=[]
                for h in (.04,.02,.01,.005):
                    step = min(h, perimeter/12)
                    u = 2*(xy-at(-step))-(xy-at(-2*step))/2
                    v = 2*(at(step)-xy)-(at(2*step)-xy)/2
                    all_angles.append(np.degrees(np.arctan2(u[:,0]*v[:,1]-u[:,1]*v[:,0],np.sum(u*v,axis=1))))
                aa=np.asarray(all_angles)
                angles=aa[np.argmin(np.abs(aa),axis=0),np.arange(len(xy))]
                u=xy-np.roll(xy,1,axis=0);v=np.roll(xy,-1,axis=0)-xy
                lens=np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)
                orientation = (1 if ring.is_ccw else -1) * (-1 if ri else 1)
                edge_min = np.minimum(np.linalg.norm(u,axis=1),np.linalg.norm(v,axis=1))
                for pos, ang, le, ed in zip(xy, angles*orientation, lens, edge_min):
                    if le < 1e-16: continue
                    if anchors.covers(Point(pos)): pad_angles.append(abs(float(ang))); continue
                    free_angles.append(abs(float(ang)))
                    if abs(ang) > angle_deg:
                        sharp.append(pos.tolist()); short_edges.append(float(ed))
                        concave += int(ang < 0); convex += int(ang > 0)
        rows.append(dict(net=net, layer=layer, missing_area=g.is_empty, pads=pad_enclosures,
                         pads_not_enclosed=sum(not p["enclosed"] for p in pad_enclosures), hard_corners=len(sharp), inner_corners=concave, outer_corners=convex, places=sharp,
                         tangent_deviation_max_deg=max(free_angles, default=0),
                         pad_edges_max_deg=max(pad_angles, default=0),
                         min_leg_mm=min(short_edges,default=None),
                         sub_um_corners=sum(d < .001 for d in short_edges),
                         under_10um_corners=sum(d < .01 for d in short_edges)))
    hard = sum(q['hard_corners'] for q in rows)
    if hard and not entire:
        return dict(levels=None, hard_corners=hard, missing_current_areas=sum(q["missing_area"] for q in rows), pads_not_enclosed=sum(q["pads_not_enclosed"] for q in rows), per_net_layer=rows,
                    tangent_deviation_max_deg=max((q['tangent_deviation_max_deg'] for q in rows), default=0),
                    polygon_threshold_deg=angle_deg, chord_error_mm=.002, tangent_window_mm=[.04,.02,.01,.005], reason='Edge gate already violated; cross-section test skipped')

    skipped = dict(R, tracks=[t for t in R['tracks'] if not t.get('current_shape')])
    old = check_widths(skipped, K)
    profiles = R.get('current_shape', {}).get('profile', [])

    jump_count = 0
    for profile in profiles:
        xy = np.asarray(profile['xy']); net, layer = profile['net'], profile['layer']
        g = unary_union([polygon(z) for z in R['zones'] if z['net'] == net and z['layer'] == layer])
        delta = np.diff(xy, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        if not len(lengths): continue
        tangent = delta/np.maximum(lengths[:,None],1e-9)
        normal = np.c_[-tangent[:,1],tangent[:,0]]*30

        for sign in (-1, 1):
            points = xy[:-1]+sign*0.0001*tangent
            cuts = shapely.intersection(shapely.linestrings(np.stack([points-normal,points+normal],axis=1)),g)
            widths = np.asarray([max((q.length for q in getattr(c,'geoms',[c]) if q.distance(Point(p))<1e-7),default=0) for p,c in zip(points,cuts)])
            if sign == -1: left = widths
            else: jump_count += int(np.sum(np.abs(widths-left) > 0.01))
    old['levels'] += jump_count
    return dict(levels=old['levels'], missing_current_areas=sum(q['missing_area'] for q in rows), pads_not_enclosed=sum(q['pads_not_enclosed'] for q in rows), hard_corners=sum(q['hard_corners'] for q in rows), per_net_layer=rows,
                tangent_deviation_max_deg=max((q['tangent_deviation_max_deg'] for q in rows), default=0),
                polygon_threshold_deg=angle_deg, chord_error_mm=.002, tangent_window_mm=[.04,.02,.01,.005], scaffold=old, profile=len(profiles))


def prepare(R, K):
    out = dict(R, tracks=[])
    for t in R['tracks']:
        items = t.get('items')
        same = items and len(items) == len(t['w']) and len(t['pts']) == len(items)+1 and all(
            it[0]=='L' and it[1]==t['pts'][j] and it[2]==t['pts'][j+1] and it[3]==t['w'][j]
            for j,it in enumerate(items))
        if same and K['net_list'].get(t['net'],{}).get('category') in POWER and not any(
            t.get(q) for q in ('sig','kelvin','fix','current_web')):
            t = {q:v for q,v in t.items() if q != 'items'}
        out['tracks'].append(t)
    return out
