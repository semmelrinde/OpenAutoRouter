""
import copy
import math
import time
from collections import defaultdict

import numpy as np
import shapely
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

POWER = ('strom', 'versorgung', 'sw')
TAG = 'strom_form_v2'


def polygon(z):
    return Polygon(z['pts'], z.get('holes', [])).buffer(0)


def track_geometry(t):
    ""
    from gemeinsam.bogen import punkte
    if 'items' in t:
        gs = []
        for it in t['items']:
            if it[0] == 'L':
                ps, w = it[1:3], it[3]
            elif it[0] == 'A':
                ps, w = punkte(*it[1:4]), it[4]
            else:
                raise ValueError('Unbekanntes Bahnitem: ' + str(it[0]))
            gs.append(LineString(ps).buffer(w / 2 + (0.00025 if it[0] == 'A' else 0), quad_segs=16))
        return unary_union(gs)
    return unary_union([LineString([a, b]).buffer(w / 2, quad_segs=16)
                        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']) if a != b])


def lagen(B):
    return ['F'] + ['In%d' % i for i in range(1, int(B.get('lagen') or 2) - 1)] + ['B']


def via_size(R):
    return tuple(map(float, str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')))


def pad_polygon(B, p, q):
    ""
    xy = np.asarray(q)
    if str(B.get('src', '')).startswith('aufgabe seed=') and len(xy) == 12:
        rr = np.linalg.norm(xy - [p['x'], p['y']], axis=1)
        if np.max(rr) - np.min(rr) < 0.00003:
            return Point(p['x'], p['y']).buffer(float(np.max(rr)), quad_segs=64)
    return Polygon(q).buffer(0)


def kupfer(B, R, *, ungefuellt='fehler'):
    ""
    from gemeinsam.zonen import vereinheitlichen, boardgeometrie
    B = boardgeometrie(B, R)
    R = vereinheitlichen(R)
    parts = defaultdict(list)
    ls = lagen(B)
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
            if ungefuellt == 'ignorieren': continue
            raise ValueError('Ungefuellte Zone: vor Erweiterung nativ fuellen/exportieren')
        parts[z['net'], z['layer']].append(polygon(z))
    return {k: unary_union(v) for k, v in parts.items()}


def _regeln(R, regeln):

    rg = (regeln or {}).get('router', regeln or {})
    op = R.get('optionen') or {}
    roh = (regeln or {}).get('roh') or {}
    def minimum(entries, fallback):
        vals = [float(source[key]) for source, key in entries if source.get(key) is not None]
        return max(vals) if vals else fallback
    clr = minimum([(R,'clr'),(roh,'min_clearance')] + [(source,key) for source in (rg,op)
                   for key in ('clr','drc','drcpad')], .1)
    edge = minimum([(roh,'min_copper_edge_clearance'),(rg,'edge'),(op,'edge')], .25)
    hole = minimum([(roh,'min_hole_clearance'),(rg,'h2h'),(op,'h2h')], .2)
    return clr, edge, hole


def _sperren(B, ls):
    out = defaultdict(list)
    for k in B.get('keepouts', []):
        vb = k.get('verbote') or {}
        if not vb.get('flaechen', vb.get('bahnen', True)):
            continue
        for l in k.get('lagen') or ls:
            out[l] += [Polygon(p).buffer(0) for p in k.get('polys', []) if len(p) >= 3]
    return {l: unary_union(g) for l, g in out.items()}


def _frei(B, R, cu, net, layer, outline, blocked, clr, hole):
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


def _teile(g):
    return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]


def _inseln_ok(outline, old, new, min_gap=0.15):
    ""
    a = outline.difference(old).buffer(-min_gap / 2)
    b = outline.difference(new).buffer(-min_gap / 2)
    before, after = _teile(a), _teile(b)
    edge = outline.buffer(-min_gap / 2).boundary
    for p in before:
        connected = [q for q in after if q.intersection(p).area > 0.002]
        if len(connected) > 1:
            return False
        if connected and p.distance(edge) < 1e-6 and connected[0].distance(edge) > 1e-6:
            return False
    return True


def _zonen(g, net, layer, ti, clr=0.1):
    return [dict(net=net, layer=layer, pts=[list(p) for p in q.exterior.coords[:-1]],
                 holes=[[list(p) for p in ring.coords[:-1]] for ring in q.interiors], prio=1000 + ti,
                 filled=True, estimated=True, full=True, min_thickness=0.05, local_clearance=clr,
                 strompfad=True, kupferbalken=TAG, track=ti)
            for q in _teile(g) if q.area > 0.001]


def pruefe_breiten(R, K, h=0.1):
    ""
    out = []
    for ti, t in enumerate(R['tracks']):
        if K['netze'].get(t['net'], {}).get('art') not in POWER or t.get('sig') or t.get('kelvin'):
            continue
        g = unary_union([track_geometry(t)] + [polygon(z) for z in R.get('zones', [])
                        if z.get('layer') == t['layer'] and z.get('net') == t['net']])
        rings = [np.asarray(r.coords) for q in _teile(g) for r in [q.exterior] + list(q.interiors)]
        rate, steps = 0.0, set()

        for poly in _teile(g):
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
        out.append(dict(track=ti, net=t['net'], layer=t['layer'], max_dw_pro_mm=rate, stufen=len(steps)))
    return dict(je_bahn=out, max_dw_pro_mm=max((q['max_dw_pro_mm'] for q in out), default=0),
                stufen=sum(q['stufen'] for q in out), abtast_mm=h, sprungschwelle_mm=0.01)


import collections
from shapely.geometry import box
from gemeinsam.fluss import zielbreite
from gemeinsam.strombahn import sollbreite

H = 0.05
SICHER = 0.005


def args_kv(argv):
    return dict(a.split('=', 1) for a in argv if '=' in a)


def lagen_von(B):
    n = int(B.get('lagen') or 2)
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def seg_poly(a, b, w):
    return (LineString([a, b]) if math.dist(a, b) > 1e-9 else Point(a)).buffer(w / 2, quad_segs=16)


def pad_polys(p, L):
    return [Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]



class Platz:
    ""
    def __init__(self, geoms_clr, outline, edge):
        self.teile = [(g, c) for g, c in geoms_clr if not g.is_empty]
        self.tree = shapely.STRtree([g for g, _ in self.teile]) if self.teile else None
        self.clr = np.array([c for _, c in self.teile]) if self.teile else np.zeros(0)
        self.rand = outline.boundary; self.outline = outline; self.edge = edge
        self.cmax = float(self.clr.max()) if len(self.clr) else 0.0

    def wmax(self, xy, cap=50.0):
        ""
        pts = shapely.points(xy)
        w = 2 * (shapely.distance(pts, self.rand) - self.edge - SICHER)
        w = np.where(shapely.contains_xy(self.outline, xy[:, 0], xy[:, 1]), w, -1.0)
        w = np.minimum(w, cap)
        if self.tree is None: return w

        r = cap / 2 + self.cmax + SICHER
        pi, gi = self.tree.query(pts, predicate='dwithin', distance=r)
        if len(pi):
            d = shapely.distance(pts[pi], np.array([g for g, _ in self.teile], dtype=object)[gi])
            ww = 2 * (d - self.clr[gi] - SICHER)
            np.minimum.at(w, pi, ww)
        return w


def hindernisse(B, R, K, L, netz, neu_kupfer, regeln):
    ""
    c_bahn, _, c_loch = _regeln(R, regeln)
    c_pad = c_bahn
    d_via = float(str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])
    out = []
    for p in B['pads']:
        if p['net'] != netz:
            for g in pad_polys(p, L): out.append((g, c_pad))
        if p.get('drill') and (p['net'] != netz or not p.get('polys', {}).get(L)):
            out.append((Point(p['x'], p['y']).buffer(p['drill'] / 2, 24), c_loch))
    for t in R['tracks']:
        if t['layer'] != L: continue
        if t['net'] == netz and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg', 'items')): continue
        if (t['net'], L) in neu_kupfer and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg', 'items')): continue
        out.append((track_geometry(t), c_bahn))
    for v in R['vias']:
        if v['net'] != netz and L in (v.get('layers') or lagen(B)) and (v['net'], L) not in neu_kupfer:
            out.append((Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', d_via)) / 2, 24), c_bahn))
    _, drill = via_size(R)
    for v in R['vias']:
        if v['net'] != netz:
            out.append((Point(v['x'], v['y']).buffer(v.get('drill', drill) / 2, 24), c_loch))
    for z in R.get('zones', []):
        if z['layer'] == L and z['net'] != netz and z.get('filled'):
            out.append((polygon(z), c_bahn))
    for (n, LL), g in neu_kupfer.items():
        if LL == L and n != netz: out.append((g, c_bahn))
    for k in B.get('keepouts', []):
        vb = k.get('verbote') or {}
        if vb.get('flaechen', vb.get('bahnen', True)) and L in (k.get('lagen') or [L]):
            for q in k.get('polys', []):
                if len(q) >= 3: out.append((Polygon(q).buffer(0), 0.0))
    return out



def pfade(tracks_idx, R, B, netz, L, vias):
    ""
    segs = []
    for ti in tracks_idx:
        t = R['tracks'][ti]
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            if math.dist(a, b) > 1e-6: segs.append([tuple(a), tuple(b), ti, si, w])

    marken = [tuple(q) for ti in tracks_idx for q in R['tracks'][ti]['pts']] + [(v['x'], v['y']) for v in vias]
    neu = []
    for a, b, ti, si, w in segs:
        ab = np.subtract(b, a); l2 = float(ab @ ab); teil = [0.0, 1.0]
        for q in marken:
            s = float(np.subtract(q, a) @ ab) / l2
            if 1e-6 < s < 1 - 1e-6 and math.dist(q, (a[0] + s * ab[0], a[1] + s * ab[1])) < 1e-4: teil.append(s)
        teil = sorted(set(teil))
        for s1, s2 in zip(teil[:-1], teil[1:]):
            neu.append(((a[0] + s1 * ab[0], a[1] + s1 * ab[1]), (a[0] + s2 * ab[0], a[1] + s2 * ab[1]), ti, si, w))
    key = lambda q: (round(q[0], 4), round(q[1], 4))
    adj = collections.defaultdict(list)
    for i, (a, b, ti, si, w) in enumerate(neu):
        adj[key(a)].append((i, key(b))); adj[key(b)].append((i, key(a)))
    padg = unary_union([g for p in B['pads'] if p['net'] == netz for g in pad_polys(p, L)])
    viak = {key((v['x'], v['y'])) for v in vias}
    def knoten(k):
        return len(adj[k]) != 2 or k in viak or (not padg.is_empty and padg.contains(Point(k)))
    benutzt = set(); out = []
    starts = [k for k in sorted(adj) if knoten(k)] + sorted(adj)
    for k0 in starts:
        for i0, k1 in sorted(adj[k0]):
            if i0 in benutzt: continue
            P = [k0]; S = []; i, k = i0, k1
            while True:
                benutzt.add(i); P.append(k); S.append(i)
                if knoten(k): break
                nxt = [(j, m) for j, m in adj[k] if j not in benutzt]
                if not nxt: break
                i, k = nxt[0]
            pts = [neu[S[0]][0] if key(neu[S[0]][0]) == P[0] else neu[S[0]][1]]
            for j, kk in zip(S, P[1:]):
                a, b = neu[j][0], neu[j][1]
                pts.append(b if key(b) == kk else a)
            out.append((pts, [(neu[j][2], neu[j][3]) for j in S], [neu[j][4] for j in S]))
    return out


def vereinfachen(P):
    ""
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


def bogen(a, b, c, r):
    ""
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


def abtasten(P, h=H):
    P = np.asarray(P, float); d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    n = max(2, int(math.ceil(d[-1] / h)) + 1); s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])], s


def runden(P, platz, w_soll, w_boden, radien=(3.0, 2.0, 1.5, 1.0, 0.7, 0.5, 0.35, 0.25, 0.15, 0.1)):
    ""
    P = vereinfachen(P)
    if len(P) < 3: return P, []
    L = [math.dist(P[i], P[i + 1]) for i in range(len(P) - 1)]
    out = [np.asarray(P[0])]; rs = []
    frei_vorne = L[0]
    for i in range(1, len(P) - 1):
        hinten = L[i] if i == len(P) - 2 else L[i] / 2
        alt, _ = abtasten([P[i - 1], P[i], P[i + 1]], 0.05)
        cap_alt = min(w_soll, float(platz.wmax(alt, cap=w_soll + 0.1).min()))
        gew = None
        for r in radien:
            b = bogen(P[i - 1], P[i], P[i + 1], r)
            if b is None: break
            T, pts = b
            if T > frei_vorne - 1e-6 or T > hinten - 1e-6: continue
            wm = platz.wmax(np.asarray(pts), cap=w_soll + 0.1)
            if wm.min() >= max(w_boden, cap_alt): gew = (r, pts, T); break
        if gew is None:
            out.append(np.asarray(P[i])); frei_vorne = L[i]
        else:
            r, pts, T = gew; out.extend(pts); rs.append(r); frei_vorne = L[i] - T
    out.append(np.asarray(P[-1]))
    return [tuple(map(float, q)) for q in out], rs


def glatt(x, n):
    ""
    if n < 1: return x
    xp = np.r_[np.full(n, x[0]), x, np.full(n, x[-1])]; c = np.cumsum(np.r_[0, xp])
    return (c[2 * n + 1:] - c[:-2 * n - 1]) / (2 * n + 1)


def plan(wm, s, w_soll, w_boden, k, a):
    ""
    ziel = np.minimum(w_soll, wm)
    floor = float(w_boden)
    ziel = np.maximum(floor, ziel)
    ds = float(s[1]-s[0]); n = max(1, int(round(a/2/ds)))
    t = ziel.copy()
    for _ in range(12):
        e = t.copy()
        for j in range(1,len(e)): e[j] = min(e[j],e[j-1]+k*ds)
        for j in range(len(e)-2,-1,-1): e[j] = min(e[j],e[j+1]+k*ds)
        r = glatt(glatt(e,n),n)
        over = r-ziel
        if float(over.max()) <= 1e-5: break
        t -= np.maximum(over,0)*1.5 + (over>1e-5)*1e-4

    r += max(0.0,floor-float(r.min()))
    spline = CubicSpline(s,r,bc_type=((1,0.0),(1,0.0)))
    return spline(s),ziel


def zug(xy, w):
    ""
    d = shapely.buffer(shapely.points(xy), w / 2, quad_segs=16)
    return shapely.union_all(shapely.convex_hull(shapely.union(d[:-1], d[1:]))) if len(d) > 1 else d[0]


def schliessen(N, frei, orte, radien=(0.4, 0.25, 0.15, 0.08, 0.04)):
    ""
    n_ok = 0
    frei = frei.intersection(box(*N.bounds).buffer(1.0)); shapely.prepare(frei)
    for r in radien:
        fenster = unary_union([Point(x, y).buffer(h + 2.5 * r, 16) for x, y, h in orte]); shapely.prepare(fenster)
        C = N.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
        add = C.difference(N)
        ok = []
        for q in getattr(add, 'geoms', [add]):
            if q.is_empty or q.area < 2e-5: continue
            if q.area > max(0.02, 1.2 * r * r): continue
            qi = q.buffer(-1e-5)
            if frei.covers(qi) and fenster.covers(qi): ok.append(q)
        if ok:
            N = unary_union([N] + ok); n_ok += len(ok)
    return N, n_ok


def pads_umschliessen(N, pads, frei, profile, margin=0.12):
    ""
    report = []; anchors = []
    for pad in pads:
        shell = pad
        chosen = 0.0
        for gap in (margin, .08, .04, .02, .01, .005):
            candidate = pad.buffer(gap, quad_segs=24)
            if frei.covers(candidate.difference(pad.buffer(.0001))):
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
                if frei.covers(added.buffer(-.00001)):
                    N=N.union(hull);taper=max(taper,reach);break

        c = pad.centroid
        radius = max(pad.bounds[2]-pad.bounds[0], pad.bounds[3]-pad.bounds[1])/2
        anchors.append((c.x,c.y,radius))
        report.append(dict(pad_bbox=list(pad.bounds), saum_mm=chosen,
                           umschlossen=chosen > 0, anschlussfenster_mm=taper, grund='' if chosen else 'Kein freier vollstaendiger Padsaum'))
    if anchors:
        N, _ = schliessen(N, frei.union(unary_union(pads)), anchors, radien=(.7,.4,.2,.1,.05))
    return N, report


def strom_je_pfad(PF, kk, pads_n, vias_n, extra=()):
    ""
    fl = kk.get('fluss') or {}; quelle = set(fl.get('quelle') or []); last = fl.get('last') or {}
    par = {}
    def f(x):
        while par.get(x, x) != x: x = par.get(x, x)
        return x
    def u(a, b): par[f(a)] = f(b)
    for vi, v in enumerate(vias_n):
        for p in pads_n:
            if any(g.covers(Point(v['x'], v['y'])) for L in (p.get('polys') or {}) for g in pad_polys(p, L)): u(('VIA', vi), ('PAD', p['ref'], p['num']))
    for a_, b_ in extra: u(a_, b_)
    for q in quelle: u(('PAD',) + tuple(q.split(':')), 'Q')
    knoten = sorted({f(x) for pf in PF for x in (pf['ka'], pf['kb'])} | {'Q'}, key=str)
    ix = {k: i for i, k in enumerate(knoten)}; n = len(knoten)
    G = np.zeros((n, n)); g_ = []
    for pf in PF:
        a, b = ix[f(pf['ka'])], ix[f(pf['kb'])]; g = 1.0 / max(pf['laenge'], 0.01); g_.append((a, b, g))
        if a == b: continue
        G[a, a] += g; G[b, b] += g; G[a, b] -= g; G[b, a] -= g
    I = np.zeros(n); ges = 0.0
    for k, A in last.items():
        key = f(('PAD',) + tuple(k.split(':')))
        if A > 0 and k not in quelle:
            if key not in ix: raise ValueError('Last ohne gezielte Zuleitung: '+k)
            I[ix[key]] -= A; ges += A
    q = ix['Q']; m = [i for i in range(n) if i != q]
    V = np.zeros(n)
    if m and ges > 0:
        V[m] = np.linalg.lstsq(G[np.ix_(m, m)], I[m], rcond=None)[0]
        if np.max(np.abs(G[m] @ V-I[m])) > max(1e-8, ges*1e-8):
            raise ValueError('Stromgraph trennt Verbraucher von Quelle')
    return [g * abs(V[a] - V[b]) for a, b, g in g_], ges, f


def stufen(w, s, schwelle=0.05):
    ""
    if len(w) < 2: return 0, 0.0
    dw = np.abs(np.diff(w)); ds = np.diff(s)
    return int((dw > schwelle).sum()), float((dw / np.maximum(ds, 1e-9)).max())


def alt_stufen(breiten, schwelle=0.05):
    return sum(1 for a, b in zip(breiten[:-1], breiten[1:]) if abs(a - b) > schwelle)



def strom_form(B, R, K, regeln=None, *, k=0.6, a=1.0, soll='ipc', cpu_s=4.5):
    original = R
    start = time.process_time()
    R = vorbereiten(R, K)
    regeln = regeln or {}
    if R.get('strom_form'):
        return copy.deepcopy(original), dict(angewendet=False, grund='bereits vorhanden', cpu_s=0.0)
    clr, edge, hole = _regeln(R, regeln)
    outline = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    gl = {str(q).split('=')[0] for q in (R.get('gndlagen') or [])}
    netze = sorted([n for n, kk in K['netze'].items() if kk.get('art') in POWER],
                   key=lambda n: (-(K['netze'][n].get('strom_A') or 0), n))
    neu_kupfer = {}; neu_tracks = {}; bericht = []; profiles = []
    for n in netze:
        kk = K['netze'][n]; tn = time.process_time()
        if tn - start > cpu_s:
            return copy.deepcopy(original), dict(angewendet=False, grund='CPU-Budget: atomar verworfen', cpu_s=tn-start)
        idx = [i for i, t in enumerate(R['tracks']) if t['net'] == n and not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg', 'items'))]
        if not idx: continue
        vias_n = [v for v in R['vias'] if v['net'] == n]
        pads_n = [p for p in B['pads'] if p['net'] == n]
        lagen_n = [L for L in sorted({R['tracks'][i]['layer'] for i in idx}) if L not in gl]
        PF = []; extra = []
        for L in lagen_n:
            pg = [(('PAD', p['ref'], p['num']), g) for p in pads_n for g in pad_polys(p, L)]
            def kn(q):
                key = next((key for key, g in pg if g.covers(Point(q))), None) or (L, round(q[0], 4), round(q[1], 4))
                for vi, v in enumerate(vias_n):
                    if math.dist(q, (v['x'], v['y'])) < 1e-3: extra.append((key, ('VIA', vi)))
                return key
            for P, S, W in pfade([i for i in idx if R['tracks'][i]['layer'] == L], R, B, n, L, vias_n):
                PF.append(dict(L=L, P=P, S=S, W=W, ka=kn(P[0]), kb=kn(P[-1]),
                               laenge=sum(math.dist(a_, b_) for a_, b_ in zip(P[:-1], P[1:]))))
        try:
            I_pf, I_ges, fnd = strom_je_pfad(PF, kk, pads_n, vias_n, extra)
        except ValueError as e:
            return copy.deepcopy(original), dict(angewendet=False, grund=str(e), cpu_s=time.process_time()-start)
        if (kk.get('fluss') or {}).get('einheit') != 'A' and I_ges > 0 and kk.get('strom_A'):
            I_pf = [i*float(kk['strom_A'])/I_ges for i in I_pf]; I_ges = float(kk['strom_A'])
        for L in lagen_n:
            platz = Platz(hindernisse(B, R, K, L, n, neu_kupfer, regeln), outline, edge)
            free = outline.buffer(-edge).difference(unary_union([g.buffer(c+SICHER,quad_segs=16) for g,c in platz.teile]))
            inherited = unary_union([track_geometry(R['tracks'][i]) for i in idx if R['tracks'][i]['layer']==L] +
                        [g for p in pads_n for g in pad_polys(p,L)] +
                        [Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',via_size(R)[0]))/2,24) for v in vias_n if L in (v.get('layers') or lagen(B))])
            allowed = free.union(inherited.buffer(0.0001))
            innen = L not in ('F', 'B')
            w_kl = float(kk.get('w_innen') or kk['w']) if innen else float(kk['w'])
            if soll == 'ipc':
                w_kl = max(w_kl, sollbreite(kk, innen=innen))
            wmin = float(kk.get('wmin') or 0.2)
            teile = []; pfad_info = []; orte = []

            J = [j for j, pf in enumerate(PF) if pf['L'] == L]
            dat = {}
            for j in J:
                pf = PF[j]; I_p = I_pf[j]; P, W = pf['P'], pf['W']
                a_p = min(1.0, I_p / I_ges) if I_ges > 0 else 1.0
                w_soll = zielbreite(a_p, wmin, w_kl) if I_ges > 0 else w_kl
                schleife = fnd(pf['ka']) == fnd(pf['kb'])


                w_boden = min(min(W), w_soll) if (a_p < 1e-3 or schleife) else min(W)
                Q, rs = P, []
                dat[j] = dict(a_p=a_p, w_soll=w_soll, w_boden=w_boden, Q=Q, rs=rs, I=I_p, schleife=schleife)


            traeg = [j for j in J if dat[j]['a_p'] > 0.01 and not dat[j]['schleife']]
            inz = collections.defaultdict(list)
            for j in traeg:
                inz[fnd(PF[j]['ka'])].append((j, 0)); inz[fnd(PF[j]['kb'])].append((j, 1))
            frei_j = set(traeg); straenge = []
            def weiter(knoten, j_alt):
                e = [x for x in inz[knoten] if x[0] != j_alt]
                return e[0] if len(inz[knoten]) == 2 and e and e[0][0] in frei_j else None
            for j0 in sorted(traeg, key=lambda j: (len(inz[fnd(PF[j]['ka'])]) == 2 and len(inz[fnd(PF[j]['kb'])]) == 2, j)):
                if j0 not in frei_j: continue
                frei_j.discard(j0); kette_ = [(j0, False)]

                kn_ = fnd(PF[j0]['ka']); jj = j0
                while True:
                    x = weiter(kn_, jj)
                    if x is None: break
                    j1, ende = x; frei_j.discard(j1)
                    rev = (ende == 0)
                    kette_.insert(0, (j1, rev)); kn_ = fnd(PF[j1]['kb'] if rev else PF[j1]['ka']); jj = j1
                kn_ = fnd(PF[j0]['kb']); jj = j0
                while True:
                    x = weiter(kn_, jj)
                    if x is None: break
                    j1, ende = x; frei_j.discard(j1)
                    rev = (ende == 1)
                    kette_.append((j1, rev)); kn_ = fnd(PF[j1]['ka'] if rev else PF[j1]['kb']); jj = j1
                straenge.append(kette_)
            straenge += [[(j, False)] for j in J if j not in traeg]

            for kette_ in straenge:
                if time.process_time() - start > cpu_s:
                    return copy.deepcopy(original), dict(angewendet=False, grund='CPU-Budget: atomar verworfen', cpu_s=time.process_time()-start)
                linie = []; grenzen = [0.0]
                for j, rev in kette_:
                    Q = dat[j]['Q'][::-1] if rev else dat[j]['Q']
                    if linie and math.dist(linie[-1], Q[0]) < 1e-6: Q = Q[1:]
                    elif linie: linie.append(Q[0]); Q = Q[1:]
                    linie += list(Q)
                    grenzen.append(sum(math.dist(p_, q_) for p_, q_ in zip(linie[:-1], linie[1:])))
                w_soll = max(dat[j]['w_soll'] for j, _ in kette_)
                boden = min(dat[j]['w_boden'] for j, _ in kette_)
                original_line = linie
                linie, radien = runden(linie, platz, w_soll, boden)
                xy, s = abtasten(linie)
                w_soll = max(dat[j]['w_soll'] for j, _ in kette_)
                teil = np.clip(np.searchsorted(grenzen, s, side='right') - 1, 0, len(kette_) - 1)


                boden = min(dat[j]['w_boden'] for j, _ in kette_)
                wm = platz.wmax(xy, cap=w_soll + 0.1)
                w, ziel = plan(wm, s, w_soll, boden, k, a)
                g_zug = zug(xy,w)
                if g_zug.difference(allowed).area > 1e-8:
                    for scale in (0.75,0.5,0.25,0.0):
                        smaller = boden + (w-boden)*scale
                        candidate = zug(xy,smaller)
                        if candidate.difference(allowed).area <= 1e-8:
                            w,g_zug = smaller,candidate; break
                    else:

                        xy,s = abtasten(original_line); w = np.full(len(s),boden)
                        g_zug = zug(xy,w); radien=[]
                teil = np.clip(np.searchsorted(grenzen,s,side='right')-1,0,len(kette_)-1)
                teile.append(g_zug)
                profiles.append(dict(net=n, layer=L, xy=xy.tolist(), s_mm=s.tolist(), w_mm=w.tolist(),
                                     uebergang_min_mm=2*a, boegen=len(radien), w_soll=w_soll))
                orte += [(xy[0][0], xy[0][1], w[0] / 2), (xy[-1][0], xy[-1][1], w[-1] / 2)]
                for i, (j, rev) in enumerate(kette_):
                    pf = PF[j]; P, W = pf['P'], pf['W']; d = dat[j]
                    m = teil == i
                    si, wi, wmi = s[m] - grenzen[i], w[m], wm[m]
                    if rev: si, wi, wmi = (grenzen[i + 1] - grenzen[i]) - si[::-1], wi[::-1], wmi[::-1]
                    orte += [(q[0], q[1], float(wi.max() if len(wi) else 0) / 2) for q in P]
                    lang = float(grenzen[i + 1] - grenzen[i])
                    st, dwds = stufen(w[m], s[m]) if m.sum() > 1 else (0, 0.0)
                    cum = np.cumsum([math.dist(*sg) for sg in zip(P[:-1], P[1:])]) / max(pf['laenge'], 1e-9)
                    pfad_info.append(dict(laenge=round(lang, 2), strom_A=round(d['I'], 3), anteil=round(d['a_p'], 3), w_soll=round(d['w_soll'], 3),
                                          strang=len(straenge) and straenge.index(kette_), alt_breiten=sorted({round(x, 3) for x in W}), alt_stufen=alt_stufen(W),
                                          alt_min=round(min(W), 3), neu_min=round(float(wi.min()), 3) if len(wi) else None,
                                          neu_max=round(float(wi.max()), 3) if len(wi) else None,
                                          neu_mittel=round(float(np.mean(wi)), 3) if len(wi) else None, neu_stufen=st, dw_ds_max=round(dwds, 3),
                                          boegen=len(d['rs']), r_min=min(d['rs']) if d['rs'] else None,
                                          anteil_soll=round(float((wi >= d['w_soll'] - 0.01).mean()), 3) if len(wi) else None,
                                          eng_min=round(float(min(wmi.min(), 99)), 3) if len(wmi) else None, enden=[str(fnd(pf['ka'])), str(fnd(pf['kb']))],
                                          profil=dict(s=[round(float(x), 3) for x in si[::2]], w=[round(float(x), 4) for x in wi[::2]],
                                                      wmax=[round(float(min(x, 9)), 4) for x in wmi[::2]],
                                                      alt=[round(float(W[min(len(W) - 1, int(np.searchsorted(cum, x / max(lang, 1e-9))))]), 4) for x in si[::2]])))
                st_all, dw_all = stufen(w, s)
                pfad_info[-1]['strang_stufen'] = st_all; pfad_info[-1]['strang_dw_ds_max'] = round(dw_all, 3)

                stride = 1
                sel = sorted(set(list(range(0, len(s), stride)) + [len(s) - 1] + [int(np.argmin(np.abs(s - g))) for g in grenzen]))
                pts = [tuple(map(float, xy[j])) for j in sel]
                simple = LineString(pts).simplify(0.002, preserve_topology=True)
                candidate = list(simple.coords)
                if g_zug.covers(LineString(candidate).buffer(min(w.min(),0.1)/2)):
                    pts = candidate
                ws = [float(min(w.min(), 0.1))] * (len(pts) - 1)
                neu_tracks.setdefault(n, []).append(dict(net=n, layer=L, pts=pts, w=ws, sig=False, strom_form=True, strom_A=max(dat[j]['I'] for j, _ in kette_)))
            eigen = [g for p in pads_n for g in pad_polys(p, L)] + [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0])) / 2, 24) for v in vias_n if L in (v.get('layers') or lagen(B))]
            protected = [track_geometry(t) for t in R['tracks'] if t['net'] == n and t['layer'] == L and any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg', 'items'))]
            N = unary_union(teile + eigen + protected)
            frei = outline.buffer(-edge).difference(unary_union([g.buffer(c + SICHER, quad_segs=16) for g, c in platz.teile]))
            N, pad_report = pads_umschliessen(N, [g for p in pads_n for g in pad_polys(p,L)], frei, [p for p in profiles if p['net']==n and p['layer']==L])
            N, n_schl = schliessen(N, frei, orte)
            N = N.buffer(-0.0001,quad_segs=16).buffer(0.0001,quad_segs=16)
            neu_kupfer[(n, L)] = N
            bericht.append(dict(netz=n, lage=L, strom_A=kk.get('strom_A'), w_klasse=w_kl, wmin=wmin, pfade=pfad_info,
                                schliessungen=n_schl, pads=pad_report, cpu_s=None))
        bericht_n = [b for b in bericht if b['netz'] == n]
        for b in bericht_n: b['cpu_s'] = round((time.process_time() - tn) / max(1, len(bericht_n)), 3)

    pruef = []
    alt_kupfer = collections.defaultdict(list)
    for t in R['tracks']:
        for a_, b_, w_ in zip(t['pts'][:-1], t['pts'][1:], t['w']): alt_kupfer[(t['net'], t['layer'])].append(seg_poly(a_, b_, w_))
    for (n, L), N in sorted(neu_kupfer.items()):
        alt = unary_union(alt_kupfer[(n, L)] + [g for p in B['pads'] if p['net'] == n for g in pad_polys(p, L)] +
                          [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0])) / 2, 24) for v in R['vias'] if v['net'] == n and L in (v.get('layers') or lagen(B))])
        dazu = N.difference(alt.buffer(1e-4))
        obs = hindernisse(B, R, K, L, n, {kk_: g for kk_, g in neu_kupfer.items() if kk_ != (n, L) and kk_[1] == L}, regeln)
        dmin = 99.0; verletz = 0
        for g, c in obs:
            if dazu.is_empty: break
            d = dazu.distance(g)
            if d < c - 1e-4: verletz += 1
            dmin = min(dmin, d - c)
        d_rand = (outline.boundary.distance(dazu) - edge) if not dazu.is_empty else None
        pruef.append(dict(netz=n, lage=L, zusatz_mm2=round(dazu.area, 3), alt_mm2=round(alt.area, 3), neu_mm2=round(N.area, 3),
                          reserve_min_mm=round(dmin, 4), verletzungen=verletz, rand_reserve_mm=round(d_rand, 4) if d_rand is not None else None))
    if time.process_time() - start > cpu_s:
        return copy.deepcopy(original), dict(angewendet=False, grund='CPU-Budget: atomar verworfen', cpu_s=time.process_time()-start)
    if not neu_kupfer:
        return copy.deepcopy(original), dict(angewendet=False, grund='keine formbaren Strombahnen', cpu_s=time.process_time()-start)
    R2 = copy.deepcopy(R)
    R2['tracks'] = [t for t in R2['tracks'] if not (t['net'] in neu_tracks and t['layer'] not in gl and
                      not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg', 'items')))] + [t for n in sorted(neu_tracks) for t in neu_tracks[n]]
    R2.setdefault('zones', [])
    clr, _, _ = _regeln(R, regeln)
    for zi, ((n, L), g) in enumerate(sorted(neu_kupfer.items())):
        R2['zones'].extend(_zonen(g, n, L, zi, clr))
    info = dict(angewendet=bool(neu_kupfer), modell='strom-form-v2', netze=bericht, pruefung=pruef,
                cpu_s=time.process_time()-start, uebergang_mm=2*a, steigung=k,
                grund='', abstand_mm=clr, rand_mm=edge, bohrabstand_mm=hole, parallel=[], profile=profiles, profil_stufen=sum(p.get('strang_stufen', 0) for b in bericht for p in b['pfade']))
    R2['strom_form'] = dict(modell=info['modell'], uebergang_mm=2*a, steigung=k, profile=profiles)
    R2['kupferbalken'] = dict(modell=info['modell'])
    R2['optionen'] = dict(R.get('optionen') or {}, strom_fix='1')
    if any(q['verletzungen'] or (q['rand_reserve_mm'] is not None and q['rand_reserve_mm'] < -0.0001) for q in pruef):
        info.update(angewendet=False, grund='Neue Abstandsverletzung: atomar verworfen')
        return copy.deepcopy(original), info
    return R2, info


def erweitern(B, R, K, regeln=None, **optionen):
    ""
    allowed = {k: v for k, v in optionen.items() if k in ('k', 'a', 'soll', 'cpu_s')}
    return strom_form(B, R, K, regeln, **allowed)


def pruefe_kanten(B, R, K, winkel_grad=15, voll=True):
    ""
    rows = []
    from gemeinsam.zonen import aktiv
    actual_copper=kupfer(B,R) if aktiv(R) else None
    targets={(z['net'],z['layer']) for z in R.get('zones',[]) if z.get('kupferbalken')}
    targets.update((p['net'],p['layer']) for p in (R.get('strom_form') or {}).get('profile',[]))
    for net, layer in sorted(targets):
        gs = [polygon(z) for z in R['zones'] if z['net'] == net and z['layer'] == layer and z.get('filled')]
        g = unary_union(gs)
        if actual_copper is not None:g=actual_copper.get((net,layer),g)
        anchors = unary_union([pad_polygon(B, p, q).buffer(0.01) for p in B['pads'] if p['net'] == net
                               for q in p.get('polys', {}).get(layer, [])] +
                              [Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', via_size(R)[0]))/2+0.01)
                               for v in R['vias'] if v['net'] == net and layer in (v.get('layers') or lagen(B))])
        shapely.prepare(anchors)
        pad_enclosures=[]
        for p in B['pads']:
            if p['net']!=net: continue
            for contour in p.get('polys',{}).get(layer,[]):
                pad=pad_polygon(B,p,contour)
                covers=g.buffer(.000001).covers(pad)
                margin=g.boundary.distance(pad) if covers else 0.0
                pad_enclosures.append(dict(ref=p['ref'],num=str(p['num']),saum_mm=margin,
                                           umschlossen=covers and margin>=.004))
        free_angles, pad_angles = [], []
        sharp = []; short_edges = []; concave = 0; convex = 0
        for poly in _teile(g):
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
                    if abs(ang) > winkel_grad:
                        sharp.append(pos.tolist()); short_edges.append(float(ed))
                        concave += int(ang < 0); convex += int(ang > 0)
        rows.append(dict(net=net, layer=layer, fehlende_flaeche=g.is_empty, pads=pad_enclosures,
                         pads_nicht_umschlossen=sum(not p["umschlossen"] for p in pad_enclosures), harte_ecken=len(sharp), innenecken=concave, aussenecken=convex, orte=sharp,
                         tangentenabweichung_max_grad=max(free_angles, default=0),
                         padkanten_max_grad=max(pad_angles, default=0),
                         min_schenkel_mm=min(short_edges,default=None),
                         sub_um_ecken=sum(d < .001 for d in short_edges),
                         unter_10um_ecken=sum(d < .01 for d in short_edges)))
    hard = sum(q['harte_ecken'] for q in rows)
    if hard and not voll:
        return dict(stufen=None, harte_ecken=hard, fehlende_stromflaechen=sum(q["fehlende_flaeche"] for q in rows), pads_nicht_umschlossen=sum(q["pads_nicht_umschlossen"] for q in rows), je_netzlage=rows,
                    tangentenabweichung_max_grad=max((q['tangentenabweichung_max_grad'] for q in rows), default=0),
                    polygon_schwelle_grad=winkel_grad, sehnenfehler_mm=.002, tangentenfenster_mm=[.04,.02,.01,.005], grund='Kantentor bereits verletzt; Querschnittstest ausgelassen')

    skipped = dict(R, tracks=[t for t in R['tracks'] if not t.get('strom_form')])
    old = pruefe_breiten(skipped, K)
    profiles = R.get('strom_form', {}).get('profile', [])

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
    old['stufen'] += jump_count
    return dict(stufen=old['stufen'], fehlende_stromflaechen=sum(q['fehlende_flaeche'] for q in rows), pads_nicht_umschlossen=sum(q['pads_nicht_umschlossen'] for q in rows), harte_ecken=sum(q['harte_ecken'] for q in rows), je_netzlage=rows,
                tangentenabweichung_max_grad=max((q['tangentenabweichung_max_grad'] for q in rows), default=0),
                polygon_schwelle_grad=winkel_grad, sehnenfehler_mm=.002, tangentenfenster_mm=[.04,.02,.01,.005], geruest=old, profile=len(profiles))


def vorbereiten(R, K):
    ""
    out = dict(R, tracks=[])
    for t in R['tracks']:
        items = t.get('items')
        same = items and len(items) == len(t['w']) and len(t['pts']) == len(items)+1 and all(
            it[0]=='L' and it[1]==t['pts'][j] and it[2]==t['pts'][j+1] and it[3]==t['w'][j]
            for j,it in enumerate(items))
        if same and K['netze'].get(t['net'],{}).get('art') in POWER and not any(
            t.get(q) for q in ('sig','kelvin','fix','strom_steg')):
            t = {q:v for q,v in t.items() if q != 'items'}
        out['tracks'].append(t)
    return out
