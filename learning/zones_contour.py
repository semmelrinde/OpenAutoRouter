import math
import os

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely import make_valid
from shapely.ops import unary_union

def _directions(N):
    return [(math.cos(2 * math.pi * k / N), math.sin(2 * math.pi * k / N)) for k in range(N)]


DIRECTION = _directions(8)


def _dp(pts, eps):
    keep = {0, len(pts) - 1}; stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1: continue
        (ax, ay), (bx, by) = pts[i], pts[j]; dx, dy = bx - ax, by - ay; l = math.hypot(dx, dy)
        best, k = -1.0, -1
        for m in range(i + 1, j):
            px, py = pts[m]
            d = abs(dy * (px - ax) - dx * (py - ay)) / l if l > 1e-12 else math.hypot(px - ax, py - ay)
            if d > best: best, k = d, m
        if best > eps:
            keep.add(k); stack += [(i, k), (k, j)]
    return sorted(keep)


def _cut(a, b):
    (n1, c1), (n2, c2) = a, b
    det = n1[0] * n2[1] - n1[1] * n2[0]
    if abs(det) < 1e-9: return None
    return ((c1 * n2[1] - c2 * n1[1]) / det, (n1[0] * c2 - n2[0] * c1) / det)


def ring_contour(coords, eps=0.25, wedge=0.3, split=1):
    N = 8 * split; Q = 2 * split; RI = DIRECTION if N == 8 else _directions(N)

    def line(d, pts):
        ux, uy = RI[d % N]; n = (uy, -ux)
        return [d % N, n, max(n[0] * x + n[1] * y for x, y in pts), pts]

    def chamfer(d, v):
        ux, uy = RI[d % N]; n = (uy, -ux)
        return [d % N, n, n[0] * v[0] + n[1] * v[1] - wedge, []]

    def level(a, c):
        m = (a[0] + Q) % N if c[2] < a[2] else (a[0] - Q) % N
        ux, uy = RI[m]; n = (uy, -ux)
        q = (a[3][-1:] + c[3][:1]) or a[3] or c[3]
        if not q: return [m, n, 0.0, []]
        x = sum(p[0] for p in q) / len(q); y = sum(p[1] for p in q) / len(q)
        return [m, n, n[0] * x + n[1] * y, []]

    ring = orient(Polygon(coords), sign=1)
    pts = list(ring.exterior.coords)[:-1]
    if len(pts) < 3: return None
    s = max(range(len(pts)), key=lambda i: (pts[i][0] - pts[0][0]) ** 2 + (pts[i][1] - pts[0][1]) ** 2)
    pts = pts[s:] + pts[:s]
    move = pts + [pts[0]]
    idx = _dp(move, eps)
    chain_list = [move[i:j + 1] for i, j in zip(idx[:-1], idx[1:])]
    L = []
    for k in chain_list:
        dx, dy = k[-1][0] - k[0][0], k[-1][1] - k[0][1]
        if math.hypot(dx, dy) < 1e-9: continue
        d = round(math.atan2(dy, dx) / (2 * math.pi / N)) % N
        if L and L[-1][0] == d: L[-1] = line(d, L[-1][3] + k)
        else: L.append(line(d, k))
    if len(L) > 1 and L[0][0] == L[-1][0]: L[0] = line(L[0][0], L[-1][3] + L[0][3]); L.pop()
    if len(L) < 3: return None


    for _ in range(4 * len(L)):
        new = []; changed = False
        for i, a in enumerate(L):
            b = L[(i + 1) % len(L)]; new.append(a); t = (b[0] - a[0]) % N
            if Q < t <= 2 * Q:
                new.append(line(a[0] + Q, a[3] + b[3])); changed = True
            elif 2 * Q < t < 3 * Q:
                v = _cut(a[1:3], b[1:3])
                if v is None: continue
                new.append(chamfer(a[0] - Q, v)); changed = True
        L = new
        if not changed: break

    for _ in range(6 * len(L) + 10):
        if len(L) < 3: return None
        V = [_cut(L[i - 1][1:3], L[i][1:3]) for i in range(len(L))]
        if any(v is None for v in V):
            i = next(i for i in range(len(L)) if V[i] is None); a, c = L[i - 1], L[i]
            if abs(a[2] - c[2]) < 1e-9: L.pop(i); continue
            L.insert(i, level(a, c)); continue
        way = None
        for i in range(len(L)):
            a, b = V[i], V[(i + 1) % len(L)]; u = RI[L[i][0]]
            if (b[0] - a[0]) * u[0] + (b[1] - a[1]) * u[1] < -1e-9: way = i; break
        if way is None:
            p = Polygon(V)
            if not p.is_valid: p = p.buffer(0)
            if p.geom_type != 'Polygon' or p.is_empty: return None
            return p
        a, c = L[way - 1], L[(way + 1) % len(L)]; pts = L[way][3]
        L.pop(way); way = way % len(L)
        t = (c[0] - a[0]) % N
        if Q < t <= 2 * Q:
            q = a[3] + pts + c[3]
            if q: L.insert(way, line(a[0] + Q, q))
        elif 2 * Q < t < 3 * Q and pts:
            v = _cut(a[1:3], c[1:3])
            if v is not None: L.insert(way, chamfer(a[0] - Q, v))
    return None


def contour(g, eps=0.25, bloat=1.5):
    out = []
    for p in getattr(g, 'geoms', [g]):
        if p.is_empty or p.geom_type != 'Polygon': continue
        q = ring_contour(list(p.exterior.coords), eps)
        if q is None or (q.area > bloat * p.area and q.area - p.area > 2.0):

            q2 = ring_contour(list(p.exterior.coords), eps, split=4)
            if q2 is not None and (q is None or q2.area < q.area): q = q2
        out.append(Polygon(p.exterior) if q is None else q)
    return unary_union(out) if out else Polygon()


def _free_pad(B, R, K, L, n, clr, border, foreign_zones):
    from learning import current_zones as SZ
    outlines = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, SZ.QS)
    _, b_via = SZ._via_ground(R)
    hole_list = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, SZ.QS)) for v in R.get('vias', [])]
    hole_list += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, SZ.QS)) for p in B['pads'] if p.get('drill')]
    part_list = SZ._parts(B, dict(R, zone_list=[]), L)
    foreign = unary_union([g for m, g in part_list if m != n] + [g for m, g in hole_list if m != n] + foreign_zones)

    keepout = foreign.buffer(clr + .003, SZ.QS)
    vacant = outlines.difference(keepout if keepout.is_valid else make_valid(keepout)).difference(SZ.keepout_area(B, L, 'areas'))
    vacant = SZ.mask_free(vacant, B, n, L, clr)
    hole = float((R.get('options') or {}).get('h2h') or .2)
    vacant = vacant.difference(unary_union([h.buffer(hole + .003, SZ.QS) for m, h in hole_list if m != n]))
    if SZ.track_protection_on(R):
        _, moat = SZ._track_protection(R, L, n)
        if not moat.is_empty: vacant = vacant.difference(moat)
    return vacant.buffer(-.002, 8)


def _gnd_pad_keepout(B, R, K, L, border=0.4):
    from learning import current_zones as SZ
    gnd = set(K.get('gnd') or ['GND'])
    gnd_vias = [Point(v['x'], v['y']) for v in R.get('vias', []) if v.get('net') in gnd and L in (v.get('layers') or SZ._layers(B))]
    from learning.copper_bars import track_geometry
    gnd_track = unary_union([track_geometry(t) for t in R.get('tracks', []) if t.get('net') in gnd and t['layer'] == L and len(t['pts']) >= 2])
    out = []
    for p in B['pads']:
        if p['net'] not in gnd or p.get('drill'): continue
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) < 3: continue
            pg = Polygon(q).buffer(0)
            if any(pg.buffer(.26).contains(v) for v in gnd_vias) or (not gnd_track.is_empty and pg.intersects(gnd_track)): continue
            out.append(pg.envelope.buffer(border, join_style=2))
    return unary_union(out) if out else Polygon()


def _fill(O, vacant, g, r_min):
    F = O.intersection(vacant)
    if r_min > 0: F = F.buffer(-r_min, 16).buffer(r_min, 16).intersection(vacant)
    F = unary_union([F, g]).buffer(0)
    if r_min > 0:
        F = unary_union([F, F.buffer(2 * r_min, 16).buffer(-2 * r_min, 16).intersection(vacant)]).buffer(0)
        F = unary_union([g, F.buffer(-.08, 16).buffer(.08, 16).intersection(F)]).buffer(0)
    return unary_union([p for p in getattr(F, 'geoms', [F]) if p.geom_type == 'Polygon' and p.intersects(g)])


def _foreign_vias(B, R, L, n, clr):
    from common.zones import via_geometry
    from learning import current_zones as SZ
    _, b_via = SZ._via_ground(R); hole = float((R.get('options') or {}).get('h2h') or .2)
    out = []
    for v in via_geometry(R)['vias']:
        if v.get('net') == n or L not in (v.get('layers') or SZ._layers(B)): continue
        r = max(v['diameter'] / 2 + clr, b_via / 2 + hole) + .003
        out.append((Point(v['x'], v['y']), r))
    return out


def via_net_class(F, m, r):
    if not F.intersects(m.buffer(r + .02, 16)): return 'vacant'
    ring = m.buffer(r + .3, 32).difference(m.buffer(r + .02, 32))
    f = F.intersection(ring).area / ring.area
    return 'too' if f > .95 else 'deep' if f > .62 else 'half' if f > .38 else 'planar'


def _edge_set(P, i, c_new):
    q = list(P.exterior.coords)[:-1]; N = len(q)
    if N < 3: return None
    L = []
    for a, b in zip(q, q[1:] + q[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]; l = math.hypot(dx, dy)
        if l < 1e-12: return None
        n = (dy / l, -dx / l); L.append((n, n[0] * a[0] + n[1] * a[1]))
    L[i] = (L[i][0], c_new)
    V = [_cut(L[k - 1], L[k]) for k in range(N)]
    if any(v is None for v in V): return None
    for k in range(N):
        a, b = V[k], V[(k + 1) % N]; u = (-L[k][0][1], L[k][0][0])
        if (b[0] - a[0]) * u[0] + (b[1] - a[1]) * u[1] < -1e-9: return None
    P2 = Polygon(V)
    return P2 if P2.is_valid and not P2.is_empty else None


def edges_for_vias(O, F, g, vacant, discs, r_n, maxr=0.8, inherited=None, max_loss=0.05):
    g0_area = g.area
    for m, r in discs:
        if not F.intersects(m.buffer(r + .02, 16)): continue
        k = via_net_class(F, m, r)
        if k not in ('planar', 'deep'): continue
        prev_k = via_net_class(g, m, r) == k
        if prev_k and inherited is None: continue
        part_list = [p for p in getattr(O, 'geoms', [O]) if p.geom_type == 'Polygon']
        best = None
        for j, P in enumerate(part_list):
            P = orient(P, sign=1); q = list(P.exterior.coords)[:-1]
            for i, (a, b) in enumerate(zip(q, q[1:] + q[:1])):
                seg = LineString([a, b]); d = seg.distance(m)
                if d > r + maxr: continue
                dx, dy = b[0] - a[0], b[1] - a[1]; l = math.hypot(dx, dy)
                if l < 1e-9: continue
                n = (dy / l, -dx / l); c = n[0] * a[0] + n[1] * a[1]; t = n[0] * m.x + n[1] * m.y
                goal = t - r - .03
                if not (0 < c - goal <= maxr): continue
                if best is None or d < best[0]: best = (d, j, P, i, goal)
        if best is None: continue
        _, j, P, i, goal = best
        P2 = _edge_set(P, i, goal)
        if P2 is None: continue
        O2 = unary_union([P2 if jj == j else part_list[jj] for jj in range(len(part_list))])
        if prev_k:


            g2 = unary_union([g.intersection(O2), inherited.intersection(g)]).buffer(0)
            if g2.area < (1 - max_loss) * g0_area: continue
            F2 = _fill(O2, vacant, g2, r_n)
            if via_net_class(F2, m, r) in ('vacant', 'half', 'too'): O, F, g = O2, F2, g2
            continue
        F2 = _fill(O2, vacant, g, r_n)
        if via_net_class(F2, m, r) in ('vacant', 'half', 'too'): O, F = O2, F2
    return O, F


def _bites(B, R, L, n, F, clr):
    outer = unary_union([Polygon(p.exterior) for p in getattr(F, 'geoms', [F]) if p.geom_type == 'Polygon'])
    return sum(1 for m, r in _foreign_vias(B, R, L, n, clr)
               if outer.intersects(m.buffer(r, 16)) and not outer.covers(m.buffer(r - .002, 16)))


def _via_rings(B, R, L, n, F, vacant, clr, ring=0.3):
    outer = unary_union([Polygon(p.exterior) for p in getattr(F, 'geoms', [F]) if p.geom_type == 'Polygon'])
    out = []
    for m, r in _foreign_vias(B, R, L, n, clr):
        d = m.buffer(r, 16)
        if not outer.intersects(d) or outer.covers(m.buffer(r - .002, 16)): continue
        a = r + ring; ok = Polygon([(m.x + a * c / math.cos(math.pi / 8), m.y + a * s_ / math.cos(math.pi / 8))
                                    for c, s_ in ((math.cos(math.pi / 8 + k * math.pi / 4), math.sin(math.pi / 8 + k * math.pi / 4))
                                                  for k in range(8))])
        if ok.difference(vacant).difference(m.buffer(r + .01, 16)).area > 1e-4: continue
        out.append(ok)
    return out


def simplify(B, R, K, zones, gnd_layers=None, clr=None, border=0.25, eps=0.25, r_min=0.15, measurement_branch=True, via_rounds=2,
                 gnd_protection=True, bay=0.8, notches=True, inherited_notches=True, currentless=0.03, peaks=0.1):
    from learning import current_zones as SZ
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    out = [dict(z) for z in zones]
    for i, z in enumerate(out):
        if z.get('source') != 'current_zones' or not z.get('polys'): continue
        n, L = z['net'], z['layer']
        g = SZ._geo(z)
        if g.is_empty: continue
        other = [SZ._geo(q) for j, q in enumerate(out) if j != i and q['layer'] == L and q['net'] != n and q.get('polys')]

        other += [SZ._geo(q) for j, q in enumerate(zones) if j != i and q['layer'] == L and q['net'] != n and q.get('polys')]
        other += [SZ._geo(q) for q in R.get('zone_list', []) if q['layer'] == L and q['net'] != n]
        vacant = _free_pad(B, R, K, L, n, clr, border, other)
        gnd_keepout = _gnd_pad_keepout(B, R, K, L)
        if not gnd_keepout.is_empty: vacant = vacant.difference(gnd_keepout)
        if measurement_branch:
            _, moat = SZ.measurement_branches(B, R, K, L, n)
            if not moat.is_empty: vacant = vacant.difference(moat)

        net_copper = (K.get('net_list') or {}).get(n) or {}
        r_n = max(r_min, max(.1, float(net_copper.get('wmin') or .1), .25 if net_copper.get('category') == 'sw' else .1) / 2 + .04)


        b_n = 0.0 if net_copper.get('category') == 'sw' else bay
        O = contour(g.buffer(b_n, 16).buffer(-b_n, 16).union(g) if b_n > 0 else g, eps)
        F = _fill(O, vacant, g, r_n)
        for _ in range(via_rounds):


            to_it = [q for q in _via_rings(B, R, L, n, F, vacant, clr) if not q.is_empty]
            if not to_it: break
            O2 = contour(unary_union([O] + to_it), eps)
            F2 = _fill(O2, vacant, g, r_n)
            if _bites(B, R, L, n, F2, clr) >= _bites(B, R, L, n, F, clr): break
            O, F = O2, F2
        if notches:
            owned = unary_union([q for m_, q in SZ._parts(B, dict(R, zone_list=[]), L) if m_ == n]) if inherited_notches else None
            O, F = edges_for_vias(O, F, g, vacant, _foreign_vias(B, R, L, n, clr), r_n, inherited=owned)
        if peaks > 0:
            F = peaks_rounds(F, g, peaks, own_set=unary_union([q for m_, q in SZ._parts(B, dict(R, zone_list=[]), L) if m_ == n]).buffer(.1))
        if F.is_empty or (not inherited_notches and F.area < g.area - 1e-6): continue
        z.update(polys=SZ._polys(F), area_mm2=round(F.area, 2), contour=dict(eps=eps, before_mm2=round(g.area, 2)))
    if currentless > 0: out = currentless_cut(B, R, K, out, eps, share=currentless)
    if gnd_protection: out = _gnd_protect(B, R, K, gnd_layers, zones, out)
    return out

def peaks_rounds(F, g, r=0.1, own_set=None):
    halt = F.intersection(own_set) if own_set is not None else F.intersection(g)
    F2 = unary_union([F.buffer(-r, 16).buffer(r, 16).intersection(F), halt]).buffer(0)
    F2 = unary_union([q for q in getattr(F2, 'geoms', [F2]) if q.geom_type == 'Polygon' and q.intersects(own_set if own_set is not None else g)])
    return F2 if not F2.is_empty else F


def currentless_cut(B, R, K, zones, eps=0.25, share=0.03, min_area=0.3, r_min=0.15):
    import tempfile
    import numpy as np
    from scipy import ndimage
    from learning import current_zones as SZ
    from analysis.copper_model import ir_values
    try:
        with tempfile.TemporaryDirectory() as td:
            w = ir_values(B, dict(R, zone_list=list(R.get('zone_list') or []) + [z for z in zones if z.get('polys')]), K, res=0.1, maps=td)
            maps = {n: dict(np.load(os.path.join(td, f))) for n, f in (w.get('maps') or {}).items() if f}
    except Exception:
        return zones
    out = []
    for z in zones:
        n, L = z['net'], z['layer']; k = maps.get(n)
        if z.get('source') != 'current_zones' or not z.get('polys') or k is None or (L + '_J_A_mm2') not in k:
            out.append(z); continue
        J = k[L + '_J_A_mm2']; X, Y = k['x_mm'], k['y_mm']
        all_items = np.concatenate([k[q].ravel() for q in k if q.endswith('_J_A_mm2')])
        all_items = all_items[all_items > 0]
        if all_items.size < 20: out.append(z); continue
        threshold = share * float(np.percentile(all_items, 90))
        F = SZ._geo(z)
        own_set = unary_union([q for m_, q in SZ._parts(B, dict(R, zone_list=[]), L) if m_ == n]).buffer(.1)
        import shapely
        inner = shapely.contains_xy(F, X, Y) & ~shapely.contains_xy(own_set, X, Y)
        tot = inner & (J < threshold)
        lab, num = ndimage.label(tot)
        h = float(abs(X[0, 1] - X[0, 0])) or .1
        way = []
        for i in range(1, num + 1):
            m = lab == i
            if m.sum() * h * h < min_area: continue
            ys, xs = np.nonzero(m)
            way.append(shapely.union_all(shapely.box(X[ys, xs] - h * .55, Y[ys, xs] - h * .55, X[ys, xs] + h * .55, Y[ys, xs] + h * .55)))
        if not way: out.append(z); continue
        F2 = F.difference(unary_union(way)).buffer(-r_min, 16).buffer(r_min, 16).intersection(F)
        F2 = unary_union([q for q in getattr(F2, 'geoms', [F2]) if q.geom_type == 'Polygon' and q.intersects(own_set)])
        if F2.is_empty: out.append(z); continue

        F3 = contour(F2, eps).intersection(F)
        F3 = F3.buffer(-.2, 16).buffer(.2, 16).intersection(F).union(F.intersection(own_set)).buffer(0)
        F3 = F3.union(F3.buffer(.3, 16).buffer(-.3, 16).intersection(F)).buffer(0)
        F3 = F3.buffer(-.08, 16).buffer(.08, 16).intersection(F3).union(F3.intersection(own_set)).buffer(0)
        F3 = unary_union([q for q in getattr(F3, 'geoms', [F3]) if q.geom_type == 'Polygon' and q.intersects(own_set)])
        if F3.is_empty or F3.area > F.area - min_area: out.append(z); continue
        out.append(dict(z, polys=SZ._polys(F3), area_mm2=round(F3.area, 2), currentless_mm2=round(F.area - F3.area, 2)))
    return out


def gnd_islands(B, R, K, gnd_layers, zs, gnd='GND'):
    from collections import defaultdict
    from learning import current_zones as SZ
    from learning.copper_bars import copper_map, cu_layers, via_size
    RR = dict(R, zone_list=list(R.get('zone_list') or []) + list(zs), vias=list(R.get('vias') or []))

    gz, gv = SZ.gnd_fill(B, RR, K, gnd_layers, pockets=False, r41=False, k3=False); RR["zone_list"] += gz; RR["vias"] += gv
    copper = copper_map(B, RR, unfilled='ignore'); vd, _ = via_size(RR)
    nodes = [(L, p) for L in cu_layers(B) for p in getattr(copper.get((gnd, L)), 'geoms', [copper.get((gnd, L))])
              if p is not None and not p.is_empty]
    par = list(range(len(nodes)))
    def f(x):
        while par[x] != x: par[x] = par[par[x]]; x = par[x]
        return x
    def hits(layers_, geo): return [i for i, (L, p) in enumerate(nodes) if L in layers_ and p.intersects(geo)]
    for v in RR['vias']:
        if v.get('net') != gnd: continue
        h = hits(v.get('layers') or cu_layers(B), Point(v['x'], v['y']).buffer(v.get('diameter', vd) / 2, 16))
        for j in h[1:]: par[f(j)] = f(h[0])
    for p in B['pads']:
        if p['net'] != gnd: continue
        h = hits(p.get('layers') or [], Point(p['x'], p['y']).buffer(.01))
        for j in h[1:]: par[f(j)] = f(h[0])
    group_list = defaultdict(list)
    for p in B['pads']:
        if p['net'] != gnd: continue
        h = hits(p.get('layers') or [], Point(p['x'], p['y']).buffer(.01))
        group_list[f(h[0]) if h else ('vacant', p['ref'], p['num'])].append(p)
    if not group_list: return []
    large = max(group_list, key=lambda k: len(group_list[k]))
    return sorted(((p['ref'], p['num']), (p['x'], p['y'])) for k, ps in group_list.items() if k != large for p in ps)


def _gnd_protect(B, R, K, gnd_layers, prior, new, close=1.5):
    from learning import current_zones as SZ
    gnd = (K.get('gnd') or ['GND'])[0]
    try:
        before = None
        for round_no in range(4):
            islands = gnd_islands(B, R, K, gnd_layers, new, gnd)
            if islands and before is None: before = {k for k, _ in gnd_islands(B, R, K, gnd_layers, prior, gnd)}
            new_island = [(k, xy) for k, xy in islands if k not in (before or set())]
            if not new_island: return new
            if round_no == 3: break

            points = unary_union([Polygon([(xy[0] + close * math.cos(math.radians(22.5 + 45 * k)), xy[1] + close * math.sin(math.radians(22.5 + 45 * k)))
                                           for k in range(8)]) for _, xy in new_island])
            back = [i for i, (a, b) in enumerate(zip(prior, new)) if b is not a and b.get('contour')
                       and SZ._geo(b).difference(SZ._geo(a)).intersects(points)]
            if not back: break
            for i in back:
                if round_no == 2: new[i] = prior[i]; continue
                ga = SZ._geo(prior[i]); gb = SZ._geo(new[i]).difference(points).union(ga).buffer(0)
                gb = peaks_rounds(gb, ga)
                gb = unary_union([q for q in getattr(gb, 'geoms', [gb]) if q.geom_type == 'Polygon' and q.intersects(ga)])
                new[i] = dict(new[i], polys=SZ._polys(gb), area_mm2=round(gb.area, 2))
    except Exception:
        return prior
    return prior
