import math
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree

R_FACTOR, R_MIN, R_MAX, R_T = 0.5, 0.15, 0.5, 0.25
FREE = float(__import__("os").environ.get("TEARDROP_FREE", "0.11"))

HOLE = 0.21
BORDER = 0.3
AMIN = 0.0008
SHRINK = float(__import__("os").environ.get("FILLET_SHRINK", "0"))

SKIP_REF = ('NT_', 'NTBAR', 'BEFORE')


def without_holes(g):
    from shapely.ops import split
    from shapely.geometry import LineString as _LS
    if g.geom_type != 'Polygon' or not g.interiors: return [g]
    h = g.interiors[0]; cx = sum(x for x, _ in h.coords[:-1]) / (len(h.coords) - 1)
    out = []
    for t in split(g, _LS([(cx, g.bounds[1] - 1), (cx, g.bounds[3] + 1)])).geoms:
        out.extend(without_holes(t))
    return out


def _arc(a, m, c, sag=0.002):
    ax, ay = a; bx, by = m; cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12: return [a, c]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    rr = math.hypot(ax - ux, ay - uy)
    t0, t1, tm = (math.atan2(p[1] - uy, p[0] - ux) for p in (a, c, m))
    def dif(x, y): return (y - x) % (2 * math.pi)
    span = dif(t0, t1) if dif(t0, tm) < dif(t0, t1) else -dif(t1, t0)
    step = 2 * math.acos(max(-1.0, 1 - sag / rr)) if rr > sag else math.pi / 4
    n = min(64, max(2, math.ceil(abs(span) / step)))
    return [(ux + rr * math.cos(t0 + span * k / n), uy + rr * math.sin(t0 + span * k / n)) for k in range(n + 1)]


def _track(t):
    st = []
    if 'items' in t:
        for it in t['items']:
            if it[0] == 'L': st.append(([it[1], it[2]], it[3]))
            else: st.append((_arc(it[1], it[2], it[3]), it[4]))
    else:
        for a, c, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): st.append(([a, c], w))
    return [(p, w) for p, w in st if LineString(p).length > 1e-6]


def teardrop(D, R, via_d=0.45, nets_off=(), cu_layers=None, entire=(), zclr=None, areas=None):
    feat, foreign, hole_list = {}, {}, []
    for p in D['pads']:
        if not p['net']:
            if p.get('drill', 0) > 0: hole_list.append(Point(p['x'], p['y']).buffer(p['drill'] / 2, 24))
            continue
        for L in p['layers']:
            if L not in p['polys']: continue
            pg = unary_union([Polygon(q) for q in p['polys'][L]])
            foreign.setdefault(L, []).append((p['net'], pg))
            if not p['ref'].startswith(SKIP_REF):
                b = pg.minimum_rotated_rectangle.exterior.coords
                feat.setdefault((p['net'], L), []).append(dict(g=pg.buffer(-SHRINK), D=min(math.dist(b[0], b[1]), math.dist(b[1], b[2])), category='pad'))
    all_layers = set(foreign) | {t['layer'] for t in R['tracks']}
    for v in R['vias']:
        cons = Point(v['x'], v['y']).buffer(via_d / 2, 32)
        for L in all_layers:
            foreign.setdefault(L, []).append((v['net'], cons))
            feat.setdefault((v['net'], L), []).append(dict(g=cons.buffer(-SHRINK), D=via_d, category='via'))
    for t in R['tracks']:
        st = _track(t)
        if not st: continue
        g = unary_union([LineString(p).buffer(w / 2, quad_segs=8) for p, w in st])
        foreign.setdefault(t['layer'], []).append((t['net'], g))
        gk = unary_union([LineString(p).buffer(w / 2 - SHRINK, quad_segs=8) for p, w in st])
        feat.setdefault((t['net'], t['layer']), []).append(dict(g=gk, D=max(w for _, w in st), category='track', st=st))
    zclr = zclr or {}
    for (n, L), gs in (areas or {}).items():
        for g in gs:
            feat.setdefault((n, L), []).append(dict(g=g.buffer(-SHRINK), D=1e9, category='surface'))
    tree_idx = {L: (STRtree([g for _, g in v]), [n for n, _ in v]) for L, v in foreign.items()}
    hole_tree = STRtree(hole_list) if hole_list else None
    border = unary_union([Polygon(o) for o in D['outline']]).buffer(-BORDER)

    def vacant(g, net, L):
        if not border.contains(g): return False
        ab = max(FREE, zclr.get((net, L), 0) + 0.01)
        if hole_tree is not None:
            for i in hole_tree.query(g.buffer(HOLE)):
                if hole_tree.geometries.take(i).distance(g) < HOLE: return False
        tr, nets = tree_idx[L]
        for i in tr.query(g.buffer(ab)):
            if nets[i] != net and tr.geometries.take(i).distance(g) < ab: return False
        return True

    def corners(local, r, must):
        too = local.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
        f = too.difference(local)
        out = []
        for q in getattr(f, 'geoms', [f]):
            if q.geom_type != 'Polygon' or q.area < AMIN: continue
            if any(m.distance(q) > 0.003 for m in must): continue
            k = q.buffer(0.001).intersection(local.buffer(0.003))
            if len(getattr(k, 'geoms', [k])) != 1: continue
            _lk = local.buffer(0.002)
            part_list = [p for p in getattr(_lk, 'geoms', [_lk]) if p.distance(q) < 0.1]
            if len(part_list) > 1: continue
            x0, y0, x1, y1 = q.bounds
            if math.hypot(x1 - x0, y1 - y0) > 3 * r or q.area > 1.2 * r * r: continue
            out.append(q)
        return out

    out = []
    for (n, L), F in feat.items():
        if n in nets_off or (n, L) in entire or (cu_layers and L not in cu_layers): continue
        U = unary_union([f['g'] for f in F])
        tr = STRtree([f['g'] for f in F])
        sites = []
        for i, a in enumerate(F):
            for j in tr.query(a['g'].buffer(0.003)):
                if j <= i: continue
                b = F[j]
                if a['category'] == 'surface' and b['category'] == 'surface': continue
                if a['g'].distance(b['g']) > 0.002: continue
                I = a['g'].intersection(b['g'])
                if I.is_empty or I.area < 1e-6: I = a['g'].buffer(0.002).intersection(b['g'].buffer(0.002))
                nb = [x for x in (a, b) if x['category'] in ('pad', 'via')]
                if nb: r0 = R_FACTOR * min(x['D'] for x in nb); category = 'via' if all(x['category'] == 'via' for x in nb) else 'pad'
                else: r0 = max(R_T, R_FACTOR * max(x['D'] for x in (a, b) if x['category'] == 'track')); category = 'T'
                sites.append((I, min(max(r0, R_MIN), R_MAX), [a['g'], b['g']], category))
            if a['category'] == 'track':
                for p, w in a['st']:
                    for k in range(1, len(p) - 1):
                        v1 = (p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); v2 = (p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1])
                        n1, n2 = math.hypot(*v1), math.hypot(*v2)
                        if n1 < 1e-6 or n2 < 1e-6: continue
                        if (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2) > math.cos(math.radians(12)): continue
                        sites.append((Point(p[k]).buffer(w / 2), min(max(R_FACTOR * w, R_MIN), R_MAX), [a['g']], 'bend'))
                for k in range(len(a['st']) - 1):
                    p, w = a['st'][k]; p2, w2 = a['st'][k + 1]
                    if math.dist(p[-1], p2[0]) > 0.01: continue
                    v1 = (p[-1][0] - p[-2][0], p[-1][1] - p[-2][1]); v2 = (p2[1][0] - p2[0][0], p2[1][1] - p2[0][1])
                    n1, n2 = math.hypot(*v1), math.hypot(*v2)
                    if n1 < 1e-6 or n2 < 1e-6: continue
                    if (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2) > math.cos(math.radians(12)) and abs(w - w2) < 0.01: continue
                    sites.append((Point(p[-1]).buffer(max(w, w2) / 2), min(max(R_FACTOR * max(w, w2), R_MIN), R_MAX), [a['g']], 'bend'))
        pieces = []
        for I, r0, must, category in sites:
            W = I.buffer(2.5 * r0 + 0.25, quad_segs=16)
            local = unary_union(must).intersection(W)

            open_items = corners(local, r0, must)
            if not open_items: continue
            r = r0
            while open_items and r >= 0.1:
                qs = open_items if r == r0 else corners(local, r, must)
                rest = []
                for o in open_items:
                    m = [q for q in qs if q.intersects(o)]
                    if not m: continue
                    q = max(m, key=lambda q: q.intersection(o).area)
                    if vacant(q, n, L): pieces.append((q, r, category))
                    else: rest.append(o)
                open_items = rest; r *= 0.75
        if not pieces: continue
        try: all_items = unary_union([U] + [q for q, _, _ in pieces])
        except Exception:
            from shapely.validation import make_valid
            import shapely; all_items = shapely.union_all([make_valid(g) for g in [U] + [q for q, _, _ in pieces]], grid_size=1e-6)

        OE = float(__import__("os").environ.get("FILLET_OPEN", "0.006"))
        pieces = [((q.buffer(-OE, quad_segs=8).buffer(OE, quad_segs=8) if OE else q), r, category) for q, r, category in pieces]
        pieces = [(q, r, category) for q, r, category in pieces if not q.is_empty and q.area > AMIN / 2]
        if not pieces: continue
        def _cut(q):
            g = q.buffer(0.01, quad_segs=8)
            try: return g.intersection(all_items)
            except Exception:
                from shapely.validation import make_valid
                try: return make_valid(g).intersection(make_valid(all_items))
                except Exception: return g.intersection(all_items.buffer(0), grid_size=1e-6)
        ges = unary_union([_cut(q) for q, _, _ in pieces]).simplify(0.0005)
        for g0 in getattr(ges, 'geoms', [ges]):
            for g in without_holes(g0):
                if g.geom_type != 'Polygon' or g.area < AMIN: continue
                involved = [(r, category) for q, r, category in pieces if q.intersects(g)]
                if not involved: continue
                kinds = {a for _, a in involved}
                category = 'pad' if 'pad' in kinds else 'via' if 'via' in kinds else 'T' if 'T' in kinds else 'bend'
                out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], category=category,
                                r=round(min(r for r, _ in involved), 3)))
    return out
