import json, sys, math
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

A = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
OE1 = float(A.get('open_', 0.006))
RD, AMIN, BORDER = 0.1, 0.0012, float(A.get('border', 0.3))
FREE = float(A.get('vacant', 0.11))
HOLE = float(A.get('hole_size', 0.2))
d = json.load(open(sys.argv[1]))


def without_holes(g):
    from shapely.ops import split
    from shapely.geometry import LineString as _LS
    if g.geom_type != 'Polygon' or not g.interiors: return [g]
    h = g.interiors[0]; cx = sum(x for x, _ in h.coords[:-1]) / (len(h.coords) - 1)
    out = []
    for t in split(g, _LS([(cx, g.bounds[1] - 1), (cx, g.bounds[3] + 1)])).geoms:
        out.extend(without_holes(t))
    return out


def poly(ps):
    return unary_union([Polygon(o, h).buffer(0) for o, h in ps])


border = poly(d['outline']).buffer(-BORDER + 0.005)
hole_list = [Point(x, y).buffer(r, 24) for x, y, r in d['hole_list']]
l_tree = STRtree(hole_list) if hole_list else None
items = {(n, L): poly(ps) for n, lay in d['items'].items() for L, ps in lay.items()}
fill, clr, hinder = {}, {}, {}
for z in d['zone_list']:
    k = (z['net'], z['layer'])
    fill.setdefault(k, []).append(poly(z['polys']))
    if z['prio'] < 50: clr[k] = max(clr.get(k, 0), z['clr'])
    else: hinder.setdefault(z['layer'], []).append((z['net'], poly(z['polys'])))


first = []
for fo in d.get('shapes', []):
    k = (fo['net'], fo['layer']); g = poly(fo['polys'])
    real = unary_union([items.get(k, Polygon())] + fill.get(k, []))
    W = g.buffer(1.2)
    too = real.intersection(W).buffer(0.5, quad_segs=32).buffer(-0.5, quad_segs=32)
    g2 = g.intersection(too.union(real))
    g2 = g2.buffer(-OE1, quad_segs=8).buffer(OE1, quad_segs=8)
    for t in getattr(g2, 'geoms', [g2]):
        if t.geom_type == 'Polygon' and t.area > AMIN / 2 and t.difference(real).area > AMIN / 4: first.append((k, t))
for k, t in first: items[k] = unary_union([items.get(k, Polygon()), t])
for (n, L), g in items.items(): hinder.setdefault(L, []).append((n, g))
htree = {L: (STRtree([g for _, g in v]), [n for n, _ in v]) for L, v in hinder.items()}


def vacant(g, net, L, ab):
    if not border.contains(g): return False
    if l_tree is not None:
        for i in l_tree.query(g.buffer(HOLE)):
            if l_tree.geometries.take(i).distance(g) < HOLE: return False
    tr, nets = htree[L]
    for i in tr.query(g.buffer(ab)):
        if nets[i] != net and tr.geometries.take(i).distance(g) < ab: return False
    for n2, g2 in new.get(L, []):
        if n2 != net and g2.distance(g) < ab: return False
    return True


def corners(local, r):
    too = local.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
    f = too.difference(local); out = []
    for q in getattr(f, 'geoms', [f]):
        if q.geom_type != 'Polygon' or q.area < AMIN * (r / RD) ** 2 * 0.5: continue
        k = q.buffer(0.001).intersection(local.buffer(0.003))
        if len(getattr(k, 'geoms', [k])) != 1: continue
        _lk = local.buffer(0.002)
        part_list = [p for p in getattr(_lk, 'geoms', [_lk]) if p.distance(q) < 0.1]
        if len(part_list) > 1: continue
        x0, y0, x1, y1 = q.bounds
        if math.hypot(x1 - x0, y1 - y0) > 3 * r or q.area > 1.2 * r * r: continue
        out.append(q)
    return out


out, stat, new = [], {}, {}
keys = set(items) | set(fill)
for k in sorted(keys):
    n, L = k
    if not n: continue
    U = unary_union([items.get(k, Polygon())] + fill.get(k, [])).buffer(-0.004)
    if U.is_empty: continue
    has = k in clr
    if not has: continue
    ab = FREE
    r0 = 0.5 if has else 0.25
    pieces = []
    for c in corners(U, RD):
        r = r0
        while r >= 0.1:
            W = c.buffer(2.5 * r + 0.2, quad_segs=12)
            local = U.intersection(W)
            m = [q for q in corners(local, r) if q.intersects(c)]
            if m:
                q = max(m, key=lambda q: q.intersection(c).area)
                if vacant(q, n, L, ab): pieces.append((q, r)); break
            r *= 0.75
    if not pieces: continue
    all_items = unary_union([U] + [q for q, _ in pieces])
    OE = float(A.get("open_", 0.006))
    pieces = [((q.buffer(-OE, quad_segs=8).buffer(OE, quad_segs=8) if OE else q), r) for q, r in pieces]
    pieces = [(q, r) for q, r in pieces if not q.is_empty and q.area > AMIN / 2]
    if not pieces: continue
    ges = unary_union([q.buffer(0.01, quad_segs=8).intersection(all_items) for q, _ in pieces]).simplify(0.0005)
    new.setdefault(L, []).append((n, ges))
    for g0 in getattr(ges, 'geoms', [ges]):
        for g in without_holes(g0):
            if g.geom_type != 'Polygon' or g.area < AMIN: continue
            out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], category='surface' if has else 'pad'))
    stat[L] = stat.get(L, 0) + len(pieces)
for (n, L), t in first:
    for g in without_holes(t.simplify(0.0005)):
        if g.geom_type == 'Polygon' and g.area > AMIN / 2:
            out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], category='pass1'))
json.dump(out, open(sys.argv[2], 'w'))
print('Pass 1 trimmed', len(first), '| follow-up fillets', sum(stat.values()), stat, '| shapes', len(out))
