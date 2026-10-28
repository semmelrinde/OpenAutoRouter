""
import json, sys, math
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

A = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
OE1 = float(A.get('oeffnen', 0.006))
RD, AMIN, RAND = 0.1, 0.0012, float(A.get('rand', 0.3))
FREI = float(A.get('frei', 0.11))
LOCH = float(A.get('loch', 0.2))
d = json.load(open(sys.argv[1]))


def ohne_loecher(g):
    ""
    from shapely.ops import split
    from shapely.geometry import LineString as _LS
    if g.geom_type != 'Polygon' or not g.interiors: return [g]
    h = g.interiors[0]; cx = sum(x for x, _ in h.coords[:-1]) / (len(h.coords) - 1)
    out = []
    for t in split(g, _LS([(cx, g.bounds[1] - 1), (cx, g.bounds[3] + 1)])).geoms:
        out.extend(ohne_loecher(t))
    return out


def poly(ps):
    return unary_union([Polygon(o, h).buffer(0) for o, h in ps])


rand = poly(d['outline']).buffer(-RAND + 0.005)
loecher = [Point(x, y).buffer(r, 24) for x, y, r in d['loecher']]
lbaum = STRtree(loecher) if loecher else None
items = {(n, L): poly(ps) for n, lay in d['items'].items() for L, ps in lay.items()}
fuell, clr, hinder = {}, {}, {}
for z in d['zonen']:
    k = (z['net'], z['layer'])
    fuell.setdefault(k, []).append(poly(z['polys']))
    if z['prio'] < 50: clr[k] = max(clr.get(k, 0), z['clr'])
    else: hinder.setdefault(z['layer'], []).append((z['net'], poly(z['polys'])))


erst = []
for fo in d.get('formen', []):
    k = (fo['net'], fo['layer']); g = poly(fo['polys'])
    echt = unary_union([items.get(k, Polygon())] + fuell.get(k, []))
    W = g.buffer(1.2)
    zu = echt.intersection(W).buffer(0.5, quad_segs=32).buffer(-0.5, quad_segs=32)
    g2 = g.intersection(zu.union(echt))
    g2 = g2.buffer(-OE1, quad_segs=8).buffer(OE1, quad_segs=8)
    for t in getattr(g2, 'geoms', [g2]):
        if t.geom_type == 'Polygon' and t.area > AMIN / 2 and t.difference(echt).area > AMIN / 4: erst.append((k, t))
for k, t in erst: items[k] = unary_union([items.get(k, Polygon()), t])
for (n, L), g in items.items(): hinder.setdefault(L, []).append((n, g))
hbaum = {L: (STRtree([g for _, g in v]), [n for n, _ in v]) for L, v in hinder.items()}


def frei(g, net, L, ab):
    if not rand.contains(g): return False
    if lbaum is not None:
        for i in lbaum.query(g.buffer(LOCH)):
            if lbaum.geometries.take(i).distance(g) < LOCH: return False
    tr, nets = hbaum[L]
    for i in tr.query(g.buffer(ab)):
        if nets[i] != net and tr.geometries.take(i).distance(g) < ab: return False
    for n2, g2 in neu.get(L, []):
        if n2 != net and g2.distance(g) < ab: return False
    return True


def ecken(lok, r):
    zu = lok.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
    f = zu.difference(lok); out = []
    for q in getattr(f, 'geoms', [f]):
        if q.geom_type != 'Polygon' or q.area < AMIN * (r / RD) ** 2 * 0.5: continue
        k = q.buffer(0.001).intersection(lok.buffer(0.003))
        if len(getattr(k, 'geoms', [k])) != 1: continue
        _lk = lok.buffer(0.002)
        teile = [p for p in getattr(_lk, 'geoms', [_lk]) if p.distance(q) < 0.1]
        if len(teile) > 1: continue
        x0, y0, x1, y1 = q.bounds
        if math.hypot(x1 - x0, y1 - y0) > 3 * r or q.area > 1.2 * r * r: continue
        out.append(q)
    return out


out, stat, neu = [], {}, {}
keys = set(items) | set(fuell)
for k in sorted(keys):
    n, L = k
    if not n: continue
    U = unary_union([items.get(k, Polygon())] + fuell.get(k, [])).buffer(-0.004)
    if U.is_empty: continue
    hat = k in clr
    if not hat: continue
    ab = FREI
    r0 = 0.5 if hat else 0.25
    stuecke = []
    for c in ecken(U, RD):
        r = r0
        while r >= 0.1:
            W = c.buffer(2.5 * r + 0.2, quad_segs=12)
            lok = U.intersection(W)
            m = [q for q in ecken(lok, r) if q.intersects(c)]
            if m:
                q = max(m, key=lambda q: q.intersection(c).area)
                if frei(q, n, L, ab): stuecke.append((q, r)); break
            r *= 0.75
    if not stuecke: continue
    alle = unary_union([U] + [q for q, _ in stuecke])
    OE = float(A.get("oeffnen", 0.006))
    stuecke = [((q.buffer(-OE, quad_segs=8).buffer(OE, quad_segs=8) if OE else q), r) for q, r in stuecke]
    stuecke = [(q, r) for q, r in stuecke if not q.is_empty and q.area > AMIN / 2]
    if not stuecke: continue
    ges = unary_union([q.buffer(0.01, quad_segs=8).intersection(alle) for q, _ in stuecke]).simplify(0.0005)
    neu.setdefault(L, []).append((n, ges))
    for g0 in getattr(ges, 'geoms', [ges]):
        for g in ohne_loecher(g0):
            if g.geom_type != 'Polygon' or g.area < AMIN: continue
            out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], art='flaeche' if hat else 'pad'))
    stat[L] = stat.get(L, 0) + len(stuecke)
for (n, L), t in erst:
    for g in ohne_loecher(t.simplify(0.0005)):
        if g.geom_type == 'Polygon' and g.area > AMIN / 2:
            out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], art='durchgang1'))
json.dump(out, open(sys.argv[2], 'w'))
print('Durchgang 1 zugeschnitten', len(erst), '| Nachlauf-Fillets', sum(stat.values()), stat, '| Formen', len(out))
