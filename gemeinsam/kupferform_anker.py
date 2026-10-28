""
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
def anker_laden(C):
    ""
    A = {}; lagen = set()
    for t in C['tracks']:
        lagen.add(t['layer'])
        for p in (t['pts'][0], t['pts'][-1]): A.setdefault((t['net'], t['layer']), []).append(tuple(p))
    for p in C['pads']:
        for L in p['polys']: lagen.add(L); A.setdefault((p['net'], L), []).append((p['x'], p['y']))
    for z in C.get('zones', []):
        if z.get('layer'): lagen.add(z['layer'])
    for v in C['vias']:
        for L in lagen: A.setdefault((v['net'], L), []).append((v['x'], v['y']))
    return A
def anbinden(g, U, pts, w=0.01, reich=1.0):
    ""
    if not pts or g.is_empty: return g
    gb = g.buffer(1e-6)
    if any(gb.contains(Point(p)) for p in pts): return g
    erlaubt = U.union(g).buffer(0.002)
    for p in sorted(pts, key=lambda p: g.distance(Point(p)))[:12]:
        P = Point(p)
        if g.distance(P) > reich: break
        if not U.buffer(-w / 2).contains(P): continue
        q = nearest_points(g, P)[0]
        steg = LineString([q, P]).buffer(w, quad_segs=4).union(P.buffer(w, 8))
        if steg.difference(erlaubt).area < 1e-8: return g.union(steg.intersection(erlaubt))
    return g
