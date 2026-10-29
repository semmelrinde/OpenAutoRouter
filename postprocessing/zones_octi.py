import json, sys, math
from shapely.geometry import Polygon, MultiPolygon, LinearRing
from shapely.ops import unary_union
Z = json.load(open(sys.argv[1])); OUT = sys.argv[2]
opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
R = float(opt.get('r', '0.12')); TOL = float(opt.get('tol', '0.08')); HMAX = float(opt.get('hmax', '0.3'))


OCTI = opt.get('octi', '0') == '1'
ONLY = set(opt['only'].split(',')) if opt.get('only') else None
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
def ring_octi(coords):
    pts = list(coords)[:-1]
    n = len(pts)
    if n < 3: return None
    lines = []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]; L = math.hypot(dx, dy)
        if L < 1e-6: continue
        k = int(round(math.degrees(math.atan2(dy, dx)) / 45.0)) % 8
        ux, uy = math.cos(math.radians(45 * k)), math.sin(math.radians(45 * k))
        nx, ny = -uy, ux; mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        lines.append([k, nx * mx + ny * my, L])

    changed = True
    while changed and len(lines) >= 3:
        changed = False
        out = []
        for ln in lines:
            if out and out[-1][0] == ln[0]:
                p = out[-1]; Ls = p[2] + ln[2]; p[1] = (p[1] * p[2] + ln[1] * ln[2]) / Ls; p[2] = Ls; changed = True
            elif out and (out[-1][0] - ln[0]) % 8 == 4:
                if ln[2] < out[-1][2]: changed = True; continue
                out[-1] = ln; changed = True
            else: out.append(ln)
        if len(out) >= 2 and out[0][0] == out[-1][0]:
            p = out[0]; q = out.pop(); Ls = p[2] + q[2]; p[1] = (p[1] * p[2] + q[1] * q[2]) / Ls; p[2] = Ls; changed = True
        lines = out
    if len(lines) < 3: return None
    verts = []
    for i in range(len(lines)):
        k1, c1, _ = lines[i]; k2, c2, _ = lines[(i + 1) % len(lines)]
        n1 = (-math.sin(math.radians(45 * k1)), math.cos(math.radians(45 * k1))); n2 = (-math.sin(math.radians(45 * k2)), math.cos(math.radians(45 * k2)))
        det = n1[0] * n2[1] - n1[1] * n2[0]
        if abs(det) < 1e-9: return None
        verts.append(((c1 * n2[1] - c2 * n1[1]) / det, (n1[0] * c2 - n2[0] * c1) / det))
    return verts
def octi(P):
    g = P.buffer(R, join_style=2).buffer(-2 * R, join_style=2).buffer(R, join_style=2)
    if g.is_empty: return None
    part_list = []
    for gg in getattr(g, 'geoms', [g]):
        gg = gg.simplify(TOL, preserve_topology=True)
        ex = ring_octi(gg.exterior.coords)
        if ex is None: return None
        holes = [h2 for h in gg.interiors if (h2 := ring_octi(h.coords)) is not None and Polygon(h2).area > 0.05]
        q = Polygon(ex, holes).buffer(0)
        part_list.append(q)
    q = unary_union(part_list)
    if q.is_empty or not q.is_valid: return None
    if q.hausdorff_distance(P) > HMAX or abs(q.area - P.area) > 0.15 * P.area + 0.2: return None
    return q
def vacant(P):
    g = P.buffer(R, quad_segs=8).buffer(-2 * R, quad_segs=8).buffer(R, quad_segs=8)
    if g.is_empty: return None
    g = g.simplify(TOL, preserve_topology=True)
    g = g.buffer(-R, quad_segs=8).buffer(R, quad_segs=8).buffer(R, quad_segs=8).buffer(-R, quad_segs=8)
    part_list = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area > 0.05]
    q = unary_union(part_list) if part_list else None
    if q is None or q.is_empty or not q.is_valid: return None
    if q.hausdorff_distance(P) > HMAX or abs(q.area - P.area) > 0.15 * P.area + 0.2: return None
    return q
plan = dict(keepouts_way=[], zone_list={}); ok = fail = 0
for z in Z:
    if z['rule'] or z['net'] == 'GND' or (ONLY and z['uuid'] not in ONLY): continue
    P = unary_union([poly(p) for p in z['polys']])
    q = octi(P) if OCTI else vacant(P)
    if q is None: fail += 1; print('  unchanged:', z['net'], z['cu_layers'], round(P.area, 2)); continue
    ok += 1
    plan['zone_list'][z['uuid']] = [dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors])
                                for gg in getattr(q, 'geoms', [q])]
json.dump(plan, open(OUT, 'w')); print('%s:' % ('straightened 0/45/90' if OCTI else 'freely shaped'), ok, '| unchanged:', fail)
