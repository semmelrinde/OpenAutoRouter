from collections import defaultdict
from shapely.geometry import MultiPoint, Point, Polygon
from shapely.ops import unary_union

VERSION = 'k17-free-notches-v2'


def measure(B, R, K, radius=.45):
    from learning import current_zones as SZ
    clr = max(.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or .1))
    hole = float((R.get('options') or {}).get('h2h') or .2)
    _, drill = SZ._via_ground(R)
    board = unary_union([Polygon(q).buffer(0) for q in B['outline'] if len(q) >= 3]).buffer(-.25, 32)
    zs = defaultdict(list)
    null_rings = 0
    contact_cleanups = 0
    for z in R.get('zone_list', []):
        if z.get('source') == 'current_zones':
            polys = []
            for p in z.get('polys') or []:
                holes = []
                for ring in p.get('inner') or []:




                    if not ring or MultiPoint(ring).convex_hull.area == 0:
                        null_rings += 1
                    else:
                        holes.append(ring)
                poly = Polygon(p['outer'], holes)
                if poly.area <= 0:
                    raise ValueError('K17: empty or invalid zone geometry')
                if not poly.is_valid:




                    clean = poly.buffer(0)
                    if clean.is_empty or abs(clean.area-poly.area) > 1e-9:
                        raise ValueError('K17: invalid contour with area change')
                    contact_cleanups += 1
                    poly = clean
                polys.append(poly)
            if not polys:
                raise ValueError('K17: empty or invalid zone geometry')
            zs[z['net'], z['layer']].append(unary_union(polys))
    if not zs:
        return dict(version=VERSION, status='without_current_zones', complete=False,
                    free_notches_mm2=None, narrow_extra_area_mm2=None, per_net_layer=[])
    details = []


    measure = dict(R, zone_list=[z for z in R.get('zone_list', []) if z.get('source') != 'gnd_fill'],
                   vias=[v for v in R.get('vias', []) if v.get('source') != 'gnd_fill'])
    raw = dict(measure, zone_list=[z for z in measure.get('zone_list', []) if z.get('source') != 'current_zones'])
    for (n, L), shapes in sorted(zs.items()):
        g = unary_union(shapes)
        parts = SZ._parts(B, measure, L)
        foreign = unary_union([q for m, q in parts if m != n])
        holes = unary_union([Point(v['x'], v['y']).buffer(float(v.get('drill') or drill)/2, 16)
                             for v in measure.get('vias', []) if v.get('net') != n and L in (v.get('layers') or SZ._layers(B))] +
                            [Point(p['x'], p['y']).buffer(p['drill']/2, 16)
                             for p in B['pads'] if p.get('drill') and p.get('net') != n])
        free = board.difference(foreign.union(holes).buffer(clr+.003, 16))
        free = free.difference(holes.buffer(hole+.003, 16)).difference(SZ.keepout_area(B, L, 'areas'))
        if SZ.track_protection_on(R):
            free = free.difference(SZ._track_protection(R, L, n)[1])
        free = free.buffer(-.002, 8)
        own = unary_union([q for m, q in SZ._parts(B, raw, L) if m == n])
        closing = g.buffer(radius, 32).buffer(-radius, 32).difference(g).intersection(free)
        thin = g.difference(g.buffer(-radius, 32).buffer(radius, 32)).difference(own)
        details.append(dict(net_name=n, layer_name=L, free_notches_mm2=round(closing.area, 6),
                            narrow_extra_area_mm2=round(thin.area, 6),
                            area_mm2=round(g.area, 6), perimeter_mm=round(g.length, 6)))
    fields = ('free_notches_mm2', 'narrow_extra_area_mm2', 'area_mm2', 'perimeter_mm')
    return dict(version=VERSION, status='measured', complete=True, radius_mm=radius,
                **{k: round(sum(x[k] for x in details), 6) for k in fields}, per_net_layer=details,
                rest_gnd_distributable=True, null_rings_removed=null_rings,
                contact_contours_cleaned=contact_cleanups,
                copper_contract='current_zones._geo: buffer(0), area change <= 1e-9 mm2',
                native_contour_valid=None)
