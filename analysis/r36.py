import math

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def measure(B, R, K):
    from common.zones import board_geometry, unify
    from learning.current_zones import _layers, _parts, current_nets, _current_known, I_ZONE
    from analysis.wobble import chain_list
    B = board_geometry(B, R)
    R = unify(R)


    nets = set(current_nets(K, set(K.get('gnd') or ['GND'])))
    zoned = {z['net'] for z in R.get('zones', []) if z.get('filled') and len(z.get('pts', [])) >= 3
             and Polygon(z['pts'], z.get('holes', [])).area > 0}
    nozone = sorted(nets - zoned)
    required_missing = [n for n in nozone if (_current_known(K['net_list'][n]) or 0.) >= I_ZONE]
    missing, length, pairs, corners, acute, excess, count = 0., 0., 0, 0, 0, 0., 0
    from analysis.copper_model import contour
    outline = contour(B)
    areas, tails = {}, 0.
    chains = chain_list(dict(R, tracks=[t for t in R.get('tracks', [])
                                   if not t.get('kelvin') and not t.get('tap')]))
    for L in ('F', 'B'):
        if L not in _layers(B): continue
        parts = _parts(B, R, L)
        core_parts = _parts(B, dict(R, zones=[], zone_list=[]), L)
        layerzones = []
        for n in sorted(nets):
            tracks = [t for t in R.get('tracks', []) if t['net'] == n and t['layer'] == L
                      and not t.get('kelvin') and not t.get('tap')]
            lines = unary_union([LineString(t['pts']) for t in tracks if len(t['pts']) >= 2])
            pads = [unary_union([Polygon(q).buffer(0) for q in p.get('polys', {}).get(L, [])])
                    for p in B['pads'] if p['net'] == n and p.get('polys', {}).get(L)]
            padmask = unary_union(pads).buffer(.02)
            Z = unary_union([Polygon(z['pts'], z.get('holes', [])).buffer(0)
                             for z in R.get('zones', []) if z['net'] == n and z['layer'] == L and z.get('filled')])
            layerzones.append(Z)
            core = unary_union([g for m,g in core_parts if m == n])
            tails += Z.difference(core.buffer(.75, quad_segs=32)).area
            core_line = lines.difference(padmask)
            length += core_line.length
            missing += core_line.difference(Z.buffer(.005)).length
            zparts = [p for p in getattr(Z, 'geoms', [Z]) if p.geom_type == 'Polygon' and not p.is_empty]
            for i, a in enumerate(pads):
                for b in pads[i+1:]:
                    if .005 < a.distance(b) <= 1.5:
                        pairs += not any(g.buffer(.005).intersects(a) and g.buffer(.005).intersects(b) for g in zparts)
            C = unary_union([g for m, g in parts if m == n])
            for poly in getattr(C, 'geoms', [C]):
                if poly.geom_type != 'Polygon' or poly.is_empty: continue
                poly = orient(poly.simplify(.002, preserve_topology=True), sign=1)
                for ring in [poly.exterior, *poly.interiors]:
                    xy = list(ring.coords[:-1])
                    for a, b, c in zip(xy[-1:]+xy[:-1], xy, xy[1:]+xy[:1]):
                        u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
                        if min(math.hypot(*u), math.hypot(*v)) < .03 or padmask.covers(Point(b)): continue
                        angle = math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1])
                        corners += abs(angle) >= math.radians(30)
                        acute += angle > math.pi/2+1e-4
            anchor = padmask.union(unary_union([Point(v['x'], v['y']).buffer(v.get('diameter', v.get('d', .45))/2+.02)
                                                for v in R.get('vias', []) if v['net'] == n and L in v.get('layers', _layers(B))]))
            for (net, layer), pts in chains:
                if (net, layer) != (n, L): continue
                path = LineString(pts)
                previous = None
                for i in range(math.ceil(path.length/.2)+1):
                    s = min(i*.2, path.length); p = path.interpolate(s)
                    if anchor.covers(p) or not C.covers(p):
                        previous = None; continue
                    width = 2*p.distance(C.boundary)
                    if previous is not None:
                        ds = s-previous[0]
                        excess += max(0., abs(width-previous[1])-ds)
                        count += 1
                    previous = s, width
        areas[L] = unary_union(layerzones).area
    fractions = {L:a/outline.area for L,a in areas.items()} if outline.area else {}
    outerarea = outline.area*len(areas)
    return dict(version='r36-shaping-v3', status='measured' if nets else 'no_current_nets',
                current_threshold_A=I_ZONE, nets_n=len(nets),
                nets_without_zone_n=len(nozone), nets_without_zone=nozone,
                from_200ma_without_zone_n=len(required_missing), from_200ma_without_zone=required_missing,
                copper_source='native_R36_Acceptance' if R.get('current_zones_acceptance', {}).get('applied') else 'without_native_R36_Acceptance',
                native_accepted=bool(R.get('current_zones_acceptance', {}).get('applied')),
                route_path_mm=round(length, 6), without_zone_mm=round(missing, 6),
                without_zone_share=round(missing/length, 6) if length else None,
                pad_pairs_without_common_zone=pairs, bends_n=corners, sharp_n=acute,
                transition_excess_mm=round(excess, 6), profile_intervals=count,
                zones_outer_mm2=round(sum(areas.values()), 6),
                zones_layers_mm2={L:round(a,6) for L,a in areas.items()},
                zones_share=round(sum(areas.values())/outerarea,6) if outerarea else None,
                zones_share_max=round(max(fractions.values(), default=0.),6) if fractions else None,
                compact_excess_mm2=round(max(0.,sum(areas.values())-.1*outerarea),6) if outerarea else None,
                runout_mm2=round(tails,6), runout_hem_mm=.75)
