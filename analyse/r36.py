""
import math

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def messen(B, R, K):
    from gemeinsam.zonen import boardgeometrie, vereinheitlichen
    from lernen.stromzonen import _lagen, _teile, stromnetze, _strom_bekannt, I_ZONE
    from analyse.wackel import ketten
    B = boardgeometrie(B, R)
    R = vereinheitlichen(R)


    nets = set(stromnetze(K, set(K.get('gnd') or ['GND'])))
    zoned = {z['net'] for z in R.get('zones', []) if z.get('filled') and len(z.get('pts', [])) >= 3
             and Polygon(z['pts'], z.get('holes', [])).area > 0}
    nozone = sorted(nets - zoned)
    required_missing = [n for n in nozone if (_strom_bekannt(K['netze'][n]) or 0.) >= I_ZONE]
    missing, length, pairs, corners, acute, excess, count = 0., 0., 0, 0, 0, 0., 0
    from analyse.kupfermodell import umriss
    outline = umriss(B)
    areas, tails = {}, 0.
    chains = ketten(dict(R, tracks=[t for t in R.get('tracks', [])
                                   if not t.get('kelvin') and not t.get('abg')]))
    for L in ('F', 'B'):
        if L not in _lagen(B): continue
        parts = _teile(B, R, L)
        coreparts = _teile(B, dict(R, zones=[], zonen=[]), L)
        layerzones = []
        for n in sorted(nets):
            tracks = [t for t in R.get('tracks', []) if t['net'] == n and t['layer'] == L
                      and not t.get('kelvin') and not t.get('abg')]
            lines = unary_union([LineString(t['pts']) for t in tracks if len(t['pts']) >= 2])
            pads = [unary_union([Polygon(q).buffer(0) for q in p.get('polys', {}).get(L, [])])
                    for p in B['pads'] if p['net'] == n and p.get('polys', {}).get(L)]
            padmask = unary_union(pads).buffer(.02)
            Z = unary_union([Polygon(z['pts'], z.get('holes', [])).buffer(0)
                             for z in R.get('zones', []) if z['net'] == n and z['layer'] == L and z.get('filled')])
            layerzones.append(Z)
            core = unary_union([g for m,g in coreparts if m == n])
            tails += Z.difference(core.buffer(.75, quad_segs=32)).area
            coreline = lines.difference(padmask)
            length += coreline.length
            missing += coreline.difference(Z.buffer(.005)).length
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
                                                for v in R.get('vias', []) if v['net'] == n and L in v.get('layers', _lagen(B))]))
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
    return dict(version='r36-form-v3', status='gemessen' if nets else 'keine_stromnetze',
                stromschwelle_A=I_ZONE, netze_n=len(nets),
                netze_ohne_zone_n=len(nozone), netze_ohne_zone=nozone,
                ab_200ma_ohne_zone_n=len(required_missing), ab_200ma_ohne_zone=required_missing,
                kupferquelle='native_R36_Abnahme' if R.get('stromzonen_abnahme', {}).get('angewendet') else 'ohne_native_R36_Abnahme',
                native_abgenommen=bool(R.get('stromzonen_abnahme', {}).get('angewendet')),
                leitweg_mm=round(length, 6), ohne_zone_mm=round(missing, 6),
                ohne_zone_anteil=round(missing/length, 6) if length else None,
                padpaare_ohne_gemeinsame_zone=pairs, knicke_n=corners, spitze_n=acute,
                uebergang_exzess_mm=round(excess, 6), profilintervalle=count,
                zonen_aussen_mm2=round(sum(areas.values()), 6),
                zonen_lagen_mm2={L:round(a,6) for L,a in areas.items()},
                zonen_anteil=round(sum(areas.values())/outerarea,6) if outerarea else None,
                zonen_anteil_max=round(max(fractions.values(), default=0.),6) if fractions else None,
                kompakt_exzess_mm2=round(max(0.,sum(areas.values())-.1*outerarea),6) if outerarea else None,
                auslauf_mm2=round(tails,6), auslauf_saum_mm=.75)
