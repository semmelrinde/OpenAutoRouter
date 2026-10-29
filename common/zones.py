from collections import defaultdict

from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def unify(R):
    if not R.get('zone_list'):
        return R
    groups = defaultdict(list)
    priorities = {}
    for z in R['zone_list']:


        key = z['net'], z['layer'], z.get('source') or ''

        priorities[key] = max(priorities.get(key, 0), max(0, z.get('prio', 1)))
        groups[key].extend(
            Polygon(p['outer'], p.get('inner') or []).buffer(0) for p in z['polys'])
    zones = list(R.get('zones') or [])
    for (net, layer, source), parts in sorted(groups.items()):



        g = unary_union(parts).simplify(.0005, preserve_topology=True)
        contours = []
        for p in sorted(getattr(g, 'geoms', [g]), key=lambda p: (p.bounds, p.area)):
            if p.is_empty or p.geom_type != 'Polygon':
                continue
            p = orient(p, sign=1)
            contours.append(dict(pts=list(p.exterior.coords), holes=[list(h.coords) for h in p.interiors]))
        import uuid
        zone_id = str(uuid.uuid5(uuid.NAMESPACE_URL, 'autorouter:r36:'+net+':'+layer+':'+source))
        for contour in contours:
            zones.append(dict(net=net, layer=layer, **contour, filled=True,
                              full=True, current_path=True, r36=True, prio=priorities[net, layer, source],
                              zone_id=zone_id, native_layers=[layer], native_outlines=contours,
                              min_thickness=.025, local_clearance=None,
                              source='R36-FullCopperModel', zones_source=source, model=True))
    return dict(R, zones=zones, zone_list=[], current_zones_model='r36-dc-v1')


def active(R):
    return bool(R.get('zone_list') or R.get('current_zones_model') or
                any(z.get('r36') for z in R.get('zones', [])))


def board_geometry(B, R):
    if R.get("rest_board"):
        from postprocessing.remaining_conflicts import effective_board
        B = effective_board(B, R)
    native = R.get('native_pad_geometry')
    if native is None:
        return B
    from collections import Counter
    def key(p):
        return (p['ref'], str(p['num']), p['net'], round(p['x'],4), round(p['y'],4), tuple(sorted(p['layers'])))
    if Counter(map(key, B['pads'])) != Counter(map(key, native)):
        moves=(R.get('native_repair') or {}).get('offset') or {}
        if not moves or not str(B.get('src','')).startswith('task seed='):
            raise ValueError('Native pad geometry belongs to other pad positions/nets/layers')
        from common.shift import relocate
        expected,_=relocate(B,moves)
        if Counter(map(key,expected['pads'])) != Counter(map(key,native)):
            raise ValueError('Declared task repair does not match native pad positions')
        B=expected
    return dict(B, pads=native, src='native KiCad pad contours from R36 refill')


def via_geometry(R):
    diameter, drill = map(float, (R.get('options') or {}).get('via', '0.45/0.20').split('/'))
    return dict(R, vias=[dict(v, diameter=float(v.get('diameter', v.get('d', diameter))),
                              drill=float(v.get('drill', drill))) for v in R.get('vias', [])])
