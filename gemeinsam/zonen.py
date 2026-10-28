""
from collections import defaultdict

from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def vereinheitlichen(R):
    ""
    if not R.get('zonen'):
        return R
    groups = defaultdict(list)
    priorities = {}
    for z in R['zonen']:


        key = z['net'], z['layer'], z.get('quelle') or ''

        priorities[key] = max(priorities.get(key, 0), max(0, z.get('prio', 1)))
        groups[key].extend(
            Polygon(p['aussen'], p.get('innen') or []).buffer(0) for p in z['polys'])
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
                              full=True, strompfad=True, r36=True, prio=priorities[net, layer, source],
                              zone_id=zone_id, native_layers=[layer], native_outlines=contours,
                              min_thickness=.025, local_clearance=None,
                              quelle='R36-Vollkupfermodell', zonen_quelle=source, modell=True))
    return dict(R, zones=zones, zonen=[], stromzonen_modell='r36-dc-v1')


def aktiv(R):
    return bool(R.get('zonen') or R.get('stromzonen_modell') or
                any(z.get('r36') for z in R.get('zones', [])))


def boardgeometrie(B, R):
    ""
    if R.get("rest_board"):
        from nachbearbeitung.restkonflikte import effektives_board
        B = effektives_board(B, R)
    native = R.get('native_padgeometrie')
    if native is None:
        return B
    from collections import Counter
    def key(p):
        return (p['ref'], str(p['num']), p['net'], round(p['x'],4), round(p['y'],4), tuple(sorted(p['layers'])))
    if Counter(map(key, B['pads'])) != Counter(map(key, native)):
        moves=(R.get('native_reparatur') or {}).get('versetzt') or {}
        if not moves or not str(B.get('src','')).startswith('aufgabe seed='):
            raise ValueError('Native Padgeometrie gehört zu anderen Padpositionen/Netzen/Lagen')
        from gemeinsam.verschieben import bewegen
        expected,_=bewegen(B,moves)
        if Counter(map(key,expected['pads'])) != Counter(map(key,native)):
            raise ValueError('Deklarierte Aufgaben-Reparatur stimmt nicht mit nativen Padpositionen ueberein')
        B=expected
    return dict(B, pads=native, src='native KiCad-Padkonturen aus R36-Neufuellung')


def viageometrie(R):
    ""
    diameter, drill = map(float, (R.get('optionen') or {}).get('via', '0.45/0.20').split('/'))
    return dict(R, vias=[dict(v, diameter=float(v.get('diameter', v.get('d', diameter))),
                              drill=float(v.get('drill', drill))) for v in R.get('vias', [])])
