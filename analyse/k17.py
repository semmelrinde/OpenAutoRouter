""
from collections import defaultdict
from shapely.geometry import MultiPoint, Point, Polygon
from shapely.ops import unary_union

VERSION = 'k17-freie-kerben-v2'


def messen(B, R, K, radius=.45):
    from lernen import stromzonen as SZ
    clr = max(.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or .1))
    hole = float((R.get('optionen') or {}).get('h2h') or .2)
    _, drill = SZ._via_masse(R)
    board = unary_union([Polygon(q).buffer(0) for q in B['outline'] if len(q) >= 3]).buffer(-.25, 32)
    zs = defaultdict(list)
    nullringe = 0
    kontaktbereinigungen = 0
    for z in R.get('zonen', []):
        if z.get('quelle') == 'stromzonen':
            polys = []
            for p in z.get('polys') or []:
                holes = []
                for ring in p.get('innen') or []:




                    if not ring or MultiPoint(ring).convex_hull.area == 0:
                        nullringe += 1
                    else:
                        holes.append(ring)
                poly = Polygon(p['aussen'], holes)
                if poly.area <= 0:
                    raise ValueError('K17: leere oder ungueltige Zonengeometrie')
                if not poly.is_valid:




                    clean = poly.buffer(0)
                    if clean.is_empty or abs(clean.area-poly.area) > 1e-9:
                        raise ValueError('K17: ungueltige Kontur mit Flaechenaenderung')
                    kontaktbereinigungen += 1
                    poly = clean
                polys.append(poly)
            if not polys:
                raise ValueError('K17: leere oder ungueltige Zonengeometrie')
            zs[z['net'], z['layer']].append(unary_union(polys))
    if not zs:
        return dict(version=VERSION, status='ohne_stromzonen', vollstaendig=False,
                    freie_kerben_mm2=None, schmale_zusatzflaeche_mm2=None, je_netz_lage=[])
    details = []


    measure = dict(R, zonen=[z for z in R.get('zonen', []) if z.get('quelle') != 'gnd_fuellung'],
                   vias=[v for v in R.get('vias', []) if v.get('quelle') != 'gnd_fuellung'])
    raw = dict(measure, zonen=[z for z in measure.get('zonen', []) if z.get('quelle') != 'stromzonen'])
    for (n, L), shapes in sorted(zs.items()):
        g = unary_union(shapes)
        parts = SZ._teile(B, measure, L)
        foreign = unary_union([q for m, q in parts if m != n])
        holes = unary_union([Point(v['x'], v['y']).buffer(float(v.get('drill') or drill)/2, 16)
                             for v in measure.get('vias', []) if v.get('net') != n and L in (v.get('layers') or SZ._lagen(B))] +
                            [Point(p['x'], p['y']).buffer(p['drill']/2, 16)
                             for p in B['pads'] if p.get('drill') and p.get('net') != n])
        free = board.difference(foreign.union(holes).buffer(clr+.003, 16))
        free = free.difference(holes.buffer(hole+.003, 16)).difference(SZ.sperrflaeche(B, L, 'flaechen'))
        if SZ.bahnschutz_an(R):
            free = free.difference(SZ._bahnschutz(R, L, n)[1])
        free = free.buffer(-.002, 8)
        own = unary_union([q for m, q in SZ._teile(B, raw, L) if m == n])
        closing = g.buffer(radius, 32).buffer(-radius, 32).difference(g).intersection(free)
        thin = g.difference(g.buffer(-radius, 32).buffer(radius, 32)).difference(own)
        details.append(dict(netz=n, lage=L, freie_kerben_mm2=round(closing.area, 6),
                            schmale_zusatzflaeche_mm2=round(thin.area, 6),
                            flaeche_mm2=round(g.area, 6), umfang_mm=round(g.length, 6)))
    fields = ('freie_kerben_mm2', 'schmale_zusatzflaeche_mm2', 'flaeche_mm2', 'umfang_mm')
    return dict(version=VERSION, status='gemessen', vollstaendig=True, radius_mm=radius,
                **{k: round(sum(x[k] for x in details), 6) for k in fields}, je_netz_lage=details,
                rest_gnd_verteilbar=True, nullringe_entfernt=nullringe,
                kontaktkonturen_bereinigt=kontaktbereinigungen,
                kupfervertrag='stromzonen._geo: buffer(0), flaechenaenderung <= 1e-9 mm2',
                native_kontur_gueltig=None)
