""
from shapely.geometry import Polygon
from shapely.ops import unary_union

KUPFER = {'F': (196, 120, 60), 'B': (70, 120, 200)}
INNEN = [(60, 160, 100), (205, 160, 40), (150, 90, 200), (40, 165, 175), (200, 95, 130), (120, 130, 50)]
GRUND = (24, 36, 30)


def farbe(L):
    if L in KUPFER: return KUPFER[L]
    try: return INNEN[(int(L[2:]) - 1) % len(INNEN)]
    except (ValueError, IndexError): return (150, 150, 150)


def _via_masse(R, K=None):
    from analyse.kupfermodell import via_geo
    return via_geo(R, K)


def _gnd(B, R, K=None):
    gl = R.get('gndlagen') or []
    gl = [str(q).split('=')[0] for q in (gl if isinstance(gl, list) else str(gl).split(',')) if q]
    netze = {p['net'] for p in B['pads'] if p['net']}
    gnd = set(K['gnd']) if K is not None and 'gnd' in K else {n for n in netze if n.lstrip('/').upper() in ('GND', 'PGND', 'AGND', 'DGND', 'GNDA', 'GNDD')}
    return gl, gnd


def kupfer_lagen(B, R, clr=None, rand=None, min_breite=0.2, fuellen=True, K=None, regeln=None):
    ""
    from gemeinsam.zonen import boardgeometrie
    B = boardgeometrie(B, R)
    nl = int(B.get('lagen') or 2)
    lagen = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
    from analyse.kupfermodell import einstellungen, flaechen, bezug, geometrie, bohrungen, sperrflaeche, angeschlossen
    clr, rand = einstellungen(K, regeln, clr, rand)
    planes, gl, gnd = flaechen(B, R, K)
    d_via, b_via = _via_masse(R, K)
    fills = {L: Polygon() for L in lagen}
    if fuellen:

        alle = {}
        for net in sorted(set(planes.values()) | gnd):
            kandidaten = {}
            for L in lagen:
                if not (planes.get(L) == net or (L not in gl and net in gnd)): continue
                f = bezug(B, R, L, {net}, clr=clr, rand=rand, via_d=d_via, drill=b_via)
                if L not in gl: f = f.buffer(-min_breite / 2, 8).buffer(min_breite / 2, 8)
                kandidaten[L] = f
            alle[net] = kandidaten


        from shapely import voronoi_polygons
        from shapely.geometry import MultiPoint
        for L in lagen:
            aktive = [n for n, fs in alle.items() if L in fs and not fs[L].is_empty]
            if len(aktive) < 2: continue
            punkte = {}
            for n, g in geometrie(B, R, L, d_via):
                if n in aktive and not g.is_empty:
                    q = g.representative_point(); xy = (q.x, q.y)
                    if xy in punkte and punkte[xy] != n:
                        raise ValueError('Getrennte Massen haben denselben Kupferanker')
                    punkte[xy] = n
            if len(set(punkte.values())) < 2:

                for n in aktive:
                    if n not in punkte.values(): alle[n][L] = Polygon()
                continue
            coords = sorted(punkte)
            zellen = voronoi_polygons(MultiPoint(coords), extend_to=unary_union([alle[n][L] for n in aktive]).envelope, ordered=True)
            rollen = {n: [] for n in aktive}
            for xy, zelle in zip(coords, zellen.geoms): rollen[punkte[xy]].append(zelle)
            for n in aktive:
                gebiet = unary_union(rollen[n]).buffer(-clr / 2, join_style=2)
                alle[n][L] = alle[n][L].intersection(gebiet)
        for net, kandidaten in alle.items():
            verbunden = angeschlossen(B, R, kandidaten, {L: net for L in kandidaten}, via_d=d_via, drill=b_via)
            for L, f in verbunden.items(): fills[L] = fills[L].union(f)
    out = {}
    for L in lagen:
        bohr = bohrungen(B, R, b_via, L)
        teile = geometrie(B, R, L, d_via)
        kupfer = unary_union([g for n, g in teile]) if teile else Polygon()
        kupfer = kupfer.union(fills[L])
        for k in B.get('keepouts', []):
            if (k.get('verbote') or {}).get('flaechen') and L in (k.get('lagen') or lagen):
                sperre = sperrflaeche(k)
                kupfer = kupfer.difference(sperre.difference(unary_union([g for n, g in teile])))
        out[L] = (kupfer.difference(bohr), bohr)
    return out


def bild_lage(B, R, L, geom, ziel, breite=420, ss=3, titel=None):
    ""
    from PIL import Image, ImageDraw
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0, x1, y1 = min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5
    S = breite * ss / max(1e-6, x1 - x0); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    im = Image.new('RGB', (W, H), (245, 245, 242)); dr = ImageDraw.Draw(im)
    tr = lambda x, y: ((x - x0) * S, (y - y0) * S)
    kupfer, bohr = geom; c = farbe(L)
    def poly(g, f, loch):

        for p in sorted(getattr(g, 'geoms', [g]), key=lambda p: -p.area):
            if p.is_empty or p.geom_type != 'Polygon': continue
            dr.polygon([tr(*q) for q in p.exterior.coords], fill=f)
            for i in p.interiors: dr.polygon([tr(*q) for q in i.coords], fill=loch)
    from analyse.kupfermodell import umriss
    poly(umriss(B), GRUND, (245, 245, 242))
    poly(kupfer, c, GRUND)
    poly(bohr, (10, 10, 10), (10, 10, 10))


    for holes in B.get('outline_holes') or []:
        for h in holes:
            if len(h) >= 3: dr.polygon([tr(*q) for q in h], fill=(245, 245, 242))
    from lernen.galerie import _luftlinien
    _luftlinien(dr, R, tr, ss)
    im = im.resize((W // ss, H // ss), Image.LANCZOS)
    ImageDraw.Draw(im).text((4, 3), titel or L, fill=(255, 255, 255))
    im.save(ziel, optimize=True)
