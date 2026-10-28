""
import copy, hashlib, json, math, sys
from collections import OrderedDict
from threading import RLock

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

VERSION = 'r39-kupfer-v4.3'
QS = 32
ARTEN = ('strom', 'versorgung', 'sw')
CACHE = OrderedDict()
CACHE_LOCK = RLock()
CACHE_N = 8
SCHRITT = .1


def lernwerte(e):
    ""
    if not e: return None
    if (e.get('abdeckung') or {}).get('komponenten') == 0:
        return dict(version=e.get('version'), abdeckung=e.get('abdeckung'))
    if e.get('nicht_messbar') or not e.get('abdeckung', {}).get('vollstaendig', True):
        out = dict(version=e.get('version'), abdeckung=e.get('abdeckung') or
                   dict(vollstaendig=False, unbekannt=len(e.get('nicht_messbar') or [])))


        if (e.get('gemessene_pfade') or {}).get('komponenten', 0) > 0: out['gemessene_pfade'] = copy.deepcopy(e['gemessene_pfade'])
        return out
    return e


def _streifen(pfad, W, s, frei, platz):


    ja, nein = [], []
    for line in getattr(pfad, 'geoms', [pfad]):
        if line.geom_type != 'LineString': continue
        coords = list(line.coords)
        for j, (a, b) in enumerate(zip(coords[:-1], coords[1:])):
            length = math.dist(a, b)
            if length <= 1e-12: continue
            dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length

            seg = LineString([a, b])
            if not seg.intersects(s.buffer(.02, QS)): continue
            count = max(1, math.ceil(length / SCHRITT))
            for i in range(count):
                lo, hi = length*i/count, length*(i+1)/count
                c = Point(a[0]+dx*(lo+hi)/2, a[1]+dy*(lo+hi)/2)
                if c.distance(s) > .02 + (hi-lo)/2: continue
                circ = c.buffer(W/2, QS)
                ratio = circ.intersection(frei).area / circ.area
                l = lo-W/2 if j == 0 and i == 0 else lo
                h = hi+W/2 if j == len(coords)-2 and i == count-1 else hi
                band = LineString([(a[0]+dx*l,a[1]+dy*l),(a[0]+dx*h,a[1]+dy*h)]).buffer(W/2, cap_style=2)
                (ja if ratio >= platz else nein).append(band)
    needed = s.intersection(unary_union(nein))
    unneeded = s.intersection(unary_union(ja)).difference(needed)
    return unneeded, needed


def _via_d(R):
    try: return float(str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')[0])
    except ValueError: return 0.45


def _lagen(B):
    nl = int(B.get('lagen') or 2)
    return ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']

def kupfer(B, R, L):
    ""
    from lernen.kupferbalken import kupfer as gesamtkupfer
    return [(n, g) for (n, layer), g in gesamtkupfer(B, R).items() if layer == L]


def engstellen(B, R, K, gndlagen=None, clr=None, rand=0.25, platz=0.85, w_max=None, min_flaeche=0.005):


    args = dict(gndlagen=gndlagen, clr=clr, rand=rand, platz=platz, w_max=w_max, min_flaeche=min_flaeche)
    key = hashlib.sha256(json.dumps([VERSION, B, R, K, args], sort_keys=True, allow_nan=False).encode()).digest()
    with CACHE_LOCK:
        if key in CACHE:
            CACHE.move_to_end(key)
            return copy.deepcopy(CACHE[key])
    out = _engstellen(B, R, K, **args)
    with CACHE_LOCK:
        CACHE[key] = copy.deepcopy(out)
        while len(CACHE) > CACHE_N: CACHE.popitem(last=False)
    return out


def _engstellen(B, R, K, gndlagen=None, clr=None, rand=0.25, platz=0.85, w_max=None, min_flaeche=0.005):
    ""
    from gemeinsam.zonen import boardgeometrie
    from lernen.kupferbalken import kupfer as gesamtkupfer
    from lernen.kupferbalken import track_geometry
    from lernen.stromzonen import sperrflaeche
    B = boardgeometrie(B, R)
    from shapely import set_precision
    cu = {key: set_precision(set_precision(g, 1e-6).simplify(2e-6, preserve_topology=True).normalize(), 0)
          for key, g in gesamtkupfer(B, R).items()}
    from gemeinsam.bogen import punkte
    mittellinien = {}
    bahnkupfer = {}
    for t in R.get('tracks', []):
        bahnkupfer.setdefault((t.get('net', ''), t['layer']), []).append(track_geometry(t))
        if 'items' in t:
            paths = [it[1:3] if it[0] == 'L' else punkte(*it[1:4]) for it in t['items']]
        else: paths = [t['pts']]
        mittellinien.setdefault((t.get('net', ''), t['layer']), []).extend(
            LineString(ps) for ps in paths if len(ps) >= 2)
    gl = gndlagen if gndlagen is not None else (R.get('gndlagen') or (R.get('optionen') or {}).get('gndlagen') or [])
    gl = {str(x).split('=')[0] for x in (gl if isinstance(gl, list) else str(gl).split(',')) if x}
    gnd = set(K.get('gnd') or ['GND'])
    clr = clr if clr is not None else float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1)
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand, QS)
    netze = {n: v for n, v in (K.get('netze') or {}).items() if v.get('art') in ARTEN and n not in gnd}
    zonenlagen = {(z['net'], z['layer']) for key in ('zonen', 'zones') for z in R.get(key, [])}

    from gemeinsam.zonen import vereinheitlichen
    zonenkupfer = {}
    for z in vereinheitlichen(R).get('zones', []):
        zonenkupfer.setdefault((z['net'], z['layer']), []).append(z)
    quellen = {n: {key: v.get(key) for key in ('art', 'grund', 'w', 'w_quelle', 'strom_A', 'strom_A_quelle')}
               for n, v in sorted(netze.items())}

    try: drill = float(str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')[1])
    except (ValueError, IndexError): drill = .2
    hole = float(R.get('h2h', (R.get('optionen') or {}).get('h2h', .2)))
    if not math.isfinite(hole) or hole < 0: raise ValueError('R39: ungueltiger Bohrungsabstand')
    unn, noetig, nicht_messbar, needed_area = [], 0, [], 0.0
    unn_gemessen = []
    components, measured = 0, 0
    for L in _lagen(B):
        if L in gl: continue
        teile = [(n, g) for (n, layer), g in cu.items() if layer == L]
        loecher = [(p.get('net', ''), Point(p['x'], p['y']).buffer(float(p['drill']) / 2, QS))
                   for p in B['pads'] if p.get('drill')]
        loecher += [(v.get('net', ''), Point(v['x'], v['y']).buffer(float(v.get('drill') or drill) / 2, QS))
                    for v in R.get('vias', []) if L in (v.get('layers') or _lagen(B))]
        for n, v in sorted(netze.items()):
            eigen = [g for m, g in teile if m == n]
            if not eigen: continue
            C = unary_union(eigen)
            W = float(v.get('w') or 0.3)
            if w_max is not None: W = min(w_max, W)
            if not math.isfinite(W) or W <= 0: raise ValueError('R39: ungueltige Sollbreite ' + n)
            pads = unary_union([Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n
                                for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3])

            radius = max(0, W / 2 - .005)
            from shapely import set_precision
            kern = set_precision(set_precision(C.buffer(-radius, QS), 1e-6), 0)
            duenn = C.difference(kern.buffer(radius, QS))
            if not pads.is_empty: duenn = duenn.difference(pads.buffer(0.02, QS))




            pfad = unary_union(mittellinien.get((n, L), []))
            covered, fully_measured = [], []
            for component in getattr(C, 'geoms', [C]):
                if component.geom_type != 'Polygon': continue
                has_path = not pfad.is_empty and pfad.intersects(component)

                if not has_path and (n, L) not in zonenlagen: continue


                zones = [Polygon(z['pts'], z.get('holes', [])).buffer(0) for z in zonenkupfer.get((n, L), [])
                         if z['net'] == n and z['layer'] == L and z.get('filled')]
                if not has_path and not any(z.intersection(component).area > 1e-9 for z in zones): continue
                components += 1
                if has_path:
                    covered.append(component)

                    broad = component.buffer(-radius, join_style=2).buffer(radius, join_style=2)


                    known = broad.union(unary_union(bahnkupfer.get((n, L), [])).buffer(.02, QS)).union(pads.buffer(.02, QS))
                    unknown = component.difference(known)
                    if unknown.area >= min_flaeche:
                        nicht_messbar.append(dict(netz=n, lage=L, grund='Zonen-Seitenast ausserhalb der gerouteten Mittellinie',
                                                 bounds=list(unknown.bounds), flaeche_mm2=round(unknown.area, 4)))
                    else:
                        measured += 1
                        fully_measured.append(component)
                else:
                    nicht_messbar.append(dict(netz=n, lage=L, grund='Kupferkomponente ohne geroutete Mittellinie',
                                             bounds=list(component.bounds), flaeche_mm2=round(component.area, 4)))
            duenn = duenn.intersection(unary_union(covered))
            known_components = unary_union(fully_measured).buffer(1e-9)
            stuecke = [s for s in getattr(duenn, 'geoms', [duenn]) if s.geom_type == 'Polygon' and s.area >= min_flaeche]
            if not stuecke: continue
            fremd = unary_union([g for m, g in teile if m != n])
            bohrungen = unary_union([g for m, g in loecher if m != n])
            frei = umriss.difference(fremd.buffer(clr, QS)).difference(sperrflaeche(B, L, 'flaechen'))
            frei = frei.difference(bohrungen.buffer(hole, QS)).union(C)
            for s in stuecke:
                hals = s.intersection(pfad.buffer(.02, QS))
                if hals.is_empty: continue
                yes, needed = _streifen(pfad, W, s, frei, platz)
                for piece in getattr(yes, 'geoms', [yes]):
                    if piece.geom_type != 'Polygon' or piece.area < min_flaeche: continue
                    c = piece.representative_point()
                    unn.append(dict(netz=n, lage=L, x=round(c.x, 3), y=round(c.y, 3),
                                    flaeche_mm2=round(piece.area, 4), soll_mm=round(W, 3)))
                    if known_components.covers(piece): unn_gemessen.append(unn[-1])
                for piece in getattr(needed, 'geoms', [needed]):
                    if piece.geom_type == 'Polygon' and piece.area >= min_flaeche:
                        noetig += 1; needed_area += piece.area
    return dict(version=VERSION, parameter=dict(clr=clr, rand=rand, platz=platz, w_max=w_max, h2h=hole,
                min_flaeche=min_flaeche, schritt_mm=SCHRITT, kern_praezision_mm=1e-6, kupfer_praezision_mm=1e-6, kupfer_glatt_mm=2e-6, gndlagen=sorted(gl), toleranz_mm=.005,
                sollbreiten={n: min(w_max, float(v.get('w') or .3)) if w_max is not None else float(v.get('w') or .3) for n, v in sorted(netze.items())}),
                abdeckung=dict(vollstaendig=not nicht_messbar, komponenten=components, messbar=measured,
                               unbekannt=len(nicht_messbar), anteil=measured/components if components else 1.0),
                gemessene_pfade=dict(version='r39-pfade-v3', komponenten=measured,
                    flaeche_mm2=round(sum(u['flaeche_mm2'] for u in unn_gemessen), 3), anzahl=len(unn_gemessen),
                    umfang='Nur vollständig messbare Kupferkomponenten mit gerouteter Mittellinie; keine unbekannten Verteilzonen/Seitenäste'),
                unnoetig=unn, unnoetig_n=len(unn), unnoetig_mm2=round(sum(u['flaeche_mm2'] for u in unn), 3),
                noetig=noetig, noetig_mm2=round(needed_area, 3), nicht_messbar=nicht_messbar, netze=quellen)



def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); p = [x for x in a if '=' not in x]
    B, R, K = (json.load(open(f)) for f in p[:3])
    E = engstellen(B, R, K, kw.get('gndlagen', '').split(',') if kw.get('gndlagen') else None)
    print(json.dumps(dict(E, unnoetig=E['unnoetig'][:15]), indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
