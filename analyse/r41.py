""
import math
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, nearest_points


def _differenz(a, b):
    ""
    try:
        return a.difference(b)
    except shapely.errors.GEOSException:
        return shapely.difference(shapely.make_valid(a), shapely.make_valid(b), grid_size=0.000001)


def netze(K):
    nk = K.get('netze') or {}
    schnell = {n for n, k in nk.items() if k.get('art') == 'sw' or
               (k.get('art') == 'schnell' and k.get('rang', 0) >= 3)}
    andere = {n for n, k in nk.items() if k.get('art') not in ('strom', 'versorgung', 'gnd')}
    return schnell, andere - set(K.get('gnd') or [])


def parameter(K, optionen=None):
    o = dict(K.get('regeln') or {}); o.update(optionen or {})
    clr = float(o.get('clr', 0.13))
    via = o.get('via', '0.45/0.20')
    dia = float(via.split('/')[0]) if isinstance(via, str) else float(via[0])
    reserve = float(o.get('r41_reserve', 0.05))

    fill = max(0.15, clr)
    minimum = float(o.get('r41_min_breite', 0.25))
    return dict(clr=clr, fill_clr=fill, via_d=dia, reserve=reserve,
                dicht=clr + reserve, spalt_ab=max(clr + reserve, 2 * fill + minimum),
                via_platz=dia + 2 * fill + reserve)


def _segmente(R):
    out = {}
    for t in R.get('tracks', []):
        for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            length = math.dist(a, b)
            if length < 1e-9: continue
            w = t['w'][i] if isinstance(t['w'], list) else t['w']
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            out.setdefault(t['layer'], []).append((t['net'], a, b, float(w), (b-a)/length, length))
    return out


def _offen(B, R, K, L, schnell, h, p):

    copper, rest, stitched = [], [], []
    for net in sorted(K.get('gnd') or ['GND']):
        cu, missing, covered = _offen_netz(B, R, L, schnell, h, p, net)
        copper.append(cu); rest.append(missing); stitched.append(covered)
    return unary_union(copper), unary_union(rest), unary_union(stitched)


def _offen_netz(B, R, L, schnell, h, p, net):
    from lernen.stromzonen import _geo, enden_stitchen
    native = [z for z in R.get('zones', []) if z['net'] == net and z['layer'] == L and z.get('filled') is True]
    cu = unary_union([Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in native] if native else
                     [_geo(z) for z in R.get('zonen', []) if z['net'] == net and z['layer'] == L])

    from analyse.kupfermodell import auf_lage
    anchors = [((v['x'], v['y']), Point(v['x'], v['y']).buffer(float(v.get('d', v.get('diameter', p['via_d'])))/2))
               for v in R.get('vias', []) if v.get('net') == net and auf_lage(v, L)]
    anchors += [((q['x'], q['y']), unary_union([Polygon(g).buffer(0) for g in q['polys'].get(L, [])]))
                for q in B.get('pads', []) if q.get('drill') and q['net'] == net and L in q['layers']]
    near = schnell.buffer(1.0)
    rest, stitched = [], []
    for s in getattr(cu, 'geoms', [cu]):
        if s.is_empty or s.geom_type != 'Polygon' or not s.intersects(near): continue
        aa = []
        for a, metal in anchors:
            if not s.intersects(metal): continue


            contact = Point(a) if s.covers(Point(a)) else nearest_points(s, Point(a))[0]
            aa.append((contact.x, contact.y))
        _, missing = enden_stitchen(s, s.intersection(near), aa, Polygon(), Polygon(), [], p['via_d'], p['fill_clr'],
                                   r=1.5, r_schnitt=1.5 + h, h=h)


        broad = s.buffer(-1.0, join_style=2).buffer(1.0, join_style=2).intersection(s)
        if missing is not None:
            rest.append(_differenz(missing.intersection(s), broad))
        stitched.append(_differenz(_differenz(s, broad).intersection(near), missing if missing is not None else Polygon()))
    return cu, unary_union(rest), unary_union(stitched)

def _fuelllagen(B, R, K):
    ""
    from analyse.flaechen import native_lagen
    native = native_lagen(B, R, K)
    gnd = set(K.get('gnd') or ['GND'])
    native.update(z['layer'] for z in R.get('zones', []) if z.get('filled') is True and z.get('net') in gnd)
    geometric = set(R.get('r41_fuellung_lagen') or [])
    geometric.update(z['layer'] for z in R.get('zonen', []) if z.get('net') in gnd)
    return native | geometric


def messen(B, R, K, schritt=0.1):
    ""
    o = R.get('optionen') or {}; p = parameter(K, dict(o, **({'clr': R['clr']} if R.get('clr') else {})))
    out = dict(version='r41-v7', aktiv=int(B.get('lagen', o.get('lagen', 2))) >= 4,
               schritt_mm=schritt, parameter=p, a_mm=0., b_mm=0., c_mm=0., d_mm=0., ohne_mm=0., schnell_mm=0.,
               fuellung_gemessen=False, vias=sum(bool(v.get('r41')) for v in R.get('vias', [])),
               geschnitten_mm2=(R.get('r41_fuellung_stat') or {}).get('geschnitten_mm2',
                   round(sum((z.get('r41') or {}).get('geschnitten_mm2', 0) for z in R.get('zonen', [])), 4)))
    if not out['aktiv']: return dict(out, fehler_mm=0.)
    fast, non_dc = netze(K)
    proven = _fuelllagen(B, R, K); required = set()
    for L, segs in sorted(_segmente(R).items()):
        fs = [s for s in segs if s[0] in fast]
        if not fs: continue
        required.add(L)
        cu, missing, stitched = _offen(B, R, K, L, unary_union([LineString([s[1],s[2]]).buffer(s[3]/2) for s in fs]), schritt, p)
        lines = [LineString([s[1], s[2]]) for s in segs]
        tree = shapely.STRtree(lines)
        for n, a, b, w, u, length in fs:
            out['schnell_mm'] += length
            num = max(1, math.ceil(length/schritt)); dl = length/num
            points = a + (np.arange(num)[:, None]+.5)*dl*u
            nachbar_radius = max(1.0, p['via_platz'])
            ids = tree.query(LineString([a,b]).buffer(nachbar_radius + w/2 + max(s[3] for s in segs)/2))
            candidates = [segs[i] for i in ids if segs[i][0] != n and segs[i][0] in non_dc and abs(np.dot(u,segs[i][4])) >= math.cos(math.radians(10))]
            for xy in points:
                labels = []

                if not missing.is_empty and missing.distance(Point(xy)) <= w/2 + p['fill_clr'] + .25:
                    labels.append('d')
                if not stitched.is_empty and stitched.distance(Point(xy)) <= w/2 + p['fill_clr'] + .25:
                    labels.append('a')
                sides = {}
                for _, aa, bb, ww, uu, ll in candidates:
                    t = float(np.dot(xy-aa,uu))
                    if not 0 <= t <= ll: continue
                    q = aa+t*uu; delta = q-xy
                    gap = float(np.linalg.norm(delta)) - (w+ww)/2
                    if gap < -1e-5 or gap > nachbar_radius: continue
                    side = 1 if u[0]*delta[1]-u[1]*delta[0] > 0 else -1
                    if side not in sides or gap < sides[side][0]: sides[side] = (gap,q,ww)
                for gap,q,ww in sides.values():
                    if gap <= p['dicht']+1e-6: labels.append('b'); continue
                    direction = (q-xy)/np.linalg.norm(q-xy)
                    cross = LineString([xy+direction*(w/2+p['fill_clr']), q-direction*(ww/2+p['fill_clr'])])
                    filled = cross.intersection(cu)
                    if not filled.is_empty and filled.length > .01 and filled.intersects(missing): labels.append('d')
                    elif filled.length >= .25 and filled.length >= cross.length-.02: labels.append('a')
                    elif p['spalt_ab'] + 1e-6 < gap < p['via_platz']: labels.append('c')
                label = next((x for x in ('d','c','b','a') if x in labels), 'ohne')
                out[label+'_mm'] += dl
    for k in ('a_mm','b_mm','c_mm','d_mm','ohne_mm','schnell_mm'): out[k] = round(out[k], 4)
    out['fehler_mm'] = round(out['c_mm'] + out['d_mm'], 4)
    out['fuellung_lagen'] = sorted(proven)
    out['fuellung_fehlende_lagen'] = sorted(required - proven)
    out['fuellung_gemessen'] = not bool(required - proven)
    if not out['fuellung_gemessen'] and out['schnell_mm'] > 0:
        out.update(a_mm=None, d_mm=None, fehler_mm=None, fehlend='GND-Fuellung nicht gemessen')
    return out
