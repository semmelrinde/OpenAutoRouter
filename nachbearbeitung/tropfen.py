""
import math
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree

R_FAKTOR, R_MIN, R_MAX, R_T = 0.5, 0.15, 0.5, 0.25
FREI = float(__import__("os").environ.get("TROPFEN_FREI", "0.11"))

LOCH = 0.21
RAND = 0.3
AMIN = 0.0008
SCHRUMPF = float(__import__("os").environ.get("FILLET_SCHRUMPF", "0"))

SKIP_REF = ('NT_', 'NTBAR', 'VOR')


def ohne_loecher(g):
    ""
    from shapely.ops import split
    from shapely.geometry import LineString as _LS
    if g.geom_type != 'Polygon' or not g.interiors: return [g]
    h = g.interiors[0]; cx = sum(x for x, _ in h.coords[:-1]) / (len(h.coords) - 1)
    out = []
    for t in split(g, _LS([(cx, g.bounds[1] - 1), (cx, g.bounds[3] + 1)])).geoms:
        out.extend(ohne_loecher(t))
    return out


def _bogen(a, m, c, sag=0.002):
    ""
    ax, ay = a; bx, by = m; cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12: return [a, c]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    rr = math.hypot(ax - ux, ay - uy)
    t0, t1, tm = (math.atan2(p[1] - uy, p[0] - ux) for p in (a, c, m))
    def dif(x, y): return (y - x) % (2 * math.pi)
    span = dif(t0, t1) if dif(t0, tm) < dif(t0, t1) else -dif(t1, t0)
    schritt = 2 * math.acos(max(-1.0, 1 - sag / rr)) if rr > sag else math.pi / 4
    n = min(64, max(2, math.ceil(abs(span) / schritt)))
    return [(ux + rr * math.cos(t0 + span * k / n), uy + rr * math.sin(t0 + span * k / n)) for k in range(n + 1)]


def _bahn(t):
    ""
    st = []
    if 'items' in t:
        for it in t['items']:
            if it[0] == 'L': st.append(([it[1], it[2]], it[3]))
            else: st.append((_bogen(it[1], it[2], it[3]), it[4]))
    else:
        for a, c, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): st.append(([a, c], w))
    return [(p, w) for p, w in st if LineString(p).length > 1e-6]


def tropfen(D, R, via_d=0.45, nets_aus=(), lagen=None, voll=(), zclr=None, flaechen=None):
    ""
    feat, fremd, loecher = {}, {}, []
    for p in D['pads']:
        if not p['net']:
            if p.get('drill', 0) > 0: loecher.append(Point(p['x'], p['y']).buffer(p['drill'] / 2, 24))
            continue
        for L in p['layers']:
            if L not in p['polys']: continue
            pg = unary_union([Polygon(q) for q in p['polys'][L]])
            fremd.setdefault(L, []).append((p['net'], pg))
            if not p['ref'].startswith(SKIP_REF):
                b = pg.minimum_rotated_rectangle.exterior.coords
                feat.setdefault((p['net'], L), []).append(dict(g=pg.buffer(-SCHRUMPF), D=min(math.dist(b[0], b[1]), math.dist(b[1], b[2])), art='pad'))
    alle_lagen = set(fremd) | {t['layer'] for t in R['tracks']}
    for v in R['vias']:
        vg = Point(v['x'], v['y']).buffer(via_d / 2, 32)
        for L in alle_lagen:
            fremd.setdefault(L, []).append((v['net'], vg))
            feat.setdefault((v['net'], L), []).append(dict(g=vg.buffer(-SCHRUMPF), D=via_d, art='via'))
    for t in R['tracks']:
        st = _bahn(t)
        if not st: continue
        g = unary_union([LineString(p).buffer(w / 2, quad_segs=8) for p, w in st])
        fremd.setdefault(t['layer'], []).append((t['net'], g))
        gk = unary_union([LineString(p).buffer(w / 2 - SCHRUMPF, quad_segs=8) for p, w in st])
        feat.setdefault((t['net'], t['layer']), []).append(dict(g=gk, D=max(w for _, w in st), art='bahn', st=st))
    zclr = zclr or {}
    for (n, L), gs in (flaechen or {}).items():
        for g in gs:
            feat.setdefault((n, L), []).append(dict(g=g.buffer(-SCHRUMPF), D=1e9, art='flaeche'))
    baum = {L: (STRtree([g for _, g in v]), [n for n, _ in v]) for L, v in fremd.items()}
    lochbaum = STRtree(loecher) if loecher else None
    rand = unary_union([Polygon(o) for o in D['outline']]).buffer(-RAND)

    def frei(g, net, L):
        if not rand.contains(g): return False
        ab = max(FREI, zclr.get((net, L), 0) + 0.01)
        if lochbaum is not None:
            for i in lochbaum.query(g.buffer(LOCH)):
                if lochbaum.geometries.take(i).distance(g) < LOCH: return False
        tr, nets = baum[L]
        for i in tr.query(g.buffer(ab)):
            if nets[i] != net and tr.geometries.take(i).distance(g) < ab: return False
        return True

    def ecken(lok, r, muss):
        ""
        zu = lok.buffer(r, quad_segs=16).buffer(-r, quad_segs=16)
        f = zu.difference(lok)
        out = []
        for q in getattr(f, 'geoms', [f]):
            if q.geom_type != 'Polygon' or q.area < AMIN: continue
            if any(m.distance(q) > 0.003 for m in muss): continue
            k = q.buffer(0.001).intersection(lok.buffer(0.003))
            if len(getattr(k, 'geoms', [k])) != 1: continue
            _lk = lok.buffer(0.002)
            teile = [p for p in getattr(_lk, 'geoms', [_lk]) if p.distance(q) < 0.1]
            if len(teile) > 1: continue
            x0, y0, x1, y1 = q.bounds
            if math.hypot(x1 - x0, y1 - y0) > 3 * r or q.area > 1.2 * r * r: continue
            out.append(q)
        return out

    out = []
    for (n, L), F in feat.items():
        if n in nets_aus or (n, L) in voll or (lagen and L not in lagen): continue
        U = unary_union([f['g'] for f in F])
        tr = STRtree([f['g'] for f in F])
        stellen = []
        for i, a in enumerate(F):
            for j in tr.query(a['g'].buffer(0.003)):
                if j <= i: continue
                b = F[j]
                if a['art'] == 'flaeche' and b['art'] == 'flaeche': continue
                if a['g'].distance(b['g']) > 0.002: continue
                I = a['g'].intersection(b['g'])
                if I.is_empty or I.area < 1e-6: I = a['g'].buffer(0.002).intersection(b['g'].buffer(0.002))
                nb = [x for x in (a, b) if x['art'] in ('pad', 'via')]
                if nb: r0 = R_FAKTOR * min(x['D'] for x in nb); art = 'via' if all(x['art'] == 'via' for x in nb) else 'pad'
                else: r0 = max(R_T, R_FAKTOR * max(x['D'] for x in (a, b) if x['art'] == 'bahn')); art = 'T'
                stellen.append((I, min(max(r0, R_MIN), R_MAX), [a['g'], b['g']], art))
            if a['art'] == 'bahn':
                for p, w in a['st']:
                    for k in range(1, len(p) - 1):
                        v1 = (p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); v2 = (p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1])
                        n1, n2 = math.hypot(*v1), math.hypot(*v2)
                        if n1 < 1e-6 or n2 < 1e-6: continue
                        if (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2) > math.cos(math.radians(12)): continue
                        stellen.append((Point(p[k]).buffer(w / 2), min(max(R_FAKTOR * w, R_MIN), R_MAX), [a['g']], 'knick'))
                for k in range(len(a['st']) - 1):
                    p, w = a['st'][k]; p2, w2 = a['st'][k + 1]
                    if math.dist(p[-1], p2[0]) > 0.01: continue
                    v1 = (p[-1][0] - p[-2][0], p[-1][1] - p[-2][1]); v2 = (p2[1][0] - p2[0][0], p2[1][1] - p2[0][1])
                    n1, n2 = math.hypot(*v1), math.hypot(*v2)
                    if n1 < 1e-6 or n2 < 1e-6: continue
                    if (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2) > math.cos(math.radians(12)) and abs(w - w2) < 0.01: continue
                    stellen.append((Point(p[-1]).buffer(max(w, w2) / 2), min(max(R_FAKTOR * max(w, w2), R_MIN), R_MAX), [a['g']], 'knick'))
        stuecke = []
        for I, r0, muss, art in stellen:
            W = I.buffer(2.5 * r0 + 0.25, quad_segs=16)
            lok = unary_union(muss).intersection(W)

            offen = ecken(lok, r0, muss)
            if not offen: continue
            r = r0
            while offen and r >= 0.1:
                qs = offen if r == r0 else ecken(lok, r, muss)
                rest = []
                for o in offen:
                    m = [q for q in qs if q.intersects(o)]
                    if not m: continue
                    q = max(m, key=lambda q: q.intersection(o).area)
                    if frei(q, n, L): stuecke.append((q, r, art))
                    else: rest.append(o)
                offen = rest; r *= 0.75
        if not stuecke: continue
        try: alle = unary_union([U] + [q for q, _, _ in stuecke])
        except Exception:
            from shapely.validation import make_valid
            import shapely; alle = shapely.union_all([make_valid(g) for g in [U] + [q for q, _, _ in stuecke]], grid_size=1e-6)

        OE = float(__import__("os").environ.get("FILLET_OEFFNEN", "0.006"))
        stuecke = [((q.buffer(-OE, quad_segs=8).buffer(OE, quad_segs=8) if OE else q), r, art) for q, r, art in stuecke]
        stuecke = [(q, r, art) for q, r, art in stuecke if not q.is_empty and q.area > AMIN / 2]
        if not stuecke: continue
        def _schnitt(q):
            g = q.buffer(0.01, quad_segs=8)
            try: return g.intersection(alle)
            except Exception:
                from shapely.validation import make_valid
                try: return make_valid(g).intersection(make_valid(alle))
                except Exception: return g.intersection(alle.buffer(0), grid_size=1e-6)
        ges = unary_union([_schnitt(q) for q, _, _ in stuecke]).simplify(0.0005)
        for g0 in getattr(ges, 'geoms', [ges]):
            for g in ohne_loecher(g0):
                if g.geom_type != 'Polygon' or g.area < AMIN: continue
                beteiligt = [(r, art) for q, r, art in stuecke if q.intersects(g)]
                if not beteiligt: continue
                arten = {a for _, a in beteiligt}
                art = 'pad' if 'pad' in arten else 'via' if 'via' in arten else 'T' if 'T' in arten else 'knick'
                out.append(dict(net=n, layer=L, pts=[list(c) for c in g.exterior.coords][:-1], art=art,
                                r=round(min(r for r, _ in beteiligt), 3)))
    return out
