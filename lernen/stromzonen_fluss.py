""
import json, os, tempfile

import numpy as np
from shapely import box, union_all
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from lernen import stromzonen as SZ
from lernen import stromzonen_minimax as SM


def _karten(B, R, K):
    ""
    from analyse.kupfermodell import ir_werte
    from analyse.strom_kette import kette
    td = tempfile.mkdtemp(prefix='fluss_')
    ir = ir_werte(B, R, K, karten=td)
    KE = kette(B, R, K, ir=ir)
    M = {}
    for n, f in (ir.get('karten') or {}).items():
        if f and os.path.exists(os.path.join(td, f)):
            z = np.load(os.path.join(td, f)); M[n] = {k: z[k] for k in z.files}
    for f in os.listdir(td): os.remove(os.path.join(td, f))
    os.rmdir(td)
    return M, KE


def _zellen(M, L, maske):
    ""
    J = M.get(L + '_J_A_mm2')
    if J is None: return None
    xx, yy = M['x_mm'], M['y_mm']
    m = np.isfinite(J) & maske(J)
    if not m.any(): return None
    h = float(abs(xx[0, 1] - xx[0, 0])) if xx.ndim == 2 and xx.shape[1] > 1 else 0.1
    iy, ix = np.nonzero(m)
    return union_all(box(xx[iy, ix] - .55 * h, yy[iy, ix] - .55 * h, xx[iy, ix] + .55 * h, yy[iy, ix] + .55 * h))


def zonen(B, R, K, gndlagen=None, clr=None, rand=0.25, J_ziel=20.0, J_start=40.0, schritt=0.3, runden=8, tot=0.1,
          r_glatt=0.15, via_feld=True):
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gndlagen(R, gndlagen))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand, SZ.QS)
    _, b_via = SZ._via_masse(R)
    hole = float((R.get('optionen') or {}).get('h2h') or .2)
    zs = SM.zonen(B, R, K, gndlagen, clr=clr, rand=rand, J=J_start, via_feld=via_feld, tot_kuerzen=False)
    basis = [z for z in R.get('zonen', []) if z.get('quelle') != 'stromzonen']
    geo = {(z['net'], z['layer']): SZ._geo(z) for z in zs}
    meta = {(z['net'], z['layer']): z for z in zs}
    eigen = {}
    for L in SZ._lagen(B):
        if L in gl: continue
        for m, g in SZ._teile(B, dict(R, zonen=basis), L):
            if m in {n for n, _ in geo}: eigen.setdefault((m, L), []).append(g)
    eigen = {k: unary_union(v) for k, v in eigen.items()}
    loecher = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, 16)) for v in R.get('vias', [])]
    loecher += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, 16)) for p in B['pads'] if p.get('drill')]

    def frei(n, L):
        ""
        teile = SZ._teile(B, dict(R, zonen=basis), L)
        fremd = [g for m, g in teile if m != n] + [g for (m, LL), g in geo.items() if LL == L and m != n]
        fremd += [g for m, g in loecher if m != n]
        f = umriss.difference(unary_union(fremd).buffer(clr + .003, 16)).difference(SZ.sperrflaeche(B, L, 'flaechen'))
        f = SZ.maskenfrei(f, B, n, L, clr, 16)
        return f.difference(unary_union([g for m, g in loecher if m != n]).buffer(hole + .003, 16))

    verlauf = []
    for r in range(runden):
        RR = dict(R, zonen=basis + [dict(meta[k], polys=SZ._polys(g)) for k, g in geo.items()])
        M, KE = _karten(B, RR, K)
        heiss = []
        for (n, L), g in geo.items():
            if n not in M: continue
            h = _zellen(M[n], L, lambda J: J > J_ziel)
            if h is not None and not h.is_empty:
                jm = float(np.nanmax(M[n][L + '_J_A_mm2'])) if (L + '_J_A_mm2') in M[n] else 0.0
                heiss.append((jm, n, L, h))
        verlauf.append(dict(runde=r, eta=KE.get('wirkungsgrad'), glied=KE.get('glied_max'), kupfer=KE.get('kupfer_mm2'), heiss=len(heiss)))
        if not heiss: break
        gewachsen = 0.0
        for jm, n, L, h in sorted(heiss, key=lambda t: (-t[0], t[1], t[2])):
            neu = h.buffer(schritt, 16).intersection(frei(n, L))
            neu = neu.difference(geo[(n, L)])
            if neu.area < 0.01: continue
            geo[(n, L)] = unary_union([geo[(n, L)], neu]); gewachsen += neu.area
        if gewachsen < 0.05: break

    RR = dict(R, zonen=basis + [dict(meta[k], polys=SZ._polys(g)) for k, g in geo.items()])
    M, KE = _karten(B, RR, K)
    out = []
    for (n, L), g in sorted(geo.items()):
        e = eigen.get((n, L), Point(0, 0).buffer(0))
        if n in M and n in KE['netze']:
            d = _zellen(M[n], L, lambda J: J < tot * J_ziel)
            if d is not None: g = g.difference(d.difference(e.buffer(.02, 8)))
        fr = frei(n, L).union(e)
        g = SZ._oeffnen_g(g, r_glatt).union(e.intersection(g))
        g = SZ._schliessen_g(g, r_glatt).intersection(fr)
        teile = [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and p.area >= 0.05 and p.intersects(e)]
        g = unary_union(teile + [e]).buffer(0)
        for _ in range(2):
            g = SZ.tangential_runden(g.simplify(.002, preserve_topology=True).union(e), e, fr)
        g = g.simplify(.0005, preserve_topology=True)
        if g.is_empty: continue
        z = dict(meta[(n, L)], polys=SZ._polys(g), flaeche_mm2=round(g.area, 2), modell='fluss', J_ziel=J_ziel)
        out.append(z)
    if out: out[0]['fluss_verlauf'] = verlauf
    return out
