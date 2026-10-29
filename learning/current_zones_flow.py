import json, os, tempfile

import numpy as np
from shapely import box, union_all
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from learning import current_zones as SZ
from learning import current_zones_minimax as SM


def _maps(B, R, K):
    from analysis.copper_model import ir_values
    from analysis.current_chain import link_chain
    td = tempfile.mkdtemp(prefix='flow_')
    ir = ir_values(B, R, K, maps=td)
    KE = link_chain(B, R, K, ir=ir)
    M = {}
    for n, f in (ir.get('maps') or {}).items():
        if f and os.path.exists(os.path.join(td, f)):
            z = np.load(os.path.join(td, f)); M[n] = {k: z[k] for k in z.files}
    for f in os.listdir(td): os.remove(os.path.join(td, f))
    os.rmdir(td)
    return M, KE


def _cells(M, L, mask):
    J = M.get(L + '_J_A_mm2')
    if J is None: return None
    xx, yy = M['x_mm'], M['y_mm']
    m = np.isfinite(J) & mask(J)
    if not m.any(): return None
    h = float(abs(xx[0, 1] - xx[0, 0])) if xx.ndim == 2 and xx.shape[1] > 1 else 0.1
    iy, ix = np.nonzero(m)
    return union_all(box(xx[iy, ix] - .55 * h, yy[iy, ix] - .55 * h, xx[iy, ix] + .55 * h, yy[iy, ix] + .55 * h))


def zone_list(B, R, K, gnd_layers=None, clr=None, border=0.25, J_goal=20.0, J_start=40.0, step=0.3, rounds_n=8, tot=0.1,
          r_smoothed=0.15, via_field=True):
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gnd_layers(R, gnd_layers))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, SZ.QS)
    _, b_via = SZ._via_ground(R)
    hole = float((R.get('options') or {}).get('h2h') or .2)
    zs = SM.zone_list(B, R, K, gnd_layers, clr=clr, border=border, J=J_start, via_field=via_field, tot_shorten=False)
    basis = [z for z in R.get('zone_list', []) if z.get('source') != 'current_zones']
    geo = {(z['net'], z['layer']): SZ._geo(z) for z in zs}
    meta = {(z['net'], z['layer']): z for z in zs}
    owned = {}
    for L in SZ._layers(B):
        if L in gl: continue
        for m, g in SZ._parts(B, dict(R, zone_list=basis), L):
            if m in {n for n, _ in geo}: owned.setdefault((m, L), []).append(g)
    owned = {k: unary_union(v) for k, v in owned.items()}
    hole_list = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, 16)) for v in R.get('vias', [])]
    hole_list += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, 16)) for p in B['pads'] if p.get('drill')]

    def vacant(n, L):
        part_list = SZ._parts(B, dict(R, zone_list=basis), L)
        foreign = [g for m, g in part_list if m != n] + [g for (m, LL), g in geo.items() if LL == L and m != n]
        foreign += [g for m, g in hole_list if m != n]
        f = contour.difference(unary_union(foreign).buffer(clr + .003, 16)).difference(SZ.keepout_area(B, L, 'areas'))
        f = SZ.mask_free(f, B, n, L, clr, 16)
        return f.difference(unary_union([g for m, g in hole_list if m != n]).buffer(hole + .003, 16))

    history = []
    for r in range(rounds_n):
        RR = dict(R, zone_list=basis + [dict(meta[k], polys=SZ._polys(g)) for k, g in geo.items()])
        M, KE = _maps(B, RR, K)
        hot = []
        for (n, L), g in geo.items():
            if n not in M: continue
            h = _cells(M[n], L, lambda J: J > J_goal)
            if h is not None and not h.is_empty:
                jm = float(np.nanmax(M[n][L + '_J_A_mm2'])) if (L + '_J_A_mm2') in M[n] else 0.0
                hot.append((jm, n, L, h))
        history.append(dict(round_no=r, eta=KE.get('efficiency'), element=KE.get('element_max'), copper_map=KE.get('copper_mm2'), hot=len(hot)))
        if not hot: break
        grown = 0.0
        for jm, n, L, h in sorted(hot, key=lambda t: (-t[0], t[1], t[2])):
            new = h.buffer(step, 16).intersection(vacant(n, L))
            new = new.difference(geo[(n, L)])
            if new.area < 0.01: continue
            geo[(n, L)] = unary_union([geo[(n, L)], new]); grown += new.area
        if grown < 0.05: break

    RR = dict(R, zone_list=basis + [dict(meta[k], polys=SZ._polys(g)) for k, g in geo.items()])
    M, KE = _maps(B, RR, K)
    out = []
    for (n, L), g in sorted(geo.items()):
        e = owned.get((n, L), Point(0, 0).buffer(0))
        if n in M and n in KE['net_list']:
            d = _cells(M[n], L, lambda J: J < tot * J_goal)
            if d is not None: g = g.difference(d.difference(e.buffer(.02, 8)))
        fr = vacant(n, L).union(e)
        g = SZ._open_g(g, r_smoothed).union(e.intersection(g))
        g = SZ._close_g(g, r_smoothed).intersection(fr)
        part_list = [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and p.area >= 0.05 and p.intersects(e)]
        g = unary_union(part_list + [e]).buffer(0)
        for _ in range(2):
            g = SZ.tangential_rounds(g.simplify(.002, preserve_topology=True).union(e), e, fr)
        g = g.simplify(.0005, preserve_topology=True)
        if g.is_empty: continue
        z = dict(meta[(n, L)], polys=SZ._polys(g), area_mm2=round(g.area, 2), model='flow', J_goal=J_goal)
        out.append(z)
    if out: out[0]['flow_history'] = history
    return out
