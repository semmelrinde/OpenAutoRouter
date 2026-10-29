import math, os

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from learning import current_zones as SZ

T = dict(outer=0.035, inner=0.0175)


def _thickness(L):
    return T['outer'] if L in ('F', 'B') else T['inner']


def segment_currents(B, R, K, n):
    from common.flow import analysis
    k = (K.get('net_list') or {}).get(n) or {}
    idx = [i for i, t in enumerate(R.get('tracks', [])) if t.get('net') == n and len(t['pts']) >= 2]
    tr = [R['tracks'][i] for i in idx]
    pads = [p for p in B['pads'] if p['net'] == n]
    vias = [v for v in R.get('vias', []) if v.get('net') == n]
    info = analysis(tr, vias, pads, k.get('flow')) if k.get('flow') else None
    I = SZ.net_current(K, n) or 0.0
    out = {}
    for j, i in enumerate(idx):
        for s in range(len(R['tracks'][i]['w'])):
            a = (info or {}).get('share', {}).get((j, s)) if info else None
            out[(i, s)] = I * (a if a is not None else 1.0)
    return out, I


def via_fields(B, R, RR, K, zones_out, net_list, currents, clr, contour, hole, per_ampere=4.0, n_max=40, r_field=2.5):
    d_via, b_via = SZ._via_ground(R); part = d_via + clr + .02; new = []
    cu_layers = SZ._layers(B); protection_on = SZ.track_protection_on(R)
    for n in net_list:
        seg_s, I = currents[n]
        if I < 0.25: continue
        zone = {}
        for z in zones_out:
            if z['net'] == n: zone[z['layer']] = SZ._geo(z) if z['layer'] not in zone else zone[z['layer']].union(SZ._geo(z))
        if len(zone) < 2: continue
        foreign = unary_union([g for L in cu_layers for m, g in SZ._parts(B, RR, L) if m != n])
        bore = unary_union([Point(v['x'], v['y']).buffer(b_via / 2, 16) for v in RR.get('vias', []) if v.get('net') != n] +
                           [Point(p['x'], p['y']).buffer(p['drill'] / 2, 16) for p in B['pads'] if p.get('drill') and p['net'] != n])
        allowed = contour.buffer(-d_via / 2, 16).difference(foreign.buffer(d_via / 2 + clr + .003, 16)).difference(bore.buffer(b_via / 2 + hole + .003, 16))
        mask_keepout = unary_union([SZ.mask_clearance(B, n, L, clr, 16) for L in cu_layers])
        if not mask_keepout.is_empty: allowed = allowed.difference(mask_keepout.buffer(d_via / 2, 16))
        if protection_on:
            protection = unary_union([SZ._track_protection(RR, L, n)[1] for L in cu_layers])
            if not protection.is_empty: allowed = allowed.difference(protection.buffer(d_via / 2, 16))
        own = [(v['x'], v['y']) for v in RR.get('vias', []) if v.get('net') == n]
        for vx, vy in sorted(set(own)):
            att = {}
            for i, t in enumerate(R.get('tracks', [])):
                if t.get('net') != n: continue
                if protection_on and any(t.get(k) for k in SZ.PROTECTION): continue
                for k, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if math.dist(a, (vx, vy)) < 1e-3 or math.dist(b, (vx, vy)) < 1e-3:
                        att[t['layer']] = max(att.get(t['layer'], 0.0), seg_s.get((i, k), 0.0))
            ls = [L for L in att if L in zone]
            if len(ls) < 2: continue
            Iv = min(sorted(att.values())[-2:])
            nominal = min(n_max, max(2, math.ceil(per_ampere * Iv)))
            close = [q for q in own + [(v['x'], v['y']) for v in new if v['net'] == n] if math.dist(q, (vx, vy)) <= r_field]
            if len(close) >= nominal: continue
            reg = unary_union([zone[L] for L in ls[:1]])
            for L in ls[1:]: reg = reg.intersection(zone[L])
            reg = reg.intersection(Point(vx, vy).buffer(r_field, 16)).intersection(allowed)
            if reg.is_empty: continue
            x0, y0, x1, y1 = reg.bounds; h = part / 2
            cand = sorted(((x0 + i * h, y0 + j * h) for i in range(int((x1 - x0) / h) + 1) for j in range(int((y1 - y0) / h) + 1)),
                          key=lambda q: math.dist(q, (vx, vy)))
            occupied = [(v['x'], v['y']) for v in RR.get('vias', [])] + [(v['x'], v['y']) for v in new]
            for q in cand:
                if len(close) >= nominal: break
                if not reg.contains(Point(q)) or any(math.dist(q, b) < part for b in occupied): continue
                v = dict(x=round(q[0], 4), y=round(q[1], 4), net=n, source='current_via', amps_a=round(Iv, 2))
                new.append(v); occupied.append(q); close.append(q)
    return new


def zone_list(B, R, K, gnd_layers=None, clr=None, border=0.25, J=40.0, w_max=6.0, hem=0.15, r_smoothed=0.3, r_open=0.15, min_area=0.05,
          via_field=True, per_ampere=4.0, tot_shorten=True, order=None, ordering=None):
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gnd_layers(R, gnd_layers))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, SZ.QS)
    d_via, b_via = SZ._via_ground(R)
    hole_list = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, SZ.QS)) for v in R.get('vias', [])]
    hole_list += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, SZ.QS)) for p in B['pads'] if p.get('drill')]
    prio = SZ.precedence(B, R, K)
    net_list = sorted(SZ.current_nets(K, gnd), key=lambda n: (-prio[n]['weight'], n))
    if order == 'link_chain' and ordering is None:


        import copy
        from analysis.current_chain import link_chain
        R1 = copy.deepcopy(R)
        z1 = zone_list(B, R1, K, gnd_layers, clr=clr, border=border, J=J, w_max=w_max, hem=hem, r_smoothed=r_smoothed, r_open=r_open,
                   min_area=min_area, via_field=via_field, per_ampere=per_ampere)
        KE = link_chain(B, dict(R1, zone_list=[z for z in R1.get('zone_list', []) if z.get('source') != 'current_zones'] + z1), K)
        eta = {n: v['efficiency'] for n, v in KE['net_list'].items()}
        ordering = sorted(net_list, key=lambda n: (eta.get(n, 1.0), n))
    if ordering is not None: net_list = [n for n in ordering if n in net_list] + [n for n in net_list if n not in ordering]
    currents = {n: segment_currents(B, R, K, n) for n in net_list}
    RR = dict(R, zone_list=list(R.get('zone_list') or [])); out = []
    protection = SZ.track_protection_on(RR)
    hole = float((R.get('options') or {}).get('h2h') or .2)
    for L in SZ._layers(B):
        if L in gl: continue
        t = _thickness(L)
        for n in net_list:
            part_list = SZ._parts(B, RR, L)
            owned = unary_union([g for m, g in part_list if m == n])
            if owned.is_empty: continue
            pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
            seg_s, I = currents[n]
            corr = []
            for i, tr in enumerate(RR.get('tracks', [])):
                if tr.get('net') != n or tr['layer'] != L or len(tr['pts']) < 2: continue
                if protection and any(tr.get(k) for k in SZ.PROTECTION): continue
                for s, (a, b, w) in enumerate(zip(tr['pts'][:-1], tr['pts'][1:], tr['w'])):
                    if a == b: continue
                    ws = min(w_max, max(w, seg_s.get((i, s), I) / (J * t)))
                    ls = LineString([a, b])
                    corr.append(ls.buffer(ws / 2, cap_style=2, join_style=2))
                    for p in (a, b):
                        corr.append(SZ._octagon(Point(p).buffer(1e-3, 4), ws / 2))
            if not corr and len(pads) < 2: continue
            vias_n = [Point(v['x'], v['y']).buffer(d_via / 2, 8) for v in RR.get('vias', []) if v.get('net') == n]
            cand = unary_union(corr + [SZ._octagon(p, hem) for p in pads + vias_n])
            foreign = unary_union([g for m, g in part_list if m != n] + [g for m, g in hole_list if m != n])
            vacant = contour.difference(foreign.buffer(clr + .003, SZ.QS)).difference(SZ.keepout_area(B, L, 'areas'))
            vacant = SZ.mask_free(vacant, B, n, L, clr)
            vacant = vacant.difference(unary_union([h.buffer(hole + .003, SZ.QS) for m, h in hole_list if m != n]))
            if protection:
                _, moat = SZ._track_protection(RR, L, n)
                if not moat.is_empty: vacant = vacant.difference(moat)
            z = SZ._open_g(cand.intersection(vacant), r_open)
            z = unary_union([z, owned])
            z = SZ._close_g(z, r_smoothed).intersection(unary_union([vacant, owned]))
            pieces = [s for s in getattr(z, 'geoms', [z]) if s.geom_type == 'Polygon' and s.area >= min_area and s.intersects(owned)]
            if not pieces: continue
            g = unary_union(pieces + [owned]).buffer(0)
            safe = unary_union([vacant, owned])
            g = SZ._open(SZ._close(g.buffer(.02, SZ.QS).intersection(safe), .01), .01).intersection(safe).union(owned)
            for _ in range(2):
                g = SZ.tangential_rounds(g.simplify(.002, preserve_topology=True).union(owned), owned, vacant)
            g = g.simplify(.0005, preserve_topology=True)
            zz = dict(net=n, layer=L, source='current_zones', amps_a=round(I, 3), polys=SZ._polys(g), area_mm2=round(g.area, 2),
                      owned_mm2=round(owned.area, 2), precedence=prio[n], model='minimax', J=J)
            out.append(zz); RR['zone_list'].append(zz)
    if via_field:
        nv = via_fields(B, R, RR, K, out, net_list, currents, clr, contour, hole, per_ampere=per_ampere)
        R.setdefault('vias', []).extend(nv)
    if tot_shorten: totes_copper_shorten(B, R, K, out, gnd_layers)
    return out


def totes_copper_shorten(B, R, K, zones_out, gnd_layers=None, share=0.1, r_smoothed=0.15):
    import tempfile
    import numpy as np
    from shapely import box, union_all
    from analysis.copper_model import ir_values
    from analysis.current_chain import link_chain
    RR = dict(R, zone_list=[z for z in R.get('zone_list', []) if z.get('source') != 'current_zones'] + list(zones_out))
    with tempfile.TemporaryDirectory() as td:
        ir = ir_values(B, RR, K, maps=td)
        KE = link_chain(B, RR, K, ir=ir)
        maps = {n: np.load(os.path.join(td, f)) for n, f in (ir.get('maps') or {}).items() if f and os.path.exists(os.path.join(td, f))}
        way = {}
        for z in zones_out:
            n, L = z['net'], z['layer']
            if n not in maps or n not in KE['net_list'] or (L + '_J_A_mm2') not in maps[n].files: continue
            M = maps[n]; J = M[L + '_J_A_mm2']; xx, yy = M['x_mm'], M['y_mm']
            threshold = share * KE['net_list'][n]['j_star']
            tot = np.isfinite(J) & (J < threshold)
            if not tot.any(): continue
            h = float(abs(xx[0, 1] - xx[0, 0])) if xx.ndim == 2 and xx.shape[1] > 1 else 0.1
            iy, ix = np.nonzero(tot)
            dead = union_all(box(xx[iy, ix] - .55 * h, yy[iy, ix] - .55 * h, xx[iy, ix] + .55 * h, yy[iy, ix] + .55 * h))
            g = SZ._geo(z)
            owned = unary_union([gg for m, gg in SZ._parts(B, R, L) if m == n])
            new = g.difference(dead.difference(owned.buffer(.02, 8)))
            new = SZ._open_g(new, r_smoothed).union(owned.intersection(g))
            new = SZ._close_g(new, r_smoothed).intersection(g)
            part_list = [p for p in getattr(new, 'geoms', [new]) if p.geom_type == 'Polygon' and p.intersects(owned)]
            new = unary_union(part_list + [owned.intersection(g)]).buffer(0).simplify(.0005, preserve_topology=True)
            if new.is_empty: continue
            way[n] = round(way.get(n, 0.0) + g.area - new.area, 2)
            z.update(polys=SZ._polys(new), area_mm2=round(new.area, 2), tot_shortened=True)
    return way
