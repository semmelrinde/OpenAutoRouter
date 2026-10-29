import math

import numpy as np
import scipy.sparse as sp
from scipy.ndimage import convolve
from scipy.sparse.linalg import splu
import shapely
from shapely import box, union_all
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from learning import current_zones as SZ
from learning import current_zones_minimax as SM

RHO = 1.72e-5
VIA_OHM_MM = 1.4e-3


def _thickness_z(B, R):
    from analysis.measures import structure
    cu_layers = SZ._layers(B)
    a = structure(len(cu_layers), [str(x).split('=')[0] for x in (R.get('gnd_layers') or [])])
    return {L: a['cu_layers'][L]['t'] for L in cu_layers}, {L: a['cu_layers'][L]['z'] for L in cu_layers}


def _loads(B, K, n):
    from analysis.measures import _current
    fl = ((K.get('net_list') or {}).get(n) or {}).get('flow') or {}
    if not isinstance(fl, dict) or not isinstance(fl.get('load') or {}, dict): return [], {}
    load = {p: float(q) for p, q in (fl.get('load') or {}).items() if float(q) > 0}
    I = _current(n, K)[0] or 0.0
    s = sum(load.values())
    k = (K.get('net_list') or {}).get(n) or {}
    provenance = k.get('current_A_source')
    amount = k.get('current_A')
    if (s <= 0 and not fl and isinstance(provenance, str) and provenance.strip()
            and isinstance(amount, (int, float)) and not isinstance(amount, bool)
            and math.isfinite(amount) and amount > 0):


        ks = sorted({'%s:%s' % (q['ref'], q.get('num')) for q in B['pads'] if q['net'] == n})
        if len(ks) >= 2 and I > 0: return [ks[0]], {k: I / (len(ks) - 1) for k in ks[1:]}
    if s <= 0 or I <= 0: return [], {}
    source = list(fl.get('source') or [])

    if any(isinstance(q, str) and q in (fl.get('load') or {}) for q in source): return [], {}
    return source, {p: q * I / s for p, q in load.items()}


def last_status(K, n, B=None):
    import math
    from analysis.minimax import load_error
    k = (K.get('net_list') or {}).get(n) or {}
    fl = k.get('flow') or {}
    if not fl:
        value = k.get('current_A')
        if (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) and value > 0 and B is not None
                and all(_loads(B, K, n))):
            return 'estimate'
        return 'unknown'
    if not isinstance(fl, dict): return 'unknown'
    sources, loads = fl.get('source'), fl.get('load')
    if (fl.get('unit') != 'A' or fl.get('unit_source') not in ('spec', 'estimate')
            or not isinstance(sources, list) or not sources or not isinstance(loads, dict) or not loads
            or B is None): return 'unknown'
    if load_error(k): return 'unknown'
    pads = {f"{p['ref']}:{p.get('num')}": p['net'] for p in B.get('pads', [])}
    if any(not isinstance(p, str) or pads.get(p) != n for p in sources + list(loads)):
        return 'unknown'
    if set(sources) & set(loads): return 'unknown'
    return 'loadless' if fl['total'] == 0 else 'positive'


def pad_groups(B, n, L, core_set, vacant, spacing=1.5, hem=.15):
    pads = [SZ._geo(dict(polys=[dict(outer=q, inner=[]) for q in p['polys'][L]]))
            for p in B['pads'] if p['net'] == n and p.get('polys', {}).get(L)]
    shapes = []
    for i, a in enumerate(pads):
        for b in pads[i+1:]:
            if a.distance(b) > spacing: continue
            pair = a.union(b)
            clipped = pair.convex_hull.buffer(hem, 32).intersection(vacant)
            for poly in getattr(clipped, 'geoms', [clipped]):
                if poly.geom_type == 'Polygon' and poly.buffer(1e-8).covers(pair):
                    shapes.append(poly)
    if not shapes: return core_set
    copper = core_set.union(unary_union(shapes)).buffer(0)
    return SZ.tangential_rounds(copper, core_set, vacant, radius=hem)


def scaffold_corridor(B, R, n, free_fn, keepout_layers=(), hem=.15):
    out = {}
    for L in SZ._layers(B):
        if L in keepout_layers: continue
        if not (any(t['net'] == n and t['layer'] == L and len(t['pts']) >= 2 for t in R.get('tracks', [])) or
                any(p['net'] == n and p.get('polys', {}).get(L) for p in B['pads'])): continue
        e = unary_union([g for m,g in SZ._parts(B,R,L) if m == n])
        if e.is_empty: continue
        out[L] = e.buffer(hem,32).intersection(free_fn(n,L)).union(e)
    return out


def optimize_net(B, RR, K, n, layers_t, layers_z, free_fn, h=0.1, r_dom=3.0, kappa=0.3, p=3.0, iters=60, r_filter=0.4,
                   rho_min=1e-3, start=1.0, keepout_layers=(), pad_cores=None, lam_min_mw=0.0, lam_max_mw=0.0):
    from learning.copper_bars import track_geometry
    d_via, _ = SZ._via_ground(RR)
    source, load_list = _loads(B, K, n)
    if not source or not load_list: return None, dict(reason='no source or load')
    pads_n = [q for q in B['pads'] if q['net'] == n]
    vias_n = [v for v in RR.get('vias', []) if v.get('net') == n]
    Ls = []
    owned = {}
    for L in SZ._layers(B):
        if L in keepout_layers: continue
        g = [track_geometry(t) for t in RR.get('tracks', []) if t.get('net') == n and t['layer'] == L and len(t['pts']) >= 2]
        pg = [Polygon(q).buffer(0) for pd in pads_n for q in (pd.get('polys') or {}).get(L, []) if len(q) >= 3]
        if not g and not pg: continue
        cons = [Point(v['x'], v['y']).buffer(d_via / 2, 16) for v in vias_n]
        owned[L] = unary_union(g + pg + cons)
        if pad_cores and L in pad_cores and not pad_cores[L].is_empty:
            owned[L] = owned[L].union(pad_cores[L])
        Ls.append(L)
    if not Ls: return None, dict(reason='no copper')
    dom = {L: free_fn(n, L).intersection(owned[L].buffer(r_dom, 16)).union(owned[L]) for L in Ls}
    x0, y0, x1, y1 = unary_union(list(dom.values())).bounds
    nx, ny = int(math.ceil((x1 - x0) / h)) + 2, int(math.ceil((y1 - y0) / h)) + 2
    X, Y = np.meshgrid(x0 + (np.arange(nx) - .5) * h, y0 + (np.arange(ny) - .5) * h)
    pinned, ind, idx = {}, {}, {}
    nn = 0
    for L in Ls:
        e = owned[L].buffer(h * 0.5, 8); shapely.prepare(e); dd = dom[L]; shapely.prepare(dd)
        f = shapely.contains_xy(e, X, Y)
        m = f | shapely.contains_xy(dd, X, Y)
        ix = np.full(m.shape, -1); ix[m] = np.arange(nn, nn + m.sum()); nn += int(m.sum())
        pinned[L], ind[L], idx[L] = f, m, ix
    if nn == 0: return None, dict(reason='leeres Raster')

    a, b, g0, ea = [], [], [], []
    for L in Ls:
        ix = idx[L]; gl = layers_t[L] / RHO
        for sl_a, sl_b in (((slice(None), slice(None, -1)), (slice(None), slice(1, None))), ((slice(None, -1), slice(None)), (slice(1, None), slice(None)))):
            A_, B_ = ix[sl_a], ix[sl_b]; m = (A_ >= 0) & (B_ >= 0)
            a.append(A_[m]); b.append(B_[m]); g0.append(np.full(m.sum(), gl)); ea.append(np.ones(m.sum(), bool))
    va, vb, cons = [], [], []
    for v in vias_n + [dict(x=q['x'], y=q['y']) for q in pads_n if q.get('drill')]:
        cx, cy = int(round((v['x'] - X[0, 0]) / h)), int(round((v['y'] - Y[0, 0]) / h))
        nodes = []
        for L in sorted(Ls, key=lambda L: layers_z[L]):
            if 0 <= cy < ny and 0 <= cx < nx and idx[L][cy, cx] >= 0: nodes.append((L, idx[L][cy, cx]))
        for (L1, k1), (L2, k2) in zip(nodes[:-1], nodes[1:]):
            va.append(k1); vb.append(k2); cons.append(1.0 / (VIA_OHM_MM * max(0.05, abs(layers_z[L2] - layers_z[L1]))))
    a = np.concatenate(a + [np.asarray(va, int)]); b = np.concatenate(b + [np.asarray(vb, int)])
    g0 = np.concatenate(g0 + [np.asarray(cons, float)]); plane = np.concatenate(ea + [np.zeros(len(va), bool)])
    rho = np.zeros(nn); fixed = np.zeros(nn, bool)
    for L in Ls:
        ix = idx[L]; m = ind[L]
        rho[ix[m]] = np.where(pinned[L][m], 1.0, start); fixed[ix[m]] = pinned[L][m]

    def pad_node(key):
        ref, num = key.split(':', 1); out = []
        for pd in pads_n:
            if pd['ref'] != ref or str(pd.get('num')) != num: continue
            for L, qs in (pd.get('polys') or {}).items():
                if L not in idx: continue
                geo = unary_union([Polygon(q).buffer(h * 0.5) for q in qs if len(q) >= 3])
                mm = shapely.intersects_xy(geo, X, Y) & (idx[L] >= 0)
                out.extend(idx[L][mm])
        return np.unique(np.asarray(out, int))
    src_pt = np.unique(np.concatenate([pad_node(q) for q in source] + [np.zeros(0, int)]))
    rhs = np.zeros(nn)
    for k, I in load_list.items():
        kn = pad_node(k)
        if len(kn): rhs[kn] += I / len(kn)
    if not len(src_pt) or rhs.sum() <= 0: return None, dict(reason='Source/load not in grid')
    from scipy.sparse.csgraph import connected_components
    adj = sp.coo_matrix((np.ones(len(a)), (a, b)), shape=(nn, nn))
    _, lab = connected_components(adj, directed=False)
    reached = np.isin(lab, np.unique(lab[src_pt]))
    lost = float(rhs[~reached].sum())
    if lost > 0.01 * float(rhs.sum()):
        return None, dict(reason='Loads not connected to the source (%.3f A of %.3f A)' % (lost, float(rhs.sum())))
    rhs[~reached] = 0.0
    free_k = reached.copy(); free_k[src_pt] = False; rhs[src_pt] = 0.0
    if rhs.sum() <= 0: return None, dict(reason='Loads not connected to the source')
    rf = max(1, int(round(r_filter / h)))
    yy_, xx_ = np.mgrid[-rf:rf + 1, -rf:rf + 1]; ker = np.maximum(0.0, rf + 0.5 - np.hypot(xx_, yy_))
    design = ~fixed
    history = []
    p_goal = p
    for it in range(iters):

        p = 1.0 if it < iters // 4 else (2.0 if it < iters // 2 else p_goal)
        s = rho_min + rho ** p * (1 - rho_min)
        sa, sb = s[a], s[b]
        f = np.where(plane, 2 * sa * sb / (sa + sb), 1.0)
        g = g0 * f
        Lm = sp.coo_matrix((np.r_[g, g, -g, -g], (np.r_[a, b, a, b], np.r_[a, b, b, a])), shape=(nn, nn)).tocsc()
        phi = np.zeros(nn)
        try: phi[free_k] = splu(Lm[free_k][:, free_k]).solve(rhs[free_k])
        except RuntimeError: return None, dict(reason='System of equations singular')
        P = float(rhs @ phi)
        dphi2 = (phi[a] - phi[b]) ** 2

        dsa = np.where(plane, -g0 * 2 * sb ** 2 / (sa + sb) ** 2 * dphi2, 0.0)
        dsb = np.where(plane, -g0 * 2 * sa ** 2 / (sa + sb) ** 2 * dphi2, 0.0)
        dPds = np.bincount(a, dsa, nn) + np.bincount(b, dsb, nn)
        dPdr = dPds * p * np.maximum(rho, 1e-9) ** (p - 1) * (1 - rho_min)

        dcf = np.zeros(nn)
        for L in Ls:
            ix = idx[L]; m = ix >= 0
            R_ = np.zeros(ix.shape); D_ = np.zeros(ix.shape); W_ = np.zeros(ix.shape)
            R_[m] = rho[ix[m]]; D_[m] = dPdr[ix[m]]; W_[m] = 1.0
            num = convolve(R_ * D_, ker, mode='constant'); the = convolve(W_, ker, mode='constant') * np.maximum(R_, 1e-3)
            dcf[ix[m]] = (num[m] / np.maximum(the[m], 1e-12))
        A = float(rho.sum() * h * h)
        lam = kappa * P / max(A, 1e-9)






        lam = max(lam, lam_min_mw * 1e-3)




        if lam_max_mw: lam = min(lam, lam_max_mw * 1e-3)
        Bc = np.maximum(-dcf, 0.0) / (lam * h * h)
        new = np.clip(rho * np.sqrt(Bc), np.maximum(0.0, rho - 0.2), np.minimum(1.0, rho + 0.2))
        new = np.where(design & reached, np.clip(new, 0.0, 1.0), np.where(reached, 1.0, 0.0))
        change = float(np.max(np.abs(new - rho))) if nn else 0.0
        rho = new
        history.append(dict(it=it, p=p, P_mW=round(P * 1000, 4), A_mm2=round(A, 2), change=round(change, 3), lost_A=round(lost, 3)))
        if change < 0.01 and it > iters // 2 + 5: break
    out = {}
    for L in Ls:
        ix = idx[L]; m = (ix >= 0)
        on = np.zeros(ix.shape, bool); on[m] = rho[ix[m]] >= 0.5
        on |= pinned[L]
        iy, ix2 = np.nonzero(on)
        g = union_all(box(X[iy, ix2] - .52 * h, Y[iy, ix2] - .52 * h, X[iy, ix2] + .52 * h, Y[iy, ix2] + .52 * h)) if len(iy) else None
        out[L] = owned[L] if g is None else g.intersection(dom[L]).union(owned[L])
    return out, dict(history=history, nodes=nn, cu_layers=Ls, current_A=round(sum(load_list.values()), 3))
def _slim(t, w_max):
    t = dict(t, w=[min(w, w_max) for w in t['w']])
    if 'items' in t: t['items'] = [list(it[:-1]) + [min(it[-1], w_max)] for it in t['items']]
    return t
def _track_id(t):
    return '%s|%s|%.4f,%.4f|%.4f,%.4f|%d' % (t.get('net'), t['layer'], *t['pts'][0], *t['pts'][-1], len(t['pts']))


def _skeleton_widths(RR, n, L, g, w_max):
    from shapely.geometry import LineString
    gg = g.buffer(1e-3, 8); shapely.prepare(gg); new = {}
    for t in RR.get('tracks', []):
        if t.get('net') != n or t['layer'] != L or len(t['pts']) < 2 or 'items' in t: continue
        ws = [w if w <= w_max or a == b or gg.contains(LineString([a, b]).buffer(w / 2, 16)) else w_max
              for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
        if ws != list(t['w']): new[_track_id(t)] = ws
    return new


def skeleton_apply(R, zs):
    new = {}
    for z in zs: new.update(z.pop('skeleton', None) or {})
    if not new: return 0
    k = 0
    for i, t in enumerate(R.get('tracks', [])):
        if 'items' not in t and len(t['pts']) >= 2 and _track_id(t) in new:
            R['tracks'][i] = dict(t, w=new[_track_id(t)]); k += 1
    return k


def zone_list(B, R, K, gnd_layers=None, clr=None, border=0.25, h=0.15, r_dom=3.0, kappa=0.3, p=3.0, iters=40, r_smoothed=0.15,
          via_field=True, order='link_chain', oct_=0.3, r_filter=0.4, r_hull=0.45, r_round=0.15, holes_fill=True,
          edges='vacant', r_dock=0.0, dock_vias=False, wedge_max=45.0, skeleton=None, only_area=0.2, lam_min_mw=0.0, r_on=None, lam_max_mw=2.0):
    from analysis.switch_node import an as sw_on, correct as sw_current, compact as sw_shape
    sw_enabled = sw_on(R)
    if sw_enabled:
        import copy
        K = copy.deepcopy(K)
        sw_current(B, K)
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gnd_layers(R, gnd_layers))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, SZ.QS)
    _, b_via = SZ._via_ground(R)
    hole = float((R.get('options') or {}).get('h2h') or .2)
    hole_list = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, 16)) for v in R.get('vias', [])]
    hole_list += [(q['net'], Point(q['x'], q['y']).buffer(q['drill'] / 2, 16)) for q in B['pads'] if q.get('drill')]
    prio = SZ.precedence(B, R, K)
    net_list = sorted(SZ.current_nets(K, gnd), key=lambda n: (-prio[n]['weight'], n))
    if order == 'link_chain':
        import copy
        from analysis.current_chain import link_chain
        R1 = copy.deepcopy(R)
        z1 = SM.zone_list(B, R1, K, gnd_layers, clr=clr, border=border, via_field=False)
        KE = link_chain(B, dict(R1, zone_list=[z for z in R1.get('zone_list', []) if z.get('source') != 'current_zones'] + z1), K)
        eta = {n: v['efficiency'] for n, v in KE['net_list'].items()}
        net_list = sorted(net_list, key=lambda n: (eta.get(n, 1.0), n))
    layers_t, layers_z = _thickness_z(B, R)
    RR = dict(R, zone_list=[z for z in R.get('zone_list', []) if z.get('source') != 'current_zones'])
    out = []; infos = {}; protection = SZ.track_protection_on(RR)

    def vacant(n, L):
        part_list = SZ._parts(B, RR, L)
        foreign = unary_union([g for m, g in part_list if m != n] + [g for m, g in hole_list if m != n])
        f = contour.difference(foreign.buffer(clr + .003, 16)).difference(SZ.keepout_area(B, L, 'areas'))
        f = SZ.mask_free(f, B, n, L, clr, 16)
        f = f.difference(unary_union([g for m, g in hole_list if m != n]).buffer(hole + .003, 16))
        if not protection: return f
        _, moat = SZ._track_protection(RR, L, n)
        return f.difference(moat) if not moat.is_empty else f

    optimized = []
    for n in net_list:
        if sw_enabled and K['net_list'][n].get('category') == 'sw':
            geo, info = sw_shape(B, RR, K, n, [L for L in SZ._layers(B) if L not in gl], vacant)
            infos[n] = info
            for L, g in sorted(geo.items()):
                e = unary_union([gg for m, gg in SZ._parts(B, dict(RR, zone_list=[]), L) if m == n])
                if g.is_empty or g.difference(e).area < .01: continue
                z = dict(net=n, layer=L, source='current_zones', amps_a=K['net_list'][n].get('current_A'),
                         current_A_source=K['net_list'][n].get('current_A_source'), model='sw_compact',
                         polys=SZ._polys(g), area_mm2=round(g.area, 2), owned_mm2=round(e.area, 2), precedence=prio[n])
                out.append(z); RR['zone_list'].append(z)
            continue
        pad_cores = {}
        status = last_status(K, n, B) if R.get('k20_corridor') else None
        if R.get('k20_corridor') and status in ('positive', 'loadless'):
            from learning.k20_pad_groups import core_set
            pad_cores = {L: core_set(B, RR, K, n, L, vacant(n, L).buffer(-.002, 8))
                        for L in SZ._layers(B) if L not in gl}



        RRo = dict(RR, tracks=[_slim(t, skeleton) if t.get('net') == n else t for t in RR.get('tracks', [])]) if skeleton else RR
        if R.get('k20_corridor') and status not in ('positive', 'loadless'):
            geo, info = None, dict(reason='incomplete load contract')
        else:
            geo, info = optimize_net(B, RRo, K, n, layers_t, layers_z, vacant, h=h, r_dom=r_dom, kappa=kappa, p=p, iters=iters, r_filter=r_filter,
                                       keepout_layers=gl, pad_cores=pad_cores, lam_min_mw=lam_min_mw, lam_max_mw=lam_max_mw)
        fallback = False
        if (R.get('r36_scaffold') and not geo and info.get('reason') == 'no source or load'
                and (SZ._current_known(K['net_list'][n]) or 0.) >= SZ.I_ZONE):
            geo = scaffold_corridor(B,RR,n,vacant,gl)
            fallback = bool(geo)
            info = dict(info, fallback='scaffold_corridor_without_load_model', current_field_optimized=False)
        infos[n] = info
        if geo:
            optimized.append(n)
        elif R.get('k20_corridor') and status == 'loadless':


            geo = {L: core_set.union(unary_union([g for m,g in SZ._parts(B, dict(RR,zone_list=[]), L) if m == n]))
                   for L,core_set in pad_cores.items() if not core_set.is_empty}
            info['pad_groups_without_current_model'] = bool(geo)
        if not geo: continue
        I = SZ.net_current(K, n)
        for L, g in sorted(geo.items()):
            if L in gl: continue
            e = unary_union([gg for m, gg in SZ._parts(B, dict(RRo, zone_list=[]), L) if m == n])
            connection_core = e.union(pad_cores[L]) if L in pad_cores and not pad_cores[L].is_empty else e
            fr = vacant(n, L).union(e)

            g = SZ._open_g(g, r_smoothed if r_on is None else r_on).union(e.intersection(g))
            g = _fl(SZ._close_g(g, r_smoothed).intersection(fr))
            part_list = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(part_list + [connection_core]).buffer(0)


            if r_hull:
                g = _fl(SZ._close_g(g, r_hull).intersection(fr)).union(e)
                g = SZ._open_g(g, r_smoothed).union(e.intersection(g))
            if oct_ and edges == 'oct_':
                g = _fl(octilinear(g, oct_).intersection(fr)).union(e)
            elif oct_:


                g = _fl(g.simplify(oct_, preserve_topology=True).intersection(fr)).union(e)


            if r_dock:
                fen = g.buffer(2 * r_dock + .1, 8)
                if dock_vias: blk = contour.difference(fr).intersection(fen)
                else:
                    from learning.copper_bars import track_geometry
                    lin = [track_geometry(tr) for tr in RR.get('tracks', []) if tr['layer'] == L and tr.get('net') != n and len(tr['pts']) >= 2]
                    lin += [Polygon(q).buffer(0) for pd in B['pads'] if pd['net'] != n for q in (pd.get('polys') or {}).get(L, []) if len(q) >= 3]
                    lin += [SZ._geo(z) for z in RR['zone_list'] if z['layer'] == L and z['net'] != n]
                    lin = [x for x in lin if x.intersects(fen)]
                    blk = unary_union(lin).buffer(clr + .003, 16).intersection(fen) if lin else None
                if blk is not None and not blk.is_empty:
                    g = _fl(g.union(blk).buffer(r_dock, 16).buffer(-r_dock, 16).intersection(fr)).union(e)
            part_list = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(part_list + [e]).buffer(0)
            fs = vacant(n, L).buffer(-.002, 8).union(e)
            g = _fl(g.intersection(fs)).union(e)
            if wedge_max:
                g = wedges_fill(g, fs, [gg for m, gg in SZ._parts(B, RR, L) if m != n], clr, wedge_max)



            g = _fl(g.buffer(-r_round, 32).buffer(r_round, 32).intersection(g)).union(e)
            throat = _fl(g.buffer(r_round, 32).buffer(-r_round, 32).difference(g).intersection(fs))
            g = g.union(throat).buffer(0)
            if holes_fill:
                inn = unary_union([Polygon(i) for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' for i in q.interiors])
                if not inn.is_empty:
                    new = _fl(inn.intersection(fs))
                    gg = g.union(new).buffer(0)
                    g = g.union(gg.buffer(-r_round, 32).buffer(r_round, 32).intersection(new)).buffer(0)
            g = _fl(g.intersection(fs)).union(e)
            part_list = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(part_list + [connection_core]).buffer(0).simplify(.0005, preserve_topology=True)
            if only_area:






                g = unary_union([q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon'
                                 and q.difference(e).area >= only_area]).buffer(0)
            if R.get('r36_pad_groups'):
                g = pad_groups(B, n, L, g, fs)
            if g.is_empty or g.area <= e.area + 0.01: continue
            z = dict(net=n, layer=L, source='current_zones', amps_a=round(I or 0, 3), polys=SZ._polys(g), area_mm2=round(g.area, 2),
                     owned_mm2=round(e.area, 2), precedence=prio[n], model='scaffold-without-load_model' if fallback else 'topo', kappa=kappa)
            if not (K['net_list'][n].get('flow') or {}) and last_status(K, n, B) == 'estimate':
                z['current_model'] = 'estimate; source and equal load distribution from pad order, no complete load contract'
            if info.get('pad_groups_without_current_model'):
                status = last_status(K, n, B)
                z['amps_a'] = 0.0 if status == 'loadless' else None
                z['current_model'] = status+'; only geometric pad connection, no positive topo DC load case'
            if skeleton:
                z['skeleton'] = _skeleton_widths(RR, n, L, g, skeleton)
            out.append(z); RR['zone_list'].append(z)
        infos[n] = info
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true'):
        from learning.k17_contour import notches_fill
        for z in out:
            if K['net_list'][z['net']].get('category') == 'sw':
                continue
            g = SZ._geo(z)
            fs = vacant(z['net'], z['layer']).buffer(-.002, 8)
            proposal = notches_fill(g, fs)
            if proposal.area > g.area + 1e-9:
                z.update(polys=SZ._polys(proposal), area_mm2=round(proposal.area, 2))

    if out and (R.get('k20_corridor') or R.get('r36_pad_groups') or R.get('topo_diagnose')):
        out[0]['topo_info'] = infos
    if via_field and out:
        currents = {n: SM.segment_currents(B, R, K, n) for n in net_list}
        via_nets = optimized if R.get('k20_corridor') else net_list
        if sw_enabled: via_nets = [n for n in via_nets if K['net_list'][n].get('category') != 'sw']
        nv = SM.via_fields(B, R, RR, K, out, via_nets, currents, clr, contour, hole)
        R.setdefault('vias', []).extend(nv)
    return out


def wedges_fill(g, fs, foreign, clr, w_max=30.0, w_end=0.45, l_max=2.0, l_min=0.1):
    fen = g.buffer(l_max + .5, 8)
    foreign = [x for x in foreign if x.intersects(fen)]
    if not foreign: return g
    h = clr / 2 + .01
    copper_map = unary_union([g.buffer(h, 8)] + [x.buffer(h, 8) for x in foreign])
    white = fen.difference(copper_map)
    close = g.buffer(h + .04, 8).difference(g.buffer(h - .04, 8))
    new = []
    for poly in getattr(white, 'geoms', [white]):
        if poly.geom_type != 'Polygon' or poly.area < 1e-4: continue
        for ring, outer in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
            pts = list(ring.simplify(0.01).coords)[:-1]
            if len(pts) < 3: continue
            fl = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
            for i in range(len(pts)):
                a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
                v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
                l1, l2 = math.hypot(*v1), math.hypot(*v2)
                if l1 < l_min or l2 < l_min: continue
                w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
                cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
                convex = (cross > 0) == (fl > 0) if outer else (cross > 0) != (fl > 0)
                if not convex or w >= w_max or not close.contains(Point(b)): continue
                L = min(l_max, w_end / max(1e-3, 2 * math.tan(math.radians(w) / 2)))
                u1, u2 = (v1[0] / l1, v1[1] / l1), (v2[0] / l2, v2[1] / l2)
                new.append(Polygon([b, (b[0] + L * u1[0], b[1] + L * u1[1]), (b[0] + L * u2[0], b[1] + L * u2[1])]).buffer(h, 8))
    if not new: return g
    add = _fl(unary_union(new).intersection(fs))
    add = unary_union([q for q in getattr(add, 'geoms', [add]) if q.geom_type == 'Polygon' and q.buffer(.01).intersects(g)])
    return g.union(add).buffer(0) if not add.is_empty else g


def _fl(g):
    return unary_union([q for q in shapely.get_parts(g) if q.geom_type == 'Polygon' and q.area > 1e-7])


def _ring_oct(coords, tol):
    from shapely.geometry import LinearRing
    r = LinearRing(coords).simplify(tol, preserve_topology=False)
    pts = list(r.coords)[:-1]
    if len(pts) < 3: return None
    group_list = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        l = math.dist(a, b)
        if l < 1e-6: continue
        k = int(round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) / 45.0)) % 8
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        if group_list and group_list[-1][0] == k: g = group_list[-1]; g[1] += l; g[2] += mx * l; g[3] += my * l
        else: group_list.append([k, l, mx * l, my * l])
    if len(group_list) > 1 and group_list[0][0] == group_list[-1][0]:
        g = group_list.pop(); group_list[0][1] += g[1]; group_list[0][2] += g[2]; group_list[0][3] += g[3]
    if len(group_list) < 3: return None
    lines = [((g[2] / g[1], g[3] / g[1]), (math.cos(math.radians(45 * g[0])), math.sin(math.radians(45 * g[0])))) for g in group_list]
    out = []
    for i in range(len(lines)):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        the = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(the) < 1e-9:
            out.append(p1); out.append(p2); continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / the
        out.append((p1[0] + t * d1[0], p1[1] + t * d1[1]))
    return out if len(out) >= 3 else None


def octilinear(g, tol=0.1):
    part_list = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon' or p.is_empty: continue
        off = _ring_oct(list(p.exterior.coords), tol)
        if not off: continue
        inn = [r for r in (_ring_oct(list(i.coords), tol) for i in p.interiors) if r]
        q = Polygon(off, inn).buffer(0)
        if not q.is_empty: part_list.append(q)
    return unary_union(part_list) if part_list else g
