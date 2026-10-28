""
import math

import numpy as np
import scipy.sparse as sp
from scipy.ndimage import convolve
from scipy.sparse.linalg import splu
import shapely
from shapely import box, union_all
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from lernen import stromzonen as SZ
from lernen import stromzonen_minimax as SM

RHO = 1.72e-5
VIA_OHM_MM = 1.4e-3


def _dicke_z(B, R):
    from analyse.metriken import aufbau
    lagen = SZ._lagen(B)
    a = aufbau(len(lagen), [str(x).split('=')[0] for x in (R.get('gndlagen') or [])])
    return {L: a['lagen'][L]['t'] for L in lagen}, {L: a['lagen'][L]['z'] for L in lagen}


def _lasten(B, K, n):
    from analyse.metriken import _strom
    fl = ((K.get('netze') or {}).get(n) or {}).get('fluss') or {}
    if not isinstance(fl, dict) or not isinstance(fl.get('last') or {}, dict): return [], {}
    last = {p: float(q) for p, q in (fl.get('last') or {}).items() if float(q) > 0}
    I = _strom(n, K)[0] or 0.0
    s = sum(last.values())
    k = (K.get('netze') or {}).get(n) or {}
    herkunft = k.get('strom_A_quelle')
    wert = k.get('strom_A')
    if (s <= 0 and not fl and isinstance(herkunft, str) and herkunft.strip()
            and isinstance(wert, (int, float)) and not isinstance(wert, bool)
            and math.isfinite(wert) and wert > 0):


        ks = sorted({'%s:%s' % (q['ref'], q.get('num')) for q in B['pads'] if q['net'] == n})
        if len(ks) >= 2 and I > 0: return [ks[0]], {k: I / (len(ks) - 1) for k in ks[1:]}
    if s <= 0 or I <= 0: return [], {}
    quelle = list(fl.get('quelle') or [])

    if any(isinstance(q, str) and q in (fl.get('last') or {}) for q in quelle): return [], {}
    return quelle, {p: q * I / s for p, q in last.items()}


def laststatus(K, n, B=None):
    ""
    import math
    from analyse.minimax import lastfehler
    k = (K.get('netze') or {}).get(n) or {}
    fl = k.get('fluss') or {}
    if not fl:
        value = k.get('strom_A')
        if (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) and value > 0 and B is not None
                and all(_lasten(B, K, n))):
            return 'schaetzung'
        return 'unbekannt'
    if not isinstance(fl, dict): return 'unbekannt'
    sources, loads = fl.get('quelle'), fl.get('last')
    if (fl.get('einheit') != 'A' or fl.get('einheit_quelle') not in ('vorgabe', 'schaetzung')
            or not isinstance(sources, list) or not sources or not isinstance(loads, dict) or not loads
            or B is None): return 'unbekannt'
    if lastfehler(k): return 'unbekannt'
    pads = {f"{p['ref']}:{p.get('num')}": p['net'] for p in B.get('pads', [])}
    if any(not isinstance(p, str) or pads.get(p) != n for p in sources + list(loads)):
        return 'unbekannt'
    if set(sources) & set(loads): return 'unbekannt'
    return 'lastlos' if fl['gesamt'] == 0 else 'positiv'


def padgruppen(B, n, L, kern, frei, abstand=1.5, saum=.15):
    ""
    pads = [SZ._geo(dict(polys=[dict(aussen=q, innen=[]) for q in p['polys'][L]]))
            for p in B['pads'] if p['net'] == n and p.get('polys', {}).get(L)]
    shapes = []
    for i, a in enumerate(pads):
        for b in pads[i+1:]:
            if a.distance(b) > abstand: continue
            pair = a.union(b)
            clipped = pair.convex_hull.buffer(saum, 32).intersection(frei)
            for poly in getattr(clipped, 'geoms', [clipped]):
                if poly.geom_type == 'Polygon' and poly.buffer(1e-8).covers(pair):
                    shapes.append(poly)
    if not shapes: return kern
    copper = kern.union(unary_union(shapes)).buffer(0)
    return SZ.tangential_runden(copper, kern, frei, radius=saum)


def geruestkorridor(B, R, n, frei_fn, sperr_lagen=(), saum=.15):
    ""
    out = {}
    for L in SZ._lagen(B):
        if L in sperr_lagen: continue
        if not (any(t['net'] == n and t['layer'] == L and len(t['pts']) >= 2 for t in R.get('tracks', [])) or
                any(p['net'] == n and p.get('polys', {}).get(L) for p in B['pads'])): continue
        e = unary_union([g for m,g in SZ._teile(B,R,L) if m == n])
        if e.is_empty: continue
        out[L] = e.buffer(saum,32).intersection(frei_fn(n,L)).union(e)
    return out


def optimiere_netz(B, RR, K, n, lagen_t, lagen_z, frei_fn, h=0.1, r_dom=3.0, kappa=0.3, p=3.0, iters=60, r_filter=0.4,
                   rho_min=1e-3, start=1.0, sperr_lagen=(), padkerne=None, lam_min_mw=0.0, lam_max_mw=0.0):
    ""
    from lernen.kupferbalken import track_geometry
    d_via, _ = SZ._via_masse(RR)
    quelle, lasten = _lasten(B, K, n)
    if not quelle or not lasten: return None, dict(grund='keine Quelle oder Last')
    pads_n = [q for q in B['pads'] if q['net'] == n]
    vias_n = [v for v in RR.get('vias', []) if v.get('net') == n]
    Ls = []
    eigen = {}
    for L in SZ._lagen(B):
        if L in sperr_lagen: continue
        g = [track_geometry(t) for t in RR.get('tracks', []) if t.get('net') == n and t['layer'] == L and len(t['pts']) >= 2]
        pg = [Polygon(q).buffer(0) for pd in pads_n for q in (pd.get('polys') or {}).get(L, []) if len(q) >= 3]
        if not g and not pg: continue
        vg = [Point(v['x'], v['y']).buffer(d_via / 2, 16) for v in vias_n]
        eigen[L] = unary_union(g + pg + vg)
        if padkerne and L in padkerne and not padkerne[L].is_empty:
            eigen[L] = eigen[L].union(padkerne[L])
        Ls.append(L)
    if not Ls: return None, dict(grund='kein Kupfer')
    dom = {L: frei_fn(n, L).intersection(eigen[L].buffer(r_dom, 16)).union(eigen[L]) for L in Ls}
    x0, y0, x1, y1 = unary_union(list(dom.values())).bounds
    nx, ny = int(math.ceil((x1 - x0) / h)) + 2, int(math.ceil((y1 - y0) / h)) + 2
    X, Y = np.meshgrid(x0 + (np.arange(nx) - .5) * h, y0 + (np.arange(ny) - .5) * h)
    fest, ind, idx = {}, {}, {}
    nn = 0
    for L in Ls:
        e = eigen[L].buffer(h * 0.5, 8); shapely.prepare(e); dd = dom[L]; shapely.prepare(dd)
        f = shapely.contains_xy(e, X, Y)
        m = f | shapely.contains_xy(dd, X, Y)
        ix = np.full(m.shape, -1); ix[m] = np.arange(nn, nn + m.sum()); nn += int(m.sum())
        fest[L], ind[L], idx[L] = f, m, ix
    if nn == 0: return None, dict(grund='leeres Raster')

    a, b, g0, ea = [], [], [], []
    for L in Ls:
        ix = idx[L]; gl = lagen_t[L] / RHO
        for sl_a, sl_b in (((slice(None), slice(None, -1)), (slice(None), slice(1, None))), ((slice(None, -1), slice(None)), (slice(1, None), slice(None)))):
            A_, B_ = ix[sl_a], ix[sl_b]; m = (A_ >= 0) & (B_ >= 0)
            a.append(A_[m]); b.append(B_[m]); g0.append(np.full(m.sum(), gl)); ea.append(np.ones(m.sum(), bool))
    va, vb, vg = [], [], []
    for v in vias_n + [dict(x=q['x'], y=q['y']) for q in pads_n if q.get('drill')]:
        cx, cy = int(round((v['x'] - X[0, 0]) / h)), int(round((v['y'] - Y[0, 0]) / h))
        knoten = []
        for L in sorted(Ls, key=lambda L: lagen_z[L]):
            if 0 <= cy < ny and 0 <= cx < nx and idx[L][cy, cx] >= 0: knoten.append((L, idx[L][cy, cx]))
        for (L1, k1), (L2, k2) in zip(knoten[:-1], knoten[1:]):
            va.append(k1); vb.append(k2); vg.append(1.0 / (VIA_OHM_MM * max(0.05, abs(lagen_z[L2] - lagen_z[L1]))))
    a = np.concatenate(a + [np.asarray(va, int)]); b = np.concatenate(b + [np.asarray(vb, int)])
    g0 = np.concatenate(g0 + [np.asarray(vg, float)]); ebene = np.concatenate(ea + [np.zeros(len(va), bool)])
    rho = np.zeros(nn); fixed = np.zeros(nn, bool)
    for L in Ls:
        ix = idx[L]; m = ind[L]
        rho[ix[m]] = np.where(fest[L][m], 1.0, start); fixed[ix[m]] = fest[L][m]

    def padknoten(key):
        ref, num = key.split(':', 1); out = []
        for pd in pads_n:
            if pd['ref'] != ref or str(pd.get('num')) != num: continue
            for L, qs in (pd.get('polys') or {}).items():
                if L not in idx: continue
                geo = unary_union([Polygon(q).buffer(h * 0.5) for q in qs if len(q) >= 3])
                mm = shapely.intersects_xy(geo, X, Y) & (idx[L] >= 0)
                out.extend(idx[L][mm])
        return np.unique(np.asarray(out, int))
    quell = np.unique(np.concatenate([padknoten(q) for q in quelle] + [np.zeros(0, int)]))
    rhs = np.zeros(nn)
    for k, I in lasten.items():
        kn = padknoten(k)
        if len(kn): rhs[kn] += I / len(kn)
    if not len(quell) or rhs.sum() <= 0: return None, dict(grund='Quelle/Last nicht im Raster')
    from scipy.sparse.csgraph import connected_components
    adj = sp.coo_matrix((np.ones(len(a)), (a, b)), shape=(nn, nn))
    _, lab = connected_components(adj, directed=False)
    erreicht = np.isin(lab, np.unique(lab[quell]))
    verloren = float(rhs[~erreicht].sum())
    if verloren > 0.01 * float(rhs.sum()):
        return None, dict(grund='Lasten nicht mit der Quelle verbunden (%.3f A von %.3f A)' % (verloren, float(rhs.sum())))
    rhs[~erreicht] = 0.0
    frei_k = erreicht.copy(); frei_k[quell] = False; rhs[quell] = 0.0
    if rhs.sum() <= 0: return None, dict(grund='Lasten nicht mit der Quelle verbunden')
    rf = max(1, int(round(r_filter / h)))
    yy_, xx_ = np.mgrid[-rf:rf + 1, -rf:rf + 1]; ker = np.maximum(0.0, rf + 0.5 - np.hypot(xx_, yy_))
    design = ~fixed
    verlauf = []
    p_ziel = p
    for it in range(iters):

        p = 1.0 if it < iters // 4 else (2.0 if it < iters // 2 else p_ziel)
        s = rho_min + rho ** p * (1 - rho_min)
        sa, sb = s[a], s[b]
        f = np.where(ebene, 2 * sa * sb / (sa + sb), 1.0)
        g = g0 * f
        Lm = sp.coo_matrix((np.r_[g, g, -g, -g], (np.r_[a, b, a, b], np.r_[a, b, b, a])), shape=(nn, nn)).tocsc()
        phi = np.zeros(nn)
        try: phi[frei_k] = splu(Lm[frei_k][:, frei_k]).solve(rhs[frei_k])
        except RuntimeError: return None, dict(grund='Gleichungssystem singulär')
        P = float(rhs @ phi)
        dphi2 = (phi[a] - phi[b]) ** 2

        dsa = np.where(ebene, -g0 * 2 * sb ** 2 / (sa + sb) ** 2 * dphi2, 0.0)
        dsb = np.where(ebene, -g0 * 2 * sa ** 2 / (sa + sb) ** 2 * dphi2, 0.0)
        dPds = np.bincount(a, dsa, nn) + np.bincount(b, dsb, nn)
        dPdr = dPds * p * np.maximum(rho, 1e-9) ** (p - 1) * (1 - rho_min)

        dcf = np.zeros(nn)
        for L in Ls:
            ix = idx[L]; m = ix >= 0
            R_ = np.zeros(ix.shape); D_ = np.zeros(ix.shape); W_ = np.zeros(ix.shape)
            R_[m] = rho[ix[m]]; D_[m] = dPdr[ix[m]]; W_[m] = 1.0
            num = convolve(R_ * D_, ker, mode='constant'); den = convolve(W_, ker, mode='constant') * np.maximum(R_, 1e-3)
            dcf[ix[m]] = (num[m] / np.maximum(den[m], 1e-12))
        A = float(rho.sum() * h * h)
        lam = kappa * P / max(A, 1e-9)






        lam = max(lam, lam_min_mw * 1e-3)




        if lam_max_mw: lam = min(lam, lam_max_mw * 1e-3)
        Bc = np.maximum(-dcf, 0.0) / (lam * h * h)
        neu = np.clip(rho * np.sqrt(Bc), np.maximum(0.0, rho - 0.2), np.minimum(1.0, rho + 0.2))
        neu = np.where(design & erreicht, np.clip(neu, 0.0, 1.0), np.where(erreicht, 1.0, 0.0))
        aend = float(np.max(np.abs(neu - rho))) if nn else 0.0
        rho = neu
        verlauf.append(dict(it=it, p=p, P_mW=round(P * 1000, 4), A_mm2=round(A, 2), aend=round(aend, 3), verloren_A=round(verloren, 3)))
        if aend < 0.01 and it > iters // 2 + 5: break
    out = {}
    for L in Ls:
        ix = idx[L]; m = (ix >= 0)
        on = np.zeros(ix.shape, bool); on[m] = rho[ix[m]] >= 0.5
        on |= fest[L]
        iy, ix2 = np.nonzero(on)
        g = union_all(box(X[iy, ix2] - .52 * h, Y[iy, ix2] - .52 * h, X[iy, ix2] + .52 * h, Y[iy, ix2] + .52 * h)) if len(iy) else None
        out[L] = eigen[L] if g is None else g.intersection(dom[L]).union(eigen[L])
    return out, dict(verlauf=verlauf, knoten=nn, lagen=Ls, strom_A=round(sum(lasten.values()), 3))
def _schlank(t, w_max):
    ""
    t = dict(t, w=[min(w, w_max) for w in t['w']])
    if 'items' in t: t['items'] = [list(it[:-1]) + [min(it[-1], w_max)] for it in t['items']]
    return t
def _bahn_id(t):
    return '%s|%s|%.4f,%.4f|%.4f,%.4f|%d' % (t.get('net'), t['layer'], *t['pts'][0], *t['pts'][-1], len(t['pts']))


def _skelett_breiten(RR, n, L, g, w_max):
    ""
    from shapely.geometry import LineString
    gg = g.buffer(1e-3, 8); shapely.prepare(gg); neu = {}
    for t in RR.get('tracks', []):
        if t.get('net') != n or t['layer'] != L or len(t['pts']) < 2 or 'items' in t: continue
        ws = [w if w <= w_max or a == b or gg.contains(LineString([a, b]).buffer(w / 2, 16)) else w_max
              for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
        if ws != list(t['w']): neu[_bahn_id(t)] = ws
    return neu


def skelett_anwenden(R, zs):
    ""
    neu = {}
    for z in zs: neu.update(z.pop('skelett', None) or {})
    if not neu: return 0
    k = 0
    for i, t in enumerate(R.get('tracks', [])):
        if 'items' not in t and len(t['pts']) >= 2 and _bahn_id(t) in neu:
            R['tracks'][i] = dict(t, w=neu[_bahn_id(t)]); k += 1
    return k


def zonen(B, R, K, gndlagen=None, clr=None, rand=0.25, h=0.15, r_dom=3.0, kappa=0.3, p=3.0, iters=40, r_glatt=0.15,
          via_feld=True, reihenfolge='kette', okt=0.3, r_filter=0.4, r_huelle=0.45, r_rund=0.15, loecher_fuellen=True,
          kanten='frei', r_andock=0.0, andock_vias=False, keil_max=45.0, skelett=None, nur_flaeche=0.2, lam_min_mw=0.0, r_auf=None, lam_max_mw=2.0):
    from analyse.schaltknoten import an as sw_an, korrigiere as sw_strom, kompakt as sw_form
    sw_enabled = sw_an(R)
    if sw_enabled:
        import copy
        K = copy.deepcopy(K)
        sw_strom(B, K)
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gndlagen(R, gndlagen))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand, SZ.QS)
    _, b_via = SZ._via_masse(R)
    hole = float((R.get('optionen') or {}).get('h2h') or .2)
    loecher = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, 16)) for v in R.get('vias', [])]
    loecher += [(q['net'], Point(q['x'], q['y']).buffer(q['drill'] / 2, 16)) for q in B['pads'] if q.get('drill')]
    prio = SZ.vorrang(B, R, K)
    netze = sorted(SZ.stromnetze(K, gnd), key=lambda n: (-prio[n]['gewicht'], n))
    if reihenfolge == 'kette':
        import copy
        from analyse.strom_kette import kette
        R1 = copy.deepcopy(R)
        z1 = SM.zonen(B, R1, K, gndlagen, clr=clr, rand=rand, via_feld=False)
        KE = kette(B, dict(R1, zonen=[z for z in R1.get('zonen', []) if z.get('quelle') != 'stromzonen'] + z1), K)
        eta = {n: v['wirkungsgrad'] for n, v in KE['netze'].items()}
        netze = sorted(netze, key=lambda n: (eta.get(n, 1.0), n))
    lagen_t, lagen_z = _dicke_z(B, R)
    RR = dict(R, zonen=[z for z in R.get('zonen', []) if z.get('quelle') != 'stromzonen'])
    out = []; infos = {}; schutz = SZ.bahnschutz_an(RR)

    def frei(n, L):
        teile = SZ._teile(B, RR, L)
        fremd = unary_union([g for m, g in teile if m != n] + [g for m, g in loecher if m != n])
        f = umriss.difference(fremd.buffer(clr + .003, 16)).difference(SZ.sperrflaeche(B, L, 'flaechen'))
        f = SZ.maskenfrei(f, B, n, L, clr, 16)
        f = f.difference(unary_union([g for m, g in loecher if m != n]).buffer(hole + .003, 16))
        if not schutz: return f
        _, moat = SZ._bahnschutz(RR, L, n)
        return f.difference(moat) if not moat.is_empty else f

    optimiert = []
    for n in netze:
        if sw_enabled and K['netze'][n].get('art') == 'sw':
            geo, info = sw_form(B, RR, K, n, [L for L in SZ._lagen(B) if L not in gl], frei)
            infos[n] = info
            for L, g in sorted(geo.items()):
                e = unary_union([gg for m, gg in SZ._teile(B, dict(RR, zonen=[]), L) if m == n])
                if g.is_empty or g.difference(e).area < .01: continue
                z = dict(net=n, layer=L, quelle='stromzonen', strom_a=K['netze'][n].get('strom_A'),
                         strom_A_quelle=K['netze'][n].get('strom_A_quelle'), modell='sw_kompakt',
                         polys=SZ._polys(g), flaeche_mm2=round(g.area, 2), eigen_mm2=round(e.area, 2), vorrang=prio[n])
                out.append(z); RR['zonen'].append(z)
            continue
        padkerne = {}
        status = laststatus(K, n, B) if R.get('k20_korridor') else None
        if R.get('k20_korridor') and status in ('positiv', 'lastlos'):
            from lernen.k20_padgruppen import kern
            padkerne = {L: kern(B, RR, K, n, L, frei(n, L).buffer(-.002, 8))
                        for L in SZ._lagen(B) if L not in gl}



        RRo = dict(RR, tracks=[_schlank(t, skelett) if t.get('net') == n else t for t in RR.get('tracks', [])]) if skelett else RR
        if R.get('k20_korridor') and status not in ('positiv', 'lastlos'):
            geo, info = None, dict(grund='unvollstaendiger Lastvertrag')
        else:
            geo, info = optimiere_netz(B, RRo, K, n, lagen_t, lagen_z, frei, h=h, r_dom=r_dom, kappa=kappa, p=p, iters=iters, r_filter=r_filter,
                                       sperr_lagen=gl, padkerne=padkerne, lam_min_mw=lam_min_mw, lam_max_mw=lam_max_mw)
        fallback = False
        if (R.get('r36_geruest') and not geo and info.get('grund') == 'keine Quelle oder Last'
                and (SZ._strom_bekannt(K['netze'][n]) or 0.) >= SZ.I_ZONE):
            geo = geruestkorridor(B,RR,n,frei,gl)
            fallback = bool(geo)
            info = dict(info, ersatz='geruestkorridor_ohne_lastmodell', stromfeld_optimiert=False)
        infos[n] = info
        if geo:
            optimiert.append(n)
        elif R.get('k20_korridor') and status == 'lastlos':


            geo = {L: kern.union(unary_union([g for m,g in SZ._teile(B, dict(RR,zonen=[]), L) if m == n]))
                   for L,kern in padkerne.items() if not kern.is_empty}
            info['padgruppen_ohne_strommodell'] = bool(geo)
        if not geo: continue
        I = SZ.netz_strom(K, n)
        for L, g in sorted(geo.items()):
            if L in gl: continue
            e = unary_union([gg for m, gg in SZ._teile(B, dict(RRo, zonen=[]), L) if m == n])
            anschlusskern = e.union(padkerne[L]) if L in padkerne and not padkerne[L].is_empty else e
            fr = frei(n, L).union(e)

            g = SZ._oeffnen_g(g, r_glatt if r_auf is None else r_auf).union(e.intersection(g))
            g = _fl(SZ._schliessen_g(g, r_glatt).intersection(fr))
            teile = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(teile + [anschlusskern]).buffer(0)


            if r_huelle:
                g = _fl(SZ._schliessen_g(g, r_huelle).intersection(fr)).union(e)
                g = SZ._oeffnen_g(g, r_glatt).union(e.intersection(g))
            if okt and kanten == 'okt':
                g = _fl(oktilinear(g, okt).intersection(fr)).union(e)
            elif okt:


                g = _fl(g.simplify(okt, preserve_topology=True).intersection(fr)).union(e)


            if r_andock:
                fen = g.buffer(2 * r_andock + .1, 8)
                if andock_vias: blk = umriss.difference(fr).intersection(fen)
                else:
                    from lernen.kupferbalken import track_geometry
                    lin = [track_geometry(tr) for tr in RR.get('tracks', []) if tr['layer'] == L and tr.get('net') != n and len(tr['pts']) >= 2]
                    lin += [Polygon(q).buffer(0) for pd in B['pads'] if pd['net'] != n for q in (pd.get('polys') or {}).get(L, []) if len(q) >= 3]
                    lin += [SZ._geo(z) for z in RR['zonen'] if z['layer'] == L and z['net'] != n]
                    lin = [x for x in lin if x.intersects(fen)]
                    blk = unary_union(lin).buffer(clr + .003, 16).intersection(fen) if lin else None
                if blk is not None and not blk.is_empty:
                    g = _fl(g.union(blk).buffer(r_andock, 16).buffer(-r_andock, 16).intersection(fr)).union(e)
            teile = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(teile + [e]).buffer(0)
            fs = frei(n, L).buffer(-.002, 8).union(e)
            g = _fl(g.intersection(fs)).union(e)
            if keil_max:
                g = keile_fuellen(g, fs, [gg for m, gg in SZ._teile(B, RR, L) if m != n], clr, keil_max)



            g = _fl(g.buffer(-r_rund, 32).buffer(r_rund, 32).intersection(g)).union(e)
            kehle = _fl(g.buffer(r_rund, 32).buffer(-r_rund, 32).difference(g).intersection(fs))
            g = g.union(kehle).buffer(0)
            if loecher_fuellen:
                inn = unary_union([Polygon(i) for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' for i in q.interiors])
                if not inn.is_empty:
                    neu = _fl(inn.intersection(fs))
                    gg = g.union(neu).buffer(0)
                    g = g.union(gg.buffer(-r_rund, 32).buffer(r_rund, 32).intersection(neu)).buffer(0)
            g = _fl(g.intersection(fs)).union(e)
            teile = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= 0.05 and q.intersects(e)]
            g = unary_union(teile + [anschlusskern]).buffer(0).simplify(.0005, preserve_topology=True)
            if nur_flaeche:






                g = unary_union([q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon'
                                 and q.difference(e).area >= nur_flaeche]).buffer(0)
            if R.get('r36_padgruppen'):
                g = padgruppen(B, n, L, g, fs)
            if g.is_empty or g.area <= e.area + 0.01: continue
            z = dict(net=n, layer=L, quelle='stromzonen', strom_a=round(I or 0, 3), polys=SZ._polys(g), flaeche_mm2=round(g.area, 2),
                     eigen_mm2=round(e.area, 2), vorrang=prio[n], modell='geruest-ohne-lastmodell' if fallback else 'topo', kappa=kappa)
            if not (K['netze'][n].get('fluss') or {}) and laststatus(K, n, B) == 'schaetzung':
                z['strommodell'] = 'schaetzung; Quelle und gleiche Lastverteilung aus Padreihenfolge, kein vollständiger Lastvertrag'
            if info.get('padgruppen_ohne_strommodell'):
                status = laststatus(K, n, B)
                z['strom_a'] = 0.0 if status == 'lastlos' else None
                z['strommodell'] = status+'; nur geometrische Padverbindung, kein positiver Topo-DC-Lastfall'
            if skelett:
                z['skelett'] = _skelett_breiten(RR, n, L, g, skelett)
            out.append(z); RR['zonen'].append(z)
        infos[n] = info
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true'):
        from lernen.k17_kontur import kerben_fuellen
        for z in out:
            if K['netze'][z['net']].get('art') == 'sw':
                continue
            g = SZ._geo(z)
            fs = frei(z['net'], z['layer']).buffer(-.002, 8)
            proposal = kerben_fuellen(g, fs)
            if proposal.area > g.area + 1e-9:
                z.update(polys=SZ._polys(proposal), flaeche_mm2=round(proposal.area, 2))

    if out and (R.get('k20_korridor') or R.get('r36_padgruppen') or R.get('topo_diagnose')):
        out[0]['topo_info'] = infos
    if via_feld and out:
        stroeme = {n: SM.segmentstroeme(B, R, K, n) for n in netze}
        via_netze = optimiert if R.get('k20_korridor') else netze
        if sw_enabled: via_netze = [n for n in via_netze if K['netze'][n].get('art') != 'sw']
        nv = SM.via_felder(B, R, RR, K, out, via_netze, stroeme, clr, umriss, hole)
        R.setdefault('vias', []).extend(nv)
    return out


def keile_fuellen(g, fs, fremd, clr, w_max=30.0, w_ende=0.45, l_max=2.0, l_min=0.1):
    ""
    fen = g.buffer(l_max + .5, 8)
    fremd = [x for x in fremd if x.intersects(fen)]
    if not fremd: return g
    h = clr / 2 + .01
    kupfer = unary_union([g.buffer(h, 8)] + [x.buffer(h, 8) for x in fremd])
    weiss = fen.difference(kupfer)
    nah = g.buffer(h + .04, 8).difference(g.buffer(h - .04, 8))
    neu = []
    for poly in getattr(weiss, 'geoms', [weiss]):
        if poly.geom_type != 'Polygon' or poly.area < 1e-4: continue
        for ring, aussen in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
            pts = list(ring.simplify(0.01).coords)[:-1]
            if len(pts) < 3: continue
            fl = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
            for i in range(len(pts)):
                a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
                v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
                l1, l2 = math.hypot(*v1), math.hypot(*v2)
                if l1 < l_min or l2 < l_min: continue
                w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
                kreuz = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
                konvex = (kreuz > 0) == (fl > 0) if aussen else (kreuz > 0) != (fl > 0)
                if not konvex or w >= w_max or not nah.contains(Point(b)): continue
                L = min(l_max, w_ende / max(1e-3, 2 * math.tan(math.radians(w) / 2)))
                u1, u2 = (v1[0] / l1, v1[1] / l1), (v2[0] / l2, v2[1] / l2)
                neu.append(Polygon([b, (b[0] + L * u1[0], b[1] + L * u1[1]), (b[0] + L * u2[0], b[1] + L * u2[1])]).buffer(h, 8))
    if not neu: return g
    add = _fl(unary_union(neu).intersection(fs))
    add = unary_union([q for q in getattr(add, 'geoms', [add]) if q.geom_type == 'Polygon' and q.buffer(.01).intersects(g)])
    return g.union(add).buffer(0) if not add.is_empty else g


def _fl(g):
    ""
    return unary_union([q for q in shapely.get_parts(g) if q.geom_type == 'Polygon' and q.area > 1e-7])


def _ring_okt(coords, tol):
    ""
    from shapely.geometry import LinearRing
    r = LinearRing(coords).simplify(tol, preserve_topology=False)
    pts = list(r.coords)[:-1]
    if len(pts) < 3: return None
    gruppen = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        l = math.dist(a, b)
        if l < 1e-6: continue
        k = int(round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) / 45.0)) % 8
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        if gruppen and gruppen[-1][0] == k: g = gruppen[-1]; g[1] += l; g[2] += mx * l; g[3] += my * l
        else: gruppen.append([k, l, mx * l, my * l])
    if len(gruppen) > 1 and gruppen[0][0] == gruppen[-1][0]:
        g = gruppen.pop(); gruppen[0][1] += g[1]; gruppen[0][2] += g[2]; gruppen[0][3] += g[3]
    if len(gruppen) < 3: return None
    linien = [((g[2] / g[1], g[3] / g[1]), (math.cos(math.radians(45 * g[0])), math.sin(math.radians(45 * g[0])))) for g in gruppen]
    out = []
    for i in range(len(linien)):
        (p1, d1), (p2, d2) = linien[i - 1], linien[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            out.append(p1); out.append(p2); continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        out.append((p1[0] + t * d1[0], p1[1] + t * d1[1]))
    return out if len(out) >= 3 else None


def oktilinear(g, tol=0.1):
    ""
    teile = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon' or p.is_empty: continue
        aus = _ring_okt(list(p.exterior.coords), tol)
        if not aus: continue
        inn = [r for r in (_ring_okt(list(i.coords), tol) for i in p.interiors) if r]
        q = Polygon(aus, inn).buffer(0)
        if not q.is_empty: teile.append(q)
    return unary_union(teile) if teile else g
