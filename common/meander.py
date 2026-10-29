import math
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

K = 8 - 2 * math.pi
TOL = 0.002
MICRO = 0.05
STANDARD = dict(levels=(0.30, 0.25, 0.20), s_values=(0.6, 0.5, 0.4, 0.35, 0.3, 0.28), r_shares=(0.5, 0.4, 0.3), n_max=6, amax=1.0,
                rmin=0.07, rmin_emergency=0.05, lmin=0.6, border_t=0.15, step_t=0.05, lat_max=1.0, close_max=4.0, sites=3, emin=0.001, chord=0.004)


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def octal(a, b, tol=0.05):
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
    return min(ang, 45 - ang) < tol




def arc_geometry(a, m, b):
    ux, uy = m[0] - a[0], m[1] - a[1]; vx, vy = b[0] - a[0], b[1] - a[1]
    det = 2 * (ux * vy - uy * vx)
    if abs(det) < 1e-14: raise ValueError('degenerate arc')
    uu, vv = ux * ux + uy * uy, vx * vx + vy * vy
    c = (a[0] + (uu * vy - vv * uy) / det, a[1] + (ux * vv - vx * uu) / det)
    w = [math.atan2(p[1] - c[1], p[0] - c[0]) for p in (a, m, b)]
    ccw = (w[2] - w[0]) % (2 * math.pi)
    sweep = ccw if (w[1] - w[0]) % (2 * math.pi) <= ccw + 1e-9 else ccw - 2 * math.pi
    return c, dist(a, c), w[0], sweep


def arc_length(a, m, b):
    _, r, _, sw = arc_geometry(a, m, b)
    return r * abs(sw)


def arc_points(a, m, b, chord=0.004):
    c, r, w0, sw = arc_geometry(a, m, b)
    n = max(2, int(math.ceil(abs(sw) / (2 * math.acos(max(-1.0, 1 - chord / max(r, 1e-9)))))) if r > chord else 2)
    pts = [tuple(a)] + [(c[0] + r * math.cos(w0 + sw * k / n), c[1] + r * math.sin(w0 + sw * k / n)) for k in range(1, n)] + [tuple(b)]
    return pts


def primary_length(p):
    return dist(p[1], p[2]) if p[0] == 'seg' else arc_length(p[1], p[2], p[3])


def primary_points(p, chord=0.004):
    return [tuple(p[1]), tuple(p[2])] if p[0] == 'seg' else arc_points(p[1], p[2], p[3], chord)


def line(prims, chord=0.004):
    pts = []
    for p in prims:
        q = primary_points(p, chord)
        if pts and dist(pts[-1], q[0]) < 1e-9: q = q[1:]
        pts += q
    return pts




def dome(x0, s, r, A):
    pr = []

    def arc(c, a0, a1):
        p0 = (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0)); p1 = (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1))
        am = (a0 + a1) / 2; pr.append(('arc', p0, (c[0] + r * math.cos(am), c[1] + r * math.sin(am)), p1))

    def seg(p0, p1):
        if dist(p0, p1) > 1e-6: pr.append(('seg', p0, p1))
    pi = math.pi
    arc((x0, r), -pi / 2, 0)
    seg((x0 + r, r), (x0 + r, A - r))
    arc((x0 + 2 * r, A - r), pi, pi / 2)
    seg((x0 + 2 * r, A), (x0 + s, A))
    arc((x0 + s, A - r), pi / 2, 0)
    seg((x0 + s + r, A - r), (x0 + s + r, r))
    arc((x0 + s + 2 * r, r), pi, 3 * pi / 2)
    return pr, x0 + s + 2 * r

def meander_local(n, s, r, A):
    gap = s - 2 * r; pr = []; x = 0.0
    for k in range(n):
        p, x = dome(x, s, r, A); pr += p
        if k < n - 1:
            if gap > 1e-6: pr.append(('seg', (x, 0.0), (x + gap, 0.0)))
            x += gap
    return pr, x


def extra_length(n, s, r, A):
    return n * (2 * A - K * r)


def straight_ok(l):
    return l < 1e-6 or l >= MICRO - 1e-9


def shaping(e, n, s, r, rmin=0.05, amax=1.0):
    A = (e / n + K * r) / 2
    if A < 2 * r:
        r = e / n / (4 - K); A = 2 * r
        if 2 * r > s + 1e-9: return None
    if r < rmin - 1e-9 or A > amax + 1e-9: return None
    if not (straight_ok(A - 2 * r) and straight_ok(s - 2 * r)): return None
    return r, A


def world(pr, a, u, v):
    T = lambda q: (a[0] + q[0] * u[0] + q[1] * v[0], a[1] + q[0] * u[1] + q[1] * v[1])
    return [(p[0],) + tuple(T(q) for q in p[1:]) for p in pr]




class Nodes:
    def __init__(self, tol=TOL):
        self.p = []; self.tol = tol

    def __call__(self, q):
        for i, p in enumerate(self.p):
            if dist(p, q) < self.tol: return i
        self.p.append((float(q[0]), float(q[1]))); return len(self.p) - 1


def chain_list(edges, pinned=()):
    kn = Nodes()
    for q in pinned: kn(q)
    pad_pts = set(range(len(kn.p)))
    E = []
    for k in edges:
        i, j = kn(k['a']), kn(k['b'])
        if i == j and k['type_'] == 'seg': continue
        E.append((i, j, k))
    nb = {}
    for ei, (i, j, k) in enumerate(E):
        nb.setdefault(i, []).append(ei); nb.setdefault(j, []).append(ei)
    fixed_pts = {v for v in nb if len(nb[v]) != 2 or v in pad_pts}
    used = set(); res = []
    for s0 in sorted(fixed_pts):
        for e0 in nb[s0]:
            if e0 in used: continue
            nodes = [s0]; nc = []; v = s0; e = e0
            while True:
                used.add(e); i, j, k = E[e]
                w_ = j if i == v else i
                if k['type_'] == 'seg': nc.append((('seg', kn.p[v], kn.p[w_]), k))
                else:
                    m = tuple(k['m'])
                    nc.append((('arc', kn.p[v], m, kn.p[w_]), k))
                nodes.append(w_); v = w_
                if v in fixed_pts: break
                nx = [x for x in nb[v] if x not in used]
                if not nx: break
                e = nx[0]
            res.append(dict(nodes=[kn.p[x] for x in nodes], edges=[p for p, _ in nc], w=[k['w'] for _, k in nc],
                            uuids=sorted({k['uuid'] for _, k in nc if k.get('uuid')})))
    return res


def total_length(prims):
    return sum(primary_length(p) for p in prims)


def straighten(link_chain, vacant, lat_max=1.0, chord=0.004):
    pr = link_chain['edges']; net_copper = len(pr)
    pts0 = [tuple(p[1]) for p in pr] + [tuple(pr[-1][-1])]
    out = []; prior = []; i = 0
    while i < net_copper:
        best = None
        for j in range(net_copper, i + 1, -1):
            a, b = pts0[i], pts0[j]
            if dist(a, b) < 1e-6 or not octal(a, b): continue
            L = dist(a, b); ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            zw = [q for p in pr[i:j] for q in primary_points(p, chord)[1:]][:-1]
            lat = [-(q[0] - a[0]) * uy + (q[1] - a[1]) * ux for q in zw]
            proj = [(q[0] - a[0]) * ux + (q[1] - a[1]) * uy for q in zw]
            if not lat or max(abs(x) for x in lat) > lat_max or min(proj) < -0.01 or max(proj) > L + 0.01: continue
            if min(lat) < -1e-4 and max(lat) > 1e-4: continue
            way = total_length(pr[i:j])
            if way - L < 0.005: continue
            if not vacant(a, b): continue
            best = j; places = [q for q, x in zip(zw, lat) if abs(x) > 0.02]; break
        if best is None:
            out.append(pr[i]); i += 1
        else:
            a, b = pts0[i], pts0[best]
            prior.append(dict(from_=list(a), until=list(b), more=round(total_length(pr[i:best]) - dist(a, b), 4), middle=[(a[0] + b[0]) / 2, (a[1] + b[1]) / 2],
                            arcs=sum(1 for p in pr[i:best] if p[0] == 'arc'), places=[list(q) for q in places[::max(1, len(places) // 12)]]))
            out.append(('seg', a, b)); i = best

    red = []
    for p in out:
        if red and p[0] == 'seg' and red[-1][0] == 'seg':
            a, m, b = red[-1][1], red[-1][2], p[2]
            cr = (m[0] - a[0]) * (b[1] - m[1]) - (m[1] - a[1]) * (b[0] - m[0])
            if abs(cr) < 1e-6 and ((m[0] - a[0]) * (b[0] - m[0]) + (m[1] - a[1]) * (b[1] - m[1])) > 0:
                red[-1] = ('seg', a, b); continue
        red.append(p)
    return red, prior




class Environment:

    def __init__(self, fwd, net_list, replaced=None):
        self.fwd = fwd; self.net_list = set(net_list); self.replaced = dict(replaced or {})
        self.owned = max([fwd.net_spacing.get(n, 0.0) for n in self.net_list] + [0.0])

    def verify(self, cu, layer, reserve=None):
        fwd = self.fwd; res = fwd.res if reserve is None else reserve
        best = 1e9
        if layer in fwd.tree_idx:
            for i in fwd.tree_idx[layer].query(cu.buffer(fwd.dmax + 0.35 + res)):
                n, g, d, track = fwd.obj[layer][int(i)]
                if n in self.net_list or n in self.replaced: continue
                dd = max(d, self.owned if track else 0.0, fwd.net_spacing.get(n, 0.0) if track else 0.0)
                x = cu.distance(g)
                if x < dd + res: return None
                if not n.startswith('#'): best = min(best, x)
        for n, geo in sorted(self.replaced.items()):
            g = geo.get(layer)
            if n in self.net_list or g is None or g.is_empty: continue
            x = cu.distance(g)
            if x < max(fwd.drc, self.owned) + res: return None
            best = min(best, x)
        return best


def prims_copper(prims, w, chord=0.004):
    return unary_union([LineString(primary_points(p, chord)).buffer(w / 2, 16) for p in prims])




def _level(x, levels):
    for k, s in enumerate(levels):
        if x >= s - 1e-9: return k
    return None


def crests_candidate(seg, e, w, layer, environment, partner_geo, owned_geo, ref, opt, n_list=None, pclr=None):
    a, b = seg; L = dist(a, b); o = opt
    if L < o['lmin']: return None
    u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
    best = None
    for sgn in (1, -1):
        v = (-u[1] * sgn, u[0] * sgn)
        middle0 = (a[0] + u[0] * L / 2, a[1] + u[1] * L / 2); probe = (middle0[0] + v[0] * 0.2, middle0[1] + v[1] * 0.2)
        if partner_geo is not None and not partner_geo.is_empty and Point(probe).distance(partner_geo) < Point(middle0).distance(partner_geo):
            continue
        for n in (n_list or range(1, o['n_max'] + 1)):
            if best and best['rank'][:2] == (0, 0) and best['n'] < n: break
            for s_ in o['s_values']:
                for rf in o['r_shares']:
                    f = shaping(e, n, s_, s_ * rf, o['rmin'], o['amax'])
                    if f is None: continue
                    r, A = f
                    pr, F = meander_local(n, s_, r, A)
                    if L - F < 2 * o['border_t'] - 1e-9: continue

                    cu0 = LineString(line(pr, o['chord'])).buffer(w / 2, 16)
                    ts = [o['border_t'] + o['step_t'] * k for k in range(int((L - F - 2 * o['border_t']) / o['step_t'] + 1e-9) + 1)]
                    if ref:
                        ts.sort(key=lambda t: (min(dist((a[0] + u[0] * (t + F / 2), a[1] + u[1] * (t + F / 2)), q) for q in ref), t))
                    for t in ts:
                        st = (a[0] + u[0] * t, a[1] + u[1] * t)
                        middle = (st[0] + u[0] * F / 2, st[1] + u[1] * F / 2)
                        close = min((dist(middle, q) for q in ref), default=0.0)
                        far = 1 if close > o['close_max'] else 0
                        rank_rest = (n, round(abs(r / s_ - 0.5), 4), round(abs(s_ - 0.6), 4), round(close, 1))
                        if best and (far, 0) + rank_rest >= best['rank']: continue
                        M = (u[0], v[0], u[1], v[1], st[0], st[1])
                        cu = affinity.affine_transform(cu0, M)
                        if partner_geo is not None and not partner_geo.is_empty and \
                                cu.distance(partner_geo) < (environment.fwd.drc + environment.fwd.res if pclr is None else pclr): continue
                        if owned_geo is not None and not owned_geo.is_empty and cu.distance(owned_geo) < max(environment.fwd.drc, w): continue
                        dfr = environment.verify(cu, layer)
                        if dfr is None: continue
                        level = _level(dfr, o['levels'])
                        if level is None: continue
                        rank = (far, level) + rank_rest
                        if best is None or rank < best['rank']:
                            best = dict(rank=rank, n=n, s=s_, r=round(r, 6), A=round(A, 6), F=round(F, 6), t=round(t, 4), start=st, u=u, v=v,
                                        prims=world(pr, st, u, v), spacing_foreign=round(dfr, 3), more=round(extra_length(n, s_, r, A), 6), board_side=sgn)
    return best

def pair_schedule(pair, environment, opt=None):
    o = dict(STANDARD, **(opt or {}))
    sides = {}
    for s in ('P', 'N'):
        sides[s] = {L: chain_list(d['edges'], d.get('pinned', ())) for L, d in sorted(pair[s]['cu_layers'].items())}

    prev_geo = {s: {L: unary_union([LineString(line(k['edges'])).buffer(k['w'][0] / 2, 16) for k in ks]) if ks else None
                   for L, ks in sides[s].items()} for s in ('P', 'N')}

    pclr = {}
    for L in sorted(set(prev_geo['P']) & set(prev_geo['N'])):
        gp, gn = prev_geo['P'][L], prev_geo['N'][L]
        if gp is not None and gn is not None and not gp.is_empty and not gn.is_empty:
            pclr[L] = min(environment.fwd.drc + environment.fwd.res, gp.distance(gn)) - 1e-4
    info0 = dict(pair_spacing={L: round(v + 1e-4, 4) for L, v in pclr.items()})
    new = {'P': {}, 'N': {}}; prior = {'P': [], 'N': []}
    for s in ('P', 'N'):
        q = 'N' if s == 'P' else 'P'
        for L, ks in sides[s].items():
            for k in ks:
                w = k['w'][0]
                if max(k['w']) - min(k['w']) > 1e-9: new[s].setdefault(L, []).append(dict(k, prims=list(k['edges']), changed=False)); continue
                pg = prev_geo[q].get(L)

                def vacant(a, b, w=w, L=L, pg=pg):
                    cu = LineString([a, b]).buffer(w / 2, 16)
                    if pg is not None and not pg.is_empty and cu.distance(pg) < pclr.get(L, environment.fwd.drc + environment.fwd.res): return False
                    return environment.verify(cu, L) is not None
                pr, al = straighten(k, vacant, o['lat_max'], o['chord'])
                prior[s] += al
                new[s].setdefault(L, []).append(dict(k, prims=pr, changed=bool(al)))
    Lg = {s: sum(total_length(k['prims']) for ks in new[s].values() for k in ks) for s in ('P', 'N')}
    L0 = {s: sum(total_length(k['edges']) for ks in sides[s].values() for k in ks) for s in ('P', 'N')}
    short = 'P' if Lg['P'] < Lg['N'] else 'N'; long = 'N' if short == 'P' else 'P'
    E = abs(Lg['P'] - Lg['N'])
    info = dict(L_P_before=round(L0['P'], 4), L_N_before=round(L0['N'], 4), skew_before=round(L0['P'] - L0['N'], 4),
                L_P_straight=round(Lg['P'], 4), L_N_straight=round(Lg['N'], 4), E=round(E, 4), short=pair[short]['net_name'],
                straight_drawn=dict(P=prior['P'], N=prior['N']), sites=[], **info0)

    ref = [tuple(q) for x in prior['P'] + prior['N'] for q in (x['places'] or [x['middle']])]
    if not ref:
        for L, ks in new[short].items():
            for k in ks:
                for p, q in zip(k['prims'], k['prims'][1:]):
                    if p[0] == 'seg' and q[0] == 'seg':
                        d1 = math.atan2(p[2][1] - p[1][1], p[2][0] - p[1][0]); d2 = math.atan2(q[2][1] - q[1][1], q[2][0] - q[1][0])
                        if abs((d2 - d1 + math.pi) % (2 * math.pi) - math.pi) > 0.1: ref.append(tuple(p[2]))
    rest = E
    partner_geo = {L: unary_union([LineString(line(k['prims'])).buffer(k['w'][0] / 2, 16) for k in ks]) for L, ks in new[long].items()}
    for _ in range(o['sites']):
        if rest <= o['emin']: break
        chosen = None
        for share, rmin in ((1.0, o['rmin']), (0.5, o['rmin']), (1 / 3, o['rmin']), (1.0, o['rmin_emergency'])):
            if rmin < o['rmin'] and rest <= o.get('tolerance', 0.0): break
            o_ = dict(o, rmin=rmin)
            e = rest * share
            for L, ks in sorted(new[short].items()):
                for ai, k in enumerate(ks):
                    w = k['w'][0]
                    if max(k['w']) - min(k['w']) > 1e-9: continue
                    for si, p in enumerate(k['prims']):
                        if p[0] != 'seg': continue
                        own_set = [LineString(line([q])).buffer(w / 2, 16) for j, q in enumerate(k['prims']) if abs(j - si) > 1]
                        own_set += [LineString(line(kk['prims'])).buffer(kk['w'][0] / 2, 16) for kj, kk in enumerate(ks) if kj != ai]
                        c = crests_candidate((p[1], p[2]), e, w, L, environment, partner_geo.get(L), unary_union(own_set) if own_set else None, ref, o_,
                                            pclr=pclr.get(L))
                        if c and (chosen is None or c['rank'] < chosen[0]['rank']): chosen = (c, L, ai, si)
            if chosen: break
        if not chosen: break
        c, L, ai, si = chosen; k = new[short][L][ai]; p = k['prims'][si]
        en = (c['start'][0] + c['u'][0] * c['F'], c['start'][1] + c['u'][1] * c['F'])
        fallback = [('seg', p[1], c['start'])] + c['prims'] + [('seg', en, p[2])]
        k['prims'] = k['prims'][:si] + fallback + k['prims'][si + 1:]; k['changed'] = True
        rest -= c['more']
        info['sites'].append({x: c[x] for x in ('rank', 'n', 's', 'r', 'A', 'F', 'spacing_foreign', 'more', 'start')} |
                               dict(layer_name=L, from_=list(p[1]), until=list(p[2])))
    Ln = {s: sum(total_length(k['prims']) for ks in new[s].values() for k in ks) for s in ('P', 'N')}
    info.update(L_P_new=round(Ln['P'], 4), L_N_new=round(Ln['N'], 4), skew_new=round(Ln['P'] - Ln['N'], 4),
                rest=round(max(0.0, rest), 4), micro=micro_pieces(new))

    geo = {s: {L: unary_union([LineString(line(k['prims'])).buffer(k['w'][0] / 2, 16) for k in ks]) for L, ks in new[s].items()} for s in ('P', 'N')}
    for L in sorted(set(geo['P']) & set(geo['N'])):
        if geo['P'][L].distance(geo['N'][L]) < pclr.get(L, environment.fwd.drc + environment.fwd.res) - 1e-6:
            info['faults'] = 'P and N too close on %s' % L
            return dict(info=info, new={}, geo={})
    off = {}
    for s in ('P', 'N'):
        for L, ks in new[s].items():
            for k in ks:
                if k['changed']: off.setdefault(pair[s]['net_name'], {}).setdefault(L, []).append(dict(prims=k['prims'], w=k['w'][0], uuids=k['uuids']))
    return dict(info=info, new=off, geo={pair[s]['net_name']: geo[s] for s in ('P', 'N')})


def micro_pieces(new):
    return sum(1 for ks in new.values() for nc in ks.values() for k in nc for p in k['prims'] if p[0] == 'seg' and 1e-6 < primary_length(p) < MICRO)




def crests_shapes(extra, L, w, clearance, n_max=None):
    out = []
    for small, rmin in ((0, STANDARD['rmin']), (1, STANDARD['rmin_emergency'])):
        for n in range(1, (n_max or STANDARD['n_max']) + 1):
            for s in STANDARD['s_values']:
                if s < max(2 * w, w + clearance) - 1e-9: continue
                for rf in STANDARD['r_shares']:
                    f = shaping(extra, n, s, s * rf, rmin, STANDARD['amax'])
                    if f is None: continue
                    r, A = f; F = meander_local(n, s, r, A)[1]
                    if F > L - 2 * MICRO: continue
                    k = (small, n, round(abs(r / s - 0.5), 4), round(abs(s - 0.6), 4), A, s, r, F)
                    if k not in out: out.append(k)
    out.sort()
    return [(n, s, r, A, F) for _, n, _, _, A, s, r, F in out[:12]]


def dome_track(chain, ai, n, s, r, A, lead, side, w, chord=0.0002):
    a, b = chain[ai], chain[ai + 1]; L = dist(a, b)
    e = ((b[0] - a[0]) / L, (b[1] - a[1]) / L); v = (-e[1] * side, e[0] * side)
    pr, F = meander_local(n, s, r, A)
    st = (a[0] + e[0] * lead, a[1] + e[1] * lead)
    T = lambda q: [round(st[0] + q[0] * e[0] + q[1] * v[0], 6), round(st[1] + q[0] * e[1] + q[1] * v[1], 6)]
    prims = [('seg', list(p), list(q)) for p, q in zip(chain[:ai], chain[1:ai + 1])]
    prims += [('seg', list(a), T((0, 0)))] + [(p[0],) + tuple(T(q) for q in p[1:]) for p in pr] + [('seg', T((F, 0)), list(b))]
    prims += [('seg', list(p), list(q)) for p, q in zip(chain[ai + 1:], chain[ai + 2:])]
    items = []; pts = [list(prims[0][1])]
    for p in prims:
        if p[0] == 'seg':
            if dist(p[1], p[2]) < 1e-9: continue
            items.append(['L', list(p[1]), list(p[2]), w]); pts.append(list(p[2]))
        else:
            items.append(['A', list(p[1]), list(p[2]), list(p[3]), w])
            pts += [[round(x, 6), round(y, 6)] for x, y in arc_points(p[1], p[2], p[3], chord)[1:]]
    pts[0] = list(chain[0]); pts[-1] = list(chain[-1])
    return pts, items
