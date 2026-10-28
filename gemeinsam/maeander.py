""
import math
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

K = 8 - 2 * math.pi
TOL = 0.002
MIKRO = 0.05
STANDARD = dict(stufen=(0.30, 0.25, 0.20), s_werte=(0.6, 0.5, 0.4, 0.35, 0.3, 0.28), r_anteile=(0.5, 0.4, 0.3), n_max=6, amax=1.0,
                rmin=0.07, rmin_notfall=0.05, lmin=0.6, rand_t=0.15, schritt_t=0.05, lat_max=1.0, nah_max=4.0, stellen=3, emin=0.001, sehne=0.004)


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def oktal(a, b, tol=0.05):
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
    return min(ang, 45 - ang) < tol




def bogen_geometrie(a, m, b):
    ""
    ux, uy = m[0] - a[0], m[1] - a[1]; vx, vy = b[0] - a[0], b[1] - a[1]
    det = 2 * (ux * vy - uy * vx)
    if abs(det) < 1e-14: raise ValueError('degenerierter Bogen')
    uu, vv = ux * ux + uy * uy, vx * vx + vy * vy
    c = (a[0] + (uu * vy - vv * uy) / det, a[1] + (ux * vv - vx * uu) / det)
    w = [math.atan2(p[1] - c[1], p[0] - c[0]) for p in (a, m, b)]
    ccw = (w[2] - w[0]) % (2 * math.pi)
    sweep = ccw if (w[1] - w[0]) % (2 * math.pi) <= ccw + 1e-9 else ccw - 2 * math.pi
    return c, dist(a, c), w[0], sweep


def bogen_laenge(a, m, b):
    _, r, _, sw = bogen_geometrie(a, m, b)
    return r * abs(sw)


def bogen_punkte(a, m, b, sehne=0.004):
    ""
    c, r, w0, sw = bogen_geometrie(a, m, b)
    n = max(2, int(math.ceil(abs(sw) / (2 * math.acos(max(-1.0, 1 - sehne / max(r, 1e-9)))))) if r > sehne else 2)
    pts = [tuple(a)] + [(c[0] + r * math.cos(w0 + sw * k / n), c[1] + r * math.sin(w0 + sw * k / n)) for k in range(1, n)] + [tuple(b)]
    return pts


def prim_laenge(p):
    return dist(p[1], p[2]) if p[0] == 'seg' else bogen_laenge(p[1], p[2], p[3])


def prim_punkte(p, sehne=0.004):
    return [tuple(p[1]), tuple(p[2])] if p[0] == 'seg' else bogen_punkte(p[1], p[2], p[3], sehne)


def linie(prims, sehne=0.004):
    pts = []
    for p in prims:
        q = prim_punkte(p, sehne)
        if pts and dist(pts[-1], q[0]) < 1e-9: q = q[1:]
        pts += q
    return pts




def kuppe(x0, s, r, A):
    ""
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

def maeander_lokal(n, s, r, A):
    ""
    gap = s - 2 * r; pr = []; x = 0.0
    for k in range(n):
        p, x = kuppe(x, s, r, A); pr += p
        if k < n - 1:
            if gap > 1e-6: pr.append(('seg', (x, 0.0), (x + gap, 0.0)))
            x += gap
    return pr, x


def mehrlaenge(n, s, r, A):
    return n * (2 * A - K * r)


def gerade_ok(l):
    return l < 1e-6 or l >= MIKRO - 1e-9


def form(e, n, s, r, rmin=0.05, amax=1.0):
    ""
    A = (e / n + K * r) / 2
    if A < 2 * r:
        r = e / n / (4 - K); A = 2 * r
        if 2 * r > s + 1e-9: return None
    if r < rmin - 1e-9 or A > amax + 1e-9: return None
    if not (gerade_ok(A - 2 * r) and gerade_ok(s - 2 * r)): return None
    return r, A


def welt(pr, a, u, v):
    T = lambda q: (a[0] + q[0] * u[0] + q[1] * v[0], a[1] + q[0] * u[1] + q[1] * v[1])
    return [(p[0],) + tuple(T(q) for q in p[1:]) for p in pr]




class Knoten:
    def __init__(self, tol=TOL):
        self.p = []; self.tol = tol

    def __call__(self, q):
        for i, p in enumerate(self.p):
            if dist(p, q) < self.tol: return i
        self.p.append((float(q[0]), float(q[1]))); return len(self.p) - 1


def ketten(kanten, fest=()):
    ""
    kn = Knoten()
    for q in fest: kn(q)
    padk = set(range(len(kn.p)))
    E = []
    for k in kanten:
        i, j = kn(k['a']), kn(k['b'])
        if i == j and k['typ'] == 'seg': continue
        E.append((i, j, k))
    nb = {}
    for ei, (i, j, k) in enumerate(E):
        nb.setdefault(i, []).append(ei); nb.setdefault(j, []).append(ei)
    festk = {v for v in nb if len(nb[v]) != 2 or v in padk}
    benutzt = set(); res = []
    for s0 in sorted(festk):
        for e0 in nb[s0]:
            if e0 in benutzt: continue
            knoten = [s0]; kl = []; v = s0; e = e0
            while True:
                benutzt.add(e); i, j, k = E[e]
                w_ = j if i == v else i
                if k['typ'] == 'seg': kl.append((('seg', kn.p[v], kn.p[w_]), k))
                else:
                    m = tuple(k['m'])
                    kl.append((('arc', kn.p[v], m, kn.p[w_]), k))
                knoten.append(w_); v = w_
                if v in festk: break
                nx = [x for x in nb[v] if x not in benutzt]
                if not nx: break
                e = nx[0]
            res.append(dict(knoten=[kn.p[x] for x in knoten], kanten=[p for p, _ in kl], w=[k['w'] for _, k in kl],
                            uuids=sorted({k['uuid'] for _, k in kl if k.get('uuid')})))
    return res


def laenge(prims):
    return sum(prim_laenge(p) for p in prims)


def geradeziehen(kette, frei, lat_max=1.0, sehne=0.004):
    ""
    pr = kette['kanten']; nk = len(pr)
    pts0 = [tuple(p[1]) for p in pr] + [tuple(pr[-1][-1])]
    out = []; alt = []; i = 0
    while i < nk:
        best = None
        for j in range(nk, i + 1, -1):
            a, b = pts0[i], pts0[j]
            if dist(a, b) < 1e-6 or not oktal(a, b): continue
            L = dist(a, b); ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            zw = [q for p in pr[i:j] for q in prim_punkte(p, sehne)[1:]][:-1]
            lat = [-(q[0] - a[0]) * uy + (q[1] - a[1]) * ux for q in zw]
            proj = [(q[0] - a[0]) * ux + (q[1] - a[1]) * uy for q in zw]
            if not lat or max(abs(x) for x in lat) > lat_max or min(proj) < -0.01 or max(proj) > L + 0.01: continue
            if min(lat) < -1e-4 and max(lat) > 1e-4: continue
            weg = laenge(pr[i:j])
            if weg - L < 0.005: continue
            if not frei(a, b): continue
            best = j; orte = [q for q, x in zip(zw, lat) if abs(x) > 0.02]; break
        if best is None:
            out.append(pr[i]); i += 1
        else:
            a, b = pts0[i], pts0[best]
            alt.append(dict(von=list(a), bis=list(b), mehr=round(laenge(pr[i:best]) - dist(a, b), 4), mitte=[(a[0] + b[0]) / 2, (a[1] + b[1]) / 2],
                            boegen=sum(1 for p in pr[i:best] if p[0] == 'arc'), orte=[list(q) for q in orte[::max(1, len(orte) // 12)]]))
            out.append(('seg', a, b)); i = best

    red = []
    for p in out:
        if red and p[0] == 'seg' and red[-1][0] == 'seg':
            a, m, b = red[-1][1], red[-1][2], p[2]
            cr = (m[0] - a[0]) * (b[1] - m[1]) - (m[1] - a[1]) * (b[0] - m[0])
            if abs(cr) < 1e-6 and ((m[0] - a[0]) * (b[0] - m[0]) + (m[1] - a[1]) * (b[1] - m[1])) > 0:
                red[-1] = ('seg', a, b); continue
        red.append(p)
    return red, alt




class Umgebung:
    ""

    def __init__(self, hin, netze, ersetzt=None):
        self.hin = hin; self.netze = set(netze); self.ersetzt = dict(ersetzt or {})
        self.eigen = max([hin.netzabstand.get(n, 0.0) for n in self.netze] + [0.0])

    def pruefen(self, cu, layer, reserve=None):
        hin = self.hin; res = hin.res if reserve is None else reserve
        best = 1e9
        if layer in hin.baum:
            for i in hin.baum[layer].query(cu.buffer(hin.dmax + 0.35 + res)):
                n, g, d, bahn = hin.obj[layer][int(i)]
                if n in self.netze or n in self.ersetzt: continue
                dd = max(d, self.eigen if bahn else 0.0, hin.netzabstand.get(n, 0.0) if bahn else 0.0)
                x = cu.distance(g)
                if x < dd + res: return None
                if not n.startswith('#'): best = min(best, x)
        for n, geo in sorted(self.ersetzt.items()):
            g = geo.get(layer)
            if n in self.netze or g is None or g.is_empty: continue
            x = cu.distance(g)
            if x < max(hin.drc, self.eigen) + res: return None
            best = min(best, x)
        return best


def prims_kupfer(prims, w, sehne=0.004):
    return unary_union([LineString(prim_punkte(p, sehne)).buffer(w / 2, 16) for p in prims])




def _stufe(x, stufen):
    for k, s in enumerate(stufen):
        if x >= s - 1e-9: return k
    return None


def kuppen_kandidat(seg, e, w, layer, umg, partner_geo, eigen_geo, ref, opt, n_liste=None, pclr=None):
    ""
    a, b = seg; L = dist(a, b); o = opt
    if L < o['lmin']: return None
    u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
    best = None
    for sgn in (1, -1):
        v = (-u[1] * sgn, u[0] * sgn)
        mitte0 = (a[0] + u[0] * L / 2, a[1] + u[1] * L / 2); probe = (mitte0[0] + v[0] * 0.2, mitte0[1] + v[1] * 0.2)
        if partner_geo is not None and not partner_geo.is_empty and Point(probe).distance(partner_geo) < Point(mitte0).distance(partner_geo):
            continue
        for n in (n_liste or range(1, o['n_max'] + 1)):
            if best and best['rang'][:2] == (0, 0) and best['n'] < n: break
            for s_ in o['s_werte']:
                for rf in o['r_anteile']:
                    f = form(e, n, s_, s_ * rf, o['rmin'], o['amax'])
                    if f is None: continue
                    r, A = f
                    pr, F = maeander_lokal(n, s_, r, A)
                    if L - F < 2 * o['rand_t'] - 1e-9: continue

                    cu0 = LineString(linie(pr, o['sehne'])).buffer(w / 2, 16)
                    ts = [o['rand_t'] + o['schritt_t'] * k for k in range(int((L - F - 2 * o['rand_t']) / o['schritt_t'] + 1e-9) + 1)]
                    if ref:
                        ts.sort(key=lambda t: (min(dist((a[0] + u[0] * (t + F / 2), a[1] + u[1] * (t + F / 2)), q) for q in ref), t))
                    for t in ts:
                        st = (a[0] + u[0] * t, a[1] + u[1] * t)
                        mitte = (st[0] + u[0] * F / 2, st[1] + u[1] * F / 2)
                        nah = min((dist(mitte, q) for q in ref), default=0.0)
                        fern = 1 if nah > o['nah_max'] else 0
                        rang_rest = (n, round(abs(r / s_ - 0.5), 4), round(abs(s_ - 0.6), 4), round(nah, 1))
                        if best and (fern, 0) + rang_rest >= best['rang']: continue
                        M = (u[0], v[0], u[1], v[1], st[0], st[1])
                        cu = affinity.affine_transform(cu0, M)
                        if partner_geo is not None and not partner_geo.is_empty and \
                                cu.distance(partner_geo) < (umg.hin.drc + umg.hin.res if pclr is None else pclr): continue
                        if eigen_geo is not None and not eigen_geo.is_empty and cu.distance(eigen_geo) < max(umg.hin.drc, w): continue
                        dfr = umg.pruefen(cu, layer)
                        if dfr is None: continue
                        stufe = _stufe(dfr, o['stufen'])
                        if stufe is None: continue
                        rang = (fern, stufe) + rang_rest
                        if best is None or rang < best['rang']:
                            best = dict(rang=rang, n=n, s=s_, r=round(r, 6), A=round(A, 6), F=round(F, 6), t=round(t, 4), start=st, u=u, v=v,
                                        prims=welt(pr, st, u, v), abstand_fremd=round(dfr, 3), mehr=round(mehrlaenge(n, s_, r, A), 6), seite=sgn)
    return best

def paar_planen(paar, umg, opt=None):
    ""
    o = dict(STANDARD, **(opt or {}))
    seiten = {}
    for s in ('P', 'N'):
        seiten[s] = {L: ketten(d['kanten'], d.get('fest', ())) for L, d in sorted(paar[s]['lagen'].items())}

    alt_geo = {s: {L: unary_union([LineString(linie(k['kanten'])).buffer(k['w'][0] / 2, 16) for k in ks]) if ks else None
                   for L, ks in seiten[s].items()} for s in ('P', 'N')}

    pclr = {}
    for L in sorted(set(alt_geo['P']) & set(alt_geo['N'])):
        gp, gn = alt_geo['P'][L], alt_geo['N'][L]
        if gp is not None and gn is not None and not gp.is_empty and not gn.is_empty:
            pclr[L] = min(umg.hin.drc + umg.hin.res, gp.distance(gn)) - 1e-4
    info0 = dict(paarabstand={L: round(v + 1e-4, 4) for L, v in pclr.items()})
    neu = {'P': {}, 'N': {}}; alt = {'P': [], 'N': []}
    for s in ('P', 'N'):
        q = 'N' if s == 'P' else 'P'
        for L, ks in seiten[s].items():
            for k in ks:
                w = k['w'][0]
                if max(k['w']) - min(k['w']) > 1e-9: neu[s].setdefault(L, []).append(dict(k, prims=list(k['kanten']), geaendert=False)); continue
                pg = alt_geo[q].get(L)

                def frei(a, b, w=w, L=L, pg=pg):
                    cu = LineString([a, b]).buffer(w / 2, 16)
                    if pg is not None and not pg.is_empty and cu.distance(pg) < pclr.get(L, umg.hin.drc + umg.hin.res): return False
                    return umg.pruefen(cu, L) is not None
                pr, al = geradeziehen(k, frei, o['lat_max'], o['sehne'])
                alt[s] += al
                neu[s].setdefault(L, []).append(dict(k, prims=pr, geaendert=bool(al)))
    Lg = {s: sum(laenge(k['prims']) for ks in neu[s].values() for k in ks) for s in ('P', 'N')}
    L0 = {s: sum(laenge(k['kanten']) for ks in seiten[s].values() for k in ks) for s in ('P', 'N')}
    kurz = 'P' if Lg['P'] < Lg['N'] else 'N'; lang = 'N' if kurz == 'P' else 'P'
    E = abs(Lg['P'] - Lg['N'])
    info = dict(L_P_vorher=round(L0['P'], 4), L_N_vorher=round(L0['N'], 4), skew_vorher=round(L0['P'] - L0['N'], 4),
                L_P_gerade=round(Lg['P'], 4), L_N_gerade=round(Lg['N'], 4), E=round(E, 4), kurz=paar[kurz]['netz'],
                gerade_gezogen=dict(P=alt['P'], N=alt['N']), stellen=[], **info0)

    ref = [tuple(q) for x in alt['P'] + alt['N'] for q in (x['orte'] or [x['mitte']])]
    if not ref:
        for L, ks in neu[kurz].items():
            for k in ks:
                for p, q in zip(k['prims'], k['prims'][1:]):
                    if p[0] == 'seg' and q[0] == 'seg':
                        d1 = math.atan2(p[2][1] - p[1][1], p[2][0] - p[1][0]); d2 = math.atan2(q[2][1] - q[1][1], q[2][0] - q[1][0])
                        if abs((d2 - d1 + math.pi) % (2 * math.pi) - math.pi) > 0.1: ref.append(tuple(p[2]))
    rest = E
    partner_geo = {L: unary_union([LineString(linie(k['prims'])).buffer(k['w'][0] / 2, 16) for k in ks]) for L, ks in neu[lang].items()}
    for _ in range(o['stellen']):
        if rest <= o['emin']: break
        gewaehlt = None
        for anteil, rmin in ((1.0, o['rmin']), (0.5, o['rmin']), (1 / 3, o['rmin']), (1.0, o['rmin_notfall'])):
            if rmin < o['rmin'] and rest <= o.get('toleranz', 0.0): break
            o_ = dict(o, rmin=rmin)
            e = rest * anteil
            for L, ks in sorted(neu[kurz].items()):
                for ki, k in enumerate(ks):
                    w = k['w'][0]
                    if max(k['w']) - min(k['w']) > 1e-9: continue
                    for si, p in enumerate(k['prims']):
                        if p[0] != 'seg': continue
                        eig = [LineString(linie([q])).buffer(w / 2, 16) for j, q in enumerate(k['prims']) if abs(j - si) > 1]
                        eig += [LineString(linie(kk['prims'])).buffer(kk['w'][0] / 2, 16) for kj, kk in enumerate(ks) if kj != ki]
                        c = kuppen_kandidat((p[1], p[2]), e, w, L, umg, partner_geo.get(L), unary_union(eig) if eig else None, ref, o_,
                                            pclr=pclr.get(L))
                        if c and (gewaehlt is None or c['rang'] < gewaehlt[0]['rang']): gewaehlt = (c, L, ki, si)
            if gewaehlt: break
        if not gewaehlt: break
        c, L, ki, si = gewaehlt; k = neu[kurz][L][ki]; p = k['prims'][si]
        en = (c['start'][0] + c['u'][0] * c['F'], c['start'][1] + c['u'][1] * c['F'])
        ersatz = [('seg', p[1], c['start'])] + c['prims'] + [('seg', en, p[2])]
        k['prims'] = k['prims'][:si] + ersatz + k['prims'][si + 1:]; k['geaendert'] = True
        rest -= c['mehr']
        info['stellen'].append({x: c[x] for x in ('rang', 'n', 's', 'r', 'A', 'F', 'abstand_fremd', 'mehr', 'start')} |
                               dict(lage=L, von=list(p[1]), bis=list(p[2])))
    Ln = {s: sum(laenge(k['prims']) for ks in neu[s].values() for k in ks) for s in ('P', 'N')}
    info.update(L_P_neu=round(Ln['P'], 4), L_N_neu=round(Ln['N'], 4), skew_neu=round(Ln['P'] - Ln['N'], 4),
                rest=round(max(0.0, rest), 4), mikro=mikro_stuecke(neu))

    geo = {s: {L: unary_union([LineString(linie(k['prims'])).buffer(k['w'][0] / 2, 16) for k in ks]) for L, ks in neu[s].items()} for s in ('P', 'N')}
    for L in sorted(set(geo['P']) & set(geo['N'])):
        if geo['P'][L].distance(geo['N'][L]) < pclr.get(L, umg.hin.drc + umg.hin.res) - 1e-6:
            info['fehler'] = 'P und N zu nah auf %s' % L
            return dict(info=info, neu={}, geo={})
    aus = {}
    for s in ('P', 'N'):
        for L, ks in neu[s].items():
            for k in ks:
                if k['geaendert']: aus.setdefault(paar[s]['netz'], {}).setdefault(L, []).append(dict(prims=k['prims'], w=k['w'][0], uuids=k['uuids']))
    return dict(info=info, neu=aus, geo={paar[s]['netz']: geo[s] for s in ('P', 'N')})


def mikro_stuecke(neu):
    return sum(1 for ks in neu.values() for kl in ks.values() for k in kl for p in k['prims'] if p[0] == 'seg' and 1e-6 < prim_laenge(p) < MIKRO)




def kuppen_formen(extra, L, w, clearance, n_max=None):
    ""
    out = []
    for klein, rmin in ((0, STANDARD['rmin']), (1, STANDARD['rmin_notfall'])):
        for n in range(1, (n_max or STANDARD['n_max']) + 1):
            for s in STANDARD['s_werte']:
                if s < max(2 * w, w + clearance) - 1e-9: continue
                for rf in STANDARD['r_anteile']:
                    f = form(extra, n, s, s * rf, rmin, STANDARD['amax'])
                    if f is None: continue
                    r, A = f; F = maeander_lokal(n, s, r, A)[1]
                    if F > L - 2 * MIKRO: continue
                    k = (klein, n, round(abs(r / s - 0.5), 4), round(abs(s - 0.6), 4), A, s, r, F)
                    if k not in out: out.append(k)
    out.sort()
    return [(n, s, r, A, F) for _, n, _, _, A, s, r, F in out[:12]]


def kuppe_bahn(chain, ki, n, s, r, A, lead, side, w, sehne=0.0002):
    ""
    a, b = chain[ki], chain[ki + 1]; L = dist(a, b)
    e = ((b[0] - a[0]) / L, (b[1] - a[1]) / L); v = (-e[1] * side, e[0] * side)
    pr, F = maeander_lokal(n, s, r, A)
    st = (a[0] + e[0] * lead, a[1] + e[1] * lead)
    T = lambda q: [round(st[0] + q[0] * e[0] + q[1] * v[0], 6), round(st[1] + q[0] * e[1] + q[1] * v[1], 6)]
    prims = [('seg', list(p), list(q)) for p, q in zip(chain[:ki], chain[1:ki + 1])]
    prims += [('seg', list(a), T((0, 0)))] + [(p[0],) + tuple(T(q) for q in p[1:]) for p in pr] + [('seg', T((F, 0)), list(b))]
    prims += [('seg', list(p), list(q)) for p, q in zip(chain[ki + 1:], chain[ki + 2:])]
    items = []; pts = [list(prims[0][1])]
    for p in prims:
        if p[0] == 'seg':
            if dist(p[1], p[2]) < 1e-9: continue
            items.append(['L', list(p[1]), list(p[2]), w]); pts.append(list(p[2]))
        else:
            items.append(['A', list(p[1]), list(p[2]), list(p[3]), w])
            pts += [[round(x, 6), round(y, 6)] for x, y in bogen_punkte(p[1], p[2], p[3], sehne)[1:]]
    pts[0] = list(chain[0]); pts[-1] = list(chain[-1])
    return pts, items
