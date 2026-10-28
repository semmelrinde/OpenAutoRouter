""
import json, math, os, sys, collections

WINKEL = 20.0
STOERER = {'sw': (2.0, 0.8), 'schnell': (1.0, 0.5)}


def mengen(K):
    ""
    N = K['netze']
    opfer = {n for n, k in N.items() if (k.get('art') == 'analog' and k.get('rang', 0) >= 2) or k.get('kelvin') or k.get('empfindlich_sw')}
    stoerer = {}
    from analyse.metriken import review_aktiv
    if not review_aktiv(K):
        for n, k in N.items():
            a = k.get('art')
            if a == 'sw': stoerer[n] = STOERER['sw']
            elif a == 'schnell' and k.get('rang', 0) >= 3: stoerer[n] = STOERER['schnell']
        return opfer, stoerer
    from analyse.stoerer import lesen
    S = lesen(K); opfer = set(S['opfer'])
    for n in S['sw'] + S['strom']: stoerer[n] = STOERER['sw']
    for n in S['digital']: stoerer[n] = (1.0, 0.5 * S['netze'][n]['abstand_faktor'])
    return opfer, stoerer


def _segmente(R, nur):
    out = collections.defaultdict(list)
    for t in R['tracks']:
        if t['net'] not in nur and not (t.get('kelvin') and '*kelvin' in nur): continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-6: out[t['layer']].append((t['net'], a, b, w, bool(t.get('kelvin'))))
    return out


def _paar(a, b, c, d):
    ""
    ux, uy = b[0] - a[0], b[1] - a[1]; L = math.hypot(ux, uy); ux /= L; uy /= L
    vx, vy = d[0] - c[0], d[1] - c[1]; M = math.hypot(vx, vy)
    if abs(ux * vy - uy * vx) / M > math.sin(math.radians(WINKEL)): return None
    s1 = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy; s2 = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
    lo, hi = max(0.0, min(s1, s2)), min(L, max(s1, s2))
    if hi - lo <= 1e-6: return None
    q1 = (c[0] - a[0]) * -uy + (c[1] - a[1]) * ux; q2 = (d[0] - a[0]) * -uy + (d[1] - a[1]) * ux
    def quer(s): return q1 + (q2 - q1) * ((s - s1) / (s2 - s1)) if abs(s2 - s1) > 1e-9 else q1
    abst = min(abs(quer(lo)), abs(quer(hi)))
    return hi - lo, abst, (a[0] + ux * (lo + hi) / 2, a[1] + uy * (lo + hi) / 2)


def kopplung(R, K, opfer=None, stoerer=None):
    o0, s0 = mengen(K)
    opfer = o0 if opfer is None else opfer
    stoerer = s0 if stoerer is None else stoerer
    so = _segmente(R, set(opfer) | {'*kelvin'})
    ss = _segmente(R, set(stoerer))
    summe = 0.0; je = collections.defaultdict(float); stellen = []
    for L in sorted(so):
        st = ss.get(L, [])
        if not st: continue
        for n, a, b, w, _k in so[L]:
            for m, c, d, w2, _ in st:
                if m == n: continue
                gew, spalt = stoerer[m]
                if min(c[0], d[0]) > max(a[0], b[0]) + spalt + 1 or max(c[0], d[0]) < min(a[0], b[0]) - spalt - 1: continue
                if min(c[1], d[1]) > max(a[1], b[1]) + spalt + 1 or max(c[1], d[1]) < min(a[1], b[1]) - spalt - 1: continue
                p = _paar(a, b, c, d)
                if p is None: continue
                lang, mitte, xy = p
                kante = mitte - w / 2 - w2 / 2
                if kante >= spalt: continue
                wert = lang * gew * min(4.0, spalt / max(kante, spalt / 4))
                summe += wert; je[n] += wert
                stellen.append((round(wert, 3), n, m, L, round(xy[0], 3), round(xy[1], 3), round(lang, 3), round(kante, 3)))
    stellen.sort(reverse=True)
    return dict(summe=round(summe, 2), je_opfer={k: round(v, 2) for k, v in sorted(je.items(), key=lambda x: -x[1])}, stellen=stellen)


D_MAX = 3.0


def johnson_k(D, H):
    ""
    return 1.0 / (1.0 + (D / H) ** 2)


def _paarnetz(a, b, N):
    if (N.get(a) or {}).get('paar') == b or (N.get(b) or {}).get('paar') == a: return True
    for p, q in (('DP', 'DN'), ('_P', '_N'), ('+', '-')):
        if (a.endswith(p) and b == a[:-len(p)] + q) or (a.endswith(q) and b == a[:-len(q)] + p): return True
    return False


def johnson(R, K, A, tr_ns, GND=()):
    ""
    import shapely
    N = K['netze']; GND = set(GND); LG = A['lagen']
    seg = []
    for t in R['tracks']:
        g = LG.get(t['layer'])
        if t['net'] in GND or g is None or g['typ'] == 'ebene': continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-6: seg.append((t['net'], t['layer'], a, b, w))
    acc = collections.defaultdict(lambda: [0.0, 0.0, 0.0])
    if seg:
        geo = shapely.linestrings([[s[2], s[3]] for s in seg])
        I, J = shapely.STRtree(geo).query(geo, predicate='dwithin', distance=D_MAX)
        from analyse.metriken import leitung
        pn = {}; tp = {}
        for i, j in zip(I.tolist(), J.tolist()):
            if j <= i: continue
            n1, L1, a, b, w1 = seg[i]; n2, L2, c, d, w2 = seg[j]
            if n1 == n2: continue
            if (n1, n2) not in pn: pn[(n1, n2)] = _paarnetz(n1, n2, N)
            if pn[(n1, n2)]: continue
            g1, g2 = LG[L1], LG[L2]
            if L1 != L2:
                if abs(g1['i'] - g2['i']) != 1: continue
            if g1['H'] is None or g2['H'] is None: continue
            p = _paar(a, b, c, d)
            if p is None: continue
            lang, quer, _xy = p
            if L1 == L2: D2, H2 = quer ** 2, g1['H'] ** 2
            else: D2, H2 = quer ** 2 + (g1['z'] - g2['z']) ** 2, g1['H'] * g2['H']
            if D2 > D_MAX ** 2: continue
            k = 1.0 / (1.0 + D2 / H2)
            if (L1, w1) not in tp: tp[(L1, w1)] = leitung(A, L1, w1)[1]
            tpd = tp[(L1, w1)]
            e = acc[(n1, n2) if n1 < n2 else (n2, n1)]; e[0] += k * lang; e[1] += lang; e[2] += tpd * lang
    opfer = collections.defaultdict(float); stoer = collections.defaultdict(float); paare = []
    for (n1, n2), (S, Lp, T) in sorted(acc.items()):
        if Lp <= 0: continue
        for o, s in ((n1, n2), (n2, n1)):
            tr = tr_ns.get(s)
            if not tr: continue
            xt = (S / Lp) * min(1.0, 2 * T / (1000 * tr))
            opfer[o] += xt; stoer[s] += xt; paare.append((round(xt, 5), o, s, round(Lp, 2)))
    paare.sort(reverse=True)
    return dict(opfer=dict(sorted(opfer.items())), stoer=dict(sorted(stoer.items())), paare=paare)


def signal_kopplung(B, R, K, A, M, tr_ns):
    ""
    from analyse.signalintegritaet import stoerungen
    return stoerungen(B, R, K, A, M, tr_ns)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    R = json.load(open(a[0])); K = json.load(open(a[1]))
    E = kopplung(R, K)
    if kw.get('json') == '1': print(json.dumps(E)); sys.exit()
    o, s = mengen(K)
    print('Kopplung %.2f (Opfer %d Netze, Stoerer %d Netze, %d Stellen)' % (E['summe'], len(o), len(s), len(E['stellen'])))
    for n, v in list(E['je_opfer'].items())[:int(kw.get('n', '10'))]: print('  %-24s %.2f' % (n.split('/')[-1], v))
    for st in E['stellen'][:int(kw.get('n', '10'))]:
        print('  %.2f  %s neben %s auf %s bei (%.2f, %.2f): %.2f mm lang, Kante %.3f mm' % (st[0], st[1].split('/')[-1], st[2].split('/')[-1], st[3], st[4], st[5], st[6], st[7]))
