import json, math, os, sys, collections

ANGLE = 20.0
AGGRESSOR = {'sw': (2.0, 0.8), 'fast': (1.0, 0.5)}


def sets(K):
    N = K['net_list']
    victim = {n for n, k in N.items() if (k.get('category') == 'analog' and k.get('rank', 0) >= 2) or k.get('kelvin') or k.get('sensitive_sw')}
    aggressor = {}
    from analysis.measures import review_active
    if not review_active(K):
        for n, k in N.items():
            a = k.get('category')
            if a == 'sw': aggressor[n] = AGGRESSOR['sw']
            elif a == 'fast' and k.get('rank', 0) >= 3: aggressor[n] = AGGRESSOR['fast']
        return victim, aggressor
    from analysis.aggressor import read_in
    S = read_in(K); victim = set(S['victim'])
    for n in S['sw'] + S['current_val']: aggressor[n] = AGGRESSOR['sw']
    for n in S['digital']: aggressor[n] = (1.0, 0.5 * S['net_list'][n]['spacing_factor'])
    return victim, aggressor


def _segments(R, only):
    out = collections.defaultdict(list)
    for t in R['tracks']:
        if t['net'] not in only and not (t.get('kelvin') and '*kelvin' in only): continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-6: out[t['layer']].append((t['net'], a, b, w, bool(t.get('kelvin'))))
    return out


def _pair(a, b, c, d):
    ux, uy = b[0] - a[0], b[1] - a[1]; L = math.hypot(ux, uy); ux /= L; uy /= L
    vx, vy = d[0] - c[0], d[1] - c[1]; M = math.hypot(vx, vy)
    if abs(ux * vy - uy * vx) / M > math.sin(math.radians(ANGLE)): return None
    s1 = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy; s2 = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
    lo, hi = max(0.0, min(s1, s2)), min(L, max(s1, s2))
    if hi - lo <= 1e-6: return None
    q1 = (c[0] - a[0]) * -uy + (c[1] - a[1]) * ux; q2 = (d[0] - a[0]) * -uy + (d[1] - a[1]) * ux
    def cross(s): return q1 + (q2 - q1) * ((s - s1) / (s2 - s1)) if abs(s2 - s1) > 1e-9 else q1
    dist = min(abs(cross(lo)), abs(cross(hi)))
    return hi - lo, dist, (a[0] + ux * (lo + hi) / 2, a[1] + uy * (lo + hi) / 2)


def coupling(R, K, victim=None, aggressor=None):
    o0, s0 = sets(K)
    victim = o0 if victim is None else victim
    aggressor = s0 if aggressor is None else aggressor
    so = _segments(R, set(victim) | {'*kelvin'})
    ss = _segments(R, set(aggressor))
    total_sum = 0.0; per = collections.defaultdict(float); sites = []
    for L in sorted(so):
        st = ss.get(L, [])
        if not st: continue
        for n, a, b, w, _k in so[L]:
            for m, c, d, w2, _ in st:
                if m == n: continue
                wt, gap_width = aggressor[m]
                if min(c[0], d[0]) > max(a[0], b[0]) + gap_width + 1 or max(c[0], d[0]) < min(a[0], b[0]) - gap_width - 1: continue
                if min(c[1], d[1]) > max(a[1], b[1]) + gap_width + 1 or max(c[1], d[1]) < min(a[1], b[1]) - gap_width - 1: continue
                p = _pair(a, b, c, d)
                if p is None: continue
                long, middle, xy = p
                edge = middle - w / 2 - w2 / 2
                if edge >= gap_width: continue
                amount = long * wt * min(4.0, gap_width / max(edge, gap_width / 4))
                total_sum += amount; per[n] += amount
                sites.append((round(amount, 3), n, m, L, round(xy[0], 3), round(xy[1], 3), round(long, 3), round(edge, 3)))
    sites.sort(reverse=True)
    return dict(total_sum=round(total_sum, 2), per_victim={k: round(v, 2) for k, v in sorted(per.items(), key=lambda x: -x[1])}, sites=sites)


D_MAX = 3.0


def johnson_k(D, H):
    return 1.0 / (1.0 + (D / H) ** 2)


def _pair_net(a, b, N):
    if (N.get(a) or {}).get('pair') == b or (N.get(b) or {}).get('pair') == a: return True
    for p, q in (('DP', 'DN'), ('_P', '_N'), ('+', '-')):
        if (a.endswith(p) and b == a[:-len(p)] + q) or (a.endswith(q) and b == a[:-len(q)] + p): return True
    return False


def johnson(R, K, A, tr_ns, GND=()):
    import shapely
    N = K['net_list']; GND = set(GND); LG = A['cu_layers']
    seg = []
    for t in R['tracks']:
        g = LG.get(t['layer'])
        if t['net'] in GND or g is None or g['type_'] == 'plane': continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-6: seg.append((t['net'], t['layer'], a, b, w))
    acc = collections.defaultdict(lambda: [0.0, 0.0, 0.0])
    if seg:
        geo = shapely.linestrings([[s[2], s[3]] for s in seg])
        I, J = shapely.STRtree(geo).query(geo, predicate='dwithin', distance=D_MAX)
        from analysis.measures import line
        pn = {}; tp = {}
        for i, j in zip(I.tolist(), J.tolist()):
            if j <= i: continue
            n1, L1, a, b, w1 = seg[i]; n2, L2, c, d, w2 = seg[j]
            if n1 == n2: continue
            if (n1, n2) not in pn: pn[(n1, n2)] = _pair_net(n1, n2, N)
            if pn[(n1, n2)]: continue
            g1, g2 = LG[L1], LG[L2]
            if L1 != L2:
                if abs(g1['i'] - g2['i']) != 1: continue
            if g1['H'] is None or g2['H'] is None: continue
            p = _pair(a, b, c, d)
            if p is None: continue
            long, cross, _xy = p
            if L1 == L2: D2, H2 = cross ** 2, g1['H'] ** 2
            else: D2, H2 = cross ** 2 + (g1['z'] - g2['z']) ** 2, g1['H'] * g2['H']
            if D2 > D_MAX ** 2: continue
            k = 1.0 / (1.0 + D2 / H2)
            if (L1, w1) not in tp: tp[(L1, w1)] = line(A, L1, w1)[1]
            tpd = tp[(L1, w1)]
            e = acc[(n1, n2) if n1 < n2 else (n2, n1)]; e[0] += k * long; e[1] += long; e[2] += tpd * long
    victim = collections.defaultdict(float); disturb = collections.defaultdict(float); pairs = []
    for (n1, n2), (S, Lp, T) in sorted(acc.items()):
        if Lp <= 0: continue
        for o, s in ((n1, n2), (n2, n1)):
            tr = tr_ns.get(s)
            if not tr: continue
            xt = (S / Lp) * min(1.0, 2 * T / (1000 * tr))
            victim[o] += xt; disturb[s] += xt; pairs.append((round(xt, 5), o, s, round(Lp, 2)))
    pairs.sort(reverse=True)
    return dict(victim=dict(sorted(victim.items())), disturb=dict(sorted(disturb.items())), pairs=pairs)


def signal_coupling(B, R, K, A, M, tr_ns):
    from analysis.signal_integrity import disturbances
    return disturbances(B, R, K, A, M, tr_ns)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    R = json.load(open(a[0])); K = json.load(open(a[1]))
    E = coupling(R, K)
    if kw.get('json') == '1': print(json.dumps(E)); sys.exit()
    o, s = sets(K)
    print('Coupling %.2f (victim %d nets, aggressor %d nets, %d spots)' % (E['total_sum'], len(o), len(s), len(E['sites'])))
    for n, v in list(E['per_victim'].items())[:int(kw.get('n', '10'))]: print('  %-24s %.2f' % (n.split('/')[-1], v))
    for st in E['sites'][:int(kw.get('n', '10'))]:
        print('  %.2f  %s next to %s on %s at (%.2f, %.2f): %.2f mm long, edge %.3f mm' % (st[0], st[1].split('/')[-1], st[2].split('/')[-1], st[3], st[4], st[5], st[6], st[7]))
