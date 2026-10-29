import math


def pieces(length_list, width_list):
    st = []
    for i, (l, w) in enumerate(zip(length_list, width_list)):
        if l < 1e-9: continue
        if st and abs(st[-1][0] - w) < 1e-6: st[-1][1] += l; st[-1][3] = i
        else: st.append([w, l, i, i])
    return st


def count(length_list, width_list, short=1.0):
    st = pieces(length_list, width_list)
    return max(0, len(st) - 1), sum(1 for a, b, c in zip(st[:-2], st[1:-1], st[2:]) if b[1] < short and b[0] > a[0] + 1e-6 and b[0] > c[0] + 1e-6)


def resistance(length_list, width_list):
    return sum(l / w for l, w in zip(length_list, width_list) if l >= 1e-9)


def _at_goal(w, z):
    return w >= math.floor(z * 40) / 40 - 1e-9


def _intended(wa, za, wb, zb):
    if za is None or zb is None: return False
    if wa > wb: wa, za, wb, zb = wb, zb, wa, za
    return zb > za + 1e-9 and _at_goal(wa, za)


def _profile(l, w, z, bo, lam, mu):
    idx = [i for i in range(len(l)) if l[i] >= 1e-9]
    lev = sorted({round(w[i], 6) for i in idx}, reverse=True)
    INF = float('inf'); cost = []; back = []
    for n, i in enumerate(idx):
        cur = []; zr = []
        for L in lev:
            if L > w[i] + 1e-9 or L < bo[i] - 1e-9: cur.append(INF); zr.append(-1); continue
            c0 = l[i] * (math.log(w[i] / L) + mu * (1.0 / L - 1.0 / w[i]))
            if n == 0: cur.append(c0); zr.append(-1); continue
            ip = idx[n - 1]; best = INF; bj = -1
            for j, Lp in enumerate(lev):
                cp = cost[-1][j]
                if cp == INF: continue
                if abs(Lp - L) > 1e-9 and not _intended(Lp, z[ip], L, z[i]): cp += lam
                if cp < best - 1e-12: best = cp; bj = j
            cur.append(best + c0); zr.append(bj)
        cost.append(cur); back.append(zr)
    k = min(range(len(lev)), key=lambda j: (cost[-1][j], j)); new = list(w)
    for n in range(len(idx) - 1, -1, -1):
        new[idx[n]] = min(w[idx[n]], lev[k]); k = back[n][k]
    return new


def _humps_way(l, w, z, bo, short, limit):
    w = list(w)
    for _ in range(len(w) + 1):
        st = pieces(l, w); cand = []
        for a, b, c in zip(st[:-2], st[1:-1], st[2:]):
            if not (b[1] < short and b[0] > a[0] + 1e-6 and b[0] > c[0] + 1e-6): continue
            if _intended(a[0], z[a[3]], b[0], z[b[2]]) and _intended(c[0], z[c[2]], b[0], z[b[3]]): continue
            v = list(w); goal = max(a[0], c[0])
            for i in range(b[2], b[3] + 1): v[i] = min(v[i], max(goal, bo[i]))
            if v != w and resistance(l, v) <= limit: cand.append((resistance(l, v), b[2], v))
        if not cand: break
        w = min(cand)[2]
    return w


def calm(length_list, width_list, goal=None, floor=0.0, lam=0.35, short=1.0, r_max=0.05):
    l = list(length_list); w = list(width_list)
    if len(pieces(l, w)) < 2: return w
    z = list(goal) if goal is not None else [None] * len(w)
    bo = [min(wi, floor) for wi in w]
    limit = resistance(l, w) * (1.0 + r_max) + 1e-12
    if short > 0: w = _humps_way(l, w, z, bo, short, limit)
    p = _profile(l, w, z, bo, lam, 0.0)
    if resistance(l, p) > limit:
        lo, hi = 0.0, 0.25
        while resistance(l, _profile(l, w, z, bo, lam, hi)) > limit and hi < 1e6: lo = hi; hi *= 4.0
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            if resistance(l, _profile(l, w, z, bo, lam, mid)) > limit: lo = mid
            else: hi = mid
        p = _profile(l, w, z, bo, lam, hi)
        if resistance(l, p) > limit: p = w
    return _humps_way(l, p, z, bo, short, limit) if short > 0 else p


def _self_test():
    def b(profile_data, **kw):
        l = [x[1] for x in profile_data]; w = [x[0] for x in profile_data]
        return [[round(a[0], 3), round(a[1], 2)] for a in pieces(l, calm(l, w, **kw))]

    kv = [(0.2, 1.24), (0.25, 0.35), (0.2, 0.65), (0.25, 0.57), (0.15, 0.8), (0.2, 1.39), (0.25, 0.71)]
    assert b(kv, r_max=1.0) == [[0.2, 2.81], [0.15, 0.8], [0.2, 2.1]], b(kv, r_max=1.0)

    l = [x[1] for x in kv]; w = [x[0] for x in kv]
    assert all(n <= a + 1e-9 for n, a in zip(calm(l, w), w))

    assert b([(0.225, 12.0), (0.2, 2.8)]) == [[0.225, 12.0], [0.2, 2.8]]

    assert b([(0.3, 5.0), (1.0, 0.5), (0.25, 5.0)], r_max=1.0) == [[0.3, 5.5], [0.25, 5.0]]

    assert b([(1.0, 3.0), (0.6, 0.3)], goal=[1.0, 0.6]) == [[1.0, 3.0], [0.6, 0.3]]

    assert b([(0.15, 3.0), (0.3, 0.4), (0.15, 3.0)], floor=0.3, r_max=1.0) == [[0.15, 3.0], [0.3, 0.4], [0.15, 3.0]]

    assert b([(0.3, 6.26), (1.0, 0.85), (0.3, 1.2)], r_max=0.05) == [[0.3, 6.26], [1.0, 0.85], [0.3, 1.2]]
    print('width_profile: self-test ok')


if __name__ == '__main__':
    import json, sys
    _self_test()
    for f in sys.argv[1:]:
        R = json.load(open(f)); n = s = h = 0
        for t in R['tracks']:
            c, k = count([math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:])], t['w']); n += c > 0; s += c; h += k
        print('%s: %d tracks, %d with width change, %d changes, %d short hooks' % (f, len(R['tracks']), n, s, h))
