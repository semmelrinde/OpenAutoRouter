""
import math


def stuecke(laengen, breiten):
    ""
    st = []
    for i, (l, w) in enumerate(zip(laengen, breiten)):
        if l < 1e-9: continue
        if st and abs(st[-1][0] - w) < 1e-6: st[-1][1] += l; st[-1][3] = i
        else: st.append([w, l, i, i])
    return st


def zaehlen(laengen, breiten, kurz=1.0):
    ""
    st = stuecke(laengen, breiten)
    return max(0, len(st) - 1), sum(1 for a, b, c in zip(st[:-2], st[1:-1], st[2:]) if b[1] < kurz and b[0] > a[0] + 1e-6 and b[0] > c[0] + 1e-6)


def widerstand(laengen, breiten):
    ""
    return sum(l / w for l, w in zip(laengen, breiten) if l >= 1e-9)


def _am_ziel(w, z):
    ""
    return w >= math.floor(z * 40) / 40 - 1e-9


def _gewollt(wa, za, wb, zb):
    ""
    if za is None or zb is None: return False
    if wa > wb: wa, za, wb, zb = wb, zb, wa, za
    return zb > za + 1e-9 and _am_ziel(wa, za)


def _profil(l, w, z, bo, lam, mu):
    ""
    idx = [i for i in range(len(l)) if l[i] >= 1e-9]
    lev = sorted({round(w[i], 6) for i in idx}, reverse=True)
    INF = float('inf'); kosten = []; zurueck = []
    for n, i in enumerate(idx):
        cur = []; zr = []
        for L in lev:
            if L > w[i] + 1e-9 or L < bo[i] - 1e-9: cur.append(INF); zr.append(-1); continue
            c0 = l[i] * (math.log(w[i] / L) + mu * (1.0 / L - 1.0 / w[i]))
            if n == 0: cur.append(c0); zr.append(-1); continue
            ip = idx[n - 1]; best = INF; bj = -1
            for j, Lp in enumerate(lev):
                cp = kosten[-1][j]
                if cp == INF: continue
                if abs(Lp - L) > 1e-9 and not _gewollt(Lp, z[ip], L, z[i]): cp += lam
                if cp < best - 1e-12: best = cp; bj = j
            cur.append(best + c0); zr.append(bj)
        kosten.append(cur); zurueck.append(zr)
    k = min(range(len(lev)), key=lambda j: (kosten[-1][j], j)); neu = list(w)
    for n in range(len(idx) - 1, -1, -1):
        neu[idx[n]] = min(w[idx[n]], lev[k]); k = zurueck[n][k]
    return neu


def _hoecker_weg(l, w, z, bo, kurz, grenze):
    ""
    w = list(w)
    for _ in range(len(w) + 1):
        st = stuecke(l, w); kand = []
        for a, b, c in zip(st[:-2], st[1:-1], st[2:]):
            if not (b[1] < kurz and b[0] > a[0] + 1e-6 and b[0] > c[0] + 1e-6): continue
            if _gewollt(a[0], z[a[3]], b[0], z[b[2]]) and _gewollt(c[0], z[c[2]], b[0], z[b[3]]): continue
            v = list(w); ziel = max(a[0], c[0])
            for i in range(b[2], b[3] + 1): v[i] = min(v[i], max(ziel, bo[i]))
            if v != w and widerstand(l, v) <= grenze: kand.append((widerstand(l, v), b[2], v))
        if not kand: break
        w = min(kand)[2]
    return w


def beruhigen(laengen, breiten, ziel=None, boden=0.0, lam=0.35, kurz=1.0, r_max=0.05):
    ""
    l = list(laengen); w = list(breiten)
    if len(stuecke(l, w)) < 2: return w
    z = list(ziel) if ziel is not None else [None] * len(w)
    bo = [min(wi, boden) for wi in w]
    grenze = widerstand(l, w) * (1.0 + r_max) + 1e-12
    if kurz > 0: w = _hoecker_weg(l, w, z, bo, kurz, grenze)
    p = _profil(l, w, z, bo, lam, 0.0)
    if widerstand(l, p) > grenze:
        lo, hi = 0.0, 0.25
        while widerstand(l, _profil(l, w, z, bo, lam, hi)) > grenze and hi < 1e6: lo = hi; hi *= 4.0
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            if widerstand(l, _profil(l, w, z, bo, lam, mid)) > grenze: lo = mid
            else: hi = mid
        p = _profil(l, w, z, bo, lam, hi)
        if widerstand(l, p) > grenze: p = w
    return _hoecker_weg(l, p, z, bo, kurz, grenze) if kurz > 0 else p


def _selbsttest():
    def b(profil, **kw):
        l = [x[1] for x in profil]; w = [x[0] for x in profil]
        return [[round(a[0], 3), round(a[1], 2)] for a in stuecke(l, beruhigen(l, w, **kw))]

    kv = [(0.2, 1.24), (0.25, 0.35), (0.2, 0.65), (0.25, 0.57), (0.15, 0.8), (0.2, 1.39), (0.25, 0.71)]
    assert b(kv, r_max=1.0) == [[0.2, 2.81], [0.15, 0.8], [0.2, 2.1]], b(kv, r_max=1.0)

    l = [x[1] for x in kv]; w = [x[0] for x in kv]
    assert all(n <= a + 1e-9 for n, a in zip(beruhigen(l, w), w))

    assert b([(0.225, 12.0), (0.2, 2.8)]) == [[0.225, 12.0], [0.2, 2.8]]

    assert b([(0.3, 5.0), (1.0, 0.5), (0.25, 5.0)], r_max=1.0) == [[0.3, 5.5], [0.25, 5.0]]

    assert b([(1.0, 3.0), (0.6, 0.3)], ziel=[1.0, 0.6]) == [[1.0, 3.0], [0.6, 0.3]]

    assert b([(0.15, 3.0), (0.3, 0.4), (0.15, 3.0)], boden=0.3, r_max=1.0) == [[0.15, 3.0], [0.3, 0.4], [0.15, 3.0]]

    assert b([(0.3, 6.26), (1.0, 0.85), (0.3, 1.2)], r_max=0.05) == [[0.3, 6.26], [1.0, 0.85], [0.3, 1.2]]
    print('breitenprofil: Selbsttest ok')


if __name__ == '__main__':
    import json, sys
    _selbsttest()
    for f in sys.argv[1:]:
        R = json.load(open(f)); n = s = h = 0
        for t in R['tracks']:
            c, k = zaehlen([math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:])], t['w']); n += c > 0; s += c; h += k
        print('%s: %d Bahnen, %d mit Breitenwechsel, %d Wechsel, %d kurze Hoecker' % (f, len(R['tracks']), n, s, h))
