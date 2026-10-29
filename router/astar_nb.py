import math
import types
import numpy as np
import numba

OFX = np.array([1, -1, 0, 0, 1, 1, -1, -1], dtype=np.int64)
OFY = np.array([0, 0, 1, -1, 1, -1, 1, -1], dtype=np.int64)


@numba.njit(cache=True, nogil=True)
def _h(n, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend=0.0):
    idx = n % NN; ix = idx % NX; iy = idx // NX
    x = X0 + (ix + 0.5) * RES; y = Y0 + (iy + 0.5) * RES
    dx = max(tx0 - x, 0.0, x - tx1); dy = max(ty0 - y, 0.0, y - ty1)


    return math.sqrt(dx * dx + dy * dy) if bend > 0.0 else math.hypot(dx, dy)


@numba.njit(cache=True, nogil=True, inline='always')
def _hpush(hf, hn, hs, f, n):
    i = hs
    while i > 0:
        p = (i - 1) >> 1
        if hf[p] < f or (hf[p] == f and hn[p] <= n): break
        hf[i] = hf[p]; hn[i] = hn[p]; i = p
    hf[i] = f; hn[i] = n
    return hs + 1


@numba.njit(cache=True, nogil=True, inline='always')
def _hpop(hf, hn, hs):
    hs -= 1
    if hs == 0: return 0
    f = hf[hs]; n = hn[hs]; i = 0
    while True:
        c = 2 * i + 1
        if c >= hs: break
        r = c + 1
        if r < hs and (hf[r] < hf[c] or (hf[r] == hf[c] and hn[r] < hn[c])): c = r
        if hf[c] < f or (hf[c] == f and hn[c] < n):
            hf[i] = hf[c]; hn[i] = hn[c]; i = c
        else: break
    hf[i] = f; hn[i] = n
    return hs


@numba.njit(cache=True, nogil=True)
def astar_core(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win,
               NX, NN, NL, X0, Y0, RES, g, par, gst, cst, tst, run, ofx, ofy, ofs, wminf, hfac, info, viaz, vzk,
               bend, via_extra, vpr, via_anchor):
    ixa, ixb, iya, iyb = win[0], win[1], win[2], win[3]
    tx0, ty0, tx1, ty1 = tbox[0], tbox[1], tbox[2], tbox[3]
    for t in tgt: tst[t] = run
    cap_k = max(65536, 2 * src.shape[0] + 64)
    hf = np.empty(cap_k, dtype=np.float64); hn = np.empty(cap_k, dtype=np.int64); hs = 0
    for s in src:
        gst[s] = run; g[s] = 0.0; par[s] = -1
        f0 = 0.0 if via_target else hfac * _h(s, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
        hs = _hpush(hf, hn, hs, f0, np.int64(s))
    while hs > 0:
        n = hn[0]
        hs = _hpop(hf, hn, hs)
        if hs + NL + 12 > cap_k:
            cap_k *= 2
            hf2 = np.empty(cap_k, dtype=np.float64); hn2 = np.empty(cap_k, dtype=np.int64)
            hf2[:hs] = hf[:hs]; hn2[:hs] = hn[:hs]; hf = hf2; hn = hn2
        if n < 0:
            n = -n - 1
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q; k -= 1; q = par[q]
            return out
        if cst[n] == run: continue
        cst[n] = run
        idx = n % NN
        li = n // NN
        if via_target:
            if tst[n] == run:
                L = 0; q = n
                while q != -1: L += 1; q = par[q]
                out = np.empty(L, dtype=np.int64); q = n; k = L - 1
                while q != -1: out[k] = q; k -= 1; q = par[q]
                return out
            if viaf[idx] and (li == 0 or li == NL - 1):
                hv = _h(n, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)

                hs = _hpush(hf, hn, hs, g[n] + 0.3 + 2.0 * hv + (3.0 * PFv if vbad[idx] else 0.0)
                            + (vzk * np.float64(viaz[idx]) if viaz.shape[0] > 0 else 0.0), np.int64(-n - 1))
        elif tst[n] == run:
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q; k -= 1; q = par[q]
            return out
        iy = idx // NX; ix = idx - iy * NX
        gn = g[n]


        pdx = 0; pdy = 0; has_direction = False
        if bend > 0.0 and par[n] >= 0 and par[n] // NN == li:
            pq = par[n] % NN
            pdx = ix - pq % NX; pdy = iy - pq // NX; has_direction = True
        for k in range(8):
            jx = ix + ofx[k]; jy = iy + ofy[k]
            if jx < ixa or jx > ixb or jy < iya or jy > iyb:
                info[0] += 1
                continue
            m = li * NN + jy * NX + jx
            wm = wmin if wminf.shape[0] == 0 else wminf[m]
            if capHf[m] < wm and tst[m] != run: continue
            ng = gn + ofs[k] * np.float64(mult[m])
            if has_direction and (ofx[k] != pdx or ofy[k] != pdy): ng += bend
            if gst[m] != run or ng < g[m]:
                gst[m] = run; g[m] = ng; par[m] = n
                hm = 0.0 if via_target else hfac * _h(m, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
                hs = _hpush(hf, hn, hs, ng + hm, np.int64(m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PFv * vbad[idx]) + 0.2 * np.float64(hist[n])

            if via_anchor.shape[0] > 0: vc = vc + np.float64(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                m = lj * NN + idx
                vcl = vc
                if via_extra.shape[0] > 0: vcl = vc + vpr[li * NL + lj] * np.float64(via_extra[idx])
                ng = gn + vcl
                if gst[m] != run or ng < g[m]:
                    gst[m] = run; g[m] = ng; par[m] = n
                    hs = _hpush(hf, hn, hs, ng + hfac * _h(m, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend), np.int64(m))
    return np.empty(0, dtype=np.int64)


@numba.njit(cache=True, nogil=True)
def astar_direction(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win,
               NX, NN, NL, X0, Y0, RES, g, par, gst, cst, tst, run, ofx, ofy, ofs, wminf, hfac, info, viaz, vzk,
               bend, via_extra, vpr, via_anchor):
    ixa, ixb, iya, iyb = win[0], win[1], win[2], win[3]
    tx0, ty0, tx1, ty1 = tbox[0], tbox[1], tbox[2], tbox[3]
    hfac = min(1.0, hfac) * (1.4142 / math.sqrt(2.0))
    for t in tgt: tst[t] = run
    cap_k = max(65536, 2 * src.shape[0] + 64)
    hf = np.empty(cap_k, dtype=np.float64); hn = np.empty(cap_k, dtype=np.int64); hs = 0
    for s0 in src:
        s = s0 * 9 + 8
        gst[s] = run; g[s] = 0.0; par[s] = -1
        f0 = 0.0 if via_target else hfac * _h(s // 9, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
        hs = _hpush(hf, hn, hs, f0, np.int64(s))
    while hs > 0:
        n = hn[0]
        hs = _hpop(hf, hn, hs)
        if hs + NL + 12 > cap_k:
            cap_k *= 2
            hf2 = np.empty(cap_k, dtype=np.float64); hn2 = np.empty(cap_k, dtype=np.int64)
            hf2[:hs] = hf[:hs]; hn2[:hs] = hn[:hs]; hf = hf2; hn = hn2
        if n < 0:
            n = -n - 1
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        if cst[n] == run: continue
        cst[n] = run
        node = n // 9; direction = n % 9
        idx = node % NN
        li = node // NN
        if via_target:
            if tst[node] == run:
                L = 0; q = n
                while q != -1: L += 1; q = par[q]
                out = np.empty(L, dtype=np.int64); q = n; k = L - 1
                while q != -1: out[k] = q // 9; k -= 1; q = par[q]
                return out
            if viaf[idx] and (li == 0 or li == NL - 1):
                hv = _h(node, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)

                hs = _hpush(hf, hn, hs, g[n] + 0.3 + 2.0 * hv + (3.0 * PFv if vbad[idx] else 0.0)
                            + (vzk * np.float64(viaz[idx]) if viaz.shape[0] > 0 else 0.0), np.int64(-n - 1))
        elif tst[node] == run:
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        iy = idx // NX; ix = idx - iy * NX
        gn = g[n]

        for k in range(8):
            jx = ix + ofx[k]; jy = iy + ofy[k]
            if jx < ixa or jx > ixb or jy < iya or jy > iyb:
                info[0] += 1
                continue
            nextnode = li * NN + jy * NX + jx
            m = nextnode * 9 + k
            wm = wmin if wminf.shape[0] == 0 else wminf[nextnode]
            if capHf[nextnode] < wm and tst[nextnode] != run: continue
            ng = gn + ofs[k] * np.float64(mult[nextnode])
            if direction != 8 and direction != k: ng += bend
            if gst[m] != run or ng < g[m]:
                gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                hm = 0.0 if via_target else hfac * _h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
                hs = _hpush(hf, hn, hs, ng + hm, np.int64(m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PFv * vbad[idx]) + 0.2 * np.float64(hist[node])

            if via_anchor.shape[0] > 0: vc = vc + np.float64(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                nextnode = lj * NN + idx
                m = nextnode * 9 + 8
                vcl = vc
                if via_extra.shape[0] > 0: vcl = vc + vpr[li * NL + lj] * np.float64(via_extra[idx])
                ng = gn + vcl
                if gst[m] != run or ng < g[m]:
                    gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                    hs = _hpush(hf, hn, hs, ng + hfac * _h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend), np.int64(m))
    return np.empty(0, dtype=np.int64)



_PY_GLOBALS = dict(globals(), _h=_h.py_func, _hpush=_hpush.py_func, _hpop=_hpop.py_func)
_ASTAR_DIRECTION_PY = types.FunctionType(astar_direction.py_func.__code__, _PY_GLOBALS)

_NO = np.zeros(0, dtype=np.float32)
_NO64 = np.zeros(1, dtype=np.float64)


class Search:
    def __init__(self, NL, NN, RES, direction=False):
        self.direction = direction
        self.exact = None
        size = NL * NN * (9 if direction else 1)
        self.g = np.zeros(size, dtype=np.float64); self.par = np.full(size, -1, dtype=np.int64)
        self.gst = np.zeros(size, dtype=np.int32); self.cst = np.zeros(size, dtype=np.int32)
        self.tst = np.zeros(NL * NN, dtype=np.int32); self.run = 0
        self.ofs = np.array([RES, RES, RES, RES, RES * 1.4142, RES * 1.4142, RES * 1.4142, RES * 1.4142], dtype=np.float64)
        self.info = np.zeros(1, dtype=np.int64)

    def __call__(self, src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win, NX, NN, NL, X0, Y0, RES, wminf, hfac=1.0,
                 viaz=None, vzk=0.0, bend=0.0, via_extra=None, vpr=None, via_anchor=None, python=False):
        if bend > 0 and not self.direction:
            if self.exact is None: self.exact = Search(NL, NN, RES, direction=True)
            result = self.exact(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target,
                                win, NX, NN, NL, X0, Y0, RES, wminf, hfac, viaz, vzk, bend, via_extra, vpr, via_anchor, python)
            self.info[:] = self.exact.info
            return result
        if self.direction:

            x0, x1, y0, y1 = win
            view = mult.reshape(NL, NN // NX, NX)[:, y0:y1+1, x0:x1+1]
            hfac = min(float(hfac), max(0.0, float(view.min())))
            if len(tgt):
                points = [(X0+(t % NN % NX+.5)*RES, Y0+(t % NN // NX+.5)*RES) for t in tgt]
                tbox = (min(tbox[0], min(x for x,y in points)), min(tbox[1], min(y for x,y in points)),
                        max(tbox[2], max(x for x,y in points)), max(tbox[3], max(y for x,y in points)))
        self.run += 1; self.info[0] = 0
        if self.run >= 2 ** 31 - 2:
            self.gst[:] = 0; self.cst[:] = 0; self.tst[:] = 0; self.run = 1
        kernel = astar_direction if self.direction else astar_core
        if python: kernel = _ASTAR_DIRECTION_PY if self.direction else kernel.py_func
        res = kernel(np.asarray(src, dtype=np.int64), np.asarray(list(tgt), dtype=np.int64), np.asarray(tbox, dtype=np.float64),
                         capHf, mult, viaf, vbad, hist, float(wmin), float(VC), float(PFv), bool(via_target),
                         np.asarray(win, dtype=np.int64), NX, NN, NL, float(X0), float(Y0), float(RES),
                         self.g, self.par, self.gst, self.cst, self.tst, np.int32(self.run), OFX, OFY, self.ofs,
                         np.asarray(wminf, dtype=np.float32), float(hfac), self.info,
                         _NO if viaz is None else viaz, float(vzk), float(bend), _NO if via_extra is None else via_extra,
                         _NO64 if vpr is None else vpr, _NO if via_anchor is None else via_anchor)
        return res.tolist() if len(res) else None
