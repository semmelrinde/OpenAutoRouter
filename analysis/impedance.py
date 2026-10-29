import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl

C0 = 299792458.0; EPS0 = 8.8541878128e-12

def grid(points, lo, hi, fine, coarse, grow=1.15):
    p = sorted(set([lo, hi] + [q for q in points if lo <= q <= hi]))
    out = [p[0]]
    for a, b in zip(p[:-1], p[1:]):
        L = b - a

        seq = []; st = fine; pos = 0
        while True:
            if 2*pos + 2*st >= L: break
            seq.append(st); pos += st; st = min(st*grow, coarse)
        rest = L - 2*pos
        n_mid = max(1, int(np.ceil(rest/coarse)))
        mid = [rest/n_mid]*n_mid
        steps = seq + mid + seq[::-1]
        x = a
        for d in steps:
            x += d; out.append(x)
        out[-1] = b
    return np.array(out)

def solve(w, s, h, t, er, er_m=3.8, c_sub=0.025, c_tr=0.012, g=None, mode='odd', single=False,
          fine=0.0008, coarse=0.04, X=3.0, Y=3.0, with_mask=True, upper=None, layers=None):
    if not all(np.isfinite(v) for v in (w, s, h, t, er)) or w <= 0 or s < 0 or h <= 0 or t < 0 or er <= 0:
        raise ValueError('Field solver: invalid geometry/permittivity')
    if upper is not None:
        if upper <= 0: raise ValueError("Stripline: upper spacing must be positive")
        Y = h + t + upper
        with_mask = False
    if single:
        track_list = [(-w/2, w/2, 1.0)]
    else:
        xm = (w + s)/2; v2 = -1.0 if mode == 'odd' else 1.0
        track_list = [(-xm - w/2, -xm + w/2, 1.0), (xm - w/2, xm + w/2, v2)]
    outer = max(b[1] for b in track_list)
    gk = [] if g is None else [(-X, -outer - g), (outer + g, X)]
    if not with_mask: c_sub = c_tr = 0.0
    kx = [0.0]
    for x0, x1, _ in track_list: kx += [x0, x1, x0 - c_tr, x1 + c_tr]
    for x0, x1 in gk: kx += [x0, x1, x0 - c_tr if x0 > -X else x0, x1 + c_tr if x1 < X else x1]
    ky = [0.0, h, h + t, h + c_sub, h + t + c_tr]
    if layers:
        ky.extend(np.cumsum([d for d, _ in layers]).tolist())
    x = grid([q for q in kx if -X < q < X], -X, X, fine, coarse)
    y = grid([q for q in ky if 0 < q < Y], 0.0, Y, fine, coarse)
    nx, ny = len(x), len(y)
    dx = np.diff(x); dy = np.diff(y)
    xc = (x[:-1] + x[1:])/2; yc = (y[:-1] + y[1:])/2
    XC, YC = np.meshgrid(xc, yc, indexing='ij')
    def cu(XX, YY):
        m = np.zeros(XX.shape, bool)
        for x0, x1, _ in track_list: m |= (XX >= x0) & (XX <= x1) & (YY >= h) & (YY <= h + t)
        for x0, x1 in gk: m |= (XX >= x0) & (XX <= x1) & (YY >= h) & (YY <= h + t)
        return m
    eps = np.ones(XC.shape); eps[YC < h] = er
    if upper is not None: eps[:] = er
    if layers:
        z0 = 0.0
        for d, dk in layers:
            eps[(YC >= z0) & (YC < z0 + d)] = dk
            z0 += d
    if c_sub > 0 or c_tr > 0:
        lack = (YC >= h) & (YC < h + c_sub)
        for x0, x1, _ in track_list: lack |= (XC >= x0 - c_tr) & (XC <= x1 + c_tr) & (YC >= h) & (YC < h + t + c_tr)
        for x0, x1 in gk: lack |= (XC >= x0 - c_tr) & (XC <= x1 + c_tr) & (YC >= h) & (YC < h + t + c_tr)
        lack &= ~cu(XC, YC)
        eps[lack] = er_m
    XN, YN = np.meshgrid(x, y, indexing='ij')
    V = np.zeros((nx, ny)); pinned = np.zeros((nx, ny), bool)
    pinned[:, 0] = pinned[:, -1] = pinned[0, :] = pinned[-1, :] = True
    for x0, x1, v in track_list:
        m = (XN >= x0 - 1e-12) & (XN <= x1 + 1e-12) & (YN >= h - 1e-12) & (YN <= h + t + 1e-12); pinned |= m; V[m] = v
    for x0, x1 in gk:
        m = (XN >= x0 - 1e-12) & (XN <= x1 + 1e-12) & (YN >= h - 1e-12) & (YN <= h + t + 1e-12); pinned |= m; V[m] = 0

    E = np.zeros((nx - 1, ny + 1)); E[:, 1:-1] = eps * dy[None, :]
    ax = 0.5*(E[:, :-1] + E[:, 1:]) / dx[:, None]
    F = np.zeros((nx + 1, ny - 1)); F[1:-1, :] = eps * dx[:, None]
    ay = 0.5*(F[:-1, :] + F[1:, :]) / dy[None, :]
    vacant = ~pinned; idx = -np.ones((nx, ny), int); idx[vacant] = np.arange(vacant.sum()); n = vacant.sum()
    diag = np.zeros((nx, ny)); b = np.zeros(n); R = []; Cc = []; Vv = []
    def edge(i0, j0, i1, j1, a):
        f0 = vacant[i0, j0]; f1 = vacant[i1, j1]
        np.add.at(diag, (i0[f0], j0[f0]), a[f0]); np.add.at(diag, (i1[f1], j1[f1]), a[f1])
        m = f0 & f1
        R.extend([idx[i0, j0][m], idx[i1, j1][m]]); Cc.extend([idx[i1, j1][m], idx[i0, j0][m]]); Vv.extend([-a[m], -a[m]])
        m2 = f0 & ~f1; np.add.at(b, idx[i0, j0][m2], a[m2]*V[i1, j1][m2])
        m3 = ~f0 & f1; np.add.at(b, idx[i1, j1][m3], a[m3]*V[i0, j0][m3])
    I, J = np.meshgrid(np.arange(nx - 1), np.arange(ny), indexing='ij'); edge(I, J, I + 1, J, ax)
    I, J = np.meshgrid(np.arange(nx), np.arange(ny - 1), indexing='ij'); edge(I, J, I, J + 1, ay)
    R.append(idx[vacant]); Cc.append(idx[vacant]); Vv.append(diag[vacant])
    A = sp.csc_matrix((np.concatenate(Vv), (np.concatenate(R), np.concatenate(Cc))), shape=(n, n))
    phi = V.copy(); phi[vacant] = spl.spsolve(A, b)
    W = 0.5*EPS0*(np.sum(ax*np.diff(phi, axis=0)**2) + np.sum(ay*np.diff(phi, axis=1)**2))
    return W, (nx, ny)

def z(w, s, h=0.0994, t=0.035, er=4.1, **kw):
    only_odd = kw.pop('only_odd', False)
    clearance = dict(kw); clearance['er_m'] = 1.0
    if kw.get('layers'): clearance['layers'] = [(d, 1.0) for d, _ in kw['layers']]
    if kw.get('single'):
        C = 2*solve(w, s, h, t, er, **kw)[0]; Ca = 2*solve(w, s, h, t, 1.0, **clearance)[0]
        return dict(Zse=1/(C0*np.sqrt(C*Ca)), eeff=C/Ca)
    Co = solve(w, s, h, t, er, mode='odd', **kw)[0]; Coa = solve(w, s, h, t, 1.0, mode='odd', **clearance)[0]
    if only_odd: return dict(Zdiff=2/(C0*np.sqrt(Co*Coa)), eeff=Co/Coa)
    Ce = solve(w, s, h, t, er, mode='even', **kw)[0]; Cea = solve(w, s, h, t, 1.0, mode='even', **clearance)[0]
    Zo = 1/(C0*np.sqrt(Co*Coa)); Ze = 1/(C0*np.sqrt(Ce*Cea))
    return dict(Zdiff=2*Zo, Zodd=Zo, Zeven=Ze, Zcomm=Ze/2, eeff=Co/Coa)

if __name__ == '__main__':
    import sys
    a = [x for x in sys.argv[1:] if '=' not in x]; o = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    kw = dict(h=float(o.get('h', 0.0994)), er=float(o.get('er', 4.1)), t=float(o.get('t', 0.035)))
    if 'g' in o: kw['g'] = float(o['g'])
    if 'upper' in o: kw['upper'] = float(o['upper'])
    if o.get('lack', '1') == '0': kw['with_mask'] = False
    if a and a[0] == 'search':
        s, goal = float(o.get('s', 0.2)), float(o.get('goal', 90))
        lo, hi = 0.05, 0.6
        for _ in range(30):
            m = (lo + hi) / 2
            if z(m, s, **kw)['Zdiff'] > goal: lo = m
            else: hi = m
        print('Width %.4f mm at spacing %.3f mm -> Zdiff %.1f Ohm' % (m, s, z(m, s, **kw)['Zdiff']))
    elif len(a) >= 2 and a[1] == 'single':
        r = z(float(a[0]), 0.0, single=True, **kw); print('Zse %.1f Ohm, eeff %.2f' % (r['Zse'], r['eeff']))
    else:
        r = z(float(a[0]), float(a[1]), **kw)
        print('Zdiff %.1f Ohm, Zodd %.1f, Zeven %.1f, Zcomm %.1f, eeff %.2f' % (r['Zdiff'], r['Zodd'], r['Zeven'], r['Zcomm'], r['eeff']))
