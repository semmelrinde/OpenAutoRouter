import json, math, sys, collections
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

MAXPADS_NET = 16


def mst(pts):
    P = np.asarray(pts, float); n = len(P)
    if n < 2: return np.zeros((0, 4)), 0.0
    inside = np.zeros(n, bool); inside[0] = True
    d = np.hypot(*(P - P[0]).T); from_ = np.zeros(n, int); edges = []; L = 0.0
    for _ in range(n - 1):
        d2 = np.where(inside, np.inf, d); j = int(np.argmin(d2)); i = from_[j]
        edges.append((P[i, 0], P[i, 1], P[j, 0], P[j, 1])); L += d2[j]; inside[j] = True
        dn = np.hypot(*(P - P[j]).T); better = dn < d; d = np.where(better, dn, d); from_ = np.where(better, j, from_)
    return np.array(edges), L


def crossings(S):
    if len(S) < 2: return 0
    ax, ay, bx, by = (S[:, i][:, None] for i in range(4)); cx, cy, dx, dy = (S[:, i][None, :] for i in range(4))
    def o(px, py, qx, qy, rx, ry): return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    m = (o(ax, ay, bx, by, cx, cy) * o(ax, ay, bx, by, dx, dy) < -1e-9) & (o(cx, cy, dx, dy, ax, ay) * o(cx, cy, dx, dy, bx, by) < -1e-9)
    return int(np.count_nonzero(np.triu(m, 1)))




EXTRA = ['rudy_signal_max', 'rudy_signal_p95', 'rudy_signal_over', 'pin_f_max', 'pin_b_max',
          'pin_inner_max', 'island_radius_medium', 'island_radius_max', 'converter_analog_min',
          'converter_analog_close', 'bga_nets', 'bga_escape_ratio', 'bga_pitch_min',
          'gnd_layers_n', 'signal_layers_n', 'baseline_known', 'fast_mst', 'analog_mst', 'sw_mst']

def extra_features(B, K, RG=None, gnd_layers=None):
    rg = (RG or {}).get('router', {})
    L = int(B.get('cu_layers') or rg.get('cu_layers') or 2)
    pitch = max(1e-6, float(rg.get('sigw', 0.25)) + float(rg.get('clr', 0.2)))
    if gnd_layers is None and 'gnd_layers' in rg:
        gnd_layers = rg['gnd_layers']
    if isinstance(gnd_layers, str): gnd_layers = [s for s in gnd_layers.split(',') if s]
    ng = len(set(gnd_layers)) if gnd_layers is not None else None
    reserved = ng
    from analysis.areas import active
    if active({}, K) and gnd_layers is not None and any('=' in s for s in gnd_layers):
        from analysis.areas import assignment
        fl = assignment(B, {}, K, gnd_layers=gnd_layers)
        reserved = len(fl)
        ng = sum(bool(nn) and set(nn) <= set(K.get('gnd') or []) for nn in fl.values())
    ns = max(1, L - (reserved or 0))
    pts = np.asarray([p for o in B['outline'] for p in o], float)
    lo, hi = pts.min(0), pts.max(0); span = np.maximum(hi - lo, 1e-6)
    cell = span / 16; area = float(np.prod(cell))
    layers = ['F', 'B'] + ['In%d' % i for i in range(1, max(1, L - 1))]
    pins = np.zeros((len(layers), 16, 16)); demand = np.zeros((16, 16))
    nets = collections.defaultdict(list); byref = collections.defaultdict(list)
    for p in B['pads']:
        if not p.get('net') or p['net'].startswith(('unconnected', '#')): continue
        nets[p['net']].append(p); byref[p['ref']].append(p)
        i, j = np.clip(((np.array([p['x'], p['y']]) - lo) / cell).astype(int), 0, 15)
        for layer in p['layers']:
            if layer in layers: pins[layers.index(layer), j, i] += 1
    sums = dict(fast=0., analog=0., sw=0.)
    for n, ps in sorted(nets.items()):
        if n in set(K.get('gnd') or []) or len(ps) < 2: continue
        xy = np.asarray([[p['x'], p['y']] for p in ps]); a, b = xy.min(0), xy.max(0)
        size = np.maximum(b - a, cell)
        ij0 = np.clip(((a - lo) / cell).astype(int), 0, 15)
        ij1 = np.clip(((b - lo) / cell).astype(int), 0, 15) + 1
        demand[ij0[1]:ij1[1], ij0[0]:ij1[0]] += size.sum() / np.prod(size) * pitch / ns
        category = (K.get('net_list', {}).get(n) or {}).get('category')
        if category in sums: sums[category] += mst(xy)[1]
    fps = {f['ref']: f for f in B['fps']}
    radii = []
    for anchor, isl in (K.get('islands') or {}).items():
        if anchor not in fps: continue
        a = fps[anchor]
        ds = [math.hypot(fps[r]['x'] - a['x'], fps[r]['y'] - a['y'])
              for r in isl.get('members', []) if r in fps]
        if ds: radii.append(max(ds))
    swrefs = {p['ref'] for n, ps in nets.items() if (K.get('net_list', {}).get(n) or {}).get('category') == 'sw' for p in ps}
    arefs = {p['ref'] for n, ps in nets.items() if (K.get('net_list', {}).get(n) or {}).get('category') == 'analog' for p in ps}
    distances = [math.hypot(fps[a]['x'] - fps[b]['x'], fps[a]['y'] - fps[b]['y'])
                 for a in sorted(swrefs & fps.keys()) for b in sorted(arefs & fps.keys()) if a != b]
    bga_count = 0; ratios = []; pitches = []
    for r, f in sorted(fps.items()):
        if not any(t in (f.get('fp', '') + f.get('lib', '')).upper() for t in ('BGA', 'CSP')): continue
        ps = byref[r]
        if len(ps) < 2: continue
        xy = np.unique([[p['x'], p['y']] for p in ps], axis=0)

        gaps = [d for axis in (0, 1) for d in np.diff(np.unique(np.round(xy[:, axis], 4))) if d > 0.01]
        bp = min(gaps) if gaps else None
        if bp is not None: pitches.append(bp)
        needed = len({p['net'] for p in ps if p['net'] not in set(K.get('gnd') or []) and any(q['ref'] != r for q in nets[p['net']])})
        bga_count += needed
        perimeter = 2 * float(np.ptp(xy, axis=0).sum())
        capacity = perimeter / pitch * ns
        if needed: ratios.append(needed / max(capacity, 1e-6))
    return dict(features_version=2, rudy_signal_max=float(demand.max()),
                rudy_signal_p95=float(np.percentile(demand, 95)), rudy_signal_over=float(np.mean(demand > 1)),
                pin_f_max=float(pins[0].max() / area), pin_b_max=float(pins[1].max() / area),
                pin_inner_max=float(pins[2:].max() / area) if L > 2 else 0., island_radius_medium=float(np.mean(radii)) if radii else 0.,
                island_radius_max=max(radii, default=0.), converter_analog_min=min(distances) if distances else None,
                converter_analog_close=sum(d < 3 for d in distances), bga_nets=bga_count,
                bga_escape_ratio=max(ratios, default=0.), bga_pitch_min=min(pitches) if pitches else None,
                gnd_layers_n=ng, signal_layers_n=L - reserved if reserved is not None else None, baseline_known=int(ng is not None),
                **{k + '_mst': v for k, v in sums.items()})


def feats(B, K, RG=None, gnd_layers=None):
    R = (RG or {}).get('router', {})
    clr = float(R.get('clr', 0.2)); w = float(R.get('sigw', 0.25)); L = int(B.get('cu_layers') or R.get('cu_layers') or 2)
    contour = unary_union([Polygon(o) for o in B['outline'] if len(o) >= 3]).buffer(0)
    A = max(contour.area, 1e-6); bx = contour.bounds
    GND = set(K.get('gnd') or [])
    pads = [p for p in B['pads'] if p['net'] and not p['net'].startswith(('unconnected', '#'))]
    net_list = collections.defaultdict(list)
    for p in pads: net_list[p['net']].append(p)
    multi = {n: v for n, v in net_list.items() if len(v) >= 2}

    mst_sum = mst_gnd = 0.0; segs = []; net_id = []; double_sided = 0; hpwl = 0.0; longest = 0.0
    for k, (n, v) in enumerate(sorted(multi.items())):
        pts = [(p['x'], p['y']) for p in v]; S, Lm = mst(pts)
        if n in GND: mst_gnd += Lm
        else: mst_sum += Lm
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]; hpwl += max(xs) - min(xs) + max(ys) - min(ys)
        if len(S): longest = max(longest, float(np.max(np.hypot(S[:, 2] - S[:, 0], S[:, 3] - S[:, 1]))))
        smd = {p['layers'][0] for p in v if len(p['layers']) == 1}
        if len(smd) > 1: double_sided += 1
        if n not in GND and len(v) <= MAXPADS_NET and len(S):
            segs.append(S); net_id += [k] * len(S)
    cross = 0
    if segs:
        S = np.vstack(segs); ids = np.array(net_id)
        if len(S) <= 3000:
            ax, ay, bx_, by = (S[:, i][:, None] for i in range(4)); cx, cy, dx, dy = (S[:, i][None, :] for i in range(4))
            def o(px, py, qx, qy, rx, ry): return (qx - px) * (ry - py) - (qy - py) * (rx - px)
            m = (o(ax, ay, bx_, by, cx, cy) * o(ax, ay, bx_, by, dx, dy) < -1e-9) & (o(cx, cy, dx, dy, ax, ay) * o(cx, cy, dx, dy, bx_, by) < -1e-9)
            m &= ids[:, None] != ids[None, :]
            cross = int(np.count_nonzero(np.triu(m, 1)))

    g = 1.0; nx = max(1, int(math.ceil((bx[2] - bx[0]) / g))); ny = max(1, int(math.ceil((bx[3] - bx[1]) / g)))
    rudy = np.zeros((ny, nx))
    for S in segs + ([mst([(p['x'], p['y']) for p in net_list[n]])[0] for n in GND if n in multi and len(net_list[n]) <= 200]):
        for x1, y1, x2, y2 in S:
            x0, x1_ = sorted((x1, x2)); y0, y1_ = sorted((y1, y2))
            wb, hb = max(x1_ - x0, g), max(y1_ - y0, g)
            i0 = int((x0 - bx[0]) / g); i1 = min(nx, int((x1_ - bx[0]) / g) + 1)
            j0 = int((y0 - bx[1]) / g); j1 = min(ny, int((y1_ - bx[1]) / g) + 1)
            rudy[j0:j1, i0:i1] += (wb + hb) / (wb * hb)
    cap_k = L * g / (w + clr)
    inner = rudy[rudy > 0]

    H, _, _ = np.histogram2d([p['x'] for p in pads], [p['y'] for p in pads], bins=[max(1, nx // 2), max(1, ny // 2)],
                             range=[[bx[0], bx[2]], [bx[1], bx[3]]])
    crt = [Polygon(q) for c in B.get('courtyards', []) for q in c['polys'] if len(q) >= 3]
    occupied = sum(c.area for c in crt)

    border_n = sum(1 for p in pads if contour.boundary.distance(__import__('shapely').geometry.Point(p['x'], p['y'])) < 1.0)
    n_pads = len(pads); n_nets = len(multi)
    out = dict(
        cu_layers=L, clr=clr, track=w, surface=round(A, 2), n_parts=len(B['fps']), n_pads=n_pads, n_nets=n_nets,
        pads_per_cm2=round(100 * n_pads / A, 3), occupancy=round(occupied / A, 4),
        mst=round(mst_sum, 2), mst_gnd=round(mst_gnd, 2), hpwl=round(hpwl, 2), mst_per_area=round(mst_sum / A, 4),
        mst_rel_diag=round(mst_sum / max(1e-6, math.hypot(bx[2] - bx[0], bx[3] - bx[1])), 3), longest_edge=round(longest, 2),
        crossings=cross, cross_per_net=round(cross / max(1, n_nets), 4), double_sided=double_sided,
        rudy_max=round(float(rudy.max()) / cap_k, 4) if rudy.size else 0.0,
        rudy_p95=round(float(np.percentile(inner, 95)) / cap_k, 4) if inner.size else 0.0,
        rudy_medium=round(float(inner.mean()) / cap_k, 4) if inner.size else 0.0,
        rudy_over=round(float(np.mean(rudy > 0.5 * cap_k)), 4),
        pads_max_4mm2=int(H.max()) if H.size else 0, border_pads=border_n,
        share_current=round(sum(1 for n in multi if (K['net_list'].get(n) or {}).get('category') in ('current_val', 'supply', 'sw')) / max(1, n_nets), 3),
    )

    out.update(extra_features(B, K, RG, gnd_layers))
    return out


if __name__ == '__main__':
    B = json.load(open(sys.argv[1])); K = json.load(open(sys.argv[2])); RG = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else None
    import time; t = time.time(); M = feats(B, K, RG); M['_s'] = round(time.time() - t, 3)
    print(json.dumps(M, indent=1))
