import json, sys, os, math
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union


def geo(polys):
    ps = [Polygon(o, [h for h in hs if len(h) >= 3]) for o, hs in polys if len(o) >= 3]
    ps = [p if p.is_valid else p.buffer(0) for p in ps]
    return unary_union(ps) if ps else Polygon()


def part_list(g): return [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and not p.is_empty]


def raster45(p, tol=math.radians(2.0)):
    r = p.exterior.simplify(0.05); xy = np.asarray(r.coords)
    d = np.diff(xy, axis=0); l = np.hypot(d[:, 0], d[:, 1]); a = np.arctan2(d[:, 1], d[:, 0]) % (math.pi / 4)
    ok = (a < tol) | (a > math.pi / 4 - tol)
    return float((l * ok).sum()), float(l.sum())


for D in sys.argv[1:]:
    C = json.load(open(os.path.join(D, 'cu.json'))); Z = json.load(open(os.path.join(D, 'extra.json'))); E = json.load(open(os.path.join(D, 'k20.json')))
    sup, gnd = set(E['sup']), set(E['gnd'])
    zf = {}
    for z in Z['zones']:
        if z['fill']: zf.setdefault((z['net'], z['layer']), []).append(geo(z['fill']))
    pads = {}
    for p in C['pads']:
        for L in p.get('layers') or []: pads.setdefault((p['net'], L), []).append(Point(p['x'], p['y']))
    out = {}
    for board_side, cu_layers in (('outer', [L for L in E['cu_layers'] if L in ('F', 'B')]), ('inner', [L for L in E['cu_layers'] if L not in ('F', 'B')])):
        st = []; ok45 = tot = 0.0; ok45_g = tot_g = 0.0
        for (n, L), gs in zf.items():
            if L not in cu_layers: continue
            g = unary_union(gs)
            for p in part_list(g):
                if p.area < 0.01: continue
                a, b = raster45(p)
                if n in sup:
                    ok45 += a; tot += b
                    npads = sum(p.covers(q) for q in pads.get((n, L), []))
                    st.append(dict(net=n, layer_name=L, a=p.area, width=2 * p.area / max(1e-9, p.exterior.length), pads=npads,
                                   convex=p.area / max(1e-9, p.convex_hull.area)))
                elif n in gnd:
                    ok45_g += a; tot_g += b
        fl = sum(E['per_layer'][L]['area_mm2'] for L in cu_layers)
        A = np.array([s['a'] for s in st]) if st else np.zeros(1)
        out[board_side] = dict(pieces=len(st), median_mm2=round(float(np.median(A)), 2), p90_mm2=round(float(np.percentile(A, 90)), 1), max_mm2=round(float(A.max()), 1),
                          total_sum_mm2=round(float(A.sum()), 1), share_layer_area=round(float(A.sum()) / max(1e-9, fl), 4),
                          width_median=round(float(np.median([s['width'] for s in st])), 3) if st else 0,
                          pads_median=float(np.median([s['pads'] for s in st])) if st else 0,
                          without_pad=sum(s['pads'] == 0 for s in st), convex_median=round(float(np.median([s['convex'] for s in st])), 3) if st else 0,
                          edge45_supply=round(ok45 / max(1e-9, tot), 3), edge45_gnd=round(ok45_g / max(1e-9, tot_g), 3),
                          largest=[(s['net'], s['layer_name'], round(s['a'], 1)) for s in sorted(st, key=lambda s: -s['a'])[:4]])
    json.dump(out, open(os.path.join(D, 'k20_shape.json'), 'w'), indent=1)
    print(os.path.basename(D), json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'largest'} for k, v in out.items()}))
    print('   largest outer:', out['outer']['largest'], ' inner:', out['inner']['largest'])
