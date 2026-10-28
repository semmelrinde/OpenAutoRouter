""
import json, sys, os, math
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union


def geo(polys):
    ps = [Polygon(o, [h for h in hs if len(h) >= 3]) for o, hs in polys if len(o) >= 3]
    ps = [p if p.is_valid else p.buffer(0) for p in ps]
    return unary_union(ps) if ps else Polygon()


def teile(g): return [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and not p.is_empty]


def raster45(p, tol=math.radians(2.0)):
    ""
    r = p.exterior.simplify(0.05); xy = np.asarray(r.coords)
    d = np.diff(xy, axis=0); l = np.hypot(d[:, 0], d[:, 1]); a = np.arctan2(d[:, 1], d[:, 0]) % (math.pi / 4)
    ok = (a < tol) | (a > math.pi / 4 - tol)
    return float((l * ok).sum()), float(l.sum())


for D in sys.argv[1:]:
    C = json.load(open(os.path.join(D, 'cu.json'))); Z = json.load(open(os.path.join(D, 'zusatz.json'))); E = json.load(open(os.path.join(D, 'k20.json')))
    vers, gnd = set(E['vers']), set(E['gnd'])
    zf = {}
    for z in Z['zones']:
        if z['fill']: zf.setdefault((z['net'], z['layer']), []).append(geo(z['fill']))
    pads = {}
    for p in C['pads']:
        for L in p.get('layers') or []: pads.setdefault((p['net'], L), []).append(Point(p['x'], p['y']))
    out = {}
    for seite, lagen in (('aussen', [L for L in E['lagen'] if L in ('F', 'B')]), ('innen', [L for L in E['lagen'] if L not in ('F', 'B')])):
        st = []; ok45 = tot = 0.0; ok45_g = tot_g = 0.0
        for (n, L), gs in zf.items():
            if L not in lagen: continue
            g = unary_union(gs)
            for p in teile(g):
                if p.area < 0.01: continue
                a, b = raster45(p)
                if n in vers:
                    ok45 += a; tot += b
                    npads = sum(p.covers(q) for q in pads.get((n, L), []))
                    st.append(dict(net=n, lage=L, a=p.area, breite=2 * p.area / max(1e-9, p.exterior.length), pads=npads,
                                   konvex=p.area / max(1e-9, p.convex_hull.area)))
                elif n in gnd:
                    ok45_g += a; tot_g += b
        fl = sum(E['je_lage'][L]['flaeche_mm2'] for L in lagen)
        A = np.array([s['a'] for s in st]) if st else np.zeros(1)
        out[seite] = dict(stuecke=len(st), median_mm2=round(float(np.median(A)), 2), p90_mm2=round(float(np.percentile(A, 90)), 1), max_mm2=round(float(A.max()), 1),
                          summe_mm2=round(float(A.sum()), 1), anteil_lagenflaeche=round(float(A.sum()) / max(1e-9, fl), 4),
                          breite_median=round(float(np.median([s['breite'] for s in st])), 3) if st else 0,
                          pads_median=float(np.median([s['pads'] for s in st])) if st else 0,
                          ohne_pad=sum(s['pads'] == 0 for s in st), konvex_median=round(float(np.median([s['konvex'] for s in st])), 3) if st else 0,
                          kante45_vers=round(ok45 / max(1e-9, tot), 3), kante45_gnd=round(ok45_g / max(1e-9, tot_g), 3),
                          groesste=[(s['net'], s['lage'], round(s['a'], 1)) for s in sorted(st, key=lambda s: -s['a'])[:4]])
    json.dump(out, open(os.path.join(D, 'k20_form.json'), 'w'), indent=1)
    print(os.path.basename(D), json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'groesste'} for k, v in out.items()}))
    print('   groesste aussen:', out['aussen']['groesste'], ' innen:', out['innen']['groesste'])
