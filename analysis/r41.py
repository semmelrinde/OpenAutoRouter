import math
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, nearest_points


def _difference(a, b):
    try:
        return a.difference(b)
    except shapely.errors.GEOSException:
        return shapely.difference(shapely.make_valid(a), shapely.make_valid(b), grid_size=0.000001)


def net_list(K):
    net_copper = K.get('net_list') or {}
    fast = {n for n, k in net_copper.items() if k.get('category') == 'sw' or
               (k.get('category') == 'fast' and k.get('rank', 0) >= 3)}
    other = {n for n, k in net_copper.items() if k.get('category') not in ('current_val', 'supply', 'gnd')}
    return fast, other - set(K.get('gnd') or [])


def parameter(K, options=None):
    o = dict(K.get('design_rules') or {}); o.update(options or {})
    clr = float(o.get('clr', 0.13))
    via = o.get('via', '0.45/0.20')
    dia = float(via.split('/')[0]) if isinstance(via, str) else float(via[0])
    reserve = float(o.get('r41_reserve', 0.05))

    fill = max(0.15, clr)
    minimum = float(o.get('r41_min_width', 0.25))
    return dict(clr=clr, fill_clr=fill, via_d=dia, reserve=reserve,
                dense=clr + reserve, gap_from=max(clr + reserve, 2 * fill + minimum),
                via_slot=dia + 2 * fill + reserve)


def _segments(R):
    out = {}
    for t in R.get('tracks', []):
        for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            length = math.dist(a, b)
            if length < 1e-9: continue
            w = t['w'][i] if isinstance(t['w'], list) else t['w']
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            out.setdefault(t['layer'], []).append((t['net'], a, b, float(w), (b-a)/length, length))
    return out


def _open(B, R, K, L, fast, h, p):

    copper, rest, stitched = [], [], []
    for net in sorted(K.get('gnd') or ['GND']):
        cu, missing, covered = _open_net(B, R, L, fast, h, p, net)
        copper.append(cu); rest.append(missing); stitched.append(covered)
    return unary_union(copper), unary_union(rest), unary_union(stitched)


def _open_net(B, R, L, fast, h, p, net):
    from learning.current_zones import _geo, ends_stitch
    native = [z for z in R.get('zones', []) if z['net'] == net and z['layer'] == L and z.get('filled') is True]
    cu = unary_union([Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in native] if native else
                     [_geo(z) for z in R.get('zone_list', []) if z['net'] == net and z['layer'] == L])

    from analysis.copper_model import on_layer
    anchors = [((v['x'], v['y']), Point(v['x'], v['y']).buffer(float(v.get('d', v.get('diameter', p['via_d'])))/2))
               for v in R.get('vias', []) if v.get('net') == net and on_layer(v, L)]
    anchors += [((q['x'], q['y']), unary_union([Polygon(g).buffer(0) for g in q['polys'].get(L, [])]))
                for q in B.get('pads', []) if q.get('drill') and q['net'] == net and L in q['layers']]
    near = fast.buffer(1.0)
    rest, stitched = [], []
    for s in getattr(cu, 'geoms', [cu]):
        if s.is_empty or s.geom_type != 'Polygon' or not s.intersects(near): continue
        aa = []
        for a, metal in anchors:
            if not s.intersects(metal): continue


            contact = Point(a) if s.covers(Point(a)) else nearest_points(s, Point(a))[0]
            aa.append((contact.x, contact.y))
        _, missing = ends_stitch(s, s.intersection(near), aa, Polygon(), Polygon(), [], p['via_d'], p['fill_clr'],
                                   r=1.5, r_cut=1.5 + h, h=h)


        broad = s.buffer(-1.0, join_style=2).buffer(1.0, join_style=2).intersection(s)
        if missing is not None:
            rest.append(_difference(missing.intersection(s), broad))
        stitched.append(_difference(_difference(s, broad).intersection(near), missing if missing is not None else Polygon()))
    return cu, unary_union(rest), unary_union(stitched)

def _fill_layers(B, R, K):
    from analysis.areas import native_layers
    native = native_layers(B, R, K)
    gnd = set(K.get('gnd') or ['GND'])
    native.update(z['layer'] for z in R.get('zones', []) if z.get('filled') is True and z.get('net') in gnd)
    geometric = set(R.get('r41_fill_layers') or [])
    geometric.update(z['layer'] for z in R.get('zone_list', []) if z.get('net') in gnd)
    return native | geometric


def measure(B, R, K, step=0.1):
    o = R.get('options') or {}; p = parameter(K, dict(o, **({'clr': R['clr']} if R.get('clr') else {})))
    out = dict(version='r41-v7', active=int(B.get('cu_layers', o.get('cu_layers', 2))) >= 4,
               step_mm=step, parameter=p, a_mm=0., b_mm=0., c_mm=0., d_mm=0., without_mm=0., fast_mm=0.,
               fill_measured=False, vias=sum(bool(v.get('r41')) for v in R.get('vias', [])),
               cut_mm2=(R.get('r41_fill_stat') or {}).get('cut_mm2',
                   round(sum((z.get('r41') or {}).get('cut_mm2', 0) for z in R.get('zone_list', [])), 4)))
    if not out['active']: return dict(out, error_mm=0.)
    fast, non_dc = net_list(K)
    proven = _fill_layers(B, R, K); required = set()
    for L, segs in sorted(_segments(R).items()):
        fs = [s for s in segs if s[0] in fast]
        if not fs: continue
        required.add(L)
        cu, missing, stitched = _open(B, R, K, L, unary_union([LineString([s[1],s[2]]).buffer(s[3]/2) for s in fs]), step, p)
        lines = [LineString([s[1], s[2]]) for s in segs]
        tree = shapely.STRtree(lines)
        for n, a, b, w, u, length in fs:
            out['fast_mm'] += length
            num = max(1, math.ceil(length/step)); dl = length/num
            points = a + (np.arange(num)[:, None]+.5)*dl*u
            neighbor_radius = max(1.0, p['via_slot'])
            ids = tree.query(LineString([a,b]).buffer(neighbor_radius + w/2 + max(s[3] for s in segs)/2))
            candidates = [segs[i] for i in ids if segs[i][0] != n and segs[i][0] in non_dc and abs(np.dot(u,segs[i][4])) >= math.cos(math.radians(10))]
            for xy in points:
                labels = []

                if not missing.is_empty and missing.distance(Point(xy)) <= w/2 + p['fill_clr'] + .25:
                    labels.append('d')
                if not stitched.is_empty and stitched.distance(Point(xy)) <= w/2 + p['fill_clr'] + .25:
                    labels.append('a')
                sides = {}
                for _, aa, bb, ww, uu, ll in candidates:
                    t = float(np.dot(xy-aa,uu))
                    if not 0 <= t <= ll: continue
                    q = aa+t*uu; delta = q-xy
                    gap = float(np.linalg.norm(delta)) - (w+ww)/2
                    if gap < -1e-5 or gap > neighbor_radius: continue
                    side = 1 if u[0]*delta[1]-u[1]*delta[0] > 0 else -1
                    if side not in sides or gap < sides[side][0]: sides[side] = (gap,q,ww)
                for gap,q,ww in sides.values():
                    if gap <= p['dense']+1e-6: labels.append('b'); continue
                    direction = (q-xy)/np.linalg.norm(q-xy)
                    cross = LineString([xy+direction*(w/2+p['fill_clr']), q-direction*(ww/2+p['fill_clr'])])
                    filled = cross.intersection(cu)
                    if not filled.is_empty and filled.length > .01 and filled.intersects(missing): labels.append('d')
                    elif filled.length >= .25 and filled.length >= cross.length-.02: labels.append('a')
                    elif p['gap_from'] + 1e-6 < gap < p['via_slot']: labels.append('c')
                label = next((x for x in ('d','c','b','a') if x in labels), 'without')
                out[label+'_mm'] += dl
    for k in ('a_mm','b_mm','c_mm','d_mm','without_mm','fast_mm'): out[k] = round(out[k], 4)
    out['error_mm'] = round(out['c_mm'] + out['d_mm'], 4)
    out['fill_layers'] = sorted(proven)
    out['fill_missing_layers'] = sorted(required - proven)
    out['fill_measured'] = not bool(required - proven)
    if not out['fill_measured'] and out['fast_mm'] > 0:
        out.update(a_mm=None, d_mm=None, error_mm=None, absent='GND fill not measured')
    return out
