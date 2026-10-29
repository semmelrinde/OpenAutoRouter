import json
import math
import sys
from collections import Counter

import numpy as np
from scipy.spatial import cKDTree
from shapely import contains_xy
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union
from shapely.strtree import STRtree

VERSION = 'via-density-v3'
WINDOW_MM = 2.0
STEP_MM = 0.25


THRESHOLD = 1.25


def physical_vias(R):
    out = {}
    for v in R.get('vias', []):
        x, y = float(v['x']), float(v['y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Non-finite via coordinate')
        key = (round(x, 6), round(y, 6), v.get('net', ''))
        if key in out and out[key].get('layers') != v.get('layers'):
            raise ValueError('Duplicate via with contradictory layer span')
        if key not in out:
            out[key] = dict(v)
        else:


            for mark in ('gr', 'n', 'rv', 'ret_via', 'current_via', 'fix', 'kelvin'):
                if v.get(mark): out[key][mark] = max(out[key].get(mark) or 0, v[mark])
    return [out[k] for k in sorted(out)]


def sharp(xy, window=WINDOW_MM):
    if not len(xy): return 0, None
    xy = np.asarray(xy, dtype=float)
    best, location = 0, None
    for x in np.unique(xy[:, 0]):
        ys = np.sort(xy[(xy[:, 0] >= x) & (xy[:, 0] <= x + window + 1e-9), 1])
        hi = np.searchsorted(ys, ys + window + 1e-9, side='right')
        counts = hi - np.arange(len(ys))
        i = int(np.argmax(counts))
        if int(counts[i]) > best:
            best, location = int(counts[i]), [float(x + window / 2), float(ys[i] + window / 2)]
    return best, location


def measure(B, R, threshold=THRESHOLD, window=WINDOW_MM, step=STEP_MM):
    if not all(math.isfinite(v) and v > 0 for v in (window, step)) or not math.isfinite(threshold) or threshold < 0:
        raise ValueError('Invalid via density measurement contract')
    vias = physical_vias(R)
    xy = np.asarray([(v['x'], v['y']) for v in vias], dtype=float).reshape(-1, 2)
    maximum, location = sharp(xy, window)
    cu_layers = ['F'] + ['In%d' % i for i in range(1, int(B.get('cu_layers') or R.get('cu_layers') or 2) - 1)] + ['B']
    brd = unary_union([Polygon(p) for p in B['outline'] if len(p) >= 3])

    holes = B.get('outline_holes') or []
    rings = []
    for group in holes:
        if not group: continue
        if isinstance(group[0][0], (int, float)): rings.append(group)
        else: rings.extend(group)
    if rings:
        brd = brd.difference(unary_union([Polygon(p) for p in rings if len(p) >= 3]))
    inner = brd.buffer(1e-9)
    x0, y0, x1, y1 = brd.bounds
    xx = np.arange(x0, x1 + 1e-9, step)
    yy = np.arange(y0, y1 + 1e-9, step)
    tree = cKDTree(xy) if len(xy) else None
    hist, hot, n = {}, [], 0

    for y in yy:
        centers = np.column_stack((xx, np.full(len(xx), y)))
        centers = centers[contains_xy(inner, centers[:, 0], centers[:, 1])]
        counts = tree.query_ball_point(centers, window / 2 + 1e-9, p=np.inf, return_length=True) if tree else np.zeros(len(centers), int)
        n += len(centers)
        vals, nums = np.unique(counts, return_counts=True)
        for a, b in zip(vals, nums): hist[int(a)] = hist.get(int(a), 0) + int(b)
        hot.extend(centers[counts > threshold * window ** 2].tolist())


    centers = hot + ([location] if location is not None else [])
    geometries = [box(x - window / 2, y - window / 2, x + window / 2, y + window / 2) for x, y in centers]
    occupied = np.zeros(len(centers), int)
    from common.zones import board_geometry
    from learning.copper_bars import pad_polygon, track_geometry
    BB = board_geometry(B, R)
    for L in cu_layers:
        copper_map = [track_geometry(t) for t in R.get('tracks', []) if t['layer'] == L]
        for p in BB.get('pads', []):
            if p.get('pad_type') == 'npth': continue
            holes = [Polygon(q) for q in p.get('hole_polys', []) if len(q) >= 3]
            if not holes and p.get('drill'):


                holes = [Point(p['x'], p['y']).buffer(p['drill'] / (2 * math.cos(math.pi / 256)), quad_segs=64)]
            hole = unary_union(holes)
            for q in (p.get('polys') or {}).get(L, []):
                if len(q) >= 3:
                    g = pad_polygon(BB, p, q).difference(hole)
                    if not g.is_empty: copper_map.append(g)
        copper_map += [Polygon(z['pts'], z.get('holes') or z.get('hole_list') or []) for z in R.get('zones', [])
                   if z.get('layer') == L and z.get('filled') and len(z.get('pts', [])) >= 3]
        copper_map += [Polygon(q['outer'], q.get('inner') or []) for z in R.get('zone_list', []) if z.get('layer') == L
                   for q in z.get('polys', []) if len(q.get('outer', [])) >= 3]
        if copper_map and geometries:
            pairs = STRtree(copper_map).query(geometries, predicate='intersects')
            occupied[np.unique(pairs[0])] += 1
    spans, nets = set(), Counter()
    if location is not None:
        for i in tree.query_ball_point(location, window / 2 + 1e-9, p=np.inf):
            spans.update(vias[i].get('layers') or cu_layers)
            nets[vias[i].get('net', '')] += 1
    excess = sum(max(0, k - threshold * window ** 2) * v for k, v in hist.items())
    return dict(version=VERSION, window_mm=window, step_mm=step, threshold_vias_mm2=threshold,
                vias=len(vias), sharp_vias=maximum, sharp_vias_mm2=maximum / window ** 2, sharp_xy=location,
                window=n, window_over=len(hot), share_window_over=len(hot) / max(1, n),
                overload_integral_mm2=round(excess * step ** 2, 6),
                sharp_copper_layers_share=float(occupied[-1] / len(cu_layers)) if location is not None else 0.0,
                hot_copper_layers_share=float(np.mean(occupied[:len(hot)]) / len(cu_layers)) if hot else 0.0,
                sharp_via_layers_share=len(spans.intersection(cu_layers)) / len(cu_layers),
                sharp_nets=dict(sorted(nets.items())),
                copper_layers_model='Segment/arc widths, pad contours without drill, filled zones and full-copper model zone_list; without via rings',
                pad_drill_assumption='Circle for missing drill contour' if any(p.get('drill') and not p.get('hole_polys') for p in BB.get('pads', [])) else None,
                via_layers_assumption='Through via for missing layer span' if any(not v.get('layers') for v in vias) else None,
                histogram={str(k): hist[k] for k in sorted(hist)})


def guard_nets(K, R=None, impedance=None):
    from analysis.signal_integrity import spec
    nets = K.get('net_list') or {}
    protected = set((R or {}).get('impedance_table') or {}) | set(impedance or {})
    for group in ((K.get('specs') or {}).get('length_groups') or {}).values():
        protected.update(group['net_list'])
    for n, k in nets.items():
        if k.get('category') not in ('signal', 'analog', 'fast'):
            protected.add(n)


        for field in ('crystal', 'pair', 'z_ohm', 'z_diff_ohm', 'length_group', 'kelvin', 'fix'):
            for value in (k.get(field), spec(K, n, field)):
                if value:
                    protected.add(n)
                    if field == 'pair': protected.add(value)
    return protected


def via_guarded(v, K, net_list=None):
    if net_list is None: net_list = guard_nets(K)
    n = v.get('net')
    return (n in net_list or (K.get('net_list', {}).get(n) or {}).get('category') not in ('signal', 'analog', 'fast')
            or any(v.get(k) for k in ('gr', 'n', 'rv', 'ret_via', 'current_via', 'fix', 'kelvin')))

def short(R, K, window=WINDOW_MM, threshold=THRESHOLD):
    if not math.isfinite(window) or window <= 0 or not math.isfinite(threshold) or threshold < 0:
        raise ValueError('Invalid R40 measurement contract')
    vs = physical_vias(R)
    protected = guard_nets(K, R)
    signal = [v for v in vs if not via_guarded(v, K, protected)]
    def peak(vs): return sharp([(v['x'], v['y']) for v in vs], window)[0] / window ** 2
    xy = np.asarray([(v['x'], v['y']) for v in vs], dtype=float).reshape(-1, 2)


    counts = (cKDTree(xy).query_ball_point([(v['x'], v['y']) for v in signal], window / 2 + 1e-9,
                                         p=np.inf, return_length=True) if signal else np.array([], int))
    return dict(version='r40-short-v2', window_mm=window, threshold_vias_mm2=threshold,
                vias=len(vs), signal_vias=len(signal), guarded_vias=len(vs)-len(signal),
                sharp_vias_mm2=peak(vs), signal_sharp_vias_mm2=peak(signal),
                signal_hot_vias=int(np.sum(counts > threshold * window ** 2)),
                signal_overload=float(np.maximum(0, counts - threshold * window ** 2).sum()),
                exit_blocked=None)


def calibrate(measurements):
    ms = list(measurements)
    if not ms: raise ValueError('No human references')
    if any(m['window_mm'] != WINDOW_MM for m in ms): raise ValueError('Different window')
    peaks = sorted(m['sharp_vias_mm2'] for m in ms)
    return dict(version=VERSION, window_mm=WINDOW_MM, refs=len(ms), method='Board peaks p95 nearest-rank',
                threshold_vias_mm2=peaks[math.ceil(0.95 * len(peaks)) - 1], peaks=peaks)
if __name__ == '__main__':
    B, R = (json.load(open(p)) for p in sys.argv[1:3])
    kw = dict(a.split('=', 1) for a in sys.argv[3:])
    print(json.dumps(measure(B, R, **{k: float(v) for k, v in kw.items()}), indent=1))
