import math
from shapely.geometry import LineString, Point


def nominal_width(k, inner=False):
    base = float(k.get('w_inner' if inner else 'w') or k.get('w') or 0.3)
    if str(k.get('w_source', '')).startswith('spec'):
        return base
    return max(base, float(k.get('w_ipc') or 0) * (2.3 if inner else 1),
               float(k.get('w_budget') or 0) * (2 if inner else 1))


def correct(tracks, before, layer_name, obstacles, reserve=0.005):
    count = 0
    for t, old in zip(tracks, before):
        if t['layer'] != layer_name: continue
        widths = list(t['w'])
        for i, (a, b, w, floor) in enumerate(zip(t['pts'][:-1], t['pts'][1:], widths, old['w'])):
            if w <= floor + 1e-9: continue
            line = LineString([a, b]) if a != b else Point(a)
            cap = w
            for g, clr in obstacles:
                d = line.distance(g)
                if d - w / 2 < clr - 1e-6:
                    cap = min(cap, 2 * (d - clr - reserve))
            if cap < w - 1e-9:
                widths[i] = max(floor, math.floor(max(0, cap) * 40) / 40)
                count += 1
        t['w'] = widths
    return count
