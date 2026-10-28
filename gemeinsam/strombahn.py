""
import math
from shapely.geometry import LineString, Point


def sollbreite(k, innen=False):
    ""
    base = float(k.get('w_innen' if innen else 'w') or k.get('w') or 0.3)
    if str(k.get('w_quelle', '')).startswith('vorgabe'):
        return base
    return max(base, float(k.get('w_ipc') or 0) * (2.3 if innen else 1),
               float(k.get('w_budget') or 0) * (2 if innen else 1))


def korrigiere(tracks, vorher, lage, hindernisse, reserve=0.005):
    ""
    count = 0
    for t, old in zip(tracks, vorher):
        if t['layer'] != lage: continue
        widths = list(t['w'])
        for i, (a, b, w, floor) in enumerate(zip(t['pts'][:-1], t['pts'][1:], widths, old['w'])):
            if w <= floor + 1e-9: continue
            line = LineString([a, b]) if a != b else Point(a)
            cap = w
            for g, clr in hindernisse:
                d = line.distance(g)
                if d - w / 2 < clr - 1e-6:
                    cap = min(cap, 2 * (d - clr - reserve))
            if cap < w - 1e-9:
                widths[i] = max(floor, math.floor(max(0, cap) * 40) / 40)
                count += 1
        t['w'] = widths
    return count
