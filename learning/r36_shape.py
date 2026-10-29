import math

from shapely.geometry import Point
from shapely.ops import unary_union


def transitions(R, layer, net, grow):
    segments, widths = [], {}
    for t in R.get('tracks', []):
        if t['layer'] != layer or t['net'] != net or t.get('kelvin') or t.get('tap'):
            continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            a, b = tuple(a), tuple(b)
            if a == b: continue
            segments.append((a, b, float(w)))
            for p in (a, b): widths[p] = max(widths.get(p, 0), float(w))

    import heapq
    adj = {p: [] for p in widths}
    for a, b, _ in segments:
        d = math.dist(a, b)
        adj[a].append((b, d)); adj[b].append((a, d))
    heap = [(-w, p) for p, w in widths.items()]
    heapq.heapify(heap)
    while heap:
        neg, a = heapq.heappop(heap)
        if -neg < widths[a]-1e-12: continue
        for b, d in adj[a]:
            w = widths[a]-d
            if w > widths[b]+1e-12:
                widths[b] = w; heapq.heappush(heap, (-w, b))
    out = []
    for a, b, w in segments:
        d = math.dist(a, b)
        wa, wb = widths[a], widths[b]
        if max(wa, wb) <= w+1e-10: continue

        positions = sorted({0., d, min(d, wa-w), max(0., d-(wb-w)),
                            max(0., min(d, (wa-wb+d)/2))})
        circles = []
        profile = []
        for s in positions:
            p = (a[0]+(b[0]-a[0])*s/d, a[1]+(b[1]-a[1])*s/d)
            width = max(w, wa-s, wb-(d-s))
            profile.append(width)
            circles.append(Point(p).buffer(width/2+grow, quad_segs=32))
        out.extend(unary_union([x, y]).convex_hull for x, y, wx, wy in
                   zip(circles, circles[1:], profile, profile[1:]) if max(wx, wy) > w+1e-10)
    return unary_union(out)
