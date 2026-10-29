import math, collections
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union
from shapely import STRtree

VIA_R = 0.225


def via_radius(v):
    return float(v.get('d', 2 * VIA_R)) / 2


def pad_geo(p):
    g = {}
    for L, qs in (p.get('polys') or {}).items():
        ps = [Polygon(q).buffer(0) for q in qs if len(q) >= 3]
        if ps: g[L] = unary_union(ps)
    for L in p.get('layers', []):
        if L not in g: g[L] = box(*p['bbox'])
    return g


def _seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy), t


def _seg_box(a, b, x0, y0, x1, y1):
    t0, t1 = 0.0, 1.0; dx, dy = b[0] - a[0], b[1] - a[1]
    for p_, q_ in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p_) < 1e-12:
            if q_ < 0: return False
        else:
            r = q_ / p_
            if p_ < 0: t0 = max(t0, r)
            else: t1 = min(t1, r)
            if t0 > t1: return False
    return True


class NetGraphCache:
    def __init__(self, endpoints=False): self.graphs = {}; self._owner = {}; self.endpoints = endpoints
    def graph(self, tracks, vias, pads, tol=0.03, endpoints=None):
        ep = self.endpoints if endpoints is None else endpoints
        key = (tuple(map(id, tracks)), tuple(map(id, vias)), tuple(map(id, pads)), tol, ep)
        if key not in self.graphs:
            self.graphs[key] = net_graph(tracks, vias, pads, tol, endpoints=ep)
            self._owner[key] = (tuple(tracks), tuple(vias), tuple(pads))
        return self.graphs[key]


def net_graph(tracks, vias, pads, tol=0.03, endpoints=False):
    def net_copper(L, x, y): return (L, round(x / tol) * tol, round(y / tol) * tol)
    cu_layers = sorted({t['layer'] for t in tracks})
    ends = collections.defaultdict(list)
    for t in tracks:
        for q in t['pts']: ends[t['layer']].append(tuple(q))
    for v in vias:
        for L in cu_layers: ends[L].append((v['x'], v['y']))
    geo = [pad_geo(p) for p in pads]
    segs = collections.defaultdict(list); seg_order = collections.defaultdict(list); points = collections.defaultdict(list)
    for ti, t in enumerate(tracks):
        L = t['layer']
        seg_order[L].extend((ti, si) for si in range(len(t['pts']) - 1))
        segs[L].extend(zip(t['pts'][:-1], t['pts'][1:], t['w']))
        points[L].extend((t['pts'][0], t['pts'][-1]) + (() if endpoints else tuple(t['pts'][1:-1])))
    segtree = {L: STRtree([box(min(a[0], b[0]) - w/2 - tol, min(a[1], b[1]) - w/2 - tol,
                             max(a[0], b[0]) + w/2 + tol, max(a[1], b[1]) + w/2 + tol) for a, b, w in S]) for L, S in segs.items()}
    pttree = {L: STRtree([Point(q) for q in Q]) for L, Q in points.items()}
    vitree = STRtree([Point(v['x'], v['y']).buffer(via_radius(v) + tol).envelope for v in vias])
    split = [[] for _ in pads]
    for pi, p in enumerate(pads):
        if endpoints: continue
        G = geo[pi]; x0, y0, x1, y1 = p['bbox']
        candidates = [(*seg_order[L][int(sj)], L, int(sj)) for L in set(p['layers']) & set(G) & set(segtree)
                      for sj in segtree[L].query(box(x0, y0, x1, y1))]
        for _, _, L, sj in sorted(candidates):
                a, b, w = segs[L][sj]
                if G[L].distance(LineString([a, b])) > w / 2 + tol: continue
                if G[L].distance(Point(a)) <= tol or G[L].distance(Point(b)) <= tol: continue
                d, ss = _seg_dist((p['x'], p['y']), a, b)
                q = (a[0] + ss * (b[0] - a[0]), a[1] + ss * (b[1] - a[1]))
                ends[L].append(q); split[pi].append((L, q))
    endtree = {L: STRtree([Point(q) for q in Q]) for L, Q in ends.items()}
    adj = collections.defaultdict(list)
    def edge(a, b, kid):
        adj[a].append((b, kid)); adj[b].append((a, kid))
    for ti, t in enumerate(tracks):
        L = t['layer']
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) < 1e-9: continue
            part = [(0.0, tuple(a)), (1.0, tuple(b))]
            bb = box(min(a[0], b[0]) - tol, min(a[1], b[1]) - tol, max(a[0], b[0]) + tol, max(a[1], b[1]) + tol)
            for qi in sorted(endtree[L].query(bb)):
                q = ends[L][int(qi)]
                d, s = _seg_dist(q, a, b)
                if d < tol and 1e-6 < s < 1 - 1e-6: part.append((s, q))
            part.sort()
            for (s1, p1), (s2, p2) in zip(part[:-1], part[1:]):
                if math.dist(p1, p2) < 1e-9: continue
                edge(net_copper(L, *p1), net_copper(L, *p2), ('s', ti, si, s1, s2))
    for vi, v in enumerate(vias):
        ks = [net_copper(L, v['x'], v['y']) for L in cu_layers]
        for k in ks[1:]: edge(ks[0], k, ('v', vi))
    pad_pts = {}
    for pi, p in enumerate(pads):
        pk = ('PAD', p['ref'], p['num']); pad_pts['%s:%s' % (p['ref'], p['num'])] = pk
        G = geo[pi]; x0, y0, x1, y1 = p['bbox']
        for L in p['layers']:
            if L not in G: continue
            if L not in pttree: continue
            for qi in sorted(pttree[L].query(box(x0 - tol, y0 - tol, x1 + tol, y1 + tol))):
                q = points[L][int(qi)]
                if G[L].distance(Point(q)) <= tol if not endpoints else G[L].covers(Point(q)):
                    edge(pk, net_copper(L, *q), None)
        for L, q in split[pi]: edge(pk, net_copper(L, *q), None)
        for vi in sorted(vitree.query(box(x0, y0, x1, y1))):
            v = vias[int(vi)]; border = via_radius(v) + tol
            if x0 - border <= v['x'] <= x1 + border and y0 - border <= v['y'] <= y1 + border and \
                    min(G[L].distance(Point(v['x'], v['y'])) for L in p['layers'] if L in G) <= (tol if endpoints else via_radius(v) + tol):
                for L in cu_layers: edge(pk, net_copper(L, v['x'], v['y']), None)
    padtree = STRtree([box(*p['bbox']) for p in pads])
    for i, p in enumerate(pads):
        for j in sorted(int(j) for j in padtree.query(box(*p['bbox'])) if int(j) > i):
            q = pads[j]
            gem = set(p['layers']) & set(q['layers']) & set(geo[i]) & set(geo[j])
            if gem and p['bbox'][0] <= q['bbox'][2] and q['bbox'][0] <= p['bbox'][2] and p['bbox'][1] <= q['bbox'][3] \
                    and q['bbox'][1] <= p['bbox'][3] and any(geo[i][L].distance(geo[j][L]) <= 1e-6 for L in gem):
                edge(pad_pts['%s:%s' % (p['ref'], p['num'])], pad_pts['%s:%s' % (q['ref'], q['num'])], None)
    return dict(adj=adj, pad_pts=pad_pts, cu_layers=cu_layers)


def analysis(tracks, vias, pads, flow, tol=0.03, graph=None, endpoints=False):
    if not flow or not tracks: return None
    source = set(flow.get('source') or []); load = flow.get('load') or {}
    g = graph if graph is not None else net_graph(tracks, vias, pads, tol, endpoints=endpoints); adj = g['adj']; pad_pts = g['pad_pts']
    starts = [pad_pts[q] for q in sorted(source) if q in pad_pts and pad_pts[q] in adj]
    if not starts: return None

    parent = {s: None for s in starts}; pseg = {}; order = list(starts); i = 0
    while i < len(order):
        n = order[i]; i += 1
        for m, kid in adj[n]:
            if m in parent: continue
            parent[m] = n; pseg[m] = (kid[1], kid[2]) if kid and kid[0] == 's' else None; order.append(m)
    bottom = collections.defaultdict(float)
    for k, pk in pad_pts.items():
        if pk in parent and k not in source: bottom[pk] += load.get(k, 0.0)
    width = {(ti, si): w for ti, t in enumerate(tracks) for si, w in enumerate(t['w'])}
    wmax_bottom = collections.defaultdict(float)
    for n in reversed(order):
        p = parent[n]
        if p is None: continue
        bottom[p] += bottom[n]
        sid = pseg[n]
        wn = max(wmax_bottom[n], width.get(sid, 0.0) if sid else 0.0)
        wmax_bottom[p] = max(wmax_bottom[p], wn)
    total = sum(bottom[s] for s in starts) or 1.0
    share = {}; bottom_max = {}
    for n in order:
        sid = pseg.get(n)
        if sid is None: continue
        share[sid] = max(share.get(sid, 0.0), bottom[n] / total)
        bottom_max[sid] = max(bottom_max.get(sid, 0.0), wmax_bottom[n])
    reached = sum(1 for k, pk in pad_pts.items() if pk in parent)
    return dict(share=share, bottom_max=bottom_max, total=total, pads_reached=reached, pads=len(pad_pts))


def goal_width(share, wmin, wmax, exp=0.5):
    return wmin + (wmax - wmin) * (max(0.0, min(1.0, share)) ** exp)


def narrowing(tracks, info):
    if not info: return 0.0, []
    s = 0.0; sites = []
    for ti, t in enumerate(tracks):
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            um = info['bottom_max'].get((ti, si), 0.0)
            if um > w + 0.026:
                d = (um - w) * math.dist(a, b); s += d
                sites.append((round(d, 3), ti, si, round(w, 3), round(um, 3), [round(c, 2) for c in a]))
    sites.sort(reverse=True)
    return s, sites
