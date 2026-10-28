""
import math, collections
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union
from shapely import STRtree

VIA_R = 0.225


def via_radius(v):
    ""
    return float(v.get('d', 2 * VIA_R)) / 2


def padgeo(p):
    ""
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
    ""
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


class NetzgraphCache:
    ""
    def __init__(self, endpunkte=False): self.graphen = {}; self._besitzer = {}; self.endpunkte = endpunkte
    def graph(self, tracks, vias, pads, tol=0.03, endpunkte=None):
        ep = self.endpunkte if endpunkte is None else endpunkte
        key = (tuple(map(id, tracks)), tuple(map(id, vias)), tuple(map(id, pads)), tol, ep)
        if key not in self.graphen:
            self.graphen[key] = netzgraph(tracks, vias, pads, tol, endpunkte=ep)
            self._besitzer[key] = (tuple(tracks), tuple(vias), tuple(pads))
        return self.graphen[key]


def netzgraph(tracks, vias, pads, tol=0.03, endpunkte=False):
    ""
    def nk(L, x, y): return (L, round(x / tol) * tol, round(y / tol) * tol)
    lagen = sorted({t['layer'] for t in tracks})
    ends = collections.defaultdict(list)
    for t in tracks:
        for q in t['pts']: ends[t['layer']].append(tuple(q))
    for v in vias:
        for L in lagen: ends[L].append((v['x'], v['y']))
    geo = [padgeo(p) for p in pads]
    segs = collections.defaultdict(list); segordnung = collections.defaultdict(list); punkte = collections.defaultdict(list)
    for ti, t in enumerate(tracks):
        L = t['layer']
        segordnung[L].extend((ti, si) for si in range(len(t['pts']) - 1))
        segs[L].extend(zip(t['pts'][:-1], t['pts'][1:], t['w']))
        punkte[L].extend((t['pts'][0], t['pts'][-1]) + (() if endpunkte else tuple(t['pts'][1:-1])))
    segtree = {L: STRtree([box(min(a[0], b[0]) - w/2 - tol, min(a[1], b[1]) - w/2 - tol,
                             max(a[0], b[0]) + w/2 + tol, max(a[1], b[1]) + w/2 + tol) for a, b, w in S]) for L, S in segs.items()}
    pttree = {L: STRtree([Point(q) for q in Q]) for L, Q in punkte.items()}
    vitree = STRtree([Point(v['x'], v['y']).buffer(via_radius(v) + tol).envelope for v in vias])
    teilung = [[] for _ in pads]
    for pi, p in enumerate(pads):
        if endpunkte: continue
        G = geo[pi]; x0, y0, x1, y1 = p['bbox']
        kandidaten = [(*segordnung[L][int(sj)], L, int(sj)) for L in set(p['layers']) & set(G) & set(segtree)
                      for sj in segtree[L].query(box(x0, y0, x1, y1))]
        for _, _, L, sj in sorted(kandidaten):
                a, b, w = segs[L][sj]
                if G[L].distance(LineString([a, b])) > w / 2 + tol: continue
                if G[L].distance(Point(a)) <= tol or G[L].distance(Point(b)) <= tol: continue
                d, ss = _seg_dist((p['x'], p['y']), a, b)
                q = (a[0] + ss * (b[0] - a[0]), a[1] + ss * (b[1] - a[1]))
                ends[L].append(q); teilung[pi].append((L, q))
    endtree = {L: STRtree([Point(q) for q in Q]) for L, Q in ends.items()}
    adj = collections.defaultdict(list)
    def kante(a, b, kid):
        adj[a].append((b, kid)); adj[b].append((a, kid))
    for ti, t in enumerate(tracks):
        L = t['layer']
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) < 1e-9: continue
            teil = [(0.0, tuple(a)), (1.0, tuple(b))]
            bb = box(min(a[0], b[0]) - tol, min(a[1], b[1]) - tol, max(a[0], b[0]) + tol, max(a[1], b[1]) + tol)
            for qi in sorted(endtree[L].query(bb)):
                q = ends[L][int(qi)]
                d, s = _seg_dist(q, a, b)
                if d < tol and 1e-6 < s < 1 - 1e-6: teil.append((s, q))
            teil.sort()
            for (s1, p1), (s2, p2) in zip(teil[:-1], teil[1:]):
                if math.dist(p1, p2) < 1e-9: continue
                kante(nk(L, *p1), nk(L, *p2), ('s', ti, si, s1, s2))
    for vi, v in enumerate(vias):
        ks = [nk(L, v['x'], v['y']) for L in lagen]
        for k in ks[1:]: kante(ks[0], k, ('v', vi))
    padk = {}
    for pi, p in enumerate(pads):
        pk = ('PAD', p['ref'], p['num']); padk['%s:%s' % (p['ref'], p['num'])] = pk
        G = geo[pi]; x0, y0, x1, y1 = p['bbox']
        for L in p['layers']:
            if L not in G: continue
            if L not in pttree: continue
            for qi in sorted(pttree[L].query(box(x0 - tol, y0 - tol, x1 + tol, y1 + tol))):
                q = punkte[L][int(qi)]
                if G[L].distance(Point(q)) <= tol if not endpunkte else G[L].covers(Point(q)):
                    kante(pk, nk(L, *q), None)
        for L, q in teilung[pi]: kante(pk, nk(L, *q), None)
        for vi in sorted(vitree.query(box(x0, y0, x1, y1))):
            v = vias[int(vi)]; rand = via_radius(v) + tol
            if x0 - rand <= v['x'] <= x1 + rand and y0 - rand <= v['y'] <= y1 + rand and \
                    min(G[L].distance(Point(v['x'], v['y'])) for L in p['layers'] if L in G) <= (tol if endpunkte else via_radius(v) + tol):
                for L in lagen: kante(pk, nk(L, v['x'], v['y']), None)
    padtree = STRtree([box(*p['bbox']) for p in pads])
    for i, p in enumerate(pads):
        for j in sorted(int(j) for j in padtree.query(box(*p['bbox'])) if int(j) > i):
            q = pads[j]
            gem = set(p['layers']) & set(q['layers']) & set(geo[i]) & set(geo[j])
            if gem and p['bbox'][0] <= q['bbox'][2] and q['bbox'][0] <= p['bbox'][2] and p['bbox'][1] <= q['bbox'][3] \
                    and q['bbox'][1] <= p['bbox'][3] and any(geo[i][L].distance(geo[j][L]) <= 1e-6 for L in gem):
                kante(padk['%s:%s' % (p['ref'], p['num'])], padk['%s:%s' % (q['ref'], q['num'])], None)
    return dict(adj=adj, padk=padk, lagen=lagen)


def analyse(tracks, vias, pads, fluss, tol=0.03, graph=None, endpunkte=False):
    ""
    if not fluss or not tracks: return None
    quelle = set(fluss.get('quelle') or []); last = fluss.get('last') or {}
    g = graph if graph is not None else netzgraph(tracks, vias, pads, tol, endpunkte=endpunkte); adj = g['adj']; padk = g['padk']
    starts = [padk[q] for q in sorted(quelle) if q in padk and padk[q] in adj]
    if not starts: return None

    parent = {s: None for s in starts}; pseg = {}; order = list(starts); i = 0
    while i < len(order):
        n = order[i]; i += 1
        for m, kid in adj[n]:
            if m in parent: continue
            parent[m] = n; pseg[m] = (kid[1], kid[2]) if kid and kid[0] == 's' else None; order.append(m)
    unten = collections.defaultdict(float)
    for k, pk in padk.items():
        if pk in parent and k not in quelle: unten[pk] += last.get(k, 0.0)
    breite = {(ti, si): w for ti, t in enumerate(tracks) for si, w in enumerate(t['w'])}
    wmax_unten = collections.defaultdict(float)
    for n in reversed(order):
        p = parent[n]
        if p is None: continue
        unten[p] += unten[n]
        sid = pseg[n]
        wn = max(wmax_unten[n], breite.get(sid, 0.0) if sid else 0.0)
        wmax_unten[p] = max(wmax_unten[p], wn)
    gesamt = sum(unten[s] for s in starts) or 1.0
    anteil = {}; unten_max = {}
    for n in order:
        sid = pseg.get(n)
        if sid is None: continue
        anteil[sid] = max(anteil.get(sid, 0.0), unten[n] / gesamt)
        unten_max[sid] = max(unten_max.get(sid, 0.0), wmax_unten[n])
    erreicht = sum(1 for k, pk in padk.items() if pk in parent)
    return dict(anteil=anteil, unten_max=unten_max, gesamt=gesamt, pads_erreicht=erreicht, pads=len(padk))


def zielbreite(anteil, wmin, wmax, exp=0.5):
    ""
    return wmin + (wmax - wmin) * (max(0.0, min(1.0, anteil)) ** exp)


def verengung(tracks, info):
    ""
    if not info: return 0.0, []
    s = 0.0; stellen = []
    for ti, t in enumerate(tracks):
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            um = info['unten_max'].get((ti, si), 0.0)
            if um > w + 0.026:
                d = (um - w) * math.dist(a, b); s += d
                stellen.append((round(d, 3), ti, si, round(w, 3), round(um, 3), [round(c, 2) for c in a]))
    stellen.sort(reverse=True)
    return s, stellen
