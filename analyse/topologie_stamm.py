""
import collections
import itertools
import math
import heapq

import shapely
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree

from analyse.topologie_native import (
    _measurement_routes,
    _pads,
    _parallel_runs,
    _segmente,
)

VERSION = 'topologie-v8-stamm'
ARM_R = .15
GEOM_GRID = 1e-9


def _kontakte(R):
    ""
    groups = collections.defaultdict(list)
    for t in R['tracks']:
        pts = [tuple(round(v, 9) for v in p) for p in t['pts']]


        if len(set(pts)) < 2:
            continue
        groups[(t['net'], t['layer'])].append(dict(t, pts=pts))
    tracks = []; contacts = {}
    for ts in groups.values():
        source = [(LineString([a, b]), t['w'][min(i, len(t['w'])-1)])
                  for t in ts for i, (a, b) in enumerate(itertools.pairwise(t['pts'])) if a != b]
        gs, widths = zip(*source)
        tree = STRtree(gs)
        paths = [LineString([(round(x, 9), round(y, 9)) for x, y in g.coords])
                 for g in _lines(shapely.line_merge(shapely.union_all(gs, grid_size=GEOM_GRID)))]
        graph = collections.defaultdict(set)
        for g in paths:
            for a, b in itertools.pairwise(g.coords):
                graph[a].add(b); graph[b].add(a)
        targets = []
        for arm, g in enumerate(paths):
            for a, b in itertools.pairwise(g.coords):
                seg = LineString([a, b])
                mid = seg.interpolate(.5, normalized=True)
                w = max(widths[int(j)] for j in tree.query(mid.buffer(2*GEOM_GRID))
                        if mid.distance(gs[int(j)]) <= 2*GEOM_GRID)
                targets.append((arm, seg, w))
        target_tree = STRtree([s for _, s, _ in targets])
        connectors = []
        for arm, g in enumerate(paths):
            for p in (g.coords[0], g.coords[-1]):
                if len(graph[p]) != 1:
                    continue
                point = Point(p)
                own_w = max(w for i, seg, w in targets if i == arm and point.distance(seg) <= 2*GEOM_GRID)


                near = {p: 0.}; pending = [(0., p)]
                while pending:
                    distance, node = heapq.heappop(pending)
                    if distance != near[node]: continue
                    for nxt in graph[node]:
                        step = distance + math.dist(node, nxt)
                        if step <= ARM_R and step < near.get(nxt, float('inf')):
                            near[nxt] = step; heapq.heappush(pending, (step, nxt))
                best = None
                for j in target_tree.query(point.buffer(.03+GEOM_GRID)):
                    other, seg, w = targets[int(j)]
                    d = point.distance(seg)
                    if d > min(.03, (own_w+w)/2)+GEOM_GRID:
                        continue
                    q = tuple(seg.interpolate(seg.project(point)).coords[0])
                    if min(near.get(tuple(e), float('inf')) + math.dist(e, q) for e in seg.coords) <= ARM_R:
                        continue


                    if len(graph.get(q, ())) == 1 and p < q:
                        continue
                    key = (d, q)
                    if best is None or key < best:
                        best = key
                if best is not None and best[0] > 0:



                    original = next(t for t in ts if p in t['pts'])
                    connectors.append(dict(original, pts=[p, best[1]], w=[own_w]))


                    q = tuple(round(v, 9) for v in best[1])
                    contacts[(original['net'], original['layer'], q, p)] = p
        tracks.extend(ts + connectors)
    return dict(R, tracks=tracks, _r27_kontakte=contacts)
def _lines(g):
    if g.geom_type == 'LineString':
        return [g] if g.length > 0 else []
    return [p for part in getattr(g, 'geoms', []) for p in _lines(part)]

def _parallel_paths(geometries):
    ""
    graph = collections.defaultdict(set)
    for g in _lines(shapely.union_all(geometries, grid_size=GEOM_GRID)):
        for a, b in itertools.pairwise(g.coords):
            if a != b:
                graph[a].add(b); graph[b].add(a)
    continuation = {}
    for p, neighbours in sorted(graph.items()):
        candidates = []
        for a, b in itertools.combinations(sorted(neighbours), 2):
            va, vb = (a[0]-p[0], a[1]-p[1]), (b[0]-p[0], b[1]-p[1])
            dot = sum(x*y for x, y in zip(va, vb)) / (math.dist(a, p)*math.dist(b, p))
            turn = math.degrees(math.acos(max(-1., min(1., -dot))))
            if len(neighbours) == 2 or turn <= 15+1e-6:
                candidates.append((turn, a, b))
        used = set()
        for _, a, b in sorted(candidates):
            if a not in used and b not in used:
                continuation[p, a] = b; continuation[p, b] = a
                used.update((a, b))
    edges = {tuple(sorted((p, q))) for p in graph for q in graph[p]}
    starts = sorted((p, q) for p in graph for q in graph[p] if (p, q) not in continuation)

    starts += sorted((p, q) for p, q in edges)
    for p, q in starts:
        if tuple(sorted((p, q))) not in edges:
            continue
        pts = [p]
        while tuple(sorted((p, q))) in edges:
            edges.remove(tuple(sorted((p, q)))); pts.append(q)
            nxt = continuation.get((q, p))
            if nxt is None:
                break
            p, q = q, nxt
        yield pts


def _kreis(p, a, b, r):
    ""
    dx, dy = b[0]-a[0], b[1]-a[1]
    ux, uy = a[0]-p[0], a[1]-p[1]
    aa = dx*dx+dy*dy
    bb = 2*(ux*dx+uy*dy)
    cc = ux*ux+uy*uy-r*r
    t = (-bb+math.sqrt(max(0., bb*bb-4*aa*cc))) / (2*aa)
    return (a[0]+t*dx, a[1]+t*dy)


def messen(R, B=None, bsp_max=8):
    R = _kontakte(_measurement_routes(R))
    pads = _pads(B)
    groups = collections.defaultdict(list)
    par_groups = collections.defaultdict(list)
    for t in R['tracks']:
        if len(t['pts']) < 2:
            continue
        g = LineString(t['pts'])
        if g.length <= 0:
            continue
        key = (t['net'], t['layer'])
        groups[key].append(g)



        art = 'abgriff' if t.get('kelvin') else 'kette' if t.get('kette') else 'rest'
        par_groups[key + (art,)].append(g)
    counts = collections.Counter(); by_net = collections.defaultdict(collections.Counter)
    examples = []
    for (net, layer), geometries in sorted(groups.items()):
        previous = counts.copy()
        graph = collections.defaultdict(set)

        for g in _lines(shapely.union_all(geometries, grid_size=GEOM_GRID)):
            pts = [tuple(q) for q in g.coords]
            for a, b in itertools.pairwise(pts):
                graph[a].add(b)
                graph[b].add(a)
        for p in sorted(graph):
            if len(graph[p]) < 3:
                continue
            directions = []
            for start in sorted(graph[p]):
                origin = R['_r27_kontakte'].get((net, layer, p, start), p)
                a, b = p, start
                seen = {p}

                while math.dist(origin, b) < ARM_R-1e-10 and len(graph[b]) == 2 and b not in seen:
                    seen.add(b)
                    a, b = b, next(q for q in graph[b] if q != a)
                if math.dist(origin, b) < ARM_R-1e-10:


                    q = b
                else:
                    q = _kreis(origin, a, b, ARM_R)
                directions.append(math.degrees(math.atan2(q[1]-origin[1], q[0]-origin[0])) % 360)
            angles = sorted(directions)
            gap = min((angles[(i+1) % len(angles)]-a) % 360 for i, a in enumerate(angles))
            in_pad = any(g.buffer(.03).covers(Point(p)) for g in pads.get((net, layer), []))
            counts['knoten'] += 1
            counts['knoten_pad'] += in_pad
            counts['stern'] += len(angles) > 3
            counts['stern_pad'] += (len(angles) > 3 and in_pad)
            counts['spitz'] += gap < 60-1e-6
            counts['spitz_pad'] += (gap < 60-1e-6 and in_pad)
            if len(angles) > 3 or gap < 60-1e-6:
                examples.append((len(angles), round(gap, 3), net, layer, p, 'im Pad' if in_pad else 'frei'))
        by_net[net].update(counts - previous)



    canonical = dict(R, tracks=[])



    unique = {}
    for key in sorted(groups):
        covered = None
        for art in ('rest', 'kette', 'abgriff'):
            geometries = par_groups.get(key + (art,), [])
            if not geometries: continue
            geometry = shapely.union_all(geometries, grid_size=GEOM_GRID)
            remainder = geometry if covered is None else geometry.difference(covered)
            unique[key + (art,)] = _lines(remainder)
            covered = geometry if covered is None else shapely.union_all([covered, geometry], grid_size=GEOM_GRID)
    for (net, layer, art), geometries in sorted(unique.items()):
        for pts in _parallel_paths(geometries):
            canonical['tracks'].append({'net': net, 'layer': layer, 'pts': pts, 'w': [.1]*(len(pts)-1),
                                        'sig': art == 'abgriff', 'kette': art == 'kette'})
    parallel = collections.Counter()
    parallel_nets = collections.Counter()
    parallel_examples = []
    for (net, layer), S in sorted(_segmente(canonical).items()):
        for ti, tj, art, length in _parallel_runs(S, canonical, flat_segments=True, self_pairs=True):
            parallel[art] += length
            parallel_nets[net] += length
            by_net[net]['parallel_'+art+'_mm'] += length
            a, b = canonical['tracks'][ti], canonical['tracks'][tj]
            parallel_examples.append((round(length, 2), art, net, layer,
                [v for p in (a['pts'][0], a['pts'][-1]) for v in p],
                [v for p in (b['pts'][0], b['pts'][-1]) for v in p],
                round(LineString(a['pts']).distance(LineString(b['pts'])), 3)))
    out = {k: counts[k] for k in ('knoten', 'knoten_pad', 'stern', 'stern_pad', 'spitz', 'spitz_pad')}
    out.update(version=VERSION, bsp_knoten=examples[:bsp_max],
               parallel_netze={n: round(v, 1) for n, v in sorted(parallel_nets.items())},
               bsp_parallel=parallel_examples[:bsp_max],
               **{'parallel_'+k+'_mm': round(parallel[k], 1) for k in ('rest', 'abgriff', 'kette')})



    normal = dict(R, tracks=[])
    for net, layer in sorted(groups):
        gs = unique.get((net, layer, 'rest'), []) + unique.get((net, layer, 'kette'), [])
        for pts in _parallel_paths(gs):
            normal['tracks'].append(dict(net=net, layer=layer, pts=pts, w=[.1]*(len(pts)-1)))
    r27_parallel = collections.Counter()
    for (net, layer), S in sorted(_segmente(normal).items()):
        for _, _, _, length in _parallel_runs(S, normal, flat_segments=True, self_pairs=True):
            r27_parallel[net] += length
    out['parallel_r27_mm'] = round(sum(r27_parallel.values()), 1)
    out['wert'] = round(out['stern']-out['stern_pad']+out['spitz']-out['spitz_pad']+out['parallel_r27_mm'], 1)
    out['netze'] = {net: dict(stern_frei=c['stern']-c['stern_pad'], spitz_frei=c['spitz']-c['spitz_pad'],
                              parallel_rest_mm=round(c['parallel_rest_mm'], 1),
                              parallel_r27_mm=round(r27_parallel[net], 1)) for net, c in sorted(by_net.items())}
    return out
