import copy
import math
import os

import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union


def replacement_routes(B, R, K):
    from learning.copper_bars import copper_map, POWER
    cu = copper_map(B, R)
    out = dict(R, tracks=[])
    for t in R['tracks']:
        if K['net_list'].get(t['net'], {}).get('category') not in POWER or t.get('sig') or t.get('kelvin'):
            out['tracks'].append(t)
            continue
        g = cu.get((t['net'], t['layer']))
        new = dict(t, w=[], copper_j_min=[])
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            d = math.dist(a, b)
            if d < 1e-9 or g is None:
                new['w'].append(w)
                new['copper_j_min'].append(w)
                continue


            unit = (np.asarray(b) - a) / d
            vertices = shapely.get_coordinates(g)
            events = (vertices - a) @ unit
            breaks = np.unique(np.r_[0., d, events[(events > 0) & (events < d)]])
            lengths = np.diff(breaks)
            valid = lengths > 1e-10
            lo, hi, lengths = breaks[:-1][valid], breaks[1:][valid], lengths[valid]
            eps = np.minimum(lengths * 1e-6, 1e-8)
            positions = np.stack([lo + eps, hi - eps], axis=1).ravel()
            xy = np.asarray(a) + positions[:, None] * unit
            normal = np.array([-unit[1], unit[0]]) * (math.hypot(g.bounds[2]-g.bounds[0], g.bounds[3]-g.bounds[1]) + 1)
            cuts = shapely.intersection(shapely.linestrings(np.stack([xy-normal, xy+normal], axis=1)), g)
            widths = np.asarray([max((c.length for c in getattr(cut, 'geoms', [cut])
                                     if c.distance(Point(p)) < 1e-7), default=w)
                                 for p, cut in zip(xy, cuts)]).reshape(-1, 2)
            widths = np.maximum(widths, 1e-9)
            left, right = widths.T
            inverse = 1 / left
            changing = np.abs(right-left) > 1e-9
            inverse[changing] = np.log(right[changing]/left[changing]) / (right[changing]-left[changing])
            new['w'].append(float(lengths.sum() / np.sum(lengths * inverse)))
            new['copper_j_min'].append(float(widths.min()))
        out['tracks'].append(new)
    out['power_areas_mm2'] = {n: sum(g.area for (nn, _), g in cu.items() if nn == n)
                                   for n, k in K['net_list'].items() if k.get('category') in POWER}
    return out


def ir_values(B, R, K, res=0.1, maps=None):
    from common.zones import unify, board_geometry, via_geometry
    B = board_geometry(B, R)
    R = via_geometry(unify(R))
    from codex.zones_ir.solver_fast import ir_metrics
    from analysis.measures import _current, structure
    from learning.copper_bars import cu_layers, pad_polygon
    from common.arc import points
    rr = dict(R, copper_bars=R.get('copper_bars') or {'model':'PowerAreas'}, tracks=[])
    for t in R['tracks']:


        if 'items' not in t:
            rr['tracks'].append(t)
            continue

        for it in t['items']:
            ps, w = (it[1:3], it[3]) if it[0] == 'L' else (points(*it[1:4]), it[4])
            rr['tracks'].append(dict(net=t['net'], layer=t['layer'], pts=ps, w=[w] * (len(ps) - 1)))
    kk = copy.deepcopy(K)
    for n, k in kk['net_list'].items():
        fl = k.get('flow') or {}
        load = fl.get('load') or {}
        total = sum(float(q) for q in load.values())
        if fl.get('unit') in (None, 'relative') and total > 0:
            i, source = _current(n, K)
            fl.update(load={p: float(q) * i / total for p, q in load.items()}, unit='A',
                      provenance={p: 'current_A/' + str(source) for p in load})
    bb = dict(B, pads=[dict(p, polys={l: [list(pad_polygon(B, p, q).exterior.coords[:-1]) for q in ps]
                                     for l, ps in p.get('polys', {}).items()}) for p in B['pads']])
    if not B.get('stackup'):

        cons = (K.get('specs') or {}).get('structure') or {}
        a = structure(len(cu_layers(B)), R.get('gnd_layers') or [], diel=cons.get('dielectrics'))
        bb['stackup'] = {l: dict(copper_um=q['t'] * 1000, z_mm=q['z']) for l, q in a['cu_layers'].items()}
    if maps:
        from pathlib import Path
        from codex.zones_ir.solver_fast import _analysis
        Path(maps).mkdir(parents=True, exist_ok=True)
        raw = _analysis(bb, rr, kk, res=res, outdir=Path(maps))
    else:
        raw = ir_metrics(bb, rr, kk, res=res)
    solved = {n: q for n, q in raw['net_list'].items() if q.get('status') == 'solved'}
    return dict(status=raw['status'], loss_mW=1000 * sum(q['power_W'] for q in solved.values()),
                loss_per_net_mW={n: q['power_W'] * 1000 for n, q in solved.items()},
                max_drop_mV={n: q['max_drop_V'] * 1000 for n, q in solved.items()},
                current_per_net_A={n: q['total_current_A'] for n, q in solved.items()},
                error_nets=[n for n, q in raw['net_list'].items() if q.get('status') == 'error'],
                error_per_net={n: q['error'] for n, q in raw['net_list'].items() if q.get('status') == 'error'},
                J_max_A_mm2={n:q.get('J_max_A_mm2') for n,q in solved.items()},
                J_x={n:q.get('J_x') for n,q in solved.items()},
                J_max_location={n:q.get('J_max_location') for n,q in solved.items()},
                J_hist={n:q.get('J_hist') for n,q in solved.items()},
                drop_per_pad_mV={n:{p:1000*v for p,v in q['drop_V'].items()} for n,q in solved.items()},
                raster_mm=res, run_duration_s=raw['run_duration_s'], model='filled-Polygons/JLC-Stackup/DC-Raster',
                current_mode_per_net={n: q.get('current_mode') for n, q in solved.items()},
                maps={n: q.get('maps_npz') for n, q in solved.items()} if maps else None)


def unconnected_polygons(B, R, GND):
    from learning.copper_bars import copper_map, cu_layers, via_size, pad_polygon
    from collections import defaultdict
    from shapely import STRtree
    copper = copper_map(B, R, unfilled='ignore')
    vd, _ = via_size(R)
    by_net = defaultdict(list)
    for p in B['pads']:
        if p['net'] and p['net'] not in GND:
            by_net[p['net']].append(p)
    missing = []
    for net, pads in sorted(by_net.items()):
        if len({(p['ref'], p['num']) for p in pads}) < 2: continue
        gs, indices, trees, parent = {}, {}, {}, {}
        for layer in cu_layers(B):
            g = copper.get((net, layer))
            parts = list(getattr(g, 'geoms', [g])) if g is not None else []
            gs[layer] = [p for p in parts if not p.is_empty]
            indices[layer] = [(layer, i) for i in range(len(gs[layer]))]
            trees[layer] = STRtree(gs[layer])
            parent.update({q:q for q in indices[layer]})
        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x
        def union(keys):
            for q in keys[1:]: parent[find(q)] = find(keys[0])
        def contact(layer, g):

            out = []
            for i in trees[layer].query(g, predicate='intersects'):
                meet = g.intersection(gs[layer][int(i)])
                if meet.area > 1e-10 or meet.length > 1e-8:
                    out.append(indices[layer][int(i)])
            return out
        for v in R['vias']:
            if v['net'] != net: continue
            disk = Point(v['x'], v['y']).buffer(v.get('d', v.get('diameter', vd))/2, quad_segs=24)
            keys = [q for layer in v.get('layers') or cu_layers(B) if layer in trees for q in contact(layer, disk)]
            union(keys)
        pad_roots = []
        for pad in pads:
            keys = [q for layer, polygons in pad.get('polys', {}).items() if layer in trees
                    for p in polygons for q in contact(layer, pad_polygon(B, pad, p))]

            if pad.get('drill'): union(keys)
            pad_roots.append(keys)
        sets = [{find(q) for q in keys} for keys in pad_roots]
        if not sets or any(not q for q in sets) or not set.intersection(*sets): missing.append(net)
    return missing


def current_density_areas(tracks, tree_idx, A, net, pads, current_A=None, ipc_model=None):
    from collections import defaultdict
    from analysis.measures import current_density
    from shapely.geometry import LineString, box
    from shapely.ops import unary_union
    from analysis.measures import J_NECK_D, J_NECK_L
    pad_regions = {L: unary_union([box(p['bbox'][0]-J_NECK_D, p['bbox'][1]-J_NECK_D,
                                     p['bbox'][2]+J_NECK_D, p['bbox'][3]+J_NECK_D)
                                for p in pads if L in p['layers']]) for L in A['cu_layers']}
    raw = []
    for t_index, t in enumerate(tracks):
        if t.get('current_shape') and 'current_A' in t:
            raw.extend((t['layer'], float(t['current_A']), a, b, t.get('copper_j_min', t['w'])[i])
                       for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])))
    for ti, si, layer, a, b, w, length, current, _ in (tree_idx or {}).get('seg', []):
        if not tracks[ti].get('current_shape'):
            raw.append((layer, current, a, b, tracks[ti].get('copper_j_min', tracks[ti]['w'])[si]))
    if tree_idx is None:
        unassigned = [t for t in tracks if not (t.get('current_shape') and 'current_A' in t)]
        if unassigned and current_A is None:
            return None, []
        for t in unassigned:
            raw.extend((t['layer'], float(current_A), a, b, t.get('copper_j_min', t['w'])[i])
                       for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])))
    grouped = defaultdict(list)
    for layer, current, a, b, w in raw:
        if math.dist(a, b) < 1e-10: continue
        line = LineString([a, b]); region = pad_regions[layer]


        for inside, geom in [(True, line.intersection(region)), (False, line.difference(region))]:
            for part in getattr(geom, 'geoms', [geom]):
                if part.geom_type != 'LineString' or part.length < 1e-10: continue
                coords = list(part.coords)
                for x, y in zip(coords[:-1], coords[1:]):
                    grouped[(layer, current, inside)].append((x, y, w, math.dist(x, y)))
    seg = []
    for (layer, current, inside), parts in sorted(grouped.items()):
        adj = defaultdict(list)
        def key(p): return tuple(round(float(v), 9) for v in p)
        for i, (a, b, _, _) in enumerate(parts):
            adj[key(a)].append(i); adj[key(b)].append(i)
        seen = set()
        def strand(start, first):
            path = []; node = start; edge = first
            while edge not in seen:
                seen.add(edge); path.append(edge)
                a, b, _, _ = parts[edge]
                node = key(b) if key(a) == node else key(a)
                if len(adj[node]) != 2: break
                edge = next((e for e in adj[node] if e not in seen), edge)
            length = sum(parts[e][3] for e in path)
            if inside and length <= J_NECK_L + 1e-9: return


            narrow = min((parts[e] for e in path), key=lambda p: p[2])
            a, b, w, _ = narrow
            seg.append((len(seg), 0, layer, a, b, w, length, current, A['cu_layers'][layer]['t']))
        for node in sorted(adj):
            if len(adj[node]) != 2:
                for edge in adj[node]:
                    if edge not in seen: strand(node, edge)
        for edge in range(len(parts)):
            if edge not in seen: strand(key(parts[edge][0]), edge)
    return current_density(seg, A, net, None, ipc_model=ipc_model)

def ground_corridors(B, before, after, K, design_rules=None):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from learning.copper_bars import copper_map, _islands_ok, _design_rules
    gnd = set(K.get('gnd') or [])
    if not gnd:
        return dict(passed=True, cu_layers=[], reason='no ground net')
    old, new = copper_map(B, before), copper_map(B, after)
    c0, e0, _ = _design_rules(before, design_rules)
    c1, e1, _ = _design_rules(after, design_rules)
    clr = max(c0, c1)
    outline = unary_union([Polygon(q).buffer(0) for q in B['outline']]).buffer(-max(e0, e1))
    rows = []
    for L in sorted({l for _, l in new} | {l for _, l in old}):
        a = unary_union([g for (n, l), g in old.items() if l == L and n not in gnd]).buffer(clr)
        b = unary_union([g for (n, l), g in new.items() if l == L and n not in gnd]).buffer(clr)
        if a.equals(b): ok = True
        else: ok = _islands_ok(outline, a, b)
        rows.append(dict(layer_name=L, passed=ok))
    return dict(passed=all(q['passed'] for q in rows), cu_layers=rows,
                reason='free ground corridors at 0.15 mm minimum web, no RF simulation')


VERSION = 'copper_map-fill-v3'


def via_geo(R, K=None):
    rg = (K or {}).get('design_rules') or {}
    rg = rg.get('router', rg)
    value = str((R.get('options') or {}).get('via') or rg.get('via') or '0.45/0.20')
    try:
        d, b = (float(x) for x in value.split('/'))
        if d <= 0 or b <= 0 or b > d: raise ValueError('Via-Geometry')
        return d, b
    except (ValueError, TypeError): return 0.45, 0.20


def settings(K=None, design_rules=None, clr=None, border=None):
    rg = design_rules if design_rules is not None else (K or {}).get('design_rules') or {}
    rg = rg.get('router', rg)
    c = float(clr if clr is not None else os.environ.get('ZCLR', rg.get('zclr', max(0.15, float(rg.get('drc') or 0)))))
    r = float(border if border is not None else os.environ.get('ZONE_EDGE', rg.get('zone_edge', 0.25)))
    if c < 0 or r < 0: raise ValueError('Zone spacing and edge must be non-negative')
    return c, r


def areas(B, R, K=None, gnd_layers=None):
    gl = gnd_layers if gnd_layers is not None else (R.get('gnd_layers') or (R.get('options') or {}).get('gnd_layers') or [])
    labels = gl.split(',') if isinstance(gl, str) else gl
    if K is not None and 'gnd' in K: gnd = set(K['gnd'])
    else:
        gnd = {p['net'] for p in B['pads'] if p['net'] and p['net'].lstrip('/').upper() in ('GND', 'PGND', 'AGND', 'DGND', 'GNDA', 'GNDD')}
    known = gnd | set((K or {}).get('net_list') or {}) | {p['net'] for p in B['pads']}
    planes, reserved = {}, set()
    for label in labels:
        if not label: continue
        fields = str(label).split('=', 1); reserved.add(fields[0])
        main = (K or {}).get('gnd_main')
        ns = set(fields[1].split('|')) if len(fields) == 2 else ({main} if main in gnd else gnd)
        ns = {n if n in known else ('/' + n if '/' + n in known else n.lstrip('/')) for n in ns}
        if len(ns) == 1 and not ns & {'', '?'}: planes[fields[0]] = next(iter(ns))
    return planes, reserved, gnd


def ground(B, R, K=None, gnd_layers=None):
    planes, _, gnd = areas(B, R, K, gnd_layers)
    return {L: n for L, n in planes.items() if n in gnd}, gnd


def via_ground(v, via_d=0.45, drill=0.2):
    return float(v.get('diameter', v.get('d', via_d))), float(v.get('drill', drill))


def on_layer(v, L):
    return L is None or 'layers' not in v or L in v['layers']


def keepout_area(k):
    lo = k.get('hole_list') or []
    return unary_union([Polygon(q, lo[i] if i < len(lo) else []).buffer(0)
                        for i, q in enumerate(k.get('polys', [])) if len(q) >= 3])


def geometry(B, R, L, via_d=0.45):
    out = []
    for t in R.get('tracks', []):
        if t['layer'] != L: continue
        if t.get('items'):
            from learning.copper_bars import track_geometry
            out.append((t['net'], track_geometry(t)))
        else:
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                out.append((t['net'], (LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8)))
    for z in R.get('zone_list', []):
        if z['layer'] == L:
            out.append((z['net'], unary_union([Polygon(q['outer'], q.get('inner') or []).buffer(0) for q in z['polys']])))
    for z in R.get('zones', []):
        if z.get('layer') == L and z.get('filled'):
            out.append((z.get('net', ''), Polygon(z['pts'], z.get('holes') or []).buffer(0)))
    for v in R.get('vias', []):
        if on_layer(v, L): out.append((v.get('net', ''), Point(v['x'], v['y']).buffer(via_ground(v, via_d)[0] / 2, 12)))
    for p in B['pads']:
        polys = (p.get('polys') or {}).get(L, [])
        if not polys and p.get('drill') and L in p.get('layers', []): polys = next(iter((p.get('polys') or {}).values()), [])
        for q in polys:
            if len(q) >= 3:
                from learning.copper_bars import pad_polygon
                out.append((p['net'], pad_polygon(B, p, q) if R.get('copper_bars') else Polygon(q).buffer(0)))
    return out


def drills(B, R, drill=0.2, L=None):
    geo = [Point(v['x'], v['y']).buffer(via_ground(v, drill=drill)[1] / 2, 12) for v in R.get('vias', []) if on_layer(v, L)]
    geo += [Point(p['x'], p['y']).buffer(p['drill'] / 2, 12) for p in B['pads'] if p.get('drill')]
    return unary_union(geo) if geo else Polygon()


def contour(B):
    holes = B.get('outline_holes') or []
    return unary_union([Polygon(q, holes[i] if i < len(holes) else []).buffer(0)
                        for i, q in enumerate(B['outline']) if len(q) >= 3])


def baseline(B, R, L, net_list, *, clr=0.15, border=0.25, via_d=0.45, drill=0.2):
    foreign = unary_union([g for n, g in geometry(B, R, L, via_d) if n not in set(net_list)])
    fill = contour(B).buffer(-border).difference(foreign.buffer(clr, 12))
    for k in B.get('keepouts', []):
        if (k.get('prohibitions') or {}).get('areas') and L in (k.get('cu_layers') or [L]):
            fill = fill.difference(keepout_area(k))
    return fill.difference(drills(B, R, drill, L))


def attached(B, R, fills, net_list, *, via_d=0.45, drill=0.2):
    nl = int(B.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or 2)
    cu_layers = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
    cu_layers = sorted(set(cu_layers) | set(fills))
    part_list = {L: geometry(B, R, L, via_d) for L in cu_layers}
    out = {L: Polygon() for L in fills}
    for net in sorted(set(net_list.values())):
        comps, per_layer, adj, seeds = [], {}, [], set()
        for L in cu_layers:
            g = unary_union([q for n, q in part_list[L] if n == net] + ([fills[L]] if net_list.get(L) == net else []))
            g = g.difference(drills(B, R, drill, L))
            per_layer[L] = []
            for q in getattr(g, 'geoms', [g]):
                if q.is_empty or q.geom_type != 'Polygon': continue
                i = len(comps); comps.append(q); adj.append(set()); per_layer[L].append(i)
        for p in B['pads']:
            if p.get('net') != net or p.get('graphic'): continue
            for L in cu_layers:

                pp = geometry(dict(pads=[p]), {}, L, via_d)
                for _, q in pp:
                    seeds.update(i for i in per_layer[L] if comps[i].intersects(q))
        for v in R.get('vias', []) + [p for p in B['pads'] if p.get('drill')]:
            if v.get('net') != net: continue
            d, b = via_ground(v, via_d, drill)
            if 'ref' in v:
                d = b = float(v['drill'])
                ring = Point(v['x'], v['y']).buffer(b / 2, 12).boundary
            else: ring = Point(v['x'], v['y']).buffer(d / 2, 12).difference(Point(v['x'], v['y']).buffer(b / 2, 12))
            ids = [i for L in cu_layers if on_layer(v, L) for i in per_layer[L] if comps[i].intersects(ring)]
            if ids:
                for i in ids[1:]: adj[ids[0]].add(i); adj[i].add(ids[0])
        seen = set(seeds); todo = list(sorted(seeds))
        while todo:
            for j in sorted(adj[todo.pop()] - seen): seen.add(j); todo.append(j)
        for L in fills:
            if net_list.get(L) == net:
                out[L] = fills[L].intersection(unary_union([comps[i] for i in per_layer[L] if i in seen]))
    return out


def ground_unconnected(B, R, K):
    planes, gnd = ground(B, R, K)
    clr, border = settings(K); d, b = via_geo(R, K)
    nl = int(B.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or 2)
    cu_layers = sorted(set(['F', 'B'] + ['In%d' % i for i in range(1, nl - 1)]) | set(planes))
    bad = []
    for net in sorted(set(planes.values()) & gnd):
        pads = [p for p in B['pads'] if p['net'] == net and not p.get('graphic')]
        fills = {L: baseline(B, R, L, {net}, clr=clr, border=border, via_d=d, drill=b)
                 for L, n in planes.items() if n == net}
        comps, per, parent, plane_ids = [], {}, [], set()
        for L in cu_layers:
            copper = unary_union([q for n, q in geometry(B, R, L, d) if n == net] + ([fills[L]] if L in fills else []))
            copper = copper.difference(drills(B, R, b, L)); per[L] = []
            for q in getattr(copper, 'geoms', [copper]):
                if q.is_empty or q.geom_type != 'Polygon': continue
                i = len(comps); comps.append(q); parent.append(i); per[L].append(i)
                if L in fills and q.intersection(fills[L]).area > 1e-12: plane_ids.add(i)
        def root(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        for v in R.get('vias', []) + [p for p in pads if p.get('drill')]:
            if v.get('net') != net: continue
            vd, vb = via_ground(v, d, b)
            ring = Point(v['x'], v['y']).buffer(vd / 2, 12).difference(Point(v['x'], v['y']).buffer(vb / 2, 12))
            if 'ref' in v: ring = Point(v['x'], v['y']).buffer(v['drill'] / 2, 12).boundary
            ids = [i for L in cu_layers if on_layer(v, L) for i in per[L] if comps[i].intersects(ring)]
            for i in ids[1:]: parent[root(i)] = root(ids[0])
        reached = {root(i) for i in plane_ids}; common = set(reached)
        for p in pads:
            ids = {root(i) for L in cu_layers for _, q in geometry(dict(pads=[p]), {}, L, d)
                   for i in per[L] if comps[i].intersects(q)}
            common &= ids
        if pads and not common: bad.append(net)
    return bad
