import math
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree


def owner(q):
    return q.get('unit') or q['net']


def rule_owner(u, units, owners):
    return u if u.startswith('GND#') and u in owners else units.get(u)


def _unit(board, data, unit, bad, hints):
    tracks = [(i, t) for i, t in enumerate(data['tracks']) if owner(t) == unit]
    vias = [(i, v) for i, v in enumerate(data['vias']) if owner(v) == unit]
    if any('items' in t for _, t in tracks):
        raise ValueError('native items: conservative full teardown')
    if any(owner(z) == unit for k in ('zones', 'zone_list') for z in data.get(k, [])):
        raise ValueError('Unit with zone: conservative full teardown')
    nets = {q['net'] for _, q in tracks + vias}
    if len(nets) != 1:
        raise ValueError('multiple copper nets in one unit')
    net = next(iter(nets))
    stack = ['F'] + ['In%d' % i for i in range(1, int(board.get('cu_layers') or 2)-1)] + ['B']

    pads = []
    for pi, p in enumerate(board['pads']):
        if p.get('net') != net: continue
        for layer, polys in (p.get('polys') or {}).items():
            for poly in polys: pads.append((pi, layer, Polygon(poly)))
    segs = []
    for ti, t in tracks:
        for si, (a, b, w) in enumerate(zip(t['pts'], t['pts'][1:], t['w'])):
            from postprocessing.short_circuit_protection import _null
            if not _null(a, b):
                segs.append(dict(ti=ti, si=si, layer=t['layer'], a=a, b=b, w=w,
                                 line=LineString([a, b]), cuts={0., 1.}))


    by_layer = defaultdict(list)
    for i, s in enumerate(segs): by_layer[s['layer']].append(i)
    def cut(s, p):
        if s['line'].distance(p) <= 1e-9:
            s['cuts'].add(max(0., min(1., s['line'].project(p, normalized=True))))
    for layer, ids in sorted(by_layer.items()):
        lines = [segs[i]['line'] for i in ids]
        tree = STRtree(lines)
        for j, line in enumerate(lines):
            s = segs[ids[j]]
            for k in sorted(tree.query(line).tolist()):
                if k <= j: continue
                inter = line.intersection(lines[k])
                if inter.is_empty: continue
                if inter.geom_type == 'Point': points = [inter]
                elif inter.geom_type == 'MultiPoint': points = list(inter.geoms)
                elif inter.geom_type == 'LineString': points = [Point(inter.coords[0]), Point(inter.coords[-1])]
                else: raise ValueError('unclear centerline topology')
                for p in points: cut(s, p); cut(segs[ids[k]], p)
            for _, v in vias:
                if layer in (v.get('layers') or stack): cut(s, Point(v['x'], v['y']))
            for _, L, g in pads:
                if L == layer and line.intersects(g):

                    cut(s, line.interpolate(line.project(g.representative_point())))
    nodes = {}; anchors = defaultdict(set); edges = []; track_edges = defaultdict(list)
    def node(layer, xy):
        key = (layer, round(xy[0], 9), round(xy[1], 9))
        if key not in nodes: nodes[key] = len(nodes)
        n = nodes[key]
        for pi, L, g in pads:
            if L == layer and g.covers(Point(xy)): anchors[n].add(pi)
        return n
    def edge(a, b, **kw):
        i = len(edges); edges.append(dict(a=a, b=b, **kw)); return i
    for s in segs:
        cuts = sorted(s['cuts'])
        for lo, hi in zip(cuts, cuts[1:]):
            a = list(s['line'].interpolate(lo, normalized=True).coords[0]) if lo else s['a']
            b = list(s['line'].interpolate(hi, normalized=True).coords[0]) if hi != 1 else s['b']
            if math.dist(a, b) < 1e-9: continue
            idx = edge(node(s['layer'], a), node(s['layer'], b), kind='track',
                       ti=s['ti'], si=s['si'], layer=s['layer'], pts=[a, b], w=s['w'])
            track_edges[s['ti']].append(idx)
    for vi, v in vias:
        hub = len(nodes); nodes[('via', vi)] = hub
        for layer in v.get('layers') or stack:
            edge(hub, node(layer, [v['x'], v['y']]), kind='via', vi=vi, layer=layer)
    adj = defaultdict(set)
    for i, e in enumerate(edges):
        adj[e['a']].add(i); adj[e['b']].add(i)
    remove = set()
    for i, e in enumerate(edges):
        key = ('tracks', e['ti'], e['si']) if e['kind'] == 'track' else ('vias', e['vi'])
        if key in bad:
            if e['kind'] == 'via': remove.add(i)
            else:
                copper = LineString(e['pts']).buffer(e['w']/2/math.cos(math.pi/64)+1e-9, quad_segs=16)
                if any(copper.intersects(g) for g in bad[key]): remove.add(i)


    for layer, x, y in hints:
        p = Point(x, y); candidates = []
        for i, e in enumerate(edges):
            if layer != '*' and e['layer'] != layer: continue
            if e['kind'] == 'track':
                if layer == '*': continue
                d = max(0., LineString(e['pts']).distance(p) - e['w']/2)
            else:
                v = data['vias'][e['vi']]; d = math.hypot(v['x']-x, v['y']-y)
            candidates.append((d, i))
        if not candidates: raise ValueError('Rule location without copper')
        nearest = min(d for d, _ in candidates)
        remove.update(i for d, i in candidates if d <= nearest + .015)
    if not remove: raise ValueError('no conflict branch assigned')


    pending = list(sorted(remove))
    while pending:
        e = edges[pending.pop()]
        for n in (e['a'], e['b']):
            if anchors[n] or len(adj[n]) != 2: continue
            for j in sorted(adj[n] - remove): remove.add(j); pending.append(j)
    keep = set(range(len(edges))) - remove


    while True:
        alive = defaultdict(set)
        for i in sorted(keep):
            e = edges[i]; alive[e['a']].add(i); alive[e['b']].add(i)
        drop = set()
        for n, es in alive.items():
            if len(es) == 1 and not anchors[n]: drop.update(es)
        for vi, _ in vias:
            arms = {i for i in keep if edges[i]['kind'] == 'via' and edges[i]['vi'] == vi}
            touched = set()
            for i in arms:
                e = edges[i]; n = e['b']
                if anchors[n] or any(edges[j]['kind'] == 'track' for j in alive[n]): touched.add(e['layer'])
            if len(touched) < 2: drop.update(arms)
            else: drop.update(i for i in arms if edges[i]['layer'] not in touched)
        if not drop: break
        keep -= drop

    seen = set()
    for start in sorted(keep):
        if start in seen: continue
        todo = [start]; component = set(); terminals = set()
        while todo:
            i = todo.pop()
            if i in component: continue
            component.add(i); e = edges[i]
            for n in (e['a'], e['b']):
                terminals.update(anchors[n]); todo.extend(sorted((adj[n] & keep) - component))
        seen.update(component)
        if len(terminals) < 2: keep -= component
    result_tracks = {}
    for ti, t in tracks:
        ids = track_edges[ti]
        if ids and all(i in keep for i in ids): result_tracks[ti] = [t]; continue
        groups = []; pts = []; widths = []
        def flush():
            if widths:
                q = dict(t, pts=list(pts), w=list(widths))
                if 'nm_pts' in q: q['nm_pts'] = [[round(x*1e6) for x in p] for p in pts]
                if 'nm_w' in q: q['nm_w'] = [round(w*1e6) for w in widths]
                groups.append(q)
        for i in ids:
            e = edges[i]
            if i not in keep:
                flush(); pts = []; widths = []; continue
            if pts and math.dist(pts[-1], e['pts'][0]) > 1e-9:
                flush(); pts = []; widths = []
            if not pts: pts = [e['pts'][0]]
            pts.append(e['pts'][1]); widths.append(e['w'])
        flush(); result_tracks[ti] = groups
    result_vias = {vi for vi, _ in vias if any(i in keep and e['kind'] == 'via' and e['vi'] == vi for i, e in enumerate(edges))}
    return result_tracks, result_vias


def partial_teardown(board, data, chosen, units, pairs, check=None):
    from postprocessing.short_circuit_protection import export_contacts
    owners = {owner(q) for k in ('tracks', 'vias') for q in data[k]}
    from shapely import from_wkb
    bad = defaultdict(lambda: defaultdict(list)); hints = defaultdict(list); fallback = {}; partial = {}
    for c in export_contacts(board, data, details=True)[0]:
        for u, obj in zip(c['unit_list'], c['objects']):
            if u in chosen and obj is not None: bad[u][tuple(obj)].append(from_wkb(c['contact_wkb']))
    for layer, _, a, b, x, y in pairs:
        for u in (rule_owner(a, units, owners), rule_owner(b, units, owners)):
            if u in chosen: hints[u].append((layer, x, y))
    if check is not None and getattr(check,'r44_witnesses',False):
        _, witnesses=check(data,witnesses=True)
        for u in chosen:
            for key,wkb in witnesses.get(u,[]): bad[u][tuple(key)].append(from_wkb(wkb))

            if hints[u] and not witnesses.get(u): fallback[u]='Rule conflict without exact copper witnesses'
            hints[u]=[]
    for u in sorted(chosen):
        if u in fallback: continue
        try:
            if hints[u] and check is None: raise ValueError('Rule conflict without full router cross-check')
            partial[u] = _unit(board, data, u, bad[u], hints[u])
        except Exception as exc:
            fallback[u] = str(exc)[:180]
    def materialize():
        result = dict(data); ts = []; vs = []
        for i, t in enumerate(data['tracks']):
            u = owner(t)
            if u not in chosen: ts.append(t)
            elif u in partial and u not in fallback: ts.extend(partial[u][0].get(i, []))
        for i, v in enumerate(data['vias']):
            u = owner(v)
            if u not in chosen or (u in partial and u not in fallback and i in partial[u][1]): vs.append(v)
        result.update(tracks=ts, vias=vs)
        for k in ('zones', 'zone_list'):
            if k in data: result[k] = [z for z in data[k] if owner(z) not in chosen]
        return result
    result = materialize()
    rest = export_contacts(board, result)[0]
    for c in rest:
        for u in c['unit_list']:
            if u in chosen: fallback[u] = 'Contact after partial teardown'
    if check is not None:


        try:
            remaining = list(check(result))
            for _, _, a, b, _, _ in remaining:
                for u in (rule_owner(a, units, owners), rule_owner(b, units, owners)):
                    if u in chosen: fallback[u] = 'Rule conflict after partial teardown'
        except Exception as exc:
            fallback.update({u: 'Rule check: '+str(exc)[:150] for u in chosen})
    result = materialize()
    if check is not None:
        try:
            if list(check(result)): raise ValueError('Residual rules after fallback')
        except Exception as exc:
            fallback.update({u: str(exc)[:180] for u in chosen})
            result = materialize()
    if export_contacts(board, result)[0]:
        raise ValueError('R44 partial teardown: residual contact after full teardown')
    retained = {owner(q) for k in ('tracks', 'vias') for q in result[k]} & chosen
    info = dict(version=1, units_partially=sorted(retained), fallback=fallback,
                segments_before=sum(len(t['w']) for t in data['tracks'] if owner(t) in chosen),
                segments_after=sum(len(t['w']) for t in result['tracks'] if owner(t) in chosen),
                vias_before=sum(owner(v) in chosen for v in data['vias']),
                vias_after=sum(owner(v) in chosen for v in result['vias']),
                full_rule_check=check is not None)
    return result, chosen - retained, info
