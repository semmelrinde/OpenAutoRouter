import copy
import math

from shapely.geometry import LineString, Point, box

KINDS = {'window_layer', 'ball_via', 'priority'}


def check(m):
    fields = dict(window_layer={'category', 'unit', 'window', 'layer_name', 'cur_state'},
                  ball_via={'category', 'unit', 'ref', 'pad', 'layer_name', 'cur_state'},
                  priority={'category', 'unit', 'before', 'cur_state'})
    if not isinstance(m, dict) or m.get('category') not in fields or set(m) != fields[m['category']]:
        raise ValueError('Missing/unknown agent move fields')
    if not all(isinstance(m[k], str) and m[k] for k in fields[m['category']] - {'window'}):
        raise ValueError('Agent move needs text identifiers')
    if len(m['cur_state']) != 64 or any(x not in '0123456789abcdef' for x in m['cur_state']):
        raise ValueError('Agent move needs state SHA256')
    if m['category'] == 'window_layer':
        b = m['window']
        if (not isinstance(b, list) or len(b) != 4 or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in b) or
            not 0 < b[2] - b[0] <= 6 or not 0 < b[3] - b[1] <= 6):
            raise ValueError('Window needs finite bbox, sides <=6 mm')
    if m['category'] == 'priority' and m['unit'] == m['before']:
        raise ValueError('SelfPriority')
    return copy.deepcopy(m)


def local(G, m, c, radius):
    u = m['unit']
    if u not in G['U']: return False
    if m['category'] == 'window_layer': return box(*m['window']).intersects(box(*c['bbox']))
    if m['category'] == 'priority':
        return m['before'] in G['U'] and all(G['_close_on'](G['U'][w], c['points'], radius) for w in (u, m['before']))
    pads = [p for p in G['D']['pads'] if p['ref'] == m['ref'] and p['num'] == m['pad']]
    return len(pads) == 1 and any(math.dist((pads[0]['x'], pads[0]['y']), q) <= radius for q in c['points'])


def window_geometry(r, bounds, layer):
    r = copy.deepcopy(r); region = box(*bounds); tracks = []; changed = False
    for t in r['tracks']:
        if t['layer'] == layer:
            tracks.append(t); continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            line = LineString([a, b]); cut = line.intersection(region)
            if cut.is_empty or cut.length < 1e-8:
                tracks.append(dict(t, pts=[a, b], w=[w])); continue
            if any(t.get(k) for k in ('web', 'current_web', 'kelvin', 'tap', 'rv', 'link_chain')):
                raise ValueError('Window hits protected track')
            coords = list(cut.coords)
            c, d = sorted((list(coords[0]), list(coords[-1])), key=lambda p: line.project(Point(p)))
            if math.dist(a, c) > 1e-8: tracks.append(dict(t, pts=[a, c], w=[w]))
            tracks.append(dict(t, layer=layer, pts=[c, d], w=[w])); changed = True
            if math.dist(d, b) > 1e-8: tracks.append(dict(t, pts=[d, b], w=[w]))
            for x, y in (c, d):
                if not any(math.dist((x, y), (v['x'], v['y'])) < 1e-7 for v in r['vias']):
                    r['vias'].append(dict(x=x, y=y))
    r['tracks'] = tracks
    return r, changed


def ball_geometry(G, r, m):
    ps = [p for p in G['D']['pads'] if p['ref'] == m['ref'] and p['num'] == m['pad']]
    f = next((f for f in G['D']['fps'] if f['ref'] == m['ref']), {})
    if len(ps) != 1 or 'BGA' not in f.get('fp', '').upper(): raise ValueError('No unambiguous BGA ball')
    p = ps[0]
    if p['net'] != r['net'] or p.get('drill') or len(p['layers']) != 1:
        raise ValueError('Ball/net/layer contract does not match')
    x, y = p['x'], p['y']
    r, _ = window_geometry(r, [x - .3, y - .3, x + .3, y + .3], m['layer_name'])

    lines = [LineString(t['pts']) for t in r['tracks'] if t['layer'] == m['layer_name']]
    if not lines or min(l.distance(Point(x, y)) for l in lines) > G['VIA_D'] / 2:
        raise ValueError('Ball has no local track to the target layer')
    if not any(math.dist((x, y), (v['x'], v['y'])) < 1e-7 for v in r['vias']): r['vias'].append(dict(x=x, y=y))
    return r


def order(order, edges):
    todo = list(order); out = []
    while todo:
        available = next((u for u in todo if not any(b == u and a in todo for a, b in edges)), None)
        if available is None: raise ValueError('Cyclic net priority')
        todo.remove(available); out.append(available)
    return out


def apply(rl, m, c):
    G = rl.G; u = m['unit']
    if G.get('_AL') is None: raise ValueError('Agent move needs agent_loop=1')
    if not rl.movable(u) or not local(G, m, c, rl.radius): raise ValueError('Agent move not local/movable')
    if m['category'] == 'priority':
        if not rl.movable(m['before']): raise ValueError('Priority target fixed')
        edges = G['_AGENT_PRIORITY'] + [(u, m['before'])]
        order(G['units'], edges)
        G['_AGENT_PRIORITY'] = edges
        return order(sorted({u, m['before'], *rl.neighbours(c)}, key=lambda q: (G['grp'](q), q)), edges)
    if m['layer_name'] not in G['LAYERS']: raise ValueError('Target layer is not a routing layer')
    r = G['U'][u]
    if m['category'] == 'window_layer':
        r, changed = window_geometry(r, m['window'], m['layer_name'])
        if not changed: raise ValueError('Window does not change copper')
    else: r = ball_geometry(G, r, m)
    G['del_unit'](u); G['add_unit'](u, r['net'], r['tracks'], r['vias'])
    G['_AGENT_SPECS'].append(copy.deepcopy(m))
    return sorted(set(rl.neighbours(c)) - {u}, key=lambda q: (G['grp'](q), q))


def update(G, u):
    if u not in G['U']: return
    r = G['U'][u]; changed = False
    for m in G['_AGENT_SPECS']:
        if m['unit'] != u: continue
        if m['category'] == 'window_layer': r, _ = window_geometry(r, m['window'], m['layer_name'])
        else: r = ball_geometry(G, r, m)
        changed = True
    if changed:
        G['del_unit'](u); G['add_unit'](u, r['net'], r['tracks'], r['vias'])
