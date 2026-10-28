""
import copy
import math

from shapely.geometry import LineString, Point, box

ARTEN = {'fenster_lage', 'kugel_via', 'prioritaet'}


def pruefe(m):
    fields = dict(fenster_lage={'art', 'einheit', 'fenster', 'lage', 'zustand'},
                  kugel_via={'art', 'einheit', 'ref', 'pad', 'lage', 'zustand'},
                  prioritaet={'art', 'einheit', 'vor', 'zustand'})
    if not isinstance(m, dict) or m.get('art') not in fields or set(m) != fields[m['art']]:
        raise ValueError('Fehlende/unbekannte Agent-Zugfelder')
    if not all(isinstance(m[k], str) and m[k] for k in fields[m['art']] - {'fenster'}):
        raise ValueError('Agent-Zug braucht Textkennungen')
    if len(m['zustand']) != 64 or any(x not in '0123456789abcdef' for x in m['zustand']):
        raise ValueError('Agent-Zug braucht Zustand-SHA256')
    if m['art'] == 'fenster_lage':
        b = m['fenster']
        if (not isinstance(b, list) or len(b) != 4 or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in b) or
            not 0 < b[2] - b[0] <= 6 or not 0 < b[3] - b[1] <= 6):
            raise ValueError('Fenster braucht endliche bbox, Seiten <=6 mm')
    if m['art'] == 'prioritaet' and m['einheit'] == m['vor']:
        raise ValueError('Selbstprioritaet')
    return copy.deepcopy(m)


def lokal(G, m, c, radius):
    u = m['einheit']
    if u not in G['U']: return False
    if m['art'] == 'fenster_lage': return box(*m['fenster']).intersects(box(*c['bbox']))
    if m['art'] == 'prioritaet':
        return m['vor'] in G['U'] and all(G['_nah_an'](G['U'][w], c['punkte'], radius) for w in (u, m['vor']))
    pads = [p for p in G['D']['pads'] if p['ref'] == m['ref'] and p['num'] == m['pad']]
    return len(pads) == 1 and any(math.dist((pads[0]['x'], pads[0]['y']), q) <= radius for q in c['punkte'])


def fenster_geometrie(r, bounds, layer):
    ""
    r = copy.deepcopy(r); region = box(*bounds); tracks = []; changed = False
    for t in r['tracks']:
        if t['layer'] == layer:
            tracks.append(t); continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            line = LineString([a, b]); cut = line.intersection(region)
            if cut.is_empty or cut.length < 1e-8:
                tracks.append(dict(t, pts=[a, b], w=[w])); continue
            if any(t.get(k) for k in ('steg', 'strom_steg', 'kelvin', 'abg', 'rv', 'kette')):
                raise ValueError('Fenster trifft geschuetzte Bahn')
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


def ball_geometrie(G, r, m):
    ps = [p for p in G['D']['pads'] if p['ref'] == m['ref'] and p['num'] == m['pad']]
    f = next((f for f in G['D']['fps'] if f['ref'] == m['ref']), {})
    if len(ps) != 1 or 'BGA' not in f.get('fp', '').upper(): raise ValueError('Keine eindeutige BGA-Kugel')
    p = ps[0]
    if p['net'] != r['net'] or p.get('drill') or len(p['layers']) != 1:
        raise ValueError('Kugel/Netz/Lagenvertrag passt nicht')
    x, y = p['x'], p['y']
    r, _ = fenster_geometrie(r, [x - .3, y - .3, x + .3, y + .3], m['lage'])

    lines = [LineString(t['pts']) for t in r['tracks'] if t['layer'] == m['lage']]
    if not lines or min(l.distance(Point(x, y)) for l in lines) > G['VIA_D'] / 2:
        raise ValueError('Kugel hat keine lokale Bahn zur Ziellage')
    if not any(math.dist((x, y), (v['x'], v['y'])) < 1e-7 for v in r['vias']): r['vias'].append(dict(x=x, y=y))
    return r


def reihenfolge(order, edges):
    ""
    todo = list(order); out = []
    while todo:
        available = next((u for u in todo if not any(b == u and a in todo for a, b in edges)), None)
        if available is None: raise ValueError('Zyklische Netzprioritaet')
        todo.remove(available); out.append(available)
    return out


def anwenden(rl, m, c):
    G = rl.G; u = m['einheit']
    if G.get('_AL') is None: raise ValueError('Agent-Zug braucht agent_loop=1')
    if not rl.movable(u) or not lokal(G, m, c, rl.radius): raise ValueError('Agent-Zug nicht lokal/beweglich')
    if m['art'] == 'prioritaet':
        if not rl.movable(m['vor']): raise ValueError('Prioritaetsziel fest')
        edges = G['_AGENT_PRIORITAET'] + [(u, m['vor'])]
        reihenfolge(G['units'], edges)
        G['_AGENT_PRIORITAET'] = edges
        return reihenfolge(sorted({u, m['vor'], *rl.neighbours(c)}, key=lambda q: (G['grp'](q), q)), edges)
    if m['lage'] not in G['LAYERS']: raise ValueError('Ziellage keine Routinglage')
    r = G['U'][u]
    if m['art'] == 'fenster_lage':
        r, changed = fenster_geometrie(r, m['fenster'], m['lage'])
        if not changed: raise ValueError('Fenster aendert kein Kupfer')
    else: r = ball_geometrie(G, r, m)
    G['del_unit'](u); G['add_unit'](u, r['net'], r['tracks'], r['vias'])
    G['_AGENT_VORGABEN'].append(copy.deepcopy(m))
    return sorted(set(rl.neighbours(c)) - {u}, key=lambda q: (G['grp'](q), q))


def nachfuehren(G, u):
    ""
    if u not in G['U']: return
    r = G['U'][u]; changed = False
    for m in G['_AGENT_VORGABEN']:
        if m['einheit'] != u: continue
        if m['art'] == 'fenster_lage': r, _ = fenster_geometrie(r, m['fenster'], m['lage'])
        else: r = ball_geometrie(G, r, m)
        changed = True
    if changed:
        G['del_unit'](u); G['add_unit'](u, r['net'], r['tracks'], r['vias'])
