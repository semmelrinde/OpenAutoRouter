""
import copy
import math
import os

import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union


def ersatzrouten(B, R, K):
    from lernen.kupferbalken import kupfer, POWER
    cu = kupfer(B, R)
    out = dict(R, tracks=[])
    for t in R['tracks']:
        if K['netze'].get(t['net'], {}).get('art') not in POWER or t.get('sig') or t.get('kelvin'):
            out['tracks'].append(t)
            continue
        g = cu.get((t['net'], t['layer']))
        new = dict(t, w=[], kupfer_j_min=[])
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            d = math.dist(a, b)
            if d < 1e-9 or g is None:
                new['w'].append(w)
                new['kupfer_j_min'].append(w)
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
            new['kupfer_j_min'].append(float(widths.min()))
        out['tracks'].append(new)
    out['leistungsflaechen_mm2'] = {n: sum(g.area for (nn, _), g in cu.items() if nn == n)
                                   for n, k in K['netze'].items() if k.get('art') in POWER}
    return out


def ir_werte(B, R, K, res=0.1, karten=None):
    ""
    from gemeinsam.zonen import vereinheitlichen, boardgeometrie, viageometrie
    B = boardgeometrie(B, R)
    R = viageometrie(vereinheitlichen(R))
    from codex.zonen_ir.solver_fast import ir_kennzahlen
    from analyse.metriken import _strom, aufbau
    from lernen.kupferbalken import lagen, pad_polygon
    from gemeinsam.bogen import punkte
    rr = dict(R, kupferbalken=R.get('kupferbalken') or {'modell':'Leistungsflaechen'}, tracks=[])
    for t in R['tracks']:


        if 'items' not in t:
            rr['tracks'].append(t)
            continue

        for it in t['items']:
            ps, w = (it[1:3], it[3]) if it[0] == 'L' else (punkte(*it[1:4]), it[4])
            rr['tracks'].append(dict(net=t['net'], layer=t['layer'], pts=ps, w=[w] * (len(ps) - 1)))
    kk = copy.deepcopy(K)
    for n, k in kk['netze'].items():
        fl = k.get('fluss') or {}
        last = fl.get('last') or {}
        total = sum(float(q) for q in last.values())
        if fl.get('einheit') in (None, 'relativ') and total > 0:
            i, source = _strom(n, K)
            fl.update(last={p: float(q) * i / total for p, q in last.items()}, einheit='A',
                      herkunft={p: 'strom_A/' + str(source) for p in last})
    bb = dict(B, pads=[dict(p, polys={l: [list(pad_polygon(B, p, q).exterior.coords[:-1]) for q in ps]
                                     for l, ps in p.get('polys', {}).items()}) for p in B['pads']])
    if not B.get('stackup'):

        vg = (K.get('vorgaben') or {}).get('aufbau') or {}
        a = aufbau(len(lagen(B)), R.get('gndlagen') or [], diel=vg.get('dielektrika'))
        bb['stackup'] = {l: dict(copper_um=q['t'] * 1000, z_mm=q['z']) for l, q in a['lagen'].items()}
    if karten:
        from pathlib import Path
        from codex.zonen_ir.solver_fast import _analyse
        Path(karten).mkdir(parents=True, exist_ok=True)
        raw = _analyse(bb, rr, kk, res=res, outdir=Path(karten))
    else:
        raw = ir_kennzahlen(bb, rr, kk, res=res)
    solved = {n: q for n, q in raw['netze'].items() if q.get('status') == 'solved'}
    return dict(status=raw['status'], verlust_mW=1000 * sum(q['power_W'] for q in solved.values()),
                verlust_je_netz_mW={n: q['power_W'] * 1000 for n, q in solved.items()},
                max_abfall_mV={n: q['max_drop_V'] * 1000 for n, q in solved.items()},
                strom_je_netz_A={n: q['total_current_A'] for n, q in solved.items()},
                fehler_netze=[n for n, q in raw['netze'].items() if q.get('status') == 'error'],
                fehler_je_netz={n: q['error'] for n, q in raw['netze'].items() if q.get('status') == 'error'},
                J_max_A_mm2={n:q.get('J_max_A_mm2') for n,q in solved.items()},
                J_x={n:q.get('J_x') for n,q in solved.items()},
                J_max_ort={n:q.get('J_max_ort') for n,q in solved.items()},
                J_hist={n:q.get('J_hist') for n,q in solved.items()},
                abfall_je_pad_mV={n:{p:1000*v for p,v in q['drop_V'].items()} for n,q in solved.items()},
                raster_mm=res, laufzeit_s=raw['laufzeit_s'], modell='filled-Polygone/JLC-Stackup/DC-Raster',
                strommodus_je_netz={n: q.get('strommodus') for n, q in solved.items()},
                karten={n: q.get('karten_npz') for n, q in solved.items()} if karten else None)


def unverbunden_polygone(B, R, GND):
    ""
    from lernen.kupferbalken import kupfer, lagen, via_size, pad_polygon
    from collections import defaultdict
    from shapely import STRtree
    copper = kupfer(B, R, ungefuellt='ignorieren')
    vd, _ = via_size(R)
    bynet = defaultdict(list)
    for p in B['pads']:
        if p['net'] and p['net'] not in GND:
            bynet[p['net']].append(p)
    missing = []
    for net, pads in sorted(bynet.items()):
        if len({(p['ref'], p['num']) for p in pads}) < 2: continue
        gs, indices, trees, parent = {}, {}, {}, {}
        for layer in lagen(B):
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
            keys = [q for layer in v.get('layers') or lagen(B) if layer in trees for q in contact(layer, disk)]
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


def stromdichte_flaechen(tracks, baum, A, net, pads, strom_A=None, ipc_modell=None):
    ""
    from collections import defaultdict
    from analyse.metriken import stromdichte
    from shapely.geometry import LineString, box
    from shapely.ops import unary_union
    from analyse.metriken import J_HALS_D, J_HALS_L
    padgebiete = {L: unary_union([box(p['bbox'][0]-J_HALS_D, p['bbox'][1]-J_HALS_D,
                                     p['bbox'][2]+J_HALS_D, p['bbox'][3]+J_HALS_D)
                                for p in pads if L in p['layers']]) for L in A['lagen']}
    raw = []
    for t_index, t in enumerate(tracks):
        if t.get('strom_form') and 'strom_A' in t:
            raw.extend((t['layer'], float(t['strom_A']), a, b, t.get('kupfer_j_min', t['w'])[i])
                       for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])))
    for ti, si, layer, a, b, w, length, current, _ in (baum or {}).get('seg', []):
        if not tracks[ti].get('strom_form'):
            raw.append((layer, current, a, b, tracks[ti].get('kupfer_j_min', tracks[ti]['w'])[si]))
    if baum is None:
        unassigned = [t for t in tracks if not (t.get('strom_form') and 'strom_A' in t)]
        if unassigned and strom_A is None:
            return None, []
        for t in unassigned:
            raw.extend((t['layer'], float(strom_A), a, b, t.get('kupfer_j_min', t['w'])[i])
                       for i, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])))
    grouped = defaultdict(list)
    for layer, current, a, b, w in raw:
        if math.dist(a, b) < 1e-10: continue
        line = LineString([a, b]); region = padgebiete[layer]


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
            if inside and length <= J_HALS_L + 1e-9: return


            narrow = min((parts[e] for e in path), key=lambda p: p[2])
            a, b, w, _ = narrow
            seg.append((len(seg), 0, layer, a, b, w, length, current, A['lagen'][layer]['t']))
        for node in sorted(adj):
            if len(adj[node]) != 2:
                for edge in adj[node]:
                    if edge not in seen: strand(node, edge)
        for edge in range(len(parts)):
            if edge not in seen: strand(key(parts[edge][0]), edge)
    return stromdichte(seg, A, net, None, ipc_modell=ipc_modell)

def masse_korridore(B, vorher, nachher, K, regeln=None):
    ""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from lernen.kupferbalken import kupfer, _inseln_ok, _regeln
    gnd = set(K.get('gnd') or [])
    if not gnd:
        return dict(bestanden=True, lagen=[], grund='kein Massennetz')
    old, new = kupfer(B, vorher), kupfer(B, nachher)
    c0, e0, _ = _regeln(vorher, regeln)
    c1, e1, _ = _regeln(nachher, regeln)
    clr = max(c0, c1)
    outline = unary_union([Polygon(q).buffer(0) for q in B['outline']]).buffer(-max(e0, e1))
    rows = []
    for L in sorted({l for _, l in new} | {l for _, l in old}):
        a = unary_union([g for (n, l), g in old.items() if l == L and n not in gnd]).buffer(clr)
        b = unary_union([g for (n, l), g in new.items() if l == L and n not in gnd]).buffer(clr)
        if a.equals(b): ok = True
        else: ok = _inseln_ok(outline, a, b)
        rows.append(dict(lage=L, bestanden=ok))
    return dict(bestanden=all(q['bestanden'] for q in rows), lagen=rows,
                grund='freie Massekorridore bei 0.15-mm-Mindeststeg, keine HF-Simulation')


VERSION = 'kupfer-fuellung-v3'


def via_geo(R, K=None):
    ""
    rg = (K or {}).get('regeln') or {}
    rg = rg.get('router', rg)
    value = str((R.get('optionen') or {}).get('via') or rg.get('via') or '0.45/0.20')
    try:
        d, b = (float(x) for x in value.split('/'))
        if d <= 0 or b <= 0 or b > d: raise ValueError('Via-Geometrie')
        return d, b
    except (ValueError, TypeError): return 0.45, 0.20


def einstellungen(K=None, regeln=None, clr=None, rand=None):
    rg = regeln if regeln is not None else (K or {}).get('regeln') or {}
    rg = rg.get('router', rg)
    c = float(clr if clr is not None else os.environ.get('ZCLR', rg.get('zclr', max(0.15, float(rg.get('drc') or 0)))))
    r = float(rand if rand is not None else os.environ.get('ZONE_EDGE', rg.get('zone_edge', 0.25)))
    if c < 0 or r < 0: raise ValueError('Zonenabstand und Rand muessen nichtnegativ sein')
    return c, r


def flaechen(B, R, K=None, gndlagen=None):
    ""
    gl = gndlagen if gndlagen is not None else (R.get('gndlagen') or (R.get('optionen') or {}).get('gndlagen') or [])
    labels = gl.split(',') if isinstance(gl, str) else gl
    if K is not None and 'gnd' in K: gnd = set(K['gnd'])
    else:
        gnd = {p['net'] for p in B['pads'] if p['net'] and p['net'].lstrip('/').upper() in ('GND', 'PGND', 'AGND', 'DGND', 'GNDA', 'GNDD')}
    bekannt = gnd | set((K or {}).get('netze') or {}) | {p['net'] for p in B['pads']}
    planes, reserviert = {}, set()
    for label in labels:
        if not label: continue
        fields = str(label).split('=', 1); reserviert.add(fields[0])
        haupt = (K or {}).get('gnd_haupt')
        ns = set(fields[1].split('|')) if len(fields) == 2 else ({haupt} if haupt in gnd else gnd)
        ns = {n if n in bekannt else ('/' + n if '/' + n in bekannt else n.lstrip('/')) for n in ns}
        if len(ns) == 1 and not ns & {'', '?'}: planes[fields[0]] = next(iter(ns))
    return planes, reserviert, gnd


def masse(B, R, K=None, gndlagen=None):
    planes, _, gnd = flaechen(B, R, K, gndlagen)
    return {L: n for L, n in planes.items() if n in gnd}, gnd


def via_masse(v, via_d=0.45, drill=0.2):
    ""
    return float(v.get('diameter', v.get('d', via_d))), float(v.get('drill', drill))


def auf_lage(v, L):
    return L is None or 'layers' not in v or L in v['layers']


def sperrflaeche(k):
    lo = k.get('loecher') or []
    return unary_union([Polygon(q, lo[i] if i < len(lo) else []).buffer(0)
                        for i, q in enumerate(k.get('polys', [])) if len(q) >= 3])


def geometrie(B, R, L, via_d=0.45):
    out = []
    for t in R.get('tracks', []):
        if t['layer'] != L: continue
        if t.get('items'):
            from lernen.kupferbalken import track_geometry
            out.append((t['net'], track_geometry(t)))
        else:
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                out.append((t['net'], (LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8)))
    for z in R.get('zonen', []):
        if z['layer'] == L:
            out.append((z['net'], unary_union([Polygon(q['aussen'], q.get('innen') or []).buffer(0) for q in z['polys']])))
    for z in R.get('zones', []):
        if z.get('layer') == L and z.get('filled'):
            out.append((z.get('net', ''), Polygon(z['pts'], z.get('holes') or []).buffer(0)))
    for v in R.get('vias', []):
        if auf_lage(v, L): out.append((v.get('net', ''), Point(v['x'], v['y']).buffer(via_masse(v, via_d)[0] / 2, 12)))
    for p in B['pads']:
        polys = (p.get('polys') or {}).get(L, [])
        if not polys and p.get('drill') and L in p.get('layers', []): polys = next(iter((p.get('polys') or {}).values()), [])
        for q in polys:
            if len(q) >= 3:
                from lernen.kupferbalken import pad_polygon
                out.append((p['net'], pad_polygon(B, p, q) if R.get('kupferbalken') else Polygon(q).buffer(0)))
    return out


def bohrungen(B, R, drill=0.2, L=None):
    geo = [Point(v['x'], v['y']).buffer(via_masse(v, drill=drill)[1] / 2, 12) for v in R.get('vias', []) if auf_lage(v, L)]
    geo += [Point(p['x'], p['y']).buffer(p['drill'] / 2, 12) for p in B['pads'] if p.get('drill')]
    return unary_union(geo) if geo else Polygon()


def umriss(B):
    ""
    holes = B.get('outline_holes') or []
    return unary_union([Polygon(q, holes[i] if i < len(holes) else []).buffer(0)
                        for i, q in enumerate(B['outline']) if len(q) >= 3])


def bezug(B, R, L, netze, *, clr=0.15, rand=0.25, via_d=0.45, drill=0.2):
    ""
    fremd = unary_union([g for n, g in geometrie(B, R, L, via_d) if n not in set(netze)])
    fill = umriss(B).buffer(-rand).difference(fremd.buffer(clr, 12))
    for k in B.get('keepouts', []):
        if (k.get('verbote') or {}).get('flaechen') and L in (k.get('lagen') or [L]):
            fill = fill.difference(sperrflaeche(k))
    return fill.difference(bohrungen(B, R, drill, L))


def angeschlossen(B, R, fills, netze, *, via_d=0.45, drill=0.2):
    ""
    nl = int(B.get('lagen') or (R.get('optionen') or {}).get('lagen') or 2)
    lagen = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
    lagen = sorted(set(lagen) | set(fills))
    teile = {L: geometrie(B, R, L, via_d) for L in lagen}
    out = {L: Polygon() for L in fills}
    for net in sorted(set(netze.values())):
        comps, je_lage, adj, seeds = [], {}, [], set()
        for L in lagen:
            g = unary_union([q for n, q in teile[L] if n == net] + ([fills[L]] if netze.get(L) == net else []))
            g = g.difference(bohrungen(B, R, drill, L))
            je_lage[L] = []
            for q in getattr(g, 'geoms', [g]):
                if q.is_empty or q.geom_type != 'Polygon': continue
                i = len(comps); comps.append(q); adj.append(set()); je_lage[L].append(i)
        for p in B['pads']:
            if p.get('net') != net or p.get('graphic'): continue
            for L in lagen:

                pp = geometrie(dict(pads=[p]), {}, L, via_d)
                for _, q in pp:
                    seeds.update(i for i in je_lage[L] if comps[i].intersects(q))
        for v in R.get('vias', []) + [p for p in B['pads'] if p.get('drill')]:
            if v.get('net') != net: continue
            d, b = via_masse(v, via_d, drill)
            if 'ref' in v:
                d = b = float(v['drill'])
                ring = Point(v['x'], v['y']).buffer(b / 2, 12).boundary
            else: ring = Point(v['x'], v['y']).buffer(d / 2, 12).difference(Point(v['x'], v['y']).buffer(b / 2, 12))
            ids = [i for L in lagen if auf_lage(v, L) for i in je_lage[L] if comps[i].intersects(ring)]
            if ids:
                for i in ids[1:]: adj[ids[0]].add(i); adj[i].add(ids[0])
        seen = set(seeds); todo = list(sorted(seeds))
        while todo:
            for j in sorted(adj[todo.pop()] - seen): seen.add(j); todo.append(j)
        for L in fills:
            if netze.get(L) == net:
                out[L] = fills[L].intersection(unary_union([comps[i] for i in je_lage[L] if i in seen]))
    return out


def masse_unverbunden(B, R, K):
    ""
    planes, gnd = masse(B, R, K)
    clr, rand = einstellungen(K); d, b = via_geo(R, K)
    nl = int(B.get('lagen') or (R.get('optionen') or {}).get('lagen') or 2)
    lagen = sorted(set(['F', 'B'] + ['In%d' % i for i in range(1, nl - 1)]) | set(planes))
    bad = []
    for net in sorted(set(planes.values()) & gnd):
        pads = [p for p in B['pads'] if p['net'] == net and not p.get('graphic')]
        fills = {L: bezug(B, R, L, {net}, clr=clr, rand=rand, via_d=d, drill=b)
                 for L, n in planes.items() if n == net}
        comps, je, parent, plane_ids = [], {}, [], set()
        for L in lagen:
            copper = unary_union([q for n, q in geometrie(B, R, L, d) if n == net] + ([fills[L]] if L in fills else []))
            copper = copper.difference(bohrungen(B, R, b, L)); je[L] = []
            for q in getattr(copper, 'geoms', [copper]):
                if q.is_empty or q.geom_type != 'Polygon': continue
                i = len(comps); comps.append(q); parent.append(i); je[L].append(i)
                if L in fills and q.intersection(fills[L]).area > 1e-12: plane_ids.add(i)
        def root(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        for v in R.get('vias', []) + [p for p in pads if p.get('drill')]:
            if v.get('net') != net: continue
            vd, vb = via_masse(v, d, b)
            ring = Point(v['x'], v['y']).buffer(vd / 2, 12).difference(Point(v['x'], v['y']).buffer(vb / 2, 12))
            if 'ref' in v: ring = Point(v['x'], v['y']).buffer(v['drill'] / 2, 12).boundary
            ids = [i for L in lagen if auf_lage(v, L) for i in je[L] if comps[i].intersects(ring)]
            for i in ids[1:]: parent[root(i)] = root(ids[0])
        reached = {root(i) for i in plane_ids}; common = set(reached)
        for p in pads:
            ids = {root(i) for L in lagen for _, q in geometrie(dict(pads=[p]), {}, L, d)
                   for i in je[L] if comps[i].intersects(q)}
            common &= ids
        if pads and not common: bad.append(net)
    return bad
