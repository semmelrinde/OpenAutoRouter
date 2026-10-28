""
from __future__ import annotations

from collections import defaultdict
from time import perf_counter
import math
import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from codex.zonen_ir.fuellung import fuelle_zonen, polygone
from codex.zonen_ir.solver_fast import Grid, geometry, layers_of, ir_kennzahlen


def gruppen(zones):
    out = defaultdict(list)
    for z in zones:
        out[z['net'], z['layer']].append(Polygon(z['pts'], z.get('holes', [])))
    return {k: unary_union(v) for k, v in out.items()}


def als_zonen(groups, prios, *, filled=False):
    ""
    out = []
    for (net, layer), geom in sorted(groups.items()):
        for p in polygone(geom):
            if p.area < 1e-8:
                continue
            z = dict(net=net, layer=layer, prio=prios.get(net, 4), full=True, polygon_dc=True,
                     pts=[list(c) for c in p.exterior.coords][:-1])
            if p.interiors:
                z['holes'] = [list(map(list, r.coords))[:-1] for r in p.interiors]
            if filled:
                z.update(filled=True, estimated=True)
            out.append(z)
    return out


def netz_loesen(B, R, K, net, zs, res):
    layers = layers_of(B, R, K)
    vd, drill = map(float, (R.get('optionen') or {}).get('via', '0.45/0.20').split('/'))
    g, pads, barrels, _, _ = geometry(B, dict(R, zones=zs), net, layers, vd)
    f = K['netze'][net]['fluss']
    loads = {k: float(v) for k, v in f['last'].items()}
    total = sum(loads.values())
    if total <= 0 or not math.isfinite(total):
        raise ValueError('Kein positiver DC-Lastfall')
    if f.get('einheit') != 'A':
        loads = {k: v/total for k, v in loads.items()}
    grid = Grid(g, pads, barrels, layers, res, B.get('stackup'), via_drill=drill)
    result, state = grid.solve(f['quelle'], loads)
    return result, grid, state


def dichte_flaeche(grid, state, threshold, margin):
    ""
    density = np.zeros(grid.n)
    planar = np.array([k == 'sheet' for k in grid.kinds])

    planar |= np.array([k == 'copper' for k in grid.kinds])
    js = np.nan_to_num(state['density'][planar])
    np.maximum.at(density, grid.a[planar], js)
    np.maximum.at(density, grid.b[planar], js)
    out = {}
    for layer, ix in grid.idx.items():
        mask = ix >= 0
        mask[mask] &= density[ix[mask]] > threshold
        rects = []

        for j in np.flatnonzero(mask.any(axis=1)):
            edges = np.diff(np.r_[False, mask[j], False].astype(int))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                rects.append(box(grid.x0+a*grid.res, grid.y0+j*grid.res,
                                 grid.x0+b*grid.res, grid.y0+(j+1)*grid.res))
        g = unary_union(rects)

        out[layer] = g.buffer(margin, join_style='mitre').simplify(grid.res*.22)
    return out, density


def akzeptabel(result, baseline, rel, abs_v):
    if result['power_W'] > baseline['power_W'] * (1+rel) + 1e-12:
        return False
    return all(result['drop_V'][p] <= v*(1+rel)+abs_v
               for p, v in baseline['drop_V'].items())


def leistungs_polygone(B, R, K, zones, *, options=None):
    ""
    t0 = perf_counter()
    o = dict(options or {})
    res = float(o.get('ZONE_POLY_RES', '.1'))
    grow = float(o.get('ZONE_POLY_GROW', '.8'))
    area_ratio = float(o.get('ZONE_POLY_AREA', '1.0'))
    rel = float(o.get('ZONE_POLY_TOL', '.001'))
    abs_v = float(o.get('ZONE_POLY_ABS_MV', '.01')) / 1000
    gap = max(float(o.get('ZONE_GAP', '.2')), float(o.get('ZCLR', '.15')))
    clr = float(o.get('ZCLR', '.15'))
    if res <= 0 or grow < 0 or not 0 < area_ratio <= 1 or rel < 0:
        raise ValueError('Ungültige Polygonparameter')

    power = {n for n, k in K['netze'].items()
             if k.get('art') in ('strom', 'versorgung') and k.get('fluss')
             and sum(k['fluss'].get('last', {}).values()) > 0}
    fixed = [z for z in zones if z['net'] not in power or z.get('prio', 0) == 0]
    editable = [z for z in zones if z['net'] in power and z.get('prio', 0) > 0]
    prios = {z['net']: z['prio'] for z in editable}
    base = gruppen(editable)
    fixed_groups = gruppen(fixed)
    filled, _ = fuelle_zonen(B, R, K, zones, clearance=clr)
    base_fill = gruppen(filled)
    ir = ir_kennzahlen(B, dict(R, zones=filled), K, res=res)
    if ir['status'] != 'solved':
        raise ValueError('Vollständige Basis-IR fehlt: '+str(ir['status']))
    outline = unary_union([Polygon(p) for p in B['outline']]).buffer(-float(o.get('ZONE_EDGE', '.25')))
    if o.get('ZUNGE'):
        a, b, y = map(float, o['ZUNGE'].split(','))
        x0, y0, x1, y1 = outline.bounds
        outline = outline.difference(box(x0-1, y, x1+1, y1+1)).union(
            outline.intersection(box(a, y, b, y1+1)))
    current = dict(base)
    rows = []

    order = sorted({n for n, _ in base}, key=lambda n: (-ir['netze'][n]['power_W'], n))
    thresholds = [0, .015, .03, .06, .10, .16, .24, .36, .50, .70, 1.0]
    for net in order:
        tn = perf_counter()
        keys = [k for k in base if k[0] == net]
        baseline = ir['netze'][net]
        budget = sum(base_fill.get(k, Polygon()).area for k in keys) * area_ratio
        candidates = {}
        for key in keys:
            _, layer = key

            other = [g.buffer(gap, join_style='mitre') for k, g in current.items()
                     if k[1] == layer and k[0] != net]
            other += [g.buffer(gap) for k, g in fixed_groups.items()
                      if k[1] == layer and k[0] != net and k[0] in power]
            candidates[key] = base[key].buffer(grow, join_style='mitre').intersection(outline).difference(unary_union(other))
        raw = als_zonen(candidates, prios)
        enlarged, _ = fuelle_zonen(B, R, K, raw, clearance=clr)
        available = gruppen(enlarged)
        result, grid, state = netz_loesen(B, R, K, net, enlarged, res)
        _, density = dichte_flaeche(grid, state, 0, .15)
        positive = density[density > 1e-12]
        scale = float(np.quantile(positive, .90)) if len(positive) else 1.0

        ls = layers_of(B, R, K)
        vd = float((R.get('optionen') or {}).get('via', '0.45/0.20').split('/')[0])
        protected, _, _, _, _ = geometry(B, dict(R, zones=[]), net, ls, vd)
        trials = []
        best = None
        for fraction in thresholds:
            active, _ = dichte_flaeche(grid, state, fraction*scale, .15)
            trial = {k: g.intersection(active.get(k[1], Polygon()).union(
                         protected.get(k[1], Polygon()).buffer(.08, join_style='mitre')))
                     for k, g in available.items()}
            zs = als_zonen(trial, prios)

            fzs, _ = fuelle_zonen(B, R, K, zs, clearance=clr)
            area = sum(g.area for g in gruppen(fzs).values())
            rr, _, _ = netz_loesen(B, R, K, net, fzs, res)
            ok = area <= budget+1e-7 and akzeptabel(rr, baseline, rel, abs_v)
            trials.append(dict(threshold_fraction=fraction, threshold_A_mm2=fraction*scale,
                               area_mm2=area, power_mW=rr['power_W']*1000,
                               max_drop_mV=max(rr['drop_V'].values())*1000, accepted=ok))
            if ok and (best is None or (rr['power_W'], area) < best[0]):
                best = ((rr['power_W'], area), trial, fraction)
        if best:
            for k in keys:
                current[k] = best[1].get(k, Polygon())
        row = dict(net=net, budget_mm2=budget, basis_power_mW=baseline['power_W']*1000,
                   selected_fraction=best[2] if best else None, fallback=best is None,
                   trials=trials, elapsed_s=perf_counter()-tn)
        rows.append(row)
    output = fixed + als_zonen(current, prios)
    return output, dict(method='stromdichte_freiraum_v1', heuristic=True,
                        options=o, order=order, nets=rows, elapsed_s=perf_counter()-t0,
                        objective='Minimum I^2R among tested contours within per-net area and per-load drop bounds',
                        sw_unchanged=True, tracks_unchanged=True)
