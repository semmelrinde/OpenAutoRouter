from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

VERSION = 'k20-copper_map-v4'


def _pad_groups(pads, spacing=1.5):
    parents = list(range(len(pads)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for i, (_, a) in enumerate(pads):
        for j in range(i):
            if a.distance(pads[j][1]) <= spacing:
                parents[root(i)] = root(j)
    groups = defaultdict(list)
    for i, (ident, _) in enumerate(pads):
        groups[root(i)].append(i)
    return [g for g in groups.values() if len(g) >= 2]


def measure(B, R, K):
    from common.zones import board_geometry
    B = board_geometry(B, R)
    native, model = defaultdict(list), defaultdict(list)
    np, mp = defaultdict(list), defaultdict(list)
    for z in R.get('zones', []):
        if not z.get('filled'): continue
        key = z['net'], z['layer']


        is_native = z.get('native_fill') or (z.get('model') is not True and
            (R.get('copper_bars') or {}).get('model') == 'native refill')
        modeled = not is_native and (z.get('model') or z.get('source') == 'R36-FullCopperModel')
        target, priorities = (model, mp) if modeled else (native, np)
        target[key].append(Polygon(z['pts'], z.get('holes') or []).buffer(0))
        priorities[key].append(z.get('prio'))
    for z in R.get('zone_list', []):
        key = z['net'], z['layer']
        model[key].extend(Polygon(p['outer'], p.get('inner') or []).buffer(0) for p in z.get('polys', []))
        mp[key].append(z.get('prio', -1 if z.get('source') == 'gnd_fill' else 1))
    native_ok = bool(native and (R.get('current_zones_acceptance') or {}).get('applied'))
    n = _measure(B, R, K, native, np, 'native_accepted' if native_ok else
                'filled_export' if native else 'no_zones', native_ok)
    m = _measure(B, R, K, model, mp, 'model' if model else 'no_zones', False)


    out = dict(n if native else m)
    out.update(kicad_native=n, model=m)
    return out
def _measure(B, R, K, fills, priorities, source, native_ok):
    from learning.current_zones import _layers, CURRENT_KINDS
    power = {n for n, v in K.get('net_list', {}).items()
             if v.get('category') in CURRENT_KINDS and not v.get('kelvin') and n not in K.get('gnd', [])}
    from learning.current_zones_topo import last_status
    status = {n: last_status(K, n, B) for n in power}
    without_contract = sorted(n for n in power if status[n] == 'unknown')
    loadless = sorted(n for n in power if status[n] == 'loadless')
    gnd = set(K.get('gnd') or ['GND'])
    fills = {key: unary_union(parts) for key, parts in fills.items()}
    rows = []
    for L in _layers(B):
        for n in sorted(power):
            pads = [(dict(instance=i, ref=p['ref'], num=str(p['num']), x=p['x'], y=p['y']),
                     unary_union([Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]))
                    for i, p in enumerate(B['pads']) if p['net'] == n and (p.get('polys') or {}).get(L)]
            pad = [g for _, g in pads]
            paths = [LineString(t['pts']) for t in R.get('tracks', [])
                     if t['net'] == n and t['layer'] == L and len(t['pts']) >= 2]
            if not pad and not paths: continue
            zone = fills.get((n, L), Polygon())
            pieces = [p for p in getattr(zone, 'geoms', [zone]) if p.geom_type == 'Polygon' and p.area >= .01]


            path = unary_union(paths).difference(unary_union(pad).buffer(.02))
            missing = path.difference(zone.buffer(.008)).length
            groups = _pad_groups(pads)
            bad_groups = 0
            for group in groups:
                members = []
                for i in group:
                    pd = B['pads'][pads[i][0]['instance']]
                    drill = float(pd.get('drill') or 0.0)



                    members.append(pad[i].difference(Point(pd['x'],pd['y']).buffer(drill/2,64)) if drill else pad[i])
                if not any(all(part.buffer(.008).covers(p) for p in members) for part in pieces):
                    bad_groups += 1
            rows.append(dict(net_name=n, layer_name=L, corridor_mm=round(path.length, 4),
                             without_zone_mm=round(missing, 4), pad_groups=len(groups),
                             pad_groups_members=[[pads[i][0] for i in group] for group in groups],
                             pad_groups_without_common_zone=bad_groups, pieces=len(pieces),
                             zones_mm2=round(zone.area, 4)))
    ground_layers = sorted({L for (n, L), f in fills.items() if n in gnd and not f.is_empty})
    bad_priority = []
    for (n, L), vals in sorted(priorities.items()):
        if n not in power: continue
        ground = [v for (gn, layer), vs in priorities.items() if gn in gnd and layer == L for v in vs]
        if ground and (any(v is None for v in vals + ground) or min(vals) <= max(ground)):
            bad_priority.append(dict(net_name=n, layer_name=L))
    length = sum(r['corridor_mm'] for r in rows)
    absent = sum(r['without_zone_mm'] for r in rows)
    from analysis.copper_model import contour
    board = contour(B)
    outer_power = sum(unary_union([f for (n, layer), f in fills.items()
                                  if n in power and layer == L]).intersection(board).area for L in ('F', 'B'))
    return dict(version=VERSION, source=source, kicad_native_accepted=native_ok, per=rows,
                nets_without_load_contract=without_contract, nets_without_load_contract_n=len(without_contract),
                nets_dc_loadless=loadless, nets_dc_loadless_n=len(loadless),
                corridor_mm=round(length, 4), without_zone_mm=round(absent, 4),
                without_zone_share=round(absent/length, 6) if length > 1e-9 else None,
                pad_groups=sum(r['pad_groups'] for r in rows),
                pad_groups_without_common_zone=sum(r['pad_groups_without_common_zone'] for r in rows),
                gnd_zone_layers=ground_layers, gnd_zone_layers_missing=sorted(set(_layers(B))-set(ground_layers)),
                gnd_zone_layers_missing_n=len(set(_layers(B))-set(ground_layers)),
                priority_error=bad_priority, priority_error_n=len(bad_priority),
                board_area_mm2=round(board.area, 4), supply_outer_mm2=round(outer_power,4),
                supply_outer_share=round(outer_power/(2*board.area),6) if board.area else None,
                fillers=sum(r['pieces'] for r in rows),
                hint='Diagnostic; no weighting, no invented GND fill, no DRC/IR acceptance.')


def diagnose(out, B, R, K):
    try:
        out['k20'] = measure(B, R, K)
    except Exception as exc:
        out['k20'] = None
        out.setdefault('diagnose_error', {})['k20'] = repr(exc)[:200]
