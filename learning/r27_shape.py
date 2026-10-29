import copy

from analysis.topology_trunk import measure


def candidate(B, raw, formed, rules):
    a, b = measure(raw, B), measure(formed, B)
    names = sorted(n for n in a['net_list'].keys() | b['net_list'].keys()
                   if any(b['net_list'].get(n, {}).get(k, 0) > a['net_list'].get(n, {}).get(k, 0)+1e-8
                          for k in ('star_free', 'sharp_free', 'parallel_r27_mm')))
    info = dict(version='r27-shaping-v2', net_list=names, proposed=False, applied=False,
                r27_version=a['version'], r27_raw=a['amount'], r27_shape=b['amount'])
    if not names:
        info['reason'] = 'No R27 degradation through shape'
        return formed, info
    if raw.get('vias') != formed.get('vias'):
        info['reason'] = 'Via geometry changed'
        return formed, info
    out = copy.deepcopy(formed)
    out['tracks'] = [t for t in out['tracks'] if t['net'] not in names] + [copy.deepcopy(t) for t in raw['tracks'] if t['net'] in names]



    from analysis.topology_native import _measurement_routes
    from postprocessing.pulltight_routes import Guard
    from postprocessing.net_shape import ideal_rules
    measured = _measurement_routes(out)
    guard = Guard(B, measured, ideal_rules(B, rules))
    for i, t in enumerate(measured['tracks']):
        if t['net'] not in names: continue
        for j, (p, q) in enumerate(zip(t['pts'], t['pts'][1:])):
            if not guard.clear([p, q], t['w'][j], t['layer'], t['net'], i):
                info.update(reason='SpacingGate', spacing=dict(guard.tests))
                return formed, info
    info.update(proposed=True, spacing=dict(guard.tests))
    return out, info
