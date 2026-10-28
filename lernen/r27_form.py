""
import copy

from analyse.topologie_stamm import messen


def kandidat(B, raw, formed, rules):
    a, b = messen(raw, B), messen(formed, B)
    names = sorted(n for n in a['netze'].keys() | b['netze'].keys()
                   if any(b['netze'].get(n, {}).get(k, 0) > a['netze'].get(n, {}).get(k, 0)+1e-8
                          for k in ('stern_frei', 'spitz_frei', 'parallel_r27_mm')))
    info = dict(version='r27-form-v2', netze=names, vorgeschlagen=False, angewendet=False,
                r27_version=a['version'], r27_roh=a['wert'], r27_form=b['wert'])
    if not names:
        info['grund'] = 'Keine R27-Verschlechterung durch Form'
        return formed, info
    if raw.get('vias') != formed.get('vias'):
        info['grund'] = 'Via-Geometrie geaendert'
        return formed, info
    out = copy.deepcopy(formed)
    out['tracks'] = [t for t in out['tracks'] if t['net'] not in names] + [copy.deepcopy(t) for t in raw['tracks'] if t['net'] in names]



    from analyse.topologie_native import _measurement_routes
    from nachbearbeitung.pulltight_routes import Guard
    from nachbearbeitung.netzform import ideal_rules
    measured = _measurement_routes(out)
    guard = Guard(B, measured, ideal_rules(B, rules))
    for i, t in enumerate(measured['tracks']):
        if t['net'] not in names: continue
        for j, (p, q) in enumerate(zip(t['pts'], t['pts'][1:])):
            if not guard.clear([p, q], t['w'][j], t['layer'], t['net'], i):
                info.update(grund='Abstandstor', abstand=dict(guard.tests))
                return formed, info
    info.update(vorgeschlagen=True, abstand=dict(guard.tests))
    return out, info
