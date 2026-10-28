""
import math


def bezugsflaechen(board, gnd, routes=None, netze=None):
    ""
    if not board.get('plane_domains'): return None
    import json
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    with open(board['plane_domains']) as f: data = json.load(f)
    declared = bezugsnetze(routes, gnd, netze) if routes is not None else None
    groups = {}
    for q in data['zones']:
        if q['net'] in (declared.get(q['layer'], []) if declared is not None else gnd):
            groups.setdefault(q['layer'], []).append(Polygon(q['pts'], q.get('holes', [])).buffer(0))
    return {l: unary_union(gs) for l, gs in groups.items()}


def bezugsnetze(routes, gnd, netze=None):
    ""
    option = (routes.get('optionen') or {}).get('gndlagen', '')
    refs0 = routes.get('gndlagen') or []
    refs0 = refs0.split(',') if isinstance(refs0, str) else refs0
    from analyse.flaechen import aktiv
    if aktiv(routes, {}) and ('=' in option or any('=' in q for q in refs0)):
        from analyse.flaechen import zuordnung
        raw = option.split(',') if option else refs0
        L = max([int(q.split('=')[0][2:]) + 2 for q in raw if q.startswith('In') and q.split('=')[0][2:].isdigit()] + [2])
        board = dict(lagen=L, pads=[dict(net=n) for n in sorted(netze or gnd)])
        return {l: nn for l, nn in zuordnung(board, routes, dict(gnd=sorted(gnd))).items() if nn and set(nn) <= set(gnd)}
    declared = {q.split('=', 1)[0]: q.split('=', 1)[1].split('|') for q in option.split(',') if '=' in q}
    refs = routes.get('gndlagen', option.split(','))
    if isinstance(refs, str): refs = refs.split(',')
    allnets = set(netze or gnd)
    def netz(n):
        if n in allnets: return n
        if '/' + n in allnets: return '/' + n
        return n.lstrip('/') if n.lstrip('/') in allnets else n
    out = {}
    for q in refs:
        name, _, nets = q.partition('=')
        domains = nets.split('|') if nets else declared.get(name, [])
        if declared and not domains: continue
        if name and (not domains or {netz(n) for n in domains} <= set(gnd)):
            out[name] = sorted({netz(n) for n in domains} if domains else set(gnd))
    return out


def bezugslagen(routes, gnd, netze=None):
    return list(bezugsnetze(routes, gnd, netze))


def typisch(n):
    ds = {2: [1.5], 4: [0.0994, 1.265, 0.0994],
          6: [0.0994, 0.55, 0.1088, 0.55, 0.0994],
          8: [0.0994, 0.21, 0.0994, 0.55, 0.0994, 0.21, 0.0994]}
    if n not in ds: raise ValueError('Aufbau: nur 2/4/6/8 Lagen')
    return dict(modell='typisch-jlc-aehnlich-v2', quelle='Trainingsmodell, 4/6 nach https://jlcpcb.com/impedance; 8 geschaetzt; Fertiger bestaetigt separat',
                dielektrika=ds[n], er_lagen=[4.1, 4.6, 4.16, 4.6, 4.1] if n == 6 else [4.1 if i % 2 == 0 else 4.6 for i in range(n - 1)],
                kupfer_mm=[0.035] + [0.0152] * (n - 2) + [0.035],
                gndlagen=[] if n == 2 else ['In1'] if n == 4 else ['In1', 'In4'] if n == 6 else ['In1', 'In3', 'In6'])


def normalisieren(aufbau, n):
    ""
    a = dict(typisch(n), **(aufbau or {}))
    if aufbau and 'er' in aufbau and not aufbau.get('er_lagen'):
        a['er_lagen'] = [aufbau['er']] * (n - 1)
    for key in ('dielektrika', 'er_lagen', 'kupfer_mm'):
        a[key] = [float(x) for x in a[key]]
    if len(a['dielektrika']) != n - 1 or len(a['er_lagen']) != n - 1 or len(a['kupfer_mm']) != n:
        raise ValueError('Aufbau: falsche Zahl Dielektrika/er/Kupferlagen')
    if not all(math.isfinite(x) and x > 0 for key in ('dielektrika', 'er_lagen', 'kupfer_mm') for x in a[key]):
        raise ValueError('Aufbau: Werte muessen endlich und positiv sein')
    return a


def modelle(aufbau, n, gndlagen):
    ""
    a = normalisieren(aufbau, n)
    ds, er, cu = a['dielektrika'], a['er_lagen'], a['kupfer_mm']
    names = ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']
    refs = set(gndlagen); out = {}
    if refs - set(names): raise ValueError('Aufbau: unbekannte Bezugslage')
    for i, name in enumerate(names):
        if name in refs: continue
        ob = [j for j in range(i) if names[j] in refs]
        un = [j for j in range(i + 1, n) if names[j] in refs]
        if i in (0, n - 1):
            ref = un[0] if i == 0 and un else ob[-1] if i == n - 1 and ob else None
            if ref is None: continue
            inds = list(range(min(i, ref), max(i, ref)))
            if i == 0: inds.reverse()
            layers = [(ds[j] + (cu[j] if i == 0 and j > i else cu[j + 1] if i == n - 1 and j + 1 < i else 0.0), er[j]) for j in inds]
            out[name] = dict(h=sum(d for d, _ in layers), t=cu[i], er=layers[-1][1], schichten=layers,
                             ref=[names[ref]], typ='mikrostreifen')
        elif ob and un:
            lo, hi = ob[-1], un[0]
            layers = []
            for j in range(lo, hi):
                layers.append((ds[j], er[j]))
                if j + 1 < hi: layers.append((cu[j + 1], er[j]))
            h = sum(ds[lo:i]) + sum(cu[lo + 1:i])
            out[name] = dict(h=h, t=cu[i], er=er[i - 1], oben=sum(x[0] for x in layers) - h - cu[i],
                             schichten=layers, ref=[names[lo], names[hi]], typ='stripline')
    return out
