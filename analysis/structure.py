import math


def reference_planes(board, gnd, routes=None, net_list=None):
    if not board.get('plane_domains'): return None
    import json
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    with open(board['plane_domains']) as f: data = json.load(f)
    declared = reference_nets(routes, gnd, net_list) if routes is not None else None
    groups = {}
    for q in data['zones']:
        if q['net'] in (declared.get(q['layer'], []) if declared is not None else gnd):
            groups.setdefault(q['layer'], []).append(Polygon(q['pts'], q.get('holes', [])).buffer(0))
    return {l: unary_union(gs) for l, gs in groups.items()}


def reference_nets(routes, gnd, net_list=None):
    option = (routes.get('options') or {}).get('gnd_layers', '')
    refs0 = routes.get('gnd_layers') or []
    refs0 = refs0.split(',') if isinstance(refs0, str) else refs0
    from analysis.areas import active
    if active(routes, {}) and ('=' in option or any('=' in q for q in refs0)):
        from analysis.areas import assignment
        raw = option.split(',') if option else refs0
        L = max([int(q.split('=')[0][2:]) + 2 for q in raw if q.startswith('In') and q.split('=')[0][2:].isdigit()] + [2])
        board = dict(cu_layers=L, pads=[dict(net=n) for n in sorted(net_list or gnd)])
        return {l: nn for l, nn in assignment(board, routes, dict(gnd=sorted(gnd))).items() if nn and set(nn) <= set(gnd)}
    declared = {q.split('=', 1)[0]: q.split('=', 1)[1].split('|') for q in option.split(',') if '=' in q}
    refs = routes.get('gnd_layers', option.split(','))
    if isinstance(refs, str): refs = refs.split(',')
    allnets = set(net_list or gnd)
    def net_name(n):
        if n in allnets: return n
        if '/' + n in allnets: return '/' + n
        return n.lstrip('/') if n.lstrip('/') in allnets else n
    out = {}
    for q in refs:
        name, _, nets = q.partition('=')
        domains = nets.split('|') if nets else declared.get(name, [])
        if declared and not domains: continue
        if name and (not domains or {net_name(n) for n in domains} <= set(gnd)):
            out[name] = sorted({net_name(n) for n in domains} if domains else set(gnd))
    return out


def reference_layers(routes, gnd, net_list=None):
    return list(reference_nets(routes, gnd, net_list))


def typical(n):
    ds = {2: [1.5], 4: [0.0994, 1.265, 0.0994],
          6: [0.0994, 0.55, 0.1088, 0.55, 0.0994],
          8: [0.0994, 0.21, 0.0994, 0.55, 0.0994, 0.21, 0.0994]}
    if n not in ds: raise ValueError('Stackup: only 2/4/6/8 layers')
    return dict(model='typical-jlc-similar-v2', source='Training model, 4/6 per https://jlcpcb.com/impedance; 8 estimated; manufacturer confirms separately',
                dielectrics=ds[n], er_layers=[4.1, 4.6, 4.16, 4.6, 4.1] if n == 6 else [4.1 if i % 2 == 0 else 4.6 for i in range(n - 1)],
                copper_mm=[0.035] + [0.0152] * (n - 2) + [0.035],
                gnd_layers=[] if n == 2 else ['In1'] if n == 4 else ['In1', 'In4'] if n == 6 else ['In1', 'In3', 'In6'])


def normalize(structure, n):
    a = dict(typical(n), **(structure or {}))
    if structure and 'er' in structure and not structure.get('er_layers'):
        a['er_layers'] = [structure['er']] * (n - 1)
    for key in ('dielectrics', 'er_layers', 'copper_mm'):
        a[key] = [float(x) for x in a[key]]
    if len(a['dielectrics']) != n - 1 or len(a['er_layers']) != n - 1 or len(a['copper_mm']) != n:
        raise ValueError('Stackup: wrong number of dielectrics/er/copper layers')
    if not all(math.isfinite(x) and x > 0 for key in ('dielectrics', 'er_layers', 'copper_mm') for x in a[key]):
        raise ValueError('Stackup: values must be finite and positive')
    return a


def models(structure, n, gnd_layers):
    a = normalize(structure, n)
    ds, er, cu = a['dielectrics'], a['er_layers'], a['copper_mm']
    names = ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']
    refs = set(gnd_layers); out = {}
    if refs - set(names): raise ValueError('Stackup: unknown reference layer')
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
            out[name] = dict(h=sum(d for d, _ in layers), t=cu[i], er=layers[-1][1], layers=layers,
                             ref=[names[ref]], type_='microstrip')
        elif ob and un:
            lo, hi = ob[-1], un[0]
            layers = []
            for j in range(lo, hi):
                layers.append((ds[j], er[j]))
                if j + 1 < hi: layers.append((cu[j + 1], er[j]))
            h = sum(ds[lo:i]) + sum(cu[lo + 1:i])
            out[name] = dict(h=h, t=cu[i], er=er[i - 1], upper=sum(x[0] for x in layers) - h - cu[i],
                             layers=layers, ref=[names[lo], names[hi]], type_='stripline')
    return out
