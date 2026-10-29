import math, re

VERSION = 'aggressor-v1'


def tr_from(n, K, weights=None):
    from analysis.measures import net_class, load_weights
    G = weights if weights is not None else load_weights()
    for rx, value in sorted(((K.get('specs') or {}).get('tr_ns') or {}).items()):
        try:
            if n == rx or n.split('/')[-1] == rx or re.fullmatch(rx, n.split('/')[-1]):
                return float(value) if value is not None else None
        except re.error: pass
    k = K['net_list'].get(n) or {}

    value = k['tr_ns'] if 'tr_ns' in k else (G['netclasses'].get(net_class(k), {}) or {}).get('tr_ns')
    return float(value) if value is not None else None


def spacing_factor(tr_ns):
    return math.sqrt(max(0.0, 26.0 * 2.0 / max(float(tr_ns), 0.1) - 1.0)) / 5.0

def create(K, weights=None):
    from analysis.measures import load_weights
    G = weights if weights is not None else load_weights()
    net_list = {}; digital = []; current_val = []; sw = []
    for n, k in sorted(K['net_list'].items()):
        category = k.get('category'); tr = tr_from(n, K, G)
        if category == 'gnd': continue
        d = category in ('signal', 'fast') and tr is not None and 0 < tr <= 5.0
        if d: digital.append(n)
        if category in ('current_val', 'supply'): current_val.append(n)
        if category == 'sw': sw.append(n)
        if tr is not None and tr > 0 or category in ('current_val', 'supply', 'sw'):
            net_list[n] = dict(category=category, tr_ns=tr, digital=d, spacing_factor=spacing_factor(tr) if d else 1.0)
    victim = sorted(n for n, k in K['net_list'].items() if k.get('category') == 'analog' or k.get('kelvin') or k.get('sensitive_sw'))
    coils = sorted(r for r, t in K.get('part_list', {}).items() if t.get('function') == 'coil_sw')
    return dict(version=VERSION, net_list=net_list, digital=digital, current_val=current_val, sw=sw, victim=victim, coils=coils)


def read_in(K, weights=None):
    S = K.get('aggressor')
    current = create(K, weights)
    return S if S == current else current


def edges(K, weights=None):
    S = read_in(K, weights)
    return {n: (S['net_list'].get(n) or {}).get('tr_ns') for n in K['net_list']}
