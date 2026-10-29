import copy
import math

from shapely.geometry import Polygon
from analysis.evaluation import comparison
from analysis.topology import topology
from analysis.coupling import coupling
from common.shift import relocate

TOPO = ('parallel_rest_mm', 'star', 'sharp', 'kelvin_close_mm', 'tap_via_in_pad')


def keepout(B, geo, own=()):
    for k in B.get('keepouts', []):
        if k.get('ref') in own: continue
        if not k.get('ref') and not (k.get('prohibitions') or {}).get('part_list', True): continue
        cu_layers = k.get('layers') or k.get('cu_layers') or ['F', 'B']
        cu_layers = {s.replace('.Cu', '') for s in cu_layers}
        for s, g in geo.items():
            if s not in cu_layers and '*.Cu' not in cu_layers and '*' not in cu_layers: continue
            if any(g.intersects(Polygon(q)) for q in k['polys'] if len(q) >= 3): return True
    return False


def rule_values(B, Rt, K):
    t = topology(Rt, B)
    return dict(topology={n: t[n] for n in TOPO}, coupling=coupling(Rt, K)['total_sum'])


def assumption(E0, E1, B0, R0, B1, R1, K, minimum=2.0, price=0.0, rating='v3'):
    if rating == 'v3' and E0.get('score_v3_version') != E1.get('score_v3_version'):
        return dict(delta=None, better=False, reason='different score-v3 versions', rating=rating)
    c = comparison(E0, E1, B0, R0, B1, R1, K, rating=rating)
    a = E0.get('move_design_rules') or rule_values(B0, R0, K)
    b = E1.get('move_design_rules') or rule_values(B1, R1, K)
    worse = [n for n in TOPO if b['topology'][n] > a['topology'][n] + 1e-6]
    if b['coupling'] > a['coupling'] + 1e-6: worse.append('coupling')
    c.update(design_rules_worse=worse, price=price)
    if not c['better'] or worse:
        c['better'] = False
        if worse: c['reason'] = 'Rules worse: ' + ', '.join(worse)
        return c


    if c.get('rating') != 'minimax' and c['delta'] is not None and c['delta'] + price >= -minimum:
        c.update(better=False, reason='Gain does not suffice for minimum gain and movement price')
    return c


def movement(B, k):
    return relocate(B, k['refs'], sides=k.get('sides'))[0]


def octi(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return math.dist(a, b) > 1e-6 and (min(dx, dy) < 1e-6 or abs(dx - dy) < 1e-6)


def route_move(Rt, k):
    R = copy.deepcopy(Rt)
    for m in k.get('route_moves', []):
        obj = R[m['fld']][m['index']]
        if any(obj.get(f) != v for f, v in m['prior'].items()): raise ValueError('local move no longer matches the state')
        obj.update(copy.deepcopy(m['new']))
    return R
