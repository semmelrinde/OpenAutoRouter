import json, sys, math
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union

bj, rin, rout = sys.argv[1:4]
A = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
MAXD = float(A.get('max', 0.6)); VD = float(A.get('via', 0.45)); DR = float(A.get('drill', 0.2))
CLR = float(A.get('clr', 0.095)); H2H = float(A.get('h2h', 0.2))
RAD = float(A.get('radius', 1.2))
GND = A.get('gnd', '/GND')
B = json.load(open(bj)); R = json.load(open(rin))
KEEPOUT = [unary_union([Polygon(q) for q in v['polys']]) for v in B.get('via_keepout', [])]
NT = {v['ref'] for v in B.get('via_keepout', [])}


def keepout_ok(x, y):
    return all(g.distance(Point(x, y)) >= H2H + DR / 2 + 0.005 for g in KEEPOUT)


def vacant(x, y):
    pt = Point(x, y)
    for w in R['vias']:
        if math.hypot(w['x'] - x, w['y'] - y) < DR + H2H + 0.005: return False
        if w['net'] != GND and math.hypot(w['x'] - x, w['y'] - y) < VD + CLR + 0.005: return False
    for t in R['tracks']:
        if t['net'] == GND: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if LineString([a, b]).distance(pt) < VD / 2 + w / 2 + CLR: return False
    for p in B['pads']:
        if p['net'] == GND or p['ref'].startswith('NTBAR_'): continue
        for L in p['layers']:
            if L in ('F', 'B') and unary_union([Polygon(q) for q in p['polys'][L]]).distance(pt) < VD / 2 + CLR + 0.005: return False
    return True


n = 0
for p in B['pads']:
    if not ((p['ref'] in NT or p['ref'].startswith('NT_')) and p['net'] == GND): continue
    d0 = min([math.hypot(v['x'] - p['x'], v['y'] - p['y']) for v in R['vias'] if v['net'] == GND] + [9])
    if d0 <= MAXD: continue
    core = unary_union([Polygon(q) for L in p['layers'] for q in p['polys'][L]]).buffer(-0.05)
    best = None
    for r in [0.0] + [k * 0.025 for k in range(1, int(RAD / 0.025) + 1)]:
        for k in range(1 if r == 0 else 32):
            a = 2 * math.pi * k / 32; x, y = p['x'] + r * math.cos(a), p['y'] + r * math.sin(a)
            if (core.contains(Point(x, y)) or r > 0.2) and vacant(x, y) and keepout_ok(x, y): best = (x, y); break
        if best: break
    if not best: print('no room in the pad', p['ref']); continue
    R['vias'].append(dict(net=GND, x=round(best[0], 4), y=round(best[1], 4))); n += 1
    print('GND via in %s.%s at (%.3f, %.3f), previously nearest %.2f mm' % (p['ref'], p['num'], best[0], best[1], d0))
json.dump(R, open(rout, 'w')); print('assigned', n, '->', rout)
