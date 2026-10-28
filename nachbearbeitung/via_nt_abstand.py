""
import json, sys, math
import shapely
from shapely.geometry import Polygon, Point, LineString

bj, rin, rout = sys.argv[1:4]
A = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
HCU = float(A.get('hole_cu', 0.2)); DR = float(A.get('drill', 0.2)); VD = float(A.get('via', 0.45))
CLR = float(A.get('clr', 0.095)); CLRP = float(A.get('clrpad', 0.10)); H2H = float(A.get('h2h', 0.2)); MAXD = float(A.get('max', 0.2))
NEED = HCU + DR / 2 + 0.005
B = json.load(open(bj)); R = json.load(open(rin))
LAG = sorted({L for p in B['pads'] for L in p['layers']} | {t['layer'] for t in R['tracks']})
sperre = {v['ref']: shapely.union_all([Polygon(p) for p in v['polys']]) for v in B.get('via_sperre', [])}
ntpad = {}
for p in B['pads']:
    if p['ref'] in sperre:
        for L, qs in p['polys'].items(): ntpad.setdefault(p['ref'], []).append(shapely.union_all([Polygon(q) for q in qs]).buffer(-0.05))
padg = {L: [] for L in LAG}
for p in B['pads']:
    for L in p['layers']:
        if L in padg: padg[L].append((p['net'], shapely.union_all([Polygon(q) for q in p['polys'][L]])))


def same(a, b): return abs(a[0] - b[0]) < 1e-3 and abs(a[1] - b[1]) < 1e-3


def ok_at(vi, x, y):
    v = R['vias'][vi]; net = v['net']; pt = Point(x, y)
    for ref, poly in sperre.items():
        if poly.distance(pt) < NEED and not any(q.contains(pt) for q in ntpad.get(ref, [])): return False
    for j, w in enumerate(R['vias']):
        if j != vi and math.hypot(w['x'] - x, w['y'] - y) < DR + H2H + 0.005: return False
        if j != vi and w['net'] != net and math.hypot(w['x'] - x, w['y'] - y) < VD + CLR + 0.005: return False
    for L in LAG:
        for n, g in padg[L]:
            if n != net and g.distance(pt) < VD / 2 + CLRP: return False

    moved = []
    for t in R['tracks']:
        if t['net'] != net: continue
        for k, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if same(a, (v['x'], v['y'])) or same(b, (v['x'], v['y'])):
                a2 = (x, y) if same(a, (v['x'], v['y'])) else tuple(a); b2 = (x, y) if same(b, (v['x'], v['y'])) else tuple(b)
                moved.append((t['layer'], LineString([a2, b2]) if not same(a2, b2) else Point(a2), t['w'][k]))
    for t in R['tracks']:
        if t['net'] == net: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            seg = LineString([a, b])
            if seg.distance(pt) < VD / 2 + w / 2 + CLR: return False
            for L, g, wm in moved:
                if L == t['layer'] and seg.distance(g) < w / 2 + wm / 2 + CLR: return False
    for L, g, wm in moved:
        for n, pg in padg.get(L, []):
            if n != net and pg.distance(g) < wm / 2 + CLRP: return False
        for j, w in enumerate(R['vias']):
            if j != vi and w['net'] != net and Point(w['x'], w['y']).distance(g) < VD / 2 + wm / 2 + CLR: return False
    return True


changed = 0
for vi, v in enumerate(R['vias']):
    pt = Point(v['x'], v['y'])
    bad = [ref for ref, poly in sperre.items() if poly.distance(pt) < NEED and not any(q.contains(pt) for q in ntpad.get(ref, []))]
    if not bad: continue
    best = None
    steps = int(MAXD / 0.005)
    for r in range(1, steps + 1):
        d = r * 0.005
        for k in range(32):
            a = 2 * math.pi * k / 32; x, y = v['x'] + d * math.cos(a), v['y'] + d * math.sin(a)
            if ok_at(vi, x, y): best = (x, y); break
        if best: break
    if not best:
        print('KEIN Platz fuer Via', v, 'an', bad); continue
    ox, oy = v['x'], v['y']; v['x'], v['y'] = round(best[0], 4), round(best[1], 4)
    for t in R['tracks']:
        if t['net'] != v['net']: continue
        t['pts'] = [[v['x'], v['y']] if same(p, (ox, oy)) else p for p in t['pts']]
    changed += 1
    print('Via %s (%.3f, %.3f) -> (%.3f, %.3f), %.3f mm, Steg %s' % (v['net'], ox, oy, v['x'], v['y'], math.hypot(v['x'] - ox, v['y'] - oy), ','.join(bad)))
json.dump(R, open(rout, 'w'))
print('verschoben', changed, '->', rout)
