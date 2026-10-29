import json, sys, math
from shapely.geometry import Polygon, Point
B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); OUT = sys.argv[3]; REF = sys.argv[4]
a = dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x)
NET = a.get('net_name', '/GND'); ST = float(a.get('spacing', '0.65')); MIN_C = float(a.get('h2h', '0.2')) + float(a.get('drill', '0.2')) + 0.03
VR = float(a.get('via', '0.45')) / 2
added = 0
for p in B['pads']:
    if p['ref'] != REF or p['net'] != NET: continue
    L = p['layers'][0]; poly = Polygon(p['polys'][L][0]); inner = poly.buffer(-VR + 0.05)
    x0, y0, x1, y1 = p['bbox']; long_y = (y1 - y0) >= (x1 - x0)
    n = 1; cand = []
    lo, hi = (y0, y1) if long_y else (x0, x1)
    k = lo + VR
    while k <= hi - VR + 1e-6:
        cand.append((p['x'], k) if long_y else (k, p['y'])); k += ST
    for q in cand:
        if not inner.contains(Point(q)) and not poly.contains(Point(q)): continue
        if any(math.dist(q, (v['x'], v['y'])) < MIN_C for v in R['vias']): continue
        R['vias'].append(dict(net=NET, x=round(q[0], 4), y=round(q[1], 4))); added += 1
json.dump(R, open(OUT, 'w'))
print('additional vias', added, 'in', REF)
