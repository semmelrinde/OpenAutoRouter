import math
from shapely.geometry import LineString
from analysis.wobble import chain_list, TOL

VERSION = 'r18-shaping-v2'


def measure(B, R, K=None):
    signals = {n for n, k in (K or {}).get('net_list', {}).items()
               if k.get('category') in ('signal', 'analog', 'fast')}

    totals={name:dict(chain_list=0,bends=0,hairpins=0,zigzag=0,bend_excess=0,
                     length_mm=0.,detour_octi_mm=0.) for name in ('all_items','signal')}
    for (n, _), pts in chain_list(R):
        line = LineString(pts); ps = list(line.simplify(TOL, preserve_topology=False).coords)
        angles = []
        ring = len(ps) > 3 and ps[0] == ps[-1]
        triples = zip(ps, ps[1:], ps[2:])
        if ring:
            ps = ps[:-1]
            triples = zip(ps[-1:] + ps[:-1], ps, ps[1:] + ps[:1])
        for a, b, c in triples:
            d = (math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(b[1]-a[1], b[0]-a[0]) + math.pi) % (2*math.pi)-math.pi
            if abs(d) >= math.radians(30)-1e-9: angles.append(d)
        dx, dy = abs(pts[-1][0]-pts[0][0]), abs(pts[-1][1]-pts[0][1])
        bound = max(dx, dy) + (math.sqrt(2)-1)*min(dx, dy)
        minimum = int(min(dx, dy) > TOL and abs(dx-dy) > TOL)
        cycle = angles + angles[:2] if ring else angles
        contribution=dict(chain_list=1,bends=len(angles),hairpins=sum(abs(a)>=math.radians(135)-1e-9 for a in angles),
            zigzag=sum(a*b<0 and b*c<0 for a,b,c in zip(cycle,cycle[1:],cycle[2:])),
            bend_excess=max(0,len(angles)-minimum),length_mm=line.length,detour_octi_mm=max(0.,line.length-bound))
        for name in (('all_items','signal') if n in signals else ('all_items',)):
            for key,value in contribution.items():totals[name][key]+=value
    for values in totals.values():
        values['bends_per_10mm']=round(10*values['bends']/max(values['length_mm'],1e-9),6)
        values['length_mm']=round(values['length_mm'],6)
        values['detour_octi_mm']=round(values['detour_octi_mm'],6)
    return dict(version=VERSION, tolerance_mm=TOL, angle_deg=30, **totals)
