""
import math
from shapely.geometry import LineString
from analyse.wackel import ketten, TOL

VERSION = 'r18-form-v2'


def messen(B, R, K=None):
    signale = {n for n, k in (K or {}).get('netze', {}).items()
               if k.get('art') in ('signal', 'analog', 'schnell')}

    totals={name:dict(ketten=0,knicke=0,haarnadeln=0,zickzack=0,knick_ueberschuss=0,
                     laenge_mm=0.,umweg_octi_mm=0.) for name in ('alle','signal')}
    for (n, _), pts in ketten(R):
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
        contribution=dict(ketten=1,knicke=len(angles),haarnadeln=sum(abs(a)>=math.radians(135)-1e-9 for a in angles),
            zickzack=sum(a*b<0 and b*c<0 for a,b,c in zip(cycle,cycle[1:],cycle[2:])),
            knick_ueberschuss=max(0,len(angles)-minimum),laenge_mm=line.length,umweg_octi_mm=max(0.,line.length-bound))
        for name in (('alle','signal') if n in signale else ('alle',)):
            for key,value in contribution.items():totals[name][key]+=value
    for values in totals.values():
        values['knicke_je_10mm']=round(10*values['knicke']/max(values['laenge_mm'],1e-9),6)
        values['laenge_mm']=round(values['laenge_mm'],6)
        values['umweg_octi_mm']=round(values['umweg_octi_mm'],6)
    return dict(version=VERSION, toleranz_mm=TOL, winkel_grad=30, **totals)
