""
import math
from platzierung.insel_physik import huelle


def schleife(cpads, zpads, netz, pins, gnd, via):
    cv = [p for p in cpads if p[1] == netz]
    zv = [p for p in zpads if p[1] == netz and p[0] in pins]
    cg = [p for p in cpads if p[1] in gnd]
    zg = [p for p in zpads if p[1] in gnd]
    best = None
    for c in cv:
        for z in zv:
            for m in cg:
                grounds = [q for q in zg if q[1] == m[1]]
                if grounds:


                    q = min(grounds, key=lambda p: (math.dist(p[2:4], z[2:4]), str(p[0])))
                    ds = math.dist(c[2:4], z[2:4]) + via(c[4], z[4])
                    dg = math.dist(m[2:4], q[2:4]) + via(m[4], q[4])
                    area = huelle([p[2:4] for p in (z, c, m, q)])
                    cost = 4 * ds + 2 * dg + .25 * area
                    if best is None or cost < best: best = cost
    return best
