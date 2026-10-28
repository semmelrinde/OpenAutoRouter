""
from collections import Counter
import copy


def iu(ring):
    points = [tuple(round(v*1_000_000) for v in p) for p in ring]
    if any(any(abs(v/1_000_000-q) > 1e-12 for v,q in zip(p, source))
           for p,source in zip(points, ring)):
        raise ValueError('Keine nativen IU-Koordinaten')
    return points[:-1] if len(points)>1 and points[0] == points[-1] else points


def area(ring):
    return sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(ring,ring[1:]+ring[:1]))


def kanten(rings):
    out = Counter()
    for ring in rings:
        for a,b in zip(ring,ring[1:]+ring[:1]):
            if a == b:
                continue
            key = tuple(sorted((a,b)))
            out[key] += 1 if a < b else -1
    return {k:v for k,v in out.items() if v}


def zyklen(ring):
    stack, positions, cycles = [], {}, []
    for p in ring + ring[:1]:
        if p in positions:
            index = positions[p]
            cycle = stack[index:]
            if len(cycle) >= 3 and area(cycle):
                cycles.append(cycle)
            for q in stack[index+1:]:
                del positions[q]
            stack = stack[:index+1]
        else:
            positions[p] = len(stack)
            stack.append(p)
    return cycles


def zerlegen(poly):
    from shapely.geometry import Polygon
    original = [iu(poly['aussen'])] + [iu(h) for h in poly['loecher']]
    source_area = area(original[0])
    if not source_area:
        raise ValueError('Aussenring ohne gerichtete Flaeche')
    sign = 1 if source_area > 0 else -1


    for h in original[1:]:
        if area(h)*sign > 0:
            h.reverse()
    cycles = [c for ring in original for c in zyklen(ring)]
    outer = [c for c in cycles if area(c)*sign > 0]
    holes = [c for c in cycles if area(c)*sign < 0]
    shapes = [Polygon(c) for c in outer]
    assigned = [[] for _ in outer]
    for h in holes:
        shape = Polygon(h)
        owners = [i for i,s in enumerate(shapes) if s.covers(shape)]
        if not owners:
            raise ValueError('Lochzyklus ohne Aussenring')
        assigned[min(owners,key=lambda i:abs(area(outer[i])))].append(h)
    final = [c for o,hs in zip(outer,assigned) for c in [o]+hs]
    if sum(map(area,original)) != sum(map(area,final)) or kanten(original) != kanten(final):
        raise ValueError('Native Kanten/Flaeche durch Zerlegung veraendert')
    def mm(r):
        return [[x/1_000_000,y/1_000_000] for x,y in r]
    return [dict(aussen=mm(o),loecher=[mm(h) for h in hs]) for o,hs in zip(outer,assigned)]


def kupfer(data):
    out = copy.deepcopy(data)
    def replace(ps):
        return [q for p in ps for q in zerlegen(p)]
    out['outline'] = replace(out['outline'])
    for key in ('zones','formen'):
        for z in out[key]:
            z['polys'] = replace(z['polys'])
    for p in out['pads']:
        p['polys'] = {l:replace(ps) for l,ps in p['polys'].items()}
        p['hole_polys'] = replace(p['hole_polys'])
    return out
