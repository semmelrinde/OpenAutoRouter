import json, sys, collections
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

CU_MIN, MK_HINT = 0.15, 0.25

def geom(x, k):
    return unary_union([Polygon(p).buffer(0) for p in x[k]]) if x.get(k) else None

def check(d):
    res = []
    for side in 'FB':
        P = [x for x in d if x['side'] == side]
        for k, lim in (('cu', CU_MIN), ('mk', MK_HINT)):
            Q = [(x, geom(x, k)) for x in P if x.get(k)]
            t = STRtree([g for _, g in Q])
            for i, (a, ga) in enumerate(Q):
                for j in t.query(ga.buffer(lim)):
                    j = int(j)
                    if j <= i:
                        continue
                    b, gb = Q[j]
                    if a['net'] == b['net'] and a['net'] != '':
                        continue
                    g = ga.distance(gb)
                    if g < lim:
                        res.append(dict(category=k, side=side, gap=round(g, 3), a=f"{a['ref']}.{a['pad']}", b=f"{b['ref']}.{b['pad']}",
                                        fa=a['fpn'], fb=b['fpn'], same=a['ref'] == b['ref'], na=a['net'], nb=b['net']))
    return res

if __name__ == '__main__':
    res = check(json.load(open(sys.argv[1])))
    if len(sys.argv) > 2:
        json.dump(res, open(sys.argv[2], 'w'), indent=1)
    agg = collections.defaultdict(lambda: [9, 0, set()])
    for r in res:
        key = (r['category'], r['fa'] if r['same'] else r['fa'] + ' <-> ' + r['fb'])
        v = agg[key]; v[0] = min(v[0], r['gap']); v[1] += 1; v[2].add(r['a'].split('.')[0])
    faults = 0
    for (category, fp), (g, n, refs) in sorted(agg.items(), key=lambda kv: (kv[0][0], kv[1][0])):
        level = 'ERROR  ' if category == 'cu' else 'Hint'
        faults += category == 'cu'
        print(f"{level} {'Copper' if category == 'cu' else 'Web   '} min {g:.3f} mm  n={n:3}  {fp}  [{', '.join(sorted(refs))[:80]}]")
    print('OK: no copper clearance below 0.15 mm' if not faults else f'{faults} Footprint(s) with copper clearance below 0.15 mm')
    sys.exit(1 if faults else 0)
