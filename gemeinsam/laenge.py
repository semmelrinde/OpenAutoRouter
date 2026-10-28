""
import collections
import math

import numpy as np
import shapely
from shapely.geometry import Polygon


def pad_flaechen(B):
    ""
    g = collections.defaultdict(list)
    for p in B['pads']:
        n = p.get('net')
        if not n: continue
        for L, qs in (p.get('polys') or {}).items():
            for q in qs:
                if len(q) >= 3: g[(n, L)].append(Polygon(q).buffer(0))
    return {k: shapely.union_all(v) for k, v in g.items()}


def aussen(R, B, F=None):
    ""
    out = [[math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:])] for t in R['tracks']]
    if B is None: return out
    if F is None: F = pad_flaechen(B)
    je = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        if (t['net'], t['layer']) not in F: continue
        for si in range(len(t['pts']) - 1):
            if out[ti][si] > 1e-9: je[(t['net'], t['layer'])].append((ti, si))
    for k in sorted(je):
        idx = je[k]
        lines = shapely.linestrings([[R['tracks'][ti]['pts'][si], R['tracks'][ti]['pts'][si + 1]] for ti, si in idx])
        drin = shapely.length(shapely.intersection(lines, F[k]))
        for (ti, si), d in zip(idx, np.asarray(drin, dtype=float)):
            out[ti][si] = max(0.0, out[ti][si] - float(d))
    return out
