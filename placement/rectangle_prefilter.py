import numpy as np


class Prefilter:
    def __init__(self):
        self._cache = {}

    def _rectangle(self, g):
        key = id(g)
        cached = self._cache.get(key)
        if cached is None or cached[0] is not g:
            cached = (g, g.geom_type == 'Polygon' and g.equals(g.envelope))
            self._cache[key] = cached
        return cached[1]

    def hits(self, part, st, bel, ignore):
        local, _ = part._rot(st[2], st[3])
        for side, g in local.items():
            if not self._rectangle(g):
                continue
            bb = bel._bbox(side)
            if not len(bb):
                continue
            b = g.bounds
            x, y = st[:2]
            eps = 1e-8
            hits = np.nonzero((bb[:, 0] < b[2] + x - eps) & (bb[:, 2] > b[0] + x + eps)
                              & (bb[:, 1] < b[3] + y - eps) & (bb[:, 3] > b[1] + y + eps))[0]
            for i in hits:
                ref, other = bel.items[side][i]
                if ref not in ignore and self._rectangle(other):
                    return True
        return False
