""
import math


def gruppen(refs, inseln, bindungen, fest):
    ""
    refs = set(refs)
    parent = {r: r for r in refs}

    def root(r):
        while parent[r] != r:
            parent[r] = parent[parent[r]]
            r = parent[r]
        return r

    for rs in list(inseln) + list(bindungen):
        rs = sorted(set(rs) & refs)
        for r in rs[1:]: parent[root(r)] = root(rs[0])
    out = {}
    for r in sorted(refs): out.setdefault(root(r), []).append(r)
    return [rs for rs in out.values() if not set(rs) & set(fest)]


def kandidaten(rs, lage, pads, net_refs, gewichte):
    ""
    own = set(rs)
    dx = dy = wsum = 0.0
    for n in sorted(net_refs):
        innen = sorted(own & net_refs[n]); aussen = sorted(net_refs[n] - own)
        if not innen or not aussen: continue
        a = [(x, y) for r in innen for _, net, x, y, _ in pads[r] if net == n]
        b = [(x, y) for r in aussen for _, net, x, y, _ in pads[r] if net == n]
        w = gewichte[n]
        dx += w * (sum(p[0] for p in b) / len(b) - sum(p[0] for p in a) / len(a))
        dy += w * (sum(p[1] for p in b) / len(b) - sum(p[1] for p in a) / len(a))
        wsum += w
    moves = []
    if wsum:
        dx /= wsum; dy /= wsum
        scale = min(1.0, 4.0 / max(math.hypot(dx, dy), 1e-12))
        moves += [(dx * scale * f, dy * scale * f) for f in (1, .5, .25)]
    moves += [(x * step, y * step) for step in (1, .5, .25)
              for x, y in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))]
    for dx, dy in moves:
        if math.hypot(dx, dy) < 1e-8: continue
        yield {r: (lage[r][0] + dx, lage[r][1] + dy, *lage[r][2:]) for r in rs}
