import json
import sys

from shapely.geometry import Polygon
from shapely.ops import unary_union


def area_nets(gnd_layers, gnd=()):
    out = {n for q in (gnd_layers or '').split(',') if '=' in q for n in q.split('=', 1)[1].split('|') if n}
    if any(q and '=' not in q for q in (gnd_layers or '').split(',')): out |= set(gnd)
    return out | {'/' + n.lstrip('/') for n in out} | {n.lstrip('/') for n in out}


def _layers(z):
    return z.get('native_layers') or [z['layer']]


def _contour(z):
    return unary_union([Polygon(c['pts'], c.get('holes', [])).buffer(0) for c in (z.get('native_outlines') or [z])])


def group_list(B, Z, gnd_layers='', gnd=()):
    eben = area_nets(gnd_layers, gnd)
    raw_data = {}
    for z in Z:
        n = z.get('net')
        if not n or n in eben: continue
        for L in _layers(z):
            g = _contour(z)
            for w in Z:
                if w is not z and w.get('net') != n and L in _layers(w) and w.get('prio', 0) > z.get('prio', 0): g = g.difference(_contour(w))
            pieces = [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]
            pads = [p for p in B['pads'] if p.get('net') == n and L in (p.get('polys') or {}) and not p.get('board_copper')]
            for q in pieces:
                gr = sorted({'%s:%s' % (p['ref'], p['num']) for p in pads
                             if any(len(r) >= 3 and Polygon(r).buffer(0).intersects(q) for r in p['polys'][L])})
                if len(gr) >= 2: raw_data.setdefault(n, []).append(gr)
    off = {}
    for n, gs in sorted(raw_data.items()):
        sets = []
        for g in gs:
            s = set(g); rest = []
            for m in sets:
                if m & s: s |= m
                else: rest.append(m)
            sets = rest + [s]
        off[n] = sorted(sorted(m) for m in sets)
    return off


def main():
    bj, zj, out = sys.argv[1:4]
    args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    G = group_list(json.load(open(bj)), json.load(open(zj)), args.get('gnd_layers', ''))
    json.dump(G, open(out, 'w'), indent=1, sort_keys=True)
    print('Zone groups: %d nets, %d groups, %d pads' % (len(G), sum(len(v) for v in G.values()), sum(len(g) for v in G.values() for g in v)))


if __name__ == '__main__':
    main()
