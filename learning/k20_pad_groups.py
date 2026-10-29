import re

from shapely.geometry import Polygon
from shapely.ops import unary_union


def core_set(B, R, K, net, layer, vacant, spacing=1.5):
    if (K.get('net_list', {}).get(net) or {}).get('kelvin'):
        return Polygon()
    taps = K.get('taps') or {}
    if (taps.get(net) if isinstance(taps, dict) else net in taps):
        return Polygon()
    sense = re.compile(r'SENS|SNS|KELVIN|\bFB\b|FEEDBACK|REMOTE', re.I)
    if any(p['net'] == net and sense.search(str(p.get('pinfunc') or '')+' '+str(p.get('pintype') or ''))
           for p in B['pads']):


        return Polygon()
    pads = []
    for p in B['pads']:
        if p['net'] != net: continue
        shapes = [Polygon(q).buffer(0) for q in p.get('polys', {}).get(layer, []) if len(q) >= 3]
        g = unary_union(shapes)
        if not g.is_empty: pads.append((p['ref'],str(p['num']),p['x'],p['y'],g))
    pads.sort(key=lambda p: p[:4])
    parent = list(range(len(pads)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    edges = sorted((a[-1].distance(b[-1]),i,j) for i,a in enumerate(pads)
                   for j,b in enumerate(pads[:i]) if a[-1].distance(b[-1]) <= spacing)
    out = []
    for distance,i,j in edges:
        if root(i) == root(j): continue
        a,b = pads[i][-1],pads[j][-1]
        bridge = unary_union([a,b]).convex_hull

        added = bridge.difference(unary_union([a,b]))
        if not vacant.buffer(1e-8).covers(added): continue
        parent[root(i)] = root(j)
        if added.area > 1e-8:
            from learning.current_zones import tangential_rounds
            out.append(tangential_rounds(bridge, a.union(b), vacant.union(a).union(b), radius=.15))
    return unary_union(out)
