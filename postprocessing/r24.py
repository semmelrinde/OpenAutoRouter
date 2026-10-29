from collections import defaultdict
import copy
import json
import math

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, nearest_points
from shapely.strtree import STRtree

from postprocessing.smooth45 import _octi, _merge

VERSION = 'r24-chain_list-v5'
EPS = 1e-7
FLAGS = ('fix', 'kelvin', 'link_chain', 'current_web', 'current_via', 'ret_via')


def _meta(t):
    return {k: v for k, v in t.items() if k not in ('pts', 'w', 'items')}


def _length_guard(K):

    from analysis.length_impedance import length_groups
    protected = {n for g in length_groups(K).values() for n in g['net_list']}


    for n, meta in (K.get('net_list') or {}).items():
        if meta.get('pair'): protected.update((n, meta['pair']))
    return protected


def _protection(t, K, length_nets=None):
    if length_nets is None: length_nets = _length_guard(K)
    n = (K.get('net_list') or {}).get(t.get('net'), {})
    return bool(any(t.get(k) for k in FLAGS) or t.get('net') in length_nets or n.get('pair') or n.get('length_group') or
                n.get('tolerance_mm') is not None or
                any(i[0] != 'L' for i in t.get('items', [])))


def _layers(B):
    n = int(B.get('cu_layers', 2))
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def _zones(R):
    return list(R.get('zones') or []) + list(R.get('zone_list') or [])


def _zone_polys(z):
    if z.get('polys'):
        return [Polygon(p['outer'], p.get('hole_list') or p.get('inner') or []).buffer(0) for p in z['polys']]
    if (z.get('filled') or z.get('model')) and z.get('pts'):
        return [Polygon(z['pts'], z.get('holes') or []).buffer(0)]
    return []


def _terminals(B, R, n, L):
    out = []
    for p in B.get('pads', []):
        if p.get('net') == n:
            out.extend(Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3)
    vd = float(str((R.get('options') or {}).get('via', '0.45/0.20')).split('/')[0])
    for v in R.get('vias', []):
        if v.get('net') == n and L in (v.get('layers') or _layers(B)):
            out.append(Point(v['x'], v['y']).buffer(float(v.get('d', v.get('diameter', vd))) / 2, 16))

    for z in _zones(R):
        if z.get('net') == n and z.get('layer') == L:
            out.extend(_zone_polys(z))
    return unary_union(out)


def _graph(B, R, K, n, L, tracks, length_nets):
    raw = []
    for ti, t in tracks:
        protected = _protection(t, K, length_nets)
        meta = _meta(t)
        signature = json.dumps(meta, sort_keys=True)
        for a, b, w in zip(t['pts'], t['pts'][1:], t['w']):
            a, b = tuple(a), tuple(b)
            if math.dist(a, b) <= EPS: continue
            ls = LineString([a, b])
            raw.append(dict(a=a, b=b, w=w, line=ls, cu=ls.buffer(w / 2, 16),
                            ti=ti, meta=meta, sig=signature, protected=protected))
    if not raw: return [], {}, set()
    tree = STRtree([e['line'] for e in raw])
    cuts = [{e['a'], e['b']} for e in raw]
    terminals = _terminals(B, R, n, L)
    terminal_parts = list(getattr(terminals, 'geoms', [terminals]))
    for e, points in zip(raw, cuts):
        for g in terminal_parts:
            if not g.is_empty and e['cu'].intersects(g):


                projected=e['line'].interpolate(e['line'].project(g.centroid))
                anchor=projected if projected.distance(g)<=e['w']/2+EPS else nearest_points(e['line'],g)[0]
                points.add(tuple(anchor.coords[0]))
    for i, e in enumerate(raw):
        for j in tree.query(e['line']):
            if j <= i: continue
            inter = e['line'].intersection(raw[j]['line'])
            if inter.is_empty: continue
            if inter.geom_type == 'Point':
                p = tuple(inter.coords[0]); cuts[i].add(p); cuts[j].add(p)
            elif inter.geom_type == 'MultiPoint':
                for q in inter.geoms:
                    p = tuple(q.coords[0]); cuts[i].add(p); cuts[j].add(p)
            else:
                e['protected'] = raw[j]['protected'] = True
    edges = []
    for e, points in zip(raw, cuts):
        points = sorted(points, key=lambda p: e['line'].project(Point(p)))
        for a, b in zip(points, points[1:]):
            if math.dist(a, b) <= EPS: continue
            ls = LineString([a, b])
            edges.append(dict(e, a=a, b=b, line=ls, cu=ls.buffer(e['w'] / 2, 16)))
    adj = defaultdict(list)
    for i, e in enumerate(edges): adj[e['a']].append(i); adj[e['b']].append(i)
    anchors = set()
    ctree = STRtree([e['cu'] for e in edges])
    for i, e in enumerate(edges):


        for j in ctree.query(e['cu']):
            if j <= i or not e['cu'].intersects(edges[j]['cu']): continue
            q = edges[j]
            if not {e['a'], e['b']} & {q['a'], q['b']}:
                e['protected'] = q['protected'] = True
        if not terminals.is_empty:
            for p in (e['a'], e['b']):
                if Point(p).distance(terminals) <= e['w']/2 + EPS: anchors.add(p)
    for e in edges:
        if e['protected']: anchors.update((e['a'], e['b']))
    return edges, adj, anchors


def _chains(edges, adj, anchors, deleted):
    seen = set()
    def fixed(p):
        es = [i for i in adj[p] if i not in deleted]
        return p in anchors or len(es) != 2 or len({(edges[i]['w'], edges[i]['sig']) for i in es}) != 1
    out = []
    for start in sorted(adj):
        if not fixed(start): continue
        for i in adj[start]:
            if i in seen or i in deleted: continue
            pts, ids, p = [start], [], start
            while i not in seen:
                seen.add(i); ids.append(i); e = edges[i]
                p = e['b'] if p == e['a'] else e['a']; pts.append(p)
                if fixed(p): break
                i = next(k for k in adj[p] if k != i and k not in deleted)
            out.append((pts, ids))

    return out


class Corridor:
    def __init__(self, B, R, L):
        op = R.get('options') or {}
        self.clr = max(float(R.get('clr') or 0), *(float(op.get(k) or 0) for k in ('clr', 'drc', 'drcpad', 'drcvia'))) + .002
        self.edge = float(op.get('edge') or .2)
        self.inside = unary_union([Polygon(q).buffer(0) for q in B['outline']]).buffer(-self.edge)
        geo, nets = [], []
        def add(g, n): geo.append(g); nets.append(n)
        for t in R.get('tracks', []):
            if t['layer'] == L:
                for a, b, w in zip(t['pts'], t['pts'][1:], t['w']):
                    add((LineString([a, b]) if a != b else Point(a)).buffer(w/2, 16), t.get('net'))
        vd = float(str(op.get('via', '0.45/0.20')).split('/')[0])
        for v in R.get('vias', []):
            if L in (v.get('layers') or _layers(B)):
                add(Point(v['x'], v['y']).buffer(float(v.get('d', v.get('diameter', vd)))/2, 16), v.get('net'))
        for p in B.get('pads', []):
            for q in (p.get('polys') or {}).get(L, []): add(Polygon(q).buffer(0), p.get('net'))
            if p.get('drill'):

                add(Point(p['x'], p['y']).buffer(float(p['drill'])/2, 16), p.get('net') or '#hole_size')
        for z in _zones(R):
            if z.get('layer') == L:
                for g in _zone_polys(z): add(g, z.get('net'))
        for k in B.get('keepouts', []):
            if L in (k.get('layers') or k.get('cu_layers') or _layers(B)) and (k.get('prohibitions') or {}).get('track_list', True):
                for q in k.get('polys', []): add(Polygon(q).buffer(0), '#keepout')
        self.geo, self.nets = geo, nets
        self.tree = STRtree(geo) if geo else None

    def free(self, pts, w, n):
        cu = LineString(pts).buffer(w/2, 16)
        if not self.inside.covers(cu): return False
        if self.tree:
            for i in self.tree.query(cu.buffer(self.clr)):
                if self.nets[i] != n and cu.distance(self.geo[i]) < self.clr: return False
        return True


def _tighten(pts, w, n, corridor):
    lengths=[0.0]
    for a,b in zip(pts,pts[1:]): lengths.append(lengths[-1]+math.dist(a,b))
    out=[pts[0]];i=0;end=len(pts)-1
    while i < end:
        accepted=False


        for j in sorted(set(range(i+2,min(end,i+64)+1)) | {end},reverse=True):
            if j<=i+1: continue
            limit=lengths[j]-lengths[i]
            for q in _octi(pts[i],pts[j]):
                if LineString(q).length > limit+1e-9: continue
                if not corridor.free(q,w,n): continue
                out.extend(q[1:]);i=j;accepted=True;break
            if accepted:break
        if not accepted:out.append(pts[i+1]);i+=1
    return _merge(out)
def _analysis(B, R, K, *, reconstruct, shorten=True):
    new = copy.deepcopy(R) if reconstruct else None
    length_nets = _length_guard(K)
    protected_nets = set()
    groups = defaultdict(list)
    for i, t in enumerate(R.get('tracks', [])): groups[(t.get('net'), t['layer'])].append((i, t))
    replaced = {}; removed = set(); examples = []
    info = dict(version=VERSION, dead_pieces=0, dead_mm=0.0, shortenable_chains=0, free_mm=0.0,
                chain_list=0, protected_nets=0, arcs_guarded=0)
    corridors = {}
    for (n, L), tracks in sorted(groups.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
        if not n: continue


        if any(_protection(t, K, length_nets) for _, t in tracks) or n in R.get('area_nets', []) or any(
                z.get('net') == n and not _zone_polys(z) for z in _zones(R)):
            protected_nets.add(n)
            info['arcs_guarded'] += sum(any(x[0] == 'A' for x in t.get('items', [])) for _, t in tracks)
            continue
        edges, adj, anchors = _graph(B, R, K, n, L, tracks, length_nets)
        deleted = set()
        while True:
            dead = set()
            for p, ids in adj.items():
                live = [i for i in ids if i not in deleted]
                if p not in anchors and len(live) == 1 and not edges[live[0]]['protected']:
                    dead.add(live[0])
            if not dead: break
            deleted.update(dead)
        shortened = set(); additions = []
        for pts, ids in _chains(edges, adj, anchors, deleted):
            info['chain_list'] += 1
            if len(pts) < 3 or any(edges[i]['protected'] for i in ids): continue
            w = edges[ids[0]]['w']; old_len = LineString(pts).length
            if pts[0] == pts[-1]: continue
            if L not in corridors: corridors[L] = Corridor(B, R, L)


            choices = [q for q in _octi(pts[0], pts[-1]) if LineString(q).length < old_len - .005 and corridors[L].free(q, w, n)]
            if choices:
                q = min(choices, key=lambda q: (LineString(q).length, q))
            else:
                q = _tighten(pts,w,n,corridors[L])
                if LineString(q).length >= old_len-.005: continue
            info['shortenable_chains'] += 1; info['free_mm'] += old_len - LineString(q).length
            if len(examples) < 8: examples.append(dict(net=n, layer=L, from_=pts[0], post=pts[-1], gain_mm=old_len-LineString(q).length))
            if reconstruct and shorten:
                shortened.update(ids)
                additions.append(dict(edges[ids[0]]['meta'], pts=[list(p) for p in _merge(q)], w=[w]*(len(q)-1)))
        info['dead_pieces'] += len(deleted); info['dead_mm'] += sum(edges[i]['line'].length for i in deleted)
        if not reconstruct or (not deleted and not shortened): continue

        removed.update(i for i, _ in tracks)
        out = []
        for i, e in enumerate(edges):
            if i not in deleted and i not in shortened:
                out.append(dict(e['meta'], pts=[list(e['a']), list(e['b'])], w=[e['w']]))
        replaced[min(i for i, _ in tracks)] = out + additions
    if reconstruct:
        new['tracks'] = [t for i, t in enumerate(R.get('tracks', [])) if i not in removed]
        for i in sorted(replaced): new['tracks'].extend(replaced[i])
    info['protected_nets'] = len(protected_nets)
    info.update(dead_mm=round(info['dead_mm'], 6), free_mm=round(info['free_mm'], 6), sites=examples,
                hint='Free geometric proposal; native DRC/net function not yet checked. Zone shapes and arcs excluded.')
    info.update(_shape(R))
    return new, info

def _shape(R):
    from analysis.wobble import chain_list, nodes_bends, TOL, ANGLE
    turns, hooks, zigzag = 0, 0, 0
    chains = chain_list(R)
    turns += sum(nodes_bends(chains).values())
    for _, pts in chains:
        if len(pts) < 3: continue
        ps = list(LineString(pts).simplify(TOL, preserve_topology=False).coords)
        angles = []
        for a,b,c in zip(ps,ps[1:],ps[2:]):
            h1 = math.atan2(b[1]-a[1],b[0]-a[0]); h2 = math.atan2(c[1]-b[1],c[0]-b[0])
            d = (h2-h1+math.pi) % (2*math.pi)-math.pi
            if abs(math.degrees(d)) >= ANGLE: angles.append(d)
        turns += len(angles); hooks += sum(abs(x) >= math.radians(135) for x in angles)


        zigzag += sum(a*b < 0 and b*c < 0 for a,b,c in zip(angles,angles[1:],angles[2:]))
    return dict(visible_bends=turns, hairpins=hooks, zigzag=zigzag)


def measure(B, R, K, *, search=False):
    if search: return _analysis(B, R, K, reconstruct=False)[1]
    return dict(version='r24-shaping-v3', search=False, **_shape(R))


def suggestion(B, R, K, *, shorten=True):
    return _analysis(B, R, K, reconstruct=True, shorten=shorten)
