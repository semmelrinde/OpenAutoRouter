""
from collections import defaultdict
import copy
import json
import math

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, nearest_points
from shapely.strtree import STRtree

from nachbearbeitung.glaetten45 import _octi, _zusammenfassen

VERSION = 'r24-ketten-v5'
EPS = 1e-7
FLAGS = ('fix', 'kelvin', 'kette', 'strom_steg', 'strom_via', 'rueck_via')


def _meta(t):
    return {k: v for k, v in t.items() if k not in ('pts', 'w', 'items')}


def _laengenschutz(K):

    from analyse.laengenimpedanz import laengengruppen
    protected = {n for g in laengengruppen(K).values() for n in g['netze']}


    for n, meta in (K.get('netze') or {}).items():
        if meta.get('paar'): protected.update((n, meta['paar']))
    return protected


def _schutz(t, K, laengennetze=None):
    if laengennetze is None: laengennetze = _laengenschutz(K)
    n = (K.get('netze') or {}).get(t.get('net'), {})
    return bool(any(t.get(k) for k in FLAGS) or t.get('net') in laengennetze or n.get('paar') or n.get('laengengruppe') or
                n.get('toleranz_mm') is not None or
                any(i[0] != 'L' for i in t.get('items', [])))


def _layers(B):
    n = int(B.get('lagen', 2))
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def _zones(R):
    return list(R.get('zones') or []) + list(R.get('zonen') or [])


def _zone_polys(z):
    if z.get('polys'):
        return [Polygon(p['aussen'], p.get('loecher') or p.get('innen') or []).buffer(0) for p in z['polys']]
    if (z.get('filled') or z.get('modell')) and z.get('pts'):
        return [Polygon(z['pts'], z.get('holes') or []).buffer(0)]
    return []


def _terminals(B, R, n, L):
    out = []
    for p in B.get('pads', []):
        if p.get('net') == n:
            out.extend(Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3)
    vd = float(str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])
    for v in R.get('vias', []):
        if v.get('net') == n and L in (v.get('layers') or _layers(B)):
            out.append(Point(v['x'], v['y']).buffer(float(v.get('d', v.get('diameter', vd))) / 2, 16))

    for z in _zones(R):
        if z.get('net') == n and z.get('layer') == L:
            out.extend(_zone_polys(z))
    return unary_union(out)


def _graph(B, R, K, n, L, tracks, laengennetze):
    raw = []
    for ti, t in tracks:
        protected = _schutz(t, K, laengennetze)
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
        op = R.get('optionen') or {}
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

                add(Point(p['x'], p['y']).buffer(float(p['drill'])/2, 16), p.get('net') or '#loch')
        for z in _zones(R):
            if z.get('layer') == L:
                for g in _zone_polys(z): add(g, z.get('net'))
        for k in B.get('keepouts', []):
            if L in (k.get('layers') or k.get('lagen') or _layers(B)) and (k.get('verbote') or {}).get('bahnen', True):
                for q in k.get('polys', []): add(Polygon(q).buffer(0), '#sperre')
        self.geo, self.nets = geo, nets
        self.tree = STRtree(geo) if geo else None

    def free(self, pts, w, n):
        cu = LineString(pts).buffer(w/2, 16)
        if not self.inside.covers(cu): return False
        if self.tree:
            for i in self.tree.query(cu.buffer(self.clr)):
                if self.nets[i] != n and cu.distance(self.geo[i]) < self.clr: return False
        return True


def _straffen(pts, w, n, corridor):
    ""
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
    return _zusammenfassen(out)
def _analyse(B, R, K, *, rekonstruieren, kuerzen=True):
    ""
    new = copy.deepcopy(R) if rekonstruieren else None
    laengennetze = _laengenschutz(K)
    protected_nets = set()
    groups = defaultdict(list)
    for i, t in enumerate(R.get('tracks', [])): groups[(t.get('net'), t['layer'])].append((i, t))
    replaced = {}; removed = set(); examples = []
    info = dict(version=VERSION, tote_stuecke=0, tote_mm=0.0, abkuerzbare_ketten=0, gratis_mm=0.0,
                ketten=0, geschuetzte_netze=0, boegen_geschuetzt=0)
    corridors = {}
    for (n, L), tracks in sorted(groups.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
        if not n: continue


        if any(_schutz(t, K, laengennetze) for _, t in tracks) or n in R.get('flaechennetze', []) or any(
                z.get('net') == n and not _zone_polys(z) for z in _zones(R)):
            protected_nets.add(n)
            info['boegen_geschuetzt'] += sum(any(x[0] == 'A' for x in t.get('items', [])) for _, t in tracks)
            continue
        edges, adj, anchors = _graph(B, R, K, n, L, tracks, laengennetze)
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
            info['ketten'] += 1
            if len(pts) < 3 or any(edges[i]['protected'] for i in ids): continue
            w = edges[ids[0]]['w']; old_len = LineString(pts).length
            if pts[0] == pts[-1]: continue
            if L not in corridors: corridors[L] = Corridor(B, R, L)


            choices = [q for q in _octi(pts[0], pts[-1]) if LineString(q).length < old_len - .005 and corridors[L].free(q, w, n)]
            if choices:
                q = min(choices, key=lambda q: (LineString(q).length, q))
            else:
                q = _straffen(pts,w,n,corridors[L])
                if LineString(q).length >= old_len-.005: continue
            info['abkuerzbare_ketten'] += 1; info['gratis_mm'] += old_len - LineString(q).length
            if len(examples) < 8: examples.append(dict(net=n, layer=L, von=pts[0], nach=pts[-1], gewinn_mm=old_len-LineString(q).length))
            if rekonstruieren and kuerzen:
                shortened.update(ids)
                additions.append(dict(edges[ids[0]]['meta'], pts=[list(p) for p in _zusammenfassen(q)], w=[w]*(len(q)-1)))
        info['tote_stuecke'] += len(deleted); info['tote_mm'] += sum(edges[i]['line'].length for i in deleted)
        if not rekonstruieren or (not deleted and not shortened): continue

        removed.update(i for i, _ in tracks)
        out = []
        for i, e in enumerate(edges):
            if i not in deleted and i not in shortened:
                out.append(dict(e['meta'], pts=[list(e['a']), list(e['b'])], w=[e['w']]))
        replaced[min(i for i, _ in tracks)] = out + additions
    if rekonstruieren:
        new['tracks'] = [t for i, t in enumerate(R.get('tracks', [])) if i not in removed]
        for i in sorted(replaced): new['tracks'].extend(replaced[i])
    info['geschuetzte_netze'] = len(protected_nets)
    info.update(tote_mm=round(info['tote_mm'], 6), gratis_mm=round(info['gratis_mm'], 6), stellen=examples,
                hinweis='Freier geometrischer Vorschlag; native DRC/Netzfunktion noch ungeprueft. Zonenformen und Boegen ausgenommen.')
    info.update(_form(R))
    return new, info

def _form(R):
    ""
    from analyse.wackel import ketten, knoten_knicke, TOL, WINKEL
    turns, hooks, zigzag = 0, 0, 0
    chains = ketten(R)
    turns += sum(knoten_knicke(chains).values())
    for _, pts in chains:
        if len(pts) < 3: continue
        ps = list(LineString(pts).simplify(TOL, preserve_topology=False).coords)
        angles = []
        for a,b,c in zip(ps,ps[1:],ps[2:]):
            h1 = math.atan2(b[1]-a[1],b[0]-a[0]); h2 = math.atan2(c[1]-b[1],c[0]-b[0])
            d = (h2-h1+math.pi) % (2*math.pi)-math.pi
            if abs(math.degrees(d)) >= WINKEL: angles.append(d)
        turns += len(angles); hooks += sum(abs(x) >= math.radians(135) for x in angles)


        zigzag += sum(a*b < 0 and b*c < 0 for a,b,c in zip(angles,angles[1:],angles[2:]))
    return dict(sichtbare_knicke=turns, haarnadeln=hooks, zickzack=zigzag)


def messen(B, R, K, *, suchen=False):
    ""
    if suchen: return _analyse(B, R, K, rekonstruieren=False)[1]
    return dict(version='r24-form-v3', suche=False, **_form(R))


def vorschlag(B, R, K, *, kuerzen=True):
    ""
    return _analyse(B, R, K, rekonstruieren=True, kuerzen=kuerzen)
