""
import hashlib, heapq, json, math, re
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

TOL = 1e-6


class Netzkupfer:
    ""

    def __init__(self, H):
        self.H = H
        self.lagen = sorted({t['layer'] for t in H.get('tracks', [])} | {p['layer'] for p in H.get('pads', [])})
        self.pads = defaultdict(list); self.vias = defaultdict(list); self.tracks = defaultdict(list)
        for p in H.get('pads', []):
            if p.get('net') and len(p.get('poly') or []) >= 3:
                g = Polygon(p['poly']).buffer(0); c = (p['x'], p['y']) if 'x' in p else g.centroid.coords[0]


                self.pads[p['net']].append(('P:%s.%s@%.3f,%.3f' % (p.get('ref', '?'), p.get('num', '?'), c[0], c[1]), p['layer'], g))
        for v in H.get('vias', []):
            if v.get('net'): self.vias[v['net']].append(v)
        for t in H.get('tracks', []):
            self.tracks[t['net']].append(t)

    def _items(self, net, tracks, flaechen=None):
        items = []
        for pid, L, g in self.pads.get(net, []): items.append((pid, {L}, g))
        for v in self.vias.get(net, []):
            items.append(('V:' + str(v.get('uuid') or '%.6f,%.6f' % (v['x'], v['y'])), set(v.get('layers') or self.lagen),
                          Point(v['x'], v['y']).buffer(v['d'] / 2, 32)))
        for t in tracks:
            a, b = tuple(t['a']), tuple(t['b'])
            g = LineString([a, b]) if a != b else Point(a)
            items.append(('T:' + str(t.get('uuid') or len(items)), {t['layer']}, g.buffer(t['w'] / 2, 16)))
        for L, g in flaechen or []:
            for teil in (g.geoms if hasattr(g, 'geoms') else [g]):
                if not teil.is_empty: items.append(('Z:%d' % len(items), {L}, teil))
        return items

    def komponenten(self, net, tracks=None, flaechen=None):
        ""
        tracks = self.tracks.get(net, []) if tracks is None else tracks
        items = [(None if i[0].startswith(('T:', 'Z:')) else i[0], i[1], i[2]) for i in self._items(net, tracks, flaechen)]
        par = list(range(len(items)))
        def f(i):
            while par[i] != i: par[i] = par[par[i]]; i = par[i]
            return i
        if items:
            baum = STRtree([g for _, _, g in items])
            for i, (_, Ls, g) in enumerate(items):
                for j in baum.query(g.buffer(TOL)):
                    j = int(j)
                    if j <= i or not (Ls & items[j][1]): continue
                    if g.distance(items[j][2]) <= TOL: par[f(i)] = f(j)
        erst = {}
        for i, (pid, _, _) in enumerate(items):
            if not pid: continue
            if pid in erst: par[f(i)] = f(erst[pid])
            else: erst[pid] = i
        return {pid: f(i) for pid, i in erst.items()}

    def trennt(self, net, alt, neu, flaechen_alt=None, flaechen_neu=None):
        ""
        k0, k1 = self.komponenten(net, alt, flaechen_alt), self.komponenten(net, neu, flaechen_neu)
        gruppen = defaultdict(set)
        for pid, c in k0.items(): gruppen[c].add(pid)
        schlecht = []
        for c, ms in sorted(gruppen.items(), key=lambda x: sorted(x[1])):
            if len({k1.get(m) for m in ms}) > 1: schlecht.append(sorted(ms))
        return schlecht

    def pfad(self, net, von, nach, tracks=None, flaechen=None, status=False):
        ""
        tracks = self.tracks.get(net, []) if tracks is None else tracks
        items = self._items(net, tracks, flaechen)
        ti = [k for k, it in enumerate(items) if it[0].startswith('T:')]
        laenge = {i: math.dist(t['a'], t['b']) for i, t in zip(ti, tracks)}
        nb = defaultdict(list)
        if items:
            baum = STRtree([g for _, _, g in items])
            for i, (_, Ls, g) in enumerate(items):
                for j in baum.query(g.buffer(TOL)):
                    j = int(j)
                    if j != i and (Ls & items[j][1]) and g.distance(items[j][2]) <= TOL: nb[i].append(j)
        gleich = defaultdict(list)
        for i, it in enumerate(items):
            if it[0].startswith('P:'): gleich[it[0]].append(i)
        for ids in gleich.values():
            for i in ids: nb[i] += [j for j in ids if j != i]
        start = [i for i, it in enumerate(items) if it[0] in von]; ziel = {i for i, it in enumerate(items) if it[0] in nach}
        dist = {i: 0.0 for i in start}; vor = {}; q = [(0.0, i) for i in start]; heapq.heapify(q); end = None
        while q:
            d, i = heapq.heappop(q)
            if d > dist.get(i, 1e18): continue
            if i in ziel: end = i; break
            for j in nb[i]:
                nd = d + laenge.get(j, 0.0)
                if nd < dist.get(j, 1e18) - 1e-12: dist[j] = nd; vor[j] = i; heapq.heappush(q, (nd, j))
        if end is None: return (False, []) if status else []
        out = []
        while True:
            if items[end][0].startswith('T:'): out.append(items[end][0][2:])
            if end not in vor: break
            end = vor[end]
        return (True, sorted(out)) if status else sorted(out)


def anwenden(tracks, remove, add):
    ""
    rm = set(remove)
    return [t for t in tracks if t.get('uuid') not in rm] + [dict(t) for t in add]


def geo_hash(H, nets):
    ""
    nets = set(nets)
    r = lambda v: round(float(v), 6)
    T = sorted((t['net'], t['layer'], tuple(sorted([(r(t['a'][0]), r(t['a'][1])), (r(t['b'][0]), r(t['b'][1]))])), r(t['w']))
               for t in H.get('tracks', []) if t['net'] in nets)
    V = sorted((v['net'], r(v['x']), r(v['y']), r(v['d']), r(v.get('drill', 0)), tuple(sorted(v.get('layers') or [])))
               for v in H.get('vias', []) if v['net'] in nets)
    return hashlib.sha256(json.dumps([T, V]).encode()).hexdigest()


class Hindernisse:
    ""

    def __init__(self, H, drc=0.09, drcpad=0.10, drcbga=None, drcvia=None, loch=0.25, kante=0.2, netzabstand=None, reserve=0.002,
                 flaechen=None):
        self.netzabstand = netzabstand or {}; self.res = reserve; self.drc = drc
        drcbga = drcpad if drcbga is None else drcbga; drcvia = drc if drcvia is None else drcvia
        self.obj = defaultdict(list)
        lagen = {t['layer'] for t in H.get('tracks', [])} | {p['layer'] for p in H.get('pads', [])}
        for t in H.get('tracks', []):
            g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
            self.obj[t['layer']].append((t['net'], g.buffer(t['w'] / 2, 16), drc, True))
        for p in H.get('pads', []):
            if len(p.get('poly') or []) < 3: continue
            cp = drcbga if any(k in (p.get('fp') or '').upper() for k in ('BGA', 'WLCSP')) else drcpad
            self.obj[p['layer']].append((p['net'], Polygon(p['poly']).buffer(0), cp, False))
        for v in H.get('vias', []):
            c = Point(v['x'], v['y']).buffer(v['d'] / 2, 32)
            for L in (v.get('layers') or lagen):
                self.obj[L].append((v['net'], c, drcvia, False))
        for h in H.get('holes', []):
            c = Point(h['x'], h['y']).buffer(h['d'] / 2, 32)
            for L in lagen: self.obj[L].append((h.get('net') or '#LOCH', c, loch, False))
        for k in H.get('keepouts', []):
            if k.get('tracks'):
                for L in k.get('layers', []): self.obj[L].append(('#SPERR', Polygon(k['poly']).buffer(0), 0.0, False))
        for e in H.get('edges', []):
            g = LineString(list(e) + [e[0]]) if len(e) > 2 else LineString(e)
            for L in lagen: self.obj[L].append(('#KANTE', g, kante, False))
        for net, L, g, clr in flaechen or []:
            self.obj[L].append((net, g, max(0.0, clr - reserve - 1e-4), False))
        self.baum = {L: STRtree([o[1] for o in os_]) for L, os_ in self.obj.items()}
        self.dmax = max([o[2] for os_ in self.obj.values() for o in os_] + list(self.netzabstand.values()) + [0.3])
        self.neu = defaultdict(list)

    def zusatz(self, net, layer, a, b, w):
        g = LineString([a, b]) if tuple(a) != tuple(b) else Point(a)
        self.neu[layer].append((net, g.buffer(w / 2, 16)))

    def frei(self, a, b, w, layer, net):
        ""
        ax = LineString([a, b]) if tuple(a) != tuple(b) else Point(a)
        eigen = self.netzabstand.get(net, 0.0)
        for i in self.baum[layer].query(ax.buffer(w / 2 + self.dmax + self.res)) if layer in self.baum else []:
            n, g, d, bahn = self.obj[layer][int(i)]
            if n == net: continue
            dd = max(d, eigen if bahn else 0.0, self.netzabstand.get(n, 0.0) if bahn else 0.0)
            if ax.distance(g) < w / 2 + dd + self.res: return False
        for n, g in self.neu.get(layer, []):
            if n == net: continue
            dd = max(self.drc, eigen, self.netzabstand.get(n, 0.0))
            if ax.distance(g) < w / 2 + dd + self.res: return False
        return True


def zonen_fuellung(text):
    ""
    out = []
    for m in re.finditer(r'\n\t\(zone\n', text):
        e = text.find('\n\t)', m.end()); blk = text[m.start():e]
        n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', blk)
        if not n: continue
        c = re.search(r'\(connect_pads[^()]*\(clearance ([0-9.]+)\)', blk); t = re.search(r'\(min_thickness ([0-9.]+)\)', blk)
        for f in re.finditer(r'\(filled_polygon\s+\(layer "([^"]+)"\)\s+\(pts(.*?)\n\t\t\t\)', blk, re.S):
            pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-0-9.]+) ([-0-9.]+)\)', f.group(2))]
            L = f.group(1).replace('.Cu', '')
            if len(pts) >= 3:
                out.append(dict(net=n.group(1).replace('\\"', '"').replace('\\\\', '\\'), layer=L, poly=Polygon(pts).buffer(0),
                                clr=float(c.group(1)) if c else 0.2, t=float(t.group(1)) if t else 0.25))
    return out


def zonen_umrisse(text):
    ""
    out = []
    for m in re.finditer(r'\n\t\(zone\n', text):
        e = text.find('\n\t)', m.end()); blk = text[m.start():e]
        if '(keepout' in blk: continue
        n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', blk)
        if not n or not n.group(1): continue
        ls = re.search(r'\n\t\t\(layers? ([^)]*)\)', blk)
        lagen = [L.replace('.Cu', '') for L in re.findall(r'"([^"]+)"', ls.group(1))] if ls else []
        u = re.search(r'\n\t\t\(polygon\s+\(pts(.*?)\n\t\t\t\)', blk, re.S)
        if not u: continue
        pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-0-9.]+) ([-0-9.]+)\)', u.group(1))]
        if len(pts) < 3: continue
        g = Polygon(pts).buffer(0)
        c = re.search(r'\(connect_pads[^()]*\(clearance ([0-9.]+)\)', blk)
        for L in lagen: out.append(dict(net=n.group(1).replace('\\"', '"').replace('\\\\', '\\'), layer=L, poly=g, clr=float(c.group(1)) if c else 0.2))
    return out


class ZonenWaechter:
    ""

    def __init__(self, H, fuellungen, drc=0.09, verlust_max=0.1):
        self.NK = Netzkupfer(H); self.drc = drc; self.verlust_max = verlust_max; self.verluste = {}
        self.z = [dict(f, cur=f['poly'], box=f['poly'].bounds) for f in fuellungen if f['net'] and not f['poly'].is_empty]
        self.basis = {}
        self._kc = {}

    def _teile(self, g, t):
        ps = [p for p in (g.geoms if hasattr(g, 'geoms') else [g]) if not p.is_empty and p.area > 0]
        return [p for p in ps if p.area > 4 * t * t or not p.buffer(-t / 2 + 1e-6).is_empty]

    def _schnitt(self, net, add, schnitte=None):
        ""
        neu = {}
        r6 = lambda p: (round(p[0], 6), round(p[1], 6))
        key = lambda s: (s['layer'], tuple(sorted([r6(s['a']), r6(s['b'])])), round(s['w'], 6))
        alt = self.NK.tracks.get(net, []); alt_k = {key(t) for t in alt}
        add = [s for s in add if key(s) not in alt_k]
        geo = lambda s: LineString([s['a'], s['b']]) if tuple(s['a']) != tuple(s['b']) else Point(s['a'])
        for k, z in enumerate(self.z):
            if z['net'] == net: continue
            segs = [s for s in add if s['layer'] == z['layer']]
            if not segs: continue
            c = max(z['clr'], self.drc)
            cut = unary_union([geo(s).buffer(s['w'] / 2 + c + z['t'], 8) for s in segs])
            b0 = cut.bounds
            schon = [geo(t).buffer(t['w'] / 2 + c, 8) for t in alt if t['layer'] == z['layer'] and not (
                max(t['a'][0], t['b'][0]) < b0[0] - 1 or min(t['a'][0], t['b'][0]) > b0[2] + 1 or max(t['a'][1], t['b'][1]) < b0[1] - 1
                or min(t['a'][1], t['b'][1]) > b0[3] + 1)]
            if schon: cut = cut.difference(unary_union(schon))
            if cut.is_empty: continue
            b0, b1 = cut.bounds, z['box']
            if b0[2] < b1[0] or b0[0] > b1[2] or b0[3] < b1[1] or b0[1] > b1[3] or not cut.intersects(z['cur']): continue
            neu[k] = unary_union(self._teile(z['cur'].difference(cut), z['t']))
            if schnitte is not None: schnitte[k] = cut
        return neu

    def verlust(self, k, cut_exakt, rand=2.0):
        ""
        from shapely.geometry import box as _box
        if cut_exakt.is_empty: return 0.0
        z = self.z[k]; t = z['t']
        b = cut_exakt.bounds; win = _box(b[0] - rand, b[1] - rand, b[2] + rand, b[3] + rand)
        rahmen = win.boundary.buffer(1e-3)
        anker = [g for g, _ in self._kontakte(z['net'], z['layer'])]
        def behalten(g):
            g = g.buffer(-t / 2 + 1e-6, 8).buffer(t / 2 - 1e-6, 8) if t > 0 else g
            return sum(p.area for p in (g.geoms if hasattr(g, 'geoms') else [g])
                       if not p.is_empty and (p.intersects(rahmen) or any(p.distance(a) <= 1e-6 for a in anker)))
        alt = z['cur'].intersection(win)
        neu = alt.difference(cut_exakt)
        v = (neu.area - behalten(neu)) - (alt.area - behalten(alt))
        return max(0.0, v)

    def pruefen(self, net, add):
        ""
        schnitte = {}; neu = self._schnitt(net, add, schnitte); kaputt = set(); self.verluste.pop(net, None)
        for k, g in sorted(neu.items()):
            z = self.z[k]
            if self.verlust_max is not None:
                v = self.verlust(k, self._exakt(net, add, k))
                if v > self.verlust_max:
                    self.verluste.setdefault(net, []).append(dict(zone=z['net'], lage=z['layer'], verlust_mm2=round(v, 3)))
                    kaputt.add(z['net']); continue


            if len(self._teile(g, z['t'])) <= len(self._teile(z['cur'], z['t'])) and not self._kontakt_weg(z, g, schnitte[k]): continue
            zn = z['net']
            fl0 = [(y['layer'], y['poly']) for y in self.z if y['net'] == zn]
            fl1 = [(y['layer'], neu.get(i, y['cur'])) for i, y in enumerate(self.z) if y['net'] == zn]
            if zn not in self.basis: self.basis[zn] = fl0
            if self.NK.trennt(zn, self.NK.tracks.get(zn, []), self.NK.tracks.get(zn, []), fl0, fl1): kaputt.add(zn)
        return sorted(kaputt)

    def _exakt(self, net, add, k):
        ""
        z = self.z[k]; c = max(z['clr'], self.drc)
        r6 = lambda p: (round(p[0], 6), round(p[1], 6))
        key = lambda s: (s['layer'], tuple(sorted([r6(s['a']), r6(s['b'])])), round(s['w'], 6))
        alt = self.NK.tracks.get(net, []); alt_k = {key(t) for t in alt}
        geo = lambda s: LineString([s['a'], s['b']]) if tuple(s['a']) != tuple(s['b']) else Point(s['a'])
        segs = [s for s in add if s['layer'] == z['layer'] and key(s) not in alt_k]
        cut = unary_union([geo(s).buffer(s['w'] / 2 + c, 8) for s in segs])
        b0 = cut.bounds
        schon = [geo(t).buffer(t['w'] / 2 + c, 8) for t in alt if t['layer'] == z['layer'] and not (
            max(t['a'][0], t['b'][0]) < b0[0] - 1 or min(t['a'][0], t['b'][0]) > b0[2] + 1 or max(t['a'][1], t['b'][1]) < b0[1] - 1
            or min(t['a'][1], t['b'][1]) > b0[3] + 1)]
        return cut.difference(unary_union(schon)) if schon else cut

    def _kontakte(self, net, L):
        ""
        k = (net, L)
        if k not in self._kc:
            out = [gp for _, L2, gp in self.NK.pads.get(net, []) if L2 == L]
            out += [Point(v['x'], v['y']).buffer(v['d'] / 2, 16) for v in self.NK.vias.get(net, []) if L in (v.get('layers') or [L])]
            out += [(LineString([t['a'], t['b']]) if tuple(t['a']) != tuple(t['b']) else Point(t['a'])).buffer(t['w'] / 2, 8)
                    for t in self.NK.tracks.get(net, []) if t['layer'] == L]
            self._kc[k] = [(g, g.bounds) for g in out]
        return self._kc[k]

    def _kontakt_weg(self, z, g, cut):
        ""
        b = cut.bounds
        for gp, c in self._kontakte(z['net'], z['layer']):
            if c[2] < b[0] or c[0] > b[2] or c[3] < b[1] or c[1] > b[3]: continue
            if gp.distance(z['cur']) <= 1e-6 and gp.distance(g) > 1e-6: return True
        return False

    def commit(self, net, add, remove=()):
        for k, g in self._schnitt(net, add).items(): self.z[k]['cur'] = g
        if remove or add:
            self.NK.tracks[net] = anwenden(self.NK.tracks.get(net, []), remove, add)
            for k in [k for k in self._kc if k[0] == net]: del self._kc[k]

    def gesperrt(self, sperren):
        ""
        return [(z['net'], z['layer'], z['poly'], z['clr']) for z in self.z if z['net'] in sperren]
