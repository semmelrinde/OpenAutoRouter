import hashlib, heapq, json, math, re
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

TOL = 1e-6


class NetCopper:

    def __init__(self, H):
        self.H = H
        self.cu_layers = sorted({t['layer'] for t in H.get('tracks', [])} | {p['layer'] for p in H.get('pads', [])})
        self.pads = defaultdict(list); self.vias = defaultdict(list); self.tracks = defaultdict(list)
        for p in H.get('pads', []):
            if p.get('net') and len(p.get('poly') or []) >= 3:
                g = Polygon(p['poly']).buffer(0); c = (p['x'], p['y']) if 'x' in p else g.centroid.coords[0]


                self.pads[p['net']].append(('P:%s.%s@%.3f,%.3f' % (p.get('ref', '?'), p.get('num', '?'), c[0], c[1]), p['layer'], g))
        for v in H.get('vias', []):
            if v.get('net'): self.vias[v['net']].append(v)
        for t in H.get('tracks', []):
            self.tracks[t['net']].append(t)

    def _items(self, net, tracks, areas=None):
        items = []
        for pid, L, g in self.pads.get(net, []): items.append((pid, {L}, g))
        for v in self.vias.get(net, []):
            items.append(('V:' + str(v.get('uuid') or '%.6f,%.6f' % (v['x'], v['y'])), set(v.get('layers') or self.cu_layers),
                          Point(v['x'], v['y']).buffer(v['d'] / 2, 32)))
        for t in tracks:
            a, b = tuple(t['a']), tuple(t['b'])
            g = LineString([a, b]) if a != b else Point(a)
            items.append(('T:' + str(t.get('uuid') or len(items)), {t['layer']}, g.buffer(t['w'] / 2, 16)))
        for L, g in areas or []:
            for part in (g.geoms if hasattr(g, 'geoms') else [g]):
                if not part.is_empty: items.append(('Z:%d' % len(items), {L}, part))
        return items

    def components(self, net, tracks=None, areas=None):
        tracks = self.tracks.get(net, []) if tracks is None else tracks
        items = [(None if i[0].startswith(('T:', 'Z:')) else i[0], i[1], i[2]) for i in self._items(net, tracks, areas)]
        par = list(range(len(items)))
        def f(i):
            while par[i] != i: par[i] = par[par[i]]; i = par[i]
            return i
        if items:
            tree_idx = STRtree([g for _, _, g in items])
            for i, (_, Ls, g) in enumerate(items):
                for j in tree_idx.query(g.buffer(TOL)):
                    j = int(j)
                    if j <= i or not (Ls & items[j][1]): continue
                    if g.distance(items[j][2]) <= TOL: par[f(i)] = f(j)
        first = {}
        for i, (pid, _, _) in enumerate(items):
            if not pid: continue
            if pid in first: par[f(i)] = f(first[pid])
            else: first[pid] = i
        return {pid: f(i) for pid, i in first.items()}

    def separates(self, net, prior, new, areas_prev=None, areas_new=None):
        k0, k1 = self.components(net, prior, areas_prev), self.components(net, new, areas_new)
        group_list = defaultdict(set)
        for pid, c in k0.items(): group_list[c].add(pid)
        bad = []
        for c, ms in sorted(group_list.items(), key=lambda x: sorted(x[1])):
            if len({k1.get(m) for m in ms}) > 1: bad.append(sorted(ms))
        return bad

    def fpath(self, net, from_, post, tracks=None, areas=None, status=False):
        tracks = self.tracks.get(net, []) if tracks is None else tracks
        items = self._items(net, tracks, areas)
        ti = [k for k, it in enumerate(items) if it[0].startswith('T:')]
        total_length = {i: math.dist(t['a'], t['b']) for i, t in zip(ti, tracks)}
        nb = defaultdict(list)
        if items:
            tree_idx = STRtree([g for _, _, g in items])
            for i, (_, Ls, g) in enumerate(items):
                for j in tree_idx.query(g.buffer(TOL)):
                    j = int(j)
                    if j != i and (Ls & items[j][1]) and g.distance(items[j][2]) <= TOL: nb[i].append(j)
        equal = defaultdict(list)
        for i, it in enumerate(items):
            if it[0].startswith('P:'): equal[it[0]].append(i)
        for ids in equal.values():
            for i in ids: nb[i] += [j for j in ids if j != i]
        start = [i for i, it in enumerate(items) if it[0] in from_]; goal = {i for i, it in enumerate(items) if it[0] in post}
        dist = {i: 0.0 for i in start}; before = {}; q = [(0.0, i) for i in start]; heapq.heapify(q); end = None
        while q:
            d, i = heapq.heappop(q)
            if d > dist.get(i, 1e18): continue
            if i in goal: end = i; break
            for j in nb[i]:
                nd = d + total_length.get(j, 0.0)
                if nd < dist.get(j, 1e18) - 1e-12: dist[j] = nd; before[j] = i; heapq.heappush(q, (nd, j))
        if end is None: return (False, []) if status else []
        out = []
        while True:
            if items[end][0].startswith('T:'): out.append(items[end][0][2:])
            if end not in before: break
            end = before[end]
        return (True, sorted(out)) if status else sorted(out)


def apply(tracks, remove, add):
    rm = set(remove)
    return [t for t in tracks if t.get('uuid') not in rm] + [dict(t) for t in add]


def geo_hash(H, nets):
    nets = set(nets)
    r = lambda v: round(float(v), 6)
    T = sorted((t['net'], t['layer'], tuple(sorted([(r(t['a'][0]), r(t['a'][1])), (r(t['b'][0]), r(t['b'][1]))])), r(t['w']))
               for t in H.get('tracks', []) if t['net'] in nets)
    V = sorted((v['net'], r(v['x']), r(v['y']), r(v['d']), r(v.get('drill', 0)), tuple(sorted(v.get('layers') or [])))
               for v in H.get('vias', []) if v['net'] in nets)
    return hashlib.sha256(json.dumps([T, V]).encode()).hexdigest()


class Obstacles:

    def __init__(self, H, drc=0.09, drcpad=0.10, drcbga=None, drcvia=None, hole_size=0.25, edge=0.2, net_spacing=None, reserve=0.002,
                 areas=None):
        self.net_spacing = net_spacing or {}; self.res = reserve; self.drc = drc
        drcbga = drcpad if drcbga is None else drcbga; drcvia = drc if drcvia is None else drcvia
        self.obj = defaultdict(list)
        cu_layers = {t['layer'] for t in H.get('tracks', [])} | {p['layer'] for p in H.get('pads', [])}
        for t in H.get('tracks', []):
            g = LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])
            self.obj[t['layer']].append((t['net'], g.buffer(t['w'] / 2, 16), drc, True))
        for p in H.get('pads', []):
            if len(p.get('poly') or []) < 3: continue
            cp = drcbga if any(k in (p.get('fp') or '').upper() for k in ('BGA', 'WLCSP')) else drcpad
            self.obj[p['layer']].append((p['net'], Polygon(p['poly']).buffer(0), cp, False))
        for v in H.get('vias', []):
            c = Point(v['x'], v['y']).buffer(v['d'] / 2, 32)
            for L in (v.get('layers') or cu_layers):
                self.obj[L].append((v['net'], c, drcvia, False))
        for h in H.get('holes', []):
            c = Point(h['x'], h['y']).buffer(h['d'] / 2, 32)
            for L in cu_layers: self.obj[L].append((h.get('net') or '#HOLE', c, hole_size, False))
        for k in H.get('keepouts', []):
            if k.get('tracks'):
                for L in k.get('layers', []): self.obj[L].append(('#KEEPOUT', Polygon(k['poly']).buffer(0), 0.0, False))
        for e in H.get('edges', []):
            g = LineString(list(e) + [e[0]]) if len(e) > 2 else LineString(e)
            for L in cu_layers: self.obj[L].append(('#EDGE', g, edge, False))
        for net, L, g, clr in areas or []:
            self.obj[L].append((net, g, max(0.0, clr - reserve - 1e-4), False))
        self.tree_idx = {L: STRtree([o[1] for o in os_]) for L, os_ in self.obj.items()}
        self.dmax = max([o[2] for os_ in self.obj.values() for o in os_] + list(self.net_spacing.values()) + [0.3])
        self.new = defaultdict(list)

    def extra(self, net, layer, a, b, w):
        g = LineString([a, b]) if tuple(a) != tuple(b) else Point(a)
        self.new[layer].append((net, g.buffer(w / 2, 16)))

    def vacant(self, a, b, w, layer, net):
        ax = LineString([a, b]) if tuple(a) != tuple(b) else Point(a)
        owned = self.net_spacing.get(net, 0.0)
        for i in self.tree_idx[layer].query(ax.buffer(w / 2 + self.dmax + self.res)) if layer in self.tree_idx else []:
            n, g, d, track = self.obj[layer][int(i)]
            if n == net: continue
            dd = max(d, owned if track else 0.0, self.net_spacing.get(n, 0.0) if track else 0.0)
            if ax.distance(g) < w / 2 + dd + self.res: return False
        for n, g in self.new.get(layer, []):
            if n == net: continue
            dd = max(self.drc, owned, self.net_spacing.get(n, 0.0))
            if ax.distance(g) < w / 2 + dd + self.res: return False
        return True


def zones_fill(text):
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


def zones_contours(text):
    out = []
    for m in re.finditer(r'\n\t\(zone\n', text):
        e = text.find('\n\t)', m.end()); blk = text[m.start():e]
        if '(keepout' in blk: continue
        n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', blk)
        if not n or not n.group(1): continue
        ls = re.search(r'\n\t\t\(layers? ([^)]*)\)', blk)
        cu_layers = [L.replace('.Cu', '') for L in re.findall(r'"([^"]+)"', ls.group(1))] if ls else []
        u = re.search(r'\n\t\t\(polygon\s+\(pts(.*?)\n\t\t\t\)', blk, re.S)
        if not u: continue
        pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-0-9.]+) ([-0-9.]+)\)', u.group(1))]
        if len(pts) < 3: continue
        g = Polygon(pts).buffer(0)
        c = re.search(r'\(connect_pads[^()]*\(clearance ([0-9.]+)\)', blk)
        for L in cu_layers: out.append(dict(net=n.group(1).replace('\\"', '"').replace('\\\\', '\\'), layer=L, poly=g, clr=float(c.group(1)) if c else 0.2))
    return out


class ZonesGuard:

    def __init__(self, H, fills, drc=0.09, loss_max=0.1):
        self.NET_COPPER = NetCopper(H); self.drc = drc; self.loss_max = loss_max; self.losses = {}
        self.z = [dict(f, cur=f['poly'], box=f['poly'].bounds) for f in fills if f['net'] and not f['poly'].is_empty]
        self.basis = {}
        self._kc = {}

    def _parts(self, g, t):
        ps = [p for p in (g.geoms if hasattr(g, 'geoms') else [g]) if not p.is_empty and p.area > 0]
        return [p for p in ps if p.area > 4 * t * t or not p.buffer(-t / 2 + 1e-6).is_empty]

    def _cut(self, net, add, cuts=None):
        new = {}
        r6 = lambda p: (round(p[0], 6), round(p[1], 6))
        key = lambda s: (s['layer'], tuple(sorted([r6(s['a']), r6(s['b'])])), round(s['w'], 6))
        prior = self.NET_COPPER.tracks.get(net, []); prev_k = {key(t) for t in prior}
        add = [s for s in add if key(s) not in prev_k]
        geo = lambda s: LineString([s['a'], s['b']]) if tuple(s['a']) != tuple(s['b']) else Point(s['a'])
        for k, z in enumerate(self.z):
            if z['net'] == net: continue
            segs = [s for s in add if s['layer'] == z['layer']]
            if not segs: continue
            c = max(z['clr'], self.drc)
            cut = unary_union([geo(s).buffer(s['w'] / 2 + c + z['t'], 8) for s in segs])
            b0 = cut.bounds
            already = [geo(t).buffer(t['w'] / 2 + c, 8) for t in prior if t['layer'] == z['layer'] and not (
                max(t['a'][0], t['b'][0]) < b0[0] - 1 or min(t['a'][0], t['b'][0]) > b0[2] + 1 or max(t['a'][1], t['b'][1]) < b0[1] - 1
                or min(t['a'][1], t['b'][1]) > b0[3] + 1)]
            if already: cut = cut.difference(unary_union(already))
            if cut.is_empty: continue
            b0, b1 = cut.bounds, z['box']
            if b0[2] < b1[0] or b0[0] > b1[2] or b0[3] < b1[1] or b0[1] > b1[3] or not cut.intersects(z['cur']): continue
            new[k] = unary_union(self._parts(z['cur'].difference(cut), z['t']))
            if cuts is not None: cuts[k] = cut
        return new

    def power_loss(self, k, cut_exact, border=2.0):
        from shapely.geometry import box as _box
        if cut_exact.is_empty: return 0.0
        z = self.z[k]; t = z['t']
        b = cut_exact.bounds; win = _box(b[0] - border, b[1] - border, b[2] + border, b[3] + border)
        frame = win.boundary.buffer(1e-3)
        anchor = [g for g, _ in self._contacts(z['net'], z['layer'])]
        def keep(g):
            g = g.buffer(-t / 2 + 1e-6, 8).buffer(t / 2 - 1e-6, 8) if t > 0 else g
            return sum(p.area for p in (g.geoms if hasattr(g, 'geoms') else [g])
                       if not p.is_empty and (p.intersects(frame) or any(p.distance(a) <= 1e-6 for a in anchor)))
        prior = z['cur'].intersection(win)
        new = prior.difference(cut_exact)
        v = (new.area - keep(new)) - (prior.area - keep(prior))
        return max(0.0, v)

    def verify(self, net, add):
        cuts = {}; new = self._cut(net, add, cuts); broken = set(); self.losses.pop(net, None)
        for k, g in sorted(new.items()):
            z = self.z[k]
            if self.loss_max is not None:
                v = self.power_loss(k, self._exact(net, add, k))
                if v > self.loss_max:
                    self.losses.setdefault(net, []).append(dict(zone=z['net'], layer_name=z['layer'], loss_mm2=round(v, 3)))
                    broken.add(z['net']); continue


            if len(self._parts(g, z['t'])) <= len(self._parts(z['cur'], z['t'])) and not self._contact_way(z, g, cuts[k]): continue
            zn = z['net']
            fl0 = [(y['layer'], y['poly']) for y in self.z if y['net'] == zn]
            fl1 = [(y['layer'], new.get(i, y['cur'])) for i, y in enumerate(self.z) if y['net'] == zn]
            if zn not in self.basis: self.basis[zn] = fl0
            if self.NET_COPPER.separates(zn, self.NET_COPPER.tracks.get(zn, []), self.NET_COPPER.tracks.get(zn, []), fl0, fl1): broken.add(zn)
        return sorted(broken)

    def _exact(self, net, add, k):
        z = self.z[k]; c = max(z['clr'], self.drc)
        r6 = lambda p: (round(p[0], 6), round(p[1], 6))
        key = lambda s: (s['layer'], tuple(sorted([r6(s['a']), r6(s['b'])])), round(s['w'], 6))
        prior = self.NET_COPPER.tracks.get(net, []); prev_k = {key(t) for t in prior}
        geo = lambda s: LineString([s['a'], s['b']]) if tuple(s['a']) != tuple(s['b']) else Point(s['a'])
        segs = [s for s in add if s['layer'] == z['layer'] and key(s) not in prev_k]
        cut = unary_union([geo(s).buffer(s['w'] / 2 + c, 8) for s in segs])
        b0 = cut.bounds
        already = [geo(t).buffer(t['w'] / 2 + c, 8) for t in prior if t['layer'] == z['layer'] and not (
            max(t['a'][0], t['b'][0]) < b0[0] - 1 or min(t['a'][0], t['b'][0]) > b0[2] + 1 or max(t['a'][1], t['b'][1]) < b0[1] - 1
            or min(t['a'][1], t['b'][1]) > b0[3] + 1)]
        return cut.difference(unary_union(already)) if already else cut

    def _contacts(self, net, L):
        k = (net, L)
        if k not in self._kc:
            out = [gp for _, L2, gp in self.NET_COPPER.pads.get(net, []) if L2 == L]
            out += [Point(v['x'], v['y']).buffer(v['d'] / 2, 16) for v in self.NET_COPPER.vias.get(net, []) if L in (v.get('layers') or [L])]
            out += [(LineString([t['a'], t['b']]) if tuple(t['a']) != tuple(t['b']) else Point(t['a'])).buffer(t['w'] / 2, 8)
                    for t in self.NET_COPPER.tracks.get(net, []) if t['layer'] == L]
            self._kc[k] = [(g, g.bounds) for g in out]
        return self._kc[k]

    def _contact_way(self, z, g, cut):
        b = cut.bounds
        for gp, c in self._contacts(z['net'], z['layer']):
            if c[2] < b[0] or c[0] > b[2] or c[3] < b[1] or c[1] > b[3]: continue
            if gp.distance(z['cur']) <= 1e-6 and gp.distance(g) > 1e-6: return True
        return False

    def commit(self, net, add, remove=()):
        for k, g in self._cut(net, add).items(): self.z[k]['cur'] = g
        if remove or add:
            self.NET_COPPER.tracks[net] = apply(self.NET_COPPER.tracks.get(net, []), remove, add)
            for k in [k for k in self._kc if k[0] == net]: del self._kc[k]

    def blocked(self, keepout_list):
        return [(z['net'], z['layer'], z['poly'], z['clr']) for z in self.z if z['net'] in keepout_list]
