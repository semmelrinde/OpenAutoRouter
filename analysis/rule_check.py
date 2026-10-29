import json, sys, math, os, collections, heapq, statistics
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
from common.flow import pad_geo, _seg_dist
from common.values import base_name

TOL = 0.03
NOMINAL = dict(spacing=dict(small=1.5, medium=1.5, large=10.0),
            stub=0.6,
            in_pad=1 / 3, in_pad_better=0.5,
            ground_pad_min=4, ground_pad_large=9, ground_pad_large_from=9.0,
            crystal=5.0,
            on_way=0.05)


class Copper:

    def __init__(self, tracks, vias, pads, via_r, surface=False):
        self.adj = collections.defaultdict(list); self.vias = vias; self.pads = pads; self._kan = {}; self.seg = []
        cu_layers = sorted({t['layer'] for t in tracks})
        pts = collections.defaultdict(list)
        for t in tracks:
            for q in t['pts']: pts[t['layer']].append((q[0], q[1]))
        for v in vias:
            for L in cu_layers: pts[L].append((v['x'], v['y']))


        geo = {idx: pad_geo(p) for idx, p in pads}; connection = collections.defaultdict(list)
        for idx, p in pads:
            G = geo[idx]; bx0, by0, bx1, by1 = p['bbox']
            for t in tracks:
                L = t['layer']
                if L not in G or L not in p['layers']: continue
                for p1, p2, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                    m = w / 2 + TOL
                    if max(p1[0], p2[0]) < bx0 - m or min(p1[0], p2[0]) > bx1 + m or max(p1[1], p2[1]) < by0 - m or min(p1[1], p2[1]) > by1 + m: continue
                    if math.dist(p1, p2) < 1e-9: continue
                    ls = LineString([p1, p2])
                    if G[L].distance(ls) > m: continue
                    inside = ls.intersection(G[L])
                    if inside.is_empty:
                        q = nearest_points(ls, G[L])[0]; connection[idx].append((L, (q.x, q.y), G[L].distance(q)))
                    else:
                        for part in getattr(inside, 'geoms', [inside]):
                            cs = list(part.coords)
                            for c in (cs[0], cs[-1]): connection[idx].append((L, (c[0], c[1]), 0.0))
            for L, q, _ in connection[idx]: pts[L].append(q)
        for t in tracks:
            L = t['layer']
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                a = tuple(a); b = tuple(b)
                if math.dist(a, b) < 1e-9: continue
                x0, x1 = min(a[0], b[0]) - TOL, max(a[0], b[0]) + TOL; y0, y1 = min(a[1], b[1]) - TOL, max(a[1], b[1]) + TOL
                part = {0.0: a, 1.0: b}
                for q in pts[L]:
                    if x0 <= q[0] <= x1 and y0 <= q[1] <= y1:
                        d, s = _seg_dist(q, a, b)
                        if d < TOL and 1e-6 < s < 1 - 1e-6: part[round(s, 6)] = q
                ks = sorted(part)
                for s1, s2 in zip(ks[:-1], ks[1:]):
                    p1, p2 = part[s1], part[s2]; u, v = self.nodes(L, *p1), self.nodes(L, *p2)
                    if u == v: continue
                    self.edge(u, v, math.dist(p1, p2), 0.0); self.seg.append((L, p1, p2, w, u, v))
        vl = collections.defaultdict(set)
        for i, v in enumerate(vias):
            for L in cu_layers: self.edge(('V', i, L), self.nodes(L, v['x'], v['y']), 0.0, 0); vl[i].add(L)
        for idx, p in pads:
            G = geo[idx]; pk = ('P', idx); self.adj[pk]
            bx0, by0, bx1, by1 = p['bbox']
            for L, q, gap in connection[idx]: self.edge(pk, self.nodes(L, *q), gap, 0)
            for i, v in enumerate(vias):
                if not (bx0 - via_r - TOL <= v['x'] <= bx1 + via_r + TOL and by0 - via_r - TOL <= v['y'] <= by1 + via_r + TOL): continue
                for L in sorted(G):
                    d = G[L].distance(Point(v['x'], v['y']))
                    if d <= via_r + TOL: self.edge(pk, ('V', i, L), d, 0); vl[i].add(L)
        for a, (i, p) in enumerate(pads):
            for j, q in pads[a + 1:]:
                if p['bbox'][0] > q['bbox'][2] or q['bbox'][0] > p['bbox'][2] or p['bbox'][1] > q['bbox'][3] or q['bbox'][1] > p['bbox'][3]: continue
                if any(L in geo[j] and geo[i][L].distance(geo[j][L]) <= 1e-6 for L in geo[i]): self.edge(('P', i), ('P', j), 0.0, 0)
        if surface:
            for i in range(len(vias)): vl[i].add('#')
            kn = [(('V', i, '#'), (v['x'], v['y'])) for i, v in enumerate(vias)] + [(('P', i), (p['x'], p['y'])) for i, p in pads if p.get('drill')]
            for a, (ka, pa) in enumerate(kn):
                for kb, pb in kn[a + 1:]: self.edge(ka, kb, math.dist(pa, pb), 0)
        for i, Ls in sorted(vl.items()):
            Ls = sorted(Ls)
            for a, La in enumerate(Ls):
                for Lb in Ls[a + 1:]: self.edge(('V', i, La), ('V', i, Lb), 0.0, 1)

    def nodes(self, L, x, y):
        cx, cy = int(math.floor(x / TOL)), int(math.floor(y / TOL))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for px, py in self._kan.get((L, cx + dx, cy + dy), ()):
                    if math.hypot(px - x, py - y) <= TOL: return ('N', L, px, py)
        self._kan.setdefault((L, cx, cy), []).append((x, y)); return ('N', L, x, y)

    def edge(self, a, b, total_length, change):
        self.adj[a].append((b, total_length, change)); self.adj[b].append((a, total_length, change))

    def paths(self, start):
        EPS = 1e-6
        dist = {start: 0.0}; lw = {start: 0}; cost = {start: 0.0}; before = {}; h = [(0.0, 0, start)]; z = 0
        while h:
            k, _, n = heapq.heappop(h)
            if k > cost[n] + 1e-15: continue
            for m, l, x in self.adj[n]:
                net_copper = k + l + EPS * x
                if net_copper < cost.get(m, 1e18) - 1e-12:
                    cost[m] = net_copper; dist[m] = dist[n] + l; lw[m] = lw[n] + x; before[m] = n; z += 1; heapq.heappush(h, (net_copper, z, m))
        return dist, lw, before


class Checker:
    def __init__(self, B, K, R, p3_measure=None):
        self.B, self.K, self.R = B, K, R
        flag = (R.get('options') or {}).get('p3_measure', (K.get('specs') or {}).get('p3_measure', bool(K.get('p3_pin_groups'))))
        self.p3_measure = str(flag).lower() in ('1', 'true') if p3_measure is None else p3_measure
        self._paths = {}
        self.N = K['net_list']; self.GND = set(K.get('gnd') or [])
        self.fl = set(R.get('area_nets') or [])
        try: self.via_d = float(str((R.get('options') or {}).get('via') or (K.get('design_rules') or {}).get('via') or '0.45').split('/')[0])
        except ValueError: self.via_d = 0.45
        self.via_r = self.via_d / 2
        self.pads_net = collections.defaultdict(list); self.pads_ref = collections.defaultdict(list)
        for i, p in enumerate(B['pads']):
            self.pads_ref[p['ref']].append((i, p))
            if p['net']: self.pads_net[p['net']].append((i, p))
        self.tr = collections.defaultdict(list); self.vi = collections.defaultdict(list)
        for t in R.get('tracks', []): self.tr[t['net']].append(t)
        for v in R.get('vias', []): self.vi[v['net']].append(v)
        self.fps = {f['ref']: f for f in B.get('fps', [])}
        self._g = {}; self._geo = {}
        self.prior = not any('function' in t for t in K['part_list'].values())
        L = int(K.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or 2)
        self.stack = ['F'] + ['In%d' % i for i in range(1, L - 1)] + ['B']
        from analysis.areas import active, metric
        self.area_layers = (set(metric(B, R, K)['gnd_layers']) if active(R, K) else
                              {str(x).split('=')[0] for x in (R.get('gnd_layers') or [])})

    def graph(self, n):
        if n not in self._g: self._g[n] = Copper(self.tr[n], self.vi[n], self.pads_net[n], self.via_r, surface=n in self.fl)
        return self._g[n]

    def paths(self, n, start):
        key = (n, start)
        if key not in self._paths: self._paths[key] = self.graph(n).paths(start)
        return self._paths[key]

    def area_model(self, n):
        return n in self.fl or any(v.get('net') == n for v in
            list(self.R.get('zone_list') or []) + list(self.R.get('zones') or []))

    def geo(self, i):
        if i not in self._geo: self._geo[i] = pad_geo(self.B['pads'][i])
        return self._geo[i]

    def spacing(self, i, j):
        return min(a.distance(b) for a in self.geo(i).values() for b in self.geo(j).values())

    def function(self, r):
        t = self.K['part_list'][r]
        if 'function' in t: return t['function']
        if t.get('decoupling') and t.get('C'): return 'decoupling_' + ('small' if t['C'] <= 220e-9 * 1.001 else 'medium' if t['C'] <= 4.7e-6 * 1.001 else 'large')
        return t.get('role') if t.get('role') != 'small' else 'misc'

    def supports(self, r):
        t = self.K['part_list'][r]
        if t.get('supports') or not self.prior: return t.get('supports')
        for a, I in self.K.get('islands', {}).items():
            for e in I.get('decoupling', []):
                if e['ref'] == r and e.get('pins'): return dict(ref=a, pin=e['pins'][0], pins=e['pins'], net_name=e['net_name'])
        return None


    def decoupling(self):
        out = []
        for r in sorted(self.K['part_list']):
            f = self.function(r)
            if not f.startswith('decoupling_'): continue
            st = self.supports(r) or {}; level = f.split('_', 1)[1]
            cp = [(i, p) for i, p in self.pads_ref[r] if p['net'] and p['net'] not in self.GND]
            e = dict(part=r, level=level, C=self.K['part_list'][r].get('C'), out_port=bool(st.get('over_coil')), pin=None, spacing=None, track=None,
                     layer_change=None, net_name=cp[0][1]['net'] if cp else None, x=cp[0][1]['x'] if cp else None, y=cp[0][1]['y'] if cp else None)
            nums = set(st.get('pins') or ([st['pin']] if st.get('pin') else []))
            goal = [(i, p) for i, p in self.pads_ref.get(st.get('ref'), []) if p['net'] == st.get('net_name') and p['num'] in nums]
            if cp and goal:
                ci = next((i for i, p in cp if p['net'] == st['net_name']), cp[0][0])
                zi, zp = min(goal, key=lambda ip: (self.spacing(ci, ip[0]), ip[0]))
                d, lw, _ = self.paths(st['net_name'], ('P', ci))
                e.update(pin='%s:%s' % (zp['ref'], zp['num']), spacing=round(self.spacing(ci, zi), 3), net_name=st['net_name'],
                         track=round(d[('P', zi)], 3) if ('P', zi) in d else None, layer_change=int(round(lw[('P', zi)])) if ('P', zi) in d else None,
                         board_side='equal' if set(self.B['pads'][ci]['layers']) & set(zp['layers']) else 'opposite')
            out.append(e)
        group_list = {}
        for g in ('small', 'medium', 'large', 'out_port'):
            es = [e for e in out if (e['level'] == g and not e['out_port']) or (g == 'out_port' and e['out_port'])]
            with_ = [e for e in es if e['spacing'] is not None]
            if not es: continue
            nominal = NOMINAL['spacing'].get(g, NOMINAL['spacing']['large'])
            group_list[g] = dict(count=len(es), without_pin=len(es) - len(with_), nominal_spacing=nominal,
                              spacing_median=_med([e['spacing'] for e in with_]), spacing_max=max([e['spacing'] for e in with_], default=None),
                              track_median=_med([e['track'] for e in with_ if e['track'] is not None]),
                              track_max=max([e['track'] for e in with_ if e['track'] is not None], default=None),
                              over_nominal=sum(1 for e in with_ if e['spacing'] > nominal), with_layer_change=sum(1 for e in with_ if e['layer_change']),
                              not_joined=sum(1 for e in with_ if e['track'] is None))
        result = dict(group_list=group_list, part_list=out)
        if self.p3_measure:
            try: result['loops'] = self.decoupling_loops(out)
            except Exception as exc: result['loops_error'] = repr(exc)[:200]
        return result

    def decoupling_loops(self, part_list):
        from placement.island_physics import hull
        out = []
        for e in part_list:
            if e['out_port']: continue
            r = e['part']; st = self.supports(r) or {}
            z = dict(part=r, level=e['level'], hull_mm2=None, clearance_mm=None,
                     track_mm=None, ground_track_mm=None, layer_change=None,
                     model='Pad centers; copper graph; no nH')
            cp = [(i, p) for i, p in self.pads_ref[r] if p['net'] == e['net_name']]
            zp = [(i, p) for i, p in self.pads_ref.get(st.get('ref'), [])
                  if '%s:%s' % (p['ref'], p['num']) == e['pin']]
            pairs = [(ci, c, gi, g) for ci, c in self.pads_ref[r] if c['net'] in self.GND
                     for gi, g in self.pads_ref.get(st.get('ref'), []) if g['net'] == c['net']]
            if cp and zp and pairs:
                _, c = cp[0]; _, p = zp[0]
                ci, cm, gi, pg = min(pairs, key=lambda q: (math.dist((q[3]['x'], q[3]['y']), (p['x'], p['y'])), q[0], q[2]))
                pts = [(v['x'], v['y']) for v in (p, c, cm, pg)]
                z.update(hull_mm2=round(hull(pts), 4),
                         clearance_mm=round(math.dist(pts[0], pts[1]) + math.dist(pts[2], pts[3]), 4),
                         ground_pin='%s:%s' % (pg['ref'], pg['num']))


                supply_area = self.area_model(e['net_name'])
                if supply_area:
                    z['model'] += '; supply area: copper path unknown'
                if self.area_model(cm['net']):
                    z['model'] += '; ground area: copper path unknown'
                else:
                    d, lw, _ = self.paths(cm['net'], ('P', ci))
                    if ('P', gi) in d:
                        z['ground_track_mm'] = round(d['P', gi], 4)
                        if e['track'] is not None and not supply_area:
                            z['track_mm'] = round(e['track'] + d['P', gi], 4)
                            z['layer_change'] = e['layer_change'] + int(round(lw['P', gi]))
            out.append(z)
        def numbers(es):
            geo_missing = sum(v['clearance_mm'] is None for v in es)
            copper_unknown = sum(v['track_mm'] is None for v in es)
            return dict(count=len(es), geometry_missing=geo_missing,
                        copper_path_unknown=copper_unknown,
                        clearance_total_sum_mm=None if geo_missing else round(sum(v['clearance_mm'] for v in es), 4),
                        clearance_max_mm=None if geo_missing else max((v['clearance_mm'] for v in es), default=None),
                        hull_total_sum_mm2=None if geo_missing else round(sum(v['hull_mm2'] for v in es), 4),
                        track_max_mm=None if copper_unknown else max((v['track_mm'] for v in es), default=None))
        return dict(version='p3-loop-v1', model='Geometry/track model, no hardware measurement',
                    part_list=out, **numbers(out),
                    group_list={g:numbers([v for v in out if v['level']==g]) for g in ('small', 'medium', 'large')
                             if any(v['level']==g for v in out)})


    def link_chain(self):
        out = []
        for a, I in sorted(self.K.get('islands', {}).items()):
            per_net = collections.defaultdict(list)
            for e in I.get('decoupling', []):
                if not e.get('over_coil') and e.get('pins'):
                    group = tuple(sorted(e['pins'])) if self.K.get('p3_pin_groups') else ()
                    per_net[e['net_name'], group].append(e)
            for (n, _group), es in sorted(per_net.items()):
                es = sorted(es, key=lambda e: (e['C'], e['ref']))
                pins = [(i, p) for i, p in self.pads_ref[a] if p['net'] == n and p['num'] in set(es[0]['pins'])]
                cpad = {e['ref']: next((i for i, p in self.pads_ref[e['ref']] if p['net'] == n), None) for e in es}
                if not pins or cpad[es[0]['ref']] is None: continue
                wc = {r: self.paths(n, ('P', i)) for r, i in cpad.items() if i is not None}
                d0 = wc[es[0]['ref']][0]
                pi, pp = min(pins, key=lambda ip: (d0.get(('P', ip[0]), 1e9), self.spacing(cpad[es[0]['ref']], ip[0]), ip[0]))
                dp, lwp, before = self.paths(n, ('P', pi))
                source = set((self.N.get(n, {}).get('flow') or {}).get('source') or [])
                q_pads = [i for i, p in self.pads_net[n] if '%s:%s' % (p['ref'], p['num']) in source]
                out_port = any(self.B['pads'][i]['ref'] == a for i in q_pads)
                caps_refs = {e['ref'] for e in es}
                if out_port: targets = [i for i, p in self.pads_net[n] if p['ref'] != a and p['ref'] not in caps_refs and not self.function(p['ref']).startswith('decoupling_')]
                else: targets = sorted([i for i in q_pads if ('P', i) in dp], key=lambda i: dp[('P', i)])[:1]
                k = dict(chip=a, pin='%s:%s' % (a, pp['num']), net_name=n, mode='out_port' if out_port else 'input_', x=pp['x'], y=pp['y'], capacitors=[], verdict=None)
                for e in es:
                    r = e['ref']; ci = cpad[r]
                    if ci is None or ('P', ci) not in dp:
                        k['capacitors'].append(dict(part=r, C=e['C'], track=None)); continue
                    dc = wc[r][0]; st = [(dp[('P', ci)] + dc[('P', z)] - dp[('P', z)]) / 2 for z in targets if ('P', z) in dp and ('P', z) in dc]
                    k['capacitors'].append(dict(part=r, C=e['C'], track=round(dp[('P', ci)], 3), layer_change=int(round(lwp[('P', ci)])),
                                                   stub=round(max(0.0, min(st)), 3) if st and not out_port else None,
                                                   share_behind=round(sum(1 for s in st if s <= NOMINAL['on_way']) / len(st), 2) if st and out_port else None,
                                                   layer_name=round(dp[('P', ci)] - max(0.0, min(st)), 3) if st and not out_port else None))
                first = k['capacitors'][0]
                via_layer = None
                if not out_port and targets and ('P', targets[0]) in dp:
                    kn = ('P', targets[0]); k['way_to_source'] = round(dp[kn], 3)
                    while kn in before:
                        if kn[0] == 'V' and before[kn][0] == 'V': via_layer = dp[kn]
                        kn = before[kn]
                    k['first_via'] = round(via_layer, 3) if via_layer is not None else None
                if first.get('track') is None: k['verdict'] = 'not_joined'
                elif out_port:
                    k['verdict'] = 'link_chain' if first['share_behind'] == 1.0 and not first['layer_change'] else ('via_before_c' if first['layer_change'] else 'branch')
                elif not targets: k['verdict'] = 'source_unknown' if not first['layer_change'] else 'via_before_c'
                elif first['layer_change'] or (via_layer is not None and via_layer < first['layer_name'] - NOMINAL['on_way']): k['verdict'] = 'via_before_c'
                elif first['stub'] > NOMINAL['on_way']: k['verdict'] = 'branch'
                else: k['verdict'] = 'link_chain'
                lg = [c['layer_name'] for c in k['capacitors'] if c.get('layer_name') is not None]
                k['order'] = None if out_port or len(lg) < 2 else all(x <= y + NOMINAL['on_way'] for x, y in zip(lg[:-1], lg[1:]))
                out.append(k)
        return dict(count=len(out), verdicts=dict(collections.Counter(k['verdict'] for k in out)), chain_list=out)


    def ground_pads(self):
        from shapely.geometry import Polygon
        ep = {}
        for c, t in sorted(self.K['part_list'].items()):
            if t.get('role') != 'chip' or len(self.pads_ref[c]) < 5: continue
            fl = {i: sum(g.area for g in self.geo(i).values()) / max(1, len(self.geo(i))) for i, p in self.pads_ref[c]}
            med = statistics.median(fl.values())
            cand = [(i, p) for i, p in self.pads_ref[c] if p['net'] in self.GND and fl[i] > 2.5 * med and fl[i] >= 0.5]
            rest = [p for i, p in self.pads_ref[c] if (i, p) not in cand]
            if not cand or len(rest) < 3: continue
            x0, x1 = min(p['x'] for p in rest), max(p['x'] for p in rest); y0, y1 = min(p['y'] for p in rest), max(p['y'] for p in rest)
            mx, my = 0.15 * (x1 - x0), 0.15 * (y1 - y0)
            for i, p in cand:
                inner_x = x1 - x0 < 1e-6 or x0 + mx <= p['x'] <= x1 - mx; inner_y = y1 - y0 < 1e-6 or y0 + my <= p['y'] <= y1 - my
                if inner_x and inner_y: ep.setdefault((c, p['num']), []).append(i)
        return ep

    def ground_vias(self):
        ep = {i for v in self.ground_pads().values() for i in v}
        pads = []; circle = Point(0, 0).buffer(self.via_r).area
        for g in sorted(self.GND):
            G = self.graph_without_area(g); vias = self.vi[g]
            for i, p in self.pads_net[g]:
                r = p['ref']; t = self.K['part_list'].get(r, {}); f = self.function(r) if r in self.K['part_list'] else 'misc'
                group = 'decoupling' if f.startswith('decoupling_') else ('chip' if t.get('role') == 'chip' else 'rest')
                e = dict(pad='%s:%s' % (r, p['num']), part=r, group=group, level=f.split('_', 1)[1] if group == 'decoupling' else None, x=p['x'], y=p['y'],
                         ground_pad=i in ep)
                if p.get('drill'): e.update(category='plated_through', stub=0.0, share=None); pads.append(e); continue
                d, _, _ = G.paths(('P', i))
                per = {}
                for kn, dv in d.items():
                    if kn[0] == 'V': per[kn[1]] = min(per.get(kn[1], 1e9), dv)
                vd = sorted((dv, j) for j, dv in per.items())
                if not vd: e.update(category='without_via', stub=None, share=0.0, via=None); pads.append(e); continue
                def share(j): return max(Point(vias[j]['x'], vias[j]['y']).buffer(self.via_r).intersection(gg).area for gg in self.geo(i).values()) / circle

                close = [(share(j), -dj, -j) for dj, j in vd if dj <= self.via_r + TOL + 1e-9]
                if close and max(close)[0] > 1e-6: ant, st, vi_ = max(close); st, vi_ = -st, -vi_
                else: (st, vi_), ant = vd[0], 0.0
                v = vias[vi_]
                e.update(stub=round(st, 3), share=round(ant, 3), via=vi_, via_x=v['x'], via_y=v['y'],
                         category='in_pad' if ant >= NOMINAL['in_pad'] - 1e-9 else ('at_pad' if ant > 1e-6 else 'stub'))
                pads.append(e)
        per_via = collections.Counter(e['via'] for e in pads if e.get('via') is not None)
        for e in pads: e['shared'] = per_via.get(e.get('via'), 0) if e.get('via') is not None else 0
        group_list = {}
        for g in ('decoupling', 'chip', 'rest'):
            es = [e for e in pads if e['group'] == g and e['category'] != 'plated_through']
            if not es: continue
            st = [e['stub'] for e in es if e['category'] == 'stub']
            group_list[g] = dict(pads=len(es), in_pad=sum(1 for e in es if e['category'] == 'in_pad'),
                              in_pad_half=sum(1 for e in es if e['category'] == 'in_pad' and e['share'] >= NOMINAL['in_pad_better'] - 1e-9),
                              at_pad=sum(1 for e in es if e['category'] == 'at_pad'), stub=len(st), stub_median=_med(st), stub_max=max(st, default=None),
                              stub_over_nominal=sum(1 for s in st if s > NOMINAL['stub']), without_via=sum(1 for e in es if e['category'] == 'without_via'),
                              shared_via=sum(1 for e in es if e['shared'] > 1))
        return dict(group_list=group_list, nominal_stub=NOMINAL['stub'], pads=pads)

    def graph_without_area(self, n):
        k = (n, 'without')
        if k not in self._g: self._g[k] = Copper(self.tr[n], self.vi[n], self.pads_net[n], self.via_r, surface=False)
        return self._g[k]

    def ground_pad(self):
        out = []
        for (c, num), idx in sorted(self.ground_pads().items()):
            ps = [self.B['pads'][i] for i in idx]; n = ps[0]['net']
            fl = sum(max(g.area for g in self.geo(i).values()) for i in idx)
            geos = [gg for i in idx for gg in self.geo(i).values()]
            inside = [v for v in self.vi[n] if any(gg.contains(Point(v['x'], v['y'])) for gg in geos)]
            part = [v for v in self.vi[n] if v not in inside and any(Point(v['x'], v['y']).buffer(self.via_r).intersection(gg).area >= NOMINAL['in_pad'] * math.pi * self.via_r ** 2 for gg in geos)]
            p_ = self.via_d + 0.05; fits = 0
            for q in ps:
                bw, bh = q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]
                fits += (int((bw - self.via_d) / p_) + 1 if bw >= self.via_d else 0) * (int((bh - self.via_d) / p_) + 1 if bh >= self.via_d else 0)
            nominal = NOMINAL['ground_pad_large'] if fl >= NOMINAL['ground_pad_large_from'] else NOMINAL['ground_pad_min']
            out.append(dict(chip=c, pad=num, net_name=n, surface=round(fl, 2), partial_pads=len(idx), vias=len(inside), vias_at_border=len(part), nominal=nominal, fits=fits,
                            missing=max(0, nominal - len(inside) - len(part)), x=round(sum(q['x'] for q in ps) / len(ps), 3), y=round(sum(q['y'] for q in ps) / len(ps), 3)))
        return dict(count=len(out), under_nominal=sum(1 for e in out if e['missing']), pads=out)


    def crystal(self):
        out = []
        for r in sorted(self.K['part_list']):
            if self.function(r) != 'crystal': continue
            st = self.supports(r) or {}; q = dict(part=r, chip=st.get('ref'), net_list=[], longest=None)
            for n in st.get('net_list') or []:
                qi = next((i for i, p in self.pads_ref[r] if p['net'] == n), None)
                if qi is None: continue
                g = self.graph(n); d, lw, _ = g.paths(('P', qi))
                e = dict(net_name=n, x=self.B['pads'][qi]['x'], y=self.B['pads'][qi]['y'], chip_track=None, over_r=None, load=[])
                ci = next((i for i, p in self.pads_net[n] if p['ref'] == st.get('ref')), None)
                if ci is not None and ('P', ci) in d: e.update(chip_track=round(d[('P', ci)], 3), chip_layer_change=int(round(lw[('P', ci)])), chip_spacing=round(self.spacing(qi, ci), 3))
                elif ci is None:
                    for rr in sorted(self.K['part_list']):
                        s2 = self.K['part_list'][rr].get('supports') or {}
                        if self.function(rr) == 'series_r' and s2.get('crystal') == r and any(p['net'] == n for _, p in self.pads_ref[rr]):
                            ri = next(i for i, p in self.pads_ref[rr] if p['net'] == n); ro = next((i for i, p in self.pads_ref[rr] if p['net'] != n), None)
                            cj = next((i for i, p in self.pads_ref[st['ref']] if ro is not None and p['net'] == self.B['pads'][ro]['net']), None)
                            if cj is not None and ('P', ri) in d:
                                d2 = self.graph(self.B['pads'][ro]['net']).paths(('P', ro))[0]
                                if ('P', cj) in d2: e.update(chip_track=round(d[('P', ri)] + d2[('P', cj)], 3), over_r=rr)
                for c in sorted(self.K['part_list']):
                    s2 = self.K['part_list'][c].get('supports') or {}
                    if self.function(c) == 'crystal_load' and s2.get('crystal') == r:
                        li = next((i for i, p in self.pads_ref[c] if p['net'] == n), None)
                        if li is not None: e['load'].append(dict(part=c, track=round(d[('P', li)], 3) if ('P', li) in d else None,
                                                                 layer_change=int(round(lw[('P', li)])) if ('P', li) in d else None))
                q['net_list'].append(e)
            ls = [e['chip_track'] for e in q['net_list'] if e['chip_track'] is not None] + [l['track'] for e in q['net_list'] for l in e['load'] if l['track'] is not None]
            q['longest'] = max(ls, default=None); out.append(q)
        return dict(count=len(out), nominal=NOMINAL['crystal'], over_nominal=sum(1 for q in out if q['longest'] is None or q['longest'] > NOMINAL['crystal']), crystals=out)


    def beside_area(self, layer_name):
        if layer_name not in self.stack: return False
        k = self.stack.index(layer_name)
        return any(0 <= j < len(self.stack) and self.stack[j] in self.area_layers for j in (k - 1, k + 1))

    def fast(self):
        out = []
        for n, k in sorted(self.N.items()):
            if k.get('category') != 'fast': continue
            per = collections.defaultdict(float)
            for t in self.tr[n]:
                for a, b in zip(t['pts'][:-1], t['pts'][1:]): per[t['layer']] += math.dist(a, b)
            L = sum(per.values()); clearance = _mst([(p['x'], p['y']) for _, p in self.pads_net[n]])
            inner = sum(v for lg, v in per.items() if lg not in ('F', 'B') and self.beside_area(lg))
            beside = sum(v for lg, v in per.items() if self.beside_area(lg))
            p0 = self.pads_net[n][0][1] if self.pads_net[n] else {}
            out.append(dict(net_name=n, rank=k.get('rank'), reason=k.get('reason'), pads=len(self.pads_net[n]), vias=len(self.vi[n]), total_length=round(L, 2), air_line=round(clearance, 2),
                            ratio=round(L / clearance, 2) if clearance > 1e-6 and L else None, share_inner_beside_area=round(inner / L, 2) if L else None,
                            share_beside_area=round(beside / L, 2) if L else None, per_layer={lg: round(v, 2) for lg, v in sorted(per.items())},
                            pair=k.get('pair'), x=p0.get('x'), y=p0.get('y')))
        out.sort(key=lambda e: (-(e['rank'] or 0), -e['vias'], e['net_name']))
        Ls = sum(e['total_length'] for e in out)
        return dict(count=len(out), vias=sum(e['vias'] for e in out), nets_with_vias=sum(1 for e in out if e['vias']),
                    share_inner_beside_area=round(sum(e['total_length'] * (e['share_inner_beside_area'] or 0) for e in out) / Ls, 2) if Ls else None,
                    area_layers=sorted(self.area_layers), net_list=out)

    def vias_unnecessary(self):
        out = []; single_sided = 0
        for n, ps in sorted(self.pads_net.items()):
            if n in self.GND or n in self.fl or len(ps) < 2 or n not in self.N: continue
            if any(p.get('drill') for _, p in ps): continue
            sides = {tuple(p['layers']) for _, p in ps}
            if len(sides) != 1 or len(next(iter(sides))) != 1: continue
            single_sided += 1
            nv = [v for v in self.vi[n] if not v.get('current_via')]
            if not nv: continue
            L = sum(math.dist(a, b) for t in self.tr[n] for a, b in zip(t['pts'][:-1], t['pts'][1:]))
            clearance = _mst([(p['x'], p['y']) for _, p in ps])
            out.append(dict(net_name=n, category=self.N[n].get('category'), rank=self.N[n].get('rank'), board_side=next(iter(sides))[0], pads=len(ps), vias=len(nv),
                            total_length=round(L, 2), air_line=round(clearance, 2), x=nv[0]['x'], y=nv[0]['y']))
        out.sort(key=lambda e: (-e['vias'], e['net_name']))
        per_kind = collections.Counter()
        for e in out: per_kind[e['category']] += e['vias']
        return dict(nets_single_sided=single_sided, nets_with_vias=len(out), vias=sum(e['vias'] for e in out), per_kind=dict(sorted(per_kind.items())), net_list=out,
                    hint='Via candidates on one-sided nets; free alternative not checked.')


    def min_width(self):
        out = []
        for n, k in sorted(self.N.items()):
            if k.get('category') not in ('current_val', 'supply', 'sw') or 'wmin' not in k: continue
            wmin = k['wmin']; under = 0.0; ges = 0.0; bad = None; tap = 0.0
            for t in self.tr[n]:
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                    l = math.dist(a, b)
                    if t.get('sig'): tap += l; continue
                    ges += l
                    if w < wmin - 1e-6:
                        under += l
                        if bad is None or (w, -l) < (bad[0], -bad[1]): bad = (w, l, a[0], a[1], t['layer'])
            if under > 0:
                out.append(dict(net_name=n, category=k['category'], current_val=k.get('current_val'), wmin=wmin, w_nominal=k.get('w'), length_under=round(under, 2), total_length=round(ges, 2),
                                narrowest=bad[0], narrowest_length=round(bad[1], 2), x=bad[2], y=bad[3], layer_name=bad[4], tap=round(tap, 2)))
        out.sort(key=lambda e: (-(e['wmin'] - e['narrowest']) * e['length_under'], e['net_name']))
        return dict(net_list=len(out), length_under=round(sum(e['length_under'] for e in out), 2), sites=out)

    def everything(self):
        E = dict(version='rule_check-v1', nominal=NOMINAL, via_diameter=self.via_d, without_function=self.prior,
                 conflict_list=len(self.R.get('conflicts') or {}), open_items=len(self.R.get('failed') or []))
        E['decoupling'] = self.decoupling(); E['link_chain'] = self.link_chain(); E['ground_vias'] = self.ground_vias(); E['ground_pad'] = self.ground_pad()
        E['crystal'] = self.crystal(); E['fast'] = self.fast(); E['vias_unnecessary'] = self.vias_unnecessary(); E['min_width'] = self.min_width()
        return E

def _med(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 3) if xs else None


def _mst(pts):
    if len(pts) < 2: return 0.0
    inside = [pts[0]]; rest = list(pts[1:]); L = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inside), i) for i, r in enumerate(rest)); L += d; inside.append(rest.pop(i))
    return L


def check(B, K, R, p3_measure=None):
    return Checker(B, K, R, p3_measure=p3_measure).everything()


def _z(v, f='%.2f'): return '-' if v is None else f % v


def report(E, top=5):
    o = []; bn = lambda n: base_name(n) if n else '-'
    o.append('Rule check (%s): via %.2f mm, conflicts %d, open %d%s' % (E['version'], E['via_diameter'], E['conflict_list'], E['open_items'],
             ' | netclasses.json without component function: decoupling from the islands, crystal unchecked' if E['without_function'] else ''))
    e = E['decoupling']
    o.append('Decoupling (P3, P20): spacing pin-C edge to edge, track in copper, layer change')
    for g, s in e['group_list'].items():
        o.append('  %-8s %2d pieces | spacing median %s, max %s mm (target <= %.1f: %d above) | track median %s, max %s mm | with layer change %d | not connected %d%s' % (
            g, s['count'], _z(s['spacing_median']), _z(s['spacing_max']), s['nominal_spacing'], s['over_nominal'], _z(s['track_median']), _z(s['track_max']),
            s['with_layer_change'], s['not_joined'], ' | without pin %d' % s['without_pin'] if s['without_pin'] else ''))
    rank = {'small': 0, 'medium': 1, 'large': 2}
    bad = sorted([t for t in e['part_list'] if t['spacing'] is not None and not t['out_port']],
                  key=lambda t: (rank[t['level']], -(t['layer_change'] or 0), -(t['track'] if t['track'] is not None else 99), t['part']))
    for t in bad[:top + 3]:
        o.append('    %-9s %-6s -> %-10s spacing %s mm, track %s mm, layer change %s, side %s  (%.2f, %.2f)' % (
            t['part'], t['level'], t['pin'], _z(t['spacing']), _z(t['track']), _z(t['layer_change'], '%d'), t.get('board_side', '-'), t['x'], t['y']))
    k = E['link_chain']
    o.append('Supply in chain (R22): %d chains, verdicts %s' % (k['count'], k['verdicts']))
    for c in k['chain_list']:
        cs = ', '.join('%s %s (track %s, %s, LW %s)' % (x['part'], _si(x['C']), _z(x.get('track')),
                       ('Stub %s' % _z(x.get('stub'))) if c['mode'] == 'input_' else ('behind %s' % _z(x.get('share_behind'))), _z(x.get('layer_change'), '%d'))
                       for x in c['capacitors'])
        o.append('    %-10s %-8s %-7s %-16s first via at %s mm, order %s | %s  (%.2f, %.2f)' % (
            c['pin'], bn(c['net_name']), c['mode'], c['verdict'], _z(c.get('first_via')), {None: '-', True: 'ok', False: 'wrong'}[c['order']], cs, c['x'], c['y']))
    m = E['ground_vias']
    o.append('Ground vias (R8, R9, R23, R25): in pad from 1/3 of the via area, stub = track from pad to the nearest via (target <= %.1f mm)' % m['nominal_stub'])
    for g, s in m['group_list'].items():
        o.append('  %-11s %3d pads | in pad %d (of which >= 1/2: %d) | at pad %d | stub %d (median %s, max %s mm, above target %d) | without via %d | via shared with others %d' % (
            g, s['pads'], s['in_pad'], s['in_pad_half'], s['at_pad'], s['stub'], _z(s['stub_median']), _z(s['stub_max']), s['stub_over_nominal'], s['without_via'], s['shared_via']))
    bad = sorted([p for p in m['pads'] if p['category'] in ('stub', 'without_via')], key=lambda p: (p['category'] != 'without_via', -(p['stub'] or 0), p['pad']))
    for p in bad[:top + 3]:
        o.append('    %-12s %-11s %s%s  (%.2f, %.2f)' % (p['pad'], p['group'] + ('/' + p['level'] if p['level'] else ''),
                 'without via' if p['category'] == 'without_via' else 'Stub %.2f mm' % p['stub'], ', via shared with %d pads' % p['shared'] if p['shared'] > 1 else '', p['x'], p['y']))
    mp = E['ground_pad']
    o.append('Ground pad under chips (R16): %d pads, %d below target' % (mp['count'], mp['under_nominal']))
    for p in mp['pads']:
        o.append('    %-8s pad %-3s %5.2f mm2 (%d sub-pads): %d vias in pad, %d at edge; target %d, room for about %d; missing %d  (%.2f, %.2f)' % (
            p['chip'], p['pad'], p['surface'], p['partial_pads'], p['vias'], p['vias_at_border'], p['nominal'], p['fits'], p['missing'], p['x'], p['y']))
    q = E['crystal']
    o.append('Crystal (P20): %d crystals, %d above the guideline %.0f mm' % (q['count'], q['over_nominal'], q['nominal']))
    for x in q['crystals']:
        for n in x['net_list']:
            o.append('    %-6s %-10s -> %s track %s mm%s | load: %s  (%.2f, %.2f)' % (x['part'], bn(n['net_name']), x['chip'], _z(n['chip_track']), ' above ' + n['over_r'] if n['over_r'] else '',
                     ', '.join('%s %s mm' % (l['part'], _z(l['track'])) for l in n['load']) or '-', n['x'], n['y']))
    s = E['fast']
    o.append('Fast nets (R14, R19): %d nets, %d vias, %d nets with vias, share on inner layers next to an area %s (area layers %s)' % (
        s['count'], s['vias'], s['nets_with_vias'], _z(s['share_inner_beside_area']), ','.join(s['area_layers']) or '-'))
    for n in s['net_list'][:max(top, 8)]:
        o.append('    %-16s rank %s: %d vias, %s mm at straight-line %s (x %s), inner next to area %s, next to area total %s, per layer %s' % (
            bn(n['net_name']), n['rank'], n['vias'], _z(n['total_length']), _z(n['air_line']), _z(n['ratio']), _z(n['share_inner_beside_area']),
            _z(n['share_beside_area']), n['per_layer']))
    v = E['vias_unnecessary']
    o.append('Via candidates (R19, free alternative not checked): %d of %d one-sided nets have vias, %d in total (per kind %s)' % (v['nets_with_vias'], v['nets_single_sided'], v['vias'], v['per_kind']))
    for n in v['net_list'][:top + 3]:
        o.append('    %-20s %-10s side %s, %d pads: %d vias, %s mm at straight-line %s  (%.2f, %.2f)' % (bn(n['net_name']), n['category'], n['board_side'], n['pads'], n['vias'], _z(n['total_length']), _z(n['air_line']), n['x'], n['y']))
    b = E['min_width']
    o.append('Below the minimum width of the netclass (current nets): %d nets, %.1f mm' % (b['net_list'], b['length_under']))
    for n in b['sites'][:top + 3]:
        o.append('    %-14s %-10s wmin %.2f: %.1f of %.1f mm narrower, narrowest spot %.2f mm (%.2f mm long, layer %s)  (%.2f, %.2f)' % (
            bn(n['net_name']), n['category'], n['wmin'], n['length_under'], n['total_length'], n['narrowest'], n['narrowest_length'], n['layer_name'], n['x'], n['y']))
    return '\n'.join(o)


def _si(c):
    if not c: return '?'
    for f, u in ((1e-6, 'u'), (1e-9, 'n'), (1e-12, 'p')):
        if c >= f: return ('%g' % round(c / f, 2)) + u
    return '%g' % c


if __name__ == '__main__':
    if len(sys.argv) < 4: sys.exit(__doc__)
    B = json.load(open(sys.argv[1])); K = json.load(open(sys.argv[2])); R = json.load(open(sys.argv[3]))
    kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    E = check(B, K, R)
    print(report(E, top=int(kw.get('top', 5))))
    if 'json' in kw: json.dump(E, open(kw['json'], 'w'), indent=1)
