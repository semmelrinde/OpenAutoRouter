""
import json, sys, math, os, collections, heapq, statistics
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
from gemeinsam.fluss import padgeo, _seg_dist
from gemeinsam.werte import basisname

TOL = 0.03
SOLL = dict(abstand=dict(klein=1.5, mittel=1.5, gross=10.0),
            stich=0.6,
            im_pad=1 / 3, im_pad_besser=0.5,
            massepad_min=4, massepad_gross=9, massepad_gross_ab=9.0,
            quarz=5.0,
            auf_weg=0.05)


class Kupfer:
    ""

    def __init__(self, tracks, vias, pads, via_r, flaeche=False):
        self.adj = collections.defaultdict(list); self.vias = vias; self.pads = pads; self._kan = {}; self.seg = []
        lagen = sorted({t['layer'] for t in tracks})
        pts = collections.defaultdict(list)
        for t in tracks:
            for q in t['pts']: pts[t['layer']].append((q[0], q[1]))
        for v in vias:
            for L in lagen: pts[L].append((v['x'], v['y']))


        geo = {idx: padgeo(p) for idx, p in pads}; anschluss = collections.defaultdict(list)
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
                    drin = ls.intersection(G[L])
                    if drin.is_empty:
                        q = nearest_points(ls, G[L])[0]; anschluss[idx].append((L, (q.x, q.y), G[L].distance(q)))
                    else:
                        for teil in getattr(drin, 'geoms', [drin]):
                            cs = list(teil.coords)
                            for c in (cs[0], cs[-1]): anschluss[idx].append((L, (c[0], c[1]), 0.0))
            for L, q, _ in anschluss[idx]: pts[L].append(q)
        for t in tracks:
            L = t['layer']
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                a = tuple(a); b = tuple(b)
                if math.dist(a, b) < 1e-9: continue
                x0, x1 = min(a[0], b[0]) - TOL, max(a[0], b[0]) + TOL; y0, y1 = min(a[1], b[1]) - TOL, max(a[1], b[1]) + TOL
                teil = {0.0: a, 1.0: b}
                for q in pts[L]:
                    if x0 <= q[0] <= x1 and y0 <= q[1] <= y1:
                        d, s = _seg_dist(q, a, b)
                        if d < TOL and 1e-6 < s < 1 - 1e-6: teil[round(s, 6)] = q
                ks = sorted(teil)
                for s1, s2 in zip(ks[:-1], ks[1:]):
                    p1, p2 = teil[s1], teil[s2]; u, v = self.knoten(L, *p1), self.knoten(L, *p2)
                    if u == v: continue
                    self.kante(u, v, math.dist(p1, p2), 0.0); self.seg.append((L, p1, p2, w, u, v))
        vl = collections.defaultdict(set)
        for i, v in enumerate(vias):
            for L in lagen: self.kante(('V', i, L), self.knoten(L, v['x'], v['y']), 0.0, 0); vl[i].add(L)
        for idx, p in pads:
            G = geo[idx]; pk = ('P', idx); self.adj[pk]
            bx0, by0, bx1, by1 = p['bbox']
            for L, q, luecke in anschluss[idx]: self.kante(pk, self.knoten(L, *q), luecke, 0)
            for i, v in enumerate(vias):
                if not (bx0 - via_r - TOL <= v['x'] <= bx1 + via_r + TOL and by0 - via_r - TOL <= v['y'] <= by1 + via_r + TOL): continue
                for L in sorted(G):
                    d = G[L].distance(Point(v['x'], v['y']))
                    if d <= via_r + TOL: self.kante(pk, ('V', i, L), d, 0); vl[i].add(L)
        for a, (i, p) in enumerate(pads):
            for j, q in pads[a + 1:]:
                if p['bbox'][0] > q['bbox'][2] or q['bbox'][0] > p['bbox'][2] or p['bbox'][1] > q['bbox'][3] or q['bbox'][1] > p['bbox'][3]: continue
                if any(L in geo[j] and geo[i][L].distance(geo[j][L]) <= 1e-6 for L in geo[i]): self.kante(('P', i), ('P', j), 0.0, 0)
        if flaeche:
            for i in range(len(vias)): vl[i].add('#')
            kn = [(('V', i, '#'), (v['x'], v['y'])) for i, v in enumerate(vias)] + [(('P', i), (p['x'], p['y'])) for i, p in pads if p.get('drill')]
            for a, (ka, pa) in enumerate(kn):
                for kb, pb in kn[a + 1:]: self.kante(ka, kb, math.dist(pa, pb), 0)
        for i, Ls in sorted(vl.items()):
            Ls = sorted(Ls)
            for a, La in enumerate(Ls):
                for Lb in Ls[a + 1:]: self.kante(('V', i, La), ('V', i, Lb), 0.0, 1)

    def knoten(self, L, x, y):
        cx, cy = int(math.floor(x / TOL)), int(math.floor(y / TOL))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for px, py in self._kan.get((L, cx + dx, cy + dy), ()):
                    if math.hypot(px - x, py - y) <= TOL: return ('N', L, px, py)
        self._kan.setdefault((L, cx, cy), []).append((x, y)); return ('N', L, x, y)

    def kante(self, a, b, laenge, wechsel):
        self.adj[a].append((b, laenge, wechsel)); self.adj[b].append((a, laenge, wechsel))

    def wege(self, start):
        ""
        EPS = 1e-6
        dist = {start: 0.0}; lw = {start: 0}; kost = {start: 0.0}; vor = {}; h = [(0.0, 0, start)]; z = 0
        while h:
            k, _, n = heapq.heappop(h)
            if k > kost[n] + 1e-15: continue
            for m, l, x in self.adj[n]:
                nk = k + l + EPS * x
                if nk < kost.get(m, 1e18) - 1e-12:
                    kost[m] = nk; dist[m] = dist[n] + l; lw[m] = lw[n] + x; vor[m] = n; z += 1; heapq.heappush(h, (nk, z, m))
        return dist, lw, vor


class Pruefer:
    def __init__(self, B, K, R, p3_messen=None):
        self.B, self.K, self.R = B, K, R
        flag = (R.get('optionen') or {}).get('p3_messen', (K.get('vorgaben') or {}).get('p3_messen', bool(K.get('p3_pin_gruppen'))))
        self.p3_messen = str(flag).lower() in ('1', 'true') if p3_messen is None else p3_messen
        self._wege = {}
        self.N = K['netze']; self.GND = set(K.get('gnd') or [])
        self.fl = set(R.get('flaechennetze') or [])
        try: self.via_d = float(str((R.get('optionen') or {}).get('via') or (K.get('regeln') or {}).get('via') or '0.45').split('/')[0])
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
        self.alt = not any('funktion' in t for t in K['teile'].values())
        L = int(K.get('lagen') or (R.get('optionen') or {}).get('lagen') or 2)
        self.stapel = ['F'] + ['In%d' % i for i in range(1, L - 1)] + ['B']
        from analyse.flaechen import aktiv, kennzahl
        self.flaechenlagen = (set(kennzahl(B, R, K)['gndlagen']) if aktiv(R, K) else
                              {str(x).split('=')[0] for x in (R.get('gndlagen') or [])})

    def graph(self, n):
        if n not in self._g: self._g[n] = Kupfer(self.tr[n], self.vi[n], self.pads_net[n], self.via_r, flaeche=n in self.fl)
        return self._g[n]

    def wege(self, n, start):
        key = (n, start)
        if key not in self._wege: self._wege[key] = self.graph(n).wege(start)
        return self._wege[key]

    def flaechenmodell(self, n):
        return n in self.fl or any(v.get('net') == n for v in
            list(self.R.get('zonen') or []) + list(self.R.get('zones') or []))

    def geo(self, i):
        if i not in self._geo: self._geo[i] = padgeo(self.B['pads'][i])
        return self._geo[i]

    def abstand(self, i, j):
        ""
        return min(a.distance(b) for a in self.geo(i).values() for b in self.geo(j).values())

    def funktion(self, r):
        t = self.K['teile'][r]
        if 'funktion' in t: return t['funktion']
        if t.get('entkopplung') and t.get('C'): return 'entkopplung_' + ('klein' if t['C'] <= 220e-9 * 1.001 else 'mittel' if t['C'] <= 4.7e-6 * 1.001 else 'gross')
        return t.get('rolle') if t.get('rolle') != 'klein' else 'sonstig'

    def stuetzt(self, r):
        t = self.K['teile'][r]
        if t.get('stuetzt') or not self.alt: return t.get('stuetzt')
        for a, I in self.K.get('inseln', {}).items():
            for e in I.get('entkopplung', []):
                if e['ref'] == r and e.get('pins'): return dict(ref=a, pin=e['pins'][0], pins=e['pins'], netz=e['netz'])
        return None


    def entkopplung(self):
        out = []
        for r in sorted(self.K['teile']):
            f = self.funktion(r)
            if not f.startswith('entkopplung_'): continue
            st = self.stuetzt(r) or {}; stufe = f.split('_', 1)[1]
            cp = [(i, p) for i, p in self.pads_ref[r] if p['net'] and p['net'] not in self.GND]
            e = dict(teil=r, stufe=stufe, C=self.K['teile'][r].get('C'), ausgang=bool(st.get('ueber_spule')), pin=None, abstand=None, bahn=None,
                     lagenwechsel=None, netz=cp[0][1]['net'] if cp else None, x=cp[0][1]['x'] if cp else None, y=cp[0][1]['y'] if cp else None)
            nums = set(st.get('pins') or ([st['pin']] if st.get('pin') else []))
            ziel = [(i, p) for i, p in self.pads_ref.get(st.get('ref'), []) if p['net'] == st.get('netz') and p['num'] in nums]
            if cp and ziel:
                ci = next((i for i, p in cp if p['net'] == st['netz']), cp[0][0])
                zi, zp = min(ziel, key=lambda ip: (self.abstand(ci, ip[0]), ip[0]))
                d, lw, _ = self.wege(st['netz'], ('P', ci))
                e.update(pin='%s:%s' % (zp['ref'], zp['num']), abstand=round(self.abstand(ci, zi), 3), netz=st['netz'],
                         bahn=round(d[('P', zi)], 3) if ('P', zi) in d else None, lagenwechsel=int(round(lw[('P', zi)])) if ('P', zi) in d else None,
                         seite='gleich' if set(self.B['pads'][ci]['layers']) & set(zp['layers']) else 'gegenueber')
            out.append(e)
        gruppen = {}
        for g in ('klein', 'mittel', 'gross', 'ausgang'):
            es = [e for e in out if (e['stufe'] == g and not e['ausgang']) or (g == 'ausgang' and e['ausgang'])]
            mit = [e for e in es if e['abstand'] is not None]
            if not es: continue
            soll = SOLL['abstand'].get(g, SOLL['abstand']['gross'])
            gruppen[g] = dict(anzahl=len(es), ohne_pin=len(es) - len(mit), soll_abstand=soll,
                              abstand_median=_med([e['abstand'] for e in mit]), abstand_max=max([e['abstand'] for e in mit], default=None),
                              bahn_median=_med([e['bahn'] for e in mit if e['bahn'] is not None]),
                              bahn_max=max([e['bahn'] for e in mit if e['bahn'] is not None], default=None),
                              ueber_soll=sum(1 for e in mit if e['abstand'] > soll), mit_lagenwechsel=sum(1 for e in mit if e['lagenwechsel']),
                              nicht_verbunden=sum(1 for e in mit if e['bahn'] is None))
        result = dict(gruppen=gruppen, teile=out)
        if self.p3_messen:
            try: result['schleifen'] = self.entkopplung_schleifen(out)
            except Exception as exc: result['schleifen_fehler'] = repr(exc)[:200]
        return result

    def entkopplung_schleifen(self, teile):
        ""
        from platzierung.insel_physik import huelle
        out = []
        for e in teile:
            if e['ausgang']: continue
            r = e['teil']; st = self.stuetzt(r) or {}
            z = dict(teil=r, stufe=e['stufe'], huelle_mm2=None, luft_mm=None,
                     bahn_mm=None, masse_bahn_mm=None, lagenwechsel=None,
                     modell='Padmitten; Kupfergraph; keine nH')
            cp = [(i, p) for i, p in self.pads_ref[r] if p['net'] == e['netz']]
            zp = [(i, p) for i, p in self.pads_ref.get(st.get('ref'), [])
                  if '%s:%s' % (p['ref'], p['num']) == e['pin']]
            pairs = [(ci, c, gi, g) for ci, c in self.pads_ref[r] if c['net'] in self.GND
                     for gi, g in self.pads_ref.get(st.get('ref'), []) if g['net'] == c['net']]
            if cp and zp and pairs:
                _, c = cp[0]; _, p = zp[0]
                ci, cm, gi, pg = min(pairs, key=lambda q: (math.dist((q[3]['x'], q[3]['y']), (p['x'], p['y'])), q[0], q[2]))
                pts = [(v['x'], v['y']) for v in (p, c, cm, pg)]
                z.update(huelle_mm2=round(huelle(pts), 4),
                         luft_mm=round(math.dist(pts[0], pts[1]) + math.dist(pts[2], pts[3]), 4),
                         masse_pin='%s:%s' % (pg['ref'], pg['num']))


                versorgung_flaeche = self.flaechenmodell(e['netz'])
                if versorgung_flaeche:
                    z['modell'] += '; Versorgungsflaeche: Kupferweg unbekannt'
                if self.flaechenmodell(cm['net']):
                    z['modell'] += '; Masseflaeche: Kupferweg unbekannt'
                else:
                    d, lw, _ = self.wege(cm['net'], ('P', ci))
                    if ('P', gi) in d:
                        z['masse_bahn_mm'] = round(d['P', gi], 4)
                        if e['bahn'] is not None and not versorgung_flaeche:
                            z['bahn_mm'] = round(e['bahn'] + d['P', gi], 4)
                            z['lagenwechsel'] = e['lagenwechsel'] + int(round(lw['P', gi]))
            out.append(z)
        def zahlen(es):
            geo_fehlend = sum(v['luft_mm'] is None for v in es)
            kupfer_unbekannt = sum(v['bahn_mm'] is None for v in es)
            return dict(anzahl=len(es), geometrie_fehlend=geo_fehlend,
                        kupferweg_unbekannt=kupfer_unbekannt,
                        luft_summe_mm=None if geo_fehlend else round(sum(v['luft_mm'] for v in es), 4),
                        luft_max_mm=None if geo_fehlend else max((v['luft_mm'] for v in es), default=None),
                        huelle_summe_mm2=None if geo_fehlend else round(sum(v['huelle_mm2'] for v in es), 4),
                        bahn_max_mm=None if kupfer_unbekannt else max((v['bahn_mm'] for v in es), default=None))
        return dict(version='p3-schleife-v1', modell='Geometrie/Bahnmodell, keine Hardwaremessung',
                    teile=out, **zahlen(out),
                    gruppen={g:zahlen([v for v in out if v['stufe']==g]) for g in ('klein', 'mittel', 'gross')
                             if any(v['stufe']==g for v in out)})


    def kette(self):
        out = []
        for a, I in sorted(self.K.get('inseln', {}).items()):
            je_netz = collections.defaultdict(list)
            for e in I.get('entkopplung', []):
                if not e.get('ueber_spule') and e.get('pins'):
                    gruppe = tuple(sorted(e['pins'])) if self.K.get('p3_pin_gruppen') else ()
                    je_netz[e['netz'], gruppe].append(e)
            for (n, _gruppe), es in sorted(je_netz.items()):
                es = sorted(es, key=lambda e: (e['C'], e['ref']))
                pins = [(i, p) for i, p in self.pads_ref[a] if p['net'] == n and p['num'] in set(es[0]['pins'])]
                cpad = {e['ref']: next((i for i, p in self.pads_ref[e['ref']] if p['net'] == n), None) for e in es}
                if not pins or cpad[es[0]['ref']] is None: continue
                wc = {r: self.wege(n, ('P', i)) for r, i in cpad.items() if i is not None}
                d0 = wc[es[0]['ref']][0]
                pi, pp = min(pins, key=lambda ip: (d0.get(('P', ip[0]), 1e9), self.abstand(cpad[es[0]['ref']], ip[0]), ip[0]))
                dp, lwp, vor = self.wege(n, ('P', pi))
                quelle = set((self.N.get(n, {}).get('fluss') or {}).get('quelle') or [])
                qpads = [i for i, p in self.pads_net[n] if '%s:%s' % (p['ref'], p['num']) in quelle]
                ausgang = any(self.B['pads'][i]['ref'] == a for i in qpads)
                caps_refs = {e['ref'] for e in es}
                if ausgang: ziele = [i for i, p in self.pads_net[n] if p['ref'] != a and p['ref'] not in caps_refs and not self.funktion(p['ref']).startswith('entkopplung_')]
                else: ziele = sorted([i for i in qpads if ('P', i) in dp], key=lambda i: dp[('P', i)])[:1]
                k = dict(chip=a, pin='%s:%s' % (a, pp['num']), netz=n, modus='ausgang' if ausgang else 'eingang', x=pp['x'], y=pp['y'], kondensatoren=[], urteil=None)
                for e in es:
                    r = e['ref']; ci = cpad[r]
                    if ci is None or ('P', ci) not in dp:
                        k['kondensatoren'].append(dict(teil=r, C=e['C'], bahn=None)); continue
                    dc = wc[r][0]; st = [(dp[('P', ci)] + dc[('P', z)] - dp[('P', z)]) / 2 for z in ziele if ('P', z) in dp and ('P', z) in dc]
                    k['kondensatoren'].append(dict(teil=r, C=e['C'], bahn=round(dp[('P', ci)], 3), lagenwechsel=int(round(lwp[('P', ci)])),
                                                   stich=round(max(0.0, min(st)), 3) if st and not ausgang else None,
                                                   anteil_dahinter=round(sum(1 for s in st if s <= SOLL['auf_weg']) / len(st), 2) if st and ausgang else None,
                                                   lage=round(dp[('P', ci)] - max(0.0, min(st)), 3) if st and not ausgang else None))
                erst = k['kondensatoren'][0]
                via_lage = None
                if not ausgang and ziele and ('P', ziele[0]) in dp:
                    kn = ('P', ziele[0]); k['weg_zur_quelle'] = round(dp[kn], 3)
                    while kn in vor:
                        if kn[0] == 'V' and vor[kn][0] == 'V': via_lage = dp[kn]
                        kn = vor[kn]
                    k['erstes_via'] = round(via_lage, 3) if via_lage is not None else None
                if erst.get('bahn') is None: k['urteil'] = 'nicht_verbunden'
                elif ausgang:
                    k['urteil'] = 'kette' if erst['anteil_dahinter'] == 1.0 and not erst['lagenwechsel'] else ('via_vor_c' if erst['lagenwechsel'] else 'abzweig')
                elif not ziele: k['urteil'] = 'quelle_unbekannt' if not erst['lagenwechsel'] else 'via_vor_c'
                elif erst['lagenwechsel'] or (via_lage is not None and via_lage < erst['lage'] - SOLL['auf_weg']): k['urteil'] = 'via_vor_c'
                elif erst['stich'] > SOLL['auf_weg']: k['urteil'] = 'abzweig'
                else: k['urteil'] = 'kette'
                lg = [c['lage'] for c in k['kondensatoren'] if c.get('lage') is not None]
                k['reihenfolge'] = None if ausgang or len(lg) < 2 else all(x <= y + SOLL['auf_weg'] for x, y in zip(lg[:-1], lg[1:]))
                out.append(k)
        return dict(anzahl=len(out), urteile=dict(collections.Counter(k['urteil'] for k in out)), ketten=out)


    def massepads(self):
        ""
        from shapely.geometry import Polygon
        ep = {}
        for c, t in sorted(self.K['teile'].items()):
            if t.get('rolle') != 'chip' or len(self.pads_ref[c]) < 5: continue
            fl = {i: sum(g.area for g in self.geo(i).values()) / max(1, len(self.geo(i))) for i, p in self.pads_ref[c]}
            med = statistics.median(fl.values())
            kand = [(i, p) for i, p in self.pads_ref[c] if p['net'] in self.GND and fl[i] > 2.5 * med and fl[i] >= 0.5]
            rest = [p for i, p in self.pads_ref[c] if (i, p) not in kand]
            if not kand or len(rest) < 3: continue
            x0, x1 = min(p['x'] for p in rest), max(p['x'] for p in rest); y0, y1 = min(p['y'] for p in rest), max(p['y'] for p in rest)
            mx, my = 0.15 * (x1 - x0), 0.15 * (y1 - y0)
            for i, p in kand:
                innen_x = x1 - x0 < 1e-6 or x0 + mx <= p['x'] <= x1 - mx; innen_y = y1 - y0 < 1e-6 or y0 + my <= p['y'] <= y1 - my
                if innen_x and innen_y: ep.setdefault((c, p['num']), []).append(i)
        return ep

    def masse_vias(self):
        ep = {i for v in self.massepads().values() for i in v}
        pads = []; kreis = Point(0, 0).buffer(self.via_r).area
        for g in sorted(self.GND):
            G = self.graph_ohne_flaeche(g); vias = self.vi[g]
            for i, p in self.pads_net[g]:
                r = p['ref']; t = self.K['teile'].get(r, {}); f = self.funktion(r) if r in self.K['teile'] else 'sonstig'
                gruppe = 'entkopplung' if f.startswith('entkopplung_') else ('chip' if t.get('rolle') == 'chip' else 'rest')
                e = dict(pad='%s:%s' % (r, p['num']), teil=r, gruppe=gruppe, stufe=f.split('_', 1)[1] if gruppe == 'entkopplung' else None, x=p['x'], y=p['y'],
                         massepad=i in ep)
                if p.get('drill'): e.update(art='durchkontaktiert', stich=0.0, anteil=None); pads.append(e); continue
                d, _, _ = G.wege(('P', i))
                je = {}
                for kn, dv in d.items():
                    if kn[0] == 'V': je[kn[1]] = min(je.get(kn[1], 1e9), dv)
                vd = sorted((dv, j) for j, dv in je.items())
                if not vd: e.update(art='ohne_via', stich=None, anteil=0.0, via=None); pads.append(e); continue
                def anteil(j): return max(Point(vias[j]['x'], vias[j]['y']).buffer(self.via_r).intersection(gg).area for gg in self.geo(i).values()) / kreis

                nah = [(anteil(j), -dj, -j) for dj, j in vd if dj <= self.via_r + TOL + 1e-9]
                if nah and max(nah)[0] > 1e-6: ant, st, vi_ = max(nah); st, vi_ = -st, -vi_
                else: (st, vi_), ant = vd[0], 0.0
                v = vias[vi_]
                e.update(stich=round(st, 3), anteil=round(ant, 3), via=vi_, via_x=v['x'], via_y=v['y'],
                         art='im_pad' if ant >= SOLL['im_pad'] - 1e-9 else ('am_pad' if ant > 1e-6 else 'stich'))
                pads.append(e)
        je_via = collections.Counter(e['via'] for e in pads if e.get('via') is not None)
        for e in pads: e['geteilt'] = je_via.get(e.get('via'), 0) if e.get('via') is not None else 0
        gruppen = {}
        for g in ('entkopplung', 'chip', 'rest'):
            es = [e for e in pads if e['gruppe'] == g and e['art'] != 'durchkontaktiert']
            if not es: continue
            st = [e['stich'] for e in es if e['art'] == 'stich']
            gruppen[g] = dict(pads=len(es), im_pad=sum(1 for e in es if e['art'] == 'im_pad'),
                              im_pad_halb=sum(1 for e in es if e['art'] == 'im_pad' and e['anteil'] >= SOLL['im_pad_besser'] - 1e-9),
                              am_pad=sum(1 for e in es if e['art'] == 'am_pad'), stich=len(st), stich_median=_med(st), stich_max=max(st, default=None),
                              stich_ueber_soll=sum(1 for s in st if s > SOLL['stich']), ohne_via=sum(1 for e in es if e['art'] == 'ohne_via'),
                              geteiltes_via=sum(1 for e in es if e['geteilt'] > 1))
        return dict(gruppen=gruppen, soll_stich=SOLL['stich'], pads=pads)

    def graph_ohne_flaeche(self, n):
        k = (n, 'ohne')
        if k not in self._g: self._g[k] = Kupfer(self.tr[n], self.vi[n], self.pads_net[n], self.via_r, flaeche=False)
        return self._g[k]

    def massepad(self):
        out = []
        for (c, num), idx in sorted(self.massepads().items()):
            ps = [self.B['pads'][i] for i in idx]; n = ps[0]['net']
            fl = sum(max(g.area for g in self.geo(i).values()) for i in idx)
            geos = [gg for i in idx for gg in self.geo(i).values()]
            drin = [v for v in self.vi[n] if any(gg.contains(Point(v['x'], v['y'])) for gg in geos)]
            teil = [v for v in self.vi[n] if v not in drin and any(Point(v['x'], v['y']).buffer(self.via_r).intersection(gg).area >= SOLL['im_pad'] * math.pi * self.via_r ** 2 for gg in geos)]
            p_ = self.via_d + 0.05; passt = 0
            for q in ps:
                bw, bh = q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]
                passt += (int((bw - self.via_d) / p_) + 1 if bw >= self.via_d else 0) * (int((bh - self.via_d) / p_) + 1 if bh >= self.via_d else 0)
            soll = SOLL['massepad_gross'] if fl >= SOLL['massepad_gross_ab'] else SOLL['massepad_min']
            out.append(dict(chip=c, pad=num, netz=n, flaeche=round(fl, 2), teilpads=len(idx), vias=len(drin), vias_am_rand=len(teil), soll=soll, passt=passt,
                            fehlen=max(0, soll - len(drin) - len(teil)), x=round(sum(q['x'] for q in ps) / len(ps), 3), y=round(sum(q['y'] for q in ps) / len(ps), 3)))
        return dict(anzahl=len(out), unter_soll=sum(1 for e in out if e['fehlen']), pads=out)


    def quarz(self):
        out = []
        for r in sorted(self.K['teile']):
            if self.funktion(r) != 'quarz': continue
            st = self.stuetzt(r) or {}; q = dict(teil=r, chip=st.get('ref'), netze=[], laengste=None)
            for n in st.get('netze') or []:
                qi = next((i for i, p in self.pads_ref[r] if p['net'] == n), None)
                if qi is None: continue
                g = self.graph(n); d, lw, _ = g.wege(('P', qi))
                e = dict(netz=n, x=self.B['pads'][qi]['x'], y=self.B['pads'][qi]['y'], chip_bahn=None, ueber_r=None, last=[])
                ci = next((i for i, p in self.pads_net[n] if p['ref'] == st.get('ref')), None)
                if ci is not None and ('P', ci) in d: e.update(chip_bahn=round(d[('P', ci)], 3), chip_lagenwechsel=int(round(lw[('P', ci)])), chip_abstand=round(self.abstand(qi, ci), 3))
                elif ci is None:
                    for rr in sorted(self.K['teile']):
                        s2 = self.K['teile'][rr].get('stuetzt') or {}
                        if self.funktion(rr) == 'serien_r' and s2.get('quarz') == r and any(p['net'] == n for _, p in self.pads_ref[rr]):
                            ri = next(i for i, p in self.pads_ref[rr] if p['net'] == n); ro = next((i for i, p in self.pads_ref[rr] if p['net'] != n), None)
                            cj = next((i for i, p in self.pads_ref[st['ref']] if ro is not None and p['net'] == self.B['pads'][ro]['net']), None)
                            if cj is not None and ('P', ri) in d:
                                d2 = self.graph(self.B['pads'][ro]['net']).wege(('P', ro))[0]
                                if ('P', cj) in d2: e.update(chip_bahn=round(d[('P', ri)] + d2[('P', cj)], 3), ueber_r=rr)
                for c in sorted(self.K['teile']):
                    s2 = self.K['teile'][c].get('stuetzt') or {}
                    if self.funktion(c) == 'quarz_last' and s2.get('quarz') == r:
                        li = next((i for i, p in self.pads_ref[c] if p['net'] == n), None)
                        if li is not None: e['last'].append(dict(teil=c, bahn=round(d[('P', li)], 3) if ('P', li) in d else None,
                                                                 lagenwechsel=int(round(lw[('P', li)])) if ('P', li) in d else None))
                q['netze'].append(e)
            ls = [e['chip_bahn'] for e in q['netze'] if e['chip_bahn'] is not None] + [l['bahn'] for e in q['netze'] for l in e['last'] if l['bahn'] is not None]
            q['laengste'] = max(ls, default=None); out.append(q)
        return dict(anzahl=len(out), soll=SOLL['quarz'], ueber_soll=sum(1 for q in out if q['laengste'] is None or q['laengste'] > SOLL['quarz']), quarze=out)


    def neben_flaeche(self, lage):
        if lage not in self.stapel: return False
        k = self.stapel.index(lage)
        return any(0 <= j < len(self.stapel) and self.stapel[j] in self.flaechenlagen for j in (k - 1, k + 1))

    def schnell(self):
        out = []
        for n, k in sorted(self.N.items()):
            if k.get('art') != 'schnell': continue
            je = collections.defaultdict(float)
            for t in self.tr[n]:
                for a, b in zip(t['pts'][:-1], t['pts'][1:]): je[t['layer']] += math.dist(a, b)
            L = sum(je.values()); luft = _mst([(p['x'], p['y']) for _, p in self.pads_net[n]])
            innen = sum(v for lg, v in je.items() if lg not in ('F', 'B') and self.neben_flaeche(lg))
            neben = sum(v for lg, v in je.items() if self.neben_flaeche(lg))
            p0 = self.pads_net[n][0][1] if self.pads_net[n] else {}
            out.append(dict(netz=n, rang=k.get('rang'), grund=k.get('grund'), pads=len(self.pads_net[n]), vias=len(self.vi[n]), laenge=round(L, 2), luftlinie=round(luft, 2),
                            verhaeltnis=round(L / luft, 2) if luft > 1e-6 and L else None, anteil_innen_neben_flaeche=round(innen / L, 2) if L else None,
                            anteil_neben_flaeche=round(neben / L, 2) if L else None, je_lage={lg: round(v, 2) for lg, v in sorted(je.items())},
                            paar=k.get('paar'), x=p0.get('x'), y=p0.get('y')))
        out.sort(key=lambda e: (-(e['rang'] or 0), -e['vias'], e['netz']))
        Ls = sum(e['laenge'] for e in out)
        return dict(anzahl=len(out), vias=sum(e['vias'] for e in out), netze_mit_vias=sum(1 for e in out if e['vias']),
                    anteil_innen_neben_flaeche=round(sum(e['laenge'] * (e['anteil_innen_neben_flaeche'] or 0) for e in out) / Ls, 2) if Ls else None,
                    flaechenlagen=sorted(self.flaechenlagen), netze=out)

    def vias_unnoetig(self):
        ""
        out = []; einseitig = 0
        for n, ps in sorted(self.pads_net.items()):
            if n in self.GND or n in self.fl or len(ps) < 2 or n not in self.N: continue
            if any(p.get('drill') for _, p in ps): continue
            seiten = {tuple(p['layers']) for _, p in ps}
            if len(seiten) != 1 or len(next(iter(seiten))) != 1: continue
            einseitig += 1
            nv = [v for v in self.vi[n] if not v.get('strom_via')]
            if not nv: continue
            L = sum(math.dist(a, b) for t in self.tr[n] for a, b in zip(t['pts'][:-1], t['pts'][1:]))
            luft = _mst([(p['x'], p['y']) for _, p in ps])
            out.append(dict(netz=n, art=self.N[n].get('art'), rang=self.N[n].get('rang'), seite=next(iter(seiten))[0], pads=len(ps), vias=len(nv),
                            laenge=round(L, 2), luftlinie=round(luft, 2), x=nv[0]['x'], y=nv[0]['y']))
        out.sort(key=lambda e: (-e['vias'], e['netz']))
        je_art = collections.Counter()
        for e in out: je_art[e['art']] += e['vias']
        return dict(netze_einseitig=einseitig, netze_mit_vias=len(out), vias=sum(e['vias'] for e in out), je_art=dict(sorted(je_art.items())), netze=out,
                    hinweis='Via-Kandidaten auf einseitigen Netzen; freie Alternative nicht geprueft.')


    def mindestbreite(self):
        out = []
        for n, k in sorted(self.N.items()):
            if k.get('art') not in ('strom', 'versorgung', 'sw') or 'wmin' not in k: continue
            wmin = k['wmin']; unter = 0.0; ges = 0.0; schlecht = None; abgriff = 0.0
            for t in self.tr[n]:
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                    l = math.dist(a, b)
                    if t.get('sig'): abgriff += l; continue
                    ges += l
                    if w < wmin - 1e-6:
                        unter += l
                        if schlecht is None or (w, -l) < (schlecht[0], -schlecht[1]): schlecht = (w, l, a[0], a[1], t['layer'])
            if unter > 0:
                out.append(dict(netz=n, art=k['art'], strom=k.get('strom'), wmin=wmin, w_soll=k.get('w'), laenge_unter=round(unter, 2), laenge=round(ges, 2),
                                schmalste=schlecht[0], schmalste_laenge=round(schlecht[1], 2), x=schlecht[2], y=schlecht[3], lage=schlecht[4], abgriff=round(abgriff, 2)))
        out.sort(key=lambda e: (-(e['wmin'] - e['schmalste']) * e['laenge_unter'], e['netz']))
        return dict(netze=len(out), laenge_unter=round(sum(e['laenge_unter'] for e in out), 2), stellen=out)

    def alles(self):
        E = dict(version='regelpruefung-v1', soll=SOLL, via_durchmesser=self.via_d, ohne_funktion=self.alt,
                 konflikte=len(self.R.get('conflicts') or {}), offen=len(self.R.get('failed') or []))
        E['entkopplung'] = self.entkopplung(); E['kette'] = self.kette(); E['masse_vias'] = self.masse_vias(); E['massepad'] = self.massepad()
        E['quarz'] = self.quarz(); E['schnell'] = self.schnell(); E['vias_unnoetig'] = self.vias_unnoetig(); E['mindestbreite'] = self.mindestbreite()
        return E

def _med(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 3) if xs else None


def _mst(pts):
    if len(pts) < 2: return 0.0
    drin = [pts[0]]; rest = list(pts[1:]); L = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in drin), i) for i, r in enumerate(rest)); L += d; drin.append(rest.pop(i))
    return L


def pruefe(B, K, R, p3_messen=None):
    ""
    return Pruefer(B, K, R, p3_messen=p3_messen).alles()


def _z(v, f='%.2f'): return '-' if v is None else f % v


def bericht(E, top=5):
    o = []; bn = lambda n: basisname(n) if n else '-'
    o.append('Regelpruefung (%s): Via %.2f mm, Konflikte %d, offen %d%s' % (E['version'], E['via_durchmesser'], E['konflikte'], E['offen'],
             ' | klassen.json ohne Bauteilfunktion: Entkopplung aus den Inseln, Quarz ungeprueft' if E['ohne_funktion'] else ''))
    e = E['entkopplung']
    o.append('Entkopplung (P3, P20): Abstand Pin-C Kante zu Kante, Bahn im Kupfer, Lagenwechsel')
    for g, s in e['gruppen'].items():
        o.append('  %-8s %2d Stueck | Abstand Median %s, max %s mm (Soll <= %.1f: %d darueber) | Bahn Median %s, max %s mm | mit Lagenwechsel %d | nicht verbunden %d%s' % (
            g, s['anzahl'], _z(s['abstand_median']), _z(s['abstand_max']), s['soll_abstand'], s['ueber_soll'], _z(s['bahn_median']), _z(s['bahn_max']),
            s['mit_lagenwechsel'], s['nicht_verbunden'], ' | ohne Pin %d' % s['ohne_pin'] if s['ohne_pin'] else ''))
    rang = {'klein': 0, 'mittel': 1, 'gross': 2}
    schl = sorted([t for t in e['teile'] if t['abstand'] is not None and not t['ausgang']],
                  key=lambda t: (rang[t['stufe']], -(t['lagenwechsel'] or 0), -(t['bahn'] if t['bahn'] is not None else 99), t['teil']))
    for t in schl[:top + 3]:
        o.append('    %-9s %-6s -> %-10s Abstand %s mm, Bahn %s mm, Lagenwechsel %s, Seite %s  (%.2f, %.2f)' % (
            t['teil'], t['stufe'], t['pin'], _z(t['abstand']), _z(t['bahn']), _z(t['lagenwechsel'], '%d'), t.get('seite', '-'), t['x'], t['y']))
    k = E['kette']
    o.append('Versorgung in Kette (R22): %d Ketten, Urteile %s' % (k['anzahl'], k['urteile']))
    for c in k['ketten']:
        cs = ', '.join('%s %s (Bahn %s, %s, LW %s)' % (x['teil'], _si(x['C']), _z(x.get('bahn')),
                       ('Stich %s' % _z(x.get('stich'))) if c['modus'] == 'eingang' else ('dahinter %s' % _z(x.get('anteil_dahinter'))), _z(x.get('lagenwechsel'), '%d'))
                       for x in c['kondensatoren'])
        o.append('    %-10s %-8s %-7s %-16s erstes Via bei %s mm, Reihenfolge %s | %s  (%.2f, %.2f)' % (
            c['pin'], bn(c['netz']), c['modus'], c['urteil'], _z(c.get('erstes_via')), {None: '-', True: 'ok', False: 'falsch'}[c['reihenfolge']], cs, c['x'], c['y']))
    m = E['masse_vias']
    o.append('Masse-Vias (R8, R9, R23, R25): im Pad ab 1/3 der Via-Flaeche, Stich = Bahn vom Pad zum naechsten Via (Soll <= %.1f mm)' % m['soll_stich'])
    for g, s in m['gruppen'].items():
        o.append('  %-11s %3d Pads | im Pad %d (davon >= 1/2: %d) | am Pad %d | Stich %d (Median %s, max %s mm, ueber Soll %d) | ohne Via %d | Via mit anderen geteilt %d' % (
            g, s['pads'], s['im_pad'], s['im_pad_halb'], s['am_pad'], s['stich'], _z(s['stich_median']), _z(s['stich_max']), s['stich_ueber_soll'], s['ohne_via'], s['geteiltes_via']))
    schl = sorted([p for p in m['pads'] if p['art'] in ('stich', 'ohne_via')], key=lambda p: (p['art'] != 'ohne_via', -(p['stich'] or 0), p['pad']))
    for p in schl[:top + 3]:
        o.append('    %-12s %-11s %s%s  (%.2f, %.2f)' % (p['pad'], p['gruppe'] + ('/' + p['stufe'] if p['stufe'] else ''),
                 'ohne Via' if p['art'] == 'ohne_via' else 'Stich %.2f mm' % p['stich'], ', Via geteilt mit %d Pads' % p['geteilt'] if p['geteilt'] > 1 else '', p['x'], p['y']))
    mp = E['massepad']
    o.append('Massepad unter Chips (R16): %d Pads, %d unter Soll' % (mp['anzahl'], mp['unter_soll']))
    for p in mp['pads']:
        o.append('    %-8s Pad %-3s %5.2f mm2 (%d Teilpads): %d Vias im Pad, %d am Rand; Soll %d, Platz fuer etwa %d; fehlen %d  (%.2f, %.2f)' % (
            p['chip'], p['pad'], p['flaeche'], p['teilpads'], p['vias'], p['vias_am_rand'], p['soll'], p['passt'], p['fehlen'], p['x'], p['y']))
    q = E['quarz']
    o.append('Quarz (P20): %d Quarze, %d ueber dem Richtwert %.0f mm' % (q['anzahl'], q['ueber_soll'], q['soll']))
    for x in q['quarze']:
        for n in x['netze']:
            o.append('    %-6s %-10s -> %s Bahn %s mm%s | Last: %s  (%.2f, %.2f)' % (x['teil'], bn(n['netz']), x['chip'], _z(n['chip_bahn']), ' ueber ' + n['ueber_r'] if n['ueber_r'] else '',
                     ', '.join('%s %s mm' % (l['teil'], _z(l['bahn'])) for l in n['last']) or '-', n['x'], n['y']))
    s = E['schnell']
    o.append('Schnelle Netze (R14, R19): %d Netze, %d Vias, %d Netze mit Vias, Anteil auf Innenlagen neben einer Flaeche %s (Flaechenlagen %s)' % (
        s['anzahl'], s['vias'], s['netze_mit_vias'], _z(s['anteil_innen_neben_flaeche']), ','.join(s['flaechenlagen']) or '-'))
    for n in s['netze'][:max(top, 8)]:
        o.append('    %-16s Rang %s: %d Vias, %s mm bei Luftlinie %s (x %s), innen neben Flaeche %s, neben Flaeche gesamt %s, je Lage %s' % (
            bn(n['netz']), n['rang'], n['vias'], _z(n['laenge']), _z(n['luftlinie']), _z(n['verhaeltnis']), _z(n['anteil_innen_neben_flaeche']),
            _z(n['anteil_neben_flaeche']), n['je_lage']))
    v = E['vias_unnoetig']
    o.append('Via-Kandidaten (R19, freie Alternative nicht geprueft): %d von %d einseitigen Netzen haben Vias, zusammen %d (je Art %s)' % (v['netze_mit_vias'], v['netze_einseitig'], v['vias'], v['je_art']))
    for n in v['netze'][:top + 3]:
        o.append('    %-20s %-10s Seite %s, %d Pads: %d Vias, %s mm bei Luftlinie %s  (%.2f, %.2f)' % (bn(n['netz']), n['art'], n['seite'], n['pads'], n['vias'], _z(n['laenge']), _z(n['luftlinie']), n['x'], n['y']))
    b = E['mindestbreite']
    o.append('Unter der Mindestbreite der Klasse (Stromnetze): %d Netze, %.1f mm' % (b['netze'], b['laenge_unter']))
    for n in b['stellen'][:top + 3]:
        o.append('    %-14s %-10s wmin %.2f: %.1f von %.1f mm schmaler, schmalste Stelle %.2f mm (%.2f mm lang, Lage %s)  (%.2f, %.2f)' % (
            bn(n['netz']), n['art'], n['wmin'], n['laenge_unter'], n['laenge'], n['schmalste'], n['schmalste_laenge'], n['lage'], n['x'], n['y']))
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
    E = pruefe(B, K, R)
    print(bericht(E, top=int(kw.get('top', 5))))
    if 'json' in kw: json.dump(E, open(kw['json'], 'w'), indent=1)
