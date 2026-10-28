""
import collections
import heapq
import math
import re


def vorgabe(K, n, feld, standard=None):
    ""
    vg = K.get('vorgaben') or {}
    tab = vg.get(feld)
    if isinstance(tab, dict):
        for rx, v in sorted(tab.items()):
            try:
                if re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): return v
            except re.error: continue
    elif feld in vg: return tab
    for rx, k in (vg.get('netze') or {}).items():
        try:
            if re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]):
                if feld in k: return k[feld]
                break
        except re.error: continue
    return (K['netze'].get(n) or {}).get(feld, standard)


def zahl(v, default, minimum=0.0):
    if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= minimum: return float(v)
    return default


def gamma(z1, z2):
    return abs((z2 - z1) / (z2 + z1))


def _auf_segment(p, a, b, tol=0.03):
    dx, dy = b[0]-a[0], b[1]-a[1]; ll=dx*dx+dy*dy
    u = max(0.0,min(1.0,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/ll)) if ll else 0.0
    return math.dist(p,(a[0]+u*dx,a[1]+u*dy))<=tol


def digital(k):
    return k.get('art') == 'schnell' or (k.get('art') == 'signal' and k.get('rang', 0) >= 3)


def parameter(K, n, C):
    ""
    f = zahl(vorgabe(K, n, 'f_Hz'), C.get('f_Hz', 50e6), 1.0)
    rate = zahl(vorgabe(K, n, 'datenrate'), None, 1.0)
    q = 'datenrate'
    if rate is None: rate = 2 * f; q = '2*f_Hz (Annahme)'
    raw_tr = vorgabe(K, n, 'tr_ns', C.get('tr_ns', 2.0))
    tr = None if raw_tr is None else zahl(raw_tr, C.get('tr_ns') or 2.0, 1e-6)
    zd = zahl(vorgabe(K, n, 'z_diff_ohm'), None, 1.0)
    zs = zahl(vorgabe(K, n, 'z_ohm'), C.get('z_ohm', 50.0), 1.0)
    zq = 'Vorgabe/Klasse' if zd is not None else None
    partner = vorgabe(K, n, 'paar')
    if zd is None and partner:
        zd = zahl(C.get('z_diff_ohm'), 2 * zs, 1.0)
        zq = 'Klassenstandard' if C.get('z_diff_ohm') else '2*z_ohm (Paarannahme)'
    gruppe = vorgabe(K, n, 'laengengruppe', C.get('laengengruppe'))


    if not gruppe and partner: gruppe = 'paar:' + '|'.join(sorted((n, partner)))
    interface = vorgabe(K, n, 'schnittstelle', C.get('schnittstelle')) or gruppe
    if not interface: interface = 'paar:' + '|'.join(sorted((n, partner))) if partner else n
    return dict(f_Hz=f, datenrate=rate, datenrate_quelle=q, ui_ns=1e9 / rate, tr_ns=tr, z_ohm=zs,
                z_diff_ohm=zd, z_diff_quelle=zq, gruppe=gruppe,
                toleranz_mm=zahl(vorgabe(K, n, 'toleranz_mm'), C.get('toleranz_mm')),
                schnittstelle=interface)


def _wege(g, seg, A, start):
    ""
    adj = g['adj']; best = {start: 0.0}; dist = {start: 0.0}; h = [(0.0, start)]
    while h:
        d, k = heapq.heappop(h)
        if d > best[k]: continue
        for m, kid in adj[k]:
            t = l = 0.0
            if kid and kid[0] == 's':
                s = seg[kid[1:3]]; l = s['l'] * (kid[4] - kid[3]); t = s['tpd'] * l / 1000
            elif kid and kid[0] == 'v':
                l = abs(A['lagen'][k[0]]['z'] - A['lagen'][m[0]]['z']); t = l * math.sqrt(A['er']) * 3.336 / 1000
            if d + t < best.get(m, math.inf):
                best[m] = d + t; dist[m] = dist[k] + l; heapq.heappush(h, (d + t, m))
    return best, dist


def _zweig(adj, seg, k, nxt, kid):
    ""
    t = l = 0.0; seen = {k}
    while nxt not in seen:
        seen.add(nxt)
        if kid and kid[0] == 's':
            s = seg[kid[1:3]]; dl = s['l'] * (kid[4] - kid[3]); l += dl; t += s['tpd'] * dl / 1000
        rest = [(m, ki) for m, ki in adj[nxt] if m != k]
        if nxt[0] == 'PAD' or len(rest) != 1: break
        k, (nxt, kid) = nxt, rest[0]
    return l, t


def _paar_gap(a, b, w, partner, layer):
    ""
    from analyse.kopplung import _paar
    length = math.dist(a, b)
    if length < 1e-9: return None
    ux, uy = (b[0]-a[0])/length, (b[1]-a[1])/length
    spans = collections.defaultdict(list)
    for t in partner:
        if t['layer'] != layer: continue
        for c, d, w2 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            dl = math.dist(c, d)
            if dl < 1e-9: continue
            pair = _paar(a, b, c, d)
            if not pair: continue
            vx, vy = (d[0]-c[0])/dl, (d[1]-c[1])/dl
            if vx < -1e-12 or (abs(vx) <= 1e-12 and vy < 0): vx, vy = -vx, -vy
            line = tuple(round(v, 9) for v in (vx, vy, -vy*c[0]+vx*c[1], w2))
            x1, x2 = [(p[0]-a[0])*ux+(p[1]-a[1])*uy for p in (c, d)]
            lo, hi = max(0.0, min(x1, x2)), min(length, max(x1, x2))
            spans[line].append((lo, hi, max(0.0, pair[1]-(w+w2)/2)))
    gaps = []
    for intervals in spans.values():
        lo = hi = gap = None
        for left, right, g in sorted(intervals):
            if hi is None or left > hi + 1e-9:
                if hi is not None and hi-lo >= 0.8*length-1e-9: gaps.append(gap)
                lo, hi, gap = left, right, g
            else:
                hi, gap = max(hi, right), min(gap, g)
        if hi is not None and hi-lo >= 0.8*length-1e-9: gaps.append(gap)
    return min(gaps) if gaps else None


def _via_lagen(v, seg, pads):
    ""
    from shapely.geometry import Point
    from gemeinsam.fluss import padgeo, via_radius
    point = Point(v['x'], v['y'])
    local = [s for s in seg if _auf_segment((v['x'], v['y']), s['a'], s['b'])]
    layers = {s['layer'] for s in local}
    for p in pads:
        for layer, polygon in padgeo(p).items():
            if polygon.distance(point) <= via_radius(v) + 0.03: layers.add(layer)
    return local, layers


def augen(B, R, K, A, messung, G, kopplung):
    from analyse.metriken import klasse, leitung, via_lc, RHO
    from gemeinsam.fluss import NetzgraphCache
    cache = NetzgraphCache(); out = {}; gruppen = collections.defaultdict(list)
    tracks = collections.defaultdict(list); vias = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R.get('tracks', []): tracks[t['net']].append(t)
    for v in R.get('vias', []): vias[v['net']].append(v)
    for p in B['pads']: pads[p['net']].append(p)
    for n, k in sorted(K['netze'].items()):
        if not digital(k) or n not in messung: continue
        C = G['klassen'].get(klasse(k)) or {}; par = parameter(K, n, C); tr = par['tr_ns']; ui = par['ui_ns']
        if tr is None: continue
        seg = {}; hin = []
        tn = tracks[n]; vn = vias[n]; pn = pads[n]; partner = vorgabe(K, n, 'paar')
        if par['datenrate_quelle'] != 'datenrate': hin.append('Bitrate angenommen')
        if par['z_diff_quelle'] == '2*z_ohm (Paarannahme)': hin.append('Differenz-Sollimpedanz als 2*z_ohm angenommen')
        if tr >= ui: hin.append('vorgegebene Flankenzeit >= Bitfenster; horizontale Huellkurve null')
        if vorgabe(K, n, 'f_Hz') is None: hin.append('Frequenz aus Klassenstandard')
        if vorgabe(K, n, 'z_ohm') is None and not par['z_diff_ohm']: hin.append('Sollimpedanz aus Klassenstandard')
        if not A['ebenen']: hin.append('kein nachgewiesener HF-Bezug; freies Leitungsmodell')
        for ti, t in enumerate(tn):
            if t['layer'] not in A['lagen']: continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                z, tp, _, _ = leitung(A, t['layer'], w); zs = z; l = math.dist(a, b); gap = None
                if par['z_diff_ohm']:
                    h = A['lagen'][t['layer']]['h']
                    gap = _paar_gap(a, b, w, tracks.get(partner, []), t['layer'])
                    z = 2 * zs * (1 - 0.48 * math.exp(-0.96 * gap / h)) if gap is not None and h else 2 * zs
                    if gap is None: hin.append('Paarstrecke ungekuppelt/Partner fehlt')
                seg[(ti, si)] = dict(z=z, zs=zs, l=l, tpd=tp, w=w, layer=t['layer'], gap_mm=gap, a=a, b=b)
        if not seg:
            out[n] = dict(**par, vertikal_pct=0.0, horizontal_pct=0.0, status='fehlende_bahn', hinweise=['keine bewertbare Bahn'])
            continue
        g = cache.graph(tn, vn, pn); adj = g['adj']; pk = g['padk']
        drivers = [pk['%s:%s' % (p['ref'], p['num'])] for p in pn if p.get('pintype') in ('output', 'tri_state')]
        if not drivers: drivers = sorted(pk.values())[:1]; hin.append('Treiber unbekannt; erstes Pad als Quelle')
        receivers = [p for p in sorted(pk.values()) if p not in drivers]
        delays = []; lengths = []; connected = True
        for dr in sorted(drivers):
            td, ld = _wege(g, seg, A, dr)
            delays += [td[p] for p in receivers if p in td]; lengths += [ld[p] for p in receivers if p in ld]
            if any(p not in td for p in receivers): hin.append('Empfaenger nicht verbunden'); connected = False
        if not delays: hin.append('keine verbundene Quelle/Senke'); connected = False
        tau = max(delays, default=0.0); length = max(lengths, default=0.0)
        target = par['z_diff_ohm'] or par['z_ohm']; factor = min(1.0, 2 * tau / tr)
        mismatch = max(gamma(target, s['z']) for s in seg.values()) * factor
        spruenge = t_ref = t_ab = offen = 0.0; n_t = n_open = 0; stub_mm = 0.0
        for node, edges in sorted(adj.items()):
            if node[0] == 'PAD': continue
            se = [(m, ki) for m, ki in edges if ki and ki[0] == 's']

            if len(se) == 2:
                za, zb = [seg[ki[1:3]]['z'] for _, ki in se]; spruenge += gamma(za, zb) * factor
            elif len(se) >= 3:
                legs = [_zweig(adj, seg, node, m, ki) for m, ki in se]
                lt, tt = min(legs, key=lambda x: x[1]); stub_mm = max(stub_mm, lt); n_t += 1
                zs = [seg[ki[1:3]]['z'] for _, ki in se]
                ga = max(gamma(z, 1 / sum(1 / zz for j, zz in enumerate(zs) if j != i)) for i, z in enumerate(zs))
                t_ab += ga * min(1.0, 2 * tt / tr); t_ref = max(t_ref, 2 * tt)
            if len(edges) == 1 and len(se) == 1:
                lt, tt = _zweig(adj, seg, node, *se[0]); offen += min(1.0, 2 * tt / tr); n_open += 1; stub_mm = max(stub_mm, lt)

        zl = vorgabe(K, n, 'z_last_ohm'); zq = vorgabe(K, n, 'z_quelle_ohm')
        if zl is None: hin.append('Last hochohmig angenommen')
        if zq is None: hin.append('Quelle ideal niederohmig angenommen')
        gl = 1.0 if zl is None else gamma(target, max(0.0, float(zl)))
        gq = 1.0 if zq is None else gamma(target, max(0.0, float(zq)))
        endecho = gl * gq * factor
        fv = max(par['f_Hz'], 0.35e9 / tr); omega = 2 * math.pi * fv
        via_ref = 0.0
        for v in vn:
            local, layers = _via_lagen(v, seg.values(), pn)


            jumps = [gamma(a['z'],b['z']) for i,a in enumerate(local) for b in local[i+1:] if a['layer']!=b['layer']]
            spruenge += max(jumps,default=0.0)*factor
            z = [A['lagen'][L]['z'] for L in sorted(layers) if L in A['lagen']]
            height = max(z) - min(z) if len(z) >= 2 else A['dicke']
            lv, cv = via_lc(height, float(v.get('drill', 0.2)), A, float(v.get('d', 0.45)))
            xl = omega * lv * 1e-9 / (2 * par['z_ohm']); xc = omega * cv * 1e-12 * par['z_ohm'] / 2
            via_ref += xl / math.sqrt(1 + xl * xl) + xc / math.sqrt(1 + xc * xc)
            ts = max(0.0, A['dicke'] - height) * math.sqrt(A['er']) * 3.336e-12
            b = math.tan(min(math.pi / 2 - 1e-6, omega * ts)); via_ref += abs(b) / math.sqrt(4 + b * b)
        tand = zahl(vorgabe(K, n, 'tan_delta'), 0.02)
        delta = math.sqrt(2 * 1.72e-8 / (omega * 4 * math.pi * 1e-7)) * 1000

        ltotal = sum(s['l'] for s in seg.values()); alpha = 0.0
        for s in seg.values():
            t = A['lagen'][s['layer']]['t']; rac = RHO * s['l'] / (s['w'] * min(t, 2 * delta))
            alpha += rac / (2 * s['zs']) + math.pi * fv * tand * s['l'] * s['tpd'] * 1e-12
        alpha *= length / ltotal if ltotal else 0.0
        amp = math.exp(-alpha); loss = 1 - amp
        xt = kopplung.get(n, {}); nx = xt.get('next', 0.0); fx = xt.get('fext', 0.0)
        reflection = mismatch + spruenge + t_ab + offen + endecho + via_ref
        noise = reflection + nx + fx
        vert = max(0.0, min(1.0, amp - 2 * noise))
        jitter = 2 * tr * noise / max(amp, 1e-9)
        rise = tr / max(amp, 1e-9)
        out[n] = dict(**par, laufzeit_ns=tau, laufzeit_min_ns=min(delays, default=0.0), pfad_mm=length, vertikal_pct=100 * vert if connected else 0.0,
                      horizontal_pct=100 * max(0.0, 1 - (rise + jitter + tau - min(delays, default=0.0)) / ui) if connected else 0.0,
                      reflexion=reflection, sollsprung=mismatch, spruenge=spruenge, t_reflexion=t_ab, offene_reflexion=offen,
                      endecho=endecho, via_reflexion=via_ref, next=nx, fext=fx, daempfung=loss,
                      daempfung_dB=alpha * 20 / math.log(10), jitter_ns=jitter, rise_ns=rise, t_abzweige=n_t,
                      offene_enden=n_open, stub_max_mm=stub_mm, echo_ns=max(2 * tau, t_ref),
                      hinweise=sorted(set(hin)), status='modell' if connected else 'unverbunden', segmente=list(seg.values()))
        if par['gruppe']: gruppen[par['gruppe']].append(n)
    for group, nn in sorted(gruppen.items()):
        ok = [n for n in nn if 'laufzeit_ns' in out[n]]
        if not ok: continue

        spread = max(out[n]['laufzeit_ns'] for n in ok) - min(out[n]['laufzeit_min_ns'] for n in ok)
        for n in ok:
            e = out[n]; tol = e['toleranz_mm']; tpd = e['laufzeit_ns'] / e['pfad_mm'] if e['pfad_mm'] else 0.006
            e['skew_ns'] = spread; e['skew_x'] = spread / max(1e-9, tol * tpd) if tol is not None else 0.0
            e['horizontal_pct'] = max(0.0, e['horizontal_pct'] - 100 * max(0.0, spread - e['laufzeit_ns'] + e['laufzeit_min_ns']) / e['ui_ns'])
    schnitt = collections.defaultdict(list)
    for n, e in out.items(): schnitt[e['schnittstelle']].append(n)
    interfaces = {s: dict(netze=nn, vertikal_pct=min(out[n]['vertikal_pct'] for n in nn),
                         horizontal_pct=min(out[n]['horizontal_pct'] for n in nn)) for s, nn in sorted(schnitt.items())}
    return out, interfaces


def gemeinsame_masse(B, K, A, analog):
    ""
    from analyse.metriken import RHO
    if not A['ebenen']: return {}, 'keine GND-Flaeche; gemeinsame Impedanz unbekannt'
    gnd = set(K.get('gnd') or []); teile = K.get('teile') or {}; gn = collections.defaultdict(list)
    for p in B['pads']:
        if p['net'] in gnd: gn[p['ref']].append((p['x'], p['y']))
    sources = set()
    for n, k in K['netze'].items():
        fl = k.get('fluss') or {}
        if fl.get('von') or k.get('art') not in ('strom', 'versorgung'): continue
        for key in fl.get('quelle') or []:
            ref = key.split(':')[0]
            if (teile.get(ref) or {}).get('rolle') == 'stecker' and gn.get(ref): sources.add(ref)
    sinks = [pt for ref in sorted(sources) for pt in sorted(gn[ref])]
    currents = []
    for ref, t in sorted(teile.items()):
        i = zahl(t.get('strom_A'), 0.0)
        if not i or not gn.get(ref) or ref in sources or t.get('rolle') != 'chip': continue
        if 'durchfluss' in str(t.get('strom_A_quelle') or ''): continue
        currents.append((ref, i, gn[ref]))
    if not sinks or not currents: return {}, 'Einspeise-GND oder Chipstroeme fehlen; gemeinsame Impedanz unbekannt'
    rq = RHO / sum(A['lagen'][L]['t'] for L in A['ebenen'])
    def potential(points, ipts, current):
        vals = []
        for p in points:
            u = -sum(math.log(max(0.2, math.dist(p, q))) for q in ipts) / len(ipts)
            u += sum(math.log(max(0.2, math.dist(p, q))) for q in sinks) / len(sinks)
            vals.append(1000 * rq * current * u / (2 * math.pi))
        return sum(vals) / len(vals)
    out = {}
    for n in sorted(analog):
        refs = sorted({p['ref'] for p in B['pads'] if p['net'] == n and gn.get(p['ref'])})
        if len(refs) < 2: continue
        contributions = []
        for ref, current, ipts in currents:
            values = [potential(gn[r], ipts, current) for r in refs]
            contributions.append(dict(stoerer=ref, mV=max(values) - min(values), I_A=current))
        out[n] = dict(mV=sum(c['mV'] for c in contributions), bezuege=refs, beitraege=contributions,
                      quelle='unendliche GND-Platte aus Chipstroemen; keine AC-Masseimpedanz')
    return out, 'Plattenmodell; Slots/Zonenrand und AC-Masseimpedanz unbekannt'


def stoerungen(B, R, K, A, M, tr_ns):
    ""
    import shapely
    from analyse.kopplung import _paar, _paarnetz
    from analyse.metriken import leitung
    N = K['netze']; seg = []
    analog = {n for n, k in N.items() if k.get('art') in ('analog', 'kelvin') or k.get('kelvin')}
    aggress = {n for n, k in N.items() if (digital(k) or k.get('art') == 'sw')
               and tr_ns.get(n, 5.0) is not None}
    for t in R.get('tracks', []):
        if t['net'] not in analog | aggress or t['layer'] not in A['lagen']: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-9: seg.append((t['net'], t['layer'], a, b, w))
    acc = collections.defaultdict(lambda: dict(C_pF=0.0, L_nH=0.0, next_l=0.0, kc_l=0.0, kl_l=0.0, lp=0.0, tau_ns=0.0))
    if seg:
        geo = shapely.linestrings([[s[2], s[3]] for s in seg]); ii, jj = shapely.STRtree(geo).query(geo, predicate='dwithin', distance=10.0)
        for i, j in sorted(zip(ii.tolist(), jj.tolist())):
            if j <= i: continue
            n, L, a, b, w = seg[i]; q, Q, c, d, w2 = seg[j]
            if n == q or _paarnetz(n, q, N): continue
            g, h = A['lagen'][L], A['lagen'][Q]
            if L != Q and abs(g['i'] - h['i']) != 1: continue
            p = _paar(a, b, c, d)
            if p is None: continue
            lp, side, _ = p; H2 = (g['H'] or 1.5) * (h['H'] or 1.5)
            kn = 1 / (1 + (side * side + (g['z'] - h['z']) ** 2) / H2)
            ratio = 1.0 if g['typ'] == h['typ'] == 'strip2' else 0.7
            kc = min(1.0, 4 * kn / (1 + ratio)); kl = ratio * kc
            _, tp, l1, c1 = leitung(A, L, w); _, tq, l2, c2 = leitung(A, Q, w2)
            tau = max(tp, tq) * lp / 1000
            for victim, ag in ((n, q), (q, n)):
                if ag not in aggress: continue
                tr = zahl(tr_ns.get(ag), 5.0, 1e-6); v = acc[(victim, ag)]
                v['C_pF'] += kc * math.sqrt(c1 * c2) * lp; v['L_nH'] += kl * math.sqrt(l1 * l2) * lp
                v['next_l'] += kn * lp
                v['kc_l'] += kc * lp; v['kl_l'] += kl * lp; v['lp'] += lp; v['tau_ns'] += tau
    xt = collections.defaultdict(lambda: dict(next=0.0, fext=0.0)); out = {}; paare = []
    for (victim, ag), v in sorted(acc.items()):
        tr = zahl(tr_ns.get(ag), 5.0, 1e-6)
        v['next'] = v['next_l'] / v['lp'] * min(1.0, 2 * v['tau_ns'] / tr)
        v['fext'] = 0.5 * abs(v['kc_l'] - v['kl_l']) / v['lp'] * v['tau_ns'] / tr
        xt[victim]['next'] += v['next']; xt[victim]['fext'] += v['fext']
        if victim not in analog: continue
        swing = zahl(vorgabe(K, ag, 'hub_V'), 12.0 if N[ag].get('art') == 'sw' else 3.3)
        tr = zahl(tr_ns.get(ag), 5.0, 1e-6)
        rz = zahl(vorgabe(K, victim, 'eingang_ohm'), 1000.0)
        cin = zahl(vorgabe(K, victim, 'eingang_pF'), 5.0) + (M.get(victim, {}).get('m') or {}).get('C_pF', 0.0)

        uc = swing * min(1.0, v['C_pF'] / max(v['C_pF'] + cin, 1e-9), rz * v['C_pF'] * 1e-3 / tr)
        di = zahl(vorgabe(K, ag, 'hub_A'), swing / zahl(vorgabe(K, ag, 'z_ohm'), 50.0, 1.0))
        ul = v['L_nH'] * di / tr
        bw = zahl(vorgabe(K, victim, 'bandbreite_Hz'), None, 1.0)
        filt = 1 / math.sqrt(1 + (0.35e9 / tr / bw) ** 2) if bw else 1.0
        paare.append(dict(opfer=victim, stoerer=ag, kapazitiv_mV=1000 * uc * filt, induktiv_mV=1000 * ul * filt, **v))
    ground, ground_hint = gemeinsame_masse(B, K, A, analog)
    for n in sorted(analog & set(M)):
        pp = [p for p in paare if p['opfer'] == n]; cm = sum(p['kapazitiv_mV'] for p in pp); lm = sum(p['induktiv_mV'] for p in pp)
        common = max(abs(M[n]['m'].get('masse_uV') or 0.0) / 1000, (ground.get(n) or {}).get('mV', 0.0))
        r = zahl(vorgabe(K, n, 'r_gemeinsam_mOhm'), 0.0); l = zahl(vorgabe(K, n, 'l_gemeinsam_nH'), 0.0)
        for ag in sorted(aggress):
            tr = zahl(tr_ns.get(ag), 5.0, 1e-6); di = zahl(vorgabe(K, ag, 'hub_A'), 0.0)
            common += r * di + 1000 * l * di / tr
        budget = zahl(vorgabe(K, n, 'stoerbudget_mV'), 1.0, 1e-9)
        hin = [ground_hint, 'Eingangsimpedanz/Kapazitaet und Hub ohne Vorgabe geschaetzt', 'gleichzeitige Stoerer addiert; 10-mm-Suchradius']
        if not common and not r and not l: hin.append('gemeinsamer Massepfad nicht nachgewiesen')
        if not A['ebenen']: hin.append('freies Kopplungsmodell H=1,5 mm')
        out[n] = dict(stoer_mV=cm + lm + common, kapazitiv_mV=cm, induktiv_mV=lm, masse_mV=common,
                      budget_mV=budget, x=(cm + lm + common) / budget, status='modell', hinweise=hin, masse_modell=ground.get(n))
    return dict(xt), out, paare


def kurz(M):
    ""
    eyes = list((M.get('augen') or {}).values()); noise = list((M.get('analog_stoer') or {}).values())
    return dict(augen_vertikal_pct=min((e['vertikal_pct'] for e in eyes), default=None),
                augen_horizontal_pct=min((e['horizontal_pct'] for e in eyes), default=None),
                analog_stoer_mV=max((e['stoer_mV'] for e in noise), default=None),
                si_status='Modellschaetzung', si_version=M.get('version'), verlust_mW=M.get('verlust_mW'))
