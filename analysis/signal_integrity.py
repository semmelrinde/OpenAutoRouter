import collections
import heapq
import math
import re


def spec(K, n, fld, standard=None):
    cons = K.get('specs') or {}
    tab = cons.get(fld)
    if isinstance(tab, dict):
        for rx, v in sorted(tab.items()):
            try:
                if re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): return v
            except re.error: continue
    elif fld in cons: return tab
    for rx, k in (cons.get('net_list') or {}).items():
        try:
            if re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]):
                if fld in k: return k[fld]
                break
        except re.error: continue
    return (K['net_list'].get(n) or {}).get(fld, standard)


def count(v, default, minimum=0.0):
    if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= minimum: return float(v)
    return default


def gamma(z1, z2):
    return abs((z2 - z1) / (z2 + z1))


def _on_segment(p, a, b, tol=0.03):
    dx, dy = b[0]-a[0], b[1]-a[1]; ll=dx*dx+dy*dy
    u = max(0.0,min(1.0,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/ll)) if ll else 0.0
    return math.dist(p,(a[0]+u*dx,a[1]+u*dy))<=tol


def digital(k):
    return k.get('category') == 'fast' or (k.get('category') == 'signal' and k.get('rank', 0) >= 3)


def parameter(K, n, C):
    f = count(spec(K, n, 'f_Hz'), C.get('f_Hz', 50e6), 1.0)
    rate = count(spec(K, n, 'data_rate'), None, 1.0)
    q = 'data_rate'
    if rate is None: rate = 2 * f; q = '2*f_Hz (assumption)'
    raw_tr = spec(K, n, 'tr_ns', C.get('tr_ns', 2.0))
    tr = None if raw_tr is None else count(raw_tr, C.get('tr_ns') or 2.0, 1e-6)
    zd = count(spec(K, n, 'z_diff_ohm'), None, 1.0)
    zs = count(spec(K, n, 'z_ohm'), C.get('z_ohm', 50.0), 1.0)
    zq = 'Spec/NetClass' if zd is not None else None
    partner = spec(K, n, 'pair')
    if zd is None and partner:
        zd = count(C.get('z_diff_ohm'), 2 * zs, 1.0)
        zq = 'ClassDefault' if C.get('z_diff_ohm') else '2*z_ohm (pair assumption)'
    group = spec(K, n, 'length_group', C.get('length_group'))


    if not group and partner: group = 'pair:' + '|'.join(sorted((n, partner)))
    interface = spec(K, n, 'interface', C.get('interface')) or group
    if not interface: interface = 'pair:' + '|'.join(sorted((n, partner))) if partner else n
    return dict(f_Hz=f, data_rate=rate, data_rate_source=q, ui_ns=1e9 / rate, tr_ns=tr, z_ohm=zs,
                z_diff_ohm=zd, z_diff_source=zq, group=group,
                tolerance_mm=count(spec(K, n, 'tolerance_mm'), C.get('tolerance_mm')),
                interface=interface)


def _paths(g, seg, A, start):
    adj = g['adj']; best = {start: 0.0}; dist = {start: 0.0}; h = [(0.0, start)]
    while h:
        d, k = heapq.heappop(h)
        if d > best[k]: continue
        for m, kid in adj[k]:
            t = l = 0.0
            if kid and kid[0] == 's':
                s = seg[kid[1:3]]; l = s['l'] * (kid[4] - kid[3]); t = s['tpd'] * l / 1000
            elif kid and kid[0] == 'v':
                l = abs(A['cu_layers'][k[0]]['z'] - A['cu_layers'][m[0]]['z']); t = l * math.sqrt(A['er']) * 3.336 / 1000
            if d + t < best.get(m, math.inf):
                best[m] = d + t; dist[m] = dist[k] + l; heapq.heappush(h, (d + t, m))
    return best, dist


def _branch(adj, seg, k, nxt, kid):
    t = l = 0.0; seen = {k}
    while nxt not in seen:
        seen.add(nxt)
        if kid and kid[0] == 's':
            s = seg[kid[1:3]]; dl = s['l'] * (kid[4] - kid[3]); l += dl; t += s['tpd'] * dl / 1000
        rest = [(m, ai) for m, ai in adj[nxt] if m != k]
        if nxt[0] == 'PAD' or len(rest) != 1: break
        k, (nxt, kid) = nxt, rest[0]
    return l, t


def _pair_gap(a, b, w, partner, layer):
    from analysis.coupling import _pair
    length = math.dist(a, b)
    if length < 1e-9: return None
    ux, uy = (b[0]-a[0])/length, (b[1]-a[1])/length
    spans = collections.defaultdict(list)
    for t in partner:
        if t['layer'] != layer: continue
        for c, d, w2 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            dl = math.dist(c, d)
            if dl < 1e-9: continue
            pair = _pair(a, b, c, d)
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


def _via_layers(v, seg, pads):
    from shapely.geometry import Point
    from common.flow import pad_geo, via_radius
    point = Point(v['x'], v['y'])
    local = [s for s in seg if _on_segment((v['x'], v['y']), s['a'], s['b'])]
    layers = {s['layer'] for s in local}
    for p in pads:
        for layer, polygon in pad_geo(p).items():
            if polygon.distance(point) <= via_radius(v) + 0.03: layers.add(layer)
    return local, layers


def eyes(B, R, K, A, measurement, G, coupling):
    from analysis.measures import net_class, line, via_lc, RHO
    from common.flow import NetGraphCache
    cache = NetGraphCache(); out = {}; group_list = collections.defaultdict(list)
    tracks = collections.defaultdict(list); vias = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R.get('tracks', []): tracks[t['net']].append(t)
    for v in R.get('vias', []): vias[v['net']].append(v)
    for p in B['pads']: pads[p['net']].append(p)
    for n, k in sorted(K['net_list'].items()):
        if not digital(k) or n not in measurement: continue
        C = G['netclasses'].get(net_class(k)) or {}; par = parameter(K, n, C); tr = par['tr_ns']; ui = par['ui_ns']
        if tr is None: continue
        seg = {}; fwd = []
        tn = tracks[n]; vn = vias[n]; pn = pads[n]; partner = spec(K, n, 'pair')
        if par['data_rate_source'] != 'data_rate': fwd.append('Bit rate assumed')
        if par['z_diff_source'] == '2*z_ohm (pair assumption)': fwd.append('Differential target impedance assumed as 2*z_ohm')
        if tr >= ui: fwd.append('given edge time >= bit window; horizontal envelope zero')
        if spec(K, n, 'f_Hz') is None: fwd.append('Frequency from netclass default')
        if spec(K, n, 'z_ohm') is None and not par['z_diff_ohm']: fwd.append('Target impedance from netclass default')
        if not A['levels']: fwd.append('no demonstrated RF reference; free line model')
        for ti, t in enumerate(tn):
            if t['layer'] not in A['cu_layers']: continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                z, tp, _, _ = line(A, t['layer'], w); zs = z; l = math.dist(a, b); gap = None
                if par['z_diff_ohm']:
                    h = A['cu_layers'][t['layer']]['h']
                    gap = _pair_gap(a, b, w, tracks.get(partner, []), t['layer'])
                    z = 2 * zs * (1 - 0.48 * math.exp(-0.96 * gap / h)) if gap is not None and h else 2 * zs
                    if gap is None: fwd.append('Pair section uncoupled/partner missing')
                seg[(ti, si)] = dict(z=z, zs=zs, l=l, tpd=tp, w=w, layer=t['layer'], gap_mm=gap, a=a, b=b)
        if not seg:
            out[n] = dict(**par, vertical_pct=0.0, horizontal_pct=0.0, status='missing_track', hints=['no evaluable track'])
            continue
        g = cache.graph(tn, vn, pn); adj = g['adj']; pk = g['pad_pts']
        drivers = [pk['%s:%s' % (p['ref'], p['num'])] for p in pn if p.get('pintype') in ('output', 'tri_state')]
        if not drivers: drivers = sorted(pk.values())[:1]; fwd.append('Driver unknown; first pad as source')
        receivers = [p for p in sorted(pk.values()) if p not in drivers]
        delays = []; lengths = []; connected = True
        for dr in sorted(drivers):
            td, ld = _paths(g, seg, A, dr)
            delays += [td[p] for p in receivers if p in td]; lengths += [ld[p] for p in receivers if p in ld]
            if any(p not in td for p in receivers): fwd.append('Receiver not connected'); connected = False
        if not delays: fwd.append('no connected source/sink'); connected = False
        tau = max(delays, default=0.0); length = max(lengths, default=0.0)
        target = par['z_diff_ohm'] or par['z_ohm']; factor = min(1.0, 2 * tau / tr)
        mismatch = max(gamma(target, s['z']) for s in seg.values()) * factor
        leaps = t_ref = t_from = open_items = 0.0; n_t = n_open = 0; stub_mm = 0.0
        for node, edges in sorted(adj.items()):
            if node[0] == 'PAD': continue
            se = [(m, ai) for m, ai in edges if ai and ai[0] == 's']

            if len(se) == 2:
                za, zb = [seg[ai[1:3]]['z'] for _, ai in se]; leaps += gamma(za, zb) * factor
            elif len(se) >= 3:
                legs = [_branch(adj, seg, node, m, ai) for m, ai in se]
                lt, tt = min(legs, key=lambda x: x[1]); stub_mm = max(stub_mm, lt); n_t += 1
                zs = [seg[ai[1:3]]['z'] for _, ai in se]
                ga = max(gamma(z, 1 / sum(1 / zz for j, zz in enumerate(zs) if j != i)) for i, z in enumerate(zs))
                t_from += ga * min(1.0, 2 * tt / tr); t_ref = max(t_ref, 2 * tt)
            if len(edges) == 1 and len(se) == 1:
                lt, tt = _branch(adj, seg, node, *se[0]); open_items += min(1.0, 2 * tt / tr); n_open += 1; stub_mm = max(stub_mm, lt)

        zl = spec(K, n, 'z_load_ohm'); zq = spec(K, n, 'z_source_ohm')
        if zl is None: fwd.append('Load assumed high-impedance')
        if zq is None: fwd.append('Source assumed ideal low-impedance')
        gl = 1.0 if zl is None else gamma(target, max(0.0, float(zl)))
        gq = 1.0 if zq is None else gamma(target, max(0.0, float(zq)))
        endecho = gl * gq * factor
        fv = max(par['f_Hz'], 0.35e9 / tr); omega = 2 * math.pi * fv
        via_ref = 0.0
        for v in vn:
            local, layers = _via_layers(v, seg.values(), pn)


            jumps = [gamma(a['z'],b['z']) for i,a in enumerate(local) for b in local[i+1:] if a['layer']!=b['layer']]
            leaps += max(jumps,default=0.0)*factor
            z = [A['cu_layers'][L]['z'] for L in sorted(layers) if L in A['cu_layers']]
            height = max(z) - min(z) if len(z) >= 2 else A['thickness']
            lv, cv = via_lc(height, float(v.get('drill', 0.2)), A, float(v.get('d', 0.45)))
            xl = omega * lv * 1e-9 / (2 * par['z_ohm']); xc = omega * cv * 1e-12 * par['z_ohm'] / 2
            via_ref += xl / math.sqrt(1 + xl * xl) + xc / math.sqrt(1 + xc * xc)
            ts = max(0.0, A['thickness'] - height) * math.sqrt(A['er']) * 3.336e-12
            b = math.tan(min(math.pi / 2 - 1e-6, omega * ts)); via_ref += abs(b) / math.sqrt(4 + b * b)
        tand = count(spec(K, n, 'tan_delta'), 0.02)
        delta = math.sqrt(2 * 1.72e-8 / (omega * 4 * math.pi * 1e-7)) * 1000

        ltotal = sum(s['l'] for s in seg.values()); alpha = 0.0
        for s in seg.values():
            t = A['cu_layers'][s['layer']]['t']; rac = RHO * s['l'] / (s['w'] * min(t, 2 * delta))
            alpha += rac / (2 * s['zs']) + math.pi * fv * tand * s['l'] * s['tpd'] * 1e-12
        alpha *= length / ltotal if ltotal else 0.0
        amp = math.exp(-alpha); loss = 1 - amp
        xt = coupling.get(n, {}); nx = xt.get('next', 0.0); fx = xt.get('fext', 0.0)
        reflection = mismatch + leaps + t_from + open_items + endecho + via_ref
        noise = reflection + nx + fx
        vert = max(0.0, min(1.0, amp - 2 * noise))
        jitter = 2 * tr * noise / max(amp, 1e-9)
        rise = tr / max(amp, 1e-9)
        out[n] = dict(**par, run_duration_ns=tau, run_duration_min_ns=min(delays, default=0.0), path_mm=length, vertical_pct=100 * vert if connected else 0.0,
                      horizontal_pct=100 * max(0.0, 1 - (rise + jitter + tau - min(delays, default=0.0)) / ui) if connected else 0.0,
                      reflection=reflection, target_jump=mismatch, leaps=leaps, t_reflection=t_from, open_reflection=open_items,
                      endecho=endecho, via_reflection=via_ref, next=nx, fext=fx, damping=loss,
                      damping_dB=alpha * 20 / math.log(10), jitter_ns=jitter, rise_ns=rise, t_branches=n_t,
                      open_ends=n_open, stub_max_mm=stub_mm, echo_ns=max(2 * tau, t_ref),
                      hints=sorted(set(fwd)), status='model' if connected else 'not_connected', segments=list(seg.values()))
        if par['group']: group_list[par['group']].append(n)
    for group, nn in sorted(group_list.items()):
        ok = [n for n in nn if 'run_duration_ns' in out[n]]
        if not ok: continue

        spread = max(out[n]['run_duration_ns'] for n in ok) - min(out[n]['run_duration_min_ns'] for n in ok)
        for n in ok:
            e = out[n]; tol = e['tolerance_mm']; tpd = e['run_duration_ns'] / e['path_mm'] if e['path_mm'] else 0.006
            e['skew_ns'] = spread; e['skew_x'] = spread / max(1e-9, tol * tpd) if tol is not None else 0.0
            e['horizontal_pct'] = max(0.0, e['horizontal_pct'] - 100 * max(0.0, spread - e['run_duration_ns'] + e['run_duration_min_ns']) / e['ui_ns'])
    cut = collections.defaultdict(list)
    for n, e in out.items(): cut[e['interface']].append(n)
    interfaces = {s: dict(net_list=nn, vertical_pct=min(out[n]['vertical_pct'] for n in nn),
                         horizontal_pct=min(out[n]['horizontal_pct'] for n in nn)) for s, nn in sorted(cut.items())}
    return out, interfaces


def common_ground(B, K, A, analog):
    from analysis.measures import RHO
    if not A['levels']: return {}, 'no GND area; common impedance unknown'
    gnd = set(K.get('gnd') or []); part_list = K.get('part_list') or {}; gn = collections.defaultdict(list)
    for p in B['pads']:
        if p['net'] in gnd: gn[p['ref']].append((p['x'], p['y']))
    sources = set()
    for n, k in K['net_list'].items():
        fl = k.get('flow') or {}
        if fl.get('from_') or k.get('category') not in ('current_val', 'supply'): continue
        for key in fl.get('source') or []:
            ref = key.split(':')[0]
            if (part_list.get(ref) or {}).get('role') == 'connector' and gn.get(ref): sources.add(ref)
    sinks = [pt for ref in sorted(sources) for pt in sorted(gn[ref])]
    currents = []
    for ref, t in sorted(part_list.items()):
        i = count(t.get('current_A'), 0.0)
        if not i or not gn.get(ref) or ref in sources or t.get('role') != 'chip': continue
        if 'throughflow' in str(t.get('current_A_source') or ''): continue
        currents.append((ref, i, gn[ref]))
    if not sinks or not currents: return {}, 'Feed GND or chip currents missing; common impedance unknown'
    rq = RHO / sum(A['cu_layers'][L]['t'] for L in A['levels'])
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
            contributions.append(dict(aggressor=ref, mV=max(values) - min(values), I_A=current))
        out[n] = dict(mV=sum(c['mV'] for c in contributions), references=refs, contributions=contributions,
                      source='infinite GND plate from chip currents; no AC ground impedance')
    return out, 'Plate model; slots/zone edge and AC ground impedance unknown'


def disturbances(B, R, K, A, M, tr_ns):
    import shapely
    from analysis.coupling import _pair, _pair_net
    from analysis.measures import line
    N = K['net_list']; seg = []
    analog = {n for n, k in N.items() if k.get('category') in ('analog', 'kelvin') or k.get('kelvin')}
    aggress = {n for n, k in N.items() if (digital(k) or k.get('category') == 'sw')
               and tr_ns.get(n, 5.0) is not None}
    for t in R.get('tracks', []):
        if t['net'] not in analog | aggress or t['layer'] not in A['cu_layers']: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-9: seg.append((t['net'], t['layer'], a, b, w))
    acc = collections.defaultdict(lambda: dict(C_pF=0.0, L_nH=0.0, next_l=0.0, kc_l=0.0, nc_l=0.0, lp=0.0, tau_ns=0.0))
    if seg:
        geo = shapely.linestrings([[s[2], s[3]] for s in seg]); ii, jj = shapely.STRtree(geo).query(geo, predicate='dwithin', distance=10.0)
        for i, j in sorted(zip(ii.tolist(), jj.tolist())):
            if j <= i: continue
            n, L, a, b, w = seg[i]; q, Q, c, d, w2 = seg[j]
            if n == q or _pair_net(n, q, N): continue
            g, h = A['cu_layers'][L], A['cu_layers'][Q]
            if L != Q and abs(g['i'] - h['i']) != 1: continue
            p = _pair(a, b, c, d)
            if p is None: continue
            lp, side, _ = p; H2 = (g['H'] or 1.5) * (h['H'] or 1.5)
            kn = 1 / (1 + (side * side + (g['z'] - h['z']) ** 2) / H2)
            ratio = 1.0 if g['type_'] == h['type_'] == 'strip2' else 0.7
            kc = min(1.0, 4 * kn / (1 + ratio)); nc = ratio * kc
            _, tp, l1, c1 = line(A, L, w); _, tq, l2, c2 = line(A, Q, w2)
            tau = max(tp, tq) * lp / 1000
            for victim, ag in ((n, q), (q, n)):
                if ag not in aggress: continue
                tr = count(tr_ns.get(ag), 5.0, 1e-6); v = acc[(victim, ag)]
                v['C_pF'] += kc * math.sqrt(c1 * c2) * lp; v['L_nH'] += nc * math.sqrt(l1 * l2) * lp
                v['next_l'] += kn * lp
                v['kc_l'] += kc * lp; v['nc_l'] += nc * lp; v['lp'] += lp; v['tau_ns'] += tau
    xt = collections.defaultdict(lambda: dict(next=0.0, fext=0.0)); out = {}; pairs = []
    for (victim, ag), v in sorted(acc.items()):
        tr = count(tr_ns.get(ag), 5.0, 1e-6)
        v['next'] = v['next_l'] / v['lp'] * min(1.0, 2 * v['tau_ns'] / tr)
        v['fext'] = 0.5 * abs(v['kc_l'] - v['nc_l']) / v['lp'] * v['tau_ns'] / tr
        xt[victim]['next'] += v['next']; xt[victim]['fext'] += v['fext']
        if victim not in analog: continue
        swing = count(spec(K, ag, 'hub_V'), 12.0 if N[ag].get('category') == 'sw' else 3.3)
        tr = count(tr_ns.get(ag), 5.0, 1e-6)
        rz = count(spec(K, victim, 'input_ohm'), 1000.0)
        cin = count(spec(K, victim, 'input_pF'), 5.0) + (M.get(victim, {}).get('m') or {}).get('C_pF', 0.0)

        uc = swing * min(1.0, v['C_pF'] / max(v['C_pF'] + cin, 1e-9), rz * v['C_pF'] * 1e-3 / tr)
        di = count(spec(K, ag, 'hub_A'), swing / count(spec(K, ag, 'z_ohm'), 50.0, 1.0))
        ul = v['L_nH'] * di / tr
        bw = count(spec(K, victim, 'bandwidth_Hz'), None, 1.0)
        filt = 1 / math.sqrt(1 + (0.35e9 / tr / bw) ** 2) if bw else 1.0
        pairs.append(dict(victim=victim, aggressor=ag, capacitive_mV=1000 * uc * filt, inductive_mV=1000 * ul * filt, **v))
    ground, ground_hint = common_ground(B, K, A, analog)
    for n in sorted(analog & set(M)):
        pp = [p for p in pairs if p['victim'] == n]; cm = sum(p['capacitive_mV'] for p in pp); lm = sum(p['inductive_mV'] for p in pp)
        common = max(abs(M[n]['m'].get('ground_uV') or 0.0) / 1000, (ground.get(n) or {}).get('mV', 0.0))
        r = count(spec(K, n, 'r_common_mOhm'), 0.0); l = count(spec(K, n, 'l_common_nH'), 0.0)
        for ag in sorted(aggress):
            tr = count(tr_ns.get(ag), 5.0, 1e-6); di = count(spec(K, ag, 'hub_A'), 0.0)
            common += r * di + 1000 * l * di / tr
        budget = count(spec(K, n, 'noise_budget_mV'), 1.0, 1e-9)
        fwd = [ground_hint, 'Input impedance/capacitance and swing estimated without constraint', 'simultaneous aggressors added; 10 mm search radius']
        if not common and not r and not l: fwd.append('common ground path not demonstrated')
        if not A['levels']: fwd.append('free coupling model H=1.5 mm')
        out[n] = dict(disturb_mV=cm + lm + common, capacitive_mV=cm, inductive_mV=lm, ground_mV=common,
                      budget_mV=budget, x=(cm + lm + common) / budget, status='model', hints=fwd, ground_model=ground.get(n))
    return dict(xt), out, pairs


def short(M):
    eyes = list((M.get('eyes') or {}).values()); noise = list((M.get('analog_disturb') or {}).values())
    return dict(eyes_vertical_pct=min((e['vertical_pct'] for e in eyes), default=None),
                eyes_horizontal_pct=min((e['horizontal_pct'] for e in eyes), default=None),
                analog_disturb_mV=max((e['disturb_mV'] for e in noise), default=None),
                si_status='ModelEstimate', si_version=M.get('version'), loss_mW=M.get('loss_mW'))
