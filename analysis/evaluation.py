import json, sys, math, collections, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.flow import analysis as flow_analysis, narrowing as flow_narrowing, goal_width as flow_goal_width, pad_geo, via_radius, NetGraphCache
from shapely.geometry import Point

WT = {'sw': 4.0, 'current_val': 2.5, 'supply': 1.0, 'fast': 3.0, 'analog': 2.0, 'signal': 1.0}


def mst(pts):
    if len(pts) < 2: return 0.0
    inT = [pts[0]]; rest = list(pts[1:]); L = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L


def gnd_areas(R, GND, K=None):
    from analysis.areas import active
    if not active(R, K or {}):
        covered = set(R.get('area_nets') or []) & GND
        raw = (R.get('options') or {}).get('gnd_layers') or []
        labels = (raw.split(',') if isinstance(raw, str) else list(raw))
        gl = R.get('gnd_layers') or []
        labels += gl.split(',') if isinstance(gl, str) else list(gl)
        for label in labels:
            if not label: continue
            if '=' in str(label):
                for name in str(label).split('=', 1)[1].split('|'):
                    covered.update(n for n in GND if n == name or n == '/' + name)
            elif len(GND) == 1: covered.update(GND)
            else: covered.update(n for n in GND if n.split('/')[-1] == 'GND')
        return covered
    from analysis.areas import assignment
    raw = (R.get('options') or {}).get('gnd_layers')
    if raw is None: raw = R.get('gnd_layers') or []
    labels = raw.split(',') if isinstance(raw, str) else list(raw)
    layer_names = [str(x).split('=', 1)[0] for x in labels]
    L = max([int(x[2:]) + 2 for x in layer_names if x.startswith('In') and x[2:].isdigit()] + [2])
    B = dict(cu_layers=L, pads=[dict(net=n) for n in sorted(GND)])
    mapping = assignment(B, R, dict(gnd=sorted(GND)))
    covered = ({n for nn in mapping.values() for n in nn} if labels else set(R.get('area_nets') or [])) & GND
    return covered


def copper_fingerprint(R):
    import hashlib
    return hashlib.sha256(json.dumps({k: R.get(k) for k in ('tracks', 'vias', 'conflicts', 'failed')}, sort_keys=True).encode()).hexdigest()


def native_areas(R, K):
    proof = R.get('native_areas')
    d = R.get('drc')
    if not isinstance(proof, dict) or proof.get('version') != 'native-areas-v1': return set()
    if not isinstance(d, dict) or d.get('version') != 'corpus-drc-v1' or 'faults' in d: return set()
    keys = ('all_items', 'slot', 'routing', 'not_connected', 'zones_copper')
    if not all(type(d.get(k)) is int and d[k] >= 0 for k in keys): return set()
    if any(d[k] for k in ('routing', 'not_connected', 'zones_copper')): return set()
    if d.get('routes_sha256') != copper_fingerprint(R): return set()
    if not isinstance(proof.get('net_list'), list): return set()
    return {n for n in proof['net_list'] if (K.get('net_list', {}).get(n) or {}).get('category') in ('gnd', 'current_val', 'supply')}


def not_connected(B, R, GND, flow_cache=None, K=None):
    from common.zones import unify
    R = unify(R)
    from analysis.areas import active, copper_map, assignment, joined
    KK = K or dict(gnd=sorted(GND), cu_layers=B.get('cu_layers') or int((R.get('options') or {}).get('cu_layers', 2)))
    FR = copper_map(B, R, KK) if active(R, KK) else R
    fn = ({n for nn in assignment(B, R, KK).values() for n in nn} |
          {z['net'] for z in FR.get('zones', [])}) if active(R, KK) else set()
    if any(z.get('filled') for z in FR.get('zones', [])):
        from analysis.copper_model import unconnected_polygons
        return unconnected_polygons(B, FR, set(GND) - fn)
    pads = collections.defaultdict(list); tr = collections.defaultdict(list); vi = collections.defaultdict(list)
    for p in B['pads']:
        if p['net'] and (p['net'] not in GND or p['net'] in fn): pads[p['net']].append(p)
    for t in R.get('tracks', []): tr[t['net']].append(t)
    for v in R.get('vias', []): vi[v['net']].append(v)
    out = []
    for n, P in sorted(pads.items()):
        keys = sorted({'%s:%s' % (p['ref'], p['num']) for p in P})
        if len(keys) < 2: continue
        if n in fn:
            if not joined(B, FR, KK, n): out.append(n)
            continue
        if tr[n]:
            info = flow_analysis(tr[n], vi[n], P, {'source': [keys[0]], 'load': {}},
                                 graph=flow_cache.graph(tr[n], vi[n], P, endpoints=False) if flow_cache is not None else None)
            if info is None or info['pads_reached'] < info['pads']: out.append(n)
            continue

        part = {k: k for k in keys}
        def root(k):
            while part[k] != k: k = part[k]
            return k
        G = {id(p): pad_geo(p) for p in P}
        for v in vi[n]:
            ks = sorted({'%s:%s' % (p['ref'], p['num']) for p in P
                         if min(G[id(p)][L].distance(Point(v['x'], v['y'])) for L in p['layers']) <= via_radius(v) + 0.03})
            for k in ks[1:]: part[root(k)] = root(ks[0])
        for i, p in enumerate(P):
            for q in P[i + 1:]:
                gem = set(p['layers']) & set(q['layers'])
                if any(G[id(p)][L].distance(G[id(q)][L]) <= 1e-6 for L in gem):
                    part[root('%s:%s' % (p['ref'], p['num']))] = root('%s:%s' % (q['ref'], q['num']))
        if len({root(k) for k in keys}) > 1: out.append(n)
    return out


SEPARATED = 'Consumers separated from source: '


def ir_block(B, R, K, zone_list=None):
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'codex', 'ir_drop'))
        from solver import ir_metrics
        def solve(KK):
            if zone_list is None: return ir_metrics(B, R, KK)
            from codex.zones_ir.integration import zones_ir_metrics
            return zones_ir_metrics(B, R, KK, options=zone_list)
        I = solve(K); without = {}
        for n, v in sorted(I['net_list'].items()):
            f = str(v.get('error') or '')
            if v.get('status') == 'error' and f.startswith(SEPARATED): without[n] = sorted(x.strip() for x in f[len(SEPARATED):].split(','))
        if without:
            import copy
            K2 = dict(K); K2['net_list'] = dict(K['net_list'])
            for n, way in without.items():
                k = K2['net_list'][n] = copy.deepcopy(K['net_list'][n]); fl = k.get('flow') or {}
                for s in way:
                    (fl.get('load') or {}).pop(s, None); (fl.get('provenance') or {}).pop(s, None)
            I2 = solve(K2)
            for n in without:
                if (I2['net_list'].get(n) or {}).get('status') == 'solved': I['net_list'][n] = I2['net_list'][n]
            rest = sum(1 for v in I['net_list'].values() if v.get('status') == 'error')
            I['count_error'] = rest
            if not rest: I['status'] = 'solved'
    except Exception as e:
        return dict(status='faults', faults=repr(e)[:200], loss_mW=None)
    ok = {n: v for n, v in I['net_list'].items() if v.get('status') == 'solved'}
    return dict(status=I.get('status'), loss_mW=1000 * sum(float(v['power_W']) for v in ok.values()),
                loss_per_net_mW={n: 1000 * float(v['power_W']) for n, v in sorted(ok.items())},
                max_drop_mV={n: 1000 * float(v['max_drop_V']) for n, v in sorted(ok.items())},
                current_per_net_A={n: float(v.get('total_current_A') or v.get('source_current_A') or 0.0) for n, v in sorted(ok.items())},
                current_mode_per_net={n: v.get('current_mode') for n, v in sorted(ok.items())},
                error_nets=sorted(n for n, v in I['net_list'].items() if v.get('status') == 'error'), without_load=without,
                current_mode=sorted({v.get('current_mode') for v in ok.values()}), faults=I.get('count_error'), zones_model=I.get('zones_model'))


MU0 = 1.2566
RHO = 1.72e-5


def sw_impedance(B, R, K, n):
    CONS = K.get('specs') or {}; fsw_cons = CONS.get('fsw') or {}
    chips = sorted({p['ref'] for p in B['pads'] if p['net'] == n and K['part_list'].get(p['ref'], {}).get('role') == 'chip'})
    f = next((float(fsw_cons[c]) for c in chips if c in fsw_cons), None); f_q = 'spec' if f else 'assumption_1MHz'
    f = f or 1e6
    h = 0.1; Rs = 0.0; Ls = 0.0
    for t in R['tracks']:
        if t['net'] != n: continue
        thick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            l = math.dist(a, b); Rs += RHO * l / (w * thick); Ls += MU0 * h / (w + 2 * h) * l
    nv = sum(1 for v in R['vias'] if v['net'] == n); Rs += 0.0015 * nv; Ls += 0.3 * nv
    Z = math.hypot(Rs, 2 * math.pi * f * Ls * 1e-9)
    fl = (K['net_list'].get(n) or {}).get('flow') or {}
    I = fl.get('total') if fl.get('unit') == 'A' else 1.0
    return dict(R_mOhm=1000 * Rs, L_nH=Ls, f_Hz=f, f_source=f_q, Z_mOhm=1000 * Z, I_A=I,
                loss_ac_mW=1000 * I * I * Rs, drop_ac_mV=1000 * I * Z)


def budgets(K):
    import re
    CONS = K.get('specs') or {}; per, gr = {}, {}
    for n in K['net_list']:
        for rx, v in (CONS.get('budget_mV') or {}).items():
            if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): per[n] = float(v); break
    for name, g in (CONS.get('channel_budget_mV') or {}).items():
        nn = [n for n in g.get('net_list', []) if n in K['net_list']]
        if nn: gr[name] = (nn, float(g['mV']))
    return per, gr


def budget_over(I, K):
    per, gr = budgets(K)
    if not per and not gr: return {}
    d = I.get('max_drop_mV') or {}
    out = {n: round(d[n] - b, 3) for n, b in per.items() if n in d and d[n] > b}
    for name, (nn, b) in gr.items():
        s = sum(d.get(n, 0.0) for n in nn)
        if s > b: out['channel:' + name] = round(s - b, 3)
    return out


def dc_values(E, B, R, K, flow_cache=None):
    I = E.get('ir')
    if not I or I.get('loss_mW') is None: return None
    out = {n: (float(I['max_drop_mV'][n]), float(I['loss_per_net_mW'][n]), 'ir') for n in I['max_drop_mV']}
    missing = [n for n in I.get('error_nets') or [] if n not in out]
    if missing:
        from analysis.measures import direct_current_estimate
        for n, (dv, p) in direct_current_estimate(B, R, K, missing, how_solver=True, flow_cache=flow_cache).items(): out[n] = (dv, p, 'estimate')
    return out


def score_v2(E, B, R, K, dc=None):
    I = E.get('ir')
    if dc is None: dc = dc_values(E, B, R, K)
    if dc is None or not I: return None, None
    N = K['net_list']; GND = set(K['gnd'])
    current_val = {n for n, k in N.items() if k.get('category') in ('current_val', 'supply', 'sw')}
    sw_nets = {n for n, k in N.items() if k.get('category') == 'sw'}
    total_length = sum(WT.get(N.get(n, {}).get('category', 'signal'), 1.0) * l for n, l in E['_L'].items() if n not in current_val and n not in GND)
    vias = sum(1 for v in R['vias'] if v['net'] not in current_val and v['net'] not in GND)
    S = total_length + 0.6 * vias + 50 * E['conflict_list'] + 200 * E['open_items']
    dc_P = sum(v[1] for n, v in sorted(dc.items()) if n not in sw_nets)
    dc_D = sum(v[0] for n, v in sorted(dc.items()) if n not in sw_nets)
    sw = {n: sw_impedance(B, R, K, n) for n in sorted(sw_nets) if (N[n].get('flow') or {}).get('total', 0) > 0}
    part_list = dict(S=S, power_loss=2 * dc_P, drop=dc_D, sw_loss=2 * sum(z['loss_ac_mW'] for z in sw.values()),
                 sw_drop=sum(z['drop_ac_mV'] for z in sw.values()))
    over = budget_over(dict(max_drop_mV={n: v[0] for n, v in dc.items()}), K)
    if over: part_list['budget'] = 20 * sum(over.values())
    fallback = sorted(n for n, v in dc.items() if v[2] != 'ir')
    return sum(part_list.values()), dict(part_list=part_list, sw=sw, budget_over=over, fallback=fallback, dc={n: [round(v[0], 3), round(v[1], 3), v[2]] for n, v in sorted(dc.items())})


def rate(B, R, K, ir=False, zone_list=None, v3=False, minimax=None, length_gate=False, si_score=False, si_measure=False, flow_cache=None):
    from analysis.standard import goal as default_target
    goal = default_target(K.get('specs'))
    if minimax is not None: goal = 'minimax' if minimax else 'v3'
    if goal not in ('v3', 'minimax'): raise ValueError('evaluation must be v3 or minimax')
    from common.zones import active, unify, board_geometry
    B = board_geometry(B, R)
    from analysis.r44 import measure as r44_measure, passed as r44_passed
    r44 = r44_measure(B, R)
    r36 = active(R)
    if any(z.get('r41') for z in R.get('zone_list', [])):
        R = dict(R, r41_fill_stat=dict(cut_mm2=sum((z.get('r41') or {}).get('cut_mm2', 0) for z in R['zone_list'])))
    R = unify(R)
    v3 = v3 or goal == 'minimax'
    endpoints = bool(R.get('current_shape') or str((R.get('options') or {}).get('current_fix', '0')) == '1')
    if flow_cache is None or (endpoints and not flow_cache.endpoints):
        flow_cache = NetGraphCache(endpoints=endpoints)
    infos = {}
    N = K['net_list']; GND = set(K['gnd'])
    pads = collections.defaultdict(list)
    for p in B['pads']:
        if p['net']: pads[p['net']].append((p['x'], p['y']))
    L = collections.defaultdict(float); deficit = 0.0
    pads_n = collections.defaultdict(list)
    for p in B['pads']: pads_n[p['net']].append(p)
    nominal = {}
    for n, k in N.items():
        if k.get('category') not in ('current_val', 'supply') or not k.get('flow'): continue
        tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
        vi = [v for v in R['vias'] if v['net'] == n]
        info = flow_analysis(tr, vi, pads_n[n], k['flow'], graph=flow_cache.graph(tr, vi, pads_n[n]))
        infos[n] = info
        if info:
            for ti, t in enumerate(tr):
                for si in range(len(t['w'])): nominal[(id(t), si)] = flow_goal_width(info['share'].get((ti, si), 1.0), k['wmin'], k['w'])


    from common.length import outer
    AU = outer(R, B); L_ges = 0.0
    for ti, t in enumerate(R['tracks']):
        k = N.get(t['net'], {})
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            d = AU[ti][si]; L[t['net']] += d; L_ges += math.dist(a, b)
            if k.get('category') in ('current_val', 'supply', 'sw') and not t.get('sig'): deficit += max(0.0, nominal.get((id(t), si), k['w']) - w) * d
    kind_L = collections.defaultdict(float); wt = 0.0; detours = []
    for n, l in L.items():
        a = 'gnd' if n in GND else N.get(n, {}).get('category', 'signal'); kind_L[a] += l
        if n in GND: continue
        wt += WT.get(a, 1.0) * l
        m = mst(pads[n])
        if l > m + 1.0: detours.append((round(l - m, 2), n, round(l, 1), round(m, 1)))
    detours.sort(reverse=True)
    diagnose = 'conflicts' in R and 'failed' in R
    conf = len(R.get('conflicts') or {}); open_items = len(R.get('failed') or [])
    unconn = not_connected(B, R, gnd_areas(R, GND, K) | native_areas(R, K), flow_cache=flow_cache, K=K)
    vias = len(R['vias'])
    tight = 0.0; tight_sites = []
    for n, k in N.items():
        if k.get('category') not in ('current_val', 'supply') or not k.get('flow'): continue
        tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
        info = infos.get(n)
        v, st = flow_narrowing(tr, info)
        tight += v; tight_sites += [(x[0], n, x[3], x[4], x[5]) for x in st[:3]]
    tight_sites.sort(reverse=True)
    summands = dict(total_length=wt, vias=0.6 * vias, deficit=10 * deficit, narrowing=10 * tight, conflict_list=50 * conf, open_items=200 * open_items)
    rating = sum(summands.values())
    rg = K.get('design_rules') or {}; rg = rg.get('router', rg)
    from analysis.measures import review_active
    minimum = max(float(rg.get('track_min') or 0),
                  float((R.get('options') or {}).get('track_min') or 0)) if review_active(K) else 0.0
    under = [dict(net_name=t['net'], layer_name=t['layer'], track=ti, segment=si, width=w, minimum=minimum)
             for ti, t in enumerate(R['tracks']) for si, w in enumerate(t['w']) if w < minimum - 1e-9]
    input_error = (R.get('short_circuit_protection') or {}).get('input_pad_contacts') or []
    is_valid = diagnose and conf == 0 and open_items == 0 and not unconn and not under and not input_error and r44_passed(r44)
    E_ir = ir_block(B, R, K, zone_list=zone_list) if ir else None
    if (R.get('copper_bars') or r36) and (v3 or ir):
        from analysis.copper_model import ir_values
        E_ir = ir_values(B, R, K)
        if r36 and E_ir['error_nets']:
            E_ir = ir_values(B, R, K, res=.05)

        affected = {z['net'] for z in R.get('zones', []) if z.get('copper_bars') or z.get('r36')}
        if affected.intersection(E_ir['error_nets']) or (r36 and E_ir['error_nets']):
            is_valid = False
    E = dict(ir=E_ir, _L=dict(L), rating=round(rating, 1), score_raw=rating, summands=summands, evaluation_version='rating-v1', is_valid=is_valid, rank=[0 if is_valid else 1, open_items + len(unconn), conf, round(rating, 1)],
                r44=r44, input_copper_error=input_error, conflict_list=conf, open_items=open_items, not_connected=unconn, diagnose=diagnose, vias=vias, weighted=round(wt, 1),
                total_length={a: round(v, 1) for a, v in sorted(kind_L.items())}, length_total=round(L_ges, 1), length_in_pad=round(L_ges - sum(L.values()), 1), deficit=round(deficit, 2),
                narrowing=round(tight, 2), narrowings=tight_sites[:10],
                detour_total_sum=round(sum(u[0] for u in detours), 1), detours=detours[:15])
    if under: E['min_width_violations'] = under
    dc = dc_values(E, B, R, K, flow_cache=flow_cache) if ir else None
    v2, v2d = score_v2(E, B, R, K, dc=dc) if ir else (None, None)
    E.update(score_v2=round(v2, 1) if v2 is not None else None, score_v2_raw=v2, score_v2_parts=v2d,
             score_v2_complete=(not v2d['fallback']) if v2d else None)
    E['current_paths'] = current_paths(B, R, K, E, dc=dc)
    if v3:
        from analysis.measures import measures
        M = measures(B, R, K, ir=E_ir if (E_ir and (E_ir.get('max_drop_mV') or R.get('copper_bars'))) else None, AU=AU, flow_cache=flow_cache, minimax=goal == 'minimax', si_score=si_score, si_measure=si_measure or length_gate)
        E.update(score_v3=round(M['score_v3'] + 50 * conf + 200 * open_items, 1), score_v3_nets=M['score_v3'], measures=M,
                 score_v3_version=M.get('version'))
        if goal == 'minimax':
            E.update({k: M[k] for k in ('minimax', 'minimax_version', 'minimax_profile', 'minimax_rank', 'minimax_total_sum',
                                          'minimax_complete', 'minimax_board_contract', 'minimax_sources', 'chokepoint', 'safety')})
            E['evaluation_target'] = goal
            p0, _ = current_loss_equal(E, E, B, R, B, R, K)
            E['minimax_current_paths'] = dict(detour_mm=E['current_paths']['detour_mm'], loss_mW=p0)
            from analysis import minimax as MM
            drc = R.get('drc')
            if 'drc' in R:
                schemas = {'drc-v1': ('faults', 'open_items', 'routing', 'not_connected'),
                           'corpus-drc-v1': ('all_items', 'slot', 'routing', 'not_connected')}
                keys = schemas.get(drc.get('version'), ()) if isinstance(drc, dict) and isinstance(drc.get('version'), str) else ()
                known = bool(keys) and all(
                    isinstance(drc.get(k), int) and not isinstance(drc[k], bool) and drc[k] >= 0 for k in keys)
                if not known:
                    E['safety']['absent'].append(dict(net_name='Board', criterion='DRC', reason='complete versioned DRC schema missing'))
                    E['minimax_complete'] = False

                if isinstance(drc, dict) and any(isinstance(drc.get(k), int) and not isinstance(drc[k], bool) and drc[k] > 0 for k in ('faults', 'open_items', 'routing', 'not_connected', 'all_items', 'slot')):
                    E['safety']['faults'].append(dict(net_name='Board', criterion='DRC', reason='external DRC not error-free'))

            gnd_open = [n for n in unconn if n in GND]
            if gnd_open:
                E['safety']['faults'].extend(dict(net_name=n, criterion='joined', reason='GND without own area unconnected') for n in gnd_open)
                E['safety']['safe'] = False
            MM.safety_status(E['safety'])
            E['minimax_status'] = ('invalid' if not E['is_valid'] or E['safety']['faults'] else
                                   'unknown' if not E['minimax_complete'] else 'is_valid')
            E['minimax_valid'] = None if E['minimax_status'] == 'unknown' else E['minimax_status'] == 'is_valid'
            if goal == 'minimax':
                E['evaluation_version'] = E['minimax_version']
                E['valid_routing'] = E['is_valid']


        from analysis import rule_metrics as RK
        E['design_rules'] = RK.measure(B, R, K, measures_result=M, si_score=si_score, r44_result=r44)
        from analysis.areas import active
        if active(R, K) and E['design_rules'].get('faults'):
            E.update(is_valid=False, evaluation_error=E['design_rules']['faults'])
            E['rank'][0] = 1
        if (K.get('specs') or {}).get('physics_version') == 'v3.3-contract_spec-option':
            from analysis import contract_spec as VT
            E['proofs'] = VT.mandatory_proofs(M, K)
            E['geometric_valid'] = is_valid
            E['gate_check'] = VT.cur_state(E)
            E['complete_valid'] = E['gate_check']['complete_valid']
            E['design_rules']['gate_check'] = E['gate_check']
    if (si_measure or si_score or length_gate) and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('length_group') for k in N.values()) or (K.get('specs') or {}).get('length_groups')):
        from analysis.length_impedance import measure as _si_measure
        E['signal_integrity'] = M['signal_integrity'] if v3 else _si_measure(B, R, K)
        E['si_score_active'] = bool(v3 and si_score)
        if length_gate and not E['signal_integrity']['length_gate']:
            E['is_valid'] = False
            E['rank'][0] = 1
            E['rule_violations_length'] = [g for g, q in E['signal_integrity']['length_groups'].items()
                                          if not q['complete'] or q['over_mm'] > 1e-6]


    if 'minimax_status' in E and not E['is_valid']:
        E['minimax_status'] = 'invalid'; E['minimax_valid'] = False
    return E


def current_paths(B, R, K, E, estimate=False, dc=False):
    N = K['net_list']; GND = set(K['gnd'])
    net_list = sorted(n for n, k in N.items() if k.get('category') in ('current_val', 'supply', 'sw') and n not in GND)
    pads = collections.defaultdict(list)
    for p in B['pads']:
        if p['net']: pads[p['net']].append((p['x'], p['y']))
    L = E['_L']; um = {}
    for n in net_list:
        if n in L: um[n] = round(max(0.0, L[n] - mst(pads[n])), 3)
    if dc is False: dc = dc_values(E, B, R, K)
    P = {n: v[1] for n, v in dc.items()} if dc is not None else {}
    missing = [n for n in net_list if n not in P and n in L]
    if missing and (dc is not None or estimate):
        from analysis.measures import direct_current_estimate
        for n, (_dv, p) in direct_current_estimate(B, R, K, missing, how_solver=True).items(): P[n] = p
    entire = dc is not None or estimate
    return dict(detour_mm=round(sum(um.values()), 2), loss_mW=round(sum(P.get(n, 0.0) for n in net_list if n in L), 3) if entire else None,
                source='ir' if dc is not None else ('tree_idx' if estimate else None),
                per_net={n: [um.get(n, 0.0), round(P[n], 3) if n in P else None] for n in net_list if n in L})
def comparison(E0, E1, B0, R0, B1, R1, K, rating='v2', tol_detour_mm=1.0, tol_loss=0.02, K1=None):
    if rating == 'v3' and (E0.get('evaluation_target') == 'minimax' or E1.get('evaluation_target') == 'minimax'): rating = 'minimax'
    out = dict(rating=rating)
    if rating in ('v3', 'minimax'):
        key = 'score_v3_version' if rating == 'v3' else 'minimax_version'
        if not E0.get(key) or E0.get(key) != E1.get(key):
            return dict(rating=rating, delta=None, better=False, reason='different/missing evaluation versions')
    if rating == 'minimax':
        from analysis import minimax as MM
        return dict(rating=rating, **MM.assumption(E0, E1, tol_detour_mm, tol_loss))
    if any(e.get('gate_check') or (e.get('design_rules') or {}).get('measurement_version') for e in (E0,E1)):
        from analysis.contract_spec import comparison as contract_comparison
        c=contract_comparison(E0,E1)
        if not c['not_worse']:return dict(rating=rating,delta=None,better=False,reason=c['reason'])
    r0, r1 = tuple(E0['rank'][:3]), tuple(E1['rank'][:3])
    if r0 != r1:
        out.update(delta=None, better=r1 < r0, reason='hard gates %s -> %s' % (list(r0), list(r1))); return out
    if rating == 'v2':
        d0, d1 = dc_values(E0, B0, R0, K), dc_values(E1, B1, R1, K)
        if d0 is None or d1 is None:
            out.update(rating='v1', delta=E1['score_raw'] - E0['score_raw'], hint='IR missing on one side: score-v1 on both sides')
        else:
            from analysis.measures import direct_current_estimate
            both = {n for n in set(d0) | set(d1) if d0.get(n, (0, 0, ''))[2] == 'ir' and d1.get(n, (0, 0, ''))[2] == 'ir'}
            rest = sorted((set(d0) | set(d1)) - both)
            s0 = direct_current_estimate(B0, R0, K, rest, how_solver=True) if rest else {}
            s1 = direct_current_estimate(B1, R1, K, rest, how_solver=True) if rest else {}
            c0 = {n: d0[n] for n in both}; c1 = {n: d1[n] for n in both}
            for n in rest:
                if n in s0: c0[n] = s0[n] + ('estimate',)
                if n in s1: c1[n] = s1[n] + ('estimate',)
            v0, _ = score_v2(E0, B0, R0, K, dc=c0); v1, _ = score_v2(E1, B1, R1, K, dc=c1)
            out.update(delta=v1 - v0, equal_computed=rest)
    elif rating == 'v3':
        if E0.get('score_v3') is None or E1.get('score_v3') is None: raise ValueError('score_v3 missing: rate(..., v3=True)')
        out['delta'] = E1['score_v3'] - E0['score_v3']
    else:
        out['delta'] = E1['score_raw'] - E0['score_raw']
    s0 = E0.get('current_paths') or current_paths(B0, R0, K, E0, dc=None); s1 = E1.get('current_paths') or current_paths(B1, R1, K, E1, dc=None)
    du = (s1.get('detour_mm') or 0.0) - (s0.get('detour_mm') or 0.0)
    p0, p1 = current_loss_equal(E0, E1, B0, R0, B1, R1, K)
    dp = p1 - p0
    worse = du > tol_detour_mm and dp > tol_loss * max(1e-9, p0)
    out.update(loss_current_before_mW=round(p0, 3))
    out.update(detour_current_mm=round(du, 2), loss_current_mW=round(dp, 3), current_paths_worse=worse)
    from analysis import rule_metrics as RK
    cons = RK.assumption(E0.get('design_rules') or RK.measure(B0, R0, K),
                    E1.get('design_rules') or RK.measure(B1, R1, K1 if K1 is not None else K))
    out['rule_gate'] = cons


    out['better'] = out['delta'] < -1e-6 and cons['not_worse'] and not worse
    out['reason'] = 'Current paths: detour %+.1f mm, loss %+.1f mW' % (du, dp) if (worse and out['delta'] < 0) else ('Score %+.1f' % out['delta'])
    if not cons['not_worse']: out['reason'] = cons['reason']
    return out


def current_loss_equal(E0, E1, B0, R0, B1, R1, K):
    from analysis.measures import direct_current_estimate
    N = K['net_list']; GND = set(K['gnd'])
    net_list = sorted(n for n, k in N.items() if k.get('category') in ('current_val', 'supply', 'sw') and n not in GND)
    d0, d1 = dc_values(E0, B0, R0, K) or {}, dc_values(E1, B1, R1, K) or {}
    both = {n for n in net_list if (d0.get(n) or (0, 0, ''))[2] == 'ir' and (d1.get(n) or (0, 0, ''))[2] == 'ir'}
    rest = [n for n in net_list if n not in both]
    s0 = direct_current_estimate(B0, R0, K, rest, how_solver=True) if rest else {}
    s1 = direct_current_estimate(B1, R1, K, rest, how_solver=True) if rest else {}
    P0 = sum(d0[n][1] for n in sorted(both)) + sum(s0[n][1] for n in rest if n in s0)
    P1 = sum(d1[n][1] for n in sorted(both)) + sum(s1[n][1] for n in rest if n in s1)
    return P0, P1


def better(E1, E2):
    if E1.get('evaluation_target') == 'minimax' or E2.get('evaluation_target') == 'minimax': return better_v3(E1, E2)
    return tuple(E1['rank']) < tuple(E2['rank'])


def better_v3(E1, E2):
    if any(e.get('gate_check') or (e.get('design_rules') or {}).get('measurement_version') for e in (E1,E2)):
        from analysis.contract_spec import comparison
        return comparison(E2,E1)['better']
    if E1.get('evaluation_target') == 'minimax' or E2.get('evaluation_target') == 'minimax':
        from analysis import minimax as MM
        return MM.assumption(E2, E1)['better']
    if not E1.get('score_v3_version') or E1.get('score_v3_version') != E2.get('score_v3_version'): return False
    a, b = tuple(E1['rank'][:3]), tuple(E2['rank'][:3])
    if a != b: return a < b
    return E1['score_v3'] < E2['score_v3']


def image(B, R, K, prefix, E=None, top=None):
    from PIL import Image, ImageDraw, ImageFont
    N = K['net_list']; GND = set(K['gnd'])
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0 = min(xs) - 1, min(ys) - 1; S = 40.0 if max(xs) - min(xs) < 60 else 12.0
    W, H = int((max(xs) - x0 + 1) * S), int((max(ys) - y0 + 1) * S)
    tr = lambda x, y: ((x - x0) * S, (y - y0) * S)
    FAR = {'sw': (200, 30, 30), 'current_val': (220, 70, 70), 'supply': (230, 120, 120), 'fast': (240, 140, 0),
           'analog': (30, 150, 60), 'signal': (60, 100, 220)}
    try: font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 13)
    except OSError: font = None
    um = {u[1] for u in (top or [])}
    cu_layers = sorted({t['layer'] for t in R['tracks']}, key=lambda L: (L != 'F', L == 'B', L))
    for Lg in cu_layers:
        in_ = Image.new('RGB', (W, H), 'white'); d = ImageDraw.Draw(in_)
        for o in B['outline']: d.polygon([tr(*p) for p in o], outline=(0, 0, 0))
        for p in B['pads']:
            if Lg in p['polys']:
                for q in p['polys'][Lg]:
                    if len(q) >= 3: d.polygon([tr(*c) for c in q], fill=(200, 200, 200))
        for t in R['tracks']:
            if t['layer'] != Lg or t['net'] in GND: continue
            col = FAR.get(N.get(t['net'], {}).get('category', 'signal'), (90, 90, 90))
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if t['net'] in um: d.line([tr(*a), tr(*b)], fill=(160, 40, 200), width=max(3, int(w * S) + 6))
                d.line([tr(*a), tr(*b)], fill=col, width=max(1, int(w * S)))
        for v in R['vias']:
            cx, cy = tr(v['x'], v['y']); d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], outline=(0, 0, 0))
        if E:
            for r, mv in (E.get('shifts') or {}).items():
                ps = [(p['x'], p['y']) for p in B['pads'] if p['ref'] == r]
                if not ps: continue
                cx = sum(p[0] for p in ps) / len(ps); cy = sum(p[1] for p in ps) / len(ps)
                a = tr(cx, cy); b = tr(cx + mv['dx'] * 6, cy + mv['dy'] * 6)
                d.line([a, b], fill=(0, 0, 0), width=3); d.ellipse([b[0] - 4, b[1] - 4, b[0] + 4, b[1] + 4], fill=(0, 0, 0))
                if font: d.text((b[0] + 5, b[1] - 6), r, fill=(0, 0, 0), font=font)
        if font: d.text((8, 6), 'Layer %s  red current, orange fast, green analog, blue signal; violet: detours; arrows: suggestions (x6)' % Lg,
                        fill=(0, 0, 0), font=font)
        in_.save('%s_%s.png' % (prefix, Lg))
    return ['%s_%s.png' % (prefix, L) for L in cu_layers]


if __name__ == '__main__':
    B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3]))
    kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    if 'layer_stack' in kw:
        K = dict(K, specs=dict(K.get('specs') or {}, layer_stack=kw['layer_stack'] == '1'))
    V3 = kw.get('v3', '1') == '1'
    MM = kw.get('minimax') == '1' or kw.get('rating') == 'minimax'
    E1 = rate(B, R, K, ir=kw.get('ir') == '1', v3=V3, minimax=True if MM else None, length_gate=kw.get('length_gate') == '1', si_score=kw.get('si_score') == '1', si_measure=kw.get('si_measure') == '1')
    print('%s | ' % ('VALID' if E1['is_valid'] else 'INVALID (do not compare score with valid ones)'), end='')
    if E1['not_connected']: print('unconnected: %s | ' % ', '.join(n.split('/')[-1] for n in E1['not_connected'][:6]), end='')
    if not E1['diagnose']: print('Diagnostic fields missing | ', end='')
    print('Score %.1f | Conflicts %d | open %d | Vias %d | weighted %.1f | Deficit current %.2f mm2 | Constriction %.2f mm2 | Detours %.1f mm' % (
        E1['rating'], E1['conflict_list'], E1['open_items'], E1['vias'], E1['weighted'], E1['deficit'], E1['narrowing'], E1['detour_total_sum']))
    for v in E1['narrowings'][:4]: print('  Flow constriction %-10s %.2f mm instead of %.2f mm at %s (%.2f mm2)' % (v[1].split('/')[-1], v[2], v[3], v[4], v[0]))
    if E1.get('score_v2') is not None:
        t = E1['score_v2_parts']['part_list']
        print('score-v2 %.1f (Signal S %.1f, Loss %.1f, Drop %.1f, Switching node loss %.1f, Switching node drop %.1f; score-v1 %.1f)' % (
            E1['score_v2'], t['S'], t['power_loss'], t['drop'], t['sw_loss'], t['sw_drop'], E1['rating']))
        for n, z in E1['score_v2_parts']['sw'].items():
            print('  Switching node %-14s R %.1f mOhm, L %.2f nH, f %.2f MHz (%s), |Z| %.1f mOhm at %.2f A' % (
                n.split('/')[-1], z['R_mOhm'], z['L_nH'], z['f_Hz'] / 1e6, z['f_source'], z['Z_mOhm'], z['I_A']))
    if E1.get('ir') and E1['ir'].get('loss_mW') is not None:
        mx = sorted(E1['ir']['max_drop_mV'].items(), key=lambda kv: -kv[1])[:3]
        print('Voltage drop (%s): losses %.1f mW, largest drop %s' % (','.join(str(m) for m in E1['ir'].get('current_mode') or []) or '?', E1['ir']['loss_mW'],
              ', '.join('%s %.1f mV' % (n.split('/')[-1], v) for n, v in mx)))
    if E1.get('score_v3') is not None:
        from analysis.measures import table
        print('score-v3 %.1f (Nets %.1f, %.2f s CPU); worst nets:' % (E1['score_v3'], E1['score_v3_nets'], E1['measures'].get('cpu_s', E1['measures']['time_s'])))
        print(table(E1['measures'], 10))
    if E1.get('minimax') is not None:
        e = E1['chokepoint'] or {}
        print('Minimax %.6g (%s); bottleneck: %s / %s, x=%.3g; total v3.1 %.2f; safe %s' %
              (E1['minimax'], E1['minimax_profile'], e.get('net_name', '-'), e.get('criterion', '-'), e.get('x', 0),
               E1['minimax_total_sum'], E1['safety']['safe']))
    print('Length per type:', E1['total_length'])
    for u in E1['detours'][:6]: print('  Detour %-26s +%.1f mm (%.1f instead of %.1f)' % (u[1].split('/')[-1], u[0], u[2], u[3]))
    if E1.get('current_paths'):
        sp = E1['current_paths']
        print('Current paths: detour %.1f mm, loss %s (%s)' % (sp['detour_mm'], '%.1f mW' % sp['loss_mW'] if sp['loss_mW'] is not None else '-', sp['source']))
    if 'comparison' in kw:
        R2 = json.load(open(kw['comparison']))
        E2 = rate(B, R2, K, ir=kw.get('ir') == '1', v3=V3, minimax=True if MM else None, length_gate=kw.get('length_gate') == '1', si_score=kw.get('si_score') == '1', si_measure=kw.get('si_measure') == '1')
        print('Comparison (hard gates first): %s' % ('better' if better(E1, E2) else ('equal' if E1['rank'] == E2['rank'] else 'worse')))
        for nt in (['v3'] if V3 else []) + (['v2'] if kw.get('ir') == '1' else []):
            cons = comparison(E2, E1, B, R2, B, R, K, rating=nt)
            print('Comparison %s (computed the same way): %s, %s%s' % (cons['rating'], 'better' if cons['better'] else 'not better', cons['reason'],
                  '; current paths detour %+.1f mm, loss %+.1f mW' % (cons['detour_current_mm'], cons['loss_current_mW']) if 'detour_current_mm' in cons else ''))
        print('Comparison: Score %.1f -> %.1f (%+.1f), Vias %d -> %d, Deficit %.2f -> %.2f, Constriction %.2f -> %.2f, Detours %.1f -> %.1f, Conflicts %d -> %d' % (
            E2['rating'], E1['rating'], E1['rating'] - E2['rating'], E2['vias'], E1['vias'], E2['deficit'], E1['deficit'],
            E2['narrowing'], E1['narrowing'], E2['detour_total_sum'], E1['detour_total_sum'], E2['conflict_list'], E1['conflict_list']))
    if 'json' in kw: json.dump(E1, open(kw['json'], 'w'), indent=1)
    if 'image' in kw:
        E = json.load(open(kw['bottlenecks'])) if 'bottlenecks' in kw else None
        print('Images:', ' '.join(image(B, R, K, kw['image'], E, E1['detours'][:8])))
