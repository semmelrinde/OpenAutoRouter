import collections
import hashlib
import heapq
import json
import math
import re

VERSION = 'minimax-v4.1-copper_contract'


def _count(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x) and x >= 0


def load_contract(n, K, I, source, ir=None):
    k = K['net_list'][n]; ir = ir or {}
    v = dict(I_A=I, I_mode=source, current_A=k.get('current_A'), current_A_source=k.get('current_A_source'),
             flow=k.get('flow'), category=k.get('category'),
             ir_current_A=(ir.get('current_per_net_A') or {}).get(n),
             ir_mode=(ir.get('current_mode_per_net') or {}).get(n),
             ir_without_load=(ir.get('without_load') or {}).get(n), ir_zones_model=ir.get('zones_model'))
    return hashlib.sha256(json.dumps(v, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_error(k):
    fl = k.get('flow') or {}
    load = fl.get('load') or {}
    if not isinstance(load, dict) or any(not _count(q) for q in load.values()): return 'Load distribution invalid'
    if fl.get('unit') == 'A':
        i = fl.get('total')
        if not _count(i): return 'flow.total in A missing/invalid'
        if i == 0 and (any(q > 0 for q in load.values()) or (k.get('current_A') or 0) > 0):
            return 'Zero load contradicts positive load'


        if load and abs(sum(load.values()) - i) > (len(load) + 1) * 5e-8 + 1e-12:
            return 'Load sum does not match total'
    elif not _count(k.get('current_A')): return 'current_A missing/invalid (fallback 1A)'
    return None

def priorities(cons, net_list):
    raw = cons.get('priority_nets') or {}
    if isinstance(raw, list): raw = {n: 2.0 for n in raw}
    if not isinstance(raw, dict): raise ValueError('priority_nets must be a list or weights')
    if any(not _count(q) or q < 1 for q in raw.values()): raise ValueError('Net priorities must be finite and >= 1')
    return {n: float(raw.get(n, 1)) for n in sorted(net_list)}


def _spec(v, n):
    for rx, x in sorted((v or {}).items()):
        if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): return float(x)
    return None


def _path_length(tracks, vias, pads, A):
    from common.flow import net_graph
    if len(pads) != 2: return None
    g = net_graph(tracks, vias, pads, 0.03)
    s, z = [('PAD', p['ref'], p['num']) for p in pads]
    best = {s: 0.0}; h = [(0.0, s)]
    while h:
        d, k = heapq.heappop(h)
        if k == z: return d
        if d > best[k]: continue
        for q, kid in g['adj'].get(k, []):
            l = 0.0
            if kid and kid[0] == 's':
                t = tracks[kid[1]]; l = math.dist(t['pts'][kid[2]], t['pts'][kid[2] + 1]) * (kid[4] - kid[3])
            elif kid and kid[0] == 'v': l = abs(A['cu_layers'][k[0]]['z'] - A['cu_layers'][q[0]]['z'])
            nd = d + l
            if nd < best.get(q, math.inf): best[q] = nd; heapq.heappush(h, (nd, q))
    return None


def _shape_line(t):
    if len(t['pts']) < 2: return t
    pts = [t['pts'][0], t['pts'][1]]; ws = [t['w'][0]]
    for q, w in zip(t['pts'][2:], t['w'][1:]):
        a, b = pts[-2:]; u = (b[0]-a[0], b[1]-a[1]); v = (q[0]-b[0], q[1]-b[1])
        if abs(w-ws[-1]) < 1e-12 and abs(u[0]*v[1]-u[1]*v[0]) < 1e-10 and u[0]*v[0]+u[1]*v[1] > 0:
            pts[-1] = q
        else: pts.append(q); ws.append(w)
    return dict(t, pts=pts, w=ws)


def _shape_chains(tracks):
    segs = []; adj = collections.defaultdict(list)
    def key(t, p): return (t['net'], t['layer'], round(p[0], 9), round(p[1], 9))
    for t in tracks:
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) < 1e-12: continue
            i = len(segs); segs.append((t, a, b, w))
            adj[key(t, a)].append(i); adj[key(t, b)].append(i)
    used = set(); out = []
    def follow(i, a):
        t, x, y, w = segs[i]; b = y if key(t, x) == key(t, a) else x
        used.add(i); pts = [a, b]; ws = [w]
        while len(adj[key(t, b)]) == 2:
            j = next((j for j in adj[key(t, b)] if j not in used), None)
            if j is None: break
            tj, xj, yj, wj = segs[j]
            if abs(wj - ws[-1]) > 1e-12: break
            q = yj if key(tj, xj) == key(t, b) else xj
            used.add(j); pts.append(q); ws.append(wj); b = q

        b = a
        while len(adj[key(t, b)]) == 2:
            j = next((j for j in adj[key(t, b)] if j not in used), None)
            if j is None: break
            tj, xj, yj, wj = segs[j]
            if abs(wj - ws[0]) > 1e-12: break
            q = yj if key(tj, xj) == key(t, b) else xj
            used.add(j); pts.insert(0, q); ws.insert(0, wj); b = q
        return _shape_line(dict(net=t['net'], layer=t['layer'], pts=pts, w=ws))

    for i, (t, a, b, w) in enumerate(segs):
        if i in used: continue
        if len(adj[key(t, a)]) != 2: out.append(follow(i, a))
        elif len(adj[key(t, b)]) != 2: out.append(follow(i, b))
    for i, (t, a, b, w) in enumerate(segs):
        if i not in used: out.append(follow(i, a))
    return out

def measure_extra(M, B, R, K, G, ir=None):
    from analysis.metrics import metrics
    from analysis.measures import structure, line, width_ipc, current_tree, current_density, VIA_SLEEVE, T_INNER
    cfg = G['minimax']; S = cfg['thresholds']; CONS = K.get('specs') or {}
    A = structure(len(M['structure']['stack']), M['structure']['levels'],
               diel=M['structure']['dielectrics'], er=float((CONS.get('structure') or {}).get('er', 4.1)))
    tracks = collections.defaultdict(list); vias = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R['tracks']: tracks[t['net']].append(t)
    for v in R['vias']: vias[v['net']].append(v)
    for p in B['pads']: pads[p['net']].append(p)
    absent = []; group_list = collections.defaultdict(list)
    from analysis.current_chain import link_chain
    from common.zones import unify
    chain = link_chain(B, unify(R), K, ir=ir)
    M['current_chain'] = chain
    M['minimax_priorities'] = priorities(CONS, M['net_list'])
    budget_contract = {}
    drill = float(str((R.get('options') or {}).get('via') or (K.get('design_rules') or {}).get('via') or '0.45/0.20').split('/')[-1])
    for n, v in sorted(M['net_list'].items()):
        m = v['m']; net_copper = K['net_list'].get(n) or {}; x = {}
        lf = load_error(net_copper) if m.get('carries') else None
        if lf: absent.append(n + ': ' + lf)
        if net_copper.get('category') in ('current_val', 'supply') and m.get('carries') and not lf and m['safety_raw']['I_A'] > 0:
            q = chain['net_list'].get(n)
            if q:
                m['current_chain'] = q
                raw_data = q['raw_data']
                eta, element = raw_data['efficiency'], raw_data['element']
                if _count(eta) and eta > 0: x['chain_eta'] = 1 / eta / S['chain_inefficiency']
                else: absent.append(n + ': chain efficiency missing')
                if _count(element): x['chain_element'] = element / S['chain_element']
                else: absent.append(n + ': chain current density missing')


                area = _spec(CONS.get('copper_budget_mm2'), n)
                if area is None:
                    dv = m['safety_raw'].get('s_dc_mV')
                    l = raw_data['l_eff_mm']; i = raw_data['current_A']
                    area = max(net_copper.get('wmin', .1) * l, (1.72e-5 / .035) * i * l*l * 1000 / dv) if dv else None
                if _count(area) and area > 0: x['chain_copper'] = raw_data['copper_mm2'] / area / S['chain_copper']
                else: absent.append(n + ': copper budget missing')
            else: absent.append(n + ': current chain not solved (load/source/copper)')
        budget_contract[n] = dict(wmin_mm=net_copper.get('wmin', .1), drop_mV=m.get('safety_raw', {}).get('s_dc_mV'),
             loss_mW=_spec(CONS.get('loss_budget_mW'), n),
             copper_mm2=_spec(CONS.get('copper_budget_mm2'), n),
             z_ohm=net_copper.get('z_ohm'), z_diff_ohm=net_copper.get('z_diff_ohm'),
             z_tolerance_rel=net_copper.get('z_tolerance_rel', S['z_tolerance_rel']),
             length_group=net_copper.get('length_group'), tolerance_mm=net_copper.get('tolerance_mm'))
        if not A['levels'] and (v['net_class'].startswith('fast') or v['net_class'] == 'analog'):
            absent.append(n + ': return path/Johnson not measured without reference area')
        if m.get('carries') and (m.get('safety_raw', {}).get('I_A', m.get('I_A')) or 0) > 0:

            tl = _shape_chains([t for t in tracks[n] if not t.get('sig') and t['layer'] in A['cu_layers']])
            tree_idx = current_tree(tl, vias[n], pads[n], net_copper.get('flow'), A, drill,
                             m['safety_raw']['I_A']) if tl else None
            j = current_density(tree_idx['seg'], A, n, pads[n])[0] if tree_idx else None
            m['safety_raw']['J_x'] = j
            if j is not None: x['j'] = j
            sb = _spec(CONS.get('loss_budget_mW'), n)
            if sb is None: sb = m['I_A'] * (m.get('s_dc_mV') or 0)
            if not sb or sb <= 0: absent.append(n + ': loss budget missing')
            elif m.get('P_mW') is not None: x['power_loss'] = m['P_mW'] / sb
            else: absent.append(n + ': loss measurement missing')

            vi = m.get('via_I_A')
            if vias[n]:
                currents = vi if vi is not None else [m['safety_raw']['I_A']] * len(vias[n])
                barrel_w = math.pi * (drill + VIA_SLEEVE) * VIA_SLEEVE / T_INNER
                m['J_via_x'] = max((width_ipc(i, True) / barrel_w for i in currents), default=0.0)
            else: m['J_via_x'] = 0.0
        if tracks[n]:

            logical = []; arcs = 0
            for t in tracks[n]:
                if 'items' not in t: logical.append(_shape_line(t)); continue
                for it in t['items']:
                    if it[0] == 'A': arcs += 1
                    elif it[0] == 'L': logical.append(dict(net=n, layer=t['layer'], pts=[it[1], it[2]], w=[it[3]]))
            f = metrics(dict(tracks=_shape_chains(logical), vias=vias[n]))
            f['segments'] += arcs
            if arcs:


                absent.append(n + ': native arc geometry for shape/connections not fully measured')
            x['shape_mini'] = f['mini'] / max(1, f['segments']) / S['mini_share']
            x['shape_bends'] = f['bends'] / max(1.0, m['length_mm']) / S['bends_pro_mm']
            x['shape_joints'] = (f['fast_joints'] + f['vias_fast']) / S['fast_joints']
        z = net_copper.get('z_ohm')
        if z:
            tol = float(net_copper.get('z_tolerance_rel') or S['z_tolerance_rel'])
            zz = [line(A, t['layer'], w)[0] for t in tracks[n] if t['layer'] in A['cu_layers'] for w in t['w']]
            if zz and all(q > 0 for q in zz): x['impedance'] = max(abs(q / z - 1) / tol for q in zz)
            else: absent.append(n + ': impedance without reference/line')
        if net_copper.get('z_diff_ohm'): absent.append(n + ': differential impedance not measured')
        gr = net_copper.get('length_group')
        if gr: group_list[gr].append((n, net_copper.get('tolerance_mm')))
        m['minimax_x'] = x
    for gr, gg in sorted((CONS.get('length_groups') or {}).items()):
        group_list[gr] = [(n, gg.get('tolerance_mm')) for n in gg['net_list'] if n in M['net_list']]
    for gr, nn in sorted(group_list.items()):
        ll = {n: _path_length(tracks[n], vias[n], pads[n], A) for n, _ in nn}
        if len(nn) < 2 or any(l is None for l in ll.values()):
            absent.append(gr + ': length matching needs at least two two-point nets'); continue
        delta = max(ll.values()) - min(ll.values())
        for n, tol in nn:
            if not tol or tol <= 0: absent.append(n + ': length tolerance missing'); continue
            M['net_list'][n]['m']['minimax_x']['length_error'] = delta / tol
    M['minimax_board_contract'] = dict(net_list=budget_contract, length_groups=CONS.get('length_groups'),
                                        channel_budget_mV={g:v.get('mV') for g,v in M.get('group_list', {}).items()}, priorities=M['minimax_priorities'])
    M['minimax_standard_contract'] = standard_contract(G)
    M['minimax_missing'] = absent
    return M


def safety(M):
    faults = []; absent = []
    def verify(n, key, amount, limit):
        if not _count(amount) or not _count(limit) or limit <= 0:
            absent.append(dict(net_name=n, criterion=key, reason='Measurement/limit missing or invalid'))
        elif amount > limit + 1e-9:
            faults.append(dict(net_name=n, criterion=key, amount=amount, limit=limit))
    for n, v in sorted(M['net_list'].items()):
        m = v['m']; raw_data = m.get('safety_raw') or m
        if not m.get('carries'): continue
        current_val = raw_data.get('I_A', m.get('I_A'))
        if not _count(current_val):
            absent.append(dict(net_name=n, criterion='current_val', reason='I_A missing or invalid')); continue
        if current_val == 0: continue
        for key, amount, limit in [('current_heat', raw_data.get('J_x'), 1.0), ('via_heat', m.get('J_via_x'), 1.0),
                                   ('drop', raw_data.get('dV_mV'), raw_data.get('s_dc_mV'))]:
            verify(n, key, amount, limit)
    for gr, v in sorted((M.get('group_list') or {}).items()):

        verify('channel:' + gr, 'drop', v.get('total_sum_raw_mV', v.get('total_sum_mV')), v.get('mV'))
    return safety_status(dict(faults=faults, absent=absent,
                                  model='IPC-2221/10K, flow tree, via copper cross-section'))


def safety_status(s):
    s['status'] = 'uncertain' if s['faults'] else ('unknown' if s.get('absent') else 'safe')
    s['safe'] = s['status'] == 'safe'
    return s

def evaluate(M, G, profile_data='balanced'):
    from analysis.measures import net_x
    if M.get('minimax_standard_contract') and M['minimax_standard_contract'] != standard_contract(G):
        raise ValueError('Minimax thresholds/rise times changed: measure again')
    if profile_data not in G['profile']: raise ValueError('Unknown profile: ' + str(profile_data))
    w = G['profile'][profile_data]['weights']; part_list = []
    for n, v in sorted(M['net_list'].items()):
        x = net_x(v['m'], G['netclasses'].get(v['net_class']) or {}, v['net_class'] == 'sw')
        if not v['net_class'].startswith('fast'): x.pop('vias', None)

        m = v['m']
        if m.get('carries') and m.get('s_dc_mV'): x['dc'] = m['dV_mV'] / m['s_dc_mV']
        x.update(m.get('minimax_x') or {})
        for k, q in sorted(x.items()):
            if not _count(q): raise ValueError('Invalid minimax measurement: %s %s' % (n, k))
            weight = float(w.get(k, 1.0)) * (M.get('minimax_priorities') or {}).get(n, 1)
            if not math.isfinite(weight) or weight <= 0: raise ValueError('Minimax weight must be positive: ' + k)
            part_list.append(dict(net_name=n, criterion=k, x=q, weight=weight, amount=weight * max(0.0, q - 1.0)))
    for gr, v in sorted((M.get('group_list') or {}).items()):
        if v.get('mV') and v['mV'] > 0:
            x = v['total_sum_mV'] / v['mV']; g = float(w.get('dc', 1.0)) * max(((M.get('minimax_priorities') or {}).get(n, 1) for n in v.get('net_list', [])), default=1)
            part_list.append(dict(net_name='channel:' + gr, criterion='dc', x=x, weight=g, amount=g * max(0.0, x - 1)))
    part_list.sort(key=lambda t: (-t['amount'], t['net_name'], t['criterion']))

    version = version_contract(G, profile_data, M.get('version'), M.get('minimax_board_contract'))
    safe = safety(M)
    M.update(minimax=max((t['amount'] for t in part_list), default=0.0), minimax_version=version,
             minimax_profile=profile_data, minimax_rank=[t['amount'] for t in part_list], minimax_parts=part_list,
             chokepoint=max(part_list, key=lambda t: (t['amount'], t['weight'] * t['x'])) if part_list else None, minimax_total_sum=M['score_v3'], safety=safe,
             minimax_complete=not M.get('minimax_missing') and not safe['absent'],
             minimax_sources={n: dict(load=v['m'].get('load_contract'), measurement=v['m'].get('dV_source'))
                             for n, v in sorted(M['net_list'].items()) if v['m'].get('carries')})
    return M

def standard_contract(G):
    norm = dict(thresholds=G['minimax']['thresholds'],
                netclasses={k: dict(s=v.get('s'), tr_ns=v.get('tr_ns')) for k, v in G['netclasses'].items()})
    return hashlib.sha256(json.dumps(norm, sort_keys=True, allow_nan=False).encode()).hexdigest()


def version_contract(G, profile_data, measurement_version='v3.1', board_contract=None):
    contract_spec = dict(version=VERSION, board=board_contract, measurement_version=measurement_version, config=G['minimax'], weights=G['profile'][profile_data]['weights'], netclasses=G['netclasses'], total_sum={k: G.get(k) for k in ('bend', 'bend_per', 'w_rank', 'group_list')})
    digest = hashlib.sha256(json.dumps(contract_spec, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
    return VERSION + ':' + digest


def current(e):
    from analysis.measures import load_weights
    p = e.get('minimax_profile')
    G = load_weights(si=e.get('score_v3_version', e.get('version')) == 'v3.2')
    return p in G.get('profile', {}) and e.get('minimax_version') == version_contract(G, p, G.get('version', 'v3.1'), e.get('minimax_board_contract'))


def rank(E):
    v = list(E['minimax_rank'])
    while v and v[-1] == 0: v.pop()
    return tuple(v), E['minimax_total_sum']


def complete(e):
    rr = e.get('minimax_rank')
    return bool(e.get('minimax_complete') and isinstance(rr, list) and
                all(_count(v) for v in rr) and _count(e.get('minimax_total_sum')) and
                _count(e.get('minimax')) and isinstance(e.get('minimax_sources'), dict) and
                all(isinstance(v, dict) and isinstance(v.get('load'), str) and re.fullmatch('[0-9a-f]{64}', v['load'])
                    and isinstance(v.get('measurement'), str) for v in e['minimax_sources'].values()))


def comparison(a, b):
    if not a.get('minimax_version') or a.get('minimax_version') != b.get('minimax_version') or a.get('minimax_profile') != b.get('minimax_profile'):
        return dict(better=False, delta=None, reason='different/missing minimax versions or profiles')
    if 'minimax_sources' not in a or 'minimax_sources' not in b:
        return dict(better=False, delta=None, reason='Minimax current models/IR sources missing')
    if a.get('minimax_sources') != b.get('minimax_sources'):
        return dict(better=False, delta=None, reason='different minimax current models/IR sources')
    if not complete(a) or not complete(b):
        return dict(better=False, delta=None, reason='Minimax measured quantities missing')

    ar, br = a['minimax_rank'], b['minimax_rank']
    for i in range(max(len(ar), len(br))):
        x, y = (ar[i] if i < len(ar) else 0), (br[i] if i < len(br) else 0)
        if x != y: return dict(better=y < x, delta=y-x, rank_site=i, reason='Minimax rank position %d: %g -> %g' % (i+1, x, y))
    d = b['minimax_total_sum'] - a['minimax_total_sum']
    return dict(better=d < 0, delta=d, rank_site=None, reason='Minimax equal, sum %+.3f' % d)


def assumption(a, b, tol_detour_mm=1.0, tol_loss=0.02):
    from analysis import rule_metrics as RK
    c = comparison(a, b)
    out = dict(c, not_worse=False)
    if c['delta'] is None: return out
    if not (current(a) and current(b)):
        return dict(out, better=False, reason='outdated minimax version')
    ga = a.get('minimax_valid', (a.get('safety') or {}).get('safe'))
    gb = b.get('minimax_valid', (b.get('safety') or {}).get('safe'))
    if not gb:
        return dict(out, better=False, reason='Safety gate failed')
    gate_check = RK.assumption(a.get('design_rules') or a.get('rule_gate'), b.get('design_rules') or b.get('rule_gate'))
    out['rule_gate'] = gate_check
    if not gate_check['not_worse']: return dict(out, better=False, reason=gate_check['reason'])
    sa, sb = a.get('minimax_current_paths') or {}, b.get('minimax_current_paths') or {}
    if not all(_count(s.get(k)) for s in (sa, sb) for k in ('detour_mm', 'loss_mW')):
        return dict(out, better=False, reason='Minimax current path protection missing')
    du = sb['detour_mm'] - sa['detour_mm']; dp = sb['loss_mW'] - sa['loss_mW']
    worse = du > tol_detour_mm and dp > tol_loss * max(1e-9, sa['loss_mW'])
    out.update(detour_current_mm=du, loss_current_mW=dp, current_paths_worse=worse)
    if worse: return dict(out, better=False, reason='Current paths worsened')
    out['not_worse'] = not ga or c['delta'] <= 0
    out['better'] = not ga or c['better']
    return out


def off_row(z):
    r = z.get('rule') or {}
    return r if 'minimax_version' in r else z


def reward(z):
    e = off_row(z)
    if not current(e) or not complete(e): return None
    if z.get('is_valid') is None or e.get('minimax_status') == 'unknown': return None
    if not z.get('is_valid') or not e.get('safety', {}).get('safe'): return 2.0
    v = e.get('minimax')
    return v / (1.0 + v) if _count(v) else None
