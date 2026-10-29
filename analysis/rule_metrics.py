import json, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VERSION = 'design_rules-v1'
SI_VERSIONS = ('v3.2', 'v3.2+r35-si2-option')
PARTS_V3 = dict(power_loss=('dc', 'j', 'power'), crosstalk=('xt_victim', 'xt_disturb', 'ground'), return_path=('ret',),
                switch_node=('sw_area', 'l_load', 'hot_L', 'hot_A'), signal=('total_length', 'c_load', 'vias', 'detour'))
PARTS_V3_SI = dict(PARTS_V3, crosstalk=PARTS_V3['crosstalk'] + ('disturb',), eye=('eye_v', 'eye_h', 'eye_skew'))
GATE = ('score_v3', 'power_loss', 'crosstalk', 'return_path', 'topology', 'coupling', 'rule_checker')
GATE_SI = GATE + ('eye', 'eye_vertical_loss', 'eye_horizontal_loss', 'noise_budget', 'noise_budget_max')
BESIDE = ('switch_node', 'signal', 'bends', 'mini', 'vias_unnecessary', 'parallel_mm', 'via_density_sharp', 'via_density_window_over', 'current_bottlenecks_mm2')

TOL_ABS = dict(eye=0.01, eye_vertical_loss=0.01, eye_horizontal_loss=0.01, noise_budget=0.01, noise_budget_max=0.01,
               score_v3=0.5, power_loss=0.5, crosstalk=0.5, return_path=0.5, switch_node=0.5, signal=0.5, topology=1.0,
               rule_checker=1.0, bends=2.0, mini=2.0, coupling=0.5, vias_unnecessary=1.0, parallel_mm=1.0)


def _gnd_layers(gl):
    if gl is None: return None
    if isinstance(gl, str): gl = [x for x in gl.split(',') if x]
    return [str(x).split('=')[0] for x in gl]


def v3_parts(M):
    part_list = PARTS_V3_SI if M.get('version') in SI_VERSIONS else PARTS_V3
    out = {k: 0.0 for k in part_list}; rest = 0.0
    wo = {g: k for k, gs in part_list.items() for g in gs}
    for v in M['net_list'].values():
        W = v.get('weight') or 1.0
        for g, t in (v.get('part_list') or {}).items():
            if g in wo: out[wo[g]] += W * t
            else: rest += W * t
    out['power_loss'] += sum(v for k, v in (M.get('board_parts') or {}).items() if k != 'layer_stack')
    rest += (M.get('board_parts') or {}).get('layer_stack', 0)
    if M.get('si_score_active'): out['signal'] += M['signal_integrity']['score_extra']
    if abs(rest) > 1e-9: out['otherwise'] = rest
    return {k: round(v, 3) for k, v in out.items()}


def _rule_checker(E, with_area):
    dec = E['decoupling']['group_list']
    z = dict(decoupling_spacing=sum(g['over_nominal'] for n, g in dec.items() if n != 'out_port'),
             decoupling_layer_change=(dec.get('small') or {}).get('with_layer_change', 0),
             link_chain=sum(v for u, v in E['link_chain']['verdicts'].items() if u in ('via_before_c', 'branch')),
             crystal=E['crystal']['over_nominal'], min_width=E['min_width']['net_list'],
             vias_unnecessary=E['vias_unnecessary']['vias'], chain_list=E['link_chain']['count'],
             decoupling_parts=sum(g['count'] for n, g in dec.items() if n != 'out_port'))
    mv = E['ground_vias']['group_list']
    z['ground_stub'] = sum(g['stub_over_nominal'] for n, g in mv.items() if n in ('decoupling', 'chip')) if with_area else 0
    z['ground_without_via'] = sum(g['without_via'] for n, g in mv.items() if n in ('decoupling', 'chip')) if with_area else 0
    z['ground_pad_missing'] = sum(p['missing'] for p in E['ground_pad']['pads']) if with_area else 0
    z['total_sum'] = sum(z[k] for k in ('decoupling_spacing', 'decoupling_layer_change', 'link_chain', 'crystal', 'min_width',
                                    'ground_stub', 'ground_without_via', 'ground_pad_missing'))
    return z


def _r36_diagnose(out, B, R, K):
    try:
        from analysis.r36 import measure
        out['r36'] = measure(B, R, K)
    except Exception as e:
        out['r36'] = dict(version='r36-shaping-v3', status='measurement_error')
        out['diagnose_error'] = dict(out.get('diagnose_error') or {}, r36=repr(e)[:200])


def _r18_diagnose(out, B, R, K):
    try:
        from analysis.r18 import measure
        out['r18'] = measure(B, R, K)
    except Exception as e:
        out.setdefault('diagnose_error', {})['r18'] = repr(e)[:200]


def _r41_diagnose(out, B, R, K):
    flag = (R.get('options') or {}).get('r41_measure', (K.get('specs') or {}).get('r41_measure', False))
    if not (flag is True or flag == '1'): return
    try:
        from analysis.r41 import measure as r41_measure
        out['r41'] = r41_measure(B, R, K)
        out['r41_error_mm'] = out['r41']['error_mm']
    except Exception as e:
        out['r41_error_mm'] = None
        out.setdefault('diagnose_error', {})['r41'] = repr(e)[:200]


def r19_measure(B, R, K):
    from analysis.wobble import measure as wobble, chain_list
    from analysis.r19 import outliers, bend_excess45
    signals = {n for n, k in K['net_list'].items() if k.get('category') in ('signal', 'analog', 'fast')}
    rs = dict(R, tracks=[t for t in R.get('tracks', []) if t['net'] in signals],
              vias=[v for v in R.get('vias', []) if v['net'] in signals and not v.get('current_via')
                    and not v.get('ret_via') and not v.get('rv')])
    chains = chain_list(rs)
    return dict(wobble(B, rs, gnd=K.get('gnd') or [], chains=chains), version='r19-signal-v2', outliers=outliers(B, rs, chains=chains),
                **bend_excess45(rs, chains=chains),
                hint='Detour versus pad MST is a diagnosis, not proof of a free alternative; zones not counted.')


def measure(B, R, K, gnd_layers=None, area_nets=None, measures_result=None, minimax=None, si_score=False, si_measure=False, topology_version=None, r27_diagnose=True, r44_result=None):


    from analysis.r44 import measure as r44_measure
    r44 = r44_result if r44_result is not None else r44_measure(B, R)
    if (K.get('specs') or {}).get('physics_version') == 'v3.3-contract_spec-option':
        from analysis.rule_metrics_physics import measure as physics
        out = physics(B, R, K, gnd_layers=gnd_layers, area_nets=area_nets,
                      measures_result=measures_result, minimax=minimax, si_score=si_score,
                      si_measure=si_measure, topology_version=topology_version)
        out['r44'] = r44
        _r41_diagnose(out, B, R, K)
        _r40_diagnose(out, R, K)
        _via_diagnose(out, B, R)
        _r36_diagnose(out, B, R, K)
        _r24_diagnose(out, B, R, K)
        _r18_diagnose(out, B, R, K)


        try:
            out['r19'] = r19_measure(B, R, K)
        except Exception as e:
            out['diagnose_error'] = dict(out.get('diagnose_error') or {}, r19=repr(e)[:200])
        _p26(B, K, out)
        _k20_diagnose(out, B, R, K)
        _k17_diagnose(out, B, R, K)
        return out
    t0 = time.time(); c0 = time.process_time()
    R = dict(R)
    if gnd_layers is not None:
        R['gnd_layers'] = list(gnd_layers) if not isinstance(gnd_layers, str) else [x for x in gnd_layers.split(',') if x]
        from analysis.areas import active
        if active(R, K): R['options'] = dict(R.get('options') or {}, gnd_layers=','.join(R['gnd_layers']))
    if area_nets is not None: R['area_nets'] = sorted(area_nets)
    R.setdefault('conflicts', {}); R.setdefault('failed', [])
    from analysis.areas import active, metric
    la_active = active(R, K)
    la = metric(B, R, K) if la_active else None
    cu_layers = la['gnd_copper_layers'] if la_active else (_gnd_layers(R.get('gnd_layers')) or [])
    out = dict(version=VERSION, gnd_layers=cu_layers, area_nets=sorted(R.get('area_nets') or []), r44=r44)
    _r18_diagnose(out, B, R, K)
    faults = {}
    _p26(B, K, out)
    try:
        from analysis.measures import measures
        M = measures_result if measures_result is not None else measures(B, R, K, gnd_layers=gnd_layers if la_active else R.get('gnd_layers', cu_layers), minimax=minimax, si_score=si_score, si_measure=si_measure)
        if 'minimax_version' in M:
            out.update({k: M[k] for k in ('minimax', 'minimax_version', 'minimax_profile', 'minimax_rank', 'minimax_total_sum',
                                          'minimax_complete', 'minimax_sources', 'chokepoint', 'safety')})
            out['minimax_missing'] = M.get('minimax_missing')
        if 'signal_integrity' in M and bool(M.get('si_score_active')) != bool(si_score):
            raise ValueError('measures_result: differing si_score option')
        T = v3_parts(M)
        from analysis.r11 import measure as r11_measure
        out['r11'] = r11_measure(K, M, B, R)
        out.update(score_v3_version=M.get('version'), score_v3=round(M['score_v3'], 3), loss_mW=M.get('loss_mW'), hot_loops=len(M.get('hot_loop') or []),
                   ground_max_uV=(M.get('ground') or {}).get('max_uV'), **T)
        if 'signal_integrity' in M:
            out.update(signal_integrity=M['signal_integrity'], si_score_active=M['si_score_active'], score_v3_basis=M['score_v3_basis'])
        if M.get('version') in SI_VERSIONS:
            from analysis.signal_integrity import short
            out.update(version='design_rules-v2-si', score_v3_version=M['version'], **short(M))
            out['noise_budget'] = round(sum(v['x'] for v in (M.get('analog_disturb') or {}).values()), 6)
            out['noise_budget_max'] = max((v['x'] for v in (M.get('analog_disturb') or {}).values()), default=0.0)
            out['eye_vertical_loss'] = 100 - out['eyes_vertical_pct'] if out['eyes_vertical_pct'] is not None else 0.0
            out['eye_horizontal_loss'] = 100 - out['eyes_horizontal_pct'] if out['eyes_horizontal_pct'] is not None else 0.0

        out['v3_nets'] = [[n, round(v['contribution'], 3), v['net_class']] for n, v in sorted(M['net_list'].items(), key=lambda kv: (-kv[1]['contribution'], kv[0]))[:5]]
    except Exception as e:
        faults['score_v3'] = repr(e)[:200]
    try:
        from analysis.metrics import metrics
        k = metrics(R)
        out.update(bends=k['bends'], mini=k['mini'], share_45=k['share_45'], segments=k['segments'], fast_joints=k['fast_joints'],
                   width_change=k['width_change'], vias=k['vias'], signal_vias=k['signal_vias'], total_length=k['total_length'])
    except Exception as e:
        faults['metrics'] = repr(e)[:200]
    try:
        out['r19'] = r19_measure(B, R, K)
    except Exception as e:

        out['diagnose_error'] = dict(out.get('diagnose_error') or {}, r19=repr(e)[:200])
    try:
        from analysis.topology import topology
        tp = topology(R, B, **({'version': topology_version} if topology_version is not None else {}))
        if 'version' in tp: out['topology_version'] = tp['version']
        top = dict(star_free=tp['star'] - tp['star_pad'], sharp_free=tp['sharp'] - tp['sharp_pad'], star_pad=tp['star_pad'],
                   sharp_pad=tp['sharp_pad'], nodes=tp['nodes'], parallel_rest_mm=tp['parallel_rest_mm'],
                   parallel_chain_mm=tp['parallel_chain_mm'], parallel_tap_mm=tp['parallel_tap_mm'], tap_close=tp['tap_close'],
                   tap_beside_current_mm=tp['tap_beside_current_mm'], tap_via_in_pad=tp['tap_via_in_pad'])
        out['topology_parts'] = top
        if 'parallel_r27_mm' in tp: top['parallel_r27_mm'] = tp['parallel_r27_mm']
        out['topology'] = round(top['star_free'] + top['sharp_free'] + top['tap_close'] + top.get('parallel_r27_mm', top['parallel_rest_mm']), 1)
        out['parallel_mm'] = round(top['parallel_rest_mm'] + top['parallel_chain_mm'] + top['parallel_tap_mm'], 1)
    except Exception as e:
        faults['topology'] = repr(e)[:200]


    if r27_diagnose:
        try:
            from analysis.topology_trunk import measure as trunk_measure
            out['r27'] = trunk_measure(R, B)
        except Exception as e:
            out['diagnose_error'] = dict(out.get('diagnose_error') or {}, r27=repr(e)[:200])
    try:
        from analysis.coupling import coupling
        out['coupling'] = coupling(R, K)['total_sum']
    except Exception as e:
        faults['coupling'] = repr(e)[:200]
    try:
        from analysis.rule_check import check
        from common.zones import active
        if R.get('copper_bars') and not active(R):
            from analysis.copper_model import replacement_routes
            R = replacement_routes(B, R, K)
        rule_report = check(B, K, R)
        rp = _rule_checker(rule_report, bool(cu_layers))
        if 'loops' in rule_report['decoupling']:
            out['decoupling_loops'] = rule_report['decoupling']['loops']
        if 'loops_error' in rule_report['decoupling']:
            out.setdefault('diagnose_error', {})['p3'] = rule_report['decoupling']['loops_error']
        if la_active:
            rp['layer_stack'] = la['total_sum'] if la['gate_active'] else 0
            rp['total_sum'] += rp['layer_stack']
            out['layer_stack'] = la
        out['rule_checker_parts'] = rp; out['rule_checker'] = rp['total_sum']; out['vias_unnecessary'] = rp['vias_unnecessary']
    except Exception as e:
        faults['rule_checker'] = repr(e)[:200]
    r39_measure(B, R, K, gnd_layers, out, faults)
    _r41_diagnose(out, B, R, K)
    _r40_diagnose(out, R, K)
    _via_diagnose(out, B, R)
    _r36_diagnose(out, B, R, K)
    _r24_diagnose(out, B, R, K)
    _k20_diagnose(out, B, R, K)
    _k17_diagnose(out, B, R, K)
    if faults: out['faults'] = faults
    out['time_s'] = round(time.time() - t0, 2); out['cpu_s'] = round(time.process_time() - c0, 2)
    if 'M' in locals() and 'score_v3' not in faults:
        from learning.pair_contract import measurement_signature, board_id
        out.update(pair_measurement_version=measurement_signature(), board_id=board_id(B))
    return out


def _k17_diagnose(out, B, R, K):
    flag = (R.get('options') or {}).get('k17_measure',
            (K.get('specs') or {}).get('k17_measure', R.get('r36_k17', False)))
    if str(flag).lower() not in ('1', 'true'):
        return
    try:
        from analysis.k17 import measure
        out['k17'] = measure(B, R, K)
    except Exception as e:
        out['diagnose_error'] = dict(out.get('diagnose_error') or {}, k17=repr(e)[:200])


def _k20_diagnose(out, B, R, K):
    flag = (R.get('options') or {}).get('k20_measure', (K.get('specs') or {}).get('k20_measure', False))
    if str(flag).lower() in ('1', 'true'):
        from analysis.k20.metric import diagnose as k20_diagnose
        k20_diagnose(out, B, R, K)


def r39_measure(B, R, K, gnd_layers, out, faults):
    out['r39_gate_active'] = str((R.get('options') or {}).get('r39_gate', '0')) == '1'
    opt = R.get('options') or {}
    if not out['r39_gate_active'] and str(opt.get('r39_diagnose', opt.get('r39', '0'))) != '1': return
    try:
        from analysis.current_bottlenecks import bottlenecks
        r39 = bottlenecks(B, R, K, gnd_layers)
        out.update(current_bottlenecks=r39, current_bottlenecks_mm2=r39['unnecessary_mm2'] if not r39['not_measurable'] and r39['coverage']['components'] else None,
                   current_bottlenecks_n=r39['unnecessary_n'] if not r39['not_measurable'] and r39['coverage']['components'] else None,
                   r39_version=r39['version'], r39_parameter=r39['parameter'], r39_coverage=r39['coverage'])
        if r39['not_measurable']:
            out['r39_error'] = 'Zone components or side branches without routed centerline cannot be fully measured'
            if out['r39_gate_active']: faults['current_bottlenecks'] = out['r39_error']
    except Exception as e:
        out['r39_coverage'] = dict(complete=False, unknown=1, share=0.0)
        out['r39_error'] = repr(e)[:200]
        if out['r39_gate_active']: faults['current_bottlenecks'] = out['r39_error']
def _r40_diagnose(out, R, K):
    try:
        from analysis.via_density import short
        out['r40'] = short(R, K)
    except Exception as e:
        out['r40'] = dict(version='r40-short-v1', status='measurement_error')
        out.setdefault('diagnose_error', {})['r40'] = repr(e)[:200]


def _via_diagnose(out, B, R):
    opt = R.get('options') or {}
    flag = opt.get('via_density_diagnose', opt.get('via_density', '0'))
    if str(flag).lower() not in ('1', 'true'): return
    try:
        from analysis.via_density import measure as via_measure, THRESHOLD, WINDOW_MM
        vd = via_measure(B, R, threshold=float(opt.get('via_density_threshold', THRESHOLD)),
                       window=float(opt.get('via_density_window', WINDOW_MM)))
        out.update(via_density=vd, via_density_sharp=vd['sharp_vias_mm2'], via_density_window_over=vd['window_over'])
    except Exception as e:
        out.setdefault('diagnose_error', {})['via_density'] = repr(e)[:200]


def _r24_diagnose(out, B, R, K):
    try:
        from postprocessing.r24 import measure
        search = str((R.get('options') or {}).get('r24_diagnose', '0')).lower() in ('1', 'true')
        info = measure(B, R, K, search=search)
        info.pop('sites', None)
        out['r24'] = info
    except Exception as e:
        out.setdefault('diagnose_error', {})['r24'] = repr(e)[:200]


def _p26(B, K, out):
    if not B.get('fps'): return
    try:
        from analysis.density import diagnose
        out['p26'] = dict(diagnose(B, K, K.get('design_rules')), gate_active=False)
        from analysis.p26_group_paths import measure
        out['p26']['group_paths'] = measure(B, K)
    except Exception as e:
        out['p26'] = dict(gate_active=False, faults=repr(e)[:200])


def amount(m, k):
    if not m: return None
    v = m.get(k)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def r27_shape_gate(B, before, after, K, gnd_layers=None):
    measurements = [measure(B, r, K, gnd_layers=gnd_layers) for r in (before, after)]
    a, b = measurements
    keys = tuple(dict.fromkeys(gate_fields(a) + gate_fields(b)))
    missing = [k for k in keys if amount(a, k) is None or amount(b, k) is None]
    worse = [k for k in keys if k not in missing and amount(b, k) > amount(a, k) + 1e-8]
    r27 = []
    for m in measurements:
        r = m.get('r27') or {}
        complete = all(amount(r, k) is not None and amount(r, k) >= 0 for k in (
                            'star', 'star_pad', 'sharp', 'sharp_pad', 'parallel_r27_mm'))
        complete = complete and r['star_pad'] <= r['star'] and r['sharp_pad'] <= r['sharp']
        r27.append(dict(star_free=r['star']-r['star_pad'], sharp_free=r['sharp']-r['sharp_pad'],
                        parallel_r27_mm=r['parallel_r27_mm']) if complete else {})
    for k in ('star_free', 'sharp_free', 'parallel_r27_mm'):
        if any(amount(m, k) is None for m in r27): missing.append('r27.'+k)
        elif r27[1][k] > r27[0][k] + 1e-8: worse.append('r27.'+k)
    versions = [m.get('r27', {}).get('version') for m in measurements]
    if versions[0] != versions[1] or not versions[0]: missing.append('r27.version')
    if any(m.get('faults') for m in measurements): missing.append('evaluation_error')
    return dict(version='r27-shape_gate-v1', taken=not missing and not worse,
                worse=worse, missing=missing, r27_version=versions[0],
                before=dict(design_rules={k: a.get(k) for k in keys}, r27=r27[0]),
                after=dict(design_rules={k: b.get(k) for k in keys}, r27=r27[1]))


def gate_fields(m):
    m = m or {}
    keys = GATE_SI if m.get('score_v3_version', m.get('version')) in SI_VERSIONS else GATE
    return keys + ('current_bottlenecks_mm2',) if m.get('r39_gate_active') else keys


def totals(ms, keys=None):
    ms = list(ms)
    if keys is None: keys = tuple(dict.fromkeys(GATE + tuple(k for m in ms for k in gate_fields(m)))) + BESIDE
    out = {}
    for k in keys:
        w = [amount(m, k) for m in ms]; w = [x for x in w if x is not None]
        out[k] = round(sum(w), 3) if w else None
    return out


TOL_REL = 0.05



NI_LIMITS = {k: dict(total_sum=0.0, single=0.0) for k in GATE_SI + ('current_bottlenecks_mm2',)}
ALPHA = 0.10
GEO = ('score_v3', 'power_loss', 'crosstalk', 'return_path', 'switch_node', 'signal', 'coupling')


def worse(a, b, k, tol_rel=TOL_REL):
    if a is None or b is None: return 0
    t = max(tol_rel * abs(a), TOL_ABS.get(k, 0.5))
    return 1 if b > a + t else (-1 if b < a - t else 0)


def p_at_least(w, n):
    if n <= 0 or w <= 0: return 1.0
    return sum(math.comb(n, i) for i in range(w, n + 1)) / 2.0 ** n


def comparison(A, Bv, tol_rel=TOL_REL, keys=None, alpha=ALPHA):
    names = sorted(n for n in set(A) & set(Bv) if A[n] and Bv[n])
    all_items = sorted(set(A) | set(Bv))
    gates = {n: tuple(dict.fromkeys(gate_fields(A.get(n)) + gate_fields(Bv.get(n)))) for n in all_items}
    if keys is None: keys = tuple(dict.fromkeys(GATE + tuple(k for n in all_items for k in gates[n]))) + BESIDE
    gate_check = set(k for fs in gates.values() for k in fs)
    absent = [dict(board=n, size=k, board_side=board_side) for board_side, data in (('baseline', A), ('new', Bv))
               for n in all_items for k in keys if k in gates[n] and amount(data.get(n), k) is None]
    absent += [dict(board=n, size='EvaluationContract', board_side='both') for n in names
                if gate_fields(A[n]) != gate_fields(Bv[n])]
    absent += [dict(board=n, size='TopologyVersion', board_side='both',
                     baseline=A[n].get('topology_version', 'topology-v1'),
                     new=Bv[n].get('topology_version', 'topology-v1')) for n in names
                if A[n].get('topology_version', 'topology-v1') != Bv[n].get('topology_version', 'topology-v1')]
    absent += [dict(board=n, size='R39-MeasurementVersion', board_side='both',
                     baseline=A[n].get('r39_version'), new=Bv[n].get('r39_version')) for n in names
                if 'current_bottlenecks_mm2' in gates[n] and (not A[n].get('r39_version') or
                                                        A[n].get('r39_version') != Bv[n].get('r39_version'))]
    absent += [dict(board=n, size='R39-MeasurementParameters',
                     baseline=A[n].get('r39_parameter'), new=Bv[n].get('r39_parameter')) for n in names
                if 'current_bottlenecks_mm2' in gates[n] and (not A[n].get('r39_parameter') or
                     A[n].get('r39_parameter') != Bv[n].get('r39_parameter'))]
    out = dict(boards=names, per={}, tol_rel=tol_rel, alpha=alpha, complete=bool(names) and not absent,
               absent=absent)
    for k in keys:
        pa = [(amount(A[n], k), amount(Bv[n], k)) for n in names if k not in gate_check or k in gates[n]]
        pa = [(x, y) for x, y in pa if x is not None and y is not None]
        if not pa: out['per'][k] = None; continue
        sa, sb = sum(x for x, _ in pa), sum(y for _, y in pa)
        per = [worse(x, y, k, tol_rel) for x, y in pa]
        w, g = sum(1 for s in per if s > 0), sum(1 for s in per if s < 0)
        v = dict(n=len(pa), baseline=round(sa, 3), new=round(sb, 3), delta=round(sb - sa, 3), rel=round((sb - sa) / abs(sa), 4) if abs(sa) > 1e-9 else None,
                 better=g, worse=w, equal=len(pa) - w - g, p_worse=round(p_at_least(w, w + g), 4), p_better=round(p_at_least(g, w + g), 4),
                 gate_check=k in gate_check)
        if k in GEO:
            z = TOL_ABS.get(k, 0.5)
            q = [math.log((y + z) / (x + z)) for x, y in pa if x + z > 0 and y + z > 0]
            v['geo'] = round(math.exp(sum(q) / len(q)), 4) if q else 1.0
            direction = 1 if v['geo'] > 1 else (-1 if v['geo'] < 1 else 0)
        else:
            direction = 1 if sb > sa else (-1 if sb < sa else 0)
        v['verdict'] = 1 if (v['p_worse'] < alpha and w > g and direction > 0) else (-1 if (v['p_better'] < alpha and g > w and direction < 0) else 0)
        v['statistics_verdict'] = v['verdict']
        if k in gate_check:
            gr = NI_LIMITS[k]
            delta_max = max(y - x for x, y in pa)
            ni = sb - sa <= gr['total_sum'] and delta_max <= gr['single']
            v.update(not_worse=ni, limits=dict(gr), single_max=delta_max)

            v['verdict'] = 1 if not ni else (-1 if sb < sa else 0)
        out['per'][k] = v
    return out


def assumption(a, b):
    v = comparison({'board': a}, {'board': b})
    bad = [k for k, t in v['per'].items() if t and t['gate_check'] and t['verdict'] > 0]
    good = [k for k, t in v['per'].items() if t and t['gate_check'] and t['verdict'] < 0]
    return dict(better=v['complete'] and not bad and bool(good),
                not_worse=v['complete'] and not bad,
                reason='Rule metrics missing' if not v['complete'] else
                      ('Rule regression: ' + ', '.join(bad) if bad else ('Rules better' if good else 'Rules equal')),
                comparison=v)




def copper_map(C, S=None, clr=0.15, tol=0.03, border=0.5):
    from shapely.geometry import Polygon, LineString, Point
    from shapely.ops import unary_union
    def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
    board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-border)
    cu_layers = sorted({t['layer'] for t in C.get('tracks', [])} | {z['layer'] for z in C.get('zones', [])} |
                   {L for p in C.get('pads', []) for L in (p.get('polys') or {})}, key=lambda L: (L != 'F', L == 'B', L))
    out = {}
    for L in cu_layers:
        cu = []
        for t in C.get('tracks', []):
            if t['layer'] == L: cu.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
        for v in C.get('vias', []): cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
        for p in C.get('pads', []):
            cu += [poly(q) for q in (p.get('polys') or {}).get(L, [])]
        pieces = {}
        for z in C.get('zones', []):
            if z['layer'] != L: continue
            ps = [poly(q) for q in z['polys']]; cu += ps
            pieces.setdefault(z.get('net') or '', []).extend(ps)
        for f in C.get('shapes', []):
            if f['layer'] == L: cu += [poly(q) for q in f['polys']]
        copper_ = unary_union(cu).intersection(board)
        blocked = unary_union([Polygon(s['pts']).buffer(0) for s in (S or []) if L in s['cu_layers'] and s.get('fill')]) if S else None
        white = inner.difference(copper_.buffer(clr + tol))
        if blocked is not None and not blocked.is_empty: white = white.difference(blocked.buffer(clr))
        part_list = [g for g in getattr(white, 'geoms', [white]) if g.area > 0.002]
        zs = {n: len([g for x in ps for g in getattr(unary_union([x]), 'geoms', [x]) if g.area > 1e-4]) for n, ps in sorted(pieces.items())}
        out[L] = dict(copper_share=round(copper_.area / board.area, 4) if board.area else None, white_sites=len(part_list),
                      white_mm2=round(sum(g.area for g in part_list), 2), zones_pieces=zs)
    return dict(cu_layers=out, copper_share=round(sum(v['copper_share'] or 0 for v in out.values()) / max(1, len(out)), 4),
                white_sites=sum(v['white_sites'] for v in out.values()), white_mm2=round(sum(v['white_mm2'] for v in out.values()), 2))


def text(m):
    if not m: return '-'
    f = lambda k, fm='%.1f': ('-' if m.get(k) is None else fm % m[k])
    return 'v3 %s (loss %s, crosstalk %s, return path %s, switching node %s, signal %s) | topology %s | rule checker %s | bends %s | coupling %s' % (
        f('score_v3'), f('power_loss'), f('crosstalk'), f('return_path'), f('switch_node'), f('signal'), f('topology'),
        f('rule_checker', '%d'), f('bends', '%d'), f('coupling'))


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    if a[:1] == ['copper_map']:
        C = json.load(open(a[1])); S = json.load(open(a[2])) if len(a) > 2 else None
        print(json.dumps(copper_map(C, S, float(kw.get('clr', '0.15')), float(kw.get('tol', '0.03')), float(kw.get('border', '0.5'))), indent=1))
        sys.exit(0)
    if len(a) != 3: sys.exit(__doc__)
    B, R, K = (json.load(open(x)) for x in a)
    for name in ('r41_measure', 'k20_measure', 'k17_measure'):
        if name in kw:
            R = dict(R, options=dict(R.get('options') or {}, **{name: kw[name]}))
    vd_opt = {k: v for k, v in kw.items() if k in ('via_density_diagnose', 'via_density_window', 'via_density_threshold', 'r24_diagnose')}
    if vd_opt: R = dict(R, options=dict(R.get('options') or {}, **vd_opt))
    if 'layer_stack' in kw:
        K = dict(K, specs=dict(K.get('specs') or {}, layer_stack=kw['layer_stack'] == '1'))
    if 'layer_stack_heuristic' in kw:
        K = dict(K, specs=dict(K.get('specs') or {}, layer_stack_heuristic=kw['layer_stack_heuristic'] == '1'))
    fl = kw['area_nets'].split(',') if 'area_nets' in kw else None
    m = measure(B, R, K, gnd_layers=kw.get('gnd_layers'), area_nets=fl, si_score=kw.get('si_score') == '1', si_measure=kw.get('si_measure') == '1', topology_version=kw.get('topology_version'))
    print(text(m)); print(json.dumps(m, indent=1))
