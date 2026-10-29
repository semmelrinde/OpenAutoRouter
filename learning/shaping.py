import json, os, shutil, sys, time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VPY = os.path.join(R, '.venv', 'bin', 'python')


def is_valid(B, K, rj, gnd_layers, flow_cache=None, routes=None, minimax=None):
    sys.path.insert(0, R)
    from analysis import corpus as KO
    RR = KO.normalized_route(B, K, routes if routes is not None else json.load(open(rj)), gnd_layers) if routes is not None or (rj and os.path.exists(rj)) else None
    E = KO.rate_who(B, K, rj, gnd_layers=gnd_layers, flow_cache=flow_cache, routes=RR) if RR is not None else None
    from analysis.measures import review_active
    if E and gnd_layers and not review_active(K):
        from analysis.evaluation import rate, not_connected
        E['not_connected'] = len(not_connected(B, RR, set(K.get('gnd') or []), flow_cache=flow_cache, K=K))
        E['is_valid'] = bool(rate(B, RR, K, minimax=False, flow_cache=flow_cache)['is_valid']) and E['not_connected'] == 0 and E['conflict_list'] == 0 and E['open_items'] == 0
    from analysis.standard import goal
    if E and goal(K.get('specs'), explicit=('minimax' if minimax else 'v3') if minimax is not None else None) == 'minimax':
        from analysis.evaluation import rate
        EE = rate(B, RR if review_active(K) else dict(RR, gnd_layers=list(gnd_layers)), K, v3=True, minimax=True, flow_cache=flow_cache)
        E['valid_routing'] = E['is_valid']
        E['is_valid'] = E['is_valid'] and EE['minimax_valid']
        E['minimax_status'] = EE['minimax_status']
        E['safety'] = EE['safety']
    if E and (K.get('specs') or {}).get('physics_version') == 'v3.3-contract_spec-option':
        from analysis.evaluation import rate
        EE = rate(B, dict(RR, gnd_layers=list(gnd_layers or [])), K, v3=True, flow_cache=flow_cache)
        from analysis.contract_spec import cur_state
        E.update(design_rules=EE['design_rules'], proofs=EE['proofs'], measurement_version=EE['gate_check'].get('measurement_version'), score_v3_version=EE['score_v3_version'])
        E['gate_check'] = cur_state(E)
        E['complete_valid'] = E['gate_check']['complete_valid']
    return E['is_valid'] if E else False, E


def receiver_gate_on(K):
    return bool((K.get('specs') or {}).get('receiver_gate')) or os.environ.get('RECEIVER_GATE') == '1'


def shapes(bj, kj, rj, design_rules, gnd_layers, style='45r', log=None, time_s=120, current_shape=False, copper_map=False, minimax=None, outcome=None, flow_cache=None, board=None, netclasses=None, r27_shape_gate=False, r18=False, r18_final=None):
    if r18:
        if style != '45r': raise ValueError('r18=1 requires shaping=45r')
        start = time.thread_time()
        info = shapes(bj, kj, rj, design_rules, gnd_layers, style, log, time_s, current_shape,
                      copper_map, minimax, outcome, flow_cache, board, netclasses, r27_shape_gate)

        rest = max(0., time_s - info.get('cpu_s', 0.) - (time.thread_time()-start))
        gate = dict(applied=False, cpu_s=0., reason='Base shape not finished')
        if info.get('basis_completed') and info.get('smoothed_sha256'):
            if rest < 2:
                gate['reason'] = 'R18: CPU budget exhausted'
            else:
                from learning.r18_shape import apply

                try:
                    gate = apply(bj, kj, rj, design_rules, gnd_layers, cpu_s=rest/2,
                        raw_sha256=info['raw_sha256'], smoothed_sha256=info['smoothed_sha256'],
                        final=r18_final, minimax=minimax)
                except Exception as exc:


                    gate = dict(applied=False, completed=False, cpu_s=0.,
                                reason='R18 error: ' + repr(exc)[:300])
        info['r18'] = gate; info['cpu_s'] = info.get('cpu_s', 0.) + gate['cpu_s']
        if (gate.get('applied') or gate.get('completed')) and outcome is not None:
            B = board if board is not None else json.load(open(bj))
            K = netclasses if netclasses is not None else json.load(open(kj))
            RR = json.load(open(rj)); g,e = is_valid(B,K,rj,gnd_layers,routes=RR,minimax=minimax)
            outcome.update(is_valid=g,outcome=e,routes=RR)
        return info
    if style not in (None, '', 'raw_data', '45r', 'organic', 'ideal', 'per_net_kind'):
        raise ValueError('unknown shape style: '+str(style))
    shape_start = time.thread_time()
    def current_final(info):
        rest = max(0., time_s - info.get('cpu_s', 0.) - (time.thread_time()-shape_start))
        if rest < 1:
            info['current_shape'] = dict(applied=False, reason='Current shape: CPU budget exhausted')
        else:
            info['current_shape'] = current_post_run(bj, kj, rj, design_rules, minimax=minimax, cpu_s=rest)
            info['cpu_s'] = info.get('cpu_s', 0.) + info['current_shape'].get('compute_slot', {}).get('cpu_s', 0.)
        return info
    if style in (None, '', 'raw_data') or not os.path.exists(rj):
        info = dict(style=style, applied=False, reason='off')
        if (current_shape or copper_map) and os.path.exists(rj):
            current_final(info)
            info['applied'] = info['current_shape']['applied']
            info['reason'] = info['current_shape'].get('reason', '')
        return info
    raw_data = rj[:-5] + '_raw.json'; off = rj[:-5] + '_shape.json'
    from pathlib import Path
    import hashlib
    smoothed = rj[:-5] + '_smoothed.json'
    for q in (raw_data, off, smoothed): Path(q).unlink(missing_ok=True)
    shutil.copyfile(rj, raw_data)
    raw_sha = hashlib.sha256(Path(raw_data).read_bytes()).hexdigest()
    smoothed_sha = None

    opt = ['style=ideal', 'topology_version=topology-v4-symmetric'] if style == 'ideal' else (['style=per_net_kind'] if style == 'per_net_kind' else ['pull=0', 'rmax=0.5'] if style == '45r' else ['rmax=3'])
    one = raw_data
    if style in ('45r', 'per_net_kind'):
        try:
            from postprocessing.smooth45 import smooth
            Bg, Rg = json.load(open(bj)), json.load(open(raw_data))
            keepout_list = None
            if style == 'per_net_kind':
                from common.net_style import organic
                Cgfx = netclasses if netclasses is not None else json.load(open(kj))
                keepout_list = {t['net'] for t in Rg['tracks'] if organic(t['net'], Cgfx)}
            Rg['smooth45'] = smooth(Bg, Rg, keepout_list=keepout_list)
            Rg['r18_raw_sha256'] = raw_sha
            one = smoothed; Path(one).write_text(json.dumps(Rg))
            smoothed_sha = hashlib.sha256(Path(one).read_bytes()).hexdigest()
        except Exception as e:
            one = raw_data; smoothed_sha = None
            Path(smoothed).unlink(missing_ok=True)
            if log: Path(log + '.smooth').write_text(repr(e))
    cmd = [VPY, os.path.join(R, 'postprocessing/pulltight_routes.py'), bj, one, off, 'design_rules=' + design_rules, 'netclasses=' + kj] + opt
    try:
        from common import slots as PL
        r = PL.job(cmd, cpu_s=time_s, log=log, cwd=R, env=dict(os.environ, PYTHONHASHSEED='0'))
    except OSError:
        return dict(style=style, applied=False, reason='time_s')
    if r['cur_state'] != 'ok' or not os.path.exists(off): return dict(style=style, applied=False, reason=r['cur_state'], cpu_s=r['cpu_s'])
    B = board if board is not None else json.load(open(bj))
    K = netclasses if netclasses is not None else json.load(open(kj))
    R0, R1 = json.load(open(raw_data)), json.load(open(off))
    try:
        from postprocessing.smooth45 import via_middle
        R1['via_middle'] = via_middle(R1); json.dump(R1, open(off, 'w'))
    except Exception as e:
        R1['via_middle'] = dict(faults=repr(e)[:160]); json.dump(R1, open(off, 'w'))
    g0, e0 = is_valid(B, K, raw_data, gnd_layers, flow_cache=flow_cache, routes=R0, minimax=minimax); g1, e1 = is_valid(B, K, off, gnd_layers, flow_cache=flow_cache, routes=R1, minimax=minimax)
    try: st = json.load(open(off)).get('organic_post_run', {}).get('statistics', {})
    except ValueError: st = {}
    from analysis.contract_spec import comparison
    an = bool(e0 and e1 and comparison(e0,e1)['not_worse']) if (e0 or {}).get('gate_check') or (e1 or {}).get('gate_check') else g1 or not g0

    info = json.load(open(off)).get('organic_post_run', {})
    if style == 'ideal' and not info.get('rule_gate', {}).get('taken', False):
        an = False



    length_ok = True
    if receiver_gate_on(K):
        from analysis.length_impedance import length_list, receiver_protection, receiver_netclasses
        length_k = receiver_netclasses(K)
        length_ok = receiver_protection(length_list(B, R0, length_k), length_list(B, R1, length_k))
    an = an and length_ok
    r27_gate = None
    if r27_shape_gate and an:
        gate_cpu = time.thread_time()
        try:
            from learning.r27_shape import candidate
            repaired, r27_gate = candidate(B, R0, R1, json.load(open(design_rules)) if design_rules else {})
            if r27_gate['proposed']:
                from analysis.rule_metrics import r27_shape_gate as check_r27
                r27_gate['rule_gate'] = check_r27(B, R1, repaired, K, gnd_layers)
                g2, e2 = is_valid(B, K, None, gnd_layers, flow_cache=flow_cache, routes=repaired, minimax=minimax)
                r27_gate['is_valid'] = g2
                receiver_ok = True
                if receiver_gate_on(K):
                    receiver_ok = receiver_protection(length_list(B, R1, length_k), length_list(B, repaired, length_k))
                r27_gate['receiver_gate'] = receiver_ok
                contract = comparison(e1, e2) if (e1 or {}).get('gate_check') or (e2 or {}).get('gate_check') else dict(not_worse=g2 or not g1)
                r27_gate['comparison'] = contract
                r27_gate['applied'] = bool(contract['not_worse'] and receiver_ok and r27_gate['rule_gate']['taken'])
        except Exception as exc:
            r27_gate = dict(version='r27-shaping-v2', proposed=False, applied=False,
                            reason='R27-MeasurementError', faults=repr(exc)[:200])
        if r27_gate.get('applied'):
            json.dump(repaired, open(off, 'w'))
            R1, e1, g1 = repaired, e2, g2
        r27_gate['cpu_s'] = time.thread_time()-gate_cpu
        r['cpu_s'] += r27_gate['cpu_s']
    if an: shutil.copyfile(off, rj)
    if outcome is not None: outcome.update(is_valid=g1 if an else g0, outcome=e1 if an else e0, routes=R1 if an else R0)
    info = dict(style=style, applied=an, basis_completed=True, raw_sha256=raw_sha, smoothed_sha256=smoothed_sha, valid_raw=g0, valid_shape=g1, arcs=st.get('arcs'), guarded=st.get('guarded'),
                reason='' if an else ('ReceiverGate' if not length_ok else 'RuleGate' if style == 'ideal' else 'Shape would be invalid'), cpu_s=r['cpu_s'],
                receiver_gate=length_ok,
                shape_kicad_native=info.get('shape_kicad_native'), rule_gate=info.get('rule_gate'))
    if r27_gate is not None: info['r27_shape_gate'] = r27_gate
    if current_shape or copper_map: current_final(info)
    return info


def current_post_run(bj, kj, rj, design_rules, minimax=None, cpu_s=180):
    import tempfile
    from pathlib import Path
    from common import slots as PL
    with tempfile.TemporaryDirectory(prefix='autorouter-shaping-worker-') as folder:
        result = Path(folder) / 'info.json'
        original = Path(rj).read_bytes()
        candidate = Path(folder) / 'routes.json'
        candidate.write_bytes(original)
        cmd = [VPY, '-m', 'learning.shaping', bj, kj, str(candidate), design_rules or '', str(result),
               'auto' if minimax is None else str(int(minimax))]
        run = PL.job(cmd, cpu_s=cpu_s, wall_s=10*cpu_s, cwd=R,
                      log=str(Path(folder)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['cur_state'] == 'ok' and result.exists() else dict(
            applied=False, reason='Copper follow-up: ' + run['cur_state'])
        info['compute_slot'] = run
        if info['applied']:
            if Path(rj).read_bytes() != original:
                info.update(applied=False, reason='Route changed during follow-up')
                return info
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.current_shape_', delete=False) as f:
                tmp = f.name
                f.write(candidate.read_bytes())
            try:
                Path(rj[:-5] + '_before_copper.json').write_bytes(original)
                os.replace(tmp, rj)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        return info


def _current_post_run(bj, kj, rj, design_rules, minimax=None):
    import time
    import math
    from learning.copper_bars import extend, check_edges
    from analysis.evaluation import rate
    from analysis.rule_metrics import gate_fields, assumption
    from pathlib import Path
    B, K, Rr = (json.loads(Path(q).read_text()) for q in (bj, kj, rj))
    RG = json.loads(Path(design_rules).read_text()) if design_rules else None



    prepared = dict(Rr, tracks=[])
    for t in Rr['tracks']:
        items = t.get('items')
        same = (items and len(items) == len(t['w']) and len(t['pts']) == len(items) + 1 and
                all(it[0] == 'L' and it[1] == t['pts'][j] and it[2] == t['pts'][j + 1] and
                    it[3] == t['w'][j] for j, it in enumerate(items)))
        if same and K['net_list'].get(t['net'], {}).get('category') in ('current_val', 'supply', 'sw') and \
                not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'current_web')):
            t = {q: v for q, v in t.items() if q != 'items'}
        prepared['tracks'].append(t)
    out, info = extend(B, prepared, K, RG, cpu_s=8.0)
    if not info['applied']:
        return info
    t0 = time.process_time()
    import resource
    child = resource.getrusage(resource.RUSAGE_CHILDREN)
    child0 = child.ru_utime + child.ru_stime
    def analysis_cpu():
        child = resource.getrusage(resource.RUSAGE_CHILDREN)
        return time.process_time()-t0+child.ru_utime+child.ru_stime-child0
    from analysis.copper_model import ground_corridors


    from learning.copper_kicad_native import verify
    import tempfile
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter-current_shape-') as native_dir:
            baseline, out, native_info = verify(native_dir, B, Rr, out, RG)
        info['kicad_native'] = native_info
    except Exception as exc:
        info.update(applied=False, reason='Native check failed: ' + str(exc),
                    evaluation_cpu_s=analysis_cpu())
        return info
    if not native_info['suggestion']['is_valid']:
        info.update(applied=False, reason='Native DRC/refill blocks acceptance',
                    evaluation_cpu_s=analysis_cpu())
        return info
    info['ground_corridors'] = ground_corridors(B, baseline, out, K, RG)
    audit = check_edges(B, out, K, entire=True)
    info['width_check'] = audit
    if audit['levels'] or audit['hard_corners'] or audit.get('pads_not_enclosed',0) or audit.get('missing_current_areas',0) or not info['ground_corridors']['passed']:
        info.update(applied=False, reason='Native edge gate/ground corridor not passed',
                    evaluation_cpu_s=analysis_cpu())
        return info
    if minimax is None:
        from analysis.standard import goal
        minimax = goal(K.get('specs')) == 'minimax'
    e0, e1 = rate(B, baseline, K, v3=True, minimax=minimax), rate(B, out, K, v3=True, minimax=minimax)
    keys = tuple(dict.fromkeys(gate_fields(e0['design_rules']) + gate_fields(e1['design_rules'])))
    missing = [q for q in keys if not all(isinstance(e['design_rules'].get(q), (int, float)) and
               math.isfinite(e['design_rules'][q]) for e in (e0, e1))]
    worse = [q for q in keys if q not in missing and e1['design_rules'][q] > e0['design_rules'][q] + 1e-8]
    contract = assumption(e0['design_rules'], e1['design_rules'])
    if minimax:
        from analysis.minimax import assumption as minimax_assumption
        contract = minimax_assumption(e0, e1)
    info.update(evaluation_target='minimax' if minimax else 'v3', acceptance_gate=contract)
    good = contract['not_worse'] and e1['is_valid'] and not missing and not worse and not e0['ir']['error_nets'] and not e1['ir']['error_nets'] and audit['levels'] == 0 and audit['hard_corners'] == 0
    info.update(applied=good, gate_worse=worse, gate_missing=missing, valid_before=e0['is_valid'], valid_after=e1['is_valid'],
                score_v3_before=e0['score_v3_nets'], score_v3_after=e1['score_v3_nets'], evaluation_cpu_s=analysis_cpu(),
                ir_error_before=e0['ir']['error_nets'], ir_error_after=e1['ir']['error_nets'])
    if good:
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=Path(rj).parent, prefix='.current_shape_', delete=False) as f:
            tmp = f.name
            json.dump(out, f, ensure_ascii=False)
        try:
            shutil.copyfile(rj, rj[:-5] + '_before_copper.json')
            os.replace(tmp, rj)
        finally:
            if os.path.exists(tmp): os.unlink(tmp)
    else:
        info['reason'] = 'Gate not passed: no acceptance'
    return info


bar_post_run = current_post_run


def finalize(bj, kj, rj, design_rules, gnd_layers, cfg, mode, r36, cpu=None):
    route_gnd_layers = gnd_layers
    z = {}
    if cpu is not None and cpu.rest < 1:
        raise TimeoutError('Finish: CPU budget exhausted')
    try:
        from learning.zone_planner import final
        _rv = json.load(open(rj)) if os.path.exists(rj) else {}
        if _rv.get('conflicts') or _rv.get('failed'):


            z['current_zones'] = dict(applied=False, off=True, native_acceptance=False, reason='router_invalid')
        elif r36 == '0' and mode != 'local':
            z['current_zones'] = dict(applied=False,off=True,native_acceptance=False)
        else:
            z['current_zones'] = final(bj, kj, rj, design_rules,
                route_gnd_layers, mode='local' if mode == 'local' else r36, cpu_s=720 if cpu is None else max(1,int(cpu.rest)))
            if cpu: cpu.add(z['current_zones'].get('compute_slot',{}).get('cpu_s',0))
    except Exception as e: z['current_zones'] = dict(applied=False,native_acceptance=False,faults=repr(e)[:160])
    if cfg.get('k20_zones'):
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Finish: CPU budget exhausted')
        from learning.k20_post_run import apply as k20_apply
        z['k20_zones'] = k20_apply(bj, kj, rj,
                                    design_rules, route_gnd_layers,
                                    cpu_s=720 if cpu is None else int(cpu.rest), zone_plan=mode)
        if cpu: cpu.add(z['k20_zones'].get('compute_slot', {}).get('cpu_s', 0))
        if z['k20_zones'].get('applied'):
            from learning.k20_post_run import acceptance_info
            z['current_zones_standard'] = z.get('current_zones')
            z['current_zones'] = acceptance_info(z.get('current_zones'), z['k20_zones'])
    if cfg.get('r39_repair'):
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Finish: CPU budget exhausted')
        from learning.r39_post_run import apply as r39_apply
        z['r39_repair'] = r39_apply(bj, kj, rj, route_gnd_layers,
                                         cpu_s=240 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r39_repair']['compute_slot']['cpu_s'])
        if z['r39_repair'].get('applied'):
            z['current_zones_before_r39'] = z.get('current_zones')
            z['current_zones'] = dict(applied=True, native_acceptance=False, fallback='geometric',
                                   reason='R39 geometry replaces native acceptance')
    if cfg.get('r11_ir'):
        if cpu is not None and cpu.rest < 1:
            z['r11_ir'] = dict(applied=False, reason='time_limit',
                               compute_slot=dict(cpu_s=0.0, cur_state='not_started'))
            raise TimeoutError('Finish: CPU budget exhausted')
        from learning.r11_post_run import apply as r11_apply
        z['r11_ir'] = r11_apply(bj, kj, rj,
                                  design_rules,
                                  cpu_s=240 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r11_ir']['compute_slot']['cpu_s'])
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Finish: CPU budget exhausted')
    if cfg.get('r24'):
        from learning.r24_post_run import apply as r24_apply
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Finish: CPU budget exhausted')
        z['r24'] = r24_apply(bj, kj, rj,
                               design_rules,
                               cpu_s=180 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r24']['compute_slot']['cpu_s'])
    return z


if __name__ == '__main__':
    bj, kj, rj, rg, result, mode = sys.argv[1:]
    info = _current_post_run(bj, kj, rj, rg or None, minimax=None if mode == 'auto' else mode == '1')
    from pathlib import Path
    Path(result).write_text(json.dumps(info, ensure_ascii=False))
