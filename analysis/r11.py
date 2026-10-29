import math


def _count(x, positive=False):
    return (isinstance(x, (int, float)) and not isinstance(x, bool)
            and math.isfinite(x) and (x > 0 if positive else x >= 0))


def measure(K, M, B=None, R=None, *, chain_measure=None):
    from analysis.minimax import load_error
    pads = {f"{p['ref']}:{p.get('num')}": p['net'] for p in (B or {}).get('pads', [])}
    rows = {}
    missing = []
    reasons = {}
    idle = []
    for n, k in sorted(K.get('net_list', {}).items()):
        if k.get('category') not in ('current_val', 'supply', 'sw'): continue
        m = M.get('net_list', {}).get(n, {}).get('m', {})
        flow = k.get('flow') or {}
        if not isinstance(flow, dict):


            reasons[n] = 'Flow type invalid'
            missing.append(n)
            continue
        current = m.get('I_A')
        sources, loads = flow.get('source'), flow.get('load')
        reason = load_error(k)
        if flow.get('unit') != 'A' or flow.get('unit_source') not in ('spec', 'estimate'):
            reason = 'Ampere load with origin missing'
        elif not isinstance(sources, list) or not sources or not isinstance(loads, dict) or not loads:
            reason = 'Source/load connections missing'
        elif B is None or any(not isinstance(p, str) or pads.get(p) != n for p in sources + list(loads)):
            reason = 'Source/load not assigned on this net'
        elif set(sources) & set(loads):


            reason = 'Pad is both source and load'
        if reason:
            reasons[n] = reason
            missing.append(n)
            continue

        tolerance = 5e-5 + (len(loads)+1)*5e-8+1e-12

        origin = flow['unit_source']
        if flow.get('learning_limit'):
            origin += ' (learning-limited)'
        elif origin == 'estimate':
            origin = k.get('current_A_source')
        if flow['total'] == 0:

            drop, j = m.get('dV_mV'), m.get('J_x')
            if ((current is not None and (not _count(current) or current > tolerance))
                    or (m.get('I_source') is not None and m.get('I_source') != origin)
                    or (drop is not None and (not _count(drop) or drop > 1e-6))
                    or (j is not None and (not _count(j) or j > 1e-6))):
                reasons[n] = 'Measured values contradict the zero load'
                missing.append(n)
                continue
            idle.append(n)
            continue
        if (not _count(current) or abs(current-flow['total']) > tolerance
                or not isinstance(origin, str) or not origin or origin == '1A'
                or m.get('I_source') != origin):
            reasons[n] = 'Measurement current/origin does not match the load contract'
            missing.append(n)
            continue
        budget, drop = m.get('s_dc_mV'), m.get('dV_mV')
        j = m.get('J_x')
        if not (_count(budget, positive=True) and _count(drop) and _count(current)
                and _count(j) and _count(drop / budget)):
            missing.append(n)
            continue
        rows[n] = dict(dc_x=drop / budget, drop_mV=drop, budget_mV=budget,
                       group=m.get('group'), goal_x=drop/budget,
                       J_x=j, current_A=current, model=m.get('dV_source'),
                       load_source=flow.get('unit_source') or m.get('I_source'))
    groups = {}
    for name, q in sorted(M.get('group_list', {}).items()):
        members = q.get('net_list') or []
        budget = q.get('mV')
        if (not _count(budget, positive=True) or not isinstance(members, list) or not members
                or any(not isinstance(n, str) for n in members) or len(set(members)) != len(members)
                or any(n not in rows and n not in idle for n in members)):
            missing.append('group:'+name)
            continue


        value = sum(rows[n]['drop_mV'] for n in members if n in rows) / budget
        if not _count(value):
            missing.append('group:'+name)
            continue
        groups[name] = dict(dc_x=value, net_list=members, budget_mV=budget,
                            loadless_nets=[n for n in members if n in idle])
        for n in members:
            if n in rows: rows[n]['goal_x'] = value
    for n, row in rows.items():
        name = row['group']
        if name and (not isinstance(name, str) or name not in groups or n not in groups[name]['net_list']):
            missing.append(n+':budget_group:'+str(name))
    criteria = [q['dc_x'] for q in rows.values() if not q['group']] + [q['dc_x'] for q in groups.values()]
    maximum = max(criteria, default=None)
    chain = dict(status='not_measured', efficiency=None, element_max=None)
    if chain_measure is None:
        option = ((R or {}).get('options') or {}).get('r11_chain',
                    (K.get('specs') or {}).get('r11_chain', False))
        chain_measure = option is True or option == 1 or option == '1'
    if (chain_measure and rows and B is not None and R is not None
            and 'J_max_A_mm2' in (M.get('copper_ir') or {})):



        try:
            from analysis.current_chain import link_chain
            ck = dict(K, net_list={n: K['net_list'][n] for n in rows})
            result = link_chain(B, R, ck, ir=M['copper_ir'])
            measured = set(result.get('net_list') or {})
            absent = sorted(set(rows) - measured)
            status = 'incomplete' if absent and measured else 'not_measured' if not measured else 'measured'
            chain = dict(result, status=status, missing_nets=absent)
        except Exception as exc:
            chain = dict(chain, status='measurement_error', faults=repr(exc)[:200])
    return dict(version='r11-v2', status='incomplete' if missing else 'measured' if rows else 'loadless' if idle else 'no_current_model',
                nets_measured=len(rows), missing_models=len(missing),
                model_complete=bool(rows) and not missing,
                net_list=rows, group_list=groups, missing_nets=missing, model_error=reasons,
                loadless_nets=idle, loadless_n=len(idle), link_chain=chain, dc_x_max=maximum,
                nets_over_budget=sum(x > 1 for x in criteria) if criteria else None,
                J_x_max=max((q['J_x'] for q in rows.values() if q['J_x'] is not None), default=None))
