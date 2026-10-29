

def assumption(B, old, new, K):
    from analysis.evaluation import rate
    from analysis.rule_metrics import assumption as rule_gate
    from analysis.k20.metric import measure
    before, after = rate(B, old, K, v3=True), rate(B, new, K, v3=True)
    gate = rule_gate(before['design_rules'], after['design_rules'])
    a, b = measure(B, old, K), measure(B, new, K)
    fields = ('without_zone_mm', 'pad_groups_without_common_zone')
    worse = [k for k in fields if b[k] > a[k] + 1e-4]
    gain = [k for k in fields if b[k] < a[k] - 1e-4]
    for name in ('gnd_zone_layers_missing', 'priority_error'):
        def identities(values):
            return {tuple(sorted(v.items())) if isinstance(v,dict) else v for v in values}
        old_ids,new_ids=identities(a[name]),identities(b[name])
        if new_ids-old_ids: worse.append(name)
        if old_ids-new_ids: gain.append(name)



    aa,bb=a['supply_outer_share'],b['supply_outer_share']
    if aa is None or bb is None: worse.append('Outer area unknown')
    elif bb > max(.1, aa) + 1e-6: worse.append('supply_outer_share')
    elif aa > .1 and bb < aa - 1e-6: gain.append('supply_outer_share')
    valid = bool(after['is_valid'])
    return dict(taken=valid and gate['not_worse'] and not worse and bool(gain),
                valid_before=before['is_valid'], valid_after=valid,
                rule_gate=gate, worse=worse, better=gain, before=a, after=b)


def apply(bj, kj, rj, design_rules=None, gnd_layers=None, cpu_s=720, zone_plan=None):
    from learning.current_zones import apply as zone_list
    if cpu_s < 1:
        return dict(applied=False, reason='time_limit',
                    compute_slot=dict(cpu_s=0.0, cur_state='not_started'))
    if zone_plan == 'local':
        return dict(applied=False, reason='K20 blocked with local zone plan',
                    compute_slot=dict(cpu_s=0.0, cur_state='not_started'))
    try:
        return zone_list(bj,kj,rj,gnd_layers,design_rules=design_rules,cpu_s=cpu_s,k20=True)
    except Exception as exc:
        return dict(applied=False, reason='K20 check failed', faults=repr(exc))


def acceptance_info(standard, option):
    if (option or {}).get('applied'):
        return dict(option, native_acceptance=True, final='k20')
    return standard
