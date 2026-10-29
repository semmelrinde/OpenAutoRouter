from analysis import rule_metrics as RK


def choose(baseline, candidate, *, valid_baseline, valid_candidate):
    out = dict(candidate=False, reason='Keep reference')
    if not isinstance(baseline, dict) or not isinstance(candidate, dict):
        return dict(out, reason='Result missing')
    if type(valid_baseline) is not bool or type(valid_candidate) is not bool:
        return dict(out, reason='Validity unknown')
    if not valid_candidate:
        return dict(out, reason='Candidate invalid')
    a, b = baseline.get('design_rules'), candidate.get('design_rules')
    if not isinstance(b, dict) or (valid_baseline and not isinstance(a, dict)):
        return dict(out, reason='Rule measurement missing')
    try:

        own = RK.assumption(b, b)
        if not own['not_worse']:
            return dict(out, reason=own['reason'])
        if not valid_baseline:
            return dict(candidate=True, reason='ValidityGain')
        gate_check = RK.assumption(a, b)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return dict(out, reason='Rule contract: '+str(exc))
    return dict(candidate=gate_check['better'], reason=gate_check['reason'], gate_check=gate_check['comparison'])
