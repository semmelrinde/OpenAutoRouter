import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def measure(B, R, K, gl):
    from analysis.evaluation import rate
    from learning.shaping import is_valid
    from learning.collect import rule_measurement
    from analysis.wobble import measure as wobble
    e = rate(B, dict(R, gnd_layers=list(gl)), K, v3=True, minimax=False)
    good, _ = is_valid(B, K, None, gl, routes=R, minimax=False)
    return dict(is_valid=good and e['is_valid'], design_rules=e['design_rules'],
                learning_rules=rule_measurement(B, R, K, gl)['design_rules'],
                wobble=wobble(B, R, gnd=K.get('gnd') or []))


def gate_check(a, b):
    from analysis.rule_metrics import assumption
    gates = [assumption(a[key], b[key]) for key in ('design_rules', 'learning_rules')]
    shaping = b['wobble']['move_records'] <= a['wobble']['move_records'] and b['wobble']['length_mm'] <= a['wobble']['length_mm']
    return dict(passed=bool(a['is_valid'] and b['is_valid'] and shaping and all(g['not_worse'] for g in gates)),
                design_rules=[dict(passed=g['not_worse'], reason=g['reason']) for g in gates], shaping=shaping)


def optimize(B, R, K, rules, gl):
    from analysis.standard import goal
    if goal(K.get('specs')) != 'v3':
        return R, dict(applied=False, reason='Only for explicit v3 contract')
    from postprocessing.r19 import suggestion
    candidate, info = suggestion(B, R, K, rules)
    info.update(applied=False, attempts=[])
    if not info['changes']:
        info['reason'] = 'No free shortcut'
        return R, info
    a = measure(B, R, K, gl); info['before'] = a
    if not a['is_valid']:
        info['reason'] = 'Invalid base'
        return R, info
    def verify(trial, net):
        from analysis.length_impedance import length_list, receiver_protection, receiver_netclasses
        lk = receiver_netclasses(K)
        if not receiver_protection(length_list(B, R, lk), length_list(B, trial, lk)):
            return trial, dict(net_name=net, passed=False, reason='ReceiverGate'), None
        b = measure(B, trial, K, gl); result = dict(gate_check(a, b), net_name=net)
        return trial, result, b
    result, gate, b = verify(candidate, '*')
    info['attempts'].append(gate)
    if gate['passed']:
        info.update(applied=True, after=b, reason='Rule and shape gate passed')
        return result, info


    gains = {}
    for c in info['changes']:
        gains[c['net']] = gains.get(c['net'], 0.)+c['length_before']-c['length_after']
    current = R
    for net in sorted(gains, key=lambda n: (-gains[n], n))[:4]:
        trial = copy.deepcopy(current)
        for i, t in enumerate(candidate['tracks']):
            if t['net'] == net: trial['tracks'][i] = copy.deepcopy(t)
        result, gate, b = verify(trial, net); info['attempts'].append(gate)
        if gate['passed']:
            current = result; a = b
            info.update(applied=True, after=b)
    info['reason'] = 'Single nets accepted' if info['applied'] else 'Rule gate blocks shortcuts'
    return current, info


def apply(bj, kj, rj, rules, gl, cpu_s=180):
    from common import slots as PL
    original = Path(rj).read_bytes()
    with tempfile.TemporaryDirectory(prefix='r19_post_run_') as folder:
        out, meta = Path(folder)/'routes.json', Path(folder)/'info.json'
        run = PL.job([str(ROOT/'.venv/bin/python'), str(Path(__file__)), 'worker',
                       str(bj), str(kj), str(rj), str(rules), json.dumps(gl), str(out), str(meta)],
                      cpu_s=cpu_s, log=str(Path(folder)/'worker.log'), cwd=str(ROOT),
                      env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(meta.read_text()) if run['cur_state'] == 'ok' and meta.exists() else dict(
            applied=False, reason='Worker '+run['cur_state'])
        info['compute_slot'] = run
        if info['applied']:
            if Path(rj).read_bytes() != original:
                info.update(applied=False, reason='Route changed during follow-up')
                return info
            Path(rj).with_name(Path(rj).stem+'_before_r19.json').write_bytes(original)
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                tmp = f.name; f.write(out.read_bytes())
            os.replace(tmp, rj)
        return info


if __name__ == '__main__':
    _, bj, kj, rj, rules, gl, out, meta = sys.argv[1:]
    t0 = time.process_time()
    B, K, R, rg = [json.loads(Path(p).read_text()) for p in (bj, kj, rj, rules)]
    result, info = optimize(B, R, K, rg, json.loads(gl))
    info['cpu_s'] = time.process_time()-t0
    Path(out).write_text(json.dumps(result)); Path(meta).write_text(json.dumps(info))
