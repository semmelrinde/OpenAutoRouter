import hashlib
import json
import math
from pathlib import Path
import re
import sys


def read_in(path):
    with open(path) as f:
        for line in f:
            try: yield json.loads(line)
            except ValueError: continue


def cpu_s(z):
    if 'cpu_total' in z and (not isinstance(z['cpu_total'], (int, float)) or
            isinstance(z['cpu_total'], bool) or not math.isfinite(z['cpu_total']) or z['cpu_total'] < 0):
        return None
    if isinstance(z.get('cpu_total'), (int, float)):
        total = z['cpu_total']
    else:
        total = children_cpu_s(z) + sum((z.get(k) or {}).get('cpu_s', 0) or 0
                                     for k in ('dataset', 'labels_candidates'))
        total += (z.get('router') or {}).get('cpu_owned_s', 0) or 0
        total += (z.get('r44') or {}).get('cpu_s', 0) or 0



    if 'cpu_racetrack_measurement_s' not in z:
        extra = ((z.get('racetrack') or {}).get('measurement') or {}).get('compute_slot', {}).get('cpu_s')
        if isinstance(extra, (int, float)) and not isinstance(extra, bool) and math.isfinite(extra) and extra >= 0:
            total += extra
    return total


def children_cpu_s(z):
    contained = set((z.get('router') or {}).get('cpu_contains', []))
    return sum(((z.get(k) or {}).get('cpu_s', 0) or 0) - ((z.get(k) or {}).get('cpu_owned_s', 0) or 0) for k in
               ('placer', 'router', 'shaping', 'netclasses_input', 'netclasses_placement') if k not in contained) + sum(
                    (z.get(k) or {}).get('compute_slot', {}).get('cpu_s', 0) or 0
                    for k in ('zone_plan', 'current_zones', 'r11_ir', 'r24') if k not in contained)


def cpu_total_sum(rows):
    charges = [cpu_s(z) for z in rows]
    return round(sum(charges), 3) if all(c is not None for c in charges) else None


def summary(rows):
    out = {}
    for cu_layers in (2, 4, 6, 8, 'all_items'):
        rr = [z for z in rows if cu_layers == 'all_items' or z.get('cu_layers') == cu_layers]
        charges = [cpu_s(z) for z in rr]
        unknown = sum(c is None for c in charges)
        cpu = sum(c for c in charges if c is not None)
        usable = [z for z in rr if not z.get('code_changed') and not z.get('generator_failure')]
        labels = sum((z.get('dataset') or {}).get('labels_n', 0) or 0 for z in usable
                     if (z.get('dataset') or {}).get('cur_state') == 'ok')
        seen = set(); new_labels = 0; known = 0
        for z in usable:
            ds = z.get('dataset') or {}; h = ds.get('goal_sha256')
            identity = (ds.get('goal_hash_version', 1), ds.get('router_observer_version', 1), h)
            if ds.get('cur_state') == 'ok' and h:
                known += 1
                if identity not in seen: new_labels += ds.get('labels_n', 0); seen.add(identity)
        valid = sum(_limit_result(z) is True for z in usable if not z.get('not_routed'))
        dense = sum((z.get('dataset') or {}).get('cur_state') == 'ok' for z in usable)
        full = sum((z.get('learning_data') or {}).get('complete', False) for z in usable)
        best = {}; gains = 0
        for z in usable:
            key = (z.get('task', z.get('board')), z.get('factor'), z.get('evaluation_version'))
            n = z.get('score_v3')
            if _limit_result(z) is True and isinstance(n, (int, float)) and math.isfinite(n):
                if key in best and n < best[key]: gains += 1
                best[key] = min(best.get(key, float('inf')), n)
        rate = lambda n: n * 3600 / cpu if cpu and not unknown else None
        out[str(cu_layers)] = dict(attempts=len(rr), cpu_s=round(cpu, 3), is_valid=valid,
            valid_per_cpu_h=rate(valid), shards=dense, shards_per_cpu_h=rate(dense),
            table_targets=labels, table_targets_per_cpu_h=rate(labels),
            new_table_targets=new_labels if known else None, new_table_targets_per_cpu_h=rate(new_labels) if known else None,
            novelty_shards_measured=known,
            learning_rows_whole=full, learning_rows_per_cpu_h=rate(full),
            own_best_score_gains=gains, own_best_score_gains_per_cpu_h=rate(gains),
            options_decided=None, model_gain=None,
            cpu_unknown=unknown,
            cpu_complete=sum(z.get('cpu_measurement') in ('entire', 'whole_wait4') for z in rr))
    return dict(version=1, mass='Data yield; model gain and independent information not measured', cu_layers=out)


def complete(z, c):
    c = Path(c); rows = list(read_in(c / 'history.jsonl')) if (c / 'history.jsonl').exists() else []
    if not rows:
        try: txt = (c / 'route.log').read_text(errors='replace')
        except OSError: txt = ''
        rows = [dict(round_no=int(n) - 1, conflict_list=int(k), open_items=int(f), cpu_s=None)
                for n, k, f in re.findall(r'Iteration (\d+): conflicts (\d+), incomplete (\d+)', txt)]
    r = {}
    try: r = json.loads((c / 'routes.json').read_text())
    except (OSError, ValueError): pass
    z.setdefault('is_valid', False)
    limits = ('SIGXCPU', 'cpu_budget', 'early_abort', 'budget_before_post_run', 'time_limit', 'wall_limit', 'time_s')
    state = z.get('cur_state', 'ok')
    attempt_limit = (z.get('cpu_time_limit') or z.get('budget_end') or
                     (z.get('cpu_budget') or {}).get('overflow', 0) > 0 or
                     any(state == q or state.endswith('_' + q) for q in limits))
    child_limit = any((z.get(q) or {}).get('cur_state') in limits or
                      (z.get(q) or {}).get('compute_slot', {}).get('cur_state') in limits or
                      (z.get(q) or {}).get('reason') in limits
                      for q in ('placer', 'router', 'shaping', 'zone_plan', 'current_zones', 'r11_ir',
                                'netclasses_input', 'netclasses_placement'))
    router_limit = r.get('final') in limits
    if r.get('final') == 'early_abort':

        z.update(is_valid=None, early_abort=r.get('early_abort') or dict(censored=True))
        if state != 'r44_unchecked' and not z.get('code_changed'):
            z.update(cur_state='early_abort', invalid_reason='Prediction: no success within CPU budget (censored)')
            state = 'early_abort'
    limited = bool(attempt_limit or child_limit or router_limit)
    incomplete = bool(z.get('faults') or z.get('rule_error') or
                      any((z.get(q) or {}).get('faults') for q in ('shaping', 'zone_plan', 'current_zones', 'r11_ir')) or
                      ((z.get('shaping') or {}).get('applied') is False and (z.get('shaping') or {}).get('reason') not in (None, '', 'off', 'ReceiverGate', 'RuleGate', 'Shape would be invalid')) or state not in ('ok', 'only_substitute_ground') or
                      any((z.get(q) or {}).get('cur_state') == 'faults' or
                          (z.get(q) or {}).get('compute_slot', {}).get('cur_state') == 'faults'
                          for q in ('router', 'shaping', 'zone_plan', 'current_zones', 'r11_ir')))
    reason = ('TotalBudget' if z.get('cpu_time_limit') or (z.get('cpu_budget') or {}).get('overflow', 0) > 0
              else state if attempt_limit or state not in ('ok', 'only_substitute_ground')
              else 'EvaluationAbort' if z.get('rule_error')
              else 'TrailingTimeLimit' if child_limit
              else 'FollowupIncomplete' if incomplete else
              z.get('invalid_reason') or r.get('final') or state)
    if limited:
        z['budget_end'] = True
        if state == 'ok' and not z.get('code_changed'):
            z['cur_state'] = 'early_abort' if r.get('final') == 'early_abort' else 'time_limit'
    z['conflict_history'] = [[q['conflict_list'], q['open_items']] for q in rows]
    z['learning_data'] = dict(version=1, cur_state=z.get('cur_state', 'faults'), cause=reason,
        faults=z.get('faults') or z.get('rule_error'),
        compute_runs={q: z[q] for q in ('placer', 'router', 'netclasses_input', 'netclasses_placement') if q in z},
        complete=True, history=rows, partial_result=bool(r),
        censored=limited or incomplete or not r, solvability_known=not (limited or incomplete or not r) and z.get('is_valid') is True,
        labels_present=(c / 'labels.npz').exists() or (c / 'routes_labels.npz').exists(),
        conflict_sites=r.get('learning_state', {}).get('pairs', []) or r.get('short_circuit_protection', {}).get('pairs_before_cleanup', []),
        export_protection=r.get('short_circuit_protection'), budget=r.get('learning_budget'),
        placer_history=z.get('placer_attempts', []))
    return z


def decision(rows, pads, policy):
    n = int(policy.get('round_no', 4))
    rr = [q for q in rows if q.get('phase') == 'negotiation']
    if len(rr) < n: return False
    tail = rr[-n:]; first = max(1, tail[0]['conflict_list'] + tail[0]['open_items'])
    current = tail[-1]['conflict_list'] + tail[-1]['open_items']
    return (current > 8 and current / max(1, pads) >= float(policy['conflicts_pro_pad']) and
            current / first >= float(policy.get('remaining_share', .95)))


def promising(rows):
    rr = [q for q in rows if q.get('phase') == 'negotiation'][-4:]
    if len(rr) < 3: return False
    v = [q['conflict_list'] + q['open_items'] for q in rr]
    return 0 < v[-1] < .8 * max(1, v[0])


def _limit_result(z):
    if z.get('code_changed') or z.get('generator_failure'): return None
    if type(z.get('is_valid')) is not bool: return None
    if ((z.get('learning_data') or {}).get('censored') or z.get('cpu_time_limit') or
            z.get('budget_end') or z.get('faults') or z.get('rule_error') or
            (z.get('cpu_budget') or {}).get('overflow', 0) > 0): return None
    if z.get('cur_state', 'ok') not in ('ok', 'only_substitute_ground', 'placer_collision'): return None
    shaping = z.get('shaping') or {}
    if shaping.get('applied') is False and shaping.get('reason') not in (
            None, '', 'off', 'ReceiverGate', 'RuleGate', 'Shape would be invalid'): return None
    for key in ('placer', 'router', 'shaping', 'zone_plan', 'current_zones', 'r11_ir',
                'netclasses_input', 'netclasses_placement'):
        child = z.get(key) or {}
        if child.get('faults'): return None
        for state in (child.get('cur_state'), (child.get('compute_slot') or {}).get('cur_state'), child.get('reason')):
            if state in ('SIGXCPU', 'cpu_budget', 'early_abort', 'budget_before_post_run',
                         'time_limit', 'wall_limit', 'time_s', 'faults'): return None
    return z['is_valid']


def limit_factor(rows, net_class, fallback):
    groups = {}
    for z in rows[-3000:]:
        if z.get('net_class') != net_class: continue
        result = _limit_result(z)
        if result is None: continue
        if not isinstance(z.get('factor'), (float, int)): continue
        key = (z.get('task'), round(z['factor'], 1))
        groups[key] = groups.get(key, False) or result
    if len({k[0] for k in groups}) < 12: return fallback
    bins = {}
    for (_, f), g in groups.items(): bins.setdefault(f, []).append(g)
    eligible = [(abs((sum(gs) + 1) / (len(gs) + 2) - .5), f) for f, gs in bins.items() if len(gs) >= 6]
    return max(1.1, min(2.5, min(eligible)[1])) if eligible else fallback


def tasks_order(tasks, rows, cu_layers, rnd):
    rates = {}
    for z in rows[-20000:]:
        result = _limit_result(z)
        if result is None: continue
        rates.setdefault(z.get('board'), []).append(result)
    decorated = []
    for task in tasks:
        rr = rates.get(Path(task).name, [])[-40:]
        p = (sum(rr) + 1) / (len(rr) + 2)
        explore = rnd.random() < .2 or len(rr) < 6
        decorated.append(((0 if cu_layers.get(Path(task).name) in (4, 6, 8) else 1,
                          0 if explore else 1, rnd.random() if explore else abs(p - .5)), task))
    return [task for _, task in sorted(decorated)]


def calibrate(rows):
    result = {}
    for layer in (4, 6, 8):
        rr = [z for z in rows if z.get('cu_layers') == layer and z.get('conflict_history') and not z.get('code_changed')
              and (z.get('learning_data') or {}).get('cause') != 'early_abort']
        def holdout(z):
            family = str(z.get('family', z.get('design_family', z.get('task', z.get('board')))))
            return int(hashlib.sha256(family.encode()).hexdigest(), 16) % 3 == 0
        train = [z for z in rr if not holdout(z)]; test = [z for z in rr if holdout(z)]
        def evaluate(data, policy):
            hits = []; savings = 0.; known = 0
            for z in data:
                n = policy['round_no']
                seq = [dict(phase='negotiation', conflict_list=k, open_items=f) for k, f in z['conflict_history'][:n]]
                pads = z.get('pads') or (z.get('feats') or {}).get('n_pads')
                if not pads or not decision(seq, pads, policy): continue
                hits.append(z)
                v = [q for q in (z.get('learning_data') or {}).get('history', []) if q.get('phase') == 'negotiation']
                cp = (z.get('router') or {}).get('cpu_s')
                if z.get('is_valid') is False and len(v) >= n and cp and isinstance(v[n-1].get('cpu_s'), (float, int)):
                    known += 1; savings += max(0, cp - v[n-1]['cpu_s'])
            invalid_cpu = sum((z.get('router') or {}).get('cpu_s', 0) or 0 for z in data if z.get('is_valid') is False)
            return dict(n=len(data), valid=sum(z.get('is_valid') is True for z in data), hit=len(hits),
                        lost=sum(z.get('is_valid') is True for z in hits), cpu_saving_share=savings / invalid_cpu if known and invalid_cpu else None,
                        timing_hits=known)
        candidates = []
        for n in (3, 4, 5, 6):
            for threshold in (.1, .2, .3, .5):
                for rest in (.8, .9, .95, 1.):
                    p = dict(version=1, status='experimental', round_no=n, conflicts_pro_pad=threshold, remaining_share=rest)
                    e = evaluate(train, p)
                    if e['hit'] and e['lost'] == 0: candidates.append((e['hit'], p, e))
        if not candidates:
            result[str(layer)] = dict(status='unknown', reason='no trainable candidate'); continue
        _, p, e = sorted(candidates, key=lambda q: (-q[0], q[1]['round_no'], -q[1]['conflicts_pro_pad']))[0]
        h = evaluate(test, p)
        result[str(layer)] = dict(policy=p, train=e, holdout=h, released=False,
            empirical_gate_fulfilled=bool(h['valid'] and h['lost'] / h['valid'] <= .02 and h['cpu_saving_share'] is not None and h['cpu_saving_share'] >= .5),
            reason='Replay is not a prospective release; demonstrate CPU gate and validity gate paired')
    return dict(version=1, split='sha256(family)%3=0 is holdout', cu_layers=result)


def main():
    paths = [x for x in sys.argv[1:] if '=' not in x and x != 'calibrate']
    kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    rows = [z for p in paths for z in read_in(p)]
    result = calibrate(rows) if 'calibrate' in sys.argv[1:] else summary(rows)
    if 'calibrate' not in sys.argv[1:]:
        window = max(1, int(kw.get('window', '200')))
        result['history'] = [dict(lines_from=i, lines_until=min(i + window, len(rows)),
            cpu_cumulative_s=cpu_total_sum(rows[:i + window]),
            cu_layers=summary(rows[i:i + window])['cu_layers']) for i in range(0, len(rows), window)]
    result['source_list'] = [dict(fpath=str(Path(p).resolve()), sha256=hashlib.sha256(Path(p).read_bytes()).hexdigest()) for p in paths]
    s = json.dumps(result, indent=2, ensure_ascii=False)
    if kw.get('out'): Path(kw['out']).write_text(s + '\n')
    else: print(s)


if __name__ == '__main__': main()
