import collections, json, os, sys, time
from pathlib import Path

R = Path(__file__).resolve().parents[1]
Q = Path.home() / 'Library/Caches/Autorouter/codex_orders'


def cause(r):
    g = r.get('invalid_reason') or ''
    z = r.get('cur_state') or ''
    if r.get('is_valid'): return 'is_valid'
    if g.startswith('Placement') or z == 'placer_collision': return 'placement_collision'
    if 'time_limit' in z and 'placer' in z: return 'placer_time_limit'
    if 'geometry' in g.lower(): return 'zones_geometry'
    if z in ('work_measurement_incomplete', 'native_basis_incomplete') or 'incomplete' in z: return 'measurement_incomplete'
    if z == 'faults' or 'without partial result' in g: return 'router_error'
    k = r.get('metrics') or (r.get('outcome') or {})
    if (k.get('conflict_list') or 0) > 0: return 'router_conflicts'
    if (k.get('open_items') or 0) > 0: return 'router_open'

    nm = r.get('native_champion') or {}
    if nm.get('faults') or nm.get('open_items'):
        kinds = nm.get('kinds') or {}
        if kinds: return 'kicad_native_' + max(sorted(kinds), key=lambda a: kinds[a])
        return 'kicad_native_open' if not nm.get('faults') else 'kicad_native_unknown'
    return 'router_misc' if g else (z or 'unknown')


def evaluate(lines):
    out = collections.OrderedDict()
    n = len(lines) or 1
    cpu_ges = sum(float(r.get('cpu_total') or 0) for r in lines) or 1.0
    grp = collections.defaultdict(list)
    for r in lines: grp[cause(r)].append(r)
    for u, rs in sorted(grp.items(), key=lambda x: -len(x[1])):
        cpu = sum(float(r.get('cpu_total') or 0) for r in rs)
        out[u] = dict(count=len(rs), share=round(len(rs) / n, 3), cpu_share=round(cpu / cpu_ges, 3),
                      netclasses=collections.Counter(r.get('net_class') for r in rs).most_common(5))

    arms = collections.defaultdict(lambda: [0, 0])
    for r in lines:
        for k, v in sorted((r.get('cfg') or {}).items()):
            if k in ('seed',) or isinstance(v, (dict, list)): continue
            a = arms['%s=%s' % (k, v)]; a[0] += 1; a[1] += 0 if r.get('is_valid') else 1
    arm = {k: dict(n=a[0], failure=round(a[1] / a[0], 3)) for k, a in sorted(arms.items()) if a[0] >= 20}
    return out, arm


TEMPLATES = {
    'placement_collision': ('placer_collisions', 'Placer/legalization: in selfplay {share:.0%} of the runs end with residual collisions '
        'after placer repetitions and legal_lp (typically 1-4 collisions, also on easy levels; classes {netclasses}). '
        'Goal: eliminate residual collisions in general (e.g. targeted moving/rotating of the colliding parts into free space, '
        'MILP window, larger roaming radius only for colliding parts), without quality loss. Measure: selfplay sample '
        'and fixed 15s (v10/v11, budget=work): share of collision-free placements, valid boards, no regression.'),
    'placer_time_limit': ('placer_time', 'Placer time limit: {share:.0%} of the runs end in the placer time limit (classes {netclasses}). '
        'Find the cause (which step), speed it up or stage the budget sensibly; no regression.'),
    'router_conflicts': ('router_conflicts', 'Router residual conflicts: {share:.0%} of the runs end with conflicts (classes {netclasses}). '
        'Evaluate the most frequent conflict pairs/locations from the runs and solve the largest group in general.'),
    'router_open': ('router_open', 'Router open connections: {share:.0%} of the runs end with unreachable pins (classes {netclasses}). '
        'Solve pin_access/escape in general for the most frequent package types.'),
    'router_error': ('router_error', 'Router aborts: {share:.0%} of the runs end without partial result. Fix the cause per abort type.'),
    'kicad_native_unknown': ('native_gap', 'Native gap: {share:.0%} of the runs are internally valid, but KiCad reports errors/open items '
        '(classes {netclasses}). Error types per run are only in native_champion.kinds from E96 on; find the most frequent type and in the '
        'Fix the router/export contract in general, do not loosen it in the checker.'),
    'zones_geometry': ('zones_geometry', 'Zone geometry errors: {share:.0%} of the runs. Fix the cause.'),
}


def job_create(u, info, window):
    if u not in TEMPLATES: return None
    mark, text = TEMPLATES[u]
    for folder in ('wait', 'run'):
        if any(mark in p.name for p in (Q / folder).glob('*.txt')): return 'already queued'

    if any(mark in p.name and time.time() - p.stat().st_mtime < 2 * 86400 for p in (Q / 'finished').glob('*')):
        return 'recently edited'
    nc = ', '.join('%s %d' % (k, n) for k, n in info['netclasses'])
    body = ('effort: high\nPART %s · NEW (automatically from learning/error_book.py, %s; window %d selfplay runs). '
            'New working copy of main. No push. Report knowledge/codex_%s.md.\n'
            'DATA PROTECTION (hard): Technical lock active (macOS sandbox). Only allowed are repository, generated tasks, '
            'H6/PCBWorld and testboard.\n%s\nAcceptance rule (user): more valid boards without loss wins; '
            'Deliver the result as an option, Claude accepts it after a preliminary check.\n<= 4 slots.\n') % (
        mark, time.strftime('%d.%m. %H:%M'), window, mark, text.format(share=info['share'], netclasses=nc))
    goal = Q / 'wait' / ('03_%s.txt' % mark)
    goal.write_text(body)
    return str(goal)


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    window = int(kw.get('window', '600'))
    all_items = [json.loads(l) for l in open(R / 'learning/data/selfplay.jsonl') if l.strip()]
    now, before = all_items[-window:], all_items[-2 * window:-window]
    u_now, arms = evaluate(now)
    u_before, _ = evaluate(before)
    for u, info in u_now.items():
        info['trend'] = round(info['share'] - u_before.get(u, {}).get('share', 0.0), 3)
    cost = [(info['share'] * max(info['cpu_share'], 0.01), u) for u, info in u_now.items() if u != 'is_valid']
    top = max(cost)[1] if cost else None
    erg = dict(time_s=time.strftime('%Y-%m-%d %H:%M'), window=len(now), causes=u_now,
               wasted_arms={k: v for k, v in arms.items() if v['failure'] >= 0.5}, top=top)
    if kw.get('job', '0') == '1' and top:
        erg['job'] = job_create(top, u_now[top], len(now))
    goal = Path(kw.get('goal', R / 'learning/data/error_book.json'))
    goal.write_text(json.dumps(erg, ensure_ascii=False, indent=1))
    for u, info in u_now.items():
        print('%-24s %4d  %5.1f %%  CPU %5.1f %%  trend %+5.1f %%  %s' % (u, info['count'], 100 * info['share'], 100 * info['cpu_share'],
              100 * info['trend'], ', '.join('%s %d' % k for k in info['netclasses'][:3])))
    print('Wasted arms (failure >= 50 %):', ', '.join('%s %.0f %%' % (k, 100 * v['failure']) for k, v in erg['wasted_arms'].items()) or '-')
    print('Top-Cause:', top, '| Task:', erg.get('job'))


if __name__ == '__main__':
    main()
