import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

WINDOW_S = 48 * 3600
REST_REDUCTION = .10
NET_REDUCTION = .05
VALID_GROWTH = 1
MEASURE_SPACING_S = 24 * 3600
PREVIOUS_WEEK_S = 7 * 86400
ROOT = Path(__file__).resolve().parents[1]


def epoch(value):
    if type(value) in (int, float) and math.isfinite(value):
        return float(value)
    dt = datetime.fromisoformat(value)
    return dt.timestamp()


def count(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def read_in(path, faults):
    if not path.exists():
        return []
    try:
        if path.suffix == '.jsonl':
            out = []
            for n, line in enumerate(path.read_text().splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    faults.append(f'{path}:{n}: unreadable row')
            return out
        return [json.loads(path.read_text())]
    except (OSError, ValueError) as exc:
        faults.append(f'{path}: {exc}')
        return []


def history(root, now):
    faults = []
    series = dict(learning_curve=[], bga=[], refboard=[])

    def add(name, row, source):
        try:
            t = epoch(row['time_s'])
            g = row['is_valid']
            if not count(g) or int(g) != g or t > now:
                raise ValueError('Time/validity invalid')
            from learning.learning_curve import point_incomplete
            if point_incomplete(row):
                raise ValueError('Validity partly unknown')
            n = row.get('tasks')
            if n is not None and (not count(n) or n < g):
                raise ValueError('Task count invalid')
            if name == 'learning_curve' and n != 15:
                raise ValueError('no fixed series of 15')
            if name == 'bga' and n != 42:
                raise ValueError('no full series of 42')
            for k in ('remaining_conflicts', 'worst_net'):
                if row.get(k) is not None and not count(row[k]):
                    raise ValueError(k + ' invalid')

            if row.get('cpu_per_valid') is not None and not count(row['cpu_per_valid']):
                faults.append(f'{source}: cpu_per_valid invalid (secondary figure only)')
                row = dict(row, cpu_per_valid=None)
            series[name].append(dict(row, epoch=t, source=str(source)))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            faults.append(f'{source}: {exc}')

    path = root / 'learning/data/learning_curve.jsonl'
    for row in read_in(path, faults):
        if not isinstance(row, dict):
            faults.append(f'{path}: not an object')
            continue
        if row.get('dataset', 'learning_curve') != 'learning_curve':
            continue
        row = dict(row, contract_spec=json.dumps([row.get(k) for k in
                   ('generator_version', 'tasks_sha', 'evaluation_target', 'measurement_contract')]))
        add('learning_curve', row, path)

    bga_root = Path(os.environ.get('STAGNATION_BGA_RESULTS', root / 'codex/bga_fanout/results'))
    for path in sorted(bga_root.glob('**/balance.json')):
        try:
            c = json.loads(path.with_name('contract_spec.json').read_text())
            b = json.loads(path.read_text())['total']

            if c.get('format') != 'bga-fanout-v1' or c.get('arm') != 'basis':
                continue
            if sorted(c['indices']) != list(range(42)):
                continue
            contract = json.dumps({k: c.get(k) for k in
                ('format', 'indices', 'tasks', 'cpu_phase', 'iter', 'cfg', 'arm')}, sort_keys=True)
            add('bga', dict(time_s=path.stat().st_mtime, is_valid=b['is_valid'],
                tasks=b['n'], unknown=b['unknown'], code=c.get('commit'), contract_spec=contract), path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            faults.append(f'{path}: {exc}')
    path = root / 'codex/refboard/history.jsonl'
    for row in read_in(path, faults):
        if not isinstance(row, dict) or not row.get('contract_spec'):
            faults.append(f'{path}: reference board contract missing')
            continue
        add('refboard', row, path)
    for name, rows in series.items():
        rows.sort(key=lambda r: r['epoch'])

        if rows:
            contract = rows[-1]['contract_spec']
            series[name] = [r for r in rows if r['contract_spec'] == contract]
    return series, faults


def decision(series, now):
    events = []
    start = []
    details = {}
    for name, rows in series.items():
        if not rows:
            details[name] = dict(status='missing')
            continue
        start.append(rows[0]['epoch'])
        best_g = rows[0]['is_valid']

        anchors = {}
        for index, row in enumerate(rows):
            g = row['is_valid']
            reasons = []
            if g >= best_g + VALID_GROWTH:
                reasons.append('+1 valid')
            if g >= best_g:
                metrics = ([('remaining_conflicts', REST_REDUCTION),
                            ('worst_net', NET_REDUCTION)] if name == 'refboard' else [])
                for key, fraction in metrics:
                    value = row.get(key)
                    if value is None:
                        continue
                    anchor = anchors.get((g, key))
                    recent = [r.get(key) for r in rows[:index]
                              if row['epoch'] - WINDOW_S <= r['epoch'] < row['epoch']
                              and r['is_valid'] == g and r.get(key) is not None]
                    window_gain = any(v > 0 and value <= v * (1 - fraction) for v in recent)
                    if anchor is not None and anchor > 0 and value <= anchor * (1 - fraction) and window_gain:
                        reasons.append(key)
                        anchors[g, key] = value
                    elif anchor is None:
                        anchors[g, key] = value
            best_g = max(best_g, g)
            if reasons:
                events.append(dict(epoch=row['epoch'], series=name, reasons=reasons))
        details[name] = dict(status='current' if now - rows[-1]['epoch'] <= MEASURE_SPACING_S else 'outdated',
                             current=rows[-1], measurements=len(rows))
    load = max((e['epoch'] for e in events), default=min(start) if start else None)
    fresh = any(d['status'] == 'current' for d in details.values())
    covered = any(rows and now - rows[0]['epoch'] >= WINDOW_S
                  and now - rows[-1]['epoch'] <= MEASURE_SPACING_S for rows in series.values())
    status = ('unknown' if load is None or not fresh else
              'improvement' if any(now - e['epoch'] < WINDOW_S for e in events) else
              'stall' if covered and now - load >= WINDOW_S else 'observation')
    basis_rows = [r for rows in series.values() for r in rows if r['epoch'] == load]
    code = next((r.get('code') for r in basis_rows if r.get('code')), None)
    return dict(code_baseline=code, status=status, last_improvement_epoch=max((e['epoch'] for e in events), default=None),
                baseline_epoch=load, series=details, improvements=events)


def report(series, now):
    def current(name):
        return series[name][-1] if series[name] else {}

    def week(name):
        return next((r for r in reversed(series[name]) if r['epoch'] <= now - PREVIOUS_WEEK_S), {})

    def fmt(row, key):
        x = row.get(key)
        return 'unknown' if x is None else f'{x:g}'

    a, w, b, v, vw = current('learning_curve'), week('learning_curve'), current('bga'), current('refboard'), week('refboard')
    line = (f"Curve: 15-series {fmt(a, 'is_valid')}/15 (previous week {fmt(w, 'is_valid')}/15), "
            f"CPU per valid {fmt(a, 'cpu_per_valid')} s (previous week {fmt(w, 'cpu_per_valid')} s); "
            f"refboard BGA {fmt(b, 'is_valid')}/42; reference board residual conflicts {fmt(v, 'remaining_conflicts')} "
            f"(previous week {fmt(vw, 'remaining_conflicts')})")
    stale = [name for name in series if series[name] and now - series[name][-1]['epoch'] > MEASURE_SPACING_S]
    return line + ('; outdated: ' + ', '.join(stale) if stale else '')


def atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(text)
    temp.replace(path)


def prefer(queue):
    source = queue / 'deferred'
    target = queue / 'wait'
    if not source.exists():
        return None
    target.mkdir(parents=True, exist_ok=True)
    for p in sorted(source.iterdir()):
        if not p.is_file() or p.is_symlink() or not re.fullmatch(r'0[0-9x]_strategy_.*\.txt', p.name):
            continue
        dest = target / ('00_' + p.name[3:])
        try:
            os.link(p, dest)
        except FileExistsError:
            continue
        p.unlink()
        return str(dest)
    return None


def verify(root=ROOT, queue=None, now=None):
    now = time.time() if now is None else now
    queue = Path(queue or Path.home() / 'Library/Caches/Autorouter/codex_orders')
    data = root / 'learning/data'
    data.mkdir(parents=True, exist_ok=True)
    with (data / '.stagnation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        series, faults = history(root, now)
        result = decision(series, now)
        result.update(time_s=datetime.fromtimestamp(now, timezone.utc).isoformat(), read_error=faults)
        state = data / 'stagnation.json'
        old = read_in(state, faults)
        if result['status'] != 'stall':
            return result

        if old and isinstance(old[0], dict) and old[0].get('baseline_epoch') == result['baseline_epoch']:
            return result
        since = datetime.fromtimestamp(result['baseline_epoch'], timezone.utc).isoformat()
        code = result.get('code_baseline')
        revision = None
        if isinstance(code, str) and re.fullmatch(r'[0-9a-f]{7,40}', code):
            resolved = subprocess.run(['git', 'rev-parse', '--verify', code + '^{commit}'],
                                      cwd=root, capture_output=True, text=True)
            if resolved.returncode == 0:
                revision = code + '..HEAD'
        result['commit_assignment'] = 'revision' if revision else 'temporal'
        cmd = ['git', 'log', '--format=%H %s'] + ([revision] if revision else ['--since=' + since, 'HEAD'])
        git = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
        result['code_commits_since_improvement'] = git.stdout.splitlines() if git.returncode == 0 else None
        result['git_error'] = git.stderr.strip() if git.returncode else None

        atomic(state, json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        result['preferred'] = prefer(queue)
        atomic(state, json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        day = datetime.fromtimestamp(now).strftime('%Y-%m-%d')
        text = (f'# Stagnation {day}\n\nNo threshold progress in the comparable measurements for 48 h.\n\n'
                + report(series, now) + '\n\nSeries status: '
                + ', '.join(f"{k}: {v['status']}" for k, v in result['series'].items())
                + '\n\nCode since load improvement (otherwise since first measurement):\n'
                + ('\n'.join(result['code_commits_since_improvement'] or []) or 'None determined; see JSON.')
                + '\n\nPulled forward: ' + str(result['preferred'])
                + '\n\nRead error: ' + json.dumps(faults, ensure_ascii=False) + '\n')
        atomic(root / f'knowledge/stagnation_{day}.md', text)
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', nargs='?', choices=('verify', 'report'), default='verify')
    p.add_argument('--root', type=Path, default=Path(os.environ.get('LEARNING_CURVE_MAIN', ROOT)))
    p.add_argument('--queue', type=Path)
    a = p.parse_args()
    if a.action == 'report':
        series, faults = history(a.root, time.time())
        print(report(series, time.time()) + ('; read error: ' + str(len(faults)) if faults else ''))
    else:
        print(json.dumps(verify(a.root, a.queue), ensure_ascii=False))


if __name__ == '__main__':
    main()
