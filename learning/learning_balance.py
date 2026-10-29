import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
VERSION = 'learning_balance-v1'
SHA = re.compile('[0-9a-f]{64}')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def number(x):
    try:
        return type(x) in (int, float) and math.isfinite(x) and x >= 0
    except OverflowError:
        return False


def seal(boards, train_families, created):
    if len(boards) != 20 or len({b['id'] for b in boards}) != 20:
        raise ValueError('Exactly 20 unique boards required')
    if len({b['family'] for b in boards}) != 20:
        raise ValueError('20 different design families required')
    if not train_families:
        raise ValueError('Training family inventory missing')
    if any(b['family'] in train_families or not SHA.fullmatch(b.get('sha256', '')) for b in boards):
        raise ValueError('Training overlap or missing board hash')
    body = dict(version='learning_check_set-v1', boards=sorted(boards, key=lambda b: b['id']),
                training_families_sha=digest(sorted(train_families)), created=created,
                unseen=True)
    return dict(body, sha256=digest(body))


def check_set(dataset):
    body = {k: v for k, v in dataset.items() if k != 'sha256'}
    boards = dataset.get('boards', [])
    if (dataset.get('version') != 'learning_check_set-v1' or dataset.get('unseen') is not True or
        dataset.get('sha256') != digest(body) or len(boards) != 20 or
        len({b['id'] for b in boards}) != 20 or len({b['family'] for b in boards}) != 20 or
        not SHA.fullmatch(dataset.get('training_families_sha', '')) or
        any(not SHA.fullmatch(b.get('sha256', '')) for b in boards)):
        raise ValueError('Check set missing, not sealed or not family-separated')
    return {b['id']: b for b in boards}


def snapshot_errors(z, dataset):
    errors = []
    if z.get('version') != VERSION or z.get('set_sha') != dataset['sha256']:
        errors.append('wrong balance/check set version')
    if not all(SHA.fullmatch(z.get(k, '')) for k in ('code_sha', 'model_sha', 'steering_sha', 'evaluator_sha', 'budget_sha')):
        errors.append('complete code/model/steering/evaluator/budget binding missing')
    if z.get('effective') is not True:
        errors.append('only prepared, not effective')
    rows = z.get('boards', [])
    ids = [r.get('id') for r in rows]
    expected = {b['id']: b for b in dataset['boards']}
    if len(ids) != 20 or len(set(ids)) != 20 or set(ids) != set(expected):
        errors.append('empty/incomplete/duplicated matrix')
    for r in rows:
        if r.get('board_sha') != expected.get(r.get('id'), {}).get('sha256'):
            errors.append('Board revision missing/different')
        if not SHA.fullmatch(r.get('copper_sha', '')) or not SHA.fullmatch(r.get('drc_sha', '')):
            errors.append('Copper/DRC evidence missing')
        d = r.get('drc') or {}
        schemas = {'drc-v1': ('faults', 'open_items', 'routing', 'not_connected'),
                   'corpus-drc-v1': ('all_items', 'slot', 'routing', 'not_connected')}
        keys = schemas.get(d.get('version'), ())
        if not keys or any(type(d.get(k)) is not int or d[k] < 0 for k in keys):
            errors.append('native DRC unknown')
        e = r.get('evaluation') or {}
        if type(e.get('is_valid')) is not bool:
            errors.append('Connection gate unknown')

        if e.get('is_valid') is True and (e.get('minimax_complete') is not True or
                                        (e.get('safety') or {}).get('safe') is not True):
            errors.append('Required evidence unknown')
        if e.get('is_valid') is True and not current_minimax(e):
            errors.append('outdated minimax contract')
        if e.get('is_valid') is True:
            errors.extend(measurement_errors(r))
    return sorted(set(errors))

def decision(row):
    return dict(row['evaluation'], design_rules=row.get('design_rules') or {})


def measurement_errors(row):
    from analysis import minimax as MM
    from analysis import rule_metrics as RK
    e = row['evaluation']
    errors = []
    if not MM.complete(e):
        errors.append('Minimax measured quantities missing')
    if not RK.assumption(row.get('design_rules') or {}, row.get('design_rules') or {})['not_worse']:
        errors.append('Rule metrics missing')
    if any(not number((e.get('minimax_current_paths') or {}).get(k)) for k in ('detour_mm', 'loss_mW')):
        errors.append('Minimax current path protection missing')
    safe = e.get('safety')
    if (not isinstance(safe, dict) or safe.get('safe') is not True or safe.get('status') != 'safe'
            or safe.get('faults') != [] or safe.get('absent') != []):
        errors.append('Safety evidence missing/contradictory')
    if e.get('minimax_valid', True) is not True:
        errors.append('Safety gate contradictory')
    if any(k in e and e[k] != row.get('design_rules') for k in ('design_rules', 'rule_gate')):
        errors.append('contradictory rule metrics')
    return errors


def current_minimax(e):
    from analysis import minimax as MM
    g = json.loads((ROOT / 'analysis/measures_weights.json').read_text())
    if e.get('score_v3_version', e.get('version')) == 'v3.2':
        si = json.loads((ROOT / 'analysis/measures_weights_si.json').read_text())
        for key in ('minimax', 'profile'):
            si.setdefault(key, g[key])
        g = si
    p = e.get('minimax_profile')
    return p in g['profile'] and e.get('minimax_version') == MM.version_contract(g, p, g.get('version', 'v3.1'), e.get('minimax_board_contract'))


def hard_valid(row):
    d = row['drc']
    keys = ('all_items', 'slot', 'routing', 'not_connected') if d['version'] == 'corpus-drc-v1' else ('faults', 'open_items', 'routing', 'not_connected')
    e = row['evaluation']
    return (e['is_valid'] is True and all(d[k] == 0 for k in keys) and
            e.get('minimax_complete') is True and (e.get('safety') or {}).get('safe') is True and
            current_minimax(e) and not measurement_errors(row))


def balance(dataset, old, new, now=None):
    t0 = time.process_time()
    report = dict(version=VERSION, status='unknown', cpu_s=None,
                  valid_per_hour=None, minimax_percent_per_hour=None)
    try:
        check_set(dataset)
        errors = snapshot_errors(old, dataset) + snapshot_errors(new, dataset)
        for k in ('evaluator_sha', 'budget_sha'):
            if old.get(k) != new.get(k):
                errors.append('different ' + k)
        times = [old.get('development_start'), old.get('activated'), new.get('development_start'), new.get('finished'), new.get('activated')]
        if not all(number(t) for t in times) or not (times[0] <= times[1] <= times[2] <= times[3] <= times[4] <= (now or time.time())):
            errors.append('Lifecycle times missing/invalid')
        elif dataset['created'] > times[0]:
            errors.append('Set sealed only after work began')
        if errors:
            report['reasons'] = sorted(set(errors))
            return report
        hours = (new['activated'] - old['development_start']) / 3600
        if hours <= 0:
            report['reasons'] = ['no elapsed time']
            return report
        a = {r['id']: r for r in old['boards']}
        b = {r['id']: r for r in new['boards']}
        ga, gb = {k for k in a if hard_valid(a[k])}, {k for k in b if hard_valid(b[k])}
        from analysis import minimax as MM
        improvements, regressions, missing = [], [], []
        lost, gained = sorted(ga - gb), sorted(gb - ga)
        details = [dict(id=k, category='validity', reason='hard-valid board lost') for k in lost]
        for k in sorted(ga & gb):
            ea, eb = a[k]['evaluation'], b[k]['evaluation']
            cmp = MM.assumption(decision(a[k]), decision(b[k]))
            if cmp['delta'] is None or not cmp['not_worse']:
                (missing if cmp['delta'] is None or 'missing' in cmp['reason'] or 'different' in cmp['reason'] else regressions).append(k)
                if k in missing:
                    continue
                details.append(dict(id=k, category='metrics', reason=cmp['reason'], delta=cmp['delta']))


            x, y = ea['minimax'], eb['minimax']
            if x > 0:
                improvements.append(100 * (x - y) / x)
            elif y == 0:
                improvements.append(0.0)
        if missing:
            report['reasons'] = ['Minimax/rule contract missing on jointly valid boards']
            return report
        report.update(status='regression' if lost or regressions else 'measured',
                      set_sha=dataset['sha256'], basis_code=old['code_sha'], effective_code=new['code_sha'],
                      hours=hours, valid_basis=len(ga), valid_effective=len(gb),
                      additional_valid=len(gb) - len(ga), common_valid=len(ga & gb),
                      valid_per_hour=(len(gb) - len(ga)) / hours,

                      minimax_percent_per_hour=min(improvements) / hours if improvements else None,
                      regressions=len(regressions) + len(lost), regression_details=details,
                      lost_valid=lost, new_valid=gained)
        return report
    except (ValueError, KeyError, TypeError, AttributeError) as e:
        report['reasons'] = [str(e)]
        return report
    finally:
        report['cpu_s'] = time.process_time() - t0

def main():
    if sys.argv[1:2] == ['operation']:
        import argparse
        p = argparse.ArgumentParser(description='Read-only data yield balance; no quality evidence')
        p.add_argument('operation')
        p.add_argument('data', type=Path)
        p.add_argument('--start', type=float)
        p.add_argument('--end', type=float, default=None)
        p.add_argument('--cur_state', type=Path)
        args = p.parse_args()
        end = args.end if args.end is not None else time.time()
        report = operating_balance(args.data, args.start if args.start is not None else end - 86400, end)
        if args.cur_state:
            report['input_'] = observe(args.data, args.cur_state, end)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False))
        return 1 if report['status'] == 'unknown' else 0
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    s, a, b = [json.loads(Path(p).read_text()) for p in sys.argv[1:]]
    report = balance(s, a, b)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report['status'] == 'measured' else 1


SOURCES = ('duration.jsonl', 'selfplay.jsonl')
STAGES = ('placer', 'router', 'shaping', 'netclasses_input', 'netclasses_placement',
          'zone_plan', 'current_zones', 'r11_ir', 'r24', 'r44')
ABORTS = {'time_limit', 'wall_limit', 'faults', 'infrastructure', 'SIGXCPU',
          'cpu_budget', 'early_abort', 'budget_before_post_run', 'code_changed', 'time_s'}


def rate_interval(n, hours):
    if hours <= 0:
        raise ValueError('positive exposure required')

    def quantile(k, target):
        def cdf(mu):
            if mu == 0:
                return 1.0
            return sum(math.exp(-mu + i * math.log(mu) - math.lgamma(i + 1)) for i in range(k + 1))
        lo, hi = 0.0, max(1.0, k + 1.0)
        while cdf(hi) > target:
            hi *= 2
        for _ in range(60):
            mid = (lo + hi) / 2
            if cdf(mid) > target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2
    return [quantile(n - 1, .975) / hours if n else 0.0,
            quantile(n, .025) / hours]


def completed_at(z):
    value = z.get('time_s')
    if not isinstance(value, str) or not number(z.get('s')):
        return None
    try:
        return datetime.strptime(value, '%Y-%m-%d %H:%M:%S').timestamp() + z['s']
    except ValueError:
        return None


def row_measure(z):
    validate_row(z)
    from learning.learning_effect import cpu_s, _limit_result
    children = [z.get(k) or {} for k in STAGES]
    stages = [s.get('compute_slot') or s for s in children]
    states = [z.get('cur_state', 'ok')] + [s.get(k) for s in children + stages for k in ('cur_state', 'reason')]
    waits = [s['wait_s'] for s in stages if number(s.get('wait_s'))]


    normalized = dict(z)
    for key in STAGES:
        child = dict(z.get(key) or {})
        if child.get('cpu_s') is None and number((child.get('compute_slot') or {}).get('cpu_s')):
            child['cpu_s'] = child['compute_slot']['cpu_s']
        if key in ('zone_plan', 'current_zones', 'r11_ir', 'r24') and not number((child.get('compute_slot') or {}).get('cpu_s')) and number(child.get('cpu_s')):
            child['compute_slot'] = dict(child.get('compute_slot') or {}, cpu_s=child['cpu_s'])
        normalized[key] = child
    cpu_sources = [normalized[k] for k in STAGES] + [z.get(k) or {} for k in
                    ('dataset', 'labels_candidates')]
    rb = ((z.get('racetrack') or {}).get('measurement') or {}).get('compute_slot') or {}
    has_cpu = (number(z.get('cpu_total')) or any(number(c.get('cpu_s')) for c in cpu_sources) or
               number(normalized['router'].get('cpu_owned_s')) or
               ('cpu_racetrack_measurement_s' not in z and number(rb.get('cpu_s'))))
    cpu = cpu_s(normalized) if has_cpu else None
    aborted = bool(z.get('code_changed') or z.get('cpu_time_limit') or z.get('budget_end') or
                   (z.get('learning_data') or {}).get('censored') or
                   (z.get('cpu_budget') or {}).get('overflow', 0) or
                   z.get('faults') or z.get('rule_error') or
                   any(s in ABORTS or any(s.endswith('_' + a) for a in ABORTS)
                       for s in states if s) or any(s.get('faults') for s in children))
    return dict(is_valid=int(not aborted and _limit_result(z) is True),
                abort=int(aborted),
                wall_limit=int(any(s == 'wall_limit' or s.endswith('_wall_limit') for s in states if s)),
                cpu_s=cpu if has_cpu else None,
                cpu_whole=int(z.get('cpu_measurement') in ('entire', 'whole_wait4')),
                wait_s=sum(waits), wait_records=len(waits),
                load=z.get('load') if number(z.get('load')) else None)


def validate_row(z):
    def obj(value, label):
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise ValueError('Row schema: ' + label + ' must be an object')
        return value

    def fields(value, label):
        for key in ('cpu_s', 'cpu_owned_s', 'wait_s', 'overflow'):
            if key in value and (key == 'overflow' or value[key] is not None) and not number(value[key]):
                raise ValueError('Row schema: ' + label + '.' + key)
        for key in ('cur_state', 'reason'):
            if value.get(key) is not None and not isinstance(value[key], str):
                raise ValueError('Row schema: ' + label + '.' + key)
        if 'cpu_contains' in value and (not isinstance(value['cpu_contains'], list) or
                                        any(not isinstance(k, str) for k in value['cpu_contains'])):
            raise ValueError('Row schema: ' + label + '.cpu_contains')
        if 'compute_slot' in value:
            if not isinstance(value['compute_slot'], dict):
                raise ValueError('Row schema: ' + label + '.compute_slot')
            fields(obj(value['compute_slot'], label + '.compute_slot'), label + '.compute_slot')

    obj(z, 'Row')
    if not isinstance(z, dict):
        raise ValueError('Row schema: row must be an object')
    fields(z, 'Row')
    for key in ('cpu_total', 'cpu_racetrack_measurement_s', 's', 'load'):
        if key in z and z[key] is not None and not number(z[key]):
            raise ValueError('Row schema: ' + key)
    for key in ('is_valid', 'cpu_time_limit', 'budget_end'):
        if z.get(key) is not None and type(z[key]) is not bool:
            raise ValueError('Row schema: ' + key)
    if z.get('time_s') is not None and not isinstance(z['time_s'], str):
        raise ValueError('Row schema: time_s')
    for key in STAGES + ('dataset', 'labels_candidates', 'learning_data', 'cpu_budget'):
        fields(obj(z.get(key), key), key)
    shaping = z.get('shaping') or {}
    if 'applied' in shaping and type(shaping['applied']) is not bool:
        raise ValueError('Row schema: shaping.applied')
    ld = z.get('learning_data') or {}
    if ld.get('censored') is not None and type(ld['censored']) is not bool:
        raise ValueError('Row schema: learning_data.censored')
    rb = obj(z.get('racetrack'), 'racetrack')
    fields(obj(rb.get('measurement'), 'racetrack.measurement'), 'racetrack.measurement')


def empty_counts():
    return dict(lines=0, is_valid=0, abort=0, wall_limit=0, cpu_s=0.0,
                cpu_unknown=0, cpu_whole=0, wait_s=0.0, wait_records=0,
                load_n=0, load_total_sum=0.0, load_over_cores_n=0)


def add_row(counts, z, cores):
    m = row_measure(z)
    counts['lines'] += 1
    for k in ('is_valid', 'abort', 'wall_limit', 'cpu_whole', 'wait_s', 'wait_records'):
        counts[k] += m[k]
    if m['cpu_s'] is None:
        counts['cpu_unknown'] += 1
    else:
        counts['cpu_s'] += m['cpu_s']
    if m['load'] is not None:
        counts['load_n'] += 1
        counts['load_total_sum'] += m['load']
        counts['load_over_cores_n'] += int(m['load'] > cores)


def finish_counts(counts, hours):
    result = dict(counts)
    result.update(valid_per_hour=counts['is_valid'] / hours,
                  valid_per_hour_ci95=rate_interval(counts['is_valid'], hours),
                  cpu_per_valid_lower_bound_s=counts['cpu_s'] / counts['is_valid'] if counts['is_valid'] else None,
                  abort_share=counts['abort'] / counts['lines'] if counts['lines'] else None,
                  load_medium=counts['load_total_sum'] / counts['load_n'] if counts['load_n'] else None,
                  load_over_cores_share=counts['load_over_cores_n'] / counts['load_n'] if counts['load_n'] else None)
    return result


def complete_lines(path, offset=0):
    with path.open('rb') as f:
        yield from descriptor_lines(f, os.fstat(f.fileno()).st_size, offset)


def descriptor_lines(f, end, offset=0, consumed=None):
    f.seek(offset)
    while f.tell() < end:
        line = f.readline(end - f.tell())
        if not line.endswith(b'\n'):
            break
        if consumed is not None:
            consumed.update(line)
        try:
            z = json.loads(line)
            validate_row(z)
        except (ValueError, UnicodeDecodeError, RecursionError):
            z = None
        yield f.tell(), z


def prefix_sha(f, end, accumulator=None):
    f.seek(0)
    h = accumulator if accumulator is not None else hashlib.sha256()
    remaining = end
    while remaining:
        chunk = f.read(min(remaining, 1024 * 1024))
        if not chunk:
            return None
        h.update(chunk)
        remaining -= len(chunk)
    return h.hexdigest()


def operating_balance(data, start, end, cores=None):
    import os
    if not number(start) or not number(end) or end <= start:
        raise ValueError('invalid measurement window')
    hours = (end - start) / 3600
    sources = {}
    for name in SOURCES:
        counts = empty_counts()
        hourly = [0] * math.ceil(hours)
        untimed = total = malformed = offset = 0
        path = Path(data) / name
        try:
            rows = complete_lines(path)

            rows = iter(rows)
            first = next(rows, None)
        except FileNotFoundError:
            sources[name] = dict(status='missing', valid_per_hour=None)
            continue
        from itertools import chain
        for offset, z in chain(() if first is None else (first,), rows):
            if z is None:
                malformed += 1
                continue
            total += 1
            t = completed_at(z) if name == 'selfplay.jsonl' else None
            if t is None:
                untimed += 1
            elif start <= t < end:
                add_row(counts, z, cores or os.cpu_count() or 1)
                hourly[int((t - start) // 3600)] += row_measure(z)['is_valid']
        sources[name] = dict(finish_counts(counts, hours),
                             status='partially' if untimed or malformed else 'measured',
                             lines_file=total, without_time=untimed, defective=malformed,
                             prefix_bytes=offset, time_base='Start time+s; before gallery/append' if name.startswith('itself') else 'no historical time evidence')


        if hours >= 6 and hours == int(hours) and name == 'selfplay.jsonl':
            import random
            rnd = random.Random(14010)
            draws = sorted(sum(rnd.choice(hourly) for _ in hourly) / hours for _ in range(2000))
            sources[name]['hour_block_ci95'] = [draws[49], draws[1949]]
            sources[name]['hours_valid'] = hourly
        if name == 'duration.jsonl' and untimed:
            for key in ('valid_per_hour', 'valid_per_hour_ci95'):
                sources[name][key] = None
    missing = [name for name in SOURCES if sources[name]['status'] == 'missing']
    return dict(version='learning_mode-v1', status='unknown' if len(missing) == len(SOURCES) else 'DataYield',
                missing_sources=missing, measurement_basis='missing' if len(missing) == len(SOURCES) else
                'partially' if missing or any(s['status'] == 'partially' for s in sources.values()) else 'complete',
                start=start, end=end,
                hours=hours, source_list=sources, quality_gain=None,
                band_assumption='95% Poisson, independent events; candidates are correlated. No causal effect band.',
                limits=['CPU is a historical lower bound; unsaved/active attempts missing.',
                         'Load is an event-weighted systematic sample, not a time series or double-occupancy count.',
                         'Wait time only explicit fields; admission/reserve wait time and rejected children missing.'])


def observe(data, state_path, now=None, cores=None):
    import os
    import fcntl
    now = time.time() if now is None else now
    data, state_path = Path(data).resolve(), Path(state_path).resolve()
    if state_path == data or data in state_path.parents:
        raise ValueError('Observer state must not be in learning data')
    state_path.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(state_path) + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        old = json.loads(state_path.read_text()) if state_path.exists() else None
        if old and (old.get('version') not in ('learning_input-v1', 'learning_input-v2') or
                    old.get('data') != str(data) or now <= old['epoch']):
            raise ValueError('Observer state/source/time does not match')
        cursors, sources = {}, {}
        for name in SOURCES:
            path = data / name
            try:
                f = path.open('rb')
            except FileNotFoundError:
                sources[name] = dict(status='missing')
                continue
            with f:
                st = os.fstat(f.fileno())
                prev = (old or {}).get('cursors', {}).get(name)
                identity = [st.st_dev, st.st_ino]
                reset = (not prev or prev['identity'] != identity or st.st_size < prev['offset'] or
                         not prev.get('prefix_sha256') or prefix_sha(f, prev['offset']) != prev['prefix_sha256'])
                counts = empty_counts()
                offset = st.st_size if reset else prev['offset']
                malformed = 0
                if reset:

                    while offset > 0:
                        f.seek(offset - 1)
                        if f.read(1) == b'\n': break
                        offset -= 1
                else:
                    consumed = hashlib.sha256()
                    prefix_sha(f, offset, consumed)
                    for offset, z in descriptor_lines(f, st.st_size, offset, consumed):
                        if z is None: malformed += 1
                        else: add_row(counts, z, cores or os.cpu_count() or 1)
                binding = prefix_sha(f, offset)


                if not reset and (binding != consumed.hexdigest() or
                                  prefix_sha(f, prev['offset']) != prev['prefix_sha256']):
                    reset = True
                    binding = None
            cursors[name] = dict(identity=identity, offset=offset, prefix_sha256=binding)
            sources[name] = (dict(status='initial_state' if not old else 'gap_file_switch') if reset else
                             dict(finish_counts(counts, (now - old['epoch']) / 3600),
                                  status='partially' if malformed else 'measured', defective=malformed))
        state = dict(version='learning_input-v2', data=str(data), epoch=now, cursors=cursors)
        temp = Path(str(state_path) + '.tmp')
        temp.write_text(json.dumps(state, allow_nan=False))
        temp.replace(state_path)
        return dict(version='learning_input-v2', start=old['epoch'] if old else None, end=now,
                    time_base='newly appended completed rows since baseline snapshot', source_list=sources)


if __name__ == '__main__':
    sys.exit(main())
