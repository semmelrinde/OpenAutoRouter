from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import random
import tempfile

VERSION = 4
QUOTA_WINDOW = 100
HISTORY_WINDOW = 100
BGA_MEASURE_SERIES_FREE = False


def category(z):
    if (z.get('learn_axis') or {}).get('package') in ('bga', 'qfn'):
        return z['learn_axis']['package']
    return 'bga' if any('BGA' in b.upper() for b in z.get('blocks', [])) else 'qfn'


def keyname(package, cu_layers):
    return '%s_L%d' % (package, cu_layers)


def fpath(kw):
    return Path(kw['data']).with_suffix('.axes.json')


@contextmanager
def cur_state(kw):
    p = fpath(kw); p.parent.mkdir(parents=True, exist_ok=True)
    with open(str(p) + '.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            S = json.loads(p.read_text()) if p.exists() else dict(version=VERSION, axes={}, tasks={}, bga_window=[], power={}, reserved_n=0, reserved_bga=0, archive_until=-1)
            if S.get('version') in (1, 2):



                for cell in S['axes'].values():
                    cell['window'] = []
                    cell['generation'] = cell.get('generation', -1) + 1
                for row in S['tasks'].values(): row['generation'] = -1
                S['bga_window'] = []
                if S['version'] == 1:
                    S['quota_window'] = [S['tasks'][k]['package']
                        for k in sorted(S['tasks'], key=int)][-QUOTA_WINDOW:]
                S['version'] = VERSION
            if S.get('version') == 3:



                S['bga_window'] = []
                S['version'] = VERSION
            if S.get('version') != VERSION:
                raise ValueError('Unknown axis version')
            for cell in S['axes'].values():
                history = cell.get('history', [])
                cell['history_n'] = max(cell.get('history_n', 0), len(history))
                cell['history'] = history[-HISTORY_WINDOW:]
                cell['history_archived'] = cell['history_n'] - len(cell['history'])
            yield S
            fd, tmp = tempfile.mkstemp(dir=p.parent, prefix=p.name + '.')
            try:
                with os.fdopen(fd, 'w') as f:
                    json.dump(S, f, indent=1); f.flush(); os.fsync(f.fileno())
                os.replace(tmp, p)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def reserve(kw, seed, level_choice=None):
    with cur_state(kw) as S:
        key = str(seed)
        if key in S['tasks']: return dict(S['tasks'][key])
        if seed <= S.get('archive_until', -1):
            raise ValueError('Task number already archived; use new seed0')
        fen = S['bga_window'][-20:]
        vacant = len(fen) >= 8 and sum(fen) / len(fen) >= .25
        cap = (.5 if vacant else .35) if BGA_MEASURE_SERIES_FREE else .10
        total = S.get('reserved_n', len(S['tasks']))
        bga_n = S.get('reserved_bga', sum(x['package'] == 'bga' for x in S['tasks'].values()))
        quote = S.setdefault('quota_window', [])[-(QUOTA_WINDOW - 1):]
        package = 'bga' if (quote.count('bga') + 1) * 100 <= (len(quote) + 1) * int(cap * 100) else 'qfn'
        S['quota_window'] = quote + [package]
        rnd = random.Random(seed * 7919 + 29)

        layers = int(kw.get('cu_layers', '0')) or rnd.choice([4, 6, 8])
        if layers not in (4, 6, 8): raise ValueError('Axes: cu_layers=4|6|8')
        axis = keyname(package, layers)
        init = float(kw.get('level', '0'))
        cell = S['axes'].setdefault(axis, dict(level=init, window=[], history=[], generation=0))
        st = cell['level'] if level_choice is None else level_choice(cell['level'], dict(package=package, cu_layers=layers))
        row = dict(package=package, cu_layers=layers, axis=axis, level=st, bga_cap=cap,
                   bga_quote=sum(fen) / len(fen) if fen else None, cur_state='reserved', generation=cell['generation'])
        S['tasks'][key] = row
        S['reserved_n'] = total + 1
        S['reserved_bga'] = bga_n + (package == 'bga')


        for old in sorted(S['tasks'], key=int):
            if len(S['tasks']) <= 1000: break
            if S['tasks'][old]['cur_state'] not in ('finished', 'aborted'): continue
            S['archive_until'] = max(S.get('archive_until', -1), int(old))
            del S['tasks'][old]
        return dict(row)


def report(kw, seed, succeeded, cpu_s=None, valid_ones=0, generator_failure=False, aborted=False):
    with cur_state(kw) as S:
        if str(seed) not in S['tasks'] and seed <= S.get('archive_until', -1): return
        row = S['tasks'][str(seed)]
        if row['cur_state'] in ('finished', 'aborted'): return
        if aborted and succeeded is not True: succeeded = None
        row.update(censored=succeeded is None, cur_state='aborted' if aborted else 'finished', succeeded=succeeded, cpu_s=cpu_s, valid_ones=valid_ones, generator_failure=generator_failure)
        perf = S.setdefault('power', {}).setdefault(row['axis'],
                    dict(tasks=0, succeeded=0, valid_ones=0, cpu_s=0., cpu_unknown=0))
        perf['tasks'] += 1
        if isinstance(cpu_s, (int, float)) and cpu_s > 0:
            perf['cpu_s'] += cpu_s
            perf['succeeded'] += succeeded is True
            perf['valid_ones'] += valid_ones
        else: perf['cpu_unknown'] += 1
        perf['valid_pro_cpu_h'] = perf['valid_ones'] * 3600 / perf['cpu_s'] if perf['cpu_s'] else None
        if succeeded is None:
            perf['censored'] = perf.get('censored', 0) + 1
            return
        cell = S['axes'][row['axis']]

        if row['generation'] != cell['generation'] or row['level'] != cell['level']:
            cell['outdated'] = cell.get('outdated', 0) + 1
            return
        if row['package'] == 'bga': S['bga_window'] = (S['bga_window'] + [bool(succeeded)])[-20:]
        cell['window'].append(bool(succeeded))
        if len(cell['window']) == 10:
            n = sum(cell['window']); old = cell['level']
            cell['level'] = round(max(0., old + (.5 if n > 6 else -.5 if n < 4 else 0.)), 2)
            cell['history'].append(dict(seed=seed, successes=n, n=10, from_=old, level=cell['level']))
            cell['history_n'] += 1
            cell['history'] = cell['history'][-HISTORY_WINDOW:]
            cell['history_archived'] = cell['history_n'] - len(cell['history'])
            cell['window'] = []
            cell['generation'] += 1


def efficiency(lines):
    out = {}
    for z in lines:
        for key in (category(z), 'L%d' % z['cu_layers'], keyname(category(z), z['cu_layers'])):
            s = out.setdefault(key, dict(n=0, is_valid=0, cpu_s=0., cpu_unknown=0, labels=0))
            s['n'] += 1
            cpu = z.get('cpu_total')
            if isinstance(cpu, (int, float)) and cpu > 0:
                s['cpu_s'] += cpu; s['is_valid'] += z.get('is_valid') is True
            else: s['cpu_unknown'] += 1
            s['labels'] += (z.get('dataset') or {}).get('labels_n', 0)
    for s in out.values():
        s['valid_pro_cpu_h'] = s['is_valid'] * 3600 / s['cpu_s'] if s['cpu_s'] else None
    return out
