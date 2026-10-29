import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

ARMS = {
    'via_density': ('router', {'via_density': '1'}),
    'r27_trunk': ('router', {'r27_trunk': '1'}),
    'branch': ('router', {'branch': '1'}),
    'bend': ('router', {'bend': '.2'}),
    'bend_current': ('router', {'bend': '.2', 'bend_current': '0'}),
    'dense': ('placer', {'dense': '2'}),
    'dense_inner': ('placer', {'dense_inner': '2'}),
    'bga_fanout': ('router', {'bga_fanout': '1'}),
    'current_via_price': ('router', {'current_via_price': '1'}),
    'current_first': ('router', {'current_first': '1'}),
    'p35': ('placer', {'p35': '1'}),
    'r24': ('cfg', {'r24': True}),
    'r19_shape': ('router', {'r19_shape': '1'}),
    'pair_search_space': ('router', {'pair': '1', 'pair_search_space': '1'}),
    'hist_start': ('hist', {}),
}
ALPHA = .05
MIN_FAMILIES = 8


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def code_identical(expected, results, actual):
    return actual == expected and all(all(z.get(k) == v for k, v in expected.items()) for z in results)


def family(A):
    import re
    declared = A.get('family') or (A.get('lineage') or {}).get('family')
    if declared: return declared
    skeleton = [re.sub(r'\d+(?:[.,]\d+)?', '#', b) for b in A['blocks']]
    return 'block_skeleton:' + digest(sorted(skeleton))


def applicable(arm, A, history=None):
    if arm == 'hist_start': return history is not None and Path(history).exists()
    if arm == 'bga_fanout': return A['cu_layers'] >= 6 and any('BGA' in b for b in A['blocks'])
    if arm == 'pair_search_space': return bool(A.get('length_groups'))
    return True


def configure(cfg, arm, an, history=None):
    c = copy.deepcopy(cfg)
    kind, values = ARMS[arm]
    if kind == 'hist':
        c.pop('hist_start', None)
        if an:
            if not history: raise ValueError('Self-imitation without predecessor')
            c['hist_start'] = str(Path(history).resolve())
    elif kind == 'router':
        from learning.selfplay import ROUTER_VARIANTS
        r = c.get('router', 'prior')
        r = list(ROUTER_VARIANTS[r] if isinstance(r, str) else r)


        if arm == 'bend_current' and not an:
            c['router'] = r
            return c
        r = [v for v in r if v.split('=', 1)[0] not in values]

        if arm == 'pair_search_space': r += ['pair=1']
        if arm == 'bend_current':
            r += ['bend=.2', 'bend_current=0']
        else:
            r += [f'{k}={v if an else 0}' for k, v in values.items() if k != 'pair']
        c['router'] = r
    else:
        c.update({k: (v if an else (False if isinstance(v, bool) else '0')) for k, v in values.items()})
    return c


def _measurement(ad, kid, z):
    from analysis import evaluation as BW
    from analysis import rule_metrics as RK
    from analysis.current_chain import link_chain
    c = Path(ad) / f'k{kid}'
    try:
        from learning.selfplay import code_hashes
        before = code_hashes()
        if z.get('code_changed') or any(z.get(k) != v for k, v in before.items()):
            raise ValueError('Code change within attempt')
        B, R, K = [json.loads((c / f'{n}.json').read_text()) for n in ('board', 'routes', 'netclasses')]
        R['gnd_layers'] = z.get('gnd_layers', [])
        E = BW.rate(B, R, K, v3=True, minimax=False)
        M = E['design_rules']
        if any(RK.amount(M, k) is None for k in RK.gate_fields(M)):
            raise ValueError('Rule gate incomplete')
        chain = link_chain(B, R, K)

        needs = {n for n, v in K['net_list'].items() if any(finite(i) and i > 1e-6 for i in ((v.get('flow') or {}).get('load') or {}).values()) and
                 v.get('category') in ('current_val', 'supply', 'sw')}
        complete = not chain.get('skipped') and needs <= set(chain['net_list'])
        if chain['net_list']:
            complete = complete and all(finite(v.get('element')) and finite(v.get('efficiency')) for v in chain['net_list'].values())
        elif needs: complete = False
        if code_hashes() != before: raise ValueError('Code change in measurement')
        return dict(completed=z.get('cur_state') == 'ok', is_valid=z.get('is_valid') is True and E['is_valid'] is True,
                    design_rules=M, link_chain=chain, chain_complete=bool(complete), version=E['score_v3_version'])
    except Exception as e:
        return dict(is_valid=z.get('is_valid') is True, faults=repr(e))

def measurement(ad, kid, z):
    from common import slots as PL
    from learning.selfplay import VPY
    c = Path(ad) / f'k{kid}'
    c.mkdir(parents=True, exist_ok=True)
    if any(not (c/f'{name}.json').exists() for name in ('board', 'routes', 'netclasses')):
        return dict(is_valid=z.get('is_valid') is True, completed=z.get('cur_state') == 'placer_collision',
                    faults='Attempt without complete artifacts', compute_slot=dict(cpu_s=0., cur_state='not_started'))
    input_file = c/'racetrack_input.json'
    input_file.write_text(json.dumps(z))
    result = PL.job([VPY, str(Path(__file__).resolve()), 'measure', str(ad), str(kid)],
                     cpu_s=180, log=str(c/'racetrack_measurement.log'),
                     env=dict(os.environ, PYTHONHASHSEED='0'))
    output = c/'racetrack_measurement.json'
    m = json.loads(output.read_text()) if result['cur_state'] == 'ok' and output.exists() else dict(
        is_valid=z.get('is_valid') is True, faults='Measurement '+result['cur_state'])
    m['compute_slot'] = result
    return m


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def pair_value(m):
    n = m['design_rules']['score_v3']
    c = m['link_chain']
    if not c['net_list']: return math.log1p(n)
    return math.log1p(n) - math.log(max(1e-12, c['efficiency'])) + math.log1p(c['element_max'])


def compare(a, b):
    from analysis import rule_metrics as RK
    out = dict(valid_delta=int(b['is_valid'])-int(a['is_valid']), gate_check=False, measured=False, delta=None)
    if not a['is_valid'] and not b['is_valid'] and a.get('completed') and b.get('completed'):
        return dict(out, gate_check=True, validity_measured=True, reason='both_invalid')
    if a.get('completed') is False or b.get('completed') is False:
        return dict(out, reason='censored attempt')
    if not a['is_valid'] and b['is_valid'] and a.get('completed'):


        if b.get('faults') or not b.get('version') or not b.get('chain_complete') or any(
                RK.amount(b.get('design_rules', {}), k) is None for k in RK.gate_fields(b.get('design_rules', {}))):
            return dict(out, reason='Validity gain measured incompletely')
        return dict(out, gate_check=True, validity_measured=True, reason='validity_gain')
    if a.get('faults') or b.get('faults'): return dict(out, reason='MeasurementError')
    if a.get('version') != b.get('version'): return dict(out, reason='EvaluationContract')
    if not a['is_valid'] or not b['is_valid']:
        return dict(out, reason='invalid pair')
    if not a.get('chain_complete') or not b.get('chain_complete'):
        return dict(out, reason='Chain measurement incomplete')
    gate = RK.assumption(a['design_rules'], b['design_rules'])
    same = set(a['link_chain']['net_list']) == set(b['link_chain']['net_list'])
    out.update(gate_check=gate['not_worse'] and same, measured=True, rule_comparison=gate,
               validity_measured=True,
               delta=pair_value(b)-pair_value(a), reason=gate['reason'] if same else 'other chain nets')
    return out


def statistics(rows, alpha=ALPHA):
    families = {}
    for r in rows:
        if r.get('evidence', True) and r['comparison'].get('measured'):
            families.setdefault(r['family'], r['comparison']['delta'])
    family_deltas = {}
    for r in rows:
        if r['comparison'].get('measured'):
            family_deltas.setdefault(r['family'], []).append(r['comparison']['delta'])
    mean_delta = sum(sum(v)/len(v) for v in family_deltas.values()) / len(family_deltas) if family_deltas else None
    signs = [1 if v < -1e-9 else -1 if v > 1e-9 else 0 for v in families.values()]
    def e_value(signs):
        return sum(math.exp(min(690., sum(math.log1p(lam*s) for s in signs))) for lam in (.1, .25, .5, .75)) / 4
    e = e_value(signs)
    validity = {}
    for r in rows:
        if r.get('evidence', True) and r['comparison'].get('validity_measured'):
            validity.setdefault(r['family'], r['comparison']['valid_delta'])
    ev = e_value(list(validity.values()))
    bad = any(r['comparison']['valid_delta'] < 0 or
              (r['comparison'].get('measured') and not r['comparison']['gate_check']) for r in rows)
    missing = any(not r['comparison'].get('measured') and r['comparison'].get('reason') not in ('validity_gain', 'both_invalid') for r in rows)
    threshold = 2 * len(ARMS) / alpha
    quality_win = mean_delta is not None and mean_delta < -1e-9 and len(families) >= MIN_FAMILIES and e >= threshold
    validity_win = (len(validity) >= MIN_FAMILIES and sum(validity.values()) > 0 and ev >= threshold
                    and (mean_delta is None or mean_delta <= 1e-9))
    return dict(n=len(rows), families=len(families), delta_families_medium=mean_delta, siege=signs.count(1), defeats=signs.count(-1), e=e,
                validity_families=len(validity), e_validity=ev, alpha=alpha, threshold=threshold,
                gate_check=not bad, complete=not missing,
                promote=not bad and not missing and (quality_win or validity_win))


class Racetrack:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def transact(self, fn):
        with open(str(self.path)+'.lock', 'a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            d = json.loads(self.path.read_text()) if self.path.exists() else dict(format='racetrack-v1', contexts={})
            result = fn(d)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, suffix='.json')
            try:
                with os.fdopen(fd, 'w') as out: json.dump(d, out, indent=1, allow_nan=False)
                os.replace(tmp, self.path)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            return result

    @staticmethod
    def error_budget(d, context, generation):


        trials = d.setdefault('checks', [])
        for ctx, c in sorted(d['contexts'].items()):
            for gen in range(c['generation'] + 1):
                key = [ctx, gen]
                if key not in trials: trials.append(key)
        key = [context, generation]
        if key not in trials: trials.append(key)
        n = trials.index(key) + 1
        return ALPHA / (n * (n + 1))

    def choose(self, context, available, rnd, pinned=None):
        def choose(d):
            c = d['contexts'].setdefault(context, dict(standards=[], generation=0, pairs=[], reserved={}))
            self.error_budget(d, context, c['generation'])
            rows = [r for r in c['pairs'] if r['generation'] == c['generation']]
            if pinned:
                if pinned not in available: raise ValueError('Arm not applicable: '+pinned)
                arm = pinned
            else:
                allowed = [a for a in available if a not in c.get('blocked', [])]
                if not allowed: return None, list(c['standards']), c['generation']
                def priority(a):
                    st = statistics([r for r in rows if r['arm'] == a])
                    pending = c['reserved'].get(a, 0)
                    return (st['n'] + pending > 0, rnd.betavariate(1+st['defeats'], 1+st['siege']), st['n']+pending, a)
                arm = min(allowed, key=priority)
            c['reserved'][arm] = c['reserved'].get(arm, 0)+1
            return arm, list(c['standards']), c['generation']
        return self.transact(choose)

    def report(self, context, row):
        row = copy.deepcopy(row)
        def update(d):
            c = d['contexts'][context]
            c['reserved'][row['arm']] = max(0, c['reserved'].get(row['arm'], 1)-1)

            if any(r['id'] == row['id'] for r in c['pairs']): return dict(action='doubled')


            row['evidence'] = not any(r['arm'] == row['arm'] and r['family'] == row['family']
                and (r['comparison'].get('measured') or r['comparison'].get('validity_measured'))
                for prior in d['contexts'].values() for r in prior['pairs'])
            c['pairs'].append(row)
            arm = row['arm']; cmp = row['comparison']



            regression = (cmp['valid_delta'] < 0 or
                (not cmp.get('measured') and cmp.get('reason') not in ('validity_gain', 'both_invalid')) or
                (cmp.get('measured') and (not cmp['gate_check'] or cmp['delta'] > 1e-9)))
            if regression:
                affected = set(row.get('standards', [])) | {arm}
                c.setdefault('blocked', [])[:] = sorted(set(c.get('blocked', [])) | affected)
                if c['standards']:
                    before = list(c['standards'])
                    c['standards'] = []; c['generation'] += 1
                    return dict(action='downgrade', removed=before, standards=[],
                                generation=c['generation'], late=row['generation'] != c['generation']-1)
            if row['generation'] != c['generation']:
                return dict(action='old_generation', gate_check=not regression, standards=list(c['standards']), generation=c['generation'])
            st = statistics([r for r in c['pairs'] if r['generation'] == c['generation'] and r['arm'] == arm],
                           self.error_budget(d, context, c['generation']))
            action = 'measure'
            if arm not in c['standards'] and st['promote'] and arm not in c.get('blocked', []):
                c['standards'].append(arm); c['standards'].sort(); action = 'promote'
            if action != 'measure': c['generation'] += 1
            return dict(action=action, statistics=st, standards=list(c['standards']), generation=c['generation'])
        return self.transact(update)



CFG_OPTIONS = ('learning_budget', 'learn_cpu_max', 'learning_policy', 'budget_before_post_run', 'learning_data',
                'pair_quality', 'r36_smoothed', 'r36_corridor', 'current_shape', 'r11_ir',
                'k20_zones', 'shaping', 'legal_lp', 'r39', 'r24', 'r27_shape_gate')
BOOL_OPTIONS = set(CFG_OPTIONS) - {'learning_budget', 'learn_cpu_max', 'learning_policy', 'budget_before_post_run', 'learning_data', 'shaping'}


def options(kw):
    for key, default in (('selection', 'all_items'), ('densify', '0'), ('teacher', '0'),
                         ('potential', '0'), ('learning_limit', '0')):
        if key in kw and str(kw[key]) != default:
            raise ValueError('Racetrack supports '+key+'='+str(kw[key])+' not')

    for key in ('value_shadow', 'start_limit', 'power_board', 'bga_axis', 'total_cpu'):
        if key in kw: raise ValueError('Racetrack supports option '+key+' not')
    allowed = set(CFG_OPTIONS) | {'selection', 'densify', 'teacher', 'potential', 'learning_limit',
        'value_shadow', 'start_limit', 'power_board', 'bga_axis', 'total_cpu',
        'racetrack', 'racetrack_arm', 'racetrack_state', 'goal', 'zone_plan', 'r36',
        'data', 'stop', 'time_s', 'iter', 'attempts', 'keep', 'level', 'anchor_legal', 'power_share', 'blocks',
        'tasks', 'parallel', 'seed0', 'frozen', 'gallery',
        'value_train', 'value_model', 'value_new', 'value_cpu', 'value_route', '_value_service'}
    unknown = set(kw)-allowed
    if unknown: raise ValueError('Racetrack: unknown/supported options missing: '+','.join(sorted(unknown)))
    if kw.get('gallery', '0') != '0': raise ValueError('Racetrack does not support gallery=1')
    for key in BOOL_OPTIONS & set(kw):
        if str(kw[key]).lower() not in ('1','0','true','false'):
            raise ValueError('Racetrack: '+key+' must be 0 or 1')
    return {k: (str(kw[k]).lower() in ('1', 'true') if k in BOOL_OPTIONS else kw[k])
            for k in CFG_OPTIONS if k in kw}


def balance(*part_list):
    known = all(finite(v) and v >= 0 for v in part_list)
    return dict(cpu_s=sum(part_list) if known else None, complete=known,
                components=list(part_list))


def meas_cpu(z, measurement):
    z['cpu_attempt_intake_s'] = z.get('cpu_total')
    charge = measurement.get('compute_slot', {}).get('cpu_s')
    z['cpu_racetrack_measurement_s'] = charge
    total = balance(z.get('cpu_total'), charge)
    z['cpu_total'] = total['cpu_s']
    z['cpu_measurement'] = 'whole_wait4' if total['complete'] else 'unknown'


def worker(ad, A, cfg, kid, time_s, it, phase='attempt'):
    from common import slots as PL
    from learning import selfplay as SS
    c = Path(ad)/f'k{kid}'; c.mkdir(parents=True, exist_ok=True)
    inp, out = c/(phase+'_input.json'), c/(phase+'_out_port.json')
    inp.write_text(json.dumps(dict(A=A, cfg=cfg, time_s=time_s, it=it)))
    out.unlink(missing_ok=True)
    result = PL.job([SS.VPY, str(Path(__file__).resolve()), phase, str(ad), str(kid)],
                     cpu_s=None, log=str(c/(phase+'_worker.log')),
                     env=dict(os.environ, PYTHONHASHSEED='0'))
    if result['cur_state'] == 'ok' and out.exists(): z = json.loads(out.read_text())
    else: z = dict(is_valid=False, cur_state=result['cur_state'], faults='Racetrack worker without result')
    z['cpu_total'] = result['cpu_s']; z['cpu_measurement'] = 'whole_wait4'
    z['racetrack_compute_slot'] = result
    return z


def play(out, seed, kw, lines, rnd):
    from learning import selfplay as SS
    SS.options_verify(kw)
    overrides = options(kw)
    task_cpu_start = time.thread_time()
    goal = SS.default_target(explicit=kw.get('goal'), context='selfplay')
    if goal != 'v3': raise ValueError('racetrack=1 supports only goal=v3; goal=minimax needs its own pair gate')
    mode = kw.get('zone_plan', 'off')
    if mode not in ('off', 'diagnose', 'local'): raise ValueError('zone_plan=off|diagnose|local')
    level = SS.level_read(kw)[1]['level']
    ad = Path(out).resolve() / f'task_{seed:06d}'
    A = SS.task_generate(str(ad), seed, goal=goal, level=level,
                            anchor_legal=kw.get('anchor_legal', '0') == '1',
                            power_share=float(kw.get('power_share', '0')),
                            **({'blocks': kw['blocks']} if 'blocks' in kw else {}))
    time_s, it = int(kw.get('time_s', '300')), int(kw.get('iter', '40'))
    SS.ZONE_PLAN.update(mode=mode, r36=kw.get('r36', 'geometric'))

    from common import slots as PL
    rg = PL.job([SS.VPY, str(Path(SS.R)/'analysis/netclasses.py'), str(ad/'board.json'), str(ad/'netclasses.json'),
                  'design_rules='+str(ad/'design_rules.json'), 'specs='+str(ad/'specs.json')], cpu_s=60,
                 log=str(ad/'netclasses_input.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
    if rg['cur_state'] != 'ok' or not (ad/'netclasses.json').exists():
        raise ValueError('Racetrack: input classification '+rg['cur_state'])
    hashes = SS.code_hashes()
    context = digest(dict(code=hashes['code_sha256'], net_class=SS.net_class(A), time_s=time_s, iter=it,
                          generator=A['version'], zone_list=kw.get('r36', 'geometric'), zone_plan=mode, goal=goal, shaping=kw.get('shaping'),
                          options=overrides, protocol='racetrack-v3',
                          **({'shape_arms': kw['shape_arms']} if 'shape_arms' in kw else {})))
    task = digest({n: hashlib.sha256((ad/n).read_bytes()).hexdigest() for n in ('board.json', 'design_rules.json', 'specs.json')})
    track = Racetrack(kw.get('racetrack_state', str(Path(out)/'racetrack.json')))
    trainer = None
    if kw.get('value_train', '0') == '1':
        from learning.value_model import Service
        trainer = kw.get('_value_service') or Service([kw['data']], kw.get('value_model', str(Path(out)/'amount')),
            new=int(kw.get('value_new', '100')), cpu_s=int(kw.get('value_cpu', '300')), route=int(kw.get('value_route', '4')))
    rows = []; history = None
    if kw.get('racetrack_arm') == 'hist_start':
        cfg = SS.choice(lines, SS.net_class(A), rnd, goal='v3', shape_arms=kw.get('shape_arms'))
        cfg.update(hist_context=task, learning_data='1', dataset='0', zone_plan=mode, r36=kw.get('r36', 'geometric'))
        cfg.update(overrides)
        warmup = worker(str(ad), A, cfg, -1, time_s, it)
        warmup['racetrack'] = dict(phase='predecessor', context=context)
        warmup['design_family'] = family(A)
        warmup = capture(str(ad), A, warmup, -1, time_s, it)
        with SS.LOCK:
            with open(kw['data'], 'a') as f: f.write(json.dumps(warmup)+'\n')
        rows.append(warmup)
        history = ad/'k-1'/'labels.npz'
        if not history.exists(): raise ValueError('Self-imitation: predecessor without conflict map')
    for pair in range(int(kw.get('attempts', '3'))):
        if Path(kw['stop']).exists(): break
        available = [a for a in ARMS if applicable(a, A, history)]
        arm, standards, generation = track.choose(context, available, rnd, kw.get('racetrack_arm'))
        if arm is None: break
        cfg = SS.choice(lines, SS.net_class(A), rnd, goal='v3', shape_arms=kw.get('shape_arms'))
        cfg.update(hist_context=task, learning_data='1', dataset='0', zone_plan=mode, r36=kw.get('r36', 'geometric'))
        cfg.update(overrides)
        for a in standards:
            if a != arm and applicable(a, A, history): cfg = configure(cfg, a, True, history)
        results = {}; measurements = {}

        for on in ([False, True] if (seed+pair)%2 == 0 else [True, False]):
            kid = 2*pair+int(on)
            trial_cfg = configure(cfg, arm, on, history)
            z = worker(str(ad), A, trial_cfg, kid, time_s, it)
            z.update(design_family=family(A), board=f'erf_{seed:06d}',
                     racetrack=dict(arm=arm, an=on, pair=pair, context=context, generation=generation))
            measurements[on] = measurement(ad, kid, z)
            z['racetrack']['measurement'] = measurements[on]
            results[on] = z
        record = dict(hist_sha256=hashlib.sha256(Path(history).read_bytes()).hexdigest() if history else None, id=digest(dict(task=task, cfg=cfg, arm=arm, generation=generation, pair=pair)),
                      arm=arm, generation=generation, standards=standards, family=family(A), task=task,
                      comparison=compare(measurements[False], measurements[True]),
                      baseline=measurements[False], new=measurements[True], hashes=hashes,
                      seed=seed, budget=dict(cpu_s=time_s, iter=it), time_s=time.strftime('%Y-%m-%d %H:%M:%S'), load=os.getloadavg(),
                      cpu_s=[results[on].get('cpu_total') for on in (False, True)])
        if not code_identical(hashes, results.values(), SS.code_hashes()):
            record['comparison'] = dict(valid_delta=0, gate_check=False, measured=False, delta=None, reason='Code change within pair')
        decision = track.report(context, record)


        for on in (False, True):
            results[on] = capture(str(ad), A, results[on], 2*pair+int(on), time_s, it)
        for on in (False, True):
            meas_cpu(results[on], measurements[on])
        record['cpu_balance'] = balance(*(results[on].get('cpu_total') for on in (False, True)))


        track.transact(lambda d: next(r for r in d['contexts'][context]['pairs'] if r['id']==record['id']).update(cpu_balance=record['cpu_balance']))
        for on in (False, True):
            z = results[on]; z['racetrack']['decision'] = decision
            z['racetrack']['cpu_balance'] = record['cpu_balance']
            with SS.LOCK:
                with open(kw['data'], 'a') as f: f.write(json.dumps(z)+'\n')
            rows.append(z)
        if trainer is not None: trainer.verify()


        load = 2*pair+int((seed+pair)%2 == 0)
        candidate = ad / f'k{load}' / 'labels.npz'
        if candidate.exists(): history = candidate
        print(f'Racetrack {seed} pair {pair} {arm}: delta={record["comparison"]["delta"]} Gate={record["comparison"]["gate_check"]} {decision["action"]}', flush=True)

    complete = len(rows) - int(kw.get('racetrack_arm') == 'hist_start') == 2*int(kw.get('attempts', '3'))
    completed_rows = [z for z in rows if z.get('racetrack', {}).get('phase') != 'predecessor']
    if complete and completed_rows and all(z['racetrack']['measurement'].get('completed') for z in completed_rows):
        SS.level_report(kw, any(z['racetrack']['measurement'].get('is_valid') is True for z in completed_rows), seed)
    if kw.get('keep') != '1' and completed_rows and all((z.get('dataset') or {}).get('cur_state') == 'ok' for z in rows):
        import shutil
        shutil.rmtree(ad)
    task_bill = balance(time.thread_time()-task_cpu_start, rg.get('cpu_s'),
                      *(z.get('cpu_total') for z in rows))
    if trainer is not None:


        task_bill = dict(task_bill, attempt_cpu_s=task_bill['cpu_s'], cpu_s=None,
                         complete=False, training_cpu_s=None, reason='asynchronous value training')
    (Path(out)/f'racetrack_cpu_{seed:06d}.json').write_text(json.dumps(dict(task_bill, seed=seed, load=os.getloadavg()), indent=1))
    return rows


def capture(ad, A, z, kid, time_s, it):
    c = Path(ad)/f'k{kid}'; c.mkdir(parents=True, exist_ok=True)
    (c/'intake_row.json').write_text(json.dumps(z))
    updated = worker(ad, A, z.get('cfg', {}), kid, time_s, it, phase='intake')
    if updated.get('faults') and 'racetrack' not in updated:
        updated = dict(z, cpu_total=updated.get('cpu_total'), dataset=dict(cur_state='faults', faults=updated['faults']))
    charge = updated.get('cpu_total')
    total = balance(z.get('cpu_total'), charge)
    updated['cpu_intake_s'] = charge
    updated['cpu_total'] = total['cpu_s']
    updated['cpu_measurement'] = 'whole_wait4' if total['complete'] else 'unknown'
    return updated

if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    if len(sys.argv) != 4 or sys.argv[1] not in ('measure', 'attempt', 'intake'): raise SystemExit('Internal worker call only')
    phase, ad, kid = sys.argv[1], sys.argv[2], int(sys.argv[3])
    c = Path(ad)/f'k{kid}'
    if phase == 'measure':
        z = json.loads((c/'racetrack_input.json').read_text())
        result = _measurement(ad, kid, z)
        output = c/'racetrack_measurement.json'
    else:
        from common import slots as PL
        from learning import selfplay as SS
        original = PL.job
        def nested(*args, **kw):
            kw['with_slot'] = False
            return original(*args, **kw)
        PL.job = nested
        inp = json.loads((c/(phase+'_input.json')).read_text())
        if phase == 'attempt':
            result = SS.attempt(ad, inp['A'], inp['cfg'], kid, inp['time_s'], inp['it'], candidates=1, candidates_auto=False)
        else:
            result = json.loads((c/'intake_row.json').read_text())
            SS.dataset_capture(ad, inp['A'], result, kid, candidates=1, candidates_auto=False)
        output = c/(phase+'_out_port.json')
    output.write_text(json.dumps(result, allow_nan=False))
