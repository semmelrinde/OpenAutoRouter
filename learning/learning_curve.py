import glob, hashlib, json, os, shutil, subprocess, sys, threading, time, uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
from common.process_guard import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
FOLDER = os.environ.get('LEARNING_CURVE_FOLDER') or os.path.expanduser('~/Library/Caches/Autorouter/learning_curve')
DATA = os.environ.get('LEARNING_CURVE_DATA') or os.path.join(R, 'learning', 'data', 'learning_curve.jsonl')
MAIN = os.environ.get('LEARNING_CURVE_MAIN') or R
LEVELS = (0, 2, 4, 6, 8); SEEDS = (900001, 900002, 900003)

SETS = dict(learning_curve=(SEEDS, 'tasks', DATA),
              acceptance=((950001, 950002, 950003, 950004), 'acceptance', os.path.join(MAIN, 'learning', 'data', 'acceptance.jsonl')))
ACCEPTANCE_DAYS = 7
REF = dict(cross=2, separate=2, via_cost=0.6, fine='1', pfmul=1.25, shaping='45r', seed=1)


def tasks(dataset='learning_curve'):
    from learning import tasks as AU
    out = []; seeds, under, _ = SETS[dataset]
    for st in LEVELS:
        for s in seeds:
            d = os.path.join(FOLDER, under, 'v' + AU.VERSION, 's%d_%d' % (st, s))
            if not os.path.exists(os.path.join(d, 'task.json')): AU.generate(d, s * 10 + st, level=st)
            out.append(d)
    return out


def best_cfg(lines, nc, goal=None):
    from learning import selfplay as SS
    from learning import tasks as AU
    lines = [z for z in lines if str(z.get('version', '')).split('+')[0] == AU.VERSION]
    lines = [x for x in lines if x.get('cur_state') != 'code_changed' and not x.get('not_routed')]
    from analysis.standard import goal as default_target
    goal = default_target(explicit=goal, context='learning_curve')
    lines = [z for z in lines if SS.evaluation_target(z) == goal]
    if goal == 'minimax': lines = [z for z in lines if SS.reward(z) is not None]
    rew = [SS.reward(z) for z in lines] if goal == 'minimax' else SS.rewards(lines)
    cfg = {}
    for k, value_list in sorted(SS.ARMS.items()):
        best, bm = value_list[0], 9e9
        for w in value_list:
            r = [b for x, b in zip(lines, rew) if SS.arm(x, k) == w and x.get('net_class') == nc] or \
                [b for x, b in zip(lines, rew) if SS.arm(x, k) == w]
            if len(r) >= 3 and sum(r) / len(r) < bm: best, bm = w, sum(r) / len(r)
        cfg[k] = best
    cfg['seed'] = 1
    return cfg


def meas_cfg(cfg, budget, work_max, emergency_stop, router):
    if budget not in ('work', 'cpu'): raise ValueError('budget=work|cpu')
    if work_max <= 0 or emergency_stop <= 0: raise ValueError('Work amount/emergency stop must be positive')
    result = dict(cfg, dataset='0', learning_data='0', budget=budget)
    if budget == 'work': result.update(work_max=work_max, work_cpu_emergency_stop=emergency_stop)
    if router:
        from learning.selfplay import ROUTER_VARIANTS
        old = result.get('router', 'prior')
        result['router'] = list(ROUTER_VARIANTS[old] if isinstance(old, str) else old) + list(router)
    return result


def learning_rows(path):
    with open(path, 'rb') as f:
        data = f.read(os.fstat(f.fileno()).st_size)
    data = data[:data.rfind(b'\n') + 1]
    return [json.loads(line) for line in data.splitlines() if line.strip()]


def point_incomplete(row):
    if row.get('unknown') or row.get('unknown_ref'): return True
    for result in row.get('results') or []:
        for part in (result, result.get('ref') or {}):
            if 'is_valid' in part and part['is_valid'] is None: return True
    return False


def measurement_contract(budget, time_s, it, work_max, emergency_stop, router):
    return dict(budget=budget, work_max=work_max if budget == 'work' else None,
                cpu_emergency_stop=emergency_stop if budget == 'work' else None,
                time_s=time_s if budget == 'cpu' else None,
                cpu_scaling='int(time_s * min(5.0, max(1.0, pads / 300.0)))' if budget == 'cpu' else None,
                legal_lp_budget='60 CPU-s' if budget == 'cpu' else 'work: fixed node count; CPU emergency stop',
                iter=it, router=list(router),validity='model_v3_with_cost_kicad_native-v2')


def measure(par=3, time_s=300, it=40, dataset='learning_curve', goal=None, budget='cpu',
           work_max=59147, emergency_stop=60000, router=(), keep=False, steering=None):
    from learning import selfplay as SS
    from analysis.standard import goal as default_target
    cli_goal = goal
    goal = default_target(explicit=cli_goal, context='learning_curve')
    p = os.path.join(MAIN, 'learning', 'data', 'selfplay.jsonl')
    frozen_choice = json.loads(Path(steering).read_text()) if steering else None
    if frozen_choice and frozen_choice['evaluation_target'] != goal: raise ValueError('Learning choice has a different evaluation target')
    lines = learning_rows(p) if not steering and os.path.exists(p) else []
    meas_cfg({}, budget, work_max, emergency_stop, router)
    head = os.environ.get('LEARNING_CURVE_HEAD') or subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=R, text=True).strip()
    job = time.strftime('%Y%m%d_%H%M%S') + '_' + dataset + '_' + uuid.uuid4().hex[:8]; results = []
    run_folder = os.path.join(FOLDER, 'runs', job)

    def one(d):
        A = json.load(open(os.path.join(d, 'task.json')))
        w = os.path.join(FOLDER, 'runs', job, os.path.basename(d)); shutil.copytree(d, w)
        vgp = os.path.join(w, 'specs.json'); cons = json.load(open(vgp))
        board_goal = default_target(cons, explicit=cli_goal, context='learning_curve')
        cons['evaluation'] = board_goal
        A['evaluation_target'] = board_goal
        if board_goal == 'minimax':
            cons.setdefault('profile_data', 'balanced'); A['profile_data'] = cons['profile_data']
        json.dump(cons, open(vgp, 'w'), indent=1)
        from analysis import corpus as KO
        KO.netclasses_run(w, os.path.join(w, 'board.json'), os.path.join(w, 'netclasses.json'), os.path.join(w, 'specs.json'))
        out = {}



        learned = frozen_choice['cfg'][SS.net_class(A, board_goal)] if frozen_choice else best_cfg(lines, SS.net_class(A, board_goal), board_goal)
        if not frozen_choice:
            from learning import champion as ME
            md = ME.read_in()
            _, me = ME.selection(md, ME.net_class(A), exact=True) if md['netclasses'] else (None, None)
            if me: learned = dict(REF)
        for category, cfg in (('ref', dict(REF, legal_lp=True)), ('learned', dict(learned, legal_lp=True))):
            cfg = meas_cfg(cfg, budget, work_max, emergency_stop, router)
            wa = w + '_' + category
            if not os.path.exists(wa): shutil.copytree(w, wa)
            z = SS.attempt(wa, A, cfg, 0, time_s, it)
            json.dump(z, open(os.path.join(wa, 'measurement.json'), 'w'), indent=1)
            cpu = z.get('cpu_total') or (((z.get('placer') or {}).get('cpu_s') or 0) + ((z.get('router') or {}).get('cpu_s') or 0))
            rp = Path(wa, 'k0', 'routes.json')
            route = json.loads(rp.read_text()) if rp.exists() else {}
            pp = Path(wa, 'k0', 'placement.json')
            placement_sha = hashlib.sha256(pp.read_bytes()).hexdigest() if pp.exists() else None
            copper_sha = hashlib.sha256(json.dumps({k: route.get(k) for k in ('tracks', 'vias', 'zones')}, sort_keys=True).encode()).hexdigest() if route else None
            out[category] = dict(rule=z.get('rule'), evaluation_target=board_goal, valid_routing=z.get('valid_routing', z['is_valid']), is_valid=z['is_valid'], score_v3=z.get('score_v3'), cpu_s=round(cpu, 1), vias=(z.get('metrics') or {}).get('vias'),
                            remaining_conflicts=(z.get('metrics') or {}).get('conflict_list'), open_items=(z.get('metrics') or {}).get('open_items'),
                            reason=z.get('invalid_reason'), cur_state=z.get('cur_state'), faults=z.get('faults'), cfg=cfg,
                            copper_sha=copper_sha, work=route.get('work_budget'), placement_sha=placement_sha,
                            placer=z.get('placer'), placer_metrics=z.get('placer_metrics'),
                            champion=route.get('champion'),cost=(z.get('cfg') or {}).get('cost'),
                            native_champion=z.get('native_champion'))
        g = out['learned']
        return dict(generator_version=A['version'], rule=g['rule'], evaluation_target=board_goal, valid_routing=g['valid_routing'], task=os.path.basename(d), level=A.get('level'), is_valid=g['is_valid'], score_v3=g['score_v3'], cpu_s=g['cpu_s'],
                    vias=g['vias'], remaining_conflicts=g.get('remaining_conflicts'), open_items=g.get('open_items'), reason=g['reason'], cur_state=g['cur_state'], faults=g['faults'], cfg=g['cfg'], copper_sha=g['copper_sha'], work=g['work'],
                    placement_sha=g['placement_sha'], placer=g['placer'], placer_metrics=g['placer_metrics'],
                    champion=g.get('champion'),cost=g.get('cost'),native_champion=g.get('native_champion'),ref=out['ref'])
    with ThreadPoolExecutor(par) as ex: results = list(ex.map(one, tasks(dataset)))
    n_g = sum(e['is_valid'] is True for e in results); cpu = sum(e['cpu_s'] for e in results)
    h = hashlib.sha256()
    for d in tasks(dataset):
        for f in ('board.json', 'design_rules.json', 'specs.json', 'task.json'):
            if os.path.exists(os.path.join(d, f)): h.update(open(os.path.join(d, f), 'rb').read())
    from learning import tasks as AU
    row = dict(generator_version=AU.VERSION, evaluation_target=goal, unknown=sum(e['is_valid'] is None for e in results),
                 unknown_ref=sum(e['ref']['is_valid'] is None for e in results), time_s=time.strftime('%Y-%m-%d %H:%M'), dataset=dataset, code=head, tasks=len(results), is_valid=n_g, tasks_sha=h.hexdigest()[:16],
                 measurement_contract=measurement_contract(budget, time_s, it, work_max, emergency_stop, router),
                 source_sha={p: hashlib.sha256(Path(R, p).read_bytes()).hexdigest() for p in ('router/route_pf.py', 'analysis/netclasses.py')},
                 steering_sha=hashlib.sha256(Path(steering).read_bytes()).hexdigest() if steering else hashlib.sha256(json.dumps(lines, sort_keys=True).encode()).hexdigest(),
                 steering_source=frozen_choice.get('source') if frozen_choice else p,
                 cpu_balance='Selfplay-PartialBalance', run_folder=run_folder if keep else None,
                 valid_ref=sum(1 for e in results if e['ref']['is_valid']), load=round(os.getloadavg()[0], 1),
                 cpu_per_valid=round(cpu / n_g, 1) if n_g else None,
                 valid_per_level={str(st): sum(e['is_valid'] is True for e in results if e['level'] == st) for st in LEVELS},
                 results=results)

    _sum = lambda k: sum(int(e.get(k) or 0) for e in results) if all(e.get(k) is not None for e in results if e['is_valid'] is not None) else None
    row['remaining_conflicts_total_sum'] = _sum('remaining_conflicts'); row['open_total_sum'] = _sum('open_items')
    Path(SETS[dataset][2]).parent.mkdir(parents=True, exist_ok=True)
    with open(SETS[dataset][2], 'a') as f: f.write(json.dumps(row) + '\n')
    if not keep: shutil.rmtree(run_folder, ignore_errors=True)
    print('%s %s (%s): %d/%d valid, %d unknown, residual conflicts %s, open %s, CPU per valid task %s s, per level %s' % (
        dataset.capitalize(), row['time_s'], head, n_g, len(results), row['unknown'], row['remaining_conflicts_total_sum'], row['open_total_sum'], row['cpu_per_valid'], row['valid_per_level']), flush=True)
    stagnation_verify()
    return row


def stagnation_verify():
    if Path(DATA).resolve() != Path(MAIN, 'learning/data/learning_curve.jsonl').resolve():
        print('Diagnostic curve: no default stagnation alarm', flush=True)
        return
    try:
        from learning.stagnation import verify
        result = verify(Path(MAIN))
        print('Stagnation watchdog: ' + result['status'], flush=True)
    except Exception as exc:
        print('Stagnation watchdog unknown: ' + repr(exc), flush=True)


def read_in(dataset='learning_curve'):
    from learning import tasks as AU
    p = SETS[dataset][2]
    rows = [json.loads(l) for l in open(p)] if os.path.exists(p) else []
    return [z for z in rows if z.get('generator_version') == AU.VERSION]


def acceptance_due():
    z = read_in('acceptance')
    if not z: return True
    return time.time() - time.mktime(time.strptime(z[-1]['time_s'], '%Y-%m-%d %H:%M')) > ACCEPTANCE_DAYS * 86400


def frozen(par, dataset='learning_curve'):
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True).stdout.strip()
    code = os.path.join(FOLDER, 'code', head[:12])
    if not os.path.exists(os.path.join(code, 'learning', 'learning_curve.py')):
        os.makedirs(code, exist_ok=True)
        archive = subprocess.check_output(['git', '-C', R, 'archive', head])
        subprocess.run(['tar', '-x', '-C', code], input=archive, check=True)
        os.symlink(os.path.join(R, '.venv'), os.path.join(code, '.venv'))
    prior = sorted(glob.glob(os.path.join(FOLDER, 'code', '*')), key=os.path.getmtime)[:-3]
    for d in prior: shutil.rmtree(d, ignore_errors=True)
    from learning import champion as ME
    snapshot=ME.freeze(Path(FOLDER)/'champion_snapshots'/(uuid.uuid4().hex+'.json'))
    env = dict(os.environ, LEARNING_CURVE_DATA=DATA, LEARNING_CURVE_MAIN=R, LEARNING_CURVE_HEAD=head[:7],
               AUTOROUTER_CHAMPION=snapshot['fpath'],PYTHONHASHSEED='0')
    subprocess.run([os.path.join(R, '.venv', 'bin', 'python'), os.path.join(code, 'learning', 'learning_curve.py'), 'measure',
                    'parallel=%s' % par, 'dataset=' + dataset], cwd=code, env=env)


RELEVANT = ['router/', 'placement/', 'analysis/netclasses.py', 'learning/current_zones.py', 'learning/current_zones_topo.py',
            'learning/shaping.py', 'learning/tasks.py', 'learning/bga_occupancy.py', 'learning/zone_planner.py', 'common/']


def old_h(z):
    try: return (time.time() - time.mktime(time.strptime(z.get('time_s', ''), '%Y-%m-%d %H:%M'))) / 3600
    except ValueError: return 1e9


def relevant_changed(prior, new):
    r = subprocess.run(['git', 'diff', '--name-only', prior, new, '--'] + RELEVANT, cwd=R, capture_output=True, text=True)
    return r.returncode != 0 or bool(r.stdout.strip())


def measurement_due(rows):
    return not rows or old_h(rows[-1]) >= 24


def main():
    a = sys.argv[1:] or ['show']; kw = dict(x.split('=', 1) for x in a if '=' in x)
    if a[0] == 'balance':
        return balance(kw)
    if a[0] == 'measure':
        measure(int(kw.get('parallel', '3')), time_s=int(kw.get('time_s', '300')), it=int(kw.get('iter', '40')),
               dataset=kw.get('dataset', 'learning_curve'), goal=kw.get('goal'), budget=kw.get('budget', 'cpu'),
               work_max=int(kw.get('work_max', '59147')), emergency_stop=int(kw.get('emergency_stop', '60000')),
               router=kw.get('router', '').split(',') if kw.get('router') else (), keep=kw.get('keep') == '1', steering=kw.get('steering'))
        return
    if a[0] == 'duration':
        stop = os.path.join(R, 'learning', 'STOP'); iv = int(kw.get('interval', '10800'))
        if iv < 1 or int(kw.get('loops','0')) < 0: raise ValueError('interval>0, loops>=0')


        cycles = 0

        threading.Thread(target=hourly_report, args=(kw, stop), daemon=True).start()
        while not os.path.exists(stop):


            try:
                from learning.champion_gate import verify as champion_verify
                decision = champion_verify({k[8:]: v for k, v in kw.items() if k.startswith('champion_')})
                print('Champion service: ' + json.dumps(decision), flush=True)
                if isinstance(decision,dict) and 'gate_check' in decision: cycles += 1
            except Exception as exc:
                print('Champion service unknown: ' + repr(exc), flush=True)

            prior = read_in()
            if kw.get('daily_measurement','1') != '0' and measurement_due(prior):
                frozen(kw.get('parallel', '3'))
            if kw.get('daily_measurement','1') != '0' and acceptance_due() and not os.path.exists(stop): frozen(kw.get('parallel', '3'), dataset='acceptance')
            if int(kw.get('loops','0')) and cycles >= int(kw['loops']): return

            current = read_in()
            rest = max(60, (24 - old_h(current[-1])) * 3600) if current else 60
            t0 = time.time()
            while time.time() - t0 < min(iv, rest, 60) and not os.path.exists(stop):
                time.sleep(min(60,iv))
        return
    for z in read_in(): print(z['time_s'], z['code'], '%d/%d valid' % (z['is_valid'], z['tasks']), 'CPU/is_valid', z['cpu_per_valid'], z['valid_per_level'])

def hourly_report(kw, stop):
    while not os.path.exists(stop):
        try:
            balance(kw)
        except Exception as e:
            print('Learning balance unknown: ' + type(e).__name__ + ': ' + str(e), flush=True)
        deadline = time.monotonic() + 3600
        while time.monotonic() < deadline and not os.path.exists(stop): time.sleep(60)


def balance(kw):
    from learning.learning_balance import balance, operating_balance, observe
    keys = ('balance_set', 'balance_basis', 'balance_effective')
    if not all(kw.get(k) for k in keys):
        if any(kw.get(k) for k in keys):
            print('Learning balance unknown: incomplete quality balance arguments', flush=True)
            return 1
        now = time.time()
        try:
            data = kw.get('balance_data', os.path.join(MAIN, 'learning/data'))
            state = kw.get('balance_state', os.path.expanduser('~/Library/Caches/Autorouter/learning_balance/input_.json'))
            report = operating_balance(data, now - 86400, now)
            report['input_'] = observe(data, state)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
            report = dict(status='unknown', reasons=[str(e)])
        print('Learning operation ' + json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
        return 1 if report['status'] == 'unknown' else 0
    try:
        if any(os.path.getsize(kw[k]) > 2_000_000 for k in keys):
            raise ValueError('Balance needs compact result objects, no raw corpora')
        report = balance(*(json.load(open(kw[k])) for k in keys))
    except (OSError, ValueError) as e:
        report = dict(status='unknown', reasons=[str(e)])
    print('Learning balance ' + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'measured' else 1


if __name__ == '__main__':
    sys.exit(main())
