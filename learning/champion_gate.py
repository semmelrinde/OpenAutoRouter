import contextlib
import copy
import fcntl
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
from learning import champion as M
from common import slots as PL


def atomic(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.new')
    with tmp.open('w') as f:
        json.dump(value, f, indent=1, sort_keys=True, allow_nan=False); f.write('\n')
        f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def jsonl(path):
    if not Path(path).exists(): return []
    from learning.learning_curve import learning_rows
    return learning_rows(path)


def tasks(root):
    from learning import tasks as A
    out = []
    for v in ('10', '11'):
        for st in (0, 2, 4, 6, 8):
            for seed in (900001, 900002, 900003):
                folder = Path(root) / ('v'+v) / ('s%d_%d' % (st, seed))
                if not (folder/'task.json').exists():
                    A.generate(str(folder), seed*10+st, level=st, generator_version=v)
                a = json.loads((folder/'task.json').read_text())
                if a['version'] != v or a['seed'] != seed*10+st or a['level'] != st:
                    raise ValueError('Gate set changed: '+str(folder))
                hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(folder.iterdir()) if p.is_file()}
                if v=='11':
                    reference=json.loads((R/'codex/generator_real/learning_curve_v11.json').read_text())
                    expected=next(x['sha256'] for x in reference['tasks'] if x['id']==folder.name)
                    if any(hashes.get(f)!=digest for f,digest in expected.items()):
                        raise ValueError('V11 deviates from the versioned gate set: '+folder.name)
                else:
                    from learning.learning_curve import FOLDER
                    canonical=Path(FOLDER)/'tasks/v10'/folder.name
                    for f,digest in hashes.items():
                        if (canonical/f).exists() and hashlib.sha256((canonical/f).read_bytes()).hexdigest()!=digest:
                            raise ValueError('V10 deviates from the frozen learning curve set: '+folder.name+'/'+f)
                out.append(dict(id='v'+v+'/'+folder.name, fpath=str(folder), net_class=M.net_class(a), sha=hashes))
    manifest = Path(root)/'dataset.json'
    if manifest.exists() and json.loads(manifest.read_text()) != out:
        raise ValueError('Gate set content changed against frozen manifest')
    atomic(manifest, out)
    return out


def head():
    r = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True)
    if r.returncode == 0 and r.stdout.strip(): return r.stdout.strip()
    from learning.collect import code_state
    return 'without_git:' + code_state()


def run_duration():
    cli=Path('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli')
    packages={}
    for name in ('numpy','scipy','shapely','numba'):
        try:packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:packages[name]=None
    return dict(python=sys.version,platform=platform.platform(),packages=packages,
        kicad=subprocess.check_output([str(cli),'version'],text=True).strip(),
        kicad_cli_sha=hashlib.sha256(cli.read_bytes()).hexdigest())


def effective_cfg(cfg):
    cfg=copy.deepcopy(cfg);cost=M.parameter(cfg.get('cost') or {})
    if 'shaping' in cost:cfg['shaping']=cost['shaping']
    if 'placer_style' in cost and isinstance(cfg.get('placer','prior'),str):cfg['placer']=cost['placer_style']
    if 'router_style' in cost and isinstance(cfg.get('router','prior'),str):cfg['router']=cost['router_style']
    return cfg


def placement_cfg(cfg):
    cfg = effective_cfg(cfg)
    cfg['cost'] = {k:v for k,v in (cfg.get('cost') or {}).items() if k in M.PLACER}
    for key in M.ROUTER | {'shaping','router'}:
        cfg.pop(key, None)
    return cfg


def reuse(task, cfg, source_sha, reference, run_duration_sha=None):
    if not reference:return None
    z, directory=reference
    directory=Path(directory);p=directory/'measurement.json'
    if not p.exists():return None
    raw=json.loads(p.read_text())
    measured={k:v for k,v in z.items() if k!='reuse'}
    if M.sha(raw)!=M.sha(measured):return None
    if z.get('reuse') and hashlib.sha256(p.read_bytes()).hexdigest()!=z['reuse']['measurement_sha']:
        raise ValueError('Reused original evidence changed')
    meta=raw.get('champion_measurement') or {}
    if run_duration_sha is not None and meta.get('run_duration_sha')!=run_duration_sha:return None
    cfg=effective_cfg(cfg)
    placement_only = (raw.get('cur_state') == 'placer_collision' and
                      placement_cfg(raw.get('cfg') or {}) == placement_cfg(cfg))
    if (raw.get('code_sha256')!=source_sha or (raw.get('cfg')!=cfg and not placement_only) or
        meta.get('input_')!=task['sha'] or meta.get('net_class')!=task['net_class'] or
        meta.get('budget')!='work' or meta.get('work')!=cfg.get('work_max') or
        meta.get('seed')!=cfg.get('seed') or type(raw.get('is_valid')) is not bool):
        return None
    if raw.get('cur_state') not in ('ok','placer_collision'):return None
    if raw['cur_state']=='ok' and not raw.get('native_champion'):return None
    artifacts=meta.get('artifacts')
    if not isinstance(artifacts,dict) or not artifacts:return None
    for name,digest in artifacts.items():
        artifact=(directory/name).resolve()
        if not artifact.is_relative_to(directory.resolve()):raise ValueError('Artifact outside the gate case')
        if not artifact.exists() or hashlib.sha256(artifact.read_bytes()).hexdigest()!=digest:
            raise ValueError('Reuse evidence changed: '+str(artifact))
    raw['reuse']=dict(folder=str(directory),measurement_sha=hashlib.sha256(p.read_bytes()).hexdigest(),
        reason='Identical source/input/full configuration/seed/work budget/native artifacts',
        again_routed=False)
    if placement_only:
        raw['reuse'].update(reason='Identical placement before routing; router costs were not executed',
                                       requested_cfg=cfg, only_placement=True)
    return raw


def last_refs(root):
    root=Path(root).resolve();pointer=root/'last_measurement.json'
    if not pointer.exists():return {}
    item=json.loads(pointer.read_text());p=Path(item['report']).resolve()
    if not p.is_relative_to(root):raise ValueError('Gate cache outside its measurement root')
    if not p.exists():return {}
    if hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Gate cache report changed')
    report=json.loads(p.read_text());arm=item['arm'];data=p.parent/arm/'results.json'
    if not data.exists():return {}
    rows=json.loads(data.read_text())
    if M.sha(rows)!=report['results_sha'][arm]:raise ValueError('Gate cache individual evidence changed')
    return {k:(z,report['evidence_folder'][arm][k]) for k,z in rows.items()}


def configuration(task, doc, work):
    from learning.learning_curve import REF, meas_cfg
    _, entry = M.selection(doc, task['net_class'], exact=True)
    cfg = dict(REF, legal_lp=True, champion='0', cost=entry['parameter'] if entry else {}, router=['attempts=0'])
    return effective_cfg(meas_cfg(cfg, 'work', work, 60000, ()))


def measure(dataset, doc, folder, parallel=3, work=59147, it=40, refs=None):
    from learning import selfplay as SS
    from learning.learning_curve import REF, meas_cfg
    from analysis import corpus as KO
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    runtime=run_duration();runtime_sha=M.sha(runtime)
    def one(task):
        cfg = configuration(task, doc, work)
        reused=reuse(task,cfg,SS.code_hashes()['code_sha256'],(refs or {}).get(task['id']),run_duration_sha=runtime_sha)
        if reused is not None:
            reused['reuse']['requested_model_sha']=M.sha(doc)
            print('Champion gate reused '+task['id']+' '+str(reused['is_valid']),flush=True)
            return task['id'],reused
        w = folder/task['id']; shutil.copytree(task['fpath'], w)
        A = json.loads((w/'task.json').read_text())
        KO.netclasses_run(str(w), str(w/'board.json'), str(w/'netclasses.json'), str(w/'specs.json'))
        z = SS.attempt(str(w), A, cfg, 0, 300, it)
        if z.get('cur_state') == 'ok' and not z.get('native_champion'):
            try:
                kicad_native(w, A, z)
            except Exception as exc:
                z.update(is_valid=None, cur_state='native_gate_measurement_incomplete', faults=repr(exc))

        artifacts = {}
        for p in sorted((w/'k0').rglob('*')):
            if p.is_file() and p.suffix in ('.json', '.kicad_pcb', '.kicad_dru', '.kicad_pro'):
                artifacts[str(p.relative_to(w))] = hashlib.sha256(p.read_bytes()).hexdigest()
        z['champion_measurement'] = dict(net_class=task['net_class'], model_sha=M.sha(doc), input_=task['sha'],
                                    artifacts=artifacts, seed=1, budget='work', work=work,
                                    run_duration=runtime,run_duration_sha=runtime_sha,
                                    load=os.getloadavg(), source_sha={p:hashlib.sha256((R/p).read_bytes()).hexdigest()
                                        for p in ('router/route_pf.py', 'analysis/netclasses.py')})
        placement = w/'k0/placement.json'
        if placement.exists():

            z['champion_measurement']['placement_sha'] = M.sha(json.loads(placement.read_text())['parts'])
        atomic(w/'measurement.json', z)
        print('Champion gate '+task['id']+' '+str(z.get('is_valid'))+' '+str(M.numbers(z)), flush=True)
        return task['id'], z
    with ThreadPoolExecutor(parallel) as pool:
        result = dict(pool.map(one, dataset))
    atomic(folder/'results.json', result)
    return result


def order(dataset, refs, old=None, new=None, work=59147):
    def key(task):
        row = refs.get(task['id'], ({}, None))[0]
        cpu = row.get('cpu_total')
        if type(cpu) not in (int, float): cpu = float('inf')
        unchanged = (old is not None and new is not None and
                     configuration(task,old,work) == configuration(task,new,work))
        return (row.get('is_valid') is not True, unchanged, cpu, int(task['id'].split('/s')[1].split('_')[0])
                if '/s' in task['id'] else 0, task['id'])
    return sorted(dataset, key=key)


def sequential(dataset, old, new, out, parallel, work, it, refs=None):
    before, after = {}, {}
    from common.cpu_tree import CpuTree
    monitor = CpuTree(subprocess.Popen)
    cpu_start = monitor.sample(os.getpid())
    load_start = os.getloadavg()
    references = refs or {}
    planned = order(dataset, references, old, new, work)
    stop = None
    for index, task in enumerate(planned):
        name = task['id']
        a = measure([task], old, out/'champion', 1, work, it, refs=references)[name]
        origin = (a.get('reuse') or {}).get('folder', str(out/'champion'/name))
        b = measure([task], new, out/'challenger', 1, work, it, refs={name:(a,origin)})[name]
        before[name], after[name] = a,b
        if a.get('is_valid') is True and b.get('is_valid') is not True:
            stop = 'Valid board lost: '+name
        elif a.get('is_valid') is True and b.get('is_valid') is True and M.quality(a,b):
            stop = 'Rule regression: '+name
        elif any(z.get('is_valid') is None or z.get('cur_state') not in ('ok','placer_collision') for z in (a,b)):
            stop = 'Unknown completion: '+name
        else:
            remaining = planned[index+1:]




            unchanged = all(configuration(t,old,work) == configuration(t,new,work) for t in remaining)
            gained = any(after[k].get('is_valid') is True and before[k].get('is_valid') is not True for k in before)
            ds = [M.total_remainder(before[k])-M.total_remainder(after[k]) for k in before
                  if M.total_remainder(before[k]) is not None and M.total_remainder(after[k]) is not None]
            optimistic = M.significance(ds+[0]*len(remaining))
            if remaining and unchanged and not gained and (
                    optimistic['p'] > optimistic['alpha'] or sum(ds) <= 0):
                stop = 'Sign test/total residual not reachable; remaining configurations identical'
        if stop: break
    for arm, rows in (('champion',before),('challenger',after)):
        atomic(out/arm/'results.json', rows)
    return before, after, dict(complete=len(before)==len(dataset), measured=len(before), planned=len(dataset),
                              order=[t['id'] for t in planned], early_declined=stop,
                              cpu_s=monitor.sample(os.getpid())-cpu_start,
                              cpu_measurement='libproc process tree, sample before/after sequence',
                              load_before=load_start,load_after=os.getloadavg())

def native_zones(routes, basis):
    zones = list(routes.get('zones') or basis())
    if routes.get('zone_list'):
        from common.zones import unify
        zones += unify(dict(routes, zones=[]))['zones']
    return zones


def kicad_native(w, A, z, cpu_budget=None):
    from kicad.python_call import python_command
    from codex.bga_fanout.polygon_rings import copper_map
    w=Path(w);c = w/('k%d' % z.get('candidate',0)); source=c/'native_basis/base.kicad_pcb'; out=c/'champion_kicad_native';out.mkdir()
    runs=[]
    def call(cmd,log):
        limit=min(900,cpu_budget.rest) if cpu_budget is not None else 900
        if limit<1:raise RuntimeError('No CPU remainder for native evaluation')
        run=PL.job(list(map(str,cmd)),cpu_s=limit,log=str(log),cwd=R,
                    env=dict(os.environ,PYTHONHASHSEED='0'))
        runs.append(run)
        if '_cpu_kind_s' in z:z['_cpu_kind_s']+=run['cpu_s']
        else:z['cpu_total']=(z.get('cpu_total') or 0)+run['cpu_s']
        if cpu_budget is not None:cpu_budget.add(run['cpu_s'])
        if run['cur_state']!='ok':raise RuntimeError('Native gate measurement: '+str(run))
    if not source.exists():
        board=json.loads((c/'board.json').read_text())
        if not str(board.get('src','')).startswith('task seed='):
            raise ValueError('Native selfplay evaluation needs a generated task')
        source.parent.mkdir(exist_ok=True)
        v11=str(A.get('version','')).split('+')[0]=='11'
        command=(python_command(R/'codex/generator_real/kicad_native.py',c/'board.json',source,w) if v11 else
                 python_command(R/'kicad/current_board.py',c/'board.json',source))
        call(command,out/'structure.log')
        if not v11:
            from learning.copper_kicad_native import rule_contract
            project,dru=rule_contract(board,dict(options={}),json.loads((w/'design_rules.json').read_text()))
            atomic(source.with_suffix('.kicad_pro'),project);source.with_suffix('.kicad_dru').write_text(dru)
        zones=(json.loads((w/'zones.json').read_text()) if v11 else
               [dict(net='GND',layer=layer,pts=poly,full=True,prio=0) for layer in A['gnd_layers'] for poly in board['outline']])
        atomic(source.parent/'zones.json',zones)
    routes=json.loads((c/'routes.json').read_text())
    zones=native_zones(routes, lambda: json.loads((c/'native_basis/zones.json').read_text()))
    atomic(out/'zones.json', zones)
    rules=json.loads((w/'design_rules.json').read_text())
    commands=[python_command(R/'kicad/import_routes.py', source, c/'routes.json', out/'zones.json',
                          out/'filled.kicad_pcb', 'project=1', 'cu_layers='+str(A['cu_layers']), 'via='+rules['router']['via']),
              ['/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli', 'pcb','drc','--format','json',
               '--severity-all','-o',str(out/'drc.json'),str(out/'filled.kicad_pcb')],
              python_command(R/'kicad/copper_export.py',out/'filled.kicad_pcb',out/'cu_raw.json','polygons_exact=1')]
    for i, cmd in enumerate(commands):
        call(cmd,out/(str(i)+'.log'))
        if i == 0:
            from postprocessing.thermal_island import finish
            repair = finish(out/'filled.kicad_pcb', routes, call,
                dict(board=c/'board.json',netclasses=c/'netclasses.json',design_rules=w/'design_rules.json',routes=c/'routes.json'))
            if repair is not None:
                z['thermal_island'] = repair
                if repair.get('taken'):
                    from learning import collect as S
                    B=json.loads((c/'board.json').read_text());K=json.loads((c/'netclasses.json').read_text())
                    routes=json.loads((c/'routes.json').read_text())
                    from learning.shaping import is_valid as model_valid
                    model_ok,z['outcome']=model_valid(B,K,c/'routes.json',A['gnd_layers'],routes=routes)
                    z['rule']=S.rule_measurement(B,routes,K,A['gnd_layers'],dense=True)
                    z['score_v3']=z['rule']['score_v3']
                    z['is_valid']=z['is_valid'] and model_ok
                    z['metrics']=dict(vias=z['outcome'].get('vias'),conflict_list=z['outcome'].get('conflict_list'),
                        open_items=z['outcome'].get('open_items'),total_length=z['outcome'].get('length_mm'),
                        bends=z['outcome'].get('bends'),share_45=z['outcome'].get('share_45'))
    atomic(out/'cu.json', copper_map(json.loads((out/'cu_raw.json').read_text())))
    d=json.loads((out/'drc.json').read_text())
    if not all(k in d for k in ('violations','unconnected_items','kicad_version')):raise ValueError('DRC incomplete')
    errors=sum(v['severity']=='error' for v in d['violations']);opened=len(d['unconnected_items'])

    kinds={};[kinds.__setitem__(v['type'],kinds.get(v['type'],0)+1) for v in d['violations'] if v['severity']=='error']
    z['native_champion']=dict(faults=errors,open_items=opened,kinds=dict(sorted(kinds.items())),version=d['kicad_version'],runs=runs)
    z['valid_model']=z['is_valid'];z['is_valid']=z['is_valid'] and not errors and not opened
    z.setdefault('outcome',{})['drc_error']=errors


def pre_test(snapshot, dataset, out, parallel):

    v10 = next(Path(t['fpath']).parent for t in dataset if t['id'].startswith('v10/'))
    env = dict(os.environ, AUTOROUTER_CHAMPION=str(snapshot), PRE_TEST_TASKS=str(v10),
               PYTHONHASHSEED='0')
    cmd = [sys.executable, str(R/'tests/pre_test.py'), 'parallel='+str(parallel)]

    cmd = [sys.executable, '-S', str(R/'common/process_guard.py'), 'coordinate=1', '--'] + cmd
    p = subprocess.run(cmd, cwd=R, env=env, stdout=(Path(out)/'pre_test.log').open('w'), stderr=subprocess.STDOUT)
    return dict(argv=cmd, exit=p.returncode, log_sha=hashlib.sha256((Path(out)/'pre_test.log').read_bytes()).hexdigest())

def accept(path, old, new, report, history, curve=None):
    if not report['gate_check']['taken'] or report['pre_test']['exit'] != 0:
        raise ValueError('Champion acceptance without passed gate')
    if report.get('before_sha') != M.sha(old) or report.get('after_sha') != M.sha(new):
        raise ValueError('Gate measurement does not match champion/challenger')
    bindings=report.get('results_sha') or {}
    if set(bindings)!={'champion','challenger'} or any(len(v)!=64 for v in bindings.values()):
        raise ValueError('Gate measurement without bound individual evidence')
    if M.sha(M.read_in(path)) != M.sha(old): raise ValueError('Champion changed in the meantime')
    report_sha = M.sha(report)
    generation = old['generation'] + 1
    changed = []
    for nc, entry in new['netclasses'].items():
        if entry['parameter'] != old['netclasses'].get(nc, {}).get('parameter', {}):
            entry.update(generation=generation, evidence_sha=report_sha); changed.append(nc)
    event = dict(version=M.VERSION, generation=generation, time_s=time.strftime('%Y-%m-%d %H:%M:%S'),
                 netclasses=changed, gate_check=report['gate_check'], evidence_sha=report_sha, evidence=report['fpath'],
                 before_sha=M.sha(old), code=report['code'], measurement_contract=report['measurement_contract'],
                 curve_points=report.get('curve_points', []))
    new['generation'] = generation
    new['history'] = old.get('history', []) + [event]


    atomic(path, new)
    history_sync(new, history)
    if curve: curve_sync(new, curve)
    return event


def history_sync(doc, history):
    rows = jsonl(history)
    known = {x.get('evidence_sha') for x in rows}
    Path(history).parent.mkdir(parents=True, exist_ok=True)
    with Path(history).open('a') as f:
        for event in doc.get('history', []):
            if event['evidence_sha'] not in known: f.write(json.dumps(event, sort_keys=True)+'\n')
        f.flush(); os.fsync(f.fileno())


def curve_sync(doc, path):
    known={r.get('champion_point_id') for r in jsonl(path)}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('a') as f:
        for event in doc.get('history', []):
            for point in event.get('curve_points', []):
                ident=event['evidence_sha']+':'+point['generator_version']
                if ident in known: continue
                f.write(json.dumps(dict(point,champion_point_id=ident,champion_generation=event['generation'],
                    time_s=event['time_s'][:16],code=event['code'],champion_evidence=event['evidence_sha']))+'\n')
        f.flush();os.fsync(f.fileno())


def curve_points(before, after, report):
    points=[]
    for v in ('10','11'):
        keys=[k for k in sorted(after) if k.startswith('v'+v+'/')]
        def part(z):
            r=M.numbers(z)
            return dict(is_valid=z.get('is_valid'),score_v3=z.get('score_v3'),remaining_conflicts=r[0] if r else None,
                        native_drc_error=M.native_error(z),total_remainder=M.total_remainder(z),
                        open_items=r[1] if r else None,cur_state=z.get('cur_state'))
        results=[dict(part(after[k]),task=k,level=after[k].get('level_task'),ref=part(before[k])) for k in keys]
        points.append(dict(generator_version=v,dataset='champion_gate_v'+v,tasks=len(keys),
            is_valid=sum(after[k].get('is_valid') is True for k in keys),valid_ref=sum(before[k].get('is_valid') is True for k in keys),
            unknown=sum(after[k].get('is_valid') is None for k in keys),evaluation_target='v3',
            tasks_sha=report['set_sha'],measurement_contract=report['measurement_contract'],results=results,
            cpu_per_valid=None,valid_per_level={str(s):sum(r['is_valid'] is True for r in results if r['level']==s) for s in (0,2,4,6,8)}))
    return points


def target_classes(dataset, nc, pool='auto'):
    if pool not in ('auto','family'): raise ValueError('pool=auto|family')
    local = [t for t in dataset if M.family(t['net_class']) == M.family(nc)]
    selected = local
    level = 'layers_package'
    if pool == 'auto' and len(local) < 5:
        package = M.family(nc).split('_',1)[1]
        selected = [t for t in dataset if M.family(t['net_class']).split('_',1)[1] == package]
        level = 'shared_package'
    return sorted({t['net_class'] for t in selected}), dict(plane=level,
        families_pairs=len(local),planned_pairs=len(selected),
        smallest_p_without_bindings=2.**(-len(selected)),alpha=.05)


def verify(kw):
    from learning.learning_curve import MAIN
    root = Path(kw.get('root') or Path(MAIN)/'codex/self_learning/results/champion')
    root.mkdir(parents=True, exist_ok=True)
    model = Path(kw.get('model') or M.file())
    data = Path(kw.get('data') or Path(MAIN)/'learning/data/selfplay.jsonl')
    history = Path(kw.get('history') or model.parent.parent/'data/champion_history.jsonl')
    curve = Path(kw.get('curve') or model.parent.parent/'data/learning_curve.jsonl')
    with (root/'service.lock').open('a') as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return dict(status='another gate is running')
        old = M.read_in(model); history_sync(old, history); curve_sync(old, curve)
        rows = jsonl(data)
        statepath = root/'state.json'; state = json.loads(statepath.read_text()) if statepath.exists() else {}
        code = head()
        from learning.collect import code_state
        source_sha=code_state()
        dataset=tasks(root/'gate_set')
        covered={t['net_class'] for t in dataset};uncovered=[]
        choices = []
        for nc in sorted({M.row_class(r) for r in rows} - {None}):
            if not nc.startswith(('v10:','v11:')):continue
            if nc not in covered:
                uncovered.append(nc);continue
            if kw.get('net_class') and nc != kw['net_class']: continue
            rs = [r for r in rows if M.row_class(r) == nc and M.reward(r) is not None and r.get('cfg', {}).get('cost')]
            context=M.sha(dict(code=source_sha,champion=M.sha(old)))
            previous=state.get(nc,{})
            blocked=list(previous.get('checks',{}).get(context,[]))
            if nc in old['netclasses']:blocked.append(M.sha(old['netclasses'][nc]['parameter']))
            candidate = M.challenger(rows, nc, excluded=blocked)
            if len(rs) - state.get(nc, {}).get('n', 0) < int(kw.get('new', '30')) or not candidate: continue
            if candidate == old['netclasses'].get(nc, {}).get('parameter'): continue
            rewards=[M.reward(r) for r in rs if r['cfg']['cost']==candidate]
            score=tuple(sum(b[i] for b in rewards)/len(rewards) for i in range(5))
            choices.append((score, nc, candidate, len(rs)))
        if not choices: return dict(status='not due',without_gate_set=uncovered)
        _, nc, candidate, n = min(choices,key=lambda x:(x[0],x[1]))
        out = root/('gate_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]); out.mkdir()
        from common.cpu_tree import CpuTree
        gate_monitor = CpuTree(subprocess.Popen)
        gate_cpu_start = gate_monitor.sample(os.getpid())
        gate_load_start = os.getloadavg()
        new = json.loads(json.dumps(old))


        targets, pooling = target_classes(dataset, nc, kw.get('pool','auto'))
        for target in targets:
            new['netclasses'][target] = dict(parameter=candidate, generation=old['generation']+1, evidence_sha='challenger')
        snapshot = out/'challenger.json'; atomic(snapshot, new)
        par = int(kw.get('parallel', '3')); work = int(kw.get('work', '59147')); it = int(kw.get('iter', '40'))
        runtime=run_duration()
        report = dict(version='champion-gate_check-v2', net_class=nc, target_classes=targets, pooling=pooling, code=code, fpath=str(out/'gate_check.json'),
                      pre_test=dict(exit=None,status='Run only before an actual acceptance'),
                      measurement_contract=dict(budget='work', work=work, iter=it, seed=1, sets=('v10', 'v11'),
                          validity='evaluation+kicad_native-v1',rest='conflict_list+max(model_open,kicad_native_open)+native_error',reward=M.REWARD_VERSION,
                          statistics='one-sided exact sign test alpha=0.05, total residual must decrease',
                          reuse='Only identical source/input/cfg/seed/budget with unchanged native original evidence; no new run'),
                      candidate=candidate, training_rows=n, source_sha=source_sha, set_sha=M.sha(dataset), before_sha=M.sha(old), after_sha=M.sha(new),run_duration=runtime)


        before, after, sequence = sequential(dataset, old, new, out, par, work, it,
                                              refs=last_refs(root))
        report['sequence'] = sequence
        retryable=any(z.get('is_valid') is None or z.get('cur_state') not in ('ok','placer_collision')
                      for arm in (before,after) for z in arm.values())
        report['gate_check'] = M.gate_check(before, after)
        if not sequence['complete']:
            report['gate_check']['taken'] = False
            report['gate_check']['reasons'].append('Set incomplete: '+str(sequence['early_declined']))
        report['results_sha']=dict(champion=M.sha(before),challenger=M.sha(after))
        report['evidence_folder']={arm:{k:(z.get('reuse') or {}).get('folder',str(out/arm/k))
                                    for k,z in rows_.items()} for arm,rows_ in (('champion',before),('challenger',after))}
        report['reused']=sum(bool(z.get('reuse')) for z in after.values())
        report['reused_before']=sum(bool(z.get('reuse')) for z in before.values())
        report['curve_points'] = curve_points(before, after, report)
        if report['gate_check']['taken']:
            report['pre_test'] = pre_test(snapshot, dataset, out, par)
            if report['pre_test']['exit'] != 0:
                retryable=True
                report['gate_check']['taken']=False
                report['gate_check']['reasons'].append('Preliminary test failed')
        if code_state()!=source_sha or any(z.get('code_sha256')!=source_sha for arm in (before,after) for z in arm.values()):
            report['gate_check']['taken']=False
            report['gate_check']['reasons'].append('Code changed during gate measurement')
        if run_duration()!=runtime or any(z['champion_measurement'].get('run_duration_sha')!=M.sha(runtime) for arm in (before,after) for z in arm.values()):
            report['gate_check']['taken']=False
            report['gate_check']['reasons'].append('Runtime environment changed during gate measurement')
        report['cpu_s'] = gate_monitor.sample(os.getpid())-gate_cpu_start
        report['cpu_measurement'] = 'libproc process tree before/after gate, including preliminary test'
        report['load_before'] = gate_load_start
        report['load_after'] = os.getloadavg()
        atomic(out/'gate_check.json', report)
        if report.get('results_sha'):


            atomic(root/'last_measurement.json',dict(report=str(out.resolve()/'gate_check.json'),
                sha256=hashlib.sha256((out/'gate_check.json').read_bytes()).hexdigest(),
                arm='challenger' if report['gate_check']['taken'] else 'champion'))
        if report['gate_check']['taken']:
            event = accept(model, old, new, report, history, curve)
            atomic(out/'adoption.json', event)
        context=M.sha(dict(code=source_sha,champion=M.sha(old)))
        tried=state.get(nc,{}).get('checks',{})
        if not retryable:tried.setdefault(context,[]).append(M.sha(candidate))
        state[nc] = dict(n=n, code=code, candidate_sha=M.sha(candidate), gate_check=str(out/'gate_check.json'),checks=tried,
                        status='unknown_again_after_new_data' if retryable else 'decided')
        atomic(statepath, state)
        with (root/'loops.jsonl').open('a') as f:
            f.write(json.dumps(dict(time_s=time.time(),report=report['fpath'],gate_check=report['gate_check'],
                generation=M.read_in(model)['generation'],sequence=report.get('sequence')))+'\n')
            f.flush();os.fsync(f.fileno())
        return report


if __name__ == '__main__':
    from common.process_guard import bootstrap
    bootstrap(heavy=False)
    print(json.dumps(verify(dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)), indent=1))
