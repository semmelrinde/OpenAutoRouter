import json
import hashlib
import fcntl
from contextlib import contextmanager
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _preserved(a,b,keys):
    return all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for m in (a,b) for k in keys for v in [m.get(k)]) and all(b[k]<=a[k]+1e-9 for k in keys)


def gate_check(B, old, new, K, gnd_layers):
    from learning.collect import rule_measurement
    from analysis.rule_metrics import gate_fields, amount
    from analysis.r18 import measure
    from analysis.wobble import measure as wobble
    from analysis.length_impedance import length_list, receiver_protection, receiver_netclasses
    ms = [rule_measurement(B,q,K,gnd_layers,dense=True) for q in (old,new)]
    contract = evaluation_contract(*ms)
    a,b = [m.get('rule_gate') or m['design_rules'] for m in ms]
    keys = tuple(dict.fromkeys(gate_fields(a)+gate_fields(b)))
    missing = [k for k in keys if amount(a,k) is None or amount(b,k) is None]
    worse = [k for k in keys if k not in missing and amount(b,k)>amount(a,k)+1e-9]
    score_ok = _preserved(ms[0],ms[1],('score_v3',))
    shaping = [measure(B,q,K) for q in (old,new)]
    shape_ok = all(_preserved(shaping[0][s],shaping[1][s],('bends','hairpins','zigzag','bend_excess','detour_octi_mm')) for s in ('all_items','signal'))
    wa = [wobble(B,q,K.get('gnd') or []) for q in (old,new)]
    wobble_ok = _preserved(wa[0],wa[1],('move_records','moves_per_10mm','length_mm'))
    lk = receiver_netclasses(K)
    receiver_ok = receiver_protection(length_list(B,old,lk),length_list(B,new,lk))
    improved = any(shaping[1][s][k]<shaping[0][s][k] for s in ('all_items','signal') for k in ('bends','hairpins','zigzag','bend_excess','detour_octi_mm'))
    accepted = bool(ms[1]['is_valid'] and contract['not_worse'] and not missing and not worse and score_ok and shape_ok and wobble_ok and receiver_ok and improved)
    return dict(taken=accepted, contract_spec=contract, is_valid=[m['is_valid'] for m in ms], absent=missing, worse=worse,
                learning_score=[m.get('score_v3') for m in ms], learning_score_preserved=score_ok,
                r18=shaping, r18_preserved=shape_ok, wobble=wa, wobble_preserved=wobble_ok,
                receiver_gate=receiver_ok, improved=improved)



def evaluation_contract(a, b):
    gates = {}
    if a.get('gate_check') or b.get('gate_check'):
        from analysis.contract_spec import comparison
        if not a.get('gate_check') or not b.get('gate_check'):
            return dict(not_worse=False, reason='Required contract missing')
        gates['gate_check'] = comparison(a['gate_check'], b['gate_check'])
    if any('minimax_version' in e or 'minimax_valid' in e or e.get('evaluation_target') == 'minimax' for e in (a,b)):
        from analysis.minimax import assumption
        gates['minimax'] = assumption(*[dict(e, design_rules=e.get('rule_gate') or e.get('design_rules')) for e in (a,b)])
    return dict(not_worse=all(g['not_worse'] for g in gates.values()), gates=gates)


@contextmanager
def file_lock(rj):
    with Path(str(Path(rj).resolve())+'.r18.lock').open('a+b') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)



@contextmanager
def occupied_slot():
    from common import slots as PL
    job = PL.job
    def child(*args, **kw):
        kw['with_slot'] = False
        return job(*args, **kw)
    PL.job = child
    try: yield
    finally: PL.job = job

def swap(rj, original, candidate):
    with file_lock(rj):
        if Path(rj).read_bytes() != original: return False
        with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.r18_', delete=False) as f:
            tmp = Path(f.name); f.write(candidate)
        try:
            Path(rj[:-5]+'_before_r18.json').write_bytes(original)
            os.replace(tmp, rj)
        finally: tmp.unlink(missing_ok=True)
    return True


def references_freeze(value, artifact, base, loaded=None, images=None):
    loaded = {} if loaded is None else loaded
    images = {} if images is None else images
    def read_in(src):
        key = str(src.resolve())
        if key not in loaded: loaded[key] = src.read_bytes()
        return loaded[key]
    if isinstance(value, list):
        return [references_freeze(v, artifact, base, loaded, images) for v in value]
    if not isinstance(value, dict): return value
    out = {}
    for key, v in value.items():
        if key == 'board_file' and isinstance(v, str):
            src=Path(v).expanduser()
            if not src.is_absolute(): src=base/src
            dest=artifact/'source_list'/hashlib.sha256(str(src.resolve()).encode()).hexdigest()[:16]/src.name
            if str(dest) not in images:
                obj=references_freeze(json.loads(read_in(src)),artifact,src.parent,loaded,images)
                dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_text(json.dumps(obj)); images[str(dest)]=obj
            out[key]=str(dest)
        elif key in ('src', 'source') and isinstance(v, str) and Path(v).suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            src = Path(v).expanduser()
            if not src.is_absolute(): src = base / src
            if str(src.resolve()) not in loaded and not src.is_file():
                raise ValueError('Native snapshot source missing: ' + str(src))
            dest = artifact / 'source_list' / hashlib.sha256(str(src.resolve()).encode()).hexdigest()[:16] / src.name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(read_in(src))
            for ext in ('.kicad_pro', '.kicad_dru'):
                companion = src.with_suffix(ext)
                if str(companion.resolve()) in loaded or companion.is_file():
                    dest.with_suffix(ext).write_bytes(read_in(companion))
            out[key] = str(dest)
        else: out[key] = references_freeze(v, artifact, base, loaded, images)
    return out


def save(path, value):
    path=Path(path)
    with tempfile.NamedTemporaryFile(dir=path.parent,prefix='.r18_',delete=False,mode='w') as f:
        tmp=Path(f.name); json.dump(value,f)
    try: os.replace(tmp,path)
    finally: tmp.unlink(missing_ok=True)


class _Declined(Exception):
    ""


def apply(bj,kj,rj,design_rules,gnd_layers,cpu_s=180,raw_sha256=None,smoothed_sha256=None,final=None,minimax=None):
    from common import slots as PL
    rj = str(Path(rj).resolve())
    artifact = None
    try:
        with file_lock(rj): original = Path(rj).read_bytes()
        raw = Path(rj[:-5]+'_smoothed.json'); raw_data = Path(rj[:-5]+'_raw.json')

        if not raw_sha256 or not smoothed_sha256 or not raw.exists() or not raw_data.exists():
            raise _Declined('Current tightened base missing')
        raw_bytes = raw.read_bytes()
        if (hashlib.sha256(raw_data.read_bytes()).hexdigest()!=raw_sha256 or
            hashlib.sha256(raw_bytes).hexdigest()!=smoothed_sha256 or
            json.loads(raw_bytes).get('r18_raw_sha256')!=raw_sha256):
            raise _Declined('Intermediate artifacts do not belong to the current raw route')
        inputs=[json.loads(Path(p).read_text()) for p in (bj,kj,design_rules)]
        original_plan=None
        plan=Path(bj).parent/'zone_plan/zone_plan.json'

        loaded={}
        if (final or {}).get('mode') == 'local':


            from learning.zone_planner import check_inputs, sha
            try:
                original_plan=json.loads(plan.read_text())
                route=json.loads(original); smoothed=json.loads(raw_bytes)
                if not original_plan.get('board_file'):
                    raise ValueError('Zone plan without native board file')




                native=Path(original_plan['board_file']).expanduser()
                if not native.is_absolute(): native=plan.parent/native
                native_bytes=native.read_bytes()
                check_inputs(dict(original_plan,board_file=None),*inputs,
                    options=dict(route.get('options') or {},gnd_layers=','.join(gnd_layers)))
                if sha(json.loads(native_bytes)) != original_plan['input_hashes']['board']:
                    raise ValueError('Zone plan: native pad geometry changed')
                if any(q.get('zone_plan_id') != original_plan['id'] for q in (route,smoothed)):
                    raise ValueError('Zone plan ID of the current route/base missing or foreign')
                loaded[str(native.resolve())]=native_bytes
            except Exception as exc:
                raise _Declined('Zone plan original contract: '+str(exc))
        parent = Path(rj[:-5]+'_r18'); parent.mkdir(exist_ok=True)
        artifact = Path(tempfile.mkdtemp(prefix='run_',dir=parent))
        source=artifact/'input_.json'; raw_source=artifact/'smoothed.json'

        frozen=[]; frozen_objs=[]; images={}
        for (path,name),value in zip(((bj,'board.json'),(kj,'netclasses.json'),(design_rules,'design_rules.json')),inputs):
            dest=artifact/name
            snapshot=references_freeze(value, artifact, Path(path).resolve().parent, loaded, images)
            dest.write_text(json.dumps(snapshot)); frozen.append(str(dest)); frozen_objs.append(snapshot)
        plan_ids = None
        source_bytes, raw_source_bytes = original, raw_bytes
        if (final or {}).get('mode') == 'local':
            target=artifact/'zone_plan/zone_plan.json'; target.parent.mkdir()
            snapshot=references_freeze(original_plan,artifact,plan.parent,loaded,images)
            old_id=snapshot['id']



            snapshot['routing_board_sha256']=sha(frozen_objs[0])
            snapshot['input_hashes']=dict(snapshot['input_hashes'],netclasses=sha(frozen_objs[1]),
                design_rules=sha(frozen_objs[2]),board=sha(images[snapshot['board_file']]))
            snapshot['id']=sha({k:v for k,v in snapshot.items() if k not in ('id','board_file')})
            plan_ids=(old_id,snapshot['id'])
            target.write_text(json.dumps(snapshot))
            def bind(data):
                q=json.loads(data)
                if q.get('zone_plan_id')!=old_id: return data
                q['zone_plan_id']=snapshot['id']; return json.dumps(q).encode()
            source_bytes, raw_source_bytes = bind(original), bind(raw_bytes)
        source.write_bytes(source_bytes); raw_source.write_bytes(raw_source_bytes)
        (artifact/'snapshot.json').write_text(json.dumps({str(p.relative_to(artifact)):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(artifact.rglob('*')) if p.is_file()},indent=1))
    except _Declined as exc:
        return dict(applied=False,completed=False,reason=str(exc),cpu_s=0.)
    except Exception as exc:
        if artifact is not None: shutil.rmtree(artifact, ignore_errors=True)
        return dict(applied=False,completed=False,reason='R18-Snapshot: '+repr(exc)[:300],cpu_s=0.)
    bj,kj,design_rules=frozen
    result=artifact/'info.json'
    run=PL.job([str(ROOT/'.venv/bin/python'), str(Path(__file__)), bj,kj,str(source),str(raw_source),design_rules,
        json.dumps(list(gnd_layers)),str(artifact),json.dumps(final),str(cpu_s),json.dumps(minimax)],
        cpu_s=cpu_s, cwd=str(ROOT),log=str(artifact/'worker.log'),env=dict(os.environ,PYTHONHASHSEED='0'))
    checkpoint=json.loads(result.read_text()) if result.exists() else {}
    info=checkpoint if run['cur_state']=='ok' or checkpoint.get('completed') else dict(
        applied=False,reason='R18-Worker: '+run['cur_state'])
    if run['cur_state'] != 'ok':
        info.update(applied=False,reason='R18 proposal: '+run['cur_state'])
        if info.get('reference_final') is not None: info['final']=info['reference_final']
    info.update(compute_slot=run,cpu_s=run['cpu_s'],workdir=str(artifact))
    if info.get('applied') or info.get('completed'):
        selected=artifact/'routes.json' if run['cur_state']=='ok' else artifact/'reference/routes.json'
        output=selected.read_bytes()
        if plan_ids:
            q=json.loads(output)
            if q.get('zone_plan_id')==plan_ids[1]:
                q['zone_plan_id']=plan_ids[0];output=json.dumps(q).encode()
        if not swap(rj,original,output):
            info.update(applied=False,completed=False,reason='Route changed in the meantime')
    return info


def worker(bj,kj,rj,raw,design_rules,gnd,folder,final='null',cpu_s='180',minimax='null'):
    from postprocessing.r18 import suggestion
    from postprocessing.pulltight_routes import optimize
    from postprocessing.smooth45 import via_middle
    from analysis.evaluation import not_connected
    from learning.shaping import finalize
    from learning.cpu_budget import Budget
    from learning.zone_planner import acceptance_gate
    budget=Budget(float(cpu_s)); gnd=json.loads(gnd); finish=json.loads(final)
    B,K,old,R0,RG=[json.loads(Path(p).read_text()) for p in (bj,kj,rj,raw,design_rules)]

    mm=json.loads(minimax)
    if mm is not None:
        K=dict(K,specs=dict(K.get('specs') or {},evaluation='minimax' if mm else 'v3'))
        kj=str(Path(folder,'netclasses.json')); Path(kj).write_text(json.dumps(K))

    reference=Path(folder,'reference'); reference.mkdir()
    refpath=reference/'routes.json'; refpath.write_text(json.dumps(old))
    def close(path):
        if not finish: return {}
        with occupied_slot():
            return finalize(bj,kj,str(path),design_rules,gnd,finish['cfg'],finish['mode'],finish['r36'],budget)
    refinfo=close(refpath)
    old=json.loads(refpath.read_text()); selected=old
    info=dict(applied=False, completed=bool(finish), final=refinfo, reference_final=refinfo)

    save(Path(folder,'routes.json'),selected)
    save(Path(folder,'info.json'),info)
    try:
        if budget.rest < 1: raise TimeoutError('R18 search: CPU budget exhausted')
        new, proposal=suggestion(B,R0,K)
        info.update(proposal)
        if info['changed']:
            if budget.rest < 1: raise TimeoutError('R18 rounding: CPU budget exhausted')
            new=optimize(B,new,RG['router'],K,pull=False,rmax=.5,connectivity=not_connected)
            new['via_middle']=via_middle(new)
            Path(folder,'candidate.json').write_text(json.dumps(new))
            if budget.rest < 1: raise TimeoutError('R18 shape gate: CPU budget exhausted')
            info['gate_check']=gate_check(B,old,new,K,gnd)
            if info['gate_check']['taken']:
                candidate=Path(folder,'suggestion');candidate.mkdir()
                cp=candidate/'routes.json';cp.write_text(json.dumps(new))
                candidate_info=close(cp); new=json.loads(cp.read_text())
                if budget.rest < 1: raise TimeoutError('R18 completion gate: CPU budget exhausted')
                mode=finish['mode'] if finish else 'off'; r36=finish['r36'] if finish else '0'
                duty='kicad_native' if mode=='local' or r36=='kicad_native' else ('legacy' if r36=='legacy' else False)
                closure_ok=acceptance_gate(dict(is_valid=True),candidate_info.get('current_zones'),duty)['is_valid']
                info['termination_gate']=gate_check(B,old,new,K,gnd)
                if closure_ok and info['termination_gate']['taken']:
                    if budget.rest < 1: raise TimeoutError('R18 native: CPU budget exhausted')
                    from learning.copper_kicad_native import verify
                    own=time.thread_time()
                    _,native_new,native=verify(Path(folder,'kicad_native'),B,old,new,RG,cpu_s=min(120,budget.rest))


                    budget.add(max(0.,native['cpu_s']-(time.thread_time()-own)))
                    info['kicad_native']=native
                    if budget.rest < 1: raise TimeoutError('R18 final gate: CPU budget exhausted')
                    info['end_gate']=gate_check(B,old,native_new,K,gnd)
                    if native['suggestion']['is_valid'] and info['end_gate']['taken'] and budget.rest>0:
                        selected=native_new
                        info.update(applied=True,final=candidate_info)
    except Exception as exc:
        info['final_error']=repr(exc)[:500]
    info['reason']='' if info['applied'] else 'No winning proposal under all shape/completion/rule gates'
    info['cpu_budget']=dict(used=budget.used,children=budget.children,rest=budget.rest)
    save(Path(folder,'routes.json'),selected)
    save(Path(folder,'info.json'),info)


if __name__=='__main__': worker(*sys.argv[1:])
