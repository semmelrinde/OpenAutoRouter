import fcntl
import hashlib
import json
import os
from pathlib import Path
import time

from learning.shape_features import off_measurement, VERSION
from learning.pair_contract import measurement_manifest

CONFIG = dict(version='shaping-live-v2', evaluation=dict(v3=True, minimax=False),
              diagnose=dict(r39_diagnose='1', via_density_diagnose='1', k20_measure=True),
              workbench=dict(only_shape=True), rule_measurement=dict(dense=True))


def signature():
    return hashlib.sha256(json.dumps(dict(source_list=measurement_manifest(), configuration=CONFIG),
                                        sort_keys=True, allow_nan=False).encode()).hexdigest()


def stable(v):
    if isinstance(v, dict):
        return {k:stable(x) for k,x in v.items() if k not in ('cpu_s', 'time_s', 'time_ms', 'checker_cpu_s')}
    if isinstance(v, list): return [stable(x) for x in v]
    return v


def measure(g, h, sig):
    from analysis.evaluation import rate
    from learning.collect import rule_measurement
    b,r,k = g['board'],g['routes'],g['netclasses']
    e = rate(b,r,k,**CONFIG['evaluation'])


    m = e['design_rules']
    rm = rule_measurement(b,r,k,r.get('gnd_layers') or [],dense=True,v3_evaluation=e)
    rm.pop('_physics',None)
    shaping,ke,w,cost = shape_measure(g,h,sig,m,e.get('measures') or {})
    return stable(dict(outcome=e, rule_metrics=m, rule=rm, current_chain=ke,
                       zones_shape=w, shape_measurement=shaping)), cost


def shape_measure(g,h,sig,m,measures=None):
    from analysis import rule_metrics as RK
    from analysis.current_chain import link_chain
    from analysis.minimax import load_error
    from learning.zones_workbench import measure as workbench
    t=time.process_time()
    b,r,k=g['board'],g['routes'],g['netclasses']
    measures=measures or {}


    import copy
    fm = copy.deepcopy(m)
    rd = dict(r, options=dict(r.get('options') or {}, **CONFIG['diagnose']))
    if 'current_bottlenecks' not in fm and 'r39_error' not in fm:
        RK.r39_measure(b,rd,k,r.get('gnd_layers') or [],fm,{})
    RK._r40_diagnose(fm,rd,k)
    RK._via_diagnose(fm,b,rd)
    RK._k20_diagnose(fm,b,rd,k)
    RK._k17_diagnose(fm,b,rd,k)
    ke = measures.get('current_chain')
    if ke is None:
        try:


            ir=measures.get('copper_ir') if not any(load_error(v) for v in k.get('net_list',{}).values()) else None
            ke = link_chain(b,r,k,ir=ir)
        except Exception as exc: ke=dict(faults=repr(exc))
    w = workbench(b,r,k,r.get('gnd_layers') or [],design_rules=g.get('design_rules'),
                 current_chain=ke, bottlenecks=fm.get('current_bottlenecks'), only_shape=True)
    return off_measurement(w,fm,h,sig),ke,w,dict(extra_cpu_s=time.process_time()-t)


def remeasure(g, h, cache=None):
    start = time.process_time()
    sig = signature()
    folder = Path(cache or os.environ.get('AUTOROUTER_SHAPE_CACHE',
                    '~/Library/Caches/Autorouter/shape_measurement')).expanduser()
    folder.mkdir(parents=True,exist_ok=True)
    path = folder/f'{h}_{sig}.json'
    with (folder/f'{h}_{sig}.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        try: old = json.loads(path.read_text())
        except (OSError, ValueError): old = {}
        fm = (old.get('measured') or {}).get('shape_measurement') or {}
        hit = (old.get('geometry_sha256') == h and old.get('evaluator_sha256') == sig
               and old.get('complete') is True and 'faults' not in old
               and fm.get('version') == VERSION and fm.get('geometry_sha256') == h
               and fm.get('evaluator_sha256') == sig and fm.get('complete') is True
               and all(isinstance((old.get('measured') or {}).get(k),dict)
                       for k in ('outcome','rule_metrics','rule','current_chain','zones_shape')))
        if not hit:
            t = time.process_time()
            result = measure(g,h,sig)

            if isinstance(result,tuple): result,part_list=result
            else: part_list={}
            if signature() != sig: raise ValueError('Evaluator changed during re-measurement')
            old = dict(geometry_sha256=h, evaluator_sha256=sig, complete=True,
                       measured=result, cpu_s=time.process_time()-t, parts_cpu_s=part_list)
            from learning.judge import atomic
            atomic(path,old)
        if hit and signature() != sig: raise ValueError('Evaluator changed during cache access')
    return old['measured'], dict(cache=hit, cpu_s=time.process_time()-start,
                                 measure_core_cpu_s=0.0 if hit else old['cpu_s'],
                                 original_cpu_s=old['cpu_s'], parts_cpu_s=old.get('parts_cpu_s') or {}, load=os.getloadavg()[0])
