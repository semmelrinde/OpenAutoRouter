import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr
from sklearn.metrics import roc_auc_score

BASELINES = ('rudy', 'hpwl', 'collisions', 'selection', 'heuristic')
MEASURES = ('auc', 'spearman', 'pair_hits')
DESIGN_RULES = ('score_v3', 'dc', 'j', 'ret', 'xt', 'tap_close_mm', 'branch_middle',
          'parallel_rest_mm', 'sharp', 'coupling')


def source_family(f):
    if not isinstance(f,str) or not f.strip():raise ValueError('Source family missing')
    f=f.strip().lower()
    return 'generator:tasks-v9' if f.startswith('erf_') else f


def check_family(f,provenance):
    f=source_family(f)
    if f.startswith('generator:') or provenance!='endurance_run':
        raise ValueError('Check only on foreign continuous-run families, no selfplay')
    return f


def _numbers(rows, name):
    ys=[]; ps=[]; ns=[]; qs=[]; hits=[]
    for r in rows:
        pred=r['forecasts'][name]
        if r.get('candidate_id') is not None:
            ys.append(int(r['is_valid']));ps.append(pred['p_valid'])
            ns.append(r['score_v3']);qs.append(pred['score_v3'])
        elif r.get('better') in ('a','b'):
            hits.append(float(r['better']==pred['better']))
    return dict(auc=float(roc_auc_score(ys,ps)) if len(set(ys))==2 else None,
                spearman=float(spearmanr(ns,qs).statistic) if len(ns)>2 and len(set(ns))>1 and len(set(qs))>1 else None,
                pair_hits=float(np.mean(hits)) if hits else None)


def _finite(x): return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)


def _comparison_contract(rows):
    candidates={};pairs={}
    for r in rows:
        pid=r.get('pair_id');kid=r.get('candidate_id');family=check_family(r['family'],r.get('provenance'))
        if not isinstance(pid,str) or not pid:raise ValueError('Pair ID missing')
        if kid is not None:
            if 'better' in r or 'candidate_ids' in r:raise ValueError('mixed candidate/pair row')
            if not isinstance(kid,str) or not kid or kid in candidates:raise ValueError('Candidate ID missing/duplicate')
            candidates[kid]=(pid,family)
            if not isinstance(r.get('is_valid'),bool) or not _finite(r.get('score_v3')):raise ValueError('Candidate measurement missing')
        else:
            if 'is_valid' in r or 'score_v3' in r:raise ValueError('mixed candidate/pair row')
            ids=r.get('candidate_ids')
            if pid in pairs or not isinstance(ids,list) or len(ids)!=2 or len(set(ids))!=2 or r.get('better') not in ('a','b','equal'):
                raise ValueError('Pair measurement/IDs missing or duplicate')
            pairs[pid]=(ids,family)
        for name in ('model',)+BASELINES:
            pred=r.get('forecasts',{}).get(name)
            if not isinstance(pred,dict) or pred.get('pair_id')!=pid:raise ValueError('Forecast/pair ID missing: '+name)
            if kid is not None:
                if pred.get('candidate_id')!=kid or not _finite(pred.get('p_valid')) or not 0<=pred['p_valid']<=1 or not _finite(pred.get('score_v3')):
                    raise ValueError('Forecast/candidate ID missing: '+name)
            elif pred.get('candidate_ids')!=ids or pred.get('better') not in ('a','b'):
                raise ValueError('Forecast/pair content missing: '+name)
    for pid,(ids,family) in pairs.items():
        if any(candidates.get(kid)!=(pid,family) for kid in ids):raise ValueError('Pair without common candidates')
    expected={kid for ids,_ in pairs.values() for kid in ids}
    if expected!=set(candidates):raise ValueError('Candidate without complete pair')


def compare(rows, repetitions=2000, seed=20261008, min_families=10):
    if not rows: return dict(released=False,reasons=['no test data'],families=0)
    fs=sorted({source_family(r['family']) for r in rows}); groups={f:[r for r in rows if source_family(r['family'])==f] for f in fs}
    if any(r.get('split')!='test' for r in rows): raise ValueError('Release comparison test only')
    _comparison_contract(rows)
    names=('model',)+BASELINES; point={n:_numbers(rows,n) for n in names}; rng=np.random.default_rng(seed)
    deltas={m:[] for m in MEASURES}
    for _ in range(repetitions):
        rr=[r for f in rng.choice(fs,len(fs),replace=True) for r in groups[f]]
        values={n:_numbers(rr,n) for n in names}
        for m in MEASURES:
            v=values['model'][m]; bs=[values[n][m] for n in BASELINES]
            if _finite(v) and all(_finite(b) for b in bs): deltas[m].append(v-max(bs))
    ci={m:np.quantile(a,[.025,.975]).tolist() if len(a)>=.95*repetitions else None for m,a in deltas.items()}
    reasons=[]
    if len(fs)<min_families: reasons.append('too few independent test families')
    if any(any(not _finite(point[n][m]) for m in MEASURES) for n in names): reasons.append('Metric or strong baseline missing')
    if any(ci[m] is None or ci[m][0]<=0 for m in MEASURES): reasons.append('Confidence interval against best baseline not above zero')
    return dict(released=not reasons,reasons=reasons,families=len(fs),lines=len(rows),point=point,delta_ci95=ci,
                bootstrap=repetitions,seed=seed,min_families=min_families,unit='Source family, not pixel/seed')


def _native_verify(measurement,root):
    n=measurement['native_proof'];root=Path(root or '.')
    board=root/n['board'];report=root/n['report']
    if board.suffix!='.kicad_pcb' or sha(board)!=n['board_sha256'] or sha(report)!=n['report_sha256']:
        raise ValueError('native artifact hash')
    raw=json.loads(report.read_text())
    if Path(raw['source']).name!=board.name:raise ValueError('native check source')
    if not isinstance(raw['violations'],list) or not isinstance(raw['unconnected_items'],list):raise ValueError('native report incomplete')
    if any(not isinstance(v,dict) or v.get('severity') not in ('error','warning','ignore') for v in raw['violations']):raise ValueError('native error severity missing')
    native_ok=not raw['unconnected_items'] and not any(v['severity']=='error' for v in raw['violations'])
    if measurement['is_valid'] and not native_ok:raise ValueError('native DRC contradicts validity')


def benefit_gate(pairs,deployment='selection',root=None):
    if not pairs: return dict(released=False,reasons=['no independent final checks'])
    if deployment not in ('selection','placer','judge'):raise ValueError('Deployment')
    reasons=[]; cpu=[];ids=set();use=False
    for r in pairs:
        a,b=r.get('without',{}),r.get('with_',{})
        if any(r.get(k) is not True for k in ('same_task','same_budget','native_final_check','in_advance_fixed')):
            reasons.append('Pair contract/final check missing');continue
        if r.get('deployment')!=deployment:reasons.append('Final measurement does not belong to the deployment')
        if b.get('routability_deployment')!=deployment or a.get('routability_deployment') is not None:
            reasons.append('Active deployment missing in with/without final measurement')
        if not isinstance(r.get('id'),str) or not r['id'] or r['id'] in ids:reasons.append('Final pair ID missing/duplicate')
        else:ids.add(r['id'])
        try:check_family(r['family'],r.get('provenance'))
        except (KeyError,ValueError):reasons.append('Final check without foreign continuous-run family')
        if not isinstance(a.get('is_valid'),bool) or not isinstance(b.get('is_valid'),bool): reasons.append('Validity unknown'); continue
        for m in (a,b):
            try:_native_verify(m,root)
            except (OSError,ValueError,KeyError,TypeError):reasons.append('Native check report missing/invalid')
        if a['is_valid'] and not b['is_valid']: reasons.append('fewer valid')
        if b['is_valid'] and not a['is_valid']:use=True
        for k in DESIGN_RULES:
            x,y=a.get(k),b.get(k)
            if not _finite(x) or not _finite(y): reasons.append('Rule missing: '+k)
            elif y>x+1e-6: reasons.append('Rule worse: '+k)
            elif a['is_valid'] and b['is_valid'] and y<x-1e-6:use=True
        if any(not _finite(m.get('cpu_s')) or m['cpu_s']<0 for m in (a,b)): reasons.append('CPU measurement missing/invalid')
        else: cpu.append(b['cpu_s']-a['cpu_s'])
    if cpu and float(np.mean(cpu))<0 and all(r.get('without',{}).get('is_valid') is True and r.get('with_',{}).get('is_valid') is True for r in pairs):use=True
    if not use:reasons.append('No measured benefit for deployment')
    return dict(released=not reasons,reasons=sorted(set(reasons)),deployment=deployment,pairs=len(pairs),cpu_delta_medium=float(np.mean(cpu)) if cpu else None)


def release_gate(statistics, usage, calibration, latency_ms=None,deployment='placer'):
    reasons=list(statistics.get('reasons') or [])
    if statistics.get('released') is not True: reasons.append('Statistics gate locked')
    from learning.ai.routability_calibrate import proof_verify
    reasons+=proof_verify(calibration)
    if usage.get('released') is not True: reasons+=usage.get('reasons') or ['Benefit gate locked']
    if usage.get('deployment')!=deployment:reasons.append('Benefit evidence for a different deployment')
    if deployment not in ('selection','judge','placer'): raise ValueError('Deployment')
    if deployment=='placer' and (not _finite(latency_ms) or not 0<=latency_ms<20): reasons.append('End-to-end query not under 20 ms')
    return dict(released=not reasons,reasons=sorted(set(reasons)),statistics=statistics,usage=usage,
                calibration=calibration,latency_ms=latency_ms,deployment=deployment)


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic(path, value):
    import os,tempfile
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(dir=p.parent,prefix='.'+p.name)
    try:
        with os.fdopen(fd,'w') as f:
            json.dump(value,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(tmp,p)
    finally: Path(tmp).unlink(missing_ok=True)
