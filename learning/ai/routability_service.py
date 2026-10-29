import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import torch
from learning.ai.routability import Routability, Configuration, input_, hw_device, VERSION, PHYSICS
from learning.ai.routability_checking import atomic, sha, release_gate

ROOT=Path(__file__).resolve().parents[2]


def sample_from_board(B,K,RG=None,gnd_layers=None):
    from learning.ai.features_raster import features_raster
    data=features_raster(B,K,RG,gnd_layers)
    t={k:torch.as_tensor(v.copy()) for k,v in data.items() if v.dtype.kind in 'bifu'}
    names={k:v.tolist() for k,v in data.items() if v.dtype.kind not in 'bifu'}
    names['design_rules_json']=json.dumps(RG or {})
    return dict(tensors=t,names=names)


class Predictor:
    def __init__(self,path,device='mps'):
        self.path=Path(path);self.device=hw_device(device);self.meta=json.loads((self.path/'manifest.json').read_text());self.models=[]
        if self.meta['version']!=VERSION:raise ValueError('ModelVersion')
        for item in self.meta['models']:
            p=self.path/item['fpath']
            if sha(p)!=item['sha256']:raise ValueError('ModelHash')
            q=torch.load(p,map_location='cpu',weights_only=True)
            if q['version']!=VERSION:raise ValueError('CheckpointVersion')
            m=Routability(Configuration(**q['cfg']));m.load_state_dict(q['state']);m.to(self.device).eval();self.models.append(m)
        if not self.models:raise ValueError('leeres Ensemble')

    @torch.inference_mode()
    def predictions(self,samples,maps=False,calibrate=True):
        inputs=[input_(s,self.device) for s in samples]
        raw=[m(inputs) for m in self.models];preds=[]


        for member in raw:
            cpu_member=[]
            for record in member:
                keys=[k for k in record if not k.startswith('overload_')]
                joined=torch.cat([record[k].reshape(-1) for k in keys]).cpu();off=0;q={}
                for k in keys:
                    n=record[k].numel();q[k]=joined[off:off+n].reshape(record[k].shape);off+=n
                if maps:
                    for k in ('overload_mu','overload_logvar'):q[k]=record[k].cpu()
                cpu_member.append(q)
            preds.append(cpu_member)
        out=[]
        cal=(self.meta.get('calibration') or {}) if calibrate else {}
        def classification(p,key):
            temp=(cal.get('temperatures') or {}).get(key)
            mean=p.mean(0)
            if temp is not None:
                mean=torch.sigmoid(torch.logit(mean.clamp(1e-6,1-1e-6))/temp)
                p=torch.sigmoid(torch.logit(p.clamp(1e-6,1-1e-6))/temp)
            return mean,p.std(0,unbiased=False)
        for i in range(len(samples)):
            p=torch.stack([torch.sigmoid(v[i]['valid_logit']) for v in preds]);mean,pstd=classification(p,'is_valid')
            res=dict(p_valid=float(mean),
                valid_uncertainty=dict(entropy=float(-(mean.clamp(1e-6,1-1e-6)*mean.clamp(1e-6,1-1e-6).log()+(1-mean).clamp(1e-6,1-1e-6)*(1-mean).clamp(1e-6,1-1e-6).log())),ensemble_std=float(pstd)),
                calibrated=bool(cal.get('coverage_checked') and cal.get('classification_checked')))
            for name,key in [('score_v3','rating'),('physics','physics'),('net_physics','net_physics')]:
                mu=torch.stack([v[i][key+'_mu'] for v in preds]);lv=torch.stack([v[i][key+'_logvar'] for v in preds]);mean=mu.mean(0);var=lv.exp().mean(0)+mu.var(0,unbiased=False)

                std=var.sqrt();factor=cal.get('factor');factor=1.96 if factor is None else factor
                res[name]=dict(amount=torch.expm1(mean.clamp(0,20)).cpu().tolist(),log_mu=mean.cpu().tolist(),log_std=std.cpu().tolist(),
                    interval=[torch.expm1((mean-factor*std).clamp(0,20)).cpu().tolist(),torch.expm1((mean+factor*std).clamp(0,20)).cpu().tolist()])
            npred=torch.stack([torch.sigmoid(v[i]['net_logit']) for v in preds]);nmean,nstd=classification(npred,'net_name');res['net_p_joined']=nmean.cpu().tolist();res['net_std']=nstd.cpu().tolist()
            if maps:
                mu=torch.stack([v[i]['overload_mu'] for v in preds]);lv=torch.stack([v[i]['overload_logvar'] for v in preds])
                res['overload']=dict(log_mu=mu.mean(0).cpu().numpy(),log_std=(lv.exp().mean(0)+mu.var(0,unbiased=False)).sqrt().cpu().numpy())
            out.append(res)
        return out


def release_verify(version,deployment='selection'):
    if deployment not in ('selection','placer','judge'):raise ValueError('Deployment')
    p=Path(version);store=json.loads((p/'release.json').read_text());manifest=p/'manifest.json'
    cert=store.get('uses',{}).get(deployment,{})
    if store.get('version')!=2 or cert.get('deployment')!=deployment or cert.get('released') is not True or cert.get('manifest_sha256')!=sha(manifest):raise ValueError('Model not released for deployment')
    meta=json.loads(manifest.read_text())
    if meta.get('diagnose'):raise ValueError('Diagnostic model must not take effect')
    if meta.get('version')!=VERSION or not meta.get('models'):raise ValueError('Model version/ensemble missing')
    report=p/cert['report']
    if cert.get('report_sha256')!=sha(report):raise ValueError('Check report changed')
    r=json.loads(report.read_text())
    if r.get('deployment')!=deployment:raise ValueError('Check report for a different deployment')
    _family_contract(meta,r['test_rows'],r['end_pairs'],r['calibration'])
    from learning.ai.routability_checking import compare, benefit_gate
    if meta.get('calibration')!=r['calibration']:raise ValueError('Calibration not in manifest')

    st=compare(r['test_rows'])
    if st!=r.get('statistics'):raise ValueError('Stored statistics contradict raw measurements')
    actual=release_gate(st,benefit_gate(r['end_pairs'],deployment,root=p),r['calibration'],r.get('latency_ms'),deployment=deployment)
    if not actual['released']:raise ValueError('Release gate locked')
    for item in meta['models']:
        if sha(p/item['fpath'])!=item['sha256']:raise ValueError('Model hash changed')
    return meta


def accept(root,version,deployment='selection'):
    root=Path(root);version=Path(version).resolve();release_verify(version,deployment)
    if version.parent!=root.resolve():raise ValueError('Version must be under the model folder')
    root.mkdir(parents=True,exist_ok=True)
    with open(root/'.service.lock','a') as f:
        fcntl.flock(f,fcntl.LOCK_EX)
        old=json.loads((root/'current.json').read_text()) if (root/'current.json').exists() else None
        row=dict(version=version.name,manifest_sha256=sha(version/'manifest.json'),before=old,utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        atomic(root/'current.json',row)
        with open(root/'history.jsonl','a') as log:log.write(json.dumps(row,sort_keys=True)+'\n');log.flush();os.fsync(log.fileno())
    return row


class Service:
    def __init__(self,root=None,route=4,device='mps',deployment='selection',seed=20261008):
        self.root=Path(root or ROOT/'learning/model/routability');self.route=route;self.device=device;self.deployment=deployment;self.predictor=None;self.version=None;self.reason='no released model'
        import threading
        self.seed=seed;self.rng=np.random.default_rng(seed);self.selection_nr=0;self.rng_lock=threading.Lock()
        try:
            cur=json.loads((self.root/'current.json').read_text());p=self.root/cur['version'];release_verify(p,deployment)
            if sha(p/'manifest.json')!=cur['manifest_sha256']:raise ValueError('VersionPointer')
            self.predictor=Predictor(p,device);self.version=cur['version']
        except (OSError,ValueError,KeyError,RuntimeError) as e:self.reason=str(e)

    def selection(self,pool,category='routability',route=None,explore=1,samples=None):
        if category!='routability':raise ValueError('selection=routability')
        if self.predictor is not None and self.deployment!='selection':raise ValueError('Selection deployment not released')
        k=min(len(pool),max(1,route or self.route));ids=list(range(len(pool)))
        if self.predictor is None:

            from analysis.variants import key_heuristic
            ids.sort(key=lambda i:(key_heuristic(pool[i]['feats']),i));pred=None
        else:
            if samples is None:
                samples=[]
                for z in pool:
                    p=Path(z['routability_input']);B=json.loads((p/'board.json').read_text());K=json.loads((p/'netclasses.json').read_text())

                    rg=Path(z['routability_design_rules']);RG=json.loads(rg.read_text())
                    samples.append(sample_from_board(B,K,RG,z.get('gnd_layers')))
            pred=self.predictor.predictions(samples)
            ids.sort(key=lambda i:(-pred[i]['p_valid'],pred[i]['score_v3']['interval'][1],i))
        chosen=ids[:k];rest=ids[k:];e=min(max(0,explore),len(rest))
        with self.rng_lock:
            nr=self.selection_nr;self.selection_nr+=1
            if e:chosen+=self.rng.choice(rest,e,replace=False).tolist()
        return chosen,dict(category='routability' if pred is not None else 'heuristic',requested=category,version=self.version,
                          reason=self.reason if pred is None else None,forecasts=pred,chosen=chosen,
                          selection_probability={str(i):1. if i in ids[:k] else (e/len(rest) if rest else 0.) for i in ids},
                          exploration=dict(seed=self.seed,call=nr,method='sequential PCG64, without replacement'),
                          independent_final_check_required=True)

    def physics(self,samples):
        if self.predictor is None:return dict(cur_state='unknown',reason=self.reason)
        if self.deployment!='judge':raise ValueError('Judge deployment not released')
        return dict(cur_state='forecast',metrics=PHYSICS,forecasts=self.predictor.predictions(samples),
                    independent_final_check_required=True)

    def fine_costs(self,B,K,RG,T,layer_name,ch,gnd_layers=None):
        if self.predictor is None or self.deployment!='placer':raise ValueError('Placer cost term locked: '+self.reason)
        from learning.selection import board_off_placement
        states=[layer_name,dict(layer_name,**ch)];samples=[]
        for state in states:
            parts={}
            for ref,s in state.items():
                parts[ref]=dict(x=s[0],y=s[1],rotation=s[2],side=s[3])
            samples.append(sample_from_board(board_off_placement(B,dict(parts=parts),exact=True),K,RG,gnd_layers))
        p=self.predictor.predictions(samples)
        def c(v):return (1-v['p_valid'])*10+float(v['score_v3']['log_mu'])+float(v['score_v3']['log_std'])
        return c(p[1])-c(p[0])


def new_train(root,data,epochs=20,cpu_s=1800,at_night=False,gpu_free=False):
    hour=datetime.datetime.now().hour
    if not gpu_free or (at_night and not (0<=hour<7)):return dict(cur_state='skipped',reason='no free GPU/night')
    from common import slots as PL
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    with open(root/'.train.lock','a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return dict(cur_state='running_already')
        v=root/datetime.datetime.now(datetime.timezone.utc).strftime('v_%Y%m%dT%H%M%S_%f')
        run=PL.job(['caffeinate','-i','-s',sys.executable,'-m','learning.ai.routability_train',str(v),'--data',str(data),'--epochs',str(epochs)],cpu_s=cpu_s,log=str(root/'training.log'),cwd=str(ROOT))

        return dict(cur_state=run['cur_state'],version=str(v),job=run,accepted=False)


def _family_contract(meta,test_rows,end_pairs,calibration):
    from learning.ai.routability_checking import source_family,check_family
    for s in ('val','calibration','calibration_check','test'):
        for f in meta['data'].get(s,{}).get('families',[]):check_family(f,'endurance_run')
    train={source_family(f) for s in ('train','val') for f in meta['data'][s]['families']}
    fit={source_family(f) for f in meta['data'].get('calibration',{}).get('families',[])}
    fit|={source_family(f) for f in calibration.get('families',[])}
    check={source_family(f) for f in calibration.get('check_families',[])}
    check|={source_family(f) for f in meta['data'].get('calibration_check',{}).get('families',[])}

    fit|={source_family(r['family']) for r in calibration.get('fit_raw',[])}
    check|={source_family(r['family']) for r in calibration.get('checking_raw',[])}
    if train&(fit|check) or fit&check:raise ValueError('Calibration/train/selection families overlap')
    for r in test_rows+end_pairs:
        if 'family' not in r:raise ValueError('Final-check/test family missing')
        if source_family(r['family']) in train|fit|check:raise ValueError('Test family in training/checkpoint selection/calibration')


def certify(version,test_rows,end_pairs,calibration,latency_ms=None,deployment='selection'):
    from learning.ai.routability_checking import compare,benefit_gate
    if deployment not in ('selection','placer','judge'):raise ValueError('Deployment')
    p=Path(version);meta=json.loads((p/'manifest.json').read_text())
    if meta.get('diagnose'):raise ValueError('Diagnostic model not certifiable')
    _family_contract(meta,test_rows,end_pairs,calibration)
    if meta.get('calibration')!=calibration:raise ValueError('Calibration not in versioned model manifest')
    report=release_gate(compare(test_rows),benefit_gate(end_pairs,deployment,root=p),calibration,latency_ms,deployment=deployment)

    with open(p/'.release.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        name='checking_'+deployment+'.json'
        atomic(p/name,dict(report,test_rows=test_rows,end_pairs=end_pairs))
        cert=dict(deployment=deployment,released=report['released'],manifest_sha256=sha(p/'manifest.json'),report=name,report_sha256=sha(p/name))
        store=json.loads((p/'release.json').read_text()) if (p/'release.json').exists() else {}
        if store.get('version')!=2:store=dict(version=2,uses={})
        store['uses'][deployment]=cert
        atomic(p/'release.json',store)
    return cert


_TRAINING = {}
_TRAINING_LOCK = __import__('threading').Lock()


def training_requests(root,data,**options):
    import threading
    key=str(Path(root).resolve())
    with _TRAINING_LOCK:
        old=_TRAINING.get(key)
        if old and old.is_alive():return dict(cur_state='running_already')
        if not options.get('gpu_free'):return dict(cur_state='skipped',reason='GPU window not declared')
        if options.get('at_night') and not 0<=datetime.datetime.now().hour<7:return dict(cur_state='skipped',reason='no night window')
        def run():
            try:
                result=new_train(root,data,**options);atomic(Path(root)/'service_state.json',result)
            except Exception as e:atomic(Path(root)/'service_state.json',dict(cur_state='faults',reason=str(e)))
        thread=threading.Thread(target=run,name='routability-training',daemon=True);_TRAINING[key]=thread;thread.start()
        return dict(cur_state='requested',adoption='only via certify/accept')
