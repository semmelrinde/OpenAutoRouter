import argparse
import hashlib
import json
import random
import subprocess
import time
from pathlib import Path
import numpy as np
import torch
from learning.ai.data import RasterDataset, VERSION as DATA_VERSION, OBSERVER_VERSION, GOAL_HASH_VERSION, READ_CONTRACT
from learning.ai.routability import Routability, Configuration, input_, power_loss, hw_device, VERSION
ROOT=Path(__file__).resolve().parents[2]
from learning.ai.routability_checking import atomic, source_family, sha


def exclusion_hashes(fpath=None):
    from learning.learning_curve import FOLDER,LEVELS,SETS
    from learning.ai.data import _check_hash
    path=Path(fpath or Path(__file__).with_name('routability_exclusion_v1.json'))
    data=json.loads(path.read_text());refs={r['fpath']:r for r in data['boards']}
    expected={f'{folder}/s{st}_{seed}/board.json' for seeds,folder,_ in SETS.values() for st in LEVELS for seed in seeds}
    if data.get('version')!=1 or set(refs)!=expected or len(refs)!=len(data['boards']):raise ValueError('Exclusion hash version incomplete')
    blocked=set();missing=[]
    for rel in sorted(refs):
        r=refs[rel];hashes=(r['raw_sha256'],r['canonical_sha256'])
        if any(not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h) for h in hashes):raise ValueError('Exclusion hash invalid')
        blocked.update(hashes);p=Path(FOLDER)/rel
        try:stat=p.stat()
        except FileNotFoundError:missing.append(rel);continue
        if _check_hash(str(p),stat.st_mtime_ns,stat.st_size)!=hashes:raise ValueError('Reference changed; version the exclusion hashes anew: '+rel)
    return blocked,dict(version=1,sha256=sha(path),refs=len(refs),missing_refs=missing)


def snapshot(root, dest):
    from learning.ai.data import keepout
    root=Path(root)
    with keepout(root, exclusive=False): raw=(root/'index.jsonl').read_bytes()
    lines=raw.splitlines(keepends=True)
    rows=[json.loads(l) for l in lines if l.endswith(b'\n')]
    atomic(dest,dict(index_sha256=hashlib.sha256(raw).hexdigest(),rows=rows,root=str(root.resolve())))
    return rows


def datasets(root, rows):
    allrows=RasterDataset(root,split='all_items',check_hash=True,rows=rows)
    blocked,proof=exclusion_hashes();frozen={r['id']:r for r in rows};eligible=[];roles={}
    for r in allrows.rows:
        if frozen.get(r['id'])!=r or r.get('cur_state')!='ok':continue
        if r.get('board_sha256') in blocked or r.get('source_board_sha256') in blocked:continue
        r=dict(r);family=source_family(r['design_family'])

        if family.startswith('generator:') or r.get('provenance')=='selfplay':
            r.update(design_family='generator:tasks-v9',split='train',origin_split=r['split'])
            family='generator:tasks-v9'
        elif r.get('provenance')!='endurance_run':continue
        roles.setdefault(family,set()).add(r['split']);eligible.append(r)
    leaks={f for f,v in roles.items() if len(v)>1}
    ds={s:RasterDataset(root,split='all_items',check_hash=True,rows=[]) for s in ('train','val','test')}
    for s,d in ds.items():
        d.rows=[r for r in eligible if r['split']==s and source_family(r['design_family']) not in leaks]
        d.exclusion_proof=proof


    cal=RasterDataset(root,split='val',check_hash=True,rows=[])
    cal.rows=[r for r in ds['val'].rows if int(hashlib.sha256(('routability-cal-v1:'+source_family(r['design_family'])).encode()).hexdigest()[:8],16)%2==0]
    calids={r['id'] for r in cal.rows};ds['val'].rows=[r for r in ds['val'].rows if r['id'] not in calids]
    check=RasterDataset(root,split='val',check_hash=True,rows=[])
    check.rows=[r for r in cal.rows if int(hashlib.sha256(('routability-coverage-v2:'+source_family(r['design_family'])).encode()).hexdigest()[:8],16)%2==0]
    check_ids={r['id'] for r in check.rows};cal.rows=[r for r in cal.rows if r['id'] not in check_ids]
    ds['calibration']=cal
    ds['calibration_check']=check
    return ds, sorted(leaks)


def buckets(samples,batch,rng=None):

    ids=sorted(range(len(samples)),key=lambda i:(samples[i]['tensors']['input_'].shape[0],tuple((v+63)//64 for v in samples[i]['tensors']['input_'].shape[-2:])))
    chunks=[ids[i:i+batch] for i in range(0,len(ids),batch)]
    if rng:rng.shuffle(chunks)
    return [[samples[i] for i in c] for c in chunks]


def train(root,out,epochs=20,batch=2,ensemble=3,device='mps',seed=20261008,diagnose=False):
    out=Path(out)
    if (out/'manifest.json').exists():raise ValueError('Version exists; never overwrite')
    out.mkdir(parents=True,exist_ok=True);rows=snapshot(root,out/'snapshot.json');ds,leaks=datasets(root,rows)
    sizes={s:dict(lines=len(d),families=sorted({source_family(r['design_family']) for r in d.rows})) for s,d in ds.items()}
    if not len(ds['train']):raise ValueError('no leak-free training data')
    if not diagnose and (len(sizes['train']['families'])<2 or not len(ds['val'])):
        atomic(out/'abort.json',dict(reason='real train/val families missing',data=sizes,blocked_families=leaks));return None
    if epochs<1 or batch<1 or ensemble<1:raise ValueError('TrainingParameters')
    source={str(p):sha(p) for p in (Path(__file__).parent).glob('routability*.py')}
    dev=hw_device(device);torch.set_num_threads(2);cpu0=time.process_time(); wall0=time.monotonic()
    samples={s:[d[i] for i in range(len(d))] for s,d in ds.items()};models=[];history=[]
    for member in range(ensemble):
        torch.manual_seed(seed+member);random.seed(seed+member);np.random.seed(seed+member)
        m=Routability().to(dev); opt=torch.optim.AdamW(m.parameters(),lr=2e-4,weight_decay=1e-4);rng=random.Random(seed+member);best=float('inf'); state=None
        for epoch in range(epochs):
            m.train();tr=[]
            for batch_s in buckets(samples['train'],batch,rng):
                opt.zero_grad(set_to_none=True);p=m([input_(s,dev) for s in batch_s]);ls=[power_loss(v,s)[0] for v,s in zip(p,batch_s)]
                loss=torch.stack(ls).mean()
                if not torch.isfinite(loss):raise ValueError('non-finite training loss')
                loss.backward();torch.nn.utils.clip_grad_norm_(m.parameters(),5);opt.step();tr.append(float(loss.detach()))
            m.eval();va=[]
            with torch.inference_mode():
                for batch_s in buckets(samples['val'],batch):
                    va.extend(float(power_loss(v,s)[0]) for v,s in zip(m([input_(s,dev) for s in batch_s]),batch_s))
            score=float(np.mean(va if va else tr));h=dict(model=member,epoch=epoch,train_loss=float(np.mean(tr)),val_loss=float(np.mean(va)) if va else None);history.append(h);print(h,flush=True)
            if score<best:best=score;state={k:v.detach().cpu().clone() for k,v in m.state_dict().items()}
        path=out/('model_%d.pt'%member);torch.save(dict(version=VERSION,cfg=vars(m.cfg),state=state),path)
        models.append(dict(fpath=path.name,sha256=sha(path)));del m,opt
    if dev.type=='mps':torch.mps.synchronize()
    if any(sha(ROOT / p)!=h for p,h in source.items()): raise RuntimeError('Source code changed during training; no manifest')
    manifest=dict(version=VERSION, data_contract=dict(label_version=DATA_VERSION, observer_version=OBSERVER_VERSION, goal_hash_version=GOAL_HASH_VERSION, read_contract=READ_CONTRACT), diagnose=diagnose,models=models,seed=seed,epochs=epochs,batch=batch,hw_device=str(dev),
                  parameter=sum(v.numel() for v in state.values()),data=sizes,blocked_families=leaks,history=history,
                  cpu_s=time.process_time()-cpu0,wall_s=time.monotonic()-wall0,load=subprocess.getoutput('uptime'),
                  router_sha256=sha('router/route_pf.py'),netclasses_sha256=sha('analysis/netclasses.py'),
                  code=source,exclusion_proof=ds['train'].exclusion_proof,
                  split_rule='routability-v2: selfplay one generator family train only; val/calibration/test only foreign continuous run',
                  released=False,reason='Gate and independent final check required separately',calibration=None)
    atomic(out/'manifest.json',manifest)
    if not diagnose and samples['calibration']:
        from learning.ai.routability_service import Predictor
        from learning.ai.routability_calibrate import calibrate, verify
        predictor=Predictor(out,device)
        forbidden=sizes['train']['families']+sizes['val']['families']+sizes['test']['families']
        manifest['calibration']=calibrate(predictor,samples['calibration'],banned=forbidden+sizes['calibration_check']['families'])
        manifest['calibration']=verify(predictor,samples['calibration_check'],manifest['calibration'],banned=forbidden)
        atomic(out/'manifest.json',manifest)
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('off');p.add_argument('--data',default=str(Path.home()/'Library/Caches/Autorouter/dataset'))
    p.add_argument('--epochs',type=int,default=20);p.add_argument('--batch',type=int,default=2);p.add_argument('--ensemble',type=int,default=3);p.add_argument('--hw_device',default='mps');p.add_argument('--diagnose',action='store_true')
    a=p.parse_args();train(a.data,a.off,a.epochs,a.batch,a.ensemble,a.hw_device,diagnose=a.diagnose)
