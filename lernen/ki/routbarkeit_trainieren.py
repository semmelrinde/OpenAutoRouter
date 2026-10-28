""
import argparse
import hashlib
import json
import random
import subprocess
import time
from pathlib import Path
import numpy as np
import torch
from lernen.ki.daten import RasterDataset, VERSION as DATEN_VERSION, BEOBACHTERVERSION, ZIEL_HASH_VERSION, LESEVERTRAG
from lernen.ki.routbarkeit import Routbarkeit, Konfiguration, eingang, verlust, geraet, VERSION
ROOT=Path(__file__).resolve().parents[2]
from lernen.ki.routbarkeit_pruefung import atomar, quellfamilie, sha


def ausschlusshashes(pfad=None):
    ""
    from lernen.lernkurve import ORDNER,STUFEN,SAETZE
    from lernen.ki.daten import _pruefhash
    path=Path(pfad or Path(__file__).with_name('routbarkeit_ausschluss_v1.json'))
    data=json.loads(path.read_text());refs={r['pfad']:r for r in data['boards']}
    expected={f'{folder}/s{st}_{seed}/board.json' for seeds,folder,_ in SAETZE.values() for st in STUFEN for seed in seeds}
    if data.get('version')!=1 or set(refs)!=expected or len(refs)!=len(data['boards']):raise ValueError('Ausschlusshash-Version unvollständig')
    blocked=set();missing=[]
    for rel in sorted(refs):
        r=refs[rel];hashes=(r['raw_sha256'],r['kanonisch_sha256'])
        if any(not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h) for h in hashes):raise ValueError('Ausschlusshash ungültig')
        blocked.update(hashes);p=Path(ORDNER)/rel
        try:stat=p.stat()
        except FileNotFoundError:missing.append(rel);continue
        if _pruefhash(str(p),stat.st_mtime_ns,stat.st_size)!=hashes:raise ValueError('Referenz geändert; Ausschlusshashes neu versionieren: '+rel)
    return blocked,dict(version=1,sha256=sha(path),referenzen=len(refs),fehlende_referenzen=missing)


def snapshot(root, dest):
    ""
    from lernen.ki.daten import sperre
    root=Path(root)
    with sperre(root, exklusiv=False): raw=(root/'index.jsonl').read_bytes()
    lines=raw.splitlines(keepends=True)
    rows=[json.loads(l) for l in lines if l.endswith(b'\n')]
    atomar(dest,dict(index_sha256=hashlib.sha256(raw).hexdigest(),rows=rows,root=str(root.resolve())))
    return rows


def datensaetze(root, rows):
    allrows=RasterDataset(root,split='alle',pruefe_hash=True,rows=rows)
    blocked,proof=ausschlusshashes();frozen={r['id']:r for r in rows};eligible=[];roles={}
    for r in allrows.rows:
        if frozen.get(r['id'])!=r or r.get('zustand')!='ok':continue
        if r.get('board_sha256') in blocked or r.get('quellboard_sha256') in blocked:continue
        r=dict(r);family=quellfamilie(r['designfamilie'])

        if family.startswith('generator:') or r.get('herkunft')=='selbstspiel':
            r.update(designfamilie='generator:aufgaben-v9',split='train',ursprung_split=r['split'])
            family='generator:aufgaben-v9'
        elif r.get('herkunft')!='dauerlauf':continue
        roles.setdefault(family,set()).add(r['split']);eligible.append(r)
    leaks={f for f,v in roles.items() if len(v)>1}
    ds={s:RasterDataset(root,split='alle',pruefe_hash=True,rows=[]) for s in ('train','val','test')}
    for s,d in ds.items():
        d.rows=[r for r in eligible if r['split']==s and quellfamilie(r['designfamilie']) not in leaks]
        d.ausschlussnachweis=proof


    cal=RasterDataset(root,split='val',pruefe_hash=True,rows=[])
    cal.rows=[r for r in ds['val'].rows if int(hashlib.sha256(('routbarkeit-kal-v1:'+quellfamilie(r['designfamilie'])).encode()).hexdigest()[:8],16)%2==0]
    calids={r['id'] for r in cal.rows};ds['val'].rows=[r for r in ds['val'].rows if r['id'] not in calids]
    check=RasterDataset(root,split='val',pruefe_hash=True,rows=[])
    check.rows=[r for r in cal.rows if int(hashlib.sha256(('routbarkeit-abdeckung-v2:'+quellfamilie(r['designfamilie'])).encode()).hexdigest()[:8],16)%2==0]
    checkids={r['id'] for r in check.rows};cal.rows=[r for r in cal.rows if r['id'] not in checkids]
    ds['kalibrierung']=cal
    ds['kalibrierpruefung']=check
    return ds, sorted(leaks)


def buckets(samples,batch,rng=None):

    ids=sorted(range(len(samples)),key=lambda i:(samples[i]['tensoren']['eingang'].shape[0],tuple((v+63)//64 for v in samples[i]['tensoren']['eingang'].shape[-2:])))
    chunks=[ids[i:i+batch] for i in range(0,len(ids),batch)]
    if rng:rng.shuffle(chunks)
    return [[samples[i] for i in c] for c in chunks]


def trainieren(root,out,epochs=20,batch=2,ensemble=3,device='mps',seed=20261008,diagnose=False):
    out=Path(out)
    if (out/'manifest.json').exists():raise ValueError('Version existiert; nie überschreiben')
    out.mkdir(parents=True,exist_ok=True);rows=snapshot(root,out/'snapshot.json');ds,leaks=datensaetze(root,rows)
    sizes={s:dict(zeilen=len(d),familien=sorted({quellfamilie(r['designfamilie']) for r in d.rows})) for s,d in ds.items()}
    if not len(ds['train']):raise ValueError('keine leckfreien Trainingsdaten')
    if not diagnose and (len(sizes['train']['familien'])<2 or not len(ds['val'])):
        atomar(out/'abbruch.json',dict(grund='reale train/val-Familien fehlen',daten=sizes,gesperrte_familien=leaks));return None
    if epochs<1 or batch<1 or ensemble<1:raise ValueError('Trainingsparameter')
    source={str(p):sha(p) for p in (Path(__file__).parent).glob('routbarkeit*.py')}
    dev=geraet(device);torch.set_num_threads(2);cpu0=time.process_time(); wall0=time.monotonic()
    samples={s:[d[i] for i in range(len(d))] for s,d in ds.items()};models=[];history=[]
    for member in range(ensemble):
        torch.manual_seed(seed+member);random.seed(seed+member);np.random.seed(seed+member)
        m=Routbarkeit().to(dev); opt=torch.optim.AdamW(m.parameters(),lr=2e-4,weight_decay=1e-4);rng=random.Random(seed+member);best=float('inf'); state=None
        for epoch in range(epochs):
            m.train();tr=[]
            for batch_s in buckets(samples['train'],batch,rng):
                opt.zero_grad(set_to_none=True);p=m([eingang(s,dev) for s in batch_s]);ls=[verlust(v,s)[0] for v,s in zip(p,batch_s)]
                loss=torch.stack(ls).mean()
                if not torch.isfinite(loss):raise ValueError('nicht endlicher Trainingsverlust')
                loss.backward();torch.nn.utils.clip_grad_norm_(m.parameters(),5);opt.step();tr.append(float(loss.detach()))
            m.eval();va=[]
            with torch.inference_mode():
                for batch_s in buckets(samples['val'],batch):
                    va.extend(float(verlust(v,s)[0]) for v,s in zip(m([eingang(s,dev) for s in batch_s]),batch_s))
            score=float(np.mean(va if va else tr));h=dict(modell=member,epoche=epoch,train_loss=float(np.mean(tr)),val_loss=float(np.mean(va)) if va else None);history.append(h);print(h,flush=True)
            if score<best:best=score;state={k:v.detach().cpu().clone() for k,v in m.state_dict().items()}
        path=out/('modell_%d.pt'%member);torch.save(dict(version=VERSION,cfg=vars(m.cfg),state=state),path)
        models.append(dict(pfad=path.name,sha256=sha(path)));del m,opt
    if dev.type=='mps':torch.mps.synchronize()
    if any(sha(ROOT / p)!=h for p,h in source.items()): raise RuntimeError('Quellcode während Training geändert; kein Manifest')
    manifest=dict(version=VERSION, datenvertrag=dict(etikettenversion=DATEN_VERSION, beobachterversion=BEOBACHTERVERSION, ziel_hash_version=ZIEL_HASH_VERSION, lesevertrag=LESEVERTRAG), diagnose=diagnose,modelle=models,seed=seed,epochen=epochs,batch=batch,geraet=str(dev),
                  parameter=sum(v.numel() for v in state.values()),daten=sizes,gesperrte_familien=leaks,history=history,
                  cpu_s=time.process_time()-cpu0,wand_s=time.monotonic()-wall0,last=subprocess.getoutput('uptime'),
                  router_sha256=sha('router/route_pf.py'),klassen_sha256=sha('analyse/klassen.py'),
                  code=source,ausschlussnachweis=ds['train'].ausschlussnachweis,
                  split_regel='routbarkeit-v2: Selbstspiel eine Generatorfamilie nur train; val/kalibrierung/test nur fremder Dauerlauf',
                  freigegeben=False,grund='Tor und unabhängige Endprüfung separat erforderlich',kalibrierung=None)
    atomar(out/'manifest.json',manifest)
    if not diagnose and samples['kalibrierung']:
        from lernen.ki.routbarkeit_dienst import Praediktor
        from lernen.ki.routbarkeit_kalibrieren import kalibrieren, pruefen
        predictor=Praediktor(out,device)
        forbidden=sizes['train']['familien']+sizes['val']['familien']+sizes['test']['familien']
        manifest['kalibrierung']=kalibrieren(predictor,samples['kalibrierung'],verboten=forbidden+sizes['kalibrierpruefung']['familien'])
        manifest['kalibrierung']=pruefen(predictor,samples['kalibrierpruefung'],manifest['kalibrierung'],verboten=forbidden)
        atomar(out/'manifest.json',manifest)
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('aus');p.add_argument('--daten',default=str(Path.home()/'Library/Caches/Autorouter/datensatz'))
    p.add_argument('--epochen',type=int,default=20);p.add_argument('--batch',type=int,default=2);p.add_argument('--ensemble',type=int,default=3);p.add_argument('--geraet',default='mps');p.add_argument('--diagnose',action='store_true')
    a=p.parse_args();trainieren(a.daten,a.aus,a.epochen,a.batch,a.ensemble,a.geraet,diagnose=a.diagnose)
