""
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
from lernen.ki.routbarkeit import Routbarkeit, Konfiguration, eingang, geraet, VERSION, PHYSIK
from lernen.ki.routbarkeit_pruefung import atomar, sha, freigabetor

ROOT=Path(__file__).resolve().parents[2]


def sample_von_board(B,K,RG=None,gndlagen=None):
    from lernen.ki.merkmale_raster import merkmale_raster
    data=merkmale_raster(B,K,RG,gndlagen)
    t={k:torch.as_tensor(v.copy()) for k,v in data.items() if v.dtype.kind in 'bifu'}
    names={k:v.tolist() for k,v in data.items() if v.dtype.kind not in 'bifu'}
    names['regeln_json']=json.dumps(RG or {})
    return dict(tensoren=t,namen=names)


class Praediktor:
    def __init__(self,path,device='mps'):
        self.path=Path(path);self.device=geraet(device);self.meta=json.loads((self.path/'manifest.json').read_text());self.models=[]
        if self.meta['version']!=VERSION:raise ValueError('Modellversion')
        for item in self.meta['modelle']:
            p=self.path/item['pfad']
            if sha(p)!=item['sha256']:raise ValueError('Modellhash')
            q=torch.load(p,map_location='cpu',weights_only=True)
            if q['version']!=VERSION:raise ValueError('Checkpointversion')
            m=Routbarkeit(Konfiguration(**q['cfg']));m.load_state_dict(q['state']);m.to(self.device).eval();self.models.append(m)
        if not self.models:raise ValueError('leeres Ensemble')

    @torch.inference_mode()
    def vorhersagen(self,samples,karten=False,kalibrieren=True):
        inputs=[eingang(s,self.device) for s in samples]
        raw=[m(inputs) for m in self.models];preds=[]


        for member in raw:
            cpu_member=[]
            for record in member:
                keys=[k for k in record if not k.startswith('ueberlast_')]
                joined=torch.cat([record[k].reshape(-1) for k in keys]).cpu();off=0;q={}
                for k in keys:
                    n=record[k].numel();q[k]=joined[off:off+n].reshape(record[k].shape);off+=n
                if karten:
                    for k in ('ueberlast_mu','ueberlast_logvar'):q[k]=record[k].cpu()
                cpu_member.append(q)
            preds.append(cpu_member)
        out=[]
        cal=(self.meta.get('kalibrierung') or {}) if kalibrieren else {}
        def klassifikation(p,key):
            temp=(cal.get('temperaturen') or {}).get(key)
            mean=p.mean(0)
            if temp is not None:
                mean=torch.sigmoid(torch.logit(mean.clamp(1e-6,1-1e-6))/temp)
                p=torch.sigmoid(torch.logit(p.clamp(1e-6,1-1e-6))/temp)
            return mean,p.std(0,unbiased=False)
        for i in range(len(samples)):
            p=torch.stack([torch.sigmoid(v[i]['gueltig_logit']) for v in preds]);mean,pstd=klassifikation(p,'gueltig')
            res=dict(p_gueltig=float(mean),
                gueltig_unsicherheit=dict(entropie=float(-(mean.clamp(1e-6,1-1e-6)*mean.clamp(1e-6,1-1e-6).log()+(1-mean).clamp(1e-6,1-1e-6)*(1-mean).clamp(1e-6,1-1e-6).log())),ensemble_std=float(pstd)),
                kalibriert=bool(cal.get('abdeckung_geprueft') and cal.get('klassifikation_geprueft')))
            for name,key in [('note_v3','note'),('physik','physik'),('netz_physik','netz_physik')]:
                mu=torch.stack([v[i][key+'_mu'] for v in preds]);lv=torch.stack([v[i][key+'_logvar'] for v in preds]);mean=mu.mean(0);var=lv.exp().mean(0)+mu.var(0,unbiased=False)

                std=var.sqrt();factor=cal.get('faktor');factor=1.96 if factor is None else factor
                res[name]=dict(wert=torch.expm1(mean.clamp(0,20)).cpu().tolist(),log_mu=mean.cpu().tolist(),log_std=std.cpu().tolist(),
                    intervall=[torch.expm1((mean-factor*std).clamp(0,20)).cpu().tolist(),torch.expm1((mean+factor*std).clamp(0,20)).cpu().tolist()])
            npred=torch.stack([torch.sigmoid(v[i]['netz_logit']) for v in preds]);nmean,nstd=klassifikation(npred,'netz');res['netz_p_verbunden']=nmean.cpu().tolist();res['netz_std']=nstd.cpu().tolist()
            if karten:
                mu=torch.stack([v[i]['ueberlast_mu'] for v in preds]);lv=torch.stack([v[i]['ueberlast_logvar'] for v in preds])
                res['ueberlast']=dict(log_mu=mu.mean(0).cpu().numpy(),log_std=(lv.exp().mean(0)+mu.var(0,unbiased=False)).sqrt().cpu().numpy())
            out.append(res)
        return out


def freigabe_pruefen(version,einsatz='auswahl'):
    if einsatz not in ('auswahl','placer','richter'):raise ValueError('Einsatz')
    p=Path(version);store=json.loads((p/'freigabe.json').read_text());manifest=p/'manifest.json'
    cert=store.get('einsaetze',{}).get(einsatz,{})
    if store.get('version')!=2 or cert.get('einsatz')!=einsatz or cert.get('freigegeben') is not True or cert.get('manifest_sha256')!=sha(manifest):raise ValueError('Modell für Einsatz nicht freigegeben')
    meta=json.loads(manifest.read_text())
    if meta.get('diagnose'):raise ValueError('Diagnosemodell darf nicht wirken')
    if meta.get('version')!=VERSION or not meta.get('modelle'):raise ValueError('Modellversion/Ensemble fehlt')
    report=p/cert['bericht']
    if cert.get('bericht_sha256')!=sha(report):raise ValueError('Prüfbericht geändert')
    r=json.loads(report.read_text())
    if r.get('einsatz')!=einsatz:raise ValueError('Prüfbericht für anderen Einsatz')
    _familienvertrag(meta,r['test_rows'],r['endpaare'],r['kalibrierung'])
    from lernen.ki.routbarkeit_pruefung import vergleichen, nutzentor
    if meta.get('kalibrierung')!=r['kalibrierung']:raise ValueError('Kalibrierung nicht im Manifest')

    st=vergleichen(r['test_rows'])
    if st!=r.get('statistik'):raise ValueError('Gespeicherte Statistik widerspricht Rohmessungen')
    actual=freigabetor(st,nutzentor(r['endpaare'],einsatz,root=p),r['kalibrierung'],r.get('latency_ms'),einsatz=einsatz)
    if not actual['freigegeben']:raise ValueError('Freigabetor gesperrt')
    for item in meta['modelle']:
        if sha(p/item['pfad'])!=item['sha256']:raise ValueError('Modellhash geändert')
    return meta


def uebernehmen(root,version,einsatz='auswahl'):
    ""
    root=Path(root);version=Path(version).resolve();freigabe_pruefen(version,einsatz)
    if version.parent!=root.resolve():raise ValueError('Version muss unter Modellordner liegen')
    root.mkdir(parents=True,exist_ok=True)
    with open(root/'.dienst.lock','a') as f:
        fcntl.flock(f,fcntl.LOCK_EX)
        old=json.loads((root/'aktuell.json').read_text()) if (root/'aktuell.json').exists() else None
        row=dict(version=version.name,manifest_sha256=sha(version/'manifest.json'),vorher=old,utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        atomar(root/'aktuell.json',row)
        with open(root/'verlauf.jsonl','a') as log:log.write(json.dumps(row,sort_keys=True)+'\n');log.flush();os.fsync(log.fileno())
    return row


class Dienst:
    def __init__(self,root=None,routen=4,device='mps',einsatz='auswahl',seed=20261008):
        self.root=Path(root or ROOT/'lernen/modell/routbarkeit');self.routen=routen;self.device=device;self.einsatz=einsatz;self.predictor=None;self.version=None;self.grund='kein freigegebenes Modell'
        import threading
        self.seed=seed;self.rng=np.random.default_rng(seed);self.auswahl_nr=0;self.rng_lock=threading.Lock()
        try:
            cur=json.loads((self.root/'aktuell.json').read_text());p=self.root/cur['version'];freigabe_pruefen(p,einsatz)
            if sha(p/'manifest.json')!=cur['manifest_sha256']:raise ValueError('Versionszeiger')
            self.predictor=Praediktor(p,device);self.version=cur['version']
        except (OSError,ValueError,KeyError,RuntimeError) as e:self.grund=str(e)

    def auswahl(self,pool,art='routbarkeit',routen=None,erkunden=1,samples=None):
        if art!='routbarkeit':raise ValueError('auswahl=routbarkeit')
        if self.predictor is not None and self.einsatz!='auswahl':raise ValueError('Auswahl-Einsatz nicht freigegeben')
        k=min(len(pool),max(1,routen or self.routen));ids=list(range(len(pool)))
        if self.predictor is None:

            from analyse.varianten import schluessel_faust
            ids.sort(key=lambda i:(schluessel_faust(pool[i]['merkmale']),i));pred=None
        else:
            if samples is None:
                samples=[]
                for z in pool:
                    p=Path(z['routbarkeit_eingang']);B=json.loads((p/'board.json').read_text());K=json.loads((p/'klassen.json').read_text())

                    rg=Path(z['routbarkeit_regeln']);RG=json.loads(rg.read_text())
                    samples.append(sample_von_board(B,K,RG,z.get('gndlagen')))
            pred=self.predictor.vorhersagen(samples)
            ids.sort(key=lambda i:(-pred[i]['p_gueltig'],pred[i]['note_v3']['intervall'][1],i))
        chosen=ids[:k];rest=ids[k:];e=min(max(0,erkunden),len(rest))
        with self.rng_lock:
            nr=self.auswahl_nr;self.auswahl_nr+=1
            if e:chosen+=self.rng.choice(rest,e,replace=False).tolist()
        return chosen,dict(art='routbarkeit' if pred is not None else 'faust',angefordert=art,version=self.version,
                          grund=self.grund if pred is None else None,prognosen=pred,gewaehlt=chosen,
                          auswahlwahrscheinlichkeit={str(i):1. if i in ids[:k] else (e/len(rest) if rest else 0.) for i in ids},
                          exploration=dict(seed=self.seed,aufruf=nr,verfahren='fortlaufender PCG64, ohne Zurücklegen'),
                          unabhaengige_endpruefung_erforderlich=True)

    def physik(self,samples):
        if self.predictor is None:return dict(zustand='unbekannt',grund=self.grund)
        if self.einsatz!='richter':raise ValueError('Richter-Einsatz nicht freigegeben')
        return dict(zustand='prognose',kennzahlen=PHYSIK,prognosen=self.predictor.vorhersagen(samples),
                    unabhaengige_endpruefung_erforderlich=True)

    def feinkosten(self,B,K,RG,T,lage,ch,gndlagen=None):
        ""
        if self.predictor is None or self.einsatz!='placer':raise ValueError('Placer-Kostenterm gesperrt: '+self.grund)
        from lernen.auswahl import board_aus_placement
        states=[lage,dict(lage,**ch)];samples=[]
        for state in states:
            parts={}
            for ref,s in state.items():
                parts[ref]=dict(x=s[0],y=s[1],drehung=s[2],side=s[3])
            samples.append(sample_von_board(board_aus_placement(B,dict(parts=parts),exakt=True),K,RG,gndlagen))
        p=self.predictor.vorhersagen(samples)
        def c(v):return (1-v['p_gueltig'])*10+float(v['note_v3']['log_mu'])+float(v['note_v3']['log_std'])
        return c(p[1])-c(p[0])


def neu_trainieren(root,daten,epochen=20,cpu_s=1800,nachts=False,gpu_frei=False):
    ""
    hour=datetime.datetime.now().hour
    if not gpu_frei or (nachts and not (0<=hour<7)):return dict(zustand='ausgelassen',grund='keine freie GPU/Nacht')
    from gemeinsam import plaetze as PL
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    with open(root/'.train.lock','a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return dict(zustand='läuft_bereits')
        v=root/datetime.datetime.now(datetime.timezone.utc).strftime('v_%Y%m%dT%H%M%S_%f')
        run=PL.lauf(['caffeinate','-i','-s',sys.executable,'-m','lernen.ki.routbarkeit_trainieren',str(v),'--daten',str(daten),'--epochen',str(epochen)],cpu_s=cpu_s,log=str(root/'training.log'),cwd=str(ROOT))

        return dict(zustand=run['zustand'],version=str(v),lauf=run,uebernommen=False)


def _familienvertrag(meta,test_rows,endpaare,kalibrierung):
    from lernen.ki.routbarkeit_pruefung import quellfamilie,prueffamilie
    for s in ('val','kalibrierung','kalibrierpruefung','test'):
        for f in meta['daten'].get(s,{}).get('familien',[]):prueffamilie(f,'dauerlauf')
    train={quellfamilie(f) for s in ('train','val') for f in meta['daten'][s]['familien']}
    fit={quellfamilie(f) for f in meta['daten'].get('kalibrierung',{}).get('familien',[])}
    fit|={quellfamilie(f) for f in kalibrierung.get('familien',[])}
    check={quellfamilie(f) for f in kalibrierung.get('prueffamilien',[])}
    check|={quellfamilie(f) for f in meta['daten'].get('kalibrierpruefung',{}).get('familien',[])}

    fit|={quellfamilie(r['familie']) for r in kalibrierung.get('fit_roh',[])}
    check|={quellfamilie(r['familie']) for r in kalibrierung.get('pruefung_roh',[])}
    if train&(fit|check) or fit&check:raise ValueError('Kalibrier-/Train-/Wahl-Familien überlappen')
    for r in test_rows+endpaare:
        if 'familie' not in r:raise ValueError('Endprüf-/Testfamilie fehlt')
        if quellfamilie(r['familie']) in train|fit|check:raise ValueError('Testfamilie im Training/Checkpointwahl/Kalibrierung')


def zertifizieren(version,test_rows,endpaare,kalibrierung,latency_ms=None,einsatz='auswahl'):
    ""
    from lernen.ki.routbarkeit_pruefung import vergleichen,nutzentor
    if einsatz not in ('auswahl','placer','richter'):raise ValueError('Einsatz')
    p=Path(version);meta=json.loads((p/'manifest.json').read_text())
    if meta.get('diagnose'):raise ValueError('Diagnosemodell nicht zertifizierbar')
    _familienvertrag(meta,test_rows,endpaare,kalibrierung)
    if meta.get('kalibrierung')!=kalibrierung:raise ValueError('Kalibrierung nicht im versionierten Modellmanifest')
    report=freigabetor(vergleichen(test_rows),nutzentor(endpaare,einsatz,root=p),kalibrierung,latency_ms,einsatz=einsatz)

    with open(p/'.freigabe.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        name='pruefung_'+einsatz+'.json'
        atomar(p/name,dict(report,test_rows=test_rows,endpaare=endpaare))
        cert=dict(einsatz=einsatz,freigegeben=report['freigegeben'],manifest_sha256=sha(p/'manifest.json'),bericht=name,bericht_sha256=sha(p/name))
        store=json.loads((p/'freigabe.json').read_text()) if (p/'freigabe.json').exists() else {}
        if store.get('version')!=2:store=dict(version=2,einsaetze={})
        store['einsaetze'][einsatz]=cert
        atomar(p/'freigabe.json',store)
    return cert


_TRAINING = {}
_TRAINING_LOCK = __import__('threading').Lock()


def training_anfragen(root,daten,**optionen):
    ""
    import threading
    key=str(Path(root).resolve())
    with _TRAINING_LOCK:
        old=_TRAINING.get(key)
        if old and old.is_alive():return dict(zustand='läuft_bereits')
        if not optionen.get('gpu_frei'):return dict(zustand='ausgelassen',grund='GPU-Fenster nicht deklariert')
        if optionen.get('nachts') and not 0<=datetime.datetime.now().hour<7:return dict(zustand='ausgelassen',grund='kein Nachtfenster')
        def run():
            try:
                result=neu_trainieren(root,daten,**optionen);atomar(Path(root)/'dienst_stand.json',result)
            except Exception as e:atomar(Path(root)/'dienst_stand.json',dict(zustand='fehler',grund=str(e)))
        thread=threading.Thread(target=run,name='routbarkeit-training',daemon=True);_TRAINING[key]=thread;thread.start()
        return dict(zustand='angefragt',uebernahme='nur über zertifizieren/uebernehmen')
