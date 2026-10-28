""
import copy
import math
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.special import expit
from lernen.ki.routbarkeit import PHYSIK, ziel_physik, ziel_physik_global
from lernen.ki.routbarkeit_pruefung import quellfamilie, prueffamilie, _finite

REGRESSIONSKOEPFE=('note',)+PHYSIK+tuple('netz_'+k for k in PHYSIK)+('ueberlast',)
KLASSIFIKATIONSKOEPFE=('gueltig','netz')
MIN_PRUEFFAMILIEN=20
MAX_ECE=.1


def temperatur(p,t):
    if not _finite(t) or t<=0:raise ValueError('Klassifikationstemperatur')
    p=np.clip(p,1e-6,1-1e-6)
    return expit((np.log(p)-np.log1p(-p))/t)


def rohmessungen(predictor,samples,verboten=()):
    forbidden={quellfamilie(f) for f in verboten};rows=[]
    preds=predictor.vorhersagen(samples,karten=True,kalibrieren=False)
    if len(preds)!=len(samples):raise ValueError('fehlende Kalibrierprognose')
    for sample,pred in zip(samples,preds):
        meta=sample['meta'];family=prueffamilie(meta['designfamilie'],meta.get('herkunft'))
        if meta['split']!='val' or family in forbidden:raise ValueError('Kalibrierfamilie wurde zum Fit/Wählen/Test genutzt')
        t=sample['tensoren'];reg={};classes={}
        def regression(head,y,mask,mu,std):
            y=np.asarray(y,dtype=float);mask=np.asarray(mask,dtype=bool)
            mu=np.asarray(mu,dtype=float);std=np.asarray(std,dtype=float)
            if y.shape!=mask.shape or y.shape!=mu.shape or y.shape!=std.shape:raise ValueError('Kalibrierkopf-Form: '+head)
            if not mask.any():return
            y=y[mask];mu=mu[mask];std=std[mask]
            if not (np.isfinite(y).all() and (y>=0).all() and np.isfinite(mu).all() and np.isfinite(std).all() and (std>0).all()):raise ValueError('Kalibrierkopf-Werte: '+head)
            yl=np.log1p(y);ix=int(np.argmax(np.abs(yl-mu)/std))
            reg[head]=dict(ziel_log=float(yl[ix]),mu_log=float(mu[ix]),std_log=float(std[ix]),anzahl=len(y))
        def classification(head,y,mask,p):
            y=np.asarray(y,dtype=float);mask=np.asarray(mask,dtype=bool);p=np.asarray(p,dtype=float)
            if y.shape!=mask.shape or y.shape!=p.shape:raise ValueError('Klassifikationskopf-Form: '+head)
            if mask.any():classes[head]=np.stack((y[mask],p[mask]),1).tolist()
        gy=t['global_ziel'].cpu().numpy();gm=t['global_maske'].cpu().numpy()
        if gm[1]:regression('note',np.array([gy[1]]),np.array([True]),np.array([pred['note_v3']['log_mu']]),np.array([pred['note_v3']['log_std']]))
        if gm[0]:classification('gueltig',gy[:1],gm[:1],np.array([pred['p_gueltig']]))
        if 'physik_x' in t:
            y,mask=ziel_physik(t);g,mg=ziel_physik_global(t)
            for j,k in enumerate(PHYSIK):
                if mg[j]:regression(k,np.array([float(g[j])]),np.array([True]),np.array([pred['physik']['log_mu'][j]]),np.array([pred['physik']['log_std'][j]]))
                if mask[:,j].any():regression('netz_'+k,y[:,j].cpu().numpy(),mask[:,j].cpu().numpy(),np.asarray(pred['netz_physik']['log_mu'])[:,j],np.asarray(pred['netz_physik']['log_std'])[:,j])
        if 'netz_verbunden' in t:classification('netz',t['netz_verbunden'].cpu().numpy(),t['netz_verbunden_maske'].cpu().numpy(),np.asarray(pred['netz_p_verbunden']))
        if 'ziel_raster' in t:regression('ueberlast',t['ziel_raster'][:,0].cpu().numpy(),t['ziel_raster_maske'][:,0].cpu().numpy(),pred['ueberlast']['log_mu'],pred['ueberlast']['log_std'])
        rows.append(dict(id=str(meta['id']),familie=family,herkunft=meta['herkunft'],regression=reg,klassifikation=classes))
    return rows


def _gruppen(rows):
    groups={};ids=set()
    for r in rows:
        if not isinstance(r.get('id'),str) or not r['id'] or r['id'] in ids:raise ValueError('Kalibrierproben-ID fehlt/doppelt')
        ids.add(r['id']);family=prueffamilie(r['familie'],r.get('herkunft'));group=groups.setdefault(family,dict(scores={},klassen={}))
        for head,v in r['regression'].items():
            if head not in REGRESSIONSKOEPFE or not all(_finite(v.get(k)) for k in ('ziel_log','mu_log','std_log')) or v['ziel_log']<0 or v['std_log']<=0 or not isinstance(v.get('anzahl'),int) or v['anzahl']<1:
                raise ValueError('ungültige Regressionsrohmessung')
            score=abs(v['ziel_log']-v['mu_log'])/v['std_log'];group['scores'][head]=max(group['scores'].get(head,0.),score)
        for head,values in r['klassifikation'].items():
            if head not in KLASSIFIKATIONSKOEPFE or not values:raise ValueError('Klassifikationsrohmessung fehlt')
            for y,p in values:
                if y not in (0,1) or not _finite(p) or not 0<=p<=1:raise ValueError('Klassifikationsrohmessung ungültig')
            group['klassen'].setdefault(head,[]).extend(values)
    return groups


def _klassen(groups,head):
    ys=[];ps=[];weights=[]
    for g in groups.values():
        v=g['klassen'].get(head,[])
        for y,p in v:ys.append(y);ps.append(p);weights.append(1/len(v)/len(groups))
    return np.array(ys),np.array(ps),np.array(weights)


def _fit(rows,alpha):
    if not _finite(alpha) or not 0<alpha<1:raise ValueError('alpha')
    groups=_gruppen(rows);heads=sorted({h for g in groups.values() for h in g['scores']})
    scores=[max(g['scores'].values()) for g in groups.values() if g['scores']]
    n=len(groups);rank=math.ceil((n+1)*(1-alpha));factor=float(np.sort(scores)[rank-1]) if len(scores)==n and 1<=rank<=n else None
    temps={}
    for head in KLASSIFIKATIONSKOEPFE:
        y,p,w=_klassen(groups,head)
        if len(set(y))<2:temps[head]=None;continue
        def loss(logt):
            q=np.clip(temperatur(p,float(np.exp(logt))),1e-12,1-1e-12)
            return -float(np.sum(w*(y*np.log(q)+(1-y)*np.log1p(-q))))
        result=minimize_scalar(loss,bounds=(-3,3),method='bounded')
        temps[head]=float(np.exp(result.x)) if result.success else None
    return dict(faktor=factor,alpha=alpha,familien=sorted(groups),familien_n=n,koepfe=heads,temperaturen=temps)


def kalibrieren(predictor,samples,verboten=(),alpha=.05):
    rows=rohmessungen(predictor,samples,verboten)
    return dict(_fit(rows,alpha),fit_roh=rows,pruefung_roh=[],unabhaengige_familien=bool(rows),abdeckung_geprueft=False,
                klassifikation_geprueft=False,grund='unabhängige Prüfung mit pruefen() erforderlich',einheit='maximaler Fehler je Kopf und Familie')


def _abnahme(fit,rows):
    groups=_gruppen(rows);factor=fit['faktor'];reasons=[];cover={};ece={}
    if len(groups)<MIN_PRUEFFAMILIEN:reasons.append('zu wenige unabhängige Kalibrierprüffamilien')
    for head in REGRESSIONSKOEPFE:
        scores=[g['scores'].get(head) for g in groups.values()]
        if not scores or any(v is None for v in scores) or factor is None:
            reasons.append('Abdeckungskopf fehlt: '+head);continue
        cover[head]=float(np.mean(np.array(scores)<=factor+1e-9))
        if cover[head]+1e-9<1-fit['alpha']:reasons.append('Abdeckung ungenügend: '+head)
    if len(cover)==len(REGRESSIONSKOEPFE):
        cover['gemeinsam']=float(np.mean([max(g['scores'].values())<=factor+1e-9 for g in groups.values()]))
        if cover['gemeinsam']+1e-9<1-fit['alpha']:reasons.append('Abdeckung gemeinsam ungenügend')
    for head in KLASSIFIKATIONSKOEPFE:
        temp=fit['temperaturen'].get(head)
        if temp is None or any(not g['klassen'].get(head) for g in groups.values()):reasons.append('Klassifikationskalibrierung fehlt: '+head);continue
        y,p,w=_klassen(groups,head)
        if len(set(y))<2:reasons.append('Klassifikationsklassen fehlen: '+head);continue
        q=temperatur(p,temp);bins=np.minimum((q*10).astype(int),9);error=0.
        for b in range(10):
            m=bins==b
            if m.any():error+=abs(float(np.sum(w[m]*(q[m]-y[m]))))
        ece[head]=error
        if error>MAX_ECE:reasons.append('Klassifikation ungenügend: '+head)
    return dict(prueffamilien=sorted(groups),abdeckung=cover,ece=ece,gruende=reasons)


def pruefen(predictor,samples,kalibrierung,verboten=()):
    ""
    out=copy.deepcopy(kalibrierung);fit=_fit(out['fit_roh'],out['alpha'])
    rows=rohmessungen(predictor,samples,list(verboten)+fit['familien']);check=_abnahme(fit,rows)
    out.update(fit);out.update(check,pruefung_roh=rows,abdeckung_geprueft=not any('Abdeckung' in r or 'familien' in r for r in check['gruende']),
               klassifikation_geprueft=not any('Klassifik' in r or 'familien' in r for r in check['gruende']))
    out['grund']='geprüft' if not check['gruende'] else '; '.join(check['gruende'])
    return out


def nachweis_pruefen(c):
    ""
    try:
        fit=_fit(c['fit_roh'],c['alpha']);groups=_gruppen(c['fit_roh']);check=_abnahme(fit,c['pruefung_roh']);reasons=list(check['gruende'])
        if fit['faktor'] is None or not _finite(fit['faktor']) or fit['faktor']<0:reasons.append('Kalibrierfaktor fehlt')
        if fit['alpha']>.05:reasons.append('Abdeckungsziel unter 95 Prozent')
        if any(set(g['scores'])!=set(REGRESSIONSKOEPFE) for g in groups.values()) or fit['koepfe']!=sorted(REGRESSIONSKOEPFE):reasons.append('Fitköpfe unvollständig')
        if any(set(g['klassen'])!=set(KLASSIFIKATIONSKOEPFE) for g in groups.values()):reasons.append('Fitklassifikationsköpfe unvollständig')
        if set(fit['familien'])&set(check['prueffamilien']):reasons.append('Kalibrierung/Abdeckung teilen Familien')
        for k in fit:
            if c.get(k)!=fit[k]:reasons.append('Kalibriernachweis geändert: '+k)
        for k in ('prueffamilien','abdeckung','ece'):
            if c.get(k)!=check[k]:reasons.append('Kalibrierprüfnachweis geändert: '+k)
        if any(c.get(k) is not True for k in ('unabhaengige_familien','abdeckung_geprueft','klassifikation_geprueft')):reasons.append('Kalibrierprüfung unvollständig')
        return reasons
    except (KeyError,TypeError,ValueError,IndexError,OverflowError):return ['Unsicherheit ohne vollständige unabhängige Rohmessungen']
