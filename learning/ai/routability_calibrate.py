import copy
import math
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.special import expit
from learning.ai.routability import PHYSICS, goal_physics, goal_physics_global
from learning.ai.routability_checking import source_family, check_family, _finite

REGRESSION_HEADS=('rating',)+PHYSICS+tuple('net_'+k for k in PHYSICS)+('overload',)
CLASSIFICATION_HEADS=('is_valid','net_name')
MIN_CHECK_FAMILIES=20
MAX_ECE=.1


def temperature(p,t):
    if not _finite(t) or t<=0:raise ValueError('ClassificationTemperature')
    p=np.clip(p,1e-6,1-1e-6)
    return expit((np.log(p)-np.log1p(-p))/t)


def raw_measurements(predictor,samples,banned=()):
    forbidden={source_family(f) for f in banned};rows=[]
    preds=predictor.predictions(samples,maps=True,calibrate=False)
    if len(preds)!=len(samples):raise ValueError('missing calibration forecast')
    for sample,pred in zip(samples,preds):
        meta=sample['meta'];family=check_family(meta['design_family'],meta.get('provenance'))
        if meta['split']!='val' or family in forbidden:raise ValueError('Calibration family was used for fit/selection/test')
        t=sample['tensors'];reg={};classes={}
        def regression(head,y,mask,mu,std):
            y=np.asarray(y,dtype=float);mask=np.asarray(mask,dtype=bool)
            mu=np.asarray(mu,dtype=float);std=np.asarray(std,dtype=float)
            if y.shape!=mask.shape or y.shape!=mu.shape or y.shape!=std.shape:raise ValueError('Calibration head shape: '+head)
            if not mask.any():return
            y=y[mask];mu=mu[mask];std=std[mask]
            if not (np.isfinite(y).all() and (y>=0).all() and np.isfinite(mu).all() and np.isfinite(std).all() and (std>0).all()):raise ValueError('Calibration head values: '+head)
            yl=np.log1p(y);ix=int(np.argmax(np.abs(yl-mu)/std))
            reg[head]=dict(goal_log=float(yl[ix]),mu_log=float(mu[ix]),std_log=float(std[ix]),count=len(y))
        def classification(head,y,mask,p):
            y=np.asarray(y,dtype=float);mask=np.asarray(mask,dtype=bool);p=np.asarray(p,dtype=float)
            if y.shape!=mask.shape or y.shape!=p.shape:raise ValueError('Classification head shape: '+head)
            if mask.any():classes[head]=np.stack((y[mask],p[mask]),1).tolist()
        gy=t['global_goal'].cpu().numpy();gm=t['global_mask'].cpu().numpy()
        if gm[1]:regression('rating',np.array([gy[1]]),np.array([True]),np.array([pred['score_v3']['log_mu']]),np.array([pred['score_v3']['log_std']]))
        if gm[0]:classification('is_valid',gy[:1],gm[:1],np.array([pred['p_valid']]))
        if 'physics_x' in t:
            y,mask=goal_physics(t);g,mg=goal_physics_global(t)
            for j,k in enumerate(PHYSICS):
                if mg[j]:regression(k,np.array([float(g[j])]),np.array([True]),np.array([pred['physics']['log_mu'][j]]),np.array([pred['physics']['log_std'][j]]))
                if mask[:,j].any():regression('net_'+k,y[:,j].cpu().numpy(),mask[:,j].cpu().numpy(),np.asarray(pred['net_physics']['log_mu'])[:,j],np.asarray(pred['net_physics']['log_std'])[:,j])
        if 'net_joined' in t:classification('net_name',t['net_joined'].cpu().numpy(),t['net_joined_mask'].cpu().numpy(),np.asarray(pred['net_p_joined']))
        if 'goal_raster' in t:regression('overload',t['goal_raster'][:,0].cpu().numpy(),t['goal_raster_mask'][:,0].cpu().numpy(),pred['overload']['log_mu'],pred['overload']['log_std'])
        rows.append(dict(id=str(meta['id']),family=family,provenance=meta['provenance'],regression=reg,classification=classes))
    return rows


def _groups(rows):
    groups={};ids=set()
    for r in rows:
        if not isinstance(r.get('id'),str) or not r['id'] or r['id'] in ids:raise ValueError('Calibration sample ID missing/duplicate')
        ids.add(r['id']);family=check_family(r['family'],r.get('provenance'));group=groups.setdefault(family,dict(scores={},netclasses={}))
        for head,v in r['regression'].items():
            if head not in REGRESSION_HEADS or not all(_finite(v.get(k)) for k in ('goal_log','mu_log','std_log')) or v['goal_log']<0 or v['std_log']<=0 or not isinstance(v.get('count'),int) or v['count']<1:
                raise ValueError('invalid regression raw measurement')
            score=abs(v['goal_log']-v['mu_log'])/v['std_log'];group['scores'][head]=max(group['scores'].get(head,0.),score)
        for head,values in r['classification'].items():
            if head not in CLASSIFICATION_HEADS or not values:raise ValueError('Classification raw measurement missing')
            for y,p in values:
                if y not in (0,1) or not _finite(p) or not 0<=p<=1:raise ValueError('Classification raw measurement invalid')
            group['netclasses'].setdefault(head,[]).extend(values)
    return groups


def _netclasses(groups,head):
    ys=[];ps=[];weights=[]
    for g in groups.values():
        v=g['netclasses'].get(head,[])
        for y,p in v:ys.append(y);ps.append(p);weights.append(1/len(v)/len(groups))
    return np.array(ys),np.array(ps),np.array(weights)


def _fit(rows,alpha):
    if not _finite(alpha) or not 0<alpha<1:raise ValueError('alpha')
    groups=_groups(rows);heads=sorted({h for g in groups.values() for h in g['scores']})
    scores=[max(g['scores'].values()) for g in groups.values() if g['scores']]
    n=len(groups);rank=math.ceil((n+1)*(1-alpha));factor=float(np.sort(scores)[rank-1]) if len(scores)==n and 1<=rank<=n else None
    temps={}
    for head in CLASSIFICATION_HEADS:
        y,p,w=_netclasses(groups,head)
        if len(set(y))<2:temps[head]=None;continue
        def loss(logs):
            q=np.clip(temperature(p,float(np.exp(logs))),1e-12,1-1e-12)
            return -float(np.sum(w*(y*np.log(q)+(1-y)*np.log1p(-q))))
        result=minimize_scalar(loss,bounds=(-3,3),method='bounded')
        temps[head]=float(np.exp(result.x)) if result.success else None
    return dict(factor=factor,alpha=alpha,families=sorted(groups),families_n=n,heads=heads,temperatures=temps)


def calibrate(predictor,samples,banned=(),alpha=.05):
    rows=raw_measurements(predictor,samples,banned)
    return dict(_fit(rows,alpha),fit_raw=rows,checking_raw=[],independent_families=bool(rows),coverage_checked=False,
                classification_checked=False,reason='independent check with verify() required',unit='maximum error per head and family')


def _acceptance(fit,rows):
    groups=_groups(rows);factor=fit['factor'];reasons=[];cover={};ece={}
    if len(groups)<MIN_CHECK_FAMILIES:reasons.append('too few independent calibration check families')
    for head in REGRESSION_HEADS:
        scores=[g['scores'].get(head) for g in groups.values()]
        if not scores or any(v is None for v in scores) or factor is None:
            reasons.append('Coverage head missing: '+head);continue
        cover[head]=float(np.mean(np.array(scores)<=factor+1e-9))
        if cover[head]+1e-9<1-fit['alpha']:reasons.append('Coverage insufficient: '+head)
    if len(cover)==len(REGRESSION_HEADS):
        cover['common']=float(np.mean([max(g['scores'].values())<=factor+1e-9 for g in groups.values()]))
        if cover['common']+1e-9<1-fit['alpha']:reasons.append('Coverage insufficient jointly')
    for head in CLASSIFICATION_HEADS:
        temp=fit['temperatures'].get(head)
        if temp is None or any(not g['netclasses'].get(head) for g in groups.values()):reasons.append('Classification calibration missing: '+head);continue
        y,p,w=_netclasses(groups,head)
        if len(set(y))<2:reasons.append('Classification classes missing: '+head);continue
        q=temperature(p,temp);bins=np.minimum((q*10).astype(int),9);error=0.
        for b in range(10):
            m=bins==b
            if m.any():error+=abs(float(np.sum(w[m]*(q[m]-y[m]))))
        ece[head]=error
        if error>MAX_ECE:reasons.append('Classification insufficient: '+head)
    return dict(check_families=sorted(groups),coverage=cover,ece=ece,reasons=reasons)


def verify(predictor,samples,calibration,banned=()):
    out=copy.deepcopy(calibration);fit=_fit(out['fit_raw'],out['alpha'])
    rows=raw_measurements(predictor,samples,list(banned)+fit['families']);check=_acceptance(fit,rows)
    out.update(fit);out.update(check,checking_raw=rows,coverage_checked=not any('coverage' in r.lower() or 'families' in r for r in check['reasons']),
               classification_checked=not any('Classif' in r or 'families' in r for r in check['reasons']))
    out['reason']='checked' if not check['reasons'] else '; '.join(check['reasons'])
    return out


def proof_verify(c):
    try:
        fit=_fit(c['fit_raw'],c['alpha']);groups=_groups(c['fit_raw']);check=_acceptance(fit,c['checking_raw']);reasons=list(check['reasons'])
        if fit['factor'] is None or not _finite(fit['factor']) or fit['factor']<0:reasons.append('Calibration factor missing')
        if fit['alpha']>.05:reasons.append('Coverage target below 95 percent')
        if any(set(g['scores'])!=set(REGRESSION_HEADS) for g in groups.values()) or fit['heads']!=sorted(REGRESSION_HEADS):reasons.append('Fit heads incomplete')
        if any(set(g['netclasses'])!=set(CLASSIFICATION_HEADS) for g in groups.values()):reasons.append('Fit classification heads incomplete')
        if set(fit['families'])&set(check['check_families']):reasons.append('Calibration/coverage share families')
        for k in fit:
            if c.get(k)!=fit[k]:reasons.append('Calibration evidence changed: '+k)
        for k in ('check_families','coverage','ece'):
            if c.get(k)!=check[k]:reasons.append('Calibration check evidence changed: '+k)
        if any(c.get(k) is not True for k in ('independent_families','coverage_checked','classification_checked')):reasons.append('Calibration check incomplete')
        return reasons
    except (KeyError,TypeError,ValueError,IndexError,OverflowError):return ['Uncertainty without complete independent raw measurements']
