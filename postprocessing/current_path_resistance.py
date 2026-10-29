from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import shapely
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve
from shapely.geometry import Polygon
from shapely.ops import unary_union

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.flow import pad_geo


def resistance(cu, source, sink, *, h=0.03, t_um=35., rho=1.72e-8):
    if not all(math.isfinite(x) and x>0 for x in (h,t_um,rho)):
        raise ValueError('Invalid grid or material')
    if source.intersects(sink):
        raise ValueError('Source and target contact overlap')
    parts=list(getattr(cu,'geoms',[cu]))
    linked=[p for p in parts if p.intersects(source) and p.intersects(sink)]
    if not linked:
        raise ValueError('Contacts are separated in the exact copper')

    cu=unary_union(linked)
    x0,y0,x1,y1=cu.bounds
    xs=np.arange(x0-h,x1+2*h,h);ys=np.arange(y0-h,y1+2*h,h)
    if len(xs)*len(ys)>3_000_000:
        raise ValueError('Resistance grid > 3 million cells')
    X,Y=np.meshgrid(xs,ys)
    M=shapely.contains_xy(cu.buffer(h*0.3),X,Y)
    A=shapely.contains_xy(source.buffer(-h*0.3),X,Y)&M
    B=shapely.contains_xy(sink.buffer(-h*0.3),X,Y)&M
    if not A.any() or not B.any() or (A&B).any():
        raise ValueError('Contacts in the grid missing or overlapping')
    labels,_=ndimage.label(M)
    reached=np.isin(labels,np.unique(labels[A]))&M
    if not np.all(reached[B]):
        raise ValueError('Contacts separated in the resistance grid')
    M=reached
    free=M&~A&~B
    idx=np.full(M.shape,-1,dtype=int);idx[free]=np.arange(free.sum())
    n=int(free.sum());ii,jj=np.nonzero(free)
    rows=[];cols=[];vals=[];rhs=np.zeros(n);diag=np.zeros(n)
    for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
        ni,nj=ii+di,jj+dj
        ok=(ni>=0)&(ni<M.shape[0])&(nj>=0)&(nj<M.shape[1])
        ni,nj=ni[ok],nj[ok];k=idx[ii[ok],jj[ok]]
        nb=M[ni,nj];np.add.at(diag,k,nb.astype(float))
        f=nb&free[ni,nj]
        rows.append(k[f]);cols.append(idx[ni[f],nj[f]]);vals.append(np.full(f.sum(),-1.))
        np.add.at(rhs,k[nb&A[ni,nj]],1.)
    rows.append(np.arange(n));cols.append(np.arange(n));vals.append(diag)
    matrix=coo_matrix((np.concatenate(vals),(np.concatenate(rows),np.concatenate(cols))),shape=(n,n)).tocsr()
    V=spsolve(matrix,rhs)
    if not np.isfinite(V).all():
        raise ValueError('Resistance solution not finite')
    full=np.zeros(M.shape);full[A]=1.;full[free]=V
    I=0.;ai,aj=np.nonzero(A)
    for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
        ni,nj=ai+di,aj+dj
        ok=(ni>=0)&(ni<M.shape[0])&(nj>=0)&(nj<M.shape[1]);ni,nj=ni[ok],nj[ok]
        f=M[ni,nj]&~A[ni,nj]
        I+=float(np.sum(1.-full[ni[f],nj[f]]))
    if I<=0 or not math.isfinite(I):
        raise ValueError('No source current')
    return dict(R_mOhm=(rho/(t_um*1e-6)*1000)/I, nodes=n,
                kcl_rel_max=float(np.max(np.abs(matrix@V-rhs))) if n else 0.,h_mm=h,t_um=t_um)


def metrics(B,K,copper,*,h=0.03,t_um=35.):
    out={'model':'2D Dirichlet 1 V / 0 V, each target individually, other targets unloaded',
         'rho_ohm_m':1.72e-8,'t_um':t_um,'h_mm':h,'net_list':{},'faults':[]}
    for net,k in sorted(K['net_list'].items()):
        if k.get('category') not in ('current_val','supply'):
            continue
        pp={str(p['ref'])+':'+str(p['num']):p for p in B['pads'] if p['net']==net}
        flow=k.get('flow') or {};source=flow.get('source') or []
        targets=sorted(q for q,I in (flow.get('load') or {}).items() if float(I)>0 and q not in source)
        if not source or not targets:
            out['faults'].append(dict(net=net,reason='Source/load pad missing in the flow contract'));continue
        result={}
        if any(v['net']==net for v in copper.get('vias',[])):
            out['faults'].append(dict(net=net,reason='Net with vias: multi-layer DC solver required'))
            out['net_list'][net]=dict(pads={},medium_mOhm=None,cv=None)
            continue
        for target in targets:
            try:
                if target not in pp or any(s not in pp for s in source):
                    raise ValueError('Unknown flow pad')
                common=set(pp[target]['layers'])
                for s in source:
                    common &= set(pp[s]['layers'])
                common &= set(copper['nets'].get(net) or {})
                if len(common)!=1:
                    raise ValueError('No unique single-layer channel (measure multi-layer/vias separately with the DC net solver)')
                layer=next(iter(common))
                geom=unary_union([Polygon(o,holes) for o,holes in copper['nets'][net][layer]])
                a=unary_union([pad_geo(pp[s])[layer] for s in source]);b=pad_geo(pp[target])[layer]
                result[target]=dict(layer=layer,**resistance(geom,a,b,h=h,t_um=t_um))
            except ValueError as ex:
                out['faults'].append(dict(net=net,pad=target,reason=str(ex)))
        vals=[q['R_mOhm'] for q in result.values()]
        out['net_list'][net]=dict(pads=result,medium_mOhm=float(np.mean(vals)) if vals else None,
                             cv=float(np.std(vals)/np.mean(vals)) if vals else None)
    out['complete']=not out['faults']
    return out


def main():
    b,k,c,out=sys.argv[1:5];kw=dict(a.split('=',1) for a in sys.argv[5:] if '=' in a)
    B,K,C=[json.loads(Path(f).read_text()) for f in (b,k,c)]
    result=metrics(B,K,C,**{a:float(v) for a,v in kw.items() if a in ('h','t_um')})
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('Channel resistances: %d nets, %d errors' % (len(result['net_list']),len(result['faults'])))
    return 0 if result['complete'] else 2


if __name__=='__main__':
    sys.exit(main())
