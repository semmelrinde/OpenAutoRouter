""
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
from gemeinsam.fluss import padgeo


def widerstand(cu, quelle, senke, *, h=0.03, t_um=35., rho=1.72e-8):
    if not all(math.isfinite(x) and x>0 for x in (h,t_um,rho)):
        raise ValueError('Ungueltiges Raster oder Material')
    if quelle.intersects(senke):
        raise ValueError('Quellen- und Zielkontakt ueberlappen')
    parts=list(getattr(cu,'geoms',[cu]))
    linked=[p for p in parts if p.intersects(quelle) and p.intersects(senke)]
    if not linked:
        raise ValueError('Kontakte sind im exakten Kupfer getrennt')

    cu=unary_union(linked)
    x0,y0,x1,y1=cu.bounds
    xs=np.arange(x0-h,x1+2*h,h);ys=np.arange(y0-h,y1+2*h,h)
    if len(xs)*len(ys)>3_000_000:
        raise ValueError('Widerstandsraster > 3 Millionen Zellen')
    X,Y=np.meshgrid(xs,ys)
    M=shapely.contains_xy(cu.buffer(h*0.3),X,Y)
    A=shapely.contains_xy(quelle.buffer(-h*0.3),X,Y)&M
    B=shapely.contains_xy(senke.buffer(-h*0.3),X,Y)&M
    if not A.any() or not B.any() or (A&B).any():
        raise ValueError('Kontakte im Raster fehlen oder ueberlappen')
    labels,_=ndimage.label(M)
    reached=np.isin(labels,np.unique(labels[A]))&M
    if not np.all(reached[B]):
        raise ValueError('Kontakte im Widerstandsraster getrennt')
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
        raise ValueError('Widerstandsloesung nicht endlich')
    full=np.zeros(M.shape);full[A]=1.;full[free]=V
    I=0.;ai,aj=np.nonzero(A)
    for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
        ni,nj=ai+di,aj+dj
        ok=(ni>=0)&(ni<M.shape[0])&(nj>=0)&(nj<M.shape[1]);ni,nj=ni[ok],nj[ok]
        f=M[ni,nj]&~A[ni,nj]
        I+=float(np.sum(1.-full[ni[f],nj[f]]))
    if I<=0 or not math.isfinite(I):
        raise ValueError('Kein Quellenstrom')
    return dict(R_mOhm=(rho/(t_um*1e-6)*1000)/I, knoten=n,
                kcl_rel_max=float(np.max(np.abs(matrix@V-rhs))) if n else 0.,h_mm=h,t_um=t_um)


def kennzahlen(B,K,copper,*,h=0.03,t_um=35.):
    out={'modell':'2D Dirichlet 1 V / 0 V, je Ziel einzeln, andere Ziele unbelastet',
         'rho_ohm_m':1.72e-8,'t_um':t_um,'h_mm':h,'netze':{},'fehler':[]}
    for net,k in sorted(K['netze'].items()):
        if k.get('art') not in ('strom','versorgung'):
            continue
        pp={str(p['ref'])+':'+str(p['num']):p for p in B['pads'] if p['net']==net}
        flow=k.get('fluss') or {};source=flow.get('quelle') or []
        targets=sorted(q for q,I in (flow.get('last') or {}).items() if float(I)>0 and q not in source)
        if not source or not targets:
            out['fehler'].append(dict(net=net,grund='Quelle/Lastpad fehlt im Flussvertrag'));continue
        result={}
        if any(v['net']==net for v in copper.get('vias',[])):
            out['fehler'].append(dict(net=net,grund='Netz mit Vias: Mehrlagen-DC-Loeser erforderlich'))
            out['netze'][net]=dict(pads={},mittel_mOhm=None,cv=None)
            continue
        for target in targets:
            try:
                if target not in pp or any(s not in pp for s in source):
                    raise ValueError('Unbekanntes Flusspad')
                common=set(pp[target]['layers'])
                for s in source:
                    common &= set(pp[s]['layers'])
                common &= set(copper['nets'].get(net) or {})
                if len(common)!=1:
                    raise ValueError('Kein eindeutiger einlagiger Kanal (Mehrlagen/Vias separat mit DC-Netzloeser messen)')
                layer=next(iter(common))
                geom=unary_union([Polygon(o,holes) for o,holes in copper['nets'][net][layer]])
                a=unary_union([padgeo(pp[s])[layer] for s in source]);b=padgeo(pp[target])[layer]
                result[target]=dict(layer=layer,**widerstand(geom,a,b,h=h,t_um=t_um))
            except ValueError as ex:
                out['fehler'].append(dict(net=net,pad=target,grund=str(ex)))
        vals=[q['R_mOhm'] for q in result.values()]
        out['netze'][net]=dict(pads=result,mittel_mOhm=float(np.mean(vals)) if vals else None,
                             cv=float(np.std(vals)/np.mean(vals)) if vals else None)
    out['vollstaendig']=not out['fehler']
    return out


def main():
    b,k,c,out=sys.argv[1:5];kw=dict(a.split('=',1) for a in sys.argv[5:] if '=' in a)
    B,K,C=[json.loads(Path(f).read_text()) for f in (b,k,c)]
    result=kennzahlen(B,K,C,**{a:float(v) for a,v in kw.items() if a in ('h','t_um')})
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('Kanalwiderstaende: %d Netze, %d Fehler' % (len(result['netze']),len(result['fehler'])))
    return 0 if result['vollstaendig'] else 2


if __name__=='__main__':
    sys.exit(main())
