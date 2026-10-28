""
import math

import numpy as np


def je_last(B, R, K, res=.1):
    from gemeinsam.zonen import boardgeometrie, vereinheitlichen, viageometrie
    from codex.zonen_ir.solver_fast import Grid, geometry, layers_of
    from analyse.metriken import aufbau
    from gemeinsam.bogen import punkte
    B, R = boardgeometrie(B,R), viageometrie(vereinheitlichen(R))
    rr = dict(R, tracks=[])
    for t in R['tracks']:
        if 'items' not in t:
            rr['tracks'].append(t); continue
        for it in t['items']:
            ps,w = (it[1:3],it[3]) if it[0]=='L' else (punkte(*it[1:4]),it[4])
            rr['tracks'].append(dict(net=t['net'],layer=t['layer'],pts=ps,w=[w]*(len(ps)-1)))
    layers = layers_of(B,R,K)
    stack = B.get('stackup')
    if not stack:
        vg = (K.get('vorgaben') or {}).get('aufbau') or {}
        build = aufbau(int(B['lagen']), R.get('gndlagen') or [], diel=vg.get('dielektrika'))
        stack = {l:dict(copper_um=q['t']*1000,z_mm=q['z']) for l,q in build['lagen'].items()}
    diameter,drill = map(float,(R.get('optionen') or {}).get('via','0.45/0.20').split('/'))
    out = dict(raster_mm=res, J_je_last_A_mm2={}, fehler={}, lastlos=[], modell='Einzelast/LU/inklusive Viahuelsen')
    for n,v in sorted(K['netze'].items()):
        if v.get('art') not in ('strom','versorgung','sw'):
            continue
        try:
            f = v.get('fluss') or {}; loads=dict(f.get('last') or {})
            if not f or 'last' not in f:
                raise ValueError('Lastvertrag fehlt; keine behauptete Lastlosigkeit')
            if any(not math.isfinite(x) or x<0 for x in loads.values()):
                raise ValueError('Ungueltige Last')
            total=sum(loads.values())
            if total==0:
                out['lastlos'].append(n); continue
            if f.get('einheit') in (None,'relativ'):
                from lernen.zonenplaner import strom
                amps, origin = strom(v)
                if amps is None or not v.get('strom_A_quelle'):
                    raise ValueError('Ampere/Herkunft unbekannt: '+origin)
                loads={p:x*amps/total for p,x in loads.items()}
            elif f.get('einheit')!='A':
                raise ValueError('Unbekannte Lasteinheit')
            if not f.get('quelle'):
                raise ValueError('Keine Quelle')
            polys,pads,barrels,_,_ = geometry(B,rr,n,layers,diameter)
            polys = {l:g.simplify(res/100,preserve_topology=True) for l,g in polys.items()}
            grid=Grid(polys,pads,barrels,layers,res,stack,via_drill=drill)
            _,state=grid.solve(f['quelle'],loads)
            bypad={}; free=state['free']; reached=state['reached']
            for p,x in sorted(loads.items()):
                if x<=0: continue
                rhs=np.zeros(grid.n); rhs[grid.pad_nodes[p]]=x/len(grid.pad_nodes[p])
                u=np.zeros(grid.n); u[free]=state['lu'].solve(rhs[free])
                density=np.abs(grid.g*(u[grid.b]-u[grid.a]))/grid.area
                density[~(reached[grid.a]&reached[grid.b])]=np.nan
                bypad[p]=float(np.nanmax(density))
            out['J_je_last_A_mm2'][n]=bypad
        except Exception as exc:
            out['fehler'][n]=str(exc)
    return out


def tor(a,b):
    bad=[]; missing=[]
    if a.get('raster_mm')!=b.get('raster_mm'): missing.append('Raster verschieden')
    missing += sorted(set(a.get('fehler',{}))|set(b.get('fehler',{})))
    av,bv=a.get('J_je_last_A_mm2',{}),b.get('J_je_last_A_mm2',{})
    for n in sorted(set(av)|set(bv)):
        for p in sorted(set(av.get(n,{}))|set(bv.get(n,{}))):
            x,y=av.get(n,{}).get(p),bv.get(n,{}).get(p)
            if x is None or y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(n+':'+p)
            elif y>x+max(1e-6,abs(x)*1e-6): bad.append(n+':'+p)
    return dict(bestanden=not missing and not bad, fehlt=missing, schlechter=bad, vorher=a,nachher=b)
