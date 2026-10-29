import math

import numpy as np


def per_load(B, R, K, res=.1):
    from common.zones import board_geometry, unify, via_geometry
    from codex.zones_ir.solver_fast import Grid, geometry, layers_of
    from analysis.measures import structure
    from common.arc import points
    B, R = board_geometry(B,R), via_geometry(unify(R))
    rr = dict(R, tracks=[])
    for t in R['tracks']:
        if 'items' not in t:
            rr['tracks'].append(t); continue
        for it in t['items']:
            ps,w = (it[1:3],it[3]) if it[0]=='L' else (points(*it[1:4]),it[4])
            rr['tracks'].append(dict(net=t['net'],layer=t['layer'],pts=ps,w=[w]*(len(ps)-1)))
    layers = layers_of(B,R,K)
    stack = B.get('stackup')
    if not stack:
        cons = (K.get('specs') or {}).get('structure') or {}
        build = structure(int(B['cu_layers']), R.get('gnd_layers') or [], diel=cons.get('dielectrics'))
        stack = {l:dict(copper_um=q['t']*1000,z_mm=q['z']) for l,q in build['cu_layers'].items()}
    diameter,drill = map(float,(R.get('options') or {}).get('via','0.45/0.20').split('/'))
    out = dict(raster_mm=res, J_per_load_A_mm2={}, faults={}, loadless=[], model='Single branch/LU/including via sleeves')
    for n,v in sorted(K['net_list'].items()):
        if v.get('category') not in ('current_val','supply','sw'):
            continue
        try:
            f = v.get('flow') or {}; loads=dict(f.get('load') or {})
            if not f or 'load' not in f:
                raise ValueError('Load contract missing; no claimed absence of load')
            if any(not math.isfinite(x) or x<0 for x in loads.values()):
                raise ValueError('Invalid load')
            total=sum(loads.values())
            if total==0:
                out['loadless'].append(n); continue
            if f.get('unit') in (None,'relative'):
                from learning.zone_planner import current_val
                amps, origin = current_val(v)
                if amps is None or not v.get('current_A_source'):
                    raise ValueError('Ampere/origin unknown: '+origin)
                loads={p:x*amps/total for p,x in loads.items()}
            elif f.get('unit')!='A':
                raise ValueError('Unknown load unit')
            if not f.get('source'):
                raise ValueError('No source')
            polys,pads,barrels,_,_ = geometry(B,rr,n,layers,diameter)
            polys = {l:g.simplify(res/100,preserve_topology=True) for l,g in polys.items()}
            grid=Grid(polys,pads,barrels,layers,res,stack,via_drill=drill)
            _,state=grid.solve(f['source'],loads)
            bypad={}; free=state['free']; reached=state['reached']
            for p,x in sorted(loads.items()):
                if x<=0: continue
                rhs=np.zeros(grid.n); rhs[grid.pad_nodes[p]]=x/len(grid.pad_nodes[p])
                u=np.zeros(grid.n); u[free]=state['lu'].solve(rhs[free])
                density=np.abs(grid.g*(u[grid.b]-u[grid.a]))/grid.area
                density[~(reached[grid.a]&reached[grid.b])]=np.nan
                bypad[p]=float(np.nanmax(density))
            out['J_per_load_A_mm2'][n]=bypad
        except Exception as exc:
            out['faults'][n]=str(exc)
    return out


def gate_check(a,b):
    bad=[]; missing=[]
    if a.get('raster_mm')!=b.get('raster_mm'): missing.append('Grid differs')
    missing += sorted(set(a.get('faults',{}))|set(b.get('faults',{})))
    av,bv=a.get('J_per_load_A_mm2',{}),b.get('J_per_load_A_mm2',{})
    for n in sorted(set(av)|set(bv)):
        for p in sorted(set(av.get(n,{}))|set(bv.get(n,{}))):
            x,y=av.get(n,{}).get(p),bv.get(n,{}).get(p)
            if x is None or y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(n+':'+p)
            elif y>x+max(1e-6,abs(x)*1e-6): bad.append(n+':'+p)
    return dict(passed=not missing and not bad, missing=missing, worse=bad, before=a,after=b)
