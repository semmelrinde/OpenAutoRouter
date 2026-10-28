""
from collections import defaultdict
import math
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union, nearest_points

from codex.zonen_ir.fuellung import fuelle_zonen, polygone
from codex.zonen_ir.solver_fast import Grid, geometry, layers_of




def netz_loesen(B, R, K, net, zs, res):
    layers = layers_of(B, R, K)
    vd, drill = map(float, (R.get('optionen') or {}).get('via', '0.45/0.20').split('/'))
    g, pads, barrels, _, _ = geometry(B, dict(R, zones=zs), net, layers, vd)
    f = K['netze'][net]['fluss']
    loads = {k: float(v) for k, v in f['last'].items()}
    total = sum(loads.values())
    if total <= 0 or not math.isfinite(total):
        raise ValueError('Kein positiver DC-Lastfall')
    if f.get('einheit') != 'A':
        loads = {k: v/total for k, v in loads.items()}
    grid = Grid(g, pads, barrels, layers, res, B.get('stackup'), via_drill=drill)
    result, state = grid.solve(f['quelle'], loads)
    return result, grid, state

def dichte_flaeche(grid, state, threshold, margin):
    ""
    density = np.zeros(grid.n)
    planar = np.array([k == 'sheet' for k in grid.kinds])

    planar |= np.array([k == 'copper' for k in grid.kinds])
    js = np.nan_to_num(state['density'][planar])
    np.maximum.at(density, grid.a[planar], js)
    np.maximum.at(density, grid.b[planar], js)
    out = {}
    for layer, ix in grid.idx.items():
        mask = ix >= 0
        mask[mask] &= density[ix[mask]] > threshold
        rects = []

        for j in np.flatnonzero(mask.any(axis=1)):
            edges = np.diff(np.r_[False, mask[j], False].astype(int))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                rects.append(box(grid.x0+a*grid.res, grid.y0+j*grid.res,
                                 grid.x0+b*grid.res, grid.y0+(j+1)*grid.res))
        g = unary_union(rects)

        out[layer] = g.buffer(margin, join_style='mitre').simplify(grid.res*.22)
    return out, density


def gruppen(zones):
    d = defaultdict(list)
    for z in zones:
        d[z['net'], z['layer']].append(Polygon(z['pts'], z.get('holes', [])).buffer(0))
    return {k: unary_union(v) for k,v in d.items()}


def als_zonen(groups, prios):
    result=[]
    for (net, layer),g in sorted(groups.items()):
        parts=polygone(g)
        if any(a.distance(b)<1e-6 for i,a in enumerate(parts) for b in parts[i+1:]):



            g=g.union(g.buffer(.002,quad_segs=4).buffer(-.002,quad_segs=4))
        for p in polygone(g):
            if p.area<.05: continue
            result.append(dict(net=net,layer=layer,prio=prios[net],full=True,polygon_dc=True,
                               k15=True,pts=list(map(list,p.exterior.coords))[:-1],
                               holes=[list(map(list,r.coords))[:-1] for r in p.interiors]))
    return result
def tropfenmaske(x,y,dx,dy,r0,r1,length):
    ""
    s=np.linspace(0,length,25)
    q=s/length
    radii=r0+(r1-r0)*(3*q*q-2*q*q*q)
    a=[(x+u*dx-r*dy,y+u*dy+r*dx) for u,r in zip(s,radii)]
    b=[(x+u*dx+r*dy,y+u*dy-r*dx) for u,r in zip(s[::-1],radii[::-1])]
    return Polygon(a+b).union(Point(x,y).buffer(r0,quad_segs=16))


def terminale(R,net,layer,via_d,B=None):
    ""
    out=[]
    tracks=[t for t in R['tracks'] if t['net']==net and t['layer']==layer]
    for v in R['vias']:
        if v['net']!=net:continue
        hit=[]
        for t in tracks:
            pts=t['pts']
            if len(pts)<2:continue
            if math.dist(pts[-1],(v['x'],v['y']))<.06:pts=pts[::-1]
            if math.dist(pts[0],(v['x'],v['y']))>=.06:continue
            for q in pts[1:]:
                if math.dist(q,pts[0])>.02:
                    dx,dy=q[0]-pts[0][0],q[1]-pts[0][1]
                    dist=math.hypot(dx,dy);dx/=dist;dy/=dist
                    break
            else:continue
            length=0
            for a,b in zip(pts,pts[1:]):
                vx,vy=b[0]-a[0],b[1]-a[1]
                if abs(vx*dy-vy*dx)>.01 or vx*dx+vy*dy<0:break
                length+=math.hypot(vx,vy)
            hit.append((v['x'],v['y'],dx,dy,length,v.get('diameter',v.get('d',via_d))/2+.12,'via'))
        if len(hit)==1:out+=hit
    for p in (B or {}).get('pads',[]):
        if p['net']!=net or layer not in p['polys']:continue
        pad=unary_union([Polygon(q) for q in p['polys'][layer]])
        hit=[]
        for t in tracks:
            pts=t['pts']
            if len(pts)<2:continue
            if pad.distance(Point(pts[-1]))<.02:pts=pts[::-1]
            if pad.distance(Point(pts[0]))>=.02:continue
            dx,dy=np.array(pts[1])-np.array(pts[0]);norm=math.hypot(dx,dy)
            if norm<.02:continue
            dx,dy=dx/norm,dy/norm;length=0
            for a,b in zip(pts,pts[1:]):
                vx,vy=b[0]-a[0],b[1]-a[1]
                if abs(vx*dy-vy*dx)>.01 or vx*dx+vy*dy<0:break
                length+=math.hypot(vx,vy)
            x,y=pad.centroid.coords[0]
            normal=[-dy,dx]
            extent=max(abs((q[0]-x)*normal[0]+(q[1]-y)*normal[1])
                       for poly in polygone(pad) for q in poly.exterior.coords)
            hit.append((x,y,dx,dy,length,extent+.14,'pad:'+p['ref']+':'+str(p['num'])))
        if len(hit)==1:out+=hit
    return out


def form_zonen(B,R,K,zones,*,options=None):
    o=options or {}
    res=float(o.get('ZONE_K15_RES','.1'))
    clr=float(o.get('ZCLR','.15'))
    near=float(o.get('ZONE_K15_PAD_NAH','.3'))
    margin=float(o.get('ZONE_K15_PAD_RAND','.14'))
    grow=float(o.get('ZONE_K15_UMWEG','.6'))
    mode=o.get('ZONE_K15_MODE','all')
    if res<=0 or min(clr,near,margin,grow)<0 or mode not in ('all','pads'):
        raise ValueError('Ungültige K15-Optionen')
    eligible={n for n,k in K['netze'].items() if k.get('art') in ('strom','versorgung')}
    editable=[z for z in zones if z['net'] in eligible and z.get('prio',0)>0 and not z.get('tropfen')]
    fixed=[z for z in zones if z not in editable]
    prios={z['net']:z['prio'] for z in editable}
    current=gruppen(editable)
    pads=defaultdict(list)
    for p in B['pads']:
        for L,polys in p['polys'].items():
            pads[p['net'],L]+=[Polygon(q).buffer(0) for q in polys]


    pads={k:unary_union(v) for k,v in pads.items()}
    outline=unary_union([Polygon(q) for q in B['outline']]).buffer(-float(o.get('ZONE_EDGE','.25')))
    via_d=float((R.get('optionen') or {}).get('via','.45/.20').split('/')[0])
    rows=[]
    for net in sorted({n for n,L in current}):
        keys=sorted(k for k in current if k[0]==net)
        source={k:current[k] for k in keys}
        all_z=fixed+als_zonen(current,prios)
        fbase,_=fuelle_zonen(B,R,K,all_z,clearance=clr)
        f=K['netze'][net].get('fluss',{})
        loaded=sum(f.get('last',{}).values())>0
        baseline=None
        if loaded:
            baseline,_,_=netz_loesen(B,R,K,net,fbase,res)
        anchor={};target={};windows={};mask_rows=[];obstacle_count=0;pad_count=0
        for key,g in source.items():
            L=key[1]
            ownpads=pads.get(key,Polygon())

            pp=[p for p in polygone(ownpads) if p.distance(g)<=near]
            pad_count+=len(pp)
            pad_geometries=[]
            for p in pp:
                skirt=p.buffer(margin,quad_segs=12)
                if p.distance(g)>1e-8:


                    _,target_point=nearest_points(p,g)
                    skirt=skirt.union(target_point.buffer(max(.12,margin))).convex_hull
                pad_geometries.append(skirt)
            additions=unary_union(pad_geometries)
            anchor[key]=g.union(additions).intersection(outline)
            shaped=anchor[key];taper_masks=[]
            if mode=='all' and loaded:
                for x,y,dx,dy,straight,r0,kind in terminale(R,net,L,via_d,B):
                    length=min(1.8,straight*.8)
                    if length<.55:continue
                    cut=LineString([(x+length*dx-3*dy,y+length*dy+3*dx),
                                    (x+length*dx+3*dy,y+length*dy-3*dx)])
                    r1=min(1.6,max(r0,shaped.intersection(cut).length/2))
                    if r1<r0+.08:continue
                    mask=tropfenmaske(x,y,dx,dy,r0,r1,length)
                    a=(x-r0*dx,y-r0*dy);b=(x+length*dx,y+length*dy)
                    window=Polygon([(a[0]-2*dy,a[1]+2*dx),(b[0]-2*dy,b[1]+2*dx),
                                    (b[0]+2*dy,b[1]-2*dx),(a[0]+2*dy,a[1]-2*dx)])
                    shaped=shaped.difference(window).union(shaped.intersection(mask)).union(additions)
                    taper_masks.append((window,mask))
                    mask_rows.append(dict(layer=L,x=x,y=y,length=length,r0=r0,r1=r1,kind=kind))


                halos=[]
                for v in R['vias']:
                    if v['net']==net:continue
                    pt=Point(v['x'],v['y'])
                    if pt.distance(g)>.35:continue
                    halos.append(pt.buffer(v.get('diameter',v.get('d',via_d))/2+clr+grow))
                obstacle_count+=len(halos)
                windows[key]=unary_union(halos)
                if halos:
                    shaped=shaped.union(g.buffer(grow,quad_segs=8).intersection(unary_union(halos)))



                for window,mask in taper_masks:
                    shaped=shaped.difference(window).union(shaped.intersection(mask)).union(additions)
            target[key]=shaped.intersection(outline)
        candidates=[('pad+tropfen+umweg',target),('pad',anchor)] if mode=='all' and loaded else [('pad',anchor)]
        chosen=None;trials=[]
        for label,trial in candidates:
            legal={}
            for key,g in trial.items():

                foreign=[p.buffer(clr+.002,quad_segs=12) for k,p in pads.items() if k[1]==key[1] and k[0]!=net]
                other=[p.buffer(clr+.002,quad_segs=8) for k,p in current.items() if k[1]==key[1] and k[0]!=net]


                additions=g.difference(source[key]).difference(unary_union(foreign+other))
                legal[key]=source[key].intersection(g).union(additions).buffer(0)
            new=dict(current)
            new.update(legal)
            output=fixed+als_zonen(new,prios)
            filled,_=fuelle_zonen(B,R,K,output,clearance=clr)
            ok=True;result=None;error=None;trim_area=0
            if loaded:
                try:
                    if label=='pad+tropfen+umweg' and windows:



                        _,grid,state=netz_loesen(B,R,K,net,filled,res)
                        _,density=dichte_flaeche(grid,state,0,0)
                        positive=density[density>1e-12]
                        threshold=.01*float(np.quantile(positive,.95)) if len(positive) else 0
                        active,_=dichte_flaeche(grid,state,threshold,.2)
                        protect,_,_,_,_=geometry(B,dict(R,zones=[]),net,layers_of(B,R,K),via_d)
                        for key,g in legal.items():
                            window=windows.get(key,Polygon())
                            keep=active.get(key[1],Polygon()).union(protect.get(key[1],Polygon()).buffer(margin))
                            shaped=g.difference(window).union(g.intersection(keep))
                            trim_area+=g.difference(shaped).area
                            legal[key]=shaped
                        new.update(legal)
                        filled,_=fuelle_zonen(B,R,K,fixed+als_zonen(new,prios),clearance=clr)
                    result,_,_=netz_loesen(B,R,K,net,filled,res)
                    ok=result['power_W']<=baseline['power_W']+1e-12 and all(
                        result['drop_V'][p]<=v+1e-9 for p,v in baseline['drop_V'].items())
                except Exception as exc:
                    ok=False;error=repr(exc)
            trials.append(dict(candidate=label,accepted=ok,error=error,trim_mm2=trim_area,
                               power_mW=result['power_W']*1000 if result else None))
            if ok:
                chosen=label;current.update(legal);break
        rows.append(dict(net=net,selected=chosen or 'unveraendert',pads=pad_count,
                         taper_candidates=mask_rows,foreign_vias=obstacle_count,
                         baseline_power_mW=baseline['power_W']*1000 if baseline else None,trials=trials))
    return fixed+als_zonen(current,prios),dict(method='K15_local_guarded_v1',options=o,nets=rows,
                                              native_validation_required=True,res_mm=res)
