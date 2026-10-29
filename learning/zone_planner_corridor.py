import heapq
import math
import numpy as np
import shapely
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from learning.current_zones import _polys, keepout_area, tangential_rounds


def tree_idx(vacant, points, width, raster=.25):
    if not points: raise ValueError('No source terminals')
    usable=vacant.buffer(-width/2-.002)
    if usable.is_empty:raise ValueError('No legally wide corridor')
    x0,y0,x1,y1=vacant.bounds
    xs=np.arange(math.floor(x0/raster),math.ceil(x1/raster)+1)*raster
    ys=np.arange(math.floor(y0/raster),math.ceil(y1/raster)+1)*raster
    xx,yy=np.meshgrid(xs,ys);valid=shapely.intersects_xy(usable,xx,yy);ny,nx=valid.shape
    def xy(q):return (float(xs[q%nx]),float(ys[q//nx]))
    def node(p):
        ix=int(round((p[0]-xs[0])/raster));iy=int(round((p[1]-ys[0])/raster))
        candidates=sorted((math.dist(p,xy(y*nx+x)),y*nx+x) for y in range(max(0,iy-3),min(ny,iy+4)) for x in range(max(0,ix-3),min(nx,ix+4)) if valid[y,x])
        for _,q in candidates:
            if usable.covers(LineString([p,xy(q)])):return q
        raise ValueError('Terminal without legal grid exit')
    nodes=[node(p) for p in points];tree={nodes[0]}; lines=[]
    for p,q in zip(points,nodes):lines.append(LineString([p,xy(q)]).buffer(width/2,quad_segs=32))
    moves=[(dy,dx,math.hypot(dx,dy)*raster) for dy,dx in ((0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1))]
    for target in nodes[1:]:
        if target in tree:continue
        queue=[(0.,target)];distance={target:0.};parent={};end=None
        while queue:
            cost,q=heapq.heappop(queue)
            if cost!=distance[q]:continue
            if q in tree:end=q;break
            y,x=divmod(q,nx)
            for dy,dx,step in moves:
                yy,xx=y+dy,x+dx
                if not 0<=yy<ny or not 0<=xx<nx or not valid[yy,xx]:continue
                other=yy*nx+xx;new=cost+step
                if new>=distance.get(other,float('inf')):continue

                if not usable.covers(LineString([xy(q),xy(other)])):continue
                distance[other]=new;parent[other]=q;heapq.heappush(queue,(new,other))
        if end is None:raise ValueError('Terminal not reachable from the source tree')
        path=[end]
        while path[-1]!=target:path.append(parent[path[-1]])
        tree.update(path);lines.append(LineString([xy(q) for q in path]).buffer(width/2,quad_segs=32))
    body=unary_union(lines)
    if body.geom_type!='Polygon':raise ValueError('Source tree not connected')
    rounded=tangential_rounds(body,body,vacant,.12)
    if rounded.geom_type=='MultiPolygon':
        main=max(rounded.geoms,key=lambda g:g.area)
        if main.buffer(1e-7).covers(body):rounded=main
    if not vacant.buffer(1e-8).covers(rounded) or rounded.geom_type!='Polygon':raise ValueError('Corridor shape outside free space')
    return rounded


def complete(B,K,RG,plan,board,clr):
    mapping={}
    for label in plan['reference_protection']['gnd_layers']:
        if '=' not in label:continue
        layer,names=label.split('=',1)
        if layer.startswith('In'):
            rails=[n for n in names.split('|') if n and n not in K.get('gnd',[]) and n in plan['net_list'] and plan['net_list'][n]['chosen']]
            if rails:mapping[layer]=rails
    plan['potential_layers']=sorted(mapping);plan['corridors']=[];plan['via_window']=[]
    occupied={L:[] for L in mapping};term={t['id']:t for t in plan['terminals']}
    for L,names in sorted(mapping.items()):
        for n in sorted(names,key=lambda n:(-(plan['net_list'].get(n,{}).get('current_A') or 0),n)):
            fields=[f for f in plan['via_candidates'] if f['net']==n and f['taken']]
            cores={z['core_id']:z for z in plan['core_set']}
            source=[f for f in fields if any(term[t]['source'] for t in cores[f['core_id']]['terminals'])]
            if not source:
                plan['audit']['rejections'].append(dict(net=n,layer=L,reason='Potential: source field missing'));continue
            fields=sorted(fields,key=lambda f:(f not in source,f['core_id']))

            points=[(v['x'],v['y']) for f in fields for v in f['vias']]
            amp=plan['net_list'][n]['current_A']; width=max(.5,min(4.,.5+.4*(amp or 0)),float(K['net_list'][n].get('wmin') or 0))
            obstacles=[Polygon(q).buffer(clr) for p in B['pads'] if p['net']!=n for q in p.get('polys',{}).get(L,[])]
            obstacles += [Point(v['x'],v['y']).buffer(v['d']/2+clr) for f in plan['via_candidates'] if f['net']!=n and f['taken'] for v in f['vias']]
            obstacles += [g.buffer(clr) for g in occupied[L]]
            obstacles += [keepout_area(B,L,'areas'),keepout_area(B,L,'track_list'),keepout_area(B,L,'vias')]
            free=board.difference(unary_union(obstacles))
            try:
                shape=tree_idx(free,points,width)

                target=min(100.,.2*board.area)
                for extra in (.25,.5,1.,1.5,2.):
                    expanded=shape.buffer(extra,join_style=2).intersection(free)
                    if expanded.geom_type=='Polygon':shape=expanded
                    if shape.area>=target:break
                occupied[L].append(shape)
                ident=n+':'+L+':corridor'
                zone=dict(net=n,layer=L,polys=_polys(shape),source='zone_planner',core_id=ident,
                          min_thickness=max(.1,float((RG.get('router') or {}).get('track_min') or 0)),r36=True,
                          terminals=[t for f in fields for t in cores[f['core_id']]['terminals']],corridor=True)
                plan['core_set'].append(zone)
                record=dict(net=n,layer=L,core_id=ident,width_mm=width,current_A=amp,width_model='Option .5+.4*A mm, not thermally calibrated',
                            joined=True,via_fields=len(fields),area_mm2=shape.area,z1_goal_mm2=target,z1_fulfilled=shape.area>=target,hole_budget_share=.15)

                vd=float(str((RG.get('router') or {}).get('via','0.45/0.20')).split('/')[0]);radius=vd/2+clr+.075
                holes=[];x0,y0,x1,y1=shape.bounds
                for x in np.arange(math.ceil(x0),x1,2.):
                    for y in np.arange(math.ceil(y0),y1,2.):
                        h=Point(x,y).buffer(radius,quad_segs=32)
                        if not shape.covers(h) or any(math.dist((x,y),p)<radius+.5 for p in points):continue
                        if sum(g.area for g in holes)+h.area>.15*shape.area:continue
                        remain=shape.difference(unary_union(holes+[h]))
                        if remain.geom_type!='Polygon' or remain.buffer(-zone['min_thickness']/2).geom_type!='Polygon':continue
                        holes.append(h);plan['via_window'].append(dict(net=n,layer=L,x=float(x),y=float(y),radius_mm=.025,max_via_d=vd))
                record['budget_window']=len(holes);record['budget_area_mm2']=sum(h.area for h in holes)
                plan['corridors'].append(record)
            except ValueError as exc:
                plan['audit']['rejections'].append(dict(net=n,layer=L,reason='Potential: '+str(exc)))

    plan['audit'].update(stage=3,inner_corridors=True,corridors=len(plan['corridors']),
                         potential_missing=[q for q in plan['audit']['rejections'] if q['reason'].startswith('Potential:')])


def native_checking(R,plan):
    from learning.current_zones import _geo
    records=[]
    for c in plan.get('corridors',[]):
        n,L=c['net'],c['layer']
        filled=unary_union([Polygon(z['pts'],z.get('holes',[])) for z in R.get('zones',[]) if z['net']==n and z['layer']==L and z.get('filled')])
        contacts=[Point(v['x'],v['y']).buffer(v['d']/2) for f in plan['via_candidates'] if f['net']==n and f['taken'] for v in f['vias']]
        copper=unary_union([filled]+contacts)
        original=unary_union([_geo(z) for z in plan['core_set'] if z['net']==n and z['layer']==L])
        holes=unary_union([Point(v['x'],v['y']).buffer(float(v.get('d',v.get('diameter',.45)))/2+.15)
                           for v in R['vias'] if v['net']!=n and L in (v.get('layers') or [])]).intersection(original)
        fraction=holes.area/max(original.area,1e-12)
        foreign=[t for t in R['tracks'] if t['layer']==L and t['net']!=n]
        records.append(dict(net=n,layer=L,joined=copper.geom_type=='Polygon' and not filled.is_empty,
                            contacts=all(filled.distance(g)<1e-6 for g in contacts),hole_share=fraction,
                            foreign_tracks=len(foreign),passed=copper.geom_type=='Polygon' and not filled.is_empty and all(filled.distance(g)<1e-6 for g in contacts) and fraction<=.15+1e-9 and not foreign))
    return dict(passed=not plan['audit'].get('potential_missing') and all(c['passed'] for c in records),net_list=records)
