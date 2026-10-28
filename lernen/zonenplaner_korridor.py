""
import heapq
import math
import numpy as np
import shapely
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from lernen.stromzonen import _polys, sperrflaeche, tangential_runden


def baum(frei, points, width, raster=.25):
    ""
    if not points: raise ValueError('Keine Quellenterminale')
    usable=frei.buffer(-width/2-.002)
    if usable.is_empty:raise ValueError('Kein legal breiter Korridor')
    x0,y0,x1,y1=frei.bounds
    xs=np.arange(math.floor(x0/raster),math.ceil(x1/raster)+1)*raster
    ys=np.arange(math.floor(y0/raster),math.ceil(y1/raster)+1)*raster
    xx,yy=np.meshgrid(xs,ys);valid=shapely.intersects_xy(usable,xx,yy);ny,nx=valid.shape
    def xy(q):return (float(xs[q%nx]),float(ys[q//nx]))
    def node(p):
        ix=int(round((p[0]-xs[0])/raster));iy=int(round((p[1]-ys[0])/raster))
        candidates=sorted((math.dist(p,xy(y*nx+x)),y*nx+x) for y in range(max(0,iy-3),min(ny,iy+4)) for x in range(max(0,ix-3),min(nx,ix+4)) if valid[y,x])
        for _,q in candidates:
            if usable.covers(LineString([p,xy(q)])):return q
        raise ValueError('Terminal ohne legalen Rasterausgang')
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
        if end is None:raise ValueError('Terminal nicht mit Quellbaum erreichbar')
        path=[end]
        while path[-1]!=target:path.append(parent[path[-1]])
        tree.update(path);lines.append(LineString([xy(q) for q in path]).buffer(width/2,quad_segs=32))
    body=unary_union(lines)
    if body.geom_type!='Polygon':raise ValueError('Quellbaum nicht zusammenhaengend')
    rounded=tangential_runden(body,body,frei,.12)
    if rounded.geom_type=='MultiPolygon':
        main=max(rounded.geoms,key=lambda g:g.area)
        if main.buffer(1e-7).covers(body):rounded=main
    if not frei.buffer(1e-8).covers(rounded) or rounded.geom_type!='Polygon':raise ValueError('Korridorform ausserhalb Freiraum')
    return rounded


def ergaenzen(B,K,RG,plan,board,clr):
    mapping={}
    for label in plan['referenzschutz']['gndlagen']:
        if '=' not in label:continue
        layer,names=label.split('=',1)
        if layer.startswith('In'):
            rails=[n for n in names.split('|') if n and n not in K.get('gnd',[]) and n in plan['netze'] and plan['netze'][n]['gewaehlt']]
            if rails:mapping[layer]=rails
    plan['potenziallagen']=sorted(mapping);plan['korridore']=[];plan['via_fenster']=[]
    occupied={L:[] for L in mapping};term={t['id']:t for t in plan['terminale']}
    for L,names in sorted(mapping.items()):
        for n in sorted(names,key=lambda n:(-(plan['netze'].get(n,{}).get('strom_A') or 0),n)):
            fields=[f for f in plan['via_kandidaten'] if f['net']==n and f['angenommen']]
            cores={z['kern_id']:z for z in plan['kern']}
            source=[f for f in fields if any(term[t]['quelle'] for t in cores[f['kern_id']]['terminale'])]
            if not source:
                plan['audit']['ablehnungen'].append(dict(net=n,layer=L,grund='Potenzial: Quellfeld fehlt'));continue
            fields=sorted(fields,key=lambda f:(f not in source,f['kern_id']))

            points=[(v['x'],v['y']) for f in fields for v in f['vias']]
            amp=plan['netze'][n]['strom_A']; width=max(.5,min(4.,.5+.4*(amp or 0)),float(K['netze'][n].get('wmin') or 0))
            obstacles=[Polygon(q).buffer(clr) for p in B['pads'] if p['net']!=n for q in p.get('polys',{}).get(L,[])]
            obstacles += [Point(v['x'],v['y']).buffer(v['d']/2+clr) for f in plan['via_kandidaten'] if f['net']!=n and f['angenommen'] for v in f['vias']]
            obstacles += [g.buffer(clr) for g in occupied[L]]
            obstacles += [sperrflaeche(B,L,'flaechen'),sperrflaeche(B,L,'bahnen'),sperrflaeche(B,L,'vias')]
            free=board.difference(unary_union(obstacles))
            try:
                shape=baum(free,points,width)

                target=min(100.,.2*board.area)
                for extra in (.25,.5,1.,1.5,2.):
                    expanded=shape.buffer(extra,join_style=2).intersection(free)
                    if expanded.geom_type=='Polygon':shape=expanded
                    if shape.area>=target:break
                occupied[L].append(shape)
                ident=n+':'+L+':korridor'
                zone=dict(net=n,layer=L,polys=_polys(shape),quelle='zonenplaner',kern_id=ident,
                          min_thickness=max(.1,float((RG.get('router') or {}).get('bahn_min') or 0)),r36=True,
                          terminale=[t for f in fields for t in cores[f['kern_id']]['terminale']],korridor=True)
                plan['kern'].append(zone)
                record=dict(net=n,layer=L,kern_id=ident,breite_mm=width,strom_A=amp,breitenmodell='Option .5+.4*A mm, nicht thermisch kalibriert',
                            verbunden=True,via_felder=len(fields),flaeche_mm2=shape.area,z1_ziel_mm2=target,z1_erfuellt=shape.area>=target,lochbudget_anteil=.15)

                vd=float(str((RG.get('router') or {}).get('via','0.45/0.20')).split('/')[0]);radius=vd/2+clr+.075
                holes=[];x0,y0,x1,y1=shape.bounds
                for x in np.arange(math.ceil(x0),x1,2.):
                    for y in np.arange(math.ceil(y0),y1,2.):
                        h=Point(x,y).buffer(radius,quad_segs=32)
                        if not shape.covers(h) or any(math.dist((x,y),p)<radius+.5 for p in points):continue
                        if sum(g.area for g in holes)+h.area>.15*shape.area:continue
                        remain=shape.difference(unary_union(holes+[h]))
                        if remain.geom_type!='Polygon' or remain.buffer(-zone['min_thickness']/2).geom_type!='Polygon':continue
                        holes.append(h);plan['via_fenster'].append(dict(net=n,layer=L,x=float(x),y=float(y),radius_mm=.025,max_via_d=vd))
                record['budget_fenster']=len(holes);record['budget_flaeche_mm2']=sum(h.area for h in holes)
                plan['korridore'].append(record)
            except ValueError as exc:
                plan['audit']['ablehnungen'].append(dict(net=n,layer=L,grund='Potenzial: '+str(exc)))

    plan['audit'].update(etappe=3,innenkorridore=True,korridore=len(plan['korridore']),
                         potenzial_fehlend=[q for q in plan['audit']['ablehnungen'] if q['grund'].startswith('Potenzial:')])


def native_pruefung(R,plan):
    ""
    from lernen.stromzonen import _geo
    records=[]
    for c in plan.get('korridore',[]):
        n,L=c['net'],c['layer']
        filled=unary_union([Polygon(z['pts'],z.get('holes',[])) for z in R.get('zones',[]) if z['net']==n and z['layer']==L and z.get('filled')])
        contacts=[Point(v['x'],v['y']).buffer(v['d']/2) for f in plan['via_kandidaten'] if f['net']==n and f['angenommen'] for v in f['vias']]
        copper=unary_union([filled]+contacts)
        original=unary_union([_geo(z) for z in plan['kern'] if z['net']==n and z['layer']==L])
        holes=unary_union([Point(v['x'],v['y']).buffer(float(v.get('d',v.get('diameter',.45)))/2+.15)
                           for v in R['vias'] if v['net']!=n and L in (v.get('layers') or [])]).intersection(original)
        fraction=holes.area/max(original.area,1e-12)
        foreign=[t for t in R['tracks'] if t['layer']==L and t['net']!=n]
        records.append(dict(net=n,layer=L,verbunden=copper.geom_type=='Polygon' and not filled.is_empty,
                            kontakte=all(filled.distance(g)<1e-6 for g in contacts),lochanteil=fraction,
                            fremde_bahnen=len(foreign),bestanden=copper.geom_type=='Polygon' and not filled.is_empty and all(filled.distance(g)<1e-6 for g in contacts) and fraction<=.15+1e-9 and not foreign))
    return dict(bestanden=not plan['audit'].get('potenzial_fehlend') and all(c['bestanden'] for c in records),netze=records)
