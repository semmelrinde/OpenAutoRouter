""
import copy
import heapq
import math
import numpy as np
import shapely
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union


MAX_RASTERPUNKTE=250000
MAX_SUCHZUSTAENDE=30000
MAX_WARTESCHLANGE=60000


def pfad(a,b,free,*,diagnose=None,max_raster=MAX_RASTERPUNKTE,
         max_zustaende=MAX_SUCHZUSTAENDE,max_queue=MAX_WARTESCHLANGE):
    ""
    if min(max_raster,max_zustaende,max_queue)<4:
        raise ValueError('Suchbudgets muessen mindestens 4 betragen')
    info=diagnose if diagnose is not None else {}
    info.update(raster_limit=max_raster,zustaende_limit=max_zustaende,
                queue_limit=max_queue,raster=[],zustaende=0,queue_max=0,grund='')
    def budgetende(grund):
        info['grund']=grund
        raise ValueError('Suchbudget erschoepft (%s; Raster <= %d, Zustaende %d/%d, Queue %d/%d)' %
                         (grund,max_raster,info['zustaende'],max_zustaende,info['queue_max'],max_queue))
    shapely.prepare(free)
    direct=LineString([a,b])
    if free.covers(direct):
        info['grund']='direkt';return [a,b]

    windows=[(np.minimum(a,b)-m,np.maximum(a,b)+m,.05) for m in (1.,3.,8.)]


    windows += [(np.asarray(free.bounds[:2]),np.asarray(free.bounds[2:]),s) for s in (.05,.025)]
    if not free.covers(Point(a)) or not free.covers(Point(b)):
        raise ValueError('Kein lokaler legaler Korridor zwischen '+str((a,b)))
    for lo,hi,step in windows:
        requested=step


        def dimensionen():
            return tuple(math.ceil(float(d)/step)+1 for d in hi-lo)
        nx,ny=dimensionen()
        while nx*ny>max_raster:
            step*=max(1.1,math.sqrt(nx*ny/max_raster))
            nx,ny=dimensionen()
        info['raster'].append(dict(soll_mm=requested,ist_mm=step,punkte=nx*ny))
        xx,yy=np.meshgrid(lo[0]+np.arange(nx)*step,lo[1]+np.arange(ny)*step)
        legal=shapely.intersects_xy(free,xx,yy)
        ends=[]
        for p in (a,b):
            choices=[]
            ix,iy=np.rint((np.asarray(p)-lo)/step).astype(int)
            for dx in range(-3,4):
                for dy in range(-3,4):
                    x,y=ix+dx,iy+dy
                    if 0<=x<nx and 0<=y<ny and legal[y,x]:
                        q=(lo[0]+x*step,lo[1]+y*step)
                        if free.covers(LineString([p,q])):choices.append((math.dist(p,q),(x,y)))
            if not choices:break
            ends.append(min(choices)[1])
        if len(ends)!=2:continue
        if info['zustaende']>=max_zustaende:budgetende('Zustaende')
        start,end=ends;todo=[(0.,0.,start)];cost={start:0};prev={}
        info['zustaende']+=1;info['queue_max']=max(info['queue_max'],1)
        while todo:
            _,d,u=heapq.heappop(todo)
            if d!=cost[u]:continue
            if u==end:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                v=(u[0]+dx,u[1]+dy)
                if not(0<=v[0]<nx and 0<=v[1]<ny and legal[v[1],v[0]]):continue
                nd=d+math.hypot(dx,dy)
                if nd>=cost.get(v,float('inf')):continue
                p=lo+np.asarray(u)*step;q=lo+np.asarray(v)*step
                if not free.covers(LineString([p,q])):continue
                if v not in cost:
                    if info['zustaende']>=max_zustaende:budgetende('Zustaende')
                    info['zustaende']+=1
                if len(todo)>=max_queue:budgetende('Queue')
                cost[v]=nd;prev[v]=u
                heapq.heappush(todo,(nd+math.dist(v,end),nd,v))
                info['queue_max']=max(info['queue_max'],len(todo))
        if end not in cost:continue
        chain=[end]
        while chain[-1]!=start:chain.append(prev[chain[-1]])
        pts=[tuple(a)]+[tuple(lo+np.asarray(u)*step) for u in reversed(chain)]+[tuple(b)]
        result=[pts[0]];i=0
        while i<len(pts)-1:
            j=len(pts)-1
            while j>i+1 and not free.covers(LineString([pts[i],pts[j]])):j-=1
            result.append(pts[j]);i=j
        info['grund']='gefunden';return result
    info['grund']='Raster ohne Weg (kein Unerreichbarkeitsbeweis)'
    raise ValueError('Kein lokaler legaler Korridor zwischen '+str((a,b))+
                     '; Suchbudget: Raster <= %d, Zustaende %d/%d, Queue %d/%d; %s' %
                     (max_raster,info['zustaende'],max_zustaende,info['queue_max'],max_queue,info['grund']))


def reparieren(B,R,oldpads,moved,clr,hole):
    from lernen.stromzonen import sperrflaeche,_lagen,_via_masse
    from lernen.kupferbalken import track_geometry
    R=copy.deepcopy(R);vd,_=_via_masse(R);changes=[]

    translations=[]
    for p in oldpads:
        if p['ref'] not in moved:continue
        dx,dy,_=moved[p['ref']]
        g=unary_union([Polygon(q) for qs in p['polys'].values() for q in qs]).buffer(.025)
        translations.append((p['net'],g,dx,dy))
    for v in R['vias']:
        for n,g,dx,dy in translations:
            if v['net']==n and g.intersects(Point(v['x'],v['y'])):
                ox,oy=v['x'],v['y'];v['x']+=dx;v['y']+=dy
                translations.append((n,Point(ox,oy).buffer(.01),dx,dy));break
    changed_tracks=set()
    for ti,t in enumerate(R['tracks']):
        changed=False
        for index in (0,-1):
            for n,g,dx,dy in translations:
                if t['net']==n and g.intersects(Point(t['pts'][index])):
                    x,y=t['pts'][index];t['pts'][index]=[x+dx,y+dy];changed=True;break
        if changed:
            changed_tracks.add(ti)
            t.pop('items',None);t.pop('arcs',None);t.pop('bogen',None)
    outer=unary_union([Polygon(q) for q in B['outline']]).buffer(-.2)
    shapes=[track_geometry(t) for t in R['tracks']]
    for i,t in enumerate(R['tracks']):
        n,L=t['net'],t['layer']
        def free_for(w):
            obstacles=[sperrflaeche(B,L,'bahnen').buffer(w/2+.004)]
            obstacles += [Polygon(q).buffer(clr+w/2+.004) for p in B['pads'] if p['net']!=n for q in p['polys'].get(L,[])]
            obstacles += [Point(p['x'],p['y']).buffer(p['drill']/2+hole+w/2+.004) for p in B['pads'] if p.get('drill') and p['net']!=n]
            obstacles += [shapes[j].buffer(clr+w/2+.004) for j,other in enumerate(R['tracks']) if j!=i and other['net']!=n and other['layer']==L]
            obstacles += [Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',vd))/2+clr+w/2+.004) for v in R['vias'] if v['net']!=n and L in v.get('layers',_lagen(B))]
            return outer.buffer(-w/2).difference(unary_union(obstacles))


        g=shapes[i]
        bad=g.intersection(sperrflaeche(B,L,'bahnen')).area>1e-8
        bad |= any(g.distance(Point(p['x'],p['y']).buffer(p['drill']/2))<hole-.0001 for p in B['pads'] if p.get('drill') and p['net']!=n)
        bad |= any(g.distance(shapes[j])<clr-.0001 for j,o in enumerate(R['tracks']) if j!=i and o['net']!=n and o['layer']==L)
        bad |= i in changed_tracks
        if not bad:continue

        pts=[];widths=[];free_cache={};searches=[]
        original_pts=t['pts']; original_widths=t['w']; index=0

        contacts=[Polygon(q) for p in B['pads'] if p['net']==n for q in p['polys'].get(L,[])]
        contacts += [Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',vd))/2)
                     for v in R['vias'] if v['net']==n and L in v.get('layers',_lagen(B))]
        contacts += [Point(q).buffer(.001) for j,o in enumerate(R['tracks']) if j!=i and o['net']==n and o['layer']==L
                     for q in (o['pts'][0],o['pts'][-1])]
        while index<len(original_widths):
            a,b=original_pts[index:index+2];w0=original_widths[index]
            start=index;end=index+1
            for w in (w0,.1):
                if w not in free_cache:free_cache[w]=free_for(w)
                search={};searches.append(search)
                try:
                    q=pfad(a,b,free_cache[w],diagnose=search)
                    break
                except ValueError:
                    if w!=.1:continue
                    free=free_cache[w]


                    start=index
                    while start>0 and not free.covers(Point(original_pts[start])):start-=1
                    while end<len(original_pts)-1 and not free.covers(Point(original_pts[end])):end+=1
                    if start==index and end==index+1:raise
                    search={};searches.append(search)
                    q=pfad(original_pts[start],original_pts[end],free,diagnose=search)
                    if start<index:


                        anchor=next((j for j in range(len(pts)-1,-1,-1)
                                     if math.dist(pts[j],original_pts[start])<1e-8),None)
                        if anchor is None:raise ValueError('Kein erhaltener legaler Korridoranker: '+n)
                        pts=pts[:anchor+1];widths=widths[:anchor]


            old_span=LineString(original_pts[start:end+1]).buffer(w/2)
            new_span=LineString(q).buffer(w/2)
            if any(old_span.intersects(c) and not new_span.intersects(c) for c in contacts):
                raise ValueError('Lokaler Korridor wuerde Pad-/Via-/Abzweigkontakt verlieren: '+n)
            pts+=q if not pts else q[1:]
            widths += [w]*(len(q)-1)
            index=end
        t['pts']=[list(q) for q in pts];t['w']=widths
        t.pop('items',None);t.pop('arcs',None);t.pop('bogen',None)
        shapes[i]=track_geometry(t)
        changes.append(dict(net=n,layer=L,breite_min=min(widths),suche=searches))
    return R,changes
