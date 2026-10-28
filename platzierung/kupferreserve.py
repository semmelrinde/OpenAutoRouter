""
import math
from shapely import affinity
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from lernen.zonenplaner import auswahl, padid
from lernen.zonenplaner_vertrag import geometrie


def anwenden(T, B, K, RG):


    if int(B['lagen']) < 4:return []
    op=geometrie(RG);d,drill=op['via'];clr=max(op['clr'],op['drcpad'])+.02
    pitch=max(.5, drill+op['h2h']+.01)
    selected=auswahl(K); groups={}; result=[]
    fps={f['ref']:f for f in B['fps']}
    for p in B['pads']:
        n=p['net']; ti=K['teile'].get(p['ref'],{}); spec=selected.get(n,{})
        if not spec.get('gewaehlt') or spec.get('lokal_ohne_lagenwechsel') or p['ref'] not in T:continue
        if any(x in str(p.get('pinfunc','')).upper() for x in ('FB','SENSE','VSNS','ISNS')) or ti.get('funktion') in ('teiler','rueckkopplung','einstell_r'):continue
        group=(ti.get('stuetzt') or {}).get('ref') or ti.get('insel') or p['ref']
        groups.setdefault((n,group),[]).append(p)
    for (n,group),ps in sorted(groups.items()):

        source=set((K['netze'][n].get('fluss') or {}).get('quelle') or [])
        p=min(ps,key=lambda p:(padid(p) not in source,not p['ref'].startswith('C'),'BGA' in fps[p['ref']].get('fp','').upper(),p['ref'],str(p['num'])))
        r=p['ref']; t=T[r]; amps=selected[n]['strom_A']
        count=max(2,math.ceil(4*amps),math.ceil(1.3*amps/(drill/.3))) if amps is not None and drill>0 else 2
        cols=math.ceil(math.sqrt(count)); rows=math.ceil(count/cols)
        own=[(q['net'],Polygon(poly)) for q in B['pads'] if q['ref']==r for polys in q['polys'].values() for poly in polys]
        chosen=None
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            for reach in (.7,1.2,1.7,2.2,3.):
                points=[(p['x']+dx*reach+(j%cols-(cols-1)/2)*pitch,
                         p['y']+dy*reach+(j//cols-(rows-1)/2)*pitch) for j in range(count)]
                disks=unary_union([Point(*xy).buffer(d/2+clr) for xy in points])
                if any(disks.intersects(g) for net,g in own if net!=n):continue
                if op['vip']!=1 and any(disks.distance(g)<.1 for _,g in own):continue
                bridge=LineString([(p['x'],p['y']),points[0]]).buffer(.125+clr)
                if any(bridge.intersects(g) for net,g in own if net!=n):continue
                chosen=points;break
            if chosen:break
        if chosen is None:
            result.append(dict(net=n,gruppe=group,ref=r,soll=count,grund='eigener Pad-Ausgang gesperrt',vias=[]));continue
        bridge=LineString([(p['x'],p['y']),chosen[0]]).buffer(max(.25,float(K['netze'][n].get('wmin') or .25))/2+clr)
        footprint=unary_union([bridge]+[Point(*xy).buffer(d/2+clr) for xy in chosen])
        local=affinity.translate(footprint,-t.x0,-t.y0)
        for side in ('own','other'):
            t.g[side]=local if t.g[side] is None else t.g[side].union(local)
        t._cache.clear();t.flaeche=t.g['own'].area
        result.append(dict(net=n,gruppe=group,ref=r,pad=padid(p),soll=count,strom_A=amps,pitch_mm=pitch,
                           vias=[dict(x=x-t.x0,y=y-t.y0,d=d,drill=drill) for x,y in chosen],
                           polys=[dict(aussen=list(local.exterior.coords),innen=[])] if local.geom_type=='Polygon' else []))
    return result


def absolut(reserves, T, lage, fps):
    out=[]
    for field in reserves:
        r=field['ref'];x,y,theta,side=lage.get(r,(fps[r]['x'],fps[r]['y'],0,fps[r]['side']))
        def transform(g):
            if side!=T[r].side0:g=affinity.scale(g,-1,1,origin=(0,0))
            return affinity.translate(affinity.rotate(g,theta,origin=(0,0)),x,y)
        vias=[]
        for v in field['vias']:
            p=transform(Point(v['x'],v['y']));vias.append(dict(v,x=p.x,y=p.y))
        out.append(dict(field,vias=vias,layer=side))
    return out
