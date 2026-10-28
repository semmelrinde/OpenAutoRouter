""
import math
import numpy as np
import shapely
from scipy.ndimage import distance_transform_edt, label, convolve
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from lernen import stromzonen as SZ
from numba import njit

VERSION='zonen-geometrie-v2-skelettkanten/medial-0.025'


def teile(g):
    return [p for p in getattr(g,'geoms',[g]) if p.geom_type=='Polygon' and not p.is_empty]


def raster(g,h=.025,pad=.25):
    x0,y0,x1,y1=g.bounds
    x0=math.floor((x0-pad)/h)*h;y0=math.floor((y0-pad)/h)*h
    if math.ceil((x1+pad-x0)/h)*math.ceil((y1+pad-y0)/h)>5_000_000:
        raise ValueError('Medialraster zu groß; keine unvollständige Kennzahl')
    xx,yy=np.meshgrid(x0+(np.arange(math.ceil((x1+pad-x0)/h))+.5)*h,
                      y0+(np.arange(math.ceil((y1+pad-y0)/h))+.5)*h)
    return shapely.intersects_xy(g,xx,yy),xx,yy


@njit(cache=True)
def _duennen(m):


    candidates = []
    boundary = np.zeros(m.shape,np.bool_)
    for y in range(1,m.shape[0]-1):
        for x in range(1,m.shape[1]-1):
            if m[y,x] and not (m[y-1,x] and m[y,x+1] and m[y+1,x] and m[y,x-1]):
                candidates.append((y,x)); boundary[y,x] = True
    while True:
        removed = 0
        for step in (0,1):
            gone = []
            for y,x in candidates:
                if not m[y,x]: continue
                p = (m[y-1,x],m[y-1,x+1],m[y,x+1],m[y+1,x+1],
                     m[y+1,x],m[y+1,x-1],m[y,x-1],m[y-1,x-1])
                count = 0; transitions = 0
                for i in range(8):
                    count += int(p[i]); transitions += int(not p[i] and p[(i+1)%8])
                preserve = ((p[0] and p[2] and p[4]) or (p[2] and p[4] and p[6])) if step == 0 else ((p[0] and p[2] and p[6]) or (p[0] and p[4] and p[6]))
                if 2<=count<=6 and transitions==1 and not preserve: gone.append((y,x))
            for y,x in gone: m[y,x] = False
            removed += len(gone)
            new = []
            for y,x in candidates:
                if m[y,x]: new.append((y,x))
                else: boundary[y,x] = False
            for y,x in gone:
                for dy in range(-1,2):
                    for dx in range(-1,2):
                        yy,xx = y+dy,x+dx
                        if m[yy,xx] and not boundary[yy,xx]:
                            new.append((yy,xx)); boundary[yy,xx] = True
            candidates = new
        if not removed: break
    return m[1:-1,1:-1]


def skelett(mask):
    return _duennen(np.pad(mask.astype(bool), 1))


def skelettkanten(mask,h):
    ""
    m=np.pad(mask.astype(bool),1); degree=np.zeros(m.shape,np.int32);length=0.
    for dy,dx in ((0,1),(1,0),(1,1),(1,-1)):
        edges=m & np.roll(m,(-dy,-dx),axis=(0,1))
        if dy and dx:
            edges &= ~np.roll(m,-dy,axis=0) & ~np.roll(m,-dx,axis=1)
        length += int(np.count_nonzero(edges))*math.hypot(dy,dx)*h
        degree += edges
        degree += np.roll(edges,(dy,dx),axis=(0,1))
    return float(length),degree[1:-1,1:-1]


def _mark(art,n,l,x,y,**kwargs):
    return dict(art=art,netz=n,lage=l,ort=[round(float(x),4),round(float(y),4),l],**kwargs)


def _gebiet(xx,yy,ys,xs,h):
    ""
    cells=shapely.box(xx[ys,xs]-h/2,yy[ys,xs]-h/2,xx[ys,xs]+h/2,yy[ys,xs]+h/2)
    return SZ._polys(shapely.union_all(cells))


def medial_pruefen(g,n,l,own,minimum,h=.025):
    mask,xx,yy=raster(g,h)
    if mask.size>5_000_000:
        raise ValueError('Medialraster zu groß; Auflösung explizit erhöhen')
    sk=skelett(mask);d=distance_transform_edt(mask)*h
    degree=convolve(sk.astype(int),np.ones((3,3),int),mode='constant')-sk
    narrow=sk&(2*d<minimum+2*h)
    marks=[]
    lab,num=label(narrow,np.ones((3,3)))
    for i in range(1,num+1):
        ys,xs=np.nonzero(lab==i);j=int(np.argmin(d[ys,xs]));y,x=ys[j],xs[j]
        point=Point(xx[y,x],yy[y,x]);width=2*d[y,x]
        length,graphdegree=skelettkanten(lab[ys.min():ys.max()+1,xs.min():xs.max()+1]==i,h)
        marks.append(_mark('nebenstreifen',n,l,point.x,point.y,breite_mm=float(width),laenge_mm=length,
                           laengenmass='Skelettkantensumme',verzweigungspixel=int(np.count_nonzero(graphdegree>2)),
                           endlappen=bool(np.any(degree[ys,xs]<=1)),geerbt=False,
                           sicher_unter_minimum=bool(width+2*h<minimum),unsicherheit_mm=2*h,
                           gebiet=_gebiet(xx,yy,ys,xs,h)))
    for p in teile(g):
        if p.area<.5:
            q=p.representative_point();marks.append(_mark('splitter',n,l,q.x,q.y,flaeche_mm2=p.area,geerbt=own.buffer(h).covers(p),gebiet=SZ._polys(p)))


    white=~mask;ws=skelett(white);dw=distance_transform_edt(white)*h
    ys,xs=np.nonzero(ws&(2*dw<max(.25,minimum)+2*h)&(dw>0))
    slot=np.zeros(mask.shape,bool)
    for dy,dx in ((0,1),(1,0),(1,1),(1,-1)):
        norm=math.hypot(dx,dy);rad=dw[ys,xs]+h*1.5
        qx=xx[ys,xs];qy=yy[ys,xs]
        both=shapely.intersects_xy(g,qx+rad*dx/norm,qy+rad*dy/norm)&shapely.intersects_xy(g,qx-rad*dx/norm,qy-rad*dy/norm)
        slot[ys[both],xs[both]]=True
    lab,num=label(slot,np.ones((3,3)))
    for i in range(1,num+1):
        ys,xs=np.nonzero(lab==i)
        length,graphdegree=skelettkanten(lab[ys.min():ys.max()+1,xs.min():xs.max()+1]==i,h)
        if length<.05:continue
        j=int(np.argmin(dw[ys,xs]));y,x=ys[j],xs[j]
        marks.append(_mark('eigennetz_schlitz',n,l,xx[y,x],yy[y,x],breite_mm=float(2*dw[y,x]),
                           laenge_mm=length,laengenmass='Skelettkantensumme',
                           verzweigungspixel=int(np.count_nonzero(graphdegree>2)),unsicherheit_mm=2*h,geerbt=False,
                           gebiet=_gebiet(xx,yy,ys,xs,h)))
    cover = own.buffer(h)
    for mark in marks:
        if mark['art'] == 'nebenstreifen':
            support = unary_union([Polygon(q['aussen'],q.get('innen') or []) for q in mark['gebiet']])
            mark['geerbt'] = cover.covers(support)
    return marks,(mask,sk,d,xx,yy,degree)


def via_pruefen(raum,n,l,g,own):
    from analyse.kupfermodell import via_masse,auf_lage
    out=[]
    external=unary_union([Polygon(p.exterior) for p in teile(g)])
    for v in raum.R.get('vias',[]):
        if v['net']==n or not auf_lage(v,l):continue
        d,dr=via_masse(v,raum.vd,raum.drill);radius=max(d/2+raum.clr,dr/2+raum.hole)
        disk=Point(v['x'],v['y']).buffer(radius+.02,32)

        cut=g.intersection(disk)
        if external.intersects(disk) and not external.covers(disk) and not cut.is_empty:
            q=cut.representative_point();out.append(_mark('via_biss',n,l,q.x,q.y,via=[v['x'],v['y']],via_netz=v['net'],
                    resthals_mm=float(g.boundary.distance(Point(v['x'],v['y']))*2),geerbt=own.buffer(.025).covers(cut),
                    gebiet=SZ._polys(cut),unsicherheit_mm=.025))
    return out


def messen(B, R, K, h=.025):
    ""
    import time
    from types import SimpleNamespace
    from analyse.kupfermodell import via_geo
    start = time.process_time()
    zs = [z for z in R.get('zonen',[]) if z.get('quelle') == 'stromzonen']
    gs = {}
    for z in zs:
        key = z['net'], z['layer']
        gs.setdefault(key, []).append(SZ._geo(z))
    vd, drill = via_geo(R,K)
    rg = (K.get('regeln') or {}).get('router', K.get('regeln') or {})
    raum = SimpleNamespace(R=R, vd=vd, drill=drill,
        clr=max(.15,float(R.get('clr') or (R.get('optionen') or {}).get('clr') or rg.get('clr') or .15)),
        hole=float((R.get('optionen') or {}).get('loch_clr') or rg.get('loch_clr') or .2))
    base = dict(R, zonen=[z for z in R.get('zonen',[]) if z.get('quelle') not in ('stromzonen','gnd_fuellung')])
    parts = {l: SZ._teile(B,base,l) for l in sorted({l for _,l in gs})}
    marks = []
    for (n,l), pieces in sorted(gs.items()):
        g = unary_union(pieces)
        own = unary_union([q for m,q in parts[l] if m == n])
        minimum = max(.1, float(K.get('netze',{}).get(n,{}).get('wmin') or .1),
                      .25 if K.get('netze',{}).get(n,{}).get('art') == 'sw' else .1)


        components = teile(g)
        tree = shapely.STRtree(components)
        parent = list(range(len(components)))
        def find(i):
            while parent[i] != i: i = parent[i]
            return i
        reach = max(.25,minimum)+2*h
        for i,c in enumerate(components):
            for j in sorted(tree.query(c.buffer(reach).envelope)):
                if j > i and c.distance(components[j]) <= reach: parent[find(int(j))] = find(i)
        groups = {}
        for i,c in enumerate(components): groups.setdefault(find(i),[]).append(c)
        for pieces in groups.values():
            component = unary_union(pieces)
            mm, _ = medial_pruefen(component,n,l,own,minimum,h)
            for mark in mm:
                if mark['art'] == 'nebenstreifen': mark['minimum_mm'] = minimum
            marks += [q for q in mm if q['art'] != 'splitter']
        marks += via_pruefen(raum,n,l,g,own)
    kinds = ('nebenstreifen','eigennetz_schlitz','via_biss')
    return dict(version=VERSION, raster_mm=h, unsicherheit_mm=2*h,
        cpu_s=time.process_time()-start, markierungen=marks,
        zaehler={k:sum(q['art']==k and not q['geerbt'] for q in marks) for k in kinds},
        geerbt={k:sum(q['art']==k and q['geerbt'] for q in marks) for k in kinds})
