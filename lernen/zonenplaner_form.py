""
import math

STEP=22.501
FLAT=.02
TANGENT=.2


def turn(a,b,c):
    u=(b[0]-a[0],b[1]-a[1]);v=(c[0]-b[0],c[1]-b[1])
    return math.degrees(math.atan2(u[0]*v[1]-u[1]*v[0],u[0]*v[0]+u[1]*v[1]))


def circle(points):
    a,b,c=points[0],points[len(points)//2],points[-1]
    bx,by=b[0]-a[0],b[1]-a[1];cx,cy=c[0]-a[0],c[1]-a[1]
    den=2*(bx*cy-by*cx)
    if abs(den)<1e-14:return None
    x=a[0]+((bx*bx+by*by)*cy-(cx*cx+cy*cy)*by)/den
    y=a[1]+(bx*(cx*cx+cy*cy)-cx*(bx*bx+by*by))/den
    r=math.dist(a,(x,y));tol=max(4e-6,r*1e-4)
    if not math.isfinite(r) or r<1e-7:return None
    if any(abs(math.dist(p,(x,y))-r)>tol for p in points):return None
    ts=[turn(*points[j:j+3]) for j in range(len(points)-2)]
    if not ts or min(abs(t) for t in ts)<=FLAT or max(abs(t) for t in ts)>STEP:return None
    if min(ts)*max(ts)<0:return None
    return (x,y),r,tol


def tangent(points,start,length,fit):
    center,r,tol=fit;n=len(points)
    for endpoint,outside,direction in ((start,start-1,-1),(start+length-1,start+length,1)):
        p=points[endpoint%n];q=points[outside%n]
        if abs(math.dist(q,center)-r)<=tol:continue
        line=(q[0]-p[0],q[1]-p[1]);radial=(center[0]-p[0],center[1]-p[1])
        den=math.hypot(*line)*r
        if den and abs(line[0]*radial[0]+line[1]*radial[1])/den<=math.sin(math.radians(TANGENT)):continue

        adjacent=[points[(endpoint+direction*j)%n] for j in range(min(7,n))]
        other=circle(adjacent)
        if other:
            rad2=(other[0][0]-p[0],other[0][1]-p[1]);den2=r*other[1]
            if den2 and abs(radial[0]*rad2[1]-radial[1]*rad2[0])/den2<=math.sin(math.radians(TANGENT)):continue
        return False
    return True


def pruefen(g):
    bad=[];arcs=0;checked=0
    if g.is_empty:return dict(bestanden=False,freie_knicke=[],grund='Leeres Kupfer')
    for poly in getattr(g,'geoms',[g]):
        if poly.geom_type!='Polygon' or not poly.is_valid:
            return dict(bestanden=False,freie_knicke=[],grund='Ungültige Kupferkontur')
        for ring in [poly.exterior,*poly.interiors]:
            points=[]
            for p in list(ring.coords)[:-1]:
                if not points or math.dist(points[-1],p)>1e-9:points.append(p)
            if len(points)>1 and math.dist(points[0],points[-1])<1e-9:points.pop()
            n=len(points)
            for i,p in enumerate(points):
                angle=turn(points[i-1],p,points[(i+1)%n]);checked+=1
                if abs(angle)<=FLAT:continue
                found=False
                if abs(angle)<=STEP:
                    for length in (7,4):
                        if n<length:continue
                        for offset in range(length):
                            start=i-offset;window=[points[(start+j)%n] for j in range(length)]
                            fit=circle(window)
                            if fit and tangent(points,start,length,fit):found=True;break
                        if found:break
                if found:arcs+=1
                else:bad.append(dict(xy=list(p),richtungswechsel_grad=round(angle,3)))
    return dict(bestanden=not bad,freie_knicke=bad,knoten=checked,kreisbogensehnen=arcs,
                sehnenwinkel_max_grad=STEP,tangenten_toleranz_grad=TANGENT,version='native-k17-kreis-v1')


def kurz(result):
    return dict({k:v for k,v in result.items() if k!='freie_knicke'},
                freie_knicke_anzahl=len(result['freie_knicke']),beispiele=result['freie_knicke'][:8])


def mindestbreiten(B,R,RG,plan):
    ""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    floor=max(float((B.get('design') or {}).get('min_breite') or 0),
              float((RG or {}).get('roh',{}).get('min_track_width') or 0),
              float((RG or {}).get('router',{}).get('bahn_min') or 0))
    for z in R.get('zones',[]):
        outlines=z.get('native_outlines') or [z]
        g=unary_union([Polygon(q['pts'],q.get('holes',[])) for q in outlines])
        sw=plan['netze'].get(z['net'],{}).get('lokal_ohne_lagenwechsel')
        minimum=.25 if sw or g.area>=20 else .1
        z['prio']=0 if z['net'] in plan.get('kontext',{}).get('gndnetze',['GND','/GND']) else max(1,int(z.get('prio') or 1))
        z['full']=True
        z['min_thickness']=max(float(z.get('min_thickness') or 0),minimum,floor)
    return R


def kupferformen(R):
    ""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    groups={}
    for z in R.get('zones',[]):
        groups.setdefault((z['net'],z['layer']),[]).append(Polygon(z['pts'],z.get('holes',[])))
    return [dict(net=n,layer=L,**kurz(pruefen(unary_union(parts)))) for (n,L),parts in sorted(groups.items())]
