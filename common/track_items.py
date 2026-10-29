import math


def arc_geometry(it):
    a,m,b=it[1:4]
    ux,uy=m[0]-a[0],m[1]-a[1];vx,vy=b[0]-a[0],b[1]-a[1]
    det=2*(ux*vy-uy*vx)
    if abs(det)<1e-14:raise ValueError('degenerate A arc')
    uu,vv=ux*ux+uy*uy,vx*vx+vy*vy
    c=(a[0]+(uu*vy-vv*uy)/det,a[1]+(ux*vv-vx*uu)/det)
    ang=[math.atan2(p[1]-c[1],p[0]-c[0]) for p in (a,m,b)]
    ccw=(ang[2]-ang[0])%(2*math.pi)
    sweep=ccw if (ang[1]-ang[0])%(2*math.pi)<=ccw+1e-9 else ccw-2*math.pi
    return c,math.dist(a,c),ang[0],sweep


def native_arc_representable(it):
    a,m,b=it[1:4]
    def cross(points):
        a,m,b=points
        return (m[0]-a[0])*(b[1]-a[1])-(m[1]-a[1])*(b[0]-a[0])
    area=cross([a,m,b]);q=cross([[int(v*1e6) for v in p] for p in (a,m,b)])
    return abs(area)>=5e-15 and q!=0 and area*q>0


def validate(t,error=.00025):
    if not math.isfinite(error) or error<0:raise ValueError('invalid chord error')
    if 'items' not in t:return
    items=t['items'];P=t['pts'];W=t['w'];eps=2e-6
    if len(P)<2 or len(W)!=len(P)-1 or not items:raise ValueError('invalid item preview')
    cursor=None
    for it in items:
        if not it or it[0] not in ('L','A') or len(it)!=(4 if it[0]=='L' else 5):raise ValueError('invalid track item')
        if not isinstance(it[-1],(int,float)) or not math.isfinite(it[-1]) or it[-1]<=0:raise ValueError('invalid item width')
        if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in it[1:-1]):raise ValueError('invalid item points')
        if cursor is not None and math.dist(cursor,it[1])>eps:raise ValueError('interrupted item chain')
        if it[0]=='A':
            arc_geometry(it)
            if not native_arc_representable(it):raise ValueError('Arc degenerate/inverted on KiCad grid')
        cursor=it[2] if it[0]=='L' else it[3]
    if math.dist(items[0][1],P[0])>eps or math.dist(cursor,P[-1])>eps:raise ValueError('Items/preview ends differ')
    idx=0
    for a,b,w in zip(P,P[1:],W):
        if not math.isfinite(w) or w<=0 or len(a)!=2 or len(b)!=2 or not all(math.isfinite(v) for p in (a,b) for v in p):raise ValueError('invalid preview')
        if math.dist(a,b)<1e-10:continue
        while idx<len(items)-1 and math.dist(a,items[idx][2] if items[idx][0]=='L' else items[idx][3])<eps:idx+=1
        it=items[idx]
        if abs(w-it[-1])>1e-9:raise ValueError('Items/preview widths differ')
        if it[0]=='L':
            p,q=it[1:3];dx,dy=q[0]-p[0],q[1]-p[1];ll=dx*dx+dy*dy
            if ll<1e-16:raise ValueError('degenerate item straight segment')
            prev=-eps
            for v in (a,b):
                s=((v[0]-p[0])*dx+(v[1]-p[1])*dy)/ll
                if not -eps<=s<=1+eps or s<prev-eps or math.dist(v,(p[0]+s*dx,p[1]+s*dy))>eps:raise ValueError('Items/preview straight segments differ')
                prev=s
        else:
            c,r,ang,sweep=arc_geometry(it);sgn=1 if sweep>0 else -1;angles=[]
            for v in (a,b):
                if abs(math.dist(v,c)-r)>eps:raise ValueError('Items/preview arcs differ')
                d=(sgn*(math.atan2(v[1]-c[1],v[0]-c[0])-ang))%(2*math.pi)
                if d>2*math.pi-1e-7:d=0.
                if d>abs(sweep)+1e-7:raise ValueError('Preview outside the arc')
                angles.append(d)
            if angles[1]<angles[0]-1e-7 or r*(1-math.cos((angles[1]-angles[0])/2))>error+eps:raise ValueError('Preview arc too coarse/inverted')
    if idx!=len(items)-1:raise ValueError('Preview omits items')


def primitive_parameter(it, p, tolerance=1e-7):
    a = it[1]
    if it[0] == 'L':
        b = it[2]; dx,dy = b[0]-a[0],b[1]-a[1]; ll = dx*dx+dy*dy
        if ll < 1e-16: return None
        u = ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/ll
        if -tolerance <= u <= 1+tolerance and math.dist(p,(a[0]+u*dx,a[1]+u*dy)) <= tolerance:
            return min(1.,max(0.,u))
    elif it[0] == 'A':
        c,r,ang,sweep = arc_geometry(it)
        if abs(math.dist(p,c)-r) > tolerance: return None
        if math.dist(p,a) <= tolerance: return 0.
        if math.dist(p,it[3]) <= tolerance: return 1.
        d = ((1 if sweep>0 else -1)*(math.atan2(p[1]-c[1],p[0]-c[0])-ang))%(2*math.pi)
        if d <= abs(sweep)+tolerance/r: return min(1.,d/abs(sweep))
    else: raise ValueError('unknown track item: '+str(it[0]))
    return None


def primitive_intersections(a, b):
    candidates = [a[1],a[2] if a[0]=='L' else a[3],b[1],b[2] if b[0]=='L' else b[3]]
    if a[0] == b[0] == 'L':
        p,q = a[1:3]; r,s = b[1:3]
        u = (q[0]-p[0],q[1]-p[1]); v = (s[0]-r[0],s[1]-r[1])
        det = u[0]*v[1]-u[1]*v[0]
        if abs(det)>1e-14:
            t = ((r[0]-p[0])*v[1]-(r[1]-p[1])*v[0])/det
            candidates.append((p[0]+t*u[0],p[1]+t*u[1]))
    elif a[0]=='L' or b[0]=='L':
        lin,arc = (a,b) if a[0]=='L' else (b,a)
        c,r,_,_ = arc_geometry(arc); p,q = lin[1:3]
        u = (q[0]-p[0],q[1]-p[1]); ll = u[0]**2+u[1]**2
        if ll>1e-16:
            t = ((c[0]-p[0])*u[0]+(c[1]-p[1])*u[1])/ll
            foot = (p[0]+t*u[0],p[1]+t*u[1]); h2 = r*r-math.dist(foot,c)**2
            if h2>=-1e-12:
                h = math.sqrt(max(0.,h2)/ll)
                candidates.extend((p[0]+(t+d)*u[0],p[1]+(t+d)*u[1]) for d in (-h,h))
    else:
        c,r,_,_ = arc_geometry(a); d,s,_,_ = arc_geometry(b); ll = math.dist(c,d)
        if ll>1e-12 and abs(r-s)-1e-9 <= ll <= r+s+1e-9:
            x = (ll*ll+r*r-s*s)/(2*ll); h = math.sqrt(max(0.,r*r-x*x))
            u = ((d[0]-c[0])/ll,(d[1]-c[1])/ll)
            candidates.extend((c[0]+x*u[0]-k*h*u[1],c[1]+x*u[1]+k*h*u[0]) for k in (-1,1))
    return [tuple(p) for p in candidates if primitive_parameter(a,p) is not None and primitive_parameter(b,p) is not None]


def split_native_contacts(it, points):
    a,b = it[1],it[2] if it[0]=='L' else it[3]
    inner = sorted((u,tuple(p)) for p in points if (u:=primitive_parameter(it,p)) is not None and 1e-8<u<1-1e-8)
    chain = [(0.,a)]+inner+[(1.,b)]
    for (u,p),(v,q) in zip(chain,chain[1:]):
        if v-u<1e-8: continue
        if it[0]=='L': yield ['L',p,q,it[-1]]
        else:
            c,r,ang,sweep = arc_geometry(it); mid = ang+(u+v)/2*sweep
            yield ['A',p,(c[0]+r*math.cos(mid),c[1]+r*math.sin(mid)),q,it[-1]]
