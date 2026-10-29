import math


def points(a, m, b, faults=0.00025):
    ax, ay = a; mx, my = m; bx, by = b
    ux,uy=mx-ax,my-ay;vx,vy=bx-ax,by-ay
    d=2*(ux*vy-uy*vx)
    if abs(d) < 1e-14: return [a, m, b]
    uu,vv=ux*ux+uy*uy,vx*vx+vy*vy
    cx=ax+(uu*vy-vv*uy)/d
    cy=ay+(ux*vv-vx*uu)/d
    r = math.hypot(ax-cx, ay-cy)
    theta = [math.atan2(p[1]-cy, p[0]-cx) for p in (a,m,b)]
    sg = 1 if (mx-ax)*(by-my)-(my-ay)*(bx-mx) > 0 else -1
    step = 2 * math.acos(max(-1, min(1, 1-faults/r)))
    out = []
    for p,q,u,v in ((a,m,theta[0],theta[1]), (m,b,theta[1],theta[2])):
        sweep = sg * ((sg*(v-u)) % (2*math.pi))
        n = max(1, math.ceil(abs(sweep)/max(step, 1e-10)))
        ps = [[cx+r*math.cos(u+sweep*k/n), cy+r*math.sin(u+sweep*k/n)] for k in range(n+1)]
        ps[0],ps[-1]=list(p),list(q)
        out.extend(ps if not out else ps[1:])
    return out
