from shapely.geometry import Polygon, box
from shapely.ops import unary_union

def impossible(board):
    edges=board.get('outline',[])
    if (len(edges)!=1 or board.get('outline_holes') or board.get('drc_model')
            or board.get('copper_graphic') or board.get('net_ties')
            or any(str(f.get('ref','')).startswith('NT') for f in board.get('fps',[]))): return []
    boundary=Polygon(edges[0])
    if boundary.is_empty or not boundary.is_valid or not boundary.equals(boundary.envelope): return []
    inner=boundary.buffer(-0.2,join_style=2)
    xmin,ymin,xmax,ymax=boundary.bounds
    fps={f['ref']:f for f in board['fps']}
    courtyards={}
    for c in board.get('courtyards',[]):
        if c.get('side')!='F' or len(c.get('polys',[]))!=1: continue
        p=Polygon(c['polys'][0])
        if p.is_empty or not p.is_valid or p.area < .001001 or not p.equals(p.envelope): continue
        courtyards[c['ref']]=p
    fixed={r for r,f in fps.items() if f.get('locked') and r in courtyards}
    pads_by_ref={}
    for pad in board.get('pads',[]): pads_by_ref.setdefault(pad['ref'],[]).append(pad)
    result=[]
    for r in sorted(courtyards.keys()-fixed):
        f=fps[r]; p=courtyards[r]
        if not inner.covers(p): continue
        if any(pad.get('mask') for pad in pads_by_ref.get(r,[])): continue
        if any(not inner.covers(Polygon(poly).buffer(0.075,join_style=2))
               for pad in pads_by_ref.get(r,[]) for polys in pad.get('polys',{}).values()
               for poly in polys if len(poly)>=3):
            continue
        a,b,c,d=p.bounds
        w,h=c-a,d-b
        if not w or not h: continue
        no_space=True
        for width,height in ((w,h),(h,w)):
            ex=.001001/height+1e-7; ey=.001001/width+1e-7
            lowx=xmin+width/2-ex; highx=xmax-width/2+ex
            lowy=ymin+height/2-ey; highy=ymax-height/2+ey
            if lowx>highx or lowy>highy: continue
            if highx-lowx < 1e-9 or highy-lowy < 1e-9:
                no_space=False; break
            region=box(lowx,lowy,highx,highy)
            obstacles=[]
            for q in sorted(fixed):
                l,t,rr,bb=courtyards[q].bounds
                obstacle=box(l-width/2,t-height/2,rr+width/2,bb+height/2).buffer(-1e-7,join_style=2)
                if not obstacle.is_empty: obstacles.append(obstacle)
            if not obstacles or not region.difference(unary_union(obstacles)).is_empty:
                no_space=False; break
        if no_space: result.append(r)
    return result
