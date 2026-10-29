from shapely.geometry import Polygon, box


def hull(g):
    if g.is_empty:return g
    points=list(g.convex_hull.exterior.coords)
    sums=[x+y for x,y in points];diffs=[x-y for x,y in points]
    lo,hi=min(sums),max(sums);dl,dh=min(diffs),max(diffs)
    diagonal=Polygon([((u+v)/2,(u-v)/2) for u,v in ((lo,dl),(lo,dh),(hi,dh),(hi,dl))])
    return box(*g.bounds).intersection(diagonal)
