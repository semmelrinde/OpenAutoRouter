import math
import numpy as np
from PIL import Image, ImageDraw
from scipy.signal import fftconvolve


def search(r,st0,bel,inner,geometry,problem,ths,raster=.1,n_max=4000,blocked=None):
    x0,y0,_,sd=st0;bx=inner.bounds
    nx=int((bx[2]-bx[0])/raster)+1;ny=int((bx[3]-bx[1])/raster)+1
    def draw(d,g,fn,fill):
        for p in getattr(g,'geoms',[g]):
            if p.is_empty or not hasattr(p,'exterior'): continue
            d.polygon([fn(*q) for q in p.exterior.coords],fill=fill)
            for h in p.interiors:d.polygon([fn(*q) for q in h.coords],fill=1-fill)
    px=lambda x,y:((x-bx[0])/raster,(y-bx[1])/raster)
    free={};best=None
    for th in ths:
        gs=geometry(0,0,th,sd)
        b=(min(g.bounds[0] for g in gs.values()),min(g.bounds[1] for g in gs.values()),
           max(g.bounds[2] for g in gs.values()),max(g.bounds[3] for g in gs.values()))
        mx=int(math.ceil((b[2]-b[0])/raster))+3;my=int(math.ceil((b[3]-b[1])/raster))+3
        if mx>nx or my>ny:continue
        fits=None
        for side,g in gs.items():
            if side not in free:
                in_=Image.new('1',(nx,ny),0);d=ImageDraw.Draw(in_);draw(d,inner,px,1)
                for ref,occupied in bel.items.get(side,[]):
                    if ref!=r:draw(d,occupied,px,0)
                if blocked and side in blocked:draw(d,blocked[side][0],px,0)
                free[side]=np.asarray(in_,dtype=np.float64)
            mask=Image.new('1',(mx,my),0);draw(ImageDraw.Draw(mask),g,lambda x,y:((x-b[0])/raster+1,(y-b[1])/raster+1),1)
            m=np.asarray(mask,dtype=np.float64);need=m.sum()

            ok=fftconvolve(free[side],m[::-1,::-1],mode='valid')>=need-.01
            fits=ok if fits is None else fits&ok
        ys,xs=np.nonzero(fits)
        xx=bx[0]+(xs+1.5)*raster-b[0];yy=bx[1]+(ys+1.5)*raster-b[1]
        dist=np.hypot(xx-x0,yy-y0)
        for j in np.argsort(dist)[:n_max]:
            st=(float(xx[j]),float(yy[j]),th,sd)
            if problem(r,st,bel):continue
            cost=float(dist[j])+.2*(th!=st0[2])
            if best is None or cost<best[0]:best=(cost,st)
            break
    return best
