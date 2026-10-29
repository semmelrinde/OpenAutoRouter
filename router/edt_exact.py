import numpy as np
from numba import njit
from scipy.ndimage import distance_transform_edt as scipy_edt

@njit(cache=True, nogil=True)
def _squared(mask):
    ny,nx=mask.shape
    far=float((nx+ny+1)**2)
    tmp=np.empty((ny,nx),np.float64)

    for y in range(ny):
        load=-nx-ny-1
        for x in range(nx):
            if not mask[y,x]: load=x
            d=x-load; tmp[y,x]=min(float(d*d),far)
        load=2*nx+ny+1
        for x in range(nx-1,-1,-1):
            if not mask[y,x]: load=x
            d=load-x
            if d*d<tmp[y,x]: tmp[y,x]=float(d*d)
    out=np.empty((ny,nx),np.float64)
    v=np.empty(ny,np.int64); z=np.empty(ny+1,np.float64)

    for x in range(nx):
        k=0;v[0]=0;z[0]=-np.inf;z[1]=np.inf
        for q in range(1,ny):
            s=((tmp[q,x]+q*q)-(tmp[v[k],x]+v[k]*v[k]))/(2.0*(q-v[k]))
            while s<=z[k]:
                k-=1
                s=((tmp[q,x]+q*q)-(tmp[v[k],x]+v[k]*v[k]))/(2.0*(q-v[k]))
            k+=1;v[k]=q;z[k]=s;z[k+1]=np.inf
        k=0
        for q in range(ny):
            while z[k+1]<q: k+=1
            d=q-v[k];out[q,x]=float(d*d)+tmp[v[k],x]
    return out

def distance_transform_edt(mask):
    a=np.asarray(mask,dtype=np.bool_)
    if a.ndim != 2 or not a.size or a.all():
        return scipy_edt(a)
    return np.sqrt(_squared(a))
