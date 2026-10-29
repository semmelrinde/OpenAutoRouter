import math
from numba import njit


@njit(cache=True, nogil=True, boundscheck=True)
def line_cap_min_kernel(cap, x0, y0, x1, y1, n, X0, Y0, RES):
    step = 1.0 / (n - 1)
    best = math.inf
    for k in range(n):
        t = 1.0 if k == n - 1 else k * step
        ix = int((x0 + t * (x1 - x0) - X0) / RES)
        iy = int((y0 + t * (y1 - y0) - Y0) / RES)
        value = float(cap[iy, ix])
        if math.isnan(value): return value
        if value < best: best = value
    return best
