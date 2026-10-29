import numpy as np


def load_data(path, context, layers, origin, nx, ny, res):
    with np.load(path, allow_pickle=False) as d:
        if not context or str(d['hist_context']) != context:
            raise ValueError('hist_start: different task/rules')
        if list(d['cu_layers']) != list(layers) or not np.array_equal(d['origin_mm'], origin):
            raise ValueError('hist_start: different layers/coordinates')
        step = float(d['raster_mm'])
        ratio = round(step / res)
        if ratio != 5 or abs(step - ratio * res) > 1e-9:
            raise ValueError('hist_start: different grid')
        a = d['conflict_rounds'].astype(np.float32)
        if a.shape != (len(layers), (ny+4)//5, (nx+4)//5):
            raise ValueError('hist_start: different map size')
        if not np.isfinite(a).all() or (a < 0).any():
            raise ValueError('hist_start: invalid conflict costs')
        return (np.repeat(np.repeat(np.minimum(a, 32)*.5, 5, 1), 5, 2)[:, :ny, :nx]).ravel()
