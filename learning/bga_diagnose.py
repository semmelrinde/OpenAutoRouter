from pathlib import Path

import numpy as np
from shapely import contains_xy
from shapely.geometry import MultiPoint
from shapely.ops import unary_union


def classify(board, labels):
    refs = {f['ref'] for f in board.get('fps', [])
            if 'BGA' in (f.get('fp', '') + f.get('lib', '')).upper()}
    if not refs:
        return dict(cur_state='no_bga')
    fields = []
    for ref in sorted(refs):
        ps = [p for p in board['pads'] if p['ref'] == ref]
        if ps:
            fields.append(MultiPoint([(p['x'], p['y']) for p in ps]).convex_hull)
    if not fields:
        return dict(cur_state='unknown', reason='BGA pads missing')
    grid = labels.get('router_conflict_rounds', labels.get('conflict_rounds'))
    if grid is None:
        return dict(cur_state='unknown', reason='Conflict labels missing')
    origin = labels.get('router_origin_mm', labels.get('origin_mm'))
    raster = float(labels.get('router_raster_mm', labels.get('raster_mm', .25)))
    grid = np.asarray(grid)
    yy, xx = np.mgrid[:grid.shape[-2], :grid.shape[-1]]
    x = origin[0] + (xx + .5) * raster
    y = origin[1] + (yy + .5) * raster
    fld = unary_union(fields)
    inner = contains_xy(fld.buffer(1e-9), x, y)
    nearby = contains_xy(fld.buffer(3), x, y)
    counts = {'ball_array': int(grid[:, inner].sum()),
              'border_3mm': int(grid[:, nearby & ~inner].sum()),
              'outside': int(grid[:, ~nearby].sum())}
    total = sum(counts.values())
    cause = max(counts, key=counts.get) if total else 'no_conflict_label'
    return dict(cur_state='ok', bga_refs=sorted(refs), conflict_cell_rounds=counts,
                total=total, share_3mm=(counts['ball_array'] + counts['border_3mm']) / total if total else None,
                place_class=cause, raster_mm=raster,
                meaning='accumulated conflict cell rounds, no final DRC cause')


def off_files(board, labels):
    if not Path(labels).exists():
        return dict(cur_state='unknown', reason='Conflict labels missing')
    with np.load(labels, allow_pickle=False) as d:
        return classify(board, d)
