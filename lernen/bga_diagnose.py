""
from pathlib import Path

import numpy as np
from shapely import contains_xy
from shapely.geometry import MultiPoint
from shapely.ops import unary_union


def klassifizieren(board, etiketten):
    refs = {f['ref'] for f in board.get('fps', [])
            if 'BGA' in (f.get('fp', '') + f.get('lib', '')).upper()}
    if not refs:
        return dict(zustand='kein_bga')
    felder = []
    for ref in sorted(refs):
        ps = [p for p in board['pads'] if p['ref'] == ref]
        if ps:
            felder.append(MultiPoint([(p['x'], p['y']) for p in ps]).convex_hull)
    if not felder:
        return dict(zustand='unbekannt', grund='BGA-Pads fehlen')
    grid = etiketten.get('router_konflikt_runden', etiketten.get('konflikt_runden'))
    if grid is None:
        return dict(zustand='unbekannt', grund='Konfliktetiketten fehlen')
    origin = etiketten.get('router_origin_mm', etiketten.get('origin_mm'))
    raster = float(etiketten.get('router_raster_mm', etiketten.get('raster_mm', .25)))
    grid = np.asarray(grid)
    yy, xx = np.mgrid[:grid.shape[-2], :grid.shape[-1]]
    x = origin[0] + (xx + .5) * raster
    y = origin[1] + (yy + .5) * raster
    feld = unary_union(felder)
    innen = contains_xy(feld.buffer(1e-9), x, y)
    nahe = contains_xy(feld.buffer(3), x, y)
    counts = {'ballfeld': int(grid[:, innen].sum()),
              'rand_3mm': int(grid[:, nahe & ~innen].sum()),
              'ausserhalb': int(grid[:, ~nahe].sum())}
    total = sum(counts.values())
    cause = max(counts, key=counts.get) if total else 'kein_konfliktetikett'
    return dict(zustand='ok', bga_refs=sorted(refs), konflikt_zellrunden=counts,
                gesamt=total, anteil_3mm=(counts['ballfeld'] + counts['rand_3mm']) / total if total else None,
                ortsklasse=cause, raster_mm=raster,
                bedeutung='kumulierte Konfliktzellrunden, keine finale DRC-Ursache')


def aus_dateien(board, etiketten):
    if not Path(etiketten).exists():
        return dict(zustand='unbekannt', grund='Konfliktetiketten fehlen')
    with np.load(etiketten, allow_pickle=False) as d:
        return klassifizieren(board, d)
