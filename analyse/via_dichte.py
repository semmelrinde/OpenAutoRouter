""
import json
import math
import sys
from collections import Counter

import numpy as np
from scipy.spatial import cKDTree
from shapely import contains_xy
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union
from shapely.strtree import STRtree

VERSION = 'via-dichte-v3'
FENSTER_MM = 2.0
SCHRITT_MM = 0.25


SCHWELLE = 1.25


def physische_vias(R):
    out = {}
    for v in R.get('vias', []):
        x, y = float(v['x']), float(v['y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Nichtendliche Via-Koordinate')
        key = (round(x, 6), round(y, 6), v.get('net', ''))
        if key in out and out[key].get('layers') != v.get('layers'):
            raise ValueError('Doppelte Via mit widerspruechlichem Lagenspann')
        if key not in out:
            out[key] = dict(v)
        else:


            for mark in ('gr', 'n', 'rv', 'rueck_via', 'strom_via', 'fix', 'kelvin'):
                if v.get(mark): out[key][mark] = max(out[key].get(mark) or 0, v[mark])
    return [out[k] for k in sorted(out)]


def spitze(xy, fenster=FENSTER_MM):
    ""
    if not len(xy): return 0, None
    xy = np.asarray(xy, dtype=float)
    best, ort = 0, None
    for x in np.unique(xy[:, 0]):
        ys = np.sort(xy[(xy[:, 0] >= x) & (xy[:, 0] <= x + fenster + 1e-9), 1])
        hi = np.searchsorted(ys, ys + fenster + 1e-9, side='right')
        counts = hi - np.arange(len(ys))
        i = int(np.argmax(counts))
        if int(counts[i]) > best:
            best, ort = int(counts[i]), [float(x + fenster / 2), float(ys[i] + fenster / 2)]
    return best, ort


def messen(B, R, schwelle=SCHWELLE, fenster=FENSTER_MM, schritt=SCHRITT_MM):
    if not all(math.isfinite(v) and v > 0 for v in (fenster, schritt)) or not math.isfinite(schwelle) or schwelle < 0:
        raise ValueError('Ungueltiger Via-Dichte-Messvertrag')
    vias = physische_vias(R)
    xy = np.asarray([(v['x'], v['y']) for v in vias], dtype=float).reshape(-1, 2)
    maximum, ort = spitze(xy, fenster)
    lagen = ['F'] + ['In%d' % i for i in range(1, int(B.get('lagen') or R.get('lagen') or 2) - 1)] + ['B']
    brd = unary_union([Polygon(p) for p in B['outline'] if len(p) >= 3])

    holes = B.get('outline_holes') or []
    rings = []
    for group in holes:
        if not group: continue
        if isinstance(group[0][0], (int, float)): rings.append(group)
        else: rings.extend(group)
    if rings:
        brd = brd.difference(unary_union([Polygon(p) for p in rings if len(p) >= 3]))
    innen = brd.buffer(1e-9)
    x0, y0, x1, y1 = brd.bounds
    xx = np.arange(x0, x1 + 1e-9, schritt)
    yy = np.arange(y0, y1 + 1e-9, schritt)
    tree = cKDTree(xy) if len(xy) else None
    hist, hot, n = {}, [], 0

    for y in yy:
        centers = np.column_stack((xx, np.full(len(xx), y)))
        centers = centers[contains_xy(innen, centers[:, 0], centers[:, 1])]
        counts = tree.query_ball_point(centers, fenster / 2 + 1e-9, p=np.inf, return_length=True) if tree else np.zeros(len(centers), int)
        n += len(centers)
        vals, nums = np.unique(counts, return_counts=True)
        for a, b in zip(vals, nums): hist[int(a)] = hist.get(int(a), 0) + int(b)
        hot.extend(centers[counts > schwelle * fenster ** 2].tolist())


    centers = hot + ([ort] if ort is not None else [])
    geometrien = [box(x - fenster / 2, y - fenster / 2, x + fenster / 2, y + fenster / 2) for x, y in centers]
    belegt = np.zeros(len(centers), int)
    from gemeinsam.zonen import boardgeometrie
    from lernen.kupferbalken import pad_polygon, track_geometry
    BB = boardgeometrie(B, R)
    for L in lagen:
        kupfer = [track_geometry(t) for t in R.get('tracks', []) if t['layer'] == L]
        for p in BB.get('pads', []):
            if p.get('pad_type') == 'npth': continue
            holes = [Polygon(q) for q in p.get('hole_polys', []) if len(q) >= 3]
            if not holes and p.get('drill'):


                holes = [Point(p['x'], p['y']).buffer(p['drill'] / (2 * math.cos(math.pi / 256)), quad_segs=64)]
            hole = unary_union(holes)
            for q in (p.get('polys') or {}).get(L, []):
                if len(q) >= 3:
                    g = pad_polygon(BB, p, q).difference(hole)
                    if not g.is_empty: kupfer.append(g)
        kupfer += [Polygon(z['pts'], z.get('holes') or z.get('loecher') or []) for z in R.get('zones', [])
                   if z.get('layer') == L and z.get('filled') and len(z.get('pts', [])) >= 3]
        kupfer += [Polygon(q['aussen'], q.get('innen') or []) for z in R.get('zonen', []) if z.get('layer') == L
                   for q in z.get('polys', []) if len(q.get('aussen', [])) >= 3]
        if kupfer and geometrien:
            pairs = STRtree(kupfer).query(geometrien, predicate='intersects')
            belegt[np.unique(pairs[0])] += 1
    spans, nets = set(), Counter()
    if ort is not None:
        for i in tree.query_ball_point(ort, fenster / 2 + 1e-9, p=np.inf):
            spans.update(vias[i].get('layers') or lagen)
            nets[vias[i].get('net', '')] += 1
    excess = sum(max(0, k - schwelle * fenster ** 2) * v for k, v in hist.items())
    return dict(version=VERSION, fenster_mm=fenster, schritt_mm=schritt, schwelle_vias_mm2=schwelle,
                vias=len(vias), spitze_vias=maximum, spitze_vias_mm2=maximum / fenster ** 2, spitze_xy=ort,
                fenster=n, fenster_ueber=len(hot), anteil_fenster_ueber=len(hot) / max(1, n),
                ueberlast_integral_mm2=round(excess * schritt ** 2, 6),
                spitze_kupferlagen_anteil=float(belegt[-1] / len(lagen)) if ort is not None else 0.0,
                heiss_kupferlagen_anteil=float(np.mean(belegt[:len(hot)]) / len(lagen)) if hot else 0.0,
                spitze_via_lagen_anteil=len(spans.intersection(lagen)) / len(lagen),
                spitze_netze=dict(sorted(nets.items())),
                kupferlagen_modell='Segment-/Bogenbreiten, Padkonturen ohne Bohrung, gefuellte zones und Vollkupfermodell zonen; ohne Via-Ringe',
                pad_bohrannahme='Kreis fuer fehlende Bohrkontur' if any(p.get('drill') and not p.get('hole_polys') for p in BB.get('pads', [])) else None,
                via_lagen_annahme='Durchgangsvia fuer fehlenden Lagenspann' if any(not v.get('layers') for v in vias) else None,
                histogramm={str(k): hist[k] for k in sorted(hist)})


def schutznetze(K, R=None, impedanz=None):
    ""
    from analyse.signalintegritaet import vorgabe
    nets = K.get('netze') or {}
    protected = set((R or {}).get('impedanz_tabelle') or {}) | set(impedanz or {})
    for group in ((K.get('vorgaben') or {}).get('laengengruppen') or {}).values():
        protected.update(group['netze'])
    for n, k in nets.items():
        if k.get('art') not in ('signal', 'analog', 'schnell'):
            protected.add(n)


        for field in ('quarz', 'paar', 'z_ohm', 'z_diff_ohm', 'laengengruppe', 'kelvin', 'fix'):
            for value in (k.get(field), vorgabe(K, n, field)):
                if value:
                    protected.add(n)
                    if field == 'paar': protected.add(value)
    return protected


def via_geschuetzt(v, K, netze=None):
    ""
    if netze is None: netze = schutznetze(K)
    n = v.get('net')
    return (n in netze or (K.get('netze', {}).get(n) or {}).get('art') not in ('signal', 'analog', 'schnell')
            or any(v.get(k) for k in ('gr', 'n', 'rv', 'rueck_via', 'strom_via', 'fix', 'kelvin')))

def kurz(R, K, fenster=FENSTER_MM, schwelle=SCHWELLE):
    ""
    if not math.isfinite(fenster) or fenster <= 0 or not math.isfinite(schwelle) or schwelle < 0:
        raise ValueError('Ungueltiger R40-Messvertrag')
    vs = physische_vias(R)
    protected = schutznetze(K, R)
    signal = [v for v in vs if not via_geschuetzt(v, K, protected)]
    def peak(vs): return spitze([(v['x'], v['y']) for v in vs], fenster)[0] / fenster ** 2
    xy = np.asarray([(v['x'], v['y']) for v in vs], dtype=float).reshape(-1, 2)


    counts = (cKDTree(xy).query_ball_point([(v['x'], v['y']) for v in signal], fenster / 2 + 1e-9,
                                         p=np.inf, return_length=True) if signal else np.array([], int))
    return dict(version='r40-kurz-v2', fenster_mm=fenster, schwelle_vias_mm2=schwelle,
                vias=len(vs), signal_vias=len(signal), geschuetzt_vias=len(vs)-len(signal),
                spitze_vias_mm2=peak(vs), signal_spitze_vias_mm2=peak(signal),
                signal_heiss_vias=int(np.sum(counts > schwelle * fenster ** 2)),
                signal_ueberlast=float(np.maximum(0, counts - schwelle * fenster ** 2).sum()),
                ausfahrt_blockiert=None)


def kalibrieren(messungen):
    ""
    ms = list(messungen)
    if not ms: raise ValueError('Keine menschlichen Referenzen')
    if any(m['fenster_mm'] != FENSTER_MM for m in ms): raise ValueError('Anderes Fenster')
    peaks = sorted(m['spitze_vias_mm2'] for m in ms)
    return dict(version=VERSION, fenster_mm=FENSTER_MM, referenzen=len(ms), methode='Board-Spitzen p95 nearest-rank',
                schwelle_vias_mm2=peaks[math.ceil(0.95 * len(peaks)) - 1], spitzen=peaks)
if __name__ == '__main__':
    B, R = (json.load(open(p)) for p in sys.argv[1:3])
    kw = dict(a.split('=', 1) for a in sys.argv[3:])
    print(json.dumps(messen(B, R, **{k: float(v) for k, v in kw.items()}), indent=1))
