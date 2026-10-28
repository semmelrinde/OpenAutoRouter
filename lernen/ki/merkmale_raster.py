""
import math
import numpy as np
import shapely
from shapely.geometry import Polygon

VERSION = 1
ARTEN = ('sw', 'schnell', 'strom', 'versorgung', 'analog', 'signal', 'gnd')
KANALE = ('umriss', 'sperre_bahnen', 'sperre_vias', 'sperre_teile', 'pads',
          'pin_dichte_mm2', 'fest', 'flaechenlage', 'rudy_sw', 'rudy_schnell',
          'rudy_strom', 'rudy_versorgung', 'rudy_analog', 'rudy_signal', 'rudy_gnd',
          'rudy_strom_unbekannt', 'rudy_schnell_unbekannt')
TEIL_SPALTEN = ('x_mm', 'y_mm', 'rot_deg', 'seite', 'fest', 'insel', 'entkopplung')
PAD_SPALTEN = ('x_mm', 'y_mm', 'teil', 'netz', 'drill_mm')
NETZ_SPALTEN = ('klasse', 'strom_A', 'f_Hz', 'insel', 'hpwl_mm', 'pads_n')


def lagen(B, RG=None):
    n = int(B.get('lagen') or (RG or {}).get('router', {}).get('lagen') or 2)
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def raster(B, origin=None, shape=None):
    xy = np.asarray([p for q in B['outline'] for p in q], float)
    lo = xy.min(0) - .3 if origin is None else np.asarray(origin, float)

    if shape is None:
        fine = np.ceil((xy.max(0) + .3 - lo) / .05).astype(int)
        shape = (int((fine[1] + 4) // 5), int((fine[0] + 4) // 5))
    return lo, tuple(shape)

def geometrie(polys, holes=()):
    return shapely.union_all([Polygon(p, holes[i] if i < len(holes) else []).buffer(0)
                              for i, p in enumerate(polys) if len(p) >= 3])


def mal(g, origin, shape):
    ""
    out = np.zeros(shape, bool)
    if g.is_empty: return out
    x0, y0, x1, y1 = g.bounds
    a = np.maximum(0, np.floor((np.array([x0, y0]) - origin) / .25).astype(int))
    b = np.minimum(shape[::-1], np.floor((np.array([x1, y1]) - origin) / .25).astype(int) + 1)
    if np.any(b <= a): return out
    yy, xx = np.mgrid[a[1]:b[1], a[0]:b[0]]
    x = origin[0] + xx * .25; y = origin[1] + yy * .25
    boxes = shapely.box(x, y, x + .25, y + .25)

    out[a[1]:b[1], a[0]:b[0]] = shapely.area(shapely.intersection(g, boxes)) > 1e-12
    return out


def elektrisch(k):
    a = k.get('strom_A')
    fl = k.get('fluss') or {}
    if a is None and fl.get('einheit') == 'A': a = fl.get('gesamt')
    f = k.get('f_Hz')
    def wert(v):
        return float(v) if isinstance(v, (float, int)) and math.isfinite(v) and v >= 0 else np.nan
    return wert(a), wert(f)


def merkmale_raster(B, K, RG=None, gndlagen=None, origin=None, shape=None):
    ls = lagen(B, RG); li = {s: i for i, s in enumerate(ls)}
    origin, shape = raster(B, origin, shape); ny, nx = shape
    planes = gndlagen
    if planes is None: planes = (RG or {}).get('router', {}).get('gndlagen', [])
    if isinstance(planes, str): planes = planes.split(',')
    planes = {s.split('=')[0] for s in (planes or [])}
    ns = max(1, len(ls) - len(planes & set(ls)))
    X = np.zeros((len(ls), len(KANALE), ny, nx), np.float32)
    X[:, 0] = mal(geometrie(B['outline']), origin, shape)
    for l in planes & set(ls): X[li[l], 7] = 1
    for ko in B.get('keepouts', []):
        m = mal(geometrie(ko.get('polys', []), ko.get('loecher') or []), origin, shape)
        bans = ko.get('verbote') or {'teile': True}
        for l in ko.get('lagen') or ls:
            if l not in li: continue
            for key, ch in (('bahnen', 1), ('vias', 2), ('teile', 3)):
                if bans.get(key): X[li[l], ch] = np.maximum(X[li[l], ch], m)
    fs = sorted(B.get('fps', []), key=lambda f: f['ref'])
    refs = [f['ref'] for f in fs]; ri = {r: i for i, r in enumerate(refs)}
    islands = sorted(set(K.get('inseln') or {}) | {t['insel'] for t in (K.get('teile') or {}).values() if t.get('insel')})
    ii = {r: i for i, r in enumerate(islands)}
    parts = np.zeros((len(fs), len(TEIL_SPALTEN)), np.float32)
    for i, fp in enumerate(fs):
        t = (K.get('teile') or {}).get(fp['ref'], {})
        parts[i] = [fp['x'], fp['y'], fp.get('rot', 0), li.get(fp.get('side', 'F'), 0),
                    bool(t.get('fest') or fp.get('locked')), ii.get(t.get('insel'), -1), bool(t.get('entkopplung'))]
    for cy in B.get('courtyards', []):
        i = ri.get(cy.get('ref'))
        if i is not None and parts[i, 4]:
            l = cy.get('side', 'F')
            if l in li: X[li[l], 6] = np.maximum(X[li[l], 6], mal(geometrie(cy['polys']), origin, shape))
    pads = [p for p in B['pads'] if not p.get('net', '').startswith('#')]
    names = sorted({p.get('net', '') for p in pads}); ni = {n: i for i, n in enumerate(names)}
    pn = np.zeros((len(pads), len(PAD_SPALTEN)), np.float32)
    pl = np.zeros((len(pads), len(ls)), bool); bynet = {}
    for i, p in enumerate(pads):
        n = p.get('net', ''); bynet.setdefault(n, []).append(p)
        pn[i] = [p['x'], p['y'], ri.get(p['ref'], -1), ni[n], p.get('drill') or 0]
        x, y = np.floor((np.array([p['x'], p['y']]) - origin) / .25).astype(int)
        for l in p['layers']:
            if l not in li: continue
            j = li[l]; pl[i, j] = True
            if 0 <= x < nx and 0 <= y < ny: X[j, 5, y, x] += 16
            X[j, 4] = np.maximum(X[j, 4], mal(geometrie(p.get('polys', {}).get(l, [])), origin, shape))
    nn = np.zeros((len(names), len(NETZ_SPALTEN)), np.float32)
    nn[:, 1:3] = np.nan
    for n, i in ni.items():
        ps = bynet[n]; k = (K.get('netze') or {}).get(n, {})
        art = 'gnd' if n in (K.get('gnd') or []) else k.get('art', 'signal')
        ci = ARTEN.index(art) if art in ARTEN else ARTEN.index('signal')
        xy = np.asarray([[p['x'], p['y']] for p in ps]); a, b = xy.min(0), xy.max(0)
        hpwl = float((b - a).sum()); amp, freq = elektrisch(k)
        owners = {ii.get((K.get('teile') or {}).get(p['ref'], {}).get('insel'), -1) for p in ps}
        owner = next(iter(owners)) if len(owners) == 1 else -1
        nn[i] = [ci, amp, freq, owner, hpwl, len(ps)]
        if len(ps) < 2 or not n or n.startswith('unconnected'): continue
        size = np.maximum(b - a, .25)
        start = np.maximum(0, np.floor((a - origin) / .25).astype(int))
        end = np.minimum((nx, ny), np.floor((b - origin) / .25).astype(int) + 1)
        weight = 1.0; ch = 8 + ci
        if art in ('strom', 'versorgung'):
            if np.isfinite(amp): weight = amp
            else: ch = 15
        if art == 'schnell':
            if np.isfinite(freq): weight = freq / 1e6
            else: ch = 16
        demand = hpwl / float(np.prod(size)) / ns * weight
        for l, j in li.items():
            if l not in planes: X[j, ch, start[1]:end[1], start[0]:end[0]] += demand
    return dict(eingang=X, kanal_namen=np.asarray(KANALE), origin_mm=origin, raster_mm=np.asarray(.25),
                lagen=np.asarray(ls), global_spalten=np.asarray(['lagen_n', 'flaechenlagen_n', 'routlagen_n']), global_werte=np.array([len(ls), len(planes & set(ls)), ns], np.float32),
                teil_namen=np.asarray(refs), teil_spalten=np.asarray(TEIL_SPALTEN), teil_werte=parts,
                pad_namen=np.asarray([p['ref'] + ':' + p['num'] for p in pads]), pad_spalten=np.asarray(PAD_SPALTEN), pad_werte=pn, pad_lagen=pl,
                netz_namen=np.asarray(names), netz_spalten=np.asarray(NETZ_SPALTEN), netz_werte=nn,
                netz_klassen=np.asarray(ARTEN), insel_namen=np.asarray(islands), merkmale_version=np.asarray(VERSION))


def ersatzmasse(B, K, RG=None, gndlagen=None):
    ""
    from scipy.ndimage import uniform_filter
    m = merkmale_raster(B, K, RG, gndlagen)
    hp = float(m['netz_werte'][:, 4].sum()); loops = 0.; dec = []
    ps = {}; fs = {f['ref']: f for f in B.get('fps', [])}
    for p in B['pads']: ps.setdefault(p['net'], []).append(p)
    for n, k in (K.get('netze') or {}).items():
        if k.get('art') == 'sw' and ps.get(n):
            xy = np.asarray([[p['x'], p['y']] for p in ps[n]])
            loops += float(np.prod(np.ptp(xy, axis=0)))
    for owner, isl in (K.get('inseln') or {}).items():
        for d in isl.get('entkopplung', []):
            cp = [p for p in ps.get(d['netz'], []) if p['ref'] == d['ref']]
            up = [p for p in ps.get(d['netz'], []) if p['ref'] == owner and (not d.get('pins') or p['num'] in d['pins'])]
            if cp and up: dec.append(min(math.dist((p['x'], p['y']), (q['x'], q['y'])) for p in cp for q in up))
    courts = [(c['ref'], c.get('side', 'F'), geometrie(c['polys'])) for c in B.get('courtyards', [])]
    collisions = sum(1 for i, (r, l, g) in enumerate(courts) for s, ll, h in courts[i + 1:]
                     if r != s and l == ll and g.intersection(h).area > 1e-8)
    pin = uniform_filter(m['eingang'][:, 5], size=(1, 9, 9), mode='constant').max()
    return m, dict(hpwl_mm=hp, rudy_spitze=float(m['eingang'][:, 8:].sum(1).max()), kollisionen=collisions,
                   sw_box_flaeche_mm2=loops, entkopplung_max_mm=max(dec) if dec else None,
                   entkopplung_mittel_mm=float(np.mean(dec)) if dec else None, pin_dichte_max_mm2=float(pin))
