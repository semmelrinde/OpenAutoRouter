import math
import numpy as np
import shapely
from shapely.geometry import Polygon

VERSION = 1
KINDS = ('sw', 'fast', 'current_val', 'supply', 'analog', 'signal', 'gnd')
CHANNELS = ('contour', 'keepout_tracks', 'keepout_vias', 'keepout_parts', 'pads',
          'pin_density_mm2', 'pinned', 'area_layer', 'rudy_sw', 'rudy_fast',
          'rudy_current', 'rudy_supply', 'rudy_analog', 'rudy_signal', 'rudy_gnd',
          'rudy_current_unknown', 'rudy_fast_unknown')
PART_COLUMNS = ('x_mm', 'y_mm', 'rot_deg', 'board_side', 'pinned', 'island', 'decoupling')
PAD_COLUMNS = ('x_mm', 'y_mm', 'part', 'net_name', 'drill_mm')
NET_COLUMNS = ('net_class', 'current_A', 'f_Hz', 'island', 'hpwl_mm', 'pads_n')


def cu_layers(B, RG=None):
    n = int(B.get('cu_layers') or (RG or {}).get('router', {}).get('cu_layers') or 2)
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def raster(B, origin=None, shape=None):
    xy = np.asarray([p for q in B['outline'] for p in q], float)
    lo = xy.min(0) - .3 if origin is None else np.asarray(origin, float)

    if shape is None:
        fine = np.ceil((xy.max(0) + .3 - lo) / .05).astype(int)
        shape = (int((fine[1] + 4) // 5), int((fine[0] + 4) // 5))
    return lo, tuple(shape)

def geometry(polys, holes=()):
    return shapely.union_all([Polygon(p, holes[i] if i < len(holes) else []).buffer(0)
                              for i, p in enumerate(polys) if len(p) >= 3])


def times(g, origin, shape):
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


def electrical(k):
    a = k.get('current_A')
    fl = k.get('flow') or {}
    if a is None and fl.get('unit') == 'A': a = fl.get('total')
    f = k.get('f_Hz')
    def amount(v):
        return float(v) if isinstance(v, (float, int)) and math.isfinite(v) and v >= 0 else np.nan
    return amount(a), amount(f)


def features_raster(B, K, RG=None, gnd_layers=None, origin=None, shape=None):
    ls = cu_layers(B, RG); li = {s: i for i, s in enumerate(ls)}
    origin, shape = raster(B, origin, shape); ny, nx = shape
    planes = gnd_layers
    if planes is None: planes = (RG or {}).get('router', {}).get('gnd_layers', [])
    if isinstance(planes, str): planes = planes.split(',')
    planes = {s.split('=')[0] for s in (planes or [])}
    ns = max(1, len(ls) - len(planes & set(ls)))
    X = np.zeros((len(ls), len(CHANNELS), ny, nx), np.float32)
    X[:, 0] = times(geometry(B['outline']), origin, shape)
    for l in planes & set(ls): X[li[l], 7] = 1
    for ko in B.get('keepouts', []):
        m = times(geometry(ko.get('polys', []), ko.get('hole_list') or []), origin, shape)
        bans = ko.get('prohibitions') or {'part_list': True}
        for l in ko.get('cu_layers') or ls:
            if l not in li: continue
            for key, ch in (('track_list', 1), ('vias', 2), ('part_list', 3)):
                if bans.get(key): X[li[l], ch] = np.maximum(X[li[l], ch], m)
    fs = sorted(B.get('fps', []), key=lambda f: f['ref'])
    refs = [f['ref'] for f in fs]; ri = {r: i for i, r in enumerate(refs)}
    islands = sorted(set(K.get('islands') or {}) | {t['island'] for t in (K.get('part_list') or {}).values() if t.get('island')})
    ii = {r: i for i, r in enumerate(islands)}
    parts = np.zeros((len(fs), len(PART_COLUMNS)), np.float32)
    for i, fp in enumerate(fs):
        t = (K.get('part_list') or {}).get(fp['ref'], {})
        parts[i] = [fp['x'], fp['y'], fp.get('rot', 0), li.get(fp.get('side', 'F'), 0),
                    bool(t.get('pinned') or fp.get('locked')), ii.get(t.get('island'), -1), bool(t.get('decoupling'))]
    for cy in B.get('courtyards', []):
        i = ri.get(cy.get('ref'))
        if i is not None and parts[i, 4]:
            l = cy.get('side', 'F')
            if l in li: X[li[l], 6] = np.maximum(X[li[l], 6], times(geometry(cy['polys']), origin, shape))
    pads = [p for p in B['pads'] if not p.get('net', '').startswith('#')]
    names = sorted({p.get('net', '') for p in pads}); ni = {n: i for i, n in enumerate(names)}
    pn = np.zeros((len(pads), len(PAD_COLUMNS)), np.float32)
    pl = np.zeros((len(pads), len(ls)), bool); by_net = {}
    for i, p in enumerate(pads):
        n = p.get('net', ''); by_net.setdefault(n, []).append(p)
        pn[i] = [p['x'], p['y'], ri.get(p['ref'], -1), ni[n], p.get('drill') or 0]
        x, y = np.floor((np.array([p['x'], p['y']]) - origin) / .25).astype(int)
        for l in p['layers']:
            if l not in li: continue
            j = li[l]; pl[i, j] = True
            if 0 <= x < nx and 0 <= y < ny: X[j, 5, y, x] += 16
            X[j, 4] = np.maximum(X[j, 4], times(geometry(p.get('polys', {}).get(l, [])), origin, shape))
    nn = np.zeros((len(names), len(NET_COLUMNS)), np.float32)
    nn[:, 1:3] = np.nan
    for n, i in ni.items():
        ps = by_net[n]; k = (K.get('net_list') or {}).get(n, {})
        category = 'gnd' if n in (K.get('gnd') or []) else k.get('category', 'signal')
        ci = KINDS.index(category) if category in KINDS else KINDS.index('signal')
        xy = np.asarray([[p['x'], p['y']] for p in ps]); a, b = xy.min(0), xy.max(0)
        hpwl = float((b - a).sum()); amp, freq = electrical(k)
        owners = {ii.get((K.get('part_list') or {}).get(p['ref'], {}).get('island'), -1) for p in ps}
        owner = next(iter(owners)) if len(owners) == 1 else -1
        nn[i] = [ci, amp, freq, owner, hpwl, len(ps)]
        if len(ps) < 2 or not n or n.startswith('unconnected'): continue
        size = np.maximum(b - a, .25)
        start = np.maximum(0, np.floor((a - origin) / .25).astype(int))
        end = np.minimum((nx, ny), np.floor((b - origin) / .25).astype(int) + 1)
        weight = 1.0; ch = 8 + ci
        if category in ('current_val', 'supply'):
            if np.isfinite(amp): weight = amp
            else: ch = 15
        if category == 'fast':
            if np.isfinite(freq): weight = freq / 1e6
            else: ch = 16
        demand = hpwl / float(np.prod(size)) / ns * weight
        for l, j in li.items():
            if l not in planes: X[j, ch, start[1]:end[1], start[0]:end[0]] += demand
    return dict(input_=X, channel_names=np.asarray(CHANNELS), origin_mm=origin, raster_mm=np.asarray(.25),
                cu_layers=np.asarray(ls), global_columns=np.asarray(['layers_n', 'area_layers_n', 'route_layers_n']), global_values=np.array([len(ls), len(planes & set(ls)), ns], np.float32),
                part_names=np.asarray(refs), part_columns=np.asarray(PART_COLUMNS), part_values=parts,
                pad_names=np.asarray([p['ref'] + ':' + p['num'] for p in pads]), pad_columns=np.asarray(PAD_COLUMNS), pad_values=pn, pad_layers=pl,
                net_names=np.asarray(names), net_columns=np.asarray(NET_COLUMNS), net_values=nn,
                net_netclasses=np.asarray(KINDS), island_names=np.asarray(islands), features_version=np.asarray(VERSION))


def substitute_ground(B, K, RG=None, gnd_layers=None):
    from scipy.ndimage import uniform_filter
    m = features_raster(B, K, RG, gnd_layers)
    hp = float(m['net_values'][:, 4].sum()); loops = 0.; dec = []
    ps = {}; fs = {f['ref']: f for f in B.get('fps', [])}
    for p in B['pads']: ps.setdefault(p['net'], []).append(p)
    for n, k in (K.get('net_list') or {}).items():
        if k.get('category') == 'sw' and ps.get(n):
            xy = np.asarray([[p['x'], p['y']] for p in ps[n]])
            loops += float(np.prod(np.ptp(xy, axis=0)))
    for owner, isl in (K.get('islands') or {}).items():
        for d in isl.get('decoupling', []):
            cp = [p for p in ps.get(d['net_name'], []) if p['ref'] == d['ref']]
            up = [p for p in ps.get(d['net_name'], []) if p['ref'] == owner and (not d.get('pins') or p['num'] in d['pins'])]
            if cp and up: dec.append(min(math.dist((p['x'], p['y']), (q['x'], q['y'])) for p in cp for q in up))
    courts = [(c['ref'], c.get('side', 'F'), geometry(c['polys'])) for c in B.get('courtyards', [])]
    collisions = sum(1 for i, (r, l, g) in enumerate(courts) for s, ll, h in courts[i + 1:]
                     if r != s and l == ll and g.intersection(h).area > 1e-8)
    pin = uniform_filter(m['input_'][:, 5], size=(1, 9, 9), mode='constant').max()
    return m, dict(hpwl_mm=hp, rudy_sharp=float(m['input_'][:, 8:].sum(1).max()), collisions=collisions,
                   sw_box_area_mm2=loops, decoupling_max_mm=max(dec) if dec else None,
                   decoupling_medium_mm=float(np.mean(dec)) if dec else None, pin_density_max_mm2=float(pin))
