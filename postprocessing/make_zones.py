import json, sys
from shapely.geometry import Polygon, LineString, Point, mapping
from shapely.ops import unary_union

D = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); OUT = sys.argv[3]
POWER = {}
PRIO = {}
import os
NC = json.load(open(os.environ['NETCLASSES'])) if os.environ.get('NETCLASSES') else None
GNDN = '/GND'
if NC:
    _B = {'high': 0.9, 'medium': 0.6, 'small': 0.4}; _P = {'high': 6, 'medium': 5, 'small': 3}
    POWER = {n: _B[k['current_val']] for n, k in NC['net_list'].items() if k['category'] in ('current_val', 'supply') and k.get('current_val')}
    PRIO = {n: _P[k['current_val']] for n, k in NC['net_list'].items() if k['category'] in ('current_val', 'supply') and k.get('current_val')}
    GNDN = NC.get('gnd_main') or GNDN
EDGEZ = float(os.environ.get('ZONE_EDGE', '0.3'))
outline = unary_union([Polygon(o) for o in D['outline']]).buffer(-EDGEZ)
zones = []
import shapely
from shapely.geometry import MultiPoint
FAIR = os.environ.get('ZONE_FAIR', '1') == '1'
ZG = float(os.environ.get('ZONE_GAP', '0.2'))
PRECEDENCE = os.environ.get('ZONE_PRECEDENCE', '0') == '1'
LAYS = sorted({t['layer'] for t in R['tracks']} | {'F', 'B'})


PLANAR = os.environ.get('ZONE_SHAPE', 'track') == 'polygon' or os.environ.get('ZONE_ENDS', 'planar') == 'planar'; VIA_BORDER = float(os.environ.get('ZONE_VIA_BORDER', '0.25'))


ROUND = float(os.environ.get('ZONE_ROUND', '0.5')); ROUND_A = float(os.environ.get('ZONE_ROUND_OUTER', '0.15'))
def track_area(pts, ws, buf):
    out = []
    for a, b, w in zip(pts[:-1], pts[1:], ws):
        out.append(LineString([a, b]).buffer(w / 2 + buf, cap_style='flat' if PLANAR else 'round'))
    if PLANAR:
        for i in range(1, len(pts) - 1):
            out.append(Point(pts[i]).buffer(max(ws[i - 1], ws[i]) / 2 + buf))
    return [g for g in out if not g.is_empty]
CAND = {}
COPPER = {}
for net, buf in POWER.items():
    for L in LAYS:
        geo = []; cu = []
        for t in R['tracks']:
            if t['net'] == net and t['layer'] == L and not t.get('sig'):
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                    cu.append(LineString([a, b]).buffer(w / 2))
                geo += track_area(t['pts'], t['w'], buf)
        for p in D['pads']:
            if p['net'] == net and L in p['layers']:
                for pts in p['polys'][L]: geo.append(Polygon(pts).buffer(min(buf, 0.4)))
        ends = [tuple(q) for t in R['tracks'] if t['net'] == net and t['layer'] == L for q in (t['pts'][0], t['pts'][-1])]
        for v in R['vias']:
            if v['net'] == net and any(abs(v['x'] - x) < 0.06 and abs(v['y'] - y) < 0.06 for x, y in ends):
                geo.append(Point(v['x'], v['y']).buffer(0.175 + (min(buf, VIA_BORDER) if PLANAR else buf)))
        for p in D['pads']:
            if p['net'] == net and L in p['layers']:
                for pts in p['polys'][L]: cu.append(Polygon(pts))
        if not geo: continue
        CAND[(net, L)] = unary_union(geo).intersection(outline)
        COPPER[(net, L)] = unary_union(cu) if cu else unary_union(geo).buffer(-buf)

for L in LAYS:
    keys = [k for k in CAND if k[1] == L]
    region = {}
    if FAIR and len(keys) > 1:
        pts, own = [], []
        for k in keys:
            g = COPPER[k]
            for part in getattr(g, 'geoms', [g]):
                if part.is_empty: continue
                ring = part.exterior
                n = max(4, int(ring.length / 0.1))
                for i in range(n):
                    q = ring.interpolate(i / n, normalized=True); pts.append((q.x, q.y)); own.append(k)
        seen = {}; P2 = []; O2 = []
        for q, k in zip(pts, own):
            key = (round(q[0], 4), round(q[1], 4))
            if key in seen: continue
            seen[key] = 1; P2.append(q); O2.append(k)
        cells = shapely.voronoi_polygons(MultiPoint(P2), ordered=True, extend_to=outline.buffer(2))
        by = {}
        for c, k in zip(cells.geoms, O2): by.setdefault(k, []).append(shapely.make_valid(c).buffer(0))
        for k in keys: region[k] = unary_union(by.get(k, [])).buffer(-ZG / 2)
        if PRECEDENCE:
            hi = [k for k in keys if PRIO.get(k[0], 4) >= 5]; lo = [k for k in keys if k not in hi]
            if hi and lo:
                lo_cu = unary_union([COPPER[k] for k in lo]).buffer(ZG)
                for k in hi:
                    rh = [unary_union(by.get(o, [])) for o in hi if o != k]
                    r = outline.buffer(2)
                    for o in rh: r = r.difference(o)
                    region[k] = (r.buffer(-ZG / 2) if rh else r).difference(lo_cu)
                hi_z = unary_union([CAND[k].intersection(region[k]) for k in hi]).buffer(ZG)
                for k in lo: region[k] = region[k].difference(hi_z)
    for k in keys:
        u = CAND[k]
        allowed = region[k].intersection(outline) if k in region else outline
        u = u.intersection(allowed)
        if ROUND > 0:
            u = u.buffer(ROUND, quad_segs=8).buffer(-ROUND, quad_segs=8).intersection(allowed)
        if ROUND_A > 0:
            u = u.buffer(-ROUND_A, quad_segs=6).buffer(ROUND_A, quad_segs=6)
        u = u.simplify(0.01)
        for g in getattr(u, 'geoms', [u]):
            if g.is_empty or g.geom_type != 'Polygon' or g.area < 0.05: continue
            zones.append(dict(net=k[0], layer=L, prio=PRIO.get(k[0], 4), pts=[list(c) for c in g.exterior.coords][:-1], full=True))



if os.environ.get('TEARDROP', '1') == '1':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from teardrop import teardrop
    _GL = [(g.split('=', 1) + [GNDN])[:2] for g in (sys.argv[4].split(',') if len(sys.argv) > 4 else ['In1'])]
    _FN = dict(x.split(':', 1) for x in os.environ.get('FILL_NET', '').split(';') if ':' in x)
    entire = {(_FN.get(L_, n_), L_) for L_, n_ in _GL}
    if os.environ.get('ZONE_GND_SIGNAL') == '1' or os.environ.get('GND_ALL') == '1':
        entire |= {(_FN.get(L, GNDN), L) for L in LAYS}
    off = set(POWER) | {GNDN} if os.environ.get('TEARDROP_NETS', 'all_items') == 'signal' else set()
    areas = {}
    for z in zones:
        if not z.get('teardrop'): areas.setdefault((z['net'], z['layer']), []).append(Polygon(z['pts']).buffer(0))
    TD = teardrop(D, R, via_d=float(os.environ.get('TEARDROP_VIA', '0.45')), nets_off=off, entire=entire,
                 areas=areas if os.environ.get('TEARDROP_AREAS') == '1' else None)
    for td in TD:
        zones.append(dict(net=td['net'], layer=td['layer'], prio=50, pts=td['pts'], full=True, teardrop=td['category']))
    import collections as _c
    print('Fillet-Shapes', len(TD), dict(_c.Counter(q['category'] for q in TD)))
ob = [list(c) for c in outline.exterior.coords][:-1]

FN = dict(x.split(':', 1) for x in os.environ.get('FILL_NET', '').split(';') if ':' in x)
_raw = sys.argv[4] if len(sys.argv) > 4 else 'In1'
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis.areas import active
if '=' in _raw and active(R, NC or {}):
    from analysis.areas import assignment, copper_map, new_contours
    KK = NC or dict(gnd=[GNDN], net_list={}, cu_layers=D.get('cu_layers', 8))
    for L_, nn in assignment(D, {}, KK, gnd_layers=_raw).items():
        if L_ in FN: nn = FN[L_].split('|')
        if not nn: continue
        GL = L_ + '=' + '|'.join(nn)
        rr = dict(new_contours(R), options=dict(R.get('options') or {}, gnd_layers=GL, zclr=ZG, edge=float(os.environ.get('ZONE_EDGE', '0.25'))))
        zones.extend(dict(z, prio=0, full=True) for z in copper_map(D, rr, KK)['zones'])
else:
    for GL in _raw.split(',') if _raw else []:
        L_, n_ = (GL.split('=', 1) + [GNDN])[:2]
        zones.append(dict(net=FN.get(L_, n_), layer=L_, prio=0, pts=ob, full=True))
if os.environ.get('ZONE_GND_SIGNAL') == '1' or os.environ.get('GND_ALL') == '1':
    for L in LAYS:
        zones.append(dict(net=FN.get(L, GNDN), layer=L, prio=0, pts=ob, full=True, gnd_signal=True))



SHAPE = os.environ.get('ZONE_SHAPE', 'track')
if SHAPE not in ('track', 'polygon'):
    raise ValueError('Unknown ZONE_SHAPE: ' + SHAPE)
if SHAPE == 'polygon':
    if NC is None:
        raise ValueError('ZONE_SHAPE=polygon needs NETCLASSES with flow/load model')
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from codex.power_polygon.polygon import power_polygons
    _opt = {k: v for k, v in os.environ.items()
            if k.startswith('ZONE_') or k in ('ZCLR', 'TONGUE', 'FILL_NET', 'GND_ALL')}
    zones, _diag = power_polygons(D, R, NC, zones, options=_opt)
    if ROUND_A > 0:
        for z in zones:
            if z.get('net') in POWER and len(z['pts']) >= 3 and not z.get('holes'):
                g = Polygon(z['pts']).buffer(-ROUND_A, quad_segs=6).buffer(ROUND_A, quad_segs=6)
                if not g.is_empty and g.geom_type == 'Polygon': z['pts'] = [list(c) for c in g.exterior.coords][:-1]
    json.dump(_diag, open(OUT + '.polygon.json', 'w'), indent=2)

if os.environ.get('ZONE_K15', '0') == '1':
    if NC is None:
        raise ValueError('ZONE_K15=1 needs NETCLASSES')
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from postprocessing.zone_shape import shape_zones
    _k15opt = {k: v for k, v in os.environ.items() if k.startswith('ZONE_') or k == 'ZCLR'}
    zones, _k15diag = shape_zones(D, R, NC, zones, options=_k15opt)
    json.dump(_k15diag, open(OUT + '.k15.json', 'w'), indent=2)
json.dump(zones, open(OUT, 'w'))
print('Zones', len(zones))
