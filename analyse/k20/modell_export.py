""
import json, sys, os
sys.path.insert(0, '.')
from shapely.geometry import Polygon, mapping
from shapely.ops import unary_union
from lernen.kupfer import _via_masse
from gemeinsam.zonen import boardgeometrie
from analyse.kupfermodell import einstellungen, flaechen, bezug, geometrie, bohrungen, sperrflaeche, angeschlossen, auf_lage, via_masse
bj, kj, rj, gj, out = sys.argv[1:6]
B, K, R = json.load(open(bj)), json.load(open(kj)), json.load(open(rj))
regeln = json.load(open(gj)) if os.path.exists(gj) else None
B = boardgeometrie(B, R)
nl = int(B.get('lagen') or 2)
lagen = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
clr, rand = einstellungen(K, regeln, None, None)
min_breite = 0.2
planes, gl, gnd = flaechen(B, R, K)
d_via, b_via = _via_masse(R, K)

alle = {}
for net in sorted(set(planes.values()) | gnd):
    kandidaten = {}
    for L in lagen:
        if not (planes.get(L) == net or (L not in gl and net in gnd)): continue
        f = bezug(B, R, L, {net}, clr=clr, rand=rand, via_d=d_via, drill=b_via)
        if L not in gl: f = f.buffer(-min_breite / 2, 8).buffer(min_breite / 2, 8)
        kandidaten[L] = f
    alle[net] = kandidaten
from shapely import voronoi_polygons
from shapely.geometry import MultiPoint
for L in lagen:
    aktive = [n for n, fs in alle.items() if L in fs and not fs[L].is_empty]
    if len(aktive) < 2: continue
    punkte = {}
    for n, g in geometrie(B, R, L, d_via):
        if n in aktive and not g.is_empty:
            q = g.representative_point(); punkte[(q.x, q.y)] = n
    if len(set(punkte.values())) < 2:
        for n in aktive:
            if n not in punkte.values(): alle[n][L] = Polygon()
        continue
    coords = sorted(punkte)
    zellen = voronoi_polygons(MultiPoint(coords), extend_to=unary_union([alle[n][L] for n in aktive]).envelope, ordered=True)
    rollen = {n: [] for n in aktive}
    for xy, zelle in zip(coords, zellen.geoms): rollen[punkte[xy]].append(zelle)
    for n in aktive: alle[n][L] = alle[n][L].intersection(unary_union(rollen[n]).buffer(-clr / 2, join_style=2))
fills = {}
for net, kandidaten in alle.items():
    verbunden = angeschlossen(B, R, kandidaten, {L: net for L in kandidaten}, via_d=d_via, drill=b_via)
    for L, f in verbunden.items():
        if not f.is_empty: fills[(net, L)] = f

def polys(g):
    res = []
    for p in getattr(g, 'geoms', [g]):
        if p.is_empty or p.geom_type != 'Polygon': continue
        res.append([list(p.exterior.coords), [list(i.coords) for i in p.interiors]])
    return res
cu = dict(layers=lagen, nets={}, vias=[], pads=[])
zus = dict(layers=lagen, outline=[], keepouts=[], zones=[], tracks=[], vias=[], min_clearance=clr, quelle='modell', gndlagen=sorted(gl))
outl = unary_union([Polygon(q, (B.get('outline_holes') or [[]] * len(B['outline']))[i] if i < len(B.get('outline_holes') or []) else []).buffer(0) for i, q in enumerate(B['outline']) if len(q) >= 3])
zus['outline'] = polys(outl)
for k in B.get('keepouts', []):
    zus['keepouts'].append(dict(layers=k.get('lagen') or lagen, kein_kupfer=bool((k.get('verbote') or {}).get('flaechen')), polys=polys(sperrflaeche(k))))
for L in lagen:
    teile = geometrie(B, R, L, d_via)
    jen = {}
    for n, g in teile: jen.setdefault(n, []).append(g)
    for (n, LL), f in fills.items():
        if LL == L: jen.setdefault(n, []).append(f)
    bohr = bohrungen(B, R, b_via, L)
    for n, gs in jen.items():
        g = unary_union(gs)
        for k in B.get('keepouts', []):
            if (k.get('verbote') or {}).get('flaechen') and L in (k.get('lagen') or lagen):
                pass
        cu['nets'].setdefault(n, {})[L] = polys(g)

    zg = {}
    for z in R.get('zonen', []):
        if z['layer'] == L:
            zg.setdefault((z['net'], z.get('quelle', '')), []).append(unary_union([Polygon(q['aussen'], q.get('innen') or []).buffer(0) for q in z['polys']]))
    for (n, LL), f in fills.items():
        if LL == L: zg.setdefault((n, 'modell_fuellung'), []).append(f)
    for (n, q), gs in sorted(zg.items()):
        zus['zones'].append(dict(net=n, layer=L, quelle=q, prio=None, clearance=clr, min_breite=None, fill=polys(unary_union(gs))))
for t in R.get('tracks', []):
    pts = t['pts']; ws = t['w'] if isinstance(t['w'], list) else [t['w']] * (len(pts) - 1)
    for a, b, w in zip(pts[:-1], pts[1:], ws):
        zus['tracks'].append(dict(net=t['net'], layer=t['layer'], a=a, b=b, w=w, arc=False))
for v in R.get('vias', []):
    d, dr = via_masse(v, d_via, b_via)
    vl = [L for L in lagen if auf_lage(v, L)]
    zus['vias'].append(dict(net=v.get('net', ''), x=v['x'], y=v['y'], d=d, drill=dr, layers=vl))
zus['bohrungen'] = [dict(x=p['x'], y=p['y'], d=p['drill'], dmax=p['drill'], npth=not p.get('net'), net=p.get('net', '')) for p in B['pads'] if p.get('drill')]
cu['vias'] = [dict(net=v['net'], x=v['x'], y=v['y'], d=v['d'], drill=v['drill']) for v in zus['vias']]
cu['pads'] = [dict(ref=p.get('ref'), num=p.get('num'), net=p.get('net'), x=p.get('x'), y=p.get('y'), layers=p.get('layers')) for p in B['pads']]
os.makedirs(out, exist_ok=True)
json.dump(cu, open(os.path.join(out, 'cu.json'), 'w')); json.dump(zus, open(os.path.join(out, 'zusatz.json'), 'w'))
print('Lagen', lagen, 'gndlagen', sorted(gl), 'planes', planes, 'clr', clr, 'rand', rand, 'Fuellungen', len(fills), 'Zonen', len(zus['zones']), 'Bahnsegmente', len(zus['tracks']))
