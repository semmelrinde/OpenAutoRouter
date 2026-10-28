""
import collections
import math
import shapely
from shapely.geometry import Polygon, Point, LineString, MultiPoint
from shapely.ops import unary_union
from gemeinsam.fluss import padgeo


def aktiv(R, K):
    ""
    flag = (K.get('vorgaben') or {}).get('lagenaufbau', (R.get('optionen') or {}).get('lagenaufbau', False))
    return flag is True or flag == '1'


def native_lagen(B, R, K):
    ""
    complete = R.get('fuellung_vollstaendig') is True
    complete |= 'zones' in R and str(R.get('quelle', '')).startswith('platine:')
    return set(R.get('fuellung_lagen', stapel(B, K))) if complete else set()


def stapel(B, K):
    n = int(B.get('lagen') or K.get('lagen') or 2)
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def zuordnung(B, R, K, gndlagen=None):
    ""
    names = {p['net'] for p in B['pads'] if p['net']}
    def name(n):
        return n if n in names else '/' + n if '/' + n in names else n.lstrip('/') if n.lstrip('/') in names else n
    raw = gndlagen if gndlagen is not None else (R.get('optionen') or {}).get('gndlagen')
    if raw is None: raw = R.get('gndlagen') or []
    qs = [q for q in raw.split(',') if q] if isinstance(raw, str) else list(raw)
    explicit = any('=' in q for q in qs)
    ground = sorted(K.get('gnd') or [])
    if len(ground) > 1: ground = [n for n in ground if n.split('/')[-1] == 'GND']
    out = {}
    for q in qs:
        L, *nn = q.split('=', 1)
        if L not in stapel(B, K): continue
        out[L] = sorted({name(n) for n in nn[0].split('|') if n}) if nn else ([] if explicit else ground)
    return out


def _polys(g):
    if g.is_empty: return []
    if g.geom_type == 'Polygon': return [g]
    return [p for q in getattr(g, 'geoms', []) for p in _polys(q)]


def umriss(B):
    ""
    holes = B.get('outline_holes') or []
    return unary_union([Polygon(q, holes[i] if i < len(holes) else []).buffer(0)
                        for i, q in enumerate(B.get('outline', [])) if len(q) >= 3])


def neue_umrisse(R):
    ""
    return {k: ([] if k == 'zones' else v) for k, v in R.items()
            if k not in ('fuellung_vollstaendig', 'fuellung_lagen', 'quelle')} | {'zones': []}


def plan(B, R, K):
    ""
    mapping = zuordnung(B, R, K)
    rr = neue_umrisse(R)
    rr['vias'] = [dict(net=p['net'], x=p['x'], y=p['y']) for p in B['pads'] if p['net']]
    rr['tracks'] = []
    filled = kupfer(B, rr, K)
    failed = filled.get('flaechen_bahnbaum') or []
    free = {x['lage'] for x in failed}
    return {L: nn for L, nn in mapping.items() if L not in free}, failed


def kupfer(B, R, K, gndlagen=None):
    ""
    mapping = zuordnung(B, R, K, gndlagen=gndlagen); layers = stapel(B, K)

    zones = [z for z in R.get('zones', []) if z.get('filled') and not z.get('estimated')]
    actual = {z['layer'] for z in zones} | native_lagen(B, R, K)
    if not mapping or not B.get('outline'): return dict(R, zones=zones)
    opt = R.get('optionen') or {}
    clr = float(opt.get('zclr', opt.get('clr', 0.15)))
    edge = float(opt.get('edge', 0.25))
    outline = umriss(B).buffer(-edge)
    vd = float(str(opt.get('via', '0.45/0.20')).split('/')[0])
    raw = collections.defaultdict(list); contacts = collections.defaultdict(list)
    holes = []
    for p in B['pads']:
        pg = padgeo(p)
        for L, g in pg.items(): raw[L, p['net']].append(g)
        if p.get('drill'):
            mechanical = not pg or not p.get('net')
            radius = float(p['drill']) / 2 + (clr if mechanical else 0)

            holes.append(Point(p['x'], p['y']).buffer(radius / math.cos(math.pi / 64), quad_segs=16))
        for L in p.get('layers', []): contacts[L, p['net']].append((p['x'], p['y']))
    for t in R.get('tracks', []):
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            raw[t['layer'], t['net']].append(LineString([a, b]).buffer(w / 2))
    for v in R.get('vias', []):
        for L in v.get('layers') or layers:
            raw[L, v['net']].append(Point(v['x'], v['y']).buffer(float(v.get('diameter', v.get('d', vd))) / 2))
            contacts[L, v['net']].append((v['x'], v['y']))
    fallback = []
    for L, nets in sorted(mapping.items()):
        if L in actual or not nets: continue
        regions = {n: outline for n in nets}
        if len(nets) > 1:
            sites = {}
            for n in nets:
                xy = contacts[L, n] or [(p['x'], p['y']) for p in B['pads'] if p['net'] == n]
                for q in xy:
                    if q in sites and sites[q] != n: raise ValueError('Flächenanschlüsse verschiedener Netze am selben Ort')
                    sites[q] = n
            if len(set(sites.values())) != len(nets): raise ValueError('Flächennetz ohne Anschlussposition')
            qs = sorted(sites)
            cells = shapely.voronoi_polygons(MultiPoint(qs), extend_to=outline.envelope)
            owned = collections.defaultdict(list)
            for cell in cells.geoms:
                i = next(i for i, q in enumerate(qs) if cell.covers(Point(q)))
                owned[sites[qs[i]]].append(shapely.make_valid(cell))

            regions = {n: shapely.union_all(owned[n], grid_size=1e-6).buffer(-clr / 2).intersection(outline) for n in nets}
        for n in nets:

            obs = holes + [g.buffer(clr) for (l, m), gg in raw.items() if l == L and m != n for g in gg]
            obs += [Polygon(q['pts'], q.get('holes', [])) for q in B.get('zone_keepouts', []) if q['layer'] == L]
            for q in B.get('keepouts', []) + B.get('rule_areas', []):
                if L in q.get('lagen', []) and q.get('verbote', {}).get('flaechen'):
                    obs += [Polygon(pts, (q.get('loecher') or [[] for _ in q.get('polys', [])])[i]) for i, pts in enumerate(q.get('polys', []))]
            g = regions[n].difference(unary_union(obs)) if obs else regions[n]
            parts = _polys(g)
            if n not in set(K.get('gnd') or []):
                xy = contacts[L, n] or [(p['x'], p['y']) for p in B['pads'] if p['net'] == n]


                candidates = [p for p in parts if xy and all(p.distance(Point(q)) < vd / 2 - 1e-6 for q in xy)]
                if not candidates:
                    fallback.append(dict(netz=n, lage=L, grund='keine zusammenhängende Fläche für alle Anschlüsse', teile=len(parts)))
                parts = sorted(candidates, key=lambda p: (-p.area, p.bounds))[:1]
            for part in parts:
                zones.append(dict(net=n, layer=L, pts=list(map(list, part.exterior.coords)),
                                  holes=[list(map(list, h.coords)) for h in part.interiors], filled=True, estimated=True))
    return dict(R, zones=zones, **({'flaechen_bahnbaum': fallback} if fallback else {}))


def verbunden(B, R, K, n, details=False):
    ""
    layers = stapel(B, K); raw = collections.defaultdict(list)
    vd = float(str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])
    pads = [p for p in B['pads'] if p['net'] == n]
    for p in pads:
        for L, g in padgeo(p).items(): raw[L].append(g)
    for t in R.get('tracks', []):
        if t['net'] == n:
            raw[t['layer']] += [LineString([a, b]).buffer(w / 2) for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
    for z in R.get('zones', []):
        if z['net'] == n and z.get('filled'): raw[z['layer']].append(Polygon(z['pts'], z.get('holes', [])).buffer(0))
    barrels = []
    for v in R.get('vias', []):
        if v['net'] != n: continue
        g = Point(v['x'], v['y']).buffer(float(v.get('diameter', v.get('d', vd))) / 2)
        ls = v.get('layers') or layers
        for L in ls: raw[L].append(g)
        barrels.append((ls, g))
    for p in pads:
        if p.get('drill'): barrels.append((p['layers'], Point(p['x'], p['y']).buffer(p['drill'] / 2)))
    comps = {}; parent = []
    for L, gs in raw.items():
        comps[L] = []
        for g in _polys(unary_union(gs)):
            i = len(parent); parent.append(i); comps[L].append((i, g))
    def root(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    def hits(ls, gs):
        return [i for L in ls for i, g in comps.get(L, []) if g.distance(gs[L] if isinstance(gs, dict) else gs) < 1e-6]
    for ls, g in barrels:
        ids = hits(ls, g)
        for i in ids[1:]: parent[root(i)] = root(ids[0])

    by = collections.defaultdict(list)
    for p in pads: by[p['ref'], str(p['num'])] += hits(p['layers'], padgeo(p))
    for ids in by.values():
        for i in ids[1:]: parent[root(i)] = root(ids[0])
    roots = {root(i) for ids in by.values() for i in ids}
    if details:


        sources = ((K.get('netze') or {}).get(n, {}).get('fluss') or {}).get('quelle') or []
        refs = {root(i) for key, ids in by.items() if '%s:%s' % key in sources for i in ids}
        selected = refs if sources else roots
        if len(selected) != 1: selected = set()
        return {L: [g for i, g in pcs if root(i) in selected] for L, pcs in comps.items()}

    return bool(by) and all(ids for ids in by.values()) and len(roots) == 1


def kennzahl(B, R, K, gndlagen=None):
    ""
    S = stapel(B, K); Z = zuordnung(B, R, K, gndlagen=gndlagen); G = set(K.get('gnd') or [])
    conflicts = []; native_ground = set(); actual = False


    ol = umriss(B)
    authoritative = native_lagen(B, R, K)
    for L in S:
        zz = [z for z in R.get('zones', []) if z.get('filled') and not z.get('estimated') and z['layer'] == L]
        if not zz and L not in authoritative: continue
        actual = True
        gg = [(z['net'], Polygon(z['pts'], z.get('holes', [])).buffer(0).intersection(ol)) for z in zz]
        nets = sorted({n for n, g in gg if not g.is_empty and g.area > 0})
        if any(n in G and not g.is_empty and g.area > 0 for n, g in gg): native_ground.add(L)
        coverage = unary_union([g for _, g in gg]).area / ol.area if not ol.is_empty else 0
        declared_nets = Z.pop(L, None)
        if declared_nets is not None and (declared_nets != nets or coverage < .8):
            conflicts.append(dict(lage=L, deklariert=declared_nets, gefuellt=nets,
                                  kupferanteil=round(coverage, 6), grund='Netze abweichend' if declared_nets != nets else 'Teilfüllung unter 80%'))
        if nets and coverage >= .8: Z[L] = nets
    gl = [L for L in S if Z.get(L) and set(Z[L]) <= G]
    pl = [L for L in S if any(n not in G for n in Z.get(L, []))]
    sig = [L for L in S if L not in Z]
    nb = {L: [S[j] for j in (i - 1, i + 1) if 0 <= j < len(S) and S[j] in gl] for i, L in enumerate(S) if L in sig}
    N = K.get('netze') or {}
    high = max([float(k.get('strom_A') or 0) for k in N.values()] + [0])
    fast = any(k.get('art') == 'schnell' for k in N.values())
    analogue = {n for n, k in N.items() if k.get('art') in ('analog', 'kelvin')}
    sensitive = any(N[n].get('funktion') != 'rueckkopplung' for n in analogue)
    power = len(S) >= 4 and len(sig) == 2 and high >= 3 and not fast and not sensitive and bool(pl) and all(Z.get(L) for L in S if L not in sig)
    need = 3 if len(S) >= 8 else 2 if len(S) >= 6 else 1 if len(S) >= 4 else 0
    absent = max(0, need - len(gl)) if not power else 0
    unref = [L for L in sig if not nb[L]] if len(S) >= 4 and not power else []
    unshielded = sorted({t['net'] for t in R.get('tracks', []) if t['net'] in analogue and
                        (t['layer'] not in nb or len(nb[t['layer']]) != 2)}) if len(S) >= 4 and not power else []
    pabs = int(len(S) >= 8 and not pl and not power)
    declared = 'gndlagen' in (R.get('optionen') or {}) or 'gndlagen' in R or gndlagen is not None
    proof = 'exportierte Füllung' if actual else ('deklarierte Flächenlagen' if declared else 'kein Füllnachweis')


    flag = (K.get('vorgaben') or {}).get('lagenaufbau_heuristik', (R.get('optionen') or {}).get('lagenaufbau_heuristik', False))
    active = flag is True or flag == '1'
    return dict(nachweis=proof, gndlagen=gl, gnd_kupferlagen=[L for L in S if L in native_ground or L in gl],
                widersprueche=conflicts, tor_aktiv=active, potenziallagen=pl, signallagen=sig, nachbarn=nb, gnd_fehlen=absent,
                potenzial_fehlt=pabs, ohne_bezug=unref, analog_ungeschirmt=unshielded,
                leistungsboard=power, ausnahme_grund=('Hochstrom >= 3 A, zwei Signallagen, Rest Flächen, keine schnellen/empfindlichen Netze' if power else None),
                summe=absent + pabs + len(unref) + len(unshielded), modell='K19; Flächenerkennung >=80% Kupferabdeckung')
