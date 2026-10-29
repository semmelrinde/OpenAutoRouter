import json, sys, re, math, os, time, collections
import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

D, NAME, KIND = sys.argv[1:4]
T0 = time.time()
C = json.load(open(os.path.join(D, 'cu.json')))
Z = json.load(open(os.path.join(D, 'extra.json')))
K = json.load(open(os.path.join(D, 'netclasses.json'))) if os.path.exists(os.path.join(D, 'netclasses.json')) else None
LAYERS = Z['layers']
OUTER = {'F', 'B'}
TOL = 0.03
CLR_UNIT = 0.15
CLR_OWNED = {'model_example': 0.10, 'kicad_native': 0.10}.get(KIND, float(Z.get('min_clearance') or 0.15))
SMOOTHED = 0.05
CORNER_MIN = math.radians(30)
R_ROUND, EPS = 0.10, 0.012
MIDDLE = 0.01


def geo(polys):
    ps = []
    for o, hs in polys:
        if len(o) < 3: continue
        p = Polygon(o, [h for h in hs if len(h) >= 3])
        if not p.is_valid: p = p.buffer(0)
        ps.append(p)
    return unary_union(ps) if ps else Polygon()


def part_list(g):
    return [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and not p.is_empty]



GND_RE = re.compile(r'(^|[/_\-])(A|D|P)?GND\w*$|(^|[/_\-])GND[AD]?$|^VSS$', re.I)
SUPPLY_TOK = re.compile(r'^(\d+V\d*[A-Z]?|V(CC|DD|BUS|IN|AA|DC|REF|CORE|LO|SYS|BAT|IO|CAP|PP|MOT|LED)\w*|AVDD\w*|DVDD\w*|IOVDD\w*|PVDD\w*|DREG|AREG|SW|BST|BOOT)$', re.I)
SIG_TOK = re.compile(r'^(PWM|RXD?\d*|TXD?\d*|I?SENSE|SNS|EN|FB|NC\d*|SSCAP|PG|OUT|IN\d+|SDA|SCL|CLK|DATA)$', re.I)
def supply_name(n):
    if n.lower().startswith('unconnected'): return False
    sheet = n.split('/')[-1]
    m = re.match(r'^Net-\((.*)\)$', sheet)
    if m: sheet = m.group(1).split('-')[-1]
    tok = [t for t in re.split(r'[_\-+/() ]+', sheet) if t]
    return any(SUPPLY_TOK.match(t) for t in tok) and not any(SIG_TOK.match(t) for t in tok)
net_list = sorted(set(C['nets']) | {t['net'] for t in Z['tracks']})
gnd = {n for n in net_list if n and GND_RE.search(n)}
if K: gnd |= set(K.get('gnd') or [])
zone_nets = {z['net'] for z in Z['zones'] if z['fill']}
sup = {n for n in net_list if n and n not in gnd and (supply_name(n) or n in zone_nets)}
if K: sup |= {n for n, v in (K.get('net_list') or {}).items() if v.get('category') in ('current_val', 'supply', 'sw') and n not in gnd}
category = lambda n: 'gnd' if n in gnd else 'sup' if n in sup else 'sig'


outline = geo(Z['outline'])
if KIND == 'model_example':
    levels = unary_union([geo(jl[L]) for n, jl in C['nets'].items() for L in ('In1', 'In6') if L in jl])
    entire = unary_union([Polygon(p.exterior) for p in part_list(levels) if p.area > 50])
    outline = outline.intersection(entire.buffer(0.45, 16))
keepout_sources = list(Z['keepouts'])
if not keepout_sources and os.path.exists(os.path.join(D, 'board.json')):
    BJ = json.load(open(os.path.join(D, 'board.json')))
    for k in BJ.get('keepouts', []):
        lo = k.get('hole_list') or []
        keepout_sources.append(dict(layers=k.get('cu_layers') or LAYERS, no_copper=bool((k.get('prohibitions') or {}).get('areas')),
                                  polys=[[q, lo[i] if i < len(lo) else []] for i, q in enumerate(k.get('polys', []))]))
blocked = {L: unary_union([geo(k['polys']) for k in keepout_sources if k.get('no_copper') and L in k['layers']] or [Polygon()]) for L in LAYERS}
layer_area = {L: outline.difference(blocked[L]) for L in LAYERS}
cu = {L: {} for L in LAYERS}
for n, jl in C['nets'].items():
    for L, ps in jl.items():
        if L in cu:
            g = geo(ps)
            if not g.is_empty: cu[L][n] = g
zf = {}
for z in Z['zones']:
    if z['fill'] and z['layer'] in cu: zf.setdefault((z['net'], z['layer']), []).append(geo(z['fill']))
zf = {k: unary_union(v) for k, v in zf.items()}
track_list = {}
for t in Z['tracks']:
    a, b = tuple(t['a']), tuple(t['b'])
    g = (LineString([a, b]) if a != b else Point(a)).buffer(t['w'] / 2, 8)
    track_list.setdefault((t['net'], t['layer']), []).append(g)
track_list = {k: unary_union(v) for k, v in track_list.items()}
bore_pts = [(b['x'], b['y'], b['d']) for b in Z.get('drills', [])]


def corners(G):
    out, total_length = [], 0.0
    for p in part_list(G):
        p = orient(p, 1.0)
        for ring in [p.exterior] + list(p.interiors):
            total_length += ring.length
            r = ring.simplify(SMOOTHED)
            xy = np.asarray(r.coords)[:-1]
            if len(xy) < 3: continue
            v1 = xy - np.roll(xy, 1, axis=0); v2 = np.roll(xy, -1, axis=0) - xy
            cr = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]; dt = (v1 * v2).sum(1)
            tau = np.arctan2(cr, dt)
            for (x, y), t in zip(xy, tau):
                if abs(t) >= CORNER_MIN: out.append((x, y, t))
    return out, total_length


def corner_size(G):
    E, total_length = corners(G)
    if not E: return dict(border_mm=total_length, corners=0, sharp_cu=0, sharp_white=0, rounded=0)
    xs = np.array([e[0] for e in E]); ys = np.array([e[1] for e in E]); ts = np.array([e[2] for e in E])
    on = G.buffer(-R_ROUND, 16).buffer(R_ROUND + EPS, 16)
    too = G.buffer(R_ROUND, 16).buffer(-R_ROUND - EPS, 16)
    shapely.prepare(on); shapely.prepare(too)
    convex = ts > 0
    rounded = np.where(convex, shapely.contains_xy(on, xs, ys), ~shapely.contains_xy(too, xs, ys))
    return dict(border_mm=total_length, corners=int(len(E)), sharp_cu=int((ts > math.pi / 2).sum()), sharp_white=int((ts < -math.pi / 2).sum()), rounded=int(rounded.sum()),
                sharp_cu_xy=[(round(x, 3), round(y, 3)) for x, y, t in E if t > math.pi / 2][:400],
                sharp_white_xy=[(round(x, 3), round(y, 3)) for x, y, t in E if t < -math.pi / 2][:400])


erg = dict(name=NAME, category=KIND, cu_layers=LAYERS, area_mm2=round(outline.area, 1), keepout_mm2={L: round(blocked[L].area, 1) for L in LAYERS}, clr_owned=CLR_OWNED, gnd=sorted(gnd), sup=sorted(sup), per_layer={})
totals = collections.Counter()
for L in LAYERS:
    t = time.time()
    A = layer_area[L].area
    um = layer_area[L]
    g_cu = unary_union([g for n, g in cu[L].items() if n in gnd] or [Polygon()]).intersection(um)
    v_cu = unary_union([g for n, g in cu[L].items() if n in sup] or [Polygon()]).intersection(um)
    s_cu = unary_union([g for n, g in cu[L].items() if category(n) == 'sig'] or [Polygon()]).intersection(um)
    everything = unary_union(list(cu[L].values()) or [Polygon()])
    r = dict(area_mm2=round(A, 1), gnd_share=round(g_cu.area / A, 4), supply_share=round(v_cu.area / A, 4), sig_share=round(s_cu.area / A, 4))

    gz = unary_union([g for (n, LL), g in zf.items() if LL == L and n in gnd] or [Polygon()])
    st = [p for p in part_list(g_cu) if p.area >= 0.01 and p.intersects(gz)]
    anchor = [Point(v['x'], v['y']) for v in Z['vias'] if v['net'] in gnd and (not v.get('layers') or L in v['layers'])]
    anchor += [Point(p_['x'], p_['y']) for p_ in C['pads'] if p_.get('net') in gnd and L in (p_.get('layers') or [])]
    from shapely.strtree import STRtree
    tree_idx = STRtree(anchor) if anchor else None
    def has_anchor(p):
        return tree_idx is not None and any(p.covers(anchor[i]) for i in tree_idx.query(p))
    islands = [p for p in st if p.area >= 1.0]
    spl = [p for p in st if p.area < 1.0]
    r['gnd_pieces'] = len(st)
    r['gnd_islands'] = len(islands)
    r['gnd_splitter_with_connection'] = sum(has_anchor(p) for p in spl)
    r['gnd_splitter_without'] = len(spl) - r['gnd_splitter_with_connection']
    r['gnd_smallest_island_mm2'] = round(min((p.area for p in islands), default=0), 3)
    r['gnd_smallest_island_share'] = round(min((p.area for p in islands), default=0) / max(1e-9, sum(p.area for p in st)), 5)

    for tag, clr in (('15', CLR_UNIT), ('owned', CLR_OWNED)):
        hole_size = unary_union([Point(x, y).buffer(d / 2 + 0.255 - clr, 12) for x, y, d in bore_pts] or [Polygon()])
        inner = outline.buffer(-0.5).difference(blocked[L].buffer(clr))
        w = inner.difference(everything.union(hole_size).buffer(clr + TOL, 12))
        wt = [p for p in part_list(w) if p.area > 0.002]
        r['white%s_mm2' % tag] = round(sum(p.area for p in wt), 3); r['white%s_n' % tag] = len(wt)
        if tag == '15': r['white_xy'] = [(round(p.centroid.x, 3), round(p.centroid.y, 3), round(p.area, 4)) for p in wt]

    vz = []
    for (n, LL), f in sorted(zf.items()):
        if LL != L or n not in sup: continue
        own = cu[L].get(n, Polygon())
        zc = f.union(own)
        stk = [p for p in part_list(f) if p.area >= 0.01]
        with_ = [c for c in part_list(zc) if c.intersects(f)]
        hole_nc = sum(Polygon(i).area for c in with_ for i in c.interiors if Polygon(i).area < 3)
        vz.append(dict(net=n, pieces=len(stk), parts_with_owned=len(with_), splitter=sum(p.area < 0.5 for p in stk), surface=round(f.area, 3),
                       piece_mm2=sorted(round(p.area, 3) for p in stk), hole_mm2=round(hole_nc, 3), copper_mm2=round(sum(c.area for c in with_), 3)))
    r['supply_zones'] = vz

    v_zone = unary_union([f for (n, LL), f in zf.items() if LL == L and n in sup] or [Polygon()])
    v_track = unary_union([g for (n, LL), g in track_list.items() if LL == L and n in sup] or [Polygon()])
    r['supply_zone_mm2'] = round(v_zone.area, 2); r['supply_track_mm2'] = round(v_track.difference(v_zone).area, 2)

    for net_class, pool in (('gnd', gnd), ('sup', sup)):
        Gs = []
        for (n, LL), f in zf.items():
            if LL != L or n not in pool: continue
            zc = f.union(cu[L].get(n, Polygon()))
            Gs += [c for c in part_list(zc) if c.intersects(f)]
        G = unary_union(Gs) if Gs else Polygon()
        r['corners_' + net_class] = corner_size(G) if not G.is_empty else dict(border_mm=0, corners=0, sharp_cu=0, sharp_white=0, rounded=0)
    r['secondary'] = round(time.time() - t, 1)
    erg['per_layer'][L] = r
    print(NAME, L, 'GND %.0f%%' % (100 * r['gnd_share']), 'White15 %.2f mm2 (%d)' % (r['white15_mm2'], r['white15_n']), 'GND-Islands', r['gnd_islands'], 'Splinters with/without', r['gnd_splitter_with_connection'], r['gnd_splitter_without'],
          'Supply-Zones', len(vz), 'sharp', r['corners_gnd']['sharp_cu'] + r['corners_supply']['sharp_cu'], '%.1fs' % r['secondary'], flush=True)


grid_cells = collections.defaultdict(list)
for i, t in enumerate(Z['tracks']):
    for p in (t['a'], t['b']): grid_cells[(int(p[0] // 1), int(p[1] // 1))].append(i)
def seg_dist(px, py, a, b):
    ax, ay = a; bx, by = b; dx, dy = bx - ax, by - ay; l2 = dx * dx + dy * dy
    u = 0 if l2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)
conn = middle_n = 0; vias_with = vias_skewed = 0; skewed_xy = []
for v in Z['vias']:
    r = v['d'] / 2; cx, cy = v['x'], v['y']
    cand = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1): cand.update(grid_cells.get((int(cx // 1) + dx, int(cy // 1) + dy), []))
    S_ = []
    for i in sorted(cand):
        t = Z['tracks'][i]
        if t['net'] != v['net'] or (v.get('layers') and t['layer'] not in v['layers']): continue
        if any(math.hypot(p[0] - cx, p[1] - cy) < r - 1e-6 for p in (t['a'], t['b'])) or seg_dist(cx, cy, t['a'], t['b']) <= MIDDLE: S_.append(i)
    if not S_: continue
    parent = {i: i for i in S_}
    def root(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    nodes = collections.defaultdict(list)
    for i in S_:
        t = Z['tracks'][i]
        for p in (t['a'], t['b']):
            if math.hypot(p[0] - cx, p[1] - cy) > MIDDLE: nodes[(t['layer'], round(p[0], 4), round(p[1], 4))].append(i)
    for k, ids in nodes.items():
        for j in ids[1:]: parent[root(j)] = root(ids[0])
    group_list = collections.defaultdict(list)
    for i in S_: group_list[root(i)].append(i)
    vias_with += 1; skewed = False
    for g, ids in group_list.items():
        conn += 1
        if any(seg_dist(cx, cy, Z['tracks'][i]['a'], Z['tracks'][i]['b']) <= MIDDLE for i in ids): middle_n += 1
        else: skewed = True
    if skewed: vias_skewed += 1; skewed_xy.append((round(cx, 3), round(cy, 3)))
erg['r37'] = dict(connections=conn, middle=middle_n, share=round(middle_n / max(1, conn), 4), vias_with_track=vias_with, vias_skewed=vias_skewed,
                  vias=len(Z['vias']), skewed_xy=skewed_xy[:300])


J = erg['per_layer']
def agg(cu_layers):
    vz = [z for L in cu_layers for z in J[L]['supply_zones']]
    ez = {k: sum(J[L]['corners_supply'][k] + J[L]['corners_gnd'][k] for L in cu_layers) for k in ('border_mm', 'corners', 'sharp_cu', 'sharp_white', 'rounded')}
    ev = {k: sum(J[L]['corners_supply'][k] for L in cu_layers) for k in ('border_mm', 'corners', 'sharp_cu', 'sharp_white', 'rounded')}
    cu = sum(z['copper_mm2'] for z in vz); hole_size = sum(z['hole_mm2'] for z in vz)
    zo = sum(J[L]['supply_zone_mm2'] for L in cu_layers); ba = sum(J[L]['supply_track_mm2'] for L in cu_layers)
    return dict(
        gnd_share=round(sum(J[L]['gnd_share'] * J[L]['area_mm2'] for L in cu_layers) / max(1e-9, sum(J[L]['area_mm2'] for L in cu_layers)), 4),
        gnd_min_layer=min((J[L]['gnd_share'] for L in cu_layers), default=0),
        gnd_pieces=sum(J[L]['gnd_pieces'] for L in cu_layers), gnd_islands=sum(J[L]['gnd_islands'] for L in cu_layers), gnd_splitter_with=sum(J[L]['gnd_splitter_with_connection'] for L in cu_layers), gnd_splitter_without=sum(J[L]['gnd_splitter_without'] for L in cu_layers), gnd_islands_max_layer=max((J[L]['gnd_islands'] for L in cu_layers), default=0), gnd_smallest_island_share=min((J[L]['gnd_smallest_island_share'] for L in cu_layers if J[L]['gnd_islands']), default=0),
        white15_mm2=round(sum(J[L]['white15_mm2'] for L in cu_layers), 2), white15_n=sum(J[L]['white15_n'] for L in cu_layers),
        white15_per_cm2=round(sum(J[L]['white15_mm2'] for L in cu_layers) / max(1e-9, sum(J[L]['area_mm2'] for L in cu_layers)) * 100, 4),
        white_own_mm2=round(sum(J[L]['white_own_mm2'] for L in cu_layers), 2), white_own_n=sum(J[L]['white_own_n'] for L in cu_layers),
        white_own_per_cm2=round(sum(J[L]['white_own_mm2'] for L in cu_layers) / max(1e-9, sum(J[L]['area_mm2'] for L in cu_layers)) * 100, 4),
        supply_net_layer=len(vz), supply_pieces=sum(z['pieces'] for z in vz), supply_one_piece=sum(z['pieces'] == 1 for z in vz),
        supply_one_piece_with_owned=sum(z['parts_with_owned'] == 1 for z in vz), supply_splitter=sum(z['splitter'] for z in vz),
        supply_piece_median_mm2=float(np.median([a for z in vz for a in z['piece_mm2']])) if vz else 0,
        zone_share=round(zo / max(1e-9, zo + ba), 4), zone_mm2=round(zo, 1), track_mm2=round(ba, 1),
        hole_share=round(hole_size / max(1e-9, cu + hole_size), 4),
        sharp_cu_per100=round(ez['sharp_cu'] / max(1e-9, ez['border_mm']) * 100, 2), sharp_white_per100=round(ez['sharp_white'] / max(1e-9, ez['border_mm']) * 100, 2),
        round_share=round(ez['rounded'] / max(1, ez['corners']), 4), corners=ez['corners'], border_mm=round(ez['border_mm'], 1),
        supply_sharp_cu_per100=round(ev['sharp_cu'] / max(1e-9, ev['border_mm']) * 100, 2), supply_sharp_white_per100=round(ev['sharp_white'] / max(1e-9, ev['border_mm']) * 100, 2),
        supply_round_share=round(ev['rounded'] / max(1, ev['corners']), 4), supply_corners=ev['corners'], supply_border_mm=round(ev['border_mm'], 1))
erg['total'] = agg(LAYERS)
erg['outer'] = agg([L for L in LAYERS if L in OUTER])
erg['inner'] = agg([L for L in LAYERS if L not in OUTER])
json.dump(erg, open(os.path.join(D, 'k20.json'), 'w'), indent=1)
g = erg['total']
print(NAME, 'FINISHED %.0fs' % (time.time() - T0), json.dumps({k: g[k] for k in ('gnd_share', 'white15_mm2', 'white_own_mm2', 'gnd_islands', 'gnd_splitter_with', 'gnd_splitter_without', 'supply_net_layer', 'supply_pieces', 'zone_share', 'hole_share', 'sharp_cu_per100', 'round_share')}), 'R37', erg['r37'])
