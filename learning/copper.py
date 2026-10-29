from shapely.geometry import Polygon
from shapely.ops import unary_union

COPPER = {'F': (196, 120, 60), 'B': (70, 120, 200)}
INNER = [(60, 160, 100), (205, 160, 40), (150, 90, 200), (40, 165, 175), (200, 95, 130), (120, 130, 50)]
REASON = (24, 36, 30)


def color(L):
    if L in COPPER: return COPPER[L]
    try: return INNER[(int(L[2:]) - 1) % len(INNER)]
    except (ValueError, IndexError): return (150, 150, 150)


def _via_ground(R, K=None):
    from analysis.copper_model import via_geo
    return via_geo(R, K)


def _gnd(B, R, K=None):
    gl = R.get('gnd_layers') or []
    gl = [str(q).split('=')[0] for q in (gl if isinstance(gl, list) else str(gl).split(',')) if q]
    net_list = {p['net'] for p in B['pads'] if p['net']}
    gnd = set(K['gnd']) if K is not None and 'gnd' in K else {n for n in net_list if n.lstrip('/').upper() in ('GND', 'PGND', 'AGND', 'DGND', 'GNDA', 'GNDD')}
    return gl, gnd


def copper_layers(B, R, clr=None, border=None, min_width=0.2, fill=True, K=None, design_rules=None):
    from common.zones import board_geometry
    B = board_geometry(B, R)
    nl = int(B.get('cu_layers') or 2)
    cu_layers = ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']
    from analysis.copper_model import settings, areas, baseline, geometry, drills, keepout_area, attached
    clr, border = settings(K, design_rules, clr, border)
    planes, gl, gnd = areas(B, R, K)
    d_via, b_via = _via_ground(R, K)
    fills = {L: Polygon() for L in cu_layers}
    if fill:

        all_items = {}
        for net in sorted(set(planes.values()) | gnd):
            candidates = {}
            for L in cu_layers:
                if not (planes.get(L) == net or (L not in gl and net in gnd)): continue
                f = baseline(B, R, L, {net}, clr=clr, border=border, via_d=d_via, drill=b_via)
                if L not in gl: f = f.buffer(-min_width / 2, 8).buffer(min_width / 2, 8)
                candidates[L] = f
            all_items[net] = candidates


        from shapely import voronoi_polygons
        from shapely.geometry import MultiPoint
        for L in cu_layers:
            active = [n for n, fs in all_items.items() if L in fs and not fs[L].is_empty]
            if len(active) < 2: continue
            points = {}
            for n, g in geometry(B, R, L, d_via):
                if n in active and not g.is_empty:
                    q = g.representative_point(); xy = (q.x, q.y)
                    if xy in points and points[xy] != n:
                        raise ValueError('Separate grounds have the same copper anchor')
                    points[xy] = n
            if len(set(points.values())) < 2:

                for n in active:
                    if n not in points.values(): all_items[n][L] = Polygon()
                continue
            coords = sorted(points)
            grid_cells = voronoi_polygons(MultiPoint(coords), extend_to=unary_union([all_items[n][L] for n in active]).envelope, ordered=True)
            roles = {n: [] for n in active}
            for xy, cell in zip(coords, grid_cells.geoms): roles[points[xy]].append(cell)
            for n in active:
                region = unary_union(roles[n]).buffer(-clr / 2, join_style=2)
                all_items[n][L] = all_items[n][L].intersection(region)
        for net, candidates in all_items.items():
            joined = attached(B, R, candidates, {L: net for L in candidates}, via_d=d_via, drill=b_via)
            for L, f in joined.items(): fills[L] = fills[L].union(f)
    out = {}
    for L in cu_layers:
        bore = drills(B, R, b_via, L)
        part_list = geometry(B, R, L, d_via)
        copper_map = unary_union([g for n, g in part_list]) if part_list else Polygon()
        copper_map = copper_map.union(fills[L])
        for k in B.get('keepouts', []):
            if (k.get('prohibitions') or {}).get('areas') and L in (k.get('cu_layers') or cu_layers):
                keepout = keepout_area(k)
                copper_map = copper_map.difference(keepout.difference(unary_union([g for n, g in part_list])))
        out[L] = (copper_map.difference(bore), bore)
    return out


def image_layer(B, R, L, geom, goal, width=420, ss=3, title=None):
    from PIL import Image, ImageDraw
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0, x1, y1 = min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5
    S = width * ss / max(1e-6, x1 - x0); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    in_ = Image.new('RGB', (W, H), (245, 245, 242)); dr = ImageDraw.Draw(in_)
    tr = lambda x, y: ((x - x0) * S, (y - y0) * S)
    copper_map, bore = geom; c = color(L)
    def poly(g, f, hole_size):

        for p in sorted(getattr(g, 'geoms', [g]), key=lambda p: -p.area):
            if p.is_empty or p.geom_type != 'Polygon': continue
            dr.polygon([tr(*q) for q in p.exterior.coords], fill=f)
            for i in p.interiors: dr.polygon([tr(*q) for q in i.coords], fill=hole_size)
    from analysis.copper_model import contour
    poly(contour(B), REASON, (245, 245, 242))
    poly(copper_map, c, REASON)
    poly(bore, (10, 10, 10), (10, 10, 10))


    for holes in B.get('outline_holes') or []:
        for h in holes:
            if len(h) >= 3: dr.polygon([tr(*q) for q in h], fill=(245, 245, 242))
    from learning.gallery import _ratsnest
    _ratsnest(dr, R, tr, ss)
    in_ = in_.resize((W // ss, H // ss), Image.LANCZOS)
    ImageDraw.Draw(in_).text((4, 3), title or L, fill=(255, 255, 255))
    in_.save(goal, optimize=True)
