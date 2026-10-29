import copy, hashlib, json, math, sys
from collections import OrderedDict
from threading import RLock

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

VERSION = 'r39-copper_map-v4.3'
QS = 32
KINDS = ('current_val', 'supply', 'sw')
CACHE = OrderedDict()
CACHE_LOCK = RLock()
CACHE_N = 8
STEP = .1


def learned_values(e):
    if not e: return None
    if (e.get('coverage') or {}).get('components') == 0:
        return dict(version=e.get('version'), coverage=e.get('coverage'))
    if e.get('not_measurable') or not e.get('coverage', {}).get('complete', True):
        out = dict(version=e.get('version'), coverage=e.get('coverage') or
                   dict(complete=False, unknown=len(e.get('not_measurable') or [])))


        if (e.get('measured_paths') or {}).get('components', 0) > 0: out['measured_paths'] = copy.deepcopy(e['measured_paths'])
        return out
    return e


def _bands(fpath, W, s, vacant, slot):


    yes, no = [], []
    for line in getattr(fpath, 'geoms', [fpath]):
        if line.geom_type != 'LineString': continue
        coords = list(line.coords)
        for j, (a, b) in enumerate(zip(coords[:-1], coords[1:])):
            length = math.dist(a, b)
            if length <= 1e-12: continue
            dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length

            seg = LineString([a, b])
            if not seg.intersects(s.buffer(.02, QS)): continue
            count = max(1, math.ceil(length / STEP))
            for i in range(count):
                lo, hi = length*i/count, length*(i+1)/count
                c = Point(a[0]+dx*(lo+hi)/2, a[1]+dy*(lo+hi)/2)
                if c.distance(s) > .02 + (hi-lo)/2: continue
                circ = c.buffer(W/2, QS)
                ratio = circ.intersection(vacant).area / circ.area
                l = lo-W/2 if j == 0 and i == 0 else lo
                h = hi+W/2 if j == len(coords)-2 and i == count-1 else hi
                band = LineString([(a[0]+dx*l,a[1]+dy*l),(a[0]+dx*h,a[1]+dy*h)]).buffer(W/2, cap_style=2)
                (yes if ratio >= slot else no).append(band)
    needed = s.intersection(unary_union(no))
    unneeded = s.intersection(unary_union(yes)).difference(needed)
    return unneeded, needed


def _via_d(R):
    try: return float(str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')[0])
    except ValueError: return 0.45


def _layers(B):
    nl = int(B.get('cu_layers') or 2)
    return ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']

def copper_map(B, R, L):
    from learning.copper_bars import copper_map as total_copper
    return [(n, g) for (n, layer), g in total_copper(B, R).items() if layer == L]


def bottlenecks(B, R, K, gnd_layers=None, clr=None, border=0.25, slot=0.85, w_max=None, min_area=0.005):


    args = dict(gnd_layers=gnd_layers, clr=clr, border=border, slot=slot, w_max=w_max, min_area=min_area)
    key = hashlib.sha256(json.dumps([VERSION, B, R, K, args], sort_keys=True, allow_nan=False).encode()).digest()
    with CACHE_LOCK:
        if key in CACHE:
            CACHE.move_to_end(key)
            return copy.deepcopy(CACHE[key])
    out = _bottlenecks(B, R, K, **args)
    with CACHE_LOCK:
        CACHE[key] = copy.deepcopy(out)
        while len(CACHE) > CACHE_N: CACHE.popitem(load=False)
    return out


def _bottlenecks(B, R, K, gnd_layers=None, clr=None, border=0.25, slot=0.85, w_max=None, min_area=0.005):
    from common.zones import board_geometry
    from learning.copper_bars import copper_map as total_copper
    from learning.copper_bars import track_geometry
    from learning.current_zones import keepout_area
    B = board_geometry(B, R)
    from shapely import set_precision
    cu = {key: set_precision(set_precision(g, 1e-6).simplify(2e-6, preserve_topology=True).normalize(), 0)
          for key, g in total_copper(B, R).items()}
    from common.arc import points
    centerlines = {}
    track_copper = {}
    for t in R.get('tracks', []):
        track_copper.setdefault((t.get('net', ''), t['layer']), []).append(track_geometry(t))
        if 'items' in t:
            paths = [it[1:3] if it[0] == 'L' else points(*it[1:4]) for it in t['items']]
        else: paths = [t['pts']]
        centerlines.setdefault((t.get('net', ''), t['layer']), []).extend(
            LineString(ps) for ps in paths if len(ps) >= 2)
    gl = gnd_layers if gnd_layers is not None else (R.get('gnd_layers') or (R.get('options') or {}).get('gnd_layers') or [])
    gl = {str(x).split('=')[0] for x in (gl if isinstance(gl, list) else str(gl).split(',')) if x}
    gnd = set(K.get('gnd') or ['GND'])
    clr = clr if clr is not None else float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1)
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, QS)
    net_list = {n: v for n, v in (K.get('net_list') or {}).items() if v.get('category') in KINDS and n not in gnd}
    zone_layers = {(z['net'], z['layer']) for key in ('zone_list', 'zones') for z in R.get(key, [])}

    from common.zones import unify
    zone_copper = {}
    for z in unify(R).get('zones', []):
        zone_copper.setdefault((z['net'], z['layer']), []).append(z)
    source_list = {n: {key: v.get(key) for key in ('category', 'reason', 'w', 'w_source', 'current_A', 'current_A_source')}
               for n, v in sorted(net_list.items())}

    try: drill = float(str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')[1])
    except (ValueError, IndexError): drill = .2
    hole = float(R.get('h2h', (R.get('options') or {}).get('h2h', .2)))
    if not math.isfinite(hole) or hole < 0: raise ValueError('R39: invalid drill spacing')
    unn, required, not_measurable, needed_area = [], 0, [], 0.0
    unn_measured = []
    components, measured = 0, 0
    for L in _layers(B):
        if L in gl: continue
        part_list = [(n, g) for (n, layer), g in cu.items() if layer == L]
        hole_list = [(p.get('net', ''), Point(p['x'], p['y']).buffer(float(p['drill']) / 2, QS))
                   for p in B['pads'] if p.get('drill')]
        hole_list += [(v.get('net', ''), Point(v['x'], v['y']).buffer(float(v.get('drill') or drill) / 2, QS))
                    for v in R.get('vias', []) if L in (v.get('layers') or _layers(B))]
        for n, v in sorted(net_list.items()):
            owned = [g for m, g in part_list if m == n]
            if not owned: continue
            C = unary_union(owned)
            W = float(v.get('w') or 0.3)
            if w_max is not None: W = min(w_max, W)
            if not math.isfinite(W) or W <= 0: raise ValueError('R39: invalid target width ' + n)
            pads = unary_union([Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n
                                for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3])

            radius = max(0, W / 2 - .005)
            from shapely import set_precision
            core_set = set_precision(set_precision(C.buffer(-radius, QS), 1e-6), 0)
            thin = C.difference(core_set.buffer(radius, QS))
            if not pads.is_empty: thin = thin.difference(pads.buffer(0.02, QS))




            fpath = unary_union(centerlines.get((n, L), []))
            covered, fully_measured = [], []
            for component in getattr(C, 'geoms', [C]):
                if component.geom_type != 'Polygon': continue
                has_path = not fpath.is_empty and fpath.intersects(component)

                if not has_path and (n, L) not in zone_layers: continue


                zones = [Polygon(z['pts'], z.get('holes', [])).buffer(0) for z in zone_copper.get((n, L), [])
                         if z['net'] == n and z['layer'] == L and z.get('filled')]
                if not has_path and not any(z.intersection(component).area > 1e-9 for z in zones): continue
                components += 1
                if has_path:
                    covered.append(component)

                    broad = component.buffer(-radius, join_style=2).buffer(radius, join_style=2)


                    known = broad.union(unary_union(track_copper.get((n, L), [])).buffer(.02, QS)).union(pads.buffer(.02, QS))
                    unknown = component.difference(known)
                    if unknown.area >= min_area:
                        not_measurable.append(dict(net_name=n, layer_name=L, reason='Zone side branch outside the routed centerline',
                                                 bounds=list(unknown.bounds), area_mm2=round(unknown.area, 4)))
                    else:
                        measured += 1
                        fully_measured.append(component)
                else:
                    not_measurable.append(dict(net_name=n, layer_name=L, reason='Copper component without routed centerline',
                                             bounds=list(component.bounds), area_mm2=round(component.area, 4)))
            thin = thin.intersection(unary_union(covered))
            known_components = unary_union(fully_measured).buffer(1e-9)
            pieces = [s for s in getattr(thin, 'geoms', [thin]) if s.geom_type == 'Polygon' and s.area >= min_area]
            if not pieces: continue
            foreign = unary_union([g for m, g in part_list if m != n])
            drills = unary_union([g for m, g in hole_list if m != n])
            vacant = contour.difference(foreign.buffer(clr, QS)).difference(keepout_area(B, L, 'areas'))
            vacant = vacant.difference(drills.buffer(hole, QS)).union(C)
            for s in pieces:
                neck = s.intersection(fpath.buffer(.02, QS))
                if neck.is_empty: continue
                yes, needed = _bands(fpath, W, s, vacant, slot)
                for piece in getattr(yes, 'geoms', [yes]):
                    if piece.geom_type != 'Polygon' or piece.area < min_area: continue
                    c = piece.representative_point()
                    unn.append(dict(net_name=n, layer_name=L, x=round(c.x, 3), y=round(c.y, 3),
                                    area_mm2=round(piece.area, 4), nominal_mm=round(W, 3)))
                    if known_components.covers(piece): unn_measured.append(unn[-1])
                for piece in getattr(needed, 'geoms', [needed]):
                    if piece.geom_type == 'Polygon' and piece.area >= min_area:
                        required += 1; needed_area += piece.area
    return dict(version=VERSION, parameter=dict(clr=clr, border=border, slot=slot, w_max=w_max, h2h=hole,
                min_area=min_area, step_mm=STEP, core_precision_mm=1e-6, copper_precision_mm=1e-6, copper_smoothed_mm=2e-6, gnd_layers=sorted(gl), tolerance_mm=.005,
                target_widths={n: min(w_max, float(v.get('w') or .3)) if w_max is not None else float(v.get('w') or .3) for n, v in sorted(net_list.items())}),
                coverage=dict(complete=not not_measurable, components=components, measurable=measured,
                               unknown=len(not_measurable), share=measured/components if components else 1.0),
                measured_paths=dict(version='r39-paths-v3', components=measured,
                    area_mm2=round(sum(u['area_mm2'] for u in unn_measured), 3), count=len(unn_measured),
                    perimeter='Only fully measurable copper components with routed centerline; no unknown distribution zones/side branches'),
                unnecessary=unn, unnecessary_n=len(unn), unnecessary_mm2=round(sum(u['area_mm2'] for u in unn), 3),
                required=required, required_mm2=round(needed_area, 3), not_measurable=not_measurable, net_list=source_list)



def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); p = [x for x in a if '=' not in x]
    B, R, K = (json.load(open(f)) for f in p[:3])
    E = bottlenecks(B, R, K, kw.get('gnd_layers', '').split(',') if kw.get('gnd_layers') else None)
    print(json.dumps(dict(E, unnecessary=E['unnecessary'][:15]), indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
