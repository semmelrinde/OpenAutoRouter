from shapely.geometry import box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.errors import GEOSException
from common.parts import load_parts, circuit_board


def geometry_ok(g):
    return (isinstance(g, dict) and bool(g) and all(
        isinstance(p, BaseGeometry) and not p.is_empty and p.is_valid and p.area > 0 for p in g.values()))


def hull_box(geos):
    bs = [g.bounds for geo in geos.values() for g in geo.values() if not g.is_empty]
    if not bs: return None
    return (min(b[0] for b in bs), min(b[1] for b in bs),
            max(b[2] for b in bs), max(b[3] for b in bs))


def hull_area(geos):
    b = hull_box(geos)
    return (b[2] - b[0]) * (b[3] - b[1]) if b else 0.0


class HullTarget:
    def __init__(self, geos, refs=None):
        self.bounds = {}
        for r, geo in geos.items():
            if refs is None or r in refs: self.assign(r, geo)

    def assign(self, r, geo):
        self.bounds[r] = hull_box({r: geo})

    def surface(self, ch=None):
        new = {r: hull_box({r: g}) for r, g in (ch or {}).items()}
        bs = [new.get(r, b) for r, b in self.bounds.items() if new.get(r, b) is not None]
        if not bs: return 0.0
        return (max(b[2] for b in bs) - min(b[0] for b in bs)) * (max(b[3] for b in bs) - min(b[1] for b in bs))


def island_groups(K, refs, pinned=()):
    refs = set(refs)
    vacant = movable(K, refs, pinned)
    islands = K.get('islands')
    if not isinstance(islands, dict): raise ValueError('Netclass contract: islands missing')
    group_list = {}
    owner = {}
    for a, island in sorted(islands.items()):
        members = island.get('members') if isinstance(island, dict) else None
        if not isinstance(members, list) or any(not isinstance(r, str) for r in members):
            raise ValueError('Netclass contract: island members missing/invalid: ' + a)
        rs = {a, *members}
        if rs - refs: raise ValueError('Island parts without footprint: ' + ', '.join(sorted(rs - refs)))



        for r in sorted(rs):
            if r in owner: raise ValueError('Ambiguous island assignment: ' + r)
            owner[r] = a
        if len(rs) < 2 or not rs & vacant: continue
        group_list[a] = sorted(rs)
    return group_list


class IslandTarget:
    def __init__(self, geos, group_list):
        self.group_list = group_list
        self.from_ = {r: a for a, rs in group_list.items() for r in rs}
        self.bounds = {r: hull_box({r: geos[r]}) for r in self.from_}
        if any(b is None for b in self.bounds.values()): raise ValueError('Island without geometry')

    def assign(self, r, geo):
        if r in self.bounds: self.bounds[r] = hull_box({r: geo})

    def contribution(self, ch):
        new = {r: hull_box({r: g}) for r, g in ch.items() if r in self.from_}
        c = 0.0
        for a in sorted({self.from_[r] for r in new}):
            bs = [new.get(r, self.bounds[r]) for r in self.group_list[a]]
            c += max(b[2] for b in bs) - min(b[0] for b in bs)
            c += max(b[3] for b in bs) - min(b[1] for b in bs)
        return c


def island_measure(B, K, RG=None, layer_name=None, part_list=None, *, pinned=(), geos=None):
    meta = dict(version='p26-islands.v1', status='unknown', count=None,
                half_perimeter_mm=None, hull_mm2=None, packing_loss=None, group_list=[])
    fps = {f['ref']: f for f in B['fps']}
    try:
        islands = K.get('islands')
        if not isinstance(islands, dict): raise ValueError('Netclass contract: islands missing/invalid')
        T = part_list if part_list is not None else (load_parts(B, float((RG or {}).get('placer', {}).get('pad_spacing', 0.15)))
                                           if geos is None else None)
        gs = {}


        for a, island in islands.items():
            members = island.get('members') if isinstance(island, dict) else None
            if not isinstance(members, list) or any(not isinstance(r, str) for r in members):
                raise ValueError('Island members missing/invalid: ' + a)
            rs = {a, *members}
            if rs - fps.keys(): raise ValueError('Island parts without footprint: ' + ', '.join(sorted(rs - fps.keys())))
            for r in sorted(rs):
                if r in gs: continue
                g = geos.get(r, {}) if geos is not None else T[r].geo(
                    *(layer_name or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])))
                if not geometry_ok(g):
                    raise ValueError('Island part without complete valid geometry: ' + r)
                gs[r] = g
        group_list = island_groups(K, fps, pinned)
        out = {}
        for a, rs in group_list.items():
            group_geos = {r: gs[r] for r in rs}
            b = hull_box(group_geos)
            h = (b[2] - b[0]) * (b[3] - b[1])
            u = unary_union([g for geo in group_geos.values() for g in geo.values()]).area
            out[a] = dict(refs=rs, hull_mm2=h, occupied_mm2=u,
                          half_perimeter_mm=b[2] - b[0] + b[3] - b[1])
    except (KeyError, ValueError, GEOSException) as e:
        return dict(meta, faults=str(e))
    if not out: return dict(meta, status='not_applicable', count=0)
    h = sum(g['hull_mm2'] for g in out.values())
    u = sum(g['occupied_mm2'] for g in out.values())


    return dict(meta, status='ok', count=len(out), group_list=[dict(anchor=a, **g) for a, g in out.items()],
                hull_mm2=round(h, 6), half_perimeter_mm=round(sum(g['half_perimeter_mm'] for g in out.values()), 6),
                packing_loss=round(max(0.0, 1 - u / h), 6))


def measure(B, RG=None, layer_name=None, part_list=None, *, geos=None):
    T = part_list if part_list is not None else load_parts(B, float((RG or {}).get('placer', {}).get('pad_spacing', 0.15)))
    fps = {f['ref']: f for f in B['fps']}
    if geos is None:
        geos = {r: t.geo(*(layer_name or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r, t in T.items()}
    U = unary_union([g for geo in geos.values() for g in geo.values()])
    board = circuit_board(dict(B, drc_model=True), 0)[0]
    A = board.area
    return dict(version='density.v1', definition='Courtyard+pad spacing, projection F/B',
                board_mm2=round(A, 6), occupied_mm2=round(U.area, 6),
                hull_mm2=round(hull_area(geos), 6),
                overhang_mm2=round(U.difference(board).area, 6),
                density=round(U.intersection(board).area / A, 6) if A else None)


def against(B, original, RG=None):
    D = measure(B, RG); O = original if isinstance(original, dict) and 'board_mm2' in original else measure(original, RG)
    for k in ('board_mm2', 'occupied_mm2', 'hull_mm2', 'density'):
        D[k + '_rel_original'] = round(D[k] / O[k], 6) if O.get(k) else None
    return D


def movable(K, refs, pinned=()):
    refs = set(refs)
    kt = K.get('part_list') if isinstance(K, dict) else None
    if not isinstance(kt, dict): raise ValueError('Netclass contract: part_list missing')
    for r in sorted(refs):
        t = kt.get(r)
        if not isinstance(t, dict): raise ValueError('Netclass contract: entry missing: ' + r)
        if type(t.get('pinned')) is not bool: raise ValueError('Netclass contract: pinned missing/invalid: ' + r)
        if t.get('role') not in ('chip', 'small', 'connector', 'mechanics', 'without'):
            raise ValueError('Netclass contract: role missing/invalid: ' + r)
    return {r for r in refs if not kt[r]['pinned'] and r not in pinned
            and kt[r]['role'] not in ('mechanics', 'without')}


def inner(B, K, RG=None, layer_name=None, part_list=None, *, pinned=(), geos=None):
    fps = {f['ref']: f for f in B['fps']}
    meta = dict(version='p26-inner.v2', status='unknown', part_list=None, refs=None,
                hull_mm2=None, occupied_mm2=None, packing_loss=None,
                contour_fixed=bool(((K if isinstance(K, dict) else {}).get('specs') or {}).get('contour_fixed')))
    try:
        vacant = movable(K, fps, pinned)
    except ValueError as e:
        return dict(meta, faults=str(e))
    T = part_list if part_list is not None else load_parts(B, float((RG or {}).get('placer', {}).get('pad_spacing', 0.15)))
    if vacant - T.keys(): return dict(meta, faults='Free parts without geometry: ' + ', '.join(sorted(vacant - T.keys())))
    refs = sorted(vacant)
    gs = {r: geos.get(r, {}) if geos is not None else
          T[r].geo(*(layer_name or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r in refs}
    for r, geo in gs.items():
        if not geometry_ok(geo):
            return dict(meta, faults='Free part without complete valid geometry: ' + r)
    A = hull_area(gs)
    U = unary_union([g for geo in gs.values() for g in geo.values()])
    return dict(meta, status='ok', part_list=len(refs), refs=refs, hull_mm2=round(A, 6),
                occupied_mm2=round(U.area, 6),
                packing_loss=round(max(0.0, 1 - U.area / A), 6) if A else None)


def diagnose(B, K, RG=None, layer_name=None, part_list=None, *, pinned=()):
    contract = B.get('p26_fixed')
    if contract is not None:
        if contract.get('version') != 'p26-pinned-v1':
            raise ValueError('P26 fixed-part contract unknown')
        pinned = set(pinned) | set(contract['refs'])



        try:
            movable(K, (f['ref'] for f in B['fps']))
        except ValueError:


            pass
        else:
            K = dict(K, part_list={r: dict(t, pinned=r in pinned) if isinstance(t, dict) else t
                               for r, t in K['part_list'].items()})
    try:
        T = part_list if part_list is not None else load_parts(B, float((RG or {}).get('placer', {}).get('pad_spacing', 0.15)))
        fps = {f['ref']: f for f in B['fps']}
        geos = {r: t.geo(*(layer_name or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r, t in T.items()}
        return dict(total=measure(B, RG, layer_name, T, geos=geos),
                    inner=inner(B, K, RG, layer_name, T, pinned=pinned, geos=geos),
                    islands=island_measure(B, K, RG, layer_name, T, pinned=pinned, geos=geos))
    except GEOSException as e:

        failure = dict(status='unknown', faults=repr(e)[:200], hull_mm2=None,
                       occupied_mm2=None, packing_loss=None)
        return dict(total=dict(failure, version='density.v1'),
                    inner=dict(failure, version='p26-inner.v2', refs=None, part_list=None),
                    islands=dict(failure, version='p26-islands.v1', count=None, half_perimeter_mm=None, group_list=[]))

def tighter_contour(B, geos, border, pinned=False):
    if pinned: return None, 'contour_fixed'
    if any(B.get('outline_holes') or []): return None, 'Excerpt'
    if any(f['ref'] not in geos for f in B['fps']): return None, 'Part without collision geometry'
    if B.get('keepouts') or B.get('copper_graphic'): return None, 'KeepoutArea/Graphic'
    board = circuit_board(B, 0)[0]; b = board.bounds
    if len(B['outline']) != 1 or board.symmetric_difference(box(*b)).area > 1e-6:
        return None, 'not a simple rectangle'
    h = hull_box(geos)
    if h is None: return None, 'no parts'


    import math
    x0 = max(b[0], math.floor((h[0] - border - 0.001) * 1000 + 1e-9) / 1000)
    y0 = max(b[1], math.floor((h[1] - border - 0.001) * 1000 + 1e-9) / 1000)
    x1 = min(b[2], math.ceil((h[2] + border + 0.001) * 1000 - 1e-9) / 1000)
    y1 = min(b[3], math.ceil((h[3] + border + 0.001) * 1000 - 1e-9) / 1000)
    if x1 <= x0 or y1 <= y0 or (x1 - x0) * (y1 - y0) >= board.area - 1e-6:
        return None, 'no free edge'
    return [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]], 'trimmed'


def group_paths(B, K, layer_name=None, part_list=None):
    from analysis.p26_group_paths import measure
    return measure(B, K, layer_name, part_list)
