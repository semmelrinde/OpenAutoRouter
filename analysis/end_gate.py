import hashlib, json, math, os, sys, time

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon

VERSION = 'end_gate-v1'
TOL = 0.0002
_CACHE = {}


def active(R):
    o = R.get('options') or {}
    return bool(o.get('netclasses')) and str(o.get('end_gate', '1')) != '0' and os.environ.get('END_GATE', '1') != '0'


def _f(*value_list, fallback=0.0):
    w = [float(v) for v in value_list if v is not None and v != '']
    return max(w) if w else fallback


def cu_layers(R, B):
    n = int(_f((R.get('options') or {}).get('cu_layers'), B.get('cu_layers') if isinstance(B.get('cu_layers'), (int, float)) else None, fallback=2))
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def _arc(it, faults=1e-4):
    from common.track_items import arc_geometry
    c, r, a0, sw = arc_geometry(it)
    n = max(2, int(math.ceil(abs(sw) / (2 * math.acos(max(-1.0, 1 - faults / max(r, 1e-9)))))) + 1)
    return [(c[0] + r * math.cos(a0 + sw * k / n), c[1] + r * math.sin(a0 + sw * k / n)) for k in range(n + 1)]


def _track_pieces(t):
    if t.get('items'):
        for it in t['items']:
            if it[0] == 'L': yield (LineString([it[1], it[2]]) if it[1] != it[2] else Point(it[1])), float(it[3])
            elif it[0] == 'A': yield LineString(_arc(it)), float(it[4])
        return
    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
        yield (LineString([a, b]) if a != b else Point(a)), float(w)


def _pad_type(p):
    if p.get('graphic') or p.get('board_copper') or str(p.get('ref', '')).startswith('CU_'): return 'Graphic', ''
    return 'Pad', p.get('dru_pad_type') or ('NPTH, mechanical' if p.get('pad_type') == 'npth' else 'Through-hole' if p.get('drill') else 'SMD')


def limits(B, R, K):
    op = R.get('options') or {}; rg = (K or {}).get('design_rules') or {}; rg = rg.get('router', rg)
    raw_data = ((K or {}).get('design_rules') or {}).get('raw_data') or {}; de = B.get('design') or {}
    return dict(drc=_f(op.get('drc'), rg.get('drc'), raw_data.get('min_clearance'), de.get('min_spacing'), fallback=0.127),
                pad=_f(op.get('drcpad'), rg.get('drcpad')),
                edge_clr=_f(op.get('edge'), rg.get('edge'), raw_data.get('min_copper_edge_clearance'), de.get('border')),
                hole_size=_f(op.get('h2h'), rg.get('h2h'), op.get('hole_clr'), raw_data.get('min_hole_clearance'), de.get('hole_spacing')),
                h2h=_f(raw_data.get('min_hole_to_hole'), de.get('hole_hole'), fallback=_f(op.get('h2h'), rg.get('h2h'))))


def _spacing(B, R, K, g):
    from analysis.spacings import Spacing, contract_spec
    c = R.get('spacings') or (K or {}).get('spacings')
    try:
        c = dict(c) if c else contract_spec(B, (K or {}).get('design_rules') or {})
        c['board_min'] = max(float(c.get('board_min') or 0), g['drc'])
        if g['pad'] > 0:
            c['dru'] = [dict(name='End gate pad clearance', cond="A.Type == 'Pad' || B.Type == 'Pad'",
                             constraints=dict(clearance=dict(min=g['pad'])))] + list(c.get('dru') or [])
        return Spacing(c), None
    except (ValueError, KeyError, TypeError) as e:
        return None, repr(e)[:200]


def _key(B, R, K):
    part_list = [R.get(k) for k in ('tracks', 'vias', 'zones', 'options', 'spacings')] + [B.get(k) for k in ('pads', 'outline', 'outline_holes', 'design', 'net_classes', 'netclass')]
    part_list.append((K or {}).get('design_rules')); part_list.append((K or {}).get('spacings'))
    return hashlib.sha1(json.dumps(part_list, sort_keys=True, default=str).encode()).hexdigest()


def verify(B, R, K=None, max_examples=30):
    key = _key(B, R, K)
    if key in _CACHE: return _CACHE[key]
    t0 = time.process_time()
    g = limits(B, R, K); L = cu_layers(R, B); op = R.get('options') or {}
    vd, vdr = [float(v) for v in str(op.get('via') or '0.45/0.20').split('/')]
    ab, contract_error = _spacing(B, R, K, g)

    def limit(n1, n2, layer_name, t1, t2, p1='', p2=''):
        if ab is None: return g['pad'] if 'Pad' in (t1, t2) else g['drc']
        n1 = '' if str(n1).startswith('#') else n1; n2 = '' if str(n2).startswith('#') else n2
        return ab.pair(n1, n2, layer_name + '.Cu', t1, t2, p1, p2)[0]


    E = {l: [] for l in L}; bore = []
    for t in R['tracks']:
        if t.get('layer') not in E: continue
        for core_set, w in _track_pieces(t): E[t['layer']].append(('Track', t['net'], core_set, w / 2, ''))
    for v in R['vias']:
        d = float(v.get('d') or vd); dr = float(v.get('drill') or v.get('dr') or vdr); p = Point(v['x'], v['y'])
        for l in (v.get('layers') or L):
            if l in E: E[l].append(('Via', v.get('net', ''), p, d / 2, ''))
        bore.append(('Via', v.get('net', ''), p, dr / 2))
    for z in R.get('zones') or []:
        if z.get('teardrop') and z.get('layer') in E and len(z.get('pts') or []) >= 3:
            E[z['layer']].append(('Track', z['net'], Polygon(z['pts']), 0.0, ''))
    from common.drills import pad_drill_hole
    for p in B['pads']:
        type_, pt = _pad_type(p)
        for l, polys in (p.get('polys') or {}).items():
            if l not in E: continue
            for q in polys:
                if len(q) >= 3: E[l].append((type_, p.get('net') or '', Polygon(q).buffer(0), 0.0, pt))
        try:
            with np.errstate(divide='ignore', invalid='ignore'): h = pad_drill_hole(p)
        except ValueError: h = None
        if h is not None: bore.append(('PadHole' if h['circle'] else 'Slot', p.get('net') or '', h['geo'], 0.0))

    faults = {}
    def report(category, layer_name, n1, n2, d, lim, geo1, geo2):
        line = shapely.shortest_line(geo1, geo2); a, b = shapely.get_point(line, 0), shapely.get_point(line, -1)
        x, y = (a.x + b.x) / 2, (a.y + b.y) / 2
        k = (category, layer_name, tuple(sorted((str(n1), str(n2)))), round(x / 0.05), round(y / 0.05))
        if k in faults and faults[k]['spacing'] <= d: return
        faults[k] = dict(category=category, layer_name=layer_name, net_list=[n1, n2], spacing=round(float(d), 4), rule=round(float(lim), 4), x=round(x, 3), y=round(y, 3))


    border = None
    if B.get('outline'):
        hl = B.get('outline_holes') or []
        board = shapely.union_all([Polygon(q, hl[i] if i < len(hl) else []) for i, q in enumerate(B['outline'])])
        border = board.boundary
    for l in L:
        G = E[l]
        if not G: continue
        core_set = np.array([e[2] for e in G], dtype=object); h = np.array([e[3] for e in G])
        net_name = [e[1] for e in G]; pad = np.array([e[0] in ('Pad', 'Graphic') for e in G])

        tree = shapely.STRtree(core_set)
        rich = (ab.maximum if ab is not None else max(g['drc'], g['pad'])) + h.max() + 0.01
        qi, qj = tree.query(core_set, predicate='dwithin', distance=h + rich)
        m = (qj > qi) & ~(pad[qi] & pad[qj])
        qi, qj = qi[m], qj[m]
        if len(qi):
            d = shapely.distance(core_set[qi], core_set[qj]) - h[qi] - h[qj]
            for i, j, dij in zip(qi.tolist(), qj.tolist(), d.tolist()):
                if net_name[i] == net_name[j] and net_name[i]: continue
                ei, ej = G[i], G[j]
                lim = limit(ei[1], ej[1], l, ei[0], ej[0], ei[4], ej[4])
                if dij < lim - TOL: report('clearance', l, ei[1], ej[1], dij, lim, ei[2], ej[2])

        if border is not None and g['edge_clr'] > 0:
            net_copper = np.nonzero(~pad)[0]
            if len(net_copper):
                d = shapely.distance(core_set[net_copper], border) - h[net_copper]
                off = ~shapely.covers(board, core_set[net_copper])
                for k, dk, a in zip(net_copper.tolist(), d.tolist(), off.tolist()):
                    if a or dk < g['edge_clr'] - TOL: report('edge', l, G[k][1], 'Edge', -dk if a else dk, g['edge_clr'], G[k][2], border)

        if bore and g['hole_size'] > 0:
            hk = np.array([b[2] for b in bore], dtype=object); hr = np.array([b[3] for b in bore])
            ti, tj = tree.query(hk, predicate='dwithin', distance=hr + h.max() + g['hole_size'] + 0.01)
            for i, j in zip(ti.tolist(), tj.tolist()):
                b, e = bore[i], G[j]
                if (b[1] == e[1] and b[1]) or (b[0] != 'Via' and e[0] in ('Pad', 'Graphic')): continue
                if b[0] == 'Via' and e[0] == 'Via' and e[2].equals(b[2]): continue
                dij = shapely.distance(b[2], e[2]) - b[3] - e[3]
                if dij < g['hole_size'] - TOL: report('hole_clearance', l, e[1], b[1], dij, g['hole_size'], e[2], b[2])

    if len(bore) > 1 and g['h2h'] > 0:
        hk = np.array([b[2] for b in bore], dtype=object); hr = np.array([b[3] for b in bore])
        ti, tj = shapely.STRtree(hk).query(hk, predicate='dwithin', distance=hr + hr.max() + g['h2h'] + 0.01)
        for i, j in zip(ti.tolist(), tj.tolist()):
            if j <= i or 'Via' not in (bore[i][0], bore[j][0]): continue
            dij = shapely.distance(hk[i], hk[j]) - hr[i] - hr[j]
            if dij < -TOL: report('hole_overlap', '*', bore[i][1], bore[j][1], dij, 0.0, hk[i], hk[j])
            elif 'Slot' not in (bore[i][0], bore[j][0]) and dij < g['h2h'] - TOL:
                report('hole_to_hole', '*', bore[i][1], bore[j][1], dij, g['h2h'], hk[i], hk[j])
    faults = list(faults.values()); kinds = {}
    for f in faults: kinds[f['category']] = kinds.get(f['category'], 0) + 1
    faults.sort(key=lambda f: (f['spacing'] - f['rule'], f['category'], f['layer_name'], f['x'], f['y']))
    T = dict(version=VERSION, faults=len(faults), kinds=dict(sorted(kinds.items())), examples=faults[:max_examples],
             limits={k: round(v, 4) for k, v in g.items()}, contract_spec=ab is not None, cpu_s=round(time.process_time() - t0, 3))
    if contract_error: T['contract_error'] = contract_error
    if len(_CACHE) > 32: _CACHE.clear()
    _CACHE[key] = T
    return T


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    B, R = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    K = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else None
    print(json.dumps(verify(B, R, K), indent=1, ensure_ascii=False))
