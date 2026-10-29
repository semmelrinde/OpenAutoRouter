import collections, heapq, json, math, os, re, sys, time

import numpy as np
import shapely
from shapely.geometry import Polygon, Point

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VERSION = 'v3.1-review_fix-r3'
ER = 4.1
T_OUTER, T_INNER = 0.035, 0.0175
PS_MM = 3.336
RHO = 1.72e-5
MU0 = 1.2566
L_FREE = 1.0
THICKNESS = 1.6
PREPREG = 0.0994
STRUCTURE_JLC = {2: [1.5], 4: [0.0994, 1.265, 0.0994], 6: [0.0994, 0.55, 0.109, 0.55, 0.0994]}
ZCLR = 0.15
SAMPLE = 0.25
GAP_MIN = 1.0
GAP_NH = 5.0
S_MAX = 30.0
VIA_SLEEVE = 0.02
ANTIPAD = 0.4
J_L_WHOLE = 1.0
J_NECK_L, J_NECK_D = 2.0, 1.0
TOL = 0.03
I_MIN_LOAD = 0.01


def stack_names(n):
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def dielectrics(n):
    if n in STRUCTURE_JLC: return list(STRUCTURE_JLC[n])
    k = n - 1; pp = [i for i in range(k) if i % 2 == 0]; core_set = [i for i in range(k) if i % 2 == 1]
    rest = THICKNESS - 2 * T_OUTER - (n - 2) * T_INNER - PREPREG * len(pp)
    return [PREPREG if i % 2 == 0 else max(0.1, rest / max(1, len(core_set))) for i in range(k)]


def structure(n_layers, levels, diel=None, er=ER):
    S = stack_names(n_layers); D = list(diel) if diel else dielectrics(n_layers)
    z = [0.0]
    for d in D: z.append(z[-1] + d)
    E = [L for L in S if L in set(levels)]
    cu_layers = {}
    for i, L in enumerate(S):
        t = T_OUTER if L in ('F', 'B') else T_INNER
        ob = [j for j in range(i) if S[j] in E]; un = [j for j in range(i + 1, len(S)) if S[j] in E]
        h1 = z[i] - z[ob[-1]] if ob else None; h2 = z[un[0]] - z[i] if un else None
        if L in E: type_, ref, h, H = 'plane', L, 0.0, 0.0
        elif h1 is None and h2 is None: type_, ref, h, H = 'open_items', None, None, None
        else:
            ref = S[ob[-1]] if (h2 is None or (h1 is not None and h1 <= h2)) else S[un[0]]
            h = min(x for x in (h1, h2) if x is not None)
            if L in ('F', 'B'): type_ = 'micro'
            else: type_ = 'strip2' if (h1 is not None and h2 is not None) else 'strip1'
            H = h1 * h2 / (h1 + h2) if (h1 is not None and h2 is not None) else h
        cu_layers[L] = dict(i=i, z=z[i], t=t, type_=type_, ref=ref, h=h, h1=h1, h2=h2, H=H)
    return dict(stack=S, z=z, dielectrics=D, levels=E, er=er, cu_layers=cu_layers, thickness=z[-1])


def line(A, L, w):
    g = A['cu_layers'][L]; er = A['er']; t = g['t']
    if g['type_'] == 'micro':
        Z = 87 / math.sqrt(er + 1.41) * math.log(max(1.05, 5.98 * g['h'] / (0.8 * w + t)))
        tpd = math.sqrt(0.475 * er + 0.67) * PS_MM
    elif g['type_'] in ('strip1', 'strip2'):
        b = (g['h1'] + g['h2'] + t) if g['type_'] == 'strip2' else 2 * g['h'] + t
        Z = 60 / math.sqrt(er) * math.log(max(1.05, 4 * b / (0.67 * math.pi * (0.8 * w + t))))
        tpd = math.sqrt(er) * PS_MM
    else:
        tpd = math.sqrt(0.475 * er + 0.67) * PS_MM; Z = 1000 * L_FREE / tpd
    Z = max(Z, 5.0)
    return Z, tpd, Z * tpd / 1000.0, tpd / Z


def l_plane(A, L, w):
    g = A['cu_layers'].get(L)
    if g is None or g['type_'] in ('open_items',) or not g.get('h'): return L_FREE
    if g['type_'] == 'plane': return 0.0
    return min(MU0 * g['h'] / (w + 2 * g['h']), line(A, L, w)[2] if w < 3 * g['h'] else 1e9)


def via_lc(h, d, A, d_pad):
    h = max(h, 0.05)
    return 0.2 * h * (math.log(4 * h / d) + 1), 0.0555 * A['er'] * A['thickness'] * d_pad / ANTIPAD


def width_ipc(I, inner=False, t=None, model=None):
    if not I or I <= 0: return 0.0
    if model == '2221':
        from analysis.current_width import ipc2221
        return ipc2221(I, t=t if t is not None else (T_INNER if inner else T_OUTER), inner=inner)
    A_mil2 = (I / (0.048 * 10 ** 0.44)) ** (1 / 0.725)
    return A_mil2 / 1.378 * 0.0254 * (2.3 if inner else 1.0)


def load_weights(fpath=None, si=False):
    fpath = fpath or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'measures_weights_si.json' if si else 'measures_weights.json')
    with open(fpath) as f: G = json.load(f)

    if G.get('version') == 'v3.2':
        standard = load_weights()
        for key in ('minimax', 'profile'):
            G.setdefault(key, standard[key])
    return G


def net_class(k):
    a = (k or {}).get('category', 'signal'); r = (k or {}).get('rank', 0) or 0
    if a == 'fast': return 'fast_r5' if r >= 5 else ('fast_r4' if r == 4 else 'fast_r3')
    if a == 'sw': return 'sw'
    if a in ('current_val', 'supply'): return 'current_val'
    if a in ('analog', 'kelvin'): return 'analog'
    if a == 'gnd': return 'gnd'
    return 'fast_r3' if r >= 3 else ('slow' if r >= 1 else 'gpio')


def penalty(x, bend):
    return x if x <= 1 else 1 + bend * (x - 1)


def _mst(pts):
    if len(pts) < 2: return 0.0
    inT = [pts[0]]; rest = list(pts[1:]); s = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inT), i) for i, r in enumerate(rest)); s += d; inT.append(rest.pop(i))
    return s


def review_active(K):
    v = K.get('specs') or {}
    return bool(v.get('review_fix') or v.get('disturb_flank') or K.get('disturb_flank') or v.get('ret_height') or v.get('ipc_inner_model') == '2221')


def _via_geo(R, K):
    if review_active(K):
        from analysis.copper_model import via_geo
        return via_geo(R, K)
    v = str((R.get('options') or {}).get('via') or (K.get('design_rules') or {}).get('via') or '0.45/0.20').split('/')
    try: return float(v[0]), float(v[1])
    except (ValueError, IndexError): return 0.45, 0.20


def _return_path_standard(B, R, A, GND, via_d, drill, areas=False, K=None):
    per = collections.defaultdict(lambda: dict(l_without=0.0, l_border=0.0, n_gap=0))
    if areas:

        for t in R.get('tracks', []):
            g = A['cu_layers'].get(t['layer'])
            if t['net'] not in GND and g is not None and g['type_'] == 'open_items':
                per[t['net']]['l_without'] += sum(math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    if not A['levels']: return per, {}
    ol = [Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]
    contour = shapely.union_all(ol) if ol else None
    if areas:
        from analysis.areas import contour as board_contour
        contour = board_contour(B)
    border = contour.boundary if contour is not None else None

    hole_xy, hole_r, hole_n = [], [], []
    for v in R.get('vias', []):
        if v['net'] in GND: continue
        hole_xy.append((v['x'], v['y'])); hole_r.append(via_d / 2 + ZCLR); hole_n.append(v['net'])
    for p in B['pads']:
        if not p.get('drill') or p['net'] in GND: continue
        bb = p.get('bbox') or [p['x'], p['y'], p['x'], p['y']]
        hole_xy.append((p['x'], p['y'])); hole_r.append(max(bb[2] - bb[0], bb[3] - bb[1], p['drill']) / 2 + ZCLR); hole_n.append(p['net'] or '')
    hole_list = shapely.buffer(shapely.points(hole_xy), hole_r, quad_segs=4) if hole_xy else None
    tree_idx = shapely.STRtree(hole_list) if hole_list is not None else None

    P, PN, PT, PL, PH, PREF = [], [], [], [], [], []
    for ti, t in enumerate(R.get('tracks', [])):
        g = A['cu_layers'].get(t['layer'])
        if t['net'] in GND or g is None or g['type_'] in ('plane', 'open_items'): continue
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            k = max(1, int(math.ceil(d / SAMPLE)))
            for j in range(k):
                f = (j + 0.5) / k; P.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                PN.append(t['net']); PT.append(ti); PL.append(d / k); PH.append(g['h']); PREF.append(g['ref'])
    if not P: return per, {}
    pts = shapely.points(P); vacant = np.zeros(len(P), bool)
    filled = collections.defaultdict(list)
    for z in R.get('zones', []) if areas else []:
        if z.get('filled') and z['net'] in GND:
            filled[z['layer']].append(Polygon(z['pts'], z.get('holes', [])).buffer(0))
    if filled:

        from analysis.areas import joined
        connected = collections.defaultdict(list)
        for n in sorted(GND):
            cc = joined(B, R, K or dict(cu_layers=len(A['stack']), gnd=sorted(GND)), n, details=True)
            for L, gg in cc.items(): connected[L] += gg
        for L, gg in filled.items():
            g = shapely.union_all(gg).intersection(shapely.union_all(connected[L]))
            filled[L] = [g]
    if areas:
        vacant |= np.asarray([ref not in filled for ref in PREF], dtype=bool)
    if tree_idx is not None:
        pi, li = tree_idx.query(pts, predicate='intersects')
        for a, b in zip(pi.tolist(), li.tolist()):
            if hole_n[b] != PN[a] and PREF[a] not in filled: vacant[a] = True

    for L, gg in filled.items():
        ids = np.asarray([i for i, ref in enumerate(PREF) if ref == L], dtype=int)
        if len(ids): vacant[ids] |= ~shapely.covers(shapely.union_all(gg), pts[ids])
    at_border = np.zeros(len(P), bool)
    if border is not None:
        at_border = shapely.distance(pts, border) < 3 * np.asarray(PH)
        if contour is not None: vacant |= ~shapely.contains(contour, pts)
    run = 0.0; before_t = None; before_free = False; left = False
    for i in range(len(P)):
        n = PN[i]; e = per[n]
        if vacant[i]: e['l_without'] += PL[i]
        elif at_border[i]: e['l_border'] += PL[i]
        if PT[i] != before_t: run = 0.0; left = False; before_free = False
        if vacant[i]: run += PL[i]
        else:
            if before_free and left and run >= GAP_MIN: e['n_gap'] += 1
            run = 0.0; left = True
        before_free = bool(vacant[i]); before_t = PT[i]
    return per, {}




def _nodes(L, x, y):
    return (L, round(x / TOL) * TOL, round(y / TOL) * TOL)


def return_path(B, R, A, GND, via_d, drill, K=None, areas=False):
    if areas:
        return _return_path_standard(B, R, A, GND, via_d, drill, areas=True, K=K)
    if K is not None and not review_active(K): return _return_path_standard(B, R, A, GND, via_d, drill)
    if not A['levels']: return {}, {}
    ol = [Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]
    contour = shapely.union_all(ol) if ol else None
    border = contour.boundary if contour is not None else None
    from analysis.copper_model import baseline, settings, attached
    clr, zone_edge = settings(K)
    fill = {L: baseline(B, R, L, {A['area_nets'][L]} if 'area_nets' in A else GND,
                     clr=clr, border=zone_edge, via_d=via_d, drill=drill) for L in A['levels']}
    net_list = A.get('area_nets') or {L: next(iter(GND)) for L in A['levels'] if len(GND) == 1}
    raw_data = fill
    fill = attached(B, R, raw_data, net_list, via_d=via_d, drill=drill)

    P, PN, PT, PL, PH, PR = [], [], [], [], [], []
    for ti, t in enumerate(R.get('tracks', [])):
        g = A['cu_layers'].get(t['layer'])
        if t['net'] in GND or g is None or g['type_'] in ('plane', 'open_items'): continue
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            k = max(1, int(math.ceil(d / SAMPLE)))
            for j in range(k):
                f = (j + 0.5) / k; P.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                PN.append(t['net']); PT.append(ti); PL.append(d / k); PH.append(g['h']); PR.append(g['ref'])
    per = collections.defaultdict(lambda: dict(l_without=0.0, l_border=0.0, n_gap=0, l_unknown=0.0))
    if not P: return per, {}
    pts = shapely.points(P); vacant = np.zeros(len(P), bool); unknown = np.zeros(len(P), bool)
    for L, geom in fill.items():
        idx = np.flatnonzero(np.array([r == L for r in PR]))
        if len(idx):
            vacant[idx] = ~shapely.covers(geom, pts[idx])
            unknown[idx] = vacant[idx] & shapely.covers(raw_data[L], pts[idx])
    at_border = np.zeros(len(P), bool)
    if border is not None:
        at_border = shapely.distance(pts, border) < 3 * np.asarray(PH)
        if contour is not None: vacant |= ~shapely.contains(contour, pts)
    run = 0.0; before_t = None; before_free = False; left = False
    for i in range(len(P)):
        n = PN[i]; e = per[n]
        if unknown[i]: e['l_unknown'] += PL[i]
        if vacant[i]: e['l_without'] += PL[i]
        elif at_border[i]: e['l_border'] += PL[i]
        if PT[i] != before_t: run = 0.0; left = False; before_free = False
        if vacant[i]: run += PL[i]
        else:
            if before_free and left and run >= GAP_MIN: e['n_gap'] += 1
            run = 0.0; left = True
        before_free = bool(vacant[i]); before_t = PT[i]
    return per, {}




def _nodes(L, x, y):
    return (L, round(x / TOL) * TOL, round(y / TOL) * TOL)


def current_tree(tracks, vias, pads, flow, A, drill, I, graph=None):
    from common.flow import net_graph
    if not flow or not tracks: return None
    source_k = set(flow.get('source') or []); load = flow.get('load') or {}
    R_via = RHO * A['thickness'] / (math.pi * (drill + VIA_SLEEVE) * VIA_SLEEVE)
    g = graph if graph is not None else net_graph(tracks, vias, pads, TOL); adj = g['adj']; pad_pts = g['pad_pts']
    seg = []; s_idx = {}; s_r = {}
    for ti, t in enumerate(tracks):
        L = t['layer']; tl = A['cu_layers'].get(L); thickness = tl['t'] if tl else T_OUTER
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            l = math.dist(a, b)
            if l < 1e-9: continue
            s_idx[(ti, si)] = len(seg); s_r[(ti, si)] = RHO * l / (w * thickness)
            seg.append([ti, si, L, a, b, w, l, 0.0, thickness])
    def resistance(kid):
        if kid is None: return 0.0
        if kid[0] == 'v': return R_via
        return s_r[(kid[1], kid[2])] * (kid[4] - kid[3])
    starts = [pad_pts[q] for q in sorted(source_k) if q in pad_pts and pad_pts[q] in adj]
    if not starts: return None
    before = {s: None for s in starts}; ordering = list(starts); i = 0
    while i < len(ordering):
        k = ordering[i]; i += 1
        for m, kid in adj[k]:
            if m in before: continue
            before[m] = (k, resistance(kid), kid); ordering.append(m)
    bottom = collections.defaultdict(float)
    for key, pk in pad_pts.items():
        if pk in before and key not in source_k: bottom[pk] += float(load.get(key, 0.0))
    for k in reversed(ordering):
        if before[k] is not None: bottom[before[k][0]] += bottom[k]
    total = sum(bottom[s] for s in starts) or 1.0
    U = {}; P = 0.0; via_I = [0.0] * len(vias)
    for k in ordering:
        if before[k] is None: U[k] = 0.0; continue
        p_, r, kid = before[k]; i_k = bottom[k] / total * I
        U[k] = U[p_] + i_k * r; P += i_k * i_k * r
        if kid and kid[0] == 's':
            j = s_idx[(kid[1], kid[2])]; seg[j][7] = max(seg[j][7], i_k)
        elif kid and kid[0] == 'v': via_I[kid[1]] = max(via_I[kid[1]], i_k)
    reached = sum(1 for pk in pad_pts.values() if pk in before)
    return dict(P=P, dV=max(U.values()) if U else 0.0, seg=[tuple(s_) for s_ in seg], via_I=via_I, reached=reached, pads=len(pad_pts))


def current_density(seg, A, n, pads=None, ipc_model=None):
    x_max = 0.0; st = []
    pp = [(p['bbox'], set(p['layers'])) for p in (pads or [])]
    def at_pad(L, a, b, l):
        if l > J_NECK_L: return False
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        for (x0, y0, x1, y1), lg in pp:
            if L in lg and x0 - J_NECK_D <= mx <= x1 + J_NECK_D and y0 - J_NECK_D <= my <= y1 + J_NECK_D: return True
        return False
    for ti, si, L, a, b, w, l, i_s, thickness in seg:
        if i_s <= 0 or at_pad(L, a, b, l): continue
        inner = L not in ('F', 'B')
        ws = width_ipc(i_s, inner, t=thickness, model=ipc_model)
        x = ws / w
        x_eff = x if x <= 1 else 1 + (x - 1) * min(1.0, l / J_L_WHOLE)
        if x_eff > x_max: x_max = x_eff
        if x > 1: st.append((round(x_eff, 3), n, L, round((a[0] + b[0]) / 2, 2), round((a[1] + b[1]) / 2, 2), round(w, 3), round(ws, 3),
                             round(i_s, 3), round(i_s / (w * thickness), 1), round(l, 2)))
    return x_max, st


def _current(n, K, how_solver=False):
    k = K['net_list'].get(n) or {}; fl = k.get('flow') or {}
    if fl.get('unit') == 'A':
        q = fl.get('unit_source') or 'A'
        if fl.get('learning_limit'): q += ' (learning-limited)'
        return float(fl.get('total') or 0.0), q
    if how_solver: return 1.0, '1A'
    if k.get('current_A') is not None: return float(k['current_A']), 'estimate'
    return 1.0, '1A'


def direct_current_estimate(B, R, K, net_list, how_solver=False, flow_cache=None):
    n_layers = int(K.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or B.get('cu_layers') or 2)
    A = structure(n_layers, [])
    via_d, drill = _via_geo(R, K)
    R_via = RHO * A['thickness'] / (math.pi * (drill + VIA_SLEEVE) * VIA_SLEEVE)
    want = set(net_list); tr = collections.defaultdict(list); vi = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R.get('tracks', []):
        if t['net'] in want and not t.get('sig') and t['layer'] in A['cu_layers']: tr[t['net']].append(t)
    for v in R.get('vias', []):
        if v['net'] in want: vi[v['net']].append(v)
    for p in B['pads']:
        if p['net'] in want: pads[p['net']].append(p)
    out = {}
    for n in sorted(want):
        if n not in K['net_list'] or not tr[n]: continue
        I, _q = _current(n, K, how_solver)
        b = current_tree(tr[n], vi[n], pads[n], K['net_list'][n].get('flow'), A, drill, I, graph=flow_cache.graph(tr[n], vi[n], pads[n], TOL) if flow_cache is not None else None) if I > 0 else None
        if b: out[n] = (1000 * b['dV'], 1000 * b['P'])
        else:
            r = sum(RHO * math.dist(a, c) / (w * A['cu_layers'][t['layer']]['t']) for t in tr[n] for a, c, w in zip(t['pts'][:-1], t['pts'][1:], t['w']))
            r += len(vi[n]) * R_via
            out[n] = (1000 * I * r, 1000 * I * I * r)
    return out


def voltage_with_source(n, K):
    CONS = K.get('specs') or {}; b = n.split('/')[-1]
    for rx, v in sorted((CONS.get('voltage') or {}).items()):
        try:
            if b == rx or n == rx or re.fullmatch(rx, b):
                if isinstance(v, (int, float)) and math.isfinite(v): return float(v), 'spec'
        except re.error: pass
    k = K['net_list'].get(n) or {}; v = k.get('voltage_V')
    if isinstance(v, (int, float)) and math.isfinite(v): return float(v), k.get('voltage_source') or 'net_class_without_provenance'
    m = re.search(r'(\d+)V(\d*)', b.upper())
    if m: return float(m.group(1) + ('.' + m.group(2) if m.group(2) else '')), 'name_estimate'
    return None, None


def voltage(n, K):
    return voltage_with_source(n, K)[0]


def budgets(K):
    CONS = K.get('specs') or {}; per, gr = {}, {}
    for n in K['net_list']:
        for rx, v in (CONS.get('budget_mV') or {}).items():
            try:
                if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): per[n] = float(v); break
            except re.error: pass
    for name, g in sorted((CONS.get('channel_budget_mV') or {}).items()):
        nn = [x for x in g.get('net_list', []) if x in K['net_list']]
        if nn: gr[name] = (nn, float(g['mV']))
    return per, gr




def _pad_poly(p, L):
    qs = [Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    if qs: return shapely.union_all(qs)
    return shapely.box(*p['bbox'])


def _net_graph(tracks, vias, pads_net, A, drill):
    from common.flow import _seg_dist
    ends = collections.defaultdict(list)
    for t in tracks:
        for q in t['pts']: ends[t['layer']].append(tuple(q))
    for v in vias:
        for L in sorted({t['layer'] for t in tracks}): ends[L].append((v['x'], v['y']))
    adj = collections.defaultdict(list); pos = {}
    def edge(a, b, Lnh, l, nv):
        adj[a].append((b, Lnh, l, nv)); adj[b].append((a, Lnh, l, nv))
    for t in tracks:
        L = t['layer']
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            part = [(0.0, tuple(a)), (1.0, tuple(b))]
            for q in ends[L]:
                d, s = _seg_dist(q, a, b)
                if d < TOL and 1e-6 < s < 1 - 1e-6: part.append((s, q))
            part.sort(); lp = l_plane(A, L, w)
            for (s1, p1), (s2, p2) in zip(part[:-1], part[1:]):
                l = math.dist(p1, p2)
                if l < 1e-9: continue
                k1, k2 = _nodes(L, *p1), _nodes(L, *p2); pos[k1] = p1; pos[k2] = p2
                edge(k1, k2, lp * l, l, 0)
    cu_layers = sorted({t['layer'] for t in tracks} | {L for p in pads_net for L in p['layers'] if L in A['cu_layers']})
    for v in vias:
        ks = [_nodes(L, v['x'], v['y']) for L in cu_layers if L in A['cu_layers']]
        for k in ks: pos[k] = (v['x'], v['y'])
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                h = abs(A['cu_layers'][ks[i][0]]['z'] - A['cu_layers'][ks[j][0]]['z'])
                edge(ks[i], ks[j], via_lc(h, drill, A, 0.45)[0], h, 1)
    nodes = [k for k in pos if k[0] != 'PAD']
    kp = shapely.points([pos[k] for k in nodes]) if nodes else None
    geo = {}
    for p in pads_net:
        pk = ('PAD', p['ref'], p['num']); pos[pk] = (p['x'], p['y'])
        for L in p['layers']:
            if L not in A['cu_layers']: continue
            g = geo[(pk, L)] = _pad_poly(p, L)
            if kp is None: continue
            d = shapely.distance(g, kp)
            for i in np.nonzero(d <= TOL)[0].tolist():
                if nodes[i][0] == L: edge(pk, nodes[i], 0.0, 0.0, 0)

    pl = [p for p in pads_net]
    for i, p in enumerate(pl):
        for q in pl[i + 1:]:
            for L in sorted(set(p['layers']) & set(q['layers'])):
                if L not in A['cu_layers']: continue
                a, b = geo[(('PAD', p['ref'], p['num']), L)], geo[(('PAD', q['ref'], q['num']), L)]
                if a.distance(b) <= 1e-6:
                    l = math.dist((p['x'], p['y']), (q['x'], q['y']))
                    w = max(0.2, min(a.bounds[2] - a.bounds[0], a.bounds[3] - a.bounds[1]))
                    edge(('PAD', p['ref'], p['num']), ('PAD', q['ref'], q['num']), l_plane(A, L, w) * l, l, 0); break
    return adj, pos


def _shortest(adj, s, z):
    if s not in adj or z not in adj: return None
    best = {s: 0.0}; before = {}; hq = [(0.0, 0, s)]; n = 0
    while hq:
        d, _, k = heapq.heappop(hq)
        if k == z: break
        if d > best.get(k, 1e18): continue
        for m, Lnh, l, nv in adj[k]:
            nd = d + Lnh + 1e-6 * l
            if nd < best.get(m, 1e18): best[m] = nd; before[m] = (k, l, nv); n += 1; heapq.heappush(hq, (nd, n, m))
    if z not in best: return None
    way = [z]; l = 0.0; nv = 0
    while way[-1] != s:
        k, li, vi = before[way[-1]]; l += li; nv += vi; way.append(k)
    return best[z], l, nv, way[::-1]


def _ground_connection(p, A, gnd_vias, gnd_tracks, drill):
    if not A['levels']: return None, 'no ground layer'
    L_pad = next((L for L in p['layers'] if L in A['cu_layers']), None)
    if L_pad is None: return None, 'Layer'
    z0 = A['cu_layers'][L_pad]['z']
    h = min(abs(A['cu_layers'][E]['z'] - z0) for E in A['levels'])
    if h < 1e-9: return 0.0, 'Pad on the ground layer'
    lv = via_lc(h, drill, A, 0.45)[0]
    g = _pad_poly(p, L_pad)
    if p.get('drill'): return lv / 2, 'THT'
    if not len(gnd_vias): return L_FREE * 5, 'no ground via'
    d = np.asarray(shapely.distance(g, shapely.points(gnd_vias)), dtype=float)
    in_ = int(np.sum(d <= 0.05))
    if in_: return lv / in_, '%d via in pad' % in_
    j = int(np.argmin(d)); dist = float(d[j])
    w = 0.25
    for t in gnd_tracks:
        if t['layer'] == L_pad and any(g.distance(Point(q)) <= TOL for q in (t['pts'][0], t['pts'][-1])): w = max(w, max(t['w']))
    return l_plane(A, L_pad, w) * dist + lv, 'Stub %.2f mm' % dist


def hot_loops(B, R, K, A, GND, drill):
    T = K.get('part_list') or {}; N = K['net_list']
    pads = collections.defaultdict(list); pad = {}
    for p in B['pads']:
        pad[(p['ref'], p['num'])] = p
        if p['net']: pads[p['net']].append(p)
    regulator = {}
    for r, t in sorted(T.items()):
        st = t.get('supports') or {}
        if t.get('function') == 'coil_sw' and st.get('ref'): regulator.setdefault(st['ref'], st.get('net_name'))
    gnd_vias = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] or np.zeros((0, 2)))
    gnd_tracks = [t for t in R.get('tracks', []) if t['net'] in GND]
    out = []
    for chip, sw in sorted(regulator.items()):
        cs = []
        for r, t in sorted(T.items()):
            st = t.get('supports') or {}
            if not str(t.get('function', '')).startswith('decoupling') or st.get('ref') != chip or st.get('over_coil'): continue
            if st.get('net_name') == sw: continue
            cs.append((t.get('C') or 1.0, r, st))
        if not cs: continue
        C, cr, st = min(cs, key=lambda c: (c[0], c[1]))
        vin_net = st.get('net_name'); vin_pin = pad.get((chip, str(st.get('pin'))))
        cp = [p for p in B['pads'] if p['ref'] == cr]
        c_plus = next((p for p in cp if p['net'] == vin_net), None); c_minus = next((p for p in cp if p['net'] in GND), None)
        pg = [p for p in B['pads'] if p['ref'] == chip and p['net'] in GND]
        if not (vin_pin and c_plus and c_minus and pg): continue
        pgnd = min(pg, key=lambda p: (math.dist((p['x'], p['y']), (vin_pin['x'], vin_pin['y'])), p['num']))
        tr = [t for t in R.get('tracks', []) if t['net'] == vin_net and not t.get('sig')]
        vi = [v for v in R.get('vias', []) if v['net'] == vin_net]
        adj, pos = _net_graph(tr, vi, pads[vin_net], A, drill)
        w = _shortest(adj, ('PAD', c_plus['ref'], c_plus['num']), ('PAD', vin_pin['ref'], vin_pin['num']))
        L_c = next((L for L in c_plus['layers'] if L in A['cu_layers']), 'F')
        if w is not None:
            L_vin, l_vin, n_v, way = w; points = [pos[k] for k in way if k in pos]; source = 'Track'
        else:
            gem = set(c_plus['layers']) & set(vin_pin['layers'])
            if any(_pad_poly(c_plus, L).distance(_pad_poly(vin_pin, L)) <= 1e-6 for L in gem):
                L_vin, l_vin, n_v = 0.0, 0.0, 0; source = 'Pads touch each other'
            else:
                l_vin = math.dist((c_plus['x'], c_plus['y']), (vin_pin['x'], vin_pin['y'])); n_v = 0
                L_vin = l_plane(A, L_c, max(0.3, (N.get(vin_net) or {}).get('w') or 0.3)) * l_vin; source = 'Straight line (not connected)'
            points = [(c_plus['x'], c_plus['y']), (vin_pin['x'], vin_pin['y'])]
        L_pg, q_pg = _ground_connection(pgnd, A, gnd_vias, gnd_tracks, drill)
        L_cm, q_cm = _ground_connection(c_minus, A, gnd_vias, gnd_tracks, drill)
        poly = [(c_plus['x'], c_plus['y'])] + points + [(vin_pin['x'], vin_pin['y']), (pgnd['x'], pgnd['y']), (c_minus['x'], c_minus['y'])]
        fl = 0.5 * abs(sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1] for i in range(len(poly))))
        L_hl = L_vin + (L_pg or 0.0) + (L_cm or 0.0)
        out.append(dict(chip=chip, sw=sw, vin=vin_net, c_in=cr, C_F=C, vin_pin=vin_pin['num'], pgnd_pin=pgnd['num'],
                        L_nH=round(L_hl, 4), L_vin_nH=round(L_vin, 4), l_vin_mm=round(l_vin, 3), vias_vin=n_v, vin_way=source,
                        L_pgnd_nH=None if L_pg is None else round(L_pg, 4), pgnd=q_pg, L_cground_nH=None if L_cm is None else round(L_cm, 4),
                        cground=q_cm, A_mm2=round(fl, 3)))
    return out




def ground_coupling(B, R, K, A, GND):
    nts = list(K.get('net_ties') or [])
    if not nts or not A['levels']: return None
    T = K.get('part_list') or {}; N = K['net_list']
    t_ges = sum(A['cu_layers'][E]['t'] for E in A['levels'])
    Rq = RHO / t_ges
    pads = collections.defaultdict(list)
    for p in B['pads']: pads[p['ref']].append(p)
    gv = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] or np.zeros((0, 2)))
    GV = shapely.points(gv) if len(gv) else None
    gt = [t for t in R.get('tracks', []) if t['net'] in GND]
    def entry(p):
        if p.get('drill') or GV is None: return [(p['x'], p['y'])]
        g = _pad_poly(p, p['layers'][0])
        d = np.asarray(shapely.distance(g, GV), dtype=float)
        close = [tuple(gv[i].tolist()) for i in np.argsort(d, kind='stable')[:4] if d[i] <= 0.6]
        if close: return close
        st = []
        for t in gt:
            if t['layer'] not in p['layers']: continue
            a, b = t['pts'][0], t['pts'][-1]
            for e, f in ((a, b), (b, a)):
                if g.distance(Point(e)) <= TOL:
                    j = int(np.argmin(np.hypot(gv[:, 0] - f[0], gv[:, 1] - f[1])))
                    if math.dist(gv[j], f) <= 0.05: st.append(tuple(gv[j].tolist()))
        if st: return sorted(set(st))
        j = int(np.argmin(d))
        return [tuple(gv[j].tolist())] if d[j] <= 3.0 else [(p['x'], p['y'])]

    victim = {}
    for nt in sorted(nts):
        P = pads.get(nt) or []
        g = [p for p in P if p['net'] in GND]; s = [p for p in P if p['net'] and p['net'] not in GND]
        if not g or not s: continue
        mm = s[0]['net']
        I = 0.0
        for r, t in sorted(T.items()):
            if r == nt or not t.get('current_A'): continue
            if any(p['net'] == mm for p in pads.get(r, [])) and any((N.get(p['net']) or {}).get('category') in ('current_val', 'supply') for p in pads.get(r, [])):
                I += float(t['current_A'])
        victim[nt] = dict(net_name=mm, point=entry(g[0]), I=I)
    if not victim: return None

    count = collections.Counter()
    for p in B['pads']:
        if p['net'] and (N.get(p['net']) or {}).get('category') == 'analog' and (T.get(p['ref']) or {}).get('role') == 'chip': count[p['ref']] += 1
    ref_chip = min(count, key=lambda r: (-count[r], r)) if count else None
    ref_pts = [q for p in pads.get(ref_chip, []) if p['net'] in GND for q in entry(p)][:8] if ref_chip else []
    if not ref_pts: return None

    sink_one = []
    for n, k in sorted(N.items()):
        fl = k.get('flow') or {}
        if fl.get('from_') or k.get('category') not in ('current_val', 'supply'): continue
        for q in fl.get('source') or []:
            r = q.split(':')[0]
            if (T.get(r) or {}).get('role') == 'connector':
                sink_one += [pt for p in pads.get(r, []) if p['net'] in GND for pt in entry(p)]
    sink_one = sorted(set(sink_one))
    regulator = sorted({(t.get('supports') or {}).get('ref') for t in T.values() if t.get('function') == 'coil_sw'} - {None})
    CONS = K.get('specs') or {}
    def sinks(D):
        s = []
        rp = [pt for r in regulator for p in pads.get(r, []) if p['net'] in GND for pt in entry(p)]
        if sink_one: s += [(pt, (D if rp else 1.0) / len(sink_one)) for pt in sink_one]
        if rp: s += [(pt, (1 - D if sink_one else 1.0) / len(rp)) for pt in rp]
        return s
    D = float(np.mean([float(CONS['implementation'][r]) for r in regulator if r in (CONS.get('implementation') or {})])) if regulator and CONS.get('implementation') else 0.5

    attack = {}
    for nt, o in victim.items():
        if o['I'] >= I_MIN_LOAD: attack[nt] = [(pt, o['I'] / len(o['point'])) for pt in o['point']]
    for r, t in sorted(T.items()):
        if t.get('role') != 'chip' or r in regulator or (t.get('current_A') or 0) < I_MIN_LOAD: continue
        if 'throughflow' in str(t.get('current_A_source') or ''): continue
        gp = [pt for p in pads.get(r, []) if p['net'] in GND for pt in entry(p)]
        if gp: attack[r] = [(pt, float(t['current_A']) / len(gp)) for pt in gp]
    if not attack: return None
    S = sinks(D)
    if not S: return None
    k = Rq / (2 * math.pi) * 1e6
    def pot(pt, source_list):
        u = 0.0
        for q, I in source_list:
            u -= I * math.log(max(math.dist(pt, q), 0.2))
        return u
    tab = {}; mx = 0.0; per = collections.defaultdict(float)
    for a, Q in sorted(attack.items()):
        I = sum(i for _, i in Q)
        source_list = Q + [(pt, -I * f) for pt, f in S]
        u_ref = float(np.mean([pot(p, source_list) for p in ref_pts]))
        for v, o in sorted(victim.items()):
            if v == a: continue
            e = k * (float(np.mean([pot(p, source_list) for p in o['point']])) - u_ref)
            tab['%s<-%s' % (v, a)] = round(e, 3); per[o['net_name']] = max(per[o['net_name']], abs(e)); mx = max(mx, abs(e))
    return dict(max_uV=round(mx, 3), per_net={n: round(v, 3) for n, v in sorted(per.items())}, table=tab, R_square_mOhm=round(1000 * Rq, 4),
                baseline=ref_chip, sinks=len(S), D=round(D, 3), attacker=sorted(attack))




def measure(B, R, K, ir=None, weights=None, gnd_layers=None, AU=None, flow_cache=None, minimax=False):
    from common.zones import active, unify, board_geometry
    B = board_geometry(B, R)
    r36 = active(R)
    R = unify(R)
    copper_model = bool(R.get('copper_bars')) or r36
    if copper_model:
        from analysis.copper_model import replacement_routes, ir_values
        if ir is None:
            ir = ir_values(B, R, K)
            if r36 and ir['error_nets']:
                ir = ir_values(B, R, K, res=.05)
        if not r36:
            R = replacement_routes(B, R, K)

        from common.flow import NetGraphCache
        flow_cache = NetGraphCache(endpoints=True)
    review = review_active(K)
    G = weights or load_weights(si=(K.get('specs') or {}).get('si_evaluation') is True)
    si_active = G.get('version') == 'v3.2' and bool(G.get('si'))
    N = K['net_list']; GND = set(K.get('gnd') or [])
    from analysis.areas import active, assignment, copper_map, metric
    la_active = active(R, K)
    LAYER_STACK = None; FLMAP = {}; FR = R
    if la_active:
        FLMAP = assignment(B, R, K, gnd_layers=gnd_layers)
        supply_areas = {n for nn in FLMAP.values() for n in nn} - GND
        supply_areas |= set(R.get('area_nets') or []) - GND
        supply_areas |= {z['net'] for z in R.get('zones', []) if z.get('filled') and z['net'] not in GND and
                              (z['net'] not in N or N[z['net']].get('category') in ('current_val', 'supply'))}
        n_layers = int(K.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or (K.get('design_rules') or {}).get('cu_layers') or B.get('cu_layers') or 2)
        LAYER_STACK = metric(B, R, K, gnd_layers=gnd_layers)


        levels = LAYER_STACK['gnd_copper_layers']


        if not GND: levels = [L for L, nn in FLMAP.items() if not nn]


        unknown = supply_areas - set(N)
        if unknown: raise ValueError('Unknown area nets: %s' % sorted(unknown))
        FR = copper_map(B, R, K, gnd_layers=gnd_layers) if FLMAP else R
    else:
        supply_areas = set(R.get('area_nets') or []) - GND
        n_layers = int(K.get('cu_layers') or (R.get('options') or {}).get('cu_layers') or (K.get('design_rules') or {}).get('cu_layers') or B.get('cu_layers') or 2)
    planes = {}
    if review and not la_active:
        from analysis.copper_model import ground
        planes, _ = ground(B, R, K, gnd_layers=gnd_layers)
        levels = list(planes)
    if not la_active and not review:
        raw = (R.get('options') or {}).get('gnd_layers') or ''
        label = [x for x in raw.split(',') if x] if isinstance(raw, str) else list(raw)
        label += list(R.get('gnd_layers') or [])
        layer_net = dict(str(x).split('=', 1) for x in label if '=' in str(x))
        gl = gnd_layers if gnd_layers is not None else (R.get('gnd_layers') or [])
        if isinstance(gl, str): gl = [x for x in gl.split(',') if x]
        levels = []
        for x in gl:
            part_list = str(x).split('=', 1); layer_name = part_list[0]
            n = part_list[1] if len(part_list) == 2 else layer_net.get(layer_name)

            if n is None or all(x in GND or '/' + x in GND for x in n.split('|')): levels.append(layer_name)
    CONS = K.get('specs') or {}
    A = structure(n_layers, levels, diel=(CONS.get('structure') or {}).get('dielectrics'), er=float((CONS.get('structure') or {}).get('er', ER)))
    if la_active and CONS.get('structure'):
        from analysis.structure import normalize
        explicit = normalize(CONS['structure'], n_layers)
        A = structure(n_layers, levels, diel=explicit['dielectrics'], er=float((CONS['structure']).get('er', ER)))
        for L, t in zip(A['stack'], explicit['copper_mm']): A['cu_layers'][L]['t'] = t
    A['area_nets'] = planes
    if review and not la_active:
        from analysis.current_width import copper_thickness
        for Lg, g in A['cu_layers'].items(): g['t'] = copper_thickness(CONS, inner=Lg not in ('F', 'B'))
    via_d, drill = _via_geo(R, K)
    if AU is None:
        from common.length import outer
        AU = outer(R, B)
    tr = collections.defaultdict(list); vi = collections.defaultdict(list); pads = collections.defaultdict(list)
    for ti, t in enumerate(R.get('tracks', [])): tr[t['net']].append(ti)
    for v in R.get('vias', []): vi[v['net']].append(v)
    for p in B['pads']:
        if p['net']: pads[p['net']].append(p)
    gnd_xy = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] +
                      [(p['x'], p['y']) for p in B['pads'] if p.get('drill') and p['net'] in GND] or np.zeros((0, 2)))
    RW, _ = return_path(B, FR, A, GND, via_d, drill, areas=la_active, K=K)
    from analysis.coupling import johnson
    tr_cons = (K.get('specs') or {}).get('tr_ns') or {}
    def tr_from(n):
        if si_active:
            from analysis.signal_integrity import spec, count
            standard = (G['netclasses'].get(net_class(N.get(n)), {}) or {}).get('tr_ns')
            raw = spec(K, n, 'tr_ns', standard)
            return None if raw is None else count(raw, standard, 1e-6)
        b = n.split('/')[-1]
        for rx, v in sorted(tr_cons.items()):
            try:
                if b == rx or n == rx or re.fullmatch(rx, b): return float(v) if v is not None else None
            except re.error: pass
        if (N.get(n) or {}).get('tr_ns') is not None: return float(N[n]['tr_ns'])
        return (G['netclasses'].get(net_class(N.get(n)), {}) or {}).get('tr_ns')
    from analysis.aggressor import edges
    tr_ns = {n: tr_from(n) for n in set(N) | set(tr)} if si_active or not review else edges(K, G if weights is not None or 'aggressor' not in K else None)
    XT = johnson(R, K, A, tr_ns, GND)
    sw_nets = {n for n, k in N.items() if k.get('category') == 'sw'}
    IRd = dict((ir or {}).get('max_drop_mV') or {}); IRp = dict((ir or {}).get('loss_per_net_mW') or {})
    IRi = dict((ir or {}).get('current_per_net_A') or {})
    plane_ir = {}
    if la_active and supply_areas:
        from codex.zones_ir.solver_fast import ir_metrics
        KP = dict(K, net_list={n: N[n] for n in sorted(supply_areas)})


        stack = {L: dict(copper_um=g['t'] * 1000, z_mm=g['z']) for L, g in A['cu_layers'].items()}
        for L, v in (B.get('stackup') or {}).items(): stack.setdefault(L, {}).update(v)
        BI = dict(B, stackup=stack)
        plane_ir = ir_metrics(BI, FR, KP)['net_list']
        missing = supply_areas - set(plane_ir)
        if missing: raise ValueError('IR area model missing coverage: %s' % sorted(missing))
        bad = {n: e.get('error') for n, e in plane_ir.items() if e.get('status') not in ('solved', 'no_load')}
        if bad: raise ValueError('IR area model incomplete: %s' % bad)
        for n, e in plane_ir.items():
            IRd[n] = 1000 * float(e.get('max_drop_V') or 0)
            IRp[n] = 1000 * float(e.get('power_W') or 0)
            IRi[n] = float(e.get('total_current_A') or _current(n, K)[0])
    BU, BG = budgets(K)
    out = {}; sites = []
    for n in sorted(set(tr) | set(vi) | set(pads)):
        if n in GND or n not in N: continue
        nc = net_class(N[n]); C = G['netclasses'].get(nc)
        if C is None: continue
        l = 0.0; L_nH = 0.0; C_pF = 0.0; T_ps = 0.0; R_ohm = 0.0; A_mm2 = 0.0; per_layer = collections.defaultdict(float)
        L_rp_height = 0.0
        ends = collections.defaultdict(list)
        for ti in tr[n]:
            t = R['tracks'][ti]; Lg = t['layer']
            if Lg not in A['cu_layers']: continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                d = AU[ti][si]
                Z, tpd, Lp, Cp = line(A, Lg, w)
                if CONS.get('ret_height'):
                    good = [line(A, Lb, w)[2] for Lb, gb in A['cu_layers'].items() if gb['type_'] not in ('plane', 'open_items')]

                    L_rp_height += d * max(0.0, Lp - min(good, default=0.0))
                l += d; L_nH += Lp * d; C_pF += Cp * d; T_ps += tpd * d; per_layer[Lg] += d
                if not t.get('sig'): R_ohm += RHO * d / (w * A['cu_layers'][Lg]['t'])
                A_mm2 += w * math.dist(a, b)
            ends[Lg] += [t['pts'][0], t['pts'][-1]]

        n_via = len(vi[n]); rv_nH = 0.0; change = []; via_L = 0.0; via_C = 0.0
        s_ok = (C.get('s') or {}).get('ret_via_mm', 1.27)
        for v in vi[n]:
            gen = {Lg for Lg, E in ends.items() if any(abs(e[0] - v['x']) < via_d / 2 + 0.05 and abs(e[1] - v['y']) < via_d / 2 + 0.05 for e in E)}
            for p in pads[n]:
                bb = p.get('bbox')
                if bb and bb[0] - 0.01 <= v['x'] <= bb[2] + 0.01 and bb[1] - 0.01 <= v['y'] <= bb[3] + 0.01: gen |= set(p['layers'])
            gen |= {L for L, nn in FLMAP.items() if n in nn and (not v.get('layers') or L in v['layers'])}
            gen = {g for g in gen if g in A['cu_layers']}
            zs = [A['cu_layers'][g]['z'] for g in gen]
            h = (max(zs) - min(zs)) if len(zs) >= 2 else A['thickness']
            lv, cv = via_lc(h, drill, A, via_d); via_L += lv; via_C += cv
            refs = sorted({A['cu_layers'][g]['ref'] for g in gen if A['cu_layers'][g]['ref']})
            if len(refs) >= 2:
                h_ref = abs(A['cu_layers'][refs[0]]['z'] - A['cu_layers'][refs[-1]]['z'])
                s = float(np.min(np.hypot(gnd_xy[:, 0] - v['x'], gnd_xy[:, 1] - v['y']))) if len(gnd_xy) else S_MAX
                s = min(s, S_MAX)
                extra = 0.4 * h_ref * math.log(s / s_ok) if s > s_ok else 0.0
                rv_nH += extra; change.append(dict(x=v['x'], y=v['y'], s_mm=round(s, 2), extra_nH=round(extra, 3)))
        if n_via:
            R_ohm += n_via * RHO * A['thickness'] / (math.pi * (drill + VIA_SLEEVE) * VIA_SLEEVE)
        L_nH += via_L; C_pF += via_C
        if copper_model and n in sw_nets:
            A_mm2 = R.get('power_areas_mm2', {}).get(n, A_mm2)
        rw = RW.get(n, dict(l_without=0.0, l_border=0.0, n_gap=0, l_unknown=0.0))
        L_ideal = (L_nH - via_L) / l if l > 0 else 0.0
        L_rp = max(0.0, L_FREE - L_ideal) * (rw['l_without'] + rw['l_border']) + rv_nH + GAP_NH * rw['n_gap']
        L_rp += L_rp_height
        tpd_m = T_ps / l if l > 0 else 6.0
        tr_ = tr_ns.get(n)
        l_crit = (1000 * tr_ / (6 * tpd_m)) if tr_ else (C.get('s') or {}).get('length_mm')
        clearance = _mst([(p['x'], p['y']) for p in pads[n]])
        fl = N[n].get('flow') or {}
        carries = bool(fl) or n in sw_nets or N[n].get('category') in ('current_val', 'supply')

        if not carries: I, I_q = 0.0, None
        else: I, I_q = _current(n, K)

        tree_idx = None; P_mW = None; j_x = None
        if carries and I > 0 and tr[n]:
            tl = [R['tracks'][ti] for ti in tr[n] if not R['tracks'][ti].get('sig') and R['tracks'][ti]['layer'] in A['cu_layers']]
            tree_idx = current_tree(tl, vi[n], pads[n], fl, A, drill, I, graph=flow_cache.graph(tl, vi[n], pads[n], TOL) if flow_cache is not None else None) if tl else None
            if tree_idx:
                if copper_model:

                    tree_idx['seg'] = [tuple(list(s[:5]) + [tl[s[0]].get('copper_j_min', tl[s[0]]['w'])[s[1]]] + list(s[6:]))
                                   for s in tree_idx['seg']]
                j_x, st = current_density(tree_idx['seg'], A, n, pads[n], ipc_model=CONS.get('ipc_inner_model')); sites += st; P_mW = 1000 * tree_idx['P']
        if copper_model and carries and I > 0:
            from analysis.copper_model import current_density_areas
            j_x, st = current_density_areas(tl if tr[n] else [], tree_idx, A, n, pads[n], current_A=I, ipc_model=CONS.get('ipc_inner_model')); sites += st
        if si_active and carries and I > 0 and j_x is None:

            seg_free = [[ti, si, t['layer'], a, b, w, math.dist(a, b), I, A['cu_layers'][t['layer']]['t']]
                        for ti in tr[n] for t in [R['tracks'][ti]] if not t.get('sig') and t['layer'] in A['cu_layers']
                        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))]
            j_x, st = current_density(seg_free, A, n, pads[n], ipc_model=CONS.get('ipc_inner_model')); sites += st

        f_ir = (I / IRi[n]) if (n in IRi and IRi[n] > 0 and carries) else 1.0
        if n in IRd: dV, dV_q = IRd[n] * f_ir, ('ir_area' if n in plane_ir else 'ir') if abs(f_ir - 1) < 1e-9 else 'ir*%.3g' % f_ir
        elif tree_idx: dV, dV_q = 1000 * tree_idx['dV'], 'tree_idx(%s)' % I_q
        elif carries: dV, dV_q = 1000 * I * R_ohm, 'R*I(%s)' % I_q
        else: dV, dV_q = 0.0, 'no current'
        if n in IRp: P_mW = IRp[n] * f_ir * f_ir
        elif P_mW is None and carries: P_mW = 1000 * I * I * R_ohm
        if copper_model and n in IRp and I > 0:
            R_ohm = P_mW / (1000 * I * I)
        if r36 and carries:
            j_x = (ir.get('J_x') or {}).get(n)

        U, U_q = voltage_with_source(n, K)
        if n in BU: s_dc, s_q = BU[n], 'budget'
        elif U and carries: s_dc, s_q = max(10.0, 10.0 * abs(U)), '1%%_from_%gV' % U
        else: s_dc, s_q = (C.get('s') or {}).get('dc_mV'), 'net_class'
        group = next((g for g, (nn, _mv) in BG.items() if n in nn), None)
        m = dict(length_mm=round(l, 2), air_line_mm=round(clearance, 2), vias=n_via, L_nH=round(L_nH, 3), C_pF=round(C_pF, 3),
                 run_duration_ps=round(T_ps, 1), l_crit_mm=round(l_crit, 1) if l_crit else None, R_mOhm=round(1000 * R_ohm, 2),
                 dV_mV=round(dV, 6 if si_active else 2), dV_source=dV_q, I_A=round(I, 4) if carries else None,
                 voltage_V=U, voltage_source=U_q,
                 I_source=(N[n].get('current_A_source') if I_q == 'estimate' else I_q),
                 **({'J_max_A_mm2':(ir.get('J_max_A_mm2') or {}).get(n)} if r36 and ir else {}),
                 P_mW=round(P_mW, 6 if si_active else 3) if P_mW is not None else None, J_x=round(j_x, 3) if j_x is not None else None,
                 s_dc_mV=round(s_dc, 2) if s_dc else None, s_dc_source=s_q if carries else None, group=group,
                 **(dict(l_baseline_unknown_mm=round(rw.get('l_unknown', 0), 2), baseline_status='unknown' if rw.get('l_unknown', 0) > 0 else ('attached' if A['levels'] else 'open_items')) if review else {}),
                 l_without_mm=round(rw['l_without'], 2), l_border_mm=round(rw['l_border'], 2), n_gap=rw['n_gap'], ret_via_nH=round(rv_nH, 3),
                 layer_change=len(change), L_rp_nH=round(L_rp, 3), xt_victim=round(XT['victim'].get(n, 0.0), 4),
                 xt_disturb=round(XT['disturb'].get(n, 0.0), 4), sw_area_mm2=round(A_mm2, 2) if n in sw_nets else None,
                 carries=carries, tr_ns=tr_, per_layer={k: round(v, 2) for k, v in sorted(per_layer.items())})
        if minimax:
            from analysis.minimax import load_contract
            m.update(safety_raw=dict(I_A=I, J_x=j_x, dV_mV=dV, s_dc_mV=s_dc),
                     via_I_A=tree_idx['via_I'] if tree_idx else None,
                     load_contract=load_contract(n, K, I, I_q, ir) if carries else None)
        out[n] = dict(net_class=nc, rank=N[n].get('rank') or 0, m=m, change=change[:5])

    HL = hot_loops(B, R, K, A, GND, drill)
    for h in HL:
        z = h['sw'] if h['sw'] in out else h['vin']
        if z in out:
            m = out[z]['m']
            if m.get('hot_L_nH') is None or h['L_nH'] > m['hot_L_nH']: m.update(hot_L_nH=h['L_nH'], hot_A_mm2=h['A_mm2'], hot_regulator=h['chip'])

    MK = ground_coupling(B, R, K, A, GND)
    if MK:
        for n, e in MK['per_net'].items():
            if n in out: out[n]['m']['ground_uV'] = e

    from analysis.signal_integrity import eyes, spec, count
    from analysis.coupling import signal_coupling
    AX, AS, AP, SI = {}, {}, [], {}
    if si_active:
        SX, AS, AP = signal_coupling(B, R, K, A, out, tr_ns)
        AX, SI = eyes(B, R, K, A, out, G, SX)
        for n, e in AX.items():
            out[n]['m'].update(eye_v_pct=e['vertical_pct'], eye_h_pct=e['horizontal_pct'], eye_skew_x=e.get('skew_x', 0.0))
        for n, e in AS.items(): out[n]['m'].update(disturb_mV=e['disturb_mV'], noise_budget_mV=e['budget_mV'])
        for n, e in out.items():
            m = e['m']
            if m.get('carries'):
                m['s_loss_mW'] = count(spec(K, n, 'loss_budget_mW'),
                                         max(1e-9, (m.get('I_A') or 0) * (m.get('s_dc_mV') or 30)), 1e-9)
    group_list = {}
    for g, (nn, mv) in BG.items():
        group_list[g] = dict(net_list=nn, mV=mv, total_sum_mV=round(sum(out[x]['m']['dV_mV'] for x in nn if x in out), 6 if si_active else 3),
                          source_list=sorted({out[x]['m']['dV_source'].split('(')[0] for x in nn if x in out}))
        if minimax: group_list[g]['total_sum_raw_mV'] = sum(out[x]['m']['safety_raw']['dV_mV'] for x in nn if x in out)
    sites.sort(reverse=True)
    return dict(version=G.get('version', 'v3.1'), **({'model_revision': VERSION} if review else {}), eyes=AX, interfaces=SI, analog_disturb=AS, disturb_pairs=AP, **(dict(layer_stack=LAYER_STACK, areas_ir={n: {k: e.get(k) for k in ('status', 'current_mode', 'total_current_A', 'nodes', 'warnings', 'assumptions', 'copper_area_per_layer_mm2')} for n, e in plane_ir.items()}, area_split=dict(model='Voronoi connections; estimated, no DRC', zone_list=len(FR.get('zones', [])))) if la_active else {}), net_list=out, xt_pairs=XT['pairs'][:30], hot_loop=HL, ground=MK, group_list=group_list, current_sites=sites[:20],
                **({'copper_ir': ir, 'copper_model': 'r36-dc-v1' if r36 else 'copper_map-v1'} if copper_model else {}),
                loss_mW=round(sum(v['m']['P_mW'] or 0.0 for v in out.values()), 3),
                supply_areas=sorted(supply_areas), ground_areas=sorted(({n for nn in FLMAP.values() for n in nn} if la_active else set(R.get('area_nets') or [])) & GND),
                structure=dict(stack=A['stack'], levels=A['levels'], dielectrics=A['dielectrics'],
                            cu_layers={k: {q: (round(w, 4) if isinstance(w, float) else w) for q, w in v.items()} for k, v in A['cu_layers'].items()}))




def net_x(m, C, sw):
    s = C.get('s') or {}; x = {}
    tr_ = m['tr_ns'] if 'tr_ns' in m else C.get('tr_ns')
    s_ret = 10 * tr_ if tr_ else s.get('ret_nH')
    if s_ret: x['ret'] = m['L_rp_nH'] / s_ret
    if m.get('l_crit_mm'): x['total_length'] = m['length_mm'] / m['l_crit_mm']
    if s.get('c_load_pF'): x['c_load'] = m['C_pF'] / s['c_load_pF']
    if s.get('l_load_nH'): x['l_load'] = m['L_nH'] / s['l_load_nH']
    if s.get('vias'): x['vias'] = m['vias'] / s['vias']
    if s.get('xt_victim'): x['xt_victim'] = m['xt_victim'] / s['xt_victim']
    if s.get('xt_disturb'): x['xt_disturb'] = m['xt_disturb'] / s['xt_disturb']
    if m.get('carries') and m.get('s_dc_mV') and not m.get('group'): x['dc'] = m['dV_mV'] / m['s_dc_mV']
    if m.get('carries') and m.get('J_x') is not None: x['j'] = m['J_x']
    if s.get('sw_area_mm2') and sw and m.get('sw_area_mm2') is not None: x['sw_area'] = m['sw_area_mm2'] / s['sw_area_mm2']
    if s.get('detour_mm'): x['detour'] = max(0.0, m['length_mm'] - m['air_line_mm']) / s['detour_mm']
    if m.get('hot_L_nH') is not None and s.get('hot_L_nH'): x['hot_L'] = m['hot_L_nH'] / s['hot_L_nH']
    if m.get('hot_A_mm2') is not None and s.get('hot_A_mm2'): x['hot_A'] = m['hot_A_mm2'] / s['hot_A_mm2']
    if m.get('ground_uV') is not None and s.get('ground_uV'): x['ground'] = m['ground_uV'] / s['ground_uV']
    if m.get('eye_v_pct') is not None and s.get('eye_v_loss_pct'):
        x['eye_v'] = (100 - m['eye_v_pct']) / s['eye_v_loss_pct']
    if m.get('eye_h_pct') is not None and s.get('eye_h_loss_pct'):
        x['eye_h'] = (100 - m['eye_h_pct']) / s['eye_h_loss_pct']
    if m.get('eye_skew_x') is not None: x['eye_skew'] = m['eye_skew_x']
    if m.get('disturb_mV') is not None: x['disturb'] = m['disturb_mV'] / m['noise_budget_mV']
    if m.get('carries') and m.get('s_loss_mW') and m.get('P_mW') is not None:
        x['power'] = m['P_mW'] / m['s_loss_mW']
    return x


def evaluate(M, weights=None):
    G = weights or load_weights(); bend = float(G.get('bend', 4.0)); wr = float(G.get('w_rank', 0.5))
    kj = G.get('bend_per') or {}
    for n, v in M['net_list'].items():
        C = G['netclasses'].get(v['net_class']) or {}
        g = C.get('g') or {}
        x = net_x(v['m'], C, v['net_class'] == 'sw')
        part_list = {k: g.get(k, 0) * penalty(xv, float(kj.get(k, bend))) for k, xv in sorted(x.items()) if g.get(k, 0)}
        Nn = sum(part_list.values()); W = 1 + wr * (v['rank'] or 0)
        v.update(rating=round(Nn, 3), weight=W, contribution=round(W * Nn, 3), x={k: round(xv, 3) for k, xv in x.items()},
                 part_list={k: round(t, 3) for k, t in part_list.items()})
    bt = {}
    gg = (G.get('group_list') or {}); xs = []
    for name, gr in sorted((M.get('group_list') or {}).items()):
        if not gr.get('mV'): continue
        xs.append(gr['total_sum_mV'] / gr['mV'])
        if gg.get('g'): bt['channel:' + name] = round(float(gg['g']) * penalty(xs[-1], float(kj.get('dc', bend))), 3)
    if xs and gg.get('g_max'):
        bt['channel_max'] = round(float(gg['g_max']) * penalty(max(xs), float(kj.get('dc', bend))), 3)

    if G.get('layer_stack', 0): bt['layer_stack'] = float(G['layer_stack']) * M.get('layer_stack', {}).get('total_sum', 0)
    M['board_parts'] = bt
    M['score_v3'] = round(sum(v['contribution'] for v in M['net_list'].values()) + sum(bt.values()), 2)
    if 'signal_integrity' in M:
        M['score_v3_basis'] = M['score_v3']
        if M.get('si_score_active'):
            M['score_v3'] = round(M['score_v3'] + M['signal_integrity']['score_extra'], 2)
    if 'minimax_version' in M:
        from analysis import minimax as MM
        MM.evaluate(M, G, M['minimax_profile'])
    return M


def measures(B, R, K, ir=None, weights=None, gnd_layers=None, AU=None, flow_cache=None, minimax=None, si_score=False, si_measure=False):
    if (K.get('specs') or {}).get('physics_version') == 'v3.3-contract_spec-option':
        from analysis.measures_physics import measures as physics
        return physics(B, R, K, ir=ir, weights=weights, gnd_layers=gnd_layers, AU=AU,
                      flow_cache=flow_cache, minimax=minimax, si_score=si_score, si_measure=si_measure)
    t0 = time.time(); c0 = time.process_time()
    G = weights or load_weights(si=(K.get('specs') or {}).get('si_evaluation') is True)
    if minimax is None:
        from analysis.standard import goal
        minimax = goal(K.get('specs')) == 'minimax'
    M = measure(B, R, K, ir=ir, weights=G, gnd_layers=gnd_layers, AU=AU, flow_cache=flow_cache, minimax=minimax)
    if (si_measure or si_score) and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('length_group') for k in K['net_list'].values()) or (K.get('specs') or {}).get('length_groups')):
        from analysis.length_impedance import measure as si_measure
        SR = dict(R, gnd_layers=gnd_layers) if gnd_layers is not None else R
        M['signal_integrity'] = si_measure(B, SR, K)
        M['si_score_active'] = bool(si_score)
        if si_score: M['version'] += '+r35-si2-option'
    evaluate(M, G)
    if minimax:
        from analysis import minimax as MM
        MM.measure_extra(M, B, R, K, G, ir=ir)
        MM.evaluate(M, G, (K.get('specs') or {}).get('profile_data', 'balanced'))
    M['time_s'] = round(time.time() - t0, 3); M['cpu_s'] = round(time.process_time() - c0, 3)
    return M


def table(M, n=10):
    z = []
    if 'signal_integrity' in M:
        z.append('  SI extra score %.3f (%s; heuristic, uncalibrated)' % (
            M['signal_integrity']['score_extra'], 'opened' if M.get('si_score_active') else 'separate, si_score=1 to add'))
    for name, v in sorted(M['net_list'].items(), key=lambda kv: -kv[1]['contribution'])[:n]:
        m = v['m']; top = sorted(v['part_list'].items(), key=lambda kv: -kv[1])[:3]
        z.append('  %-22s %-10s %6.2f  [%s]  l %.1f/%s mm, Vias %d, L %.2f nH, C %.2f pF, L_rp %.2f nH (without %.1f, edge %.1f, gap %d, change %d), '
                 'XT %.3f/%.3f, R %.1f mOhm, dV %.2f mV%s' % (
                     name.split('/')[-1][:22], v['net_class'], v['contribution'], ', '.join('%s %.2f (x %.2f)' % (k, w, v['x'][k]) for k, w in top),
                     m['length_mm'], m['l_crit_mm'], m['vias'], m['L_nH'], m['C_pF'], m['L_rp_nH'], m['l_without_mm'], m['l_border_mm'], m['n_gap'],
                     m['layer_change'], m['xt_victim'], m['xt_disturb'], m['R_mOhm'], m['dV_mV'],
                     ', P %.1f mW, J x %.2f' % (m['P_mW'] or 0, m['J_x'] or 0) if m.get('carries') else ''))
        if m.get('eye_v_pct') is not None:
            z[-1] += ' | Eye V/H %.1f/%.1f %% (model)' % (m['eye_v_pct'], m['eye_h_pct'])
        if m.get('disturb_mV') is not None:
            z[-1] += ' | Analog %.4f / %.4f mV (model)' % (m['disturb_mV'], m['noise_budget_mV'])
    for h in M.get('hot_loop') or []:
        z.append('  Hot Loop %-8s %s -> %s:%s -> %s:%s: L %.2f nH (VIN %.2f, %.1f mm, %d vias, %s; PGND %s; C ground %s), area %.1f mm2' % (
            h['chip'], h['c_in'], h['chip'], h['vin_pin'], h['chip'], h['pgnd_pin'], h['L_nH'], h['L_vin_nH'], h['l_vin_mm'], h['vias_vin'],
            h['vin_way'], h['pgnd'], h['cground'], h['A_mm2']))
    if M.get('ground'):
        mk = M['ground']
        z.append('  Ground coupling max %.2f uV (reference %s, %d aggressors): %s' % (mk['max_uV'], mk['baseline'], len(mk['attacker']),
                 ', '.join('%s %.2f' % (k.split('/')[-1], v) for k, v in sorted(mk['per_net'].items(), key=lambda kv: -kv[1])[:6])))
    for k, v in sorted((M.get('board_parts') or {}).items()):
        if ':' in k:
            g = M['group_list'].get(k.split(':', 1)[1]) or {}
            z.append('  %-12s %6.2f  (total %.1f of %.0f mV)' % (k, v, g.get('total_sum_mV', 0), g.get('mV', 0)))
        else:
            z.append('  %-12s %6.2f  (worst channel group)' % (k, v))
    for s in (M.get('current_sites') or [])[:3]:
        z.append('  Current density %s on %s at (%.1f, %.1f): %.2f mm instead of %.2f mm for %.2f A (J %.0f A/mm2, %.1f mm long), x %.2f' % (
            s[1].split('/')[-1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[0]))
    return '\n'.join(z)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    B = json.load(open(a[0])); R = json.load(open(a[1])); K = json.load(open(a[2]))
    if 'layer_stack' in kw:
        K = dict(K, specs=dict(K.get('specs') or {}, layer_stack=kw['layer_stack'] == '1'))
    M = measures(B, R, K, gnd_layers=kw['gnd_layers'].split(',') if 'gnd_layers' in kw else None, si_score=kw.get('si_score') == '1', si_measure=kw.get('si_measure') == '1')
    print('Score v3 %.1f (%d nets, %.3f s CPU); setup %s, layers %s' % (M['score_v3'], len(M['net_list']), M['cpu_s'],
          '/'.join('%.3f' % d for d in M['structure']['dielectrics']), ','.join(M['structure']['levels'])))
    print(table(M, int(kw.get('n', '10'))))
    if 'json' in kw: json.dump(M, open(kw['json'], 'w'), indent=1)
