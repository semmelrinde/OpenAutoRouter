import json, math, os, sys
from collections import Counter, defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.copper_net import Obstacles, NetCopper, apply


MIDDLE_EPS = 0.005


def K(p): return (round(p[0], 6), round(p[1], 6))


def _direction(a, b):
    d = math.dist(a, b)
    return None if d < 1e-9 else ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _line(a, b):
    r = _direction(a, b)
    if r[0] < -1e-9 or (abs(r[0]) <= 1e-9 and r[1] < 0): r = (-r[0], -r[1])
    n = (-r[1], r[0]); off = a[0] * n[0] + a[1] * n[1]
    return (round(r[0], 6), round(r[1], 6), round(off, 6)), r, a[0] * r[0] + a[1] * r[1], b[0] * r[0] + b[1] * r[1], n, off


def _octi(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; ax, ay = abs(dx), abs(dy)
    if ax < 1e-4 or ay < 1e-4 or abs(ax - ay) < 1e-4: return [[a, b]]
    m = min(ax, ay); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    return [[a, (b[0] - sx * m, b[1] - sy * m), b], [a, (a[0] + sx * m, a[1] + sy * m), b]]


def _stretch(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def tidy_up(H, keepout_list=(), net_list=None, fwd=None, micro=0.03, fix=()):
    keepout_list = set(keepout_list); fwd = fwd or Obstacles(H); fix = set(fix)
    NET_COPPER = NetCopper(H); stat = Counter(); remove, add = [], []
    for net in sorted({t['net'] for t in H['tracks'] if t['net']} if net_list is None else set(net_list)):
        if net in keepout_list: continue
        prev_net = NET_COPPER.tracks.get(net, []); n_rm, n_add = [], []
        for L in sorted({t['layer'] for t in prev_net}):
            segs = [t for t in prev_net if t['layer'] == L]
            arc = {K(t[e]) for t in segs if t.get('arc') or t.get('uuid') in fix for e in ('a', 'b')}
            free_ = [t for t in segs if not t.get('arc') and t.get('uuid') not in fix and K(t['a']) not in arc and K(t['b']) not in arc]
            if not free_: continue
            vias = [(Point(v['x'], v['y']), v['d'] / 2) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])]
            pads_ = [p for p in H['pads'] if p['net'] == net and p['layer'] == L and p.get('type_', 'smd') in ('smd', 'conn') and len(p['poly']) >= 3]
            padg = [Polygon(p['poly']).buffer(0) for p in pads_]
            targets = [(c.buffer(r, 64), (c.x, c.y)) for c, r in vias] + [(g, (p['x'], p['y'])) for g, p in zip(padg, pads_) if 'x' in p]
            def on_goal(p):
                q = Point(p)
                return any(c.distance(q) <= r for c, r in vias) or any(g.distance(q) <= 1e-9 for g in padg)

            S = {i: [tuple(t['a']), tuple(t['b']), t['w'], t['uuid']] for i, t in enumerate(free_)}
            fixed_ends = [(tuple(t[e]), t['w']) for t in segs if t not in free_ for e in ('a', 'b')]
            def hangs_on(p, w, except_=()):
                for j, (a, b, wj, _) in S.items():
                    if j in except_: continue
                    for e in (a, b):
                        if 1e-9 < math.dist(e, p) <= (wj + w) / 2: return True
                return any(1e-9 < math.dist(e, p) <= (wj + w) / 2 for e, wj in fixed_ends)
            next_ = [len(S)]
            def new_id():
                next_[0] += 1; return next_[0]

            seen = {}
            for i in sorted(S):
                a, b, w, _ = S[i]
                if math.dist(a, b) < 1e-6: del S[i]; stat['without length'] += 1; continue
                key = tuple(sorted([K(a), K(b)]))
                if key in seen:
                    j = seen[key]
                    if abs(S[j][2] - w) > 1e-9:
                        stat['duplicate, width differs (stays)'] += 1; continue
                    del S[i]
                    stat['doubled'] += 1
                else: seen[key] = i

            for gz, C in targets:
                gi = gz.buffer(-1e-4)


                other = [c for _, c in targets if math.dist(c, C) > 1e-6]
                def axis(i): return LineString([S[i][0], S[i][1]]) if math.dist(S[i][0], S[i][1]) > 1e-9 else Point(S[i][0])
                inner = [i for i in sorted(S) if gi.contains(Point(S[i][0])) and gi.contains(Point(S[i][1]))
                         and not any(axis(i).distance(Point(c)) <= MIDDLE_EPS for c in other)]
                if not inner: continue
                pkt_inner = {K(S[i][e]) for i in inner for e in (0, 1)}
                axes_inner = [(LineString([S[i][0], S[i][1]]), S[i][2]) for i in inner]
                gz_ = gz.buffer(1e-6)
                end_count = Counter(K(S[x][e_]) for x in S for e_ in (0, 1)) + Counter(K(p_) for p_, _ in fixed_ends)


                already_inside = {K(S[x][1 - e_]) for x in S if x not in inner for e_ in (0, 1) if K(S[x][e_]) in pkt_inner}
                already_inside |= {K(c) for c in other}
                outer_on = defaultdict(list)
                provenance = defaultdict(list)
                exact = any(K(S[x][e_]) in pkt_inner for x in S if x not in inner for e_ in (0, 1))
                for j, (a, b, w, _) in S.items():
                    if j in inner: continue
                    for e, o_ in ((a, b), (b, a)):


                        if K(e) in pkt_inner:
                            outer_on[K(e)].append(w); provenance[K(e)].append(o_); continue

                        if exact or K(o_) in pkt_inner or K(e) in already_inside: continue


                        if not (gz_.contains(Point(e)) or end_count[K(e)] == 1): continue
                        if (gz.distance(Point(e)) <= w / 2 + 1e-9 and any(ax.distance(Point(e)) <= (w + wi) / 2 + 1e-9 for ax, wi in axes_inner)) or \
                                (end_count[K(e)] == 1 and gz.distance(Point(e)) <= w / 2 + 1e-9):

                            outer_on[K(e)].append(w); provenance[K(e)].append(o_)
                to_middle = any(axis(i).distance(Point(C)) <= MIDDLE_EPS for i in inner)
                nominal = sorted((E, max(ws)) for E, ws in outer_on.items() if to_middle and math.dist(E, C) > 1e-6)

                paths = []
                def without_wende(E, c):
                    v2 = (c[1][0] - c[0][0], c[1][1] - c[0][1])
                    return all((E[0] - o_[0]) * v2[0] + (E[1] - o_[1]) * v2[1] >= -1e-9 for o_ in provenance[E])
                prev_axes = unary_union([ax for ax, _ in axes_inner])
                for E, w in nominal:
                    hull = gz.buffer(1e-6) if gz.contains(Point(E)) else gz.buffer(w / 2 + 1e-6)
                    cand = sorted(_octi(E, C), key=lambda c: LineString(c).hausdorff_distance(prev_axes))
                    paths.append(next((c for c in cand if hull.contains(LineString(c)) and without_wende(E, c)
                                      and all(fwd.vacant(a, b, w, L, net) for a, b in zip(c, c[1:]))), None))
                if any(c is None for c in paths): stat['Tangle stays (no path to the center)'] += 1; continue

                def way_r(c):
                    d_ = math.dist(c[-1], c[-2]) if len(c) > 1 else 0.0
                    return None if d_ < 1e-9 else ((c[-2][0] - c[-1][0]) / d_, (c[-2][1] - c[-1][1]) / d_)
                rs = [way_r(c) for c in paths]
                if any(a_ and b_ and a_[0] * b_[0] + a_[1] * b_[1] > 1e-6 and abs(a_[0] * b_[1] - a_[1] * b_[0]) > 1e-6
                       for x_, a_ in enumerate(rs) for b_ in rs[x_ + 1:]):
                    stat['Tangle stays (paths converged at a sharp angle)'] += 1; continue
                K4 = lambda p: (round(p[0], 4), round(p[1], 4))
                actual = sorted(tuple(sorted([K4(S[i][0]), K4(S[i][1])])) + (S[i][2],) for i in inner)
                if actual == sorted(tuple(sorted([K4(a), K4(b)])) + (w,) for (E, w), c in zip(nominal, paths) for a, b in zip(c, c[1:])): continue
                for i in inner: del S[i]
                for (E, w), c in zip(nominal, paths):
                    for a, b in zip(c, c[1:]): S[new_id()] = [tuple(a), tuple(b), w, None]
                stat['in the via/pad'] += len(inner); stat['of which replaced by piece to the center'] += len(nominal)

            group_list = defaultdict(list)
            for i, (a, b, w, _) in S.items():
                key, r, s0, s1, n, off = _line(a, b); group_list[(key, round(w, 6))].append((min(s0, s1), max(s0, s1), i, r, n, off))
            for (key, w), xs in sorted(group_list.items()):
                if len(xs) < 2: continue
                xs.sort(); runs = [[xs[0]]]
                for x in xs[1:]:
                    if x[0] < max(y[1] for y in runs[-1]) - 1e-6: runs[-1].append(x)
                    else: runs.append([x])
                for lf in runs:
                    if len(lf) < 2: continue
                    lo, hi = min(x[0] for x in lf), max(x[1] for x in lf); _, _, _, r, n, off = lf[0]
                    P = lambda s_: (n[0] * off + r[0] * s_, n[1] * off + r[1] * s_)
                    ids = {x[2] for x in lf}
                    if any(hangs_on(S[x[2]][e], w, ids) for x in lf for e in (0, 1) if math.dist(S[x[2]][e], P(lo)) > 1e-6 and math.dist(S[x[2]][e], P(hi)) > 1e-6):
                        stat['collinear stays (connection)'] += 1; continue
                    for x in lf: del S[x[2]]
                    S[new_id()] = [P(lo), P(hi), w, None]; stat['collinear merged'] += len(lf) - 1

            for _ in range(3):
                nb = defaultdict(list)
                for i, (a, b, w, _) in S.items(): nb[K(a)].append(i); nb[K(b)].append(i)
                changed = False
                for i in sorted(S):
                    if i not in S: continue
                    a, b, w, _ = S[i]
                    if math.dist(a, b) >= micro: continue
                    na = [j for j in nb[K(a)] if j != i and j in S]; nb_ = [j for j in nb[K(b)] if j != i and j in S]
                    if (not na) != (not nb_) and len(na + nb_) == 1:
                        free_end = a if not na else b

                        on_axis = any(_stretch(free_end, S[x][0], S[x][1]) <= (S[x][2] + w) / 2 + 1e-9 for x in S if x != i) or \
                            any(_stretch(free_end, t['a'], t['b']) <= (t['w'] + w) / 2 + 1e-9 for t in segs if t not in free_)
                        if not on_goal(free_end) and not on_axis:
                            del S[i]; stat['MicroStub'] += 1; changed = True
                        continue
                    if len(na) != 1 or len(nb_) != 1 or on_goal(a) or on_goal(b): continue
                    j, k = na[0], nb_[0]
                    if hangs_on(a, w, {i, j, k}) or hangs_on(b, w, {i, j, k}): continue
                    pa = S[j][1] if K(S[j][0]) == K(a) else S[j][0]; pb = S[k][1] if K(S[k][0]) == K(b) else S[k][0]
                    r1, r2 = _direction(pa, a), _direction(pb, b)
                    if r1 is None or r2 is None: continue
                    the = r1[0] * r2[1] - r1[1] * r2[0]
                    if abs(the) < 1e-9: continue
                    t_ = ((pb[0] - pa[0]) * r2[1] - (pb[1] - pa[1]) * r2[0]) / the
                    X = (pa[0] + t_ * r1[0], pa[1] + t_ * r1[1])
                    if math.dist(X, a) > micro + w or math.dist(X, b) > micro + w: continue
                    if (X[0] - pa[0]) * r1[0] + (X[1] - pa[1]) * r1[1] <= 1e-6 or (X[0] - pb[0]) * r2[0] + (X[1] - pb[1]) * r2[1] <= 1e-6: continue


                    if r1[0] * r2[0] + r1[1] * r2[1] > 1e-9: continue
                    if abs(r1[0] * r2[0] + r1[1] * r2[1]) <= math.sin(math.radians(0.5)) and not on_goal(X):
                        stat['Micro piece stays (otherwise 90° corner)'] += 1; continue
                    if not (fwd.vacant(pa, X, S[j][2], L, net) and fwd.vacant(pb, X, S[k][2], L, net)): continue
                    wj, wk = S[j][2], S[k][2]
                    del S[i], S[j], S[k]
                    S[new_id()] = [pa, X, wj, None]; S[new_id()] = [X, pb, wk, None]
                    stat['Micro piece to the axis intersection'] += 1; changed = True
                if not changed: break

            while True:
                nb = defaultdict(list)
                for i, (a, b, w, _) in S.items(): nb[K(a)].append(i); nb[K(b)].append(i)
                tat = False
                for p, ids in sorted(nb.items()):
                    if len(ids) != 2: continue
                    i, j = ids
                    if i not in S or j not in S or abs(S[i][2] - S[j][2]) > 1e-9: continue
                    qa = S[i][0] if K(S[i][1]) == p else S[i][1]; qb = S[j][0] if K(S[j][1]) == p else S[j][1]
                    r1, r2 = _direction(qa, p), _direction(p, qb)
                    if r1 is None or r2 is None or abs(r1[0] * r2[1] - r1[1] * r2[0]) > 1e-7 or r1[0] * r2[0] + r1[1] * r2[1] <= 0: continue
                    if LineString([qa, qb]).distance(Point(p)) > 1e-6: continue
                    if hangs_on(p, S[i][2], {i, j}): continue
                    w = S[i][2]; del S[i], S[j]; S[new_id()] = [qa, qb, w, None]; stat['collinear combined'] += 1; tat = True
                    break
                if not tat: break
            prev_ids = {t['uuid'] for t in free_}
            remains = {s[3] for s in S.values() if s[3]}
            way = sorted(prev_ids - remains); new = [dict(net=net, layer=L, a=list(s[0]), b=list(s[1]), w=s[2]) for s in S.values() if not s[3]]
            if way or new: n_rm += way; n_add += new
        if not (n_rm or n_add): continue
        if NET_COPPER.separates(net, prev_net, apply(prev_net, n_rm, n_add)):
            stat['rejected (splits net)'] += 1; print('rejected, splits the net:', net, file=sys.stderr); continue
        for t in n_add: fwd.extra(net, t['layer'], t['a'], t['b'], t['w'])
        remove += n_rm; add += n_add; stat['Nets changed'] += 1
    return dict(remove=remove, add=add, stat=dict(sorted(stat.items())))


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from postprocessing.keepout_nets import keepout_nets, net_spacings
    net_list = {t['net'] for t in H['tracks']}
    S = keepout_nets(pcb=o.get('pcb'), pro=o.get('pro'), dru=o.get('dru'), netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'),
                   net_list=net_list, pads=H.get('pads'), net_ties=H.get('net_ties'))
    dru = o.get('dru') or (os.path.splitext(o['pcb'])[0] + '.kicad_dru' if o.get('pcb') else None)
    fwd = Obstacles(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', o.get('drcpad', 0.10))),
                      hole_size=float(o.get('hole_size', 0.25)), edge=float(o.get('edge', 0.2)), net_spacing=net_spacings(dru, net_list))
    R = tidy_up(H, keepout_list=S, net_list=[n for n in o['net_list'].split(';') if n] if o.get('net_list') else None, fwd=fwd,
                   micro=float(o.get('micro', 0.03)), fix=json.load(open(o['fix'])) if o.get('fix') else ())
    json.dump(R, open(a[1], 'w'))
    print(json.dumps(R['stat'], ensure_ascii=False), 'removed', len(R['remove']), 'new', len(R['add']))


if __name__ == '__main__':
    main()
