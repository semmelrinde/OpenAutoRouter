import json, math, os, sys
from collections import Counter, defaultdict

from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.copper_net import Obstacles, NetCopper, apply
from analysis.finished_metrics import corners90_new

S2 = math.sqrt(0.5)
RICHT = [(1, 0), (S2, S2), (0, 1), (-S2, S2), (-1, 0), (-S2, -S2), (0, -1), (S2, -S2)]


def K(p): return (round(p[0], 6), round(p[1], 6))


def _unit(a, b):
    d = math.dist(a, b)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d) if d > 1e-9 else None


def _octi(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; ax, ay = abs(dx), abs(dy)
    if ax < 1e-6 or ay < 1e-6 or abs(ax - ay) < 1e-6: return [[a, b]]
    m = min(ax, ay); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    return [[a, (a[0] + sx * m, a[1] + sy * m), b], [a, (b[0] - sx * m, b[1] - sy * m), b]]


def _cut(p, r, q, s):
    the = r[0] * s[1] - r[1] * s[0]
    if abs(the) < 1e-12: return None
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / the
    u = ((q[0] - p[0]) * r[1] - (q[1] - p[1]) * r[0]) / the
    return t, u


def _shape_layer(H, net, L, tracks):
    from analysis.finished_metrics import gate_check
    Hn = dict(H, tracks=[dict(t, net=net, layer=L) for t in tracks], vias=[v for v in H['vias'] if v['net'] == net],
              pads=[p for p in H['pads'] if p['net'] == net])
    return gate_check(Hn)['hairpins'], Hn


def middle(H, keepout_list=(), net_list=None, cu_layers=None, fwd=None, eps=0.005, pads=True, pad_max=1.6, minseg=0.03, push=1.0, diag=None, fix=(), short_min=0.1,
          hmax=None):
    keepout_list = set(keepout_list); fwd = fwd or Obstacles(H); fix = set(fix)
    NET_COPPER = NetCopper(H); stat = Counter(); remove, add, sites = [], [], []
    all_items = sorted({t['net'] for t in H['tracks'] if t['net']} if net_list is None else set(net_list))
    for net in all_items:
        if net in keepout_list: stat['blocked'] += 1; continue
        n_rm, n_add, n_st = [], [], []
        for L in sorted({t['layer'] for t in NET_COPPER.tracks.get(net, []) if cu_layers is None or t['layer'] in cu_layers}):
            state = (len(n_rm), len(n_add), len(n_st))
            segs = [t for t in NET_COPPER.tracks[net] if t['layer'] == L]
            arc = {K(t[e]) for t in segs if t.get('arc') for e in ('a', 'b')}
            straight = [t for t in segs if not t.get('arc') and t['a'] != t['b']]
            nb = defaultdict(list)
            for t in straight: nb[K(t['a'])].append(t); nb[K(t['b'])].append(t)
            targets = [('via', (v['x'], v['y']), Point(v['x'], v['y']).buffer(v['d'] / 2, 32)) for v in H['vias']
                     if v['net'] == net and L in (v.get('layers') or [L])]
            if pads:
                for p in H['pads']:
                    if p['net'] != net or p['layer'] != L or p.get('type_') not in ('smd', 'conn') or 'x' not in p: continue
                    g = Polygon(p['poly']).buffer(0); x0, y0, x1, y1 = g.bounds
                    if max(x1 - x0, y1 - y0) <= pad_max and g.contains(Point(p['x'], p['y'])): targets.append(('pad', (p['x'], p['y']), g))
            fixed_pkt = [g for _, _, g in targets]


            centers = [(v['x'], v['y']) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])] + \
                      [(p['x'], p['y']) for p in H['pads'] if p['net'] == net and p['layer'] == L and p.get('type_') in ('smd', 'conn') and 'x' in p
                       and len(p.get('poly') or []) >= 3 and Polygon(p['poly']).buffer(0).contains(Point(p['x'], p['y']))]
            def touched(lines):
                return {i for i, g in enumerate(copper_map) if any(l.intersects(g) for l in lines)}
            def reached(lines):
                return {c for c in centers if any(g.distance(Point(c)) <= 0.005 for g in lines)}
            copper_map = [g for category, _, g in targets if category == 'via'] + [Polygon(p['poly']).buffer(0) for p in H['pads']
                                                                 if p['net'] == net and p['layer'] == L and len(p.get('poly') or []) >= 3]
            used = {t['uuid'] for t in straight if t.get('uuid') in fix}
            for E in sorted(nb):
                wE = min(t['w'] for t in nb[E])

                goal = next(((category, c, g) for category, c, g in targets if g.distance(Point(E)) <= (wE / 2 if len(nb[E]) == 1 else 1e-9)), None)
                if goal is None: continue
                category, C, gz = goal
                if math.dist(E, C) <= eps: continue
                stat['Ends next to the center (%s)' % category] += 1
                for s in sorted(nb[E], key=lambda t: t['uuid']):
                    if s['uuid'] in used or E in arc: continue
                    Sx = tuple(s['b']) if K(s['a']) == E else tuple(s['a'])
                    if gz.distance(Point(Sx)) <= 1e-9: stat['Piece entirely in the target'] += 1; continue

                    on_S = [t for t in nb[K(Sx)] if t is not s]
                    prev = on_S[0] if (len(on_S) == 1 and on_S[0]['uuid'] not in used and K(Sx) not in arc
                                       and not any(g.distance(Point(Sx)) <= 1e-9 for g in fixed_pkt)) else None
                    P = None if prev is None else (tuple(prev['b']) if K(prev['a']) == K(Sx) else tuple(prev['a']))


                    replaced = [s] + ([prev] if prev else [])
                    axes = [LineString([t['a'], t['b']]) for t in replaced]
                    own = {K(E), K(Sx)} | ({K(P)} if P else set())
                    contacts = []
                    for t in straight:
                        if t in replaced: continue
                        for e in ('a', 'b'):
                            q = K(t[e])
                            if q in own or gz.distance(Point(q)) <= 1e-9: continue
                            if any(ax.distance(Point(q)) <= (t['w'] + u['w']) / 2 + 1e-6 for ax, u in zip(axes, replaced)):

                                contacts.append((Point(q), t['w'] / 2, any(ax.distance(Point(q)) <= 1e-3 for ax in axes)))
                    for g in copper_map:
                        if g is gz or g.distance(Point(E)) <= 1e-9: continue
                        if any(ax.distance(g) <= u['w'] / 2 + 1e-6 for ax, u in zip(axes, replaced)): contacts.append((g, 0.0, False))
                    cand = []
                    d0 = _unit(Sx, E)

                    if d0 and abs((C[0] - Sx[0]) * d0[1] - (C[1] - Sx[1]) * d0[0]) < 1e-5 and (C[0] - Sx[0]) * d0[0] + (C[1] - Sx[1]) * d0[1] > minseg:
                        cand.append(('A', [Sx, C], [s['w']], [s]))
                    if prev is not None:
                        rp = (Sx[0] - P[0], Sx[1] - P[1]); lp = math.hypot(*rp)
                        dp = (rp[0] / lp, rp[1] / lp) if lp > 1e-9 else None
                        opts = []
                        for d in RICHT:
                            if dp is None or dp[0] * d[0] + dp[1] * d[1] < -1e-9: continue
                            if abs(dp[0] * d[1] - dp[1] * d[0]) < 1e-9: continue
                            tu = _cut(P, rp, C, (-d[0], -d[1]))
                            if tu is None: continue
                            t_, u_ = tu; S1 = (P[0] + t_ * rp[0], P[1] + t_ * rp[1])
                            if t_ * lp < minseg or u_ < minseg or math.dist(S1, Sx) > push: continue
                            equal = d0 is not None and abs(d[0] - d0[0]) < 1e-6 and abs(d[1] - d0[1]) < 1e-6
                            opts.append((0 if equal else 1, t_ * lp + u_, d, S1))
                        for _, _, d, S1 in sorted(opts):
                            cand.append(('B', [P, S1, C], [prev['w'], s['w']], [prev, s]))


                        if abs(prev['w'] - s['w']) < 1e-9:
                            prev_d = LineString([P, Sx, E])
                            for c in sorted(_octi(P, C), key=lambda c: LineString(c).hausdorff_distance(prev_d)):
                                if LineString(c).hausdorff_distance(prev_d) > 0.3: continue
                                cand.append(('D', c, [s['w']] * (len(c) - 1), [prev, s]))



                    s_middle = any(math.dist(Sx, c2) <= eps for _, c2, _ in targets)
                    e_inner = any(t is not s and gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9 for t in nb[E])
                    if prev is None and s_middle and e_inner:
                        stat['Bend in the target, both ends in targets'] += 1; continue
                    if prev is None:
                        prev_l = math.dist(Sx, E)
                        prev_ls = LineString([Sx, E])
                        for c in sorted(_octi(Sx, C), key=lambda c: LineString(c).hausdorff_distance(prev_ls)):
                            if len(c) == 3 and min(math.dist(c[0], c[1]), math.dist(c[1], c[2])) < minseg: continue
                            if LineString(c).length > prev_l + 0.3: continue
                            w1 = _unit(c[0], c[1])
                            if w1 and any((lambda o: o and o[0] * w1[0] + o[1] * w1[1] > 1e-9)(_unit(Sx, tuple(t['b']) if K(t['a']) == K(Sx) else tuple(t['a'])))
                                          for t in on_S):
                                continue
                            cand.append(('C', c, [s['w']] * (len(c) - 1), [s]))
                    def holds(pts, ws):
                        new_ax = [(LineString([a, b]), w) for (a, b), w in zip(zip(pts, pts[1:]), ws)]
                        return all(any(ax.distance(g) <= (1e-3 if exact else h + w / 2 + 1e-6) for ax, w in new_ax) for g, h, exact in contacts)
                    def visible_short(a, b):
                        g = LineString([a, b]) if a != b else Point(a)
                        return math.dist(a, b) < short_min and not any(z.buffer(1e-6).contains(g) for z in copper_map)
                    def shaping(pts_, prior):
                        au = {t['uuid'] for t in prior}
                        nb0 = [tuple(t['b']) if K(t['a']) == K(pts_[0]) else tuple(t['a']) for t in straight
                               if t['uuid'] not in au and K(pts_[0]) in (K(t['a']), K(t['b']))]


                        nbe = [tuple(t['b']) if K(t['a']) == K(pts_[-1]) else tuple(t['a']) for t in straight
                               if t['uuid'] not in au and K(pts_[-1]) in (K(t['a']), K(t['b'])) and not (len(pts_) > 1 and K(pts_[-1]) == K(pts_[0]))
                               and not (rest_falls(au) and E in (K(t['a']), K(t['b'])) and gz.distance(Point(t['a'])) <= 1e-9
                                        and gz.distance(Point(t['b'])) <= 1e-9)]

                        q = ([nb0[0]] if len(nb0) == 1 else []) + list(pts_) + ([nbe[0]] if len(nbe) == 1 else [])
                        corners = needles = 0
                        if len(nb0) > 1:
                            v2 = _unit(pts_[0], pts_[1])
                            needles += sum(1 for o_ in nb0 if v2 and _unit(pts_[0], o_) and
                                          _unit(pts_[0], o_)[0] * v2[0] + _unit(pts_[0], o_)[1] * v2[1] > 1e-6)
                        for a, b, c in zip(q, q[1:], q[2:]):
                            v1, v2 = _unit(b, a), _unit(b, c)
                            if not v1 or not v2: continue
                            cs = v1[0] * v2[0] + v1[1] * v2[1]
                            needles += cs > 1e-6
                            if not any(z.distance(Point(b)) <= 1e-9 for z in copper_map): corners += abs(cs) <= math.sin(math.radians(0.5))
                        return corners, needles, frozenset((K(a), K(b)) if K(a) <= K(b) else (K(b), K(a)) for a, b in zip(pts_, pts_[1:]) if visible_short(a, b))
                    def rest_falls(au):
                        return all(t['uuid'] in au or t['uuid'] in used for t in nb[E]
                                   if not (gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9))
                    prev_shape = {}
                    for type_, pts, ws, prior in cand:


                        ka = tuple(t['uuid'] for t in prior)
                        if ka not in prev_shape:
                            prev_shape[ka] = shaping([P, Sx, E] if len(prior) == 2 else [Sx, E], prior)
                        h_ = (hmax or {}).get((net, L), 0.0)
                        if h_ and LineString(pts).hausdorff_distance(LineString([P, Sx, E] if len(prior) == 2 else [Sx, E])) > h_:
                            stat['Approach rejected (hmax)'] += 1; continue
                        f_ = shaping(pts, prior)
                        if f_[0] > prev_shape[ka][0] or f_[1] > prev_shape[ka][1] or f_[2] - prev_shape[ka][2]:
                            stat['Shape worse (90°, hairpin, short piece)'] += 1; continue
                        if not holds(pts, ws): stat['Contact at the approach would be lost'] += 1; continue
                        if reached([LineString([t['a'], t['b']]) for t in prior]) - reached([LineString([a, b]) for a, b in zip(pts, pts[1:])]):
                            stat['other center would be lost'] += 1; continue


                        if touched([LineString([t['a'], t['b']]) for t in prior]) - touched([LineString([a, b]) for a, b in zip(pts, pts[1:]) if a != b]):
                            stat['Pad/via would be lost'] += 1; continue
                        if all(fwd.vacant(a, b, w, L, net) for (a, b), w in zip(zip(pts, pts[1:]), ws)):
                            for t in prior: used.add(t['uuid'])
                            n_rm += [t['uuid'] for t in prior]
                            new = [dict(net=net, layer=L, a=list(a), b=list(b), w=w) for (a, b), w in zip(zip(pts, pts[1:]), ws)]
                            n_add += new
                            n_st.append(dict(net=net, layer=L, category=category, type_=type_, middle=list(C), end=list(E), offset=round(math.dist(E, C), 4)))
                            break
                    else:
                        stat['no free approach'] += 1
                        if diag is not None: diag.append(dict(net=net, layer=L, category=category, end=list(E), middle=list(C), offset=round(math.dist(E, C), 4),
                                                              prev=prev is not None, candidates=[k[0] for k in cand], length_s=round(math.dist(Sx, E), 4)))

                inner = [t for t in nb[E] if gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9]
                outer = [t for t in nb[E] if t not in inner]


                if inner and outer and all(t['uuid'] in used and t['uuid'] not in fix for t in outer) and \
                        any(LineString([a_['a'], a_['b']]).distance(Point(C)) <= 0.005 for a_ in n_add if a_['a'] != a_['b'] and a_['layer'] == L):
                    for t in inner:
                        if t['uuid'] not in used:
                            used.add(t['uuid']); n_rm.append(t['uuid']); stat['Remaining piece in the target removed'] += 1


            if len(n_rm) > state[0] or len(n_add) > state[1]:
                rm_L, add_L = n_rm[state[0]:], n_add[state[1]:]
                prev_L = [t for t in segs]; new_L = apply(prev_L, rm_L, add_L)
                (n1, H1_), (n0, H0_) = _shape_layer(H, net, L, new_L), _shape_layer(H, net, L, prev_L)
                if n1 > n0 or corners90_new(H0_, H1_):
                    stat['Layer rejected (hairpin/90° in the cross-check)'] += 1
                    del n_rm[state[0]:]; del n_add[state[1]:]; del n_st[state[2]:]
        if not n_rm: continue
        prior = NET_COPPER.tracks.get(net, [])
        if NET_COPPER.separates(net, prior, apply(prior, n_rm, n_add)):
            stat['rejected (splits net)'] += 1; print('rejected, splits the net:', net, file=sys.stderr); continue
        for t in n_add: fwd.extra(net, t['layer'], t['a'], t['b'], t['w'])
        remove += n_rm; add += n_add; sites += n_st
        for s_ in n_st: stat['to the center (%s, %s)' % (s_['category'], s_['type_'])] += 1
    return dict(remove=remove, add=add, sites=sites, stat=dict(sorted(stat.items())))

def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from postprocessing.keepout_nets import keepout_nets, net_spacings
    net_list = {t['net'] for t in H['tracks']}
    S = keepout_nets(pcb=o.get('pcb'), pro=o.get('pro'), dru=o.get('dru'), netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'),
                   net_list=net_list, pads=H.get('pads'), net_ties=H.get('net_ties'))
    dru = o.get('dru') or (os.path.splitext(o['pcb'])[0] + '.kicad_dru' if o.get('pcb') else None)
    fwd = Obstacles(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', o.get('drcpad', 0.10))),
                      drcvia=float(o.get('drcvia', o.get('drc', 0.09))), hole_size=float(o.get('hole_size', 0.25)), edge=float(o.get('edge', 0.2)),
                      net_spacing=net_spacings(dru, net_list))
    R = middle(H, keepout_list=S, net_list=[n for n in o['net_list'].split(';') if n] if o.get('net_list') else None,
              cu_layers=o['cu_layers'].split(',') if o.get('cu_layers') else None, fwd=fwd, eps=float(o.get('eps', 0.005)),
              pads=o.get('pads', '1') == '1', pad_max=float(o.get('pad_max', 1.6)), minseg=float(o.get('minseg', 0.03)),
              push=float(o.get('push', 1.0)), fix=json.load(open(o['fix'])) if o.get('fix') else (), short_min=float(o.get('short_min', 0.1)))
    json.dump(R, open(a[1], 'w'))
    print(json.dumps(R['stat'], ensure_ascii=False), 'removed', len(R['remove']), 'new', len(R['add']))


if __name__ == '__main__':
    main()
