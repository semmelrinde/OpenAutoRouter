import collections, math

MICRO = 0.05


def _k(p):
    return (round(p[0], 4), round(p[1], 4))


def not_closer(fwd, e, a1, a2):
    from shapely.geometry import LineString
    L, net, w = e['L'], e['net'], e['w']
    new = LineString([a1, a2]); prior = LineString([a1, e['p'], a2])
    owned = fwd.net_spacing.get(net, 0.0)
    for i in fwd.tree_idx[L].query(new.buffer(w / 2 + fwd.dmax + fwd.res)) if L in fwd.tree_idx else []:
        n, g, d, track = fwd.obj[L][int(i)]
        if n == net: continue
        dd = max(d, owned if track else 0.0, fwd.net_spacing.get(n, 0.0) if track else 0.0)
        x = new.distance(g)
        if x < w / 2 + dd + fwd.res and x < prior.distance(g) - 1e-6: return False
    for n, g in fwd.new.get(L, []):
        if n != net and new.distance(g) < w / 2 + max(fwd.drc, owned, fwd.net_spacing.get(n, 0.0)) + fwd.res: return False
    return True


def corners45(H, keepout_list=(), net_list=None, fix=(), fwd=None, fill=None, cmax=0.4, cmin=0.08):
    from shapely.geometry import Point, Polygon
    S = set(keepout_list); F = set(fix); only = None if net_list is None else set(net_list)
    end = collections.defaultdict(list)
    for t in H['tracks']:
        if t.get('arc') or t['net'] in S or not t['net'] or t['uuid'] in F or (only is not None and t['net'] not in only): continue
        if math.dist(t['a'], t['b']) < 1e-6: continue
        key = (t['net'], t['layer'], round(t['w'], 6))
        end[(key, _k(t['a']))].append((t, 'a')); end[(key, _k(t['b']))].append((t, 'b'))
    all_ends = collections.Counter()
    for t in H['tracks']:
        for q in (t['a'], t['b']): all_ends[(t['net'], t['layer'], _k(q))] += 1
    pads = collections.defaultdict(list)
    for p in H.get('pads', []):
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    vias = collections.defaultdict(list)
    for v in H.get('vias', []): vias[v['net']].append((v['x'], v['y'], v['d'] / 2, v.get('layers')))
    zone_list = collections.defaultdict(list)
    for n, L, g in fill or []: zone_list[(n, L)].append(g)
    stat = collections.Counter(); corners = []
    for (key, p), lst in sorted(end.items(), key=lambda x: (x[0][0], x[0][1])):
        net, L, w = key
        if len(lst) != 2 or all_ends[(net, L, p)] != 2: continue
        (t1, e1), (t2, e2) = lst
        o1 = t1['b'] if e1 == 'a' else t1['a']; o2 = t2['b'] if e2 == 'a' else t2['a']
        v1 = (o1[0] - p[0], o1[1] - p[1]); v2 = (o2[0] - p[0], o2[1] - p[1]); l1, l2 = math.hypot(*v1), math.hypot(*v2)
        cw = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / l1 / l2))
        if abs(math.degrees(math.acos(cw)) - 90) > 1: continue
        P = Point(p)
        if any(g.covers(P) for g in pads[(net, L)]): stat['in pad'] += 1; continue
        if any(math.dist((x, y), p) <= r + 1e-6 for x, y, r, Ls in vias[net] if not Ls or L in Ls): stat['in the via'] += 1; continue
        if any(g.buffer(-w / 2).covers(P) for g in zone_list[(net, L)]): stat['in own zone'] += 1; continue
        corners.append(dict(net=net, L=L, w=w, p=p, t1=t1, e1=e1, t2=t2, e2=e2, u1=(v1[0] / l1, v1[1] / l1), u2=(v2[0] / l2, v2[1] / l2),
                          l1=l1, l2=l2))

    per_track = collections.Counter()
    for e in corners: per_track[e['t1']['uuid']] += 1; per_track[e['t2']['uuid']] += 1
    def limit(l, n): return 0.45 * l if n == 1 else min(0.45 * l, (l - MICRO) / 2)
    new_end = {}; diag = []; track_list = {}
    for e in corners:
        c0 = min(cmax, limit(e['l1'], per_track[e['t1']['uuid']]), limit(e['l2'], per_track[e['t2']['uuid']]))
        if c0 < cmin: stat['Leg too short'] += 1; continue
        for c in (c0, 0.75 * c0, 0.5 * c0):
            if c < cmin: c = None; break
            a1 = (e['p'][0] + e['u1'][0] * c, e['p'][1] + e['u1'][1] * c); a2 = (e['p'][0] + e['u2'][0] * c, e['p'][1] + e['u2'][1] * c)
            if fwd is None or fwd.vacant(a1, a2, e['w'], e['L'], e['net']) or not_closer(fwd, e, a1, a2): break
        else:
            c = None
        if c is None: stat['Diagonal not free'] += 1; continue
        if c < c0: stat['smaller chamfer'] += 1
        for t, w_, q in ((e['t1'], e['e1'], a1), (e['t2'], e['e2'], a2)):
            track_list[t['uuid']] = t; new_end.setdefault(t['uuid'], {})[w_] = [round(q[0], 6), round(q[1], 6)]
        diag.append(dict(net=e['net'], layer=e['L'], a=[round(a1[0], 6), round(a1[1], 6)], b=[round(a2[0], 6), round(a2[1], 6)], w=e['w']))
        stat['chamfered'] += 1
        if fwd is not None: fwd.extra(e['net'], e['L'], a1, a2, e['w'])
    add = []
    for u in sorted(track_list):
        t = track_list[u]; ne = new_end[u]
        add.append(dict(net=t['net'], layer=t['layer'], a=ne.get('a', t['a']), b=ne.get('b', t['b']), w=t['w']))
    return dict(remove=sorted(track_list), add=add + diag, stat=dict(corners45=dict(stat)))
