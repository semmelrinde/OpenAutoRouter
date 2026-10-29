import json, math, os, subprocess, sys, tempfile
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from analysis import r18 as R18, wobble as WK


def _router_format(H, without=()):
    R = dict(tracks=[dict(net=t['net'], layer=t['layer'], pts=[t['a'], t['b']], w=[t['w']]) for t in H['tracks'] if t['net'] not in without],
             vias=[dict(net=v['net'], x=v['x'], y=v['y']) for v in H['vias'] if v['net'] not in without])
    B = dict(pads=[dict(net=p['net'], x=p.get('x', Polygon(p['poly']).centroid.x), y=p.get('y', Polygon(p['poly']).centroid.y))
                   for p in H['pads'] if p['net'] not in without and len(p.get('poly') or []) >= 3])
    return B, R


def _stairs(H, without=()):
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, 'h.json')
        json.dump(dict(tracks=[t for t in H['tracks'] if t['net'] not in without], vias=[v for v in H['vias'] if v['net'] not in without]), open(f, 'w'))
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stairs.py'), f, 'piece=0.6', 'list_=0'],
                           capture_output=True, text=True)
    z = r.stdout.split('\n')[0].replace(':', ' ').replace(',', ' ').split()
    try: return dict(stairs=int(z[1]), net_list=int(z[3]), bends=int(z[5]))
    except (IndexError, ValueError): return dict(faults=r.stdout[-200:] + r.stderr[-200:])


def _targets(H, without=()):
    without = set(without); out = []
    layers_net = defaultdict(set)
    for t in H['tracks']: layers_net[t['net']].add(t['layer'])
    for v in H['vias']:
        if v['net'] in without: continue
        c = (round(v['x'], 6), round(v['y'], 6)); g = Point(v['x'], v['y']).buffer(v['d'] / 2, 32)
        for L in (v.get('layers') or sorted(layers_net[v['net']])): out.append((v['net'], L, 'via', c, g))
    for p in H['pads']:
        if p['net'] in without or p.get('type_') not in ('smd', 'conn') or 'x' not in p or len(p.get('poly') or []) < 3: continue
        g = Polygon(p['poly']).buffer(0)
        if g.contains(Point(p['x'], p['y'])): out.append((p['net'], p['layer'], 'pad', (round(p['x'], 6), round(p['y'], 6)), g))
    return out


def mid(H, without=(), eps=0.005):
    without = set(without); axes = defaultdict(list); out = set()
    for t in H['tracks']:
        if t['net'] not in without: axes[(t['net'], t['layer'])].append(t)
    def reached(n, L, c):
        for t in axes.get((n, L), ()):
            a, b = t['a'], t['b']
            if min(a[0], b[0]) - eps <= c[0] <= max(a[0], b[0]) + eps and min(a[1], b[1]) - eps <= c[1] <= max(a[1], b[1]) + eps:
                if (LineString([a, b]) if a != b else Point(a)).distance(Point(c)) <= eps: return True
        return False
    for n, L, category, c, _ in _targets(H, without):
        if reached(n, L, c): out.add((n, L, category, c))
    return out


def middle_loss(H0, H1, without=(), eps=0.005):
    way = mid(H0, without) - mid(H1, without)
    if not way: return {}
    geo = {(n, L, category, c): g for n, L, category, c, g in _targets(H1, without)}
    centers = defaultdict(set)
    for n, L, category, c in geo: centers[(n, L)].add(c)
    ends = defaultdict(list)
    for t in H1['tracks']:
        for q in (t['a'], t['b']): ends[(t['net'], t['layer'])].append((tuple(q), t['w']))
    out = defaultdict(int)
    for k in sorted(way):
        n, L, category, c = k; g = geo.get(k)
        if g is None: continue
        for q, w in ends.get((n, L), ()):
            if g.distance(Point(q)) <= w / 2 + 1e-9 and all(math.dist(q, z) > eps for z in centers[(n, L)]):
                out[n] += 1; break
    return dict(sorted(out.items()))


def ends_error(H, fill=None, without=(), net_list=None, close=0.05):
    without = set(without); net_list = None if net_list is None else set(net_list)
    Q = lambda p: (round(p[0] * 1000), round(p[1] * 1000))
    T = defaultdict(list); seen = set()
    for t in H['tracks']:
        if t['net'] in without or (net_list is not None and t['net'] not in net_list): continue
        k = (t['net'], t['layer']) + tuple(sorted([Q(t['a']), Q(t['b'])]))
        if k in seen: continue
        seen.add(k); T[(t['net'], t['layer'])].append(t)
    vias = defaultdict(list)
    for v in H['vias']: vias[v['net']].append(v)
    pads = defaultdict(list)
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    zone_list = defaultdict(list)
    for f in fill or ():
        zone_list[(f['net'], f['layer'])].append(f['poly'])
    out = {}
    for (n, L), ts in sorted(T.items()):
        nodes = defaultdict(list)
        for t in ts:
            for e in ('a', 'b'): nodes[Q(t[e])].append(t)
        for k, an in sorted(nodes.items()):
            if len(an) != 1: continue
            x, y = k[0] / 1000.0, k[1] / 1000.0; q = Point(x, y)
            if any(math.dist((v['x'], v['y']), (x, y)) <= v['d'] / 2 + 1e-4 for v in vias.get(n, ()) if not v.get('layers') or L in v['layers']):
                continue
            if any(g.distance(q) <= 1e-6 for g in pads.get((n, L), ())): continue
            t0 = an[0]; on_axis = False
            for t in ts:
                if t is t0 or t.get('arc'): continue
                a, b = t['a'], t['b']; dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
                if ll < 1e-12: continue
                s = ((x - a[0]) * dx + (y - a[1]) * dy) / ll
                if 0.0 < s < 1.0 and abs((x - a[0]) * dy - (y - a[1]) * dx) / math.sqrt(ll) <= 1e-3: on_axis = True; break
            if on_axis: continue
            nearest = min((math.dist((x, y), (k2[0] / 1000.0, k2[1] / 1000.0)) for k2 in nodes if k2 != k), default=9.0)
            if nearest < close: out[(n, L, k)] = 'offset'
            elif any(g.distance(q) <= 1e-6 for g in zone_list.get((n, L), ())): continue
            else: out[(n, L, k)] = 'loose'
    return out


def new_places(prior, new, tol=0.3):
    g = defaultdict(list)
    for k in prior: g[(k[0], k[1])].append(k[2])
    vacant = {key: list(v) for key, v in g.items()}
    out = []
    for k in sorted(new):
        cand = vacant.get((k[0], k[1]), [])
        best = min(cand, key=lambda q: math.dist(q, k[2]), default=None)
        if best is not None and math.dist(best, k[2]) <= tol * 1000:
            cand.remove(best); continue
        out.append(k)
    return out


def ends_new(H0, H1, fill0=None, fill1=None, net_list=None, tol=0.3):
    out = defaultdict(int)
    for k in new_places(ends_error(H0, fill0, net_list=net_list), ends_error(H1, fill1, net_list=net_list), tol): out[k[0]] += 1
    return dict(sorted(out.items()))


def corners90_new(H0, H1, net_list=None, tol=0.3):
    out = defaultdict(int)
    for k in new_places(corners90_places(H0, net_list=net_list), corners90_places(H1, net_list=net_list), tol): out[(k[0], k[1])] += 1
    return dict(out)


def goal_leave(H0, H1, net_list=None):
    net_list = None if net_list is None else set(net_list)
    def targets(H):
        out = []
        for v in H['vias']:
            if net_list is not None and v['net'] not in net_list: continue
            for L in (v.get('layers') or sorted({t['layer'] for t in H['tracks'] if t['net'] == v['net']})):
                out.append((v['net'], L, ('V', round(v['x'], 6), round(v['y'], 6)), Point(v['x'], v['y']).buffer(v['d'] / 2, 32)))
        for p in H['pads']:
            if (net_list is not None and p['net'] not in net_list) or len(p.get('poly') or []) < 3: continue
            out.append((p['net'], p['layer'], ('P', p.get('ref'), p.get('num'), round(p.get('x', 0), 6), round(p.get('y', 0), 6)),
                        Polygon(p['poly']).buffer(0)))
        return out
    def state(H):
        T = defaultdict(list)
        for t in H['tracks']:
            if net_list is None or t['net'] in net_list: T[(t['net'], t['layer'])].append(t)
        inside, touched = set(), set()
        for n, L, metric, g in targets(H):
            ts = T.get((n, L), ())


            if any(g.distance(Point(t[e])) <= 1e-6 for t in ts for e in ('a', 'b')) or \
                    any(t['a'] != t['b'] and LineString([t['a'], t['b']]).intersects(g) for t in ts): inside.add((n, L, metric))
            elif any((LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])).distance(g) <= t['w'] / 2 + 1e-6 for t in ts):
                touched.add((n, L, metric))
        return inside, touched
    d0, _ = state(H0); d1, b1 = state(H1)
    out = defaultdict(int)
    for k in sorted((d0 - d1) & b1): out[k[0]] += 1
    return dict(sorted(out.items()))


def corners90(H, without=(), net_list=None):
    out = defaultdict(int)
    for k in corners90_places(H, without, net_list): out[(k[0], k[1])] += 1
    return dict(out)


def corners90_places(H, without=(), net_list=None):
    without = set(without); net_list = None if net_list is None else set(net_list)
    Q = lambda p: (round(p[0] * 1000), round(p[1] * 1000))
    kn = defaultdict(dict)
    for t in H['tracks']:
        if t['net'] in without or (net_list is not None and t['net'] not in net_list) or t.get('arc'): continue
        a, b = Q(t['a']), Q(t['b'])
        if a == b: continue
        k = tuple(sorted([a, b]))
        kn[(t['net'], t['layer'], a)][k] = t; kn[(t['net'], t['layer'], b)][k] = t
    vias = defaultdict(list); pads = defaultdict(list)
    for v in H['vias']: vias[v['net']].append(v)
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    out = set()
    for (n, L, q), an in kn.items():
        if len(an) != 2: continue
        x, y = q[0] / 1000.0, q[1] / 1000.0
        r = []
        for t in an.values():
            o = t['b'] if Q(t['a']) == q else t['a']; d = math.dist((x, y), o)
            r.append(((o[0] - x) / d, (o[1] - y) / d))
        c = max(-1.0, min(1.0, r[0][0] * r[1][0] + r[0][1] * r[1][1]))
        if abs(math.degrees(math.acos(c)) - 90.0) > 0.5: continue
        if any(math.dist((v['x'], v['y']), (x, y)) <= v['d'] / 2 + 1e-4 for v in vias.get(n, ()) if not v.get('layers') or L in v['layers']):
            continue
        if any(g.distance(Point(x, y)) <= 1e-6 for g in pads.get((n, L), ())): continue
        out.add((n, L, q))
    return out


def length_list(H):
    out = defaultdict(float)
    for t in H['tracks']: out[t['net']] += math.dist(t['a'], t['b'])
    return dict(out)


def lengths_violation(L0, H1, net_list, percent=2.0, mm=1.0):
    L1 = length_list(H1); out = {}
    for n in sorted(net_list):
        a, b = L0.get(n, 0.0), L1.get(n, 0.0)
        if abs(a - b) > min(mm, percent / 100.0 * a) + 1e-9: out[n] = (round(a, 4), round(b, 4))
    return out


def gate_per_net(H, without=()):
    net_list = sorted({t['net'] for t in H['tracks'] if t['net'] not in set(without)})
    out = {}
    for n in net_list:
        Hn = dict(H, tracks=[t for t in H['tracks'] if t['net'] == n], vias=[v for v in H['vias'] if v['net'] == n],
                  pads=[p for p in H['pads'] if p['net'] == n])
        t = gate_check(Hn)
        if any(t.values()): out[n] = (t['angle'], t['r32'], t['hairpins'])
    return out


def gate_check(H, without=()):
    without = set(without); angle = 0; nb = defaultdict(list)
    K = lambda p: (round(p[0], 6), round(p[1], 6))
    for t in H['tracks']:
        if t['net'] in without: continue
        a, b = tuple(t['a']), tuple(t['b'])
        if not t.get('arc') and math.dist(a, b) > 0.02:
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
            angle += min(ang, 45 - ang) > 0.5
        for q in (a, b): nb[(t['net'], t['layer'], K(q))].append(t['w'])
    goal = defaultdict(list)
    for v in H['vias']: goal[v['net']].append(Point(v['x'], v['y']).buffer(v['d'] / 2, 16))
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: goal[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    r32 = 0
    for (n, L, q), ws in nb.items():
        if len(ws) == 2 and abs(ws[0] - ws[1]) > 1e-6 and not any(g.distance(Point(q)) <= 1e-9 for g in goal.get(n, []) + goal.get((n, L), [])):
            r32 += 1
    B, R = _router_format(H, without)
    return dict(angle=angle, r32=r32, hairpins=R18.measure(B, R)['all_items']['hairpins'])


def measure(H, without=()):
    without = set(without)
    T = [t for t in H['tracks'] if t['net'] not in without]
    vias = defaultdict(list)
    for v in H['vias']:
        if v['net'] not in without: vias[v['net']].append((v['x'], v['y'], v['d'] / 2, set(v.get('layers') or [])))
    pads = defaultdict(list)
    for p in H['pads']:
        if p['net'] in without or p.get('type_') not in ('smd', 'conn') or 'x' not in p or len(p['poly']) < 3: continue
        g = Polygon(p['poly']).buffer(0)
        if g.contains(Point(p['x'], p['y'])): pads[(p['net'], p['layer'])].append((g, (p['x'], p['y'])))
    deg = defaultdict(int)
    for t in T:
        for q in (t['a'], t['b']): deg[(t['net'], t['layer'], round(q[0], 6), round(q[1], 6))] += 1
    def in_(t, q):
        tol = t['w'] / 2 if deg[(t['net'], t['layer'], round(q[0], 6), round(q[1], 6))] == 1 else 0.0
        for x, y, r, Ls in vias.get(t['net'], []):
            if (not Ls or t['layer'] in Ls) and math.dist(q, (x, y)) <= r + tol: return ('via', (x, y), r)
        for g, c in pads.get((t['net'], t['layer']), []):
            if g.distance(Point(q)) <= tol + 1e-9: return ('pad', c, g)
        return None
    micro = dict(under_0_08=0, under_0_03=0, visible_under_0_08=0); r37 = defaultdict(int); angle = 0; L = 0.0
    for t in T:
        a, b = tuple(t['a']), tuple(t['b']); l = math.dist(a, b); L += l
        za, zb = in_(t, a), in_(t, b)
        whole_inner = za is not None and zb is not None and za[1] == zb[1]
        if l < 0.08 and not t.get('arc'):
            micro['under_0_08'] += 1; micro['under_0_03'] += l < 0.03; micro['visible_under_0_08'] += not whole_inner
        if not t.get('arc') and l > 0.02:
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
            angle += min(ang, 45 - ang) > 0.5
        if whole_inner: continue
        for q, z in ((a, za), (b, zb)):
            if z is None: continue
            d = math.dist(q, z[1])
            if d > 0.005: r37['%s_beside_middle' % z[0]] += 1
            if d > 0.05: r37['%s_over_0_05' % z[0]] += 1
    B, R = _router_format(H, without)
    w = WK.measure(B, R); r = R18.measure(B, R)['all_items']
    return dict(length_mm=round(L, 2), segments=len(T), vias=sum(len(v) for v in vias.values()),
                wobble=dict(move_records=w['move_records'], moves_per_10mm=w['moves_per_10mm'], detour=w['detour']),
                r18=dict(bends=r['bends'], hairpins=r['hairpins'], zigzag=r['zigzag'], bend_excess=r['bend_excess'],
                         detour_octi_mm=round(r['detour_octi_mm'], 2), bends_per_10mm=round(r['bends_per_10mm'], 3)),
                stairs=_stairs(H, without), micro=micro, r37=dict(sorted(r37.items())), angle_outside=angle,
                r32_width_change=gate_check(H, without)['r32'])


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[1:] if '=' in x)
    H = json.load(open(a[0]))
    without = [n for n in o.get('without', '').split(';') if n]
    if o.get('without_json'): without += list(json.load(open(o['without_json'])))
    out = dict(all_items=measure(H))
    if without: out['without_blocked'] = measure(H, without)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    if o.get('json'): json.dump(out, open(o['json'], 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
