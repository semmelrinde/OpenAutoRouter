import json, math, sys, collections
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

PAR_DEG, PAR_D, PAR_L = 15.0, 0.6, 0.8
SHARP_DEG = 59.0
BORDER_PAD, BORDER_TRUNK = 0.3, 0.6
BESIDE_D = 0.8
ARM_R = 0.15


def _segments(R):
    d = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) > 1e-6: d[(t['net'], t['layer'])].append((ti, tuple(a), tuple(b), t['w'][si] if si < len(t['w']) else t['w'][-1], bool(t.get('sig'))))
    return d


def _pads(B):
    P = collections.defaultdict(list)
    if not B: return P
    for p in B['pads']:
        if not p.get('net'): continue
        for L, polys in p['polys'].items():
            g = shapely.union_all([Polygon(q) for q in polys if len(q) >= 3])
            if not g.is_empty: P[(p['net'], L.replace('.Cu', ''))].append(g)
    return P


def _foot(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
    return (a[0] + t * dx, a[1] + t * dy)


def _dist_point_seg(p, a, b):
    return math.dist(p, _foot(p, a, b))


def _arm(pts, i, step, p, r=ARM_R):
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += step
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0



VERSION = 'topology-v1'
NATIVE_VERSION = 'topology-v4-symmetric'
TRUNK_PREV_VERSION = 'topology-v5-trunk'
TRUNK_V6_VERSION = 'topology-v6-trunk'
TRUNK_V7_VERSION = 'topology-v7-trunk'
TRUNK_VERSION = 'topology-v8-trunk'


def topology(R, B=None, example_max=8, version=VERSION):
    if version in (TRUNK_PREV_VERSION, TRUNK_V6_VERSION, TRUNK_V7_VERSION, TRUNK_VERSION):
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from analysis.topology_native import topology as native
        if version == TRUNK_PREV_VERSION:
            from analysis.topology_trunk_v5 import measure
        elif version == TRUNK_V6_VERSION:
            from analysis.topology_trunk_v6 import measure
        elif version == TRUNK_V7_VERSION:
            from analysis.topology_trunk_v7 import measure
        else:
            from analysis.topology_trunk import measure
        out = native(R, B, example_max)
        trunk = measure(R, B, example_max)
        out.update({k: v for k, v in trunk.items() if k != 'amount'})

        out['examples_nodes'] = out.pop('example_nodes')
        out['examples_parallel'] = out.pop('example_parallel')


        out.pop('branch_middle', None)
        return out
    if version == NATIVE_VERSION:
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from analysis.topology_native import topology as native
        return native(R, B, example_max)
    if version != VERSION:
        raise ValueError('unknown topology version: '+str(version))
    D = _segments(R); PADS = _pads(B)
    clr = float(R.get('clr') or 0.13)
    try: via_d = float(str((R.get('options') or {}).get('via', '0.45/0.20')).split('/')[0])
    except ValueError: via_d = 0.45
    vias = collections.defaultdict(list)
    for v in R['vias']: vias[v['net']].append((v['x'], v['y']))
    par = dict(tap=collections.Counter(), link_chain=collections.Counter(), rest=collections.Counter()); example_par = []
    kn = collections.Counter(); example_kn = []
    tap = []
    via_in_pad = 0; via_counted = set()
    for (net, L), S in sorted(D.items()):

        for i in range(len(S)):
            ti, a, b, wa_, sa = S[i]; la = LineString([a, b]); wa = math.atan2(b[1] - a[1], b[0] - a[0]); puf = None
            for j in range(i + 1, len(S)):
                tj, c, d, wb_, sb = S[j]
                if tj == ti: continue
                if min(c[0], d[0]) - PAR_D > max(a[0], b[0]) or max(c[0], d[0]) + PAR_D < min(a[0], b[0]) or \
                   min(c[1], d[1]) - PAR_D > max(a[1], b[1]) or max(c[1], d[1]) + PAR_D < min(a[1], b[1]): continue
                wb = math.atan2(d[1] - c[1], d[0] - c[0]); dw = abs((wa - wb + math.pi / 2) % math.pi - math.pi / 2)
                if dw > math.radians(PAR_DEG): continue
                if puf is None: puf = la.buffer(PAR_D, cap_style='flat')
                lb = LineString([c, d]); close = puf.intersection(lb).length
                if close > PAR_L and la.distance(lb) > 0.02:
                    category = 'tap' if (sa or sb) else ('link_chain' if (R['tracks'][ti].get('link_chain') or R['tracks'][tj].get('link_chain')) else 'rest')
                    par[category][net] += close
                    example_par.append((round(close, 2), category, net, L, [round(v, 2) for v in a + b], [round(v, 2) for v in c + d], round(la.distance(lb), 3)))

        pk = PADS.get((net, L), [])
        move_records = [[tuple(q) for q in t['pts']] for t in R['tracks'] if t['net'] == net and t['layer'] == L]
        cand = sorted({q for P in move_records for q in P}); finished = []
        for p in cand:
            if any(math.dist(p, q) < 0.05 for q in finished): continue
            raw_data = []; middle = False; ends = 0
            for P in move_records:
                for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
                    if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
                    f = _foot(p, a, b)
                    if math.dist(p, f) > 0.03: continue
                    raw_data += [_arm(P, k + 1, 1, f), _arm(P, k, -1, f)]
                if math.dist(P[0], p) <= 0.03 or math.dist(P[-1], p) <= 0.03: ends += 1
                elif any(_dist_point_seg(p, a, b) <= 0.03 for a, b in zip(P[:-1], P[1:])) and all(math.dist(q, p) > 0.03 for q in P): middle = True
            arms = []
            for x in sorted(x for x in raw_data if x is not None):
                if not arms or (x - arms[-1]) > 3.0: arms.append(x)
            if len(arms) > 1 and (arms[0] + 360.0 - arms[-1]) <= 3.0: arms.pop()
            if len(arms) < 3: continue
            finished.append(p)
            lu = [(arms[(k + 1) % len(arms)] - arms[k]) % 360 for k in range(len(arms))]
            in_pad = any(g.buffer(0.03).contains(Point(p)) for g in pk) if pk else False
            kn['nodes'] += 1; kn['nodes_pad'] += in_pad; kn['branch_middle'] += middle
            if len(arms) > 3: kn['star'] += 1; kn['star_pad'] += in_pad
            if min(lu) < SHARP_DEG: kn['sharp'] += 1; kn['sharp_pad'] += in_pad
            if len(arms) > 3 or min(lu) < SHARP_DEG:
                example_kn.append((len(arms), round(min(lu), 1), net, L, (round(p[0], 2), round(p[1], 2)), 'in pad' if in_pad else 'vacant'))

        sig_ti = sorted({s[0] for s in S if s[4]})
        if not sig_ti: continue
        nsig = [s for s in S if not s[4]]
        sig_ends = [tuple(q) for t in R['tracks'] if t['net'] == net and t.get('sig') for q in (t['pts'][0], t['pts'][-1])]
        mine_vias = []; tap_vias = []
        for v in vias[net]:
            (tap_vias if any(math.dist(v, e) < 0.05 for e in sig_ends) else mine_vias).append(v)
        net_pads = [g for (n2, L2), gs in PADS.items() if n2 == net for g in gs]
        end_pads = [g for g in net_pads if any(g.buffer(0.03).contains(Point(e)) for e in sig_ends)]
        for v in tap_vias:
            if v not in via_counted and any(g.contains(Point(v)) for g in net_pads) and not any(g.contains(Point(v)) for g in end_pads):
                via_counted.add(v); via_in_pad += 1
        copper_map = [LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig] + [Point(v).buffer(via_d / 2) for v in mine_vias]
        for ti in sig_ti:
            t = R['tracks'][ti]; pts = [tuple(q) for q in t['pts']]; anchor = []; category = 'kelvin'; own_pads = []
            for e in (pts[0], pts[-1]):
                hit = [g for g in pk if g.buffer(0.03).contains(Point(e))]
                if hit: anchor += [g.buffer(BORDER_PAD) for g in hit]; own_pads += hit; continue
                if any(math.dist(e, v) < 0.05 for v in tap_vias): continue
                if not t.get('kelvin') and any(_dist_point_seg(e, a, b) < w / 2 + 0.03 for _, a, b, w, _ in nsig):
                    category = 'branch'; anchor.append(Point(e).buffer(BORDER_TRUNK))

            A = shapely.union_all(anchor) if anchor else None
            O = shapely.union_all(copper_map + [g for g in pk if not any(g is h for h in own_pads)]) if (copper_map or pk) else None
            ws = t['w']; part_list = [LineString([a, b]).buffer((ws[i] if i < len(ws) else ws[-1]) / 2) for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])) if math.dist(a, b) > 1e-6]
            if not part_list: continue
            T = shapely.union_all(part_list); lin = LineString(pts)
            if A is not None: T = T.difference(A); lin = lin.difference(A)
            if O is not None and A is not None: O = O.difference(A)
            lg = lin.length if not lin.is_empty else 0.0
            if O is None or O.is_empty or T.is_empty:
                tap.append(dict(net=net, layer_name=L, category=category, spacing=None, close_mm=0.0, location=None, total_length=lg, beside_mm=0.0)); continue
            Ks = shapely.union_all([LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig]) if nsig else None
            if Ks is not None and A is not None: Ks = Ks.difference(A)
            beside = lin.intersection(Ks.buffer(BESIDE_D)).length if (Ks is not None and not Ks.is_empty and not lin.is_empty) else 0.0
            d = T.distance(O); close = lin.intersection(O.buffer(clr + min(ws) / 2 - 1e-6)).length if not lin.is_empty else 0.0
            location = None
            if d < clr:
                q = nearest_points(T, O)[0]; location = (round(q.x, 2), round(q.y, 2))
            tap.append(dict(net=net, layer_name=L, category=category, spacing=round(d, 3), close_mm=round(close, 2), location=location, total_length=lg, beside_mm=beside, flag=bool(t.get('kelvin'))))
    kel = [a for a in tap if a['category'] == 'kelvin' and a['spacing'] is not None]
    kfl = [a for a in kel if a.get('flag')]
    all_items = par['tap'] + par['link_chain'] + par['rest']
    erg = dict(parallel_tap_mm=round(sum(par['tap'].values()), 1), parallel_chain_mm=round(sum(par['link_chain'].values()), 1),
               parallel_rest_mm=round(sum(par['rest'].values()), 1), parallel_nets=dict(sorted((n, round(v, 1)) for n, v in all_items.items())),
               nodes=kn['nodes'], nodes_pad=kn['nodes_pad'], branch_middle=kn['branch_middle'],
               star=kn['star'], star_pad=kn['star_pad'], sharp=kn['sharp'], sharp_pad=kn['sharp_pad'],
               taps=len(tap), taps_kelvin=sum(1 for a in tap if a['category'] == 'kelvin'), taps_branch=sum(1 for a in tap if a['category'] == 'branch'),
               tap_spacing_min=min((a['spacing'] for a in kel), default=None),
               tap_close_mm=round(sum(a['close_mm'] for a in kel), 1), tap_close=sum(1 for a in kel if a['spacing'] < clr - 1e-6),
               tap_length_mm=round(sum(a['total_length'] for a in tap if a['category'] == 'kelvin'), 1),
               tap_beside_current_mm=round(sum(a['beside_mm'] for a in tap if a['category'] == 'kelvin'), 1),
               kelvin_marked=len(kfl), kelvin_spacing_min=min((a['spacing'] for a in kfl), default=None),
               kelvin_close=sum(1 for a in kfl if a['spacing'] < clr - 1e-6), kelvin_close_mm=round(sum(a['close_mm'] for a in kfl), 1),
               kelvin_length_mm=round(sum(a['total_length'] for a in kfl), 1), kelvin_beside_current_mm=round(sum(a['beside_mm'] for a in kfl), 1),
               tap_via_in_pad=via_in_pad, with_board=bool(B), clr=clr)
    erg['examples_parallel'] = sorted(example_par, reverse=True)[:example_max]
    erg['examples_nodes'] = sorted(example_kn, key=lambda k: (-k[0], k[1]))[:example_max]
    erg['examples_tap'] = sorted((a for a in kel if a['spacing'] < clr - 1e-6), key=lambda a: a['spacing'])[:example_max]
    return erg


if __name__ == '__main__':
    opt = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    sharp_limit = 60.0 if opt.get('version') in (NATIVE_VERSION, TRUNK_PREV_VERSION, TRUNK_V6_VERSION, TRUNK_V7_VERSION, TRUNK_VERSION) else SHARP_DEG
    B = json.load(open(opt['board'])) if 'board' in opt else None
    for f in [a for a in sys.argv[1:] if '=' not in a]:
        e = topology(json.load(open(f)), B, version=opt.get('version', VERSION))
        if opt.get('json') == '1': print(json.dumps(dict(file=f, **e))); continue
        print('%-22s parallel in own net: tap %5.1f mm, chain %4.1f mm, rest %5.1f mm | nodes %3d (mid-segment %s, in pad %d), star (>3 arms) %d (in pad %d), '
              'Angle < %g degrees %d (in pad %d)' % (f.split('/')[-1][:22], e['parallel_tap_mm'], e['parallel_chain_mm'], e['parallel_rest_mm'], e['nodes'], e.get('branch_middle', 'n/a'),
                                                  e['nodes_pad'], e['star'], e['star_pad'], sharp_limit, e['sharp'], e['sharp_pad']))
        print('    Taps %d (Kelvin %d, branch off the current path %d): smallest spacing Kelvin tap to own copper %s mm (rule %g), closer than the rule: %d taps over %.1f mm; '
              'Tap vias in intermediate pad %d; Kelvin taps %.1f mm long, of which %.1f mm on the layer of the own current path at less than %g mm spacing%s' % (
                  e['taps'], e['taps_kelvin'], e['taps_branch'], e['tap_spacing_min'], e['clr'], e['tap_close'],
                  e['tap_close_mm'], e['tap_via_in_pad'], e['tap_length_mm'], e['tap_beside_current_mm'], BESIDE_D, '' if B else '  (without board=: pads not taken into account)'))
        if e['kelvin_marked']:
            print('    of which marked as Kelvin by the router %d: smallest spacing %s mm, closer than the rule %d over %.1f mm, %.1f mm long, of which %.1f mm next to the current path' % (
                e['kelvin_marked'], e['kelvin_spacing_min'], e['kelvin_close'], e['kelvin_close_mm'], e['kelvin_length_mm'], e['kelvin_beside_current_mm']))
        print('   ', [(n[-14:], v) for n, v in sorted(e['parallel_nets'].items(), key=lambda kv: -kv[1])[:8]])
        if 'parallel_r27_mm' in e:
            print('    R27 including chains/normal sig branches, only Kelvin excluded: %.1f mm parallel' % e['parallel_r27_mm'])
        if opt.get('details') == '1':
            for b in e['examples_parallel']: print('    parallel', b)
            for b in e['examples_nodes']: print('    Nodes (arms, smallest angle, net, layer, location)', b)
            for b in e['examples_tap']: print('    Tap near', b)
