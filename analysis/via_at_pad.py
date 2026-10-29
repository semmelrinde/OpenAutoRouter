import heapq, json, math, sys
import shapely
from shapely.geometry import Point, Polygon


def _net_class(ref, NC):
    t = (NC or {}).get('part_list', {}).get(ref) or {}
    if t.get('decoupling'): return 'decoupling'
    if t.get('role') == 'chip': return 'chip'
    return 'rest'


def evaluate(D, R, NC=None, via_d=None):
    fl = set(R.get('area_nets') or (NC or {}).get('gnd') or [])
    opt = R.get('options') or {}
    if via_d is None: via_d = float(str(opt.get('via', '0.45/0.20')).split('/')[0])
    clr = float(opt.get('clr', 0.13))
    gnd_layers = set(R.get('gnd_layers') or [])
    vr = via_d / 2; circle_a = Point(0, 0).buffer(vr, 32).area
    pads = []
    for p in D['pads']:
        if p['net'] not in fl or any(L in gnd_layers for L in p['layers']): continue
        L = p['layers'][0]
        pads.append(dict(p=p, layer_name=L, poly=shapely.union_all([Polygon(q) for q in p['polys'][L] if len(q) >= 3])))
    lines = []
    for net in sorted(fl):
        vias = [(v['x'], v['y']) for v in R['vias'] if v['net'] == net]
        trs = [t for t in R['tracks'] if t['net'] == net]
        kn = {}
        pos = []; adj = []
        def nodes(L, x, y):
            k = (L, round(x, 2), round(y, 2))
            if k not in kn: kn[k] = len(pos); pos.append((L, x, y)); adj.append([])
            return kn[k]
        for t in trs:
            ids = [nodes(t['layer'], x, y) for x, y in t['pts']]
            for a, b, pa, pb in zip(ids[:-1], ids[1:], t['pts'][:-1], t['pts'][1:]):
                d = math.dist(pa, pb); adj[a].append((b, d)); adj[b].append((a, d))
        via_on = {}
        for i, (L, x, y) in enumerate(pos):
            for j, (vx, vy) in enumerate(vias):
                if math.hypot(x - vx, y - vy) <= 0.03: via_on[i] = j; break
        for q in pads:
            p = q['p']
            if p['net'] != net: continue
            g = q['poly']; gb = g.buffer(0.02)
            dist = {}; heap = []
            for i, (L, x, y) in enumerate(pos):
                if L == q['layer_name'] and gb.contains(Point(x, y)): dist[i] = 0.0; heap.append((0.0, i))
            heapq.heapify(heap)
            best = None
            for j, (vx, vy) in enumerate(vias):
                if gb.contains(Point(vx, vy)):
                    c = (0.0, math.hypot(vx - p['x'], vy - p['y']), j)
                    if best is None or c < best: best = c
            finished = set()
            while heap:
                d, i = heapq.heappop(heap)
                if i in finished: continue
                finished.add(i)
                if best is not None and d > best[0]: break
                if i in via_on:
                    vx, vy = vias[via_on[i]]; c = (d, math.hypot(vx - p['x'], vy - p['y']), via_on[i])
                    if best is None or c < best: best = c
                for m, w in adj[i]:
                    if d + w < dist.get(m, 1e18): dist[m] = d + w; heapq.heappush(heap, (d + w, m))
            z = dict(ref=p['ref'], num=p['num'], net_name=net, net_class=_net_class(p['ref'], NC), layer_name=q['layer_name'],
                     pad_mm2=round(g.area, 3))
            if best is None: z.update(via=None)
            else:
                vx, vy = vias[best[2]]

                z.update(via=[vx, vy], stub=round(best[0], 3), spacing=round(best[1], 3),
                         share=round(Point(vx, vy).buffer(vr, 32).intersection(g).area / min(circle_a, g.area), 3))
            lines.append(z)


    def small(p): return min(p['bbox'][2] - p['bbox'][0], p['bbox'][3] - p['bbox'][1]) < 1.0
    smd = {}
    for p in D['pads']:
        if not p.get('drill'): smd.setdefault(p['ref'], []).append(p)
    per = {}
    for q in pads:
        p = q['p']; P = smd.get(p['ref'], [])
        if p.get('drill') or len(P) < 8 or small(p) or len(p['layers']) != 1 or p['layers'][0] not in ('F', 'B'): continue
        xs = [o['x'] for o in P if small(o)]; ys = [o['y'] for o in P if small(o)]
        if len(xs) < 6 or not (min(xs) < p['x'] < max(xs) and min(ys) < p['y'] < max(ys)): continue
        per.setdefault((p['ref'], p['num'], p['net']), []).append(q)
    mp = []
    for (ref, num, net), qs in sorted(per.items()):
        g = shapely.union_all([q['poly'] for q in qs])
        vs = [(v['x'], v['y']) for v in R['vias'] if v['net'] == net and g.buffer(0.02).contains(Point(v['x'], v['y']))]
        dmin = min((math.dist(a, b_) for i, a in enumerate(vs) for b_ in vs[i + 1:]), default=None)
        mp.append(dict(ref=ref, num=num, net_name=net, pad_mm2=round(g.area, 2), fields=len(qs), vias=len(vs),
                       via_spacing_min=None if dmin is None else round(dmin, 3),
                       cutouts_separated=None if dmin is None else bool(dmin >= via_d + 2 * clr + 0.2 - 1e-6)))
    return lines, mp


def merge(lines):
    out = {}
    for nc in ('decoupling', 'chip', 'rest', 'all_items'):
        zs = [z for z in lines if nc == 'all_items' or z['net_class'] == nc]
        with_ = [z for z in zs if z.get('via')]
        if not zs: continue
        ab = sorted(z['spacing'] for z in with_); st = sorted(z['stub'] for z in with_); n = len(with_)
        def med(v): return None if not v else round(v[len(v) // 2] if len(v) % 2 else (v[len(v) // 2 - 1] + v[len(v) // 2]) / 2, 3)
        out[nc] = dict(pads=len(zs), without_via=len(zs) - n,
                       whole_in_pad=sum(1 for z in with_ if z['share'] >= 0.99), from_half=sum(1 for z in with_ if z['share'] >= 0.5 - 1e-6),
                       from_third=sum(1 for z in with_ if z['share'] >= 1 / 3 - 1e-6), beside=sum(1 for z in with_ if z['share'] < 1 / 3 - 1e-6),
                       spacing_median=med(ab), spacing_medium=round(sum(ab) / n, 3) if n else None, spacing_max=ab[-1] if n else None,
                       spacing_over_05=sum(1 for a in ab if a > 0.5), spacing_over_1=sum(1 for a in ab if a > 1.0),
                       stub_medium=round(sum(st) / n, 3) if n else None, stub_max=st[-1] if n else None, stub_total_sum=round(sum(st), 2))
    return out


if __name__ == '__main__':
    if len(sys.argv) < 3: sys.exit(__doc__)
    args = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    D = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2]))
    NC = json.load(open(args['netclasses'])) if 'netclasses' in args else None
    lines, mp = evaluate(D, R, NC, float(args['via']) if 'via' in args else None)
    S = merge(lines)
    print('Vias of the area nets at the pad (R9: in pad = at least a third, better half of the via area)')
    print('%-12s %5s %5s | %5s %7s %8s %8s | %7s %7s %6s %6s %5s | %7s %6s' % (
        'NetClass', 'Pads', 'without', 'whole', '>=1/2', '>=1/3', 'beside', 'Median', 'Medium', 'Max', '>0,5', '>1,0', 'Stub', 'Max'))
    for nc, s in S.items():
        print('%-12s %5d %5d | %5d %7d %8d %8d | %7s %7s %6s %6d %5d | %7s %6s' % (
            nc, s['pads'], s['without_via'], s['whole_in_pad'], s['from_half'], s['from_third'], s['beside'], s['spacing_median'],
            s['spacing_medium'], s['spacing_max'], s['spacing_over_05'], s['spacing_over_1'], s['stub_medium'], s['stub_max']))
    print('   (spacing = pad center to via center in mm, stub = track length from pad to via in mm; >=1/2 contains entirely, >=1/3 contains >=1/2)')
    if mp:
        print('Ground pads under chips (R16; target at least 4, from about 9 mm2 nine):')
        for m in mp:
            print('   %-10s pad %-4s %-8s %6.2f mm2 in %d fields: %d vias, smallest spacing %s mm, cutouts separate: %s' % (
                m['ref'], m['num'], m['net_name'], m['pad_mm2'], m['fields'], m['vias'], m['via_spacing_min'],
                {None: '-', True: 'yes', False: 'no'}[m['cutouts_separated']]))
    if 'json' in args: json.dump(dict(summary=S, ground_pads=mp, pads=lines), open(args['json'], 'w'), indent=1)
