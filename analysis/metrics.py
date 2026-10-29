import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.width_profile import count as _widths_count


EXACT = 0.001
CLOSE = 0.1


def _dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def joints(R, B=None):
    per = {}
    for ti, t in enumerate(R['tracks']):
        for a, b in zip(t['pts'][:-1], t['pts'][1:]): per.setdefault((t['net'], t['layer']), []).append((ti, a, b))
    vias = {}
    for v in R['vias']: vias.setdefault(v['net'], []).append((v['x'], v['y']))
    ends = exact = fast = 0; sites = []
    for ti, t in enumerate(R['tracks']):
        if len(t['pts']) < 2: continue
        for e in (t['pts'][0], t['pts'][-1]):
            ends += 1
            d = [_dist(e, a, b) for tj, a, b in per.get((t['net'], t['layer']), []) if tj != ti
                 and abs((a[0] + b[0]) / 2 - e[0]) <= abs(b[0] - a[0]) / 2 + CLOSE and abs((a[1] + b[1]) / 2 - e[1]) <= abs(b[1] - a[1]) / 2 + CLOSE]
            d += [math.hypot(e[0] - x, e[1] - y) for x, y in vias.get(t['net'], []) if abs(e[0] - x) <= CLOSE and abs(e[1] - y) <= CLOSE]
            if not d: continue
            m = min(d)
            if m <= EXACT: exact += 1
            elif m < CLOSE: fast += 1; sites.append(dict(net_name=t['net'], layer_name=t['layer'], x=e[0], y=e[1], spacing=round(m, 4)))
    vfast = 0; vsites = []
    for v in R['vias']:
        for (n_, L), segs in per.items():
            if n_ != v['net']: continue
            e = (v['x'], v['y'])
            d = [_dist(e, a, b) for _, a, b in segs
                 if abs((a[0] + b[0]) / 2 - e[0]) <= abs(b[0] - a[0]) / 2 + CLOSE and abs((a[1] + b[1]) / 2 - e[1]) <= abs(b[1] - a[1]) / 2 + CLOSE]
            if d and EXACT < min(d) < CLOSE: vfast += 1; vsites.append(dict(net_name=v['net'], layer_name=L, x=v['x'], y=v['y'], spacing=round(min(d), 4)))
    out = dict(ends=ends, ends_exact=exact, fast_joints=fast, fast_joints_sites=sorted(sites, key=lambda s: -s['spacing'])[:40],
               vias_fast=vfast, vias_fast_sites=sorted(vsites, key=lambda s: -s['spacing'])[:40])
    if B is not None:
        from shapely.geometry import Point
        from common.flow import pad_geo
        pads = {}
        for p in B['pads']:
            if p.get('net'): pads.setdefault(p['net'], []).append((p, pad_geo(p)))
        in_ = middle = hole_size = 0; beside = []
        for t in R['tracks']:
            if len(t['pts']) < 2: continue
            for e in (t['pts'][0], t['pts'][-1]):
                tr = [(math.hypot(e[0] - p['x'], e[1] - p['y']), bool(p.get('drill')), p) for p, g in pads.get(t['net'], [])
                      if t['layer'] in g and p['bbox'][0] - 0.03 <= e[0] <= p['bbox'][2] + 0.03 and p['bbox'][1] - 0.03 <= e[1] <= p['bbox'][3] + 0.03
                      and g[t['layer']].distance(Point(e)) <= 0.03]
                if not tr: continue
                d, dr, p = min(tr, key=lambda x: x[:2])
                in_ += 1; hole_size += dr
                if d <= EXACT: middle += 1
                elif not dr: beside.append(dict(net_name=t['net'], layer_name=t['layer'], pad='%s:%s' % (p['ref'], p['num']), x=e[0], y=e[1], spacing=round(d, 4)))
        out.update(ends_in_pad=in_, ends_pad_center=middle, ends_in_pad_hole=hole_size, share_pad_center=round(100.0 * middle / in_, 1) if in_ else 0.0,
                   ends_beside_pad_center=sorted(beside, key=lambda s: -s['spacing'])[:40])
    return out


def metrics(R, B=None):
    seg = short = 0; L = oct_ = 0.0
    bw = bw1 = bw3 = hgt = 0
    nodes = {}
    for t in R['tracks']:
        _c, _h = _widths_count([math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:])], t['w'])
        bw += _c; bw1 += _c >= 1; bw3 += _c >= 3; hgt += _h
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            seg += 1; L += d; short += d < 0.1
            w = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            r = (w + 360.0) % 45.0
            if r < 0.5 or r > 44.5: oct_ += d
            nodes.setdefault((t['net'], t['layer'], round(a[0], 3), round(a[1], 3)), []).append(w)
            nodes.setdefault((t['net'], t['layer'], round(b[0], 3), round(b[1], 3)), []).append(w + 180.0)


    kn = sum(1 for w in nodes.values() if len(w) == 2 and abs((w[0] - w[1]) % 360.0 - 180.0) > 1.0)
    fl = set(R.get('area_nets') or [])
    if B is not None:
        from common.length import outer
        La = sum(sum(s) for s in outer(R, B))
    else: La = L
    return dict(joints(R, B), length_outer=round(La, 1), length_in_pad=round(L - La, 1), track_list=len(R['tracks']), segments=seg, bends=kn, mini=short, total_length=round(L, 1), share_45=round(100 * oct_ / L, 1) if L else 0.0,
                vias=len(R['vias']), signal_vias=sum(1 for v in R['vias'] if v['net'] not in fl and not v.get('current_via')),
                conflict_list=len(R.get('conflicts') or {}), open_items=len(R.get('failed') or []),
                width_change=bw, tracks_change=bw1, tracks_change3=bw3, humps=hgt)

if __name__ == '__main__':
    files = [a for a in sys.argv[1:] if '=' not in a]
    _b = [a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('board=')]
    if not _b and len(files) == 2 and 'pads' in json.load(open(files[1])): _b = [files.pop()]
    B = json.load(open(_b[0])) if _b else None
    for f in files:
        k = metrics(json.load(open(f)), B)
        if 'json=1' in sys.argv: print(json.dumps(dict(file=f, **k)))
        else: print('%-30s Tracks %4d  Segments %5d  Bends %5d  Mini %4d  Length %7.1f mm  0/45/90 %5.1f %%  Vias %4d (Signal %4d)  Conflicts %d  open %d'
                    '  Width changes %4d (tracks %3d, from 3: %3d)  Spurs %3d' % (
            f.split('/')[-1][:30], k['track_list'], k['segments'], k['bends'], k['mini'], k['total_length'], k['share_45'], k['vias'], k['signal_vias'], k['conflict_list'], k['open_items'],
            k['width_change'], k['tracks_change'], k['tracks_change3'], k['humps'])
            + '  Track ends %4d (exactly on own net %4d, near misses %3d, vias next to own track %3d)' % (k['ends'], k['ends_exact'], k['fast_joints'], k['vias_fast'])
            + ('  in pad %4d, at pad center %4d (%5.1f %%), in through-hole pads %3d' % (k['ends_in_pad'], k['ends_pad_center'], k['share_pad_center'], k['ends_in_pad_hole']) if B else '')
            + ('  Length outside own pads %7.1f mm (in pad %5.1f)' % (k['length_outer'], k['length_in_pad']) if B else ''))
