import json, math, sys
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

QS = 8
EPS = 1e-4


def _via_ground(R):
    try: d, b = [float(x) for x in str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')]
    except ValueError: d, b = 0.45, 0.20
    return d, b


def _octi(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    ax, ay = abs(dx), abs(dy)
    if ax < EPS or ay < EPS or abs(ax - ay) < EPS: return [[a, b]]
    sx, sy = math.copysign(1, dx), math.copysign(1, dy); d = min(ax, ay)
    diag = (a[0] + sx * d, a[1] + sy * d)
    straight = (b[0] - sx * d, b[1] - sy * d)
    return [[a, diag, b], [a, straight, b]]


def _merge(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, out[-1]) < EPS: continue
        if len(out) >= 2:
            a, b = out[-2], out[-1]
            cr = (b[0] - a[0]) * (p[1] - b[1]) - (b[1] - a[1]) * (p[0] - b[0])
            dt = (b[0] - a[0]) * (p[0] - b[0]) + (b[1] - a[1]) * (p[1] - b[1])
            if abs(cr) < 1e-6 and dt > 0: out[-1] = p; continue
        out.append(p)
    return out


class Layer:
    def __init__(self, B, R, L, d_via):
        self.geo, self.net_name = [], []
        for t in R.get('tracks', []):
            if t['layer'] != L: continue
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                self.geo.append((LineString([a, b]) if a != b else Point(a)).buffer(w / 2, QS)); self.net_name.append(t.get('net', ''))
        for v in R.get('vias', []):
            self.geo.append(Point(v['x'], v['y']).buffer(d_via / 2, QS)); self.net_name.append(v.get('net', ''))
        for p in B['pads']:
            for q in (p.get('polys') or {}).get(L, []):
                if len(q) >= 3: self.geo.append(Polygon(q).buffer(0)); self.net_name.append(p['net'])
            if p.get('drill') and not (p.get('polys') or {}).get(L):
                self.geo.append(Point(p['x'], p['y']).buffer(p['drill'] / 2, QS)); self.net_name.append('#drill_hole')
        for n, L2, g in B.get('areas', ()):
            if L2 == L: self.geo.append(g); self.net_name.append(n)
        self.tree_idx = STRtree(self.geo) if self.geo else None

    def vacant(self, g, net_name, spacing):
        if self.tree_idx is None: return True
        for i in self.tree_idx.query(g.buffer(spacing, 4)):
            if self.net_name[i] != net_name and self.geo[i].distance(g) < spacing: return False
        return True


def _bend(a, b, c):
    v1 = (a[0] - b[0], a[1] - b[1]); v2 = (c[0] - b[0], c[1] - b[1]); l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-9 or l2 < 1e-9: return None
    return (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)


def _worse(a, b, c, a0, c0, free_pkt):
    k, k0 = _bend(a, b, c), _bend(a0, b, c0)
    if k is None: return False
    sharp, sharp0 = k > 1e-6, (k0 is not None and k0 > 1e-6)
    right, right0 = abs(k) <= math.sin(math.radians(0.5)), (k0 is not None and abs(k0) <= math.sin(math.radians(0.5)))
    return (sharp and not sharp0) or (free_pkt and right and not right0 and not sharp0)


def _tighten(pts, w, n, lg, inner, keepout, clr, hmax=0.0, before=None, post=None, free_pkt=None, centers=(), goal_geo=()):
    protection = free_pkt is not None; free_pkt = free_pkt or (lambda p: True)
    out = [pts[0]]; i = 0; e = len(pts) - 1
    while i < e:
        located = False
        for j in sorted({e, i + 64, i + 32, i + 16, i + 8, i + 4, i + 2}, reverse=True):
            if j > e or j <= i + 1: continue
            prior = sum(math.dist(pts[k], pts[k + 1]) for k in range(i, j))
            for cand in _octi(pts[i], pts[j]):
                ls = LineString(cand)
                if ls.length > prior * 1.0824 + 1e-6: continue
                if hmax and ls.hausdorff_distance(LineString(pts[i:j + 1])) > hmax: continue
                if protection:
                    prev_ls = LineString(pts[i:j + 1])
                    if any(prev_ls.distance(Point(z)) <= 0.005 and ls.distance(Point(z)) > 0.005 for z in centers): continue
                    if any(prev_ls.intersects(g_) and not ls.intersects(g_) for g_ in goal_geo): continue
                    pa = out[-2] if len(out) >= 2 else before
                    pa0 = pts[i - 1] if i > 0 else before
                    if pa is not None and _worse(pa, cand[0], cand[1], pa0, pts[i + 1], free_pkt(cand[0])): continue
                    pn = pts[j + 1] if j < e else post
                    if pn is not None and _worse(cand[-2], cand[-1], pn, pts[j - 1], pn, free_pkt(cand[-1])): continue
                g = ls.buffer(w / 2, QS)
                if not inner.contains(g): continue
                if keepout is not None and g.intersects(keepout): continue
                if not lg.vacant(g, n, clr): continue
                out.extend(cand[1:]); i = j; located = True; break
            if located: break
        if not located: out.append(pts[i + 1]); i += 1
    return _merge(out)


def smooth(B, R, clr=None, border=None, metadata=True, keepout_list=None, anchor=None, prev_obstacle=False, hmax=None, corners_protection=False):
    keepout_list = set(keepout_list or ()); anchor = anchor or {}; old = []
    clr = (clr if clr is not None else float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1)) + 0.002
    border = border if border is not None else float((R.get('options') or {}).get('edge') or 0.2)
    d_via, _ = _via_ground(R)
    inner = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, QS)
    keepout = [Polygon(q).buffer(0) for k in B.get('keepouts', []) if (k.get('prohibitions') or {}).get('track_list', True)
              for q in k.get('polys', []) if len(q) >= 3]
    keepout = unary_union(keepout) if keepout else None
    rd = lambda p: (round(p[0], 4), round(p[1], 4))
    L0 = sum(math.dist(a, b) for t in R.get('tracks', []) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    via_pkt = defaultdict(set)
    for v in R.get('vias', []): via_pkt[v.get('net', '')].add(rd((v['x'], v['y'])))
    pad_geo = defaultdict(list)
    for p in B['pads']:
        for L, qs in (p.get('polys') or {}).items():
            for q in qs:
                if len(q) >= 3: pad_geo[(p['net'], L)].append(Polygon(q).buffer(0))
    group_list = defaultdict(list)
    for t in R.get('tracks', []): group_list[(t.get('net', ''), t['layer'])].append(t)
    new = {}; changed = 0; chains_n = 0
    for L in sorted({t['layer'] for t in R.get('tracks', [])}):
        lg = Layer(B, R, L, d_via)
        for (n, L2) in sorted(group_list):
            if L2 != L or not n or n in keepout_list: continue
            fixed_extra = anchor.get((n, L), set())
            if metadata and any(t.get('kelvin') or t.get('fix') or 'items' in t for t in group_list[(n, L2)]):



                continue
            seg = []
            for t in group_list[(n, L2)]:
                meta = {k: v for k, v in t.items() if k not in ('net', 'layer', 'pts', 'w', 'items')} if metadata else {}
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                    a, b = rd(a), rd(b)
                    if a != b: seg.append((a, b, w, meta))
            nb = defaultdict(list)
            for k, (a, b, w, meta) in enumerate(seg): nb[a].append(k); nb[b].append(k)
            pads = unary_union(pad_geo.get((n, L), [])) if pad_geo.get((n, L)) else None
            centers = (sorted(via_pkt[n]) + [(p['x'], p['y']) for p in B['pads'] if p['net'] == n and L in (p.get('polys') or {})
                                             and 'x' in p and not str(p['net']).startswith('#')]) if corners_protection else ()
            goal_geo = ([Point(v).buffer(d_via / 2, QS) for v in sorted(via_pkt[n])] + list(pad_geo.get((n, L), []))) if corners_protection else ()
            def pinned(p):
                ks = nb[p]
                if len(ks) != 2 or p in via_pkt[n] or p in fixed_extra: return True
                if abs(seg[ks[0]][2] - seg[ks[1]][2]) > 1e-9: return True
                if seg[ks[0]][3] != seg[ks[1]][3]: return True
                return pads is not None and pads.distance(Point(p)) < 1e-3
            vacant = set(range(len(seg))); chain_list = []
            def job(start, k):
                way = [start]; p = start; w, meta = seg[k][2:]
                while True:
                    vacant.discard(k); a, b, _, _ = seg[k]; q = b if a == p else a; way.append(q); p = q
                    if pinned(p): return way, w, meta
                    nxt = [j for j in nb[p] if j in vacant]
                    if not nxt: return way, w, meta
                    k = nxt[0]
            for p in sorted(nb):
                if pinned(p):
                    for k in list(nb[p]):
                        if k in vacant: chain_list.append(job(p, k))
            while vacant:
                k = min(vacant); vacant.discard(k); a, b, w, meta = seg[k]; chain_list.append(([a, b], w, meta))
            new_t = []; different = False
            for pts, w, meta in chain_list:
                chains_n += 1
                if len(pts) >= 3:
                    def against(p, q0):
                        other = [seg[k][1] if seg[k][0] == p else seg[k][0] for k in nb[p]]
                        other = [x for x in other if x != q0]
                        return other[0] if len(nb[p]) == 2 and len(other) == 1 else None
                    def free_pkt(p):
                        return not (pads is not None and pads.distance(Point(p)) < 1e-3) and \
                            not any(math.dist(p, v) <= d_via / 2 for v in via_pkt[n])
                    q = _tighten(pts, w, n, lg, inner, keepout, clr, (hmax or {}).get((n, L), 0.0),
                                  **(dict(before=against(pts[0], pts[1]), post=against(pts[-1], pts[-2]), free_pkt=free_pkt, centers=centers,
                                          goal_geo=goal_geo)
                                     if corners_protection else {}))
                    if len(q) != len(pts) or any(math.dist(x, y) > 1e-6 for x, y in zip(q, pts)): different = True; changed += 1
                    pts = q
                new_t.append(dict(meta, net=n, layer=L, pts=[[round(x, 5), round(y, 5)] for x, y in pts], w=[w] * (len(pts) - 1)))
            if different:
                new[(n, L)] = new_t

                if prev_obstacle: old += [t for t in R['tracks'] if t.get('net', '') == n and t['layer'] == L]
                R['tracks'] = [t for t in R['tracks'] if not (t.get('net', '') == n and t['layer'] == L)] + new_t
                lg = Layer(B, dict(R, tracks=R['tracks'] + [t for t in old if t['layer'] == L]) if prev_obstacle else R, L, d_via)
    L1 = sum(math.dist(a, b) for t in R.get('tracks', []) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    return dict(chain_list=chains_n, changed=changed, length_before=round(L0, 1), length_after=round(L1, 1))


def implicit_anchor(R, B=None):
    rd = lambda p: (round(p[0], 4), round(p[1], 4))
    anchor = defaultdict(set); group_list = defaultdict(list); new = []
    goal = defaultdict(list)
    for v in R.get('vias', []):
        if v.get('d'): goal[v.get('net', '')].append(Point(v['x'], v['y']).buffer(v['d'] / 2, QS))
    for p in (B or {}).get('pads', []):
        for L, qs in (p.get('polys') or {}).items():
            for q in qs:
                if len(q) >= 3: goal[(p['net'], L)].append(Polygon(q).buffer(0))
    for t in R.get('tracks', []): group_list[(t.get('net', ''), t['layer'])].append(t)
    for key in sorted(group_list):
        ts = group_list[key]
        ends = sorted({(tuple(p), w) for t in ts for p, w in ((t['pts'][0], t['w'][0]), (t['pts'][-1], t['w'][-1]))})
        for t in ts:
            if t.get('fix') or len(t['pts']) != 2: new.append(t); continue
            a, b = tuple(t['pts'][0]), tuple(t['pts'][1]); ln = LineString([a, b]) if a != b else Point(a); cuts = []
            for q, wq in ends:
                if math.dist(q, a) < 1e-6 or math.dist(q, b) < 1e-6: continue
                if ln.distance(Point(q)) > (wq + t['w'][0]) / 2: continue
                if any(g.distance(Point(q)) <= 1e-9 and g.distance(ln) <= 1e-9 for g in goal.get(key[0], []) + goal.get(key, [])): continue
                s_ = ln.project(Point(q)) if a != b else 0.0
                if 1e-3 < s_ < ln.length - 1e-3: cuts.append(round(s_, 5))
                else: anchor[key].add(rd(a if s_ <= 1e-3 else b))
                anchor[key].add(rd(q))
            if not cuts: new.append(t); continue
            ds = [0.0] + sorted(set(cuts)) + [ln.length]
            pts = [tuple(ln.interpolate(d).coords[0]) for d in ds]; pts[0], pts[-1] = a, b
            for p in pts[1:-1]: anchor[key].add(rd(p))
            for p, q in zip(pts, pts[1:]):
                if math.dist(p, q) > 1e-6: new.append(dict(t, pts=[list(p), list(q)], w=[t['w'][0]]))
    return new, dict(anchor)


def via_middle(R):
    d_via, _ = _via_ground(R)
    vias = defaultdict(list)
    for v in R.get('vias', []): vias[v.get('net', '')].append((v['x'], v['y']))
    n = 0
    for t in R.get('tracks', []):
        if len(t['pts']) < 2: continue
        for end in (0, -1):
            p = tuple(t['pts'][end])
            for c in vias.get(t.get('net', ''), []):
                dd = math.dist(p, c)
                if 1e-3 < dd < d_via / 2 - 1e-4:
                    w = t['w'][0] if end == 0 else t['w'][-1]
                    if end == 0: t['pts'].insert(0, [c[0], c[1]]); t['w'].insert(0, w)
                    else: t['pts'].append([c[0], c[1]]); t['w'].append(w)
                    if t.get('items'):
                        if end == 0: t['items'].insert(0, ['L', [c[0], c[1]], list(p), w])
                        else: t['items'].append(['L', list(p), [c[0], c[1]], w])
                    n += 1; break
    return n


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[3:] if '=' in x)
    B, R = json.load(open(a[0])), json.load(open(a[1]))
    S = {}
    if o:
        import os; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from postprocessing.keepout_nets import keepout_nets
        S = keepout_nets(pcb=o.get('pcb'), pro=o.get('pro'), dru=o.get('dru'), netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'),
                       net_list={t.get('net', '') for t in R.get('tracks', [])}, pads=B.get('pads'), net_ties=B.get('net_ties'))
        for n, g in S.items(): print('locked: %s [%s]' % (n, g), file=sys.stderr)
    info = smooth(B, R, keepout_list=S)
    R['smooth45'] = info
    json.dump(R, open(a[2], 'w'))
    print(json.dumps(info))


if __name__ == '__main__':
    main()
