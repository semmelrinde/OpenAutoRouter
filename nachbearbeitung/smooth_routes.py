""
import json, sys, math
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree
from shapely.ops import unary_union

B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); OUT = sys.argv[3]
args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
RMAX = float(args.get('rmax', '3')); DRC = float(args.get('drc', '0.09')) + 0.01; DRCP = float(args.get('drcpad', '0.10')) + 0.01
VR = float(args.get('via', '0.45')) / 2


items = {}
def add(L, net, g, clr): items.setdefault(L, []).append((net, g, clr))
for t in R['tracks']:
    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
        add(t['layer'], t['net'], LineString([a, b]).buffer(w / 2, 16) if a != b else Point(a).buffer(w / 2, 16), DRC)
LAYERS = sorted({t['layer'] for t in R['tracks']} | {'F', 'B'})
for v in R['vias']:
    for L in LAYERS: add(L, v['net'], Point(v['x'], v['y']).buffer(VR, 16), DRC)
for p in B['pads']:
    for L, qs in p['polys'].items():
        for q in qs: add(L, p['net'] if p['net'] and not p['net'].startswith('unconnected') else '#%s%s' % (p['ref'], p['num']), Polygon(q), DRCP)
LOCH = float(args.get('loch', '0.2')) + 0.01
for p in B['pads']:
    dr = p.get('drill') or 0
    dr = max(dr) if isinstance(dr, (list, tuple)) else dr
    if dr and not p['net']:
        for L in LAYERS: add(L, '#NPTH%s%s' % (p['ref'], p['num']), Point(p['x'], p['y']).buffer(dr / 2, 16), LOCH)
TREE = {L: STRtree([g for _, g, _ in v]) for L, v in items.items()}


attach = {}
for t in R['tracks']:
    for q in (t['pts'][0], t['pts'][-1]): attach.setdefault((t['net'], t['layer']), []).append(tuple(q))
for v in R['vias']:
    for L in LAYERS: attach.setdefault((v['net'], L), []).append((v['x'], v['y']))

NEW = {}


def clear_ok(L, net, geom, own_idx):
    for j in TREE[L].query(geom.buffer(max(DRC, DRCP, LOCH))):
        n2, g2, clr = items[L][int(j)]
        if n2 == net: continue
        if geom.distance(g2) < clr: return False
    bx = geom.bounds
    for n2, g2 in NEW.get(L, []):
        if n2 == net: continue
        b2 = g2.bounds
        if b2[0] > bx[2] + DRC or b2[2] < bx[0] - DRC or b2[1] > bx[3] + DRC or b2[3] < bx[1] - DRC: continue
        if geom.distance(g2) < DRC: return False
    return True


def fillet(p0, p1, p2, R):
    ""
    v1 = (p0[0] - p1[0], p0[1] - p1[1]); v2 = (p2[0] - p1[0], p2[1] - p1[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-6 or l2 < 1e-6: return None
    u1 = (v1[0] / l1, v1[1] / l1); u2 = (v2[0] / l2, v2[1] / l2)
    cosang = max(-1, min(1, u1[0] * u2[0] + u1[1] * u2[1])); ang = math.acos(cosang)
    if ang > math.radians(178): return None
    d = R / math.tan(ang / 2)
    t1 = (p1[0] + u1[0] * d, p1[1] + u1[1] * d); t2 = (p1[0] + u2[0] * d, p1[1] + u2[1] * d)
    bis = (u1[0] + u2[0], u1[1] + u2[1]); bl = math.hypot(*bis)
    bis = (bis[0] / bl, bis[1] / bl)
    dc = R / math.sin(ang / 2)
    c = (p1[0] + bis[0] * dc, p1[1] + bis[1] * dc)
    mid = (c[0] - bis[0] * R, c[1] - bis[1] * R)
    return t1, mid, t2, d, c


def arc_pts(t1, mid, t2, c, R, n=12):
    a1 = math.atan2(t1[1] - c[1], t1[0] - c[0]); a2 = math.atan2(t2[1] - c[1], t2[0] - c[0]); am = math.atan2(mid[1] - c[1], mid[0] - c[0])
    def norm(a): return (a + math.pi) % (2 * math.pi) - math.pi
    da = norm(a2 - a1)
    if abs(norm(a1 + da / 2 - am)) > 1e-3: da = da - math.copysign(2 * math.pi, da)
    return [(c[0] + R * math.cos(a1 + da * k / n), c[1] + R * math.sin(a1 + da * k / n)) for k in range(n + 1)]


out_tracks = []; narc = 0; nskip = 0
for t in R['tracks']:
    P = [tuple(q) for q in t['pts']]; W = list(t['w']); L = t['layer']; net = t['net']
    if len(P) < 3:
        out_tracks.append(dict(t, items=[('L', P[0], P[-1], W[0])])); continue
    att = [a for a in attach.get((net, L), []) if a not in (P[0], P[-1])]

    cuts = {}
    for i in range(1, len(P) - 1):
        lprev = math.dist(P[i - 1], P[i]) * (1.0 if i - 1 == 0 else 0.5)
        lnext = math.dist(P[i], P[i + 1]) * (1.0 if i + 1 == len(P) - 1 else 0.5)

        for a in att:
            for q0, q1, lim in ((P[i], P[i - 1], 'p'), (P[i], P[i + 1], 'n')):
                seg = LineString([q0, q1])
                if seg.distance(Point(a)) < 0.03:
                    dd = math.dist(q0, a) - 0.05
                    if lim == 'p': lprev = min(lprev, max(0.0, dd))
                    else: lnext = min(lnext, max(0.0, dd))
        f = fillet(P[i - 1], P[i], P[i + 1], 1.0)
        if f is None: continue
        dmax = min(lprev, lnext) - 1e-3
        if dmax <= 0.01: nskip += 1; continue
        ratio = f[3] / 1.0
        Rhi = min(RMAX, dmax / ratio); Rlo = 0.0
        w = min(W[i - 1], W[i])
        best = None
        for _ in range(14):
            Rm = (Rlo + Rhi) / 2 if best is not None or Rhi < RMAX else Rhi
            fr = fillet(P[i - 1], P[i], P[i + 1], Rm)
            ap = arc_pts(fr[0], fr[1], fr[2], fr[4], Rm)
            g = LineString(ap).buffer(w / 2, 16)
            if clear_ok(L, net, g, None): best = (fr, Rm); Rlo = Rm
            else: Rhi = Rm
            if Rhi - Rlo < 0.01: break
        if best and best[1] > 0.02:
            cuts[i] = best
            fr, Rm = best
            NEW.setdefault(L, []).append((net, LineString(arc_pts(fr[0], fr[1], fr[2], fr[4], Rm)).buffer(w / 2, 16)))
        else: nskip += 1

    its = []; cur = P[0]
    for i in range(1, len(P)):
        if i in cuts:
            (t1, mid, t2, d, c), Rr = cuts[i]
            if math.dist(cur, t1) > 1e-4: its.append(('L', cur, t1, W[i - 1]))
            its.append(('A', t1, mid, t2, min(W[i - 1], W[i]), c, Rr)); narc += 1
            cur = t2
        else:
            if math.dist(cur, P[i]) > 1e-4: its.append(('L', cur, P[i], W[i - 1]))
            cur = P[i]
    dense, dw = [its[0][1]], []
    for it in its:
        if it[0] == 'L': dense.append(it[2]); dw.append(it[3])
        else:
            ap = arc_pts(it[1], it[2], it[3], it[5], it[6])
            for q in ap[1:]: dense.append(q); dw.append(it[4])
    out_tracks.append(dict(net=net, layer=L, pts=[list(q) for q in dense], w=dw, sig=t.get('sig', False),
                           items=[[k[0]] + [list(x) if isinstance(x, tuple) else x for x in k[1:5]] for k in its]))
R2 = dict(R); R2['tracks'] = out_tracks
json.dump(R2, open(OUT, 'w'))
print('Boegen', narc, 'Ecken ohne Bogen', nskip, '->', OUT)
