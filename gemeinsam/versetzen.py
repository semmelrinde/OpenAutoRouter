""
import math, sys, os, json
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union, nearest_points
from shapely import affinity
from gemeinsam.teile import lade_teile, platine, Belegung

VERSORGUNG = ('versorgung', 'leistung', 'masse')

VERSATZ = sorted(((i * 0.25, j * 0.25) for i in range(-4, 5) for j in range(-4, 5)), key=lambda e: (round(math.hypot(*e), 4), e))


def netze_von_teil(K, B, ref):
    ""
    nets = sorted({p['net'] for p in B['pads'] if p['ref'] == ref and p['net']})
    if len(nets) != 2: return None
    gnd = set(K.get('gnd') or [])
    sup = [n for n in nets if n in gnd or (K['netze'].get(n) or {}).get('art') in VERSORGUNG]
    if len(sup) == 1: return (nets[0] if nets[1] in sup else nets[1]), sup[0]
    s = ((K['teile'].get(ref) or {}).get('stuetzt') or {}).get('netz')
    if s in nets: return s, [n for n in nets if n != s][0]
    return None


def _kupfer(B, Rt, net, side, ohne_ref, nur=True, clr=0.0, via_d=0.45):
    ""
    g = []
    for t in Rt.get('tracks', []):
        if t['layer'] != side or (t['net'] == net) != nur: continue
        w = max(t['w']) if isinstance(t['w'], list) else t['w']
        ls = LineString(t['pts']) if len(t['pts']) >= 2 else Point(t['pts'][0])
        g.append(ls.buffer(w / 2 + clr) if clr or not nur else ls)
    for v in Rt.get('vias', []):
        if (v['net'] == net) != nur: continue
        g.append(Point(v['x'], v['y']).buffer(via_d / 2 + clr) if clr or not nur else Point(v['x'], v['y']))
    for p in B['pads']:
        if p['ref'] == ohne_ref or (p['net'] == net) != nur: continue
        if side not in p['polys'] and not (p.get('drill', 0) > 0): continue
        qs = [Polygon(q) for L, qq in p['polys'].items() for q in qq if len(q) >= 3 and (L == side or p.get('drill', 0) > 0)]
        if qs: g.append(unary_union(qs).buffer(clr) if clr else unary_union(qs))
    return g


def ziele(B, Rt, K, ref, n=2, schritt=0.25, nah=1.5, abstand=2.0, mindest=1.0, bahn_max=0.75, rand=0.2, padabst=0.15, T=None, bel=None, plat=None):
    ""
    sn = netze_von_teil(K, B, ref)
    if not sn: return []
    s_net, o_net = sn
    fps = {f['ref']: f for f in B['fps']}; f = fps[ref]; side = f['side']
    T = T or lade_teile(B, padabst)
    if ref not in T or len(T[ref].pads) != 2: return []
    if plat is None: plat = platine(B, rand)
    _, inner, inner_p, _ = plat
    keep = [Polygon(q) for k in B.get('keepouts', []) if k.get('ref') != ref and (k.get('ref') or (k.get('verbote') or {}).get('teile', True))
            for q in k['polys'] if len(q) >= 3]
    keep = unary_union(keep) if keep else None
    if bel is None:
        bel = Belegung()
        for r in sorted(T): bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
    clr = float(Rt.get('clr') or 0.1)
    via_d = float(str((Rt.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])

    s_geo = _kupfer(B, Rt, s_net, side, ref)
    o_geo = _kupfer(B, Rt, o_net, side, ref)
    if not s_geo or not o_geo: return []
    O = unary_union(o_geo)
    S = unary_union(s_geo)
    fremd = {nt: unary_union(_kupfer(B, Rt, nt, side, ref, nur=False, clr=clr, via_d=via_d)) for nt in (s_net, o_net)}
    pts = []
    for g in s_geo:
        if g.geom_type == 'Point': pts.append((g.x, g.y)); continue
        if g.geom_type == 'LineString':
            k = max(1, int(math.ceil(g.length / schritt)))
            pts += [tuple(g.interpolate(i / k, normalized=True).coords[0]) for i in range(k + 1)]
        else:
            c = g.representative_point(); pts.append((c.x, c.y))
    pts = sorted({(round(x, 3), round(y, 3)) for x, y in pts if math.hypot(x - f['x'], y - f['y']) > nah})
    if not pts: return []
    P = shapely.points(np.array(pts))
    d = shapely.distance(P, O)
    ordn = sorted(range(len(pts)), key=lambda i: (round(float(d[i]), 4), pts[i]))

    pl = {}
    for p in B['pads']:
        if p['ref'] != ref: continue
        qs = [Polygon([(x - f['x'], y - f['y']) for x, y in q]) for L, qq in p['polys'].items() for q in qq if len(q) >= 3]
        pl[p['net']] = ((p['x'] - f['x'], p['y'] - f['y']), unary_union(qs) if qs else Point(p['x'] - f['x'], p['y'] - f['y']).buffer(0.25))
    if set(pl) != {s_net, o_net}: return []
    bel.remove(ref)
    out = []
    try:
        for i in ordn:
            if len(out) >= n: break
            p = pts[i]
            if any(math.hypot(p[0] - z['p'][0], p[1] - z['p'][1]) < abstand for z in out): continue
            q = nearest_points(Point(p), O)[1].coords[0]
            vx, vy = q[0] - p[0], q[1] - p[1]
            (sx, sy), _ = pl[s_net]; (ox, oy), _ = pl[o_net]
            ax, ay = ox - sx, oy - sy
            if math.hypot(vx, vy) < 1e-6: vx, vy = ax, ay
            th0 = math.degrees(math.atan2(vy, vx) - math.atan2(ay, ax))
            th = (round(th0 / 90.0) * 90) % 360
            c, sn_ = math.cos(math.radians(th)), math.sin(math.radians(th))
            rot = lambda x, y: (c * x - sn_ * y, sn_ * x + c * y)
            rs, ro = rot(sx, sy), rot(ox, oy)
            lagen = [('pad_auf_bahn', (p[0] - rs[0], p[1] - rs[1])),
                     ('mitte', ((p[0] + q[0]) / 2 - (rs[0] + ro[0]) / 2, (p[1] + q[1]) / 2 - (rs[1] + ro[1]) / 2)),
                     ('pad_auf_anderem', (q[0] - ro[0], q[1] - ro[1]))]
            versuche = [(name, (c0[0] + ex, c0[1] + ey)) for ex, ey in VERSATZ for name, c0 in lagen]
            for name, (cx, cy) in versuche:
                cx, cy = round(cx, 3), round(cy, 3)
                if math.hypot(cx - f['x'], cy - f['y']) < mindest: continue
                if S.distance(Point(cx + rs[0], cy + rs[1])) > bahn_max: continue
                geo = T[ref].geo(cx, cy, th, side)
                if bel.kollision(geo) or not all(inner_p.contains(x) for x in geo.values()): continue
                if keep is not None and any(x.intersects(keep) for x in geo.values()): continue
                frei = True
                for nt, (_, poly) in pl.items():
                    g = affinity.translate(affinity.rotate(poly, th, origin=(0, 0)), cx, cy)
                    if g.intersects(fremd[nt]): frei = False; break
                if not frei: continue
                out.append(dict(refs={ref: (round(cx - f['x'], 3), round(cy - f['y'], 3), float(th))}, d=round(float(d[i]), 3),
                                p=p, q=(round(q[0], 3), round(q[1], 3)), signal=s_net, anderes=o_net, lage=name, ziel=(cx, cy, th)))
                break
    finally:
        bel.add(ref, T[ref].geo(f['x'], f['y'], 0.0, side))
    return out


if __name__ == '__main__':
    B = json.load(open(sys.argv[1])); Rt = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3]))
    refs = sys.argv[4:] or sorted(r for r, t in K['teile'].items() if t.get('funktion') in ('pullup', 'pulldown'))
    for r in refs:
        for z in ziele(B, Rt, K, r, n=3):
            print('%-10s %-12s -> %-10s d=%.2f  %s  %s' % (r, z['signal'], z['anderes'], z['d'], z['refs'][r], z['lage']))
