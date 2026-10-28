""
import json, sys, os, math, collections

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
from gemeinsam.teile import lade_teile, platine, Belegung
from gemeinsam.wanderweite import tabelle, beweglich as darf_wandern
from nachbearbeitung.zugregeln import sperre

B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3]))
args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
OUT = args.get('out', sys.argv[2].replace('.json', '_engstellen.json'))
MAXS = float(args.get('max_schritt', '0.3'))
NETZ = K['netze']; TEILE = K['teile']; GND = set(K['gnd'])
fps = {f['ref']: f for f in B['fps']}
T = lade_teile(B, 0.15)
board, inner, inner_p, keep = platine(B, 0.2)
WT = tabelle(K)

padg = collections.defaultdict(list)
for p in B['pads']:
    for L, qs in p['polys'].items():
        g = unary_union([Polygon(q) for q in qs if len(q) >= 3])
        padg[L].append((p['ref'], p['net'], g))
tree = {L: STRtree([g for _, _, g in v]) for L, v in padg.items()}
beweglich = lambda r: r in WT and darf_wandern(WT[r]) and not fps[r].get('locked')
vorschlag = collections.defaultdict(lambda: [0.0, 0.0, []])


def schieben(r, dx, dy, grund):
    v = vorschlag[r]; v[0] += dx; v[1] += dy; v[2].append(grund)



kf = sys.argv[2].replace('.json', '_konflikte.json')
konf = json.load(open(kf)) if os.path.exists(kf) else []
for lay, dist, ua, ub, x, y in konf:
    pt = Point(x, y)
    if lay not in tree: continue
    for j in tree[lay].query(pt.buffer(0.6)):
        r, net, g = padg[lay][int(j)]
        if not beweglich(r) or g.distance(pt) > 0.5: continue
        c = g.centroid; d = math.hypot(c.x - x, c.y - y) or 1e-3
        k = (0.15 + max(0.0, 0.1 - dist)) / d
        schieben(r, (c.x - x) * k, (c.y - y) * k, 'Konflikt %s/%s auf %s bei (%.2f, %.2f)' % (ua, ub, lay, x, y))

from gemeinsam.fluss import analyse as fluss_analyse, zielbreite
soll_seg = {}
pads_n = collections.defaultdict(list)
for p in B['pads']: pads_n[p['net']].append(p)
for n, k in NETZ.items():
    if k.get('art') not in ('strom', 'versorgung') or not k.get('fluss'): continue
    tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
    info = fluss_analyse(tr, [v for v in R['vias'] if v['net'] == n], pads_n[n], k['fluss'])
    if not info: continue
    for ti, t in enumerate(tr):
        for si in range(len(t['w'])):
            soll_seg[(id(t), si)] = max(zielbreite(info['anteil'].get((ti, si), 1.0), k['wmin'], k['w']), info['unten_max'].get((ti, si), 0.0))
schmal = []
for t in R['tracks']:
    k = NETZ.get(t['net'])
    if not k or k['art'] not in ('strom', 'versorgung', 'sw') or t.get('sig') or t['layer'] not in tree: continue
    for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
        soll = soll_seg.get((id(t), si), k['w'])
        if w >= 0.8 * soll or math.dist(a, b) < 0.2: continue
        seg = LineString([a, b]); best = None
        for j in tree[t['layer']].query(seg.buffer(k['w'] / 2 + 0.3)):
            r, net, g = padg[t['layer']][int(j)]
            if net == t['net']: continue
            d = seg.distance(g)
            if best is None or d < best[0]: best = (d, r, g)
        if not best: continue
        d, r, g = best
        schmal.append(dict(netz=t['net'], lage=t['layer'], breite=w, soll=round(soll, 3), begrenzt_durch=r, abstand=round(d, 3)))
        if beweglich(r):
            c = g.centroid; q = seg.interpolate(seg.project(c)); dd = math.hypot(c.x - q.x, c.y - q.y) or 1e-3
            need = min(MAXS, (soll - w) / 2)
            schieben(r, (c.x - q.x) / dd * need, (c.y - q.y) / dd * need, '%s nur %.2f statt %.2f mm' % (t['net'], w, soll))

by = collections.defaultdict(float)
for t in R['tracks']:
    for a, b in zip(t['pts'][:-1], t['pts'][1:]): by[t['net']] += math.dist(a, b)
pads_net = collections.defaultdict(list)
for p in B['pads']: pads_net[p['net']].append((p['x'], p['y']))
def mst(pts):
    inT = [pts[0]]; rest = pts[1:]; L = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L
umweg = []
for n, L in by.items():
    if n in GND or n not in pads_net or len(pads_net[n]) < 2: continue
    m = mst(pads_net[n])
    if L > 3.0 and L / max(m, 0.1) > 1.6: umweg.append(dict(netz=n, laenge=round(L, 1), luftlinie=round(m, 1), art=NETZ.get(n, {}).get('art')))
umweg.sort(key=lambda u: -(u['laenge'] - u['luftlinie']))


bel = Belegung()
for r in T: bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
ok = {}; abgelehnt = []
for r, (dx, dy, gr) in sorted(vorschlag.items(), key=lambda kv: -math.hypot(kv[1][0], kv[1][1])):
    s = math.hypot(dx, dy)
    if s < 0.02: continue
    grenze = min(MAXS, WT[r]['max'])
    if s > grenze: dx, dy = dx * grenze / s, dy * grenze / s
    dx, dy = round(dx, 3), round(dy, 3)
    st = (fps[r]['x'] + dx, fps[r]['y'] + dy, 0.0, fps[r]['side'])
    g = T[r].geo(*st)
    bel.remove(r)
    frei = not bel.kollision(g) and all(inner_p.contains(x) for x in g.values()) and not (keep is not None and sperre(B, g, (r,)))
    if frei: ok[r] = (st, gr); bel.add(r, g)
    else:
        abgelehnt.append((r, gr[0])); bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
erg = dict(konflikte=len(konf), schmale_strompfade=schmal, umwege=umweg[:20],
           verschiebungen={r: dict(dx=round(st[0] - fps[r]['x'], 3), dy=round(st[1] - fps[r]['y'], 3), gruende=gr) for r, (st, gr) in ok.items()},
           nicht_moeglich=[dict(teil=r, grund=g) for r, g in abgelehnt])
json.dump(erg, open(OUT, 'w'), indent=1)
if 'placement' in args and ok:
    parts = {r: dict(side=st[3], pads={nr: [round(x, 4), round(y, 4)] for nr, net, x, y, ps in T[r].pads_abs(*st)}) for r, (st, gr) in ok.items()}
    json.dump(dict(parts=parts), open(args['placement'], 'w'))
print('Konflikte %d | schmale Strompfad-Segmente %d | Umwege %d | Verschiebungen moeglich %d, nicht moeglich %d -> %s' % (
    len(konf), len(schmal), len(umweg), len(ok), len(abgelehnt), OUT))
for r, (st, gr) in list(ok.items())[:12]:
    print('  %-10s um (%+.2f, %+.2f) mm: %s' % (r, st[0] - fps[r]['x'], st[1] - fps[r]['y'], gr[0]))
for u in umweg[:6]: print('  Umweg %-28s %5.1f mm statt %5.1f mm (%s)' % (u['netz'], u['laenge'], u['luftlinie'], u['art']))
