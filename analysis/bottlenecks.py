import json, sys, os, math, collections

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
from common.parts import load_parts, circuit_board, Occupancy
from common.travel_range import table, movable as may_wander
from postprocessing.move_rules import keepout

B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3]))
args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
OUT = args.get('out', sys.argv[2].replace('.json', '_bottlenecks.json'))
MAXS = float(args.get('max_step', '0.3'))
NET = K['net_list']; PARTS = K['part_list']; GND = set(K['gnd'])
fps = {f['ref']: f for f in B['fps']}
T = load_parts(B, 0.15)
board, inner, inner_p, keep = circuit_board(B, 0.2)
WT = table(K)

padg = collections.defaultdict(list)
for p in B['pads']:
    for L, qs in p['polys'].items():
        g = unary_union([Polygon(q) for q in qs if len(q) >= 3])
        padg[L].append((p['ref'], p['net'], g))
tree = {L: STRtree([g for _, _, g in v]) for L, v in padg.items()}
movable = lambda r: r in WT and may_wander(WT[r]) and not fps[r].get('locked')
suggestion = collections.defaultdict(lambda: [0.0, 0.0, []])


def shift(r, dx, dy, reason):
    v = suggestion[r]; v[0] += dx; v[1] += dy; v[2].append(reason)



kf = sys.argv[2].replace('.json', '_conflicts.json')
conf = json.load(open(kf)) if os.path.exists(kf) else []
for lay, dist, ua, ub, x, y in conf:
    pt = Point(x, y)
    if lay not in tree: continue
    for j in tree[lay].query(pt.buffer(0.6)):
        r, net, g = padg[lay][int(j)]
        if not movable(r) or g.distance(pt) > 0.5: continue
        c = g.centroid; d = math.hypot(c.x - x, c.y - y) or 1e-3
        k = (0.15 + max(0.0, 0.1 - dist)) / d
        shift(r, (c.x - x) * k, (c.y - y) * k, 'Conflict %s/%s on %s at (%.2f, %.2f)' % (ua, ub, lay, x, y))

from common.flow import analysis as flow_analysis, goal_width
nominal_seg = {}
pads_n = collections.defaultdict(list)
for p in B['pads']: pads_n[p['net']].append(p)
for n, k in NET.items():
    if k.get('category') not in ('current_val', 'supply') or not k.get('flow'): continue
    tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
    info = flow_analysis(tr, [v for v in R['vias'] if v['net'] == n], pads_n[n], k['flow'])
    if not info: continue
    for ti, t in enumerate(tr):
        for si in range(len(t['w'])):
            nominal_seg[(id(t), si)] = max(goal_width(info['share'].get((ti, si), 1.0), k['wmin'], k['w']), info['bottom_max'].get((ti, si), 0.0))
narrow = []
for t in R['tracks']:
    k = NET.get(t['net'])
    if not k or k['category'] not in ('current_val', 'supply', 'sw') or t.get('sig') or t['layer'] not in tree: continue
    for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
        nominal = nominal_seg.get((id(t), si), k['w'])
        if w >= 0.8 * nominal or math.dist(a, b) < 0.2: continue
        seg = LineString([a, b]); best = None
        for j in tree[t['layer']].query(seg.buffer(k['w'] / 2 + 0.3)):
            r, net, g = padg[t['layer']][int(j)]
            if net == t['net']: continue
            d = seg.distance(g)
            if best is None or d < best[0]: best = (d, r, g)
        if not best: continue
        d, r, g = best
        narrow.append(dict(net_name=t['net'], layer_name=t['layer'], width=w, nominal=round(nominal, 3), limited_by=r, spacing=round(d, 3)))
        if movable(r):
            c = g.centroid; q = seg.interpolate(seg.project(c)); dd = math.hypot(c.x - q.x, c.y - q.y) or 1e-3
            need = min(MAXS, (nominal - w) / 2)
            shift(r, (c.x - q.x) / dd * need, (c.y - q.y) / dd * need, '%s only %.2f instead of %.2f mm' % (t['net'], w, nominal))

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
detour = []
for n, L in by.items():
    if n in GND or n not in pads_net or len(pads_net[n]) < 2: continue
    m = mst(pads_net[n])
    if L > 3.0 and L / max(m, 0.1) > 1.6: detour.append(dict(net_name=n, total_length=round(L, 1), air_line=round(m, 1), category=NET.get(n, {}).get('category')))
detour.sort(key=lambda u: -(u['total_length'] - u['air_line']))


bel = Occupancy()
for r in T: bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
ok = {}; declined = []
for r, (dx, dy, gr) in sorted(suggestion.items(), key=lambda kv: -math.hypot(kv[1][0], kv[1][1])):
    s = math.hypot(dx, dy)
    if s < 0.02: continue
    limit = min(MAXS, WT[r]['max'])
    if s > limit: dx, dy = dx * limit / s, dy * limit / s
    dx, dy = round(dx, 3), round(dy, 3)
    st = (fps[r]['x'] + dx, fps[r]['y'] + dy, 0.0, fps[r]['side'])
    g = T[r].geo(*st)
    bel.remove(r)
    vacant = not bel.collision(g) and all(inner_p.contains(x) for x in g.values()) and not (keep is not None and keepout(B, g, (r,)))
    if vacant: ok[r] = (st, gr); bel.add(r, g)
    else:
        declined.append((r, gr[0])); bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
erg = dict(conflict_list=len(conf), narrow_current_paths=narrow, detours=detour[:20],
           shifts={r: dict(dx=round(st[0] - fps[r]['x'], 3), dy=round(st[1] - fps[r]['y'], 3), reasons=gr) for r, (st, gr) in ok.items()},
           not_possible=[dict(part=r, reason=g) for r, g in declined])
json.dump(erg, open(OUT, 'w'), indent=1)
if 'placement' in args and ok:
    parts = {r: dict(side=st[3], pads={nr: [round(x, 4), round(y, 4)] for nr, net, x, y, ps in T[r].pads_abs(*st)}) for r, (st, gr) in ok.items()}
    json.dump(dict(parts=parts), open(args['placement'], 'w'))
print('Conflicts %d | narrow current-path segments %d | detours %d | shifts possible %d, not possible %d -> %s' % (
    len(conf), len(narrow), len(detour), len(ok), len(declined), OUT))
for r, (st, gr) in list(ok.items())[:12]:
    print('  %-10s by (%+.2f, %+.2f) mm: %s' % (r, st[0] - fps[r]['x'], st[1] - fps[r]['y'], gr[0]))
for u in detour[:6]: print('  Detour %-28s %5.1f mm instead of %5.1f mm (%s)' % (u['net_name'], u['total_length'], u['air_line'], u['category']))
