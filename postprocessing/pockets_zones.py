import json, sys, os, math
import numpy as np
from shapely.prepared import prep
from shapely.strtree import STRtree
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from common.pair_nets import norm
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
C = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); Z = json.load(open(sys.argv[3])); OUT = sys.argv[4]
opt = dict(a.split('=', 1) for a in sys.argv[5:] if '=' in a)
MIN_WIDTH = float(opt.get('min_width', '0.13'))
NETS = set(opt['net_list'].split(',')) if opt.get('net_list') else {z['net'] for z in Z if not z['rule'] and z['net'] and z['net'] != opt.get('gnd', 'GND')}
SUPPLY = {norm(n) for n, k in json.load(open(opt['netclasses']))['net_list'].items() if k.get('category') == 'supply'} - NETS if opt.get('netclasses') else set()
CLR = float(opt.get('clr', '0.15')); TOL = float(opt.get('tol', '0.01')); AMIN = float(opt.get('amin', '0.03'))
def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)
board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-0.5)
plan = dict(keepouts_way=[], zone_list={}, new=[])

VD, VDR = [float(q) for q in opt.get('via', '0.45/0.2').split('/')]; VR = VD / 2
DRC = float(opt.get('drc', '0.128')) + 0.01; H2H = float(opt.get('h2h', '0.25')) + 0.01; BORDER = float(opt.get('border', '0.5')) + 0.01
HOLE = float(opt.get('hole_size', '0.255')) + 0.01
innerv = prep(board.buffer(-(BORDER + VR)))
hard = [(LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2) for t in C['tracks'] if t['net'] != 'GND']
hard += [poly(q) for p in C['pads'] if p['net'] != 'GND' for qs in p['polys'].values() for q in qs]
hard += [Point(v['x'], v['y']).buffer(v['d'] / 2) for v in C['vias'] if v['net'] != 'GND']
hard += [poly(q) for f in C.get('shapes', []) if f['net'] != 'GND' for q in f['polys']]
hard = [g for g in hard if not g.is_empty]; tree = STRtree(hard)
bore = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p.get('drill', 0) > 0]
bore_pads = [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p.get('drill', 0) > 0 and p['net'] != 'GND']
keepout_v = prep(unary_union([Polygon(s['pts']).buffer(VR + 0.05) for s in S if s['fill'] or s['vias']]))
def via_ok(x, y):
    pt = Point(x, y)
    if not innerv.contains(pt) or keepout_v.contains(pt): return False
    if any(hard[int(j)].distance(pt) < VR + DRC for j in tree.query(pt.buffer(VR + DRC))): return False
    if any(math.hypot(bx - x, by - y) < br + VDR / 2 + H2H for bx, by, br in bore): return False
    return not any(math.hypot(bx - x, by - y) < br + VR + HOLE for bx, by, br in bore_pads)
def via_fits(g, L):
    bx = g.buffer(VR).bounds; gi = prep(g.buffer(VR - 0.05))
    return any(gi.contains(Point(x, y)) and via_ok(x, y) for x in np.arange(bx[0], bx[2], 0.025) for y in np.arange(bx[1], bx[3], 0.025))
for L in ('F', 'In2', 'B'):
    items = []
    for t in C['tracks']:
        if t['layer'] == L: items.append(((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2), t['net']))
    for v in C['vias']: items.append((Point(v['x'], v['y']).buffer(v['d'] / 2), v['net']))
    for p in C['pads']:
        items += [(poly(q), p['net']) for q in p['polys'].get(L, [])]
        if p.get('drill', 0) > 0: items.append((Point(p['x'], p['y']).buffer(p['drill'] / 2 + 0.105), '#DrillHole'))
    for z in C['zones']:
        if z['layer'] == L: items += [(poly(q), z['net']) for q in z['polys']]
    for f in C.get('shapes', []):
        if f['layer'] == L: items += [(poly(q), f['net']) for q in f['polys']]
    blocked = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['cu_layers'] and s['fill']])
    white = inner.difference(unary_union([g for g, _ in items]).buffer(CLR + TOL)).difference(blocked.buffer(CLR))
    for g in getattr(white, 'geoms', [white]):
        if g.area < AMIN or g.buffer(-MIN_WIDTH / 2).is_empty: continue
        border = g.buffer(CLR + TOL + 0.04)
        contact = {}
        for it, net in items:
            if (net in NETS or norm(net) in SUPPLY) and it.intersects(border): contact[net] = contact.get(net, 0) + it.intersection(border).area
        if not contact: continue
        net = max(contact, key=contact.get)
        if net not in NETS and via_fits(g, L):
            print('%-3s pocket %.3f mm² at (%.2f, %.2f) at track %s: GND via fits, stays for gaps_vias' % (L, g.area, g.centroid.x, g.centroid.y, net)); continue
        contour = g.buffer(CLR + 0.05, join_style=2)

        goal = None
        for z in Z:
            if not z['rule'] and z['net'] == net and L in z['cu_layers']:
                zo = unary_union([poly(p) for p in plan['zone_list'].get(z['uuid'], z['polys'])]) if z['uuid'] in plan['zone_list'] else unary_union([poly(p) for p in z['polys']])
                if zo.intersects(contour): goal = (z['uuid'], zo); break
        if goal:
            u, zo = goal; ng = zo.union(contour)
            plan['zone_list'][u] = [dict(outer=[list(c) for c in gg.exterior.coords[:-1]], hole_list=[[list(c) for c in h.coords[:-1]] for h in gg.interiors]) for gg in getattr(ng, 'geoms', [ng])]
            print('%-3s pocket %.3f mm² at (%.2f, %.2f) -> zone %s extended' % (L, g.area, g.centroid.x, g.centroid.y, net))
        else:
            plan['new'].append(dict(net=net, layer_name=L, prio=6, polys=[dict(outer=[list(c) for c in contour.exterior.coords[:-1]], hole_list=[])]))
            print('%-3s pocket %.3f mm² at (%.2f, %.2f) -> new zone %s' % (L, g.area, g.centroid.x, g.centroid.y, net))
json.dump(plan, open(OUT, 'w')); print('new zones:', len(plan['new']), '| extended:', len(plan['zone_list']))
