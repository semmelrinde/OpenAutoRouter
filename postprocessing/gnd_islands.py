import json, sys, os, math, subprocess, tempfile, re
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree
from shapely.prepared import prep

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KPY = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
src, out = sys.argv[1], sys.argv[2]
args = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
DIST = float(args.get('spacing', '3.0')); DIST_S = float(args.get('spacing_fast', '1.5')); NMIN = int(args.get('min', '2'))
VD, VDR = [float(q) for q in args.get('via', '0.45/0.20').split('/')]; VR = VD / 2
DRC = float(args.get('drc', '0.09')) + 0.01; H2H = float(args.get('h2h', '0.2')) + 0.01; EDGE = float(args.get('edge', '0.2')) + 0.01
NC = json.load(open(args['netclasses'])) if 'netclasses' in args else None
SPLIT = float(args.get('splitter', '0')); RET = float(args.get('ret', '1.27'))
POCKET = float(args.get('pocket', '2.0'))
sys.path.insert(0, R)


def kicad(*a):
    r = subprocess.run([KPY] + list(a), capture_output=True, text=True)
    lines = [l for l in (r.stdout + r.stderr).splitlines() if not re.search(r'swig|assert|wxApp', l)]
    if r.returncode != 0: raise SystemExit('\n'.join(lines))
    return lines


def poly(p): return Polygon(p['outer'], p['hole_list']).buffer(0)


tmp = tempfile.mkdtemp()
kicad(os.path.join(R, 'kicad', 'copper_export.py'), src, os.path.join(tmp, 'cu.json'))
C = json.load(open(os.path.join(tmp, 'cu.json')))
GND = set(NC['gnd']) if NC else {n for n in {z['net'] for z in C['zones']} if re.search(r'(^|/)(A|D|P)?_?GND\d*$', n)}
FAST = {n for n, k in (NC or {}).get('net_list', {}).items() if k['category'] == 'sw' or (k['category'] == 'fast' and k['rank'] >= 3)}
board = unary_union([poly(p) for p in C['outline']])
inner = board.buffer(-(EDGE + VR)); inner_p = prep(inner)

foreign = {L: [] for L in C['cu_layers']}
for t in C['tracks']:
    if t['net'] in GND or t['layer'] not in foreign: continue
    g = LineString(t['pts']).buffer(t['w'] / 2) if len(t['pts']) > 1 else Point(t['pts'][0]).buffer(t['w'] / 2)
    foreign[t['layer']].append(g)
for v in C['vias']:
    if v['net'] in GND: continue
    for L in foreign: foreign[L].append(Point(v['x'], v['y']).buffer(v['d'] / 2))
for p in C['pads']:
    if p['net'] in GND: continue
    for L, ps in p['polys'].items():
        if L in foreign: foreign[L].extend(poly(q) for q in ps)
for z in C['zones']:
    if z['net'] in GND: continue
    foreign[z['layer']].extend(poly(q) for q in z['polys'])
tree_idx = {L: (STRtree(g), g) for L, g in foreign.items() if g}
bore = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0]
fast_geo = {L: [] for L in C['cu_layers']}
for t in C['tracks']:
    if t['net'] in FAST and t['layer'] in fast_geo: fast_geo[t['layer']].append(LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0]))
gnd_vias = [(v['x'], v['y']) for v in C['vias'] if v['net'] in GND] + [(p['x'], p['y']) for p in C['pads'] if p['net'] in GND and p['drill'] > 0]


def via_ok(x, y, new):
    from postprocessing.gnd_spacing import via_free
    return via_free(x, y, inner_p, tree_idx, bore, new, VR, VDR, DRC, H2H)


area_board = board.area
report = []; new = []; keepout_list = []
for z in C['zones']:
    if z['net'] not in GND: continue
    for q in z['polys']:
        P = poly(q)
        if P.is_empty or P.area < 0.05: continue
        if P.area > 0.6 * area_board: continue
        Pp = prep(P.buffer(0.01))
        da = [v for v in gnd_vias + [(a, b_) for a, b_, n_ in new if n_ == z['net']] if Pp.contains(Point(v))]
        close = fast_geo.get(z['layer']) and any(P.distance(g) < 1.0 for g in fast_geo[z['layer']])
        A = DIST_S if close else DIST

        bx = P.bounds
        xs = np.arange(bx[0], bx[2], 0.2); ys = np.arange(bx[1], bx[3], 0.2)
        probe = [(x, y) for x in xs for y in ys if Pp.contains(Point(x, y))] or [(P.representative_point().x, P.representative_point().y)]
        Pin = prep(P.buffer(-VR))
        cand = [(x, y) for x in np.arange(bx[0], bx[2], 0.1) for y in np.arange(bx[1], bx[3], 0.1) if Pin.contains(Point(x, y))]
        def open_items(vs):
            return [p for p in probe if not vs or min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > A]
        vs = list(da); assigned = 0
        while len(vs) < NMIN or open_items(vs):
            op = open_items(vs)
            best = None
            for x, y in cand:
                if any(math.hypot(x - a, y - b_) < 0.6 for a, b_ in vs): continue
                if op: gain = sum(1 for p in op if math.hypot(p[0] - x, p[1] - y) <= A)
                else: gain = min(math.hypot(x - a, y - b_) for a, b_ in vs) if vs else 1.0
                if gain <= 0: continue
                if best is not None and gain <= best[0]: continue
                if not via_ok(x, y, [(a, b_) for a, b_, n_ in new]): continue
                best = (gain, x, y)
            if best is None: break
            vs.append((best[1], best[2])); new.append((round(best[1], 3), round(best[2], 3), z['net'])); assigned += 1
        way = len(vs) < NMIN and P.area < SPLIT
        if way:
            for _ in range(assigned): new.pop()
            keepout_list.append(dict(layer=z['layer'], pts=[list(c) for c in P.buffer(0.05).exterior.coords]))
        pocket = 0.0
        if not way and POCKET > 0 and close and vs:
            sg = [g for g in fast_geo[z['layer']] if P.distance(g) < 1.0]
            op = [p for p in probe if min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > POCKET and min(g.distance(Point(p)) for g in sg) < 1.0]
            if op:
                from learning.current_zones import open_pocket
                og = unary_union([Point(p).buffer(0.15, 4) for p in op])
                t = open_pocket(P, og, vs, VD)
                for q in ([] if t is None else [q for q in getattr(t, 'geoms', [t]) if q.geom_type == 'Polygon']):
                    keepout_list.append(dict(layer=z['layer'], pts=[list(c) for c in q.buffer(0.02).exterior.coords])); pocket += q.area
        report.append(dict(layer_name=z['layer'], net_name=z['net'], surface=round(P.area, 2), before=len(da), new=0 if way else assigned,
                            after=len(vs), nearby_fast=bool(close), unsupplied=0 if way else len(open_items(vs)),
                            too_little=len(vs) < NMIN and not way, removed=way, pocket_mm2=round(pocket, 2)))

ret = dict(vias=0, already=0, new=0, no_slot=0)
if RET > 0 and NC:
    SCH = {n for n, k in NC['net_list'].items() if k['category'] == 'fast'}
    GNDZ = {}
    for z in C['zones']:
        if z['net'] in GND: GNDZ.setdefault(z['layer'], []).extend(poly(q) for q in z['polys'])
    GNDZ = {L: prep(unary_union(g).buffer(-VR)) for L, g in GNDZ.items() if g}
    gnd_net = NC.get('gnd_main') or sorted(GND)[0]
    for v in C['vias']:
        if v['net'] not in SCH: continue
        ret['vias'] += 1
        all_items = gnd_vias + [(a, b_) for a, b_, n_ in new]
        if any(math.hypot(v['x'] - a, v['y'] - b_) <= RET for a, b_ in all_items): ret['already'] += 1; continue
        ok = False
        for r in (0.55, 0.7, 0.85, 1.0, 1.15, RET - 0.02):
            for k in range(24):
                x = v['x'] + r * math.cos(2 * math.pi * k / 24); y = v['y'] + r * math.sin(2 * math.pi * k / 24)
                if not any(Z.contains(Point(x, y)) for Z in GNDZ.values()): continue
                if not via_ok(x, y, [(a, b_) for a, b_, n_ in new]): continue
                new.append((round(x, 3), round(y, 3), gnd_net)); ok = True; break
            if ok: break
        ret['new' if ok else 'no_slot'] += 1
    print('Return vias of fast nets: %d signal vias, %d already with GND via <= %.2f mm, %d new, %d without room' % (
        ret['vias'], ret['already'], RET, ret['new'], ret['no_slot']))
print('GND islands: %d, new vias: %d, slivers removed: %d, open strip ends removed (R41): %.1f mm2, with < %d vias: before %d, after %d, spots > spacing: %d' % (
    len(report), len(new), sum(b['removed'] for b in report), sum(b['pocket_mm2'] for b in report), NMIN, sum(1 for b in report if b['before'] < NMIN), sum(1 for b in report if b['too_little']),
    sum(b['unsupplied'] for b in report)))
json.dump(report, open(out.replace('.kicad_pcb', '_gnd_islands.json'), 'w'), indent=1)
if args.get('only_report') == '1' or not (new or keepout_list):
    if src != out and not (new or keepout_list):
        import shutil; shutil.copy(src, out)
    sys.exit(0)
json.dump(dict(vias=[dict(x=x, y=y, net=n) for x, y, n in new], keepout_list=keepout_list), open(os.path.join(tmp, 'vias.json'), 'w'))
for l in kicad(os.path.join(R, 'kicad', 'vias_set.py'), src, os.path.join(tmp, 'vias.json'), out, 'via=%g/%g' % (VD, VDR)): print(l)
