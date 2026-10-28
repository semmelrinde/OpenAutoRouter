""
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
ABST = float(args.get('abstand', '3.0')); ABST_S = float(args.get('abstand_schnell', '1.5')); NMIN = int(args.get('min', '2'))
VD, VDR = [float(q) for q in args.get('via', '0.45/0.20').split('/')]; VR = VD / 2
DRC = float(args.get('drc', '0.09')) + 0.01; H2H = float(args.get('h2h', '0.2')) + 0.01; EDGE = float(args.get('edge', '0.2')) + 0.01
KL = json.load(open(args['klassen'])) if 'klassen' in args else None
SPLIT = float(args.get('splitter', '0')); RUECK = float(args.get('rueck', '1.27'))
TASCHE = float(args.get('tasche', '2.0'))
sys.path.insert(0, R)


def kicad(*a):
    r = subprocess.run([KPY] + list(a), capture_output=True, text=True)
    lines = [l for l in (r.stdout + r.stderr).splitlines() if not re.search(r'swig|assert|wxApp', l)]
    if r.returncode != 0: raise SystemExit('\n'.join(lines))
    return lines


def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)


tmp = tempfile.mkdtemp()
kicad(os.path.join(R, 'kicad', 'kupfer_export.py'), src, os.path.join(tmp, 'cu.json'))
C = json.load(open(os.path.join(tmp, 'cu.json')))
GND = set(KL['gnd']) if KL else {n for n in {z['net'] for z in C['zones']} if re.search(r'(^|/)(A|D|P)?_?GND\d*$', n)}
SCHNELL = {n for n, k in (KL or {}).get('netze', {}).items() if k['art'] == 'sw' or (k['art'] == 'schnell' and k['rang'] >= 3)}
board = unary_union([poly(p) for p in C['outline']])
inner = board.buffer(-(EDGE + VR)); inner_p = prep(inner)

fremd = {L: [] for L in C['lagen']}
for t in C['tracks']:
    if t['net'] in GND or t['layer'] not in fremd: continue
    g = LineString(t['pts']).buffer(t['w'] / 2) if len(t['pts']) > 1 else Point(t['pts'][0]).buffer(t['w'] / 2)
    fremd[t['layer']].append(g)
for v in C['vias']:
    if v['net'] in GND: continue
    for L in fremd: fremd[L].append(Point(v['x'], v['y']).buffer(v['d'] / 2))
for p in C['pads']:
    if p['net'] in GND: continue
    for L, ps in p['polys'].items():
        if L in fremd: fremd[L].extend(poly(q) for q in ps)
for z in C['zones']:
    if z['net'] in GND: continue
    fremd[z['layer']].extend(poly(q) for q in z['polys'])
baum = {L: (STRtree(g), g) for L, g in fremd.items() if g}
bohr = [(v['x'], v['y'], v['drill'] / 2) for v in C['vias']] + [(p['x'], p['y'], p['drill'] / 2) for p in C['pads'] if p['drill'] > 0]
schnell_geo = {L: [] for L in C['lagen']}
for t in C['tracks']:
    if t['net'] in SCHNELL and t['layer'] in schnell_geo: schnell_geo[t['layer']].append(LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0]))
gnd_vias = [(v['x'], v['y']) for v in C['vias'] if v['net'] in GND] + [(p['x'], p['y']) for p in C['pads'] if p['net'] in GND and p['drill'] > 0]


def via_ok(x, y, neu):
    if not inner_p.contains(Point(x, y)): return False
    pt = Point(x, y)
    for L, (tr, gs) in baum.items():
        for j in tr.query(pt.buffer(VR + DRC)):
            if gs[int(j)].distance(pt) < VR + DRC: return False
    for bx, by, br in bohr + [(a, b_, VDR / 2) for a, b_ in neu]:
        if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
    return True


area_board = board.area
bericht = []; neu = []; sperren = []
for z in C['zones']:
    if z['net'] not in GND: continue
    for q in z['polys']:
        P = poly(q)
        if P.is_empty or P.area < 0.05: continue
        if P.area > 0.6 * area_board: continue
        Pp = prep(P.buffer(0.01))
        da = [v for v in gnd_vias + [(a, b_) for a, b_, n_ in neu if n_ == z['net']] if Pp.contains(Point(v))]
        nah = schnell_geo.get(z['layer']) and any(P.distance(g) < 1.0 for g in schnell_geo[z['layer']])
        A = ABST_S if nah else ABST

        bx = P.bounds
        xs = np.arange(bx[0], bx[2], 0.2); ys = np.arange(bx[1], bx[3], 0.2)
        probe = [(x, y) for x in xs for y in ys if Pp.contains(Point(x, y))] or [(P.representative_point().x, P.representative_point().y)]
        Pin = prep(P.buffer(-VR))
        kand = [(x, y) for x in np.arange(bx[0], bx[2], 0.1) for y in np.arange(bx[1], bx[3], 0.1) if Pin.contains(Point(x, y))]
        def offen(vs):
            return [p for p in probe if not vs or min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > A]
        vs = list(da); gesetzt = 0
        while len(vs) < NMIN or offen(vs):
            op = offen(vs)
            best = None
            for x, y in kand:
                if any(math.hypot(x - a, y - b_) < 0.6 for a, b_ in vs): continue
                if op: gain = sum(1 for p in op if math.hypot(p[0] - x, p[1] - y) <= A)
                else: gain = min(math.hypot(x - a, y - b_) for a, b_ in vs) if vs else 1.0
                if gain <= 0: continue
                if best is not None and gain <= best[0]: continue
                if not via_ok(x, y, [(a, b_) for a, b_, n_ in neu]): continue
                best = (gain, x, y)
            if best is None: break
            vs.append((best[1], best[2])); neu.append((round(best[1], 3), round(best[2], 3), z['net'])); gesetzt += 1
        weg = len(vs) < NMIN and P.area < SPLIT
        if weg:
            for _ in range(gesetzt): neu.pop()
            sperren.append(dict(layer=z['layer'], pts=[list(c) for c in P.buffer(0.05).exterior.coords]))
        tasche = 0.0
        if not weg and TASCHE > 0 and nah and vs:
            sg = [g for g in schnell_geo[z['layer']] if P.distance(g) < 1.0]
            op = [p for p in probe if min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > TASCHE and min(g.distance(Point(p)) for g in sg) < 1.0]
            if op:
                from lernen.stromzonen import offene_tasche
                og = unary_union([Point(p).buffer(0.15, 4) for p in op])
                t = offene_tasche(P, og, vs, VD)
                for q in ([] if t is None else [q for q in getattr(t, 'geoms', [t]) if q.geom_type == 'Polygon']):
                    sperren.append(dict(layer=z['layer'], pts=[list(c) for c in q.buffer(0.02).exterior.coords])); tasche += q.area
        bericht.append(dict(lage=z['layer'], netz=z['net'], flaeche=round(P.area, 2), vorher=len(da), neu=0 if weg else gesetzt,
                            nachher=len(vs), nahe_schnell=bool(nah), unversorgt=0 if weg else len(offen(vs)),
                            zu_wenig=len(vs) < NMIN and not weg, entfernt=weg, tasche_mm2=round(tasche, 2)))

rueck = dict(vias=0, schon=0, neu=0, kein_platz=0)
if RUECK > 0 and KL:
    SCH = {n for n, k in KL['netze'].items() if k['art'] == 'schnell'}
    GNDZ = {}
    for z in C['zones']:
        if z['net'] in GND: GNDZ.setdefault(z['layer'], []).extend(poly(q) for q in z['polys'])
    GNDZ = {L: prep(unary_union(g).buffer(-VR)) for L, g in GNDZ.items() if g}
    gnetz = KL.get('gnd_haupt') or sorted(GND)[0]
    for v in C['vias']:
        if v['net'] not in SCH: continue
        rueck['vias'] += 1
        alle = gnd_vias + [(a, b_) for a, b_, n_ in neu]
        if any(math.hypot(v['x'] - a, v['y'] - b_) <= RUECK for a, b_ in alle): rueck['schon'] += 1; continue
        ok = False
        for r in (0.55, 0.7, 0.85, 1.0, 1.15, RUECK - 0.02):
            for k in range(24):
                x = v['x'] + r * math.cos(2 * math.pi * k / 24); y = v['y'] + r * math.sin(2 * math.pi * k / 24)
                if not any(Z.contains(Point(x, y)) for Z in GNDZ.values()): continue
                if not via_ok(x, y, [(a, b_) for a, b_, n_ in neu]): continue
                neu.append((round(x, 3), round(y, 3), gnetz)); ok = True; break
            if ok: break
        rueck['neu' if ok else 'kein_platz'] += 1
    print('Rueck-Vias schneller Netze: %d Signal-Vias, %d schon mit GND-Via <= %.2f mm, %d neu, %d ohne Platz' % (
        rueck['vias'], rueck['schon'], RUECK, rueck['neu'], rueck['kein_platz']))
print('GND-Inseln: %d, neue Vias: %d, Splitter entfernt: %d, offene Streifenenden entfernt (R41): %.1f mm2, mit < %d Vias: vorher %d, nachher %d, Stellen > Abstand: %d' % (
    len(bericht), len(neu), sum(b['entfernt'] for b in bericht), sum(b['tasche_mm2'] for b in bericht), NMIN, sum(1 for b in bericht if b['vorher'] < NMIN), sum(1 for b in bericht if b['zu_wenig']),
    sum(b['unversorgt'] for b in bericht)))
json.dump(bericht, open(out.replace('.kicad_pcb', '_gnd_inseln.json'), 'w'), indent=1)
if args.get('nur_bericht') == '1' or not (neu or sperren):
    if src != out and not (neu or sperren):
        import shutil; shutil.copy(src, out)
    sys.exit(0)
json.dump(dict(vias=[dict(x=x, y=y, net=n) for x, y, n in neu], sperren=sperren), open(os.path.join(tmp, 'vias.json'), 'w'))
for l in kicad(os.path.join(R, 'kicad', 'vias_setzen.py'), src, os.path.join(tmp, 'vias.json'), out, 'via=%g/%g' % (VD, VDR)): print(l)
