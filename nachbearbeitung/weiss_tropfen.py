""
import json, os, sys
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.paarnetze import paarnetze, norm
from gemeinsam.kupferform_anker import anker_laden, anbinden
D = json.load(open(sys.argv[1])); S = json.load(open(sys.argv[2])); OUT = sys.argv[3]
PADS = {}
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
ANKER = {}
if opt.get('pads'):
    _C = json.load(open(opt['pads']))
    ANKER = anker_laden(_C)
    for p in _C['pads']:
        for L, qs in p['polys'].items():
            for q in qs: PADS.setdefault(L, []).append(Polygon(q['aussen'], q['loecher']).buffer(0))
R = float(opt.get('r', '0.4')); FREI = float(opt.get('frei', '0.155')); RAND = float(opt.get('rand', '0.5'))
FREI_HART = float(opt.get('frei_hart', '0.155'))
RADIEN = [R] + sorted({float(x) for x in opt.get('radien', '0.3,0.2,0.15,0.1,0.07,0.05').split(',') if x and float(x) < R}, reverse=True)
ZONEN_HART = opt.get('zonen', 'hart') == 'hart'
EPS = 2e-6
HAAR = 0.001
DICKE = float(opt.get('dicke', '0.008'))
LOCH = float(opt.get('loch', '0.27')); AMIN = float(opt.get('amin', '0.00002'))
AUS = [a for a in opt.get('aus', '').split(',') if a]
NETZE = opt.get('netze', 'flaeche' if opt.get('nur_flaechen', '1') == '1' else 'alle')


def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps])
board_in = poly(D['outline']).buffer(-RAND)
loecher = unary_union([Point(x, y).buffer(r + LOCH, 24) for x, y, r in D['loecher']]) if D['loecher'] else Polygon()
kupfer, hart, zfill = {}, {}, {}
for net, lay in D['items'].items():
    for L, ps in lay.items(): kupfer.setdefault((net, L), []).append(poly(ps)); hart.setdefault((net, L), []).append(poly(ps))
for z in D['zonen']: kupfer.setdefault((z['net'], z['layer']), []).append(poly(z['polys'])); zfill.setdefault((z['net'], z['layer']), []).append(poly(z['polys']))
for f in D.get('formen', []): kupfer.setdefault((f['net'], f['layer']), []).append(poly(f['polys'])); hart.setdefault((f['net'], f['layer']), []).append(poly(f['polys']))
PAARE = paarnetze({n for n, _ in kupfer}, opt.get('klassen'))
kupfer = {k: unary_union(v) for k, v in kupfer.items()}; hart = {k: unary_union(v) for k, v in hart.items()}; zfill = {k: unary_union(v) for k, v in zfill.items()}
out = []; stat = {}; rstat = {}; verworfen = 0; neu = {}
from gemeinsam.abstand_tor import zu_nah, ausgabe_form
for L in sorted({L for _, L in kupfer}):
    alle = {n: g for (n, l), g in kupfer.items() if l == L and n}
    sperr = unary_union([Polygon(s['pts']).buffer(0) for s in S if L in s['lagen'] and (s['fuellen'] or s['bahnen'])])
    mit_zone = {z['net'] for z in D['zonen'] if z['layer'] == L}
    for n, U in sorted(alle.items()):
        paar = norm(n) in PAARE
        if (paar and not PADS) or any(a in n for a in AUS) or (NETZE == 'flaeche' and n not in mit_zone) or (NETZE == 'signal' and n in mit_zone): continue


        fremd_h = unary_union([g for (m, l), g in hart.items() if l == L and m and m != n] + [g for (m, l), g in neu.items() if l == L and m != n])
        fremd_z = unary_union([g for (m, l), g in zfill.items() if l == L and m and m != n]) if ZONEN_HART else Polygon()
        verbot = unary_union([fremd_h.buffer(FREI_HART, quad_segs=16), fremd_z.buffer(FREI, quad_segs=16), sperr, loecher])
        def zuwachs(Ux, r): return Ux.buffer(r, quad_segs=16).buffer(-r, quad_segs=16).difference(Ux)
        def stuecke(g): return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= AMIN]
        def passt(q):
            if q.difference(board_in).area > EPS or (q.intersects(verbot) and q.intersection(verbot).area > EPS): return False
            an = [pg for pg in PADS.get(L, []) if pg.distance(q) < 0.01]
            if len(an) >= 2: return False
            if paar and (not an or not an[0].buffer(1.5 * R, quad_segs=16).contains(q)): return False
            return True
        offen = [(0, q) for q in stuecke(zuwachs(U, RADIEN[0]))]; n_add = 0
        while offen:
            k, q = offen.pop()
            q = q.buffer(-HAAR, quad_segs=4).buffer(HAAR, quad_segs=4)
            if q.geom_type != 'Polygon':
                offen += [(k, p) for p in stuecke(q)]; continue
            if q.area < AMIN or q.buffer(-DICKE / 2).is_empty or q.distance(U) > 0.002: continue
            if not passt(q):
                if k + 1 < len(RADIEN):
                    r2 = RADIEN[k + 1]; Uloc = U.intersection(q.buffer(2 * r2 + 0.01))
                    offen += [(k + 1, q2) for q2 in stuecke(zuwachs(Uloc, r2).intersection(q.buffer(0.001)))]
                continue
            g = q.buffer(0.02, quad_segs=8).intersection(unary_union([U, q]).buffer(1e-7))

            for gg in getattr(g, 'geoms', [g]):
                if gg.geom_type == 'Polygon' and gg.area >= AMIN:
                    gg = anbinden(gg, U, ANKER.get((n, L), []))
                    pts = [list(c) for c in gg.simplify(0.0005).exterior.coords][:-1]
                    if zu_nah(ausgabe_form(pts), fremd_h, FREI_HART, fremd_z if ZONEN_HART else None, FREI):
                        verworfen += 1; continue
                    out.append(dict(net=n, layer=L, r=RADIEN[k], pts=pts)); n_add += 1
                    neu[(n, L)] = unary_union([neu.get((n, L), Polygon()), ausgabe_form(pts)])
                    rstat[RADIEN[k]] = rstat.get(RADIEN[k], 0) + 1
        if n_add: stat[(L, n)] = n_add
json.dump(out, open(OUT, 'w'))
print('Tropfen-Formen:', len(out), 'verworfen (Abstand):', verworfen, 'je Radius', dict(sorted(rstat.items(), reverse=True)), {'%s %s' % k: v for k, v in sorted(stat.items(), key=lambda kv: -kv[1])[:8]})
