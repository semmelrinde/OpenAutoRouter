""
import math, os

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from lernen import stromzonen as SZ

T = dict(aussen=0.035, innen=0.0175)


def _dicke(L):
    return T['aussen'] if L in ('F', 'B') else T['innen']


def segmentstroeme(B, R, K, n):
    ""
    from gemeinsam.fluss import analyse
    k = (K.get('netze') or {}).get(n) or {}
    idx = [i for i, t in enumerate(R.get('tracks', [])) if t.get('net') == n and len(t['pts']) >= 2]
    tr = [R['tracks'][i] for i in idx]
    pads = [p for p in B['pads'] if p['net'] == n]
    vias = [v for v in R.get('vias', []) if v.get('net') == n]
    info = analyse(tr, vias, pads, k.get('fluss')) if k.get('fluss') else None
    I = SZ.netz_strom(K, n) or 0.0
    out = {}
    for j, i in enumerate(idx):
        for s in range(len(R['tracks'][i]['w'])):
            a = (info or {}).get('anteil', {}).get((j, s)) if info else None
            out[(i, s)] = I * (a if a is not None else 1.0)
    return out, I


def via_felder(B, R, RR, K, zonen_out, netze, stroeme, clr, umriss, hole, je_ampere=4.0, n_max=40, r_feld=2.5):
    ""
    d_via, b_via = SZ._via_masse(R); teil = d_via + clr + .02; neu = []
    lagen = SZ._lagen(B); schutz_an = SZ.bahnschutz_an(R)
    for n in netze:
        seg_s, I = stroeme[n]
        if I < 0.25: continue
        zon = {}
        for z in zonen_out:
            if z['net'] == n: zon[z['layer']] = SZ._geo(z) if z['layer'] not in zon else zon[z['layer']].union(SZ._geo(z))
        if len(zon) < 2: continue
        fremd = unary_union([g for L in lagen for m, g in SZ._teile(B, RR, L) if m != n])
        bohr = unary_union([Point(v['x'], v['y']).buffer(b_via / 2, 16) for v in RR.get('vias', []) if v.get('net') != n] +
                           [Point(p['x'], p['y']).buffer(p['drill'] / 2, 16) for p in B['pads'] if p.get('drill') and p['net'] != n])
        erlaubt = umriss.buffer(-d_via / 2, 16).difference(fremd.buffer(d_via / 2 + clr + .003, 16)).difference(bohr.buffer(b_via / 2 + hole + .003, 16))
        maskensperre = unary_union([SZ.maskenfreiraum(B, n, L, clr, 16) for L in lagen])
        if not maskensperre.is_empty: erlaubt = erlaubt.difference(maskensperre.buffer(d_via / 2, 16))
        if schutz_an:
            schutz = unary_union([SZ._bahnschutz(RR, L, n)[1] for L in lagen])
            if not schutz.is_empty: erlaubt = erlaubt.difference(schutz.buffer(d_via / 2, 16))
        eigene = [(v['x'], v['y']) for v in RR.get('vias', []) if v.get('net') == n]
        for vx, vy in sorted(set(eigene)):
            att = {}
            for i, t in enumerate(R.get('tracks', [])):
                if t.get('net') != n: continue
                if schutz_an and any(t.get(k) for k in SZ.SCHUTZ): continue
                for k, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if math.dist(a, (vx, vy)) < 1e-3 or math.dist(b, (vx, vy)) < 1e-3:
                        att[t['layer']] = max(att.get(t['layer'], 0.0), seg_s.get((i, k), 0.0))
            ls = [L for L in att if L in zon]
            if len(ls) < 2: continue
            Iv = min(sorted(att.values())[-2:])
            soll = min(n_max, max(2, math.ceil(je_ampere * Iv)))
            nah = [q for q in eigene + [(v['x'], v['y']) for v in neu if v['net'] == n] if math.dist(q, (vx, vy)) <= r_feld]
            if len(nah) >= soll: continue
            reg = unary_union([zon[L] for L in ls[:1]])
            for L in ls[1:]: reg = reg.intersection(zon[L])
            reg = reg.intersection(Point(vx, vy).buffer(r_feld, 16)).intersection(erlaubt)
            if reg.is_empty: continue
            x0, y0, x1, y1 = reg.bounds; h = teil / 2
            kand = sorted(((x0 + i * h, y0 + j * h) for i in range(int((x1 - x0) / h) + 1) for j in range(int((y1 - y0) / h) + 1)),
                          key=lambda q: math.dist(q, (vx, vy)))
            belegt = [(v['x'], v['y']) for v in RR.get('vias', [])] + [(v['x'], v['y']) for v in neu]
            for q in kand:
                if len(nah) >= soll: break
                if not reg.contains(Point(q)) or any(math.dist(q, b) < teil for b in belegt): continue
                v = dict(x=round(q[0], 4), y=round(q[1], 4), net=n, quelle='strom_via', strom_a=round(Iv, 2))
                neu.append(v); belegt.append(q); nah.append(q)
    return neu


def zonen(B, R, K, gndlagen=None, clr=None, rand=0.25, J=40.0, w_max=6.0, saum=0.15, r_glatt=0.3, r_oeffnen=0.15, min_flaeche=0.05,
          via_feld=True, je_ampere=4.0, tot_kuerzen=True, reihenfolge=None, ordnung=None):
    ""
    gnd = set(K.get('gnd') or ['GND']); gl = set(SZ._gndlagen(R, gndlagen))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand, SZ.QS)
    d_via, b_via = SZ._via_masse(R)
    loecher = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, SZ.QS)) for v in R.get('vias', [])]
    loecher += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, SZ.QS)) for p in B['pads'] if p.get('drill')]
    prio = SZ.vorrang(B, R, K)
    netze = sorted(SZ.stromnetze(K, gnd), key=lambda n: (-prio[n]['gewicht'], n))
    if reihenfolge == 'kette' and ordnung is None:


        import copy
        from analyse.strom_kette import kette
        R1 = copy.deepcopy(R)
        z1 = zonen(B, R1, K, gndlagen, clr=clr, rand=rand, J=J, w_max=w_max, saum=saum, r_glatt=r_glatt, r_oeffnen=r_oeffnen,
                   min_flaeche=min_flaeche, via_feld=via_feld, je_ampere=je_ampere)
        KE = kette(B, dict(R1, zonen=[z for z in R1.get('zonen', []) if z.get('quelle') != 'stromzonen'] + z1), K)
        eta = {n: v['wirkungsgrad'] for n, v in KE['netze'].items()}
        ordnung = sorted(netze, key=lambda n: (eta.get(n, 1.0), n))
    if ordnung is not None: netze = [n for n in ordnung if n in netze] + [n for n in netze if n not in ordnung]
    stroeme = {n: segmentstroeme(B, R, K, n) for n in netze}
    RR = dict(R, zonen=list(R.get('zonen') or [])); out = []
    schutz = SZ.bahnschutz_an(RR)
    hole = float((R.get('optionen') or {}).get('h2h') or .2)
    for L in SZ._lagen(B):
        if L in gl: continue
        t = _dicke(L)
        for n in netze:
            teile = SZ._teile(B, RR, L)
            eigen = unary_union([g for m, g in teile if m == n])
            if eigen.is_empty: continue
            pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
            seg_s, I = stroeme[n]
            korr = []
            for i, tr in enumerate(RR.get('tracks', [])):
                if tr.get('net') != n or tr['layer'] != L or len(tr['pts']) < 2: continue
                if schutz and any(tr.get(k) for k in SZ.SCHUTZ): continue
                for s, (a, b, w) in enumerate(zip(tr['pts'][:-1], tr['pts'][1:], tr['w'])):
                    if a == b: continue
                    ws = min(w_max, max(w, seg_s.get((i, s), I) / (J * t)))
                    ls = LineString([a, b])
                    korr.append(ls.buffer(ws / 2, cap_style=2, join_style=2))
                    for p in (a, b):
                        korr.append(SZ._achteck(Point(p).buffer(1e-3, 4), ws / 2))
            if not korr and len(pads) < 2: continue
            vias_n = [Point(v['x'], v['y']).buffer(d_via / 2, 8) for v in RR.get('vias', []) if v.get('net') == n]
            kand = unary_union(korr + [SZ._achteck(p, saum) for p in pads + vias_n])
            fremd = unary_union([g for m, g in teile if m != n] + [g for m, g in loecher if m != n])
            frei = umriss.difference(fremd.buffer(clr + .003, SZ.QS)).difference(SZ.sperrflaeche(B, L, 'flaechen'))
            frei = SZ.maskenfrei(frei, B, n, L, clr)
            frei = frei.difference(unary_union([h.buffer(hole + .003, SZ.QS) for m, h in loecher if m != n]))
            if schutz:
                _, moat = SZ._bahnschutz(RR, L, n)
                if not moat.is_empty: frei = frei.difference(moat)
            z = SZ._oeffnen_g(kand.intersection(frei), r_oeffnen)
            z = unary_union([z, eigen])
            z = SZ._schliessen_g(z, r_glatt).intersection(unary_union([frei, eigen]))
            stuecke = [s for s in getattr(z, 'geoms', [z]) if s.geom_type == 'Polygon' and s.area >= min_flaeche and s.intersects(eigen)]
            if not stuecke: continue
            g = unary_union(stuecke + [eigen]).buffer(0)
            safe = unary_union([frei, eigen])
            g = SZ._oeffnen(SZ._schliessen(g.buffer(.02, SZ.QS).intersection(safe), .01), .01).intersection(safe).union(eigen)
            for _ in range(2):
                g = SZ.tangential_runden(g.simplify(.002, preserve_topology=True).union(eigen), eigen, frei)
            g = g.simplify(.0005, preserve_topology=True)
            zz = dict(net=n, layer=L, quelle='stromzonen', strom_a=round(I, 3), polys=SZ._polys(g), flaeche_mm2=round(g.area, 2),
                      eigen_mm2=round(eigen.area, 2), vorrang=prio[n], modell='minimax', J=J)
            out.append(zz); RR['zonen'].append(zz)
    if via_feld:
        nv = via_felder(B, R, RR, K, out, netze, stroeme, clr, umriss, hole, je_ampere=je_ampere)
        R.setdefault('vias', []).extend(nv)
    if tot_kuerzen: totes_kupfer_kuerzen(B, R, K, out, gndlagen)
    return out


def totes_kupfer_kuerzen(B, R, K, zonen_out, gndlagen=None, anteil=0.1, r_glatt=0.15):
    ""
    import tempfile
    import numpy as np
    from shapely import box, union_all
    from analyse.kupfermodell import ir_werte
    from analyse.strom_kette import kette
    RR = dict(R, zonen=[z for z in R.get('zonen', []) if z.get('quelle') != 'stromzonen'] + list(zonen_out))
    with tempfile.TemporaryDirectory() as td:
        ir = ir_werte(B, RR, K, karten=td)
        KE = kette(B, RR, K, ir=ir)
        karten = {n: np.load(os.path.join(td, f)) for n, f in (ir.get('karten') or {}).items() if f and os.path.exists(os.path.join(td, f))}
        weg = {}
        for z in zonen_out:
            n, L = z['net'], z['layer']
            if n not in karten or n not in KE['netze'] or (L + '_J_A_mm2') not in karten[n].files: continue
            M = karten[n]; J = M[L + '_J_A_mm2']; xx, yy = M['x_mm'], M['y_mm']
            schwelle = anteil * KE['netze'][n]['j_stern']
            tot = np.isfinite(J) & (J < schwelle)
            if not tot.any(): continue
            h = float(abs(xx[0, 1] - xx[0, 0])) if xx.ndim == 2 and xx.shape[1] > 1 else 0.1
            iy, ix = np.nonzero(tot)
            dead = union_all(box(xx[iy, ix] - .55 * h, yy[iy, ix] - .55 * h, xx[iy, ix] + .55 * h, yy[iy, ix] + .55 * h))
            g = SZ._geo(z)
            eigen = unary_union([gg for m, gg in SZ._teile(B, R, L) if m == n])
            neu = g.difference(dead.difference(eigen.buffer(.02, 8)))
            neu = SZ._oeffnen_g(neu, r_glatt).union(eigen.intersection(g))
            neu = SZ._schliessen_g(neu, r_glatt).intersection(g)
            teile = [p for p in getattr(neu, 'geoms', [neu]) if p.geom_type == 'Polygon' and p.intersects(eigen)]
            neu = unary_union(teile + [eigen.intersection(g)]).buffer(0).simplify(.0005, preserve_topology=True)
            if neu.is_empty: continue
            weg[n] = round(weg.get(n, 0.0) + g.area - neu.area, 2)
            z.update(polys=SZ._polys(neu), flaeche_mm2=round(neu.area, 2), tot_gekuerzt=True)
    return weg
