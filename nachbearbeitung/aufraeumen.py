""
import json, math, os, sys
from collections import Counter, defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.kupfer_netz import Hindernisse, Netzkupfer, anwenden


MITTE_EPS = 0.005


def K(p): return (round(p[0], 6), round(p[1], 6))


def _richtung(a, b):
    d = math.dist(a, b)
    return None if d < 1e-9 else ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _linie(a, b):
    ""
    r = _richtung(a, b)
    if r[0] < -1e-9 or (abs(r[0]) <= 1e-9 and r[1] < 0): r = (-r[0], -r[1])
    n = (-r[1], r[0]); off = a[0] * n[0] + a[1] * n[1]
    return (round(r[0], 6), round(r[1], 6), round(off, 6)), r, a[0] * r[0] + a[1] * r[1], b[0] * r[0] + b[1] * r[1], n, off


def _octi(a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; ax, ay = abs(dx), abs(dy)
    if ax < 1e-4 or ay < 1e-4 or abs(ax - ay) < 1e-4: return [[a, b]]
    m = min(ax, ay); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    return [[a, (b[0] - sx * m, b[1] - sy * m), b], [a, (a[0] + sx * m, a[1] + sy * m), b]]


def _strecke(p, a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def aufraeumen(H, sperren=(), netze=None, hin=None, mikro=0.03, fix=()):
    sperren = set(sperren); hin = hin or Hindernisse(H); fix = set(fix)
    NK = Netzkupfer(H); stat = Counter(); remove, add = [], []
    for net in sorted({t['net'] for t in H['tracks'] if t['net']} if netze is None else set(netze)):
        if net in sperren: continue
        alt_net = NK.tracks.get(net, []); n_rm, n_add = [], []
        for L in sorted({t['layer'] for t in alt_net}):
            segs = [t for t in alt_net if t['layer'] == L]
            bogen = {K(t[e]) for t in segs if t.get('arc') or t.get('uuid') in fix for e in ('a', 'b')}
            frei_ = [t for t in segs if not t.get('arc') and t.get('uuid') not in fix and K(t['a']) not in bogen and K(t['b']) not in bogen]
            if not frei_: continue
            vias = [(Point(v['x'], v['y']), v['d'] / 2) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])]
            pads_ = [p for p in H['pads'] if p['net'] == net and p['layer'] == L and p.get('typ', 'smd') in ('smd', 'conn') and len(p['poly']) >= 3]
            padg = [Polygon(p['poly']).buffer(0) for p in pads_]
            ziele = [(c.buffer(r, 64), (c.x, c.y)) for c, r in vias] + [(g, (p['x'], p['y'])) for g, p in zip(padg, pads_) if 'x' in p]
            def an_ziel(p):
                q = Point(p)
                return any(c.distance(q) <= r for c, r in vias) or any(g.distance(q) <= 1e-9 for g in padg)

            S = {i: [tuple(t['a']), tuple(t['b']), t['w'], t['uuid']] for i, t in enumerate(frei_)}
            fest_enden = [(tuple(t[e]), t['w']) for t in segs if t not in frei_ for e in ('a', 'b')]
            def haengt_an(p, w, ausser=()):
                ""
                for j, (a, b, wj, _) in S.items():
                    if j in ausser: continue
                    for e in (a, b):
                        if 1e-9 < math.dist(e, p) <= (wj + w) / 2: return True
                return any(1e-9 < math.dist(e, p) <= (wj + w) / 2 for e, wj in fest_enden)
            naechste = [len(S)]
            def neu_id():
                naechste[0] += 1; return naechste[0]

            gesehen = {}
            for i in sorted(S):
                a, b, w, _ = S[i]
                if math.dist(a, b) < 1e-6: del S[i]; stat['ohne Länge'] += 1; continue
                key = tuple(sorted([K(a), K(b)]))
                if key in gesehen:
                    j = gesehen[key]
                    if abs(S[j][2] - w) > 1e-9:
                        stat['doppelt, Breite verschieden (bleibt)'] += 1; continue
                    del S[i]
                    stat['doppelt'] += 1
                else: gesehen[key] = i

            for gz, C in ziele:
                gi = gz.buffer(-1e-4)


                andere = [c for _, c in ziele if math.dist(c, C) > 1e-6]
                def achse(i): return LineString([S[i][0], S[i][1]]) if math.dist(S[i][0], S[i][1]) > 1e-9 else Point(S[i][0])
                innen = [i for i in sorted(S) if gi.contains(Point(S[i][0])) and gi.contains(Point(S[i][1]))
                         and not any(achse(i).distance(Point(c)) <= MITTE_EPS for c in andere)]
                if not innen: continue
                pkt_innen = {K(S[i][e]) for i in innen for e in (0, 1)}
                achsen_innen = [(LineString([S[i][0], S[i][1]]), S[i][2]) for i in innen]
                gz_ = gz.buffer(1e-6)
                ende_zahl = Counter(K(S[x][e_]) for x in S for e_ in (0, 1)) + Counter(K(p_) for p_, _ in fest_enden)


                schon_drin = {K(S[x][1 - e_]) for x in S if x not in innen for e_ in (0, 1) if K(S[x][e_]) in pkt_innen}
                schon_drin |= {K(c) for c in andere}
                aussen_an = defaultdict(list)
                herkunft = defaultdict(list)
                genau = any(K(S[x][e_]) in pkt_innen for x in S if x not in innen for e_ in (0, 1))
                for j, (a, b, w, _) in S.items():
                    if j in innen: continue
                    for e, o_ in ((a, b), (b, a)):


                        if K(e) in pkt_innen:
                            aussen_an[K(e)].append(w); herkunft[K(e)].append(o_); continue

                        if genau or K(o_) in pkt_innen or K(e) in schon_drin: continue


                        if not (gz_.contains(Point(e)) or ende_zahl[K(e)] == 1): continue
                        if (gz.distance(Point(e)) <= w / 2 + 1e-9 and any(ax.distance(Point(e)) <= (w + wi) / 2 + 1e-9 for ax, wi in achsen_innen)) or \
                                (ende_zahl[K(e)] == 1 and gz.distance(Point(e)) <= w / 2 + 1e-9):

                            aussen_an[K(e)].append(w); herkunft[K(e)].append(o_)
                zur_mitte = any(achse(i).distance(Point(C)) <= MITTE_EPS for i in innen)
                soll = sorted((E, max(ws)) for E, ws in aussen_an.items() if zur_mitte and math.dist(E, C) > 1e-6)

                wege = []
                def ohne_wende(E, c):
                    ""
                    v2 = (c[1][0] - c[0][0], c[1][1] - c[0][1])
                    return all((E[0] - o_[0]) * v2[0] + (E[1] - o_[1]) * v2[1] >= -1e-9 for o_ in herkunft[E])
                alt_achsen = unary_union([ax for ax, _ in achsen_innen])
                for E, w in soll:
                    huelle = gz.buffer(1e-6) if gz.contains(Point(E)) else gz.buffer(w / 2 + 1e-6)
                    kand = sorted(_octi(E, C), key=lambda c: LineString(c).hausdorff_distance(alt_achsen))
                    wege.append(next((c for c in kand if huelle.contains(LineString(c)) and ohne_wende(E, c)
                                      and all(hin.frei(a, b, w, L, net) for a, b in zip(c, c[1:]))), None))
                if any(c is None for c in wege): stat['Knäuel bleibt (kein Weg zur Mitte)'] += 1; continue

                def weg_r(c):
                    d_ = math.dist(c[-1], c[-2]) if len(c) > 1 else 0.0
                    return None if d_ < 1e-9 else ((c[-2][0] - c[-1][0]) / d_, (c[-2][1] - c[-1][1]) / d_)
                rs = [weg_r(c) for c in wege]
                if any(a_ and b_ and a_[0] * b_[0] + a_[1] * b_[1] > 1e-6 and abs(a_[0] * b_[1] - a_[1] * b_[0]) > 1e-6
                       for x_, a_ in enumerate(rs) for b_ in rs[x_ + 1:]):
                    stat['Knäuel bleibt (Wege liefen spitz zusammen)'] += 1; continue
                K4 = lambda p: (round(p[0], 4), round(p[1], 4))
                ist = sorted(tuple(sorted([K4(S[i][0]), K4(S[i][1])])) + (S[i][2],) for i in innen)
                if ist == sorted(tuple(sorted([K4(a), K4(b)])) + (w,) for (E, w), c in zip(soll, wege) for a, b in zip(c, c[1:])): continue
                for i in innen: del S[i]
                for (E, w), c in zip(soll, wege):
                    for a, b in zip(c, c[1:]): S[neu_id()] = [tuple(a), tuple(b), w, None]
                stat['im Via/Pad'] += len(innen); stat['davon ersetzt durch Stück zur Mitte'] += len(soll)

            gruppen = defaultdict(list)
            for i, (a, b, w, _) in S.items():
                key, r, s0, s1, n, off = _linie(a, b); gruppen[(key, round(w, 6))].append((min(s0, s1), max(s0, s1), i, r, n, off))
            for (key, w), xs in sorted(gruppen.items()):
                if len(xs) < 2: continue
                xs.sort(); laeufe = [[xs[0]]]
                for x in xs[1:]:
                    if x[0] < max(y[1] for y in laeufe[-1]) - 1e-6: laeufe[-1].append(x)
                    else: laeufe.append([x])
                for lf in laeufe:
                    if len(lf) < 2: continue
                    lo, hi = min(x[0] for x in lf), max(x[1] for x in lf); _, _, _, r, n, off = lf[0]
                    P = lambda s_: (n[0] * off + r[0] * s_, n[1] * off + r[1] * s_)
                    ids = {x[2] for x in lf}
                    if any(haengt_an(S[x[2]][e], w, ids) for x in lf for e in (0, 1) if math.dist(S[x[2]][e], P(lo)) > 1e-6 and math.dist(S[x[2]][e], P(hi)) > 1e-6):
                        stat['kollinear bleibt (Anschluss)'] += 1; continue
                    for x in lf: del S[x[2]]
                    S[neu_id()] = [P(lo), P(hi), w, None]; stat['kollinear vereinigt'] += len(lf) - 1

            for _ in range(3):
                nb = defaultdict(list)
                for i, (a, b, w, _) in S.items(): nb[K(a)].append(i); nb[K(b)].append(i)
                geaendert = False
                for i in sorted(S):
                    if i not in S: continue
                    a, b, w, _ = S[i]
                    if math.dist(a, b) >= mikro: continue
                    na = [j for j in nb[K(a)] if j != i and j in S]; nb_ = [j for j in nb[K(b)] if j != i and j in S]
                    if (not na) != (not nb_) and len(na + nb_) == 1:
                        frei_ende = a if not na else b

                        auf_achse = any(_strecke(frei_ende, S[x][0], S[x][1]) <= (S[x][2] + w) / 2 + 1e-9 for x in S if x != i) or \
                            any(_strecke(frei_ende, t['a'], t['b']) <= (t['w'] + w) / 2 + 1e-9 for t in segs if t not in frei_)
                        if not an_ziel(frei_ende) and not auf_achse:
                            del S[i]; stat['Mikrostummel'] += 1; geaendert = True
                        continue
                    if len(na) != 1 or len(nb_) != 1 or an_ziel(a) or an_ziel(b): continue
                    j, k = na[0], nb_[0]
                    if haengt_an(a, w, {i, j, k}) or haengt_an(b, w, {i, j, k}): continue
                    pa = S[j][1] if K(S[j][0]) == K(a) else S[j][0]; pb = S[k][1] if K(S[k][0]) == K(b) else S[k][0]
                    r1, r2 = _richtung(pa, a), _richtung(pb, b)
                    if r1 is None or r2 is None: continue
                    den = r1[0] * r2[1] - r1[1] * r2[0]
                    if abs(den) < 1e-9: continue
                    t_ = ((pb[0] - pa[0]) * r2[1] - (pb[1] - pa[1]) * r2[0]) / den
                    X = (pa[0] + t_ * r1[0], pa[1] + t_ * r1[1])
                    if math.dist(X, a) > mikro + w or math.dist(X, b) > mikro + w: continue
                    if (X[0] - pa[0]) * r1[0] + (X[1] - pa[1]) * r1[1] <= 1e-6 or (X[0] - pb[0]) * r2[0] + (X[1] - pb[1]) * r2[1] <= 1e-6: continue


                    if r1[0] * r2[0] + r1[1] * r2[1] > 1e-9: continue
                    if abs(r1[0] * r2[0] + r1[1] * r2[1]) <= math.sin(math.radians(0.5)) and not an_ziel(X):
                        stat['Mikrostück bleibt (sonst 90°-Ecke)'] += 1; continue
                    if not (hin.frei(pa, X, S[j][2], L, net) and hin.frei(pb, X, S[k][2], L, net)): continue
                    wj, wk = S[j][2], S[k][2]
                    del S[i], S[j], S[k]
                    S[neu_id()] = [pa, X, wj, None]; S[neu_id()] = [X, pb, wk, None]
                    stat['Mikrostück zum Achsenschnitt'] += 1; geaendert = True
                if not geaendert: break

            while True:
                nb = defaultdict(list)
                for i, (a, b, w, _) in S.items(): nb[K(a)].append(i); nb[K(b)].append(i)
                tat = False
                for p, ids in sorted(nb.items()):
                    if len(ids) != 2: continue
                    i, j = ids
                    if i not in S or j not in S or abs(S[i][2] - S[j][2]) > 1e-9: continue
                    qa = S[i][0] if K(S[i][1]) == p else S[i][1]; qb = S[j][0] if K(S[j][1]) == p else S[j][1]
                    r1, r2 = _richtung(qa, p), _richtung(p, qb)
                    if r1 is None or r2 is None or abs(r1[0] * r2[1] - r1[1] * r2[0]) > 1e-7 or r1[0] * r2[0] + r1[1] * r2[1] <= 0: continue
                    if LineString([qa, qb]).distance(Point(p)) > 1e-6: continue
                    if haengt_an(p, S[i][2], {i, j}): continue
                    w = S[i][2]; del S[i], S[j]; S[neu_id()] = [qa, qb, w, None]; stat['kollinear zusammengefasst'] += 1; tat = True
                    break
                if not tat: break
            alt_ids = {t['uuid'] for t in frei_}
            bleibt = {s[3] for s in S.values() if s[3]}
            weg = sorted(alt_ids - bleibt); neu = [dict(net=net, layer=L, a=list(s[0]), b=list(s[1]), w=s[2]) for s in S.values() if not s[3]]
            if weg or neu: n_rm += weg; n_add += neu
        if not (n_rm or n_add): continue
        if NK.trennt(net, alt_net, anwenden(alt_net, n_rm, n_add)):
            stat['verworfen (trennt Netz)'] += 1; print('verworfen, trennt das Netz:', net, file=sys.stderr); continue
        for t in n_add: hin.zusatz(net, t['layer'], t['a'], t['b'], t['w'])
        remove += n_rm; add += n_add; stat['Netze geändert'] += 1
    return dict(remove=remove, add=add, stat=dict(sorted(stat.items())))


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from nachbearbeitung.sperrnetze import sperrnetze, netz_abstaende
    netze = {t['net'] for t in H['tracks']}
    S = sperrnetze(pcb=o.get('pcb'), pro=o.get('pro'), dru=o.get('dru'), klassen=o.get('klassen'), sperren=o.get('sperren'),
                   netze=netze, pads=H.get('pads'), net_ties=H.get('net_ties'))
    dru = o.get('dru') or (os.path.splitext(o['pcb'])[0] + '.kicad_dru' if o.get('pcb') else None)
    hin = Hindernisse(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', o.get('drcpad', 0.10))),
                      loch=float(o.get('loch', 0.25)), kante=float(o.get('kante', 0.2)), netzabstand=netz_abstaende(dru, netze))
    R = aufraeumen(H, sperren=S, netze=[n for n in o['netze'].split(';') if n] if o.get('netze') else None, hin=hin,
                   mikro=float(o.get('mikro', 0.03)), fix=json.load(open(o['fix'])) if o.get('fix') else ())
    json.dump(R, open(a[1], 'w'))
    print(json.dumps(R['stat'], ensure_ascii=False), 'entfernt', len(R['remove']), 'neu', len(R['add']))


if __name__ == '__main__':
    main()
