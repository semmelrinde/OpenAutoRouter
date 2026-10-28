""
import json, math, os, sys
from collections import Counter, defaultdict

from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.kupfer_netz import Hindernisse, Netzkupfer, anwenden
from analyse.fertig_kennzahlen import ecken90_neu

S2 = math.sqrt(0.5)
RICHT = [(1, 0), (S2, S2), (0, 1), (-S2, S2), (-1, 0), (-S2, -S2), (0, -1), (S2, -S2)]


def K(p): return (round(p[0], 6), round(p[1], 6))


def _einheit(a, b):
    d = math.dist(a, b)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d) if d > 1e-9 else None


def _octi(a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; ax, ay = abs(dx), abs(dy)
    if ax < 1e-6 or ay < 1e-6 or abs(ax - ay) < 1e-6: return [[a, b]]
    m = min(ax, ay); sx, sy = math.copysign(1, dx), math.copysign(1, dy)
    return [[a, (a[0] + sx * m, a[1] + sy * m), b], [a, (b[0] - sx * m, b[1] - sy * m), b]]


def _schnitt(p, r, q, s):
    ""
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-12: return None
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
    u = ((q[0] - p[0]) * r[1] - (q[1] - p[1]) * r[0]) / den
    return t, u


def _form_lage(H, net, L, tracks):
    ""
    from analyse.fertig_kennzahlen import tor
    Hn = dict(H, tracks=[dict(t, net=net, layer=L) for t in tracks], vias=[v for v in H['vias'] if v['net'] == net],
              pads=[p for p in H['pads'] if p['net'] == net])
    return tor(Hn)['haarnadeln'], Hn


def mitte(H, sperren=(), netze=None, lagen=None, hin=None, eps=0.005, pads=True, padmax=1.6, minseg=0.03, schub=1.0, diag=None, fix=(), kurzmin=0.1,
          hmax=None):
    sperren = set(sperren); hin = hin or Hindernisse(H); fix = set(fix)
    NK = Netzkupfer(H); stat = Counter(); remove, add, stellen = [], [], []
    alle = sorted({t['net'] for t in H['tracks'] if t['net']} if netze is None else set(netze))
    for net in alle:
        if net in sperren: stat['gesperrt'] += 1; continue
        n_rm, n_add, n_st = [], [], []
        for L in sorted({t['layer'] for t in NK.tracks.get(net, []) if lagen is None or t['layer'] in lagen}):
            stand = (len(n_rm), len(n_add), len(n_st))
            segs = [t for t in NK.tracks[net] if t['layer'] == L]
            bogen = {K(t[e]) for t in segs if t.get('arc') for e in ('a', 'b')}
            gerade = [t for t in segs if not t.get('arc') and t['a'] != t['b']]
            nb = defaultdict(list)
            for t in gerade: nb[K(t['a'])].append(t); nb[K(t['b'])].append(t)
            ziele = [('via', (v['x'], v['y']), Point(v['x'], v['y']).buffer(v['d'] / 2, 32)) for v in H['vias']
                     if v['net'] == net and L in (v.get('layers') or [L])]
            if pads:
                for p in H['pads']:
                    if p['net'] != net or p['layer'] != L or p.get('typ') not in ('smd', 'conn') or 'x' not in p: continue
                    g = Polygon(p['poly']).buffer(0); x0, y0, x1, y1 = g.bounds
                    if max(x1 - x0, y1 - y0) <= padmax and g.contains(Point(p['x'], p['y'])): ziele.append(('pad', (p['x'], p['y']), g))
            fest_pkt = [g for _, _, g in ziele]


            zentren = [(v['x'], v['y']) for v in H['vias'] if v['net'] == net and L in (v.get('layers') or [L])] + \
                      [(p['x'], p['y']) for p in H['pads'] if p['net'] == net and p['layer'] == L and p.get('typ') in ('smd', 'conn') and 'x' in p
                       and len(p.get('poly') or []) >= 3 and Polygon(p['poly']).buffer(0).contains(Point(p['x'], p['y']))]
            def beruehrt(linien):
                return {i for i, g in enumerate(kupfer) if any(l.intersects(g) for l in linien)}
            def erreicht(linien):
                return {c for c in zentren if any(g.distance(Point(c)) <= 0.005 for g in linien)}
            kupfer = [g for art, _, g in ziele if art == 'via'] + [Polygon(p['poly']).buffer(0) for p in H['pads']
                                                                 if p['net'] == net and p['layer'] == L and len(p.get('poly') or []) >= 3]
            benutzt = {t['uuid'] for t in gerade if t.get('uuid') in fix}
            for E in sorted(nb):
                wE = min(t['w'] for t in nb[E])

                ziel = next(((art, c, g) for art, c, g in ziele if g.distance(Point(E)) <= (wE / 2 if len(nb[E]) == 1 else 1e-9)), None)
                if ziel is None: continue
                art, C, gz = ziel
                if math.dist(E, C) <= eps: continue
                stat['Enden neben der Mitte (%s)' % art] += 1
                for s in sorted(nb[E], key=lambda t: t['uuid']):
                    if s['uuid'] in benutzt or E in bogen: continue
                    Sx = tuple(s['b']) if K(s['a']) == E else tuple(s['a'])
                    if gz.distance(Point(Sx)) <= 1e-9: stat['Stück ganz im Ziel'] += 1; continue

                    an_S = [t for t in nb[K(Sx)] if t is not s]
                    prev = an_S[0] if (len(an_S) == 1 and an_S[0]['uuid'] not in benutzt and K(Sx) not in bogen
                                       and not any(g.distance(Point(Sx)) <= 1e-9 for g in fest_pkt)) else None
                    P = None if prev is None else (tuple(prev['b']) if K(prev['a']) == K(Sx) else tuple(prev['a']))


                    ersetzt = [s] + ([prev] if prev else [])
                    achsen = [LineString([t['a'], t['b']]) for t in ersetzt]
                    eigene = {K(E), K(Sx)} | ({K(P)} if P else set())
                    kontakte = []
                    for t in gerade:
                        if t in ersetzt: continue
                        for e in ('a', 'b'):
                            q = K(t[e])
                            if q in eigene or gz.distance(Point(q)) <= 1e-9: continue
                            if any(ax.distance(Point(q)) <= (t['w'] + u['w']) / 2 + 1e-6 for ax, u in zip(achsen, ersetzt)):

                                kontakte.append((Point(q), t['w'] / 2, any(ax.distance(Point(q)) <= 1e-3 for ax in achsen)))
                    for g in kupfer:
                        if g is gz or g.distance(Point(E)) <= 1e-9: continue
                        if any(ax.distance(g) <= u['w'] / 2 + 1e-6 for ax, u in zip(achsen, ersetzt)): kontakte.append((g, 0.0, False))
                    kand = []
                    d0 = _einheit(Sx, E)

                    if d0 and abs((C[0] - Sx[0]) * d0[1] - (C[1] - Sx[1]) * d0[0]) < 1e-5 and (C[0] - Sx[0]) * d0[0] + (C[1] - Sx[1]) * d0[1] > minseg:
                        kand.append(('A', [Sx, C], [s['w']], [s]))
                    if prev is not None:
                        rp = (Sx[0] - P[0], Sx[1] - P[1]); lp = math.hypot(*rp)
                        dp = (rp[0] / lp, rp[1] / lp) if lp > 1e-9 else None
                        opts = []
                        for d in RICHT:
                            if dp is None or dp[0] * d[0] + dp[1] * d[1] < -1e-9: continue
                            if abs(dp[0] * d[1] - dp[1] * d[0]) < 1e-9: continue
                            tu = _schnitt(P, rp, C, (-d[0], -d[1]))
                            if tu is None: continue
                            t_, u_ = tu; S1 = (P[0] + t_ * rp[0], P[1] + t_ * rp[1])
                            if t_ * lp < minseg or u_ < minseg or math.dist(S1, Sx) > schub: continue
                            gleich = d0 is not None and abs(d[0] - d0[0]) < 1e-6 and abs(d[1] - d0[1]) < 1e-6
                            opts.append((0 if gleich else 1, t_ * lp + u_, d, S1))
                        for _, _, d, S1 in sorted(opts):
                            kand.append(('B', [P, S1, C], [prev['w'], s['w']], [prev, s]))


                        if abs(prev['w'] - s['w']) < 1e-9:
                            alt_d = LineString([P, Sx, E])
                            for c in sorted(_octi(P, C), key=lambda c: LineString(c).hausdorff_distance(alt_d)):
                                if LineString(c).hausdorff_distance(alt_d) > 0.3: continue
                                kand.append(('D', c, [s['w']] * (len(c) - 1), [prev, s]))



                    s_mitte = any(math.dist(Sx, c2) <= eps for _, c2, _ in ziele)
                    e_innen = any(t is not s and gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9 for t in nb[E])
                    if prev is None and s_mitte and e_innen:
                        stat['Knick im Ziel, beide Enden in Zielen'] += 1; continue
                    if prev is None:
                        alt_l = math.dist(Sx, E)
                        alt_ls = LineString([Sx, E])
                        for c in sorted(_octi(Sx, C), key=lambda c: LineString(c).hausdorff_distance(alt_ls)):
                            if len(c) == 3 and min(math.dist(c[0], c[1]), math.dist(c[1], c[2])) < minseg: continue
                            if LineString(c).length > alt_l + 0.3: continue
                            w1 = _einheit(c[0], c[1])
                            if w1 and any((lambda o: o and o[0] * w1[0] + o[1] * w1[1] > 1e-9)(_einheit(Sx, tuple(t['b']) if K(t['a']) == K(Sx) else tuple(t['a'])))
                                          for t in an_S):
                                continue
                            kand.append(('C', c, [s['w']] * (len(c) - 1), [s]))
                    def haelt(pts, ws):
                        neu_ax = [(LineString([a, b]), w) for (a, b), w in zip(zip(pts, pts[1:]), ws)]
                        return all(any(ax.distance(g) <= (1e-3 if genau else h + w / 2 + 1e-6) for ax, w in neu_ax) for g, h, genau in kontakte)
                    def sichtbar_kurz(a, b):
                        g = LineString([a, b]) if a != b else Point(a)
                        return math.dist(a, b) < kurzmin and not any(z.buffer(1e-6).contains(g) for z in kupfer)
                    def form(pts_, alt):
                        ""
                        au = {t['uuid'] for t in alt}
                        nb0 = [tuple(t['b']) if K(t['a']) == K(pts_[0]) else tuple(t['a']) for t in gerade
                               if t['uuid'] not in au and K(pts_[0]) in (K(t['a']), K(t['b']))]


                        nbe = [tuple(t['b']) if K(t['a']) == K(pts_[-1]) else tuple(t['a']) for t in gerade
                               if t['uuid'] not in au and K(pts_[-1]) in (K(t['a']), K(t['b'])) and not (len(pts_) > 1 and K(pts_[-1]) == K(pts_[0]))
                               and not (rest_faellt(au) and E in (K(t['a']), K(t['b'])) and gz.distance(Point(t['a'])) <= 1e-9
                                        and gz.distance(Point(t['b'])) <= 1e-9)]

                        q = ([nb0[0]] if len(nb0) == 1 else []) + list(pts_) + ([nbe[0]] if len(nbe) == 1 else [])
                        ecken = nadeln = 0
                        if len(nb0) > 1:
                            v2 = _einheit(pts_[0], pts_[1])
                            nadeln += sum(1 for o_ in nb0 if v2 and _einheit(pts_[0], o_) and
                                          _einheit(pts_[0], o_)[0] * v2[0] + _einheit(pts_[0], o_)[1] * v2[1] > 1e-6)
                        for a, b, c in zip(q, q[1:], q[2:]):
                            v1, v2 = _einheit(b, a), _einheit(b, c)
                            if not v1 or not v2: continue
                            cs = v1[0] * v2[0] + v1[1] * v2[1]
                            nadeln += cs > 1e-6
                            if not any(z.distance(Point(b)) <= 1e-9 for z in kupfer): ecken += abs(cs) <= math.sin(math.radians(0.5))
                        return ecken, nadeln, frozenset((K(a), K(b)) if K(a) <= K(b) else (K(b), K(a)) for a, b in zip(pts_, pts_[1:]) if sichtbar_kurz(a, b))
                    def rest_faellt(au):
                        ""
                        return all(t['uuid'] in au or t['uuid'] in benutzt for t in nb[E]
                                   if not (gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9))
                    alt_form = {}
                    for typ, pts, ws, alt in kand:


                        ka = tuple(t['uuid'] for t in alt)
                        if ka not in alt_form:
                            alt_form[ka] = form([P, Sx, E] if len(alt) == 2 else [Sx, E], alt)
                        h_ = (hmax or {}).get((net, L), 0.0)
                        if h_ and LineString(pts).hausdorff_distance(LineString([P, Sx, E] if len(alt) == 2 else [Sx, E])) > h_:
                            stat['Anflug verworfen (hmax)'] += 1; continue
                        f_ = form(pts, alt)
                        if f_[0] > alt_form[ka][0] or f_[1] > alt_form[ka][1] or f_[2] - alt_form[ka][2]:
                            stat['Form schlechter (90°, Haarnadel, Kurzstück)'] += 1; continue
                        if not haelt(pts, ws): stat['Kontakt am Anflug ginge verloren'] += 1; continue
                        if erreicht([LineString([t['a'], t['b']]) for t in alt]) - erreicht([LineString([a, b]) for a, b in zip(pts, pts[1:])]):
                            stat['andere Mitte ginge verloren'] += 1; continue


                        if beruehrt([LineString([t['a'], t['b']]) for t in alt]) - beruehrt([LineString([a, b]) for a, b in zip(pts, pts[1:]) if a != b]):
                            stat['Pad/Via ginge verloren'] += 1; continue
                        if all(hin.frei(a, b, w, L, net) for (a, b), w in zip(zip(pts, pts[1:]), ws)):
                            for t in alt: benutzt.add(t['uuid'])
                            n_rm += [t['uuid'] for t in alt]
                            neu = [dict(net=net, layer=L, a=list(a), b=list(b), w=w) for (a, b), w in zip(zip(pts, pts[1:]), ws)]
                            n_add += neu
                            n_st.append(dict(net=net, layer=L, art=art, typ=typ, mitte=list(C), ende=list(E), versatz=round(math.dist(E, C), 4)))
                            break
                    else:
                        stat['kein freier Anflug'] += 1
                        if diag is not None: diag.append(dict(net=net, layer=L, art=art, ende=list(E), mitte=list(C), versatz=round(math.dist(E, C), 4),
                                                              prev=prev is not None, kandidaten=[k[0] for k in kand], laenge_s=round(math.dist(Sx, E), 4)))

                innen = [t for t in nb[E] if gz.distance(Point(t['a'])) <= 1e-9 and gz.distance(Point(t['b'])) <= 1e-9]
                aussen = [t for t in nb[E] if t not in innen]


                if innen and aussen and all(t['uuid'] in benutzt and t['uuid'] not in fix for t in aussen) and \
                        any(LineString([a_['a'], a_['b']]).distance(Point(C)) <= 0.005 for a_ in n_add if a_['a'] != a_['b'] and a_['layer'] == L):
                    for t in innen:
                        if t['uuid'] not in benutzt:
                            benutzt.add(t['uuid']); n_rm.append(t['uuid']); stat['Reststück im Ziel entfernt'] += 1


            if len(n_rm) > stand[0] or len(n_add) > stand[1]:
                rm_L, add_L = n_rm[stand[0]:], n_add[stand[1]:]
                alt_L = [t for t in segs]; neu_L = anwenden(alt_L, rm_L, add_L)
                (n1, H1_), (n0, H0_) = _form_lage(H, net, L, neu_L), _form_lage(H, net, L, alt_L)
                if n1 > n0 or ecken90_neu(H0_, H1_):
                    stat['Lage verworfen (Haarnadel/90° in der Gegenprobe)'] += 1
                    del n_rm[stand[0]:]; del n_add[stand[1]:]; del n_st[stand[2]:]
        if not n_rm: continue
        alt = NK.tracks.get(net, [])
        if NK.trennt(net, alt, anwenden(alt, n_rm, n_add)):
            stat['verworfen (trennt Netz)'] += 1; print('verworfen, trennt das Netz:', net, file=sys.stderr); continue
        for t in n_add: hin.zusatz(net, t['layer'], t['a'], t['b'], t['w'])
        remove += n_rm; add += n_add; stellen += n_st
        for s_ in n_st: stat['in die Mitte (%s, %s)' % (s_['art'], s_['typ'])] += 1
    return dict(remove=remove, add=add, stellen=stellen, stat=dict(sorted(stat.items())))

def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from nachbearbeitung.sperrnetze import sperrnetze, netz_abstaende
    netze = {t['net'] for t in H['tracks']}
    S = sperrnetze(pcb=o.get('pcb'), pro=o.get('pro'), dru=o.get('dru'), klassen=o.get('klassen'), sperren=o.get('sperren'),
                   netze=netze, pads=H.get('pads'), net_ties=H.get('net_ties'))
    dru = o.get('dru') or (os.path.splitext(o['pcb'])[0] + '.kicad_dru' if o.get('pcb') else None)
    hin = Hindernisse(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', o.get('drcpad', 0.10))),
                      drcvia=float(o.get('drcvia', o.get('drc', 0.09))), loch=float(o.get('loch', 0.25)), kante=float(o.get('kante', 0.2)),
                      netzabstand=netz_abstaende(dru, netze))
    R = mitte(H, sperren=S, netze=[n for n in o['netze'].split(';') if n] if o.get('netze') else None,
              lagen=o['lagen'].split(',') if o.get('lagen') else None, hin=hin, eps=float(o.get('eps', 0.005)),
              pads=o.get('pads', '1') == '1', padmax=float(o.get('padmax', 1.6)), minseg=float(o.get('minseg', 0.03)),
              schub=float(o.get('schub', 1.0)), fix=json.load(open(o['fix'])) if o.get('fix') else (), kurzmin=float(o.get('kurzmin', 0.1)))
    json.dump(R, open(a[1], 'w'))
    print(json.dumps(R['stat'], ensure_ascii=False), 'entfernt', len(R['remove']), 'neu', len(R['add']))


if __name__ == '__main__':
    main()
