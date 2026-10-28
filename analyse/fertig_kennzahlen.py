""
import json, math, os, subprocess, sys, tempfile
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from analyse import r18 as R18, wackel as WK


def _routerformat(H, ohne=()):
    R = dict(tracks=[dict(net=t['net'], layer=t['layer'], pts=[t['a'], t['b']], w=[t['w']]) for t in H['tracks'] if t['net'] not in ohne],
             vias=[dict(net=v['net'], x=v['x'], y=v['y']) for v in H['vias'] if v['net'] not in ohne])
    B = dict(pads=[dict(net=p['net'], x=p.get('x', Polygon(p['poly']).centroid.x), y=p.get('y', Polygon(p['poly']).centroid.y))
                   for p in H['pads'] if p['net'] not in ohne and len(p.get('poly') or []) >= 3])
    return B, R


def _treppen(H, ohne=()):
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, 'h.json')
        json.dump(dict(tracks=[t for t in H['tracks'] if t['net'] not in ohne], vias=[v for v in H['vias'] if v['net'] not in ohne]), open(f, 'w'))
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'treppen.py'), f, 'stueck=0.6', 'liste=0'],
                           capture_output=True, text=True)
    z = r.stdout.split('\n')[0].replace(':', ' ').replace(',', ' ').split()
    try: return dict(treppen=int(z[1]), netze=int(z[3]), knicke=int(z[5]))
    except (IndexError, ValueError): return dict(fehler=r.stdout[-200:] + r.stderr[-200:])


def _ziele(H, ohne=()):
    ""
    ohne = set(ohne); out = []
    lagen_netz = defaultdict(set)
    for t in H['tracks']: lagen_netz[t['net']].add(t['layer'])
    for v in H['vias']:
        if v['net'] in ohne: continue
        c = (round(v['x'], 6), round(v['y'], 6)); g = Point(v['x'], v['y']).buffer(v['d'] / 2, 32)
        for L in (v.get('layers') or sorted(lagen_netz[v['net']])): out.append((v['net'], L, 'via', c, g))
    for p in H['pads']:
        if p['net'] in ohne or p.get('typ') not in ('smd', 'conn') or 'x' not in p or len(p.get('poly') or []) < 3: continue
        g = Polygon(p['poly']).buffer(0)
        if g.contains(Point(p['x'], p['y'])): out.append((p['net'], p['layer'], 'pad', (round(p['x'], 6), round(p['y'], 6)), g))
    return out


def mitten(H, ohne=(), eps=0.005):
    ""
    ohne = set(ohne); achsen = defaultdict(list); out = set()
    for t in H['tracks']:
        if t['net'] not in ohne: achsen[(t['net'], t['layer'])].append(t)
    def erreicht(n, L, c):
        for t in achsen.get((n, L), ()):
            a, b = t['a'], t['b']
            if min(a[0], b[0]) - eps <= c[0] <= max(a[0], b[0]) + eps and min(a[1], b[1]) - eps <= c[1] <= max(a[1], b[1]) + eps:
                if (LineString([a, b]) if a != b else Point(a)).distance(Point(c)) <= eps: return True
        return False
    for n, L, art, c, _ in _ziele(H, ohne):
        if erreicht(n, L, c): out.add((n, L, art, c))
    return out


def mitte_verlust(H0, H1, ohne=(), eps=0.005):
    ""
    weg = mitten(H0, ohne) - mitten(H1, ohne)
    if not weg: return {}
    geo = {(n, L, art, c): g for n, L, art, c, g in _ziele(H1, ohne)}
    zentren = defaultdict(set)
    for n, L, art, c in geo: zentren[(n, L)].add(c)
    enden = defaultdict(list)
    for t in H1['tracks']:
        for q in (t['a'], t['b']): enden[(t['net'], t['layer'])].append((tuple(q), t['w']))
    out = defaultdict(int)
    for k in sorted(weg):
        n, L, art, c = k; g = geo.get(k)
        if g is None: continue
        for q, w in enden.get((n, L), ()):
            if g.distance(Point(q)) <= w / 2 + 1e-9 and all(math.dist(q, z) > eps for z in zentren[(n, L)]):
                out[n] += 1; break
    return dict(sorted(out.items()))


def enden_fehler(H, fuell=None, ohne=(), netze=None, nah=0.05):
    ""
    ohne = set(ohne); netze = None if netze is None else set(netze)
    Q = lambda p: (round(p[0] * 1000), round(p[1] * 1000))
    T = defaultdict(list); gesehen = set()
    for t in H['tracks']:
        if t['net'] in ohne or (netze is not None and t['net'] not in netze): continue
        k = (t['net'], t['layer']) + tuple(sorted([Q(t['a']), Q(t['b'])]))
        if k in gesehen: continue
        gesehen.add(k); T[(t['net'], t['layer'])].append(t)
    vias = defaultdict(list)
    for v in H['vias']: vias[v['net']].append(v)
    pads = defaultdict(list)
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    zonen = defaultdict(list)
    for f in fuell or ():
        zonen[(f['net'], f['layer'])].append(f['poly'])
    out = {}
    for (n, L), ts in sorted(T.items()):
        knoten = defaultdict(list)
        for t in ts:
            for e in ('a', 'b'): knoten[Q(t[e])].append(t)
        for k, an in sorted(knoten.items()):
            if len(an) != 1: continue
            x, y = k[0] / 1000.0, k[1] / 1000.0; q = Point(x, y)
            if any(math.dist((v['x'], v['y']), (x, y)) <= v['d'] / 2 + 1e-4 for v in vias.get(n, ()) if not v.get('layers') or L in v['layers']):
                continue
            if any(g.distance(q) <= 1e-6 for g in pads.get((n, L), ())): continue
            t0 = an[0]; auf_achse = False
            for t in ts:
                if t is t0 or t.get('arc'): continue
                a, b = t['a'], t['b']; dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
                if ll < 1e-12: continue
                s = ((x - a[0]) * dx + (y - a[1]) * dy) / ll
                if 0.0 < s < 1.0 and abs((x - a[0]) * dy - (y - a[1]) * dx) / math.sqrt(ll) <= 1e-3: auf_achse = True; break
            if auf_achse: continue
            naechst = min((math.dist((x, y), (k2[0] / 1000.0, k2[1] / 1000.0)) for k2 in knoten if k2 != k), default=9.0)
            if naechst < nah: out[(n, L, k)] = 'versatz'
            elif any(g.distance(q) <= 1e-6 for g in zonen.get((n, L), ())): continue
            else: out[(n, L, k)] = 'lose'
    return out


def neue_orte(alt, neu, tol=0.3):
    ""
    g = defaultdict(list)
    for k in alt: g[(k[0], k[1])].append(k[2])
    frei = {key: list(v) for key, v in g.items()}
    out = []
    for k in sorted(neu):
        kand = frei.get((k[0], k[1]), [])
        best = min(kand, key=lambda q: math.dist(q, k[2]), default=None)
        if best is not None and math.dist(best, k[2]) <= tol * 1000:
            kand.remove(best); continue
        out.append(k)
    return out


def enden_neu(H0, H1, fuell0=None, fuell1=None, netze=None, tol=0.3):
    ""
    out = defaultdict(int)
    for k in neue_orte(enden_fehler(H0, fuell0, netze=netze), enden_fehler(H1, fuell1, netze=netze), tol): out[k[0]] += 1
    return dict(sorted(out.items()))


def ecken90_neu(H0, H1, netze=None, tol=0.3):
    ""
    out = defaultdict(int)
    for k in neue_orte(ecken90_orte(H0, netze=netze), ecken90_orte(H1, netze=netze), tol): out[(k[0], k[1])] += 1
    return dict(out)


def ziel_verlassen(H0, H1, netze=None):
    ""
    netze = None if netze is None else set(netze)
    def ziele(H):
        out = []
        for v in H['vias']:
            if netze is not None and v['net'] not in netze: continue
            for L in (v.get('layers') or sorted({t['layer'] for t in H['tracks'] if t['net'] == v['net']})):
                out.append((v['net'], L, ('V', round(v['x'], 6), round(v['y'], 6)), Point(v['x'], v['y']).buffer(v['d'] / 2, 32)))
        for p in H['pads']:
            if (netze is not None and p['net'] not in netze) or len(p.get('poly') or []) < 3: continue
            out.append((p['net'], p['layer'], ('P', p.get('ref'), p.get('num'), round(p.get('x', 0), 6), round(p.get('y', 0), 6)),
                        Polygon(p['poly']).buffer(0)))
        return out
    def stand(H):
        T = defaultdict(list)
        for t in H['tracks']:
            if netze is None or t['net'] in netze: T[(t['net'], t['layer'])].append(t)
        drin, beruehrt = set(), set()
        for n, L, kz, g in ziele(H):
            ts = T.get((n, L), ())


            if any(g.distance(Point(t[e])) <= 1e-6 for t in ts for e in ('a', 'b')) or \
                    any(t['a'] != t['b'] and LineString([t['a'], t['b']]).intersects(g) for t in ts): drin.add((n, L, kz))
            elif any((LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a'])).distance(g) <= t['w'] / 2 + 1e-6 for t in ts):
                beruehrt.add((n, L, kz))
        return drin, beruehrt
    d0, _ = stand(H0); d1, b1 = stand(H1)
    out = defaultdict(int)
    for k in sorted((d0 - d1) & b1): out[k[0]] += 1
    return dict(sorted(out.items()))


def ecken90(H, ohne=(), netze=None):
    ""
    out = defaultdict(int)
    for k in ecken90_orte(H, ohne, netze): out[(k[0], k[1])] += 1
    return dict(out)


def ecken90_orte(H, ohne=(), netze=None):
    ""
    ohne = set(ohne); netze = None if netze is None else set(netze)
    Q = lambda p: (round(p[0] * 1000), round(p[1] * 1000))
    kn = defaultdict(dict)
    for t in H['tracks']:
        if t['net'] in ohne or (netze is not None and t['net'] not in netze) or t.get('arc'): continue
        a, b = Q(t['a']), Q(t['b'])
        if a == b: continue
        k = tuple(sorted([a, b]))
        kn[(t['net'], t['layer'], a)][k] = t; kn[(t['net'], t['layer'], b)][k] = t
    vias = defaultdict(list); pads = defaultdict(list)
    for v in H['vias']: vias[v['net']].append(v)
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    out = set()
    for (n, L, q), an in kn.items():
        if len(an) != 2: continue
        x, y = q[0] / 1000.0, q[1] / 1000.0
        r = []
        for t in an.values():
            o = t['b'] if Q(t['a']) == q else t['a']; d = math.dist((x, y), o)
            r.append(((o[0] - x) / d, (o[1] - y) / d))
        c = max(-1.0, min(1.0, r[0][0] * r[1][0] + r[0][1] * r[1][1]))
        if abs(math.degrees(math.acos(c)) - 90.0) > 0.5: continue
        if any(math.dist((v['x'], v['y']), (x, y)) <= v['d'] / 2 + 1e-4 for v in vias.get(n, ()) if not v.get('layers') or L in v['layers']):
            continue
        if any(g.distance(Point(x, y)) <= 1e-6 for g in pads.get((n, L), ())): continue
        out.add((n, L, q))
    return out


def laengen(H):
    ""
    out = defaultdict(float)
    for t in H['tracks']: out[t['net']] += math.dist(t['a'], t['b'])
    return dict(out)


def laengen_verstoss(L0, H1, netze, prozent=2.0, mm=1.0):
    ""
    L1 = laengen(H1); out = {}
    for n in sorted(netze):
        a, b = L0.get(n, 0.0), L1.get(n, 0.0)
        if abs(a - b) > min(mm, prozent / 100.0 * a) + 1e-9: out[n] = (round(a, 4), round(b, 4))
    return out


def tor_je_netz(H, ohne=()):
    ""
    netze = sorted({t['net'] for t in H['tracks'] if t['net'] not in set(ohne)})
    out = {}
    for n in netze:
        Hn = dict(H, tracks=[t for t in H['tracks'] if t['net'] == n], vias=[v for v in H['vias'] if v['net'] == n],
                  pads=[p for p in H['pads'] if p['net'] == n])
        t = tor(Hn)
        if any(t.values()): out[n] = (t['winkel'], t['r32'], t['haarnadeln'])
    return out


def tor(H, ohne=()):
    ""
    ohne = set(ohne); winkel = 0; nb = defaultdict(list)
    K = lambda p: (round(p[0], 6), round(p[1], 6))
    for t in H['tracks']:
        if t['net'] in ohne: continue
        a, b = tuple(t['a']), tuple(t['b'])
        if not t.get('arc') and math.dist(a, b) > 0.02:
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
            winkel += min(ang, 45 - ang) > 0.5
        for q in (a, b): nb[(t['net'], t['layer'], K(q))].append(t['w'])
    ziel = defaultdict(list)
    for v in H['vias']: ziel[v['net']].append(Point(v['x'], v['y']).buffer(v['d'] / 2, 16))
    for p in H['pads']:
        if len(p.get('poly') or []) >= 3: ziel[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    r32 = 0
    for (n, L, q), ws in nb.items():
        if len(ws) == 2 and abs(ws[0] - ws[1]) > 1e-6 and not any(g.distance(Point(q)) <= 1e-9 for g in ziel.get(n, []) + ziel.get((n, L), [])):
            r32 += 1
    B, R = _routerformat(H, ohne)
    return dict(winkel=winkel, r32=r32, haarnadeln=R18.messen(B, R)['alle']['haarnadeln'])


def messen(H, ohne=()):
    ohne = set(ohne)
    T = [t for t in H['tracks'] if t['net'] not in ohne]
    vias = defaultdict(list)
    for v in H['vias']:
        if v['net'] not in ohne: vias[v['net']].append((v['x'], v['y'], v['d'] / 2, set(v.get('layers') or [])))
    pads = defaultdict(list)
    for p in H['pads']:
        if p['net'] in ohne or p.get('typ') not in ('smd', 'conn') or 'x' not in p or len(p['poly']) < 3: continue
        g = Polygon(p['poly']).buffer(0)
        if g.contains(Point(p['x'], p['y'])): pads[(p['net'], p['layer'])].append((g, (p['x'], p['y'])))
    grad = defaultdict(int)
    for t in T:
        for q in (t['a'], t['b']): grad[(t['net'], t['layer'], round(q[0], 6), round(q[1], 6))] += 1
    def im(t, q):
        ""
        tol = t['w'] / 2 if grad[(t['net'], t['layer'], round(q[0], 6), round(q[1], 6))] == 1 else 0.0
        for x, y, r, Ls in vias.get(t['net'], []):
            if (not Ls or t['layer'] in Ls) and math.dist(q, (x, y)) <= r + tol: return ('via', (x, y), r)
        for g, c in pads.get((t['net'], t['layer']), []):
            if g.distance(Point(q)) <= tol + 1e-9: return ('pad', c, g)
        return None
    mikro = dict(unter_0_08=0, unter_0_03=0, sichtbar_unter_0_08=0); r37 = defaultdict(int); winkel = 0; L = 0.0
    for t in T:
        a, b = tuple(t['a']), tuple(t['b']); l = math.dist(a, b); L += l
        za, zb = im(t, a), im(t, b)
        ganz_innen = za is not None and zb is not None and za[1] == zb[1]
        if l < 0.08 and not t.get('arc'):
            mikro['unter_0_08'] += 1; mikro['unter_0_03'] += l < 0.03; mikro['sichtbar_unter_0_08'] += not ganz_innen
        if not t.get('arc') and l > 0.02:
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
            winkel += min(ang, 45 - ang) > 0.5
        if ganz_innen: continue
        for q, z in ((a, za), (b, zb)):
            if z is None: continue
            d = math.dist(q, z[1])
            if d > 0.005: r37['%s_neben_mitte' % z[0]] += 1
            if d > 0.05: r37['%s_ueber_0_05' % z[0]] += 1
    B, R = _routerformat(H, ohne)
    w = WK.messen(B, R); r = R18.messen(B, R)['alle']
    return dict(laenge_mm=round(L, 2), segmente=len(T), vias=sum(len(v) for v in vias.values()),
                wackel=dict(zuege=w['zuege'], zuege_je_10mm=w['zuege_je_10mm'], umweg=w['umweg']),
                r18=dict(knicke=r['knicke'], haarnadeln=r['haarnadeln'], zickzack=r['zickzack'], knick_ueberschuss=r['knick_ueberschuss'],
                         umweg_octi_mm=round(r['umweg_octi_mm'], 2), knicke_je_10mm=round(r['knicke_je_10mm'], 3)),
                treppen=_treppen(H, ohne), mikro=mikro, r37=dict(sorted(r37.items())), winkel_ausserhalb=winkel,
                r32_breitenwechsel=tor(H, ohne)['r32'])


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[1:] if '=' in x)
    H = json.load(open(a[0]))
    ohne = [n for n in o.get('ohne', '').split(';') if n]
    if o.get('ohne_json'): ohne += list(json.load(open(o['ohne_json'])))
    out = dict(alle=messen(H))
    if ohne: out['ohne_gesperrte'] = messen(H, ohne)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    if o.get('json'): json.dump(out, open(o['json'], 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
