""
import collections, math

MIKRO = 0.05


def _k(p):
    return (round(p[0], 4), round(p[1], 4))


def nicht_naeher(hin, e, a1, a2):
    ""
    from shapely.geometry import LineString
    L, net, w = e['L'], e['net'], e['w']
    neu = LineString([a1, a2]); alt = LineString([a1, e['p'], a2])
    eigen = hin.netzabstand.get(net, 0.0)
    for i in hin.baum[L].query(neu.buffer(w / 2 + hin.dmax + hin.res)) if L in hin.baum else []:
        n, g, d, bahn = hin.obj[L][int(i)]
        if n == net: continue
        dd = max(d, eigen if bahn else 0.0, hin.netzabstand.get(n, 0.0) if bahn else 0.0)
        x = neu.distance(g)
        if x < w / 2 + dd + hin.res and x < alt.distance(g) - 1e-6: return False
    for n, g in hin.neu.get(L, []):
        if n != net and neu.distance(g) < w / 2 + max(hin.drc, eigen, hin.netzabstand.get(n, 0.0)) + hin.res: return False
    return True


def ecken45(H, sperren=(), netze=None, fix=(), hin=None, fuell=None, cmax=0.4, cmin=0.08):
    ""
    from shapely.geometry import Point, Polygon
    S = set(sperren); F = set(fix); nur = None if netze is None else set(netze)
    ende = collections.defaultdict(list)
    for t in H['tracks']:
        if t.get('arc') or t['net'] in S or not t['net'] or t['uuid'] in F or (nur is not None and t['net'] not in nur): continue
        if math.dist(t['a'], t['b']) < 1e-6: continue
        key = (t['net'], t['layer'], round(t['w'], 6))
        ende[(key, _k(t['a']))].append((t, 'a')); ende[(key, _k(t['b']))].append((t, 'b'))
    alle_enden = collections.Counter()
    for t in H['tracks']:
        for q in (t['a'], t['b']): alle_enden[(t['net'], t['layer'], _k(q))] += 1
    pads = collections.defaultdict(list)
    for p in H.get('pads', []):
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(Polygon(p['poly']).buffer(0))
    vias = collections.defaultdict(list)
    for v in H.get('vias', []): vias[v['net']].append((v['x'], v['y'], v['d'] / 2, v.get('layers')))
    zonen = collections.defaultdict(list)
    for n, L, g in fuell or []: zonen[(n, L)].append(g)
    stat = collections.Counter(); ecken = []
    for (key, p), lst in sorted(ende.items(), key=lambda x: (x[0][0], x[0][1])):
        net, L, w = key
        if len(lst) != 2 or alle_enden[(net, L, p)] != 2: continue
        (t1, e1), (t2, e2) = lst
        o1 = t1['b'] if e1 == 'a' else t1['a']; o2 = t2['b'] if e2 == 'a' else t2['a']
        v1 = (o1[0] - p[0], o1[1] - p[1]); v2 = (o2[0] - p[0], o2[1] - p[1]); l1, l2 = math.hypot(*v1), math.hypot(*v2)
        cw = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / l1 / l2))
        if abs(math.degrees(math.acos(cw)) - 90) > 1: continue
        P = Point(p)
        if any(g.covers(P) for g in pads[(net, L)]): stat['im Pad'] += 1; continue
        if any(math.dist((x, y), p) <= r + 1e-6 for x, y, r, Ls in vias[net] if not Ls or L in Ls): stat['im Via'] += 1; continue
        if any(g.buffer(-w / 2).covers(P) for g in zonen[(net, L)]): stat['in eigener Zone'] += 1; continue
        ecken.append(dict(net=net, L=L, w=w, p=p, t1=t1, e1=e1, t2=t2, e2=e2, u1=(v1[0] / l1, v1[1] / l1), u2=(v2[0] / l2, v2[1] / l2),
                          l1=l1, l2=l2))

    je_bahn = collections.Counter()
    for e in ecken: je_bahn[e['t1']['uuid']] += 1; je_bahn[e['t2']['uuid']] += 1
    def grenze(l, n): return 0.45 * l if n == 1 else min(0.45 * l, (l - MIKRO) / 2)
    neu_ende = {}; diag = []; bahnen = {}
    for e in ecken:
        c0 = min(cmax, grenze(e['l1'], je_bahn[e['t1']['uuid']]), grenze(e['l2'], je_bahn[e['t2']['uuid']]))
        if c0 < cmin: stat['Schenkel zu kurz'] += 1; continue
        for c in (c0, 0.75 * c0, 0.5 * c0):
            if c < cmin: c = None; break
            a1 = (e['p'][0] + e['u1'][0] * c, e['p'][1] + e['u1'][1] * c); a2 = (e['p'][0] + e['u2'][0] * c, e['p'][1] + e['u2'][1] * c)
            if hin is None or hin.frei(a1, a2, e['w'], e['L'], e['net']) or nicht_naeher(hin, e, a1, a2): break
        else:
            c = None
        if c is None: stat['Diagonale nicht frei'] += 1; continue
        if c < c0: stat['kleinere Fase'] += 1
        for t, w_, q in ((e['t1'], e['e1'], a1), (e['t2'], e['e2'], a2)):
            bahnen[t['uuid']] = t; neu_ende.setdefault(t['uuid'], {})[w_] = [round(q[0], 6), round(q[1], 6)]
        diag.append(dict(net=e['net'], layer=e['L'], a=[round(a1[0], 6), round(a1[1], 6)], b=[round(a2[0], 6), round(a2[1], 6)], w=e['w']))
        stat['angefast'] += 1
        if hin is not None: hin.zusatz(e['net'], e['L'], a1, a2, e['w'])
    add = []
    for u in sorted(bahnen):
        t = bahnen[u]; ne = neu_ende[u]
        add.append(dict(net=t['net'], layer=t['layer'], a=ne.get('a', t['a']), b=ne.get('b', t['b']), w=t['w']))
    return dict(remove=sorted(bahnen), add=add + diag, stat=dict(ecken45=dict(stat)))
