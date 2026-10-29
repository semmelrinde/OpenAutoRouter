import sys, json, math, os

mode = sys.argv[1]
A = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
P = [a for a in sys.argv[2:] if '=' not in a]
VD, VDR = [float(q) for q in A.get('via', '0.45/0.20').split('/')]; VR = VD / 2


def candidates(bj, out):
    from shapely.geometry import Polygon, Point, LineString
    from shapely.ops import unary_union, nearest_points
    B = json.load(open(bj))
    N = int(A.get('n', 12))

    def pg(p): return unary_union([Polygon(q).buffer(0) for L, v in p['polys'].items() for q in v])
    board = Polygon(B['outline'][0]) if isinstance(B['outline'][0][0], list) else Polygon(B['outline'])
    inner = board.buffer(-(0.3 + VR))
    keepout = unary_union([Polygon(q) for v in B.get('via_keepout', []) for q in v['polys']])
    nts = sorted({p['ref'] for p in B['pads'] if p['ref'].startswith('NT_')})
    res = {}
    for nt in nts:
        gp = [p for p in B['pads'] if p['ref'] == nt and p['net'] == '/GND']
        if not gp: continue
        gp = gp[0]; pad = pg(gp)
        foreign = [(pg(p), p) for p in B['pads'] if p['net'] != '/GND' and math.hypot(p['x'] - gp['x'], p['y'] - gp['y']) < 4]
        bore = [(p['x'], p['y'], (max(p['drill']) if isinstance(p['drill'], list) else p['drill']) / 2) for p in B['pads']
                if p.get('drill') and math.hypot(p['x'] - gp['x'], p['y'] - gp['y']) < 4]
        cand = []
        zone = pad.buffer(float(A.get('radius', 0.8)))
        Lp = gp['layers'][0]; pad_in = pad.buffer(-0.08)
        foreign_l = [g for g, p in foreign if Lp in p['layers'] and not p['ref'].startswith('NTBAR')]
        bx = zone.bounds
        x = bx[0]
        while x <= bx[2]:
            y = bx[1]
            while y <= bx[3]:
                pt = Point(x, y)
                if zone.contains(pt) and inner.contains(pt) \
                   and all(g.distance(pt) >= VR + 0.10 + 0.01 for g, _ in foreign) \
                   and all(math.hypot(bx_ - x, by_ - y) >= br + VDR / 2 + 0.21 for bx_, by_, br in bore):
                    stub = pad.distance(pt.buffer(VR)) > -1e-9 and not pt.buffer(VR).buffer(-0.1).intersects(pad)
                    tx, ty = x, y
                    if stub:
                        q = nearest_points(pad_in, pt)[0]; tx, ty = q.x, q.y
                        st = LineString([(x, y), (tx, ty)]).buffer(0.1)
                        if any(g.distance(st) < 0.11 for g in foreign_l): y += 0.025; continue
                    cand.append((round(math.hypot(x - gp['x'], y - gp['y']) + (0 if not stub else 0.1), 4), round(x, 4), round(y, 4), round(tx, 4), round(ty, 4)))
                y += 0.025
            x += 0.025
        cand.sort()
        res[nt] = dict(pad=[gp['x'], gp['y']], num=gp['num'], layer_name=gp['layers'][0], over=[round(VR - 0.1, 3)], cand=[list(k[1:]) for k in cand[:N * 8:8]] if len(cand) > N else [list(k[1:]) for k in cand])
        print(nt, 'Candidates', len(cand), 'nearest %.3f mm' % cand[0][0] if cand else '-')
    json.dump(res, open(out, 'w'), indent=1)


def verify(base, kj, out, pro):
    import pcbnew, subprocess, tempfile, shutil
    MM = pcbnew.FromMM
    K = json.load(open(kj)); idx = {nt: 0 for nt in K}; ok = {}
    tmp = tempfile.mkdtemp()
    for round_no in range(12):
        open_items = [nt for nt in K if nt not in ok and idx[nt] < len(K[nt]['cand'])]
        if not open_items: break
        b = pcbnew.LoadBoard(base); gnd = b.FindNet('/GND'); put = {}
        for nt in list(ok) + open_items:
            x, y, tx, ty = ok[nt] if nt in ok else K[nt]['cand'][idx[nt]]
            v = pcbnew.PCB_VIA(b); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetWidth(MM(VD)); v.SetDrill(MM(VDR))
            v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y))); b.Add(v); v.SetNet(gnd); put[nt] = (x, y, tx, ty)
            px, py = tx, ty
            if math.hypot(x - px, y - py) > 1e-6:
                t = pcbnew.PCB_TRACK(b); t.SetStart(pcbnew.VECTOR2I(MM(x), MM(y))); t.SetEnd(pcbnew.VECTOR2I(MM(px), MM(py)))
                t.SetWidth(MM(0.2)); t.SetLayer(pcbnew.B_Cu if K[nt]['layer_name'] == 'B' else pcbnew.F_Cu); b.Add(t); t.SetNet(gnd)
        f = os.path.join(tmp, 't.kicad_pcb'); b.Save(f); shutil.copy(pro, os.path.join(tmp, 't.kicad_pro'))
        shutil.copy(pro.replace('.kicad_pro', '.kicad_dru'), os.path.join(tmp, 't.kicad_dru')) if os.path.exists(pro.replace('.kicad_pro', '.kicad_dru')) else None
        subprocess.run(['/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli', 'pcb', 'drc', '--format', 'json', '-o',
                        os.path.join(tmp, 'd.json'), f], capture_output=True)
        D = json.load(open(os.path.join(tmp, 'd.json')))
        bad = set()
        for vio in D['violations']:
            if vio['severity'] != 'error': continue
            for it in vio['items']:
                if not it['description'].startswith(('Via', 'Track')): continue
                for nt, (x, y, px, py) in put.items():
                    if math.hypot(it['pos']['x'] - x, it['pos']['y'] - y) < 0.01 or (it['description'].startswith('Track') and min(math.hypot(it['pos']['x'] - x, it['pos']['y'] - y), math.hypot(it['pos']['x'] - px, it['pos']['y'] - py)) < 0.01): bad.add(nt)
        for nt in open_items:
            if nt in bad: idx[nt] += 1
            else: ok[nt] = put[nt]
        print('Round', round_no + 1, 'ok', sorted(ok), 'next attempt', sorted(bad))
    res = {nt: dict(x=ok[nt][0], y=ok[nt][1], num=K[nt]['num'], pad=K[nt]['pad'], goal=[ok[nt][2], ok[nt][3]], layer_name=K[nt]['layer_name'], spacing=round(math.hypot(ok[nt][0] - K[nt]['pad'][0], ok[nt][1] - K[nt]['pad'][1]), 3))
           for nt in ok}
    json.dump(res, open(out, 'w'), indent=1)
    print('StarPoint-Vias', {nt: r['spacing'] for nt, r in res.items()}, '| without room', sorted(set(K) - set(ok)))


def pads(bj, sj, out):
    B = json.load(open(bj)); S = json.load(open(sj))
    cu_layers = ['F', 'In1', 'In2', 'In3', 'In4', 'B'][:int(A.get('cu_layers', 6))]
    for nt, r in S.items():
        x, y = r['x'], r['y']
        circ = [[round(x + VR * math.cos(2 * math.pi * k / 24), 5), round(y + VR * math.sin(2 * math.pi * k / 24), 5)] for k in range(24)]
        polys = {L: [circ] for L in cu_layers}
        px, py = r['goal']
        if math.hypot(x - px, y - py) > 1e-6:
            from shapely.geometry import LineString, Point
            g = LineString([(x, y), (px, py)]).buffer(0.1, 8).union(Point(x, y).buffer(VR, 24))
            polys[r['layer_name']] = [[[round(a, 5), round(b_, 5)] for a, b_ in g.exterior.coords][:-1]]
        B['pads'].append(dict(ref='STAR_' + nt, num='1', net='/GND', layers=cu_layers, polys=polys, mask={},
                              x=x, y=y, bbox=[x - VR, y - VR, x + VR, y + VR], drill=VDR, fp_side='F', pinfunc='', pintype='passive'))
    json.dump(B, open(out, 'w'))
    print(';'.join('%s:%s' % (nt, r['num']) for nt, r in S.items()))


def vias(rj, sj, out):
    R = json.load(open(rj)); S = json.load(open(sj))


    way = [v for v in R['vias'] if v['net'] == '/GND' and any(math.hypot(v['x'] - r['x'], v['y'] - r['y']) < VDR + 0.2 for r in S.values())]
    for v in way:
        r = min(S.values(), key=lambda r: math.hypot(v['x'] - r['x'], v['y'] - r['y']))
        if math.hypot(v['x'] - r['x'], v['y'] - r['y']) < VR - 0.02: continue
        for t in R['tracks']:
            if t['net'] != '/GND': continue
            for i in (0, -1):
                if math.hypot(t['pts'][i][0] - v['x'], t['pts'][i][1] - v['y']) < 1e-3:
                    if i == 0: t['pts'].insert(0, [r['x'], r['y']]); t['w'].insert(0, t['w'][0])
                    else: t['pts'].append([r['x'], r['y']]); t['w'].append(t['w'][-1])
    R['vias'] = [v for v in R['vias'] if v not in way]
    if way: print('duplicate GND vias at the star point removed:', len(way))
    for nt, r in S.items():
        R['vias'].append(dict(net='/GND', x=r['x'], y=r['y']))
        px, py = r['goal']
        if math.hypot(r['x'] - px, r['y'] - py) > 1e-6:
            R['tracks'].append(dict(net='/GND', layer=r['layer_name'], pts=[[r['x'], r['y']], [px, py]], w=[0.2], sig=False))
    json.dump(R, open(out, 'w'))
    print('Star-point vias in routes:', len(S))


if mode == 'candidates': candidates(P[0], P[1])
elif mode == 'verify': verify(P[0], P[1], P[2], P[3])
elif mode == 'pads': pads(P[0], P[1], P[2])
elif mode == 'vias': vias(P[0], P[1], P[2])
