""
import json, sys, math, copy


def bewegen(B, moves, seiten=None):
    ""
    B = copy.deepcopy(B)
    fps = {f['ref']: f for f in B['fps']}
    seiten = seiten or {}
    swap = {'F': 'B', 'B': 'F'}
    for i in range(1, B.get('lagen', 2) - 1):
        swap['In%d' % i] = 'In%d' % (B['lagen'] - 1 - i)
    parts = {}
    for r in dict.fromkeys([*moves, *seiten]):
        dx, dy, th = moves.get(r, (0, 0, 0))
        f = fps[r]; ox, oy = f['x'], f['y']
        side = seiten.get(r, f['side']); flip = side != f['side']
        if side not in ('F', 'B'): raise ValueError('Ungueltige Zielseite: ' + side)
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        def tf(x, y):
            rx, ry = x - ox, y - oy
            if flip: rx = -rx
            return [round(ox + c * rx - s * ry + dx, 5), round(oy + s * rx + c * ry + dy, 5)]
        def polygons(qs): return [[tf(x, y) for x, y in q] for q in qs]
        def layer(L): return swap.get(L, L) if flip else L
        def geometry(p):
            if 'x' in p: p['x'], p['y'] = tf(p['x'], p['y'])
            if isinstance(p['polys'], dict):
                p['polys'] = {layer(L): polygons(qs) for L, qs in p['polys'].items()}
                points = [v for qs in p['polys'].values() for q in qs for v in q]
            else:
                p['polys'] = polygons(p['polys'])
                points = [v for q in p['polys'] for v in q]
            if 'hole_polys' in p: p['hole_polys'] = polygons(p['hole_polys'])
            if 'loecher' in p: p['loecher'] = [polygons(qs) for qs in p['loecher']]
            if not points:
                points = [v for q in p.get('hole_polys', []) for v in q]
            if 'bbox' in p and not points:
                x1, y1, x2, y2 = p['bbox']
                points = [tf(x, y) for x, y in ((x1, y1), (x2, y1), (x2, y2), (x1, y2))]
            if 'bbox' in p and points:
                p['bbox'] = [min(x for x, _ in points), min(y for _, y in points),
                             max(x for x, _ in points), max(y for _, y in points)]
            for key in ('layers', 'lagen'):
                if key in p: p[key] = [layer(L) for L in p[key]]
            for key in ('side', 'fp_side', 'layer'):
                if key in p: p[key] = layer(p[key])
            if 'maske' in p: p['maske'] = {layer(L): v for L, v in p['maske'].items()}
        steg = 'NTBAR_' + (r[3:] if r.startswith('NT_') else r)
        electrical = []
        for p in B['pads']:
            if p['ref'] not in (r, steg): continue
            geometry(p)
            if p['ref'] == r: electrical.append(p)
        for field in ('footprint_copper', 'mask_graphics', 'via_sperre', 'courtyards', 'keepouts'):
            for p in B.get(field, []):
                if p.get('ref') == r: geometry(p)
        f['x'], f['y'] = round(ox + dx, 5), round(oy + dy, 5)
        f['rot'] = ((180 - f.get('rot', 0) if flip else f.get('rot', 0)) - th) % 360
        f['side'] = side
        parts[r] = dict(side=side, x=f['x'], y=f['y'], drehung=th, transformation_eindeutig=True,
                        pads={p['num']: [p['x'], p['y']] for p in electrical},
                        padinstanzen=[dict(num=p['num'], net=p['net'], x=p['x'], y=p['y'],
                            side='FB' if p.get('drill', 0) > 0 or len(p['layers']) > 1 else side) for p in electrical])
    from platzierung.p3_gruppen import board_mit_gruppen
    return B, board_mit_gruppen(dict(parts=parts), B)


def lese_moves(specs):
    m = {}
    for sp in specs:
        r, v = sp.split(':', 1); q = [float(x) for x in v.split(',')]
        m[r] = (q[0], q[1], q[2] if len(q) > 2 else 0.0)
    return m


if __name__ == '__main__':
    kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a and ':' not in a.split('=', 1)[0])
    specs = [a for a in sys.argv[3:] if ':' in a and not a.startswith('placement=')]
    B = json.load(open(sys.argv[1]))
    B2, P = bewegen(B, lese_moves(specs))
    json.dump(B2, open(sys.argv[2], 'w'))
    if 'placement' in kw: json.dump(P, open(kw['placement'], 'w'))
    print('verschoben:', ', '.join(specs))
