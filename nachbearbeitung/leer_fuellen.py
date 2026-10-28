""
import sys, json, math

modus = sys.argv[1]


def kicad_export(src, out):
    import pcbnew
    ToMM, MM = pcbnew.ToMM, pcbnew.FromMM
    b = pcbnew.LoadBoard(src)
    CU = list(b.GetEnabledLayers().CuStack()); LN = {l: b.GetLayerName(l).replace('.Cu', '') for l in CU}

    def ps_out(ps):
        res = []
        for i in range(ps.OutlineCount()):
            o = ps.Outline(i)
            res.append([[(ToMM(o.CPoint(k).x), ToMM(o.CPoint(k).y)) for k in range(o.PointCount())],
                        [[(ToMM(ps.Hole(i, h).CPoint(k).x), ToMM(ps.Hole(i, h).CPoint(k).y)) for k in range(ps.Hole(i, h).PointCount())]
                         for h in range(ps.HoleCount(i))]])
        return res
    acc = {}

    def ps_for(net, L):
        if (net, L) not in acc: acc[(net, L)] = pcbnew.SHAPE_POLY_SET()
        return acc[(net, L)]
    d = dict(lagen=[LN[l] for l in CU], vias=[], bohr=[], npth=[], zonen=[], sperren=[], items={})
    for t in b.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            d['vias'].append(dict(x=ToMM(t.GetPosition().x), y=ToMM(t.GetPosition().y), d=ToMM(t.GetWidth(pcbnew.F_Cu)),
                                  drill=ToMM(t.GetDrillValue()), net=t.GetNetname()))
        for L in CU:
            if t.IsOnLayer(L): t.TransformShapeToPolygon(ps_for(t.GetNetname(), LN[L]), L, 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                d['npth'].append([ToMM(p.GetPosition().x), ToMM(p.GetPosition().y), ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2])
                continue
            if p.GetDrillSize().x > 0:
                d['bohr'].append([ToMM(p.GetPosition().x), ToMM(p.GetPosition().y), ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2, p.GetNetname()])
            for L in CU:
                if p.IsOnLayer(L) and p.FlashLayer(L): p.TransformShapeToPolygon(ps_for(p.GetNetname(), LN[L]), L, 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for dr in b.GetDrawings():
        if dr.IsOnCopperLayer() and dr.GetLayer() in LN:
            dr.TransformShapeToPolygon(ps_for(dr.GetNetname(), LN[dr.GetLayer()]), dr.GetLayer(), 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for z in b.Zones():
        lagen = [LN[L] for L in CU if z.IsOnLayer(L)]
        if z.GetIsRuleArea():
            d['sperren'].append(dict(lagen=lagen, kupfer=bool(z.GetDoNotAllowZoneFills() or z.GetDoNotAllowTracks()),
                                     vias=bool(z.GetDoNotAllowVias()), polys=ps_out(z.Outline())))
            continue
        for L in CU:
            if not z.IsOnLayer(L): continue
            fp = z.GetFilledPolysList(L)
            d['zonen'].append(dict(net=z.GetNetname(), layer=LN[L], prio=z.GetAssignedPriority(), name=z.GetZoneName(),
                                   flaeche_umriss=z.Outline().Area() / 1e12, polys=ps_out(fp) if fp.OutlineCount() else []))
    for (net, L), ps in acc.items():
        ps.Simplify(); d['items'].setdefault(net, {})[L] = ps_out(ps)
    ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, False)
    d['outline'] = ps_out(ol)
    json.dump(d, open(out, 'w'))
    print('Export', len(acc), 'Netz-Lagen,', len(d['vias']), 'Vias,', len(d['zonen']), 'Zonen-Lagen,', len(d['sperren']), 'Sperrflaechen')


def laden(path):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    d = json.load(open(path))

    def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps]) if ps else Polygon()
    return d, poly


def flecken(d, poly, A):
    ""
    from shapely.geometry import box
    from shapely.ops import unary_union
    GND = A.get('gnd', '/GND'); RAND = float(A.get('rand', 0.3)); ZCLR = float(A.get('zclr', 0.2)); MINW = float(A.get('minw', 0.2))
    AMIN = float(A.get('amin', 0.3))
    board = poly(d['outline']); innen = board.buffer(-RAND)
    zx0, zx1, zy = [float(q) for q in A.get('zunge', '-5.7,5.7,36.0').split(',')]
    x0, y0, x1, y1 = board.bounds
    innen = innen.difference(unary_union([box(x0 - 1, zy, zx0, y1 + 1), box(zx1, zy, x1 + 1, y1 + 1)]))
    out = {}
    for L in d['lagen']:
        sp = unary_union([poly(s['polys']) for s in d['sperren'] if s['kupfer'] and L in s['lagen']])
        cu_n = {n: poly(lay[L]) for n, lay in d['items'].items() if L in lay}
        for z in d['zonen']:
            if z['layer'] == L and z['polys']:
                cu_n[z['net']] = unary_union([cu_n.get(z['net'], poly([])), poly(z['polys'])])
        alles = unary_union(list(cu_n.values()))
        fremd = unary_union([g for n, g in cu_n.items() if n != GND])
        gnd = cu_n.get(GND)
        if gnd is None:
            from shapely.geometry import Polygon
            gnd = Polygon()
        pour = innen.difference(sp).difference(fremd.buffer(ZCLR, quad_segs=8))
        pour = pour.buffer(-MINW / 2, quad_segs=8).buffer(MINW / 2, quad_segs=8)
        leer = pour.difference(gnd.buffer(0.05)).buffer(-MINW / 2, quad_segs=8).buffer(MINW / 2, quad_segs=8)
        teile = [g for g in getattr(leer, 'geoms', [leer]) if g.geom_type == 'Polygon' and g.area >= AMIN]
        voll = gnd.area > 0.6 * board.area and fremd.area < 0.04 * board.area
        hat_gnd_zone = any(z['net'] == GND and z['layer'] == L and z['prio'] == 0 for z in d['zonen'])
        out[L] = dict(board=board, alles=alles, fremd=fremd, gnd=gnd, teile=teile, voll=voll, hat_gnd_zone=hat_gnd_zone,
                      sperre=sp, anteil=alles.intersection(board).area / board.area)
    return out


def rechnen(src, plan, A):
    import numpy as np
    from shapely.geometry import Point
    from shapely.ops import unary_union
    from shapely.strtree import STRtree
    from shapely.prepared import prep
    d, poly = laden(src)
    GND = A.get('gnd', '/GND')
    VD, VDR = [float(q) for q in A.get('via', '0.45/0.20').split('/')]; VR = VD / 2
    DRC = float(A.get('drc', 0.09)) + 0.01; H2H = float(A.get('h2h', 0.2)) + 0.01; RAND = float(A.get('rand', 0.3))
    ABST = float(A.get('abstand', 3.0)); NMIN = int(A.get('min', 2)); SPLIT = float(A.get('splitter', 3.0))
    F = flecken(d, poly, A)
    board = F[d['lagen'][0]]['board']; vinnen = prep(board.buffer(-(RAND + VR)))


    hind = {}
    for L in d['lagen']:
        g = [poly(lay[L]) for n, lay in d['items'].items() if n != GND and L in lay]
        g += [poly(z['polys']) for z in d['zonen'] if z['layer'] == L and z['net'] != GND and z['polys']
              and not (z['prio'] <= 1 and z['flaeche_umriss'] > 0.6 * board.area)]
        gs = [x for h in g for x in getattr(h, 'geoms', [h]) if not x.is_empty]
        hind[L] = (STRtree(gs), gs) if gs else None
    viasperre = prep(unary_union([poly(s['polys']) for s in d['sperren'] if s['vias']])) if any(s['vias'] for s in d['sperren']) else None
    bohr = [(v['x'], v['y'], v['drill'] / 2) for v in d['vias']] + [tuple(b[:3]) for b in d['bohr']] + [tuple(b) for b in d['npth']]
    gnd_vias = [(v['x'], v['y']) for v in d['vias'] if v['net'] == GND] + [(b[0], b[1]) for b in d['bohr'] if len(b) > 3 and b[3] == GND]

    def via_ok(x, y, neu):
        pt = Point(x, y)
        if not vinnen.contains(pt): return False
        if viasperre is not None and viasperre.contains(pt): return False
        for L, hb in hind.items():
            if hb is None: continue
            tr, gs = hb
            for j in tr.query(pt.buffer(VR + DRC)):
                if gs[int(j)].distance(pt) < VR + DRC: return False
        for bx, by, br in bohr + [(a, b_, VDR / 2) for a, b_ in neu]:
            if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
        return True
    neu, zonen_lagen, bericht = [], [], []
    for L, f in F.items():
        if f['voll']: continue
        braucht_zone = not f['hat_gnd_zone']
        for P in f['teile']:
            an_gnd = P.distance(f['gnd']) < 0.06 if not f['gnd'].is_empty else False
            Pp = prep(P.buffer(0.01))
            da = [v for v in gnd_vias + neu if Pp.contains(Point(v))]
            eintrag = dict(lage=L, flaeche=round(P.area, 2), mitte=[round(P.representative_point().x, 2), round(P.representative_point().y, 2)],
                           an_gnd=an_gnd, vorher=len(da), neu=0)
            if an_gnd and braucht_zone:
                eintrag['art'] = 'angeschlossen'; bericht.append(eintrag); zonen_lagen.append(L); continue
            if an_gnd:
                eintrag['art'] = 'rest'; bericht.append(eintrag); continue
            bx = P.bounds
            probe = [(x, y) for x in np.arange(bx[0], bx[2], 0.2) for y in np.arange(bx[1], bx[3], 0.2) if Pp.contains(Point(x, y))] \
                or [(P.representative_point().x, P.representative_point().y)]
            kand = [(x, y) for x in np.arange(bx[0], bx[2], 0.05) for y in np.arange(bx[1], bx[3], 0.05) if Pp.contains(Point(x, y))]

            def offen(vs):
                return [p for p in probe if not vs or min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > ABST]
            vs = list(da); gesetzt = []
            while len(vs) < NMIN or offen(vs):
                op = offen(vs); best = None
                for x, y in kand:
                    if any(math.hypot(x - a, y - b_) < 0.7 for a, b_ in vs): continue
                    gain = sum(1 for p in op if math.hypot(p[0] - x, p[1] - y) <= ABST) if op else min(math.hypot(x - a, y - b_) for a, b_ in vs)
                    if gain <= 0 or (best is not None and gain <= best[0]): continue
                    if not via_ok(x, y, neu + gesetzt): continue
                    best = (gain, x, y)
                if best is None: break
                vs.append((best[1], best[2])); gesetzt.append((round(best[1], 3), round(best[2], 3)))
            if len(vs) < NMIN and P.area < SPLIT:
                eintrag['art'] = 'zu_klein'; bericht.append(eintrag); continue
            if not vs:
                eintrag['art'] = 'kein_platz'; bericht.append(eintrag); continue
            neu.extend(gesetzt); eintrag.update(art='vias', neu=len(gesetzt), nachher=len(vs)); bericht.append(eintrag)
            if braucht_zone: zonen_lagen.append(L)


    for L, f in F.items():
        if f['voll'] or f['gnd'].is_empty: continue
        for P in getattr(f['gnd'], 'geoms', [f['gnd']]):
            if P.geom_type != 'Polygon' or P.area < 0.05: continue
            Pp = prep(P.buffer(0.01))
            if any(Pp.contains(Point(v)) for v in gnd_vias + neu): continue
            bx = P.bounds
            kand = [(x, y) for x in np.arange(bx[0], bx[2], 0.05) for y in np.arange(bx[1], bx[3], 0.05) if Pp.contains(Point(x, y))]
            gesetzt = []
            for _ in range(NMIN):
                best = None
                for x, y in kand:
                    dmin = min([math.hypot(x - a, y - b_) for a, b_ in gesetzt] or [9.0])
                    if dmin < 0.7: continue
                    gain = (P.exterior.distance(Point(x, y)) if not gesetzt else dmin)
                    if best is not None and gain <= best[0]: continue
                    if not via_ok(x, y, neu + gesetzt): continue
                    best = (gain, x, y)
                if best is None: break
                gesetzt.append((round(best[1], 3), round(best[2], 3)))
            neu.extend(gesetzt)
            bericht.append(dict(lage=L, flaeche=round(P.area, 2), mitte=[round(P.representative_point().x, 2), round(P.representative_point().y, 2)],
                                an_gnd=True, vorher=0, neu=len(gesetzt), art='gnd_ohne_via' if gesetzt else 'gnd_ohne_via_kein_platz'))
    zonen_lagen = sorted(set(zonen_lagen))
    json.dump(dict(vias=[dict(x=x, y=y, net=GND) for x, y in neu], gnd_zonen=zonen_lagen, flecken=bericht), open(plan, 'w'), indent=1)
    import collections
    c = collections.Counter((b['lage'], b['art']) for b in bericht)
    print('Flecken', len(bericht), dict(c), '| neue GND-Vias', len(neu), '| neue GND-Flaechen auf', zonen_lagen or '-')


def einfuegen(src, plan, out):
    import pcbnew
    MM = pcbnew.FromMM
    b = pcbnew.LoadBoard(src); P = json.load(open(plan))
    CU = list(b.GetEnabledLayers().CuStack()); LAY = {b.GetLayerName(l).replace('.Cu', ''): l for l in CU}
    gnd = b.FindNet('/GND')
    vorlage = next((t for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetname() == '/GND'), None)
    for v in P['vias']:
        nv = pcbnew.PCB_VIA(b); nv.SetViaType(vorlage.GetViaType()); nv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        nv.SetWidth(vorlage.GetWidth(pcbnew.F_Cu)); nv.SetDrill(vorlage.GetDrillValue())
        nv.SetPosition(pcbnew.VECTOR2I(MM(v['x']), MM(v['y']))); b.Add(nv); nv.SetNet(gnd)
    if P['gnd_zonen']:
        ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, False)
        zv = next(z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == '/GND' and z.GetAssignedPriority() == 0)
        for Ln in P['gnd_zonen']:
            L = LAY[Ln]
            for z in b.Zones():
                if not z.GetIsRuleArea() and z.IsOnLayer(L) and z.GetAssignedPriority() == 0 and z.GetNetname() != '/GND':
                    z.SetAssignedPriority(1)
            nz = zv.Duplicate(False).Cast(); nz.SetLayer(L); nz.SetNet(gnd); nz.SetAssignedPriority(0); b.Add(nz)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(out)
    print('Leere Flecken:', len(P['vias']), 'GND-Vias,', 'GND-Flaechen neu auf', P['gnd_zonen'] or '-', '->', out)


def bericht(src, A):
    d, poly = laden(src)
    F = flecken(d, poly, A)
    for L, f in F.items():
        print('%-4s Kupfer %5.1f %%  leer %6.2f mm2 in %3d Flecken%s' % (L, 100 * f['anteil'], sum(p.area for p in f['teile']),
              len(f['teile']), '  (volle GND-Lage)' if f['voll'] else ''))


if __name__ == '__main__':
    A = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    pos = [a for a in sys.argv[2:] if '=' not in a]
    if modus == 'export': kicad_export(pos[0], pos[1])
    elif modus == 'rechnen': rechnen(pos[0], pos[1], A)
    elif modus == 'einfuegen': einfuegen(pos[0], pos[1], pos[2])
    elif modus == 'bericht': bericht(pos[0], A)
