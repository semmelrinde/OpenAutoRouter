import sys, json, math

mode = sys.argv[1]


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
    d = dict(cu_layers=[LN[l] for l in CU], vias=[], bore=[], npth=[], zone_list=[], keepout_list=[], items={})
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
                d['bore'].append([ToMM(p.GetPosition().x), ToMM(p.GetPosition().y), ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2, p.GetNetname()])
            for L in CU:
                if p.IsOnLayer(L) and p.FlashLayer(L): p.TransformShapeToPolygon(ps_for(p.GetNetname(), LN[L]), L, 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for dr in b.GetDrawings():
        if dr.IsOnCopperLayer() and dr.GetLayer() in LN:
            dr.TransformShapeToPolygon(ps_for(dr.GetNetname(), LN[dr.GetLayer()]), dr.GetLayer(), 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for z in b.Zones():
        cu_layers = [LN[L] for L in CU if z.IsOnLayer(L)]
        if z.GetIsRuleArea():
            d['keepout_list'].append(dict(cu_layers=cu_layers, copper_map=bool(z.GetDoNotAllowZoneFills() or z.GetDoNotAllowTracks()),
                                     vias=bool(z.GetDoNotAllowVias()), polys=ps_out(z.Outline())))
            continue
        for L in CU:
            if not z.IsOnLayer(L): continue
            fp = z.GetFilledPolysList(L)
            d['zone_list'].append(dict(net=z.GetNetname(), layer=LN[L], prio=z.GetAssignedPriority(), name=z.GetZoneName(),
                                   area_contour=z.Outline().Area() / 1e12, polys=ps_out(fp) if fp.OutlineCount() else []))
    for (net, L), ps in acc.items():
        ps.Simplify(); d['items'].setdefault(net, {})[L] = ps_out(ps)
    ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, False)
    d['outline'] = ps_out(ol)
    json.dump(d, open(out, 'w'))
    print('Export', len(acc), 'Net-Layers,', len(d['vias']), 'Vias,', len(d['zone_list']), 'Zones-Layers,', len(d['keepout_list']), 'KeepoutAreas')


def load_data(path):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    d = json.load(open(path))

    def poly(ps): return unary_union([Polygon(o, h).buffer(0) for o, h in ps]) if ps else Polygon()
    return d, poly


def patches(d, poly, A):
    from shapely.geometry import box
    from shapely.ops import unary_union
    GND = A.get('gnd', '/GND'); BORDER = float(A.get('border', 0.3)); ZCLR = float(A.get('zclr', 0.2)); MIN_WIDTH = float(A.get('min_width', 0.2))
    AMIN = float(A.get('amin', 0.3))
    board = poly(d['outline']); inner = board.buffer(-BORDER)
    zx0, zx1, zy = [float(q) for q in A.get('tongue', '-5.7,5.7,36.0').split(',')]
    x0, y0, x1, y1 = board.bounds
    inner = inner.difference(unary_union([box(x0 - 1, zy, zx0, y1 + 1), box(zx1, zy, x1 + 1, y1 + 1)]))
    out = {}
    for L in d['cu_layers']:
        sp = unary_union([poly(s['polys']) for s in d['keepout_list'] if s['copper_map'] and L in s['cu_layers']])
        cu_n = {n: poly(lay[L]) for n, lay in d['items'].items() if L in lay}
        for z in d['zone_list']:
            if z['layer'] == L and z['polys']:
                cu_n[z['net']] = unary_union([cu_n.get(z['net'], poly([])), poly(z['polys'])])
        everything = unary_union(list(cu_n.values()))
        foreign = unary_union([g for n, g in cu_n.items() if n != GND])
        gnd = cu_n.get(GND)
        if gnd is None:
            from shapely.geometry import Polygon
            gnd = Polygon()
        pour = inner.difference(sp).difference(foreign.buffer(ZCLR, quad_segs=8))
        pour = pour.buffer(-MIN_WIDTH / 2, quad_segs=8).buffer(MIN_WIDTH / 2, quad_segs=8)
        empty = pour.difference(gnd.buffer(0.05)).buffer(-MIN_WIDTH / 2, quad_segs=8).buffer(MIN_WIDTH / 2, quad_segs=8)
        part_list = [g for g in getattr(empty, 'geoms', [empty]) if g.geom_type == 'Polygon' and g.area >= AMIN]
        entire = gnd.area > 0.6 * board.area and foreign.area < 0.04 * board.area
        has_gnd_zone = any(z['net'] == GND and z['layer'] == L and z['prio'] == 0 for z in d['zone_list'])
        out[L] = dict(board=board, everything=everything, foreign=foreign, gnd=gnd, part_list=part_list, entire=entire, has_gnd_zone=has_gnd_zone,
                      keepout=sp, share=everything.intersection(board).area / board.area)
    return out


def compute(src, plan, A):
    import numpy as np
    from shapely.geometry import Point
    from shapely.ops import unary_union
    from shapely.strtree import STRtree
    from shapely.prepared import prep
    d, poly = load_data(src)
    GND = A.get('gnd', '/GND')
    VD, VDR = [float(q) for q in A.get('via', '0.45/0.20').split('/')]; VR = VD / 2
    DRC = float(A.get('drc', 0.09)) + 0.01; H2H = float(A.get('h2h', 0.2)) + 0.01; BORDER = float(A.get('border', 0.3))
    DIST = float(A.get('spacing', 3.0)); NMIN = int(A.get('min', 2)); SPLIT = float(A.get('splitter', 3.0))
    F = patches(d, poly, A)
    board = F[d['cu_layers'][0]]['board']; vinner = prep(board.buffer(-(BORDER + VR)))


    obst = {}
    for L in d['cu_layers']:
        g = [poly(lay[L]) for n, lay in d['items'].items() if n != GND and L in lay]
        g += [poly(z['polys']) for z in d['zone_list'] if z['layer'] == L and z['net'] != GND and z['polys']
              and not (z['prio'] <= 1 and z['area_contour'] > 0.6 * board.area)]
        gs = [x for h in g for x in getattr(h, 'geoms', [h]) if not x.is_empty]
        obst[L] = (STRtree(gs), gs) if gs else None
    via_keepout = prep(unary_union([poly(s['polys']) for s in d['keepout_list'] if s['vias']])) if any(s['vias'] for s in d['keepout_list']) else None
    bore = [(v['x'], v['y'], v['drill'] / 2) for v in d['vias']] + [tuple(b[:3]) for b in d['bore']] + [tuple(b) for b in d['npth']]
    gnd_vias = [(v['x'], v['y']) for v in d['vias'] if v['net'] == GND] + [(b[0], b[1]) for b in d['bore'] if len(b) > 3 and b[3] == GND]

    def via_ok(x, y, new):
        pt = Point(x, y)
        if not vinner.contains(pt): return False
        if via_keepout is not None and via_keepout.contains(pt): return False
        for L, hb in obst.items():
            if hb is None: continue
            tr, gs = hb
            for j in tr.query(pt.buffer(VR + DRC)):
                if gs[int(j)].distance(pt) < VR + DRC: return False
        for bx, by, br in bore + [(a, b_, VDR / 2) for a, b_ in new]:
            if math.hypot(bx - x, by - y) < br + VDR / 2 + H2H: return False
        return True
    new, zones_layers, report = [], [], []
    for L, f in F.items():
        if f['entire']: continue
        needs_zone = not f['has_gnd_zone']
        for P in f['part_list']:
            on_gnd = P.distance(f['gnd']) < 0.06 if not f['gnd'].is_empty else False
            Pp = prep(P.buffer(0.01))
            da = [v for v in gnd_vias + new if Pp.contains(Point(v))]
            entry = dict(layer_name=L, surface=round(P.area, 2), middle=[round(P.representative_point().x, 2), round(P.representative_point().y, 2)],
                           on_gnd=on_gnd, before=len(da), new=0)
            if on_gnd and needs_zone:
                entry['category'] = 'attached'; report.append(entry); zones_layers.append(L); continue
            if on_gnd:
                entry['category'] = 'rest'; report.append(entry); continue
            bx = P.bounds
            probe = [(x, y) for x in np.arange(bx[0], bx[2], 0.2) for y in np.arange(bx[1], bx[3], 0.2) if Pp.contains(Point(x, y))] \
                or [(P.representative_point().x, P.representative_point().y)]
            cand = [(x, y) for x in np.arange(bx[0], bx[2], 0.05) for y in np.arange(bx[1], bx[3], 0.05) if Pp.contains(Point(x, y))]

            def open_items(vs):
                return [p for p in probe if not vs or min(math.hypot(p[0] - a, p[1] - b_) for a, b_ in vs) > DIST]
            vs = list(da); assigned = []
            while len(vs) < NMIN or open_items(vs):
                op = open_items(vs); best = None
                for x, y in cand:
                    if any(math.hypot(x - a, y - b_) < 0.7 for a, b_ in vs): continue
                    gain = sum(1 for p in op if math.hypot(p[0] - x, p[1] - y) <= DIST) if op else min(math.hypot(x - a, y - b_) for a, b_ in vs)
                    if gain <= 0 or (best is not None and gain <= best[0]): continue
                    if not via_ok(x, y, new + assigned): continue
                    best = (gain, x, y)
                if best is None: break
                vs.append((best[1], best[2])); assigned.append((round(best[1], 3), round(best[2], 3)))
            if len(vs) < NMIN and P.area < SPLIT:
                entry['category'] = 'too_small'; report.append(entry); continue
            if not vs:
                entry['category'] = 'no_slot'; report.append(entry); continue
            new.extend(assigned); entry.update(category='vias', new=len(assigned), after=len(vs)); report.append(entry)
            if needs_zone: zones_layers.append(L)


    for L, f in F.items():
        if f['entire'] or f['gnd'].is_empty: continue
        for P in getattr(f['gnd'], 'geoms', [f['gnd']]):
            if P.geom_type != 'Polygon' or P.area < 0.05: continue
            Pp = prep(P.buffer(0.01))
            if any(Pp.contains(Point(v)) for v in gnd_vias + new): continue
            bx = P.bounds
            cand = [(x, y) for x in np.arange(bx[0], bx[2], 0.05) for y in np.arange(bx[1], bx[3], 0.05) if Pp.contains(Point(x, y))]
            assigned = []
            for _ in range(NMIN):
                best = None
                for x, y in cand:
                    dmin = min([math.hypot(x - a, y - b_) for a, b_ in assigned] or [9.0])
                    if dmin < 0.7: continue
                    gain = (P.exterior.distance(Point(x, y)) if not assigned else dmin)
                    if best is not None and gain <= best[0]: continue
                    if not via_ok(x, y, new + assigned): continue
                    best = (gain, x, y)
                if best is None: break
                assigned.append((round(best[1], 3), round(best[2], 3)))
            new.extend(assigned)
            report.append(dict(layer_name=L, surface=round(P.area, 2), middle=[round(P.representative_point().x, 2), round(P.representative_point().y, 2)],
                                on_gnd=True, before=0, new=len(assigned), category='gnd_without_via' if assigned else 'gnd_without_via_no_slot'))
    zones_layers = sorted(set(zones_layers))
    json.dump(dict(vias=[dict(x=x, y=y, net=GND) for x, y in new], gnd_zones=zones_layers, patches=report), open(plan, 'w'), indent=1)
    import collections
    c = collections.Counter((b['layer_name'], b['category']) for b in report)
    print('Patches', len(report), dict(c), '| new GND vias', len(new), '| new GND areas on', zones_layers or '-')


def insert(src, plan, out):
    import pcbnew
    MM = pcbnew.FromMM
    b = pcbnew.LoadBoard(src); P = json.load(open(plan))
    CU = list(b.GetEnabledLayers().CuStack()); LAY = {b.GetLayerName(l).replace('.Cu', ''): l for l in CU}
    gnd = b.FindNet('/GND')
    template = next((t for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetname() == '/GND'), None)
    for v in P['vias']:
        nv = pcbnew.PCB_VIA(b); nv.SetViaType(template.GetViaType()); nv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        nv.SetWidth(template.GetWidth(pcbnew.F_Cu)); nv.SetDrill(template.GetDrillValue())
        nv.SetPosition(pcbnew.VECTOR2I(MM(v['x']), MM(v['y']))); b.Add(nv); nv.SetNet(gnd)
    if P['gnd_zones']:
        ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, False)
        zv = next(z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == '/GND' and z.GetAssignedPriority() == 0)
        for Ln in P['gnd_zones']:
            L = LAY[Ln]
            for z in b.Zones():
                if not z.GetIsRuleArea() and z.IsOnLayer(L) and z.GetAssignedPriority() == 0 and z.GetNetname() != '/GND':
                    z.SetAssignedPriority(1)
            nz = zv.Duplicate(False).Cast(); nz.SetLayer(L); nz.SetNet(gnd); nz.SetAssignedPriority(0); b.Add(nz)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(out)
    print('Empty patches:', len(P['vias']), 'GND-Vias,', 'GND areas new on', P['gnd_zones'] or '-', '->', out)


def report(src, A):
    d, poly = load_data(src)
    F = patches(d, poly, A)
    for L, f in F.items():
        print('%-4s copper %5.1f %%  empty %6.2f mm2 in %3d patches%s' % (L, 100 * f['share'], sum(p.area for p in f['part_list']),
              len(f['part_list']), '  (full GND layer)' if f['entire'] else ''))


if __name__ == '__main__':
    A = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    pos = [a for a in sys.argv[2:] if '=' not in a]
    if mode == 'export': kicad_export(pos[0], pos[1])
    elif mode == 'compute': compute(pos[0], pos[1], A)
    elif mode == 'insert': insert(pos[0], pos[1], pos[2])
    elif mode == 'report': report(pos[0], A)
