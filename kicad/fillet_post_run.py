import pcbnew, sys, json, re
ToMM, MM = pcbnew.ToMM, pcbnew.FromMM


def ps_out(ps):
    res = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i)
        res.append([[(ToMM(o.CPoint(k).x), ToMM(o.CPoint(k).y)) for k in range(o.PointCount())],
                    [[(ToMM(ps.Hole(i, h).CPoint(k).x), ToMM(ps.Hole(i, h).CPoint(k).y)) for k in range(ps.Hole(i, h).PointCount())]
                     for h in range(ps.HoleCount(i))]])
    return res


mode, src = sys.argv[1:3]
b = pcbnew.LoadBoard(src)
CU = list(b.GetEnabledLayers().CuStack())
LN = {l: pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', '') for l in CU}
if mode == 'export':
    data = {'items': {}, 'zone_list': [], 'outline': [], 'hole_list': []}
    acc = {}
    def ps_for(net, L):
        if (net, L) not in acc: acc[(net, L)] = pcbnew.SHAPE_POLY_SET()
        return acc[(net, L)]
    for t in b.GetTracks():
        for L in CU:
            if t.IsOnLayer(L): t.TransformShapeToPolygon(ps_for(t.GetNetname(), LN[L]), L, 0, MM(0.002), pcbnew.ERROR_INSIDE)
    data['shapes'] = []
    for dr in b.GetDrawings():
        if dr.IsOnCopperLayer() and dr.GetNetname():
            fps = pcbnew.SHAPE_POLY_SET(); dr.TransformShapeToPolygon(fps, dr.GetLayer(), 0, MM(0.002), pcbnew.ERROR_INSIDE)
            data['shapes'].append(dict(net=dr.GetNetname(), layer=LN[dr.GetLayer()], polys=ps_out(fps)))
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                data['hole_list'].append([ToMM(p.GetPosition().x), ToMM(p.GetPosition().y), ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2])
                continue
            for L in CU:
                if p.IsOnLayer(L) and p.FlashLayer(L): p.TransformShapeToPolygon(ps_for(p.GetNetname(), LN[L]), L, 0, MM(0.002), pcbnew.ERROR_INSIDE)
    for (net, L), ps in acc.items():
        ps.Simplify(); data['items'].setdefault(net, {})[L] = ps_out(ps)
    for z in b.Zones():
        if z.GetIsRuleArea(): continue
        for L in CU:
            if not z.IsOnLayer(L): continue
            fp = z.GetFilledPolysList(L)
            if fp.OutlineCount():
                data['zone_list'].append(dict(net=z.GetNetname(), layer=LN[L], prio=z.GetAssignedPriority(),
                                          clr=ToMM(z.GetLocalClearance() or 0), name=z.GetZoneName(), polys=ps_out(fp)))
    ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, False)
    data['outline'] = ps_out(ol)
    json.dump(data, open(sys.argv[3], 'w'))
    print('Copper', len(acc), 'Net-Layers,', len(data['zone_list']), 'ZoneFills,', len(data['hole_list']), 'NPTH')
elif mode == 'insert':
    F = json.load(open(sys.argv[3])); out = sys.argv[4]
    LAY = {LN[l]: l for l in CU}
    prior = [dr for dr in b.GetDrawings() if dr.IsOnCopperLayer() and dr.GetNetname()]
    for dr in prior: b.Remove(dr)
    for z in F:
        fo = pcbnew.PCB_SHAPE(b); fo.SetShape(pcbnew.SHAPE_T_POLY)
        ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
        for x, y in z['pts']: ps.Append(MM(x), MM(y))
        fo.SetPolyShape(ps); fo.SetFilled(True); fo.SetWidth(0); fo.SetLayer(LAY[z['layer']]); b.Add(fo); fo.SetNet(b.FindNet(z['net']))
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(out)
    print('Fillet shapes follow-up', len(F), '->', out)
