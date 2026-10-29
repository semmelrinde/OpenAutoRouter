import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); mm = pcbnew.ToMM; out = []
for z in b.Zones():
    o = z.Outline(); polys = []
    for i in range(o.OutlineCount()):
        ol = o.Outline(i)
        polys.append(dict(outer=[(mm(ol.CPoint(k).x), mm(ol.CPoint(k).y)) for k in range(ol.PointCount())],
                          hole_list=[[(mm(o.Hole(i, h).CPoint(k).x), mm(o.Hole(i, h).CPoint(k).y)) for k in range(o.Hole(i, h).PointCount())] for h in range(o.HoleCount(i))]))
    out.append(dict(uuid=z.m_Uuid.AsString(), rule=z.GetIsRuleArea(), net=z.GetNetname(), prio=z.GetAssignedPriority(),
                    cu_layers=[pcbnew.BOARD.GetStandardLayerName(L).replace('.Cu', '') for L in z.GetLayerSet().CuStack()],
                    fill=z.GetDoNotAllowZoneFills() if z.GetIsRuleArea() else False, polys=polys))
json.dump(out, open(sys.argv[2], 'w')); print('Zones', len(out))
