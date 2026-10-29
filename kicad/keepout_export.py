import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); mm = pcbnew.ToMM; out = []
zs = [(z, '') for z in b.Zones()] + [(z, fp.GetReference()) for fp in b.GetFootprints() for z in fp.Zones()]
for z, ref in zs:
    if not z.GetIsRuleArea(): continue
    o = z.Outline()
    for i in range(o.OutlineCount()):
        ol = o.Outline(i)
        out.append(dict(ref=ref, cu_layers=[pcbnew.BOARD.GetStandardLayerName(L).replace('.Cu', '') for L in z.GetLayerSet().CuStack()],
                        fill=z.GetDoNotAllowZoneFills(), track_list=z.GetDoNotAllowTracks(), vias=z.GetDoNotAllowVias(),
                        pts=[(mm(ol.CPoint(k).x), mm(ol.CPoint(k).y)) for k in range(ol.PointCount())]))
json.dump(out, open(sys.argv[2], 'w')); print('KeepoutAreas', len(out))
