import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); F = json.load(open(sys.argv[2])); MM = pcbnew.FromMM
LAY = {pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', ''): l for l in b.GetEnabledLayers().CuStack()}
for z in F:
    fo = pcbnew.PCB_SHAPE(b); fo.SetShape(pcbnew.SHAPE_T_POLY); ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
    for x, y in z['pts']: ps.Append(MM(x), MM(y))
    fo.SetPolyShape(ps); fo.SetFilled(True); fo.SetWidth(0); fo.SetLayer(LAY[z['layer']]); b.Add(fo); fo.SetNet(b.FindNet(z['net']))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(sys.argv[3]); print('Shapes attached:', len(F), '->', sys.argv[3])
