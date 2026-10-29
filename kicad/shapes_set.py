import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); F = json.load(open(sys.argv[2])); MM = pcbnew.FromMM
SKIP = {tuple(map(tuple, s)) for s in (json.load(open(sys.argv[4])) if len(sys.argv) > 4 else [])}
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu, 'In2': pcbnew.In2_Cu, 'In5': pcbnew.In5_Cu}
n = 0
for z in F:
    if tuple(map(tuple, z['pts'])) in SKIP: continue
    fo = pcbnew.PCB_SHAPE(b); fo.SetShape(pcbnew.SHAPE_T_POLY)
    ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
    for x, y in z['pts']: ps.Append(MM(x), MM(y))
    fo.SetPolyShape(ps); fo.SetFilled(True); fo.SetWidth(0); fo.SetLayer(LAY[z['layer']]); b.Add(fo); fo.SetNet(b.FindNet(z['net'])); n += 1
print('Copper shapes placed', n)
pcbnew.ZONE_FILLER(b).Fill(b.Zones()); b.Save(sys.argv[3])
