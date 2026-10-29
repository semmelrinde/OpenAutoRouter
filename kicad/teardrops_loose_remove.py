import pcbnew, sys
src, out = sys.argv[1:3]; opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a); MAX = float(opt.get('max', '0.2'))
b = pcbnew.LoadBoard(src); ges = 0
for round_no in range(5):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones()); b.BuildConnectivity(); conn = b.GetConnectivity(); way = []
    for d in b.GetDrawings():
        if not (d.IsOnCopperLayer() and d.GetNetname()): continue
        ps = pcbnew.SHAPE_POLY_SET(); d.TransformShapeToPolygon(ps, d.GetLayer(), 0, pcbnew.FromMM(0.002), pcbnew.ERROR_INSIDE)
        if ps.Area() / 1e12 >= MAX: continue
        if not [i for i in conn.GetConnectedItems(d) if i.m_Uuid.AsString() != d.m_Uuid.AsString()]: way.append(d)
    for d in way: b.Remove(d)
    ges += len(way)
    if not way: break
b.Save(out); print('loose teardrops removed:', ges, '(rounds %d)' % (round_no + 1))
