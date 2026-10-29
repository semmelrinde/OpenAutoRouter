import pcbnew, sys
src, out = sys.argv[1:3]; opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a); MAX = float(opt.get('max', '0.2'))
b = pcbnew.LoadBoard(src); way = stay = 0
for d in list(b.GetDrawings()):
    if not (d.IsOnCopperLayer() and d.GetNetname()): continue
    ps = pcbnew.SHAPE_POLY_SET(); d.TransformShapeToPolygon(ps, d.GetLayer(), 0, pcbnew.FromMM(0.002), pcbnew.ERROR_INSIDE)
    if ps.Area() / 1e12 < MAX: b.Remove(d); way += 1
    else: stay += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out); print('Teardrops removed:', way, '| Design copper kept:', stay)
