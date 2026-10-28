""
import pcbnew, sys
src, out = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src)
bb = b.GetBoardEdgesBoundingBox()
if sys.argv[3] == 'box':
    x0, y0, x1, y1, X0, Y0, X1, Y1 = [pcbnew.FromMM(float(v)) for v in sys.argv[4:12]]
    def T(p): return pcbnew.VECTOR2I(int(X0 + (p.x - x0) * (X1 - X0) / (x1 - x0)), int(Y0 + (p.y - y0) * (Y1 - Y0) / (y1 - y0)))
    f, cx, cy = (X1 - X0) * (Y1 - Y0) / ((x1 - x0) * (y1 - y0)), 0.0, 0.0
else:
    f = float(sys.argv[3])
    cx, cy = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else (pcbnew.ToMM(bb.Centre().x), pcbnew.ToMM(bb.Centre().y))
    C = pcbnew.VECTOR2I(pcbnew.FromMM(cx), pcbnew.FromMM(cy))
    def T(p): return pcbnew.VECTOR2I(int(C.x + f * (p.x - C.x)), int(C.y + f * (p.y - C.y)))
n = 0
for d in b.GetDrawings():
    if d.GetLayer() != pcbnew.Edge_Cuts or not isinstance(d, pcbnew.PCB_SHAPE): continue
    s = d.GetShape()
    if s == pcbnew.SHAPE_T_ARC:
        a, m, e = d.GetStart(), d.GetArcMid(), d.GetEnd(); d.SetArcGeometry(T(a), T(m), T(e))
    elif s == pcbnew.SHAPE_T_CIRCLE:
        c, e = d.GetCenter(), d.GetEnd(); d.SetCenter(T(c)); d.SetEnd(T(e))
    elif s == pcbnew.SHAPE_T_POLY:
        ps = d.GetPolyShape()
        for i in range(ps.OutlineCount()):
            o = ps.Outline(i)
            for k in range(o.PointCount()): o.SetPoint(k, T(o.CPoint(k)))
    else:
        a, e = d.GetStart(), d.GetEnd(); d.SetStart(T(a)); d.SetEnd(T(e))
    n += 1

mit = set(next((a.split('=', 1)[1].split(',') for a in sys.argv if a.startswith('mit=')), []))
for fp in b.GetFootprints():
    if fp.GetReference() in mit:
        p = fp.GetPosition(); q = T(p); fp.Move(pcbnew.VECTOR2I(q.x - p.x, q.y - p.y))
b.Save(out)
print('Umriss skaliert: %d Elemente, Faktor %.3f um (%.2f, %.2f) -> %s' % (n, f, cx, cy, out))
