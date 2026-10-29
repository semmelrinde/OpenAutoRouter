import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); X0, Y0, X1, Y1 = map(float, sys.argv[2:6]); mm = pcbnew.ToMM
LAY = {pcbnew.F_Cu: 'F', pcbnew.B_Cu: 'B', **{getattr(pcbnew, 'In%d_Cu' % i): 'In%d' % i for i in range(1, 31) if hasattr(pcbnew, 'In%d_Cu' % i)}}
EN = b.GetEnabledLayers()
def inside(x, y, m=1.0): return X0 - m <= x <= X1 + m and Y0 - m <= y <= Y1 + m
out = dict(tracks=[], vias=[], pads=[], holes=[], keepouts=[], edges=[])
for t in b.GetTracks():
    n = t.GetNetname()
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        if inside(mm(p.x), mm(p.y)): out['vias'].append(dict(net=n, x=mm(p.x), y=mm(p.y), d=mm(t.GetWidth(pcbnew.F_Cu)), drill=mm(t.GetDrillValue()), uuid=t.m_Uuid.AsString(),
                                                         layers=[nm for L, nm in LAY.items() if EN.Contains(L) and t.IsOnLayer(L)]))
    elif t.Type() == pcbnew.PCB_ARC_T and t.GetLayer() in LAY:
        s, m, e = t.GetStart(), t.GetMid(), t.GetEnd()
        if inside(mm(s.x), mm(s.y)) or inside(mm(e.x), mm(e.y)):
            for a_, b_ in ((s, m), (m, e)):
                out['tracks'].append(dict(net=n, layer=LAY[t.GetLayer()], a=[mm(a_.x), mm(a_.y)], b=[mm(b_.x), mm(b_.y)], w=mm(t.GetWidth()), uuid=t.m_Uuid.AsString(), arc=True))
    elif t.Type() == pcbnew.PCB_TRACE_T and t.GetLayer() in LAY:
        s, e = t.GetStart(), t.GetEnd()
        if inside(mm(s.x), mm(s.y)) or inside(mm(e.x), mm(e.y)):
            out['tracks'].append(dict(net=n, layer=LAY[t.GetLayer()], a=[mm(s.x), mm(s.y)], b=[mm(e.x), mm(e.y)], w=mm(t.GetWidth()), uuid=t.m_Uuid.AsString()))
TYPE = {pcbnew.PAD_ATTRIB_SMD: 'smd', pcbnew.PAD_ATTRIB_PTH: 'pth', pcbnew.PAD_ATTRIB_NPTH: 'npth', pcbnew.PAD_ATTRIB_CONN: 'conn'}
out['net_ties'] = []
for f in b.GetFootprints():
    if f.IsNetTie(): out['net_ties'].append(dict(ref=f.GetReference(), nets=sorted({p.GetNetname() for p in f.Pads() if p.GetNetname()})))
    for p in f.Pads():
        q = p.GetPosition()
        if not inside(mm(q.x), mm(q.y), 2): continue
        for L, nm in LAY.items():
            if not p.IsOnLayer(L): continue
            sp = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(sp, L, 0, pcbnew.FromMM(0.002), pcbnew.ERROR_INSIDE)
            for k in range(sp.OutlineCount()):
                o = sp.Outline(k); pts = [[mm(o.CPoint(i).x), mm(o.CPoint(i).y)] for i in range(o.PointCount())]

                out['pads'].append(dict(net=p.GetNetname(), layer=nm, ref=f.GetReference(), num=p.GetNumber(), poly=pts,
                                        x=mm(q.x), y=mm(q.y), type_=TYPE.get(p.GetAttribute(), '?'), fp=f.GetFPIDAsString()))
        if p.HasHole():
            out['holes'].append(dict(net=p.GetNetname(), x=mm(q.x), y=mm(q.y), d=mm(max(p.GetDrillSize().x, p.GetDrillSize().y))))
for z in b.Zones():
    if not z.GetIsRuleArea(): continue
    o = z.Outline()
    for k in range(o.OutlineCount()):
        ol = o.Outline(k); pts = [[mm(ol.CPoint(i).x), mm(ol.CPoint(i).y)] for i in range(ol.PointCount())]
        out['keepouts'].append(dict(tracks=z.GetDoNotAllowTracks(), vias=z.GetDoNotAllowVias(), layers=[LAY[L] for L in LAY if z.IsOnLayer(L)], poly=pts))
for f in b.GetFootprints():
    for z in f.Zones():
        if not z.GetIsRuleArea(): continue
        o = z.Outline()
        for k in range(o.OutlineCount()):
            ol = o.Outline(k); pts = [[mm(ol.CPoint(i).x), mm(ol.CPoint(i).y)] for i in range(ol.PointCount())]
            out['keepouts'].append(dict(tracks=z.GetDoNotAllowTracks(), vias=z.GetDoNotAllowVias(), layers=[LAY[L] for L in LAY if z.IsOnLayer(L)], poly=pts))

edge = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(edge, True)
def ring(o): return [[mm(o.CPoint(i).x), mm(o.CPoint(i).y)] for i in range(o.PointCount())]
def touched(pts): return not (max(q[0] for q in pts) < X0 - 1 or min(q[0] for q in pts) > X1 + 1 or max(q[1] for q in pts) < Y0 - 1 or min(q[1] for q in pts) > Y1 + 1)
for k in range(edge.OutlineCount()):
    for o in [edge.Outline(k)] + [edge.Hole(k, h) for h in range(edge.HoleCount(k))]:
        pts = ring(o)
        if touched(pts): out['edges'].append(pts)
json.dump(out, open(sys.argv[6], 'w'))
print({k: len(v) for k, v in out.items()})
