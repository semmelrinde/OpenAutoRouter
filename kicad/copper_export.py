import pcbnew, sys, json, math
import wx

APP = wx.GetApp() or wx.App(False)
_log_goal = wx.LogStderr(); wx.Log.SetActiveTarget(_log_goal)

src, out = sys.argv[1], sys.argv[2]
opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
b = pcbnew.LoadBoard(src)
if opt.get('fill') == '1': pcbnew.ZONE_FILLER(b).Fill(b.Zones())


def mm(v): return round(pcbnew.ToMM(v), 6 if opt.get('polygons_exact') == '1' else 5)
cu = [L for L in b.GetEnabledLayers().CuStack()]
name = {L: pcbnew.BOARD.GetStandardLayerName(L).replace('.Cu', '') for L in cu}


def polyset(sps):
    res = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        holes = [[(mm(sps.Hole(i, h).CPoint(k).x), mm(sps.Hole(i, h).CPoint(k).y)) for k in range(sps.Hole(i, h).PointCount())]
                 for h in range(sps.HoleCount(i))]
        res.append(dict(outer=[(mm(o.CPoint(k).x), mm(o.CPoint(k).y)) for k in range(o.PointCount())], hole_list=holes))
    return res


zones = []
for z in b.Zones():
    if z.GetIsRuleArea(): continue
    for L in cu:
        if not z.IsOnLayer(L): continue
        fp = z.GetFilledPolysList(L)
        if fp is None or fp.OutlineCount() == 0: continue
        entry = dict(net=z.GetNetname(), layer=name[L], prio=z.GetAssignedPriority(), polys=polyset(fp))
        if opt.get('thermal_metadata') == '1':
            entry.update(zone_id=z.m_Uuid.AsString(), layer_name=b.GetLayerName(L), thermal_gap=mm(z.GetThermalReliefGap()),
                         thermal_spoke=mm(z.GetThermalReliefSpokeWidth()), min_thickness=mm(z.GetMinThickness()))
        zones.append(entry)
tracks = []; vias = []
for t in b.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        vias.append(dict(net=t.GetNetname(), x=mm(t.GetPosition().x), y=mm(t.GetPosition().y), d=mm(t.GetWidth(pcbnew.F_Cu)),
                         drill=mm(t.GetDrillValue())))
        continue
    pts = [(mm(t.GetStart().x), mm(t.GetStart().y))]
    if t.GetClass() == 'PCB_ARC':
        pts.append((mm(t.GetMid().x), mm(t.GetMid().y)))
    pts.append((mm(t.GetEnd().x), mm(t.GetEnd().y)))
    tracks.append(dict(net=t.GetNetname(), layer=name.get(t.GetLayer(), '?'), pts=pts, w=mm(t.GetWidth())))
pads = []
for f in b.GetFootprints():
    for p in f.Pads():
        pl = {}
        for L in cu:
            if not p.IsOnLayer(L): continue
            sps = pcbnew.SHAPE_POLY_SET()
            p.TransformShapeToPolygon(sps, L, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
            pl[name[L]] = polyset(sps)
        hole = pcbnew.SHAPE_POLY_SET()
        drill_size = [0.0, 0.0]
        if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
            p.TransformHoleToPolygon(hole, 0, pcbnew.FromMM(0.001), pcbnew.ERROR_OUTSIDE)
            drill_size = [mm(p.GetDrillSize().x), mm(p.GetDrillSize().y)]
        pads.append(dict(ref=f.GetReference(), num=p.GetNumber(), net=p.GetNetname(), x=mm(p.GetPosition().x), y=mm(p.GetPosition().y),
                         drill=mm(p.GetDrillSize().x) if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) else 0.0,
                         layers=sorted(pl), hole_polys=polyset(hole), drill_size=drill_size,
                         pad_type='npth' if p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH else 'pth' if p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH else 'smd', polys=pl))
if opt.get('thermal_metadata') == '1':
    for entry, native in zip(pads, (p for f in b.GetFootprints() for p in f.Pads())):
        entry['pad_id'] = native.m_Uuid.AsString()
shapes = []
for d in b.GetDrawings():
    if not d.IsOnCopperLayer() or not hasattr(d, 'GetNetName') or not d.GetNetname(): continue
    sps = pcbnew.SHAPE_POLY_SET(); d.TransformShapeToPolygon(sps, d.GetLayer(), 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    shapes.append(dict(net=d.GetNetname(), layer=name.get(d.GetLayer(), '?'), polys=polyset(sps)))
edge = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(edge, True)
json.dump(dict(src=src, cu_layers=[name[L] for L in cu], zones=zones, tracks=tracks, vias=vias, pads=pads, shapes=shapes, outline=polyset(edge)),
          open(out, 'w'))
print('Zones', len(zones), 'Tracks', len(tracks), 'Vias', len(vias), 'Pads', len(pads), 'CopperShapes', len(shapes))
