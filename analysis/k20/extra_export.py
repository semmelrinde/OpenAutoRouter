import pcbnew, sys, json
src, out = sys.argv[1:3]
b = pcbnew.LoadBoard(src); ToMM = pcbnew.ToMM
CU = list(b.GetEnabledLayers().CuStack())
LN = {l: pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', '') for l in CU}
def ps_out(ps):
    res = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i)
        res.append([[(ToMM(o.CPoint(k).x), ToMM(o.CPoint(k).y)) for k in range(o.PointCount())],
                    [[(ToMM(h.CPoint(k).x), ToMM(h.CPoint(k).y)) for k in range(h.PointCount())] for h in (ps.Hole(i, j) for j in range(ps.HoleCount(i)))]])
    return res
ol = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(ol, True)
D = dict(layers=[LN[l] for l in CU], outline=ps_out(ol), keepouts=[], zones=[], tracks=[], vias=[])
for z in b.Zones():
    lay = [LN[l] for l in CU if z.IsOnLayer(l)]
    if z.GetIsRuleArea():
        ps = z.Outline()
        D['keepouts'].append(dict(layers=lay, no_copper=bool(z.GetDoNotAllowZoneFills()), no_tracks=bool(z.GetDoNotAllowTracks()), polys=ps_out(ps)))
        continue
    for l in CU:
        if not z.IsOnLayer(l): continue
        D['zones'].append(dict(net=z.GetNetname(), layer=LN[l], prio=z.GetAssignedPriority(), clearance=ToMM(z.GetLocalClearance() or 0) if z.GetLocalClearance() is not None else None,
                               min_width=ToMM(z.GetMinThickness()), fill=ps_out(z.GetFilledPolysList(l)), contour=ps_out(z.Outline())))
for t in b.GetTracks():
    if t.Type() == pcbnew.PCB_VIA_T:
        D['vias'].append(dict(net=t.GetNetname(), x=ToMM(t.GetPosition().x), y=ToMM(t.GetPosition().y), d=ToMM(t.GetWidth(pcbnew.F_Cu)), drill=ToMM(t.GetDrillValue()),
                              layers=[LN[l] for l in CU if t.IsOnLayer(l)]))
        continue
    D['tracks'].append(dict(net=t.GetNetname(), layer=LN[t.GetLayer()], a=(ToMM(t.GetStart().x), ToMM(t.GetStart().y)), b=(ToMM(t.GetEnd().x), ToMM(t.GetEnd().y)),
                            w=ToMM(t.GetWidth()), arc=t.Type() == pcbnew.PCB_ARC_T))
D['drills'] = []
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.HasHole():
            sz = p.GetDrillSize(); D['drills'].append(dict(x=ToMM(p.GetPosition().x), y=ToMM(p.GetPosition().y), d=ToMM(min(sz.x, sz.y)), dmax=ToMM(max(sz.x, sz.y)), npth=p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH, net=p.GetNetname()))
ds = b.GetDesignSettings()
D['min_clearance'] = ToMM(ds.m_MinClearance)
try: D['netclass_clearance'] = ToMM(ds.m_NetSettings.GetDefaultNetclass().GetClearance())
except Exception as e: D['netclass_clearance'] = None
json.dump(D, open(out, 'w'))
print('Zones', len(D['zones']), 'Tracks', len(D['tracks']), 'Vias', len(D['vias']), 'KeepoutAreas', len(D['keepouts']), 'Layers', D['layers'], 'clr', D['min_clearance'], D['netclass_clearance'])
