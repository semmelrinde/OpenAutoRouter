""
import json
import sys
import pcbnew
import wx
app = wx.GetApp() or wx.App(False)
log = wx.LogStderr()
wx.Log.SetActiveTarget(log)
b = pcbnew.LoadBoard(sys.argv[1])
cu = list(b.GetEnabledLayers().CuStack())
names = {lid: 'In%d' % i for i, lid in enumerate(cu) if 0 < i < len(cu)-1}
def ring(q):
    return [[pcbnew.ToMM(q.CPoint(i).x), pcbnew.ToMM(q.CPoint(i).y)] for i in range(q.PointCount())]
nativ = 'native_ebenen=1' in sys.argv[3:]
out = []
for z in b.Zones():
    if z.GetIsRuleArea(): continue
    for lid, name in names.items():
        if not z.IsOnLayer(lid): continue
        ps = z.Outline()
        for i in range(ps.OutlineCount()):
            zone = dict(net=z.GetNetname(), layer=name, pts=ring(ps.Outline(i)),
                        holes=[ring(ps.Hole(i, h)) for h in range(ps.HoleCount(i))],
                        prio=z.GetAssignedPriority(), full=False)
            if nativ:
                zone.update(dict(filled=[[ring(z.GetFilledPolysList(lid).Outline(j)),
                                     [ring(z.GetFilledPolysList(lid).Hole(j, h)) for h in range(z.GetFilledPolysList(lid).HoleCount(j))]]
                                    for j in range(z.GetFilledPolysList(lid).OutlineCount())],
                            min_thickness=pcbnew.ToMM(z.GetMinThickness()),
                            local_clearance=None if z.GetLocalClearance() is None else pcbnew.ToMM(z.GetLocalClearance()),
                            island_removal=z.GetIslandRemovalMode(), fill_mode=z.GetFillMode(),
                            min_island_area_iu2=z.GetMinIslandArea(), pad_connection=z.GetPadConnection(),
                            hatch_thickness_iu=z.GetHatchThickness(), hatch_gap_iu=z.GetHatchGap(),
                            hatch_orientation=z.GetHatchOrientation().AsDegrees(), hatch_smoothing_level=z.GetHatchSmoothingLevel(),
                            hatch_smoothing_value=z.GetHatchSmoothingValue(), hatch_hole_min_area=z.GetHatchHoleMinArea(),
                            hatch_border_algorithm=z.GetHatchBorderAlgorithm(),
                            thermal_gap=pcbnew.ToMM(z.GetThermalReliefGap()), thermal_spoke=pcbnew.ToMM(z.GetThermalReliefSpokeWidth())))
            out.append(zone)
pads = sorted([f.GetReference(), p.GetNumber(), p.GetNetname(), round(pcbnew.ToMM(p.GetPosition().x), 3), round(pcbnew.ToMM(p.GetPosition().y), 3)] for f in b.GetFootprints() for p in f.Pads())
data = dict(zonen=out, pads=pads)
if nativ:
    tracks = [dict(net=t.GetNetname(), layer=names[t.GetLayer()]) for t in b.GetTracks()
              if not isinstance(t, pcbnew.PCB_VIA) and t.GetLayer() in names]
    data.update(lagen=len(cu), tracks=tracks)
json.dump(data, open(sys.argv[2], 'w'))
