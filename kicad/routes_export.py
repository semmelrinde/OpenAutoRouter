""
import pcbnew, sys, json, os
import wx
app = wx.GetApp() or wx.App(False)
_log_ziel = wx.LogStderr(); wx.Log.SetActiveTarget(_log_ziel)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
voll = 'voll=1' in sys.argv[3:]
einheiten = 'einheiten=1' in sys.argv[3:]
b = pcbnew.LoadBoard(sys.argv[1]); ToMM = pcbnew.ToMM
cu = list(b.GetEnabledLayers().CuStack())
names = {lid: ('F' if i == 0 else 'B' if i == len(cu) - 1 else 'In%d' % i) for i, lid in enumerate(cu)}
def lname(l): return names[l]
tracks, vias = [], []
for t in b.GetTracks():
    n = t.GetNetname()
    if isinstance(t, pcbnew.PCB_VIA):
        p = t.GetPosition(); d=ToMM(t.GetWidth(pcbnew.F_Cu))
        v = dict(net=n, x=ToMM(p.x), y=ToMM(p.y), d=d)
        if voll: v.update(diameter=d, drill=ToMM(t.GetDrillValue()), layers=[names[L] for L in cu if t.IsOnLayer(L)])
        if einheiten: v['einheit'] = str(t.m_Uuid.AsString())
        vias.append(v)
        continue
    pts = [t.GetStart(), t.GetEnd()]
    if isinstance(t, pcbnew.PCB_ARC): pts = [t.GetStart(), t.GetMid(), t.GetEnd()]
    w = ToMM(t.GetWidth())
    xy = [[ToMM(q.x), ToMM(q.y)] for q in pts]
    if voll and isinstance(t, pcbnew.PCB_ARC):
        from gemeinsam.bogen import punkte
        xy = punkte(*xy)
    tracks.append(dict(net=n, layer=lname(t.GetLayer()), pts=xy, w=[w] * (len(xy) - 1), sig=False,
                       **({'items': [['A', *[[ToMM(q.x),ToMM(q.y)] for q in pts], w]]} if voll and isinstance(t,pcbnew.PCB_ARC) else {}),
                       **({'einheit': str(t.m_Uuid.AsString())} if einheiten else {})))
R = dict(tracks=tracks, vias=vias, conflicts={}, failed=[], quelle=('platine:' if voll else 'mensch:') + sys.argv[1])
if voll:
    from kicad.abschlusskupfer import zeichnungen
    R.update(fuellung_vollstaendig=True, fuellung_lagen=list(names.values()))
    R['zones'] = zeichnungen(b, names, einheiten)
    R['board_copper_vollstaendig'] = True
    R['abschlusskupfer_vertrag'] = 'native-abschlusskupfer-v2'
    for zi, z in enumerate(z for z in b.Zones() if not z.GetIsRuleArea()):
        if z.GetIsRuleArea(): continue
        for L in cu:
            if not z.IsOnLayer(L): continue
            ps = z.GetFilledPolysList(L)
            if ps is None: continue
            for i in range(ps.OutlineCount()):
                o=ps.Outline(i)
                xy=lambda c: [[ToMM(c.CPoint(k).x),ToMM(c.CPoint(k).y)] for k in range(c.PointCount())]
                outline = z.Outline()
                clr = z.GetLocalClearance()
                meta = dict(native_layers=[lname(k) for k in cu if z.IsOnLayer(k)], zone_id=str(z.m_Uuid.AsString()), zone_index=zi, prio=z.GetAssignedPriority(),
                            local_clearance=ToMM(clr) if clr is not None else None,
                            min_thickness=ToMM(z.GetMinThickness()), pad_connection=int(z.GetPadConnection()),
                            island_removal=int(z.GetIslandRemovalMode()),
                            island_area_min=z.GetMinIslandArea(), fill_mode=z.GetFillMode(),
                            thermal_gap=ToMM(z.GetThermalReliefGap()),
                            thermal_spoke=ToMM(z.GetThermalReliefSpokeWidth()),
                            corner_smoothing=int(z.GetCornerSmoothingType()), corner_radius=ToMM(z.GetCornerRadius()),
                            hatch_thickness=ToMM(z.GetHatchThickness()), hatch_gap=ToMM(z.GetHatchGap()),
                            hatch_orientation=z.GetHatchOrientation().AsDegrees(),
                            hatch_smoothing_level=z.GetHatchSmoothingLevel(), hatch_smoothing_value=z.GetHatchSmoothingValue(),
                            hatch_hole_min_area=z.GetHatchHoleMinArea(), hatch_border_algorithm=z.GetHatchBorderAlgorithm(),
                            hatch_style=int(z.GetHatchStyle()),
                            native_outlines=[dict(pts=xy(outline.Outline(j)),
                                holes=[xy(outline.Hole(j,h)) for h in range(outline.HoleCount(j))])
                                for j in range(outline.OutlineCount())])
                R['zones'].append(dict(net=z.GetNetname(),layer=lname(L),filled=True,pts=xy(o),
                                       holes=[xy(ps.Hole(i,h)) for h in range(ps.HoleCount(i))], **meta, **({'einheit': str(z.m_Uuid.AsString())} if einheiten else {})))
with open(sys.argv[2], 'w') as f: json.dump(R, f)
print('Bahnen', len(tracks), 'Vias', len(vias))
