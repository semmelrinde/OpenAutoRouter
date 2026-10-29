import pcbnew, sys, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kicad.kicad_protection import activate
activate()
from common.track_items import validate
import wx
APP = wx.GetApp() or wx.App(False)
_log_goal = wx.LogStderr(); wx.Log.SetActiveTarget(_log_goal)
from project_copy import design_rules_save, design_rules_copy, design_rules_goal_clear
base, rj, zj, out = sys.argv[1:5]
_design_rules = design_rules_save(base)
_project_verify = dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x).get('project') == '1'
if _project_verify:


    _manager = pcbnew.GetSettingsManager()
    _pro = str(Path(base).resolve().with_suffix('.kicad_pro'))
    _project = _manager.GetProject(_pro)
    if _project is None:
        if not _manager.LoadProject(_pro): raise ValueError('Project cannot be loaded: ' + _pro)
        _project = _manager.GetProject(_pro)
    if _project is None: raise ValueError('Loaded project missing: ' + _pro)
b = pcbnew.LoadBoard(base)
if b is None: raise ValueError('KiCad base board not readable: '+base)
if _project_verify:

    if b.GetProject() is None: raise ValueError('Project of the base board missing: ' + _pro)
    b.SynchronizeNetsAndNetClasses(False)
NLAY = int(dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x).get('cu_layers', '0'))
if NLAY: b.SetCopperLayerCount(NLAY)
ZCLR = float(dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x).get('zclr', '0.2'))
VD, VDR = [float(q) for q in dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x).get('via', '0.45/0.20').split('/')]
R = json.load(open(rj)); Z = json.load(open(zj))
if R.get('rest_board'):
    from kicad.rest_placement import native_placement
    native_placement(b, R, pcbnew)


zone_keys = {json.dumps(z, sort_keys=True) for z in Z}
for z in R.get('zones', []):
    if (z.get('copper_bars') or z.get('r36')) and json.dumps(z, sort_keys=True) not in zone_keys:
        Z.append(z)
        zone_keys.add(json.dumps(z, sort_keys=True))
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu}
LAY.update({'In%d' % k: getattr(pcbnew, 'In%d_Cu' % k) for k in range(1, 31)})
EXPORT_NM = (R.get('options') or {}).get('export_nm', '0')
if EXPORT_NM not in ('0', '1'): raise ValueError('export_nm must be 0 or 1')
def MM(v):


    return int(round(float(v) * 1000000)) if EXPORT_NM == '1' else pcbnew.FromMM(v)
def V(x, y): return pcbnew.VECTOR2I(MM(x), MM(y))


PREV = list(b.GetTracks()) + [z for z in b.Zones() if not z.GetIsRuleArea()]
nets = {}
def net(n):
    if n not in nets: nets[n] = b.FindNet(n)
    return nets[n]

for t in R['tracks']:
    validate(t,error=float((R.get('organic_post_run') or {}).get('chord_error_mm',.00025)))
ns = na = 0
for t in R['tracks']:
    if 'items' in t:
        for it in t['items']:
            if it[0] == 'L':
                a, c, w = it[1], it[2], it[3]
                if abs(a[0] - c[0]) < 1e-6 and abs(a[1] - c[1]) < 1e-6: continue
                s = pcbnew.PCB_TRACK(b); s.SetStart(V(*a)); s.SetEnd(V(*c))
            elif it[0] == 'A':
                na += 1
                a, m, c, w = it[1], it[2], it[3], it[4]
                s = pcbnew.PCB_ARC(b); s.SetStart(V(*a)); s.SetMid(V(*m)); s.SetEnd(V(*c))
            s.SetWidth(MM(w)); s.SetLayer(LAY[t['layer']]); s.SetNet(net(t['net'])); b.Add(s); ns += 1
        continue
    for a, c, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
        if abs(a[0] - c[0]) < 1e-6 and abs(a[1] - c[1]) < 1e-6: continue
        s = pcbnew.PCB_TRACK(b); s.SetStart(V(*a)); s.SetEnd(V(*c)); s.SetWidth(MM(w)); s.SetLayer(LAY[t['layer']]); s.SetNet(net(t['net'])); b.Add(s); ns += 1
for v in R['vias']:
    vd, vdr = (v['d'], v['drill']) if R.get('native_vias') or v.get('copper_bars') or v.get('thermal_island') else (VD, VDR)
    vi = pcbnew.PCB_VIA(b); vi.SetPosition(V(v['x'], v['y'])); vi.SetWidth(MM(vd)); vi.SetDrill(MM(vdr)); vi.SetNet(net(v['net'])); b.Add(vi)
polygon_zones = []
NF = 0
imported_zone_ids = set()
for z in Z:
    source_id = z.get("zone_id") if z.get("native_layers") else (z.get("zone_id"), z["layer"])
    if z.get("native_outlines") and z.get("zone_id"):
        if source_id in imported_zone_ids: continue
        imported_zone_ids.add(source_id)
    if z.get('teardrop'):
        fo = pcbnew.PCB_SHAPE(b); fo.SetShape(pcbnew.SHAPE_T_POLY)
        ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
        for x, y in z['pts']: ps.Append(MM(x), MM(y))
        fo.SetPolyShape(ps); fo.SetFilled(True); fo.SetWidth(0); fo.SetLayer(LAY[z['layer']]); b.Add(fo); fo.SetNet(net(z['net']))
        NF += 1; continue
    zo = pcbnew.ZONE(b); zo.SetLayer(LAY[z['layer']]); zo.SetNet(net(z['net'])); zo.SetAssignedPriority(z.get('prio',1))
    if z.get('zone_id'): zo.SetUuid(pcbnew.KIID(z['zone_id']))
    if z.get('native_layers'):
        layer_set = pcbnew.LSET()
        for layer in z['native_layers']: layer_set.AddLayer(LAY[layer])
        zo.SetLayerSet(layer_set)
    ol = zo.Outline()
    for contour in z.get('native_outlines') or [z]:
        oi = ol.NewOutline()
        for x, y in contour['pts']: ol.Append(MM(x), MM(y), oi)
        for ring in contour.get('holes', []):
            hi = ol.NewHole(oi)
            for x, y in ring: ol.Append(MM(x), MM(y), oi, hi)
    zo.SetMinThickness(MM(z.get('min_thickness',0.05 if z.get("current_path") else 0.2)))
    clr=z.get('local_clearance',ZCLR); zo.SetLocalClearance(None if clr is None else MM(clr))
    zo.SetIslandRemovalMode(z.get('island_removal',pcbnew.ISLAND_REMOVAL_MODE_ALWAYS))
    if 'fill_mode' in z: zo.SetFillMode(z['fill_mode'])
    if 'min_island_area_iu2' in z: zo.SetMinIslandArea(int(z['min_island_area_iu2']))
    for key, setter in [('hatch_thickness_iu', zo.SetHatchThickness), ('hatch_gap_iu', zo.SetHatchGap),
                        ('hatch_orientation', lambda value: zo.SetHatchOrientation(pcbnew.EDA_ANGLE(value, pcbnew.DEGREES_T))), ('hatch_smoothing_level', zo.SetHatchSmoothingLevel),
                        ('hatch_smoothing_value', zo.SetHatchSmoothingValue), ('hatch_hole_min_area', zo.SetHatchHoleMinArea),
                        ('hatch_border_algorithm', zo.SetHatchBorderAlgorithm)]:
        if key in z: setter(z[key])
    zo.SetPadConnection(z.get('pad_connection',pcbnew.ZONE_CONNECTION_FULL if z.get('full') else pcbnew.ZONE_CONNECTION_THERMAL))
    zo.SetThermalReliefGap(MM(z.get('thermal_gap',0.2))); zo.SetThermalReliefSpokeWidth(MM(z.get('thermal_spoke',0.3)))
    if 'island_area_min' in z: zo.SetMinIslandArea(int(z['island_area_min']))
    if 'fill_mode' in z: zo.SetFillMode(z['fill_mode'])
    for key,method in [('corner_smoothing','SetCornerSmoothingType'),('hatch_smoothing_level','SetHatchSmoothingLevel'),
                       ('hatch_smoothing_value','SetHatchSmoothingValue'),('hatch_hole_min_area','SetHatchHoleMinArea'),
                       ('hatch_border_algorithm','SetHatchBorderAlgorithm'),('hatch_style','SetHatchStyle')]:
        if key in z: getattr(zo,method)(z[key])
    for key,method in [('corner_radius','SetCornerRadius'),('hatch_thickness','SetHatchThickness'),('hatch_gap','SetHatchGap')]:
        if key in z: getattr(zo,method)(MM(z[key]))
    if 'hatch_orientation' in z: zo.SetHatchOrientation(pcbnew.EDA_ANGLE(z['hatch_orientation'],pcbnew.DEGREES_T))
    b.Add(zo)
    if z.get('polygon_dc'): polygon_zones.append(zo)
for o in PREV: b.Remove(o)
_fill = dict(x.split('=', 1) for x in sys.argv[5:] if '=' in x).get('fill') != '0'
if _fill: pcbnew.ZONE_FILLER(b).Fill(b.Zones())


if polygon_zones and _fill:
    b.BuildConnectivity()
    isolated = [z for z in polygon_zones
                if all(q.m_Uuid == z.m_Uuid for q in b.GetConnectivity().GetConnectedItems(z))]
    for z in isolated:
        print('Polygon without native contact removed:', z.GetNetname(), z.GetPosition())
        b.Remove(z)
    if isolated:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
design_rules_goal_clear(out)
pcbnew.SaveBoard(out, b, True) if _project_verify else b.Save(out)
if _project_verify and '.kicad_pro' not in _design_rules:
    _manager.SaveProjectCopy(str(Path(out).with_suffix('.kicad_pro')), b.GetProject())
design_rules_copy(_design_rules, out, generated_pro=True)
print('Segments', ns, 'true arcs', na, 'Vias', len(R['vias']), 'Zones', len(Z) - NF, 'Fillet-Shapes', NF, '->', out)
