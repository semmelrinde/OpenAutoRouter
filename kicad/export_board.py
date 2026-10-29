"""Export a KiCad board to the JSON format used by the placer and router (run with KiCad's Python).

usage: <kicad-python> kicad/export_board.py <board.kicad_pcb> <board.json>
Writes pads (with polygons), footprints, courtyards, outline, keepouts, layers and net classes.
"""
import pcbnew, sys, json
import wx
_app = wx.GetApp() or wx.App(False)
_log_target = wx.LogStderr()
_log_previous = wx.Log.SetActiveTarget(_log_target)

src, out = sys.argv[1], sys.argv[2]
_output_copper = 'output_copper=1' in sys.argv[3:]
b = pcbnew.LoadBoard(src)


def mm(v): return round(pcbnew.ToMM(v), 5)


def poly_pts(sps):
    res = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        res.append([(mm(o.CPoint(k).x), mm(o.CPoint(k).y)) for k in range(o.PointCount())])
    return res


pads = []
footprint_copper = []
for f in b.GetFootprints():
    for p in f.Pads():
        cu = list(b.GetEnabledLayers().CuStack())
        names = {lid: ('F' if i == 0 else 'B' if i == len(cu) - 1 else 'In%d' % i)
                 for i, lid in enumerate(cu)}
        copper = {names[lid]: lid for lid in cu if p.IsOnLayer(lid)}
        lays = list(copper)
        polys = {}
        for L, lid in copper.items():
            sps = pcbnew.SHAPE_POLY_SET()
            p.TransformShapeToPolygon(sps, lid, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
            polys[L] = poly_pts(sps)
        bb = p.GetBoundingBox()
        hole = pcbnew.SHAPE_POLY_SET()
        drill_size = [0.0, 0.0]
        if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
            p.TransformHoleToPolygon(hole, 0, pcbnew.FromMM(0.001), pcbnew.ERROR_OUTSIDE)
            drill_size = [mm(p.GetDrillSize().x), mm(p.GetDrillSize().y)]
        mask = {}
        for L, ml in (('F', pcbnew.F_Mask), ('B', pcbnew.B_Mask)):
            if L in polys and p.IsOnLayer(ml):
                try:
                    e = mm(p.GetSolderMaskExpansion(ml))
                    mask[L] = round(e, 4)
                except Exception: pass
        pads.append(dict(ref=f.GetReference(), num=p.GetNumber(), net=p.GetNetname(), layers=lays, polys=polys, mask=mask,
                         padstack_complete=True, hole_polys=poly_pts(hole), drill_size=drill_size,
                         dru_pad_type={pcbnew.PAD_ATTRIB_PTH:'Through-hole',pcbnew.PAD_ATTRIB_SMD:'SMD',pcbnew.PAD_ATTRIB_CONN:'Edge connector',pcbnew.PAD_ATTRIB_NPTH:'NPTH, mechanical'}[p.GetAttribute()],
                         pad_type='npth' if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH else 'pth' if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else 'smd',
                         x=mm(p.GetPosition().x), y=mm(p.GetPosition().y),
                         bbox=[mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())],
                         drill=mm(p.GetDrillSize().x) if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) else 0.0,
                         fp_side='B' if f.IsFlipped() else 'F',
                         pinfunc=p.GetPinFunction(), pintype=p.GetPinType()))

via_keepout = []
for f in b.GetFootprints():
    ref = f.GetReference()
    if not (f.IsNetTie() or ref.startswith('NT_')): continue
    for g in f.GraphicalItems():
        L = g.GetLayer()
        if L not in (pcbnew.F_Cu, pcbnew.B_Cu): continue
        sps = pcbnew.SHAPE_POLY_SET(); g.TransformShapeToPolygon(sps, L, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
        via_keepout.append(dict(ref=ref, polys=poly_pts(sps)))
        for pq in f.Pads():
            if not pq.IsOnLayer(L): continue
            c = pcbnew.SHAPE_POLY_SET(); pq.TransformShapeToPolygon(c, L, pcbnew.FromMM(0.01), pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
            sps.BooleanSubtract(c)
        if not sps.OutlineCount(): continue
        side = 'F' if L == pcbnew.F_Cu else 'B'
        bb = sps.BBox(); k = ref[3:] if ref.startswith('NT_') else ref
        pads.append(dict(ref='NTBAR_' + k, num='1', net='#NTBAR_' + k, layers=[side], polys={side: poly_pts(sps)},
                         x=mm(bb.Centre().x), y=mm(bb.Centre().y), bbox=[mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())],
                         drill=0.0, fp_side=side, pinfunc='', pintype=''))


_cu = list(b.GetEnabledLayers().CuStack()); cu_ids = set(_cu)
cu_name = {lid: ('F' if i == 0 else 'B' if i == len(_cu) - 1 else 'In%d' % i) for i, lid in enumerate(_cu)}
copper_graphic = []
for k_, d in enumerate(b.GetDrawings()):
    L = d.GetLayer()
    if not pcbnew.IsCopperLayer(L) or L not in cu_ids: continue
    try:
        sps = pcbnew.SHAPE_POLY_SET(); d.TransformShapeToPolygon(sps, L, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
    except Exception:
        continue
    if not sps.OutlineCount(): continue
    side = cu_name[L]; bb = sps.BBox()
    if not bb.Intersects(b.GetBoardEdgesBoundingBox()): continue
    try: net_name = d.GetNetname() if hasattr(d, 'GetNetName') else ''
    except Exception: net_name = ''
    copper_graphic.append(dict(layer=side, net=net_name, polys=poly_pts(sps)))
    pads.append(dict(ref='CU_%d' % k_, num='1', net=(net_name or '#CU_%d' % k_) if _output_copper else '#CU_%d' % k_,
                     **({'board_copper': True, 'native_uuid': str(d.m_Uuid.AsString())} if _output_copper else {}), layers=[side], polys={side: poly_pts(sps)},
                     x=mm(bb.Centre().x), y=mm(bb.Centre().y), bbox=[mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())],
                     drill=0.0, fp_side=side, pinfunc='', pintype=''))


for f in b.GetFootprints():
    if f.IsNetTie() or f.GetReference().startswith('NT_'): continue
    for gi, g in enumerate(f.GraphicalItems()):
        if g.GetLayer() not in cu_ids: continue
        sps = pcbnew.SHAPE_POLY_SET()
        g.TransformShapeToPolygon(sps, g.GetLayer(), 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
        if not sps.OutlineCount(): continue
        side = cu_name[g.GetLayer()]; bb = sps.BBox()
        footprint_copper.append(dict(ref=f.GetReference(), num='#CU%d' % gi, net='#FPCU_%s_%d' % (f.GetReference(), gi),
                         layers=[side], polys={side: poly_pts(sps)}, mask={}, graphic=True,
                         x=mm(bb.Centre().x), y=mm(bb.Centre().y),
                         bbox=[mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())],
                         drill=0.0, fp_side='B' if f.IsFlipped() else 'F', pinfunc='', pintype=''))


mask_graphics = []
for f, graphics in [(f, f.GraphicalItems()) for f in b.GetFootprints()] + [(None, b.GetDrawings())]:
    for gi, g in enumerate(graphics):
        if g.GetLayer() not in (pcbnew.F_Mask, pcbnew.B_Mask): continue
        sps = pcbnew.SHAPE_POLY_SET()
        g.TransformShapeToPolygon(sps, g.GetLayer(), 0, pcbnew.FromMM(0.005), pcbnew.ERROR_OUTSIDE)
        if not sps.OutlineCount(): continue
        bb = sps.BBox(); side = 'F' if g.GetLayer() == pcbnew.F_Mask else 'B'
        ref = f.GetReference() if f else '#BOARD_MASK'
        mask_graphics.append(dict(ref=ref, num='#MASK%d' % gi, net='#MASK_%s_%d' % (ref, gi),
                                  layers=[side], polys={side: poly_pts(sps)}, mask={}, graphic=True,
                                  x=mm(bb.Centre().x), y=mm(bb.Centre().y),
                                  bbox=[mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())],
                                  drill=0.0, fp_side='B' if f and f.IsFlipped() else 'F', pinfunc='', pintype=''))

edge = pcbnew.SHAPE_POLY_SET()
b.GetBoardPolygonOutlines(edge, True)
outline = poly_pts(edge)
outline_holes = [[[(mm(h.CPoint(k).x), mm(h.CPoint(k).y)) for k in range(h.PointCount())]
                  for h in (edge.Hole(i, j) for j in range(edge.HoleCount(i)))] for i in range(edge.OutlineCount())]

crt = []
for f in b.GetFootprints():
    for side, lid in (('F', pcbnew.F_CrtYd), ('B', pcbnew.B_CrtYd)):
        cy = f.GetCourtyard(lid)
        if cy.OutlineCount():
            crt.append(dict(ref=f.GetReference(), side=side, polys=poly_pts(cy)))

def rule_range(z, ref):
    vb = {}
    for k_, f_ in (('track_list', 'GetDoNotAllowTracks'), ('vias', 'GetDoNotAllowVias'), ('pads', 'GetDoNotAllowPads'),
                   ('areas', 'GetDoNotAllowZoneFills'), ('areas', 'GetDoNotAllowCopperPour'), ('part_list', 'GetDoNotAllowFootprints')):
        if k_ in vb: continue
        try: vb[k_] = bool(getattr(z, f_)())
        except Exception: pass
    try: cu_layers = [cu_name[l] for l in z.GetLayerSet().Seq() if l in cu_name]
    except Exception: cu_layers = []
    o = z.Outline()
    lo = [[[(mm(h.CPoint(k).x), mm(h.CPoint(k).y)) for k in range(h.PointCount())] for h in (o.Hole(i, j) for j in range(o.HoleCount(i)))]
          for i in range(o.OutlineCount())]

    return dict(ref=ref, name=z.GetZoneName(), polys=poly_pts(o), cu_layers=cu_layers, prohibitions=vb, **({'hole_list': lo} if any(lo) else {}))
keepouts = []
for f in b.GetFootprints():
    for z in f.Zones():
        if z.GetIsRuleArea(): keepouts.append(rule_range(z, f.GetReference()))
for z in b.Zones():
    if z.GetIsRuleArea(): keepouts.append(rule_range(z, ''))
def attr(f):
    a = f.GetAttributes()
    return 'smd' if a & pcbnew.FP_SMD else ('tht' if a & pcbnew.FP_THROUGH_HOLE else 'no')


def net_tie_groups(f):


    if not f.IsNetTie(): return []
    return [list(g) for g in sorted({tuple(sorted({q.GetNumber() for q in f.GetNetTiePads(p)}))
                                    for p in f.Pads() if f.GetNetTiePads(p)})]


fps = [dict(ref=f.GetReference(), x=mm(f.GetPosition().x), y=mm(f.GetPosition().y), rot=f.GetOrientationDegrees(),
            net_tie=bool(f.IsNetTie()), net_tie_pad_groups=net_tie_groups(f),
            side='B' if f.IsFlipped() else 'F', fp=str(f.GetFPID().GetLibItemName()), lib=str(f.GetFPID().GetLibNickname()),
            value=f.GetValue(), sheet=f.GetSheetname(), locked=bool(f.IsLocked()), attr=attr(f), dnp=bool(f.IsDNP()),
            board_only=bool(f.GetAttributes() & pcbnew.FP_BOARD_ONLY), n_pads=len(f.Pads())) for f in b.GetFootprints()]
netclass = {}
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNetname(): netclass[p.GetNetname()] = p.GetNetClassName()

ds = b.GetDesignSettings()
def _mm(a):
    try: return round(mm(getattr(ds, a)), 4)
    except Exception: return None
design = dict(min_width=_mm('m_TrackMinWidth'), min_spacing=_mm('m_MinClearance'), border=_mm('m_CopperEdgeClearance'),
              via_min=_mm('m_ViasMinSize'), bore_min=_mm('m_MinThroughDrill'), ring_min=_mm('m_ViasMinAnnularWidth'),
              hole_hole=_mm('m_HoleToHoleMin'), hole_spacing=_mm('m_HoleClearance'), mask=_mm('m_SolderMaskExpansion'),
              mask_web=_mm('m_SolderMaskMinWidth'))
def _nc(nc):
    d = {}
    for k, f in (('clr', 'GetClearance'), ('w', 'GetTrackWidth'), ('via_d', 'GetViaDiameter'), ('via_b', 'GetViaDrill')):
        try: d[k] = round(mm(getattr(nc, f)()), 4)
        except Exception: pass
    return d
netclasses_def = {}
try:
    ns = ds.m_NetSettings; netclasses_def['Default'] = _nc(ns.GetDefaultNetclass())
    for k, v in ns.GetNetclasses().items(): netclasses_def[str(k)] = _nc(v)
except Exception: pass

netclass_effective = {}
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNetname():


            nc = ds.m_NetSettings.GetEffectiveNetClass(p.GetNetname())
            netclass_effective[p.GetNetname()] = dict(_nc(nc), clr=pcbnew.ToMM(nc.GetClearance()))
kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
p3 = {}
if 'placement' in kw:
    placement = json.load(open(kw['placement']))
    if 'p3_pin_groups' in placement: p3['p3_pin_groups'] = placement['p3_pin_groups']
json.dump(dict(**p3, src=src, pads=pads, footprint_copper=footprint_copper, mask_graphics=mask_graphics, outline=outline, outline_holes=outline_holes, courtyards=crt, keepouts=keepouts, fps=fps, via_keepout=via_keepout,
               nets=sorted({p['net'] for p in pads}), netclass=netclass, netclass_effective=netclass_effective, net_classes=netclasses_def, design=design, copper_graphic=copper_graphic,
               cu_layers=b.GetCopperLayerCount()), open(out, 'w'))
print('Pads', len(pads), 'Nets', len({p['net'] for p in pads}), 'Contour-Points', sum(len(o) for o in outline))
