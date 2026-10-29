import pcbnew, sys, json

src, vj, out = sys.argv[1:4]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
VD, VDR = [float(q) for q in opt.get('via', '0.45/0.20').split('/')]
b = pcbnew.LoadBoard(src)
MM = pcbnew.FromMM
J = json.load(open(vj))
VIAS = J if isinstance(J, list) else J.get('vias', [])
n = 0
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu}
LAY.update({'In%d' % k: getattr(pcbnew, 'In%d_Cu' % k) for k in range(1, 31)})
for sp in ([] if isinstance(J, list) else J.get('keepout_list', [])):
    z = pcbnew.ZONE(b); z.SetIsRuleArea(True); z.SetLayer(LAY[sp['layer']])
    (z.SetDoNotAllowZoneFills if hasattr(z, 'SetDoNotAllowZoneFills') else z.SetDoNotAllowCopperPour)(True); z.SetDoNotAllowTracks(False); z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
    ol = z.Outline(); ol.NewOutline()
    for x, y in sp['pts']: ol.Append(MM(x), MM(y))
    b.Add(z)
for v in VIAS:
    vi = pcbnew.PCB_VIA(b); vi.SetPosition(pcbnew.VECTOR2I(MM(v['x']), MM(v['y']))); vi.SetWidth(MM(VD)); vi.SetDrill(MM(VDR))
    vi.SetNet(b.FindNet(v['net'])); b.Add(vi); n += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out)
print('Vias placed', n, 'KeepoutAreas', 0 if isinstance(J, list) else len(J.get('keepout_list', [])), '->', out)
