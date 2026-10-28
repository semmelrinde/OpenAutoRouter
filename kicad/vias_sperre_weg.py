""
import pcbnew, sys
src, out = sys.argv[1:3]
opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
NETZ = opt.get('netz', 'GND')
b = pcbnew.LoadBoard(src)
areas = [z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias()]
for fp in b.GetFootprints():
    areas += [z for z in fp.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias()]
weg = []
for t in b.GetTracks():
    if t.Type() != pcbnew.PCB_VIA_T or t.GetNetname() != NETZ: continue
    for z in areas:
        if any(z.IsOnLayer(L) and t.IsOnLayer(L) for L in b.GetEnabledLayers().CuStack()) and \
           z.Outline().Collide(t.GetPosition(), t.GetWidth(pcbnew.F_Cu) // 2):
            weg.append(t); break
for v in weg:
    print('Via entfernt (Sperrflaeche):', v.GetNetname(), round(pcbnew.ToMM(v.GetPosition().x), 3), round(pcbnew.ToMM(v.GetPosition().y), 3))
    b.Remove(v)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out)
print('entfernt', len(weg), '->', out)
