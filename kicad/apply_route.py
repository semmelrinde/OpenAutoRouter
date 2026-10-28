""
import pcbnew, sys, json, os
if os.path.realpath(sys.argv[4]) == os.path.realpath(sys.argv[1]) or (os.path.exists(sys.argv[4]) and os.path.samefile(sys.argv[1], sys.argv[4])):
    raise SystemExit('apply_route: Ausgabe muss eine neue Datei sein (nie ins Original speichern)')
b = pcbnew.LoadBoard(sys.argv[1]); R = json.load(open(sys.argv[2])); RM = set(u for u in sys.argv[3].split(',') if u)
MM = pcbnew.FromMM; LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu, **{'In%d' % i: getattr(pcbnew, 'In%d_Cu' % i) for i in range(1, 31) if hasattr(pcbnew, 'In%d_Cu' % i)}}
W0 = float(R.get('w') or 0.1); VD, VDR = [float(x) for x in (R.get('via') or (0.45, 0.2))]
n = 0
for t in list(b.GetTracks()):
    if t.m_Uuid.AsString() in RM: b.Remove(t); n += 1
net = b.FindNet(R['net'])
def V(p): return pcbnew.VECTOR2I(MM(p[0]), MM(p[1]))
k = 0
for t in R['tracks']:
    w = float(t.get('w') or W0)
    for a, c in zip(t['pts'][:-1], t['pts'][1:]):
        s = pcbnew.PCB_TRACK(b); s.SetStart(V(a)); s.SetEnd(V(c)); s.SetWidth(MM(w)); s.SetLayer(LAY[t['layer']]); s.SetNet(net); b.Add(s); k += 1
for p in R['vias']:
    v = pcbnew.PCB_VIA(b); v.SetPosition(V(p)); v.SetWidth(MM(VD)); v.SetDrill(MM(VDR)); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net); b.Add(v)
print('entfernt', n, 'neu Bahnen', k, 'Vias', len(R['vias']), 'Breite', W0, 'Via', VD, VDR)
if len(sys.argv) > 5: pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(sys.argv[4])
