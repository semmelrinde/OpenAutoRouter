import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); D = json.load(open(sys.argv[2])); MM = pcbnew.FromMM
SKIP = set(n for n in (sys.argv[4].split(';') if len(sys.argv) > 4 else []) if n)
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu, **{'In%d' % i: getattr(pcbnew, 'In%d_Cu' % i) for i in range(1, 31) if hasattr(pcbnew, 'In%d_Cu' % i)}}
rm = set(D['remove']); n_rm = 0
for t in list(b.GetTracks()):
    if t.m_Uuid.AsString() in rm and t.GetNetname() not in SKIP: b.Remove(t); n_rm += 1
n_add = 0
for s in D['add']:
    if s['net'] in SKIP: continue
    t = pcbnew.PCB_TRACK(b); t.SetStart(pcbnew.VECTOR2I(MM(s['a'][0]), MM(s['a'][1]))); t.SetEnd(pcbnew.VECTOR2I(MM(s['b'][0]), MM(s['b'][1])))
    t.SetWidth(MM(s['w'])); t.SetLayer(LAY[s['layer']]); t.SetNet(b.FindNet(s['net'])); b.Add(t); n_add += 1
print('removed', n_rm, 'new', n_add, 'skipped', sorted(SKIP))
pcbnew.ZONE_FILLER(b).Fill(b.Zones()); b.Save(sys.argv[3])
