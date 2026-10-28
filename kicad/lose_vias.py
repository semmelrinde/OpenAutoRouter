""
import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); D = json.load(open(sys.argv[2]))
U = {i['uuid'] for v in D['violations'] if v['type'] == 'via_dangling' for i in v['items'] if i.get('uuid')}
n = 0
for t in list(b.GetTracks()):
    if t.Type() == pcbnew.PCB_VIA_T and t.m_Uuid.AsString() in U: b.Remove(t); n += 1
if n: pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(sys.argv[3]); print('lose Vias entfernt:', n)
