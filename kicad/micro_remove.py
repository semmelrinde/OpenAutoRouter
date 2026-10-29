import pcbnew, sys, collections
b = pcbnew.LoadBoard(sys.argv[1]); X0, Y0, X1, Y1, LIM = map(float, sys.argv[2:7]); mm = pcbnew.ToMM
c = collections.Counter(); n = 0
for t in list(b.GetTracks()):
    if t.Type() != pcbnew.PCB_TRACE_T: continue
    s, e = t.GetStart(), t.GetEnd()
    if not all(X0 <= mm(p.x) <= X1 and Y0 <= mm(p.y) <= Y1 for p in (s, e)): continue
    if mm(t.GetLength()) < LIM: b.Remove(t); n += 1; c[t.GetNetname().split('/')[-1]] += 1
print('removed', n, c.most_common(8))
pcbnew.ZONE_FILLER(b).Fill(b.Zones()); b.Save(sys.argv[7])
