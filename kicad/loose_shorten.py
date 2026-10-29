import pcbnew, sys, json, math
b = pcbnew.LoadBoard(sys.argv[1]); D = json.load(open(sys.argv[2])); NET = sys.argv[4] if len(sys.argv) > 4 else None
U = {i['uuid'] for v in D['violations'] if v['type'] == 'track_dangling' for i in v['items']}
SHORT = 0.1
mm = pcbnew.ToMM; MM = pcbnew.FromMM
by_id = {t.m_Uuid.AsString(): t for t in b.GetTracks()}
def anchors(t):
    a, c = t.GetStart(), t.GetEnd(); dx, dy = c.x - a.x, c.y - a.y; L2 = dx * dx + dy * dy
    if L2 == 0: return []
    out = []
    def add(P, tol):
        s = ((P.x - a.x) * dx + (P.y - a.y) * dy) / L2
        if -0.01 <= s <= 1.01:
            qx, qy = a.x + s * dx, a.y + s * dy
            if math.hypot(P.x - qx, P.y - qy) <= tol: out.append(min(1.0, max(0.0, s)))
    for u in b.GetTracks():
        if u.GetNetCode() != t.GetNetCode() or u.m_Uuid.AsString() == t.m_Uuid.AsString() or u.m_Uuid.AsString() in U: continue
        if u.Type() == pcbnew.PCB_VIA_T: add(u.GetPosition(), u.GetWidth(pcbnew.F_Cu) // 2)
        elif u.GetLayer() == t.GetLayer():
            for P in (u.GetStart(), u.GetEnd()): add(P, t.GetWidth() // 2)
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetCode() == t.GetNetCode() and p.IsOnLayer(t.GetLayer()):
                for k in range(0, 101):
                    s = k / 100.0; P = pcbnew.VECTOR2I(int(a.x + s * dx), int(a.y + s * dy))
                    if p.HitTest(P, 0): out.append(s)
    return sorted(out)
n_del = n_cut = 0
for uid in sorted(U):
    t = by_id.get(uid)
    if t is None or t.Type() != pcbnew.PCB_TRACE_T or (NET and t.GetNetname() != NET): continue
    if t.GetLength() < MM(SHORT): b.Remove(t); n_del += 1; continue
    s = anchors(t)
    if not s: b.Remove(t); n_del += 1; continue
    a, c = t.GetStart(), t.GetEnd(); dx, dy = c.x - a.x, c.y - a.y
    lo, hi = s[0], s[-1]
    tol = (t.GetWidth() / 2) / max(1, math.hypot(dx, dy))
    if lo > tol: t.SetStart(pcbnew.VECTOR2I(int(round(a.x + lo * dx)), int(round(a.y + lo * dy)))); n_cut += 1
    if hi < 1 - tol: t.SetEnd(pcbnew.VECTOR2I(int(round(a.x + hi * dx)), int(round(a.y + hi * dy)))); n_cut += 1
    if t.GetLength() < MM(0.005): b.Remove(t); n_del += 1
print('loose tracks: deleted', n_del, 'shortened', n_cut)
pcbnew.ZONE_FILLER(b).Fill(b.Zones()); b.Save(sys.argv[3])
