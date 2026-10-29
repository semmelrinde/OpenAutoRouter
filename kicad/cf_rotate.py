import pcbnew, sys, math
src, out, pre = sys.argv[1], sys.argv[2], tuple(sys.argv[3].split(','))
b = pcbnew.LoadBoard(src)
def xy(p): return (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y))
def cross(a, b2, c, d):
    def o(p, q, r): return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return o(a, b2, c) * o(a, b2, d) < 0 and o(c, d, a) * o(c, d, b2) < 0
fps = list(b.GetFootprints()); n = 0
for fp in fps:
    if not fp.GetReference().startswith(pre) or len(list(fp.Pads())) != 2: continue
    pads = list(fp.Pads()); left = []
    for p in pads:
        best = None
        for f2 in fps:
            if f2.GetReference() == fp.GetReference() or len(list(f2.Pads())) < 5: continue
            for q in f2.Pads():
                if q.GetNetname() == p.GetNetname() and q.GetNetname():
                    d = math.dist(xy(p), xy(q))
                    if best is None or d < best[0]: best = (d, q)
        left.append((p, best[1] if best else None))
    if any(q is None for _, q in left) or max(math.dist(xy(p), xy(q)) for p, q in left) > 2.5: continue
    (p1, q1), (p2, q2) = left
    if cross(xy(p1), xy(q1), xy(p2), xy(q2)):
        fp.Rotate(fp.GetPosition(), pcbnew.EDA_ANGLE(180.0, pcbnew.DEGREES_T)); n += 1
        print('rotated', fp.GetReference())
b.Save(out); print('rotated in total', n)
