""
import pcbnew, sys, itertools
b = pcbnew.LoadBoard(sys.argv[1])
def xy(p): return (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y))
def cross(a, b2, c, d):
    o = lambda p, q, r: (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return o(a, b2, c) * o(a, b2, d) < 0 and o(c, d, a) * o(c, d, b2) < 0
two = [f for f in b.GetFootprints() if len(list(f.Pads())) == 2 and all(p.GetNetname() for p in f.Pads())]
n = 0
for f1, f2 in itertools.combinations(two, 2):
    if f1.IsFlipped() != f2.IsFlipped(): continue
    p1 = {p.GetNetname(): xy(p) for p in f1.Pads()}; p2 = {p.GetNetname(): xy(p) for p in f2.Pads()}
    if set(p1) != set(p2) or len(p1) != 2 or '/GND' in p1: continue
    a, c = list(p1)
    if pcbnew.ToMM(f1.GetPosition().x - f2.GetPosition().x) ** 2 + pcbnew.ToMM(f1.GetPosition().y - f2.GetPosition().y) ** 2 > 16: continue
    if cross(p1[a], p2[a], p1[c], p2[c]):
        f2.SetOrientationDegrees(f2.GetOrientationDegrees() + 180); n += 1
        print('gedreht', f2.GetReference(), '(gekreuzt mit', f1.GetReference() + ')')
b.Save(sys.argv[2]); print('gedreht', n)
