""
import pcbnew, sys, math, collections
b = pcbnew.LoadBoard(sys.argv[1]); X0, Y0, X1, Y1 = map(float, sys.argv[2:6]); mm = pcbnew.ToMM
def K(p): return (round(mm(p.x), 3), round(mm(p.y), 3))
adj = collections.defaultdict(list)
for t in b.GetTracks():
    if t.Type() != pcbnew.PCB_TRACE_T: continue
    a, c = K(t.GetStart()), K(t.GetEnd())
    if not all(X0 <= q[0] <= X1 and Y0 <= q[1] <= Y1 for q in (a, c)): continue
    key = (t.GetNetname().split('/')[-1], b.GetLayerName(t.GetLayer()))
    adj[(key, a)].append(c); adj[(key, c)].append(a)
hair, stairs = [], []
for (key, p), nb in adj.items():
    if len(nb) != 2: continue
    v1 = (nb[0][0] - p[0], nb[0][1] - p[1]); v2 = (nb[1][0] - p[0], nb[1][1] - p[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < 1e-6 or l2 < 1e-6: continue
    ang = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / l1 / l2))))
    if ang < 100: hair.append((key, p, round(ang), round(l1, 3), round(l2, 3)))
    elif ang < 179 and min(l1, l2) < 0.12: stairs.append((key, p, round(min(l1, l2), 3)))
print('Haarnadeln/spitze Knicke (Innenwinkel <= 90):', len(hair))
for h in sorted(hair)[:40]: print('  ', h)
print('Knicke mit Stueck < 0,12 mm (Treppen):', len(stairs), collections.Counter(s[0][0] for s in stairs).most_common(10))
