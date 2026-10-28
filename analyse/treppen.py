""
import json, math, sys
from collections import defaultdict
H = json.load(open(sys.argv[1])); opt = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
ST = float(opt.get('stueck', '0.6')); MK = int(opt.get('min_knicke', '3')); LISTE = int(opt.get('liste', '20'))
def rd(p): return (round(p[0], 4), round(p[1], 4))
vias = defaultdict(set)
for v in H['vias']: vias[v['net']].add(rd((v['x'], v['y'])))
seg = defaultdict(list)
for t in H['tracks']:
    if t.get('arc'): continue
    a, b = rd(t['a']), rd(t['b'])
    if a != b: seg[(t['net'], t['layer'])].append((a, b))
treppen = []
for (n, L), ss in sorted(seg.items()):
    nb = defaultdict(list)
    for k, (a, b) in enumerate(ss): nb[a].append(k); nb[b].append(k)
    def fest(p): return len(nb[p]) != 2 or p in vias[n]
    frei = set(range(len(ss))); ketten = []
    def lauf(p, k):
        weg = [p]
        while True:
            frei.discard(k); a, b = ss[k]; p = b if a == p else a; weg.append(p)
            if fest(p): return weg
            nx = [j for j in nb[p] if j in frei]
            if not nx: return weg
            k = nx[0]
    for p in sorted(nb):
        if fest(p):
            for k in list(nb[p]):
                if k in frei: ketten.append(lauf(p, k))
    for pts in ketten:

        q = [pts[0]]
        for p in pts[1:]:
            if len(q) >= 2:
                (x0, y0), (x1, y1) = q[-2], q[-1]
                if abs((x1 - x0) * (p[1] - y1) - (y1 - y0) * (p[0] - x1)) < 1e-7: q[-1] = p; continue
            q.append(p)
        if len(q) < 4: continue
        kn = []
        for i in range(1, len(q) - 1):
            (x0, y0), (x1, y1), (x2, y2) = q[i - 1], q[i], q[i + 1]
            c = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1); kn.append((i, 1 if c > 0 else -1))
        lauf_ = []
        def ende():
            if len(lauf_) >= MK:
                i0, i1 = lauf_[0] - 1, lauf_[-1] + 1
                treppen.append(dict(net=n, layer=L, knicke=len(lauf_), x=round((q[i0][0] + q[i1][0]) / 2, 2), y=round((q[i0][1] + q[i1][1]) / 2, 2),
                                    laenge=round(sum(math.dist(q[k], q[k + 1]) for k in range(i0, i1)), 2), von=q[i0], bis=q[i1]))
        for j, (i, s) in enumerate(kn):
            kurz = math.dist(q[i - 1], q[i]) < ST and math.dist(q[i], q[i + 1]) < ST
            if kurz and (not lauf_ or s != kn[j - 1][1]) and (not lauf_ or kn[j - 1][0] == lauf_[-1]): lauf_.append(i)
            else:
                ende(); lauf_ = [i] if kurz else []
        ende()
netze = sorted({t['net'] for t in treppen})
print('Treppen: %d in %d Netzen, %d Knicke' % (len(treppen), len(netze), sum(t['knicke'] for t in treppen)))
for t in sorted(treppen, key=lambda t: -t['knicke'])[:LISTE]:
    print('  %-4s %-28s %2d Knicke, %.2f mm, bei (%.2f|%.2f)' % (t['layer'], t['net'][-28:], t['knicke'], t['laenge'], t['x'], t['y']))
if opt.get('netze'): open(opt['netze'], 'w').write(';'.join(netze))
