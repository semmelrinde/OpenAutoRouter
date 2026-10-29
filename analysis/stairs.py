import json, math, sys
from collections import defaultdict
H = json.load(open(sys.argv[1])); opt = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
ST = float(opt.get('piece', '0.6')); MK = int(opt.get('min_bends', '3')); LIST = int(opt.get('list_', '20'))
def rd(p): return (round(p[0], 4), round(p[1], 4))
vias = defaultdict(set)
for v in H['vias']: vias[v['net']].add(rd((v['x'], v['y'])))
seg = defaultdict(list)
for t in H['tracks']:
    if t.get('arc'): continue
    a, b = rd(t['a']), rd(t['b'])
    if a != b: seg[(t['net'], t['layer'])].append((a, b))
stairs = []
for (n, L), ss in sorted(seg.items()):
    nb = defaultdict(list)
    for k, (a, b) in enumerate(ss): nb[a].append(k); nb[b].append(k)
    def pinned(p): return len(nb[p]) != 2 or p in vias[n]
    vacant = set(range(len(ss))); chain_list = []
    def job(p, k):
        way = [p]
        while True:
            vacant.discard(k); a, b = ss[k]; p = b if a == p else a; way.append(p)
            if pinned(p): return way
            nx = [j for j in nb[p] if j in vacant]
            if not nx: return way
            k = nx[0]
    for p in sorted(nb):
        if pinned(p):
            for k in list(nb[p]):
                if k in vacant: chain_list.append(job(p, k))
    for pts in chain_list:

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
        run_ = []
        def end():
            if len(run_) >= MK:
                i0, i1 = run_[0] - 1, run_[-1] + 1
                stairs.append(dict(net=n, layer=L, bends=len(run_), x=round((q[i0][0] + q[i1][0]) / 2, 2), y=round((q[i0][1] + q[i1][1]) / 2, 2),
                                    total_length=round(sum(math.dist(q[k], q[k + 1]) for k in range(i0, i1)), 2), from_=q[i0], until=q[i1]))
        for j, (i, s) in enumerate(kn):
            short = math.dist(q[i - 1], q[i]) < ST and math.dist(q[i], q[i + 1]) < ST
            if short and (not run_ or s != kn[j - 1][1]) and (not run_ or kn[j - 1][0] == run_[-1]): run_.append(i)
            else:
                end(); run_ = [i] if short else []
        end()
net_list = sorted({t['net'] for t in stairs})
print('Stairs: %d in %d nets, %d bends' % (len(stairs), len(net_list), sum(t['bends'] for t in stairs)))
for t in sorted(stairs, key=lambda t: -t['bends'])[:LIST]:
    print('  %-4s %-28s %2d bends, %.2f mm, at (%.2f|%.2f)' % (t['layer'], t['net'][-28:], t['bends'], t['total_length'], t['x'], t['y']))
if opt.get('net_list'): open(opt['net_list'], 'w').write(';'.join(net_list))
