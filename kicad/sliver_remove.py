import re, sys, subprocess, json, os, collections
src = sys.argv[1]
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
TMP = '/tmp/sliver_remove'
for ext in ('kicad_pro', 'kicad_dru'):
    q = src[:-len('kicad_pcb')] + ext
    if os.path.exists(q): subprocess.run(['cp', q, TMP + '.' + ext])
t = open(src).read()


def blocks(t):
    out = []
    for m in re.finditer(r'\n\t\(gr_poly\b', t):
        i = m.start() + 1; d = 0
        for j in range(i, len(t)):
            if t[j] == '(': d += 1
            elif t[j] == ')':
                d -= 1
                if d == 0: out.append((i, j + 1)); break
    return [(a, e) for a, e in out if '(net "' in t[a:e]]


def without(t, B, way):
    way = set(way); parts = []; load = 0
    for k, (a, e) in enumerate(B):
        if k in way: parts.append(t[load:a]); load = e
    parts.append(t[load:]); return ''.join(parts)


def splitter(text):
    open(TMP + '.kicad_pcb', 'w').write(text)
    subprocess.run([KC, 'pcb', 'drc', '--format', 'json', '--severity-all', '--refill-zones', '-o', TMP + '.json', TMP + '.kicad_pcb'], capture_output=True)
    d = json.load(open(TMP + '.json'))
    return collections.Counter(v['description'].split('(')[-1].rstrip(')').replace('.Cu', '') for v in d['violations'] if v['type'] == 'copper_sliver')


B = blocks(t)
basis = splitter(t)
print('CopperShapes', len(B), 'Slivers per layer', dict(basis))
way = []
for L, n in basis.items():
    idx = [k for k, (a, e) in enumerate(B) if '(layer "%s.Cu")' % L in t[a:e]]
    cache = {}
    before = list(way)
    def count(pool):
        key = tuple(pool)
        if key not in cache: cache[key] = splitter(without(t, B, list(pool) + before)).get(L, 0)
        return cache[key]
    goal = n
    def search(pool):
        if not pool or count(pool) >= goal: return
        if len(pool) == 1: way.append(pool[0]); return
        h = len(pool) // 2; search(pool[:h]); search(pool[h:])
    search(idx)
for k in way:
    blk = t[B[k][0]:B[k][1]]
    pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-\d.e]+) ([-\d.e]+)\)', blk)]
    print('removed', re.search(r'\(net "([^"]*)"', blk).group(1), re.search(r'\(layer "([^"]*)"', blk).group(1),
          'at %.3f %.3f' % (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)))
new = without(t, B, way)
rest = splitter(new)
open(src, 'w').write(new)
print('Slivers afterwards', dict(rest), '->', src)
