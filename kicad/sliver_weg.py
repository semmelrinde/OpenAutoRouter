""
import re, sys, subprocess, json, os, collections
src = sys.argv[1]
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
TMP = '/tmp/sliver_weg'
for ext in ('kicad_pro', 'kicad_dru'):
    q = src[:-len('kicad_pcb')] + ext
    if os.path.exists(q): subprocess.run(['cp', q, TMP + '.' + ext])
t = open(src).read()


def bloecke(t):
    out = []
    for m in re.finditer(r'\n\t\(gr_poly\b', t):
        i = m.start() + 1; d = 0
        for j in range(i, len(t)):
            if t[j] == '(': d += 1
            elif t[j] == ')':
                d -= 1
                if d == 0: out.append((i, j + 1)); break
    return [(a, e) for a, e in out if '(net "' in t[a:e]]


def ohne(t, B, weg):
    weg = set(weg); parts = []; last = 0
    for k, (a, e) in enumerate(B):
        if k in weg: parts.append(t[last:a]); last = e
    parts.append(t[last:]); return ''.join(parts)


def splitter(text):
    open(TMP + '.kicad_pcb', 'w').write(text)
    subprocess.run([KC, 'pcb', 'drc', '--format', 'json', '--severity-all', '--refill-zones', '-o', TMP + '.json', TMP + '.kicad_pcb'], capture_output=True)
    d = json.load(open(TMP + '.json'))
    return collections.Counter(v['description'].split('(')[-1].rstrip(')').replace('.Cu', '') for v in d['violations'] if v['type'] == 'copper_sliver')


B = bloecke(t)
basis = splitter(t)
print('Kupferformen', len(B), 'Splitter je Lage', dict(basis))
weg = []
for L, n in basis.items():
    idx = [k for k, (a, e) in enumerate(B) if '(layer "%s.Cu")' % L in t[a:e]]
    cache = {}
    vorher = list(weg)
    def zahl(menge):
        key = tuple(menge)
        if key not in cache: cache[key] = splitter(ohne(t, B, list(menge) + vorher)).get(L, 0)
        return cache[key]
    ziel = n
    def suche(menge):
        if not menge or zahl(menge) >= ziel: return
        if len(menge) == 1: weg.append(menge[0]); return
        h = len(menge) // 2; suche(menge[:h]); suche(menge[h:])
    suche(idx)
for k in weg:
    blk = t[B[k][0]:B[k][1]]
    pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-\d.e]+) ([-\d.e]+)\)', blk)]
    print('entfernt', re.search(r'\(net "([^"]*)"', blk).group(1), re.search(r'\(layer "([^"]*)"', blk).group(1),
          'bei %.3f %.3f' % (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)))
neu = ohne(t, B, weg)
rest = splitter(neu)
open(src, 'w').write(neu)
print('Splitter danach', dict(rest), '->', src)
