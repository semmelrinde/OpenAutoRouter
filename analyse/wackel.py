""
import json, math, sys
from collections import defaultdict

from shapely.geometry import LineString
from shapely.ops import unary_union

TOL = 0.05
WINKEL = 20.0


def _laenge(pts):
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))


def _zuege(pts):
    if len(pts) < 3: return 0
    s = list(LineString(pts).simplify(TOL, preserve_topology=False).coords)
    n = 0
    for a, b, c in zip(s[:-2], s[1:-1], s[2:]):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0]); h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        d = abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
        if math.degrees(d) >= WINKEL: n += 1
    return n


def _mst(pts):
    if len(pts) < 2: return 0.0
    rest = list(pts[1:]); L = 0.0
    best = {i: math.dist(pts[0], p) for i, p in enumerate(rest)}
    while rest:
        i = min(best, key=best.get); L += best[i]; p = rest[i]
        del best[i]; rest[i] = None
        for j, q in enumerate(rest):
            if q is not None and j in best: best[j] = min(best[j], math.dist(p, q))
        if all(q is None for q in rest): break
    return L


def ketten(R):
    ""
    seg = defaultdict(list)
    for t in R.get('tracks', []):

        pts = [tuple(map(float, p)) for p in t['pts']]
        for a, b in zip(pts[:-1], pts[1:]):
            if a != b: seg[(t.get('net', ''), t['layer'])].append((a, b))
    out = []
    for key, raw in sorted(seg.items()):
        noded = unary_union([LineString([a, b]) for a, b in sorted(set(raw))])
        lines = list(getattr(noded, 'geoms', [noded]))


        S = sorted({tuple(sorted((a, b))) for line in lines
                    for a, b in zip(line.coords, list(line.coords)[1:]) if a != b})
        nb = defaultdict(list)
        for i, (a, b) in enumerate(S): nb[a].append(i); nb[b].append(i)
        frei = set(range(len(S)))
        def lauf(start, i):
            weg = [start]; p = start
            while True:
                frei.discard(i); a, b = S[i]; q = b if a == p else a; weg.append(q); p = q
                nxt = [j for j in nb[p] if j in frei]
                if len(nb[p]) != 2 or not nxt: return weg
                i = nxt[0]
        for p in sorted(nb):
            if len(nb[p]) != 2:
                for i in list(nb[p]):
                    if i in frei: out.append((key, lauf(p, i)))
        while frei:
            i = min(frei); out.append((key, lauf(S[i][0], i)))
    return out


def knoten_knicke(chains):
    ""
    ends = defaultdict(list)
    for key, pts in chains:
        for path in (pts, pts[::-1]):
            p = path[0]
            q = next((q for q in path[1:] if math.dist(p, q) >= TOL), path[-1])
            if q != p: ends[key, p].append(math.atan2(q[1]-p[1], q[0]-p[0]))
    counts = defaultdict(int)
    for (key, _), directions in ends.items():
        if len(directions) < 3: continue
        if any(WINKEL <= abs(math.degrees((a-b+math.pi) % (2*math.pi)-math.pi)) <= 180-WINKEL
               for i, a in enumerate(directions) for b in directions[i+1:]):
            counts[key[0]] += 1
    return counts


def messen(B, R, gnd=('GND',), *, chains=None):
    gnd = set(gnd)
    laenge = defaultdict(float); zuege = 0; L_ges = 0.0
    if chains is None: chains = ketten(R)
    zuege += sum(knoten_knicke(chains).values())
    for (n, L), pts in chains:
        l = _laenge(pts); laenge[n] += l; L_ges += l
        zuege += _zuege(pts)
    pads = defaultdict(list)
    for p in B['pads']:
        if p['net']: pads[p['net']].append((p['x'], p['y']))
    umw, gew, ueber = 0.0, 0.0, 0
    for n, l in laenge.items():
        if n in gnd or n.lstrip('/').upper() in ('GND', 'PGND', 'AGND', 'DGND'): continue
        m = _mst(sorted(set(pads.get(n, []))))
        if m < 1.0 or l <= 0: continue
        r = l / m; umw += r * l; gew += l; ueber += r > 1.5
    vias = defaultdict(int)
    for v in R.get('vias', []): vias[v.get('net', '')] += 1
    netze = max(1, len([n for n in laenge if n not in gnd]))
    return dict(laenge_mm=round(L_ges, 1), zuege=zuege, zuege_je_10mm=round(10 * zuege / max(L_ges, 1e-9), 3),
                umweg=round(umw / gew, 3) if gew else None, netze_umweg_ueber_1_5=ueber,
                vias=len(R.get('vias', [])), vias_je_netz=round(len(R.get('vias', [])) / netze, 2))


def main():
    a = sys.argv[1:]
    B = json.load(open(a[0])); out = dict(auto=messen(B, json.load(open(a[1]))))
    if len(a) > 2: out['mensch'] = messen(B, json.load(open(a[2])))
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
