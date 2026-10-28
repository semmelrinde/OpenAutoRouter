""
import math
from collections import defaultdict

from analyse.wackel import _laenge, _mst, _zuege, ketten, knoten_knicke


def formmasse(r):
    ""
    tracks = [dict(t, net=t.get('net', r.get('net', ''))) for t in r.get('tracks', [])]
    chains = ketten(dict(tracks=tracks))
    return dict(laenge_mm=sum(_laenge(t['pts']) for t in tracks),
                zuege=sum(_zuege(pts) for _, pts in chains))


def form_ok(alt, neu):
    ""
    a, b = formmasse(alt), formmasse(neu)
    return b['laenge_mm'] <= a['laenge_mm'] + 1e-9 and b['zuege'] <= a['zuege']


def knickexzess45(R, *, chains=None):
    ""
    op = R.get('optionen') or {}
    out = dict(knickexzess_45=None, teilketten_45=None, knickexzess_45_version='r19-knickexzess-v1')
    if op.get('stil') == 'organisch' or not (op.get('stil') == '45' or str(op.get('winkel45')) == '1'):
        return out
    excess = 0
    if chains is None: chains = ketten(R)
    for _, pts in chains:
        a, b = pts[0], pts[-1]
        dx, dy = abs(b[0]-a[0]), abs(b[1]-a[1])
        ideal = 0 if min(dx, dy) < 1e-6 or abs(dx-dy) < 1e-6 else 1
        excess += max(0, _zuege(pts)-ideal)
    return dict(out, knickexzess_45=excess, teilketten_45=len(chains))


def ausreisser(B, R, *, chains=None):
    ""
    lengths, bends, pads, vias = defaultdict(float), defaultdict(int), defaultdict(list), defaultdict(int)
    if chains is None: chains = ketten(R)
    bends.update(knoten_knicke(chains))
    for (n, _), pts in chains:
        lengths[n] += _laenge(pts)
        bends[n] += _zuege(pts)
    for p in B['pads']:
        pads[p['net']].append((p['x'], p['y']))
    for v in R.get('vias', []):
        vias[v['net']] += 1
    ratios = []; details = []
    for n, length in sorted(lengths.items()):
        mst = _mst(sorted(set(pads[n])))
        if mst >= 1.0 and length > 0:
            ratios.append(length / mst)
        details.append(dict(netz=n, laenge_mm=length, pad_mst_mm=mst,
                            umweg=length/mst if mst >= 1.0 and length > 0 else None,
                            zuege=bends[n], vias=vias[n]))
    ratios.sort()
    return dict(version='r19-netze-v2', netze=len(lengths), umweg_messbar=len(ratios),
                umweg_unmessbar=len(lengths) - len(ratios),
                umweg_max=max(ratios) if ratios else None,
                umweg_p95=ratios[max(0, math.ceil(.95 * len(ratios)) - 1)] if ratios else None,
                zuege_max=max(bends.values(), default=0),
                vias_max=max(vias.values(), default=0),
                netze_mehr_als_zwei_vias=sum(v > 2 for v in vias.values()),
                schlechteste=sorted(details, key=lambda d: (-(d['umweg'] or 0), -d['zuege'], d['netz']))[:8])
