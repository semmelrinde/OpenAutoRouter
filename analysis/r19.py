import math
from collections import defaultdict

from analysis.wobble import _length, _mst, _moves, chain_list, nodes_bends


def shape_size(r):
    tracks = [dict(t, net=t.get('net', r.get('net', ''))) for t in r.get('tracks', [])]
    chains = chain_list(dict(tracks=tracks))
    return dict(length_mm=sum(_length(t['pts']) for t in tracks),
                move_records=sum(_moves(pts) for _, pts in chains))


def shape_ok(prior, new):
    a, b = shape_size(prior), shape_size(new)
    return b['length_mm'] <= a['length_mm'] + 1e-9 and b['move_records'] <= a['move_records']


def bend_excess45(R, *, chains=None):
    op = R.get('options') or {}
    out = dict(bend_excess_45=None, partial_chains_45=None, bend_excess_45_version='r19-bend_excess-v1')
    if op.get('style') == 'organic' or not (op.get('style') == '45' or str(op.get('angle45')) == '1'):
        return out
    excess = 0
    if chains is None: chains = chain_list(R)
    for _, pts in chains:
        a, b = pts[0], pts[-1]
        dx, dy = abs(b[0]-a[0]), abs(b[1]-a[1])
        ideal = 0 if min(dx, dy) < 1e-6 or abs(dx-dy) < 1e-6 else 1
        excess += max(0, _moves(pts)-ideal)
    return dict(out, bend_excess_45=excess, partial_chains_45=len(chains))


def outliers(B, R, *, chains=None):
    lengths, bends, pads, vias = defaultdict(float), defaultdict(int), defaultdict(list), defaultdict(int)
    if chains is None: chains = chain_list(R)
    bends.update(nodes_bends(chains))
    for (n, _), pts in chains:
        lengths[n] += _length(pts)
        bends[n] += _moves(pts)
    for p in B['pads']:
        pads[p['net']].append((p['x'], p['y']))
    for v in R.get('vias', []):
        vias[v['net']] += 1
    ratios = []; details = []
    for n, length in sorted(lengths.items()):
        mst = _mst(sorted(set(pads[n])))
        if mst >= 1.0 and length > 0:
            ratios.append(length / mst)
        details.append(dict(net_name=n, length_mm=length, pad_mst_mm=mst,
                            detour=length/mst if mst >= 1.0 and length > 0 else None,
                            move_records=bends[n], vias=vias[n]))
    ratios.sort()
    return dict(version='r19-net_list-v2', net_list=len(lengths), detour_measurable=len(ratios),
                detour_unmeasurable=len(lengths) - len(ratios),
                detour_max=max(ratios) if ratios else None,
                detour_p95=ratios[max(0, math.ceil(.95 * len(ratios)) - 1)] if ratios else None,
                moves_max=max(bends.values(), default=0),
                vias_max=max(vias.values(), default=0),
                nets_more_than_two_vias=sum(v > 2 for v in vias.values()),
                worst=sorted(details, key=lambda d: (-(d['detour'] or 0), -d['move_records'], d['net_name']))[:8])
