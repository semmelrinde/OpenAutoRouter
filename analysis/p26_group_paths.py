import math
from analysis.density import movable, island_groups
from common.parts import load_parts


def measure(B, K, layer_name=None, part_list=None):
    meta = dict(version='p26-group_paths.v1', status='unknown', net_list=None,
                hpwl_mm=None, max_hpwl_mm=None, bundle=None)
    try:
        refs = {f['ref'] for f in B['fps']}

        movable(K, refs)
        island_groups(dict(K, part_list={r: dict(K['part_list'][r], pinned=False) for r in refs}), refs)
        owner = {r: r for r in refs}
        for a, island in sorted(K['islands'].items()):
            for r in {a, *island['members']}: owner[r] = a
        T = part_list if part_list is not None else load_parts(B, float((K.get('design_rules') or {}).get('placer', {}).get('pad_spacing', .15)))
        fps = {f['ref']: f for f in B['fps']}
        pos = {}; groups = {}
        for r in sorted(refs):
            s = (layer_name or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
            for _, n, x, y, _ in T[r].pads_abs(*s):
                if not n or n in K['gnd']: continue
                category = K['net_list'][n]['category']
                if category not in ('signal', 'fast', 'analog'): continue
                if not all(math.isfinite(v) for v in (x, y)): raise ValueError('Non-finite pad layer')
                pos.setdefault(n, []).append((x, y)); groups.setdefault(n, set()).add(owner[r])
        spans = []; pairs = {}
        for n, pts in sorted(pos.items()):
            if len(groups[n]) < 2: continue
            spans.append(max(x for x, y in pts) - min(x for x, y in pts)
                         + max(y for x, y in pts) - min(y for x, y in pts))
            gs = sorted(groups[n])
            for i, a in enumerate(gs):
                for b in gs[i + 1:]: pairs[a, b] = pairs.get((a, b), 0) + 1
        return dict(meta, status='ok' if spans else 'not_applicable', net_list=len(spans),
                    hpwl_mm=round(sum(spans), 6) if spans else None,
                    max_hpwl_mm=round(max(spans), 6) if spans else None,
                    bundle=sum(v >= 2 for v in pairs.values()))
    except (KeyError, ValueError, TypeError) as e:
        return dict(meta, faults=str(e))

