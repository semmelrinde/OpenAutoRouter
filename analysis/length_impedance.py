import collections
import heapq
import math
from common.flow import net_graph
from analysis.structure import models, normalize, reference_layers, reference_planes
from analysis.impedance_table import amount


def path_lengths(B, R, net, structure=None, all_pads=False, source_ref=None):
    tracks = [t for t in R.get('tracks', []) if t['net'] == net]
    vias = [v for v in R.get('vias', []) if v['net'] == net]
    pads = [p for p in B['pads'] if p['net'] == net]
    if len(pads) < 2: return []
    G = net_graph(tracks, vias, pads, tol=0.0002)
    names = ['F'] + ['In%d' % i for i in range(1, B.get('cu_layers', 4) - 1)] + ['B']
    a = normalize(structure, len(names)); heights = {names[0]: 0.0}
    cu = a['copper_mm']
    for i, d in enumerate(a['dielectrics']): heights[names[i + 1]] = heights[names[i]] + d + (cu[i] + cu[i+1])/2
    pmap = {('PAD', p['ref'], p['num']): p for p in pads}


    for u in list(G['adj']):
        G['adj'][u] = [(v, kid) for v, kid in G['adj'][u] if not (kid and kid[0] == 'v')]
    for vi, v in enumerate(vias):
        nodes = [(l, round(v['x'] / .0002) * .0002, round(v['y'] / .0002) * .0002) for l in names]
        for u, w in zip(nodes[:-1], nodes[1:]):
            G['adj'][u].append((w, ('v', vi))); G['adj'][w].append((u, ('v', vi)))
    def pad_path(p, node):


        dz = min(abs(heights[node[0]] - heights[l]) for l in p['layers'] if l in heights)
        return math.dist((p['x'], p['y']), node[1:]) + dz
    def cost(u, v, kid):
        if kid and kid[0] == 's':
            _, ti, si, f1, f2 = kid
            return math.dist(tracks[ti]['pts'][si], tracks[ti]['pts'][si + 1]) * (f2 - f1)
        if kid and kid[0] == 'v': return abs(heights[u[0]] - heights[v[0]])
        if u[0] == 'PAD' and v[0] != 'PAD': return pad_path(pmap[u], v)
        if v[0] == 'PAD' and u[0] != 'PAD': return pad_path(pmap[v], u)
        return 0.0
    ends = sorted(set(G['pad_pts'].values()))
    sources = [u for u in ends if pmap[u].get('pintype') in ('output', 'tri_state')]
    if source_ref is not None:
        sources = [u for u in ends if u[1] == source_ref]
        if not sources: return []

    if len(ends) == 2: sources = (sources or ends)[:1]
    if not sources and (len(ends) == 2 or all_pads): sources = ends[:1]
    if not sources: return []
    paths = []
    for src in sources:
        dist = {src: 0.0}; queue = [(0.0, src)]
        while queue:
            du, u = heapq.heappop(queue)
            if du != dist[u]: continue
            for v, kid in G['adj'].get(u, []):
                dv = du + cost(u, v, kid)
                if dv < dist.get(v, math.inf): dist[v] = dv; heapq.heappush(queue, (dv, v))
        for dst in ends:
            if dst == src: continue
            paths.append(dict(source=':'.join(src[1:]), receiver=':'.join(dst[1:]),
                              assignment=src[1] + '->' + dst[1], two_point=len(ends) == 2,
                              length_mm=dist.get(dst)))
    return paths


def path_length(B, R, net, structure=None):
    paths = path_lengths(B, R, net, structure)
    return max(p['length_mm'] for p in paths) if paths and all(p['length_mm'] is not None for p in paths) else None


def length_groups(K):
    groups = {g: dict(v, net_list=list(v['net_list'])) for g, v in ((K.get('specs') or {}).get('length_groups') or {}).items()}
    explicit_tol = {g for g, v in groups.items() if v.get('tolerance_mm') is not None}
    nets = K['net_list']
    for n, k in sorted(nets.items()):
        if k.get('length_group'):
            g = groups.setdefault(k['length_group'], dict(net_list=[], tolerance_mm=k.get('tolerance_mm', 0.15)))
            g['net_list'] = sorted(set(g.get('net_list', []) + [n]))
            if k['length_group'] not in explicit_tol and k.get('tolerance_mm') is not None: g['tolerance_mm'] = min(float(g.get('tolerance_mm', 0.15)), float(k['tolerance_mm']))
        if k.get('pair') and k.get('z_diff_ohm'):
            p = sorted([n, k['pair']]); name = 'pair:' + '|'.join(p)
            ts = [nets[q].get('pair_tolerance_mm', nets[q].get('tolerance_mm')) for q in p if q in nets]
            tol = min(float(t) for t in ts if t is not None) if any(t is not None for t in ts) else .15
            groups.setdefault(name, dict(net_list=p, tolerance_mm=tol))
        elif k.get('z_diff_ohm'):
            groups['pair_unknown:' + n] = dict(net_list=[n], tolerance_mm=0.15, pair_unknown=True)
    return groups


def length_list(B, R, K):
    A = normalize((K.get('specs') or {}).get('structure') or K.get('structure'), B.get('cu_layers', 4))
    groups = length_groups(K)
    nets = K['net_list']
    wanted = sorted({n for g in groups.values() for n in g['net_list']})
    pair_sources = {}
    for n in wanted:
        k = nets[n]; partner = k.get('pair')
        if not partner or k.get('pair_kind') != 'fast': continue
        pp = {q: collections.Counter(p['ref'] for p in B['pads'] if p['net'] == q) for q in (n, partner)}
        unique = sorted(r for r in set(pp[n]) & set(pp[partner]) if pp[n][r] == pp[partner][r] == 1)


        drivers = {p['ref'] for p in B['pads'] if p['net'] == n and p.get('pintype') in ('output', 'tri_state')}
        chips = [r for r in unique if r.startswith(('U', 'IC')) and all(
            any(p['net'] == q and p['ref'] == r and p.get('pintype') in ('output', 'tri_state', 'bidirectional')
                for p in B['pads']) for q in (n, partner))]
        explicit = k.get('source_ref')
        if explicit is not None: pair_sources[n] = explicit
        elif len(chips) == 1 and (not drivers or drivers == {chips[0]}): pair_sources[n] = chips[0]
    paths = {n: path_lengths(B, R, n, A, source_ref=nets[n].get('source_ref', pair_sources.get(n))) for n in wanted}
    lengths = {n: max(p['length_mm'] for p in pp) if pp and all(p['length_mm'] is not None for p in pp) else None for n, pp in paths.items()}
    out = {}
    for g, q in sorted(groups.items()):
        vals = {n: lengths[n] for n in q['net_list']}; tol = float(q.get('tolerance_mm', 0.15))
        if not math.isfinite(tol) or tol < 0: raise ValueError('Length tolerance must be finite and >= 0')
        keyed = {n: collections.defaultdict(list) for n in vals}
        for n in vals:
            for p in paths[n]:
                key = 'point_point' if g.startswith('pair:') and p['two_point'] else p['assignment']
                keyed[n][key].append(p['length_mm'])
        receivers = sorted({e for nn in keyed.values() for e in nn})
        comparisons = {}
        for e in receivers:
            vv = {n: keyed[n].get(e, []) for n in vals}



            usb_c_pins = all({p['receiver'].rsplit(':', 1)[1] for p in paths[n] if p['assignment'] == e}
                             in ({'A6', 'B6'}, {'A7', 'B7'}) for n in vals)
            if g.startswith('pair:') and all(n in pair_sources for n in vals) and usb_c_pins and len({len(v) for v in vv.values()}) == 1 and len(next(iter(vv.values()))) > 1:
                ordered = {n: sorted((p for p in paths[n] if p['assignment'] == e), key=lambda p: p['receiver']) for n in vals}
                for i in range(len(next(iter(vv.values())))):
                    lens = {n: ordered[n][i]['length_mm'] for n in vals}
                    ok = all(v is not None for v in lens.values())
                    comparisons[e + ':pin%d' % i] = dict(net_list={n: [v] for n,v in lens.items()},
                        error_mm=max(lens.values())-min(lens.values()) if ok else None, complete=ok,
                        pins={n: ordered[n][i]['receiver'] for n in vals})
                continue

            ok = all(len(v) == 1 and v[0] is not None for v in vv.values())
            err = max(v[0] for v in vv.values()) - min(v[0] for v in vv.values()) if ok else None
            comparisons[e] = dict(net_list=vv, error_mm=err, complete=ok)
        known = bool(receivers) and not q.get('pair_unknown') and all(v['complete'] for v in comparisons.values())
        error = max(v['error_mm'] for v in comparisons.values()) if known else None
        out[g] = dict(net_list=vals, tolerance_mm=tol, error_mm=error, complete=known,
                      paths={n: paths[n] for n in vals}, receiver=comparisons,
                      over_mm=max(0.0, error - tol) if known else None)
    return out


def receiver_protection(before, after, net_list=None, strict=False):
    for name, old in before.items():
        if net_list is not None and not set(old['net_list']).intersection(net_list): continue
        for receiver, a in old['receiver'].items():
            if not a['complete']: continue
            b = (after.get(name) or {}).get('receiver', {}).get(receiver)
            if b is None or not b['complete']: return False
            limit = a['error_mm'] if strict else max(a['error_mm'], old['tolerance_mm'])
            if b['error_mm'] > limit + 1e-6: return False
    return True


def receiver_netclasses(K):
    import copy
    out = copy.deepcopy(K)
    for k in out['net_list'].values():
        if k.get('pair') and k.get('pair_kind', k.get('category')) == 'fast':
            k['z_diff_ohm'] = k.get('z_diff_ohm') or 1
    return out


def measure(B, R, K):
    from shapely.geometry import LineString
    op = R.get('options') or {}; refs = reference_layers(R, K.get('gnd', []), K['net_list'])
    profiles = models((K.get('specs') or {}).get('structure') or K.get('structure'), B.get('cu_layers', int(op.get('cu_layers', 4))), refs)
    planes = reference_planes(B, K.get('gnd', []), R, K['net_list'])
    indexed = collections.defaultdict(list)
    for t in R.get('tracks', []):
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-9: indexed[t['net'], t['layer']].append((LineString([a, b]), w))
    out = {}; sumerr = total_len = 0.0; unknown = 0.0; incomplete = 0
    for n, k in sorted(K['net_list'].items()):
        diff = bool(k.get('z_diff_ohm')); target = k.get('z_diff_ohm') if diff else k.get('z_ohm')
        if not target: continue
        if not math.isfinite(float(target)) or target <= 0: raise ValueError('Target impedance must be finite and positive')
        err = length = missing = 0.0; worst = 0.0
        for (ne, layer), ss in sorted(indexed.items()):
            if ne != n: continue
            for line, w in ss:
                if layer not in profiles: missing += line.length; continue
                if planes is not None and any(ref not in planes or not planes[ref].covers(line.buffer(w/2)) for ref in profiles[layer]['ref']):
                    missing += line.length; continue
                samples = max(1, int(math.ceil(line.length / 1.0))) if diff else 1
                gaps = []
                if diff:
                    pp = indexed.get((k.get('pair'), layer), [])
                    if not pp: missing += line.length; continue
                    direction = tuple(b-a for a, b in zip(line.coords[0], line.coords[-1]))
                    parallel = [(l, pw) for l, pw in pp if abs(sum(x*(b-a) for x, a, b in zip(direction, l.coords[0], l.coords[-1]))) >= 0.98*line.length*l.length]
                    for i in range(samples):
                        center = line.interpolate((i+.5)/samples, normalized=True)
                        candidates = [(l, pw) for l, pw in parallel if 1e-5 < l.project(center) < l.length - 1e-5]
                        if not candidates: gaps.append(None); continue
                        partner, pw = min(candidates, key=lambda q: q[0].distance(center))

                        gaps.append(round(max(0.001, partner.distance(center) - (w + pw)/2), 3) if abs(pw-w) <= .001 else None)
                else: gaps = [0.0]
                for gap in gaps:
                    if gap is None: missing += line.length / samples; continue
                    if diff and gap > max(2.0, 8*profiles[layer]['h']):
                        actual = 2*amount(profiles[layer], w)
                    else: actual = amount(profiles[layer], w, gap, diff)
                    delta = abs(actual / target - 1.0); err += delta * line.length / samples; length += line.length / samples; worst = max(worst, delta)
        paths = path_lengths(B, R, n, (K.get('specs') or {}).get('structure') or K.get('structure'), all_pads=True)
        connected = bool(paths) and all(p['length_mm'] is not None for p in paths)
        absent = length + missing <= 1e-9 or not connected
        complete = not absent and missing < 1e-6
        incomplete += not complete
        out[n] = dict(deviation_rel=err / length if length else None, max_rel=worst if length else None,
                      measured_mm=length, unknown_mm=missing, nominal_ohm=target, complete=complete,
                      stretch_missing=absent, joined=connected)
        sumerr += err; total_len += length; unknown += missing
    groups = length_list(B, R, K)
    penalty = sumerr + unknown + 10*incomplete + sum(q['over_mm'] or 0.0 for q in groups.values()) + 10*sum(not q['complete'] for q in groups.values())
    return dict(version='r35-fld-v2', net_list=out, length_groups=groups,
                impedance_rel=sumerr / total_len if total_len else None, unknown_mm=unknown,
                controlled_nets_incomplete=incomplete, controlled_nets_missing=sum(v['stretch_missing'] for v in out.values()),
                length_error_max_mm=max([q['error_mm'] for q in groups.values() if q['error_mm'] is not None], default=None),
                impedance_complete=all(v['complete'] for v in out.values()),
                length_groups_unknown=sum(not q['complete'] for q in groups.values()),
                length_gate=all(q['complete'] and q['over_mm'] <= 1e-6 for q in groups.values()),
                score_extra=penalty)
