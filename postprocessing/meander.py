import json, math, os, re, sys, uuid

A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from common import meander as M
from common.pair_nets import pairs as pairs_find


def _r(v):
    return round(float(v), 6)


def pair_data(H, P, N):
    out = {}
    for s, net in (('P', P), ('N', N)):
        cu_layers = {}; arcs = {}
        for t in H['tracks']:
            if t['net'] != net: continue
            if t.get('arc'): arcs.setdefault((t['uuid'], t['layer']), []).append(t); continue
            cu_layers.setdefault(t['layer'], dict(edges=[], pinned=[]))['edges'].append(dict(type_='seg', a=t['a'], b=t['b'], w=t['w'], uuid=t['uuid']))
        for (u, L), ch in sorted(arcs.items()):
            if len(ch) != 2: continue
            cu_layers.setdefault(L, dict(edges=[], pinned=[]))['edges'].append(dict(type_='arc', a=ch[0]['a'], m=ch[0]['b'], b=ch[1]['b'], w=ch[0]['w'], uuid=u))
        for L, d in cu_layers.items():
            d['pinned'] = sorted({(p['x'], p['y']) for p in H.get('pads', []) if p['net'] == net and p['layer'] == L and 'x' in p}) + \
                        sorted({(v['x'], v['y']) for v in H.get('vias', []) if v['net'] == net and L in (v.get('layers') or [L])})
        out[s] = dict(net_name=net, cu_layers=cu_layers)
    return out


def skew_design_rules(dru_text):
    out = {}
    for blk in re.split(r'\n\s*\(rule\b', dru_text or ''):
        m = re.search(r"inDiffPair\('([^']*)'\)", blk); s = re.search(r'\(constraint\s+skew\s+\(max\s+([0-9.]+)\s*mm', blk)
        if m and s: out[m.group(1)] = float(s.group(1))
    return out


def tolerance(P, N, design_rules, std=0.1):
    import fnmatch
    best = None
    for basis, tol in sorted(design_rules.items()):
        if any(fnmatch.fnmatchcase(n, basis + '*') for n in (P, N)): best = tol if best is None else min(best, tol)
    return std if best is None else best


def _uid(*part_list):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, 'autorouter-meander/' + json.dumps(part_list, sort_keys=True)))


def generator_block(g):
    z = lambda v: ('%.6f' % v).rstrip('0').rstrip('.')
    q = lambda s: '"%s"' % s.replace('\\', '\\\\').replace('"', '\\"')
    L = g['layer'] if g['layer'].endswith('.Cu') else ('F.Cu' if g['layer'] == 'F' else 'B.Cu' if g['layer'] == 'B' else g['layer'] + '.Cu')
    (x0, y0), (x1, y1) = g['from_'], g['until']
    lines = ['(uuid %s)' % q(g['uuid']), '(type tuning_pattern)', '(name "Tuning Pattern")', '(layer %s)' % q(L),
              '(base_line\n\t\t\t(pts\n\t\t\t\t(xy %s %s) (xy %s %s)\n\t\t\t)\n\t\t)' % (z(x0), z(y0), z(x1), z(y1)),
              '(corner_radius_percent %d)' % round(g.get('corner_radius', 100)), '(end\n\t\t\t(xy %s %s)\n\t\t)' % (z(x1), z(y1)),
              '(initial_side %s)' % q(g.get('board_side', 'default')), '(load_diff_pair_gap %s)' % z(g.get('gap_width', 0)),
              '(load_net_name %s)' % q(g['net_name']), '(load_status "tuned")', '(load_track_width %s)' % z(g['w']),
              '(max_amplitude %s)' % z(g.get('amax', 1.0)), '(min_amplitude %s)' % z(g.get('amin', 0.1)), '(min_spacing %s)' % z(g['s']),
              '(origin\n\t\t\t(xy %s %s)\n\t\t)' % (z(x0), z(y0)), '(override_custom_rules no)', '(rounded yes)', '(single_sided yes)',
              '(target_skew 0)', '(target_skew_max %s)' % z(g.get('tol', 0.1)), '(target_skew_min %s)' % z(-g.get('tol', 0.1)),
              '(tuning_mode "diff_pair_skew")', '(members %s)' % ' '.join(q(u) for u in g['members'])]
    return '\n\t(generated\n' + ''.join('\t\t%s\n' % x for x in lines) + '\t)'


def meander_change(H, fwd, pairs, opt=None, keepout_list=()):
    o = dict(opt or {})
    plan_opt = {}
    if o.get('levels'): plan_opt['levels'] = tuple(float(x) for x in str(o['levels']).split(','))
    for k in ('amax', 'rmin', 'lat_max', 'close_max'):
        if k in o: plan_opt[k] = float(o[k])
    skew_min = float(o.get('skew_min', 0.02)); gen = str(o.get('generator', '0')) == '1'
    D = dict(remove=[], add=[], add_generated=[], stat=dict(pairs={}))
    replaced = {}
    for P, N, tol in sorted(pairs):
        if P in keepout_list or N in keepout_list: D['stat']['pairs'][P] = 'blocked'; continue
        pd = pair_data(H, P, N)
        if not pd['P']['cu_layers'] or not pd['N']['cu_layers']: continue
        environment = M.Environment(fwd, {P, N}, replaced)
        R = M.pair_schedule(pd, environment, dict(plan_opt, tolerance=tol))
        info = R['info']

        has_prev = any(x['arcs'] == 0 for x in info['straight_drawn']['P'] + info['straight_drawn']['N'])
        if R.get('info', {}).get('faults') or not R['new'] or (not has_prev and abs(info['skew_before']) <= skew_min):
            info['applied'] = False; D['stat']['pairs'][P] = info; continue
        if abs(info['skew_new']) > abs(info['skew_before']) + 1e-6 and abs(info['skew_new']) > tol:
            info['applied'] = False; info['reason'] = 'Skew would get worse'; D['stat']['pairs'][P] = info; continue
        info['applied'] = True; info['tolerance'] = tol; D['stat']['pairs'][P] = info
        n_before = (len(D['remove']), len(D['add']))
        for net, cu_layers in sorted(R['new'].items()):
            for L, nc in sorted(cu_layers.items()):
                for k in nc:
                    new = []
                    for p in k['prims']:
                        if p[0] == 'seg':
                            if M.dist(p[1], p[2]) < 1e-6: continue
                            it = dict(net=net, layer=L, a=[_r(p[1][0]), _r(p[1][1])], b=[_r(p[2][0]), _r(p[2][1])], w=k['w'])
                        else:
                            it = dict(net=net, layer=L, a=[_r(p[1][0]), _r(p[1][1])], m=[_r(p[2][0]), _r(p[2][1])], b=[_r(p[3][0]), _r(p[3][1])], w=k['w'])
                        it['uuid'] = _uid(it); new.append(it)
                    if {it['uuid'] for it in new} == set(k['uuids']): continue
                    D['remove'] += k['uuids']; D['add'] += new
                    if gen and net == info['short']:
                        span = [st for st in info['sites'] if st['layer_name'] == L]
                        for st in span:
                            if any(x is not st and _in_span(st, x) for x in span): continue
                            g = _generator(st, new, net, L, k['w'], tol, P, N, gap_width=info.get('pair_spacing', {}).get(L, 0.0))
                            if g: D['add_generated'].append(g)
        if (len(D['remove']), len(D['add'])) == n_before: info['applied'] = False; info['reason'] = 'unchanged'
        replaced.update(R['geo'])
    D['remove'] = sorted(set(D['remove']))
    return D


def _in_span(st, x):
    def on(q):
        a, b = x['from_'], x['until']; L = M.dist(a, b)
        if L < 1e-9: return False
        t = ((q[0] - a[0]) * (b[0] - a[0]) + (q[1] - a[1]) * (b[1] - a[1])) / L
        return -1e-6 <= t <= L + 1e-6 and abs((q[0] - a[0]) * (b[1] - a[1]) - (q[1] - a[1]) * (b[0] - a[0])) / L < 1e-5
    return on(st['from_']) and on(st['until']) and (st['from_'], st['until']) != (x['from_'], x['until'])


def _generator(st, new, net, L, w, tol, P, N, gap_width=0.0):
    i0 = [i for i, it in enumerate(new) if M.dist(it['a'], st['from_']) < 1e-5]
    i1 = [i for i, it in enumerate(new) if M.dist(it['b'], st['until']) < 1e-5]
    if not i0 or not i1 or i1[-1] < i0[0]: return None
    members = new[i0[0]:i1[-1] + 1]
    from_, until = st['from_'], st['until']

    ux, uy = until[0] - from_[0], until[1] - from_[1]
    sharp = max((it for it in members if 'm' in it), key=lambda it: abs(ux * (it['m'][1] - from_[1]) - uy * (it['m'][0] - from_[0])), default=None)
    board_side = 'default'
    if sharp:
        board_side = 'right' if ux * (sharp['m'][1] - from_[1]) - uy * (sharp['m'][0] - from_[0]) > 0 else 'left'
    return dict(uuid=_uid('gen', net, from_, until), layer=L, from_=from_, until=until, net_name=net, w=w, s=st['s'], tol=tol, board_side=board_side, gap_width=gap_width,
                members=[it['uuid'] for it in members], corner_radius=round(100 * min(1.0, st['r'] / (st['s'] / 2))))


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from common.copper_net import Obstacles
    fwd = Obstacles(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcvia=float(o.get('drcvia', o.get('drc', 0.09))),
                      hole_size=float(o.get('hole_size', 0.25)), edge=float(o.get('edge', 0.2)))
    design_rules = skew_design_rules(open(o['dru']).read()) if o.get('dru') else {}
    if o.get('pairs'):
        pl = [tuple(x.split(',')) for x in o['pairs'].split(';') if x]
    else:
        pl = pairs_find(sorted({t['net'] for t in H['tracks'] if t['net']}), o.get('netclasses'))
    D = meander_change(H, fwd, [(p, n, tolerance(p, n, design_rules)) for p, n in pl], o)
    json.dump(D, open(a[1], 'w'), indent=1, ensure_ascii=False)
    for p, inf in sorted(D['stat']['pairs'].items()):
        if isinstance(inf, dict):
            print(p, 'E %.4f' % inf['E'], 'short', inf['short'], 'Skew', inf['skew_before'], '->', inf['skew_new'], 'Sites', len(inf['sites']),
                  'applied' if inf.get('applied') else 'not applied', inf.get('reason', ''))
    print('removed %d, new %d (arcs %d), generators %d' % (len(D['remove']), len(D['add']), sum(1 for x in D['add'] if 'm' in x), len(D['add_generated'])))


if __name__ == '__main__':
    main()
