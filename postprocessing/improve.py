import json, sys, os, math, subprocess, collections, time, uuid
from concurrent.futures import ThreadPoolExecutor

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from common.process_guard import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union
from common.shift import relocate
from common.parts import load_parts, circuit_board, Occupancy
from analysis.evaluation import rate, budgets
from common import slots as PL
from postprocessing.move_rules import keepout, assumption, rule_values, movement, route_move, octi
from postprocessing import attempt_log as VL

VPY = os.path.join(R, '.venv', 'bin', 'python')
D = os.path.abspath(sys.argv[1])
args = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
os.environ['AUTOROUTER_SLOTS'] = '4'
ROUNDS = int(args.get('rounds_n', '3')); PAR = min(4, max(1, int(args.get('parallel', '4')))); NET_COPPER = int(args.get('cand', '12'))
ITER = args.get('iter', '15'); MAXS = float(args.get('max_step', '0.35')); MIND = float(args.get('minimum', '2.0'))
MOVES = set(args.get('move_records', 'shift,rotate,swap').split(',')); SWAP_FROM = float(args.get('swap_spacing', '6'))
SUPPLY_N = int(args.get('offset_n', '2')); SUPPLY_PL = int(args.get('offset_slots', '3'))
BUDGET_CPU = float(args.get('budget_cpu', '0'))
def cpu_consumed():
    import resource
    a, b = resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)
    return a.ru_utime + a.ru_stime + b.ru_utime + b.ru_stime
def budget_off(wo):
    if BUDGET_CPU and cpu_consumed() > BUDGET_CPU:
        log('   CPU budget %.0f s reached (%.0f s), %s' % (BUDGET_CPU, cpu_consumed(), wo)); return True
    return False
RANKER = args.get('ranker'); RANKER_K = int(args.get('ranker_k', '4')); RANKER_POOL = args.get('ranker_pool', 'choice')
IR = args.get('ir', '1') == '1'; IR_TOL = float(args.get('ir_protection', '0.03'))

ZONE_OPT = None if args.get('ir_zones', '1') == '0' else {
    k: os.environ[k] for k in ('ZONE_EDGE', 'ZONE_GAP', 'ZONE_FAIR', 'ZONE_PRECEDENCE',
                              'ZONE_GND_SIGNAL', 'GND_ALL', 'FILL_NET', 'TONGUE', 'ZCLR') if k in os.environ}
SCORE = args.get('rating', 'v3')
if SCORE not in ('v1', 'v2', 'v3', 'minimax'): raise ValueError('rating must be v1, v2, v3 or minimax')
if SCORE == 'v2' and not IR: SCORE = 'v1'
VL.SCORE_KEY = {'v1': 'rating', 'v2': 'score_v2', 'v3': 'score_v3', 'minimax': 'minimax'}[SCORE]
def nw(E):
    return E[VL.SCORE_KEY] if E.get(VL.SCORE_KEY) is not None else float('nan')
def evaluation(B, Rt):
    E = rate(B, Rt, K, ir=IR, zone_list=ZONE_OPT, v3=True, minimax=True if SCORE == 'minimax' else None)
    E['move_design_rules'] = rule_values(B, Rt, K)
    return E
K = json.load(open(os.path.join(D, 'netclasses.json')))
if SCORE == 'v3' and (K.get('specs') or {}).get('evaluation') == 'minimax':
    SCORE = 'minimax'; VL.SCORE_KEY = 'minimax'
NET = K['net_list']; PARTS = K['part_list']; GND = set(K['gnd'])
B0 = json.load(open(os.path.join(D, 'board.json')))
WW = args.get('travel_range', '1') == '1'
if WW:
    from common import travel_range as WWM
    WT = WWM.table(K); POS0 = {f['ref']: (f['x'], f['y']) for f in B0['fps']}
    for _r,_p in (B0.get('wander_basis') or {}).items(): POS0[_r] = (_p['x'],_p['y']) if isinstance(_p,dict) else tuple(_p[:2])
def way(fps, r, dx=0.0, dy=0.0):
    return math.hypot(fps[r]['x'] + dx - POS0[r][0], fps[r]['y'] + dy - POS0[r][1])
def in_frame(fps, refs):
    for r, v in refs.items():
        if abs(v[2] % 360) > 1e-6 and (PARTS.get(r, {}).get('function') == 'coil_sw' or (WW and not WT[r]['rotate'])):
            return False
    return not WW or all(r in WT and WWM.in_frame(WT[r], way(fps, r, v[0], v[1])) for r, v in refs.items())
def price_move(B, refs):
    if not WW: return 0.0
    fps = {f['ref']: f for f in B['fps']}
    return sum(WWM.price(WT[r], way(fps, r), way(fps, r, v[0], v[1])) for r, v in refs.items())
R0 = json.load(open(os.path.join(D, 'routes.json')))
opt = dict(R0.get('options') or {})
if 'opt' in args: opt.update(dict(x.split('=', 1) for x in args['opt'].split() if '=' in x))
for k in ('start', 'new', 'new_nets', 'all_items', 'png', 'pre', 'iter', 'short', 'relayed', 'fix', 'only', 'netclasses', 'design_rules'): opt.pop(k, None)
opt.update(design_rules=os.path.join(D, 'design_rules.json'), netclasses=os.path.join(D, 'netclasses.json'))
WORK = os.path.join(D, 'improve'); os.makedirs(WORK, exist_ok=True)
LOG = open(os.path.join(D, 'improve.log'), 'a')
def log(*a):
    s = ' '.join(str(x) for x in a); print(s, flush=True); LOG.write(s + '\n'); LOG.flush()
FAMILY = args.get('family') or (K.get('specs') or {}).get('family') or 'unknown'
JL = VL.AttemptLog(os.path.join(D, 'improve_attempts.jsonl'), dict(
    run_id=str(uuid.uuid4()), board_id=VL.sha_file(os.path.join(D, 'board.json')), family=FAMILY, code=VL.code_hashes(),
    environment=VL.environment(), start_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())))
MODEL = None; POLICY = {}
if RANKER:
    from learning import moves as ZG
    MODEL = ZG.load_data(RANKER); POLICY = dict(policy='ranker-v1-' + RANKER_POOL, model_id=MODEL.get('id'))
def ww_from(k):
    return {r: WT[r] for r in k['refs'] if r in WT} if WW else None
NC_SHA = VL.sha_file(os.path.join(D, 'netclasses.json')); RG_SHA = VL.sha_file(os.path.join(D, 'design_rules.json'))
def cur_state(B, Rt): return dict(board=VL.sha_obj(B), routes=VL.sha_obj(Rt), netclasses=NC_SHA, design_rules=RG_SHA)


def route(board, start, out, new=None, iter_=ITER, net_list=None):
    cmd = ['caffeinate', '-i', '-s', VPY, os.path.join(R, 'router', 'route_pf.py'), board, out, 'start=' + start, 'iter=' + str(iter_)]
    if new: cmd.append('new=' + ','.join(new))
    if net_list: cmd.append('new_nets=' + ','.join(net_list))
    from postprocessing.short_circuit_protection import protection_on, protection_option
    cmd += ['%s=%s' % kv for kv in opt.items()] + protection_option(protection_on(opt))

    rr = PL.job(cmd, log=out + '.route.log', env=dict(os.environ, PYTHONHASHSEED='0', AUTOROUTER_SLOTS='4'))
    json.dump(dict(rr, command=cmd), open(out + '.route.meta.json', 'w'))
    return rr['rc'] == 0 and os.path.exists(out)


def free_after(Bk, refs):
    refs = [refs] if isinstance(refs, str) else list(refs)
    T2 = load_parts(Bk, 0.15); _, _, ip, keep = circuit_board(Bk, 0.2); f2 = {f['ref']: f for f in Bk['fps']}
    bel = Occupancy()
    for q in T2:
        if q not in refs:
            gq = T2[q].geo(f2[q]['x'], f2[q]['y'], 0.0, f2[q]['side'])

            owned = dict(keepouts=[k for k in Bk.get('keepouts', []) if k.get('ref') in refs])
            if keepout(owned, gq, (q,)): return False
            bel.add(q, gq)
    for r in refs:
        if r not in T2 or f2[r].get('locked') or PARTS.get(r, {}).get('pinned') or PARTS.get(r, {}).get('role') in ('connector', 'mechanics'): return False
        g = T2[r].geo(f2[r]['x'], f2[r]['y'], 0.0, f2[r]['side'])
        if bel.collision(g) or not all(ip.contains(x) for x in g.values()) or (keep is not None and keepout(Bk, g, (r,))): return False
        bel.add(r, g)
    return True


TABU = set()
ROTATE_TABU = set()


def ranking(B, Rt, E, ks):
    for k in ks: k['before'] = VL.before(B, K, k, E, Rt=Rt, ww=ww_from(k))
    value_list = ZG.prediction(MODEL, [k['before'] for k in ks])
    for k, v in zip(ks, value_list): k['rank_value'] = round(float(v), 6)
    return sorted(ks, key=lambda k: -k['rank_value'])[:RANKER_K]


def new_moves(B, Rt, fps, T, movable, step, out, ev):
    if 'island' in MOVES:
        for name, island in sorted((K.get('islands') or {}).items()):
            refs = sorted(r for r, t in PARTS.items() if r == name or t.get('island') == name)
            if not refs or not all(r in T and movable(r) for r in refs): continue
            d = min(step(r) for r in refs)
            if d <= .02: continue
            for i, (dx, dy) in enumerate(((d, 0), (-d, 0), (0, d), (0, -d))):
                m = {r: (dx, dy, 0.0) for r in refs}
                if in_frame(fps, m):
                    out['island:%s:%d' % (name, i)] = dict(refs=m, category='island', source='bottleneck', reason='Island ' + name,
                                                        net_name=None, gain=1.0)
    if 'side_switch' in MOVES:
        for r in sorted(T):
            if not movable(r) or PARTS[r].get('role') != 'small' or len(T[r].pads) > 4: continue
            if any(p.get('drill', 0) or len(p['layers']) > 1 for p in B['pads'] if p['ref'] == r): continue
            if PARTS[r].get('function') in ('coil_sw', 'crystal', 'crystal_load'): continue

            core_ref = PARTS[r].get('island'); core_set = fps.get(core_ref)
            if not core_set or core_ref not in T: continue
            from shapely.geometry import Point
            footprint = T[core_ref].geo(core_set['x'], core_set['y'], 0.0, core_set['side'])[core_set['side']]
            if not footprint.buffer(.2).contains(Point(fps[r]['x'], fps[r]['y'])): continue
            out['board_side:' + r] = dict(refs={r: (0.0, 0.0, 0.0)}, sides={r: 'B' if fps[r]['side'] == 'F' else 'F'},
                                     category='side_switch', source='bottleneck', reason='Opposite side under island core', net_name=None, gain=1.0)
    def guarded(n):
        return bool(NET.get(n, {}).get('pair'))
    if 'bend' in MOVES:
        for ti, t in enumerate(Rt['tracks']):
            if guarded(t['net']) or t.get('kelvin') or t.get('current_web'): continue
            for i in range(1, len(t['pts']) - 1):
                a, q, b = t['pts'][i - 1:i + 2]
                z = [round(a[0] + b[0] - q[0], 5), round(a[1] + b[1] - q[1], 5)]
                if not .02 < math.dist(z, q) <= 2 * MAXS: continue
                if not octi(a, z) or not octi(z, b): continue

                if any(p['net'] == t['net'] and math.dist(q, (p['x'], p['y'])) < .4 for p in B['pads']): continue
                if any(v['net'] == t['net'] and math.dist(q, (v['x'], v['y'])) < .4 for v in Rt['vias']): continue
                if any(j != ti and u['net'] == t['net'] and u['layer'] == t['layer'] and LineString(u['pts']).distance(LineString([q, q])) < .05
                       for j, u in enumerate(Rt['tracks'])): continue
                pts = [list(p) for p in t['pts']]; pts[i] = z
                out['bend:%d:%d' % (ti, i)] = dict(refs={}, net_name=None, category='bend', source='track', reason='Bend %s/%s' % (t['net'], t['layer']),
                    gain=.5, route_moves=[dict(fld='tracks', index=ti, prior=dict(pts=t['pts']), new=dict(pts=pts))], goal_net=t['net'])


            for i in range(1, len(t['pts']) - 2):
                a, q, r, b = t['pts'][i - 1:i + 3]
                def cross(u, v): return u[0] * v[1] - u[1] * v[0]
                u = (q[0] - a[0], q[1] - a[1]); v = (b[0] - r[0], b[1] - r[1]); h = (r[0] - q[0], r[1] - q[1])
                lh = math.hypot(*h)
                if lh < .1 or abs(cross(u, h)) < 1e-6 or abs(cross(v, h)) < 1e-6: continue
                from shapely.geometry import Point
                if any(p['net'] == t['net'] and min(math.dist(q, (p['x'], p['y'])), math.dist(r, (p['x'], p['y']))) < .4 for p in B['pads']): continue
                if any(w['net'] == t['net'] and min(math.dist(q, (w['x'], w['y'])), math.dist(r, (w['x'], w['y']))) < .4 for w in Rt['vias']): continue
                if any(j != ti and w['net'] == t['net'] and w['layer'] == t['layer'] and
                       min(LineString(w['pts']).distance(Point(q)), LineString(w['pts']).distance(Point(r))) < .05 for j, w in enumerate(Rt['tracks'])): continue
                for sg in (-1, 1):
                    d = min(MAXS / 2, .15); offset = (-h[1] / lh * d * sg, h[0] / lh * d * sg)
                    tq, tr = cross(offset, h) / cross(u, h), cross(offset, h) / cross(v, h)
                    q2 = [round(q[0] + tq * u[0], 5), round(q[1] + tq * u[1], 5)]
                    r2 = [round(r[0] + tr * v[0], 5), round(r[1] + tr * v[1], 5)]
                    if max(math.dist(q, q2), math.dist(r, r2)) > MAXS + 1e-5: continue
                    if not all(octi(x, y) for x, y in ((a, q2), (q2, r2), (r2, b))): continue
                    pts = [list(p) for p in t['pts']]; pts[i:i + 2] = [q2, r2]
                    out['channel:%d:%d:%d' % (ti, i, sg)] = dict(refs={}, net_name=None, category='bend', source='track',
                        reason='Shift bend pair %s/%s' % (t['net'], t['layer']), gain=.5,
                        route_moves=[dict(fld='tracks', index=ti, prior=dict(pts=t['pts']), new=dict(pts=pts))], goal_net=t['net'])
    if 'via' in MOVES:
        for vi, v in enumerate(Rt['vias']):
            if v['net'] in GND or guarded(v['net']) or v.get('current_via'): continue
            q = [v['x'], v['y']]
            if any(p['net'] == v['net'] and math.dist(q, (p['x'], p['y'])) < .5 for p in B['pads']): continue
            ends = [(ti, i) for ti, t in enumerate(Rt['tracks']) if t['net'] == v['net']
                    for i in (0, len(t['pts']) - 1) if math.dist(t['pts'][i], q) < 1e-5]
            if len(ends) < 2: continue

            if any(t['net'] == v['net'] and any(math.dist(p, q) < 1e-5 for p in t['pts'][1:-1]) for t in Rt['tracks']): continue
            d = min(.2, MAXS)
            for j, (dx, dy) in enumerate(((d, 0), (-d, 0), (0, d), (0, -d), (d, d), (-d, -d), (d, -d), (-d, d))):
                z = [round(q[0] + dx, 5), round(q[1] + dy, 5)]; ms = []
                for ti, i in ends:
                    t = Rt['tracks'][ti]; pts = [list(p) for p in t['pts']]
                    neighbor = pts[1 if i == 0 else -2]
                    if not octi(z, neighbor): break
                    pts[i] = z; ms.append(dict(fld='tracks', index=ti, prior=dict(pts=t['pts']), new=dict(pts=pts)))
                else:
                    ms.append(dict(fld='vias', index=vi, prior=dict(x=v['x'], y=v['y']), new=dict(x=z[0], y=z[1])))
                    out['via:%d:%d' % (vi, j)] = dict(refs={}, net_name=None, category='via', source='via', reason='Via ' + v['net'],
                                                     gain=.5, route_moves=ms, goal_net=v['net'])


def candidates(B, Rt, E=None):
    fps = {f['ref']: f for f in B['fps']}
    out_port_E = E
    out = {}
    def movable(r):
        t = PARTS.get(r, {})
        if fps.get(r, {}).get('locked'): return False
        if WW: return t.get('role') in ('small', 'chip') and not t.get('pinned') and r in WT and WWM.movable(WT[r])
        return t.get('role') == 'small' and not t.get('pinned') and not t.get('decoupling')
    def step(r):
        return min(MAXS, WT[r]['max'] - way(fps, r)) if WW else MAXS

    if 'shift' in MOVES:
        tmpb = os.path.join(WORK, 'k_board.json'); tmpr = os.path.join(WORK, 'k_routes.json')
        json.dump(B, open(tmpb, 'w')); json.dump(Rt, open(tmpr, 'w'))
        kf = os.path.join(WORK, 'k_routes_conflicts.json')
        if not os.path.exists(kf): json.dump([], open(kf, 'w'))
        subprocess.run([VPY, os.path.join(R, 'analysis', 'bottlenecks.py'), tmpb, tmpr, os.path.join(D, 'netclasses.json'),
                        'out=' + os.path.join(WORK, 'k_tight.json'), 'max_step=%g' % MAXS], capture_output=True, text=True)
        E = json.load(open(os.path.join(WORK, 'k_tight.json')))
        for r, v in E['shifts'].items():
            if not movable(r) or not in_frame(fps, {r: (v['dx'], v['dy'], 0.0)}): continue
            out[r] = dict(refs={r: (v['dx'], v['dy'], 0.0)}, category='shift', source='bottleneck', reason='Bottleneck: ' + v['reasons'][0], gain=1.0, net_name=None)

    T = load_parts(B, 0.15); board, inner, inner_p, keep = circuit_board(B, 0.2)
    bel = Occupancy()
    for r in T: bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
    def free_here(r, st):
        geo = T[r].geo(*st); bel.remove(r)
        ok = not bel.collision(geo) and all(inner_p.contains(x) for x in geo.values()) and not (keep is not None and keepout(B, geo, (r,)))
        bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
        return ok
    pads_net = collections.defaultdict(list)
    for p in B['pads']: pads_net[p['net']].append(p)
    pad_poly = [(p['ref'], p['net'], unary_union([Polygon(q) for qs in p['polys'].values() for q in qs if len(q) >= 3])) for p in B['pads']]
    ev = out_port_E or evaluation(B, Rt)
    detour_nets = {}
    for gain, n, L, m in ev['detours'][:8]:
        detour_nets[n] = gain
        P = pads_net[n]
        if len(P) < 2: continue

        pts = [(p['x'], p['y']) for p in P]; inT = [0]; rest = list(range(1, len(pts))); edges = []
        while rest:
            i, j = min(((i, j) for i in inT for j in rest), key=lambda ij: math.dist(pts[ij[0]], pts[ij[1]]))
            edges.append((pts[i], pts[j])); inT.append(j); rest.remove(j)
        w = NET.get(n, {}).get('w', 0.15)
        if 'shift' in MOVES:
            for a, b in edges:
                seg = LineString([a, b]);
                if seg.length < 1e-6: continue
                corr = seg.buffer(w / 2 + 0.12)
                for r, pn, g in pad_poly:
                    if pn == n or not movable(r) or r in out: continue
                    if not corr.intersects(g): continue
                    c = g.centroid; q = seg.interpolate(seg.project(c))
                    dd = math.hypot(c.x - q.x, c.y - q.y)
                    ux, uy = ((c.x - q.x) / dd, (c.y - q.y) / dd) if dd > 1e-3 else (-(b[1] - a[1]) / seg.length, (b[0] - a[0]) / seg.length)
                    need = min(step(r), w / 2 + 0.12 + g.minimum_rotated_rectangle.length / 4 - dd + 0.05)
                    if need <= 0.02: continue
                    dx, dy = ux * need, uy * need
                    if not in_frame(fps, {r: (dx, dy, 0.0)}): continue
                    if free_here(r, (fps[r]['x'] + dx, fps[r]['y'] + dy, 0.0, fps[r]['side'])):
                        out[r] = dict(refs={r: (round(dx, 3), round(dy, 3), 0.0)}, category='shift', source='detour',
                                      reason='Detour %s (+%.1f mm)' % (n.split('/')[-1], gain), gain=gain, net_name=n)

        if 'rotate' in MOVES:
            for r in sorted({pr for pr, pn, g in pad_poly if pn == n}):
                if not movable(r) or len(T[r].pads) > 3: continue
                if (WW and not WT[r]['rotate']) or PARTS[r].get('function') == 'coil_sw': continue
                for th, factor in ((180.0, 0.8), (90.0, 0.6), (270.0, 0.6)):
                    key = '%s@%g' % (r, th)
                    if key in out or (r, th) in ROTATE_TABU: continue
                    if free_here(r, (fps[r]['x'], fps[r]['y'], th, fps[r]['side'])):
                        out[key] = dict(refs={r: (0.0, 0.0, th)}, category='rotate', source='detour',
                                        reason='Rotate %g because of detour %s' % (th, n.split('/')[-1]), gain=gain * factor, net_name=n)

    if 'swap' in MOVES and detour_nets:
        nets_from = collections.defaultdict(set)
        for p in B['pads']: nets_from[p['ref']].add(p['net'])
        group_list = collections.defaultdict(list)
        for r, f in fps.items():
            if r in T and movable(r): group_list[(f.get('fp'), f.get('value'), f['side'])].append(r)
        for g_refs in group_list.values():
            for i, r1 in enumerate(g_refs):
                for r2 in g_refs[i + 1:]:
                    f1, f2 = fps[r1], fps[r2]
                    if math.hypot(f1['x'] - f2['x'], f1['y'] - f2['y']) > SWAP_FROM: continue
                    if nets_from[r1] == nets_from[r2]: continue
                    if frozenset((r1, r2)) in TABU: continue
                    gain = max([detour_nets.get(n, 0.0) for n in nets_from[r1] | nets_from[r2]])
                    if gain <= 0: continue
                    move = None
                    for sg in (1, -1):
                        th1 = (sg * (f2.get('rot', 0) - f1.get('rot', 0))) % 360
                        m = {r1: (f2['x'] - f1['x'], f2['y'] - f1['y'], th1), r2: (f1['x'] - f2['x'], f1['y'] - f2['y'], (-th1) % 360)}
                        Bk, _ = relocate(B, m)
                        p1 = sorted((round(p['x'], 3), round(p['y'], 3)) for p in Bk['pads'] if p['ref'] == r1)
                        p2 = sorted((round(p['x'], 3), round(p['y'], 3)) for p in B['pads'] if p['ref'] == r2)
                        if p1 == p2: move = m; break
                    if move is None or not in_frame(fps, move): continue
                    key = '%s<>%s' % (r1, r2)
                    n_um = max(nets_from[r1] | nets_from[r2], key=lambda n: detour_nets.get(n, 0.0))
                    out[key] = dict(refs=move, category='swap', source='detour', reason='Swap %s<>%s because of detour %s' % (r1, r2, n_um.split('/')[-1]),
                                    gain=gain * 0.9, net_name=n_um)


    if 'offset' in MOVES and WW:
        from common.offset import targets
        for r in sorted(T):
            if not movable(r) or WT[r]['max'] != WWM.WHOLE or len(T[r].pads) != 2: continue
            for i, z in enumerate(targets(B, Rt, K, r, n=SUPPLY_N, T=T, bel=bel, plat=(board, inner, inner_p, keep))):
                if not in_frame(fps, z['refs']): continue
                out['%s~%d' % (r, i)] = dict(refs=z['refs'], category='offset', source='offset', net_name=None,
                                             reason='Move at %s next to %s (spacing %.2f mm, %s)' % (
                                                 z['signal'].split('/')[-1], z['other'].split('/')[-1], z['d'], z['layer_name']),
                                             gain=1.0 / (0.2 + z['d']))
    new_moves(B, Rt, fps, T, movable, step, out, ev)

    out = {key: k for key, k in out.items() if not k['refs'] or free_after(movement(B, k), k['refs'])}

    if WW:
        for k in out.values(): k['price'] = price_move(B, k['refs'])
    for k in out.values(): k['effort'] = 1.0 + max(0.0, k.get('price', 0.0))
    all_items = sorted(out.values(), key=lambda k: -k['gain'] / k['effort'])
    if RANKER and RANKER_POOL == 'all_items': return ranking(B, Rt, out_port_E or ev, all_items)
    choice = []
    kinds = ('shift', 'rotate', 'swap', 'offset', 'island', 'side_switch', 'bend', 'via')
    group_list = {category: [k for k in all_items if k['category'] == category][:SUPPLY_PL if category == 'offset' else 2] for category in kinds}
    if MOVES & {'island', 'side_switch', 'bend', 'via'} and sum(map(len, group_list.values())) > NET_COPPER:

        for i in range(max(map(len, group_list.values()), default=0)):
            for category in kinds:
                if i < len(group_list[category]) and len(choice) < NET_COPPER: choice.append(group_list[category][i])
    else:
        for category in kinds: choice += group_list[category]
    for k in all_items:
        if len(choice) >= NET_COPPER: break
        if k not in choice: choice.append(k)
    choice = sorted(choice[:NET_COPPER], key=lambda k: -k['gain'] / k['effort'])
    if RANKER: return ranking(B, Rt, out_port_E or ev, choice)
    return choice


def move_text(k):
    return ', '.join('%s (%+.2f, %+.2f%s)' % (r, v[0], v[1], ', %g deg' % v[2] if v[2] else '') for r, v in k['refs'].items()) or k['category'] + ': ' + k.get('reason', '')


BUDGET, BUDGET_GR = budgets(K)
IR_REF = {}


def ir_ok(E0, E1):
    if IR and E0.get('is_valid') and not IR_REF and (E0.get('ir') or {}).get('status') == 'solved':
        IR_REF.update(E0['ir']['max_drop_mV'])
    if not IR or not E0.get('is_valid') or not IR_REF: return True
    I = E1.get('ir') or {}
    if I.get('status') != 'solved' or I.get('faults') or not I.get('max_drop_mV'): return False
    d = I['max_drop_mV']

    if not all(d.get(n, float('inf')) <= max(BUDGET.get(n, 0.0), ref * (1 + IR_TOL) + 0.05) for n, ref in IR_REF.items()): return False
    for nn, b in BUDGET_GR.values():
        ref = sum(IR_REF.get(n, 0.0) for n in nn)
        if sum(d.get(n, float('inf')) for n in nn) > max(b, ref * (1 + IR_TOL) + 0.05): return False
    return True


def decision(B, Rt, E, k, rr):
    if rr['E'] is None: return None
    Bk, Rk = rr['st']
    c = assumption(E, rr['E'], B, Rt, Bk, Rk, K, minimum=MIND, price=k.get('price', 0.0), rating=SCORE)
    if not ir_ok(E, rr['E']): c.update(better=False, reason='IR budget or evidence missing')
    return c


def ropt(k):
    o = dict(opt, iter=ITER, new=','.join(k['refs']))
    if k.get('net_name'): o['new_nets'] = k['net_name']
    return o


def attempts(B, Rt, k, d):
    t1 = time.perf_counter(); os.makedirs(d, exist_ok=True); res = dict(status=None, faults=None, E=None, st=None, route_s=None, kb=None, rs=None, wall_s=None)
    try:
        Bk = movement(B, k);
        if k['category'] != 'null' and k['refs'] and (not free_after(Bk, k['refs']) or not in_frame({f['ref']: f for f in B['fps']}, k['refs'])):
            res.update(status='precheck_rejected', faults='Collision, keepout, travel range or edge'); return res
        bj = os.path.join(d, 'board.json'); json.dump(Bk, open(bj, 'w')); res['kb'] = VL.sha_obj(Bk)
        rj = os.path.join(d, 'routes.json')
        if os.path.exists(rj): os.remove(rj)
        sj = os.path.join(d, 'start.json'); json.dump(route_move(Rt, k), open(sj, 'w'))
        t2 = time.perf_counter()
        ok = route(bj, sj, rj, list(k['refs']), net_list=[k['net_name']] if k.get('net_name') else None)
        res['route_s'] = time.perf_counter() - t2
        if not ok: res.update(status='router_error', faults='Router without output or return code != 0')
        else:
            Rk = json.load(open(rj)); res['rs'] = VL.sha_obj(Rk)
            try: res.update(status='evaluated', E=evaluation(Bk, Rk), st=(Bk, Rk))
            except Exception as e: res.update(status='evaluation_error', faults=repr(e)[:300])
    except Exception as e:
        res.update(status='router_error', faults=repr(e)[:300])
    res['wall_s'] = time.perf_counter() - t1
    return res

def main():
    t0 = time.time()
    moves = collections.defaultdict(lambda: [0.0, 0.0, 0.0])
    B = B0; start = os.path.join(WORK, 'basis_routes.json')
    log('== Improve', D, time.strftime('%H:%M'), 'Moves', sorted(MOVES), 'Family', FAMILY, 'Options', opt, *(('TravelRange',) if WW else ()),
        *(('Ranker', MODEL.get('id'), 'k=%d' % RANKER_K, RANKER_POOL) if RANKER else ()))
    if not route(os.path.join(D, 'board.json'), os.path.join(D, 'routes.json'), start, None, 1):
        log('Base run failed'); return
    Rt = json.load(open(start)); E = evaluation(B, Rt)
    E_input = evaluation(B0, R0)
    if not assumption(E_input, E, B0, R0, B, Rt, K, minimum=-1e-6, rating=SCORE)['better']:
        Rt, E = R0, E_input
    json.dump(E, open(os.path.join(WORK, 'basis_evaluation.json'), 'w'))
    log('Base (%s): score %.1f, vias %d, detours %.1f mm, deficit %.1f, %s%s' % (SCORE, nw(E), E['vias'], E['detour_total_sum'], E['deficit'], 'is_valid' if E['is_valid'] else 'INVALID',
        ', losses %.1f mW' % E['ir']['loss_mW'] if IR and E['ir'] and E['ir'].get('loss_mW') is not None else ''))
    last = None
    for rnd in range(1, ROUNDS + 1):
        if budget_off('no round %d any more' % rnd): break
        ks = candidates(B, Rt, E) if RANKER else candidates(B, Rt)
        log('Round %d: %d candidates (%s)' % (rnd, len(ks), dict(collections.Counter(k['category'] for k in ks))))

        checks = {}
        for k in [dict(refs={}, net_name=None)] + ks:
            sig = (tuple(sorted(k['refs'])), k.get('net_name'))
            if sig not in checks:
                checks[sig] = dict(refs={r: (0.0, 0.0, 0.0) for r in sig[0]}, net_name=sig[1], category='null',
                                      source='inspection', reason='Null move with same reroute scope', gain=0.0, price=0.0)
        controls = list(checks.values())
        for k in controls: k['before'] = VL.before(B, K, k, E, Rt=Rt, ww=ww_from(k))
        with ThreadPoolExecutor(PAR) as ex:
            cr = list(ex.map(lambda ik: attempts(B, Rt, ik[1], os.path.join(WORK, 'r%d_null%d' % (rnd, ik[0]))), enumerate(controls)))
        for k, rr in zip(controls, cr):
            k['result'] = rr
        st0 = cur_state(B, Rt); dec = 'r%d-primary' % rnd
        for k in ks: k['before'] = k.get('before') or VL.before(B, K, k, E, Rt=Rt, ww=ww_from(k))
        with ThreadPoolExecutor(PAR) as ex:
            res = list(ex.map(lambda ik: attempts(B, Rt, ik[1], os.path.join(WORK, 'r%d_k%d' % (rnd, ik[0]))), enumerate(ks)))
        good = []
        for i, k in enumerate(controls):
            rr = k['result']; c = decision(B, Rt, E, k, rr)
            rr['comparison'] = c
            if c and c['better']: good.append(((c.get('delta') or 0), k, rr['E'], rr['st']))
            k['attempt_id'] = str(uuid.uuid4())
            log('   Null check', i, rr['status'], 'Score', nw(rr['E']) if rr['E'] else None)
        for k, rr in zip(ks, res):
            sig = (tuple(sorted(k['refs'])), k.get('net_name')); ck = checks[sig]; ctrl = ck['result']
            rr['control_id'] = ck['attempt_id']
            Ek = rr['E']
            if Ek is None: log('   %-34s %s' % (move_text(k), rr['status'])); continue
            c = decision(B, Rt, E, k, rr)
            rr['comparison'] = c
            nc = decision(ctrl['st'][0], ctrl['st'][1], ctrl['E'], k, rr) if ctrl['E'] else None
            rr['null_comparison'] = nc
            d0 = c['delta'] if c.get('delta') is not None else 0.0
            d = d0 + k.get('price', 0.0)
            ok = c['better'] and nc is not None and nc['better']
            dv = ((Ek.get('ir') or {}).get('loss_mW') or 0) - ((E.get('ir') or {}).get('loss_mW') or 0) if IR else 0
            log('   %-34s score %+.1f  vias %+d  detours %+.1f%s  %s  %s' % (
                move_text(k), d0, Ek['vias'] - E['vias'], Ek['detour_total_sum'] - E['detour_total_sum'], '  loss %+.1f mW' % dv if IR else '',
                'better' if ok else ('-' if ir_ok(E, Ek) else '- (IR)'), k['reason'] + ('  price %.1f' % k['price'] if k.get('price') else '')))
            if ok: good.append((d, k, Ek, rr['st']))
        from analysis import minimax as MM
        good.sort(key=lambda g: (tuple(g[2]['rank'][:3]), MM.rank(g[2]) if g[2].get('evaluation_target') == 'minimax' or SCORE == 'minimax' else g[0]))
        best = good[0][1] if good else None
        for i, k in enumerate(controls):
            rr = k['result']
            JL.attempt(round_no=rnd, index=i, decision_id=dec, phase='control', parent=last, original=None,
                k=k, before=k['before'], state=st0, after_E=rr['E'], status=rr['status'], faults=rr['faults'], wall_s=rr['wall_s'],
                route_s=rr['route_s'], router_opt=ropt(k), accepted=k is best, minimum=MIND, candidate_board_sha=rr['kb'],
                result_sha=rr['rs'], comparison=rr.get('comparison'), attempt_id=k['attempt_id'], **POLICY)
        for i, (k, rr) in enumerate(zip(ks, res)):
            k['attempt_id'] = JL.attempt(round_no=rnd, index=i, decision_id=dec, phase='primary', parent=last, original=None, k=k,
                                         before=k['before'], state=st0, after_E=rr['E'], status=rr['status'], faults=rr['faults'],
                                         wall_s=rr['wall_s'], route_s=rr['route_s'], router_opt=ropt(k), accepted=k is best,
                                         minimum=MIND, candidate_board_sha=rr['kb'], result_sha=rr['rs'],
                                         comparison=rr.get('comparison'), control_id=rr.get('control_id'), control_comparison=rr.get('null_comparison'), **POLICY)
        JL.round_finished()
        if not good: log('   no improvement, finished'); break
        d, k, Ek, (B, Rt) = good[0]; E = Ek; last = k['attempt_id']
        if k['category'] != 'null':
            for r, v in k['refs'].items(): moves[r][0] += v[0]; moves[r][1] += v[1]; moves[r][2] += v[2]
        if k['category'] == 'swap': TABU.add(frozenset(k['refs']))
        if k['category'] == 'rotate':
            for r_, v_ in k['refs'].items(): ROTATE_TABU.add((r_, (360.0 - v_[2]) % 360))
        log('   accepted %s: score %.1f' % (move_text(k), nw(E)))
        used = set(k['refs'])
        for j, (d2, k2, _, _) in enumerate(good[1:]):
            if k2['category'] == 'null' or used & set(k2['refs']): continue
            if k2.get('route_moves') or k2.get('sides'): continue
            if budget_off('no further follow-up check'): break
            before = VL.before(B, K, k2, E, Rt=Rt, ww=ww_from(k2)); st = cur_state(B, Rt); dec2 = 'r%d-recheck-%d' % (rnd, j)
            Bk = movement(B, k2)
            if not free_after(Bk, list(k2['refs'])) or not in_frame({f['ref']: f for f in B['fps']}, k2['refs']):
                JL.attempt(round_no=rnd, index=j, decision_id=dec2, phase='recheck', parent=last, original=k2['attempt_id'], k=k2,
                           before=before, state=st, after_E=None, status='precheck_rejected', faults='Collision or outside after previous acceptance',
                           wall_s=None, route_s=None, router_opt=ropt(k2), accepted=False, minimum=MIND, candidate_board_sha=VL.sha_obj(Bk), result_sha=None, **POLICY)
                continue
            rr = attempts(B, Rt, k2, os.path.join(WORK, 'r%d_plus_%s' % (rnd, '_'.join(k2['refs']))))
            Ek2 = rr['E']
            p2 = price_move(B, k2['refs']) if WW else 0.0
            c = decision(B, Rt, E, dict(k2, price=p2), rr)

            null = dict(refs={r: (0.0, 0.0, 0.0) for r in k2['refs']}, net_name=k2.get('net_name'), category='null',
                        source='inspection', reason='Null move for follow-up check', gain=0.0, price=0.0)
            nr = attempts(B, Rt, null, os.path.join(WORK, 'r%d_plus_null%d' % (rnd, j)))
            ncmp = decision(nr['st'][0], nr['st'][1], nr['E'], dict(k2, price=p2), rr) if nr['E'] and Ek2 else None
            nid = JL.attempt(round_no=rnd, index=j, decision_id=dec2, phase='control', parent=last, original=None, k=null,
                            before=VL.before(B, K, null, E, Rt=Rt, ww=ww_from(null)), state=st, after_E=nr['E'], status=nr['status'],
                            faults=nr['faults'], wall_s=nr['wall_s'], route_s=nr['route_s'], router_opt=ropt(null), accepted=False,
                            minimum=MIND, candidate_board_sha=nr['kb'], result_sha=nr['rs'], **POLICY)
            ok = bool(c and c['better'] and ncmp is not None and ncmp['better'])
            aid = JL.attempt(round_no=rnd, index=j, decision_id=dec2, phase='recheck', parent=last, original=k2['attempt_id'], k=k2,
                             before=before, state=st, after_E=Ek2, status=rr['status'], faults=rr['faults'], wall_s=rr['wall_s'],
                             route_s=rr['route_s'], router_opt=ropt(k2), accepted=ok, minimum=MIND, candidate_board_sha=rr['kb'], result_sha=rr['rs'],
                             comparison=c, control_id=nid, control_comparison=ncmp, **POLICY)
            if ok:
                (B, Rt), E = rr['st'], Ek2; used |= set(k2['refs']); last = aid
                if k2['category'] == 'swap': TABU.add(frozenset(k2['refs']))
                if k2['category'] == 'rotate':
                    for r_, v_ in k2['refs'].items(): ROTATE_TABU.add((r_, (360.0 - v_[2]) % 360))
                for r, v in k2['refs'].items(): moves[r][0] += v[0]; moves[r][1] += v[1]; moves[r][2] += v[2]
                log('   additionally %s: score %.1f' % (move_text(k2), nw(E)))
        JL.round_finished()
    json.dump(B, open(os.path.join(D, 'board_after.json'), 'w')); json.dump(Rt, open(os.path.join(D, 'routes_after.json'), 'w'))

    changed = {f['ref'] for f in B['fps'] if f != next(g for g in B0['fps'] if g['ref'] == f['ref'])}
    fb = {f['ref']: f for f in B['fps']}
    P = dict(parts={r: dict(side=fb[r]['side'], pads={p['num']: [p['x'], p['y']] for p in B['pads'] if p['ref'] == r}) for r in sorted(changed)})
    from placement.p3_groups import board_with_groups
    P = board_with_groups(P, B)
    json.dump(P, open(os.path.join(D, 'placement_after.json'), 'w'))
    json.dump(E, open(os.path.join(WORK, 'result_evaluation.json'), 'w'))
    log('Result (%s): score %.1f, vias %d, detours %.1f mm, deficit %.1f, %s; moved %s; %.0f s' % (
        SCORE, nw(E), E['vias'], E['detour_total_sum'], E['deficit'], 'is_valid' if E['is_valid'] else 'INVALID',
        {r: (round(v[0], 2), round(v[1], 2), v[2] % 360) for r, v in moves.items()}, time.time() - t0))
    if WW:
        fb = {f['ref']: f for f in B['fps']}
        for r in sorted(moves):
            w = WT[r]; fm = lambda x: 'whole' if x == math.inf else '%.1f' % x
            log('   Travel path %-10s %.2f mm (%s: reason %s, max %s, %.0f/mm)' % (r, way(fb, r), w['keyname'], fm(w['reason']), fm(w['max']), w['importance']))


if __name__ == '__main__':
    main()
