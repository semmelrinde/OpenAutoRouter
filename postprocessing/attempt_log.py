import copy, hashlib, json, math, os, platform, sys, uuid
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

SCHEMA = 'attempt.v1'
SCORE_KEY = 'rating'
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CODE = ['router/route_pf.py', 'router/astar_nb.py', 'analysis/netclasses.py', 'analysis/evaluation.py', 'postprocessing/improve.py',
        'postprocessing/move_rules.py', 'analysis/bottlenecks.py', 'analysis/measures.py', 'analysis/measures_weights.json',
        'analysis/minimax.py', 'analysis/current_chain.py', 'analysis/copper_model.py', 'analysis/rule_metrics.py',
        'analysis/standard.py', 'analysis/standard.json', 'analysis/topology.py', 'analysis/coupling.py', 'common/travel_range.py',
        'postprocessing/attempt_log.py', 'common/shift.py', 'common/flow.py', 'common/parts.py', 'common/offset.py',
        'codex/ir_drop/solver.py', 'codex/zones_ir/fill.py', 'codex/zones_ir/contours.py',
        'codex/zones_ir/solver_fast.py', 'codex/zones_ir/integration.py']

CODE = sorted(set(CODE + ['learning/copper_bars.py', 'analysis/standard.json'] +
    [os.path.relpath(os.path.join(root, f), R)
     for folder in ('analysis', 'common', 'codex/zones_ir', 'codex/ir_drop')
     for root, _, files in os.walk(os.path.join(R, folder)) for f in files if f.endswith('.py')]))


def sha_file(path):
    try:
        with open(path, 'rb') as f: return hashlib.sha256(f.read()).hexdigest()
    except OSError: return None


def sha_obj(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def code_hashes():
    return {c: sha_file(os.path.join(R, c)) for c in CODE}


def environment():
    import numpy, scipy, shapely
    return dict(python=sys.version.split()[0], numpy=numpy.__version__, scipy=scipy.__version__, shapely=shapely.__version__,
                platform=platform.platform(), cpu=platform.machine(), pythonhashseed=os.environ.get('PYTHONHASHSEED'))


def _finite(x):
    return x if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else None


def score(E):
    if E is None: return None
    return copy.deepcopy(dict(rating=_finite(E.get(SCORE_KEY)), score_key=SCORE_KEY, score_v3=_finite(E.get('score_v3')), score_v3_version=E.get('score_v3_version'),
                minimax=E.get('minimax'), minimax_version=E.get('minimax_version'), minimax_profile=E.get('minimax_profile'),
                minimax_rank=E.get('minimax_rank'), minimax_total_sum=E.get('minimax_total_sum'), chokepoint=E.get('chokepoint'),
                minimax_sources=E.get('minimax_sources'), minimax_current_paths=E.get('minimax_current_paths'),
                design_rules=E.get('design_rules'), rule_gate=E.get('rule_gate'), evaluation_target=E.get('evaluation_target'),
                minimax_valid=E.get('minimax_valid'), minimax_status=E.get('minimax_status'),
                minimax_board_contract=E.get('minimax_board_contract'),
                minimax_complete=E.get('minimax_complete'), safety=E.get('safety'),
                move_design_rules=E.get('move_design_rules'), score_v1=E.get('rating'), score_v2=_finite(E.get('score_v2_raw')),
                score_raw=_finite(E.get('score_raw')), is_valid=E.get('is_valid'), open_items=E.get('open_items'),
                conflict_list=E.get('conflict_list'), not_connected=len(E['not_connected']) if E.get('not_connected') is not None else None,
                vias=E.get('vias'), detour_mm=E.get('detour_total_sum'), deficit_mm2=E.get('deficit'), narrowing_mm2=E.get('narrowing'),
                summands={k: _finite(v) for k, v in (E.get('summands') or {}).items()} or None,
                evaluation_version=E.get('evaluation_version'),
                ir_loss_mW=_finite((E.get('ir') or {}).get('loss_mW')), ir_current_mode=(E.get('ir') or {}).get('current_mode'),
                ir_zones_model=(E.get('ir') or {}).get('zones_model')))


def delta(a, b):
    if a is None or b is None: return None
    def d(x, y): return None if x is None or y is None else y - x
    out = {k: d(a.get(k), b.get(k)) for k in ('rating', 'score_v1', 'score_v2', 'score_v3', 'score_raw', 'open_items', 'conflict_list', 'vias', 'detour_mm', 'deficit_mm2', 'narrowing_mm2', 'ir_loss_mW')}
    sa, sb = a.get('summands') or {}, b.get('summands') or {}
    out['summands'] = {k: d(sa.get(k), sb.get(k)) for k in sorted(set(sa) | set(sb))} or None
    return out


def label(before, post, status, minimum):
    if status != 'evaluated' or before is None or post is None or before.get('rating') is None or post.get('rating') is None:
        return dict(policy='acceptance_rule-v1-' + SCORE_KEY, success=None, gain_if_valid=None, relation=None)
    ok = bool(post['is_valid']) and (not before['is_valid'] or post['rating'] < before['rating'] - minimum)
    gain = (before['rating'] - post['rating']) if (before['is_valid'] and post['is_valid']) else None
    rv = (0 if before['is_valid'] else 1, (before['open_items'] or 0) + (before['not_connected'] or 0), before['conflict_list'] or 0, before['rating'])
    rn = (0 if post['is_valid'] else 1, (post['open_items'] or 0) + (post['not_connected'] or 0), post['conflict_list'] or 0, post['rating'])
    return dict(policy='acceptance_rule-v1-' + SCORE_KEY, success=ok, gain_if_valid=gain, relation='better' if rn < rv else ('equal' if rn == rv else 'worse'))


def _outline(B):
    try: return unary_union([Polygon(o) for o in B['outline'] if len(o) >= 3]).buffer(0)
    except Exception: return None


def local_occupancy(B, refs, half=(2.0, 5.0)):
    fps = {f['ref']: f for f in B['fps']}; brett = _outline(B)
    cy = {}
    for c in B.get('courtyards', []):
        for q in c.get('polys', []):
            if len(q) >= 3: cy.setdefault((c['side'], c['ref']), []).append(Polygon(q).buffer(0))
    out = {}
    for r in refs:
        f = fps.get(r)
        if not f: continue
        per = {}
        for h in half:
            fen = box(f['x'] - h, f['y'] - h, f['x'] + h, f['y'] + h)
            if brett is not None: fen = fen.intersection(brett)
            if fen.area <= 1e-9: per['%g' % h] = None; continue
            value_list = {}
            for side in ('F', 'B'):
                foreign = [g for (s, q), gs in cy.items() if s == side and q not in refs for g in gs if g.intersects(fen)]
                value_list[side] = round(unary_union(foreign).intersection(fen).area / fen.area, 4) if foreign else 0.0
            neighbors = sum(1 for q in fps.values() if q['ref'] not in refs and abs(q['x'] - f['x']) <= h and abs(q['y'] - f['y']) <= h)
            per['%g' % h] = dict(courtyard_share=value_list, neighbors=neighbors, window_mm2=round(fen.area, 3))
        out[r] = per
    return dict(version='local-v1-mech', part_list=out)


def surroundings(B, Rt, refs, half=(1.5, 4.0)):
    if Rt is None: return None
    fps = {f['ref']: f for f in B['fps']}
    net_list = {p['net'] for p in B['pads'] if p['ref'] in refs and p['net']}
    conf = Rt.get('conflicts') or {}
    k_nets = {str(u).split('#')[0].split('|')[0] for u in (conf if isinstance(conf, (dict, list)) else [])}
    segs = [(t['net'], (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, math.dist(a, b)) for t in Rt.get('tracks', []) for a, b in zip(t['pts'], t['pts'][1:])]
    vias = [(v['net'], v['x'], v['y']) for v in Rt.get('vias', [])]
    out = {}
    for r in refs:
        f = fps.get(r)
        if not f: continue
        per = {}
        for h in half:
            inb = lambda x, y: abs(x - f['x']) <= h and abs(y - f['y']) <= h
            per['%g' % h] = dict(copper_mm=round(sum(L for n, x, y, L in segs if inb(x, y)), 3),
                                copper_foreign_mm=round(sum(L for n, x, y, L in segs if inb(x, y) and n not in net_list), 3),
                                vias=sum(1 for n, x, y in vias if inb(x, y)))
        dk = [math.hypot(x - f['x'], y - f['y']) for n, x, y, L in segs if n in k_nets] + [math.hypot(x - f['x'], y - f['y']) for n, x, y in vias if n in k_nets]
        out[r] = dict(window=per, conflict_nets_at_part=len(net_list & k_nets), conflict_spacing_mm=round(min(dk), 3) if dk else None)
    return dict(version='surroundings-v1', conflicts_total=len(conf), part_list=out)


def before(B, K, k, E0, Rt=None, ww=None):
    fps = {f['ref']: f for f in B['fps']}; NET = K['net_list']; PARTS = K['part_list']; GND = set(K['gnd'])
    refs = list(k['refs'])
    pads = {r: [p for p in B['pads'] if p['ref'] == r] for r in refs}
    part_list = []
    for r in refs:
        f = fps[r]; t = PARTS.get(r, {})
        part_list.append(dict(ref=r, role=t.get('role'), function=t.get('function'), decoupling=t.get('decoupling'), pinned=t.get('pinned'), island=t.get('island'),
                          fp=f.get('fp'), amount=f.get('value'), n_pads=len({p['num'] for p in pads[r]}), board_side=f['side'],
                          pose=dict(x=f['x'], y=f['y'], rot=f.get('rot', 0.0))))
    net_list = {}
    for r in refs:
        for p in pads[r]:
            n = p['net']
            if not n: continue
            net_list.setdefault(n, dict(reason='part_pad', category='gnd' if n in GND else NET.get(n, {}).get('category'),
                                     rank=NET.get(n, {}).get('rank'), w=NET.get(n, {}).get('w'), wmin=NET.get(n, {}).get('wmin'),
                                     gnd=n in GND, pads=sum(1 for q in B['pads'] if q['net'] == n),
                                     flow_unit=(NET.get(n, {}).get('flow') or {}).get('unit')))
    goal = k.get('net_name') or k.get('goal_net')
    if goal and goal not in net_list:
        n = goal
        net_list[n] = dict(reason='explicit_reroute', category=NET.get(n, {}).get('category'), rank=NET.get(n, {}).get('rank'),
                        w=NET.get(n, {}).get('w'), wmin=NET.get(n, {}).get('wmin'), gnd=n in GND,
                        pads=sum(1 for q in B['pads'] if q['net'] == n), flow_unit=(NET.get(n, {}).get('flow') or {}).get('unit'))
    reason = k.get('source') or 'unknown'
    if reason == 'bottleneck' and 'onflict' in k.get('reason', ''): reason = 'conflict'
    return dict(focus=refs[0] if refs else None, part_list=part_list,
                move=dict(type=k['category'], members={r: dict(dx_mm=v[0], dy_mm=v[1], rotation_deg=v[2]) for r, v in k['refs'].items()},
                          sides=k.get('sides'), route_moves=k.get('route_moves'), target_net=k.get('goal_net')),
                reason=dict(type=reason, text=k.get('reason'), trigger_net_name=k.get('net_name'), gain_estimate=k.get('gain')),
                local_occupancy=local_occupancy(B, refs), affected_nets=[dict(name=n, **v) for n, v in sorted(net_list.items())],
                score=score(E0), price=_finite(k.get('price')), surroundings=surroundings(B, Rt, refs),
                travel_range={r: dict(keyname=w.get('keyname'), reason=_finite(w.get('reason')), max=_finite(w.get('max')),
                                     whole=w.get('max') == math.inf, importance=_finite(w.get('importance')), rotate=w.get('rotate'))
                             for r, w in sorted((ww or {}).items())} or None)


class AttemptLog:
    def __init__(self, fpath, meta):
        self.f = open(fpath, 'a'); self.meta = meta

    def store(self, rec):
        self.f.write(json.dumps(rec, ensure_ascii=False, allow_nan=False) + '\n'); self.f.flush()

    def round_finished(self):
        try: os.fsync(self.f.fileno())
        except OSError: pass

    def attempt(self, *, round_no, index, decision_id, phase, parent, original, k, before, state, after_E, status, faults,
                wall_s, route_s, router_opt, accepted, minimum, candidate_board_sha, result_sha, policy=None, model_id=None,
                comparison=None, control_id=None, control_comparison=None, attempt_id=None):
        before_s = before['score']; after_s = score(after_E)
        lab = label(before_s, after_s, status, minimum)
        if comparison is not None:
            lab.update(policy='acceptance_rule-v3-equal-design_rules-price-null', success=bool(comparison['better']),
                       relation='better' if comparison['better'] else 'not_better')
            if phase != 'control':
                lab['success'] = bool(comparison['better'] and control_comparison['better']) if control_comparison else None
            lab['gain_vs_control'] = -control_comparison['delta'] if control_comparison and control_comparison.get('delta') is not None else None
            lab['gain_vs_basis'] = -comparison['delta'] if comparison.get('delta') is not None else None
            if phase != 'control': lab['gain_if_valid'] = lab['gain_vs_control']
        if phase == 'control': lab['success'] = None
        rec = dict(schema_version=SCHEMA, record_origin='native',
                   ids=dict(attempt_id=attempt_id or str(uuid.uuid4()), run_id=self.meta['run_id'], board_id=self.meta['board_id'],
                            board_family_id=self.meta['family'], round=round_no, candidate_index=index, decision_id=decision_id,
                            parent_attempt_id=parent, original_attempt_id=original),
                   provenance=dict(snapshots=dict(state, candidate_board=candidate_board_sha, result_routes=result_sha),
                                   historical_code_sha256=self.meta['code'], router_seed=None, pythonhashseed='0',
                                   router_options=router_opt, environment=self.meta['environment'], started_at_utc=self.meta['start_utc']),
                   decision=dict(phase=phase, minimum=minimum, selection_policy=policy or 'heuristic-gain-pro-effort-v1', selection_probability=None, model_id=model_id),
                   before=before,
                   result=dict(status=status, faults=faults, score=after_s, delta=delta(before_s, after_s),
                               wall_s=round(wall_s, 3) if wall_s is not None else None, route_s=round(route_s, 3) if route_s is not None else None,
                               comparison=comparison, control_attempt_id=control_id, control_comparison=control_comparison),
                   label=lab, selection=dict(accepted=accepted),
                   quality=dict(absent=['local_occupancy.copper_per_layer', 'pending_journal'], feature_version='improve-v3'))
        self.store(rec)
        return rec['ids']['attempt_id']
