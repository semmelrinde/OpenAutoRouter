"""Self-play loop: generate tasks, solve them, score them, learn which settings work.

For every task from learning/tasks.py (consecutive seeds): net classes, placement (placement/placer.py
mode=new), apply the placement to board.json without KiCad (common/shift.py), route
(router/route_pf.py with a CPU budget from common/slots.py) and score (score v3 and rule metrics).
  invalid -> place and route again with another setting (at most attempts per size);
  valid   -> shrink the board by densify (default 0.93) and continue until no setting is valid any more;
  never valid at the start size -> up to two times 12 % more room.
Every attempt is one line in the data file (task, setting, valid, score, area, times). A Thompson-sampling bandit picks
the next setting per task class (layers, BGA or not) from these lines, so the loop learns what becomes valid on dense
boards.

usage: caffeinate -i -s python3 learning/selfplay.py <work_folder> [tasks=0 (0 = endless)] [parallel=4]
       [attempts=3] [densify=0.93] [time_s=300 (CPU seconds per run)] [iter=40] [move=1]
"""
import copy, hashlib, json, math, os, random, re, shutil, sys, threading, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
from common.process_guard import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from learning import tasks as AU
from learning import curriculum_tempo as TT
from learning import placer_loosen as LOOSEN
from learning import early_abort as FR
from learning import champion as ME
from analysis import minimax as MM
from analysis.standard import goal as default_target
from common import slots as PL
from analysis import contract_spec as VT
VPY = os.path.join(R, '.venv', 'bin', 'python')
LOCK = threading.Lock()
PAIR_SERVICES = {}
PAIR_LOCK = threading.Lock()


def learning_cost_early_abort(cfg, rnd):
    if cfg.get('budget') != 'work' and 'early_abort' not in cfg and rnd.random() < .8:
        cfg['early_abort']='q1'
    return cfg


def pair_service(model=None):
    from learning.judge_pair import Service, MODEL
    key = str(model or MODEL)
    with PAIR_LOCK:
        if key not in PAIR_SERVICES: PAIR_SERVICES[key] = Service(model=key)
        return PAIR_SERVICES[key]
QUAL_TOLERANCE = 1.15

ARMS = dict(cross=[1, 2, 4], separate=[2, 3], via_cost=[0.3, 0.6, 1.0], fine=['1', '0'], pfmul=[1.25, 1.4], shaping=['45r', 'organic'],
            placer=['prior', 'island'], router=['prior', 'convergence'])


PLACER_VARIANTS = {'prior': [], 'island': ['island_physics=1', 'pin_post_legalize=1', 'bga_against=1', 'operate_new_free=1']}


ROUTER_VARIANTS = {'prior': [], 'convergence': ['conflict_site=exact', 'source_free=1', 'convergence=local', 'pfmax=1000', 'short_via=8'],
                   'r27': ['r27_trunk=1']}


def net_class(A, goal='v3'):
    k = ('power_' if A.get('power_board') else '') + 'L%d_%s_s%d' % (A['cu_layers'], 'bga' if any('BGA' in b for b in A['blocks']) else 'qfn', int(round(A.get('level', 0) / 2.0) * 2))

    return k + '|profile_data:' + A.get('profile_data', 'balanced') if goal == 'minimax' else k


def current_val(x):
    sx = (x.get('rule') or {}).get('current_x_max')
    if not isinstance(sx, (int, float)) or isinstance(sx,bool) or not math.isfinite(sx): return 'unknown', None
    return ('violated' if sx > 1.0 else 'fulfilled'), float(sx)


CURRENT_RANK = dict(fulfilled=0, unknown=1, violated=2)


def rewards(lines):


    minima={}
    for x in lines:
        if not x.get('is_valid'):continue
        r=x.get('rule') or {};sig=r.get('measurement_version') or (x.get('gate_check') or {}).get('measurement_version')
        if not sig:continue
        key=(x.get('task'),round(x.get('factor') or 0,3),sig)
        vals=minima.setdefault(key,{})
        for k,v in VT.from_row(x)['design_rules'].items():
            if v is not None:vals[k]=min(vals.get(k,float('inf')),v)
    best = {}
    for x in lines:
        if x.get('teacher_status', 'fulfilled') == 'fulfilled' and x.get('is_valid') and isinstance(x.get('score_v3'), (int, float)) and current_val(x)[0] == 'fulfilled':
            k = (x.get('task'), round(x.get('factor') or 0, 3)); best[k] = min(best.get(k, 9e99), x['score_v3'])
    out = []
    for x in lines:
        if x.get('teacher_status', 'fulfilled') != 'fulfilled' or not x.get('is_valid'): out.append(2.0); continue
        st, sx = current_val(x)
        if st == 'unknown': out.append(1.5); continue
        if st == 'violated': out.append(round(1.5 + 1e-4 + 0.4899 * min(1.0, math.log(sx) / math.log(50.0)), 4)); continue
        b = best.get((x.get('task'), round(x.get('factor') or 0, 3))); n = x.get('score_v3')
        r=x.get('rule') or {};sig=r.get('measurement_version') or (x.get('gate_check') or {}).get('measurement_version')
        if sig:
            vals=VT.from_row(x)['design_rules'];ref=minima.get((x.get('task'),round(x.get('factor') or 0,3),sig),{})
            if any(v is None for v in vals.values()):out.append(1.5);continue
            ratios=[max(0.,v-ref.get(k,v))/max(abs(ref.get(k,v)),1e-9) for k,v in vals.items()]
            excess=sum(q/(1+q) for q in ratios)/max(1,len(ratios))
            out.append(1.+.49*excess);continue
        out.append(1.0 + 0.49 * min(1.0, n / b - 1.0) if b and isinstance(n, (int, float)) and b > 0 else 1.25)
    from learning.judge_pair import quality_reward
    return [pair_service(x.get('pair_model_path')).reward(x, quality_reward(x, b) if x.get('pair_quality') else b)
            if x.get('pair_active') else b for x, b in zip(lines, out)]


def evaluation_target(z):
    return (z.get('rule') or {}).get('evaluation_target') or 'v3'


def reward(z):
    if z.get('teacher_status', 'fulfilled') != 'fulfilled': return 2.0
    if evaluation_target(z) != 'minimax': return rewards([z])[0]
    b = MM.reward(z)
    return pair_service(z.get('pair_model_path')).reward(z, b) if z.get('pair_active') else b


def better_than(z, best):
    decision = None
    if best is not None and z.get('pair_active') and z.get('area_mm2') == best.get('area_mm2'):
        judge = pair_service(z.get('pair_model_path'))
        try:
            with judge.lock:
                judge.load_data(); released = judge.applicable(z) and judge.applicable(best.get('row') or best)
        except (OSError, ValueError, KeyError): released = False
        if released:
            decision = judge.comparison(best.get('row') or best, z)
            if not decision['not_worse']: return False
    if _better_than_basis(z, best): return True
    if best is not None and z.get('pair_active') and z.get('area_mm2') == best.get('area_mm2'):
        before = best.get('row') or best
        if evaluation_target(z) == 'minimax' and not MM.assumption(before.get('rule') or {}, z.get('rule') or {})['not_worse']:
            return False
        return bool(decision and decision['pair'] and decision['better'])
    return False


def _better_than_basis(z, best):
    if not z.get('is_valid'): return False
    if z.get('gate_check') or (z.get('rule') or {}).get('gate_check'):
        current=VT.from_row(z)
        if current['electrical']=='violated' or any(v is None for v in current['design_rules'].values()):return False
        if best is not None:
            before=VT.from_row(best.get('row') or best)
            if not VT.comparison(before,current)['not_worse']:return False
    if evaluation_target(z) != 'minimax':


        n, nb = z.get('score_v3'), (best or {}).get('score_v3')
        if best is None: return True
        if not isinstance(n, (int, float)): return False


        (sz, xz), (sb, xb) = current_val(z), current_val(best)
        if CURRENT_RANK[sz] != CURRENT_RANK[sb]: return CURRENT_RANK[sz] < CURRENT_RANK[sb]
        if sz == 'violated':
            if xz > xb * 1.1: return False
            if xz < xb / 1.1 and z['area_mm2'] <= best['area_mm2'] + 1e-6: return True
        qual_ok = not isinstance(nb, (int, float)) or n <= nb * QUAL_TOLERANCE
        return (z['area_mm2'] < best['area_mm2'] - 1e-6 and qual_ok) or (
            z['area_mm2'] <= best['area_mm2'] + 1e-6 and (not isinstance(nb, (int, float)) or n < nb))
    b = z.get('rule') or {}
    if not MM.current(b) or not MM.complete(b) or not b.get('safety', {}).get('safe'): return False
    if best is None: return True
    a = best.get('rule') or {}
    if evaluation_target(best) != 'minimax': return False
    c = MM.assumption(a, b)
    if not c['not_worse']: return False
    return c['better'] if c['delta'] else z['area_mm2'] < best['area_mm2']


def class_matching(x, nc):
    k = x.get('net_class', '')
    if k == nc: return True

    if re.search(r'_s\d+(?:$|\|)', k): return False
    base = k.split('|profile_data:')[0]
    for p in ('balanced', 'low_power', 'high_end_si', 'power'):
        if base.endswith('_' + p): base = base[:-len(p)-1]; break
    st = int(round(x.get('level_task', 0) / 2.0) * 2)
    new = base + '_s%d' % st
    if evaluation_target(x) == 'minimax': new += '|profile_data:' + (x.get('rule') or {}).get('minimax_profile', 'balanced')
    return new == nc


def arm(x, k):
    v = (x.get('cfg') or {}).get(k)
    if k == 'bga' and v is None: return 'off'
    return 'prior' if k in ('placer', 'router') and v is None else v


BGA_VARIANTS = {'off': [], 'fanout': ['bga_fanout=1'],
                 'plan': ['bga_fanout=1', 'bga_plan=1'],
                 'lanes': ['bga_fanout=1', 'bga_plan=1', 'bga_lanes=1'],
                 'own_via': ['bga_fanout=1', 'bga_plan=1', 'bga_own_via=1'],
                 'safe': ['bga_fanout=1', 'bga_plan=1', 'bga_own_via=1', 'bga_inner_width=1']}
def shape_arms_read(amount):
    if amount is None: return None
    if amount == '0': return list(ARMS['shaping'])
    if not isinstance(amount, str): raise ValueError('shape_arms must be a comma-separated string')
    arms = [x.strip() for x in amount.split(',')]
    if any(x not in ('45r', 'organic', 'per_net_kind') for x in arms) or len(set(arms)) != len(arms):
        raise ValueError('shape_arms: unique values from 45r,organic,per_net_kind expected')
    return arms


def choice(lines, nc, rnd, goal='v3', bga_axis=False, style_per_net_kind=False, shape_arms=None, tempo_error_book=None, move_arm=False):
    lines = [x for x in lines if not FR.aborted(x) and not x.get('generator_failure') and not x.get('not_routed') and evaluation_target(x) == goal and
              x.get('recipe', {}).get('category') != 'calibration' and x.get('cur_state') != 'code_changed']
    if goal == 'minimax':
        profile_data = nc.split('|profile_data:')[-1]
        lines = [x for x in lines if (x.get('rule') or {}).get('minimax_profile') == profile_data and reward(x) is not None]
    lines = [x for x in lines if str(x.get('version', '')).split('+')[0] == AU.VERSION]
    lines = lines[-3000:]
    rew = [reward(x) for x in lines] if goal == 'minimax' else rewards(lines)
    cfg = {}
    arms = dict(ARMS)
    shapes = shape_arms_read(shape_arms)
    if shapes is not None: arms['shaping'] = shapes
    elif style_per_net_kind: arms['shaping'] = [*arms['shaping'], 'per_net_kind']
    if bga_axis and re.search(r'L[68]_bga_', nc):
        arms['bga'] = list(BGA_VARIANTS)
    if move_arm: arms['move'] = ['0', '1']
    for k, value_list in sorted(arms.items()):
        best, bw = None, 9e99
        for w in value_list:
            r = [b for x, b in zip(lines, rew) if b is not None and arm(x, k) == w and class_matching(x, nc)] or \
                [b for x, b in zip(lines, rew) if b is not None and arm(x, k) == w
                 and (k != 'bga' or re.search(r'L[68]_bga_', x.get('net_class', '')))]
            if not r: m, s = 1.5, 0.5
            else:
                m = sum(r) / len(r); s = max(0.1, (sum((q - m) ** 2 for q in r) / max(1, len(r) - 1)) ** 0.5) / math.sqrt(len(r))
            v = rnd.gauss(m, s)
            if tempo_error_book is not None: v += TT.arm_penalty(tempo_error_book, k, w)
            if v < bw: best, bw = w, v
        cfg[k] = best
    cfg['seed'] = rnd.randint(1, 10 ** 6)
    return cfg


SHORT_CIRCUIT = {}
ZONE_PLAN = dict(mode='off', r36='geometric')

def job(cmd, cpu, log):
    r = PL.job(cmd, cpu_s=cpu, log=log, env=dict(os.environ, PYTHONHASHSEED='0'), cwd=R)
    return r['cur_state'], r['cpu_s']


def code_hashes():
    from learning.collect import code_state
    return dict(code_sha256=code_state(),
                router_sha256=hashlib.sha256((Path(R) / 'router/route_pf.py').read_bytes()).hexdigest(),
                netclasses_sha256=hashlib.sha256((Path(R) / 'analysis/netclasses.py').read_bytes()).hexdigest())


def code_verify(z):
    now = code_hashes()
    if any(z.get(k) != v for k, v in now.items()):
        z.update(code_changed=True, code_after=now, is_valid=False, cur_state='code_changed')
        return False
    return True



_MAIN = Path(os.environ.get('AUTOROUTER_MAIN') or Path(__file__).resolve().parents[1])
MOVE_MODEL = _MAIN / 'learning/model/move_choice.json'
MOVE_DATA = _MAIN / 'learning/data/moves_end_phase.jsonl'


def moves_collect(c, A, cfg):
    import fcntl, glob
    lines = []
    try: nc = net_class(A)
    except Exception: nc = None
    for f in sorted(glob.glob(os.path.join(c, '**', '*_moves.jsonl'), recursive=True)):
        for l in open(f):
            z = json.loads(l); z.update(task=A.get('seed'), net_class=nc, file=os.path.relpath(f, c))
            lines.append(json.dumps(z, sort_keys=True))
    if not lines: return
    with open(MOVE_DATA, 'a') as out:
        fcntl.flock(out, fcntl.LOCK_EX); out.write('\n'.join(lines) + '\n'); fcntl.flock(out, fcntl.LOCK_UN)


def _attempt(ad, A, cfg, k, time_s, it, only_place=False, prepared=False, cpu=None, logbook=None):
    from common.shift import relocate
    from analysis import corpus as KO
    from learning import collect as S
    cfg = copy.deepcopy(cfg)
    mk = ME.net_class(A); champion_doc = None
    if cfg.get('champion', '1') != '0' and str(cfg.get('std', '1')) != '0' and 'cost' not in cfg:
        champion_doc = ME.read_in(cfg.get('champion_cfg'))
        _, entry = ME.selection(champion_doc, mk, exact=True)
        if entry:
            cfg['cost'] = entry['parameter']; cfg['champion_generation'] = entry['generation']
    cost = ME.parameter(cfg.get('cost', {}))
    if 'shaping' in cost:cfg['shaping']=cost['shaping']
    if 'placer_style' in cost and isinstance(cfg.get('placer','prior'),str):cfg['placer']=cost['placer_style']
    if 'router_style' in cost and isinstance(cfg.get('router','prior'),str):cfg['router']=cost['router_style']


    def champion_options(tool):
        keys = ME.PLACER if tool == 'placer' else ME.ROUTER
        source = ['champion_cfg=' + os.path.join(c, 'champion_cfg.json')] if champion_doc and cost else ['champion=0']
        return source + (['std='+str(cfg['std'])] if 'std' in cfg else []) + ['champion_net_class=' + mk] + [key+'='+value for key,value in sorted(ME.tool_options(cost,tool).items())]
    c = os.path.join(ad, 'k%d' % k); os.makedirs(c, exist_ok=True); t0 = time.time()
    if champion_doc and cost:
        Path(c, 'champion_cfg.json').write_text(json.dumps(champion_doc))
    z = dict(task=A['seed'], candidate=k, net_class=net_class(A, A.get('evaluation_target', 'v3')), cu_layers=A['cu_layers'], factor=A['factor'], area_mm2=A['area_mm2'],
             part_list=A['part_list'], pads=A['pads'], net_list=A['net_list'], cfg=cfg, version=A.get('version', AU.VERSION), time_s=time.strftime('%Y-%m-%d %H:%M:%S'),
             level_task=A.get('level', 0), blocks=A['blocks'], length_groups=A.get('length_groups'))
    if logbook is not None:
        logbook.update(z); z = logbook
    if LOOSEN.active(cfg.get('placer_loosen')): z.update(LOOSEN.label(A))
    z['_cpu_kind_s'] = 0.0
    budget = int(time_s * min(5.0, max(1.0, A['pads'] / 300.0)))
    if cfg.get('budget') == 'work': budget = int(cfg.get('work_cpu_emergency_stop', 60000))
    if cfg.get('learning_budget') == 'adaptive': budget = int(time_s)
    def compute_run(cmd, limit, log):
        if cpu is None:
            st, used = job(cmd, limit, log)
            z['_cpu_kind_s'] += used
            return st, used
        if cpu.rest < 1: return 'time_limit', 0.0
        st, used = job(cmd, min(limit, int(cpu.rest)), log)
        cpu.add(used)
        z['_cpu_kind_s'] += used
        return st, used
    def classify(bj, kj, name):
        if cpu is None: return KO.netclasses_run(ad, bj, kj, os.path.join(ad, 'specs.json'))
        cmd = [VPY, os.path.join(R, 'analysis/netclasses.py'), bj, kj,
               'design_rules=' + os.path.join(ad, 'design_rules.json'), 'specs=' + os.path.join(ad, 'specs.json')]
        st, cp = compute_run(cmd, 300, os.path.join(c, name + '.log'))
        z[name] = dict(cur_state=st, cpu_s=cp)
        if st != 'ok': return None
        return json.load(open(kj))
    if not prepared:
        z.update(code_hashes(), format='v2', gnd_layers=list(A['gnd_layers']))
        if A.get('layer_stack'):
            z.update(format='v3', layer_stack=True, power_board=A.get('power_board', False))
        if cpu:
            from learning.teacher import context
            z['context'] = context(cpu.limit, it, z['code_sha256'])
            if classify(os.path.join(ad, 'board.json'), os.path.join(ad, 'netclasses.json'), 'netclasses_input') is None:
                z.update(is_valid=False, cur_state='netclasses_input_' + z['netclasses_input']['cur_state']); return z
        pcmd = [VPY, os.path.join(R, 'placement/placer.py'), os.path.join(ad, 'board.json'), os.path.join(ad, 'netclasses.json'),
                os.path.join(c, 'placement.json'), 'design_rules=' + os.path.join(ad, 'design_rules.json'), 'mode=new', 'sides=F', 'rotate_rule=0',
                'seed=%d' % cfg['seed'], 'cross=%g' % cfg['cross'], 'separate=%g' % cfg['separate'], 'via_cost=%g' % cfg['via_cost'], 'fine=' + cfg['fine']]
        pv = cfg.get('placer') or []
        pcmd += list(PLACER_VARIANTS[pv] if isinstance(pv, str) else pv)
        pcmd += [f'{key}={cfg[key]}' for key in ('dense', 'dense_inner', 'p35') if key in cfg]
        pcmd += champion_options('placer')
        if cfg.get('copper_reserve'): pcmd += ['copper_reserve=1']
        zp, cp = compute_run(pcmd, budget, os.path.join(c, 'placer.log'))
        z['placer_attempts'] = [dict(seed=cfg['seed'], cur_state=zp, cpu_s=cp)]


        def _kol():
            try: return ((json.load(open(os.path.join(c, 'placement.json'))).get('metrics') or {}).get('after') or {}).get('collisions')
            except (OSError, ValueError): return None
        z['placer_repeat'] = 0


        for w_ in range(0 if LOOSEN.active(cfg.get('placer_loosen')) else 2):
            if zp != 'ok' or _kol() == 0: break
            z['placer_repeat'] += 1
            pcmd2 = [x if not x.startswith('seed=') else 'seed=%d' % (cfg['seed'] + 7919 * (w_ + 1)) for x in pcmd]
            zp, cp2 = compute_run(pcmd2, budget, os.path.join(c, 'placer_repeat%d.log' % (w_ + 1))); cp += cp2
            z['placer_attempts'].append(dict(seed=cfg['seed'] + 7919 * (w_ + 1), cur_state=zp, cpu_s=cp2))
        z['placer'] = dict(cur_state=zp, cpu_s=cp)
        if zp != 'ok' or not os.path.exists(os.path.join(c, 'placement.json')):
            z.update(is_valid=None if cfg.get('budget') == 'work' else False, cur_state='placer_' + zp, s=round(time.time() - t0, 1)); return z
        kol = _kol()
        if kol is not None and kol > 0 and str(cfg.get('legal_lp', '0')).lower() in ('1', 'true'):



            cmd = [VPY, os.path.join(R, 'placement/legal_lp.py'),
                   os.path.join(ad, 'board.json'), os.path.join(ad, 'netclasses.json'),
                   os.path.join(ad, 'design_rules.json'), os.path.join(c, 'placement.json')]
            if cfg.get('budget') == 'work': cmd += [os.path.join(c, 'placement.json'), 'budget=work']
            placement_file = Path(c) / 'placement.json'
            original = placement_file.read_bytes()
            zl, cl = compute_run(cmd, budget if cfg.get('budget') == 'work' else 60, os.path.join(c, 'legal_lp.log'))
            z['legal_lp'] = dict(cur_state=zl, cpu_s=cl)
            z['placer']['cpu_s'] += cl
            if zl != 'ok': placement_file.write_bytes(original)
            if zl != 'ok' and cfg.get('budget') == 'work':
                z.update(is_valid=None, cur_state='legal_lp_incomplete'); return z
            kol = _kol()
        if kol != 0:
            try: z['placer_metrics'] = (json.load(open(os.path.join(c, 'placement.json'))).get('metrics') or {}).get('after')
            except (OSError, ValueError): pass
            z.update(is_valid=False, cur_state='placer_collision', s=round(time.time() - t0, 1),
                     invalid_reason='Placement: %s' % ('%d collisions' % kol if kol is not None else 'Collision measurement missing'))
            return z
        B = json.load(open(os.path.join(ad, 'board.json'))); P = json.load(open(os.path.join(c, 'placement.json')))['parts']
        fps = {f['ref']: f for f in B['fps']}
        moves = {r: (p['x'] - fps[r]['x'], p['y'] - fps[r]['y'], p.get('rotation', 0.0)) for r, p in P.items() if r in fps}
        BA, _ = relocate(B, moves, sides={r: p['side'] for r, p in P.items() if r in fps})
        from placement.p3_groups import board_with_groups
        BA = board_with_groups(BA, json.load(open(os.path.join(c, 'placement.json'))))
        reserve = json.load(open(os.path.join(c, 'placement.json'))).get('copper_reserve')
        if reserve: BA['copper_reserve'] = reserve
        bj = os.path.join(c, 'board.json'); json.dump(BA, open(bj, 'w'))
        KA = classify(bj, os.path.join(c, 'netclasses.json'), 'netclasses_placement')
        if KA is None:
            z.update(is_valid=False, cur_state='netclasses_placement_' + z['netclasses_placement']['cur_state']); return z
        try:
            from learning.features import feats
            z['feats'] = feats(BA, KA, json.load(open(os.path.join(ad, 'design_rules.json'))), gnd_layers=A['gnd_layers'])
            z['placer_metrics'] = (json.load(open(os.path.join(c, 'placement.json'))).get('metrics') or {}).get('after')
        except Exception as e: z['features_error'] = repr(e)[:160]
        if TT.active(cfg.get('curriculum_tempo', '0')):
            z['tempo_judge'] = TT.judge_shadow(z, cfg.get('tempo_judge_model', os.path.join(R, 'learning/model/judge')))
        z['budget'] = dict(cpu_s=budget, iter=it)
        z['s'] = round(time.time() - t0, 1)
        if not code_verify(z): return z
        if cpu: cpu.logbook(z)
        json.dump({key: value for key, value in z.items() if key != '_cpu_kind_s'},
                  open(os.path.join(c, 'prepared.json'), 'w'))
    else:
        loaded = json.load(open(os.path.join(c, 'prepared.json')))
        if logbook is not None:
            logbook.update(loaded); z = logbook
        else: z = loaded
        from learning.learning_effect import cpu_s
        preparation = cpu_s(loaded)
        if preparation is None:
            z.update(cpu_total=None, cpu_preparation_unknown=True,
                     is_valid=False, cur_state='faults',
                     faults='prepared CPU unknown'); return z
        z['_cpu_kind_s'] = 0.0
        z['_cpu_preparation_s'] = preparation
        if cpu:
            from learning.teacher import context
            cpu.add(preparation)
            if z.get('context') != context(cpu.limit, it, z.get('code_sha256')):
                z.update(is_valid=False, cur_state='faults', faults='prepared CPU context changed'); return z
        t0 -= z.get('s', 0)
        bj = os.path.join(c, 'board.json')
        BA = json.load(open(bj)); KA = json.load(open(os.path.join(c, 'netclasses.json')))
        if z.get('budget') != dict(cpu_s=budget, iter=it) or z.get('cfg') != cfg:
            z.update(is_valid=False, cur_state='faults', faults='prepared router context changed'); return z
    if not code_verify(z): return z
    if only_place: return z
    rj = os.path.join(c, 'routes.json')
    mode = cfg.get('zone_plan', ZONE_PLAN['mode'])
    if mode not in ('off', 'diagnose', 'local'): raise ValueError('zone_plan=off|diagnose|local')
    r36 = cfg.get('r36', ZONE_PLAN['r36'])
    if r36 not in ('legacy','kicad_native','geometric','0'): raise ValueError('r36=legacy|kicad_native|geometric|0')
    plan_extra = []
    if mode != 'off':
        from learning.zone_planner import prepare as zp_prepare
        from contextlib import nullcontext

        with cpu.separate() if cpu and mode == 'diagnose' else nullcontext():
            z['zone_plan'] = zp_prepare(bj, os.path.join(c, 'netclasses.json'), os.path.join(ad, 'design_rules.json'), mode,
                                          A['gnd_layers'], cpu_s=120 if cpu is None or mode == 'diagnose' else max(1,int(cpu.rest)))
        if cpu and mode == 'local': cpu.add(z['zone_plan']['compute_slot']['cpu_s'])
        if mode == 'local':
            if not z['zone_plan']['applied']:
                z.update(is_valid=False,cur_state='zone_plan_error'); return z
            plan_extra = ['zone_plan='+z['zone_plan']['file']]
    rcmd = [VPY, os.path.join(R, 'router/route_pf.py'), bj, rj, 'design_rules=' + os.path.join(ad, 'design_rules.json'), 'netclasses=' + os.path.join(c, 'netclasses.json'),
            'gnd_layers=' + ','.join(A['gnd_layers']), 'iter=%d' % it, 'pf0=1', 'pfmul=%g' % cfg['pfmul'], 'hinc=1.0', 'goal_from=20', 'short=1', 'fair=1',
            'style=' + (cfg['shaping'] if cfg.get('shaping') in ('organic', 'per_net_kind') else '45'),
            'cpu_max=%d' % budget]
    work_measurement = cfg.get('budget') == 'work'
    if work_measurement:
        if str(cfg.get('learning_data', '1')) != '0': raise ValueError('Work measurement needs learning_data=0')
        if cfg.get('learning_budget') or FR.active(cfg.get('early_abort')):
            raise ValueError('Work measurement allows no CPU-controlled learning policy')
        emergency_stop = int(cfg.get('work_cpu_emergency_stop', 60000))
        rcmd = [x for x in rcmd if not x.startswith('cpu_max=')]
        rcmd += ['budget=work', 'work_max=%d' % int(cfg.get('work_max', 59147)),
                 'seed=%d' % cfg['seed'], 'cpu_max=%d' % emergency_stop]
    if str(cfg.get('learning_data', '1')) == '1':
        rcmd += ['learning_data=1', 'labels=' + os.path.join(c, 'labels.npz')]
    route_limit = (emergency_stop if work_measurement else budget) + 60
    if cfg.get('learning_budget') == 'adaptive':
        cap = int(cfg.get('learn_cpu_max', 2 * budget))
        rcmd += ['learning_budget=adaptive', 'learn_cpu_max=%d' % cap]
        route_limit = cap + 60
    if 'budget_before_post_run' in cfg: rcmd += ['budget_before_post_run=' + str(cfg['budget_before_post_run'])]
    if cfg.get('learning_policy'): rcmd += ['learning_policy=' + str(cfg['learning_policy'])]
    if FR.active(cfg.get('early_abort')) and cpu is not None and cfg.get('learning_budget') != 'adaptive':


        rcmd = [x for x in rcmd if not x.startswith('cpu_max=')] + ['cpu_max=%d' % max(1,int(min(budget,cpu.rest)))]
    if FR.active(cfg.get('early_abort')):
        from learning.early_abort import options, context_task
        rcmd += options(cfg, context_task(A))
    rv = cfg.get('router') or 'prior'
    rcmd += (ROUTER_VARIANTS.get(rv, []) if isinstance(rv, str) else list(rv)) + plan_extra
    rcmd += BGA_VARIANTS[cfg.get('bga', 'off')]
    native_options = dict(x.split('=', 1) for x in rcmd[3:] if '=' in x)
    if work_measurement and native_options.get('std', '1') == '1' and 'netclasses' in native_options:

        native_options.setdefault('skew', '1'); native_options.setdefault('attempts', '3')
    if (work_measurement and int(native_options.get('attempts', '0')) > 0
            and 'attempts_cpu_emergency_stop' not in native_options):



        rcmd += ['attempts_cpu_emergency_stop=%d' % (emergency_stop + 60)]
    if (native_options.get('skew') == '1' or int(native_options.get('attempts', '0')) > 0) and not native_options.get('kicad_native_basis'):

        from learning.copper_kicad_native import rule_contract
        from kicad.python_call import python_command
        native = Path(c, 'native_basis'); native.mkdir(exist_ok=True)
        base = native/'base.kicad_pcb'
        native_cmd = (python_command(Path(R, 'codex/generator_real/kicad_native.py'), bj, base, ad)
                      if str(A.get('version', '')).split('+')[0] == '11' else
                      python_command(Path(R, 'kicad/current_board.py'), bj, base))
        nb, nc = compute_run(native_cmd, 300, str(native/'structure.log'))
        z['native_basis'] = dict(cur_state=nb, cpu_s=nc)
        if nb != 'ok':
            z.update(is_valid=None, cur_state='native_basis_incomplete'); return z
        if str(A.get('version', '')).split('+')[0] != '11':
            project, dru = rule_contract(BA, dict(options=native_options), json.load(open(Path(ad, 'design_rules.json'))))
            base.with_suffix('.kicad_pro').write_text(json.dumps(project))
            base.with_suffix('.kicad_dru').write_text(dru)
        zones = (json.loads(Path(ad, 'zones.json').read_text()) if str(A.get('version', '')).split('+')[0] == '11' else
                 [dict(net='GND', layer=layer, pts=poly, full=True, prio=0) for layer in A['gnd_layers'] for poly in BA['outline']])
        (native/'zones.json').write_text(json.dumps(zones))
        rcmd += ['kicad_native_basis='+str(base), 'kicad_native_zones='+str(native/'zones.json')]
    if work_measurement:
        route_limit *= max(1, int(native_options.get('attempts', '0')))
    rcmd += [f'{key}={cfg[key]}' for key in ('hist_start', 'hist_context') if cfg.get(key)]
    rcmd += [f'{key}={cfg[key]}' for key in ('remaining_conflicts', 'rest_cpu', 'rest_parts', 'rest_cand') if key in cfg]
    from postprocessing.short_circuit_protection import protection_on, protection_option
    rcmd += protection_option(protection_on(rcmd, SHORT_CIRCUIT, cfg))
    rcmd += champion_options('router')
    if str(cfg.get('move', '0')) == '1':

        rcmd += ['move=1'] + (['move_policy=' + str(MOVE_MODEL)] if MOVE_MODEL.exists() else [])
    zr, cr = compute_run(rcmd, route_limit, os.path.join(c, 'route.log'))
    if str(cfg.get('move', '0')) == '1': moves_collect(c, A, cfg)
    z['router'] = dict(cur_state=zr, cpu_s=cr)
    if work_measurement and zr != 'ok':
        z.update(is_valid=None, cur_state='work_measurement_incomplete',
                 invalid_reason='Router error/CPU emergency stop is not a reproducible quality result')
        return z
    if not code_verify(z): return z
    if not os.path.exists(rj):
        z.update(is_valid=False, cur_state=zr if zr != 'ok' else 'router_without_export',
                 invalid_reason='Router without partial result'); return z
    if FR.active(cfg.get('early_abort')) and json.load(open(rj)).get('final') == 'early_abort':

        from postprocessing.short_circuit_protection import candidate_save
        reason = candidate_save(bj, rj)
        z.update(is_valid=None, cur_state='r44_unchecked' if reason else 'early_abort',
                 invalid_reason=reason or 'Prediction: no success within CPU budget (censored)')
        return z
    if os.path.exists(rj):
        from common.zones import board_geometry
        _rest_r = json.load(open(rj))
        if _rest_r.get('rest_board'):
            BA = board_geometry(BA, _rest_r)
            json.dump(BA, open(bj, 'w'))
    if cfg.get('zones_judge_diagnose'):
        rr = json.load(open(rj)); rr['options'] = dict(rr.get('options') or {}, zones_judge_diagnose='1')
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('zones_judge') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['options'] = dict(rr.get('options') or {}, zones_judge='1')
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_smoothed') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_smoothed'] = True
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_corridor') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_corridor'] = True
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_pad_groups') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_pad_groups'] = True
        Path(rj).write_text(json.dumps(rr))
    if str(cfg.get('r36_k17', '0')).lower() in ('1', 'true') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_k17'] = True
        Path(rj).write_text(json.dumps(rr))
    if str(cfg.get('k17_measure', '0')).lower() in ('1', 'true') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['options'] = dict(rr.get('options') or {}, k17_measure='1')
        Path(rj).write_text(json.dumps(rr))
    if not code_verify(z): return z
    if cfg.get('r39') and os.path.exists(rj):
        RR = json.load(open(rj)); RR['options'] = dict(RR.get('options') or {}, r39='1')
        json.dump(RR, open(rj, 'w'))
    route_gnd_layers = A['gnd_layers']
    if 'current_first=1' in rcmd and os.path.exists(rj):


        route_gnd_layers = json.load(open(rj))['options']['gnd_layers'].split(',')
        z['gnd_layers'] = route_gnd_layers
    from learning import shaping as FO
    try:
        if cpu is not None and cpu.rest < 1:
            z['shaping'] = dict(applied=False, reason='time_limit', cpu_s=0.0)
        else:
            z['shaping'] = FO.shapes(bj, os.path.join(c, 'netclasses.json'), rj, os.path.join(ad, 'design_rules.json'), route_gnd_layers, cfg.get('shaping', '45r'),
                                  log=os.path.join(c, 'shaping.log'), time_s=(emergency_stop if work_measurement else 120) if cpu is None else max(1, int(cpu.rest)), current_shape=bool(cfg.get('current_shape', False)), minimax=True if A.get('evaluation_target') == 'minimax' else None,
                                  r27_shape_gate=str(cfg.get('r27_shape_gate', '0')).lower() in ('1', 'true'),
                                  r18=str(cfg.get('r18', '0')).lower() in ('1', 'true'),
                                  r18_final=dict(cfg=cfg, mode=mode, r36=r36))
            if cpu: cpu.add(z['shaping'].get('cpu_s', 0.0))
            if work_measurement and z['shaping'].get('reason') in ('time_s', 'time_limit', 'wall_limit', 'faults'):
                z.update(is_valid=None, cur_state='shape_incomplete'); return z
        if cpu and cpu.rest < 1:
            z.update(is_valid=False, cur_state='time_limit'); return z
        r18_info = z.get('shaping', {}).get('r18') or {}
        if r18_info.get('completed'):
            z.update(r18_info['final'])
        else:
            try:
                z.update(FO.finalize(bj, os.path.join(c, 'netclasses.json'), rj,
                    os.path.join(ad, 'design_rules.json'), route_gnd_layers, cfg, mode, r36, cpu))
            except TimeoutError:
                z.update(is_valid=False, cur_state='time_limit'); return z
        if cfg.get('r19_post_run'):
            from learning.r19_post_run import apply as r19_apply
            z['r19_post_run'] = r19_apply(bj, os.path.join(c, 'netclasses.json'), rj,
                                            os.path.join(ad, 'design_rules.json'), route_gnd_layers,
                                            cpu_s=180 if cpu is None else max(1, int(cpu.rest)))
            if cpu: cpu.add(z['r19_post_run']['compute_slot']['cpu_s'])
        from postprocessing.short_circuit_protection import candidate_save_measurement
        r44_reason, z['r44'] = candidate_save_measurement(bj, rj)

        if r44_reason:
            z.update(is_valid=False, cur_state='r44_unchecked', invalid_reason=r44_reason[:300], s=round(time.time() - t0, 1))
            code_verify(z)
            return z
        from common.flow import NetGraphCache
        _graph_cache = NetGraphCache(); _routes = json.load(open(rj))
        g, E = FO.is_valid(BA, KA, rj, route_gnd_layers, flow_cache=_graph_cache, routes=_routes)
    except Exception as e:
        from analysis.areas import active
        RR = json.load(open(rj)) if os.path.exists(rj) else {}
        if not (active(RR, KA) or A.get('layer_stack')): raise
        z.update(is_valid=False, cur_state='evaluation_error', rule_error=repr(e)[:160],
                 invalid_reason='Area evaluation incomplete', s=round(time.time() - t0, 1))
        code_verify(z)
        return z
    from learning.zone_planner import acceptance_gate
    valid_routing = E.get('valid_routing', g) if E else g
    required = 'kicad_native' if mode == 'local' or r36 == 'kicad_native' else ('legacy' if r36 in ('legacy', 'geometric') else False)
    gate = acceptance_gate(dict(is_valid=g), z.get('current_zones'), required)
    g = gate['is_valid']
    if gate.get('invalid_reason'): z['invalid_reason'] = gate['invalid_reason']
    kol = (z.get('placer_metrics') or {}).get('collisions') or 0
    if kol: g = valid_routing = False; z['invalid_reason'] = 'Placement: %d collisions' % kol
    z['outcome'] = E; z['is_valid'] = g
    if A.get('evaluation_target') == 'minimax': z['profile_data'] = A['profile_data']
    if E:
        RR = {}
        try:
            if cpu and cpu.rest < 1:
                z.update(is_valid=False, cur_state='time_limit'); return z
            RR = _routes
            z['rule'] = S.rule_measurement(BA, RR, KA, route_gnd_layers, dense=True, flow_cache=_graph_cache, link_chain=bool(cfg.get('pair_quality'))); z['score_v3'] = z['rule']['score_v3']


            if 'gate_check' in z['rule']: z['gate_check'] = z['rule']['gate_check']
            from analysis.areas import active
            if (active(RR, KA) or A.get('layer_stack')) and (z['rule'].get('faults') or (z['rule'].get('design_rules') or {}).get('faults')):
                raise ValueError('Active area model: evaluation error')
            if evaluation_target(z) == 'minimax':
                z['valid_routing'] = valid_routing
                z['is_valid'] = z['is_valid'] and z['rule']['minimax_valid']
            z['evaluation_version'] = z['rule']['version']
        except Exception as e:
            z['rule_error'] = repr(e)[:160]
            from analysis.areas import active
            if active(RR, KA) or A.get('layer_stack'):
                z.update(is_valid=False, invalid_reason='Area evaluation incomplete')
        from learning.zones_judge import diagnose
        zones_rating = diagnose(BA, RR)
        z.update(zones_judge_score=zones_rating['zones_judge_score'], zones_judge=zones_rating)
        z['metrics'] = dict(vias=E.get('vias'), conflict_list=E.get('conflict_list'), open_items=E.get('open_items'), total_length=E.get('length_mm'),
                               bends=E.get('bends'), share_45=E.get('share_45'))
    z['cur_state'] = 'ok' if zr == 'ok' else zr; z['s'] = round(time.time() - t0, 1)
    acceptance_gate(z, z.get('current_zones'), required)
    if cpu and cpu.used > cpu.limit:z.update(is_valid=False,cur_state='time_limit',invalid_reason='Total budget exceeded')
    code_verify(z)
    return z


def attempt(ad, A, cfg, k, time_s, it, only_place=False, prepared=False, total_cpu=None, cpu_budget=None, candidates=3, candidates_auto=True):
    from learning.cpu_budget import Budget
    cpu = cpu_budget or (Budget(total_cpu) if total_cpu is not None else None)
    tc = time.thread_time(); z = {}
    try:
        z = _attempt(ad, A, cfg, k, time_s, it, only_place, prepared, cpu, z)
        if not only_place and z.get('cur_state') == 'ok' and z.get('cfg', {}).get('cost'):
            from learning.champion_gate import kicad_native as champion_kicad_native
            try:
                champion_kicad_native(Path(ad), A, z, cpu_budget=cpu)
                code_verify(z)
            except Exception as exc:
                z.update(is_valid=None,cur_state='native_learning_measurement_incomplete',faults=repr(exc))
    except Exception as e:
        z.update(is_valid=None if cfg.get('budget') == 'work' else False, cur_state='faults', faults=repr(e)[:300])
    from learning.learning_effect import complete
    c = os.path.join(ad, 'k%d' % k)
    def preparation_save():
        p = Path(c) / 'prepared.json'
        if only_place and not prepared and p.exists():


            cached = json.loads(p.read_text())
            for key in ('cpu_total', 'cpu_call_s', 'cpu_measurement', 'cpu_budget', 'cpu_time_limit'):
                if key in z: cached[key] = z[key]
            p.write_text(json.dumps({key: value for key, value in cached.items()
                                     if not key.startswith('_cpu_')}))
    if cpu: z = cpu.logbook(z)
    complete(z, c)
    if z.get('cfg', {}).get('cost'):
        z['cost_reward']=dict(version=ME.REWARD_VERSION,rank=ME.reward(z))
    if cpu:
        z.pop('_cpu_kind_s', None)
        z['cpu_measurement'] = 'partially'
        z.setdefault('s', 0.0)
    if cfg.get('bga_diagnose') in (True, '1') and not only_place and not z.get('is_valid') and any('BGA' in b for b in A['blocks']):
        try:
            from learning.bga_diagnose import off_files
            bp = Path(c, 'board.json')
            z['bga_diagnose'] = off_files(json.loads(bp.read_text()), Path(c, 'labels.npz')) if bp.exists() else dict(cur_state='unknown', reason='Placement missing')
        except Exception as e:
            z['bga_diagnose'] = dict(cur_state='unknown', reason=repr(e)[:200])
    if str(cfg.get('dataset', '1')) != '0':
        dataset_capture(ad, A, z, k, only_place, cpu, candidates, candidates_auto)
    if cpu:


        if z.get('dataset', {}).get('cur_state') != 'ok':
            before = bool(z.get('cpu_time_limit'))
            cpu.logbook(z)
            if before != bool(z.get('cpu_time_limit')):
                complete(z, c)
                cpu.logbook(z)
        z['cpu_call_s'] = z['cpu_total'] - z.pop('_cpu_preparation_s', 0.0)
        if z.get('cpu_preparation_unknown'): z['cpu_total'] = None
        preparation_save()
        if FR.aborted(z): z['is_valid'] = None
        return z


    own = time.thread_time() - tc
    children = z.pop('_cpu_kind_s', 0.0)
    children += (z.get('shaping') or {}).get('cpu_s', 0) or 0

    completed = ((z.get('shaping') or {}).get('r18') or {}).get('completed')
    extra = ('zone_plan',) if completed else ('zone_plan', 'current_zones', 'r11_ir', 'r24')
    children += sum((z.get(q) or {}).get('compute_slot', {}).get('cpu_s', 0) or 0 for q in extra)
    z['cpu_call_s'] = children + own
    z['cpu_total'] = z['cpu_call_s'] + z.pop('_cpu_preparation_s', 0.0)
    if z.get('cpu_preparation_unknown'): z['cpu_total'] = None
    z['cpu_measurement'] = 'partially'
    z.setdefault('s', 0.0)
    preparation_save()
    return z


def dataset_capture(ad, A, z, k, only_place=False, cpu=None, candidates=3, candidates_auto=True):
    _tc_dense = time.thread_time()
    try:
        from learning.ai import data as D
        z.setdefault('design_family', 'erf_%06d' % A['seed'])
        z.setdefault('gnd_layers', list(A['gnd_layers']))
        metrics = (z.get('rule') or {}).pop('_physics', None)
        c = os.path.join(ad, 'k%d' % k)

        if os.path.exists(os.path.join(c, 'board.json')):
            from learning.ai.features_raster import substitute_ground
            b = json.load(open(os.path.join(c, 'board.json'))); kk = json.load(open(os.path.join(c, 'netclasses.json')))
            m, cheap = substitute_ground(b, kk, gnd_layers=z.get('gnd_layers', A['gnd_layers']))
            z['substitute_ground'] = cheap
        else: m = None
        z['comparison_group'] = '%s:%s:%s' % (z['design_family'], hashlib.sha256(Path(ad, 'board.json').read_bytes()).hexdigest(), k)
        if cpu and not only_place and cpu.rest > 0:
            z['labels_candidates'] = D.candidates(c, z, os.path.join(ad, 'board.json'), candidates, candidates_auto, cpu_budget=cpu)
        z['dataset'] = D.capture(c, z, os.path.join(ad, 'board.json'), metrics=metrics, input_=m, cpu_budget=cpu)
        z['dataset']['cpu_s'] = time.thread_time() - _tc_dense
        if not only_place and cpu is None:
            z['labels_candidates'] = D.candidates(c, z, os.path.join(ad, 'board.json'), candidates, candidates_auto)
    except Exception as e: z['dataset'] = dict(cur_state='faults', faults=repr(e)[:200])
    return z


def level_read(kw):
    p = kw['data'][:-6] + '_level.json' if kw['data'].endswith('.jsonl') else kw['data'] + '_level.json'
    try: return p, json.load(open(p))
    except (OSError, ValueError): return p, dict(level=float(kw.get('level', '0')), window=[], history=[])


def level_report(kw, succeeded, seed, generator_failure=False):
    with LOCK:
        p, S = level_read(kw)
        if generator_failure: S['generator_failure'] = S.get('generator_failure', 0) + 1
        S['window'] = (S.get('window') or [])[-7:] + [bool(succeeded)]
        if len(S['window']) >= 8:
            n = sum(S['window']); prior = S['level']
            if n >= 6: S['level'] = round(prior + 0.5, 2)
            elif n <= 2: S['level'] = round(max(0.0, prior - 0.5), 2)
            if S['level'] != prior:
                S['window'] = []; S.setdefault('history', []).append(dict(time_s=time.strftime('%Y-%m-%d %H:%M'), task=seed, level=S['level'], from_=prior))
                print('Level %.1f -> %.1f (%d of 8 achieved)' % (prior, S['level'], n), flush=True)
        json.dump(S, open(p, 'w'), indent=1)


def difficulty(A):
    bga = int(any('BGA' in b for b in A['blocks'])); w = int(any(b.startswith(('Buck', 'Boost', 'Mini')) for b in A['blocks']))
    opv = int(any('OPV' in b for b in A['blocks'])); mcu = int(any(b.startswith('Processor') for b in A['blocks']))
    return dict(points=3 * w + 2 * mcu + 4 * bga + opv + 3 + (A['cu_layers'] - 2) / 2 + A.get('level', 0), converter=w, mcu=mcu, bga=bga, opv=opv,
                pads=A['pads'], cu_layers=A['cu_layers'], level=A.get('level', 0), invented=True)


def task_generate(ad, seed, goal=None, **kwargs):
    explicit = goal
    goal = default_target(explicit=goal, context='selfplay')
    a = AU.generate(ad, seed, **kwargs)
    if goal == 'minimax':
        a['profile_data'] = 'balanced'
        a['evaluation_target'] = goal
        p = os.path.join(ad, 'specs.json'); cons = json.load(open(p))
        cons.update(evaluation=goal, profile_data=a['profile_data']); json.dump(cons, open(p, 'w'), indent=1)
        Path(ad, 'task.json').write_text(json.dumps(a, indent=1, ensure_ascii=False))
    elif explicit == 'v3' and default_target() == 'minimax':

        a['evaluation_target'] = 'v3'
        p = os.path.join(ad, 'specs.json'); cons = json.load(open(p)); cons['evaluation'] = 'v3'
        json.dump(cons, open(p, 'w'), indent=1)
        json.dump(a, open(os.path.join(ad, 'task.json'), 'w'), indent=1, ensure_ascii=False)
    return a


def limit_factor(lines, n=40, surcharge=1.10, f_min=1.10, f_max=1.55):
    best, series, seen = {}, [], set()
    for z in lines:
        a, f = z.get('task'), z.get('factor')
        if a is None or not isinstance(f, (int, float)) or z.get('cur_state') == 'code_changed': continue
        if a not in seen: seen.add(a); series.append(a)
        if z.get('is_valid'): best[a] = min(best.get(a, 9.0), f)
    value_list = [best[a] for a in series if a in best][-n:]
    if len(value_list) < 8: return None
    value_list.sort(); med = value_list[len(value_list) // 2]
    return round(min(f_max, max(f_min, med * surcharge)), 3)


def task_play(out, seed, kw, lines, rnd):
    options_verify(kw)
    SHORT_CIRCUIT.clear(); SHORT_CIRCUIT.update({k: kw[k] for k in ('short_circuit_protection',) if k in kw})
    if kw.get('racetrack', '0') == '1':
        from learning.racetrack import play
        return play(out, seed, kw, lines, rnd)
    ZONE_PLAN.update(mode=kw.get('zone_plan','off'),r36=kw.get('r36','geometric'))
    if kw.get('teacher', '0') == '1': return teacher_play(out, seed, kw, lines, rnd)
    attempts = int(kw.get('attempts', '3')); vd = float(kw.get('densify', '0.93')); time_s = int(kw.get('time_s', '300'))
    it = int(kw.get('iter', '40')); gal = None if kw.get('gallery', '1') == '0' else os.path.expanduser('~/Library/Caches/Autorouter/gallery')
    ad = os.path.join(out, 'task_%06d' % seed)
    tempo = TT.active(kw.get('curriculum_tempo', '0'))
    version = kw.get('generator_version') or ('11' if kw.get('cost_learning', '1') != '0' and seed % 5 == 0 else '10')
    axis = None
    if kw.get('bga_axis', '0') == '1':
        from learning import bga_axes as AX
        if tempo:
            axis = AX.reserve(kw, seed, level_choice=lambda fallback, axis: TT.level(lines, fallback, rnd, version, axis))
        else: axis = AX.reserve(kw, seed)
    st = axis['level'] if axis else level_read(kw)[1]['level']
    overload_level = st
    tempo_cache = TT.Preparation()
    tempo_ledger = TT.error_book(kw.get('tempo_error_book', os.path.join(R, 'learning/data/error_book.json'))) if tempo else None
    if tempo and not axis:
        st = TT.level(lines, st, rnd, version, axis)

    def tempo_attempt(ad, A, cfg, kid, time_s, it, **opts):
        if not tempo: return attempt(ad, A, cfg, kid, time_s, it, **opts)
        return TT.attempt(tempo_cache, ad, A, cfg, kid, time_s, it, **opts)
    goal = default_target(explicit=kw.get('goal'), context='selfplay')
    if goal not in ('v3', 'minimax'): raise ValueError('goal must be v3 or minimax')
    new = []; k = 0; best = None; level = 0
    cpu_task = []
    locker_cfg = None
    def cost(z):
        cpu_task.append(z.get('cpu_call_s', z.get('cpu_total')))
    def axis_finalize(aborted=False):
        if not axis: return
        from learning.learning_effect import _limit_result
        succeeded = any(_limit_result(z) is True and z['factor'] <= factor0 + 1e-9 for z in new)
        unknown = not succeeded and (aborted or any(
            not z.get('generator_failure') and not z.get('not_routed') and _limit_result(z) is None for z in new))
        AX.report(kw, seed, None if unknown else succeeded,
                  cpu_s=sum(cpu_task) if cpu_task and all(isinstance(c, (int, float)) and math.isfinite(c) and c >= 0 for c in cpu_task) and not any(z.get('generator_failure') for z in new) else None,
                  valid_ones=sum(z.get('is_valid') is True for z in new),
                  generator_failure=bool(new and new[-1].get('generator_failure')), aborted=aborted)
    def generate(**kwargs):
        try:
            if kw.get('cost_learning', '1') != '0':
                kwargs.setdefault('generator_version', kw.get('generator_version') or ('11' if seed % 5 == 0 else '10'))
            a = task_generate(ad, seed, goal=goal,
                                **(dict(package=axis['package'], cu_layers=axis['cu_layers']) if axis else {}), anchor_legal=kw.get('anchor_legal', '0') == '1', potential=kw.get('potential') == '1', power_share=float(kw.get('power_share', '0')), **kwargs)
            if axis:
                a['learn_axis'] = axis
                json.dump(a, open(os.path.join(ad, 'task.json'), 'w'), indent=1, ensure_ascii=False)
            return a
        except AU.GeneratorError as e:



            z = dict(task=seed, candidate=None, design_family='erf_%06d' % seed,
                     net_class='generator', cfg={}, version=AU.VERSION + '+anchor-legal1',
                     generator_failure=True, cur_state='generator_error', is_valid=False,
                     invalid_reason=str(e), score_v3=None, amount=2.0, level=level,
                     level_task=st, factor=kwargs.get('factor'), not_routed=True,
                     rule=dict(evaluation_target=goal), time_s=time.strftime('%Y-%m-%d %H:%M:%S'))
            if axis: z.update(learn_axis=dict(axis), cu_layers=axis['cu_layers'], blocks=[])
            with LOCK:
                with open(kw['data'], 'a') as f: f.write(json.dumps(z) + '\n')
            new.append(z)
            print('Task %d generator error: %s' % (seed, e), flush=True)
            return None


    bounds_rows = lines
    if axis:
        bounds_rows = [z for z in lines if z.get('cu_layers') == axis['cu_layers'] and AX.category(z) == axis['package']]
    fg = limit_factor(bounds_rows) if kw.get('start_limit', '1') == '1' else None
    A = generate(level=st, **({'factor': fg} if fg else {}))
    if A is None:
        AX.report(kw, seed, False, generator_failure=True) if axis else level_report(kw, False, seed, generator_failure=True)
        if kw.get('keep') != '1': shutil.rmtree(ad, ignore_errors=True)
        return new
    if kw.get('learning_limit') == '1':
        from learning.learning_effect import limit_factor as learn_limit
        from learning.ai.data import family, split_family
        training = [z for z in lines if split_family(family(z)) == 'train']
        target = learn_limit(training, net_class(A, goal), A['factor'])
        if target != A['factor']:
            A = generate(level=st, factor=target)
            if A is None:
                if axis: AX.report(kw, seed, False, generator_failure=True)
                return new
    factor = A['factor']; factor0 = factor
    from learning.value_model import Service
    selection = kw.get('selection', 'all_items')
    if selection not in ('all_items', 'heuristic', 'amount', 'value_auto', 'routability'): raise ValueError('selection=all_items|heuristic|amount|value_auto|routability')
    wd = kw.get('_value_service') or Service([kw['data']], kw.get('value_model', os.path.join(R, 'learning/model/amount')),
                new=int(kw.get('value_new', '100')), cpu_s=int(kw.get('value_cpu', '300')), route=int(kw.get('value_route', '4')))
    if selection == 'routability':
        from learning.ai.routability_service import Service as RoutService
        wd = RoutService(kw.get('routability_model'), route=int(kw.get('value_route', '4')), seed=seed)
    while True:
        valid_here = False; unknown_here = False; relaxation_here = False
        queue = []; selection_info = None
        nroute = attempts if selection == 'all_items' else (max(1, int(kw.get('value_candidates', '16'))) if kw.get('value_shadow', '0') == '1' else min(max(1, int(kw.get('value_candidates', '16'))), wd.route + int(kw.get('value_explore', '1'))))
        for v in range(nroute):
            if os.path.exists(kw['stop']):
                axis_finalize(aborted=True)
                return new
            from analysis import corpus as KO
            KO.netclasses_run(ad, os.path.join(ad, 'board.json'), os.path.join(ad, 'netclasses.json'), os.path.join(ad, 'specs.json'))
            def learn_cfg(cfg):
                if kw.get('cost_learning', '1') != '0':
                    _, me = ME.selection(ME.read_in(kw.get('champion_cfg')), ME.net_class(A), exact=True)
                    start=me['parameter'] if me else dict(ME.DEFAULT,shaping=cfg.get('shaping','45r'),
                          placer_style=cfg.get('placer','prior'),router_style=cfg.get('router','prior'))
                    cfg['cost'] = ME.suggestion(lines + new, ME.net_class(A), rnd, start)
                    cfg['champion'] = '0'

                    learning_cost_early_abort(cfg,rnd)
                if tempo:
                    cfg['curriculum_tempo'] = '1'
                    if 'tempo_judge_model' in kw: cfg['tempo_judge_model'] = kw['tempo_judge_model']
                    tempo_cache.seed(cfg)
                if axis or kw.get('bga_diagnose') == '1': cfg['bga_diagnose'] = True
                if LOOSEN.active(kw.get('placer_loosen')): cfg['placer_loosen'] = '1'
                for name in ('learning_budget', 'learn_cpu_max', 'learning_policy', 'budget_before_post_run', 'learning_data', 'early_abort', 'early_abort_model'):
                    if name in kw: cfg[name] = kw[name]
                if kw.get('pair_quality', '0') == '1': cfg['pair_quality'] = True
                return cfg
            if selection in ('heuristic', 'amount', 'value_auto', 'routability'):
                if not queue:
                    if v > 0: break
                    pool = []
                    for _ in range(max(1, int(kw.get('value_candidates', '16')))):
                        if os.path.exists(kw['stop']):
                            axis_finalize(aborted=True)
                            return new
                        if locker_cfg is not None:
                            cfg = dict(locker_cfg); locker_cfg = None
                        else:
                            with LOCK: cfg = choice(lines + new, net_class(A, goal), rnd, goal=goal, bga_axis=bool(axis), style_per_net_kind=kw.get("style_per_net_kind") == "1", shape_arms=kw.get("shape_arms"), tempo_error_book=tempo_ledger, move_arm=kw.get("move") == "1")
                        learn_cfg(cfg)
                        if kw.get('r36_smoothed', '0') == '1': cfg['r36_smoothed'] = True
                        if kw.get('zones_judge', '0') == '1': cfg['zones_judge'] = True
                        if kw.get('zones_judge_diagnose', '0') == '1': cfg['zones_judge_diagnose'] = True
                        if kw.get('r36_corridor', '0') == '1': cfg['r36_corridor'] = True
                        if kw.get('r36_pad_groups', '0') == '1': cfg['r36_pad_groups'] = True
                        if kw.get('r36_k17', '0') == '1': cfg['r36_k17'] = True
                        if kw.get('k17_measure', '0') == '1': cfg['k17_measure'] = True
                        if kw.get('current_shape', '0') == '1': cfg['current_shape'] = True
                        if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
                        if kw.get('r19_post_run', '0') == '1': cfg['r19_post_run'] = True
                        if kw.get('k20_zones', '0') == '1': cfg['k20_zones'] = True
                        if kw.get('shaping'): cfg['shaping'] = kw['shaping']
                        if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['shaping'] = '45r'
                        if kw.get('legal_lp', '0') == '1': cfg['legal_lp'] = True
                        if kw.get('r39', '0') == '1': cfg['r39'] = True
                        if kw.get('r24', '0') == '1': cfg['r24'] = True
                        if kw.get('r39_repair', '0') == '1': cfg['r39_repair'] = True
                        cfg.update({key:kw[key] for key in ('remaining_conflicts','rest_cpu','rest_parts','rest_cand') if key in kw})
                        zp = tempo_attempt(ad, A, cfg, k, time_s, it, only_place=True)
                        cost(zp)
                        if zp.get('feats') and not zp.get('code_changed') and not zp.get('faults'):
                            pool.append(zp)
                        else:
                            zp.setdefault('is_valid', False)
                            zp['board'] = 'erf_%06d_f%.2f' % (seed, factor)
                            zp['value_preparation_failed'] = True
                            with LOCK:
                                with open(kw['data'], 'a') as f: f.write(json.dumps(zp) + '\n')
                            new.append(zp)
                        k += 1
                        if LOOSEN.request(zp):
                            relaxation_here = True; locker_cfg = dict(cfg)
                            break
                    if selection == 'routability':
                        for zp in pool:
                            zp['routability_input'] = os.path.join(ad, 'k%d' % zp['candidate'])
                            zp['routability_design_rules'] = os.path.join(ad, 'design_rules.json')
                    chosen, selection_info = wd.selection(pool, selection, explore=int(kw.get('value_explore', '1')))
                    selection_info['shadow'] = kw.get('value_shadow', '0') == '1'
                    if selection_info['shadow']: chosen = list(range(len(pool)))
                    selection_info['routed'] = chosen
                    ledger = dict(task=seed, factor=factor, level=level, pool=pool, selection=selection_info)
                    with LOCK:
                        with open(kw.get('value_selection_data', os.path.join(out, 'value_selection.jsonl')), 'a') as f: f.write(json.dumps(ledger) + '\n')
                    queue = [pool[i] for i in chosen]
                    for i, zp in enumerate(pool):
                        if i in chosen: continue


                        rejected = dict(zp, is_valid=None, cur_state='selection_rejected',
                            not_routed=True, design_family='erf_%06d' % seed,
                            board='erf_%06d_f%.2f' % (seed, factor), level=level,
                            cpu_total=zp.get('cpu_call_s', zp.get('cpu_total')))
                        rejected.pop('feats', None)
                        if axis: rejected['learn_axis'] = dict(axis)
                        with LOCK:
                            with open(kw['data'], 'a') as f: f.write(json.dumps(rejected) + '\n')
                        new.append(rejected)
                    if kw.get('keep') != '1':
                        for i, zp in enumerate(pool):
                            if i not in chosen: shutil.rmtree(os.path.join(ad, 'k%d' % zp['candidate']), ignore_errors=True)
                    if not queue: break
                zp = queue.pop(0); cfg = zp['cfg']; kid = zp['candidate']
                if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['shaping'] = '45r'
                if kw.get('r27_shape_gate', '0') == '1' or kw.get('r18', '0') == '1':
                    if kw.get('r27_shape_gate', '0') == '1': cfg['r27_shape_gate'] = True
                    p = Path(ad)/('k%d' % kid)/'prepared.json'
                    prepared = json.loads(p.read_text()); prepared['cfg'] = cfg
                    p.write_text(json.dumps(prepared))
                z = tempo_attempt(ad, A, cfg, kid, time_s, it, prepared=True, candidates=int(kw.get('candidates', '3')), candidates_auto='candidates' not in kw)
                cost(z)
                z['value_selection'] = selection_info
            else:
                if locker_cfg is not None:
                    cfg = dict(locker_cfg); locker_cfg = None
                else:
                    with LOCK: cfg = choice(lines + new, net_class(A, goal), rnd, goal=goal, bga_axis=bool(axis), style_per_net_kind=kw.get("style_per_net_kind") == "1", shape_arms=kw.get("shape_arms"), tempo_error_book=tempo_ledger, move_arm=kw.get("move") == "1")
                learn_cfg(cfg)
                if kw.get('r36_smoothed', '0') == '1': cfg['r36_smoothed'] = True
                if kw.get('zones_judge', '0') == '1': cfg['zones_judge'] = True
                if kw.get('zones_judge_diagnose', '0') == '1': cfg['zones_judge_diagnose'] = True
                if kw.get('r36_corridor', '0') == '1': cfg['r36_corridor'] = True
                if kw.get('r36_pad_groups', '0') == '1': cfg['r36_pad_groups'] = True
                if kw.get('r36_k17', '0') == '1': cfg['r36_k17'] = True
                if kw.get('k17_measure', '0') == '1': cfg['k17_measure'] = True
                if kw.get('current_shape', '0') == '1': cfg['current_shape'] = True
                if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
                if kw.get('r19_post_run', '0') == '1': cfg['r19_post_run'] = True
                if kw.get('k20_zones', '0') == '1': cfg['k20_zones'] = True
                if kw.get('shaping'): cfg['shaping'] = kw['shaping']
                if kw.get('legal_lp', '0') == '1': cfg['legal_lp'] = True
                if kw.get('r39', '0') == '1': cfg['r39'] = True
                if kw.get('r24', '0') == '1': cfg['r24'] = True
                cfg.update({key:kw[key] for key in ('remaining_conflicts','rest_cpu','rest_parts','rest_cand') if key in kw})
                if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['shaping'] = '45r'
                if kw.get('r27_shape_gate', '0') == '1': cfg['r27_shape_gate'] = True
                if kw.get('r39_repair', '0') == '1': cfg['r39_repair'] = True
                kid = k; k += 1
                z = tempo_attempt(ad, A, cfg, kid, time_s, it, candidates=int(kw.get('candidates', '3')), candidates_auto='candidates' not in kw)
                cost(z)
            if kw.get('anchor_legal', '0') == '1': z['version'] = A['version']
            if axis: z['learn_axis'] = dict(axis)
            z['level'] = level; z['design_family'] = 'erf_%06d' % seed
            z['board'] = 'erf_%06d_f%.2f' % (seed, factor)
            z['pair_active'] = kw.get('judge_pair', '1') == '1'
            z['pair_quality'] = kw.get('pair_quality', '0') == '1'
            if kw.get('pair_model'): z['pair_model_path'] = kw['pair_model']
            if z['pair_active']:
                pd = pair_service(z.get('pair_model_path')); z['pair_model_path'] = str(pd.model); pd.apply(z)
                if best is not None and best.get('factor') == z.get('factor'):
                    from learning.judge_pair import reference_line
                    z['pair_reference'] = reference_line(best.get('row') or best)
            if better_than(dict(z, area_mm2=A['area_mm2']), best):
                best = dict(z, area_mm2=A['area_mm2'], candidate=kid, row=copy.deepcopy(z))
            if goal == 'minimax': z['reward'] = reward(z)
            z['amount'] = 2.0 if not z['is_valid'] else round(A['area_mm2'] / (best['area_mm2'] if best else A['area_mm2']), 4)
            if goal == 'minimax' and z['is_valid'] is None: z['amount'] = None
            z['board'] = 'erf_%06d_f%.2f' % (seed, factor); z['difficulty'] = difficulty(A)
            if gal:
                try:
                    from learning import gallery as G
                    with LOCK: G.capture(os.path.join(ad, 'k%d' % kid), dict(z, duration=dict(source='selfplay %s' % cfg.get('shaping', '')), amount=None), gal)
                except Exception as e: print('  Gallery:', repr(e)[:120], flush=True)
            with LOCK:
                with open(kw['data'], 'a') as f: f.write(json.dumps(z) + '\n')
            new.append(z)
            if kw.get('value_train', '0') == '1': wd.verify()
            print('  Task %d k%d %s factor %.2f %5.0f mm2 %s score %s  %4.0fs  %s' % (seed, kid, z['net_class'], factor, A['area_mm2'],
                  'unknown' if z['is_valid'] is None else 'valid ' if z['is_valid'] else 'invalid', z.get('score_v3'), z['s'],
                  '' if z['is_valid'] else 'confl %s open %s' % ((z.get('outcome') or {}).get('conflict_list'), (z.get('outcome') or {}).get('open_items'))), flush=True)
            if not kw.get('keep') == '1' and not z['is_valid']: shutil.rmtree(os.path.join(ad, 'k%d' % kid), ignore_errors=True)
            if LOOSEN.request(z):
                relaxation_here = True; locker_cfg = dict(cfg)
                break
            if z['is_valid'] is None: unknown_here = True
            if z['is_valid']:
                if valid_here and selection == 'all_items': break
                valid_here = True
        if relaxation_here:


            if best is None and level <= -2: break
            new_factor = LOOSEN.next_factor(A, new)
            if new_factor is None: break
            next_ = generate(factor=new_factor, level=st)
            if next_ is None: break
            A = LOOSEN.accept(A, next_); factor = A['factor']; level -= 1
            Path(ad, 'task.json').write_text(json.dumps(A, indent=1, ensure_ascii=False))
            continue
        if LOOSEN.active(kw.get('placer_loosen')) and A.get('relaxed') and valid_here: break
        if not valid_here:
            if unknown_here: break
            if best is None and level > -2:
                factor = round(factor * 1.12, 3); level -= 1
                next_ = generate(factor=factor, level=st)
                if next_ is None: break
                if LOOSEN.active(kw.get('placer_loosen')) and A.get('relaxed'):
                    next_ = LOOSEN.accept(A, next_)
                A = next_; continue
            break
        factor = round(factor * vd, 3); level = max(level, 0) + 1
        if factor < 1.02: break
        next_ = generate(factor=factor, level=st)
        if next_ is None: break
        A = next_
    succeeded = any(z['is_valid'] and z['factor'] <= factor0 + 1e-9 for z in new)
    if axis:
        axis_finalize()
    elif new and new[-1].get('generator_failure') and (not tempo or st == overload_level): level_report(kw, succeeded, seed, generator_failure=True)
    elif (not tempo or st == overload_level) and (any(z.get('is_valid') is True for z in new) or not any(z.get('is_valid') is None for z in new if not z.get('not_routed'))):
        level_report(kw, succeeded, seed)
    print('Task %d finished: %s' % (seed, 'smallest valid %.0f mm2 (factor %.2f)' % (best['area_mm2'], best['area_mm2'] / A['courtyard_mm2'])
                                     if best else 'never valid'), flush=True)
    if kw.get('keep') != '1' and not best: shutil.rmtree(ad, ignore_errors=True)
    return new


def teacher_play(out, seed, kw, lines, rnd):
    from learning.teacher import Teacher, context, random_recipe
    time_s = int(kw.get('time_s', '300')); it = int(kw.get('iter', '40'))
    budget = float(kw.get('teacher_cpu', str(time_s)))
    from learning.cpu_budget import Budget
    cpu = Budget(budget)
    calibration = kw.get('teacher_calibration', '0') == '1'
    with LOCK:
        if '_teacher' not in kw:
            kw['_teacher'] = Teacher(kw.get('teacher_folder', os.path.join(out, 'teacher')), context(budget, it), seed)
        teacher = kw['_teacher']
        recipe = (dict(random_recipe(rnd, category=('radio', 'analog')[seed % 2], seed=seed), category='calibration')
                  if calibration else teacher.suggestion())
    ad = os.path.join(out, 'teacher_%06d' % seed)
    args = {k: recipe[k] for k in ('seed', 'blocks', 'level', 'axes', 'family')}
    try:
        A = task_generate(ad, goal=default_target(explicit=kw.get('goal'), context='selfplay'), **args)
    except ValueError as e:

        teacher.reject(recipe, str(e))
        print('Teacher rejects proposal:', str(e), flush=True); return []
    new = []
    for k in range(int(kw.get('attempts', '3'))):
        if os.path.exists(kw['stop']): break
        if k: cpu = Budget(budget)
        goal = default_target(explicit=kw.get('goal'), context='selfplay')
        with LOCK: cfg = choice(lines if calibration else lines + new, net_class(A, goal), rnd, goal=goal, style_per_net_kind=kw.get("style_per_net_kind") == "1", shape_arms=kw.get("shape_arms"), move_arm=kw.get("move") == "1")
        if kw.get('zones_judge', '0') == '1': cfg['zones_judge'] = True
        if kw.get('zones_judge_diagnose', '0') == '1': cfg['zones_judge_diagnose'] = True
        if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
        if kw.get('r19_post_run', '0') == '1': cfg['r19_post_run'] = True
        if kw.get('k20_zones', '0') == '1': cfg['k20_zones'] = True
        if kw.get('shaping'): cfg['shaping'] = kw['shaping']
        if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['shaping'] = '45r'
        if kw.get('r39', '0') == '1': cfg['r39'] = True
        if kw.get('r24', '0') == '1': cfg['r24'] = True
        for name in ('early_abort', 'early_abort_model'):
            if name in kw: cfg[name] = kw[name]
        if kw.get('r39_repair', '0') == '1': cfg['r39_repair'] = True
        cfg.update({key:kw[key] for key in ('remaining_conflicts','rest_cpu','rest_parts','rest_cand') if key in kw})
        z = attempt(ad, A, cfg, k, time_s, it, cpu_budget=cpu)
        z['rule'] = dict(z.get('rule') or {}, evaluation_target=goal)
        if goal == 'minimax': z['rule']['minimax_profile'] = A['profile_data']
        z.update(design_family=A['family'], lineage=A['lineage'], axes=A['axes'])
        report = teacher.calibrate if calibration else teacher.report
        z = report(recipe, ad, A, z, os.path.join(ad, 'k%d' % k), cpu_budget=cpu)
        data = os.path.join(out, 'teacher_calibration.jsonl') if calibration else kw['data']
        with LOCK:
            with open(data, 'a') as f: f.write(json.dumps(z) + '\n')
        new.append(z)
    return new


def options_verify(kw):
    shape_arms_read(kw.get("shape_arms"))
    if TT.active(kw.get('curriculum_tempo', '0')) and (kw.get('teacher') == '1' or kw.get('racetrack') == '1'):
        raise ValueError('curriculum_tempo=1 needs normal generator selfplay')
    from learning.zones_judge_selection import an as judge_on
    judge_on({},kw.get('zones_judge_diagnose',0))
    if judge_on({},kw.get('zones_judge',0)) and (kw.get('r36','geometric')!='kicad_native' or kw.get('zone_plan','off')!='off' or kw.get('k20_zones','0')=='1'):
        raise ValueError('zones_judge=1 requires r36=kicad_native, zone_plan=off, k20_zones=0')
    if LOOSEN.active(kw.get('placer_loosen')) and (kw.get('teacher') == '1' or kw.get('racetrack') == '1'):
        raise ValueError('placer_loosen=1 needs normal generator selfplay')
    if kw.get('bga_axis') == '1' and (kw.get('teacher') == '1' or float(kw.get('power_share', '0'))):
        raise ValueError('bga_axis=1: normal selfplay without teacher/power boards')
    if kw.get('selection')=='routability' and kw.get('value_train','0')=='1':
        raise ValueError('Routability: routability_train=1 instead of value_train')


def frozen_duration(out, args, kw, clock=300, keep=3):
    import glob, subprocess
    basis = os.path.expanduser('~/Library/Caches/Autorouter/selfplay_code'); os.makedirs(out, exist_ok=True)
    stop = kw.get('stop', os.path.join(R, 'learning/STOP')); data = os.path.abspath(kw.get('data', os.path.join(R, 'learning/data/selfplay.jsonl')))
    rest = [a for a in args if a.split('=', 1)[0] not in ('frozen', 'stop', 'data', 'seed0', 'value_model')]
    children = []

    def head():
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True).stdout.strip()

    def start(sha):
        code = os.path.join(basis, sha[:12])
        if not os.path.exists(os.path.join(code, 'learning', 'selfplay.py')):
            os.makedirs(code, exist_ok=True)
            archive = subprocess.check_output(['git', '-C', R, 'archive', sha])
            subprocess.run(['tar', '-x', '-C', code], input=archive, check=True)
            if not os.path.exists(os.path.join(code, '.venv')): os.symlink(os.path.join(R, '.venv'), os.path.join(code, '.venv'))
        nrn = [int(d.split('_')[1]) for d in os.listdir(out) if d.startswith('task_') and d.split('_')[1].isdigit()] if os.path.exists(out) else []
        zs = [json.loads(l).get('task') or 0 for l in open(data)] if os.path.exists(data) else []
        seed0 = max(nrn + zs + [0]) + int(kw.get('parallel', '4')) + 2
        gstop = os.path.join(code, 'STOP_generation')
        if os.path.exists(gstop): os.remove(gstop)
        cmd = [os.path.join(R, '.venv', 'bin', 'python'), os.path.join(code, 'learning', 'selfplay.py'), out] + rest + [
            'data=' + data, 'stop=' + gstop, 'seed0=%d' % seed0, 'value_model=' + os.path.join(R, 'learning/model/amount')]
        log = open(os.path.join(out, 'generation_%s.log' % sha[:12]), 'a')
        p = subprocess.Popen(cmd, cwd=code, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONHASHSEED='0',
                             AUTOROUTER_CHAMPION=kw.get('champion_cfg') or str(ME.file())))
        print('%s generation %s started (pid %d, from task %d)' % (time.strftime('%d.%m. %H:%M'), sha[:12], p.pid, seed0), flush=True)
        children.append((p, gstop, code))

    current = head(); start(current); last_cleaned_up = 0.0
    while True:
        if time.time() - last_cleaned_up > 3600:
            subprocess.Popen(['nice', '-n', '15', os.path.join(R, '.venv', 'bin', 'python'), os.path.join(R, 'common', 'tidy_up.py')],
                             cwd=R, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); last_cleaned_up = time.time()
            if kw.get('move', '0') == '1' and MOVE_DATA.exists():
                subprocess.Popen(['nice', '-n', '15', os.path.join(R, '.venv', 'bin', 'python'), os.path.join(R, 'learning', 'move_choice.py'),
                                  str(MOVE_DATA), str(MOVE_MODEL)], cwd=R, stdout=open(str(MOVE_MODEL) + '.log', 'a'),
                                 stderr=subprocess.STDOUT)
        t0 = time.time()
        while time.time() - t0 < clock and not os.path.exists(stop) and children[-1][0].poll() is None: time.sleep(10)
        if os.path.exists(stop):
            for p, gs, _ in children: open(gs, 'w').close()
            for p, _, _ in children: p.wait()
            print('Selfplay (frozen) ended.', flush=True); return
        new = head()
        if children[-1][0].poll() is not None and new == current and kw.get('tasks'):
            for p, _, _ in children: p.wait()
            print('Selfplay (frozen) finished.', flush=True); return
        if new != current or children[-1][0].poll() is not None:
            open(children[-1][1], 'w').close()
            time.sleep(2); current = new; start(new)
        children[:] = [k for k in children[:-1] if k[0].poll() is None] + children[-1:]
        alive = {k[2] for k in children}
        prior = sorted((d for d in glob.glob(os.path.join(basis, '*')) if d not in alive), key=os.path.getmtime)[:-keep]
        for d in prior: shutil.rmtree(d, ignore_errors=True)


def main():
    out = sys.argv[1]; kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    shape_arms_read(kw.get('shape_arms'))
    if kw.get('frozen') == '1': return frozen_duration(out, sys.argv[2:], kw)
    options_verify(kw)
    kw.setdefault('data', os.path.join(R, 'learning/data/selfplay.jsonl')); kw.setdefault('stop', os.path.join(R, 'learning/STOP'))
    if kw.get('teacher', '0') == '1' and 'data' not in dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a):
        kw['data'] = os.path.join(out, 'teacher_attempts.jsonl')
    os.makedirs(out, exist_ok=True); os.makedirs(os.path.dirname(kw['data']), exist_ok=True)
    lines = [json.loads(l) for l in open(kw['data'])] if os.path.exists(kw['data']) else []
    seed0 = int(kw.get('seed0', str(max([z['task'] for z in lines] or [0]) + 1)))
    if kw.get('bga_axis') == '1' and 'seed0' not in kw:
        from learning import bga_axes as AX
        with AX.cur_state(kw) as state:
            seed0 = max(seed0, max([int(s) for s in state['tasks']] + [state.get('archive_until', -1)]) + 1)
    n = int(kw.get('tasks', '0')); par = int(kw.get('parallel', '4'))
    print('Selfplay: from task %d, %s tasks, %d parallel, %d old rows' % (seed0, n or 'endless', par, len(lines)), flush=True)
    from learning.value_model import Service
    kw['_value_service'] = Service([kw['data']], kw.get('value_model', os.path.join(R, 'learning/model/amount')),
                                new=int(kw.get('value_new', '100')), cpu_s=int(kw.get('value_cpu', '300')), route=int(kw.get('value_route', '4')))
    counter = [seed0]

    def next_():
        with LOCK:
            s = counter[0]; counter[0] += 1; return s

    def worker(_):
        while not os.path.exists(kw['stop']):
            s = next_()
            if n and s >= seed0 + n: return
            try:
                new = task_play(out, s, kw, lines, random.Random(s))
                with LOCK:
                    lines.extend(z for z in new if z.get('recipe', {}).get('category') != 'calibration')
            except Exception as e: print('Task %d error: %r' % (s, e), flush=True)
    with ThreadPoolExecutor(par) as ex: list(ex.map(worker, range(par)))
    print('Selfplay ended.', flush=True)


if __name__ == '__main__':
    main()
