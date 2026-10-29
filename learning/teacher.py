import copy
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import threading

VERSION = 'teacher-v2'
CONTRACT = 'minimax-complete-v1'
FILES = ('board.json', 'design_rules.json', 'specs.json')
LIMITS = dict(factor=(1.05, 3.0), part_list=(0, 400), active_pins=(0.05, 0.95), current_paths=(0, 12),
               ampere=(0.1, 15.0), cu_layers=(4, 8), pitch=(0.4, 1.0), bga=(0, 1), clock=(0.25, 2.0),
               pairs=(0, 8), length_groups=(0, 8), analog=(0, 0.6))
STEPS = dict(factor=-0.04, part_list=2, active_pins=0.025, current_paths=1, ampere=0.05,
                cu_layers=-2, pitch=-0.05, bga=1, clock=0.05, pairs=1, length_groups=1, analog=0.025)

FAMILIES = dict(mcu_board='train', power_supply='train', radio='calibration', analog='calibration', audio='test')
FEATURES = tuple(LIMITS) + ('area_mm2', 'courtyard_mm2', 'parts_actual', 'pads', 'net_list', 'active_actual',
                             'current_paths_actual', 'current_A_max', 'analog_actual', 'pitch_actual', 'bga_actual',
                             'clock_Hz', 'pairs_actual', 'groups_actual')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def identity(d):
    return hashlib.sha256(b''.join(Path(d, f).read_bytes() for f in FILES)).hexdigest()


def family_id(category):
    if category not in FAMILIES: raise ValueError('Teacher needs a predetermined block family, no auto/seed split')
    return 'generator-v8:' + category


def random_recipe(rnd, category=None, seed=None):
    category = category or rnd.choice([k for k, v in FAMILIES.items() if v == 'train'])
    return dict(seed=seed if seed is not None else rnd.randrange(100000, 800000), blocks=category, level=0., family=family_id(category),
                axes=dict(factor=rnd.choice([1.4, 1.6, 1.8, 2.0]), cu_layers=rnd.choice([4, 6, 8]),
                            active_pins=rnd.choice([.3, .4, .5]), ampere=rnd.choice([.1, .3, .5]),
                            part_list=rnd.choice([0, 60, 100]), current_paths=rnd.choice([0, 4, 5]), clock=rnd.choice([.5, 1.0]),
                            pairs=rnd.choice([0, 0, 1]), length_groups=rnd.choice([0, 0, 1]), analog=rnd.choice([0., .15, .25])))


def role(family):
    return FAMILIES.get(family.removeprefix('generator-v8:'), 'unknown')


def axes_verify(a):
    for k, v in a.items():
        if k not in LIMITS: raise ValueError('Unknown axis: ' + k)
        lo, hi = LIMITS[k]
        if not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('Axis %s outside [%s,%s]' % (k, lo, hi))
        if k in ('part_list', 'current_paths', 'pairs', 'length_groups', 'bga') and int(v) != v:
            raise ValueError('Integer expected: ' + k)
    if a.get('cu_layers', 4) not in (4, 6, 8): raise ValueError('K19: only 4/6/8 layers')


def task_features(A, B):
    ps = [p for p in B['pads'] if p.get('net')]
    own_set = A['nets_mine']
    pitch = []; mcu_pitch = None
    import re
    for f in B['fps']:
        m = re.search(r'_P([\d.]+)mm', f['fp'])
        if m:
            pitch.append(float(m[1]))
            if f['ref'] == A.get('mcu_ref'): mcu_pitch = float(m[1])
    return dict(area_mm2=A['area_mm2'], courtyard_mm2=A['courtyard_mm2'], parts_actual=len(B['fps']),
                pads=len(B['pads']), net_list=len(B['nets']), active_actual=sum(p.get('pintype') in ('input', 'output', 'bidirectional') for p in ps),
                current_paths_actual=len(A['rails_A']), current_A_max=max(A['rails_A'].values(), default=0),
                analog_actual=sum(e.get('category') == 'analog' for e in own_set.values()) / max(1, len(own_set)),
                pitch_actual=min(pitch, default=1.0), mcu_pitch_actual=mcu_pitch, bga_actual=int(any('BGA' in f['fp'] for f in B['fps'])),
                clock_Hz=max((e.get('f_Hz', 0) for e in own_set.values()), default=0),
                pairs_actual=sum('z_diff_ohm' in e for e in own_set.values()) / 2, groups_actual=len(A.get('length_groups', {})))


def plausible(A, B):
    faults = []
    from shapely.geometry import Polygon
    fl = sum(Polygon(p).area for p in B['outline'])
    crt = sum(Polygon(p).area for c in B['courtyards'] for p in c['polys'])
    if crt > fl + 1e-8: faults.append('Courtyard total > area')
    L = A['cu_layers']; gnd = set(A['gnd_layers'])
    allowed = {'In%d' % i for i in range(1, L - 1)}
    if not gnd <= allowed or len(gnd) < {4: 1, 6: 2, 8: 3}.get(L, 99): faults.append('K19: GND layers missing')
    by = {}
    for p in B['pads']: by.setdefault(p['net'], []).append(p)
    fps = {f['ref']: f for f in B['fps']}
    from analysis.netclasses import REGULATOR_A, table
    for n, amp in A['rails_A'].items():
        pp = by.get(n, [])
        source_list = [p for p in pp if p.get('pintype') == 'power_out' or p['ref'].startswith(('L', 'D'))]
        load_list = [p for p in pp if p.get('pintype') == 'power_in' and p['ref'].startswith(('U', 'J'))]
        if not source_list or not load_list: faults.append(n + ': source/load missing')

        for p in source_list:
            if not p['ref'].startswith('L'): continue
            sw = {q['net'] for q in B['pads'] if q['ref'] == p['ref'] and q['net'] != n}
            regs = {q['ref'] for q in B['pads'] if q['net'] in sw and q['ref'].startswith('U')}
            caps = [table(REGULATOR_A, fps[r]['value']) for r in regs]
            if caps and all(c is not None and c + 1e-6 < amp for c in caps): faults.append(n + ': regulator rated current too small')
        if not math.isfinite(amp) or amp <= 0: faults.append(n + ': current implausible')
    return faults


def context(cpu_s, it, solver=None):
    from learning.selfplay import code_hashes
    return dict(solver=solver or code_hashes()['code_sha256'], cpu_s=float(cpu_s), iter=int(it), contract_spec=CONTRACT)


def context_id(c):
    return hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()


def censored(z):
    from learning.early_abort import aborted
    return bool(z.get('cpu_time_limit') or z.get('code_changed')
                or aborted(z)
                or (z.get('learning_data') or {}).get('censored')
                or any((z.get(k) or {}).get('cur_state') in ('time_limit', 'wall_limit', 'faults', 'early_abort')
                       for k in ('placer', 'router', 'netclasses_input', 'netclasses_placement')))


def labels(z):
    if z.get('code_changed') or z.get('teacher_context_deviation'):
        return dict(placement=None, routing=None, is_valid=None)
    kol = (z.get('placer_metrics') or {}).get('collisions')
    if kol is not None: pl = float(kol == 0)
    else: pl = None
    if (z.get('teacher_checking') or {}).get('phase') == 'placement': pl = 0.0
    routing = None
    if pl == 1 and z.get('router', {}).get('cur_state') == 'ok' and z.get('outcome') is not None:
        e = z['outcome']
        if all(isinstance(e.get(k), (int, float)) for k in ('conflict_list', 'open_items', 'not_connected')):
            routing = float(e['conflict_list'] == 0 and e['open_items'] == 0 and e['not_connected'] == 0)
    if censored(z): routing = None
    status = None if censored(z) else z.get('teacher_status')
    return dict(placement=pl, routing=routing, is_valid=1.0 if status == 'fulfilled' else 0.0 if status == 'violated' else None)


def verify(d, A, z):
    kol = (z.get('placer_metrics') or {}).get('collisions')
    if kol is not None and kol > 0: return dict(status='violated', phase='placement', reason='PlacementCollision')
    if kol != 0: return dict(status='unknown', reason='Collision measurement missing')
    for f in ('board.json', 'netclasses.json', 'routes.json'):
        if not Path(d, f).exists(): return dict(status='unknown', reason=f + ' missing')
    from analysis.evaluation import rate
    B, K, rr = [json.loads(Path(d, f).read_text()) for f in ('board.json', 'netclasses.json', 'routes.json')]
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    contour = unary_union([Polygon(p) for p in B['outline']])
    crts = [(c['ref'], c.get('side', 'F'), unary_union([Polygon(p) for p in c['polys']])) for c in B['courtyards']]
    for i, (ref, side, g) in enumerate(crts):
        if not contour.buffer(1e-5).covers(g): return dict(status='violated', phase='placement', reason=ref + ': outside outline')
        if any(side == s and g.distance(h) < .00485 for _, s, h in crts[:i]):
            return dict(status='violated', phase='placement', reason=ref + ': courtyard spacing violated')
        for ko in B.get('keepouts', []):
            if ko.get('ref') != ref and ko.get('prohibitions', {}).get('part_list') and any(g.intersects(Polygon(p)) for p in ko['polys']):
                return dict(status='violated', phase='placement', reason=ref + ': component keepout violated')
    rr['gnd_layers'] = A['gnd_layers']
    E = rate(B, rr, K, v3=True, minimax=True)
    status = E['minimax_status']

    fields = ('minimax', 'minimax_version', 'minimax_profile', 'minimax_rank', 'minimax_total_sum',
              'minimax_current_paths', 'minimax_complete', 'minimax_sources', 'minimax_valid',
              'minimax_status', 'chokepoint', 'safety', 'evaluation_target')
    net_list = (E.get('measures') or {}).get('net_list') or {}
    current_x = max([max(v['x'].get('dc', 0), v['x'].get('j', 0)) for v in net_list.values()
                   if v.get('net_class') in ('current_val', 'supply', 'sw')] or [0.0])
    rule = dict(z.get('rule') or {}, **{k: E[k] for k in fields}, design_rules=E['design_rules'],
                 score_v3=E.get('score_v3_nets'), current_x_max=current_x, is_valid=E['is_valid'])

    rule['evaluation_target'] = A.get('evaluation_target', 'v3')
    return dict(status={'is_valid': 'fulfilled', 'invalid': 'violated', 'unknown': 'unknown'}[status],
                score_v3=E.get('score_v3_nets'), rule=E['design_rules'], student_rule=rule,
                safety=E['safety'], contract_spec=CONTRACT)


def vector(A):
    v = dict(A.get('axes') or {}, **(A.get('features_task') or {}))
    return [math.log1p(max(0.0, float(v.get(k, 0)))) for k in FEATURES]

def reliability(y, p):
    if not y: return dict(n=0, brier=None, curve=[])
    curve = []
    for i in range(5):
        ix = [j for j, q in enumerate(p) if i / 5 <= q < (i + 1) / 5 or (i == 4 and q == 1)]
        curve.append(dict(from_=i / 5, until=(i + 1) / 5, n=len(ix), forecast=sum(p[j] for j in ix) / len(ix) if ix else None,
                          success=sum(y[j] for j in ix) / len(ix) if ix else None))
    return dict(n=len(y), brier=sum((a-b)**2 for a, b in zip(y, p)) / len(y), curve=curve)


class Model:
    def __init__(self):
        self.heads = {}

    def fit(self, lines):
        import numpy as np
        from scipy.optimize import minimize
        from scipy.special import expit
        self.heads = {}
        for ctx in sorted({context_id(z['context']) for z in lines}):
            for goal in ('placement', 'routing', 'is_valid'):
                ds = [z for z in lines if context_id(z['context']) == ctx and labels(z)[goal] is not None]
                tr = [z for z in ds if role(z['A']['family']) == 'train']
                ca = [z for z in ds if role(z['A']['family']) == 'calibration']
                y = np.array([labels(z)[goal] for z in tr]); prior = (sum(y) + 1) / (len(y) + 2)
                h = dict(n=len(tr), n_calibration=len(ca), prior=float(prior), calibrated=False)
                if len(tr) >= 8 and len(set(y)) == 2:
                    x = np.array([vector(z['A']) for z in tr]); mu = x.mean(0); sd = np.maximum(x.std(0), 0.1)
                    x = np.column_stack((np.ones(len(x)), (x-mu)/sd))
                    def loss(w):
                        t = x @ w
                        return float(np.logaddexp(0, t).sum() - y @ t + 0.5 * np.dot(w[1:], w[1:])), x.T @ (expit(t)-y) + np.r_[0, w[1:]]
                    w = minimize(loss, np.zeros(x.shape[1]), jac=True, method='L-BFGS-B').x
                    h.update(mu=mu.tolist(), sd=sd.tolist(), w=w.tolist(), calib=[1.0, 0.0])
                    if len(ca) >= 8 and len({labels(z)[goal] for z in ca}) == 2:
                        xc = (np.array([vector(z['A']) for z in ca])-mu)/sd
                        yc = np.array([labels(z)[goal] for z in ca]); t = w[0] + xc @ w[1:]
                        def closs(ab):
                            s = ab[0]*t+ab[1]
                            return float(np.logaddexp(0, s).sum()-yc @ s + .1 * np.dot(ab, ab))
                        ab = minimize(closs, [1., 0.], bounds=[(0, 10), (-10, 10)]).x
                        h.update(calib=ab.tolist(), calibrated=True)
                self.heads[ctx + ':' + goal] = h
        return self

    def predict(self, A, c):
        from scipy.special import expit
        import numpy as np
        out = {}
        for goal in ('placement', 'routing', 'is_valid'):
            h = self.heads.get(context_id(c) + ':' + goal, {})
            p = h.get('prior', 0.5)
            if 'w' in h:
                w = np.array(h['w']); t = w[0] + ((np.array(vector(A))-h['mu'])/h['sd']) @ w[1:]
                p = float(expit(h['calib'][0] * t + h['calib'][1]))
            out[goal] = float(p)
            out[goal + '_calibrated'] = h.get('calibrated', False)
        return out

    def check_report(self, lines):
        out = {}
        for fam in sorted({z['A']['family'] for z in lines}):
            for ctx in sorted({context_id(z['context']) for z in lines if z['A']['family'] == fam}):
                for goal in ('placement', 'routing', 'is_valid'):
                    ds = [z for z in lines if z['A']['family'] == fam and context_id(z['context']) == ctx and labels(z)[goal] is not None]
                    out[fam + ':' + ctx[:12] + ':' + goal] = dict(role=role(fam), **reliability(
                        [labels(z)[goal] for z in ds], [self.predict(z['A'], z['context'])[goal] for z in ds]))
        return out


class Teacher:
    def __init__(self, folder, context, seed=1, new=0.25, anchor=0.15, mutation=0.35, capacity=128,
                 learning_window=512, calibration_window=256, fit_interval=16):
        self.root = Path(folder); self.root.mkdir(parents=True, exist_ok=True)
        self.context = context; self.rnd = random.Random(seed); self.new = new; self.anchor = anchor
        self.mutation = mutation; self.capacity = capacity; self.lock = threading.RLock()
        if min(capacity, learning_window, calibration_window, fit_interval) < 1:
            raise ValueError('Buffer/window/fit interval must be positive')
        self.learning_window = learning_window; self.calibration_window = calibration_window
        self.fit_interval = fit_interval; self.messages = 0
        self.model = Model()
        self.tick = 0; self.buf_dist = {}; self.lines = []; self.window = {}
        p = self.root / 'cur_state.json'
        if p.exists():
            s = json.loads(p.read_text())
            if s['context'] != context: raise ValueError('Teacher state has a different solver/budget/contract')
            self.tick = s['tick']; self.buf_dist = s['buf_dist']; self.lines = s['lines']; self.window = s['window']
            self.messages = s.get('messages', len(self.lines))
            self.lines = [self.learning_line(z) for z in self.lines]
            self.limit()

            def tup(x): return tuple(tup(v) if isinstance(v, list) else v for v in x)
            self.rnd.setstate(tup(s['chance']))
            if s.get('fit_config') == self.fit_config() and 'heads' in s:
                self.model.heads = s['heads']
            else: self.model.fit(self.lines)

    def fit_config(self):
        return [self.learning_window, self.calibration_window, self.fit_interval]

    @staticmethod
    def learning_line(z):
        keys = ('context', 'placer_metrics', 'router', 'outcome', 'placer', 'teacher_status',
                'teacher_checking', 'code_changed', 'teacher_context_deviation', 'cpu_time_limit',
                'cur_state', 'learning_data', 'early_abort')
        row = {k: z[k] for k in keys if k in z}
        if 'learning_data' in row: row['learning_data'] = dict(censored=censored(z))
        row['A'] = {k: z['A'][k] for k in ('family', 'axes', 'features_task') if k in z['A']}
        row['teacher_checking'] = {k: (z.get('teacher_checking') or {})[k] for k in ('status', 'phase')
                                if k in (z.get('teacher_checking') or {})}
        row['outcome'] = {k: (z.get('outcome') or {})[k] for k in ('conflict_list', 'open_items', 'not_connected')
                          if k in (z.get('outcome') or {})}
        return row

    def limit(self):
        counts = {}; rows = []
        for z in reversed(self.lines):
            r = role(z['A']['family']); n = counts.get(r, 0)
            limit = self.learning_window if r == 'train' else self.calibration_window
            if r in ('train', 'calibration') and n < limit:
                rows.append(z); counts[r] = n + 1
        self.lines = list(reversed(rows))

    def witness_verify(self, e):
        w = e.get('witness')
        if not w: return False
        try:
            d = Path(w['fpath'])
            ok = w['context'] == self.context and all(sha(d / f) == h for f, h in w['hashes'].items())
            ok = ok and verify(d, e['A'], e['witness_line'])['status'] == 'fulfilled'
        except (OSError, ValueError, KeyError): ok = False
        if not ok: e['witness'] = None
        return ok

    def save(self):
        p = self.root / 'cur_state.json'; tmp = p.with_suffix('.new')
        tmp.write_text(json.dumps(dict(version=VERSION, context=self.context, tick=self.tick, buf_dist=self.buf_dist,
                                      lines=self.lines, window=self.window, chance=self.rnd.getstate(),
                                      messages=self.messages, fit_config=self.fit_config(), heads=self.model.heads)))
        tmp.replace(p)

    def priority(self, e):
        p = self.model.predict(e['A'], self.context)['is_valid']

        band = 4 * p * (1-p)
        old = min(1., (self.tick-e['tick'])/20)
        return (.1 + band + e.get('faults', 0) + e.get('spacing_best', 0) + .3*old) / (1+.2*e['repeat'])

    def reject(self, recipe, reason):
        with self.lock:
            with (self.root / 'rejected.jsonl').open('a') as f:
                f.write(json.dumps(dict(tick=self.tick, recipe=recipe, reason=reason, context=self.context,
                                        status='rejected', feasibility='unknown')) + '\n')
            self.save()

    def suggestion(self):
        with self.lock:
            self.tick += 1
            es = list(self.buf_dist.values()); solved = [e for e in es if self.witness_verify(e)]
            u = self.rnd.random()
            if not es or u < self.new:
                return dict(random_recipe(self.rnd), category='new')
            if solved and u < self.new + self.anchor:

                e = max(solved, key=lambda x: (x['A']['axes'].get('factor', 0),
                                                -x['A'].get('features_task', {}).get('active_actual', 0)))
                e['repeat'] += 1
                return dict(copy.deepcopy(e['recipe']), category='anchor', parent=e['id'])
            e = self.rnd.choices(es, [self.priority(e) for e in es])[0]; e['repeat'] += 1
            if solved and self.rnd.random() < self.mutation:
                e = self.rnd.choices(solved, [self.priority(e) for e in solved])[0]
                r = copy.deepcopy(e['recipe']); ax = dict(r['axes'])
                usable = [k for k in STEPS if k not in ('active_pins', 'pitch', 'bga') or
                           any(b.startswith('Processor') for b in e['A'].get('blocks', []))]
                if e['A'].get('features_task', {}).get('mcu_pitch_actual') is None:
                    usable = [k for k in usable if k != 'pitch']
                k = self.rnd.choice(usable); f = self.window.get(k, [])[-8:]
                direction = -1 if len(f) >= 4 and sum(f)/len(f) < .3 else 1
                old_value = ax.get(k, e['A']['axes'].get(k, .8 if k == 'pitch' else e['A'].get('features_task', {}).get('bga_actual', 0)))


                actual_field = dict(part_list='parts_actual', current_paths='current_paths_actual', analog='analog_actual').get(k)
                if actual_field:
                    old_value = max(old_value, e['A'].get('features_task', {}).get(actual_field, 0))
                v = old_value + direction * STEPS[k]
                lo, hi = LIMITS[k]; ax[k] = round(min(hi, max(lo, v)), 4)
                if k == 'pitch':
                    pitches = [0.4, 0.5, 0.65, 0.8, 1.0]

                    prior = e['A']['features_task']['mcu_pitch_actual']
                    cand = [p for p in pitches if p < prior] if direction == 1 else [p for p in pitches if p > prior]
                    ax[k] = max(cand) if direction == 1 and cand else min(cand) if cand else prior
                r.update(axes=ax, category='mutation', axis=k, parent=e['id'], direction=direction)

                return r
            return dict(copy.deepcopy(e['recipe']), category='replay', parent=e['id'])

    def calibrate(self, recipe, ad, A, z, candidate, cpu_budget=None):
        return self.report(recipe, ad, A, z, candidate, cpu_budget=cpu_budget, calibration=True)

    def report(self, recipe, ad, A, z, candidate, cpu_budget=None, calibration=False):
        with self.lock:
            expected = 'calibration' if calibration else 'train'
            if role(A['family']) != expected: raise ValueError('Wrong family for ' + expected)
            task = identity(ad); pred = self.model.predict(A, self.context)
            e = None if calibration else self.buf_dist.get(task)
            if e is None:
                dest = self.root / ('calibration_tasks' if calibration else 'tasks') / task
                dest.mkdir(parents=True, exist_ok=True)
                for f in FILES + ('task.json',): shutil.copyfile(Path(ad, f), dest / f)
                e = dict(id=task, A=A, recipe={k: copy.deepcopy(recipe[k]) for k in ('seed', 'blocks', 'level', 'axes', 'family')},
                         lineage=dict(output_circuit=A['seed'], family=A['family'], parent_task=recipe.get('parent'),
                                         axis=recipe.get('axis'), direction=recipe.get('direction')),
                         repeat=0, tick=self.tick, witness=None)
                if not calibration: self.buf_dist[task] = e
            feedback = {k: copy.deepcopy(e[k]) for k in ('faults', 'spacing_best', 'score_best') if k in e}
            axis = recipe.get('axis'); previous_window = copy.deepcopy(self.window.get(axis))
            deviation = bool(z.get('code_changed') or z.get('code_sha256') != self.context['solver'] or
                             z.get('context') != self.context)
            if cpu_budget: cpu_budget.logbook(z)
            if deviation or censored(z):
                check = dict(status='unknown', reason='Solver/budget/contract missing/changed or CPU budget exhausted')
            else: check = verify(candidate, A, z)
            zz = dict(z, teacher_status=check['status'], teacher_checking=check, context=z.get('context'), A=A, task=task,
                      lineage=e['lineage'], recipe=recipe, forecast=pred, teacher_context_deviation=deviation)
            if 'student_rule' in check:
                zz['rule'] = check['student_rule']; zz['score_v3'] = check['score_v3']
            zz['is_valid'] = None if check['status'] == 'unknown' else check['status'] == 'fulfilled'
            if cpu_budget: cpu_budget.logbook(zz)
            if zz['teacher_status'] == 'fulfilled' and not calibration:
                dest = self.root / 'witnesses' / task
                if dest.exists(): shutil.rmtree(dest)
                shutil.copytree(candidate, dest)

                chk = verify(dest, A, z)
                if chk['status'] == 'fulfilled' and (cpu_budget is None or cpu_budget.rest > 0):
                    e['witness_line'] = dict(placer_metrics=z['placer_metrics'])
                    e['witness'] = dict(fpath=str(dest), context=self.context, task=task,
                                      hashes={str(p.relative_to(dest)): sha(p) for p in sorted(dest.rglob('*.json'))}, checking=chk)
                else:
                    e['witness'] = None
                    zz.update(teacher_status=chk['status'], teacher_checking=chk, is_valid=False)
            e['tick'] = self.tick

            while len(self.buf_dist) > self.capacity:
                candidates = [q for q in self.buf_dist.values() if not q.get('witness')] or list(self.buf_dist.values())
                q = min(candidates, key=self.priority); del self.buf_dist[q['id']]
            self.messages += 1

            if self.messages % self.fit_interval == 0:
                self.model.fit(self.lines)
            if cpu_budget: cpu_budget.logbook(zz)
            y = labels(zz)['is_valid']; e['faults'] = abs(y-pred['is_valid']) if y is not None else 0.
            n = zz['teacher_checking'].get('score_v3')
            if isinstance(n, (int, float)) and math.isfinite(n):
                best = e.get('score_best')
                e['spacing_best'] = min(1., max(0., (n-best)/max(1., abs(best)))) if best is not None else 0.
                if zz['teacher_status'] == 'fulfilled': e['score_best'] = min(n, best if best is not None else n)
            if not calibration and recipe.get('axis') and y is not None:
                k = recipe['axis']; self.window[k] = (self.window.get(k, []) + [y])[-8:]
            self.lines.append(self.learning_line(zz)); self.limit()
            self.save()
            if cpu_budget:
                cpu_budget.logbook(zz)
                self.lines[-1] = self.learning_line(zz)
                if zz.get('cpu_time_limit'):
                    e['witness'] = None
                    for k in ('faults', 'spacing_best', 'score_best'):
                        if k in feedback: e[k] = feedback[k]
                        else: e.pop(k, None)
                    if axis:
                        if previous_window is None: self.window.pop(axis, None)
                        else: self.window[axis] = previous_window
                    self.save()
                    cpu_budget.logbook(zz)
            if zz.get('teacher_status') == 'unknown': zz['is_valid'] = None
            return zz
