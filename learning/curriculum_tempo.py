import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import time


def active(value):
    if str(value) not in ('0', '1'): raise ValueError('curriculum_tempo expects 0|1')
    return str(value) == '1'


def level(lines, fallback, rnd, version, axis=None):
    from learning.learning_effect import _limit_result
    tasks = {}
    for z in lines[-3000:]:
        if str(z.get('version', '')).split('+')[0] != str(version): continue
        if z.get('not_routed') or z.get('generator_failure') or z.get('teacher_status') or z.get('recipe'): continue
        if z.get('level', 0) < 0 or _limit_result(z) is None: continue
        if axis:
            category = 'bga' if any('BGA' in b.upper() for b in z.get('blocks', [])) else 'qfn'
            if z.get('cu_layers') != axis['cu_layers'] or category != axis['package']: continue
        s = z.get('level_task')
        if not isinstance(s, (int, float)) or not math.isfinite(s): continue
        key = (z.get('task'), s)
        tasks[key] = tasks.get(key, False) or z['is_valid']
    group_list = {}
    for (_, s), ok in tasks.items(): group_list.setdefault(s, []).append(ok)
    quote = {s: sum(g)/len(g) for s, g in group_list.items() if len(g) >= 8}
    limit = [s for s, q in sorted(quote.items()) if .2 <= q <= .8]
    if not limit: return fallback

    all_items = sorted(set(quote) | {fallback})
    return rnd.choice(limit if rnd.random() < .8 else all_items)


def arm_penalty(ledger, key, value):
    arms = ledger.get('wasted_arms') or {}
    if not isinstance(arms, dict): return 0.
    row = arms.get('%s=%s' % (key, value), {})
    if not isinstance(row, dict): return 0.
    n, q = row.get('n'), row.get('failure')
    if not isinstance(n, int) or n < 20: return 0.
    if not isinstance(q, (int, float)) or not math.isfinite(q) or not .5 < q <= 1: return 0.
    return .25 * (q - .5) / .5


def error_book(fpath):
    try:
        b = json.loads(Path(fpath).read_text())
        return b if isinstance(b, dict) else {}
    except (OSError, ValueError): return {}




ROUTER = {'pfmul', 'router', 'shaping', 'early_abort', 'early_abort_model'}
FILES = ('placement.json', 'board.json', 'netclasses.json')


def keyname(ad, A, cfg, time_s, it, hashes):
    from learning import champion as ME
    if cfg.get('champion', '1') != '0': return None
    c = {k: v for k, v in cfg.items() if k not in ROUTER | {'curriculum_tempo', 'tempo_judge_model'}}
    if 'cost' in c: c['cost'] = {k: v for k, v in c['cost'].items() if k not in ME.ROUTER}
    inputs = {}
    for name in ('board.json', 'design_rules.json', 'specs.json', 'netclasses.json'):
        p = Path(ad, name)
        inputs[name] = hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
    raw = dict(cfg=c, task=A, time_s=time_s, it=it, code=hashes, inputs=inputs)
    return hashlib.sha256(json.dumps(raw, sort_keys=True, allow_nan=False).encode()).hexdigest()


class Preparation:
    def __init__(self): self.pool = {}; self.seeds = {}

    def seed(self, cfg):

        from learning import champion as ME
        if not cfg.get('cost') or cfg.get('champion') != '0': return
        key = {k: v for k, v in cfg.items() if k not in ROUTER | {'seed', 'cost'}}
        key['cost'] = {k: v for k, v in cfg['cost'].items() if k not in ME.ROUTER}
        s = json.dumps(key, sort_keys=True)
        cfg['seed'] = self.seeds.setdefault(s, cfg['seed'])

    def fetch(self, key, goal, cfg, k):
        if key is None or key not in self.pool: return False
        source, z, hashes = self.pool[key]

        if not all((source/n).is_file() and hashlib.sha256((source/n).read_bytes()).hexdigest() == hashes[n] for n in FILES): return False
        goal = Path(goal)
        if goal == source or goal.exists(): return False
        goal.mkdir(parents=True)
        for n in FILES: shutil.copyfile(source/n, goal/n)
        z = copy.deepcopy(z)
        z.update(cfg=copy.deepcopy(cfg), candidate=k, cpu_total=0., cpu_call_s=0., s=0.,
                 tempo_cache=dict(source=str(source), cpu_preparation_s=z.get('cpu_total')))


        (goal/'prepared.json').write_text(json.dumps(z))
        return True

    def remember(self, key, source, z):
        source = Path(source)
        if key is None or z.get('faults') or z.get('code_changed') or not z.get('feats'): return
        if not all((source/n).is_file() for n in (*FILES, 'prepared.json')): return
        self.pool[key] = (source, copy.deepcopy(z), {n: hashlib.sha256((source/n).read_bytes()).hexdigest() for n in FILES})


def judge_shadow(z, model):
    from learning.judge import Service
    service = Service(data='', model=model, images=False)
    probe = {k: z[k] for k in ('feats', 'cu_layers', 'net_class') if k in z}
    service.apply(probe)
    return dict(category='shadow', exclude=False, reason='no pre-routing calibration',
                rating=probe.get('score_ki'), status=probe.get('score_ki_status'), model=probe.get('score_ki_model'))


def attempt(cache, ad, A, cfg, k, time_s, it, **opts):
    from learning import selfplay as SS
    if not active(cfg.get('curriculum_tempo', '0')) or opts.get('prepared'):
        return SS.attempt(ad, A, cfg, k, time_s, it, **opts)
    cfg = copy.deepcopy(cfg)
    from learning import champion as ME
    cost = ME.parameter(cfg.get('cost', {}))
    if 'shaping' in cost: cfg['shaping'] = cost['shaping']
    for source, target in (('placer_style', 'placer'), ('router_style', 'router')):
        if source in cost and isinstance(cfg.get(target, 'prior'), str): cfg[target] = cost[source]
    tc = time.thread_time()
    key = keyname(ad, A, cfg, time_s, it, SS.code_hashes())
    c = Path(ad)/('k%d' % k)
    hit = cache.fetch(key, c, cfg, k)
    overhead = time.thread_time() - tc
    prep = SS.attempt(ad, A, cfg, k, time_s, it, only_place=True, prepared=hit)
    tc = time.thread_time()
    if not hit: cache.remember(key, c, prep)
    overhead += time.thread_time() - tc
    if opts.get('only_place') or not (c/'prepared.json').exists():
        prep['cpu_call_s'] = prep.get('cpu_call_s', 0) + overhead
        prep['cpu_total'] = prep.get('cpu_total', 0) + overhead
        return prep
    z = SS.attempt(ad, A, cfg, k, time_s, it, prepared=True, **opts)
    z['cpu_call_s'] += prep.get('cpu_call_s', 0) + overhead
    z['cpu_total'] = z['cpu_call_s']
    return z
