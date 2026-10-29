import collections, json, math, os, random, sys
os.environ.setdefault('OMP_NUM_THREADS', '2')
import numpy as np

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from learning import model as LM



RAW = collections.OrderedDict([
    ('mst', lambda m: math.log(max(m['mst'], 1e-6))),
    ('cross', lambda m: math.log(1 + 0.02 * m['crossings'])),
    ('rudy', lambda m: math.log(1 + max(0.0, m['rudy_max'] - 1))),
    ('hpwl', lambda m: math.log(max(m['hpwl'], 1e-6))),
    ('mst_gnd', lambda m: math.log(1 + m['mst_gnd'])),
    ('longest', lambda m: math.log(1 + m['longest_edge'])),
    ('rudy_p95', lambda m: math.log(0.05 + m['rudy_p95'])),
    ('density', lambda m: math.log(1 + m['pads_max_4mm2'])),
    ('border', lambda m: math.log(1 + m['border_pads'])),
])
HEURISTIC3 = ['mst', 'cross', 'rudy']
SMALL = ['mst', 'cross', 'rudy', 'hpwl', 'longest', 'density']
ALL = list(RAW)


def load_data(paths, max_lines=20000):
    Z = collections.deque(maxlen=max_lines)
    for p in paths:
        q = os.path.basename(p)
        for z in open(p):
            try: j = json.loads(z)
            except Exception: continue
            if not j.get('feats') or j.get('code_changed'): continue
            j['_source'] = q
            j['_task'] = str((j.get('rule') or {}).get('evaluation_target', 'v3')) + ':' + str((j.get('rule') or {}).get('minimax_version', '')) + ':' + str((j.get('rule') or {}).get('minimax_profile', '')) + ':' + os.path.abspath(p) + ':' + j['board'] + ':' + str(j.get('code_sha256') or j.get('code') or 'prior') + ':' + json.dumps(j.get('budget') or {}, sort_keys=True) + ':' + ','.join(j.get('gnd_layers') or []) + ':' + str(j.get('evaluation_version', 'prior'))
            if (j.get('rule') or {}).get('evaluation_target') == 'minimax':
                j['_task'] += ':' + json.dumps(j['rule'].get('minimax_sources'), sort_keys=True)
            j['_family'] = LM.design_family(j)
            if j.get('cur_state'):
                zs = j['cur_state']
                j['_state'] = ('time_limit_wall' if zs in ('wall_limit', 'placer_wall_limit') else
                                  'faults' if zs in ('faults', 'placer_error') else 'time_limit_cpu' if zs == 'time_limit' else zs)
            else:
                rc = j.get('router_rc')
                j['_state'] = 'time_limit_wall' if rc in ('time_limit', 'wall_limit') else 'faults' if (rc not in (0, None) or j.get('faults')) else 'ok'

            if not measurable(j): continue
            j['_valid'] = is_valid(j)
            v = rule_value(j)
            j['_value'] = 2.0 if not j['_valid'] else (v / (1.0 + v) if v is not None else min(1.0, amount(j) / 2.0))
            j['_rule'] = v
            Z.append(j)
    return list(Z)


def amount(z):
    g, c = LM.goal(z); m = z.get('human') or {}
    mk = (m.get('length_mm') or 0) + 3 * (m.get('vias') or 0)
    return min(2.0, c / mk) if g and c and mk > 0 else 2.0


def is_valid(z):
    from analysis.contract_spec import from_row
    e = z.get('rule') or {}
    geo = from_row(z)['is_valid']
    return geo and bool(e.get('minimax_valid')) if e.get('evaluation_target') == 'minimax' else geo


def rule_value(z):
    e = z.get('rule') or {}
    if e.get('evaluation_target') == 'minimax':
        from analysis import minimax as MM
        return e.get('minimax') if MM.current(e) and MM.complete(e) else None
    v = z.get('score_v3')
    if v is None: v = (z.get('rule') or {}).get('score_v3')
    try:
        v = float(v)
        if not math.isfinite(v) or v < 0: return None
        m = float(z.get('score_v3_human') or 0)
        return v / m if m > 1e-6 else v
    except (ValueError, TypeError): return None


def measurable(z):
    from learning.early_abort import aborted
    if aborted(z): return False
    e = z.get('rule') or {}
    if e.get('evaluation_target') != 'minimax': return True
    from analysis import minimax as MM
    return MM.reward(z) is not None


def rank(z):
    v = rule_value(z); e = z.get('rule') or {}
    if not measurable(z): return (2, ((), float('inf')))
    if e.get('evaluation_target') == 'minimax':
        from analysis import minimax as MM
        return (0 if is_valid(z) else 1, MM.rank(e))
    return (0 if is_valid(z) else 1, ((v if v is not None else float('inf'),), 0.0))


def _contract(z):
    e = z.get('rule') or {}
    return (e.get('evaluation_target', 'v3'), e.get('version', z.get('evaluation_version', 'prior')),
            e.get('minimax_version'), e.get('minimax_profile'))


def _gate_error(reason, **value_list):
    return dict(dict(better=False, not_worse=False, complete=False,
                     is_valid=[0, 0], common=0, metrics={}, reason=reason), **value_list)


def gate_measure(Z, candidate, baseline, B=2000):
    group_list = collections.defaultdict(list)
    for ks in tasks(Z, only_measurable=False).values():
        a, b = candidate(ks)[0], baseline(ks)[0]
        if _contract(a) != _contract(b):
            return _gate_error('different evaluation contracts')
        group_list[_contract(a)].extend(ks)
    if len(group_list) <= 1: return _gate_contract_measure(Z, candidate, baseline, B)
    gates = {repr(v): _gate_contract_measure(z, candidate, baseline, B) for v, z in sorted(group_list.items(), key=lambda q: repr(q[0]))}
    return dict(better=all(t.get('not_worse', False) for t in gates.values()) and any(t['better'] for t in gates.values()),
                complete=all(t.get('complete', False) for t in gates.values()),
                not_worse=all(t.get('not_worse', False) for t in gates.values()),
                is_valid=[sum(t['is_valid'][i] for t in gates.values()) for i in (0, 1)],
                common=sum(t['common'] for t in gates.values()), gates=gates,
                faults={v: t['reason'] for v, t in gates.items() if t.get('reason')})


def _gate_contract_measure(Z, candidate, baseline, B=2000):
    pairs = []
    for _, ks in tasks(Z, only_measurable=False).items():
        a, b = candidate(ks)[0], baseline(ks)[0]
        ea, eb = a.get('rule') or {}, b.get('rule') or {}
        if _contract(a) != _contract(b):
            return _gate_error('different evaluation contracts')
        if ea.get('evaluation_target') == 'minimax':
            from analysis import minimax as MM
            if not measurable(a) or not measurable(b): return _gate_error('missing/outdated minimax measurement')
            if not MM.current(ea) or not MM.current(eb): return _gate_error('outdated minimax version')
            if is_valid(a):
                gate_check = MM.assumption(eb, ea)
                if not gate_check['not_worse']: return _gate_error(gate_check['reason'])
        pairs.append((a, b))
    ga = sum(is_valid(a) for a, b in pairs); gb = sum(is_valid(b) for a, b in pairs)
    common = [(a, b) for a, b in pairs if is_valid(a) and is_valid(b)]
    out = dict(is_valid=[ga, gb], common=len(common), metrics={}, better=False, not_worse=False, complete=False)
    from analysis import contract_spec as VT
    contract_gates=[VT.comparison(VT.from_row(b),VT.from_row(a)) for a,b in pairs]
    out['contract_spec']=contract_gates
    contract_ok=all(t['not_worse'] for t in contract_gates)
    electrical_keepout=any(is_valid(a) and VT.from_row(a)['electrical']=='violated' for a,b in pairs)
    if electrical_keepout:contract_ok=False
    if not contract_ok:
        out['reason']='; '.join(t['reason'] for t in contract_gates if not t['not_worse'])
        if electrical_keepout:out['reason']+=' Electrically violated selection candidate'
    if ga < gb or not common: return out
    flat=all(not isinstance((z.get('rule') or {}).get('topology'),dict) for pair in common for z in pair)
    keys = list(VT.GATE) if flat else ['score_v3', 'coupling', 'parallel_rest_mm', 'branch_middle', 'sharp', 'tap_close_mm']
    if any((a.get('rule') or {}).get('evaluation_target') == 'minimax' for a, b in common):
        from analysis.rule_metrics import gate_fields
        keys += ['rule:' + k for k in gate_fields(common[0][0]['rule'])]
    si = any((z.get('rule') or {}).get('version') == 'v3.2' for a, b in common for z in (a, b))
    si_keys = ('eye', 'eye_vertical_loss', 'eye_horizontal_loss', 'noise_budget', 'noise_budget_max',
               'power_loss', 'crosstalk', 'return_path', 'topology', 'rule_checker')
    if si: keys += list(si_keys)
    def value(z, key):
        if flat:return VT.count((z.get('rule') or {}).get(key))
        if key in si_keys:
            r = z.get('rule') or {}
            return (r.get('design_rules') or r).get(key)
        if key.startswith('rule:'): return (z.get('rule') or {}).get('rule_gate', {}).get(key[6:])
        if key == 'score_v3':
            if (z.get('rule') or {}).get('evaluation_target') == 'minimax':
                return (z.get('rule') or {}).get('score_v3')
            return rule_value(z)
        r = z.get('rule') or {}
        return r.get('coupling') if key == 'coupling' else (r.get('topology') or {}).get(key)
    families = sorted({a['_family'] for a, b in common}); rnd = random.Random(0)
    draws = [rnd.choices(families, k=len(families)) for _ in range(B)]
    strict = ga > gb or any((a.get('rule') or {}).get('evaluation_target') == 'minimax' and
                             MM.assumption(b['rule'], a['rule'])['better'] for a, b in common)
    complete = contract_ok
    for key in keys:
        ds = collections.defaultdict(list)
        for a, b in common:
            va, vb = value(a, key), value(b, key)
            if va is None or vb is None or not math.isfinite(va) or not math.isfinite(vb): complete = False; continue
            ds[a['_family']].append(va-vb)
        if len(ds) != len(families) or sum(map(len, ds.values())) != len(common):
            complete = False
            out['metrics'][key] = dict(n=sum(map(len, ds.values())), ci=None); continue
        diff = float(np.mean([d for v in ds.values() for d in v]))
        boot = [np.mean([d for f in draw for d in ds.get(f, [])]) for draw in draws]
        ci = [float(np.percentile(boot, q)) for q in (2.5, 97.5)]
        single_max = max(d for v in ds.values() for d in v)
        out['metrics'][key] = dict(n=sum(map(len, ds.values())), diff=diff, ci=ci, single_max=single_max)
        if diff < -1e-9: strict = True
        if diff > 1e-9 or ci[1] > 1e-9: complete = False


        if si and single_max > 1e-9: complete = False
    out['better'] = complete and strict and len(families) >= 3
    out['not_worse'] = complete and ga >= gb
    out['complete'] = complete; out['families'] = len(families)
    return out


def tasks(Z, max_k=64, only_measurable=True):
    pro = collections.OrderedDict()
    for z in Z:
        if only_measurable and not measurable(z): continue
        pro.setdefault(z['_task'], []).append(z)
    return collections.OrderedDict((a, ks[-max_k:]) for a, ks in pro.items())


def relative(ks, names):
    X = np.array([[RAW[n](k['feats']) for n in names] for k in ks], float)
    return X - X.mean(0)


def pairs(Z, names, without_noise=True, weighted=True):
    X, y, w = [], [], []
    for a, ks in tasks(Z).items():
        Xr = relative(ks, names)
        for i in range(len(ks)):
            for j in range(len(ks)):
                if i == j: continue
                if without_noise and (ks[i]['_state'] in ('time_limit_wall', 'faults') or ks[j]['_state'] in ('time_limit_wall', 'faults')): continue
                gi, gj = is_valid(ks[i]), is_valid(ks[j])
                ei, ej = ks[i].get('rule') or {}, ks[j].get('rule') or {}
                if ei.get('evaluation_target', 'v3') != ej.get('evaluation_target', 'v3'): continue
                if ei.get('evaluation_target') == 'minimax':
                    from analysis import minimax as MM
                    c = MM.assumption(ej, ei)
                    if not (gi and c['better']) and not (gj and MM.assumption(ei, ej)['better']): continue
                    d = (-1.0 if gi else 1.0) if gi != gj else c['delta'] / (1.0 + abs(c['delta']))
                elif gi != gj: d = -1.0 if gi else 1.0
                elif not gi: continue
                else:
                    vi, vj = rule_value(ks[i]), rule_value(ks[j])
                    if vi is None or vj is None: continue
                    d = (vi - vj) / (1.0 + abs(vi) + abs(vj))
                if abs(d) < 1e-9: continue
                X.append(Xr[i] - Xr[j]); y.append(1 if d > 0 else 0); w.append(abs(d) if weighted else 1.0)
    return np.array(X), np.array(y), np.array(w)


def train(Z, names=SMALL, C=1.0, without_noise=True, weighted=True):
    from sklearn.linear_model import LogisticRegression
    X, y, w = pairs(Z, names, without_noise, weighted)
    if len(X) < 8 or len(set(y)) < 2: return dict(goal='valid_score_v3', names=list(names), scale=[1.0] * len(names), weights=[1.0 if n in HEURISTIC3 else 0.0 for n in names], pairs=len(X))
    s = X.std(0); s[s < 1e-9] = 1.0
    lr = LogisticRegression(fit_intercept=False, C=C, max_iter=1000).fit(X / s, y, sample_weight=w)
    return dict(goal='valid_score_v3', names=list(names), scale=[float(v) for v in s], weights=[float(v) for v in lr.coef_[0]], pairs=int(len(X) // 2))


def points(mod, ks):
    Xr = relative(ks, mod['names'])
    return Xr @ (np.array(mod['weights']) / np.array(mod['scale']))


def ordering_rank(mod):
    return lambda ks: [ks[i] for i in np.argsort(points(mod, ks), kind='stable')]


def heuristic_value(k):
    m = k['feats']; return m['mst'] * (1 + 0.02 * m['crossings']) * (1 + max(0.0, m['rudy_max'] - 1))


def ordering_heuristic(ks): return sorted(ks, key=heuristic_value)
def ordering_first(ks): return sorted(ks, key=lambda k: k['candidate'])
def ordering_oracle(ks): return sorted(ks, key=rank)


def ordering_prev(mod):
    def o(ks):
        P, C = LM.prediction(mod, [k['feats'] for k in ks]); E = P * C + (1 - P) * 2 * float(np.median(C))
        return [ks[i] for i in np.argsort(E, kind='stable')]
    return o


def chance(ks):
    w = [k['_value'] for k in ks]; n = len(w)
    if n < 2: return float(np.mean(w)), float(np.mean(w))
    return float(np.mean(w)), float(np.mean([min(w[i], w[j]) for i in range(n) for j in range(i + 1, n)]))


def measure(Z_pool, Z_test, models, prior=False):
    T = tasks(Z_test); fam = collections.OrderedDict()
    for a, ks in T.items(): fam.setdefault(ks[0]['_family'], []).append(a)
    erg = {}
    for f, on in fam.items():
        tr = [z for z in Z_pool if z['_family'] != f]
        order = dict(first=ordering_first, heuristic=ordering_heuristic, oracle=ordering_oracle)
        for name, kw in models.items(): order[name] = ordering_rank(train(tr, **kw))
        if prior and len({LM.goal(z)[0] for z in tr}) > 1:
            order['prev_abs'] = ordering_prev(LM.train(tr)); order['prev_rel'] = ordering_prev(LM.train_rel(tr))
        for a in on:
            ks = T[a]; e = {}
            for name, o in order.items():
                r = o(ks); e[name] = (r[0]['_value'], min(k['_value'] for k in r[:2]))
            e['chance'] = chance(ks); e['_family'] = f; e['_n'] = len(ks)
            erg[a] = e
    return erg


def fixed_measure(Z_test, orders):
    erg = {}
    for a, ks in tasks(Z_test).items():
        e = {name: ((r := o(ks))[0]['_value'], min(k['_value'] for k in r[:2])) for name, o in orders.items()}
        e['chance'] = chance(ks); e['_family'] = ks[0]['_family']; e['_n'] = len(ks)
        erg[a] = e
    return erg


def table(erg, baseline='heuristic', B=4000, seed=0):
    on = sorted(erg); fams = sorted({erg[a]['_family'] for a in on}); names = [k for k in erg[on[0]] if not k.startswith('_')]
    idx = {f: [i for i, a in enumerate(on) if erg[a]['_family'] == f] for f in fams}
    rnd = random.Random(seed); out = collections.OrderedDict()
    draw = [[i for f in rnd.choices(fams, k=len(fams)) for i in idx[f]] for _ in range(B)]
    for n in names:
        z = dict()
        for t, tag in ((0, 'one'), (1, 'two')):
            v = np.array([erg[a][n][t] for a in on]); b = np.array([erg[a][baseline][t] for a in on]); d = v - b
            bs = np.array([d[s].mean() for s in draw])
            z[tag] = dict(amount=round(float(v.mean()), 4), is_valid=int((v < 2.0).sum()), diff=round(float(d.mean()), 4),
                          ci=[round(float(np.percentile(bs, 2.5)), 4), round(float(np.percentile(bs, 97.5)), 4)],
                          better=int((d < -1e-9).sum()), worse=int((d > 1e-9).sum()))
        out[n] = z
    return dict(tasks=len(on), families=len(fams), strategies=out)


def prints(title, tab):
    print('\n%s  (%d tasks, %d families)' % (title, tab['tasks'], tab['families']))
    print('  %-14s | %-28s | %-28s' % ('Strategy', 'one candidate', 'best of two'))
    print('  %-14s | %6s %4s %8s %15s %5s | %6s %4s %8s %15s %5s' % ('', 'Value', 'valid', 'Δ heuristic', '95 %', '+/−', 'Value', 'valid', 'Δ heuristic', '95 %', '+/−'))
    for n, z in tab['strategies'].items():
        print('  %-14s | %s | %s' % (n, *['%6.4f %4d %+8.4f [%+6.3f,%+6.3f] %2d/%-2d' % (z[t]['amount'], z[t]['is_valid'], z[t]['diff'], z[t]['ci'][0], z[t]['ci'][1],
                                                                                              z[t]['better'], z[t]['worse']) for t in ('one', 'two')]))


def feature_rank(Z):
    from scipy.stats import spearmanr
    out = {}
    for n in ALL + ['heuristic']:
        r_all, r_valid = [], []
        for a, ks in tasks(Z).items():
            x = [math.log(max(heuristic_value(k), 1e-9)) if n == 'heuristic' else RAW[n](k['feats']) for k in ks]; w = [k['_value'] for k in ks]
            if len(ks) >= 3 and len(set(w)) > 1 and len(set(x)) > 1: r_all.append(spearmanr(x, w)[0])
            g = [(xi, wi) for xi, wi in zip(x, w) if wi < 2.0]
            if len(g) >= 3 and len({q[1] for q in g}) > 1 and len({q[0] for q in g}) > 1: r_valid.append(spearmanr(*zip(*g))[0])
        out[n] = dict(all_items=round(float(np.mean(r_all)), 3) if r_all else None, n_all=len(r_all),
                      valid_ones=round(float(np.mean(r_valid)), 3) if r_valid else None, n_valid=len(r_valid))
    return out


def state(Z):
    A = tasks(Z); c = collections.Counter(z['_state'] for z in Z)
    gem = sum(1 for ks in A.values() if 0 < sum(k['_value'] < 2.0 for k in ks) < len(ks))
    return dict(attempts=len(Z), tasks=len(A), families=len({z['_family'] for z in Z}), is_valid=sum(z['_value'] < 2.0 for z in Z),
                cur_state=dict(c), tasks_mixed=gem, tasks_all_valid=sum(1 for ks in A.values() if all(k['_value'] < 2.0 for k in ks)),
                tasks_none_valid=sum(1 for ks in A.values() if all(k['_value'] >= 2.0 for k in ks)))


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    paths = kw.get('data', os.path.join(R, 'learning/data/attempts.jsonl')).split(',')
    Z = load_data(paths); print('Data:', json.dumps(state(Z)))
    if sys.argv[1] == 'measure':
        mod = dict(rank=dict(names=SMALL), rank_heuristic3=dict(names=HEURISTIC3), rank_all=dict(names=ALL))
        erg = measure(Z, Z, mod, prior=kw.get('prior', '0') == '1'); tab = table(erg); prints('Leave out each family once', tab)
        print('\nRank correlation feature ↔ value within a task:', json.dumps(feature_rank(Z), indent=1))
        if kw.get('off'): json.dump(dict(data=paths, state=state(Z), table=tab), open(kw['off'], 'w'), indent=1)
    elif sys.argv[1] == 'transfer':
        T = load_data(kw['test'].split(',')); print('Test:', json.dumps(state(T)))
        erg = measure(Z, T, dict(rank=dict(names=SMALL), rank_heuristic3=dict(names=HEURISTIC3)), prior=kw.get('prior', '0') == '1')
        prints('Training on data=, test on test=', table(erg))
    elif sys.argv[1] == 'train':
        m = train(Z, SMALL); m.update(data=[os.path.relpath(p, R) for p in paths], state=state(Z), category='rank_linear_pairwise')
        off = kw.get('off', os.path.join(R, 'learning/model/rank_v2.json')); json.dump(m, open(off, 'w'), indent=1)
        print('Weights (per standard deviation):', dict(zip(m['names'], [round(g, 3) for g in m['weights']])), '->', off)


if __name__ == '__main__':
    main()
