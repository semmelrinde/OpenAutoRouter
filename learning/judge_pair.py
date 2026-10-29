import collections
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
from learning.judge import atomic, family, back
from analysis import contract_spec as VT
from learning.pair_contract import measurement_signature as measurement_signature

VERSION = 'judge-pair-v1'
MEASUREMENT_SCHEME = 'judge-pair-meas-v3'
SEED = 7
MODEL = R / 'learning/model/judge_pair'
NUM = ('vias', 'signal_vias', 'bends', 'mini', 'fast_joints', 'width_change',
       'total_length', 'length_outer', 'occupancy', 'density', 'packing_loss',
       'parallel_mm', 'rule_checker', 'return_path', 'crosstalk', 'power_loss',
       'loss_mW', 'eye', 'eye_vertical_loss', 'eye_horizontal_loss', 'noise_budget', 'noise_budget_max', 'coupling', 'current_x_max', 'topology', 'current_bottlenecks_mm2')


def count(x):
    return float(x) if not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x) else None


def score_v3(z):
    for d in (z, z.get('rule') or {}, z.get('metric') or {}, z.get('design_rules') or {}):
        if not isinstance(d, dict): continue
        v = count(d.get('score_v3'))
        if v is not None and v >= 0: return v
    return None


def basis(z):
    v = score_v3(z)
    return -math.log1p(v) if v is not None else None


def components(M):
    out = collections.defaultdict(float)
    for v in (M.get('net_list') or {}).values():
        for k, x in (v.get('part_list') or {}).items():
            if count(x) is not None: out[v['net_class'] + '.' + k] += x * v.get('weight', 1)
    for k, x in (M.get('board_parts') or {}).items():
        if count(x) is not None:
            out['board.' + ('g_max' if k == 'channel_max' else 'g' if k.startswith('channel:') else k)] += x
    if M.get('si_score_active'):
        value = (M.get('signal_integrity') or {}).get('score_extra')
        if count(value) is not None: out['board.si_extra'] += value
    return dict(sorted(out.items()))


def component_occupancy(M):
    if not str(M.get('version', '')).startswith('v3') or not isinstance(M.get('net_list'), dict) or not isinstance(M.get('board_parts'), dict): return {}
    for v in M['net_list'].values():
        if not isinstance(v.get('net_class'), str) or not isinstance(v.get('part_list'), dict): return {}
        if any(count(x) is None for x in v['part_list'].values()) or count(v.get('weight', 1)) is None: return {}
    if any(count(x) is None for x in M['board_parts'].values()): return {}
    if M.get('si_score_active') and count((M.get('signal_integrity') or {}).get('score_extra')) is None: return {}
    c = components(M)
    if count(M.get('score_v3')) is None or abs(sum(c.values()) - M['score_v3']) > .011: return {}
    groups = _component_groups(c, M.get('version'))
    return dict(version='v3-components-v2', score_version=M['version'], score_v3=M['score_v3'],
                sha256=hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest(), group_list=sorted(groups))


def _component_groups(c, version):
    from analysis.rule_metrics import PARTS_V3, PARTS_V3_SI, SI_VERSIONS
    mapping = PARTS_V3_SI if version in SI_VERSIONS else PARTS_V3
    out = dict.fromkeys(mapping, 0.0)

    terms = {t:k for k,v in PARTS_V3_SI.items() for t in v}
    for key,value in c.items():
        if not isinstance(key,str) or '.' not in key or count(value) is None: return {}
        cls,term = key.split('.',1)
        group = ('power_loss' if cls == 'board' and term in ('g','g_max') else
                 'signal' if cls == 'board' and term == 'si_extra' else terms.get(term,'otherwise'))
        out[group] = out.get(group,0.0) + value
    return out

def component_groups(z):
    from analysis.rule_metrics import PARTS_V3, PARTS_V3_SI, SI_VERSIONS
    r = z.get('rule') or {}
    c = z.get('v3_components') if 'v3_components' in z else r.get('v3_components')
    if c is not None:
        if not isinstance(c,dict): return {}
        proof = z.get('v3_components_evidence') or r.get('v3_components_evidence') or {}
        if proof.get('sha256') != hashlib.sha256(json.dumps(c,sort_keys=True).encode()).hexdigest(): return {}
        version = proof.get('score_version','v3.1')
        out = _component_groups(c,version)
        if proof.get('group_list') != sorted(out): return {}
        if proof.get('version') == 'v3-components-v2':
            if count(proof.get('score_v3')) is None or abs(sum(out.values())-proof['score_v3']) > .011: return {}
        elif proof.get('version') != 'v3-components-v1': return {}
        return out
    rr = r.get('design_rules') or z.get('design_rules') or {}
    version = z.get('score_v3_version') or r.get('score_v3_version') or r.get('version')
    groups = PARTS_V3_SI if version in SI_VERSIONS else PARTS_V3
    if not isinstance(rr,dict) or any(count(rr.get(k)) is None for k in groups): return {}
    return {k:float(rr[k]) for k in groups}


def board_id(z):


    fingerprints = [v for v in (z.get('board_id'),
                    (z.get('rule') or {}).get('board_id'),
                    z['design_rules'].get('board_id') if isinstance(z.get('design_rules'),dict) else None)
                    if isinstance(v, str) and v.strip()]
    if fingerprints:
        return fingerprints[0] if len(set(fingerprints)) == 1 else None
    for v in (z.get('board'), z.get('task')):
        if isinstance(v, (str, int)) and not isinstance(v, bool) and str(v).strip():
            return str(v)
    return None


def reference_line(z):
    return {k:z[k] for k in ('board','board_id','task','factor','is_valid','score_v3','rule','gate_check',
                              'metrics','metric','feats','density','current_chain','shape_measurement','geometry_sha256','v3_components','pair_measurement_version','v3_components_evidence','score_v3_version') if k in z}



def gate_line(z):
    r = z.get('rule') or z.get('design_rules') or {}
    declared = z.get('score_v3_version') or r.get('score_v3_version') or r.get('version')
    row = dict(z, rule=r)
    if isinstance(declared, str) and declared.startswith('v3'):
        row['outcome'] = dict(z.get('outcome') or {}, score_v3_version=declared)
    t = VT.from_row(row)
    sig = z.get('pair_measurement_version') or (z.get('rule') or z.get('design_rules') or {}).get('pair_measurement_version')
    if not t.get('measurement_version') and isinstance(sig,str) and len(sig)==64:
        t['measurement_version'] = sig
    declared = z.get('score_v3_version') or (z.get('rule') or {}).get('score_v3_version') or (z.get('rule') or {}).get('version')
    if not str(t.get('score_version') or '').startswith('v3') and isinstance(declared,str) and declared.startswith('v3'):
        t['score_version'] = declared
    return t


def measurement_contract(z):
    t = gate_line(z)
    sig = z.get('pair_measurement_version') or (z.get('rule') or z.get('design_rules') or {}).get('pair_measurement_version')
    if not isinstance(sig, str) or len(sig) != 64 or not all(c in '0123456789abcdef' for c in sig): return None
    if not isinstance(t.get('measurement_version'), str) or not t['measurement_version'].strip(): return None
    if not isinstance(t.get('score_version'), str) or not t['score_version'].startswith('v3'): return None
    return dict(schema=MEASUREMENT_SCHEME, contract_spec=t['version'], rating=t['score_version'], evaluator=t['measurement_version'], pair_evaluator=sig)


def training_contract(P):
    contracts = [measurement_contract(z) for p in P for z in (p['a'], p['b'])]
    return contracts[0] if contracts and contracts[0] and all(c == contracts[0] for c in contracts) else None


def feats(z):
    from learning.shape_features import group_error, chain_valid, r39_ok
    r = z.get('rule') or {}
    rr = r.get('design_rules') or z.get('design_rules') or {}
    if not isinstance(rr, dict): rr = {}
    sources = (z,r,rr)
    tight_ok = r39_ok(sources)
    out = {}
    def add(k, v):
        v = count(v)
        if v is not None: out[k] = math.copysign(math.log1p(abs(v)), v)
    for d in (z.get('metric') or {}, z.get('metrics') or {}, z.get('feats') or {}, rr, r, z):
        if group_error([d]): continue
        for k in NUM:
            if group_error((*sources,d), k): continue
            if k == 'width_change' and group_error((*sources,d), 'metrics', 'width_change'): continue
            if k.startswith('current_bottlenecks') and not r39_ok((*sources,d)): continue
            if count(d.get(k)) is not None: add(k, d[k])
    for name, data in [('r27', r.get('r27') or rr.get('r27') or z.get('r27') or {}),
                       ('tight', r.get('current_bottlenecks') or rr.get('current_bottlenecks') or {}),
                       ('link_chain', z.get('current_chain') or r.get('current_chain') or rr.get('current_chain') or {}),
                       ('density', ((rr.get('p26') or {}).get('inner') or {}))]:
        if 'faults' in data or data.get('not_measurable') or data.get('skipped') or data.get('complete') is False: continue
        if name == 'tight' and not tight_ok: continue
        if name == 'r27' and group_error(sources, 'r27'): continue
        if name == 'link_chain' and (not chain_valid(data) or group_error(sources, 'current_chain')): continue
        for k in ('sharp','star','nodes','parallel_rest_mm','parallel_tap_mm',
                  'unnecessary_mm2','unnecessary_n','efficiency','element_max','copper_mm2','packing_loss'):
            add(name + '.' + k, data.get(k))
    from learning.shape_features import value_list as shape_values
    for k, v in shape_values(z).items(): add('shaping.' + k, v)

    for k, v in sorted((z.get('v3_components') or r.get('v3_components') or {}).items()):
        add('v3.' + k, v)
    for k,v in component_groups(z).items(): add('v3_group.'+k,v)
    return out


def _sid(z):
    return str(z.get('sample_id') or z.get('image') or '')

def read_in(paths, user=None):
    G = {}; stats = collections.Counter(); users = {}
    for p in paths:
        if not Path(p).is_file(): raise FileNotFoundError(p)
        for line in Path(p).read_text().splitlines():
            try: z = json.loads(line)
            except ValueError: stats['defective'] += 1; continue
            stats['verdicts'] += 1
            if not isinstance(z, dict) or not z.get('board') or not _sid(z) or not family(z):
                stats['identity_missing'] += 1; continue
            z = dict(z, _family=family(z))
            key = (str(z['board']), str(z.get('image') or _sid(z)), str(z.get('judge') or '?'))
            G[key] = z
    if user and not Path(user).is_file(): raise FileNotFoundError(user)
    if user:
        for line in Path(user).read_text().splitlines():
            try: u = json.loads(line)
            except ValueError: stats['user_defective'] += 1; continue
            stats['user_submissions'] += 1
            for e in u.get('entries', []):
                if not e.get('image'): continue

                key=(str(u['board']),str(e['image']),'user')
                z = dict(G.get(key) or {}, **e)
                z.update(board=u['board'], judge='user', is_valid=(e.get('metric') or {}).get('is_valid',z.get('is_valid')),
                         group=u['board'], user_image=str(Path.home() / 'Library/Caches/Autorouter/user_scores' / e['image']))
                z['_family'] = family(z)
                users[key] = z
        G.update(users)
    stats['user_candidates'] = len(users)
    Z = list(G.values())


    parents = {}
    def root(f):
        parents.setdefault(f, f)
        while parents[f] != f:
            parents[f] = parents[parents[f]]; f = parents[f]
        return f
    bases = {}
    for z in Z:
        board = str(z['board'])
        base = z['_family'] if board.startswith('erf_') else family(dict(board=board))
        bases[board] = base
        a,b = root(base),root(z['_family'])
        if a != b: parents[b] = a
    stems = collections.defaultdict(set)
    for b in bases.values(): stems[root(b)].add(b)
    aliases = collections.defaultdict(set)
    for a in parents: aliases[root(a)].add(a)
    for z in Z:
        group = root(bases[str(z['board'])])
        z['_family'] = min(stems[group]); z['_family_aliases'] = sorted(aliases[group])
    stats['candidates_reviewer'] = len(Z)
    return Z, dict(stats)


def fixed_families(Z,d):
    p=Path(d)/'families.json';reg=json.loads(p.read_text()) if p.is_file() else {}
    groups=collections.defaultdict(list)
    for z in Z: groups[z['_family']].append(z)
    for f,zs in sorted(groups.items()):
        keys=sorted(set(a for z in zs for a in z.get('_family_aliases',[f])) | {'board:'+str(z['board']) for z in zs})
        known={reg[k]['family'] for k in keys if k in reg}
        conflict=len({back(k) for k in known})>1 or any(reg[k].get('conflict') for k in keys if k in reg)
        target=min((k for k in known if back(k)),default=min(known,default=f))
        for k in keys:reg[k]=dict(family=target,conflict=bool(conflict))
        for z in zs:z['_family']=target;z['_split_conflict']=bool(conflict)
    atomic(p,reg)
    return Z


def pairs(Z, goal='rating'):
    groups = collections.defaultdict(list); out = []
    for z in Z:
        if score_v3(z) is None: continue
        if z.get('is_valid') is not True and not (z.get('judge') == 'user' and z.get('is_valid') is False): continue
        identity = board_id(z)
        if identity is None: continue
        groups[(identity, str(z.get('judge') or '?'))].append(z)
    for (board, judge), zs in sorted(groups.items()):
        for a, b in itertools.combinations(sorted(zs, key=_sid), 2):
            if _sid(a) == _sid(b) or a['_family'] != b['_family']: continue
            if a.get('is_valid') != b.get('is_valid'): continue
            ra, rb = count(a.get('rank')), count(b.get('rank'))
            na, nb = count(a.get(goal)), count(b.get(goal))
            if judge == 'user' and goal == 'rating' and ra is not None and rb is not None:
                y = 1 if ra < rb else 0 if rb < ra else None
            elif na is not None and nb is not None and 1 <= na <= 10 and 1 <= nb <= 10:
                y = 1 if na > nb else 0 if nb > na else None
            else: y = None
            if y is None: continue
            out.append(dict(a=a,b=b,y=y,family=a['_family'],board=board,judge=judge,
                            weight=5.0 if judge == 'user' else 1.0))
    return out


def differences(P, keys):
    import numpy as np
    X = []; B = []
    for p in P:
        a,b = feats(p['a']),feats(p['b'])

        X.append([a[k]-b[k] if k in a and k in b else 0.0 for k in keys])
        B.append(basis(p['a'])-basis(p['b']))
    return np.asarray(X, dtype=float).reshape(len(P), len(keys)), np.asarray(B)


def pair_weights(P):
    import numpy as np
    w = np.array([5.0 if p.get('judge') == 'user' else 1.0 if p.get('judge') else p.get('weight',1.0) for p in P])
    return w / w.sum() if len(w) else w


def learning(P, strength=1.0):
    import numpy as np
    from scipy.optimize import minimize
    from scipy.special import expit
    keys = sorted(set().union(*(set(feats(p['a'])) & set(feats(p['b'])) for p in P)))
    X,b = differences(P,keys)

    w = pair_weights(P)
    scale = np.sqrt(w @ (X*X)) if len(P) else np.ones(len(keys))
    scale[scale < 1e-8] = 1
    X = X/scale
    y = np.array([p['y'] for p in P])
    def loss(theta):
        t = b + X@theta
        return float(w @ (np.logaddexp(0,t)-y*t) + strength*.5*(theta@theta)), X.T@(w*(expit(t)-y))+strength*theta
    fit = minimize(loss,np.zeros(len(keys)),jac=True,method='L-BFGS-B') if len(P) and keys else None
    if fit is not None and not fit.success: raise RuntimeError('Pair optimization: ' + fit.message)
    return dict(version=VERSION, keys=keys, scale=scale.tolist(), coeff=fit.x.tolist() if fit is not None else [],
                strength=strength, fixed_effects={j:0.0 for j in sorted({p['judge'] for p in P})},
                effect_hint='Additive reviewer effects cancel out exactly in the conditional same-board/same-judge comparison.')


def forecast(M,P,alpha=None):
    import numpy as np
    X,B = differences(P,M['keys'])
    return B + (M.get('alpha',1) if alpha is None else alpha)*(X/np.array(M['scale']))@np.array(M['coeff'])


def measure(M,P,bootstrap=2000,seed=SEED,scores=None):
    import numpy as np
    if not P: return dict(n=0,families=0,hits=None,v3=None,difference95=None,pro_family={})
    pp = forecast(M,P) if scores is None else np.asarray(scores); _,bb = differences(P,[])

    y = np.array([p['y'] for p in P])
    hit = lambda scores: np.where(abs(scores)<1e-12,.5,((scores>0)==y).astype(float))
    h,v=hit(pp),hit(bb); all_weights=pair_weights(P); fs=sorted({p['family'] for p in P}); F={}
    for f in fs:
        ix=np.array([p['family']==f for p in P]); w=all_weights[ix]
        F[f]=dict(n=int(ix.sum()),hits=float(np.average(h[ix],weights=w)),v3=float(np.average(v[ix],weights=w)))
    w = pair_weights(P)

    masses = np.array([w[np.array([p['family']==f for p in P])].sum() for f in fs])
    dh = np.array([F[f]['hits']-F[f]['v3'] for f in fs])
    sample = np.random.default_rng(seed).integers(len(fs),size=(bootstrap,len(fs)))
    ci = np.quantile((dh[sample]*masses[sample]).sum(axis=1)/masses[sample].sum(axis=1),[.025,.975]).tolist()
    return dict(n=len(P),families=len(F),hits=float(w@h),v3=float(w@v),
                hits_families=float(np.mean([q['hits'] for q in F.values()])),
                v3_families=float(np.mean([q['v3'] for q in F.values()])),
                hits_pairs=float(h.mean()),v3_pairs=float(v.mean()),difference95=ci,
                pro_family=F,weight='Per pair AI 1, user 5; AI with family clusters')


def release(m, blind=False):
    reasons=[]
    if m['n'] < 60 or m['families'] < 3: reasons.append('under 60 test pairs / 3 test families')
    if not blind: reasons.append('no independent blind control set')
    if m['hits'] is None or m['v3'] is None: reasons.append('no evaluable control measurement')
    elif m['hits'] < m['v3'] or m.get('hits_pairs',m['hits']) < m.get('v3_pairs',m['v3']): reasons.append('worse than v3')
    families = m.get('pro_family') or {}
    if len(families) != m['families']: reasons.append('Family evidence incomplete')
    for f, q in sorted(families.items()):
        a, b = count(q.get('hits')), count(q.get('v3'))
        if a is None or b is None or a < b: reasons.append('Family regression/unknown: ' + f)
    if not (m['difference95'] and m['difference95'][0] > 0):
        reasons.append('no positive family CI of the difference to v3')
    return not reasons,reasons


def gauging(P, target_path, weights=None):
    import numpy as np
    from scipy.optimize import minimize
    from scipy.special import expit
    src=Path(weights or R/'analysis/measures_weights.json'); G=json.loads(src.read_text())
    if Path(target_path).resolve()==src.resolve(): raise ValueError('Calibration must not overwrite default weights')
    components=component_groups
    def representable(z):
        groups=components(z)
        c=z.get('v3_components') or (z.get('rule') or {}).get('v3_components') or {}
        version=(z.get('v3_components_evidence') or (z.get('rule') or {}).get('v3_components_evidence') or {}).get('score_version')
        return bool(groups) and not groups.get('otherwise') and not c.get('board.si_extra') and (version is None or version==G.get('version'))
    pp=[p for p in P if representable(p['a']) and representable(p['b'])]
    keys=sorted(set().union(*(set(components(p['a'])) | set(components(p['b'])) for p in pp))) if pp else []
    X=np.array([[-(components(p['a']).get(k,0)-components(p['b']).get(k,0))/max(1,score_v3(p['a'])+score_v3(p['b'])) for k in keys] for p in pp])
    multipliers={}
    if keys:
        y=np.array([p['y'] for p in pp]); w=pair_weights(pp)
        def objective(t):
            v=X@t
            return float(w@(np.logaddexp(0,v)-y*v)+.1*np.sum((t-1)**2)), X.T@(w*(expit(v)-y))+.2*(t-1)
        fit=minimize(objective,np.ones(len(keys)),jac=True,bounds=[(0,5)]*len(keys),method='L-BFGS-B')
        if not fit.success: raise RuntimeError('Calibration: '+fit.message)
        multipliers=dict(zip(keys,fit.x.tolist()))
        for key,factor in multipliers.items():
            from analysis.rule_metrics import PARTS_V3_SI
            for cls in G['netclasses'].values():
                for term in PARTS_V3_SI.get(key,()):
                    if term in cls['g']: cls['g'][term] *= factor
            if key == 'power_loss':
                for term in ('g','g_max'): G['group_list'][term] *= factor
    G['_pair_gauging']=dict(version=VERSION,pairs=len(pp),not_representable=len(P)-len(pp),multipliers=multipliers,released=False,
                           source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
                           reason='Separate H6/family check required' if pp else 'Complete H12 component sums missing')
    atomic(target_path,G); return G['_pair_gauging']


def train(paths,folder,user=None,seed=SEED):
    import numpy as np
    from sklearn.model_selection import GroupKFold
    from threadpoolctl import threadpool_limits
    import fcntl
    d=Path(folder); d.mkdir(parents=True,exist_ok=True)
    with (d/'.training.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        t0=time.process_time(); Z,stats=read_in(paths,user)
        if not Z: raise ValueError('No input verdicts')
        Z=fixed_families(Z,d); result={}
        with threadpool_limits(limits=1):
            for goal in ('rating','current_score'):
                P=pairs(Z,goal); historical_n=len(P)
                contracts=collections.defaultdict(list); current_sig=measurement_signature()
                for p in P:
                    c=training_contract([p])
                    if c and c['pair_evaluator']==current_sig:
                        contracts[json.dumps(c,sort_keys=True)].append(p)
                if contracts:
                    P=max(sorted(contracts.items()),key=lambda kv:len(kv[1]))[1]
                tr=[p for p in P if not back(p['family'])]; te=[p for p in P if back(p['family'])]
                fs=sorted({p['family'] for p in tr}); configs=[]
                if len(fs)>=3:
                    folds=list(GroupKFold(min(4,len(fs))).split(tr,groups=[p['family'] for p in tr]))
                    for strength in (.01,.1,1.0):
                        predictions={a:[] for a in (0,.1,.25,.5,1)}
                        truth=[]; families=[]; weights=[]
                        for it,iv in folds:
                            model=learning([tr[i] for i in it],strength)
                            pv=[tr[i] for i in iv]
                            for a in predictions: predictions[a].extend(forecast(model,pv,a).tolist())
                            truth.extend(p['y'] for p in pv); families.extend(p['family'] for p in pv); weights.extend(p['weight'] for p in pv)
                        for a,scores in predictions.items():
                            h=np.where(np.abs(scores)<1e-12,.5,(np.array(scores)>0)==truth)
                            value=float(np.average(h,weights=weights))
                            configs.append((value,-a,strength,a))
                chosen=max(configs,default=(0,0,1,0),key=lambda q:(q[0],q[1],q[2]))
                M=learning(tr,chosen[2]); M['alpha']=chosen[3]; M['goal']=goal
                test=measure(M,te,seed=seed)
                from learning.pair_blind import control_pairs
                control, blind_audit = control_pairs(te)
                control_test=measure(M,control,seed=seed)
                ok,reasons=release(control_test,bool(control))
                contract = training_contract(tr + control)
                if contract is None:
                    ok = False; reasons.append('Measurement schema/evaluator signature missing or mixed')
                M['measurement_contract'] = contract
                coverage=collections.Counter(k for z in Z for k in feats(z))
                meta=dict(version=VERSION,measurement_contract=contract,category='pair',goal=goal,released=ok,reasons=reasons,test=test,inspection=control_test,
                          blind_audit=blind_audit, training_cv=dict(hits=chosen[0],alpha=M['alpha'],strength=M['strength']),
                          training_families=fs,test_families=sorted({p['family'] for p in te}),pairs=len(P),
                          training_boards=sorted({p['board'] for p in tr}),test_boards=sorted({p['board'] for p in te}),
                          historical_pairs=historical_n, measurement_mode='current_Contract' if contracts else 'historical_Pilot',
                          training_pairs=len(tr),test_pairs=len(te),user_pairs=sum(p['judge']=='user' for p in P),
                          coverage=dict(coverage),data=stats,seed=seed)
                ident=hashlib.sha256(json.dumps(dict(M=M,labels=[(_sid(p['a']),_sid(p['b']),p['judge'],p['y'],p['weight']) for p in P]),sort_keys=True).encode()).hexdigest()[:20]
                mp=d/(goal+'_'+ident+'.json'); atomic(mp,M)
                meta.update(id=ident,file=mp.name,sha256=hashlib.sha256(mp.read_bytes()).hexdigest(),cpu_s=round(time.process_time()-t0,3),load=os.getloadavg()[0])
                atomic(d/(goal+'_challenger.json'),meta)

                champion=d/(goal+'_champion.json'); replaced=True
                if champion.exists():
                    old=json.loads(champion.read_text())
                    if old.get('released'):
                        replaced=False
                        if ok and old.get('measurement_contract') == contract and not set(old['training_families']) & {p['family'] for p in te} and not set(old.get('training_boards',[])) & {p['board'] for p in te}:
                            am=load_model(d,old); prior=measure(am,control,seed=seed)
                            ds=np.array([control_test['pro_family'][f]['hits']-prior['pro_family'][f]['hits'] for f in control_test['pro_family']])
                            ci=np.quantile(np.random.default_rng(seed).choice(ds,(2000,len(ds))).mean(axis=1),[.025,.975]).tolist()
                            meta['champion_difference95']=ci; replaced=ci[0]>0
                meta['replaced']=replaced
                atomic(d/(goal+'_challenger.json'),meta)
                if replaced: atomic(champion,meta)
                result[goal]=meta
                if goal=='rating':
                    calibration=gauging(tr,d/'measures_weights.json')
                    cp=[p for p in control if component_groups(p['a']) and component_groups(p['b'])]
                    weights=calibration['multipliers']
                    scores=[-sum(weights.get(k,1)*(component_groups(p['a']).get(k,0)-component_groups(p['b']).get(k,0)) for k in set(component_groups(p['a'])) | set(component_groups(p['b'])))/max(1,score_v3(p['a'])+score_v3(p['b'])) for p in cp]
                    calibration['test']=measure(None,cp,seed=seed,scores=np.asarray(scores))
                    calibration['pair_gate'],calibration['pair_reasons']=release(calibration['test'],bool(cp))

                    result['gauging']=calibration
                    candidate=json.loads((d/'measures_weights.json').read_text());candidate['_pair_gauging']=calibration
                    atomic(d/'measures_weights.json',candidate)
        atomic(d/'last_training.json',result); return result


def load_model(d,meta):
    p=Path(d)/meta['file']
    if p.parent.resolve()!=Path(d).resolve() or hashlib.sha256(p.read_bytes()).hexdigest()!=meta['sha256']:
        raise ValueError('Pair model path/hash does not match')
    M=json.loads(p.read_text())
    if M.get('version')!=VERSION: raise ValueError('PairModel-Version')
    keys,scale,coef=M.get('keys'),M.get('scale'),M.get('coeff')
    if not all(isinstance(v,list) for v in (keys,scale,coef)) or len(keys)!=len(scale) or len(keys)!=len(coef):
        raise ValueError('PairModel-Dimensions')
    if any(not isinstance(k,str) for k in keys) or len(set(keys))!=len(keys): raise ValueError('PairModel-FeatureList')
    if any(count(v) is None or v<=0 for v in scale) or any(count(v) is None for v in coef):
        raise ValueError('PairModel-Coefficients/Scale')
    if count(M.get('alpha')) is None or not 0<=M['alpha']<=1: raise ValueError('PairModel-CorrectionFactor')
    if M.get('measurement_contract') != meta.get('measurement_contract'): raise ValueError('PairModel-MeasurementContract')
    return M


class Service:
    def __init__(self,data=None,model=MODEL,new=20,images=False,epochs=0,cpu=300,user=None,goal='rating'):
        self.data=Path(data or R/'learning/data/appraisal.jsonl'); self.model=Path(model)
        self.user=Path(user or self.data.with_name('user_scores.jsonl'))
        self.new=new; self.cpu=cpu; self.goal=goal; self.lock=threading.RLock()
        self.meta={}; self.M=None; self.stamp=None; self.thread=None; self.check_time=0; self.seen=None
        self.verdicts = -1; self.user_state = None
        load = self.model/'last_training.json'
        if load.is_file():
            try: self.verdicts = json.loads(load.read_text())['rating']['data']['verdicts']
            except (ValueError,KeyError): pass

    def load_data(self):
        p=self.model/(self.goal+'_champion.json')
        if not p.is_file(): self.M=None; self.meta={}; self.stamp=None; return
        stamp=p.stat().st_mtime_ns
        if stamp!=self.stamp:
            meta=json.loads(p.read_text()); M=load_model(self.model,meta)
            self.M,self.meta,self.stamp=M,meta,stamp

    def apply(self,z,image=None):
        with self.lock:
            z.update(pair_released=False,pair_status='no_scores',pair_model=None,pair_score=None,
                     pair_model_path=str(self.model.resolve()), pair_model_file=None)
            try:
                self.load_data()
                if self.M and basis(z) is not None:
                    features = feats(z)
                    score = basis(z) + self.M.get('alpha',1) * sum(features.get(k,0)/s*c for k,s,c in
                                zip(self.M['keys'],self.M['scale'],self.M['coeff']))
                    applicable = self.applicable(z)
                    z.update(pair_model=self.meta['id'], pair_model_file=str((self.model/self.meta['file']).resolve()),
                             pair_released=applicable, pair_status='released' if applicable else 'pilot', pair_score=score)
            except (OSError,ValueError,KeyError): z['pair_status']='faults'
        return z

    def applicable(self, z):
        contract = measurement_contract(z)
        return bool(board_id(z) is not None and self.M and self.meta.get('released') is True and contract and
                    self.M.get('measurement_contract') == self.meta.get('measurement_contract') == contract)

    def comparison(self,a,b):
        if board_id(a) is None or board_id(a) != board_id(b):
            return dict(better=False,not_worse=False,reason='Board missing/different',pair=False)
        gate=VT.comparison(gate_line(a),gate_line(b))
        answer=dict(better=False,not_worse=gate['not_worse'],reason=gate['reason'],pair=False)
        if not a.get('is_valid') or not b.get('is_valid') or not gate['not_worse']: return answer
        na,nb=score_v3(a),score_v3(b)
        if na is None or nb is None or nb>na+1e-9: return answer
        try:
            with self.lock:
                self.load_data()
                if self.applicable(a) and self.applicable(b) and set(feats(a)) == set(feats(b)):
                    score=float(forecast(self.M,[dict(a=b,b=a)])[0])
                    answer.update(better=score>1e-12,reason='Pair model after hard gate',pair=True,difference=score)
                    return answer
        except (OSError,ValueError,KeyError): pass
        answer.update(better=gate['better'],reason='v3/contract (pair model not released)')
        return answer

    def extra(self,z,reference,weight=.03):
        if not z.get('is_valid') or not reference or not reference.get('is_valid'): return 0.0
        if board_id(z) is None or board_id(z) != board_id(reference): return 0.0
        try:
            with self.lock:
                self.load_data()
                if not self.applicable(z) or not self.applicable(reference) or basis(z) is None or basis(reference) is None: return 0.0

                if not VT.comparison(gate_line(reference), gate_line(z))['not_worse']: return 0.0
                if set(feats(z))!=set(feats(reference)): return 0.0
                score=float(forecast(self.M,[dict(a=z,b=reference)])[0])
                return -min(.05,max(0,weight))*math.tanh(score/2)
        except (OSError,ValueError,KeyError): return 0.0

    def reward(self,z,basis,weight=.03):
        if basis is None or basis>=2 or not z.get('is_valid') or z.get('code_changed'): return basis
        ref=z.get('pair_reference')
        extra = self.extra(z,ref,weight)
        if extra == 0.0: return basis
        value = min(1.9999,max(0,basis+extra))

        if z.get('task') is not None and (z.get('rule') or {}).get('evaluation_target') != 'minimax':
            sx = count((z.get('rule') or {}).get('current_x_max'))
            if sx is None: return basis
            value = min(1.4999,max(1,value)) if sx <= 1 else min(1.9999,max(1.5001,value))
        return value

    def verify(self):
        with self.lock:
            if time.monotonic()-self.check_time<30 or self.thread and self.thread.is_alive(): return
            self.check_time=time.monotonic()
            from common import slots as PL
            if PL.tight(): return
            stamp=tuple((p.stat().st_size,p.stat().st_mtime_ns) if p.is_file() else None for p in (self.data,self.user))
            if stamp==self.seen or not self.data.is_file(): return
            count=sum(1 for line in self.data.open() if line.strip())
            if self.verdicts>=0 and count-self.verdicts<self.new and stamp[1]==self.user_state: return
            def train():
                self.model.mkdir(parents=True,exist_ok=True)
                cmd=[str(R/'.venv/bin/python'),str(R/'learning/judge_pair.py'),'train','data='+str(self.data),
                     'model='+str(self.model)]
                if self.user.is_file(): cmd.append('user='+str(self.user))
                run=PL.job(cmd,cpu_s=self.cpu,log=str(self.model/'training.log'),cwd=str(R))
                atomic(self.model/'training_run.json',run)
                if run['cur_state']=='ok':
                    self.seen=stamp; self.verdicts=count; self.user_state=stamp[1]
            self.thread=threading.Thread(target=train,daemon=True); self.thread.start()


def quality_reward(z,base_value):
    if base_value is None or base_value>=2 or not z.get('is_valid'): return base_value
    r=z.get('rule') or {}; chain=z.get('current_chain') or r.get('current_chain') or {}
    from learning.shape_features import chain_valid, group_error
    if not chain_valid(chain) or group_error((z, r, r.get('design_rules') or {}, z.get('design_rules') or {}), 'current_chain', 'current_x_max'):
        return base_value
    eta,element=count(chain.get('efficiency')),count(chain.get('element_max'))
    sx=count(r.get('current_x_max'))
    if chain.get('skipped') or 'faults' in chain or eta is None or eta<=0 or element is None or sx is None: return base_value
    worst=max(sx,1/eta,element,1)
    severity=max(0,1-1/worst)

    n=score_v3(z); tail=n/(1+n) if n is not None else 1
    if sx>1: return 1.5001+.4599*severity+.03*tail
    return 1+.4599*severity+.03*tail


if __name__=='__main__':
    kw=dict(a.split('=',1) for a in sys.argv[2:] if '=' in a)
    if sys.argv[1:2]!=['train']: raise SystemExit(__doc__)
    user_path=Path(kw.get('user',R/'learning/data/user_scores.jsonl')).expanduser()
    result=train([Path(p).expanduser() for p in kw.get('data',str(R/'learning/data/appraisal.jsonl')).split(',')],
                      Path(kw.get('model',MODEL)).expanduser(),user_path if 'user' in kw or user_path.is_file() else None,int(kw.get('seed',SEED)))
    print(json.dumps({k:{kk:vv for kk,vv in v.items() if kk not in ('coverage','test','training_families','test_families')} for k,v in result.items()},indent=1))
