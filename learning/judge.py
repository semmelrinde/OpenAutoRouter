import collections
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
import zlib

os.environ.setdefault('OMP_NUM_THREADS', '1')
R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
VERSION = 'judge.v1'
SEED = 7
MIN_SCORES = 100
MIN_DOUBLE = 10


def _hash(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def atomic(p, z):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    t = p.with_name(p.name + '.tmp.%s.%s' % (os.getpid(), threading.get_ident()))
    with t.open('w') as f:
        json.dump(z, f, indent=1, allow_nan=False); f.flush(); os.fsync(f.fileno())
    os.replace(t, p)


def family(z):
    if z.get('design_family'): return str(z['design_family']).lower()

    f = str(z.get('design_family') or z.get('family') or z.get('board') or '')
    f = re.sub(r'^\d+_', '', f).lower().removesuffix('_tight')
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]


def back(f):
    return zlib.crc32(f.encode()) % 5 == 0


def feats(z):
    from learning.shape_features import group_error, chain_valid, r39_ok
    rule = z.get('rule') or {}
    source_list = [z, rule, rule.get('design_rules') or {}]
    out = {}
    def planar(v, pf, parents=(), target=None, raw=False):
        if target is None: target = out
        if isinstance(v, dict):
            if 'faults' in v and pf not in ('', 'rule','rule.design_rules'): return
            if v.get('complete') is False or v.get('skipped'): return
            errors = dict(v.get('diagnose_error') or {})
            if isinstance(v.get('faults'),dict): errors.update(v['faults'])
            elif 'faults' in v: return
            sources = (*source_list, *parents, v)
            if group_error([v]): return
            for k, x in sorted(v.items()):
                if k in ('example_nodes', 'example_parallel'): continue
                if k in errors or k+'_error' in v: continue
                if k == 'current_bottlenecks' and isinstance(x, dict) and not r39_ok(sources) and 'faults' not in x and not x.get('not_measurable'):


                    from analysis.current_bottlenecks import learned_values
                    gp = (x if raw else learned_values(x)).get('measured_paths')
                    if isinstance(gp, dict) and not group_error([gp]):
                        planar(gp, pf + '.current_bottlenecks.measured_paths', (), target, raw)
                    continue
                if k not in ('rule', 'design_rules') and group_error(sources, k): continue
                if k == 'width_change' and group_error(sources, 'metrics', 'width_change'): continue
                if k == 'current_chain' and not chain_valid(x): continue
                if (k in ('net_list', 'parallel_nets') or (k == 'group_list' and pf == 'rule.design_rules.r11')) and isinstance(x, dict):
                    sums = collections.defaultdict(list)
                    for q in x.values():
                        values = {}
                        planar(q, '', sources, values, True)
                        for path, value in values.items(): sums[path].append(value)
                    for path, vals in sums.items():

                        if len(vals) != len(x): continue


                        axis = 'group_list' if k == 'group_list' else 'net_list'
                        planar(dict(medium=sum(vals)/len(vals), maximum=max(vals), total_sum=sum(vals)), pf + '.' + axis + path, sources, target, raw)
                else:
                    if k == 'current_bottlenecks':
                        if not r39_ok(sources): continue
                        if not raw:
                            from analysis.current_bottlenecks import learned_values
                            x = learned_values(x)
                    if k in ('current_bottlenecks_mm2', 'current_bottlenecks_n') and not r39_ok(sources):
                        continue
                    planar(x, pf + '.' + str(k), sources, target, raw)
        elif isinstance(v, (int, float)) and math.isfinite(v):
            target[pf] = v if raw else math.copysign(math.log1p(abs(v)), v)
            if not raw: target[pf + '.present'] = 1.0
        elif isinstance(v, list) and not raw:

            nums = [x for x in v if isinstance(x, (int, float)) and math.isfinite(x)]
            if nums: planar(dict(medium=sum(nums) / len(nums), maximum=max(nums), count=len(nums)), pf, parents, target, raw)
    SOURCES = ('metrics', 'feats', 'rule', 'r27', 'topology', 'coupling', 'score_v3', 'v3_parts',
               'score_v3_parts', 'measures_parts', 'v3_rel', 'cu_layers', 'density', 'is_valid')
    for k in SOURCES:
        planar(z.get(k), k)
    from learning.shape_features import value_list as shape_values
    for k, v in shape_values(z).items(): planar(v, 'shaping.' + k)

    out['net_class=' + str(z.get('net_class') or (z.get('duration') or {}).get('net_class') or '?')] = 1.0


    r27 = (z.get('rule') or {}).get('r27') or z.get('r27') or {}
    if isinstance(r27, dict) and r27.get('version') in ('topology-v5-trunk', 'topology-v6-trunk', 'topology-v7-trunk', 'topology-v8-trunk'):
        out['r27_version=' + r27['version']] = 1.0


    copper_map, paths = set(), set()
    def r39(v, depth=0):
        if not isinstance(v, dict) or depth > 4: return
        for kk, x in v.items():
            if kk == 'r39_version' and isinstance(x, str): copper_map.add(x)
            elif kk == 'current_bottlenecks' and isinstance(x, dict):
                if isinstance(x.get('version'), str): copper_map.add(x['version'])
                gp = x.get('measured_paths')
                if isinstance(gp, dict) and isinstance(gp.get('version'), str): paths.add(gp['version'])
            r39(x, depth + 1)
    for k in SOURCES: r39(z.get(k))
    for v in sorted(copper_map):
        if re.fullmatch(r'r39-copper_map-v\d+(\.\d+)?', v): out['r39_version=' + v] = 1.0
    for v in sorted(paths):
        if re.fullmatch(r'r39-paths-v\d+(\.\d+)?', v): out['r39_paths_version=' + v] = 1.0
    return out


def read_in(paths, max_n=5000):
    G = collections.OrderedDict(); raw_data = 0; rejected = 0
    for p in paths:
        if not Path(p).exists(): continue
        with Path(p).open() as stream:
            for l in stream:
                try:
                    z = json.loads(l); n = z['rating']; f = family(z)
                    if isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n) or not 1 <= n <= 10 or not f:
                        raise ValueError('Score/Family')
                except (ValueError, KeyError, TypeError): rejected += 1; continue
                raw_data += 1
                sid = z.get('sample_id') or hashlib.sha256((str(z.get('board')) + ':' + str(z.get('image'))).encode()).hexdigest()
                if sid in G and G[sid]['_family'] != f: raise ValueError('Contradictory family for image ' + sid)
                g = G.setdefault(sid, dict(z, sample_id=sid, _family=f, _scores={}))
                g['_scores'][str(z.get('judge') or 'unknown')] = float(n)
                if z.get('judge') == 'user': g.update({k: z.get(k) for k in ('rank', 'group', 'group_n')})

                g['rating'] = g['_scores']['user'] if 'user' in g['_scores'] else sum(g['_scores'].values()) / len(g['_scores'])
                if len(G) > max_n: G.popitem(load=False)
    Z = list(G.values())[-max_n:]
    return Z, dict(raw_data=raw_data, candidates=len(Z), rejected=rejected, window=max_n, limited=raw_data > max_n)


def measure(y, p, families):
    import numpy as np
    from scipy.stats import spearmanr
    if not len(y): return dict(n=0, families=0, spearman=None, mae=None, pro_family={})
    y, p = np.array(y), np.array(p)
    def rho(a, b):
        if len(a) < 4 or len(set(a)) < 2 or len(set(b)) < 2: return None
        v = float(spearmanr(a, b).statistic)
        return v if math.isfinite(v) else None
    F = {}
    for f in sorted(set(families)):
        ix = np.array([v == f for v in families])
        F[f] = dict(n=int(sum(ix)), mae=float(np.mean(abs(y[ix] - p[ix]))), spearman=rho(y[ix], p[ix]))
    rhos = [v['spearman'] for v in F.values() if v['spearman'] is not None]
    return dict(n=len(y), families=len(F), spearman=rho(y, p), mae=float(np.mean(abs(y - p))),
                mae_families=float(np.mean([v['mae'] for v in F.values()])),
                spearman_families=float(np.mean(rhos)) if rhos else None, pro_family=F)


def metrics_learning(Z, seed=SEED):
    from sklearn.feature_extraction import DictVectorizer
    from sklearn.ensemble import ExtraTreesRegressor
    from sklearn.dummy import DummyRegressor
    V = DictVectorizer(sparse=False); X = V.fit_transform([feats(z) for z in Z])
    M = ExtraTreesRegressor(n_estimators=48, max_depth=10, min_samples_leaf=2, random_state=seed, n_jobs=1) if len(Z) >= 8 else DummyRegressor()
    M.fit(X, [z['rating'] for z in Z])
    return dict(category='metrics', vector=V, model=M, version=VERSION)


def image_array(p, size=96):
    import numpy as np
    from PIL import Image, ImageOps
    with Image.open(p) as in_:
        in_ = in_.convert('RGB'); in_ = in_.crop((0, min(22, in_.height - 1), in_.width, in_.height))
        in_ = ImageOps.pad(in_, (size, size), color=(250, 250, 247))
        return np.asarray(in_, dtype=np.float32).transpose(2, 0, 1).copy() / 255.0


def net_name(n):
    import torch.nn as nn
    class CNN(nn.Module):
        def __init__(self):
            super().__init__()
            self.image = nn.Sequential(nn.Conv2d(3, 8, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
                                      nn.Conv2d(8, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
                                      nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.AdaptiveAvgPool2d((3, 3)), nn.Flatten())
            self.head = nn.Sequential(nn.Linear(288 + n, 32), nn.ReLU(), nn.Linear(32, 1), nn.Sigmoid())
        def forward(self, in_, x):
            import torch
            return 1 + 9 * self.head(torch.cat([self.image(in_), x], dim=1)).flatten()
    return CNN()


def image_learning(Z, seed=SEED, epochs=12):
    import numpy as np
    import torch
    from sklearn.feature_extraction import DictVectorizer
    from sklearn.preprocessing import StandardScaler
    if not torch.backends.mps.is_available(): raise RuntimeError('Image training needs MPS; no GPU fallback')
    torch.set_num_threads(1); torch.manual_seed(seed)
    V = DictVectorizer(sparse=False); sc = StandardScaler()
    X = sc.fit_transform(V.fit_transform([feats(z) for z in Z])).astype('float32')

    M = net_name(X.shape[1]).to('mps'); opt = torch.optim.Adam(M.parameters(), lr=0.002)
    rnd = np.random.default_rng(seed); M.train()
    for _ in range(epochs):
        ids = rnd.permutation(len(Z))
        for a in range(0, len(ids), 16):
            ix = ids[a:a + 16]
            in_ = torch.from_numpy(np.stack([image_array(Z[i]['image_path']) for i in ix])).to('mps')
            x = torch.from_numpy(X[ix]).to('mps'); y = torch.tensor([Z[i]['rating'] for i in ix], device='mps')
            opt.zero_grad(); loss = ((M(in_, x) - y) ** 2).mean(); loss.backward(); opt.step()
    torch.mps.synchronize()
    state = {k: v.detach().cpu() for k, v in M.state_dict().items()}
    del M, opt; torch.mps.empty_cache()
    return dict(category='image', vector=V, scale=sc, state=state, version=VERSION, epochs=epochs, seed=seed, hw_device='mps')


def prediction(M, Z):
    import numpy as np
    X = M['vector'].transform([feats(z) for z in Z])
    if M['category'] == 'metrics': return np.clip(M['model'].predict(X), 1, 10).tolist()
    import torch
    if not torch.backends.mps.is_available(): raise RuntimeError('Image prediction needs MPS')
    if '_net' not in M:
        M['_net'] = net_name(X.shape[1]).to('mps'); M['_net'].load_state_dict(M['state']); M['_net'].eval()
    X = M['scale'].transform(X).astype('float32'); out = []
    with torch.inference_mode():
        for a in range(0, len(Z), 16):
            in_ = torch.from_numpy(np.stack([image_array(z['image_path']) for z in Z[a:a + 16]])).to('mps')
            x = torch.from_numpy(X[a:a + 16]).to('mps'); out.extend(M['_net'](in_, x).cpu().tolist())
    return out


def double_measure(Z, pred):
    y, a, p, f = [], [], [], []
    for z, v in zip(Z, pred):
        N = [(k, n) for k, n in sorted(z['_scores'].items()) if k != 'unknown']
        if len(N) < 2: continue

        y.append(N[1][1]); a.append(N[0][1]); p.append(v); f.append(z['_family'])
    return dict(reviewer=measure(y, a, f), ai=measure(y, p, f), reference='lexically second reviewer',
                candidates=[z['sample_id'] for z in Z if len(z['_scores']) >= 2])


def release(m, n, real=True):
    g, k = m['double']['reviewer'], m['double']['ai']; ges = m['test']
    reasons = []
    if not real: reasons.append('synthetic or not permanently archived')
    if n < MIN_SCORES: reasons.append('under 100 distinct candidates')
    if m.get('split') != 'pinned': reasons.append('no fixed test families')
    if ges.get('n', 0) < 20 or ges.get('families', 0) < 3: reasons.append('under 20 test images / 3 test families')
    if g.get('n', 0) < MIN_DOUBLE or g.get('families', 0) < 2: reasons.append('under 10 double scores / 2 test families')
    for rk in ('spearman', 'spearman_families'):
        if g.get(rk) is None or k.get(rk) is None or g[rk] <= 0 or k[rk] < 0.8 * g[rk]:
            reasons.append('80-percent gate ' + rk)
    if g.get('mae_families') is None or k.get('mae_families') is None or k['mae_families'] > g['mae_families'] / 0.8:
        reasons.append('80-percent gate mean error')
    return not reasons, reasons


def champion_gate(Z, new, prior):
    import numpy as np
    a = measure([z['rating'] for z in Z], prior, [z['_family'] for z in Z])
    b = measure([z['rating'] for z in Z], new, [z['_family'] for z in Z])
    ds = np.array([b['pro_family'][f]['mae'] - a['pro_family'][f]['mae'] for f in sorted(a['pro_family'])])
    if not len(ds): return dict(better=False, reason='no test families')
    rnd = np.random.default_rng(SEED)
    ci = np.quantile([np.mean(rnd.choice(ds, len(ds))) for _ in range(1000)], [0.025, 0.975]).tolist()
    ok = len(ds) >= 3 and ci[1] < 0 and b['spearman'] is not None and a['spearman'] is not None and b['spearman'] >= a['spearman']
    return dict(better=bool(ok), mae_diff95=ci, champion=a, challenger=b)


def train(paths, model_dir, images=True, epochs=12, seed=SEED):

    import fcntl
    d = Path(model_dir); d.mkdir(parents=True, exist_ok=True)
    with (d / '.training.lock').open('a') as f:
        try: fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return dict(outcome='training_running', released=False)
        return _train(paths, model_dir, images, epochs, seed)


def _train(paths, model_dir, images=True, epochs=12, seed=SEED):
    import joblib
    from sklearn.model_selection import GroupKFold
    from threadpoolctl import threadpool_limits
    t0 = time.process_time(); Z, data = read_in(paths); d = Path(model_dir); d.mkdir(parents=True, exist_ok=True)
    if not Z:
        E = dict(version=VERSION, outcome='no_scores', data=data, released=False)
        atomic(d / 'last_training.json', E); return E
    tr = [z for z in Z if not back(z['_family'])]; te = [z for z in Z if back(z['_family'])]
    split = 'pinned'
    if not tr or not te:
        tr, te = Z, []; split = 'pilot_cv'
    kinds = ['metrics']
    image_ok = all(z.get('image_path') and Path(z['image_path']).is_file() for z in Z)
    if images and image_ok: kinds.append('image')
    results = {}; models = {}; faults = {}
    with threadpool_limits(limits=1):
        for category in kinds:
            fit = lambda T: metrics_learning(T, seed) if category == 'metrics' else image_learning(T, seed, epochs)
            try:

                trainfs = sorted({z['_family'] for z in tr})
                cv = dict(n=0, families=0, mae_families=None)
                if len(trainfs) >= 2:
                    P = [None] * len(tr)
                    folds = GroupKFold(n_splits=min(3, len(trainfs)))
                    for a, b in folds.split(tr, groups=[z['_family'] for z in tr]):
                        mm = fit([tr[i] for i in a]); pp = prediction(mm, [tr[i] for i in b])
                        for i, v in zip(b, pp): P[i] = v
                        del mm
                    cv = measure([z['rating'] for z in tr], P, [z['_family'] for z in tr])
                M = fit(tr)
                if te: testz, p = te, prediction(M, te)
                elif cv['n']: testz, p = tr, P
                else: testz, p = [], []
                test = measure([z['rating'] for z in testz], p, [z['_family'] for z in testz])
                meas = dict(split=split, training_cv=cv, test=test, double=double_measure(testz, p),
                            training_families=sorted({z['_family'] for z in tr}), test_families=sorted({z['_family'] for z in te}))
                real = all(z.get('sample_id') and z.get('archive') and Path(z['archive'], 'metrics.json').is_file()
                           and not z.get('synthetic') for z in Z)
                ok, reason = release(meas, len(Z), real)
                results[category] = dict(meas, released=ok, reasons=reason); models[category] = M
            except Exception as e:
                faults[category] = repr(e)
    if not models: raise RuntimeError('No model: ' + str(faults))
    category = min(models, key=lambda a: (results[a]['training_cv'].get('mae_families') if results[a]['training_cv'].get('mae_families') is not None else 999, a != 'metrics'))
    M = models[category]; M.pop('_net', None)
    identity = hashlib.sha256(json.dumps(dict(version=VERSION, seed=seed, category=category, epochs=epochs,
                            data=[(z['sample_id'], z['rating'], sorted(z['_scores'].items())) for z in Z]), sort_keys=True).encode()).hexdigest()[:20]
    mp = d / (identity + '.joblib')
    tmp = mp.with_suffix('.tmp.%d' % os.getpid()); joblib.dump(M, tmp); os.replace(tmp, mp)
    E = dict(version=VERSION, scores_units=data['raw_data'], id=identity, file=mp.name, sha256=_hash(mp), category=category, data=data,
             measurement=results[category], variants=results, faults=faults, n=len(Z), seed=seed,
             released=results[category]['released'], training_families=results[category]['training_families'],
             test_families=results[category]['test_families'], cpu_s=round(time.process_time() - t0, 3), load=os.getloadavg()[0])
    replaced = True; champion_p = d / 'champion.json'
    if champion_p.exists():
        prior = json.loads(champion_p.read_text())
        if prior.get('released'):
            if not E['released']: replaced = False
            elif set(prior['training_families']) & {z['_family'] for z in te}: replaced = False
            else:
                am = load_model(d, prior)
                E['champion_gate'] = champion_gate(te, prediction(M, te), prediction(am, te))
                replaced = E['champion_gate']['better']
    E['replaced'] = replaced
    atomic(d / 'challenger.json', E)
    if replaced: atomic(champion_p, E)
    atomic(d / 'last_training.json', E)
    return E


def load_model(d, meta):
    import joblib
    p = Path(d) / meta['file']
    if p.parent.resolve() != Path(d).resolve() or _hash(p) != meta['sha256']: raise ValueError('Model hash/path does not match')
    return joblib.load(p)


class Service:
    def __init__(self, data, model, new=20, images=True, epochs=12, cpu=300):
        self.data, self.model = Path(data), Path(model)
        self.new, self.images, self.epochs, self.cpu = max(1, new), images, epochs, cpu
        self.lock = threading.Lock(); self.thread = None; self.M = None; self.meta = {}; self.stamp = None
        self.check_time = 0; self.seen = -1
        p = self.model / 'last_training.json'
        if p.exists():
            try: self.seen = json.loads(p.read_text()).get('scores_units', -1)
            except ValueError: pass

    def load_data(self):
        p = self.model / 'champion.json'
        if not p.exists(): return
        stamp = p.stat().st_mtime_ns
        if stamp == self.stamp: return
        meta = json.loads(p.read_text()); M = load_model(self.model, meta)
        self.M, self.meta, self.stamp = M, meta, stamp

    def apply(self, z, image=None):
        with self.lock:
            z.update(score_ki=None, score_ki_model=None, score_ki_released=False, score_ki_status='no_scores')
            try:
                self.load_data()
                if self.M:
                    zz = dict(z, image_path=str(image)) if image else z
                    z['score_ki'] = float(prediction(self.M, [zz])[0]); z['score_ki_model'] = self.meta['id']
                    z['score_ki_released'] = self.meta.get('released', False)
                    z['score_ki_status'] = 'released' if z['score_ki_released'] else 'pilot'
            except Exception as e: z['score_ki_status'] = 'faults'; z['score_ki_error'] = repr(e)[:200]
        return z

    def reward(self, z, basis, weight=0.2):
        if basis is None or basis >= 2 or z.get('code_changed') or not z.get('is_valid'): return basis
        with self.lock:
            try: self.load_data()
            except Exception: return basis
            n = z.get('score_ki')
            if not self.meta.get('released') or z.get('score_ki_model') != self.meta.get('id') or not isinstance(n, (int, float)) or not math.isfinite(n) or not 1 <= n <= 10:
                return basis
        return (1 - weight) * basis + weight * (10 - n) / 9

    def verify(self):

        with self.lock:
            if time.monotonic() - self.check_time < 30 or (self.thread and self.thread.is_alive()): return
            self.check_time = time.monotonic()
            if not self.data.exists(): return
            from common import slots as PL

            if PL.tight() or os.getloadavg()[0] > 3.0 * (os.cpu_count() or 8): return
            Z, state = read_in([self.data]); n = state['raw_data']
            if not n or (self.seen >= 0 and n - self.seen < self.new): return
            self.seen = n
            def train():
                cmd = [str(R / '.venv/bin/python'), str(R / 'learning/judge.py'), 'train',
                       'data=' + str(self.data), 'model=' + str(self.model), 'images=%d' % self.images, 'epochs=%d' % self.epochs]
                self.model.mkdir(parents=True, exist_ok=True)
                r = PL.job(cmd, cpu_s=self.cpu, log=str(self.model / 'training.log'), cwd=str(R))
                atomic(self.model / 'training_run.json', r)
                try: blocked = 'training_running' in (self.model / 'training.log').read_text()[-2000:]
                except OSError: blocked = False
                if r['cur_state'] != 'ok' or blocked:
                    with self.lock: self.seen = -1
            self.thread = threading.Thread(target=train, daemon=True); self.thread.start()


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    d = Path(kw.get('model', str(R / 'learning/model/judge'))).expanduser()
    if sys.argv[1] == 'train':
        E = train([Path(p).expanduser() for p in kw.get('data', str(R / 'learning/data/appraisal.jsonl')).split(',')],
                       d, kw.get('images', '1') == '1', int(kw.get('epochs', '12')), int(kw.get('seed', SEED)))
        print(json.dumps(E, indent=1)); return
    if sys.argv[1] == 'predictions':
        z = json.loads(Path(kw['row']).read_text()); D = Service('', d); D.apply(z, kw.get('image')); print(json.dumps(z)); return
    raise SystemExit(__doc__)


if __name__ == '__main__': main()


def physics_prediction(samples, model_folder=None):
    from learning.ai.routability_service import Service as PhysicsService
    return PhysicsService(model_folder, deployment='judge').physics(samples)
