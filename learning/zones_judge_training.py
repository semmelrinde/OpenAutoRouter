import fcntl
import json
import math
import os
from pathlib import Path
import tempfile

from learning import zones_judge as Z

RECIPES = ('claude_et2', 'claude_et1', 'pair_et2', 'claude_rf2')
POLICY = 'claude-nested-family-v2'


def _score_fit(X, samples, seed, recipe):
    import numpy as np
    from sklearn.ensemble import ExtraTreesRegressor, RandomForestRegressor
    cls = RandomForestRegressor if recipe == 'claude_rf2' else ExtraTreesRegressor
    model = cls(n_estimators=64, min_samples_leaf=1 if recipe == 'claude_et1' else 2,
                max_depth=6 if recipe in ('claude_et2', 'pair_et2') else None,
                n_jobs=1, random_state=seed)
    y = [s['verdicts']['claude']['rating'] for s in samples]
    if recipe == 'pair_et2':
        y = np.column_stack([y, [s['verdicts']['codex']['rating'] for s in samples]])
    return model.fit(X, y)


def _score_predict(model, X):
    a = model.predict(X)
    return a[:, 0] if a.ndim == 2 else a


def _choose(X, samples, seed):
    import numpy as np
    from sklearn.model_selection import GroupKFold
    groups = [s['family'] for s in samples]
    if len(set(groups)) < 2:
        return RECIPES[0], []
    folds = list(GroupKFold(n_splits=min(3, len(set(groups)))).split(X, groups=groups))
    scores = []
    y = np.array([s['verdicts']['claude']['rating'] for s in samples])
    for recipe in RECIPES:
        pred = np.zeros(len(samples))
        for tr, te in folds:
            pred[te] = _score_predict(_score_fit(X[tr], [samples[i] for i in tr], seed, recipe), X[te])
        scores.append(dict(recipe=recipe, spearman=Z._rho(y, pred), mae=float(np.mean(abs(y-pred)))))
    best = max(scores, key=lambda s: (s['spearman'] if s['spearman'] is not None else -2, -s['mae'], -RECIPES.index(s['recipe'])))
    return best['recipe'], scores


def validate(samples, names, seed=14, faults=True, train_max=None):
    import numpy as np
    from sklearn.model_selection import LeaveOneGroupOut
    X = np.array([[s['feats'][k] for k in names] for s in samples])
    folds, oof = [], {}
    for tr, te in LeaveOneGroupOut().split(X, groups=[s['family'] for s in samples]):
        if train_max is not None:
            tr = tr[:train_max]
        training = [samples[i] for i in tr]
        recipe, inner = _choose(X[tr], training, seed)
        grade = _score_predict(_score_fit(X[tr], training, seed, recipe), X[te])
        sy = {}
        if faults:
            main, severity = Z._fit(X[tr], [[s['verdicts']['claude']] for s in training], seed)
            yy = main.predict(X[te]); sy = {k: m.predict(X[te]) for k, m in severity.items()}
        train_families = sorted({s['family'] for s in training})
        test_families = sorted({samples[i]['family'] for i in te})
        assert not set(train_families) & set(test_families)
        folds.append(dict(train_families=train_families, test_families=test_families,
                          train_n=len(tr), test_n=len(te), recipe=recipe, inner_choice=inner))
        for j, i in enumerate(te):
            oof[int(i)] = dict(rating=float(grade[j]),
                error_p={k: float(yy[j, n+1]) if faults else 0. for n, k in enumerate(Z.KEYS)},
                heavy={k: float(sy[k][j]) if k in sy else None for k in Z.KEYS})
    agreement = {r: Z._metrics([(s['verdicts'][r], oof[i]) for i, s in enumerate(samples)]) for r in ('claude', 'codex')}
    human = Z._metrics([(s['verdicts']['claude'], dict(rating=s['verdicts']['codex']['rating'],
        error_p={k: float(k in s['verdicts']['codex']['faults']) for k in Z.KEYS},
        heavy={k: s['verdicts']['codex']['heavy'].get(k) for k in Z.KEYS})) for s in samples])
    return dict(n=len(samples), families=sorted({s['family'] for s in samples}), folds=folds,
        local_against=agreement, local_double_verdicts=agreement, claude_against_codex=human,
        goal_reached=(agreement['claude']['spearman'] is not None and human['spearman'] is not None and
                       agreement['claude']['spearman'] >= human['spearman']),
        released=False, reason='Family validation; no independent production/H6 evidence',
        oof=[dict(id=s['id'], family=s['family'], verdicts=s['verdicts'], prediction=oof[i]) for i, s in enumerate(samples)])


TOLERANCE = 0.015


def acceptance_gate(report, incumbent):
    current = report['local_against']['claude']['spearman']
    if current is None or not math.isfinite(current):
        return dict(passed=False, reason='Claude-Spearman unknown')
    if incumbent is None:
        return dict(passed=True, reason='First model with finite family validation', new_spearman=current)
    previous = incumbent.get('validation', {}).get('oof') or []
    if not previous:
        return dict(passed=False, reason='Historical family predictions missing')
    key = lambda s: json.dumps(s['id'], sort_keys=True)
    index = {key(s): s for s in report['oof']}
    if any(key(s) not in index for s in previous):
        return dict(passed=False, reason='Historical comparison zones missing', n_prev=len(previous), n_common=sum(key(s) in index for s in previous))
    if any(s['verdicts']['claude'] != index[key(s)]['verdicts']['claude'] or s['family'] != index[key(s)]['family'] for s in previous):
        return dict(passed=False, reason='Historical Claude labels/families changed')
    target = [s['verdicts']['claude']['rating'] for s in previous]
    before = Z._rho(target, [s['prediction']['rating'] for s in previous])
    after = Z._rho(target, [index[key(s)]['prediction']['rating'] for s in previous])


    good = before is not None and after is not None and after >= before - TOLERANCE
    return dict(passed=good, reason='Historical cohort not worse' if good else 'Historical Claude-Spearman dropped/unknown',
                n=len(previous), before_spearman=before, after_spearman=after, new_spearman=current)


def _atomic(path, data):
    path = Path(path)
    fd, tmp = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n')
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def train(path, out, gallery=Z.GALLERY, seed=14):
    import numpy as np
    import scipy, sklearn, shapely
    out = Path(out)


    raw = Path(path).read_bytes()
    fd, snapshot = tempfile.mkstemp(suffix='.jsonl')
    try:
        with os.fdopen(fd, 'wb') as f: f.write(raw)
        cache_path = out/'feature_cache.json'
        cache = json.loads(cache_path.read_text()) if cache_path.exists() else None
        samples, rejected = Z.data_read(snapshot, gallery, cache)
    finally:
        os.unlink(snapshot)
    incomplete = [s['id'] for s in samples if set(s['verdicts']) != {'claude', 'codex'}]
    samples = [s for s in samples if set(s['verdicts']) == {'claude', 'codex'}]
    if not samples:
        raise ValueError('No usable zone verdicts: '+str(rejected[:3]))
    if len({s['family'] for s in samples}) < 2:
        raise ValueError('At least two genuine source families required')
    names = sorted(samples[0]['feats'])
    if any(sorted(s['feats']) != names or any(not math.isfinite(v) for v in s['feats'].values()) for s in samples):
        raise ValueError('Feature cache inconsistent')
    out.mkdir(parents=True, exist_ok=True)
    with (out/'.training.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        model_path = out/'model.json'
        previous_path = model_path
        if not previous_path.exists() and out.resolve() == Z.MODEL.parent.resolve():
            previous_path = Z.ROOT/'learning/model/zones_judge/v1/model.json'
        incumbent = json.loads(previous_path.read_text()) if previous_path.exists() else None
        sample_sha = Z.sha(json.dumps(samples,sort_keys=True,separators=(',',':')).encode())

        code_sha = Z.sha(Path(__file__).read_bytes()+Path(Z.__file__).read_bytes())
        if incumbent and incumbent.get('data_sha') == Z.sha(raw) and incumbent.get('code_sha') == code_sha and incumbent.get('seed') == seed and incumbent.get('sample_sha') == sample_sha:
            report = dict(incumbent['validation'], unchanged=True)
            _atomic(out/'validation.json', report)
            return report
        report = validate(samples, names, seed)
        report.update(version=Z.VERSION, policy=POLICY, declined=rejected, incomplete_verdict_pairs=incomplete)
        report['adoption'] = acceptance_gate(report, incumbent)
        X = np.array([[s['feats'][k] for k in names] for s in samples])
        recipe, scores = _choose(X, samples, seed)
        main, severity = Z._fit(X, [[s['verdicts']['claude']] for s in samples], seed)
        score_model = _score_fit(X, samples, seed, recipe)
        score_trees = Z._export(score_model)

        for tree in score_trees:
            tree['value'] = [[v[0]] for v in tree['value']]
        report.update(recipe=recipe, inner_choice=scores)
        artifact = dict(version=Z.VERSION, trainer=POLICY, feature_sha=Z.feature_contract(), feats=names,
            faults=Z.KEYS, seed=seed, data_sha=Z.sha(raw), sample_sha=sample_sha, code_sha=code_sha,
            train_families=report['families'], train_n=len(samples), released=False,
            runtime=dict(python=__import__('sys').version.split()[0], numpy=np.__version__, scipy=scipy.__version__, sklearn=sklearn.__version__, shapely=shapely.__version__),
            range_=dict(min=X.min(axis=0).tolist(), max=X.max(axis=0).tolist()),
            trees=Z._export(main), score_trees=score_trees, heavy={k: Z._export(v) for k, v in severity.items()}, validation=report)
        _atomic(out/'candidate_model.json', artifact)
        _atomic(out/'features.json', samples)
        _atomic(out/'feature_cache.json', dict(feature_sha=Z.feature_contract(), samples=samples))
        _atomic(out/'validation.json', report)
        if report['adoption']['passed']:
            _atomic(model_path, artifact)
        return report


def learning_curve(samples, seed=14):
    names = sorted(samples[0]['feats']); rows = []
    for n in (40, 120, 220):
        report = validate(samples, names, seed, faults=False, train_max=n)
        rows.append(dict(train_max=n, test_n=len(samples), train_n_min=min(f['train_n'] for f in report['folds']),
                         train_n_max=max(f['train_n'] for f in report['folds']), local_against={k: {m: v[m] for m in ('n','spearman','mae_score')} for k,v in report['local_against'].items()},
                         codex_against_claude=report['claude_against_codex'], folds=report['folds'], oof=report['oof']))
    return rows
