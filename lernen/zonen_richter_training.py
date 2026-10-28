""
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile

from lernen import zonen_richter as Z

REZEPTE = ('claude_et2', 'claude_et1', 'paar_et2', 'claude_rf2')
POLITIK = 'claude-nested-family-v2'


def _note_fit(X, samples, seed, rezept):
    import numpy as np
    from sklearn.ensemble import ExtraTreesRegressor, RandomForestRegressor
    cls = RandomForestRegressor if rezept == 'claude_rf2' else ExtraTreesRegressor
    model = cls(n_estimators=64, min_samples_leaf=1 if rezept == 'claude_et1' else 2,
                max_depth=6 if rezept in ('claude_et2', 'paar_et2') else None,
                n_jobs=1, random_state=seed)
    y = [s['urteile']['claude']['note'] for s in samples]
    if rezept == 'paar_et2':
        y = np.column_stack([y, [s['urteile']['codex']['note'] for s in samples]])
    return model.fit(X, y)


def _note_predict(model, X):
    a = model.predict(X)
    return a[:, 0] if a.ndim == 2 else a


def _waehlen(X, samples, seed):
    ""
    import numpy as np
    from sklearn.model_selection import GroupKFold
    groups = [s['familie'] for s in samples]
    if len(set(groups)) < 2:
        return REZEPTE[0], []
    folds = list(GroupKFold(n_splits=min(3, len(set(groups)))).split(X, groups=groups))
    scores = []
    y = np.array([s['urteile']['claude']['note'] for s in samples])
    for recipe in REZEPTE:
        pred = np.zeros(len(samples))
        for tr, te in folds:
            pred[te] = _note_predict(_note_fit(X[tr], [samples[i] for i in tr], seed, recipe), X[te])
        scores.append(dict(rezept=recipe, spearman=Z._rho(y, pred), mae=float(np.mean(abs(y-pred)))))
    best = max(scores, key=lambda s: (s['spearman'] if s['spearman'] is not None else -2, -s['mae'], -REZEPTE.index(s['rezept'])))
    return best['rezept'], scores


def validieren(samples, names, seed=14, fehler=True, train_max=None):
    import numpy as np
    from sklearn.model_selection import LeaveOneGroupOut
    X = np.array([[s['merkmale'][k] for k in names] for s in samples])
    folds, oof = [], {}
    for tr, te in LeaveOneGroupOut().split(X, groups=[s['familie'] for s in samples]):
        if train_max is not None:
            tr = tr[:train_max]
        training = [samples[i] for i in tr]
        recipe, inner = _waehlen(X[tr], training, seed)
        grade = _note_predict(_note_fit(X[tr], training, seed, recipe), X[te])
        sy = {}
        if fehler:
            main, severity = Z._fit(X[tr], [[s['urteile']['claude']] for s in training], seed)
            yy = main.predict(X[te]); sy = {k: m.predict(X[te]) for k, m in severity.items()}
        train_families = sorted({s['familie'] for s in training})
        test_families = sorted({samples[i]['familie'] for i in te})
        assert not set(train_families) & set(test_families)
        folds.append(dict(train_familien=train_families, test_familien=test_families,
                          train_n=len(tr), test_n=len(te), rezept=recipe, innere_wahl=inner))
        for j, i in enumerate(te):
            oof[int(i)] = dict(note=float(grade[j]),
                fehler_p={k: float(yy[j, n+1]) if fehler else 0. for n, k in enumerate(Z.KEYS)},
                schwere={k: float(sy[k][j]) if k in sy else None for k in Z.KEYS})
    agreement = {r: Z._metrics([(s['urteile'][r], oof[i]) for i, s in enumerate(samples)]) for r in ('claude', 'codex')}
    human = Z._metrics([(s['urteile']['claude'], dict(note=s['urteile']['codex']['note'],
        fehler_p={k: float(k in s['urteile']['codex']['fehler']) for k in Z.KEYS},
        schwere={k: s['urteile']['codex']['schwere'].get(k) for k in Z.KEYS})) for s in samples])
    return dict(n=len(samples), familien=sorted({s['familie'] for s in samples}), folds=folds,
        lokal_gegen=agreement, lokal_doppelurteile=agreement, claude_gegen_codex=human,
        ziel_erreicht=(agreement['claude']['spearman'] is not None and human['spearman'] is not None and
                       agreement['claude']['spearman'] >= human['spearman']),
        freigegeben=False, grund='Familienvalidierung; kein unabhaengiger Produktions-/H6-Nachweis',
        oof=[dict(id=s['id'], familie=s['familie'], urteile=s['urteile'], vorhersage=oof[i]) for i, s in enumerate(samples)])


TOLERANZ = 0.015


def uebernahmetor(report, incumbent):
    ""
    current = report['lokal_gegen']['claude']['spearman']
    if current is None or not math.isfinite(current):
        return dict(bestanden=False, grund='Claude-Spearman unbekannt')
    if incumbent is None:
        return dict(bestanden=True, grund='Erstmodell mit endlicher Familienvalidierung', neu_spearman=current)
    previous = incumbent.get('validierung', {}).get('oof') or []
    if not previous:
        return dict(bestanden=False, grund='Historische Familienvorhersagen fehlen')
    key = lambda s: json.dumps(s['id'], sort_keys=True)
    index = {key(s): s for s in report['oof']}
    if any(key(s) not in index for s in previous):
        return dict(bestanden=False, grund='Historische Vergleichszonen fehlen', n_alt=len(previous), n_gemeinsam=sum(key(s) in index for s in previous))
    if any(s['urteile']['claude'] != index[key(s)]['urteile']['claude'] or s['familie'] != index[key(s)]['familie'] for s in previous):
        return dict(bestanden=False, grund='Historische Claude-Labels/Familien geaendert')
    target = [s['urteile']['claude']['note'] for s in previous]
    before = Z._rho(target, [s['vorhersage']['note'] for s in previous])
    after = Z._rho(target, [index[key(s)]['vorhersage']['note'] for s in previous])


    good = before is not None and after is not None and after >= before - TOLERANZ
    return dict(bestanden=good, grund='Historische Kohorte nicht schlechter' if good else 'Historischer Claude-Spearman gesunken/unbekannt',
                n=len(previous), vorher_spearman=before, nachher_spearman=after, neu_spearman=current)


def _atomic(path, data):
    ""
    path = Path(path)
    fd, tmp = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n')
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def trainieren(path, out, galerie=Z.GALERIE, seed=14):
    import numpy as np
    import scipy, sklearn, shapely
    out = Path(out)


    raw = Path(path).read_bytes()
    fd, snapshot = tempfile.mkstemp(suffix='.jsonl')
    try:
        with os.fdopen(fd, 'wb') as f: f.write(raw)
        cache_path = out/'merkmalscache.json'
        cache = json.loads(cache_path.read_text()) if cache_path.exists() else None
        samples, rejected = Z.daten_lesen(snapshot, galerie, cache)
    finally:
        os.unlink(snapshot)
    incomplete = [s['id'] for s in samples if set(s['urteile']) != {'claude', 'codex'}]
    samples = [s for s in samples if set(s['urteile']) == {'claude', 'codex'}]
    if not samples:
        raise ValueError('Keine brauchbaren Zonenurteile: '+str(rejected[:3]))
    if len({s['familie'] for s in samples}) < 2:
        raise ValueError('Mindestens zwei echte Quellenfamilien erforderlich')
    names = sorted(samples[0]['merkmale'])
    if any(sorted(s['merkmale']) != names or any(not math.isfinite(v) for v in s['merkmale'].values()) for s in samples):
        raise ValueError('Merkmalscache inkonsistent')
    out.mkdir(parents=True, exist_ok=True)
    with (out/'.training.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        model_path = out/'modell.json'
        previous_path = model_path
        if not previous_path.exists() and out.resolve() == Z.MODELL.parent.resolve():
            previous_path = Z.ROOT/'lernen/modell/zonen_richter/v1/modell.json'
        incumbent = json.loads(previous_path.read_text()) if previous_path.exists() else None
        sample_sha = Z.sha(json.dumps(samples,sort_keys=True,separators=(',',':')).encode())

        code_sha = Z.sha(Path(__file__).read_bytes()+Path(Z.__file__).read_bytes())
        if incumbent and incumbent.get('daten_sha') == Z.sha(raw) and incumbent.get('code_sha') == code_sha and incumbent.get('seed') == seed and incumbent.get('sample_sha') == sample_sha:
            report = dict(incumbent['validierung'], unveraendert=True)
            _atomic(out/'validierung.json', report)
            return report
        report = validieren(samples, names, seed)
        report.update(version=Z.VERSION, politik=POLITIK, abgelehnt=rejected, unvollstaendige_urteilspaare=incomplete)
        report['uebernahme'] = uebernahmetor(report, incumbent)
        X = np.array([[s['merkmale'][k] for k in names] for s in samples])
        recipe, scores = _waehlen(X, samples, seed)
        main, severity = Z._fit(X, [[s['urteile']['claude']] for s in samples], seed)
        note_model = _note_fit(X, samples, seed, recipe)
        note_trees = Z._export(note_model)

        for tree in note_trees:
            tree['value'] = [[v[0]] for v in tree['value']]
        report.update(rezept=recipe, innere_wahl=scores)
        artifact = dict(version=Z.VERSION, trainer=POLITIK, merkmal_sha=Z.merkmalsvertrag(), merkmale=names,
            fehler=Z.KEYS, seed=seed, daten_sha=Z.sha(raw), sample_sha=sample_sha, code_sha=code_sha,
            train_familien=report['familien'], train_n=len(samples), freigegeben=False,
            runtime=dict(python=__import__('sys').version.split()[0], numpy=np.__version__, scipy=scipy.__version__, sklearn=sklearn.__version__, shapely=shapely.__version__),
            bereich=dict(min=X.min(axis=0).tolist(), max=X.max(axis=0).tolist()),
            baeume=Z._export(main), note_baeume=note_trees, schwere={k: Z._export(v) for k, v in severity.items()}, validierung=report)
        _atomic(out/'kandidat_modell.json', artifact)
        _atomic(out/'merkmale.json', samples)
        _atomic(out/'merkmalscache.json', dict(merkmal_sha=Z.merkmalsvertrag(), samples=samples))
        _atomic(out/'validierung.json', report)
        if report['uebernahme']['bestanden']:
            _atomic(model_path, artifact)
        return report


def lernkurve(samples, seed=14):
    ""
    names = sorted(samples[0]['merkmale']); rows = []
    for n in (40, 120, 220):
        report = validieren(samples, names, seed, fehler=False, train_max=n)
        rows.append(dict(train_max=n, test_n=len(samples), train_n_min=min(f['train_n'] for f in report['folds']),
                         train_n_max=max(f['train_n'] for f in report['folds']), lokal_gegen={k: {m: v[m] for m in ('n','spearman','mae_note')} for k,v in report['lokal_gegen'].items()},
                         codex_gegen_claude=report['claude_gegen_codex'], folds=report['folds'], oof=report['oof']))
    return rows
