import collections
import json
import math
import numpy as np


def code_from(z):
    return None if z.get('code_changed') else (z.get('code_sha256') or z.get('code') or None)


def block_from(z):
    return (z.get('board'), json.dumps(z.get('budget') or {}, sort_keys=True),
            tuple(z.get('gnd_layers') or []), z.get('gnd_zones_sha256'), z.get('evaluation_version', 'prior'), (z.get('rule') or {}).get('minimax_version'),
            (z.get('rule') or {}).get('minimax_profile'), (z.get('rule') or {}).get('evaluation_target'))


def profile_class(k, profile_data):
    suffix = '|profile_data:' + profile_data
    return k if k.endswith(suffix) else k + suffix


def net_class_from(z, net_class):
    k = (z.get('duration') or {}).get('net_class') or net_class(z.get('features_human'))
    e = z.get('rule') or {}
    return profile_class(k, e.get('minimax_profile', 'balanced')) if e.get('evaluation_target') == 'minimax' else k


class Additive:
    def __init__(self, lines, arms, reward, net_class, code=None):
        self.arms = arms
        known = [code_from(z) for z in lines if code_from(z)]
        self.code = code or (collections.Counter(known).most_common(1)[0][0] if known else None)
        self.models = {}
        by = collections.defaultdict(list)
        for z in lines:
            if not self.code or code_from(z) != self.code: continue
            w = reward(z)
            if w is None or not math.isfinite(w): continue
            by[net_class_from(z, net_class)].append((z, w))
        for k, rows in by.items(): self.models[k] = self._fit(rows)

    def _fit(self, rows):

        defaults = dict(style='standard')
        levels = {d: sorted({_arm((z.get('cfg') or {}).get(d, defaults.get(d))) for z, _ in rows}, key=str) for d in self.arms}
        levels = {d: [a for a in v if a != 'None'] for d, v in levels.items()}
        cols = [(d, a) for d, v in levels.items() for a in v[1:]]
        def cfg(z, d): return _arm((z.get('cfg') or {}).get(d, defaults.get(d)))
        X = np.array([[float(cfg(z, d) == a) for d, a in cols] for z, _ in rows]).reshape(len(rows), len(cols))
        y = np.array([w for _, w in rows]); groups = collections.defaultdict(list)
        for i, (z, _) in enumerate(rows): groups[block_from(z)].append(i)
        for ix in groups.values():
            if len(cols): X[ix] -= X[ix].mean(axis=0)
            y[ix] -= y[ix].mean()
        inv = np.linalg.pinv(X.T @ X, rcond=1e-9)
        beta = inv @ X.T @ y; resid = y - X @ beta
        clusters = collections.defaultdict(list)
        for i, (z, _) in enumerate(rows): clusters[z.get('family') or z['board']].append(i)
        meat = np.zeros((len(cols), len(cols)))
        for ix in clusters.values():
            s = X[ix].T @ resid[ix]; meat += np.outer(s, s)
        G = len(clusters)
        cov = inv @ meat @ inv * (G / (G - 1) if G > 1 else 1)
        projection = inv @ X.T @ X
        n = {(d, a): sum(cfg(z, d) == a for z, _ in rows) for d, v in levels.items() for a in v}
        return dict(levels=levels, cols=cols, beta=beta, cov=cov, projection=projection, n=n,
                    boards=G, lines=len(rows))

    def contrast(self, k, d, a, b):
        m = self.models.get(k)
        if not m: return None
        def canonical(v):
            try: return '%g' % float(v)
            except (ValueError, TypeError): return str(v)
        def level(v): return next((x for x in m['levels'].get(d, []) if canonical(x) == canonical(v)), None)
        a, b = level(a), level(b)
        if a is None or b is None: return None
        c = np.array([float(dd == d and aa == a) - float(dd == d and aa == b) for dd, aa in m['cols']])
        identifiable = np.linalg.norm(c - m['projection'] @ c) < 1e-6
        delta = float(c @ m['beta']); se = math.sqrt(max(0, float(c @ m['cov'] @ c)))

        from scipy.stats import t
        q = float(t.ppf(0.975, max(1, m['boards'] - 1)))
        return dict(diff=delta, se=se, ci=[delta-q*se, delta+q*se] if identifiable and m['boards'] >= 3 else None,
                    identified=bool(identifiable), n=m['n'][(d, a)], n_baseline=m['n'][(d, b)], boards=m['boards'])

    def posterior(self, k, d, a):
        m = self.models.get(k)
        if not m or not m['levels'].get(d): return 0.0, 0.5, 0
        c = self.contrast(k, d, a, m['levels'][d][0])
        if not c or not c['identified']: return 0.0, 0.5, (c or {}).get('n', 0)
        return c['diff'], max(c['se'], 0.05 if not c['ci'] else 0.01), c['n']

    def export(self):
        out = {}
        for k in self.models:
            out[k] = {}
            for d, arms in self.arms.items():
                out[k][d] = {}
                for a in arms:
                    for b in arms:
                        c = self.contrast(k, d, a, b)
                        if c is not None: out[k][d]['%s|%s' % (_arm(a), _arm(b))] = c
        return dict(format='steering.v3', method='additive_board_code', code=self.code, released=False, contrasts=out,
                    measurement={k: dict(boards=m['boards'], lines=m['lines']) for k, m in self.models.items()})


def _arm(a):
    try: return '%g' % float(a)
    except (TypeError, ValueError): return str(a)
