""
import collections
import json
import math
import numpy as np


def code_von(z):
    return None if z.get('code_geaendert') else (z.get('code_sha256') or z.get('code') or None)


def block_von(z):
    return (z.get('board'), json.dumps(z.get('budget') or {}, sort_keys=True),
            tuple(z.get('gndlagen') or []), z.get('gndzonen_sha256'), z.get('bewertung_version', 'alt'), (z.get('regel') or {}).get('minimax_version'),
            (z.get('regel') or {}).get('minimax_profil'), (z.get('regel') or {}).get('bewertungsziel'))


def profilklasse(k, profil):
    suffix = '|profil:' + profil
    return k if k.endswith(suffix) else k + suffix


def klasse_von(z, klasse):
    k = (z.get('dauer') or {}).get('klasse') or klasse(z.get('merkmale_mensch'))
    e = z.get('regel') or {}
    return profilklasse(k, e.get('minimax_profil', 'ausgewogen')) if e.get('bewertungsziel') == 'minimax' else k


class Additiv:
    def __init__(self, zeilen, arme, reward, klasse, code=None):
        self.arme = arme
        known = [code_von(z) for z in zeilen if code_von(z)]
        self.code = code or (collections.Counter(known).most_common(1)[0][0] if known else None)
        self.modelle = {}
        by = collections.defaultdict(list)
        for z in zeilen:
            if not self.code or code_von(z) != self.code: continue
            w = reward(z)
            if w is None or not math.isfinite(w): continue
            by[klasse_von(z, klasse)].append((z, w))
        for k, rows in by.items(): self.modelle[k] = self._fit(rows)

    def _fit(self, rows):

        defaults = dict(stil='standard')
        levels = {d: sorted({_arm((z.get('cfg') or {}).get(d, defaults.get(d))) for z, _ in rows}, key=str) for d in self.arme}
        levels = {d: [a for a in v if a != 'None'] for d, v in levels.items()}
        cols = [(d, a) for d, v in levels.items() for a in v[1:]]
        def cfg(z, d): return _arm((z.get('cfg') or {}).get(d, defaults.get(d)))
        X = np.array([[float(cfg(z, d) == a) for d, a in cols] for z, _ in rows]).reshape(len(rows), len(cols))
        y = np.array([w for _, w in rows]); groups = collections.defaultdict(list)
        for i, (z, _) in enumerate(rows): groups[block_von(z)].append(i)
        for ix in groups.values():
            if len(cols): X[ix] -= X[ix].mean(axis=0)
            y[ix] -= y[ix].mean()
        inv = np.linalg.pinv(X.T @ X, rcond=1e-9)
        beta = inv @ X.T @ y; resid = y - X @ beta
        clusters = collections.defaultdict(list)
        for i, (z, _) in enumerate(rows): clusters[z.get('familie') or z['board']].append(i)
        meat = np.zeros((len(cols), len(cols)))
        for ix in clusters.values():
            s = X[ix].T @ resid[ix]; meat += np.outer(s, s)
        G = len(clusters)
        cov = inv @ meat @ inv * (G / (G - 1) if G > 1 else 1)
        projection = inv @ X.T @ X
        n = {(d, a): sum(cfg(z, d) == a for z, _ in rows) for d, v in levels.items() for a in v}
        return dict(levels=levels, cols=cols, beta=beta, cov=cov, projection=projection, n=n,
                    boards=G, zeilen=len(rows))

    def kontrast(self, k, d, a, b):
        m = self.modelle.get(k)
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
                    identifiziert=bool(identifiable), n=m['n'][(d, a)], n_bezug=m['n'][(d, b)], boards=m['boards'])

    def posterior(self, k, d, a):
        m = self.modelle.get(k)
        if not m or not m['levels'].get(d): return 0.0, 0.5, 0
        c = self.kontrast(k, d, a, m['levels'][d][0])
        if not c or not c['identifiziert']: return 0.0, 0.5, (c or {}).get('n', 0)
        return c['diff'], max(c['se'], 0.05 if not c['ci'] else 0.01), c['n']

    def export(self):
        out = {}
        for k in self.modelle:
            out[k] = {}
            for d, arms in self.arme.items():
                out[k][d] = {}
                for a in arms:
                    for b in arms:
                        c = self.kontrast(k, d, a, b)
                        if c is not None: out[k][d]['%s|%s' % (_arm(a), _arm(b))] = c
        return dict(format='lenkung.v3', methode='additiv_board_code', code=self.code, freigegeben=False, kontraste=out,
                    messung={k: dict(boards=m['boards'], zeilen=m['zeilen']) for k, m in self.modelle.items()})


def _arm(a):
    try: return '%g' % float(a)
    except (TypeError, ValueError): return str(a)
