import functools
import hashlib
import json
import os
import tempfile
from analysis.structure import models
from analysis.impedance import z

VERSION = 'fv-stack-v3'


@functools.lru_cache(maxsize=8192)
def compute(profile_json, w, s, diff):
    p = json.loads(profile_json)
    for k in ('ref', 'type_'): p.pop(k, None)
    return float(z(w, s, single=not diff, only_odd=diff, **p, fine=0.002, coarse=0.07,
                   X=max(1.5, 4 * p['h'], w + s + 1.0), Y=max(1.5, 4 * p['h']))['Zdiff' if diff else 'Zse'])


def amount(profile_data, w, s=0.0, diff=False):
    return compute(json.dumps(profile_data, sort_keys=True), round(float(w), 4), round(float(s), 4), diff)


def table(structure, n, refs, goal, diff=False, wmin=0.09, gap_width=0.13):
    if not (0 < goal < 10000 and 0 < wmin <= 1.5 and gap_width >= 0):
        raise ValueError('Impedance table: invalid target/manufacturing limit')
    profiles = json.loads(json.dumps(models(structure, n, refs)))
    request = dict(version=VERSION, profiles=profiles, goal=goal, diff=diff, wmin=wmin, gap_width=gap_width)
    key = hashlib.sha256(json.dumps(request, sort_keys=True).encode()).hexdigest()
    cache = os.path.join(tempfile.gettempdir(), 'autorouter_impedance_cache')
    os.makedirs(cache, exist_ok=True); path = os.path.join(cache, key + '.json')
    try:
        with open(path) as f: return json.load(f)
    except (OSError, ValueError): pass
    out = {}
    for layer, p in sorted(profiles.items()):
        lo, hi = wmin, 1.5
        gap = gap_width if diff else 0.0
        zlo, zhi = amount(p, lo, gap, diff), amount(p, hi, gap, diff)
        limited = not zhi <= goal <= zlo
        for _ in range(12):
            mid = (lo + hi) / 2
            if amount(p, mid, gap, diff) > goal: lo = mid
            else: hi = mid
        w = round(wmin if goal > zlo else 1.5 if goal < zhi else (lo + hi) / 2, 4)
        out[layer] = dict(w=w, gap_width=gap, z=amount(p, w, gap, diff), goal=goal,
                          status='limited' if limited else 'ok', profile_data=p)
    fd, tmp = tempfile.mkstemp(dir=cache)
    with os.fdopen(fd, 'w') as f: json.dump(out, f, sort_keys=True)
    os.replace(tmp, path)
    return out
