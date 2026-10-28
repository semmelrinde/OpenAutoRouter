""
import functools
import hashlib
import json
import os
import tempfile
from analyse.aufbau import modelle
from analyse.impedanz import z

VERSION = 'fv-stack-v3'


@functools.lru_cache(maxsize=8192)
def rechnen(profil_json, w, s, diff):
    p = json.loads(profil_json)
    for k in ('ref', 'typ'): p.pop(k, None)
    return float(z(w, s, einzeln=not diff, nur_odd=diff, **p, fein=0.002, grob=0.07,
                   X=max(1.5, 4 * p['h'], w + s + 1.0), Y=max(1.5, 4 * p['h']))['Zdiff' if diff else 'Zse'])


def wert(profil, w, s=0.0, diff=False):
    return rechnen(json.dumps(profil, sort_keys=True), round(float(w), 4), round(float(s), 4), diff)


def tabelle(aufbau, n, refs, ziel, diff=False, wmin=0.09, spalt=0.13):
    ""
    if not (0 < ziel < 10000 and 0 < wmin <= 1.5 and spalt >= 0):
        raise ValueError('Impedanztabelle: ungueltiges Ziel/Fertigungsgrenze')
    profiles = json.loads(json.dumps(modelle(aufbau, n, refs)))
    request = dict(version=VERSION, profiles=profiles, ziel=ziel, diff=diff, wmin=wmin, spalt=spalt)
    key = hashlib.sha256(json.dumps(request, sort_keys=True).encode()).hexdigest()
    cache = os.path.join(tempfile.gettempdir(), 'autorouter_impedanz_cache')
    os.makedirs(cache, exist_ok=True); path = os.path.join(cache, key + '.json')
    try:
        with open(path) as f: return json.load(f)
    except (OSError, ValueError): pass
    out = {}
    for layer, p in sorted(profiles.items()):
        lo, hi = wmin, 1.5
        gap = spalt if diff else 0.0
        zlo, zhi = wert(p, lo, gap, diff), wert(p, hi, gap, diff)
        begrenzt = not zhi <= ziel <= zlo
        for _ in range(12):
            mid = (lo + hi) / 2
            if wert(p, mid, gap, diff) > ziel: lo = mid
            else: hi = mid
        w = round(wmin if ziel > zlo else 1.5 if ziel < zhi else (lo + hi) / 2, 4)
        out[layer] = dict(w=w, spalt=gap, z=wert(p, w, gap, diff), ziel=ziel,
                          status='begrenzt' if begrenzt else 'ok', profil=p)
    fd, tmp = tempfile.mkstemp(dir=cache)
    with os.fdopen(fd, 'w') as f: json.dump(out, f, sort_keys=True)
    os.replace(tmp, path)
    return out
