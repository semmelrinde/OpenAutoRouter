import copy
import math
from collections import Counter

from shapely.strtree import STRtree
from analysis.copper_model import geometry
from analysis.wobble import _length, _moves
from common.track_items import validate
from postprocessing.smooth45 import _octi
from postprocessing.net_shape import IdealGuard


class Corridor(IdealGuard):
    def __init__(self, B, R, rules):
        super().__init__(B, R, rules)


        for L in self.layers:
            zones = dict(R, tracks=[], vias=[])
            for net, g in geometry(dict(B, pads=[]), zones, L):
                self.obs.setdefault(L, []).append(('zone', net, g, 0., self.clr, True))
        self.trees = {L: STRtree([o[2] for o in obs]) for L, obs in self.obs.items()}
        self.maxreach = {L: max((o[3]+o[4] for o in obs), default=0.) for L, obs in self.obs.items()}


def tighten(P, w, L, net, owner, guard):
    P = [tuple(p) for p in P]
    contacts = guard.contacts(P, w, L, net, owner)
    while len(P) > 2:
        prefix = [0.]
        for a, b in zip(P, P[1:]): prefix.append(prefix[-1]+math.dist(a, b))
        proposals = []
        for i in range(len(P)-2):
            for j in range(i+2, len(P)):
                for q in _octi(P[i], P[j]):
                    if any(not (abs(b[0]-a[0]) < 1e-7 or abs(b[1]-a[1]) < 1e-7 or
                                abs(abs(b[0]-a[0])-abs(b[1]-a[1])) < 1e-7)
                           for a, b in zip(q, q[1:])): continue
                    trial = P[:i]+q+P[j+1:]
                    gain = prefix[j]-prefix[i]-_length(q)
                    if gain < -1e-9 or _moves(trial) > _moves(P): continue
                    if gain < .01 and _moves(trial) >= _moves(P): continue
                    proposals.append((-gain, len(trial), i, j, trial, q))
        chosen = None
        for _, _, i, j, trial, q in sorted(proposals):
            if guard.clear(q, w, L, net, owner) and guard.retain(trial, contacts):
                chosen = trial
                break
        if chosen is None: break
        P = chosen
    return [list(p) for p in P]


def suggestion(B, R, K, rules):
    out = copy.deepcopy(R)
    guard = Corridor(B, R, rules)
    protected = {t['net'] for key in ('tracks', 'vias', 'zones', 'zone_list') for t in R.get(key, [])
                 if any(t.get(k) for k in ('kelvin', 'fix', 'link_chain', 'current_web', 'tap'))}
    for n, k in K['net_list'].items():
        if k.get('kelvin') or k.get('fix') or k.get('pair') or k.get('length_group'):
            protected.add(n)
            if k.get('pair'): protected.add(k['pair'])

    protected.update(R.get('fix_nets') or [])
    fix = (R.get('options') or {}).get('fix')
    if fix:

        if 'fix_nets' not in R:
            from pathlib import Path
            import json
            fixed = json.loads(Path(fix).read_text())
            protected.update(t['net'] for key in ('tracks', 'vias', 'zones', 'zone_list') for t in fixed.get(key, []))
    stats = Counter(); changed = []
    for i, t in enumerate(out['tracks']):
        validate(t)
        if (K['net_list'].get(t['net'], {}).get('category') not in ('signal', 'analog') or
                t['net'] in protected or len(t['pts']) < 3 or
                any(it[0] != 'L' for it in t.get('items', [])) or
                any(abs(w-t['w'][0]) > 1e-9 for w in t['w'])):
            stats['guarded'] += 1
            continue
        P = tighten(t['pts'], t['w'][0], t['layer'], t['net'], i, guard)
        if P == t['pts']: continue
        old = t['pts']; t['pts'] = P; t['w'] = [t['w'][0]]*(len(P)-1)
        if 'items' in t: t['items'] = [['L', a, b, w] for a, b, w in zip(P, P[1:], t['w'])]
        validate(t)
        guard.add(P, t['w'][0], t['layer'], t['net'])
        changed.append(dict(track=i, net=t['net'], layer=t['layer'],
                            length_before=_length(old), length_after=_length(P),
                            moves_before=_moves(old), moves_after=_moves(P)))
    return out, dict(statistics=dict(stats), checks=dict(guard.tests), changes=changed)
