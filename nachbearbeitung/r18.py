""
import copy
import math
from collections import defaultdict
from shapely.geometry import LineString
from nachbearbeitung import r24
from nachbearbeitung.glaetten45 import _octi, _zusammenfassen

VERSION = 'r18-ketten-v1'


def _direction(a, b):
    return round(math.atan2(b[1]-a[1], b[0]-a[0]), 9)


def straffen(pts, w, n, corridor):
    lengths = [0.]
    for a,b in zip(pts, pts[1:]): lengths.append(lengths[-1]+math.dist(a,b))

    states = [dict() for _ in pts]; states[0][None] = (0, 0., None, None)
    for i in range(len(pts)-1):
        choices = [(i+1, [pts[i], pts[i+1]])]
        for j in sorted(set(range(i+2, min(len(pts), i+65))) | {len(pts)-1}):
            if j <= i+1: continue
            for q in _octi(pts[i], pts[j]):
                q = _zusammenfassen(q)
                ll = sum(math.dist(a,b) for a,b in zip(q,q[1:]))
                if ll <= lengths[j]-lengths[i]+1e-9 and corridor.free(q,w,n): choices.append((j,q))
        for j,q in choices:
            dirs = [_direction(a,b) for a,b in zip(q,q[1:])]
            if not dirs: continue
            internal = sum(a != b for a,b in zip(dirs,dirs[1:]))
            ll = sum(math.dist(a,b) for a,b in zip(q,q[1:]))
            for last, state in sorted(states[i].items(), key=lambda kv: str(kv[0])):
                cost = (state[0]+internal+int(last is not None and last != dirs[0]), state[1]+ll)
                old = states[j].get(dirs[-1])
                if old is None or cost < old[:2]: states[j][dirs[-1]] = (*cost, (i,last), q)
    j = len(pts)-1; last = min(states[j], key=lambda d: (states[j][d][:2], d))
    pieces = []
    while j:
        state = states[j][last]; pieces.append(state[3]); j,last = state[2]
    return _zusammenfassen([pts[0]] + [p for q in reversed(pieces) for p in q[1:]])


def vorschlag(B, R, K):
    new = copy.deepcopy(R); groups = defaultdict(list)
    for i,t in enumerate(R.get('tracks', [])): groups[(t.get('net'),t['layer'])].append((i,t))
    protected = r24._laengenschutz(K); info = dict(version=VERSION, ketten=0, geaendert=0, gratis_mm=0., geschuetzt=0)
    removed = set(); additions = []; corridors = {}
    for (n,L),tracks in sorted(groups.items(), key=lambda kv: (str(kv[0][0]),kv[0][1])):
        if not n: continue
        if (any(r24._schutz(t,K,protected) for _,t in tracks) or n in R.get('flaechennetze', []) or
            any(z.get('net') == n and not r24._zone_polys(z) for z in r24._zones(R))):
            info['geschuetzt'] += 1; continue
        edges,adj,anchors = r24._graph(B,R,K,n,L,tracks,protected)
        changed = set(); added = []
        for pts,ids in r24._chains(edges,adj,anchors,set()):
            info['ketten'] += 1
            if len(pts)<3 or pts[0]==pts[-1] or any(edges[i]['protected'] for i in ids): continue
            if L not in corridors: corridors[L] = r24.Corridor(B,R,L)
            w = edges[ids[0]]['w']; q = straffen(pts,w,n,corridors[L])


            if len(q)<2 or q[0]!=pts[0] or q[-1]!=pts[-1]: continue
            if q == _zusammenfassen(pts): continue
            old = sum(math.dist(a,b) for a,b in zip(pts,pts[1:])); after = sum(math.dist(a,b) for a,b in zip(q,q[1:]))
            if after>old+1e-8: continue
            if abs(after-old)<1e-6 and LineString(q).hausdorff_distance(LineString(pts))<1e-6: continue
            changed.update(ids); info['geaendert'] += 1; info['gratis_mm'] += old-after
            added.append(dict(edges[ids[0]]['meta'], pts=[list(p) for p in q], w=[w]*(len(q)-1)))
        if not changed: continue
        removed.update(i for i,_ in tracks)


        for i,e in enumerate(edges):
            if i not in changed: added.append(dict(e['meta'], pts=[list(e['a']),list(e['b'])], w=[e['w']]))
        additions.extend(added)
        new['tracks'] = [t for i,t in enumerate(R['tracks']) if i not in removed] + additions
        corridors[L] = r24.Corridor(B,new,L)
    info['gratis_mm'] = round(info['gratis_mm'],6)
    return new,info
