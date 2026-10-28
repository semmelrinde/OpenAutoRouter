""
import copy
import math

from shapely.geometry import Polygon
from analyse.bewertung import vergleich
from analyse.topologie import topologie
from analyse.kopplung import kopplung
from gemeinsam.verschieben import bewegen

TOPO = ('parallel_rest_mm', 'stern', 'spitz', 'kelvin_nah_mm', 'abgriff_via_im_pad')


def sperre(B, geo, eigene=()):
    ""
    for k in B.get('keepouts', []):
        if k.get('ref') in eigene: continue
        if not k.get('ref') and not (k.get('verbote') or {}).get('teile', True): continue
        lagen = k.get('layers') or k.get('lagen') or ['F', 'B']
        lagen = {s.replace('.Cu', '') for s in lagen}
        for s, g in geo.items():
            if s not in lagen and '*.Cu' not in lagen and '*' not in lagen: continue
            if any(g.intersects(Polygon(q)) for q in k['polys'] if len(q) >= 3): return True
    return False


def regelwerte(B, Rt, K):
    t = topologie(Rt, B)
    return dict(topologie={n: t[n] for n in TOPO}, kopplung=kopplung(Rt, K)['summe'])


def annahme(E0, E1, B0, R0, B1, R1, K, mindest=2.0, preis=0.0, note='v3'):
    ""
    if note == 'v3' and E0.get('note_v3_version') != E1.get('note_v3_version'):
        return dict(delta=None, besser=False, grund='verschiedene Note-v3-Versionen', note=note)
    c = vergleich(E0, E1, B0, R0, B1, R1, K, note=note)
    a = E0.get('zug_regeln') or regelwerte(B0, R0, K)
    b = E1.get('zug_regeln') or regelwerte(B1, R1, K)
    schlechter = [n for n in TOPO if b['topologie'][n] > a['topologie'][n] + 1e-6]
    if b['kopplung'] > a['kopplung'] + 1e-6: schlechter.append('kopplung')
    c.update(regeln_schlechter=schlechter, preis=preis)
    if not c['besser'] or schlechter:
        c['besser'] = False
        if schlechter: c['grund'] = 'Regeln schlechter: ' + ', '.join(schlechter)
        return c


    if c.get('note') != 'minimax' and c['delta'] is not None and c['delta'] + preis >= -mindest:
        c.update(besser=False, grund='Gewinn reicht nicht fuer Mindestgewinn und Bewegungspreis')
    return c


def bewegung(B, k):
    ""
    return bewegen(B, k['refs'], seiten=k.get('seiten'))[0]


def octi(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return math.dist(a, b) > 1e-6 and (min(dx, dy) < 1e-6 or abs(dx - dy) < 1e-6)


def routenzug(Rt, k):
    ""
    R = copy.deepcopy(Rt)
    for m in k.get('route_moves', []):
        obj = R[m['feld']][m['index']]
        if any(obj.get(f) != v for f, v in m['alt'].items()): raise ValueError('lokaler Zug passt nicht mehr zum Zustand')
        obj.update(copy.deepcopy(m['neu']))
    return R
