""
import math, re

VERSION = 'stoerer-v1'


def tr_von(n, K, gewichte=None):
    from analyse.metriken import klasse, lade_gewichte
    G = gewichte if gewichte is not None else lade_gewichte()
    for rx, value in sorted(((K.get('vorgaben') or {}).get('tr_ns') or {}).items()):
        try:
            if n == rx or n.split('/')[-1] == rx or re.fullmatch(rx, n.split('/')[-1]):
                return float(value) if value is not None else None
        except re.error: pass
    k = K['netze'].get(n) or {}

    value = k['tr_ns'] if 'tr_ns' in k else (G['klassen'].get(klasse(k), {}) or {}).get('tr_ns')
    return float(value) if value is not None else None


def abstand_faktor(tr_ns):
    return math.sqrt(max(0.0, 26.0 * 2.0 / max(float(tr_ns), 0.1) - 1.0)) / 5.0

def erstellen(K, gewichte=None):
    from analyse.metriken import lade_gewichte
    G = gewichte if gewichte is not None else lade_gewichte()
    netze = {}; digital = []; strom = []; sw = []
    for n, k in sorted(K['netze'].items()):
        art = k.get('art'); tr = tr_von(n, K, G)
        if art == 'gnd': continue
        d = art in ('signal', 'schnell') and tr is not None and 0 < tr <= 5.0
        if d: digital.append(n)
        if art in ('strom', 'versorgung'): strom.append(n)
        if art == 'sw': sw.append(n)
        if tr is not None and tr > 0 or art in ('strom', 'versorgung', 'sw'):
            netze[n] = dict(art=art, tr_ns=tr, digital=d, abstand_faktor=abstand_faktor(tr) if d else 1.0)
    opfer = sorted(n for n, k in K['netze'].items() if k.get('art') == 'analog' or k.get('kelvin') or k.get('empfindlich_sw'))
    spulen = sorted(r for r, t in K.get('teile', {}).items() if t.get('funktion') == 'spule_sw')
    return dict(version=VERSION, netze=netze, digital=digital, strom=strom, sw=sw, opfer=opfer, spulen=spulen)


def lesen(K, gewichte=None):
    ""
    S = K.get('stoerer')
    aktuell = erstellen(K, gewichte)
    return S if S == aktuell else aktuell


def flanken(K, gewichte=None):
    S = lesen(K, gewichte)
    return {n: (S['netze'].get(n) or {}).get('tr_ns') for n in K['netze']}
