""
import json, sys, os, math, random, collections, time
import fnmatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gemeinsam.prozessschutz import bootstrap
if __name__ == "__main__": bootstrap(heavy=True)
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely import affinity
from gemeinsam.teile import Belegung, lade_teile, platine, GEGEN, sperrflaechen, hoehe
from gemeinsam.werte import typ
from gemeinsam import wanderweite as WWM
from analyse import dichte as DM
from platzierung.insel_physik import InselRegeln

t_start = time.time()
B = json.load(open(sys.argv[1])); K = json.load(open(sys.argv[2])); OUT = sys.argv[3]
args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
from lernen.meister import anwenden as _meister_anwenden
MEISTER_BELEG = _meister_anwenden(args, B, 'placer')
RG = json.load(open(args['regeln'])) if 'regeln' in args else None
DRCL = str(args.get('drc_modell', (RG or {}).get('placer', {}).get('drc_modell', '0'))) == '1'
PADABST = float(args.get('padabst', (RG or {}).get('placer', {}).get('padabst_drc' if DRCL else 'padabst', 0.15)))
RAND = float(args.get('rand', (RG or {}).get('placer', {}).get('rand', 0.2)))
SEITEN = args.get('seiten', 'FB')
VIA = float(args.get('via_kosten', '0.6'))
TRENN = float(args.get('trenn', '3.0'))
MODUS = args.get('modus', 'neu')
if args.get('p35', '0') not in ('0', '1'): raise ValueError('p35=0|1')
P35 = args.get('p35', '0') == '1'
P35_KARTE = None; P35_BERICHT = None
if P35:
    from platzierung.p35_platz import Karte as P35Karte, lokal as p35_lokal, positiv as p35_positiv
    P35_W = p35_positiv(args.get('p35_w', '4'), 'p35_w')
random.seed(int(args.get('seed', '1'))); np.random.seed(int(args.get('seed', '1')))

ROUTBARKEIT = None
if args.get('routbarkeit', '0') == '1':
    from lernen.ki.routbarkeit_dienst import Dienst as RoutDienst
    if args.get('routbarkeit_endpruefung') != '1':
        raise ValueError('routbarkeit benötigt unabhängige Endprüfung: routbarkeit_endpruefung=1')
    ROUTBARKEIT = RoutDienst(args.get('routbarkeit_modell'), einsatz='placer')
    if ROUTBARKEIT.predictor is None: raise ValueError('Routbarkeits-Kostenterm gesperrt: ' + ROUTBARKEIT.grund)
    RW = float(args.get('routbarkeit_w', '1'))
    if not math.isfinite(RW) or RW < 0: raise ValueError('routbarkeit_w')

NETZ = K['netze']; GND = set(K['gnd']); TEILE = K['teile']; INSELN = K['inseln']; VG = K.get('vorgaben') or {}
DICHT = float(args.get('dicht', '0'))
if not math.isfinite(DICHT) or DICHT < 0: raise ValueError('dicht muss endlich und >= 0 sein')
if VG.get('umriss_fest', False): DICHT = 0.0
DICHT_INSELN = float(args.get('dicht_inseln', '0'))
if not math.isfinite(DICHT_INSELN) or DICHT_INSELN < 0: raise ValueError('dicht_inseln muss endlich und >= 0 sein')
DICHT_GRUPPEN = args.get('dicht_gruppen', '0')
if DICHT_GRUPPEN not in ('0', '1'): raise ValueError('dicht_gruppen muss 0 oder 1 sein')
DICHT_GRUPPEN = DICHT_GRUPPEN == '1'
DICHT_INNEN = float(args.get('dicht_innen', '0'))
if not math.isfinite(DICHT_INNEN) or DICHT_INNEN < 0: raise ValueError('dicht_innen muss endlich und >= 0 sein')
fps = {f['ref']: f for f in B['fps']}
if MODUS == 'neu' and args.get('bedien_neu_frei', '0') == '1':


    for r, t in TEILE.items():
        if t.get('bedien') and not fps[r].get('locked') and not any(fnmatch.fnmatchcase(r, p) for p in VG.get('fest', [])):
            t['fest'] = False
            if r in INSELN: INSELN[r]['fest'] = False
if DRCL:
    geometry_board = dict(B, drc_modell=True)
else:

    geometry_board = dict(B, pads=[{k: v for k, v in p.items() if k != 'hole_polys'} for p in B['pads']], outline_holes=[])
T = lade_teile(geometry_board, PADABST)
KUPFERRESERVE = []
if args.get('kupferreserve') == '1':
    from platzierung.kupferreserve import anwenden as reserve_anwenden
    KUPFERRESERVE = reserve_anwenden(T, B, K, RG)
PHYSIK = args.get('insel_physik', '0') == '1'
PHYS = InselRegeln(T, K, B) if PHYSIK or args.get('insel_messung') == '1' else None
BGA_GEGEN = args.get('bga_gegen', '0') == '1' and MODUS == 'neu' and args.get('seiten_original') != '1'
POWER = {}; POWER_OF = {}; POWER_REFIT = set(); PIN_NACHLEGAL = set()
board, inner, inner_p, _ = platine(geometry_board, RAND)
DICHT_NORM = max(1.0, math.hypot(board.bounds[2] - board.bounds[0], board.bounds[3] - board.bounds[1]))
PADRAND = None
if PHYSIK and args.get('pad_rand_exakt','0') == '1':
    from platzierung.pad_rand import PadRand
    PADRAND = PadRand(PHYS.polys,fps,board,float((RG or {}).get('router',{}).get('edge',RAND)))

KEEP = [(k['ref'], Polygon(q).buffer(0)) for k in B.get('keepouts', []) if k['ref'] for q in k['polys'] if len(q) >= 3]


def wnet(n):
    k = NETZ.get(n)
    if not k or n in GND: return 0.0
    a = k['art']
    if a == 'schnell': return 2.0 + 0.5 * k['rang']
    if a == 'sw': return 4.0
    if a == 'strom': return {'hoch': 3.0, 'mittel': 2.5, 'klein': 1.5}[k['strom']]
    if a == 'versorgung': return {'hoch': 1.0, 'mittel': 0.6, 'klein': 0.5}[k['strom']]
    if a == 'analog': return 1.5 + 0.5 * k['rang']
    return 1.0 + 0.3 * k.get('rang', 0)


def via_k(a, b): return 0.0 if a == b or 'FB' in (a, b) else VIA


DIRS = [(math.cos(2 * math.pi * k / 16), math.sin(2 * math.pi * k / 16)) for k in range(16)]
SPIRALE = sorted([(i * 0.1, j * 0.1, math.hypot(i, j) * 0.1) for i in range(-100, 101) for j in range(-100, 101)
                   if math.hypot(i, j) <= 100] +
                  [(i * 0.25, j * 0.25, math.hypot(i, j) * 0.25) for i in range(-160, 161) for j in range(-160, 161)
                   if 40 < math.hypot(i, j) <= 160], key=lambda t: t[2])




SPERR_AN = args.get('sperr', '1') == '1'
SPERR = sperrflaechen(B) if SPERR_AN else {'F': None, 'B': None}
SPERR_BB = {s: g.bounds for s, g in SPERR.items() if g is not None}
SPERR_H = []
if SPERR_AN:
    for z in VG.get('sperr_hoehe') or []:
        g = unary_union([Polygon(q).buffer(0) for q in z.get('polys', []) if len(q) >= 3]).buffer(0.005)
        for s in z.get('seite', 'FB'):
            if s in ('F', 'B') and not g.is_empty: SPERR_H.append((s, float(z.get('hoehe_max', 0.0)), g))
HOEHE = {r: hoehe(fps[r].get('fp', ''), r, VG.get('hoehen')) for r in T}
_SPK = {}


def sperr_von(r):
    ""
    h = HOEHE.get(r, 1.5) if SPERR_H else None
    if h not in _SPK:
        out = {}
        for s in ('F', 'B'):
            gs = ([SPERR[s]] if SPERR.get(s) is not None else []) + [g for ss, hm, g in SPERR_H if ss == s and h > hm]
            if gs:
                u = unary_union(gs) if len(gs) > 1 else gs[0]
                out[s] = (u, u.bounds)
        _SPK[h] = out
    return _SPK[h]

def sperr_flaeche(r, geo):
    ""
    sp = sperr_von(r)
    if not sp: return 0.0
    a = 0.0
    for s, g in geo.items():
        if s not in sp: continue
        k, kb = sp[s]; b = g.bounds
        if b[0] < kb[2] and kb[0] < b[2] and b[1] < kb[3] and kb[1] < b[3]: a += g.intersection(k).area
    return a



KPL = collections.defaultdict(list); KPLK = collections.defaultdict(list)
for _kr, _g in KEEP:
    if _kr in fps:
        _l = affinity.translate(_g, -fps[_kr]['x'], -fps[_kr]['y'])
        KPL[_kr].append(_l); KPLK[_kr].append(_l.buffer(-0.01))


def kp_abs(kr, s, klein=False):
    return [affinity.translate(affinity.rotate(g, s[2], origin=(0, 0)), s[0], s[1]) for g in (KPLK if klein else KPL)[kr]]


WT = WWM.tabelle(K)
WW_AN = MODUS == 'verfeinern' and args.get('wanderweite', '1') == '1'
WW_F = float(args.get('ww_f', '1.0'))
DREHEN_REGEL = args.get('drehen_regel', '1') == '1'
STUETZT = {}
for _r, _t in TEILE.items():
    _s = (_t.get('stuetzt') or {}).get('ref')
    if _r in T and _s in T and _s != _r: STUETZT[_r] = _s


def darf_drehen(r):
    return TEILE.get(r, {}).get('rolle') == 'klein' and (not DREHEN_REGEL or WT.get(r, {}).get('drehen', True))


def weg(r, s, lage, ch=None):
    ""
    x0, y0 = fps[r]['x'], fps[r]['y']
    c = STUETZT.get(r)
    if c is not None and (c in lage or (ch and c in ch)):
        sc = ch[c] if ch and c in ch else lage[c]
        x0 += sc[0] - fps[c]['x']; y0 += sc[1] - fps[c]['y']
    return math.hypot(s[0] - x0, s[1] - y0)


PIN_LEGAL = args.get('pin_legal', '1') == '1'
QUARZ_PIN = args.get('quarz_pin', '1') == '1'
PIN = {}
for _a, _I in INSELN.items():
    for _e in _I['entkopplung']:
        if not _e.get('ueber_spule') and _e['ref'] in T and _a in T: PIN[_e['ref']] = (_a, _e['netz'], set(_e['pins']))
PIN_ENT = dict(PIN)
if PHYSIK:
    PIN.update(PHYS.pin)
    PIN_ENT = {r: v for r,v in PIN.items() if typ(r,fps[r].get('fp','')) == 'C'}
P3_SCHLEIFE = args.get('p3_schleife', '0') == '1'
if P3_SCHLEIFE:
    for _c, _z in (K.get('p3_pin_gruppen') or {}).get('teile', {}).items():
        PIN[_c] = PIN_ENT[_c] = (_z['ref'], _z['netz'], set(_z['pins']))
    from platzierung.entkopplung import schleife as p3_schleife
if QUARZ_PIN:
    for _r, _t in sorted(TEILE.items()):
        _s = _t.get('stuetzt') or {}
        if _t.get('funktion') == 'quarz_last' and _r in T and _s.get('quarz') in T and _r not in PIN:
            PIN[_r] = (_s['quarz'], _s.get('netz'), None)
PIN_OF = collections.defaultdict(list)
for _c, (_z, _n, _p) in sorted(PIN.items()): PIN_OF[_z].append(_c)


def pin_abst(cpads, zpads, n, pins, ref=None):
    cp = [(x, y, sd) for nr, net, x, y, sd in cpads if net == n]
    zp = [(x, y, sd) for nr, net, x, y, sd in zpads if (nr in pins if pins is not None else net == n)]
    if not cp or not zp: return 0.0
    alt = 4.0 * min(math.hypot(p[0] - q[0], p[1] - q[1]) + via_k(p[2], q[2]) for p in cp for q in zp)
    if P3_SCHLEIFE and ref in PIN_ENT:
        loop = p3_schleife(cpads, zpads, n, pins, GND, via_k)
        if loop is not None:
            C = TEILE.get(ref, {}).get('C') or 100e-9
            w = 1.0 if C <= 220e-9 * 1.001 else (.4 if C <= 4.7e-6 * 1.001 else .1)
            return alt + w * loop
    return alt



def baue_gruppen():
    if args.get('gruppen', '1') != '1': return {}
    nts = set(K.get('net_ties') or [])
    by = collections.defaultdict(list)
    for p in B['pads']: by[p['ref']].append(p)
    out = {}
    for n in sorted(nts):
        if n not in T or n not in fps: continue
        best = None
        for p in by[n]:
            if not p['net']: continue
            for q in B['pads']:
                if q['ref'] == n or q['ref'] in nts or q['ref'] not in T or q['net'] != p['net']: continue
                d = math.hypot(p['x'] - q['x'], p['y'] - q['y'])


                k = (p['net'] in GND, -T[q['ref']].n_pads, d, q['ref'])
                if d <= 1.0 and (best is None or k < best[0]): best = (k, q['ref'])
        if best is not None:
            l = best[1]
            out[n] = (l, fps[n]['x'] - fps[l]['x'], fps[n]['y'] - fps[l]['y'])
    return out


FOLGE = baue_gruppen()
GRUPPE = collections.defaultdict(list)
for _f, (_l, _dx, _dy) in sorted(FOLGE.items()): GRUPPE[_l].append(_f)
FEST_EXTRA = {_l for _f, (_l, _dx, _dy) in FOLGE.items() if TEILE.get(_f, {}).get('fest')}
if FOLGE: print('Starre Gruppen (Net-Tie am Fusspunkt):', ', '.join('%s->%s' % (f, l) for f, (l, _, _) in sorted(FOLGE.items())))


def mates(r):
    l = FOLGE[r][0] if r in FOLGE else r
    return {l, *GRUPPE.get(l, ())}


def folge(f, sl):
    l, dx, dy = FOLGE[f]; X, Y, TH, S = sl
    sf = fps[f]['side']
    if S != fps[l]['side']: dx = -dx; sf = GEGEN[sf]
    c, s = math.cos(math.radians(TH)), math.sin(math.radians(TH))
    return (X + c * dx - s * dy, Y + s * dx + c * dy, TH, sf)


def gruppen_drehbar(r, chip=False):
    ""
    if not (darf_drehen(r) or chip): return False
    return all(not TEILE.get(q, {}).get('fest') and
               (not DREHEN_REGEL or WT.get(q, {}).get('drehen', True))
               for q in bewegungs_refs(r))


def lage_zurueck(lage, gespeichert):
    ""
    lage.clear(); lage.update(gespeichert)
    return {r: kp_abs(r, lage[r], klein=True) for r in KPLK if r in lage}


def mit_gruppe(r, s, lage=None):
    if lage is not None and (s[2] - lage[r][2]) % 360 and not gruppen_drehbar(r, chip=True):
        raise ValueError('Gruppendrehung verboten: ' + r)
    out = {r: s}
    for f in GRUPPE.get(r, ()): out[f] = folge(f, s)
    if lage is not None and r in POWER:
        dr = s[2]-lage[r][2]
        c, sn = math.cos(math.radians(dr)), math.sin(math.radians(dr))
        for q in POWER[r]:
            if q == r or q not in lage: continue
            x, y = lage[q][0]-lage[r][0], lage[q][1]-lage[r][1]
            out[q] = (s[0]+c*x-sn*y, s[1]+sn*x+c*y, (lage[q][2]+dr)%360, lage[q][3])
    return out


def bewegungs_refs(r):
    if r not in POWER_OF and r not in POWER: return mates(r)
    a = POWER_OF.get(r, r)
    return set(POWER.get(a, [a])) | mates(a)


if args.get('seiten_board', '1') == '1' and len(SEITEN) > 1 and args.get('seiten_original') != '1':
    _auf = [r for r in T if T[r].n_pads and board.contains(Point(fps[r]['x'], fps[r]['y']))]
    _sd = sorted({fps[r]['side'] for r in _auf})
    if len(_auf) >= 2 and len(_sd) == 1:
        print('Seiten: Platine einseitig bestueckt (%s), seiten=%s -> %s (P16; seiten_board=0 schaltet ab)' % (_sd[0], SEITEN, _sd[0]))
        SEITEN = _sd[0]


RANG = args.get('rang', '1') == '1'
RANG_P20 = {'quarz': 3.0, 'quarz_last': 3.5, 'serien_r': 4.0, 'entkopplung_mittel': 5.0, 'bootstrap': 5.0, 'rueckkopplung': 5.5,
            'rueckkopplung_c': 5.5, 'einstell_r': 5.5, 'shunt': 5.5, 'entkopplung_gross': 6.0, 'sonstig': 6.0, 'teiler': 6.5,
            'rc_glied': 6.5, 'pullup': 7.0, 'pulldown': 7.0}
GROB = VG.get('grob') or {}

def feder_teil(r, s):
    ""
    g = GROB.get(r)
    if not g: return 0.0
    d = math.hypot(s[0] - g.get('x', s[0]), s[1] - g.get('y', s[1]))
    return 2.0 * max(0.0, d - g.get('radius', 0.5)) ** 2



TRENN_W = float(args.get('trenn_w', '1.0')); TRENN_PAD = float(args.get('trenn_pad', '2.0'))
AGG = {}; VIC = set()
for _n, _k in NETZ.items():
    _a = _k.get('art')
    if _a == 'sw': AGG[_n] = 2.0
    elif _a == 'schnell' and _k.get('rang', 0) >= 3: AGG[_n] = 1.0
    if (_a == 'analog' and _k.get('rang', 0) >= 2) or _k.get('kelvin') or _k.get('empfindlich_sw'): VIC.add(_n)
_gg = args.get('gnd_gegen', 'auto')
GND_GEGEN = _gg == '1' or (_gg == 'auto' and str(((RG or {}).get('router') or {}).get('vip', '0')) == '1')
try: _vd = float(str(((RG or {}).get('router') or {}).get('via', '0.45/0.2')).split('/')[0])
except ValueError: _vd = 0.45
GG_RV = float(args.get('gnd_via_r', str(_vd / 2 + float(((RG or {}).get('router') or {}).get('clr', 0.1)))))
GG_W = float(args.get('gnd_gegen_w', '2.0'))
if args.get('bga_via_gegen','0') not in ('0','1'): raise ValueError('bga_via_gegen=0|1')
BGA_VIA_GEGEN = args.get('bga_via_gegen','0') == '1'
if BGA_VIA_GEGEN and str(((RG or {}).get('router') or {}).get('vip','0')) != '1':
    raise ValueError('bga_via_gegen braucht explizites vip=1 im Boardvertrag')
BGA_VIA_W = float(args.get('bga_via_w','80'))
if BGA_VIA_GEGEN and (not math.isfinite(BGA_VIA_W) or BGA_VIA_W <= 0): raise ValueError('bga_via_w muss endlich und positiv sein')
BGA_VIA_R = _vd/2 + max(float(((RG or {}).get('router') or {}).get('drcpad',((RG or {}).get('router') or {}).get('drc',.1))),
                         float(((RG or {}).get('router') or {}).get('clr',.1)))
def bga_via_pins(board, footprints, nets, grounds, rules):
    ""
    chosen=set();rg=(rules or {}).get('router') or {}
    for ref,f in footprints.items():
        if 'BGA' not in f.get('fp','').upper():continue
        pads=[p for p in board['pads'] if p['ref']==ref and not p.get('drill')]
        if len(pads)<16:continue
        xy=np.array([(p['x'],p['y']) for p in pads]);dd=np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)
        np.fill_diagonal(dd,np.inf);near=dd.argmin(axis=1);pitch=float(np.median(dd[np.arange(len(xy)),near]))
        if not .3<=pitch<=1.05:continue
        direction=xy[near]-xy;angle=float(np.angle(np.mean(np.exp(4j*np.arctan2(direction[:,1],direction[:,0])))))/4
        rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        local=xy@rot;origin=local.min(axis=0);ij=np.rint((local-origin)/pitch).astype(int)
        if np.max(np.abs(local-origin-ij*pitch))>.02:continue
        mx,my=ij.max(axis=0)
        for p,(i,j) in zip(pads,ij):
            net=p.get('net')
            if not net or net.startswith('unconnected'):continue
            bb=p['bbox'];ring=min(i,j,mx-i,my-j)
            minimum=1 if pitch-max(bb[2]-bb[0],bb[3]-bb[1])-2*float(rg.get('drcpad',rg.get('drc',.1))) < float(rg.get('sigmin',.1))-1e-9 else 2
            if ring>=minimum or p.get('pintype')=='power_in' or net in grounds or nets.get(net,{}).get('art') in ('versorgung','strom'):
                chosen.add((ref,str(p['num'])))
    return chosen

BGA_VIA_PADS = bga_via_pins(B,fps,NETZ,GND,RG) if BGA_VIA_GEGEN else set()
AUSF = float(args.get('ausfaecher', '0')); AUSF_W = float(args.get('ausfaecher_w', '0.5'))
SIDX = {'F': 0, 'B': 1, 'FB': 2}


def _f_trenn(wa, gew):
    def f(a, b):
        d = np.hypot(a['x'][:, None] - b['x'][None, :], a['y'][:, None] - b['y'][None, :])
        return gew * wa[a['i']][:, None] * np.maximum(0.0, TRENN_PAD - d) ** 2
    return f


def _f_gg(gew):
    def f(a, b):
        ""
        gegen = (b['sd'][None, :] == 2) | (b['sd'][None, :] == 1 - a['sd'][:, None])
        dx = np.maximum(np.abs(a['x'][:, None] - b['x'][None, :]) - b['hx'][None, :], 0.0)
        dy = np.maximum(np.abs(a['y'][:, None] - b['y'][None, :]) - b['hy'][None, :], 0.0)
        return gew * np.where(gegen, np.maximum(0.0, GG_RV - np.hypot(dx, dy)) / GG_RV, 0.0)
    return f


def _f_ausf(a, b):
    ""
    gleich = (a['sd'][:, None] == b['sd'][None, :]) | (a['sd'][:, None] == 2) | (b['sd'][None, :] == 2)
    dx = np.maximum(np.abs(a['x'][:, None] - b['x'][None, :]) - a['hx'][:, None] - b['hx'][None, :], 0.0)
    dy = np.maximum(np.abs(a['y'][:, None] - b['y'][None, :]) - a['hy'][:, None] - b['hy'][None, :], 0.0)
    return AUSF_W * np.where(gleich, np.maximum(0.0, AUSF - np.hypot(dx, dy)) ** 2, 0.0)


def _f_kanal(netz, radius, gew, via=False):
    def f(a, b):
        same = (a['sd'][:, None] == b['sd'][None, :]) | (a['sd'][:, None] == 2) | (b['sd'][None, :] == 2)
        if via: same = ~same | (b['sd'][None, :] == 2)
        fremd = netz[a['i']][:, None] != netz[b['i']][None, :]
        dx = np.maximum(np.abs(a['x'][:, None]-b['x'][None, :])-b['hx'][None, :], 0)
        dy = np.maximum(np.abs(a['y'][:, None]-b['y'][None, :])-b['hy'][None, :], 0)
        if not via:
            dx = np.maximum(dx-a['hx'][:, None], 0); dy = np.maximum(dy-a['hy'][:, None], 0)
        return gew * np.where(same & fremd, np.maximum(0, radius[a['i']][:, None]-np.hypot(dx,dy))**2, 0)
    return f


class Paare:
    ""
    def __init__(self, lage, alle=False):
        self.refs = sorted(r for r in lage if T[r].n_pads)
        self.idx = {}; rows = []
        for r in self.refs:
            self.idx[r] = np.arange(len(rows), len(rows) + T[r].n_pads)
            rows += [(r, k) for k in range(T[r].n_pads)]
        self.N = len(rows)
        rid = {r: i for i, r in enumerate(self.refs)}
        self.ref = np.array([rid[r] for r, k in rows], int)
        net = [T[r].pads[k][1] for r, k in rows]
        tht = np.array([T[r].pads[k][4] for r, k in rows], bool)
        rolle = [TEILE.get(r, {}).get('rolle') for r, k in rows]
        self.X = np.zeros(self.N); self.Y = np.zeros(self.N); self.HX = np.zeros(self.N); self.HY = np.zeros(self.N)
        self.SD = np.zeros(self.N, int)
        for r in self.refs: self.setze(r, lage[r])
        self.terme = []
        if TRENN_W > 0 or alle:
            A = np.array([n in AGG for n in net], bool); Bm = np.array([n in VIC for n in net], bool)
            self._term('trenn', A, Bm, _f_trenn(np.array([AGG.get(n, 0.0) for n in net]), 1.0 if alle else TRENN_W))
        if GND_GEGEN or alle:
            self._term('gnd_gegen', np.array([n in GND for n in net], bool) & ~tht, np.array([n not in GND for n in net], bool),
                       _f_gg(1.0 if alle else GG_W))
        if AUSF > 0:
            self._term('ausfaecher', np.array([x == 'chip' for x in rolle], bool), np.array([x == 'stecker' for x in rolle], bool), _f_ausf)
        if BGA_VIA_GEGEN:
            bga = np.array([(r,str(T[r].pads[k][0])) in BGA_VIA_PADS for r,k in rows],bool) & ~tht
            self._term('bga_via_frei',bga,np.array([bool(n) for n in net],bool),
                       _f_kanal(np.array(net),np.full(self.N,BGA_VIA_R),BGA_VIA_W,via=True))
        if PHYSIK:
            live = np.array([bool(n) and n not in GND for n in net], bool)
            cpin = np.array([r in PIN_ENT for r, _ in rows], bool) & ~tht
            self._term('c_via_frei', cpin, np.array([bool(n) for n in net], bool),
                       _f_kanal(np.array(net), np.full(self.N, GG_RV), 20.0, via=True))

            rr = np.array([float(NETZ.get(n, {}).get('w', 0.1) or 0.1) + PADABST for n in net])
            self._term('kanal', live, live, _f_kanal(np.array(net), rr, float(args.get('kanal_w', '0.25'))))

    def _term(self, name, A, Bm, f):
        if not A.any() or not Bm.any(): return
        hat = {r: (bool(A[i].any()), bool(Bm[i].any())) for r, i in self.idx.items()}
        self.terme.append(dict(name=name, A=A, B=Bm, Ai=np.nonzero(A)[0], Bi=np.nonzero(Bm)[0], f=f, hat=hat))

    def werte(self, r, s):
        pa = T[r].pads_abs(*s); ph = T[r].pads_halb(s[2])
        return (np.array([p[2] for p in pa]), np.array([p[3] for p in pa]), np.array([h[0] for h in ph]),
                np.array([h[1] for h in ph]), np.array([SIDX[p[4]] for p in pa], int))

    def setze(self, r, s):
        i = self.idx.get(r)
        if i is None or not len(i): return
        self.X[i], self.Y[i], self.HX[i], self.HY[i], self.SD[i] = self.werte(r, s)

    def _rows(self, i):
        return dict(i=i, x=self.X[i], y=self.Y[i], hx=self.HX[i], hy=self.HY[i], sd=self.SD[i], ref=self.ref[i])

    @staticmethod
    def _sum(f, a, b):
        if not len(a['i']) or not len(b['i']): return 0.0
        M = f(a, b)
        return float(np.where(a['ref'][:, None] != b['ref'][None, :], M, 0.0).sum())

    def beitrag(self, ch):
        ""
        if not self.terme: return 0.0
        refs = [r for r in ch if r in self.idx and len(self.idx[r])]
        tot = 0.0; C = None
        for t in self.terme:
            ha = any(t['hat'][r][0] for r in refs); hb = any(t['hat'][r][1] for r in refs)
            if not ha and not hb: continue
            if C is None:
                ich = np.concatenate([self.idx[r] for r in refs])
                v = [self.werte(r, ch[r]) for r in refs]
                C = dict(i=ich, x=np.concatenate([q[0] for q in v]), y=np.concatenate([q[1] for q in v]),
                         hx=np.concatenate([q[2] for q in v]), hy=np.concatenate([q[3] for q in v]),
                         sd=np.concatenate([q[4] for q in v]), ref=self.ref[ich])
                inch = np.zeros(self.N, bool); inch[ich] = True
            ma = t['A'][C['i']]; mb = t['B'][C['i']]
            sub = lambda m: {k: v_[m] for k, v_ in C.items()}
            if ha:
                rb = t['Bi'][~inch[t['Bi']]]
                tot += self._sum(t['f'], sub(ma), self._rows(rb))
            if hb:
                ra = t['Ai'][~inch[t['Ai']]]
                tot += self._sum(t['f'], self._rows(ra), sub(mb))
            if ha and hb: tot += self._sum(t['f'], sub(ma), sub(mb))
        return tot

    def gesamt(self):
        ""
        out = {}
        for t in self.terme:
            a = self._rows(t['Ai']); b = self._rows(t['Bi'])
            out[t['name']] = round(self._sum(t['f'], a, b), 2)
            if t['name'] == 'trenn':
                d = np.hypot(a['x'][:, None] - b['x'][None, :], a['y'][:, None] - b['y'][None, :])
                d = np.where(a['ref'][:, None] != b['ref'][None, :], d, np.inf)
                out['trenn_min_mm'] = round(float(d.min()), 2) if d.size else None
            if t['name'] == 'gnd_gegen':
                M = _f_gg(1.0)(a, b); M = np.where(a['ref'][:, None] != b['ref'][None, :], M, 0.0)
                out['gnd_gegen_belegt'] = int((M.max(axis=1) > 0).sum()) if M.size else 0
            if t['name'] in ('c_via_frei','bga_via_frei'):
                M = t['f'](a,b); M = np.where(a['ref'][:, None] != b['ref'][None,:], M, 0)
                out['c_via_belegt' if t['name']=='c_via_frei' else 'bga_via_belegt'] = int((M.max(axis=1) > 0).sum()) if M.size else 0
        return out



def insel_bauen(a, I):
    S = fps[a]['side']
    bel = Belegung(); st = {a: (0.0, 0.0, 0.0, S)}
    bel.add(a, T[a].geo(0, 0, 0, S))
    placed = collections.defaultdict(list)
    def add_pads(r):
        for nr, net, x, y, s in T[r].pads_abs(*st[r]): placed[net].append((x, y, s, r))
    def del_pads(r):
        for net in list(placed):
            placed[net] = [q for q in placed[net] if q[3] != r]
    add_pads(a)
    agnd = [(x, y, s) for nr, net, x, y, s in T[a].pads_abs(0, 0, 0, S) if net in GND]

    unter = T[a].geo(0, 0, 0, S)[S].buffer(0.2)
    ent = {e['ref']: e for e in I['entkopplung'] if e['ref'] in T}

    apads = {nr: (x, y, s) for nr, net, x, y, s in T[a].pads_abs(0, 0, 0, S)}
    grp = {}
    for e in ent.values():
        if e['netz'] in grp: continue
        pins = [apads[n] for n in e['pins'] if n in apads]
        cl = []
        for p in pins:
            for c in cl:
                if any(math.hypot(p[0] - q[0], p[1] - q[1]) < 1.0 for q in c): c.append(p); break
            else: cl.append([p])
        cl.sort(key=lambda c: -len(c))
        grp[e['netz']] = [dict(pins=c, n=0) for c in cl]
    ziel = {}
    mem = [r for r in I['mitglieder'] if r in T and not TEILE[r]['fest'] and r not in FOLGE]
    def ist_sw(r): return typ(r, fps[r].get('fp', '')) == 'L' or any(NETZ.get(net, {}).get('art') == 'sw' for _, net, *_ in T[r].pads)
    e1 = [r for r in mem if r in ent and not ent[r].get('ueber_spule')]
    e1.sort(key=lambda r: ent[r]['C'])
    sw = [r for r in mem if r not in ent and ist_sw(r)]
    if PHYSIK: sw.sort(key=lambda r: typ(r,fps[r].get('fp','')) != 'L')
    e2 = sorted([r for r in mem if r in ent and ent[r].get('ueber_spule')], key=lambda r: ent[r]['C'])
    rest = [r for r in mem if r not in e1 and r not in sw and r not in e2]
    if RANG:


        def rg(r): return RANG_P20.get(TEILE[r].get('funktion'), 6.0)
        rest = sorted(rest, key=rg)
        if sw: order = e1 + sw + e2 + rest
        else:
            fr = [r for r in rest if rg(r) <= 4.0]
            e1k = [r for r in e1 if TEILE[r].get('funktion') in ('entkopplung_klein', None)]
            order = e1k + fr + [r for r in e1 if r not in e1k] + e2 + [r for r in rest if r not in fr]
    else:
        order = e1 + sw + e2 + rest

    def ziel_pins(r):


        if PHYSIK and r in PHYS.pin:
            z,n,pins = PHYS.pin[r]
            if z in st: return [(x,y,sd) for num,net,x,y,sd in T[z].pads_abs(*st[z]) if num in pins]
        return (ziel.get(r) or {}).get('pins', [])

    def kosten(r, x, y, th, side):
        c = 0.0
        isent = r in ent
        for nr, net, px, py, ps in T[r].pads_abs(x, y, th, side):
            if net in GND:
                if agnd: c += (0.4 if isent else 0.1) * min(math.hypot(px - gx, py - gy) + via_k(ps, gs) for gx, gy, gs in agnd)
                continue
            zp = ziel_pins(r)
            if zp and net == (PIN[r][1] if PHYSIK and r in PHYS.pin else ent[r]['netz']):
                c += 4.0 * min(math.hypot(px - qx, py - qy) + via_k(ps, qs) for qx, qy, qs in zp)
                continue
            L = [q for q in placed.get(net, []) if q[3] != r]
            if not L: continue
            c += wnet(net) * min(math.hypot(px - q[0], py - q[1]) + via_k(ps, q[2]) for q in L)
        c += 0.05 * math.hypot(x, y)
        if side != S: c += 0.3
        if P3_SCHLEIFE and r in PIN_ENT and PIN[r][0] in st:
            z, n, pins = PIN[r]
            cp = T[r].pads_abs(x, y, th, side)
            zp = T[z].pads_abs(*st[z])
            c += pin_abst(cp, zp, n, pins, r) - pin_abst(cp, zp, n, pins)
        if PHYSIK:
            ps = {q: T[q].pads_abs(*v) for q, v in st.items()}
            ps[r] = T[r].pads_abs(x, y, th, side)
            c += PHYS.kosten({r: (x,y,th,side)}, ps)
        return c

    def kandidaten(r):
        sides = [fps[r]['side']] if args.get('seiten_original') == '1' else [S]
        if args.get('seiten_original') != '1' and 'F' in SEITEN and 'B' in SEITEN and T[r].n_pads <= 4 and not any(p[4] for p in T[r].pads) and TEILE[a]['rolle'] == 'chip':
            sides.append(GEGEN[S])
        if BGA_GEGEN and 'BGA' in fps[a].get('fp', '') and r in PIN_ENT and T[r].n_pads <= 4 and not any(p[4] for p in T[r].pads):
            sides = sorted(set(sides + [GEGEN[S]]))
        res = []
        for side in sides:
            for th in (0.0, 90.0, 180.0, 270.0):
                pads = T[r]._rot(th, side)[1]

                tgt = None
                zp = ziel_pins(r)
                if zp:
                    k = [p for p in pads if p[1] == (PIN[r][1] if PHYSIK and r in PIN else ent[r]['netz'])]
                    if k:
                        pin = min(zp, key=lambda q: math.hypot(q[0], q[1]))
                        tgt = (pin[0] - k[0][2], pin[1] - k[0][3])
                if tgt is None:
                    best = None
                    for nr, net, px, py, ps in pads:
                        if net in GND or not placed.get(net): continue
                        w = wnet(net)
                        q = min(placed[net], key=lambda q: math.hypot(q[0], q[1]))
                        if best is None or w > best[0]: best = (w, q[0] - px, q[1] - py)
                    tgt = (best[1], best[2]) if best else (0.0, 0.0)
                res.append((side, th, tgt))
        return res

    def setze(r):
        best = None
        for side, th, (x0, y0) in kandidaten(r):
            def frei(x, y): return not bel.kollision(T[r].geo(x, y, th, side))
            cands = []
            if frei(x0, y0): cands.append((x0, y0))
            else:
                for dx, dy in DIRS:
                    prev, d, hit = 0.0, 0.1, None
                    while d <= 10.0:
                        if frei(x0 + dx * d, y0 + dy * d): hit = d; break
                        prev = d; d += 0.1 if d < 2.0 else 0.25
                    if hit is None: continue
                    lo, hi = prev, hit
                    for _ in range(4):
                        mid = (lo + hi) / 2
                        if frei(x0 + dx * mid, y0 + dy * mid): hi = mid
                        else: lo = mid
                    cands.append((x0 + dx * hi, y0 + dy * hi))
            for x, y in cands:
                if side != S and not unter.contains(T[r].geo(x, y, th, side)[side].centroid): continue
                c = kosten(r, x, y, th, side)
                if best is None or c < best[0]: best = (c, x, y, th, side)
        return best

    for r in order:
        if P3_SCHLEIFE and r in (K.get('p3_pin_gruppen') or {}).get('teile', {}):
            z, n, pins = PIN_ENT[r]
            ziel[r] = dict(pins=[(x, y, sd) for num, net, x, y, sd in T[z].pads_abs(*st[z]) if num in pins])
        elif PHYSIK and r in PHYS.pin and PIN[r][0] in st:
            z, n, pins = PIN[r]
            ziel[r] = dict(pins=[(x,y,sd) for num, net, x,y,sd in T[z].pads_abs(*st[z]) if num in pins])
        elif r in ent:
            g = grp.get(ent[r]['netz'])
            if g:
                tgt = min(g, key=lambda c: c['n']); tgt['n'] += 1; ziel[r] = tgt
                if P3_SCHLEIFE and r in PIN_ENT:


                    pins = {nr for nr, p in apads.items() if p in tgt['pins'] and nr in ent[r]['pins']}
                    PIN[r] = (a, ent[r]['netz'], pins)
                    PIN_ENT[r] = PIN[r]
        b = setze(r)
        if b is None: continue
        st[r] = b[1:]; bel.add(r, T[r].geo(*st[r])); add_pads(r)

    for rnd in range(2):
        for r in order:
            if r not in st: continue
            old = st[r]
            bel.remove(r); del_pads(r)
            c_old = kosten(r, *old)
            b = setze(r)
            if b is not None and b[0] < c_old - 1e-3: st[r] = b[1:]
            bel.add(r, T[r].geo(*st[r])); add_pads(r)
    return st



def rot_pt(x, y, R):
    c, s = math.cos(math.radians(R)), math.sin(math.radians(R)); return c * x - s * y, s * x + c * y


class Insel:
    def __init__(self, name, rel, fest):
        self.name = name; self.rel = rel; self.fest = fest
        self.refs = list(rel)
        self.cache = {}
        pads = []
        for r, s in rel.items():
            for nr, net, x, y, ps in T[r].pads_abs(*s): pads.append((r, nr, net, x, y, ps))
        self.pads = pads

        self.keep = [(kr, affinity.translate(g, -fps[kr]['x'], -fps[kr]['y'])) for kr, g in KEEP if kr in rel]
        a = self.refs[0]
        self.anker = a

    def lokal(self, R):
        ""
        if R not in self.cache:
            geos = {'F': [], 'B': []}
            for r, (x, y, th, s) in self.rel.items():
                xr, yr = rot_pt(x, y, R)
                for side, g in T[r].geo(xr, yr, th + R, s).items(): geos[side].append(g)
            u = {s: unary_union(v).simplify(0.02) for s, v in geos.items() if v}
            pads = []
            for r, nr, net, x, y, ps in self.pads:
                xr, yr = rot_pt(x, y, R); pads.append((r, nr, net, xr, yr, ps))
            kp = []
            for kr, g in self.keep:
                x, y, th, s = self.rel[kr]; xr, yr = rot_pt(x, y, R)
                kp.append(affinity.translate(affinity.rotate(g, th + R, origin=(0, 0)), xr, yr))
            self.cache[R] = (u, pads, kp)
        return self.cache[R]


def gesamt_lage(inseln, zust):
    ""
    out = {}
    for I in inseln:
        X, Y, R = zust[I.name]
        for r, (x, y, th, s) in I.rel.items():
            xr, yr = rot_pt(x, y, R); out[r] = (X + xr, Y + yr, (th + R) % 360, s)
    return out


def main():
    global P35_KARTE
    print('Teile', len(T), 'Inseln', len(INSELN), 'Netze', len(NETZ), 'Modus', MODUS)
    if MODUS == 'verfeinern':
        inseln = [Insel(r, {r: (0.0, 0.0, 0.0, fps[r]['side'])}, bool(TEILE[r]['fest']) or TEILE[r]['rolle'] in ('mechanik', 'ohne'))
                  for r in T]
        lage = {r: (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']) for r in T}
        zust = {r: (fps[r]['x'], fps[r]['y'], 0.0) for r in T}
        if P35: P35_KARTE = P35Karte(B,K,RG,T,lage,args)
        lage = feinschliff(inseln, lage, hart_ab=0.0)
        lage = legalisieren(inseln, zust, lage)
        if P35: lage = p35_nachlauf(inseln,zust,lage)
        return ausgabe(inseln, zust, lage)

    rels = {}
    t0 = time.time()
    for a, I in sorted(INSELN.items(), key=lambda kv: -len(kv[1]['mitglieder'])):
        if a not in T: continue
        rels[a] = insel_bauen(a, I)

    for f, (l, dx, dy) in sorted(FOLGE.items()):
        for a, st in rels.items():
            if l in st: st[f] = folge(f, st[l]); break
    print('Inseln gebaut in %.1fs' % (time.time() - t0))
    in_insel = {r for st in rels.values() for r in st}
    inseln = []
    for a, st in rels.items():
        fest = bool(TEILE[a]['fest']) or args.get('fest_anker') == '1'
        inseln.append(Insel(a, st, fest))

    for r, t in TEILE.items():
        if r in in_insel or r not in T or r in FOLGE: continue
        if t['rolle'] == 'ohne' and not T[r].pads and not T[r].hat_kupfergrafik: continue
        rel = {r: (0.0, 0.0, 0.0, fps[r]['side'])}
        for f in GRUPPE.get(r, ()):
            if f not in in_insel: rel[f] = folge(f, rel[r])
        inseln.append(Insel(r, rel, bool(t['fest']) or t['rolle'] in ('mechanik', 'ohne') or r in FEST_EXTRA))
    IDX = {I.name: I for I in inseln}
    zust = {}
    for I in inseln:
        a = I.anker; zust[I.name] = (fps[a]['x'], fps[a]['y'], 0.0)
    grob = VG.get('grob') or {}
    if MODUS == 'neu' and args.get('nur') != 'inseln':


        bx = inner.bounds
        for I in inseln:
            if I.fest: continue
            if args.get('zufall') != '1' and inner_p.contains(Point(zust[I.name][0], zust[I.name][1])): continue
            zust[I.name] = (random.uniform(bx[0], bx[2]), random.uniform(bx[1], bx[3]), random.choice((0.0, 90.0, 180.0, 270.0)))
    for a, g in grob.items():
        if a in zust: zust[a] = (g.get('x', zust[a][0]), g.get('y', zust[a][1]), float(g.get('rot', zust[a][2])))
    if args.get('nur') == 'inseln':
        return ausgabe(inseln, zust, {})


    net_isl = collections.defaultdict(set)
    for I in inseln:
        for r, nr, net, x, y, ps in I.pads:
            if net in NETZ and net not in GND: net_isl[net].add(I.name)
    NETS = {n: s for n, s in net_isl.items() if len(s) > 1}
    W = {n: wnet(n) for n in NETS}
    isl_nets = collections.defaultdict(list)
    for n, s in NETS.items():
        for i in s: isl_nets[i].append(n)

    erlaubt = {}
    for I in inseln:
        X, Y, R = (fps[I.anker]['x'], fps[I.anker]['y'], 0.0)
        u, _, _ = I.lokal(0.0)
        erlaubt[I.name] = sum(affinity.translate(g, X, Y).difference(inner).area for g in u.values()) if MODUS != 'neu' or I.fest else 0.0
    AVC = {}
    def agg_vic(I, R):
        if (I.name, R) in AVC: return AVC[(I.name, R)]
        _, pads, _ = I.lokal(R)
        ag = [(x, y) for r, nr, net, x, y, ps in pads if NETZ.get(net, {}).get('art') == 'sw' or
              (NETZ.get(net, {}).get('art') == 'schnell' and NETZ[net]['rang'] >= 3)]
        vi = [(x, y) for r, nr, net, x, y, ps in pads if NETZ.get(net, {}).get('art') == 'analog' and NETZ[net]['rang'] >= 2]
        f = lambda L: (sum(p[0] for p in L) / len(L), sum(p[1] for p in L) / len(L), len(L)) if L else None
        AVC[(I.name, R)] = (f(ag), f(vi))
        return AVC[(I.name, R)]

    WO = [20.0]
    def pads_abs(I, st):
        X, Y, R = st; _, pads, _ = I.lokal(R)
        return [(net, X + x, Y + y) for r, nr, net, x, y, ps in pads if net in NETS]
    padcache = {I.name: pads_abs(I, zust[I.name]) for I in inseln}
    def hpwl(n, over):
        xs, ys = [], []
        for i in NETS[n]:
            for net, x, y in (over[i] if i in over else padcache[i]):
                if net == n: xs.append(x); ys.append(y)
        return (max(xs) - min(xs) + max(ys) - min(ys)) if xs else 0.0
    def geo_abs(I, st):
        X, Y, R = st; u, _, kp = I.lokal(R)
        return {s: affinity.translate(g, X, Y) for s, g in u.items()}, [affinity.translate(g, X, Y) for g in kp]
    geocache = {I.name: geo_abs(I, zust[I.name]) for I in inseln}
    DZ = DM.Huellziel({r: g[0] for r, g in geocache.items()}) if DICHT else None
    DZI = DM.Huellziel({r: g[0] for r, g in geocache.items()},
                      {I.name for I in inseln if not I.fest}) if DICHT_INNEN else None
    def overlap(ga, gb):
        a = 0.0
        for s in ('F', 'B'):
            if s in ga[0] and s in gb[0]:
                g1, g2 = ga[0][s], gb[0][s]
                b1, b2 = g1.bounds, g2.bounds
                if b1[0] < b2[2] and b2[0] < b1[2] and b1[1] < b2[3] and b2[1] < b1[3]: a += g1.intersection(g2).area
        for k in ga[1]:
            for g in gb[0].values():
                if k.intersects(g): a += k.intersection(g).area
        for k in gb[1]:
            for g in ga[0].values():
                if k.intersects(g): a += k.intersection(g).area
        return a
    def sperr_insel(geos):
        ""
        a = 0.0
        for s, g in geos.items():
            k = SPERR.get(s)
            if k is None: continue
            kb = SPERR_BB[s]; b = g.bounds
            if b[0] < kb[2] and kb[0] < b[2] and b[1] < kb[3] and kb[1] < b[3]: a += g.intersection(k).area
        return a
    def aussen(I, ga):
        a = 0.0
        for g in ga[0].values():
            if not inner_p.contains(g): a += g.difference(inner).area
        return max(0.0, a - erlaubt[I.name] - 1e-3) + sperr_insel(ga[0])
    avc = {I.name: agg_vic(I, zust[I.name][2]) for I in inseln}
    def sep(i, sti, j, stj):
        ai, vi = agg_vic(IDX[i], sti[2]) if sti is not None else avc[i]
        aj, vj = agg_vic(IDX[j], stj[2]) if stj is not None else avc[j]
        Xi, Yi, _ = sti or zust[i]; Xj, Yj, _ = stj or zust[j]
        p = 0.0
        for A, Ao, V, Vo in ((ai, (Xi, Yi), vj, (Xj, Yj)), (aj, (Xj, Yj), vi, (Xi, Yi))):
            if A and V:
                d = math.hypot(A[0] + Ao[0] - V[0] - Vo[0], A[1] + Ao[1] - V[1] - Vo[1])
                if d < TRENN: p += (TRENN - d) ** 2 * math.sqrt(min(A[2], V[2]))
        return p
    sep_isl = [I.name for I in inseln if any(avc[I.name])]
    def feder(i, st):
        g = grob.get(i)
        if not g: return 0.0
        d = math.hypot(st[0] - g.get('x', st[0]), st[1] - g.get('y', st[1]))
        return 2.0 * max(0.0, d - g.get('radius', 0.5)) ** 2

    def lokal_kosten(ch):
        ""
        over = {i: pads_abs(IDX[i], s) for i, s in ch.items()}
        gx = {i: geo_abs(IDX[i], s) for i, s in ch.items()}
        nets = set()
        for i in ch: nets.update(isl_nets[i])
        c = sum(W[n] * hpwl(n, over) for n in sorted(nets))
        names = list(ch)
        for i in names:
            c += WO[0] * aussen(IDX[i], gx[i]) + feder(i, ch[i])
            for J in inseln:
                j = J.name
                if j in ch: continue
                c += WO[0] * overlap(gx[i], geocache[j])
        for a_ in range(len(names)):
            for b_ in range(a_ + 1, len(names)): c += WO[0] * overlap(gx[names[a_]], gx[names[b_]])
        for i in names:
            if i not in sep_isl: continue
            for j in sep_isl:
                if j == i or (j in ch and j < i): continue
                c += 2.0 * sep(i, ch[i], j, ch.get(j))
        if DZ: c += DICHT * DZ.flaeche({k: g[0] for k, g in gx.items()}) / DICHT_NORM
        if DZI: c += DICHT_INNEN * DZI.flaeche({k: g[0] for k, g in gx.items()}) / DICHT_NORM
        return c, over, gx

    beweglich = [I.name for I in inseln if not I.fest]
    N = int(args.get('iter', str(min(60000, max(8000, 600 * len(beweglich))))))
    if not beweglich: N = 0
    bx = inner.bounds; diag = math.hypot(bx[2] - bx[0], bx[3] - bx[1])
    def gesamt():
        c = sum(W[n] * hpwl(n, {}) for n in sorted(NETS))
        for I in inseln:
            c += WO[0] * aussen(I, geocache[I.name]) + feder(I.name, zust[I.name])
        for a_ in range(len(inseln)):
            for b_ in range(a_ + 1, len(inseln)): c += WO[0] * overlap(geocache[inseln[a_].name], geocache[inseln[b_].name])
        if DZ: c += DICHT * DZ.flaeche() / DICHT_NORM
        if DZI: c += DICHT_INNEN * DZI.flaeche() / DICHT_NORM
        return c
    c_start = gesamt()
    T0 = float(args.get('t0', '0' if MODUS == 'verfeinern' else '0')) or max(1.0, 0.05 * c_start / max(1, len(beweglich)))
    if MODUS == 'verfeinern': T0 *= 0.2
    sig0 = diag / 4 if MODUS == 'neu' else min(2.0, diag / 10)
    acc = 0; t1 = time.time()
    wert = None
    if args.get('wert', '0') == '1':
        from lernen.wert_placer import Grobwert
        wert = Grobwert(B, K, RG, T, args)
        print('Wertmodell grob:', wert.protokoll, flush=True)
    for it in range(N):
        f = it / max(1, N)
        Tt = T0 * (1 - f) ** 2 + 1e-3
        WO[0] = 20.0 + 380.0 * f
        sig = max(0.05, sig0 * (1 - f) ** 1.5)
        mv = random.random()
        i = random.choice(beweglich)
        if mv < 0.75:
            X, Y, R = zust[i]; ch = {i: (min(bx[2], max(bx[0], X + random.gauss(0, sig))), min(bx[3], max(bx[1], Y + random.gauss(0, sig))), R)}
        elif mv < 0.92:
            X, Y, R = zust[i]; ch = {i: (X, Y, (R + random.choice((90.0, 180.0, 270.0))) % 360)}
        else:
            j = random.choice(beweglich)
            if j == i: continue
            ch = {i: (zust[j][0], zust[j][1], zust[i][2]), j: (zust[i][0], zust[i][1], zust[j][2])}
        c_old, _, _ = lokal_kosten({k: zust[k] for k in ch})
        c_new, over, gx = lokal_kosten(ch)
        d = c_new - c_old
        if wert is not None and it % wert.alle == 0:
            kandidat = dict(zust, **ch)
            d += max(1., c_start / max(1, len(beweglich))) * wert.delta(gesamt_lage(inseln, zust), gesamt_lage(inseln, kandidat))
        if d < 0 or random.random() < math.exp(-d / Tt):
            for k, s in ch.items():
                zust[k] = s; padcache[k] = over[k]; geocache[k] = gx[k]
                if DZ: DZ.setze(k, gx[k][0])
                if DZI and k in DZI.bounds: DZI.setze(k, gx[k][0])
                avc[k] = agg_vic(IDX[k], s[2])
            acc += 1
        if it % max(1, N // 8) == 0:
            print('  SA %6d/%d  T %.3f  Kosten %.1f  angenommen %d  (%.0fs)' % (it, N, Tt, gesamt(), acc, time.time() - t1), flush=True)
    if wert is not None:
        args['wert_protokoll'] = wert.protokoll
        print('Wertmodell grob Ende:', wert.protokoll, flush=True)
    WO[0] = 400.0
    print('Grobplatzierung: Kosten %.1f -> %.1f (%.0fs)' % (c_start, gesamt(), time.time() - t1))

    lage = gesamt_lage(inseln, zust)
    if PHYSIK and args.get('leistung_fest', '1') == '1':
        for a, rs in PHYS.gruppen.items():
            if all(r in lage and not TEILE[r]['fest'] and r not in FOLGE and r not in GRUPPE for r in rs):
                POWER[a] = rs
                for r in rs: POWER_OF[r] = a
    if P35: P35_KARTE = P35Karte(B,K,RG,T,lage,args)
    if args.get('fein', '1') == '1': lage = feinschliff(inseln, lage)
    lage = legalisieren(inseln, zust, lage)
    if P35: lage = p35_nachlauf(inseln,zust,lage)
    return ausgabe(inseln, zust, lage)


def innen_fest(inseln, lage):
    ""
    fest = {r for I in inseln for r in I.refs if TEILE.get(r, {}).get('fest') or (I.fest and I.anker == r)} | (FEST_EXTRA & set(lage))


    for r in list(fest):
        fest.update(mates(r) & set(lage))
    return fest


def feinschliff(inseln, lage, hart_ab=0.7):
    ""
    t0 = time.time()
    fest = innen_fest(inseln, lage)
    mov = [r for r in lage if r not in fest and r not in FOLGE and POWER_OF.get(r, r) == r]
    if not mov: return lage
    pads = {r: T[r].pads_abs(*lage[r]) for r in lage}
    net_refs = collections.defaultdict(set)
    for r, ps in pads.items():
        for nr, net, x, y, sd in ps:
            if net in NETZ and net not in GND: net_refs[net].add(r)
    W = {n: wnet(n) for n in net_refs}
    PZ = {c: z for c, z in PIN.items() if c in lage and z[0] in lage}
    PZ_OF = collections.defaultdict(list)
    for c, (z, n, pins) in sorted(PZ.items()): PZ_OF[z].append(c)
    def hpwl(n, ov):
        xs = []; ys = []
        for r in net_refs[n]:
            for nr, net, x, y, sd in (ov[r] if r in ov else pads[r]):
                if net == n: xs.append(x); ys.append(y)
        return (max(xs) - min(xs) + max(ys) - min(ys)) if len(xs) > 1 else 0.0
    def net_pts(n, ov):
        return [(x, y) for r in net_refs[n] for nr, net, x, y, sd in (ov[r] if r in ov else pads[r]) if net == n]
    KNETS = sorted(n for n in net_refs if len(net_pts(n, {})) <= KREUZ_MAXPADS) if KREUZ > 0 else []
    NID = {n: i for i, n in enumerate(KNETS)}
    SEG = {n: np.array(mst_kanten(net_pts(n, {})), float).reshape(-1, 4) for n in KNETS}
    def baue_alle():
        arrs = [SEG[n] for n in KNETS if len(SEG[n])]
        if not arrs: return np.zeros((0, 4)), np.zeros(0, int)
        return np.concatenate(arrs), np.concatenate([np.full(len(SEG[n]), NID[n]) for n in KNETS if len(SEG[n])])
    ALLE = [baue_alle()]
    def kreuz_term(nets, ov):
        ""
        nets = sorted(n for n in nets if n in NID)
        if not nets: return 0.0, {}
        neu = {n: np.array(mst_kanten(net_pts(n, ov)), float).reshape(-1, 4) for n in nets}
        A, ids = ALLE[0]
        rest = A[~np.isin(ids, [NID[n] for n in nets])]
        c = sum(kreuzungen(neu[n], rest) for n in sorted(nets))
        for i in range(len(nets)):
            for j in range(i + 1, len(nets)): c += kreuzungen(neu[nets[i]], neu[nets[j]])
        return KREUZ * c, neu
    def pin_term(c, ov):
        z, n, pins = PZ[c]
        return pin_abst(ov[c] if c in ov else pads[c], ov[z] if z in ov else pads[z], n, pins, c)
    bel = Belegung()
    geo = {r: T[r].geo(*lage[r]) for r in lage}
    DZ = DM.Huellziel(geo) if DICHT else None
    DZI = DM.Huellziel(geo, DM.bewegliche(K, lage, fest)) if DICHT_INNEN else None
    DIS = DM.Inselziel(geo, DM.inselgruppen(K, lage, fest)) if DICHT_INSELN else None
    for r in lage: bel.add(r, geo[r])

    erlaubt = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']).values()) for r in lage}
    def aussen(r, g):
        a = sum(gg.difference(inner).area for gg in g.values() if not inner_p.contains(gg))
        return max(0.0, a - erlaubt[r] - 1e-3) + sperr_flaeche(r, g)
    KR = sorted(kr for kr in KPL if kr in lage) if SPERR_AN else []
    KPABS = {kr: kp_abs(kr, lage[kr]) for kr in KR}
    def kp_ueber(ch, gx):
        ""
        if not KR: return 0.0
        a = 0.0
        ks = {kr: (kp_abs(kr, ch[kr]) if kr in ch else KPABS[kr]) for kr in KR}
        for r in ch:
            for kr in KR:
                if kr == r: continue
                for k in ks[kr]:
                    kb = k.bounds
                    for g in gx[r].values():
                        b = g.bounds
                        if b[0] < kb[2] and kb[0] < b[2] and b[1] < kb[3] and kb[1] < b[3]: a += g.intersection(k).area
        for kr in KR:
            if kr not in ch: continue
            for k in ks[kr]: a += bel.ueberlappung({'F': k, 'B': k}, ignore=set(ch))[0]
        return a
    PF = Paare(lage)
    if not PF.terme: PF = None
    else: print('  Feinschliff-Paarterme:', ', '.join(t['name'] for t in PF.terme), flush=True)
    WO = [10.0]
    def kosten(ch):
        ov = {r: T[r].pads_abs(*s) for r, s in ch.items()}
        gx = {r: T[r].geo(*s) for r, s in ch.items()}
        nets = set()
        for r in ch:
            for nr, net, x, y, sd in ov[r]:
                if net in net_refs: nets.add(net)
        c = sum(W[n] * hpwl(n, ov) for n in sorted(nets))
        kt, segneu = kreuz_term(nets, ov) if KREUZ > 0 else (0.0, {})
        c += kt
        ents = set()
        for r in ch:
            if r in PZ: ents.add(r)
            ents.update(PZ_OF.get(r, []))
        c += sum(pin_term(e, ov) for e in sorted(ents))
        for r in ch:
            a_, _ = bel.ueberlappung(gx[r], ignore=set(ch))
            c += WO[0] * (a_ + aussen(r, gx[r])) + feder_teil(r, ch[r])
            if WW_AN and r in WT and r not in FOLGE: c += WW_F * WWM.preis(WT[r], 0.0, weg(r, ch[r], lage, ch))
        c += WO[0] * kp_ueber(ch, gx)
        rs = list(ch)
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                if rs[j] in mates(rs[i]): continue
                for sd in gx[rs[i]]:
                    if sd in gx[rs[j]]: c += WO[0] * gx[rs[i]][sd].intersection(gx[rs[j]][sd]).area
        if PF is not None: c += PF.beitrag(ch)
        if DZ: c += DICHT * DZ.flaeche(gx) / DICHT_NORM
        if DZI: c += DICHT_INNEN * DZI.flaeche(gx) / DICHT_NORM
        if DIS: c += DICHT_INSELN * DIS.beitrag(gx)
        if PHYSIK:
            c += PHYS.kosten(ch, dict(pads, **ov))
        if P35_KARTE is not None: c += P35_W * P35_KARTE.kosten(ch)
        return c, ov, gx, segneu
    same = collections.defaultdict(list)
    for r in mov:
        if r not in GRUPPE: same[(fps[r]['fp'], T[r].n_pads)].append(r)
    N = int(args.get('fein_iter', str(min(80000, 400 * len(mov)))))
    acc = 0
    for it in range(N):
        f = it / N
        WO[0] = 10.0 + 490.0 * f
        Tt = 0.3 * (1 - f) ** 2 + 1e-3
        sig = 0.6 * (1 - f) + 0.03
        r = random.choice(mov); x, y, th, sd = lage[r]
        mvz = random.random(); tausch = False
        if mvz < 0.8: ch = mit_gruppe(r, (x + random.gauss(0, sig), y + random.gauss(0, sig), th, sd), lage)
        elif mvz < 0.92 and gruppen_drehbar(r): ch = mit_gruppe(r, (x, y, (th + random.choice((90.0, 180.0, 270.0))) % 360, sd), lage)
        else:
            g = same.get((fps[r]['fp'], T[r].n_pads), [])
            q = random.choice(g) if len(g) > 1 else None
            if q is None or q == r or lage[q][3] != sd or r in GRUPPE or r in POWER or q in POWER: continue
            ch = {r: (lage[q][0], lage[q][1], lage[q][2], sd), q: (x, y, th, sd)}; tausch = True
        for k in ch: bel.remove(k)
        c_old, _, gx_old, _ = kosten({k: lage[k] for k in ch})
        c_new, ov, gx, segneu = kosten(ch)
        if ROUTBARKEIT is not None:
            c_new += RW * ROUTBARKEIT.feinkosten(B, K, RG, T, lage, ch,gndlagen=args.get('gndlagen'))
        d = c_new - c_old
        hart_ok = True
        if f >= hart_ab:
            o_new = sum(bel.ueberlappung(gx[k], ignore=set(ch))[0] + aussen(k, gx[k]) for k in ch) + kp_ueber(ch, gx)
            o_old = sum(bel.ueberlappung(gx_old[k], ignore=set(ch))[0] + aussen(k, gx_old[k]) for k in ch) + kp_ueber({k: lage[k] for k in ch}, gx_old)
            hart_ok = o_new <= o_old + 1e-9
            if hart_ok:
                ks = list(ch)
                for k in ks:
                    if bel.kollision(gx[k], ignore=set(ch)) and not bel.kollision(gx_old[k], ignore=set(ch)): hart_ok = False
                if hart_ok and tausch:
                    b2 = Belegung(); b2.add(ks[1], gx[ks[1]])
                    if b2.kollision(gx[ks[0]]): hart_ok = False
        if hart_ok and WW_AN:
            for k in ch:
                if k in FOLGE or k not in WT: continue
                wn = weg(k, ch[k], lage, ch)
                if wn > WT[k]['max'] + 1e-6 and wn > weg(k, lage[k], lage) + 1e-9: hart_ok = False; break
        if hart_ok and (d < 0 or random.random() < math.exp(-d / Tt)):
            for k, s in ch.items():
                lage[k] = s; pads[k] = ov[k]; geo[k] = gx[k]
                if DZ: DZ.setze(k, gx[k])
                if DZI and k in DZI.bounds: DZI.setze(k, gx[k])
                if DIS: DIS.setze(k, gx[k])
                if k in KPABS: KPABS[k] = kp_abs(k, s)
                if PF is not None: PF.setze(k, s)
            if segneu: SEG.update(segneu); ALLE[0] = baue_alle()
            acc += 1
        for k in ch: bel.add(k, geo[k])
        if it % max(1, N // 6) == 0:
            print('  Feinschliff %6d/%d  angenommen %d  (%.0fs)' % (it, N, acc, time.time() - t0), flush=True)
    if DICHT_GRUPPEN:
        from platzierung.gruppenkompakt import gruppen, kandidaten
        binds = [set(v) | {r} for r, v in GRUPPE.items()] + list(POWER.values())
        binds += [{c, z[0]} for c, z in PIN.items()]
        groups = gruppen(lage, [{a, *i['mitglieder']} for a, i in sorted(INSELN.items())], binds, fest)
        WO[0] = 500.0
        accepted = 0
        for _ in range(2):
            for rs in groups:
                for ch in kandidaten(rs, lage, pads, net_refs, W):
                    old = {r: lage[r] for r in rs}
                    for r in rs: bel.remove(r)
                    co, _, _, _ = kosten(old)
                    cn, ov, gx, sn = kosten(ch)
                    nets = sorted({n for r in rs for _, n, _, _, _ in pads[r] if n in net_refs})


                    ok = cn < co - 1e-8 and kp_ueber(ch, gx) <= 1e-9
                    if ok:
                        ok = sum(W[n] * hpwl(n, ov) for n in nets) < sum(W[n] * hpwl(n, {}) for n in nets) - 1e-8
                        if ok and KREUZ > 0: ok = kreuz_term(nets, ov)[0] <= kreuz_term(nets, {})[0] + 1e-9
                    if ok:
                        ok = all(not bel.kollision(gx[r], ignore=set(rs)) and aussen(r, gx[r]) <= 1e-9
                                 and all(inner_p.contains(Point(x, y)) for _, _, x, y, _ in ov[r])
                                 and (PADRAND is None or PADRAND.frei(r, ch[r])) for r in rs)
                    if ok and WW_AN:
                        ok = all(r in FOLGE or r not in WT or weg(r, ch[r], lage, ch) <= WT[r]['max'] + 1e-6 for r in rs)
                    if ok and ROUTBARKEIT is not None:
                        ok = cn + RW * ROUTBARKEIT.feinkosten(B, K, RG, T, lage, ch, gndlagen=args.get('gndlagen')) < co - 1e-8
                    if ok:


                        ok = PF is None or PF.beitrag(ch) <= PF.beitrag(old) + 1e-9
                    if ok:
                        for r in rs:
                            lage[r] = ch[r]; pads[r] = ov[r]; geo[r] = gx[r]
                            if DZ: DZ.setze(r, gx[r])
                            if DZI and r in DZI.bounds: DZI.setze(r, gx[r])
                            if DIS: DIS.setze(r, gx[r])
                            if r in KPABS: KPABS[r] = kp_abs(r, ch[r])
                            if PF is not None: PF.setze(r, ch[r])
                        if sn: SEG.update(sn); ALLE[0] = baue_alle()
                        accepted += 1
                    for r in rs: bel.add(r, geo[r])
        print('P26 starre Gruppen: %d Gruppen, %d Translationen angenommen' % (len(groups), accepted), flush=True)
    print('Feinschliff fertig (%.0fs)' % (time.time() - t0))
    return lage


def kosten_global(r, s, lage, padidx):
    ""
    ch = mit_gruppe(r, s, lage)
    over = collections.defaultdict(list)
    for q, sq in ch.items():
        for _, net, x, y, sd in T[q].pads_abs(*sq): over[net].append((x, y, sd, q))
    c = 0.0
    for q, sq in ch.items():
        for _, net, px, py, ps in T[q].pads_abs(*sq):
            if net in GND or net not in padidx: continue
            L = [p for p in padidx[net] if p[3] not in ch] + over[net]
            L = [p for p in L if p[3] != q]
            if L: c += wnet(net) * min(math.hypot(px-p[0], py-p[1]) + via_k(ps,p[2]) for p in L)
    if DICHT_INSELN:

        fest = {q for q in lage if TEILE[q]['fest']} | (FEST_EXTRA & set(lage))
        if args.get('fest_anker') == '1': fest.update(set(INSELN) & set(lage))
        for q in list(fest): fest.update(mates(q) & set(lage))
        gruppen = DM.inselgruppen(K, lage, fest)
        betroffen = {a: rs for a, rs in gruppen.items() if set(rs) & ch.keys()}
        gs = {q: T[q].geo(*lage[q]) for rs in betroffen.values() for q in rs}
        ziel = DM.Inselziel(gs, betroffen)
        c += DICHT_INSELN * ziel.beitrag({q: T[q].geo(*sq) for q, sq in ch.items()})
    return c


def pin_kosten(r, st, lage):
    ""
    ch = mit_gruppe(r, st, lage)
    refs = set(ch) & PIN.keys()
    for q in ch: refs.update(PIN_OF.get(q, ()))
    c = 0.0
    for cc in sorted(refs):
        z, n, pins = PIN[cc]
        if cc in lage and z in lage:
            c += pin_abst(T[cc].pads_abs(*ch.get(cc, lage[cc])),
                          T[z].pads_abs(*ch.get(z, lage[z])), n, pins, cc)
    return c


LEGAL_AUSNAHMEN = {}


def legal_wanderwahl(r, weich, hart, lage, grund):
    ""
    if not WW_AN or hart is not None: return hart if hart is not None else weich
    if weich is None: return None
    ch = mit_gruppe(r, weich[1])
    for q, sq in ch.items():
        if q in WT and q not in FOLGE:
            w = weg(q, sq, lage, ch)
            if w > WT[q]['max'] + 1e-6:
                LEGAL_AUSNAHMEN[q] = dict(weg_mm=round(w, 4), max_mm=WT[q]['max'],
                    grund=grund + '; kein Kandidat innerhalb der Wandergrenze in Spiral-/Freiraumsuche',
                    nachweis='begrenzte geometrische Suche, kein globaler Unmoeglichkeitsbeweis')
    return weich


def legalisieren(inseln, zust, lage, nachpins=True):
    owner = {r: I for I in inseln for r in I.refs}
    def neu_index():
        bel = Belegung(); padidx = collections.defaultdict(list)
        for r, s in lage.items():
            bel.add(r, T[r].geo(*s))
            for nr, net, x, y, ps in T[r].pads_abs(*s): padidx[net].append((x, y, ps, r))
        return bel, padidx


    erlaubt = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']).values()) for r in lage}
    keep_abs = {kr: kp_abs(kr, lage[kr], klein=True) for kr in KPLK if kr in lage}
    def problem(r, s, bel, ign=None):
        geo = T[r].geo(*s)
        if bel.kollision(geo, ignore=ign or {r}): return True
        if not TEILE.get(r, {}).get('fest') and any(not inner_p.contains(Point(x, y)) for nr, net, x, y, ps in T[r].pads_abs(*s)):
            return True
        if PADRAND is not None and not TEILE.get(r,{}).get('fest') and not PADRAND.frei(r,s):return True
        if sum(g.difference(inner).area for g in geo.values()) > erlaubt[r] + 1e-3: return True
        if not TEILE.get(r, {}).get('fest') and sperr_flaeche(r, geo) > 1e-7: return True
        for kr, ks in keep_abs.items():
            if kr == r: continue
            for k in ks:
                if any(k.intersects(g) for g in geo.values()): return True
        if SPERR_AN and r in keep_abs and not TEILE.get(r, {}).get('fest'):
            for k in kp_abs(r, s, klein=True):
                if bel.ueberlappung({'F': k, 'B': k}, ignore=ign or {r})[0] > 1e-7: return True
        return False
    def problem_g(r, s, bel):
        ign = bewegungs_refs(r)
        return any(problem(q, sq, bel, ign) for q, sq in mit_gruppe(r, s, lage).items() if q in lage)
    def wander_ok(r, s):
        ch = mit_gruppe(r, s, lage)
        return not WW_AN or all(q in FOLGE or q not in WT or weg(q, sq, lage, ch) <= WT[q]['max'] + 1e-6 for q, sq in ch.items())
    def problem_hart(r, s, bel): return problem_g(r, s, bel) or not wander_ok(r, s)
    def ursache(r, s, bel):
        g = T[r].geo(*s); gr = []
        if bel.kollision(g, ignore=mates(r)): gr.append('Kollision')
        if sperr_flaeche(r, g) > 1e-7: gr.append('Sperrflaeche')
        if sum(x.difference(inner).area for x in g.values()) > erlaubt[r]+1e-3: gr.append('Rand')
        return '/'.join(gr) or 'geometrische Regelverletzung (Pad-/Footprint-Sperre)'
    def suche_frei(r, st, bel, prob, **kw):
        def geometrie(x,y,th,sd):
            gs = collections.defaultdict(list)
            for q,sq in mit_gruppe(r,(x,y,th,sd),lage).items():
                for side,g in T[q].geo(*sq).items(): gs[side].append(g)
            return {side:unary_union(v) for side,v in gs.items()}
        return freiraum(r,st,bel,prob,geometrie=geometrie if r in POWER else None,**kw)
    def kompakt_refit(refs, paar=False, kreuz_hart=False):



        order=sorted(refs,key=lambda r:(typ(r,'') != 'L',typ(r,'') != 'D',r))
        pf=Paare(lage) if paar else None
        for _ in range(2):
            for r in order:
                if r not in PHYS.pin or TEILE[r]['fest'] or r in FEST_EXTRA or r in FOLGE or r in GRUPPE: continue
                if owner[r].fest and owner[r].anker==r: continue
                z,n,pins=PHYS.pin[r]
                if z not in lage: continue
                bel,idx=neu_index();bel.remove(r)
                old=lage[r];pads={q:T[q].pads_abs(*(lage.get(q) or (fps[q]['x'],fps[q]['y'],0,fps[q]['side']))) for q in T}
                oldpin=pin_kosten(r,old,lage)
                if kreuz_hart and problem(r,old,bel): continue


                rest=np.zeros((0,4));kn=[];oldcross=0
                if kreuz_hart:
                    pts=collections.defaultdict(list)
                    for q in T:
                        for _,net,x,y,_ in pads[q]:
                            if net in NETZ and net not in GND:pts[net].append((x,y))
                    kn=sorted({p[1] for p in pads[r] if p[1] in pts and len(pts[p[1]])<=KREUZ_MAXPADS})
                    seg=[np.asarray(mst_kanten(ps),float).reshape(-1,4) for net,ps in sorted(pts.items()) if net not in kn and len(ps)<=KREUZ_MAXPADS]
                    if seg:rest=np.concatenate(seg)
                def crossing(st):
                    seg=[]
                    for net in kn:
                        pts=[(x,y) for q in T for _,n,x,y,_ in (T[r].pads_abs(*st) if q==r else pads[q]) if n==net]
                        seg.append(np.asarray(mst_kanten(pts),float).reshape(-1,4))
                    return sum(kreuzungen(s,rest) for s in seg)+sum(kreuzungen(a,b) for i,a in enumerate(seg) for b in seg[i+1:])
                if kn:oldcross=crossing(old)
                def preis(st):
                    ps=dict(pads);ps[r]=T[r].pads_abs(*st)
                    c=kosten_global(r,st,lage,idx)+pin_kosten(r,st,lage)+PHYS.kosten({r:st},ps)
                    if pf is not None:c+=pf.beitrag({r:st})+feder_teil(r,st)
                    return c
                best=(preis(old),old)
                ths=[old[2]]+([(old[2]+t)%360 for t in (90,180,270)] if darf_drehen(r) else [])
                for th in ths:
                    rp=[p for p in T[r].pads_abs(0,0,th,old[3]) if p[1] == n]
                    zp=[p for p in pads[z] if p[0] in pins and p[1] == n]
                    for p in rp:
                        for q in zp:
                            x0,y0=q[2]-p[2],q[3]-p[3]
                            for dx,dy in DIRS:
                                hits=0
                                for step in range(49):
                                    st=(x0+dx*step*.25,y0+dy*step*.25,th,old[3])
                                    if problem(r,st,bel) or not wander_ok(r,st): continue
                                    if paar and pin_kosten(r,st,lage)>oldpin+1e-9: continue
                                    cost=preis(st)
                                    if cost < best[0]-1e-6 and (not kn or crossing(st)<=oldcross): best=(cost,st)
                                    hits+=1
                                    if hits >= 4: break
                lage[r]=best[1]
                if paar and best[1]!=old:PIN_NACHLEGAL.add(r)
                if pf is not None:pf.setze(r,best[1])
                if r in keep_abs: keep_abs[r]=kp_abs(r,best[1],klein=True)
    if FREIRAUM:



        gross = [r for r in lage if TEILE[r]['fest'] or TEILE[r]['rolle'] != 'klein']
        for r in sorted([r for r in gross if not TEILE[r]['fest'] and r not in FOLGE and r not in FEST_EXTRA], key=lambda r: -T[r].flaeche):
            bel_g = Belegung()
            for q in gross:
                if q not in bewegungs_refs(r): bel_g.add(q, T[q].geo(*lage[q]))
            if not problem_g(r, lage[r], bel_g): continue
            hart = suche_frei(r, lage[r], bel_g, problem_hart, drehen=True) if WW_AN else None
            best = hart or suche_frei(r, lage[r], bel_g, problem_g, drehen=True)
            best = legal_wanderwahl(r, best, hart, lage, ursache(r, lage[r], bel_g))
            if best:
                print('  Chip %s vor dem Legalisieren an freien Platz (%.1f mm)' % (r, best[0]), flush=True)
                for q, sq in mit_gruppe(r, best[1], lage).items():
                    lage[q] = sq
                    if SPERR_AN and q in keep_abs: keep_abs[q] = kp_abs(q, sq, klein=True)
    def wichtig(r): return WT.get(r, {}).get('wichtigkeit', 0.0) if PIN_LEGAL else 0.0
    vorher = None
    for rnd in range(8):
        bel, padidx = neu_index()
        bad = [r for r in lage if problem(r, lage[r], bel, mates(r))]
        print('  Legalisieren Runde %d: %d Teile mit Kollision/Rand' % (rnd + 1, len(bad)), flush=True)
        if not bad: break
        weit = vorher is not None and len(bad) >= vorher
        if weit and rnd >= 3: break
        vorher = len(bad)


        bad.sort(key=lambda r: (TEILE[r]['rolle'] != 'klein', wichtig(r), T[r].flaeche))
        erledigt = set()
        for r0 in bad:
            r = FOLGE[r0][0] if r0 in FOLGE else r0
            r = POWER_OF.get(r, r)
            if r in erledigt or r not in lage: continue
            erledigt.add(r)
            if owner[r].fest and owner[r].anker == r: continue
            if TEILE[r]['fest'] or r in FEST_EXTRA: continue
            if not problem_g(r, lage[r], bel): continue
            for q in bewegungs_refs(r): bel.remove(q)
            x0, y0, th0, s0 = lage[r]
            rmax = (40.0 if weit else 10.0) if TEILE[r]['rolle'] == 'klein' else (10.0 if weit else 3.0)
            best = None; hart = None
            for th in sorted({th0, (th0 + 90) % 360, (th0 + 180) % 360, (th0 + 270) % 360}, key=lambda t: t != th0):
                if not gruppen_drehbar(r) and th != th0: continue
                gefunden = 0; d_first = None
                for dx, dy, d in SPIRALE:
                    if d > rmax or (d_first is not None and d > d_first + 1.0) or gefunden >= 12: break
                    st = (x0 + dx, y0 + dy, th, s0)
                    if problem_g(r, st, bel): continue
                    if d_first is None: d_first = d
                    gefunden += 1
                    c = kosten_global(r, st, lage, padidx) + 0.5 * d + 0.2 * (th != th0)
                    if PIN_LEGAL: c += pin_kosten(r, st, lage)
                    if PHYSIK:
                        ch = mit_gruppe(r, st, lage)
                        ps = {q: T[q].pads_abs(*ch.get(q, sq)) for q, sq in lage.items()}
                        c += PHYS.kosten(ch, ps)
                    if WW_AN and r in WT: c += WW_F * WWM.preis(WT[r], 0.0, weg(r, st, lage))
                    if best is None or c < best[0]: best = (c, st)
                    if wander_ok(r, st) and (hart is None or c < hart[0]): hart = (c, st)
            if WW_AN and hart is None and FREIRAUM: hart = suche_frei(r, lage[r], bel, problem_hart)
            if best is None and FREIRAUM:
                best = suche_frei(r, lage[r], bel, problem_g)
            best = legal_wanderwahl(r, best, hart, lage, ursache(r, lage[r], bel))
            if best:
                for q, sq in mit_gruppe(r, best[1], lage).items():
                    lage[q] = sq
                    for nr, net, x, y, ps in T[q].pads_abs(*sq): padidx[net].append((x, y, ps, q))
                    if SPERR_AN and q in keep_abs: keep_abs[q] = kp_abs(q, sq, klein=True)
            for q in bewegungs_refs(r):
                if q in lage: bel.add(q, T[q].geo(*lage[q]))
    if PHYSIK and args.get('legal_pack', '1') == '1' and MODUS == 'neu':


        bel, _ = neu_index()
        alt_bad = sum(problem(r, s, bel, mates(r)) for r, s in lage.items())
        if alt_bad:
            alt = dict(lage); best = (alt_bad, dict(lage))
            leaders = [r for r in lage if r not in FOLGE and POWER_OF.get(r, r) == r]
            fixed = {r for r in leaders if TEILE[r]['fest'] or r in FEST_EXTRA or (owner[r].fest and owner[r].anker == r)}
            fixed_bel = Belegung()
            fixed_refs = {q for r in fixed for q in bewegungs_refs(r)}
            for q in fixed_refs: fixed_bel.add(q,T[q].geo(*lage[q]))
            floor = sum(fixed_bel.kollision(T[q].geo(*lage[q]),ignore=mates(q)) for q in fixed_refs)
            for rnd in range(3 + (args.get('legal_voll','0') == '1')):
                if best[0] <= floor: break
                keep_abs = lage_zurueck(lage, alt)
                bel = Belegung()
                for r in sorted(fixed):
                    for q in bewegungs_refs(r): bel.add(q, T[q].geo(*lage[q]))
                def pack_rang(r):
                    return (TEILE[r]['rolle'] == 'klein', -sum(T[q].flaeche for q in bewegungs_refs(r)), -wichtig(r), r)
                order = sorted(set(leaders)-fixed, key=pack_rang)
                if rnd == 1: order.sort(key=lambda r: (TEILE[r]['rolle'] == 'klein', -wichtig(r), -T[r].flaeche, r))
                if rnd >= 2:



                    order.sort(key=lambda r: (-sum(T[q].flaeche for q in bewegungs_refs(r)), -wichtig(r), r))
                for r in order:
                    st = lage[r]
                    if rnd == 3:



                        gs = [g for q,sq in mit_gruppe(r,(0,0,st[2],st[3]),lage).items() for g in T[q].geo(*sq).values()]
                        st=(inner.bounds[0]+.01-min(g.bounds[0] for g in gs),
                            inner.bounds[1]+.01-min(g.bounds[1] for g in gs),st[2],st[3])
                    if rnd == 3 or problem_g(r, st, bel):
                        found = suche_frei(r, st, bel, problem_g, raster=0.1, n_max=4000,
                                         drehen=not DREHEN_REGEL) if FREIRAUM else None
                        if found:
                            lage.update(mit_gruppe(r, found[1], lage))
                            for q in bewegungs_refs(r):
                                if q in keep_abs: keep_abs[q] = kp_abs(q, lage[q], klein=True)
                    for q in bewegungs_refs(r): bel.add(q, T[q].geo(*lage[q]))
                bad = sum(problem(r, s, bel, mates(r)) for r, s in lage.items())
                if bad < best[0]: best = (bad, dict(lage))
                print('  Dichtes Neupacken %d: %d -> %d Regelprobleme' % (rnd+1, alt_bad, bad), flush=True)
                if bad == 0: break
            keep_abs = lage_zurueck(lage, best[1])




            if best[0] > floor and POWER and args.get('leistung_refit','1') == '1':
                bel,_=neu_index()
                keep_abs={kr:kp_abs(kr,lage[kr],klein=True) for kr in KPLK if kr in lage}
                affected={POWER_OF[r] for r,s in lage.items() if r in POWER_OF and problem(r,s,bel,mates(r))}
                if affected:
                    saved=dict(lage);pg=dict(POWER);po=dict(POWER_OF);pr=set(POWER_REFIT)
                    for a in affected:
                        for q in POWER.pop(a): POWER_OF.pop(q,None)
                    neu=dict(legalisieren(inseln,zust,lage,nachpins=False))
                    assert set(neu) == set(saved), 'Leistungs-Refit darf keine Teile verlieren'
                    lage.clear();lage.update(neu)
                    kompakt_refit({q for a in affected for q in pg[a]})
                    bel,_=neu_index()
                    keep_abs={kr:kp_abs(kr,lage[kr],klein=True) for kr in KPLK if kr in lage}
                    count=sum(problem(r,s,bel,mates(r)) for r,s in lage.items())
                    POWER.clear();POWER.update(pg);POWER_OF.clear();POWER_OF.update(po)
                    if count < best[0]:
                        POWER_REFIT.update(affected)
                        print('  Leistungskerne neu geformt und wieder starr: %s, %d -> %d Regelprobleme' % (','.join(sorted(affected)),best[0],count),flush=True)
                    else:
                        keep_abs = lage_zurueck(lage, saved)
                        POWER_REFIT.clear();POWER_REFIT.update(pr)
    if nachpins and PHYSIK and MODUS=='neu' and args.get('pin_nachlegal','0')=='1':
        kompakt_refit({r for r in PIN_ENT if r in lage and r not in POWER_OF},paar=True,kreuz_hart=True)
        print('  Einzel-C nach Legalisierung an heutige Pins nachoptimiert: %s (legal, Kreuzungen nicht erhoeht)' % ','.join(sorted(PIN_NACHLEGAL)),flush=True)
    return lage


FREIRAUM = args.get('freiraum', '1') == '1'

def freiraum(r, st0, bel, problem, raster=0.1, n_max=400, drehen=False, geometrie=None):
    ""
    from PIL import Image, ImageDraw
    from scipy import ndimage
    x0, y0, th0, s0 = st0
    bx = inner.bounds; nx = int((bx[2] - bx[0]) / raster) + 1; ny = int((bx[3] - bx[1]) / raster) + 1
    def px(x, y): return ((x - bx[0]) / raster, (y - bx[1]) / raster)
    img0 = Image.new('1', (nx, ny), 0); d0 = ImageDraw.Draw(img0)
    for g in getattr(inner, 'geoms', [inner]):
        d0.polygon([px(*c) for c in g.exterior.coords], fill=1)
        for h in g.interiors: d0.polygon([px(*c) for c in h.coords], fill=0)
    geo = geometrie or T[r].geo
    seiten = set(geo(*st0).keys())
    for s_ in seiten:
        for ref, g in bel.items.get(s_, []):
            if ref == r: continue
            for gg in getattr(g, 'geoms', [g]):
                if gg.is_empty or not hasattr(gg, 'exterior'): continue
                d0.polygon([px(*c) for c in gg.exterior.coords], fill=0)
        sp = sperr_von(r).get(s_)
        if sp is not None:
            for gg in getattr(sp[0], 'geoms', [sp[0]]):
                if gg.is_empty or not hasattr(gg, 'exterior'): continue
                d0.polygon([px(*c) for c in gg.exterior.coords], fill=0)
    frei = np.array(img0, dtype=bool)
    if PHYSIK:

        edt = ndimage.distance_transform_edt(np.pad(frei, 1, constant_values=False))[1:-1, 1:-1] * raster
    else:
        edt = ndimage.distance_transform_edt(frei) * raster
    beste = None
    ths = [th0] + ([(th0 + 90) % 360, (th0 + 180) % 360, (th0 + 270) % 360] if gruppen_drehbar(r, chip=drehen) else [])
    for th in ths:
        g0 = geo(0.0, 0.0, th, s0); b = [min(g.bounds[0] for g in g0.values()), min(g.bounds[1] for g in g0.values()),
                                               max(g.bounds[2] for g in g0.values()), max(g.bounds[3] for g in g0.values())]
        halb = 0.5 * min(b[2] - b[0], b[3] - b[1])
        ys, xs = np.nonzero(edt >= halb)
        if not len(xs): continue
        cx = bx[0] + (xs + 0.5) * raster - (b[0] + b[2]) / 2; cy = bx[1] + (ys + 0.5) * raster - (b[1] + b[3]) / 2
        d = np.hypot(cx - x0, cy - y0); idx = np.argsort(d)[:n_max]
        for i in idx:
            st = (float(cx[i]), float(cy[i]), th, s0)
            if not problem(r, st, bel):
                c = float(d[i]) + 0.2 * (th != th0)
                if beste is None or c < beste[0]: beste = (c, st)
                break
    if beste is None and PHYSIK and geometrie is not None:
        from platzierung.freiraum_gruppen import suche
        beste = suche(r,st0,bel,inner,geo,problem,ths,raster,n_max,sperr_von(r))
        if beste: print('  Nichtrechteckige Gruppe %s im Konfigurationsraum an freien Platz (%.1f mm)' % (r,beste[0]),flush=True)
    return beste


KREUZ = float(args.get('kreuz', '0'))
KREUZ_MAXPADS = 16


def mst_kanten(pts):
    ""
    n = len(pts)
    if n < 2: return []
    rest = list(range(1, n)); out = []
    best = {j: (math.dist(pts[0], pts[j]), 0) for j in rest}
    while rest:
        j = min(rest, key=lambda j: best[j][0]); i = best[j][1]
        out.append((pts[i][0], pts[i][1], pts[j][0], pts[j][1])); rest.remove(j)
        for k in rest:
            d = math.dist(pts[j], pts[k])
            if d < best[k][0]: best[k] = (d, j)
    return out


def kreuzungen(P, Q):
    ""
    if len(P) == 0 or len(Q) == 0: return 0
    ax, ay, bx, by = (P[:, i][:, None] for i in range(4))
    cx, cy, dx, dy = (Q[:, i][None, :] for i in range(4))
    def orient(px, py, qx, qy, rx, ry): return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    o1 = orient(ax, ay, bx, by, cx, cy); o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay); o4 = orient(cx, cy, dx, dy, bx, by)
    return int(np.count_nonzero((o1 * o2 < -1e-9) & (o3 * o4 < -1e-9)))


def kennzahlen(lage):
    ""
    pos = collections.defaultdict(list)
    for r in T:
        s = lage.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        if r not in lage and s[3] != fps[r]['side']: pass
        for nr, net, x, y, ps in (T[r].pads_abs(*s) if r in lage else
                                  [(nr, net, fps[r]['x'] + lx, fps[r]['y'] + ly, fps[r]['side']) for nr, net, lx, ly, tht in T[r].pads]):
            if net in NETZ and net not in GND: pos[net].append((x, y))
    tot = wt = 0.0; schnell = 0.0
    segs = [(np.array(mst_kanten(pts), float).reshape(-1, 4)) for n, pts in sorted(pos.items()) if 2 <= len(pts) <= KREUZ_MAXPADS]
    kreuz = sum(kreuzungen(segs[i], segs[j]) for i in range(len(segs)) for j in range(i + 1, len(segs)))
    for n, pts in pos.items():
        if len(pts) < 2: continue
        inT = [0]; rest = list(range(1, len(pts))); Lm = 0.0
        while rest:
            i, j = min(((i, j) for i in inT for j in rest), key=lambda ij: math.dist(pts[ij[0]], pts[ij[1]]))
            Lm += math.dist(pts[i], pts[j]); inT.append(j); rest.remove(j)
        tot += Lm; wt += wnet(n) * Lm
        if NETZ[n]['art'] == 'schnell': schnell += Lm
    ent = []
    for a, I in INSELN.items():
        if a not in T: continue
        sa = lage.get(a) or (fps[a]['x'], fps[a]['y'], 0.0, fps[a]['side'])
        ap = {nr: (x, y, s) for nr, net, x, y, s in (T[a].pads_abs(*sa) if a in lage else
              [(nr, net, fps[a]['x'] + lx, fps[a]['y'] + ly, fps[a]['side']) for nr, net, lx, ly, tht in T[a].pads])}
        for e in I['entkopplung']:
            r = e['ref']
            if r not in T or e.get('ueber_spule'): continue
            sr = lage.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
            cp = [(x, y) for nr, net, x, y, s in (T[r].pads_abs(*sr) if r in lage else
                  [(nr, net, fps[r]['x'] + lx, fps[r]['y'] + ly, fps[r]['side']) for nr, net, lx, ly, tht in T[r].pads]) if net == e['netz']]
            pins = [ap[n][:2] for n in e['pins'] if n in ap]
            if cp and pins: ent.append(min(math.dist(c, p) for c in cp for p in pins))
    bel = Belegung(); kol = 0
    for r in T:
        s = lage.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        g = T[r].geo(*s) if r in lage else T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        if bel.kollision(g, luft=0.00485, ignore=mates(r)): kol += 1
        bel.add(r, g)
    return dict(luftlinie=round(tot, 1), gewichtet=round(wt, 1), schnell=round(schnell, 1), kreuzungen=kreuz,
                entkopplung_mittel=round(sum(ent) / len(ent), 2) if ent else None, entkopplung_max=round(max(ent), 2) if ent else None,
                kollisionen=kol)


def regel_kennzahlen(lage):
    ""
    st = {r: lage.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']) for r in T}
    out = dict(in_sperrflaeche=sorted(r for r in T if not TEILE.get(r, {}).get('fest') and sperr_flaeche(r, T[r].geo(*st[r])) > 1e-6))
    if PADRAND is not None:
        out['pad_rand_exakt_mm'] = float((RG or {}).get('router',{}).get('edge',RAND))
        out['pad_rand_verletzt'] = sorted(r for r in T if not TEILE.get(r,{}).get('fest') and not PADRAND.frei(r,st[r]))
    out.update(Paare(st, alle=True).gesamt())
    ql = [pin_abst(T[c].pads_abs(*st[c]), T[z].pads_abs(*st[z]), n, p) / 4.0 for c, (z, n, p) in PIN.items() if c not in PIN_ENT]
    out['quarz_last_mittel'] = round(sum(ql) / len(ql), 2) if ql else None
    ww = [(r, weg(r, st[r], st)) for r in T if r in WT and r not in FOLGE and not TEILE.get(r, {}).get('fest')]
    out['ww_preis'] = round(sum(WWM.preis(WT[r], 0.0, w) for r, w in ww), 1)
    out['ww_ueber_max'] = sorted(r for r, w in ww if w > WT[r]['max'] + 1e-3)
    out['ww_ausnahmen'] = {r: dict(LEGAL_AUSNAHMEN.get(r) or dict(grund='nicht durch Legalisierung begruendet', nachweis='fehlt'),
                                     endweg_mm=round(weg(r, st[r], st), 4)) for r in out['ww_ueber_max']}
    out['chips_weg_mm'] = round(sum(w for r, w in ww if TEILE.get(r, {}).get('rolle') == 'chip'), 2)
    out['gedreht_verboten'] = sorted(r for r in T if r in WT and not WT[r]['drehen'] and TEILE.get(r, {}).get('rolle') == 'klein'
                                     and round(st[r][2]) % 360 != 0)
    out['gruppen_abweichung_mm'] = round(max([math.dist(st[f][:2], folge(f, st[l])[:2]) for f, (l, _, _) in FOLGE.items()] or [0.0]), 4)
    if PHYSIK or args.get('insel_messung') == '1':
        out.update(PHYS.messen(st))
        out['leistung_gruppen'] = POWER
        out['leistung_neugeformt'] = sorted(POWER_REFIT)
        out['pin_nachlegal_bewegt'] = sorted(PIN_NACHLEGAL)
        fixed = sorted(r for r in T if TEILE.get(r, {}).get('fest'))
        gs = {r: T[r].geo(*st[r]) for r in fixed}
        out['feste_kollisionen'] = [[r,q] for i,r in enumerate(fixed) for q in fixed[i+1:]
                                  if any(g.distance(gs[q][s]) < 0.00485 for s,g in gs[r].items() if s in gs[q])]
    return out


def p35_nachlauf(inseln,zust,lage):
    global P35_BERICHT
    karte = P35Karte(B,K,RG,T,lage,args)
    fest = innen_fest(inseln,lage)
    beweglich = [r for r in sorted(lage) if r not in fest and r not in FOLGE and r not in GRUPPE and r not in POWER_OF]
    erlaubt = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'],fps[r]['y'],0.,fps[r]['side']).values()) for r in lage}
    def legal(candidate):
        saved = dict(LEGAL_AUSNAHMEN)
        power, power_of, refit, pins = dict(POWER), dict(POWER_OF), set(POWER_REFIT), set(PIN_NACHLEGAL)
        try: result = legalisieren(inseln,zust,dict(candidate),nachpins=False)
        finally:
            LEGAL_AUSNAHMEN.clear(); LEGAL_AUSNAHMEN.update(saved)
            POWER.clear(); POWER.update(power); POWER_OF.clear(); POWER_OF.update(power_of)
            POWER_REFIT.clear(); POWER_REFIT.update(refit); PIN_NACHLEGAL.clear(); PIN_NACHLEGAL.update(pins)

        anchors={r for r,_,_,_,_ in karte.fanouts}
        if any(result[r] != lage[r] for r in lage if r not in beweglich or r in karte.terminals or r in anchors or not r.startswith(('C','R'))): return None
        if any(math.dist(result[r][:2],lage[r][:2])>1.+1e-9 for r in lage): return None
        bel = Belegung()
        for r,st in result.items(): bel.add(r,T[r].geo(*st))
        for r in beweglich:
            gs=T[r].geo(*result[r])
            if bel.kollision(gs,ignore=set(mates(r))|{r}): return None
            if any(not inner_p.contains(Point(x,y)) for _,_,x,y,_ in T[r].pads_abs(*result[r])): return None
            if sum(g.difference(inner).area for g in gs.values()) > erlaubt[r]+1e-3: return None
            if sperr_flaeche(r,gs)>1e-7: return None
            if PADRAND is not None and not PADRAND.frei(r,result[r]): return None
            if WW_AN and r in WT and weg(r,result[r],result)>WT[r]['max']+1e-6 and weg(r,result[r],result)>weg(r,lage[r],lage)+1e-9: return None
            if any(k.intersects(g) for kr in KPLK if kr!=r for k in kp_abs(kr,result[kr],klein=True) for g in gs.values()): return None
        return result
    lage,info = p35_lokal(karte,lage,beweglich,darf_drehen,legal,pin_kosten)
    P35_BERICHT = dict(version='p35-v2',grobbaum=P35_KARTE.design,endbaum=karte.design,**info)
    print('P35 lokale Zuege:',len(info['zuege']),'/',info['versuche'],flush=True)
    return lage


def ausgabe(inseln, zust, lage):
    if not lage: lage = gesamt_lage(inseln, zust)
    parts = {}
    for r, s in lage.items():
        if TEILE.get(r, {}).get('fest') and abs(s[0] - fps[r]['x']) < 1e-6 and abs(s[1] - fps[r]['y']) < 1e-6 and s[2] == 0: continue
        parts[r] = dict(side=s[3], pads={nr: [round(x, 4), round(y, 4)] for nr, net, x, y, ps in T[r].pads_abs(*s)},
                        x=round(s[0], 4), y=round(s[1], 4), drehung=s[2])
        if DRCL:
            parts[r]['transformation_eindeutig'] = True
            parts[r]['padinstanzen'] = [dict(num=nr, net=net, x=round(x, 4), y=round(y, 4), side=ps)
                                       for nr, net, x, y, ps in T[r].pads_abs(*s)]
    umriss = None; grund = 'umriss_fest' if VG.get('umriss_fest') else 'dicht=0'
    if DICHT and args.get('dicht_rand', '1') == '1':
        umriss, grund = DM.enger_umriss(B, {r: T[r].geo(*lage.get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r in T}, RAND)
    dichte_ausgabe = {}
    if DICHT or DICHT_INNEN or DICHT_INSELN or args.get('dichte_messen') == '1':
        dv = DM.messen(B, RG, teile=T); dn = DM.messen(dict(B, outline=umriss or B['outline']), RG, lage, T)
        dichte_ausgabe['dichte'] = dict(vorher=dv, nachher=dn, gewicht=DICHT, umriss_grund=grund)
        fest = innen_fest(inseln, lage)
        dichte_ausgabe['p26_innen'] = dict(vorher=DM.innen(B, K, RG, teile=T, fest=fest),
                                         nachher=DM.innen(B, K, RG, lage, T, fest=fest), gewicht=DICHT_INNEN)
        if DICHT_INSELN:
            dichte_ausgabe['p26_inseln'] = dict(vorher=DM.insel_messen(B, K, RG, teile=T, fest=fest),
                                              nachher=DM.insel_messen(B, K, RG, lage, T, fest=fest), gewicht=DICHT_INSELN)
    vorher = kennzahlen({}); nachher = kennzahlen(lage)
    print('Kennzahlen vorher ', vorher)
    print('Kennzahlen nachher', nachher)
    rv = regel_kennzahlen({}); rn = regel_kennzahlen(lage)
    print('Regeln vorher ', rv)
    print('Regeln nachher', rn)
    from platzierung.kupferreserve import absolut as reserve_absolut
    json.dump(dict(parts=parts, **(dict(meister=MEISTER_BELEG) if MEISTER_BELEG is not None else {}), **(dict(kupferreserve=reserve_absolut(KUPFERRESERVE,T,lage,fps)) if KUPFERRESERVE else {}), inseln={I.name: I.refs for I in inseln if len(I.refs) > 1},
                   **(dict(p3_pin_gruppen=dict(version='p3-pin-gruppen-v1', teile={
                       r: dict(ref=a, netz=n, pins=sorted(pins)) for r, (a, n, pins) in sorted(PIN_ENT.items())
                       if pins and TEILE[r].get('funktion', '').startswith('entkopplung_')
                       and not (TEILE[r].get('stuetzt') or {}).get('ueber_spule')})) if P3_SCHLEIFE else
                      (dict(p3_pin_gruppen=K['p3_pin_gruppen']) if K.get('p3_pin_gruppen') else {})),
                   zustand={k: list(v) for k, v in zust.items()}, kennzahlen=dict(vorher=vorher, nachher=nachher),
                   **(dict(p26_fest=dict(version='p26-fest-v1', refs=sorted(innen_fest(inseln, lage))))
                      if DICHT or DICHT_INNEN or args.get('dichte_messen') == '1' or args.get('fest_anker') == '1'
                      or args.get('bedien_neu_frei') == '1'
                      or innen_fest(inseln, lage) - {r for r in lage if TEILE.get(r, {}).get('fest')
                                                    or TEILE.get(r, {}).get('rolle') in ('mechanik', 'ohne')} else {}),
                   regeln=dict(vorher=rv, nachher=rn), optionen=args, **dichte_ausgabe,
                   **(dict(p35=P35_BERICHT) if P35_BERICHT is not None else {}),
                   **(dict(routbarkeit=dict(version=ROUTBARKEIT.version,endpruefung_erforderlich=True)) if ROUTBARKEIT is not None else {}),
                   **(dict(outline=umriss, outline_original=B['outline']) if umriss else {})), open(OUT, 'w'), indent=1)
    if 'bild' in args: bild(lage, inseln, args['bild'])
    print('fertig in %.0fs ->' % (time.time() - t_start), OUT)


def bild(lage, inseln, prefix):
    from PIL import Image, ImageDraw, ImageFont
    bx = board.bounds; S = 40.0 if (bx[2] - bx[0]) < 60 else 12.0
    W, H = int((bx[2] - bx[0] + 4) * S), int((bx[3] - bx[1] + 4) * S)
    tr = lambda x, y: ((x - bx[0] + 2) * S, (y - bx[1] + 2) * S)
    FARBE = {'digital': (60, 110, 205), 'analog': (40, 160, 80), 'leistung': (205, 60, 60), 'gemischt': (150, 90, 190),
             'neutral': (130, 130, 130)}
    try: font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 12)
    except OSError: font = None
    owner = {r: I for I in inseln for r in I.refs}
    for side in ('F', 'B'):
        im = Image.new('RGB', (W, H), 'white'); d = ImageDraw.Draw(im)
        for g in getattr(board, 'geoms', [board]): d.polygon([tr(*p) for p in g.exterior.coords], outline=(0, 0, 0))
        if SPERR.get(side) is not None:
            for gg in getattr(SPERR[side], 'geoms', [SPERR[side]]):
                if hasattr(gg, 'exterior'): d.polygon([tr(*p) for p in gg.exterior.coords], fill=(225, 225, 225), outline=(150, 150, 150))
        for r, s in lage.items():
            I = owner.get(r); art = INSELN.get(I.anker, {}).get('art', 'neutral') if I else 'neutral'
            col = FARBE.get(art, (130, 130, 130))
            for sd, g in T[r].geo(*s).items():
                if sd != side: continue
                for gg in getattr(g, 'geoms', [g]):
                    light = tuple(int(c + (255 - c) * (0.55 if (I is None or r != I.anker) else 0.25)) for c in col)
                    d.polygon([tr(*p) for p in gg.exterior.coords], fill=light, outline=col)
            if s[3] == side and T[r].n_pads >= 5 and font: d.text(tr(s[0], s[1]), r, fill=(0, 0, 0), font=font)

        pos = collections.defaultdict(list)
        for r, s in lage.items():
            for nr, net, x, y, ps in T[r].pads_abs(*s):
                if NETZ.get(net, {}).get('art') in ('schnell', 'sw'): pos[net].append((x, y))
        for n, pts in pos.items():
            for p, q in zip(pts[:-1], pts[1:]): d.line([tr(*p), tr(*q)], fill=(230, 120, 0), width=1)
        if font: d.text((10, 10), 'Seite %s  (blau digital, gruen analog, rot Leistung, lila gemischt; orange: schnelle Netze; grau: Sperrflaeche)' % side,
                        fill=(0, 0, 0), font=font)
        im.save('%s_%s.png' % (prefix, side))
    print('Bilder', prefix + '_F.png', prefix + '_B.png')


if __name__ == '__main__':
    main()
