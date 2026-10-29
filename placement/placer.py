"""Island-based placer for KiCad boards.

1. Build islands: each chip is the anchor of an island. Decoupling capacitors go first, smallest value first and as
   close as possible to the supply pin they serve (also on the opposite side under the chip), then inductors and
   bootstrap parts, output capacitors, and the remaining small parts by importance (fast > current > analog > GPIO).
   Courtyards may touch but never overlap.
2. Global placement: islands are rigid blocks (side and 90-degree rotation) placed by simulated annealing. The cost
   combines weighted airwires (fast nets weigh most), overlap, board edge, keepouts, separation between switching or
   fast pads and sensitive analog pads, and springs towards rough positions from specs.json ('coarse').
3. Refinement and legalisation: parts and islands are moved in small steps until no collisions are left.

mode=new places a fresh board; mode=refine refines an already placed, dense board, keeping every move
collision-free.

usage: python3 placement/placer.py <board.json> <netclasses.json> <placement.json> [design_rules=design_rules.json]
       [mode=new|refine] [sides=FB] [iter=30000] [seed=1] [cross=0] [separate=3.0]
       [via_cost=0.6] [image=<prefix>]
cross=<weight>: each crossing of two airwires costs as much as <weight> mm of weighted airwire length (0 = off).
The result is applied to a KiCad board with placement/apply_place.py, or without KiCad with
common/shift.py (as learning/selfplay.py does).
"""
import json, sys, os, math, random, collections, time
import fnmatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from common.process_guard import bootstrap
if __name__ == "__main__": bootstrap(heavy=True)
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely import affinity
from common.parts import Occupancy, load_parts, circuit_board, AGAINST, keepout_areas, height
from common.values import type_
from common import travel_range as WWM
from analysis import density as DM
from placement.island_physics import IslandDesignRules

t_start = time.time()
B = json.load(open(sys.argv[1])); K = json.load(open(sys.argv[2])); OUT = sys.argv[3]
args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
from learning.champion import apply as _champion_apply
CHAMPION_EVIDENCE = _champion_apply(args, B, 'placer')
RG = json.load(open(args['design_rules'])) if 'design_rules' in args else None
DRCL = str(args.get('drc_model', (RG or {}).get('placer', {}).get('drc_model', '0'))) == '1'
PAD_SPACING = float(args.get('pad_spacing', (RG or {}).get('placer', {}).get('pad_spacing_drc' if DRCL else 'pad_spacing', 0.15)))
BORDER = float(args.get('border', (RG or {}).get('placer', {}).get('border', 0.2)))
SIDES = args.get('sides', 'FB')
VIA = float(args.get('via_cost', '0.6'))
SEPARATE = float(args.get('separate', '3.0'))
MODE = args.get('mode', 'new')
if args.get('p35', '0') not in ('0', '1'): raise ValueError('p35=0|1')
P35 = args.get('p35', '0') == '1'
P35_MAP = None; P35_REPORT = None
if P35:
    from placement.p35_slot import Map as P35Map, local as p35_local, positive as p35_positive
    P35_W = p35_positive(args.get('p35_w', '4'), 'p35_w')
random.seed(int(args.get('seed', '1'))); np.random.seed(int(args.get('seed', '1')))

ROUTABILITY = None
if args.get('routability', '0') == '1':
    from learning.ai.routability_service import Service as RoutService
    if args.get('routability_final_check') != '1':
        raise ValueError('routability requires independent final check: routability_final_check=1')
    ROUTABILITY = RoutService(args.get('routability_model'), deployment='placer')
    if ROUTABILITY.predictor is None: raise ValueError('Routability cost term locked: ' + ROUTABILITY.reason)
    RW = float(args.get('routability_w', '1'))
    if not math.isfinite(RW) or RW < 0: raise ValueError('routability_w')

NET = K['net_list']; GND = set(K['gnd']); PARTS = K['part_list']; ISLANDS = K['islands']; CONS = K.get('specs') or {}
DENSE = float(args.get('dense', '0'))
if not math.isfinite(DENSE) or DENSE < 0: raise ValueError('dense must be finite and >= 0')
if CONS.get('contour_fixed', False): DENSE = 0.0
DENSE_ISLANDS = float(args.get('dense_islands', '0'))
if not math.isfinite(DENSE_ISLANDS) or DENSE_ISLANDS < 0: raise ValueError('dense_islands must be finite and >= 0')
DENSE_GROUPS = args.get('dense_groups', '0')
if DENSE_GROUPS not in ('0', '1'): raise ValueError('dense_groups must be 0 or 1')
DENSE_GROUPS = DENSE_GROUPS == '1'
DENSE_INNER = float(args.get('dense_inner', '0'))
if not math.isfinite(DENSE_INNER) or DENSE_INNER < 0: raise ValueError('dense_inner must be finite and >= 0')
fps = {f['ref']: f for f in B['fps']}
if MODE == 'new' and args.get('operate_new_free', '0') == '1':


    for r, t in PARTS.items():
        if t.get('operate') and not fps[r].get('locked') and not any(fnmatch.fnmatchcase(r, p) for p in CONS.get('pinned', [])):
            t['pinned'] = False
            if r in ISLANDS: ISLANDS[r]['pinned'] = False
if DRCL:
    geometry_board = dict(B, drc_model=True)
else:

    geometry_board = dict(B, pads=[{k: v for k, v in p.items() if k != 'hole_polys'} for p in B['pads']], outline_holes=[])
T = load_parts(geometry_board, PAD_SPACING)
COPPER_RESERVE = []
if args.get('copper_reserve') == '1':
    from placement.copper_reserve import apply as reserve_apply
    COPPER_RESERVE = reserve_apply(T, B, K, RG)
PHYSICS = args.get('island_physics', '0') == '1'
PHYS = IslandDesignRules(T, K, B) if PHYSICS or args.get('island_measurement') == '1' else None
BGA_AGAINST = args.get('bga_against', '0') == '1' and MODE == 'new' and args.get('sides_original') != '1'
POWER = {}; POWER_OF = {}; POWER_REFIT = set(); PIN_POST_LEGALIZE = set()
board, inner, inner_p, _ = circuit_board(geometry_board, BORDER)
DENSE_NORM = max(1.0, math.hypot(board.bounds[2] - board.bounds[0], board.bounds[3] - board.bounds[1]))
PAD_EDGE = None
if PHYSICS and args.get('pad_border_exact','0') == '1':
    from placement.pad_border import PadBorder
    PAD_EDGE = PadBorder(PHYS.polys,fps,board,float((RG or {}).get('router',{}).get('edge',BORDER)))

KEEP = [(k['ref'], Polygon(q).buffer(0)) for k in B.get('keepouts', []) if k['ref'] for q in k['polys'] if len(q) >= 3]


def net_weight(n):
    k = NET.get(n)
    if not k or n in GND: return 0.0
    a = k['category']
    if a == 'fast': return 2.0 + 0.5 * k['rank']
    if a == 'sw': return 4.0
    if a == 'current_val': return {'high': 3.0, 'medium': 2.5, 'small': 1.5}[k['current_val']]
    if a == 'supply': return {'high': 1.0, 'medium': 0.6, 'small': 0.5}[k['current_val']]
    if a == 'analog': return 1.5 + 0.5 * k['rank']
    return 1.0 + 0.3 * k.get('rank', 0)


def via_k(a, b): return 0.0 if a == b or 'FB' in (a, b) else VIA


DIRS = [(math.cos(2 * math.pi * k / 16), math.sin(2 * math.pi * k / 16)) for k in range(16)]
SPIRAL = sorted([(i * 0.1, j * 0.1, math.hypot(i, j) * 0.1) for i in range(-100, 101) for j in range(-100, 101)
                   if math.hypot(i, j) <= 100] +
                  [(i * 0.25, j * 0.25, math.hypot(i, j) * 0.25) for i in range(-160, 161) for j in range(-160, 161)
                   if 40 < math.hypot(i, j) <= 160], key=lambda t: t[2])




KEEPOUT_ON = args.get('blocked', '1') == '1'
KEEPOUT = keepout_areas(B) if KEEPOUT_ON else {'F': None, 'B': None}
KEEPOUT_BB = {s: g.bounds for s, g in KEEPOUT.items() if g is not None}
KEEPOUT_H = []
if KEEPOUT_ON:
    for z in CONS.get('keepout_height') or []:
        g = unary_union([Polygon(q).buffer(0) for q in z.get('polys', []) if len(q) >= 3]).buffer(0.005)
        for s in z.get('board_side', 'FB'):
            if s in ('F', 'B') and not g.is_empty: KEEPOUT_H.append((s, float(z.get('height_max', 0.0)), g))
HEIGHT = {r: height(fps[r].get('fp', ''), r, CONS.get('heights')) for r in T}
_SPK = {}


def keepout_from(r):
    h = HEIGHT.get(r, 1.5) if KEEPOUT_H else None
    if h not in _SPK:
        out = {}
        for s in ('F', 'B'):
            gs = ([KEEPOUT[s]] if KEEPOUT.get(s) is not None else []) + [g for ss, hm, g in KEEPOUT_H if ss == s and h > hm]
            if gs:
                u = unary_union(gs) if len(gs) > 1 else gs[0]
                out[s] = (u, u.bounds)
        _SPK[h] = out
    return _SPK[h]

def keepout_area(r, geo):
    sp = keepout_from(r)
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


def kp_abs(kr, s, small=False):
    return [affinity.translate(affinity.rotate(g, s[2], origin=(0, 0)), s[0], s[1]) for g in (KPLK if small else KPL)[kr]]


WT = WWM.table(K)
WW_ON = MODE == 'refine' and args.get('travel_range', '1') == '1'
WW_F = float(args.get('ww_f', '1.0'))
ROTATE_RULE = args.get('rotate_rule', '1') == '1'
SUPPORTS = {}
for _r, _t in PARTS.items():
    _s = (_t.get('supports') or {}).get('ref')
    if _r in T and _s in T and _s != _r: SUPPORTS[_r] = _s


def may_rotate(r):
    return PARTS.get(r, {}).get('role') == 'small' and (not ROTATE_RULE or WT.get(r, {}).get('rotate', True))


def way(r, s, layer_name, ch=None):
    x0, y0 = fps[r]['x'], fps[r]['y']
    c = SUPPORTS.get(r)
    if c is not None and (c in layer_name or (ch and c in ch)):
        sc = ch[c] if ch and c in ch else layer_name[c]
        x0 += sc[0] - fps[c]['x']; y0 += sc[1] - fps[c]['y']
    return math.hypot(s[0] - x0, s[1] - y0)


PIN_LEGAL = args.get('pin_legal', '1') == '1'
CRYSTAL_PIN = args.get('crystal_pin', '1') == '1'
PIN = {}
for _a, _I in ISLANDS.items():
    for _e in _I['decoupling']:
        if not _e.get('over_coil') and _e['ref'] in T and _a in T: PIN[_e['ref']] = (_a, _e['net_name'], set(_e['pins']))
PIN_DEC = dict(PIN)
if PHYSICS:
    PIN.update(PHYS.pin)
    PIN_DEC = {r: v for r,v in PIN.items() if type_(r,fps[r].get('fp','')) == 'C'}
P3_LOOP = args.get('p3_loop', '0') == '1'
if P3_LOOP:
    for _c, _z in (K.get('p3_pin_groups') or {}).get('part_list', {}).items():
        PIN[_c] = PIN_DEC[_c] = (_z['ref'], _z['net_name'], set(_z['pins']))
    from placement.decoupling import loop as p3_loop
if CRYSTAL_PIN:
    for _r, _t in sorted(PARTS.items()):
        _s = _t.get('supports') or {}
        if _t.get('function') == 'crystal_load' and _r in T and _s.get('crystal') in T and _r not in PIN:
            PIN[_r] = (_s['crystal'], _s.get('net_name'), None)
PIN_OF = collections.defaultdict(list)
for _c, (_z, _n, _p) in sorted(PIN.items()): PIN_OF[_z].append(_c)


def pin_dist(cpads, zpads, n, pins, ref=None):
    cp = [(x, y, sd) for nr, net, x, y, sd in cpads if net == n]
    zp = [(x, y, sd) for nr, net, x, y, sd in zpads if (nr in pins if pins is not None else net == n)]
    if not cp or not zp: return 0.0
    prior = 4.0 * min(math.hypot(p[0] - q[0], p[1] - q[1]) + via_k(p[2], q[2]) for p in cp for q in zp)
    if P3_LOOP and ref in PIN_DEC:
        loop = p3_loop(cpads, zpads, n, pins, GND, via_k)
        if loop is not None:
            C = PARTS.get(ref, {}).get('C') or 100e-9
            w = 1.0 if C <= 220e-9 * 1.001 else (.4 if C <= 4.7e-6 * 1.001 else .1)
            return prior + w * loop
    return prior



def build_groups():
    if args.get('group_list', '1') != '1': return {}
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


SEQUENCE = build_groups()
GROUP = collections.defaultdict(list)
for _f, (_l, _dx, _dy) in sorted(SEQUENCE.items()): GROUP[_l].append(_f)
FIXED_EXTRA = {_l for _f, (_l, _dx, _dy) in SEQUENCE.items() if PARTS.get(_f, {}).get('pinned')}
if SEQUENCE: print('Rigid groups (net tie at the foot point):', ', '.join('%s->%s' % (f, l) for f, (l, _, _) in sorted(SEQUENCE.items())))


def mates(r):
    l = SEQUENCE[r][0] if r in SEQUENCE else r
    return {l, *GROUP.get(l, ())}


def sequence(f, sl):
    l, dx, dy = SEQUENCE[f]; X, Y, TH, S = sl
    sf = fps[f]['side']
    if S != fps[l]['side']: dx = -dx; sf = AGAINST[sf]
    c, s = math.cos(math.radians(TH)), math.sin(math.radians(TH))
    return (X + c * dx - s * dy, Y + s * dx + c * dy, TH, sf)


def groups_rotatable(r, chip=False):
    if not (may_rotate(r) or chip): return False
    return all(not PARTS.get(q, {}).get('pinned') and
               (not ROTATE_RULE or WT.get(q, {}).get('rotate', True))
               for q in movement_refs(r))


def layer_back(layer_name, stored):
    layer_name.clear(); layer_name.update(stored)
    return {r: kp_abs(r, layer_name[r], small=True) for r in KPLK if r in layer_name}


def with_group(r, s, layer_name=None):
    if layer_name is not None and (s[2] - layer_name[r][2]) % 360 and not groups_rotatable(r, chip=True):
        raise ValueError('Group rotation forbidden: ' + r)
    out = {r: s}
    for f in GROUP.get(r, ()): out[f] = sequence(f, s)
    if layer_name is not None and r in POWER:
        dr = s[2]-layer_name[r][2]
        c, sn = math.cos(math.radians(dr)), math.sin(math.radians(dr))
        for q in POWER[r]:
            if q == r or q not in layer_name: continue
            x, y = layer_name[q][0]-layer_name[r][0], layer_name[q][1]-layer_name[r][1]
            out[q] = (s[0]+c*x-sn*y, s[1]+sn*x+c*y, (layer_name[q][2]+dr)%360, layer_name[q][3])
    return out


def movement_refs(r):
    if r not in POWER_OF and r not in POWER: return mates(r)
    a = POWER_OF.get(r, r)
    return set(POWER.get(a, [a])) | mates(a)


if args.get('sides_board', '1') == '1' and len(SIDES) > 1 and args.get('sides_original') != '1':
    _on = [r for r in T if T[r].n_pads and board.contains(Point(fps[r]['x'], fps[r]['y']))]
    _sd = sorted({fps[r]['side'] for r in _on})
    if len(_on) >= 2 and len(_sd) == 1:
        print('Sides: board populated on one side (%s), sides=%s -> %s (P16; sides_board=0 switches off)' % (_sd[0], SIDES, _sd[0]))
        SIDES = _sd[0]


RANK = args.get('rank', '1') == '1'
RANK_P20 = {'crystal': 3.0, 'crystal_load': 3.5, 'series_r': 4.0, 'decoupling_medium': 5.0, 'bootstrap': 5.0, 'feedback': 5.5,
            'feedback_c': 5.5, 'setting_r': 5.5, 'shunt': 5.5, 'decoupling_large': 6.0, 'misc': 6.0, 'divider': 6.5,
            'rc_element': 6.5, 'pullup': 7.0, 'pulldown': 7.0}
COARSE = CONS.get('coarse') or {}

def feder_part(r, s):
    g = COARSE.get(r)
    if not g: return 0.0
    d = math.hypot(s[0] - g.get('x', s[0]), s[1] - g.get('y', s[1]))
    return 2.0 * max(0.0, d - g.get('radius', 0.5)) ** 2



SEPARATE_W = float(args.get('separate_w', '1.0')); SEPARATE_PAD = float(args.get('separate_pad', '2.0'))
AGG = {}; VIC = set()
for _n, _k in NET.items():
    _a = _k.get('category')
    if _a == 'sw': AGG[_n] = 2.0
    elif _a == 'fast' and _k.get('rank', 0) >= 3: AGG[_n] = 1.0
    if (_a == 'analog' and _k.get('rank', 0) >= 2) or _k.get('kelvin') or _k.get('sensitive_sw'): VIC.add(_n)
_gg = args.get('gnd_against', 'auto')
GND_AGAINST = _gg == '1' or (_gg == 'auto' and str(((RG or {}).get('router') or {}).get('vip', '0')) == '1')
try: _vd = float(str(((RG or {}).get('router') or {}).get('via', '0.45/0.2')).split('/')[0])
except ValueError: _vd = 0.45
GG_RV = float(args.get('gnd_via_r', str(_vd / 2 + float(((RG or {}).get('router') or {}).get('clr', 0.1)))))
GG_W = float(args.get('gnd_against_w', '2.0'))
if args.get('bga_via_against','0') not in ('0','1'): raise ValueError('bga_via_against=0|1')
BGA_VIA_AGAINST = args.get('bga_via_against','0') == '1'
if BGA_VIA_AGAINST and str(((RG or {}).get('router') or {}).get('vip','0')) != '1':
    raise ValueError('bga_via_against needs explicit vip=1 in the board contract')
BGA_VIA_W = float(args.get('bga_via_w','80'))
if BGA_VIA_AGAINST and (not math.isfinite(BGA_VIA_W) or BGA_VIA_W <= 0): raise ValueError('bga_via_w must be finite and positive')
BGA_VIA_R = _vd/2 + max(float(((RG or {}).get('router') or {}).get('drcpad',((RG or {}).get('router') or {}).get('drc',.1))),
                         float(((RG or {}).get('router') or {}).get('clr',.1)))
def bga_via_pins(board, footprints, nets, grounds, rules):
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
            if ring>=minimum or p.get('pintype')=='power_in' or net in grounds or nets.get(net,{}).get('category') in ('supply','current_val'):
                chosen.add((ref,str(p['num'])))
    return chosen

BGA_VIA_PADS = bga_via_pins(B,fps,NET,GND,RG) if BGA_VIA_AGAINST else set()
FAN_OUT = float(args.get('fan_out', '0')); FAN_OUT_W = float(args.get('fan_out_w', '0.5'))
SIDX = {'F': 0, 'B': 1, 'FB': 2}


def _f_separate(wa, wt):
    def f(a, b):
        d = np.hypot(a['x'][:, None] - b['x'][None, :], a['y'][:, None] - b['y'][None, :])
        return wt * wa[a['i']][:, None] * np.maximum(0.0, SEPARATE_PAD - d) ** 2
    return f


def _f_gg(wt):
    def f(a, b):
        against = (b['sd'][None, :] == 2) | (b['sd'][None, :] == 1 - a['sd'][:, None])
        dx = np.maximum(np.abs(a['x'][:, None] - b['x'][None, :]) - b['hx'][None, :], 0.0)
        dy = np.maximum(np.abs(a['y'][:, None] - b['y'][None, :]) - b['hy'][None, :], 0.0)
        return wt * np.where(against, np.maximum(0.0, GG_RV - np.hypot(dx, dy)) / GG_RV, 0.0)
    return f


def _f_fan_out(a, b):
    equal = (a['sd'][:, None] == b['sd'][None, :]) | (a['sd'][:, None] == 2) | (b['sd'][None, :] == 2)
    dx = np.maximum(np.abs(a['x'][:, None] - b['x'][None, :]) - a['hx'][:, None] - b['hx'][None, :], 0.0)
    dy = np.maximum(np.abs(a['y'][:, None] - b['y'][None, :]) - a['hy'][:, None] - b['hy'][None, :], 0.0)
    return FAN_OUT_W * np.where(equal, np.maximum(0.0, FAN_OUT - np.hypot(dx, dy)) ** 2, 0.0)


def _f_channel(net_name, radius, wt, via=False):
    def f(a, b):
        same = (a['sd'][:, None] == b['sd'][None, :]) | (a['sd'][:, None] == 2) | (b['sd'][None, :] == 2)
        if via: same = ~same | (b['sd'][None, :] == 2)
        foreign = net_name[a['i']][:, None] != net_name[b['i']][None, :]
        dx = np.maximum(np.abs(a['x'][:, None]-b['x'][None, :])-b['hx'][None, :], 0)
        dy = np.maximum(np.abs(a['y'][:, None]-b['y'][None, :])-b['hy'][None, :], 0)
        if not via:
            dx = np.maximum(dx-a['hx'][:, None], 0); dy = np.maximum(dy-a['hy'][:, None], 0)
        return wt * np.where(same & foreign, np.maximum(0, radius[a['i']][:, None]-np.hypot(dx,dy))**2, 0)
    return f


class Pairs:
    def __init__(self, layer_name, all_items=False):
        self.refs = sorted(r for r in layer_name if T[r].n_pads)
        self.idx = {}; rows = []
        for r in self.refs:
            self.idx[r] = np.arange(len(rows), len(rows) + T[r].n_pads)
            rows += [(r, k) for k in range(T[r].n_pads)]
        self.N = len(rows)
        rid = {r: i for i, r in enumerate(self.refs)}
        self.ref = np.array([rid[r] for r, k in rows], int)
        net = [T[r].pads[k][1] for r, k in rows]
        tht = np.array([T[r].pads[k][4] for r, k in rows], bool)
        role = [PARTS.get(r, {}).get('role') for r, k in rows]
        self.X = np.zeros(self.N); self.Y = np.zeros(self.N); self.HX = np.zeros(self.N); self.HY = np.zeros(self.N)
        self.SD = np.zeros(self.N, int)
        for r in self.refs: self.assign(r, layer_name[r])
        self.terms = []
        if SEPARATE_W > 0 or all_items:
            A = np.array([n in AGG for n in net], bool); Bm = np.array([n in VIC for n in net], bool)
            self._term('separate', A, Bm, _f_separate(np.array([AGG.get(n, 0.0) for n in net]), 1.0 if all_items else SEPARATE_W))
        if GND_AGAINST or all_items:
            self._term('gnd_against', np.array([n in GND for n in net], bool) & ~tht, np.array([n not in GND for n in net], bool),
                       _f_gg(1.0 if all_items else GG_W))
        if FAN_OUT > 0:
            self._term('fan_out', np.array([x == 'chip' for x in role], bool), np.array([x == 'connector' for x in role], bool), _f_fan_out)
        if BGA_VIA_AGAINST:
            bga = np.array([(r,str(T[r].pads[k][0])) in BGA_VIA_PADS for r,k in rows],bool) & ~tht
            self._term('bga_via_free',bga,np.array([bool(n) for n in net],bool),
                       _f_channel(np.array(net),np.full(self.N,BGA_VIA_R),BGA_VIA_W,via=True))
        if PHYSICS:
            live = np.array([bool(n) and n not in GND for n in net], bool)
            cpin = np.array([r in PIN_DEC for r, _ in rows], bool) & ~tht
            self._term('c_via_free', cpin, np.array([bool(n) for n in net], bool),
                       _f_channel(np.array(net), np.full(self.N, GG_RV), 20.0, via=True))

            rr = np.array([float(NET.get(n, {}).get('w', 0.1) or 0.1) + PAD_SPACING for n in net])
            self._term('channel', live, live, _f_channel(np.array(net), rr, float(args.get('channel_w', '0.25'))))

    def _term(self, name, A, Bm, f):
        if not A.any() or not Bm.any(): return
        has = {r: (bool(A[i].any()), bool(Bm[i].any())) for r, i in self.idx.items()}
        self.terms.append(dict(name=name, A=A, B=Bm, Ai=np.nonzero(A)[0], Bi=np.nonzero(Bm)[0], f=f, has=has))

    def value_list(self, r, s):
        pa = T[r].pads_abs(*s); ph = T[r].pads_half(s[2])
        return (np.array([p[2] for p in pa]), np.array([p[3] for p in pa]), np.array([h[0] for h in ph]),
                np.array([h[1] for h in ph]), np.array([SIDX[p[4]] for p in pa], int))

    def assign(self, r, s):
        i = self.idx.get(r)
        if i is None or not len(i): return
        self.X[i], self.Y[i], self.HX[i], self.HY[i], self.SD[i] = self.value_list(r, s)

    def _rows(self, i):
        return dict(i=i, x=self.X[i], y=self.Y[i], hx=self.HX[i], hy=self.HY[i], sd=self.SD[i], ref=self.ref[i])

    @staticmethod
    def _sum(f, a, b):
        if not len(a['i']) or not len(b['i']): return 0.0
        M = f(a, b)
        return float(np.where(a['ref'][:, None] != b['ref'][None, :], M, 0.0).sum())

    def contribution(self, ch):
        if not self.terms: return 0.0
        refs = [r for r in ch if r in self.idx and len(self.idx[r])]
        tot = 0.0; C = None
        for t in self.terms:
            ha = any(t['has'][r][0] for r in refs); hb = any(t['has'][r][1] for r in refs)
            if not ha and not hb: continue
            if C is None:
                i = np.concatenate([self.idx[r] for r in refs])
                v = [self.value_list(r, ch[r]) for r in refs]
                C = dict(i=i, x=np.concatenate([q[0] for q in v]), y=np.concatenate([q[1] for q in v]),
                         hx=np.concatenate([q[2] for q in v]), hy=np.concatenate([q[3] for q in v]),
                         sd=np.concatenate([q[4] for q in v]), ref=self.ref[i])
                inch = np.zeros(self.N, bool); inch[i] = True
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

    def total(self):
        out = {}
        for t in self.terms:
            a = self._rows(t['Ai']); b = self._rows(t['Bi'])
            out[t['name']] = round(self._sum(t['f'], a, b), 2)
            if t['name'] == 'separate':
                d = np.hypot(a['x'][:, None] - b['x'][None, :], a['y'][:, None] - b['y'][None, :])
                d = np.where(a['ref'][:, None] != b['ref'][None, :], d, np.inf)
                out['separate_min_mm'] = round(float(d.min()), 2) if d.size else None
            if t['name'] == 'gnd_against':
                M = _f_gg(1.0)(a, b); M = np.where(a['ref'][:, None] != b['ref'][None, :], M, 0.0)
                out['gnd_against_occupied'] = int((M.max(axis=1) > 0).sum()) if M.size else 0
            if t['name'] in ('c_via_free','bga_via_free'):
                M = t['f'](a,b); M = np.where(a['ref'][:, None] != b['ref'][None,:], M, 0)
                out['c_via_occupied' if t['name']=='c_via_free' else 'bga_via_occupied'] = int((M.max(axis=1) > 0).sum()) if M.size else 0
        return out



def island_build(a, I):
    S = fps[a]['side']
    bel = Occupancy(); st = {a: (0.0, 0.0, 0.0, S)}
    bel.add(a, T[a].geo(0, 0, 0, S))
    placed = collections.defaultdict(list)
    def add_pads(r):
        for nr, net, x, y, s in T[r].pads_abs(*st[r]): placed[net].append((x, y, s, r))
    def del_pads(r):
        for net in list(placed):
            placed[net] = [q for q in placed[net] if q[3] != r]
    add_pads(a)
    agnd = [(x, y, s) for nr, net, x, y, s in T[a].pads_abs(0, 0, 0, S) if net in GND]

    under = T[a].geo(0, 0, 0, S)[S].buffer(0.2)
    dec = {e['ref']: e for e in I['decoupling'] if e['ref'] in T}

    apads = {nr: (x, y, s) for nr, net, x, y, s in T[a].pads_abs(0, 0, 0, S)}
    grp = {}
    for e in dec.values():
        if e['net_name'] in grp: continue
        pins = [apads[n] for n in e['pins'] if n in apads]
        cl = []
        for p in pins:
            for c in cl:
                if any(math.hypot(p[0] - q[0], p[1] - q[1]) < 1.0 for q in c): c.append(p); break
            else: cl.append([p])
        cl.sort(key=lambda c: -len(c))
        grp[e['net_name']] = [dict(pins=c, n=0) for c in cl]
    goal = {}
    mem = [r for r in I['members'] if r in T and not PARTS[r]['pinned'] and r not in SEQUENCE]
    def actual_sw(r): return type_(r, fps[r].get('fp', '')) == 'L' or any(NET.get(net, {}).get('category') == 'sw' for _, net, *_ in T[r].pads)
    e1 = [r for r in mem if r in dec and not dec[r].get('over_coil')]
    e1.sort(key=lambda r: dec[r]['C'])
    sw = [r for r in mem if r not in dec and actual_sw(r)]
    if PHYSICS: sw.sort(key=lambda r: type_(r,fps[r].get('fp','')) != 'L')
    e2 = sorted([r for r in mem if r in dec and dec[r].get('over_coil')], key=lambda r: dec[r]['C'])
    rest = [r for r in mem if r not in e1 and r not in sw and r not in e2]
    if RANK:


        def rg(r): return RANK_P20.get(PARTS[r].get('function'), 6.0)
        rest = sorted(rest, key=rg)
        if sw: order = e1 + sw + e2 + rest
        else:
            fr = [r for r in rest if rg(r) <= 4.0]
            e1k = [r for r in e1 if PARTS[r].get('function') in ('decoupling_small', None)]
            order = e1k + fr + [r for r in e1 if r not in e1k] + e2 + [r for r in rest if r not in fr]
    else:
        order = e1 + sw + e2 + rest

    def goal_pins(r):


        if PHYSICS and r in PHYS.pin:
            z,n,pins = PHYS.pin[r]
            if z in st: return [(x,y,sd) for num,net,x,y,sd in T[z].pads_abs(*st[z]) if num in pins]
        return (goal.get(r) or {}).get('pins', [])

    def cost(r, x, y, th, side):
        c = 0.0
        isent = r in dec
        for nr, net, px, py, ps in T[r].pads_abs(x, y, th, side):
            if net in GND:
                if agnd: c += (0.4 if isent else 0.1) * min(math.hypot(px - gx, py - gy) + via_k(ps, gs) for gx, gy, gs in agnd)
                continue
            zp = goal_pins(r)
            if zp and net == (PIN[r][1] if PHYSICS and r in PHYS.pin else dec[r]['net_name']):
                c += 4.0 * min(math.hypot(px - qx, py - qy) + via_k(ps, qs) for qx, qy, qs in zp)
                continue
            L = [q for q in placed.get(net, []) if q[3] != r]
            if not L: continue
            c += net_weight(net) * min(math.hypot(px - q[0], py - q[1]) + via_k(ps, q[2]) for q in L)
        c += 0.05 * math.hypot(x, y)
        if side != S: c += 0.3
        if P3_LOOP and r in PIN_DEC and PIN[r][0] in st:
            z, n, pins = PIN[r]
            cp = T[r].pads_abs(x, y, th, side)
            zp = T[z].pads_abs(*st[z])
            c += pin_dist(cp, zp, n, pins, r) - pin_dist(cp, zp, n, pins)
        if PHYSICS:
            ps = {q: T[q].pads_abs(*v) for q, v in st.items()}
            ps[r] = T[r].pads_abs(x, y, th, side)
            c += PHYS.cost({r: (x,y,th,side)}, ps)
        return c

    def candidates(r):
        sides = [fps[r]['side']] if args.get('sides_original') == '1' else [S]
        if args.get('sides_original') != '1' and 'F' in SIDES and 'B' in SIDES and T[r].n_pads <= 4 and not any(p[4] for p in T[r].pads) and PARTS[a]['role'] == 'chip':
            sides.append(AGAINST[S])
        if BGA_AGAINST and 'BGA' in fps[a].get('fp', '') and r in PIN_DEC and T[r].n_pads <= 4 and not any(p[4] for p in T[r].pads):
            sides = sorted(set(sides + [AGAINST[S]]))
        res = []
        for side in sides:
            for th in (0.0, 90.0, 180.0, 270.0):
                pads = T[r]._rot(th, side)[1]

                tgt = None
                zp = goal_pins(r)
                if zp:
                    k = [p for p in pads if p[1] == (PIN[r][1] if PHYSICS and r in PIN else dec[r]['net_name'])]
                    if k:
                        pin = min(zp, key=lambda q: math.hypot(q[0], q[1]))
                        tgt = (pin[0] - k[0][2], pin[1] - k[0][3])
                if tgt is None:
                    best = None
                    for nr, net, px, py, ps in pads:
                        if net in GND or not placed.get(net): continue
                        w = net_weight(net)
                        q = min(placed[net], key=lambda q: math.hypot(q[0], q[1]))
                        if best is None or w > best[0]: best = (w, q[0] - px, q[1] - py)
                    tgt = (best[1], best[2]) if best else (0.0, 0.0)
                res.append((side, th, tgt))
        return res

    def assign(r):
        best = None
        for side, th, (x0, y0) in candidates(r):
            def vacant(x, y): return not bel.collision(T[r].geo(x, y, th, side))
            cands = []
            if vacant(x0, y0): cands.append((x0, y0))
            else:
                for dx, dy in DIRS:
                    prev, d, hit = 0.0, 0.1, None
                    while d <= 10.0:
                        if vacant(x0 + dx * d, y0 + dy * d): hit = d; break
                        prev = d; d += 0.1 if d < 2.0 else 0.25
                    if hit is None: continue
                    lo, hi = prev, hit
                    for _ in range(4):
                        mid = (lo + hi) / 2
                        if vacant(x0 + dx * mid, y0 + dy * mid): hi = mid
                        else: lo = mid
                    cands.append((x0 + dx * hi, y0 + dy * hi))
            for x, y in cands:
                if side != S and not under.contains(T[r].geo(x, y, th, side)[side].centroid): continue
                c = cost(r, x, y, th, side)
                if best is None or c < best[0]: best = (c, x, y, th, side)
        return best

    for r in order:
        if P3_LOOP and r in (K.get('p3_pin_groups') or {}).get('part_list', {}):
            z, n, pins = PIN_DEC[r]
            goal[r] = dict(pins=[(x, y, sd) for num, net, x, y, sd in T[z].pads_abs(*st[z]) if num in pins])
        elif PHYSICS and r in PHYS.pin and PIN[r][0] in st:
            z, n, pins = PIN[r]
            goal[r] = dict(pins=[(x,y,sd) for num, net, x,y,sd in T[z].pads_abs(*st[z]) if num in pins])
        elif r in dec:
            g = grp.get(dec[r]['net_name'])
            if g:
                tgt = min(g, key=lambda c: c['n']); tgt['n'] += 1; goal[r] = tgt
                if P3_LOOP and r in PIN_DEC:


                    pins = {nr for nr, p in apads.items() if p in tgt['pins'] and nr in dec[r]['pins']}
                    PIN[r] = (a, dec[r]['net_name'], pins)
                    PIN_DEC[r] = PIN[r]
        b = assign(r)
        if b is None: continue
        st[r] = b[1:]; bel.add(r, T[r].geo(*st[r])); add_pads(r)

    for rnd in range(2):
        for r in order:
            if r not in st: continue
            old = st[r]
            bel.remove(r); del_pads(r)
            c_old = cost(r, *old)
            b = assign(r)
            if b is not None and b[0] < c_old - 1e-3: st[r] = b[1:]
            bel.add(r, T[r].geo(*st[r])); add_pads(r)
    return st



def rot_pt(x, y, R):
    c, s = math.cos(math.radians(R)), math.sin(math.radians(R)); return c * x - s * y, s * x + c * y


class Island:
    def __init__(self, name, rel, pinned):
        self.name = name; self.rel = rel; self.pinned = pinned
        self.refs = list(rel)
        self.cache = {}
        pads = []
        for r, s in rel.items():
            for nr, net, x, y, ps in T[r].pads_abs(*s): pads.append((r, nr, net, x, y, ps))
        self.pads = pads

        self.keep = [(kr, affinity.translate(g, -fps[kr]['x'], -fps[kr]['y'])) for kr, g in KEEP if kr in rel]
        a = self.refs[0]
        self.anchor = a

    def local(self, R):
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


def total_layer(islands, state):
    out = {}
    for I in islands:
        X, Y, R = state[I.name]
        for r, (x, y, th, s) in I.rel.items():
            xr, yr = rot_pt(x, y, R); out[r] = (X + xr, Y + yr, (th + R) % 360, s)
    return out


def main():
    global P35_MAP
    print('Parts', len(T), 'Islands', len(ISLANDS), 'Nets', len(NET), 'Mode', MODE)
    if MODE == 'refine':
        islands = [Island(r, {r: (0.0, 0.0, 0.0, fps[r]['side'])}, bool(PARTS[r]['pinned']) or PARTS[r]['role'] in ('mechanics', 'without'))
                  for r in T]
        layer_name = {r: (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']) for r in T}
        state = {r: (fps[r]['x'], fps[r]['y'], 0.0) for r in T}
        if P35: P35_MAP = P35Map(B,K,RG,T,layer_name,args)
        layer_name = fine_tuning(islands, layer_name, hard_from=0.0)
        layer_name = legalize(islands, state, layer_name)
        if P35: layer_name = p35_post_run(islands,state,layer_name)
        return outputs(islands, state, layer_name)

    rels = {}
    t0 = time.time()
    for a, I in sorted(ISLANDS.items(), key=lambda kv: -len(kv[1]['members'])):
        if a not in T: continue
        rels[a] = island_build(a, I)

    for f, (l, dx, dy) in sorted(SEQUENCE.items()):
        for a, st in rels.items():
            if l in st: st[f] = sequence(f, st[l]); break
    print('Islands built in %.1fs' % (time.time() - t0))
    in_island = {r for st in rels.values() for r in st}
    islands = []
    for a, st in rels.items():
        pinned = bool(PARTS[a]['pinned']) or args.get('fixed_anchor') == '1'
        islands.append(Island(a, st, pinned))

    for r, t in PARTS.items():
        if r in in_island or r not in T or r in SEQUENCE: continue
        if t['role'] == 'without' and not T[r].pads and not T[r].has_copper_graphic: continue
        rel = {r: (0.0, 0.0, 0.0, fps[r]['side'])}
        for f in GROUP.get(r, ()):
            if f not in in_island: rel[f] = sequence(f, rel[r])
        islands.append(Island(r, rel, bool(t['pinned']) or t['role'] in ('mechanics', 'without') or r in FIXED_EXTRA))
    IDX = {I.name: I for I in islands}
    state = {}
    for I in islands:
        a = I.anchor; state[I.name] = (fps[a]['x'], fps[a]['y'], 0.0)
    coarse = CONS.get('coarse') or {}
    if MODE == 'new' and args.get('only') != 'islands':


        bx = inner.bounds
        for I in islands:
            if I.pinned: continue
            if args.get('chance') != '1' and inner_p.contains(Point(state[I.name][0], state[I.name][1])): continue
            state[I.name] = (random.uniform(bx[0], bx[2]), random.uniform(bx[1], bx[3]), random.choice((0.0, 90.0, 180.0, 270.0)))
    for a, g in coarse.items():
        if a in state: state[a] = (g.get('x', state[a][0]), g.get('y', state[a][1]), float(g.get('rot', state[a][2])))
    if args.get('only') == 'islands':
        return outputs(islands, state, {})


    net_isl = collections.defaultdict(set)
    for I in islands:
        for r, nr, net, x, y, ps in I.pads:
            if net in NET and net not in GND: net_isl[net].add(I.name)
    NETS = {n: s for n, s in net_isl.items() if len(s) > 1}
    W = {n: net_weight(n) for n in NETS}
    isl_nets = collections.defaultdict(list)
    for n, s in NETS.items():
        for i in s: isl_nets[i].append(n)

    allowed = {}
    for I in islands:
        X, Y, R = (fps[I.anchor]['x'], fps[I.anchor]['y'], 0.0)
        u, _, _ = I.local(0.0)
        allowed[I.name] = sum(affinity.translate(g, X, Y).difference(inner).area for g in u.values()) if MODE != 'new' or I.pinned else 0.0
    AVC = {}
    def agg_vic(I, R):
        if (I.name, R) in AVC: return AVC[(I.name, R)]
        _, pads, _ = I.local(R)
        ag = [(x, y) for r, nr, net, x, y, ps in pads if NET.get(net, {}).get('category') == 'sw' or
              (NET.get(net, {}).get('category') == 'fast' and NET[net]['rank'] >= 3)]
        vi = [(x, y) for r, nr, net, x, y, ps in pads if NET.get(net, {}).get('category') == 'analog' and NET[net]['rank'] >= 2]
        f = lambda L: (sum(p[0] for p in L) / len(L), sum(p[1] for p in L) / len(L), len(L)) if L else None
        AVC[(I.name, R)] = (f(ag), f(vi))
        return AVC[(I.name, R)]

    WO = [20.0]
    def pads_abs(I, st):
        X, Y, R = st; _, pads, _ = I.local(R)
        return [(net, X + x, Y + y) for r, nr, net, x, y, ps in pads if net in NETS]
    padcache = {I.name: pads_abs(I, state[I.name]) for I in islands}
    def hpwl(n, over):
        xs, ys = [], []
        for i in NETS[n]:
            for net, x, y in (over[i] if i in over else padcache[i]):
                if net == n: xs.append(x); ys.append(y)
        return (max(xs) - min(xs) + max(ys) - min(ys)) if xs else 0.0
    def geo_abs(I, st):
        X, Y, R = st; u, _, kp = I.local(R)
        return {s: affinity.translate(g, X, Y) for s, g in u.items()}, [affinity.translate(g, X, Y) for g in kp]
    geo_cache = {I.name: geo_abs(I, state[I.name]) for I in islands}
    DZ = DM.HullTarget({r: g[0] for r, g in geo_cache.items()}) if DENSE else None
    DZI = DM.HullTarget({r: g[0] for r, g in geo_cache.items()},
                      {I.name for I in islands if not I.pinned}) if DENSE_INNER else None
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
    def keepout_island(geos):
        a = 0.0
        for s, g in geos.items():
            k = KEEPOUT.get(s)
            if k is None: continue
            kb = KEEPOUT_BB[s]; b = g.bounds
            if b[0] < kb[2] and kb[0] < b[2] and b[1] < kb[3] and kb[1] < b[3]: a += g.intersection(k).area
        return a
    def outer(I, ga):
        a = 0.0
        for g in ga[0].values():
            if not inner_p.contains(g): a += g.difference(inner).area
        return max(0.0, a - allowed[I.name] - 1e-3) + keepout_island(ga[0])
    avc = {I.name: agg_vic(I, state[I.name][2]) for I in islands}
    def sep(i, sti, j, stj):
        ai, vi = agg_vic(IDX[i], sti[2]) if sti is not None else avc[i]
        aj, vj = agg_vic(IDX[j], stj[2]) if stj is not None else avc[j]
        Xi, Yi, _ = sti or state[i]; Xj, Yj, _ = stj or state[j]
        p = 0.0
        for A, Ao, V, Vo in ((ai, (Xi, Yi), vj, (Xj, Yj)), (aj, (Xj, Yj), vi, (Xi, Yi))):
            if A and V:
                d = math.hypot(A[0] + Ao[0] - V[0] - Vo[0], A[1] + Ao[1] - V[1] - Vo[1])
                if d < SEPARATE: p += (SEPARATE - d) ** 2 * math.sqrt(min(A[2], V[2]))
        return p
    sep_isl = [I.name for I in islands if any(avc[I.name])]
    def feder(i, st):
        g = coarse.get(i)
        if not g: return 0.0
        d = math.hypot(st[0] - g.get('x', st[0]), st[1] - g.get('y', st[1]))
        return 2.0 * max(0.0, d - g.get('radius', 0.5)) ** 2

    def local_cost(ch):
        over = {i: pads_abs(IDX[i], s) for i, s in ch.items()}
        gx = {i: geo_abs(IDX[i], s) for i, s in ch.items()}
        nets = set()
        for i in ch: nets.update(isl_nets[i])
        c = sum(W[n] * hpwl(n, over) for n in sorted(nets))
        names = list(ch)
        for i in names:
            c += WO[0] * outer(IDX[i], gx[i]) + feder(i, ch[i])
            for J in islands:
                j = J.name
                if j in ch: continue
                c += WO[0] * overlap(gx[i], geo_cache[j])
        for a_ in range(len(names)):
            for b_ in range(a_ + 1, len(names)): c += WO[0] * overlap(gx[names[a_]], gx[names[b_]])
        for i in names:
            if i not in sep_isl: continue
            for j in sep_isl:
                if j == i or (j in ch and j < i): continue
                c += 2.0 * sep(i, ch[i], j, ch.get(j))
        if DZ: c += DENSE * DZ.surface({k: g[0] for k, g in gx.items()}) / DENSE_NORM
        if DZI: c += DENSE_INNER * DZI.surface({k: g[0] for k, g in gx.items()}) / DENSE_NORM
        return c, over, gx

    movable = [I.name for I in islands if not I.pinned]
    N = int(args.get('iter', str(min(60000, max(8000, 600 * len(movable))))))
    if not movable: N = 0
    bx = inner.bounds; diag = math.hypot(bx[2] - bx[0], bx[3] - bx[1])
    def total():
        c = sum(W[n] * hpwl(n, {}) for n in sorted(NETS))
        for I in islands:
            c += WO[0] * outer(I, geo_cache[I.name]) + feder(I.name, state[I.name])
        for a_ in range(len(islands)):
            for b_ in range(a_ + 1, len(islands)): c += WO[0] * overlap(geo_cache[islands[a_].name], geo_cache[islands[b_].name])
        if DZ: c += DENSE * DZ.surface() / DENSE_NORM
        if DZI: c += DENSE_INNER * DZI.surface() / DENSE_NORM
        return c
    c_start = total()
    T0 = float(args.get('t0', '0' if MODE == 'refine' else '0')) or max(1.0, 0.05 * c_start / max(1, len(movable)))
    if MODE == 'refine': T0 *= 0.2
    sig0 = diag / 4 if MODE == 'new' else min(2.0, diag / 10)
    acc = 0; t1 = time.time()
    amount = None
    if args.get('amount', '0') == '1':
        from learning.value_placer import CoarseValue
        amount = CoarseValue(B, K, RG, T, args)
        print('Value model coarse:', amount.logbook, flush=True)
    for it in range(N):
        f = it / max(1, N)
        Tt = T0 * (1 - f) ** 2 + 1e-3
        WO[0] = 20.0 + 380.0 * f
        sig = max(0.05, sig0 * (1 - f) ** 1.5)
        mv = random.random()
        i = random.choice(movable)
        if mv < 0.75:
            X, Y, R = state[i]; ch = {i: (min(bx[2], max(bx[0], X + random.gauss(0, sig))), min(bx[3], max(bx[1], Y + random.gauss(0, sig))), R)}
        elif mv < 0.92:
            X, Y, R = state[i]; ch = {i: (X, Y, (R + random.choice((90.0, 180.0, 270.0))) % 360)}
        else:
            j = random.choice(movable)
            if j == i: continue
            ch = {i: (state[j][0], state[j][1], state[i][2]), j: (state[i][0], state[i][1], state[j][2])}
        c_old, _, _ = local_cost({k: state[k] for k in ch})
        c_new, over, gx = local_cost(ch)
        d = c_new - c_old
        if amount is not None and it % amount.all_items == 0:
            candidate = dict(state, **ch)
            d += max(1., c_start / max(1, len(movable))) * amount.delta(total_layer(islands, state), total_layer(islands, candidate))
        if d < 0 or random.random() < math.exp(-d / Tt):
            for k, s in ch.items():
                state[k] = s; padcache[k] = over[k]; geo_cache[k] = gx[k]
                if DZ: DZ.assign(k, gx[k][0])
                if DZI and k in DZI.bounds: DZI.assign(k, gx[k][0])
                avc[k] = agg_vic(IDX[k], s[2])
            acc += 1
        if it % max(1, N // 8) == 0:
            print('  SA %6d/%d  T %.3f  cost %.1f  accepted %d  (%.0fs)' % (it, N, Tt, total(), acc, time.time() - t1), flush=True)
    if amount is not None:
        args['value_logbook'] = amount.logbook
        print('Value model coarse end:', amount.logbook, flush=True)
    WO[0] = 400.0
    print('Rough placement: cost %.1f -> %.1f (%.0fs)' % (c_start, total(), time.time() - t1))

    layer_name = total_layer(islands, state)
    if PHYSICS and args.get('power_fixed', '1') == '1':
        for a, rs in PHYS.group_list.items():
            if all(r in layer_name and not PARTS[r]['pinned'] and r not in SEQUENCE and r not in GROUP for r in rs):
                POWER[a] = rs
                for r in rs: POWER_OF[r] = a
    if P35: P35_MAP = P35Map(B,K,RG,T,layer_name,args)
    if args.get('fine', '1') == '1': layer_name = fine_tuning(islands, layer_name)
    layer_name = legalize(islands, state, layer_name)
    if P35: layer_name = p35_post_run(islands,state,layer_name)
    return outputs(islands, state, layer_name)


def inner_fixed(islands, layer_name):
    pinned = {r for I in islands for r in I.refs if PARTS.get(r, {}).get('pinned') or (I.pinned and I.anchor == r)} | (FIXED_EXTRA & set(layer_name))


    for r in list(pinned):
        pinned.update(mates(r) & set(layer_name))
    return pinned


def fine_tuning(islands, layer_name, hard_from=0.7):
    t0 = time.time()
    pinned = inner_fixed(islands, layer_name)
    mov = [r for r in layer_name if r not in pinned and r not in SEQUENCE and POWER_OF.get(r, r) == r]
    if not mov: return layer_name
    pads = {r: T[r].pads_abs(*layer_name[r]) for r in layer_name}
    net_refs = collections.defaultdict(set)
    for r, ps in pads.items():
        for nr, net, x, y, sd in ps:
            if net in NET and net not in GND: net_refs[net].add(r)
    W = {n: net_weight(n) for n in net_refs}
    PZ = {c: z for c, z in PIN.items() if c in layer_name and z[0] in layer_name}
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
    KNETS = sorted(n for n in net_refs if len(net_pts(n, {})) <= CROSS_MAXPADS) if CROSS > 0 else []
    NID = {n: i for i, n in enumerate(KNETS)}
    SEG = {n: np.array(mst_edges(net_pts(n, {})), float).reshape(-1, 4) for n in KNETS}
    def build_all():
        arrs = [SEG[n] for n in KNETS if len(SEG[n])]
        if not arrs: return np.zeros((0, 4)), np.zeros(0, int)
        return np.concatenate(arrs), np.concatenate([np.full(len(SEG[n]), NID[n]) for n in KNETS if len(SEG[n])])
    ALL = [build_all()]
    def cross_term(nets, ov):
        nets = sorted(n for n in nets if n in NID)
        if not nets: return 0.0, {}
        new = {n: np.array(mst_edges(net_pts(n, ov)), float).reshape(-1, 4) for n in nets}
        A, ids = ALL[0]
        rest = A[~np.isin(ids, [NID[n] for n in nets])]
        c = sum(crossings(new[n], rest) for n in sorted(nets))
        for i in range(len(nets)):
            for j in range(i + 1, len(nets)): c += crossings(new[nets[i]], new[nets[j]])
        return CROSS * c, new
    def pin_term(c, ov):
        z, n, pins = PZ[c]
        return pin_dist(ov[c] if c in ov else pads[c], ov[z] if z in ov else pads[z], n, pins, c)
    bel = Occupancy()
    geo = {r: T[r].geo(*layer_name[r]) for r in layer_name}
    DZ = DM.HullTarget(geo) if DENSE else None
    DZI = DM.HullTarget(geo, DM.movable(K, layer_name, pinned)) if DENSE_INNER else None
    DIS = DM.IslandTarget(geo, DM.island_groups(K, layer_name, pinned)) if DENSE_ISLANDS else None
    for r in layer_name: bel.add(r, geo[r])

    allowed = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']).values()) for r in layer_name}
    def outer(r, g):
        a = sum(gg.difference(inner).area for gg in g.values() if not inner_p.contains(gg))
        return max(0.0, a - allowed[r] - 1e-3) + keepout_area(r, g)
    KR = sorted(kr for kr in KPL if kr in layer_name) if KEEPOUT_ON else []
    KPABS = {kr: kp_abs(kr, layer_name[kr]) for kr in KR}
    def kp_over(ch, gx):
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
            for k in ks[kr]: a += bel.overlap({'F': k, 'B': k}, ignore=set(ch))[0]
        return a
    PF = Pairs(layer_name)
    if not PF.terms: PF = None
    else: print('  Fine-tuning pair terms:', ', '.join(t['name'] for t in PF.terms), flush=True)
    WO = [10.0]
    def cost(ch):
        ov = {r: T[r].pads_abs(*s) for r, s in ch.items()}
        gx = {r: T[r].geo(*s) for r, s in ch.items()}
        nets = set()
        for r in ch:
            for nr, net, x, y, sd in ov[r]:
                if net in net_refs: nets.add(net)
        c = sum(W[n] * hpwl(n, ov) for n in sorted(nets))
        kt, seg_new = cross_term(nets, ov) if CROSS > 0 else (0.0, {})
        c += kt
        ents = set()
        for r in ch:
            if r in PZ: ents.add(r)
            ents.update(PZ_OF.get(r, []))
        c += sum(pin_term(e, ov) for e in sorted(ents))
        for r in ch:
            a_, _ = bel.overlap(gx[r], ignore=set(ch))
            c += WO[0] * (a_ + outer(r, gx[r])) + feder_part(r, ch[r])
            if WW_ON and r in WT and r not in SEQUENCE: c += WW_F * WWM.price(WT[r], 0.0, way(r, ch[r], layer_name, ch))
        c += WO[0] * kp_over(ch, gx)
        rs = list(ch)
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                if rs[j] in mates(rs[i]): continue
                for sd in gx[rs[i]]:
                    if sd in gx[rs[j]]: c += WO[0] * gx[rs[i]][sd].intersection(gx[rs[j]][sd]).area
        if PF is not None: c += PF.contribution(ch)
        if DZ: c += DENSE * DZ.surface(gx) / DENSE_NORM
        if DZI: c += DENSE_INNER * DZI.surface(gx) / DENSE_NORM
        if DIS: c += DENSE_ISLANDS * DIS.contribution(gx)
        if PHYSICS:
            c += PHYS.cost(ch, dict(pads, **ov))
        if P35_MAP is not None: c += P35_W * P35_MAP.cost(ch)
        return c, ov, gx, seg_new
    same = collections.defaultdict(list)
    for r in mov:
        if r not in GROUP: same[(fps[r]['fp'], T[r].n_pads)].append(r)
    N = int(args.get('fine_iter', str(min(80000, 400 * len(mov)))))
    acc = 0
    for it in range(N):
        f = it / N
        WO[0] = 10.0 + 490.0 * f
        Tt = 0.3 * (1 - f) ** 2 + 1e-3
        sig = 0.6 * (1 - f) + 0.03
        r = random.choice(mov); x, y, th, sd = layer_name[r]
        mvz = random.random(); swap = False
        if mvz < 0.8: ch = with_group(r, (x + random.gauss(0, sig), y + random.gauss(0, sig), th, sd), layer_name)
        elif mvz < 0.92 and groups_rotatable(r): ch = with_group(r, (x, y, (th + random.choice((90.0, 180.0, 270.0))) % 360, sd), layer_name)
        else:
            g = same.get((fps[r]['fp'], T[r].n_pads), [])
            q = random.choice(g) if len(g) > 1 else None
            if q is None or q == r or layer_name[q][3] != sd or r in GROUP or r in POWER or q in POWER: continue
            ch = {r: (layer_name[q][0], layer_name[q][1], layer_name[q][2], sd), q: (x, y, th, sd)}; swap = True
        for k in ch: bel.remove(k)
        c_old, _, gx_old, _ = cost({k: layer_name[k] for k in ch})
        c_new, ov, gx, seg_new = cost(ch)
        if ROUTABILITY is not None:
            c_new += RW * ROUTABILITY.fine_costs(B, K, RG, T, layer_name, ch,gnd_layers=args.get('gnd_layers'))
        d = c_new - c_old
        hard_ok = True
        if f >= hard_from:
            o_new = sum(bel.overlap(gx[k], ignore=set(ch))[0] + outer(k, gx[k]) for k in ch) + kp_over(ch, gx)
            o_old = sum(bel.overlap(gx_old[k], ignore=set(ch))[0] + outer(k, gx_old[k]) for k in ch) + kp_over({k: layer_name[k] for k in ch}, gx_old)
            hard_ok = o_new <= o_old + 1e-9
            if hard_ok:
                ks = list(ch)
                for k in ks:
                    if bel.collision(gx[k], ignore=set(ch)) and not bel.collision(gx_old[k], ignore=set(ch)): hard_ok = False
                if hard_ok and swap:
                    b2 = Occupancy(); b2.add(ks[1], gx[ks[1]])
                    if b2.collision(gx[ks[0]]): hard_ok = False
        if hard_ok and WW_ON:
            for k in ch:
                if k in SEQUENCE or k not in WT: continue
                wn = way(k, ch[k], layer_name, ch)
                if wn > WT[k]['max'] + 1e-6 and wn > way(k, layer_name[k], layer_name) + 1e-9: hard_ok = False; break
        if hard_ok and (d < 0 or random.random() < math.exp(-d / Tt)):
            for k, s in ch.items():
                layer_name[k] = s; pads[k] = ov[k]; geo[k] = gx[k]
                if DZ: DZ.assign(k, gx[k])
                if DZI and k in DZI.bounds: DZI.assign(k, gx[k])
                if DIS: DIS.assign(k, gx[k])
                if k in KPABS: KPABS[k] = kp_abs(k, s)
                if PF is not None: PF.assign(k, s)
            if seg_new: SEG.update(seg_new); ALL[0] = build_all()
            acc += 1
        for k in ch: bel.add(k, geo[k])
        if it % max(1, N // 6) == 0:
            print('  Fine tuning %6d/%d  accepted %d  (%.0fs)' % (it, N, acc, time.time() - t0), flush=True)
    if DENSE_GROUPS:
        from placement.group_compact import group_list, candidates
        binds = [set(v) | {r} for r, v in GROUP.items()] + list(POWER.values())
        binds += [{c, z[0]} for c, z in PIN.items()]
        groups = group_list(layer_name, [{a, *i['members']} for a, i in sorted(ISLANDS.items())], binds, pinned)
        WO[0] = 500.0
        accepted = 0
        for _ in range(2):
            for rs in groups:
                for ch in candidates(rs, layer_name, pads, net_refs, W):
                    old = {r: layer_name[r] for r in rs}
                    for r in rs: bel.remove(r)
                    co, _, _, _ = cost(old)
                    cn, ov, gx, sn = cost(ch)
                    nets = sorted({n for r in rs for _, n, _, _, _ in pads[r] if n in net_refs})


                    ok = cn < co - 1e-8 and kp_over(ch, gx) <= 1e-9
                    if ok:
                        ok = sum(W[n] * hpwl(n, ov) for n in nets) < sum(W[n] * hpwl(n, {}) for n in nets) - 1e-8
                        if ok and CROSS > 0: ok = cross_term(nets, ov)[0] <= cross_term(nets, {})[0] + 1e-9
                    if ok:
                        ok = all(not bel.collision(gx[r], ignore=set(rs)) and outer(r, gx[r]) <= 1e-9
                                 and all(inner_p.contains(Point(x, y)) for _, _, x, y, _ in ov[r])
                                 and (PAD_EDGE is None or PAD_EDGE.vacant(r, ch[r])) for r in rs)
                    if ok and WW_ON:
                        ok = all(r in SEQUENCE or r not in WT or way(r, ch[r], layer_name, ch) <= WT[r]['max'] + 1e-6 for r in rs)
                    if ok and ROUTABILITY is not None:
                        ok = cn + RW * ROUTABILITY.fine_costs(B, K, RG, T, layer_name, ch, gnd_layers=args.get('gnd_layers')) < co - 1e-8
                    if ok:


                        ok = PF is None or PF.contribution(ch) <= PF.contribution(old) + 1e-9
                    if ok:
                        for r in rs:
                            layer_name[r] = ch[r]; pads[r] = ov[r]; geo[r] = gx[r]
                            if DZ: DZ.assign(r, gx[r])
                            if DZI and r in DZI.bounds: DZI.assign(r, gx[r])
                            if DIS: DIS.assign(r, gx[r])
                            if r in KPABS: KPABS[r] = kp_abs(r, ch[r])
                            if PF is not None: PF.assign(r, ch[r])
                        if sn: SEG.update(sn); ALL[0] = build_all()
                        accepted += 1
                    for r in rs: bel.add(r, geo[r])
        print('P26 rigid groups: %d groups, %d translations accepted' % (len(groups), accepted), flush=True)
    print('Fine tuning finished (%.0fs)' % (time.time() - t0))
    return layer_name


def cost_global(r, s, layer_name, pad_idx):
    ch = with_group(r, s, layer_name)
    over = collections.defaultdict(list)
    for q, sq in ch.items():
        for _, net, x, y, sd in T[q].pads_abs(*sq): over[net].append((x, y, sd, q))
    c = 0.0
    for q, sq in ch.items():
        for _, net, px, py, ps in T[q].pads_abs(*sq):
            if net in GND or net not in pad_idx: continue
            L = [p for p in pad_idx[net] if p[3] not in ch] + over[net]
            L = [p for p in L if p[3] != q]
            if L: c += net_weight(net) * min(math.hypot(px-p[0], py-p[1]) + via_k(ps,p[2]) for p in L)
    if DENSE_ISLANDS:

        pinned = {q for q in layer_name if PARTS[q]['pinned']} | (FIXED_EXTRA & set(layer_name))
        if args.get('fixed_anchor') == '1': pinned.update(set(ISLANDS) & set(layer_name))
        for q in list(pinned): pinned.update(mates(q) & set(layer_name))
        group_list = DM.island_groups(K, layer_name, pinned)
        affected = {a: rs for a, rs in group_list.items() if set(rs) & ch.keys()}
        gs = {q: T[q].geo(*layer_name[q]) for rs in affected.values() for q in rs}
        goal = DM.IslandTarget(gs, affected)
        c += DENSE_ISLANDS * goal.contribution({q: T[q].geo(*sq) for q, sq in ch.items()})
    return c


def pin_cost(r, st, layer_name):
    ch = with_group(r, st, layer_name)
    refs = set(ch) & PIN.keys()
    for q in ch: refs.update(PIN_OF.get(q, ()))
    c = 0.0
    for cc in sorted(refs):
        z, n, pins = PIN[cc]
        if cc in layer_name and z in layer_name:
            c += pin_dist(T[cc].pads_abs(*ch.get(cc, layer_name[cc])),
                          T[z].pads_abs(*ch.get(z, layer_name[z])), n, pins, cc)
    return c


LEGAL_EXCEPTIONS = {}


def legal_roam_choice(r, soft, hard, layer_name, reason):
    if not WW_ON or hard is not None: return hard if hard is not None else soft
    if soft is None: return None
    ch = with_group(r, soft[1])
    for q, sq in ch.items():
        if q in WT and q not in SEQUENCE:
            w = way(q, sq, layer_name, ch)
            if w > WT[q]['max'] + 1e-6:
                LEGAL_EXCEPTIONS[q] = dict(way_mm=round(w, 4), max_mm=WT[q]['max'],
                    reason=reason + '; no candidate within the travel limit in spiral/free-space search',
                    proof='bounded geometric search, no global impossibility proof')
    return soft


def legalize(islands, state, layer_name, post_pins=True):
    owner = {r: I for I in islands for r in I.refs}
    def new_index():
        bel = Occupancy(); pad_idx = collections.defaultdict(list)
        for r, s in layer_name.items():
            bel.add(r, T[r].geo(*s))
            for nr, net, x, y, ps in T[r].pads_abs(*s): pad_idx[net].append((x, y, ps, r))
        return bel, pad_idx


    allowed = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']).values()) for r in layer_name}
    keep_abs = {kr: kp_abs(kr, layer_name[kr], small=True) for kr in KPLK if kr in layer_name}
    def problem(r, s, bel, ign=None):
        geo = T[r].geo(*s)
        if bel.collision(geo, ignore=ign or {r}): return True
        if not PARTS.get(r, {}).get('pinned') and any(not inner_p.contains(Point(x, y)) for nr, net, x, y, ps in T[r].pads_abs(*s)):
            return True
        if PAD_EDGE is not None and not PARTS.get(r,{}).get('pinned') and not PAD_EDGE.vacant(r,s):return True
        if sum(g.difference(inner).area for g in geo.values()) > allowed[r] + 1e-3: return True
        if not PARTS.get(r, {}).get('pinned') and keepout_area(r, geo) > 1e-7: return True
        for kr, ks in keep_abs.items():
            if kr == r: continue
            for k in ks:
                if any(k.intersects(g) for g in geo.values()): return True
        if KEEPOUT_ON and r in keep_abs and not PARTS.get(r, {}).get('pinned'):
            for k in kp_abs(r, s, small=True):
                if bel.overlap({'F': k, 'B': k}, ignore=ign or {r})[0] > 1e-7: return True
        return False
    def problem_g(r, s, bel):
        ign = movement_refs(r)
        return any(problem(q, sq, bel, ign) for q, sq in with_group(r, s, layer_name).items() if q in layer_name)
    def wander_ok(r, s):
        ch = with_group(r, s, layer_name)
        return not WW_ON or all(q in SEQUENCE or q not in WT or way(q, sq, layer_name, ch) <= WT[q]['max'] + 1e-6 for q, sq in ch.items())
    def problem_hard(r, s, bel): return problem_g(r, s, bel) or not wander_ok(r, s)
    def cause(r, s, bel):
        g = T[r].geo(*s); gr = []
        if bel.collision(g, ignore=mates(r)): gr.append('Collision')
        if keepout_area(r, g) > 1e-7: gr.append('KeepoutArea')
        if sum(x.difference(inner).area for x in g.values()) > allowed[r]+1e-3: gr.append('Border')
        return '/'.join(gr) or 'geometric rule violation (pad/footprint keepout)'
    def search_free(r, st, bel, prob, **kw):
        def geometry(x,y,th,sd):
            gs = collections.defaultdict(list)
            for q,sq in with_group(r,(x,y,th,sd),layer_name).items():
                for side,g in T[q].geo(*sq).items(): gs[side].append(g)
            return {side:unary_union(v) for side,v in gs.items()}
        return clearance(r,st,bel,prob,geometry=geometry if r in POWER else None,**kw)
    def compact_refit(refs, pair=False, cross_hard=False):



        order=sorted(refs,key=lambda r:(type_(r,'') != 'L',type_(r,'') != 'D',r))
        pf=Pairs(layer_name) if pair else None
        for _ in range(2):
            for r in order:
                if r not in PHYS.pin or PARTS[r]['pinned'] or r in FIXED_EXTRA or r in SEQUENCE or r in GROUP: continue
                if owner[r].pinned and owner[r].anchor==r: continue
                z,n,pins=PHYS.pin[r]
                if z not in layer_name: continue
                bel,idx=new_index();bel.remove(r)
                old=layer_name[r];pads={q:T[q].pads_abs(*(layer_name.get(q) or (fps[q]['x'],fps[q]['y'],0,fps[q]['side']))) for q in T}
                oldpin=pin_cost(r,old,layer_name)
                if cross_hard and problem(r,old,bel): continue


                rest=np.zeros((0,4));kn=[];oldcross=0
                if cross_hard:
                    pts=collections.defaultdict(list)
                    for q in T:
                        for _,net,x,y,_ in pads[q]:
                            if net in NET and net not in GND:pts[net].append((x,y))
                    kn=sorted({p[1] for p in pads[r] if p[1] in pts and len(pts[p[1]])<=CROSS_MAXPADS})
                    seg=[np.asarray(mst_edges(ps),float).reshape(-1,4) for net,ps in sorted(pts.items()) if net not in kn and len(ps)<=CROSS_MAXPADS]
                    if seg:rest=np.concatenate(seg)
                def crossing(st):
                    seg=[]
                    for net in kn:
                        pts=[(x,y) for q in T for _,n,x,y,_ in (T[r].pads_abs(*st) if q==r else pads[q]) if n==net]
                        seg.append(np.asarray(mst_edges(pts),float).reshape(-1,4))
                    return sum(crossings(s,rest) for s in seg)+sum(crossings(a,b) for i,a in enumerate(seg) for b in seg[i+1:])
                if kn:oldcross=crossing(old)
                def price(st):
                    ps=dict(pads);ps[r]=T[r].pads_abs(*st)
                    c=cost_global(r,st,layer_name,idx)+pin_cost(r,st,layer_name)+PHYS.cost({r:st},ps)
                    if pf is not None:c+=pf.contribution({r:st})+feder_part(r,st)
                    return c
                best=(price(old),old)
                ths=[old[2]]+([(old[2]+t)%360 for t in (90,180,270)] if may_rotate(r) else [])
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
                                    if pair and pin_cost(r,st,layer_name)>oldpin+1e-9: continue
                                    cost=price(st)
                                    if cost < best[0]-1e-6 and (not kn or crossing(st)<=oldcross): best=(cost,st)
                                    hits+=1
                                    if hits >= 4: break
                layer_name[r]=best[1]
                if pair and best[1]!=old:PIN_POST_LEGALIZE.add(r)
                if pf is not None:pf.assign(r,best[1])
                if r in keep_abs: keep_abs[r]=kp_abs(r,best[1],small=True)
    if CLEARANCE:



        large = [r for r in layer_name if PARTS[r]['pinned'] or PARTS[r]['role'] != 'small']
        for r in sorted([r for r in large if not PARTS[r]['pinned'] and r not in SEQUENCE and r not in FIXED_EXTRA], key=lambda r: -T[r].surface):
            bel_g = Occupancy()
            for q in large:
                if q not in movement_refs(r): bel_g.add(q, T[q].geo(*layer_name[q]))
            if not problem_g(r, layer_name[r], bel_g): continue
            hard = search_free(r, layer_name[r], bel_g, problem_hard, rotate=True) if WW_ON else None
            best = hard or search_free(r, layer_name[r], bel_g, problem_g, rotate=True)
            best = legal_roam_choice(r, best, hard, layer_name, cause(r, layer_name[r], bel_g))
            if best:
                print('  Chip %s moved to free room before legalizing (%.1f mm)' % (r, best[0]), flush=True)
                for q, sq in with_group(r, best[1], layer_name).items():
                    layer_name[q] = sq
                    if KEEPOUT_ON and q in keep_abs: keep_abs[q] = kp_abs(q, sq, small=True)
    def important(r): return WT.get(r, {}).get('importance', 0.0) if PIN_LEGAL else 0.0
    before = None
    for rnd in range(8):
        bel, pad_idx = new_index()
        bad = [r for r in layer_name if problem(r, layer_name[r], bel, mates(r))]
        print('  Legalize round %d: %d parts with collision/edge' % (rnd + 1, len(bad)), flush=True)
        if not bad: break
        wide = before is not None and len(bad) >= before
        if wide and rnd >= 3: break
        before = len(bad)


        bad.sort(key=lambda r: (PARTS[r]['role'] != 'small', important(r), T[r].surface))
        done = set()
        for r0 in bad:
            r = SEQUENCE[r0][0] if r0 in SEQUENCE else r0
            r = POWER_OF.get(r, r)
            if r in done or r not in layer_name: continue
            done.add(r)
            if owner[r].pinned and owner[r].anchor == r: continue
            if PARTS[r]['pinned'] or r in FIXED_EXTRA: continue
            if not problem_g(r, layer_name[r], bel): continue
            for q in movement_refs(r): bel.remove(q)
            x0, y0, th0, s0 = layer_name[r]
            rmax = (40.0 if wide else 10.0) if PARTS[r]['role'] == 'small' else (10.0 if wide else 3.0)
            best = None; hard = None
            for th in sorted({th0, (th0 + 90) % 360, (th0 + 180) % 360, (th0 + 270) % 360}, key=lambda t: t != th0):
                if not groups_rotatable(r) and th != th0: continue
                located = 0; d_first = None
                for dx, dy, d in SPIRAL:
                    if d > rmax or (d_first is not None and d > d_first + 1.0) or located >= 12: break
                    st = (x0 + dx, y0 + dy, th, s0)
                    if problem_g(r, st, bel): continue
                    if d_first is None: d_first = d
                    located += 1
                    c = cost_global(r, st, layer_name, pad_idx) + 0.5 * d + 0.2 * (th != th0)
                    if PIN_LEGAL: c += pin_cost(r, st, layer_name)
                    if PHYSICS:
                        ch = with_group(r, st, layer_name)
                        ps = {q: T[q].pads_abs(*ch.get(q, sq)) for q, sq in layer_name.items()}
                        c += PHYS.cost(ch, ps)
                    if WW_ON and r in WT: c += WW_F * WWM.price(WT[r], 0.0, way(r, st, layer_name))
                    if best is None or c < best[0]: best = (c, st)
                    if wander_ok(r, st) and (hard is None or c < hard[0]): hard = (c, st)
            if WW_ON and hard is None and CLEARANCE: hard = search_free(r, layer_name[r], bel, problem_hard)
            if best is None and CLEARANCE:
                best = search_free(r, layer_name[r], bel, problem_g)
            best = legal_roam_choice(r, best, hard, layer_name, cause(r, layer_name[r], bel))
            if best:
                for q, sq in with_group(r, best[1], layer_name).items():
                    layer_name[q] = sq
                    for nr, net, x, y, ps in T[q].pads_abs(*sq): pad_idx[net].append((x, y, ps, q))
                    if KEEPOUT_ON and q in keep_abs: keep_abs[q] = kp_abs(q, sq, small=True)
            for q in movement_refs(r):
                if q in layer_name: bel.add(q, T[q].geo(*layer_name[q]))
    if PHYSICS and args.get('legal_pack', '1') == '1' and MODE == 'new':


        bel, _ = new_index()
        prev_bad = sum(problem(r, s, bel, mates(r)) for r, s in layer_name.items())
        if prev_bad:
            prior = dict(layer_name); best = (prev_bad, dict(layer_name))
            leaders = [r for r in layer_name if r not in SEQUENCE and POWER_OF.get(r, r) == r]
            fixed = {r for r in leaders if PARTS[r]['pinned'] or r in FIXED_EXTRA or (owner[r].pinned and owner[r].anchor == r)}
            fixed_bel = Occupancy()
            fixed_refs = {q for r in fixed for q in movement_refs(r)}
            for q in fixed_refs: fixed_bel.add(q,T[q].geo(*layer_name[q]))
            floor = sum(fixed_bel.collision(T[q].geo(*layer_name[q]),ignore=mates(q)) for q in fixed_refs)
            for rnd in range(3 + (args.get('legal_whole','0') == '1')):
                if best[0] <= floor: break
                keep_abs = layer_back(layer_name, prior)
                bel = Occupancy()
                for r in sorted(fixed):
                    for q in movement_refs(r): bel.add(q, T[q].geo(*layer_name[q]))
                def pack_rank(r):
                    return (PARTS[r]['role'] == 'small', -sum(T[q].surface for q in movement_refs(r)), -important(r), r)
                order = sorted(set(leaders)-fixed, key=pack_rank)
                if rnd == 1: order.sort(key=lambda r: (PARTS[r]['role'] == 'small', -important(r), -T[r].surface, r))
                if rnd >= 2:



                    order.sort(key=lambda r: (-sum(T[q].surface for q in movement_refs(r)), -important(r), r))
                for r in order:
                    st = layer_name[r]
                    if rnd == 3:



                        gs = [g for q,sq in with_group(r,(0,0,st[2],st[3]),layer_name).items() for g in T[q].geo(*sq).values()]
                        st=(inner.bounds[0]+.01-min(g.bounds[0] for g in gs),
                            inner.bounds[1]+.01-min(g.bounds[1] for g in gs),st[2],st[3])
                    if rnd == 3 or problem_g(r, st, bel):
                        found = search_free(r, st, bel, problem_g, raster=0.1, n_max=4000,
                                         rotate=not ROTATE_RULE) if CLEARANCE else None
                        if found:
                            layer_name.update(with_group(r, found[1], layer_name))
                            for q in movement_refs(r):
                                if q in keep_abs: keep_abs[q] = kp_abs(q, layer_name[q], small=True)
                    for q in movement_refs(r): bel.add(q, T[q].geo(*layer_name[q]))
                bad = sum(problem(r, s, bel, mates(r)) for r, s in layer_name.items())
                if bad < best[0]: best = (bad, dict(layer_name))
                print('  Dense repacking %d: %d -> %d rule problems' % (rnd+1, prev_bad, bad), flush=True)
                if bad == 0: break
            keep_abs = layer_back(layer_name, best[1])




            if best[0] > floor and POWER and args.get('power_refit','1') == '1':
                bel,_=new_index()
                keep_abs={kr:kp_abs(kr,layer_name[kr],small=True) for kr in KPLK if kr in layer_name}
                affected={POWER_OF[r] for r,s in layer_name.items() if r in POWER_OF and problem(r,s,bel,mates(r))}
                if affected:
                    saved=dict(layer_name);pg=dict(POWER);po=dict(POWER_OF);pr=set(POWER_REFIT)
                    for a in affected:
                        for q in POWER.pop(a): POWER_OF.pop(q,None)
                    new=dict(legalize(islands,state,layer_name,post_pins=False))
                    assert set(new) == set(saved), 'Power refit must not lose parts'
                    layer_name.clear();layer_name.update(new)
                    compact_refit({q for a in affected for q in pg[a]})
                    bel,_=new_index()
                    keep_abs={kr:kp_abs(kr,layer_name[kr],small=True) for kr in KPLK if kr in layer_name}
                    count=sum(problem(r,s,bel,mates(r)) for r,s in layer_name.items())
                    POWER.clear();POWER.update(pg);POWER_OF.clear();POWER_OF.update(po)
                    if count < best[0]:
                        POWER_REFIT.update(affected)
                        print('  Power cores reshaped and rigid again: %s, %d -> %d rule problems' % (','.join(sorted(affected)),best[0],count),flush=True)
                    else:
                        keep_abs = layer_back(layer_name, saved)
                        POWER_REFIT.clear();POWER_REFIT.update(pr)
    if post_pins and PHYSICS and MODE=='new' and args.get('pin_post_legalize','0')=='1':
        compact_refit({r for r in PIN_DEC if r in layer_name and r not in POWER_OF},pair=True,cross_hard=True)
        print('  Single C reoptimized to current pins after legalization: %s (legal, crossings not increased)' % ','.join(sorted(PIN_POST_LEGALIZE)),flush=True)
    return layer_name


CLEARANCE = args.get('clearance', '1') == '1'

def clearance(r, st0, bel, problem, raster=0.1, n_max=400, rotate=False, geometry=None):
    from PIL import Image, ImageDraw
    from scipy import ndimage
    x0, y0, th0, s0 = st0
    bx = inner.bounds; nx = int((bx[2] - bx[0]) / raster) + 1; ny = int((bx[3] - bx[1]) / raster) + 1
    def px(x, y): return ((x - bx[0]) / raster, (y - bx[1]) / raster)
    img0 = Image.new('1', (nx, ny), 0); d0 = ImageDraw.Draw(img0)
    for g in getattr(inner, 'geoms', [inner]):
        d0.polygon([px(*c) for c in g.exterior.coords], fill=1)
        for h in g.interiors: d0.polygon([px(*c) for c in h.coords], fill=0)
    geo = geometry or T[r].geo
    sides = set(geo(*st0).keys())
    for s_ in sides:
        for ref, g in bel.items.get(s_, []):
            if ref == r: continue
            for gg in getattr(g, 'geoms', [g]):
                if gg.is_empty or not hasattr(gg, 'exterior'): continue
                d0.polygon([px(*c) for c in gg.exterior.coords], fill=0)
        sp = keepout_from(r).get(s_)
        if sp is not None:
            for gg in getattr(sp[0], 'geoms', [sp[0]]):
                if gg.is_empty or not hasattr(gg, 'exterior'): continue
                d0.polygon([px(*c) for c in gg.exterior.coords], fill=0)
    vacant = np.array(img0, dtype=bool)
    if PHYSICS:

        edt = ndimage.distance_transform_edt(np.pad(vacant, 1, constant_values=False))[1:-1, 1:-1] * raster
    else:
        edt = ndimage.distance_transform_edt(vacant) * raster
    best = None
    ths = [th0] + ([(th0 + 90) % 360, (th0 + 180) % 360, (th0 + 270) % 360] if groups_rotatable(r, chip=rotate) else [])
    for th in ths:
        g0 = geo(0.0, 0.0, th, s0); b = [min(g.bounds[0] for g in g0.values()), min(g.bounds[1] for g in g0.values()),
                                               max(g.bounds[2] for g in g0.values()), max(g.bounds[3] for g in g0.values())]
        half = 0.5 * min(b[2] - b[0], b[3] - b[1])
        ys, xs = np.nonzero(edt >= half)
        if not len(xs): continue
        cx = bx[0] + (xs + 0.5) * raster - (b[0] + b[2]) / 2; cy = bx[1] + (ys + 0.5) * raster - (b[1] + b[3]) / 2
        d = np.hypot(cx - x0, cy - y0); idx = np.argsort(d)[:n_max]
        for i in idx:
            st = (float(cx[i]), float(cy[i]), th, s0)
            if not problem(r, st, bel):
                c = float(d[i]) + 0.2 * (th != th0)
                if best is None or c < best[0]: best = (c, st)
                break
    if best is None and PHYSICS and geometry is not None:
        from placement.clearance_groups import search
        best = search(r,st0,bel,inner,geo,problem,ths,raster,n_max,keepout_from(r))
        if best: print('  Non-rectangular group %s in configuration space moved to free room (%.1f mm)' % (r,best[0]),flush=True)
    return best


CROSS = float(args.get('cross', '0'))
CROSS_MAXPADS = 16


def mst_edges(pts):
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


def crossings(P, Q):
    if len(P) == 0 or len(Q) == 0: return 0
    ax, ay, bx, by = (P[:, i][:, None] for i in range(4))
    cx, cy, dx, dy = (Q[:, i][None, :] for i in range(4))
    def orient(px, py, qx, qy, rx, ry): return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    o1 = orient(ax, ay, bx, by, cx, cy); o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay); o4 = orient(cx, cy, dx, dy, bx, by)
    return int(np.count_nonzero((o1 * o2 < -1e-9) & (o3 * o4 < -1e-9)))


def metrics(layer_name):
    pos = collections.defaultdict(list)
    for r in T:
        s = layer_name.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        if r not in layer_name and s[3] != fps[r]['side']: pass
        for nr, net, x, y, ps in (T[r].pads_abs(*s) if r in layer_name else
                                  [(nr, net, fps[r]['x'] + lx, fps[r]['y'] + ly, fps[r]['side']) for nr, net, lx, ly, tht in T[r].pads]):
            if net in NET and net not in GND: pos[net].append((x, y))
    tot = wt = 0.0; fast = 0.0
    segs = [(np.array(mst_edges(pts), float).reshape(-1, 4)) for n, pts in sorted(pos.items()) if 2 <= len(pts) <= CROSS_MAXPADS]
    cross = sum(crossings(segs[i], segs[j]) for i in range(len(segs)) for j in range(i + 1, len(segs)))
    for n, pts in pos.items():
        if len(pts) < 2: continue
        inT = [0]; rest = list(range(1, len(pts))); Lm = 0.0
        while rest:
            i, j = min(((i, j) for i in inT for j in rest), key=lambda ij: math.dist(pts[ij[0]], pts[ij[1]]))
            Lm += math.dist(pts[i], pts[j]); inT.append(j); rest.remove(j)
        tot += Lm; wt += net_weight(n) * Lm
        if NET[n]['category'] == 'fast': fast += Lm
    dec = []
    for a, I in ISLANDS.items():
        if a not in T: continue
        sa = layer_name.get(a) or (fps[a]['x'], fps[a]['y'], 0.0, fps[a]['side'])
        ap = {nr: (x, y, s) for nr, net, x, y, s in (T[a].pads_abs(*sa) if a in layer_name else
              [(nr, net, fps[a]['x'] + lx, fps[a]['y'] + ly, fps[a]['side']) for nr, net, lx, ly, tht in T[a].pads])}
        for e in I['decoupling']:
            r = e['ref']
            if r not in T or e.get('over_coil'): continue
            sr = layer_name.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
            cp = [(x, y) for nr, net, x, y, s in (T[r].pads_abs(*sr) if r in layer_name else
                  [(nr, net, fps[r]['x'] + lx, fps[r]['y'] + ly, fps[r]['side']) for nr, net, lx, ly, tht in T[r].pads]) if net == e['net_name']]
            pins = [ap[n][:2] for n in e['pins'] if n in ap]
            if cp and pins: dec.append(min(math.dist(c, p) for c in cp for p in pins))
    bel = Occupancy(); kol = 0
    for r in T:
        s = layer_name.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        g = T[r].geo(*s) if r in layer_name else T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])
        if bel.collision(g, clearance=0.00485, ignore=mates(r)): kol += 1
        bel.add(r, g)
    return dict(air_line=round(tot, 1), weighted=round(wt, 1), fast=round(fast, 1), crossings=cross,
                decoupling_medium=round(sum(dec) / len(dec), 2) if dec else None, decoupling_max=round(max(dec), 2) if dec else None,
                collisions=kol)


def rule_metrics(layer_name):
    st = {r: layer_name.get(r) or (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']) for r in T}
    out = dict(in_keepout_area=sorted(r for r in T if not PARTS.get(r, {}).get('pinned') and keepout_area(r, T[r].geo(*st[r])) > 1e-6))
    if PAD_EDGE is not None:
        out['pad_border_exact_mm'] = float((RG or {}).get('router',{}).get('edge',BORDER))
        out['pad_border_violated'] = sorted(r for r in T if not PARTS.get(r,{}).get('pinned') and not PAD_EDGE.vacant(r,st[r]))
    out.update(Pairs(st, all_items=True).total())
    ql = [pin_dist(T[c].pads_abs(*st[c]), T[z].pads_abs(*st[z]), n, p) / 4.0 for c, (z, n, p) in PIN.items() if c not in PIN_DEC]
    out['crystal_load_medium'] = round(sum(ql) / len(ql), 2) if ql else None
    ww = [(r, way(r, st[r], st)) for r in T if r in WT and r not in SEQUENCE and not PARTS.get(r, {}).get('pinned')]
    out['ww_price'] = round(sum(WWM.price(WT[r], 0.0, w) for r, w in ww), 1)
    out['ww_over_max'] = sorted(r for r, w in ww if w > WT[r]['max'] + 1e-3)
    out['ww_exceptions'] = {r: dict(LEGAL_EXCEPTIONS.get(r) or dict(reason='not justified by legalization', proof='missing'),
                                     final_path_mm=round(way(r, st[r], st), 4)) for r in out['ww_over_max']}
    out['chips_way_mm'] = round(sum(w for r, w in ww if PARTS.get(r, {}).get('role') == 'chip'), 2)
    out['rotated_banned'] = sorted(r for r in T if r in WT and not WT[r]['rotate'] and PARTS.get(r, {}).get('role') == 'small'
                                     and round(st[r][2]) % 360 != 0)
    out['groups_deviation_mm'] = round(max([math.dist(st[f][:2], sequence(f, st[l])[:2]) for f, (l, _, _) in SEQUENCE.items()] or [0.0]), 4)
    if PHYSICS or args.get('island_measurement') == '1':
        out.update(PHYS.measure(st))
        out['power_groups'] = POWER
        out['power_reshaped'] = sorted(POWER_REFIT)
        out['pin_post_legalize_moved'] = sorted(PIN_POST_LEGALIZE)
        fixed = sorted(r for r in T if PARTS.get(r, {}).get('pinned'))
        gs = {r: T[r].geo(*st[r]) for r in fixed}
        out['fixed_collisions'] = [[r,q] for i,r in enumerate(fixed) for q in fixed[i+1:]
                                  if any(g.distance(gs[q][s]) < 0.00485 for s,g in gs[r].items() if s in gs[q])]
    return out


def p35_post_run(islands,state,layer_name):
    global P35_REPORT
    grid_map = P35Map(B,K,RG,T,layer_name,args)
    pinned = inner_fixed(islands,layer_name)
    movable = [r for r in sorted(layer_name) if r not in pinned and r not in SEQUENCE and r not in GROUP and r not in POWER_OF]
    allowed = {r: sum(g.difference(inner).area for g in T[r].geo(fps[r]['x'],fps[r]['y'],0.,fps[r]['side']).values()) for r in layer_name}
    def legal(candidate):
        saved = dict(LEGAL_EXCEPTIONS)
        power, power_of, refit, pins = dict(POWER), dict(POWER_OF), set(POWER_REFIT), set(PIN_POST_LEGALIZE)
        try: result = legalize(islands,state,dict(candidate),post_pins=False)
        finally:
            LEGAL_EXCEPTIONS.clear(); LEGAL_EXCEPTIONS.update(saved)
            POWER.clear(); POWER.update(power); POWER_OF.clear(); POWER_OF.update(power_of)
            POWER_REFIT.clear(); POWER_REFIT.update(refit); PIN_POST_LEGALIZE.clear(); PIN_POST_LEGALIZE.update(pins)

        anchors={r for r,_,_,_,_ in grid_map.fanouts}
        if any(result[r] != layer_name[r] for r in layer_name if r not in movable or r in grid_map.terminals or r in anchors or not r.startswith(('C','R'))): return None
        if any(math.dist(result[r][:2],layer_name[r][:2])>1.+1e-9 for r in layer_name): return None
        bel = Occupancy()
        for r,st in result.items(): bel.add(r,T[r].geo(*st))
        for r in movable:
            gs=T[r].geo(*result[r])
            if bel.collision(gs,ignore=set(mates(r))|{r}): return None
            if any(not inner_p.contains(Point(x,y)) for _,_,x,y,_ in T[r].pads_abs(*result[r])): return None
            if sum(g.difference(inner).area for g in gs.values()) > allowed[r]+1e-3: return None
            if keepout_area(r,gs)>1e-7: return None
            if PAD_EDGE is not None and not PAD_EDGE.vacant(r,result[r]): return None
            if WW_ON and r in WT and way(r,result[r],result)>WT[r]['max']+1e-6 and way(r,result[r],result)>way(r,layer_name[r],layer_name)+1e-9: return None
            if any(k.intersects(g) for kr in KPLK if kr!=r for k in kp_abs(kr,result[kr],small=True) for g in gs.values()): return None
        return result
    layer_name,info = p35_local(grid_map,layer_name,movable,may_rotate,legal,pin_cost)
    P35_REPORT = dict(version='p35-v2',coarse_tree=P35_MAP.design,end_tree=grid_map.design,**info)
    print('P35 local moves:',len(info['move_records']),'/',info['attempts'],flush=True)
    return layer_name


def outputs(islands, state, layer_name):
    if not layer_name: layer_name = total_layer(islands, state)
    parts = {}
    for r, s in layer_name.items():
        if PARTS.get(r, {}).get('pinned') and abs(s[0] - fps[r]['x']) < 1e-6 and abs(s[1] - fps[r]['y']) < 1e-6 and s[2] == 0: continue
        parts[r] = dict(side=s[3], pads={nr: [round(x, 4), round(y, 4)] for nr, net, x, y, ps in T[r].pads_abs(*s)},
                        x=round(s[0], 4), y=round(s[1], 4), rotation=s[2])
        if DRCL:
            parts[r]['transformation_unambiguous'] = True
            parts[r]['pad_instances'] = [dict(num=nr, net=net, x=round(x, 4), y=round(y, 4), side=ps)
                                       for nr, net, x, y, ps in T[r].pads_abs(*s)]
    contour = None; reason = 'contour_fixed' if CONS.get('contour_fixed') else 'dense=0'
    if DENSE and args.get('dense_border', '1') == '1':
        contour, reason = DM.tighter_contour(B, {r: T[r].geo(*layer_name.get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r in T}, BORDER)
    density_export = {}
    if DENSE or DENSE_INNER or DENSE_ISLANDS or args.get('density_measure') == '1':
        dv = DM.measure(B, RG, part_list=T); dn = DM.measure(dict(B, outline=contour or B['outline']), RG, layer_name, T)
        density_export['density'] = dict(before=dv, after=dn, weight=DENSE, contour_reason=reason)
        pinned = inner_fixed(islands, layer_name)
        density_export['p26_inner'] = dict(before=DM.inner(B, K, RG, part_list=T, pinned=pinned),
                                         after=DM.inner(B, K, RG, layer_name, T, pinned=pinned), weight=DENSE_INNER)
        if DENSE_ISLANDS:
            density_export['p26_islands'] = dict(before=DM.island_measure(B, K, RG, part_list=T, pinned=pinned),
                                              after=DM.island_measure(B, K, RG, layer_name, T, pinned=pinned), weight=DENSE_ISLANDS)
    before = metrics({}); after = metrics(layer_name)
    print('Metrics before ', before)
    print('Metrics after', after)
    rv = rule_metrics({}); rn = rule_metrics(layer_name)
    print('Rules before ', rv)
    print('Rules after', rn)
    from placement.copper_reserve import absolute as reserve_absolute
    json.dump(dict(parts=parts, **(dict(champion=CHAMPION_EVIDENCE) if CHAMPION_EVIDENCE is not None else {}), **(dict(copper_reserve=reserve_absolute(COPPER_RESERVE,T,layer_name,fps)) if COPPER_RESERVE else {}), islands={I.name: I.refs for I in islands if len(I.refs) > 1},
                   **(dict(p3_pin_groups=dict(version='p3-pin-group_list-v1', part_list={
                       r: dict(ref=a, net_name=n, pins=sorted(pins)) for r, (a, n, pins) in sorted(PIN_DEC.items())
                       if pins and PARTS[r].get('function', '').startswith('decoupling_')
                       and not (PARTS[r].get('supports') or {}).get('over_coil')})) if P3_LOOP else
                      (dict(p3_pin_groups=K['p3_pin_groups']) if K.get('p3_pin_groups') else {})),
                   cur_state={k: list(v) for k, v in state.items()}, metrics=dict(before=before, after=after),
                   **(dict(p26_fixed=dict(version='p26-pinned-v1', refs=sorted(inner_fixed(islands, layer_name))))
                      if DENSE or DENSE_INNER or args.get('density_measure') == '1' or args.get('fixed_anchor') == '1'
                      or args.get('operate_new_free') == '1'
                      or inner_fixed(islands, layer_name) - {r for r in layer_name if PARTS.get(r, {}).get('pinned')
                                                    or PARTS.get(r, {}).get('role') in ('mechanics', 'without')} else {}),
                   design_rules=dict(before=rv, after=rn), options=args, **density_export,
                   **(dict(p35=P35_REPORT) if P35_REPORT is not None else {}),
                   **(dict(routability=dict(version=ROUTABILITY.version,final_check_required=True)) if ROUTABILITY is not None else {}),
                   **(dict(outline=contour, outline_original=B['outline']) if contour else {})), open(OUT, 'w'), indent=1)
    if 'image' in args: image(layer_name, islands, args['image'])
    print('finished in %.0fs ->' % (time.time() - t_start), OUT)


def image(layer_name, islands, prefix):
    from PIL import Image, ImageDraw, ImageFont
    bx = board.bounds; S = 40.0 if (bx[2] - bx[0]) < 60 else 12.0
    W, H = int((bx[2] - bx[0] + 4) * S), int((bx[3] - bx[1] + 4) * S)
    tr = lambda x, y: ((x - bx[0] + 2) * S, (y - bx[1] + 2) * S)
    COLOR = {'digital': (60, 110, 205), 'analog': (40, 160, 80), 'power': (205, 60, 60), 'mixed': (150, 90, 190),
             'neutral': (130, 130, 130)}
    try: font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 12)
    except OSError: font = None
    owner = {r: I for I in islands for r in I.refs}
    for side in ('F', 'B'):
        in_ = Image.new('RGB', (W, H), 'white'); d = ImageDraw.Draw(in_)
        for g in getattr(board, 'geoms', [board]): d.polygon([tr(*p) for p in g.exterior.coords], outline=(0, 0, 0))
        if KEEPOUT.get(side) is not None:
            for gg in getattr(KEEPOUT[side], 'geoms', [KEEPOUT[side]]):
                if hasattr(gg, 'exterior'): d.polygon([tr(*p) for p in gg.exterior.coords], fill=(225, 225, 225), outline=(150, 150, 150))
        for r, s in layer_name.items():
            I = owner.get(r); category = ISLANDS.get(I.anchor, {}).get('category', 'neutral') if I else 'neutral'
            col = COLOR.get(category, (130, 130, 130))
            for sd, g in T[r].geo(*s).items():
                if sd != side: continue
                for gg in getattr(g, 'geoms', [g]):
                    light = tuple(int(c + (255 - c) * (0.55 if (I is None or r != I.anchor) else 0.25)) for c in col)
                    d.polygon([tr(*p) for p in gg.exterior.coords], fill=light, outline=col)
            if s[3] == side and T[r].n_pads >= 5 and font: d.text(tr(s[0], s[1]), r, fill=(0, 0, 0), font=font)

        pos = collections.defaultdict(list)
        for r, s in layer_name.items():
            for nr, net, x, y, ps in T[r].pads_abs(*s):
                if NET.get(net, {}).get('category') in ('fast', 'sw'): pos[net].append((x, y))
        for n, pts in pos.items():
            for p, q in zip(pts[:-1], pts[1:]): d.line([tr(*p), tr(*q)], fill=(230, 120, 0), width=1)
        if font: d.text((10, 10), 'Side %s  (blue digital, green analog, red power, purple mixed; orange: fast nets; gray: keepout area)' % side,
                        fill=(0, 0, 0), font=font)
        in_.save('%s_%s.png' % (prefix, side))
    print('Images', prefix + '_F.png', prefix + '_B.png')


if __name__ == '__main__':
    main()
