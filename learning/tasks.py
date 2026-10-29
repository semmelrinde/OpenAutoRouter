"""Task generator: random but plausible boards for self-play, built only from KiCad library footprints.

A task is an invented circuit made of building blocks (MCU or processor, often BGA, memory, buck and boost converters,
LDOs, ADC/DAC, op-amp stages, sensors, radio module, crystal with load capacitors, decoupling per supply pin, pull-ups,
LEDs, connectors fixed at the edge). Values do not have to be exact, but current, voltage, switching frequency and net
kind are part of the task so that net classes, widths and scoring are meaningful. The board is tight: the sum of the
courtyards times a factor (random 1.25 to 1.6 by default), with 4, 6 or 8 layers.

Footprints are read directly from the KiCad .kicad_mod library files (small own S-expression reader, no pcbnew), so a
task is created in milliseconds. Output per task, in the input format of the tool chain:
  board.json      like kicad/export_board.py (pads with polygons, footprints, courtyards, outline, keepouts)
  design_rules.json     like analysis/design_rules.py
  specs.json   net kinds, currents, voltages, switching frequencies, fixed parts (read by analysis/netclasses.py)
  task.json    the build plan (blocks, nets with current, speed and kind)
All parts start unrotated inside the outline; the placer (mode=new) places them. Same seed, same files.

usage: python3 learning/tasks.py <target_folder> [seed=1] [n=1] [level=0] [cu_layers=0 (random 4/6/8)]
       [factor=0 (random)]
"""
import functools, glob, json, math, os, random, re, sys

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
FP_DIR = os.environ.get('KICAD_FOOTPRINTS', '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints')
VERSION = '10'
REAL_VERSION = '11'
BGA_MEASUREMENT_VERSION = '12'

class GeneratorError(ValueError):
    ""



_TOK = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')


def sexp(t):
    st = [[]]
    for m in _TOK.finditer(t):
        s = m.group(0)
        if s == '(': st.append([])
        elif s == ')': x = st.pop(); st[-1].append(x)
        else: st[-1].append(s[1:-1] if s[0] == '"' else s)
    return st[0][0]


def _f(el, key):
    for x in el:
        if isinstance(x, list) and x and x[0] == key: return x
    return None


@functools.lru_cache(maxsize=None)
def fp_load(name):
    lib, n = name.split(':', 1)
    T = sexp(open(os.path.join(FP_DIR, lib + '.pretty', n + '.kicad_mod'), errors='replace').read())
    pads, crt, blocked = [], [], []
    for el in T:
        if not isinstance(el, list) or not el: continue
        if el[0] == 'zone' and _f(el, 'keepout'):
            kp = _f(el, 'keepout'); pg = _f(_f(el, 'polygon') or [], 'pts') or []
            q = [(float(p[1]), float(p[2])) for p in pg[1:] if isinstance(p, list) and p[0] == 'xy']
            lay = (_f(el, 'layers') or _f(el, 'layer') or [])[1:]
            vb = {d: (_f(kp, k) or [k, 'allowed'])[1] == 'not_allowed' for d, k in
                  (('track_list', 'tracks'), ('vias', 'vias'), ('pads', 'pads'), ('areas', 'copperpour'), ('part_list', 'footprints'))}
            if len(q) >= 3: blocked.append(dict(poly=q, cu_layers=lay, prohibitions=vb))
            continue
        if el[0] == 'pad':
            at = _f(el, 'at'); size = _f(el, 'size'); dr = _f(el, 'drill'); lay = _f(el, 'layers') or []
            rr = _f(el, 'roundrect_rratio')
            if not at or not size: continue
            d = 0.0; hole = []; drill_size = [0.0, 0.0]; copper_offset = [0.0, 0.0]
            if dr:
                offset = _f(dr,'offset')
                if offset: copper_offset = [float(offset[1]),float(offset[2])]
                numbers = [x for x in dr[1:] if not isinstance(x, list) and re.match(r'^-?[\d.]+$', x)]
                d = float(numbers[0]) if numbers else 0.0
                if d > 0:


                    w = d; h = float(numbers[1]) if len(numbers)>1 else d
                    drill_size = [w,h]
                    from shapely.geometry import LineString, Point
                    from shapely.affinity import rotate, translate
                    radius = min(w,h)/2; half = abs(w-h)/2
                    g = (LineString([(-half,0),(half,0)]) if w>h else LineString([(0,-half),(0,half)])) if half else Point(0,0)
                    g = g.buffer(radius,32)
                    g = rotate(g,-float(at[3]) if len(at)>3 else 0,origin=(0,0))
                    hole = list(translate(g,float(at[1]),float(at[2])).exterior.coords)
            tht = el[2] in ('thru_hole', 'np_thru_hole')
            if not tht and not any(L in ('F.Cu', '*.Cu') for L in lay[1:]): continue
            pads.append(dict(num=el[1], type_=el[2], shaping=el[3], x=float(at[1]), y=float(at[2]),
                             rot=float(at[3]) if len(at) > 3 else 0.0, w=float(size[1]), h=float(size[2]),
                             drill=d, drill_size=drill_size, copper_offset=copper_offset,
                             hole_polys=[hole] if hole else [], tht=tht, npth=el[2] == 'np_thru_hole', rr=float(rr[1]) if rr else 0.0))
        elif el[0] in ('fp_line', 'fp_rect', 'fp_poly', 'fp_circle', 'fp_arc'):
            L = _f(el, 'layer')
            if not L or L[1] != 'F.CrtYd': continue
            pts = [(float(p[1]), float(p[2])) for k in ('start', 'end', 'mid', 'center') for p in [_f(el, k)] if p]
            xy = _f(el, 'pts')
            if xy: pts += [(float(p[1]), float(p[2])) for p in xy[1:] if isinstance(p, list) and p[0] == 'xy']
            if el[0] == 'fp_circle' and _f(el, 'center') and _f(el, 'end'):
                c, e = _f(el, 'center'), _f(el, 'end'); r = math.hypot(float(e[1]) - float(c[1]), float(e[2]) - float(c[2]))
                pts += [(float(c[1]) - r, float(c[2]) - r), (float(c[1]) + r, float(c[2]) + r)]
            crt += pts
    if not crt:
        crt = [(p['x'] - p['w'] / 2 - 0.25, p['y'] - p['h'] / 2 - 0.25) for p in pads] + \
              [(p['x'] + p['w'] / 2 + 0.25, p['y'] + p['h'] / 2 + 0.25) for p in pads]
    xs, ys = [p[0] for p in crt], [p[1] for p in crt]
    return dict(pads=pads, crt=(min(xs), min(ys), max(xs), max(ys)), blocked=blocked)


def keepout_layers(lay, names):
    out = set()
    for L in lay:
        if L in ('*.Cu', 'F&B.Cu'): out |= set(names) if L == '*.Cu' else {'F', 'B'}
        elif L.endswith('.Cu'): out.add(L[:-3])
    return [L for L in names if L in out]


@functools.lru_cache(maxsize=None)
def _list(lib):
    return tuple(sorted(os.path.basename(f)[:-10] for f in glob.glob(os.path.join(FP_DIR, lib + '.pretty', '*.kicad_mod'))))


def search(rnd, lib, match_pattern, without=r'HandSolder|ThermalVias|Handsoldering|_Pad\d'):
    for m in ([match_pattern] if isinstance(match_pattern, str) else match_pattern):
        k = [n for n in _list(lib) if re.match(m, n) and not re.search(without, n)]
        if k: return lib + ':' + rnd.choice(k)
    return None


def antenna_side(F):
    x0, y0, x1, y1 = F['crt']; ps = [p for p in F['pads'] if not p['npth']]
    px0 = min(p['x'] - max(p['w'], p['h']) / 2 for p in ps); px1 = max(p['x'] + max(p['w'], p['h']) / 2 for p in ps)
    py0 = min(p['y'] - max(p['w'], p['h']) / 2 for p in ps); py1 = max(p['y'] + max(p['w'], p['h']) / 2 for p in ps)
    border = {'upper': py0 - y0, 'bottom': y1 - py1, 'left': px0 - x0, 'right': x1 - px1}
    s = max(sorted(border), key=lambda k: border[k]); d = max(0.0, border[s] - 0.3)
    rect = {'upper': (x0, y0, x1, y0 + d), 'bottom': (x0, y1 - d, x1, y1), 'left': (x0, y0, x0 + d, y1), 'right': (x1 - d, y0, x1, y1)}[s]
    return s, rect


def pad_poly(p):
    w, h, f = p['w'], p['h'], p['shaping']
    if f == 'circle' or (f == 'oval' and abs(w - h) < 1e-6):
        r = w / 2; pts = [(r * math.cos(2 * math.pi * k / 12), r * math.sin(2 * math.pi * k / 12)) for k in range(12)]
    elif f == 'oval':
        a, b = max(w, h) / 2, min(w, h) / 2; m = a - b; pts = []
        for k in range(7): t = -math.pi / 2 + math.pi * k / 6; pts.append((m + b * math.cos(t), b * math.sin(t)))
        for k in range(7): t = math.pi / 2 + math.pi * k / 6; pts.append((-m + b * math.cos(t), b * math.sin(t)))
        if h > w: pts = [(y, x) for x, y in pts]
    else:
        c = min(w, h) * (p['rr'] if f == 'roundrect' else 0.0) * 0.6
        x, y = w / 2, h / 2
        pts = [(-x + c, -y), (x - c, -y), (x, -y + c), (x, y - c), (x - c, y), (-x + c, y), (-x, y - c), (-x, -y + c)] if c > 0.005 \
            else [(-x, -y), (x, -y), (x, y), (-x, y)]
    th = math.radians(p['rot']); cs, sn = math.cos(th), math.sin(th)


    dx,dy = p.get('copper_offset',[0.0,0.0])
    pts = [(u+dx,v+dy) for u,v in pts]
    return [(round(p['x'] + cs * u + sn * v, 5), round(p['y'] - sn * u + cs * v, 5)) for u, v in pts]





INTERFACES = {
    'usb_hs':  dict(f_Hz=240e6, data_rate=480e6, tr_ns=0.5, z_diff=90, tol_mm=0.15, rank=5),
    'ddr3':    dict(f_Hz=400e6, data_rate=800e6, tr_ns=0.3, z_se=40, z_diff=100, tol_mm=0.5, tol_addr_mm=2.5, rank=5),
    'sdram':   dict(f_Hz=166e6, data_rate=166e6, tr_ns=1.0, z_se=50, tol_mm=5.0, rank=4),
    'qspi':    dict(f_Hz=133e6, data_rate=133e6, tr_ns=1.0, z_se=50, tol_mm=2.5, rank=4),
    'spi':     dict(f_Hz=50e6, data_rate=50e6, tr_ns=2.0, z_se=50, rank=2),
    'uart':    dict(f_Hz=1.5e6, data_rate=3e6, tr_ns=10.0, rank=1),
    'i2c':     dict(f_Hz=400e3, data_rate=400e3, tr_ns=300.0, rank=1),
    'crystal':   dict(f_Hz=24e6, tr_ns=None, rank=3),
    'i2s':     dict(f_Hz=12.288e6, data_rate=12.288e6, tr_ns=2.0, z_se=50, tol_mm=25.0, rank=2),
    'mclk':    dict(f_Hz=24.576e6, data_rate=24.576e6, tr_ns=1.5, z_se=50, rank=3),
}

def fast(category, group=None, **more):
    s = INTERFACES[category]; d = dict(category='fast' if s['rank'] >= 2 else 'signal', rank=s['rank'], f_Hz=s['f_Hz'], interface=category)
    if s.get('tr_ns'): d['tr_ns'] = s['tr_ns']
    if s.get('data_rate'): d['data_rate'] = s['data_rate']
    if s.get('z_se'): d['z_ohm'] = s['z_se']
    if group: d['length_group'] = group; d['tolerance_mm'] = s.get('tol_mm')
    d.update(more); return d


class Task:
    def __init__(self, seed, cu_layers, factor, level=0.0, axes=None, potential=False, package=None, generator_version=None):
        self.rnd = random.Random(seed); self.seed = seed; self.cu_layers = cu_layers; self.factor = factor; self.level = float(level)
        self.potential = bool(potential) and cu_layers >= 6; self.power_board = False
        self.package = package
        self.version = generator_version or VERSION
        if self.version not in ('10', REAL_VERSION, BGA_MEASUREMENT_VERSION): raise ValueError('generator_version=10|11|12')
        self.axes = axes or {}
        self.vector_mode = axes is not None
        self.part_list = []
        self.net_list = {}
        self.counter = {}; self.blocks = []; self.current_part = {}; self.fsw = {}; self.keepout_list = []
        self.rails = {}
        self.current = None; self.island_from = {}


    def ref(self, p):
        self.counter[p] = self.counter.get(p, 0) + 1; return '%s%d' % (p, self.counter[p])

    def net_name(self, name, **own_set):
        n = self.net_list.setdefault(name, {}); n.update({k: v for k, v in own_set.items() if v is not None}); return name

    def part(self, p, fp, value, pins, type_=None, func=None, pinned=None, current_val=None):
        if fp is None: return None
        r = self.ref(p)
        self.part_list.append(dict(ref=r, fp=fp, value=value, pins=dict(pins), type_=dict(type_ or {}), func=dict(func or {}), pinned=pinned))



        if p == 'U' and not pinned: self.current = r
        elif p in ('C', 'R', 'L', 'D', 'Y', 'FB') and self.current and not pinned: self.island_from[r] = self.current
        if current_val: self.current_part[r] = current_val
        return r

    def two_pole(self, p, fp, value, a, b, type_='passive'):
        return self.part(p, fp, value, {'1': a, '2': b}, type_={'1': type_, '2': type_})

    def small(self):
        return '0201' if self.rnd.random() < min(0.85, 0.25 + 0.06 * self.level) else '0402'

    def c(self, a, b='GND', amount='100nF', size=None):
        g = size or self.small()
        fp = {'0201': 'Capacitor_SMD:C_0201_0603Metric', '0402': 'Capacitor_SMD:C_0402_1005Metric', '0603': 'Capacitor_SMD:C_0603_1608Metric',
              '0805': 'Capacitor_SMD:C_0805_2012Metric', '1206': 'Capacitor_SMD:C_1206_3216Metric', '1210': 'Capacitor_SMD:C_1210_3225Metric'}[g]
        return self.two_pole('C', fp, amount, a, b)

    def r(self, a, b, amount='10k', size=None):
        g = size or self.small()
        return self.two_pole('R', {'0201': 'Resistor_SMD:R_0201_0603Metric', '0402': 'Resistor_SMD:R_0402_1005Metric',
                                  '0603': 'Resistor_SMD:R_0603_1608Metric', '0805': 'Resistor_SMD:R_0805_2012Metric',
                                  '1206': 'Resistor_SMD:R_1206_3216Metric', '2512': 'Resistor_SMD:R_2512_6332Metric'}[g], amount, a, b)

    def ic(self, p, fp, value, roles, current_val=None):
        F = fp_load(fp); too = roles(F['pads'])
        pins = {n: v[0] for n, v in too.items()}; type_ = {n: v[1] for n, v in too.items()}; fn = {n: v[2] for n, v in too.items()}
        return self.part(p, fp, value, pins, type_, fn, current_val=current_val)


    @staticmethod
    def signal_pads(pads):
        ps, seen = [], set()
        for p in pads:
            if p['npth'] or not p['num'] or p['num'] in seen: continue
            seen.add(p['num']); ps.append(p)
        ep = None
        if len(ps) > 6:
            g = max(ps, key=lambda p: p['w'] * p['h'])
            if g['w'] * g['h'] > 3 * sorted(p['w'] * p['h'] for p in ps)[len(ps) // 2] and abs(g['x']) < 0.6 and abs(g['y']) < 0.6: ep = g
        return [p for p in ps if p is not ep], ep

    def distribute(self, pads, vdd, signals, gnd='GND', vdd_share=0.12, core_set=None, bga=False, supplies=None):
        ps, ep = self.signal_pads(pads); too = {}
        if ep: too[ep['num']] = (gnd, 'power_in', 'EP')
        if bga and self.version == BGA_MEASUREMENT_VERSION:
            from learning.bga_occupancy import occupy
            too.update(occupy(ps, signals, supplies or [vdd] + ([core_set] if core_set else []), gnd, self.rnd))
        elif bga:
            xs = sorted({round(p['x'], 2) for p in ps}); ys = sorted({round(p['y'], 2) for p in ps})
            def ring(p): return min(xs.index(round(p['x'], 2)), len(xs) - 1 - xs.index(round(p['x'], 2)),
                                    ys.index(round(p['y'], 2)), len(ys) - 1 - ys.index(round(p['y'], 2)))
            nring = max(ring(p) for p in ps) + 1
            for p in sorted(ps, key=lambda p: (ring(p), p['num'])):
                k = ring(p)
                if k >= nring - 2 and nring > 3:
                    too[p['num']] = ((gnd, 'power_in', 'GND') if (xs.index(round(p['x'], 2)) + ys.index(round(p['y'], 2))) % 2 == 0 or not core_set
                                    else (core_set, 'power_in', 'VCORE'))
                elif k == nring - 3 and nring > 4:
                    too[p['num']] = (vdd, 'power_in', 'VDD') if self.rnd.random() < 0.5 else (gnd, 'power_in', 'GND')
                elif signals and self.rnd.random() < 0.85: too[p['num']] = signals.pop(0)
                elif self.rnd.random() < 0.15: too[p['num']] = (gnd, 'power_in', 'GND')
        else:
            n = len(ps); k_vdd = max(1, int(n * vdd_share / 2))
            idx = [int(i * n / (k_vdd)) for i in range(k_vdd)]
            for i in idx:
                too[ps[i]['num']] = (vdd, 'power_in', 'VDD')
                if i + 1 < n: too[ps[i + 1]['num']] = (gnd, 'power_in', 'GND')
            if core_set and n > 30:
                for j in range(max(2, k_vdd)):
                    too[ps[(n // 2 + 3 * j) % n]['num']] = (core_set, 'power_in', 'VCORE'); too[ps[(n // 2 + 3 * j + 1) % n]['num']] = (gnd, 'power_in', 'GND')
            for p in ps:
                if p['num'] in too: continue
                if signals and self.rnd.random() < 0.9: too[p['num']] = signals.pop(0)
        return too

    def decouple(self, ref, per=1):
        t = next(t for t in self.part_list if t['ref'] == ref)
        rails = {}
        for num, net in t['pins'].items():
            if t['type_'].get(num) == 'power_in' and net != 'GND' and t['func'].get(num) in ('VDD', 'VCORE', 'VIN', 'AVDD', 'VCC'):
                rails[net] = rails.get(net, 0) + 1
                for _ in range(per): self.c(net, 'GND', self.rnd.choice(['100nF', '100nF', '10nF']), self.rnd.choice(['0201', '0402']))
        for net in sorted(rails): self.c(net, 'GND', self.rnd.choice(['4.7uF', '10uF']), self.rnd.choice(['0402', '0603']))


    def connector_input(self):
        self.current = None
        category = self.rnd.choice(['usb', 'usb', 'jst', 'jst_large'])
        if category == 'usb':
            vin = self.net_name('VBUS', category='current_val', voltage=5.0); fp = search(self.rnd, 'Connector_USB', [r'USB_C_Receptacle_HRO_TYPE-C-31-M-12', r'USB_C_Receptacle_GCT_USB4105'])
            dp = self.net_name('USB_DP', **fast('usb_hs', 'USB', z_diff_ohm=90)); dm = self.net_name('USB_DM', **fast('usb_hs', 'USB', z_diff_ohm=90))
            F = fp_load(fp); too = {}
            for p in self.signal_pads(F['pads'])[0]:
                n = p['num'].upper()
                too[p['num']] = (('GND', 'power_in', 'GND') if n.startswith(('A1', 'B1', 'A12', 'B12', 'S')) and not n.startswith(('A10', 'B10', 'A11', 'B11')) else
                                (vin, 'power_out', 'VBUS') if n.startswith(('A4', 'B4', 'A9', 'B9')) else
                                (dp, 'bidirectional', 'DP') if n in ('A6', 'B6') else (dm, 'bidirectional', 'DM') if n in ('A7', 'B7') else
                                ('CC1', 'passive', 'CC') if n == 'A5' else ('CC2', 'passive', 'CC') if n == 'B5' else ('', '', ''))
            too = {k: v for k, v in too.items() if v[0]}
            j = self.part('J', fp, 'USB-C', {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, pinned='bottom')
            self.r('CC1', 'GND', '5.1k'); self.r('CC2', 'GND', '5.1k')
            self.blocks.append('USB-C 5 V'); return vin, 5.0, (dp, dm)
        v = self.rnd.choice([12.0, 12.0, 24.0, 9.0]) if category == 'jst' else self.rnd.choice([12.0, 19.0, 24.0])
        vin = self.net_name('VIN', category='current_val', voltage=v)
        fp = search(self.rnd, 'Connector_JST', [r'JST_PH_S2B-PH-SM4', r'JST_PH_B2B-PH-K']) if category == 'jst' else \
            search(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02'])
        self.part('J', fp, 'VIN %g V' % v, {'1': vin, '2': 'GND'}, {'1': 'power_out', '2': 'power_in'}, pinned='left')
        self.c(vin, 'GND', '10uF', '1206'); self.blocks.append('Input %g V (%s)' % (v, category)); return vin, v, None
    def buck(self, vin, uin, uout, current_val, name):
        out = self.net_name(name, category='current_val', voltage=uout, current_A=current_val); sw = self.net_name('SW_' + name.strip('+'), category='sw', current_A=current_val)
        fb = self.net_name('FB_' + name.strip('+'), category='analog', rank=2); bst = self.net_name('BST_' + name.strip('+'), category='sw')
        self.net_name(vin, current_A=(self.net_list.get(vin, {}).get('current_A') or 0) + current_val * uout / max(uin, 1) / 0.9)
        if current_val <= 1.5: fp = search(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-6$', r'TSOT-23-6'])
        elif current_val <= 4: fp = search(self.rnd, 'Package_DFN_QFN', [r'QFN-16-1EP_3x3mm_P0.5mm_EP1.7', r'DFN-8-1EP_3x3mm_P0.5mm'])
        else: fp = search(self.rnd, 'Package_DFN_QFN', [r'QFN-20-1EP_4x4mm_P0.5mm_EP2.5', r'QFN-24-1EP_4x4mm_P0.5mm'])
        F = fp_load(fp); ps, ep = self.signal_pads(F['pads']); too = {}



        ps = sorted(ps, key=lambda p: math.atan2(p['y'], p['x']))
        n = len(ps); k = max(1, (n - 4) // 3) if n > 7 else 1
        blocks = [(vin, 'power_in', 'VIN')] * k + [(sw, 'power_out', 'SW')] * k + [('GND', 'power_in', 'GND')] * k
        rest = [(fb, 'input', 'FB'), (bst, 'passive', 'BST'), ('EN_' + name.strip('+'), 'input', 'EN'), ('PG_' + name.strip('+'), 'output', 'PG')]
        roles = blocks + rest + [('GND', 'power_in', 'GND')] * n
        for i, p in enumerate(ps): too[p['num']] = roles[i]
        if ep: too[ep['num']] = ('GND', 'power_in', 'EP')
        fsw = self.rnd.choice([0.5e6, 1e6, 2.2e6])
        type_ = (self.rnd.choice(['TPS62130', 'TPS54331', 'MP2307', 'LMR33630']) if current_val <= 3 else 'TPS56637' if current_val <= 6 else 'TPS568230' if current_val <= 8
                else 'TPS56C215' if current_val <= 12 else 'TPS548A28' if current_val <= 15 else 'LTC3833')
        u = self.part('U', fp, type_, {k: v[0] for k, v in too.items()},
                      {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()})
        self.fsw[u] = fsw
        lfp = (search(self.rnd, 'Inductor_SMD', [r'L_0805_2012Metric$', r'L_1008_2520Metric$', r'L_1210_3225Metric$']) if current_val <= 1.5 else
               search(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRN4018', r'L_Coilcraft_XAL50[23]0', r'L_Bourns_SRN5040']) if current_val <= 4 else
               search(self.rnd, 'Inductor_SMD', [r'L_Coilcraft_XAL70[2-7]0', r'L_Bourns_SRP1038', r'L_Vishay_IHLP-4040', r'L_Bourns_SRN8040']))
        self.two_pole('L', lfp, self.rnd.choice(['1uH', '2.2uH', '4.7uH']), sw, out)
        g_in = '0805' if current_val < 3 else '1206'; g_out = '0805' if current_val < 3 else self.rnd.choice(['1206', '1210'])
        for _ in range(1 + int(current_val > 2)): self.c(vin, 'GND', '10uF', g_in)
        self.c(vin, 'GND', '100nF', '0402')
        for _ in range(2 + int(current_val > 3) + int(current_val > 6)): self.c(out, 'GND', '22uF', g_out)
        self.c(bst, sw, '100nF', '0402')
        self.r(out, fb, '100k'); self.r(fb, 'GND', self.rnd.choice(['22k', '33k', '10k']))
        if self.power_board:


            self.net_list[fb].update(function='feedback', function_reason='Buck output divider at the FB pin of ' + u)
        self.r(vin, 'EN_' + name.strip('+'), '100k'); self.r(out, 'PG_' + name.strip('+'), '100k')
        self.current_part[u] = None; self.rails[out] = current_val
        self.blocks.append('Buck %g→%g V %.1f A %s' % (uin, uout, current_val, '%g MHz' % (fsw / 1e6)))
        return out

    def boost(self, vin, uin, uout, current_val, name):
        out = self.net_name(name, category='current_val', voltage=uout, current_A=current_val); sw = self.net_name('SW_' + name.strip('+'), category='sw', current_A=current_val * uout / uin)
        fb = self.net_name('FB_' + name.strip('+'), category='analog', rank=2)
        self.net_name(vin, current_A=(self.net_list.get(vin, {}).get('current_A') or 0) + current_val * uout / uin / 0.85)
        fp = search(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-5$', r'SOT-23-6$'])
        roles = [(sw, 'power_in', 'SW'), ('GND', 'power_in', 'GND'), (fb, 'input', 'FB'), ('EN_' + name.strip('+'), 'input', 'EN'), (vin, 'power_in', 'VIN'), ('', '', '')]
        F = fp_load(fp); ps, _ = self.signal_pads(F['pads']); too = {p['num']: roles[i] for i, p in enumerate(ps) if i < len(roles) and roles[i][0]}
        u = self.part('U', fp, self.rnd.choice(['MT3608', 'TPS61040', 'AP3012']), {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()})
        self.fsw[u] = self.rnd.choice([1.2e6, 1e6])
        self.two_pole('L', search(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRN4018', r'L_1210_3225Metric$', r'L_Coilcraft_XAL50[23]0']), '4.7uH', vin, sw)


        self.part('D', search(self.rnd, 'Diode_SMD', [r'D_SOD-123F', r'D_SOD-123$', r'D_SMA$']), 'SS34', {'1': out, '2': sw}, {'1': 'power_out', '2': 'passive'})
        self.c(vin, 'GND', '10uF', '0805'); self.c(out, 'GND', '22uF', '1206'); self.c(out, 'GND', '22uF', '1206')
        self.r(out, fb, '200k'); self.r(fb, 'GND', '10k'); self.r(vin, 'EN_' + name.strip('+'), '100k')
        if self.power_board:
            self.net_list[fb].update(function='feedback', function_reason='Boost output divider at the FB pin of ' + u)
        self.rails[out] = current_val; self.blocks.append('Boost %g→%g V %.1f A' % (uin, uout, current_val)); return out

    def ldo(self, vin, uout, name):
        out = self.net_name(name, category='supply', voltage=uout, current_A=0.2)
        fp = search(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-5$', r'SOT-223-3_TabPin2'])
        F = fp_load(fp); ps, _ = self.signal_pads(F['pads'])
        roles = ([(vin, 'power_in', 'VIN'), ('GND', 'power_in', 'GND'), (vin, 'input', 'EN'), ('', '', ''), (out, 'power_out', 'VOUT')]
                  if len(ps) >= 5 else [('GND', 'power_in', 'GND'), (out, 'power_out', 'VOUT'), (vin, 'power_in', 'VIN'), (out, 'power_out', 'VOUT')])
        too = {p['num']: roles[i] for i, p in enumerate(ps) if i < len(roles) and roles[i][0]}
        self.part('U', fp, 'LDO %g V' % uout, {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()})
        self.c(vin, 'GND', '1uF', '0402'); self.c(out, 'GND', '1uF', '0402'); self.rails[out] = 0.2; self.blocks.append('LDO %g V' % uout); return out

    def crystal(self, xi, xo):
        fp = search(self.rnd, 'Crystal', [r'Crystal_SMD_2016-4Pin', r'Crystal_SMD_2520-4Pin', r'Crystal_SMD_3225-4Pin_3.2x2.5mm$'])
        self.part('Y', fp, self.rnd.choice(['16MHz', '24MHz', '32MHz', '25MHz']), {'1': xi, '3': xo, '2': 'GND', '4': 'GND'},
                  {'1': 'passive', '3': 'passive', '2': 'passive', '4': 'passive'})
        g = self.rnd.choice(['0201', '0402']); self.c(xi, 'GND', '12pF', g); self.c(xo, 'GND', '12pF', g)
    BGA_LEVELS = [
        (0, [r'TFBGA-100_8x8mm', r'LFBGA-100_10x10mm', r'TFBGA-121_10x10mm', r'LFBGA-144_10x10mm']),
        (3, [r'TFBGA-169_9x9mm_Layout13x13_P0.65', r'TFBGA-216_13x13mm', r'TFBGA-256_13x13mm', r'TFBGA-225_10x10mm_Layout15x15_P0.65']),
        (6, [r'TFBGA-100_5.5x5.5mm', r'LFBGA-153_11.5x13mm_Layout14x14_P0.5', r'TFBGA-324_12x12mm_Layout18x18_P0.65', r'TFBGA-361_13x13mm']),
        (8, [r'TFBGA-289_9x9mm_Layout17x17_P0.5', r'BGA-153_8.0x8.0mm_Layout15x15_P0.5', r'BGA-256_11.0x11.0mm_Layout20x20_P0.5']),
        (10, [r'TFBGA-49_3x3mm_Layout7x7_P0.4', r'BGA-81_4.496x4.377mm_Layout9x9_P0.4']),
    ]

    def bus(self, category):
        N = []
        if category == 'ddr':
            for b in range(self.rnd.choice([1, 2])):
                g = 'DDR_BYTE%d' % b
                N += [self.net_name('DDR_DQ%d' % (8 * b + i), **fast('ddr3', g)) for i in range(8)]
                N += [self.net_name('DDR_DM%d' % b, **fast('ddr3', g))]
                N += [self.net_name('DDR_DQS%d_%s' % (b, s), **fast('ddr3', g, z_diff_ohm=100)) for s in 'PN']
            ga = 'DDR_ADDR'; tol = INTERFACES['ddr3']['tol_addr_mm']
            N += [self.net_name('DDR_CK_%s' % s, **fast('ddr3', ga, z_diff_ohm=100, tolerance_mm=tol)) for s in 'PN']
            N += [self.net_name('DDR_A%d' % i, **fast('ddr3', ga, tolerance_mm=tol)) for i in range(self.rnd.choice([10, 13, 14]))]
            N += [self.net_name('DDR_%s' % s, **fast('ddr3', ga, tolerance_mm=tol)) for s in ('BA0', 'BA1', 'BA2', 'RAS', 'CAS', 'WE', 'CS', 'CKE', 'ODT')]
        elif category == 'sdram':
            N += [self.net_name('SD_D%d' % i, **fast('sdram', 'SDRAM')) for i in range(self.rnd.choice([8, 16]))]
            N += [self.net_name('SD_A%d' % i, **fast('sdram', 'SDRAM')) for i in range(12)]
            N += [self.net_name('SD_%s' % s, **fast('sdram', 'SDRAM')) for s in ('CLK', 'CKE', 'CS', 'RAS', 'CAS', 'WE', 'BA0', 'BA1')]
        else:
            N += [self.net_name('QSPI_%s' % s, **fast('qspi', 'QSPI')) for s in ('CLK', 'CS', 'IO0', 'IO1', 'IO2', 'IO3')]
        return N

    def processor(self, v33, v12, usb):
        st = self.level
        bga = self.rnd.random() < min(0.95, 0.45 + 0.06 * st)
        if self.package is not None: bga = self.package == 'bga'
        if bga:
            match_pattern = [m for ab, ms in self.BGA_LEVELS if ab <= st + 1 for m in ms]
            fine = [m for ab, ms in self.BGA_LEVELS if st - 3 <= ab <= st + 1 for m in ms] or match_pattern
            fp = search(self.rnd, 'Package_BGA', [self.rnd.choice(fine if self.rnd.random() < 0.7 else match_pattern)]) or \
                search(self.rnd, 'Package_BGA', [r'TFBGA-100_8x8mm'])
        else:
            fp = search(self.rnd, 'Package_DFN_QFN', [self.rnd.choice([r'QFN-48-1EP_7x7mm_P0.5mm_EP5.6', r'QFN-56-1EP_7x7mm_P0.4mm', r'QFN-64-1EP_9x9mm_P0.5mm_EP7.3'])]) \
                or search(self.rnd, 'Package_QFP', [r'LQFP-64_10x10mm_P0.5mm$', r'LQFP-100_14x14mm_P0.5mm$'])
        if 'pitch' in self.axes or 'bga' in self.axes:
            bga = bool(self.axes.get('bga', bga))
            pitch = self.axes.get('pitch', 0.8 if bga else 0.5)
            lib = 'Package_BGA' if bga else 'Package_DFN_QFN'
            candidates = [n for n in _list(lib) if re.search(r'_P%smm' % re.escape('%g' % pitch), n)
                          and 32 <= len(self.signal_pads(fp_load(lib + ':' + n)['pads'])[0]) <= 400]
            fp = lib + ':' + self.rnd.choice(candidates) if candidates else None
            if not fp: raise ValueError('No %s footprint with pitch %g' % (lib, pitch))
        n_pins = len(self.signal_pads(fp_load(fp)['pads'])[0])
        realistic = bga and self.version == BGA_MEASUREMENT_VERSION
        if realistic and n_pins < (121 if getattr(self, 'audio_on', False) else 100):





            pitch = re.search(r'_P([0-9.]+)mm', fp).group(1)
            minimum = 121 if getattr(self, 'audio_on', False) else 100
            candidates = [n for n in _list('Package_BGA') if '_P' + pitch + 'mm' in n
                          and minimum <= len(self.signal_pads(fp_load('Package_BGA:' + n)['pads'])[0]) <= 224]
            if not candidates and 'pitch' not in self.axes:
                available = [(n,float(m.group(1))) for n in _list('Package_BGA')
                              for m in [re.search(r'_P([0-9.]+)mm',n)] if m
                              and minimum <= len(self.signal_pads(fp_load('Package_BGA:' + n)['pads'])[0]) <= 224]
                if available:
                    spacing = min(abs(p-float(pitch)) for _,p in available)
                    candidates = [n for n,p in available if abs(abs(p-float(pitch))-spacing)<1e-9]
            if not candidates: raise GeneratorError('No realistic MCU BGA with pitch ' + pitch)
            fp = 'Package_BGA:' + self.rnd.choice(candidates)
            n_pins = len(self.signal_pads(fp_load(fp)['pads'])[0])
        sig = []
        self.net_name('XTAL_IN', category='analog', rank=3, f_Hz=24e6); self.net_name('XTAL_OUT', category='analog', rank=3, f_Hz=24e6)
        sig += [('XTAL_IN', 'input', 'XIN'), ('XTAL_OUT', 'output', 'XOUT'), ('NRST', 'input', 'NRST'), ('BOOT0', 'input', 'BOOT')]
        if usb: sig += [(usb[0], 'bidirectional', 'USB_DP'), (usb[1], 'bidirectional', 'USB_DM')]
        sig += [('I2C_SCL', 'bidirectional', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA')]
        for n in ('I2C_SCL', 'I2C_SDA'): self.net_name(n, **fast('i2c'))
        p_ddr = min(0.6, 0.05 + 0.07 * st) if bga else 0.0
        u_ = self.rnd.random()
        self.mem_kind = 'qspi' if realistic else 'ddr' if u_ < p_ddr else 'sdram' if u_ < p_ddr + 0.25 else 'qspi'
        self.memory_nets = self.bus(self.mem_kind)
        sig += [(n, 'bidirectional', n) for n in self.memory_nets]
        self.spi = [self.net_name('SPI_%s' % s, **fast('spi')) for s in ('SCK', 'MOSI', 'MISO', 'CS')]
        sig += [(n, 'bidirectional', n) for n in self.spi]
        self.uart = [self.net_name('UART_%s' % s, **fast('uart')) for s in ('TX', 'RX')]
        sig += [(n, 'bidirectional', n) for n in self.uart]
        if getattr(self, 'audio_on', False): sig += [(n, 'bidirectional', n) for n in self.i2s_nets()]
        self.adc_in = [self.net_name('ADC_IN%d' % i, category='analog', rank=3) for i in range(self.rnd.choice([1, 2, 4]))]
        sig += [(n, 'input', n) for n in self.adc_in]

        n_pins = len(self.signal_pads(fp_load(fp)['pads'])[0])
        share = self.axes.get('active_pins', min(0.95, 0.30 + 0.065 * st))
        if realistic:
            from learning.bga_occupancy import shares
            signal_budget = shares(n_pins, [65, 39, 32, 88])[0]


            if len(sig) > signal_budget:
                raise GeneratorError('BGA interfaces do not fit into the realistic signal budget')
            signal_budget = max(len(sig), round(signal_budget * min(1., share / .30)))
            n_gpio = signal_budget - len(sig)
        else:
            n_gpio = max(4, int(n_pins * (0.75 if bga else 0.85) * share) - len(sig))
        self.gpio = [self.net_name('GPIO%d' % i, category='signal', rank=0) for i in range(n_gpio)]
        sig += [(n, 'bidirectional', n) for n in self.gpio]
        rails = None
        if realistic:
            rails = [v33, v12]
            for name in ('VIO_AUX', 'VANA_AUX', 'VREF_AUX'):
                rail = self.net_name(name, category='supply', voltage=3.3)
                self.r(v33, rail, '0', '0402')
                rails.append(rail)
        too = self.distribute(fp_load(fp)['pads'], v33, sig, core_set=v12, bga=bga, supplies=rails)
        u = self.part('U', fp, self.rnd.choice(['STM32H7', 'i.MX RT', 'RP2350', 'ECP5', 'iCE40', 'nRF5340', 'ESP32-S3', 'i.MX 6ULL', 'AM335x']),
                      {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, current_val=self.rnd.choice([0.15, 0.3, 0.6]))
        self.mcu_ref = u
        if rails:
            for t in self.part_list:
                if t['ref'].startswith('R') and any(n in rails[2:] for n in t['pins'].values()): self.island_from[t['ref']] = u
        self.decouple(u); self.crystal('XTAL_IN', 'XTAL_OUT'); self.r(v33, 'NRST', '10k'); self.c('NRST', 'GND', '100nF'); self.r('BOOT0', 'GND', '10k')
        self.r(v33, 'I2C_SCL', '4.7k'); self.r(v33, 'I2C_SDA', '4.7k')
        for g in self.gpio[:self.rnd.randint(1, 4)]:
            m = self.net_name('LED_' + g, category='signal'); self.r(g, m, '1k')
            self.part('D', 'LED_SMD:LED_0402_1005Metric' if self.rnd.random() < 0.7 else 'LED_SMD:LED_0603_1608Metric', 'LED', {'1': 'GND', '2': m}, {'1': 'passive', '2': 'passive'})
        self.blocks.append('Processor %s (%s)' % (fp.split(':')[1], 'BGA' if bga else 'QFN/QFP')); return u

    def memory(self, v33):
        category = getattr(self, 'mem_kind', 'qspi')
        if category == 'qspi' or not self.memory_nets:
            fp = search(self.rnd, 'Package_SON', [r'WSON-8-1EP_6x5mm', r'Texas_DSG0008A_WSON-8']) or search(self.rnd, 'Package_SO', [r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            ns = (self.memory_nets + self.spi)[:6]
            roles = [(n, 'bidirectional', n) for n in ns] + [(v33, 'power_in', 'VCC'), ('GND', 'power_in', 'GND')]
            F = fp_load(fp); ps, ep = self.signal_pads(F['pads']); too = {p['num']: roles[i] for i, p in enumerate(ps) if i < len(roles)}
            if ep: too[ep['num']] = ('GND', 'power_in', 'EP')
            self.blocks.append('QSPI flash 133 MHz')
        else:
            fp = search(self.rnd, 'Package_SO', [r'TSOP-II-54']) if category == 'sdram' else search(self.rnd, 'Package_BGA', [r'FBGA-96_', r'FBGA-78_'])
            sig = [(n, 'bidirectional', n) for n in self.memory_nets]
            too = self.distribute(fp_load(fp)['pads'], v33, sig, bga=category == 'ddr', vdd_share=0.15)
            self.blocks.append('SDRAM 166 MHz' if category == 'sdram' else 'DDR3-800 (BGA, %d nets, length groups)' % len(self.memory_nets))
        u = self.part('U', fp, category.upper(), {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, current_val=0.1)
        self.decouple(u); return u

    def i2s_nets(self):
        self.i2s_kind = 'tdm8' if self.rnd.random() < 0.5 else 'i2s_4x2'
        N = [self.net_name('I2S_MCLK', **fast('mclk')), self.net_name('I2S_BCLK', **fast('i2s', 'I2S')), self.net_name('I2S_LRCLK', **fast('i2s', 'I2S'))]
        data = ['I2S_SDOUT', 'I2S_SDIN'] if self.i2s_kind == 'tdm8' else ['I2S_SDOUT%d' % i for i in range(4)] + ['I2S_SDIN0']
        N += [self.net_name(d, **fast('i2s', 'I2S')) for d in data]
        self.i2s = N; return N

    def audio(self, v33, va):
        if not getattr(self, 'i2s', None): self.i2s_nets()
        fp = search(self.rnd, 'Package_QFP', [r'LQFP-48_7x7mm_P0.5mm$', r'LQFP-64_10x10mm_P0.5mm$']) if self.rnd.random() < 0.5 else \
            search(self.rnd, 'Package_DFN_QFN', [r'QFN-40-1EP_6x6mm_P0.5mm_EP4.6x4.6mm$', r'QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm$'])
        channels = self.rnd.choice([4, 8, 8])
        off = []
        for k in range(channels):
            for s in 'PN': off.append(self.net_name('AOUT%d_%s' % (k + 1, s), category='analog', rank=3, f_Hz=20e3))
        one = [self.net_name('AIN%d_%s' % (k + 1, s), category='analog', rank=3, f_Hz=20e3) for k in range(2) for s in 'LR']
        vcom = self.net_name('AUD_VCOM', category='analog', rank=3); vq = self.net_name('AUD_VREF', category='analog', rank=3)
        sig = [(n, 'bidirectional', n.split('_', 1)[1]) for n in self.i2s] + [('I2C_SCL', 'input', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA')] + \
              [(n, 'output', n) for n in off] + [(n, 'input', n) for n in one] + [(vcom, 'passive', 'VCOM'), (vq, 'passive', 'VREF')]
        for _ in range(3): sig.insert(self.rnd.randrange(len(sig)), (va, 'power_in', 'AVDD'))
        too = self.distribute(fp_load(fp)['pads'], v33, sig, vdd_share=0.08)
        u = self.part('U', fp, self.rnd.choice(['CS42448', 'AK4458', 'TLV320AIC3104', 'PCM3168A', 'ADAU1966']),
                      {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, current_val=0.08)
        self.decouple(u); self.c(vcom, 'GND', '10uF', '0603'); self.c(vcom, 'GND', '100nF'); self.c(vq, 'GND', '10uF', '0603')
        sockets = []
        for k in range(0, channels, 2):
            opv = search(self.rnd, 'Package_SO', [r'MSOP-8_3x3mm_P0.65mm$', r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            o = ['AUD_OUT%d' % (k + 1), 'AUD_OUT%d' % (k + 2)]; m = ['AUD_FB%d' % (k + 1), 'AUD_FB%d' % (k + 2)]; pl = ['AUD_IP%d' % (k + 1), 'AUD_IP%d' % (k + 2)]
            for n in o + m + pl: self.net_name(n, category='analog', rank=3, f_Hz=20e3)
            self.part('U', opv, self.rnd.choice(['OPA1612', 'NE5532', 'OPA1678', 'LM4562']),
                      {'1': o[0], '2': m[0], '3': pl[0], '4': 'GND', '8': va, '7': o[1], '6': m[1], '5': pl[1]},
                      {'1': 'output', '2': 'input', '3': 'input', '4': 'power_in', '8': 'power_in', '7': 'output', '6': 'input', '5': 'input'},
                      {'1': 'OUT', '2': 'IN-', '3': 'IN+', '4': 'GND', '8': 'VCC', '7': 'OUT', '6': 'IN-', '5': 'IN+'}, current_val=0.01)
            self.c(va, 'GND', '100nF')
            for j in range(2):
                self.r('AOUT%d_N' % (k + j + 1), m[j], '2.2k'); self.r(m[j], o[j], '2.2k'); self.c(m[j], o[j], '1nF')
                self.r('AOUT%d_P' % (k + j + 1), pl[j], '2.2k'); self.r(pl[j], vcom, '2.2k'); self.c(pl[j], 'GND', '1nF')
                lo = self.net_name('LINE%d' % (k + j + 1), category='analog', rank=3, f_Hz=20e3); self.r(o[j], lo, '47'); self.c(lo, 'GND', '2.2nF')
            sockets.append(['LINE%d' % (k + 1), 'LINE%d' % (k + 2)])
        for k, (a, b) in enumerate(sockets[:2]):
            self.part('J', search(self.rnd, 'Connector_Audio', [r'Jack_3.5mm_CUI_SJ-3523-SMT_Horizontal', r'Jack_3.5mm_CUI_SJ1-3523N_Horizontal']),
                      'Line Out %d' % (k + 1), {'S': 'GND', 'T': a, 'R': b}, {'S': 'passive', 'T': 'passive', 'R': 'passive'}, pinned='right')
        rest = [n for ab in sockets[2:] for n in ab]
        if rest:
            fpc = search(self.rnd, 'Connector_FFC-FPC', [r'Hirose_FH12-1[0-2]S-0.5SH_1x1[0-2]-1MP_P0.50mm_Horizontal', r'.*1x10.*P0.50mm']) or \
                search(self.rnd, 'Connector_JST', [r'JST_SH_SM04B'])
            ps, _ = self.signal_pads(fp_load(fpc)['pads']); nz = rest + ['GND'] * len(ps)
            too = {p['num']: nz[i] for i, p in enumerate(ps) if i < len(nz)}
            self.part('J', fpc, 'Audio Out', too, {k_: 'passive' for k_ in too}, pinned='bottom')
        jin = search(self.rnd, 'Connector_Audio', [r'Jack_3.5mm_CUI_SJ-3524-SMT_Horizontal', r'Jack_3.5mm_CUI_SJ1-3513N_Horizontal'])
        li = [self.net_name('LINEIN_%s' % s, category='analog', rank=3, f_Hz=20e3) for s in 'LR']
        self.part('J', jin, 'Line In', {'S': 'GND', 'T': li[0], 'R': li[1]}, {'S': 'passive', 'T': 'passive', 'R': 'passive'}, pinned='left')
        for s, n in zip('LR', li):
            self.c(n, 'AIN1_%s' % s, '10uF', '0603'); self.r('AIN1_%s' % s, vcom, '10k'); self.c('AIN1_%s' % s, 'GND', '1nF')
        self.blocks.append('Audio codec %d channels on %s (%s), OPV buffer, jacks' % (channels, 'TDM8' if self.i2s_kind == 'tdm8' else 'I2S 4x2', fp.split(':')[1]))

    def extension(self):
        real_bga = False
        candidates = list(getattr(self, 'gpio', []))
        if self.version == BGA_MEASUREMENT_VERSION and getattr(self, 'mcu_ref', None):
            mcu = next(t for t in self.part_list if t['ref'] == self.mcu_ref)
            if 'BGA' in mcu['fp']:
                real_bga = True



                candidates = list(dict.fromkeys(candidates + [n for pin,n in mcu['pins'].items()
                                  if mcu['type_'].get(pin) != 'power_in']))
        open_items = [g for g in candidates if sum(1 for t in self.part_list for n in t['pins'].values() if n == g) < 2]
        while open_items:
            k = min(len(open_items), 40); demand = k + (k + 2) // 3 + 1
            pole = sorted(int(m.group(1)) for n in _list('Connector_FFC-FPC') for m in [re.search(r'1x(\d+)-1MP_P0\.50mm_Horizontal', n)] if m)
            p_ = next((x for x in pole if x >= demand), None)
            if p_ is None: k = max(1, (pole[-1] * 3) // 4 - 1); p_ = pole[-1]
            number = ('%02d' % p_) if real_bga else str(p_)
            fp = search(self.rnd, 'Connector_FFC-FPC', [r'Hirose_FH12-%dS-0.5SH_1x%s-1MP' % (p_, number), r'.*_1x%s-1MP_P0\.50mm_Horizontal' % number])
            if not fp: break
            ps, _ = self.signal_pads(fp_load(fp)['pads']); pins = {}; type_ = {}; it = iter(open_items[:k])
            for i, p in enumerate(ps):
                if i % 4 == 0: pins[p['num']] = 'GND'; type_[p['num']] = 'passive'; continue
                n = next(it, None)
                if n is None: break
                pins[p['num']] = n; type_[p['num']] = 'bidirectional'
            self.current = None
            self.part('J', fp, 'Extension', pins, type_, pinned=self.rnd.choice(['upper', 'bottom', 'left', 'right']))
            open_items = open_items[k:]; self.blocks.append('Expansion connector %d poles' % p_)

    def sensors(self, v33):
        for _ in range(self.rnd.randint(1, 3 + int(self.level / 3))):
            fp = search(self.rnd, 'Package_LGA', [self.rnd.choice([r'Bosch_LGA-14_3x2.5mm', r'Bosch_LGA-8_2.5x2.5mm', r'LGA-12_2x2mm', r'LGA-14_3x2.5mm'])])
            ps, _ = self.signal_pads(fp_load(fp)['pads'])
            inten = self.net_name('INT_%d' % len(self.blocks), category='signal')
            roles = [(v33, 'power_in', 'VDD'), ('GND', 'power_in', 'GND'), ('I2C_SCL', 'input', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA'), (inten, 'output', 'INT'),
                      ('GND', 'power_in', 'GND'), (v33, 'power_in', 'VDDIO')]
            too = {p['num']: roles[i] for i, p in enumerate(ps) if i < len(roles)}
            u = self.part('U', fp, self.rnd.choice(['BMI270', 'BMP390', 'SHT40', 'LIS2DH']), {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, current_val=0.005)
            self.decouple(u); self.blocks.append('Sensor %s' % fp.split(':')[1])

    def analog(self, v33, va):
        if not getattr(self, 'adc_in', None): self.adc_in = [self.net_name('ADC_IN0', category='analog', rank=3)]
        for i, ain in enumerate(self.adc_in[:2]):
            fp = search(self.rnd, 'Package_SO', [r'MSOP-8_3x3mm_P0.65mm$', r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            vin_p, fbn = self.net_name('AIN%d_P' % i, category='analog', rank=3), self.net_name('OPV%d_N' % i, category='analog', rank=3)
            outn = self.net_name('OPV%d_OUT' % i, category='analog', rank=3)
            u = self.part('U', fp, self.rnd.choice(['OPA2365', 'AD8605', 'MCP6022', 'TLV9062']),
                          {'1': outn, '2': fbn, '3': vin_p, '4': 'GND', '8': va, '5': 'OPV%d_B_P' % i, '6': 'OPV%d_B_N' % i, '7': 'OPV%d_B_OUT' % i},
                          {'1': 'output', '2': 'input', '3': 'input', '4': 'power_in', '8': 'power_in', '5': 'input', '6': 'input', '7': 'output'},
                          {'1': 'OUT', '2': 'IN-', '3': 'IN+', '4': 'GND', '8': 'VCC', '5': 'IN+', '6': 'IN-', '7': 'OUT'}, current_val=0.002)
            self.net_name('OPV%d_B_P' % i, category='analog', rank=3); self.net_name('OPV%d_B_N' % i, category='analog', rank=3); self.net_name('OPV%d_B_OUT' % i, category='analog', rank=3)
            self.r(outn, fbn, '10k'); self.r(fbn, 'GND', '1k'); self.c(outn, fbn, '100pF')
            self.r('OPV%d_B_OUT' % i, 'OPV%d_B_N' % i, '0'); self.r('OPV%d_B_P' % i, outn, '1k')
            self.r(vin_p, 'GND', '100k'); self.r('OPV%d_B_OUT' % i, ain, '47'); self.c(ain, 'GND', '1nF', '0402')
            self.c(va, 'GND', '100nF', '0402')
            self.part('J', search(self.rnd, 'Connector_JST', [r'JST_GH_SM02B', r'JST_SH_SM04B']), 'Sensor', {'1': vin_p, '2': 'GND'}, {'1': 'passive', '2': 'passive'}, pinned='right')
        if self.rnd.random() < 0.6:
            fp = search(self.rnd, 'Package_SO', [r'TSSOP-16_4.4x5mm', r'MSOP-10_3x3mm']) or search(self.rnd, 'Package_DFN_QFN', [r'QFN-20-1EP_4x4mm_P0.5mm_EP2.5'])
            ps, ep = self.signal_pads(fp_load(fp)['pads'])
            sig = [(va, 'power_in', 'AVDD'), ('GND', 'power_in', 'GND'), (v33, 'power_in', 'VDD'), ('VREF', 'input', 'VREF')] + \
                  [(n, 'bidirectional', n) for n in self.spi] + [(self.net_name('ADC_X%d' % k, category='analog', rank=3), 'input', 'AIN') for k in range(4)]
            too = {p['num']: sig[i] for i, p in enumerate(ps) if i < len(sig)}
            if ep: too[ep['num']] = ('GND', 'power_in', 'EP')
            self.net_name('VREF', category='analog', rank=3)
            u = self.part('U', fp, self.rnd.choice(['ADS8688', 'AD5686', 'MCP3208']), {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, current_val=0.005)
            self.decouple(u); self.c('VREF', 'GND', '10uF', '0603'); self.blocks.append('ADC/DAC %s' % fp.split(':')[1])
        self.blocks.append('Analog: %d OPV stages' % min(2, len(self.adc_in)))

    def radio(self, v33):
        fp = search(self.rnd, 'RF_Module', [r'Raytac_MDBT42Q', r'E73-2G4M04S', r'Laird_BL652', r'ESP32-C3-WROOM-02$', r'BLE112-A'])
        if not fp: return
        ps, ep = self.signal_pads(fp_load(fp)['pads'])
        sig = [(v33, 'power_in', 'VCC'), ('GND', 'power_in', 'GND')] + [(n, 'bidirectional', n) for n in getattr(self, 'uart', [])] + [('GND', 'power_in', 'GND')]
        too = {p['num']: sig[i] for i, p in enumerate(ps) if i < len(sig)}
        if ep: too[ep['num']] = ('GND', 'power_in', 'EP')
        board_side, _ = antenna_side(fp_load(fp))
        u = self.part('U', fp, 'Bluetooth', {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()}, pinned=board_side, current_val=0.03)
        self.c(v33, 'GND', '10uF', '0603'); self.c(v33, 'GND', '100nF', '0402'); self.blocks.append('Bluetooth module %s' % fp.split(':')[1])
        self.radio_ref = u

    def power_supply(self):
        hv = self.net_name('HV_DC', category='current_val', voltage=325.0, current_A=0.05); sw = self.net_name('SW_PRI', category='sw'); secondary = self.net_name('SECONDARY', category='sw')
        v5 = self.net_name('+5V_NT', category='current_val', voltage=5.0, current_A=2.0)
        self.part('J', search(self.rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal', r'TerminalBlock_Phoenix_PT-1,5-2']) or
                  search(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02']), 'AC', {'1': 'AC_L', '2': 'AC_N'}, {'1': 'passive', '2': 'passive'}, pinned='left')
        self.net_name('AC_L', category='current_val', current_A=0.1); self.net_name('AC_N', category='current_val', current_A=0.1)
        self.part('D', search(self.rnd, 'Diode_SMD', [r'Diode_Bridge_Vishay_MBLS', r'Diode_Bridge']) or search(self.rnd, 'Package_TO_SOT_SMD', [r'TO-269AA']) or
                  'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', 'MB6S', {'1': hv, '2': 'GND', '3': 'AC_L', '4': 'AC_N'}, {'1': 'passive', '2': 'passive', '3': 'passive', '4': 'passive'})
        self.part('C', search(self.rnd, 'Capacitor_SMD', [r'CP_Elec_8x10.5', r'CP_Elec_6.3x5.4$']), '10uF 400V', {'1': hv, '2': 'GND'}, {'1': 'passive', '2': 'passive'})
        u = self.part('U', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', 'LNK306', {'1': 'GND', '2': 'GND', '4': 'FB_NT', '5': sw, '7': sw, '8': 'BP'},
                      {'1': 'power_in', '2': 'power_in', '4': 'input', '5': 'power_out', '7': 'power_out', '8': 'passive'})
        self.fsw[u] = 66e3; self.net_name('FB_NT', category='analog', rank=2)
        tr = search(self.rnd, 'Transformer_SMD', [r'Transformer_Coilcraft_', r'Pulse_']) or search(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRP1038', r'L_Bourns_SRN8040'])
        tf = fp_load(tr); ps, _ = self.signal_pads(tf['pads']); roles = [(hv, 'passive', 'P1'), (sw, 'passive', 'P2'), (secondary, 'passive', 'S1'), ('GND', 'passive', 'S2')]
        too = {p['num']: roles[i] for i, p in enumerate(ps) if i < len(roles)}
        self.part('T', tr, 'Transformer', {k: v[0] for k, v in too.items()}, {k: v[1] for k, v in too.items()}, {k: v[2] for k, v in too.items()})
        self.part('D', search(self.rnd, 'Diode_SMD', [r'D_SMA$', r'D_SMB$', r'D_SOD-123F']), 'SS34', {'1': v5, '2': secondary}, {'1': 'passive', '2': 'passive'})
        self.c('BP', 'GND', '100nF', '0603'); self.c(v5, 'GND', '220uF', '1210'); self.c(v5, 'GND', '22uF', '1206')
        self.r(v5, 'FB_NT', '10k', '0603'); self.r('FB_NT', 'GND', '2.2k', '0603')
        self.blocks.append('Mini power supply (flyback archetype) 325 V → 5 V'); return v5, 5.0


    def build(self, category, power_share=0.0):
        rnd = self.rnd
        self.power_board = category == 'power' or (category == 'auto' and random.Random(self.seed * 104729 + 19).random() < float(power_share))
        if self.power_board:
            uin = rnd.choice([12.0, 24.0]); vin = self.net_name('VIN', category='current_val', voltage=uin)
            fp = search(rnd, 'Connector_AMASS', [r'AMASS_XT60-M_1x02']) or search(rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal'])
            self.part('J', fp, 'Power input %g V' % uin, {'1': vin, '2': 'GND'}, {'1': 'power_out', '2': 'power_in'}, pinned='left')
            self.c(vin, 'GND', '22uF', '1210')
            for u, a in ((5.0, 8.0), (3.3, 6.0), (1.2, 10.0)):
                self.buck(vin, uin, u, a * (1 + .1 * self.level), '+%dV%s' % (int(u), str(round((u - int(u)) * 10)) if u != int(u) else ''))
            self.net_name('GND', category='gnd'); self.blocks.append('Power board: two signal layers, rest power'); return
        if category == 'power_supply' or (category == 'auto' and rnd.random() < 0.12):
            vin, uin = self.power_supply(); usb = None
        else:
            vin, uin, usb = self.connector_input()
        ratio = []
        high = 1.0 + 0.1 * self.level
        def current_value(amount): return self.axes.get('ampere', amount)
        v33 = self.buck(vin, uin, 3.3, current_value(round(rnd.choice([1.0, 2.0, 3.0]) * high, 1)), '+3V3') if uin > 4 else self.ldo(vin, 3.3, '+3V3')
        v12 = self.buck(vin, uin, 1.2, current_value(round(rnd.choice([1.5, 3.0, 6.0, 10.0]) * high, 1)), '+1V2') if rnd.random() < 0.7 else self.ldo(v33, 1.2, '+1V2')
        if rnd.random() < 0.4 + 0.03 * self.level and uin >= 9: self.buck(vin, uin, 5.0, current_value(round(rnd.choice([3.0, 5.0, 8.0]) * high, 1)), '+5V')
        if rnd.random() < 0.3 + 0.03 * self.level: self.boost(v33, 3.3, 12.0, min(.3, self.axes['ampere']) if 'ampere' in self.axes else round(0.3 * high, 2), '+12V_B')
        if self.level >= 3 and rnd.random() < 0.08 * self.level and uin > 4:
            self.buck(vin, uin, 1.8, current_value(round(rnd.choice([1.0, 2.0, 4.0]) * high, 1)), '+1V8')
        va = self.ldo(v33, 3.0, '+3V0A') if rnd.random() < 0.6 else v33
        if va != v33: self.net_name(va, category='supply', voltage=3.0)
        self.audio_on = category == 'audio' or (category in ('auto', 'mcu_board', 'analog') and rnd.random() < min(0.5, 0.1 + 0.05 * self.level))
        if self.vector_mode: self.audio_on = category == 'audio'
        if self.audio_on and va == v33: va = self.ldo(v33, 3.0, '+3V0A')
        if category in ('auto', 'mcu_board', 'radio', 'analog', 'audio'):
            self.processor(v33, v12, usb)
            if category != 'analog' or rnd.random() < 0.5: self.memory(v33)
            self.sensors(v33)
        if category in ('auto', 'analog', 'mcu_board') and (category == 'analog' or rnd.random() < 0.6): self.analog(v33, va)
        if category in ('auto', 'radio', 'mcu_board') and (category == 'radio' or rnd.random() < 0.5): self.radio(v33)
        if self.audio_on: self.audio(v33, va)
        self.extension()
        self.net_name('GND', category='gnd')

    def axes_complete(self):
        a = self.axes
        if not a: return
        vin = next(n for n, e in self.net_list.items() if e.get('voltage', 0) >= 4)
        uin = self.net_list[vin]['voltage']
        for i in range(max(0, int(a.get('current_paths', 0)) - len(self.rails))):
            self.buck(vin, uin, 1.8, a.get('ampere', 1.0), '+TEACHER%d' % i)

        def rc(i):
            n = self.net_name('TEACHER_A%d' % i, category='analog', rank=3)
            self.r(vin, n, '100k', '0402'); self.c(n, 'GND', '1nF', '0402')
            self.blocks.append('RC-MeasurementChain')
        for i in range(int(a.get('pairs', 0))):
            nets = [self.net_name('TEACHER_DIFF%d_%s' % (i, s), **fast('usb_hs', 'TEACHER_PAIR%d' % i, z_diff_ohm=90)) for s in ('P', 'N')]
            for _ in range(2):
                self.part('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical', 'Differential-Port',
                          {'1': nets[0], '2': nets[1], '3': 'GND'}, {'1': 'bidirectional', '2': 'bidirectional', '3': 'power_in'})
        for i in range(int(a.get('length_groups', 0))):
            n = self.net_name('TEACHER_CLK%d' % i, **fast('qspi', 'TEACHER_CLOCK%d' % i))
            m = self.net_name('TEACHER_DATA%d' % i, **fast('qspi', 'TEACHER_CLOCK%d' % i))
            for _ in range(2):
                self.part('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical', 'Clock-Port',
                          {'1': n, '2': m, '3': 'GND'}, {'1': 'bidirectional', '2': 'bidirectional', '3': 'power_in'})
        i = 0
        while (len(self.part_list) < a.get('part_list', 0) or
               sum(e.get('category') == 'analog' for e in self.net_list.values()) / max(1, len(self.net_list)) < a.get('analog', 0)):
            rc(i); i += 1
            if i > 200: raise ValueError('Analog/part target not reachable')
        for e in self.net_list.values():
            if e.get('interface') and e.get('f_Hz'):
                e['f_Hz'] *= a.get('clock', 1.0)
                if e.get('data_rate'): e['data_rate'] *= a.get('clock', 1.0)
                if e.get('tr_ns'): e['tr_ns'] /= a.get('clock', 1.0)


    def load_list(self):
        load = {}
        for n, a in sorted(self.rails.items()):
            version = [(t['ref'], num) for t in self.part_list for num, net in sorted(t['pins'].items())
                   if net == n and t['type_'].get(num) == 'power_in' and t['ref'][0] in 'UJ']
            if not version:
                fp = (search(self.rnd, 'Connector_AMASS', [r'AMASS_XT60-M_1x02', r'AMASS_XT60']) if a > 9 else None) or \
                     (search(self.rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal', r'TerminalBlock_Phoenix_PT-1,5-2']) if a > 2.5 else None) or \
                     (search(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02']) if a > 1.5 else None) or \
                     search(self.rnd, 'Connector_JST', [r'JST_PH_S2B-PH-SM4', r'JST_PH_B2B-PH-K_1x02'])
                j = self.part('J', fp, 'Output %s' % n, {'1': n, '2': 'GND'}, {'1': 'power_in', '2': 'power_in'}, pinned='right')
                version = [(j, '1')]; self.blocks.append('Output %s %.1f A' % (n, a))
            for ref, num in version:
                k_ = ref if ref.startswith('J') else '%s:%s' % (ref, num)
                load[k_] = round(load.get(k_, 0) + a / len(version), 4)
        return load

    def write_out(self, goal, structure=False, anchor_legal=False):
        rnd = self.rnd; os.makedirs(goal, exist_ok=True); L = self.cu_layers
        LOAD = self.load_list()
        real = self.version == REAL_VERSION
        if real:

            for suffix in ('P', 'N'):
                self.net_name('USB_TEST_' + suffix, **fast('usb_hs', 'USB_TEST', z_diff_ohm=90, tolerance_mm=.10))
            for pinned in ('upper', None):
                self.part('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical',
                          'USB-TestPort', {'1': 'USB_TEST_P', '2': 'USB_TEST_N', '3': 'GND'}, pinned=pinned)
        names = ['F'] + ['In%d' % i for i in range(1, L - 1)] + ['B']

        count = {}
        for t in self.part_list:
            for n in t['pins'].values():
                if n: count[n] = count.get(n, 0) + 1
        for t in self.part_list:
            t['pins'] = {k: (v if count.get(v, 0) >= 2 else '') for k, v in t['pins'].items()}
        surface = 0.0; F = {}
        for t in self.part_list:
            F[t['ref']] = fp_load(t['fp']); x0, y0, x1, y1 = F[t['ref']]['crt']; surface += (x1 - x0) * (y1 - y0)
        A = surface * self.factor; since = rnd.uniform(1.0, 1.7); W = math.sqrt(A * since); H = A / W
        if real:


            W, H = W*1.20, H*1.20
        W, H = round(W, 1), round(H, 1)
        outline = [[(0.0, 0.0), (W, 0.0), (W, H), (0.0, H)]]
        pads, fps, crt, keep = [], [], [], []
        if real:
            outline, holes, edges = real_contour(W, H)
            from shapely.geometry import Polygon, box
            board_poly = Polygon(outline[0], holes[0])


        operate = set()
        if anchor_legal or real:
            from analysis.netclasses import OPERATE_HINT
            operate = {t['ref'] for t in self.part_list if OPERATE_HINT.search(t['fp'] + ' ' + t['value'])}
        if real: operate = {r for r in operate if not r.startswith('J')}
        occupied = []

        def vacant(X, Y, c):
            x0, y0, x1, y1 = c
            r = (round(X, 3) + x0 - 0.3, round(Y, 3) + y0 - 0.3,
                 round(X, 3) + x1 + 0.3, round(Y, 3) + y1 + 0.3)
            if real and not board_poly.covers(box(X+x0, Y+y0, X+x1, Y+y1)):
                return False
            if all(r[2] <= b[0] or b[2] <= r[0] or r[3] <= b[1] or b[3] <= r[1] for b in occupied):
                occupied.append(r); return True
            return False

        def border_slot(t, x0, y0, x1, y1):
            w, h = x1 - x0, y1 - y0


            for s in [t['pinned']] + [s for s in ('upper', 'bottom', 'left', 'right') if s != t['pinned']]:
                if s in ('upper', 'bottom'):
                    n_ = max(0, int((W - w) / 0.5))
                    cand = [(-x0 + k * 0.5, -y0 if s == 'upper' else H - y1) for k in range(n_ + 1)]
                else:
                    n_ = max(0, int((H - h) / 0.5))
                    cand = [(-x0 if s == 'left' else W - x1, -y0 + k * 0.5) for k in range(n_ + 1)]
                rnd.shuffle(cand) if s != 'bottom' else cand.sort(key=lambda p: abs(p[0] + (x0 + x1) / 2 - W / 2))
                for X, Y in cand:
                    if X + x1 > W + 1e-6 or Y + y1 > H + 1e-6 or X + x0 < -1e-6 or Y + y0 < -1e-6: continue
                    if vacant(X, Y, (x0, y0, x1, y1)):
                        t['pinned'] = s; return X, Y
            raise GeneratorError('Task cannot be generated: no free edge spot for ' + t['ref'])

        def operate_slot(c):
            x0, y0, x1, y1 = c
            cand = [(-x0 + i * 0.5, -y0 + j * 0.5)
                    for i in range(max(0, int((W - x1 + x0) / 0.5)) + 1)
                    for j in range(max(0, int((H - y1 + y0) / 0.5)) + 1)]
            rnd.shuffle(cand)
            for X, Y in cand:
                if X + x1 <= W and Y + y1 <= H and vacant(X, Y, c): return X, Y
            raise GeneratorError('Task cannot be generated: no free spot for control element')

        def border_slot_prev(t, x0, y0, x1, y1):
            w, h = x1 - x0, y1 - y0; s = t['pinned']
            if s in ('upper', 'bottom'): n_ = max(1, int((W - w) / 0.5)); cand = [(-x0 + k * 0.5, -y0 if s == 'upper' else H - y1) for k in range(n_ + 1)]
            else: n_ = max(1, int((H - h) / 0.5)); cand = [(-x0 if s == 'left' else W - x1, -y0 + k * 0.5) for k in range(n_ + 1)]
            rnd.shuffle(cand) if s != 'bottom' else cand.sort(key=lambda p: abs(p[0] + (x0 + x1) / 2 - W / 2))
            for X, Y in cand:
                r = (X + x0 - 0.3, Y + y0 - 0.3, X + x1 + 0.3, Y + y1 + 0.3)
                if X + x1 > W + 1e-6 or Y + y1 > H + 1e-6 or X + x0 < -1e-6 or Y + y0 < -1e-6: continue
                if all(r[2] <= b[0] or b[2] <= r[0] or r[3] <= b[1] or b[3] <= r[1] for b in occupied):
                    occupied.append(r); return X, Y
            t['pinned'] = None
            return rnd.uniform(-x0, max(-x0, W - x1)), rnd.uniform(-y0, max(-y0, H - y1))

        def order(t):
            area = -(lambda c: (c[2] - c[0]) * (c[3] - c[1]))(F[t['ref']]['crt'])
            if real: return (not bool(t['pinned']), not t['ref'].startswith('J'), t['ref'] not in operate, area)
            return (not bool(t['pinned']), t['ref'] not in operate, area) if anchor_legal else area

        border_search = border_slot if anchor_legal or real else border_slot_prev
        for t in sorted(self.part_list, key=order):
            x0, y0, x1, y1 = F[t['ref']]['crt']
            if t['pinned']: X, Y = border_search(t, x0, y0, x1, y1)
            elif real and t['ref'].startswith('J'):
                t['pinned'] = 'bottom'
                X, Y = border_slot(t, x0,y0,x1,y1)
                t['pinned'] = None
            elif t['ref'] in operate: X, Y = operate_slot((x0, y0, x1, y1))
            elif real:


                for _ in range(20000):
                    X, Y = rnd.uniform(-x0, W-x1), rnd.uniform(-y0, H-y1)
                    if vacant(X, Y, (x0,y0,x1,y1)):
                        occupied.pop(); break
                else: raise GeneratorError('No connector-free start spot: %s board=%s part=%s anchor=%s' % (t['ref'],(W,H),(x0,y0,x1,y1),occupied))
            else: X, Y = rnd.uniform(-x0, max(-x0, W - x1)), rnd.uniform(-y0, max(-y0, H - y1))
            X, Y = round(X, 3), round(Y, 3)
            tht = any(p['tht'] for p in F[t['ref']]['pads'])
            fps.append(dict(ref=t['ref'], x=X, y=Y, rot=0.0, side='F', fp=t['fp'].split(':')[1], lib=t['fp'].split(':')[0], value=t['value'],
                            sheet='', locked=bool(t['pinned'] or t['ref'] in operate), attr='through_hole' if tht else 'smd', dnp=False, board_only=False,
                            n_pads=len(F[t['ref']]['pads'])))
            crt.append(dict(ref=t['ref'], side='F', polys=[[(X + x0, Y + y0), (X + x1, Y + y0), (X + x1, Y + y1), (X + x0, Y + y1)]]))
            for p in F[t['ref']]['pads']:
                lays = names if p['tht'] else ['F']
                poly = [(round(X + u, 5), round(Y + v, 5)) for u, v in pad_poly(p)]
                xs, ys = [q[0] for q in poly], [q[1] for q in poly]
                net = '' if p['npth'] else t['pins'].get(p['num'], '')
                pads.append(dict(ref=t['ref'], num=p['num'], net=net, layers=lays, polys={Lx: [poly] for Lx in lays}, mask={},
                                 x=round(X + p['x'], 5), y=round(Y + p['y'], 5), bbox=[min(xs), min(ys), max(xs), max(ys)],
                                 drill=p['drill'] if p['tht'] else 0.0,
                                 **(dict(drill_size=p['drill_size'], hole_polys=[[(round(X+u,5),round(Y+v,5)) for u,v in q] for q in p['hole_polys']]) if p['tht'] else {}),
                                 fp_side='F', pinfunc=t['func'].get(p['num'], ''),
                                 pintype=t['type_'].get(p['num'], '') if net else ''))
            for i, s in enumerate(F[t['ref']]['blocked']):
                lg = keepout_layers(s['cu_layers'], names)
                if lg: keep.append(dict(ref=t['ref'], name='keepout %d' % i, polys=[[(X + u, Y + v) for u, v in s['poly']]],
                                        cu_layers=lg, prohibitions=dict(s['prohibitions'])))
            if t['ref'] == getattr(self, 'radio_ref', None) and not F[t['ref']]['blocked']:
                _, (a0, b0, a1, b1) = antenna_side(F[t['ref']])
                if a1 - a0 > 0.5 and b1 - b0 > 0.5:
                    keep.append(dict(ref=t['ref'], name='antenna keepout', polys=[[(X + a0, Y + b0), (X + a1, Y + b0), (X + a1, Y + b1), (X + a0, Y + b1)]],
                                     cu_layers=names, prohibitions=dict(track_list=True, vias=True, areas=True, pads=False, part_list=True)))
        if self.version == BGA_MEASUREMENT_VERSION:
            from learning.bga_occupancy import too_target
            bga_refs = {t['ref'] for t in self.part_list if 'BGA' in t['fp']}
            for p in pads: p['_bga'] = p['ref'] in bga_refs
            too_target(pads)
        cnt = {}
        for p in pads:
            if p['net']: cnt[p['net']] = cnt.get(p['net'], 0) + 1
        for p in pads:
            if p['net'] and cnt[p['net']] < 2: p['net'] = ''; p['pintype'] = ''
        nets = sorted({p['net'] for p in pads if p['net']})
        version = self.version + ('+anchor-legal1' if anchor_legal else '') + ('+r35-structure1' if structure else '')
        B = dict(src='task seed=%d (learning/tasks.py v%s)' % (self.seed, version), pads=pads, outline=outline, courtyards=crt, keepouts=keep,
                 fps=fps, via_keepout=[], nets=nets, netclass={n: 'Default' for n in nets}, net_classes={}, design={}, copper_graphic=[], cu_layers=L)
        if real:
            B.update(outline_holes=holes, edge_primitives=edges)
        json.dump(B, open(os.path.join(goal, 'board.json'), 'w'))

        fine = min([abs(a['x'] - b['x']) for t in self.part_list if 'BGA' in t['fp'] for a, b in [sorted(F[t['ref']]['pads'], key=lambda p: (p['y'], p['x']))[:2]]] or [1.0])
        via = '0.30/0.15' if fine < 0.6 else '0.40/0.20' if fine < 0.75 else '0.45/0.20'
        clr = 0.075 if fine < 0.6 else 0.09


        sw_ = 0.075 if fine < 0.6 else 0.09 if fine < 0.75 else 0.1
        router = dict(cu_layers=L, drc=clr, drcpad=clr + 0.01, clr=round(clr + 0.005, 3), edge=0.2, h2h=0.2, via=via, sigw=sw_, sigmin=sw_, vip='1')
        RG = dict(source='learning/tasks.py', layers_source=L, raw_data=dict(min_clearance=clr, min_track_width=router['sigmin'], min_copper_edge_clearance=0.2,
                  min_hole_clearance=0.2, min_hole_to_hole=0.2, netclass_clearance=clr, netclass_track=router['sigw'],
                  netclass_via=[float(v) for v in via.split('/')], via_dimensions=[dict(diameter=float(via.split('/')[0]), drill=float(via.split('/')[1]))]),
                  router=router, placer=dict(pad_spacing=0.15, border=0.2),
                  args=' '.join('%s=%s' % (k, v) for k, v in router.items()))
        if structure:
            from analysis.structure import typical
            RG['structure'] = typical(L)
        json.dump(RG, open(os.path.join(goal, 'design_rules.json'), 'w'), indent=1)
        nets_cons = {}
        for n, e in sorted(self.net_list.items()):
            if n not in nets or n == 'GND' or 'category' not in e: continue
            d = {'category': e['category']}
            for k_ in ('rank', 'f_Hz', 'data_rate', 'tr_ns', 'z_ohm', 'z_diff_ohm', 'length_group', 'tolerance_mm', 'interface'):
                if e.get(k_) is not None: d[k_] = e[k_]
            if e['category'] in ('current_val', 'supply'):
                a_ = e.get('current_A') or 0.3
                d['current_val'] = 'high' if a_ >= 2.0 else 'medium' if a_ >= 0.5 else 'small'
            nets_cons['^' + re.escape(n) + '$'] = d
        current_val = dict(LOAD)
        group_list = {}
        for n, e in sorted(self.net_list.items()):
            if n in nets and e.get('length_group'):
                g = group_list.setdefault(e['length_group'], dict(net_list=[], tolerance_mm=e.get('tolerance_mm'), interface=e.get('interface')))
                g['net_list'].append(n); g['tolerance_mm'] = min(g['tolerance_mm'] or 9e9, e.get('tolerance_mm') or 9e9)
        refs_present = {f['ref'] for f in fps}
        CONS = dict(net_list=nets_cons, gnd=['GND'], pinned=sorted(t['ref'] for t in self.part_list if t['pinned'] or t['ref'] in operate), current_val=current_val,
                  assignment={r: u for r, u in sorted(self.island_from.items()) if r in refs_present and u in refs_present},
                  tr_ns={'^' + re.escape(n) + '$': e['tr_ns'] for n, e in sorted(self.net_list.items()) if e.get('tr_ns') and n in nets},
                  length_groups=group_list,
                  voltage={'^' + re.escape(n) + '$': e['voltage'] for n, e in sorted(self.net_list.items()) if e.get('voltage') and n in nets},
                  fsw={r: f for r, f in self.fsw.items() if f}, family='task_%d' % (self.seed // 5))
        if self.power_board or self.potential:
            CONS.update(layer_stack=True, power_board=self.power_board)
        if structure: CONS['structure'] = RG['structure']
        json.dump(CONS, open(os.path.join(goal, 'specs.json'), 'w'), indent=1)
        if self.power_board:

            rail = sorted(n for n in nets if self.net_list.get(n, {}).get('category') in ('current_val', 'supply'))
            gnd = ['In1=GND'] + ['In%d=%s' % (i, '|'.join(rail)) for i in range(2, L - 1)]
        elif L >= 6 and self.potential:
            rail = sorted(n for n in self.rails if n in nets)
            gnd = (['In1=GND', 'In4=GND'] if L == 6 else ['In1=GND', 'In3=GND', 'In6=GND']) + ([('In2=' if L == 6 else 'In4=') + '|'.join(rail)] if rail else [])
        else:
            gnd = ['In1'] + (['In4'] if L == 6 else ['In3', 'In6'] if L == 8 else [])
        A_ = dict(seed=self.seed, version=version, level=self.level, rails_A={n: a for n, a in sorted(self.rails.items())}, cu_layers=L, gnd_layers=gnd, factor=round(self.factor, 3), width_mm=W, height_mm=H,
                  length_groups={g: len(v['net_list']) for g, v in sorted(group_list.items())},
                  area_mm2=round(W * H, 1), courtyard_mm2=round(surface, 1), part_list=len(self.part_list), pads=len(pads), net_list=len(nets),
                  blocks=self.blocks, nets_mine={n: e for n, e in sorted(self.net_list.items()) if n in nets})
        if self.version == BGA_MEASUREMENT_VERSION:
            from learning.bga_occupancy import statistics
            A_['bga_occupancy'] = {t['ref']: statistics([p for p in pads if p['ref'] == t['ref']])
                                  for t in self.part_list if 'BGA' in t['fp']}
        if self.power_board or self.potential:
            A_.update(layer_stack=True, power_board=self.power_board)
        if real:
            real_design_rules(goal, B, RG, A_)
        json.dump(A_, open(os.path.join(goal, 'task.json'), 'w'), indent=1, ensure_ascii=False)
        return A_


def level_axes(level, seed=1):
    rnd = random.Random(seed * 7919 + 17); st = max(0.0, float(level))
    L = rnd.choice([6, 8, 6, 8] if st < 2 else [4, 6, 6, 8] if st < 5 else [4, 4, 6, 6, 8] if st < 8 else [4, 4, 4, 6])
    return dict(factor=round(max(1.12, rnd.uniform(1.5, 1.7) - 0.045 * st), 2), cu_layers=L,
                active_pins=min(0.95, 0.30 + 0.065 * st),
                part_list=0, current_paths=0, clock=1.0, pairs=0, length_groups=0, analog=0.0)


def generate(goal, seed=1, cu_layers=0, factor=0.0, blocks='auto', level=0.0, structure=False, axes=None, family=None, potential=False, power_share=0.0, anchor_legal=False, package=None, generator_version=None, **parameter):
    if package not in (None, 'bga', 'qfn'): raise ValueError('package=bga|qfn')
    if package and (blocks != 'auto' or power_share):
        raise ValueError('Housing axis needs auto with processor (power_share=0)')
    rnd = random.Random(seed * 7919 + 17); st = max(0.0, float(level))
    L = cu_layers or rnd.choice([6, 8, 6, 8] if st < 2 else [4, 6, 6, 8] if st < 5 else [4, 4, 6, 6, 8] if st < 8 else [4, 4, 4, 6])
    f = factor or round(max(1.12, rnd.uniform(1.5, 1.7) - 0.045 * st), 2)
    new = axes is not None or bool(parameter) or family is not None
    ax = dict(axes or {}, **parameter)
    if new:
        from learning.teacher import axes_verify, family_id
        axes_verify(ax)

        fam = family or family_id(blocks)
        if fam.startswith('generator-v8:') and fam != family_id(blocks):
            raise ValueError('Block family does not match blueprint')
        L = int(ax.get('cu_layers', L)); f = float(ax.get('factor', f))
    a = Task(seed, L, f, st, ax if new else None, potential=potential, package=package, generator_version=generator_version); a.build(blocks, power_share=power_share)
    if new: a.axes_complete()
    A = a.write_out(goal, structure=structure, anchor_legal=anchor_legal)
    if new:
        from learning.teacher import task_features, plausible
        A.update(axes={**level_axes(st, seed), 'ampere': max(A['rails_A'].values(), default=0), **ax, 'cu_layers': L, 'factor': f}, family=fam,
                 lineage=dict(output_circuit=seed, family=fam), version=a.version + ('+anchor-legal1' if anchor_legal else '') + '+axes1')
        B = json.load(open(os.path.join(goal, 'board.json')))
        A['mcu_ref'] = getattr(a, 'mcu_ref', None)
        A['features_task'] = task_features(A, B)
        if A['part_list'] < ax.get('part_list', 0) or A['features_task']['analog_actual'] + 1e-12 < ax.get('analog', 0):
            raise ValueError('Final artifact falls below parts/analog minimum')
        A['plausibility'] = plausible(A, B)
        A['feasibility'] = 'unknown'
        if A['plausibility']: raise ValueError('Implausible task: ' + '; '.join(A['plausibility']))
        cons = json.load(open(os.path.join(goal, 'specs.json'))); cons['family'] = fam
        json.dump(cons, open(os.path.join(goal, 'specs.json'), 'w'), indent=1)
        json.dump(A, open(os.path.join(goal, 'task.json'), 'w'), indent=1, ensure_ascii=False)
    return A


def real_contour(w, h):
    r = min(3.0, w*.06, h*.06)
    edges, outer = [], []
    def line(a, b):
        edges.append(dict(type_='line', start=a, end=b)); outer.append(a)
    def arc(cx, cy, radius, a, b, target):
        def point(t): return [round(cx+radius*math.cos(t), 8), round(cy+radius*math.sin(t), 8)]
        edges.append(dict(type_='arc', start=point(a), mid=point((a+b)/2), end=point(b)))
        n = max(8, math.ceil(abs(b-a)/(.004/radius)**.5))
        target.extend(point(a+(b-a)*i/n) for i in range(n))
    line([r,0], [w-r,0]); arc(w-r,r,r,-math.pi/2,0,outer)
    line([w,r], [w,h-r]); arc(w-r,h-r,r,0,math.pi/2,outer)
    line([w-r,h], [r,h]); arc(r,h-r,r,math.pi/2,math.pi,outer)
    line([0,h-r], [0,r]); arc(r,r,r,math.pi,3*math.pi/2,outer)
    hole = []; radius = min(1.0,w*.018,h*.018)
    for i in range(4): arc(w*.10,h*.5,radius,i*math.pi/2,(i+1)*math.pi/2,hole)
    return [outer], [[hole]], edges


def real_design_rules(goal, board, design_rules, task):
    from pathlib import Path
    d = Path(goal); clr = design_rules['router']['drc']; sw = design_rules['router']['sigw']
    classes = dict(Default=dict(clr=clr,w=sw), Power=dict(clr=.22,w=.4),
                   Analog=dict(clr=.18,w=.14), USB=dict(clr=.13,w=.16))
    for vals in classes.values():
        vals.update(via_d=float(design_rules['router']['via'].split('/')[0]),
                    via_b=float(design_rules['router']['via'].split('/')[1]))
    assignment = {}
    for n in board['nets']:
        category = task['nets_mine'].get(n, {}).get('category')
        assignment[n] = ('USB' if n.startswith('USB_TEST_') else 'Power' if category in ('current_val','supply')
                         else 'Analog' if category == 'analog' else 'Default')
    board.update(netclass=assignment,net_classes=classes,
                 design=dict(clearance_min=clr,track_width_min=sw,copper_edge_clearance=.2))
    project = dict(meta=dict(filename='board.kicad_pro',version=3),
        board=dict(design_settings=dict(rules=dict(design_rules['raw_data']))),
        net_settings=dict(meta=dict(version=4),classes=[dict(name=n,clearance=v['clr'],track_width=v['w'],
            via_diameter=v['via_d'],via_drill=v['via_b'],diff_pair_width=.16,diff_pair_gap=.16) for n,v in classes.items()],
            netclass_patterns=[dict(pattern=n,netclass=c) for n,c in sorted(assignment.items())]))


    cons = json.loads((d/'specs.json').read_text())
    cons['anchor'] = [f['ref'] for f in board['fps'] if f['ref'].startswith('J') and not f['locked']]
    (d/'specs.json').write_text(json.dumps(cons,indent=1))

    project['board']['design_settings']['rules'] = {k:v for k,v in design_rules['raw_data'].items() if k.startswith('min_')}
    rules = project['board']['design_settings']['rules']
    vd, vb = classes['Default']['via_d'], classes['Default']['via_b']
    rules.update(min_via_diameter=vd,min_through_hole_diameter=vb,
                 min_via_annular_width=round((vd-vb)/2,6),
                 min_microvia_diameter=.2,min_microvia_drill=.1)
    dru = '''(version 1)
(rule "V11 Power Analog pair spacing"
 (condition "(A.NetClass == 'Power' && B.NetClass == 'Analog') || (A.NetClass == 'Analog' && B.NetClass == 'Power')")
 (constraint clearance (min 0.30mm)))
(rule "V11 Power F width"
 (condition "A.NetClass == 'Power' && A.Layer == 'F.Cu'")
 (constraint track_width (min 0.40mm)))
(rule "V11 Analog width"
 (condition "A.NetClass == 'Analog'") (constraint track_width (min 0.14mm)))
(rule "V11 USB name and layer width"
 (condition "A.NetName == 'USB_TEST_*' && A.Layer == 'F.Cu'")
 (constraint track_width (min 0.13mm) (max 0.20mm)))
(rule "V11 USB pair spacing"
 (condition "A.inDiffPair('USB_TEST')") (constraint diff_pair_gap (min 0.13mm) (max 0.25mm)))
(rule "V11 USB Skew"
 (condition "A.inDiffPair('USB_TEST')") (constraint skew (max 0.10mm)))
'''



    for fp in sorted(board['fps'],key=lambda q:q['ref']):
        ref = fp['ref']
        dru += ('(rule "V11 internal pad pair %s"\n'
                ' (condition "A.Type == \'Pad\' && B.Type == \'Pad\' && A.memberOfFootprint(\'%s\') && B.memberOfFootprint(\'%s\')")\n'
                ' (constraint clearance (min %.9gmm)))\n') % (ref,ref,ref,clr)
    zones = [dict(net='GND',layer='In1',pts=board['outline'][0],
                  holes=board['outline_holes'][0],full=True,prio=0,local_clearance=clr,min_thickness=.1)]
    board['native_zones'] = zones
    task['real_features'] = dict(version=1, netclasses=sorted(classes),
        locked_connector=[f['ref'] for f in board['fps'] if f['ref'].startswith('J') and f['locked']],
        loose_connector=[f['ref'] for f in board['fps'] if f['ref'].startswith('J') and not f['locked']],
        arcs=8,cutouts=1,usb_skew_mm=.1)
    (d/'board.json').write_text(json.dumps(board))
    (d/'board.kicad_pro').write_text(json.dumps(project,indent=1)+'\n')
    (d/'board.kicad_dru').write_text(dru)
    (d/'zones.json').write_text(json.dumps(zones))


def learning_curve_v11(goal):
    import hashlib
    from pathlib import Path
    root = Path(goal)/'learning_curve_v11'; root.mkdir(parents=True,exist_ok=True)
    manifest = dict(version='learning_curve-v11-1',generator_version=REAL_VERSION,tasks=[])
    for st in (0,2,4,6,8):
        for seed in (900001,900002,900003):
            name = 's%d_%d' % (st,seed); d = root/name
            if not d.exists(): generate(str(d),seed*10+st,level=st,generator_version=REAL_VERSION)
            a = json.loads((d/'task.json').read_text())
            if a['version'] != REAL_VERSION or a['seed'] != seed*10+st or a['level'] != st:
                raise ValueError('Foreign set content: '+str(d))
            files = ('board.json','design_rules.json','specs.json','task.json','board.kicad_pro','board.kicad_dru','zones.json')
            manifest['tasks'].append(dict(id=name,seed=a['seed'],level=st,
                sha256={f:hashlib.sha256((d/f).read_bytes()).hexdigest() for f in files}))
    text = json.dumps(manifest,indent=1)+'\n'; p=root/'dataset.json'
    reference = Path(R)/'codex/generator_real/learning_curve_v11.json'
    if reference.exists() and json.loads(reference.read_text()) != manifest:
        raise ValueError('V11 deviates from the versioned fixed set')
    if p.exists() and p.read_text()!=text: raise ValueError('Frozen set changed')
    p.write_text(text)
    return manifest


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    goal = sys.argv[1]
    if kw.get('dataset'):
        if kw['dataset'] != 'learning_curve_v11': raise ValueError('dataset=learning_curve_v11')
        learning_curve_v11(goal); return
    s0 = int(kw.get('seed', '1')); n = int(kw.get('n', '1'))
    for s in range(s0, s0 + n):
        d = os.path.join(goal, 'task_%06d' % s) if n > 1 or not kw.get('exact') else goal
        ax = json.loads(kw['axes']) if 'axes' in kw else None
        A = generate(d, s, int(kw.get('cu_layers', '0')), float(kw.get('factor', '0')), kw.get('blocks', 'auto'), float(kw.get('level', '0')), structure=kw.get('structure') == '1', axes=ax, anchor_legal=kw.get('anchor_legal') == '1', potential=kw.get('potential') == '1', generator_version=kw.get('generator_version'), power_share=float(kw.get('power_share', '0')))
        print('Level %.1f %s: %d layers, %.1f x %.1f mm (factor %.2f), %d parts, %d pads, %d nets | %s' % (
            A['level'], os.path.basename(d), A['cu_layers'], A['width_mm'], A['height_mm'], A['factor'], A['part_list'], A['pads'], A['net_list'], '; '.join(A['blocks'])))


if __name__ == '__main__':
    main()
