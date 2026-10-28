""
import functools, glob, json, math, os, random, re, sys

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
FP_DIR = os.environ.get('KICAD_FOOTPRINTS', '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints')
VERSION = '10'
ECHT_VERSION = '11'
BGA_MESSVERSION = '12'

class GeneratorFehler(ValueError):
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
def fp_laden(name):
    ""
    lib, n = name.split(':', 1)
    T = sexp(open(os.path.join(FP_DIR, lib + '.pretty', n + '.kicad_mod'), errors='replace').read())
    pads, crt = [], []
    for el in T:
        if not isinstance(el, list) or not el: continue
        if el[0] == 'pad':
            at = _f(el, 'at'); size = _f(el, 'size'); dr = _f(el, 'drill'); lay = _f(el, 'layers') or []
            rr = _f(el, 'roundrect_rratio')
            if not at or not size: continue
            d = 0.0; hole = []; drill_size = [0.0, 0.0]; copper_offset = [0.0, 0.0]
            if dr:
                offset = _f(dr,'offset')
                if offset: copper_offset = [float(offset[1]),float(offset[2])]
                zahlen = [x for x in dr[1:] if not isinstance(x, list) and re.match(r'^-?[\d.]+$', x)]
                d = float(zahlen[0]) if zahlen else 0.0
                if d > 0:


                    w = d; h = float(zahlen[1]) if len(zahlen)>1 else d
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
            pads.append(dict(num=el[1], typ=el[2], form=el[3], x=float(at[1]), y=float(at[2]),
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
    return dict(pads=pads, crt=(min(xs), min(ys), max(xs), max(ys)))


@functools.lru_cache(maxsize=None)
def _liste(lib):
    return tuple(sorted(os.path.basename(f)[:-10] for f in glob.glob(os.path.join(FP_DIR, lib + '.pretty', '*.kicad_mod'))))


def suche(rnd, lib, muster, ohne=r'HandSolder|ThermalVias|Handsoldering|_Pad\d'):
    ""
    for m in ([muster] if isinstance(muster, str) else muster):
        k = [n for n in _liste(lib) if re.match(m, n) and not re.search(ohne, n)]
        if k: return lib + ':' + rnd.choice(k)
    return None


def antennenseite(F):
    ""
    x0, y0, x1, y1 = F['crt']; ps = [p for p in F['pads'] if not p['npth']]
    px0 = min(p['x'] - max(p['w'], p['h']) / 2 for p in ps); px1 = max(p['x'] + max(p['w'], p['h']) / 2 for p in ps)
    py0 = min(p['y'] - max(p['w'], p['h']) / 2 for p in ps); py1 = max(p['y'] + max(p['w'], p['h']) / 2 for p in ps)
    rand = {'oben': py0 - y0, 'unten': y1 - py1, 'links': px0 - x0, 'rechts': x1 - px1}
    s = max(sorted(rand), key=lambda k: rand[k]); d = max(0.0, rand[s] - 0.3)
    rect = {'oben': (x0, y0, x1, y0 + d), 'unten': (x0, y1 - d, x1, y1), 'links': (x0, y0, x0 + d, y1), 'rechts': (x1 - d, y0, x1, y1)}[s]
    return s, rect


def pad_poly(p):
    ""
    w, h, f = p['w'], p['h'], p['form']
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





SCHNITTSTELLEN = {
    'usb_hs':  dict(f_Hz=240e6, datenrate=480e6, tr_ns=0.5, z_diff=90, tol_mm=0.15, rang=5),
    'ddr3':    dict(f_Hz=400e6, datenrate=800e6, tr_ns=0.3, z_se=40, z_diff=100, tol_mm=0.5, tol_adr_mm=2.5, rang=5),
    'sdram':   dict(f_Hz=166e6, datenrate=166e6, tr_ns=1.0, z_se=50, tol_mm=5.0, rang=4),
    'qspi':    dict(f_Hz=133e6, datenrate=133e6, tr_ns=1.0, z_se=50, tol_mm=2.5, rang=4),
    'spi':     dict(f_Hz=50e6, datenrate=50e6, tr_ns=2.0, z_se=50, rang=2),
    'uart':    dict(f_Hz=1.5e6, datenrate=3e6, tr_ns=10.0, rang=1),
    'i2c':     dict(f_Hz=400e3, datenrate=400e3, tr_ns=300.0, rang=1),
    'quarz':   dict(f_Hz=24e6, tr_ns=None, rang=3),
    'i2s':     dict(f_Hz=12.288e6, datenrate=12.288e6, tr_ns=2.0, z_se=50, tol_mm=25.0, rang=2),
    'mclk':    dict(f_Hz=24.576e6, datenrate=24.576e6, tr_ns=1.5, z_se=50, rang=3),
}

def schnell(art, gruppe=None, **mehr):
    ""
    s = SCHNITTSTELLEN[art]; d = dict(art='schnell' if s['rang'] >= 2 else 'signal', rang=s['rang'], f_Hz=s['f_Hz'], schnittstelle=art)
    if s.get('tr_ns'): d['tr_ns'] = s['tr_ns']
    if s.get('datenrate'): d['datenrate'] = s['datenrate']
    if s.get('z_se'): d['z_ohm'] = s['z_se']
    if gruppe: d['laengengruppe'] = gruppe; d['toleranz_mm'] = s.get('tol_mm')
    d.update(mehr); return d


class Aufgabe:
    def __init__(self, seed, lagen, faktor, stufe=0.0, achsen=None, potenzial=False, gehaeuse=None, generatorversion=None):
        self.rnd = random.Random(seed); self.seed = seed; self.lagen = lagen; self.faktor = faktor; self.stufe = float(stufe)
        self.potenzial = bool(potenzial) and lagen >= 6; self.leistungsboard = False
        self.gehaeuse = gehaeuse
        self.version = generatorversion or VERSION
        if self.version not in ('10', ECHT_VERSION, BGA_MESSVERSION): raise ValueError('generatorversion=10|11|12')
        self.achsen = achsen or {}
        self.vektormodus = achsen is not None
        self.teile = []
        self.netze = {}
        self.zaehler = {}; self.bloecke = []; self.strom_teil = {}; self.fsw = {}; self.sperren = []
        self.schienen = {}
        self.aktuell = None; self.insel_von = {}


    def ref(self, p):
        self.zaehler[p] = self.zaehler.get(p, 0) + 1; return '%s%d' % (p, self.zaehler[p])

    def netz(self, name, **eig):
        n = self.netze.setdefault(name, {}); n.update({k: v for k, v in eig.items() if v is not None}); return name

    def teil(self, p, fp, value, pins, typ=None, func=None, fest=None, strom=None):
        if fp is None: return None
        r = self.ref(p)
        self.teile.append(dict(ref=r, fp=fp, value=value, pins=dict(pins), typ=dict(typ or {}), func=dict(func or {}), fest=fest))



        if p == 'U' and not fest: self.aktuell = r
        elif p in ('C', 'R', 'L', 'D', 'Y', 'FB') and self.aktuell and not fest: self.insel_von[r] = self.aktuell
        if strom: self.strom_teil[r] = strom
        return r

    def zweipol(self, p, fp, value, a, b, typ='passive'):
        return self.teil(p, fp, value, {'1': a, '2': b}, typ={'1': typ, '2': typ})

    def klein(self):
        ""
        return '0201' if self.rnd.random() < min(0.85, 0.25 + 0.06 * self.stufe) else '0402'

    def c(self, a, b='GND', wert='100nF', groesse=None):
        g = groesse or self.klein()
        fp = {'0201': 'Capacitor_SMD:C_0201_0603Metric', '0402': 'Capacitor_SMD:C_0402_1005Metric', '0603': 'Capacitor_SMD:C_0603_1608Metric',
              '0805': 'Capacitor_SMD:C_0805_2012Metric', '1206': 'Capacitor_SMD:C_1206_3216Metric', '1210': 'Capacitor_SMD:C_1210_3225Metric'}[g]
        return self.zweipol('C', fp, wert, a, b)

    def r(self, a, b, wert='10k', groesse=None):
        g = groesse or self.klein()
        return self.zweipol('R', {'0201': 'Resistor_SMD:R_0201_0603Metric', '0402': 'Resistor_SMD:R_0402_1005Metric',
                                  '0603': 'Resistor_SMD:R_0603_1608Metric', '0805': 'Resistor_SMD:R_0805_2012Metric',
                                  '1206': 'Resistor_SMD:R_1206_3216Metric', '2512': 'Resistor_SMD:R_2512_6332Metric'}[g], wert, a, b)

    def ic(self, p, fp, value, rollen, strom=None):
        ""
        F = fp_laden(fp); zu = rollen(F['pads'])
        pins = {n: v[0] for n, v in zu.items()}; typ = {n: v[1] for n, v in zu.items()}; fn = {n: v[2] for n, v in zu.items()}
        return self.teil(p, fp, value, pins, typ, fn, strom=strom)


    @staticmethod
    def signalpads(pads):
        ""
        ps, seen = [], set()
        for p in pads:
            if p['npth'] or not p['num'] or p['num'] in seen: continue
            seen.add(p['num']); ps.append(p)
        ep = None
        if len(ps) > 6:
            g = max(ps, key=lambda p: p['w'] * p['h'])
            if g['w'] * g['h'] > 3 * sorted(p['w'] * p['h'] for p in ps)[len(ps) // 2] and abs(g['x']) < 0.6 and abs(g['y']) < 0.6: ep = g
        return [p for p in ps if p is not ep], ep

    def verteile(self, pads, vdd, signale, gnd='GND', vdd_anteil=0.12, kern=None, bga=False, versorgungen=None):
        ""
        ps, ep = self.signalpads(pads); zu = {}
        if ep: zu[ep['num']] = (gnd, 'power_in', 'EP')
        if bga and self.version == BGA_MESSVERSION:
            from lernen.bga_belegung import belegen
            zu.update(belegen(ps, signale, versorgungen or [vdd] + ([kern] if kern else []), gnd, self.rnd))
        elif bga:
            xs = sorted({round(p['x'], 2) for p in ps}); ys = sorted({round(p['y'], 2) for p in ps})
            def ring(p): return min(xs.index(round(p['x'], 2)), len(xs) - 1 - xs.index(round(p['x'], 2)),
                                    ys.index(round(p['y'], 2)), len(ys) - 1 - ys.index(round(p['y'], 2)))
            nring = max(ring(p) for p in ps) + 1
            for p in sorted(ps, key=lambda p: (ring(p), p['num'])):
                k = ring(p)
                if k >= nring - 2 and nring > 3:
                    zu[p['num']] = ((gnd, 'power_in', 'GND') if (xs.index(round(p['x'], 2)) + ys.index(round(p['y'], 2))) % 2 == 0 or not kern
                                    else (kern, 'power_in', 'VCORE'))
                elif k == nring - 3 and nring > 4:
                    zu[p['num']] = (vdd, 'power_in', 'VDD') if self.rnd.random() < 0.5 else (gnd, 'power_in', 'GND')
                elif signale and self.rnd.random() < 0.85: zu[p['num']] = signale.pop(0)
                elif self.rnd.random() < 0.15: zu[p['num']] = (gnd, 'power_in', 'GND')
        else:
            n = len(ps); k_vdd = max(1, int(n * vdd_anteil / 2))
            idx = [int(i * n / (k_vdd)) for i in range(k_vdd)]
            for i in idx:
                zu[ps[i]['num']] = (vdd, 'power_in', 'VDD')
                if i + 1 < n: zu[ps[i + 1]['num']] = (gnd, 'power_in', 'GND')
            if kern and n > 30:
                for j in range(max(2, k_vdd)):
                    zu[ps[(n // 2 + 3 * j) % n]['num']] = (kern, 'power_in', 'VCORE'); zu[ps[(n // 2 + 3 * j + 1) % n]['num']] = (gnd, 'power_in', 'GND')
            for p in ps:
                if p['num'] in zu: continue
                if signale and self.rnd.random() < 0.9: zu[p['num']] = signale.pop(0)
        return zu

    def entkoppeln(self, ref, je=1):
        ""
        t = next(t for t in self.teile if t['ref'] == ref)
        schienen = {}
        for num, net in t['pins'].items():
            if t['typ'].get(num) == 'power_in' and net != 'GND' and t['func'].get(num) in ('VDD', 'VCORE', 'VIN', 'AVDD', 'VCC'):
                schienen[net] = schienen.get(net, 0) + 1
                for _ in range(je): self.c(net, 'GND', self.rnd.choice(['100nF', '100nF', '10nF']), self.rnd.choice(['0201', '0402']))
        for net in sorted(schienen): self.c(net, 'GND', self.rnd.choice(['4.7uF', '10uF']), self.rnd.choice(['0402', '0603']))


    def stecker_eingang(self):
        self.aktuell = None
        art = self.rnd.choice(['usb', 'usb', 'jst', 'jst_gross'])
        if art == 'usb':
            vin = self.netz('VBUS', art='strom', spannung=5.0); fp = suche(self.rnd, 'Connector_USB', [r'USB_C_Receptacle_HRO_TYPE-C-31-M-12', r'USB_C_Receptacle_GCT_USB4105'])
            dp = self.netz('USB_DP', **schnell('usb_hs', 'USB', z_diff_ohm=90)); dm = self.netz('USB_DM', **schnell('usb_hs', 'USB', z_diff_ohm=90))
            F = fp_laden(fp); zu = {}
            for p in self.signalpads(F['pads'])[0]:
                n = p['num'].upper()
                zu[p['num']] = (('GND', 'power_in', 'GND') if n.startswith(('A1', 'B1', 'A12', 'B12', 'S')) and not n.startswith(('A10', 'B10', 'A11', 'B11')) else
                                (vin, 'power_out', 'VBUS') if n.startswith(('A4', 'B4', 'A9', 'B9')) else
                                (dp, 'bidirectional', 'DP') if n in ('A6', 'B6') else (dm, 'bidirectional', 'DM') if n in ('A7', 'B7') else
                                ('CC1', 'passive', 'CC') if n == 'A5' else ('CC2', 'passive', 'CC') if n == 'B5' else ('', '', ''))
            zu = {k: v for k, v in zu.items() if v[0]}
            j = self.teil('J', fp, 'USB-C', {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, fest='unten')
            self.r('CC1', 'GND', '5.1k'); self.r('CC2', 'GND', '5.1k')
            self.bloecke.append('USB-C 5 V'); return vin, 5.0, (dp, dm)
        v = self.rnd.choice([12.0, 12.0, 24.0, 9.0]) if art == 'jst' else self.rnd.choice([12.0, 19.0, 24.0])
        vin = self.netz('VIN', art='strom', spannung=v)
        fp = suche(self.rnd, 'Connector_JST', [r'JST_PH_S2B-PH-SM4', r'JST_PH_B2B-PH-K']) if art == 'jst' else \
            suche(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02'])
        self.teil('J', fp, 'VIN %g V' % v, {'1': vin, '2': 'GND'}, {'1': 'power_out', '2': 'power_in'}, fest='links')
        self.c(vin, 'GND', '10uF', '1206'); self.bloecke.append('Eingang %g V (%s)' % (v, art)); return vin, v, None
    def buck(self, vin, uin, uout, strom, name):
        ""
        out = self.netz(name, art='strom', spannung=uout, strom_A=strom); sw = self.netz('SW_' + name.strip('+'), art='sw', strom_A=strom)
        fb = self.netz('FB_' + name.strip('+'), art='analog', rang=2); bst = self.netz('BST_' + name.strip('+'), art='sw')
        self.netz(vin, strom_A=(self.netze.get(vin, {}).get('strom_A') or 0) + strom * uout / max(uin, 1) / 0.9)
        if strom <= 1.5: fp = suche(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-6$', r'TSOT-23-6'])
        elif strom <= 4: fp = suche(self.rnd, 'Package_DFN_QFN', [r'QFN-16-1EP_3x3mm_P0.5mm_EP1.7', r'DFN-8-1EP_3x3mm_P0.5mm'])
        else: fp = suche(self.rnd, 'Package_DFN_QFN', [r'QFN-20-1EP_4x4mm_P0.5mm_EP2.5', r'QFN-24-1EP_4x4mm_P0.5mm'])
        F = fp_laden(fp); ps, ep = self.signalpads(F['pads']); zu = {}



        ps = sorted(ps, key=lambda p: math.atan2(p['y'], p['x']))
        n = len(ps); k = max(1, (n - 4) // 3) if n > 7 else 1
        bloecke = [(vin, 'power_in', 'VIN')] * k + [(sw, 'power_out', 'SW')] * k + [('GND', 'power_in', 'GND')] * k
        rest = [(fb, 'input', 'FB'), (bst, 'passive', 'BST'), ('EN_' + name.strip('+'), 'input', 'EN'), ('PG_' + name.strip('+'), 'output', 'PG')]
        rollen = bloecke + rest + [('GND', 'power_in', 'GND')] * n
        for i, p in enumerate(ps): zu[p['num']] = rollen[i]
        if ep: zu[ep['num']] = ('GND', 'power_in', 'EP')
        fsw = self.rnd.choice([0.5e6, 1e6, 2.2e6])
        typ_ = (self.rnd.choice(['TPS62130', 'TPS54331', 'MP2307', 'LMR33630']) if strom <= 3 else 'TPS56637' if strom <= 6 else 'TPS568230' if strom <= 8
                else 'TPS56C215' if strom <= 12 else 'TPS548A28' if strom <= 15 else 'LTC3833')
        u = self.teil('U', fp, typ_, {k: v[0] for k, v in zu.items()},
                      {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()})
        self.fsw[u] = fsw
        lfp = (suche(self.rnd, 'Inductor_SMD', [r'L_0805_2012Metric$', r'L_1008_2520Metric$', r'L_1210_3225Metric$']) if strom <= 1.5 else
               suche(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRN4018', r'L_Coilcraft_XAL50[23]0', r'L_Bourns_SRN5040']) if strom <= 4 else
               suche(self.rnd, 'Inductor_SMD', [r'L_Coilcraft_XAL70[2-7]0', r'L_Bourns_SRP1038', r'L_Vishay_IHLP-4040', r'L_Bourns_SRN8040']))
        self.zweipol('L', lfp, self.rnd.choice(['1uH', '2.2uH', '4.7uH']), sw, out)
        g_in = '0805' if strom < 3 else '1206'; g_out = '0805' if strom < 3 else self.rnd.choice(['1206', '1210'])
        for _ in range(1 + int(strom > 2)): self.c(vin, 'GND', '10uF', g_in)
        self.c(vin, 'GND', '100nF', '0402')
        for _ in range(2 + int(strom > 3) + int(strom > 6)): self.c(out, 'GND', '22uF', g_out)
        self.c(bst, sw, '100nF', '0402')
        self.r(out, fb, '100k'); self.r(fb, 'GND', self.rnd.choice(['22k', '33k', '10k']))
        if self.leistungsboard:


            self.netze[fb].update(funktion='rueckkopplung', funktion_grund='Buck-Ausgangsteiler am FB-Pin von ' + u)
        self.r(vin, 'EN_' + name.strip('+'), '100k'); self.r(out, 'PG_' + name.strip('+'), '100k')
        self.strom_teil[u] = None; self.schienen[out] = strom
        self.bloecke.append('Buck %g→%g V %.1f A %s' % (uin, uout, strom, '%g MHz' % (fsw / 1e6)))
        return out

    def boost(self, vin, uin, uout, strom, name):
        out = self.netz(name, art='strom', spannung=uout, strom_A=strom); sw = self.netz('SW_' + name.strip('+'), art='sw', strom_A=strom * uout / uin)
        fb = self.netz('FB_' + name.strip('+'), art='analog', rang=2)
        self.netz(vin, strom_A=(self.netze.get(vin, {}).get('strom_A') or 0) + strom * uout / uin / 0.85)
        fp = suche(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-5$', r'SOT-23-6$'])
        rollen = [(sw, 'power_in', 'SW'), ('GND', 'power_in', 'GND'), (fb, 'input', 'FB'), ('EN_' + name.strip('+'), 'input', 'EN'), (vin, 'power_in', 'VIN'), ('', '', '')]
        F = fp_laden(fp); ps, _ = self.signalpads(F['pads']); zu = {p['num']: rollen[i] for i, p in enumerate(ps) if i < len(rollen) and rollen[i][0]}
        u = self.teil('U', fp, self.rnd.choice(['MT3608', 'TPS61040', 'AP3012']), {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()})
        self.fsw[u] = self.rnd.choice([1.2e6, 1e6])
        self.zweipol('L', suche(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRN4018', r'L_1210_3225Metric$', r'L_Coilcraft_XAL50[23]0']), '4.7uH', vin, sw)


        self.teil('D', suche(self.rnd, 'Diode_SMD', [r'D_SOD-123F', r'D_SOD-123$', r'D_SMA$']), 'SS34', {'1': out, '2': sw}, {'1': 'power_out', '2': 'passive'})
        self.c(vin, 'GND', '10uF', '0805'); self.c(out, 'GND', '22uF', '1206'); self.c(out, 'GND', '22uF', '1206')
        self.r(out, fb, '200k'); self.r(fb, 'GND', '10k'); self.r(vin, 'EN_' + name.strip('+'), '100k')
        if self.leistungsboard:
            self.netze[fb].update(funktion='rueckkopplung', funktion_grund='Boost-Ausgangsteiler am FB-Pin von ' + u)
        self.schienen[out] = strom; self.bloecke.append('Boost %g→%g V %.1f A' % (uin, uout, strom)); return out

    def ldo(self, vin, uout, name):
        out = self.netz(name, art='versorgung', spannung=uout, strom_A=0.2)
        fp = suche(self.rnd, 'Package_TO_SOT_SMD', [r'SOT-23-5$', r'SOT-223-3_TabPin2'])
        F = fp_laden(fp); ps, _ = self.signalpads(F['pads'])
        rollen = ([(vin, 'power_in', 'VIN'), ('GND', 'power_in', 'GND'), (vin, 'input', 'EN'), ('', '', ''), (out, 'power_out', 'VOUT')]
                  if len(ps) >= 5 else [('GND', 'power_in', 'GND'), (out, 'power_out', 'VOUT'), (vin, 'power_in', 'VIN'), (out, 'power_out', 'VOUT')])
        zu = {p['num']: rollen[i] for i, p in enumerate(ps) if i < len(rollen) and rollen[i][0]}
        self.teil('U', fp, 'LDO %g V' % uout, {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()})
        self.c(vin, 'GND', '1uF', '0402'); self.c(out, 'GND', '1uF', '0402'); self.schienen[out] = 0.2; self.bloecke.append('LDO %g V' % uout); return out

    def quarz(self, xi, xo):
        fp = suche(self.rnd, 'Crystal', [r'Crystal_SMD_2016-4Pin', r'Crystal_SMD_2520-4Pin', r'Crystal_SMD_3225-4Pin_3.2x2.5mm$'])
        self.teil('Y', fp, self.rnd.choice(['16MHz', '24MHz', '32MHz', '25MHz']), {'1': xi, '3': xo, '2': 'GND', '4': 'GND'},
                  {'1': 'passive', '3': 'passive', '2': 'passive', '4': 'passive'})
        g = self.rnd.choice(['0201', '0402']); self.c(xi, 'GND', '12pF', g); self.c(xo, 'GND', '12pF', g)
    BGA_STUFEN = [
        (0, [r'TFBGA-100_8x8mm', r'LFBGA-100_10x10mm', r'TFBGA-121_10x10mm', r'LFBGA-144_10x10mm']),
        (3, [r'TFBGA-169_9x9mm_Layout13x13_P0.65', r'TFBGA-216_13x13mm', r'TFBGA-256_13x13mm', r'TFBGA-225_10x10mm_Layout15x15_P0.65']),
        (6, [r'TFBGA-100_5.5x5.5mm', r'LFBGA-153_11.5x13mm_Layout14x14_P0.5', r'TFBGA-324_12x12mm_Layout18x18_P0.65', r'TFBGA-361_13x13mm']),
        (8, [r'TFBGA-289_9x9mm_Layout17x17_P0.5', r'BGA-153_8.0x8.0mm_Layout15x15_P0.5', r'BGA-256_11.0x11.0mm_Layout20x20_P0.5']),
        (10, [r'TFBGA-49_3x3mm_Layout7x7_P0.4', r'BGA-81_4.496x4.377mm_Layout9x9_P0.4']),
    ]

    def bus(self, art):
        ""
        N = []
        if art == 'ddr':
            for b in range(self.rnd.choice([1, 2])):
                g = 'DDR_BYTE%d' % b
                N += [self.netz('DDR_DQ%d' % (8 * b + i), **schnell('ddr3', g)) for i in range(8)]
                N += [self.netz('DDR_DM%d' % b, **schnell('ddr3', g))]
                N += [self.netz('DDR_DQS%d_%s' % (b, s), **schnell('ddr3', g, z_diff_ohm=100)) for s in 'PN']
            ga = 'DDR_ADR'; tol = SCHNITTSTELLEN['ddr3']['tol_adr_mm']
            N += [self.netz('DDR_CK_%s' % s, **schnell('ddr3', ga, z_diff_ohm=100, toleranz_mm=tol)) for s in 'PN']
            N += [self.netz('DDR_A%d' % i, **schnell('ddr3', ga, toleranz_mm=tol)) for i in range(self.rnd.choice([10, 13, 14]))]
            N += [self.netz('DDR_%s' % s, **schnell('ddr3', ga, toleranz_mm=tol)) for s in ('BA0', 'BA1', 'BA2', 'RAS', 'CAS', 'WE', 'CS', 'CKE', 'ODT')]
        elif art == 'sdram':
            N += [self.netz('SD_D%d' % i, **schnell('sdram', 'SDRAM')) for i in range(self.rnd.choice([8, 16]))]
            N += [self.netz('SD_A%d' % i, **schnell('sdram', 'SDRAM')) for i in range(12)]
            N += [self.netz('SD_%s' % s, **schnell('sdram', 'SDRAM')) for s in ('CLK', 'CKE', 'CS', 'RAS', 'CAS', 'WE', 'BA0', 'BA1')]
        else:
            N += [self.netz('QSPI_%s' % s, **schnell('qspi', 'QSPI')) for s in ('CLK', 'CS', 'IO0', 'IO1', 'IO2', 'IO3')]
        return N

    def prozessor(self, v33, v12, usb):
        st = self.stufe
        bga = self.rnd.random() < min(0.95, 0.45 + 0.06 * st)
        if self.gehaeuse is not None: bga = self.gehaeuse == 'bga'
        if bga:
            muster = [m for ab, ms in self.BGA_STUFEN if ab <= st + 1 for m in ms]
            fein = [m for ab, ms in self.BGA_STUFEN if st - 3 <= ab <= st + 1 for m in ms] or muster
            fp = suche(self.rnd, 'Package_BGA', [self.rnd.choice(fein if self.rnd.random() < 0.7 else muster)]) or \
                suche(self.rnd, 'Package_BGA', [r'TFBGA-100_8x8mm'])
        else:
            fp = suche(self.rnd, 'Package_DFN_QFN', [self.rnd.choice([r'QFN-48-1EP_7x7mm_P0.5mm_EP5.6', r'QFN-56-1EP_7x7mm_P0.4mm', r'QFN-64-1EP_9x9mm_P0.5mm_EP7.3'])]) \
                or suche(self.rnd, 'Package_QFP', [r'LQFP-64_10x10mm_P0.5mm$', r'LQFP-100_14x14mm_P0.5mm$'])
        if 'pitch' in self.achsen or 'bga' in self.achsen:
            bga = bool(self.achsen.get('bga', bga))
            pitch = self.achsen.get('pitch', 0.8 if bga else 0.5)
            lib = 'Package_BGA' if bga else 'Package_DFN_QFN'
            kandidaten = [n for n in _liste(lib) if re.search(r'_P%smm' % re.escape('%g' % pitch), n)
                          and 32 <= len(self.signalpads(fp_laden(lib + ':' + n)['pads'])[0]) <= 400]
            fp = lib + ':' + self.rnd.choice(kandidaten) if kandidaten else None
            if not fp: raise ValueError('Kein %s-Footprint mit Pitch %g' % (lib, pitch))
        n_pins = len(self.signalpads(fp_laden(fp)['pads'])[0])
        realistisch = bga and self.version == BGA_MESSVERSION
        if realistisch and n_pins < (121 if getattr(self, 'audio_an', False) else 100):





            pitch = re.search(r'_P([0-9.]+)mm', fp).group(1)
            minimum = 121 if getattr(self, 'audio_an', False) else 100
            kandidaten = [n for n in _liste('Package_BGA') if '_P' + pitch + 'mm' in n
                          and minimum <= len(self.signalpads(fp_laden('Package_BGA:' + n)['pads'])[0]) <= 224]
            if not kandidaten and 'pitch' not in self.achsen:
                verfuegbar = [(n,float(m.group(1))) for n in _liste('Package_BGA')
                              for m in [re.search(r'_P([0-9.]+)mm',n)] if m
                              and minimum <= len(self.signalpads(fp_laden('Package_BGA:' + n)['pads'])[0]) <= 224]
                if verfuegbar:
                    abstand = min(abs(p-float(pitch)) for _,p in verfuegbar)
                    kandidaten = [n for n,p in verfuegbar if abs(abs(p-float(pitch))-abstand)<1e-9]
            if not kandidaten: raise GeneratorFehler('Kein realistischer MCU-BGA mit Raster ' + pitch)
            fp = 'Package_BGA:' + self.rnd.choice(kandidaten)
            n_pins = len(self.signalpads(fp_laden(fp)['pads'])[0])
        sig = []
        self.netz('XTAL_IN', art='analog', rang=3, f_Hz=24e6); self.netz('XTAL_OUT', art='analog', rang=3, f_Hz=24e6)
        sig += [('XTAL_IN', 'input', 'XIN'), ('XTAL_OUT', 'output', 'XOUT'), ('NRST', 'input', 'NRST'), ('BOOT0', 'input', 'BOOT')]
        if usb: sig += [(usb[0], 'bidirectional', 'USB_DP'), (usb[1], 'bidirectional', 'USB_DM')]
        sig += [('I2C_SCL', 'bidirectional', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA')]
        for n in ('I2C_SCL', 'I2C_SDA'): self.netz(n, **schnell('i2c'))
        p_ddr = min(0.6, 0.05 + 0.07 * st) if bga else 0.0
        u_ = self.rnd.random()
        self.mem_art = 'qspi' if realistisch else 'ddr' if u_ < p_ddr else 'sdram' if u_ < p_ddr + 0.25 else 'qspi'
        self.speicher_netze = self.bus(self.mem_art)
        sig += [(n, 'bidirectional', n) for n in self.speicher_netze]
        self.spi = [self.netz('SPI_%s' % s, **schnell('spi')) for s in ('SCK', 'MOSI', 'MISO', 'CS')]
        sig += [(n, 'bidirectional', n) for n in self.spi]
        self.uart = [self.netz('UART_%s' % s, **schnell('uart')) for s in ('TX', 'RX')]
        sig += [(n, 'bidirectional', n) for n in self.uart]
        if getattr(self, 'audio_an', False): sig += [(n, 'bidirectional', n) for n in self.i2s_netze()]
        self.adc_in = [self.netz('ADC_IN%d' % i, art='analog', rang=3) for i in range(self.rnd.choice([1, 2, 4]))]
        sig += [(n, 'input', n) for n in self.adc_in]

        n_pins = len(self.signalpads(fp_laden(fp)['pads'])[0])
        anteil = self.achsen.get('aktive_pins', min(0.95, 0.30 + 0.065 * st))
        if realistisch:
            from lernen.bga_belegung import anteile
            signalbudget = anteile(n_pins, [65, 39, 32, 88])[0]


            if len(sig) > signalbudget:
                raise GeneratorFehler('BGA-Schnittstellen passen nicht in das realistische Signalbudget')
            signalbudget = max(len(sig), round(signalbudget * min(1., anteil / .30)))
            n_gpio = signalbudget - len(sig)
        else:
            n_gpio = max(4, int(n_pins * (0.75 if bga else 0.85) * anteil) - len(sig))
        self.gpio = [self.netz('GPIO%d' % i, art='signal', rang=0) for i in range(n_gpio)]
        sig += [(n, 'bidirectional', n) for n in self.gpio]
        rails = None
        if realistisch:
            rails = [v33, v12]
            for name in ('VIO_AUX', 'VANA_AUX', 'VREF_AUX'):
                rail = self.netz(name, art='versorgung', spannung=3.3)
                self.r(v33, rail, '0', '0402')
                rails.append(rail)
        zu = self.verteile(fp_laden(fp)['pads'], v33, sig, kern=v12, bga=bga, versorgungen=rails)
        u = self.teil('U', fp, self.rnd.choice(['STM32H7', 'i.MX RT', 'RP2350', 'ECP5', 'iCE40', 'nRF5340', 'ESP32-S3', 'i.MX 6ULL', 'AM335x']),
                      {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, strom=self.rnd.choice([0.15, 0.3, 0.6]))
        self.mcu_ref = u
        if rails:
            for t in self.teile:
                if t['ref'].startswith('R') and any(n in rails[2:] for n in t['pins'].values()): self.insel_von[t['ref']] = u
        self.entkoppeln(u); self.quarz('XTAL_IN', 'XTAL_OUT'); self.r(v33, 'NRST', '10k'); self.c('NRST', 'GND', '100nF'); self.r('BOOT0', 'GND', '10k')
        self.r(v33, 'I2C_SCL', '4.7k'); self.r(v33, 'I2C_SDA', '4.7k')
        for g in self.gpio[:self.rnd.randint(1, 4)]:
            m = self.netz('LED_' + g, art='signal'); self.r(g, m, '1k')
            self.teil('D', 'LED_SMD:LED_0402_1005Metric' if self.rnd.random() < 0.7 else 'LED_SMD:LED_0603_1608Metric', 'LED', {'1': 'GND', '2': m}, {'1': 'passive', '2': 'passive'})
        self.bloecke.append('Prozessor %s (%s)' % (fp.split(':')[1], 'BGA' if bga else 'QFN/QFP')); return u

    def speicher(self, v33):
        art = getattr(self, 'mem_art', 'qspi')
        if art == 'qspi' or not self.speicher_netze:
            fp = suche(self.rnd, 'Package_SON', [r'WSON-8-1EP_6x5mm', r'Texas_DSG0008A_WSON-8']) or suche(self.rnd, 'Package_SO', [r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            ns = (self.speicher_netze + self.spi)[:6]
            rollen = [(n, 'bidirectional', n) for n in ns] + [(v33, 'power_in', 'VCC'), ('GND', 'power_in', 'GND')]
            F = fp_laden(fp); ps, ep = self.signalpads(F['pads']); zu = {p['num']: rollen[i] for i, p in enumerate(ps) if i < len(rollen)}
            if ep: zu[ep['num']] = ('GND', 'power_in', 'EP')
            self.bloecke.append('QSPI-Flash 133 MHz')
        else:
            fp = suche(self.rnd, 'Package_SO', [r'TSOP-II-54']) if art == 'sdram' else suche(self.rnd, 'Package_BGA', [r'FBGA-96_', r'FBGA-78_'])
            sig = [(n, 'bidirectional', n) for n in self.speicher_netze]
            zu = self.verteile(fp_laden(fp)['pads'], v33, sig, bga=art == 'ddr', vdd_anteil=0.15)
            self.bloecke.append('SDRAM 166 MHz' if art == 'sdram' else 'DDR3-800 (BGA, %d Netze, Laengengruppen)' % len(self.speicher_netze))
        u = self.teil('U', fp, art.upper(), {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, strom=0.1)
        self.entkoppeln(u); return u

    def i2s_netze(self):
        ""
        self.i2s_art = 'tdm8' if self.rnd.random() < 0.5 else 'i2s_4x2'
        N = [self.netz('I2S_MCLK', **schnell('mclk')), self.netz('I2S_BCLK', **schnell('i2s', 'I2S')), self.netz('I2S_LRCLK', **schnell('i2s', 'I2S'))]
        daten = ['I2S_SDOUT', 'I2S_SDIN'] if self.i2s_art == 'tdm8' else ['I2S_SDOUT%d' % i for i in range(4)] + ['I2S_SDIN0']
        N += [self.netz(d, **schnell('i2s', 'I2S')) for d in daten]
        self.i2s = N; return N

    def audio(self, v33, va):
        ""
        if not getattr(self, 'i2s', None): self.i2s_netze()
        fp = suche(self.rnd, 'Package_QFP', [r'LQFP-48_7x7mm_P0.5mm$', r'LQFP-64_10x10mm_P0.5mm$']) if self.rnd.random() < 0.5 else \
            suche(self.rnd, 'Package_DFN_QFN', [r'QFN-40-1EP_6x6mm_P0.5mm_EP4.6x4.6mm$', r'QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm$'])
        kanaele = self.rnd.choice([4, 8, 8])
        aus = []
        for k in range(kanaele):
            for s in 'PN': aus.append(self.netz('AOUT%d_%s' % (k + 1, s), art='analog', rang=3, f_Hz=20e3))
        ein = [self.netz('AIN%d_%s' % (k + 1, s), art='analog', rang=3, f_Hz=20e3) for k in range(2) for s in 'LR']
        vcom = self.netz('AUD_VCOM', art='analog', rang=3); vq = self.netz('AUD_VREF', art='analog', rang=3)
        sig = [(n, 'bidirectional', n.split('_', 1)[1]) for n in self.i2s] + [('I2C_SCL', 'input', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA')] + \
              [(n, 'output', n) for n in aus] + [(n, 'input', n) for n in ein] + [(vcom, 'passive', 'VCOM'), (vq, 'passive', 'VREF')]
        for _ in range(3): sig.insert(self.rnd.randrange(len(sig)), (va, 'power_in', 'AVDD'))
        zu = self.verteile(fp_laden(fp)['pads'], v33, sig, vdd_anteil=0.08)
        u = self.teil('U', fp, self.rnd.choice(['CS42448', 'AK4458', 'TLV320AIC3104', 'PCM3168A', 'ADAU1966']),
                      {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, strom=0.08)
        self.entkoppeln(u); self.c(vcom, 'GND', '10uF', '0603'); self.c(vcom, 'GND', '100nF'); self.c(vq, 'GND', '10uF', '0603')
        buchsen = []
        for k in range(0, kanaele, 2):
            opv = suche(self.rnd, 'Package_SO', [r'MSOP-8_3x3mm_P0.65mm$', r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            o = ['AUD_OUT%d' % (k + 1), 'AUD_OUT%d' % (k + 2)]; m = ['AUD_FB%d' % (k + 1), 'AUD_FB%d' % (k + 2)]; pl = ['AUD_IP%d' % (k + 1), 'AUD_IP%d' % (k + 2)]
            for n in o + m + pl: self.netz(n, art='analog', rang=3, f_Hz=20e3)
            self.teil('U', opv, self.rnd.choice(['OPA1612', 'NE5532', 'OPA1678', 'LM4562']),
                      {'1': o[0], '2': m[0], '3': pl[0], '4': 'GND', '8': va, '7': o[1], '6': m[1], '5': pl[1]},
                      {'1': 'output', '2': 'input', '3': 'input', '4': 'power_in', '8': 'power_in', '7': 'output', '6': 'input', '5': 'input'},
                      {'1': 'OUT', '2': 'IN-', '3': 'IN+', '4': 'GND', '8': 'VCC', '7': 'OUT', '6': 'IN-', '5': 'IN+'}, strom=0.01)
            self.c(va, 'GND', '100nF')
            for j in range(2):
                self.r('AOUT%d_N' % (k + j + 1), m[j], '2.2k'); self.r(m[j], o[j], '2.2k'); self.c(m[j], o[j], '1nF')
                self.r('AOUT%d_P' % (k + j + 1), pl[j], '2.2k'); self.r(pl[j], vcom, '2.2k'); self.c(pl[j], 'GND', '1nF')
                lo = self.netz('LINE%d' % (k + j + 1), art='analog', rang=3, f_Hz=20e3); self.r(o[j], lo, '47'); self.c(lo, 'GND', '2.2nF')
            buchsen.append(['LINE%d' % (k + 1), 'LINE%d' % (k + 2)])
        for k, (a, b) in enumerate(buchsen[:2]):
            self.teil('J', suche(self.rnd, 'Connector_Audio', [r'Jack_3.5mm_CUI_SJ-3523-SMT_Horizontal', r'Jack_3.5mm_CUI_SJ1-3523N_Horizontal']),
                      'Line Out %d' % (k + 1), {'S': 'GND', 'T': a, 'R': b}, {'S': 'passive', 'T': 'passive', 'R': 'passive'}, fest='rechts')
        rest = [n for ab in buchsen[2:] for n in ab]
        if rest:
            fpc = suche(self.rnd, 'Connector_FFC-FPC', [r'Hirose_FH12-1[0-2]S-0.5SH_1x1[0-2]-1MP_P0.50mm_Horizontal', r'.*1x10.*P0.50mm']) or \
                suche(self.rnd, 'Connector_JST', [r'JST_SH_SM04B'])
            ps, _ = self.signalpads(fp_laden(fpc)['pads']); nz = rest + ['GND'] * len(ps)
            zu = {p['num']: nz[i] for i, p in enumerate(ps) if i < len(nz)}
            self.teil('J', fpc, 'Audio Out', zu, {k_: 'passive' for k_ in zu}, fest='unten')
        jin = suche(self.rnd, 'Connector_Audio', [r'Jack_3.5mm_CUI_SJ-3524-SMT_Horizontal', r'Jack_3.5mm_CUI_SJ1-3513N_Horizontal'])
        li = [self.netz('LINEIN_%s' % s, art='analog', rang=3, f_Hz=20e3) for s in 'LR']
        self.teil('J', jin, 'Line In', {'S': 'GND', 'T': li[0], 'R': li[1]}, {'S': 'passive', 'T': 'passive', 'R': 'passive'}, fest='links')
        for s, n in zip('LR', li):
            self.c(n, 'AIN1_%s' % s, '10uF', '0603'); self.r('AIN1_%s' % s, vcom, '10k'); self.c('AIN1_%s' % s, 'GND', '1nF')
        self.bloecke.append('Audio-Codec %d Kanaele an %s (%s), OPV-Puffer, Klinken' % (kanaele, 'TDM8' if self.i2s_art == 'tdm8' else 'I2S 4x2', fp.split(':')[1]))

    def erweiterung(self):
        ""
        real_bga = False
        kandidaten = list(getattr(self, 'gpio', []))
        if self.version == BGA_MESSVERSION and getattr(self, 'mcu_ref', None):
            mcu = next(t for t in self.teile if t['ref'] == self.mcu_ref)
            if 'BGA' in mcu['fp']:
                real_bga = True



                kandidaten = list(dict.fromkeys(kandidaten + [n for pin,n in mcu['pins'].items()
                                  if mcu['typ'].get(pin) != 'power_in']))
        offen = [g for g in kandidaten if sum(1 for t in self.teile for n in t['pins'].values() if n == g) < 2]
        while offen:
            k = min(len(offen), 40); bedarf = k + (k + 2) // 3 + 1
            pole = sorted(int(m.group(1)) for n in _liste('Connector_FFC-FPC') for m in [re.search(r'1x(\d+)-1MP_P0\.50mm_Horizontal', n)] if m)
            p_ = next((x for x in pole if x >= bedarf), None)
            if p_ is None: k = max(1, (pole[-1] * 3) // 4 - 1); p_ = pole[-1]
            nummer = ('%02d' % p_) if real_bga else str(p_)
            fp = suche(self.rnd, 'Connector_FFC-FPC', [r'Hirose_FH12-%dS-0.5SH_1x%s-1MP' % (p_, nummer), r'.*_1x%s-1MP_P0\.50mm_Horizontal' % nummer])
            if not fp: break
            ps, _ = self.signalpads(fp_laden(fp)['pads']); pins = {}; typ = {}; it = iter(offen[:k])
            for i, p in enumerate(ps):
                if i % 4 == 0: pins[p['num']] = 'GND'; typ[p['num']] = 'passive'; continue
                n = next(it, None)
                if n is None: break
                pins[p['num']] = n; typ[p['num']] = 'bidirectional'
            self.aktuell = None
            self.teil('J', fp, 'Erweiterung', pins, typ, fest=self.rnd.choice(['oben', 'unten', 'links', 'rechts']))
            offen = offen[k:]; self.bloecke.append('Erweiterungsstecker %d Pole' % p_)

    def sensoren(self, v33):
        for _ in range(self.rnd.randint(1, 3 + int(self.stufe / 3))):
            fp = suche(self.rnd, 'Package_LGA', [self.rnd.choice([r'Bosch_LGA-14_3x2.5mm', r'Bosch_LGA-8_2.5x2.5mm', r'LGA-12_2x2mm', r'LGA-14_3x2.5mm'])])
            ps, _ = self.signalpads(fp_laden(fp)['pads'])
            inten = self.netz('INT_%d' % len(self.bloecke), art='signal')
            rollen = [(v33, 'power_in', 'VDD'), ('GND', 'power_in', 'GND'), ('I2C_SCL', 'input', 'SCL'), ('I2C_SDA', 'bidirectional', 'SDA'), (inten, 'output', 'INT'),
                      ('GND', 'power_in', 'GND'), (v33, 'power_in', 'VDDIO')]
            zu = {p['num']: rollen[i] for i, p in enumerate(ps) if i < len(rollen)}
            u = self.teil('U', fp, self.rnd.choice(['BMI270', 'BMP390', 'SHT40', 'LIS2DH']), {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, strom=0.005)
            self.entkoppeln(u); self.bloecke.append('Sensor %s' % fp.split(':')[1])

    def analog(self, v33, va):
        ""
        if not getattr(self, 'adc_in', None): self.adc_in = [self.netz('ADC_IN0', art='analog', rang=3)]
        for i, ain in enumerate(self.adc_in[:2]):
            fp = suche(self.rnd, 'Package_SO', [r'MSOP-8_3x3mm_P0.65mm$', r'SOIC-8_3.9x4.9mm_P1.27mm$'])
            vin_p, fbn = self.netz('AIN%d_P' % i, art='analog', rang=3), self.netz('OPV%d_N' % i, art='analog', rang=3)
            outn = self.netz('OPV%d_OUT' % i, art='analog', rang=3)
            u = self.teil('U', fp, self.rnd.choice(['OPA2365', 'AD8605', 'MCP6022', 'TLV9062']),
                          {'1': outn, '2': fbn, '3': vin_p, '4': 'GND', '8': va, '5': 'OPV%d_B_P' % i, '6': 'OPV%d_B_N' % i, '7': 'OPV%d_B_OUT' % i},
                          {'1': 'output', '2': 'input', '3': 'input', '4': 'power_in', '8': 'power_in', '5': 'input', '6': 'input', '7': 'output'},
                          {'1': 'OUT', '2': 'IN-', '3': 'IN+', '4': 'GND', '8': 'VCC', '5': 'IN+', '6': 'IN-', '7': 'OUT'}, strom=0.002)
            self.netz('OPV%d_B_P' % i, art='analog', rang=3); self.netz('OPV%d_B_N' % i, art='analog', rang=3); self.netz('OPV%d_B_OUT' % i, art='analog', rang=3)
            self.r(outn, fbn, '10k'); self.r(fbn, 'GND', '1k'); self.c(outn, fbn, '100pF')
            self.r('OPV%d_B_OUT' % i, 'OPV%d_B_N' % i, '0'); self.r('OPV%d_B_P' % i, outn, '1k')
            self.r(vin_p, 'GND', '100k'); self.r('OPV%d_B_OUT' % i, ain, '47'); self.c(ain, 'GND', '1nF', '0402')
            self.c(va, 'GND', '100nF', '0402')
            self.teil('J', suche(self.rnd, 'Connector_JST', [r'JST_GH_SM02B', r'JST_SH_SM04B']), 'Sensor', {'1': vin_p, '2': 'GND'}, {'1': 'passive', '2': 'passive'}, fest='rechts')
        if self.rnd.random() < 0.6:
            fp = suche(self.rnd, 'Package_SO', [r'TSSOP-16_4.4x5mm', r'MSOP-10_3x3mm']) or suche(self.rnd, 'Package_DFN_QFN', [r'QFN-20-1EP_4x4mm_P0.5mm_EP2.5'])
            ps, ep = self.signalpads(fp_laden(fp)['pads'])
            sig = [(va, 'power_in', 'AVDD'), ('GND', 'power_in', 'GND'), (v33, 'power_in', 'VDD'), ('VREF', 'input', 'VREF')] + \
                  [(n, 'bidirectional', n) for n in self.spi] + [(self.netz('ADC_X%d' % k, art='analog', rang=3), 'input', 'AIN') for k in range(4)]
            zu = {p['num']: sig[i] for i, p in enumerate(ps) if i < len(sig)}
            if ep: zu[ep['num']] = ('GND', 'power_in', 'EP')
            self.netz('VREF', art='analog', rang=3)
            u = self.teil('U', fp, self.rnd.choice(['ADS8688', 'AD5686', 'MCP3208']), {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, strom=0.005)
            self.entkoppeln(u); self.c('VREF', 'GND', '10uF', '0603'); self.bloecke.append('ADC/DAC %s' % fp.split(':')[1])
        self.bloecke.append('Analog: %d OPV-Stufen' % min(2, len(self.adc_in)))

    def funk(self, v33):
        fp = suche(self.rnd, 'RF_Module', [r'Raytac_MDBT42Q', r'E73-2G4M04S', r'Laird_BL652', r'ESP32-C3-WROOM-02$', r'BLE112-A'])
        if not fp: return
        ps, ep = self.signalpads(fp_laden(fp)['pads'])
        sig = [(v33, 'power_in', 'VCC'), ('GND', 'power_in', 'GND')] + [(n, 'bidirectional', n) for n in getattr(self, 'uart', [])] + [('GND', 'power_in', 'GND')]
        zu = {p['num']: sig[i] for i, p in enumerate(ps) if i < len(sig)}
        if ep: zu[ep['num']] = ('GND', 'power_in', 'EP')
        seite, _ = antennenseite(fp_laden(fp))
        u = self.teil('U', fp, 'Bluetooth', {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()}, fest=seite, strom=0.03)
        self.c(v33, 'GND', '10uF', '0603'); self.c(v33, 'GND', '100nF', '0402'); self.bloecke.append('Bluetooth-Modul %s' % fp.split(':')[1])
        self.funk_ref = u

    def netzteil(self):
        ""
        hv = self.netz('HV_DC', art='strom', spannung=325.0, strom_A=0.05); sw = self.netz('SW_PRI', art='sw'); sek = self.netz('SEK', art='sw')
        v5 = self.netz('+5V_NT', art='strom', spannung=5.0, strom_A=2.0)
        self.teil('J', suche(self.rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal', r'TerminalBlock_Phoenix_PT-1,5-2']) or
                  suche(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02']), 'AC', {'1': 'AC_L', '2': 'AC_N'}, {'1': 'passive', '2': 'passive'}, fest='links')
        self.netz('AC_L', art='strom', strom_A=0.1); self.netz('AC_N', art='strom', strom_A=0.1)
        self.teil('D', suche(self.rnd, 'Diode_SMD', [r'Diode_Bridge_Vishay_MBLS', r'Diode_Bridge']) or suche(self.rnd, 'Package_TO_SOT_SMD', [r'TO-269AA']) or
                  'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', 'MB6S', {'1': hv, '2': 'GND', '3': 'AC_L', '4': 'AC_N'}, {'1': 'passive', '2': 'passive', '3': 'passive', '4': 'passive'})
        self.teil('C', suche(self.rnd, 'Capacitor_SMD', [r'CP_Elec_8x10.5', r'CP_Elec_6.3x5.4$']), '10uF 400V', {'1': hv, '2': 'GND'}, {'1': 'passive', '2': 'passive'})
        u = self.teil('U', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', 'LNK306', {'1': 'GND', '2': 'GND', '4': 'FB_NT', '5': sw, '7': sw, '8': 'BP'},
                      {'1': 'power_in', '2': 'power_in', '4': 'input', '5': 'power_out', '7': 'power_out', '8': 'passive'})
        self.fsw[u] = 66e3; self.netz('FB_NT', art='analog', rang=2)
        tr = suche(self.rnd, 'Transformer_SMD', [r'Transformer_Coilcraft_', r'Pulse_']) or suche(self.rnd, 'Inductor_SMD', [r'L_Bourns_SRP1038', r'L_Bourns_SRN8040'])
        tf = fp_laden(tr); ps, _ = self.signalpads(tf['pads']); rollen = [(hv, 'passive', 'P1'), (sw, 'passive', 'P2'), (sek, 'passive', 'S1'), ('GND', 'passive', 'S2')]
        zu = {p['num']: rollen[i] for i, p in enumerate(ps) if i < len(rollen)}
        self.teil('T', tr, 'Trafo', {k: v[0] for k, v in zu.items()}, {k: v[1] for k, v in zu.items()}, {k: v[2] for k, v in zu.items()})
        self.teil('D', suche(self.rnd, 'Diode_SMD', [r'D_SMA$', r'D_SMB$', r'D_SOD-123F']), 'SS34', {'1': v5, '2': sek}, {'1': 'passive', '2': 'passive'})
        self.c('BP', 'GND', '100nF', '0603'); self.c(v5, 'GND', '220uF', '1210'); self.c(v5, 'GND', '22uF', '1206')
        self.r(v5, 'FB_NT', '10k', '0603'); self.r('FB_NT', 'GND', '2.2k', '0603')
        self.bloecke.append('Mini-Netzteil (Sperrwandler-Archetyp) 325 V → 5 V'); return v5, 5.0


    def bauen(self, art, leistungsanteil=0.0):
        rnd = self.rnd
        self.leistungsboard = art == 'leistung' or (art == 'auto' and random.Random(self.seed * 104729 + 19).random() < float(leistungsanteil))
        if self.leistungsboard:
            uin = rnd.choice([12.0, 24.0]); vin = self.netz('VIN', art='strom', spannung=uin)
            fp = suche(rnd, 'Connector_AMASS', [r'AMASS_XT60-M_1x02']) or suche(rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal'])
            self.teil('J', fp, 'Leistungseingang %g V' % uin, {'1': vin, '2': 'GND'}, {'1': 'power_out', '2': 'power_in'}, fest='links')
            self.c(vin, 'GND', '22uF', '1210')
            for u, a in ((5.0, 8.0), (3.3, 6.0), (1.2, 10.0)):
                self.buck(vin, uin, u, a * (1 + .1 * self.stufe), '+%dV%s' % (int(u), str(round((u - int(u)) * 10)) if u != int(u) else ''))
            self.netz('GND', art='gnd'); self.bloecke.append('Leistungsboard: zwei Signallagen, Rest Leistung'); return
        if art == 'netzteil' or (art == 'auto' and rnd.random() < 0.12):
            vin, uin = self.netzteil(); usb = None
        else:
            vin, uin, usb = self.stecker_eingang()
        ratio = []
        hoch = 1.0 + 0.1 * self.stufe
        def stromwert(wert): return self.achsen.get('ampere', wert)
        v33 = self.buck(vin, uin, 3.3, stromwert(round(rnd.choice([1.0, 2.0, 3.0]) * hoch, 1)), '+3V3') if uin > 4 else self.ldo(vin, 3.3, '+3V3')
        v12 = self.buck(vin, uin, 1.2, stromwert(round(rnd.choice([1.5, 3.0, 6.0, 10.0]) * hoch, 1)), '+1V2') if rnd.random() < 0.7 else self.ldo(v33, 1.2, '+1V2')
        if rnd.random() < 0.4 + 0.03 * self.stufe and uin >= 9: self.buck(vin, uin, 5.0, stromwert(round(rnd.choice([3.0, 5.0, 8.0]) * hoch, 1)), '+5V')
        if rnd.random() < 0.3 + 0.03 * self.stufe: self.boost(v33, 3.3, 12.0, min(.3, self.achsen['ampere']) if 'ampere' in self.achsen else round(0.3 * hoch, 2), '+12V_B')
        if self.stufe >= 3 and rnd.random() < 0.08 * self.stufe and uin > 4:
            self.buck(vin, uin, 1.8, stromwert(round(rnd.choice([1.0, 2.0, 4.0]) * hoch, 1)), '+1V8')
        va = self.ldo(v33, 3.0, '+3V0A') if rnd.random() < 0.6 else v33
        if va != v33: self.netz(va, art='versorgung', spannung=3.0)
        self.audio_an = art == 'audio' or (art in ('auto', 'mainboard', 'analog') and rnd.random() < min(0.5, 0.1 + 0.05 * self.stufe))
        if self.vektormodus: self.audio_an = art == 'audio'
        if self.audio_an and va == v33: va = self.ldo(v33, 3.0, '+3V0A')
        if art in ('auto', 'mainboard', 'funk', 'analog', 'audio'):
            self.prozessor(v33, v12, usb)
            if art != 'analog' or rnd.random() < 0.5: self.speicher(v33)
            self.sensoren(v33)
        if art in ('auto', 'analog', 'mainboard') and (art == 'analog' or rnd.random() < 0.6): self.analog(v33, va)
        if art in ('auto', 'funk', 'mainboard') and (art == 'funk' or rnd.random() < 0.5): self.funk(v33)
        if self.audio_an: self.audio(v33, va)
        self.erweiterung()
        self.netz('GND', art='gnd')

    def achsen_ergaenzen(self):
        ""
        a = self.achsen
        if not a: return
        vin = next(n for n, e in self.netze.items() if e.get('spannung', 0) >= 4)
        uin = self.netze[vin]['spannung']
        for i in range(max(0, int(a.get('strompfade', 0)) - len(self.schienen))):
            self.buck(vin, uin, 1.8, a.get('ampere', 1.0), '+LEHRER%d' % i)

        def rc(i):
            n = self.netz('LEHRER_A%d' % i, art='analog', rang=3)
            self.r(vin, n, '100k', '0402'); self.c(n, 'GND', '1nF', '0402')
            self.bloecke.append('RC-Messkette')
        for i in range(int(a.get('paare', 0))):
            nets = [self.netz('LEHRER_DIFF%d_%s' % (i, s), **schnell('usb_hs', 'LEHRER_PAIR%d' % i, z_diff_ohm=90)) for s in ('P', 'N')]
            for _ in range(2):
                self.teil('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical', 'Differential-Port',
                          {'1': nets[0], '2': nets[1], '3': 'GND'}, {'1': 'bidirectional', '2': 'bidirectional', '3': 'power_in'})
        for i in range(int(a.get('laengengruppen', 0))):
            n = self.netz('LEHRER_CLK%d' % i, **schnell('qspi', 'LEHRER_CLOCK%d' % i))
            m = self.netz('LEHRER_DATA%d' % i, **schnell('qspi', 'LEHRER_CLOCK%d' % i))
            for _ in range(2):
                self.teil('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical', 'Clock-Port',
                          {'1': n, '2': m, '3': 'GND'}, {'1': 'bidirectional', '2': 'bidirectional', '3': 'power_in'})
        i = 0
        while (len(self.teile) < a.get('teile', 0) or
               sum(e.get('art') == 'analog' for e in self.netze.values()) / max(1, len(self.netze)) < a.get('analog', 0)):
            rc(i); i += 1
            if i > 200: raise ValueError('Analog/Teile-Ziel nicht erreichbar')
        for e in self.netze.values():
            if e.get('schnittstelle') and e.get('f_Hz'):
                e['f_Hz'] *= a.get('takt', 1.0)
                if e.get('datenrate'): e['datenrate'] *= a.get('takt', 1.0)
                if e.get('tr_ns'): e['tr_ns'] /= a.get('takt', 1.0)


    def lasten(self):
        ""
        last = {}
        for n, a in sorted(self.schienen.items()):
            ver = [(t['ref'], num) for t in self.teile for num, net in sorted(t['pins'].items())
                   if net == n and t['typ'].get(num) == 'power_in' and t['ref'][0] in 'UJ']
            if not ver:
                fp = (suche(self.rnd, 'Connector_AMASS', [r'AMASS_XT60-M_1x02', r'AMASS_XT60']) if a > 9 else None) or \
                     (suche(self.rnd, 'TerminalBlock_Phoenix', [r'TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal', r'TerminalBlock_Phoenix_PT-1,5-2']) if a > 2.5 else None) or \
                     (suche(self.rnd, 'Connector_JST', [r'JST_XH_B2B-XH-A_1x02']) if a > 1.5 else None) or \
                     suche(self.rnd, 'Connector_JST', [r'JST_PH_S2B-PH-SM4', r'JST_PH_B2B-PH-K_1x02'])
                j = self.teil('J', fp, 'Ausgang %s' % n, {'1': n, '2': 'GND'}, {'1': 'power_in', '2': 'power_in'}, fest='rechts')
                ver = [(j, '1')]; self.bloecke.append('Ausgang %s %.1f A' % (n, a))
            for ref, num in ver:
                k_ = ref if ref.startswith('J') else '%s:%s' % (ref, num)
                last[k_] = round(last.get(k_, 0) + a / len(ver), 4)
        return last

    def schreiben(self, ziel, aufbau=False, anker_legal=False):
        rnd = self.rnd; os.makedirs(ziel, exist_ok=True); L = self.lagen
        LAST = self.lasten()
        echt = self.version == ECHT_VERSION
        if echt:

            for suffix in ('P', 'N'):
                self.netz('USB_TEST_' + suffix, **schnell('usb_hs', 'USB_TEST', z_diff_ohm=90, toleranz_mm=.10))
            for fest in ('oben', None):
                self.teil('J', 'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical',
                          'USB-Testport', {'1': 'USB_TEST_P', '2': 'USB_TEST_N', '3': 'GND'}, fest=fest)
        namen = ['F'] + ['In%d' % i for i in range(1, L - 1)] + ['B']

        zahl = {}
        for t in self.teile:
            for n in t['pins'].values():
                if n: zahl[n] = zahl.get(n, 0) + 1
        for t in self.teile:
            t['pins'] = {k: (v if zahl.get(v, 0) >= 2 else '') for k, v in t['pins'].items()}
        flaeche = 0.0; F = {}
        for t in self.teile:
            F[t['ref']] = fp_laden(t['fp']); x0, y0, x1, y1 = F[t['ref']]['crt']; flaeche += (x1 - x0) * (y1 - y0)
        A = flaeche * self.faktor; seit = rnd.uniform(1.0, 1.7); W = math.sqrt(A * seit); H = A / W
        if echt:


            W, H = W*1.20, H*1.20
        W, H = round(W, 1), round(H, 1)
        outline = [[(0.0, 0.0), (W, 0.0), (W, H), (0.0, H)]]
        pads, fps, crt, keep = [], [], [], []
        if echt:
            outline, holes, edges = echt_umriss(W, H)
            from shapely.geometry import Polygon, box
            board_poly = Polygon(outline[0], holes[0])


        bedien = set()
        if anker_legal or echt:
            from analyse.klassen import BEDIEN_HINT
            bedien = {t['ref'] for t in self.teile if BEDIEN_HINT.search(t['fp'] + ' ' + t['value'])}
        if echt: bedien = {r for r in bedien if not r.startswith('J')}
        belegt = []

        def frei(X, Y, c):
            x0, y0, x1, y1 = c
            r = (round(X, 3) + x0 - 0.3, round(Y, 3) + y0 - 0.3,
                 round(X, 3) + x1 + 0.3, round(Y, 3) + y1 + 0.3)
            if echt and not board_poly.covers(box(X+x0, Y+y0, X+x1, Y+y1)):
                return False
            if all(r[2] <= b[0] or b[2] <= r[0] or r[3] <= b[1] or b[3] <= r[1] for b in belegt):
                belegt.append(r); return True
            return False

        def rand_platz(t, x0, y0, x1, y1):
            w, h = x1 - x0, y1 - y0


            for s in [t['fest']] + [s for s in ('oben', 'unten', 'links', 'rechts') if s != t['fest']]:
                if s in ('oben', 'unten'):
                    n_ = max(0, int((W - w) / 0.5))
                    kand = [(-x0 + k * 0.5, -y0 if s == 'oben' else H - y1) for k in range(n_ + 1)]
                else:
                    n_ = max(0, int((H - h) / 0.5))
                    kand = [(-x0 if s == 'links' else W - x1, -y0 + k * 0.5) for k in range(n_ + 1)]
                rnd.shuffle(kand) if s != 'unten' else kand.sort(key=lambda p: abs(p[0] + (x0 + x1) / 2 - W / 2))
                for X, Y in kand:
                    if X + x1 > W + 1e-6 or Y + y1 > H + 1e-6 or X + x0 < -1e-6 or Y + y0 < -1e-6: continue
                    if frei(X, Y, (x0, y0, x1, y1)):
                        t['fest'] = s; return X, Y
            raise GeneratorFehler('Aufgabe nicht erzeugbar: kein freier Randplatz fuer ' + t['ref'])

        def bedien_platz(c):
            x0, y0, x1, y1 = c
            kand = [(-x0 + i * 0.5, -y0 + j * 0.5)
                    for i in range(max(0, int((W - x1 + x0) / 0.5)) + 1)
                    for j in range(max(0, int((H - y1 + y0) / 0.5)) + 1)]
            rnd.shuffle(kand)
            for X, Y in kand:
                if X + x1 <= W and Y + y1 <= H and frei(X, Y, c): return X, Y
            raise GeneratorFehler('Aufgabe nicht erzeugbar: kein freier Platz fuer Bedienteil')

        def rand_platz_alt(t, x0, y0, x1, y1):
            w, h = x1 - x0, y1 - y0; s = t['fest']
            if s in ('oben', 'unten'): n_ = max(1, int((W - w) / 0.5)); kand = [(-x0 + k * 0.5, -y0 if s == 'oben' else H - y1) for k in range(n_ + 1)]
            else: n_ = max(1, int((H - h) / 0.5)); kand = [(-x0 if s == 'links' else W - x1, -y0 + k * 0.5) for k in range(n_ + 1)]
            rnd.shuffle(kand) if s != 'unten' else kand.sort(key=lambda p: abs(p[0] + (x0 + x1) / 2 - W / 2))
            for X, Y in kand:
                r = (X + x0 - 0.3, Y + y0 - 0.3, X + x1 + 0.3, Y + y1 + 0.3)
                if X + x1 > W + 1e-6 or Y + y1 > H + 1e-6 or X + x0 < -1e-6 or Y + y0 < -1e-6: continue
                if all(r[2] <= b[0] or b[2] <= r[0] or r[3] <= b[1] or b[3] <= r[1] for b in belegt):
                    belegt.append(r); return X, Y
            t['fest'] = None
            return rnd.uniform(-x0, max(-x0, W - x1)), rnd.uniform(-y0, max(-y0, H - y1))

        def reihenfolge(t):
            area = -(lambda c: (c[2] - c[0]) * (c[3] - c[1]))(F[t['ref']]['crt'])
            if echt: return (not bool(t['fest']), not t['ref'].startswith('J'), t['ref'] not in bedien, area)
            return (not bool(t['fest']), t['ref'] not in bedien, area) if anker_legal else area

        rand_suche = rand_platz if anker_legal or echt else rand_platz_alt
        for t in sorted(self.teile, key=reihenfolge):
            x0, y0, x1, y1 = F[t['ref']]['crt']
            if t['fest']: X, Y = rand_suche(t, x0, y0, x1, y1)
            elif echt and t['ref'].startswith('J'):
                t['fest'] = 'unten'
                X, Y = rand_platz(t, x0,y0,x1,y1)
                t['fest'] = None
            elif t['ref'] in bedien: X, Y = bedien_platz((x0, y0, x1, y1))
            elif echt:


                for _ in range(20000):
                    X, Y = rnd.uniform(-x0, W-x1), rnd.uniform(-y0, H-y1)
                    if frei(X, Y, (x0,y0,x1,y1)):
                        belegt.pop(); break
                else: raise GeneratorFehler('Kein steckerfreier Startplatz: %s board=%s teil=%s anker=%s' % (t['ref'],(W,H),(x0,y0,x1,y1),belegt))
            else: X, Y = rnd.uniform(-x0, max(-x0, W - x1)), rnd.uniform(-y0, max(-y0, H - y1))
            X, Y = round(X, 3), round(Y, 3)
            tht = any(p['tht'] for p in F[t['ref']]['pads'])
            fps.append(dict(ref=t['ref'], x=X, y=Y, rot=0.0, side='F', fp=t['fp'].split(':')[1], lib=t['fp'].split(':')[0], value=t['value'],
                            sheet='', locked=bool(t['fest'] or t['ref'] in bedien), attr='through_hole' if tht else 'smd', dnp=False, board_only=False,
                            n_pads=len(F[t['ref']]['pads'])))
            crt.append(dict(ref=t['ref'], side='F', polys=[[(X + x0, Y + y0), (X + x1, Y + y0), (X + x1, Y + y1), (X + x0, Y + y1)]]))
            for p in F[t['ref']]['pads']:
                lays = namen if p['tht'] else ['F']
                poly = [(round(X + u, 5), round(Y + v, 5)) for u, v in pad_poly(p)]
                xs, ys = [q[0] for q in poly], [q[1] for q in poly]
                net = '' if p['npth'] else t['pins'].get(p['num'], '')
                pads.append(dict(ref=t['ref'], num=p['num'], net=net, layers=lays, polys={Lx: [poly] for Lx in lays}, maske={},
                                 x=round(X + p['x'], 5), y=round(Y + p['y'], 5), bbox=[min(xs), min(ys), max(xs), max(ys)],
                                 drill=p['drill'] if p['tht'] else 0.0,
                                 **(dict(drill_size=p['drill_size'], hole_polys=[[(round(X+u,5),round(Y+v,5)) for u,v in q] for q in p['hole_polys']]) if p['tht'] else {}),
                                 fp_side='F', pinfunc=t['func'].get(p['num'], ''),
                                 pintype=t['typ'].get(p['num'], '') if net else ''))
            if t['ref'] == getattr(self, 'funk_ref', None):
                _, (a0, b0, a1, b1) = antennenseite(F[t['ref']])
                if a1 - a0 > 0.5 and b1 - b0 > 0.5:
                    keep.append(dict(ref=t['ref'], name='antenna keepout', polys=[[(X + a0, Y + b0), (X + a1, Y + b0), (X + a1, Y + b1), (X + a0, Y + b1)]],
                                     lagen=namen, verbote=dict(bahnen=True, vias=True, flaechen=True, pads=False, teile=True)))
        if self.version == BGA_MESSVERSION:
            from lernen.bga_belegung import zu_zielen
            bgarefs = {t['ref'] for t in self.teile if 'BGA' in t['fp']}
            for p in pads: p['_bga'] = p['ref'] in bgarefs
            zu_zielen(pads)
        anz = {}
        for p in pads:
            if p['net']: anz[p['net']] = anz.get(p['net'], 0) + 1
        for p in pads:
            if p['net'] and anz[p['net']] < 2: p['net'] = ''; p['pintype'] = ''
        nets = sorted({p['net'] for p in pads if p['net']})
        version = self.version + ('+anker-legal1' if anker_legal else '') + ('+r35-aufbau1' if aufbau else '')
        B = dict(src='aufgabe seed=%d (lernen/aufgaben.py v%s)' % (self.seed, version), pads=pads, outline=outline, courtyards=crt, keepouts=keep,
                 fps=fps, via_sperre=[], nets=nets, netclass={n: 'Default' for n in nets}, netzklassen={}, design={}, kupfer_grafik=[], lagen=L)
        if echt:
            B.update(outline_holes=holes, edge_primitives=edges)
        json.dump(B, open(os.path.join(ziel, 'board.json'), 'w'))

        fein = min([abs(a['x'] - b['x']) for t in self.teile if 'BGA' in t['fp'] for a, b in [sorted(F[t['ref']]['pads'], key=lambda p: (p['y'], p['x']))[:2]]] or [1.0])
        via = '0.30/0.15' if fein < 0.6 else '0.40/0.20' if fein < 0.75 else '0.45/0.20'
        clr = 0.075 if fein < 0.6 else 0.09


        sw_ = 0.075 if fein < 0.6 else 0.09 if fein < 0.75 else 0.1
        router = dict(lagen=L, drc=clr, drcpad=clr + 0.01, clr=round(clr + 0.005, 3), edge=0.2, h2h=0.2, via=via, sigw=sw_, sigmin=sw_, vip='1')
        RG = dict(quelle='lernen/aufgaben.py', lagen_quelle=L, roh=dict(min_clearance=clr, min_track_width=router['sigmin'], min_copper_edge_clearance=0.2,
                  min_hole_clearance=0.2, min_hole_to_hole=0.2, netclass_clearance=clr, netclass_track=router['sigw'],
                  netclass_via=[float(v) for v in via.split('/')], via_dimensions=[dict(diameter=float(via.split('/')[0]), drill=float(via.split('/')[1]))]),
                  router=router, placer=dict(padabst=0.15, rand=0.2),
                  args=' '.join('%s=%s' % (k, v) for k, v in router.items()))
        if aufbau:
            from analyse.aufbau import typisch
            RG['aufbau'] = typisch(L)
        json.dump(RG, open(os.path.join(ziel, 'regeln.json'), 'w'), indent=1)
        netze_vg = {}
        for n, e in sorted(self.netze.items()):
            if n not in nets or n == 'GND' or 'art' not in e: continue
            d = {'art': e['art']}
            for k_ in ('rang', 'f_Hz', 'datenrate', 'tr_ns', 'z_ohm', 'z_diff_ohm', 'laengengruppe', 'toleranz_mm', 'schnittstelle'):
                if e.get(k_) is not None: d[k_] = e[k_]
            if e['art'] in ('strom', 'versorgung'):
                a_ = e.get('strom_A') or 0.3
                d['strom'] = 'hoch' if a_ >= 2.0 else 'mittel' if a_ >= 0.5 else 'klein'
            netze_vg['^' + re.escape(n) + '$'] = d
        strom = dict(LAST)
        gruppen = {}
        for n, e in sorted(self.netze.items()):
            if n in nets and e.get('laengengruppe'):
                g = gruppen.setdefault(e['laengengruppe'], dict(netze=[], toleranz_mm=e.get('toleranz_mm'), schnittstelle=e.get('schnittstelle')))
                g['netze'].append(n); g['toleranz_mm'] = min(g['toleranz_mm'] or 9e9, e.get('toleranz_mm') or 9e9)
        refs_da = {f['ref'] for f in fps}
        VG = dict(netze=netze_vg, gnd=['GND'], fest=sorted(t['ref'] for t in self.teile if t['fest'] or t['ref'] in bedien), strom=strom,
                  zuordnung={r: u for r, u in sorted(self.insel_von.items()) if r in refs_da and u in refs_da},
                  tr_ns={'^' + re.escape(n) + '$': e['tr_ns'] for n, e in sorted(self.netze.items()) if e.get('tr_ns') and n in nets},
                  laengengruppen=gruppen,
                  spannung={'^' + re.escape(n) + '$': e['spannung'] for n, e in sorted(self.netze.items()) if e.get('spannung') and n in nets},
                  fsw={r: f for r, f in self.fsw.items() if f}, familie='aufgabe_%d' % (self.seed // 5))
        if self.leistungsboard or self.potenzial:
            VG.update(lagenaufbau=True, leistungsboard=self.leistungsboard)
        if aufbau: VG['aufbau'] = RG['aufbau']
        json.dump(VG, open(os.path.join(ziel, 'vorgaben.json'), 'w'), indent=1)
        if self.leistungsboard:

            rail = sorted(n for n in nets if self.netze.get(n, {}).get('art') in ('strom', 'versorgung'))
            gnd = ['In1=GND'] + ['In%d=%s' % (i, '|'.join(rail)) for i in range(2, L - 1)]
        elif L >= 6 and self.potenzial:
            rail = sorted(n for n in self.schienen if n in nets)
            gnd = (['In1=GND', 'In4=GND'] if L == 6 else ['In1=GND', 'In3=GND', 'In6=GND']) + ([('In2=' if L == 6 else 'In4=') + '|'.join(rail)] if rail else [])
        else:
            gnd = ['In1'] + (['In4'] if L == 6 else ['In3', 'In6'] if L == 8 else [])
        A_ = dict(seed=self.seed, version=version, stufe=self.stufe, schienen_A={n: a for n, a in sorted(self.schienen.items())}, lagen=L, gndlagen=gnd, faktor=round(self.faktor, 3), breite_mm=W, hoehe_mm=H,
                  laengengruppen={g: len(v['netze']) for g, v in sorted(gruppen.items())},
                  flaeche_mm2=round(W * H, 1), courtyard_mm2=round(flaeche, 1), teile=len(self.teile), pads=len(pads), netze=len(nets),
                  bloecke=self.bloecke, netze_eig={n: e for n, e in sorted(self.netze.items()) if n in nets})
        if self.version == BGA_MESSVERSION:
            from lernen.bga_belegung import statistik
            A_['bga_belegung'] = {t['ref']: statistik([p for p in pads if p['ref'] == t['ref']])
                                  for t in self.teile if 'BGA' in t['fp']}
        if self.leistungsboard or self.potenzial:
            A_.update(lagenaufbau=True, leistungsboard=self.leistungsboard)
        if echt:
            echt_regeln(ziel, B, RG, A_)
        json.dump(A_, open(os.path.join(ziel, 'aufgabe.json'), 'w'), indent=1, ensure_ascii=False)
        return A_


def stufe_achsen(stufe, seed=1):
    ""
    rnd = random.Random(seed * 7919 + 17); st = max(0.0, float(stufe))
    L = rnd.choice([6, 8, 6, 8] if st < 2 else [4, 6, 6, 8] if st < 5 else [4, 4, 6, 6, 8] if st < 8 else [4, 4, 4, 6])
    return dict(faktor=round(max(1.12, rnd.uniform(1.5, 1.7) - 0.045 * st), 2), lagen=L,
                aktive_pins=min(0.95, 0.30 + 0.065 * st),
                teile=0, strompfade=0, takt=1.0, paare=0, laengengruppen=0, analog=0.0)


def erzeugen(ziel, seed=1, lagen=0, faktor=0.0, bloecke='auto', stufe=0.0, aufbau=False, achsen=None, familie=None, potenzial=False, leistungsanteil=0.0, anker_legal=False, gehaeuse=None, generatorversion=None, **parameter):
    ""
    if gehaeuse not in (None, 'bga', 'qfn'): raise ValueError('gehaeuse=bga|qfn')
    if gehaeuse and (bloecke != 'auto' or leistungsanteil):
        raise ValueError('Gehaeuse-Achse braucht auto mit Prozessor (leistungsanteil=0)')
    rnd = random.Random(seed * 7919 + 17); st = max(0.0, float(stufe))
    L = lagen or rnd.choice([6, 8, 6, 8] if st < 2 else [4, 6, 6, 8] if st < 5 else [4, 4, 6, 6, 8] if st < 8 else [4, 4, 4, 6])
    f = faktor or round(max(1.12, rnd.uniform(1.5, 1.7) - 0.045 * st), 2)
    neu = achsen is not None or bool(parameter) or familie is not None
    ax = dict(achsen or {}, **parameter)
    if neu:
        from lernen.lehrer import achsen_pruefen, familie_id
        achsen_pruefen(ax)

        fam = familie or familie_id(bloecke)
        if fam.startswith('generator-v8:') and fam != familie_id(bloecke):
            raise ValueError('Blockfamilie stimmt nicht mit Bauplan ueberein')
        L = int(ax.get('lagen', L)); f = float(ax.get('faktor', f))
    a = Aufgabe(seed, L, f, st, ax if neu else None, potenzial=potenzial, gehaeuse=gehaeuse, generatorversion=generatorversion); a.bauen(bloecke, leistungsanteil=leistungsanteil)
    if neu: a.achsen_ergaenzen()
    A = a.schreiben(ziel, aufbau=aufbau, anker_legal=anker_legal)
    if neu:
        from lernen.lehrer import aufgabenmerkmale, plausibel
        A.update(achsen={**stufe_achsen(st, seed), 'ampere': max(A['schienen_A'].values(), default=0), **ax, 'lagen': L, 'faktor': f}, familie=fam,
                 abstammung=dict(ausgangsschaltung=seed, familie=fam), version=a.version + ('+anker-legal1' if anker_legal else '') + '+achsen1')
        B = json.load(open(os.path.join(ziel, 'board.json')))
        A['mcu_ref'] = getattr(a, 'mcu_ref', None)
        A['merkmale_aufgabe'] = aufgabenmerkmale(A, B)
        if A['teile'] < ax.get('teile', 0) or A['merkmale_aufgabe']['analog_ist'] + 1e-12 < ax.get('analog', 0):
            raise ValueError('Endartefakt unterschreitet Teile-/Analogminimum')
        A['plausibilitaet'] = plausibel(A, B)
        A['machbarkeit'] = 'unbekannt'
        if A['plausibilitaet']: raise ValueError('Unplausible Aufgabe: ' + '; '.join(A['plausibilitaet']))
        vg = json.load(open(os.path.join(ziel, 'vorgaben.json'))); vg['familie'] = fam
        json.dump(vg, open(os.path.join(ziel, 'vorgaben.json'), 'w'), indent=1)
        json.dump(A, open(os.path.join(ziel, 'aufgabe.json'), 'w'), indent=1, ensure_ascii=False)
    return A


def echt_umriss(w, h):
    ""
    r = min(3.0, w*.06, h*.06)
    edges, outer = [], []
    def line(a, b):
        edges.append(dict(typ='line', start=a, end=b)); outer.append(a)
    def arc(cx, cy, radius, a, b, target):
        def point(t): return [round(cx+radius*math.cos(t), 8), round(cy+radius*math.sin(t), 8)]
        edges.append(dict(typ='arc', start=point(a), mid=point((a+b)/2), end=point(b)))
        n = max(8, math.ceil(abs(b-a)/(.004/radius)**.5))
        target.extend(point(a+(b-a)*i/n) for i in range(n))
    line([r,0], [w-r,0]); arc(w-r,r,r,-math.pi/2,0,outer)
    line([w,r], [w,h-r]); arc(w-r,h-r,r,0,math.pi/2,outer)
    line([w-r,h], [r,h]); arc(r,h-r,r,math.pi/2,math.pi,outer)
    line([0,h-r], [0,r]); arc(r,r,r,math.pi,3*math.pi/2,outer)
    hole = []; radius = min(1.0,w*.018,h*.018)
    for i in range(4): arc(w*.10,h*.5,radius,i*math.pi/2,(i+1)*math.pi/2,hole)
    return [outer], [[hole]], edges


def echt_regeln(ziel, board, regeln, aufgabe):
    ""
    from pathlib import Path
    d = Path(ziel); clr = regeln['router']['drc']; sw = regeln['router']['sigw']
    classes = dict(Default=dict(clr=clr,w=sw), Power=dict(clr=.22,w=.4),
                   Analog=dict(clr=.18,w=.14), USB=dict(clr=.13,w=.16))
    for vals in classes.values():
        vals.update(via_d=float(regeln['router']['via'].split('/')[0]),
                    via_b=float(regeln['router']['via'].split('/')[1]))
    assignment = {}
    for n in board['nets']:
        art = aufgabe['netze_eig'].get(n, {}).get('art')
        assignment[n] = ('USB' if n.startswith('USB_TEST_') else 'Power' if art in ('strom','versorgung')
                         else 'Analog' if art == 'analog' else 'Default')
    board.update(netclass=assignment,netzklassen=classes,
                 design=dict(clearance_min=clr,track_width_min=sw,copper_edge_clearance=.2))
    project = dict(meta=dict(filename='board.kicad_pro',version=3),
        board=dict(design_settings=dict(rules=dict(regeln['roh']))),
        net_settings=dict(meta=dict(version=4),classes=[dict(name=n,clearance=v['clr'],track_width=v['w'],
            via_diameter=v['via_d'],via_drill=v['via_b'],diff_pair_width=.16,diff_pair_gap=.16) for n,v in classes.items()],
            netclass_patterns=[dict(pattern=n,netclass=c) for n,c in sorted(assignment.items())]))


    vg = json.loads((d/'vorgaben.json').read_text())
    vg['anker'] = [f['ref'] for f in board['fps'] if f['ref'].startswith('J') and not f['locked']]
    (d/'vorgaben.json').write_text(json.dumps(vg,indent=1))

    project['board']['design_settings']['rules'] = {k:v for k,v in regeln['roh'].items() if k.startswith('min_')}
    rules = project['board']['design_settings']['rules']
    vd, vb = classes['Default']['via_d'], classes['Default']['via_b']
    rules.update(min_via_diameter=vd,min_through_hole_diameter=vb,
                 min_via_annular_width=round((vd-vb)/2,6),
                 min_microvia_diameter=.2,min_microvia_drill=.1)
    dru = '''(version 1)
(rule "V11 Power Analog Paarabstand"
 (condition "(A.NetClass == 'Power' && B.NetClass == 'Analog') || (A.NetClass == 'Analog' && B.NetClass == 'Power')")
 (constraint clearance (min 0.30mm)))
(rule "V11 Power F Breite"
 (condition "A.NetClass == 'Power' && A.Layer == 'F.Cu'")
 (constraint track_width (min 0.40mm)))
(rule "V11 Analog Breite"
 (condition "A.NetClass == 'Analog'") (constraint track_width (min 0.14mm)))
(rule "V11 USB Name und Layer Breite"
 (condition "A.NetName == 'USB_TEST_*' && A.Layer == 'F.Cu'")
 (constraint track_width (min 0.13mm) (max 0.20mm)))
(rule "V11 USB Paarabstand"
 (condition "A.inDiffPair('USB_TEST')") (constraint diff_pair_gap (min 0.13mm) (max 0.25mm)))
(rule "V11 USB Skew"
 (condition "A.inDiffPair('USB_TEST')") (constraint skew (max 0.10mm)))
'''



    for fp in sorted(board['fps'],key=lambda q:q['ref']):
        ref = fp['ref']
        dru += ('(rule "V11 internes Padpaar %s"\n'
                ' (condition "A.Type == \'Pad\' && B.Type == \'Pad\' && A.memberOfFootprint(\'%s\') && B.memberOfFootprint(\'%s\')")\n'
                ' (constraint clearance (min %.9gmm)))\n') % (ref,ref,ref,clr)
    zones = [dict(net='GND',layer='In1',pts=board['outline'][0],
                  holes=board['outline_holes'][0],full=True,prio=0,local_clearance=clr,min_thickness=.1)]
    board['native_zones'] = zones
    aufgabe['echt_merkmale'] = dict(version=1, klassen=sorted(classes),
        verriegelte_stecker=[f['ref'] for f in board['fps'] if f['ref'].startswith('J') and f['locked']],
        lose_stecker=[f['ref'] for f in board['fps'] if f['ref'].startswith('J') and not f['locked']],
        boegen=8,aussparungen=1,usb_skew_mm=.1)
    (d/'board.json').write_text(json.dumps(board))
    (d/'board.kicad_pro').write_text(json.dumps(project,indent=1)+'\n')
    (d/'board.kicad_dru').write_text(dru)
    (d/'zones.json').write_text(json.dumps(zones))


def lernkurve_v11(ziel):
    ""
    import hashlib
    from pathlib import Path
    root = Path(ziel)/'lernkurve_v11'; root.mkdir(parents=True,exist_ok=True)
    manifest = dict(version='lernkurve-v11-1',generatorversion=ECHT_VERSION,aufgaben=[])
    for st in (0,2,4,6,8):
        for seed in (900001,900002,900003):
            name = 's%d_%d' % (st,seed); d = root/name
            if not d.exists(): erzeugen(str(d),seed*10+st,stufe=st,generatorversion=ECHT_VERSION)
            a = json.loads((d/'aufgabe.json').read_text())
            if a['version'] != ECHT_VERSION or a['seed'] != seed*10+st or a['stufe'] != st:
                raise ValueError('Fremder Satzinhalt: '+str(d))
            files = ('board.json','regeln.json','vorgaben.json','aufgabe.json','board.kicad_pro','board.kicad_dru','zones.json')
            manifest['aufgaben'].append(dict(id=name,seed=a['seed'],stufe=st,
                sha256={f:hashlib.sha256((d/f).read_bytes()).hexdigest() for f in files}))
    text = json.dumps(manifest,indent=1)+'\n'; p=root/'satz.json'
    reference = Path(R)/'codex/generator_echt/lernkurve_v11.json'
    if reference.exists() and json.loads(reference.read_text()) != manifest:
        raise ValueError('V11 weicht vom versionierten festen Satz ab')
    if p.exists() and p.read_text()!=text: raise ValueError('Eingefrorener Satz veraendert')
    p.write_text(text)
    return manifest


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    ziel = sys.argv[1]
    if kw.get('satz'):
        if kw['satz'] != 'lernkurve_v11': raise ValueError('satz=lernkurve_v11')
        lernkurve_v11(ziel); return
    s0 = int(kw.get('seed', '1')); n = int(kw.get('n', '1'))
    for s in range(s0, s0 + n):
        d = os.path.join(ziel, 'aufgabe_%06d' % s) if n > 1 or not kw.get('genau') else ziel
        ax = json.loads(kw['achsen']) if 'achsen' in kw else None
        A = erzeugen(d, s, int(kw.get('lagen', '0')), float(kw.get('faktor', '0')), kw.get('bloecke', 'auto'), float(kw.get('stufe', '0')), aufbau=kw.get('aufbau') == '1', achsen=ax, anker_legal=kw.get('anker_legal') == '1', potenzial=kw.get('potenzial') == '1', generatorversion=kw.get('generatorversion'), leistungsanteil=float(kw.get('leistungsanteil', '0')))
        print('Stufe %.1f %s: %d Lagen, %.1f x %.1f mm (Faktor %.2f), %d Teile, %d Pads, %d Netze | %s' % (
            A['stufe'], os.path.basename(d), A['lagen'], A['breite_mm'], A['hoehe_mm'], A['faktor'], A['teile'], A['pads'], A['netze'], '; '.join(A['bloecke'])))


if __name__ == '__main__':
    main()
