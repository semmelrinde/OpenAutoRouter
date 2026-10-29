import glob, json, math, os, re, sys

VERSION = 4

REGULATOR = re.compile(r'TPS5\d|TPS6\d|TPS54|TPS62|TPS61|TLV6[12]|LM259|LM267|LM2575|LM2576|LM2621|LM3671|LMR\d|LMZ|MP1\d{3}|MP2\d{3}|MP3\d{3}|'
                    r'MT3608|XL\d{4}|AP30\d\d|AP62|SX1308|RT8\d{3}|RT9\d{3}|SY8\d{3}|AOZ\d|LTC3\d{3}|LTM46|ADP2\d{3}|'
                    r'MAX177[01]|MAX172[2456]|TPS40|LM5\d{3}|NCP3\d{3}|FAN53|MCP1640|MCP16\d\d|CE8301|'
                    r'LM25\d{2,3}|LM340\d|LM3409|PT41\d\d|AL88\d\d|CL68\d\d|TPS92|MP24\d\d|LT8\d{3}|LTC7\d{3}|ISL8\d{3}|UCC2\d{4}|UC384\d|TL494|MC34063|'
                    r'buck|boost|step.?down|step.?up|DC.?DC|SMPS', re.I)
MCU = re.compile(r'STM32|STM8|ATMEGA|ATTINY|ATSAM|SAMD\d|ESP32|ESP8266|ESP-?WROOM|ESP-?12|RP2040|RP2350|NRF5\d|PIC\d\d|PIC32|DSPIC|'
                 r'MSP430|EFM32|EFR32|GD32|CH32|CH55\d|PY32|LPC\d{3,4}|MK\d\d|KINETIS|XMEGA|AT90|ATXMEGA|Arduino_Nano|Arduino_Pro.?Mini|Arduino_Uno|CC13[15]\d|CC26\d{2}|CC32\d{2}|CYPRESS|PSOC|'
                 r'IMXRT|MIMXRT|CY7C6801[34]', re.I)
FPGA = re.compile(r'ICE40|ECP5|LFE5|XC2C|XC3S|XC6S|XC7[A-Z]|ZYNQ|SPARTAN|ARTIX|MAX10|GW[125]N|EP[234]C|10CL|5CE|LCMXO|MachXO', re.I)
OPV = re.compile(r'LM358|LM324|LM321|LM741|TL07\d|TL08\d|TL06\d|NE5532|NE5534|OPA\d|MCP60\d|MCP61\d|MCP65\d|AD80[0123456]\d|AD86[0123456]\d|'
                 r'LMV3\d\d|TLV9\d|TLV2\d{3}|LT100[1678]|LT101[234]|LT111[2345]|LT112[245678]|LT12[0123456]\d|LT6\d{3}|OP07|OP27|OP37|OPA2\d|LMC6\d|TSV\d|LM833|LM4562|INA\d{3}|LTC6800|'
                 r'Amplifier_Operational|OpAmp|Op-Amp', re.I)


def _blocks(t, kinds):
    pattern = re.compile(r'"(?:\\.|[^"\\])*"|\((?:' + '|'.join(kinds) + r')(?=\s)')
    pos = 0
    while (m := pattern.search(t, pos)):
        if m.group().startswith('"'):
            pos = m.end()
            continue
        depth, quoted, escaped = 0, False, False
        for i in range(m.start(), len(t)):
            c = t[i]
            if quoted:
                if escaped: escaped = False
                elif c == "\\": escaped = True
                elif c == '"': quoted = False
            elif c == '"': quoted = True
            elif c == '(': depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    yield t[m.start():i + 1]
                    pos = i + 1
                    break
        else:
            return


def _footprints(t):
    out = []
    for s in _blocks(t, ('footprint', 'module')):
        first = re.match(r'\((?:footprint|module)\s+("(?:\\.|[^"\\])*"|[^\s()]+)', s)
        name = first.group(1).strip('"')

        properties = list(_blocks(s, ('property',)))
        m = next((m for b in properties if (m := re.match(r'\(property\s+"Value"\s+"((?:\\.|[^"\\])*)"', b))), None)
        if m is None:
            m = next((m for b in _blocks(s, ('fp_text',)) if (m := re.match(r'\(fp_text\s+value\s+(?:"((?:\\.|[^"\\])*)"|([^\s()]+))', b))), None)
        value = next((x for x in m.groups() if x is not None), '') if m else ''
        pads = list(_blocks(s, ('pad',)))
        nets = {n for pad in pads for n in re.findall(r'\(net\s+\d+\s+"([^"\n]*)"\)', pad)}
        out.append((name, value, len(pads), nets))
    return out


COIL = re.compile(r'(^|_)(Inductor|L_\d|L_[A-Z]|Choke|CD5\d|CD7\d|SRR|SRN|SRP|NR\d|DR\d|IHLP|XAL|XAL\d|SLF|VLS|LQH|MSS)', re.I)
CONNECTOR = re.compile(r'Conn|Pin_?Header|Socket|Header|USB|JST|Terminal|Jack|Phoenix|Molex|SMA_|Mounting|Hole|Logo|TestPoint', re.I)
GND = re.compile(r'GND|VSS|^0$|^$', re.I)


def _excluded(name, value):
    return bool(re.fullmatch(r'\s*(DNP|FB)\s*', value, re.I) or
                re.search(r'Ferrite|Bead|FB_', name + ' ' + value, re.I))


def _coil(name, value):
    return not _excluded(name, value) and bool(
        COIL.search(name.split(':')[-1]) or name.split(':')[0].lower().startswith('inductor'))


def _connection(name):

    library, _, short = name.rpartition(':')
    return bool(CONNECTOR.search(short or name) or
                re.match(r'Connector|PinHeader|PinSocket|Socket_Arduino', library, re.I))


def _switch_node(fp):
    ics = [x for x in fp if x[2] >= 5 and not CONNECTOR.search(x[0]) and not MCU.search(x[1]) and not FPGA.search(x[1]) and not OPV.search(x[1])]
    for n, v, p, net_list in fp:
        if not _coil(n, v): continue
        for nz in net_list:
            if GND.search(nz) or re.search(r'(^|/)(VCC|VDD|VIN|VOUT|VBAT|VBUS|VUSB|[+]?\d+[V.]\d*)$', nz, re.I): continue
            if any(nz in i[3] for i in ics): return True
    return False


def _arc_points(block, coords):
    if 'mid' in coords:
        a, m, b = (coords[k] for k in ('start', 'mid', 'end'))

        ux, uy = m[0] - a[0], m[1] - a[1]
        vx, vy = b[0] - a[0], b[1] - a[1]
        det = 2 * (ux * vy - uy * vx)
        if abs(det) < 1e-12: return [a, m, b]
        u2, v2 = ux * ux + uy * uy, vx * vx + vy * vy
        c = (a[0] + (u2 * vy - v2 * uy) / det, a[1] + (ux * v2 - vx * u2) / det)
        start, middle, end = (math.atan2(p[1] - c[1], p[0] - c[0]) for p in (a, m, b))
        sweep = (end - start) % math.tau
        if (middle - start) % math.tau > sweep: sweep -= math.tau
    else:
        c, a = coords['start'], coords['end']
        angle = next(_blocks(block, ('angle',)), '')
        sweep = math.radians(float(re.match(r'\(angle\s+([-+\d.eE]+)', angle).group(1)))
        start = math.atan2(a[1] - c[1], a[0] - c[0])
        r = math.hypot(a[0] - c[0], a[1] - c[1])
        b = (c[0] + r * math.cos(start + sweep), c[1] + r * math.sin(start + sweep))
    r = math.hypot(a[0] - c[0], a[1] - c[1])
    points = [a, b]
    sign = 1 if sweep >= 0 else -1
    for angle in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
        if (sign * (angle - start)) % math.tau <= abs(sweep) + 1e-12:
            points.append((c[0] + r * math.cos(angle), c[1] + r * math.sin(angle)))
    return points


def _contour_cm2(t):
    xs, ys = [], []
    for block in _blocks(t, ('gr_line', 'gr_rect', 'gr_arc', 'gr_circle', 'gr_poly')):
        if not re.search(r'\(layer\s+"?Edge\.Cuts"?\)', block): continue
        coords = re.findall(r'\((?:start|end|mid|center|xy)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\)', block)
        if block.startswith('(gr_circle') and len(coords) >= 2:
            cx, cy = map(float, coords[0]); ex, ey = map(float, coords[1])
            r = ((ex - cx)**2 + (ey - cy)**2)**0.5
            xs.extend((cx-r, cx+r)); ys.extend((cy-r, cy+r))
        elif block.startswith('(gr_arc'):
            fields = {k: (float(x), float(y)) for k, x, y in re.findall(
                r'\((start|end|mid)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\)', block)}
            for x, y in _arc_points(block, fields): xs.append(x); ys.append(y)
        else:
            for x, y in coords: xs.append(float(x)); ys.append(float(y))
    if len(xs) < 2 or max(xs) == min(xs) or max(ys) == min(ys): return None
    return (max(xs) - min(xs)) * (max(ys) - min(ys)) / 100.0


def evaluate(fpath, cu_layers=None):
    with open(fpath, errors='replace') as f:
        t = f.read()
    fp = _footprints(t)
    active = [x for x in fp if not _excluded(x[0], x[1])]
    net_list = set(re.findall(r'\(net \d+ "([^"]*)"\)', t))
    sw = any(re.search(r'(^|/|_)(SW|LX|PHASE|SW\d|LX\d)$', n, re.I) for n in net_list)
    converter = int(any((p >= 3 and REGULATOR.search(v)) or (p >= 5 and REGULATOR.search(n)) for n, v, p, _ in active if not n.split(':')[-1].startswith(('R_', 'C_')))
                  or _switch_node(active) or (sw and any(_coil(n, v) for n, v, _, _ in active)))
    mcu = int(any(MCU.search(v) or MCU.search(n) for n, v, _, _ in fp if not _connection(n)))
    fpga = int(any(FPGA.search(v) or FPGA.search(n) for n, v, _, _ in fp))
    bga = int(any(re.search(r'BGA|WLCSP|(^|_)CSP|iCE40UP5K-UWG30', n, re.I) for n, _, _, _ in fp))
    opv = int(any(OPV.search(v) or OPV.search(n) for n, v, _, _ in fp))
    pads = sum(p for _, _, p, _ in fp)
    table = next(_blocks(t, ('layers',)), '')


    cu_layers = len(re.findall(r'\(\d+\s+(?:"(?:\\.|[^"\\])*"|[^\s()]+)\s+(?:signal|power|mixed|jumper)(?=[\s)])', table)) or cu_layers or 2
    surface = _contour_cm2(t)
    density = pads / surface if surface else 0.0
    ds = 0 if density < 1.5 else 1 if density < 3.0 else 2 if density < 6.0 else 3
    ps = 0 if pads < 150 else 1 if pads < 400 else 2
    points = 3 * converter + 2 * max(mcu, fpga) + 4 * bga + opv + ds + max(0, cu_layers - 2) / 2 + ps
    return dict(converter=converter, mcu=mcu, fpga=fpga, bga=bga, opv=opv, pads=pads, cu_layers=cu_layers, area_cm2=round(surface or 0, 1),
                density=round(density, 2), points=round(points, 1))


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a); mn = float(kw.get('min', '5'))
    Z = []
    for d in sorted(glob.glob(os.path.join(sys.argv[1], '*'))):
        p = os.path.join(d, 'processed.kicad_pcb')
        if not os.path.exists(p): continue
        try: lg = int(json.load(open(os.path.join(d, 'metadata.json'))).get('layers') or 2)
        except Exception: lg = None
        Z.append((os.path.basename(d), evaluate(p, lg)))
    Z.sort(key=lambda x: -x[1]['points'])
    for n, s in Z:
        print('%-55s %4.1f  W%d M%d B%d O%d  %4d pads %d layers %5.2f/cm2' % (n[:55], s['points'], s['converter'], s['mcu'], s['bga'], s['opv'], s['pads'], s['cu_layers'], s['density']))
    print('%d boards, %d hard (>= %g), converters %d, MCU %d, BGA %d, OPV %d' % (len(Z), sum(s['points'] >= mn for _, s in Z), mn,
          sum(s['converter'] for _, s in Z), sum(s['mcu'] for _, s in Z), sum(s['bga'] for _, s in Z), sum(s['opv'] for _, s in Z)))


if __name__ == '__main__':
    main()
