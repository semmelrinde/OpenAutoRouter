import re

_PF = {'p': 1e-12, 'n': 1e-9, 'u': 1e-6, 'µ': 1e-6, 'm': 1e-3}
_PR = {'': 1.0, 'r': 1.0, 'R': 1.0, 'Ω': 1.0, 'k': 1e3, 'K': 1e3, 'M': 1e6, 'm': 1e-3}


def capacity(v):
    if not v: return None
    m = re.search(r'(\d+(?:[.,]\d+)?)\s*([pnuµm])\s*(\d*)\s*F?(?![a-zA-Z])', v)
    if not m:
        m = re.search(r'(\d+(?:[.,]\d+)?)\s*([pnuµm])(\d*)', v)
        if not m: return None
    count = m.group(1).replace(',', '.') + ('.' + m.group(3) if m.group(3) and '.' not in m.group(1) else '')
    try:
        return float(count) * _PF[m.group(2)]
    except ValueError:
        return None


def resistance(v):
    if not v: return None
    m = re.match(r'\s*[RrΩ](\d+)\b', v)
    if m: return float('0.' + m.group(1))
    m = re.match(r'\s*(\d+(?:[.,]\d+)?)\s*([RrkKMmΩ]?)(\d*)', v)
    if not m: return None
    count = m.group(1).replace(',', '.')
    if m.group(3) and '.' not in count: count += '.' + m.group(3)
    try:
        return float(count) * _PR[m.group(2)]
    except (ValueError, KeyError):
        return None


def prefix(ref):
    m = re.match(r'[A-Za-z]+', ref)
    return m.group(0).upper() if m else ''


def tokens(name):
    base = name.split('/')[-1] if name.split('/')[-1] else name
    base = base.replace('{slash}', '/').split('/')[-1] or base
    return [t for t in re.split(r'[_\-()/{}\s.,]+', base.upper()) if t]


def base_name(name):
    return name.split('/')[-1].replace('{slash}', '/')


SIZE = {'0201': 1, '0402': 2, '0603': 3, '0805': 4, '1206': 5, '1210': 6, '1812': 7, '2220': 8}


def package(fp):
    for k, v in SIZE.items():
        if k in fp: return v
    return 9


_FP_TYPE = [(re.compile(r'^(C_|CP_|CPol|C_Elec)|capacitor', re.I), 'C'), (re.compile(r'^R_|resistor|R_Array', re.I), 'R'),
           (re.compile(r'^(L_|IND)|inductor', re.I), 'L'), (re.compile(r'^(D_|LED_)|diode|SOD-', re.I), 'D'),
           (re.compile(r'crystal|^Y_|oscillator', re.I), 'Y'), (re.compile(r'fuse|^F_', re.I), 'F')]
KNOWN = {'R', 'C', 'L', 'D', 'Q', 'F', 'Y', 'U', 'J', 'FB', 'TVS', 'LED', 'RN', 'TP', 'SW', 'H', 'MH', 'IC', 'MCU', 'DAC',
           'ADC', 'OPV', 'OP', 'P', 'CN', 'CON', 'USB', 'X', 'ESD', 'NTC', 'BUCK', 'LDO', 'FID'}


def type_(ref, fp=''):
    if prefix(ref) == 'F' and re.match(r'^(R_|C_)', fp or ''): return 'F'
    for rx, t in _FP_TYPE:
        if rx.search(fp or ''): return t
    pf = prefix(ref)
    if pf in KNOWN: return pf
    return pf[:1] if pf[:1] in ('R', 'C', 'L', 'D', 'Q', 'F', 'Y', 'U', 'J') else pf
