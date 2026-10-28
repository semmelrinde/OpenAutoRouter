""
import json, re
SUFFIX = re.compile(r'(.*?)(_?)(P|N|DP|DN|DM|\+|-)$')
SCHNELL = re.compile(r'USB|LVDS|HDMI|TMDS|MIPI|CSI|DSI|PCIE|SATA|ETH|RGMII|SGMII|TX|RX|CLK', re.I)
def norm(n): return n.replace('{slash}', '/')
def paarnetze(netze, klassen=None):
    ""
    if klassen:
        return {norm(n) for n, k in json.load(open(klassen))['netze'].items() if k.get('paar')}
    by = {}
    for n in map(norm, netze):
        pfad, _, basis = n.rpartition('/')
        m = SUFFIX.match(basis)
        if m and SCHNELL.search(n): by.setdefault((pfad, m.group(1)), set()).add(n)
    return {n for v in by.values() if len(v) == 2 for n in v}


def _positiv(n):
    ""
    m = SUFFIX.match(norm(n).rpartition('/')[2])
    return bool(m) and m.group(3) in ('P', 'DP', '+')


def paare(netze, klassen=None):
    ""
    netze = sorted(set(netze)); roh = {norm(n): n for n in netze}
    if klassen:
        K = json.load(open(klassen))['netze']; out = set()
        for n, k in sorted(K.items()):
            q = k.get('paar')
            if not q or norm(n) not in roh or norm(q) not in roh: continue
            a, b = roh[norm(n)], roh[norm(q)]
            out.add((a, b) if _positiv(a) or not _positiv(b) and a < b else (b, a))
        return sorted(out)
    by = {}
    for n in netze:
        nn = norm(n); pfad, _, basis = nn.rpartition('/')
        m = SUFFIX.match(basis)
        if m and SCHNELL.search(nn): by.setdefault((pfad, m.group(1)), []).append(n)
    out = []
    for _, v in sorted(by.items()):
        if len(v) != 2: continue
        a, b = sorted(v)
        out.append((a, b) if _positiv(a) else (b, a))
    return sorted(out)
