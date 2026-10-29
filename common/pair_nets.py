import json, re
SUFFIX = re.compile(r'(.*?)(_?)(P|N|DP|DN|DM|\+|-)$')
FAST = re.compile(r'USB|LVDS|HDMI|TMDS|MIPI|CSI|DSI|PCIE|SATA|ETH|RGMII|SGMII|TX|RX|CLK', re.I)
def norm(n): return n.replace('{slash}', '/')
def pair_nets(net_list, netclasses=None):
    if netclasses:
        return {norm(n) for n, k in json.load(open(netclasses))['net_list'].items() if k.get('pair')}
    by = {}
    for n in map(norm, net_list):
        fpath, _, basis = n.rpartition('/')
        m = SUFFIX.match(basis)
        if m and FAST.search(n): by.setdefault((fpath, m.group(1)), set()).add(n)
    return {n for v in by.values() if len(v) == 2 for n in v}


def _positive(n):
    m = SUFFIX.match(norm(n).rpartition('/')[2])
    return bool(m) and m.group(3) in ('P', 'DP', '+')


def pairs(net_list, netclasses=None):
    net_list = sorted(set(net_list)); raw_data = {norm(n): n for n in net_list}
    if netclasses:
        K = json.load(open(netclasses))['net_list']; out = set()
        for n, k in sorted(K.items()):
            q = k.get('pair')
            if not q or norm(n) not in raw_data or norm(q) not in raw_data: continue
            a, b = raw_data[norm(n)], raw_data[norm(q)]
            out.add((a, b) if _positive(a) or not _positive(b) and a < b else (b, a))
        return sorted(out)
    by = {}
    for n in net_list:
        nn = norm(n); fpath, _, basis = nn.rpartition('/')
        m = SUFFIX.match(basis)
        if m and FAST.search(nn): by.setdefault((fpath, m.group(1)), []).append(n)
    out = []
    for _, v in sorted(by.items()):
        if len(v) != 2: continue
        a, b = sorted(v)
        out.append((a, b) if _positive(a) else (b, a))
    return sorted(out)
