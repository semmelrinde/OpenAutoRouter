import fnmatch, json, math, os, re, sys
from collections import defaultdict

SI_CONSTRAINTS = ('skew', 'diff_pair_gap', 'diff_pair_uncoupled', 'length')
HF_NAME = re.compile(r'(?i)\b(hf|rf)\b|ohm|impedan|antenn|\bant\b|50\s*ohm')
PAIR_NET_CLASS = re.compile(r'(?i)usb|diff|pair|pair|lvds|mipi|hdmi|ethernet')
HF_NET_CLASS = re.compile(r'(?i)\b(hf|rf)\b|ohm|impedan|antenn|^50')
GROUND = re.compile(r'(?i)(^|/)([ADP]?GND|VSS|GND_.*|.*_GND)$')
MEAS_NAME = re.compile(r'(?i)sense|shunt|isens|isns')
MEAS_IC = re.compile(r'(?i)shunt|sense|isens|^ina|\bina\d|csa|current')


def _design_rules(text):
    t = '\n'.join(l for l in text.splitlines() if not l.lstrip().startswith('#'))
    out = []
    for m in re.finditer(r'\(rule\s+"([^"]*)"', t):
        i, depth = m.start(), 0
        for j in range(i, len(t)):
            if t[j] == '(': depth += 1
            elif t[j] == ')':
                depth -= 1
                if depth == 0: out.append((m.group(1), t[i:j + 1])); break
    return out


def _dp_basis(n):
    for a, b in (('+', '-'), ('-', '+'), ('P', 'N'), ('N', 'P')):
        if n.endswith(a): return n[:-1], n[:-1] + b
    return None, None


def pair_partner(n):
    for a, b in (('_DP', '_DM'), ('_DM', '_DP')):
        if n.endswith(a): return n[:-len(a)] + b
    _, p = _dp_basis(n)
    return p


def pairs_after_names(net_list, exceptions=()):
    all_items = set(net_list); off = set(exceptions); out = {}
    for n in sorted(all_items):
        p = pair_partner(n)
        if not p or p not in all_items or n in off or p in off or n.startswith('unconnected-') or GROUND.search(n) or GROUND.search(p): continue
        out[n] = p
    return out


def _chains(segs):
    K = lambda p: (round(p[0], 4), round(p[1], 4))
    nb = defaultdict(list)
    for i, t in enumerate(segs): nb[K(t['a'])].append(i); nb[K(t['b'])].append(i)
    vacant = set(range(len(segs))); out = []
    def job(p, i):
        pts = [p]
        while True:
            vacant.discard(i); t = segs[i]; q = K(t['b']) if K(t['a']) == p else K(t['a']); pts.append(q); p = q
            nx = [j for j in nb[p] if j in vacant]
            if len(nb[p]) != 2 or not nx: return pts
            i = nx[0]
    for p in sorted(nb):
        if len(nb[p]) != 2:
            for i in list(nb[p]):
                if i in vacant: out.append(job(p, i))
    while vacant:
        i = min(vacant); out.append(job(K(segs[i]['a']), i))
    return out


def meander_turns(pts, nmin=3, factor=1.2, lmax=2.0, umin=150.0):
    seg = [(a, b) for a, b in zip(pts, pts[1:]) if math.dist(a, b) > 1e-6]
    if len(seg) < 3: return 0
    th = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in seg]
    pos, s = [], 0.0
    for a, b in seg: s += math.dist(a, b); pos.append(s)
    U, cl = [], None
    for i in range(len(seg) - 1):
        t = (th[i + 1] - th[i] + math.pi) % (2 * math.pi) - math.pi; p = pos[i]
        if abs(t) < math.radians(1): continue
        vz = 1 if t > 0 else -1
        if cl and cl[0] == vz and p - cl[3] <= lmax: cl[1] += t; cl[3] = p; continue
        if cl and abs(cl[1]) >= math.radians(umin): U.append(cl)
        cl = [vz, t, p, p]
    if cl and abs(cl[1]) >= math.radians(umin): U.append(cl)
    if len(U) < nmin: return 0
    def point(sx):
        acc = 0.0
        for a, b in seg:
            l = math.dist(a, b)
            if acc + l >= sx - 1e-12:
                f = 0.0 if l < 1e-12 else (sx - acc) / l
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            acc += l
        return seg[-1][1]
    best, i = 0, 0
    while i < len(U):
        j = i + 1
        while j < len(U) and U[j][0] != U[j - 1][0]: j += 1
        if j - i >= nmin and U[j - 1][3] - U[i][2] >= factor * max(math.dist(point(U[i][2]), point(U[j - 1][3])), 1e-6):
            best = max(best, j - i)
        i = j
    return best


def meander_nets(tracks, exceptions=()):
    g = defaultdict(list); off = set(exceptions)
    for t in tracks:
        if t.get('net') and t['net'] not in off and not GROUND.search(t['net']): g[(t['net'], t['layer'])].append(t)
    out = {}
    for (n, L), segs in sorted(g.items()):
        for pts in _chains(segs):
            k = meander_turns(pts)
            if k: out[n] = max(out.get(n, 0), k)
    return out


def _trunk(n):
    for a in ('_DP', '_DM', '_P', '_N', '+', '-'):
        if n.endswith(a): n = n[:-len(a)]; break
    m = re.match(r'^(.*?\D)\d+$', n)
    return m.group(1) if m else None


def length_groups(tracks, meander=(), pairs=(), tolerance=0.03):
    total_length = defaultdict(float)
    for t in tracks:
        if t.get('net'): total_length[t['net']] += math.dist(t['a'], t['b'])
    gr = defaultdict(list)
    for n in sorted(total_length):
        st = _trunk(n)
        if st and not GROUND.search(n): gr[st].append(n)
    out = {}; mae = set(meander)
    for st, ns in sorted(gr.items()):
        if len(ns) < 2: continue
        ls = [total_length[n] for n in ns]
        if (set(ns) & mae) or (len(ns) >= 3 and max(ls) > 1.0 and (max(ls) - min(ls)) <= tolerance * max(ls)):
            for n in ns: out[n] = st
    return out


def _ohm(amount):
    w = (amount or '').strip().split()[0] if (amount or '').strip() else ''
    w = w.replace('Ω', 'R').replace('ohm', 'R').replace('Ohm', 'R').replace('OHM', 'R')
    m = re.match(r'^(\d*)([RrKkMm])(\d+)$', w)
    if m and m.group(1) + m.group(3):
        f = {'r': 1.0, 'k': 1e3, 'm': 1e6}[m.group(2).lower()] if m.group(2) != 'm' else 1e-3
        return float((m.group(1) or '0') + '.' + m.group(3)) * f
    m = re.match(r'^(\d+(?:\.\d+)?)\s*([mkKM]?)R?$', w)
    if not m: return None
    return float(m.group(1)) * {'': 1.0, 'm': 1e-3, 'k': 1e3, 'K': 1e3, 'M': 1e6}[m.group(2)]


def _fp_read(pcb):
    t = open(pcb, encoding='utf-8').read(); out = {}
    for m in re.finditer(r'\n\t\(footprint "((?:[^"\\]|\\.)*)"', t):
        e = t.find('\n\t)', m.start() + 1); blk = t[m.start():e if e > 0 else len(t)]
        ref = re.search(r'\(property "Reference" "((?:[^"\\]|\\.)*)"', blk); val = re.search(r'\(property "Value" "((?:[^"\\]|\\.)*)"', blk)
        pads = []
        for pd in re.findall(r'\n\t\t\(pad "([^"]*)"(.*?)\n\t\t\)', blk, re.S):
            n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', pd[1]); pads.append((pd[0], n.group(1) if n else ''))
        out[ref.group(1) if ref else '?%d' % m.start()] = dict(value=val.group(1) if val else '', fp=m.group(1), pads=pads)
    return out


def shunts(fps):
    an = defaultdict(set)
    for ref, f in fps.items():
        if len(f['pads']) >= 5:
            for _, n in f['pads']:
                if n: an[n].add(ref)
    out = []
    for ref, f in sorted(fps.items()):
        ns = sorted({n for _, n in f['pads'] if n})
        if not re.match(r'(?i)^R', ref) or not 2 <= len(f['pads']) <= 4 or len(ns) < 2: continue
        ohm = _ohm(f.get('value'))
        small = ohm is not None and ohm <= 1.0
        name_r = bool(MEAS_IC.search(ref) or MEAS_IC.search(f.get('value') or ''))
        for ic in sorted(set().union(*(an[n] for n in ns)) - {ref}):
            common = [n for n in ns if ic in an[n]]
            if len(common) < 2: continue
            name_ic = bool(MEAS_IC.search(ic) or MEAS_IC.search(fps[ic].get('value') or ''))
            if small or name_r or (ohm is None and name_ic):
                out.append(dict(ref=ref, ic=ic, nets=common, reason='Shunt %s (%s) on %s' % (ref, f.get('value') or '?', ic)))
    return out


def kelvin_tracks(H, shunt_list, fill=None, without_path=None):
    if not shunt_list: return {}
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from common.copper_net import NetCopper
    NET_COPPER = NetCopper(H); out = {}
    for s_ in shunt_list:
        for n in s_['nets']:
            from_ = {pid for pid, _, _ in NET_COPPER.pads.get(n, []) if pid.startswith('P:%s.' % s_['ic'])}
            post = {pid for pid, _, _ in NET_COPPER.pads.get(n, []) if pid.startswith('P:%s.' % s_['ref'])}
            fl = [(f['layer'], f['poly']) for f in fill or () if f['net'] == n]
            ok, us = NET_COPPER.fpath(n, from_, post, areas=fl, status=True)
            if not ok:
                g = 'Kelvin sense line without copper path from IC to shunt (%s)' % s_['reason']
                print('WARNING: %s: %s; net fully locked' % (n, g), file=sys.stderr)
                if without_path is not None: without_path.setdefault(n, g)
            for u in us: out.setdefault(u, 'Kelvin sense line %s' % s_['reason'])
    return out


def _fits(match_pattern, n):
    return fnmatch.fnmatchcase(n, match_pattern) if any(c in match_pattern for c in '*?') else n == match_pattern


def _pcb_read(pcb):
    t = open(pcb, encoding='utf-8').read()
    net_list = set(re.findall(r'\(net "((?:[^"\\]|\\.)*)"\)', t))
    fps, ties = {}, []
    for m in re.finditer(r'\n\t\(footprint "', t):
        e = t.find('\n\t)', m.start() + 1); blk = t[m.start():e if e > 0 else len(t)]
        ref = re.search(r'\(property "Reference" "((?:[^"\\]|\\.)*)"', blk)
        ref = ref.group(1) if ref else '?%d' % m.start()
        pads = re.findall(r'\n\t\t\(pad "[^"]*".*?\n\t\t\)', blk, re.S)
        pn = [re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', p) for p in pads]
        fps[ref] = [x.group(1) if x else '' for x in pn]
        if '(net_tie_pad_groups' in blk: ties.append(dict(ref=ref, nets=sorted({x for x in fps[ref] if x})))
    zone_list = set()
    for m in re.finditer(r'\n\t\(zone\b', t):
        blk = t[m.start():m.start() + 800]
        if '(keepout' in t[m.start():t.find('\n\t)', m.start() + 1)]: continue
        z = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', blk)
        if z and z.group(1): zone_list.add(z.group(1))
    return net_list, fps, zone_list, ties


def _netclasses_nets(pro, net_list):
    if not pro or not os.path.exists(pro): return {}
    ns = json.load(open(pro)).get('net_settings') or {}
    out = {}
    for n, k in (ns.get('netclass_assignments') or {}).items():
        out[n] = k if isinstance(k, str) else ','.join(k)
    for p in ns.get('netclass_patterns') or []:
        for n in net_list:
            if n not in out and _fits(p.get('pattern', ''), n): out[n] = p.get('netclass', '')
    return out


def keepout_nets(pcb=None, pro=None, dru=None, net_list=None, pads=None, net_ties=None, netclasses=None, keepout_list=None, tracks=None, names=True):
    S = {}
    def blocked(n, reason):
        if n and n not in S: S[n] = reason
    fps, zone_list = defaultdict(list), set()
    all_items = set(net_list or [])
    if pcb:
        nz, fp, zone_list, ties = _pcb_read(pcb); all_items |= nz; fps.update(fp); net_ties = (net_ties or []) + ties
        b = os.path.splitext(pcb)[0]
        pro = pro or (b + '.kicad_pro' if os.path.exists(b + '.kicad_pro') else None)
        dru = dru or (b + '.kicad_dru' if os.path.exists(b + '.kicad_dru') else None)


    off_pcb = set(fps); seen = set()
    for p in pads or []:
        k = (p.get('ref'), p.get('num'), p.get('net'))
        if not p.get('ref') or p.get('net') is None or p['ref'] in off_pcb or k in seen: continue
        seen.add(k); fps[p['ref']].append(p['net'])
    all_items |= {n for ns in fps.values() for n in ns if n}
    all_items = sorted(all_items)
    hf = set()

    if dru and os.path.exists(dru):
        for name, txt in _design_rules(open(dru, encoding='utf-8').read()):
            cond = ' '.join(re.findall(r'\(condition\s+"((?:[^"\\]|\\.)*)"', txt))
            cons = re.findall(r'\(constraint\s+(\w+)([^()]*(?:\([^()]*\)[^()]*)*)\)', txt)
            types = {c for c, _ in cons}
            fixed_width = any(c == 'track_width' and '(max' in rest for c, rest in cons)
            for basis in re.findall(r"inDiffPair\('([^']*)'\)", cond):
                for n in all_items:
                    b0, _ = _dp_basis(n)
                    if b0 is not None and (_fits(basis, b0) or _fits(basis, b0.rstrip('_'))):
                        blocked(n, 'Differential pair (rule \"%s\")' % name)
            actual_hf = bool(HF_NAME.search(name))
            if actual_hf or types & set(SI_CONSTRAINTS) or fixed_width:
                for match_pattern in re.findall(r"NetName\s*==\s*'([^']*)'", cond):
                    for n in all_items:
                        if _fits(match_pattern, n):
                            if actual_hf: hf.add(n); blocked(n, 'RF (rule \"%s\")' % name)
                            else: blocked(n, 'Pair/length/fixed width (rule \"%s\")' % name)
                for nc in re.findall(r"NetClass\s*==\s*'([^']*)'", cond):
                    for n, k in _netclasses_nets(pro, all_items).items():
                        if k == nc:
                            if actual_hf: hf.add(n)
                            blocked(n, 'Netclass %s (rule \"%s\")' % (k, name))

    for n, k in sorted(_netclasses_nets(pro, all_items).items()):
        if HF_NET_CLASS.search(k): hf.add(n); blocked(n, 'RF netclass %s' % k)
        elif PAIR_NET_CLASS.search(k): blocked(n, 'Netclass %s' % k)
    new = True
    while new and hf:
        new = False
        for ref, ns in sorted(fps.items()):
            if len(ns) > 3 or not (set(ns) & hf): continue
            for n in ns:
                if n and n not in hf and n not in zone_list and not GROUND.search(n) and not n.startswith('unconnected-'):
                    hf.add(n); blocked(n, 'Matching net (%s to RF)' % ref); new = True
    if names:
        for n, p in pairs_after_names(all_items, zone_list).items(): blocked(n, 'Differential pair by name (with %s)' % p)
        for n in all_items:
            if MEAS_NAME.search(n) and n not in zone_list and not GROUND.search(n): blocked(n, 'sense line (name)')
    if tracks:
        for n, k in meander_nets(tracks, zone_list).items(): blocked(n, 'Meander (%d U-turns)' % k)

    count = defaultdict(int)
    for ns in fps.values():
        for n in ns: count[n] += 1
    for t in net_ties or []:
        ns = sorted(set(t.get('nets') or []))
        if len(ns) != 2: continue
        small = min(ns, key=lambda n: (count[n], n))
        blocked(small, 'Kelvin/Net-Tie %s' % t.get('ref', '?'))
    if netclasses:
        K = json.load(open(netclasses)) if isinstance(netclasses, str) else netclasses
        for n, k in sorted((K.get('net_list') or {}).items()):
            if k.get('kelvin'): blocked(n, 'Kelvin (netclasses.json, %s)' % k['kelvin'])
            if k.get('taps'): blocked(n, 'Tap (netclasses.json)')
        for a in K.get('taps') or []:
            if isinstance(a, dict) and a.get('net'): blocked(a['net'], 'Tap (netclasses.json)')
    for rx in [r for r in (keepout_list or '').split(';') if r]:
        for n in all_items:
            if re.search(rx, n): blocked(n, 'by hand (%s)' % rx)
    return dict(sorted(S.items()))


def keepout_extent(H, pcb=None, pro=None, dru=None, netclasses=None, keepout_list=None, auto=True):
    nz = {t['net'] for t in H.get('tracks', []) if t.get('net')}
    S = keepout_nets(pcb=pcb if auto else None, pro=pro if auto else None, dru=dru if auto else None, netclasses=netclasses if auto else None,
                   keepout_list=keepout_list, net_list=nz, pads=H.get('pads') if auto else None, net_ties=H.get('net_ties') if auto else None,
                   tracks=H.get('tracks') if auto else None, names=auto)
    if not auto: return dict(net_list=S, track_list={}, group_list={}, pairs={})
    zone_list = _pcb_read(pcb)[2] if pcb else set()
    all_items = nz | ({n for n in _pcb_read(pcb)[0]} if pcb else set())
    fps = _fp_read(pcb) if pcb else {}
    if not fps:
        tmp = defaultdict(dict)
        for p in H.get('pads', []):
            if p.get('ref'): tmp[p['ref']][p.get('num')] = p.get('net') or ''
        fps = {r: dict(value='', fp='', pads=sorted(d.items(), key=lambda x: str(x[0]))) for r, d in tmp.items()}
    fill = None
    if pcb:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from common.copper_net import zones_fill
        fill = zones_fill(open(pcb, encoding='utf-8').read())
    without = {}
    B = {u: g for u, g in kelvin_tracks(H, shunts(fps), fill, without).items()}
    for n, g in sorted(without.items()):
        if n not in S: S[n] = g
    S = dict(sorted(S.items()))
    pairs = pairs_after_names(all_items, zone_list)
    mae = {n for n, g in S.items() if g.startswith('Meander')}
    G = length_groups(H.get('tracks', []), mae, pairs)
    return dict(net_list=S, track_list=B, group_list=G, pairs=pairs, without_path=sorted(without))


def net_spacings(dru, net_list):
    out = {}
    if not dru or not os.path.exists(dru): return out
    for name, txt in _design_rules(open(dru, encoding='utf-8').read()):
        cond = ' '.join(re.findall(r'\(condition\s+"((?:[^"\\]|\\.)*)"', txt))
        if re.search(r"B\.Type\s*==\s*'(Zone|Pad|Via)'", cond): continue
        m = re.findall(r'\(constraint\s+clearance\s+\(min\s+([0-9.]+)\s*mm\)', txt)
        if not m: continue
        d = max(float(x) for x in m)
        for match_pattern in re.findall(r"A\.NetName\s*==\s*'([^']*)'", cond):
            for n in net_list:
                if _fits(match_pattern, n): out[n] = max(out.get(n, 0.0), d)
    return out


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[1:] if '=' in x)
    if o.get('obst'):
        U = keepout_extent(json.load(open(o['obst'])), pcb=a[0], pro=o.get('pro'), dru=o.get('dru'), netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'))
        S = U['net_list']
        for u, g in U['track_list'].items(): print('Track %-45s %s' % (u, g))
        print(len(U['track_list']), 'Tracks locked;', len(U['group_list']), 'Nets in length groups')
    else:
        S = keepout_nets(pcb=a[0], pro=o.get('pro'), dru=o.get('dru'), netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'))
    for n, g in S.items(): print('%-50s %s' % (n, g))
    print(len(S), 'Nets locked')
    if o.get('json'): json.dump(S, open(o['json'], 'w'), indent=1)


if __name__ == '__main__':
    main()
