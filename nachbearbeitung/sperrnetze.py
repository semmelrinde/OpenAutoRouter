""
import fnmatch, json, math, os, re, sys
from collections import defaultdict

SI_CONSTRAINTS = ('skew', 'diff_pair_gap', 'diff_pair_uncoupled', 'length')
HF_NAME = re.compile(r'(?i)\b(hf|rf)\b|ohm|impedan|antenn|\bant\b|50\s*ohm')
PAAR_KLASSE = re.compile(r'(?i)usb|diff|paar|pair|lvds|mipi|hdmi|ethernet')
HF_KLASSE = re.compile(r'(?i)\b(hf|rf)\b|ohm|impedan|antenn|^50')
MASSE = re.compile(r'(?i)(^|/)([ADP]?GND|VSS|GND_.*|.*_GND)$')
MESS_NAME = re.compile(r'(?i)sense|shunt|isens|isns')
MESS_IC = re.compile(r'(?i)shunt|sense|isens|^ina|\bina\d|csa|current')


def _regeln(text):
    ""
    t = '\n'.join(l for l in text.splitlines() if not l.lstrip().startswith('#'))
    out = []
    for m in re.finditer(r'\(rule\s+"([^"]*)"', t):
        i, tiefe = m.start(), 0
        for j in range(i, len(t)):
            if t[j] == '(': tiefe += 1
            elif t[j] == ')':
                tiefe -= 1
                if tiefe == 0: out.append((m.group(1), t[i:j + 1])); break
    return out


def _dp_basis(n):
    ""
    for a, b in (('+', '-'), ('-', '+'), ('P', 'N'), ('N', 'P')):
        if n.endswith(a): return n[:-1], n[:-1] + b
    return None, None


def paar_partner(n):
    ""
    for a, b in (('_DP', '_DM'), ('_DM', '_DP')):
        if n.endswith(a): return n[:-len(a)] + b
    _, p = _dp_basis(n)
    return p


def paare_nach_namen(netze, ausnahmen=()):
    ""
    alle = set(netze); aus = set(ausnahmen); out = {}
    for n in sorted(alle):
        p = paar_partner(n)
        if not p or p not in alle or n in aus or p in aus or n.startswith('unconnected-') or MASSE.search(n) or MASSE.search(p): continue
        out[n] = p
    return out


def _ketten(segs):
    ""
    K = lambda p: (round(p[0], 4), round(p[1], 4))
    nb = defaultdict(list)
    for i, t in enumerate(segs): nb[K(t['a'])].append(i); nb[K(t['b'])].append(i)
    frei = set(range(len(segs))); out = []
    def lauf(p, i):
        pts = [p]
        while True:
            frei.discard(i); t = segs[i]; q = K(t['b']) if K(t['a']) == p else K(t['a']); pts.append(q); p = q
            nx = [j for j in nb[p] if j in frei]
            if len(nb[p]) != 2 or not nx: return pts
            i = nx[0]
    for p in sorted(nb):
        if len(nb[p]) != 2:
            for i in list(nb[p]):
                if i in frei: out.append(lauf(p, i))
    while frei:
        i = min(frei); out.append(lauf(K(segs[i]['a']), i))
    return out


def maeander_kehren(pts, nmin=3, faktor=1.2, lmax=2.0, umin=150.0):
    ""
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
    def punkt(sx):
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
        if j - i >= nmin and U[j - 1][3] - U[i][2] >= faktor * max(math.dist(punkt(U[i][2]), punkt(U[j - 1][3])), 1e-6):
            best = max(best, j - i)
        i = j
    return best


def maeander_netze(tracks, ausnahmen=()):
    ""
    g = defaultdict(list); aus = set(ausnahmen)
    for t in tracks:
        if t.get('net') and t['net'] not in aus and not MASSE.search(t['net']): g[(t['net'], t['layer'])].append(t)
    out = {}
    for (n, L), segs in sorted(g.items()):
        for pts in _ketten(segs):
            k = maeander_kehren(pts)
            if k: out[n] = max(out.get(n, 0), k)
    return out


def _stamm(n):
    ""
    for a in ('_DP', '_DM', '_P', '_N', '+', '-'):
        if n.endswith(a): n = n[:-len(a)]; break
    m = re.match(r'^(.*?\D)\d+$', n)
    return m.group(1) if m else None


def laengengruppen(tracks, maeander=(), paare=(), toleranz=0.03):
    ""
    laenge = defaultdict(float)
    for t in tracks:
        if t.get('net'): laenge[t['net']] += math.dist(t['a'], t['b'])
    gr = defaultdict(list)
    for n in sorted(laenge):
        st = _stamm(n)
        if st and not MASSE.search(n): gr[st].append(n)
    out = {}; mae = set(maeander)
    for st, ns in sorted(gr.items()):
        if len(ns) < 2: continue
        ls = [laenge[n] for n in ns]
        if (set(ns) & mae) or (len(ns) >= 3 and max(ls) > 1.0 and (max(ls) - min(ls)) <= toleranz * max(ls)):
            for n in ns: out[n] = st
    return out


def _ohm(wert):
    ""
    w = (wert or '').strip().split()[0] if (wert or '').strip() else ''
    w = w.replace('Ω', 'R').replace('ohm', 'R').replace('Ohm', 'R').replace('OHM', 'R')
    m = re.match(r'^(\d*)([RrKkMm])(\d+)$', w)
    if m and m.group(1) + m.group(3):
        f = {'r': 1.0, 'k': 1e3, 'm': 1e6}[m.group(2).lower()] if m.group(2) != 'm' else 1e-3
        return float((m.group(1) or '0') + '.' + m.group(3)) * f
    m = re.match(r'^(\d+(?:\.\d+)?)\s*([mkKM]?)R?$', w)
    if not m: return None
    return float(m.group(1)) * {'': 1.0, 'm': 1e-3, 'k': 1e3, 'K': 1e3, 'M': 1e6}[m.group(2)]


def _fp_lesen(pcb):
    ""
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
    ""
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
        klein = ohm is not None and ohm <= 1.0
        name_r = bool(MESS_IC.search(ref) or MESS_IC.search(f.get('value') or ''))
        for ic in sorted(set().union(*(an[n] for n in ns)) - {ref}):
            gemeinsam = [n for n in ns if ic in an[n]]
            if len(gemeinsam) < 2: continue
            name_ic = bool(MESS_IC.search(ic) or MESS_IC.search(fps[ic].get('value') or ''))
            if klein or name_r or (ohm is None and name_ic):
                out.append(dict(ref=ref, ic=ic, nets=gemeinsam, grund='Shunt %s (%s) an %s' % (ref, f.get('value') or '?', ic)))
    return out


def kelvin_bahnen(H, shunt_liste, fuell=None, ohne_pfad=None):
    ""
    if not shunt_liste: return {}
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from gemeinsam.kupfer_netz import Netzkupfer
    NK = Netzkupfer(H); out = {}
    for s_ in shunt_liste:
        for n in s_['nets']:
            von = {pid for pid, _, _ in NK.pads.get(n, []) if pid.startswith('P:%s.' % s_['ic'])}
            nach = {pid for pid, _, _ in NK.pads.get(n, []) if pid.startswith('P:%s.' % s_['ref'])}
            fl = [(f['layer'], f['poly']) for f in fuell or () if f['net'] == n]
            ok, us = NK.pfad(n, von, nach, flaechen=fl, status=True)
            if not ok:
                g = 'Kelvin-Messleitung ohne Kupferweg vom IC zum Shunt (%s)' % s_['grund']
                print('WARNUNG: %s: %s; Netz ganz gesperrt' % (n, g), file=sys.stderr)
                if ohne_pfad is not None: ohne_pfad.setdefault(n, g)
            for u in us: out.setdefault(u, 'Kelvin-Messleitung %s' % s_['grund'])
    return out


def _passt(muster, n):
    return fnmatch.fnmatchcase(n, muster) if any(c in muster for c in '*?') else n == muster


def _pcb_lesen(pcb):
    ""
    t = open(pcb, encoding='utf-8').read()
    netze = set(re.findall(r'\(net "((?:[^"\\]|\\.)*)"\)', t))
    fps, ties = {}, []
    for m in re.finditer(r'\n\t\(footprint "', t):
        e = t.find('\n\t)', m.start() + 1); blk = t[m.start():e if e > 0 else len(t)]
        ref = re.search(r'\(property "Reference" "((?:[^"\\]|\\.)*)"', blk)
        ref = ref.group(1) if ref else '?%d' % m.start()
        pads = re.findall(r'\n\t\t\(pad "[^"]*".*?\n\t\t\)', blk, re.S)
        pn = [re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', p) for p in pads]
        fps[ref] = [x.group(1) if x else '' for x in pn]
        if '(net_tie_pad_groups' in blk: ties.append(dict(ref=ref, nets=sorted({x for x in fps[ref] if x})))
    zonen = set()
    for m in re.finditer(r'\n\t\(zone\b', t):
        blk = t[m.start():m.start() + 800]
        if '(keepout' in t[m.start():t.find('\n\t)', m.start() + 1)]: continue
        z = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', blk)
        if z and z.group(1): zonen.add(z.group(1))
    return netze, fps, zonen, ties


def _klassen_netze(pro, netze):
    ""
    if not pro or not os.path.exists(pro): return {}
    ns = json.load(open(pro)).get('net_settings') or {}
    out = {}
    for n, k in (ns.get('netclass_assignments') or {}).items():
        out[n] = k if isinstance(k, str) else ','.join(k)
    for p in ns.get('netclass_patterns') or []:
        for n in netze:
            if n not in out and _passt(p.get('pattern', ''), n): out[n] = p.get('netclass', '')
    return out


def sperrnetze(pcb=None, pro=None, dru=None, netze=None, pads=None, net_ties=None, klassen=None, sperren=None, tracks=None, namen=True):
    ""
    S = {}
    def sperr(n, grund):
        if n and n not in S: S[n] = grund
    fps, zonen = defaultdict(list), set()
    alle = set(netze or [])
    if pcb:
        nz, fp, zonen, ties = _pcb_lesen(pcb); alle |= nz; fps.update(fp); net_ties = (net_ties or []) + ties
        b = os.path.splitext(pcb)[0]
        pro = pro or (b + '.kicad_pro' if os.path.exists(b + '.kicad_pro') else None)
        dru = dru or (b + '.kicad_dru' if os.path.exists(b + '.kicad_dru') else None)


    aus_pcb = set(fps); gesehen = set()
    for p in pads or []:
        k = (p.get('ref'), p.get('num'), p.get('net'))
        if not p.get('ref') or p.get('net') is None or p['ref'] in aus_pcb or k in gesehen: continue
        gesehen.add(k); fps[p['ref']].append(p['net'])
    alle |= {n for ns in fps.values() for n in ns if n}
    alle = sorted(alle)
    hf = set()

    if dru and os.path.exists(dru):
        for name, txt in _regeln(open(dru, encoding='utf-8').read()):
            cond = ' '.join(re.findall(r'\(condition\s+"((?:[^"\\]|\\.)*)"', txt))
            cons = re.findall(r'\(constraint\s+(\w+)([^()]*(?:\([^()]*\)[^()]*)*)\)', txt)
            typen = {c for c, _ in cons}
            fest_breite = any(c == 'track_width' and '(max' in rest for c, rest in cons)
            for basis in re.findall(r"inDiffPair\('([^']*)'\)", cond):
                for n in alle:
                    b0, _ = _dp_basis(n)
                    if b0 is not None and (_passt(basis, b0) or _passt(basis, b0.rstrip('_'))):
                        sperr(n, 'Differenzpaar (Regel "%s")' % name)
            ist_hf = bool(HF_NAME.search(name))
            if ist_hf or typen & set(SI_CONSTRAINTS) or fest_breite:
                for muster in re.findall(r"NetName\s*==\s*'([^']*)'", cond):
                    for n in alle:
                        if _passt(muster, n):
                            if ist_hf: hf.add(n); sperr(n, 'HF (Regel "%s")' % name)
                            else: sperr(n, 'Paar/Länge/feste Breite (Regel "%s")' % name)
                for kl in re.findall(r"NetClass\s*==\s*'([^']*)'", cond):
                    for n, k in _klassen_netze(pro, alle).items():
                        if k == kl:
                            if ist_hf: hf.add(n)
                            sperr(n, 'Netzklasse %s (Regel "%s")' % (k, name))

    for n, k in sorted(_klassen_netze(pro, alle).items()):
        if HF_KLASSE.search(k): hf.add(n); sperr(n, 'HF-Netzklasse %s' % k)
        elif PAAR_KLASSE.search(k): sperr(n, 'Netzklasse %s' % k)
    neu = True
    while neu and hf:
        neu = False
        for ref, ns in sorted(fps.items()):
            if len(ns) > 3 or not (set(ns) & hf): continue
            for n in ns:
                if n and n not in hf and n not in zonen and not MASSE.search(n) and not n.startswith('unconnected-'):
                    hf.add(n); sperr(n, 'Anpassnetz (%s an HF)' % ref); neu = True
    if namen:
        for n, p in paare_nach_namen(alle, zonen).items(): sperr(n, 'Differenzpaar am Namen (mit %s)' % p)
        for n in alle:
            if MESS_NAME.search(n) and n not in zonen and not MASSE.search(n): sperr(n, 'Messleitung (Name)')
    if tracks:
        for n, k in maeander_netze(tracks, zonen).items(): sperr(n, 'Mäander (%d U-Kehren)' % k)

    zahl = defaultdict(int)
    for ns in fps.values():
        for n in ns: zahl[n] += 1
    for t in net_ties or []:
        ns = sorted(set(t.get('nets') or []))
        if len(ns) != 2: continue
        klein = min(ns, key=lambda n: (zahl[n], n))
        sperr(klein, 'Kelvin/Net-Tie %s' % t.get('ref', '?'))
    if klassen:
        K = json.load(open(klassen)) if isinstance(klassen, str) else klassen
        for n, k in sorted((K.get('netze') or {}).items()):
            if k.get('kelvin'): sperr(n, 'Kelvin (klassen.json, %s)' % k['kelvin'])
            if k.get('abgriffe'): sperr(n, 'Abgriff (klassen.json)')
        for a in K.get('abgriffe') or []:
            if isinstance(a, dict) and a.get('net'): sperr(a['net'], 'Abgriff (klassen.json)')
    for rx in [r for r in (sperren or '').split(';') if r]:
        for n in alle:
            if re.search(rx, n): sperr(n, 'von Hand (%s)' % rx)
    return dict(sorted(S.items()))


def sperrumfang(H, pcb=None, pro=None, dru=None, klassen=None, sperren=None, auto=True):
    ""
    nz = {t['net'] for t in H.get('tracks', []) if t.get('net')}
    S = sperrnetze(pcb=pcb if auto else None, pro=pro if auto else None, dru=dru if auto else None, klassen=klassen if auto else None,
                   sperren=sperren, netze=nz, pads=H.get('pads') if auto else None, net_ties=H.get('net_ties') if auto else None,
                   tracks=H.get('tracks') if auto else None, namen=auto)
    if not auto: return dict(netze=S, bahnen={}, gruppen={}, paare={})
    zonen = _pcb_lesen(pcb)[2] if pcb else set()
    alle = nz | ({n for n in _pcb_lesen(pcb)[0]} if pcb else set())
    fps = _fp_lesen(pcb) if pcb else {}
    if not fps:
        tmp = defaultdict(dict)
        for p in H.get('pads', []):
            if p.get('ref'): tmp[p['ref']][p.get('num')] = p.get('net') or ''
        fps = {r: dict(value='', fp='', pads=sorted(d.items(), key=lambda x: str(x[0]))) for r, d in tmp.items()}
    fuell = None
    if pcb:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from gemeinsam.kupfer_netz import zonen_fuellung
        fuell = zonen_fuellung(open(pcb, encoding='utf-8').read())
    ohne = {}
    B = {u: g for u, g in kelvin_bahnen(H, shunts(fps), fuell, ohne).items()}
    for n, g in sorted(ohne.items()):
        if n not in S: S[n] = g
    S = dict(sorted(S.items()))
    paare = paare_nach_namen(alle, zonen)
    mae = {n for n, g in S.items() if g.startswith('Mäander')}
    G = laengengruppen(H.get('tracks', []), mae, paare)
    return dict(netze=S, bahnen=B, gruppen=G, paare=paare, ohne_pfad=sorted(ohne))


def netz_abstaende(dru, netze):
    ""
    out = {}
    if not dru or not os.path.exists(dru): return out
    for name, txt in _regeln(open(dru, encoding='utf-8').read()):
        cond = ' '.join(re.findall(r'\(condition\s+"((?:[^"\\]|\\.)*)"', txt))
        if re.search(r"B\.Type\s*==\s*'(Zone|Pad|Via)'", cond): continue
        m = re.findall(r'\(constraint\s+clearance\s+\(min\s+([0-9.]+)\s*mm\)', txt)
        if not m: continue
        d = max(float(x) for x in m)
        for muster in re.findall(r"A\.NetName\s*==\s*'([^']*)'", cond):
            for n in netze:
                if _passt(muster, n): out[n] = max(out.get(n, 0.0), d)
    return out


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[1:] if '=' in x)
    if o.get('hind'):
        U = sperrumfang(json.load(open(o['hind'])), pcb=a[0], pro=o.get('pro'), dru=o.get('dru'), klassen=o.get('klassen'), sperren=o.get('sperren'))
        S = U['netze']
        for u, g in U['bahnen'].items(): print('Bahn %-45s %s' % (u, g))
        print(len(U['bahnen']), 'Bahnen gesperrt;', len(U['gruppen']), 'Netze in Längengruppen')
    else:
        S = sperrnetze(pcb=a[0], pro=o.get('pro'), dru=o.get('dru'), klassen=o.get('klassen'), sperren=o.get('sperren'))
    for n, g in S.items(): print('%-50s %s' % (n, g))
    print(len(S), 'Netze gesperrt')
    if o.get('json'): json.dump(S, open(o['json'], 'w'), indent=1)


if __name__ == '__main__':
    main()
