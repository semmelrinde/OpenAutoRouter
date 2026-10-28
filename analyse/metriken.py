""
import collections, heapq, json, math, os, re, sys, time

import numpy as np
import shapely
from shapely.geometry import Polygon, Point

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VERSION = 'v3.1-review_fix-r3'
ER = 4.1
T_AUSSEN, T_INNEN = 0.035, 0.0175
PS_MM = 3.336
RHO = 1.72e-5
MU0 = 1.2566
L_FREI = 1.0
DICKE = 1.6
PREPREG = 0.0994
AUFBAU_JLC = {2: [1.5], 4: [0.0994, 1.265, 0.0994], 6: [0.0994, 0.55, 0.109, 0.55, 0.0994]}
ZCLR = 0.15
ABTAST = 0.25
SPALT_MIN = 1.0
SPALT_NH = 5.0
S_MAX = 30.0
VIA_HUELSE = 0.02
ANTIPAD = 0.4
J_L_VOLL = 1.0
J_HALS_L, J_HALS_D = 2.0, 1.0
TOL = 0.03
I_MIN_LAST = 0.01


def stapel_namen(n):
    return ['F'] + ['In%d' % i for i in range(1, n - 1)] + ['B']


def dielektrika(n):
    if n in AUFBAU_JLC: return list(AUFBAU_JLC[n])
    k = n - 1; pp = [i for i in range(k) if i % 2 == 0]; kern = [i for i in range(k) if i % 2 == 1]
    rest = DICKE - 2 * T_AUSSEN - (n - 2) * T_INNEN - PREPREG * len(pp)
    return [PREPREG if i % 2 == 0 else max(0.1, rest / max(1, len(kern))) for i in range(k)]


def aufbau(n_lagen, ebenen, diel=None, er=ER):
    ""
    S = stapel_namen(n_lagen); D = list(diel) if diel else dielektrika(n_lagen)
    z = [0.0]
    for d in D: z.append(z[-1] + d)
    E = [L for L in S if L in set(ebenen)]
    lagen = {}
    for i, L in enumerate(S):
        t = T_AUSSEN if L in ('F', 'B') else T_INNEN
        ob = [j for j in range(i) if S[j] in E]; un = [j for j in range(i + 1, len(S)) if S[j] in E]
        h1 = z[i] - z[ob[-1]] if ob else None; h2 = z[un[0]] - z[i] if un else None
        if L in E: typ, ref, h, H = 'ebene', L, 0.0, 0.0
        elif h1 is None and h2 is None: typ, ref, h, H = 'offen', None, None, None
        else:
            ref = S[ob[-1]] if (h2 is None or (h1 is not None and h1 <= h2)) else S[un[0]]
            h = min(x for x in (h1, h2) if x is not None)
            if L in ('F', 'B'): typ = 'micro'
            else: typ = 'strip2' if (h1 is not None and h2 is not None) else 'strip1'
            H = h1 * h2 / (h1 + h2) if (h1 is not None and h2 is not None) else h
        lagen[L] = dict(i=i, z=z[i], t=t, typ=typ, ref=ref, h=h, h1=h1, h2=h2, H=H)
    return dict(stapel=S, z=z, dielektrika=D, ebenen=E, er=er, lagen=lagen, dicke=z[-1])


def leitung(A, L, w):
    ""
    g = A['lagen'][L]; er = A['er']; t = g['t']
    if g['typ'] == 'micro':
        Z = 87 / math.sqrt(er + 1.41) * math.log(max(1.05, 5.98 * g['h'] / (0.8 * w + t)))
        tpd = math.sqrt(0.475 * er + 0.67) * PS_MM
    elif g['typ'] in ('strip1', 'strip2'):
        b = (g['h1'] + g['h2'] + t) if g['typ'] == 'strip2' else 2 * g['h'] + t
        Z = 60 / math.sqrt(er) * math.log(max(1.05, 4 * b / (0.67 * math.pi * (0.8 * w + t))))
        tpd = math.sqrt(er) * PS_MM
    else:
        tpd = math.sqrt(0.475 * er + 0.67) * PS_MM; Z = 1000 * L_FREI / tpd
    Z = max(Z, 5.0)
    return Z, tpd, Z * tpd / 1000.0, tpd / Z


def l_ebene(A, L, w):
    ""
    g = A['lagen'].get(L)
    if g is None or g['typ'] in ('offen',) or not g.get('h'): return L_FREI
    if g['typ'] == 'ebene': return 0.0
    return min(MU0 * g['h'] / (w + 2 * g['h']), leitung(A, L, w)[2] if w < 3 * g['h'] else 1e9)


def via_lc(h, d, A, d_pad):
    ""
    h = max(h, 0.05)
    return 0.2 * h * (math.log(4 * h / d) + 1), 0.0555 * A['er'] * A['dicke'] * d_pad / ANTIPAD


def breite_ipc(I, innen=False, t=None, modell=None):
    ""
    if not I or I <= 0: return 0.0
    if modell == '2221':
        from analyse.strombreite import ipc2221
        return ipc2221(I, t=t if t is not None else (T_INNEN if innen else T_AUSSEN), innen=innen)
    A_mil2 = (I / (0.048 * 10 ** 0.44)) ** (1 / 0.725)
    return A_mil2 / 1.378 * 0.0254 * (2.3 if innen else 1.0)


def lade_gewichte(pfad=None, si=False):
    pfad = pfad or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'metriken_gewichte_si.json' if si else 'metriken_gewichte.json')
    with open(pfad) as f: G = json.load(f)

    if G.get('version') == 'v3.2':
        standard = lade_gewichte()
        for key in ('minimax', 'profile'):
            G.setdefault(key, standard[key])
    return G


def klasse(k):
    a = (k or {}).get('art', 'signal'); r = (k or {}).get('rang', 0) or 0
    if a == 'schnell': return 'schnell_r5' if r >= 5 else ('schnell_r4' if r == 4 else 'schnell_r3')
    if a == 'sw': return 'sw'
    if a in ('strom', 'versorgung'): return 'strom'
    if a in ('analog', 'kelvin'): return 'analog'
    if a == 'gnd': return 'gnd'
    return 'schnell_r3' if r >= 3 else ('langsam' if r >= 1 else 'gpio')


def strafe(x, knick):
    return x if x <= 1 else 1 + knick * (x - 1)


def _mst(pts):
    if len(pts) < 2: return 0.0
    inT = [pts[0]]; rest = list(pts[1:]); s = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inT), i) for i, r in enumerate(rest)); s += d; inT.append(rest.pop(i))
    return s


def review_aktiv(K):
    ""
    v = K.get('vorgaben') or {}
    return bool(v.get('review_fix') or v.get('stoer_flanke') or K.get('stoer_flanke') or v.get('rueck_hoehe') or v.get('ipc_innen_modell') == '2221')


def _via_geo(R, K):
    if review_aktiv(K):
        from analyse.kupfermodell import via_geo
        return via_geo(R, K)
    v = str((R.get('optionen') or {}).get('via') or (K.get('regeln') or {}).get('via') or '0.45/0.20').split('/')
    try: return float(v[0]), float(v[1])
    except (ValueError, IndexError): return 0.45, 0.20


def _rueckweg_standard(B, R, A, GND, via_d, drill, flaechen=False, K=None):
    ""
    je = collections.defaultdict(lambda: dict(l_ohne=0.0, l_rand=0.0, n_spalt=0))
    if flaechen:

        for t in R.get('tracks', []):
            g = A['lagen'].get(t['layer'])
            if t['net'] not in GND and g is not None and g['typ'] == 'offen':
                je[t['net']]['l_ohne'] += sum(math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    if not A['ebenen']: return je, {}
    ol = [Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]
    umriss = shapely.union_all(ol) if ol else None
    if flaechen:
        from analyse.flaechen import umriss as board_umriss
        umriss = board_umriss(B)
    rand = umriss.boundary if umriss is not None else None

    loch_xy, loch_r, loch_n = [], [], []
    for v in R.get('vias', []):
        if v['net'] in GND: continue
        loch_xy.append((v['x'], v['y'])); loch_r.append(via_d / 2 + ZCLR); loch_n.append(v['net'])
    for p in B['pads']:
        if not p.get('drill') or p['net'] in GND: continue
        bb = p.get('bbox') or [p['x'], p['y'], p['x'], p['y']]
        loch_xy.append((p['x'], p['y'])); loch_r.append(max(bb[2] - bb[0], bb[3] - bb[1], p['drill']) / 2 + ZCLR); loch_n.append(p['net'] or '')
    loecher = shapely.buffer(shapely.points(loch_xy), loch_r, quad_segs=4) if loch_xy else None
    baum = shapely.STRtree(loecher) if loecher is not None else None

    P, PN, PT, PL, PH, PREF = [], [], [], [], [], []
    for ti, t in enumerate(R.get('tracks', [])):
        g = A['lagen'].get(t['layer'])
        if t['net'] in GND or g is None or g['typ'] in ('ebene', 'offen'): continue
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            k = max(1, int(math.ceil(d / ABTAST)))
            for j in range(k):
                f = (j + 0.5) / k; P.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                PN.append(t['net']); PT.append(ti); PL.append(d / k); PH.append(g['h']); PREF.append(g['ref'])
    if not P: return je, {}
    pts = shapely.points(P); frei = np.zeros(len(P), bool)
    filled = collections.defaultdict(list)
    for z in R.get('zones', []) if flaechen else []:
        if z.get('filled') and z['net'] in GND:
            filled[z['layer']].append(Polygon(z['pts'], z.get('holes', [])).buffer(0))
    if filled:

        from analyse.flaechen import verbunden
        connected = collections.defaultdict(list)
        for n in sorted(GND):
            cc = verbunden(B, R, K or dict(lagen=len(A['stapel']), gnd=sorted(GND)), n, details=True)
            for L, gg in cc.items(): connected[L] += gg
        for L, gg in filled.items():
            g = shapely.union_all(gg).intersection(shapely.union_all(connected[L]))
            filled[L] = [g]
    if flaechen:
        frei |= np.asarray([ref not in filled for ref in PREF], dtype=bool)
    if baum is not None:
        pi, li = baum.query(pts, predicate='intersects')
        for a, b in zip(pi.tolist(), li.tolist()):
            if loch_n[b] != PN[a] and PREF[a] not in filled: frei[a] = True

    for L, gg in filled.items():
        ids = np.asarray([i for i, ref in enumerate(PREF) if ref == L], dtype=int)
        if len(ids): frei[ids] |= ~shapely.covers(shapely.union_all(gg), pts[ids])
    am_rand = np.zeros(len(P), bool)
    if rand is not None:
        am_rand = shapely.distance(pts, rand) < 3 * np.asarray(PH)
        if umriss is not None: frei |= ~shapely.contains(umriss, pts)
    run = 0.0; vor_t = None; vor_frei = False; links = False
    for i in range(len(P)):
        n = PN[i]; e = je[n]
        if frei[i]: e['l_ohne'] += PL[i]
        elif am_rand[i]: e['l_rand'] += PL[i]
        if PT[i] != vor_t: run = 0.0; links = False; vor_frei = False
        if frei[i]: run += PL[i]
        else:
            if vor_frei and links and run >= SPALT_MIN: e['n_spalt'] += 1
            run = 0.0; links = True
        vor_frei = bool(frei[i]); vor_t = PT[i]
    return je, {}




def _knoten(L, x, y):
    return (L, round(x / TOL) * TOL, round(y / TOL) * TOL)


def rueckweg(B, R, A, GND, via_d, drill, K=None, flaechen=False):
    if flaechen:
        return _rueckweg_standard(B, R, A, GND, via_d, drill, flaechen=True, K=K)
    ""
    if K is not None and not review_aktiv(K): return _rueckweg_standard(B, R, A, GND, via_d, drill)
    if not A['ebenen']: return {}, {}
    ol = [Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]
    umriss = shapely.union_all(ol) if ol else None
    rand = umriss.boundary if umriss is not None else None
    from analyse.kupfermodell import bezug, einstellungen, angeschlossen
    clr, zrand = einstellungen(K)
    fill = {L: bezug(B, R, L, {A['flaechennetze'][L]} if 'flaechennetze' in A else GND,
                     clr=clr, rand=zrand, via_d=via_d, drill=drill) for L in A['ebenen']}
    netze = A.get('flaechennetze') or {L: next(iter(GND)) for L in A['ebenen'] if len(GND) == 1}
    roh = fill
    fill = angeschlossen(B, R, roh, netze, via_d=via_d, drill=drill)

    P, PN, PT, PL, PH, PR = [], [], [], [], [], []
    for ti, t in enumerate(R.get('tracks', [])):
        g = A['lagen'].get(t['layer'])
        if t['net'] in GND or g is None or g['typ'] in ('ebene', 'offen'): continue
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            k = max(1, int(math.ceil(d / ABTAST)))
            for j in range(k):
                f = (j + 0.5) / k; P.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
                PN.append(t['net']); PT.append(ti); PL.append(d / k); PH.append(g['h']); PR.append(g['ref'])
    je = collections.defaultdict(lambda: dict(l_ohne=0.0, l_rand=0.0, n_spalt=0, l_unbekannt=0.0))
    if not P: return je, {}
    pts = shapely.points(P); frei = np.zeros(len(P), bool); unbekannt = np.zeros(len(P), bool)
    for L, geom in fill.items():
        idx = np.flatnonzero(np.array([r == L for r in PR]))
        if len(idx):
            frei[idx] = ~shapely.covers(geom, pts[idx])
            unbekannt[idx] = frei[idx] & shapely.covers(roh[L], pts[idx])
    am_rand = np.zeros(len(P), bool)
    if rand is not None:
        am_rand = shapely.distance(pts, rand) < 3 * np.asarray(PH)
        if umriss is not None: frei |= ~shapely.contains(umriss, pts)
    run = 0.0; vor_t = None; vor_frei = False; links = False
    for i in range(len(P)):
        n = PN[i]; e = je[n]
        if unbekannt[i]: e['l_unbekannt'] += PL[i]
        if frei[i]: e['l_ohne'] += PL[i]
        elif am_rand[i]: e['l_rand'] += PL[i]
        if PT[i] != vor_t: run = 0.0; links = False; vor_frei = False
        if frei[i]: run += PL[i]
        else:
            if vor_frei and links and run >= SPALT_MIN: e['n_spalt'] += 1
            run = 0.0; links = True
        vor_frei = bool(frei[i]); vor_t = PT[i]
    return je, {}




def _knoten(L, x, y):
    return (L, round(x / TOL) * TOL, round(y / TOL) * TOL)


def strombaum(tracks, vias, pads, fluss, A, drill, I, graph=None):
    ""
    from gemeinsam.fluss import netzgraph
    if not fluss or not tracks: return None
    quelle_k = set(fluss.get('quelle') or []); last = fluss.get('last') or {}
    R_via = RHO * A['dicke'] / (math.pi * (drill + VIA_HUELSE) * VIA_HUELSE)
    g = graph if graph is not None else netzgraph(tracks, vias, pads, TOL); adj = g['adj']; padk = g['padk']
    seg = []; s_idx = {}; s_r = {}
    for ti, t in enumerate(tracks):
        L = t['layer']; tl = A['lagen'].get(L); dicke = tl['t'] if tl else T_AUSSEN
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            l = math.dist(a, b)
            if l < 1e-9: continue
            s_idx[(ti, si)] = len(seg); s_r[(ti, si)] = RHO * l / (w * dicke)
            seg.append([ti, si, L, a, b, w, l, 0.0, dicke])
    def widerstand(kid):
        if kid is None: return 0.0
        if kid[0] == 'v': return R_via
        return s_r[(kid[1], kid[2])] * (kid[4] - kid[3])
    starts = [padk[q] for q in sorted(quelle_k) if q in padk and padk[q] in adj]
    if not starts: return None
    vor = {s: None for s in starts}; ordnung = list(starts); i = 0
    while i < len(ordnung):
        k = ordnung[i]; i += 1
        for m, kid in adj[k]:
            if m in vor: continue
            vor[m] = (k, widerstand(kid), kid); ordnung.append(m)
    unten = collections.defaultdict(float)
    for key, pk in padk.items():
        if pk in vor and key not in quelle_k: unten[pk] += float(last.get(key, 0.0))
    for k in reversed(ordnung):
        if vor[k] is not None: unten[vor[k][0]] += unten[k]
    gesamt = sum(unten[s] for s in starts) or 1.0
    U = {}; P = 0.0; via_I = [0.0] * len(vias)
    for k in ordnung:
        if vor[k] is None: U[k] = 0.0; continue
        p_, r, kid = vor[k]; i_k = unten[k] / gesamt * I
        U[k] = U[p_] + i_k * r; P += i_k * i_k * r
        if kid and kid[0] == 's':
            j = s_idx[(kid[1], kid[2])]; seg[j][7] = max(seg[j][7], i_k)
        elif kid and kid[0] == 'v': via_I[kid[1]] = max(via_I[kid[1]], i_k)
    erreicht = sum(1 for pk in padk.values() if pk in vor)
    return dict(P=P, dV=max(U.values()) if U else 0.0, seg=[tuple(s_) for s_ in seg], via_I=via_I, erreicht=erreicht, pads=len(padk))


def stromdichte(seg, A, n, pads=None, ipc_modell=None):
    ""
    x_max = 0.0; st = []
    pp = [(p['bbox'], set(p['layers'])) for p in (pads or [])]
    def am_pad(L, a, b, l):
        if l > J_HALS_L: return False
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        for (x0, y0, x1, y1), lg in pp:
            if L in lg and x0 - J_HALS_D <= mx <= x1 + J_HALS_D and y0 - J_HALS_D <= my <= y1 + J_HALS_D: return True
        return False
    for ti, si, L, a, b, w, l, i_s, dicke in seg:
        if i_s <= 0 or am_pad(L, a, b, l): continue
        innen = L not in ('F', 'B')
        ws = breite_ipc(i_s, innen, t=dicke, modell=ipc_modell)
        x = ws / w
        x_eff = x if x <= 1 else 1 + (x - 1) * min(1.0, l / J_L_VOLL)
        if x_eff > x_max: x_max = x_eff
        if x > 1: st.append((round(x_eff, 3), n, L, round((a[0] + b[0]) / 2, 2), round((a[1] + b[1]) / 2, 2), round(w, 3), round(ws, 3),
                             round(i_s, 3), round(i_s / (w * dicke), 1), round(l, 2)))
    return x_max, st


def _strom(n, K, wie_loeser=False):
    ""
    k = K['netze'].get(n) or {}; fl = k.get('fluss') or {}
    if fl.get('einheit') == 'A':
        q = fl.get('einheit_quelle') or 'A'
        if fl.get('lernbegrenzung'): q += ' (lernbegrenzt)'
        return float(fl.get('gesamt') or 0.0), q
    if wie_loeser: return 1.0, '1A'
    if k.get('strom_A') is not None: return float(k['strom_A']), 'schaetzung'
    return 1.0, '1A'


def gleichstrom_schaetzung(B, R, K, netze, wie_loeser=False, fluss_cache=None):
    ""
    n_lagen = int(K.get('lagen') or (R.get('optionen') or {}).get('lagen') or B.get('lagen') or 2)
    A = aufbau(n_lagen, [])
    via_d, drill = _via_geo(R, K)
    R_via = RHO * A['dicke'] / (math.pi * (drill + VIA_HUELSE) * VIA_HUELSE)
    want = set(netze); tr = collections.defaultdict(list); vi = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R.get('tracks', []):
        if t['net'] in want and not t.get('sig') and t['layer'] in A['lagen']: tr[t['net']].append(t)
    for v in R.get('vias', []):
        if v['net'] in want: vi[v['net']].append(v)
    for p in B['pads']:
        if p['net'] in want: pads[p['net']].append(p)
    out = {}
    for n in sorted(want):
        if n not in K['netze'] or not tr[n]: continue
        I, _q = _strom(n, K, wie_loeser)
        b = strombaum(tr[n], vi[n], pads[n], K['netze'][n].get('fluss'), A, drill, I, graph=fluss_cache.graph(tr[n], vi[n], pads[n], TOL) if fluss_cache is not None else None) if I > 0 else None
        if b: out[n] = (1000 * b['dV'], 1000 * b['P'])
        else:
            r = sum(RHO * math.dist(a, c) / (w * A['lagen'][t['layer']]['t']) for t in tr[n] for a, c, w in zip(t['pts'][:-1], t['pts'][1:], t['w']))
            r += len(vi[n]) * R_via
            out[n] = (1000 * I * r, 1000 * I * I * r)
    return out


def spannung_mit_quelle(n, K):
    ""
    VG = K.get('vorgaben') or {}; b = n.split('/')[-1]
    for rx, v in sorted((VG.get('spannung') or {}).items()):
        try:
            if b == rx or n == rx or re.fullmatch(rx, b):
                if isinstance(v, (int, float)) and math.isfinite(v): return float(v), 'vorgabe'
        except re.error: pass
    k = K['netze'].get(n) or {}; v = k.get('spannung_V')
    if isinstance(v, (int, float)) and math.isfinite(v): return float(v), k.get('spannung_quelle') or 'klasse_ohne_herkunft'
    m = re.search(r'(\d+)V(\d*)', b.upper())
    if m: return float(m.group(1) + ('.' + m.group(2) if m.group(2) else '')), 'name_schaetzung'
    return None, None


def spannung(n, K):
    return spannung_mit_quelle(n, K)[0]


def budgets(K):
    ""
    VG = K.get('vorgaben') or {}; je, gr = {}, {}
    for n in K['netze']:
        for rx, v in (VG.get('budget_mV') or {}).items():
            try:
                if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): je[n] = float(v); break
            except re.error: pass
    for name, g in sorted((VG.get('kanal_budget_mV') or {}).items()):
        nn = [x for x in g.get('netze', []) if x in K['netze']]
        if nn: gr[name] = (nn, float(g['mV']))
    return je, gr




def _pad_poly(p, L):
    qs = [Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    if qs: return shapely.union_all(qs)
    return shapely.box(*p['bbox'])


def _netzgraph(tracks, vias, pads_netz, A, drill):
    ""
    from gemeinsam.fluss import _seg_dist
    ends = collections.defaultdict(list)
    for t in tracks:
        for q in t['pts']: ends[t['layer']].append(tuple(q))
    for v in vias:
        for L in sorted({t['layer'] for t in tracks}): ends[L].append((v['x'], v['y']))
    adj = collections.defaultdict(list); pos = {}
    def kante(a, b, Lnh, l, nv):
        adj[a].append((b, Lnh, l, nv)); adj[b].append((a, Lnh, l, nv))
    for t in tracks:
        L = t['layer']
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            teil = [(0.0, tuple(a)), (1.0, tuple(b))]
            for q in ends[L]:
                d, s = _seg_dist(q, a, b)
                if d < TOL and 1e-6 < s < 1 - 1e-6: teil.append((s, q))
            teil.sort(); lp = l_ebene(A, L, w)
            for (s1, p1), (s2, p2) in zip(teil[:-1], teil[1:]):
                l = math.dist(p1, p2)
                if l < 1e-9: continue
                k1, k2 = _knoten(L, *p1), _knoten(L, *p2); pos[k1] = p1; pos[k2] = p2
                kante(k1, k2, lp * l, l, 0)
    lagen = sorted({t['layer'] for t in tracks} | {L for p in pads_netz for L in p['layers'] if L in A['lagen']})
    for v in vias:
        ks = [_knoten(L, v['x'], v['y']) for L in lagen if L in A['lagen']]
        for k in ks: pos[k] = (v['x'], v['y'])
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                h = abs(A['lagen'][ks[i][0]]['z'] - A['lagen'][ks[j][0]]['z'])
                kante(ks[i], ks[j], via_lc(h, drill, A, 0.45)[0], h, 1)
    knoten = [k for k in pos if k[0] != 'PAD']
    kp = shapely.points([pos[k] for k in knoten]) if knoten else None
    geo = {}
    for p in pads_netz:
        pk = ('PAD', p['ref'], p['num']); pos[pk] = (p['x'], p['y'])
        for L in p['layers']:
            if L not in A['lagen']: continue
            g = geo[(pk, L)] = _pad_poly(p, L)
            if kp is None: continue
            d = shapely.distance(g, kp)
            for i in np.nonzero(d <= TOL)[0].tolist():
                if knoten[i][0] == L: kante(pk, knoten[i], 0.0, 0.0, 0)

    pl = [p for p in pads_netz]
    for i, p in enumerate(pl):
        for q in pl[i + 1:]:
            for L in sorted(set(p['layers']) & set(q['layers'])):
                if L not in A['lagen']: continue
                a, b = geo[(('PAD', p['ref'], p['num']), L)], geo[(('PAD', q['ref'], q['num']), L)]
                if a.distance(b) <= 1e-6:
                    l = math.dist((p['x'], p['y']), (q['x'], q['y']))
                    w = max(0.2, min(a.bounds[2] - a.bounds[0], a.bounds[3] - a.bounds[1]))
                    kante(('PAD', p['ref'], p['num']), ('PAD', q['ref'], q['num']), l_ebene(A, L, w) * l, l, 0); break
    return adj, pos


def _kuerzester(adj, s, z):
    ""
    if s not in adj or z not in adj: return None
    best = {s: 0.0}; vor = {}; hq = [(0.0, 0, s)]; n = 0
    while hq:
        d, _, k = heapq.heappop(hq)
        if k == z: break
        if d > best.get(k, 1e18): continue
        for m, Lnh, l, nv in adj[k]:
            nd = d + Lnh + 1e-6 * l
            if nd < best.get(m, 1e18): best[m] = nd; vor[m] = (k, l, nv); n += 1; heapq.heappush(hq, (nd, n, m))
    if z not in best: return None
    weg = [z]; l = 0.0; nv = 0
    while weg[-1] != s:
        k, li, vi = vor[weg[-1]]; l += li; nv += vi; weg.append(k)
    return best[z], l, nv, weg[::-1]


def _masse_anschluss(p, A, gnd_vias, gnd_tracks, drill):
    ""
    if not A['ebenen']: return None, 'keine Masselage'
    L_pad = next((L for L in p['layers'] if L in A['lagen']), None)
    if L_pad is None: return None, 'Lage'
    z0 = A['lagen'][L_pad]['z']
    h = min(abs(A['lagen'][E]['z'] - z0) for E in A['ebenen'])
    if h < 1e-9: return 0.0, 'Pad auf der Masselage'
    lv = via_lc(h, drill, A, 0.45)[0]
    g = _pad_poly(p, L_pad)
    if p.get('drill'): return lv / 2, 'THT'
    if not len(gnd_vias): return L_FREI * 5, 'kein Masse-Via'
    d = np.asarray(shapely.distance(g, shapely.points(gnd_vias)), dtype=float)
    im = int(np.sum(d <= 0.05))
    if im: return lv / im, '%d Via im Pad' % im
    j = int(np.argmin(d)); dist = float(d[j])
    w = 0.25
    for t in gnd_tracks:
        if t['layer'] == L_pad and any(g.distance(Point(q)) <= TOL for q in (t['pts'][0], t['pts'][-1])): w = max(w, max(t['w']))
    return l_ebene(A, L_pad, w) * dist + lv, 'Stich %.2f mm' % dist


def hot_loops(B, R, K, A, GND, drill):
    ""
    T = K.get('teile') or {}; N = K['netze']
    pads = collections.defaultdict(list); pad = {}
    for p in B['pads']:
        pad[(p['ref'], p['num'])] = p
        if p['net']: pads[p['net']].append(p)
    regler = {}
    for r, t in sorted(T.items()):
        st = t.get('stuetzt') or {}
        if t.get('funktion') == 'spule_sw' and st.get('ref'): regler.setdefault(st['ref'], st.get('netz'))
    gnd_vias = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] or np.zeros((0, 2)))
    gnd_tracks = [t for t in R.get('tracks', []) if t['net'] in GND]
    out = []
    for chip, sw in sorted(regler.items()):
        cs = []
        for r, t in sorted(T.items()):
            st = t.get('stuetzt') or {}
            if not str(t.get('funktion', '')).startswith('entkopplung') or st.get('ref') != chip or st.get('ueber_spule'): continue
            if st.get('netz') == sw: continue
            cs.append((t.get('C') or 1.0, r, st))
        if not cs: continue
        C, cr, st = min(cs, key=lambda c: (c[0], c[1]))
        vin_net = st.get('netz'); vin_pin = pad.get((chip, str(st.get('pin'))))
        cp = [p for p in B['pads'] if p['ref'] == cr]
        c_plus = next((p for p in cp if p['net'] == vin_net), None); c_minus = next((p for p in cp if p['net'] in GND), None)
        pg = [p for p in B['pads'] if p['ref'] == chip and p['net'] in GND]
        if not (vin_pin and c_plus and c_minus and pg): continue
        pgnd = min(pg, key=lambda p: (math.dist((p['x'], p['y']), (vin_pin['x'], vin_pin['y'])), p['num']))
        tr = [t for t in R.get('tracks', []) if t['net'] == vin_net and not t.get('sig')]
        vi = [v for v in R.get('vias', []) if v['net'] == vin_net]
        adj, pos = _netzgraph(tr, vi, pads[vin_net], A, drill)
        w = _kuerzester(adj, ('PAD', c_plus['ref'], c_plus['num']), ('PAD', vin_pin['ref'], vin_pin['num']))
        L_c = next((L for L in c_plus['layers'] if L in A['lagen']), 'F')
        if w is not None:
            L_vin, l_vin, n_v, weg = w; punkte = [pos[k] for k in weg if k in pos]; quelle = 'Bahn'
        else:
            gem = set(c_plus['layers']) & set(vin_pin['layers'])
            if any(_pad_poly(c_plus, L).distance(_pad_poly(vin_pin, L)) <= 1e-6 for L in gem):
                L_vin, l_vin, n_v = 0.0, 0.0, 0; quelle = 'Pads beruehren sich'
            else:
                l_vin = math.dist((c_plus['x'], c_plus['y']), (vin_pin['x'], vin_pin['y'])); n_v = 0
                L_vin = l_ebene(A, L_c, max(0.3, (N.get(vin_net) or {}).get('w') or 0.3)) * l_vin; quelle = 'Luftlinie (nicht verbunden)'
            punkte = [(c_plus['x'], c_plus['y']), (vin_pin['x'], vin_pin['y'])]
        L_pg, q_pg = _masse_anschluss(pgnd, A, gnd_vias, gnd_tracks, drill)
        L_cm, q_cm = _masse_anschluss(c_minus, A, gnd_vias, gnd_tracks, drill)
        poly = [(c_plus['x'], c_plus['y'])] + punkte + [(vin_pin['x'], vin_pin['y']), (pgnd['x'], pgnd['y']), (c_minus['x'], c_minus['y'])]
        fl = 0.5 * abs(sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1] for i in range(len(poly))))
        L_hl = L_vin + (L_pg or 0.0) + (L_cm or 0.0)
        out.append(dict(chip=chip, sw=sw, vin=vin_net, c_in=cr, C_F=C, vin_pin=vin_pin['num'], pgnd_pin=pgnd['num'],
                        L_nH=round(L_hl, 4), L_vin_nH=round(L_vin, 4), l_vin_mm=round(l_vin, 3), vias_vin=n_v, vin_weg=quelle,
                        L_pgnd_nH=None if L_pg is None else round(L_pg, 4), pgnd=q_pg, L_cmasse_nH=None if L_cm is None else round(L_cm, 4),
                        cmasse=q_cm, A_mm2=round(fl, 3)))
    return out




def masse_kopplung(B, R, K, A, GND):
    ""
    nts = list(K.get('net_ties') or [])
    if not nts or not A['ebenen']: return None
    T = K.get('teile') or {}; N = K['netze']
    t_ges = sum(A['lagen'][E]['t'] for E in A['ebenen'])
    Rq = RHO / t_ges
    pads = collections.defaultdict(list)
    for p in B['pads']: pads[p['ref']].append(p)
    gv = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] or np.zeros((0, 2)))
    GV = shapely.points(gv) if len(gv) else None
    gt = [t for t in R.get('tracks', []) if t['net'] in GND]
    def eintritt(p):
        ""
        if p.get('drill') or GV is None: return [(p['x'], p['y'])]
        g = _pad_poly(p, p['layers'][0])
        d = np.asarray(shapely.distance(g, GV), dtype=float)
        nah = [tuple(gv[i].tolist()) for i in np.argsort(d, kind='stable')[:4] if d[i] <= 0.6]
        if nah: return nah
        st = []
        for t in gt:
            if t['layer'] not in p['layers']: continue
            a, b = t['pts'][0], t['pts'][-1]
            for e, f in ((a, b), (b, a)):
                if g.distance(Point(e)) <= TOL:
                    j = int(np.argmin(np.hypot(gv[:, 0] - f[0], gv[:, 1] - f[1])))
                    if math.dist(gv[j], f) <= 0.05: st.append(tuple(gv[j].tolist()))
        if st: return sorted(set(st))
        j = int(np.argmin(d))
        return [tuple(gv[j].tolist())] if d[j] <= 3.0 else [(p['x'], p['y'])]

    opfer = {}
    for nt in sorted(nts):
        P = pads.get(nt) or []
        g = [p for p in P if p['net'] in GND]; s = [p for p in P if p['net'] and p['net'] not in GND]
        if not g or not s: continue
        mm = s[0]['net']
        I = 0.0
        for r, t in sorted(T.items()):
            if r == nt or not t.get('strom_A'): continue
            if any(p['net'] == mm for p in pads.get(r, [])) and any((N.get(p['net']) or {}).get('art') in ('strom', 'versorgung') for p in pads.get(r, [])):
                I += float(t['strom_A'])
        opfer[nt] = dict(netz=mm, punkt=eintritt(g[0]), I=I)
    if not opfer: return None

    zahl = collections.Counter()
    for p in B['pads']:
        if p['net'] and (N.get(p['net']) or {}).get('art') == 'analog' and (T.get(p['ref']) or {}).get('rolle') == 'chip': zahl[p['ref']] += 1
    ref_chip = min(zahl, key=lambda r: (-zahl[r], r)) if zahl else None
    ref_pts = [q for p in pads.get(ref_chip, []) if p['net'] in GND for q in eintritt(p)][:8] if ref_chip else []
    if not ref_pts: return None

    senke_ein = []
    for n, k in sorted(N.items()):
        fl = k.get('fluss') or {}
        if fl.get('von') or k.get('art') not in ('strom', 'versorgung'): continue
        for q in fl.get('quelle') or []:
            r = q.split(':')[0]
            if (T.get(r) or {}).get('rolle') == 'stecker':
                senke_ein += [pt for p in pads.get(r, []) if p['net'] in GND for pt in eintritt(p)]
    senke_ein = sorted(set(senke_ein))
    regler = sorted({(t.get('stuetzt') or {}).get('ref') for t in T.values() if t.get('funktion') == 'spule_sw'} - {None})
    VG = K.get('vorgaben') or {}
    def senken(D):
        ""
        s = []
        rp = [pt for r in regler for p in pads.get(r, []) if p['net'] in GND for pt in eintritt(p)]
        if senke_ein: s += [(pt, (D if rp else 1.0) / len(senke_ein)) for pt in senke_ein]
        if rp: s += [(pt, (1 - D if senke_ein else 1.0) / len(rp)) for pt in rp]
        return s
    D = float(np.mean([float(VG['umsetzung'][r]) for r in regler if r in (VG.get('umsetzung') or {})])) if regler and VG.get('umsetzung') else 0.5

    angr = {}
    for nt, o in opfer.items():
        if o['I'] >= I_MIN_LAST: angr[nt] = [(pt, o['I'] / len(o['punkt'])) for pt in o['punkt']]
    for r, t in sorted(T.items()):
        if t.get('rolle') != 'chip' or r in regler or (t.get('strom_A') or 0) < I_MIN_LAST: continue
        if 'durchfluss' in str(t.get('strom_A_quelle') or ''): continue
        gp = [pt for p in pads.get(r, []) if p['net'] in GND for pt in eintritt(p)]
        if gp: angr[r] = [(pt, float(t['strom_A']) / len(gp)) for pt in gp]
    if not angr: return None
    S = senken(D)
    if not S: return None
    k = Rq / (2 * math.pi) * 1e6
    def pot(pt, quellen):
        u = 0.0
        for q, I in quellen:
            u -= I * math.log(max(math.dist(pt, q), 0.2))
        return u
    tab = {}; mx = 0.0; je = collections.defaultdict(float)
    for a, Q in sorted(angr.items()):
        I = sum(i for _, i in Q)
        quellen = Q + [(pt, -I * f) for pt, f in S]
        u_ref = float(np.mean([pot(p, quellen) for p in ref_pts]))
        for v, o in sorted(opfer.items()):
            if v == a: continue
            e = k * (float(np.mean([pot(p, quellen) for p in o['punkt']])) - u_ref)
            tab['%s<-%s' % (v, a)] = round(e, 3); je[o['netz']] = max(je[o['netz']], abs(e)); mx = max(mx, abs(e))
    return dict(max_uV=round(mx, 3), je_netz={n: round(v, 3) for n, v in sorted(je.items())}, tabelle=tab, R_quadrat_mOhm=round(1000 * Rq, 4),
                bezug=ref_chip, senken=len(S), D=round(D, 3), angreifer=sorted(angr))




def messen(B, R, K, ir=None, gewichte=None, gndlagen=None, AU=None, fluss_cache=None, minimax=False):
    ""
    from gemeinsam.zonen import aktiv, vereinheitlichen, boardgeometrie
    B = boardgeometrie(B, R)
    r36 = aktiv(R)
    R = vereinheitlichen(R)
    kupfer_modell = bool(R.get('kupferbalken')) or r36
    if kupfer_modell:
        from analyse.kupfermodell import ersatzrouten, ir_werte
        if ir is None:
            ir = ir_werte(B, R, K)
            if r36 and ir['fehler_netze']:
                ir = ir_werte(B, R, K, res=.05)
        if not r36:
            R = ersatzrouten(B, R, K)

        from gemeinsam.fluss import NetzgraphCache
        fluss_cache = NetzgraphCache(endpunkte=True)
    review = review_aktiv(K)
    G = gewichte or lade_gewichte(si=(K.get('vorgaben') or {}).get('si_bewertung') is True)
    si_aktiv = G.get('version') == 'v3.2' and bool(G.get('si'))
    N = K['netze']; GND = set(K.get('gnd') or [])
    from analyse.flaechen import aktiv, zuordnung, kupfer, kennzahl
    la_aktiv = aktiv(R, K)
    LAGENAUFBAU = None; FLMAP = {}; FR = R
    if la_aktiv:
        FLMAP = zuordnung(B, R, K, gndlagen=gndlagen)
        versorgungsflaechen = {n for nn in FLMAP.values() for n in nn} - GND
        versorgungsflaechen |= set(R.get('flaechennetze') or []) - GND
        versorgungsflaechen |= {z['net'] for z in R.get('zones', []) if z.get('filled') and z['net'] not in GND and
                              (z['net'] not in N or N[z['net']].get('art') in ('strom', 'versorgung'))}
        n_lagen = int(K.get('lagen') or (R.get('optionen') or {}).get('lagen') or (K.get('regeln') or {}).get('lagen') or B.get('lagen') or 2)
        LAGENAUFBAU = kennzahl(B, R, K, gndlagen=gndlagen)


        ebenen = LAGENAUFBAU['gnd_kupferlagen']


        if not GND: ebenen = [L for L, nn in FLMAP.items() if not nn]


        unknown = versorgungsflaechen - set(N)
        if unknown: raise ValueError('Unbekannte Flächennetze: %s' % sorted(unknown))
        FR = kupfer(B, R, K, gndlagen=gndlagen) if FLMAP else R
    else:
        versorgungsflaechen = set(R.get('flaechennetze') or []) - GND
        n_lagen = int(K.get('lagen') or (R.get('optionen') or {}).get('lagen') or (K.get('regeln') or {}).get('lagen') or B.get('lagen') or 2)
    planes = {}
    if review and not la_aktiv:
        from analyse.kupfermodell import masse
        planes, _ = masse(B, R, K, gndlagen=gndlagen)
        ebenen = list(planes)
    if not la_aktiv and not review:
        raw = (R.get('optionen') or {}).get('gndlagen') or ''
        label = [x for x in raw.split(',') if x] if isinstance(raw, str) else list(raw)
        label += list(R.get('gndlagen') or [])
        lage_netz = dict(str(x).split('=', 1) for x in label if '=' in str(x))
        gl = gndlagen if gndlagen is not None else (R.get('gndlagen') or [])
        if isinstance(gl, str): gl = [x for x in gl.split(',') if x]
        ebenen = []
        for x in gl:
            teile = str(x).split('=', 1); lage = teile[0]
            n = teile[1] if len(teile) == 2 else lage_netz.get(lage)

            if n is None or all(x in GND or '/' + x in GND for x in n.split('|')): ebenen.append(lage)
    VG = K.get('vorgaben') or {}
    A = aufbau(n_lagen, ebenen, diel=(VG.get('aufbau') or {}).get('dielektrika'), er=float((VG.get('aufbau') or {}).get('er', ER)))
    if la_aktiv and VG.get('aufbau'):
        from analyse.aufbau import normalisieren
        explizit = normalisieren(VG['aufbau'], n_lagen)
        A = aufbau(n_lagen, ebenen, diel=explizit['dielektrika'], er=float((VG['aufbau']).get('er', ER)))
        for L, t in zip(A['stapel'], explizit['kupfer_mm']): A['lagen'][L]['t'] = t
    A['flaechennetze'] = planes
    if review and not la_aktiv:
        from analyse.strombreite import kupferdicke
        for Lg, g in A['lagen'].items(): g['t'] = kupferdicke(VG, innen=Lg not in ('F', 'B'))
    via_d, drill = _via_geo(R, K)
    if AU is None:
        from gemeinsam.laenge import aussen
        AU = aussen(R, B)
    tr = collections.defaultdict(list); vi = collections.defaultdict(list); pads = collections.defaultdict(list)
    for ti, t in enumerate(R.get('tracks', [])): tr[t['net']].append(ti)
    for v in R.get('vias', []): vi[v['net']].append(v)
    for p in B['pads']:
        if p['net']: pads[p['net']].append(p)
    gnd_xy = np.array([(v['x'], v['y']) for v in R.get('vias', []) if v['net'] in GND] +
                      [(p['x'], p['y']) for p in B['pads'] if p.get('drill') and p['net'] in GND] or np.zeros((0, 2)))
    RW, _ = rueckweg(B, FR, A, GND, via_d, drill, flaechen=la_aktiv, K=K)
    from analyse.kopplung import johnson
    tr_vg = (K.get('vorgaben') or {}).get('tr_ns') or {}
    def tr_von(n):
        if si_aktiv:
            from analyse.signalintegritaet import vorgabe, zahl
            standard = (G['klassen'].get(klasse(N.get(n)), {}) or {}).get('tr_ns')
            raw = vorgabe(K, n, 'tr_ns', standard)
            return None if raw is None else zahl(raw, standard, 1e-6)
        b = n.split('/')[-1]
        for rx, v in sorted(tr_vg.items()):
            try:
                if b == rx or n == rx or re.fullmatch(rx, b): return float(v) if v is not None else None
            except re.error: pass
        if (N.get(n) or {}).get('tr_ns') is not None: return float(N[n]['tr_ns'])
        return (G['klassen'].get(klasse(N.get(n)), {}) or {}).get('tr_ns')
    from analyse.stoerer import flanken
    tr_ns = {n: tr_von(n) for n in set(N) | set(tr)} if si_aktiv or not review else flanken(K, G if gewichte is not None or 'stoerer' not in K else None)
    XT = johnson(R, K, A, tr_ns, GND)
    sw_netze = {n for n, k in N.items() if k.get('art') == 'sw'}
    IRd = dict((ir or {}).get('max_abfall_mV') or {}); IRp = dict((ir or {}).get('verlust_je_netz_mW') or {})
    IRi = dict((ir or {}).get('strom_je_netz_A') or {})
    plane_ir = {}
    if la_aktiv and versorgungsflaechen:
        from codex.zonen_ir.solver_fast import ir_kennzahlen
        KP = dict(K, netze={n: N[n] for n in sorted(versorgungsflaechen)})


        stack = {L: dict(copper_um=g['t'] * 1000, z_mm=g['z']) for L, g in A['lagen'].items()}
        for L, v in (B.get('stackup') or {}).items(): stack.setdefault(L, {}).update(v)
        BI = dict(B, stackup=stack)
        plane_ir = ir_kennzahlen(BI, FR, KP)['netze']
        missing = versorgungsflaechen - set(plane_ir)
        if missing: raise ValueError('IR-Flächenmodell fehlende Abdeckung: %s' % sorted(missing))
        bad = {n: e.get('error') for n, e in plane_ir.items() if e.get('status') not in ('solved', 'keine_last')}
        if bad: raise ValueError('IR-Flächenmodell unvollständig: %s' % bad)
        for n, e in plane_ir.items():
            IRd[n] = 1000 * float(e.get('max_drop_V') or 0)
            IRp[n] = 1000 * float(e.get('power_W') or 0)
            IRi[n] = float(e.get('total_current_A') or _strom(n, K)[0])
    BU, BG = budgets(K)
    out = {}; stellen = []
    for n in sorted(set(tr) | set(vi) | set(pads)):
        if n in GND or n not in N: continue
        kl = klasse(N[n]); C = G['klassen'].get(kl)
        if C is None: continue
        l = 0.0; L_nH = 0.0; C_pF = 0.0; T_ps = 0.0; R_ohm = 0.0; A_mm2 = 0.0; je_lage = collections.defaultdict(float)
        L_rp_hoehe = 0.0
        enden = collections.defaultdict(list)
        for ti in tr[n]:
            t = R['tracks'][ti]; Lg = t['layer']
            if Lg not in A['lagen']: continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                d = AU[ti][si]
                Z, tpd, Lp, Cp = leitung(A, Lg, w)
                if VG.get('rueck_hoehe'):
                    gut = [leitung(A, Lb, w)[2] for Lb, gb in A['lagen'].items() if gb['typ'] not in ('ebene', 'offen')]

                    L_rp_hoehe += d * max(0.0, Lp - min(gut, default=0.0))
                l += d; L_nH += Lp * d; C_pF += Cp * d; T_ps += tpd * d; je_lage[Lg] += d
                if not t.get('sig'): R_ohm += RHO * d / (w * A['lagen'][Lg]['t'])
                A_mm2 += w * math.dist(a, b)
            enden[Lg] += [t['pts'][0], t['pts'][-1]]

        n_via = len(vi[n]); rv_nH = 0.0; wechsel = []; via_L = 0.0; via_C = 0.0
        s_ok = (C.get('s') or {}).get('rueck_via_mm', 1.27)
        for v in vi[n]:
            gen = {Lg for Lg, E in enden.items() if any(abs(e[0] - v['x']) < via_d / 2 + 0.05 and abs(e[1] - v['y']) < via_d / 2 + 0.05 for e in E)}
            for p in pads[n]:
                bb = p.get('bbox')
                if bb and bb[0] - 0.01 <= v['x'] <= bb[2] + 0.01 and bb[1] - 0.01 <= v['y'] <= bb[3] + 0.01: gen |= set(p['layers'])
            gen |= {L for L, nn in FLMAP.items() if n in nn and (not v.get('layers') or L in v['layers'])}
            gen = {g for g in gen if g in A['lagen']}
            zs = [A['lagen'][g]['z'] for g in gen]
            h = (max(zs) - min(zs)) if len(zs) >= 2 else A['dicke']
            lv, cv = via_lc(h, drill, A, via_d); via_L += lv; via_C += cv
            refs = sorted({A['lagen'][g]['ref'] for g in gen if A['lagen'][g]['ref']})
            if len(refs) >= 2:
                h_ref = abs(A['lagen'][refs[0]]['z'] - A['lagen'][refs[-1]]['z'])
                s = float(np.min(np.hypot(gnd_xy[:, 0] - v['x'], gnd_xy[:, 1] - v['y']))) if len(gnd_xy) else S_MAX
                s = min(s, S_MAX)
                zus = 0.4 * h_ref * math.log(s / s_ok) if s > s_ok else 0.0
                rv_nH += zus; wechsel.append(dict(x=v['x'], y=v['y'], s_mm=round(s, 2), zus_nH=round(zus, 3)))
        if n_via:
            R_ohm += n_via * RHO * A['dicke'] / (math.pi * (drill + VIA_HUELSE) * VIA_HUELSE)
        L_nH += via_L; C_pF += via_C
        if kupfer_modell and n in sw_netze:
            A_mm2 = R.get('leistungsflaechen_mm2', {}).get(n, A_mm2)
        rw = RW.get(n, dict(l_ohne=0.0, l_rand=0.0, n_spalt=0, l_unbekannt=0.0))
        L_ideal = (L_nH - via_L) / l if l > 0 else 0.0
        L_rp = max(0.0, L_FREI - L_ideal) * (rw['l_ohne'] + rw['l_rand']) + rv_nH + SPALT_NH * rw['n_spalt']
        L_rp += L_rp_hoehe
        tpd_m = T_ps / l if l > 0 else 6.0
        tr_ = tr_ns.get(n)
        l_krit = (1000 * tr_ / (6 * tpd_m)) if tr_ else (C.get('s') or {}).get('laenge_mm')
        luft = _mst([(p['x'], p['y']) for p in pads[n]])
        fl = N[n].get('fluss') or {}
        traegt = bool(fl) or n in sw_netze or N[n].get('art') in ('strom', 'versorgung')

        if not traegt: I, I_q = 0.0, None
        else: I, I_q = _strom(n, K)

        baum = None; P_mW = None; j_x = None
        if traegt and I > 0 and tr[n]:
            tl = [R['tracks'][ti] for ti in tr[n] if not R['tracks'][ti].get('sig') and R['tracks'][ti]['layer'] in A['lagen']]
            baum = strombaum(tl, vi[n], pads[n], fl, A, drill, I, graph=fluss_cache.graph(tl, vi[n], pads[n], TOL) if fluss_cache is not None else None) if tl else None
            if baum:
                if kupfer_modell:

                    baum['seg'] = [tuple(list(s[:5]) + [tl[s[0]].get('kupfer_j_min', tl[s[0]]['w'])[s[1]]] + list(s[6:]))
                                   for s in baum['seg']]
                j_x, st = stromdichte(baum['seg'], A, n, pads[n], ipc_modell=VG.get('ipc_innen_modell')); stellen += st; P_mW = 1000 * baum['P']
        if kupfer_modell and traegt and I > 0:
            from analyse.kupfermodell import stromdichte_flaechen
            j_x, st = stromdichte_flaechen(tl if tr[n] else [], baum, A, n, pads[n], strom_A=I, ipc_modell=VG.get('ipc_innen_modell')); stellen += st
        if si_aktiv and traegt and I > 0 and j_x is None:

            seg_frei = [[ti, si, t['layer'], a, b, w, math.dist(a, b), I, A['lagen'][t['layer']]['t']]
                        for ti in tr[n] for t in [R['tracks'][ti]] if not t.get('sig') and t['layer'] in A['lagen']
                        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))]
            j_x, st = stromdichte(seg_frei, A, n, pads[n], ipc_modell=VG.get('ipc_innen_modell')); stellen += st

        f_ir = (I / IRi[n]) if (n in IRi and IRi[n] > 0 and traegt) else 1.0
        if n in IRd: dV, dV_q = IRd[n] * f_ir, ('ir_flaeche' if n in plane_ir else 'ir') if abs(f_ir - 1) < 1e-9 else 'ir*%.3g' % f_ir
        elif baum: dV, dV_q = 1000 * baum['dV'], 'baum(%s)' % I_q
        elif traegt: dV, dV_q = 1000 * I * R_ohm, 'R*I(%s)' % I_q
        else: dV, dV_q = 0.0, 'kein Strom'
        if n in IRp: P_mW = IRp[n] * f_ir * f_ir
        elif P_mW is None and traegt: P_mW = 1000 * I * I * R_ohm
        if kupfer_modell and n in IRp and I > 0:
            R_ohm = P_mW / (1000 * I * I)
        if r36 and traegt:
            j_x = (ir.get('J_x') or {}).get(n)

        U, U_q = spannung_mit_quelle(n, K)
        if n in BU: s_dc, s_q = BU[n], 'budget'
        elif U and traegt: s_dc, s_q = max(10.0, 10.0 * abs(U)), '1%%_von_%gV' % U
        else: s_dc, s_q = (C.get('s') or {}).get('dc_mV'), 'klasse'
        gruppe = next((g for g, (nn, _mv) in BG.items() if n in nn), None)
        m = dict(laenge_mm=round(l, 2), luftlinie_mm=round(luft, 2), vias=n_via, L_nH=round(L_nH, 3), C_pF=round(C_pF, 3),
                 laufzeit_ps=round(T_ps, 1), l_krit_mm=round(l_krit, 1) if l_krit else None, R_mOhm=round(1000 * R_ohm, 2),
                 dV_mV=round(dV, 6 if si_aktiv else 2), dV_quelle=dV_q, I_A=round(I, 4) if traegt else None,
                 spannung_V=U, spannung_quelle=U_q,
                 I_quelle=(N[n].get('strom_A_quelle') if I_q == 'schaetzung' else I_q),
                 **({'J_max_A_mm2':(ir.get('J_max_A_mm2') or {}).get(n)} if r36 and ir else {}),
                 P_mW=round(P_mW, 6 if si_aktiv else 3) if P_mW is not None else None, J_x=round(j_x, 3) if j_x is not None else None,
                 s_dc_mV=round(s_dc, 2) if s_dc else None, s_dc_quelle=s_q if traegt else None, gruppe=gruppe,
                 **(dict(l_bezug_unbekannt_mm=round(rw.get('l_unbekannt', 0), 2), bezug_status='unbekannt' if rw.get('l_unbekannt', 0) > 0 else ('angeschlossen' if A['ebenen'] else 'offen')) if review else {}),
                 l_ohne_mm=round(rw['l_ohne'], 2), l_rand_mm=round(rw['l_rand'], 2), n_spalt=rw['n_spalt'], rueck_via_nH=round(rv_nH, 3),
                 lagenwechsel=len(wechsel), L_rp_nH=round(L_rp, 3), xt_opfer=round(XT['opfer'].get(n, 0.0), 4),
                 xt_stoer=round(XT['stoer'].get(n, 0.0), 4), sw_flaeche_mm2=round(A_mm2, 2) if n in sw_netze else None,
                 traegt=traegt, tr_ns=tr_, je_lage={k: round(v, 2) for k, v in sorted(je_lage.items())})
        if minimax:
            from analyse.minimax import lastvertrag
            m.update(sicherheit_roh=dict(I_A=I, J_x=j_x, dV_mV=dV, s_dc_mV=s_dc),
                     via_I_A=baum['via_I'] if baum else None,
                     lastvertrag=lastvertrag(n, K, I, I_q, ir) if traegt else None)
        out[n] = dict(klasse=kl, rang=N[n].get('rang') or 0, m=m, wechsel=wechsel[:5])

    HL = hot_loops(B, R, K, A, GND, drill)
    for h in HL:
        z = h['sw'] if h['sw'] in out else h['vin']
        if z in out:
            m = out[z]['m']
            if m.get('hot_L_nH') is None or h['L_nH'] > m['hot_L_nH']: m.update(hot_L_nH=h['L_nH'], hot_A_mm2=h['A_mm2'], hot_regler=h['chip'])

    MK = masse_kopplung(B, R, K, A, GND)
    if MK:
        for n, e in MK['je_netz'].items():
            if n in out: out[n]['m']['masse_uV'] = e

    from analyse.signalintegritaet import augen, vorgabe, zahl
    from analyse.kopplung import signal_kopplung
    AX, AS, AP, SI = {}, {}, [], {}
    if si_aktiv:
        SX, AS, AP = signal_kopplung(B, R, K, A, out, tr_ns)
        AX, SI = augen(B, R, K, A, out, G, SX)
        for n, e in AX.items():
            out[n]['m'].update(auge_v_pct=e['vertikal_pct'], auge_h_pct=e['horizontal_pct'], auge_skew_x=e.get('skew_x', 0.0))
        for n, e in AS.items(): out[n]['m'].update(stoer_mV=e['stoer_mV'], stoerbudget_mV=e['budget_mV'])
        for n, e in out.items():
            m = e['m']
            if m.get('traegt'):
                m['s_verlust_mW'] = zahl(vorgabe(K, n, 'verlustbudget_mW'),
                                         max(1e-9, (m.get('I_A') or 0) * (m.get('s_dc_mV') or 30)), 1e-9)
    gruppen = {}
    for g, (nn, mv) in BG.items():
        gruppen[g] = dict(netze=nn, mV=mv, summe_mV=round(sum(out[x]['m']['dV_mV'] for x in nn if x in out), 6 if si_aktiv else 3),
                          quellen=sorted({out[x]['m']['dV_quelle'].split('(')[0] for x in nn if x in out}))
        if minimax: gruppen[g]['summe_roh_mV'] = sum(out[x]['m']['sicherheit_roh']['dV_mV'] for x in nn if x in out)
    stellen.sort(reverse=True)
    return dict(version=G.get('version', 'v3.1'), **({'modellrevision': VERSION} if review else {}), augen=AX, schnittstellen=SI, analog_stoer=AS, stoer_paare=AP, **(dict(lagenaufbau=LAGENAUFBAU, flaechen_ir={n: {k: e.get(k) for k in ('status', 'strommodus', 'total_current_A', 'nodes', 'warnungen', 'annahmen', 'kupferflaeche_je_lage_mm2')} for n, e in plane_ir.items()}, flaechenteilung=dict(modell='Voronoi-Anschlüsse; geschätzt, kein DRC', zonen=len(FR.get('zones', [])))) if la_aktiv else {}), netze=out, xt_paare=XT['paare'][:30], hot_loop=HL, masse=MK, gruppen=gruppen, strom_stellen=stellen[:20],
                **({'kupfer_ir': ir, 'kupfer_modell': 'r36-dc-v1' if r36 else 'kupfer-v1'} if kupfer_modell else {}),
                verlust_mW=round(sum(v['m']['P_mW'] or 0.0 for v in out.values()), 3),
                versorgungsflaechen=sorted(versorgungsflaechen), masseflaechen=sorted(({n for nn in FLMAP.values() for n in nn} if la_aktiv else set(R.get('flaechennetze') or [])) & GND),
                aufbau=dict(stapel=A['stapel'], ebenen=A['ebenen'], dielektrika=A['dielektrika'],
                            lagen={k: {q: (round(w, 4) if isinstance(w, float) else w) for q, w in v.items()} for k, v in A['lagen'].items()}))




def netz_x(m, C, sw):
    ""
    s = C.get('s') or {}; x = {}
    tr_ = m['tr_ns'] if 'tr_ns' in m else C.get('tr_ns')
    s_rueck = 10 * tr_ if tr_ else s.get('rueck_nH')
    if s_rueck: x['rueck'] = m['L_rp_nH'] / s_rueck
    if m.get('l_krit_mm'): x['laenge'] = m['laenge_mm'] / m['l_krit_mm']
    if s.get('c_last_pF'): x['c_last'] = m['C_pF'] / s['c_last_pF']
    if s.get('l_last_nH'): x['l_last'] = m['L_nH'] / s['l_last_nH']
    if s.get('vias'): x['vias'] = m['vias'] / s['vias']
    if s.get('xt_opfer'): x['xt_opfer'] = m['xt_opfer'] / s['xt_opfer']
    if s.get('xt_stoer'): x['xt_stoer'] = m['xt_stoer'] / s['xt_stoer']
    if m.get('traegt') and m.get('s_dc_mV') and not m.get('gruppe'): x['dc'] = m['dV_mV'] / m['s_dc_mV']
    if m.get('traegt') and m.get('J_x') is not None: x['j'] = m['J_x']
    if s.get('sw_flaeche_mm2') and sw and m.get('sw_flaeche_mm2') is not None: x['sw_flaeche'] = m['sw_flaeche_mm2'] / s['sw_flaeche_mm2']
    if s.get('umweg_mm'): x['umweg'] = max(0.0, m['laenge_mm'] - m['luftlinie_mm']) / s['umweg_mm']
    if m.get('hot_L_nH') is not None and s.get('hot_L_nH'): x['hot_L'] = m['hot_L_nH'] / s['hot_L_nH']
    if m.get('hot_A_mm2') is not None and s.get('hot_A_mm2'): x['hot_A'] = m['hot_A_mm2'] / s['hot_A_mm2']
    if m.get('masse_uV') is not None and s.get('masse_uV'): x['masse'] = m['masse_uV'] / s['masse_uV']
    if m.get('auge_v_pct') is not None and s.get('auge_v_verlust_pct'):
        x['auge_v'] = (100 - m['auge_v_pct']) / s['auge_v_verlust_pct']
    if m.get('auge_h_pct') is not None and s.get('auge_h_verlust_pct'):
        x['auge_h'] = (100 - m['auge_h_pct']) / s['auge_h_verlust_pct']
    if m.get('auge_skew_x') is not None: x['auge_skew'] = m['auge_skew_x']
    if m.get('stoer_mV') is not None: x['stoer'] = m['stoer_mV'] / m['stoerbudget_mV']
    if m.get('traegt') and m.get('s_verlust_mW') and m.get('P_mW') is not None:
        x['leistung'] = m['P_mW'] / m['s_verlust_mW']
    return x


def bewerten(M, gewichte=None):
    ""
    G = gewichte or lade_gewichte(); knick = float(G.get('knick', 4.0)); wr = float(G.get('w_rang', 0.5))
    kj = G.get('knick_je') or {}
    for n, v in M['netze'].items():
        C = G['klassen'].get(v['klasse']) or {}
        g = C.get('g') or {}
        x = netz_x(v['m'], C, v['klasse'] == 'sw')
        teile = {k: g.get(k, 0) * strafe(xv, float(kj.get(k, knick))) for k, xv in sorted(x.items()) if g.get(k, 0)}
        Nn = sum(teile.values()); W = 1 + wr * (v['rang'] or 0)
        v.update(note=round(Nn, 3), gewicht=W, beitrag=round(W * Nn, 3), x={k: round(xv, 3) for k, xv in x.items()},
                 teile={k: round(t, 3) for k, t in teile.items()})
    bt = {}
    gg = (G.get('gruppen') or {}); xs = []
    for name, gr in sorted((M.get('gruppen') or {}).items()):
        if not gr.get('mV'): continue
        xs.append(gr['summe_mV'] / gr['mV'])
        if gg.get('g'): bt['kanal:' + name] = round(float(gg['g']) * strafe(xs[-1], float(kj.get('dc', knick))), 3)
    if xs and gg.get('g_max'):
        bt['kanal_max'] = round(float(gg['g_max']) * strafe(max(xs), float(kj.get('dc', knick))), 3)

    if G.get('lagenaufbau', 0): bt['lagenaufbau'] = float(G['lagenaufbau']) * M.get('lagenaufbau', {}).get('summe', 0)
    M['board_teile'] = bt
    M['note_v3'] = round(sum(v['beitrag'] for v in M['netze'].values()) + sum(bt.values()), 2)
    if 'signalintegritaet' in M:
        M['note_v3_basis'] = M['note_v3']
        if M.get('si_note_aktiv'):
            M['note_v3'] = round(M['note_v3'] + M['signalintegritaet']['note_zusatz'], 2)
    if 'minimax_version' in M:
        from analyse import minimax as MM
        MM.bewerten(M, G, M['minimax_profil'])
    return M


def metriken(B, R, K, ir=None, gewichte=None, gndlagen=None, AU=None, fluss_cache=None, minimax=None, si_note=False, si_messen=False):
    ""
    if (K.get('vorgaben') or {}).get('physik_version') == 'v3.3-vertrag-option':
        from analyse.metriken_physik import metriken as physik
        return physik(B, R, K, ir=ir, gewichte=gewichte, gndlagen=gndlagen, AU=AU,
                      fluss_cache=fluss_cache, minimax=minimax, si_note=si_note, si_messen=si_messen)
    t0 = time.time(); c0 = time.process_time()
    G = gewichte or lade_gewichte(si=(K.get('vorgaben') or {}).get('si_bewertung') is True)
    if minimax is None:
        from analyse.standard import ziel
        minimax = ziel(K.get('vorgaben')) == 'minimax'
    M = messen(B, R, K, ir=ir, gewichte=G, gndlagen=gndlagen, AU=AU, fluss_cache=fluss_cache, minimax=minimax)
    if (si_messen or si_note) and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('laengengruppe') for k in K['netze'].values()) or (K.get('vorgaben') or {}).get('laengengruppen')):
        from analyse.laengenimpedanz import messen as si_messen
        SR = dict(R, gndlagen=gndlagen) if gndlagen is not None else R
        M['signalintegritaet'] = si_messen(B, SR, K)
        M['si_note_aktiv'] = bool(si_note)
        if si_note: M['version'] += '+r35-si2-option'
    bewerten(M, G)
    if minimax:
        from analyse import minimax as MM
        MM.messen_zusatz(M, B, R, K, G, ir=ir)
        MM.bewerten(M, G, (K.get('vorgaben') or {}).get('profil', 'ausgewogen'))
    M['zeit_s'] = round(time.time() - t0, 3); M['cpu_s'] = round(time.process_time() - c0, 3)
    return M


def tabelle(M, n=10):
    z = []
    if 'signalintegritaet' in M:
        z.append('  SI-Zusatznote %.3f (%s; heuristisch, unkalibriert)' % (
            M['signalintegritaet']['note_zusatz'], 'aufgeschlagen' if M.get('si_note_aktiv') else 'separat, si_note=1 zum Aufschlagen'))
    for name, v in sorted(M['netze'].items(), key=lambda kv: -kv[1]['beitrag'])[:n]:
        m = v['m']; top = sorted(v['teile'].items(), key=lambda kv: -kv[1])[:3]
        z.append('  %-22s %-10s %6.2f  [%s]  l %.1f/%s mm, Vias %d, L %.2f nH, C %.2f pF, L_rp %.2f nH (ohne %.1f, Rand %.1f, Spalt %d, Wechsel %d), '
                 'XT %.3f/%.3f, R %.1f mOhm, dV %.2f mV%s' % (
                     name.split('/')[-1][:22], v['klasse'], v['beitrag'], ', '.join('%s %.2f (x %.2f)' % (k, w, v['x'][k]) for k, w in top),
                     m['laenge_mm'], m['l_krit_mm'], m['vias'], m['L_nH'], m['C_pF'], m['L_rp_nH'], m['l_ohne_mm'], m['l_rand_mm'], m['n_spalt'],
                     m['lagenwechsel'], m['xt_opfer'], m['xt_stoer'], m['R_mOhm'], m['dV_mV'],
                     ', P %.1f mW, J x %.2f' % (m['P_mW'] or 0, m['J_x'] or 0) if m.get('traegt') else ''))
        if m.get('auge_v_pct') is not None:
            z[-1] += ' | Auge V/H %.1f/%.1f %% (Modell)' % (m['auge_v_pct'], m['auge_h_pct'])
        if m.get('stoer_mV') is not None:
            z[-1] += ' | Analog %.4f / %.4f mV (Modell)' % (m['stoer_mV'], m['stoerbudget_mV'])
    for h in M.get('hot_loop') or []:
        z.append('  Hot Loop %-8s %s -> %s:%s -> %s:%s: L %.2f nH (VIN %.2f, %.1f mm, %d Vias, %s; PGND %s; C-Masse %s), Flaeche %.1f mm2' % (
            h['chip'], h['c_in'], h['chip'], h['vin_pin'], h['chip'], h['pgnd_pin'], h['L_nH'], h['L_vin_nH'], h['l_vin_mm'], h['vias_vin'],
            h['vin_weg'], h['pgnd'], h['cmasse'], h['A_mm2']))
    if M.get('masse'):
        mk = M['masse']
        z.append('  Masse-Kopplung max %.2f uV (Bezug %s, %d Angreifer): %s' % (mk['max_uV'], mk['bezug'], len(mk['angreifer']),
                 ', '.join('%s %.2f' % (k.split('/')[-1], v) for k, v in sorted(mk['je_netz'].items(), key=lambda kv: -kv[1])[:6])))
    for k, v in sorted((M.get('board_teile') or {}).items()):
        if ':' in k:
            g = M['gruppen'].get(k.split(':', 1)[1]) or {}
            z.append('  %-12s %6.2f  (Summe %.1f von %.0f mV)' % (k, v, g.get('summe_mV', 0), g.get('mV', 0)))
        else:
            z.append('  %-12s %6.2f  (schlechteste Kanalgruppe)' % (k, v))
    for s in (M.get('strom_stellen') or [])[:3]:
        z.append('  Stromdichte %s auf %s bei (%.1f, %.1f): %.2f mm statt %.2f mm fuer %.2f A (J %.0f A/mm2, %.1f mm lang), x %.2f' % (
            s[1].split('/')[-1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[0]))
    return '\n'.join(z)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    B = json.load(open(a[0])); R = json.load(open(a[1])); K = json.load(open(a[2]))
    if 'lagenaufbau' in kw:
        K = dict(K, vorgaben=dict(K.get('vorgaben') or {}, lagenaufbau=kw['lagenaufbau'] == '1'))
    M = metriken(B, R, K, gndlagen=kw['gndlagen'].split(',') if 'gndlagen' in kw else None, si_note=kw.get('si_note') == '1', si_messen=kw.get('si_messen') == '1')
    print('Note v3 %.1f (%d Netze, %.3f s CPU); Aufbau %s, Ebenen %s' % (M['note_v3'], len(M['netze']), M['cpu_s'],
          '/'.join('%.3f' % d for d in M['aufbau']['dielektrika']), ','.join(M['aufbau']['ebenen'])))
    print(tabelle(M, int(kw.get('n', '10'))))
    if 'json' in kw: json.dump(M, open(kw['json'], 'w'), indent=1)
