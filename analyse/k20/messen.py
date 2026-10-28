""
import json, sys, re, math, os, time, collections
import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

D, NAME, ART = sys.argv[1:4]
T0 = time.time()
C = json.load(open(os.path.join(D, 'cu.json')))
Z = json.load(open(os.path.join(D, 'zusatz.json')))
K = json.load(open(os.path.join(D, 'klassen.json'))) if os.path.exists(os.path.join(D, 'klassen.json')) else None
LAGEN = Z['layers']
AUSSEN = {'F', 'B'}
TOL = 0.03
CLR_EINHEIT = 0.15
CLR_EIGEN = {'vorbild': 0.10, 'nativ': 0.10}.get(ART, float(Z.get('min_clearance') or 0.15))
GLATT = 0.05
ECKE_MIN = math.radians(30)
R_RUND, EPS = 0.10, 0.012
MITTE = 0.01


def geo(polys):
    ps = []
    for o, hs in polys:
        if len(o) < 3: continue
        p = Polygon(o, [h for h in hs if len(h) >= 3])
        if not p.is_valid: p = p.buffer(0)
        ps.append(p)
    return unary_union(ps) if ps else Polygon()


def teile(g):
    return [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and not p.is_empty]



GND_RE = re.compile(r'(^|[/_\-])(A|D|P)?GND\w*$|(^|[/_\-])GND[AD]?$|^VSS$', re.I)
VERS_TOK = re.compile(r'^(\d+V\d*[A-Z]?|V(CC|DD|BUS|IN|AA|DC|REF|CORE|LO|SYS|BAT|IO|CAP|PP|MOT|LED)\w*|AVDD\w*|DVDD\w*|IOVDD\w*|PVDD\w*|DREG|AREG|SW|BST|BOOT)$', re.I)
SIG_TOK = re.compile(r'^(PWM|RXD?\d*|TXD?\d*|I?SENSE|SNS|EN|FB|NC\d*|SSCAP|PG|OUT|IN\d+|SDA|SCL|CLK|DATA)$', re.I)
def vers_name(n):
    if n.lower().startswith('unconnected'): return False
    blatt = n.split('/')[-1]
    m = re.match(r'^Net-\((.*)\)$', blatt)
    if m: blatt = m.group(1).split('-')[-1]
    tok = [t for t in re.split(r'[_\-+/() ]+', blatt) if t]
    return any(VERS_TOK.match(t) for t in tok) and not any(SIG_TOK.match(t) for t in tok)
netze = sorted(set(C['nets']) | {t['net'] for t in Z['tracks']})
gnd = {n for n in netze if n and GND_RE.search(n)}
if K: gnd |= set(K.get('gnd') or [])
zonennetze = {z['net'] for z in Z['zones'] if z['fill']}
vers = {n for n in netze if n and n not in gnd and (vers_name(n) or n in zonennetze)}
if K: vers |= {n for n, v in (K.get('netze') or {}).items() if v.get('art') in ('strom', 'versorgung', 'sw') and n not in gnd}
art = lambda n: 'gnd' if n in gnd else 'vers' if n in vers else 'sig'


outline = geo(Z['outline'])
if ART == 'vorbild':
    ebenen = unary_union([geo(jl[L]) for n, jl in C['nets'].items() for L in ('In1', 'In6') if L in jl])
    voll = unary_union([Polygon(p.exterior) for p in teile(ebenen) if p.area > 50])
    outline = outline.intersection(voll.buffer(0.45, 16))
sperr_quellen = list(Z['keepouts'])
if not sperr_quellen and os.path.exists(os.path.join(D, 'board.json')):
    BJ = json.load(open(os.path.join(D, 'board.json')))
    for k in BJ.get('keepouts', []):
        lo = k.get('loecher') or []
        sperr_quellen.append(dict(layers=k.get('lagen') or LAGEN, kein_kupfer=bool((k.get('verbote') or {}).get('flaechen')),
                                  polys=[[q, lo[i] if i < len(lo) else []] for i, q in enumerate(k.get('polys', []))]))
sperr = {L: unary_union([geo(k['polys']) for k in sperr_quellen if k.get('kein_kupfer') and L in k['layers']] or [Polygon()]) for L in LAGEN}
lagenflaeche = {L: outline.difference(sperr[L]) for L in LAGEN}
cu = {L: {} for L in LAGEN}
for n, jl in C['nets'].items():
    for L, ps in jl.items():
        if L in cu:
            g = geo(ps)
            if not g.is_empty: cu[L][n] = g
zf = {}
for z in Z['zones']:
    if z['fill'] and z['layer'] in cu: zf.setdefault((z['net'], z['layer']), []).append(geo(z['fill']))
zf = {k: unary_union(v) for k, v in zf.items()}
bahnen = {}
for t in Z['tracks']:
    a, b = tuple(t['a']), tuple(t['b'])
    g = (LineString([a, b]) if a != b else Point(a)).buffer(t['w'] / 2, 8)
    bahnen.setdefault((t['net'], t['layer']), []).append(g)
bahnen = {k: unary_union(v) for k, v in bahnen.items()}
bohr_pts = [(b['x'], b['y'], b['d']) for b in Z.get('bohrungen', [])]


def ecken(G):
    ""
    out, laenge = [], 0.0
    for p in teile(G):
        p = orient(p, 1.0)
        for ring in [p.exterior] + list(p.interiors):
            laenge += ring.length
            r = ring.simplify(GLATT)
            xy = np.asarray(r.coords)[:-1]
            if len(xy) < 3: continue
            v1 = xy - np.roll(xy, 1, axis=0); v2 = np.roll(xy, -1, axis=0) - xy
            cr = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]; dt = (v1 * v2).sum(1)
            tau = np.arctan2(cr, dt)
            for (x, y), t in zip(xy, tau):
                if abs(t) >= ECKE_MIN: out.append((x, y, t))
    return out, laenge


def eckenmass(G):
    E, laenge = ecken(G)
    if not E: return dict(rand_mm=laenge, ecken=0, spitz_cu=0, spitz_weiss=0, rund=0)
    xs = np.array([e[0] for e in E]); ys = np.array([e[1] for e in E]); ts = np.array([e[2] for e in E])
    auf = G.buffer(-R_RUND, 16).buffer(R_RUND + EPS, 16)
    zu = G.buffer(R_RUND, 16).buffer(-R_RUND - EPS, 16)
    shapely.prepare(auf); shapely.prepare(zu)
    konvex = ts > 0
    rund = np.where(konvex, shapely.contains_xy(auf, xs, ys), ~shapely.contains_xy(zu, xs, ys))
    return dict(rand_mm=laenge, ecken=int(len(E)), spitz_cu=int((ts > math.pi / 2).sum()), spitz_weiss=int((ts < -math.pi / 2).sum()), rund=int(rund.sum()),
                spitz_cu_xy=[(round(x, 3), round(y, 3)) for x, y, t in E if t > math.pi / 2][:400],
                spitz_weiss_xy=[(round(x, 3), round(y, 3)) for x, y, t in E if t < -math.pi / 2][:400])


erg = dict(name=NAME, art=ART, lagen=LAGEN, flaeche_mm2=round(outline.area, 1), sperr_mm2={L: round(sperr[L].area, 1) for L in LAGEN}, clr_eigen=CLR_EIGEN, gnd=sorted(gnd), vers=sorted(vers), je_lage={})
summen = collections.Counter()
for L in LAGEN:
    t = time.time()
    A = lagenflaeche[L].area
    um = lagenflaeche[L]
    g_cu = unary_union([g for n, g in cu[L].items() if n in gnd] or [Polygon()]).intersection(um)
    v_cu = unary_union([g for n, g in cu[L].items() if n in vers] or [Polygon()]).intersection(um)
    s_cu = unary_union([g for n, g in cu[L].items() if art(n) == 'sig'] or [Polygon()]).intersection(um)
    alles = unary_union(list(cu[L].values()) or [Polygon()])
    r = dict(flaeche_mm2=round(A, 1), gnd_anteil=round(g_cu.area / A, 4), vers_anteil=round(v_cu.area / A, 4), sig_anteil=round(s_cu.area / A, 4))

    gz = unary_union([g for (n, LL), g in zf.items() if LL == L and n in gnd] or [Polygon()])
    st = [p for p in teile(g_cu) if p.area >= 0.01 and p.intersects(gz)]
    anker = [Point(v['x'], v['y']) for v in Z['vias'] if v['net'] in gnd and (not v.get('layers') or L in v['layers'])]
    anker += [Point(p_['x'], p_['y']) for p_ in C['pads'] if p_.get('net') in gnd and L in (p_.get('layers') or [])]
    from shapely.strtree import STRtree
    baum = STRtree(anker) if anker else None
    def hat_anker(p):
        return baum is not None and any(p.covers(anker[i]) for i in baum.query(p))
    inseln = [p for p in st if p.area >= 1.0]
    spl = [p for p in st if p.area < 1.0]
    r['gnd_stuecke'] = len(st)
    r['gnd_inseln'] = len(inseln)
    r['gnd_splitter_mit_anschluss'] = sum(hat_anker(p) for p in spl)
    r['gnd_splitter_ohne'] = len(spl) - r['gnd_splitter_mit_anschluss']
    r['gnd_kleinste_insel_mm2'] = round(min((p.area for p in inseln), default=0), 3)
    r['gnd_kleinste_insel_anteil'] = round(min((p.area for p in inseln), default=0) / max(1e-9, sum(p.area for p in st)), 5)

    for tag, clr in (('15', CLR_EINHEIT), ('eigen', CLR_EIGEN)):
        loch = unary_union([Point(x, y).buffer(d / 2 + 0.255 - clr, 12) for x, y, d in bohr_pts] or [Polygon()])
        innen = outline.buffer(-0.5).difference(sperr[L].buffer(clr))
        w = innen.difference(alles.union(loch).buffer(clr + TOL, 12))
        wt = [p for p in teile(w) if p.area > 0.002]
        r['weiss%s_mm2' % tag] = round(sum(p.area for p in wt), 3); r['weiss%s_n' % tag] = len(wt)
        if tag == '15': r['weiss_xy'] = [(round(p.centroid.x, 3), round(p.centroid.y, 3), round(p.area, 4)) for p in wt]

    vz = []
    for (n, LL), f in sorted(zf.items()):
        if LL != L or n not in vers: continue
        own = cu[L].get(n, Polygon())
        zc = f.union(own)
        stk = [p for p in teile(f) if p.area >= 0.01]
        mit = [c for c in teile(zc) if c.intersects(f)]
        loch_kl = sum(Polygon(i).area for c in mit for i in c.interiors if Polygon(i).area < 3)
        vz.append(dict(net=n, stuecke=len(stk), teile_mit_eigen=len(mit), splitter=sum(p.area < 0.5 for p in stk), flaeche=round(f.area, 3),
                       stueck_mm2=sorted(round(p.area, 3) for p in stk), loch_mm2=round(loch_kl, 3), kupfer_mm2=round(sum(c.area for c in mit), 3)))
    r['vers_zonen'] = vz

    vzone = unary_union([f for (n, LL), f in zf.items() if LL == L and n in vers] or [Polygon()])
    vbahn = unary_union([g for (n, LL), g in bahnen.items() if LL == L and n in vers] or [Polygon()])
    r['vers_zone_mm2'] = round(vzone.area, 2); r['vers_bahn_mm2'] = round(vbahn.difference(vzone).area, 2)

    for klasse, menge in (('gnd', gnd), ('vers', vers)):
        Gs = []
        for (n, LL), f in zf.items():
            if LL != L or n not in menge: continue
            zc = f.union(cu[L].get(n, Polygon()))
            Gs += [c for c in teile(zc) if c.intersects(f)]
        G = unary_union(Gs) if Gs else Polygon()
        r['ecken_' + klasse] = eckenmass(G) if not G.is_empty else dict(rand_mm=0, ecken=0, spitz_cu=0, spitz_weiss=0, rund=0)
    r['sek'] = round(time.time() - t, 1)
    erg['je_lage'][L] = r
    print(NAME, L, 'GND %.0f%%' % (100 * r['gnd_anteil']), 'Weiss15 %.2f mm2 (%d)' % (r['weiss15_mm2'], r['weiss15_n']), 'GND-Inseln', r['gnd_inseln'], 'Splitter mit/ohne', r['gnd_splitter_mit_anschluss'], r['gnd_splitter_ohne'],
          'Vers-Zonen', len(vz), 'spitz', r['ecken_gnd']['spitz_cu'] + r['ecken_vers']['spitz_cu'], '%.1fs' % r['sek'], flush=True)


zellen = collections.defaultdict(list)
for i, t in enumerate(Z['tracks']):
    for p in (t['a'], t['b']): zellen[(int(p[0] // 1), int(p[1] // 1))].append(i)
def seg_abst(px, py, a, b):
    ax, ay = a; bx, by = b; dx, dy = bx - ax, by - ay; l2 = dx * dx + dy * dy
    u = 0 if l2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)
anschl = mitte_n = 0; vias_mit = vias_schief = 0; schief_xy = []
for v in Z['vias']:
    r = v['d'] / 2; cx, cy = v['x'], v['y']
    kand = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1): kand.update(zellen.get((int(cx // 1) + dx, int(cy // 1) + dy), []))
    S_ = []
    for i in sorted(kand):
        t = Z['tracks'][i]
        if t['net'] != v['net'] or (v.get('layers') and t['layer'] not in v['layers']): continue
        if any(math.hypot(p[0] - cx, p[1] - cy) < r - 1e-6 for p in (t['a'], t['b'])) or seg_abst(cx, cy, t['a'], t['b']) <= MITTE: S_.append(i)
    if not S_: continue
    eltern = {i: i for i in S_}
    def wurzel(i):
        while eltern[i] != i: eltern[i] = eltern[eltern[i]]; i = eltern[i]
        return i
    knoten = collections.defaultdict(list)
    for i in S_:
        t = Z['tracks'][i]
        for p in (t['a'], t['b']):
            if math.hypot(p[0] - cx, p[1] - cy) > MITTE: knoten[(t['layer'], round(p[0], 4), round(p[1], 4))].append(i)
    for k, ids in knoten.items():
        for j in ids[1:]: eltern[wurzel(j)] = wurzel(ids[0])
    gruppen = collections.defaultdict(list)
    for i in S_: gruppen[wurzel(i)].append(i)
    vias_mit += 1; schief = False
    for g, ids in gruppen.items():
        anschl += 1
        if any(seg_abst(cx, cy, Z['tracks'][i]['a'], Z['tracks'][i]['b']) <= MITTE for i in ids): mitte_n += 1
        else: schief = True
    if schief: vias_schief += 1; schief_xy.append((round(cx, 3), round(cy, 3)))
erg['r37'] = dict(anschluesse=anschl, mitte=mitte_n, anteil=round(mitte_n / max(1, anschl), 4), vias_mit_bahn=vias_mit, vias_schief=vias_schief,
                  vias=len(Z['vias']), schief_xy=schief_xy[:300])


J = erg['je_lage']
def agg(lagen):
    vz = [z for L in lagen for z in J[L]['vers_zonen']]
    ez = {k: sum(J[L]['ecken_vers'][k] + J[L]['ecken_gnd'][k] for L in lagen) for k in ('rand_mm', 'ecken', 'spitz_cu', 'spitz_weiss', 'rund')}
    ev = {k: sum(J[L]['ecken_vers'][k] for L in lagen) for k in ('rand_mm', 'ecken', 'spitz_cu', 'spitz_weiss', 'rund')}
    kup = sum(z['kupfer_mm2'] for z in vz); loch = sum(z['loch_mm2'] for z in vz)
    zo = sum(J[L]['vers_zone_mm2'] for L in lagen); ba = sum(J[L]['vers_bahn_mm2'] for L in lagen)
    return dict(
        gnd_anteil=round(sum(J[L]['gnd_anteil'] * J[L]['flaeche_mm2'] for L in lagen) / max(1e-9, sum(J[L]['flaeche_mm2'] for L in lagen)), 4),
        gnd_min_lage=min((J[L]['gnd_anteil'] for L in lagen), default=0),
        gnd_stuecke=sum(J[L]['gnd_stuecke'] for L in lagen), gnd_inseln=sum(J[L]['gnd_inseln'] for L in lagen), gnd_splitter_mit=sum(J[L]['gnd_splitter_mit_anschluss'] for L in lagen), gnd_splitter_ohne=sum(J[L]['gnd_splitter_ohne'] for L in lagen), gnd_inseln_max_lage=max((J[L]['gnd_inseln'] for L in lagen), default=0), gnd_kleinste_insel_anteil=min((J[L]['gnd_kleinste_insel_anteil'] for L in lagen if J[L]['gnd_inseln']), default=0),
        weiss15_mm2=round(sum(J[L]['weiss15_mm2'] for L in lagen), 2), weiss15_n=sum(J[L]['weiss15_n'] for L in lagen),
        weiss15_je_cm2=round(sum(J[L]['weiss15_mm2'] for L in lagen) / max(1e-9, sum(J[L]['flaeche_mm2'] for L in lagen)) * 100, 4),
        weisseigen_mm2=round(sum(J[L]['weisseigen_mm2'] for L in lagen), 2), weisseigen_n=sum(J[L]['weisseigen_n'] for L in lagen),
        weisseigen_je_cm2=round(sum(J[L]['weisseigen_mm2'] for L in lagen) / max(1e-9, sum(J[L]['flaeche_mm2'] for L in lagen)) * 100, 4),
        vers_netz_lage=len(vz), vers_stuecke=sum(z['stuecke'] for z in vz), vers_einteilig=sum(z['stuecke'] == 1 for z in vz),
        vers_einteilig_mit_eigen=sum(z['teile_mit_eigen'] == 1 for z in vz), vers_splitter=sum(z['splitter'] for z in vz),
        vers_stueck_median_mm2=float(np.median([a for z in vz for a in z['stueck_mm2']])) if vz else 0,
        zone_anteil=round(zo / max(1e-9, zo + ba), 4), zone_mm2=round(zo, 1), bahn_mm2=round(ba, 1),
        lochanteil=round(loch / max(1e-9, kup + loch), 4),
        spitz_cu_je100=round(ez['spitz_cu'] / max(1e-9, ez['rand_mm']) * 100, 2), spitz_weiss_je100=round(ez['spitz_weiss'] / max(1e-9, ez['rand_mm']) * 100, 2),
        rund_anteil=round(ez['rund'] / max(1, ez['ecken']), 4), ecken=ez['ecken'], rand_mm=round(ez['rand_mm'], 1),
        vers_spitz_cu_je100=round(ev['spitz_cu'] / max(1e-9, ev['rand_mm']) * 100, 2), vers_spitz_weiss_je100=round(ev['spitz_weiss'] / max(1e-9, ev['rand_mm']) * 100, 2),
        vers_rund_anteil=round(ev['rund'] / max(1, ev['ecken']), 4), vers_ecken=ev['ecken'], vers_rand_mm=round(ev['rand_mm'], 1))
erg['gesamt'] = agg(LAGEN)
erg['aussen'] = agg([L for L in LAGEN if L in AUSSEN])
erg['innen'] = agg([L for L in LAGEN if L not in AUSSEN])
json.dump(erg, open(os.path.join(D, 'k20.json'), 'w'), indent=1)
g = erg['gesamt']
print(NAME, 'FERTIG %.0fs' % (time.time() - T0), json.dumps({k: g[k] for k in ('gnd_anteil', 'weiss15_mm2', 'weisseigen_mm2', 'gnd_inseln', 'gnd_splitter_mit', 'gnd_splitter_ohne', 'vers_netz_lage', 'vers_stuecke', 'zone_anteil', 'lochanteil', 'spitz_cu_je100', 'rund_anteil')}), 'R37', erg['r37'])
