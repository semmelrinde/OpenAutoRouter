""
import json, sys, os, math, subprocess, collections, time, uuid
from concurrent.futures import ThreadPoolExecutor

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from gemeinsam.prozessschutz import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union
from gemeinsam.verschieben import bewegen
from gemeinsam.teile import lade_teile, platine, Belegung
from analyse.bewertung import bewerte, budgets
from gemeinsam import plaetze as PL
from nachbearbeitung.zugregeln import sperre, annahme, regelwerte, bewegung, routenzug, octi
from nachbearbeitung import versuchslog as VL

VPY = os.path.join(R, '.venv', 'bin', 'python')
D = os.path.abspath(sys.argv[1])
args = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
os.environ['AUTOROUTER_PLAETZE'] = '4'
RUNDEN = int(args.get('runden', '3')); PAR = min(4, max(1, int(args.get('parallel', '4')))); NK = int(args.get('kand', '12'))
ITER = args.get('iter', '15'); MAXS = float(args.get('max_schritt', '0.35')); MIND = float(args.get('mindest', '2.0'))
ZUEGE = set(args.get('zuege', 'schieben,drehen,tauschen').split(',')); TAUSCH_AB = float(args.get('tausch_abstand', '6'))
VERS_N = int(args.get('versetzen_n', '2')); VERS_PL = int(args.get('versetzen_plaetze', '3'))
BUDGET_CPU = float(args.get('budget_cpu', '0'))
def cpu_verbraucht():
    import resource
    a, b = resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)
    return a.ru_utime + a.ru_stime + b.ru_utime + b.ru_stime
def budget_aus(wo):
    if BUDGET_CPU and cpu_verbraucht() > BUDGET_CPU:
        log('   CPU-Budget %.0f s erreicht (%.0f s), %s' % (BUDGET_CPU, cpu_verbraucht(), wo)); return True
    return False
RANKER = args.get('ranker'); RANKER_K = int(args.get('ranker_k', '4')); RANKER_POOL = args.get('ranker_pool', 'wahl')
IR = args.get('ir', '1') == '1'; IR_TOL = float(args.get('ir_schutz', '0.03'))

ZONE_OPT = None if args.get('ir_zonen', '1') == '0' else {
    k: os.environ[k] for k in ('ZONE_EDGE', 'ZONE_GAP', 'ZONE_FAIR', 'ZONE_VORRANG',
                              'ZONE_GND_SIGNAL', 'GND_ALLE', 'FILL_NETZ', 'ZUNGE', 'ZCLR') if k in os.environ}
NOTE = args.get('note', 'v3')
if NOTE not in ('v1', 'v2', 'v3', 'minimax'): raise ValueError('note muss v1, v2, v3 oder minimax sein')
if NOTE == 'v2' and not IR: NOTE = 'v1'
VL.NOTE_KEY = {'v1': 'note', 'v2': 'note_v2', 'v3': 'note_v3', 'minimax': 'minimax'}[NOTE]
def nw(E):
    ""
    return E[VL.NOTE_KEY] if E.get(VL.NOTE_KEY) is not None else float('nan')
def bewertung(B, Rt):
    E = bewerte(B, Rt, K, ir=IR, zonen=ZONE_OPT, v3=True, minimax=True if NOTE == 'minimax' else None)
    E['zug_regeln'] = regelwerte(B, Rt, K)
    return E
K = json.load(open(os.path.join(D, 'klassen.json')))
if NOTE == 'v3' and (K.get('vorgaben') or {}).get('bewertung') == 'minimax':
    NOTE = 'minimax'; VL.NOTE_KEY = 'minimax'
NETZ = K['netze']; TEILE = K['teile']; GND = set(K['gnd'])
B0 = json.load(open(os.path.join(D, 'board.json')))
WW = args.get('wanderweite', '1') == '1'
if WW:
    from gemeinsam import wanderweite as WWM
    WT = WWM.tabelle(K); POS0 = {f['ref']: (f['x'], f['y']) for f in B0['fps']}
    for _r,_p in (B0.get('wander_basis') or {}).items(): POS0[_r] = (_p['x'],_p['y']) if isinstance(_p,dict) else tuple(_p[:2])
def weg(fps, r, dx=0.0, dy=0.0):
    ""
    return math.hypot(fps[r]['x'] + dx - POS0[r][0], fps[r]['y'] + dy - POS0[r][1])
def im_rahmen(fps, refs):
    ""
    for r, v in refs.items():
        if abs(v[2] % 360) > 1e-6 and (TEILE.get(r, {}).get('funktion') == 'spule_sw' or (WW and not WT[r]['drehen'])):
            return False
    return not WW or all(r in WT and WWM.im_rahmen(WT[r], weg(fps, r, v[0], v[1])) for r, v in refs.items())
def preis_zug(B, refs):
    ""
    if not WW: return 0.0
    fps = {f['ref']: f for f in B['fps']}
    return sum(WWM.preis(WT[r], weg(fps, r), weg(fps, r, v[0], v[1])) for r, v in refs.items())
R0 = json.load(open(os.path.join(D, 'routes.json')))
opt = dict(R0.get('optionen') or {})
if 'opt' in args: opt.update(dict(x.split('=', 1) for x in args['opt'].split() if '=' in x))
for k in ('start', 'neu', 'neu_netze', 'alle', 'png', 'pre', 'iter', 'kurz', 'umleg', 'fix', 'nur', 'klassen', 'regeln'): opt.pop(k, None)
opt.update(regeln=os.path.join(D, 'regeln.json'), klassen=os.path.join(D, 'klassen.json'))
ARBEIT = os.path.join(D, 'nachbessern'); os.makedirs(ARBEIT, exist_ok=True)
LOG = open(os.path.join(D, 'nachbessern.log'), 'a')
def log(*a):
    s = ' '.join(str(x) for x in a); print(s, flush=True); LOG.write(s + '\n'); LOG.flush()
FAMILIE = args.get('familie') or (K.get('vorgaben') or {}).get('familie') or 'unbekannt'
JL = VL.Versuchslog(os.path.join(D, 'nachbessern_versuche.jsonl'), dict(
    run_id=str(uuid.uuid4()), board_id=VL.sha_datei(os.path.join(D, 'board.json')), familie=FAMILIE, code=VL.code_hashes(),
    umgebung=VL.umgebung(), start_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())))
MODELL = None; POLICY = {}
if RANKER:
    from lernen import zuege as ZG
    MODELL = ZG.laden(RANKER); POLICY = dict(policy='ranker-v1-' + RANKER_POOL, model_id=MODELL.get('id'))
def ww_von(k):
    ""
    return {r: WT[r] for r in k['refs'] if r in WT} if WW else None
KL_SHA = VL.sha_datei(os.path.join(D, 'klassen.json')); RG_SHA = VL.sha_datei(os.path.join(D, 'regeln.json'))
def zustand(B, Rt): return dict(board=VL.sha_obj(B), routes=VL.sha_obj(Rt), klassen=KL_SHA, regeln=RG_SHA)


def route(board, start, out, neu=None, iter_=ITER, netze=None):
    ""
    cmd = ['caffeinate', '-i', '-s', VPY, os.path.join(R, 'router', 'route_pf.py'), board, out, 'start=' + start, 'iter=' + str(iter_)]
    if neu: cmd.append('neu=' + ','.join(neu))
    if netze: cmd.append('neu_netze=' + ','.join(netze))
    from nachbearbeitung.kurzschluss_schutz import schutz_an, schutz_option
    cmd += ['%s=%s' % kv for kv in opt.items()] + schutz_option(schutz_an(opt))

    rr = PL.lauf(cmd, log=out + '.route.log', env=dict(os.environ, PYTHONHASHSEED='0', AUTOROUTER_PLAETZE='4'))
    json.dump(dict(rr, command=cmd), open(out + '.route.meta.json', 'w'))
    return rr['rc'] == 0 and os.path.exists(out)


def frei_nach(Bk, refs):
    ""
    refs = [refs] if isinstance(refs, str) else list(refs)
    T2 = lade_teile(Bk, 0.15); _, _, ip, keep = platine(Bk, 0.2); f2 = {f['ref']: f for f in Bk['fps']}
    bel = Belegung()
    for q in T2:
        if q not in refs:
            gq = T2[q].geo(f2[q]['x'], f2[q]['y'], 0.0, f2[q]['side'])

            owned = dict(keepouts=[k for k in Bk.get('keepouts', []) if k.get('ref') in refs])
            if sperre(owned, gq, (q,)): return False
            bel.add(q, gq)
    for r in refs:
        if r not in T2 or f2[r].get('locked') or TEILE.get(r, {}).get('fest') or TEILE.get(r, {}).get('rolle') in ('stecker', 'mechanik'): return False
        g = T2[r].geo(f2[r]['x'], f2[r]['y'], 0.0, f2[r]['side'])
        if bel.kollision(g) or not all(ip.contains(x) for x in g.values()) or (keep is not None and sperre(Bk, g, (r,))): return False
        bel.add(r, g)
    return True


TABU = set()
DREH_TABU = set()


def rangfolge(B, Rt, E, ks):
    ""
    for k in ks: k['before'] = VL.vorher(B, K, k, E, Rt=Rt, ww=ww_von(k))
    werte = ZG.vorhersage(MODELL, [k['before'] for k in ks])
    for k, v in zip(ks, werte): k['rang_wert'] = round(float(v), 6)
    return sorted(ks, key=lambda k: -k['rang_wert'])[:RANKER_K]


def neue_zuege(B, Rt, fps, T, beweglich, schritt, out, ev):
    ""
    if 'insel' in ZUEGE:
        for name, insel in sorted((K.get('inseln') or {}).items()):
            refs = sorted(r for r, t in TEILE.items() if r == name or t.get('insel') == name)
            if not refs or not all(r in T and beweglich(r) for r in refs): continue
            d = min(schritt(r) for r in refs)
            if d <= .02: continue
            for i, (dx, dy) in enumerate(((d, 0), (-d, 0), (0, d), (0, -d))):
                m = {r: (dx, dy, 0.0) for r in refs}
                if im_rahmen(fps, m):
                    out['insel:%s:%d' % (name, i)] = dict(refs=m, art='insel', quelle='engstelle', grund='Insel ' + name,
                                                        netz=None, gewinn=1.0)
    if 'seitenwechsel' in ZUEGE:
        for r in sorted(T):
            if not beweglich(r) or TEILE[r].get('rolle') != 'klein' or len(T[r].pads) > 4: continue
            if any(p.get('drill', 0) or len(p['layers']) > 1 for p in B['pads'] if p['ref'] == r): continue
            if TEILE[r].get('funktion') in ('spule_sw', 'quarz', 'quarz_last'): continue

            kernref = TEILE[r].get('insel'); kern = fps.get(kernref)
            if not kern or kernref not in T: continue
            from shapely.geometry import Point
            footprint = T[kernref].geo(kern['x'], kern['y'], 0.0, kern['side'])[kern['side']]
            if not footprint.buffer(.2).contains(Point(fps[r]['x'], fps[r]['y'])): continue
            out['seite:' + r] = dict(refs={r: (0.0, 0.0, 0.0)}, seiten={r: 'B' if fps[r]['side'] == 'F' else 'F'},
                                     art='seitenwechsel', quelle='engstelle', grund='Gegenseite unter Inselkern', netz=None, gewinn=1.0)
    def geschuetzt(n):
        return bool(NETZ.get(n, {}).get('paar'))
    if 'knick' in ZUEGE:
        for ti, t in enumerate(Rt['tracks']):
            if geschuetzt(t['net']) or t.get('kelvin') or t.get('strom_steg'): continue
            for i in range(1, len(t['pts']) - 1):
                a, q, b = t['pts'][i - 1:i + 2]
                z = [round(a[0] + b[0] - q[0], 5), round(a[1] + b[1] - q[1], 5)]
                if not .02 < math.dist(z, q) <= 2 * MAXS: continue
                if not octi(a, z) or not octi(z, b): continue

                if any(p['net'] == t['net'] and math.dist(q, (p['x'], p['y'])) < .4 for p in B['pads']): continue
                if any(v['net'] == t['net'] and math.dist(q, (v['x'], v['y'])) < .4 for v in Rt['vias']): continue
                if any(j != ti and u['net'] == t['net'] and u['layer'] == t['layer'] and LineString(u['pts']).distance(LineString([q, q])) < .05
                       for j, u in enumerate(Rt['tracks'])): continue
                pts = [list(p) for p in t['pts']]; pts[i] = z
                out['knick:%d:%d' % (ti, i)] = dict(refs={}, netz=None, art='knick', quelle='bahn', grund='Knick %s/%s' % (t['net'], t['layer']),
                    gewinn=.5, route_moves=[dict(feld='tracks', index=ti, alt=dict(pts=t['pts']), neu=dict(pts=pts))], ziel_netz=t['net'])


            for i in range(1, len(t['pts']) - 2):
                a, q, r, b = t['pts'][i - 1:i + 3]
                def kreuz(u, v): return u[0] * v[1] - u[1] * v[0]
                u = (q[0] - a[0], q[1] - a[1]); v = (b[0] - r[0], b[1] - r[1]); h = (r[0] - q[0], r[1] - q[1])
                lh = math.hypot(*h)
                if lh < .1 or abs(kreuz(u, h)) < 1e-6 or abs(kreuz(v, h)) < 1e-6: continue
                from shapely.geometry import Point
                if any(p['net'] == t['net'] and min(math.dist(q, (p['x'], p['y'])), math.dist(r, (p['x'], p['y']))) < .4 for p in B['pads']): continue
                if any(w['net'] == t['net'] and min(math.dist(q, (w['x'], w['y'])), math.dist(r, (w['x'], w['y']))) < .4 for w in Rt['vias']): continue
                if any(j != ti and w['net'] == t['net'] and w['layer'] == t['layer'] and
                       min(LineString(w['pts']).distance(Point(q)), LineString(w['pts']).distance(Point(r))) < .05 for j, w in enumerate(Rt['tracks'])): continue
                for sg in (-1, 1):
                    d = min(MAXS / 2, .15); offset = (-h[1] / lh * d * sg, h[0] / lh * d * sg)
                    tq, tr = kreuz(offset, h) / kreuz(u, h), kreuz(offset, h) / kreuz(v, h)
                    q2 = [round(q[0] + tq * u[0], 5), round(q[1] + tq * u[1], 5)]
                    r2 = [round(r[0] + tr * v[0], 5), round(r[1] + tr * v[1], 5)]
                    if max(math.dist(q, q2), math.dist(r, r2)) > MAXS + 1e-5: continue
                    if not all(octi(x, y) for x, y in ((a, q2), (q2, r2), (r2, b))): continue
                    pts = [list(p) for p in t['pts']]; pts[i:i + 2] = [q2, r2]
                    out['kanal:%d:%d:%d' % (ti, i, sg)] = dict(refs={}, netz=None, art='knick', quelle='bahn',
                        grund='Knickpaar verschieben %s/%s' % (t['net'], t['layer']), gewinn=.5,
                        route_moves=[dict(feld='tracks', index=ti, alt=dict(pts=t['pts']), neu=dict(pts=pts))], ziel_netz=t['net'])
    if 'via' in ZUEGE:
        for vi, v in enumerate(Rt['vias']):
            if v['net'] in GND or geschuetzt(v['net']) or v.get('strom_via'): continue
            q = [v['x'], v['y']]
            if any(p['net'] == v['net'] and math.dist(q, (p['x'], p['y'])) < .5 for p in B['pads']): continue
            ends = [(ti, i) for ti, t in enumerate(Rt['tracks']) if t['net'] == v['net']
                    for i in (0, len(t['pts']) - 1) if math.dist(t['pts'][i], q) < 1e-5]
            if len(ends) < 2: continue

            if any(t['net'] == v['net'] and any(math.dist(p, q) < 1e-5 for p in t['pts'][1:-1]) for t in Rt['tracks']): continue
            d = min(.2, MAXS)
            for j, (dx, dy) in enumerate(((d, 0), (-d, 0), (0, d), (0, -d), (d, d), (-d, -d), (d, -d), (-d, d))):
                z = [round(q[0] + dx, 5), round(q[1] + dy, 5)]; ms = []
                for ti, i in ends:
                    t = Rt['tracks'][ti]; pts = [list(p) for p in t['pts']]
                    neighbor = pts[1 if i == 0 else -2]
                    if not octi(z, neighbor): break
                    pts[i] = z; ms.append(dict(feld='tracks', index=ti, alt=dict(pts=t['pts']), neu=dict(pts=pts)))
                else:
                    ms.append(dict(feld='vias', index=vi, alt=dict(x=v['x'], y=v['y']), neu=dict(x=z[0], y=z[1])))
                    out['via:%d:%d' % (vi, j)] = dict(refs={}, netz=None, art='via', quelle='via', grund='Via ' + v['net'],
                                                     gewinn=.5, route_moves=ms, ziel_netz=v['net'])


def kandidaten(B, Rt, E=None):
    ""
    fps = {f['ref']: f for f in B['fps']}
    ausgang_E = E
    out = {}
    def beweglich(r):
        t = TEILE.get(r, {})
        if fps.get(r, {}).get('locked'): return False
        if WW: return t.get('rolle') in ('klein', 'chip') and not t.get('fest') and r in WT and WWM.beweglich(WT[r])
        return t.get('rolle') == 'klein' and not t.get('fest') and not t.get('entkopplung')
    def schritt(r):
        return min(MAXS, WT[r]['max'] - weg(fps, r)) if WW else MAXS

    if 'schieben' in ZUEGE:
        tmpb = os.path.join(ARBEIT, 'k_board.json'); tmpr = os.path.join(ARBEIT, 'k_routes.json')
        json.dump(B, open(tmpb, 'w')); json.dump(Rt, open(tmpr, 'w'))
        kf = os.path.join(ARBEIT, 'k_routes_konflikte.json')
        if not os.path.exists(kf): json.dump([], open(kf, 'w'))
        subprocess.run([VPY, os.path.join(R, 'analyse', 'engstellen.py'), tmpb, tmpr, os.path.join(D, 'klassen.json'),
                        'out=' + os.path.join(ARBEIT, 'k_eng.json'), 'max_schritt=%g' % MAXS], capture_output=True, text=True)
        E = json.load(open(os.path.join(ARBEIT, 'k_eng.json')))
        for r, v in E['verschiebungen'].items():
            if not beweglich(r) or not im_rahmen(fps, {r: (v['dx'], v['dy'], 0.0)}): continue
            out[r] = dict(refs={r: (v['dx'], v['dy'], 0.0)}, art='schieben', quelle='engstelle', grund='Engstelle: ' + v['gruende'][0], gewinn=1.0, netz=None)

    T = lade_teile(B, 0.15); board, inner, inner_p, keep = platine(B, 0.2)
    bel = Belegung()
    for r in T: bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
    def frei_hier(r, st):
        geo = T[r].geo(*st); bel.remove(r)
        ok = not bel.kollision(geo) and all(inner_p.contains(x) for x in geo.values()) and not (keep is not None and sperre(B, geo, (r,)))
        bel.add(r, T[r].geo(fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
        return ok
    pads_net = collections.defaultdict(list)
    for p in B['pads']: pads_net[p['net']].append(p)
    padpoly = [(p['ref'], p['net'], unary_union([Polygon(q) for qs in p['polys'].values() for q in qs if len(q) >= 3])) for p in B['pads']]
    ev = ausgang_E or bewertung(B, Rt)
    umweg_netze = {}
    for gain, n, L, m in ev['umwege'][:8]:
        umweg_netze[n] = gain
        P = pads_net[n]
        if len(P) < 2: continue

        pts = [(p['x'], p['y']) for p in P]; inT = [0]; rest = list(range(1, len(pts))); edges = []
        while rest:
            i, j = min(((i, j) for i in inT for j in rest), key=lambda ij: math.dist(pts[ij[0]], pts[ij[1]]))
            edges.append((pts[i], pts[j])); inT.append(j); rest.remove(j)
        w = NETZ.get(n, {}).get('w', 0.15)
        if 'schieben' in ZUEGE:
            for a, b in edges:
                seg = LineString([a, b]);
                if seg.length < 1e-6: continue
                korr = seg.buffer(w / 2 + 0.12)
                for r, pn, g in padpoly:
                    if pn == n or not beweglich(r) or r in out: continue
                    if not korr.intersects(g): continue
                    c = g.centroid; q = seg.interpolate(seg.project(c))
                    dd = math.hypot(c.x - q.x, c.y - q.y)
                    ux, uy = ((c.x - q.x) / dd, (c.y - q.y) / dd) if dd > 1e-3 else (-(b[1] - a[1]) / seg.length, (b[0] - a[0]) / seg.length)
                    need = min(schritt(r), w / 2 + 0.12 + g.minimum_rotated_rectangle.length / 4 - dd + 0.05)
                    if need <= 0.02: continue
                    dx, dy = ux * need, uy * need
                    if not im_rahmen(fps, {r: (dx, dy, 0.0)}): continue
                    if frei_hier(r, (fps[r]['x'] + dx, fps[r]['y'] + dy, 0.0, fps[r]['side'])):
                        out[r] = dict(refs={r: (round(dx, 3), round(dy, 3), 0.0)}, art='schieben', quelle='umweg',
                                      grund='Umweg %s (+%.1f mm)' % (n.split('/')[-1], gain), gewinn=gain, netz=n)

        if 'drehen' in ZUEGE:
            for r in sorted({pr for pr, pn, g in padpoly if pn == n}):
                if not beweglich(r) or len(T[r].pads) > 3: continue
                if (WW and not WT[r]['drehen']) or TEILE[r].get('funktion') == 'spule_sw': continue
                for th, fak in ((180.0, 0.8), (90.0, 0.6), (270.0, 0.6)):
                    key = '%s@%g' % (r, th)
                    if key in out or (r, th) in DREH_TABU: continue
                    if frei_hier(r, (fps[r]['x'], fps[r]['y'], th, fps[r]['side'])):
                        out[key] = dict(refs={r: (0.0, 0.0, th)}, art='drehen', quelle='umweg',
                                        grund='Drehen %g wegen Umweg %s' % (th, n.split('/')[-1]), gewinn=gain * fak, netz=n)

    if 'tauschen' in ZUEGE and umweg_netze:
        netze_von = collections.defaultdict(set)
        for p in B['pads']: netze_von[p['ref']].add(p['net'])
        gruppen = collections.defaultdict(list)
        for r, f in fps.items():
            if r in T and beweglich(r): gruppen[(f.get('fp'), f.get('value'), f['side'])].append(r)
        for g_refs in gruppen.values():
            for i, r1 in enumerate(g_refs):
                for r2 in g_refs[i + 1:]:
                    f1, f2 = fps[r1], fps[r2]
                    if math.hypot(f1['x'] - f2['x'], f1['y'] - f2['y']) > TAUSCH_AB: continue
                    if netze_von[r1] == netze_von[r2]: continue
                    if frozenset((r1, r2)) in TABU: continue
                    gain = max([umweg_netze.get(n, 0.0) for n in netze_von[r1] | netze_von[r2]])
                    if gain <= 0: continue
                    zug = None
                    for sg in (1, -1):
                        th1 = (sg * (f2.get('rot', 0) - f1.get('rot', 0))) % 360
                        m = {r1: (f2['x'] - f1['x'], f2['y'] - f1['y'], th1), r2: (f1['x'] - f2['x'], f1['y'] - f2['y'], (-th1) % 360)}
                        Bk, _ = bewegen(B, m)
                        p1 = sorted((round(p['x'], 3), round(p['y'], 3)) for p in Bk['pads'] if p['ref'] == r1)
                        p2 = sorted((round(p['x'], 3), round(p['y'], 3)) for p in B['pads'] if p['ref'] == r2)
                        if p1 == p2: zug = m; break
                    if zug is None or not im_rahmen(fps, zug): continue
                    key = '%s<>%s' % (r1, r2)
                    n_um = max(netze_von[r1] | netze_von[r2], key=lambda n: umweg_netze.get(n, 0.0))
                    out[key] = dict(refs=zug, art='tauschen', quelle='umweg', grund='Tausch %s<>%s wegen Umweg %s' % (r1, r2, n_um.split('/')[-1]),
                                    gewinn=gain * 0.9, netz=n_um)


    if 'versetzen' in ZUEGE and WW:
        from gemeinsam.versetzen import ziele
        for r in sorted(T):
            if not beweglich(r) or WT[r]['max'] != WWM.GANZ or len(T[r].pads) != 2: continue
            for i, z in enumerate(ziele(B, Rt, K, r, n=VERS_N, T=T, bel=bel, plat=(board, inner, inner_p, keep))):
                if not im_rahmen(fps, z['refs']): continue
                out['%s~%d' % (r, i)] = dict(refs=z['refs'], art='versetzen', quelle='versetzen', netz=None,
                                             grund='Versetzen an %s neben %s (Abstand %.2f mm, %s)' % (
                                                 z['signal'].split('/')[-1], z['anderes'].split('/')[-1], z['d'], z['lage']),
                                             gewinn=1.0 / (0.2 + z['d']))
    neue_zuege(B, Rt, fps, T, beweglich, schritt, out, ev)

    out = {key: k for key, k in out.items() if not k['refs'] or frei_nach(bewegung(B, k), k['refs'])}

    if WW:
        for k in out.values(): k['preis'] = preis_zug(B, k['refs'])
    for k in out.values(): k['aufwand'] = 1.0 + max(0.0, k.get('preis', 0.0))
    alle = sorted(out.values(), key=lambda k: -k['gewinn'] / k['aufwand'])
    if RANKER and RANKER_POOL == 'alle': return rangfolge(B, Rt, ausgang_E or ev, alle)
    wahl = []
    arten = ('schieben', 'drehen', 'tauschen', 'versetzen', 'insel', 'seitenwechsel', 'knick', 'via')
    gruppen = {art: [k for k in alle if k['art'] == art][:VERS_PL if art == 'versetzen' else 2] for art in arten}
    if ZUEGE & {'insel', 'seitenwechsel', 'knick', 'via'} and sum(map(len, gruppen.values())) > NK:

        for i in range(max(map(len, gruppen.values()), default=0)):
            for art in arten:
                if i < len(gruppen[art]) and len(wahl) < NK: wahl.append(gruppen[art][i])
    else:
        for art in arten: wahl += gruppen[art]
    for k in alle:
        if len(wahl) >= NK: break
        if k not in wahl: wahl.append(k)
    wahl = sorted(wahl[:NK], key=lambda k: -k['gewinn'] / k['aufwand'])
    if RANKER: return rangfolge(B, Rt, ausgang_E or ev, wahl)
    return wahl


def zug_text(k):
    return ', '.join('%s (%+.2f, %+.2f%s)' % (r, v[0], v[1], ', %g Grad' % v[2] if v[2] else '') for r, v in k['refs'].items()) or k['art'] + ': ' + k.get('grund', '')


BUDGET, BUDGET_GR = budgets(K)
IR_REF = {}


def ir_ok(E0, E1):
    ""
    if IR and E0.get('gueltig') and not IR_REF and (E0.get('ir') or {}).get('status') == 'solved':
        IR_REF.update(E0['ir']['max_abfall_mV'])
    if not IR or not E0.get('gueltig') or not IR_REF: return True
    I = E1.get('ir') or {}
    if I.get('status') != 'solved' or I.get('fehler') or not I.get('max_abfall_mV'): return False
    d = I['max_abfall_mV']

    if not all(d.get(n, float('inf')) <= max(BUDGET.get(n, 0.0), ref * (1 + IR_TOL) + 0.05) for n, ref in IR_REF.items()): return False
    for nn, b in BUDGET_GR.values():
        ref = sum(IR_REF.get(n, 0.0) for n in nn)
        if sum(d.get(n, float('inf')) for n in nn) > max(b, ref * (1 + IR_TOL) + 0.05): return False
    return True


def entscheidung(B, Rt, E, k, rr):
    if rr['E'] is None: return None
    Bk, Rk = rr['st']
    c = annahme(E, rr['E'], B, Rt, Bk, Rk, K, mindest=MIND, preis=k.get('preis', 0.0), note=NOTE)
    if not ir_ok(E, rr['E']): c.update(besser=False, grund='IR-Budget oder Nachweis fehlt')
    return c


def ropt(k):
    o = dict(opt, iter=ITER, neu=','.join(k['refs']))
    if k.get('netz'): o['neu_netze'] = k['netz']
    return o


def versuche(B, Rt, k, d):
    ""
    t1 = time.perf_counter(); os.makedirs(d, exist_ok=True); res = dict(status=None, fehler=None, E=None, st=None, route_s=None, kb=None, rs=None, wall_s=None)
    try:
        Bk = bewegung(B, k);
        if k['art'] != 'null' and k['refs'] and (not frei_nach(Bk, k['refs']) or not im_rahmen({f['ref']: f for f in B['fps']}, k['refs'])):
            res.update(status='precheck_rejected', fehler='Kollision, Sperre, Wanderweite oder Rand'); return res
        bj = os.path.join(d, 'board.json'); json.dump(Bk, open(bj, 'w')); res['kb'] = VL.sha_obj(Bk)
        rj = os.path.join(d, 'routes.json')
        if os.path.exists(rj): os.remove(rj)
        sj = os.path.join(d, 'start.json'); json.dump(routenzug(Rt, k), open(sj, 'w'))
        t2 = time.perf_counter()
        ok = route(bj, sj, rj, list(k['refs']), netze=[k['netz']] if k.get('netz') else None)
        res['route_s'] = time.perf_counter() - t2
        if not ok: res.update(status='router_error', fehler='Router ohne Ausgabe oder Rueckgabecode != 0')
        else:
            Rk = json.load(open(rj)); res['rs'] = VL.sha_obj(Rk)
            try: res.update(status='evaluated', E=bewertung(Bk, Rk), st=(Bk, Rk))
            except Exception as e: res.update(status='evaluation_error', fehler=repr(e)[:300])
    except Exception as e:
        res.update(status='router_error', fehler=repr(e)[:300])
    res['wall_s'] = time.perf_counter() - t1
    return res

def main():
    t0 = time.time()
    moves = collections.defaultdict(lambda: [0.0, 0.0, 0.0])
    B = B0; start = os.path.join(ARBEIT, 'basis_routes.json')
    log('== Nachbessern', D, time.strftime('%H:%M'), 'Zuege', sorted(ZUEGE), 'Familie', FAMILIE, 'Optionen', opt, *(('Wanderweite',) if WW else ()),
        *(('Ranker', MODELL.get('id'), 'k=%d' % RANKER_K, RANKER_POOL) if RANKER else ()))
    if not route(os.path.join(D, 'board.json'), os.path.join(D, 'routes.json'), start, None, 1):
        log('Basislauf fehlgeschlagen'); return
    Rt = json.load(open(start)); E = bewertung(B, Rt)
    E_input = bewertung(B0, R0)
    if not annahme(E_input, E, B0, R0, B, Rt, K, mindest=-1e-6, note=NOTE)['besser']:
        Rt, E = R0, E_input
    json.dump(E, open(os.path.join(ARBEIT, 'basis_bewertung.json'), 'w'))
    log('Basis (%s): Note %.1f, Vias %d, Umwege %.1f mm, Defizit %.1f, %s%s' % (NOTE, nw(E), E['vias'], E['umweg_summe'], E['defizit'], 'gueltig' if E['gueltig'] else 'UNGUELTIG',
        ', Verluste %.1f mW' % E['ir']['verlust_mW'] if IR and E['ir'] and E['ir'].get('verlust_mW') is not None else ''))
    letzter = None
    for rnd in range(1, RUNDEN + 1):
        if budget_aus('keine Runde %d mehr' % rnd): break
        ks = kandidaten(B, Rt, E) if RANKER else kandidaten(B, Rt)
        log('Runde %d: %d Kandidaten (%s)' % (rnd, len(ks), dict(collections.Counter(k['art'] for k in ks))))

        kontrollen = {}
        for k in [dict(refs={}, netz=None)] + ks:
            sig = (tuple(sorted(k['refs'])), k.get('netz'))
            if sig not in kontrollen:
                kontrollen[sig] = dict(refs={r: (0.0, 0.0, 0.0) for r in sig[0]}, netz=sig[1], art='null',
                                      quelle='kontrolle', grund='Nullzug mit gleichem Neuverlegungsumfang', gewinn=0.0, preis=0.0)
        controls = list(kontrollen.values())
        for k in controls: k['before'] = VL.vorher(B, K, k, E, Rt=Rt, ww=ww_von(k))
        with ThreadPoolExecutor(PAR) as ex:
            cr = list(ex.map(lambda ik: versuche(B, Rt, ik[1], os.path.join(ARBEIT, 'r%d_null%d' % (rnd, ik[0]))), enumerate(controls)))
        for k, rr in zip(controls, cr):
            k['result'] = rr
        st0 = zustand(B, Rt); dec = 'r%d-primary' % rnd
        for k in ks: k['before'] = k.get('before') or VL.vorher(B, K, k, E, Rt=Rt, ww=ww_von(k))
        with ThreadPoolExecutor(PAR) as ex:
            res = list(ex.map(lambda ik: versuche(B, Rt, ik[1], os.path.join(ARBEIT, 'r%d_k%d' % (rnd, ik[0]))), enumerate(ks)))
        gut = []
        for i, k in enumerate(controls):
            rr = k['result']; c = entscheidung(B, Rt, E, k, rr)
            rr['vergleich'] = c
            if c and c['besser']: gut.append(((c.get('delta') or 0), k, rr['E'], rr['st']))
            k['attempt_id'] = str(uuid.uuid4())
            log('   Nullkontrolle', i, rr['status'], 'Note', nw(rr['E']) if rr['E'] else None)
        for k, rr in zip(ks, res):
            sig = (tuple(sorted(k['refs'])), k.get('netz')); ck = kontrollen[sig]; ctrl = ck['result']
            rr['control_id'] = ck['attempt_id']
            Ek = rr['E']
            if Ek is None: log('   %-34s %s' % (zug_text(k), rr['status'])); continue
            c = entscheidung(B, Rt, E, k, rr)
            rr['vergleich'] = c
            nc = entscheidung(ctrl['st'][0], ctrl['st'][1], ctrl['E'], k, rr) if ctrl['E'] else None
            rr['nullvergleich'] = nc
            d0 = c['delta'] if c.get('delta') is not None else 0.0
            d = d0 + k.get('preis', 0.0)
            ok = c['besser'] and nc is not None and nc['besser']
            dv = ((Ek.get('ir') or {}).get('verlust_mW') or 0) - ((E.get('ir') or {}).get('verlust_mW') or 0) if IR else 0
            log('   %-34s Note %+.1f  Vias %+d  Umwege %+.1f%s  %s  %s' % (
                zug_text(k), d0, Ek['vias'] - E['vias'], Ek['umweg_summe'] - E['umweg_summe'], '  Verlust %+.1f mW' % dv if IR else '',
                'besser' if ok else ('-' if ir_ok(E, Ek) else '- (IR)'), k['grund'] + ('  Preis %.1f' % k['preis'] if k.get('preis') else '')))
            if ok: gut.append((d, k, Ek, rr['st']))
        from analyse import minimax as MM
        gut.sort(key=lambda g: (tuple(g[2]['rang'][:3]), MM.rang(g[2]) if g[2].get('bewertungsziel') == 'minimax' or NOTE == 'minimax' else g[0]))
        bester = gut[0][1] if gut else None
        for i, k in enumerate(controls):
            rr = k['result']
            JL.versuch(runde=rnd, index=i, decision_id=dec, phase='control', parent=letzter, original=None,
                k=k, before=k['before'], state=st0, nach_E=rr['E'], status=rr['status'], fehler=rr['fehler'], wall_s=rr['wall_s'],
                route_s=rr['route_s'], router_opt=ropt(k), accepted=k is bester, mindest=MIND, kandidatenboard_sha=rr['kb'],
                ergebnis_sha=rr['rs'], comparison=rr.get('vergleich'), attempt_id=k['attempt_id'], **POLICY)
        for i, (k, rr) in enumerate(zip(ks, res)):
            k['attempt_id'] = JL.versuch(runde=rnd, index=i, decision_id=dec, phase='primary', parent=letzter, original=None, k=k,
                                         before=k['before'], state=st0, nach_E=rr['E'], status=rr['status'], fehler=rr['fehler'],
                                         wall_s=rr['wall_s'], route_s=rr['route_s'], router_opt=ropt(k), accepted=k is bester,
                                         mindest=MIND, kandidatenboard_sha=rr['kb'], ergebnis_sha=rr['rs'],
                                         comparison=rr.get('vergleich'), control_id=rr.get('control_id'), control_comparison=rr.get('nullvergleich'), **POLICY)
        JL.runde_fertig()
        if not gut: log('   keine Verbesserung, fertig'); break
        d, k, Ek, (B, Rt) = gut[0]; E = Ek; letzter = k['attempt_id']
        if k['art'] != 'null':
            for r, v in k['refs'].items(): moves[r][0] += v[0]; moves[r][1] += v[1]; moves[r][2] += v[2]
        if k['art'] == 'tauschen': TABU.add(frozenset(k['refs']))
        if k['art'] == 'drehen':
            for r_, v_ in k['refs'].items(): DREH_TABU.add((r_, (360.0 - v_[2]) % 360))
        log('   uebernommen %s: Note %.1f' % (zug_text(k), nw(E)))
        benutzt = set(k['refs'])
        for j, (d2, k2, _, _) in enumerate(gut[1:]):
            if k2['art'] == 'null' or benutzt & set(k2['refs']): continue
            if k2.get('route_moves') or k2.get('seiten'): continue
            if budget_aus('keine weitere Nachpruefung'): break
            before = VL.vorher(B, K, k2, E, Rt=Rt, ww=ww_von(k2)); st = zustand(B, Rt); dec2 = 'r%d-recheck-%d' % (rnd, j)
            Bk = bewegung(B, k2)
            if not frei_nach(Bk, list(k2['refs'])) or not im_rahmen({f['ref']: f for f in B['fps']}, k2['refs']):
                JL.versuch(runde=rnd, index=j, decision_id=dec2, phase='recheck', parent=letzter, original=k2['attempt_id'], k=k2,
                           before=before, state=st, nach_E=None, status='precheck_rejected', fehler='Kollision oder ausserhalb nach vorheriger Uebernahme',
                           wall_s=None, route_s=None, router_opt=ropt(k2), accepted=False, mindest=MIND, kandidatenboard_sha=VL.sha_obj(Bk), ergebnis_sha=None, **POLICY)
                continue
            rr = versuche(B, Rt, k2, os.path.join(ARBEIT, 'r%d_plus_%s' % (rnd, '_'.join(k2['refs']))))
            Ek2 = rr['E']
            p2 = preis_zug(B, k2['refs']) if WW else 0.0
            c = entscheidung(B, Rt, E, dict(k2, preis=p2), rr)

            null = dict(refs={r: (0.0, 0.0, 0.0) for r in k2['refs']}, netz=k2.get('netz'), art='null',
                        quelle='kontrolle', grund='Nullzug fuer Nachpruefung', gewinn=0.0, preis=0.0)
            nr = versuche(B, Rt, null, os.path.join(ARBEIT, 'r%d_plus_null%d' % (rnd, j)))
            ncmp = entscheidung(nr['st'][0], nr['st'][1], nr['E'], dict(k2, preis=p2), rr) if nr['E'] and Ek2 else None
            nid = JL.versuch(runde=rnd, index=j, decision_id=dec2, phase='control', parent=letzter, original=None, k=null,
                            before=VL.vorher(B, K, null, E, Rt=Rt, ww=ww_von(null)), state=st, nach_E=nr['E'], status=nr['status'],
                            fehler=nr['fehler'], wall_s=nr['wall_s'], route_s=nr['route_s'], router_opt=ropt(null), accepted=False,
                            mindest=MIND, kandidatenboard_sha=nr['kb'], ergebnis_sha=nr['rs'], **POLICY)
            ok = bool(c and c['besser'] and ncmp is not None and ncmp['besser'])
            aid = JL.versuch(runde=rnd, index=j, decision_id=dec2, phase='recheck', parent=letzter, original=k2['attempt_id'], k=k2,
                             before=before, state=st, nach_E=Ek2, status=rr['status'], fehler=rr['fehler'], wall_s=rr['wall_s'],
                             route_s=rr['route_s'], router_opt=ropt(k2), accepted=ok, mindest=MIND, kandidatenboard_sha=rr['kb'], ergebnis_sha=rr['rs'],
                             comparison=c, control_id=nid, control_comparison=ncmp, **POLICY)
            if ok:
                (B, Rt), E = rr['st'], Ek2; benutzt |= set(k2['refs']); letzter = aid
                if k2['art'] == 'tauschen': TABU.add(frozenset(k2['refs']))
                if k2['art'] == 'drehen':
                    for r_, v_ in k2['refs'].items(): DREH_TABU.add((r_, (360.0 - v_[2]) % 360))
                for r, v in k2['refs'].items(): moves[r][0] += v[0]; moves[r][1] += v[1]; moves[r][2] += v[2]
                log('   zusaetzlich %s: Note %.1f' % (zug_text(k2), nw(E)))
        JL.runde_fertig()
    json.dump(B, open(os.path.join(D, 'board_nach.json'), 'w')); json.dump(Rt, open(os.path.join(D, 'routes_nach.json'), 'w'))

    changed = {f['ref'] for f in B['fps'] if f != next(g for g in B0['fps'] if g['ref'] == f['ref'])}
    fb = {f['ref']: f for f in B['fps']}
    P = dict(parts={r: dict(side=fb[r]['side'], pads={p['num']: [p['x'], p['y']] for p in B['pads'] if p['ref'] == r}) for r in sorted(changed)})
    from platzierung.p3_gruppen import board_mit_gruppen
    P = board_mit_gruppen(P, B)
    json.dump(P, open(os.path.join(D, 'placement_nach.json'), 'w'))
    json.dump(E, open(os.path.join(ARBEIT, 'ergebnis_bewertung.json'), 'w'))
    log('Ergebnis (%s): Note %.1f, Vias %d, Umwege %.1f mm, Defizit %.1f, %s; verschoben %s; %.0f s' % (
        NOTE, nw(E), E['vias'], E['umweg_summe'], E['defizit'], 'gueltig' if E['gueltig'] else 'UNGUELTIG',
        {r: (round(v[0], 2), round(v[1], 2), v[2] % 360) for r, v in moves.items()}, time.time() - t0))
    if WW:
        fb = {f['ref']: f for f in B['fps']}
        for r in sorted(moves):
            w = WT[r]; fm = lambda x: 'ganz' if x == math.inf else '%.1f' % x
            log('   Wanderweg %-10s %.2f mm (%s: grund %s, max %s, %.0f/mm)' % (r, weg(fb, r), w['schluessel'], fm(w['grund']), fm(w['max']), w['wichtigkeit']))


if __name__ == '__main__':
    main()
