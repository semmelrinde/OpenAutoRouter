class _Etiketten:
    def __init__(self):
        self.cpu = 0.0
        self.an = bool(args.get('etiketten'))
        self.runden = []; self.konflikte = []; self.signaturen = {}; self.aenderung = {}
        self.namen = sorted(n for n in bynet if not n.startswith('#'))
        self.nid = {n: i for i, n in enumerate(self.namen)}
        self.nx = (NX + 4) // 5; self.ny = (NY + 4) // 5
        if self.an:
            self.hmax = np.zeros((NL, self.ny, self.nx), np.float32)
            self.kmap = np.zeros_like(self.hmax, dtype=np.uint32)
            self.karten = []

    def grob(self, a, mittel=False):
        a = np.asarray(a).reshape(NL, NY, NX)
        p = np.pad(a, ((0, 0), (0, self.ny * 5 - NY), (0, self.nx * 5 - NX)))
        p = p.reshape(NL, self.ny, 5, self.nx, 5)
        return p.mean((2, 4)) if mittel else p.max((2, 4))

    def zellen(self, c):
        c = np.asarray(c, dtype=np.int64)
        c = c[(c >= 0) & (c < NL * NN)]
        return (c // NN) * self.nx * self.ny + ((c % NN) // NX // 5) * self.nx + (c % NX) // 5

    def netz(self, u):
        if u in U: return U[u]['net']
        if u.startswith('GND#'): return D['pads'][int(u[4:])]['net']
        return u[4:] if u.startswith('FIX#') else u

    def konfliktkopie(self, conf):
        if not self.an: return None
        t = time.process_time()
        try: return {u: list(c) for u, c in conf.items()}
        except Exception as e:
            self.an = False; print('Etiketten: Konfliktkopie ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def ergaenzen(self, conf, eng):
        if not self.an: return
        t = time.process_time()
        try:
            for u, c in eng.items(): conf.setdefault(u, []).extend(c)
        except Exception as e:
            self.an = False; print('Etiketten: Konfliktergaenzung ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def beobachten(self, conf, durchgang, phase='verhandlung', geometrie=True):
        if not self.an or phase == 'erstaufbau': return
        t = time.process_time()
        try:
            counts = np.zeros(len(self.namen), np.uint32); cells = []
            for u, c in conf.items():
                n = self.netz(u)
                if n in self.nid: counts[self.nid[n]] += len(c)
                if len(c): cells.append(self.zellen(c))
            if cells:
                self.kmap.ravel()[np.unique(np.concatenate(cells))] += 1
            np.maximum(self.hmax, self.grob(hist), out=self.hmax)
            self.runden.append((durchgang, phase)); self.konflikte.append(counts)
            if geometrie:
                by = {}
                for u in sorted(U):
                    r = U[u]; by.setdefault(r['net'], []).append((r['tracks'], r['vias']))
                for n, g in by.items():
                    sig = json.dumps(g, sort_keys=True)
                    if self.signaturen.get(n) != sig:
                        self.signaturen[n] = sig; self.aenderung[n] = durchgang
            if args.get('etiketten_runden') == '1': self.karten.append(self.grob(hist).astype(np.float32))
        except Exception as e:
            self.an = False; print('Etiketten: Beobachtung ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def ausgabebelegung(self, tracks, vias, zonen):
        ""
        from nachbearbeitung.kurzschluss_schutz import ausgabebelegung
        return ausgabebelegung(dict(tracks=tracks, vias=vias, zonen=list(zonen), optionen=args),
                               LAYERS, np.array([X0,Y0]), (self.ny,self.nx))

    def schreiben(self, tracks, vias, nur_cache=False, fehlend=(), zonen=()):
        global _LERN_LABELS
        if not self.an: return
        t = time.process_time()
        try:
            import os, tempfile
            nr = len(self.namen); a = np.zeros((nr, 7), np.float32)
            failed = {self.netz(u) for u in set(fail) | set(fehlend)}
            need = {self.netz(u) for u in units}
            if globals().get('_LERN_AN'): failed |= {self.netz(u) for u in set(units) - set(U) - set(FIXED)}
            vorhanden = {r['net'] for r in U.values()}
            _r45_label = (globals().get('_sicher') or {}) if not nur_cache else {}
            _r45_board = _r45_label.get('rest_board')
            if _r45_board: vorhanden = {q['net'] for q in tracks+vias}
            lm = np.zeros((nr, NL), bool); vmap = np.zeros_like(self.kmap)
            for n, i in self.nid.items():
                ps = ([p for p in _r45_board['pads'] if p['net'] == n] if _r45_board else [D['pads'][k] for k in bynet[n]])
                hp = max(p['x'] for p in ps) - min(p['x'] for p in ps) + max(p['y'] for p in ps) - min(p['y'] for p in ps)
                a[i, 0] = n not in failed and (n not in need or n in vorhanden)
                a[i, 2] = hp; a[i, 6] = self.aenderung.get(n, 0)
            for tr in tracks:
                i = self.nid.get(tr['net'])
                if i is None: continue
                a[i, 1] += sum(math.dist(x, y) for x, y in zip(tr['pts'][:-1], tr['pts'][1:]))
                lm[i, LI[tr['layer']]] = True
            for v in vias:
                i = self.nid.get(v['net'])
                if i is not None: a[i, 4] += 1; lm[i, :] = True
                x = int((v['x'] - X0) / .25); y = int((v['y'] - Y0) / .25)
                if 0 <= x < self.nx and 0 <= y < self.ny: vmap[:, y, x] += 1
            a[:, 3] = np.divide(a[:, 1], a[:, 2], out=np.zeros(nr, np.float32), where=a[:, 2] > 0)
            a[:, 5] = lm.sum(1)
            search_occ = self.grob(use > 0, mittel=True).astype(np.float16)
            protected = args.get('kurzschluss_schutz', '0') == '1'
            occ = self.ausgabebelegung(tracks, vias, zonen) if protected else search_occ
            data = dict(hist_context=np.asarray(args.get('hist_context', '')), etikettenversion=np.asarray(2 if protected else 1, np.int16), beobachterversion=np.asarray(4 if protected else 3, np.int16), origin_mm=np.array([X0, Y0]), raster_mm=np.asarray(.25),
                        lagen=np.asarray(LAYERS), history_max=self.hmax, history_final=self.grob(hist),
                        belegung=occ, belegung_suche=search_occ, belegung_vertrag=np.asarray('ausgabekupfer_zellkontakt' if protected else 'suchbelegung_anteil'),
                        konflikt_runden=self.kmap, vias_raster=vmap,
                        netze=np.asarray(self.namen), netz_spalten=np.asarray(['geroutet', 'laenge_mm', 'hpwl_mm', 'umweg_hpwl', 'vias', 'lagen_n', 'letzte_aenderung']),
                        netz_werte=a, netz_lagen=lm, netz_konflikte=np.asarray(self.konflikte, np.uint32).reshape(-1, nr),
                        durchgaenge=np.asarray([r[0] for r in self.runden], np.int32), phasen=np.asarray([r[1] for r in self.runden]))
            if not protected:
                del data['belegung_suche']; del data['belegung_vertrag']
            if _r45_board:
                data['r45_placement_json'] = np.asarray(json.dumps(_r45_label['rest_placement'], sort_keys=True))
                data['r45_zielpad_json'] = np.asarray(json.dumps(_r45_board['pads'], sort_keys=True))
            if self.karten: data['history_runden'] = np.asarray(self.karten)
            if nur_cache:
                _LERN_LABELS = {k: v.copy() for k, v in data.items()}
                return
            dest = os.path.abspath(args['etiketten']); os.makedirs(os.path.dirname(dest), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
            try:
                np.savez_compressed(tmp, **data); os.replace(tmp, dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            print('Etiketten: %s (%d Netze, %d Beobachtungen, %.3f CPU-s)' % (dest, nr, len(self.runden), self.cpu + time.process_time() - t), flush=True)
        except Exception as e: print('Etiketten: Schreiben ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t


_ET = None



bynet = {}
for k, p in enumerate(D['pads']):
    if p['net'] and not p['net'].startswith('unconnected'):
        bynet.setdefault(p['net'], []).append(k)
def span(n):
    pads = [D['pads'][k] for k in bynet[n]]
    return max(p['x'] for p in pads) - min(p['x'] for p in pads) + max(p['y'] for p in pads) - min(p['y'] for p in pads)


def lern_reihenwert(n, value):
    ""
    if KL is not None and n in bynet:
        art=(KL['netze'].get(n) or {}).get('art','signal')
        factor=float(args.get('kosten_rang_'+art,'1'))
        if factor!=1.:return value-(factor-1.)*math.log1p(span(n))/5.
    return value


def lern_startspan(n, power=False):
    value=span(n)
    if KL is not None:
        art=(KL['netze'].get(n) or {}).get('art','signal')
        factor=float(args.get('kosten_rang_'+art,'1'))
        if factor!=1.:return value*(4*factor-3 if power else 3-2*factor)
    return value
NETTIES = set(KL.get('net_ties', [])) if KL else set()
def grp(n):
    if n in PLANE_GROUPS:return (4.8,0)
    if paar_schnell(n):
        k = KL['netze'][n]
        return (-2, -k.get('paar_rang', k.get('rang', 0)), min(n, PAAR_NETZE[n]), n)
    if strom_modell(n): return (-1.5, -sum(map(float,strom_modell(n)['last'].values())), n)
    if n.startswith('GND#'):
        r_ = D['pads'][int(n[4:])]['ref']
        if r_.startswith('NT_') or r_ in NETTIES: return (-1, 0) if KL else (0, 0)
        if int(n[4:]) in VAP_WICHTIG: return (4.8, 0)
        return (5, 0) if KL else (4, 0)
    if KL is not None and n in KL['netze']:
        k = KL['netze'][n]; g = k.get('gruppe', 4)
        _rang_f = float(args.get('kosten_rang_' + k.get('art', 'signal'), '1'))
        rang = k.get('rang', 0) * float(args.get('kosten_rang_' + k.get('art', 'signal'), '1'))
        if g == 2: return (2 - .1 * (_rang_f-1), -lern_startspan(n,power=True))
        if REIHENFOLGE >= 2:
            if g == 1 and k.get('rang', 0) < SCHNELL_AB: g = 3.5
            elif g == 4 and k.get('rang', 0) <= 0: g = 4.5
        return (g - .1 * (_rang_f-1), -rang, lern_startspan(n))
    pr = cls(n)['prio']
    if pr <= 1: return (0, span(n))
    if pr in (2, 3, 5): return (1, -span(n))
    if pr < 8: return (2, span(n))
    return (3, span(n))
units = [n for n in bynet if n not in GNDS and n not in FLNETZE and len(bynet[n]) > 1 and not n.startswith('#')]
if args.get('nur') == 'leistung':
    units = [n for n in units if cls(n)['prio'] < 8]
esp61 = 0
OHNE = {}
for spec in args.get('gnd_ohne_via', '').split(';'):
    if ':' in spec: r_, nums = spec.split(':'); OHNE[r_] = set(nums.split(','))
for _g in sorted(FLNETZE):
    for k in bynet.get(_g, []):
        if D['pads'][k]['num'] in OHNE.get(D['pads'][k]['ref'], ()): continue
        if any(L in GNDL for L in D['pads'][k]['layers']): continue
        units.append('GND#%d' % k)

units += [g for g in sorted(GNDS - FLNETZE) if len(bynet.get(g, [])) > 1]
PLANE_GROUPS = {}
if PLANE_FEEDERS:
    for _n in sorted(PLANE_MIXED_NETS):
        _far=[k for k in bynet.get(_n,[]) if PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y']))>6.]
        if not _far:continue
        PLANE_GROUPS[_n]=sorted(_far,key=lambda k:(PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y'])),k))
        _remove={'GND#%d'%k for k in _far};units=[u for u in units if u not in _remove];units.append(_n)
FIXGND = {'FIX#' + g for g in FLNETZE}
FIXED = []
if 'fix' in args:
    FX = json.load(open(args['fix']))
    if PINZUGANG and any(abs(float(v.get('d',VIA_D))-VIA_D)>1e-9 or abs(float(v.get('drill',VIA_DR))-VIA_DR)>1e-9 for v in FX['vias']):
        raise ValueError('pinzugang: fix= mit gemischten Via-Massen im globalen Routermodell nicht unterstuetzt')
    byn = {}
    for t in FX['tracks']: byn.setdefault(t['net'], ([], []))[0].append(dict(layer=t['layer'], pts=t['pts'], w=t['w']))
    for v in FX['vias']: byn.setdefault(v['net'], ([], []))[1].append(dict(x=v['x'], y=v['y']))
    for n, (tr, vs) in byn.items():

        add_unit('FIX#' + n, n, tr, vs); FIXED.append('FIX#' + n)
    fixnets = set(byn)
    if args.get('fixhart', '1') == '1':
        fo = np.zeros(NL * NN, dtype=bool); fdr = np.zeros(NN, dtype=bool)
        for u in FIXED:
            fo[U[u]['cells']] = True; fdr[U[u]['dcells']] = True
        FIXD[0] = [(_edt(~fo[li * NN:(li + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for li in range(NL)]
        FIXD[1] = (_edt(~fdr.reshape(NY, NX)) * RES).astype(np.float32)
        FIXNETS.update(fixnets); _HM.clear(); _ERR.clear(); _FIX_KARTEN.clear()
        print('feste Kupfer als harte Hindernisse')


    def fix_pad_verbunden(k):
        p = D['pads'][k]; net = p['net']; tr, vs = byn.get(net, ([], []))
        for L in p['layers']:
            g = shapely.union_all([Polygon(q) for q in p['polys'].get(L, [])])
            kupfer = [g] + [_LS((a, b)).buffer(w / 2) for t in tr if t['layer'] == L for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            seen = {0}; offen = [0]
            while offen:
                j = offen.pop()
                if any(kupfer[j].distance(Point(v['x'], v['y'])) <= VIA_R + 1e-7 for v in vs): return True
                for q, h in enumerate(kupfer):
                    if q not in seen and kupfer[j].distance(h) <= 1e-7: seen.add(q); offen.append(q)
        return False
    units = [u for u in units if (not fix_pad_verbunden(int(u[4:])) if u.startswith('GND#') else u not in fixnets)]
    print('fest:', len(FIXED), 'Netze aus', args['fix'])
units.sort(key=grp)
if BGA_FANOUT:

    def bga_ring(u):
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        return min((BGA[id(D['pads'][k])]['ring'] for k in ks if id(D['pads'][k]) in BGA), default=-1)
    units.sort(key=lambda u: (grp(u)[:2], bga_ring(u), grp(u)[2:], u))
if PAAR_NETZE:
    _positions = {u:i for i,u in enumerate(units)}
    units.sort(key=lambda u: (grp(u)[0], -(KL['netze'].get(u) or {}).get('paar_rang',0),
                             min(u,PAAR_NETZE[u]) if u in PAAR_NETZE else '', u if u in PAAR_NETZE else _positions[u]))
print('Einheiten', len(units), '(Netze', sum(1 for u in units if not u.startswith('GND#')), ')')
if KL is not None and (REIHENFOLGE >= 2 or SCHNELL_LAGE > 0 or SCHNELL_INNEN > 0 or SCHNELL_VIA != 1 or GEWICHTET):
    _sn = [u for u in units if ist_schnell(u)]; _lfs = sorted({cls(u).get('lf') for u in _sn if cls(u).get('lf')})
    print('Regeln R1/R14/R21: reihenfolge=%d, schnell_ab=%d (%d schnelle, %d mittelschnelle Netze), schnell_via=%g, gewichtet=%d; Lagen %s' % (
        REIHENFOLGE, SCHNELL_AB, len(_sn), sum(1 for u in units if u in KL['netze'] and KL['netze'][u].get('art') == 'schnell' and not ist_schnell(u)),
        SCHNELL_VIA, GEWICHTET, ', '.join('%s %s' % (L, {2: 'neben Masse', 1: 'neben Flaeche', 0: 'ohne Flaeche'}[LAGE_BEZUG[i]]) for i, L in enumerate(LAYERS))) +
        ('; Kostenfaktor schneller Netze: %s' % ' / '.join(', '.join('%s x%.2f' % (L, f) for L, f in zip(LAYERS, lf)) for lf in _lfs) if _lfs else '; Lagenwahl ohne Wirkung (alle Lagen gleich gut oder schnell_lage=0)'), flush=True)
MAXIT = int(args.get('iter', '40')); HINC = float(args.get('hinc', '0.4'))
PFMAX = float(args['pfmax']) if 'pfmax' in args else None
if PFMAX is not None and (not math.isfinite(PFMAX) or PFMAX <= 0): raise ValueError('pfmax muss endlich und > 0 sein')
if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
t0 = time.time()
fail = set()



CPU_MAX = float(args.get('cpu_max', '0'))
if _LERN_AN:
    args.setdefault('verlauf', os.path.join(os.path.dirname(OUT), 'verlauf.jsonl'))
_VERLAUF = open(args['verlauf'], 'w') if args.get('verlauf') else None
_VERLAUF_CPU = [time.process_time()]
_LERN_ROWS = []
_LERN_STOP = None
_LERN_BASIS = CPU_MAX
_LERN_MAX = float(args.get('lern_cpu_max', str(2 * CPU_MAX))) if args.get('lernbudget') == 'adaptiv' else CPU_MAX
_LERN_POLICY = None
_FRUEH_MODELL = None
_FRUEH_INFO = None
if _FRUEH_AN:
    from lernen import fruehabbruch as _FRUEH
    try:
        _FRUEH.generator_board_pruefen(D)
        _FRUEH_KONTEXT = _FRUEH.kontext_pruefen(
            json.loads(args['fruehabbruch_kontext']) if args.get('fruehabbruch_kontext')
            else _FRUEH.kontext_board(D, NL))
        _FRUEH_MODELL = _FRUEH.modell_laden(_FRUEH.modellpfad(args))
    except (OSError, ValueError, KeyError, TypeError) as _e:
        print('Fruehabbruch gesperrt: Modell-/Kontextfehler %s' % repr(_e)[:180], flush=True)
if _LERN_AN and args.get('lernpolicy'):
    _LERN_POLICY = json.load(open(args['lernpolicy']))
    if _LERN_POLICY.get('version') != 1 or _LERN_POLICY.get('status') != 'experimentell':
        raise ValueError('lernpolicy: version=1, status=experimentell erforderlich')


def ausgabe_geometrie():
    tracks = []; vias = []
    for u, r in U.items():
        if args.get('rest_reparatur') == '1':
            tracks.extend(dict(t,net=r['net'],**({'strom_steg': True} if t.get('steg') else {})) for t in r['tracks'])
            vias.extend(dict(v,net=r['net'],**({'strom_via': True} if v.get('gr') else {})) for v in r['vias'])
            continue
        for t in r['tracks']: tracks.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w'], sig=bool(t.get('sig')), **({'strom_steg': True} if t.get('steg') else {}), **({'kette': True} if t.get('kette') else {})))
        for v in r['vias']: vias.append(dict(net=r['net'], x=v['x'], y=v['y'], strom_via=True) if v.get('gr') else
                                        (dict(net=v['net'], x=v['x'], y=v['y'], rueck_via=True) if v.get('rv') else dict(net=r['net'], x=v['x'], y=v['y'])))
    if args.get('kurzschluss_schutz', '0') == '1':
        for q, u in zip(tracks, [u for u,r in U.items() for t in r['tracks']]): q['einheit'] = u if u.startswith('GND#') else q['net']
        for q, u in zip(vias, [u for u,r in U.items() for v in r['vias']]): q['einheit'] = u if u.startswith('GND#') else q['net']
    for _t, _q in zip(tracks, [t for r in U.values() for t in r['tracks']]):
        if _q.get('abg'): _t['kelvin'] = True
    if globals().get('ZONENPLAN'): vias += _ZP_VIAS
    for t in tracks:
        if _pz_active(t['net']):
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
            t['nm_pts']=[[_pz_nm(v) for v in p] for p in t['pts']]
            t['nm_w']=[_pz_nm(w) for w in t['w']]
    for v in vias:
        if _pz_active(v['net']):
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
            v['nm_xy']=[_pz_nm(v['x']),_pz_nm(v['y'])]
    return tracks, vias


def ausgabe_kontakte(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.kurzschluss_schutz import ausgabe_kontakte as impl
    return impl(*pos, **kw)


def ausgabe_auswahl(*pos, **kw):
    from nachbearbeitung.kurzschluss_schutz import ausgabe_auswahl as impl
    return impl(*pos, **kw)


def ausgabe_luftlinien(*pos, **kw):
    from nachbearbeitung.kurzschluss_schutz import ausgabe_luftlinien as impl
    return impl(*pos, **kw)


def ausgabe_sicher(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.kurzschluss_schutz import ausgabe_sicher as impl
    return impl(*pos, **kw)



def ausgabe_daten(tracks, vias, conf, fehlend=None, checkpoint=False, roh=False, **extra):
    ""
    data = dict(tracks=tracks, vias=vias, failed=sorted(fail if fehlend is None else fehlend),
        conflicts=conf, res=RES, clr=CLR, optionen=args,
        unerreichbar={n:v for n,v in sorted(UNERREICHBAR.items())},
        flaechennetze=sorted(FLNETZE), gndlagen=GNDL, unter_wmin=unter_wmin())
    if ARBEIT_MESSEN: data['arbeitsbudget'] = arbeit_beleg()
    if KA_VERTRAG is not None:
        data['abstaende'] = KA_VERTRAG
        data['abstand_legacy_deckt'] = KA_LEGACY_DECKT
    if PINZUGANG:
        data['pinzugang_exakt']=dict(breiten=PZ_BREITEN,minimax=globals().get('PZ_MINIMAX'),kanten=PZ_EDGE_STAT,lokal={n:len(ps) for n,ps in sorted(PZ_LOKAL.items())},rand_nm=PZ_RAND_NM)
    if globals().get('BREIT_DICHT_BERICHT') is not None: data['breit_dicht'] = BREIT_DICHT_BERICHT
    if globals().get('STROM_ZUERST'):
        data['strom_zuerst'] = dict(netze=STROM_AUDIT, potenziallagen=STROM_POT, J=STROM_J,
            ohne_modell=sorted(n for n,k in KL['netze'].items() if k.get('art') in ('strom','versorgung','sw') and not strom_modell(n)))
    if globals().get('ZONENPLAN'): data.update(zonenplan_id=ZONENPLAN['id'], zonen=ZONENPLAN['kern'], native_vias=True)
    if PLANE_INFO: data['plane_domains'] = PLANE_INFO
    if PLANE_GROUPS:data['plane_groups']={n:['%s:%s'%(D['pads'][k]['ref'],D['pads'][k]['num']) for k in ks] for n,ks in sorted(PLANE_GROUPS.items())}
    if POTENZIAL_PLAN: data['potenzial_plan'] = POTENZIAL_PLAN
    if IMP_TAB: data['impedanz_tabelle'] = IMP_TAB
    if FIX_PAIRS: data['fix_abstaende'] = FIX_PAIRS
    if args.get('fix'):
        data['fix_netze'] = sorted({r['net'] for u, r in U.items() if u.startswith('FIX#')})
    gruppen = []
    for r in U.values():
        for v in r['vias']:
            if v.get('n'):
                gruppen.append(dict(netz=r['net'], x=v['x'], y=v['y'], soll=v['n'],
                    ist=1 + sum(1 for w in r['vias'] if w.get('zu') == [v['x'], v['y']])))
    if gruppen: data['via_gruppen'] = gruppen
    for key, name, enabled in (
        ('r19_kurz', 'R19_KURZ_STAT', globals().get('R19_KURZ')),
        ('r19_form', 'R19_FORM_STAT', globals().get('R19_FORM')),
        ('laengenabgleich', 'LAENGEN_BERICHT', globals().get('LAENGEN_BERICHT') is not None),
        ('massepad_vias', 'MPV', True), ('via_neben_pad', 'VAPB', True),
        ('rueck_vias', 'RVB', True), ('paare', 'PAB', True),
        ('vias_sparen', 'VS_LOG', args.get('vias_sparen') == '1'),
        ('via_feinschub', 'FEINSCHUB_LOG', args.get('via_feinschub') == '1'),
        ('via_dichte_nach', 'VD_NACH_LOG', globals().get('VD_NACH')),
        ('bga_fanout', 'BGB', globals().get('BGA_FANOUT'))):
        if enabled and name in globals() and (globals()[name] or key in ('laengenabgleich', 'vias_sparen', 'via_feinschub', 'bga_fanout')):
            data[key] = globals()[name]
    if globals().get('ST') and ST.get('rest_board'):
        if args.get('neu'):
            from nachbearbeitung.restkonflikte import placement_marker
            data.update(placement_marker(ST['rest_board'],copy.deepcopy(_REST_BOARD_SOURCE)))
        else:
            for _field in ('rest_board','rest_quelle','rest_moves','rest_placement','rest_native_sources'):
                if ST.get(_field) is not None: data[_field] = ST[_field]
    if args.get('restkonflikte') == '1' and globals().get('ST'):
        for _field in ('zones','zonen','r41_fuellung_lagen'):
            if ST.get(_field): data[_field] = ST[_field]
    if globals().get('REST_BERICHT') is not None: data['restkonflikte'] = REST_BERICHT
    if globals().get('_RL') is not None:
        data['routing_loop'] = _RL.summary()
        if _RL.board(D) != _RL.initial_board:
            data.update(_RL.rest.placement_marker(_RL.initial_board, _RL.board(D)))
    data.update(extra)
    if globals().get('_FRUEH_INFO') is not None: data['fruehabbruch'] = _FRUEH_INFO




    if roh or args.get('kurzschluss_schutz', '0') != '1': return data
    return r44_ausgabe(data, checkpoint=checkpoint)
def r44_ausgabe(data, checkpoint=False, board=None, paare=None, einheiten_extra=None):
    ""
    board = D if board is None else board
    einheiten = {u:r['net'] for u,r in U.items()}
    for u, n in (einheiten_extra or {}).items(): einheiten.setdefault(u, n)
    ausgabenetze = {q['net'] for k in ('tracks', 'vias', 'zonen', 'zones') for q in data.get(k, [])}
    wichtig = {n: max(1, 20 - int(cls(n).get('prio', 10))) for n in ausgabenetze}
    wichtig.update({u:wichtig.get(n, 1) for u,n in einheiten.items() if u.startswith('GND#')})
    result = ausgabe_sicher(board, data, einheiten, wichtig, list(PAIRS) if paare is None else list(paare), checkpoint=checkpoint)
    if checkpoint:
        result = dict(result)
        result['kurzschluss_einheiten'] = {u: (board['pads'][int(u[4:])]['net'] if u.startswith('GND#') else einheiten.get(u, u[4:] if u.startswith('FIX#') else u)) for u in result['failed']}
    return result


def rest_abschluss(result):
    ""
    paare = result.pop('rest_check_paare', None)
    if args.get('kurzschluss_schutz', '0') != '1': return result
    if result.get('restkonflikte') is not None:
        result['restkonflikte']['vor_r44'] = dict(konflikte=len(result['conflicts']), offen=len(result['failed']))
    board = result.get('rest_board') or D

    extra = {}
    for u in list(result['conflicts']) + [q for p in (paare or ()) for q in p[2:4]]:
        if u in extra or u == 'PAD': continue
        if u.startswith('FIX#'): extra[u] = u[4:]
        elif u.startswith('GND#'): extra[u] = board['pads'][int(u[4:])]['net']
        else: extra[u] = u
    return r44_ausgabe(result, board=board, paare=paare, einheiten_extra=extra)


_LERN_CPU = 0.0
_LERN_TIEFE = 0


def beobachter_cpu():
    return _LERN_CPU + (_ET.cpu if _ET else 0.0)


def lern_beobachter(fn):
    ""
    def messen(*pos, **kw):
        global _LERN_CPU, _LERN_TIEFE
        if not _LERN_AN and _VERLAUF is None: return fn(*pos, **kw)
        aussen = _LERN_TIEFE == 0
        start = time.process_time(); etik = _ET.cpu if _ET else 0.0
        _LERN_TIEFE += 1
        try: return fn(*pos, **kw)
        finally:
            _LERN_TIEFE -= 1
            if aussen:
                _LERN_CPU += max(0.0, time.process_time() - start - ((_ET.cpu if _ET else 0.0) - etik))
    return messen


@lern_beobachter
def lern_snapshot(runde, conf, phase):
    global _LERN_SNAPSHOT


    tracks, vias = ausgabe_geometrie()
    data = ausgabe_daten(tracks, vias, {u: len(c) for u,c in conf.items()}, checkpoint=True,
        fehlend=set(fail) | (set(units) - set(U) - set(FIXED)),
        lernstand=dict(runde=runde, phase=phase, cpu_s=time.process_time(), paare=list(PAIRS), zensiert=True),
        lernbudget=dict(basis_cpu_s=_LERN_BASIS, aktuell_cpu_s=CPU_MAX, max_cpu_s=_LERN_MAX))


    import signal
    oldmask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGXCPU})
    try:
        if _ET:
            _ET.beobachten(conf, runde, phase)
            _ET.schreiben(data['tracks'], data['vias'], nur_cache=True, fehlend=data['failed'], zonen=data.get('zonen', []) + data.get('zones', []))
        _LERN_SNAPSHOT = json.dumps(data)
    finally: signal.pthread_sigmask(signal.SIG_SETMASK, oldmask)


def lern_entscheidung():
    global CPU_MAX, _LERN_STOP, _FRUEH_INFO
    if not _LERN_AN: return
    if globals().get('_FRUEH_MODELL') is not None and args.get('lernbudget') != 'adaptiv':
        _info = _FRUEH.entscheidung(_LERN_ROWS, _FRUEH_KONTEXT, CPU_MAX, _FRUEH_MODELL)
        if _info:
            _FRUEH_INFO = _info
            _LERN_STOP = 'fruehabbruch'
            print('Fruehabbruch: Runde %d, %d Konflikte, Score %.6f >= %.6f, %.1f CPU-s' %
                  (_info['runde'], _info['konflikte'], _info['score'], _info['schwelle'], _info['cpu_s']), flush=True)
    if _LERN_POLICY or args.get('lernbudget') == 'adaptiv':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from lernen.lerneffekt import entscheidung, vielversprechend
        if _LERN_POLICY and entscheidung(_LERN_ROWS, len(D['pads']), _LERN_POLICY):
            _LERN_STOP = 'fruehabbruch'
        elif CPU_MAX > 0 and time.process_time() >= .65 * CPU_MAX and CPU_MAX < _LERN_MAX and vielversprechend(_LERN_ROWS):
            CPU_MAX = min(_LERN_MAX, max(CPU_MAX * 1.5, time.process_time() / .6))
            print('Lernbudget: sinkende Konflikte, verlaengert auf %.1f CPU-s' % CPU_MAX, flush=True)


@lern_beobachter
def verhandlung_messen(runde, conf, phase='verhandlung'):
    if _VERLAUF is None: return
    cpu = time.process_time()
    cells = np.unique(np.concatenate([np.asarray(c, dtype=np.int64) for c in conf.values()])) if conf else np.empty(0, dtype=np.int64)
    h = hist[cells]
    row = dict(runde=runde, phase=phase, cpu_s=cpu, cpu_runde_s=cpu - _VERLAUF_CPU[0], pf=float(PF[0]),
               konflikte=len(conf), offen=len(fail), paare=list(PAIRS),
               hist_max=float(hist.max()), hist_konflikt_max=float(h.max()) if len(h) else 0.0,
               hist_konflikt_mittel=float(h.mean()) if len(h) else 0.0,
               hist_konflikt_null=float(np.mean(h == 0)) if len(h) else 0.0,
               netze=sorted(conf), vias=sum(len(r['vias']) for r in U.values()))
    _VERLAUF.write(json.dumps(row) + '\n'); _VERLAUF.flush(); _VERLAUF_CPU[0] = time.process_time()
    if _LERN_AN:
        _LERN_ROWS.append(row)
        lern_snapshot(runde, conf, phase)
        if phase == 'verhandlung': lern_entscheidung()

if args.get('etiketten'):
    _etik_c0 = time.process_time()
    try:
        _ET = _Etiketten(); _ET.cpu += time.process_time() - _etik_c0
    except Exception as _e: print('Etiketten: Initialisierung ausgelassen:', repr(_e)[:180], flush=True)


@lern_beobachter
def erstaufbau_checkpoint():

    verhandlung_messen(0, exact_conflicts(), 'erstaufbau')


def such_cpu():


    return time.process_time() + (getattr(globals().get('_AL'), 'cpu_offset', 0.))


def budget_meldung():
    if BUDGET_ARBEIT:
        return 'Arbeitsbudget (arbeit_max=%d, verbraucht=%d Auftraege)' % (ARBEIT_MAX, arbeit_gesamt())
    return 'Rechenzeit-Budget (cpu_max=%.0f s)' % CPU_MAX

def zeit_knapp(anteil=0.8):
    if BUDGET_ARBEIT:

        arbeit_notbremse()
        return arbeit_gesamt() >= anteil * ARBEIT_MAX
    if _LERN_AN and _LERN_STOP: return True
    if _LERN_AN and (args.get('lernbudget') == 'adaptiv' or _LERN_POLICY):
        return CPU_MAX > 0 and time.process_time() > anteil * CPU_MAX


    return CPU_MAX > 0 and (such_cpu() - beobachter_cpu()) > anteil * CPU_MAX


todo = None
if _LERN_AN: lern_snapshot(0, {}, 'vor_routing')
if _AGENT_SAVED is not None:
    pass
elif 'start' in args:

    ST = _REST_START_SOURCE if _REST_START_SOURCE is not None else json.load(open(args['start']))
    byn = {}
    for t in ST['tracks']: byn.setdefault(t['net'], ([], []))[0].append(t)
    for v in ST['vias']:
        _sv = dict(v) if args.get('rest_reparatur') == '1' else dict(x=v['x'], y=v['y'])
        if v.get('strom_via'): _sv['gr'] = 1
        byn.setdefault(v['net'], ([], []))[1].append(_sv)


    NEU = {r for r in args.get('neu', '').split(',') if r}
    neu_nets = {D['pads'][k]['net'] for k in range(len(D['pads'])) if D['pads'][k]['ref'] in NEU} - FLNETZE - {''}
    neu_nets |= {n for n in args.get('neu_netze', '').split(',') if n}
    neu_nets = {n for n in neu_nets if n in bynet and len(bynet[n]) > 1}
    zone = [(p['x'], p['y']) for p in D['pads'] if p['ref'] in NEU]
    gnd_neu = [k for k, p in enumerate(D['pads']) if p['net'] in FLNETZE and
               (p['ref'] in NEU or any(math.hypot(p['x'] - a, p['y'] - b_) < 1.5 for a, b_ in zone))]


    fail |= {n for n in (ST.get('failed') or []) if n in units and n not in neu_nets} - {'GND#%d' % k for k in gnd_neu}
    def in_pad(x, y, ks):
        for k in ks:
            bb = D['pads'][k]['bbox']
            if bb[0] - 0.05 <= x <= bb[2] + 0.05 and bb[1] - 0.05 <= y <= bb[3] + 0.05: return True
        return False
    for n, (tr, vs) in byn.items():
        if n in neu_nets: continue
        if n in FLNETZE:
            weg_v = set()
            tr_keep = []
            for t in tr:
                ends = [tuple(t['pts'][0]), tuple(t['pts'][-1])]
                if any(in_pad(x, y, gnd_neu) for x, y in ends):
                    for x, y in ends: weg_v.add((round(x, 3), round(y, 3)))
                    continue
                tr_keep.append(t)
            vs_keep = [v for v in vs if (round(v['x'], 3), round(v['y'], 3)) not in weg_v and not in_pad(v['x'], v['y'], gnd_neu)]
            add_unit('FIX#' + n, n, [dict(t) if args.get('rest_reparatur') == '1' else dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr_keep], vs_keep); FIXED.append('FIX#' + n)
            continue
        C = cls(n); Cs = cls('/__signal__')


        _wk = lambda t: (Cs if t.get('sig') else C)['wmin']
        tr2 = [dict(t) if args.get('rest_reparatur') == '1' else dict(net=n, layer=t['layer'], pts=t['pts'], w=[(wi if args.get('rest_pruefung') == '1' else min(wi, _wk(t))) for wi in t['w']], sig=bool(t.get('sig')), **({'steg': True} if t.get('strom_steg') else {})) for t in tr]
        add_unit(n, n, tr2, vs)
        for _t2, _t in zip(tr2, tr):
            if _t.get('kelvin'): _t2['abg'] = True
    if args.get('rest_check') == '1':
        _REST_LOKAL_KONFLIKT = True
        _rc = exact_conflicts(); _rt, _rv = ausgabe_geometrie()
        json.dump(ausgabe_daten(_rt, _rv, {u: len(v) for u,v in _rc.items()}, roh=True, rest_check_paare=list(PAIRS)), open(OUT, 'w'))
        sys.exit(0)
    if args.get('rest_reparatur') == '1':
        from nachbearbeitung.restkonflikte import reparatur_ausgabe
        reparatur_ausgabe(globals())
        sys.exit(0)
    if NEU or neu_nets:

        neu_units = sorted(neu_nets, key=lambda n: (grp(n), n)) + ['GND#%d' % k for k in gnd_neu]
        for u in paar_reihe(neu_units):
            if route_unit(u): fail.difference_update(paar_netze([u]))
            else:
                if paar_einheit(u): fail.update((u,PAAR_NETZE[u]))
                else: fail.add(u)
        if args.get('rest_teil_lokal') == '1':


            _REST_LOKAL_KONFLIKT = True
            _rc=exact_conflicts();_rt,_rv=ausgabe_geometrie()
            json.dump(ausgabe_daten(_rt,_rv,{u:len(v) for u,v in _rc.items()},roh=True,rest_check_paare=list(PAIRS)),open(OUT,'w'))
            sys.exit(0)
        units = [u for u in units if not u.startswith('GND#')] + ['GND#%d' % k for k in gnd_neu]
        args['alle'] = '0'
        print('Start aus', args['start'], ': neu verlegt', len(neu_units), 'Einheiten fuer', sorted(NEU), flush=True)
    else:
        units = [u for u in units if not u.startswith('GND#')]
        MAXIT = (MAXIT if args.get('loop') == '1' or args.get('agent_loop') == '1' else (2 if PAAR_SUCHRAUM else 1)); conf = exact_conflicts(); todo = (set(conf) | fail | ({u for u in units if paar_einheit(u)} if PAAR_SUCHRAUM else set())) - set(FIXED)
        print('Start aus', args['start'], ':', len(U), 'Einheiten geladen', flush=True)
else:
    _lern_cp_cpu = time.process_time()
    _reihe = paar_reihe(units)
    for _i, u in enumerate(_reihe):
        if (_LERN_AN and args.get('lernbudget') == 'adaptiv' and CPU_MAX > 0
                and CPU_MAX < _LERN_MAX and _i >= max(1, len(_reihe) // 2)
                and time.process_time() >= .65 * CPU_MAX):



            CPU_MAX = min(_LERN_MAX, max(CPU_MAX, time.process_time() * len(_reihe) / _i / .65))
            print('Lernbudget: Erstaufbau-Fortschritt, %.1f CPU-s' % CPU_MAX, flush=True)
        if zeit_knapp():
            rest = paar_netze(_reihe[_i:]); fail.update(rest); print('%s im ersten Durchgang verbraucht: %d Einheiten nicht verlegt' % (budget_meldung(), len(rest)), flush=True); break
        if not route_unit(u):
            if paar_einheit(u): fail.update((u,PAAR_NETZE[u]))
            else: fail.add(u)
        if u in PAAR_NETZE and u > PAAR_NETZE[u] and u not in fail and PAAR_NETZE[u] not in fail:
            paar_koppeln(nur={u, PAAR_NETZE[u]})
        if _LERN_AN and time.process_time() - _lern_cp_cpu >= 10:
            erstaufbau_checkpoint()
            _lern_cp_cpu = time.process_time()
print('Iteration 1: %.0fs, unvollstaendig %d' % (time.time() - t0, len(fail)))
class _RoutingLoop:
    VERSION = 'routing-loop-v4-local-cpu'

    def __init__(self, G):
        import hashlib
        from nachbearbeitung import restkonflikte as rest
        from analyse.bewertung import bewerte
        self.G, self.rest, self.bewerte = G, rest, bewerte
        self.initial_board = self.board(G['D'])
        a = G['args']
        G['_LOOP_START_CPU'] = time.process_time()
        code=Path(G['__file__']).resolve();root=code.parents[1]
        self.quellen={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in
                      [('router',code),('klassen_code',root/'analyse/klassen.py')]+
                      [(name,Path(a[name])) for name in ('klassen','regeln') if a.get(name)]}
        self.quellen['board']=hashlib.sha256(json.dumps(self.initial_board,sort_keys=True).encode()).hexdigest()
        self.fraction = self.number(a, 'loop_anteil', .10, 0., .5)
        self.explicit_budget = 'loop_cpu' in a
        self.limit = self.number(a, 'loop_cpu', self.budget(G, self.fraction), 0., 86400.)
        self.move_cpu = self.number(a, 'loop_zug_cpu', 5., .01, 300.)
        self.large = len(G['units']) >= 400
        self.threshold = int(self.number(a, 'loop_schwelle', 80, 1, 1000000))
        if a.get('loop_karten','lokal') not in ('lokal','voll'):
            raise ValueError('loop_karten=lokal|voll')
        if a.get('loop_suche','begrenzt') not in ('begrenzt','numba'):
            raise ValueError('loop_suche=begrenzt|numba')
        self.counts = []
        self.prefiltered = {}; self.rejected = {}
        self.k = int(self.number(a, 'loop_k', 4, 1, 100))
        self.n = int(self.number(a, 'loop_n', 4, 1, 100))
        self.cap = int(self.number(a, 'loop_kand', 24, 0, 256))
        self.radius = self.number(a, 'loop_radius', .8, .2, 3.)
        self.used = 0.; self.previous = None; self.stale = 0; self.cells = {}
        self.seen = set(); self.rounds = 0; self.accepted = {}; self.trials = 0
        self.accepted_moves = []
        self.disabled = None
        self.log = Path(a.get('loop_log', G['OUT'].replace('.json', '') + '_loop.jsonl'))
        self.exchange = Path(a.get('loop_zustand', G['OUT'].replace('.json', '') + '_engstellen'))
        if not G['KL']: self.disabled = 'loop braucht klassen='
        source = dict(G.get('ST') or {}, optionen=a)
        self.disabled = self.disabled or rest.reparatur_sperre(source)
        if any(source.get(k) for k in ('zones','zonen','r41_fuellung_lagen')):
            self.disabled = 'native Flaechen brauchen abgestimmten Kupferzustand'
        if G.get('PAAR_NETZE') or G.get('ZONENPLAN'):
            self.disabled = 'Paarnetze/Zonenplan brauchen abgestimmten Gruppenzug'
        self.input_path = a.get('loop_zuege')
        self.external = []
        if self.input_path:
            payload = json.loads(Path(self.input_path).read_text())
            if not isinstance(payload, list) or len(payload) > 256:
                raise ValueError('loop_zuege: JSON-Liste mit hoechstens 256 Zuegen erwartet')
            self.external = [self.validate_move(m) for m in payload]
        self.emit(dict(ereignis='start', standard_aus=True, grund=self.disabled,
                       budget_cpu_s=self.limit, argv=list(sys.argv)))

    @staticmethod
    def budget(G, fraction):


        total = G.get('CPU_MAX', 0)
        if total > 0: return min(86400., fraction * total)
        return min(86400., max(20., .05 * len(G['units']),
                              fraction * max(0., time.process_time() - G.get('_LOOP_START_CPU', time.process_time()))))

    @staticmethod
    def board(B):
        result = copy.deepcopy({k:v for k,v in B.items() if k != 'pads'})
        result['pads'] = [copy.deepcopy({k:v for k,v in p.items() if k not in ('cells','bcells')}) for p in B['pads']]
        return result

    @staticmethod
    def number(a, key, default, lo, hi):
        v = float(a.get(key, default))
        if not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('%s muss zwischen %s und %s liegen' % (key, lo, hi))
        return v

    @staticmethod
    def validate_move(m):
        if isinstance(m,dict) and m.get('art') in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import pruefe
            return pruefe(m)
        if not isinstance(m, dict) or m.get('art') not in ('via', 'teil', 'lage', 'umlegen'):
            raise ValueError('Unbekannte Loop-Zugart')
        fields = dict(via={'art','einheit','via','ziel','zustand'},
                      teil={'art','ref','delta','zustand'},
                      lage={'art','einheit','bahn','segment','lage','zustand'},
                      umlegen={'art','einheit','zustand'})
        if set(m) != fields[m['art']]: raise ValueError('Fehlende/unbekannte Zugfelder')
        if not isinstance(m['zustand'], str) or len(m['zustand']) != 64 or any(x not in '0123456789abcdef' for x in m['zustand']):
            raise ValueError('Zug braucht SHA-256 des Engstellenzustands')
        if m['art'] == 'teil':
            v = m['delta']
            if not isinstance(m['ref'],str) or not isinstance(v,list) or len(v)!=3:
                raise ValueError('Teilzug braucht ref und delta=[dx,dy,drehung]')
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in v): raise ValueError('Ungueltiges delta')
            if math.hypot(*v[:2]) > .500001 or v[2] not in (0,90,180,270): raise ValueError('Teilzug ausserhalb 0,5 mm/90 Grad')
        else:
            if not isinstance(m['einheit'],str): raise ValueError('einheit fehlt')
            for key in ('via','bahn','segment'):
                if key in m and (type(m[key]) is not int or m[key]<0): raise ValueError('Ungueltiger Index')
            if m['art']=='via' and (not isinstance(m['ziel'],list) or len(m['ziel'])!=2 or
                    any(type(x) not in (int,float) or not math.isfinite(x) for x in m['ziel'])): raise ValueError('Ungueltiges Viaziel')
            if m['art']=='lage' and not isinstance(m['lage'],str): raise ValueError('Ungueltige Lage')
        return copy.deepcopy(m)

    def emit(self, row):
        def finite(v):
            if isinstance(v,float) and not math.isfinite(v):return None
            if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
            if isinstance(v,(list,tuple)):return [finite(x) for x in v]
            return v
        row = dict(version=self.VERSION, quellen=self.quellen, **row)
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open('a') as f:
            f.write(json.dumps(finite(row), sort_keys=True, allow_nan=False) + '\n'); f.flush()

    def summary(self):
        return dict(version=self.VERSION, cpu_s=round(self.used,5), versuche=self.trials,
                    uebernommen=dict(self.accepted), grund=self.disabled,
                    abgelehnt=dict(self.rejected), vorgefiltert=dict(self.prefiltered),
                    budget_cpu_s=self.limit, zug_budget_cpu_s=self.move_cpu,
                    protokoll=str(self.log), engstellen=str(self.exchange),quellen=self.quellen)

    def out(self):
        G=self.G; c=G['exact_conflicts'](); ts,vs=G['ausgabe_geometrie']()
        return G['ausgabe_daten'](ts,vs,{u:len(v) for u,v in c.items()},roh=True), list(G['PAIRS'])

    def digest(self):
        import hashlib
        G=self.G
        io={'loop_zuege','loop_log','loop_zustand','etiketten','png','verlauf','klassen','regeln','start','fix','weiter','zwischenstand','agent_halt','agent_runde','agent_pakete','agent_zuege'}
        state=dict(board=self.board(G['D']), einheiten={u:{k:r[k] for k in ('net','tracks','vias')} for u,r in sorted(G['U'].items())}, failed=sorted(G['fail']),
                   quellen=self.quellen,lagen=G['LAYERS'],res=G['RES'],fest=sorted(G['FIXED']),optionen={k:v for k,v in G['args'].items() if k not in io})
        if G['args'].get('agent_loop') == '1':
            state['verhandlung']=dict(hist_sha256=hashlib.sha256(G['hist'].tobytes()).hexdigest(),
                praesenz=G['PF'],einheiten_reihenfolge=list(G['U']),
                vorgaben=G.get('_AGENT_VORGABEN',[]),prioritaet=G.get('_AGENT_PRIORITAET',[]))
        return hashlib.sha256(json.dumps(state,sort_keys=True,allow_nan=False).encode()).hexdigest()

    def clusters(self, pairs):


        from scipy.spatial import cKDTree
        if not pairs: return []
        xy=np.asarray([(p[4],p[5]) for p in pairs]);tree=cKDTree(xy)
        balls=tree.query_ball_point(xy,self.radius);remaining=set(range(len(pairs)));groups={}



        order=sorted(remaining,key=lambda i:(-len(balls[i]),pairs[i][4],pairs[i][5],pairs[i][0],pairs[i][2],pairs[i][3],pairs[i][1]))
        for i in order:
            if i not in remaining:continue
            members=sorted(set(balls[i])&remaining,key=lambda j:tuple(pairs[j]))
            remaining.difference_update(members);groups[i]=[pairs[j] for j in members]
        G=self.G; result=[]
        for ps in groups.values():
            points=sorted({(p[4],p[5]) for p in ps}); us=sorted({u for p in ps for u in p[2:4] if u in G['U']})
            xs,ys=zip(*points); bounds=[min(xs)-self.radius,min(ys)-self.radius,max(xs)+self.radius,max(ys)+self.radius]
            pads=[i for i,p in enumerate(G['D']['pads']) if any(math.dist((p['x'],p['y']),q)<=self.radius for q in points)]
            vias=[dict(einheit=u,via=i,netz=v.get('net') or r['net'],x=v['x'],y=v['y'],beteiligt=u in us,
                       gruppenschutz=any(v.get(k) for k in ('n','zu','gr','rv','strom_via','rueck_via')))
                  for u,r in sorted(G['U'].items()) for i,v in enumerate(r.get('vias',[]))
                  if any(math.dist((v['x'],v['y']),q)<=self.radius for q in points)]
            history=[]
            for p in ps:
                ix=int((p[4]-G['X0'])/G['RES']);iy=int((p[5]-G['Y0'])/G['RES'])
                if 0<=ix<G['NX'] and 0<=iy<G['NY']:
                    history.append(float(G['hist'].reshape(G['NL'],G['NY'],G['NX'])[:,iy,ix].max()))
            result.append(dict(paare=ps, punkte=points, einheiten=us, bbox=bounds,
                               netze=sorted({G['U'][u]['net'] for u in us}), pads=pads,
                               bauteile=sorted({G['D']['pads'][i]['ref'] for i in pads}),vias=vias,
                               historie=max(history,default=0)))
        return sorted(result,key=lambda c:(-len(c['paare']),-c['historie'],c['bbox']))

    def movable(self,u,via=False):
        G=self.G
        if u not in G['U'] or u.startswith('HEFT#'): return False
        if u in G['FIXED'] or u.startswith('FIX#'):

            return via and u.startswith('FIX#') and 'start' in G['args'] and not G['args'].get('fix') and G['U'][u]['net'] in G['FLNETZE']
        return True

    def neighbours(self,c):
        G=self.G
        if 'nachbarn' in c:
            return [u for u in c['nachbarn'] if u in G['U'] and self.movable(u)]
        return [u for u,r in G['U'].items() if self.movable(u) and G['_nah_an'](r,c['punkte'],self.radius)]

    def part_units(self,ref):
        G=self.G
        nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}-G['FLNETZE']
        touched={u for u,r in G['U'].items() if r['net'] in nets and self.movable(u)}
        if G.get('_AL') is not None:
            touched.update(u for u in G['units'] if u in nets and u not in G['FIXED'])


        touched.update('GND#%d'%i for i,p in enumerate(G['D']['pads'])
                       if p['ref']==ref and p['net'] in G['FLNETZE']
                       and 'GND#%d'%i in G['units'] and self.movable('GND#%d'%i))
        return touched

    def part_moves(self,c):
        ""
        import re
        from gemeinsam import wanderweite as WW
        G=self.G;B=G['D'];W=WW.tabelle(G['KL']);fps={f['ref']:f for f in B['fps']}
        raw=B.get('platzierungsgruppen') or []
        groups=raw.values() if isinstance(raw,dict) else raw
        grouped={r for g in groups if isinstance(g,dict) for r in g.get('refs',[])}
        if not c['punkte']:return
        for ref in sorted(fps,key=lambda r:(min((math.dist((fps[r]['x'],fps[r]['y']),p) for p in c['punkte']),default=math.inf),r)):
            f=fps[ref];t=G['KL']['teile'].get(ref,{})
            if (not re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp',''))
                    or ref in grouped or f.get('locked') or t.get('fest') or t.get('rolle')!='klein'
                    or ref not in W or W[ref]['max']<=0):continue
            nearest=min(c['punkte'],key=lambda p:math.dist((f['x'],f['y']),p))
            ax,ay=f['x']-nearest[0],f['y']-nearest[1]
            if math.hypot(ax,ay)>1.5:continue
            direction=(0,1 if ay>=0 else -1) if abs(ay)>=abs(ax) else (1 if ax>=0 else -1,0)
            preferred=[(round(direction[0]*d,5),round(direction[1]*d,5),0) for d in (.1,.2,.3,.4,.5)]
            rotations=[(0,0,th) for th in (90,180,270)] if W[ref]['drehen'] else []
            origin=(B.get('wander_basis') or {}).get(ref) or f
            if isinstance(origin,(list,tuple)):origin=dict(x=origin[0],y=origin[1])
            for dx,dy,th in preferred+rotations+[(x,y,0) for x,y in self.rest.offsets(.5) if (x,y,0) not in preferred]:
                if math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[ref]['max']+1e-8:continue
                yield dict(ref=ref,moves={ref:(dx,dy,th)})

    def external_local(self,m,c):
        G=self.G
        if m['art'] in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import lokal
            return lokal(G,m,c,self.radius)
        if m['art']=='teil':
            f=next((f for f in G['D']['fps'] if f['ref']==m['ref']),None)
            return f is not None and any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['punkte'])
        r=G['U'].get(m['einheit'])
        if r is None:return False
        try:
            if m['art']=='via':
                v=r['vias'][m['via']]
                return any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['punkte'])
            if m['art']=='lage':
                from shapely.geometry import LineString
                t=r['tracks'][m['bahn']];j=m['segment'];line=LineString(t['pts'][j:j+2])
                return any(line.distance(Point(p))<=self.radius for p in c['punkte'])
        except (IndexError,ValueError):return False
        return G['_nah_an'](r,c['punkte'],self.radius)

    def part_board(self,m,c):
        ""
        import re
        from gemeinsam import wanderweite as WW
        from gemeinsam.teile import lade_teile,platine,Belegung
        from gemeinsam.verschieben import bewegen
        from nachbearbeitung.zugregeln import sperre
        G=self.G;B=self.board(G['D']);r=m['ref'];dx,dy,th=m['delta']
        fps={f['ref']:f for f in B['fps']};f=fps.get(r);t=G['KL']['teile'].get(r,{})
        groups=B.get('platzierungsgruppen') or []
        if isinstance(groups,dict):groups=groups.values()
        grouped={ref for group in groups if isinstance(group,dict) for ref in group.get('refs',[])}
        W=WW.tabelle(G['KL'])
        chip=bool(G.get('_AL') is not None and f and re.search(r'(?:^|[_:])(?:SOT|SOIC)[-_]',f.get('fp',''),re.I))
        small=bool(f and re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp','')))
        if f is None or not (small or chip) or r in grouped or f.get('locked') or t.get('fest') or t.get('rolle') not in (('klein','chip') if chip else ('klein',)) or r not in W:
            raise ValueError('Teil ist kein freies C/R oder freigegebenes SOT/SOIC')
        if chip and not any(q['ref']==r for q in B.get('courtyards',[])):raise ValueError('SOT/SOIC braucht exportierten Courtyard')
        if not any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['punkte']):raise ValueError('Teil nicht lokal')
        if math.hypot(dx,dy)>.500001 or th not in (0,90,180,270) or (th and not (W[r]['drehen'] or chip)):raise ValueError('Teilzug ausserhalb Bewegungsvertrag')
        origin=(B.get('wander_basis') or {}).get(r) or next(q for q in self.initial_board['fps'] if q['ref']==r)
        if isinstance(origin,(tuple,list)):origin=dict(x=origin[0],y=origin[1])
        if W[r]['max']<=0 or math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[r]['max']+1e-8:raise ValueError('Wanderweite ueberschritten')
        B1,_=bewegen(B,{r:(dx,dy,th)});T=lade_teile(B1,.15);fps={f['ref']:f for f in B1['fps']};bel=Belegung()
        for ref in T:
            q=fps[ref]
            if ref!=r:bel.add(ref,T[ref].geo(q['x'],q['y'],0,q['side']))
        f=fps[r];geo=T[r].geo(f['x'],f['y'],0,f['side']);_,_,inside,_=platine(B,.2)
        if bel.kollision(geo) or not all(inside.contains(g) for g in geo.values()) or sperre(B1,geo,(r,)):raise ValueError('Courtyard/Padabstand/Rand/Keepout verletzt')
        return B1

    def via_move(self,owner,index,target):
        ""
        G=self.G;r=copy.deepcopy(G['U'][owner]);v=r['vias'][index];q=[v['x'],v['y']]
        for t in r['tracks']:
            ps=t['pts'];ws=t['w']
            if len(ps)<2:continue
            if math.dist(ps[0],q)<1e-6 and math.dist(q,ps[1])>.55:
                d=math.dist(q,ps[1]);a=[round(q[k]+.55/d*(ps[1][k]-q[k]),5) for k in (0,1)]
                t['pts']=[ps[0],a]+ps[1:];t['w']=[ws[0]]+ws
            elif math.dist(ps[-1],q)<1e-6 and math.dist(q,ps[-2])>.55:
                d=math.dist(q,ps[-2]);a=[round(q[k]+.55/d*(ps[-2][k]-q[k]),5) for k in (0,1)]
                t['pts']=ps[:-1]+[a,ps[-1]];t['w']=ws+[ws[-1]]
        return self.rest.via_zug(G['D'],r,index,target,G['VIA_D'],G['VIP'],owner.startswith('GND#') and int(owner[4:]) in G['EPM'],G['cls']('/__signal__')['wmin'])

    def candidates(self,c,digest):
        G=self.G; limit=max(1,self.cap//4); count=0


        vv=[]
        for u,r in sorted(G['U'].items()):
            if not self.movable(u,True): continue
            for i,v in enumerate(r['vias']):
                d=min(math.dist((v['x'],v['y']),p) for p in c['punkte'])
                if d<=self.radius: vv.append((u not in c['einheiten'],d,u,i))
        for _,_,u,i in sorted(vv):
            v=G['U'][u]['vias'][i]; targets=[]
            for step in (.1,.2,.3,.4,.5):
                for dx,dy in ((0,step),(0,-step),(step,0),(-step,0),(step,step),(-step,step),(step,-step),(-step,-step)):
                    if math.hypot(dx,dy)<=.500001:
                        x,y=v['x']+dx,v['y']+dy
                        targets.append((round(x,5),round(y,5)))
                        x=G['X0']+round((x-G['X0'])/G['RES'])*G['RES']; y=G['Y0']+round((y-G['Y0'])/G['RES'])*G['RES']
                        if .099999<=math.dist((v['x'],v['y']),(x,y))<=.500001: targets.append((round(x,5),round(y,5)))
            targets=sorted(set(targets),key=lambda p:(max(0., G['VIA_D']/2 + G['DRC_CLR'] - min(math.dist(p,q) for q in c['punkte'])),math.dist(p,(v['x'],v['y'])),p))
            for target in targets[:limit]:
                yield dict(art='via',einheit=u,via=i,ziel=list(target),zustand=digest)
                count+=1
                if count>=limit: break
            if count>=limit: break
        for i,p in enumerate(self.part_moves(c)):
            if i>=limit: break
            yield dict(art='teil',ref=p['ref'],delta=list(p['moves'][p['ref']]),zustand=digest)
        count=0
        for u in c['einheiten']:
            if not self.movable(u): continue
            for i,t in enumerate(G['U'][u]['tracks']):
                if any(t.get(k) for k in ('steg','strom_steg','kelvin','abg','rv')): continue
                for j,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                    from shapely.geometry import LineString
                    if not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['punkte']): continue
                    for layer in G['LAYERS']:
                        if layer==t['layer']: continue
                        yield dict(art='lage',einheit=u,bahn=i,segment=j,lage=layer,zustand=digest)
                        count+=1
                        if count>=limit: break
                    if count>=limit: break
                if count>=limit: break
            if count>=limit: break
        neighbours=sorted((u for u in G['U'] if self.movable(u) and G['_nah_an'](G['U'][u],c['punkte'],self.radius)),key=lambda u:(u in c['einheiten'],u))
        for u in neighbours[:limit]: yield dict(art='umlegen',einheit=u,zustand=digest)

    @staticmethod
    def pair_key(p): return (p[0],*sorted(p[2:4]),round(p[4],2),round(p[5],2))

    def pair_gate(self,p0,p1,c):
        from collections import Counter
        b=c['bbox']
        inside=lambda p:b[0]<=p[4]<=b[2] and b[1]<=p[5]<=b[3]
        before=[p for p in p0 if inside(p)]; after=[p for p in p1 if inside(p)]
        if len(after)>=len(before): return 'Paarzahl im Cluster sinkt nicht'
        old=Counter(self.pair_key(p) for p in p0 if not inside(p))
        new=Counter(self.pair_key(p) for p in p1 if not inside(p))
        if new-old: return 'neuer Konflikt ausserhalb des Clusters'

        distances={};after_distances={}
        for p in p0:
            if not inside(p):distances.setdefault(self.pair_key(p),[]).append(p[1])
        for p in p1:
            if not inside(p):after_distances.setdefault(self.pair_key(p),[]).append(p[1])
        for key,ds in after_distances.items():
            if any(n<o-1e-6 for o,n in zip(sorted(distances[key]),sorted(ds))):return 'Konflikt ausserhalb wird schwerer'
        return None

    def evaluate_gate(self,B0,R0,E0,R1,p0,p1,c):
        G=self.G; K=G['KL']; B1=G['D']
        reason=self.pair_gate(p0,p1,c)
        if reason: return reason,None
        if not set(R1['failed'])<=set(R0['failed']): return 'neue offene Einheit',None
        for check in (self.rest.harte_vias(B1,R0,R1),self.rest.harte_segmente(B1,R0,R1,diagnose=True),self.rest.teil_freiraum(B0,B1,R1)):
            if not check['ok']: return check['tor'],None
        from collections import Counter
        old_vias=Counter((v['net'],v['x'],v['y']) for v in R0['vias'])
        for v in R1['vias']:
            key=(v['net'],v['x'],v['y'])
            if old_vias[key]:old_vias[key]-=1;continue
            ix=int((v['x']-G['X0'])/G['RES']);iy=int((v['y']-G['Y0'])/G['RES'])
            if not (0<=ix<G['NX'] and 0<=iy<G['NY']) or not G['via_hart'](v['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
                return 'Via verletzt statische Rasterkarte (inklusive VIP)',None
        E1=self.bewerte(B1,R1,K,v3=True,minimax=False)
        if E1['offen']>E0['offen'] or not set(E1['unverbunden'])<=set(E0['unverbunden']): return 'Verbindungen schlechter',E1
        from analyse import regelkennzahlen as RK
        from nachbearbeitung.zugregeln import regelwerte, TOPO
        rg=RK.annahme(E0['regeln'],E1['regeln'])
        if not rg['nicht_schlechter']: return rg['grund'],E1
        if not math.isfinite(E1['note_v3']) or E1['note_v3']>E0['note_v3']+1e-6: return 'Note v3 schlechter',E1
        a=regelwerte(B0,R0,K);b=regelwerte(B1,R1,K)
        if any(b['topologie'][k]>a['topologie'][k]+1e-6 for k in TOPO) or b['kopplung']>a['kopplung']+1e-6: return 'Topologie/Kopplung schlechter',E1
        protected=set(K['gnd'])|{n for n,k in K['netze'].items() if k.get('art') in ('strom','versorgung','sw')}
        a=Counter(v['net'] for v in R0['vias'] if v['net'] in protected);b=Counter(v['net'] for v in R1['vias'])
        if a-b: return 'geschuetzte Via-Zahl sinkt',E1
        a=self.rest.massepad_zahlen(B0,R0,K);b=self.rest.massepad_zahlen(B1,R1,K)
        if any(b.get(p,0)<n for p,n in a.items()): return 'R16 Massepad-Via fehlt',E1


        from analyse.bewertung import strompfade,strom_verlust_gleich
        s0=strompfade(B0,R0,K,E0,dc=None);s1=strompfade(B1,R1,K,E1,dc=None)
        p0w,p1w=strom_verlust_gleich(E0,E1,B0,R0,B1,R1,K)
        if (s1.get('umweg_mm') or 0)-(s0.get('umweg_mm') or 0)>1. and p1w>p0w*1.02+1e-9: return 'Strompfade schlechter',E1
        return None,E1

    def static_preflight(self,R0,R1):
        ""
        v=self.rest.harte_vias(self.G['D'],R0,R1)
        if not v['ok'] and v['tor']!='Via-Bohrung-Bohrung':return v['tor']
        s=self.rest.harte_segmente(self.G['D'],R0,R1,diagnose=True)
        return None if s['ok'] else s['tor']

    def prefilter(self,m,c):
        ""
        G=self.G
        if m['art']=='teil':
            projected=self.part_units(m['ref'])|set(self.neighbours(c))
            limit=min(int(G['args'].get('loop_gruppe','24')),int(G['args'].get('loop_neu_max','4' if self.large else '24')))
            return 'Teilgruppe schon vor Placement-Pruefung zu gross' if len(projected)>limit else None
        if m['art'] != 'via': return None
        u=m['einheit']
        if not self.movable(u,True): return 'festes/fehlendes Via'
        if m['via'] >= len(G['U'][u]['vias']): return 'fehlendes Via'
        x,y=m['ziel'];v=G['U'][u]['vias'][m['via']]
        if not .099999<=math.hypot(x-v['x'],y-v['y'])<=.500001: return 'Viaweg ausserhalb Vertrag'
        ix=int((x-G['X0'])/G['RES']);iy=int((y-G['Y0'])/G['RES'])
        if not (0<=ix<G['NX'] and 0<=iy<G['NY']): return 'Viaziel ausserhalb Raster'

        if not G['via_hart'](G['U'][u]['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
            return 'Viaziel statisch gesperrt'
        if self.via_move(u,m['via'],m['ziel']) is None: return 'Via-Anbindung/Gruppenschutz'
        return None

    def rebuild(self,B):
        if self.G.get('_AL') is not None:
            changed=[i for i,(a,b) in enumerate(zip(self.board(self.G['D'])['pads'],B['pads'])) if a!=b]
            if len(changed)>2:return self.rebuild_full(B)
        if self.G['args'].get('loop_karten','lokal') != 'voll':
            return self.rebuild_local(B)
        return self.rebuild_full(B)

    def rebuild_local(self,B):
        G=self.G;original=G.copy()
        changed,window=G['loop_pad_patch'](B)


        changed=set(changed);idx={li:0 for li in range(G['NL'])}
        for key in ('_PADG','_PADXY','_PADB'):G[key]={li:list(rows) for li,rows in G[key].items()}
        for k,p in enumerate(G['D']['pads']):
            for L in p['layers']:
                if L not in G['LI']:continue
                li=G['LI'][L];j=idx[li];idx[li]+=1
                if k not in changed:continue
                pg=G['shapely'].union_all([G['Polygon'](q) for q in p['polys'][L]])
                pn=p['net'] if p['net'] and not p['net'].startswith('unconnected') else '#%d'%id(p)
                G['_PADG'][li][j]=(pn,pg);G['_PADXY'][li][j]=(p['x'],p['y'])
                if G['DRCL']:pg=G['pad_huelle'](p,L,G['DRCP'],True)
                G['_PADB'][li][j]=(pn,pg,pg.buffer(G['_DMAX']))
        G['_PADT']={li:G['shapely'].STRtree([g for _,g in rows]) for li,rows in G['_PADG'].items()}

        rn=G['VIA_KEEP_R']+max(G['DRCP'],.03)+.03
        VP=G['VP'].copy()
        for cell,(x,y,near) in VP.items():
            if not any(p['bbox'][0]-rn<=x<=p['bbox'][2]+rn and p['bbox'][1]-rn<=y<=p['bbox'][3]+rn
                       for k in changed for p in (original['D']['pads'][k],G['D']['pads'][k])):continue
            pt=Point(x,y);near=[]
            for li in range(G['NL']):
                for j in G['_PADT'][li].query(pt.buffer(rn)):
                    n,g=G['_PADG'][li][int(j)];d=float(pt.distance(g))
                    if d<rn:near.append((n,round(d,4),li in (0,G['NL']-1)))
            VP[cell]=(x,y,tuple(sorted(near)))
        G['VP']=VP
        for key in ('_HM','_ERR','HALS_CACHE','_VAPZ','_NAHE_C','_ABST_C'):
            if key in G and isinstance(G[key],dict):G[key]={}
        G['_pad_room_cached'].cache_clear()
        G['KETTEN']={};G['KF_NACH']={};G['KF_VOR']={}
        if G['KETTE']:G['_ketten_bauen']()
        G['_KIDX']=[None];G['ASTAR_COST']=[None]
        self.last_window=list(window)
        return original

    def rebuild_full(self,B):



        import tempfile,contextlib,io
        G=self.G; original=G.copy(); argv=list(sys.argv)
        code=Path(G['__file__']).read_text().split('# Dichte Etiketten sind ein passiver Beobachter;',1)[0]


        code=code.replace('\nbootstrap(heavy=True)\n','\n# bestehender Prozessschutz bleibt erhalten\n',1)
        with tempfile.TemporaryDirectory(prefix='routing_loop_') as td:
            payload=self.board(B)


            payload['pads']=payload['pads'][:original['_LOOP_SOURCE_PAD_COUNT']]
            path=Path(td)/'board.json';path.write_text(json.dumps(payload))
            opt={k:v for k,v in G['args'].items() if k not in ('start','neu','neu_netze','hist_start','hist_context','agent_loop','weiter','agent_zuege')}
            try:
                sys.argv=[argv[0],str(path),original['OUT']]+[k+'='+str(v) for k,v in opt.items()]
                with contextlib.redirect_stdout(io.StringIO()): exec(compile(code,G['__file__'],'exec'),G)
                for u,r in original['U'].items(): G['add_unit'](u,r['net'],copy.deepcopy(r['tracks']),copy.deepcopy(r['vias']))

                for key in ('args','t_start','PF','VIA_ALT','VG_WEICH','VIA_ANKER_AN','QUELLFREI','HART_FREMD','_REST_BOARD_SOURCE','_REST_START_SOURCE'):
                    if key in original: G[key]=copy.deepcopy(original[key])
                G['hist'][:]=original['hist']; G['fail']=set(original['fail']); G['_KIDX'][0]=None
                for key in original:
                    if key.startswith('_AGENT_'):G[key]=original[key]
                if G.get('_AL') is not None:G['_AL'].install_router()
            finally: sys.argv=argv
        return original
    def apply(self,m,c):
        G=self.G; art=m['art']; owner=m.get('einheit'); touched=[]
        if art in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import anwenden
            return anwenden(self,m,c)
        if art=='teil':
            marker=self.rest.placement_marker(self.initial_board,self.part_board(m,c))
            self.rebuild(marker['rest_board']); G=self.G
            ref=m['ref']; nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}
            touched=list(self.part_units(ref))


            for u,r in list(G['U'].items()):
                if not u.startswith('FIX#') or r['net'] not in nets: continue
                if G['_nah_an'](r,c['punkte'],self.radius): raise ValueError('anonymer Flaechenstich braucht gemeinsamen Zug')
        elif art=='via':
            if not self.movable(owner,True): raise ValueError('festes/fehlendes Via')
            r=G['U'][owner];v=r['vias'][m['via']]
            if not .099999<=math.dist((v['x'],v['y']),m['ziel'])<=.500001: raise ValueError('Viaweg ausserhalb 0,1 bis 0,5 mm')
            if not any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['punkte']): raise ValueError('Via nicht lokal')
            r1=self.via_move(owner,m['via'],m['ziel'])
            if r1 is None: raise ValueError('Via-Anbindung/Gruppenschutz')
            G['del_unit'](owner);G['add_unit'](owner,r1['net'],r1['tracks'],r1['vias'])
        elif art=='lage':
            if not self.movable(owner) or m['lage'] not in G['LAYERS']: raise ValueError('gesperrte Einheit/Lage')
            r=copy.deepcopy(G['U'][owner]);i=m['bahn'];j=m['segment'];t=r['tracks'][i]
            if any(t.get(k) for k in ('steg','strom_steg','kelvin','abg','rv')): raise ValueError('geschuetzte Bahn')
            a,b=t['pts'][j:j+2];w=t['w'][j]
            from shapely.geometry import LineString
            if t['layer']==m['lage'] or not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['punkte']): raise ValueError('Lagenwechsel nicht lokal')
            if math.dist(a,b)<G['DRC_HOLE']: raise ValueError('Via-Paar zu eng')
            replacement=[]
            if j: replacement.append(dict(t,pts=t['pts'][:j+1],w=t['w'][:j]))
            replacement.append(dict(t,layer=m['lage'],pts=[a,b],w=[w]))
            if j+1<len(t['w']): replacement.append(dict(t,pts=t['pts'][j+1:],w=t['w'][j+1:]))
            r['tracks'][i:i+1]=replacement
            for x,y in (a,b):
                if not any(math.dist((v['x'],v['y']),(x,y))<1e-6 for v in r['vias']):r['vias'].append(dict(x=x,y=y))
            G['del_unit'](owner);G['add_unit'](owner,r['net'],r['tracks'],r['vias'])
        else:
            if not self.movable(owner) or not G['_nah_an'](G['U'][owner],c['punkte'],self.radius): raise ValueError('Blockierer nicht beweglich/lokal')
            touched=[owner]

        touched+=[u for u in self.neighbours(c) if u!=owner]
        touched=sorted(set(touched),key=lambda u:(u!=m.get('einheit'),G['grp'](u),u))
        if len(touched)>int(G['args'].get('loop_gruppe','24')): raise ValueError('lokale Gruppe zu gross')
        return touched

    def trial(self,m,c,R0,p0,E0,deadline):
        start=time.process_time()
        G=self.G; original=G.copy(); B0=G['D']; order=list(G['U'])


        saved=G['U'].copy()
        hs=G['hist'].copy();vs=copy.deepcopy(G['VIA_ALT']);fails=set(G['fail'])
        flags={k:copy.deepcopy(G[k]) for k in ('PF','VG_WEICH','HART_FREMD','QUELLFREI','VIA_ANKER_AN')}
        semantic={k:copy.deepcopy(G[k]) for k in ('_KZ_AUS','_ERR_NETZE','UNERREICHBAR','STOSS_STAT','KETTE_STAT','RV_STAT','TOPO_STAT','STROM_AUDIT','_AGENT_VORGABEN','_AGENT_PRIORITAET') if k in G}
        ok=False;E1=None;R1=None;p1=None;reason=None;phases={'sichern':time.process_time()-start}
        old_deadline=G.get('_LOOP_DEADLINE')
        stamp=[time.process_time()]
        def phase(name):
            now=time.process_time();phases[name]=now-stamp[0];stamp[0]=now
        try:
            touched=self.apply(m,c)
            phase('anwenden')
            if G['args'].get('loop_suche','begrenzt')=='begrenzt':
                G['_LOOP_DEADLINE']=deadline-min(.5,max(0.,deadline-start)*.2)
            G['VG_WEICH'][0]=False;G['HART_FREMD'][0]=True
            R1,p1=self.out()
            phase('konflikte')
            if m['art'] in ('via','lage','fenster_lage','kugel_via'):
                blocked=self.static_preflight(R0,R1)
                if blocked:raise ValueError('statische Vorpruefung: '+blocked)
            phase('statisch')
            pair_reason=self.pair_gate(p0,p1,c)
            if m['art'] in ('via','lage','fenster_lage','kugel_via') and pair_reason and 'ausserhalb' in pair_reason:
                raise ValueError(pair_reason)

            if m['art'] in ('teil','prioritaet') or pair_reason is not None:
                if len(touched)>int(G['args'].get('loop_neu_max','4' if self.large else '24')):
                    raise ValueError('zu viele Einheiten fuer billigen Neuverlegungszug')
                for u in touched:G['del_unit'](u)
                for u in touched:
                    if time.process_time()>=deadline or G['zeit_knapp'](): raise TimeoutError('Loop-CPU-Budget')
                    if G['route_unit'](u):G['fail'].discard(u)
                    else:G['fail'].add(u)
                R1,p1=self.out()
            phase('neuverlegen')
            if time.process_time()>=deadline or G['zeit_knapp'](): raise TimeoutError('Loop-CPU-Budget')
            reason,E1=self.evaluate_gate(B0,R0,E0,R1,p0,p1,c)
            phase('tore')
            ok=reason is None
            if time.process_time()>=deadline:ok=False;reason='Loop-CPU-Budget nach Pruefung'
        except Exception as e:
            reason=type(e).__name__+': '+str(e)[:220]
        finally:
            phase('arbeit_bis_abbruch_oder_abschluss')
            G['_LOOP_DEADLINE']=old_deadline
            if not ok:
                for u in list(G['U']):
                    if G['U'][u] is not saved.get(u):G['del_unit'](u)
                for u in order:
                    if u not in G['U']:
                        r=saved[u];G['add_unit'](u,r['net'],r['tracks'],r['vias'])
                if G['D'] is not B0:


                    dynamic={k:G[k] for k in ('U','use','drill_use','SN','SN_OFFEN','UOWN','NETU','DDN','_NVER','_GEO')}
                    G.clear();G.update(original);G.update(dynamic)
                G['fail'].clear();G['fail'].update(fails)
                G['VIA_ALT'].clear();G['VIA_ALT'].update(vs)
                for k,v in semantic.items():G[k]=v
            G['hist'][:]=hs
            for k,v in flags.items():G[k][:]=v
            G['_KIDX'][0]=None

            current=G['U'].copy();G['U'].clear()
            for u in order:
                if u in current:G['U'][u]=current.pop(u)
            for u in sorted(current):G['U'][u]=current[u]
            G['exact_conflicts']()
            G['_LOOP_DEADLINE']=old_deadline
            phase('ruecknahme_oder_abschluss')
        self.trials+=1
        if not ok:self.rejected[m['art']]=self.rejected.get(m['art'],0)+1
        self.emit(dict(ereignis='zug',runde=self.rounds,engstelle=c,zug=m,angenommen=ok,
                       grund=reason or 'Cluster verbessert, volle Tore gehalten',cpu_s=round(time.process_time()-start,6),
                       cpu_phasen_s=phases, zug_budget_cpu_s=max(0.,deadline-start),
                       paare_vorher=p0,paare_nachher=p1,
                       vorher={k:E0.get(k) for k in ('gueltig','note_v3','konflikte','offen','regeln')},
                       nachher=None if E1 is None else {k:E1.get(k) for k in ('gueltig','note_v3','konflikte','offen','regeln')}))
        if ok:
            self.accepted[m['art']]=self.accepted.get(m['art'],0)+1
            self.accepted_moves.append(copy.deepcopy(m))
        return ok

    def export(self,c,digest,quick=False):
        d=self.exchange/('%04d_%s'%(self.rounds,digest[:12]));d.mkdir(parents=True,exist_ok=True)
        G=self.G;b=c['bbox'];margin=.5;b=[b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin]
        scale=min(160,1000/max(b[2]-b[0],b[3]-b[1]));size=(max(1,int((b[2]-b[0])*scale)),max(1,int((b[3]-b[1])*scale)))
        im=Image.new('RGB',size,'white');draw=ImageDraw.Draw(im)
        def p(q):return ((q[0]-b[0])*scale,(q[1]-b[1])*scale)
        for u,r in sorted(G['U'].items()):
            for t in r['tracks']:draw.line([p(q) for q in t['pts']],fill=('#315e8e' if u in c['einheiten'] else '#b3bdc7'),width=max(1,int(min(t['w'],default=.1)*scale)))
            for v in r['vias']:
                x,y=p((v['x'],v['y']));rr=G['VIA_D']/2*scale;draw.ellipse((x-rr,y-rr,x+rr,y+rr),outline='#245c40',width=2)
        for i in c['pads']:
            for qs in G['D']['pads'][i]['polys'].values():
                for q in qs:draw.polygon([p(v) for v in q],outline='#9b7300')
        for q in c['punkte']:
            x,y=p(q);draw.ellipse((x-6,y-6,x+6,y+6),outline='red',width=3)
        im.save(d/'konflikte.png')


        places={}; previous=G['HART_FREMD'][0];G['HART_FREMD'][0]=True
        try:
            ix0=max(0,int((b[0]-G['X0'])/G['RES']));ix1=min(G['NX'],int((b[2]-G['X0'])/G['RES']))
            iy0=max(0,int((b[1]-G['Y0'])/G['RES']));iy1=min(G['NY'],int((b[3]-G['Y0'])/G['RES']))
            step=max(1,int(.2/G['RES']));cells=np.asarray([iy*G['NX']+ix for iy in range(iy0,iy1,step) for ix in range(ix0,ix1,step)],dtype=np.int64)
            for net in (() if quick else c['netze']):

                allowed=G['hard_maps'](net)[1].ravel()
                probe=[k for k in cells[:256] if allowed[k]]
                points=[(G['X0']+(int(k)%G['NX']+.5)*G['RES'],G['Y0']+(int(k)//G['NX']+.5)*G['RES']) for k in probe]
                foreign=[geo for u,r in G['U'].items() for li,geo,_,ne in G['_unit_geo'](u,r) if ne!=net]
                holes=[Point(v['x'],v['y']) for r in G['U'].values() for v in r['vias']]
                copper=G['shapely'].union_all(foreign);drills=G['shapely'].union_all(holes)
                places[net]=[[round(x,5),round(y,5)] for x,y in points if (copper.is_empty or Point(x,y).distance(copper)>=G['VIA_D']/2+G['DRC_CLR']) and (drills.is_empty or Point(x,y).distance(drills)>=G['DRC_HOLE'])]
        finally:G['HART_FREMD'][0]=previous
        clean=self.board(G['D'])
        state=dict(version=self.VERSION,zustand=digest,engstelle=c,lagen=G['LAYERS'],
                   raster=dict(res=G['RES'],ursprung=[G['X0'],G['Y0']]),freie_via_rasterpunkte=places,
                   freie_via_rasterpunkte_geprueft=not quick,
                   bauteile=[f for f in G['D']['fps'] if f['ref'] in c['bauteile']],pads=[clean['pads'][i] for i in c['pads']],
                   bisherige_zuege=copy.deepcopy(self.accepted_moves),
                   einheiten={u:{k:G['U'][u][k] for k in ('net','tracks','vias')} for u in sorted(set(c['einheiten'])|set(c.get('nachbarn',[]))|{v['einheit'] for v in c.get('vias',[])})},
                   zugformat='Liste: art via/teil/lage/umlegen; zustand bindet exakt diesen Snapshot')
        (d/'zustand.json').write_text(json.dumps(state,sort_keys=True,indent=1))
        try:
            with (d/'zuege.json').open('x') as f:
                f.write(json.dumps(self.accepted_moves,sort_keys=True,indent=1)+'\n')
        except FileExistsError:
            pass
        self.emit(dict(ereignis='ungeloest',runde=self.rounds,zustand=digest,ordner=str(d)))

    def trigger(self,pairs):
        count=len(pairs);self.stale=self.stale+1 if count==self.previous else 1;self.previous=count
        self.counts=(self.counts+[count])[-self.k:]
        cells={(p[0],round(p[4]/self.G['RES']),round(p[5]/self.G['RES'])) for p in pairs}
        self.cells={k:self.cells.get(k,0)+1 for k in cells}


        if self.large and (count>self.threshold or len(self.counts)<self.k or min(self.counts)<self.counts[0]):return False
        return bool(count and (self.stale>=self.k or max(self.cells.values(),default=0)>=self.n))

    def tick(self,round_number,pairs=None):
        if not self.explicit_budget:self.limit=self.budget(self.G,self.fraction)
        if self.disabled or self.used>=self.limit or self.trials>=self.cap or self.G['zeit_knapp']():return False
        t=time.process_time();G=self.G;self.rounds=round_number;phases={};triggered=False
        def timed(name,start):phases[name]=phases.get(name,0.)+time.process_time()-start
        try:
            start=time.process_time()
            if pairs is None:
                G['exact_conflicts']();pairs=list(G['PAIRS'])
            p0=pairs
            if not self.trigger(p0):return False
            triggered=True;timed('beobachten',start)
            deadline=t+max(0,self.limit-self.used)
            clusters=[];skipped=None;start=time.process_time()
            for c in self.clusters(p0):
                neighbours=[u for u,r in G['U'].items() if self.movable(u) and G['_nah_an'](r,c['punkte'],self.radius)]
                c['nachbarn']=sorted(neighbours)
                if len(neighbours)>int(G['args'].get('loop_gruppe','24'))+1:
                    self.emit(dict(ereignis='cluster_ausgelassen',runde=round_number,grund='lokale Gruppe schon vor Zug zu gross',gruppe_n=len(neighbours),engstelle=c));skipped=skipped or c;continue
                clusters.append(c)
                if len(clusters)>=3:break
            timed('cluster',start)
            if not clusters:
                if skipped:self.export(skipped,self.digest(),quick=True)
                return False
            start=time.process_time();R0,p0=self.out();timed('ausgabe',start)
            start=time.process_time();E0=self.bewerte(G['D'],R0,G['KL'],v3=True,minimax=False);timed('ausgangsbewertung',start)
            if not math.isfinite(E0.get('note_v3',math.inf)):
                self.disabled='unvollstaendige Ausgangsbewertung';self.emit(dict(ereignis='gesperrt',grund=self.disabled));return False
            start=time.process_time();digest=self.digest();timed('zustand_hash',start)
            for c in clusters:
                candidates=[m for m in self.external if m['zustand']==digest] if self.input_path else self.candidates(c,digest)
                iterator=iter(candidates)
                while time.process_time()<deadline and self.trials<self.cap:


                    start=time.process_time()
                    try:m=next(iterator)
                    except StopIteration:
                        timed('vorschlagsbau',start);break
                    timed('vorschlagsbau',start)
                    if time.process_time()>=deadline:break
                    if self.input_path and not self.external_local(m,c):continue
                    key=json.dumps(m,sort_keys=True)
                    if key in self.seen:continue
                    self.seen.add(key)
                    if m['zustand']!=digest:
                        self.emit(dict(ereignis='abgelehnt',grund='veralteter Zustand',zug=m));continue
                    start=time.process_time();blocked=self.prefilter(m,c);timed('vorfilter',start)
                    if blocked:
                        self.prefiltered[m['art']]=self.prefiltered.get(m['art'],0)+1
                        self.emit(dict(ereignis='vorgefiltert',zug=m,grund=blocked));continue
                    start=time.process_time();ok=self.trial(m,c,R0,p0,E0,min(deadline,time.process_time()+self.move_cpu));timed('zuege',start)
                    if ok:
                        self.stale=0;self.cells.clear();return True
                start=time.process_time();self.export(c,digest,quick=time.process_time()>=deadline or G['zeit_knapp']());timed('schnittstelle',start)
            for m in self.external:
                if json.dumps(m,sort_keys=True) not in self.seen:
                    self.emit(dict(ereignis='wartend',grund='Zustand noch nicht erreicht oder kein lokales Ziel in geprueften Clustern/Budget',zug=m))
            return False
        finally:
            elapsed=time.process_time()-t;self.used+=elapsed
            self.emit(dict(ereignis='loop_runde',runde=round_number,ausgeloest=triggered,cpu_s=elapsed,cpu_phasen_s=phases,budget_cpu_s=self.limit))


_RL = _RoutingLoop(globals()) if args.get('loop') == '1' or args.get('agent_loop') == '1' else None





ENDSPIEL = args.get('endspiel', '1' if KL else '0') == '1'



WMIN_KONF = args.get('wmin_konf', '1' if KL else '0') == '1'
WMIN_BIS = int(args.get('wmin_konf_bis', str(2 * MAXIT // 3)))
ALLE_NAH = float(args.get('alle_nah', '0'))
KONVERGENZ_LOKAL = args.get('konvergenz', '') == 'lokal'
_lokal_best = [len(units), 0]
END_MAX = int(args.get('endspiel_max', '8'))
_es = dict(n=0, vor=None, stufe=0, ab=0, lief=False, cpu=0.0, aus=False)
END_ZEIT = float(args.get('endspiel_zeit', '1.0'))


def lokale_suchmaske(paare, offene, radius):
    ""
    m = np.zeros((NY, NX), dtype=bool)
    def region(x1, y1, x2, y2, rand):
        xa = max(0, min(NX, int(math.floor((x1 - rand - X0) / RES))))
        xb = max(0, min(NX, int(math.ceil((x2 + rand - X0) / RES)) + 1))
        ya = max(0, min(NY, int(math.floor((y1 - rand - Y0) / RES))))
        yb = max(0, min(NY, int(math.ceil((y2 + rand - Y0) / RES)) + 1))
        m[ya:yb, xa:xb] = True
    for (_l, _d, _a, _b, x, y) in paare: region(x, y, x, y, radius)
    for u in sorted(offene):
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        boxes = [D['pads'][k]['bbox'] for k in ks]
        if not boxes: continue

        region(min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes),
               6.1 + radius if u.startswith('GND#') else (FENSTER or 3.0) + radius)
    return m


def _nah_an(r, stellen, R):
    for v in r['vias']:
        if any(math.hypot(v['x'] - x, v['y'] - y) <= R for x, y in stellen): return True
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            for x, y in stellen:
                s = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
                if math.hypot(a[0] + s * dx - x, a[1] + s * dy - y) <= R: return True
    return False


def endspiel(conf, paare, stufe, saat, reparatur=False):
    import copy
    R = (1.0, 2.0, 3.5, 5.0)[stufe % 4]
    stellen = [(x, y) for (_l, _d, _a, _b, x, y) in paare]
    kern = sorted(u for u in conf if u in U and not u.startswith(('FIX#', 'HEFT#')))
    gruppe = list(kern)
    for w, r in U.items():
        if w in kern or w.startswith(('FIX#', 'HEFT#')): continue
        if _nah_an(r, stellen, R): gruppe.append(w)
    if PAAR_NETZE: gruppe = sorted(set(gruppe) | {PAAR_NETZE[w] for w in gruppe if w in PAAR_NETZE and PAAR_NETZE[w] in U and PAAR_NETZE[w] not in FIXED})
    if len(gruppe) == len(kern) or len(gruppe) > int(args.get('endspiel_gruppe', '40')):
        print('   Endspiel: Umkreis %.1f mm, Gruppe %d bei %d Konfliktnetzen (%s)' % (R, len(gruppe), len(kern), 'niemand im Weg' if len(gruppe) == len(kern) else 'zu gross'), flush=True)
        return False
    save = {w: copy.deepcopy(U[w]) for w in gruppe}; hsave = hist.copy(); PFs = PF[0]; weich = VG_WEICH[0]; VG_WEICH[0] = False
    quellfrei_vor = QUELLFREI[0]


    X = gruppen_index(gruppe)
    frei = False


    for versuch in range(int(args.get('endspiel_versuche', '3'))):
        _end_used = _es.get('arbeit', 0) if BUDGET_ARBEIT else _es['cpu']
        _end_total = arbeit_gesamt() if BUDGET_ARBEIT else such_cpu() - beobachter_cpu()
        if _end_used > max(30.0, END_ZEIT * (_end_total - _end_used)) or zeit_knapp():
            if BUDGET_ARBEIT: print('   Endspiel: Arbeitsanteil verbraucht (%d Auftraege)' % _end_used, flush=True)
            else: print('   Endspiel: Zeitanteil verbraucht (%.0f s)' % _es['cpu'], flush=True)
            _es['aus'] = True; break
        rng_ = budget_rng(1000 + 10 * saat + versuch); _tc = time.process_time()
        _end_work = arbeit_gesamt()
        _etik_cpu_vor = beobachter_cpu()
        QUELLFREI[0] = reparatur or quellfrei_vor
        HART_FREMD[0] = reparatur and versuch == int(args.get('endspiel_versuche', '3')) - 1
        for w in gruppe:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if versuch == 0: order = kern + sorted((w for w in gruppe if w not in kern), key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else (grp(w)[0], rng_.rand()))
        else: order = sorted(gruppe, key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else (grp(w)[0] if versuch % 2 else 0, rng_.rand()))
        if PAAR_NETZE:
            positions = {w:i for i,w in enumerate(order)}
            order.sort(key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('paar_rang',0), min(w,PAAR_NETZE[w]) if w in PAAR_NETZE else '', w if w in PAAR_NETZE else positions[w]))
        okk = True; verlauf = []; _ab = False
        for w in paar_reihe(order):
            if zeit_knapp(): _ab = True; break
            if not route_unit(w): okk = False
        for _k in range(int(args.get('endspiel_iter', '14'))):
            if not okk or _ab: break
            cfg = konflikte_gruppe(gruppe, X); verlauf.append(len(cfg))
            if _ET: _ET.beobachten(cfg, it if 'it' in globals() else 1, 'endspiel', geometrie=False)
            if args.get('endspiel_pruef') == '1':
                _v = exact_conflicts(); assert set(_v) == set(cfg), ('Endspiel-Pruefung', sorted(set(_v) ^ set(cfg)))
            if not cfg: frei = True; break
            if len(verlauf) >= 5 and min(verlauf[-4:]) >= verlauf[-5]: break
            for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
            PF[0] *= 1.5
            if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
            for w in paar_reihe(sorted((w for w in order if w in cfg), key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else rng_.rand())):
                if zeit_knapp(): _ab = True; break
                if not paar_einheit(w): del_unit(w)
                if not route_unit(w): okk = False
        print('   Endspiel: Umkreis %.1f mm, Gruppe %d (davon GND-Stiche %d), Anlauf %d, Konflikte je Runde %s%s' % (
            R, len(gruppe), sum(1 for w in gruppe if w.startswith('GND#')), versuch + 1, verlauf, '' if okk else ', unvollstaendig'), flush=True)
        HART_FREMD[0] = False
        QUELLFREI[0] = quellfrei_vor
        _es['cpu'] += time.process_time() - _tc - (beobachter_cpu() - _etik_cpu_vor)
        if BUDGET_ARBEIT: _es['arbeit'] = _es.get('arbeit', 0) + arbeit_gesamt() - _end_work
        if _ab:
            print('   Endspiel: Rechenzeit-Budget verbraucht, Anlauf abgebrochen', flush=True); _es['aus'] = True; break
        if frei: break
    hist[:] = hsave; PF[0] = PFs; VG_WEICH[0] = weich
    if frei: return True
    for w in gruppe:
        if w in U: del_unit(w)
    for w in gruppe: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    return False









ZUG = args.get('zug', '0') == '1'
ZUG_MAX = int(args.get('zug_max', '80'))
ZUG_STELLEN = int(args.get('zug_stellen', '10'))
ZUG_GRUPPE = int(args.get('zug_gruppe', '25'))
ZUG_ANTEIL = float(args.get('zug_anteil', '0.25'))
ZUG_ITER = int(args.get('zug_iter', '10'))



ZUG_STRAT = (dict(R=2.0, reihe='kern', warm=True), dict(R=3.5, reihe='kern', warm=True), dict(R=1.0, reihe='kern'),
             dict(R=2.0, reihe='kern'), dict(R=2.0, reihe='zufall'), dict(R=3.5, reihe='kern'),
             dict(R=2.0, reihe='kern', reparatur=True), dict(R=5.0, reihe='kern'))
ZUG_WARM_PF = float(args.get('zug_warm_pf', '50'))
_zs = dict(n=0, ok=0, cpu=0.0, arbeit=0, ruhig=0, best=None, log=None)

ZUG_POLITIK = json.load(open(args['zug_politik']))['tabelle'] if args.get('zug_politik') else None
def _zug_pads2(x, y):
    return sum(1 for p in D['pads'] if abs(p['x'] - x) <= 2.0 and abs(p['y'] - y) <= 2.0)

def _zug_reihe(stelle):
    if not ZUG_POLITIK: return list(enumerate(ZUG_STRAT))
    k = len(stelle['kern']); p = _zug_pads2(stelle['x'], stelle['y'])
    lage = '%s|k%d|p%d' % (stelle['typ'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)
    tab = ZUG_POLITIK.get(lage) or ZUG_POLITIK.get(stelle['typ']) or {}
    def wert(js):
        s = js[1]; e = tab.get('%s|%s|%d|%d' % (s['R'], s['reihe'], int(bool(s.get('reparatur'))), int(bool(s.get('warm')))))
        return (-(e['wert'] if e else -1.0), js[0])
    return sorted(enumerate(ZUG_STRAT), key=wert)


def _zug_log(row):
    if _zs['log'] is None:
        _zs['log'] = open(args.get('zug_log', OUT.replace('.json', '') + '_zuege.jsonl'), 'w')
    _zs['log'].write(json.dumps(row, sort_keys=True) + '\n'); _zs['log'].flush()


def _zug_stellen(paare, offene):
    ""
    st = []
    for (_l, _d, a, b, x, y) in sorted(paare, key=lambda p: (round(p[4], 3), round(p[5], 3), p[2], p[3])):
        kern = {w for w in (a, b) if w in U and not w.startswith(('FIX#', 'HEFT#'))}
        if not kern: continue
        for s in st:
            if s['typ'] == 'konflikt' and math.hypot(s['x'] - x, s['y'] - y) <= 0.5:
                s['kern'] |= kern; break
        else: st.append(dict(typ='konflikt', x=x, y=y, kern=kern))
    for u in sorted(offene):
        if u in FIXED or u.startswith(('FIX#', 'HEFT#')): continue
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        seen = []
        for k in ks:
            p = D['pads'][k]
            if any(math.hypot(p['x'] - x, p['y'] - y) <= 1.0 for x, y in seen): continue
            seen.append((p['x'], p['y']))
            st.append(dict(typ='offen', x=p['x'], y=p['y'], kern={u}))
            if len(seen) >= 4: break
    return st


def _zug_abstand(r, p):
    x, y = p; d = float('inf')
    for v in r['vias']: d = min(d, math.hypot(v['x'] - x, v['y'] - y))
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            s_ = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
            d = min(d, math.hypot(a[0] + s_ * dx - x, a[1] + s_ * dy - y))
    return d


ZUG_LOKAL = args.get('zug_lokal', '1') == '1'
ZUG_PRUEF = args.get('zug_pruef', '0') == '1'


_ZUG_RAND = _DMAX + max(DRC_HOLE, VIA_DR / 2 + VIA_R + LOCH_CLR, VIA_DR / 2 + LOCH_CLR, VIA_DR / 2 + H2H) + VIA_R + 0.05


def konflikte_lokal(gruppe):
    ""
    g = set(gruppe); b = None
    for w in sorted(g):
        if w not in U: continue
        for _li, k, _kb, _ne in _unit_geo(w, U[w]):
            x0, y0, x1, y1 = k.bounds
            b = [x0, y0, x1, y1] if b is None else [min(b[0], x0), min(b[1], y0), max(b[2], x1), max(b[3], y1)]
    if b is None: return {}
    F = (b[0] - _ZUG_RAND, b[1] - _ZUG_RAND, b[2] + _ZUG_RAND, b[3] + _ZUG_RAND)
    sv = (PAIRS[:], PAIRS_ROH[:], FIX_PAIRS[:])
    def auswahl(conf):
        paare = [p for p in PAIRS if p[2] in g or p[3] in g]
        mit = {u for p in paare for u in p[2:4]}
        return {u: cs for u, cs in conf.items() if u in mit}, sorted(paare)
    res, paare = auswahl(exact_conflicts(fenster=F))
    if ZUG_PRUEF:
        resv, paarev = auswahl(exact_conflicts())
        gleich = (sorted(res) == sorted(resv) and paare == paarev
                  and all(sorted(res[u]) == sorted(resv[u]) for u in res if u in g))
        _zs['pruef'] = _zs.get('pruef', 0) + 1
        if not gleich:
            raise AssertionError('Zug-Fensterpruefung weicht ab: %s / %s' % (sorted(set(res) ^ set(resv)), sorted(set(paare) ^ set(paarev))[:6]))
    PAIRS[:], PAIRS_ROH[:], FIX_PAIRS[:] = sv
    return res


def _zug_konf(gruppe, X):
    return konflikte_lokal(gruppe) if ZUG_LOKAL else konflikte_gruppe(gruppe, X)


def _zug_wert(gruppe, X):
    cfg = _zug_konf(gruppe, X)
    return len(cfg) + 2 * sum(1 for w in gruppe if w in fail), cfg


def zug_versuch(stelle, strat, saat):
    ""
    import copy
    R = strat['R']; xy = [(stelle['x'], stelle['y'])]
    kern = sorted(stelle['kern'])
    nah = [w for w, r in sorted(U.items()) if w not in stelle['kern'] and not w.startswith(('FIX#', 'HEFT#'))
           and w not in FIXED and _nah_an(r, xy, R)]
    gekappt = len(kern) + len(nah) > ZUG_GRUPPE
    if gekappt:
        nah = sorted(nah, key=lambda w: (_zug_abstand(U[w], xy[0]), w))[:max(0, ZUG_GRUPPE - len(kern))]
    gruppe = list(kern) + nah
    if PAAR_NETZE: gruppe = sorted(set(gruppe) | {PAAR_NETZE[w] for w in gruppe if w in PAAR_NETZE and PAAR_NETZE[w] in units and PAAR_NETZE[w] not in FIXED})
    zeile = dict(zug=_zs['n'], typ=stelle['typ'], x=round(stelle['x'], 3), y=round(stelle['y'], 3), R=R, reihe=strat['reihe'], warm=bool(strat.get('warm')),
                 reparatur=bool(strat.get('reparatur')), kern=len(kern), gruppe=len(gruppe),
                 gnd=sum(1 for w in gruppe if w.startswith('GND#')), paar=sum(1 for w in gruppe if w in PAAR_NETZE),
                 arten=sorted({(KL['netze'].get(w) or {}).get('art', 'signal') for w in kern}) if KL else [],
                 pads=sum(1 for p in D['pads'] if abs(p['x'] - stelle['x']) <= R and abs(p['y'] - stelle['y']) <= R),
                 pads2=_zug_pads2(stelle['x'], stelle['y']), politik=bool(ZUG_POLITIK),
                 konflikte_ges=len(PAIRS), offen_ges=len(fail), lagen=NL, gekappt=gekappt)
    if len(gruppe) > ZUG_GRUPPE:
        zeile.update(ergebnis='gruppe_zu_gross'); _zug_log(zeile); return False
    _tc = time.process_time(); _ta = arbeit_gesamt() if BUDGET_ARBEIT else 0
    X = None if ZUG_LOKAL else gruppen_index(gruppe)
    vor, _ = _zug_wert(gruppe, X)
    save = {w: copy.deepcopy(U[w]) for w in gruppe if w in U}; fail_vor = {w for w in gruppe if w in fail}
    hsave = hist.copy(); PFs = PF[0]; weich = VG_WEICH[0]; VG_WEICH[0] = False; qf = QUELLFREI[0]
    QUELLFREI[0] = bool(strat.get('reparatur')) or qf; HART_FREMD[0] = False
    rng_ = budget_rng(5000 + saat)
    andere = [w for w in gruppe if w not in stelle['kern']]
    if strat.get('warm'):
        for w in kern:
            if w in U: del_unit(w)
        PF[0] = min(PFs, ZUG_WARM_PF)
        order = list(kern)
    else:
        for w in gruppe:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if strat['reihe'] == 'kern': order = kern + sorted(andere, key=lambda w: (grp(w)[0], rng_.rand()))
        else: order = sorted(gruppe, key=lambda w: rng_.rand())
    ok = {}; verlauf = []
    for w in paar_reihe(order): ok[w] = route_unit(w)
    for _k in range(ZUG_ITER):
        cfg = _zug_konf(gruppe, X); verlauf.append(len(cfg))
        if not cfg or zeit_knapp(0.95): break
        if len(verlauf) >= 4 and min(verlauf[-3:]) >= verlauf[-4]: break
        if strat.get('warm') and len(verlauf) >= 3 and min(verlauf[-2:]) >= min(verlauf[:-2]): break
        for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
        PF[0] *= 1.5
        if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
        for w in paar_reihe(sorted((w for w in gruppe if w in cfg or not ok.get(w, True)), key=lambda w: rng_.rand())):
            if not paar_einheit(w): del_unit(w)
            ok[w] = route_unit(w)
    for w in gruppe:
        if ok.get(w, w in U): fail.discard(w)
        else: fail.add(w)
    nach, _ = _zug_wert(gruppe, X)
    hist[:] = hsave; PF[0] = PFs; VG_WEICH[0] = weich; QUELLFREI[0] = qf
    angenommen = nach < vor
    if not angenommen:
        for w in gruppe:
            if w in U: del_unit(w)
        for w, r in sorted(save.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
        for w in gruppe:
            if w in fail_vor: fail.add(w)
            else: fail.discard(w)
    cpu = time.process_time() - _tc; _zs['cpu'] += cpu
    if BUDGET_ARBEIT: _zs['arbeit'] += arbeit_gesamt() - _ta
    zeile.update(ergebnis='angenommen' if angenommen else 'verworfen', vorher=vor, nachher=nach, verlauf=verlauf, cpu_s=round(cpu, 3))
    _zug_log(zeile)
    return angenommen


def zug_merken():
    ""
    import copy
    wert = len(exact_conflicts()) + len(fail)
    if _zs.get('stand') is None or wert < _zs['stand'][0]:
        _zs['stand'] = (wert, {w: copy.deepcopy(r) for w, r in U.items() if not w.startswith(('FIX#', 'HEFT#'))}, set(fail))


def zug_bester():
    ""
    global conf
    if _zs.get('stand') is None: return False
    jetzt = len(exact_conflicts()) + len(fail)
    if _zs['stand'][0] >= jetzt: return False
    _, alt, alt_fail = _zs['stand']
    for w in sorted(set(U) | set(alt)):
        if w.startswith(('FIX#', 'HEFT#')): continue
        if w in U: del_unit(w)
    for w, r in sorted(alt.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
    fail.clear(); fail.update(alt_fail)
    conf = exact_conflicts()
    print('   Zugschleife: besten Stand zurueckgeholt (%d statt %d Konflikte+offen)' % (_zs['stand'][0], jetzt), flush=True)
    return True


def zug_budget_aus():
    if _zs['n'] >= ZUG_MAX or zeit_knapp(0.95): return True
    if BUDGET_ARBEIT: return _zs['arbeit'] >= ZUG_ANTEIL * ARBEIT_MAX
    return CPU_MAX > 0 and _zs['cpu'] >= ZUG_ANTEIL * CPU_MAX


def zugschleife(phase, saat):
    ""
    global conf
    conf = exact_conflicts()
    if not (0 < len(conf) + len(fail) <= ZUG_STELLEN): return 0
    gut = 0
    for i, st in enumerate(_zug_stellen(list(PAIRS), set(fail))):
        if not (st['kern'] & (set(conf) | fail)): continue
        for j, strat in _zug_reihe(st):
            if zug_budget_aus(): break
            tabu = (round(st['x'], 1), round(st['y'], 1), j)
            if _zs.setdefault('tabu', {}).get(tabu, 0) >= 2: continue
            _zs['tabu'][tabu] = _zs['tabu'].get(tabu, 0) + 1
            _zs['n'] += 1
            if zug_versuch(st, strat, 100 * saat + 10 * i + j):
                gut += 1; _zs['ok'] += 1; conf = exact_conflicts(); break
    print('   Zugschleife (%s): %d Zuege gesamt, %d uebernommen in diesem Durchgang, jetzt %d Konflikte, %d offen (%.0f CPU-s)' % (
        phase, _zs['n'], gut, len(conf), len(fail), _zs['cpu']), flush=True)
    return gut


if MAXIT < 2 and todo is None:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANKER_AN[0] = True
_AL = None
if args.get('agent_loop') == '1':
    from codex.agent_loop.zustand import AgentLoop
    _AL = AgentLoop(globals(), _AGENT_SAVED)
for it in range(_AGENT_SAVED['runde'] if _AGENT_SAVED else 2, MAXIT + 1):
    VG_WEICH[0] = it <= int(args.get('vg_iter', '10'))

    conf = exact_conflicts(); _es['lief'] = True
    if (_AL.before_round(it, list(PAIRS)) if _AL else (_RL and _RL.tick(it - 1, list(PAIRS)))):
        print('   Routing-Loop: lokalen Zug uebernommen', flush=True)
        conf = exact_conflicts()
    verhandlung_messen(it - 1, conf)
    _etik_conf = _ET.konfliktkopie(conf) if _ET else None
    if VIA_VORRANG and it <= WMIN_BIS:
        conf, _nvr = vorrang_filtern(conf)
        if _nvr: print('   Via-Vorrang (R25): %d Stellen, an denen nur das Signal weicht' % _nvr, flush=True)
    if WMIN_KONF and it <= WMIN_BIS:
        _eng = eng_konflikte()
        for _u, _c in _eng.items(): conf.setdefault(_u, []).extend(_c)
        if _ET: _ET.ergaenzen(_etik_conf, _eng)
        if _eng: print('   zu schmal (wmin_konf): %d Einheiten beteiligt' % len(_eng), flush=True)
    if _ET and not _LERN_AN: _ET.beobachten(_etik_conf, it - 1)
    todo = (set(conf) | fail) - set(FIXED)
    print('Iteration %d: Konflikte %d, unvollstaendig %d, PF %.2f (%.0fs)' % (it, len(conf), len(fail), PF[0], time.time() - t0), flush=True)
    arbeit_marke('runde_%d' % it)
    if it % 4 == 0:
        import collections
        print('   Paare', len(PAIRS), collections.Counter(tuple(sorted((x[2].split('#')[0][:12], x[3].split('#')[0][:12]))) for x in PAIRS).most_common(12), flush=True)
        print('   Beispiele', sorted(PAIRS, key=lambda x: x[1])[:6], flush=True)
    if not todo: break
    if zeit_knapp():
        print('%s zu 80 %% verbraucht: Verhandlung endet mit %d Konflikten' % (budget_meldung(), len(conf)), flush=True); break
    if ZUG and it >= 8:
        _zsum = len(conf) + len(fail)
        if _zs['best'] is None or _zsum < _zs['best']: _zs['best'] = _zsum; _zs['ruhig'] = 0
        else: _zs['ruhig'] += 1
        if _zs['ruhig'] >= 3 and 0 < _zsum <= ZUG_STELLEN and not zug_budget_aus():
            _zs['ruhig'] = 0; _zgut = 0
            for _zr in range(4):
                if not zugschleife('Verhandlung', 10 * it + _zr): break
                _zgut += 1; zug_merken()
                if not conf and not fail: break
            if _zgut:
                _zs['best'] = len(conf) + len(fail); continue
    if ENDSPIEL and not fail and len(conf) <= END_MAX and it >= 6:
        if _es.get('ok_vor') is not None:
            if set(conf) == _es['ok_vor']:
                _es['schein'] = _es.get('schein', 0) + 1; _es['stufe'] += 1; _es['ab'] = it + 3
                print('   Endspiel: Scheinerfolg %d (derselbe Konflikt wieder da)' % _es['schein'], flush=True)
                if _es['schein'] >= 2: _es['aus'] = True; print('   Endspiel: aus fuer diesen Lauf', flush=True)
            _es['ok_vor'] = None
        _es['n'] = _es['n'] + 1 if set(conf) == _es['vor'] else 1; _es['vor'] = set(conf)
        if _es['n'] >= 4 and it >= _es['ab'] and not _es['aus']:
            _ok = endspiel(conf, list(PAIRS), _es['stufe'], it)
            print('   Endspiel (Anlaufrunde %d): %s' % (_es['stufe'] + 1, 'konfliktfrei' if _ok else 'ohne Erfolg, alter Stand'), flush=True)
            if _ok: _es['ok_vor'] = set(conf); continue
            _es['stufe'] += 1; _es['ab'] = it + 3
    else:
        _es['n'] = 0; _es['vor'] = None
    for u, b in conf.items(): hist[np.array(b, dtype=np.int64)] += HINC
    PF[0] *= float(args.get('pfmul', '1.5'))
    if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
    fail_alt = fail; fail = set(); _dran = set()
    alle = args.get('alle', '1') == '1' and len(conf) > int(args.get('ziel_ab', '0'))
    nah = ALLE_NAH
    if KONVERGENZ_LOKAL:
        if len(conf) < _lokal_best[0]: _lokal_best[:] = [len(conf), 0]
        else: _lokal_best[1] += 1
        alle = it % 4 == 0
        nah = ALLE_NAH or min(6.0, 1.0 + _lokal_best[1] / 4)
    rng = budget_rng(it)
    def gkey(u):
        if PAAR_NETZE:


            p = PAAR_NETZE.get(u)
            if p: return (grp(u)[0], -KL['netze'][u].get('paar_rang', 0), min(u, p), u)
            rang = (KL['netze'].get(u) or {}).get('rang', 0) if args.get('rang_alle', '0') == '1' else 0
            if KL:
                rang *= float(args.get('kosten_rang_' + (KL['netze'].get(u) or {}).get('art', 'signal'), '1'))
            return (grp(u)[0], -rang, '', lern_reihenwert(u,rng.rand()))
        if KL and args.get('rang_alle', '0') == '1':
            _nk = KL['netze'].get(u) or {}
            return (grp(u)[0], -_nk.get('rang', 0) * float(args.get('kosten_rang_' + _nk.get('art', 'signal'), '1')), lern_reihenwert(u,rng.rand()))
        return (grp(u)[0], lern_reihenwert(u,rng.rand()))


    if alle and nah > 0:


        if KONVERGENZ_LOKAL:
            _m = lokale_suchmaske(PAIRS, fail_alt, nah)
        else:
            _m = np.zeros((NY, NX), dtype=bool); _rr = int(nah / RES)
            for (_l, _d, _a, _b, _x, _y) in PAIRS:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                _m[max(0, _iy - _rr):_iy + _rr + 1, max(0, _ix - _rr):_ix + _rr + 1] = True
        _mf = _m.ravel()
        reihe = [u for u in units if u in todo or (u in U and u not in FIXED and len(U[u]['cells']) and bool(_mf[U[u]['cells'] % NN].any()))]
        print('   Umkreis %.1f mm: %d von %d Einheiten neu' % (nah, len(reihe), len(units)), flush=True)
    else:
        reihe = units if alle else sorted(todo)
    if PAAR_NETZE:
        reihe = sorted(set(reihe) | {PAAR_NETZE[u] for u in reihe if u in PAAR_NETZE and PAAR_NETZE[u] in units})
    _verhandlungsreihe = paar_reihe(sorted(reihe, key=gkey))
    if _AL:
        from codex.agent_loop.zuege import reihenfolge as _agent_reihenfolge
        _verhandlungsreihe = _agent_reihenfolge(_verhandlungsreihe, _AGENT_PRIORITAET)
    for u in _verhandlungsreihe:
        if zeit_knapp():
            fail |= fail_alt - _dran; break
        if paar_einheit(u):
            _dran.update((u,PAAR_NETZE[u]))
            if route_unit(u): fail.discard(u); fail.discard(PAAR_NETZE[u])
            else: fail.update((u,PAAR_NETZE[u]))
        else:
            del_unit(u); _dran.add(u)
            if not route_unit(u): fail.add(u)
        if u in PAAR_NETZE and u > PAAR_NETZE[u] and u not in fail and PAAR_NETZE[u] not in fail:
            paar_koppeln(nur={u, PAAR_NETZE[u]})
if ENDSPIEL and _es['lief'] and not fail:
    conf = exact_conflicts()
    for _st in range(_es['stufe'], _es['stufe'] + 4):
        if not (0 < len(conf) <= END_MAX) or _es['aus']: break
        _ok = endspiel(conf, list(PAIRS), _st, 100 + _st)
        print('Endspiel nach dem letzten Durchgang (Anlaufrunde %d): %s' % (_st + 1, 'konfliktfrei' if _ok else 'ohne Erfolg'), flush=True)
        conf = exact_conflicts()
    todo = (set(conf) | fail) - set(FIXED)
elif _es['lief']:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
if ENDSPIEL and KL and args.get('endspiel_reparatur', '1') == '1' and not fail and not zeit_knapp():



    conf = exact_conflicts()
    if 0 < len(conf) <= END_MAX:
        _es['aus'] = False; _es_alt_cpu = _es['cpu']; _es['cpu'] = 0.0
        for _st in range(4):
            if zeit_knapp(): break

            _ok = endspiel(conf, list(PAIRS), _st, 24 + _st, reparatur=True)
            print('Endspiel-Reparatur: Umkreis %.1f mm, %s' % ((1.0, 2.0, 3.5, 5.0)[_st], 'konfliktfrei' if _ok else 'alter Stand'), flush=True)
            conf = exact_conflicts()
            if not conf or _es['aus']: break
        _es['cpu'] += _es_alt_cpu
        todo = (set(conf) | fail) - set(FIXED)
if ZUG and _es['lief']:
    zug_bester()
    for _zr in range(4):
        if zug_budget_aus() or not zugschleife('Ende %d' % (_zr + 1), 900 + _zr): break
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANKER_AN[0] = False
VG_WEICH[0] = False

_cf_checkpoint = exact_conflicts(); PAIRS_LAST = list(PAIRS)
arbeit_marke('vor_nachlauf')
verhandlung_messen(it if 'it' in globals() else 1, _cf_checkpoint, 'vor_nachlauf')
if _ET and not _LERN_AN: _ET.beobachten(_cf_checkpoint, it if 'it' in globals() else 1)
json.dump(PAIRS_LAST, open(OUT.replace('.json', '_konflikte_vor_verbreitern.json'), 'w'))
if ((BGA_PLAN and args.get('bga_budget', '0') == '1') or args.get('budget_vor_nachlauf', '0') == '1' or globals().get('_FRUEH_INFO') is not None) and zeit_knapp() and (_cf_checkpoint or fail):

    _ct, _cv = ausgabe_geometrie()
    _cc = {u:len(cs) for u,cs in _cf_checkpoint.items()}
    _sicher = ausgabe_daten(_ct, _cv, _cc,
        abschluss=(_LERN_STOP or 'cpu_budget') if _LERN_AN else 'budget_vor_nachlauf',
        **({'lernstand': json.loads(_LERN_SNAPSHOT)['lernstand'], 'lernbudget':json.loads(_LERN_SNAPSHOT)['lernbudget']} if _LERN_AN else {}))
    json.dump(_sicher, open(OUT,'w'))
    json.dump(PAIRS_LAST,open(OUT.replace('.json','_konflikte.json'),'w'))
    print('Rechenzeit-Budget: ungueltiger Stand gespeichert, Breiten-/Formnachlauf ausgelassen',flush=True)
    print('Ende: Konflikte',_cc,'unvollstaendig',sorted(fail),flush=True)
    print('Bahnen',len(_ct),'Vias',len(_cv),'Zeit %.0fs' % (time.time()-t_start),flush=True)
    if _ET: _ET.schreiben(_sicher['tracks'], _sicher['vias'], fehlend=_sicher['failed'], zonen=_sicher.get('zonen', []) + _sicher.get('zones', []))
    sys.exit(0)
import copy
def ulen(u):
    r = U[u]
    if VG and any(v.get('gr') for v in r['vias']):
        return sum(math.dist(a, c) for t in r['tracks'] if not t.get('steg') for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
    return sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
def grp_key(u): return grp(u)[0]


GEW_ART = None
if GEWICHTET:
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
        from gemeinsam.wichtigkeit import gewichte as _gewichte, tausch_besser as _tausch_besser
        GEW_ART = _gewichte()
    except Exception as _e:
        print('gewichtet=%d: Gewichte aus analyse/bewertung.py nicht ladbar (%r), es bleibt bei der nackten Laenge' % (GEWICHTET, _e), flush=True)
        GEWICHTET = 0
def u_gew(u):
    ""
    if GEW_ART is None or u not in U: return 1.0
    return GEW_ART.get(KL['netze'].get(U[u]['net'], {}).get('art', 'signal'), 1.0)
def u_vias(u):
    r = U[u]
    return n_sv(r)
def ulen_gew(u):
    ""
    nv = u_vias(u)
    return u_gew(u) * (ulen(u) - 0.6 * nv) + 0.6 * nv
def strom_widerstand(u, erreichbar=False):
    ""
    r = U[u]; net = r['net']; C = cls(net)
    k = (KL['netze'].get(net) or {}) if KL else {}
    from gemeinsam.fluss import analyse as fa, zielbreite as fz
    info = fa(r['tracks'], r['vias'], [D['pads'][i] for i in bynet_g.get(net, [])], k.get('fluss') or {})
    anteil = info['anteil'] if info and info['gesamt'] > 0 else None
    cap = None
    if erreichbar and r['tracks']:
        xy = [q for t in r['tracks'] for q in t['pts']]
        box = (min(q[0] for q in xy) - 1, min(q[1] for q in xy) - 1, max(q[0] for q in xy) + 1, max(q[1] for q in xy) + 1)
        cap = karten(net, box, C['w'] * max(f_innen(net, LI[t['layer']]) for t in r['tracks']))[6]
    out = 0.0
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('steg'): continue
        li = LI[t['layer']]; dick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            f = anteil.get((ti, si), 0.0) if anteil is not None else 1.0
            if cap is not None:
                ziel = fz(f, C['wmin'], C['w'], float(args.get('fluss_exp', '0.5'))) * f_innen(net, li)
                w = max(w, math.floor(min(ziel, line_cap_min(cap[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
            out += f * f * 1.72e-5 * math.dist(a, b) / (w * dick)
    out += 0.0024 * sum(1 for v in r['vias'] if not v.get('gr') and not v.get('rv'))
    return out


def routing_mass(u):
    if KL and args.get('strom_kurz') == '1' and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw'):

        return strom_widerstand(u, erreichbar=True) * cls(U[u]['net'])['w'] * 0.035 / 1.72e-5
    return ulen(u)


def routing_mass_gew(u):


    if KL and args.get('strom_kurz') == '1' and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw'):
        return u_gew(u) * routing_mass(u)
    return ulen_gew(u)


def kurz_key(u): return -ulen(u) * (u_gew(u) if GEWICHTET >= 2 else 1.0)


R19_KURZ = args.get('r19_kurz', '0') == '1' and KL is not None
R19_KURZ_STAT = dict(versuche=0, mehr_vias_verworfen=0, zusatz_vias_verworfen=0)
def r19_kurz_ok(alt, neu):
    return n_sv(neu) <= n_sv(alt)

if args.get('kurz') == '1' and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3
    _KIDX[0] = None
    for rnd in range(int(args.get('kurz_runden', '2'))):
        better = 0; gain = 0.0
        for u in sorted([u for u in units if not u.startswith('GND#') and u in U and not paar_einheit(u)], key=kurz_key):
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); L0 = routing_mass(u); _f0 = vg_fehl(u) if VG else 0
            del_unit(u)
            okk = route_unit(u)
            if okk and routing_mass(u) < L0 - 0.05 and (not VG or vg_fehl(u) <= _f0):
                if R19_KURZ and (KL['netze'].get(old['net']) or {}).get('art') in ('signal', 'analog', 'schnell'):
                    R19_KURZ_STAT['versuche'] += 1
                    if not r19_kurz_ok(old, U[u]):
                        R19_KURZ_STAT['mehr_vias_verworfen'] += 1
                        R19_KURZ_STAT['zusatz_vias_verworfen'] += n_sv(U[u]) - n_sv(old)
                        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u); continue
                if einheit_konfliktfrei(u): better += 1; gain += L0 - routing_mass(u); kidx_geaendert(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
        print('Kuerzen Runde %d: %d Netze kuerzer, zusammen %.1f mm (%.0fs)' % (rnd + 1, better, gain, time.time() - t2), flush=True)
        if not better: break
    PF[0] = PFs
    if R19_KURZ: print('R19 Kuerzen:', R19_KURZ_STAT, flush=True)



KURZ_VIA = float(args.get('kurz_via', '8' if KL and 'start' not in args else '0'))
if KURZ_VIA > 0 and KL is not None and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _KIDX[0] = None; VIA_FAK[0] = KURZ_VIA
    _mm = float(args.get('kurz_via_mm', '3.0')); _gesp = 0; _mehr = 0.0
    def _via_kandidat(u):
        if paar_einheit(u) or u.startswith(('GND#', 'FIX#', 'HEFT#')) or u not in U or not U[u]['vias']: return False
        if KL['netze'].get(U[u]['net'], {}).get('art') in ('strom', 'versorgung', 'sw', 'gnd'): return False
        return not any(v.get('n') or v.get('gr') for v in U[u]['vias'])



    def _via_gew(u): return SCHNELL_VIA if SCHNELL_VIA != 1 and ist_schnell(U[u]['net']) else 1.0
    def _via_key(u):
        if GEWICHTET >= 2: return (-n_sv(U[u]) * _via_gew(u) * u_gew(u), u)
        return (-n_sv(U[u]) * _via_gew(u), u)
    for rnd in range(int(args.get('kurz_via_runden', '2'))):
        _n = 0
        for u in sorted([u for u in units if _via_kandidat(u)], key=_via_key):
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); _v0 = n_sv(old); L0 = ulen(u) - 0.6 * _v0; _vg_ = _via_gew(u)
            del_unit(u)
            okk = route_unit(u)
            _v1 = n_sv(U[u]); L1 = ulen(u) - 0.6 * _v1
            _erl = _mm * (2.0 if KL['netze'].get(old['net'], {}).get('art', 'signal') == 'signal' else 1.0) * _vg_
            if okk and _v1 < _v0 and L1 <= L0 + _erl * (_v0 - _v1) and einheit_konfliktfrei(u):
                _n += 1; _gesp += _v0 - _v1; _mehr += L1 - L0; kidx_geaendert(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
        if not _n: break
    VIA_FAK[0] = 1.0; PF[0] = PFs
    print('Via-Runde (Via-Preis x%g): %d Vias gespart, Bahnen dafuer %.1f mm laenger (%.0fs)' % (KURZ_VIA, _gesp, _mehr, time.time() - t2), flush=True)





if (VAP or EPM) and args.get('via_am_pad_raeumen', '0' if 'start' in args else '1') == '1' and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _nr = 0; _nv = 0; _mehr = 0.0; _nep = 0; _gew = 0.0
    _umw = float(args.get('via_am_pad_umweg', '2.0')); _gmin = float(args.get('via_am_pad_gewinn', '0.6'))
    def _guete(u):
        r = U[u]; _p = D['pads'][int(u[4:])]; _v = r['vias'][0]; _c, _w = _vap_zellen(int(u[4:]))
        _j = np.flatnonzero(_c == int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES))
        return (sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:]))
                + 2.0 * math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']) + VAP_K * (float(_w[_j[0]]) if len(_j) else 1.0))
    for u in [u for u in units if u.startswith('GND#') and u in U and U[u]['vias']]:
        if zeit_knapp(0.9): break
        _k = int(u[4:]); _n0 = len(U[u]['vias'])
        if _k in EPM:
            if _n0 >= EPM[_k]['soll']: continue
        elif not VAP: continue
        else: _g0 = _guete(u)
        save = {u: copy.deepcopy(U[u])}
        del_unit(u)
        if _k in EPM:
            VG_WEICH[0] = True; okk = route_unit(u); VG_WEICH[0] = False
            okk = okk and len(U[u]['vias']) > _n0; _g1 = 0.0; _g0 = 0.0
        else:
            PF[0] = 0.01; okk = route_unit(u); PF[0] = 1e3
            if LAGENAUFBAU: okk = okk and bool(U[u]['vias'])
            if okk: _g1 = _guete(u); okk = _g1 < _g0 - _gmin
        if okk:
            _nv += 1; _pp = D['pads'][_k]
            _cf = konflikte_gruppe([u], gruppen_index([u])); im_weg = sorted(w for w in _cf if w != u)
            if not _cf:
                _nr += 1; _gew += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
            okk = 0 < len(im_weg) <= 4 and not any(w.startswith(('GND#', 'FIX#', 'HEFT#')) for w in im_weg)
            _grund = 'uebernommen' if okk else ('fremdes Pad im Weg' if not im_weg else 'andere Stiche, feste Einheiten oder mehr als vier im Weg')
            if okk:
                if PAAR_SUCHRAUM: im_weg=sorted(set(im_weg)|{PAAR_NETZE[w] for w in im_weg if paar_einheit(w)})
                L0 = sum(ulen(w) - 0.6 * n_sv(U[w]) for w in im_weg); _v0 = sum(n_sv(U[w]) for w in im_weg)
                _f0 = sum(vg_fehl(w) for w in im_weg) if VG else 0
                for w in im_weg: save[w]=copy.deepcopy(U[w])
                for w in paar_reihe(im_weg):
                    if not paar_einheit(w): del_unit(w)
                    if not route_unit(w): okk = False
                gruppe = [u] + im_weg
                if not okk: _grund = 'kein Weg fuer die Einheiten im Weg'
                else:
                    L1 = sum(ulen(w) - 0.6 * n_sv(U[w]) for w in im_weg); _dv = sum(n_sv(U[w]) for w in im_weg) - _v0
                    if L1 > L0 + _umw * (VAP_WF if (_k in VAP_WICHTIG or _k in EPM) else 1.0): okk = False; _grund = 'Umweg %.1f mm' % (L1 - L0)
                    elif _dv > 0: okk = False; _grund = '%d Vias mehr' % _dv
                    elif VG and sum(vg_fehl(w) for w in im_weg) > _f0: okk = False; _grund = 'Via-Gruppe verliert ein Via'
                    elif konflikte_gruppe(gruppe, gruppen_index(gruppe)): okk = False; _grund = 'Konflikt bleibt'
            print('   %s:%s %s, im Weg %s: %s' % (_pp['ref'], _pp['num'], 'Massepad %d -> %d Vias' % (_n0, len(U[u]['vias'])) if _k in EPM else 'Stichkosten %.1f -> %.1f' % (_g0, _g1),
                                               im_weg[:5], _grund), flush=True)
            if okk:
                _nr += 1; _mehr += L1 - L0; _gew += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
        for w in save:
            if w in U: del_unit(w)
        for w in save: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    PF[0] = PFs; _KIDX[0] = None
    print('Platz am Pad geraeumt: %d von %d Stichen und Massepads mit besserem Platz verbessert (Stichkosten zusammen -%.1f), %d Massepad-Vias mehr, andere Bahnen dafuer %.1f mm laenger (%.0fs)' % (
        _nr, _nv, _gew, _nep, _mehr, time.time() - t2), flush=True)

def net_mst(n):
    pts = [(D['pads'][k]['x'], D['pads'][k]['y']) for k in bynet[n]]
    inT = [pts[0]]; rest = pts[1:]; L = 0
    while rest:
        d, i = min((min(math.dist(r, t) for t in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L
def unit_box(u):
    xs, ys = [], []
    for t in U[u]['tracks']:
        for q in t['pts']: xs.append(q[0]); ys.append(q[1])
    for v in U[u]['vias']: xs.append(v['x']); ys.append(v['y'])
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None
if args.get('umleg') == '1' and not todo and not zeit_knapp():
    t3 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _ugw = [0, 0]
    for rnd in range(int(args.get('umleg_runden', '3'))):
        cand = []
        for u in units:
            if paar_einheit(u) or u.startswith('GND#') or u not in U: continue
            m = net_mst(u); L = ulen(u)
            if L - m > float(args.get('umleg_min', '1.5')) and L / max(m, 0.1) > 1.4: cand.append(((L - m) * (u_gew(u) if GEWICHTET else 1.0), u))
        cand.sort(reverse=True); done = 0
        for exc, u in cand:
            if zeit_knapp(0.9): break
            pads = [D['pads'][k] for k in bynet[u]]
            bx = (min(p['x'] for p in pads) - 0.6, min(p['y'] for p in pads) - 0.6, max(p['x'] for p in pads) + 0.6, max(p['y'] for p in pads) + 0.6)
            gruppe = [u]
            for w in list(U):
                if w == u or w.startswith(('FIX#', 'HEFT#')): continue
                b2 = unit_box(w)
                if b2 and not (b2[0] > bx[2] or b2[2] < bx[0] or b2[1] > bx[3] or b2[3] < bx[1]): gruppe.append(w)
            if PAAR_SUCHRAUM:
                gruppe=sorted(set(gruppe)|{PAAR_NETZE[w] for w in gruppe if paar_einheit(w)})
            if len(gruppe) > int(args.get('umleg_max', '40')): continue
            save = {w: copy.deepcopy(U[w]) for w in gruppe}
            _ux = gruppen_index(gruppe) if GEWICHTET else None
            L0 = sum(routing_mass(w) for w in gruppe)
            if GEWICHTET:
                _g0 = {w: (u_gew(w), routing_mass(w), routing_mass_gew(w)) for w in gruppe}
            for w in gruppe: del_unit(w)
            okk = True; hsave = hist.copy(); PF[0] = 1.0
            order = [u] + sorted((w for w in gruppe if w != u), key=grp_key)
            for w in paar_reihe(order):
                if not route_unit(w): okk = False
            for k_ in range(int(args.get('umleg_iter', '12'))):
                cfg = konflikte_gruppe(gruppe, _ux) if _ux is not None else exact_conflicts()
                if args.get('umleg_pruef') == '1': assert set(cfg) == set(exact_conflicts()), 'Umleg-Gruppenpruefung'
                bad = [w for w in cfg if w in gruppe]
                if not bad or not okk: break
                for w, cells in cfg.items():
                    if w in gruppe: hist[np.array(cells, dtype=np.int64)] += 1.0
                PF[0] *= 1.4
                for w in paar_reihe([w for w in order if w in bad]):
                    if not paar_einheit(w): del_unit(w)
                    if not route_unit(w): okk = False
            PF[0] = 1e3; hist[:] = hsave
            L1 = sum(routing_mass(w) for w in gruppe if w in U)
            _besser = L1 < L0 - 0.3
            if GEWICHTET and okk:



                _besser, _gw0, _gw1, _umgekehrt = _tausch_besser([(_g0[w][0], _g0[w][1], routing_mass(w), _g0[w][2], routing_mass_gew(w)) for w in gruppe])
                _ugw[0] += 1
                if _besser != (L1 < L0 - 0.3):
                    _ugw[1] += 1
                    print('   umleg %-24s gewichtet %s: nackt %.1f -> %.1f mm, gewichtet %.1f -> %.1f%s' % (
                        u, 'angenommen' if _besser else 'abgelehnt', L0, L1, _gw0, _gw1, ', ein wichtigeres Netz wuerde laenger' if _umgekehrt else ''), flush=True)
            if okk and _besser and not (konflikte_gruppe(gruppe, _ux) if _ux is not None else exact_conflicts()):
                done += 1; print('   umgelegt %-24s Gruppe %2d: %.1f -> %.1f mm' % (u, len(gruppe), L0, L1), flush=True)
            else:
                for w in gruppe:
                    if w in U: del_unit(w)
                for w in gruppe: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
        print('Umlegen Runde %d: %d Gruppen verbessert (%.0fs)' % (rnd + 1, done, time.time() - t3), flush=True)
        if not done: break
    PF[0] = PFs
    if GEWICHTET: print('Umlegen gewichtet (R21): %d vollstaendige Versuche, %d davon anders entschieden als nach nackter Laenge' % tuple(_ugw), flush=True)







EINZELTEST = args.get('einzeltest', '0') == '1'
ET_ORT = args.get('et_ort', 'nach')
def einzeltest():
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analyse.kopplung import kopplung as _kopplung, mengen as _kmengen
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _KIDX[0] = None
    _ET_O, _ET_S = _kmengen(KL)
    _ET_GEW = GEW_ART
    if _ET_GEW is None:
        try:
            from gemeinsam.wichtigkeit import gewichte as _gw_et
            _ET_GEW = _gw_et()
        except Exception: _ET_GEW = {}
    _p_via = float(args.get('et_via', args.get('kurz_via_mm', '3.0'))); _p_schmal = float(args.get('et_schmal', '2.0'))
    _p_kopp = float(args.get('et_kopp', '1.0')); _et_min = float(args.get('et_min', '0.1'))
    _et_vmehr = int(args.get('et_vias_mehr', '0')); _et_var = int(args.get('et_varianten', '2')); _et_stoer = float(args.get('et_stoer', '2.0'))
    _HART = ('strom', 'versorgung', 'sw')
    def _et_art(u): return KL['netze'].get(U[u]['net'], {}).get('art', 'signal')
    def _et_schmal(u):
        ""
        r = U[u]; C = cls(r['net']); Cs = cls('/__signal__'); s = 0.0
        WF = hals_feld_karte(r['net'], C, C['wmin'], False)
        for t in r['tracks']:
            if t.get('steg'): continue
            soll = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= soll - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                if WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < soll: continue
                s += math.dist(a, b)
        return s
    def _et_tr(w): return [dict(net=U[w]['net'], layer=t['layer'], pts=t['pts'], w=t['w'], kelvin=bool(t.get('abg'))) for t in U[w]['tracks']]
    def _et_kopp(u):
        ""
        net = U[u]['net']; k = 0.0
        if net in _ET_O or any(t.get('abg') for t in U[u]['tracks']):
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] in _ET_S for x in _et_tr(w)])
            k += _kopplung(R, KL, opfer={net}, stoerer=_ET_S)['summe']
        if net in _ET_S:
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] != net and (U[w]['net'] in _ET_O or any(t.get('abg') for t in U[w]['tracks'])) for x in _et_tr(w)])
            k += _kopplung(R, KL, stoerer={net: _ET_S[net]})['summe']
        return k
    def _et_mass(u):
        nv = u_vias(u); Lb = ulen(u) - 0.6 * nv; g = _ET_GEW.get(_et_art(u), 1.0)
        pv = _p_via * (2.0 if ist_schnell(U[u]['net']) else 1.0); ps = _p_schmal * (2.5 if _et_art(u) in _HART else 1.0)
        Ls = _et_schmal(u); K = _et_kopp(u)
        return dict(note=g * Lb + pv * nv + ps * Ls + _p_kopp * K, L=Lb, v=nv, s=Ls, k=K, f=vg_fehl(u) if VG else 0)
    def _et_ok(u, m0, m1):
        art = _et_art(u); hart = art in _HART
        if m1['note'] >= m0['note'] - _et_min: return False
        if m1['L'] > m0['L'] * 1.10 + 1.0: return False
        if m1['v'] > m0['v'] + (0 if hart or ist_schnell(U[u]['net']) else _et_vmehr): return False
        if m1['s'] > m0['s'] + (1e-6 if hart else 0.3): return False
        if m1['k'] > m0['k'] + 1e-6 or m1['f'] > m0['f']: return False
        return True
    _et_units = sorted([u for u in units if u in U and not paar_einheit(u) and not u.startswith(('GND#', 'FIX#', 'HEFT#')) and U[u]['tracks']
                        and not any(v.get('gr') or v.get('n') for v in U[u]['vias'])], key=lambda u: (grp(u), u))
    _et_n = 0; _et_var2 = 0; _et_d = dict(note=0.0, L=0.0, v=0, s=0.0, k=0.0); _et_det = []
    for u in _et_units:
        if zeit_knapp(0.9): break
        old = copy.deepcopy(U[u]); m0 = _et_mass(u); best = None
        varianten = [1.0] + ([_et_stoer] if _et_var >= 2 and stoer_extra(U[u]['net']) is not None else [])
        del_unit(u)
        for vi, fak in enumerate(varianten):
            STOER_FAK[0] = fak; okk = route_unit(u); STOER_FAK[0] = 1.0
            if okk:
                m1 = _et_mass(u)
                if _et_ok(u, m0, m1) and (best is None or m1['note'] < best[0]['note']) and einheit_konfliktfrei(u):
                    best = (m1, copy.deepcopy(U[u]), vi)
            del_unit(u)
        if best is not None:
            m1, nu, vi = best; add_unit(u, nu['net'], nu['tracks'], nu['vias']); kidx_geaendert(u)
            _et_n += 1; _et_var2 += vi
            for k_ in _et_d: _et_d[k_] += m1[k_] - m0[k_]
            _et_det.append('%s %.1f->%.1f (L %+.1f, V %+d, schmal %+.2f, Kopplung %+.2f%s)' % (
                u.split('/')[-1], m0['note'], m1['note'], m1['L'] - m0['L'], m1['v'] - m0['v'], m1['s'] - m0['s'], m1['k'] - m0['k'], ', Stoerkosten x%g' % _et_stoer if vi else ''))
        else:
            add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
    PF[0] = PFs; _KIDX[0] = None
    print('Einzeltest: %d von %d Einheiten besser (%d mit doppelten Stoerkosten); Mass %+.1f, Bahn %+.1f mm, Vias %+d, unter Mindestbreite %+.2f mm, Kopplung %+.2f (%.0fs)' % (
        _et_n, len(_et_units), _et_var2, _et_d['note'], _et_d['L'], _et_d['v'], _et_d['s'], _et_d['k'], time.time() - t2), flush=True)
    for _z in _et_det: print('   ' + _z, flush=True)
if EINZELTEST and ET_ORT == 'vor' and KL is not None and not todo and not zeit_knapp(): einzeltest()
def vs_kennzahlen(tracks, vias):
    ""
    L = 0.0; Lo = 0.0; knicke = 0; kurz_ = 0; seg = 0
    for t in tracks:
        rv = None; fl = t['net'] in FLNETZE
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b); L += d; seg += 1
            if not fl: Lo += d
            if d < 0.1: kurz_ += 1
            if d < 1e-9: continue
            r = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
            if rv is not None and rv[0] * r[0] + rv[1] * r[1] < 0.99985: knicke += 1
            rv = r
    art = {}
    if KL is not None:
        for v in vias:
            a_ = 'gnd' if v['net'] in GNDS else (KL['netze'].get(v['net']) or {}).get('art', 'signal'); art[a_] = art.get(a_, 0) + 1
    return dict(vias=len(vias), vias_ohne_flaeche=sum(1 for v in vias if v['net'] not in FLNETZE),
                strom_vias=sum(1 for v in vias if v.get('strom_via') or v.get('gr')), bahnen=len(tracks), segmente=seg,
                knicke=knicke, seg_unter_0_1=kurz_, laenge=round(L, 2), laenge_ohne_flaeche=round(Lo, 2),
                **({'vias_je_art': dict(sorted(art.items()))} if KL is not None else {}))


def kennzahlen_jetzt():
    return vs_kennzahlen([dict(net=r['net'], pts=t['pts']) for r in U.values() for t in r['tracks']],
                      [dict(v, net=r['net']) for r in U.values() for v in r['vias']])




def lokal_anschluss(pts, A, B, capL, li, net, w):
    q = [tuple(x) for x in pts]; q[0] = tuple(A); q[-1] = tuple(B)
    out = [q[0]]
    for a, b in zip(q[:-1], q[1:]):
        if math.dist(a, b) < 1e-8: continue
        def frei(x, y): return line_cap_min(capL, x, y) >= w - 1e-6 and pad_room(li, x, y, net) >= w / 2 - 1e-6
        if not okt_netz(net) or okt(a, b):
            if not frei(a, b): return None
            out.append(b)
        else:
            c = next((c for c in _ecken(a, b) if frei(a, c) and frei(c, b)), None)
            if c is None: return None
            out.extend((c, b))
    return out if len(out) >= 2 else None

def ast_verschieben(w):
    ""
    old = copy.deepcopy(U[w]); net = old['net']; C = cls(net)
    if paar_einheit(w) or w.startswith(('FIX#', 'HEFT#', 'GND#')) or (KL and (KL['netze'].get(net) or {}).get('quarz')): return False
    for ti, t in enumerate(old['tracks']):
        if t.get('abg') or t.get('steg') or len(t['pts']) < 3: continue
        A, B = t['pts'][0], t['pts'][-1]; li = LI[t['layer']]; ln = _LS(t['pts'])
        innen = ln.difference(_PT(A).buffer(0.03).union(_PT(B).buffer(0.03)))
        rest = [q for j, q in enumerate(old['tracks']) if j != ti]
        kontakte = [(_LS((a, b)).buffer(max(t['w']) / 2).intersection(_LS((c, d)).buffer(ww / 2)))
                    for a, b in zip(t['pts'][:-1], t['pts'][1:]) for q in rest if q['layer'] == t['layer'] for c, d, ww in zip(q['pts'][:-1], q['pts'][1:], q['w'])]
        rand = max([max(t['w'])] + [max(q['w']) for q in rest]) + 0.03
        enden = _PT(A).buffer(rand).union(_PT(B).buffer(rand))
        if any(not g.difference(enden).is_empty for g in kontakte): continue
        if any(innen.distance(_PT(v['x'], v['y'])) < VIA_D / 2 for v in old['vias']): continue
        box = (min(q[0] for q in t['pts']) - 1, min(q[1] for q in t['pts']) - 1, max(q[0] for q in t['pts']) + 1, max(q[1] for q in t['pts']) + 1)
        del_unit(w); add_unit(w, net, rest, old['vias']); NET_CUR[0] = net
        ch, vh, cs, dd, rng, vb, cc, cf, vbf = karten(net, box, C['w'])
        def node(q): return li * NN + int((q[1] - Y0) / RES) * NX + int((q[0] - X0) / RES)
        src, tgt = node(A), node(B)
        if ch[li].ravel()[src % NN] < min(t['w']) or ch[li].ravel()[tgt % NN] < min(t['w']): path = None
        else: path = astar_pf([src], {tgt}, (B[0], B[1], B[0], B[1]), cc, cs, np.zeros_like(vh), vb, C, box)
        Q = None
        if path:
            tr, vi = path_to_geometry(path, cc, C)
            if len(tr) == 1 and not vi:
                Q = lokal_anschluss(tr[0]['pts'], A, B, cc[li], li, net, min(t['w']))
        del_unit(w)
        if Q is not None:
            nt = dict(t, pts=Q, w=[min(t['w'])] * (len(Q) - 1)); neu = list(old['tracks']); neu[ti] = nt
            add_unit(w, net, neu, old['vias'])
            if einheit_konfliktfrei(w): return True
            del_unit(w)
        add_unit(w, net, old['tracks'], old['vias'])
    return False




























VS_LOG = []
if args.get('vias_sparen') == '1' and not todo and not zeit_knapp():
    t4 = time.time(); _vs_c0 = time.process_time()
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    try: from analyse.bewertung import GEW as VS_GEW
    except Exception: VS_GEW = {'sw': 4.0, 'strom': 2.5, 'versorgung': 1.0, 'schnell': 3.0, 'analog': 2.0, 'signal': 1.0}
    VS_MM = float(args.get('vs_mm', '3.0'))
    VS_VAR = [(float(x.split(':')[0]), float(x.split(':')[1]) if ':' in x else VS_MM) for x in args.get('vs_var', '1000/3').split('/') if x]
    VS_HART = float(args.get('vs_hart', '1000')); VS_GR = int(args.get('vs_gruppe', '6')); VS_IT = int(args.get('vs_iter', '4'))
    VS_MIN = float(args.get('vs_min', '1.0')); VS_TOL = float(args.get('vs_tol', '0.1'))
    VS_EPS = float(args.get('vs_gewinn', '0.5'))
    VS_KNICK = float(args.get('vs_knick', '0'))
    VS_SCHUTZ = args.get('vs_schutz', '1') == '1'
    VS_STROM = args.get('vs_strom', '0') == '1'
    VS_BREITE = args.get('vs_breite', '1') == '1'
    VS_RUECK = float(args.get('vs_rueck', '1.0'))
    VS_AMPAD = float(args.get('vs_ampad', '0.5'))
    VS_REIN = args.get('vs_rein', '0') == '1'
    VS_PADS = int(args.get('vs_pads', '6'))
    VS_GEDULD = int(args.get('vs_geduld', '1'))
    VS_NACHBAR_VIAS = args.get('vs_nachbar_vias', '1') == '1'
    VS_HOEHER = args.get('vs_hoeher', '1') == '1'
    _vs_p = sum(len(bynet[u]) for u in units if not u.startswith('GND#')) + sum(1 for u in units if u.startswith('GND#'))
    VS_ARBEIT = float(args.get('vs_arbeit', '6')) * _vs_p; _vs_arb = [0, 0.0, 0.0, 0.0]
    VS_BUDGET = max(float(args.get('vs_zeit_min', '30')), float(args.get('vs_zeit', '1.0')) * _vs_c0)
    VS_PRUEF = args.get('vs_pruef') == '1'
    _vs_alt = {w: (r['net'], r['tracks'], r['vias']) for w, r in U.items()}
    hsave = hist.copy(); hist[:] = 0; PFs = PF[0]; _KIDX[0] = None
    kz0 = kennzahlen_jetzt()
    def vs_vias(w): return sum(1 for v in U[w]['vias'] if not v.get('gr') and not v.get('rv'))
    def u_len(w): return sum(math.dist(a, c) for t in U[w]['tracks'] if not t.get('steg') for a, c in zip(t['pts'][:-1], t['pts'][1:]))
    def u_fehl(w): return vg_fehl(w) if VG else 0
    def u_knicke(w):
        n_ = 0
        for t in U[w]['tracks']:
            rv = None
            for a, b in zip(t['pts'][:-1], t['pts'][1:]):
                d = math.dist(a, b)
                if d < 1e-9: continue
                r_ = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
                if rv is not None and rv[0] * r_[0] + rv[1] * r_[1] < 0.99985: n_ += 1
                rv = r_
        return n_
    def vs_art(w):
        if w.startswith('GND#') or w in GNDS: return 'gnd'
        if KL is not None: return (KL['netze'].get(w) or {}).get('art', 'signal')
        return 'strom' if cls(w)['prio'] < 8 else 'signal'
    def vs_schutz(w): return paar_einheit(w) or VS_SCHUTZ and vs_art(w) in ('gnd', 'strom', 'versorgung', 'sw')
    def vs_strom(w): return vs_art(w) in ('gnd', 'strom', 'versorgung', 'sw')
    def vs_ist_strom(w): return VS_BREITE and not w.startswith('GND#') and vs_strom(w)
    def vs_padabst(w):
        bbs = [D['pads'][int(w[4:])]['bbox']] if w.startswith('GND#') else [D['pads'][k]['bbox'] for k in bynet_g.get(U[w]['net'], [])]
        return sorted(min(math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3])) for b in bbs)
                      for v in U[w]['vias'] if not v.get('gr') and not v.get('rv')) if bbs else []
    def vs_pad_ok(a_, n_): return not a_ or not n_ or (max(n_) <= max(a_) + 0.05 and sum(n_) / len(n_) <= sum(a_) / len(a_) + 0.05)
    def vs_leff(net, tracks):
        ""
        C = cls(net); soll = C['w']; s_ = 0.0
        xs_ = [q[0] for t in tracks for q in t['pts']] or [0.0]; ys_ = [q[1] for t in tracks for q in t['pts']] or [0.0]
        _cH, _, _cS, _dd, _, _, capC, _, _ = karten(net, (min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0), soll)
        for t in tracks:
            if t.get('steg'): continue
            li = LI[t['layer']]; dick = 1.0 if li in (0, NL - 1) else 2.0
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                d = math.dist(a, b)
                if d <= 0: continue
                if t.get('sig'): s_ += d; continue
                wv = max(w0, math.floor(min(soll, line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
                s_ += d * dick * soll / wv
        return s_
    def u_mass(w):
        if not vs_ist_strom(w): return u_len(w)
        s_ = (U[w]['net'], U[w]['tracks'], U[w]['vias']); del_unit(w); v = vs_leff(s_[0], s_[1]); add_unit(w, *s_)
        return v
    _vs_m = {}
    def u_mass0(w):
        if w not in _vs_m: _vs_m[w] = u_mass(w)
        return _vs_m[w]
    def u_gew(w): return VS_GEW.get(vs_art(w), 1.0)
    def u_vgew(w): return VS_MM * (2.0 if vs_art(w) == 'schnell' else 1.0)
    def vs_unten(n):
        gem = None
        for k in bynet[n]:
            s_ = {L for L in D['pads'][k]['layers'] if L in LI}; gem = s_ if gem is None else gem & s_
        return 0 if gem else 1
    def vs_potential(u):
        return u_gew(u) * (u_mass0(u) + u_vgew(u) * vs_vias(u) - net_mst(u) - u_vgew(u) * vs_unten(u))
    def vs_viaf(w, mm):
        if vs_schutz(w): return 1.0
        return max(1.0, mm * u_vgew(w) / VS_MM / max(cls(w)['via'], 0.05))
    def vs_pad_d(net, v):
        return min((math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3]))
                    for b in (D['pads'][k]['bbox'] for k in bynet_g.get(net, []))), default=9.0)
    def vs_maske(net, vias):
        ""
        m = np.zeros((NY, NX), dtype=bool)
        for v in vias:
            r_ = 0.0 if vs_pad_d(net, v) <= VS_AMPAD else VS_RUECK; rr = int(r_ / RES) + 1
            ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
            y0, y1, x0, x1 = max(0, iy - rr), min(NY, iy + rr + 1), max(0, ix - rr), min(NX, ix + rr + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            m[y0:y1, x0:x1] |= (xx - ix) ** 2 + (yy - iy) ** 2 <= (r_ / RES) ** 2
        return m.ravel()
    def vs_pad_vias_da(net, alt_v, neu_v):
        return all(any(abs(v['x'] - q['x']) < 0.011 and abs(v['y'] - q['y']) < 0.011 for q in neu_v) for v in alt_v if vs_pad_d(net, v) <= VS_AMPAD)
    def vs_versatz(alt_tr, neu_tr):
        m = 0.0
        for t in neu_tr:
            alte = [_LS(a['pts']) for a in alt_tr if a['layer'] == t['layer'] and len(a['pts']) >= 2]
            if not alte: return None
            for q in t['pts']: m = max(m, min(l_.distance(_PT(q[0], q[1])) for l_ in alte))
        return round(m, 2)
    _vs_eig = {}
    def vs_hart(net, tracks, vias):
        ""
        xs_ = [q[0] for t in tracks for q in t['pts']] + [v['x'] for v in vias]; ys_ = [q[1] for t in tracks for q in t['pts']] + [v['y'] for v in vias]
        if not xs_: return 0.0
        capH, viaH = hard_maps(net, _box_idx((min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0)) if GROSS else None)
        s_ = 0.0
        if net not in _vs_eig:
            _vs_eig[net] = {}
            for k in bynet_g.get(net, []):
                for L_, c in D['pads'][k]['bcells'].items(): _vs_eig[net].setdefault(LI[L_], set()).update(c.tolist())
        for t in tracks:
            li = LI[t['layer']]; capL = capH[li]; eig = _vs_eig[net].get(li, ())
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                L = math.dist(a, b)
                if L <= 0: continue
                tt = np.linspace(0, 1, max(2, int(L / (RES * 0.5)) + 1))
                ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
                eng = capL[iy, ix] < w0 - 0.005
                if eng.any(): s_ += L * sum(1 for q in (iy[eng] * NX + ix[eng]).tolist() if q not in eig) / len(tt)
        for v in vias:
            if not viaH[int((v['y'] - Y0) / RES), int((v['x'] - X0) / RES)]: s_ += VIA_D
        return s_
    def vs_fest(w, gu=None):
        if w.startswith(('FIX#', 'HEFT#')): return True
        if gu is None: return False
        if w.startswith('GND#'): return True
        if KL is not None and (KL['netze'].get(w) or {}).get('quarz'): return True
        if len(bynet[w]) > VS_PADS and not (args.get('vs_aeste') == '1' and not vs_schutz(w)): return True
        return gu is not None and not VS_HOEHER and u_gew(w) > gu + 1e-9
    def vs_route(w, wer):
        _t = time.process_time(); _vs_arb[0] += 1 if w.startswith('GND#') else len(bynet[w])
        ok_ = (ast_verschieben(w) if wer == 2 and w in U and args.get('vs_aeste') == '1' and not vs_schutz(w) else False)
        if not ok_ and not (wer == 2 and args.get('vs_aeste') == '1' and len(bynet[w]) > VS_PADS):
            if w in U: del_unit(w)
            ok_ = route_unit(w)
        _vs_arb[wer] += time.process_time() - _t
        return ok_
    def vs_versuch(u, pfw, mm, X):
        ""
        G = [u]; alt = {u: (U[u]['net'], U[u]['tracks'], U[u]['vias'])}; masken = {}
        wert = {u: (vs_vias(u), u_len(u), u_fehl(u), u_mass0(u), vs_padabst(u) if vs_schutz(u) else None, u_knicke(u) if VS_KNICK else 0)}; gu = u_gew(u)
        def k0(w): return u_gew(w) * (wert[w][3] + u_vgew(w) * wert[w][0] + VS_KNICK * wert[w][5])
        def k1(w): return u_gew(w) * (u_mass(w) + u_vgew(w) * vs_vias(w) + (VS_KNICK * u_knicke(w) if VS_KNICK else 0.0))
        hc = []; frei = False; verlauf = []; grund = 'kein eigener Gewinn'
        if VS_RUECK > 0 and vs_schutz(u): masken[u] = vs_maske(alt[u][0], alt[u][2])
        del_unit(u); PF[0] = pfw; VIAF[0] = vs_viaf(u, mm); VIA_MASKE[0] = masken.get(u)
        okk = vs_route(u, 1); VIA_MASKE[0] = None
        if okk and k1(u) < k0(u) - VS_EPS:
            grund = 'Konflikte bleiben'
            for runde in range(VS_IT + 1):
                _t = time.process_time(); cfg = konflikte_gruppe(G, X, set(G)); verlauf.append(len(cfg)); _vs_arb[3] += time.process_time() - _t
                if VS_PRUEF:
                    _v = exact_conflicts(); assert set(_v) == set(cfg), ('vias_sparen-Pruefung', sorted(set(_v) ^ set(cfg)))
                if not cfg: frei = True; break
                if runde == VS_IT: break
                neu = [w for w in sorted(cfg) if w not in alt and not vs_fest(w, gu)]
                if len(G) + len(neu) > VS_GR: grund = 'Gruppe zu gross'; break
                for w in neu:
                    alt[w] = (U[w]['net'], U[w]['tracks'], U[w]['vias']); wert[w] = (vs_vias(w), u_len(w), u_fehl(w), (_vs_m[w] if w in _vs_m else u_mass(w)) if vs_ist_strom(w) else u_len(w), vs_padabst(w) if vs_schutz(w) else None, u_knicke(w) if VS_KNICK else 0); G.append(w)
                andere = sorted((w for w in G[1:] if w in cfg), key=lambda w: vs_art(w) == 'schnell')
                if runde >= VS_GEDULD or not andere:
                    for w, cells in cfg.items():
                        c = np.array(cells, dtype=np.int64); hist[c] += 1.0; hc.append(c)
                PF[0] = VS_HART
                for w in andere:
                    if VS_RUECK > 0 and vs_schutz(w):
                        if w not in masken: masken[w] = vs_maske(alt[w][0], alt[w][2])
                        VIA_MASKE[0] = masken[w]
                    if args.get('vs_aeste') != '1' or vs_schutz(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm)
                    if not vs_route(w, 2): okk = False
                    VIA_MASKE[0] = None
                if u in cfg and (runde >= VS_GEDULD or not andere):
                    PF[0] = VS_HART if andere else pfw
                    del_unit(u); VIAF[0] = vs_viaf(u, mm); VIA_MASKE[0] = masken.get(u)
                    if not vs_route(u, 1): okk = False
                    VIA_MASKE[0] = None
                    if okk and not k1(u) < k0(u) - VS_EPS: grund = 'kein eigener Gewinn'; break
                if not okk: grund = 'unvollstaendig'; break
                if sum(vs_vias(w) for w in G) > sum(wert[w][0] for w in G): grund = 'Vias der Gruppe steigen'; break
        elif not okk: grund = 'unvollstaendig'
        def hart_schlecht(): return [w for w in G if vs_hart(U[w]['net'], U[w]['tracks'], U[w]['vias']) > vs_hart(*alt[w]) + 0.05]
        if frei and okk:



            neu_z = list(G) if VS_REIN else hart_schlecht()
            if neu_z:
                PF[0] = VS_HART
                for w in neu_z:
                    if VS_RUECK > 0 and vs_schutz(w) and w not in masken: masken[w] = vs_maske(alt[w][0], alt[w][2])
                    if args.get('vs_aeste') != '1' or vs_schutz(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm); VIA_MASKE[0] = masken.get(w)
                    if not vs_route(w, 1 if w == u else 2): okk = False
                    VIA_MASKE[0] = None
                if not okk: grund = 'unvollstaendig'
                else:
                    cfg = konflikte_gruppe(G, X, set(G)); verlauf.append(len(cfg))
                    if cfg: frei = False; grund = 'Konflikte nach dem Reinzeichnen'
                    elif hart_schlecht(): frei = False; grund = 'Bahn laege in gesperrtem Bereich (Loetstopp, Kante, Bohrung)'
        erg = None
        if frei and okk:
            V0 = sum(wert[w][0] for w in G); V1 = sum(vs_vias(w) for w in G)
            L0 = sum(wert[w][1] for w in G); L1 = sum(u_len(w) for w in G)
            M1 = {w: u_mass(w) for w in G}
            K1 = {w: u_gew(w) * (M1[w] + u_vgew(w) * vs_vias(w) + (VS_KNICK * u_knicke(w) if VS_KNICK else 0.0)) for w in G}
            bilanz = sum(K1[w] - k0(w) for w in G)
            if V1 > V0: grund = 'Vias der Gruppe steigen (%d -> %d)' % (V0, V1)
            elif sum(u_fehl(w) for w in G) > sum(wert[w][2] for w in G): grund = 'Via-Gruppe unvollstaendig'
            elif any(vs_schutz(w) and vs_vias(w) < wert[w][0] for w in G): grund = 'geschuetztes Netz verloere ein Via'
            elif any(vs_schutz(w) and not (vs_pad_ok(wert[w][4], vs_padabst(w)) and vs_pad_vias_da(alt[w][0], alt[w][2], U[w]['vias'])) for w in G):
                grund = 'Via eines geschuetzten Netzes wandert vom Pad weg'
            elif any(vs_schutz(w) and M1[w] > 1.05 * wert[w][3] + VS_TOL for w in G[1:]): grund = 'Stromnetz bekaeme mehr Widerstand'
            elif any(u_gew(w) > gu + 1e-9 and (vs_vias(w) > wert[w][0] or K1[w] > k0(w) + u_gew(w) * VS_TOL) for w in G[1:]):
                grund = 'wichtigeres Netz wuerde schlechter'
            elif not VS_NACHBAR_VIAS and any(vs_vias(w) > wert[w][0] for w in G[1:]): grund = 'Nachbar bekaeme mehr Vias'
            elif not bilanz < -VS_EPS: grund = 'Bilanz %+.1f' % bilanz
            else:
                gesch = [w for w in G[1:] if U[w]['tracks'] != alt[w][1] or U[w]['vias'] != alt[w][2]]
                neu_v = {(v['x'], v['y']) for v in U[u]['vias']}
                erg = dict(netz=u, pf=pfw, via_mm=mm, gruppe=G[1:], geschoben=gesch, vias=[V0, V1], laenge=[round(L0, 2), round(L1, 2)],
                           bilanz=round(bilanz, 2), verhaeltnis=round(-bilanz / (1 + sum(u_gew(w) for w in gesch)), 3), runden=verlauf,
                           weg=[[v['x'], v['y']] for v in alt[u][2] if not v.get('gr') and not v.get('rv') and (v['x'], v['y']) not in neu_v][:6],
                           je={w: [wert[w][0], vs_vias(w), round(wert[w][1], 2), round(u_len(w), 2), vs_versatz(alt[w][1], U[w]['tracks'])] for w in G},
                           _neu={w: (U[w]['net'], U[w]['tracks'], U[w]['vias']) for w in G}, _alt=alt)
        for c in hc: hist[c] = 0.0
        for w in G:
            if w in U: del_unit(w)
        for w in G: add_unit(w, *alt[w])
        return erg, len(G), verlauf, grund
    _vs_fehler = False
    try:
        for w in [w for w in U if vs_ist_strom(w) and not w.startswith(('FIX#', 'HEFT#'))]: u_mass0(w)
        X = gruppen_index([]); _vs_aus = False; _vs_boxen = None
        for rnd in range(int(args.get('vs_runden', '2'))):
            cand = [(grp(u)[0], -vs_potential(u), u) for u in units if u in U and not u.startswith('GND#') and not vs_fest(u)
                    and (not vs_schutz(u) or (VS_STROM and vs_art(u) != 'gnd'))]
            cand = [u for _g, p_, u in sorted(cand) if -p_ >= VS_MIN]
            if _vs_boxen is not None:
                def _nahe_box(u):
                    b = unit_box(u); pb = [D['pads'][k]['bbox'] for k in bynet[u]]
                    b = (min([q[0] for q in pb] + ([b[0]] if b else [])), min([q[1] for q in pb] + ([b[1]] if b else [])),
                         max([q[2] for q in pb] + ([b[2]] if b else [])), max([q[3] for q in pb] + ([b[3]] if b else [])))
                    return any(not (b[0] > q[2] + 1.5 or b[2] < q[0] - 1.5 or b[1] > q[3] + 1.5 or b[3] < q[1] - 1.5) for q in _vs_boxen)
                cand = [u for u in cand if _nahe_box(u)]
            boxen = []; n_ok = 0; dv = 0; dl = 0.0; db = 0.0; n_vers = 0
            for u in cand:
                if _vs_arb[0] > VS_ARBEIT: _vs_aus = 'Aufwand verbraucht'; break
                if (not BUDGET_ARBEIT and time.process_time() - _vs_c0 > VS_BUDGET) or zeit_knapp(0.9): _vs_aus = 'Zeitanteil verbraucht (Ergebnis haengt damit von der Rechenzeit ab)'; break
                pot = vs_potential(u); beste = None
                for pfw, mm in VS_VAR:
                    erg, ng, verlauf, grund = vs_versuch(u, pfw, mm, X); n_vers += 1
                    if args.get('vs_log') == '1' and erg is None and (ng > 1 or grund != 'kein eigener Gewinn'):
                        print('   kein Gewinn   %-24s PF %g, Gruppe %d, Konflikte je Runde %s: %s' % (u, pfw, ng, verlauf, grund), flush=True)
                    if erg is None: continue
                    if beste is None or erg['verhaeltnis'] > beste['verhaeltnis']: beste = erg
                    if -erg['bilanz'] >= 0.9 * pot: break
                if beste is None: continue
                neu_ = beste.pop('_neu'); alt_ = beste.pop('_alt'); _nb = len(boxen)
                for w in neu_: del_unit(w)
                for w in neu_: add_unit(w, *neu_[w])
                beste['runde'] = rnd + 1; beste['potential'] = round(pot, 2); VS_LOG.append(beste)
                n_ok += 1; dv += beste['vias'][0] - beste['vias'][1]; dl += beste['laenge'][1] - beste['laenge'][0]; db += beste['bilanz']
                for w in neu_:
                    ta = {(t['layer'], tuple(map(tuple, t['pts']))) for t in alt_[w][1]}; tn = {(t['layer'], tuple(map(tuple, t['pts']))) for t in neu_[w][1]}
                    for _l, pts in sorted(ta ^ tn): boxen.append((min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts)))
                    va = {(v['x'], v['y']) for v in alt_[w][2]}; vn = {(v['x'], v['y']) for v in neu_[w][2]}
                    for x_, y_ in sorted(va ^ vn): boxen.append((x_, y_, x_, y_))
                for w in neu_: _vs_m.pop(w, None)
                for w in [w for w in list(_vs_m) if w not in neu_]:
                    b = unit_box(w)
                    if b and any(not (b[0] > q[2] + 2.0 or b[2] < q[0] - 2.0 or b[1] > q[3] + 2.0 or b[3] < q[1] - 2.0) for q in boxen[_nb:]): _vs_m.pop(w)
                for w in [w for w in U if vs_ist_strom(w) and not w.startswith(('FIX#', 'HEFT#'))]: u_mass0(w)
                X = gruppen_index([])
                print('   %-13s %-24s Vias %d -> %d, Laenge %.1f -> %.1f mm, Bilanz %+.1f (Potential %.1f), PF %g, geschoben %s, Via weg bei %s' % (
                    'Via gespart' if beste['vias'][1] < beste['vias'][0] else 'kuerzer', u, beste['vias'][0], beste['vias'][1], beste['laenge'][0],
                    beste['laenge'][1], beste['bilanz'], pot, beste['pf'],
                    ['%s (%s mm)' % (w, beste['je'][w][4]) for w in beste['geschoben']], beste['weg']), flush=True)
            print('Vias sparen Runde %d: %d Kandidaten, %d Versuche, %d uebernommen, %d Vias weniger, Laenge %+.1f mm, Bilanz %+.1f (%.0fs)%s' % (
                rnd + 1, len(cand), n_vers, n_ok, dv, dl, db, time.time() - t4, ', ' + _vs_aus if _vs_aus else ''), flush=True)
            if not n_ok or _vs_aus: break
            _vs_boxen = boxen
    except Exception:
        if VS_PRUEF: raise
        import traceback
        traceback.print_exc(file=sys.stdout); _vs_fehler = True
    VIAF[0] = 1.0; VIA_MASKE[0] = None; PF[0] = PFs; hist[:] = hsave; _KIDX[0] = None
    if _vs_fehler or exact_conflicts():
        print('Vias sparen: %s -> alles zurueck auf den Stand davor' % ('Fehler im Nachlauf (siehe oben)' if _vs_fehler else 'Schlusspruefung findet Konflikte %s' % sorted(set((a, b) for (_l, _d, a, b, _x, _y) in PAIRS))[:6]), flush=True)
        for w in list(U): del_unit(w)
        for w, s_ in _vs_alt.items(): add_unit(w, *s_)
        VS_LOG.clear()



    for w in _vs_alt:
        if w in U: U[w] = U.pop(w)
    kz1 = kennzahlen_jetzt()
    print('Vias sparen: Vias %d -> %d (ohne Flaechennetze %d -> %d), Knicke %d -> %d, Laenge %.1f -> %.1f mm, CPU %.0f s (Kandidat %.0f, Partner %.0f, Pruefung %.0f), Aufwand %d von %d Anschluessen' % (
        kz0['vias'], kz1['vias'], kz0['vias_ohne_flaeche'], kz1['vias_ohne_flaeche'], kz0['knicke'], kz1['knicke'],
        kz0['laenge'], kz1['laenge'], time.process_time() - _vs_c0, _vs_arb[1], _vs_arb[2], _vs_arb[3], _vs_arb[0], VS_ARBEIT), flush=True)



R19_FORM = args.get('r19_form', '0') == '1' and KL is not None
R19_FORM_STAT = dict(geprueft=0, verworfen=0, beispiele=[])
def r19_form_ok(alt, neu):
    if not R19_FORM or (KL['netze'].get(alt['net']) or {}).get('art') not in ('signal', 'analog', 'schnell'): return True
    from analyse.r19 import form_ok, formmasse
    R19_FORM_STAT['geprueft'] += 1
    if form_ok(alt, neu): return True
    R19_FORM_STAT['verworfen'] += 1
    if len(R19_FORM_STAT['beispiele']) < 12:
        R19_FORM_STAT['beispiele'].append(dict(net=alt['net'], alt=formmasse(alt), neu=formmasse(neu)))
    return False
if HYPO_SPAET and not todo and not zeit_knapp():
    _HYPO_AN[0] = True; t2 = time.time(); PFs = PF[0]; PF[0] = 1e3; hist[:] = 0; _KIDX[0] = None; _nu = 0
    _htol = float(args.get('hypo_tol', '0.01'))
    _rest = sorted([u for u in units if u in U and not paar_einheit(u) and not (u.startswith('GND#') and int(u[4:]) in EPM)], key=kurz_key); _warum = {}
    for _rd in range(2):
        _bleibt = []
        for u in _rest:
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); L0 = ulen(u); _v0 = n_sv(U[u]); _f0 = vg_fehl(u) if VG else 0
            del_unit(u)
            okk = route_unit(u)
            if okk and ulen(u) <= L0 * (1 + _htol) + 0.2 and n_sv(U[u]) <= _v0 and (not VG or vg_fehl(u) <= _f0) and einheit_konfliktfrei(u) and r19_form_ok(old, U[u]):
                _nu += 1; kidx_geaendert(u); continue
            _warum[u] = 'unvollstaendig' if not okk else ('laenger %.1f statt %.1f' % (ulen(u), L0) if ulen(u) > L0 * (1 + _htol) + 0.2 else
                         ('Vias %d statt %d' % (n_sv(U[u]), _v0) if n_sv(U[u]) > _v0 else 'Via-Gruppe oder Konflikt'))
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u); _bleibt.append(u)
        _rest = _bleibt
        if not _rest: break
    PF[0] = PFs
    print('Umformen: %d Einheiten neu geformt, %d in alter Form (%.0fs)' % (_nu, len(_rest), time.time() - t2), {u: _warum[u] for u in _rest if not u.startswith('GND#')}, flush=True)
if EINZELTEST and ET_ORT != 'vor' and KL is not None and not todo and not zeit_knapp(): einzeltest()
if PAAR_NETZE and not todo and not zeit_knapp():
    print('Differenzpaare (R29) gekoppelt: %d' % paar_koppeln(), flush=True)
if RUECK > 0 and not todo and not zeit_knapp():
    _rvn = rv_nachlauf(); print('Rueck-Vias (Q2) nach der Verhandlung: %d nachgetragen, %d ueberfluessige entfernt' % _rvn, flush=True)
if KL is not None and args.get('fluss', '1') == '1':
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from gemeinsam.fluss import analyse as _fl_analyse, zielbreite as _fl_ziel
    _nf = 0
    for u, r in list(U.items()):
        if u.startswith(('GND#', 'FIX#', 'HEFT#')): continue
        k = KL['netze'].get(r['net'], {})
        if k.get('art') not in (('strom', 'versorgung', 'sw') if args.get('strom_fix') == '1' else ('strom', 'versorgung')) or not k.get('fluss'): continue
        info = _fl_analyse([t for t in r['tracks']], r['vias'], [D['pads'][i] for i in bynet[r['net']]], k['fluss'], endpunkte=args.get('strom_fix') == '1')
        if not info: continue
        C_ = cls(r['net'])
        FLUSS_W[u] = {sid: _fl_ziel(a_, C_['wmin'], C_['w'], float(args.get('fluss_exp', '0.5'))) for sid, a_ in info['anteil'].items()}
        if strom_modell(r['net']):
            FLUSS_W[u] = {sid: max(BAHN_MIN,min(HALS_W,C_['wmin']),info['gesamt'] * a_ / (STROM_J * STROM_T[r['tracks'][sid[0]]['layer']])) for sid,a_ in info['anteil'].items()}
        FLUSS_A[u] = info['anteil']
        _nf += 1
    print('Flussprinzip: Zielbreiten fuer %d Stromnetze' % _nf, flush=True)





FEINSCHUB_LOG = []
def via_feinschub():
    from analyse.bewertung import bewerte as _vf_note, unverbunden as _vf_unverbunden
    from analyse.topologie import topologie as _vf_topo
    from analyse.kopplung import kopplung as _vf_kopp
    def snapshot():
        tracks, vias = ausgabe_geometrie()
        return dict(tracks=tracks, vias=vias, conflicts={}, failed=[], optionen=args,
                    gndlagen=GNDL, flaechennetze=sorted(FLNETZE))
    def tore(R):
        e = _vf_note(D, R, KL, v3=True); tp = _vf_topo(R, D)
        gueltig = e['diagnose'] and e['konflikte'] == 0 and e['offen'] == 0 and not _vf_unverbunden(D, R, set(KL['gnd']) | FLNETZE)
        return gueltig, e['note_v3'], _vf_kopp(R, KL)['summe'], tuple(tp[k] - tp.get(k + '_pad', 0) for k in ('stern', 'spitz')), tp['parallel_rest_mm'], tp['kelvin_nah_mm']
    def besser(a, b): return b[0] and all(y <= x + 1e-7 for x, y in zip((a[1], a[2], *a[3], a[4], a[5]), (b[1], b[2], *b[3], b[4], b[5])))
    start = time.process_time(); versuche = 0; reihe = list(U); _KIDX[0] = None; verschoben = set()
    power = [u for u in U if not u.startswith(('FIX#', 'HEFT#', 'GND#')) and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung')]
    for pu in power:
        if zeit_knapp() or (not BUDGET_ARBEIT and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
        C = cls(U[pu]['net'])
        eng = [(LI[t['layer']], _LS((a, b)), max(C['wmin'], FLUSS_W.get(pu, {}).get((ti, si), C['w'])))
               for ti, t in enumerate(U[pu]['tracks']) if not t.get('sig') and not t.get('steg')
               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if w < C['w'] - 1e-6 and math.dist(a, b) > 0.3]
        for u in list(U):
            r = U[u]; k = KL['netze'].get(r['net']) or {}
            if u.startswith(('FIX#', 'HEFT#', 'GND#')) or r['net'] in GNDS or k.get('quarz') or k.get('art') in ('strom', 'versorgung', 'sw', 'schnell'): continue
            for vi, v in enumerate(list(r['vias'])):
                if (u, vi) in verschoben or v.get('gr') or v.get('rv') or v.get('n'): continue
                A = (v['x'], v['y'])
                if any(_PT(A).distance(_LS(poly + [poly[0]])) < 0.5 or _PT(A).within(Polygon(poly))
                       for pi in bynet_g.get(r['net'], []) for polys in D['pads'][pi].get('polys', {}).values() for poly in polys): continue
                near = [(li, ln, goal) for li, ln, goal in eng if ln.distance(_PT(A)) < goal / 2 + VIA_D / 2 + CLR + 0.3]
                if not near: continue
                inc = [(ti, e) for ti, t in enumerate(r['tracks']) for e in (0, -1) if math.dist(t['pts'][e], A) < 1e-6]
                if len(inc) < 2 or any(_LS((a, b)).distance(_PT(A)) < VIA_D / 2 + w / 2
                                      for tj, t in enumerate(r['tracks']) for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))
                                      if ((tj, 0) not in inc and (tj, -1) not in inc) or ((tj, 0) in inc and si > 0 and (tj, -1) not in inc) or ((tj, -1) in inc and si < len(t['w']) - 1 and (tj, 0) not in inc)): continue
                if any(r['tracks'][ti].get('abg') or r['tracks'][ti].get('steg') for ti, e in inc): continue
                old = copy.deepcopy(r); pwold = copy.deepcopy(U[pu]); before = tore(snapshot())
                for ti, e in inc:
                    q = old['tracks'][ti]['pts'][1 if e == 0 else -2]; d = math.dist(A, q)
                    for step in (0.05, 0.1, 0.2, 0.3):
                        if step >= d - 0.05 or versuche >= int(args.get('vf_versuche', '60')) or zeit_knapp() or (not BUDGET_ARBEIT and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
                        versuche += 1; B = (A[0] + (q[0] - A[0]) * step / d, A[1] + (q[1] - A[1]) * step / d)
                        del_unit(u); net = old['net']; Cn = cls(net); ch, vh, cs, dd, rng, vb, cc, cf, vbf = karten(net, (B[0]-2,B[1]-2,B[0]+2,B[1]+2), Cn['w'])
                        ix, iy = int((B[0]-X0)/RES), int((B[1]-Y0)/RES); neu = copy.deepcopy(old); ok = 0 <= ix < NX and 0 <= iy < NY and vh[iy, ix] and not vb[iy, ix]
                        for tj, end in inc:
                            t = neu['tracks'][tj]; P = t['pts'] if end == 0 else t['pts'][::-1]; W = t['w'] if end == 0 else t['w'][::-1]
                            Q = lokal_anschluss([B, P[1]], B, P[1], cc[LI[t['layer']]], LI[t['layer']], net, W[0]) if ok else None
                            if Q is None: ok = False; break
                            Q += P[2:]; ww = [W[0]] * (len(Q) - len(P) + 1) + W[1:]
                            t['pts'], t['w'] = (Q, ww) if end == 0 else (Q[::-1], ww[::-1])
                        if ok:
                            neu['vias'][vi] = dict(v, x=B[0], y=B[1]); add_unit(u, net, neu['tracks'], neu['vias']); _KIDX[0] = None; ok = einheit_konfliktfrei(u)
                            if ok:
                                widen_unit(pu, C); _KIDX[0] = None; after = tore(snapshot()); ok = einheit_konfliktfrei(pu) and besser(before, after) and strom_widerstand(pu) < strom_widerstand_alt(pwold, pu) - 1e-9
                        if ok:
                            verschoben.add((u, vi)); FEINSCHUB_LOG.append(dict(netz=u, strom=pu, alt=A, neu=B, versatz_mm=step)); kidx_geaendert(u); kidx_geaendert(pu); break
                        if u in U: del_unit(u)
                        add_unit(u, old['net'], old['tracks'], old['vias'])
                        del_unit(pu); add_unit(pu, pwold['net'], pwold['tracks'], pwold['vias']); _KIDX[0] = None
                    if FEINSCHUB_LOG and FEINSCHUB_LOG[-1]['alt'] == A: break
                r = U[u]
    for w in reihe: U[w] = U.pop(w)
    _KIDX[0] = None
    print('Via-Feinschub: %d uebernommen, %d Versuche, CPU %.1f s' % (len(FEINSCHUB_LOG), versuche, time.process_time()-start), flush=True)

def via_dichte_nachlauf():
    ""
    global VIA_DICHTE
    from analyse.via_dichte import messen as dichte, kurz, schutznetze, via_geschuetzt
    from analyse.bewertung import bewerte
    from analyse.regelkennzahlen import annahme as regeltor
    from analyse.vertrag import vergleich as vertragstor
    from analyse.minimax import annahme as minimaxtor
    from analyse.wackel import messen as wackel
    start = time.process_time(); reihe = list(U); kosten = VIA_DICHTE; pf = PF[0]
    def knapp():
        return zeit_knapp(0.95) or (not BUDGET_ARBEIT and time.process_time()-start >= float(args.get('via_dichte_nach_cpu', '45')))
    def snapshot():
        tr, vi = ausgabe_geometrie()
        return ausgabe_daten(tr, vi, {}, fehlend=[])
    def qualitaet(R):
        return bewerte(D, R, KL, v3=True, **{k: str(args.get(k, '0')).lower() in ('1', 'true')
                       for k in ('si_note', 'si_messen', 'laengentor')}), wackel(D, R, gnd=KL.get('gnd') or [])
    try:
        if knapp():
            VD_NACH_LOG.update(grund='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        base = snapshot(); vd0 = dichte(D, base, fenster=VD_FENSTER, schwelle=VD_SCHWELLE)
        if not vd0['fenster_ueber'] and vd0['spitze_vias_mm2'] <= VD_SCHWELLE:
            VD_NACH_LOG.update(grund='unter_schwelle', cpu_s=round(time.process_time()-start, 3)); return
        if knapp():
            VD_NACH_LOG.update(grund='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        e0, w0 = qualitaet(base)
        protected = schutznetze(KL, base, IMP_TAB)
        if not e0['gueltig']:
            VD_NACH_LOG.update(grund='bewertung_ungueltig', cpu_s=round(time.process_time()-start, 3)); return
    except Exception as exc:
        VD_NACH_LOG.update(grund='diagnosefehler', fehler=[repr(exc)[:160]], cpu_s=round(time.process_time()-start, 3)); return
    cand = []
    for u in sorted(U):
        r = U[u]
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#', 'GND#')): continue
        if not r['vias'] or any(via_geschuetzt(dict(v, net=r['net']), KL, protected) for v in r['vias']): continue
        score = sum(float(_vd_kosten()[int((v['y']-Y0)/RES), int((v['x']-X0)/RES)])
                    for v in r['vias'] if 0 <= int((v['x']-X0)/RES) < NX and 0 <= int((v['y']-Y0)/RES) < NY)
        if score: cand.append((-score, u))
    try:
        VIA_DICHTE = True; PF[0] = 1e3; _KIDX[0] = None
        for _, u in sorted(cand)[:int(args.get('via_dichte_nach_max', '8'))]:
            if knapp(): break
            old = copy.deepcopy(U[u]); saved_fail = set(fail)
            saved_unreachable = copy.deepcopy(UNERREICHBAR)
            VD_NACH_LOG['versuche'] += 1
            accepted = False
            try:
                del_unit(u)
                if route_unit(u) and not knapp() and einheit_konfliktfrei(u) and not exact_conflicts():
                    neu = snapshot(); q = kurz(neu, KL, VD_FENSTER, VD_SCHWELLE)
                    if q['spitze_vias_mm2'] <= vd0['spitze_vias_mm2'] and q['vias'] <= vd0['vias']:
                        if knapp(): continue
                        vd1 = dichte(D, neu, fenster=VD_FENSTER, schwelle=VD_SCHWELLE)
                        if (vd1['ueberlast_integral_mm2'] < vd0['ueberlast_integral_mm2'] - 1e-7
                                and vd1['fenster_ueber'] <= vd0['fenster_ueber']):
                            if knapp(): continue
                            e1, w1 = qualitaet(neu)
                            accepted = e1['gueltig'] and regeltor(e0.get('regeln'), e1.get('regeln'))['nicht_schlechter']
                            if accepted and any(e.get('tor') or (e.get('regeln') or {}).get('messversion') for e in (e0, e1)):
                                accepted = vertragstor(e0, e1)['nicht_schlechter']
                            if accepted and any(e.get('bewertungsziel') == 'minimax' for e in (e0, e1)):
                                accepted = minimaxtor(e0, e1)['nicht_schlechter']
                            accepted = accepted and w1['zuege'] <= w0['zuege']


                            accepted = accepted and not knapp()
                            if accepted:
                                VD_NACH_LOG['uebernommen'] += 1
                                VD_NACH_LOG['zuege'].append(dict(netz=u, integral=[vd0['ueberlast_integral_mm2'], vd1['ueberlast_integral_mm2']]))
                                vd0, e0, w0 = vd1, e1, w1
            except Exception as exc:
                VD_NACH_LOG.setdefault('fehler', []).append(repr(exc)[:160])
            finally:
                if not accepted:
                    if u in U: del_unit(u)
                    add_unit(u, old['net'], old['tracks'], old['vias'])
                    fail.clear(); fail.update(saved_fail)
                    UNERREICHBAR.clear(); UNERREICHBAR.update(saved_unreachable)
                _KIDX[0] = None
    finally:
        VIA_DICHTE = kosten; PF[0] = pf
        for u in reihe: U[u] = U.pop(u)
        _KIDX[0] = None
        VD_NACH_LOG['cpu_s'] = round(time.process_time()-start, 3)
    print('Via-Dichte-Nachlauf:', VD_NACH_LOG, flush=True)


def strom_widerstand_alt(r, u):
    now = U[u]; U[u] = r
    try: return strom_widerstand(u)
    finally: U[u] = now

t1 = time.time()
PRE = {u: copy.deepcopy(r) for u, r in U.items()}
BREIT_DICHT_BERICHT = None
if BREIT_DICHT:
    _KIDX[0] = None
    _bd_vor = exact_conflicts()
    _bd_vor_n = len(PAIRS)

if 'pre' in args:
    _t, _v = [], []
    for u, r in U.items():
        for t in r['tracks']: _t.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w']))
        for v in r['vias']: _v.append(dict(net=v.get('net') or r['net'], x=v['x'], y=v['y']))
    if args.get('kurzschluss_schutz', '0') == '1': _t, _v = ausgabe_geometrie()
    json.dump(ausgabe_daten(_t, _v, {}) if args.get('kurzschluss_schutz', '0') == '1' else dict(tracks=_t, vias=_v), open(args['pre'], 'w')); print('vor Verbreitern gespeichert:', args['pre'])
RHO = 1.72e-5
def t_cu(L): return 0.035 if L in ('F', 'B') else 0.0175
def chan_R(c):
    R = 0.0
    for u, r in U.items():
        k = r['net'].split('/')[-1]
        if k in ('K_' + c, 'S_' + c, 'M_' + c):
            for t in r['tracks']:
                if t.get('sig'): continue
                for a, b2, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): R += RHO * math.dist(a, b2) / (w * t_cu(t['layer']))
            R += 0.0024 * len(r['vias'])
    return R
CH = sorted({r['net'].split('/')[-1][2:] for r in U.values() if r['net'].split('/')[-1][:2] in ('K_', 'S_', 'M_')})
WMAX = {0: 0.8, 1: 0.35, 2: 1.5, 3: 1.2, 4: 1.0, 5: 0.5, 6: 0.35} if args.get('breiter') == '1' else {}
def ucls(u):
    if u.startswith('GND#'): return dict(w=max(0.35, BAHN_MIN), wmin=max(0.15, BAHN_MIN))
    if u in PLANE_GROUPS:return dict(w=max(0.35,BAHN_MIN),wmin=max(0.15,BAHN_MIN),prio=6.5)
    C = cls(u[4:] if u.startswith('FIX#') else u)
    return dict(C, w=max(C['w'], WMAX.get(C['prio'], 0)))
if args.get('fair') in ('1', 'alle'):

    STEP = 0.05
    pw = [u for u in U if not u.startswith(('GND#', 'FIX#', 'HEFT#')) and u not in FIXGND and u not in PLANE_GROUPS and
          ((KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw') if KL and (args.get('fair_all') == '1' or args.get('fair') == 'alle') else ucls(u)['prio'] < 8)]
    satt = set()
    for rnd in range(60):
        Rc = {c: chan_R(c) for c in CH}
        Rn = {u: strom_widerstand(u) for u in pw} if KL and (args.get('fair_all') == '1' or args.get('fair') == 'alle') else None
        def key(u):
            if Rn is not None: return (0, -Rn[u], ucls(u)['prio'])
            k = U[u]['net'].split('/')[-1]
            return (0, -Rc[k[2:]]) if k[:2] in ('K_', 'S_', 'M_') else (1, ucls(u)['prio'])
        ch = 0
        if zeit_knapp(0.95): break
        for u in sorted(pw, key=key):
            if u in satt:
                U[u] = U.pop(u); continue
            C = ucls(u); before = [list(t['w']) for t in U[u]['tracks']]
            widen_unit(u, dict(C, w_step=STEP))
            if [list(t['w']) for t in U[u]['tracks']] != before: ch += 1
            else: satt.add(u)
        if not ch: break
    print('Fair verbreitert in %d Runden' % (rnd + 1), {c: round(chan_R(c) * 1000, 1) for c in CH}, 'mOhm', flush=True)
    rest = [u for u in U if u not in pw and not u.startswith(('FIX#', 'HEFT#')) and u not in FIXGND]
else:
    rest = [u for u in U if not u.startswith(('FIX#', 'HEFT#')) and u not in FIXGND]
for u in sorted(rest, key=lambda u: (ucls(u)['prio'] if not u.startswith('GND#') else 6.5)):
    if zeit_knapp(0.95): break
    widen_unit(u, ucls(u))
print('Verbreitert in %.0fs' % (time.time() - t1))

for rnd in range(0 if BREIT_DICHT else 6):
    cf = exact_conflicts()
    if not cf: break
    print('   nach Verbreitern Konflikte', len(PAIRS), sorted(PAIRS, key=lambda x: x[1])[:4], flush=True)
    _zuletzt = {}
    for _pi, (lay, dist, ua, ub, x, y) in enumerate(PAIRS):
        if lay == '*': continue
        for _ui, u in enumerate((ua, ub)):
            if paar_einheit(u) or u not in U or u not in PRE or u.startswith(('FIX#', 'HEFT#')): continue
            r = U[u]; pre = PRE[u]; rad = 0.6 + 0.4 * rnd
            if KL and args.get('strom_fix') == '1':
                from gemeinsam.strombahn import korrigiere
                li = LI[lay]
                hind = [(g, DRC_CLR) for other, rr in U.items() if rr['net'] != r['net'] and not _fixpaar(u, other)
                        for l2, g, _, _ in _unit_geo(other, rr) if l2 == li]
                hind += [(g, DRC_PADC) for pn, g, _ in _PADB[li] if pn != r['net']]
                if korrigiere(r['tracks'], pre['tracks'], lay, hind): _zuletzt[u] = (_pi, _ui)
                continue
            chg = False
            for t, t0 in zip(r['tracks'], pre['tracks']):
                if t['layer'] != lay: continue
                ws = list(t['w'])
                for k, (a, b2) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if _LS([a, b2]).distance(_PT(x, y)) < rad and ws[k] > t0['w'][k]:
                        ws[k] = t0['w'][k] if rnd < 4 else min(t0['w'][k], ws[k])
                        chg = True
                t['w'] = ws
            if chg or rnd >= 4: _zuletzt[u] = (_pi, _ui)
    for u in sorted(_zuletzt, key=_zuletzt.get):
        r = U[u]
        tr = r['tracks'] if rnd < 5 or (KL and args.get('strom_fix') == '1') else PRE[u]['tracks']
        del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    if BGA_PLAN and not _zuletzt:


        break
if BREIT_DICHT:
    _bd_nach = exact_conflicts()
    _bd_unter = []
    for u, r in U.items():
        if u.startswith(('FIX#', 'HEFT#', 'GND#')): continue
        if KL is None or (KL['netze'].get(r['net']) or {}).get('art') not in ('strom', 'versorgung', 'sw'): continue
        for ti, t in enumerate(r['tracks']):
            if t.get('sig'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                soll = FLUSS_W.get(u, {}).get((ti, si), ucls(u)['w'])
                if not strom_modell(r['net']): soll *= f_innen(r['net'], LI[t['layer']])
                if w + 1e-7 < soll:
                    _bd_unter.append(dict(netz=r['net'], layer=t['layer'], pts=[a,b], breite=w,
                                          soll=soll, laenge=math.dist(a,b)))
    BREIT_DICHT_BERICHT = dict(phase='direkt_nach_verbreitern',
        konfliktpaare_vor=_bd_vor_n, konfliktpaare_nach=len(PAIRS),
        konflikteinheiten_vor=len(_bd_vor), konflikteinheiten_nach=len(_bd_nach),
        unter_zielbreite=_bd_unter)
    print('Dicht verbreitert: Paare %d -> %d, Einheiten %d -> %d; %d Stromstuecke unter Zielbreite' % (
        _bd_vor_n, len(PAIRS), len(_bd_vor), len(_bd_nach), len(_bd_unter)), flush=True)
if R32:

    for u in list(U):
        r = U[u]
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#')) or r['net'] in IMP_TAB or (KL['netze'].get(r['net']) or {}).get('art') not in ('signal', 'schnell', 'analog'): continue
        widths = [w for t in r['tracks'] for w in t['w']]
        ziel = max(BAHN_MIN, min(widths)) if widths else BAHN_MIN
        if widths and any(abs(w - ziel) > 1e-9 for w in widths):

            tr = [dict(t, w=[ziel] * len(t['w'])) for t in r['tracks']]
            del_unit(u); add_unit(u, r['net'], tr, r['vias'])

if RUHIG:
    _bw = [0, 0, 0]
    for u in list(U):
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#')): U[u] = U.pop(u); continue
        _bw = [a + b for a, b in zip(_bw, breite_beruhigen(u, ucls(u)))]
    print('Breitenprofil beruhigt (ruhig_r=%g): %d Bahnen schmaler, Breitenwechsel %d -> %d' % (RUHIG_R, _bw[2], _bw[0], _bw[1]), flush=True)

if PADMITTE:
    _ns = stoesse_reparieren() if args.get('stoss_fix', '1') == '1' else 0
    if _ns: print('Stoesse (R24): %d Bahnenden am Schluss exakt auf die Mittellinie der eigenen Bahn gesetzt' % _ns, flush=True)
    _nk = kollinear_zusammen() if args.get('kollinear', '1') == '1' else 0
    print('Stoesse (R24): %d kollineare Stuetzpunkte zusammengefasst; beim Verlegen (alle Durchgaenge)' % _nk, STOSS_STAT, flush=True)


if args.get('via_feinschub') == '1' and KL and not fail and not exact_conflicts() and not zeit_knapp(): via_feinschub()
LAENGEN_BERICHT = None
if (args.get('laengen', '0') == '1' and SI_FELDER or PAAR_NETZE) and (PAAR_SUCHRAUM or not fail and not exact_conflicts()) and not zeit_knapp(0.95):
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if PAAR_NETZE: paar_koppeln()
    LAENGEN_BERICHT = laengen_abgleichen(nur_paare=args.get('laengen', '0') != '1')
    print('Laengenabgleich (Option): %d Maeander, %d Versuche' % (len(LAENGEN_BERICHT['zuege']), LAENGEN_BERICHT['versuche']), flush=True)

VD_NACH_LOG = dict(versuche=0, uebernommen=0, zuege=[], cpu_s=0.0)
if VD_NACH and not fail and not exact_conflicts() and not zeit_knapp(): via_dichte_nachlauf()

if STROM_ZUERST:


    for u,r in list(U.items()):
        if strom_modell(r['net']) and not u.startswith(('FIX#','HEFT#')) and not strom_abschluss(u,ucls(u)): fail.add(u)

REST_BERICHT = None
if args.get('restkonflikte', '0') == '1':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.restkonflikte import loese_vias
    REST_BERICHT = loese_vias(globals())
    print('R45: %d Via-Zuege angenommen, %.2f CPU-s' % (REST_BERICHT['uebernommen'], REST_BERICHT['cpu_s']), flush=True)


def pinzugang_regelwerte(net):
    ""
    from analyse.kopplung import kopplung, mengen
    from analyse.topologie import topologie
    tracks, vias = ausgabe_geometrie()
    own = [t for t in tracks if t['net'] == net]
    opfer, stoerer = mengen(KL)
    k = 0.
    if net in opfer or any(t.get('kelvin') for t in own):
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and t['net'] in stoerer])
        k += kopplung(r, KL, opfer={net}, stoerer=stoerer)['summe']
    if net in stoerer:
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and (t['net'] in opfer or t.get('kelvin'))])
        k += kopplung(r, KL, opfer=opfer - {net}, stoerer={net: stoerer[net]})['summe']
    r = dict(tracks=own, vias=[v for v in vias if v['net'] == net], clr=DRCC)
    t = topologie(r, dict(D, pads=[p for p in D['pads'] if p['net'] == net]))
    keys = ('parallel_rest_mm', 'stern', 'spitz', 'kelvin_nah_mm', 'abgriff_via_im_pad')
    return dict(kopplung=k, **{key:t[key] for key in keys},
                stern_frei=t['stern']-t['stern_pad'], spitz_frei=t['spitz']-t['spitz_pad'])


def pinzugang_regelannahme(vor, nach):
    ""
    return all(math.isfinite(nach[k]) and nach[k] <= v + 1e-6 for k,v in vor.items())

def pinzugang_nachlauf():
    ""
    global _LOOP_DEADLINE
    report = dict(reparatur=[], versuche=[], uebernommen=0, cpu_s=0., budget_s=0.)
    cpu_start = time.process_time()
    start = budget_uhr()
    budget = max(0., start / 9.)
    if BUDGET_ARBEIT: budget = min(budget, max(0., .90 * ARBEIT_MAX - arbeit_gesamt()))
    elif CPU_MAX > 0: budget = min(budget, max(0., .90 * CPU_MAX - such_cpu()))
    report['budget_arbeit' if BUDGET_ARBEIT else 'budget_s'] = budget

    if budget < .1: return report
    old_deadline = _LOOP_DEADLINE
    deadline = start + budget
    old_hist, old_pf, old_hart = hist.copy(), PF[0], HART_FREMD[0]
    old_active = _PZ_BUDGET_ACTIVE[0]
    try:
        hist[:] = 0; PF[0] = 1e3; HART_FREMD[0] = True
        _PZ_BUDGET_ACTIVE[0] = True; _LOOP_DEADLINE = deadline

        removed = []
        if PZ_REST:
            _loop_cpu_check()
            conflicts = exact_conflicts()
            tr, vi = ausgabe_geometrie()
            safe = r44_ausgabe(dict(tracks=tr, vias=vi, failed=sorted(fail), conflicts=conflicts))
            removed = (safe.get('kurzschluss_schutz') or {}).get('einheiten_entfernt', [])


            removed = [u for u in removed if u in U and u not in FIXED and not u.startswith(('FIX#','HEFT#'))]
            for u in removed:
                del_unit(u); fail.add(u)
            _KIDX[0] = None


            order = sorted(removed,key=lambda u:(-len(bynet.get(u,[])),u))
            if order:
                order = order[:1] + sorted(order[1:],key=lambda u:(u.startswith('GND#'),len(bynet.get(u,[])),cls(U[u]['net'] if u in U else u).get('prio',10),u))
            repair_end = start + budget * .55
            for index, u in enumerate(order):
                if budget_uhr() >= repair_end: break
                _LOOP_DEADLINE = min(repair_end,budget_uhr()+budget*(.20 if index==0 else .07))
                entry = dict(einheit=u, angenommen=False)
                local_before = {n:set(ps) for n,ps in PZ_LOKAL.items()}
                _PZ_PARTIAL[0] = None
                regeln0 = None
                try:
                    net = D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
                    regeln0 = pinzugang_regelwerte(net)
                    ok = route_unit(u); candidate = U.get(u)
                    valid = bool(candidate and (candidate['tracks'] or candidate['vias']))
                    valid = valid and _pz_geometry_ok(candidate['net'], candidate['tracks'], candidate['vias'], False, u)
                    _KIDX[0] = None
                    valid = valid and einheit_konfliktfrei(u)
                    if valid:
                        regeln1 = pinzugang_regelwerte(net)
                        entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                        valid = pinzugang_regelannahme(regeln0, regeln1)
                        if not valid: entry['grund'] = 'Topologie/Kopplung schlechter'
                    _loop_cpu_check()
                    if valid:
                        entry.update(angenommen=True, vollstaendig=bool(ok))
                        if ok: fail.discard(u)
                except (TimeoutError, _PZBudget):
                    entry['grund'] = 'CPU-Budget'


                    partial = _PZ_PARTIAL[0]
                    _LOOP_DEADLINE = repair_end
                    try:
                        _loop_cpu_check()
                        if partial and partial[0] == u and partial[2]:
                            add_unit(u,partial[1],partial[2],partial[3])
                            candidate = U[u]; _KIDX[0] = None
                            valid = _pz_geometry_ok(candidate['net'],candidate['tracks'],candidate['vias'],False,u)
                            valid = valid and einheit_konfliktfrei(u)
                            if valid and regeln0 is not None:
                                regeln1 = pinzugang_regelwerte(net)
                                entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                                valid = pinzugang_regelannahme(regeln0, regeln1)
                                if not valid: entry['grund'] = 'Topologie/Kopplung schlechter'
                            else: valid = False
                            _loop_cpu_check()
                            if valid: entry.update(angenommen=True,vollstaendig=False,teilbaum=True)
                    except Exception as error:
                        entry['pruefung'] = repr(error)[:180]
                except Exception as error:
                    entry['grund'] = 'Kandidat verworfen: ' + repr(error)[:180]
                finally:
                    _PZ_PARTIAL[0] = None
                    if not entry['angenommen']:
                        del_unit(u)
                        PZ_LOKAL.clear(); PZ_LOKAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    report['reparatur'].append(entry); _KIDX[0] = None
                if budget_uhr() >= repair_end: break
        _LOOP_DEADLINE = deadline


        if PZ_MINI and (removed or PZ_LOKAL):
            eligible = [u for u in units if u in U and u not in FIXED and u not in fail
                        and not u.startswith(('GND#','FIX#','HEFT#')) and not paar_einheit(u)
                        and u not in PAAR_NETZE and U[u]['net'] in PZ_BREITEN and net_mst(U[u]['net']) > .000001]
            for u in sorted(eligible, key=lambda u: (-(ulen(u)-.6*n_sv(U[u]))/net_mst(U[u]['net']), u))[:5]:
                _loop_cpu_check()
                old = copy.deepcopy(U[u]); l0 = sum(math.dist(a,b) for t in old['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:]))
                entry = dict(netz=old['net'], vor_mm=l0, q_vor=l0/net_mst(old['net']), angenommen=False)
                local_before = {n:set(ps) for n,ps in PZ_LOKAL.items()}
                try:
                    regeln0 = pinzugang_regelwerte(old['net'])
                    del_unit(u)
                    ok = route_unit(u); candidate = U.get(u)
                    l1 = sum(math.dist(a,b) for t in candidate['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) if candidate else float('inf')
                    entry['nach_mm'] = l1 if math.isfinite(l1) else None
                    ok = ok and l1 < l0-.01 and n_sv(candidate) <= n_sv(old)
                    ok = ok and min((w for t in candidate['tracks'] for w in t['w']),default=BAHN_MIN) >= min((w for t in old['tracks'] for w in t['w']),default=BAHN_MIN)-1e-9
                    ok = ok and _pz_geometry_ok(old['net'],candidate['tracks'],candidate['vias'],False,u)
                    _KIDX[0] = None
                    ok = ok and einheit_konfliktfrei(u)
                    if ok:
                        from analyse.bewertung import unverbunden as _pz_unverbunden
                        b = dict(D,pads=[p for p in D['pads'] if p['net']==old['net']])
                        r = dict(tracks=[dict(t,net=old['net']) for t in candidate['tracks']],
                                 vias=[dict(v,net=v.get('net') or old['net'],d=VIA_D,drill=VIA_DR,layers=LAYERS) for v in candidate['vias']])
                        ok = not _pz_unverbunden(b,r,set())
                    if ok:
                        regeln1 = pinzugang_regelwerte(old['net'])
                        entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                        ok = pinzugang_regelannahme(regeln0, regeln1)
                        if not ok: entry['grund'] = 'Topologie/Kopplung schlechter'
                    _loop_cpu_check()
                    if ok: entry['angenommen']=True; report['uebernommen']+=1
                except (TimeoutError, _PZBudget):
                    entry['grund']='CPU-Budget'
                except Exception as error:
                    entry['grund']='Kandidat verworfen: ' + repr(error)[:180]
                finally:
                    if not entry['angenommen']:
                        del_unit(u); add_unit(u,old['net'],old['tracks'],old['vias'])
                        PZ_LOKAL.clear(); PZ_LOKAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    _KIDX[0]=None; report['versuche'].append(entry)
                if entry.get('grund')=='CPU-Budget': break
    except (TimeoutError, _PZBudget):
        report['abbruch']='CPU-Budget'
    finally:
        hist[:]=old_hist; PF[0]=old_pf; HART_FREMD[0]=old_hart
        _PZ_BUDGET_ACTIVE[0]=old_active; _LOOP_DEADLINE=old_deadline
        report['cpu_s']=time.process_time()-cpu_start
        if BUDGET_ARBEIT: report['arbeit'] = arbeit_gesamt()-start
        report['suchkern']='atomare Suchauftraege' if BUDGET_ARBEIT else 'unterbrechbarer Python-Kern'
    return report


PZ_MINIMAX = dict(versuche=[], uebernommen=0)
arbeit_marke('vor_pinzugang')
if PZ_REST or PZ_MINI:
    PZ_MINIMAX = pinzugang_nachlauf()
    print('Pinzugang Minimax:', PZ_MINIMAX, flush=True)


if PZ_LOKAL:
    for u,r in list(U.items()):
        if u in FIXED or u.startswith(('FIX#','HEFT#')) or not _pz_active(r['net']):continue
        if not _pz_geometry_ok(r['net'],r['tracks'],r['vias'],False,u):
            del_unit(u);add_unit(u,r['net'],[],[]);fail.add(u)

_cf_final = exact_conflicts()
if _ET and not _LERN_AN: _ET.beobachten(_cf_final, MAXIT + 1, 'nachlauf')
if _LERN_AN: verhandlung_messen(MAXIT + 1, _cf_final, 'abschluss')
conf = {u: len(v) for u, v in _cf_final.items()}
print('Ende: Konflikte', conf, 'unvollstaendig', sorted(fail))
json.dump(PAIRS, open(OUT.replace('.json', '_konflikte.json'), 'w'))
tracks, vias = ausgabe_geometrie()
if PAAR_SUCHRAUM: print('Gemeinsamer Paar-Suchraum:', _PAAR_SUCH_INFO, flush=True)
if ABG_AN or ABG_BAUM or ABZW: print('Topologie (R27, Summe ueber alle Durchgaenge):', TOPO_STAT, flush=True)

VGB = []
for u, r in U.items():
    for v in r['vias']:
        if v.get('n'):
            ist = 1 + sum(1 for w in r['vias'] if w.get('zu') == [v['x'], v['y']])
            VGB.append(dict(netz=r['net'], x=v['x'], y=v['y'], soll=v['n'], ist=ist))
RVB = rv_bericht()
PAB = paare_bericht(empfaenger=True) if PAAR_NETZE else []
if PAB: print('Differenzpaare (R29): %s' % ['%s/%s %s/%s mm, schief %s, Vias %s, gekoppelt %.0f %%' % (
    b['netze'][0].split('/')[-1], b['netze'][1].split('/')[-1], b['laenge'][0], b['laenge'][1], b['schief_mm'], b['vias'], 100 * b['gekoppelt']) for b in PAB], flush=True)
if RVB: print('Rueck-Vias (Q2): %d Lagenwechsel schneller Netze mit anderer Masse-Bezugsflaeche, %d mit Masse-Via in Reichweite (%d eigenes Rueck-Via), %d ohne %s; %s' % (
    RVB['wechsel'], RVB['mit_rueck'], RVB['eigenes_rueck_via'], len(RVB['ohne']), [(b['netz'], b['abstand']) for b in RVB['ohne'][:6]], RVB['stat']), flush=True)
if VGB: print('Via-Gruppen (K10): %d Lagenwechsel mit Strom, %d Zusatz-Vias, an %d Stellen zu wenig Platz' % (
    len(VGB), sum(1 for r in U.values() for v in r['vias'] if v.get('gr')), sum(1 for b in VGB if b['ist'] < b['soll'])), flush=True)
if KETTE and KETTEN:
    _kb = [t for r in U.values() for t in r['tracks'] if t.get('kette')]
    print('Versorgungsketten (R22): %d Bahnen als Glieder, %d Pads ohne Kette bzw. ohne Sperrzone (kein Weg)' % (len(_kb), len(_KZ_AUS)), sorted(_KZ_AUS), KETTE_STAT, flush=True)

UNTER = unter_wmin()
if UNTER:
    _uf = [e for e in UNTER if not e['erlaubt']]
    print('Unter Mindestbreite: %d Stuecke (%.1f mm) in %d Netzen%s; dazu %d im Hals oder dichten Feld erlaubt (unter_wmin in der Ausgabe)' % (
        len(_uf), sum(e['laenge'] for e in _uf), len({e['netz'] for e in _uf}),
        ', am schmalsten %.3f statt %.3f mm (%s)' % min((e['breite'] - e['soll'], e['breite'], e['soll'], e['netz']) for e in _uf)[1:] if _uf else '',
        len(UNTER) - len(_uf)), flush=True)
if UNERREICHBAR: print('Unerreichbar (harte Hindernisse, Pad hat keinen Ausgang):', {n: v for n, v in sorted(UNERREICHBAR.items())})

MPV = []
for _e in EPG:
    _ist = sum(1 for r in U.values() if r['net'] == _e['netz'] for v in r['vias'] if _e['poly'].contains(_PT(v['x'], v['y'])))
    _ges = False
    if any(_k in EPM for _k in _e['pads']):
        _ges = not any(via_hart(_e['netz'], EPM[_k]['zellen']) for _k in _e['pads'] if _k in EPM)
    MPV.append(dict(teil=_e['ref'], pad=_e['num'], netz=_e['netz'], flaeche=_e['flaeche'], felder=_e['felder'], soll=_e['soll'], ist=_ist,
                    **({'grund': 'gesperrt'} if _ges else {})))
if MPV: print('Massepad-Vias (R16): %d Massepads, Soll %d Vias, Ist %d, an %d Pads weniger als Soll %s' % (
    len(MPV), sum(m['soll'] for m in MPV), sum(m['ist'] for m in MPV), sum(1 for m in MPV if m['ist'] < m['soll']),
    ['%s:%s %d/%d%s' % (m['teil'], m['pad'], m['ist'], m['soll'], ' (gesperrt: kein erlaubter Platz im Pad)' if m.get('grund') else '') for m in MPV]), flush=True)


VAPB = []
if VAP:
    _zahl = dict(im_pad=0, drittel=0, daneben=0)
    for u, r in U.items():
        if not u.startswith('GND#') or int(u[4:]) in EPM or not r['vias']: continue
        _k = int(u[4:]); _c, _w = _vap_zellen(_k); _v = r['vias'][0]
        _vc = int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES); _j = np.flatnonzero(_c == _vc)
        if len(_j): _zahl['im_pad' if _w[_j[0]] == 0.0 else 'drittel'] += 1; continue
        _zahl['daneben'] += 1; _p = D['pads'][_k]
        _hart = via_hart(r['net'], _c)
        VAPB.append(dict(teil=_p['ref'], pad=_p['num'], netz=r['net'], abstand=round(math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']), 3),
                         grund='belegt' if _hart else 'gesperrt', wichtig=_k in VAP_WICHTIG))
    print('Via am Pad (R9): %d Stiche mit Via ab der Haelfte im Pad, %d ab einem Drittel, %d daneben (davon %d gesperrt, %d belegt) %s' % (
        _zahl['im_pad'], _zahl['drittel'], _zahl['daneben'], sum(1 for b in VAPB if b['grund'] == 'gesperrt'),
        sum(1 for b in VAPB if b['grund'] == 'belegt'), ['%s:%s %s' % (b['teil'], b['pad'], b['grund']) for b in VAPB]), flush=True)
BGB = []
if BGA_FANOUT:
    for p in D['pads']:
        b = BGA.get(id(p))
        if b is None or not p['net'] or p['net'].startswith('unconnected'): continue

        near = [v for v in vias if v['net'] == p['net'] and math.dist((v['x'], v['y']), (p['x'], p['y'])) <= b['pitch'] * 0.72]
        BGB.append(dict(ref=b['ref'], pad=b['pad'], ring=b['ring'], pitch=b['pitch'],
                        via_nah=bool(near), via_im_pad=any(math.dist((v['x'], v['y']), (p['x'], p['y'])) < 0.02 for v in near)))


_rest_final = ausgabe_daten(tracks, vias, conf, roh=True,
    **({'paar_suchraum_info': _PAAR_SUCH_INFO} if PAAR_SUCHRAUM else {}),
    **({'abschluss': 'fruehabbruch' if _FRUEH_INFO is not None else ('cpu_budget' if zeit_knapp() else 'fertig'),
        'lernstand': json.loads(_LERN_SNAPSHOT)['lernstand'], 'lernbudget':json.loads(_LERN_SNAPSHOT)['lernbudget']} if _LERN_AN else {}))
if args.get('restkonflikte') == '1' and args.get('rest_teile', '1') == '1' and REST_BERICHT and KL and args.get('klassen'):
    import tempfile
    from nachbearbeitung.restkonflikte import teile_nachlauf, reparatur_sperre
    _rest_budget = max(0, float(args.get('rest_cpu', '30')) - REST_BERICHT['cpu_s'])
    if CPU_MAX > 0: _rest_budget = min(_rest_budget, max(0, CPU_MAX-such_cpu()))
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter_r45_final_') as _rest_tmp:
            _rp = Path(_rest_tmp) / 'routes.json'
            _bp = Path(_rest_tmp) / 'board.json'
            _rp.write_text(json.dumps(_rest_final)); _bp.write_text(Path(sys.argv[1]).read_text())
            _rest_block = reparatur_sperre(globals().get('ST') or {})
            if _rest_block:

                _rest_budget = 0
            _rp.with_name('routes_konflikte.json').write_text(json.dumps(PAIRS))
            _rest_teile = teile_nachlauf(str(_bp), args['klassen'], str(_rp), _rest_budget)
            if _rest_block:
                _rest_teile['grund'] = _rest_block
            _rest_final = json.loads(_rp.read_text()); _rest_final['rest_teile'] = _rest_teile
        print('R45 Teile: %d Zuege, %.2f CPU-s' % (_rest_teile['uebernommen'], _rest_teile['cpu_s']), flush=True)
    except Exception as _rest_error:
        _rest_final['rest_teile'] = dict(uebernommen=0, grund='Pruefung fehlgeschlagen: '+repr(_rest_error)[:180])
_sicher = rest_abschluss(_rest_final)
if args.get('skew', '0') == '1':
    _sicher = skew_abgleichen(D, _sicher, args, Path(OUT).with_suffix('.skew'))

arbeit_notbremse()
if ARBEIT_MESSEN:
    arbeit_marke('fertig')
    _sicher['arbeitsbudget'] = arbeit_beleg()
if MEISTER_BELEG is not None: _sicher['meister'] = MEISTER_BELEG
json.dump(_sicher, open(OUT, 'w'))
if _sicher.get('rest_board'):
    Path(OUT).with_name(Path(OUT).stem+'_rest_board.json').write_text(json.dumps(_sicher['rest_board']))
    Path(OUT).with_name(Path(OUT).stem+'_rest_placement.json').write_text(json.dumps(_sicher['rest_placement']))
print('Bahnen', len(tracks), 'Vias', len(vias), 'Zeit %.0fs' % (time.time() - t_start))
if _ET: _ET.schreiben(_sicher['tracks'], _sicher['vias'], fehlend=_sicher['failed'], zonen=_sicher.get('zonen', []) + _sicher.get('zones', []))
if _sicher.get('kurzschluss_schutz'):
    print('R44: Ausgabe %d Bahnen, %d Vias, offen %s' % (len(_sicher['tracks']), len(_sicher['vias']), _sicher['kurzschluss_schutz']['netze_entfernt']), flush=True)
if 'png' in args:
    _png_ausgabe = args.get('kurzschluss_schutz', '0') == '1' or args.get('skew', '0') == '1'
    for li, L in enumerate(LAYERS):
        img = np.zeros((NY, NX, 3), dtype=np.uint8); img[inside] = (235, 235, 230)
        if li in (0, NL - 1): img[PD1[li] <= 0.0] = (150, 150, 150)
        rng = np.random.RandomState(1); cols = rng.randint(40, 220, size=(len(nets) + 1, 3)); [cols.__setitem__(NID[_g], (60, 60, 60)) for _g in GNDS if _g in NID]
        if not _png_ausgabe:
            for u, r in U.items():
                c = r['cells'][(r['cells'] >= li * NN) & (r['cells'] < (li + 1) * NN)] - li * NN
                img.reshape(-1, 3)[c] = cols[NID[r['net']]]
        im = Image.fromarray(img).resize((NX * 2, NY * 2), Image.NEAREST)
        dr = ImageDraw.Draw(im)
        xy = lambda p: ((p[0]-X0)/RES*2, (p[1]-Y0)/RES*2)
        for t in (_sicher['tracks'] if _png_ausgabe else []):
            if t['layer'] != L: continue
            c = tuple(int(q) for q in cols[NID[t['net']]])
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']): dr.line([xy(a),xy(b)], fill=c, width=max(1,round(w/RES*2)))
        for v in (_sicher['vias'] if _png_ausgabe else []):
            x,y=xy([v['x'],v['y']]); r=VIA_R/RES*2
            dr.ellipse([x-r,y-r,x+r,y+r],fill=(40,40,40))
        for e in _sicher.get('luftlinien', []): dr.line([xy(p) for p in e['pts']],fill=(230,120,20),width=1)
        if not _png_ausgabe:
            for (lay, dist, ua, ub, x, y) in PAIRS_LAST:
                if lay not in (L, '*'): continue
                cx, cy = (x - X0) / RES * 2, (y - Y0) / RES * 2
                dr.ellipse([cx - 14, cy - 14, cx + 14, cy + 14], outline=(255, 0, 0), width=3)
        im.save(args['png'] + '_' + L + '.png')

if _sicher.get('rest_board') and 'png' in args and KL:
    from analyse.bewertung import bild
    bild(_sicher['rest_board'], _sicher, KL, args['png'])
if KNICK > 0 and NB is not None:
    print('Knick-Suche: %d Aufrufe, %d Entnahmen, %d dominiert, %.6f CPU-s' %
          (_KNICK_CALLS, _KNICK_POP, _KNICK_DOM, _KNICK_CPU), flush=True)
