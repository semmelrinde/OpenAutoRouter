class _Labels:
    def __init__(self):
        self.cpu = 0.0
        self.an = bool(args.get('labels'))
        self.rounds_n = []; self.conflict_list = []; self.signatures = {}; self.change = {}
        self.names = sorted(n for n in by_net if not n.startswith('#'))
        self.nid = {n: i for i, n in enumerate(self.names)}
        self.nx = (NX + 4) // 5; self.ny = (NY + 4) // 5
        if self.an:
            self.hmax = np.zeros((NL, self.ny, self.nx), np.float32)
            self.kmap = np.zeros_like(self.hmax, dtype=np.uint32)
            self.maps = []

    def coarse(self, a, medium=False):
        a = np.asarray(a).reshape(NL, NY, NX)
        p = np.pad(a, ((0, 0), (0, self.ny * 5 - NY), (0, self.nx * 5 - NX)))
        p = p.reshape(NL, self.ny, 5, self.nx, 5)
        return p.mean((2, 4)) if medium else p.max((2, 4))

    def grid_cells(self, c):
        c = np.asarray(c, dtype=np.int64)
        c = c[(c >= 0) & (c < NL * NN)]
        return (c // NN) * self.nx * self.ny + ((c % NN) // NX // 5) * self.nx + (c % NX) // 5

    def net_name(self, u):
        if u in U: return U[u]['net']
        if u.startswith('GND#'): return D['pads'][int(u[4:])]['net']
        return u[4:] if u.startswith('FIX#') else u

    def conflict_copy(self, conf):
        if not self.an: return None
        t = time.process_time()
        try: return {u: list(c) for u, c in conf.items()}
        except Exception as e:
            self.an = False; print('Labels: conflict copy skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def complete(self, conf, tight):
        if not self.an: return
        t = time.process_time()
        try:
            for u, c in tight.items(): conf.setdefault(u, []).extend(c)
        except Exception as e:
            self.an = False; print('Labels: conflict supplement skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def observe(self, conf, pass_, phase='negotiation', geometry=True):
        if not self.an or phase == 'initial_build': return
        t = time.process_time()
        try:
            counts = np.zeros(len(self.names), np.uint32); cells = []
            for u, c in conf.items():
                n = self.net_name(u)
                if n in self.nid: counts[self.nid[n]] += len(c)
                if len(c): cells.append(self.grid_cells(c))
            if cells:
                self.kmap.ravel()[np.unique(np.concatenate(cells))] += 1
            np.maximum(self.hmax, self.coarse(hist), out=self.hmax)
            self.rounds_n.append((pass_, phase)); self.conflict_list.append(counts)
            if geometry:
                by = {}
                for u in sorted(U):
                    r = U[u]; by.setdefault(r['net'], []).append((r['tracks'], r['vias']))
                for n, g in by.items():
                    sig = json.dumps(g, sort_keys=True)
                    if self.signatures.get(n) != sig:
                        self.signatures[n] = sig; self.change[n] = pass_
            if args.get('labels_rounds') == '1': self.maps.append(self.coarse(hist).astype(np.float32))
        except Exception as e:
            self.an = False; print('Labels: observation skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def output_assignment(self, tracks, vias, zone_list):
        from postprocessing.short_circuit_protection import output_assignment
        return output_assignment(dict(tracks=tracks, vias=vias, zone_list=list(zone_list), options=args),
                               LAYERS, np.array([X0,Y0]), (self.ny,self.nx))

    def write_out(self, tracks, vias, only_cache=False, absent=(), zone_list=()):
        global _LEARN_LABELS
        if not self.an: return
        t = time.process_time()
        try:
            import os, tempfile
            nr = len(self.names); a = np.zeros((nr, 7), np.float32)
            failed = {self.net_name(u) for u in set(fail) | set(absent)}
            need = {self.net_name(u) for u in units}
            if globals().get('_LEARN_ON'): failed |= {self.net_name(u) for u in set(units) - set(U) - set(FIXED)}
            present = {r['net'] for r in U.values()}
            _r45_label = (globals().get('_safe') or {}) if not only_cache else {}
            _r45_board = _r45_label.get('rest_board')
            if _r45_board: present = {q['net'] for q in tracks+vias}
            lm = np.zeros((nr, NL), bool); vmap = np.zeros_like(self.kmap)
            for n, i in self.nid.items():
                ps = ([p for p in _r45_board['pads'] if p['net'] == n] if _r45_board else [D['pads'][k] for k in by_net[n]])
                hp = max(p['x'] for p in ps) - min(p['x'] for p in ps) + max(p['y'] for p in ps) - min(p['y'] for p in ps)
                a[i, 0] = n not in failed and (n not in need or n in present)
                a[i, 2] = hp; a[i, 6] = self.change.get(n, 0)
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
            search_occ = self.coarse(use > 0, medium=True).astype(np.float16)
            protected = args.get('short_circuit_protection', '0') == '1'
            occ = self.output_assignment(tracks, vias, zone_list) if protected else search_occ
            data = dict(hist_context=np.asarray(args.get('hist_context', '')), label_version=np.asarray(2 if protected else 1, np.int16), observer_version=np.asarray(4 if protected else 3, np.int16), origin_mm=np.array([X0, Y0]), raster_mm=np.asarray(.25),
                        cu_layers=np.asarray(LAYERS), history_max=self.hmax, history_final=self.coarse(hist),
                        occupancy=occ, occupancy_search=search_occ, occupancy_contract=np.asarray('output_copper_cell_contact' if protected else 'search_occupancy_share'),
                        conflict_rounds=self.kmap, vias_raster=vmap,
                        net_list=np.asarray(self.names), net_columns=np.asarray(['routed', 'length_mm', 'hpwl_mm', 'detour_hpwl', 'vias', 'layers_n', 'last_change']),
                        net_values=a, net_layers=lm, net_conflicts=np.asarray(self.conflict_list, np.uint32).reshape(-1, nr),
                        passes=np.asarray([r[0] for r in self.rounds_n], np.int32), phases=np.asarray([r[1] for r in self.rounds_n]))
            if not protected:
                del data['occupancy_search']; del data['occupancy_contract']
            if _r45_board:
                data['r45_placement_json'] = np.asarray(json.dumps(_r45_label['rest_placement'], sort_keys=True))
                data['r45_target_pad_json'] = np.asarray(json.dumps(_r45_board['pads'], sort_keys=True))
            if self.maps: data['history_rounds'] = np.asarray(self.maps)
            if only_cache:
                _LEARN_LABELS = {k: v.copy() for k, v in data.items()}
                return
            dest = os.path.abspath(args['labels']); os.makedirs(os.path.dirname(dest), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
            try:
                np.savez_compressed(tmp, **data); os.replace(tmp, dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            print('Labels: %s (%d nets, %d observations, %.3f CPU-s)' % (dest, nr, len(self.rounds_n), self.cpu + time.process_time() - t), flush=True)
        except Exception as e: print('Labels: writing skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t


_ET = None



by_net = {}
for k, p in enumerate(D['pads']):
    if p['net'] and not p['net'].startswith('unconnected'):
        by_net.setdefault(p['net'], []).append(k)
def span(n):
    pads = [D['pads'][k] for k in by_net[n]]
    return max(p['x'] for p in pads) - min(p['x'] for p in pads) + max(p['y'] for p in pads) - min(p['y'] for p in pads)


def learn_series_value(n, value):
    if NC is not None and n in by_net:
        category=(NC['net_list'].get(n) or {}).get('category','signal')
        factor=float(args.get('cost_rank_'+category,'1'))
        if factor!=1.:return value-(factor-1.)*math.log1p(span(n))/5.
    return value


def learn_startspan(n, power=False):
    value=span(n)
    if NC is not None:
        category=(NC['net_list'].get(n) or {}).get('category','signal')
        factor=float(args.get('cost_rank_'+category,'1'))
        if factor!=1.:return value*(4*factor-3 if power else 3-2*factor)
    return value
NETTIES = set(NC.get('net_ties', [])) if NC else set()
def grp(n):
    if n in PLANE_GROUPS:return (4.8,0)
    if pair_fast(n):
        k = NC['net_list'][n]
        return (-2, -k.get('pair_rank', k.get('rank', 0)), min(n, PAIR_NETS[n]), n)
    if current_model(n): return (-1.5, -sum(map(float,current_model(n)['load'].values())), n)
    if n.startswith('GND#'):
        r_ = D['pads'][int(n[4:])]['ref']
        if r_.startswith('NT_') or r_ in NETTIES: return (-1, 0) if NC else (0, 0)
        if int(n[4:]) in VAP_IMPORTANT: return (4.8, 0)
        return (5, 0) if NC else (4, 0)
    if NC is not None and n in NC['net_list']:
        k = NC['net_list'][n]; g = k.get('group', 4)
        _rank_f = float(args.get('cost_rank_' + k.get('category', 'signal'), '1'))
        rank = k.get('rank', 0) * float(args.get('cost_rank_' + k.get('category', 'signal'), '1'))
        if g == 2: return (2 - .1 * (_rank_f-1), -learn_startspan(n,power=True))
        if ORDER >= 2:
            if g == 1 and k.get('rank', 0) < FAST_FROM: g = 3.5
            elif g == 4 and k.get('rank', 0) <= 0: g = 4.5
        return (g - .1 * (_rank_f-1), -rank, learn_startspan(n))
    pr = cls(n)['prio']
    if pr <= 1: return (0, span(n))
    if pr in (2, 3, 5): return (1, -span(n))
    if pr < 8: return (2, span(n))
    return (3, span(n))
units = [n for n in by_net if n not in GNDS and n not in AREA_NETS and len(by_net[n]) > 1 and not n.startswith('#')]
if args.get('only') == 'power':
    units = [n for n in units if cls(n)['prio'] < 8]
esp61 = 0
WITHOUT = {}
for spec in args.get('gnd_without_via', '').split(';'):
    if ':' in spec: r_, nums = spec.split(':'); WITHOUT[r_] = set(nums.split(','))
for _g in sorted(AREA_NETS):
    for k in by_net.get(_g, []):
        if D['pads'][k]['num'] in WITHOUT.get(D['pads'][k]['ref'], ()): continue
        if any(L in GND_LAYERS for L in D['pads'][k]['layers']): continue
        units.append('GND#%d' % k)

units += [g for g in sorted(GNDS - AREA_NETS) if len(by_net.get(g, [])) > 1]
PLANE_GROUPS = {}
if PLANE_FEEDERS:
    for _n in sorted(PLANE_MIXED_NETS):
        _far=[k for k in by_net.get(_n,[]) if PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y']))>6.]
        if not _far:continue
        PLANE_GROUPS[_n]=sorted(_far,key=lambda k:(PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y'])),k))
        _remove={'GND#%d'%k for k in _far};units=[u for u in units if u not in _remove];units.append(_n)
FIXED_GND = {'FIX#' + g for g in AREA_NETS}
FIXED = []
if 'fix' in args:
    FX = json.load(open(args['fix']))
    if PIN_ACCESS and any(abs(float(v.get('d',VIA_D))-VIA_D)>1e-9 or abs(float(v.get('drill',VIA_DR))-VIA_DR)>1e-9 for v in FX['vias']):
        raise ValueError('pin_access: fix= with mixed via ground masses not supported in the global router model')
    byn = {}
    for t in FX['tracks']: byn.setdefault(t['net'], ([], []))[0].append(dict(layer=t['layer'], pts=t['pts'], w=t['w']))
    for v in FX['vias']: byn.setdefault(v['net'], ([], []))[1].append(dict(x=v['x'], y=v['y']))
    for n, (tr, vs) in byn.items():

        add_unit('FIX#' + n, n, tr, vs); FIXED.append('FIX#' + n)
    fixnets = set(byn)
    if args.get('fixed_hard', '1') == '1':
        fo = np.zeros(NL * NN, dtype=bool); fdr = np.zeros(NN, dtype=bool)
        for u in FIXED:
            fo[U[u]['cells']] = True; fdr[U[u]['dcells']] = True
        FIXD[0] = [(_edt(~fo[li * NN:(li + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for li in range(NL)]
        FIXD[1] = (_edt(~fdr.reshape(NY, NX)) * RES).astype(np.float32)
        FIXNETS.update(fixnets); _HM.clear(); _ERR.clear(); _FIX_MAPS.clear()
        print('fixed copper as hard obstacles')


    def fix_pad_joined(k):
        p = D['pads'][k]; net = p['net']; tr, vs = byn.get(net, ([], []))
        for L in p['layers']:
            g = shapely.union_all([Polygon(q) for q in p['polys'].get(L, [])])
            copper_map = [g] + [_LS((a, b)).buffer(w / 2) for t in tr if t['layer'] == L for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            seen = {0}; open_items = [0]
            while open_items:
                j = open_items.pop()
                if any(copper_map[j].distance(Point(v['x'], v['y'])) <= VIA_R + 1e-7 for v in vs): return True
                for q, h in enumerate(copper_map):
                    if q not in seen and copper_map[j].distance(h) <= 1e-7: seen.add(q); open_items.append(q)
        return False
    units = [u for u in units if (not fix_pad_joined(int(u[4:])) if u.startswith('GND#') else u not in fixnets)]
    print('pinned:', len(FIXED), 'Nets from', args['fix'])
units.sort(key=grp)
if BGA_FANOUT:

    def bga_ring(u):
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        return min((BGA[id(D['pads'][k])]['ring'] for k in ks if id(D['pads'][k]) in BGA), default=-1)
    units.sort(key=lambda u: (grp(u)[:2], bga_ring(u), grp(u)[2:], u))
if PAIR_NETS:
    _positions = {u:i for i,u in enumerate(units)}
    units.sort(key=lambda u: (grp(u)[0], -(NC['net_list'].get(u) or {}).get('pair_rank',0),
                             min(u,PAIR_NETS[u]) if u in PAIR_NETS else '', u if u in PAIR_NETS else _positions[u]))
print('Units', len(units), '(Nets', sum(1 for u in units if not u.startswith('GND#')), ')')
if NC is not None and (ORDER >= 2 or FAST_LAYER > 0 or FAST_INNER > 0 or FAST_VIA != 1 or WEIGHTED):
    _sn = [u for u in units if actual_fast(u)]; _lfs = sorted({cls(u).get('lf') for u in _sn if cls(u).get('lf')})
    print('Rules R1/R14/R21: order=%d, fast_from=%d (%d fast, %d medium-fast nets), fast_via=%g, weighted=%d; layers %s' % (
        ORDER, FAST_FROM, len(_sn), sum(1 for u in units if u in NC['net_list'] and NC['net_list'][u].get('category') == 'fast' and not actual_fast(u)),
        FAST_VIA, WEIGHTED, ', '.join('%s %s' % (L, {2: 'next to ground', 1: 'next to area', 0: 'without area'}[LAYER_BASELINE[i]]) for i, L in enumerate(LAYERS))) +
        ('; cost factor of fast nets: %s' % ' / '.join(', '.join('%s x%.2f' % (L, f) for L, f in zip(LAYERS, lf)) for lf in _lfs) if _lfs else '; layer choice without effect (all layers equally good or fast_layer=0)'), flush=True)
MAXIT = int(args.get('iter', '40')); HINC = float(args.get('hinc', '0.4'))
PFMAX = float(args['pfmax']) if 'pfmax' in args else None
if PFMAX is not None and (not math.isfinite(PFMAX) or PFMAX <= 0): raise ValueError('pfmax must be finite and > 0')
if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
t0 = time.time()
fail = set()



CPU_MAX = float(args.get('cpu_max', '0'))
if _LEARN_ON:
    args.setdefault('history', os.path.join(os.path.dirname(OUT), 'history.jsonl'))
_HISTORY = open(args['history'], 'w') if args.get('history') else None
_HISTORY_CPU = [time.process_time()]
_LEARN_ROWS = []
_LEARN_STOP = None
_LEARN_BASIS = CPU_MAX
_LEARN_MAX = float(args.get('learn_cpu_max', str(2 * CPU_MAX))) if args.get('learning_budget') == 'adaptive' else CPU_MAX
_LEARN_POLICY = None
_EARLY_MODEL = None
_EARLY_INFO = None
if _EARLY_ON:
    from learning import early_abort as _EARLY
    try:
        _EARLY.generator_board_verify(D)
        _EARLY_CONTEXT = _EARLY.context_verify(
            json.loads(args['early_abort_context']) if args.get('early_abort_context')
            else _EARLY.context_board(D, NL))
        _EARLY_MODEL = _EARLY.model_load(_EARLY.model_path(args))
    except (OSError, ValueError, KeyError, TypeError) as _e:
        print('Early abort blocked: model/context error %s' % repr(_e)[:180], flush=True)
if _LEARN_ON and args.get('learning_policy'):
    _LEARN_POLICY = json.load(open(args['learning_policy']))
    if _LEARN_POLICY.get('version') != 1 or _LEARN_POLICY.get('status') != 'experimental':
        raise ValueError('learning_policy: version=1, status=experimental required')


def export_geometry():
    tracks = []; vias = []
    for u, r in U.items():
        if args.get('rest_repair') == '1':
            tracks.extend(dict(t,net=r['net'],**({'current_web': True} if t.get('web') else {})) for t in r['tracks'])
            vias.extend(dict(v,net=r['net'],**({'current_via': True} if v.get('gr') else {})) for v in r['vias'])
            continue
        for t in r['tracks']: tracks.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w'], sig=bool(t.get('sig')), **({'current_web': True} if t.get('web') else {}), **({'link_chain': True} if t.get('link_chain') else {})))
        for v in r['vias']: vias.append(dict(net=r['net'], x=v['x'], y=v['y'], current_via=True) if v.get('gr') else
                                        (dict(net=v['net'], x=v['x'], y=v['y'], ret_via=True) if v.get('rv') else dict(net=r['net'], x=v['x'], y=v['y'])))
    if args.get('short_circuit_protection', '0') == '1':
        for q, u in zip(tracks, [u for u,r in U.items() for t in r['tracks']]): q['unit'] = u if u.startswith('GND#') else q['net']
        for q, u in zip(vias, [u for u,r in U.items() for v in r['vias']]): q['unit'] = u if u.startswith('GND#') else q['net']
    for _t, _q in zip(tracks, [t for r in U.values() for t in r['tracks']]):
        if _q.get('tap'): _t['kelvin'] = True
    if globals().get('ZONE_PLAN'): vias += _ZP_VIAS
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


def export_contacts(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.short_circuit_protection import export_contacts as impl
    return impl(*pos, **kw)


def export_selection(*pos, **kw):
    from postprocessing.short_circuit_protection import export_selection as impl
    return impl(*pos, **kw)


def export_ratsnest(*pos, **kw):
    from postprocessing.short_circuit_protection import export_ratsnest as impl
    return impl(*pos, **kw)


def export_safe(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.short_circuit_protection import export_safe as impl
    return impl(*pos, **kw)



def export_data(tracks, vias, conf, absent=None, checkpoint=False, raw_data=False, **extra):
    data = dict(tracks=tracks, vias=vias, failed=sorted(fail if absent is None else absent),
        conflicts=conf, res=RES, clr=CLR, options=args,
        unreachable={n:v for n,v in sorted(UNREACHABLE.items())},
        area_nets=sorted(AREA_NETS), gnd_layers=GND_LAYERS, under_wmin=under_wmin())
    if WORK_MEASURE: data['work_budget'] = work_evidence()
    if KA_CONTRACT is not None:
        data['spacings'] = KA_CONTRACT
        data['spacing_legacy_covers'] = KA_LEGACY_COVERS
    if PIN_ACCESS:
        data['pin_access_exact']=dict(width_list=PZ_WIDTHS,minimax=globals().get('PZ_MINIMAX'),edges=PZ_EDGE_STAT,local={n:len(ps) for n,ps in sorted(PZ_LOCAL.items())},border_nm=PZ_BORDER_NM)
    if globals().get('WIDE_DENSE_REPORT') is not None: data['wide_dense'] = WIDE_DENSE_REPORT
    if globals().get('CURRENT_FIRST'):
        data['current_first'] = dict(net_list=CURRENT_AUDIT, potential_layers=CURRENT_POT, J=CURRENT_J,
            without_model=sorted(n for n,k in NC['net_list'].items() if k.get('category') in ('current_val','supply','sw') and not current_model(n)))
    if globals().get('ZONE_PLAN'): data.update(zone_plan_id=ZONE_PLAN['id'], zone_list=ZONE_PLAN['core_set'], native_vias=True)
    if PLANE_INFO: data['plane_domains'] = PLANE_INFO
    if PLANE_GROUPS:data['plane_groups']={n:['%s:%s'%(D['pads'][k]['ref'],D['pads'][k]['num']) for k in ks] for n,ks in sorted(PLANE_GROUPS.items())}
    if POTENTIAL_PLAN: data['potential_plan'] = POTENTIAL_PLAN
    if IMP_TAB: data['impedance_table'] = IMP_TAB
    if FIX_PAIRS: data['fix_spacings'] = FIX_PAIRS
    if args.get('fix'):
        data['fix_nets'] = sorted({r['net'] for u, r in U.items() if u.startswith('FIX#')})
    group_list = []
    for r in U.values():
        for v in r['vias']:
            if v.get('n'):
                group_list.append(dict(net_name=r['net'], x=v['x'], y=v['y'], nominal=v['n'],
                    actual=1 + sum(1 for w in r['vias'] if w.get('too') == [v['x'], v['y']])))
    if group_list: data['via_groups'] = group_list
    for key, name, enabled in (
        ('r19_short', 'R19_SHORT_STAT', globals().get('R19_SHORT')),
        ('r19_shape', 'R19_SHAPE_STAT', globals().get('R19_SHAPE')),
        ('length_matching', 'LENGTHS_REPORT', globals().get('LENGTHS_REPORT') is not None),
        ('ground_pad_vias', 'MPV', True), ('via_beside_pad', 'VAPB', True),
        ('ret_vias', 'RVB', True), ('pairs', 'PAB', True),
        ('vias_save', 'VS_LOG', args.get('vias_save') == '1'),
        ('via_fine_shift', 'FINE_SHIFT_LOG', args.get('via_fine_shift') == '1'),
        ('via_density_after', 'VD_AFTER_LOG', globals().get('VD_AFTER')),
        ('bga_fanout', 'BGB', globals().get('BGA_FANOUT'))):
        if enabled and name in globals() and (globals()[name] or key in ('length_matching', 'vias_save', 'via_fine_shift', 'bga_fanout')):
            data[key] = globals()[name]
    if globals().get('ST') and ST.get('rest_board'):
        if args.get('new'):
            from postprocessing.remaining_conflicts import placement_marker
            data.update(placement_marker(ST['rest_board'],copy.deepcopy(_REST_BOARD_SOURCE)))
        else:
            for _field in ('rest_board','rest_source','rest_moves','rest_placement','rest_native_sources'):
                if ST.get(_field) is not None: data[_field] = ST[_field]
    if args.get('remaining_conflicts') == '1' and globals().get('ST'):
        for _field in ('zones','zone_list','r41_fill_layers'):
            if ST.get(_field): data[_field] = ST[_field]
    if globals().get('REST_REPORT') is not None: data['remaining_conflicts'] = REST_REPORT
    if globals().get('_RL') is not None:
        data['routing_loop'] = _RL.summary()
        if _RL.board(D) != _RL.initial_board:
            data.update(_RL.rest.placement_marker(_RL.initial_board, _RL.board(D)))
    data.update(extra)
    if globals().get('_EARLY_INFO') is not None: data['early_abort'] = _EARLY_INFO




    if raw_data or args.get('short_circuit_protection', '0') != '1': return data
    return r44_export(data, checkpoint=checkpoint)
def r44_export(data, checkpoint=False, board=None, pairs=None, units_extra=None):
    board = D if board is None else board
    unit_list = {u:r['net'] for u,r in U.items()}
    for u, n in (units_extra or {}).items(): unit_list.setdefault(u, n)
    output_nets = {q['net'] for k in ('tracks', 'vias', 'zone_list', 'zones') for q in data.get(k, [])}
    important = {n: max(1, 20 - int(cls(n).get('prio', 10))) for n in output_nets}
    important.update({u:important.get(n, 1) for u,n in unit_list.items() if u.startswith('GND#')})
    result = export_safe(board, data, unit_list, important, list(PAIRS) if pairs is None else list(pairs), checkpoint=checkpoint)
    if checkpoint:
        result = dict(result)
        result['short_circuit_units'] = {u: (board['pads'][int(u[4:])]['net'] if u.startswith('GND#') else unit_list.get(u, u[4:] if u.startswith('FIX#') else u)) for u in result['failed']}
    return result


def rest_final(result):
    pairs = result.pop('rest_check_pairs', None)
    if args.get('short_circuit_protection', '0') != '1': return result
    if result.get('remaining_conflicts') is not None:
        result['remaining_conflicts']['before_r44'] = dict(conflict_list=len(result['conflicts']), open_items=len(result['failed']))
    board = result.get('rest_board') or D

    extra = {}
    for u in list(result['conflicts']) + [q for p in (pairs or ()) for q in p[2:4]]:
        if u in extra or u == 'PAD': continue
        if u.startswith('FIX#'): extra[u] = u[4:]
        elif u.startswith('GND#'): extra[u] = board['pads'][int(u[4:])]['net']
        else: extra[u] = u
    return r44_export(result, board=board, pairs=pairs, units_extra=extra)


_LEARN_CPU = 0.0
_LEARN_DEPTH = 0


def observer_cpu():
    return _LEARN_CPU + (_ET.cpu if _ET else 0.0)


def learn_observer(fn):
    def measure(*pos, **kw):
        global _LEARN_CPU, _LEARN_DEPTH
        if not _LEARN_ON and _HISTORY is None: return fn(*pos, **kw)
        outer = _LEARN_DEPTH == 0
        start = time.process_time(); mark = _ET.cpu if _ET else 0.0
        _LEARN_DEPTH += 1
        try: return fn(*pos, **kw)
        finally:
            _LEARN_DEPTH -= 1
            if outer:
                _LEARN_CPU += max(0.0, time.process_time() - start - ((_ET.cpu if _ET else 0.0) - mark))
    return measure


@learn_observer
def learn_snapshot(round_no, conf, phase):
    global _LEARN_SNAPSHOT


    tracks, vias = export_geometry()
    data = export_data(tracks, vias, {u: len(c) for u,c in conf.items()}, checkpoint=True,
        absent=set(fail) | (set(units) - set(U) - set(FIXED)),
        learning_state=dict(round_no=round_no, phase=phase, cpu_s=time.process_time(), pairs=list(PAIRS), censored=True),
        learning_budget=dict(basis_cpu_s=_LEARN_BASIS, current_cpu_s=CPU_MAX, max_cpu_s=_LEARN_MAX))


    import signal
    oldmask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGXCPU})
    try:
        if _ET:
            _ET.observe(conf, round_no, phase)
            _ET.write_out(data['tracks'], data['vias'], only_cache=True, absent=data['failed'], zone_list=data.get('zone_list', []) + data.get('zones', []))
        _LEARN_SNAPSHOT = json.dumps(data)
    finally: signal.pthread_sigmask(signal.SIG_SETMASK, oldmask)


def learn_decision():
    global CPU_MAX, _LEARN_STOP, _EARLY_INFO
    if not _LEARN_ON: return
    if globals().get('_EARLY_MODEL') is not None and args.get('learning_budget') != 'adaptive':
        _info = _EARLY.decision(_LEARN_ROWS, _EARLY_CONTEXT, CPU_MAX, _EARLY_MODEL)
        if _info:
            _EARLY_INFO = _info
            _LEARN_STOP = 'early_abort'
            print('Early abort: round %d, %d conflicts, score %.6f >= %.6f, %.1f CPU-s' %
                  (_info['round_no'], _info['conflict_list'], _info['score'], _info['threshold'], _info['cpu_s']), flush=True)
    if _LEARN_POLICY or args.get('learning_budget') == 'adaptive':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from learning.learning_effect import decision, promising
        if _LEARN_POLICY and decision(_LEARN_ROWS, len(D['pads']), _LEARN_POLICY):
            _LEARN_STOP = 'early_abort'
        elif CPU_MAX > 0 and time.process_time() >= .65 * CPU_MAX and CPU_MAX < _LEARN_MAX and promising(_LEARN_ROWS):
            CPU_MAX = min(_LEARN_MAX, max(CPU_MAX * 1.5, time.process_time() / .6))
            print('Learning budget: falling conflicts, extended to %.1f CPU-s' % CPU_MAX, flush=True)


@learn_observer
def negotiation_measure(round_no, conf, phase='negotiation'):
    if _HISTORY is None: return
    cpu = time.process_time()
    cells = np.unique(np.concatenate([np.asarray(c, dtype=np.int64) for c in conf.values()])) if conf else np.empty(0, dtype=np.int64)
    h = hist[cells]
    row = dict(round_no=round_no, phase=phase, cpu_s=cpu, cpu_round_s=cpu - _HISTORY_CPU[0], pf=float(PF[0]),
               conflict_list=len(conf), open_items=len(fail), pairs=list(PAIRS),
               hist_max=float(hist.max()), hist_conflict_max=float(h.max()) if len(h) else 0.0,
               hist_conflict_medium=float(h.mean()) if len(h) else 0.0,
               hist_conflict_null=float(np.mean(h == 0)) if len(h) else 0.0,
               net_list=sorted(conf), vias=sum(len(r['vias']) for r in U.values()))
    _HISTORY.write(json.dumps(row) + '\n'); _HISTORY.flush(); _HISTORY_CPU[0] = time.process_time()
    if _LEARN_ON:
        _LEARN_ROWS.append(row)
        learn_snapshot(round_no, conf, phase)
        if phase == 'negotiation': learn_decision()

if args.get('labels'):
    _mark_c0 = time.process_time()
    try:
        _ET = _Labels(); _ET.cpu += time.process_time() - _mark_c0
    except Exception as _e: print('Labels: initialization skipped:', repr(_e)[:180], flush=True)


@learn_observer
def initial_build_checkpoint():

    negotiation_measure(0, exact_conflicts(), 'initial_build')


def search_cpu():


    return time.process_time() + (getattr(globals().get('_AL'), 'cpu_offset', 0.))


def budget_message():
    if BUDGET_WORK:
        return 'Work budget (work_max=%d, consumed=%d jobs)' % (WORK_MAX, work_total())
    return 'Compute time budget (cpu_max=%.0f s)' % CPU_MAX

def time_tight(share=0.8):
    if BUDGET_WORK:

        work_emergency_stop()
        return work_total() >= share * WORK_MAX
    if _LEARN_ON and _LEARN_STOP: return True
    if _LEARN_ON and (args.get('learning_budget') == 'adaptive' or _LEARN_POLICY):
        return CPU_MAX > 0 and time.process_time() > share * CPU_MAX


    return CPU_MAX > 0 and (search_cpu() - observer_cpu()) > share * CPU_MAX


todo = None
if _LEARN_ON: learn_snapshot(0, {}, 'before_routing')
if _AGENT_SAVED is not None:
    pass
elif 'start' in args:

    ST = _REST_START_SOURCE if _REST_START_SOURCE is not None else json.load(open(args['start']))
    byn = {}
    for t in ST['tracks']: byn.setdefault(t['net'], ([], []))[0].append(t)
    for v in ST['vias']:
        _sv = dict(v) if args.get('rest_repair') == '1' else dict(x=v['x'], y=v['y'])
        if v.get('current_via'): _sv['gr'] = 1
        byn.setdefault(v['net'], ([], []))[1].append(_sv)


    NEW = {r for r in args.get('new', '').split(',') if r}
    new_nets = {D['pads'][k]['net'] for k in range(len(D['pads'])) if D['pads'][k]['ref'] in NEW} - AREA_NETS - {''}
    new_nets |= {n for n in args.get('new_nets', '').split(',') if n}
    new_nets = {n for n in new_nets if n in by_net and len(by_net[n]) > 1}
    zone = [(p['x'], p['y']) for p in D['pads'] if p['ref'] in NEW]
    gnd_new = [k for k, p in enumerate(D['pads']) if p['net'] in AREA_NETS and
               (p['ref'] in NEW or any(math.hypot(p['x'] - a, p['y'] - b_) < 1.5 for a, b_ in zone))]


    fail |= {n for n in (ST.get('failed') or []) if n in units and n not in new_nets} - {'GND#%d' % k for k in gnd_new}
    def in_pad(x, y, ks):
        for k in ks:
            bb = D['pads'][k]['bbox']
            if bb[0] - 0.05 <= x <= bb[2] + 0.05 and bb[1] - 0.05 <= y <= bb[3] + 0.05: return True
        return False
    for n, (tr, vs) in byn.items():
        if n in new_nets: continue
        if n in AREA_NETS:
            way_v = set()
            tr_keep = []
            for t in tr:
                ends = [tuple(t['pts'][0]), tuple(t['pts'][-1])]
                if any(in_pad(x, y, gnd_new) for x, y in ends):
                    for x, y in ends: way_v.add((round(x, 3), round(y, 3)))
                    continue
                tr_keep.append(t)
            vs_keep = [v for v in vs if (round(v['x'], 3), round(v['y'], 3)) not in way_v and not in_pad(v['x'], v['y'], gnd_new)]
            add_unit('FIX#' + n, n, [dict(t) if args.get('rest_repair') == '1' else dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr_keep], vs_keep); FIXED.append('FIX#' + n)
            continue
        C = cls(n); Cs = cls('/__signal__')


        _wk = lambda t: (Cs if t.get('sig') else C)['wmin']
        tr2 = [dict(t) if args.get('rest_repair') == '1' else dict(net=n, layer=t['layer'], pts=t['pts'], w=[(wi if args.get('rest_checking') == '1' else min(wi, _wk(t))) for wi in t['w']], sig=bool(t.get('sig')), **({'web': True} if t.get('current_web') else {})) for t in tr]
        add_unit(n, n, tr2, vs)
        for _t2, _t in zip(tr2, tr):
            if _t.get('kelvin'): _t2['tap'] = True
    if args.get('rest_check') == '1':
        _REST_LOCAL_CONFLICT = True
        _rc = exact_conflicts(); _rt, _rv = export_geometry()
        json.dump(export_data(_rt, _rv, {u: len(v) for u,v in _rc.items()}, raw_data=True, rest_check_pairs=list(PAIRS)), open(OUT, 'w'))
        sys.exit(0)
    if args.get('rest_repair') == '1':
        from postprocessing.remaining_conflicts import repair_export
        repair_export(globals())
        sys.exit(0)
    if NEW or new_nets:

        new_units = sorted(new_nets, key=lambda n: (grp(n), n)) + ['GND#%d' % k for k in gnd_new]
        for u in pair_series(new_units):
            if route_unit(u): fail.difference_update(pair_nets([u]))
            else:
                if pair_unit(u): fail.update((u,PAIR_NETS[u]))
                else: fail.add(u)
        if args.get('rest_part_local') == '1':


            _REST_LOCAL_CONFLICT = True
            _rc=exact_conflicts();_rt,_rv=export_geometry()
            json.dump(export_data(_rt,_rv,{u:len(v) for u,v in _rc.items()},raw_data=True,rest_check_pairs=list(PAIRS)),open(OUT,'w'))
            sys.exit(0)
        units = [u for u in units if not u.startswith('GND#')] + ['GND#%d' % k for k in gnd_new]
        args['all_items'] = '0'
        print('Start off', args['start'], ': rerouted', len(new_units), 'Units for', sorted(NEW), flush=True)
    else:
        units = [u for u in units if not u.startswith('GND#')]
        MAXIT = (MAXIT if args.get('loop') == '1' or args.get('agent_loop') == '1' else (2 if PAIR_SEARCH_SPACE else 1)); conf = exact_conflicts(); todo = (set(conf) | fail | ({u for u in units if pair_unit(u)} if PAIR_SEARCH_SPACE else set())) - set(FIXED)
        print('Start off', args['start'], ':', len(U), 'Units loaded', flush=True)
else:
    _learn_cp_cpu = time.process_time()
    _series = pair_series(units)
    for _i, u in enumerate(_series):
        if (_LEARN_ON and args.get('learning_budget') == 'adaptive' and CPU_MAX > 0
                and CPU_MAX < _LEARN_MAX and _i >= max(1, len(_series) // 2)
                and time.process_time() >= .65 * CPU_MAX):



            CPU_MAX = min(_LEARN_MAX, max(CPU_MAX, time.process_time() * len(_series) / _i / .65))
            print('Learning budget: initial build progress, %.1f CPU-s' % CPU_MAX, flush=True)
        if time_tight():
            rest = pair_nets(_series[_i:]); fail.update(rest); print('%s used up in the first pass: %d units not routed' % (budget_message(), len(rest)), flush=True); break
        if not route_unit(u):
            if pair_unit(u): fail.update((u,PAIR_NETS[u]))
            else: fail.add(u)
        if u in PAIR_NETS and u > PAIR_NETS[u] and u not in fail and PAIR_NETS[u] not in fail:
            pair_couple(only={u, PAIR_NETS[u]})
        if _LEARN_ON and time.process_time() - _learn_cp_cpu >= 10:
            initial_build_checkpoint()
            _learn_cp_cpu = time.process_time()
print('Iteration 1: %.0fs, incomplete %d' % (time.time() - t0, len(fail)))
class _RoutingLoop:
    VERSION = 'routing-loop-v4-local-cpu'

    def __init__(self, G):
        import hashlib
        from postprocessing import remaining_conflicts as rest
        from analysis.evaluation import rate
        self.G, self.rest, self.rate = G, rest, rate
        self.initial_board = self.board(G['D'])
        a = G['args']
        G['_LOOP_START_CPU'] = time.process_time()
        code=Path(G['__file__']).resolve();root=code.parents[1]
        self.source_list={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in
                      [('router',code),('netclasses_code',root/'analysis/netclasses.py')]+
                      [(name,Path(a[name])) for name in ('netclasses','design_rules') if a.get(name)]}
        self.source_list['board']=hashlib.sha256(json.dumps(self.initial_board,sort_keys=True).encode()).hexdigest()
        self.fraction = self.number(a, 'loop_share', .10, 0., .5)
        self.explicit_budget = 'loop_cpu' in a
        self.limit = self.number(a, 'loop_cpu', self.budget(G, self.fraction), 0., 86400.)
        self.move_cpu = self.number(a, 'loop_move_cpu', 5., .01, 300.)
        self.large = len(G['units']) >= 400
        self.threshold = int(self.number(a, 'loop_threshold', 80, 1, 1000000))
        if a.get('loop_maps','local') not in ('local','entire'):
            raise ValueError('loop_maps=local|entire')
        if a.get('loop_search','limited') not in ('limited','numba'):
            raise ValueError('loop_search=limited|numba')
        self.counts = []
        self.prefiltered = {}; self.rejected = {}
        self.k = int(self.number(a, 'loop_k', 4, 1, 100))
        self.n = int(self.number(a, 'loop_n', 4, 1, 100))
        self.cap = int(self.number(a, 'loop_cand', 24, 0, 256))
        self.radius = self.number(a, 'loop_radius', .8, .2, 3.)
        self.used = 0.; self.previous = None; self.stale = 0; self.cells = {}
        self.seen = set(); self.rounds = 0; self.accepted = {}; self.trials = 0
        self.accepted_moves = []
        self.disabled = None
        self.log = Path(a.get('loop_log', G['OUT'].replace('.json', '') + '_loop.jsonl'))
        self.exchange = Path(a.get('loop_state', G['OUT'].replace('.json', '') + '_bottlenecks'))
        if not G['NC']: self.disabled = 'loop needs netclasses='
        source = dict(G.get('ST') or {}, options=a)
        self.disabled = self.disabled or rest.repair_keepout(source)
        if any(source.get(k) for k in ('zones','zone_list','r41_fill_layers')):
            self.disabled = 'native areas need a coordinated copper state'
        if G.get('PAIR_NETS') or G.get('ZONE_PLAN'):
            self.disabled = 'Pair nets/zone plan need a coordinated group move'
        self.input_path = a.get('loop_moves')
        self.external = []
        if self.input_path:
            payload = json.loads(Path(self.input_path).read_text())
            if not isinstance(payload, list) or len(payload) > 256:
                raise ValueError('loop_moves: JSON list with at most 256 moves expected')
            self.external = [self.validate_move(m) for m in payload]
        self.emit(dict(event='start', standard_off=True, reason=self.disabled,
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
            raise ValueError('%s must be between %s and %s' % (key, lo, hi))
        return v

    @staticmethod
    def validate_move(m):
        if isinstance(m,dict) and m.get('category') in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import check
            return check(m)
        if not isinstance(m, dict) or m.get('category') not in ('via', 'part', 'layer_name', 'relay'):
            raise ValueError('Unknown loop move type')
        fields = dict(via={'category','unit','via','goal','cur_state'},
                      part={'category','ref','delta','cur_state'},
                      layer_name={'category','unit','track','segment','layer_name','cur_state'},
                      relay={'category','unit','cur_state'})
        if set(m) != fields[m['category']]: raise ValueError('Missing/unknown move fields')
        if not isinstance(m['cur_state'], str) or len(m['cur_state']) != 64 or any(x not in '0123456789abcdef' for x in m['cur_state']):
            raise ValueError('Move needs SHA-256 of the bottleneck state')
        if m['category'] == 'part':
            v = m['delta']
            if not isinstance(m['ref'],str) or not isinstance(v,list) or len(v)!=3:
                raise ValueError('Part move needs ref and delta=[dx,dy,rotation]')
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in v): raise ValueError('Invalid delta')
            if math.hypot(*v[:2]) > .500001 or v[2] not in (0,90,180,270): raise ValueError('Part move outside 0.5 mm/90 degrees')
        else:
            if not isinstance(m['unit'],str): raise ValueError('unit missing')
            for key in ('via','track','segment'):
                if key in m and (type(m[key]) is not int or m[key]<0): raise ValueError('Invalid index')
            if m['category']=='via' and (not isinstance(m['goal'],list) or len(m['goal'])!=2 or
                    any(type(x) not in (int,float) or not math.isfinite(x) for x in m['goal'])): raise ValueError('Invalid via target')
            if m['category']=='layer_name' and not isinstance(m['layer_name'],str): raise ValueError('Invalid layer')
        return copy.deepcopy(m)

    def emit(self, row):
        def finite(v):
            if isinstance(v,float) and not math.isfinite(v):return None
            if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
            if isinstance(v,(list,tuple)):return [finite(x) for x in v]
            return v
        row = dict(version=self.VERSION, source_list=self.source_list, **row)
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open('a') as f:
            f.write(json.dumps(finite(row), sort_keys=True, allow_nan=False) + '\n'); f.flush()

    def summary(self):
        return dict(version=self.VERSION, cpu_s=round(self.used,5), attempts=self.trials,
                    accepted=dict(self.accepted), reason=self.disabled,
                    declined=dict(self.rejected), prefiltered=dict(self.prefiltered),
                    budget_cpu_s=self.limit, move_budget_cpu_s=self.move_cpu,
                    logbook=str(self.log), bottlenecks=str(self.exchange),source_list=self.source_list)

    def out(self):
        G=self.G; c=G['exact_conflicts'](); ts,vs=G['export_geometry']()
        return G['export_data'](ts,vs,{u:len(v) for u,v in c.items()},raw_data=True), list(G['PAIRS'])

    def digest(self):
        import hashlib
        G=self.G
        io={'loop_moves','loop_log','loop_state','labels','png','history','netclasses','design_rules','start','fix','further','interim_state','agent_halt','agent_round','agent_packages','agent_moves'}
        state=dict(board=self.board(G['D']), unit_list={u:{k:r[k] for k in ('net','tracks','vias')} for u,r in sorted(G['U'].items())}, failed=sorted(G['fail']),
                   source_list=self.source_list,cu_layers=G['LAYERS'],res=G['RES'],pinned=sorted(G['FIXED']),options={k:v for k,v in G['args'].items() if k not in io})
        if G['args'].get('agent_loop') == '1':
            state['negotiation']=dict(hist_sha256=hashlib.sha256(G['hist'].tobytes()).hexdigest(),
                presence=G['PF'],units_order=list(G['U']),
                specs=G.get('_AGENT_SPECS',[]),priority=G.get('_AGENT_PRIORITY',[]))
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
            vias=[dict(unit=u,via=i,net_name=v.get('net') or r['net'],x=v['x'],y=v['y'],involved=u in us,
                       group_protection=any(v.get(k) for k in ('n','too','gr','rv','current_via','ret_via')))
                  for u,r in sorted(G['U'].items()) for i,v in enumerate(r.get('vias',[]))
                  if any(math.dist((v['x'],v['y']),q)<=self.radius for q in points)]
            history=[]
            for p in ps:
                ix=int((p[4]-G['X0'])/G['RES']);iy=int((p[5]-G['Y0'])/G['RES'])
                if 0<=ix<G['NX'] and 0<=iy<G['NY']:
                    history.append(float(G['hist'].reshape(G['NL'],G['NY'],G['NX'])[:,iy,ix].max()))
            result.append(dict(pairs=ps, points=points, unit_list=us, bbox=bounds,
                               net_list=sorted({G['U'][u]['net'] for u in us}), pads=pads,
                               components=sorted({G['D']['pads'][i]['ref'] for i in pads}),vias=vias,
                               history_log=max(history,default=0)))
        return sorted(result,key=lambda c:(-len(c['pairs']),-c['history_log'],c['bbox']))

    def movable(self,u,via=False):
        G=self.G
        if u not in G['U'] or u.startswith('TACK#'): return False
        if u in G['FIXED'] or u.startswith('FIX#'):

            return via and u.startswith('FIX#') and 'start' in G['args'] and not G['args'].get('fix') and G['U'][u]['net'] in G['AREA_NETS']
        return True

    def neighbours(self,c):
        G=self.G
        if 'neighbors' in c:
            return [u for u in c['neighbors'] if u in G['U'] and self.movable(u)]
        return [u for u,r in G['U'].items() if self.movable(u) and G['_close_on'](r,c['points'],self.radius)]

    def part_units(self,ref):
        G=self.G
        nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}-G['AREA_NETS']
        touched={u for u,r in G['U'].items() if r['net'] in nets and self.movable(u)}
        if G.get('_AL') is not None:
            touched.update(u for u in G['units'] if u in nets and u not in G['FIXED'])


        touched.update('GND#%d'%i for i,p in enumerate(G['D']['pads'])
                       if p['ref']==ref and p['net'] in G['AREA_NETS']
                       and 'GND#%d'%i in G['units'] and self.movable('GND#%d'%i))
        return touched

    def part_moves(self,c):
        import re
        from common import travel_range as WW
        G=self.G;B=G['D'];W=WW.table(G['NC']);fps={f['ref']:f for f in B['fps']}
        raw=B.get('placement_groups') or []
        groups=raw.values() if isinstance(raw,dict) else raw
        grouped={r for g in groups if isinstance(g,dict) for r in g.get('refs',[])}
        if not c['points']:return
        for ref in sorted(fps,key=lambda r:(min((math.dist((fps[r]['x'],fps[r]['y']),p) for p in c['points']),default=math.inf),r)):
            f=fps[ref];t=G['NC']['part_list'].get(ref,{})
            if (not re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp',''))
                    or ref in grouped or f.get('locked') or t.get('pinned') or t.get('role')!='small'
                    or ref not in W or W[ref]['max']<=0):continue
            nearest=min(c['points'],key=lambda p:math.dist((f['x'],f['y']),p))
            ax,ay=f['x']-nearest[0],f['y']-nearest[1]
            if math.hypot(ax,ay)>1.5:continue
            direction=(0,1 if ay>=0 else -1) if abs(ay)>=abs(ax) else (1 if ax>=0 else -1,0)
            preferred=[(round(direction[0]*d,5),round(direction[1]*d,5),0) for d in (.1,.2,.3,.4,.5)]
            rotations=[(0,0,th) for th in (90,180,270)] if W[ref]['rotate'] else []
            origin=(B.get('wander_basis') or {}).get(ref) or f
            if isinstance(origin,(list,tuple)):origin=dict(x=origin[0],y=origin[1])
            for dx,dy,th in preferred+rotations+[(x,y,0) for x,y in self.rest.offsets(.5) if (x,y,0) not in preferred]:
                if math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[ref]['max']+1e-8:continue
                yield dict(ref=ref,moves={ref:(dx,dy,th)})

    def external_local(self,m,c):
        G=self.G
        if m['category'] in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import local
            return local(G,m,c,self.radius)
        if m['category']=='part':
            f=next((f for f in G['D']['fps'] if f['ref']==m['ref']),None)
            return f is not None and any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['points'])
        r=G['U'].get(m['unit'])
        if r is None:return False
        try:
            if m['category']=='via':
                v=r['vias'][m['via']]
                return any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['points'])
            if m['category']=='layer_name':
                from shapely.geometry import LineString
                t=r['tracks'][m['track']];j=m['segment'];line=LineString(t['pts'][j:j+2])
                return any(line.distance(Point(p))<=self.radius for p in c['points'])
        except (IndexError,ValueError):return False
        return G['_close_on'](r,c['points'],self.radius)

    def part_board(self,m,c):
        import re
        from common import travel_range as WW
        from common.parts import load_parts,circuit_board,Occupancy
        from common.shift import relocate
        from postprocessing.move_rules import keepout
        G=self.G;B=self.board(G['D']);r=m['ref'];dx,dy,th=m['delta']
        fps={f['ref']:f for f in B['fps']};f=fps.get(r);t=G['NC']['part_list'].get(r,{})
        groups=B.get('placement_groups') or []
        if isinstance(groups,dict):groups=groups.values()
        grouped={ref for group in groups if isinstance(group,dict) for ref in group.get('refs',[])}
        W=WW.table(G['NC'])
        chip=bool(G.get('_AL') is not None and f and re.search(r'(?:^|[_:])(?:SOT|SOIC)[-_]',f.get('fp',''),re.I))
        small=bool(f and re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp','')))
        if f is None or not (small or chip) or r in grouped or f.get('locked') or t.get('pinned') or t.get('role') not in (('small','chip') if chip else ('small',)) or r not in W:
            raise ValueError('Part is not a free C/R or released SOT/SOIC')
        if chip and not any(q['ref']==r for q in B.get('courtyards',[])):raise ValueError('SOT/SOIC needs exported courtyard')
        if not any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['points']):raise ValueError('Part not local')
        if math.hypot(dx,dy)>.500001 or th not in (0,90,180,270) or (th and not (W[r]['rotate'] or chip)):raise ValueError('Part move outside movement contract')
        origin=(B.get('wander_basis') or {}).get(r) or next(q for q in self.initial_board['fps'] if q['ref']==r)
        if isinstance(origin,(tuple,list)):origin=dict(x=origin[0],y=origin[1])
        if W[r]['max']<=0 or math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[r]['max']+1e-8:raise ValueError('Travel distance exceeded')
        B1,_=relocate(B,{r:(dx,dy,th)});T=load_parts(B1,.15);fps={f['ref']:f for f in B1['fps']};bel=Occupancy()
        for ref in T:
            q=fps[ref]
            if ref!=r:bel.add(ref,T[ref].geo(q['x'],q['y'],0,q['side']))
        f=fps[r];geo=T[r].geo(f['x'],f['y'],0,f['side']);_,_,inside,_=circuit_board(B,.2)
        if bel.collision(geo) or not all(inside.contains(g) for g in geo.values()) or keepout(B1,geo,(r,)):raise ValueError('Courtyard/pad spacing/edge/keepout violated')
        return B1

    def via_move(self,owner,index,target):
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
        return self.rest.via_shift(G['D'],r,index,target,G['VIA_D'],G['VIP'],owner.startswith('GND#') and int(owner[4:]) in G['EPM'],G['cls']('/__signal__')['wmin'])

    def candidates(self,c,digest):
        G=self.G; limit=max(1,self.cap//4); count=0


        vv=[]
        for u,r in sorted(G['U'].items()):
            if not self.movable(u,True): continue
            for i,v in enumerate(r['vias']):
                d=min(math.dist((v['x'],v['y']),p) for p in c['points'])
                if d<=self.radius: vv.append((u not in c['unit_list'],d,u,i))
        for _,_,u,i in sorted(vv):
            v=G['U'][u]['vias'][i]; targets=[]
            for step in (.1,.2,.3,.4,.5):
                for dx,dy in ((0,step),(0,-step),(step,0),(-step,0),(step,step),(-step,step),(step,-step),(-step,-step)):
                    if math.hypot(dx,dy)<=.500001:
                        x,y=v['x']+dx,v['y']+dy
                        targets.append((round(x,5),round(y,5)))
                        x=G['X0']+round((x-G['X0'])/G['RES'])*G['RES']; y=G['Y0']+round((y-G['Y0'])/G['RES'])*G['RES']
                        if .099999<=math.dist((v['x'],v['y']),(x,y))<=.500001: targets.append((round(x,5),round(y,5)))
            targets=sorted(set(targets),key=lambda p:(max(0., G['VIA_D']/2 + G['DRC_CLR'] - min(math.dist(p,q) for q in c['points'])),math.dist(p,(v['x'],v['y'])),p))
            for target in targets[:limit]:
                yield dict(category='via',unit=u,via=i,goal=list(target),cur_state=digest)
                count+=1
                if count>=limit: break
            if count>=limit: break
        for i,p in enumerate(self.part_moves(c)):
            if i>=limit: break
            yield dict(category='part',ref=p['ref'],delta=list(p['moves'][p['ref']]),cur_state=digest)
        count=0
        for u in c['unit_list']:
            if not self.movable(u): continue
            for i,t in enumerate(G['U'][u]['tracks']):
                if any(t.get(k) for k in ('web','current_web','kelvin','tap','rv')): continue
                for j,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                    from shapely.geometry import LineString
                    if not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['points']): continue
                    for layer in G['LAYERS']:
                        if layer==t['layer']: continue
                        yield dict(category='layer_name',unit=u,track=i,segment=j,layer_name=layer,cur_state=digest)
                        count+=1
                        if count>=limit: break
                    if count>=limit: break
                if count>=limit: break
            if count>=limit: break
        neighbours=sorted((u for u in G['U'] if self.movable(u) and G['_close_on'](G['U'][u],c['points'],self.radius)),key=lambda u:(u in c['unit_list'],u))
        for u in neighbours[:limit]: yield dict(category='relay',unit=u,cur_state=digest)

    @staticmethod
    def pair_key(p): return (p[0],*sorted(p[2:4]),round(p[4],2),round(p[5],2))

    def pair_gate(self,p0,p1,c):
        from collections import Counter
        b=c['bbox']
        inside=lambda p:b[0]<=p[4]<=b[2] and b[1]<=p[5]<=b[3]
        before=[p for p in p0 if inside(p)]; after=[p for p in p1 if inside(p)]
        if len(after)>=len(before): return 'Pair count in cluster does not drop'
        old=Counter(self.pair_key(p) for p in p0 if not inside(p))
        new=Counter(self.pair_key(p) for p in p1 if not inside(p))
        if new-old: return 'new conflict outside the cluster'

        distances={};after_distances={}
        for p in p0:
            if not inside(p):distances.setdefault(self.pair_key(p),[]).append(p[1])
        for p in p1:
            if not inside(p):after_distances.setdefault(self.pair_key(p),[]).append(p[1])
        for key,ds in after_distances.items():
            if any(n<o-1e-6 for o,n in zip(sorted(distances[key]),sorted(ds))):return 'Conflict outside gets worse'
        return None

    def evaluate_gate(self,B0,R0,E0,R1,p0,p1,c):
        G=self.G; K=G['NC']; B1=G['D']
        reason=self.pair_gate(p0,p1,c)
        if reason: return reason,None
        if not set(R1['failed'])<=set(R0['failed']): return 'new open unit',None
        for check in (self.rest.hard_vias(B1,R0,R1),self.rest.hard_segments(B1,R0,R1,diagnose=True),self.rest.part_clearance(B0,B1,R1)):
            if not check['ok']: return check['gate_check'],None
        from collections import Counter
        old_vias=Counter((v['net'],v['x'],v['y']) for v in R0['vias'])
        for v in R1['vias']:
            key=(v['net'],v['x'],v['y'])
            if old_vias[key]:old_vias[key]-=1;continue
            ix=int((v['x']-G['X0'])/G['RES']);iy=int((v['y']-G['Y0'])/G['RES'])
            if not (0<=ix<G['NX'] and 0<=iy<G['NY']) or not G['via_hard'](v['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
                return 'Via violates static grid map (including VIP)',None
        E1=self.rate(B1,R1,K,v3=True,minimax=False)
        if E1['open_items']>E0['open_items'] or not set(E1['not_connected'])<=set(E0['not_connected']): return 'Connections worse',E1
        from analysis import rule_metrics as RK
        from postprocessing.move_rules import rule_values, TOPO
        rg=RK.assumption(E0['design_rules'],E1['design_rules'])
        if not rg['not_worse']: return rg['reason'],E1
        if not math.isfinite(E1['score_v3']) or E1['score_v3']>E0['score_v3']+1e-6: return 'Score v3 worse',E1
        a=rule_values(B0,R0,K);b=rule_values(B1,R1,K)
        if any(b['topology'][k]>a['topology'][k]+1e-6 for k in TOPO) or b['coupling']>a['coupling']+1e-6: return 'Topology/coupling worse',E1
        protected=set(K['gnd'])|{n for n,k in K['net_list'].items() if k.get('category') in ('current_val','supply','sw')}
        a=Counter(v['net'] for v in R0['vias'] if v['net'] in protected);b=Counter(v['net'] for v in R1['vias'])
        if a-b: return 'protected via count decreases',E1
        a=self.rest.ground_pad_numbers(B0,R0,K);b=self.rest.ground_pad_numbers(B1,R1,K)
        if any(b.get(p,0)<n for p,n in a.items()): return 'R16 ground pad via missing',E1


        from analysis.evaluation import current_paths,current_loss_equal
        s0=current_paths(B0,R0,K,E0,dc=None);s1=current_paths(B1,R1,K,E1,dc=None)
        p0w,p1w=current_loss_equal(E0,E1,B0,R0,B1,R1,K)
        if (s1.get('detour_mm') or 0)-(s0.get('detour_mm') or 0)>1. and p1w>p0w*1.02+1e-9: return 'Current paths worse',E1
        return None,E1

    def static_preflight(self,R0,R1):
        v=self.rest.hard_vias(self.G['D'],R0,R1)
        if not v['ok'] and v['gate_check']!='Via-DrillHole-DrillHole':return v['gate_check']
        s=self.rest.hard_segments(self.G['D'],R0,R1,diagnose=True)
        return None if s['ok'] else s['gate_check']

    def prefilter(self,m,c):
        G=self.G
        if m['category']=='part':
            projected=self.part_units(m['ref'])|set(self.neighbours(c))
            limit=min(int(G['args'].get('loop_group','24')),int(G['args'].get('loop_new_max','4' if self.large else '24')))
            return 'Part group already too large before placement check' if len(projected)>limit else None
        if m['category'] != 'via': return None
        u=m['unit']
        if not self.movable(u,True): return 'fixed/missing via'
        if m['via'] >= len(G['U'][u]['vias']): return 'missing via'
        x,y=m['goal'];v=G['U'][u]['vias'][m['via']]
        if not .099999<=math.hypot(x-v['x'],y-v['y'])<=.500001: return 'Via path outside contract'
        ix=int((x-G['X0'])/G['RES']);iy=int((y-G['Y0'])/G['RES'])
        if not (0<=ix<G['NX'] and 0<=iy<G['NY']): return 'Via target outside grid'

        if not G['via_hard'](G['U'][u]['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
            return 'Via target statically blocked'
        if self.via_move(u,m['via'],m['goal']) is None: return 'Via-Connection/GroupProtection'
        return None

    def rebuild(self,B):
        if self.G.get('_AL') is not None:
            changed=[i for i,(a,b) in enumerate(zip(self.board(self.G['D'])['pads'],B['pads'])) if a!=b]
            if len(changed)>2:return self.rebuild_full(B)
        if self.G['args'].get('loop_maps','local') != 'entire':
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
                if G['DRCL']:pg=G['pad_hull'](p,L,G['DRCP'],True)
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
        for key in ('_HM','_ERR','NECK_CACHE','_VAPZ','_NEARBY_C','_DIST_C'):
            if key in G and isinstance(G[key],dict):G[key]={}
        G['_pad_room_cached'].cache_clear()
        G['CHAINS']={};G['KF_AFTER']={};G['KF_BEFORE']={}
        if G['CHAIN']:G['_chains_build']()
        G['_CAND_IDX']=[None];G['ASTAR_COST']=[None]
        self.last_window=list(window)
        return original

    def rebuild_full(self,B):



        import tempfile,contextlib,io
        G=self.G; original=G.copy(); argv=list(sys.argv)
        code=Path(G['__file__']).read_text().split('# Density labels are a passive observer;',1)[0]


        code=code.replace('\nbootstrap(heavy=True)\n','\n# existing process guard is kept\n',1)
        with tempfile.TemporaryDirectory(prefix='routing_loop_') as td:
            payload=self.board(B)


            payload['pads']=payload['pads'][:original['_LOOP_SOURCE_PAD_COUNT']]
            path=Path(td)/'board.json';path.write_text(json.dumps(payload))
            opt={k:v for k,v in G['args'].items() if k not in ('start','new','new_nets','hist_start','hist_context','agent_loop','further','agent_moves')}
            try:
                sys.argv=[argv[0],str(path),original['OUT']]+[k+'='+str(v) for k,v in opt.items()]
                with contextlib.redirect_stdout(io.StringIO()): exec(compile(code,G['__file__'],'exec'),G)
                for u,r in original['U'].items(): G['add_unit'](u,r['net'],copy.deepcopy(r['tracks']),copy.deepcopy(r['vias']))

                for key in ('args','t_start','PF','VIA_PREV','CONS_SOFT','VIA_ANCHOR_ON','SOURCE_FREE','HARD_FOREIGN','_REST_BOARD_SOURCE','_REST_START_SOURCE'):
                    if key in original: G[key]=copy.deepcopy(original[key])
                G['hist'][:]=original['hist']; G['fail']=set(original['fail']); G['_CAND_IDX'][0]=None
                for key in original:
                    if key.startswith('_AGENT_'):G[key]=original[key]
                if G.get('_AL') is not None:G['_AL'].install_router()
            finally: sys.argv=argv
        return original
    def apply(self,m,c):
        G=self.G; category=m['category']; owner=m.get('unit'); touched=[]
        if category in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import apply
            return apply(self,m,c)
        if category=='part':
            marker=self.rest.placement_marker(self.initial_board,self.part_board(m,c))
            self.rebuild(marker['rest_board']); G=self.G
            ref=m['ref']; nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}
            touched=list(self.part_units(ref))


            for u,r in list(G['U'].items()):
                if not u.startswith('FIX#') or r['net'] not in nets: continue
                if G['_close_on'](r,c['points'],self.radius): raise ValueError('anonymous area stub needs a joint move')
        elif category=='via':
            if not self.movable(owner,True): raise ValueError('fixed/missing via')
            r=G['U'][owner];v=r['vias'][m['via']]
            if not .099999<=math.dist((v['x'],v['y']),m['goal'])<=.500001: raise ValueError('Via path outside 0.1 to 0.5 mm')
            if not any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['points']): raise ValueError('Via not local')
            r1=self.via_move(owner,m['via'],m['goal'])
            if r1 is None: raise ValueError('Via-Connection/GroupProtection')
            G['del_unit'](owner);G['add_unit'](owner,r1['net'],r1['tracks'],r1['vias'])
        elif category=='layer_name':
            if not self.movable(owner) or m['layer_name'] not in G['LAYERS']: raise ValueError('blocked unit/layer')
            r=copy.deepcopy(G['U'][owner]);i=m['track'];j=m['segment'];t=r['tracks'][i]
            if any(t.get(k) for k in ('web','current_web','kelvin','tap','rv')): raise ValueError('protected track')
            a,b=t['pts'][j:j+2];w=t['w'][j]
            from shapely.geometry import LineString
            if t['layer']==m['layer_name'] or not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['points']): raise ValueError('Layer change not local')
            if math.dist(a,b)<G['DRC_HOLE']: raise ValueError('Via pair too tight')
            replacement=[]
            if j: replacement.append(dict(t,pts=t['pts'][:j+1],w=t['w'][:j]))
            replacement.append(dict(t,layer=m['layer_name'],pts=[a,b],w=[w]))
            if j+1<len(t['w']): replacement.append(dict(t,pts=t['pts'][j+1:],w=t['w'][j+1:]))
            r['tracks'][i:i+1]=replacement
            for x,y in (a,b):
                if not any(math.dist((v['x'],v['y']),(x,y))<1e-6 for v in r['vias']):r['vias'].append(dict(x=x,y=y))
            G['del_unit'](owner);G['add_unit'](owner,r['net'],r['tracks'],r['vias'])
        else:
            if not self.movable(owner) or not G['_close_on'](G['U'][owner],c['points'],self.radius): raise ValueError('Blocker not movable/local')
            touched=[owner]

        touched+=[u for u in self.neighbours(c) if u!=owner]
        touched=sorted(set(touched),key=lambda u:(u!=m.get('unit'),G['grp'](u),u))
        if len(touched)>int(G['args'].get('loop_group','24')): raise ValueError('local group too large')
        return touched

    def trial(self,m,c,R0,p0,E0,deadline):
        start=time.process_time()
        G=self.G; original=G.copy(); B0=G['D']; order=list(G['U'])


        saved=G['U'].copy()
        hs=G['hist'].copy();vs=copy.deepcopy(G['VIA_PREV']);fails=set(G['fail'])
        flags={k:copy.deepcopy(G[k]) for k in ('PF','CONS_SOFT','HARD_FOREIGN','SOURCE_FREE','VIA_ANCHOR_ON')}
        semantic={k:copy.deepcopy(G[k]) for k in ('_METRIC_OFF','_ERR_NETS','UNREACHABLE','JOINT_STAT','CHAIN_STAT','RV_STAT','TOPO_STAT','CURRENT_AUDIT','_AGENT_SPECS','_AGENT_PRIORITY') if k in G}
        ok=False;E1=None;R1=None;p1=None;reason=None;phases={'save':time.process_time()-start}
        old_deadline=G.get('_LOOP_DEADLINE')
        stamp=[time.process_time()]
        def phase(name):
            now=time.process_time();phases[name]=now-stamp[0];stamp[0]=now
        try:
            touched=self.apply(m,c)
            phase('apply')
            if G['args'].get('loop_search','limited')=='limited':
                G['_LOOP_DEADLINE']=deadline-min(.5,max(0.,deadline-start)*.2)
            G['CONS_SOFT'][0]=False;G['HARD_FOREIGN'][0]=True
            R1,p1=self.out()
            phase('conflict_list')
            if m['category'] in ('via','layer_name','window_layer','ball_via'):
                blocked=self.static_preflight(R0,R1)
                if blocked:raise ValueError('static pre-check: '+blocked)
            phase('static')
            pair_reason=self.pair_gate(p0,p1,c)
            if m['category'] in ('via','layer_name','window_layer','ball_via') and pair_reason and 'outside' in pair_reason:
                raise ValueError(pair_reason)

            if m['category'] in ('part','priority') or pair_reason is not None:
                if len(touched)>int(G['args'].get('loop_new_max','4' if self.large else '24')):
                    raise ValueError('too many units for a cheap rerouting move')
                for u in touched:G['del_unit'](u)
                for u in touched:
                    if time.process_time()>=deadline or G['time_tight'](): raise TimeoutError('Loop-CPU-Budget')
                    if G['route_unit'](u):G['fail'].discard(u)
                    else:G['fail'].add(u)
                R1,p1=self.out()
            phase('reroute')
            if time.process_time()>=deadline or G['time_tight'](): raise TimeoutError('Loop-CPU-Budget')
            reason,E1=self.evaluate_gate(B0,R0,E0,R1,p0,p1,c)
            phase('gates')
            ok=reason is None
            if time.process_time()>=deadline:ok=False;reason='Loop CPU budget after check'
        except Exception as e:
            reason=type(e).__name__+': '+str(e)[:220]
        finally:
            phase('work_until_abort_or_final')
            G['_LOOP_DEADLINE']=old_deadline
            if not ok:
                for u in list(G['U']):
                    if G['U'][u] is not saved.get(u):G['del_unit'](u)
                for u in order:
                    if u not in G['U']:
                        r=saved[u];G['add_unit'](u,r['net'],r['tracks'],r['vias'])
                if G['D'] is not B0:


                    dynamic={k:G[k] for k in ('U','use','drill_use','SN','SN_OPEN','UOWN','NETU','DDN','_NVER','_GEO')}
                    G.clear();G.update(original);G.update(dynamic)
                G['fail'].clear();G['fail'].update(fails)
                G['VIA_PREV'].clear();G['VIA_PREV'].update(vs)
                for k,v in semantic.items():G[k]=v
            G['hist'][:]=hs
            for k,v in flags.items():G[k][:]=v
            G['_CAND_IDX'][0]=None

            current=G['U'].copy();G['U'].clear()
            for u in order:
                if u in current:G['U'][u]=current.pop(u)
            for u in sorted(current):G['U'][u]=current[u]
            G['exact_conflicts']()
            G['_LOOP_DEADLINE']=old_deadline
            phase('rollback_or_final')
        self.trials+=1
        if not ok:self.rejected[m['category']]=self.rejected.get(m['category'],0)+1
        self.emit(dict(event='move',round_no=self.rounds,bottleneck=c,move=m,taken=ok,
                       reason=reason or 'Cluster improved, full gates held',cpu_s=round(time.process_time()-start,6),
                       cpu_phases_s=phases, move_budget_cpu_s=max(0.,deadline-start),
                       pairs_before=p0,pairs_after=p1,
                       before={k:E0.get(k) for k in ('is_valid','score_v3','conflict_list','open_items','design_rules')},
                       after=None if E1 is None else {k:E1.get(k) for k in ('is_valid','score_v3','conflict_list','open_items','design_rules')}))
        if ok:
            self.accepted[m['category']]=self.accepted.get(m['category'],0)+1
            self.accepted_moves.append(copy.deepcopy(m))
        return ok

    def export(self,c,digest,quick=False):
        d=self.exchange/('%04d_%s'%(self.rounds,digest[:12]));d.mkdir(parents=True,exist_ok=True)
        G=self.G;b=c['bbox'];margin=.5;b=[b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin]
        scale=min(160,1000/max(b[2]-b[0],b[3]-b[1]));size=(max(1,int((b[2]-b[0])*scale)),max(1,int((b[3]-b[1])*scale)))
        in_=Image.new('RGB',size,'white');draw=ImageDraw.Draw(in_)
        def p(q):return ((q[0]-b[0])*scale,(q[1]-b[1])*scale)
        for u,r in sorted(G['U'].items()):
            for t in r['tracks']:draw.line([p(q) for q in t['pts']],fill=('#315e8e' if u in c['unit_list'] else '#b3bdc7'),width=max(1,int(min(t['w'],default=.1)*scale)))
            for v in r['vias']:
                x,y=p((v['x'],v['y']));rr=G['VIA_D']/2*scale;draw.ellipse((x-rr,y-rr,x+rr,y+rr),outline='#245c40',width=2)
        for i in c['pads']:
            for qs in G['D']['pads'][i]['polys'].values():
                for q in qs:draw.polygon([p(v) for v in q],outline='#9b7300')
        for q in c['points']:
            x,y=p(q);draw.ellipse((x-6,y-6,x+6,y+6),outline='red',width=3)
        in_.save(d/'conflict_list.png')


        places={}; previous=G['HARD_FOREIGN'][0];G['HARD_FOREIGN'][0]=True
        try:
            ix0=max(0,int((b[0]-G['X0'])/G['RES']));ix1=min(G['NX'],int((b[2]-G['X0'])/G['RES']))
            iy0=max(0,int((b[1]-G['Y0'])/G['RES']));iy1=min(G['NY'],int((b[3]-G['Y0'])/G['RES']))
            step=max(1,int(.2/G['RES']));cells=np.asarray([iy*G['NX']+ix for iy in range(iy0,iy1,step) for ix in range(ix0,ix1,step)],dtype=np.int64)
            for net in (() if quick else c['net_list']):

                allowed=G['hard_maps'](net)[1].ravel()
                probe=[k for k in cells[:256] if allowed[k]]
                points=[(G['X0']+(int(k)%G['NX']+.5)*G['RES'],G['Y0']+(int(k)//G['NX']+.5)*G['RES']) for k in probe]
                foreign=[geo for u,r in G['U'].items() for li,geo,_,ne in G['_unit_geo'](u,r) if ne!=net]
                holes=[Point(v['x'],v['y']) for r in G['U'].values() for v in r['vias']]
                copper=G['shapely'].union_all(foreign);drills=G['shapely'].union_all(holes)
                places[net]=[[round(x,5),round(y,5)] for x,y in points if (copper.is_empty or Point(x,y).distance(copper)>=G['VIA_D']/2+G['DRC_CLR']) and (drills.is_empty or Point(x,y).distance(drills)>=G['DRC_HOLE'])]
        finally:G['HARD_FOREIGN'][0]=previous
        clean=self.board(G['D'])
        state=dict(version=self.VERSION,cur_state=digest,bottleneck=c,cu_layers=G['LAYERS'],
                   raster=dict(res=G['RES'],origin=[G['X0'],G['Y0']]),free_via_grid_points=places,
                   free_via_grid_points_checked=not quick,
                   components=[f for f in G['D']['fps'] if f['ref'] in c['components']],pads=[clean['pads'][i] for i in c['pads']],
                   previous_moves=copy.deepcopy(self.accepted_moves),
                   unit_list={u:{k:G['U'][u][k] for k in ('net','tracks','vias')} for u in sorted(set(c['unit_list'])|set(c.get('neighbors',[]))|{v['unit'] for v in c.get('vias',[])})},
                   move_format='List: category via/part/layer_name/relay; cur_state binds exactly this snapshot')
        (d/'cur_state.json').write_text(json.dumps(state,sort_keys=True,indent=1))
        try:
            with (d/'moves.json').open('x') as f:
                f.write(json.dumps(self.accepted_moves,sort_keys=True,indent=1)+'\n')
        except FileExistsError:
            pass
        self.emit(dict(event='unsolved',round_no=self.rounds,cur_state=digest,folder=str(d)))

    def trigger(self,pairs):
        count=len(pairs);self.stale=self.stale+1 if count==self.previous else 1;self.previous=count
        self.counts=(self.counts+[count])[-self.k:]
        cells={(p[0],round(p[4]/self.G['RES']),round(p[5]/self.G['RES'])) for p in pairs}
        self.cells={k:self.cells.get(k,0)+1 for k in cells}


        if self.large and (count>self.threshold or len(self.counts)<self.k or min(self.counts)<self.counts[0]):return False
        return bool(count and (self.stale>=self.k or max(self.cells.values(),default=0)>=self.n))

    def tick(self,round_number,pairs=None):
        if not self.explicit_budget:self.limit=self.budget(self.G,self.fraction)
        if self.disabled or self.used>=self.limit or self.trials>=self.cap or self.G['time_tight']():return False
        t=time.process_time();G=self.G;self.rounds=round_number;phases={};triggered=False
        def timed(name,start):phases[name]=phases.get(name,0.)+time.process_time()-start
        try:
            start=time.process_time()
            if pairs is None:
                G['exact_conflicts']();pairs=list(G['PAIRS'])
            p0=pairs
            if not self.trigger(p0):return False
            triggered=True;timed('observe',start)
            deadline=t+max(0,self.limit-self.used)
            clusters=[];skipped=None;start=time.process_time()
            for c in self.clusters(p0):
                neighbours=[u for u,r in G['U'].items() if self.movable(u) and G['_close_on'](r,c['points'],self.radius)]
                c['neighbors']=sorted(neighbours)
                if len(neighbours)>int(G['args'].get('loop_group','24'))+1:
                    self.emit(dict(event='cluster_skipped',round_no=round_number,reason='local group already too large before move',group_n=len(neighbours),bottleneck=c));skipped=skipped or c;continue
                clusters.append(c)
                if len(clusters)>=3:break
            timed('cluster',start)
            if not clusters:
                if skipped:self.export(skipped,self.digest(),quick=True)
                return False
            start=time.process_time();R0,p0=self.out();timed('outputs',start)
            start=time.process_time();E0=self.rate(G['D'],R0,G['NC'],v3=True,minimax=False);timed('initial_evaluation',start)
            if not math.isfinite(E0.get('score_v3',math.inf)):
                self.disabled='incomplete initial evaluation';self.emit(dict(event='blocked',reason=self.disabled));return False
            start=time.process_time();digest=self.digest();timed('state_hash',start)
            for c in clusters:
                candidates=[m for m in self.external if m['cur_state']==digest] if self.input_path else self.candidates(c,digest)
                iterator=iter(candidates)
                while time.process_time()<deadline and self.trials<self.cap:


                    start=time.process_time()
                    try:m=next(iterator)
                    except StopIteration:
                        timed('proposal_build',start);break
                    timed('proposal_build',start)
                    if time.process_time()>=deadline:break
                    if self.input_path and not self.external_local(m,c):continue
                    key=json.dumps(m,sort_keys=True)
                    if key in self.seen:continue
                    self.seen.add(key)
                    if m['cur_state']!=digest:
                        self.emit(dict(event='declined',reason='outdated state',move=m));continue
                    start=time.process_time();blocked=self.prefilter(m,c);timed('prefilter',start)
                    if blocked:
                        self.prefiltered[m['category']]=self.prefiltered.get(m['category'],0)+1
                        self.emit(dict(event='prefiltered',move=m,reason=blocked));continue
                    start=time.process_time();ok=self.trial(m,c,R0,p0,E0,min(deadline,time.process_time()+self.move_cpu));timed('move_records',start)
                    if ok:
                        self.stale=0;self.cells.clear();return True
                start=time.process_time();self.export(c,digest,quick=time.process_time()>=deadline or G['time_tight']());timed('interface',start)
            for m in self.external:
                if json.dumps(m,sort_keys=True) not in self.seen:
                    self.emit(dict(event='waiting',reason='State not reached yet or no local target in checked clusters/budget',move=m))
            return False
        finally:
            elapsed=time.process_time()-t;self.used+=elapsed
            self.emit(dict(event='loop_round',round_no=round_number,triggered=triggered,cpu_s=elapsed,cpu_phases_s=phases,budget_cpu_s=self.limit))


_RL = _RoutingLoop(globals()) if args.get('loop') == '1' or args.get('agent_loop') == '1' else None





ENDGAME = args.get('endgame', '1' if NC else '0') == '1'



WMIN_CONF = args.get('wmin_conf', '1' if NC else '0') == '1'
WMIN_UNTIL = int(args.get('wmin_conf_until', str(2 * MAXIT // 3)))
ALL_CLOSE = float(args.get('all_close', '0'))
CONVERGENCE_LOCAL = args.get('convergence', '') == 'local'
_local_best = [len(units), 0]
END_MAX = int(args.get('endgame_max', '8'))
_es = dict(n=0, before=None, level=0, ab=0, ran=False, cpu=0.0, off=False)
END_TIME = float(args.get('endgame_time', '1.0'))


def local_search_mask(pairs, open_, radius):
    m = np.zeros((NY, NX), dtype=bool)
    def region(x1, y1, x2, y2, border):
        xa = max(0, min(NX, int(math.floor((x1 - border - X0) / RES))))
        xb = max(0, min(NX, int(math.ceil((x2 + border - X0) / RES)) + 1))
        ya = max(0, min(NY, int(math.floor((y1 - border - Y0) / RES))))
        yb = max(0, min(NY, int(math.ceil((y2 + border - Y0) / RES)) + 1))
        m[ya:yb, xa:xb] = True
    for (_l, _d, _a, _b, x, y) in pairs: region(x, y, x, y, radius)
    for u in sorted(open_):
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        boxes = [D['pads'][k]['bbox'] for k in ks]
        if not boxes: continue

        region(min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes),
               6.1 + radius if u.startswith('GND#') else (WINDOW or 3.0) + radius)
    return m


def _close_on(r, sites, R):
    for v in r['vias']:
        if any(math.hypot(v['x'] - x, v['y'] - y) <= R for x, y in sites): return True
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            for x, y in sites:
                s = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
                if math.hypot(a[0] + s * dx - x, a[1] + s * dy - y) <= R: return True
    return False


def endgame(conf, pairs, level, seed, repair=False):
    import copy
    R = (1.0, 2.0, 3.5, 5.0)[level % 4]
    sites = [(x, y) for (_l, _d, _a, _b, x, y) in pairs]
    core_set = sorted(u for u in conf if u in U and not u.startswith(('FIX#', 'TACK#')))
    group = list(core_set)
    for w, r in U.items():
        if w in core_set or w.startswith(('FIX#', 'TACK#')): continue
        if _close_on(r, sites, R): group.append(w)
    if PAIR_NETS: group = sorted(set(group) | {PAIR_NETS[w] for w in group if w in PAIR_NETS and PAIR_NETS[w] in U and PAIR_NETS[w] not in FIXED})
    if len(group) == len(core_set) or len(group) > int(args.get('endgame_group', '40')):
        print('   Endgame: radius %.1f mm, group %d with %d conflict nets (%s)' % (R, len(group), len(core_set), 'nobody in the way' if len(group) == len(core_set) else 'too large'), flush=True)
        return False
    save = {w: copy.deepcopy(U[w]) for w in group}; hsave = hist.copy(); PFs = PF[0]; soft = CONS_SOFT[0]; CONS_SOFT[0] = False
    source_free_before = SOURCE_FREE[0]


    X = groups_index(group)
    vacant = False


    for attempt in range(int(args.get('endgame_attempts', '3'))):
        _end_used = _es.get('work', 0) if BUDGET_WORK else _es['cpu']
        _end_total = work_total() if BUDGET_WORK else search_cpu() - observer_cpu()
        if _end_used > max(30.0, END_TIME * (_end_total - _end_used)) or time_tight():
            if BUDGET_WORK: print('   Endgame: work share used up (%d jobs)' % _end_used, flush=True)
            else: print('   Endgame: time share used up (%.0f s)' % _es['cpu'], flush=True)
            _es['off'] = True; break
        rng_ = budget_rng(1000 + 10 * seed + attempt); _tc = time.process_time()
        _end_work = work_total()
        _mark_cpu_before = observer_cpu()
        SOURCE_FREE[0] = repair or source_free_before
        HARD_FOREIGN[0] = repair and attempt == int(args.get('endgame_attempts', '3')) - 1
        for w in group:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if attempt == 0: order = core_set + sorted((w for w in group if w not in core_set), key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else (grp(w)[0], rng_.rand()))
        else: order = sorted(group, key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else (grp(w)[0] if attempt % 2 else 0, rng_.rand()))
        if PAIR_NETS:
            positions = {w:i for i,w in enumerate(order)}
            order.sort(key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('pair_rank',0), min(w,PAIR_NETS[w]) if w in PAIR_NETS else '', w if w in PAIR_NETS else positions[w]))
        ok = True; history = []; _from = False
        for w in pair_series(order):
            if time_tight(): _from = True; break
            if not route_unit(w): ok = False
        for _k in range(int(args.get('endgame_iter', '14'))):
            if not ok or _from: break
            cfg = conflicts_group(group, X); history.append(len(cfg))
            if _ET: _ET.observe(cfg, it if 'it' in globals() else 1, 'endgame', geometry=False)
            if args.get('endgame_chk') == '1':
                _v = exact_conflicts(); assert set(_v) == set(cfg), ('Endgame-Checking', sorted(set(_v) ^ set(cfg)))
            if not cfg: vacant = True; break
            if len(history) >= 5 and min(history[-4:]) >= history[-5]: break
            for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
            PF[0] *= 1.5
            if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
            for w in pair_series(sorted((w for w in order if w in cfg), key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else rng_.rand())):
                if time_tight(): _from = True; break
                if not pair_unit(w): del_unit(w)
                if not route_unit(w): ok = False
        print('   Endgame: radius %.1f mm, group %d (of which GND stubs %d), ramp-up %d, conflicts per round %s%s' % (
            R, len(group), sum(1 for w in group if w.startswith('GND#')), attempt + 1, history, '' if ok else ', incomplete'), flush=True)
        HARD_FOREIGN[0] = False
        SOURCE_FREE[0] = source_free_before
        _es['cpu'] += time.process_time() - _tc - (observer_cpu() - _mark_cpu_before)
        if BUDGET_WORK: _es['work'] = _es.get('work', 0) + work_total() - _end_work
        if _from:
            print('   Endgame: Compute time budget used up, ramp-up aborted', flush=True); _es['off'] = True; break
        if vacant: break
    hist[:] = hsave; PF[0] = PFs; CONS_SOFT[0] = soft
    if vacant: return True
    for w in group:
        if w in U: del_unit(w)
    for w in group: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    return False









MOVE = args.get('move', '0') == '1'
MOVE_MAX = int(args.get('move_max', '80'))
MOVE_SITES = int(args.get('move_sites', '10'))
MOVE_GROUP = int(args.get('move_group', '25'))
MOVE_SHARE = float(args.get('move_share', '0.25'))
MOVE_ITER = int(args.get('move_iter', '10'))



MOVE_STRATEGY = (dict(R=2.0, series='core_set', warm=True), dict(R=3.5, series='core_set', warm=True), dict(R=1.0, series='core_set'),
             dict(R=2.0, series='core_set'), dict(R=2.0, series='chance'), dict(R=3.5, series='core_set'),
             dict(R=2.0, series='core_set', repair=True), dict(R=5.0, series='core_set'))
MOVE_WARM_PF = float(args.get('move_warm_pf', '50'))
_zs = dict(n=0, ok=0, cpu=0.0, work=0, quiet=0, best=None, log=None)

MOVE_POLICY = json.load(open(args['move_policy']))['table'] if args.get('move_policy') else None
def _move_pads2(x, y):
    return sum(1 for p in D['pads'] if abs(p['x'] - x) <= 2.0 and abs(p['y'] - y) <= 2.0)

def _move_series(site):
    if not MOVE_POLICY: return list(enumerate(MOVE_STRATEGY))
    k = len(site['core_set']); p = _move_pads2(site['x'], site['y'])
    layer_name = '%s|k%d|p%d' % (site['type_'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)
    tab = MOVE_POLICY.get(layer_name) or MOVE_POLICY.get(site['type_']) or {}
    def amount(js):
        s = js[1]; e = tab.get('%s|%s|%d|%d' % (s['R'], s['series'], int(bool(s.get('repair'))), int(bool(s.get('warm')))))
        return (-(e['amount'] if e else -1.0), js[0])
    return sorted(enumerate(MOVE_STRATEGY), key=amount)


def _move_log(row):
    if _zs['log'] is None:
        _zs['log'] = open(args.get('move_log', OUT.replace('.json', '') + '_moves.jsonl'), 'w')
    _zs['log'].write(json.dumps(row, sort_keys=True) + '\n'); _zs['log'].flush()


def _move_sites(pairs, open_):
    st = []
    for (_l, _d, a, b, x, y) in sorted(pairs, key=lambda p: (round(p[4], 3), round(p[5], 3), p[2], p[3])):
        core_set = {w for w in (a, b) if w in U and not w.startswith(('FIX#', 'TACK#'))}
        if not core_set: continue
        for s in st:
            if s['type_'] == 'conflict' and math.hypot(s['x'] - x, s['y'] - y) <= 0.5:
                s['core_set'] |= core_set; break
        else: st.append(dict(type_='conflict', x=x, y=y, core_set=core_set))
    for u in sorted(open_):
        if u in FIXED or u.startswith(('FIX#', 'TACK#')): continue
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        seen = []
        for k in ks:
            p = D['pads'][k]
            if any(math.hypot(p['x'] - x, p['y'] - y) <= 1.0 for x, y in seen): continue
            seen.append((p['x'], p['y']))
            st.append(dict(type_='open_items', x=p['x'], y=p['y'], core_set={u}))
            if len(seen) >= 4: break
    return st


def _move_spacing(r, p):
    x, y = p; d = float('inf')
    for v in r['vias']: d = min(d, math.hypot(v['x'] - x, v['y'] - y))
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            s_ = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
            d = min(d, math.hypot(a[0] + s_ * dx - x, a[1] + s_ * dy - y))
    return d


MOVE_LOCAL = args.get('move_local', '1') == '1'
MOVE_CHK = args.get('move_chk', '0') == '1'


_MOVE_BORDER = _DMAX + max(DRC_HOLE, VIA_DR / 2 + VIA_R + HOLE_CLR, VIA_DR / 2 + HOLE_CLR, VIA_DR / 2 + H2H) + VIA_R + 0.05


def conflicts_local(group):
    g = set(group); b = None
    for w in sorted(g):
        if w not in U: continue
        for _li, k, _kb, _ne in _unit_geo(w, U[w]):
            x0, y0, x1, y1 = k.bounds
            b = [x0, y0, x1, y1] if b is None else [min(b[0], x0), min(b[1], y0), max(b[2], x1), max(b[3], y1)]
    if b is None: return {}
    F = (b[0] - _MOVE_BORDER, b[1] - _MOVE_BORDER, b[2] + _MOVE_BORDER, b[3] + _MOVE_BORDER)
    sv = (PAIRS[:], PAIRS_RAW[:], FIX_PAIRS[:])
    def selection(conf):
        pairs = [p for p in PAIRS if p[2] in g or p[3] in g]
        with_ = {u for p in pairs for u in p[2:4]}
        return {u: cs for u, cs in conf.items() if u in with_}, sorted(pairs)
    res, pairs = selection(exact_conflicts(window=F))
    if MOVE_CHK:
        resv, pairs_v = selection(exact_conflicts())
        equal = (sorted(res) == sorted(resv) and pairs == pairs_v
                  and all(sorted(res[u]) == sorted(resv[u]) for u in res if u in g))
        _zs['chk'] = _zs.get('chk', 0) + 1
        if not equal:
            raise AssertionError('Move window check deviates: %s / %s' % (sorted(set(res) ^ set(resv)), sorted(set(pairs) ^ set(pairs_v))[:6]))
    PAIRS[:], PAIRS_RAW[:], FIX_PAIRS[:] = sv
    return res


def _move_conf(group, X):
    return conflicts_local(group) if MOVE_LOCAL else conflicts_group(group, X)


def _move_value(group, X):
    cfg = _move_conf(group, X)
    return len(cfg) + 2 * sum(1 for w in group if w in fail), cfg


def move_attempt(site, strategy, seed):
    """Try one repair move at a spot with one strategy.

    The move rips up the units at the spot (its core nets, plus neighbours within radius R unless the strategy is 'warm')
    and reroutes this group for a few local iterations, ordered by the strategy. The move is kept if the spot is better
    afterwards (fewer conflicts or fewer unrouted units in the group), otherwise the previous state is restored. Every
    move is logged with its features to <OUT>_move_records.jsonl, the training data for learning/move_choice.py.
    Returns True if the move was accepted.
    """
    import copy
    R = strategy['R']; xy = [(site['x'], site['y'])]
    core_set = sorted(site['core_set'])
    close = [w for w, r in sorted(U.items()) if w not in site['core_set'] and not w.startswith(('FIX#', 'TACK#'))
           and w not in FIXED and _close_on(r, xy, R)]
    capped = len(core_set) + len(close) > MOVE_GROUP
    if capped:
        close = sorted(close, key=lambda w: (_move_spacing(U[w], xy[0]), w))[:max(0, MOVE_GROUP - len(core_set))]
    group = list(core_set) + close
    if PAIR_NETS: group = sorted(set(group) | {PAIR_NETS[w] for w in group if w in PAIR_NETS and PAIR_NETS[w] in units and PAIR_NETS[w] not in FIXED})
    row = dict(move=_zs['n'], type_=site['type_'], x=round(site['x'], 3), y=round(site['y'], 3), R=R, series=strategy['series'], warm=bool(strategy.get('warm')),
                 repair=bool(strategy.get('repair')), core_set=len(core_set), group=len(group),
                 gnd=sum(1 for w in group if w.startswith('GND#')), pair=sum(1 for w in group if w in PAIR_NETS),
                 kinds=sorted({(NC['net_list'].get(w) or {}).get('category', 'signal') for w in core_set}) if NC else [],
                 pads=sum(1 for p in D['pads'] if abs(p['x'] - site['x']) <= R and abs(p['y'] - site['y']) <= R),
                 pads2=_move_pads2(site['x'], site['y']), policy=bool(MOVE_POLICY),
                 conflicts_ges=len(PAIRS), open_ges=len(fail), cu_layers=NL, capped=capped)
    if len(group) > MOVE_GROUP:
        row.update(outcome='group_too_large'); _move_log(row); return False
    _tc = time.process_time(); _ta = work_total() if BUDGET_WORK else 0
    X = None if MOVE_LOCAL else groups_index(group)
    before, _ = _move_value(group, X)
    save = {w: copy.deepcopy(U[w]) for w in group if w in U}; fail_before = {w for w in group if w in fail}
    hsave = hist.copy(); PFs = PF[0]; soft = CONS_SOFT[0]; CONS_SOFT[0] = False; qf = SOURCE_FREE[0]
    SOURCE_FREE[0] = bool(strategy.get('repair')) or qf; HARD_FOREIGN[0] = False
    rng_ = budget_rng(5000 + seed)
    other = [w for w in group if w not in site['core_set']]
    if strategy.get('warm'):
        for w in core_set:
            if w in U: del_unit(w)
        PF[0] = min(PFs, MOVE_WARM_PF)
        order = list(core_set)
    else:
        for w in group:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if strategy['series'] == 'core_set': order = core_set + sorted(other, key=lambda w: (grp(w)[0], rng_.rand()))
        else: order = sorted(group, key=lambda w: rng_.rand())
    ok = {}; history = []
    for w in pair_series(order): ok[w] = route_unit(w)
    for _k in range(MOVE_ITER):
        cfg = _move_conf(group, X); history.append(len(cfg))
        if not cfg or time_tight(0.95): break
        if len(history) >= 4 and min(history[-3:]) >= history[-4]: break
        if strategy.get('warm') and len(history) >= 3 and min(history[-2:]) >= min(history[:-2]): break
        for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
        PF[0] *= 1.5
        if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
        for w in pair_series(sorted((w for w in group if w in cfg or not ok.get(w, True)), key=lambda w: rng_.rand())):
            if not pair_unit(w): del_unit(w)
            ok[w] = route_unit(w)
    for w in group:
        if ok.get(w, w in U): fail.discard(w)
        else: fail.add(w)
    post, _ = _move_value(group, X)
    hist[:] = hsave; PF[0] = PFs; CONS_SOFT[0] = soft; SOURCE_FREE[0] = qf
    taken = post < before
    if not taken:
        for w in group:
            if w in U: del_unit(w)
        for w, r in sorted(save.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
        for w in group:
            if w in fail_before: fail.add(w)
            else: fail.discard(w)
    cpu = time.process_time() - _tc; _zs['cpu'] += cpu
    if BUDGET_WORK: _zs['work'] += work_total() - _ta
    row.update(outcome='taken' if taken else 'rejected', before=before, after=post, history=history, cpu_s=round(cpu, 3))
    _move_log(row)
    return taken


def move_remember():
    import copy
    amount = len(exact_conflicts()) + len(fail)
    if _zs.get('state') is None or amount < _zs['state'][0]:
        _zs['state'] = (amount, {w: copy.deepcopy(r) for w, r in U.items() if not w.startswith(('FIX#', 'TACK#'))}, set(fail))


def move_best():
    global conf
    if _zs.get('state') is None: return False
    now = len(exact_conflicts()) + len(fail)
    if _zs['state'][0] >= now: return False
    _, prior, prev_fail = _zs['state']
    for w in sorted(set(U) | set(prior)):
        if w.startswith(('FIX#', 'TACK#')): continue
        if w in U: del_unit(w)
    for w, r in sorted(prior.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
    fail.clear(); fail.update(prev_fail)
    conf = exact_conflicts()
    print('   Move loop: best state restored (%d instead of %d conflicts+open)' % (_zs['state'][0], now), flush=True)
    return True


def move_budget_off():
    if _zs['n'] >= MOVE_MAX or time_tight(0.95): return True
    if BUDGET_WORK: return _zs['work'] >= MOVE_SHARE * WORK_MAX
    return CPU_MAX > 0 and _zs['cpu'] >= MOVE_SHARE * CPU_MAX


def move_loop(phase, seed):
    """One pass of repair moves over all remaining problem spots (enabled with move=1).

    A spot is one conflict or one unrouted unit. For each spot the strategies are tried in order (fixed order, or the
    learned order from move_policy=<file>, see learning/move_choice.py) until one move is accepted. Each strategy is
    tried at most twice per spot. The loop only runs when conflicts plus unrouted units are at most move_sites, and
    stops when the move budget (move_max, move_share of the CPU budget) is used up.
    Returns the number of accepted moves in this pass.
    """
    global conf
    conf = exact_conflicts()
    if not (0 < len(conf) + len(fail) <= MOVE_SITES): return 0
    good = 0
    for i, st in enumerate(_move_sites(list(PAIRS), set(fail))):
        if not (st['core_set'] & (set(conf) | fail)): continue
        for j, strategy in _move_series(st):
            if move_budget_off(): break
            tabu = (round(st['x'], 1), round(st['y'], 1), j)
            if _zs.setdefault('tabu', {}).get(tabu, 0) >= 2: continue
            _zs['tabu'][tabu] = _zs['tabu'].get(tabu, 0) + 1
            _zs['n'] += 1
            if move_attempt(st, strategy, 100 * seed + 10 * i + j):
                good += 1; _zs['ok'] += 1; conf = exact_conflicts(); break
    print('   Move loop (%s): %d moves total, %d accepted in this pass, now %d conflicts, %d open (%.0f CPU-s)' % (
        phase, _zs['n'], good, len(conf), len(fail), _zs['cpu']), flush=True)
    return good


if MAXIT < 2 and todo is None:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANCHOR_ON[0] = True
_AL = None
if args.get('agent_loop') == '1':
    from codex.agent_loop.cur_state import AgentLoop
    _AL = AgentLoop(globals(), _AGENT_SAVED)
for it in range(_AGENT_SAVED['round_no'] if _AGENT_SAVED else 2, MAXIT + 1):
    CONS_SOFT[0] = it <= int(args.get('cons_iter', '10'))

    conf = exact_conflicts(); _es['ran'] = True
    if (_AL.before_round(it, list(PAIRS)) if _AL else (_RL and _RL.tick(it - 1, list(PAIRS)))):
        print('   Routing loop: local move accepted', flush=True)
        conf = exact_conflicts()
    negotiation_measure(it - 1, conf)
    _mark_conf = _ET.conflict_copy(conf) if _ET else None
    if VIA_PRECEDENCE and it <= WMIN_UNTIL:
        conf, _nvr = precedence_filter(conf)
        if _nvr: print('   Via priority (R25): %d spots where only the signal yields' % _nvr, flush=True)
    if WMIN_CONF and it <= WMIN_UNTIL:
        _tight = tight_conflicts()
        for _u, _c in _tight.items(): conf.setdefault(_u, []).extend(_c)
        if _ET: _ET.complete(_mark_conf, _tight)
        if _tight: print('   too narrow (wmin_conf): %d units involved' % len(_tight), flush=True)
    if _ET and not _LEARN_ON: _ET.observe(_mark_conf, it - 1)
    todo = (set(conf) | fail) - set(FIXED)
    print('Iteration %d: conflicts %d, incomplete %d, PF %.2f (%.0fs)' % (it, len(conf), len(fail), PF[0], time.time() - t0), flush=True)
    work_mark('round_%d' % it)
    if it % 4 == 0:
        import collections
        print('   Pairs', len(PAIRS), collections.Counter(tuple(sorted((x[2].split('#')[0][:12], x[3].split('#')[0][:12]))) for x in PAIRS).most_common(12), flush=True)
        print('   Examples', sorted(PAIRS, key=lambda x: x[1])[:6], flush=True)
    if not todo: break
    if time_tight():
        print('%s 80 %% used up: negotiation ends with %d conflicts' % (budget_message(), len(conf)), flush=True); break
    if MOVE and it >= 8:
        _zsum = len(conf) + len(fail)
        if _zs['best'] is None or _zsum < _zs['best']: _zs['best'] = _zsum; _zs['quiet'] = 0
        else: _zs['quiet'] += 1
        if _zs['quiet'] >= 3 and 0 < _zsum <= MOVE_SITES and not move_budget_off():
            _zs['quiet'] = 0; _z_good = 0
            for _zr in range(4):
                if not move_loop('Negotiation', 10 * it + _zr): break
                _z_good += 1; move_remember()
                if not conf and not fail: break
            if _z_good:
                _zs['best'] = len(conf) + len(fail); continue
    if ENDGAME and not fail and len(conf) <= END_MAX and it >= 6:
        if _es.get('ok_before') is not None:
            if set(conf) == _es['ok_before']:
                _es['apparent'] = _es.get('apparent', 0) + 1; _es['level'] += 1; _es['ab'] = it + 3
                print('   Endgame: apparent success %d (same conflict back again)' % _es['apparent'], flush=True)
                if _es['apparent'] >= 2: _es['off'] = True; print('   Endgame: off for this run', flush=True)
            _es['ok_before'] = None
        _es['n'] = _es['n'] + 1 if set(conf) == _es['before'] else 1; _es['before'] = set(conf)
        if _es['n'] >= 4 and it >= _es['ab'] and not _es['off']:
            _ok = endgame(conf, list(PAIRS), _es['level'], it)
            print('   Endgame (ramp-up round %d): %s' % (_es['level'] + 1, 'conflict_free' if _ok else 'without success, old state'), flush=True)
            if _ok: _es['ok_before'] = set(conf); continue
            _es['level'] += 1; _es['ab'] = it + 3
    else:
        _es['n'] = 0; _es['before'] = None
    for u, b in conf.items(): hist[np.array(b, dtype=np.int64)] += HINC
    PF[0] *= float(args.get('pfmul', '1.5'))
    if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
    fail_prev = fail; fail = set(); _next = set()
    all_items = args.get('all_items', '1') == '1' and len(conf) > int(args.get('goal_from', '0'))
    close = ALL_CLOSE
    if CONVERGENCE_LOCAL:
        if len(conf) < _local_best[0]: _local_best[:] = [len(conf), 0]
        else: _local_best[1] += 1
        all_items = it % 4 == 0
        close = ALL_CLOSE or min(6.0, 1.0 + _local_best[1] / 4)
    rng = budget_rng(it)
    def gkey(u):
        if PAIR_NETS:


            p = PAIR_NETS.get(u)
            if p: return (grp(u)[0], -NC['net_list'][u].get('pair_rank', 0), min(u, p), u)
            rank = (NC['net_list'].get(u) or {}).get('rank', 0) if args.get('rank_all', '0') == '1' else 0
            if NC:
                rank *= float(args.get('cost_rank_' + (NC['net_list'].get(u) or {}).get('category', 'signal'), '1'))
            return (grp(u)[0], -rank, '', learn_series_value(u,rng.rand()))
        if NC and args.get('rank_all', '0') == '1':
            _net_copper = NC['net_list'].get(u) or {}
            return (grp(u)[0], -_net_copper.get('rank', 0) * float(args.get('cost_rank_' + _net_copper.get('category', 'signal'), '1')), learn_series_value(u,rng.rand()))
        return (grp(u)[0], learn_series_value(u,rng.rand()))


    if all_items and close > 0:


        if CONVERGENCE_LOCAL:
            _m = local_search_mask(PAIRS, fail_prev, close)
        else:
            _m = np.zeros((NY, NX), dtype=bool); _rr = int(close / RES)
            for (_l, _d, _a, _b, _x, _y) in PAIRS:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                _m[max(0, _iy - _rr):_iy + _rr + 1, max(0, _ix - _rr):_ix + _rr + 1] = True
        _mf = _m.ravel()
        series = [u for u in units if u in todo or (u in U and u not in FIXED and len(U[u]['cells']) and bool(_mf[U[u]['cells'] % NN].any()))]
        print('   Radius %.1f mm: %d of %d units new' % (close, len(series), len(units)), flush=True)
    else:
        series = units if all_items else sorted(todo)
    if PAIR_NETS:
        series = sorted(set(series) | {PAIR_NETS[u] for u in series if u in PAIR_NETS and PAIR_NETS[u] in units})
    _negotiation_series = pair_series(sorted(series, key=gkey))
    if _AL:
        from codex.agent_loop.moves import order as _agent_order
        _negotiation_series = _agent_order(_negotiation_series, _AGENT_PRIORITY)
    for u in _negotiation_series:
        if time_tight():
            fail |= fail_prev - _next; break
        if pair_unit(u):
            _next.update((u,PAIR_NETS[u]))
            if route_unit(u): fail.discard(u); fail.discard(PAIR_NETS[u])
            else: fail.update((u,PAIR_NETS[u]))
        else:
            del_unit(u); _next.add(u)
            if not route_unit(u): fail.add(u)
        if u in PAIR_NETS and u > PAIR_NETS[u] and u not in fail and PAIR_NETS[u] not in fail:
            pair_couple(only={u, PAIR_NETS[u]})
if ENDGAME and _es['ran'] and not fail:
    conf = exact_conflicts()
    for _st in range(_es['level'], _es['level'] + 4):
        if not (0 < len(conf) <= END_MAX) or _es['off']: break
        _ok = endgame(conf, list(PAIRS), _st, 100 + _st)
        print('Endgame after the load pass (ramp-up round %d): %s' % (_st + 1, 'conflict_free' if _ok else 'without success'), flush=True)
        conf = exact_conflicts()
    todo = (set(conf) | fail) - set(FIXED)
elif _es['ran']:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
if ENDGAME and NC and args.get('endgame_repair', '1') == '1' and not fail and not time_tight():



    conf = exact_conflicts()
    if 0 < len(conf) <= END_MAX:
        _es['off'] = False; _es_prev_cpu = _es['cpu']; _es['cpu'] = 0.0
        for _st in range(4):
            if time_tight(): break

            _ok = endgame(conf, list(PAIRS), _st, 24 + _st, repair=True)
            print('Endgame repair: radius %.1f mm, %s' % ((1.0, 2.0, 3.5, 5.0)[_st], 'conflict_free' if _ok else 'old state'), flush=True)
            conf = exact_conflicts()
            if not conf or _es['off']: break
        _es['cpu'] += _es_prev_cpu
        todo = (set(conf) | fail) - set(FIXED)
if MOVE and _es['ran']:
    move_best()
    for _zr in range(4):
        if move_budget_off() or not move_loop('End %d' % (_zr + 1), 900 + _zr): break
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANCHOR_ON[0] = False
CONS_SOFT[0] = False

_cf_checkpoint = exact_conflicts(); PAIRS_LAST = list(PAIRS)
work_mark('before_post_run')
negotiation_measure(it if 'it' in globals() else 1, _cf_checkpoint, 'before_post_run')
if _ET and not _LEARN_ON: _ET.observe(_cf_checkpoint, it if 'it' in globals() else 1)
json.dump(PAIRS_LAST, open(OUT.replace('.json', '_conflicts_before_widen.json'), 'w'))
if ((BGA_PLAN and args.get('bga_budget', '0') == '1') or args.get('budget_before_post_run', '0') == '1' or globals().get('_EARLY_INFO') is not None) and time_tight() and (_cf_checkpoint or fail):

    _ct, _cv = export_geometry()
    _cc = {u:len(cs) for u,cs in _cf_checkpoint.items()}
    _safe = export_data(_ct, _cv, _cc,
        final=(_LEARN_STOP or 'cpu_budget') if _LEARN_ON else 'budget_before_post_run',
        **({'learning_state': json.loads(_LEARN_SNAPSHOT)['learning_state'], 'learning_budget':json.loads(_LEARN_SNAPSHOT)['learning_budget']} if _LEARN_ON else {}))
    json.dump(_safe, open(OUT,'w'))
    json.dump(PAIRS_LAST,open(OUT.replace('.json','_conflicts.json'),'w'))
    print('Compute time budget: invalid state saved, width/shape post-pass skipped',flush=True)
    print('End: conflicts',_cc,'incomplete',sorted(fail),flush=True)
    print('Tracks',len(_ct),'Vias',len(_cv),'Time %.0fs' % (time.time()-t_start),flush=True)
    if _ET: _ET.write_out(_safe['tracks'], _safe['vias'], absent=_safe['failed'], zone_list=_safe.get('zone_list', []) + _safe.get('zones', []))
    sys.exit(0)
import copy
def unit_len(u):
    r = U[u]
    if CONS and any(v.get('gr') for v in r['vias']):
        return sum(math.dist(a, c) for t in r['tracks'] if not t.get('web') for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
    return sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
def grp_key(u): return grp(u)[0]


WT_KIND = None
if WEIGHTED:
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
        from common.importance import weights as _weights, swap_better as _swap_better
        WT_KIND = _weights()
    except Exception as _e:
        print('weighted=%d: weights from analysis/evaluation.py not loadable (%r), staying with bare length' % (WEIGHTED, _e), flush=True)
        WEIGHTED = 0
def u_wt(u):
    if WT_KIND is None or u not in U: return 1.0
    return WT_KIND.get(NC['net_list'].get(U[u]['net'], {}).get('category', 'signal'), 1.0)
def u_vias(u):
    r = U[u]
    return n_sv(r)
def unit_len_wt(u):
    nv = u_vias(u)
    return u_wt(u) * (unit_len(u) - 0.6 * nv) + 0.6 * nv
def current_resistance(u, reachable=False):
    r = U[u]; net = r['net']; C = cls(net)
    k = (NC['net_list'].get(net) or {}) if NC else {}
    from common.flow import analysis as fa, goal_width as fz
    info = fa(r['tracks'], r['vias'], [D['pads'][i] for i in by_net_g.get(net, [])], k.get('flow') or {})
    share = info['share'] if info and info['total'] > 0 else None
    cap = None
    if reachable and r['tracks']:
        xy = [q for t in r['tracks'] for q in t['pts']]
        box = (min(q[0] for q in xy) - 1, min(q[1] for q in xy) - 1, max(q[0] for q in xy) + 1, max(q[1] for q in xy) + 1)
        cap = maps(net, box, C['w'] * max(f_inner(net, LI[t['layer']]) for t in r['tracks']))[6]
    out = 0.0
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('web'): continue
        li = LI[t['layer']]; thick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            f = share.get((ti, si), 0.0) if share is not None else 1.0
            if cap is not None:
                goal = fz(f, C['wmin'], C['w'], float(args.get('flow_exp', '0.5'))) * f_inner(net, li)
                w = max(w, math.floor(min(goal, line_cap_min(cap[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
            out += f * f * 1.72e-5 * math.dist(a, b) / (w * thick)
    out += 0.0024 * sum(1 for v in r['vias'] if not v.get('gr') and not v.get('rv'))
    return out


def routing_mass(u):
    if NC and args.get('current_short') == '1' and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw'):

        return current_resistance(u, reachable=True) * cls(U[u]['net'])['w'] * 0.035 / 1.72e-5
    return unit_len(u)


def routing_mass_wt(u):


    if NC and args.get('current_short') == '1' and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw'):
        return u_wt(u) * routing_mass(u)
    return unit_len_wt(u)


def short_key(u): return -unit_len(u) * (u_wt(u) if WEIGHTED >= 2 else 1.0)


R19_SHORT = args.get('r19_short', '0') == '1' and NC is not None
R19_SHORT_STAT = dict(attempts=0, more_vias_rejected=0, extra_vias_rejected=0)
def r19_short_ok(prior, new):
    return n_sv(new) <= n_sv(prior)

if args.get('short') == '1' and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3
    _CAND_IDX[0] = None
    for rnd in range(int(args.get('short_rounds', '2'))):
        better = 0; gain = 0.0
        for u in sorted([u for u in units if not u.startswith('GND#') and u in U and not pair_unit(u)], key=short_key):
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); L0 = routing_mass(u); _f0 = cons_missing(u) if CONS else 0
            del_unit(u)
            ok = route_unit(u)
            if ok and routing_mass(u) < L0 - 0.05 and (not CONS or cons_missing(u) <= _f0):
                if R19_SHORT and (NC['net_list'].get(old['net']) or {}).get('category') in ('signal', 'analog', 'fast'):
                    R19_SHORT_STAT['attempts'] += 1
                    if not r19_short_ok(old, U[u]):
                        R19_SHORT_STAT['more_vias_rejected'] += 1
                        R19_SHORT_STAT['extra_vias_rejected'] += n_sv(U[u]) - n_sv(old)
                        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u); continue
                if unit_conflict_free(u): better += 1; gain += L0 - routing_mass(u); cand_idx_changed(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
        print('Shortening round %d: %d nets shorter, %.1f mm together (%.0fs)' % (rnd + 1, better, gain, time.time() - t2), flush=True)
        if not better: break
    PF[0] = PFs
    if R19_SHORT: print('R19 shortening:', R19_SHORT_STAT, flush=True)



SHORT_VIA = float(args.get('short_via', '8' if NC and 'start' not in args else '0'))
if SHORT_VIA > 0 and NC is not None and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _CAND_IDX[0] = None; VIA_FACTOR[0] = SHORT_VIA
    _mm = float(args.get('short_via_mm', '3.0')); _saved = 0; _more = 0.0
    def _via_candidate(u):
        if pair_unit(u) or u.startswith(('GND#', 'FIX#', 'TACK#')) or u not in U or not U[u]['vias']: return False
        if NC['net_list'].get(U[u]['net'], {}).get('category') in ('current_val', 'supply', 'sw', 'gnd'): return False
        return not any(v.get('n') or v.get('gr') for v in U[u]['vias'])



    def _via_wt(u): return FAST_VIA if FAST_VIA != 1 and actual_fast(U[u]['net']) else 1.0
    def _via_key(u):
        if WEIGHTED >= 2: return (-n_sv(U[u]) * _via_wt(u) * u_wt(u), u)
        return (-n_sv(U[u]) * _via_wt(u), u)
    for rnd in range(int(args.get('short_via_rounds', '2'))):
        _n = 0
        for u in sorted([u for u in units if _via_candidate(u)], key=_via_key):
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); _v0 = n_sv(old); L0 = unit_len(u) - 0.6 * _v0; _cons_ = _via_wt(u)
            del_unit(u)
            ok = route_unit(u)
            _v1 = n_sv(U[u]); L1 = unit_len(u) - 0.6 * _v1
            _allowed = _mm * (2.0 if NC['net_list'].get(old['net'], {}).get('category', 'signal') == 'signal' else 1.0) * _cons_
            if ok and _v1 < _v0 and L1 <= L0 + _allowed * (_v0 - _v1) and unit_conflict_free(u):
                _n += 1; _saved += _v0 - _v1; _more += L1 - L0; cand_idx_changed(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
        if not _n: break
    VIA_FACTOR[0] = 1.0; PF[0] = PFs
    print('Via round (via price x%g): %d vias saved, tracks %.1f mm longer for it (%.0fs)' % (SHORT_VIA, _saved, _more, time.time() - t2), flush=True)





if (VAP or EPM) and args.get('via_at_pad_clear', '0' if 'start' in args else '1') == '1' and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _nr = 0; _nv = 0; _more = 0.0; _nep = 0; _wt = 0.0
    _detour = float(args.get('via_at_pad_detour', '2.0')); _gmin = float(args.get('via_at_pad_gain', '0.6'))
    def _quality(u):
        r = U[u]; _p = D['pads'][int(u[4:])]; _v = r['vias'][0]; _c, _w = _vap_cells(int(u[4:]))
        _j = np.flatnonzero(_c == int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES))
        return (sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:]))
                + 2.0 * math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']) + VAP_K * (float(_w[_j[0]]) if len(_j) else 1.0))
    for u in [u for u in units if u.startswith('GND#') and u in U and U[u]['vias']]:
        if time_tight(0.9): break
        _k = int(u[4:]); _n0 = len(U[u]['vias'])
        if _k in EPM:
            if _n0 >= EPM[_k]['nominal']: continue
        elif not VAP: continue
        else: _g0 = _quality(u)
        save = {u: copy.deepcopy(U[u])}
        del_unit(u)
        if _k in EPM:
            CONS_SOFT[0] = True; ok = route_unit(u); CONS_SOFT[0] = False
            ok = ok and len(U[u]['vias']) > _n0; _g1 = 0.0; _g0 = 0.0
        else:
            PF[0] = 0.01; ok = route_unit(u); PF[0] = 1e3
            if LAYER_STACK: ok = ok and bool(U[u]['vias'])
            if ok: _g1 = _quality(u); ok = _g1 < _g0 - _gmin
        if ok:
            _nv += 1; _pp = D['pads'][_k]
            _cf = conflicts_group([u], groups_index([u])); in_way = sorted(w for w in _cf if w != u)
            if not _cf:
                _nr += 1; _wt += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
            ok = 0 < len(in_way) <= 4 and not any(w.startswith(('GND#', 'FIX#', 'TACK#')) for w in in_way)
            _reason = 'accepted' if ok else ('foreign pad in the way' if not in_way else 'other stubs, fixed units or more than four in the way')
            if ok:
                if PAIR_SEARCH_SPACE: in_way=sorted(set(in_way)|{PAIR_NETS[w] for w in in_way if pair_unit(w)})
                L0 = sum(unit_len(w) - 0.6 * n_sv(U[w]) for w in in_way); _v0 = sum(n_sv(U[w]) for w in in_way)
                _f0 = sum(cons_missing(w) for w in in_way) if CONS else 0
                for w in in_way: save[w]=copy.deepcopy(U[w])
                for w in pair_series(in_way):
                    if not pair_unit(w): del_unit(w)
                    if not route_unit(w): ok = False
                group = [u] + in_way
                if not ok: _reason = 'no way for the units in the way'
                else:
                    L1 = sum(unit_len(w) - 0.6 * n_sv(U[w]) for w in in_way); _dv = sum(n_sv(U[w]) for w in in_way) - _v0
                    if L1 > L0 + _detour * (VAP_WF if (_k in VAP_IMPORTANT or _k in EPM) else 1.0): ok = False; _reason = 'Detour %.1f mm' % (L1 - L0)
                    elif _dv > 0: ok = False; _reason = '%d more vias' % _dv
                    elif CONS and sum(cons_missing(w) for w in in_way) > _f0: ok = False; _reason = 'Via group loses a via'
                    elif conflicts_group(group, groups_index(group)): ok = False; _reason = 'Conflict remains'
            print('   %s:%s %s, in the way %s: %s' % (_pp['ref'], _pp['num'], 'Ground pad %d -> %d vias' % (_n0, len(U[u]['vias'])) if _k in EPM else 'Stub cost %.1f -> %.1f' % (_g0, _g1),
                                               in_way[:5], _reason), flush=True)
            if ok:
                _nr += 1; _more += L1 - L0; _wt += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
        for w in save:
            if w in U: del_unit(w)
        for w in save: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    PF[0] = PFs; _CAND_IDX[0] = None
    print('Room at pad cleared: %d of %d stubs and ground pads improved with better room (stub cost together -%.1f), %d more ground pad vias, other tracks %.1f mm longer for it (%.0fs)' % (
        _nr, _nv, _wt, _nep, _more, time.time() - t2), flush=True)

def net_mst(n):
    pts = [(D['pads'][k]['x'], D['pads'][k]['y']) for k in by_net[n]]
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
if args.get('relayed') == '1' and not todo and not time_tight():
    t3 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _ugw = [0, 0]
    for rnd in range(int(args.get('relayed_rounds', '3'))):
        cand = []
        for u in units:
            if pair_unit(u) or u.startswith('GND#') or u not in U: continue
            m = net_mst(u); L = unit_len(u)
            if L - m > float(args.get('relayed_min', '1.5')) and L / max(m, 0.1) > 1.4: cand.append(((L - m) * (u_wt(u) if WEIGHTED else 1.0), u))
        cand.sort(reverse=True); done = 0
        for exc, u in cand:
            if time_tight(0.9): break
            pads = [D['pads'][k] for k in by_net[u]]
            bx = (min(p['x'] for p in pads) - 0.6, min(p['y'] for p in pads) - 0.6, max(p['x'] for p in pads) + 0.6, max(p['y'] for p in pads) + 0.6)
            group = [u]
            for w in list(U):
                if w == u or w.startswith(('FIX#', 'TACK#')): continue
                b2 = unit_box(w)
                if b2 and not (b2[0] > bx[2] or b2[2] < bx[0] or b2[1] > bx[3] or b2[3] < bx[1]): group.append(w)
            if PAIR_SEARCH_SPACE:
                group=sorted(set(group)|{PAIR_NETS[w] for w in group if pair_unit(w)})
            if len(group) > int(args.get('relayed_max', '40')): continue
            save = {w: copy.deepcopy(U[w]) for w in group}
            _ux = groups_index(group) if WEIGHTED else None
            L0 = sum(routing_mass(w) for w in group)
            if WEIGHTED:
                _g0 = {w: (u_wt(w), routing_mass(w), routing_mass_wt(w)) for w in group}
            for w in group: del_unit(w)
            ok = True; hsave = hist.copy(); PF[0] = 1.0
            order = [u] + sorted((w for w in group if w != u), key=grp_key)
            for w in pair_series(order):
                if not route_unit(w): ok = False
            for k_ in range(int(args.get('relayed_iter', '12'))):
                cfg = conflicts_group(group, _ux) if _ux is not None else exact_conflicts()
                if args.get('relayed_chk') == '1': assert set(cfg) == set(exact_conflicts()), 'Relayed-GroupCheck'
                bad = [w for w in cfg if w in group]
                if not bad or not ok: break
                for w, cells in cfg.items():
                    if w in group: hist[np.array(cells, dtype=np.int64)] += 1.0
                PF[0] *= 1.4
                for w in pair_series([w for w in order if w in bad]):
                    if not pair_unit(w): del_unit(w)
                    if not route_unit(w): ok = False
            PF[0] = 1e3; hist[:] = hsave
            L1 = sum(routing_mass(w) for w in group if w in U)
            _better = L1 < L0 - 0.3
            if WEIGHTED and ok:



                _better, _gw0, _gw1, _reversed = _swap_better([(_g0[w][0], _g0[w][1], routing_mass(w), _g0[w][2], routing_mass_wt(w)) for w in group])
                _ugw[0] += 1
                if _better != (L1 < L0 - 0.3):
                    _ugw[1] += 1
                    print('   reroute %-24s weighted %s: bare %.1f -> %.1f mm, weighted %.1f -> %.1f%s' % (
                        u, 'taken' if _better else 'declined', L0, L1, _gw0, _gw1, ', a more important net would get longer' if _reversed else ''), flush=True)
            if ok and _better and not (conflicts_group(group, _ux) if _ux is not None else exact_conflicts()):
                done += 1; print('   rerouted %-24s group %2d: %.1f -> %.1f mm' % (u, len(group), L0, L1), flush=True)
            else:
                for w in group:
                    if w in U: del_unit(w)
                for w in group: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
        print('Rerouting round %d: %d groups improved (%.0fs)' % (rnd + 1, done, time.time() - t3), flush=True)
        if not done: break
    PF[0] = PFs
    if WEIGHTED: print('Weighted rerouting (R21): %d complete attempts, %d of them decided differently than by bare length' % tuple(_ugw), flush=True)







SINGLE_TEST = args.get('single_test', '0') == '1'
ET_LOCATION = args.get('et_location', 'post')
def single_test():
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analysis.coupling import coupling as _coupling, sets as _kmeans
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _CAND_IDX[0] = None
    _ET_O, _ET_S = _kmeans(NC)
    _ET_WT = WT_KIND
    if _ET_WT is None:
        try:
            from common.importance import weights as _gw_et
            _ET_WT = _gw_et()
        except Exception: _ET_WT = {}
    _p_via = float(args.get('et_via', args.get('short_via_mm', '3.0'))); _p_narrow = float(args.get('et_narrow', '2.0'))
    _p_coupling = float(args.get('et_coupling', '1.0')); _et_min = float(args.get('et_min', '0.1'))
    _et_vmore = int(args.get('et_vias_more', '0')); _et_var = int(args.get('et_variants', '2')); _et_disturb = float(args.get('et_disturb', '2.0'))
    _HARD = ('current_val', 'supply', 'sw')
    def _et_kind(u): return NC['net_list'].get(U[u]['net'], {}).get('category', 'signal')
    def _et_narrow(u):
        r = U[u]; C = cls(r['net']); Cs = cls('/__signal__'); s = 0.0
        WF = neck_field_map(r['net'], C, C['wmin'], False)
        for t in r['tracks']:
            if t.get('web'): continue
            nominal = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= nominal - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                if WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < nominal: continue
                s += math.dist(a, b)
        return s
    def _et_tr(w): return [dict(net=U[w]['net'], layer=t['layer'], pts=t['pts'], w=t['w'], kelvin=bool(t.get('tap'))) for t in U[w]['tracks']]
    def _et_coupling(u):
        net = U[u]['net']; k = 0.0
        if net in _ET_O or any(t.get('tap') for t in U[u]['tracks']):
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] in _ET_S for x in _et_tr(w)])
            k += _coupling(R, NC, victim={net}, aggressor=_ET_S)['total_sum']
        if net in _ET_S:
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] != net and (U[w]['net'] in _ET_O or any(t.get('tap') for t in U[w]['tracks'])) for x in _et_tr(w)])
            k += _coupling(R, NC, aggressor={net: _ET_S[net]})['total_sum']
        return k
    def _et_mass(u):
        nv = u_vias(u); Lb = unit_len(u) - 0.6 * nv; g = _ET_WT.get(_et_kind(u), 1.0)
        pv = _p_via * (2.0 if actual_fast(U[u]['net']) else 1.0); ps = _p_narrow * (2.5 if _et_kind(u) in _HARD else 1.0)
        Ls = _et_narrow(u); K = _et_coupling(u)
        return dict(rating=g * Lb + pv * nv + ps * Ls + _p_coupling * K, L=Lb, v=nv, s=Ls, k=K, f=cons_missing(u) if CONS else 0)
    def _et_ok(u, m0, m1):
        category = _et_kind(u); hard = category in _HARD
        if m1['rating'] >= m0['rating'] - _et_min: return False
        if m1['L'] > m0['L'] * 1.10 + 1.0: return False
        if m1['v'] > m0['v'] + (0 if hard or actual_fast(U[u]['net']) else _et_vmore): return False
        if m1['s'] > m0['s'] + (1e-6 if hard else 0.3): return False
        if m1['k'] > m0['k'] + 1e-6 or m1['f'] > m0['f']: return False
        return True
    _et_units = sorted([u for u in units if u in U and not pair_unit(u) and not u.startswith(('GND#', 'FIX#', 'TACK#')) and U[u]['tracks']
                        and not any(v.get('gr') or v.get('n') for v in U[u]['vias'])], key=lambda u: (grp(u), u))
    _et_n = 0; _et_var2 = 0; _et_d = dict(rating=0.0, L=0.0, v=0, s=0.0, k=0.0); _et_det = []
    for u in _et_units:
        if time_tight(0.9): break
        old = copy.deepcopy(U[u]); m0 = _et_mass(u); best = None
        variants = [1.0] + ([_et_disturb] if _et_var >= 2 and disturb_extra(U[u]['net']) is not None else [])
        del_unit(u)
        for vi, factor in enumerate(variants):
            DISTURB_FACTOR[0] = factor; ok = route_unit(u); DISTURB_FACTOR[0] = 1.0
            if ok:
                m1 = _et_mass(u)
                if _et_ok(u, m0, m1) and (best is None or m1['rating'] < best[0]['rating']) and unit_conflict_free(u):
                    best = (m1, copy.deepcopy(U[u]), vi)
            del_unit(u)
        if best is not None:
            m1, nu, vi = best; add_unit(u, nu['net'], nu['tracks'], nu['vias']); cand_idx_changed(u)
            _et_n += 1; _et_var2 += vi
            for k_ in _et_d: _et_d[k_] += m1[k_] - m0[k_]
            _et_det.append('%s %.1f->%.1f (L %+.1f, V %+d, narrow %+.2f, coupling %+.2f%s)' % (
                u.split('/')[-1], m0['rating'], m1['rating'], m1['L'] - m0['L'], m1['v'] - m0['v'], m1['s'] - m0['s'], m1['k'] - m0['k'], ', disturbance cost x%g' % _et_disturb if vi else ''))
        else:
            add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
    PF[0] = PFs; _CAND_IDX[0] = None
    print('Single test: %d of %d units better (%d with doubled disturbance cost); metric %+.1f, track %+.1f mm, vias %+d, below minimum width %+.2f mm, coupling %+.2f (%.0fs)' % (
        _et_n, len(_et_units), _et_var2, _et_d['rating'], _et_d['L'], _et_d['v'], _et_d['s'], _et_d['k'], time.time() - t2), flush=True)
    for _z in _et_det: print('   ' + _z, flush=True)
if SINGLE_TEST and ET_LOCATION == 'before' and NC is not None and not todo and not time_tight(): single_test()
def vs_metrics(tracks, vias):
    L = 0.0; Lo = 0.0; bends = 0; short_ = 0; seg = 0
    for t in tracks:
        rv = None; fl = t['net'] in AREA_NETS
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b); L += d; seg += 1
            if not fl: Lo += d
            if d < 0.1: short_ += 1
            if d < 1e-9: continue
            r = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
            if rv is not None and rv[0] * r[0] + rv[1] * r[1] < 0.99985: bends += 1
            rv = r
    category = {}
    if NC is not None:
        for v in vias:
            a_ = 'gnd' if v['net'] in GNDS else (NC['net_list'].get(v['net']) or {}).get('category', 'signal'); category[a_] = category.get(a_, 0) + 1
    return dict(vias=len(vias), vias_without_area=sum(1 for v in vias if v['net'] not in AREA_NETS),
                current_vias=sum(1 for v in vias if v.get('current_via') or v.get('gr')), track_list=len(tracks), segments=seg,
                bends=bends, seg_under_0_1=short_, total_length=round(L, 2), length_without_area=round(Lo, 2),
                **({'vias_per_kind': dict(sorted(category.items()))} if NC is not None else {}))


def metrics_now():
    return vs_metrics([dict(net=r['net'], pts=t['pts']) for r in U.values() for t in r['tracks']],
                      [dict(v, net=r['net']) for r in U.values() for v in r['vias']])




def local_connection(pts, A, B, capL, li, net, w):
    q = [tuple(x) for x in pts]; q[0] = tuple(A); q[-1] = tuple(B)
    out = [q[0]]
    for a, b in zip(q[:-1], q[1:]):
        if math.dist(a, b) < 1e-8: continue
        def vacant(x, y): return line_cap_min(capL, x, y) >= w - 1e-6 and pad_room(li, x, y, net) >= w / 2 - 1e-6
        if not oct_net(net) or oct_(a, b):
            if not vacant(a, b): return None
            out.append(b)
        else:
            c = next((c for c in _corners(a, b) if vacant(a, c) and vacant(c, b)), None)
            if c is None: return None
            out.extend((c, b))
    return out if len(out) >= 2 else None

def ast_shift(w):
    old = copy.deepcopy(U[w]); net = old['net']; C = cls(net)
    if pair_unit(w) or w.startswith(('FIX#', 'TACK#', 'GND#')) or (NC and (NC['net_list'].get(net) or {}).get('crystal')): return False
    for ti, t in enumerate(old['tracks']):
        if t.get('tap') or t.get('web') or len(t['pts']) < 3: continue
        A, B = t['pts'][0], t['pts'][-1]; li = LI[t['layer']]; ln = _LS(t['pts'])
        inner = ln.difference(_PT(A).buffer(0.03).union(_PT(B).buffer(0.03)))
        rest = [q for j, q in enumerate(old['tracks']) if j != ti]
        contacts = [(_LS((a, b)).buffer(max(t['w']) / 2).intersection(_LS((c, d)).buffer(ww / 2)))
                    for a, b in zip(t['pts'][:-1], t['pts'][1:]) for q in rest if q['layer'] == t['layer'] for c, d, ww in zip(q['pts'][:-1], q['pts'][1:], q['w'])]
        border = max([max(t['w'])] + [max(q['w']) for q in rest]) + 0.03
        ends = _PT(A).buffer(border).union(_PT(B).buffer(border))
        if any(not g.difference(ends).is_empty for g in contacts): continue
        if any(inner.distance(_PT(v['x'], v['y'])) < VIA_D / 2 for v in old['vias']): continue
        box = (min(q[0] for q in t['pts']) - 1, min(q[1] for q in t['pts']) - 1, max(q[0] for q in t['pts']) + 1, max(q[1] for q in t['pts']) + 1)
        del_unit(w); add_unit(w, net, rest, old['vias']); NET_CUR[0] = net
        ch, vh, cs, dd, rng, vb, cc, cf, vbf = maps(net, box, C['w'])
        def node(q): return li * NN + int((q[1] - Y0) / RES) * NX + int((q[0] - X0) / RES)
        src, tgt = node(A), node(B)
        if ch[li].ravel()[src % NN] < min(t['w']) or ch[li].ravel()[tgt % NN] < min(t['w']): path = None
        else: path = astar_pf([src], {tgt}, (B[0], B[1], B[0], B[1]), cc, cs, np.zeros_like(vh), vb, C, box)
        Q = None
        if path:
            tr, vi = path_to_geometry(path, cc, C)
            if len(tr) == 1 and not vi:
                Q = local_connection(tr[0]['pts'], A, B, cc[li], li, net, min(t['w']))
        del_unit(w)
        if Q is not None:
            nt = dict(t, pts=Q, w=[min(t['w'])] * (len(Q) - 1)); new = list(old['tracks']); new[ti] = nt
            add_unit(w, net, new, old['vias'])
            if unit_conflict_free(w): return True
            del_unit(w)
        add_unit(w, net, old['tracks'], old['vias'])
    return False




























VS_LOG = []
if args.get('vias_save') == '1' and not todo and not time_tight():
    t4 = time.time(); _vs_c0 = time.process_time()
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    try: from analysis.evaluation import WT as VS_WT
    except Exception: VS_WT = {'sw': 4.0, 'current_val': 2.5, 'supply': 1.0, 'fast': 3.0, 'analog': 2.0, 'signal': 1.0}
    VS_MM = float(args.get('vs_mm', '3.0'))
    VS_VAR = [(float(x.split(':')[0]), float(x.split(':')[1]) if ':' in x else VS_MM) for x in args.get('vs_var', '1000/3').split('/') if x]
    VS_HARD = float(args.get('vs_hard', '1000')); VS_GR = int(args.get('vs_group', '6')); VS_IT = int(args.get('vs_iter', '4'))
    VS_MIN = float(args.get('vs_min', '1.0')); VS_TOL = float(args.get('vs_tol', '0.1'))
    VS_EPS = float(args.get('vs_gain', '0.5'))
    VS_BEND = float(args.get('vs_bend', '0'))
    VS_PROTECTION = args.get('vs_protection', '1') == '1'
    VS_CURRENT = args.get('vs_current', '0') == '1'
    VS_WIDTH = args.get('vs_width', '1') == '1'
    VS_RET = float(args.get('vs_ret', '1.0'))
    VS_AMPAD = float(args.get('vs_ampad', '0.5'))
    VS_PURE = args.get('vs_pure', '0') == '1'
    VS_PADS = int(args.get('vs_pads', '6'))
    VS_PATIENCE = int(args.get('vs_patience', '1'))
    VS_NEIGHBOR_VIAS = args.get('vs_neighbor_vias', '1') == '1'
    VS_HIGHER = args.get('vs_higher', '1') == '1'
    _vs_p = sum(len(by_net[u]) for u in units if not u.startswith('GND#')) + sum(1 for u in units if u.startswith('GND#'))
    VS_WORK = float(args.get('vs_work', '6')) * _vs_p; _vs_work = [0, 0.0, 0.0, 0.0]
    VS_BUDGET = max(float(args.get('vs_time_min', '30')), float(args.get('vs_time', '1.0')) * _vs_c0)
    VS_CHK = args.get('vs_chk') == '1'
    _vs_prev = {w: (r['net'], r['tracks'], r['vias']) for w, r in U.items()}
    hsave = hist.copy(); hist[:] = 0; PFs = PF[0]; _CAND_IDX[0] = None
    metric0 = metrics_now()
    def vs_vias(w): return sum(1 for v in U[w]['vias'] if not v.get('gr') and not v.get('rv'))
    def u_len(w): return sum(math.dist(a, c) for t in U[w]['tracks'] if not t.get('web') for a, c in zip(t['pts'][:-1], t['pts'][1:]))
    def u_missing(w): return cons_missing(w) if CONS else 0
    def u_bends(w):
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
    def vs_kind(w):
        if w.startswith('GND#') or w in GNDS: return 'gnd'
        if NC is not None: return (NC['net_list'].get(w) or {}).get('category', 'signal')
        return 'current_val' if cls(w)['prio'] < 8 else 'signal'
    def vs_protection(w): return pair_unit(w) or VS_PROTECTION and vs_kind(w) in ('gnd', 'current_val', 'supply', 'sw')
    def vs_current(w): return vs_kind(w) in ('gnd', 'current_val', 'supply', 'sw')
    def vs_actual_current(w): return VS_WIDTH and not w.startswith('GND#') and vs_current(w)
    def vs_pad_spacing(w):
        bbs = [D['pads'][int(w[4:])]['bbox']] if w.startswith('GND#') else [D['pads'][k]['bbox'] for k in by_net_g.get(U[w]['net'], [])]
        return sorted(min(math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3])) for b in bbs)
                      for v in U[w]['vias'] if not v.get('gr') and not v.get('rv')) if bbs else []
    def vs_pad_ok(a_, n_): return not a_ or not n_ or (max(n_) <= max(a_) + 0.05 and sum(n_) / len(n_) <= sum(a_) / len(a_) + 0.05)
    def vs_leff(net, tracks):
        C = cls(net); nominal = C['w']; s_ = 0.0
        xs_ = [q[0] for t in tracks for q in t['pts']] or [0.0]; ys_ = [q[1] for t in tracks for q in t['pts']] or [0.0]
        _cH, _, _cS, _dd, _, _, capC, _, _ = maps(net, (min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0), nominal)
        for t in tracks:
            if t.get('web'): continue
            li = LI[t['layer']]; thick = 1.0 if li in (0, NL - 1) else 2.0
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                d = math.dist(a, b)
                if d <= 0: continue
                if t.get('sig'): s_ += d; continue
                wv = max(w0, math.floor(min(nominal, line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
                s_ += d * thick * nominal / wv
        return s_
    def u_mass(w):
        if not vs_actual_current(w): return u_len(w)
        s_ = (U[w]['net'], U[w]['tracks'], U[w]['vias']); del_unit(w); v = vs_leff(s_[0], s_[1]); add_unit(w, *s_)
        return v
    _vs_m = {}
    def u_mass0(w):
        if w not in _vs_m: _vs_m[w] = u_mass(w)
        return _vs_m[w]
    def u_wt(w): return VS_WT.get(vs_kind(w), 1.0)
    def u_via_weight(w): return VS_MM * (2.0 if vs_kind(w) == 'fast' else 1.0)
    def vs_bottom(n):
        gem = None
        for k in by_net[n]:
            s_ = {L for L in D['pads'][k]['layers'] if L in LI}; gem = s_ if gem is None else gem & s_
        return 0 if gem else 1
    def vs_potential(u):
        return u_wt(u) * (u_mass0(u) + u_via_weight(u) * vs_vias(u) - net_mst(u) - u_via_weight(u) * vs_bottom(u))
    def vs_viaf(w, mm):
        if vs_protection(w): return 1.0
        return max(1.0, mm * u_via_weight(w) / VS_MM / max(cls(w)['via'], 0.05))
    def vs_pad_d(net, v):
        return min((math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3]))
                    for b in (D['pads'][k]['bbox'] for k in by_net_g.get(net, []))), default=9.0)
    def vs_mask(net, vias):
        m = np.zeros((NY, NX), dtype=bool)
        for v in vias:
            r_ = 0.0 if vs_pad_d(net, v) <= VS_AMPAD else VS_RET; rr = int(r_ / RES) + 1
            ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
            y0, y1, x0, x1 = max(0, iy - rr), min(NY, iy + rr + 1), max(0, ix - rr), min(NX, ix + rr + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            m[y0:y1, x0:x1] |= (xx - ix) ** 2 + (yy - iy) ** 2 <= (r_ / RES) ** 2
        return m.ravel()
    def vs_pad_vias_present(net, prev_v, new_v):
        return all(any(abs(v['x'] - q['x']) < 0.011 and abs(v['y'] - q['y']) < 0.011 for q in new_v) for v in prev_v if vs_pad_d(net, v) <= VS_AMPAD)
    def vs_offset(prev_tr, new_tr):
        m = 0.0
        for t in new_tr:
            old = [_LS(a['pts']) for a in prev_tr if a['layer'] == t['layer'] and len(a['pts']) >= 2]
            if not old: return None
            for q in t['pts']: m = max(m, min(l_.distance(_PT(q[0], q[1])) for l_ in old))
        return round(m, 2)
    _vs_mine = {}
    def vs_hard(net, tracks, vias):
        xs_ = [q[0] for t in tracks for q in t['pts']] + [v['x'] for v in vias]; ys_ = [q[1] for t in tracks for q in t['pts']] + [v['y'] for v in vias]
        if not xs_: return 0.0
        capH, viaH = hard_maps(net, _box_idx((min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0)) if LARGE else None)
        s_ = 0.0
        if net not in _vs_mine:
            _vs_mine[net] = {}
            for k in by_net_g.get(net, []):
                for L_, c in D['pads'][k]['bcells'].items(): _vs_mine[net].setdefault(LI[L_], set()).update(c.tolist())
        for t in tracks:
            li = LI[t['layer']]; capL = capH[li]; own_set = _vs_mine[net].get(li, ())
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                L = math.dist(a, b)
                if L <= 0: continue
                tt = np.linspace(0, 1, max(2, int(L / (RES * 0.5)) + 1))
                ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
                tight = capL[iy, ix] < w0 - 0.005
                if tight.any(): s_ += L * sum(1 for q in (iy[tight] * NX + ix[tight]).tolist() if q not in own_set) / len(tt)
        for v in vias:
            if not viaH[int((v['y'] - Y0) / RES), int((v['x'] - X0) / RES)]: s_ += VIA_D
        return s_
    def vs_fixed(w, gu=None):
        if w.startswith(('FIX#', 'TACK#')): return True
        if gu is None: return False
        if w.startswith('GND#'): return True
        if NC is not None and (NC['net_list'].get(w) or {}).get('crystal'): return True
        if len(by_net[w]) > VS_PADS and not (args.get('vs_branches') == '1' and not vs_protection(w)): return True
        return gu is not None and not VS_HIGHER and u_wt(w) > gu + 1e-9
    def vs_route(w, who):
        _t = time.process_time(); _vs_work[0] += 1 if w.startswith('GND#') else len(by_net[w])
        ok_ = (ast_shift(w) if who == 2 and w in U and args.get('vs_branches') == '1' and not vs_protection(w) else False)
        if not ok_ and not (who == 2 and args.get('vs_branches') == '1' and len(by_net[w]) > VS_PADS):
            if w in U: del_unit(w)
            ok_ = route_unit(w)
        _vs_work[who] += time.process_time() - _t
        return ok_
    def vs_attempt(u, pfw, mm, X):
        G = [u]; prior = {u: (U[u]['net'], U[u]['tracks'], U[u]['vias'])}; masks = {}
        amount = {u: (vs_vias(u), u_len(u), u_missing(u), u_mass0(u), vs_pad_spacing(u) if vs_protection(u) else None, u_bends(u) if VS_BEND else 0)}; gu = u_wt(u)
        def k0(w): return u_wt(w) * (amount[w][3] + u_via_weight(w) * amount[w][0] + VS_BEND * amount[w][5])
        def k1(w): return u_wt(w) * (u_mass(w) + u_via_weight(w) * vs_vias(w) + (VS_BEND * u_bends(w) if VS_BEND else 0.0))
        hc = []; vacant = False; history = []; reason = 'no gain of its own'
        if VS_RET > 0 and vs_protection(u): masks[u] = vs_mask(prior[u][0], prior[u][2])
        del_unit(u); PF[0] = pfw; VIAF[0] = vs_viaf(u, mm); VIA_MASK[0] = masks.get(u)
        ok = vs_route(u, 1); VIA_MASK[0] = None
        if ok and k1(u) < k0(u) - VS_EPS:
            reason = 'Conflicts remain'
            for round_no in range(VS_IT + 1):
                _t = time.process_time(); cfg = conflicts_group(G, X, set(G)); history.append(len(cfg)); _vs_work[3] += time.process_time() - _t
                if VS_CHK:
                    _v = exact_conflicts(); assert set(_v) == set(cfg), ('vias_save-Checking', sorted(set(_v) ^ set(cfg)))
                if not cfg: vacant = True; break
                if round_no == VS_IT: break
                new = [w for w in sorted(cfg) if w not in prior and not vs_fixed(w, gu)]
                if len(G) + len(new) > VS_GR: reason = 'Group too large'; break
                for w in new:
                    prior[w] = (U[w]['net'], U[w]['tracks'], U[w]['vias']); amount[w] = (vs_vias(w), u_len(w), u_missing(w), (_vs_m[w] if w in _vs_m else u_mass(w)) if vs_actual_current(w) else u_len(w), vs_pad_spacing(w) if vs_protection(w) else None, u_bends(w) if VS_BEND else 0); G.append(w)
                other = sorted((w for w in G[1:] if w in cfg), key=lambda w: vs_kind(w) == 'fast')
                if round_no >= VS_PATIENCE or not other:
                    for w, cells in cfg.items():
                        c = np.array(cells, dtype=np.int64); hist[c] += 1.0; hc.append(c)
                PF[0] = VS_HARD
                for w in other:
                    if VS_RET > 0 and vs_protection(w):
                        if w not in masks: masks[w] = vs_mask(prior[w][0], prior[w][2])
                        VIA_MASK[0] = masks[w]
                    if args.get('vs_branches') != '1' or vs_protection(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm)
                    if not vs_route(w, 2): ok = False
                    VIA_MASK[0] = None
                if u in cfg and (round_no >= VS_PATIENCE or not other):
                    PF[0] = VS_HARD if other else pfw
                    del_unit(u); VIAF[0] = vs_viaf(u, mm); VIA_MASK[0] = masks.get(u)
                    if not vs_route(u, 1): ok = False
                    VIA_MASK[0] = None
                    if ok and not k1(u) < k0(u) - VS_EPS: reason = 'no gain of its own'; break
                if not ok: reason = 'incomplete'; break
                if sum(vs_vias(w) for w in G) > sum(amount[w][0] for w in G): reason = 'Vias of the group increase'; break
        elif not ok: reason = 'incomplete'
        def hard_bad(): return [w for w in G if vs_hard(U[w]['net'], U[w]['tracks'], U[w]['vias']) > vs_hard(*prior[w]) + 0.05]
        if vacant and ok:



            new_z = list(G) if VS_PURE else hard_bad()
            if new_z:
                PF[0] = VS_HARD
                for w in new_z:
                    if VS_RET > 0 and vs_protection(w) and w not in masks: masks[w] = vs_mask(prior[w][0], prior[w][2])
                    if args.get('vs_branches') != '1' or vs_protection(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm); VIA_MASK[0] = masks.get(w)
                    if not vs_route(w, 1 if w == u else 2): ok = False
                    VIA_MASK[0] = None
                if not ok: reason = 'incomplete'
                else:
                    cfg = conflicts_group(G, X, set(G)); history.append(len(cfg))
                    if cfg: vacant = False; reason = 'Conflicts after final drawing'
                    elif hard_bad(): vacant = False; reason = 'Track would lie in blocked area (solder mask, edge, drill)'
        erg = None
        if vacant and ok:
            V0 = sum(amount[w][0] for w in G); V1 = sum(vs_vias(w) for w in G)
            L0 = sum(amount[w][1] for w in G); L1 = sum(u_len(w) for w in G)
            M1 = {w: u_mass(w) for w in G}
            K1 = {w: u_wt(w) * (M1[w] + u_via_weight(w) * vs_vias(w) + (VS_BEND * u_bends(w) if VS_BEND else 0.0)) for w in G}
            balance = sum(K1[w] - k0(w) for w in G)
            if V1 > V0: reason = 'Vias of the group increase (%d -> %d)' % (V0, V1)
            elif sum(u_missing(w) for w in G) > sum(amount[w][2] for w in G): reason = 'Via group incomplete'
            elif any(vs_protection(w) and vs_vias(w) < amount[w][0] for w in G): reason = 'protected net would lose a via'
            elif any(vs_protection(w) and not (vs_pad_ok(amount[w][4], vs_pad_spacing(w)) and vs_pad_vias_present(prior[w][0], prior[w][2], U[w]['vias'])) for w in G):
                reason = 'Via of a protected net moves away from the pad'
            elif any(vs_protection(w) and M1[w] > 1.05 * amount[w][3] + VS_TOL for w in G[1:]): reason = 'Current net would get more resistance'
            elif any(u_wt(w) > gu + 1e-9 and (vs_vias(w) > amount[w][0] or K1[w] > k0(w) + u_wt(w) * VS_TOL) for w in G[1:]):
                reason = 'more important net would get worse'
            elif not VS_NEIGHBOR_VIAS and any(vs_vias(w) > amount[w][0] for w in G[1:]): reason = 'Neighbor would get more vias'
            elif not balance < -VS_EPS: reason = 'Balance %+.1f' % balance
            else:
                speed = [w for w in G[1:] if U[w]['tracks'] != prior[w][1] or U[w]['vias'] != prior[w][2]]
                new_v = {(v['x'], v['y']) for v in U[u]['vias']}
                erg = dict(net_name=u, pf=pfw, via_mm=mm, group=G[1:], pushed=speed, vias=[V0, V1], total_length=[round(L0, 2), round(L1, 2)],
                           balance=round(balance, 2), ratio=round(-balance / (1 + sum(u_wt(w) for w in speed)), 3), rounds_n=history,
                           way=[[v['x'], v['y']] for v in prior[u][2] if not v.get('gr') and not v.get('rv') and (v['x'], v['y']) not in new_v][:6],
                           per={w: [amount[w][0], vs_vias(w), round(amount[w][1], 2), round(u_len(w), 2), vs_offset(prior[w][1], U[w]['tracks'])] for w in G},
                           _new={w: (U[w]['net'], U[w]['tracks'], U[w]['vias']) for w in G}, _prev=prior)
        for c in hc: hist[c] = 0.0
        for w in G:
            if w in U: del_unit(w)
        for w in G: add_unit(w, *prior[w])
        return erg, len(G), history, reason
    _vs_error = False
    try:
        for w in [w for w in U if vs_actual_current(w) and not w.startswith(('FIX#', 'TACK#'))]: u_mass0(w)
        X = groups_index([]); _vs_off = False; _vs_boxes = None
        for rnd in range(int(args.get('vs_rounds', '2'))):
            cand = [(grp(u)[0], -vs_potential(u), u) for u in units if u in U and not u.startswith('GND#') and not vs_fixed(u)
                    and (not vs_protection(u) or (VS_CURRENT and vs_kind(u) != 'gnd'))]
            cand = [u for _g, p_, u in sorted(cand) if -p_ >= VS_MIN]
            if _vs_boxes is not None:
                def _nearby_box(u):
                    b = unit_box(u); pb = [D['pads'][k]['bbox'] for k in by_net[u]]
                    b = (min([q[0] for q in pb] + ([b[0]] if b else [])), min([q[1] for q in pb] + ([b[1]] if b else [])),
                         max([q[2] for q in pb] + ([b[2]] if b else [])), max([q[3] for q in pb] + ([b[3]] if b else [])))
                    return any(not (b[0] > q[2] + 1.5 or b[2] < q[0] - 1.5 or b[1] > q[3] + 1.5 or b[3] < q[1] - 1.5) for q in _vs_boxes)
                cand = [u for u in cand if _nearby_box(u)]
            boxes = []; n_ok = 0; dv = 0; dl = 0.0; db = 0.0; n_supply = 0
            for u in cand:
                if _vs_work[0] > VS_WORK: _vs_off = 'Effort used up'; break
                if (not BUDGET_WORK and time.process_time() - _vs_c0 > VS_BUDGET) or time_tight(0.9): _vs_off = 'Time share used up (result thus depends on compute time)'; break
                pot = vs_potential(u); best = None
                for pfw, mm in VS_VAR:
                    erg, ng, history, reason = vs_attempt(u, pfw, mm, X); n_supply += 1
                    if args.get('vs_log') == '1' and erg is None and (ng > 1 or reason != 'no gain of its own'):
                        print('   no gain   %-24s PF %g, group %d, conflicts per round %s: %s' % (u, pfw, ng, history, reason), flush=True)
                    if erg is None: continue
                    if best is None or erg['ratio'] > best['ratio']: best = erg
                    if -erg['balance'] >= 0.9 * pot: break
                if best is None: continue
                new_ = best.pop('_new'); prev_ = best.pop('_prev'); _nb = len(boxes)
                for w in new_: del_unit(w)
                for w in new_: add_unit(w, *new_[w])
                best['round_no'] = rnd + 1; best['potential'] = round(pot, 2); VS_LOG.append(best)
                n_ok += 1; dv += best['vias'][0] - best['vias'][1]; dl += best['total_length'][1] - best['total_length'][0]; db += best['balance']
                for w in new_:
                    ta = {(t['layer'], tuple(map(tuple, t['pts']))) for t in prev_[w][1]}; tn = {(t['layer'], tuple(map(tuple, t['pts']))) for t in new_[w][1]}
                    for _l, pts in sorted(ta ^ tn): boxes.append((min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts)))
                    va = {(v['x'], v['y']) for v in prev_[w][2]}; vn = {(v['x'], v['y']) for v in new_[w][2]}
                    for x_, y_ in sorted(va ^ vn): boxes.append((x_, y_, x_, y_))
                for w in new_: _vs_m.pop(w, None)
                for w in [w for w in list(_vs_m) if w not in new_]:
                    b = unit_box(w)
                    if b and any(not (b[0] > q[2] + 2.0 or b[2] < q[0] - 2.0 or b[1] > q[3] + 2.0 or b[3] < q[1] - 2.0) for q in boxes[_nb:]): _vs_m.pop(w)
                for w in [w for w in U if vs_actual_current(w) and not w.startswith(('FIX#', 'TACK#'))]: u_mass0(w)
                X = groups_index([])
                print('   %-13s %-24s Vias %d -> %d, length %.1f -> %.1f mm, balance %+.1f (potential %.1f), PF %g, pushed %s, via gone at %s' % (
                    'Via saved' if best['vias'][1] < best['vias'][0] else 'shorter', u, best['vias'][0], best['vias'][1], best['total_length'][0],
                    best['total_length'][1], best['balance'], pot, best['pf'],
                    ['%s (%s mm)' % (w, best['per'][w][4]) for w in best['pushed']], best['way']), flush=True)
            print('Saving vias round %d: %d candidates, %d attempts, %d accepted, %d fewer vias, length %+.1f mm, balance %+.1f (%.0fs)%s' % (
                rnd + 1, len(cand), n_supply, n_ok, dv, dl, db, time.time() - t4, ', ' + _vs_off if _vs_off else ''), flush=True)
            if not n_ok or _vs_off: break
            _vs_boxes = boxes
    except Exception:
        if VS_CHK: raise
        import traceback
        traceback.print_exc(file=sys.stdout); _vs_error = True
    VIAF[0] = 1.0; VIA_MASK[0] = None; PF[0] = PFs; hist[:] = hsave; _CAND_IDX[0] = None
    if _vs_error or exact_conflicts():
        print('Saving vias: %s -> everything back to the previous state' % ('Error in post-pass (see above)' if _vs_error else 'Final check finds conflicts %s' % sorted(set((a, b) for (_l, _d, a, b, _x, _y) in PAIRS))[:6]), flush=True)
        for w in list(U): del_unit(w)
        for w, s_ in _vs_prev.items(): add_unit(w, *s_)
        VS_LOG.clear()



    for w in _vs_prev:
        if w in U: U[w] = U.pop(w)
    metric1 = metrics_now()
    print('Saving vias: vias %d -> %d (without area nets %d -> %d), bends %d -> %d, length %.1f -> %.1f mm, CPU %.0f s (candidate %.0f, partner %.0f, check %.0f), effort %d of %d connections' % (
        metric0['vias'], metric1['vias'], metric0['vias_without_area'], metric1['vias_without_area'], metric0['bends'], metric1['bends'],
        metric0['total_length'], metric1['total_length'], time.process_time() - _vs_c0, _vs_work[1], _vs_work[2], _vs_work[3], _vs_work[0], VS_WORK), flush=True)



R19_SHAPE = args.get('r19_shape', '0') == '1' and NC is not None
R19_SHAPE_STAT = dict(checked=0, rejected=0, examples=[])
def r19_shape_ok(prior, new):
    if not R19_SHAPE or (NC['net_list'].get(prior['net']) or {}).get('category') not in ('signal', 'analog', 'fast'): return True
    from analysis.r19 import shape_ok, shape_size
    R19_SHAPE_STAT['checked'] += 1
    if shape_ok(prior, new): return True
    R19_SHAPE_STAT['rejected'] += 1
    if len(R19_SHAPE_STAT['examples']) < 12:
        R19_SHAPE_STAT['examples'].append(dict(net=prior['net'], prior=shape_size(prior), new=shape_size(new)))
    return False
if HYPO_LATE and not todo and not time_tight():
    _HYPO_ON[0] = True; t2 = time.time(); PFs = PF[0]; PF[0] = 1e3; hist[:] = 0; _CAND_IDX[0] = None; _nu = 0
    _htol = float(args.get('hypo_tol', '0.01'))
    _rest = sorted([u for u in units if u in U and not pair_unit(u) and not (u.startswith('GND#') and int(u[4:]) in EPM)], key=short_key); _why = {}
    for _rd in range(2):
        _remains = []
        for u in _rest:
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); L0 = unit_len(u); _v0 = n_sv(U[u]); _f0 = cons_missing(u) if CONS else 0
            del_unit(u)
            ok = route_unit(u)
            if ok and unit_len(u) <= L0 * (1 + _htol) + 0.2 and n_sv(U[u]) <= _v0 and (not CONS or cons_missing(u) <= _f0) and unit_conflict_free(u) and r19_shape_ok(old, U[u]):
                _nu += 1; cand_idx_changed(u); continue
            _why[u] = 'incomplete' if not ok else ('longer %.1f instead of %.1f' % (unit_len(u), L0) if unit_len(u) > L0 * (1 + _htol) + 0.2 else
                         ('Vias %d instead of %d' % (n_sv(U[u]), _v0) if n_sv(U[u]) > _v0 else 'Via group or conflict'))
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u); _remains.append(u)
        _rest = _remains
        if not _rest: break
    PF[0] = PFs
    print('Reshaping: %d units reshaped, %d in old shape (%.0fs)' % (_nu, len(_rest), time.time() - t2), {u: _why[u] for u in _rest if not u.startswith('GND#')}, flush=True)
if SINGLE_TEST and ET_LOCATION != 'before' and NC is not None and not todo and not time_tight(): single_test()
if PAIR_NETS and not todo and not time_tight():
    print('Differential pairs (R29) coupled: %d' % pair_couple(), flush=True)
if RET > 0 and not todo and not time_tight():
    _rvn = rv_post_run(); print('Return vias (Q2) after negotiation: %d added, %d superfluous removed' % _rvn, flush=True)
if NC is not None and args.get('flow', '1') == '1':
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from common.flow import analysis as _fl_analysis, goal_width as _fl_goal
    _nf = 0
    for u, r in list(U.items()):
        if u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        k = NC['net_list'].get(r['net'], {})
        if k.get('category') not in (('current_val', 'supply', 'sw') if args.get('current_fix') == '1' else ('current_val', 'supply')) or not k.get('flow'): continue
        info = _fl_analysis([t for t in r['tracks']], r['vias'], [D['pads'][i] for i in by_net[r['net']]], k['flow'], endpoints=args.get('current_fix') == '1')
        if not info: continue
        C_ = cls(r['net'])
        FLOW_W[u] = {sid: _fl_goal(a_, C_['wmin'], C_['w'], float(args.get('flow_exp', '0.5'))) for sid, a_ in info['share'].items()}
        if current_model(r['net']):
            FLOW_W[u] = {sid: max(TRACK_MIN,min(NECK_W,C_['wmin']),info['total'] * a_ / (CURRENT_J * CURRENT_T[r['tracks'][sid[0]]['layer']])) for sid,a_ in info['share'].items()}
        FLOW_A[u] = info['share']
        _nf += 1
    print('Flow principle: target widths for %d current nets' % _nf, flush=True)





FINE_SHIFT_LOG = []
def via_fine_shift():
    from analysis.evaluation import rate as _vf_score, not_connected as _vf_unconnected
    from analysis.topology import topology as _vf_topo
    from analysis.coupling import coupling as _vf_coupling
    def snapshot():
        tracks, vias = export_geometry()
        return dict(tracks=tracks, vias=vias, conflicts={}, failed=[], options=args,
                    gnd_layers=GND_LAYERS, area_nets=sorted(AREA_NETS))
    def gates(R):
        e = _vf_score(D, R, NC, v3=True); tp = _vf_topo(R, D)
        is_valid = e['diagnose'] and e['conflict_list'] == 0 and e['open_items'] == 0 and not _vf_unconnected(D, R, set(NC['gnd']) | AREA_NETS)
        return is_valid, e['score_v3'], _vf_coupling(R, NC)['total_sum'], tuple(tp[k] - tp.get(k + '_pad', 0) for k in ('star', 'sharp')), tp['parallel_rest_mm'], tp['kelvin_close_mm']
    def better(a, b): return b[0] and all(y <= x + 1e-7 for x, y in zip((a[1], a[2], *a[3], a[4], a[5]), (b[1], b[2], *b[3], b[4], b[5])))
    start = time.process_time(); attempts = 0; series = list(U); _CAND_IDX[0] = None; shifted = set()
    power = [u for u in U if not u.startswith(('FIX#', 'TACK#', 'GND#')) and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply')]
    for pu in power:
        if time_tight() or (not BUDGET_WORK and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
        C = cls(U[pu]['net'])
        tight = [(LI[t['layer']], _LS((a, b)), max(C['wmin'], FLOW_W.get(pu, {}).get((ti, si), C['w'])))
               for ti, t in enumerate(U[pu]['tracks']) if not t.get('sig') and not t.get('web')
               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if w < C['w'] - 1e-6 and math.dist(a, b) > 0.3]
        for u in list(U):
            r = U[u]; k = NC['net_list'].get(r['net']) or {}
            if u.startswith(('FIX#', 'TACK#', 'GND#')) or r['net'] in GNDS or k.get('crystal') or k.get('category') in ('current_val', 'supply', 'sw', 'fast'): continue
            for vi, v in enumerate(list(r['vias'])):
                if (u, vi) in shifted or v.get('gr') or v.get('rv') or v.get('n'): continue
                A = (v['x'], v['y'])
                if any(_PT(A).distance(_LS(poly + [poly[0]])) < 0.5 or _PT(A).within(Polygon(poly))
                       for pi in by_net_g.get(r['net'], []) for polys in D['pads'][pi].get('polys', {}).values() for poly in polys): continue
                near = [(li, ln, goal) for li, ln, goal in tight if ln.distance(_PT(A)) < goal / 2 + VIA_D / 2 + CLR + 0.3]
                if not near: continue
                inc = [(ti, e) for ti, t in enumerate(r['tracks']) for e in (0, -1) if math.dist(t['pts'][e], A) < 1e-6]
                if len(inc) < 2 or any(_LS((a, b)).distance(_PT(A)) < VIA_D / 2 + w / 2
                                      for tj, t in enumerate(r['tracks']) for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))
                                      if ((tj, 0) not in inc and (tj, -1) not in inc) or ((tj, 0) in inc and si > 0 and (tj, -1) not in inc) or ((tj, -1) in inc and si < len(t['w']) - 1 and (tj, 0) not in inc)): continue
                if any(r['tracks'][ti].get('tap') or r['tracks'][ti].get('web') for ti, e in inc): continue
                old = copy.deepcopy(r); pwold = copy.deepcopy(U[pu]); before = gates(snapshot())
                for ti, e in inc:
                    q = old['tracks'][ti]['pts'][1 if e == 0 else -2]; d = math.dist(A, q)
                    for step in (0.05, 0.1, 0.2, 0.3):
                        if step >= d - 0.05 or attempts >= int(args.get('vf_attempts', '60')) or time_tight() or (not BUDGET_WORK and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
                        attempts += 1; B = (A[0] + (q[0] - A[0]) * step / d, A[1] + (q[1] - A[1]) * step / d)
                        del_unit(u); net = old['net']; Cn = cls(net); ch, vh, cs, dd, rng, vb, cc, cf, vbf = maps(net, (B[0]-2,B[1]-2,B[0]+2,B[1]+2), Cn['w'])
                        ix, iy = int((B[0]-X0)/RES), int((B[1]-Y0)/RES); new = copy.deepcopy(old); ok = 0 <= ix < NX and 0 <= iy < NY and vh[iy, ix] and not vb[iy, ix]
                        for tj, end in inc:
                            t = new['tracks'][tj]; P = t['pts'] if end == 0 else t['pts'][::-1]; W = t['w'] if end == 0 else t['w'][::-1]
                            Q = local_connection([B, P[1]], B, P[1], cc[LI[t['layer']]], LI[t['layer']], net, W[0]) if ok else None
                            if Q is None: ok = False; break
                            Q += P[2:]; ww = [W[0]] * (len(Q) - len(P) + 1) + W[1:]
                            t['pts'], t['w'] = (Q, ww) if end == 0 else (Q[::-1], ww[::-1])
                        if ok:
                            new['vias'][vi] = dict(v, x=B[0], y=B[1]); add_unit(u, net, new['tracks'], new['vias']); _CAND_IDX[0] = None; ok = unit_conflict_free(u)
                            if ok:
                                widen_unit(pu, C); _CAND_IDX[0] = None; after = gates(snapshot()); ok = unit_conflict_free(pu) and better(before, after) and current_resistance(pu) < current_resistance_prev(pwold, pu) - 1e-9
                        if ok:
                            shifted.add((u, vi)); FINE_SHIFT_LOG.append(dict(net_name=u, current_val=pu, prior=A, new=B, offset_mm=step)); cand_idx_changed(u); cand_idx_changed(pu); break
                        if u in U: del_unit(u)
                        add_unit(u, old['net'], old['tracks'], old['vias'])
                        del_unit(pu); add_unit(pu, pwold['net'], pwold['tracks'], pwold['vias']); _CAND_IDX[0] = None
                    if FINE_SHIFT_LOG and FINE_SHIFT_LOG[-1]['prior'] == A: break
                r = U[u]
    for w in series: U[w] = U.pop(w)
    _CAND_IDX[0] = None
    print('Via fine shift: %d accepted, %d attempts, CPU %.1f s' % (len(FINE_SHIFT_LOG), attempts, time.process_time()-start), flush=True)

def via_density_post_run():
    global VIA_DENSITY
    from analysis.via_density import measure as density, short, guard_nets, via_guarded
    from analysis.evaluation import rate
    from analysis.rule_metrics import assumption as rule_gate
    from analysis.contract_spec import comparison as contract_gate
    from analysis.minimax import assumption as minimax_gate
    from analysis.wobble import measure as wobble
    start = time.process_time(); series = list(U); cost = VIA_DENSITY; pf = PF[0]
    def tight():
        return time_tight(0.95) or (not BUDGET_WORK and time.process_time()-start >= float(args.get('via_density_after_cpu', '45')))
    def snapshot():
        tr, vi = export_geometry()
        return export_data(tr, vi, {}, absent=[])
    def quality(R):
        return rate(D, R, NC, v3=True, **{k: str(args.get(k, '0')).lower() in ('1', 'true')
                       for k in ('si_score', 'si_measure', 'length_gate')}), wobble(D, R, gnd=NC.get('gnd') or [])
    try:
        if tight():
            VD_AFTER_LOG.update(reason='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        base = snapshot(); vd0 = density(D, base, window=VD_WINDOW, threshold=VD_THRESHOLD)
        if not vd0['window_over'] and vd0['sharp_vias_mm2'] <= VD_THRESHOLD:
            VD_AFTER_LOG.update(reason='under_threshold', cpu_s=round(time.process_time()-start, 3)); return
        if tight():
            VD_AFTER_LOG.update(reason='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        e0, w0 = quality(base)
        protected = guard_nets(NC, base, IMP_TAB)
        if not e0['is_valid']:
            VD_AFTER_LOG.update(reason='evaluation_invalid', cpu_s=round(time.process_time()-start, 3)); return
    except Exception as exc:
        VD_AFTER_LOG.update(reason='diagnosis_error', faults=[repr(exc)[:160]], cpu_s=round(time.process_time()-start, 3)); return
    cand = []
    for u in sorted(U):
        r = U[u]
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#', 'GND#')): continue
        if not r['vias'] or any(via_guarded(dict(v, net=r['net']), NC, protected) for v in r['vias']): continue
        score = sum(float(_vd_cost()[int((v['y']-Y0)/RES), int((v['x']-X0)/RES)])
                    for v in r['vias'] if 0 <= int((v['x']-X0)/RES) < NX and 0 <= int((v['y']-Y0)/RES) < NY)
        if score: cand.append((-score, u))
    try:
        VIA_DENSITY = True; PF[0] = 1e3; _CAND_IDX[0] = None
        for _, u in sorted(cand)[:int(args.get('via_density_after_max', '8'))]:
            if tight(): break
            old = copy.deepcopy(U[u]); saved_fail = set(fail)
            saved_unreachable = copy.deepcopy(UNREACHABLE)
            VD_AFTER_LOG['attempts'] += 1
            accepted = False
            try:
                del_unit(u)
                if route_unit(u) and not tight() and unit_conflict_free(u) and not exact_conflicts():
                    new = snapshot(); q = short(new, NC, VD_WINDOW, VD_THRESHOLD)
                    if q['sharp_vias_mm2'] <= vd0['sharp_vias_mm2'] and q['vias'] <= vd0['vias']:
                        if tight(): continue
                        vd1 = density(D, new, window=VD_WINDOW, threshold=VD_THRESHOLD)
                        if (vd1['overload_integral_mm2'] < vd0['overload_integral_mm2'] - 1e-7
                                and vd1['window_over'] <= vd0['window_over']):
                            if tight(): continue
                            e1, w1 = quality(new)
                            accepted = e1['is_valid'] and rule_gate(e0.get('design_rules'), e1.get('design_rules'))['not_worse']
                            if accepted and any(e.get('gate_check') or (e.get('design_rules') or {}).get('measurement_version') for e in (e0, e1)):
                                accepted = contract_gate(e0, e1)['not_worse']
                            if accepted and any(e.get('evaluation_target') == 'minimax' for e in (e0, e1)):
                                accepted = minimax_gate(e0, e1)['not_worse']
                            accepted = accepted and w1['move_records'] <= w0['move_records']


                            accepted = accepted and not tight()
                            if accepted:
                                VD_AFTER_LOG['accepted'] += 1
                                VD_AFTER_LOG['move_records'].append(dict(net_name=u, integral=[vd0['overload_integral_mm2'], vd1['overload_integral_mm2']]))
                                vd0, e0, w0 = vd1, e1, w1
            except Exception as exc:
                VD_AFTER_LOG.setdefault('faults', []).append(repr(exc)[:160])
            finally:
                if not accepted:
                    if u in U: del_unit(u)
                    add_unit(u, old['net'], old['tracks'], old['vias'])
                    fail.clear(); fail.update(saved_fail)
                    UNREACHABLE.clear(); UNREACHABLE.update(saved_unreachable)
                _CAND_IDX[0] = None
    finally:
        VIA_DENSITY = cost; PF[0] = pf
        for u in series: U[u] = U.pop(u)
        _CAND_IDX[0] = None
        VD_AFTER_LOG['cpu_s'] = round(time.process_time()-start, 3)
    print('Via-Density-PostRun:', VD_AFTER_LOG, flush=True)


def current_resistance_prev(r, u):
    now = U[u]; U[u] = r
    try: return current_resistance(u)
    finally: U[u] = now

t1 = time.time()
PRE = {u: copy.deepcopy(r) for u, r in U.items()}
WIDE_DENSE_REPORT = None
if WIDE_DENSE:
    _CAND_IDX[0] = None
    _bd_before = exact_conflicts()
    _bd_before_n = len(PAIRS)

if 'pre' in args:
    _t, _v = [], []
    for u, r in U.items():
        for t in r['tracks']: _t.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w']))
        for v in r['vias']: _v.append(dict(net=v.get('net') or r['net'], x=v['x'], y=v['y']))
    if args.get('short_circuit_protection', '0') == '1': _t, _v = export_geometry()
    json.dump(export_data(_t, _v, {}) if args.get('short_circuit_protection', '0') == '1' else dict(tracks=_t, vias=_v), open(args['pre'], 'w')); print('saved before widening:', args['pre'])
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
WMAX = {0: 0.8, 1: 0.35, 2: 1.5, 3: 1.2, 4: 1.0, 5: 0.5, 6: 0.35} if args.get('wider') == '1' else {}
def ucls(u):
    if u.startswith('GND#'): return dict(w=max(0.35, TRACK_MIN), wmin=max(0.15, TRACK_MIN))
    if u in PLANE_GROUPS:return dict(w=max(0.35,TRACK_MIN),wmin=max(0.15,TRACK_MIN),prio=6.5)
    C = cls(u[4:] if u.startswith('FIX#') else u)
    return dict(C, w=max(C['w'], WMAX.get(C['prio'], 0)))
if args.get('fair') in ('1', 'all_items'):

    STEP = 0.05
    pw = [u for u in U if not u.startswith(('GND#', 'FIX#', 'TACK#')) and u not in FIXED_GND and u not in PLANE_GROUPS and
          ((NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw') if NC and (args.get('fair_all') == '1' or args.get('fair') == 'all_items') else ucls(u)['prio'] < 8)]
    saturated = set()
    for rnd in range(60):
        Rc = {c: chan_R(c) for c in CH}
        Rn = {u: current_resistance(u) for u in pw} if NC and (args.get('fair_all') == '1' or args.get('fair') == 'all_items') else None
        def key(u):
            if Rn is not None: return (0, -Rn[u], ucls(u)['prio'])
            k = U[u]['net'].split('/')[-1]
            return (0, -Rc[k[2:]]) if k[:2] in ('K_', 'S_', 'M_') else (1, ucls(u)['prio'])
        ch = 0
        if time_tight(0.95): break
        for u in sorted(pw, key=key):
            if u in saturated:
                U[u] = U.pop(u); continue
            C = ucls(u); before = [list(t['w']) for t in U[u]['tracks']]
            widen_unit(u, dict(C, w_step=STEP))
            if [list(t['w']) for t in U[u]['tracks']] != before: ch += 1
            else: saturated.add(u)
        if not ch: break
    print('Fairly widened in %d rounds' % (rnd + 1), {c: round(chan_R(c) * 1000, 1) for c in CH}, 'mOhm', flush=True)
    rest = [u for u in U if u not in pw and not u.startswith(('FIX#', 'TACK#')) and u not in FIXED_GND]
else:
    rest = [u for u in U if not u.startswith(('FIX#', 'TACK#')) and u not in FIXED_GND]
for u in sorted(rest, key=lambda u: (ucls(u)['prio'] if not u.startswith('GND#') else 6.5)):
    if time_tight(0.95): break
    widen_unit(u, ucls(u))
print('Widened in %.0fs' % (time.time() - t1))

for rnd in range(0 if WIDE_DENSE else 6):
    cf = exact_conflicts()
    if not cf: break
    print('   conflicts after widening', len(PAIRS), sorted(PAIRS, key=lambda x: x[1])[:4], flush=True)
    _last = {}
    for _pi, (lay, dist, ua, ub, x, y) in enumerate(PAIRS):
        if lay == '*': continue
        for _ui, u in enumerate((ua, ub)):
            if pair_unit(u) or u not in U or u not in PRE or u.startswith(('FIX#', 'TACK#')): continue
            r = U[u]; pre = PRE[u]; rad = 0.6 + 0.4 * rnd
            if NC and args.get('current_fix') == '1':
                from common.current_track import correct
                li = LI[lay]
                obst = [(g, DRC_CLR) for other, rr in U.items() if rr['net'] != r['net'] and not _fixed_pair(u, other)
                        for l2, g, _, _ in _unit_geo(other, rr) if l2 == li]
                obst += [(g, DRC_PADC) for pn, g, _ in _PADB[li] if pn != r['net']]
                if correct(r['tracks'], pre['tracks'], lay, obst): _last[u] = (_pi, _ui)
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
            if chg or rnd >= 4: _last[u] = (_pi, _ui)
    for u in sorted(_last, key=_last.get):
        r = U[u]
        tr = r['tracks'] if rnd < 5 or (NC and args.get('current_fix') == '1') else PRE[u]['tracks']
        del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    if BGA_PLAN and not _last:


        break
if WIDE_DENSE:
    _bd_after = exact_conflicts()
    _bd_under = []
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#', 'GND#')): continue
        if NC is None or (NC['net_list'].get(r['net']) or {}).get('category') not in ('current_val', 'supply', 'sw'): continue
        for ti, t in enumerate(r['tracks']):
            if t.get('sig'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                nominal = FLOW_W.get(u, {}).get((ti, si), ucls(u)['w'])
                if not current_model(r['net']): nominal *= f_inner(r['net'], LI[t['layer']])
                if w + 1e-7 < nominal:
                    _bd_under.append(dict(net_name=r['net'], layer=t['layer'], pts=[a,b], width=w,
                                          nominal=nominal, total_length=math.dist(a,b)))
    WIDE_DENSE_REPORT = dict(phase='direct_after_widen',
        conflict_pairs_before=_bd_before_n, conflict_pairs_after=len(PAIRS),
        conflict_units_before=len(_bd_before), conflict_units_after=len(_bd_after),
        under_goal_width=_bd_under)
    print('Dense widened: pairs %d -> %d, units %d -> %d; %d current pieces below target width' % (
        _bd_before_n, len(PAIRS), len(_bd_before), len(_bd_after), len(_bd_under)), flush=True)
if R32:

    for u in list(U):
        r = U[u]
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#')) or r['net'] in IMP_TAB or (NC['net_list'].get(r['net']) or {}).get('category') not in ('signal', 'fast', 'analog'): continue
        widths = [w for t in r['tracks'] for w in t['w']]
        goal = max(TRACK_MIN, min(widths)) if widths else TRACK_MIN
        if widths and any(abs(w - goal) > 1e-9 for w in widths):

            tr = [dict(t, w=[goal] * len(t['w'])) for t in r['tracks']]
            del_unit(u); add_unit(u, r['net'], tr, r['vias'])

if QUIET:
    _bw = [0, 0, 0]
    for u in list(U):
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#')): U[u] = U.pop(u); continue
        _bw = [a + b for a, b in zip(_bw, width_calm(u, ucls(u)))]
    print('Width profile smoothed (quiet_r=%g): %d tracks narrower, width changes %d -> %d' % (QUIET_R, _bw[2], _bw[0], _bw[1]), flush=True)

if PAD_CENTER:
    _ns = joints_repair() if args.get('joint_fix', '1') == '1' else 0
    if _ns: print('Joints (R24): %d track ends set exactly on the centerline of their own track at the end' % _ns, flush=True)
    _net_copper = collinear_together() if args.get('collinear', '1') == '1' else 0
    print('Joints (R24): %d collinear support points merged; while routing (all passes)' % _net_copper, JOINT_STAT, flush=True)


if args.get('via_fine_shift') == '1' and NC and not fail and not exact_conflicts() and not time_tight(): via_fine_shift()
LENGTHS_REPORT = None
if (args.get('length_list', '0') == '1' and SI_FIELDS or PAIR_NETS) and (PAIR_SEARCH_SPACE or not fail and not exact_conflicts()) and not time_tight(0.95):
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if PAIR_NETS: pair_couple()
    LENGTHS_REPORT = lengths_match(only_pairs=args.get('length_list', '0') != '1')
    print('Length matching (option): %d meanders, %d attempts' % (len(LENGTHS_REPORT['move_records']), LENGTHS_REPORT['attempts']), flush=True)

VD_AFTER_LOG = dict(attempts=0, accepted=0, move_records=[], cpu_s=0.0)
if VD_AFTER and not fail and not exact_conflicts() and not time_tight(): via_density_post_run()

if CURRENT_FIRST:


    for u,r in list(U.items()):
        if current_model(r['net']) and not u.startswith(('FIX#','TACK#')) and not current_final(u,ucls(u)): fail.add(u)

REST_REPORT = None
if args.get('remaining_conflicts', '0') == '1':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.remaining_conflicts import solve_vias
    REST_REPORT = solve_vias(globals())
    print('R45: %d via moves accepted, %.2f CPU-s' % (REST_REPORT['accepted'], REST_REPORT['cpu_s']), flush=True)


def pin_access_rule_values(net):
    from analysis.coupling import coupling, sets
    from analysis.topology import topology
    tracks, vias = export_geometry()
    own = [t for t in tracks if t['net'] == net]
    victim, aggressor = sets(NC)
    k = 0.
    if net in victim or any(t.get('kelvin') for t in own):
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and t['net'] in aggressor])
        k += coupling(r, NC, victim={net}, aggressor=aggressor)['total_sum']
    if net in aggressor:
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and (t['net'] in victim or t.get('kelvin'))])
        k += coupling(r, NC, victim=victim - {net}, aggressor={net: aggressor[net]})['total_sum']
    r = dict(tracks=own, vias=[v for v in vias if v['net'] == net], clr=DRCC)
    t = topology(r, dict(D, pads=[p for p in D['pads'] if p['net'] == net]))
    keys = ('parallel_rest_mm', 'star', 'sharp', 'kelvin_close_mm', 'tap_via_in_pad')
    return dict(coupling=k, **{key:t[key] for key in keys},
                star_free=t['star']-t['star_pad'], sharp_free=t['sharp']-t['sharp_pad'])


def pin_access_rule_assumption(before, post):
    return all(math.isfinite(post[k]) and post[k] <= v + 1e-6 for k,v in before.items())

def pin_access_post_run():
    global _LOOP_DEADLINE
    report = dict(repair=[], attempts=[], accepted=0, cpu_s=0., budget_s=0.)
    cpu_start = time.process_time()
    start = budget_clock()
    budget = max(0., start / 9.)
    if BUDGET_WORK: budget = min(budget, max(0., .90 * WORK_MAX - work_total()))
    elif CPU_MAX > 0: budget = min(budget, max(0., .90 * CPU_MAX - search_cpu()))
    report['budget_work' if BUDGET_WORK else 'budget_s'] = budget

    if budget < .1: return report
    old_deadline = _LOOP_DEADLINE
    deadline = start + budget
    old_hist, old_pf, old_hard = hist.copy(), PF[0], HARD_FOREIGN[0]
    old_active = _PZ_BUDGET_ACTIVE[0]
    try:
        hist[:] = 0; PF[0] = 1e3; HARD_FOREIGN[0] = True
        _PZ_BUDGET_ACTIVE[0] = True; _LOOP_DEADLINE = deadline

        removed = []
        if PZ_REST:
            _loop_cpu_check()
            conflicts = exact_conflicts()
            tr, vi = export_geometry()
            safe = r44_export(dict(tracks=tr, vias=vi, failed=sorted(fail), conflicts=conflicts))
            removed = (safe.get('short_circuit_protection') or {}).get('units_removed', [])


            removed = [u for u in removed if u in U and u not in FIXED and not u.startswith(('FIX#','TACK#'))]
            for u in removed:
                del_unit(u); fail.add(u)
            _CAND_IDX[0] = None


            order = sorted(removed,key=lambda u:(-len(by_net.get(u,[])),u))
            if order:
                order = order[:1] + sorted(order[1:],key=lambda u:(u.startswith('GND#'),len(by_net.get(u,[])),cls(U[u]['net'] if u in U else u).get('prio',10),u))
            repair_end = start + budget * .55
            for index, u in enumerate(order):
                if budget_clock() >= repair_end: break
                _LOOP_DEADLINE = min(repair_end,budget_clock()+budget*(.20 if index==0 else .07))
                entry = dict(unit=u, taken=False)
                local_before = {n:set(ps) for n,ps in PZ_LOCAL.items()}
                _PZ_PARTIAL[0] = None
                design_rules0 = None
                try:
                    net = D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
                    design_rules0 = pin_access_rule_values(net)
                    ok = route_unit(u); candidate = U.get(u)
                    valid = bool(candidate and (candidate['tracks'] or candidate['vias']))
                    valid = valid and _pz_geometry_ok(candidate['net'], candidate['tracks'], candidate['vias'], False, u)
                    _CAND_IDX[0] = None
                    valid = valid and unit_conflict_free(u)
                    if valid:
                        design_rules1 = pin_access_rule_values(net)
                        entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                        valid = pin_access_rule_assumption(design_rules0, design_rules1)
                        if not valid: entry['reason'] = 'Topology/coupling worse'
                    _loop_cpu_check()
                    if valid:
                        entry.update(taken=True, complete=bool(ok))
                        if ok: fail.discard(u)
                except (TimeoutError, _PZBudget):
                    entry['reason'] = 'CPU-Budget'


                    partial = _PZ_PARTIAL[0]
                    _LOOP_DEADLINE = repair_end
                    try:
                        _loop_cpu_check()
                        if partial and partial[0] == u and partial[2]:
                            add_unit(u,partial[1],partial[2],partial[3])
                            candidate = U[u]; _CAND_IDX[0] = None
                            valid = _pz_geometry_ok(candidate['net'],candidate['tracks'],candidate['vias'],False,u)
                            valid = valid and unit_conflict_free(u)
                            if valid and design_rules0 is not None:
                                design_rules1 = pin_access_rule_values(net)
                                entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                                valid = pin_access_rule_assumption(design_rules0, design_rules1)
                                if not valid: entry['reason'] = 'Topology/coupling worse'
                            else: valid = False
                            _loop_cpu_check()
                            if valid: entry.update(taken=True,complete=False,subtree=True)
                    except Exception as error:
                        entry['checking'] = repr(error)[:180]
                except Exception as error:
                    entry['reason'] = 'Candidate rejected: ' + repr(error)[:180]
                finally:
                    _PZ_PARTIAL[0] = None
                    if not entry['taken']:
                        del_unit(u)
                        PZ_LOCAL.clear(); PZ_LOCAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    report['repair'].append(entry); _CAND_IDX[0] = None
                if budget_clock() >= repair_end: break
        _LOOP_DEADLINE = deadline


        if PZ_MINI and (removed or PZ_LOCAL):
            eligible = [u for u in units if u in U and u not in FIXED and u not in fail
                        and not u.startswith(('GND#','FIX#','TACK#')) and not pair_unit(u)
                        and u not in PAIR_NETS and U[u]['net'] in PZ_WIDTHS and net_mst(U[u]['net']) > .000001]
            for u in sorted(eligible, key=lambda u: (-(unit_len(u)-.6*n_sv(U[u]))/net_mst(U[u]['net']), u))[:5]:
                _loop_cpu_check()
                old = copy.deepcopy(U[u]); l0 = sum(math.dist(a,b) for t in old['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:]))
                entry = dict(net_name=old['net'], before_mm=l0, q_before=l0/net_mst(old['net']), taken=False)
                local_before = {n:set(ps) for n,ps in PZ_LOCAL.items()}
                try:
                    design_rules0 = pin_access_rule_values(old['net'])
                    del_unit(u)
                    ok = route_unit(u); candidate = U.get(u)
                    l1 = sum(math.dist(a,b) for t in candidate['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) if candidate else float('inf')
                    entry['after_mm'] = l1 if math.isfinite(l1) else None
                    ok = ok and l1 < l0-.01 and n_sv(candidate) <= n_sv(old)
                    ok = ok and min((w for t in candidate['tracks'] for w in t['w']),default=TRACK_MIN) >= min((w for t in old['tracks'] for w in t['w']),default=TRACK_MIN)-1e-9
                    ok = ok and _pz_geometry_ok(old['net'],candidate['tracks'],candidate['vias'],False,u)
                    _CAND_IDX[0] = None
                    ok = ok and unit_conflict_free(u)
                    if ok:
                        from analysis.evaluation import not_connected as _pz_unconnected
                        b = dict(D,pads=[p for p in D['pads'] if p['net']==old['net']])
                        r = dict(tracks=[dict(t,net=old['net']) for t in candidate['tracks']],
                                 vias=[dict(v,net=v.get('net') or old['net'],d=VIA_D,drill=VIA_DR,layers=LAYERS) for v in candidate['vias']])
                        ok = not _pz_unconnected(b,r,set())
                    if ok:
                        design_rules1 = pin_access_rule_values(old['net'])
                        entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                        ok = pin_access_rule_assumption(design_rules0, design_rules1)
                        if not ok: entry['reason'] = 'Topology/coupling worse'
                    _loop_cpu_check()
                    if ok: entry['taken']=True; report['accepted']+=1
                except (TimeoutError, _PZBudget):
                    entry['reason']='CPU-Budget'
                except Exception as error:
                    entry['reason']='Candidate rejected: ' + repr(error)[:180]
                finally:
                    if not entry['taken']:
                        del_unit(u); add_unit(u,old['net'],old['tracks'],old['vias'])
                        PZ_LOCAL.clear(); PZ_LOCAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    _CAND_IDX[0]=None; report['attempts'].append(entry)
                if entry.get('reason')=='CPU-Budget': break
    except (TimeoutError, _PZBudget):
        report['abort']='CPU-Budget'
    finally:
        hist[:]=old_hist; PF[0]=old_pf; HARD_FOREIGN[0]=old_hard
        _PZ_BUDGET_ACTIVE[0]=old_active; _LOOP_DEADLINE=old_deadline
        report['cpu_s']=time.process_time()-cpu_start
        if BUDGET_WORK: report['work'] = work_total()-start
        report['search_kernel']='atomic search jobs' if BUDGET_WORK else 'interruptible Python core'
    return report


PZ_MINIMAX = dict(attempts=[], accepted=0)
work_mark('before_pin_access')
if PZ_REST or PZ_MINI:
    PZ_MINIMAX = pin_access_post_run()
    print('Pin access minimax:', PZ_MINIMAX, flush=True)


if PZ_LOCAL:
    for u,r in list(U.items()):
        if u in FIXED or u.startswith(('FIX#','TACK#')) or not _pz_active(r['net']):continue
        if not _pz_geometry_ok(r['net'],r['tracks'],r['vias'],False,u):
            del_unit(u);add_unit(u,r['net'],[],[]);fail.add(u)

_cf_final = exact_conflicts()
if _ET and not _LEARN_ON: _ET.observe(_cf_final, MAXIT + 1, 'post_run')
if _LEARN_ON: negotiation_measure(MAXIT + 1, _cf_final, 'final')
conf = {u: len(v) for u, v in _cf_final.items()}
print('End: conflicts', conf, 'incomplete', sorted(fail))
json.dump(PAIRS, open(OUT.replace('.json', '_conflicts.json'), 'w'))
tracks, vias = export_geometry()
if PAIR_SEARCH_SPACE: print('Shared pair search space:', _PAIR_SEARCH_INFO, flush=True)
if TAP_ON or TAP_TREE or BRANCH: print('Topology (R27, total over all passes):', TOPO_STAT, flush=True)

VGB = []
for u, r in U.items():
    for v in r['vias']:
        if v.get('n'):
            actual = 1 + sum(1 for w in r['vias'] if w.get('too') == [v['x'], v['y']])
            VGB.append(dict(net_name=r['net'], x=v['x'], y=v['y'], nominal=v['n'], actual=actual))
RVB = rv_report()
PAB = pairs_report(receiver=True) if PAIR_NETS else []
if PAB: print('Differential pairs (R29): %s' % ['%s/%s %s/%s mm, skewed %s, vias %s, coupled %.0f %%' % (
    b['net_list'][0].split('/')[-1], b['net_list'][1].split('/')[-1], b['total_length'][0], b['total_length'][1], b['skewed_mm'], b['vias'], 100 * b['coupled']) for b in PAB], flush=True)
if RVB: print('Return vias (Q2): %d layer changes of fast nets with different ground reference area, %d with ground via in reach (%d own return via), %d without %s; %s' % (
    RVB['change'], RVB['with_ret'], RVB['own_ret_via'], len(RVB['without']), [(b['net_name'], b['spacing']) for b in RVB['without'][:6]], RVB['stat']), flush=True)
if VGB: print('Via groups (K10): %d layer changes with current, %d extra vias, too little room at %d spots' % (
    len(VGB), sum(1 for r in U.values() for v in r['vias'] if v.get('gr')), sum(1 for b in VGB if b['actual'] < b['nominal'])), flush=True)
if CHAIN and CHAINS:
    _kb = [t for r in U.values() for t in r['tracks'] if t.get('link_chain')]
    print('Supply chains (R22): %d tracks as links, %d pads without chain or without keepout zone (no way)' % (len(_kb), len(_METRIC_OFF)), sorted(_METRIC_OFF), CHAIN_STAT, flush=True)

UNDER = under_wmin()
if UNDER:
    _uf = [e for e in UNDER if not e['allowed']]
    print('Below minimum width: %d pieces (%.1f mm) in %d nets%s; plus %d allowed in neck or dense field (under_wmin in the output)' % (
        len(_uf), sum(e['total_length'] for e in _uf), len({e['net_name'] for e in _uf}),
        ', narrowest %.3f instead of %.3f mm (%s)' % min((e['width'] - e['nominal'], e['width'], e['nominal'], e['net_name']) for e in _uf)[1:] if _uf else '',
        len(UNDER) - len(_uf)), flush=True)
if UNREACHABLE: print('Unreachable (hard obstacles, pad has no exit):', {n: v for n, v in sorted(UNREACHABLE.items())})

MPV = []
for _e in EPG:
    _actual = sum(1 for r in U.values() if r['net'] == _e['net_name'] for v in r['vias'] if _e['poly'].contains(_PT(v['x'], v['y'])))
    _ges = False
    if any(_k in EPM for _k in _e['pads']):
        _ges = not any(via_hard(_e['net_name'], EPM[_k]['grid_cells']) for _k in _e['pads'] if _k in EPM)
    MPV.append(dict(part=_e['ref'], pad=_e['num'], net_name=_e['net_name'], surface=_e['surface'], fields=_e['fields'], nominal=_e['nominal'], actual=_actual,
                    **({'reason': 'blocked'} if _ges else {})))
if MPV: print('Ground pad vias (R16): %d ground pads, target %d vias, actual %d, at %d pads fewer than target %s' % (
    len(MPV), sum(m['nominal'] for m in MPV), sum(m['actual'] for m in MPV), sum(1 for m in MPV if m['actual'] < m['nominal']),
    ['%s:%s %d/%d%s' % (m['part'], m['pad'], m['actual'], m['nominal'], ' (blocked: no allowed room in pad)' if m.get('reason') else '') for m in MPV]), flush=True)


VAPB = []
if VAP:
    _count = dict(in_pad=0, third=0, beside=0)
    for u, r in U.items():
        if not u.startswith('GND#') or int(u[4:]) in EPM or not r['vias']: continue
        _k = int(u[4:]); _c, _w = _vap_cells(_k); _v = r['vias'][0]
        _vc = int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES); _j = np.flatnonzero(_c == _vc)
        if len(_j): _count['in_pad' if _w[_j[0]] == 0.0 else 'third'] += 1; continue
        _count['beside'] += 1; _p = D['pads'][_k]
        _hard = via_hard(r['net'], _c)
        VAPB.append(dict(part=_p['ref'], pad=_p['num'], net_name=r['net'], spacing=round(math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']), 3),
                         reason='occupied' if _hard else 'blocked', important=_k in VAP_IMPORTANT))
    print('Via at pad (R9): %d stubs with via from half inside the pad, %d from one third, %d beside it (of which %d blocked, %d occupied) %s' % (
        _count['in_pad'], _count['third'], _count['beside'], sum(1 for b in VAPB if b['reason'] == 'blocked'),
        sum(1 for b in VAPB if b['reason'] == 'occupied'), ['%s:%s %s' % (b['part'], b['pad'], b['reason']) for b in VAPB]), flush=True)
BGB = []
if BGA_FANOUT:
    for p in D['pads']:
        b = BGA.get(id(p))
        if b is None or not p['net'] or p['net'].startswith('unconnected'): continue

        near = [v for v in vias if v['net'] == p['net'] and math.dist((v['x'], v['y']), (p['x'], p['y'])) <= b['pitch'] * 0.72]
        BGB.append(dict(ref=b['ref'], pad=b['pad'], ring=b['ring'], pitch=b['pitch'],
                        via_close=bool(near), via_in_pad=any(math.dist((v['x'], v['y']), (p['x'], p['y'])) < 0.02 for v in near)))


_rest_final = export_data(tracks, vias, conf, raw_data=True,
    **({'pair_search_space_info': _PAIR_SEARCH_INFO} if PAIR_SEARCH_SPACE else {}),
    **({'final': 'early_abort' if _EARLY_INFO is not None else ('cpu_budget' if time_tight() else 'finished'),
        'learning_state': json.loads(_LEARN_SNAPSHOT)['learning_state'], 'learning_budget':json.loads(_LEARN_SNAPSHOT)['learning_budget']} if _LEARN_ON else {}))
if args.get('remaining_conflicts') == '1' and args.get('rest_parts', '1') == '1' and REST_REPORT and NC and args.get('netclasses'):
    import tempfile
    from postprocessing.remaining_conflicts import parts_post_run, repair_keepout
    _rest_budget = max(0, float(args.get('rest_cpu', '30')) - REST_REPORT['cpu_s'])
    if CPU_MAX > 0: _rest_budget = min(_rest_budget, max(0, CPU_MAX-search_cpu()))
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter_r45_final_') as _rest_tmp:
            _rp = Path(_rest_tmp) / 'routes.json'
            _bp = Path(_rest_tmp) / 'board.json'
            _rp.write_text(json.dumps(_rest_final)); _bp.write_text(Path(sys.argv[1]).read_text())
            _rest_block = repair_keepout(globals().get('ST') or {})
            if _rest_block:

                _rest_budget = 0
            _rp.with_name('routes_conflicts.json').write_text(json.dumps(PAIRS))
            _rest_parts = parts_post_run(str(_bp), args['netclasses'], str(_rp), _rest_budget)
            if _rest_block:
                _rest_parts['reason'] = _rest_block
            _rest_final = json.loads(_rp.read_text()); _rest_final['rest_parts'] = _rest_parts
        print('R45 parts: %d moves, %.2f CPU-s' % (_rest_parts['accepted'], _rest_parts['cpu_s']), flush=True)
    except Exception as _rest_error:
        _rest_final['rest_parts'] = dict(accepted=0, reason='Check failed: '+repr(_rest_error)[:180])
_safe = rest_final(_rest_final)
if args.get('skew', '0') == '1':
    _safe = skew_match(D, _safe, args, Path(OUT).with_suffix('.skew'))

work_emergency_stop()
if WORK_MEASURE:
    work_mark('finished')
    _safe['work_budget'] = work_evidence()
if CHAMPION_EVIDENCE is not None: _safe['champion'] = CHAMPION_EVIDENCE
json.dump(_safe, open(OUT, 'w'))
if _safe.get('rest_board'):
    Path(OUT).with_name(Path(OUT).stem+'_rest_board.json').write_text(json.dumps(_safe['rest_board']))
    Path(OUT).with_name(Path(OUT).stem+'_rest_placement.json').write_text(json.dumps(_safe['rest_placement']))
print('Tracks', len(tracks), 'Vias', len(vias), 'Time %.0fs' % (time.time() - t_start))
if _ET: _ET.write_out(_safe['tracks'], _safe['vias'], absent=_safe['failed'], zone_list=_safe.get('zone_list', []) + _safe.get('zones', []))
if _safe.get('short_circuit_protection'):
    print('R44: output %d tracks, %d vias, open %s' % (len(_safe['tracks']), len(_safe['vias']), _safe['short_circuit_protection']['nets_removed']), flush=True)
if 'png' in args:
    _png_export = args.get('short_circuit_protection', '0') == '1' or args.get('skew', '0') == '1'
    for li, L in enumerate(LAYERS):
        img = np.zeros((NY, NX, 3), dtype=np.uint8); img[inside] = (235, 235, 230)
        if li in (0, NL - 1): img[PD1[li] <= 0.0] = (150, 150, 150)
        rng = np.random.RandomState(1); cols = rng.randint(40, 220, size=(len(nets) + 1, 3)); [cols.__setitem__(NID[_g], (60, 60, 60)) for _g in GNDS if _g in NID]
        if not _png_export:
            for u, r in U.items():
                c = r['cells'][(r['cells'] >= li * NN) & (r['cells'] < (li + 1) * NN)] - li * NN
                img.reshape(-1, 3)[c] = cols[NID[r['net']]]
        in_ = Image.fromarray(img).resize((NX * 2, NY * 2), Image.NEAREST)
        dr = ImageDraw.Draw(in_)
        xy = lambda p: ((p[0]-X0)/RES*2, (p[1]-Y0)/RES*2)
        for t in (_safe['tracks'] if _png_export else []):
            if t['layer'] != L: continue
            c = tuple(int(q) for q in cols[NID[t['net']]])
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']): dr.line([xy(a),xy(b)], fill=c, width=max(1,round(w/RES*2)))
        for v in (_safe['vias'] if _png_export else []):
            x,y=xy([v['x'],v['y']]); r=VIA_R/RES*2
            dr.ellipse([x-r,y-r,x+r,y+r],fill=(40,40,40))
        for e in _safe.get('ratsnest', []): dr.line([xy(p) for p in e['pts']],fill=(230,120,20),width=1)
        if not _png_export:
            for (lay, dist, ua, ub, x, y) in PAIRS_LAST:
                if lay not in (L, '*'): continue
                cx, cy = (x - X0) / RES * 2, (y - Y0) / RES * 2
                dr.ellipse([cx - 14, cy - 14, cx + 14, cy + 14], outline=(255, 0, 0), width=3)
        in_.save(args['png'] + '_' + L + '.png')

if _safe.get('rest_board') and 'png' in args and NC:
    from analysis.evaluation import image
    image(_safe['rest_board'], _safe, NC, args['png'])
if BEND > 0 and NB is not None:
    print('Bend search: %d calls, %d removals, %d dominated, %.6f CPU-s' %
          (_BEND_CALLS, _BEND_POP, _BEND_DOM, _BEND_CPU), flush=True)
