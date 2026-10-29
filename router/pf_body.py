from shapely.geometry import LineString as _LS, Point as _PT
import shapely

_ZP_GEO = {}; _ZP_CACHE = {}; _ZP_VIA_MASK = {}; _ZP_VIAS = []; _ZP_DRILL = None
_ZP_POT_NETS = set()
if ZONE_PLAN:
    from learning.current_zones import _geo as _zp_geo
    from shapely.ops import unary_union as _zp_union
    if ZONE_PLAN.get('audit', {}).get('stage') == 3:
        for _label in ZONE_PLAN['reference_protection']['gnd_layers']:
            _layer, *_names = _label.split('=', 1)
            if _names and _layer in ZONE_PLAN.get('potential_layers', []):
                _ZP_POT_NETS.update(n for n in _names[0].split('|')
                                    if ZONE_PLAN['net_list'].get(n, {}).get('chosen'))
    for _z in ZONE_PLAN['core_set']:
        _ZP_GEO.setdefault((_z['net'], _z['layer']), []).append(_zp_geo(_z))
    _ZP_VIAS = [v for f in ZONE_PLAN['via_candidates'] if f['taken'] for v in f['vias']]
    for _v in _ZP_VIAS:
        for _L in _v['layers']:
            _ZP_GEO.setdefault((_v['net'], _L), []).append(_PT(_v['x'], _v['y']).buffer(_v['d']/2))
    _ZP_GEO = {key:_zp_union(gs) for key,gs in _ZP_GEO.items()}
    if _ZP_VIAS:
        _zx, _zy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        _ZP_DRILL = np.full((NY,NX), 9.0, dtype=np.float32)
        for _v in _ZP_VIAS:
            _ZP_DRILL = np.minimum(_ZP_DRILL, np.hypot(_zx-_v['x'],_zy-_v['y'])-_v['drill']/2)


def _zp_maps(net):
    if net not in _ZP_CACHE:
        if len(_ZP_CACHE)>=4: del _ZP_CACHE[next(iter(_ZP_CACHE))]
        xx,yy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        hard=[]; soft=[]
        for L in LAYERS:
            g = _zp_union([g for (n,l),g in _ZP_GEO.items() if n!=net and l==L])
            mask = shapely.intersects_xy(g.buffer(QM),xx,yy)
            hard.append(((_edt(~mask)*RES-2*QM) if mask.any() else np.full((NY,NX),9.0)).astype(np.float32))
            growth = _zp_union([_zp_geo(z) for z in ZONE_PLAN['growth'] if z['net']!=net and z['layer']==L])
            soft.append(np.where(shapely.intersects_xy(growth,xx,yy),2.,1.).astype(np.float32))
        _ZP_CACHE[net] = (hard,np.concatenate([s.ravel() for s in soft]))
    return _ZP_CACHE[net]


def _zp_hard(net, ch, li, ys=slice(None), xs=slice(None)):
    if not ZONE_PLAN: return ch
    spec=ZONE_PLAN['net_list'].get(net,{})
    if spec.get('local_without_layer_change') and LAYERS[li]!=spec.get('local_layer'):
        return np.full(ch.shape,-1.,dtype=np.float32)
    return np.minimum(ch,2*(_zp_maps(net)[0][li][ys,xs]-CLR-.005))


def _zp_viah(net, v, ys=slice(None), xs=slice(None)):
    if not ZONE_PLAN: return v
    if ZONE_PLAN['net_list'].get(net,{}).get('local_without_layer_change'):
        return np.zeros(v.shape,dtype=bool)
    if ZONE_PLAN.get('corridors') or net in _ZP_POT_NETS:
        if net not in _ZP_VIA_MASK:
            xx,yy=np.meshgrid(X0+np.arange(NX)*RES,Y0+np.arange(NY)*RES)
            legal=np.ones((NY,NX),dtype=bool)
            for corridor in ZONE_PLAN['corridors']:
                if corridor['net']==net:continue
                area=_ZP_GEO[(corridor['net'],corridor['layer'])]
                forbidden=shapely.intersects_xy(area.buffer(VIA_D/2+CLR+QM),xx,yy)
                allowed=np.zeros((NY,NX),dtype=bool)
                for slot in ZONE_PLAN['via_window']:
                    if slot['net']==corridor['net'] and slot['layer']==corridor['layer'] and VIA_D<=slot['max_via_d']+1e-9:
                        allowed |= np.hypot(xx-slot['x'],yy-slot['y'])<=slot['radius_mm']
                legal &= ~forbidden | allowed
            if net in _ZP_POT_NETS:
                area=_zp_union([_zp_geo(z) for z in ZONE_PLAN['core_set']
                                if z['net']==net and z.get('corridor')
                                and z['layer'] in ZONE_PLAN['potential_layers']])
                legal &= shapely.intersects_xy(area.buffer(-VIA_D/2-QM-.002),xx,yy)
            if len(_ZP_VIA_MASK)>=4:del _ZP_VIA_MASK[next(iter(_ZP_VIA_MASK))]
            _ZP_VIA_MASK[net]=legal
        v = v & _ZP_VIA_MASK[net][ys,xs]
    if _ZP_DRILL is not None:
        v = v & (_ZP_DRILL[ys,xs] >= VIA_DR/2+H2H+.004)
    return v

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NB = None
if args.get('numba', '1') == '1':
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
        from router.astar_nb import Search as _Search
        from geometry_nb import line_cap_min_kernel as _linecap_nb
        NB = _Search(NL, NN, RES)
    except ImportError:
        NB = None
CLR_PAD = DRCP + 0.012
use = np.zeros(NL * NN, dtype=np.int16)
drill_use = np.zeros(NN, dtype=np.int16)
hist = np.zeros(NL * NN, dtype=np.float32)
if args.get('hist_start'):
    from learning.self_imitation import load_data as _hist_load
    hist[:] = _hist_load(args['hist_start'], args.get('hist_context', ''), LAYERS, [X0, Y0], NX, NY, RES)
    print('Self-imitation: conflict map loaded, max hist', float(hist.max()))
U = {}
CURRENT_AUDIT = {}
_SZ_VERSION = [0]; _SZ_RESERVE = [None]; _SZ_FLOOR = {}


def current_model(net):
    if not CURRENT_FIRST: return None
    k = NC['net_list'].get(net) or {}; f = k.get('flow') or {}
    if k.get('category') not in ('current_val', 'supply', 'sw'): return None
    if f.get('unit') != 'A' or not f.get('source') or not f.get('provenance'): return None
    ls = f.get('load') or {}
    if not ls or not all(math.isfinite(float(v)) and float(v) >= 0 for v in ls.values()): return None
    if sum(map(float, ls.values())) <= 0: return None
    return f


def current_plan(net, pads, trunk, KT):
    fl = current_model(net)
    if not fl: return None
    pk = {i: ('PAD', p['ref'], p['num']) for i, p in enumerate(pads)}
    names = {'%s:%s' % (p['ref'], p['num']): pk[i] for i, p in enumerate(pads)}
    if set(fl['source']) - set(names) or set(fl['load']) - set(names):
        raise ValueError('current_first: flow pad missing at ' + net)
    source = set(fl['source'])
    if any(float(fl['load'].get(q, 0)) > 0 for q in source):
        raise ValueError('current_first: source is also load at ' + net)


    planned = set(trunk) | {i for c in KT or [] for i in c['pins'] + c['caps']}
    required = set(fl['load']) | source
    if required - {'%s:%s' % (pads[i]['ref'], pads[i]['num']) for i in planned}:
        raise ValueError('current_first: load/source outside the current tree ' + net)
    starts = [i for i, p in enumerate(pads) if '%s:%s' % (p['ref'], p['num']) in source]
    first = starts[0]
    if first not in trunk:
        first = next(c['caps'][-1] for c in KT or [] if first in c['pins'] + c['caps'])
    order = []; parent = {}; tree = [first]; rest = [i for i in trunk if i != first]
    while rest:
        _, j, a = min((math.dist((pads[i]['x'], pads[i]['y']), (pads[a]['x'], pads[a]['y'])), i, a) for i in rest for a in tree)
        parent[j] = a; order.append(j); tree.append(j); rest.remove(j)
    adj = {q: [] for q in pk.values()}; edges = []; keys = {}
    def edge(a, b, key):
        ti = len(edges); kid = ('s', ti, 0, 0., 1.)
        adj[pk[a]].append((pk[b], kid)); adj[pk[b]].append((pk[a], kid))
        edges.append(dict(layer='F', pts=[[pads[a]['x'], pads[a]['y']], [pads[b]['x'], pads[b]['y']]], w=[1.]))
        keys[key] = ti
    for j in order: edge(parent[j], j, ('trunk', j))
    for c in KT or []:
        cs, ps = c['caps'], c['pins']
        edge(cs[0], ps[0], ('link_chain', c['nr'], 0, cs[0]))
        for j in ps[1:]: edge(ps[0], j, ('link_chain', c['nr'], 0, j))
        for s, (a, b) in enumerate(zip(cs[:-1], cs[1:])): edge(b, a, ('link_chain', c['nr'], s + 1, b))
    from common.flow import analysis as _sz_flow
    reached = {names[q] for q in source}; open_items = list(reached)
    while open_items:
        for q, _ in adj[open_items.pop()]:
            if q not in reached: reached.add(q); open_items.append(q)
    if any(names[q] not in reached for q in required):
        raise ValueError('current_first: unreachable load/source in current tree ' + net)
    info = _sz_flow(edges, [], pads, fl, graph=dict(adj=adj, pad_pts=names, cu_layers=['F']))
    if info is None or any(pk[i] not in reached for i in planned):
        raise ValueError('current_first: incomplete planned flow tree ' + net)
    amp = {key: info['total'] * info['share'].get((ti, 0), 0.) for key, ti in keys.items()}
    CURRENT_AUDIT[net] = dict(current_A=info['total'], planned_branches=len(edges), source=sorted(source),
                           J=CURRENT_J, copper_mm=CURRENT_T, detour=CURRENT_DETOUR)
    return first, order, parent, amp


def current_net_class(C, I, a, b):
    widths = tuple(max(TRACK_MIN, C.get('dru_min', 0.), min(C['wmin'], NECK_W), I / (CURRENT_J * CURRENT_T[L])) for L in LAYERS)
    return dict(C, w=max(widths), wmin=max(widths), current_w=widths, current_I=I,
                current_clearance=(a['x'], a['y'], b['x'], b['y']), hist_weight=float(args.get('hist_weight_pw', '.3')),
                via=C['via'] * max(2, math.ceil(4 * I)))


def current_segment_width(C, capacity):


    return max(TRACK_MIN, min(NECK_W,C['w']), min(C['w'],capacity))
def current_floor(net, C):
    key = (net, C['current_w'])
    if key not in _SZ_FLOOR:
        while len(_SZ_FLOOR) >= 4: del _SZ_FLOOR[next(iter(_SZ_FLOOR))]
        wf = neck_field_map(net, C, max(C['current_w']))
        base = np.repeat(np.asarray(C['current_w'], np.float32), NN)
        if wf is not None: base = np.minimum(base, wf)
        _SZ_FLOOR[key] = base
    return _SZ_FLOOR[key]


def current_reserve(net, capH, viaH):
    if not CURRENT_FIRST: return capH, viaH
    pot = [li for li, L in enumerate(LAYERS) if L in CURRENT_POT and net not in CURRENT_POT[L]]
    reserve = current_model(net) is None and net not in GNDS and not pair_fast(net)
    if not reserve and not pot: return capH, viaH


    hard = _Maps(); hard.planar = np.concatenate([c.ravel() for c in capH])
    hard.extend(hard.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL))
    vh = viaH.copy()
    for li in pot: hard[li][:] = -1.
    if reserve:
        if _SZ_RESERVE[0] is None or _SZ_RESERVE[0][0] != _SZ_VERSION[0]:
            mask = np.zeros(NL * NN, bool); drills = np.zeros(NN, bool)
            for r in U.values():
                if current_model(r['net']): mask[r['cells']] = True; drills[r['dcells']] = True
            ds = [((_edt(~mask[li*NN:(li+1)*NN].reshape(NY,NX))*RES - QM - CLR - .005)*2).astype(np.float32)
                  if mask[li*NN:(li+1)*NN].any() else np.full((NY,NX),1e6,np.float32) for li in range(NL)]
            vd = (_edt(~drills.reshape(NY, NX))*RES - QM) if drills.any() else np.full((NY,NX), 1e6)
            _SZ_RESERVE[0] = (_SZ_VERSION[0], ds, vd)
        _, ds, vd = _SZ_RESERVE[0]
        for c, d in zip(hard, ds): np.minimum(c, d, out=c)
        vh &= vd >= HOLE_C
        for d in ds: vh &= d / 2 >= VIA_KEEP_R
    return hard, vh


def current_field_offsets(n):
    pitch = max(.5, HOLE_C, VIA_D + max(DRCC, DRCP) + .02)
    step = math.ceil(pitch / RES); r = math.ceil(math.sqrt(n))
    return sorted(((x * step, y * step) for y in range(-r, r + 1) for x in range(-r, r + 1)),
                  key=lambda q: (q[0]**2 + q[1]**2, q[1], q[0]))[:n]


def current_web_width(C, n, L):
    return max(TRACK_MIN, VIA_D, C.get('current_I', 0.) / n / (CURRENT_J * CURRENT_T[L]))


def current_field_mask(viaH, n, C, capH):
    offsets = current_field_offsets(n); legal = viaH.copy()
    for x,y in C.get('current_vias', ()):
        ix,iy=int((x-X0)/RES),int((y-Y0)/RES); rr=math.ceil(HOLE_C/RES)+1
        x0,x1=max(0,ix-rr),min(NX,ix+rr+1);y0,y1=max(0,iy-rr),min(NY,iy+rr+1)
        gx,gy=np.meshgrid(GXc[x0:x1],GYc[y0:y1])
        legal[y0:y1,x0:x1] &= np.hypot(gx-x,gy-y) >= HOLE_C
    good = legal.copy()


    r = max(math.hypot(x, y) for x, y in offsets)
    for li,c in enumerate(capH):

        if LAYERS[li] in CURRENT_POT and NET_CUR[0] not in CURRENT_POT[LAYERS[li]]: continue
        good &= c >= 2 * r * RES + current_web_width(C, n, LAYERS[li])
    for dx, dy in offsets:
        shifted = np.zeros_like(viaH)
        y0,y1=max(0,-dy),min(NY,NY-dy); x0,x1=max(0,-dx),min(NX,NX-dx)
        shifted[y0:y1,x0:x1] = legal[y0+dy:y1+dy,x0+dx:x1+dx]
        good &= shifted
    return good


def current_field_set(res, vs, C):
    bridges, extra = [], []; n = max(2, math.ceil(4 * C['current_I']-1e-8), C.get('current_n',0))
    for v, (a, b) in zip(vs, [(a//NN,b//NN) for a,b in zip(res[:-1],res[1:]) if a//NN != b//NN]):
        v['n'] = n; v['current_A'] = C['current_I']; v['current_layers'] = [LAYERS[a], LAYERS[b]]
        for dx, dy in current_field_offsets(n)[1:]:
            x,y = round(v['x'] + dx*RES,4),round(v['y'] + dy*RES,4)
            extra.append(dict(x=x,y=y,gr=1,too=[v['x'],v['y']],current_A=C['current_I']))
            for li in (a,b):

                p=(v['x'],v['y']); q=(x,y); mid=(p[0]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[0]-p[0]),
                                                p[1]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[1]-p[1]))
                pts=[list(p)]+([list(mid)] if mid!=p and mid!=q else [])+[list(q)]
                w=current_web_width(C,n,LAYERS[li])
                bridges.append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),web=True))
    return bridges, extra


def current_fields_free(res, vs, C, capH=None):
    ts, extra = current_field_set(res, copy.deepcopy(vs), C)
    points = [(v['x'],v['y']) for v in vs+extra] + list(C.get('current_vias',()))
    if not all(math.dist(a,b) >= HOLE_C - 1e-6 for i,a in enumerate(points) for b in points[i+1:]): return False


    return capH is None or all(line_cap_min(capH[LI[t['layer']]], a, b) >= w
                              for t in ts for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']))

def current_actual(tracks, vias, pads, fl):
    from common.flow import net_graph, pad_geo
    from shapely.geometry import Point, LineString
    from shapely.ops import nearest_points
    from shapely import STRtree
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve
    from collections import defaultdict
    tol = 1e-7
    def net_copper(L, p): return (L, round(p[0]/tol)*tol, round(p[1]/tol)*tol)
    segs = []
    for ti,t in enumerate(tracks):
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            if math.dist(a,b)>tol: segs.append((ti,si,t['layer'],LineString([a,b]),w))
    cuts = [{0.,s[3].length} for s in segs]; left = []; via_left = []
    by_layer = defaultdict(list)
    for i,s in enumerate(segs): by_layer[s[2]].append(i)
    for L,ids in sorted(by_layer.items()):
        tree = STRtree([segs[i][3].buffer(segs[i][4]/2+tol) for i in ids])
        for ii,i in enumerate(ids):
            a=segs[i][3]; aw=segs[i][4]
            for jj in sorted(tree.query(a.buffer(aw/2+tol))):
                j=ids[int(jj)]
                if j<=i: continue
                b=segs[j][3]
                if a.distance(b)> (aw+segs[j][4])/2+tol: continue
                inter=a.intersection(b)
                if inter.is_empty: pairs=[nearest_points(a,b)]
                elif inter.geom_type=='Point': pairs=[(inter,inter)]
                else:


                    gs=list(inter.geoms) if hasattr(inter,'geoms') else [inter]
                    pairs=[(Point(p),Point(p)) for g in gs for p in (list(g.coords) if hasattr(g,'coords') else [])]
                for p,q in pairs:
                    pa,pb=a.project(p),b.project(q); cuts[i].add(pa);cuts[j].add(pb)
                    left.append((net_copper(L,a.interpolate(pa).coords[0]),net_copper(L,b.interpolate(pb).coords[0])))
            for vi,v in enumerate(vias):
                p=Point(v['x'],v['y'])
                if a.distance(p)<=aw/2+float(v.get('d',VIA_D))/2+tol:
                    d=a.project(p); cuts[i].add(d)
                    via_left.append((net_copper(L,a.interpolate(d).coords[0]),net_copper(L,(v['x'],v['y']))))
            for p in pads:
                geo=pad_geo(p).get(L)
                if geo is not None and a.distance(geo)<=aw/2+tol:
                    q,_=nearest_points(a,geo);cuts[i].add(a.project(q))
    split=[]; orig=[]
    for i,(ti,si,L,line,w) in enumerate(segs):
        ds=sorted(cuts[i])
        for a,b in zip(ds[:-1],ds[1:]):
            if b-a<=tol:continue
            split.append(dict(layer=L,pts=[list(line.interpolate(a).coords[0]),list(line.interpolate(b).coords[0])],w=[w]))
            orig.append((ti,si))
    g=net_graph(split,[],pads,tol=tol);adj=g['adj'];pad_pts=g['pad_pts']
    for vi,v in enumerate(vias):
        ks=[net_copper(L,(v['x'],v['y'])) for L in LAYERS]
        for a,b in zip(ks[:-1],ks[1:]):
            adj[a].append((b,('v',vi)));adj[b].append((a,('v',vi)))
        for p in pads:
            for L,geo in pad_geo(p).items():
                if L in LAYERS and geo.distance(Point(v['x'],v['y']))<=float(v.get('d',VIA_D))/2+tol:
                    left.append((('PAD',p['ref'],p['num']),net_copper(L,(v['x'],v['y']))))
    for a,b in left+via_left:
        adj[a].append((b,None));adj[b].append((a,None))

    parent={q:q for q in adj}
    def root(q):
        while parent[q]!=q: parent[q]=parent[parent[q]];q=parent[q]
        return q
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None: parent[root(b)]=root(a)
    sources={root(pad_pts[q]) for q in fl['source'] if q in pad_pts and pad_pts[q] in parent}
    resistors={}; members=defaultdict(list)
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None or a>b:continue
            u,v=root(a),root(b)
            if u==v:continue
            if kid[0]=='s':
                t=split[kid[1]]
                resistance=1.72e-5*math.dist(t['pts'][0],t['pts'][1])*(kid[4]-kid[3])/(t['w'][0]*CURRENT_T[t['layer']])
                tag=('s',*orig[kid[1]])
            else:
                resistance=.0024/max(1,NL-1); tag=('v',kid[1],a[0],b[0])
            if resistance<=1e-15:continue
            key=('s',a,b) if kid[0]=='s' else ('v',kid[1],a,b)


            old=resistors.get(key)
            resistors[key]=(u,v,max(old[2] if old else 0.,1/resistance));members[key].append(tag)
    edges=defaultdict(float)
    for u,v,conductance in resistors.values():edges[tuple(sorted((u,v)))]+=conductance
    conn=defaultdict(list)
    for u,v in edges:conn[u].append(v);conn[v].append(u)
    reached=set(sources);todo=list(sources)
    while todo:
        for v in conn[todo.pop()]:
            if v not in reached:reached.add(v);todo.append(v)
    missing=sorted(q for q in set(fl['load'])|set(fl['source']) if q not in pad_pts or pad_pts[q] not in parent or root(pad_pts[q]) not in reached)
    if missing:return dict(absent=missing,seg={},via={},total=sum(map(float,fl['load'].values())))
    nodes=sorted(reached-sources);idx={q:i for i,q in enumerate(nodes)};rhs=np.zeros(len(nodes));row=[];col=[];val=[]
    for q,I in fl['load'].items():
        k=root(pad_pts[q])
        if k in idx:rhs[idx[k]]+=float(I)
    for (u,v),conductance in edges.items():
        if u not in reached:continue
        for a,b in ((u,v),(v,u)):
            if a in idx:
                row.append(idx[a]);col.append(idx[a]);val.append(conductance)
                if b in idx:row.append(idx[a]);col.append(idx[b]);val.append(-conductance)
    voltage=spsolve(coo_matrix((val,(row,col)),shape=(len(nodes),len(nodes))).tocsr(),rhs) if nodes else np.zeros(0)
    if not np.all(np.isfinite(voltage)):raise ValueError('current_first: singular final graph')
    amps={};viamps={}
    for key,(u,v,conductance) in resistors.items():
        if u not in reached:continue
        I=abs((voltage[idx[u]] if u in idx else 0.)-(voltage[idx[v]] if v in idx else 0.))*conductance
        for tag in members[key]:
            if tag[0]=='s':amps[tag[1:]]=max(amps.get(tag[1:],0.),I)
            else:viamps[tag[1:]]=max(viamps.get(tag[1:],0.),I)
    return dict(absent=[],seg=amps,via=viamps,total=sum(map(float,fl['load'].values())))


def current_correct(u, C):
    r=U[u];net=r['net'];fl=current_model(net)
    info=current_actual(r['tracks'],r['vias'],[D['pads'][i] for i in by_net[net]],fl)
    if info['absent']:return info
    goal={(ti,si):max(TRACK_MIN,DRU_MIN.get(net,0.),min(NECK_W,C['wmin']),I/(CURRENT_J*CURRENT_T[r['tracks'][ti]['layer']]))
          for (ti,si),I in info['seg'].items()}
    FLOW_W[u]=goal


    points=[p for t in r['tracks'] for p in t['pts']]+[[v['x'],v['y']] for v in r['vias']]
    xs=[p[0] for p in points];ys=[p[1] for p in points]
    limit=max([C.get('w',0.)]+list(goal.values())+[w for t in r['tracks'] for w in t['w']])
    capH,*_=maps(net,(min(xs)-2.,min(ys)-2.,max(xs)+2.,max(ys)+2.),limit)
    sb=_disturb_width(net)
    X=_cand_idx_build(u);tr=copy.deepcopy(r['tracks']);vs=copy.deepcopy(r['vias'])
    metric=_metric_prepare(r,net) if CHAIN and any('metric' in t for t in tr) else None
    ab=_tap_prepare(r,net) if TAP_ON and any(t.get('tap') for t in tr) else None
    widths=[list(t['w']) for t in tr]
    for ti,t in enumerate(tr):
        li=LI[t['layer']]
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            target=max(w,goal.get((ti,si),w));limit=_r32_clearance(X,u,li,a,b,net,target)
            limit=min(limit,line_cap_min(capH[li],a,b),2*pad_room(li,a,b,net))
            if metric is not None:limit=min(limit,_metric_width(metric,r,widths,ti,a,b,li))
            if ab is not None:limit=min(limit,_tap_width(ab,a,b,li))
            for dk,sp in sb:
                limit=min(limit,2*(line_cap_min(dk[li],a,b)-sp))
                for lj,s in neighbor_spacing(li,sp):limit=min(limit,2*(line_cap_min(dk[lj],a,b)-s))
            t['w'][si]=max(w,min(target,limit))
        widths[ti]=t['w']


    for vi,v in enumerate(vs):
        v['current_actual_A']=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.])
        if not v.get('n'):continue
        ids={j for j,w in enumerate(vs) if j==vi or w.get('too')==[v['x'],v['y']]}
        sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        I=max(sums.values(),default=0.);v['current_A']=I
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*I-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
        if v['n']<=len(ids):continue
        layers=v.get('current_layers')
        if not layers:continue
        bridges,extras=current_field_set([LI[layers[0]]*NN,LI[layers[1]]*NN],[dict(x=v['x'],y=v['y'])],dict(current_I=I,current_n=v['n']))
        template=next((t for t in tr if t.get('web') and t.get('link_chain')==v.get('link_chain')), {})
        for t in bridges:t.update({k:template[k] for k in ('link_chain','kp') if k in template})
        new=[w for w in extras if all(math.dist((w['x'],w['y']),(q['x'],q['y']))>.02 for q in vs)]
        if not new:continue
        if any(math.dist((w['x'],w['y']),(q['x'],q['y']))<HOLE_C-1e-6 for w in new for q in vs):continue
        good=True
        for w in new:
            ix,iy=int((w['x']-X0)/RES),int((w['y']-Y0)/RES);q=(w['x'],w['y'])
            good &= 0<=ix<NX and 0<=iy<NY
            if not good:break
            good &= via_hard(net,np.array([iy*NX+ix]))
            good &= all(_r32_clearance(X,u,li,q,q,net,VIA_D)>=VIA_D-1e-9 for li in range(NL))
            good &= all(math.dist(q,(v2['x'],v2['y']))>=HOLE_C-1e-6
                        for uu,rr in U.items() if uu!=u for v2 in rr['vias'])
        if not good:continue
        for t in bridges:
            li=LI[t['layer']]
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                good &= _r32_clearance(X,u,li,a,b,net,w)>=w-1e-9 and 2*pad_room(li,a,b,net)>=w-1e-9
                good &= line_cap_min(capH[li],a,b)>=w-1e-9
                if metric is not None:
                    rr=dict(r,tracks=tr+[t]);ww=widths+[t['w']]
                    good &= _metric_width(metric,rr,ww,len(tr),a,b,li)>=w-1e-9
                if ab is not None:good &= _tap_width(ab,a,b,li)>=w-1e-9
        if good:tr+=bridges;vs+=new
    del_unit(u);add_unit(u,net,tr,vs)
    return info


def current_final(u, C):
    for _ in range(8):
        before=copy.deepcopy((U[u]['tracks'],U[u]['vias']))
        info=current_correct(u,C)
        if info['absent'] or before==(U[u]['tracks'],U[u]['vias']):break
    r=U[u];info=current_actual(r['tracks'],r['vias'],[D['pads'][i] for i in by_net[r['net']]],current_model(r['net']))
    deficit=[]
    for (ti,si),I in info['seg'].items():
        t=r['tracks'][ti];target=I/(CURRENT_J*CURRENT_T[t['layer']])
        if target>t['w'][si]+1e-6:deficit.append(dict(track=ti,segment=si,current_A=I,actual_mm=t['w'][si],nominal_mm=target))
    via_deficit=[]
    for vi,v in enumerate(r['vias']):
        I=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.]);v['current_actual_A']=I
        if I>.25+1e-6:via_deficit.append(dict(via=vi,current_A=I,nominal_A=.25))


    for vi,v in enumerate(r['vias']):
        if not v.get('n'):continue
        ids={j for j,w in enumerate(r['vias']) if j==vi or w.get('too')==[v['x'],v['y']]};sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        v['current_A']=max(sums.values(),default=0.)
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*v['current_A']-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
    audit=CURRENT_AUDIT.setdefault(r['net'],{})
    audit['final_check']=dict(model='Resistance graph/Kirchhoff, ideal contacts',current_A=info['total'],
                             missing_pads=info['absent'],width_deficits=deficit,via_deficits=via_deficit,
                             via_field_missing=cons_missing(u))
    return not (info['absent'] or deficit or via_deficit or cons_missing(u))
IMP_REF_MASK = {}; IMP_REF_GEOM = {}
if IMP_TAB and D.get('plane_domains'):
    from analysis.structure import reference_planes as _z_areas
    _z_polys = _z_areas(D, GNDS, dict(options={'gnd_layers': args.get('gnd_layers', 'In1')}), _all_nets)
    _z_x, _z_y = np.meshgrid(GXc, GYc)
    for _net, _tab in sorted(IMP_TAB.items()):
        for _layer, _q in sorted(_tab.items()):
            _key = (_net, _layer)
            _mask = np.ones((NY, NX), dtype=bool)
            _geom = None
            for _ref in _q['profile_data']['ref']:
                _p = _z_polys.get(_ref, Polygon())
                _geom = _p if _geom is None else _geom.intersection(_p)
            IMP_REF_GEOM[_key] = _geom

            _mask &= shapely.contains_xy(_geom.buffer(-_q['w']/2 - RES), _z_x, _z_y)
            IMP_REF_MASK[_key] = _mask
    del _z_x, _z_y


def z_ref_free(C, li, a, b, net=None):
    if not C.get('zw'): return True
    g = IMP_REF_GEOM.get((NET_CUR[0] if net is None else net, LAYERS[li]))
    return g is None or g.covers(_LS([a, b]).buffer(C['zw'][li]/2))

PLANE_MASK = {}; PLANE_AREA = {}; PLANE_INFO = None
PLANE_COPPER = {}; PLANE_TRACE = {}; PLANE_LAYER = {}; PLANE_REF = {}
PLANE_FEEDERS = False
if (BGA_PLAN or PIN_ACCESS) and D.get('plane_domains'):
    PLANE_LAYER = {}; PLANE_COPPER = {}


    import hashlib as _plane_hash
    _raw = open(D['plane_domains'], 'rb').read(); _domains = json.loads(_raw)
    PLANE_FEEDERS = PIN_ACCESS and bool(_domains.get('group_feeders',False))
    if PIN_ACCESS and _domains.get('outline_targets') and not _domains.get('reference_domains'):
        raise ValueError('outline_targets needs native reference_domains')
    PLANE_REF={}
    if PIN_ACCESS:
        for _z in _domains.get('reference_domains',_domains['zones']):
            PLANE_REF.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        PLANE_REF={key:shapely.union_all(gs) for key,gs in PLANE_REF.items()}
    _plane_nets = {}
    if PIN_ACCESS and not BGA_PLAN:

        _plane_nets = {n: [] for n in sorted(AREA_NETS)}
    for _z in _domains['zones']:
        if PIN_ACCESS:PLANE_LAYER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z.get('layer') in globals().get('PLANE_SIGNALS', set()):
            PLANE_COPPER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z['net'] in AREA_NETS:
            if PIN_ACCESS and not BGA_PLAN and _z.get('layer') not in [L for L,nn in FL_ASSIGNMENT.items() if _z['net'] in nn]: continue
            _plane_nets.setdefault(_z['net'], []).append(Polygon(_z['pts'], _z.get('holes', [])).buffer(0))
    for _n, _gs in sorted(_plane_nets.items()):
        _area = shapely.union_all(_gs).buffer(-VIA_R - 0.01)
        if PIN_ACCESS and _domains.get('outline_targets') and _n in globals().get('PLANE_MIXED_NETS',()) and _n not in GNDS:

            _original=[g for (n,L),g in PLANE_REF.items() if n==_n and L in PLANE_SIGNALS and n in FL_ASSIGNMENT.get(L,[])]
            _area=shapely.union_all(_original).buffer(-VIA_R-.01) if _original else Polygon()
        PLANE_AREA[_n] = _area
        _mask = np.zeros((NY, NX), dtype=bool)
        if not _area.is_empty:
            _x0,_y0,_x1,_y1 = _area.bounds
            _ix0,_ix1 = max(0,int((_x0-X0)/RES)),min(NX,int((_x1-X0)/RES)+2)
            _iy0,_iy1 = max(0,int((_y0-Y0)/RES)),min(NY,int((_y1-Y0)/RES)+2)
            _xx,_yy = np.meshgrid(GXc[_ix0:_ix1], GYc[_iy0:_iy1])
            _mask[_iy0:_iy1,_ix0:_ix1] = shapely.contains_xy(_area,_xx,_yy)
        PLANE_MASK[_n] = _mask
    PLANE_INFO = dict(sha256=_plane_hash.sha256(_raw).hexdigest(),net_list=sorted(PLANE_AREA),cu_layers=_domains.get('gnd_layers'))
    PLANE_COPPER = {key:shapely.union_all(gs) for key,gs in PLANE_COPPER.items()}
    PLANE_LAYER = {key:shapely.union_all(gs) for key,gs in PLANE_LAYER.items()}


def plane_trace_maps(net, capH, rng=None):
    if not PLANE_COPPER: return
    ix0,ix1,iy0,iy1 = rng if rng is not None else (0,NX,0,NY)
    ys,xs=slice(iy0,iy1),slice(ix0,ix1)
    for (n,L),g in PLANE_COPPER.items():
        if n==net:continue
        key=(n,L)
        if key not in PLANE_TRACE:
            xx,yy=np.meshgrid(GXc,GYc)
            blocked=shapely.intersects_xy(g,xx,yy)
            PLANE_TRACE[key]=(_edt(~blocked)*RES-QM-CLR-.005).astype(np.float32)*2
            PLANE_TRACE[key][blocked]=-1.
        np.minimum(capH[LI[L]][ys,xs],PLANE_TRACE[key][ys,xs],out=capH[LI[L]][ys,xs])


def plane_feeder_targets(net, p, li, viaH, viaBad, rng):
    x0,x1,y0,y1=rng
    mask=viaH[y0:y1,x0:x1] & ~viaBad[y0:y1,x0:x1] & PLANE_MASK[net][y0:y1,x0:x1]
    yy,xx=np.nonzero(mask);xx=xx+x0;yy=yy+y0
    if not len(xx):return set()
    d=(GXc[xx]-p['x'])**2+(GYc[yy]-p['y'])**2
    order=np.lexsort((xx,yy,d))[:32]
    return {li*NN+int(yy[i])*NX+int(xx[i]) for i in order}
PF = [float(args.get('pf0', '0.6'))]
HARD_FOREIGN = [False]
SOURCE_FREE = [args.get('source_free', '0') == '1']
NET_CUR = ['']


def oct_net(net=None):

    return not _net_organic(NET_CUR[0] if net is None else net, NC) if STYLE_PER_NET_KIND else OCT

DISTURB_FACTOR = [1.0]
VIA_MASK = [None]
VIA_FACTOR = [1.0]
VIAF = VIA_FACTOR
ASTAR_COST = [None]
ASTAR_HINT = [None]
_LOOP_DEADLINE = None


def _loop_cpu_check():
    work_emergency_stop()
    if _LOOP_DEADLINE is not None and budget_clock() >= _LOOP_DEADLINE:
        raise TimeoutError('Loop CPU budget in search')

TAPS = args.get('taps', '1' if NC and NC.get('taps') else '0') == '1'



TAP_NC = TAPS and bool(NC) and 'taps' in NC and args.get('taps_source', 'netclasses') == 'netclasses'
WINDOW = float(args.get('window', '0'))
CONFLICT_SITE_EXACT = args.get('conflict_site', '') == 'exact'
NECK = float(args.get('neck', '0')); NECK_CACHE = {}
_NECK_MAX = max(4, int(float(args.get('neck_mb', '600')) * 1e6 / (NL * NN * 4)))


HFAC = float(args.get('astar', '1'))


BEND = float(args.get('bend', '0'))
if not math.isfinite(BEND) or BEND < 0: raise ValueError('bend must be finite and >= 0')


BEND_CURRENT = float(args.get('bend_current', '1'))
if not math.isfinite(BEND_CURRENT) or BEND_CURRENT < 0: raise ValueError('bend_current must be finite and >= 0')
VIA_TOO = [None]




VIA_WEB = float(args.get('via_web', '0.03'))
NECK_W = float(args.get('neck_w', args.get('sigw', '0.15') if NC else '0.25'))

NECK_FIELD = args.get('neck_field', '1' if NC else '0') == '1'
DENSE = []
if NECK_FIELD:
    from scipy.spatial import cKDTree as _KD
    _per = {}
    for _p in D['pads']: _per.setdefault(_p['ref'], []).append(_p)
    for _r in sorted(_per):
        _P = _per[_r]
        if len(_P) < 16: continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _pit = float(np.median(_KD(_xy).query(_xy, k=2)[0][:, 1]))
        if _pit > 0.85: continue
        _lg = sorted({LI[L] for q in _P for L in q['layers'] if L in LI})
        DENSE.append((_lg, max(0, int((_xy[:, 0].min() - _pit - X0) / RES)), min(NX, int((_xy[:, 0].max() + _pit - X0) / RES) + 1),
                      max(0, int((_xy[:, 1].min() - _pit - Y0) / RES)), min(NY, int((_xy[:, 1].max() + _pit - Y0) / RES) + 1),
                      {q['net'] for q in _P if q['net']}))
    if DENSE: print('dense fields (BGA, fine grid):', len(DENSE))
by_net_g = {}
for _k, _p in enumerate(D['pads']):
    if _p['net']: by_net_g.setdefault(_p['net'], []).append(_k)
_EMPTY = np.zeros(0, dtype=np.float32)
_MULT = np.empty(NL * NN, dtype=np.float32)


class _Maps(list):
    planar = None





CLOSE = args.get('near_map', '1') == '1'
CLOSE_W = float(args.get('close_w', '0.6'))




AREA = float(args.get('surface', '0'))
FL_GAP = float(args.get('area_gap', '0.6'))
FL_WMAX = float(args.get('area_wmax', '0.3'))



FL_PRICE = float(args.get('area_price', '0'))
if AREA > 0 or FL_PRICE > 0: CLOSE_W = max(CLOSE_W, 2 * (FL_GAP + FL_WMAX / 2 - CLR - RES))
D_CLOSE = max(CLOSE_W, VIA_D) / 2 + CLR + 2 * RES; PCN = int(math.ceil(D_CLOSE / RES)) + 2
CAP_CLOSE = np.float32(2 * (D_CLOSE - CLR))
D_BORE = HOLE_C + 2 * RES; PCB = int(math.ceil(D_BORE / RES)) + 2
SN = {}
UOWN = {}
NETU = {}
DDN = np.full(NN, D_BORE, dtype=np.float32)
SN_OPEN = {}
_TMP = np.zeros(NL * NN, dtype=bool)
_NVER = {}
_GEO = {}
_TRUNK = {}
_TRUNK_V = {}
if CLOSE:
    SN[''] = np.full(NL * NN, CAP_CLOSE, dtype=np.float32)
    for _g in sorted(AREA_NETS):
        SN[_g] = np.full(NL * NN, CAP_CLOSE, dtype=np.float32); UOWN[_g] = np.zeros(NL * NN, dtype=np.int16); SN_OPEN[_g] = []
FIXD = [None, None]
FIXNETS = set()
OFFS = [(1, 0, RES), (-1, 0, RES), (0, 1, RES), (0, -1, RES),
        (1, 1, RES * 1.4142), (1, -1, RES * 1.4142), (-1, 1, RES * 1.4142), (-1, -1, RES * 1.4142)]
GXc = X0 + (np.arange(NX) + 0.5) * RES; GYc = Y0 + (np.arange(NY) + 0.5) * RES


def seg_cells(li, a, b, w):
    (x0, y0), (x1, y1) = a, b
    r = w / 2 + QM
    ixa = max(0, int((min(x0, x1) - r - X0) / RES) - 1); ixb = min(NX, int((max(x0, x1) + r - X0) / RES) + 2)
    iya = max(0, int((min(y0, y1) - r - Y0) / RES) - 1); iyb = min(NY, int((max(y0, y1) + r - Y0) / RES) + 2)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    dx, dy = x1 - x0, y1 - y0; ll = dx * dx + dy * dy
    t = np.clip(((GX - x0) * dx + (GY - y0) * dy) / ll, 0, 1) if ll > 0 else 0
    m = np.hypot(GX - (x0 + t * dx), GY - (y0 + t * dy)) <= r
    yy, xx = np.nonzero(m)
    return li * NN + (yy + iya) * NX + (xx + ixa)


def drill_cells(x, y):
    r = VIA_DR / 2
    ix, iy = int((x - X0) / RES), int((y - Y0) / RES); rr = int(r / RES) + 1
    out = []
    for jy in range(iy - rr, iy + rr + 1):
        for jx in range(ix - rr, ix + rr + 1):
            if 0 <= jx < NX and 0 <= jy < NY and math.hypot(GXc[jx] - x, GYc[jy] - y) <= r + QM: out.append(jy * NX + jx)
    return np.array(out, dtype=np.int64)


def _vd_slice(x, y):
    r = VD_WINDOW / 2
    ax = max(0, int(math.ceil((x - r - X0) / RES - 0.5 - 1e-9)))
    bx = min(NX, int(math.floor((x + r - X0) / RES - 0.5 + 1e-9)) + 1)
    ay = max(0, int(math.ceil((y - r - Y0) / RES - 0.5 - 1e-9)))
    by = min(NY, int(math.floor((y + r - Y0) / RES - 0.5 + 1e-9)) + 1)
    return slice(ay, by), slice(ax, bx)


def _vd_update(net, vias, delta):
    for v in vias:
        key = (round(v['x'], 6), round(v['y'], 6), v.get('net') or net)
        before = _VD_VIAS.get(key, 0); after = before + delta
        if after < 0: raise RuntimeError('Via density reference counter negative')
        if bool(before) != bool(after):
            _VD_COUNT[_vd_slice(v['x'], v['y'])] += 1 if after else -1
            _VD_VERSION[0] += 1
        if after: _VD_VIAS[key] = after
        else: _VD_VIAS.pop(key, None)


def _vd_window_max(z):
    n = 2 * int(math.floor(VD_WINDOW / (2 * RES) + 1e-9)) + 1
    return ndimage.maximum_filter1d(ndimage.maximum_filter1d(z, n, axis=0, mode='constant', cval=0),
                                    n, axis=1, mode='constant', cval=0)


def _vd_cost(local=()):
    c = _VD_CACHE[0]
    if not local and c is not None and c[0] == _VD_VERSION[0]: return c[1]
    z = _VD_COUNT.astype(np.float32)
    seen = set()
    for v in local:
        key = (round(v['x'], 6), round(v['y'], 6))
        present = (*key, v.get('net') or NET_CUR[0]) in _VD_VIAS
        if key not in seen and not present:
            z[_vd_slice(*key)] += 1; seen.add(key)


    z = np.square(np.maximum(np.float32(0), _vd_window_max(z) + np.float32(1 - VD_THRESHOLD * VD_WINDOW ** 2)))
    if not local: _VD_CACHE[0] = (_VD_VERSION[0], z)
    return z


def add_unit(u, net, tracks, vias):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if current_model(net): _SZ_VERSION[0] += 1
    vv = []
    for v in vias:
        if all(math.hypot(v['x'] - w['x'], v['y'] - w['y']) > 0.02 for w in vv): vv.append(v)
    vias = vv
    cl = []
    for t in tracks:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): cl.append(seg_cells(li, a, b, w))
    dl = []
    for v in vias:


        for li in range(NL): cl.append(seg_cells(li, (v['x'], v['y']), (v['x'], v['y']), 2 * VIA_OCC_R))
        dl.append(drill_cells(v['x'], v['y']))
    cells = np.unique(np.concatenate(cl)) if cl else np.zeros(0, dtype=np.int64)
    dcells = np.unique(np.concatenate(dl)) if dl else np.zeros(0, dtype=np.int64)
    use[cells] += 1; drill_use[dcells] += 1
    _TRUNK.clear(); _TRUNK_V.clear()
    bb = _sub_boxes(cl, dl) if CLOSE else None
    U[u] = dict(net=net, tracks=tracks, vias=vias, cells=cells, dcells=dcells, bb=bb)
    if VD_COUNTER: _vd_update(net, vias, 1)
    _NVER[net] = _NVER.get(net, 0) + 1; _GEO.pop(u, None)
    if CLOSE:
        NETU[net] = NETU.get(net, 0) + 1
        if net in UOWN: UOWN[net][cells] += 1
        _close_update(bb, net, cells)


def del_unit(u):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    r = U.pop(u, None)
    _TRUNK.clear(); _TRUNK_V.clear()
    if r is None: return
    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if current_model(r['net']): _SZ_VERSION[0] += 1
    if VD_COUNTER: _vd_update(r['net'], r['vias'], -1)
    if VIA_ANCHOR > 0: VIA_PREV[u] = [(v['x'], v['y']) for v in r['vias'] if not v.get('rv') and not v.get('gr')]
    use[r['cells']] -= 1; drill_use[r['dcells']] -= 1
    _NVER[r['net']] = _NVER.get(r['net'], 0) + 1; _GEO.pop(u, None)
    if CLOSE:
        NETU[r['net']] -= 1
        if r['net'] in UOWN: UOWN[r['net']][r['cells']] -= 1
        _close_update(r['bb'], r['net'])


def _merge(B, border):
    B = sorted(set(B)); fl = lambda b: (b[1] - b[0] + 2 * border) * (b[3] - b[2] + 2 * border) + 6000
    i = 0
    while i < len(B):
        j = i + 1; new = False
        while j < len(B):
            a, b = B[i], B[j]; v = (min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3]))
            if fl(v) <= fl(a) + fl(b): B[i] = v; del B[j]; new = True
            else: j += 1
        if not new: i += 1
    return B


def _sub_boxes(cl, dl):
    per = {}
    for a in cl:
        if not len(a): continue
        li = int(a[0]) // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        per.setdefault(li, []).append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    bo = []
    for a in dl:
        if not len(a): continue
        iy = a // NX; ix = a - iy * NX
        bo.append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    return dict(copper_map=[(li, b) for li in sorted(per) for b in _merge(per[li], 2 * PCN)], bore=_merge(bo, 2 * PCB))


def _close_new(S, key, li, y0, y1, x0, x1):
    ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
    wy0, wy1, wx0, wx1 = max(0, ay0 - PCN), min(NY, ay1 + PCN), max(0, ax0 - PCN), min(NX, ax1 + PCN)
    sl = slice(li * NN, (li + 1) * NN)
    ul = use[sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]
    f = (ul > 0) if key == '' else ((ul - UOWN[key][sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]) > 0)
    goal = S[sl].reshape(NY, NX)
    if f.any():
        d = (_edt(~f) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
        goal[ay0:ay1, ax0:ax1] = np.minimum(2 * (d - CLR), CAP_CLOSE)
    else:
        goal[ay0:ay1, ax0:ax1] = CAP_CLOSE


def _close_update(bb, net, cells=None):
    S = SN['']
    if cells is not None: _TMP[cells] = True
    for li, (y0, y1, x0, x1) in bb['copper_map']:
        if cells is None: _close_new(S, '', li, y0, y1, x0, x1)
        else:
            ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
            sl = slice(li * NN, (li + 1) * NN)
            d = (_edt(~_TMP[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]) * RES).astype(np.float32)
            z = S[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]
            np.minimum(z, np.minimum(2 * (d - CLR), CAP_CLOSE), out=z)
    if cells is not None: _TMP[cells] = False
    for key, off in SN_OPEN.items():
        if key == net or off is None: continue
        off.extend((li, b) for li, b in bb['copper_map'])
        if len(off) > 400: SN_OPEN[key] = None
    D2 = DDN.reshape(NY, NX); du = drill_use.reshape(NY, NX)
    for y0, y1, x0, x1 in bb['bore']:
        ay0, ay1, ax0, ax1 = max(0, y0 - PCB), min(NY, y1 + PCB), max(0, x0 - PCB), min(NX, x1 + PCB)
        wy0, wy1, wx0, wx1 = max(0, ay0 - PCB), min(NY, ay1 + PCB), max(0, ax0 - PCB), min(NX, ax1 + PCB)
        fd = du[wy0:wy1, wx0:wx1] > 0
        if fd.any():
            dd = (_edt(~fd) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
            D2[ay0:ay1, ax0:ax1] = np.minimum(dd, D_BORE)
        else:
            D2[ay0:ay1, ax0:ax1] = D_BORE


def _close_flush(key):
    off = SN_OPEN[key]
    if off is None:
        S = SN[key]
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            f = ((use[sl] - UOWN[key][sl]) > 0).reshape(NY, NX)
            if f.any(): S[sl] = np.minimum(2 * ((_edt(~f) * RES).astype(np.float32) - CLR), CAP_CLOSE).ravel()
            else: S[sl] = CAP_CLOSE
    else:
        per = {}
        for li, b in off: per.setdefault(li, []).append(b)
        for li in sorted(per):
            for b in _merge(per[li], 2 * PCN): _close_new(SN[key], key, li, *b)
    SN_OPEN[key] = []


def own_counts(net):
    own = np.zeros(NL * NN, dtype=np.int16); od = np.zeros(NN, dtype=np.int16)
    for r in U.values():
        if r['net'] == net:
            own[r['cells']] += 1; od[r['dcells']] += 1
    return own, od


def _box_idx(box):
    if box is None: return (0, NX, 0, NY)
    ix0, ix1 = max(0, int((box[0] - X0) / RES) - 1), min(NX, int((box[2] - X0) / RES) + 2)
    iy0, iy1 = max(0, int((box[1] - Y0) / RES) - 1), min(NY, int((box[3] - Y0) / RES) + 2)
    if ix1 <= ix0 or iy1 <= iy0: return (0, NX, 0, NY)
    return (ix0, ix1, iy0, iy1)




_KA_META = {}
_KA_HARD = {}
_KA_CELLS = {}


def _ka_kind(g, pad=False):
    return _KA_META.get(id(g), ('Pad' if pad else 'Track', ''))


def _ka_pad_kind(p):
    if p.get('graphic') or p.get('board_copper') or p.get('ref','').startswith('CU_'):
        return ('Graphic','')
    return ('Pad',p.get('dru_pad_type') or ('NPTH, mechanical' if p.get('pad_type')=='npth' else 'Through-hole' if p.get('drill') else 'SMD'))


def _ka_pad_keeps(p,L):
    mk=(p.get('mask') or {}).get(L,0.)
    if mk>0:
        g=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        yield g.buffer(mk+.01,join_style=2),'MaskKeepout'
    if p.get('drill',0)>0 and HOLE_CLR>0:
        g=(shapely.union_all([Polygon(q) for q in p['hole_polys']])
           if DRCL and p.get('hole_polys') else _PT(p['x'],p['y']).buffer(p['drill']/2,32))
        yield g.buffer(HOLE_CLR,32),'HoleKeepout'


def _ka_geo_limit(net, other, li, g=None, g2=None, pad=False):
    if globals().get('KA') is None: return _pz_clearance(net,other,pad)
    t,p = _ka_kind(g) if g is not None else ('Track','')
    t2,p2 = _ka_kind(g2,pad)
    return _ka_margin(net,other,li,t,t2,p,p2)
def _ka_obstacles(net, li, fixed=True, without=None, pads=True):
    if pads:
        for n,g in _PADG[li]:
            if n != net and not g.is_empty:
                type_,pad = _ka_kind(g,True)
                yield n,g,type_,pad
        for n,g,type_ in globals().get('_KA_PAD_KEEP',{}).get(li,[]):
            if n!=net and not g.is_empty:yield n,g,type_,''
    for u,r in U.items():
        if u == without or (fixed and not u.startswith('FIX#')): continue
        for l,g,_,n in _unit_geo(u,r):
            if l==li and n!=net and not g.is_empty:
                type_,pad = _ka_kind(g)
                yield n,g,type_,pad
    for source in (_ZP_GEO, PLANE_COPPER):
        for (n,L),g in source.items():
            if n!=net and L==LAYERS[li] and not g.is_empty:yield n,g,'Zone',''


def _ka_field(net, li, points, obstacles, type_='Track', margin=.012):
    groups={}
    for n,g,t,p in obstacles:
        c=_ka_clr(net,n,li,type_,t,'',p)+margin
        groups.setdefault(c,[]).append(g)
    out=np.full(len(points),9.,dtype=np.float64)
    for c,gs in sorted(groups.items()):

        d=_pz_distance(shapely.STRtree(gs),points)
        out=np.minimum(out,d-c)
    return out


def _ka_raster(net,li,rng,obstacles,type_='Track',margin=.012,raw=False):


    x0,x1,y0,y1=rng
    cap=max(1.,cls(net)['w']*max(1.,WIDTH_INNER)/2+.5,VIA_KEEP_R+.5)
    out=np.full((y1-y0,x1-x0),cap,dtype=np.float32)
    for n,g,t,p in obstacles:
        if g.is_empty:continue
        c=0. if raw else _ka_clr(net,n,li,type_,t,'',p)+margin
        bx=g.bounds;r=cap+c+RES
        xa=max(x0,int(math.floor((bx[0]-r-X0)/RES)));xb=min(x1,int(math.ceil((bx[2]+r-X0)/RES)))
        ya=max(y0,int(math.floor((bx[1]-r-Y0)/RES)));yb=min(y1,int(math.ceil((bx[3]+r-Y0)/RES)))
        if xa>=xb or ya>=yb:continue
        xx,yy=np.meshgrid(GXc[xa:xb],GYc[ya:yb])
        d=shapely.distance(shapely.points(xx,yy),g)-c
        v=out[ya-y0:yb-y0,xa-x0:xb-x0]
        np.minimum(v,d,out=v)
    return out


def _ka_hard_maps(net,rng=None):
    rng=tuple(rng) if LARGE and rng is not None else _WHOLE
    key=(net,rng,_PZ_VERSION[0],tuple(sorted(PZ_LOCAL.get(net,()))))
    if key in _KA_HARD:return _KA_HARD[key]
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    margin=PZ_BORDER_NM/1e6 if _pz_active(net) else .012
    capH=_Maps();capH.planar=np.full(NL*NN,-1.,dtype=np.float32)
    via=(edge_d[ys,xs]-EDGE>=VIA_R)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
    if VIA_BAN is not None:via &= ~VIA_BAN[ys,xs]
    fx0,fx1=_fix_map(net)
    if fx1 is not None:via &= fx1[ys,xs]-QM>=HOLE_C
    for li in range(NL):
        obs=list(_ka_obstacles(net,li))
        track=_ka_raster(net,li,rng,obs,margin=margin)
        vroom=_ka_raster(net,li,rng,obs,'Via',margin)
        layer=capH.planar[li*NN:(li+1)*NN].reshape(NY,NX)
        layer[ys,xs]=2*np.minimum(track,edge_d[ys,xs]-EDGE)
        capH.append(layer)
        via &= vroom>=VIA_R
        if HOLE_CLR>0:
            raw=_ka_raster(net,li,rng,[o for o in obs if o[2] not in ('MaskKeepout','HoleKeepout')],raw=True)
            via &= raw>=VIA_DR/2+HOLE_CLR+margin
        if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_WEB
    viaH=np.zeros((NY,NX),dtype=bool);viaH[ys,xs]=via
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
    viaH[ys,xs]=_zp_viah(net,viaH[ys,xs],ys,xs)
    if PAD_HOLE_DIST is not None:viaH[ys,xs] &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
    if VP:_vp_one(net,viaH,rng)

    allowed=viaH.copy()
    if EPZ:_ep_one(net,viaH,capH,rng)
    viaH &= allowed
    plane_trace_maps(net,capH,rng)
    while len(_KA_HARD)>=_HM_MAX:_KA_HARD.pop(next(iter(_KA_HARD)))
    _KA_HARD[key]=(capH,viaH)
    return capH,viaH


def _ka_raw_distance(points,geoms):
    return _pz_distance(shapely.STRtree(geoms),points)


def _ka_cells(u,r):
    if u in _KA_CELLS:return _KA_CELLS[u]
    groups={}
    for t in r['tracks']:
        li=LI[t['layer']];n=t.get('net') or r['net']
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            groups.setdefault((li,n,'Track'),[]).append(seg_cells(li,a,b,w)-li*NN)
    for v in r['vias']:
        n=v.get('net') or r['net']
        for L in v.get('layers',LAYERS):
            if L not in LI:continue
            li=LI[L]
            groups.setdefault((li,n,'Via'),[]).append(seg_cells(li,(v['x'],v['y']),(v['x'],v['y']),float(v.get('d',VIA_D)))-li*NN)
    result={k:np.unique(np.concatenate(v)) for k,v in groups.items()}
    _KA_CELLS[u]=result
    return result


def _ka_soft_maps(net,box=None):



    rng=_box_idx(box);x0,x1,y0,y1=rng
    pad=int(math.ceil((max(KA.maximum,CLR)+1.5)/RES))+2
    jx0,jx1=max(0,x0-pad),min(NX,x1+pad);jy0,jy1=max(0,y0-pad),min(NY,y1+pad)
    cap=[]
    for li in range(NL):
        groups={}
        for u,r in U.items():
            for (l,n,t),cells in _ka_cells(u,r).items():
                if l!=li or n==net:continue
                c=max(_ka_clr(net,n,li,'Track',t),_ka_clr(net,n,li,'Via',t))+.005
                mask=groups.setdefault(c,np.zeros((jy1-jy0,jx1-jx0),dtype=bool))
                yy=cells//NX;xx=cells%NX
                inside=(yy>=jy0)&(yy<jy1)&(xx>=jx0)&(xx<jx1)
                mask[yy[inside]-jy0,xx[inside]-jx0]=True
        room=np.full((jy1-jy0,jx1-jx0),9.,dtype=np.float32)
        for c,mask in sorted(groups.items()):
            if mask.any():room=np.minimum(room,_edt(~mask)*RES-c)
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[y0:y1,x0:x1]=2*room[y0-jy0:y1-jy0,x0-jx0:x1-jx0];cap.append(a)
    fd=(drill_use>0).reshape(NY,NX)
    dd=(_edt(~fd)*RES).astype(np.float32) if fd.any() else np.full(fd.shape,9.,dtype=np.float32)
    return cap,dd,rng


def _ka_room(net,li,a,b,fixed=True,without=None):
    seg=_LS([a,b]) if a!=b else _PT(a)
    margin=PZ_BORDER_NM/1e6 if _pz_active(net) else PAD_EXTRA
    room=min((seg.distance(g)-_ka_clr(net,n,li,'Track',t,'',p)-margin
              for n,g,t,p in _ka_obstacles(net,li,fixed,without)),default=9.)
    board=globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg):return -1.
        room=min(room,seg.distance(board.boundary)-float(args.get('edge','.30'))-margin)
    return room

def _ka_via_ok(net,x,y,fixed=True,without=None):
    pt=_PT(x,y);margin=PZ_BORDER_NM/1e6
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+margin):return False
    if any(pt.distance(g)<VIA_R+margin for g in _VIA_KEEP):return False
    for li in range(NL):
        for n,g,t,p in _ka_obstacles(net,li,fixed,without):
            limit=VIA_R if t in ('MaskKeepout','HoleKeepout') else max(VIA_R+_ka_clr(net,n,li,'Via',t,'',p),VIA_DR/2+HOLE_CLR)
            if pt.distance(g)<limit+margin:return False
        if li in (0,NL-1) and not VIP:
            if any(n==net and pt.distance(g)<VIA_R+VIA_WEB for n,g in _PADG[li]):return False

    return _ka_legacy_via_ok(net,x,y,fixed,without)


def soft_maps(net, box=None, wmax=None):
    if globals().get('KA') is not None: return _ka_soft_maps(net,box)
    if CLOSE and wmax is not None and wmax <= CLOSE_W:
        key = '' if NETU.get(net, 0) == 0 else (net if net in SN else None)
        if key is not None:
            if key and (SN_OPEN[key] is None or SN_OPEN[key]): _close_flush(key)
            capS = _Maps(SN[key][li * NN:(li + 1) * NN].reshape(NY, NX) for li in range(NL)); capS.planar = SN[key]
            return capS, DDN.reshape(NY, NX), _box_idx(None)
    own = None if (CLOSE and NETU.get(net, 0) == 0) else own_counts(net)[0]
    ix0, ix1, iy0, iy1 = _box_idx(box)
    P = int(1.5 / RES) + 2
    jx0, jx1, jy0, jy1 = max(0, ix0 - P), min(NX, ix1 + P), max(0, iy0 - P), min(NY, iy1 + P)
    part = box is not None and (jx0, jx1, jy0, jy1) != (0, NX, 0, NY)
    capS = []
    for li in range(NL):
        f = use[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]
        f = (f > 0) if own is None else ((f - own[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]) > 0)
        if f.any(): d = (_edt(~f) * RES).astype(np.float32)
        else: d = np.full(f.shape, 9.0, dtype=np.float32)
        if not part: capS.append(2 * (d - CLR)); continue
        full = np.full((NY, NX), -1.0, dtype=np.float32)
        full[iy0:iy1, ix0:ix1] = 2 * (d[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0] - CLR)
        capS.append(full)
    fd = (drill_use > 0).reshape(NY, NX)[jy0:jy1, jx0:jx1]
    dd = (_edt(~fd) * RES).astype(np.float32) if fd.any() else np.full(fd.shape, 9.0, dtype=np.float32)
    if part:
        full = np.zeros((NY, NX), dtype=np.float32); full[iy0:iy1, ix0:ix1] = dd[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0]; dd = full
    return capS, dd, _box_idx(box)


def _in_window(win, rng):
    ixa, ixb = max(1, int((win[0] - X0) / RES)), min(NX - 2, int((win[2] - X0) / RES))
    iya, iyb = max(1, int((win[1] - Y0) / RES)), min(NY - 2, int((win[3] - Y0) / RES))
    return rng[0] <= ixa and ixb < rng[1] and rng[2] <= iya and iyb < rng[3]


REACH = args.get('reachable', '1') == '1'
_HM = {}
_HM_MAX = max(4, int(float(args.get('hm_mb', '1500')) * 1e6 / (NL * NN * 4 + NN)))


LARGE = args.get('large', 'auto') == '1' or (args.get('large', 'auto') == 'auto' and len(nets) > _HM_MAX)
_WHOLE = (0, NX, 0, NY)
if LARGE:
    _B_CAPH = np.empty(NL * NN, dtype=np.float32); _B_CAPC = np.empty(NL * NN, dtype=np.float32)
    _B_VIAH = np.zeros(NN, dtype=bool); _B_VBAD = np.zeros(NN, dtype=bool)
    print('large board: maps per unit only in the window (%d nets, room for %d full maps)' % (len(nets), _HM_MAX))


_FIX_MAPS = {}


def _fix_map(net):
    if FIXD[0] is None or net not in FIXNETS: return FIXD
    if net not in _FIX_MAPS:
        fo = np.zeros(NL * NN, dtype=bool)
        for u, r in U.items():
            if u.startswith('FIX#') and r['net'] != net: fo[r['cells']] = True
        _FIX_MAPS[net] = [[(_edt(~fo[i * NN:(i + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for i in range(NL)], FIXD[1]]
    return _FIX_MAPS[net]


def _pad_hole_free(dist):
    return dist >= VIA_DR/2+H2H+PAD_HOLE_RESERVE




_PZ_FIXED = {}; _PZ_HARD = {}; _PZ_DYNAMIC = {}; _PZ_UNITS = {}; _PZ_VERSION = [0,0]
_PZ_BUDGET_ACTIVE = [False]
_PZ_PARTIAL = [None]
_PZ_BASE = [False]


class _PZBudget(RuntimeError):
    pass


def _pz_check():
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if _PZ_BUDGET_ACTIVE[0] and time_tight(.95):raise _PZBudget('Pin access post-pass: CPU budget')


def _pz_index(net, fixed=True, without=None):
    from shapely.strtree import STRtree
    key=(net,_PZ_VERSION[0 if fixed else 1],without)
    cache=_PZ_FIXED if fixed else _PZ_DYNAMIC
    if key in cache:return cache[key]
    units = [(u,r) for u,r in U.items() if u != without and (not fixed or u.startswith('FIX#'))]
    layers = [[] for _ in LAYERS]; holes = []
    for u,r in units:
        _pz_check()
        if u not in _PZ_UNITS:
            copper=[]; drills=[]
            for t in r['tracks']:
                if t['layer'] not in LI:continue
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    center=_LS([a,b]) if a!=b else _PT(a)
                    copper.append((r['net'],LI[t['layer']],center.buffer(w/2/math.cos(math.pi/256),64)))
            for v in r['vias']:
                radius=float(v.get('d',VIA_D))/2;drill=float(v.get('drill',VIA_DR))/2
                point=_PT(v['x'],v['y']);n=v.get('net') or r['net']
                drills.append((point,drill,n,radius))
                geom=point.buffer(max(radius,drill+HOLE_CLR-DRCC)/math.cos(math.pi/256),64)
                for L in v.get('layers',LAYERS):
                    if L in LI:copper.append((n,LI[L],geom))
            _PZ_UNITS[u]=(copper,drills)
        copper,drills=_PZ_UNITS[u];holes.extend(drills)
        for n,li,g in copper:
            if n!=net:layers[li].append(g)
    trees = [STRtree(g) for g in layers]
    hole_tree=STRtree([pt.buffer(drill/math.cos(math.pi/256),64) for pt,drill,_,_ in holes])
    result = (layers, trees, holes,hole_tree)
    if len(cache)>=4:cache.pop(next(iter(cache)))
    cache[key]=result
    return result


def _pz_distance(tree, points):
    d = np.full(len(points),9.,dtype=np.float64)
    if len(tree.geometries):
        for offset in range(0,len(points),16384):
            _pz_check()
            ij, ds = tree.query_nearest(points[offset:offset+16384], return_distance=True, all_matches=False)
            d[offset+ij[0]] = ds
    return d


def _pz_hard_maps(net, rng):
    rng = tuple(rng) if LARGE and rng is not None else _WHOLE
    key = (net,rng,_PZ_VERSION[0])
    if key in _PZ_HARD: return _PZ_HARD[key]
    _PZ_BASE[0] = True
    try: base, viaH = hard_maps(net,rng)
    finally: _PZ_BASE[0] = False
    capH = _Maps(); capH.planar = np.concatenate([a.ravel().copy() for a in base])
    capH.extend([capH.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    viaH = viaH.copy()
    _, trees, holes, hole_tree = _pz_index(net)
    margin = PZ_BORDER_NM / 1e6
    for p in D['pads']:
        if id(p) not in PZ_LOCAL[net]: continue
        bx=p['bbox'];r=PZ_RADIUS_NM/1e6
        x0=max(rng[0],int((bx[0]-r-X0)/RES));x1=min(rng[1],int((bx[2]+r-X0)/RES)+1)
        y0=max(rng[2],int((bx[1]-r-Y0)/RES));y1=min(rng[3],int((bx[3]+r-Y0)/RES)+1)
        if x0>=x1 or y0>=y1:continue
        ys=slice(y0,y1);xs=slice(x0,x1)
        xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
        via=(edge_d[ys,xs]-EDGE>=VIA_R+margin)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
        if VIA_BAN is not None:via &= ~VIA_BAN[ys,xs]
        via &= _pz_distance(hole_tree,pts).reshape(xx.shape)>=VIA_DR/2+H2H+margin
        for li in range(NL):
            dp=np.where(PN1[li][ys,xs]==NID[net],PD2[li][ys,xs],PD1[li][ys,xs])
            fixed=_pz_distance(trees[li],pts).reshape(xx.shape)
            ch=np.minimum(2*(dp-DRCP-margin),2*(edge_d[ys,xs]-EDGE-margin))
            ch=np.minimum(ch,2*(fixed-DRCC-margin))
            capH[li][ys,xs]=_zp_hard(net,ch,li,ys,xs)
            via &= (dp>=max(VIA_R+DRCP,VIA_DR/2+HOLE_CLR)+margin)&(fixed>=VIA_R+DRCC+margin)
            if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_WEB
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):via &= PLANE_MASK[net][ys,xs]
        via=_zp_viah(net,via,ys,xs)
        if PAD_HOLE_DIST is not None:via &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
        viaH[ys,xs]=via
    if VP:_vp_one(net,viaH,rng)
    plane_trace_maps(net,capH,rng)
    while len(_PZ_HARD)>=max(1,min(3,_HM_MAX)):_PZ_HARD.pop(next(iter(_PZ_HARD)))
    _PZ_HARD[key]=(capH,viaH)
    return capH,viaH


def _pz_room(net,li,a,b,fixed=True,without=None):
    if globals().get('KA') is not None: return _ka_room(net,li,a,b,fixed,without)
    seg=_LS([a,b]) if a!=b else _PT(a)
    _, trees, _, _ = _pz_index(net,fixed,without)
    best=9.
    tree=trees[li]
    if len(tree.geometries):
        j=tree.nearest(seg)
        best=min(best,seg.distance(tree.geometries[j])-DRCC-(PZ_BORDER_NM/1e6))

    board = globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg): return -1.
        best=min(best,seg.distance(board.boundary)-float(args.get('edge','.30'))-(PZ_BORDER_NM/1e6))
    return best


def _ka_legacy_via_ok(net,x,y,fixed=True,without=None):
    pt=_PT(x,y); _,trees,holes,hole_tree=_pz_index(net,fixed,without)
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+(PZ_BORDER_NM/1e6)):return False
    if any(pt.distance(g)<VIA_R+(PZ_BORDER_NM/1e6) for g in _VIA_KEEP):return False
    for li in range(NL):
        for j in _PADT[li].query(pt.buffer(VIA_R+max(DRCP,HOLE_CLR)+.01)):
            n,g=_PADG[li][int(j)];d=pt.distance(g)
            if globals().get('KA') is None and n!=net and d<max(VIA_R+DRCP,VIA_DR/2+HOLE_CLR)+(PZ_BORDER_NM/1e6):return False
            if n==net and li in (0,NL-1) and not VIP and d<VIA_R+VIA_WEB:return False
        for j in ([] if globals().get('KA') is not None else trees[li].query(pt.buffer(max(VIA_R+DRCC,VIA_DR/2+HOLE_CLR)+.01))):
            if pt.distance(trees[li].geometries[j])<max(VIA_R+DRCC,VIA_DR/2+HOLE_CLR)+(PZ_BORDER_NM/1e6):return False
        for (n,L),g in _ZP_GEO.items():
            if globals().get('KA') is None and n!=net and L==LAYERS[li] and pt.distance(g)<VIA_R+CLR+.005:return False
    if any(pt.distance(g)<VIA_DR/2+H2H+(PZ_BORDER_NM/1e6) for g in PAD_HOLE_GEOMS):return False
    for j in hole_tree.query(pt.buffer(VIA_DR/2+H2H+(PZ_BORDER_NM/1e6))):
        p,drill,_,_=holes[int(j)]
        if pt.distance(p)<VIA_DR/2+drill+H2H+(PZ_BORDER_NM/1e6):return False
    for vs in D.get('via_keepout',[]):
        g=shapely.union_all([Polygon(q) for q in vs['polys']])
        own=any(p['ref']==vs['ref'] and p['net']==net and any(Polygon(q).covers(pt) for pp in p['polys'].values() for q in pp) for p in D['pads'])
        if not own and pt.distance(g)<VIA_DR/2+H2H+(PZ_BORDER_NM/1e6):return False
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
        ix=int((x-X0)/RES);iy=int((y-Y0)/RES)
        if not (0<=ix<NX and 0<=iy<NY and PLANE_MASK[net][iy,ix]):return False
        if PIN_ACCESS and not PLANE_AREA[net].covers(pt):return False
    return True


def _pz_via_ok(net,x,y,fixed=True,without=None):
    if globals().get('KA') is not None: return _ka_via_ok(net,x,y,fixed,without)
    return _ka_legacy_via_ok(net,x,y,fixed,without)


def _pz_path_points(path,net):
    pts=[cell_xy((n%NN)%NX,(n%NN)//NX) for n in path]
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if a//NN!=b//NN:
            p=_vp_xy(net,a%NN) if VP else None
            if p:pts[i]=pts[i+1]=p
    return [_pz_snap(p) for p in pts]


def _pz_grid_geometry(path,net,w):
    points=_pz_path_points(path,net);tr=[];vi=[];run=[points[0]];li=path[0]//NN
    def finish():
        if len(run)>1:

            ps=[run[0]]
            for j,p in enumerate(run[1:]):
                if len(ps)>1 and abs((ps[-1][0]-ps[-2][0])*(p[1]-ps[-1][1])-(ps[-1][1]-ps[-2][1])*(p[0]-ps[-1][0]))<1e-12:ps[-1]=p
                else:ps.append(p)
            tr.append(dict(layer=LAYERS[li],pts=[list(p) for p in ps],w=[w]*(len(ps)-1)))
    for i,n in enumerate(path[1:],1):
        if n//NN!=li:
            finish();vi.append(dict(x=points[i][0],y=points[i][1]));li=n//NN;run=[points[i]]
        else:run.append(points[i])
    finish();return tr,vi


def _pz_edge_retry(res,src,tgt,bx,capH,capS,viaH,viaBad,C,win,extra):
    net=NET_CUR[0];w=PZ_WIDTHS[net]['w'];protected=set(src)|set(tgt)
    ch=_Maps();ch.planar=np.concatenate([a.ravel() for a in capH])
    ch.extend([ch.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    vh=viaH.copy()
    for trial in range(5):
        _pz_check()
        if not res:return None
        tr,vi=_pz_grid_geometry(res,net,w)
        if _pz_geometry_ok(net,tr,vi,not HARD_FOREIGN[0]):
            PZ_EDGE_STAT['taken']+=1
            return res,tr,vi
        if trial==4 or ('time_tight' in globals() and time_tight(.95)):break
        points=_pz_path_points(res,net);blocked=False
        for i,(a,b) in enumerate(zip(res[:-1],res[1:])):
            p,q=points[i],points[i+1]
            if a//NN!=b//NN:
                if not _pz_via_ok(net,*p,not HARD_FOREIGN[0]):vh.flat[a%NN]=False;blocked=True;break
            elif min(pad_room(a//NN,p,q,net),_pz_room(net,a//NN,p,q,not HARD_FOREIGN[0]))<w/2-1e-9:
                candidates=[n for n in (b,a) if n not in protected]
                if candidates:
                    n=min(candidates,key=lambda n:float(ch.planar[n]));ch.planar[n]=-1.;blocked=True;break
        if not blocked:

            for i,v in enumerate(vi):
                if any(math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_BORDER_NM/1e6) for q in vi[:i]):
                    c=int((v['y']-Y0)/RES)*NX+int((v['x']-X0)/RES);vh.flat[c]=False;blocked=True;break
        if not blocked:break
        PZ_EDGE_STAT['repetitions']+=1
        res=astar_pf(src,tgt,bx,ch,capS,vh,viaBad,dict(C,w=w,wmin=w),win,extra=extra)
    PZ_EDGE_STAT['rejected']+=1;return None


PZ_EDGE_STAT = dict(taken=0,repetitions=0,rejected=0)


def _pz_geometry_ok(net,tracks,vias,fixed=True,without=None):


    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        for t in tracks:
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
        for v in vias:
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
    minimum=(globals().get('PZ_WIDTHS',{}).get(net) or {}).get('dru_min')
    if minimum is not None and any(w<minimum-1e-9 for t in tracks for w in t['w']):return False
    for t in tracks:
        li=LI[t['layer']]
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            if min(pad_room(li,a,b,net),_pz_room(net,li,a,b,fixed,without))<w/2-1e-9:return False
    for i,v in enumerate(vias):
        if any(.02<math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_BORDER_NM/1e6) for q in vias[:i]):return False
        if not _pz_via_ok(v.get('net') or net,v['x'],v['y'],fixed,without):return False
    return True


def hard_maps(net, rng=None):
    if globals().get('KA') is not None: return _ka_hard_maps(net,rng)
    if _pz_active(net) and not _PZ_BASE[0]: return _pz_hard_maps(net, rng)
    if net in _HM:
        r = _HM.pop(net); _HM[net] = r; return r
    nid = NID[net]
    fx0, fx1 = _fix_map(net)
    if LARGE and rng is not None and tuple(rng) != _WHOLE:
        ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
        capH = _Maps(); capH.planar = _B_CAPH
        v = (edge_d[ys, xs] - EDGE >= VIA_R) & (VSP[ys, xs] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_BAN is not None: v &= ~VIA_BAN[ys, xs]
        if fx1 is not None: v &= fx1[ys, xs] - QM >= HOLE_C
        for li in range(NL):
            dp = np.where(PN1[li][ys, xs] == nid, PD2[li][ys, xs], PD1[li][ys, xs])
            ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d[ys, xs] - EDGE))
            if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li][ys, xs] - QM - CLR - 0.005))
            ch = _zp_hard(net,ch,li,ys,xs)
            ch = ch.astype(np.float32)
            layer_name = _B_CAPH[li * NN:(li + 1) * NN].reshape(NY, NX); layer_name[ys, xs] = ch; capH.append(layer_name)
            v &= ch / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and not VIP: v &= padd[li][ys, xs] >= VIA_R + VIA_WEB
        viaH = _B_VIAH.reshape(NY, NX); viaH[ys, xs] = v
        if VP: _vp_one(net, viaH, rng)
        if EPZ: _ep_one(net, viaH, capH, rng)
        if GS_N: _lanes_one(net, capH, rng)
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
        if ZONE_PLAN: viaH[ys,xs] = _zp_viah(net,viaH[ys,xs],ys,xs)
        if PAD_HOLE_DIST is not None: viaH[ys,xs] &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
        plane_trace_maps(net,capH,rng)
        return capH, viaH
    capH = _Maps(); capH.planar = np.empty(NL * NN, dtype=np.float32)
    for li in range(NL):
        dp = np.where(PN1[li] == nid, PD2[li], PD1[li])
        ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d - EDGE))
        if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li] - QM - CLR - 0.005))
        ch = _zp_hard(net,ch,li)
        capH.planar[li * NN:(li + 1) * NN] = ch.astype(np.float32).ravel()
        capH.append(capH.planar[li * NN:(li + 1) * NN].reshape(NY, NX))
    viaH = (edge_d - EDGE >= VIA_R) & (VSP >= H2H + VIA_DR / 2 + 0.01)
    if VIA_BAN is not None: viaH &= ~VIA_BAN
    if fx1 is not None: viaH &= fx1 - QM >= HOLE_C
    for li in range(NL):
        viaH &= capH[li] / 2 >= VIA_KEEP_R
        if li in (0, NL - 1) and not VIP: viaH &= padd[li] >= VIA_R + VIA_WEB
    if VP: _vp_one(net, viaH, None)
    if EPZ: _ep_one(net, viaH, capH, None)
    if GS_N: _lanes_one(net, capH, None)
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH &= PLANE_MASK[net]
    viaH = _zp_viah(net,viaH)
    if PAD_HOLE_DIST is not None: viaH &= _pad_hole_free(PAD_HOLE_DIST)
    plane_trace_maps(net,capH)
    while len(_HM) >= _HM_MAX: del _HM[next(iter(_HM))]
    _HM[net] = (capH, viaH)
    return capH, viaH


def _pz_soft_maps(net,rng):
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
    _,trees,holes,hole_tree=_pz_index(net,False)
    cap=[]
    for tree in trees:
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[ys,xs]=2*(_pz_distance(tree,pts).reshape(xx.shape)-DRCC-1e-6)
        cap.append(a)
    dd=np.full((NY,NX),9.,dtype=np.float32)

    dd[ys,xs]=_pz_distance(hole_tree,pts).reshape(xx.shape)+VIA_DR/2+.02-1e-6
    return cap,dd


def maps(net, box, wmax):
    capS, dd, rng = soft_maps(net, box, wmax)
    if LARGE and box is not None and tuple(rng) == _WHOLE: rng = _box_idx(box)
    capH, viaH = hard_maps(net, rng)
    if globals().get('KA') is None and _pz_active(net) and (FIXD[0] is not None or HARD_FOREIGN[0]):
        if all(u.startswith('FIX#') or r['net']==net for u,r in U.items()):
            capS=capH


            dd=np.full((NY,NX),9.,dtype=np.float32)
        else:capS,dd=_pz_soft_maps(net,rng)
    capH, viaH = current_reserve(net, capH, viaH)
    if not LARGE or tuple(rng) == _WHOLE:
        viaBad = (dd < HOLE_C)
        for li in range(NL): viaBad |= capS[li] / 2 < VIA_R
        capC = [np.minimum(capH[i], capS[i]) for i in range(NL)]
        capCf = np.concatenate([c.ravel() for c in capC])
        if HARD_FOREIGN[0]:
            capH = _Maps(); capH.planar = capCf
            capH.extend([capCf[i * NN:(i + 1) * NN].reshape(NY, NX) for i in range(NL)])
            viaH = viaH & ~viaBad
        return capH, viaH, capS, dd, rng, viaBad, capC, capCf, viaBad.ravel()
    ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
    v = dd[ys, xs] < HOLE_C
    capC = _Maps(); capC.planar = _B_CAPC
    for li in range(NL):
        v |= capS[li][ys, xs] / 2 < VIA_R
        layer_name = _B_CAPC[li * NN:(li + 1) * NN].reshape(NY, NX); layer_name[ys, xs] = np.minimum(capH[li][ys, xs], capS[li][ys, xs]); capC.append(layer_name)
    viaBad = _B_VBAD.reshape(NY, NX); viaBad[ys, xs] = v
    return (capC if HARD_FOREIGN[0] else capH), (viaH & ~viaBad if HARD_FOREIGN[0] else viaH), capS, dd, rng, viaBad, capC, _B_CAPC, _B_VBAD


def maps_larger(net, box, win, wmax):
    if not LARGE: return maps(net, None, None), None
    b2 = (min(box[0], win[0]) - 1.0, min(box[1], win[1]) - 1.0, max(box[2], win[2]) + 1.0, max(box[3], win[3]) + 1.0)
    return maps(net, b2, wmax), b2


_ERR = {}
_ERR_MAX = max(2, int(float(args.get('err_mb', '600')) * 1e6 / (NL * NN * 4)))
UNREACHABLE = {}
_ERR_NETS = set()


def _components(net, wmin):
    key = (net, wmin)
    if key in _ERR:
        K = _ERR.pop(key); _ERR[key] = K; return K
    capH, viaH = hard_maps(net)
    from scipy.ndimage import label as _label
    eight = np.ones((3, 3), dtype=bool)
    K = np.zeros(NL * NN, dtype=np.int32); n_ges = 0
    for li in range(NL):
        lab, n = _label((capH[li] >= np.float64(wmin)) | viaH, structure=eight)
        lab = lab.astype(np.int32); lab[lab > 0] += n_ges; n_ges += n
        K[li * NN:(li + 1) * NN] = lab.ravel()
    parent = list(range(n_ges + 1))
    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    vidx = np.flatnonzero(viaH.ravel())
    if len(vidx):
        k0 = K[vidx]
        for li in range(1, NL):
            nc = K[li * NN + vidx]
            new_ = np.ones(len(vidx), dtype=bool); new_[1:] = (k0[1:] != k0[:-1]) | (nc[1:] != nc[:-1])
            for a, b in np.unique(np.stack([k0[new_], nc[new_]], axis=1), axis=0).tolist():
                ra, rb = root(a), root(b)
                if ra != rb: parent[max(ra, rb)] = min(ra, rb)
    K = np.array([root(i) for i in range(n_ges + 1)], dtype=np.int32)[K]
    while len(_ERR) >= _ERR_MAX: del _ERR[next(iter(_ERR))]
    _ERR[key] = K
    return K


def _unreachable(K, src, tgt):
    def surroundings(a):
        li = a // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        out = [a]
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                jy = iy + dy; jx = ix + dx; m = (jy >= 0) & (jy < NY) & (jx >= 0) & (jx < NX)
                out.append((li * NN + jy * NX + jx)[m])
        return np.concatenate(out)
    s = surroundings(np.asarray(src, dtype=np.int64)); t = np.fromiter(tgt, dtype=np.int64, count=len(tgt))
    if np.intersect1d(s, t).size: return False
    ks = np.unique(K[s]); kt = np.unique(K[surroundings(t)])
    return not np.intersect1d(ks[ks > 0], kt[kt > 0]).size


def neck_active(net, C, wmin, stub=False):
    if wmin <= max(NECK_W, DRU_MIN.get(net, 0.)) or net not in by_net_g: return False
    fld = NECK_FIELD and any(net in f[5] for f in DENSE)
    if C.get('prio', 8) < 8: return NECK > 0 or fld
    return (stub or BGA_TIGHT) and fld


def neck_field_map(net, C, wmin, stub=False):
    if not neck_active(net, C, wmin, stub): return None
    if (net, wmin) in NECK_CACHE:
        WMINF = NECK_CACHE.pop((net, wmin)); NECK_CACHE[(net, wmin)] = WMINF; return WMINF

    while len(NECK_CACHE) >= _NECK_MAX: del NECK_CACHE[next(iter(NECK_CACHE))]
    WMINF = np.full(NL * NN, wmin, dtype=np.float32); NECK_CACHE[(net, wmin)] = WMINF
    hw = max(NECK_W, DRU_MIN.get(net, 0.))
    if NECK > 0 and C.get('prio', 8) < 8:
        m = np.zeros(NL * NN, dtype=bool)
        for k in by_net_g[net]:
            for L, cells in D['pads'][k]['cells'].items(): m[LI[L] * NN + cells] = True
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any():
                near = _edt(~m[sl].reshape(NY, NX)) * RES <= NECK
                WMINF[sl][near.ravel()] = hw
    if NECK_FIELD:
        for lg, ix0, ix1, iy0, iy1, net_list in DENSE:
            if net not in net_list: continue
            for li in lg: WMINF[li * NN:(li + 1) * NN].reshape(NY, NX)[iy0:iy1, ix0:ix1] = hw
    return WMINF



def _bend_h(n, nn, nx, x0, y0, res, tx0, ty0, tx1, ty1, bend=0.):


    idx = n % nn
    x = x0 + (idx % nx + .5) * res; y = y0 + (idx // nx + .5) * res
    dx = max(tx0-x, 0., x-tx1); dy = max(ty0-y, 0., y-ty1)
    return max(dx, dy) + (1.4142 - 1.) * min(dx, dy)

def bend_core(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win,
               NX, NN, NL, X0, Y0, RES, g, par, gst, cst, tst, run, ofx, ofy, ofs, wminf, hfac, info, viaz, vzk,
               bend, via_extra, vpr, via_anchor, vtbox):
    ixa, ixb, iya, iyb = win[0], win[1], win[2], win[3]
    tx0, ty0, tx1, ty1 = tbox[0], tbox[1], tbox[2], tbox[3]
    hfac = min(1.0, hfac) * (1.4142 / math.sqrt(2.0))
    for t in tgt: tst[t] = run
    cap_k = max(65536, 2 * src.shape[0] + 64)
    hf = np.empty(cap_k, dtype=np.float64); hn = np.empty(cap_k, dtype=np.int64); hs = 0
    for s0 in src:
        s = s0 * 9 + 8
        gst[s] = run; g[s] = 0.0; par[s] = -1
        f0 = hfac * _bend_h(s // 9, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
        hs = _hpush(hf, hn, hs, f0, np.int64(s))
    while hs > 0:
        n = hn[0]
        hs = _hpop(hf, hn, hs)
        if hs + NL + 12 > cap_k:
            cap_k *= 2
            hf2 = np.empty(cap_k, dtype=np.float64); hn2 = np.empty(cap_k, dtype=np.int64)
            hf2[:hs] = hf[:hs]; hn2[:hs] = hn[:hs]; hf = hf2; hn = hn2
        if n < 0:
            n = -n - 1
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        if cst[n] == run: continue
        if info.shape[0] > 1: info[1] += 1
        node = n // 9; direction = n % 9


        dominated = False
        for d in range(9):
            other = node * 9 + d
            if other != n and gst[other] == run and (g[other] + (0.0 if d == 8 else bend) < g[n]):
                dominated = True; break
        if dominated:
            if info.shape[0] > 2: info[2] += 1
            continue
        cst[n] = run
        idx = node % NN
        li = node // NN
        if via_target:
            if tst[node] == run:
                L = 0; q = n
                while q != -1: L += 1; q = par[q]
                out = np.empty(L, dtype=np.int64); q = n; k = L - 1
                while q != -1: out[k] = q // 9; k -= 1; q = par[q]
                return out
            if viaf[idx] and (li == 0 or li == NL - 1):
                idx0 = node % NN
                xx = X0 + (idx0 % NX + 0.5) * RES; yy = Y0 + (idx0 // NX + 0.5) * RES
                dx0 = max(vtbox[0]-xx, 0.0, xx-vtbox[2]); dy0 = max(vtbox[1]-yy, 0.0, yy-vtbox[3])
                hv = math.sqrt(dx0*dx0+dy0*dy0)

                hs = _hpush(hf, hn, hs, g[n] + 0.3 + 2.0 * hv + (3.0 * PFv if vbad[idx] else 0.0)
                            + (vzk * np.float64(viaz[idx]) if viaz.shape[0] > 0 else 0.0), np.int64(-n - 1))
        elif tst[node] == run:
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        iy = idx // NX; ix = idx - iy * NX
        gn = g[n]

        for k in range(8):
            jx = ix + ofx[k]; jy = iy + ofy[k]
            if jx < ixa or jx > ixb or jy < iya or jy > iyb:
                info[0] += 1
                continue
            nextnode = li * NN + jy * NX + jx
            m = nextnode * 9 + k
            wm = wmin if wminf.shape[0] == 0 else wminf[nextnode]
            if capHf[nextnode] < wm and tst[nextnode] != run: continue
            ng = gn + ofs[k] * np.float64(mult[nextnode])
            if direction != 8 and direction != k: ng += bend
            if gst[m] != run or ng < g[m]:
                gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                hm = hfac * _bend_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
                hs = _hpush(hf, hn, hs, ng + hm, np.int64(m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PFv * vbad[idx]) + 0.2 * np.float64(hist[node])
            if via_anchor.shape[0] > 0: vc = vc + np.float64(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                nextnode = lj * NN + idx
                m = nextnode * 9 + 8
                vcl = vc
                if via_extra.shape[0] > 0: vcl = vc + vpr[li * NL + lj] * np.float64(via_extra[idx])
                ng = gn + vcl
                if gst[m] != run or ng < g[m]:
                    gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                    hs = _hpush(hf, hn, hs, ng + hfac * _bend_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend), np.int64(m))
    return np.empty(0, dtype=np.int64)



_BEND_SEARCH = None
_BEND_CPU = 0.; _BEND_CALLS = 0; _BEND_POP = 0; _BEND_DOM = 0
if NB is not None:
    from router.astar_nb import _hpush, _hpop
    import numba as _bend_numba
    _bend_h = _bend_numba.njit(cache=True, nogil=True)(_bend_h)
    bend_core = _bend_numba.njit(cache=True, nogil=True)(bend_core)

def bend_search(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                 win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                 bend=0., via_extra=None, vpr=None, via_anchor=None):
    global _BEND_SEARCH, _BEND_CPU, _BEND_CALLS, _BEND_POP, _BEND_DOM
    if _BEND_SEARCH is None:
        _BEND_SEARCH = _Search(nl, nn, res, direction=True)
        _BEND_SEARCH.info = np.zeros(3, np.int64)
    search = _BEND_SEARCH
    xa, xb, ya, yb = win
    vtbox = tuple(tbox)
    hfac = min(float(hfac), max(0., float(mult.reshape(nl, nn//nx, nx)[:,ya:yb+1,xa:xb+1].min())))
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tbox = (min(tbox[0], min(x for x,y in points)), min(tbox[1], min(y for x,y in points)),
                max(tbox[2], max(x for x,y in points)), max(tbox[3], max(y for x,y in points)))
    search.run += 1; search.info[:] = 0
    if search.run >= 2**31-2:
        search.gst[:] = 0; search.cst[:] = 0; search.tst[:] = 0; search.run = 1
    from router.astar_nb import OFX, OFY
    start = time.process_time()
    path = bend_core(np.asarray(src, np.int64), np.asarray(sorted(tgt), np.int64), np.asarray(tbox, np.float64),
        cap, mult, via, vbad, hist, float(wmin), float(vc), float(pf), bool(via_target), np.asarray(win,np.int64),
        nx,nn,nl,float(x0),float(y0),float(res),search.g,search.par,search.gst,search.cst,search.tst,np.int32(search.run),
        OFX,OFY,search.ofs,np.asarray(wminf,np.float32),float(hfac),search.info,
        _EMPTY if viaz is None else viaz,float(vzk),float(bend),_EMPTY if via_extra is None else via_extra,
        np.zeros(1,np.float64) if vpr is None else vpr,_EMPTY if via_anchor is None else via_anchor,np.asarray(vtbox,np.float64))
    _BEND_CPU += time.process_time()-start; _BEND_CALLS += 1
    _BEND_POP += int(search.info[1]); _BEND_DOM += int(search.info[2])
    NB.info[0] = search.info[0]
    return path.tolist() if len(path) else None

def astar_pf(src, tgt_set, tgt_box, capH, capS, viaH, viaBad, C, window, via_target=False, extra=None, viaz=None, vzk=0.0):
    work_step('astar')
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    x0, y0, x1, y1 = window
    ixa, ixb = max(1, int((x0 - X0) / RES)), min(NX - 2, int((x1 - X0) / RES))
    iya, iyb = max(1, int((y0 - Y0) / RES)), min(NY - 2, int((y1 - Y0) / RES))
    w, wmin = C['w'], C['wmin']

    category = (NC.get('net_list', {}).get(NET_CUR[0], {}).get('category') if NC is not None else None)
    turn = BEND * (BEND_CURRENT if category in ('current_val','supply','sw','gnd') else 1.)
    WMINF = neck_field_map(NET_CUR[0], C, wmin, via_target)
    if C.get('current_w'): WMINF = current_floor(NET_CUR[0], C)
    if C.get('zw'):
        WMINF = np.repeat(np.asarray(C['zw'], dtype=np.float32), NN)
    capHf = capH.planar if getattr(capH, 'planar', None) is not None else np.concatenate([c.ravel() for c in capH])
    if HARD_FOREIGN[0] or (CURRENT_FIRST and not current_model(NET_CUR[0])):


        def vacant(n): return capHf[n] >= (wmin if WMINF is None else WMINF[n])
        src = [n for n in src if vacant(n)]
        tgt_set = {n for n in tgt_set if vacant(n)}
        if not src or (not via_target and not tgt_set): return None
    elif SOURCE_FREE[0]:
        def vacant(n):
            wm = wmin if WMINF is None else WMINF[n]
            return min(capHf[n], capS[n // NN].ravel()[n % NN]) >= wm
        free_src = [n for n in src if vacant(n)]
        free_tgt = {n for n in tgt_set if vacant(n)}

        if free_src: src = free_src
        if free_tgt: tgt_set = free_tgt
    hw = C.get('hist_weight', 1.0)


    mult = _MULT
    _key = (id(capH), id(capS), id(extra), w, wmin, C['inner'], C['pen'], hw, PF[0], C.get('lf'), C.get('zw'), C.get('current_w'), C.get('current_clearance'))
    _c = ASTAR_COST[0]
    if not (_c is not None and _c[0] == _key and _c[1][0] <= iya and iyb <= _c[1][1] and _c[1][2] <= ixa and ixb <= _c[1][3]):
        fy0, fy1, fx0, fx1 = iya, iyb, ixa, ixb
        for _f in (ASTAR_HINT[0], _c[1] if _c is not None and _c[0] == _key else None):
            if _f is not None: fy0, fy1, fx0, fx1 = min(fy0, _f[0]), max(fy1, _f[1]), min(fx0, _f[2]), max(fx1, _f[3])
        lf = C.get('lf') or ([1.0] + [C['inner']] * (NL - 2) + [1.0])
        ys = slice(fy0, fy1 + 1); xs = slice(fx0, fx1 + 1)
        for li in range(NL):
            if C.get('zw'): w = wmin = C['zw'][li]
            if C.get('current_w'): w = wmin = C['current_w'][li]
            sl = slice(li * NN, (li + 1) * NN)
            c = np.minimum(capH[li][ys, xs], capS[li][ys, xs])
            m = (1.0 + C['pen'] * np.clip((w - c) / w, 0, 1)) * lf[li]


            _tight = float(args.get('cost_tight', '0')) if NC is not None else 0.
            if _tight:
                m *= 1. + _tight * np.clip(1. - c / (2*w + .2), 0., 1.)
            if C.get('zw') and IMP_REF_MASK:
                _zp = IMP_TAB.get(NET_CUR[0], {}).get(LAYERS[li])
                if _zp:
                    _mask = IMP_REF_MASK[NET_CUR[0], LAYERS[li]][ys, xs]
                    m = np.where(_mask, m, m * 8.0)
            if FL_PRICE > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_GAP), m * (1.0 + FL_PRICE * (e - CLR)), m)
            elif AREA > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_GAP), m * (1.0 + AREA), m)
            if li == 0: m = np.where(under_esp[ys, xs], m * 1.8, m)
            m = (m * (1.0 + PF[0] * (c < wmin))).astype(np.float32)
            m *= (1.0 + hw * hist[sl].reshape(NY, NX)[ys, xs])
            if C.get('current_clearance') and CURRENT_DETOUR:
                ax,ay,bx,by = C['current_clearance']; gx,gy = np.meshgrid(GXc[xs],GYc[ys])
                clearance = math.hypot(bx-ax,by-ay)
                more = np.maximum(0.,np.hypot(gx-ax,gy-ay)+np.hypot(gx-bx,gy-by)-clearance)
                m *= (1. + CURRENT_DETOUR * more / max(1.,clearance)).astype(np.float32)
            if extra is not None: m *= extra[sl].reshape(NY, NX)[ys, xs]
            mult[sl].reshape(NY, NX)[ys, xs] = m

        ASTAR_COST[0] = (_key, (fy0, fy1, fx0, fx1), capH, capS, extra)
    viaf = viaH.ravel(); vbad = viaBad.ravel()
    if C.get('current_w'):
        viaf = current_field_mask(viaH, max(2, math.ceil(4*C['current_I'])), C, capH).ravel()
    if VIA_MASK[0] is not None: viaf = viaf & VIA_MASK[0]
    VC = C['via'] * VIA_FACTOR[0]
    _vz = VIA_TOO[0] if not via_target else None
    via_extra, vpr, via_anchor = _vz if _vz is not None else (None, None, None)

    bounded_loop = globals().get('_LOOP_DEADLINE') is not None and not BUDGET_WORK
    if NB is not None and not bounded_loop and turn > 0:
        return bend_search(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
            (ixa,ixb,iya,iyb),NX,NN,NL,X0,Y0,RES,_EMPTY if WMINF is None else WMINF,HFAC,viaz,vzk,turn,via_extra,vpr,via_anchor)
    if NB is not None and not bounded_loop:
        return NB(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                  (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _EMPTY if WMINF is None else WMINF, HFAC, viaz, vzk,
                  turn, via_extra, vpr, via_anchor)
    if turn > 0:

        return astar_direction_python(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                                (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _EMPTY if WMINF is None else WMINF,
                                HFAC, viaz, vzk, turn, via_extra, vpr, via_anchor)
    tx0, ty0, tx1, ty1 = tgt_box
    def h(n):
        idx = n % NN; ix = idx % NX; iy = idx // NX
        x, y = X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES
        dx = max(tx0 - x, 0, x - tx1); dy = max(ty0 - y, 0, y - ty1)
        return math.sqrt(dx * dx + dy * dy) if turn > 0.0 else math.hypot(dx, dy)
    g = {}; parent = {}; openh = []
    for n in src:
        g[n] = 0.0; parent[n] = -1; heapq.heappush(openh, (0.0 if via_target else HFAC * h(n), n))
    closed = set()
    loop_poll = 0
    while openh:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        f, n = heapq.heappop(openh)
        if n < 0:
            n = -n - 1; path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        if n in closed: continue
        closed.add(n)
        idx = n % NN
        if via_target:
            if n in tgt_set:
                path = []
                while n != -1: path.append(n); n = parent[n]
                return path[::-1]
            if viaf[idx] and n // NN in (0, NL - 1):
                heapq.heappush(openh, (g[n] + 0.3 + 2.0 * h(n) + (3.0 * PF[0] if vbad[idx] else 0.0)
                                       + (vzk * float(viaz[idx]) if viaz is not None else 0.0), -n - 1))
        elif n in tgt_set:
            path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        li = n // NN; iy, ix = divmod(idx, NX)
        gn = g[n]
        pdx = 0; pdy = 0; has_direction = False
        if turn > 0.0 and parent[n] >= 0 and parent[n] // NN == li:
            pq = parent[n] % NN
            pdx = ix - pq % NX; pdy = iy - pq // NX; has_direction = True
        for dx, dy, st in OFFS:
            jx, jy = ix + dx, iy + dy
            if jx < ixa or jx > ixb or jy < iya or jy > iyb: continue
            m = li * NN + jy * NX + jx
            if capHf[m] < (wmin if WMINF is None else WMINF[m]) and m not in tgt_set: continue
            ng = gn + st * float(mult[m])
            if has_direction and (dx != pdx or dy != pdy): ng += turn
            if ng < g.get(m, 1e18):
                g[m] = ng; parent[m] = n
                heapq.heappush(openh, (ng + (0.0 if via_target else HFAC * h(m)), m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PF[0] * vbad[idx]) + 0.2 * float(hist[n])
            if via_anchor is not None: vc = vc + float(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                m = lj * NN + idx
                vcl = vc
                if via_extra is not None: vcl = vc + float(vpr[li * NL + lj]) * float(via_extra[idx])
                ng = gn + vcl
                if ng < g.get(m, 1e18):
                    g[m] = ng; parent[m] = n
                    heapq.heappush(openh, (ng + HFAC * h(m), m))
    return None


def astar_direction_python(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                          win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                          bend=0., via_extra=None, vpr=None, via_anchor=None):
    ny = nn // nx
    xa, xb, ya, yb = win
    tx0, ty0, tx1, ty1 = tbox
    vx0, vy0, vx1, vy1 = tbox
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tx0 = min(tx0, min(x for x, y in points)); tx1 = max(tx1, max(x for x, y in points))
        ty0 = min(ty0, min(y for x, y in points)); ty1 = max(ty1, max(y for x, y in points))
    hfac = min(1., float(hfac), max(0., float(mult.reshape(nl, ny, nx)[:, ya:yb+1, xa:xb+1].min()))) * (1.4142 / math.sqrt(2.))
    def h(node):
        idx = node % nn
        x, y = x0+(idx % nx+.5)*res, y0+(idx // nx+.5)*res
        dx, dy = max(tx0-x, 0., x-tx1), max(ty0-y, 0., y-ty1)
        return max(dx,dy)+(1.4142-1.)*min(dx,dy)
    directions = ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))
    g, parent, closed, queue = {}, {}, set(), []
    for node in src:
        s = node*9+8
        g[s] = 0.; parent[s] = -1
        heapq.heappush(queue, (hfac*h(node), s))
    def relax(s, cost, previous, heuristic):
        if cost < g.get(s, float('inf')):
            g[s] = cost; parent[s] = previous; closed.discard(s)
            heapq.heappush(queue, (cost+heuristic, s))
    def path(s):
        result = []
        while s != -1:
            result.append(s//9); s = parent[s]
        return result[::-1]
    bounded_loop = globals().get('_LOOP_DEADLINE') is not None
    loop_poll = 0
    while queue:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        _, s = heapq.heappop(queue)
        if s < 0: return path(-s-1)
        if s in closed: continue
        node, direction = divmod(s, 9)
        if any(other != s and g.get(other, float('inf')) + (0. if d == 8 else bend) < g[s]
               for d in range(9) for other in (node*9+d,)): continue
        closed.add(s)
        layer, idx = divmod(node, nn)
        if node in tgt: return path(s)
        if via_target and via[idx] and layer in (0, nl-1):
            extra = vzk*float(viaz[idx]) if viaz is not None and len(viaz) else 0.
            heapq.heappush(queue, (g[s]+.3+2.*math.sqrt(max(vx0-(x0+(idx % nx+.5)*res),0.,(x0+(idx % nx+.5)*res)-vx1)**2+
                max(vy0-(y0+(idx // nx+.5)*res),0.,(y0+(idx // nx+.5)*res)-vy1)**2)+(3.*pf if vbad[idx] else 0.)+extra, -s-1))
        y, x = divmod(idx, nx)
        for k, (dx, dy) in enumerate(directions):
            xx, yy = x+dx, y+dy
            if not (xa <= xx <= xb and ya <= yy <= yb): continue
            target = layer*nn+yy*nx+xx
            wm = wmin if not len(wminf) else wminf[target]
            if cap[target] < wm and target not in tgt: continue
            cost = g[s]+(res*1.4142 if k >= 4 else res)*float(mult[target])
            if direction != 8 and direction != k: cost += bend
            relax(target*9+k, cost, s, hfac*h(target))
        if via[idx] and not via_target:
            extra = float(via_anchor[idx]) if via_anchor is not None and len(via_anchor) else 0.
            cost = vc*(1.+3.*pf*vbad[idx])+.2*float(hist[node])+extra
            for ll in range(nl):
                if ll == layer: continue
                target = ll*nn+idx
                change = cost
                if via_extra is not None and len(via_extra): change += vpr[layer*nl+ll]*float(via_extra[idx])
                relax(target*9+8, g[s]+change, s, hfac*h(target))
    return None



_PADG = {li: [] for li in range(NL)}
_KA_PAD_KEEP = {li: [] for li in range(NL)}
_PADXY = {li: [] for li in range(NL)}
for _p in D['pads']:
    for _L in _p['layers']:
        if _L in LI:
            _PADXY[LI[_L]].append((_p['x'], _p['y']) if not _p.get('drill') else None)
            _PADG[LI[_L]].append((_p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p),
                                  shapely.union_all([Polygon(q) for q in _p['polys'][_L]])))
            if globals().get('KA') is not None:
                _KA_META[id(_PADG[LI[_L]][-1][1])] = _ka_pad_kind(_p)
                for _g,_kind in _ka_pad_keeps(_p,_L):
                    _KA_META[id(_g)]=(_kind,'')
                    _KA_PAD_KEEP[LI[_L]].append((_PADG[LI[_L]][-1][0],_g,_kind))
_PADT = {li: shapely.STRtree([g for _, g in _PADG[li]]) for li in range(NL)}





VPL = args.get('via_slots', '1' if NC else '0') == '1'
VP = {}
if VPL:
    from scipy.spatial import cKDTree as _KD2
    _jr = {}
    for _p in D['pads']:
        if not _p.get('drill') and any(L in LI for L in _p['layers']): _jr.setdefault(_p['ref'], []).append(_p)
    _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
    for _r in sorted(_jr):
        _P = _jr[_r]
        if len(_P) < 16 or any(max(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) > 1.3 * min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) for q in _P): continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _d, _i = _KD2(_xy).query(_xy, k=2); _pit = float(np.median(_d[:, 1]))
        if not 0.3 <= _pit <= 1.05: continue
        _v = _xy[_i[:, 1]] - _xy; _w = np.arctan2(_v[:, 1], _v[:, 0])
        _th = float(np.angle(np.mean(np.exp(4j * _w)))) / 4
        _e1 = (math.cos(_th) * _pit / 2, math.sin(_th) * _pit / 2); _e2 = (-_e1[1], _e1[0])
        _places = sorted({(round(x + a * _e1[0] + b * _e2[0], 3), round(y + a * _e1[1] + b * _e2[1], 3)) for x, y in _xy.tolist() for a in (-1, 1) for b in (-1, 1)})
        for _x, _y in _places:
            _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
            if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
            _pt = shapely.Point(_x, _y); _kr = _pt.buffer(_rn); _close = []
            for _li in range(NL):
                for _j in _PADT[_li].query(_kr):
                    _n2, _g2 = _PADG[_li][int(_j)]; _dd = float(_pt.distance(_g2))
                    if _dd < _rn: _close.append((_n2, round(_dd, 4), _li in (0, NL - 1)))
            VP[_iy * NX + _ix] = (_x, _y, tuple(sorted(_close)))
    if VP: print('exact via slots in the ball grid:', len(VP), flush=True)



BGA = {}
if BGA_FANOUT:
    from scipy.spatial import cKDTree as _BKD
    _bf = {f['ref']: f for f in D.get('fps', []) if 'BGA' in (f.get('fp', '') + f.get('lib', '')).upper()}
    for _ref in sorted(_bf):
        _bp = [p for p in D['pads'] if p['ref'] == _ref and not p.get('drill') and len(p['layers']) == 1 and p['layers'][0] in LI]
        if len(_bp) < 16: continue
        _xy = np.array([(p['x'], p['y']) for p in _bp]); _bd, _bi = _BKD(_xy).query(_xy, k=2)
        _pitch = float(np.median(_bd[:, 1]))
        if not 0.3 <= _pitch <= 1.05: continue
        _v = _xy[_bi[:, 1]] - _xy; _th = float(np.angle(np.mean(np.exp(4j * np.arctan2(_v[:, 1], _v[:, 0]))))) / 4
        _rot = np.array([[math.cos(_th), -math.sin(_th)], [math.sin(_th), math.cos(_th)]])
        _local = _xy @ _rot; _origin = _local.min(axis=0)
        _ij = np.rint((_local - _origin) / _pitch).astype(int)
        if np.max(np.abs(_local - _origin - _ij * _pitch)) > 0.02: continue
        _mx, _my = _ij.max(axis=0); _mid = (_local.min(axis=0) + _local.max(axis=0)) / 2
        for _p, (_i, _j), _lp in zip(_bp, _ij.tolist(), _local):
            _ring = min(_i, _j, int(_mx) - _i, int(_my) - _j)
            _sx, _sy = (1 if _lp[0] >= _mid[0] else -1), (1 if _lp[1] >= _mid[1] else -1)
            _signs = [(_sx, _sy), (_sx, -_sy), (-_sx, _sy), (-_sx, -_sy)]
            _dog = [tuple(np.round(np.array([_p['x'], _p['y']]) + (np.array([a, b]) * _pitch / 2) @ _rot.T, 5)) for a, b in _signs]
            _vip = [(_p['x'], _p['y'])] if VIP else []
            _pts = (_vip if _pitch <= 0.5 + 1e-6 else _dog + _vip)
            BGA[id(_p)] = dict(ref=_ref, pad=_p['num'], ring=_ring, pitch=round(_pitch, 5), layer=_p['layers'][0], pts=[])
            for _x, _y in _pts:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                _c = _iy * NX + _ix
                if _c in VP and math.dist((_x, _y), VP[_c][:2]) > 1e-5: continue
                _pt = shapely.Point(_x, _y); _close = []; _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
                for _li in range(NL):
                    for _k in _PADT[_li].query(_pt.buffer(_rn)):
                        _n2, _g2 = _PADG[_li][int(_k)]; _dist = float(_pt.distance(_g2))
                        if _dist < _rn: _close.append((_n2, round(_dist, 4), _li in (0, NL - 1)))
                VP[_c] = (_x, _y, tuple(sorted(_close)))
                BGA[id(_p)]['pts'].append(_c)
    print('BGA fanout (option): %d balls, %d inner from ring 2' % (len(BGA), sum(v['ring'] >= 2 for v in BGA.values())), flush=True)
_VP_C = np.array(sorted(VP), dtype=np.int64); _VP_IY, _VP_IX = _VP_C // NX, _VP_C % NX







LANES = (args.get('lanes', '0') == '1' or BGA_TIGHT) and NC is not None
GS_PT = {}
GS_N = 0
if LANES:
    from shapely.ops import nearest_points as _np_pts
    _wg = float(args.get('lanes_w', '%g' % cls('/__signal__')['w'])); _rg = DRCP + PAD_EXTRA
    _gmin = _wg + 2 * _rg; _gmax = _wg + 2 * CLR_PAD + 2 * QM
    _nid = lambda n_: NID[n_] if n_ in NID else -2
    for _li in range(NL):
        _G = _PADG[_li]; _T = _PADT[_li]; _best = {}
        for _i, (_n1, _g1) in enumerate(_G):
            for _j in sorted(int(q) for q in _T.query(_g1.buffer(_gmax))):
                if _j <= _i: continue
                _n2, _g2 = _G[_j]
                if _n1 == _n2: continue
                _d = float(_g1.distance(_g2))
                if not (_gmin <= _d < _gmax): continue
                _pa, _pb = _np_pts(_g1, _g2); _ux, _uy = (_pb.x - _pa.x) / _d, (_pb.y - _pa.y) / _d
                _mx, _my = (_pa.x + _pb.x) / 2, (_pa.y + _pb.y) / 2; _vx, _vy = -_uy, _ux
                _b1, _b2 = _g1.bounds, _g2.bounds
                _L = max(_b1[2] - _b1[0], _b1[3] - _b1[1], _b2[2] - _b2[0], _b2[3] - _b2[1]) / 2 + 0.3
                _k = int(_L / (RES / 4))
                for _t in range(-_k, _k + 1):
                    _x, _y = _mx + _vx * _t * RES / 4, _my + _vy * _t * RES / 4
                    _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                    if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                    _c = _iy * NX + _ix
                    if 2 * (float(PD1[_li].ravel()[_c]) - CLR_PAD) >= _wg: continue
                    _pt = shapely.Point(_x, _y)
                    _dm = min(float(_G[int(q)][1].distance(_pt)) for q in _T.query(_pt.buffer(_gmax)))
                    _cap = 2 * (_dm - _rg)
                    if _cap < _wg - 1e-9: continue
                    _o = (abs(_x - cell_xy(_ix, _iy)[0]) + abs(_y - cell_xy(_ix, _iy)[1]), -_cap)
                    if _c not in _best or _o < _best[_c][0]: _best[_c] = (_o, _x, _y, _cap, _nid(_n1), _nid(_n2))
        for _c, (_o, _x, _y, _cap, _a, _b) in sorted(_best.items()): GS_PT[_li * NN + _c] = (round(_x, 4), round(_y, 4), _a, _b, _cap)
    GS_N = len(GS_PT)
    _GS_I = np.array(sorted(GS_PT), dtype=np.int64)
    _GS_CAP = np.array([GS_PT[c][4] for c in _GS_I.tolist()], dtype=np.float32)
    _GS_A = np.array([GS_PT[c][2] for c in _GS_I.tolist()], dtype=np.int64); _GS_B = np.array([GS_PT[c][3] for c in _GS_I.tolist()], dtype=np.int64)
    print('narrow lanes (R15, open 29): %d cells walkable on the exact centerline (track %.3f mm)' % (GS_N, _wg), flush=True)


def _lanes_one(net, capH, rng):
    nid = NID[net]; m = (_GS_A != nid) & (_GS_B != nid)
    idx = _GS_I[m]; cap = _GS_CAP[m]; ic = idx % NN; li = idx // NN
    if rng is not None:
        ix, iy = ic % NX, ic // NX; w = (ix >= rng[0]) & (ix < rng[1]) & (iy >= rng[2]) & (iy < rng[3]); idx, cap, ic, li = idx[w], cap[w], ic[w], li[w]
    if not len(idx): return
    cap = np.minimum(cap, 2 * (edge_d.ravel()[ic] - EDGE)).astype(np.float32)
    if FIXD[0] is not None and net not in FIXNETS:
        cap = np.minimum(cap, np.array([2 * (FIXD[0][l].ravel()[c] - QM - CLR - 0.005) for l, c in zip(li.tolist(), ic.tolist())], dtype=np.float32))
    capH.planar[idx] = np.maximum(capH.planar[idx], cap)

VIP_EP = (not VIP) and args.get('vip_ep', '1' if NC else '0') == '1'
EPZ = {}



EP_VIAS = bool(AREA_NETS) and args.get('ep_vias', '1' if NC else '0') == '1'
EP_DIST = max(float(args.get('ep_spacing', '%g' % (VIA_D + 0.3))), HOLE_C)
EP_DIST_K12 = max(EP_DIST, VIA_D + 2 * CLR + 0.2)
EPM = {}
EPG = []
_ep = {}; _epk = {}
if VIP_EP or EP_VIAS:
    _jr3 = {}; _jk3 = {}
    for _k, _p in enumerate(D['pads']):
        if not _p.get('drill'): _jr3.setdefault(_p['ref'], []).append(_p); _jk3[id(_p)] = _k
    for _r in sorted(_jr3):
        _P = _jr3[_r]
        if len(_P) < 8: continue
        for _p in _P:
            _lg = [L for L in _p['layers'] if L in LI and LI[L] in (0, NL - 1)]; _b = _p['bbox']
            if not _p['net'] or _p['net'].startswith('unconnected') or len(_lg) != 1 or min(_b[2] - _b[0], _b[3] - _b[1]) < 1.0: continue
            _xs = [q['x'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            _ys = [q['y'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            if len(_xs) < 6 or not (min(_xs) < _p['x'] < max(_xs) and min(_ys) < _p['y'] < max(_ys)): continue
            _ep.setdefault((_r, _p['net'], LI[_lg[0]]), []).extend(Polygon(q) for q in _p['polys'][_lg[0]] if len(q) >= 3)
            _epk.setdefault((_r, _p['net'], LI[_lg[0]], _p['num']), []).append(_jk3[id(_p)])
if VIP_EP:
    for (_r, _n, _li), _pl in sorted(_ep.items()):
        _g = shapely.union_all(_pl).buffer(-VIA_R)
        if _g.is_empty: continue
        _b = _g.bounds
        _ixa, _ixb = max(1, int((_b[0] - X0) / RES)), min(NX - 1, int((_b[2] - X0) / RES) + 1)
        _iya, _iyb = max(1, int((_b[1] - Y0) / RES)), min(NY - 1, int((_b[3] - Y0) / RES) + 1)
        if _ixb <= _ixa or _iyb <= _iya: continue
        _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
        _in = shapely.contains_xy(_g, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
        if _in.any(): EPZ.setdefault(_n, []).append((_li, _GY.ravel()[_in].astype(np.int64), _GX.ravel()[_in].astype(np.int64)))
    if EPZ: print('Thermal pads with allowed via in pad:', sum(len(v) for v in EPZ.values()), flush=True)


def _ep_axis(W):
    return max(1, int((W - VIA_D + 1e-9) / EP_DIST) + 1) if W >= VIA_D else 1


def _ep_split(W, n):
    return 0.0 if n < 2 else min((W - VIA_D) / (n - 1), max(W / n, EP_DIST_K12))


if EP_VIAS:
    for (_r, _n, _li, _num), _ks in sorted(_epk.items()):
        if _n not in AREA_NETS: continue
        _L = LAYERS[_li]
        _gf = [shapely.union_all([Polygon(q) for q in D['pads'][_k]['polys'][_L] if len(q) >= 3]) for _k in _ks]
        _g = shapely.union_all(_gf); _A = float(_g.area)
        _nominal = 4 if _A < 9.0 else (9 if _A < 25.0 else 16)
        _per = -(-_nominal // len(_ks))
        EPG.append(dict(ref=_r, num=_num, net_name=_n, surface=round(_A, 2), nominal=_nominal, fields=len(_ks), poly=_g.buffer(0.02), pads=list(_ks)))
        if _per < 2: continue
        for _k, _gk in zip(_ks, _gf):
            _b = _gk.bounds; _W, _H = _b[2] - _b[0], _b[3] - _b[1]; _cx, _cy = (_b[0] + _b[2]) / 2, (_b[1] + _b[3]) / 2
            _best = None
            for _nx in range(1, _ep_axis(_W) + 1):
                for _ny in range(1, _ep_axis(_H) + 1):
                    _q = (max(0, _per - _nx * _ny), max(0, _nx * _ny - _per), abs(_ep_split(_W, _nx) - _ep_split(_H, _ny)), _nx, _ny)
                    if _best is None or _q < _best: _best = _q
            _nx, _ny = _best[3], _best[4]; _px, _py = _ep_split(_W, _nx), _ep_split(_H, _ny)
            _gi = _gk.buffer(-VIA_R + 0.002)
            if _gi.is_empty: continue
            _places = []
            for _j in range(_ny):
                for _i in range(_nx):
                    _ix, _iy = int((_cx + (_i - (_nx - 1) / 2) * _px - X0) / RES), int((_cy + (_j - (_ny - 1) / 2) * _py - Y0) / RES)
                    if 0 < _ix < NX - 1 and 0 < _iy < NY - 1 and _iy * NX + _ix not in _places: _places.append(_iy * NX + _ix)
            _bi = _gi.bounds
            _ixa, _ixb = max(1, int((_bi[0] - X0) / RES)), min(NX - 1, int((_bi[2] - X0) / RES) + 1)
            _iya, _iyb = max(1, int((_bi[1] - Y0) / RES)), min(NY - 1, int((_bi[3] - Y0) / RES) + 1)
            if _ixb <= _ixa or _iyb <= _iya: continue
            _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
            _in = shapely.contains_xy(_gi, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
            EPM[_k] = dict(nominal=_nx * _ny, wish=_per, places=_places, grid_cells=(_GY.ravel()[_in] * NX + _GX.ravel()[_in]).astype(np.int64),
                           split=(round(_px, 3), round(_py, 3)))
    if EPG: print('Ground pads under chips (R16):', ', '.join('%s:%s %.1f mm2 target %d' % (e['ref'], e['num'], e['surface'], e['nominal']) for e in EPG), flush=True)



EP_RES = {}
for _k, _E in sorted(EPM.items()):
    _m = EP_RES.setdefault(D['pads'][_k]['net'], np.zeros((NY, NX), dtype=bool)); _rr = int(EP_DIST / RES) + 1
    for _c in _E['places']:
        _iy, _ix = divmod(_c, NX)
        _ya, _yb, _xa, _xb = max(0, _iy - _rr), min(NY, _iy + _rr + 1), max(0, _ix - _rr), min(NX, _ix + _rr + 1)
        _GX, _GY = np.meshgrid(np.arange(_xa, _xb), np.arange(_ya, _yb))
        _m[_ya:_yb, _xa:_xb] |= np.hypot(_GX - _ix, _GY - _iy) * RES < EP_DIST - 1e-6


def via_hard(net, cells):
    if not len(cells): return False
    if not LARGE: return bool(hard_maps(net)[1].ravel()[cells].any())
    iy, ix = cells // NX, cells % NX
    rng = (max(0, int(ix.min()) - 2), min(NX, int(ix.max()) + 3), max(0, int(iy.min()) - 2), min(NY, int(iy.max()) + 3))
    return bool(hard_maps(net, rng)[1].ravel()[cells].any())


def ep_set(k, viaH, vbadf, rng):
    E = EPM[k]; viaf = viaH.ravel(); Z = E['grid_cells']
    def applies(c): return rng[0] < c % NX < rng[1] - 1 and rng[2] < c // NX < rng[3] - 1
    zx = X0 + (Z % NX + 0.5) * RES; zy = Y0 + (Z // NX + 0.5) * RES
    vacant = viaf[Z] & ~vbadf[Z] & (Z % NX > rng[0]) & (Z % NX < rng[1] - 1) & (Z // NX > rng[2]) & (Z // NX < rng[3] - 1) if len(Z) else None
    def attempt(ab):
        assigned = []; open_items = []
        def wide(c):
            x, y = cell_xy(c % NX, c // NX)
            return all(math.hypot(x - a, y - b) >= EP_DIST - 1e-6 for a, b in assigned)
        for c in E['places']:
            if applies(c) and viaf[c] and not vbadf[c] and wide(c): assigned.append(cell_xy(c % NX, c // NX))
            else: open_items.append(c)
        if open_items and vacant is not None:
            rest = []
            for c in open_items:
                x, y = cell_xy(c % NX, c // NX); m = vacant.copy()
                for a, b in assigned: m &= np.hypot(zx - a, zy - b) >= ab - 1e-6
                if not m.any(): rest.append(c); continue
                d = np.hypot(zx - x, zy - y); d[~m] = 1e9; j = int(np.argmin(d))
                assigned.append((float(zx[j]), float(zy[j])))
            open_items = rest
        if CONS_SOFT[0]:
            for c in open_items:
                if applies(c) and viaf[c] and wide(c): assigned.append(cell_xy(c % NX, c // NX))
        return assigned
    assigned = attempt(EP_DIST)
    if EP_DIST_K12 > EP_DIST + 1e-9:
        g2 = attempt(EP_DIST_K12)
        if len(g2) >= len(assigned): assigned = g2
    return [dict(x=round(x, 4), y=round(y, 4)) for x, y in assigned]







VAP = VIP and bool(AREA_NETS) and args.get('via_at_pad', '1' if NC else '0') == '1'
VAP_K = 3.0 * float(args.get('via_at_pad_mm', '1.5'))
VAP_THIRD = float(args.get('via_at_pad_third', '0.25'))
VAP_WF = float(args.get('via_at_pad_important', '2.0'))
VAP_IMPORTANT = set()
_VZ = np.ones(NN, dtype=np.float32) if VAP else None
_VAPZ = {}
if VAP and NC:
    _wr = {r for r, t in NC.get('part_list', {}).items() if t.get('decoupling')}
    _wr |= {p['ref'] for p in D['pads'] if (NC['net_list'].get(p['net']) or {}).get('category') == 'sw'}
    VAP_IMPORTANT = {k for k, p in enumerate(D['pads']) if p['net'] in AREA_NETS and p['ref'] in _wr}
if VAP: print('Via at pad (R9): a via next to the pad costs %.1f (like %.1f mm stub), %d of %d pads of the area nets count %gfold (decoupling, switching node)' % (
    VAP_K, VAP_K / 3.0, len(VAP_IMPORTANT), sum(len(by_net_g.get(n, ())) for n in AREA_NETS), VAP_WF), flush=True)


def vap_share(g, xs, ys):
    if not len(xs): return np.zeros(0)
    kr = shapely.buffer(shapely.points(xs, ys), VIA_R, quad_segs=8)
    return shapely.area(shapely.intersection(kr, g)) / min(float(shapely.area(kr[0])), float(g.area))


def _vap_cells(k):
    r = _VAPZ.get(k)
    if r is None:
        p = D['pads'][k]; L = next(x for x in p['layers'] if x in LI)
        g = shapely.union_all([Polygon(q) for q in p['polys'][L] if len(q) >= 3]); b = g.bounds
        ixa, ixb = max(0, int((b[0] - VIA_R - X0) / RES)), min(NX, int((b[2] + VIA_R - X0) / RES) + 1)
        iya, iyb = max(0, int((b[1] - VIA_R - Y0) / RES)), min(NY, int((b[3] + VIA_R - Y0) / RES) + 1)
        GX, GY = np.meshgrid(np.arange(ixa, ixb), np.arange(iya, iyb)); gx = GX.ravel(); gy = GY.ravel()
        x = X0 + (gx + 0.5) * RES; y = Y0 + (gy + 0.5) * RES
        close = shapely.distance(g, shapely.points(x, y)) < VIA_R
        a = vap_share(g, x[close], y[close]); m = a >= 1 / 3 - 0.01
        r = _VAPZ[k] = ((gy[close][m] * NX + gx[close][m]).astype(np.int64), np.where(a[m] >= 0.49, 0.0, VAP_THIRD).astype(np.float32))
    return r


def _ep_one(net, viaH, capH, rng):
    for li_e, cy_, cx_ in EPZ.get(net, ()):
        m = ~viaH[cy_, cx_]
        if rng is not None: m &= (cx_ >= rng[0]) & (cx_ < rng[1]) & (cy_ >= rng[2]) & (cy_ < rng[3])
        if not m.any(): continue
        cy2, cx2 = cy_[m], cx_[m]
        ok = (edge_d[cy2, cx2] - EDGE >= VIA_R) & (VSP[cy2, cx2] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_BAN is not None: ok &= ~VIA_BAN[cy2, cx2]
        if FIXD[1] is not None and net not in FIXNETS: ok &= FIXD[1][cy2, cx2] - QM >= HOLE_C
        for li in range(NL):
            ok &= capH[li][cy2, cx2] / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and li != li_e: ok &= padd[li][cy2, cx2] >= VIA_R + VIA_WEB
        viaH[cy2[ok], cx2[ok]] = True


def _vp_ok(net, close):
    for n2, d, outer in close:
        if n2 != net:
            if d < VIA_R + DRCP + 0.006: return False
            if BGA_HOLE and d < VIA_DR / 2 + HOLE_CLR + 0.000001: return False
        elif outer and not VIP and d < VIA_R + VIA_WEB: return False
    return True
def _vp_xy(net, c):
    if globals().get('KA') is not None:
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _ka_via_ok(net,v[0],v[1]) else None
    if PIN_ACCESS and net in PLANE_AREA and net not in globals().get('PLANE_MIXED_NETS',()) and c in VP and not PLANE_AREA[net].covers(_PT(*VP[c][:2])): return None
    if _pz_active(net) and c in VP and _pz_near(net,*VP[c][:2]):
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _pz_via_ok(net,v[0],v[1]) else None
    if FIXD[0] is not None: return None
    v = VP.get(c)
    return (v[0], v[1]) if v is not None and _vp_ok(net, v[2]) else None


def _vp_one(net, viaH, rng):
    if globals().get('KA') is not None:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            if _ka_via_ok(net,*VP[c][:2]):viaH[iy,ix]=True
        return
    if _pz_active(net) and not _PZ_BASE[0]:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            v=VP[c]
            if not _pz_near(net,v[0],v[1]):continue
            if _pz_via_ok(net,v[0],v[1]): viaH[iy,ix]=True
        return
    if not len(_VP_C) or FIXD[0] is not None: return
    m = ~viaH[_VP_IY, _VP_IX] & (edge_d[_VP_IY, _VP_IX] - EDGE >= VIA_R) & (VSP[_VP_IY, _VP_IX] >= H2H + VIA_DR / 2 + 0.01)
    if VIA_BAN is not None: m &= ~VIA_BAN[_VP_IY, _VP_IX]
    if rng is not None: m &= (_VP_IX >= rng[0]) & (_VP_IX < rng[1]) & (_VP_IY >= rng[2]) & (_VP_IY < rng[3])
    if FIXD[1] is not None and net not in FIXNETS: m &= FIXD[1][_VP_IY, _VP_IX] - QM >= HOLE_C
    for c in _VP_C[m].tolist():
        if PIN_ACCESS and net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
            if not PLANE_MASK[net][c // NX, c % NX] or not PLANE_AREA[net].covers(_PT(*VP[c][:2])): continue
        if _vp_ok(net, VP[c][2]): viaH[c // NX, c % NX] = True


from functools import lru_cache as _lru_cache
PAD_ROOM_CACHE = args.get('pad_room_cache', '1') == '1'


def pad_room(li, a, b, net):

    a, b = tuple(a), tuple(b)
    return _pad_room_cached(li, a, b, net) if PAD_ROOM_CACHE else _pad_room_exact(li, a, b, net)


def _pad_room_exact(li, a, b, net):
    if globals().get('KA') is not None: return _ka_room(net,li,a,b)
    seg = _LS([a, b]) if (abs(a[0] - b[0]) > 1e-9 or abs(a[1] - b[1]) > 1e-9) else _PT(a)
    best = 9.0
    for j in _PADT[li].query(seg.buffer(1.0)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == net: continue
        best = min(best, seg.distance(g2))
    room = best - (DRCP + PAD_EXTRA)
    if _pz_active(net): room = min(best - DRCP - PZ_BORDER_NM / 1e6, _pz_room(net,li,a,b))
    if ZONE_PLAN:
        room = min(room, min((seg.distance(g)-CLR-.005 for (n,L),g in _ZP_GEO.items() if n!=net and L==LAYERS[li]), default=9.0))
    if PLANE_COPPER:
        room=min(room,min((seg.distance(g)-CLR-.005 for (n,L),g in PLANE_COPPER.items() if n!=net and L==LAYERS[li]),default=9.))
    return room


_pad_room_cached = _lru_cache(maxsize=100000)(_pad_room_exact)


def _pad_room_lower_loop(d1, d2, pn, nid, ax, ay, bx, by, x0, y0, res, nx, ny, count):
    best = 9.0
    for k in range(count):
        t = k / (count - 1)
        ix = int(math.floor((ax + t * (bx - ax) - x0) / res))
        iy = int(math.floor((ay + t * (by - ay) - y0) / res))
        if ix < 0 or ix >= nx or iy < 0 or iy >= ny: return -999.0
        d = d2[iy, ix] if pn[iy, ix] == nid else d1[iy, ix]
        if d < best: best = d
    return best


_pad_room_lower_nb = None
if NB is not None:
    from numba import njit as _pad_njit
    _pad_room_lower_nb = _pad_njit(cache=True)(_pad_room_lower_loop)


@_lru_cache(maxsize=100000)
def _pad_room_lower(li, a, b, net):
    L = math.dist(a, b); count = max(2, int(L / (RES * 0.5)) + 1)
    f = _pad_room_lower_nb if _pad_room_lower_nb is not None else _pad_room_lower_loop
    dist = f(PD1[li], PD2[li], PN1[li], NID.get(net, -1), a[0], a[1], b[0], b[1], X0, Y0, RES, NX, NY, count)
    return min(float(dist), RW - 2 * RES) - RES / math.sqrt(2) - L / (2 * (count - 1)) - 1e-5 - DRCP - PAD_EXTRA


def pad_room_min(li, a, b, net, demand):
    if globals().get('KA') is None and PAD_ROOM_CACHE and not ZONE_PLAN and not _pz_active(net):
        if _pad_room_lower(li, tuple(a), tuple(b), net) >= demand: return demand
    return min(demand, pad_room(li, a, b, net))


def line_cap_min(capL, a, b):
    (x0, y0), (x1, y1) = a, b
    L = math.hypot(x1 - x0, y1 - y0); n = max(2, int(L / (RES * 0.5)) + 1)
    if NB is not None:
        return float(_linecap_nb(capL, x0, y0, x1, y1, n, X0, Y0, RES))
    t = np.linspace(0, 1, n)
    ix = ((x0 + t * (x1 - x0) - X0) / RES).astype(int); iy = ((y0 + t * (y1 - y0) - Y0) / RES).astype(int)
    return float(capL[iy, ix].min())


def oct_(a, b):
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6






HYPO = OCT and args.get('hypo', '1') != '0'



HYPO_LATE = HYPO and args.get('hypo', 'end') == 'end' and ('start' not in args or args.get('hypo') == 'end')
_HYPO_ON = [HYPO and not HYPO_LATE]


def _corners(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; m = min(abs(dx), abs(dy))
    if m < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9: return ()
    sx, sy = (1.0 if dx > 0 else -1.0), (1.0 if dy > 0 else -1.0)
    return ((b[0] - sx * m, b[1] - sy * m), (a[0] + sx * m, a[1] + sy * m))










PAD_CENTER = args.get('pad_center', '1' if NC else '0') == '1'
PAD_CENTER_MAX = float(args.get('pad_center_max', '4.0'))


def _pad_center(li, pt):
    q = _PT(pt)
    for j in _PADT[li].query(q.buffer(0.03)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.distance(q) <= 0.03:
            c = _PADXY[li][int(j)]
            if c is not None and g2.contains(_PT(c)): return (c, g2)
    return None


def _pad_connection(P, li, capL, C):
    r = _pad_center(li, P[-1])
    if r is None: return P
    c, g = r; e = P[-1]; d = math.dist(e, c)
    if d < 1e-6 or d > PAD_CENTER_MAX: return P
    req = C['wmin']
    def vacant(a, b): return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= req / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
    def in_pad(a, b): return g.buffer(0.03).contains(_LS([a, b]))
    def paths(a, b):
        if not oct_net() or oct_(a, b): yield []
        if oct_net():
            for ck in reversed(_corners(a, b)): yield [ck]
    if len(P) >= 3:
        q, a = P[-3], P[-2]; d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        det = -d1[0] * d2[1] + d1[1] * d2[0]
        if abs(det) > 1e-9:
            t = (-(c[0] - q[0]) * d2[1] + (c[1] - q[1]) * d2[0]) / det
            a2 = (q[0] + t * d1[0], q[1] + t * d1[1])
            if t > 1e-6 and math.dist(a2, a) <= 0.25 and vacant(q, a2) and vacant(a2, c): return P[:-2] + [a2, c]
    if len(P) >= 2:
        a = P[-2]
        for zw in paths(a, c):
            pk = [a] + zw + [c]
            if all(vacant(x, y) for x, y in zip(pk[:-1], pk[1:])): return P[:-1] + zw + [c]
    for zw in paths(e, c):
        pk = [e] + zw + [c]
        if all(in_pad(x, y) and vacant(x, y) for x, y in zip(pk[:-1], pk[1:])): return P + zw + [c]
    return P












R27_TRUNK = PAD_CENTER and args.get('r27_trunk', '0') == '1'
BRANCH_EXACT = args.get('branch', '0') == '1' or (R27_TRUNK and args.get('branch_exact', '1') == '1')
OFFSET = PAD_CENTER and args.get('offset', '1') == '1'
JOINT_STAT = dict(branch_exact=0, branch_pushed=0, branch_corner=0, branch_skewed=0, branch_open=0, via_start=0, via_start_beside=0,
                  via_beside_track=0, offset=0, offset_end=0)
def _cut(p, d1, q, d2):
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-9 * max(1e-9, math.hypot(*d1) * math.hypot(*d2)): return None
    t = ((q[0] - p[0]) * d2[1] - (q[1] - p[1]) * d2[0]) / det
    return (p[0] + t * d1[0], p[1] + t * d1[1])


def _on_anchor(P, A, vacant):
    if math.dist(P[0], A) < 1e-7: return [A] + P[1:]
    if not oct_net(): return [A] + P[1:] if vacant(A, P[1]) else None
    if len(P) < 3: return None
    d1 = (P[1][0] - P[0][0], P[1][1] - P[0][1]); d2 = (P[2][0] - P[1][0], P[2][1] - P[1][1])
    a2 = _cut(A, d1, P[1], d2)
    if a2 is None or math.dist(a2, P[1]) > 0.25: return None
    if (a2[0] - A[0]) * d1[0] + (a2[1] - A[1]) * d1[1] <= 1e-9 or (P[2][0] - a2[0]) * d2[0] + (P[2][1] - a2[1]) * d2[1] <= 1e-9: return None
    return [A, a2] + P[2:] if vacant(A, a2) and vacant(a2, P[2]) else None


def _trunk_point(t, i, X):
    t['pts'] = list(t['pts'][:i + 1]) + [[X[0], X[1]]] + list(t['pts'][i + 1:])
    t['w'] = list(t['w'][:i + 1]) + [t['w'][i]] + list(t['w'][i + 1:])


def _branch(P, li, node, capL, C):
    S = P[0]
    if len(P) < 2: return P
    v = _TRUNK_V.get(node); st = [t for t in _TRUNK.get(node, ()) if LI[t['layer']] == li]
    if v is None and not st: return P
    c0 = min(C['wmin'], line_cap_min(capL, S, P[1])); r0 = min(C['wmin'] / 2, pad_room(li, S, P[1], NET_CUR[0]))



    def vacant(a, b): return pad_room(li, a, b, NET_CUR[0]) >= r0 - 1e-9 and (not oct_net() or line_cap_min(capL, a, b) >= c0 - 1e-6)
    if v is not None:
        if math.dist(S, v) < 1e-7: return P
        Q = _on_anchor(P, v, vacant); JOINT_STAT['via_start' if Q else 'via_start_beside'] += 1
        return Q or P
    cand = []; corner = []
    d1 = (P[1][0] - S[0], P[1][1] - S[1])
    for t in st:
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); L_ = (a[0] + u * dx, a[1] + u * dy); d = math.dist(S, L_)
            if d > 0.045: continue
            corner.append((d, 1, len(corner), i, L_, t))
            for j_, e_ in ((i, a), (i + 1, b)):
                if math.dist(S, e_) <= 0.08: corner.append((math.dist(S, e_), 0, len(corner), j_, (e_[0], e_[1]), t))
            if d < 1e-7: cand.append((0.0, len(cand), t, i, S)); continue
            X = _cut(S, d1, a, (dx, dy))
            if X is None: continue
            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
            if ux < -1e-9 or ux > 1 + 1e-9 or math.dist(X, S) > 0.08: continue
            if (P[1][0] - X[0]) * d1[0] + (P[1][1] - X[1]) * d1[1] <= 1e-9: continue
            cand.append((math.dist(X, S), len(cand), t, i, X))
    for d, _, t, i, X in sorted(cand, key=lambda k_: k_[:2]):
        a, b = t['pts'][i], t['pts'][i + 1]
        if math.dist(X, a) < 2e-4: X = (a[0], a[1]); new = False
        elif math.dist(X, b) < 2e-4: X = (b[0], b[1]); new = False
        else: X = (round(X[0], 4), round(X[1], 4)); new = True


        if new: _trunk_point(t, i, X)
        JOINT_STAT['branch_exact' if d <= 1e-7 else 'branch_pushed'] += 1
        return [X] + P[1:]
    for d, lot, _, i, A, t in sorted(corner, key=lambda k_: k_[:3]):
        A = (round(A[0], 4), round(A[1], 4))
        Q = _on_anchor(P, A, vacant); category = 'branch_corner'
        if Q is None and vacant(A, P[1]): Q = [A] + P[1:]; category = 'branch_skewed'
        if Q is None: continue
        if lot and math.dist(A, t['pts'][i]) > 2e-4 and math.dist(A, t['pts'][i + 1]) > 2e-4: _trunk_point(t, i, A)
        JOINT_STAT[category] += 1
        return Q
    if cand or corner: JOINT_STAT['branch_open'] += 1
    return P


def _via_on_trunk(li, node, S):
    if node in _TRUNK_V: return
    best = None
    for t in _TRUNK.get(node, ()):
        if LI[t['layer']] != li: continue
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); d = math.hypot(S[0] - a[0] - u * dx, S[1] - a[1] - u * dy)
            if best is None or d < best[0]: best = (d, t, i)
    if best is None: return
    d, t, i = best
    if d > 1e-7: JOINT_STAT['via_beside_track'] += 1; return
    X = (round(S[0], 4), round(S[1], 4))
    if math.dist(X, t['pts'][i]) > 2e-4 and math.dist(X, t['pts'][i + 1]) > 2e-4: _trunk_point(t, i, X)


def joints_repair():
    n = 0; per = {}; prior = {}
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        V = [(v['x'], v['y']) for v in r['vias'] if (v.get('net') or r['net']) == r['net']]; ch = False
        prior[u] = [(t, list(t['pts']), list(t['w'])) for t in r['tracks']]
        for t in r['tracks']:
            if len(t['pts']) < 2: continue
            li = LI[t['layer']]
            for end in (0, -1):
                E = t['pts'][end]; N = t['pts'][1] if end == 0 else t['pts'][-2]
                if any(math.dist(E, v) < 1e-6 for v in V): continue
                _pe = _PT(E)
                if any(_PADG[li][int(j)][0] == r['net'] and _PADG[li][int(j)][1].covers(_pe) for j in _PADT[li].query(_pe)): continue
                def direction_preserving(X):
                    if not oct_net(r['net']): return True
                    for t3 in r['tracks']:
                        if t3['layer'] != t['layer'] or len(t3['pts']) < 2: continue
                        for e3, n3 in ((0, 1), (-1, -2)):
                            if math.dist(t3['pts'][e3], E) >= 1e-6: continue
                            q = t3['pts'][n3]; dx, dy = E[0] - q[0], E[1] - q[1]
                            vx, vy = X[0] - q[0], X[1] - q[1]
                            if not oct_(q, X) or abs(dx * vy - dy * vx) > 1e-7 * max(1.0, math.hypot(dx, dy)) or dx * vx + dy * vy <= 0:
                                return False
                    return True
                best = None; exact = False
                for t2 in r['tracks']:
                    if t2 is t or t2['layer'] != t['layer']: continue
                    P2 = t2['pts']
                    for i in range(len(P2) - 1):
                        a, b = P2[i], P2[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
                        if l2 < 1e-12: continue
                        uu = max(0.0, min(1.0, ((E[0] - a[0]) * dx + (E[1] - a[1]) * dy) / l2))
                        d = math.hypot(E[0] - a[0] - uu * dx, E[1] - a[1] - uu * dy)
                        if d <= 1e-7: exact = True; break
                        if d > 0.045: continue
                        X = _cut(E, (N[0] - E[0], N[1] - E[1]), a, (dx, dy))
                        if X is not None:
                            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
                            if -1e-9 <= ux <= 1 + 1e-9 and math.dist(X, E) <= 0.08 and (N[0] - X[0]) * (N[0] - E[0]) + (N[1] - X[1]) * (N[1] - E[1]) > 1e-9:
                                k_ = (math.dist(X, E), 0, t2, i, X)
                                if direction_preserving(X) and (best is None or k_[:2] < best[:2]): best = k_
                                continue
                        for e_ in (a, b):
                            if math.dist(E, e_) <= 0.08:
                                k_ = (math.dist(E, e_), 1, t2, i, (e_[0], e_[1]))
                                if direction_preserving(e_) and (best is None or k_[:2] < best[:2]): best = k_
                    if exact: break
                if exact or best is None: continue
                _, _, t2, i, X = best
                a, b = t2['pts'][i], t2['pts'][i + 1]
                if math.dist(X, a) < 2e-4: X = (a[0], a[1])
                elif math.dist(X, b) < 2e-4: X = (b[0], b[1])
                else: X = (round(X[0], 4), round(X[1], 4))
                if not direction_preserving(X): continue
                if math.dist(X, a) >= 2e-4 and math.dist(X, b) >= 2e-4: _trunk_point(t2, i, X)
                for t3 in r['tracks']:
                    if t3['layer'] != t['layer']: continue
                    for e3 in (0, -1):
                        if math.dist(t3['pts'][e3], E) < 1e-6:
                            if e3 == 0: t3['pts'] = [[X[0], X[1]]] + list(t3['pts'][1:])
                            else: t3['pts'] = list(t3['pts'][:-1]) + [[X[0], X[1]]]
                n += 1; ch = True; per[u] = per.get(u, 0) + 1
        if ch: _GEO.pop(u, None)
    if per:
        post = exact_conflicts()
        for u in sorted(per):
            if u not in post: continue
            for t, p_, w_ in prior[u]: t['pts'] = p_; t['w'] = w_
            _GEO.pop(u, None); n -= per[u]; JOINT_STAT['joint_back'] = JOINT_STAT.get('joint_back', 0) + per[u]
    return n


def collinear_together():
    kn = {}
    for u, r in U.items():
        s = kn.setdefault(r['net'], set())
        for t in r['tracks']:
            for q in (t['pts'][0], t['pts'][-1]): s.add((t['layer'], round(q[0], 4), round(q[1], 4)))
        for v in r['vias']:
            if (v.get('net') or r['net']) != r['net']: continue
            for L in LAYERS: s.add((L, round(v['x'], 4), round(v['y'], 4)))
    n = 0
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        s = kn[r['net']]; ch = False
        for t in r['tracks']:
            P, W = t['pts'], t['w']
            if len(P) < 3: continue
            np_, nw = [P[0]], []
            for i in range(1, len(P)):
                if nw and abs(nw[-1] - W[i - 1]) < 1e-6 and len(np_) >= 2 and (t['layer'], round(np_[-1][0], 4), round(np_[-1][1], 4)) not in s:
                    a, b, c = np_[-2], np_[-1], P[i]; dx, dy = c[0] - a[0], c[1] - a[1]; l_ = math.hypot(dx, dy)
                    if l_ > 1e-9 and abs((b[0] - a[0]) * dy - (b[1] - a[1]) * dx) / l_ <= 2e-4 and (b[0] - a[0]) * dx + (b[1] - a[1]) * dy > 0 \
                            and (c[0] - b[0]) * dx + (c[1] - b[1]) * dy > 0:
                        np_[-1] = P[i]; n += 1; ch = True; continue
                np_.append(P[i]); nw.append(W[i - 1])
            if len(np_) != len(P): t['pts'] = np_; t['w'] = nw
        if ch: _GEO.pop(u, None)
    return n


MINI = 0.1


def _offset_way(P, li, capL, C):
    if len(P) < 4: return P
    def cap(a, b): return line_cap_min(capL, a, b)
    def vacant(a, b, req): return cap(a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= C['wmin'] / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
    def middle(Q, i):
        if i < 1 or i + 3 > len(Q) - 1: return None
        u = (Q[i][0] - Q[i - 1][0], Q[i][1] - Q[i - 1][1]); v = (Q[i + 2][0] - Q[i + 1][0], Q[i + 2][1] - Q[i + 1][1])
        lu, lv = math.hypot(*u), math.hypot(*v)
        if lu < 1e-9 or lv < 1e-9 or abs(u[0] * v[1] - u[1] * v[0]) > 1e-6 * lu * lv or u[0] * v[0] + u[1] * v[1] <= 0: return None
        w = (Q[i + 3][0] - Q[i + 2][0], Q[i + 3][1] - Q[i + 2][1])
        X = _cut(Q[i - 1], u, Q[i + 2], w)
        if X is None or math.dist(X, Q[i + 2]) > 0.2: return None
        if (X[0] - Q[i - 1][0]) * u[0] + (X[1] - Q[i - 1][1]) * u[1] <= 1e-9 or (Q[i + 3][0] - X[0]) * w[0] + (Q[i + 3][1] - X[1]) * w[1] <= 1e-9: return None
        req = min([C['wmin']] + [cap(Q[j], Q[j + 1]) for j in range(i - 1, i + 3)])
        return Q[:i] + [X] + Q[i + 3:] if vacant(Q[i - 1], X, req) and vacant(X, Q[i + 3], req) else None
    def end(Q):
        if len(Q) < 4: return None
        q, a, e, c = Q[-4], Q[-3], Q[-2], Q[-1]
        d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        a2 = _cut(q, d1, c, d2)
        if a2 is None or math.dist(a2, a) > 0.2: return None
        if (a2[0] - q[0]) * d1[0] + (a2[1] - q[1]) * d1[1] <= 1e-9 or (c[0] - a2[0]) * d2[0] + (c[1] - a2[1]) * d2[1] <= 1e-9: return None
        req = min(C['wmin'], cap(q, a), cap(a, e), cap(e, c))
        return Q[:-3] + [a2, c] if vacant(q, a2, req) and vacant(a2, c, req) else None
    for _ in range(40):
        new = None
        for i in range(len(P) - 1):
            if not (1e-9 < math.dist(P[i], P[i + 1]) < MINI + 1e-9): continue
            new = middle(P, i)
            if new is None:
                new = middle(P[::-1], len(P) - 2 - i); new = new[::-1] if new else None
            if new is not None: JOINT_STAT['offset'] += 1; break
            if i == len(P) - 2: new = end(P)
            elif i == 0: new = end(P[::-1]); new = new[::-1] if new else None
            if new is not None: JOINT_STAT['offset_end'] += 1; break
        if new is None: break
        P = new
    return P


def _in_own_pad(li, pt):
    q = _PT(pt)
    for j in _PADT[li].query(q):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.contains(q) and g2.boundary.distance(q) >= 0.02: return True
    return False


def _surcharge(li, a, b, ex):
    L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1); t = np.linspace(0, 1, n)
    ix = ((a[0] + t * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + t * (b[1] - a[1]) - Y0) / RES).astype(int)
    f = np.ones(n, dtype=np.float32) if ex is None else ex[li * NN + iy * NX + ix].astype(np.float32)
    if li == 0: f = np.where(under_esp[iy, ix], f * 1.8, f)
    if IMP_REF_MASK:
        _m = IMP_REF_MASK.get((NET_CUR[0], LAYERS[li]))
        if _m is not None: f = np.where(_m[iy, ix], f, f * 8.0)
    return float((f - 1.0).sum()) * L / n


def path_to_geometry(path, capC, C):
    runs, cur = [], [path[0]]
    for n in path[1:]:
        if n // NN != cur[-1] // NN: runs.append(cur); cur = [n]
        else: cur.append(n)
    runs.append(cur)
    out_tr, out_v = [], []
    for k, run in enumerate(runs):
        li = run[0] // NN
        if C.get('zw'): C = dict(C, w=C['zw'][li], wmin=C['zw'][li])
        if C.get('current_w'): C = dict(C, w=C['current_w'][li], wmin=C['current_w'][li])
        pts = [cell_xy((n % NN) % NX, (n % NN) // NX) for n in run]
        if GS_N:
            _nc = NID.get(NET_CUR[0], -3)
            for _q, _nq in enumerate(run):
                _g = GS_PT.get(_nq)
                if _g is not None and _g[2] != _nc and _g[3] != _nc: pts[_q] = (_g[0], _g[1])
        if VP:
            if k > 0:
                _s = _vp_xy(NET_CUR[0], run[0] % NN)
                if _s: pts[0] = _s
            if k < len(runs) - 1:
                _s = _vp_xy(NET_CUR[0], run[-1] % NN)
                if _s: pts[-1] = _s
        capL = capC[li]
        pc = [float(capL.ravel()[n % NN]) for n in run]


        _r41_pre = None
        if R41_PARAM is not None and C.get('extra') is not None:
            _r41_pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]):
                _r41_pre.append(_r41_pre[-1] + _surcharge(li, _a, _b, C['extra']))
        def _r41_shortcut(i, j):
            return _r41_pre is None or _surcharge(li, pts[i], pts[j], C['extra']) <= _r41_pre[j] - _r41_pre[i] + RES
        if BRANCH_EXACT and k == 0 and len(run) == 1 and len(runs) > 1: _via_on_trunk(li, run[0], pts[0])
        if _HYPO_ON[0] and oct_net():
            _ex = C.get('extra'); _pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]): _pre.append(_pre[-1] + _surcharge(li, _a, _b, _ex))
            def _free(a, b, req):
                return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room_min(li, a, b, NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
            def _connect(i, j):
                if j == i + 1: return []
                a, b = pts[i], pts[j]; mn = min(pc[i:j + 1]); req = max(min(C['wmin'], mn), min(C['w'], mn))
                if oct_(a, b): return [] if _free(a, b, req) and _r41_shortcut(i, j) else None
                best = None
                for c in _corners(a, b):
                    if _free(a, c, req) and _free(c, b, req):
                        kk = _surcharge(li, a, c, _ex) + _surcharge(li, c, b, _ex)
                        q = min(line_cap_min(capL, a, c), line_cap_min(capL, c, b))
                        if best is None or kk < best[0] - 1e-9 or (abs(kk - best[0]) <= 1e-9 and q > best[1] + 1e-9): best = (kk, q, c)
                if best is None or best[0] > _pre[j] - _pre[i] + 0.05: return None
                return [best[2]]
            P, i, n_ = [pts[0]], 0, len(pts)
            while i < n_ - 1:
                j, zw = n_ - 1, None
                zw = _connect(i, j)
                if zw is None:
                    lo, hi, zw = i + 1, j, []
                    while hi - lo > 1:
                        mid = (lo + hi) // 2; r = _connect(i, mid)
                        if r is None: hi = mid
                        else: lo, zw = mid, r
                    j = lo
                P.extend(zw); P.append(pts[j]); i = j


            if k == 0 and len(P) >= 3 and math.dist(P[0], P[1]) < 0.1 + 1e-9 and _in_own_pad(li, P[1]): P = P[1:]
            if k == len(runs) - 1 and len(P) >= 3 and math.dist(P[-1], P[-2]) < 0.1 + 1e-9 and _in_own_pad(li, P[-2]): P = P[:-1]
        else:
            simp, i = [0], 0
            while i < len(pts) - 1:
                j_best = i + 1
                for j in range(len(pts) - 1, i + 1, -1):
                    if oct_net() and not oct_(pts[i], pts[j]): continue
                    req = max(min(C['wmin'], min(pc[i:j + 1])), min(C['w'], min(pc[i:j + 1])))
                    if SOURCE_FREE[0]: req = max(req, max(TRACK_MIN, min(C['wmin'], NECK_W)))
                    if line_cap_min(capL, pts[i], pts[j]) >= req - 1e-6 and (j == i + 1 or pad_room_min(li, pts[i], pts[j], NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2) and (j == i + 1 or not IMP_REF_MASK or z_ref_free(C, li, pts[i], pts[j])):
                        if j > i + 1 and not _r41_shortcut(i, j): continue
                        j_best = j; break
                simp.append(j_best); i = j_best
            P = [pts[i] for i in simp]
        if PAD_CENTER and C.get('anchor', True):
            if k == len(runs) - 1: P = _pad_connection(P, li, capL, C)
            if k == 0: P = _pad_connection(P[::-1], li, capL, C)[::-1]
        if BRANCH_EXACT and k == 0 and P[0] == pts[0]: P = _branch(P, li, run[0], capL, C)
        if OFFSET and (_HYPO_ON[0] or not oct_net()): P = _offset_way(P, li, capL, C)
        seg_pts, seg_w = [P[0]], []
        for a, b in zip(P[:-1], P[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); m = max(1, int(math.ceil(L / 0.8)))
            for q in range(m):
                pa = (a[0] + (b[0] - a[0]) * q / m, a[1] + (b[1] - a[1]) * q / m)
                pb = (a[0] + (b[0] - a[0]) * (q + 1) / m, a[1] + (b[1] - a[1]) * (q + 1) / m)
                wv = min(C['w'], line_cap_min(capL, pa, pb), 2 * pad_room_min(li, pa, pb, NET_CUR[0], C['w'] / 2))
                _bo = min(C['wmin'], 0.15) if C.get('prio', 8) < 8 else min(C['wmin'], 0.1)
                if C.get('ground_map') is not None:
                    _bk = float(C['ground_map'][li * NN + int(((pa[1] + pb[1]) / 2 - Y0) / RES) * NX + int(((pa[0] + pb[0]) / 2 - X0) / RES)])
                    if _bk < _bo: _bo = _bk
                wv = max(TRACK_MIN, _bo, math.floor(wv * 40) / 40)
                if C.get('current_w'): wv = current_segment_width(C,min(line_cap_min(capL,pa,pb), 2*pad_room_min(li,pa,pb,NET_CUR[0],C['w']/2)))
                if C.get('zw'): wv = C['zw'][li]
                if _pz_active(NET_CUR[0]) and NET_CUR[0] in PZ_WIDTHS:wv=PZ_WIDTHS[NET_CUR[0]]['w']
                seg_pts.append(pb); seg_w.append(wv)
        mp, mw = [seg_pts[0]], []
        for i2, wv in enumerate(seg_w):
            pb = seg_pts[i2 + 1]
            if mw and abs(mw[-1] - wv) < 1e-6 and len(mp) >= 2:
                a, b = mp[-2], mp[-1]
                if abs((b[0] - a[0]) * (pb[1] - b[1]) - (b[1] - a[1]) * (pb[0] - b[0])) < 1e-6: mp[-1] = pb; continue
            mp.append(pb); mw.append(wv)
        if len(mp) >= 2:
            out_tr.append(dict(layer=LAYERS[li], pts=[[round(x, 4), round(y, 4)] for x, y in mp], w=mw))
        if k < len(runs) - 1:
            vx, vy = pts[-1]; out_v.append(dict(x=round(vx, 4), y=round(vy, 4)))
    return out_tr, out_v


SW_DIST = float(args.get('sw_dist', '0.8' if NC else '0'))
_SWN = [n for n in nets if n.split('/')[-1].startswith('SW_')]
SEPARATE = float(args.get('separate', '0.5'))
if NC:
    AGG_SW = {n for n, k in NC['net_list'].items() if k['category'] == 'sw'}
    AGG_FAST = {n for n, k in NC['net_list'].items() if k['category'] == 'fast' and k['rank'] >= 3}
    VICT_SW = {n for n, k in NC['net_list'].items() if k.get('sensitive_sw')}
    VICT_A = {n for n, k in NC['net_list'].items() if k['category'] == 'analog' and k['rank'] >= 2}
    if DISTURB_FLANK:
        AGG_SW = set(DISTURB_LIST['sw'] + DISTURB_LIST['current_val'])
        AGG_FAST = set(DISTURB_LIST['digital'])
        VICT_SW = VICT_A = set(DISTURB_LIST['victim'])
_COILS_M = None
if DISTURB_FLANK:
    _COILS_M = np.zeros(NL * NN, dtype=bool)
    for c in D.get('courtyards', []):
        if c['ref'] in DISTURB_LIST['coils'] and c.get('side') in LI:
            li = LI[c['side']]; _COILS_M[li * NN:(li + 1) * NN] |= raster_polys(c['polys']).ravel()


def _edges_groups():
    out = {}
    for n in sorted(AGG_FAST):
        f = DISTURB_LIST['net_list'][n]['spacing_factor']
        out.setdefault(f, set()).add(n)
    return sorted(out.items())


def _separate_mm(net):
    return SEPARATE * DISTURB_LIST['net_list'][net]['spacing_factor'] if DISTURB_FLANK and net in AGG_FAST else SEPARATE


def _mask_arr(nets):
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'] in nets:
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'] in nets: m[r['cells']] = True
    return m


def _nearby(m, dist, fac):
    ex = np.ones(NL * NN, dtype=np.float32)
    if NEIGHBOR_LAYER:
        dl = {}
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): dl[li] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        for li in range(NL):
            close = dl[li] < dist if li in dl else None
            for lj, s in neighbor_spacing(li, dist):
                if lj in dl: close = (dl[lj] < s) if close is None else (close | (dl[lj] < s))
            if close is not None: ex[li * NN:(li + 1) * NN] = np.where(close, fac, 1.0)
        return ex
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < dist, fac, 1.0)
    return ex


_NEARBY_C = {}


R41_GAP = float(args.get('r41_gap', '0'))
if not math.isfinite(R41_GAP) or R41_GAP < 0: raise ValueError('r41_gap must be finite and >= 0')
R41_FAST, R41_NON_DC, R41_PARAM = set(), set(), None
if R41_GAP and NC is not None and NLAY >= 4:
    from analysis.r41 import net_list as _r41_nets, parameter as _r41_parameter
    R41_FAST, R41_NON_DC = _r41_nets(NC)
    R41_PARAM = _r41_parameter(NC, dict(args, via='%g/%g' % (VIA_D, VIA_DR)))
    print('R41 via room (option):', R41_PARAM, 'Price', R41_GAP, flush=True)
_R41_C = {}


def r41_extra(net):
    if R41_PARAM is None or net not in R41_NON_DC: return None
    nets = (R41_NON_DC if net in R41_FAST else R41_FAST) - {net}
    if net in PAIR_NETS: nets = nets - {PAIR_NETS[net]}
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets))
    key = (net, sig)
    if key in _R41_C: return _R41_C[key]

    if len(_R41_C) >= 4: _R41_C.pop(next(iter(_R41_C)))
    ex = np.ones(NL * NN, dtype=np.float32)
    geometries = [[] for _ in range(NL)]
    for unit in U.values():
        if unit['net'] not in nets: continue
        for t in unit['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if a != b: geometries[li].append(_LS([a,b]).buffer(w/2))
    from shapely.ops import unary_union as _r41_union
    for li, gg in enumerate(geometries):
        if not gg: continue
        w = z_width(net, li, cls(net)['w']) if net in IMP_TAB else cls(net)['w']


        band = _r41_union([g.buffer(w/2 + R41_PARAM['via_slot']).difference(
            g.buffer(w/2 + R41_PARAM['gap_from'])) for g in gg])

        x0,y0,x1,y1 = band.bounds if not band.is_empty else (0,0,0,0)
        xa,xb = max(0,int((x0-X0)/RES)),min(NX,int((x1-X0)/RES)+2)
        ya,yb = max(0,int((y0-Y0)/RES)),min(NY,int((y1-Y0)/RES)+2)
        if xa >= xb or ya >= yb: continue
        xx,yy = np.meshgrid(GXc[xa:xb], GYc[ya:yb])
        mask = shapely.contains_xy(band,xx,yy)
        ex[li*NN:(li+1)*NN].reshape(NY,NX)[ya:yb,xa:xb][mask] = 1.0 + R41_GAP
    _R41_C[key] = ex
    return ex


COPPER_RESERVE = max(0.0, float(args.get('copper_reserve', '0'))) if NC else 0.0
_COPPER_RESERVE_C = {}


def copper_reserve_extra(net):
    if not COPPER_RESERVE:
        return None
    ex = np.ones(NL * NN, dtype=np.float32)
    for n, k in sorted(NC['net_list'].items()):
        if n == net or k.get('category') not in ('current_val', 'supply', 'sw'):
            continue
        version = _NVER.get(n, 0)
        cached = _COPPER_RESERVE_C.get(n)
        if cached is None or cached[0] != version:
            groups = {}
            for r in U.values():
                if r['net'] != n:
                    continue
                for t in r['tracks']:
                    if t.get('sig') or t.get('kelvin'):
                        continue
                    li = LI[t['layer']]
                    goal = cls(n)['w'] * f_inner(n, li)
                    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                        if goal <= w + 1e-6:
                            continue
                        band = math.ceil(((goal - w) / 2 + CLR) / RES) * RES
                        groups.setdefault((li, band), []).append(seg_cells(li, a, b, w))
            maps = []
            for (li, band), cells in sorted(groups.items()):
                idx = np.unique(np.concatenate(cells)) - li * NN
                if not len(idx):
                    continue
                xx, yy = idx % NX, idx // NX
                pad = math.ceil(band / RES) + 2
                xa, xb = max(0, int(xx.min()) - pad), min(NX, int(xx.max()) + pad + 1)
                ya, yb = max(0, int(yy.min()) - pad), min(NY, int(yy.max()) + pad + 1)
                mask = np.zeros((yb - ya, xb - xa), dtype=bool)
                mask[yy - ya, xx - xa] = True
                d = _edt(~mask) * RES
                cost = (1 + COPPER_RESERVE * np.clip(1 - d / band, 0, 1)).astype(np.float32)
                maps.append((li, xa, xb, ya, yb, cost))
            cached = (version, maps)
            _COPPER_RESERVE_C[n] = cached
        for li, xa, xb, ya, yb, cost in cached[1]:
            target = ex[li * NN:(li + 1) * NN].reshape(NY, NX)[ya:yb, xa:xb]
            np.maximum(target, cost, out=target)
    return ex


def _nearby_c(name, nets, dist, fac, body=False):
    key = (name, frozenset(nets), dist, fac, body) if DISTURB_FLANK else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if DISTURB_FLANK else sum(_NVER.get(n, 0) for n in nets)
    c = _NEARBY_C.get(key)
    if c is None or c[0] != sig:
        m = _mask_arr(nets)
        if body and _COILS_M is not None: m |= _COILS_M
        c = (sig, _nearby(m, dist, fac)); _NEARBY_C[key] = c
    return c[1]

def si_spacing(victim, aggressor):


    from analysis.signal_integrity import spec, count
    budget = count(spec(NC, victim, 'noise_budget_mV'), 1.0, 1e-9) / 1000
    swing = count(spec(NC, aggressor, 'hub_V'), 12.0 if aggressor in AGG_SW else 3.3)
    hs = [a['H'] for a in STRUCTURE if a['H']]
    h = max(hs) if hs else 1.5
    return min(float(args.get('si_spacing_max', '3.0')), h * math.sqrt(max(0.0, swing / budget - 1)))


def si_disturb_extra(net, ex):
    if not SI: return ex
    victim = {n for n, k in NC['net_list'].items() if k.get('category') in ('analog', 'kelvin') or k.get('kelvin')}
    agg = AGG_FAST | AGG_SW
    if net in victim:
        group_list = {}
        for ag in sorted(agg):
            d = round(si_spacing(net, ag), 3)
            group_list.setdefault(d, set()).add(ag)
    elif net in agg:
        group_list = {}
        for v in sorted(victim):
            d = round(si_spacing(v, net), 3)
            group_list.setdefault(d, set()).add(v)
    else: return ex
    for d, ns in sorted(group_list.items()):
        if d <= 0: continue
        e = _nearby_c(('si', tuple(sorted(ns)), d), ns, d, 4.0)
        ex = e if ex is None else np.maximum(ex, e)
    return ex


def disturb_extra(net):
    if NC is None: return sw_extra(net)
    ex = None
    if net in VICT_SW and SW_DIST > 0: ex = _nearby_c('sw', AGG_SW - {net} if DISTURB_FLANK else AGG_SW, SW_DIST, 6.0, body=DISTURB_FLANK)
    if net in VICT_A and SEPARATE > 0:
        if DISTURB_FLANK:
            e2 = np.ones(NL * NN, dtype=np.float32)
            for f, ns in _edges_groups(): e2 = np.maximum(e2, _nearby_c(('flank', f), ns - {net}, SEPARATE * f, 3.0))
        else: e2 = _nearby_c('fast', AGG_FAST, SEPARATE, 3.0)
        ex = e2 if ex is None else ex * e2
    if net in AGG_FAST and SEPARATE > 0:
        e2 = _nearby_c(('analog', _separate_mm(net)), VICT_A - {net} if DISTURB_FLANK else VICT_A, _separate_mm(net), 3.0); ex = e2 if ex is None else ex * e2
    if PAIR_NETS and PAIR_DISTURB > 1 and net not in PAIR_NETS:
        e2 = _nearby_c('pair', set(PAIR_NETS), pair_free_mm(net), PAIR_DISTURB); ex = e2 if ex is None else ex * e2
    if COPPER_RESERVE:
        e2 = copper_reserve_extra(net)
        ex = e2 if ex is None else ex * e2
    ex = si_disturb_extra(net, ex)
    if ZONE_PLAN:
        zex = _zp_maps(net)[1]
        ex = zex if ex is None else ex * zex
    if R41_PARAM is not None:
        e2 = r41_extra(net)
        if e2 is not None: ex = e2 if ex is None else ex * e2
    if ex is not None and DISTURB_FACTOR[0] != 1.0:
        ex = np.where(ex > 1.0, ex * np.float32(DISTURB_FACTOR[0]), ex).astype(ex.dtype)
    return ex



def pair_unit(u):
    return PAIR_SEARCH_SPACE and pair_fast(u)


def _pair_tolerances(k):
    if not PAIR_SEARCH_SPACE: return k
    nets=k['net_list'];seen=set()
    explicit=(k.get('specs') or {}).get('length_groups') or {}
    for n,x in nets.items():
        p=x.get('pair')
        if not p or p not in nets or n in seen or not pair_fast(n):continue
        seen.update((n,p));name='pair:'+'|'.join(sorted((n,p)))
        ts=[nets[q].get('pair_tolerance_mm',None if nets[q].get('length_group') else nets[q].get('tolerance_mm')) for q in (n,p)]
        ts=[float(t) for t in ts if t is not None]
        tol=explicit.get(name,{}).get('tolerance_mm')
        tol=float(tol) if tol is not None else min(ts) if ts else .15
        if not math.isfinite(tol) or tol<=0:raise ValueError('Pair skew tolerance must be finite and positive')
        for q in (n,p):nets[q]['pair_tolerance_mm']=tol
    return k


def _pair_fixed_verify(u):
    fixed = [n for n in (u, PAIR_NETS[u]) if 'FIX#' + n in FIXED]
    if fixed:
        raise ValueError('pair_search_space=1: fixed pair conductors %s; route both partners freely or supply the pair completely with fix=' % ', '.join(sorted(fixed)))


def pair_nets(series):
    out = set(series)
    out.update(PAIR_NETS[u] for u in series if pair_unit(u))
    return out


def pair_series(series):
    if not PAIR_SEARCH_SPACE: return list(series)
    out = []; seen = set()
    for u in series:
        if pair_unit(u): _pair_fixed_verify(u)
        key = min(u, PAIR_NETS[u]) if pair_unit(u) else u
        if key not in seen: seen.add(key); out.append(key)
    return sorted(out, key=lambda u: (0 if pair_unit(u) else 1, out.index(u)))


def _pair_ground(a, b, li):
    wa, wb = z_width(a, li, cls(a)['w']), z_width(b, li, cls(b)['w'])
    gap = max(z_gap(a, li), z_gap(b, li))
    if not all(math.isfinite(v) and v > 0 for v in (wa, wb, gap)):
        raise ValueError('Pair geometry must be finite and positive')
    return wa, wb, gap, (wa + wb) / 2 + gap, wa + wb + gap


def _pair_pad_groups(a, b):
    from scipy.optimize import linear_sum_assignment
    aa = [D['pads'][k] for k in by_net.get(a, [])]
    bb = [D['pads'][k] for k in by_net.get(b, [])]
    groups = []
    for ref in sorted({p['ref'] for p in aa + bb}):
        ap = sorted([p for p in aa if p['ref'] == ref], key=lambda p: str(p['num']))
        bp = sorted([p for p in bb if p['ref'] == ref], key=lambda p: str(p['num']))
        if len(ap) != len(bp): return None
        if not ap: continue
        costs = np.array([[math.dist((p['x'],p['y']),(q['x'],q['y'])) for q in bp] for p in ap])
        ii,jj = linear_sum_assignment(costs)
        for i,j in zip(ii,jj):
            p,q = ap[i],bp[j]
            layers = sorted(set(p['layers']) & set(q['layers']) & set(LAYERS), key=LI.get)
            if not layers: return None
            groups.append((p,q,layers))
    return groups if len(groups) >= 2 else None


def _pair_zp_ok(net, li, geom):
    if not ZONE_PLAN: return True
    spec = ZONE_PLAN['net_list'].get(net, {})
    if spec.get('local_without_layer_change') and LAYERS[li] != spec.get('local_layer'): return False
    return all(geom.distance(g) >= (_ka_clr(net,n,li,'Track','Zone')+.005 if globals().get('KA') is not None else CLR+.005) - 1e-9 for (n, L), g in _ZP_GEO.items() if n != net and L == LAYERS[li])


def _pair_zp_via_ok(net, x, y):
    if not ZONE_PLAN: return True
    ix, iy = math.floor((x - X0) / RES), math.floor((y - Y0) / RES)
    if not (0 <= ix < NX and 0 <= iy < NY): return False
    return bool(_zp_viah(net, np.ones((1, 1), dtype=bool), slice(iy, iy + 1), slice(ix, ix + 1))[0, 0])


def _pair_hard(a, b):
    maps = []; soft = []
    cnt = use.copy()
    for r in U.values():
        if r['net'] in (a,b): cnt[r['cells']] -= 1
    for li in range(NL):
        heights = [IMP_TAB.get(n,{}).get(LAYERS[li],{}).get('profile_data',{}).get('h') for n in (a,b)]
        heights += [STRUCTURE[li]['h'] if STRUCTURE[li]['ref_net'] in GNDS else None]
        outside = max(KA.maximum if globals().get('KA') is not None else CLR, 3*max(h for h in heights if h) if any(heights) else PAIR_FREE)
        mask = np.zeros((NY,NX), dtype=bool)
        for p in D['pads']:
            if p['net'] not in (a,b) and LAYERS[li] in p['cells']:
                mask.ravel()[p['cells'][LAYERS[li]]] = True
        for u,r in U.items():
            if u.startswith(('FIX#','TACK#')) and r['net'] not in (a,b):
                cells = r['cells']; cells = cells[cells//NN == li] % NN
                mask.ravel()[cells] = True
        dist = _edt(~mask)*RES if mask.any() else np.full((NY,NX), 1e6)
        h = np.minimum(2*(dist-outside-QM), 2*(edge_d-EDGE))

        for net in (a,b):
            ref = IMP_REF_MASK.get((net,LAYERS[li]))
            if ref is not None:
                rd = _edt(ref)*RES
                h = np.minimum(h,2*(rd-QM))

        if ZONE_PLAN: h = _zp_hard(b, _zp_hard(a, h, li), li)
        maps.append(h.astype(np.float32))
        occupied = cnt[li*NN:(li+1)*NN].reshape(NY,NX)>0
        d = _edt(~occupied)*RES if occupied.any() else np.full((NY,NX),1e6)
        soft.append((2*(d-outside-QM)).astype(np.float32))
    if HARD_FOREIGN[0]: maps = [np.minimum(h,s) for h,s in zip(maps,soft)]
    return maps, soft


def _pair_line_ok(net, li, pts, w, other=None):
    if globals().get('KA') is not None:
        if any(_ka_room(net,li,a,b,True)<w/2 for a,b in zip(pts[:-1],pts[1:])):return False
        if other is not None and _LS(pts).buffer(w/2).distance(other)<_ka_margin(net,PAIR_NETS.get(net,net),li):return False
    line = _LS(pts)
    if not line.is_simple: return False
    copper = line.buffer(w/2)
    board = border_polygon(D).buffer(-float(args.get('edge','0.3')))
    if not board.covers(copper): return False
    for nn,g in _PADG[li]:
        if nn != net and copper.distance(g) < (_ka_geo_limit(net,nn,li,g2=g,pad=True) if globals().get('KA') is not None else DRC_PADC)-1e-6: return False
    for u,r in U.items():
        if r['net'] == net: continue
        if not u.startswith(('FIX#','TACK#')) and not HARD_FOREIGN[0]: continue
        for t in r['tracks']:
            if LI[t['layer']] != li: continue
            for x,y,ww in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                if copper.distance(_LS([x,y]).buffer(ww/2)) < (_ka_margin(net,r['net'],li) if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
        for v in r['vias']:
            if copper.distance(Point(v['x'],v['y']).buffer(VIA_R)) < (_ka_margin(net,v.get('net') or r['net'],li,type2='Via') if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
    if net in IMP_TAB and any(not z_ref_free(cls(net),li,x,y,net) for x,y in zip(pts[:-1],pts[1:])): return False
    if globals().get('KA') is None and other is not None and copper.distance(other) < DRC_CLR-1e-6: return False
    if not _pair_zp_ok(net, li, copper): return False
    return True


def _pair_pad_escape(net,pad,end,li,w,direction,other=None):
    if time_tight():return None
    sx,sy=pad['x'],pad['y']
    xa=max(1,int((min(sx,end[0])-.8-X0)/RES));xb=min(NX-2,int((max(sx,end[0])+.8-X0)/RES))
    ya=max(1,int((min(sy,end[1])-.8-Y0)/RES));yb=min(NY-2,int((max(sy,end[1])+.8-Y0)/RES))
    nx,ny=xb-xa+1,yb-ya+1
    if nx<2 or ny<2:return None
    cap=hard_maps(net)[0][li][ya:yb+1,xa:xb+1].copy()
    yy,xx=np.indices((ny,nx));px=X0+(xx+xa+.5)*RES;py=Y0+(yy+ya+.5)*RES
    foreign=shapely.union_all([g for nn,g in _PADG[li] if nn!=net])
    if not foreign.is_empty:
        exact=shapely.distance(shapely.points(px,py),foreign)
        cap=np.minimum(cap,2*(exact-DRC_PADC-QM))
    if other is not None and not other.is_empty:
        distance=shapely.distance(shapely.points(px,py),other)
        cap=np.minimum(cap,2*(distance-DRC_CLR-.005))
    ix,iy=int((sx-X0)/RES)-xa,int((sy-Y0)/RES)-ya
    tx,ty=int((end[0]-X0)/RES)-xa,int((end[1]-Y0)/RES)-ya
    if not (0<=ix<nx and 0<=iy<ny and 0<=tx<nx and 0<=ty<ny):return None
    dirs=np.array(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)),np.int64)
    goals=np.zeros(nx*ny,np.int64)
    for j,(dx,dy) in enumerate(dirs):
        if (dx*direction[0]+dy*direction[1])/math.hypot(dx,dy)>=math.sqrt(.5)-1e-7:goals[ty*nx+tx]|=1<<j
    rest=(RES*np.hypot(xx-tx,yy-ty)).ravel()
    path,count=_PAIR_CORE(np.full(8,iy*nx+ix,np.int64),np.arange(8,dtype=np.int64),np.zeros(8),goals,
        cap.reshape(1,-1),np.ones((1,nx*ny)),np.zeros((8,nx*ny),np.bool_),np.zeros(nx*ny,np.bool_),np.array([w]),dirs,
        nx,ny,RES,0,0,nx-1,ny-1,0,np.ones(8,np.int64),12.,15000,rest,np.ones((1,8,nx*ny),np.bool_))
    if not len(path):return None
    raster=[(X0+(xa+n%nx+.5)*RES,Y0+(ya+n//nx+.5)*RES) for n in path]
    own=shapely.union_all([Polygon(poly) for poly in pad.get('polys',{}).get(LAYERS[li],[])])
    while raster and not own.is_empty and own.covers(Point(*raster[0])):raster.pop(0)
    pts=[(sx,sy)]+raster+[end]
    pts=_oct_move(pts)

    out=[pts[0]]
    for j in range(1,len(pts)-1):
        a,b,c=pts[j-1:j+2]
        if abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))>1e-8:out.append(b)
    out.append(pts[-1]);out=[list(p) for j,p in enumerate(out) if j==0 or math.dist(p,out[j-1])>1e-8]
    if len(out)<2:return None
    if not _pair_line_ok(net,li,out,w,other):return None
    return out


def _pair_connection(p, q, li, center, direction, side, a, b, copper=None):
    wa,wb,gap,d,total = _pair_ground(a,b,li)
    norm = (-direction[1],direction[0])
    foreign=[None,None] if copper is None else [copper[1][li],copper[0][li]]
    ends = [(center[0]+side*(wb+gap)/2*norm[0],center[1]+side*(wb+gap)/2*norm[1]),
            (center[0]-side*(wa+gap)/2*norm[0],center[1]-side*(wa+gap)/2*norm[1])]
    def paths(pad,end):
        start = (pad['x'],pad['y'])
        choices = [[start,end]] if oct_(start,end) else [[start,k,end] for k in _corners(start,end)]

        choices=[pp for pp in choices if math.dist(pp[-2],pp[-1])<1e-8 or (pp[-1][0]-pp[-2][0])*direction[0]+(pp[-1][1]-pp[-2][1])*direction[1]>=-1e-8]
        return [[list(x) for i,x in enumerate(pp) if i == 0 or math.dist(x,pp[i-1])>1e-8] for pp in choices]
    for pa in paths(p,ends[0]):
        if len(pa)<2: pa=[pa[0],pa[0]]
        if not _pair_line_ok(a,li,pa,wa,foreign[0]): continue
        for pb in paths(q,ends[1]):
            if len(pb)<2: pb=[pb[0],pb[0]]
            if _pair_line_ok(b,li,pb,wb,shapely.union_all([_LS(pa).buffer(wa/2)]+([foreign[1]] if foreign[1] is not None else []))):
                return [dict(layer=LAYERS[li],pts=pa,w=[wa]*(len(pa)-1)),
                        dict(layer=LAYERS[li],pts=pb,w=[wb]*(len(pb)-1))]


    for first in (0,1):
        pads=(p,q);nets=(a,b);widths=(wa,wb);second=1-first
        paths=[None,None]
        paths[first]=_pair_pad_escape(nets[first],pads[first],ends[first],li,widths[first],direction,foreign[first])
        if paths[first] is None:continue
        paths[second]=_pair_pad_escape(nets[second],pads[second],ends[second],li,widths[second],direction,shapely.union_all([_LS(paths[first]).buffer(widths[first]/2)]+([foreign[second]] if foreign[second] is not None else [])))
        if paths[second] is not None:
            return [dict(layer=LAYERS[li],pts=pp,w=[widths[k]]*(len(pp)-1)) for k,pp in enumerate(paths)]
    return None


def _pair_ports(group, a, b, hard, side, copper=None):
    p,q,layers = group
    middle = ((p['x']+q['x'])/2,(p['y']+q['y'])/2)
    ports = {}
    for layer in layers:
        li = LI[layer]; _,_,_,_,width = _pair_ground(a,b,li)
        for dx,dy,_ in OFFS:
            length = math.hypot(dx,dy); direction = (dx/length,dy/length)

            for step in (0.,.2,.4,.6,.8,1.0,1.4,2.0,2.6,3.2,4.0):
                c = (middle[0]+step*direction[0],middle[1]+step*direction[1])
                ix,iy = int((c[0]-X0)/RES),int((c[1]-Y0)/RES)
                if not (1<=ix<NX-1 and 1<=iy<NY-1) or hard[li][iy,ix]<width: continue
                center = cell_xy(ix,iy)
                left = _pair_connection(p,q,li,center,direction,side,a,b,copper)
                if left is not None:
                    node = li*NN+iy*NX+ix
                    cost = sum(math.dist(x,y) for t in left for x,y in zip(t['pts'][:-1],t['pts'][1:]))
                    ports.setdefault(node,[]).append((cost,left,direction))
    if not ports: return ports
    shortest = min(v[0] for vv in ports.values() for v in vv)
    return {n:[v for v in vv if v[0]<=shortest+.3] for n,vv in ports.items() if any(v[0]<=shortest+.3 for v in vv)}



def _pair_states(a,b,side,tracks,vias):
    yy,xx=np.indices((NY,NX));px=(X0+(xx+.5)*RES).ravel();py=(Y0+(yy+.5)*RES).ravel()
    copper=[[None]*NL for _ in range(2)];allowed=np.ones((NL,8,NN),np.bool_)
    for k,net in enumerate((a,b)):
        for li in range(NL):
            gg=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            gg += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg += [g for nn,g in _PADG[li] if nn==net]
            copper[k][li]=shapely.union_all(gg)
    for li in range(NL):
        wa,wb,gap,_,_=_pair_ground(a,b,li)
        for k,sign,offset,w in ((0,side,(wb+gap)/2,wa),(1,-side,(wa+gap)/2,wb)):
            other=copper[1-k][li]
            if other.is_empty:continue


            near=np.flatnonzero(shapely.contains_xy(other.buffer(offset+w/2+DRC_CLR+.01),px,py))
            for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
                norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy))
                positions=shapely.points(px[near]+sign*offset*norm[0],py[near]+sign*offset*norm[1])
                allowed[li,j,near] &= shapely.distance(positions,other)>=w/2+DRC_CLR-1e-7
    return allowed,copper


def _pair_tree_free(tracks,vias):
    for li in range(NL):
        gg=[]
        for k in range(2):
            shapes=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            shapes += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg.append(shapely.union_all(shapes))
        if gg[0].distance(gg[1])<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:return False
        for k in range(2):
            for v in vias[k]:
                if Point(v['x'], v['y']).distance(gg[1-k]) < VIA_DR/2 + HOLE_CLR - 1e-6:
                    return False
    allvias = [(k, v) for k in range(2) for v in vias[k]]
    for i, (k, v) in enumerate(allvias):
        for qk, q in allvias[i+1:]:
            dist=math.dist((v['x'],v['y']),(q['x'],q['y']))
            if k == qk and dist <= 1e-8: continue
            if dist < VIA_DR+H2H+.012: return False
    return True


def _pair_via_mask(a,b,hard,side=1,copper=None):
    distance=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012,max(_pair_ground(a,b,li)[3] for li in range(NL)))
    yy,xx=np.indices((NY,NX));out=np.zeros((8,NN),np.bool_)

    zp=[_zp_viah(n,np.ones((NY,NX),np.bool_)) for n in (a,b)] if ZONE_PLAN else None
    for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
        norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy));ok=np.ones((NY,NX),np.bool_)
        for sign in (-1,1):
            ix=np.rint(xx+sign*distance/2/RES*norm[0]).astype(int);iy=np.rint(yy+sign*distance/2/RES*norm[1]).astype(int)
            inside=(ix>=0)&(ix<NX)&(iy>=0)&(iy<NY);ix=np.clip(ix,0,NX-1);iy=np.clip(iy,0,NY-1)
            ok &= inside & (VSP[iy,ix]>=H2H+VIA_DR/2+.01)
            if VIA_BAN is not None:ok &= ~VIA_BAN[iy,ix]
            for h in hard:ok &= h[iy,ix]>=2*VIA_KEEP_R
            k=0 if sign==side else 1
            if zp is not None:ok &= zp[k][iy,ix]
            if copper is not None:
                px=X0+(xx+.5)*RES;py=Y0+(yy+.5)*RES
                for li in range(NL):
                    other=copper[1-k][li]
                    if other.is_empty:continue
                    near=shapely.contains_xy(other.buffer(distance/2+VIA_R+max(DRC_CLR,DRC_PADC)+.01),px,py)&ok
                    rows,cols=np.nonzero(near)
                    points=shapely.points(px[rows,cols]+sign*distance/2*norm[0],py[rows,cols]+sign*distance/2*norm[1])
                    ok[rows,cols] &= shapely.distance(points,other)>=VIA_R+max(DRC_CLR,DRC_PADC)
        out[j]=ok.ravel()
    return out


def _pair_remaining_cost(tgt,hard,widths,mult,viah,nx,ny,res,xmin,ymin,xmax,ymax,via_cost):
    nl=hard.shape[0];nn=nx*ny
    out=np.full(nl*nn,np.inf);queue=[(0.,np.int64(0))];queue.pop()
    for n in range(len(tgt)):
        if tgt[n]:out[n]=0.;heapq.heappush(queue,(0.,np.int64(n)))
    dxs=(1,1,0,-1,-1,-1,0,1);dys=(0,1,1,1,0,-1,-1,-1)
    while queue:
        cost,raw=heapq.heappop(queue);n=int(raw)
        if cost>out[n]+1e-10:continue
        li=n//nn;idx=n%nn;iy=idx//nx;ix=idx%nx
        for j in range(8):
            x=ix+dxs[j];y=iy+dys[j]
            if x<xmin or x>xmax or y<ymin or y>ymax:continue
            ii=y*nx+x;m=li*nn+ii
            if hard[li,ii]<widths[li]:continue
            val=cost+res*math.hypot(dxs[j],dys[j])*mult[li,idx]
            if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))


        if np.any(viah[:,idx]):
            for lj in range(nl):
                if lj==li or hard[lj,idx]<widths[lj]:continue
                m=lj*nn+idx;val=cost+via_cost
                if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))
    return out


def _pair_search_core(src_nodes, src_dirs, src_cost, tgt_dirs, hard, mult, viah, viabad, widths,
                      offsets, nx, ny, res, xmin, ymin, xmax, ymax, leadcap, via_steps, via_cost, maxexpand, rest, allowed, require_via=False):
    layers=hard.shape[0];nn=nx*ny;wx=xmax-xmin+1;wy=ymax-ymin+1;wn=wx*wy
    base_stride=8*(leadcap+1);stride=base_stride*(2 if require_via else 1)


    g={np.int64(-1):np.float64(np.inf)}
    parent={np.int64(-1):np.int64(-2)};vcenter={np.int64(-1):np.int64(-1)}
    closed={np.int64(-1):np.int64(0)}
    queue=[(0.0,np.int64(0))];queue.pop()
    if not np.any(tgt_dirs):return np.empty(0,np.int64),0
    for i in range(len(src_nodes)):
        n=src_nodes[i];li=n//nn;iy=n%nn//nx;ix=n%nn%nx
        if ix<xmin or ix>xmax or iy<ymin or iy>ymax:continue
        if not allowed[li,src_dirs[i],n%nn]:continue
        local=li*wn+(iy-ymin)*wx+ix-xmin
        state=local*stride+src_dirs[i]*(leadcap+1)
        if state not in g or src_cost[i]<g[state]:
            g[state]=src_cost[i];parent[state]=-1
            heapq.heappush(queue,(src_cost[i]+rest[n],state))
    expanded=0;goal=-1
    while queue:
        _,rawstate=heapq.heappop(queue);state=int(rawstate)
        if state in closed:continue
        closed[state]=1;expanded+=1
        if expanded>maxexpand:break
        local=state//stride;phase=(state%stride)//base_stride;direction=(state%base_stride)//(leadcap+1);lead=state%(leadcap+1)
        li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin;n=li*nn+iy*nx+ix
        if (not require_via or phase==1) and lead>=leadcap and tgt_dirs[n] & (1<<direction):goal=state;break
        for turn in (-1,0,1):
            if turn and lead<leadcap:continue
            nd=(direction+turn)%8;dx=int(offsets[nd,0]);dy=int(offsets[nd,1]);jx,jy=ix+dx,iy+dy
            if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
            nnidx=jy*nx+jx
            if hard[li,nnidx]<widths[li] or not allowed[li,nd,n%nn] or not allowed[li,nd,nnidx]:continue
            nl=min(leadcap,lead+1) if turn==0 else 0
            dest=(li*wn+(jy-ymin)*wx+jx-xmin)*stride+phase*base_stride+nd*(leadcap+1)+nl
            cost=g[state]+res*math.hypot(dx,dy)*mult[li,nnidx]+(.025 if turn else 0.)
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=-1
            heapq.heappush(queue,(cost+rest[li*nn+nnidx],dest))
        if lead<leadcap:continue
        dx=int(offsets[direction,0]);dy=int(offsets[direction,1]);vs=int(via_steps[direction])
        cx,cy=ix+vs*dx,iy+vs*dy;jx,jy=ix+2*vs*dx,iy+2*vs*dy
        if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
        ci=cy*nx+cx;end=jy*nx+jx
        if not viah[direction,ci]:continue
        for lj in range(layers):
            if lj==li:continue
            free=True;cost=g[state]+via_cost*(1.+3.*viabad[ci])
            for step in range(1,2*vs+1):
                ii=(iy+step*dy)*nx+ix+step*dx;ll=li if step<=vs else lj
                if hard[ll,ii]<widths[ll] or not allowed[ll,direction,ii]:free=False;break
                cost+=res*math.hypot(dx,dy)*mult[ll,ii]
            if not free:continue
            dest=(lj*wn+(jy-ymin)*wx+jx-xmin)*stride+(base_stride if require_via else 0)+direction*(leadcap+1)+leadcap
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=ci
            heapq.heappush(queue,(cost+rest[lj*nn+end],dest))
    if goal<0:return np.empty(0,np.int64),expanded
    reverse=[];state=goal
    while state>=0:
        local=state//stride;li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin
        reverse.append(li*nn+iy*nx+ix)
        prev=parent[state]
        if state in vcenter and vcenter[state]>=0 and prev>=0:
            pli=prev//stride//wn
            reverse.append(li*nn+vcenter[state]);reverse.append(pli*nn+vcenter[state])
        state=prev
    return np.asarray(reverse[::-1],dtype=np.int64),expanded


_PAIR_REST = _pair_remaining_cost
_PAIR_CORE = _pair_search_core
if NB is not None:
    from numba import njit as _pair_njit
    _PAIR_REST = _pair_njit(cache=True)(_pair_remaining_cost)
    _PAIR_CORE = _pair_njit(cache=True)(_pair_search_core)


def _pair_search(src,tgt,hard,soft,viaH,viaBad,C):
    directions=np.array([(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)],dtype=np.int64)
    def heading(d):return min(range(8),key=lambda i: abs(d[0]-directions[i,0]/math.hypot(*directions[i]))+abs(d[1]-directions[i,1]/math.hypot(*directions[i])))
    entries=[(n,v) for n in sorted(src) for v in src[n]]
    sn=np.array([n for n,v in entries],dtype=np.int64);sd=np.array([heading(v[2]) for n,v in entries],dtype=np.int64)
    sc=np.array([v[0]/2 for n,v in entries],dtype=np.float64)
    td=np.zeros(NL*NN,dtype=np.int64)
    for n,vv in tgt.items():
        for v in vv: td[n] |= 1<<((heading(v[2])+4)%8)
    h=np.asarray([v.ravel() for v in hard]);mult=[]
    for li in range(NL):
        c=np.minimum(hard[li],soft[li]);width=C['zw'][li]
        m=(1.+PF[0]*(c<width))*(1.+hist[li*NN:(li+1)*NN].reshape(NY,NX))*C['lf'][li]
        if C.get('pair_tree') is not None: m=np.where(C['pair_tree'][li*NN:(li+1)*NN].reshape(NY,NX),m*.2,m)
        mult.append(m.ravel())
    mult=np.asarray(mult,dtype=np.float64)
    step=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012,max(_pair_ground(NET_CUR[0],PAIR_NETS[NET_CUR[0]],li)[3] for li in range(NL)),.6)
    vs=np.array([math.ceil((step+RES)/RES/math.hypot(*v)) for v in directions],dtype=np.int64)
    lead=max(2,math.ceil(max(C['zw'])*math.tan(math.pi/8)/RES)+1)
    xs=[n%NN%NX for n in list(src)+list(tgt)];ys=[n%NN//NX for n in list(src)+list(tgt)]
    for margin in (3.,8.,20.):
        if time_tight():return None
        pad=math.ceil(margin/RES)
        xa,ya,xb,yb=max(1,min(xs)-pad),max(1,min(ys)-pad),min(NX-2,max(xs)+pad),min(NY-2,max(ys)+pad)
        states=NL*(xb-xa+1)*(yb-ya+1)*8*(lead+1)*(2 if C.get('pair_via_forced',False) else 1)
        state_limit=int(args.get('pair_states_max','0'))
        if state_limit>0 and states>state_limit:continue
        rest=_PAIR_REST(td,h,np.array(C['zw']),mult,viaH,NX,NY,RES,xa,ya,xb,yb,C['via'])
        if time_tight():return None
        if not any(math.isfinite(rest[n]) for n in sn):continue
        path,count=_PAIR_CORE(sn,sd,sc,td,h,mult,viaH,viaBad.ravel(),np.array(C['zw']),directions,NX,NY,RES,
            xa,ya,xb,yb,lead,vs,C['via'],int(args.get('pair_expansions','1000000')),rest,C['pair_permissible'],C.get('pair_via_forced',False))
        if len(path):return [int(n) for n in path]
    return None


def _pair_center(path, hard, widths):
    runs = []; cur = [path[0]]
    for n in path[1:]:
        if n//NN != cur[-1]//NN: runs.append(cur); cur=[n]
        else: cur.append(n)
    runs.append(cur); result=[]
    for run in runs:
        li=run[0]//NN
        pts=[cell_xy(n%NN%NX,n%NN//NX) for n in run]


        out=[pts[0]]
        for j in range(1,len(pts)-1):
            x,y,z=pts[j-1:j+2]
            if abs((y[0]-x[0])*(z[1]-y[1])-(y[1]-x[1])*(z[0]-y[0]))>1e-8 or (y[0]-x[0])*(z[0]-y[0])+(y[1]-x[1])*(z[1]-y[1])<=0:
                out.append(y)
        out.append(pts[-1])
        result.append((li,out))
    return result


def _pair_via_pattern(a,b,li,lj,center,direction,side):
    wa,wb,gap0,d0,_=_pair_ground(a,b,li); w1,w2,gap1,d1,_=_pair_ground(a,b,lj)
    distance=max(d0,d1,VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012)
    normal=(-direction[1],direction[0]); reach=max(distance, .6)
    pair=[]
    for net,sign,w0,wnew,offset0,offset1 in ((a,side,wa,w1,(wb+gap0)/2,(w2+gap1)/2),(b,-side,wb,w2,(wa+gap0)/2,(w1+gap1)/2)):
        v=(center[0]+sign*distance/2*normal[0],center[1]+sign*distance/2*normal[1])
        if not _pair_vias_allowed([dict(x=v[0],y=v[1])]): return None
        if not _pair_zp_via_ok(net,*v) or not all(_pair_zp_ok(net,ll,Point(*v).buffer(VIA_R)) for ll in range(NL)): return None
        board=border_polygon(D).buffer(-float(args.get('edge','0.3')))
        if not board.covers(Point(*v).buffer(VIA_R)): return None
        for ll in range(NL):
            for nn,g in _PADG[ll]:
                if nn!=net and Point(*v).distance(g)<max(VIA_R+DRC_PADC,VIA_DR/2+HOLE_CLR)+.006: return None
        start=(center[0]-reach*direction[0]+sign*offset0*normal[0],center[1]-reach*direction[1]+sign*offset0*normal[1])
        end=(center[0]+reach*direction[0]+sign*offset1*normal[0],center[1]+reach*direction[1]+sign*offset1*normal[1])
        pp=_oct_move([start,v]); qq=_oct_move([v,end])
        if not _pair_line_ok(net,li,pp,w0) or not _pair_line_ok(net,lj,qq,wnew): return None
        pair.append(dict(via=dict(x=v[0],y=v[1]),before=pp,post=qq,reach=reach))
    for ll,key in ((li,'before'),(lj,'post')):
        w0,w1=_pair_ground(a,b,ll)[:2]
        g0=_LS(pair[0][key]).buffer(w0/2);g1=_LS(pair[1][key]).buffer(w1/2)
        if g0.distance(g1)<DRC_CLR-1e-6: return None
        for i,g in enumerate((g0,g1)):
            v=pair[1-i]['via']
            if g.distance(Point(v['x'],v['y']).buffer(VIA_R))<DRC_CLR-1e-6: return None
    return pair


_PAIR_SPLIT_REASON = [None]


def _pair_split(runs, a, b, side, start, end, copper=None):
    out=[[],[]]; vias=[[],[]]
    offsets=[]
    _PAIR_SPLIT_REASON[0]='Channel-Split'
    for li,pts in runs:
        if len(pts)<2: _PAIR_SPLIT_REASON[0]='Layer segment too short'; return None
        wa,wb,gap,d,_=_pair_ground(a,b,li)
        lane=[]
        for sign,offset in ((side,(wb+gap)/2),(-side,(wa+gap)/2)):
            line=_LS(pts).offset_curve(sign*offset,join_style=2,mitre_limit=3)



            if line.geom_type=='MultiLineString': line=shapely.line_merge(line,directed=True)
            if line.geom_type!='LineString' or line.is_empty or not line.is_simple:
                _PAIR_SPLIT_REASON[0]='Offset not permitted: '+str((line.geom_type,li,pts));return None
            lane.append([list(p) for p in line.coords])
        offsets.append(lane)
    for i in range(len(runs)-1):
        li,pp=runs[i]; lj,qq=runs[i+1]
        dx,dy=pp[-1][0]-pp[-2][0],pp[-1][1]-pp[-2][1]; ll=math.hypot(dx,dy)
        direction=(dx/ll,dy/ll)

        if math.dist(qq[0],pp[-1])>1e-6: return None
        if (qq[1][0]-qq[0][0])*direction[0]+(qq[1][1]-qq[0][1])*direction[1] < math.dist(qq[0],qq[1])-.00001: return None
        pattern=_pair_via_pattern(a,b,li,lj,pp[-1],direction,side)
        if pattern is None or math.dist(pp[-2],pp[-1])<=pattern[0]['reach'] or math.dist(qq[0],qq[1])<=pattern[0]['reach']:
            _PAIR_SPLIT_REASON[0]='Via pattern or straight before/after: '+str((li,lj,pp,qq));return None
        for k in range(2):
            offsets[i][k]=offsets[i][k][:-1]+[list(p) for p in pattern[k]['before']]
            offsets[i+1][k]=[list(p) for p in pattern[k]['post']]+offsets[i+1][k][1:]
            vias[k].append(pattern[k]['via'])
    for i,(li,_) in enumerate(runs):
        for k,net in enumerate((a,b)):
            pts=offsets[i][k]
            if i==0: pts=start[k]['pts'][:-1]+pts
            if i==len(runs)-1: pts=pts+end[k]['pts'][-2::-1]
            pts=[p for j,p in enumerate(pts) if j==0 or math.dist(p,pts[j-1])>1e-8]
            w=_pair_ground(a,b,li)[k]
            if len(pts)<2 or not all(oct_(x,y) for x,y in zip(pts[:-1],pts[1:])) or not _pair_line_ok(net,li,pts,w,None if copper is None else copper[1-k][li]):
                _PAIR_SPLIT_REASON[0]='Conductor split not permitted: '+str((net,li,pts));return None
            out[k].append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),pair_search_space=True))
        ga=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[0] if LI[t['layer']]==li])
        gb=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[1] if LI[t['layer']]==li])
        if ga.distance(gb)<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:
            _PAIR_SPLIT_REASON[0]='Pair spacing at split';return None
    return out,vias


def _pair_tree_geometry(tracks,vias):
    result=[[],[]];vv=[[],[]]
    for k in range(2):
        for layer in LAYERS:
            tt=[t for t in tracks[k] if t['layer']==layer]
            if not tt:continue
            w=tt[0]['w'][0]
            assert all(all(abs(x-w)<1e-9 for x in t['w']) for t in tt)
            geom=shapely.line_merge(shapely.union_all([_LS(t['pts']) for t in tt]),directed=True)
            lines=[geom] if geom.geom_type=='LineString' else list(geom.geoms)
            for ln in lines:
                pts=[list(p) for p in ln.coords]
                if len(pts)>1:result[k].append(dict(layer=layer,pts=pts,w=[w]*(len(pts)-1),pair_search_space=True))
        seen=set()
        for v in vias[k]:
            key=(round(v['x'],8),round(v['y'],8))
            if key not in seen:seen.add(key);vv[k].append(v)
    return result,vv


_PAIR_SEARCH_INFO = {}

def _pair_planar_keepouts(C, path, src, tgt):
    candidates = [i for i in range(1, len(path)-1) if path[i] not in src and path[i] not in tgt
                  and not C['pair_tree'][path[i]]]
    if not candidates: return False
    def rank(i):
        x,y,z = path[i-1:i+2]
        dx,dy = y%NN%NX-x%NN%NX, y%NN//NX-x%NN//NX
        ex,ey = z%NN%NX-y%NN%NX, z%NN//NX-y%NN//NX
        return (0 if dx*ey-dy*ex else 1, abs(i-(len(path)-1)/2), i)
    n = path[min(candidates, key=rank)]
    C['pair_permissible'][n//NN, :, n%NN] = False
    return True


def _pair_edge(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,source,target):
    allowed = C['pair_permissible']; details = []; rejected = False
    forced = C.get('pair_via_forced', False)
    try:
        for with_vias in (False, True):
            C['pair_permissible'] = allowed.copy() if not with_vias else allowed
            attempts = 1 if with_vias else max(1, int(args.get('pair_planar_attempts', '3')))
            if not with_vias and not ({n//NN for n in src}&{n//NN for n in tgt}): continue
            for attempt in range(attempts):
                if time_tight(): return None, None, details
                ASTAR_COST[0] = None
                C['pair_via_forced'] = with_vias and rejected
                path = _pair_search(src,tgt,hard,soft,viaH if with_vias else np.zeros_like(viaH),viaBad,C)
                if path is None or len(path)<2:
                    details.append('No path '+str((with_vias,attempt,len(src),len(tgt))))
                    break
                runs = _pair_center(path,hard,widths)
                l0,p0 = runs[0]; l1,p1 = runs[-1]; found = None
                if len(p0)>=2 and len(p1)>=2:
                    def direct(x,y):
                        d=math.dist(x,y); return ((y[0]-x[0])/d,(y[1]-x[1])/d)
                    start = _pair_connection(*source[:2],l0,p0[0],direct(p0[0],p0[1]),side,a,b,copper)
                    end = _pair_connection(*target[:2],l1,p1[-1],direct(p1[-1],p1[-2]),-side,a,b,copper)
                    if start is not None and end is not None:
                        found = _pair_split(runs,a,b,side,start,end,copper)
                        if found is not None and not _pair_tree_free([alltr[k]+found[0][k] for k in range(2)], [allvs[k]+found[1][k] for k in range(2)]):
                            found = None; _PAIR_SPLIT_REASON[0] = 'Pair branch violates existing trunk'
                    else: _PAIR_SPLIT_REASON[0] = 'PadPort-Direction'
                else: _PAIR_SPLIT_REASON[0] = 'Layer segment too short'
                if found is not None:
                    if not with_vias and attempt: C['pair_planar_fallback'] = C.get('pair_planar_fallback', 0)+1
                    return found, path, details
                details.append(('Via' if with_vias else 'Planar')+' '+str(attempt)+': '+str(_PAIR_SPLIT_REASON[0]))
                if with_vias: break
                rejected = True
                C['pair_planar_rejected'] = C.get('pair_planar_rejected', 0)+1
                if attempt+1<attempts and not _pair_planar_keepouts(C,path,src,tgt): break
        return None, None, details
    finally:
        C['pair_permissible'] = allowed
        C['pair_via_forced'] = forced


def route_pair(u):
    _pair_fixed_verify(u)
    a,b=sorted((u,PAIR_NETS[u])); old={n:U.get(n) for n in (a,b)}
    anchors=copy.deepcopy(VIA_PREV); success=False
    for n in (a,b): del_unit(n)
    try:
        groups=_pair_pad_groups(a,b)
        if groups is None: _PAIR_SEARCH_INFO[a]=dict(reason='Pad pair assignment unknown'); return False
        hard,soft=_pair_hard(a,b); widths=tuple(_pair_ground(a,b,li)[4] for li in range(NL))

        C=dict(cls(a),w=max(widths),wmin=min(widths),zw=widths,via=max(12.,2*cls(a)['via']*VIA_FACTOR[0]),inner=1.,pen=3.)
        C['lf']=pair_lf(1.) or tuple(1. if li in (0,NL-1) else 4. for li in range(NL))
        NET_CUR[0]=a; ASTAR_COST[0]=None; ASTAR_HINT[0]=None; VIA_TOO[0]=None

        viaH=_pair_via_mask(a,b,hard)
        viaBad=np.logical_or.reduce([v<VIA_D for v in soft])


        def root_key(i):
            p,q,_=groups[i]; ref=p['ref']
            driver=all(x.get('pintype') in ('output','tri_state') for x in (p,q))
            chip=ref.startswith(('U','IC')) and sum(g[0]['ref']==ref for g in groups)==1 and all(x.get('pintype') in ('output','tri_state','bidirectional') for x in (p,q))
            return (0 if driver else 1 if chip else 2,ref,str(p['num']))
        root=min(range(len(groups)),key=root_key)
        def middle(g):return ((g[0]['x']+g[1]['x'])/2,(g[0]['y']+g[1]['y'])/2)
        edges=[(root,j) for j in sorted((j for j in range(len(groups)) if j!=root),key=lambda j:(math.dist(middle(groups[root]),middle(groups[j])),j))]
        C['pair_tree']=np.zeros(NL*NN,dtype=np.bool_)
        alltr=[[],[]]; allvs=[[],[]]
        for i,j in edges:
            found=None;details=[]
            dx,dy=middle(groups[j])[0]-middle(groups[i])[0],middle(groups[j])[1]-middle(groups[i])[1]
            p=groups[i][0];mid=middle(groups[i])
            preferred=1 if dx*(p['y']-mid[1])-dy*(p['x']-mid[0])>=0 else -1
            for side in (preferred,-preferred):
                if time_tight(): return False
                C['pair_permissible'],copper=_pair_states(a,b,side,alltr,allvs)
                viaH=_pair_via_mask(a,b,hard,side,copper)
                src=_pair_ports(groups[i],a,b,hard,side,copper); tgt=_pair_ports(groups[j],a,b,hard,-side,copper)
                if not src or not tgt: details.append('No pad ports '+str((len(src),len(tgt))));continue
                found,path,attempts = _pair_edge(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,groups[i],groups[j])
                details += attempts
                if found is not None: C['pair_tree'][path] = True
                if found is not None: break
            if found is None:
                _PAIR_SEARCH_INFO[a]=dict(reason='No shared channel with legal pad/via split',edges=len(edges),details=details);return False
            for k in range(2): alltr[k]+=found[0][k]; allvs[k]+=found[1][k]
        if time_tight(): return False
        alltr,allvs=_pair_tree_geometry(alltr,allvs)
        if not _pair_tree_free(alltr,allvs):
            _PAIR_SEARCH_INFO[a]=dict(reason='Pair branches/vias violate spacing');return False
        for k,n in enumerate((a,b)): add_unit(n,n,alltr[k],allvs[k])

        if RET>0 and (allvs[0] or allvs[1]) and set(RV_NET.values()).issubset(NID):
            RK=rv_map(a,(min(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,min(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,max(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2,max(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2),C['via'])
            if RK is not None:
                extra=rv_set(a,alltr[0]+alltr[1],allvs[0]+allvs[1],RK)
                del_unit(a);add_unit(a,a,alltr[0],allvs[0]+extra)
        if not time_tight(): lengths_match(only_pairs=True,only_nets={a,b})
        success=not time_tight()
        _PAIR_SEARCH_INFO[a]=dict(reason='Shared channel' if success else 'CPU-Budget',edges=len(edges),tree_begin=groups[root][0]['ref'],vias=[len(v) for v in allvs],planar_rejected=C.get('pair_planar_rejected',0),planar_fallback=C.get('pair_planar_fallback',0))
        return success
    finally:
        if not success:
            for n in (a,b): del_unit(n)
            for n,r in old.items():
                if r is not None: add_unit(n,r['net'],r['tracks'],r['vias'])
                else: add_unit(n,n,[],[])
            VIA_PREV.clear();VIA_PREV.update(anchors)
        _CAND_IDX[0]=None;ASTAR_COST[0]=None


def _pair_distance_costs(d, nominal, strength):
    channel = np.abs(d - nominal) <= .75 * RES
    return np.where(channel, PAIR_CHANNEL, 1.0 + strength * np.minimum(np.abs(d-nominal)/max(nominal,RES), 3.0)).astype(np.float32)


def pair_extra(net, C, capS, ex):
    fr = pair_free_mm(net); e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    pn = PAIR_NETS[net]
    cnt = use.copy()
    if pn in U: cnt[U[pn]['cells']] -= 1
    for li in range(NL) if PAIR_DISTURB > 1 else ():
        sl = slice(li * NN, (li + 1) * NN); f = cnt[sl] > 0
        if not f.any(): continue
        d = (_edt(~f.reshape(NY, NX)) * RES).ravel()
        e[sl] = np.where(d - C['w'] / 2 < fr, e[sl] * np.float32(PAIR_DISTURB), e[sl])
    if pn in U and U[pn]['tracks']:
        strength = PAIR_MOVE if pair_fast(net) else .25 * PAIR_MOVE
        m = np.zeros(NL * NN, dtype=bool)
        for t in U[pn]['tracks']:
            li = LI[t['layer']]
            for a, b in zip(t['pts'][:-1], t['pts'][1:]): m[seg_cells(li, a, b, 0.0)] = True
        nominal = max(C['w'], (NC['net_list'].get(pn) or {}).get('w', C['w'])) / 2 + C['w'] / 2 + PAIR_GAP
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if not m[sl].any():
                e[sl] *= np.float32(1.0 + 3*strength)
                continue
            if net in IMP_TAB:
                nominal = z_width(net, li, C['w']) / 2 + z_width(pn, li, C['w']) / 2 + z_gap(net, li)
            d = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
            e[sl] *= _pair_distance_costs(d,nominal,strength)
    return e


def _oct_move(pts):
    out = [pts[0]]
    for q in pts[1:]:
        c = _corners(out[-1], q)
        if c: out.append(c[1])
        out.append(q)
    return out


def _pair_path_groups(net):
    from analysis.length_impedance import length_list, length_groups
    k = _pair_tolerances(copy.deepcopy(NC))
    groups = length_groups(k)
    selected = {g:q for g,q in groups.items() if net in q.get('net_list', [])}
    k['specs'] = dict(k.get('specs') or {},length_groups=selected)
    for n, entry in k['net_list'].items():
        entry.pop('length_group',None)
        if n in (net,PAIR_NETS[net]) and pair_fast(n): entry['z_diff_ohm'] = entry.get('z_diff_ohm') or 1
        else: entry.pop('z_diff_ohm',None)
    tracks = [dict(t,net=r['net']) for r in U.values() for t in r['tracks']]
    vias = [dict(v,net=v.get('net',r['net'])) for r in U.values() for v in r['vias']]
    return length_list(D,dict(tracks=tracks,vias=vias),k)


def _pair_vias_allowed(vias):
    for v in vias:
        ix, iy = math.floor((v['x']-X0)/RES), math.floor((v['y']-Y0)/RES)
        if not (0 <= ix < NX and 0 <= iy < NY): return False
        if VSP[iy, ix] < H2H + VIA_DR/2 + .01: return False
        if VIA_BAN is not None and VIA_BAN[iy, ix]: return False
    return True


def _pair_candidate(net, tracks, vias, verify, gates=True):
    if gates and (time_tight() or not _pair_vias_allowed(vias)): return False
    old = U[net]; accepted = False
    anchor_present = net in VIA_PREV; anchor = copy.deepcopy(VIA_PREV.get(net))
    del_unit(net)
    try:
        add_unit(net, old['net'], tracks, vias)
        accepted = (not gates or not time_tight()) and unit_conflict_free(net) and verify() and (not gates or not time_tight())
        if accepted: cand_idx_changed(net)
        return accepted
    finally:
        if not accepted:
            del_unit(net); add_unit(net, old['net'], old['tracks'], old['vias']); cand_idx_back(net)
            if gates:
                if anchor_present: VIA_PREV[net] = anchor
                else: VIA_PREV.pop(net, None)


def _pair_shift(a, b):
    from analysis.length_impedance import path_lengths
    from shapely.geometry import LineString, Point
    if time_tight(): return False
    rp, old = U[a], U[b]
    if any(v.get('rv') or v.get('gr') for r in (rp,old) for v in r['vias']): return False
    if not rp['tracks']: return False
    base = max(rp['tracks'], key=lambda t: sum(math.dist(p, q) for p, q in zip(t['pts'][:-1], t['pts'][1:])))
    dx, dy = base['pts'][-1][0]-base['pts'][0][0], base['pts'][-1][1]-base['pts'][0][1]

    directions = [(0, 1), (1, 0)] if abs(dx) >= abs(dy) else [(1, 0), (0, 1)]
    if dx and dy: directions.insert(0, (-math.copysign(1/math.sqrt(2), dy), math.copysign(1/math.sqrt(2), dx)))
    w = cls(b)['w']
    board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
    before = next((p for p in pairs_report(only={a,b},prev_gate=not PAIR_SEARCH_SPACE) if p['net_list'] == sorted((a, b))), None)
    if before is None: return False
    before_groups = _pair_path_groups(b)
    for nx, ny in directions:
        if time_tight(): return False
        for side in (1, -1):
            if time_tight(): return False
            shifts = {LI[t['layer']]: (side*(_pair_ground(a,b,LI[t['layer']])[3] if PAIR_SEARCH_SPACE else (cls(a)['w']+w)/2+PAIR_GAP)*nx,
                                      side*(_pair_ground(a,b,LI[t['layer']])[3] if PAIR_SEARCH_SPACE else (cls(a)['w']+w)/2+PAIR_GAP)*ny) for t in rp['tracks']}
            if rp['vias'] and len(set(shifts.values())) > 1: continue
            shift = next(iter(shifts.values()))
            tracks = [dict(layer=t['layer'], pts=[[round(p[0]+shifts[LI[t['layer']]][0],4),round(p[1]+shifts[LI[t['layer']]][1],4)] for p in t['pts']],
                           w=[z_width(b, LI[t['layer']], w)]*len(t['w'])) for t in rp['tracks']]
            vias = [dict(x=round(v['x']+shift[0],4),y=round(v['y']+shift[1],4)) for v in rp['vias']]
            valid = True
            for ai in by_net[b]:
                if time_tight(): return False
                pad = D['pads'][ai]
                lines = [(LineString(t['pts']), t) for t in tracks if t['layer'] in pad['layers']]
                if not lines: valid = False; break
                point = Point(pad['x'], pad['y']); line, t = min(lines, key=lambda v: v[0].distance(point))
                q = line.interpolate(line.project(point)); pts = _oct_move([(pad['x'],pad['y']),(q.x,q.y)])
                pts = [p for i,p in enumerate(pts) if i == 0 or math.dist(p,pts[i-1]) > 1e-6]
                if len(pts)>1: tracks.append(dict(layer=t['layer'],pts=[list(p) for p in pts],w=[w]*(len(pts)-1)))
            if not valid or any(not board.covers(LineString(t['pts']).buffer(max(t['w'])/2)) for t in tracks): continue
            if any(not board.covers(Point(v['x'],v['y']).buffer(VIA_R)) for v in vias): continue
            if IMP_REF_MASK and any(not z_ref_free(cls(b), LI[t['layer']], p, q, b) for t in tracks for p,q in zip(t['pts'][:-1],t['pts'][1:])): continue
            if time_tight(): return False
            def verify():
                paths = path_lengths(D,dict(tracks=[dict(t,net=b) for t in U[b]['tracks']],
                    vias=[dict(v,net=b) for v in U[b]['vias']]),b,DESIGN_RULES_STRUCTURE,all_pads=True)
                after = next(p for p in pairs_report(only={a,b},prev_gate=not PAIR_SEARCH_SPACE) if p['net_list'] == sorted((a,b)))
                good = bool(paths) and all(p['length_mm'] is not None for p in paths)
                good = good and after['coupled'] > before['coupled'] + .01 and after['skewed_mm'] <= before['skewed_mm'] + (1e-6 if PAIR_SEARCH_SPACE else .15)
                good = good and _receiver_not_worse(before_groups,_pair_path_groups(b),b)
                return good
            if _pair_candidate(b, tracks, vias, verify): return True
    return False


def pair_couple(only=None):
    from shapely.geometry import LineString as _L2
    n_ok = 0; _CAND_IDX[0] = None
    for P in sorted(PAIR_NETS):
        if time_tight(): break
        N = PAIR_NETS[P]
        if P > N or P not in U or N not in U or pair_unit(P): continue
        if only is not None and P not in only: continue
        if any(v.get('rv') or v.get('gr') for q in (P,N) for v in U[q]['vias']): continue
        if any(n_sv(U[q]) or len(U[q]['tracks']) != 1 or len(by_net.get(q, ())) != 2 for q in (P,N)):
            if _pair_shift(P,N) or _pair_shift(N,P): n_ok += 1
            continue
        for a_, b_ in ((P, N), (N, P)):
            if time_tight(): break
            rp, rn = U[a_], U[b_]
            if n_sv(rp) or len(rp['tracks']) != 1 or len(by_net.get(a_, ())) != 2 or len(by_net.get(b_, ())) != 2: continue
            tp = rp['tracks'][0]; li = LI[tp['layer']]; Pp = [tuple(q) for q in tp['pts']]
            if len(Pp) < 2: continue
            pn = [D['pads'][k] for k in by_net[b_]]
            if not all(tp['layer'] in q['layers'] for q in pn): continue
            Cn = cls(b_); w = z_width(b_, li, Cn['w'])
            nominal = (_pair_ground(a_,b_,li)[3] if PAIR_SEARCH_SPACE else
                    ((z_width(a_, li, cls(a_)['w'])/2 + w/2 + z_gap(b_, li)) if b_ in IMP_TAB else
                     max(Cn['w'], cls(a_)['w'])/2 + w/2 + PAIR_GAP))
            pa = min(pn, key=lambda q: math.dist((q['x'], q['y']), Pp[0])); pb = [q for q in pn if q is not pa][0]
            dx, dy = Pp[1][0] - Pp[0][0], Pp[1][1] - Pp[0][1]
            board_side = 1.0 if (dx * (pa['y'] - Pp[0][1]) - dy * (pa['x'] - Pp[0][0])) > 0 else -1.0
            try: o = _L2(Pp).offset_curve(board_side * nominal, join_style=2, mitre_limit=3.0)
            except Exception: continue
            if o.geom_type != 'LineString' or o.is_empty: continue
            oq = [(round(x, 4), round(y, 4)) for x, y in o.coords]
            if math.dist(oq[0], (pa['x'], pa['y'])) > math.dist(oq[-1], (pa['x'], pa['y'])): oq = oq[::-1]
            move = [(pa['x'], pa['y'])] + oq + [(pb['x'], pb['y'])]
            if oct_net(b_): move = _oct_move(move)
            move = [q for i, q in enumerate(move) if i == 0 or math.dist(q, move[i - 1]) > 1e-6]
            new_t = [dict(net=b_, layer=tp['layer'], pts=[[x, y] for x, y in move], w=[w] * (len(move) - 1))]
            if IMP_REF_MASK and not all(z_ref_free(Cn, li, a, b, b_) for a, b in zip(move[:-1], move[1:])): continue
            Lp = sum(math.dist(x, y) for x, y in zip(Pp[:-1], Pp[1:])); Ln = sum(math.dist(x, y) for x, y in zip(move[:-1], move[1:]))
            L0 = sum(math.dist(x, y) for t in rn['tracks'] for x, y in zip(t['pts'][:-1], t['pts'][1:]))


            if abs(Ln - Lp) > abs(L0 - Lp) + (1e-6 if PAIR_SEARCH_SPACE else .15) and n_sv(rn) == 0: continue
            if not _L2(move).is_simple: continue
            board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
            if not board.covers(_L2(move).buffer(w/2)): continue
            before_groups = _pair_path_groups(b_)
            if _pair_candidate(b_, new_t, [], lambda: _receiver_not_worse(before_groups,_pair_path_groups(b_),b_)):
                n_ok += 1; break
    _CAND_IDX[0] = None
    return n_ok


def pairs_report(receiver=False, only=None, prev_gate=False):
    out = []
    for n in sorted(PAIR_NETS):
        pn = PAIR_NETS[n]
        if n > pn: continue
        if only is not None and n not in only: continue
        category = NC['net_list'][n].get('pair_kind', NC['net_list'][n].get('category'))
        if n not in U or pn not in U:
            out.append(dict(net_list=[n,pn], category=category, total_length=[None,None], skewed_mm=None,
                vias=[n_sv(U[q]) if q in U else None for q in (n,pn)], coupled=0.0, incomplete=True))
            continue
        L = {q: sum(math.dist(a,b) for t in U[q]['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) for q in (n,pn)}
        nominal = cls(n)['w']/2 + cls(pn)['w']/2 + PAIR_GAP
        fractions = []; spacings = {}
        for q, partner in ((n,pn),(pn,n)):
            shortened = 0.0
            for t in U[q]['tracks']:
                li = LI[t['layer']]
                distance = (_pair_ground(q,partner,li)[3] if PAIR_SEARCH_SPACE else
                            ((z_width(q,li,cls(q)['w'])/2 + z_width(partner,li,cls(partner)['w'])/2 + z_gap(q,li)) if q in IMP_TAB else nominal))
                spacings[t['layer']] = round(distance,4)
                for a,b in zip(t['pts'][:-1],t['pts'][1:]):
                    length = math.dist(a,b)
                    if length < 1e-8: continue
                    k = max(1, int(math.ceil(length/.05)))
                    parallel = [(c,d) for t2 in U[partner]['tracks'] if t2['layer'] == t['layer'] for c,d in zip(t2['pts'][:-1],t2['pts'][1:])
                                if math.dist(c,d)>1e-8 and abs((b[0]-a[0])*(d[0]-c[0])+(b[1]-a[1])*(d[1]-c[1]))/(length*math.dist(c,d)) > .995]
                    for i in range(k):
                        pt=(a[0]+(b[0]-a[0])*(i+.5)/k,a[1]+(b[1]-a[1])*(i+.5)/k)

                        dm=min((_seg_dist(pt,c,d) for c,d in parallel if prev_gate or
                                0 <= (pt[0]-c[0])*(d[0]-c[0])+(pt[1]-c[1])*(d[1]-c[1]) <= (d[0]-c[0])**2+(d[1]-c[1])**2),default=math.inf)
                        if abs(dm-distance)<=.05: shortened += length/k
            fractions.append(shortened/max(L[q],1e-9))
        out.append(dict(net_list=[n,pn], category=category, total_length=[round(L[n],3),round(L[pn],3)],
            skewed_mm=round(abs(L[n]-L[pn]),3), vias=[n_sv(U[n]),n_sv(U[pn])],
            coupled=round(min(fractions),3), coupled_per_net=[round(f,3) for f in fractions],
            spacing_nominal=round(nominal,3), spacing_per_layer=spacings))
    if receiver:
        from analysis.length_impedance import length_list
        k = _pair_tolerances(copy.deepcopy(NC))
        for n in PAIR_NETS:
            if pair_fast(n): k['net_list'][n]['z_diff_ohm'] = k['net_list'][n].get('z_diff_ohm') or 1
        tr,vi = export_geometry(); groups=length_list(D,dict(tracks=tr,vias=vi),k)
        for p in out:
            g=groups.get('pair:'+'|'.join(p['net_list']))
            p.update(skew_paths_mm=g['error_mm'] if g else None, tolerance_mm=g['tolerance_mm'] if g else None,
                     skew_in_tolerance=bool(g and g['complete'] and g['over_mm']<=1e-6),
                     receiver=g['receiver'] if g else {})
    return out


def _receiver_not_worse(before, after, net):
    from analysis.length_impedance import receiver_protection
    return receiver_protection(before, after, {net}, strict=PAIR_SEARCH_SPACE)


def lengths_match(only_pairs=False, only_nets=None):
    from analysis.length_impedance import length_list
    from shapely.geometry import LineString, Point
    board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
    structure = (NC.get('specs') or {}).get('structure') or NC.get('structure') or DESIGN_RULES_STRUCTURE
    def routes():
        tr, vi = export_geometry()
        return dict(tracks=tr, vias=vi)
    length_nc = _pair_tolerances(copy.deepcopy(NC))
    if only_pairs:
        length_nc['specs'] = dict(length_nc.get('specs') or {}, length_groups={})
        for k in length_nc['net_list'].values(): k.pop('length_group', None); k.pop('z_diff_ohm', None)


    for n in PAIR_NETS:
        if pair_fast(n): length_nc['net_list'][n]['z_diff_ohm'] = length_nc['net_list'][n].get('z_diff_ohm') or 1
    groups = length_list(D, routes(), length_nc)
    lengths = {n: v for g in groups.values() for n, v in g['net_list'].items()}
    targets = {n: v for n, v in lengths.items() if v is not None}

    for _ in range(len(groups) + 1):
        changed = False
        for g in groups.values():
            if not g['complete']: continue
            target = max(targets[n] for n in g['net_list'])
            for n in g['net_list']:
                if targets[n] < target: targets[n] = target; changed = True
        if not changed: break
    accepted = []; tried = 0; _CAND_IDX[0] = None
    for net in sorted(targets):
        if only_nets is not None and net not in only_nets: continue
        if net not in U or net.startswith(('FIX#', 'TACK#', 'GND#')): continue


        full_groups = _pair_path_groups(net) if only_pairs and net in PAIR_NETS else None
        tol = min((g['tolerance_mm'] for g in groups.values() if net in g['net_list']), default=0.15)
        for _ in range(int(args.get('lengths_moves', '12'))):
            if time_tight(0.95) or targets[net] - lengths[net] <= tol * 0.8: break
            old = U[net]; delta = targets[net] - lengths[net]; success = False
            segments = sorted([(math.dist(a, b), ti, si, a, b, w) for ti, t in enumerate(old['tracks'])
                               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))], reverse=True)
            for L, ti, si, a, b, w in segments:
                if not oct_(a, b): continue
                layer = old['tracks'][ti]['layer']; c = max(3 * w, 0.15); top = 4 * w + 0.02
                span = 4 * c + top; lead = (L - span) / 2
                compact = net in PAIR_NETS and (PAIR_SEARCH_SPACE or delta <= 0.6)
                if compact:



                    if PAIR_SEARCH_SPACE: delta=max(0.,targets[net]-lengths[net]-.75*tol)
                    height = delta / (2 * (math.sqrt(2) - 1)); span = 2*height; lead = (L-span)/2
                if lead < 4 * w + 0.02: continue
                e = ((b[0] - a[0]) / L, (b[1] - a[1]) / L); normal = (-e[1], e[0])

                for fraction in (1.0, 0.5, 0.25, 0.125):
                    add = delta * fraction
                    height = add / (2 * (math.sqrt(2) - 1)) if compact else add / 2 + 2 * c * (2 - math.sqrt(2))
                    if not compact and height < 2 * c + 0.02: continue
                    if compact:
                        span = 2*height; lead = (L-span)/2
                        if lead < 4*w+.02: continue
                    if PAIR_SEARCH_SPACE and net in PAIR_NETS:
                        receivers=[]
                        for g in groups.values():
                            if net not in g['net_list'] or not g['complete'] or g.get('error_mm',float('inf'))<=g['tolerance_mm']: continue
                            for path in g.get('paths',{}).get(net,[]):
                                ref,num=(path.get('receiver') or ':').rsplit(':',1)
                                receivers += [(p['x'],p['y']) for p in D.get('pads',[]) if p['net']==net and p['ref']==ref and str(p['num'])==num]
                        local_span=2*height if compact else span
                        available=L-local_span
                        margin=4*w+.02
                        if available>=2*margin and receivers:
                            da=min(math.dist(a,p) for p in receivers);db=min(math.dist(b,p) for p in receivers)
                            lead=margin if da<db else available-margin

                    positions=[lead]
                    if compact and PAIR_SEARCH_SPACE:
                        available=L-span
                        direction=1. if lead<available/2 else -1.
                        margin=4*w+.02
                        positions += [min(available-margin,max(margin,lead+direction*d)) for d in (.5,1.)]
                        positions += [available/2]
                    for lead in dict.fromkeys(positions):

                        foreign = [LineString([pa, pb]).buffer(ww/2) for r2 in U.values() if r2['net'] != net for tt in r2['tracks'] if tt['layer'] == layer
                                   for pa, pb, ww in zip(tt['pts'][:-1], tt['pts'][1:], tt['w'])]
                        foreign += [g for nn, g in _PADG[LI[layer]] if nn != net]
                        def clearance(side):
                            mid = (Point(a[0]+(lead+(height if compact else span/2))*e[0] + side*height*normal[0], a[1]+(lead+(height if compact else span/2))*e[1] + side*height*normal[1])
                                   if PAIR_SEARCH_SPACE else Point((a[0]+b[0])/2 + side*height*normal[0], (a[1]+b[1])/2 + side*height*normal[1]))
                            return min((mid.distance(g) for g in foreign), default=1e9)
                        for side in sorted((1, -1), key=lambda s: (-clearance(s), -s)):
                            if time_tight(0.95): break
                            tried += 1
                            q = [(0, 0), (lead, 0), (lead + c, c), (lead + c, height - c),
                                 (lead + 2*c, height), (lead + 2*c + top, height),
                                 (lead + 3*c + top, height - c), (lead + 3*c + top, c),
                                 (lead + span, 0), (L, 0)]
                            if compact:
                                if not PAIR_SEARCH_SPACE: lead = (L - 2*height)/2
                                q = [(0,0),(lead,0),(lead+height,height),(lead+2*height,0),(L,0)]
                            pts = [[round(a[0] + x*e[0] + side*y*normal[0], 4),
                                    round(a[1] + x*e[1] + side*y*normal[1], 4)] for x, y in q]
                            pts[0], pts[-1] = list(a), list(b)
                            line = LineString(pts); copper = line.buffer(w/2, cap_style=1)
                            if not line.is_simple or not board.covers(copper): continue
                            if IMP_REF_MASK and not all(z_ref_free(cls(net), LI[layer], a, b, net) for a, b in zip(pts[:-1], pts[1:])): continue

                            pocket = LineString(pts[1:-1])
                            own_units = [(u2, r2) for u2, r2 in U.items() if r2['net'] == net]
                            own = [LineString([pa, pb]).buffer(ww/2) for u2, r2 in own_units for tj, t in enumerate(r2['tracks']) if t['layer'] == layer
                                   for sj, (pa, pb, ww) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if (u2, tj, sj) != (net, ti, si)]
                            own += [Point(v['x'], v['y']).buffer(VIA_R) for _, r2 in own_units for v in r2['vias']]
                            own += [g for n, g in _PADG[LI[layer]] if n == net]
                            if any(pocket.distance(g) < 3*w + w/2 - 1e-5 for g in own): continue
                            new = copy.deepcopy(old['tracks']); t = new[ti]
                            t['pts'] = t['pts'][:si] + pts + t['pts'][si+2:]
                            t['w'] = t['w'][:si] + [w]*(len(pts)-1) + t['w'][si+1:]
                            result = {}
                            def verify():
                                after = length_list(D, routes(), length_nc)
                                observed = [g['net_list'][net] for g in after.values() if net in g['net_list']]
                                length = max(observed) if observed and all(v is not None for v in observed) else None
                                good = length is not None and lengths[net] + 1e-4 < length <= targets[net] + 0.001
                                good = good and _receiver_not_worse(groups, after, net)
                                after_full = _pair_path_groups(net) if good and full_groups is not None else None
                                if full_groups is not None:
                                    good = good and _receiver_not_worse(full_groups, after_full, net)
                                result.update(after=after, length=length, full=after_full)
                                return good
                            if _pair_candidate(net, new, old['vias'], verify, gates=PAIR_SEARCH_SPACE):
                                length = result['length']
                                accepted.append(dict(net=net, layer=layer, before_mm=lengths[net], after_mm=length, w=w))
                                if full_groups is not None: full_groups = result['full']
                                lengths[net] = length; groups = result['after']; success = True; break
                        if success: break
                    if success: break
                if success: break
            if not success: break
    _CAND_IDX[0] = None
    return dict(move_records=accepted, attempts=tried, group_list=length_list(D, routes(), length_nc))


def sw_extra(net):
    if SW_DIST <= 0 or not any(k in net for k in SENS): return None
    sig = sum(_NVER.get(n, 0) for n in _SWN)
    c = _NEARBY_C.get('sw_prev')
    if c is not None and c[0] == sig: return c[1]
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'].split('/')[-1].startswith('SW_'):
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'].split('/')[-1].startswith('SW_'): m[r['cells']] = True
    ex = np.ones(NL * NN, dtype=np.float32)
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < SW_DIST, 6.0, 1.0)
    _NEARBY_C['sw_prev'] = (sig, ex)
    return ex


def geo_nodes(trs, vs):
    out = set()
    for t in trs:
        li = LI[t['layer']]
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.25)) + 1)
            for k in range(n):
                x = a[0] + (b[0] - a[0]) * k / (n - 1); y = a[1] + (b[1] - a[1]) * k / (n - 1)
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if math.hypot(X0 + (ix + 0.5) * RES - x, Y0 + (iy + 0.5) * RES - y) < 0.03:
                    out.add(li * NN + iy * NX + ix)
                    if BRANCH_EXACT:
                        _l = _TRUNK.setdefault(li * NN + iy * NX + ix, [])
                        if not any(_t is t for _t in _l): _l.append(t)
    for v in vs:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        for li in range(NL):
            out.add(li * NN + iy * NX + ix)
            if BRANCH_EXACT: _TRUNK_V[li * NN + iy * NX + ix] = (v['x'], v['y'])
    return sorted(out)







CONS = args.get('via_group', '1' if NC else '0') == '1'
CONS_DIST = max(float(args.get('cons_spacing', '0.7')), HOLE_C)
CONS_IVIA = float(args.get('cons_current03', '1.0')) * VIA_DR / 0.3
CURRENT_VIA_PRICE = args.get('current_via_price', '0') == '1' and NC is not None
CONS_RES = float(args.get('cons_reserve', '1.3')); CONS_MIN = float(args.get('cons_minimum', '0.5')); CONS_MAX = int(args.get('cons_max', '0' if CURRENT_VIA_PRICE else '4'))
CONS_REL = args.get('cons_relative', '0') == '1'
CONS_SOFT = [True]


def cons_count(net, pads, tree, j, with_=()):
    k = NC['net_list'].get(net) or {}; fl = k.get('flow') or {}
    if k.get('category') not in ('current_val', 'supply', 'sw') or not (fl.get('unit') == 'A' or CONS_REL): return 1
    load = fl.get('load') or {}; source = set(fl.get('source') or [])
    def nm(i): return '%s:%s' % (pads[i]['ref'], pads[i]['num'])
    lt = sum(float(load.get(nm(i), 0.0)) for i in tree if nm(i) not in source)
    lj = float(load.get(nm(j), 0.0))
    if with_: lj += sum(float(load.get(nm(i), 0.0)) for i in with_ if nm(i) not in source)
    if nm(j) in source or any(nm(i) in source for i in with_): current_val = lt
    elif any(nm(i) in source for i in tree): current_val = lj
    else: current_val = min(lj, lt)
    if current_val < CONS_MIN: return 1
    nominal = max(2, int(math.ceil(CONS_RES * current_val / CONS_IVIA - 1e-9)))
    return min(CONS_MAX, nominal) if CONS_MAX > 0 else nominal


def cons_set(net, res, trs, vs, n_nominal, capH, viaH, capC, vbadf, Cb, already, rng):
    viaf = viaH.ravel(); w_st = max(TRACK_MIN, min(Cb['wmin'], VIA_D))
    change = [(a // NN, b // NN) for a, b in zip(res[:-1], res[1:]) if a // NN != b // NN]
    own = [(v['x'], v['y']) for v in already] + [(v['x'], v['y']) for v in vs]
    bridges, new_v = [], []
    for v, (la, lb) in zip(vs, change):
        v['n'] = n_nominal; vx, vy = v['x'], v['y']
        cand = []
        for t in trs:
            li = LI[t['layer']]
            if li not in (la, lb): continue
            for pts in (t['pts'], t['pts'][::-1]):
                if abs(pts[0][0] - vx) > 1e-6 or abs(pts[0][1] - vy) > 1e-6: continue
                way = 0.0; k = 1
                for a, b in zip(pts[:-1], pts[1:]):
                    L = math.hypot(b[0] - a[0], b[1] - a[1])
                    while k < n_nominal and L > 0 and way + L >= k * CONS_DIST - 1e-9:
                        f = (k * CONS_DIST - way) / L
                        cand.append((k, li, a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))); k += 1
                    way += L
                    if k >= n_nominal: break
        rings = [(10, CONS_DIST), (11, 1.5 * CONS_DIST)]
        if CURRENT_VIA_PRICE:
            rings += [(12 + k, (2 + k) * CONS_DIST) for k in range(max(1, int(math.ceil(math.sqrt(n_nominal))))) ]
        for ring, r in rings:
            nq = 8 if ring < 12 else max(8, int(2 * math.pi * r / CONS_DIST))
            for q in range(nq):
                cand.append((ring, -1, vx + r * math.cos(q * 2 * math.pi / nq), vy + r * math.sin(q * 2 * math.pi / nq)))
        missing = n_nominal - 1
        for soft in ((False, True) if CONS_SOFT[0] else (False,)):
            grid_map = capH if soft else capC
            for rank, lt, x, y in sorted(cand):
                if missing <= 0: break
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if not (rng[0] < ix < rng[1] - 1 and rng[2] < iy < rng[3] - 1): continue
                c = iy * NX + ix; x, y = cell_xy(ix, iy)
                if not viaf[c] or (not soft and vbadf[c]): continue
                if any(math.hypot(x - ex, y - ey) < CONS_DIST - 0.06 for ex, ey in own): continue
                st = []
                for lo in (la, lb):
                    if lo == lt: continue
                    if line_cap_min(grid_map[lo], (x, y), (vx, vy)) < w_st - 1e-6 or pad_room(lo, (x, y), (vx, vy), net) < w_st / 2: st = None; break
                    st.append(dict(layer=LAYERS[lo], pts=[[round(x, 4), round(y, 4)], [vx, vy]], w=[w_st], web=True))
                if st is None: continue
                bridges += st; new_v.append(dict(x=round(x, 4), y=round(y, 4), gr=1, too=[vx, vy])); own.append((x, y)); missing -= 1
    return bridges, new_v








RET = float(args.get('ret', '1.27')) if NC is not None else 0.0
RET_R3 = float(args.get('ret_r3', '2.5'))
RET_PRESENT = float(args.get('ret_present', '0.25')); RET_MISSING = float(args.get('ret_missing', '1.0'))
RV_PAIR = np.zeros(NL * NL, dtype=np.float64)
RV_NET = {}
if RET > 0:



    for _li, _a in enumerate(STRUCTURE):
        for _lj, _b in enumerate(STRUCTURE):
            if _li == _lj or not _a['ref'] or not _b['ref'] or _a['ref'] == _b['ref']: continue
            _gn = sorted({x for x in (_a['ref_net'], _b['ref_net']) if x in GNDS})
            if not _gn and GND_MAIN in AREA_NETS: _gn = [GND_MAIN]
            if len(_gn) == 1: RV_PAIR[_li * NL + _lj] = 1.0; RV_NET[(_li, _lj)] = _gn[0]
    if not RV_NET: RET = 0.0
    else: print('Return vias (Q2): %d layer pairs with different ground reference areas, radius %.2f mm (rank <= 3: %.2f mm)' % (
        len(RV_NET) // 2, RET, RET_R3), flush=True)
RV_STAT = dict(assigned=0, already=0, no_slot=0, post_run_new=0, post_run_way=0)


def rv_radius(net):
    if RET <= 0: return 0.0
    k = NC['net_list'].get(net) or {}
    if k.get('category') != 'fast': return 0.0
    return RET if k.get('rank', 0) >= 4 else RET_R3


def _rv_gnd(net_name):
    xy = [(v['x'], v['y']) for r in U.values() for v in r['vias'] if (v.get('net') or r['net']) == net_name]
    xy += [(D['pads'][k]['x'], D['pads'][k]['y']) for k in by_net_g.get(net_name, ()) if D['pads'][k].get('drill')]
    return np.array(xy, dtype=np.float64).reshape(-1, 2)


def rv_map(net, box, VC):
    R = rv_radius(net)
    if R <= 0: return None
    if PLANE_LAYER:
        local={g:_rv_map_net(g,box,VC,R) for g in sorted(GNDS & PLANE_AREA.keys())}
        if not local:return None

        via_extra=np.full(NN,np.float32(RET_MISSING*VC),dtype=np.float32)
        for g,rk in local.items():
            mask=PLANE_MASK[g].ravel();via_extra[mask]=np.minimum(via_extra[mask],rk['via_extra'][mask])
        return dict(local=local,via_extra=via_extra,R=R)
    net_list = sorted(set(RV_NET.values()))
    if len(net_list) != 1: return None
    g = net_list[0]
    return _rv_map_net(g,box,VC,R)


def _rv_map_net(g, box, VC, R):
    bxR = (box[0] - R, box[1] - R, box[2] + R, box[3] + R)
    ix0, ix1, iy0, iy1 = _box_idx(bxR)
    ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    rng = (ix0, ix1, iy0, iy1)
    _, vH = hard_maps(g, rng if LARGE else None)
    vacant = np.zeros((NY, NX), dtype=bool); f = np.array(vH[ys, xs], dtype=bool, copy=True)
    capS, dd, _r2 = soft_maps(g, bxR, VIA_D)
    f &= dd[ys, xs] >= HOLE_C
    for li in range(NL): f &= capS[li][ys, xs] / 2 >= VIA_R
    if PLANE_LAYER:
        xx,yy=np.meshgrid(GXc[ix0:ix1],GYc[iy0:iy1])
        for L,nn in FL_ASSIGNMENT.items():
            if g in nn:
                area=PLANE_LAYER.get((g,L),Polygon()).buffer(-VIA_R-.01)
                f &= shapely.contains_xy(area,xx,yy)
    vacant[ys, xs] = f
    via_extra = np.full(NN, np.float32(RET_MISSING * VC), dtype=np.float32); V = via_extra.reshape(NY, NX)
    if f.any(): V[ys, xs] = np.where(_edt(~f) * RES <= R, np.float32(RET_PRESENT * VC), V[ys, xs])
    gv = _rv_gnd(g)
    if len(gv):
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        jx = ((gv[:, 0] - X0) / RES).astype(int) - ix0; jy = ((gv[:, 1] - Y0) / RES).astype(int) - iy0
        ok = (jx >= 0) & (jx < ix1 - ix0) & (jy >= 0) & (jy < iy1 - iy0)
        m[jy[ok], jx[ok]] = True
        if m.any(): V[ys, xs] = np.where(_edt(~m) * RES <= R, np.float32(0.0), V[ys, xs])
    return dict(net_name=g, vacant=vacant, via_extra=via_extra, R=R)


def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _via_layers(net, v, trs):
    gen = set()
    for t in trs:
        if t.get('net', net) != net: continue
        for e in (t['pts'][0], t['pts'][-1]):
            if abs(e[0] - v['x']) < VIA_R + 0.05 and abs(e[1] - v['y']) < VIA_R + 0.05: gen.add(LI[t['layer']])
    for k in by_net_g.get(net, ()):
        p = D['pads'][k]; bb = p['bbox']
        if bb[0] - 0.01 <= v['x'] <= bb[2] + 0.01 and bb[1] - 0.01 <= v['y'] <= bb[3] + 0.01: gen |= {LI[L] for L in p['layers'] if L in LI}
    return gen


def rv_demand(net, v, trs):
    gen = sorted(_via_layers(net, v, trs))
    for i, a in enumerate(gen):
        for b in gen[i + 1:]:
            if RV_PAIR[a * NL + b] > 0:
                if PLANE_LAYER:
                    refs=[STRUCTURE[i]['ref'] for i in (a,b)]
                    mass=[L for L in refs if set(FL_ASSIGNMENT.get(L,[])) & GNDS]
                    pt=_PT(v['x'],v['y'])
                    domains=globals().get('PLANE_REF',PLANE_LAYER)
                    candidates=[g for g in sorted(GNDS) if mass and all(domains.get((g,L),Polygon()).covers(pt) for L in mass)]
                    return candidates[0] if len(candidates)==1 else None
                return RV_NET[(a, b)]
    return None


def rv_set(net, trs, vs, RK, owned=None):
    if 'local' in RK:
        new=[]
        for g,rk in sorted(RK['local'].items()):
            subset=[v for v in vs if rv_demand(net,v,trs)==g]
            new+=rv_set(net,trs,subset,rk,owned=(vs if owned is None else owned)+new)
        return new
    R = RK['R']; g = RK['net_name']; vacant = RK['vacant']; new = []
    gv = _rv_gnd(g)
    mine_v = [(v['x'], v['y']) for v in (vs if owned is None else owned)]
    r0 = VIA_D + DRCC + 0.02
    rings = [r0] + [r for r in (0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0, 2.25, 2.5) if r0 < r < R - 0.02] + [R - 0.02]
    for v in vs:
        if v.get('rv') or v.get('gr'): continue
        if rv_demand(net, v, trs) != g: continue
        all_items = [tuple(q) for q in gv.tolist()] + [(w['x'], w['y']) for w in new]
        if any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in all_items): RV_STAT['already'] += 1; continue
        ok = False
        for r in rings:
            for q in range(16):
                ix = int((v['x'] + r * math.cos(q * math.pi / 8) - X0) / RES); iy = int((v['y'] + r * math.sin(q * math.pi / 8) - Y0) / RES)
                if not (0 <= ix < NX and 0 <= iy < NY) or not vacant[iy, ix]: continue
                x, y = cell_xy(ix, iy)
                if math.hypot(x - v['x'], y - v['y']) > R: continue
                if any(math.hypot(x - a, y - b) < max(VIA_D + DRC_CLR, DRC_HOLE) + 0.01 for a, b in mine_v + [(w['x'], w['y']) for w in new]): continue
                if any(_seg_dist((x, y), a, b) < VIA_R + w_ / 2 + DRC_CLR + 0.01 for t in trs for a, b, w_ in zip(t['pts'][:-1], t['pts'][1:], t['w'])): continue
                new.append(dict(x=round(x, 4), y=round(y, 4), net=g, rv=True)); ok = True; break
            if ok: break
        RV_STAT['assigned' if ok else 'no_slot'] += 1
    return new


def rv_post_run():
    if RET <= 0: return 0, 0
    new_n = way_n = 0; _CAND_IDX[0] = None
    for u in sorted(U):
        if u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or not r['vias']: continue
        sv = [v for v in r['vias'] if not v.get('rv')]; rv = [v for v in r['vias'] if v.get('rv')]
        demand = [(v, rv_demand(net, v, r['tracks'])) for v in sv]; demand = [(v, g) for v, g in demand if g]
        if not demand and not rv: continue
        legacy_g = demand[0][1] if demand else rv[0]['net']
        foreign_nets={}
        for g in sorted({g for _,g in demand}|{v['net'] for v in rv}):
            source_g=g if PLANE_LAYER else legacy_g
            foreign_nets[g]=[(v['x'],v['y']) for w,r2 in U.items() if w!=u for v in r2['vias'] if (v.get('net') or r2['net'])==source_g]
            foreign_nets[g]+=[(D['pads'][k]['x'],D['pads'][k]['y']) for k in by_net_g.get(source_g,()) if D['pads'][k].get('drill')]
        keep = []
        for w in rv:
            required = any((not PLANE_LAYER or w['net']==g) and math.hypot(v['x'] - w['x'], v['y'] - w['y']) <= R and not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in foreign_nets[g])
                         for v, g in demand)
            if required: keep.append(w)
            else: way_n += 1
        open_items = [v for v,g in demand if not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a,b in foreign_nets[g]+[(w['x'],w['y']) for w in keep if not PLANE_LAYER or w['net']==g])]
        if len(keep) != len(rv):
            del_unit(u); add_unit(u, net, r['tracks'], sv + keep); r = U[u]; cand_idx_changed(u)
        if open_items:
            pads = [D['pads'][k] for k in by_net[net]]
            bx = (min(p['bbox'][0] for p in pads) - 3.0, min(p['bbox'][1] for p in pads) - 3.0, max(p['bbox'][2] for p in pads) + 3.0, max(p['bbox'][3] for p in pads) + 3.0)
            bx = (min([bx[0]] + [v['x'] - 1 for v in open_items]), min([bx[1]] + [v['y'] - 1 for v in open_items]), max([bx[2]] + [v['x'] + 1 for v in open_items]), max([bx[3]] + [v['y'] + 1 for v in open_items]))
            import copy as _cp
            save = _cp.deepcopy(U[u]); del_unit(u)
            RK = rv_map(net, bx, 0.0)
            extra = rv_set(net, save['tracks'], open_items, RK, owned=save['vias']) if RK else []
            add_unit(u, net, save['tracks'], save['vias'] + extra)
            if extra and not unit_conflict_free(u):
                del_unit(u); add_unit(u, net, save['tracks'], save['vias']); extra = []
            cand_idx_changed(u); new_n += len(extra)
    RV_STAT['post_run_new'] += new_n; RV_STAT['post_run_way'] += way_n; _CAND_IDX[0] = None
    return new_n, way_n








VIA_ANCHOR = float(args.get('via_anchor', '0')) if NC is not None else 0.0
VIA_PREV = {}
VIA_ANCHOR_ON = [False]
VIA_PRECEDENCE = args.get('via_precedence', '0') == '1' and NC is not None
PAIRS_RAW = []



K12 = float(args.get('k12', '0')) if (NC is not None and AREA_NETS) else 0.0
K12_DIST = float(args.get('k12_spacing', '%g' % (VIA_D + 2 * 0.15 + 0.15)))


def via_surcharge(u, net, C, box):
    k = (NC['net_list'].get(net) or {}) if NC else {}
    anchor = VIA_ANCHOR > 0 and VIA_ANCHOR_ON[0] and k.get('category') in ('current_val', 'supply', 'sw') and VIA_PREV.get(u)
    k12 = K12 > 0 and DENSE and net not in AREA_NETS
    if not (anchor or k12): return None
    V = None
    ix0, ix1, iy0, iy1 = _box_idx(box); ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    if anchor:
        V = np.full((NY, NX), np.float32(VIA_ANCHOR), dtype=np.float32)
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        for x, y in VIA_PREV[u]:
            jx, jy = int((x - X0) / RES) - ix0, int((y - Y0) / RES) - iy0
            if 0 <= jx < ix1 - ix0 and 0 <= jy < iy1 - iy0: m[jy, jx] = True
        if m.any():
            d = _edt(~m) * RES
            V[ys, xs] = (VIA_ANCHOR * np.clip((d - 1.0) / 4.0, 0.0, 1.0)).astype(np.float32)
    if k12:
        vs = [(v['x'], v['y']) for w, r in U.items() for v in r['vias'] if (v.get('net') or r['net']) not in AREA_NETS]
        for lg, dx0, dx1, dy0, dy1, _nz in DENSE:
            ax0, ax1, ay0, ay1 = dx0, dx1, dy0, dy1
            if ax1 <= ax0 or ay1 <= ay0: continue
            pz = int(K12_DIST / RES) + 2
            m2 = np.zeros((ay1 - ay0 + 2 * pz, ax1 - ax0 + 2 * pz), dtype=bool)
            for x, y in vs:
                jx, jy = int((x - X0) / RES) - ax0 + pz, int((y - Y0) / RES) - ay0 + pz
                if 0 <= jx < m2.shape[1] and 0 <= jy < m2.shape[0]: m2[jy, jx] = True
            if not m2.any(): continue
            d = (_edt(~m2) * RES)[pz:-pz, pz:-pz]
            if V is None: V = np.zeros((NY, NX), dtype=np.float32)
            V[ay0:ay1, ax0:ax1] += np.where(d < K12_DIST, np.float32(K12 * C['via'] * VIA_FACTOR[0]), np.float32(0.0))
    return None if V is None else V.ravel()


def precedence_filter(conf):
    def level1(w): return not w.startswith(('GND#', 'FIX#', 'TACK#')) and w in U and (NC['net_list'].get(U[w]['net']) or {}).get('category') in ('signal', 'analog')
    new = {}
    def rating(w, x, y, li):
        if w == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES); lst = new.setdefault(w, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    n = 0
    for li, x, y, w1, w2 in PAIRS_RAW:
        for a, b in ((w1, w2), (w2, w1)):
            if a.startswith('GND#') and level1(b) and li >= 0: break
        else: a = None
        lis = range(NL) if li < 0 else (li,)
        for l_ in lis:
            if a is None: rating(w1, x, y, l_); rating(w2, x, y, l_)
            else: rating(b, x, y, l_); n += 1
    return new, n


def rv_report():
    if RET <= 0: return None
    n = ok = owned = 0; missing = []
    for u in sorted(U):
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        for v in r['vias']:
            if v.get('rv'): continue
            g = rv_demand(net, v, r['tracks'])
            if not g: continue
            n += 1; gv = _rv_gnd(g)
            d = float(np.min(np.hypot(gv[:, 0] - v['x'], gv[:, 1] - v['y']))) if len(gv) else 99.0
            if d <= R + 1e-6:
                ok += 1; owned += any(w.get('rv') and math.hypot(w['x'] - v['x'], w['y'] - v['y']) <= R + 1e-6 for w in r['vias'])
            else: missing.append(dict(net_name=net, x=v['x'], y=v['y'], spacing=round(d, 2)))
    return dict(change=n, with_ret=ok, own_ret_via=owned, without=missing, stat=dict(RV_STAT))







ESCAPE = args.get('escape', '1' if NC else '0') == '1'
_FL = {}


def _pz_ports(net,p,w,capH,rng):
    if p.get('drill') or max(p['bbox'][2]-p['bbox'][0],p['bbox'][3]-p['bbox'][1])>1.2:return ()
    out={};a=(p['x'],p['y'])
    for L in p['layers']:
        if L not in LI:continue
        li=LI[L]
        own=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        if not own.covers(_PT(a)):continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            seen=set()
            for step in range(1,33):
                d=step*.025
                ix=int((a[0]+dx*d-X0)/RES);iy=int((a[1]+dy*d-Y0)/RES)
                if (ix,iy) in seen:continue
                seen.add((ix,iy))
                if not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
                if capH[li][iy,ix]<w:continue
                b=cell_xy(ix,iy)
                if own.covers(_PT(b)):continue
                paths=[[a,b]] if oct_(a,b) else [[a,c,b] for c in _corners(a,b)]
                good=[]
                for pts in paths:
                    pts=[_pz_snap(p) for p in pts]
                    if all(pad_room(li,x,y,net)>=w/2 and (not HARD_FOREIGN[0] or _pz_room(net,li,x,y,False)>=w/2) for x,y in zip(pts[:-1],pts[1:])):
                        good.append(pts)
                if good:
                    pts=min(good,key=lambda ps:sum(math.dist(x,y) for x,y in zip(ps[:-1],ps[1:])))
                    c=li*NN+iy*NX+ix
                    if c not in out:out[c]=(c,L,[list(pt) for pt in pts])
                    break
    return tuple(out[c] for c in sorted(out))


def escape(net, p, wk, capH, rng):
    if _pz_active(net) and id(p) in PZ_LOCAL[net]:return _pz_ports(net,p,wk,capH,rng)
    if not ESCAPE or p.get('drill'): return ()
    key = (id(p), wk)
    if key in _FL: return _FL[key]
    lag = [x for x in p['layers'] if x in LI]
    bx = p['bbox']; bw, bh = bx[2] - bx[0], bx[3] - bx[1]
    if len(lag) != 1 or min(bw, bh) > 0.45 or max(bw, bh) < 1.3 * min(bw, bh): _FL[key] = (); return ()
    L = lag[0]; li = LI[L]; cap = capH[li]; cx, cy = (bx[0] + bx[2]) / 2, (bx[1] + bx[3]) / 2
    out = []; remember = True
    for sgn in (-1.0, 1.0):
        if bw < bh: ex, ey, dx, dy = cx, (bx[3] if sgn > 0 else bx[1]), 0.0, sgn
        else: ex, ey, dx, dy = (bx[2] if sgn > 0 else bx[0]), cy, sgn, 0.0
        grid_cells = []; open_items = False
        for q in range(1, 13):
            ix, iy = int(math.floor((ex + dx * q * RES - X0) / RES)), int(math.floor((ey + dy * q * RES - Y0) / RES))


            if not (0 <= ix < NX and 0 <= iy < NY): break
            if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): open_items = True; break
            grid_cells.append((q * RES, ix, iy, bool(cap[iy, ix] >= wk)))
        if len(grid_cells) >= 2 and grid_cells[0][3] and grid_cells[1][3]: continue
        if open_items and len(grid_cells) < 2: remember = False; continue
        for d, ix, iy, ok in grid_cells:
            if not ok: continue
            zx, zy = cell_xy(ix, iy); da = max(0.0, d - 0.15); A = (ex + dx * da, ey + dy * da)
            if pad_room(li, (cx, cy), A, net) >= wk / 2 and pad_room(li, A, (zx, zy), net) >= wk / 2:
                out.append((li * NN + iy * NX + ix, L, [[round(cx, 4), round(cy, 4)], [round(A[0], 4), round(A[1], 4)], [round(zx, 4), round(zy, 4)]]))
            break
        else:
            if open_items: remember = False
    if remember: _FL[key] = tuple(out)
    return tuple(out)

def bga_own_hole_map(viaH, vias):
    if not vias: return viaH
    out = viaH.copy()
    for x, y in sorted({(float(v['x']), float(v['y'])) for v in vias}):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rad = int(math.ceil(HOLE_C / RES)) + 2
        x1, x2 = max(0, ix - rad), min(NX, ix + rad + 1)
        y1, y2 = max(0, iy - rad), min(NY, iy + rad + 1)
        if x1 >= x2 or y1 >= y2: continue
        yy, xx = np.mgrid[y1:y2, x1:x2]
        px = X0 + (xx + .5) * RES; py = Y0 + (yy + .5) * RES
        for vy, vx in zip(*np.nonzero(out[y1:y2, x1:x2])):
            c = (vy + y1) * NX + vx + x1
            if c in VP: px[vy, vx], py[vy, vx] = VP[c][:2]
        d2 = (px - x) ** 2 + (py - y) ** 2
        out[y1:y2, x1:x2] &= (d2 >= HOLE_C ** 2) | (d2 <= 1e-12)
    return out


def bga_escape(net, p, wk, capH, viaH, vbadf, rng, surface=False, soft=None, inner_w=None):
    b = BGA.get(id(p))
    ring_min = int(args.get('bga_ring_min', '2' if BGA_ESCAPE else '0')) if BGA_PLAN else 2
    if b is None: return None
    if BGA_ESCAPE and b['ring'] == 1 and 'bga_ring_min' not in args:


        bb = p['bbox']
        if b['pitch'] - max(bb[2]-bb[0],bb[3]-bb[1]) - 2*DRCP < wk - 1e-9:
            ring_min = 1


    supply_escape = BGA_ESCAPE and p.get('pintype') == 'power_in'
    if b['ring'] < ring_min and not surface and not supply_escape: return None
    inner = [i for i, L in enumerate(LAYERS) if L.startswith('In')]
    if not inner and not surface: return None
    if inner:
        q = max(0, b['ring'] - 2) % len(inner)
        inner = inner[q:] + inner[:q]
    candidates = b['pts']
    _vd = _vd_cost().ravel() if VIA_DENSITY else None
    if BGA_PLAN:


        def price(c):
            iy, ix = divmod(c, NX)



            via_narrow = (sum(max(0.0, VIA_D - float(soft[il][iy, ix])) / VIA_D for il in range(NL))
                        if BGA_ESCAPE and soft is not None else 0.0)
            return ((float(vbadf[c]) + via_narrow) * PF[0] + sum(float(hist[il * NN + c]) for il in range(NL)) +
                    (VD_MM * float(_vd[c]) if _vd is not None else 0.0),
                    math.dist((p['x'], p['y']), VP[c][:2]), c)
        candidates = sorted((c for c in candidates if rng[0] <= c % NX < rng[1] and rng[2] <= c // NX < rng[3]), key=price)
    if VIA_DENSITY and not BGA_PLAN and _vd.any():
        candidates = sorted(candidates, key=lambda c: (VD_MM * float(_vd[c]) + float(vbadf[c]) * PF[0],
                                                       math.dist((p['x'], p['y']), VP[c][:2]), c))
    best = None; best_key = None
    def sample_price(tracks):



        cells = {}
        for tr in tracks:
            il = LI[tr['layer']]
            for a_, b_, width in zip(tr['pts'][:-1], tr['pts'][1:], tr['w']):
                count = max(2, int(math.ceil(math.dist(a_, b_) / RES)) + 1)
                for x_, y_ in np.linspace(a_, b_, count):
                    ix_, iy_ = int((x_ - X0) / RES), int((y_ - Y0) / RES)
                    if not (0 <= ix_ < NX and 0 <= iy_ < len(capH[il])): return math.inf
                    key = (il, iy_ * NX + ix_)
                    cells[key] = max(cells.get(key, 0.0), width)
        cost = 0.0
        for (il, cell), width in sorted(cells.items()):
            iy_, ix_ = divmod(cell, NX)
            if soft is not None:
                cost += PF[0] * max(0.0, width - float(soft[il][iy_, ix_])) / max(width, 1e-6)
            cost += float(hist[il * NN + cell])
        return cost

    for c in candidates:
        iy, ix = divmod(c, NX)
        if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): continue
        if not viaH[iy, ix] or (vbadf[c] and not BGA_PLAN) or not _vp_ok(net, VP[c][2]): continue
        x, y = VP[c][:2]; a = (p['x'], p['y']); v = (x, y)
        if net in PLANE_AREA and not PLANE_AREA[net].covers(_PT(x,y)): continue
        pts = _oct_move([a, v]) if oct_net(net) else [a, v]
        pts = [q for i, q in enumerate(pts) if i == 0 or math.dist(q, pts[i - 1]) > 1e-7]
        li = LI[b['layer']]
        if any(pad_room(li, a_, b_, net) < wk / 2 for a_, b_ in zip(pts[:-1], pts[1:])): continue
        trs = [dict(layer=b['layer'], pts=[list(q) for q in pts], w=[wk]*(len(pts)-1))] if len(pts) > 1 else []
        vs = [dict(x=x, y=y)]
        if surface:
            result = (None, trs, vs)
            if not BGA_ESCAPE: return result
            score = (price(c)[0] if BGA_PLAN else 0.0) + sample_price(trs)
            key = (score, math.dist(a, v), c)
            if math.isfinite(score) and (best_key is None or key < best_key): best_key, best = key, result
            continue
        z = cell_xy(ix, iy)
        order = inner
        if BGA_PLAN and soft is not None:
            order = sorted(inner, key=lambda il: (PF[0] * max(0.0, wk - float(soft[il][iy, ix])) / max(wk, 1e-6) + float(hist[il * NN + c]), inner.index(il)))
        for il in order:




            iw = wk if inner_w is None else max(wk, inner_w)
            if capH[il][iy, ix] < iw: continue
            ip = _oct_move([v, z]) if oct_net(net) else [v, z]
            ip = [q for i, q in enumerate(ip) if i == 0 or math.dist(q, ip[i - 1]) > 1e-7]
            if any(pad_room(il, a_, b_, net) < iw/2 for a_, b_ in zip(ip[:-1], ip[1:])): continue
            it = [dict(layer=LAYERS[il], pts=[list(q) for q in ip], w=[iw]*(len(ip)-1))] if len(ip) > 1 else []
            result = (il*NN+c, trs+it, vs)
            if not BGA_ESCAPE: return result
            score = (price(c)[0] if BGA_PLAN else 0.0) + sample_price(trs + it)
            key = (score, math.dist(a, v), c, inner.index(il))
            if math.isfinite(score) and (best_key is None or key < best_key): best_key, best = key, result
    return best














CHAIN = NC is not None and args.get('link_chain', '1' if NLAY >= 4 else '0') == '1'
CHAIN_VIA = float(args.get('chain_via', '10'))
CHAIN_DIST = float(args.get('chain_spacing', '0.15'))
CHAIN_MAX = float(args.get('chain_max', '6.0'))
CHAIN_DETOUR = float(args.get('chain_detour', '2.0')); CHAIN_DETOUR_MM = float(args.get('chain_detour_mm', '3.0'))
CHAIN_PIN = float(args.get('chain_pin', '4.0'))
CHAINS = {}
KF_AFTER = {}
KF_BEFORE = {}
_METRIC_OFF = set()
CHAIN_STAT = dict(element=0, without_zone=0, back=0, detour=0, rest_free=0)
CHAIN_REST = CHAIN and args.get('chain_rest', '1') == '1'
CHAIN_REST_ANT = float(args.get('chain_rest_share', '0.2')); CHAIN_REST_MM = float(args.get('chain_rest_mm', '5.0'))


def _clearance_min(src, tgt):
    s = np.unique(np.asarray(src, dtype=np.int64) % NN); t = np.unique(np.fromiter(tgt, dtype=np.int64, count=len(tgt)) % NN)
    sx, sy = (s % NX) * RES, (s // NX) * RES; tx, ty = (t % NX) * RES, (t // NX) * RES
    best = np.inf
    for i in range(0, len(s), 512):
        d2 = (sx[i:i + 512, None] - tx[None, :]) ** 2 + (sy[i:i + 512, None] - ty[None, :]) ** 2
        best = min(best, float(d2.min()))
    return math.sqrt(best) * (1.4142 / math.sqrt(2.0))


def _plen(res):
    L = 0.0
    for a, b in zip(res[:-1], res[1:]):
        if a // NN != b // NN: L += 0.6; continue
        ia, ib = a % NN, b % NN
        L += RES * (1.4142 if (ia % NX != ib % NX and ia // NX != ib // NX) else 1.0)
    return L


def _pad_dist(p, q):
    a, b = p['bbox'], q['bbox']
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0.0), max(a[1] - b[3], b[1] - a[3], 0.0))


def _chains_build():
    per = {}
    for r in sorted(NC.get('part_list', {})):
        t = NC['part_list'][r]; st = t.get('supports') or {}
        if not str(t.get('function', '')).startswith('decoupling_') or st.get('over_coil') or not t.get('C'): continue
        n, chip, pin = st.get('net_name'), st.get('ref'), st.get('pin')
        if not n or chip is None or pin is None or n in GNDS or n not in by_net_g: continue
        ck = [k for k in by_net_g[n] if D['pads'][k]['ref'] == r]
        if len(ck) != 1 or not any(L in LI for L in D['pads'][ck[0]]['layers']): continue


        group = tuple(sorted(str(x) for x in (st.get('pins') or [pin]))) if st.get('p3_group') else ()
        if group:


            instances = {}
            for k in by_net_g[n]:
                p = D['pads'][k]
                if p['ref'] == chip and str(p['num']) in group:
                    instances.setdefault(str(p['num']), []).append(k)
            candidates = [ks[0] for ks in instances.values() if len(ks) == 1
                          and any(L in LI for L in D['pads'][ks[0]]['layers'])]
            if not candidates: continue
            pk = [min(candidates, key=lambda k: (_pad_dist(D['pads'][ck[0]], D['pads'][k]), k))]
        else:
            pk = [k for k in by_net_g[n] if D['pads'][k]['ref'] == chip and str(D['pads'][k]['num']) == str(pin)]
            if len(pk) != 1 or not any(L in LI for L in D['pads'][pk[0]]['layers']): continue
        if _pad_dist(D['pads'][ck[0]], D['pads'][pk[0]]) > CHAIN_MAX: continue
        g = per.setdefault((chip, n, group), dict(chain_list={}, pins=set()))
        if group:
            g.setdefault('anchor', []).append((float(t['C']), r, pk[0]))
            pk = [group]
        g['chain_list'].setdefault(pk[0], []).append((float(t['C']), r, ck[0]))
        g['pins'].update(str(x) for x in (st.get('pins') or [pin]))
    for (chip, n, group), g in sorted(per.items()):
        if group:
            anchor = min(g['anchor'])[2]
            if not any(L in LI for L in D['pads'][anchor]['layers']): continue
            ks = [dict(pins=[anchor], caps=[c for _, _, c in sorted(cs)]) for cs in g['chain_list'].values()]
        else:
            ks = [dict(pins=[pk], caps=[c for _, _, c in sorted(cs)]) for pk, cs in sorted(g['chain_list'].items())]
        occupied = {k for K_ in ks for k in K_['pins'] + K_['caps']}
        for k in by_net_g[n]:
            p = D['pads'][k]
            if p['ref'] != chip or k in occupied or str(p['num']) not in g['pins'] or not any(L in LI for L in p['layers']): continue
            if group and sum(D['pads'][j]['ref'] == chip and str(D['pads'][j]['num']) == str(p['num']) for j in by_net_g[n]) != 1: continue
            d, i = min((_pad_dist(p, D['pads'][K_['caps'][0]]), i) for i, K_ in enumerate(ks))
            if d <= CHAIN_PIN: ks[i]['pins'].append(k)
        CHAINS.setdefault(n, []).extend(ks)
    for n in sorted(CHAINS):
        if n not in AREA_NETS: continue
        for K_ in CHAINS[n]:
            for a, b in [(k, K_['caps'][0]) for k in K_['pins']] + list(zip(K_['caps'][:-1], K_['caps'][1:])):
                KF_AFTER[a] = b; KF_BEFORE.setdefault(b, []).append(a)
    if CHAINS: print('Supply chains (R22): %d chains in %d nets, of which %d nets with area' % (
        sum(len(v) for v in CHAINS.values()), len(CHAINS), sum(1 for n in CHAINS if n in AREA_NETS)), flush=True)

if CHAIN: _chains_build()

def _rectangle_cells(bx, r):
    ixa = max(0, int((bx[0] - r - X0) / RES)); ixb = min(NX, int((bx[2] + r - X0) / RES) + 1)
    iya = max(0, int((bx[1] - r - Y0) / RES)); iyb = min(NY, int((bx[3] + r - Y0) / RES) + 1)
    if ixb <= ixa or iyb <= iya: return np.zeros(0, dtype=np.int64)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    m = (np.maximum(bx[0] - GX, GX - bx[2]) <= r) & (np.maximum(bx[1] - GY, GY - bx[3]) <= r)
    yy, xx = np.nonzero(m)
    return (yy + iya) * NX + (xx + ixa)

def _metric(inner, traces, vias, vacant, wmin, dist=None):
    if dist is None: dist = CHAIN_DIST
    rb = wmin / 2 + dist; rv = VIA_R + dist; zb, zv = [], []
    for p in inner:
        for L in p['layers']:
            if L in LI: zb.append(LI[L] * NN + _rectangle_cells(p['bbox'], rb))
        zv.append(_rectangle_cells(p['bbox'], rv))
    for t in traces:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            zb.append(seg_cells(li, a, b, w + 2 * rb)); zv.append(seg_cells(0, a, b, w + 2 * rv))
    for v in vias:
        q = (v['x'], v['y'])
        for li in range(NL): zb.append(seg_cells(li, q, q, VIA_D + 2 * rb))
        zv.append(seg_cells(0, q, q, VIA_D + 2 * rv))
    zb = np.concatenate(zb) if zb else np.zeros(0, dtype=np.int64)
    zv = np.concatenate(zv) if zv else np.zeros(0, dtype=np.int64)
    for p in vacant:
        zb = zb[~np.isin(zb % NN, _rectangle_cells(p['bbox'], rb + 0.05))]
        zv = zv[~np.isin(zv, _rectangle_cells(p['bbox'], 0.0))]
    return zb, zv


def _metric_maps(capH, viaH, zb, zv):
    c2 = _Maps(); c2.planar = capH.planar.copy(); c2.planar[zb] = -1.0
    for li in range(NL): c2.append(c2.planar[li * NN:(li + 1) * NN].reshape(NY, NX))
    v2 = np.array(viaH, dtype=bool, copy=True); v2.reshape(-1)[zv] = False
    return c2, v2


def _metric_prepare(r, net):
    inner = set()
    for t in r['tracks']:
        if 'metric' in t: inner.update(t['metric'][1:])
    owned = {0: {k for k in by_net_g[net] if k not in inner}}
    for t in r['tracks']:
        if t.get('link_chain'): owned.setdefault(t['link_chain'], set()).update(t.get('kp') or ())
    geo = {k: {LI[L]: shapely.union_all([Polygon(q) for q in D['pads'][k]['polys'][L]]) for L in D['pads'][k]['layers'] if L in LI} for k in by_net_g[net]}
    G = {}
    for g in sorted(owned):
        own = owned[g]; e = {}; pf = {}; fr = {}
        for li in range(NL):
            ow = [geo[k][li] for k in sorted(own) if li in geo[k]]
            e[li] = shapely.union_all(ow) if ow else None
            pf[li] = e[li].buffer(CHAIN_DIST + 0.1) if ow else None
            fr[li] = [geo[k][li] for k in by_net_g[net] if k not in own and li in geo[k]]
        G[g] = dict(owned=e, buf_dist=pf, foreign=fr)
    xs = [q[0] for t in r['tracks'] if t.get('link_chain') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in inner for i in (0, 2)]
    ys = [q[1] for t in r['tracks'] if t.get('link_chain') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in inner for i in (1, 3)]
    box = (min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5) if xs else (0.0, 0.0, 0.0, 0.0)
    return dict(G=G, box=box)


def _metric_width(metric, r, width_list, ti, a, b, li):
    g = r['tracks'][ti].get('link_chain') or 0; bx = metric['box']
    if g == 0 and (max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]): return 9.0
    G = metric['G'][g]; seg = _LS([a, b]) if a != b else _PT(a)
    if G['owned'][li] is not None: seg = seg.difference(G['owned'][li])
    if seg.is_empty: return 9.0
    d = 9.0
    for q in G['foreign'][li]: d = min(d, seg.distance(q))
    pf = G['buf_dist'][li]
    for tj, t2 in enumerate(r['tracks']):
        if (t2.get('link_chain') or 0) == g or LI[t2['layer']] != li: continue
        for a2, b2, w2 in zip(t2['pts'][:-1], t2['pts'][1:], width_list[tj]):
            if min(a2[0], b2[0]) - 2.0 > max(a[0], b[0]) or max(a2[0], b2[0]) + 2.0 < min(a[0], b[0]) or \
               min(a2[1], b2[1]) - 2.0 > max(a[1], b[1]) or max(a2[1], b2[1]) + 2.0 < min(a[1], b[1]): continue
            s2 = _LS([a2, b2]) if a2 != b2 else _PT(a2)
            if pf is not None: s2 = s2.difference(pf)
            if not s2.is_empty: d = min(d, seg.distance(s2) - w2 / 2)
    for v in r['vias']:
        if (v.get('link_chain') or 0) == g: continue
        q = _PT(v['x'], v['y'])
        if pf is None or not pf.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - CHAIN_DIST)


def _kf_zone(k, p, wmin):
    before, st = [], [k]
    while st:
        for a in KF_BEFORE.get(st.pop(), ()): before.append(a); st.append(a)
    if not before: return None
    sp = [t for a in before if 'GND#%d' % a in U for t in U['GND#%d' % a]['tracks']]
    vv = [v for a in before if 'GND#%d' % a in U for v in U['GND#%d' % a]['vias']]
    return _metric([D['pads'][a] for a in before], sp, vv, [p], wmin)


def _kf_element(u, k, p, net, C, src, fst, wk, maps_):
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps_
    q = D['pads'][KF_AFTER[k]]; bx, qb = p['bbox'], q['bbox']
    Cb = dict(C, via=3.0 * CHAIN_VIA if set(p['layers']) & set(q['layers']) else 0.0)
    win = (min(bx[0], qb[0]) - 3.0, min(bx[1], qb[1]) - 3.0, max(bx[2], qb[2]) + 3.0, max(bx[3], qb[3]) + 3.0)
    if not _in_window(win, rng): return False
    tgt = set()
    for L, cells in q['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
    escape_target = {c: (l_, pts) for c, l_, pts in escape(net, q, wk, capH, rng)}; tgt.update(escape_target)
    src = sorted({c for c in src if c not in tgt})
    if not src or not tgt: return False
    res = None; zn = _kf_zone(k, p, Cb['wmin'])
    if zn is not None:
        cH, vH = _metric_maps(capH, viaH, *zn)
        res = astar_pf(src, tgt, qb, cH, capS, vH, viaBad, Cb, win)
        if not res: CHAIN_STAT['without_zone'] += 1
    if not res: res = astar_pf(src, tgt, qb, capH, capS, viaH, viaBad, Cb, win)
    if not res: return False
    trs, vs = path_to_geometry(res, capC, dict(Cb, w=Cb['wmin'])) if len(res) > 1 else ([], [])
    if sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > CHAIN_DETOUR * _pad_dist(p, q) + CHAIN_DETOUR_MM:
        CHAIN_STAT['detour'] += 1; return None
    for c_, f_ in ((res[0], fst), (res[-1], escape_target)):
        if c_ in f_: trs.append(dict(layer=f_[c_][0], pts=f_[c_][1], w=[wk, wk]))
    for t in trs: t['link_chain'] = True
    add_unit(u, net, trs, vs); CHAIN_STAT['element'] += 1
    return True


def n_sv(r):
    return sum(1 for v in r['vias'] if not v.get('rv') and not v.get('gr'))
def cons_missing(u):
    vs = U[u]['vias']
    return sum(max(0, v['n'] - 1 - sum(1 for w in vs if w.get('too') == [v['x'], v['y']])) for v in vs if v.get('n'))

















TAP_ON = TAPS and args.get('tap_spacing', '1' if NC else '0') == '1'
TAP_CLR = float(args.get('tap_clr', '%g' % CLR))
TAP_TREE = TAPS and args.get('tap_tree', '1' if NC else '0') == '1'
TAP_LAYER = float(args.get('tap_layer', '0')) if TAPS else 0.0


TAP_VIA = float(args.get('tap_via', '1')) if TAPS else 1.0
TAP_LAYER_D = float(args.get('tap_layer_d', '0.8'))
BRANCH = args.get('branch', '0') == '1' or R27_TRUNK
BRANCH_D = float(args.get('branch_d', '0.6')); BRANCH_L = float(args.get('branch_l', '1.0')); BRANCH_DEG = float(args.get('branch_deg', '60'))
BRANCH_FACTOR = float(args.get('branch_factor', '3.0')); BRANCH_MORE = float(args.get('branch_more', '1.5')); STAR_R = float(args.get('star_r', '0.3'))
TOPO_STAT = dict(taps=0, tap_without_zone=0, branch_checked=0, branch_shortage=0, branch_better=0, star_blocked=0)


def _arm_direction(pts, i, step, p, r=0.15):
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += step
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0


def _arms_at(traces, li, p):
    raw_data = []
    for t in traces:
        if LI[t['layer']] != li: continue
        P = t['pts']
        for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
            if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            f = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
            pf = (a[0] + f * dx, a[1] + f * dy)
            if math.dist(p, pf) > 0.036: continue
            raw_data += [_arm_direction(P, k + 1, 1, pf), _arm_direction(P, k, -1, pf)]
    arms = []
    for w in sorted(x for x in raw_data if x is not None):
        if not arms or w - arms[-1] > 3.0: arms.append(w)
    if len(arms) > 1 and arms[0] + 360.0 - arms[-1] <= 3.0: arms.pop()
    return arms


def _parallel_mm(trs, traces):
    ges = 0.0
    for t in trs:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.dist(a, b)
            if L < 1e-6: continue
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            for t2 in traces:
                if t2['layer'] != t['layer']: continue
                for c, d in zip(t2['pts'][:-1], t2['pts'][1:]):
                    L2 = math.dist(c, d)
                    if L2 < 1e-6 or abs(ux * (d[1] - c[1]) - uy * (d[0] - c[0])) / L2 > 0.2588: continue
                    sc = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy; sd = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
                    lo, hi = max(0.0, min(sc, sd)), min(L, max(sc, sd))
                    if hi - lo <= 1e-6 or abs(sd - sc) < 1e-9: continue
                    nc = -(c[0] - a[0]) * uy + (c[1] - a[1]) * ux; nd = -(d[0] - a[0]) * uy + (d[1] - a[1]) * ux
                    e0 = nc + (nd - nc) * (lo - sc) / (sd - sc); e1 = nc + (nd - nc) * (hi - sc) / (sd - sc)
                    if max(abs(e0), abs(e1)) < BRANCH_D and min(abs(e0), abs(e1)) > 0.02: ges += hi - lo
    return ges


def _branch_defects(trs, traces, li, p, at_pad):
    arms = [] if at_pad else _arms_at(traces, li, p)
    wn = None
    for t in trs:
        if LI[t['layer']] == li and math.dist(t['pts'][0], p) < 0.08: wn = _arm_direction(t['pts'], 1, 1, p); break
    wk = min((abs((wn - a + 180.0) % 360.0 - 180.0) for a in arms), default=180.0) if wn is not None else 180.0
    par = _parallel_mm(trs, traces)
    return (len(arms) >= 3, par if par > BRANCH_L else 0.0, wk < BRANCH_DEG - (1e-6 if R27_TRUNK else 1.0))


def _bands(ex, traces, wide, factor):
    m = np.zeros(NL * NN, dtype=bool)
    for t in traces:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): m[seg_cells(li, a, b, w + 2 * wide)] = True
    e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    e[m] *= np.float32(factor)
    return e


def _tap_prepare(r, net):
    lane = {li: [] for li in range(NL)}; ends = []
    for t in r['tracks']:
        if not t.get('tap'): continue
        ends += [t['pts'][0], t['pts'][-1]]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if a != b: lane[LI[t['layer']]].append((_LS([a, b]), w / 2))
    vv = [_PT(v['x'], v['y']) for v in r['vias'] if v.get('tap')]
    vacant = {}
    for li in range(NL):
        g = [g2 for n2, g2 in _PADG[li] if n2 == net and any(g2.distance(_PT(e)) <= 0.05 for e in ends)]
        vacant[li] = shapely.union_all(g).buffer(TAP_CLR + 0.1) if g else None
    xs = [e[0] for e in ends] + [q.x for q in vv]; ys = [e[1] for e in ends] + [q.y for q in vv]
    for li in range(NL):
        for s2, _ in lane[li]: b_ = s2.bounds; xs += [b_[0], b_[2]]; ys += [b_[1], b_[3]]
    return dict(lane=lane, vias=vv, vacant=vacant, box=(min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5))


def _tap_width(ab, a, b, li):
    bx = ab['box']
    if max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]: return 9.0
    seg = _LS([a, b]) if a != b else _PT(a); fr = ab['vacant'][li]
    if fr is not None: seg = seg.difference(fr)
    if seg.is_empty: return 9.0
    d = 9.0
    for s2, h2 in ab['lane'][li]:
        if fr is not None: s2 = s2.difference(fr)
        if not s2.is_empty: d = min(d, seg.distance(s2) - h2)
    for q in ab['vias']:
        if fr is None or not fr.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - TAP_CLR)


def route_unit(u):
    work_step('unit')
    ok = _route_unit_impl(u)
    net = U.get(u,{}).get('net',u)
    if not ok and PIN_ACCESS and not _pz_active(net) and net in PZ_WIDTHS and not u.startswith(('GND#','FIX#')):
        C=cls(net);capH,viaH=hard_maps(net)
        blocked=[]
        for k in by_net[net]:
            p=D['pads'][k]
            cells=[(LI[L],cs) for L,cs in p['cells'].items() if L in LI]
            free=any(np.any(capH[li].ravel()[cs]>=C['wmin']) for li,cs in cells)



            cut = '%s:%s' % (p['ref'],p['num']) in UNREACHABLE.get(net,[])
            if cut or (not free and not escape(net,p,C['wmin'],capH,_WHOLE)):blocked.append(id(p))
        if blocked:
            PZ_LOCAL[net]=set(blocked);_pad_room_cached.cache_clear()
            for key in list(_ERR):
                if key[0]==net:_ERR.pop(key)
            _ERR_NETS.difference_update(key for key in list(_ERR_NETS) if key[0]==net)
            print('Pin access local:',net,'cut-off pads',len(blocked),flush=True)
            del_unit(u)
            ok=_route_unit_impl(u)
    return ok


_PLANE_GROUP_ACTIVE = [None]


def route_plane_group(net):
    old=_PLANE_GROUP_ACTIVE[0];_PLANE_GROUP_ACTIVE[0]=net
    tr=[];vs=[];complete=True;del_unit(net)
    try:
        for k in PLANE_GROUPS[net]:
            child='GND#%d'%k
            try:
                ok=_route_unit_impl(child);r=U.get(child)
                if r:tr.extend(r['tracks']);vs.extend(r['vias'])
                complete &= bool(ok)
            finally:del_unit(child)
            del_unit(net);add_unit(net,net,tr,vs)
            vs=list(U[net]['vias'])
        return complete
    finally:_PLANE_GROUP_ACTIVE[0]=old


def _route_unit_impl(u):
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if pair_unit(u): return route_pair(u)
    if u in PLANE_GROUPS: return route_plane_group(u)
    ASTAR_COST[0] = None; ASTAR_HINT[0] = None; VIA_TOO[0] = None
    NET_CUR[0] =D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
    if u.startswith('GND#'):
        k = int(u[4:]); p = D['pads'][k]; net = p['net']
        _direct_plane = any(net in FL_ASSIGNMENT.get(L,[]) for L in PLANE_SIGNALS)
        if _direct_plane and (net not in PLANE_AREA or PLANE_AREA[net].is_empty):return False
        C = dict(w=max(0.35, TRACK_MIN), wmin=max(0.15, TRACK_MIN), pen=0.5, via=0.0, inner=99.0)
        bx = p['bbox']
        box = (bx[0] - 6.1, bx[1] - 6.1, bx[2] + 6.1, bx[3] + 6.1)
        capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps(net, box, C['w'])
        L = next(x for x in p['layers'] if x in LI)
        src = (LI[L] * NN + p['cells'][L]).tolist()
        _wk = min(C['wmin'], NECK_W) if neck_active(net, C, C['wmin'], True) else C['wmin']
        _fst = {c: (l_, pts) for c, l_, pts in escape(net, p, _wk, capH, rng)}
        src += [c for c in sorted(_fst) if c not in src]
        if CHAIN and not _direct_plane and k in KF_AFTER and (net, k) not in _METRIC_OFF:
            _cgfx = _kf_element(u, k, p, net, C, src, _fst, _wk, (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf))
            if _cgfx: return True
            if _cgfx is False: _METRIC_OFF.add((net, k))
            CHAIN_STAT['back'] += 1
        if BGA_FANOUT:
            _bf = bga_escape(net, p, _wk, capH, viaH, vbadf, rng, surface=True, soft=capS)
            if _bf: add_unit(u, net, _bf[1], _bf[2]); return True
        gnd_tgt = set()
        _plane_distance = PLANE_AREA[net].distance(_PT(p['x'],p['y'])) if _direct_plane and net in PLANE_AREA else 0.
        if _plane_distance > 6.:



            for owner,r in U.items():
                if _PLANE_GROUP_ACTIVE[0]==net and owner!=net:continue
                if r['net']!=net or not any(PLANE_AREA[net].covers(_PT(v['x'],v['y'])) for v in r['vias']):continue
                for t in r['tracks']:
                    for q in t['pts']:
                        ix,iy=int((q[0]-X0)/RES),int((q[1]-Y0)/RES)
                        if 0<=ix<NX and 0<=iy<NY:gnd_tgt.add(LI[t['layer']]*NN+iy*NX+ix)
        if k in EPM:
            _ev = ep_set(k, viaH, vbadf, rng)
            if _ev: add_unit(u, net, [], _ev); return True
        _epr = EP_RES.get(net) if k not in EPM else None
        if _epr is not None: viaH = viaH & ~_epr
        _vz = None; _via_cost = 0.0
        if VAP and not _direct_plane:
            _vzc, _vzw = _vap_cells(k); _VZ[_vzc] = _vzw; _vz = _VZ
            _via_cost = VAP_K * (VAP_WF if k in VAP_IMPORTANT else 1.0)
        if VIA_DENSITY:
            _dv = _vd_cost().ravel() * np.float32(VD_MM)
            if _dv.any():


                _vz = _dv if _vz is None else _via_cost * _vz.astype(np.float64) + _dv
                _via_cost = 1.0

        _zs = _kf_zone(k, p, C['wmin']) if CHAIN and not _direct_plane and k in KF_BEFORE and (net, k, 2) not in _METRIC_OFF else None
        res=None
        for _zn in ([_zs] if _zs is not None else []) + [None]:
            _windows = (1.8,3.5,6.0) if _plane_distance <= 6. else (max(6.,_plane_distance+2.),)
            for ml in _windows:
                win = (bx[0] - ml, bx[1] - ml, bx[2] + ml, bx[3] + ml)
                if not _in_window(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = maps_larger(net, box, win, C['w'])
                    if _b2 is not None: box = _b2
                    if _epr is not None: viaH = viaH & ~_epr
                _cH, _vH = (capH, viaH) if _zn is None else _metric_maps(capH, viaH, *_zn)
                if _direct_plane and _plane_distance<=6.:
                    _vH=_vH & PLANE_MASK[net]
                targets=gnd_tgt;target_box=(p['x'],p['y'],p['x'],p['y']);via_target=True
                if _plane_distance>6.:
                    targets=gnd_tgt | plane_feeder_targets(net,p,LI[L],_vH,viaBad,rng)
                    if not targets:continue
                    xy=[cell_xy((c%NN)%NX,(c%NN)//NX) for c in sorted(targets)]
                    target_box=(min(q[0] for q in xy),min(q[1] for q in xy),max(q[0] for q in xy),max(q[1] for q in xy))
                    via_target=False
                res = astar_pf(src, targets, target_box, _cH, capS, _vH, viaBad, C, win, via_target=via_target, viaz=_vz, vzk=_via_cost)
                if res:
                    conf = any(capCf[q] < C['wmin'] for q in res) or (res[-1] not in gnd_tgt and bool(vbadf[res[-1] % NN]))
                    if not conf or ml >= 6.0: break
                    if NB is not None and NB.info[0] == 0: break
            if res or _zn is None: break
            _METRIC_OFF.add((net, k, 2)); CHAIN_STAT['without_zone'] += 1
        if VAP and not _direct_plane and _vz is not None: _VZ[_vzc] = 1.0
        if not res: return False
        trs, _mid_vias = path_to_geometry(res, capC, dict(C, w=C['wmin'])) if len(res) > 1 else ([], [])
        if res[0] in _fst: trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        n = res[-1]; vx, vy = cell_xy((n % NN) % NX, (n % NN) // NX)
        _s = _vp_xy(net, n % NN) if VP and n not in gnd_tgt else None
        if _s and _direct_plane and not PLANE_AREA[net].covers(_PT(*_s)):_s=None
        if _s:
            vx, vy = _s
            for t in trs:
                if LI[t['layer']] == n // NN and math.hypot(t['pts'][-1][0] - vx, t['pts'][-1][1] - vy) < 0.05: t['pts'][-1] = [round(vx, 4), round(vy, 4)]
        add_unit(u, net, trs, _mid_vias+([] if n in gnd_tgt else [dict(x=round(vx, 4), y=round(vy, 4))]))
        return True
    net = u; C = cls(net)
    pads = [D['pads'][k] for k in by_net[net]]
    pbx = (min(p['bbox'][0] for p in pads), min(p['bbox'][1] for p in pads), max(p['bbox'][2] for p in pads), max(p['bbox'][3] for p in pads))
    box = (pbx[0] - 8.5, pbx[1] - 8.5, pbx[2] + 8.5, pbx[3] + 8.5)
    _m0 = WINDOW if WINDOW else 3.0
    ASTAR_HINT[0] = (max(1, int((pbx[1] - _m0 - Y0) / RES)), min(NY - 2, int((pbx[3] + _m0 - Y0) / RES)),
                     max(1, int((pbx[0] - _m0 - X0) / RES)), min(NX - 2, int((pbx[2] + _m0 - X0) / RES)))
    Cs = cls('/__signal__'); _wmax = max(C['w'], Cs['w'])
    _sz_fl = current_model(net)
    if _sz_fl: _wmax = max(_wmax, sum(map(float,_sz_fl['load'].values())) / (CURRENT_J * min(CURRENT_T.values())))
    RK = rv_map(net, box, C['via'] * VIA_FACTOR[0]) if RET > 0 else None
    _via_anchor = via_surcharge(u, net, C, box) if (VIA_ANCHOR > 0 or K12 > 0) else None
    _via_anchor_prev = _via_anchor
    if VIA_DENSITY:
        _dv = _vd_cost().ravel() * np.float32(VD_MM)
        if _dv.any(): _via_anchor = _dv if _via_anchor is None else _via_anchor + _dv
    if RK is not None or _via_anchor is not None: VIA_TOO[0] = (RK['via_extra'] if RK else None, RV_PAIR if RK else None, _via_anchor)
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps(net, box, _wmax)

    k = net.split('/')[-1]; taps = []; kel = {}
    if TAP_NC:
        _ak = {'%s:%s' % (p['ref'], p['num']): i for i, p in enumerate(pads)}
        _ae = [e for e in NC['taps'].get(net, []) if e['pad'] in _ak]
        taps = sorted(_ak[e['pad']] for e in _ae)
        for e in _ae:
            if e.get('start') in _ak: kel[_ak[e['pad']]] = [_ak[e['start']]]

    power = [i for i in range(len(pads)) if i not in taps]
    EXTRA = disturb_extra(net)
    if VIA_DENSITY and NC and (NC['net_list'].get(net) or {}).get('category') in ('current_val', 'supply') and VD_CURRENT > 0:


        _dc = np.square(np.maximum(np.float32(0), _vd_window_max(_VD_COUNT.astype(np.float32)) - np.float32(VD_THRESHOLD * VD_WINDOW ** 2)))
        if _dc.any():
            _de = np.tile(1.0 + np.float32(VD_CURRENT) * _dc.ravel(), NL)
            EXTRA = _de if EXTRA is None else EXTRA * _de
    if net in PAIR_NETS: EXTRA = pair_extra(net, C, capS, EXTRA)
    _tap_sp, _tap_vi, _tap_kn, _tap_pads = [], [], [], []
    _star = set(); _pn_set = set()

    KT = None; _inner = set(); trunk = power
    if CHAIN and net in CHAINS and not taps:
        _local = {k_: i for i, k_ in enumerate(by_net[net])}
        KT = [dict(nr=_n, pins=[_local[k_] for k_ in K_['pins']], caps=[_local[k_] for k_ in K_['caps']], pin_ok=[], end=-1, traces=[], vias=[], kn=[], stop=False)
              for _n, K_ in enumerate(CHAINS[net])]
        _inner = {i for K_ in KT for i in K_['pins'] + K_['caps'][:-1]}
        trunk = [i for i in power if i not in _inner]
    BF = {}; BF_NODES = {}; _bf_done = set()
    _bga_own_holes = BGA_PLAN and (args.get('bga_own_via', '0') == '1' or BGA_ESCAPE)
    if BGA_FANOUT and (KT is None or BGA_PLAN) and not taps:
        _bf_viaH = viaH
        _bw = min(C['wmin'], NECK_W) if neck_active(net, C, C['wmin']) else C['wmin']
        for i in sorted(power, key=lambda i: (BGA.get(id(pads[i]), {}).get('ring', -1), pads[i]['ref'], pads[i]['num'])):
            _bf = bga_escape(net, pads[i], _bw, capH, _bf_viaH, vbadf, rng, soft=capS,
                             inner_w=C['wmin'] if args.get('bga_inner_width', '0') == '1' or BGA_ESCAPE else None)
            if _bf:
                BF[i] = _bf; _old = BF_NODES.setdefault(_bf[0], ([], []))
                _old[0].extend(_bf[1]); _old[1].extend(_bf[2])


                if _bga_own_holes: _bf_viaH = bga_own_hole_map(_bf_viaH, _bf[2])
    _bga_reserved_vias = [v for bf in BF.values() for v in bf[2]] if _bga_own_holes else []
    if _sz_fl and not trunk:
        CURRENT_AUDIT[net]=dict(declined='current_first: source/load only as taps, no current trunk ' + net)
        add_unit(u,net,[],[]);return False
    cx = sum(pads[i]['x'] for i in trunk) / len(trunk); cy = sum(pads[i]['y'] for i in trunk) / len(trunk)
    first = min(trunk, key=lambda i: math.hypot(pads[i]['x'] - cx, pads[i]['y'] - cy))
    FL = NC['net_list'].get(net, {}).get('flow') if NC else None
    if FL and FL.get('source') and args.get('flow_start') == '1':
        qs = [i for i in power if '%s:%s' % (pads[i]['ref'], pads[i]['num']) in FL['source']]
        if qs: first = qs[0]
        if first in _inner: first = next(K_['caps'][-1] for K_ in KT if first in K_['pins'] + K_['caps'])
    try: SP = current_plan(net, pads, trunk, KT)
    except ValueError as e:


        CURRENT_AUDIT[net]=dict(declined=str(e))
        add_unit(u,net,[],[]);return False
    if SP: first = SP[0]
    tree = [first]; done_xy = [(pads[first]['x'], pads[first]['y'])]
    rest = [i for i in trunk if i != first]
    all_tr, all_v, pathnodes, ok = [], [], [], True
    if globals().get('PZ_REST') and HARD_FOREIGN[0] and globals().get('_LOOP_DEADLINE') is not None:
        _PZ_PARTIAL[0] = (u, net, all_tr, all_v)
    _stubs = set()
    order = []
    while rest:
        j = min(rest, key=lambda i: min(math.hypot(pads[i]['x'] - x, pads[i]['y'] - y) for x, y in done_xy))
        rest.remove(j); order.append(j); done_xy.append((pads[j]['x'], pads[j]['y']))
    if SP: order = SP[1]
    order += taps
    done_xy = [(pads[first]['x'], pads[first]['y'])]
    jobs = [(j, None) for j in order]
    _kzz = None; _kzr = None; _metric_k = None; tree_cons = tree
    if KT is not None:
        _gl = []
        for K_ in KT:
            _gl.append((K_['caps'][0], (K_, 0, [K_['pins'][0]])))
            _gl += [(q_, (K_, 0, None)) for q_ in K_['pins'][1:]]
            _gl += [(b_, (K_, s_ + 1, [a_])) for s_, (a_, b_) in enumerate(zip(K_['caps'][:-1], K_['caps'][1:]))]
        jobs = _gl + jobs; tree_cons = list(tree)
    _hint0 = ASTAR_HINT[0]
    for j, gl in jobs:
        if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
        if VIA_DENSITY and all_v:
            _dv = _vd_cost(all_v).ravel() * np.float32(VD_MM)
            _via_anchor = _dv if _via_anchor_prev is None else _via_anchor_prev + _dv
            VIA_TOO[0] = (RK['via_extra'] if RK else None, RV_PAIR if RK else None, _via_anchor)
        p = pads[j]; is_tap = j in taps; Cb = (dict(Cs, via=Cs['via'] * TAP_VIA) if TAP_VIA != 1.0 else Cs) if is_tap else C
        _sz_parent = SP[2].get(j, first) if SP else None
        if SP and not is_tap:
            _sz_key = ('trunk',j) if gl is None else ('link_chain',gl[0]['nr'],gl[1],j)
            _sz_I = SP[3].get(_sz_key, sum(map(float,_sz_fl['load'].values())))
            Cb = current_net_class(Cb,_sz_I,pads[_sz_parent] if gl is None else pads[(gl[2] or gl[0]['pins'])[0]],p)
            Cb['current_vias'] = tuple((v['x'],v['y']) for v in all_v)
        ASTAR_HINT[0] = _hint0 if gl is None else None
        _zone = None
        _tap_z = False; _ex = EXTRA
        if gl is not None:
            K_, _level, _qp = gl
            if K_['stop'] or (_level > 0 and K_['end'] != _level - 1): continue
            if _qp is None: _qp = K_['pin_ok'] + K_['caps'][:1]
            src = list(K_['kn']) if gl[2] is None else []
            for i in _qp:
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if any(set(pads[i]['layers']) & set(p['layers']) for i in _qp) and not (BGA_PLAN and (j in BF or any(i in BF for i in _qp))):
                Cb = dict(Cb if SP else C, via=Cb['via'] * CHAIN_VIA)
            if (net, by_net[net][j]) not in _METRIC_OFF:
                _mine = (set(K_['pins']) | {K_['caps'][0]}) if _level == 0 else (set(_qp) | {j})
                _zone = _metric([pads[i] for i in power if i not in _mine], K_['traces'] if _level > 0 else [], K_['vias'] if _level > 0 else [],
                            [pads[i] for i in sorted(_mine)], Cb['wmin'])
        elif j in kel:
            src = list(_tap_kn) if TAP_TREE else []
            for i in kel[j] + (_tap_pads if TAP_TREE else []):
                for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            _current = [t for t in all_tr if not t.get('tap')]
            if TAP_ON:
                _cgfx = set(kel[j]) | {t_ for t_ in kel if kel[t_] == kel[j]}
                _zone = _metric([pads[i] for i in range(len(pads)) if i not in _cgfx], _current, [v for v in all_v if not v.get('tap')],
                            [pads[i] for i in sorted(_cgfx)], Cb['wmin'], TAP_CLR + QM)
                _tap_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
            if TAP_LAYER > 0 and _current:
                _ex = _bands(EXTRA, _current, TAP_LAYER_D, 1.0 + 2.0 * Cb['via'] * VIA_FACTOR[0] / TAP_LAYER)
        else:
            _si_chain = SI_STUB and actual_fast(net) and gl is None and not is_tap
            _sz_sources = [_sz_parent] if SP and not is_tap and _sz_parent in tree else ([] if SP and not is_tap else None)
            src = [] if _si_chain or _sz_sources is not None else ([q for q in pathnodes if q not in _star] if _star else list(pathnodes))
            for i in (_sz_sources if _sz_sources is not None else (tree[-1:] if _si_chain else tree)):
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if KT is not None and not is_tap:
                if _kzz is None:
                    _zi = [pads[i] for K_ in KT for i in K_['pin_ok'] + K_['caps'][:max(K_['end'], 0)]]
                    _zf = [pads[K_['caps'][K_['end']]] for K_ in KT if K_['end'] >= 0]
                    _kzz = _metric(_zi, [t for K_ in KT for t in K_['traces']], [v for K_ in KT for v in K_['vias']], _zf, C['wmin'])
                    _kzr = _metric(_zi, [], [], _zf, C['wmin'])
                    tree_cons += [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == first for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
                if len(_kzz[0]) and (net, by_net[net][j]) not in _METRIC_OFF: _zone = _kzz
                elif len(_kzr[0]) and (net, by_net[net][j], 1) not in _METRIC_OFF: _zone = _kzr
            if TAP_ON and _tap_sp and _zone is None:
                _zone = _metric([], _tap_sp, _tap_vi, [pads[i] for i in sorted({i_ for t_ in kel for i_ in kel[t_]} | set(_tap_pads))], Cb['wmin'], TAP_CLR + QM)
                _tap_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
        tgt = set()
        if j in BF: tgt.add(BF[j][0])
        else:
            for L, cells in p['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
        if j in BF and BF[j][0] in src:


            if BGA_PLAN and gl is None:
                _c = BF[j][0]
                for _i in tree + [j]:
                    if _i in BF and BF[_i][0] == _c and _i not in _bf_done:
                        _bf_done.add(_i); all_tr += BF[_i][1]; all_v += BF[_i][2]
                _stubs.add(_c)
                tree.append(j); continue
            if not BGA_PLAN:
                all_tr += BF[j][1]; all_v += BF[j][2]; tree.append(j); continue
        if gl is None and not is_tap and j not in kel:
            tcells = set()
            for i in tree:
                for L, cells in pads[i]['cells'].items(): tcells.update((LI[L] * NN + cells).tolist())
            if tgt & tcells: tree.append(j); continue
        _wk = min(Cb['wmin'], NECK_W) if neck_active(net, Cb, Cb['wmin']) else Cb['wmin']
        _fst, _escape_target = {}, {}
        if ESCAPE:
            for i in (_qp if gl is not None else (kel[j] if j in kel else (_sz_sources if _sz_sources is not None else (tree[-1:] if SI_STUB and actual_fast(net) and not is_tap else tree)))):
                for c, l_, pts in escape(net, pads[i], _wk, capH, rng): _fst[c] = (l_, pts)
            for c, l_, pts in escape(net, p, _wk, capH, rng): _escape_target[c] = (l_, pts)
            _gem = sorted(c for c in _escape_target if c in _fst)
            if _gem:


                _c = _gem[0]; trs = [dict(layer=_escape_target[_c][0], pts=_escape_target[_c][1], w=[_wk] * (len(_escape_target[_c][1])-1))]
                if _c not in _stubs: trs.append(dict(layer=_fst[_c][0], pts=_fst[_c][1], w=[_wk] * (len(_fst[_c][1])-1)))
                _stubs.add(_c)
                if is_tap:
                    for t in trs: t['sig'] = True
                    if j in kel and (TAP_ON or TAP_TREE):
                        for t in trs: t['tap'] = True
                        _tap_sp += trs; _tap_pads.append(j)
                else: _kn = geo_nodes(trs, []); pathnodes.extend(_kn); _pn_set.update(_kn)
                all_tr += trs; tree.append(j); continue
            src.extend(_fst); tgt.update(_escape_target)
        _bf_shared = sorted(set(src) & tgt) if BGA_PLAN and gl is not None and j in BF else []
        src = sorted(set(src)) if _bf_shared else sorted({q for q in src if q not in tgt})
        if j in kel:
            src_ok = [q for q in src if capCf[q] >= Cb['wmin']]
            if src_ok: src = src_ok
        if CURRENT_VIA_PRICE and CONS and not is_tap and not SP:
            _with_cons = [i for K_ in (KT or []) if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
            _via_n = cons_count(net, pads, tree_cons, j, _with_cons)
            if _via_n > 1: Cb = dict(Cb, via=Cb['via'] * _via_n)
        bx = p['bbox']; res = None
        if not src or not tgt: ok = False; continue
        sxy = [cell_xy((q % NN) % NX, (q % NN) // NX) for q in src]
        if len(src) > 4000:
            o = sorted(range(len(src)), key=lambda q: math.hypot(sxy[q][0] - p['x'], sxy[q][1] - p['y']))[:4000]
            src_s = [src[q] for q in o]
        else: src_s = src


        _hard = REACH and (net, _wk) in _ERR_NETS and _unreachable(_components(net, _wk), src_s, tgt)
        _edges = () if _hard else ((WINDOW, 40.0) if WINDOW else (3.0, 8.0, 40.0))
        _zl = [] if _zone is None else ([_zone, _kzr] if _zone is _kzz and len(_kzr[0]) and (net, by_net[net][j], 1) not in _METRIC_OFF else [_zone])
        for _zone in _zl + [None]:
            _mz = _zone is not None
            _cH, _vH, _cC = capH, viaH, capC
            if _mz:
                if _zone is not _kzz: _cH, _vH = _metric_maps(capH, viaH, *_zone)
                else:
                    if _metric_k is None or _metric_k[0] is not capH: _metric_k = (capH,) + _metric_maps(capH, viaH, *_zone)
                    _cH, _vH = _metric_k[1], _metric_k[2]
                _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
            for _mi, margin in enumerate(_edges):
                win = (min(min(x for x, _ in sxy), bx[0]) - margin, min(min(y for _, y in sxy), bx[1]) - margin,
                       max(max(x for x, _ in sxy), bx[2]) + margin, max(max(y for _, y in sxy), bx[3]) + margin)
                if not _in_window(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = maps_larger(net, box, win, _wmax)
                    if _b2 is not None: box = _b2
                    _cH, _vH, _cC = capH, viaH, capC
                    if _mz: _cH, _vH = _metric_maps(capH, viaH, *_zone); _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
                if _bga_own_holes:



                    viaH = bga_own_hole_map(viaH, _bga_reserved_vias + all_v)
                    _vH = bga_own_hole_map(_vH, _bga_reserved_vias + all_v)
                res = astar_pf(src_s, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex)


                if res is None and REACH and _mi == len(_edges) - 2 and _unreachable(_components(net, _wk), src_s, tgt):
                    _hard = True; _ERR_NETS.add((net, _wk)); break
                if res and WINDOW: break
                if res:
                    conf = any(capCf[q] < Cb['wmin'] for q in res) or any(
                        a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(res[:-1], res[1:]))
                    if not conf or margin >= 40.0: break
                    if NB is not None and NB.info[0] == 0: break
            if _mz and res and gl is not None and Cb is not C and any(a // NN != b2 // NN for a, b2 in zip(res[:-1], res[1:])): res = None
            if res and _mz and CHAIN_REST and gl is None and not is_tap and KT is not None and (_zone is _kzz or _zone is _kzr):




                _lb = _clearance_min(src_s, tgt)
                res_f = astar_pf(src_s, tgt, bx, capH, capS, viaH, viaBad, Cb, win, extra=_ex) if _plen(res) > _lb + max(CHAIN_REST_ANT * _lb, CHAIN_REST_MM) else None
                if res_f and _plen(res) > _plen(res_f) + max(CHAIN_REST_ANT * _plen(res_f), CHAIN_REST_MM):
                    res = res_f; _mz = False; _cC = capC; CHAIN_STAT['rest_free'] += 1
                    _METRIC_OFF.add((net, by_net[net][j])); _METRIC_OFF.add((net, by_net[net][j], 1))
            if res or _hard or not _mz: break
            if _tap_z: TOPO_STAT['tap_without_zone'] += 1; continue
            _METRIC_OFF.add((net, by_net[net][j], 1) if _zone is _kzr else (net, by_net[net][j])); CHAIN_STAT['without_zone'] += 1
        trs = vs = None
        if res:
            if _mz:
                for q_ in res: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
            trs, vs = path_to_geometry(res, _cC, dict(Cb, w=Cb['wmin'], ground_map=neck_field_map(net, Cb, Cb['wmin']), extra=_ex, anchor=j not in kel))
            if _pz_active(net) and net in PZ_WIDTHS and not taps and gl is None and not BF and not _pz_geometry_ok(net,trs,vs,not HARD_FOREIGN[0]):
                repair=_pz_edge_retry(res,src_s,tgt,bx,_cH,capS,_vH,viaBad,Cb,win,_ex)
                if repair:res,trs,vs=repair
                else:res=None
            if SP and not is_tap and vs and not current_fields_free(res,vs,Cb,_cH): res = None
            if gl is not None and sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > \
               CHAIN_DETOUR * min(_pad_dist(pads[i], p) for i in _qp) + CHAIN_DETOUR_MM:
                res = None; CHAIN_STAT['detour'] += 1
        if not res and gl is not None:
            K_ = gl[0]; CHAIN_STAT['back'] += 1
            if gl[2] is None: jobs.append((j, None)); continue
            K_['stop'] = True
            _free = [i for i in K_['pins'] + K_['caps'][:-1] if i not in K_['pin_ok'] and i not in K_['caps'][:max(K_['end'], 0)]]
            jobs.extend((i, None) for i in _free); continue
        if not res:
            if _hard:
                _pk = '%s:%s' % (p['ref'], p['num'])
                if _pk not in UNREACHABLE.setdefault(net, []): UNREACHABLE[net].append(_pk)
            ok = False; continue
        _sp0 = (_tap_sp if j in kel else all_tr) if BRANCH and gl is None and (j not in kel or TAP_TREE) else None
        if _sp0 and not SP:
            TOPO_STAT['branch_checked'] += 1; _kset = set(_tap_kn) if j in kel else _pn_set
            _p0 = cell_xy((res[0] % NN) % NX, (res[0] % NN) // NX); _l0 = res[0] // NN
            _m0 = _branch_defects(trs, _sp0, _l0, _p0, res[0] not in _kset)
            if any(_m0):
                TOPO_STAT['branch_shortage'] += 1
                _ex2 = _bands(_ex, _sp0, BRANCH_D, BRANCH_FACTOR); _s2 = src_s
                if _m0[0] or _m0[2]:
                    _r = int(STAR_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX)
                    _way = {_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r}
                    _s2 = [q for q in src_s if q not in _way or q not in _kset]
                res2 = astar_pf(_s2, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex2) if _s2 else None
                if res2:



                    _prev_trunk = [(t_, t_['pts'], t_['w']) for t_ in _sp0] if R27_TRUNK else []
                    if _mz:
                        for q_ in res2: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
                    trs2, vs2 = path_to_geometry(res2, _cC, dict(Cb, w=Cb['wmin'], ground_map=neck_field_map(net, Cb, Cb['wmin']), extra=_ex2, anchor=j not in kel))
                    _p2 = cell_xy((res2[0] % NN) % NX, (res2[0] % NN) // NX)
                    _m2 = _branch_defects(trs2, _sp0, res2[0] // NN, _p2, res2[0] not in _kset)
                    _lg = lambda T_: sum(math.dist(a_, b_) for t in T_ for a_, b_ in zip(t['pts'][:-1], t['pts'][1:]))
                    _kf = lambda R_: any(capCf[q] < Cb['wmin'] for q in R_) or any(a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(R_[:-1], R_[1:]))


                    _topo_better = (all(a_ <= b_ for a_, b_ in zip(_m2, _m0)) and _m2 != _m0) if R27_TRUNK else _m2 < _m0
                    if _topo_better and len(vs2) <= len(vs) and (not _kf(res2) or _kf(res)) and _lg(trs2) <= BRANCH_MORE * _lg(trs) + 1.0:
                        res, trs, vs = res2, trs2, vs2; _p0 = _p2; _l0 = res[0] // NN; TOPO_STAT['branch_better'] += 1
                    else:
                        for t_, p_, w_ in _prev_trunk: t_['pts'], t_['w'] = p_, w_
            if res[0] in _kset and len(_arms_at(_sp0, _l0, _p0)) >= 2:
                _r = int(STAR_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX); TOPO_STAT['star_blocked'] += 1
                _star.update(_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r)


        if res[0] in _fst and res[0] not in _stubs:
            _stubs.add(res[0]); trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        if res[-1] in _escape_target:
            _stubs.add(res[-1]); trs.append(dict(layer=_escape_target[res[-1]][0], pts=_escape_target[res[-1]][1], w=[_wk] * (len(_escape_target[res[-1]][1])-1)))
        if BF_NODES:
            if BGA_PLAN:


                for _i in (_qp if gl is not None else tree) + [j]:
                    if _i in BF and BF[_i][0] in (res[0], res[-1]) and _i not in _bf_done:
                        _bf_done.add(_i); trs += BF[_i][1]; vs += BF[_i][2]
            else:
                for _c in (res[0], res[-1]):
                    if _c in BF_NODES and _c not in _stubs:
                        _stubs.add(_c); trs += BF_NODES[_c][0]; vs += BF_NODES[_c][1]
        if is_tap:
            for t in trs: t['sig'] = True
            if j in kel and (TAP_ON or TAP_TREE):
                for t in trs: t['tap'] = True
                for v in vs: v['tap'] = True
                _kn = geo_nodes(trs, vs); _tap_sp += trs; _tap_vi += vs; _tap_pads.append(j); TOPO_STAT['taps'] += 1
                _tap_kn.extend(q for q in _kn if q not in _star)
        if gl is not None:
            if SP and vs:
                _sg,_sv = current_field_set(res,vs,Cb); trs += _sg; vs += _sv
            K_ = gl[0]; CHAIN_STAT['element'] += 1
            _g = 1 + 16 * K_['nr'] + gl[1]
            _kp = [by_net[net][i] for i in (K_['pins'] + K_['caps'][:1] if gl[1] == 0 else _qp + [j])]
            for t in trs: t['link_chain'] = _g; t['kp'] = _kp
            for v in vs: v['link_chain'] = _g
            K_['traces'] += trs; K_['vias'] += vs; all_tr += trs; all_v += vs
            if gl[1] == 0: K_['kn'].extend(geo_nodes(trs, vs))
            if gl[2] is None: K_['pin_ok'].append(j)
            elif gl[1] == 0: K_['pin_ok'].append(gl[2][0]); K_['end'] = 0
            else: K_['end'] = gl[1]
            continue
        _cons = ([], [])
        if SP and vs and not is_tap and j not in kel:
            _cons = current_field_set(res,vs,Cb)
        elif CONS and NC is not None and vs and not is_tap and j not in kel:
            _with = [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]] if KT is not None else []
            _n = cons_count(net, pads, tree_cons, j, _with)
            if _n > 1: _cons = cons_set(net, res, trs, vs, _n, capH, viaH, capC, vbadf, Cb, all_v, rng)
        all_tr += trs + _cons[0]; all_v += vs + _cons[1]
        if not is_tap: _kn = geo_nodes(trs, vs); pathnodes.extend(_kn); _pn_set.update(_kn)
        tree.append(j)
        if KT is not None:
            tree_cons.append(j); tree_cons += [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
    if KT is not None:
        for K_ in KT:
            if K_['end'] >= 0 and K_['traces']:
                K_['traces'][0]['metric'] = [by_net[net][K_['caps'][K_['end']]]] + [by_net[net][i] for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
    if RK is not None and all_v: all_v = all_v + rv_set(net, all_tr, all_v, RK)
    if _pz_active(net) and not _pz_geometry_ok(net,all_tr,all_v):
        all_tr,all_v,ok=[],[],False
    add_unit(u, net, [dict(net=net, **t) for t in all_tr], all_v)
    return ok


def unit_conflicts(u, capS, dd):
    r = U[u]; bad = []
    for t in r['tracks']:
        li = LI[t['layer']]; capL = capS[li]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1)
            tt = np.linspace(0, 1, n)
            ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
            m = capL[iy, ix] < w - 0.005
            if m.any(): bad.extend((li * NN + iy[m] * NX + ix[m]).tolist())
    for v in r['vias']:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        if dd[iy, ix] < HOLE_C or any(capS[li][iy, ix] / 2 < VIA_R - 0.003 for li in range(NL)):
            bad.extend([li * NN + iy * NX + ix for li in range(NL)])
    return bad


from shapely.geometry import LineString as _LS, Point as _PT
from shapely.strtree import STRtree as _STR
DRC_CLR = DRCC + 0.004
DRC_HOLE = H2H + VIA_DR + 0.004
DRC_PADC = DRCP + (PAD_EXTRA if BGA_TIGHT else 0.004)


def _pz_clearance(net,other,pad=False):
    if globals().get('KA') is not None:
        return _ka_margin(net,other,type2='Pad' if pad else 'Track')
    if _pz_active(net) or _pz_active(other):
        return (DRCP if pad else DRCC) + PZ_BORDER_NM / 1e6
    return DRC_PADC if pad else DRC_CLR


PAIRS = []
_DMAX = max(DRC_CLR, DRC_PADC)
if globals().get('KA') is not None: _DMAX = max(_DMAX,KA.maximum+.012)
_PADB = {li: [(pn, pg, pg.buffer(_DMAX)) for pn, pg in _PADG[li]] for li in range(NL)}
if DRCL:


    _PADB = {li: [] for li in range(NL)}
    for _p in D['pads']:
        _pn = _p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p)
        for _L in _p['layers']:
            if _L not in LI: continue
            _g = (shapely.union_all([Polygon(q) for q in _p['polys'][_L]])
                  if globals().get('KA') is not None else pad_hull(_p, _L, DRCP, True))
            if globals().get('KA') is not None: _KA_META[id(_g)] = _ka_pad_kind(_p)
            _PADB[LI[_L]].append((_pn, _g, _g.buffer(_DMAX)))
if globals().get('KA') is not None:
    for _li,_keeps in _KA_PAD_KEEP.items():
        _PADB[_li].extend((_pn,_g,_g.buffer(_DMAX)) for _pn,_g,_kind in _keeps)


def _unit_geo(u, r):
    g = _GEO.get(u)
    if g is None:
        g = []; n0 = r['net']
        for t in r['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                radius=w/2/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else w/2
                k = _LS([a, b]).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8) if a != b else _PT(a).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8)
                g.append((li, k, k.buffer(_DMAX), n0))
                if globals().get('KA') is not None: _KA_META[id(k)] = ('Track','')
        for v in r['vias']:
            k = _PT(v['x'], v['y']).buffer(VIA_R/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else VIA_R, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8); kb = k.buffer(_DMAX); nv = v.get('net') or n0
            for li in range(NL):
                if globals().get('KA') is not None and LAYERS[li] not in v.get('layers',LAYERS):continue
                g.append((li, k, kb, nv))
            if globals().get('KA') is not None: _KA_META[id(k)] = ('Via','')
        _GEO[u] = g
    return g


FIX_PAIRS = []


def _fixed_pair(a, b):


    return a.startswith('FIX#') and b.startswith('FIX#') and (globals().get('args') or {}).get('rest_checking') != '1'


def _conflict_points(a, b, spacing):
    if spacing == 0:
        c = shapely.point_on_surface(shapely.intersection(a, b))
        return c, c
    line = shapely.shortest_line(a, b)
    return shapely.get_point(line, 0), shapely.get_point(line, -1)


def _pad_drill_conflicts(vias, rating, report=True):
    if not PAD_HOLE_GEOMS or not vias: return
    holes=np.array(PAD_HOLE_GEOMS,dtype=object)
    points=np.array([_PT(x,y) for _,_,x,y in vias],dtype=object)
    a,b=_STR(holes).query(shapely.envelope(shapely.buffer(points,VIA_DR/2+H2H+.0011)))
    distance=shapely.distance(points[a],holes[b])-VIA_DR/2
    for k,j in enumerate(b.tolist()):
        circle=PAD_HOLE_CIRCLES[j]
        if circle is not None:
            _,_,x,y=vias[int(a[k])]
            distance[k]=math.hypot(x-circle[0],y-circle[1])-circle[2]-VIA_DR/2
    for i,j,d in zip(a.tolist(),b.tolist(),distance.tolist()):
        if d >= H2H-1e-9: continue
        u,_,x,y=vias[i]
        for li in range(NL): rating(u,x,y,li)
        if report:
            PAIRS.append(('*',round(d,4),u,'PAD',round(x,2),round(y,2)))
            if VIA_PRECEDENCE: PAIRS_RAW.append((-1,x,y,u,'PAD'))


def _in_window(bounds, F):
    return bounds[0] <= F[2] and bounds[2] >= F[0] and bounds[1] <= F[3] and bounds[3] >= F[1]
def exact_conflicts(without=None, window=None):
    PAIRS.clear(); PAIRS_RAW.clear(); FIX_PAIRS.clear()
    geoms = {li: [] for li in range(NL)}
    drills_l = []
    for u, r in U.items():
        if without is not None and u in without: continue
        for li, k, kb, ne in _unit_geo(u, r):
            if window is None or _in_window(k.bounds, window): geoms[li].append((u, ne, k, kb))
        for v in r['vias']:


            if window is not None and not _in_window((v['x'], v['y'], v['x'], v['y']), window): continue
            nv = (v.get('net') or r['net']) if BGA_HOLE or bool(globals().get('PZ_LOCAL')) else r['net']
            drills_l.append((u, nv, v['x'], v['y']))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]:
            if window is None or _in_window(pg.bounds, window): geoms[li].append(('PAD', pn, pg, pb))
    conf = {}
    def rating(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rr = 3
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - rr), min(NY, iy + rr + 1)):
            for jx in range(max(0, ix - rr), min(NX, ix + rr + 1)):
                lst.append(li * NN + jy * NX + jx)
    for li in range(NL):
        G = geoms[li]
        if not G: continue
        ga = np.array([g for _, _, g, _ in G], dtype=object)
        tree = _STR(ga)
        net_name = np.array([n for _, n, _, _ in G], dtype=object)
        if (BGA_HOLE or bool(globals().get('PZ_LOCAL')) or globals().get('KA') is not None) and HOLE_CLR > 0 and drills_l:


            hp = np.array([_PT(x, y) for _, _, x, y in drills_l], dtype=object)
            hi, hj = tree.query(shapely.buffer(hp, VIA_DR / 2 + HOLE_CLR + 0.00001))
            hn = np.array([n for _, n, _, _ in drills_l], dtype=object)
            hd = shapely.distance(hp[hi], ga[hj]) - VIA_DR / 2
            hm = (hn[hi] != net_name[hj]) & (hd < HOLE_CLR - 0.000001)
            for i, j, dij in zip(hi[hm].tolist(), hj[hm].tolist(), hd[hm].tolist()):
                u, _, x, y = drills_l[i]; u2 = G[j][0]
                if _fixed_pair(u, u2): continue
                c2 = shapely.get_point(shapely.shortest_line(hp[i], ga[j]), -1)



                rating(u, x, y, li); rating(u2, c2.x, c2.y, li)
                cx, cy = (x+c2.x)/2, (y+c2.y)/2
                PAIRS.append((LAYERS[li], round(dij, 4), u, u2, round(cx, 2), round(cy, 2)))
                if VIA_PRECEDENCE: PAIRS_RAW.append((li, cx, cy, u, u2))
        qi, qj = tree.query(np.array([gb for _, _, _, gb in G], dtype=object))
        m = qj > qi
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        net_name = np.array([n for _, n, _, _ in G], dtype=object); pad = np.array([u == 'PAD' for u, _, _, _ in G])
        m = (net_name[qi] != net_name[qj]) & ~(pad[qi] & pad[qj])
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        dist = shapely.distance(ga[qi], ga[qj])
        limits = np.where(pad[qi] | pad[qj], DRC_PADC, DRC_CLR)
        if globals().get('PZ_LOCAL'):
            limits = np.array([_pz_clearance(net_name[i],net_name[j],bool(pad[i] or pad[j])) for i,j in zip(qi,qj)])
        if globals().get('KA') is not None:
            limits = np.array([_ka_geo_limit(net_name[i],net_name[j],li,ga[i],ga[j],bool(pad[j])) for i,j in zip(qi,qj)])
        m = dist < limits
        for i, j, dij in zip(qi[m].tolist(), qj[m].tolist(), dist[m].tolist()):
            u, net, g, _ = G[i]; u2, net2, g2, _ = G[j]
            if _fixed_pair(u, u2):
                FIX_PAIRS.append((LAYERS[li], round(dij, 4), u, u2)); continue
            if BGA_PLAN or CONFLICT_SITE_EXACT or globals().get('_REST_LOCAL_CONFLICT', False):


                c, c2 = _conflict_points(g, g2, dij)
                rating(u, c.x, c.y, li); rating(u2, c2.x, c2.y, li)
                c = _PT((c.x+c2.x)/2,(c.y+c2.y)/2)
            else:
                c = g.centroid if dij == 0 else g.boundary.interpolate(g.boundary.project(g2.centroid))
                rating(u, c.x, c.y, li); rating(u2, c.x, c.y, li)
            PAIRS.append((LAYERS[li], round(dij, 3), u, u2, round(c.x, 2), round(c.y, 2)))
            if VIA_PRECEDENCE: PAIRS_RAW.append((li, c.x, c.y, u, u2))
    _pad_drill_conflicts(drills_l,rating)
    if len(drills_l) > 1:
        from scipy.spatial import cKDTree
        P = np.array([(x, y) for _, _, x, y in drills_l])
        _vhclear = max(DRC_HOLE, VIA_DR / 2 + VIA_R + HOLE_CLR) if BGA_HOLE or bool(globals().get('PZ_LOCAL')) else DRC_HOLE
        for i, j in sorted(cKDTree(P).query_pairs(_vhclear * (1 + 1e-9) + 1e-9)):
            u, n1, x1, y1 = drills_l[i]; u2, n2, x2, y2 = drills_l[j]
            if math.hypot(x1 - x2, y1 - y2) < DRC_HOLE or ((BGA_HOLE or bool(globals().get('PZ_LOCAL'))) and n1 != n2 and math.hypot(x1 - x2, y1 - y2) < VIA_DR / 2 + VIA_R + HOLE_CLR - 0.000001):
                if _fixed_pair(u, u2):
                    FIX_PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 4), u, u2)); continue
                for li in range(NL): rating(u, x1, y1, li); rating(u2, x2, y2, li)
                if VIA_PRECEDENCE: PAIRS_RAW.append((-1, x1, y1, u, 'PAD')); PAIRS_RAW.append((-1, x2, y2, u2, 'PAD'))


                PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 3), u, u2, round((x1 + x2) / 2, 2), round((y1 + y2) / 2, 2)))
    return conf


def _narrow_raw():
    out = []; Cs = cls('/__signal__')
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        stub = u.startswith('GND#') or u in PLANE_GROUPS
        C = dict(wmin=max(0.15, TRACK_MIN)) if stub else cls(r['net'])
        WF = neck_field_map(r['net'], C, C['wmin'], stub)
        for t in r['tracks']:
            if t.get('web'): continue
            nominal = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= nominal - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                permitted = WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < nominal
                out.append((u, r['net'], li, a, b, w, nominal, bool(permitted)))
    return out
def under_wmin():
    out = []; before = None
    for u, net, li, a, b, w, nominal, permitted in _narrow_raw():
        L = math.dist(a, b)
        if before is not None and before[0] == (u, li, permitted, nominal) and before[1] == tuple(a):
            e = out[-1]; e['_p'].append((a, b)); e['total_length'] += L; e['width'] = min(e['width'], w)
        else:
            out.append(dict(net_name=net, layer_name=LAYERS[li], x=0.0, y=0.0, width=w, nominal=nominal, total_length=L, allowed=permitted, _p=[(a, b)]))
        before = ((u, li, permitted, nominal), tuple(b))
    for e in out:
        half = e['total_length'] / 2; p = e.pop('_p')
        for a, b in p:
            L = math.dist(a, b)
            if half <= L or (a, b) == p[-1]:
                f = min(1.0, half / L) if L > 0 else 0.0
                e['x'] = round(a[0] + f * (b[0] - a[0]), 3); e['y'] = round(a[1] + f * (b[1] - a[1]), 3); break
            half -= L
        e['total_length'] = round(e['total_length'], 3)
    return out


WMIN_BEFORE = args.get('wmin_precedence', '1') == '1'


def _precedence(net):
    if not NC: return 1
    return 2 if NC['net_list'].get(net, {}).get('category') in ('current_val', 'supply', 'sw', 'gnd') else 1


def tight_conflicts():
    raw_data = [s for s in _narrow_raw() if not s[7]]
    if WMIN_BEFORE and NC:
        raw_data = [s for s in raw_data if not s[0].startswith('GND#') and _precedence(s[1]) >= 2]
    conf = {}
    if not raw_data: return conf
    def rating(u, x, y, li):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    for li in sorted({s[2] for s in raw_data}):
        G = [(u, ne, k) for u, r in U.items() for l2, k, _kb, ne in _unit_geo(u, r) if l2 == li]
        ga = np.array([g for _, _, g in G], dtype=object); tree = _STR(ga)
        for u, net, _li, a, b, w, nominal, _e in raw_data:
            if _li != li: continue
            k = _LS([a, b]).buffer(nominal / 2, 8) if a != b else _PT(a).buffer(nominal / 2, 8)
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            opponent = sorted({G[j][0] for j in tree.query(k.buffer(DRC_CLR)).tolist() if G[j][1] != net and k.distance(G[j][2]) < DRC_CLR})



            soft = [u2 for u2 in opponent if _precedence(U[u2]['net']) < _precedence(net)] if WMIN_BEFORE else []
            if not soft: rating(u, mx, my, li)
            for u2 in (soft or opponent): rating(u2, mx, my, li)
            for u2 in opponent or ['PAD']: PAIRS.append((LAYERS[li], round(w - nominal, 3), u, u2, round(mx, 2), round(my, 2)))
    return conf


_CAND_IDX = [None]


def _cand_idx_build(without):
    cu_layers = {}
    per = {li: [] for li in range(NL)}
    for u, r in U.items():
        if u == without: continue
        for li, k, kb, ne in _unit_geo(u, r): per[li].append((u, ne, k, False))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: per[li].append(('PAD', pn, pg, True))
        if per[li]:
            ga = np.array([g for _, _, g, _ in per[li]], dtype=object)
            cu_layers[li] = (_STR(ga), [o for o, _, _, _ in per[li]], [n for _, n, _, _ in per[li]], ga, [q for _, _, _, q in per[li]])
    return dict(cu_layers=cu_layers, prior=set(), new={}, missing={without})


def cand_idx_changed(u):
    X = _CAND_IDX[0]
    if X is not None: X['prior'].add(u); X['new'][u] = _unit_geo(u, U[u]); X['missing'].discard(u)


def cand_idx_back(u):
    X = _CAND_IDX[0]
    if X is not None and u in X['missing']: cand_idx_changed(u)


def unit_conflict_free(u):
    if BGA_HOLE or globals().get('PZ_LOCAL') or globals().get('KA') is not None: return u not in exact_conflicts()
    X = _CAND_IDX[0]
    if X is None or len(X['prior']) > 48: X = _CAND_IDX[0] = _cand_idx_build(u)
    r = U[u]; net = r['net']; geo = _unit_geo(u, r)
    for li in range(NL):
        mine = [(k, kb, ne) for l2, k, kb, ne in geo if l2 == li]
        if not mine: continue
        T = X['cu_layers'].get(li)
        if T is not None:
            tree, own, net_name, ga, pad = T
            qi, qj = tree.query(np.array([kb for _, kb, _ in mine], dtype=object))
            for i, j in zip(qi.tolist(), qj.tolist()):
                if own[j] == u or own[j] in X['prior'] or net_name[j] == mine[i][2] or _fixed_pair(u, own[j]): continue
                if mine[i][0].distance(ga[j]) < (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)): return False
        for u2, g2 in X['new'].items():
            if u2 == u or u2 not in U or _fixed_pair(u, u2): continue
            for l2, k2, _, n2 in g2:
                if l2 != li: continue
                for k, _, ne in mine:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
        if any(ne != net for _, _, ne in mine):
            for i, (k, _, ne) in enumerate(mine):
                for k2, _, n2 in mine[i + 1:]:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
    if r['vias']:
        pad_conflict = []
        _pad_drill_conflicts([(u, net, v['x'], v['y']) for v in r['vias']],
                          lambda *q: pad_conflict.append(q), report=False)
        if pad_conflict: return False
        all_items = [(v['x'], v['y'], u2) for u2, r2 in U.items() for v in r2['vias']]
        for i, v in enumerate(r['vias']):
            for x, y, u2 in all_items:
                if (u2 == u and x == v['x'] and y == v['y']) or _fixed_pair(u, u2): continue
                if math.hypot(v['x'] - x, v['y'] - y) < DRC_HOLE: return False
    return True


def groups_index(group):
    g = set(group); per = {li: [] for li in range(NL)}; bore = []
    for u, r in U.items():
        if u in g: continue
        for li, k, kb, ne in _unit_geo(u, r): per[li].append((u, ne, k))
        for v in r['vias']: bore.append((v['x'], v['y'], u))
    cu_layers = {}
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: per[li].append(('PAD', pn, pg))
        if per[li]:
            ga = np.array([q for _, _, q in per[li]], dtype=object)
            cu_layers[li] = (_STR(ga), [o for o, _, _ in per[li]], [n for _, n, _ in per[li]], ga)
    return dict(cu_layers=cu_layers, bore=bore, bxy=np.array([(x, y) for x, y, _ in bore], dtype=np.float64).reshape(-1, 2))


def conflicts_group(group, X, without=None):
    if BGA_HOLE or globals().get('PZ_LOCAL') or globals().get('KA') is not None:


        conf = exact_conflicts(without)
        g = set(group)
        with_ = {u for pair in PAIRS if pair[2] in g or pair[3] in g for u in pair[2:4]}
        return {u: cs for u, cs in conf.items() if u in with_}
    conf = {}
    def rating(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)):
                lst.append(li * NN + jy * NX + jx)
    with_ = [u for u in group if u in U]
    for li in range(NL):
        mine = [(u, ne, k, kb) for u in with_ for l2, k, kb, ne in _unit_geo(u, U[u]) if l2 == li]
        if not mine: continue
        search = np.array([kb for _, _, _, kb in mine], dtype=object)
        T = X['cu_layers'].get(li)
        if T is not None:
            tree, own, net_name, ga = T
            qi, qj = tree.query(search)
            for i, j in zip(qi.tolist(), qj.tolist()):
                u, net, k, _ = mine[i]
                if net_name[j] == net or _fixed_pair(u, own[j]) or (without is not None and own[j] in without): continue
                d = k.distance(ga[j])
                if d < (DRC_PADC if own[j] == 'PAD' else DRC_CLR):
                    if CONFLICT_SITE_EXACT:
                        c, c2 = _conflict_points(k, ga[j], d)
                        rating(u, c.x, c.y, li); rating(own[j], c2.x, c2.y, li)
                    else:
                        c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(ga[j].centroid))
                        rating(u, c.x, c.y, li); rating(own[j], c.x, c.y, li)
        gm = np.array([k for _, _, k, _ in mine], dtype=object)
        qi, qj = _STR(gm).query(search)
        for i, j in zip(qi.tolist(), qj.tolist()):
            if j <= i or mine[i][1] == mine[j][1] or _fixed_pair(mine[i][0], mine[j][0]): continue
            d = mine[i][2].distance(mine[j][2])
            if d < DRC_CLR:
                k = mine[i][2]
                if CONFLICT_SITE_EXACT:
                    c, c2 = _conflict_points(k, mine[j][2], d)
                    rating(mine[i][0], c.x, c.y, li); rating(mine[j][0], c2.x, c2.y, li)
                else:
                    c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(mine[j][2].centroid))
                    rating(mine[i][0], c.x, c.y, li); rating(mine[j][0], c.x, c.y, li)
    own_set = [(v['x'], v['y'], u) for u in with_ for v in U[u]['vias']]
    _pad_drill_conflicts([(u,None,x,y) for x,y,u in own_set],rating,report=False)
    for i, (x, y, u) in enumerate(own_set):
        if len(X['bxy']):
            for j in np.flatnonzero(np.hypot(X['bxy'][:, 0] - x, X['bxy'][:, 1] - y) < DRC_HOLE).tolist():
                if _fixed_pair(u, X['bore'][j][2]) or (without is not None and X['bore'][j][2] in without): continue
                for li in range(NL): rating(u, x, y, li); rating(X['bore'][j][2], X['bore'][j][0], X['bore'][j][1], li)
        for x2, y2, u2 in own_set[i + 1:]:
            if math.hypot(x - x2, y - y2) < DRC_HOLE:
                if _fixed_pair(u, u2): continue
                for li in range(NL): rating(u, x, y, li); rating(u2, x2, y2, li)
    return conf


FLOW_W = {}






WIDE_DISTURB = args.get('wide_disturb', '1' if NC else '0') == '1'
R32 = args.get('r32', '0') == '1' and NC is not None
_DIST_C = {}


def _dist_c(name, nets, body=False):
    key = (name, frozenset(nets), body) if DISTURB_FLANK else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if DISTURB_FLANK else sum(_NVER.get(n, 0) for n in nets)
    c = _DIST_C.get(key)
    if c is None or c[0] != sig:
        m = _mask_arr(nets)
        if body and _COILS_M is not None: m |= _COILS_M
        d = np.full(NL * NN, 99.0, dtype=np.float32)
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): d[sl] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        c = (sig, d.reshape(NL, NY, NX)); _DIST_C[key] = c
    return c[1]


def _disturb_width(net):
    if not (WIDE_DISTURB and NC): return []
    pairs = []
    if net in AGG_SW and SW_DIST > 0: pairs.append(('b_vsw', VICT_SW, SW_DIST))
    if net in AGG_FAST and SEPARATE > 0: pairs.append(('b_va', VICT_A, _separate_mm(net)))
    if net in VICT_SW and SW_DIST > 0: pairs.append(('b_asw', AGG_SW, SW_DIST))
    if net in VICT_A and SEPARATE > 0:
        if DISTURB_FLANK:
            for f, ns in _edges_groups(): pairs.append((('b_af', f), ns, SEPARATE * f))
        else: pairs.append(('b_af', AGG_FAST, SEPARATE))
    if SI:
        victim = {n for n, k in NC['net_list'].items() if k.get('category') in ('analog', 'kelvin') or k.get('kelvin')}
        for other in sorted((AGG_SW | AGG_FAST) if net in victim else victim if net in AGG_SW | AGG_FAST else []):
            d = si_spacing(net, other) if net in victim else si_spacing(other, net)
            if d > 0: pairs.append((('b_si', other), {other}, d))
    return [(_dist_c(n_, frozenset(m_ - {net}), body=DISTURB_FLANK and n_ == 'b_asw'), sp) for n_, m_, sp in pairs if m_ - {net} or (DISTURB_FLANK and n_ == 'b_asw')]





WIDTH_INNER = float(args.get('width_inner', '2.3')) if NC is not None else 0.0


def f_inner(net, li):
    if WIDTH_INNER <= 0 or li in (0, NL - 1): return 1.0
    k = NC['net_list'].get(net) or {}
    if k.get('category') not in ('current_val', 'supply', 'sw'): return 1.0
    if args.get('current_nominal') == '1':
        from common.current_track import nominal_width
        return nominal_width(k, inner=True) / nominal_width(k)
    if k.get('w_inner') and k.get('w'): return max(1.0, float(k['w_inner']) / float(k['w']))
    return WIDTH_INNER


def _r32_clearance(X, u, li, a, b, net, limit):
    T = X['cu_layers'].get(li)
    if T is None or limit <= 0: return limit
    seg = _LS([a, b]) if a != b else _PT(a)
    tree, own, net_name, ga, pad = T
    for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-9)):
        j = int(j)
        if net_name[j] == net or _fixed_pair(u, own[j]): continue
        limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR))))
    return limit


def _widen_unit_prev(u, C):
    if pair_unit(u) or U[u]['net'] in IMP_TAB: return
    if current_model(U[u]['net']):
        current_correct(u,C);return
    r = U[u]; net = r['net']
    _sb = _disturb_width(net)
    del_unit(u)
    uniform = R32 and (NC['net_list'].get(net) or {}).get('category') in ('signal', 'fast', 'analog')
    if all(t.get('sig') for t in r['tracks']) and not uniform:
        add_unit(u, net, list(r['tracks']), r['vias']); return
    rx = _cand_idx_build(u) if uniform else None
    xs = [q[0] for t in r['tracks'] for q in t['pts']] or [0.0]; ys = [q[1] for t in r['tracks'] for q in t['pts']] or [0.0]
    goal = FLOW_W.get(u, {})
    _fmax = max([f_inner(net, LI[t['layer']]) for t in r['tracks']] + [1.0])
    capH, _, capS, dd, _, _, capC, _, _ = maps(net, (min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0), max([C['w']] + list(goal.values())) * _fmax)
    new_tr = list(r['tracks']); series = range(len(new_tr)); metric = None
    if CHAIN and any('metric' in t for t in new_tr):
        metric = _metric_prepare(r, net); width_list = [list(t['w']) for t in new_tr]
        series = sorted(series, key=lambda ti: (not new_tr[ti].get('link_chain'), new_tr[ti].get('link_chain') or 0, ti))
    ab = _tap_prepare(r, net) if TAP_ON and any(t.get('tap') for t in new_tr) else None
    for ti in series:
        t = new_tr[ti]
        if t.get('sig') and not uniform: continue
        li = LI[t['layer']]; pts = t['pts']; ws = []
        for si, (a, b, w0) in enumerate(zip(pts[:-1], pts[1:], t['w'])):
            wv = min(goal.get((ti, si), C['w']) * f_inner(net, li), line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net))
            if metric is not None: wv = min(wv, _metric_width(metric, r, width_list, ti, a, b, li))
            if ab is not None: wv = min(wv, _tap_width(ab, a, b, li))
            for _dk, _sp in _sb:
                wv = min(wv, 2 * (line_cap_min(_dk[li], a, b) - _sp))
                for _lj, _s in neighbor_spacing(li, _sp):
                    wv = min(wv, 2 * (line_cap_min(_dk[_lj], a, b) - _s))
            if 'w_step' in C: wv = min(wv, w0 + C['w_step'])
            if uniform:
                ws.append(_r32_clearance(rx, u, li, a, b, net, min(C['w'], wv)))
            else: ws.append(max(w0, math.floor(wv * 40) / 40))
        new_tr[ti] = dict(t, w=ws)
        if metric is not None: width_list[ti] = ws
    if uniform:

        widths = [w for t in new_tr for w in t['w']]
        if widths:


            minimum = TRACK_MIN if TRACK_MIN > 0 else C['wmin']
            target = max(minimum, C['w']) if min(widths) >= max(minimum, C['w']) - 1e-9 else minimum
            new_tr = [dict(t, w=[target] * len(t['w'])) for t in new_tr]
    add_unit(u, net, new_tr, r['vias'])


WIDE_DENSE = args.get('wide_dense', '0') == '1'


def _wide_dense_cap(X, u, li, a, b, net, limit):
    seg = _LS([a, b]) if a != b else _PT(a)
    T = X['cu_layers'].get(li)
    if T is not None:
        tree, own, net_name, ga, pad = T

        for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-7).envelope):
            j = int(j)
            if own[j] == u or own[j] in X['prior'] or net_name[j] == net or _fixed_pair(u, own[j]): continue
            limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)) - 1e-7))
    for other, geo in X['new'].items():
        if other == u or other not in U or _fixed_pair(u, other): continue
        for l2, g, _, n2 in geo:
            if l2 == li and n2 != net:
                limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))

    for l2, g, _, n2 in _unit_geo(u, U[u]):
        if l2 == li and n2 != net:
            limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))
    if (BGA_HOLE or bool(globals().get('PZ_LOCAL')) or globals().get('KA') is not None) and HOLE_CLR > 0:
        for other, r in U.items():
            if _fixed_pair(u, other): continue
            for v in r['vias']:
                if (v.get('net') or r['net']) != net:
                    limit = min(limit, 2 * (seg.distance(_PT(v['x'], v['y'])) - VIA_DR / 2 - HOLE_CLR - 1e-7))
    return limit
def widen_unit(u, C):
    if not WIDE_DENSE:
        return _widen_unit_prev(u, C)
    old = copy.deepcopy(U[u])
    X = _CAND_IDX[0]
    if X is None or len(X['prior']) > 48:
        X = _CAND_IDX[0] = _cand_idx_build(u)
    _widen_unit_prev(u, C)
    r = U[u]


    shape = lambda q: ([(t['layer'], t['pts']) for t in q['tracks']],
                       [(v['x'], v['y'], v.get('net')) for v in q['vias']])
    if shape(r) != shape(old):
        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias'])
        cand_idx_changed(u)
        return
    tr = []
    for t, t0 in zip(r['tracks'], old['tracks']):
        ws = []
        for a, b, w, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w'], t0['w']):
            if w > w0:
                cap = _wide_dense_cap(X, u, LI[t['layer']], a, b, r['net'], w)

                w = max(w0, min(w, math.floor(max(0., cap) * 40) / 40))
            ws.append(w)
        tr.append(dict(t, w=ws))
    del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    cand_idx_changed(u)









QUIET = args.get('width_quiet', '1' if NC else '0') == '1'
QUIET_R = float(args.get('quiet_r', '0.03')); QUIET_MM = float(args.get('quiet_mm', '0.35')); QUIET_SHORT = float(args.get('quiet_short', '1.0'))
FLOW_A = {}
if QUIET:
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from common.width_profile import calm as _bp_quiet, count as _bp_count


def width_calm(u, C):
    if pair_unit(u) or U[u]['net'] in IMP_TAB: return (0, 0, 0)
    r = U[u]; goal = FLOW_W.get(u); ant = FLOW_A.get(u); before = post = n = 0; new_tr = []
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('web') or len(t['w']) < 2: new_tr.append(t); continue
        l = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(t['pts'][:-1], t['pts'][1:])]
        _fi = f_inner(r['net'], LI[t['layer']])
        z = [goal.get((ti, si), C['w']) * _fi for si in range(len(l))] if goal else None
        a_ = max(ant.get((ti, si), 1.0) for si in range(len(l))) if ant else 1.0
        ws = _bp_quiet(l, t['w'], goal=z, floor=C['wmin'], lam=QUIET_MM, short=QUIET_SHORT, r_max=QUIET_R / max(a_, 0.2))
        before += _bp_count(l, t['w'])[0]; post += _bp_count(l, ws)[0]
        if ws != list(t['w']): n += 1; t = dict(t, w=ws)
        new_tr.append(t)
    if n: del_unit(u); add_unit(u, r['net'], new_tr, r['vias'])
    else: U[u] = U.pop(u)
    return before, post, n
def skew_match(board, routes, options, target_folder):
    import collections, hashlib, re
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    folder = Path(target_folder); folder.mkdir(parents=True, exist_ok=True)
    def sha(v): return hashlib.sha256(json.dumps(v,sort_keys=True).encode()).hexdigest()
    def signature(d):
        return collections.Counter((v['type'],v['description'],json.dumps([(i['description'],i['pos']) for i in v['items']],sort_keys=True))
            for v in d['violations'] if v['severity']=='error' and v['type']!='skew_out_of_range')
    def opens(d):
        return collections.Counter(re.search(r'\[([^]]+)\]',v['items'][0]['description'])[1] for v in d['unconnected_items'])
    def allowed(old,new):
        return not(signature(new)-signature(old)) and not(opens(new)-opens(old))
    current = copy.deepcopy(routes)
    drc = native_route_verify(board,current,options,folder/'begin')
    native_folder=folder/'begin'
    report = dict(version='native-skew-v1', move_records=[], tried_kicad_native=0, before=native_skew_error(drc),
                  input_sha=sha(routes), kicad_native_before=dict(faults=sum(v['severity']=='error' for v in drc['violations']),open_items=len(drc['unconnected_items'])))
    boundary=border_polygon(board).buffer(-float(options.get('edge',.3)))
    clearance=max(float(options.get('drc',.09)),float(options.get('drcpad',.09)))+.002
    vd=float(options.get('via','0.45/0.20').split('/')[0])
    maxmoves=int(options.get('skew_moves','24'));maxnative=int(options.get('skew_candidates','80'))
    for iteration in range(maxmoves):
        errors=native_skew_error(drc)
        if not errors: break
        moved=False
        for error in errors:
            net=error['net'];partner=error['partner'];delta=error['delta_mm']-.35*error['tolerance_mm']

            foreign={}; own={}; partners={}
            for t in current['tracks']:
                L=t['layer']
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    if math.dist(a,b)<1e-7:continue
                    g=LineString([a,b]).buffer(w/2)
                    if t['net']!=net:foreign.setdefault(L,[]).append(g)
                    if t['net']==partner:partners.setdefault(L,[]).append(g)
            for v in current['vias']:
                for L in v.get('layers') or ['F']+['In%d'%i for i in range(1,board.get('cu_layers',4)-1)]+['B']:
                    (foreign if v['net']!=net else own).setdefault(L,[]).append(Point(v['x'],v['y']).buffer(v.get('d',vd)/2))
            for p in board['pads']:
                for L,polys in p.get('polys',{}).items():
                    (foreign if p['net']!=net else own).setdefault(L,[]).extend(Polygon(q) for q in polys)
            obstacles={L:unary_union(gg) for L,gg in foreign.items()}


            spans=[]
            for ti,t in enumerate(current['tracks']):
                if t['net']!=net or t.get('items'):continue
                pts=t['pts'];ws=t['w']
                for i in range(len(ws)):
                    for j in range(i+1,len(pts)):
                        widths=ws[i:j]
                        if max(widths)-min(widths)>1e-7:break
                        a,b=pts[i],pts[j];L=math.dist(a,b)
                        if L<.25:continue
                        dx,dy=abs(a[0]-b[0]),abs(a[1]-b[1])
                        oldlen=sum(math.dist(x,y) for x,y in zip(pts[i:j],pts[i+1:j+1]))

                        if min(dx,dy,abs(dx-dy))<=2e-5:
                            spans.append((-L,ti,i,j,oldlen,(tuple(a),tuple(b)),0))
                        else:



                            diag=min(dx,dy);sx=1 if b[0]>a[0] else -1;sy=1 if b[1]>a[1] else -1
                            for mid in ((a[0]+sx*diag,a[1]+sy*diag),(b[0]-sx*diag,b[1]-sy*diag)):
                                chain=(tuple(a),mid,tuple(b))
                                for ai in range(2):
                                    length=math.dist(chain[ai],chain[ai+1])
                                    if length>=.25:spans.append((-length,ti,i,j,oldlen,chain,ai))
            spans.sort(key=lambda q:(len(q[5])>2,q))
            candidates=[]
            for _,ti,i,j,oldlen,chain,ai in spans:
                t=current['tracks'][ti];a,b=chain[ai],chain[ai+1];w=t['w'][i];L=math.dist(a,b);layer=t['layer']
                baselen=sum(math.dist(x,y) for x,y in zip(chain,chain[1:]))
                e=((b[0]-a[0])/L,(b[1]-a[1])/L);normal=(-e[1],e[0])
                rest=[g for g in own.get(layer,[]) if g.distance(Point(a))>w and g.distance(Point(b))>w]
                for tj,tt in enumerate(current['tracks']):
                    if tt['net']!=net or tt['layer']!=layer:continue
                    rest.extend(LineString([pa,pb]).buffer(ww/2) for si,(pa,pb,ww) in enumerate(zip(tt['pts'][:-1],tt['pts'][1:],tt['w'])) if tj!=ti or not max(0,i-1)<=si<min(len(tt['w']),j+1))
                selfgeo=unary_union(rest)
                for fraction in (1.,.5,.25,.125,.0625):
                    added=delta*fraction
                    if added < .01:continue
                    extra=added+oldlen-baselen
                    if extra<=.005:continue
                    c=max(w/2,.04);pitch=max(2*w+clearance,w+float(options.get('pair_gap',.2)));top=pitch-2*c
                    from common.meander import crests_shapes as _ma_shapes, dome_track as _ma_track
                    for fi_,(n_,s_,r_,A_,F_) in enumerate(_ma_shapes(extra,L,w,clearance) if options.get('skew_shape','rounded')=='rounded' else []):
                        leadspace=L-F_

                        for lead in [x for x in dict.fromkeys([leadspace/2,.05,leadspace-.05,leadspace*.25,leadspace*.75]) if x>=.05-1e-9 and leadspace-x>=.05-1e-9]:
                            for side in (1,-1):
                                points,sp_items=_ma_track(chain,ai,n_,s_,r_,A_,lead,side,w)
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple or not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue
                                pocket=LineString(points[1:-1]).buffer(w/2)

                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,0,fi_,coupling,j-i,ti,i,j,side,lead,'dome',chain,ai)
                                candidates.append((rank,ti,i,j,points,added,w,layer,'dome',sp_items))
                    for shape in ('trombone','triangle'):
                        height=(extra+4*c*(2-math.sqrt(2)))/2 if shape=='trombone' else extra/(2*(math.sqrt(2)-1))
                        span=4*c+top if shape=='trombone' else 2*height
                        if shape=='trombone' and height<2*c:continue
                        leadspace=L-span
                        if leadspace<.04:continue
                        for lead in dict.fromkeys([leadspace/2,.02,leadspace-.02,leadspace*.25,leadspace*.75]):
                            for side in (1,-1):
                                if shape=='trombone':
                                    q=[(0,0),(lead,0),(lead+c,c),(lead+c,height-c),(lead+2*c,height),(lead+2*c+top,height),(lead+3*c+top,height-c),(lead+3*c+top,c),(lead+span,0),(L,0)]
                                else:q=[(0,0),(lead,0),(lead+height,height),(lead+span,0),(L,0)]
                                points=[[round(a[0]+x*e[0]+side*y*normal[0],6),round(a[1]+x*e[1]+side*y*normal[1],6)] for x,y in q]
                                points[0]=list(a);points[-1]=list(b)
                                points=[list(p) for p in chain[:ai]]+points+[list(p) for p in chain[ai+2:]]
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple:continue
                                if not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue

                                pocket=LineString(points[1:-1]).buffer(w/2)
                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,2*w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,1,height,coupling,j-i,ti,i,j,side,lead,shape,chain,ai)
                                candidates.append((rank,ti,i,j,points,added,w,layer,shape,None))
                if len(candidates)>100:break
            candidates.sort(key=lambda q:q[0])
            seen=set()
            for _,ti,i,j,points,added,w,layer,shape,sp_items in candidates:
                key=(ti,i,j,tuple(map(tuple,points)))
                if key in seen:continue
                seen.add(key)
                if report['tried_kicad_native']>=maxnative:break
                candidate=copy.deepcopy(current);t=candidate['tracks'][ti]
                if sp_items is not None:
                    before=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][:i],t['pts'][1:i+1],t['w'][:i]) if math.dist(p,q)>1e-9]
                    post=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][j:],t['pts'][j+1:],t['w'][j:]) if math.dist(p,q)>1e-9]
                    t['items']=before+sp_items+post
                t['pts']=t['pts'][:i]+points+t['pts'][j+1:]
                t['w']=t['w'][:i]+[w]*(len(points)-1)+t['w'][j:]
                for name in ('nm_pts','nm_w'):t.pop(name,None)
                if sp_items is not None:
                    from common.track_items import validate as _bi_validate
                    try:_bi_validate(t)
                    except ValueError:continue
                report['tried_kicad_native']+=1
                nd=native_route_verify(board,candidate,options,folder/('candidate_%03d'%report['tried_kicad_native']))
                ne=native_skew_error(nd)
                oldscore=sum(max(0,q['delta_mm']-q['tolerance_mm']) for q in errors)
                newscore=sum(max(0,q['delta_mm']-q['tolerance_mm']) for q in ne)

                oldmap={(q['rule'],q['net']):q['delta_mm'] for q in errors}
                safe=all(q['delta_mm']<=oldmap.get((q['rule'],q['net']),q['tolerance_mm'])+1e-6 for q in ne)
                if allowed(drc,nd) and safe and newscore<oldscore-.005:
                    report['move_records'].append(dict(net_name=net,rule=error['rule'],layer=layer,shaping=shape,geometric_plus_mm=added,
                       before_skew_mm=error['delta_mm'],rest=[q for q in ne if q['rule']==error['rule']],evidence=str(folder/('candidate_%03d'%report['tried_kicad_native']))))
                    current=candidate;drc=nd;moved=True;native_folder=folder/('candidate_%03d'%report['tried_kicad_native'])
                    print('Skew: %s +%.4f mm, remainder %s'%(net,added,[q['delta_mm'] for q in ne if q['rule']==error['rule']]),flush=True)
                    break
            if moved:break
        if not moved:break
    report.update(post=native_skew_error(drc),out_port_sha=sha(current),
                  native_folder=str(native_folder),
                  drc_sha256=hashlib.sha256((native_folder/'drc.json').read_bytes()).hexdigest() if (native_folder/'drc.json').exists() else None,
                  pcb_sha256=hashlib.sha256((native_folder/'board.kicad_pcb').read_bytes()).hexdigest() if (native_folder/'board.kicad_pcb').exists() else None,
                  kicad_native_after=dict(faults=sum(v['severity']=='error' for v in drc['violations']),open_items=len(drc['unconnected_items'])))
    current['skew_matching']=report
    (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return current


