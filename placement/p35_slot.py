import math
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely.strtree import STRtree
from shapely import get_coordinates
from common.shift import relocate
from codex.veins import current_tree as A
from codex.veins.contract_spec import check
from learning import current_zones as SZ
from learning import current_zones_topo as ST
from analysis.minimax import load_error


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and > 0')
    return value


def layer_board(B, layer_name):
    fps = {f['ref']: f for f in B['fps']}
    return relocate(B, {r: (s[0]-fps[r]['x'], s[1]-fps[r]['y'], s[2]) for r,s in sorted(layer_name.items())},
                   sides={r:s[3] for r,s in sorted(layer_name.items())})[0]


def tree_idx(B, K, RG, layer_name, args):
    BB = layer_board(B, layer_name)
    opt = dict((RG or {}).get('router') or {})
    R = dict(tracks=[], vias=[], zone_list=[], options=opt, cu_layers=B.get('cu_layers',2),
             gnd_layers=[x for x in args.get('gnd_layers','').split(',') if x])
    J = positive(args.get('p35_j',40), 'p35_j')
    h = positive(args.get('p35_raster',0.8), 'p35_raster')
    clr = positive(opt.get('clr',0.15), 'clr')
    designs, infos = [], {}
    nets = SZ.current_nets(K,set(K.get('gnd') or []))
    expected = []
    for n in sorted(nets):
        f = K['net_list'][n].get('flow') or {}
        if (K['net_list'][n].get('current_A') or f.get('total') or 0) > 0: expected.append(n)
        try:
            err = load_error(K['net_list'][n])
            if err: raise ValueError(str(err))
            sources, loads = ST._loads(BB,K,n)
            if not loads:
                if n in expected: raise ValueError('positive load without source/load assignment')
                infos[n] = dict(status='no_load'); continue


            relaxed = dict(BB, pads=[p for p in BB['pads'] if p['net']==n], footprint_copper=[])
            cloud = [p for p in relaxed['pads'] if p['ref']+':'+p['num'] in set(sources)|set(loads)]
            span = max(1., (max(p['x'] for p in cloud)-min(p['x'] for p in cloud)+6) *
                          (max(p['y'] for p in cloud)-min(p['y'] for p in cloud)+6))
            hh = max(h, math.sqrt(span * len(SZ._layers(BB))/6000))
            G = A.graph(relaxed,R,K,n,h=hh,clr=clr, minwidth=float(opt.get('sigmin',0.15)))
            tree, info = A.steiner(G,J,rounds=2)
            d = A.corridor_json(G,tree,n,J)
            d.update(sources_pads=sources, current_provenance=f.get('provenance') or {})
            for c in d['corridors']:
                c.update(demand_mm=c['width_mm']+2*c['spacing_mm'],trimmed_mm2=0.,connection_ok=False,
                         end_geometry_checked=False)
            for field in d['via_fields']:
                field.update(status='slot_missing',count_located=0,field_geometry_checked=False)
            designs.append(d); infos[n] = dict(info,raster_mm=hh,nodes=len(G.xyz))
        except (ValueError,RuntimeError) as e:
            infos[n] = dict(faults=str(e))
    design = dict(version=A.VERSION,release=False,unit_list=dict(coordinates='mm',width='mm',current_val='A'),
                  method='steiner',raster_mm=h,net_list=designs,diagnose=infos,expected_load_nets=expected,
                  usage='Placement estimate, relaxed foreign pads; no routing certificate')
    errors = check(design)
    if errors: raise ValueError('veins-v2: '+str(errors))
    return BB, design


def field_shape(middle, count, diameter, drill, clr, h2h=.2):

    pitch = max(.5,diameter+clr+.005,drill+h2h+.005)
    nx = math.ceil(math.sqrt(count)); ny = math.ceil(count/nx)
    halfx = ((nx-1)*pitch+diameter)/2+clr
    halfy = ((ny-1)*pitch+diameter)/2+clr
    x,y = middle
    return box(x-halfx,y-halfy,x+halfx,y+halfy),pitch


def free_width(a,b,demand,obstacles):
    length=math.dist(a,b)
    if length<1e-9:return demand
    ux,uy=(b[0]-a[0])/length,(b[1]-a[1])/length
    strip=LineString([a,b]).buffer(demand/2,cap_style=2)
    blocked=obstacles.intersection(strip)
    if blocked.is_empty:return demand
    events=sorted({0.,length,*[max(0.,min(length,(x-a[0])*ux+(y-a[1])*uy)) for x,y in get_coordinates(blocked)]})
    positions=[]
    for lo,hi in zip(events,events[1:]):
        if hi-lo<1e-10:continue
        eps=min((hi-lo)*1e-5,1e-7)
        positions.extend((lo+eps,(lo+hi)/2,hi-eps))
    width=demand
    for s in positions:
        x,y=a[0]+ux*s,a[1]+uy*s
        cut=LineString([(x-uy*demand/2,y+ux*demand/2),(x+uy*demand/2,y-ux*demand/2)]).difference(blocked)
        parts=list(getattr(cut,'geoms',[cut]))
        width=min(width,max((p.length for p in parts),default=0.))
        if width<1e-9:return 0.
    return width


def field_free(meta,obstacles,outline,holes=None,h2h=.2):
    middle,count,diameter,drill,clr=meta
    _,pitch=field_shape(middle,count,diameter,drill,clr,h2h)
    nx=math.ceil(math.sqrt(count));ny=math.ceil(count/nx);free=0
    for i in range(count):
        x=middle[0]+(i%nx-(nx-1)/2)*pitch;y=middle[1]+(i//nx-(ny-1)/2)*pitch
        spot=Point(x,y).buffer(diameter/2+clr)
        drill_space=Point(x,y).buffer(drill/2+h2h+.005)
        free+=(outline.covers(spot) and not obstacles.intersects(spot) and
               (holes is None or not holes.intersects(drill_space)))
    return free


class Map:
    def __init__(self,B,K,RG,T,layer_name,args):
        self.B,self.K,self.T = B,K,T
        self.fps = {f['ref']:f for f in B['fps']}
        self.pad_idx = {r:[p for p in B['pads'] if p['ref']==r] for r in T}
        self.pad_local = {}
        self.outline = unary_union([Polygon(q).buffer(0) for q in B['outline']])
        self.BB,self.design = tree_idx(B,K,RG,layer_name,args)
        self.reserve = [];self.capacity=[]
        self.terminals = set()
        self.clr = float(((RG or {}).get('router') or {}).get('clr',.15))
        self.h2h = float(((RG or {}).get('router') or {}).get('h2h',.2))
        self.pitch = float(((RG or {}).get('router') or {}).get('sigw',.15))+self.clr
        for d in self.design['net_list']:
            n=d['net_name']; terminalkeys=set(d['sources_pads'])|set(d['load_list'])
            self.terminals.update(k.rsplit(':',1)[0] for k in terminalkeys)
            owners={p['ref'] for p in self.BB['pads'] if p['net']==n}
            for c in d['corridors']:
                g=LineString(c['centerline']).buffer(c['demand_mm']/2,cap_style=2,join_style=2)
                if g.area>1e-10:
                    self.reserve.append((n,c['layer_name'],g,owners,'edge'))
                    self.capacity.append((c['centerline'],c['demand_mm'],None))
            for f in d['via_fields']:
                g,pitch=field_shape(f['middle'],f['count_nominal'],f['via_d_mm'],f['drill_mm'],self.clr,self.h2h)
                f['reservation_raster_mm']=pitch
                self.reserve.append((n,'FB',g,owners,'via'))
                self.capacity.append((None,None,(f['middle'],f['count_nominal'],f['via_d_mm'],f['drill_mm'],self.clr)))

            fps={f['ref']:f for f in self.BB['fps']}
            for p in self.BB['pads']:
                key=p['ref']+':'+p['num']
                if key not in terminalkeys or not any(s in fps[p['ref']]['fp'].upper() for s in ('BGA','QFN','DFN')):continue
                I=(d['load_list'].get(key) or {}).get('current_A',sum(v['current_A'] for v in d['load_list'].values())/max(1,len(d['sources_pads'])))
                count=int(A.k10_nominal(I,.2)); g,_=field_shape([p['x'],p['y']],count,.45,.2,self.clr,self.h2h)
                self.reserve.append((n,'FB',g,{p['ref']},'power_pin'))
                self.capacity.append((None,None,([p['x'],p['y']],count,.45,.2,self.clr)))
        self.tree=STRtree([r[2] for r in self.reserve])
        self.cache={}
        self.netkeys=sorted({r[0] for r in self.reserve})
        expected=set(self.design['expected_load_nets'])
        self.model_missing=sorted(expected-set(self.netkeys))
        self.model_error={n:d['faults'] for n,d in self.design['diagnose'].items() if d.get('faults')}
        self.fanouts=[]
        fps={f['ref']:f for f in self.BB['fps']}
        for ref,t in sorted(T.items()):
            if t.n_pads<8:continue
            signal=[p for p in t.pads_abs(*layer_name[ref]) if K['net_list'].get(p[1],{}).get('category') in ('signal','fast','analog')]
            if not signal:continue
            sd=layer_name[ref][3]; g=t.geo(*layer_name[ref])[sd]; x0,y0,x1,y1=g.bounds
            demand=[0]*4
            for _,_,x,y,_ in signal:
                side=min(range(4),key=lambda i:[abs(x-x0),abs(x-x1),abs(y-y0),abs(y-y1)][i]);demand[side]+=1

            for side in range(4):
                cuts=[]
                for off in (.3,.7,1.2):
                    cuts.append(LineString([(x0-off,y0),(x0-off,y1)]) if side==0 else
                                LineString([(x1+off,y0),(x1+off,y1)]) if side==1 else
                                LineString([(x0,y0-off),(x1,y0-off)]) if side==2 else
                                LineString([(x0,y1+off),(x1,y1+off)]))
                self.fanouts.append((ref,sd,side,demand[side],[cut.intersection(self.outline) for cut in cuts]))
        self.fancuts=[(ref,L,side,demand,cut) for ref,L,side,demand,cuts in self.fanouts for cut in cuts if demand and not cut.is_empty]
        self.fantree=STRtree([x[-1] for x in self.fancuts])

    def pad_obstacle(self,ref,state,layer,ownnet=None,tht_only=False,holes_only=False):
        from shapely import affinity
        f=self.fps[ref]
        flip=state[3]!=f['side']
        src_layer=layer
        if flip and layer.startswith('In'):src_layer='In%d'%(self.B.get('cu_layers',2)-1-int(layer[2:]))
        if flip and layer in ('F','B'):src_layer={'F':'B','B':'F'}[layer]
        key=(ref,src_layer,ownnet,tht_only,holes_only)
        if key not in self.pad_local:
            polys=[]
            for p in self.pad_idx[ref]:
                if tht_only and not p.get('drill'):continue

                if not holes_only and not (ownnet is not None and p['net']==ownnet):
                    qs=(p.get('polys') or {}).get(src_layer,[]) if layer!='FB' else [q for v in p.get('polys',{}).values() for q in v]
                    polys.extend(Polygon(q) for q in qs)
                holes=[Polygon(q) for q in p.get('hole_polys',[]) if len(q)>=3]
                if not holes and p.get('drill',0)>0:holes=[Point(p['x'],p['y']).buffer(p['drill']/2)]
                polys.extend(holes)
            self.pad_local[key]=affinity.translate(unary_union(polys),-f['x'],-f['y'])
        co,si=math.cos(math.radians(state[2])),math.sin(math.radians(state[2]))
        return affinity.affine_transform(self.pad_local[key],[co*(-1 if flip else 1),-si,si*(-1 if flip else 1),co,*state[:2]])

    def obstacle(self,ref,state,layer):
        gs=self.T[ref].geo(*state)
        if layer=='FB':return unary_union(list(gs.values()))
        if layer in ('F','B'):return gs.get(layer,Polygon())

        return self.pad_obstacle(ref,state,layer,tht_only=True)

    def cost(self,ch):
        total=0.
        for ref,state in ch.items():
            cache_key=(ref,*state)
            if cache_key not in self.cache:
                gs=self.T[ref].geo(*state);g=unary_union(list(gs.values())); val=0.; obstacles={}
                holes=self.pad_obstacle(ref,state,'FB',holes_only=True)
                for i in self.tree.query(g.union(holes).buffer(self.h2h+.005)):
                    n,L,res,owners,kind=self.reserve[i]
                    okey=(L,n if ref in owners else None)
                    if okey not in obstacles:obstacles[okey]=self.pad_obstacle(ref,state,L,ownnet=n) if ref in owners else self.obstacle(ref,state,L)
                    obstacle=obstacles[okey]
                    hit=res.intersection(obstacle).area
                    blockage=hit/max(res.area,1e-9)
                    line,width,field=self.capacity[i]
                    if hit>1e-10 and line is not None: blockage=max(blockage,1-free_width(*line,width,obstacle)/width)
                    if field is not None:
                        baseline=field_free(field,Polygon(),self.outline,h2h=self.h2h)
                        blockage=max(blockage,(baseline-field_free(field,obstacle,self.outline,holes,self.h2h))/field[1])
                    val += blockage * (3 if kind!='edge' else 1)
                obstacles={L:g.buffer(self.pitch/2) for L,g in gs.items()};fancost={}
                for i in self.fantree.query(g.buffer(self.pitch/2)):
                    anchor,L,side,demand,cut=self.fancuts[i]
                    if anchor==ref or L not in gs:continue
                    fanout_key=(anchor,L,side);v=2*demand*cut.intersection(obstacles[L]).length/max(cut.length,self.pitch)
                    fancost[fanout_key]=max(fancost.get(fanout_key,0.),v)
                val+=sum(fancost.values())
                if len(self.cache)>4096:self.cache.clear()
                self.cache[cache_key]=val
            total+=self.cache[cache_key]
        return total

    def measure(self,layer_name):
        geos={r:self.T[r].geo(*s) for r,s in layer_name.items()}
        blocked={n:[] for n in self.netkeys}; narrow={n:[] for n in self.netkeys}; obstacle_cache={}; layers={r[1] for r in self.reserve}
        obstacle_geos={(r,L):self.obstacle(r,s,L) for r,s in layer_name.items() for L in layers}
        holes=unary_union([self.pad_obstacle(r,s,'FB',holes_only=True) for r,s in layer_name.items()])
        for i,(n,L,res,owners,kind) in enumerate(self.reserve):
            key=(n,L,tuple(sorted(owners)))
            if key not in obstacle_cache:
                obstacle_cache[key]=unary_union([obstacle_geos[r,L] for r in layer_name if r not in owners]+
                    [self.pad_obstacle(r,layer_name[r],L,ownnet=n) for r in owners if r in layer_name])
            hit=res.intersection(obstacle_cache[key]).area/res.area
            blocked[n].append(hit)
            line,width,field=self.capacity[i]
            narrow[n].append(1-free_width(*line,width,obstacle_cache[key])/width if line is not None else
                             1-field_free(field,obstacle_cache[key],self.outline,holes,self.h2h)/field[1])
        power={n:dict(max=max(narrow[n] or [0.]),total_sum=sum(v),area_max=max(v or [0.])) for n,v in blocked.items()}
        fan={}
        for ref,L,side,demand,cuts in self.fanouts:
            foreign=unary_union([g.get(L,Polygon()).buffer(self.pitch/2) for r,g in geos.items() if r!=ref])
            copper=unary_union([g for n,l,g,owners,kind in self.reserve if l in (L,'FB')])
            widths=[cut.difference(foreign.union(copper)).length for cut in cuts]
            capacity=math.floor(min(widths)/self.pitch)
            fan[f'{ref}:{L}:{side}']=dict(demand=demand,capacity=capacity,deficit=max(0,demand-capacity))
        signals={}
        pts={}
        for r,s in layer_name.items():
            for _,n,x,y,_ in self.T[r].pads_abs(*s):
                if self.K['net_list'].get(n,{}).get('category') in ('signal','fast','analog'):pts.setdefault(n,[]).append((x,y))
        for n,ps in pts.items():signals[n]=max(x for x,y in ps)-min(x for x,y in ps)+max(y for x,y in ps)-min(y for x,y in ps)
        return dict(current_val=power,fanout=fan,signal_hpwl=signals,
                    model_complete=not self.model_missing and not self.model_error)


def not_worse(a,b):
    if not a.get('model_complete',True) or not b.get('model_complete',True):return False
    if any(set(a[k])!=set(b[k]) for k in ('current_val','fanout','signal_hpwl')):return False
    return (all(b['current_val'][n][k]<=v+1e-9 for n,vals in a['current_val'].items() for k,v in vals.items()) and
            all(b['fanout'][n]['deficit']<=v['deficit'] and b['fanout'][n]['capacity']>=v['capacity'] for n,v in a['fanout'].items()) and
            all(b['signal_hpwl'][n]<=v+1e-9 for n,v in a['signal_hpwl'].items()))


def rank(m):
    return tuple(sorted((v['max'] for v in m['current_val'].values()),reverse=True)),sum(v['total_sum'] for v in m['current_val'].values())


def local(grid_map,layer_name,movable,rotatable,legal,pin_cost,max_parts=24):
    before=grid_map.measure(layer_name); current=before; moves=[]; trials=0
    if not before['model_complete']:
        return layer_name,dict(before=before,after=before,move_records=[],attempts=0,surrogate=True,
                         model_missing=grid_map.model_missing,model_error=grid_map.model_error,
                         limit='Move acceptance locked: incomplete current model')

    refs=[r for r in movable if r.startswith(('C','R')) and r not in grid_map.terminals and grid_map.cost({r:layer_name[r]})>0]
    refs=sorted(refs,key=lambda r:(-grid_map.cost({r:layer_name[r]}),r))[:max_parts]
    for r in refs:
        base=layer_name[r]; x,y,th,sd=base
        choices=[(x+dx,y+dy,th,sd) for step in (.25,.5,1.) for dx,dy in ((step,0),(-step,0),(0,step),(0,-step))]
        if rotatable(r):choices += [(x,y,(th+rot)%360,sd) for rot in (90,180,270)]
        best=None
        for st in choices:
            candidate=dict(layer_name,**{r:st}); trials+=1
            if grid_map.cost({r:st})>=grid_map.cost({r:base})-1e-9:continue
            candidate=legal(candidate)
            if candidate is None:continue
            if any(candidate[r]!=layer_name[r] for r in grid_map.terminals):continue
            m=grid_map.measure(candidate)
            if not not_worse(current,m) or rank(m)>=rank(current):continue


            if any(pin_cost(q,candidate[q],candidate)>pin_cost(q,layer_name[q],layer_name)+1e-9 for q in sorted(layer_name)):continue
            if best is None or rank(m)<rank(best[1]):best=(candidate,m)
        if best:
            layer_name,current=best; moves.append(dict(ref=r,before=list(base),after=list(layer_name[r])))
    return layer_name,dict(before=before,after=current,move_records=moves,attempts=trials,
                     surrogate=True,limit='HPWL and side capacity are no signal routing evidence')
