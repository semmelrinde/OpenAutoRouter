""
import math
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely.strtree import STRtree
from shapely import get_coordinates
from gemeinsam.verschieben import bewegen
from codex.adern import strom_baum as A
from codex.adern.vertrag import pruefe
from lernen import stromzonen as SZ
from lernen import stromzonen_topo as ST
from analyse.minimax import lastfehler


def positiv(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' muss endlich und > 0 sein')
    return value


def lage_board(B, lage):
    fps = {f['ref']: f for f in B['fps']}
    return bewegen(B, {r: (s[0]-fps[r]['x'], s[1]-fps[r]['y'], s[2]) for r,s in sorted(lage.items())},
                   seiten={r:s[3] for r,s in sorted(lage.items())})[0]


def baum(B, K, RG, lage, args):
    ""
    BB = lage_board(B, lage)
    opt = dict((RG or {}).get('router') or {})
    R = dict(tracks=[], vias=[], zonen=[], optionen=opt, lagen=B.get('lagen',2),
             gndlagen=[x for x in args.get('gndlagen','').split(',') if x])
    J = positiv(args.get('p35_j',40), 'p35_j')
    h = positiv(args.get('p35_raster',0.8), 'p35_raster')
    clr = positiv(opt.get('clr',0.15), 'clr')
    designs, infos = [], {}
    nets = SZ.stromnetze(K,set(K.get('gnd') or []))
    expected = []
    for n in sorted(nets):
        f = K['netze'][n].get('fluss') or {}
        if (K['netze'][n].get('strom_A') or f.get('gesamt') or 0) > 0: expected.append(n)
        try:
            err = lastfehler(K['netze'][n])
            if err: raise ValueError(str(err))
            sources, loads = ST._lasten(BB,K,n)
            if not loads:
                if n in expected: raise ValueError('positive Last ohne Quelle/Lastzuordnung')
                infos[n] = dict(status='keine_last'); continue


            relaxed = dict(BB, pads=[p for p in BB['pads'] if p['net']==n], footprint_copper=[])
            cloud = [p for p in relaxed['pads'] if p['ref']+':'+p['num'] in set(sources)|set(loads)]
            span = max(1., (max(p['x'] for p in cloud)-min(p['x'] for p in cloud)+6) *
                          (max(p['y'] for p in cloud)-min(p['y'] for p in cloud)+6))
            hh = max(h, math.sqrt(span * len(SZ._lagen(BB))/6000))
            G = A.graph(relaxed,R,K,n,h=hh,clr=clr, minwidth=float(opt.get('sigmin',0.15)))
            tree, info = A.steiner(G,J,rounds=2)
            d = A.corridor_json(G,tree,n,J)
            d.update(quellen_pads=sources, strom_herkunft=f.get('herkunft') or {})
            for c in d['korridore']:
                c.update(bedarf_mm=c['breite_mm']+2*c['abstand_mm'],beschnitten_mm2=0.,verbindung_ok=False,
                         endgeometrie_geprueft=False)
            for field in d['via_felder']:
                field.update(status='platz_fehlt',anzahl_gefunden=0,feld_geometrie_geprueft=False)
            designs.append(d); infos[n] = dict(info,raster_mm=hh,knoten=len(G.xyz))
        except (ValueError,RuntimeError) as e:
            infos[n] = dict(fehler=str(e))
    design = dict(version=A.VERSION,freigabe=False,einheiten=dict(koordinaten='mm',breite='mm',strom='A'),
                  verfahren='steiner',raster_mm=h,netze=designs,diagnose=infos,erwartete_lastnetze=expected,
                  verwendung='Placement-Schaetzung, entspannte Fremdpads; kein Routingzertifikat')
    errors = pruefe(design)
    if errors: raise ValueError('adern-v2: '+str(errors))
    return BB, design


def feld_form(mitte, count, diameter, drill, clr, h2h=.2):

    pitch = max(.5,diameter+clr+.005,drill+h2h+.005)
    nx = math.ceil(math.sqrt(count)); ny = math.ceil(count/nx)
    halfx = ((nx-1)*pitch+diameter)/2+clr
    halfy = ((ny-1)*pitch+diameter)/2+clr
    x,y = mitte
    return box(x-halfx,y-halfy,x+halfx,y+halfy),pitch


def freie_breite(a,b,bedarf,obstacles):
    ""
    length=math.dist(a,b)
    if length<1e-9:return bedarf
    ux,uy=(b[0]-a[0])/length,(b[1]-a[1])/length
    strip=LineString([a,b]).buffer(bedarf/2,cap_style=2)
    blocked=obstacles.intersection(strip)
    if blocked.is_empty:return bedarf
    events=sorted({0.,length,*[max(0.,min(length,(x-a[0])*ux+(y-a[1])*uy)) for x,y in get_coordinates(blocked)]})
    positions=[]
    for lo,hi in zip(events,events[1:]):
        if hi-lo<1e-10:continue
        eps=min((hi-lo)*1e-5,1e-7)
        positions.extend((lo+eps,(lo+hi)/2,hi-eps))
    width=bedarf
    for s in positions:
        x,y=a[0]+ux*s,a[1]+uy*s
        cut=LineString([(x-uy*bedarf/2,y+ux*bedarf/2),(x+uy*bedarf/2,y-ux*bedarf/2)]).difference(blocked)
        parts=list(getattr(cut,'geoms',[cut]))
        width=min(width,max((p.length for p in parts),default=0.))
        if width<1e-9:return 0.
    return width


def feld_frei(meta,obstacles,outline,holes=None,h2h=.2):
    mitte,count,diameter,drill,clr=meta
    _,pitch=feld_form(mitte,count,diameter,drill,clr,h2h)
    nx=math.ceil(math.sqrt(count));ny=math.ceil(count/nx);free=0
    for i in range(count):
        x=mitte[0]+(i%nx-(nx-1)/2)*pitch;y=mitte[1]+(i//nx-(ny-1)/2)*pitch
        spot=Point(x,y).buffer(diameter/2+clr)
        drill_space=Point(x,y).buffer(drill/2+h2h+.005)
        free+=(outline.covers(spot) and not obstacles.intersects(spot) and
               (holes is None or not holes.intersects(drill_space)))
    return free


class Karte:
    def __init__(self,B,K,RG,T,lage,args):
        self.B,self.K,self.T = B,K,T
        self.fps = {f['ref']:f for f in B['fps']}
        self.padidx = {r:[p for p in B['pads'] if p['ref']==r] for r in T}
        self.padlocal = {}
        self.outline = unary_union([Polygon(q).buffer(0) for q in B['outline']])
        self.BB,self.design = baum(B,K,RG,lage,args)
        self.reserve = [];self.capacity=[]
        self.terminals = set()
        self.clr = float(((RG or {}).get('router') or {}).get('clr',.15))
        self.h2h = float(((RG or {}).get('router') or {}).get('h2h',.2))
        self.pitch = float(((RG or {}).get('router') or {}).get('sigw',.15))+self.clr
        for d in self.design['netze']:
            n=d['netz']; terminalkeys=set(d['quellen_pads'])|set(d['lasten'])
            self.terminals.update(k.rsplit(':',1)[0] for k in terminalkeys)
            owners={p['ref'] for p in self.BB['pads'] if p['net']==n}
            for c in d['korridore']:
                g=LineString(c['mittellinie']).buffer(c['bedarf_mm']/2,cap_style=2,join_style=2)
                if g.area>1e-10:
                    self.reserve.append((n,c['lage'],g,owners,'kante'))
                    self.capacity.append((c['mittellinie'],c['bedarf_mm'],None))
            for f in d['via_felder']:
                g,pitch=feld_form(f['mitte'],f['anzahl_soll'],f['via_d_mm'],f['drill_mm'],self.clr,self.h2h)
                f['reservierung_raster_mm']=pitch
                self.reserve.append((n,'FB',g,owners,'via'))
                self.capacity.append((None,None,(f['mitte'],f['anzahl_soll'],f['via_d_mm'],f['drill_mm'],self.clr)))

            fps={f['ref']:f for f in self.BB['fps']}
            for p in self.BB['pads']:
                key=p['ref']+':'+p['num']
                if key not in terminalkeys or not any(s in fps[p['ref']]['fp'].upper() for s in ('BGA','QFN','DFN')):continue
                I=(d['lasten'].get(key) or {}).get('strom_A',sum(v['strom_A'] for v in d['lasten'].values())/max(1,len(d['quellen_pads'])))
                count=int(A.k10_soll(I,.2)); g,_=feld_form([p['x'],p['y']],count,.45,.2,self.clr,self.h2h)
                self.reserve.append((n,'FB',g,{p['ref']},'power_pin'))
                self.capacity.append((None,None,([p['x'],p['y']],count,.45,.2,self.clr)))
        self.tree=STRtree([r[2] for r in self.reserve])
        self.cache={}
        self.netkeys=sorted({r[0] for r in self.reserve})
        expected=set(self.design['erwartete_lastnetze'])
        self.modell_fehlend=sorted(expected-set(self.netkeys))
        self.modell_fehler={n:d['fehler'] for n,d in self.design['diagnose'].items() if d.get('fehler')}
        self.fanouts=[]
        fps={f['ref']:f for f in self.BB['fps']}
        for ref,t in sorted(T.items()):
            if t.n_pads<8:continue
            signal=[p for p in t.pads_abs(*lage[ref]) if K['netze'].get(p[1],{}).get('art') in ('signal','schnell','analog')]
            if not signal:continue
            sd=lage[ref][3]; g=t.geo(*lage[ref])[sd]; x0,y0,x1,y1=g.bounds
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

    def pad_hindernis(self,ref,state,layer,ownnet=None,tht_only=False,holes_only=False):
        from shapely import affinity
        f=self.fps[ref]
        flip=state[3]!=f['side']
        src_layer=layer
        if flip and layer.startswith('In'):src_layer='In%d'%(self.B.get('lagen',2)-1-int(layer[2:]))
        if flip and layer in ('F','B'):src_layer={'F':'B','B':'F'}[layer]
        key=(ref,src_layer,ownnet,tht_only,holes_only)
        if key not in self.padlocal:
            polys=[]
            for p in self.padidx[ref]:
                if tht_only and not p.get('drill'):continue

                if not holes_only and not (ownnet is not None and p['net']==ownnet):
                    qs=(p.get('polys') or {}).get(src_layer,[]) if layer!='FB' else [q for v in p.get('polys',{}).values() for q in v]
                    polys.extend(Polygon(q) for q in qs)
                holes=[Polygon(q) for q in p.get('hole_polys',[]) if len(q)>=3]
                if not holes and p.get('drill',0)>0:holes=[Point(p['x'],p['y']).buffer(p['drill']/2)]
                polys.extend(holes)
            self.padlocal[key]=affinity.translate(unary_union(polys),-f['x'],-f['y'])
        co,si=math.cos(math.radians(state[2])),math.sin(math.radians(state[2]))
        return affinity.affine_transform(self.padlocal[key],[co*(-1 if flip else 1),-si,si*(-1 if flip else 1),co,*state[:2]])

    def hindernis(self,ref,state,layer):
        gs=self.T[ref].geo(*state)
        if layer=='FB':return unary_union(list(gs.values()))
        if layer in ('F','B'):return gs.get(layer,Polygon())

        return self.pad_hindernis(ref,state,layer,tht_only=True)

    def kosten(self,ch):
        total=0.
        for ref,state in ch.items():
            cache_key=(ref,*state)
            if cache_key not in self.cache:
                gs=self.T[ref].geo(*state);g=unary_union(list(gs.values())); val=0.; obstacles={}
                holes=self.pad_hindernis(ref,state,'FB',holes_only=True)
                for i in self.tree.query(g.union(holes).buffer(self.h2h+.005)):
                    n,L,res,owners,kind=self.reserve[i]
                    okey=(L,n if ref in owners else None)
                    if okey not in obstacles:obstacles[okey]=self.pad_hindernis(ref,state,L,ownnet=n) if ref in owners else self.hindernis(ref,state,L)
                    obstacle=obstacles[okey]
                    hit=res.intersection(obstacle).area
                    blockage=hit/max(res.area,1e-9)
                    line,width,field=self.capacity[i]
                    if hit>1e-10 and line is not None: blockage=max(blockage,1-freie_breite(*line,width,obstacle)/width)
                    if field is not None:
                        baseline=feld_frei(field,Polygon(),self.outline,h2h=self.h2h)
                        blockage=max(blockage,(baseline-feld_frei(field,obstacle,self.outline,holes,self.h2h))/field[1])
                    val += blockage * (3 if kind!='kante' else 1)
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

    def messen(self,lage):
        geos={r:self.T[r].geo(*s) for r,s in lage.items()}
        blocked={n:[] for n in self.netkeys}; narrow={n:[] for n in self.netkeys}; obstacle_cache={}; layers={r[1] for r in self.reserve}
        obstacle_geos={(r,L):self.hindernis(r,s,L) for r,s in lage.items() for L in layers}
        holes=unary_union([self.pad_hindernis(r,s,'FB',holes_only=True) for r,s in lage.items()])
        for i,(n,L,res,owners,kind) in enumerate(self.reserve):
            key=(n,L,tuple(sorted(owners)))
            if key not in obstacle_cache:
                obstacle_cache[key]=unary_union([obstacle_geos[r,L] for r in lage if r not in owners]+
                    [self.pad_hindernis(r,lage[r],L,ownnet=n) for r in owners if r in lage])
            hit=res.intersection(obstacle_cache[key]).area/res.area
            blocked[n].append(hit)
            line,width,field=self.capacity[i]
            narrow[n].append(1-freie_breite(*line,width,obstacle_cache[key])/width if line is not None else
                             1-feld_frei(field,obstacle_cache[key],self.outline,holes,self.h2h)/field[1])
        power={n:dict(max=max(narrow[n] or [0.]),summe=sum(v),flaeche_max=max(v or [0.])) for n,v in blocked.items()}
        fan={}
        for ref,L,side,demand,cuts in self.fanouts:
            foreign=unary_union([g.get(L,Polygon()).buffer(self.pitch/2) for r,g in geos.items() if r!=ref])
            copper=unary_union([g for n,l,g,owners,kind in self.reserve if l in (L,'FB')])
            widths=[cut.difference(foreign.union(copper)).length for cut in cuts]
            capacity=math.floor(min(widths)/self.pitch)
            fan[f'{ref}:{L}:{side}']=dict(bedarf=demand,kapazitaet=capacity,defizit=max(0,demand-capacity))
        signals={}
        pts={}
        for r,s in lage.items():
            for _,n,x,y,_ in self.T[r].pads_abs(*s):
                if self.K['netze'].get(n,{}).get('art') in ('signal','schnell','analog'):pts.setdefault(n,[]).append((x,y))
        for n,ps in pts.items():signals[n]=max(x for x,y in ps)-min(x for x,y in ps)+max(y for x,y in ps)-min(y for x,y in ps)
        return dict(strom=power,fanout=fan,signal_hpwl=signals,
                    modell_vollstaendig=not self.modell_fehlend and not self.modell_fehler)


def nicht_schlechter(a,b):
    if not a.get('modell_vollstaendig',True) or not b.get('modell_vollstaendig',True):return False
    if any(set(a[k])!=set(b[k]) for k in ('strom','fanout','signal_hpwl')):return False
    return (all(b['strom'][n][k]<=v+1e-9 for n,vals in a['strom'].items() for k,v in vals.items()) and
            all(b['fanout'][n]['defizit']<=v['defizit'] and b['fanout'][n]['kapazitaet']>=v['kapazitaet'] for n,v in a['fanout'].items()) and
            all(b['signal_hpwl'][n]<=v+1e-9 for n,v in a['signal_hpwl'].items()))


def rang(m):
    return tuple(sorted((v['max'] for v in m['strom'].values()),reverse=True)),sum(v['summe'] for v in m['strom'].values())


def lokal(karte,lage,beweglich,drehbar,legal,pinkosten,maxteile=24):
    before=karte.messen(lage); current=before; moves=[]; trials=0
    if not before['modell_vollstaendig']:
        return lage,dict(vorher=before,nachher=before,zuege=[],versuche=0,surrogate=True,
                         modell_fehlend=karte.modell_fehlend,modell_fehler=karte.modell_fehler,
                         grenze='Zugannahme gesperrt: unvollstaendiges Strommodell')

    refs=[r for r in beweglich if r.startswith(('C','R')) and r not in karte.terminals and karte.kosten({r:lage[r]})>0]
    refs=sorted(refs,key=lambda r:(-karte.kosten({r:lage[r]}),r))[:maxteile]
    for r in refs:
        base=lage[r]; x,y,th,sd=base
        choices=[(x+dx,y+dy,th,sd) for step in (.25,.5,1.) for dx,dy in ((step,0),(-step,0),(0,step),(0,-step))]
        if drehbar(r):choices += [(x,y,(th+rot)%360,sd) for rot in (90,180,270)]
        best=None
        for st in choices:
            candidate=dict(lage,**{r:st}); trials+=1
            if karte.kosten({r:st})>=karte.kosten({r:base})-1e-9:continue
            candidate=legal(candidate)
            if candidate is None:continue
            if any(candidate[r]!=lage[r] for r in karte.terminals):continue
            m=karte.messen(candidate)
            if not nicht_schlechter(current,m) or rang(m)>=rang(current):continue


            if any(pinkosten(q,candidate[q],candidate)>pinkosten(q,lage[q],lage)+1e-9 for q in sorted(lage)):continue
            if best is None or rang(m)<rang(best[1]):best=(candidate,m)
        if best:
            lage,current=best; moves.append(dict(ref=r,vorher=list(base),nachher=list(lage[r])))
    return lage,dict(vorher=before,nachher=current,zuege=moves,versuche=trials,
                     surrogate=True,grenze='HPWL und Seitenkapazitaet sind kein Signal-Routingnachweis')
