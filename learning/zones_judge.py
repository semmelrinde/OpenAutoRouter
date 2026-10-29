import gzip
import hashlib
import inspect
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
VERSION = 'zone_list-judge-v1'
GALLERY = Path.home()/'Library/Caches/Autorouter/gallery'
MODEL = ROOT/'learning/model/zones_judge/v2/model.json'
from learning.zones_appraisal import ERROR
KEYS = sorted(ERROR)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def zone_list(R):
    from learning.current_zones import _geo, _polys
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    groups = {}
    for z in R.get('zone_list', []):
        if z.get('source') == 'current_zones' and z.get('polys'):
            groups.setdefault((z['net'], z['layer']), []).append(_geo(z))
    for z in R.get('zones', []):
        if z.get('filled') and z.get('zones_source', z.get('source')) == 'current_zones':
            groups.setdefault((z['net'], z['layer']), []).append(Polygon(z['pts'], z.get('holes') or []))
    return [dict(net=n, layer=l, polys=_polys(unary_union(gs))) for (n,l),gs in sorted(groups.items())]


def diagnose(B, R):
    from learning.zones_judge_selection import an
    if not an(R, (R.get('options') or {}).get('zones_judge_diagnose', 0)):
        return dict(status='off', zones_judge_score=None, zone_list=[])
    try:
        return Service((R.get('options') or {}).get('zones_judge_model')).measure(B, R)
    except Exception as e:
        return dict(status='faults', zones_judge_score=None, faults=repr(e))


def feats(B, R, z):
    import numpy as np
    import shapely
    from scipy.ndimage import distance_transform_edt, maximum_filter
    from shapely.geometry import LineString, Point, Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
    from learning.current_zones import _geo, _parts
    g = _geo(z)
    if g.is_empty or not g.is_valid or g.area <= 0: raise ValueError('Empty/invalid zone')
    n,l = z['net'],z['layer']; A,P = g.area,g.length
    parts = [p for p in getattr(g,'geoms',[g]) if p.geom_type == 'Polygon']
    out = dict(surface=A, perimeter=P, part_list=len(parts), hole_list=sum(len(p.interiors) for p in parts),
        splitter_share=sum(p.area for p in parts if p.area < .5)/A,
        largest_part=max(p.area for p in parts)/A, compact=4*math.pi*A/P**2,
        convex=A/max(g.convex_hull.area,1e-9))
    xy = np.asarray(g.convex_hull.exterior.coords[:-1])
    own_set, axes = np.linalg.eigh(np.cov(xy.T))
    extent = np.ptp(xy @ axes, axis=0)
    out['stretch_proxy'] = float(max(extent)/max(min(extent),1e-9))
    for tol in (.01,.05):
        s=g.simplify(tol,preserve_topology=True); turns=[]; lengths=[]; signed=[]
        for p in getattr(s,'geoms',[s]):
            if p.geom_type!='Polygon': continue
            p=orient(p,1)
            for ring in [p.exterior,*p.interiors]:
                xy=np.asarray(ring.coords[:-1]); before=xy-np.roll(xy,1,axis=0); after=np.roll(xy,-1,axis=0)-xy
                length=np.linalg.norm(after,axis=1)
                angles=np.arctan2(before[:,0]*after[:,1]-before[:,1]*after[:,0], np.sum(before*after,axis=1))
                turns.extend(np.abs(angles)); signed.extend(angles); lengths.extend(length)
        t=np.asarray(turns); lengths=np.asarray(lengths)
        prefix='border_%s_'%tol
        out.update({prefix+'corners_mm':len(t)/P, prefix+'rotation_mm':float(t.sum()/P),
            prefix+'notches_mm':sum(v < -.15 for v in signed)/P,
            prefix+'sharp_mm':float(np.sum(t>math.pi*.67)/P),
            prefix+'short_edges':float(np.mean(lengths<.1)) if len(lengths) else 0,
            prefix+'short_perimeter':float(lengths[lengths<.1].sum()/P),
            prefix+'rough_area':g.symmetric_difference(s).area/A,
            prefix+'rough_perimeter':max(0,1-s.length/P)})
        for lo,hi in ((0,15),(15,45),(45,90),(90,180)):
            out[prefix+'angle_%d_%d'%(lo,hi)]=float(np.mean((t>=math.radians(lo))&(t<math.radians(hi)))) if len(t) else 0
    smooth=g.buffer(.1,8).buffer(-.1,8)
    out['rough_smoothed']=g.symmetric_difference(smooth).area/A
    for r in (.1,.2,.3,.5,.75,1.,1.5,2.):out['erosion_%s'%r]=g.buffer(-r,8).area/A

    x0,y0,x1,y1=g.bounds; h=max(.03,max(x1-x0,y1-y0)/240)
    xs=np.arange(x0-h,x1+2*h,h);ys=np.arange(y0-h,y1+2*h,h)
    xx,yy=np.meshgrid(xs,ys); mask=shapely.contains_xy(g,xx,yy)
    d=distance_transform_edt(mask)*h; ridge=mask&(d>=maximum_filter(d,size=3)-1e-10)
    widths=2*d[ridge]; q=np.quantile(widths,[.1,.5,.9]) if len(widths) else np.zeros(3)
    out.update(width_p10=float(q[0]),width_median=float(q[1]),width_p90=float(q[2]),
        width_cv=float(np.std(widths)/max(np.mean(widths),1e-9)) if len(widths) else 0,
        narrow_share=float(np.mean(widths<max(.2,q[1]*.5))) if len(widths) else 0,raster_mm=h)
    jumps=[]
    for axis in (0,1):
        a=np.take(d,range(d.shape[axis]-1),axis=axis);b=np.take(d,range(1,d.shape[axis]),axis=axis)
        both=np.take(ridge,range(d.shape[axis]-1),axis=axis)&np.take(ridge,range(1,d.shape[axis]),axis=axis)
        jumps.extend((2*np.abs(a-b)[both]).tolist())
    out['width_jump_proxy']=max(jumps,default=0)
    pads=[Polygon(q).buffer(0) for p in B.get('pads',[]) if p.get('net')==n for q in (p.get('polys') or {}).get(l,[]) if len(q)>=3]
    near=[p for p in pads if p.distance(g)<1.]
    out['pads_close']=len(near)
    out['pad_coverage']=sum(g.intersection(p).area/max(p.area,1e-9) for p in near)/max(1,len(near))
    paths=[LineString([a,b]) for t in R.get('tracks',[]) if t['net']==n and t['layer']==l
        for a,b in zip(t['pts'][:-1],t['pts'][1:]) if a!=b]
    envelope=unary_union(pads+[p.buffer(max(.25,q[1]/2),8) for p in paths])
    out['without_path_proxy']=g.difference(envelope).area/A
    holes=[Point(v['x'],v['y']).buffer(float(v.get('diameter',v.get('d',.45)))/2+.16,16)
        for v in R.get('vias',[]) if v['net']!=n and l in (v.get('layers') or ['F','B']+['In%d'%i for i in range(1,int(B.get('cu_layers',2))-1)])]
    out['via_bites_mm']=sum(g.boundary.intersection(p).length for p in holes)/P
    foreign=unary_union([p for m,p in _parts(B,R,l) if m!=n])
    outline=unary_union([Polygon(o).buffer(0) for o in B.get('outline',[]) if len(o)>=3])
    free=g.buffer(.5,8).difference(g).intersection(outline.buffer(-.25)).difference(foreign.buffer(.15,8))
    out['free_beside']=free.area/A
    if any(not math.isfinite(v) for v in out.values()):raise ValueError('Non-finite shape feature')
    return out


def feature_contract():
    return sha(inspect.getsource(feats).encode())


def family(w):


    if w.get('family'):return str(w['family'])
    match=re.fullmatch(r'erf_(\d+)(?:_f[\d.]+)?',str(w.get('board','')))
    if match:return 'task_%d'%(int(match[1])//5)
    if w.get('source_meta'):
        from learning.families import source, family as f
        if source(w['source_meta']):return f(w)
    from learning.families import aliases
    registered=aliases().get(str(w.get('board','')).lower().removesuffix('_tight'))
    if registered:return registered
    raise ValueError('Source family missing: '+str(w.get('board')))


def data_read(path,gallery=GALLERY, feature_cache=None):
    from learning.current_zones import _geo
    cache={}; samples={}; rejected=[]; snapshots={}


    if feature_cache and feature_cache.get('feature_sha') == feature_contract():
        for entry in feature_cache.get('samples', []):
            ident=tuple(entry['id'][:3])+(entry['id'][3],)
            cache[ident]=(entry['feats'],entry['id'][4],entry.get('snapshot_family'))
    for lineno,line in enumerate(Path(path).read_text().splitlines(),1):
        try:
            w=json.loads(line)
            judge=str(w['judge']).strip().lower()
            w['judge'] = 'claude' if judge.startswith('claude') else judge
            if w['judge'] not in ('claude','codex'):raise ValueError('Unknown reviewer')
            grade=w['rating']; errors=w['faults']; severity=w['heavy']
            if isinstance(grade,bool) or not isinstance(grade,(int,float)) or not math.isfinite(grade) or not 1<=grade<=10:raise ValueError('Score')
            if not isinstance(errors,list) or len(set(errors))!=len(errors) or set(errors)-set(KEYS):raise ValueError('ErrorCatalog')
            if set(severity)!=set(errors) or any(type(v) is not int or not 1<=v<=3 for v in severity.values()):raise ValueError('Heavy')
            ident=(w['geometry_sha256'],w['net'],w['layer'],json.dumps(w.get('ground'),sort_keys=True))
            if w['geometry_sha256'] not in snapshots:
                source=Path(gallery)/w['geometry']
                if source.exists():
                    raw=source.read_bytes()
                    if sha(raw)!=w['geometry_sha256']:raise ValueError('Snapshot hash changed')
                    snapshots[w['geometry_sha256']]=raw
                elif ident not in cache:
                    raise ValueError('Snapshot missing and no matching feature cache: '+w['geometry'])
            if ident not in cache:
                data=json.loads(gzip.decompress(snapshots[w['geometry_sha256']]));B,R=data['board'],data['routes']
                zs=[z for z in R.get('zone_list',[]) if z['net']==w['net'] and z['layer']==w['layer'] and z.get('source')=='current_zones']
                if len(zs)>1:
                    m=w.get('ground') or {};zs=[z for z in zs if abs(_geo(z).area-m.get('area_mm2',-1))<.002 and abs(_geo(z).length-m.get('perimeter_mm',-1))<.002]
                if len(zs)!=1:raise ValueError('Zone not unique')

                geo_hash=sha(_geo(zs[0]).wkb)
                snapshot_family=((data.get('netclasses') or {}).get('specs') or {}).get('family')
                if w.get('family') and snapshot_family and w['family']!=snapshot_family:raise ValueError('Contradictory snapshot family')
                cache[ident]=(feats(B,R,zs[0]),geo_hash,snapshot_family)
            features,geo_hash,snapshot_family=cache[ident];key=(*ident,geo_hash)
            if w.get('family') and snapshot_family and w['family']!=snapshot_family:raise ValueError('Contradictory snapshot family')
            fam=family(dict(w, family=snapshot_family or w.get('family')))
            item=samples.setdefault(key,dict(id=list(key),family=fam,snapshot_family=snapshot_family,feats=features,verdicts={}))
            if item['family']!=fam:raise ValueError('Contradictory family')
            label=dict(rating=float(grade),faults=errors,heavy=severity)
            prior=item['verdicts'].get(w['judge'])
            if prior is not None and prior!=label:raise ValueError('Contradictory double verdict')
            item['verdicts'][w['judge']]=label
        except (ValueError,KeyError,TypeError,OSError) as e:rejected.append(dict(row=lineno,reason=str(e)))
    return list(samples.values()),rejected


def _fit(X,labels,seed):
    import numpy as np
    from sklearn.ensemble import ExtraTreesRegressor
    target=np.asarray([[sum(a['rating'] for a in ls)/len(ls)]+[sum(k in a['faults'] for a in ls)/len(ls) for k in KEYS] for ls in labels])
    def train(x,y):return ExtraTreesRegressor(n_estimators=64,max_depth=6,min_samples_leaf=2,n_jobs=1,random_state=seed).fit(x,y)
    main=train(X,target); severity={}
    for k in KEYS:
        ix=[i for i,ls in enumerate(labels) if any(k in a['faults'] for a in ls)]
        if ix:severity[k]=train(X[ix],np.asarray([sum(a['heavy'][k] for a in labels[i] if k in a['faults'])/sum(k in a['faults'] for a in labels[i]) for i in ix]))
    return main,severity


def _export(model):
    return [dict(children_left=t.tree_.children_left.tolist(),children_right=t.tree_.children_right.tolist(),
        feature=t.tree_.feature.tolist(),threshold=t.tree_.threshold.tolist(),value=t.tree_.value[:,:,0].tolist()) for t in model.estimators_]


def _predict(trees,x):
    import numpy as np
    ys=[]
    x=np.asarray(x,dtype=np.float32)
    for t in trees:
        i=0
        while t['children_left'][i]!=-1:i=t['children_left'][i] if x[t['feature'][i]]<=t['threshold'][i] else t['children_right'][i]
        ys.append(t['value'][i])
    return np.mean(ys,axis=0)


def _rho(a,b):
    from scipy.stats import spearmanr
    if len(a)<3 or len(set(a))<2 or len(set(b))<2:return None
    return float(spearmanr(a,b).statistic)


def _metrics(pairs):
    import numpy as np
    result=dict(n=len(pairs),spearman=_rho([a['rating'] for a,b in pairs],[b['rating'] for a,b in pairs]),
        mae_score=float(np.mean([abs(a['rating']-b['rating']) for a,b in pairs])) if pairs else None,faults={})
    for k in KEYS:
        tp=sum(k in a['faults'] and b['error_p'][k]>=.5 for a,b in pairs)
        fp=sum(k not in a['faults'] and b['error_p'][k]>=.5 for a,b in pairs)
        fn=sum(k in a['faults'] and b['error_p'][k]<.5 for a,b in pairs)
        sv=[abs(a['heavy'][k]-b['heavy'][k]) for a,b in pairs if k in a['faults'] and b['heavy'][k] is not None]
        result['faults'][k]=dict(brier=float(np.mean([(float(k in a['faults'])-b['error_p'][k])**2 for a,b in pairs])) if pairs else None,f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None,tp=tp,fp=fp,fn=fn,
            positive=sum(k in a['faults'] for a,b in pairs),heavy_n=len(sv),heavy_mae=float(np.mean(sv)) if sv else None)
    return result


def train(path, out, gallery=GALLERY, seed=14):
    from learning.zones_judge_training import train as train
    return train(path, out, gallery, seed)


class Service:
    def __init__(self, model=None):
        self.path=Path(model or MODEL)
        if self.path.is_dir():self.path=self.path/'model.json'
        self.model=None;self.status='no_model';self.faults=None
        if self.path.exists():
            try:
                m=json.loads(self.path.read_text())
                if m['version']!=VERSION or m['feature_sha']!=feature_contract() or m['faults']!=KEYS or not m['trees']:raise ValueError('Model contract changed')
                for trees, outputs in [(m['trees'],len(KEYS)+1)]+[(v,1) for v in m['heavy'].values()]+([(m['score_trees'],1)] if 'score_trees' in m else []):
                    if not trees:raise ValueError('Empty model head')
                    for t in trees:
                        count=len(t['feature'])
                        if not count or any(len(t[k])!=count for k in ('children_left','children_right','threshold','value')):raise ValueError('TreeDimension')
                        for i,(left,right,feature,value) in enumerate(zip(t['children_left'],t['children_right'],t['feature'],t['value'])):
                            if left==-1:
                                if right!=-1:raise ValueError('SheetContract')
                            elif not (type(left) is int and type(right) is int and i<left<count and i<right<count and type(feature) is int and 0<=feature<len(m['feats'])):raise ValueError('TreeCycle/Index')
                            if len(value)!=outputs or any(not math.isfinite(x) for x in value) or not math.isfinite(t['threshold'][i]):raise ValueError('Non-finite model value')
                self.model=m;self.status='pilot' if not m['released'] else 'released'
            except (ValueError,KeyError,TypeError) as e:self.status='incompatible';self.faults=str(e)

    def grade(self,B,R,z):
        if self.model is None:return dict(status=self.status,rating=None,faults=self.faults)
        f=feats(B,R,z);x=[f[k] for k in self.model['feats']];m=self.model
        y=_predict(m['trees'],x);outs=int(sum(v<lo-1e-9 or v>hi+1e-9 for v,lo,hi in zip(x,m['range_']['min'],m['range_']['max'])))
        rating=_predict(m['score_trees'],x)[0] if 'score_trees' in m else y[0]
        return dict(status=self.status,rating=float(max(1,min(10,rating))),
            error_p={k:float(max(0,min(1,y[i+1]))) for i,k in enumerate(KEYS)},
            heavy={k:float(max(1,min(3,_predict(m['heavy'][k],x)[0]))) if k in m['heavy'] else None for k in KEYS},
            outside_features=outs,model_sha=sha(self.path.read_bytes()),feats=f)

    def measure(self,B,R):
        if self.model is None:return dict(status=self.status,zones_judge_score=None,faults=self.faults,zone_list=[])
        rows=[dict(net=z['net'],layer=z['layer'],**self.grade(B,R,z)) for z in zone_list(R)]
        return dict(status=self.status,zones_judge_score=sum(z['rating'] for z in rows)/len(rows) if rows else None,
            zones_judge_min=min((z['rating'] for z in rows),default=None),zone_list=rows,
            model_sha=sha(self.path.read_bytes()),released=self.model['released'])


def main():
    args=sys.argv[1:];kw=dict(a.split('=',1) for a in args if '=' in a);pos=[a for a in args if '=' not in a]
    if not pos:print(__doc__);return 0
    if pos[0]=='train':
        r=train(kw.get('data',ROOT/'learning/data/zones_verdicts.jsonl'),kw.get('out',MODEL.parent),kw.get('gallery',GALLERY),int(kw.get('seed',14)))
        if kw.get('learning_curve','0') == '1':
            from learning.zones_judge_training import learning_curve, _atomic
            out=Path(kw.get('out',MODEL.parent))
            _atomic(out/'learning_curve.json',learning_curve(json.loads((out/'features.json').read_text()),int(kw.get('seed',14))))
        print(json.dumps({k:v for k,v in r.items() if k not in ('oof','folds')},ensure_ascii=False));return 0 if r['adoption']['passed'] else 2
    if pos[0]=='measure':
        B,R=[json.loads(Path(p).read_text()) for p in pos[1:3]];print(json.dumps(Service(kw.get('model')).measure(B,R)));return 0
    raise ValueError('train|measure')


if __name__=='__main__':
    from common.process_guard import bootstrap
    bootstrap(heavy=True)
    sys.exit(main())
