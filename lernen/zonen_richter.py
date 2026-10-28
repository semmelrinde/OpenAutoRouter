""
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
VERSION = 'zonen-richter-v1'
GALERIE = Path.home()/'Library/Caches/Autorouter/galerie'
MODELL = ROOT/'lernen/modell/zonen_richter/v2/modell.json'
from lernen.zonen_gutachten import FEHLER
KEYS = sorted(FEHLER)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def zonen(R):
    ""
    from lernen.stromzonen import _geo, _polys
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    groups = {}
    for z in R.get('zonen', []):
        if z.get('quelle') == 'stromzonen' and z.get('polys'):
            groups.setdefault((z['net'], z['layer']), []).append(_geo(z))
    for z in R.get('zones', []):
        if z.get('filled') and z.get('zonen_quelle', z.get('quelle')) == 'stromzonen':
            groups.setdefault((z['net'], z['layer']), []).append(Polygon(z['pts'], z.get('holes') or []))
    return [dict(net=n, layer=l, polys=_polys(unary_union(gs))) for (n,l),gs in sorted(groups.items())]


def diagnose(B, R):
    ""
    from lernen.zonen_richter_auswahl import an
    if not an(R, (R.get('optionen') or {}).get('zonen_richter_diagnose', 0)):
        return dict(status='aus', zonen_richter_note=None, zonen=[])
    try:
        return Dienst((R.get('optionen') or {}).get('zonen_richter_modell')).messen(B, R)
    except Exception as e:
        return dict(status='fehler', zonen_richter_note=None, fehler=repr(e))


def merkmale(B, R, z):
    import numpy as np
    import shapely
    from scipy.ndimage import distance_transform_edt, maximum_filter
    from shapely.geometry import LineString, Point, Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
    from lernen.stromzonen import _geo, _teile
    g = _geo(z)
    if g.is_empty or not g.is_valid or g.area <= 0: raise ValueError('Leere/ungueltige Zone')
    n,l = z['net'],z['layer']; A,P = g.area,g.length
    parts = [p for p in getattr(g,'geoms',[g]) if p.geom_type == 'Polygon']
    out = dict(flaeche=A, umfang=P, teile=len(parts), loecher=sum(len(p.interiors) for p in parts),
        splitter_anteil=sum(p.area for p in parts if p.area < .5)/A,
        groesstes_teil=max(p.area for p in parts)/A, kompakt=4*math.pi*A/P**2,
        konvex=A/max(g.convex_hull.area,1e-9))
    xy = np.asarray(g.convex_hull.exterior.coords[:-1])
    eig, axes = np.linalg.eigh(np.cov(xy.T))
    extent = np.ptp(xy @ axes, axis=0)
    out['streckung_proxy'] = float(max(extent)/max(min(extent),1e-9))
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
        prefix='rand_%s_'%tol
        out.update({prefix+'ecken_mm':len(t)/P, prefix+'drehung_mm':float(t.sum()/P),
            prefix+'kerben_mm':sum(v < -.15 for v in signed)/P,
            prefix+'spitz_mm':float(np.sum(t>math.pi*.67)/P),
            prefix+'kurzkanten':float(np.mean(lengths<.1)) if len(lengths) else 0,
            prefix+'kurzumfang':float(lengths[lengths<.1].sum()/P),
            prefix+'rau_flaeche':g.symmetric_difference(s).area/A,
            prefix+'rau_umfang':max(0,1-s.length/P)})
        for lo,hi in ((0,15),(15,45),(45,90),(90,180)):
            out[prefix+'winkel_%d_%d'%(lo,hi)]=float(np.mean((t>=math.radians(lo))&(t<math.radians(hi)))) if len(t) else 0
    smooth=g.buffer(.1,8).buffer(-.1,8)
    out['rau_glatt']=g.symmetric_difference(smooth).area/A
    for r in (.1,.2,.3,.5,.75,1.,1.5,2.):out['erosion_%s'%r]=g.buffer(-r,8).area/A

    x0,y0,x1,y1=g.bounds; h=max(.03,max(x1-x0,y1-y0)/240)
    xs=np.arange(x0-h,x1+2*h,h);ys=np.arange(y0-h,y1+2*h,h)
    xx,yy=np.meshgrid(xs,ys); mask=shapely.contains_xy(g,xx,yy)
    d=distance_transform_edt(mask)*h; ridge=mask&(d>=maximum_filter(d,size=3)-1e-10)
    widths=2*d[ridge]; q=np.quantile(widths,[.1,.5,.9]) if len(widths) else np.zeros(3)
    out.update(breite_p10=float(q[0]),breite_median=float(q[1]),breite_p90=float(q[2]),
        breite_cv=float(np.std(widths)/max(np.mean(widths),1e-9)) if len(widths) else 0,
        enganteil=float(np.mean(widths<max(.2,q[1]*.5))) if len(widths) else 0,raster_mm=h)
    jumps=[]
    for axis in (0,1):
        a=np.take(d,range(d.shape[axis]-1),axis=axis);b=np.take(d,range(1,d.shape[axis]),axis=axis)
        both=np.take(ridge,range(d.shape[axis]-1),axis=axis)&np.take(ridge,range(1,d.shape[axis]),axis=axis)
        jumps.extend((2*np.abs(a-b)[both]).tolist())
    out['breitensprung_proxy']=max(jumps,default=0)
    pads=[Polygon(q).buffer(0) for p in B.get('pads',[]) if p.get('net')==n for q in (p.get('polys') or {}).get(l,[]) if len(q)>=3]
    near=[p for p in pads if p.distance(g)<1.]
    out['pads_nah']=len(near)
    out['padabdeckung']=sum(g.intersection(p).area/max(p.area,1e-9) for p in near)/max(1,len(near))
    paths=[LineString([a,b]) for t in R.get('tracks',[]) if t['net']==n and t['layer']==l
        for a,b in zip(t['pts'][:-1],t['pts'][1:]) if a!=b]
    envelope=unary_union(pads+[p.buffer(max(.25,q[1]/2),8) for p in paths])
    out['ohne_pfad_proxy']=g.difference(envelope).area/A
    holes=[Point(v['x'],v['y']).buffer(float(v.get('diameter',v.get('d',.45)))/2+.16,16)
        for v in R.get('vias',[]) if v['net']!=n and l in (v.get('layers') or ['F','B']+['In%d'%i for i in range(1,int(B.get('lagen',2))-1)])]
    out['via_bisse_mm']=sum(g.boundary.intersection(p).length for p in holes)/P
    foreign=unary_union([p for m,p in _teile(B,R,l) if m!=n])
    outline=unary_union([Polygon(o).buffer(0) for o in B.get('outline',[]) if len(o)>=3])
    free=g.buffer(.5,8).difference(g).intersection(outline.buffer(-.25)).difference(foreign.buffer(.15,8))
    out['frei_daneben']=free.area/A
    if any(not math.isfinite(v) for v in out.values()):raise ValueError('Nichtendliches Formmerkmal')
    return out


def merkmalsvertrag():
    return sha(inspect.getsource(merkmale).encode())


def familie(w):


    if w.get('familie'):return str(w['familie'])
    match=re.fullmatch(r'erf_(\d+)(?:_f[\d.]+)?',str(w.get('board','')))
    if match:return 'aufgabe_%d'%(int(match[1])//5)
    if w.get('quelle_meta'):
        from lernen.familien import quelle, familie as f
        if quelle(w['quelle_meta']):return f(w)
    from lernen.familien import aliasse
    registered=aliasse().get(str(w.get('board','')).lower().removesuffix('_eng'))
    if registered:return registered
    raise ValueError('Quellenfamilie fehlt: '+str(w.get('board')))


def daten_lesen(path,galerie=GALERIE, merkmalscache=None):
    from lernen.stromzonen import _geo
    cache={}; samples={}; rejected=[]; snapshots={}


    if merkmalscache and merkmalscache.get('merkmal_sha') == merkmalsvertrag():
        for entry in merkmalscache.get('samples', []):
            ident=tuple(entry['id'][:3])+(entry['id'][3],)
            cache[ident]=(entry['merkmale'],entry['id'][4],entry.get('snapshot_familie'))
    for lineno,line in enumerate(Path(path).read_text().splitlines(),1):
        try:
            w=json.loads(line)
            judge=str(w['richter']).strip().lower()
            w['richter'] = 'claude' if judge.startswith('claude') else judge
            if w['richter'] not in ('claude','codex'):raise ValueError('Unbekannter Gutachter')
            grade=w['note']; errors=w['fehler']; severity=w['schwere']
            if isinstance(grade,bool) or not isinstance(grade,(int,float)) or not math.isfinite(grade) or not 1<=grade<=10:raise ValueError('Note')
            if not isinstance(errors,list) or len(set(errors))!=len(errors) or set(errors)-set(KEYS):raise ValueError('Fehlerkatalog')
            if set(severity)!=set(errors) or any(type(v) is not int or not 1<=v<=3 for v in severity.values()):raise ValueError('Schwere')
            ident=(w['geometrie_sha256'],w['net'],w['layer'],json.dumps(w.get('masse'),sort_keys=True))
            if w['geometrie_sha256'] not in snapshots:
                source=Path(galerie)/w['geometrie']
                if source.exists():
                    raw=source.read_bytes()
                    if sha(raw)!=w['geometrie_sha256']:raise ValueError('Snapshot-Hash geaendert')
                    snapshots[w['geometrie_sha256']]=raw
                elif ident not in cache:
                    raise ValueError('Snapshot fehlt und kein passender Merkmalscache: '+w['geometrie'])
            if ident not in cache:
                data=json.loads(gzip.decompress(snapshots[w['geometrie_sha256']]));B,R=data['board'],data['routes']
                zs=[z for z in R.get('zonen',[]) if z['net']==w['net'] and z['layer']==w['layer'] and z.get('quelle')=='stromzonen']
                if len(zs)>1:
                    m=w.get('masse') or {};zs=[z for z in zs if abs(_geo(z).area-m.get('flaeche_mm2',-1))<.002 and abs(_geo(z).length-m.get('umfang_mm',-1))<.002]
                if len(zs)!=1:raise ValueError('Zone nicht eindeutig')

                geo_hash=sha(_geo(zs[0]).wkb)
                snapshot_familie=((data.get('klassen') or {}).get('vorgaben') or {}).get('familie')
                if w.get('familie') and snapshot_familie and w['familie']!=snapshot_familie:raise ValueError('Widerspruechliche Snapshot-Familie')
                cache[ident]=(merkmale(B,R,zs[0]),geo_hash,snapshot_familie)
            features,geo_hash,snapshot_familie=cache[ident];key=(*ident,geo_hash)
            if w.get('familie') and snapshot_familie and w['familie']!=snapshot_familie:raise ValueError('Widerspruechliche Snapshot-Familie')
            fam=familie(dict(w, familie=snapshot_familie or w.get('familie')))
            item=samples.setdefault(key,dict(id=list(key),familie=fam,snapshot_familie=snapshot_familie,merkmale=features,urteile={}))
            if item['familie']!=fam:raise ValueError('Widerspruechliche Familie')
            label=dict(note=float(grade),fehler=errors,schwere=severity)
            prior=item['urteile'].get(w['richter'])
            if prior is not None and prior!=label:raise ValueError('Widerspruechliches Doppelurteil')
            item['urteile'][w['richter']]=label
        except (ValueError,KeyError,TypeError,OSError) as e:rejected.append(dict(zeile=lineno,grund=str(e)))
    return list(samples.values()),rejected


def _fit(X,labels,seed):
    import numpy as np
    from sklearn.ensemble import ExtraTreesRegressor
    target=np.asarray([[sum(a['note'] for a in ls)/len(ls)]+[sum(k in a['fehler'] for a in ls)/len(ls) for k in KEYS] for ls in labels])
    def train(x,y):return ExtraTreesRegressor(n_estimators=64,max_depth=6,min_samples_leaf=2,n_jobs=1,random_state=seed).fit(x,y)
    main=train(X,target); severity={}
    for k in KEYS:
        ix=[i for i,ls in enumerate(labels) if any(k in a['fehler'] for a in ls)]
        if ix:severity[k]=train(X[ix],np.asarray([sum(a['schwere'][k] for a in labels[i] if k in a['fehler'])/sum(k in a['fehler'] for a in labels[i]) for i in ix]))
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
    result=dict(n=len(pairs),spearman=_rho([a['note'] for a,b in pairs],[b['note'] for a,b in pairs]),
        mae_note=float(np.mean([abs(a['note']-b['note']) for a,b in pairs])) if pairs else None,fehler={})
    for k in KEYS:
        tp=sum(k in a['fehler'] and b['fehler_p'][k]>=.5 for a,b in pairs)
        fp=sum(k not in a['fehler'] and b['fehler_p'][k]>=.5 for a,b in pairs)
        fn=sum(k in a['fehler'] and b['fehler_p'][k]<.5 for a,b in pairs)
        sv=[abs(a['schwere'][k]-b['schwere'][k]) for a,b in pairs if k in a['fehler'] and b['schwere'][k] is not None]
        result['fehler'][k]=dict(brier=float(np.mean([(float(k in a['fehler'])-b['fehler_p'][k])**2 for a,b in pairs])) if pairs else None,f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None,tp=tp,fp=fp,fn=fn,
            positiv=sum(k in a['fehler'] for a,b in pairs),schwere_n=len(sv),schwere_mae=float(np.mean(sv)) if sv else None)
    return result


def trainieren(path, out, galerie=GALERIE, seed=14):
    from lernen.zonen_richter_training import trainieren as train
    return train(path, out, galerie, seed)


class Dienst:
    def __init__(self, modell=None):
        self.path=Path(modell or MODELL)
        if self.path.is_dir():self.path=self.path/'modell.json'
        self.modell=None;self.status='kein_modell';self.fehler=None
        if self.path.exists():
            try:
                m=json.loads(self.path.read_text())
                if m['version']!=VERSION or m['merkmal_sha']!=merkmalsvertrag() or m['fehler']!=KEYS or not m['baeume']:raise ValueError('Modellvertrag geaendert')
                for trees, outputs in [(m['baeume'],len(KEYS)+1)]+[(v,1) for v in m['schwere'].values()]+([(m['note_baeume'],1)] if 'note_baeume' in m else []):
                    if not trees:raise ValueError('Leerer Modellkopf')
                    for t in trees:
                        count=len(t['feature'])
                        if not count or any(len(t[k])!=count for k in ('children_left','children_right','threshold','value')):raise ValueError('Baumdimension')
                        for i,(left,right,feature,value) in enumerate(zip(t['children_left'],t['children_right'],t['feature'],t['value'])):
                            if left==-1:
                                if right!=-1:raise ValueError('Blattvertrag')
                            elif not (type(left) is int and type(right) is int and i<left<count and i<right<count and type(feature) is int and 0<=feature<len(m['merkmale'])):raise ValueError('Baumzyklus/Index')
                            if len(value)!=outputs or any(not math.isfinite(x) for x in value) or not math.isfinite(t['threshold'][i]):raise ValueError('Nichtendlicher Modellwert')
                self.modell=m;self.status='pilot' if not m['freigegeben'] else 'freigegeben'
            except (ValueError,KeyError,TypeError) as e:self.status='inkompatibel';self.fehler=str(e)

    def benoten(self,B,R,z):
        if self.modell is None:return dict(status=self.status,note=None,fehler=self.fehler)
        f=merkmale(B,R,z);x=[f[k] for k in self.modell['merkmale']];m=self.modell
        y=_predict(m['baeume'],x);outs=int(sum(v<lo-1e-9 or v>hi+1e-9 for v,lo,hi in zip(x,m['bereich']['min'],m['bereich']['max'])))
        note=_predict(m['note_baeume'],x)[0] if 'note_baeume' in m else y[0]
        return dict(status=self.status,note=float(max(1,min(10,note))),
            fehler_p={k:float(max(0,min(1,y[i+1]))) for i,k in enumerate(KEYS)},
            schwere={k:float(max(1,min(3,_predict(m['schwere'][k],x)[0]))) if k in m['schwere'] else None for k in KEYS},
            ausserhalb_merkmale=outs,modell_sha=sha(self.path.read_bytes()),merkmale=f)

    def messen(self,B,R):
        if self.modell is None:return dict(status=self.status,zonen_richter_note=None,fehler=self.fehler,zonen=[])
        rows=[dict(net=z['net'],layer=z['layer'],**self.benoten(B,R,z)) for z in zonen(R)]
        return dict(status=self.status,zonen_richter_note=sum(z['note'] for z in rows)/len(rows) if rows else None,
            zonen_richter_min=min((z['note'] for z in rows),default=None),zonen=rows,
            modell_sha=sha(self.path.read_bytes()),freigegeben=self.modell['freigegeben'])


def main():
    args=sys.argv[1:];kw=dict(a.split('=',1) for a in args if '=' in a);pos=[a for a in args if '=' not in a]
    if not pos:print(__doc__);return 0
    if pos[0]=='trainieren':
        r=trainieren(kw.get('daten',ROOT/'lernen/daten/zonen_urteile.jsonl'),kw.get('out',MODELL.parent),kw.get('galerie',GALERIE),int(kw.get('seed',14)))
        if kw.get('lernkurve','0') == '1':
            from lernen.zonen_richter_training import lernkurve, _atomic
            out=Path(kw.get('out',MODELL.parent))
            _atomic(out/'lernkurve.json',lernkurve(json.loads((out/'merkmale.json').read_text()),int(kw.get('seed',14))))
        print(json.dumps({k:v for k,v in r.items() if k not in ('oof','folds')},ensure_ascii=False));return 0 if r['uebernahme']['bestanden'] else 2
    if pos[0]=='messen':
        B,R=[json.loads(Path(p).read_text()) for p in pos[1:3]];print(json.dumps(Dienst(kw.get('modell')).messen(B,R)));return 0
    raise ValueError('trainieren|messen')


if __name__=='__main__':
    from gemeinsam.prozessschutz import bootstrap
    bootstrap(heavy=True)
    sys.exit(main())
