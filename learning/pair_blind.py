import collections
import gzip
import hashlib
import json
import math
import re
import time
from pathlib import Path

from learning.appraisal_data import canonical

VERSION = 'blind-geometry-v3'
INSTRUCTION_V2 = '''Evaluate only the anonymous board images presented, following learning/appraisal_design_rules.md.
Read no metrics, gallery, samples, previous scores or geometry files.
Score 1 to 10 (higher is better): islands/decoupling P2/P3, density P26, short clear signal paths R19/R24,
Y/T branches R27, current chains R11/R22, compact smooth current zones R36/R39/K17/K20.
Additionally current_score 1 to 10, null without visible current paths. Do not treat unknowns as measured.
Answer with JSON only: {"scores":[{"image":"<Alias>","rating":8,"current_score":7,"design_rules":["R19"],"short":"Reasoning"}],"weaknesses":[]}.
Images:\n'''


INSTRUCTION = '''Evaluate only the anonymous board images presented, following learning/appraisal_design_rules.md.
Read no metrics, gallery, samples, previous scores or geometry files.
Score 1 to 10 (higher is better): islands/decoupling P2/P3, density P26, short clear signal paths R19/R24,
Y/T branches R27, current chains R11/R22, compact smooth current zones R36/R39/K17/K20.
Crossing or overlapping tracks of different nets on the same layer are short circuits (R44): very bad.
Additionally current_score 1 to 10 only for current paths and current zones (compact, smooth, without notches, blobs, stubs,
jumps between thick/thin, without unnecessary bottlenecks), null without visible current paths. Do not treat unknowns as measured.
Answer with JSON only, in exactly this shaping:
{"scores":[{"image":"<Alias>","rating":8,"current_score":7,"design_rules":["R19"],"short":"Reasoning"}],
 "weaknesses":[{"what":"<common weakness of the algorithm>","design_rules":["R36"],"how_often":<number of images>,"suggestion":"<concrete approach in the code>"}]}
Images:\n'''
INSTRUCTIONS = {'blind-geometry-v2': INSTRUCTION_V2, VERSION: INSTRUCTION}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def job(aliases, cu_layers=None, version=VERSION):
    if len(set(aliases)) != len(aliases) or any(not a.startswith('blind_') or not a[6:-4].isdigit() or not a.endswith('.png') for a in aliases):
        raise ValueError('Blind alias invalid')
    cu_layers = cu_layers or {a:[] for a in aliases}
    if set(cu_layers)!=set(aliases) or any(not re.fullmatch(r'F|B|In\d+',L) for ls in cu_layers.values() for L in ls):
        raise ValueError('Layer list invalid')
    return INSTRUCTIONS[version] + '\n'.join(a + ' (overview, then layers: '+','.join(cu_layers[a])+')' for a in aliases)


def geometry(z):
    p = Path(z.get('geometry_path') or Path(z['image_path']).parent/z['geometry'])
    raw = p.read_bytes()
    g = json.loads(gzip.decompress(raw))
    if not all(isinstance(g.get(k),dict) for k in ('board','routes','netclasses')):
        raise ValueError('Snapshot incomplete')
    return raw,g


def draw(g):
    from learning.gallery import _draw
    in_ = _draw(g['board'],g['routes'],720,upper=22)
    return in_.crop((0,22,in_.width,in_.height))


def pair_sample(Z, n, rnd):
    from learning.pair_contract import board_id
    groups=collections.defaultdict(dict)
    for z in Z:
        if z.get('is_valid') is not True or not z.get('board'):continue
        try: _,g=geometry(z)
        except (OSError,ValueError,KeyError):continue


        groups[board_id(g['board'])][sha(canonical(g))]=z
    boards=sorted(b for b,g in groups.items() if len(g)>=2);rnd.shuffle(boards)
    return [z for b in boards[:n//2] for z in rnd.sample(list(groups[b].values()),2)]

def prepare(z, folder, alias):
    d = Path(folder); d.mkdir(parents=True,exist_ok=True)
    raw,g = geometry(z)
    dst = d/alias; draw(g).save(dst)
    gp = d/(alias[:-4]+'_geo.json.gz'); gp.write_bytes(raw)

    from learning.gallery import image_layers
    names = image_layers(g['board'],g['routes'],str(d/alias[:-4]),K=g['netclasses'],design_rules=g.get('design_rules'))
    paths = {L:str(d/name) for L,name in names.items()}
    from learning.pair_contract import board_id
    proof = dict(version=VERSION, alias=alias, board_id=board_id(g['board']), geometry_sha256=sha(raw),
                 images={'overview':sha(dst.read_bytes()), **{L:sha(Path(p).read_bytes()) for L,p in paths.items()}},
                 renderer=renderer_signature())
    return dict(z, image_path=str(dst), geometry=gp.name, geometry_path=str(gp), layer_images_paths=paths,
                blind=True, blind_version=VERSION, blind_evidence=proof)


def renderer_signature():
    from learning.pair_contract import ROOT
    paths = ['learning/gallery.py','learning/copper.py','common/zones.py','learning/pair_blind.py']
    return sha(b''.join(p.encode()+b'\0'+(ROOT/p).read_bytes() for p in paths))


def template(W):
    aliases=[z['blind_evidence']['alias'] for z in W]
    layers={z['blind_evidence']['alias']:sorted(z['layer_images_paths']) for z in W}
    prompt=job(aliases,layers)
    for z in W:
        z['blind_evidence'].update(aliases=aliases, cu_layers=layers, job=prompt, job_sha256=sha(prompt.encode()))
    return prompt


def answer(z, x, judge):
    p=z.get('blind_evidence')
    if not p: return {}
    return dict(blind_answer=dict(alias=p['alias'], rating=x['rating'], current_score=x.get('current_score'),
                                  judge=judge, job_sha256=p['job_sha256'],
                                  images=p['images']))


def verify(z):
    try:
        if z.get('blind') is not True or z.get('synthetic') or z.get('_split_conflict') or z.get('is_valid') is not True:
            return False,'no valid blind real verdict'
        d=Path(z['archive']); meta=json.loads((d/'metrics.json').read_text())
        proof=meta.get('blind_evidence') or {}
        if proof.get('version') not in INSTRUCTIONS or z.get('blind_evidence')!=proof:
            return False,'Template evidence missing/deviates'
        if meta.get('board')!=z.get('board') or meta.get('is_valid') is not True:
            return False,'Board/validity does not match archive'
        from learning.judge_pair import NUM
        fields = set(NUM) | {'is_valid','score_v3','score_v3_version','board_id','pair_measurement_version',
                            'rule','design_rules','gate_check','outcome','metric','metrics','feats','current_chain',
                            'r27','v3_components','v3_components_evidence','shape_measurement','geometry_sha256'}


        def measured(row):
            return {k:row[k] for k in fields if k in row and (k != 'design_rules' or isinstance(row[k],dict))}
        if canonical(measured(z)) != canonical(measured(meta)):
            return False,'Metrics do not match archive'
        imraw=(d/'overview.png').read_bytes()
        if sha(imraw)!=z['image_sha256'] or sha(imraw)!=meta['image_sha256'] or sha(imraw)!=proof['images']['overview']:
            return False,'Image hash deviates'
        paths=z.get('layer_images_paths') or {}
        layers=[]
        for L,p in sorted(paths.items()):
            path=Path(p)
            if path.parent.resolve()!=d.resolve():return False,'Layer image outside archive'
            raw=path.read_bytes();layers.append((L,raw))
            if sha(raw)!=proof['images'].get(L):return False,'Layer image hash deviates'
        if set(proof['images']) != {'overview',*paths}: return False,'Image scope differs'
        m=dict(meta);m.pop('image_sha256')
        sid=sha(canonical(m)+imraw+b''.join(L.encode()+b for L,b in layers))
        if sid!=z['sample_id'] or d.name!=sid:return False,'Archive identity deviates'
        gp=Path(z['geometry_path'])
        if gp.resolve()!= (d/'geometry.json.gz').resolve():return False,'Geometry outside archive'
        raw,g=geometry(z)
        if sha(raw)!=proof['geometry_sha256'] or sha(raw)!=meta.get('geometry_sha256'):
            return False,'Geometry hash deviates'
        from learning.pair_contract import board_id
        identity=board_id(g['board'])
        if identity!=proof.get('board_id'):return False,'Board ID does not match snapshot'
        for value in (z.get('board_id'), (z.get('rule') or {}).get('board_id')):
            if value is not None and value!=identity:return False,'Measurement ID does not match snapshot'
        prompt=job(proof['aliases'],proof['cu_layers'],proof['version'])
        if proof['alias'] not in proof['aliases'] or prompt!=proof['job'] or sha(prompt.encode())!=proof['job_sha256']:
            return False,'Metrics/identity in task or task binding missing'


        if not isinstance(proof.get('renderer'),str) or not re.fullmatch('[0-9a-f]{64}',proof['renderer']):return False,'Renderer origin missing'
        from PIL import Image
        import io
        actual=Image.open(io.BytesIO(imraw)).convert('RGB');expected=draw(g).convert('RGB')
        if actual.size!=expected.size or actual.tobytes()!=expected.tobytes():return False,'Image not free of metrics from snapshot'

        import tempfile
        from learning.gallery import image_layers
        with tempfile.TemporaryDirectory(prefix='pair_blind_') as tmp:
            names=image_layers(g['board'],g['routes'],str(Path(tmp)/'layer_name'),K=g['netclasses'],design_rules=g.get('design_rules'))
            if set(names)!=set(paths):return False,'Layer scope not reproducible'
            for L,name in names.items():
                a=Image.open(paths[L]).convert('RGB');b=Image.open(Path(tmp)/name).convert('RGB')
                if a.size!=b.size or a.tobytes()!=b.tobytes():return False,'Layer image not from snapshot'
        receipt=z.get('blind_answer') or {}
        if receipt!=dict(alias=proof['alias'],rating=z.get('rating'),current_score=z.get('current_score'),judge=z.get('judge'),
                         job_sha256=proof['job_sha256'],images=proof['images']):return False,'Verdict not bound to template'
        return True,'Archive/images/geometry/task/answer verified'
    except (OSError,ValueError,KeyError,TypeError,AttributeError):
        return False,'Blind artifacts missing/defective'


def view_id(z):
    from PIL import Image
    paths={'overview':str(Path(z['archive'])/'overview.png'), **(z.get('layer_images_paths') or {})}
    h=hashlib.sha256()
    for label,path in sorted(paths.items()):
        with Image.open(path) as in_:
            in_=in_.convert('RGB');h.update(canonical([label,in_.size]));h.update(in_.tobytes())
    return h.hexdigest()


def control_pairs(P):
    cache={};ids={};out=[];seen=set();reasons=collections.Counter()
    for p in P:
        results=[]
        for z in (p['a'],p['b']):

            key=sha(canonical(z))
            if key not in cache:
                cache[key]=verify(z)
                if cache[key][0]:ids[key]=view_id(z)
            results.append(cache[key])
        if not all(v[0] for v in results):
            reasons.update(v[1] for v in results if not v[0]);continue
        if p['a']['blind_evidence']['board_id']!=p['b']['blind_evidence']['board_id']:
            reasons['different netlist/outline despite same board name']+=1;continue
        hashes=sorted(ids[sha(canonical(z))] for z in (p['a'],p['b']))
        key=(p['a']['blind_evidence']['board_id'],*hashes)
        if hashes[0]==hashes[1] or key in seen:
            reasons['identical/duplicate geometry pair']+=1;continue
        seen.add(key);out.append(p)
    return out,dict(candidates=len(cache),pairs=len(out),rejections=dict(reasons))


def remeasure(z):
    start = time.process_time()
    from learning.shape_live import remeasure as shape_remeasure
    from learning.shape_live import stable
    from learning.judge_pair import components, component_occupancy
    from learning.pair_contract import measurement_signature, board_id
    from analysis import contract_spec as VT
    raw,g=geometry(z)
    h=sha(raw)
    for expected in (z.get('geometry_sha256'), (z.get('blind_evidence') or {}).get('geometry_sha256')):
        if expected is not None and expected != h:
            raise ValueError('Snapshot hash deviates')
    b,r,k=g['board'],g['routes'],g['netclasses']
    measured,cost=shape_remeasure(g,h)
    e,m,rm=measured['outcome'],dict(measured['rule_metrics']),measured['rule']



    keep = ('board','task','design_family','family','source','image','image_path',
            'geometry','geometry_path','layer_images_paths','blind','blind_version','blind_evidence',
            'rating','current_score','judge','appraisal_id','appraisal_time','short','blind_answer',
            '_family','_family_aliases','_split_conflict',



            'feats','cu_layers','density','net_class','duration','candidate','difficulty','design_family')
    out={key:z[key] for key in keep if key in z}
    if isinstance(out.get('feats'),dict):
        from learning.judge_pair import NUM
        out['feats']={a:x for a,x in out['feats'].items() if a not in NUM}
    if isinstance(z.get('design_rules'),list):out['design_rules']=z['design_rules']
    sig=measurement_signature()
    m.update(board_id=board_id(b),pair_measurement_version=sig)
    fresh=dict(is_valid=e['is_valid'],design_rules=m,score_v3_version=e['score_v3_version'],
               measurement_version=m.get('measurement_version') or sig,
               proofs=VT.mandatory_proofs(e['measures'],k))
    for key in ('diagnose','conflict_list','open_items','not_connected','rank','drc'):
        if key in e:fresh[key]=e[key]
    gate_check=VT.cur_state(fresh)



    total_length=sum(math.hypot(q[0]-p[0],q[1]-p[1]) for t in r.get('tracks',[]) for p,q in zip(t.get('pts') or [],(t.get('pts') or [])[1:]))
    out['metrics']=dict(vias=len(r.get('vias',[])),total_length=m.get('total_length',round(total_length,1)),conflict_list=e.get('conflict_list'),
                           open_items=e.get('open_items'),bends=m.get('bends'),
                           share_45=None if m.get('share_45') is None else round(m['share_45']/100,3))
    out.update(is_valid=e['is_valid'],score_v3=rm['score_v3'],score_v3_version=e['score_v3_version'],
               board_id=board_id(b),pair_measurement_version=sig,rule=rm,rule_metrics=m,gate_check=gate_check,outcome=fresh,
               v3_components=components(e['measures']),v3_components_evidence=component_occupancy(e['measures']))
    out.update(geometry_sha256=h, shape_measurement=measured['shape_measurement'],
               current_chain=measured['current_chain'], shape_measurement_run=dict(cost, row_cpu_s=time.process_time()-start))
    return out
