import copy
import math
from learning.zones_judge import Service


def an(R,amount=None):
    value=amount if amount is not None else (R.get('options') or {}).get('zones_judge',0)
    if str(value).lower() not in ('0','1','false','true'):raise ValueError('zones_judge=0|1')
    return str(value).lower() in ('1','true')


def gate_check(B,a,b,K,gnd_layers=None,design_rules=None):
    from analysis.evaluation import rate
    from analysis import minimax as MM
    from analysis.rule_metrics import assumption
    from learning.zones_hard_checking import verify
    from learning.current_zones_post_run import physics_comparison
    hard=verify(B,b,K,gnd_layers,design_rules)
    e0=rate(B,a,K,v3=True,minimax=True);e1=rate(B,b,K,v3=True,minimax=True)
    physics=physics_comparison(B,a,B,b,K,e0.get('ir'),e1.get('ir'),single_branch=True)
    rules=assumption(e0.get('design_rules'),e1.get('design_rules'));h18=MM.assumption(e0,e1)
    rating=all(isinstance(e.get('score_v3'),(float,int)) and math.isfinite(e['score_v3']) for e in (e0,e1)) and e1['score_v3']<=e0['score_v3']
    ok=bool(hard.get('passed')) and bool(e0.get('is_valid')) and bool(e1.get('is_valid')) and not e1.get('not_connected') and physics['passed'] and rules['not_worse'] and h18['not_worse'] and rating
    return dict(passed=ok,hard=hard,physics=physics,design_rules=rules,h18=h18,score_v3_not_worse=rating)


def choose(B,R,K,gnd_layers=None,**params):
    from learning import current_zones as SZ
    from learning import current_zones_topo as ST
    opts=R.get('options') or {};judge=Service(opts.get('zones_judge_model'))
    if judge.model is None:raise ValueError('zones_judge=1: '+judge.status)
    if SZ.skeleton(R):raise ValueError('zones_judge=1: skeleton width change not yet validated')
    if R.get('zone_plan_id') or R.get('k20_corridor'):raise ValueError('Zone judge with local plan/K20 not yet validated')
    base=copy.deepcopy(R);base['options']=dict(opts,zones_judge='0')
    default=SZ.zone_list(B,base,K,gnd_layers,zones_judge=False,**params)


    R['vias']=copy.deepcopy(base.get('vias',[]))
    recipes=[('straight_smoothed',dict(shaping='straight',r_smoothed=.3,r_open=.1,group=1.0)),
             ('straight_hull',dict(shaping='straight',r_smoothed=.7,r_open=.15,group=2.0)),
             ('round_corridor',dict(shaping='rounded',r_smoothed=.3,r_open=.1,group=1.0)),
             ('minimax',dict(shaping='minimax'))]
    variants=[('standard',default)];errors=[]
    for name,kw in recipes:
        try:
            rr=copy.deepcopy(base)
            if name=='round_corridor':rr['r36_corridor']=True
            zs=SZ.zone_list(B,rr,K,gnd_layers,zones_judge=False,**dict(params,**kw))
            if rr.get('vias')!=base.get('vias'):raise ValueError('Variant changes via scaffold')
            variants.append((name,zs))
        except (ValueError,RuntimeError) as e:errors.append(dict(variant=name,reason=str(e)))
    for name,kw in [('topo_smoothed',dict(kappa=1.,r_smoothed=.3,r_hull=.7,r_round=.2,wedge_max=60.)),
                    ('topo_narrow',dict(kappa=2.,r_smoothed=.1,r_hull=.3,r_round=.1,wedge_max=45.))]:
        try:
            zs=ST.zone_list(B,copy.deepcopy(base),K,gnd_layers,clr=params.get('clr'),border=params.get('border',.25),via_field=False,**kw)
            if any(z.get('skeleton') for z in zs):raise ValueError('Skeleton unexpected')
            variants.append((name,zs))
        except (ValueError,RuntimeError) as e:errors.append(dict(variant=name,reason=str(e)))
    def groups(zs):
        result={}
        for z in zs:
            key=(z['net'],z['layer'])
            if key in result:raise ValueError('Several unconnected zone definitions per net/layer; selection locked')
            result[key]=z
        return result
    selected=groups(default);logs=[]
    for key in sorted(selected):
        before=selected[key];score=judge.grade(B,dict(base,zone_list=list(selected.values())),before)['rating']
        for name,zs in variants[1:]:
            candidate=groups(zs).get(key)
            if candidate is None:continue
            proposed=dict(selected);proposed[key]=candidate
            a=dict(base,zone_list=list(base.get('zone_list') or [])+list(selected.values()))
            b=dict(base,zone_list=list(base.get('zone_list') or [])+list(proposed.values()))
            pred=judge.grade(B,b,candidate)
            if pred['rating']<=score:continue
            try:gate=gate_check(B,a,b,K,gnd_layers,K.get('design_rules'))
            except Exception as e:gate=dict(passed=False,faults=repr(e))
            logs.append(dict(net=key[0],layer=key[1],variant=name,rating=pred['rating'],gate_check=gate))
            if gate['passed']:selected=proposed;score=pred['rating']
        selected[key]=dict(selected[key],zones_judge=dict(rating=score,model_sha=judge.grade(B,dict(base,zone_list=list(selected.values())),selected[key])['model_sha'],native_acceptance_required=True))
    R['zones_judge_selection']=dict(status='pilot',released=False,variants=len(variants),faults=errors,decisions=logs)
    return list(selected.values())
