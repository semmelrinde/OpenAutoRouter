import copy
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def physics_gate(a, b):
    worse = []; missing = []
    if a.get('raster_mm') != b.get('raster_mm') or not a.get('raster_mm'):
        missing.append('Same positive grid resolution missing')
    if a.get('error_nets') or b.get('error_nets'):
        missing += list(set(a.get('error_nets', [])) | set(b.get('error_nets', [])))
    for key in ('loss_per_net_mW', 'J_max_A_mm2', 'J_x'):
        if key not in a or key not in b:missing.append(key+' missing')
        for n in sorted(set(a.get(key, {})) | set(b.get(key, {}))):
            x,y=a.get(key, {}).get(n),b.get(key, {}).get(n)
            if x is None or y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(key+':'+n)
            elif y>x+max(1e-6,abs(x)*1e-6): worse.append(key+':'+n)
    for n,pads in a.get('drop_per_pad_mV', {}).items():
        for p,x in pads.items():
            y=b.get('drop_per_pad_mV', {}).get(n, {}).get(p)
            if y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(n+':'+p)
            elif y>x+max(1e-6,abs(x)*1e-6): worse.append(n+':'+p)
    return dict(passed=not missing and not worse, worse=worse, missing=missing,
                relative_tolerance=1e-6, resolution_mm=a.get('raster_mm'))


def physics_comparison(B0, a, B1, b, K, ir0=None, ir1=None, single_branch=False):
    from analysis.copper_model import ir_values
    attempts=[]
    for res in (.1,.05):
        x=ir0 if res==.1 and ir0 is not None else ir_values(B0,a,K,res=res)
        y=ir1 if res==.1 and ir1 is not None else ir_values(B1,b,K,res=res)
        gate=physics_gate(x,y)
        if single_branch:
            from learning.zone_planner_physics import per_load, gate_check
            load=gate_check(per_load(B0,a,K,res),per_load(B1,b,K,res))
            gate['single_branch_J']=load
            gate['passed'] &= load['passed']
        attempts.append(dict(raster_mm=res,gate_check=gate,before=x,after=y))
        if gate['passed']:break
    return dict(gate,attempts=attempts)


def refine(B0,a,B,old,K,RG,folder,b,native,physics,gnd=True):
    from learning import current_zones as SZ
    from learning.copper_kicad_native import verify
    history=[];base=k20_basis(old);proposal=None
    for iteration in range(2):
        if physics['passed']:break
        ir0=physics['attempts'][-1]['before']
        shield=base.setdefault('r36_protection',{})
        changed=False
        local_free=base.setdefault('r36_free',{})
        j_nets=set()
        for key in physics['worse']:
            net,sep,pad=key.partition(':')
            if net in ('J_max_A_mm2','J_x') and pad in K['net_list']:
                if pad not in j_nets:
                    local_free[pad]=max(.0125,local_free.get(pad,.05)/2);j_nets.add(pad)
                changed=True;continue
            if not sep or net not in K['net_list'] or ':' not in pad:continue
            value=(ir0.get('drop_per_pad_mV',{}).get(net) or {}).get(pad)
            maximum=ir0.get('max_drop_mV',{}).get(net)
            if value is None or not maximum:continue
            shield[net]=min(.4,max(.02,value/maximum*2.5,shield.get(net,0)*1.5))
            changed=True
        if not changed:break
        proposal=copy.deepcopy(base)
        _zs=SZ.zone_list(B,proposal,K);SZ.skeleton_apply(proposal,_zs);proposal['zone_list']+=_zs
        if gnd:
            gz,gv=SZ.gnd_fill(B,proposal,K)
            proposal['zone_list']+=gz;proposal['vias']+=gv
        path=Path(folder)/('physics_'+str(iteration+1));path.mkdir(parents=True,exist_ok=True)
        (path/'suggestion.json').write_text(json.dumps(proposal))
        _,b,nt=verify(path/'kicad_native',B,base,proposal,RG,cpu_s=240)
        physics=physics_comparison(B0,a,B,b,K)
        history.append(dict(protection=copy.deepcopy(shield),vacant=copy.deepcopy(local_free),kicad_native=nt,physics_gate=physics))
        native=dict(native,suggestion=nt['suggestion'],native_suggestion_valid=nt['suggestion']['is_valid'],
                    not_more_error=nt['suggestion']['faults']<=native['before']['faults'],
                    not_more_open=nt['suggestion']['open_items']<=native['before']['open_items'])
    native['physics_improvements']=history
    return b,native,physics,proposal


def ground_layers(B, R, K):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    out=copy.deepcopy(R)
    gl=out.get('gnd_layers') or (out.get('options') or {}).get('gnd_layers') or []
    if isinstance(gl,str):gl=gl.split(',')
    gnd=K.get('gnd') or ['GND']
    for label in gl:
        layer=str(label).split('=')[0]
        net=str(label).split('=',1)[1] if '=' in str(label) else gnd[0]
        if net not in gnd:continue
        if any(z['layer']==layer and z['net']==net for z in out.get('zones',[])):continue
        shape=unary_union([Polygon(q) for q in B['outline']]).buffer(-float((R.get('options') or {}).get('edge') or .2))
        for p in getattr(shape,'geoms',[shape]):
            out.setdefault('zones',[]).append(dict(net=net,layer=layer,pts=list(p.exterior.coords),
                holes=[list(h.coords) for h in p.interiors],filled=True,full=True,prio=0,
                local_clearance=None,min_thickness=.1))
    return out


def k20_basis(route):
    out = copy.deepcopy(route)
    if out.get('k20_corridor'):
        for key in ('zones', 'zone_list'):
            out[key] = [z for z in out.get(key, []) if not z.get('r36') and
                        z.get('source') not in ('current_zones', 'gnd_fill')]
    return out


def execute(bj, kj, rj, design_rules=None, gnd=True, folder=None, zone_plan=None):
    from learning import current_zones as SZ
    from learning.copper_kicad_native import library_geometry, verify, native_items
    from analysis.evaluation import rate
    from common.zones import unify
    paths=list(map(Path,(bj,kj,rj)))
    B,K,R=[json.loads(p.read_text()) for p in paths]
    folder=Path(folder or (str(paths[2].with_suffix(''))+'_current_zones'))
    folder.mkdir(parents=True,exist_ok=True)
    rg=design_rules or (R.get('options') or {}).get('design_rules')
    RG=json.loads(Path(rg).read_text()) if rg and Path(rg).exists() else None
    plan=json.loads(Path(zone_plan).read_text()) if zone_plan else None
    if plan:
        from learning.zone_planner import sha, check_plan, check_inputs
        check_plan(plan)
        if plan['routing_board_sha256'] != sha(B) or plan['input_hashes']['netclasses'] != sha(K) or plan['input_hashes']['design_rules'] != sha(RG):
            raise ValueError('Zone plan inputs changed')
        if R.get('zone_plan_id') != plan['id']: raise ValueError('Route belongs to another/no zone plan')
        check_inputs(plan,B,K,RG,R.get('options') or {})
        B=json.loads(Path(plan['board_file']).read_text())
        if sha(B)!=plan['input_hashes']['board']:
            raise ValueError('Native plan pad geometry changed')
        pad_info=dict(source=plan['pad_geometry'])
    else:
        B,pad_info=library_geometry(folder/'library',B,R,RG)
    if plan:
        from learning.zone_planner import via_middle_items
        r37_items=via_middle_items(R)
    old=native_items(R)
    old['zone_list']=[z for z in old.get('zone_list',[]) if z.get('source') not in ('current_zones','gnd_fill','zone_planner')]
    old['vias']=[v for v in old['vias'] if v.get('source') not in ('gnd_fill','zone_planner')]
    old=ground_layers(B,old,K)
    old['current_zones_model']='r36-dc-v1'
    B0,old0=copy.deepcopy(B),copy.deepcopy(old)
    from learning.current_zones_repair import prepare
    if plan:
        repair=dict(applied=False,reason='Local plan: placement/scaffold unchanged')
    else:
        B,old,repair=prepare(B,old,K,RG,folder/'repair')
    new=k20_basis(old)
    if plan:
        import uuid
        from shapely.ops import unary_union
        grouped={}
        for z in plan['core_set']:
            grouped.setdefault((z['net'],z['layer']),[]).append(z)
        for (net,layer),zs in sorted(grouped.items()):
            shape=unary_union([SZ._geo(z) for z in zs])
            contours=[dict(pts=p['outer'],holes=p.get('inner',[])) for p in SZ._polys(shape)]
            ident=net+':'+layer
            for c in contours[:1]:
                new.setdefault('zones',[]).append(dict(net=net,layer=layer,**c,filled=True,full=True,
                    r36=True,current_path=True,source='zone_planner',core_id=ident,prio=1,
                    zone_id=str(uuid.uuid5(uuid.NAMESPACE_URL,plan['id']+ident)),native_layers=[layer],
                    native_outlines=contours,min_thickness=max(z['min_thickness'] for z in zs),local_clearance=None))
        from analysis.areas import contour
        for layer in plan.get('potential_layers',[]):
            ground=(K.get('gnd') or ['GND'])[0]
            shape=contour(B).buffer(-float((R.get('options') or {}).get('edge') or .3))
            for pg in getattr(shape,'geoms',[shape]):
                new.setdefault('zones',[]).append(dict(net=ground,layer=layer,pts=list(pg.exterior.coords),holes=[list(h.coords) for h in pg.interiors],filled=True,full=True,prio=0,min_thickness=.1,source='potential_rest_gnd'))
        new['vias'] += [v for f in plan['via_candidates'] if f['taken'] for v in f['vias']]
    else:
        _zs=SZ.zone_list(B,new,K);SZ.skeleton_apply(new,_zs);new['zone_list']+=_zs
    if gnd:
        gz,gv=SZ.gnd_fill(B,new,K);new['zone_list']+=gz;new['vias']+=gv
    if plan:


        new=unify(new)
        from learning.zone_planner_shape import min_widths
        new=min_widths(B,new,RG,plan)
    (folder/'suggestion.json').write_text(json.dumps(new))
    a,b,native=verify(folder/'kicad_native',B,old,new,RG,cpu_s=240)
    a['native_pad_geometry']=B['pads'];b['native_pad_geometry']=B['pads']
    if repair['applied']:
        a,_,native0=verify(folder/'origin_kicad_native',B0,old0,old0,RG,cpu_s=180)
        native['origin']=native0['before']
    from learning.zones_judge_selection import an as judge_on
    mm=True if judge_on(R) else None
    e0=rate(B0,a,K,v3=True,minimax=mm);e1=rate(B,b,K,v3=True,minimax=mm)
    physics=physics_comparison(B0,a,B,b,K,e0['ir'],e1['ir'],single_branch=bool(plan) or judge_on(R))
    if plan:
        refined=None
    else:
        b,native,physics,refined=refine(B0,a,B,old,K,RG,folder,b,native,physics,gnd)
    if refined is not None:
        new=refined;b['native_pad_geometry']=B['pads'];e1=rate(B,b,K,v3=True,minimax=mm)
    if judge_on(R):


        physics=physics_comparison(B0,a,B,b,K,e0['ir'],e1['ir'],single_branch=True)
    hard=SZ.verify(B,new,K)
    from learning.copper_bars import check_edges
    audit=unify(b)
    for z in audit['zones']:
        if z.get('r36') and z['net'] not in K.get('gnd',[]):z['copper_bars']='R36-Audit'
    edges=check_edges(B,audit,K)
    gates=dict(drc=native['suggestion']['is_valid'],physics=physics['passed'],
               spacing=not hard,connection=not e1['not_connected'])
    good=all(gates.values())


    failures=[]
    if not gates['drc']:
        failures.append('native DRC '+str(dict(native['suggestion']['types']))+
                        ', open_items='+str(native['suggestion']['open_items']))
    if not gates['spacing']:failures.append('Zone spacing '+str(hard[:3]))
    if not gates['connection']:failures.append('Connection '+str(e1['not_connected']))
    if not gates['physics']:failures.append('Physics '+str(physics['worse']+physics['missing']))
    plan_audit=None
    if plan:
        from learning.zone_planner_shape import verify as native_shape, short, copper_shapes
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        checks=[]
        for z in plan['core_set']:
            filled=unary_union([Polygon(q['pts'],q.get('holes',[])) for q in b['zones']
                               if q.get('source')=='zone_planner' and q['net']==z['net'] and q['layer']==z['layer']])
            checks.append(dict(core_id=z['core_id'],shaping=short(native_shape(filled)),
                               core_preserved=filled.buffer(.006).covers(SZ._geo(z))))
        missing_fields=[f['core_id'] for f in plan['via_candidates'] if not f['taken']]
        missing_cores=[q for q in plan['audit']['rejections'] if q['reason'].startswith('Local core:')]
        sw_vias=[v for v in b['vias'] if plan['net_list'].get(v['net'],{}).get('local_without_layer_change')]
        native_shapes=copper_shapes(b)
        from learning.zone_planner_corridor import native_checking
        potential=native_checking(b,plan)
        plan_audit=dict(potential=potential,native_shapes=native_shapes,cores=checks,missing_cores=missing_cores,missing_fields=missing_fields,sw_vias=sw_vias,
                        passed=potential['passed'] and not plan['audit'].get('potential_missing') and not missing_cores and not missing_fields and not sw_vias and all(q['passed'] for q in native_shapes) and all(q['shaping']['passed'] and q['core_preserved'] for q in checks))
        from analysis.rule_metrics import assumption
        rule_gate=assumption(e0['design_rules'],e1['design_rules'])
        plan_audit['rule_gate']=rule_gate
        plan_audit['passed'] &= rule_gate['not_worse']
        good &= plan_audit['passed']
    from learning.zones_judge_selection import an as judge_on
    judge_audit=None
    if judge_on(R):
        from analysis import minimax as MM
        from analysis.rule_metrics import assumption as design_rules_assumption
        rules=design_rules_assumption(e0.get('design_rules'),e1.get('design_rules')); h18=MM.assumption(e0,e1)
        rating=e1.get('score_v3') is not None and e0.get('score_v3') is not None and math.isfinite(e1['score_v3']) and e1['score_v3']<=e0['score_v3']
        judge_audit=dict(design_rules=rules,h18=h18,score_v3_not_worse=rating,
            passed=rules['not_worse'] and h18['not_worse'] and rating)
        gates['zones_judge']=judge_audit['passed'];good &= gates['zones_judge']
        if not gates['zones_judge']:failures.append('Zone judge: native rule/H18/v3 gate not passed')
    gates['zone_plan']=plan_audit['passed'] if plan_audit else True
    if not gates['zone_plan']:failures.append('Local plan: '+str(plan_audit))
    info=dict(applied=good,reason='; '.join(failures),hard_gates=gates,
              scaffold_repair=repair,
              kicad_native=native,physics_gate=physics,spacing=hard,pad_source=pad_info,edges=edges,
              score_v3_before=e0['score_v3_nets'],score_v3_after=e1['score_v3_nets'],
              loss_mW_before=e0['ir']['loss_mW'],loss_mW_after=e1['ir']['loss_mW'],
              current_zones=len(new['zone_list']),artifacts=str(folder),
              soft_than_gate=False)
    if plan: info.update(zone_plan_id=plan['id'],plan_audit=plan_audit,r37_items=r37_items)
    if judge_audit is not None:
        from learning.zones_judge import Service
        info['zones_judge_gate']=judge_audit
        info['zones_judge_selection']=new.get('zones_judge_selection')
        info['zones_judge']=Service((R.get('options') or {}).get('zones_judge_model')).measure(B,b)
    b['current_zones_acceptance']=info
    (folder/'filled.json').write_text(json.dumps(b))
    return b,info


def final(bj, kj, rj, rg, gnd, plan=None):
    route, info = execute(bj, kj, rj, rg, gnd, zone_plan=plan)
    original = json.loads(Path(rj).read_text())
    if original.get('k20_corridor') and info['applied']:
        try:
            from learning.k20_post_run import assumption
            from learning.shaping import receiver_gate_on
            from learning.current_zones import _receiver_allowed
            B, K = [json.loads(Path(p).read_text()) for p in (bj, kj)]
            info['k20_gate'] = assumption(B, original, route, K)
            info['receiver_gate'] = _receiver_allowed(B, K, original, route) if receiver_gate_on(K) else True
            if not info['k20_gate']['taken'] or not info['receiver_gate']:
                info.update(applied=False, reason='K20: no accepted gain without rule regression')
        except Exception as exc:
            info.update(applied=False, reason='K20 check failed', faults=repr(exc))
        route['current_zones_acceptance'] = info
    return route, info


if __name__=='__main__':
    bj,kj,rj,rg,result,off,gnd=sys.argv[1:8]
    plan=sys.argv[8] if len(sys.argv)>8 else None
    try:
        route,info=final(bj,kj,rj,rg or None,gnd=='1',plan)
        Path(off).write_text(json.dumps(route))
    except Exception as exc:
        info=dict(applied=False,reason=str(exc))
    Path(result).write_text(json.dumps(info))
