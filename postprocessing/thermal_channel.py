import copy
import json
import math
from pathlib import Path
import sys


def schedule(island, mains, cu, layer, net, width):
    from shapely.geometry import Polygon, LineString
    from shapely.ops import nearest_points, unary_union
    if not mains: return []
    padshapes = [Polygon(q['outer'], q['hole_list']) for p in cu['pads'] for q in p['polys'].get(layer, [])]
    fixed = unary_union(padshapes).buffer(.01)
    out=[]
    for main in sorted(mains, key=lambda p:(island.distance(p),-p.area,p.bounds))[:3]:
        a,b=nearest_points(island,main); dx,dy=b.x-a.x,b.y-a.y
        d=math.hypot(dx,dy)
        if d < 1e-6: continue

        x,y=a.x-dx/d*width,a.y-dy/d*width
        u,v=b.x+dx/d*width,b.y+dy/d*width
        short=min(abs(u-x),abs(v-y))
        mids=[(x+math.copysign(short,u-x),y+math.copysign(short,v-y)),
              (u-math.copysign(short,u-x),v-math.copysign(short,v-y))]
        for mid in mids:
            line=LineString([(x,y),mid,(u,v)])
            corridor=line.buffer(width/2,cap_style=2,join_style=2)
            if corridor.intersects(fixed): continue
            if not unary_union([Polygon(q['outer'],q['hole_list']) for q in cu['outline']]).covers(corridor):continue
            nets=sorted({t['net'] for t in cu['tracks'] if t['net']!=net and t['layer']==layer
                         and corridor.intersects(LineString(t['pts']).buffer(t['w']/2))})
            if not nets:continue
            entry=dict(net=net,layer=layer,polys=[list(corridor.exterior.coords)],net_list=nets,width=width)
            if entry not in out:out.append(entry)
    return out[:3]


def attempt(context, channel, routes, folder, call, working):
    from kicad.python_call import python_command
    root=Path(__file__).resolve().parents[1]
    def read(p):return json.loads(Path(p).read_text())
    def dump(p,d):Path(p).write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
    board,classes,rules=[Path(context[k]) for k in ('board','netclasses','design_rules')]
    B,K=read(board),read(classes)
    if any(sum(p['net']==net for p in B['pads']) < 2 for net in channel['net_list']):
        raise ValueError('Channel affects copper without reconstructable router targets')
    model=copy.deepcopy(B)
    model.setdefault('keepouts',[]).append(dict(cu_layers=[channel['layer']],polys=channel['polys'],prohibitions=dict(track_list=True,vias=True)))
    bj=folder/'channel.board.json';rj=folder/'channel.routes.json';start=folder/'channel.start.json'
    dump(bj,model)

    call(python_command(root/'kicad/routes_export.py',working,start,'entire=1'),folder/'channel.start.log')
    native=read(start);native.update(options=routes.get('options',{}),res=routes.get('res',.1));dump(start,native)
    options=copy.deepcopy(routes.get('options') or {})
    for key in ('pre','png','labels','history','interim_state','out','fix','further','new','new_nets','kicad_native_basis','kicad_native_zones','start'):
        options.pop(key,None)
    options.update(thermal_island=1,start=str(start),new_nets=','.join(channel['net_list']),netclasses=str(classes),design_rules=str(rules),
                   kicad_native_basis=str(working),kicad_native_zones=str(folder/'original.zones.json'))

    options.update(attempts=0,agent_loop=0,loop=0)
    cmd=[sys.executable,root/'router/route_pf.py',bj,rj,*[k+'='+str(v) for k,v in sorted(options.items())]]
    call(cmd,folder/'channel.route.log')
    after=read(rj)
    if after.get('rest_board') or after.get('rest_placement'):
        raise ValueError('Channel attempt must not change the placement')
    from analysis.evaluation import rate
    from analysis.rule_metrics import measure, assumption, GATE
    old_eval=rate(B,routes,K,v3=True);new_eval=rate(B,after,K,v3=True)
    old=measure(B,routes,K);new=measure(B,after,K)
    gate=assumption(old,new)
    strict=gate['not_worse'] and all(new[k] <= old[k] for k in GATE) and new_eval['score_v3'] <= old_eval['score_v3']
    dump(folder/'channel.rule_gate.json',dict(before=old,after=new,gate_check=gate,strict=strict,evaluation_before=old_eval,evaluation_after=new_eval))
    if not strict:raise ValueError('Channel violates rule/score-v3 gate')
    candidate=folder/'channel.kicad_pcb'

    call(python_command(root/'learning/real_curve_kicad_native.py','import_',working,candidate,rj,folder/'original.zones.json',
                      'cu_layers='+str(B['cu_layers']),'via='+str(options.get('via','.45/.20'))),folder/'channel.import.log')
    return candidate,after
