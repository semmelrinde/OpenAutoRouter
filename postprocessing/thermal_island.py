from collections import Counter
import copy
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile

KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
ROOT = Path(__file__).resolve().parents[1]


def active(routes):
    value = (routes.get('options') or {}).get('thermal_island', '0')
    if str(value) not in ('0', '1'):
        raise ValueError('thermal_island must be 0 or 1')
    return str(value) == '1'


def findings(drc):
    if not all(k in drc for k in ('violations', 'unconnected_items', 'kicad_version')):
        raise ValueError('Incomplete native DRC report')
    return [v for v in drc['violations'] if v.get('type') == 'starved_thermal'
            and 'isolated island' in v.get('description', '')]


def faults(drc):
    def key(v):
        return json.dumps([v.get('type'), v.get('description'),
            sorted((i.get('description', ''), tuple(sorted(i.get('pos', {}).items())))
                   for i in v.get('items', []))], sort_keys=True)
    return Counter(key(v) for v in drc['violations'] if v.get('severity') == 'error'), Counter(key(v) for v in drc['unconnected_items'])


def no_regression(before, after):
    return all(not (b - a) for a, b in zip(faults(before), faults(after)))


def candidates(cu, drc, routes, rules):
    import math
    from codex.bga_fanout.polygon_rings import copper_map
    cu = copper_map(cu)
    from shapely.geometry import Polygon, Point, LineString
    from shapely.ops import unary_union
    from shapely.prepared import prep
    from shapely.strtree import STRtree
    from postprocessing.gnd_spacing import via_free
    def poly(q):
        p = Polygon(q['outer'], q.get('hole_list', []))
        if not p.is_valid: raise ValueError('Invalid native fill geometry')
        return p
    board = unary_union([poly(q) for q in cu['outline']])
    op = routes.get('options') or {}
    rd = rules.get('board', {}).get('design_settings', {}).get('rules', {})
    vd, drill = map(float, str(op.get('via', '.45/.20')).split('/'))
    drill = max(drill, float(rd.get('min_through_hole_diameter') or 0))
    vd = max(vd, float(rd.get('min_via_diameter') or 0), drill + 2 * float(rd.get('min_via_annular_width') or 0))
    if not 0 < drill < vd: raise ValueError('Invalid via contract')
    clearance = max(float(rd.get('min_clearance') or 0), *[float(op.get(k) or 0) for k in ('clr', 'drc', 'drcpad')]) + .01
    h2h = max(float(rd.get('min_hole_to_hole') or 0), float(op.get('h2h') or 0)) + .01
    edge = max(float(rd.get('min_copper_edge_clearance') or 0), float(op.get('edge') or 0)) + .01
    radius = vd / 2
    inner = prep(board.buffer(-(edge + radius)))
    holes = [(v['x'], v['y'], v['drill']/2) for v in cu['vias']] + [(p['x'], p['y'], max(p.get('drill_size') or [p['drill']])/2) for p in cu['pads'] if p['drill']]
    out = []
    for finding in findings(drc):
        ids = {i['uuid'] for i in finding['items']}
        pads = [p for p in cu['pads'] if p.get('pad_id') in ids]
        zones = [z for z in cu['zones'] if z.get('zone_id') in ids]
        layer = re.search(r'\(layer ([^;]+);', finding['description'])
        if not layer or len(pads) != 1: raise ValueError('Thermal finding not uniquely bound to a native pad')
        layer_name=layer[1]
        layer=next((z['layer'] for z in zones if z.get('layer_name')==layer_name),layer_name.replace('.Cu',''))
        pad = pads[0]; net = pad['net']
        shapes = []
        for t in cu['tracks']:
            if t['net'] != net:
                shapes.append(LineString(t['pts']).buffer(t['w']/2))
        for p in cu['pads']:
            if p['net'] != net:
                shapes.extend(poly(q) for ps in p['polys'].values() for q in ps)
        for v in cu['vias']:
            if v['net'] != net: shapes.append(Point(v['x'], v['y']).buffer(v['d']/2))
        for z in cu['zones']:
            if z['net'] != net: shapes.extend(poly(q) for q in z['polys'])
        for f in cu.get('shapes', []):
            if f['net'] != net: shapes.extend(poly(q) for q in f['polys'])
        tree = {0: (STRtree(shapes), shapes)} if shapes else {}
        planes = [poly(q) for z in cu['zones'] if z['net'] == net and z['layer'] not in ('F', 'B') for q in z['polys'] if poly(q).area >= .6 * board.area]
        planes = unary_union(planes).buffer(-radius-.01) if planes else Polygon()
        islands = []
        for z in zones:
            if z['layer'] != layer or layer not in ('F','B'): continue
            for q in z['polys']:
                p = poly(q)
                padshape = unary_union([poly(a) for a in pad['polys'].get(layer, [])])
                if not p.intersects(padshape.buffer(.01)): continue


                others = [poly(a) for pp in cu['pads'] if pp['net'] == net and pp.get('pad_id') != pad['pad_id'] for a in pp['polys'].get(layer, [])]
                others += [Point(v['x'],v['y']).buffer(v['d']/2) for v in cu['vias'] if v['net'] == net]
                others += [LineString(t['pts']).buffer(t['w']/2) for t in cu['tracks'] if t['net'] == net and t['layer'] == layer]
                if any(p.intersects(o) for o in others): continue
                islands.append(p)
        channels = []
        from postprocessing.thermal_channel import schedule
        mains = [poly(q) for z in zones if z['layer'] == layer for q in z['polys']
                 if not any(poly(q).equals(i) for i in islands)]
        for island in islands:
            width=max([.2]+[max(z.get('thermal_spoke',.3),z.get('min_thickness',.1)*1.2) for z in zones if z['layer']==layer])
            channels.extend(schedule(island,mains,cu,layer,net,width))
        chosen = []
        for island in sorted(islands, key=lambda p: (p.area, p.bounds)):
            area = island.buffer(-radius-.01).intersection(planes)
            if area.is_empty: continue
            rp = area.representative_point()
            pts = [(rp.x, rp.y)]
            x0,y0,x1,y1 = area.bounds

            step = max(.2, math.sqrt(max(area.area, (x1-x0)*(y1-y0))/9000))
            pts += [(x0+i*step,y0+j*step) for i in range(int((x1-x0)/step)+1) for j in range(int((y1-y0)/step)+1)]
            for x,y in pts:
                x,y = round(x,5),round(y,5)
                if not area.contains(Point(x,y)): continue
                if not via_free(x,y,inner,tree,holes,[],radius,drill,clearance,h2h): continue
                if any(math.hypot(x-v['x'],y-v['y']) < vd for v in chosen): continue
                chosen.append(dict(x=x,y=y,net=net,d=vd,drill=drill))
                if len(chosen) >= 8: break
            if len(chosen) >= 8: break
        out.append(dict(pad=pad['pad_id'],net=net,layer=layer,islands=len(islands),vias=chosen,
                        channels=channels, reason='Via-Candidates' if chosen else 'no rule-compliant via to the inner main area'))
    return out


def finish(pcb, routes, call, channel_context=None):
    if not active(routes): return None
    from kicad.python_call import python_command
    from kicad.project_copy import design_rules_save, design_rules_copy
    routes=copy.deepcopy(routes)
    pcb = Path(pcb).resolve()
    original_rules = design_rules_save(pcb)
    folder = pcb.parent / (pcb.stem + '_thermal_island')
    folder.mkdir(exist_ok=False)
    report = dict(version='thermal-island-v1', before=None, after=None, taken=0, attempts=[], open_items=[], channel_attempts=[])
    def dump(p, d): Path(p).write_text(json.dumps(d, indent=2, allow_nan=False)+'\n')
    def drc(path, tag, refill=True):
        dst = folder/(tag+'.drc.json')
        fill=['--refill-zones','--save-board'] if refill else []
        call([KC,'pcb','drc',*fill,'--format','json','--severity-all','-o',str(dst),str(path)],folder/(tag+'.drc.log'))
        design_rules_copy(original_rules, path, generated_pro=True)
        return json.loads(dst.read_text())

    working = folder/'working.kicad_pcb'
    shutil.copy2(pcb, working); design_rules_copy(original_rules, working)
    (folder/'.real_curve').touch()
    def definitions(path, tag):
        dst=folder/(tag+'.zones.json')
        call(python_command(ROOT/'learning/real_curve_kicad_native.py','zone_list',path,dst),folder/(tag+'.zones.log'))
        return json.loads(dst.read_text())
    defs = definitions(working,'original')
    before = drc(working,'before'); current=before
    report['before']=len(findings(before))
    try:

        for iteration in range(min(32, len(findings(before)))):
            if not findings(current): break
            cu_path=folder/('cu%d.json'%iteration)
            call(python_command(ROOT/'kicad/copper_export.py',working,cu_path,'polygons_exact=1','thermal_metadata=1'),folder/('cu%d.log'%iteration))
            proposals=candidates(json.loads(cu_path.read_text()),current,routes,json.loads(working.with_suffix('.kicad_pro').read_text()))
            report['open_items']=proposals
            accepted=False
            for proposal in proposals:
                for via in proposal['vias']:
                    idx=len(report['attempts']); vj=folder/('via%d.json'%idx); candidate=folder/('candidate%d.kicad_pcb'%idx)
                    dump(vj,dict(vias=[via]))
                    call(python_command(ROOT/'kicad/vias_set.py',working,vj,candidate,'via=%g/%g'%(via['d'],via['drill'])),folder/('via%d.log'%idx))
                    design_rules_copy(original_rules,candidate)
                    after=drc(candidate,'candidate%d'%idx)
                    ok=(len(findings(after)) < len(findings(current)) and no_regression(current,after)
                        and definitions(candidate,'candidate%d'%idx) == defs)
                    report['attempts'].append(dict(via=via,taken=ok,thermal=len(findings(after))))
                    if ok:
                        shutil.copy2(candidate,working);current=after;report['taken']+=1;accepted=True
                        routes.setdefault('vias',[]).append(dict(via,thermal_island=True))
                        break
                if accepted:break
            if not accepted:


                if channel_context:
                    from postprocessing.thermal_channel import attempt
                    for proposal in proposals:
                        for channel in proposal.get('channels', []):
                            try:
                                candidate,new_routes=attempt(channel_context,channel,routes,folder,call,working)
                                after=drc(candidate,'channel')
                                ok=(len(findings(after))<len(findings(current)) and no_regression(current,after)
                                    and definitions(candidate,'channel')==defs)
                                report['channel_attempts'].append(dict(channel=channel,taken=ok))
                                if ok:
                                    shutil.copy2(candidate,working);current=after;accepted=True
                                    report['taken']+=1
                                    saved=folder/('taken_routes%d.json'%report['taken'])
                                    saved.write_text(json.dumps(new_routes,indent=2,allow_nan=False)+'\n')
                                    report['channel_routes']=str(saved)
                                    routes=new_routes
                                    break
                            except ValueError as exc:
                                report['channel_attempts'].append(dict(channel=channel,taken=False,reason=repr(exc)))
                        if accepted:break
                else:
                    report['channel_attempts'].append(dict(taken=False,reason='Router context missing; finding stays open'))
            if not accepted:break
        current=drc(working if report['taken'] else pcb,'outputs',refill=False)
        report['after']=len(findings(current))
        report['open_items'] = [dict(description=v['description'],items=v['items']) for v in findings(current)]
        if report['taken']:
            shutil.copy2(working,pcb);design_rules_copy(original_rules,pcb)
        if report['taken'] and channel_context and channel_context.get('routes'):
            dump(channel_context['routes'],routes)
            report['routes_file']=str(channel_context['routes'])
        dump(folder/'report.json',report)
        return report
    except Exception as exc:
        report['faults']=repr(exc);dump(folder/'report.json',report);raise
