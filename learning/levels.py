import json
import os
import subprocess
import re
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union
from common import slots as PL


def layer_count(d):
    p = Path(d)
    if p.is_dir(): p = p / 'processed.kicad_pcb'
    text = p.read_text(errors='replace').split('(setup', 1)[0]
    layers = re.search(r'\(layers\s+(.*?)\n\s*\)', text, re.S)
    if not layers: raise ValueError('Copper stackup missing: ' + str(p))
    n = len(re.findall(r'\(\s*\d+\s+(?:"[^"]+"|[^\s()]+)\s+(?:signal|power|mixed|jumper)\b', layers[1]))
    if n < 2 or n % 2: raise ValueError('Incomplete copper stackup: ' + str(p))
    return n


def detect(B, K, data):
    Z = data['zone_list']; U = unary_union([Polygon(q).buffer(0) for q in B['outline'] if len(q) >= 3])
    if U.area <= 0: raise ValueError('Outline without area')
    layers, details = [], []
    for lay in sorted({z['layer'] for z in Z}, key=lambda l: int(l[2:])):
        zz = [z for z in Z if z['layer'] == lay]
        nets = sorted({z['net'] for z in zz})
        q = unary_union([Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in zz])
        cover = q.intersection(U).area / U.area
        foreign = sorted({t['net'] for t in data.get('tracks', []) if t['layer'] == lay and t['net'] and t['net'] not in nets})
        kind = (K.get('net_list', {}).get(nets[0], {}).get('category') if len(nets) == 1 else None)
        gnd = len(nets) == 1 and nets[0] in (K.get('gnd') or []) and not K.get('net_list', {}).get(nets[0], {}).get('kelvin')
        reserve = len(nets) == 1 and (gnd or kind in ('supply', 'current_val')) and cover >= .9 and not foreign
        if reserve: layers.append(lay + '=' + nets[0])
        details.append(dict(layer=lay, nets=nets, outline_fraction=cover, foreign_track_nets=foreign, reserve=reserve,
                            evidence='outline_only', reason='one net, >=90%, no foreign tracks' if reserve else 'Partial area/mixed/foreign track'))
    return layers, Z, details


def off_original(pcb, B, K, out, kicad_native=False):
    if not kicad_native: return _off_original_prev(pcb, B, K, out)
    p = os.path.join(out, 'levels_original.json')
    Path(p).unlink(missing_ok=True)
    r = PL.job(['/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3', os.path.join(os.path.dirname(__file__), 'levels_export.py'), pcb, p, 'native_levels=1'],
                cpu_s=300, log=os.path.join(out, 'levels_export.log'))
    if r['cur_state'] != 'ok' or not os.path.exists(p): raise RuntimeError('Layer export: ' + repr(r))
    data = json.load(open(p))

    refs = {f['ref'] for f in B['fps']}
    expected = sorted([p['ref'], p['num'], p['net'], round(p['x'], 3), round(p['y'], 3)] for p in B['pads'] if p['ref'] in refs)
    original = sorted(data['pads'])
    if len(original) != len(expected) or any(a[:3] != b[:3] or max(abs(a[3]-b[3]), abs(a[4]-b[4])) > 0.002001 for a,b in zip(original,expected)):
        raise ValueError('Original does not match the practice board (pads/nets/positions, tolerance 2 um)')
    if data.get('cu_layers') != B.get('cu_layers'): raise ValueError('Original has a different copper stackup')
    if B.get('cu_layers') in (6, 8):
        layers, zones, details = detect(B, K, data)
        json.dump(dict(cu_layers=B['cu_layers'], gnd_layers=layers, layers=details), open(os.path.join(out, 'levels.json'), 'w'), indent=1)
        return layers, zones
    Z = data['zone_list']; U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3])
    gnd = set(K.get('gnd') or []); layers = []
    for lay in sorted({z['layer'] for z in Z}):
        qs = [Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in Z if z['layer'] == lay and z['net'] in gnd]
        if qs and unary_union(qs).intersection(U).area >= 0.9 * U.area: layers.append(lay)

    layers = [l for l in layers if not any(z['layer'] == l and z['net'] not in gnd for z in Z)]
    return layers, [z for z in Z if z['layer'] in layers and z['net'] in gnd]


def _drc_kicad_native(pcb, KO, pinned=None, new_fill=False):
    from kicad.python_call import python_command
    js = pcb + '.drc.json'; Path(js).unlink(missing_ok=True)


    fill = ['--refill-zones', '--save-board'] if new_fill else []
    from kicad.project_copy import design_rules_save, design_rules_copy
    rules = design_rules_save(pcb)
    r = PL.job([KO.KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, pcb] + fill,
                cpu_s=300, log=pcb + '.drc.log')
    if r['cur_state'] != 'ok' or not Path(js).exists(): return dict(faults=r)
    if new_fill: design_rules_copy(rules, pcb, generated_pro=True)
    geo = PL.job(python_command(Path(KO.R, 'kicad/drc_geometry.py'), pcb, js),
                  cpu_s=300, log=pcb + '.geometry.log')
    if geo['cur_state'] != 'ok': return dict(faults=geo)
    result = KO.drc_evaluate(json.loads(Path(js).read_text()), pinned, kicad_native=True)
    result['levels_cpu_s'] = r['cpu_s'] + geo['cpu_s']
    return result


def original_drc(pcb, out, KO):
    import shutil
    from kicad.project_copy import design_rules_save, design_rules_copy
    shutil.copy2(pcb, out)
    design_rules_copy(design_rules_save(pcb), out)
    return _drc_kicad_native(out, KO)


def k12(B, data):
    from shapely.geometry import box
    out = []
    for f in B.get('fps', []):
        if not any(k in (f.get('fp', '') + f.get('lib', '')).upper() for k in ('BGA', 'CSP')): continue
        pads = [p for p in B['pads'] if p['ref'] == f['ref']]
        if not pads: continue
        field = box(min(p['bbox'][0] for p in pads), min(p['bbox'][1] for p in pads), max(p['bbox'][2] for p in pads), max(p['bbox'][3] for p in pads))
        for layer, net in sorted({(z['layer'], z['net']) for z in data['zone_list']}):
            polys = [Polygon(q, h).buffer(0) for z in data['zone_list'] if (z['layer'], z['net']) == (layer, net) for q, h in z.get('filled', []) if len(q) >= 3]
            if not polys:
                out.append(dict(ref=f['ref'], layer=layer, net=net, unknown='no stored fill')); continue
            G = unary_union(polys).intersection(field)
            pieces = [q for q in getattr(G, 'geoms', [G]) if q.geom_type == 'Polygon' and q.area > .0001]
            out.append(dict(ref=f['ref'], layer=layer, net=net, part_list=len(pieces), copper_mm2=round(G.area, 4), largest_mm2=round(max((q.area for q in pieces), default=0), 4)))
    return out


def drc(pcb, routes, out, info, RG, pinned, KO, kicad_native=False):
    if not kicad_native: return _drc_prev(pcb, routes, out, info, RG, pinned, KO)
    from kicad.python_call import python_command
    zj = out + '.zones.json'; Path(zj).write_text(json.dumps(info.get('gnd_zones') or []))
    via = json.loads(Path(routes).read_text()).get('options', {}).get('via', info['via'])
    Path(out).unlink(missing_ok=True)
    cmd = python_command(Path(KO.R, 'kicad/import_routes.py'), pcb, routes, zj, out,
                       'cu_layers=%d' % info['cu_layers'], 'via=' + via,
                       'zclr=%g' % RG['router'].get('clr', 0.2), 'project=1', 'fill=0')
    r = PL.job(cmd, cpu_s=300, log=out + '.import.log')
    if r['cur_state'] != 'ok' or not os.path.exists(out): return dict(faults='Layer import: ' + repr(r))
    from postprocessing.thermal_island import finish
    def thermal_call(argv, log):
        rr = PL.job(list(map(str,argv)),cpu_s=300,log=str(log))
        if rr['cur_state'] != 'ok': raise RuntimeError('Thermal island: '+repr(rr))
    route_data=json.loads(Path(routes).read_text())
    options=route_data.get('options') or {}
    context=dict(board=Path(routes).with_name('board.json'),netclasses=options.get('netclasses',''),design_rules=options.get('design_rules',''),routes=routes)
    if not all(Path(context[k]).is_file() for k in ('board','netclasses','design_rules')): context=None
    finish(out,route_data,thermal_call,context)
    result = _drc_kicad_native(out, KO, pinned, new_fill=True)
    if result.get('faults'): return result
    from analysis.evaluation import copper_fingerprint
    result['routes_sha256'] = copper_fingerprint(json.loads(Path(routes).read_text()))
    result['levels_cpu_s'] += r['cpu_s']

    ep = out + '.levels.json'; Path(ep).unlink(missing_ok=True)
    rr = PL.job(python_command(Path(__file__).with_name('levels_export.py'), out, ep, 'native_levels=1'),
                 cpu_s=300, log=out + '.levels.log')
    if rr['cur_state'] != 'ok' or not os.path.exists(ep): result['levels_error'] = rr
    else:
        B = json.loads(Path(routes).with_name('board.json').read_text())
        result['k12'] = k12(B, json.loads(Path(ep).read_text()))
        result['levels_cpu_s'] += rr['cpu_s']
    return result



def _off_original_prev(pcb, B, K, out):
    p = os.path.join(out, 'levels_original.json')
    r = subprocess.run(['/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3', os.path.join(os.path.dirname(__file__), 'levels_export.py'), pcb, p],
                       capture_output=True, text=True, timeout=300)
    with open(os.path.join(out, 'levels_export.log'), 'a') as f:
        f.write(pcb + '\n' + r.stdout + r.stderr + '\n')
    if r.returncode != 0 or not os.path.exists(p): raise RuntimeError('Layer export: ' + r.stderr[-500:])
    data = json.load(open(p))
    expected = sorted([p['ref'], p['num'], p['net'], round(p['x'], 3), round(p['y'], 3)] for p in B['pads'])
    original = sorted(data['pads'])
    if len(original) != len(expected) or any(a[:3] != b[:3] or max(abs(a[3]-b[3]), abs(a[4]-b[4])) > 0.002001 for a,b in zip(original,expected)):
        raise ValueError('Original does not match the practice board (pads/nets/positions, tolerance 2 um)')
    Z = data['zone_list']; U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3])
    gnd = set(K.get('gnd') or []); layers = []
    for lay in sorted({z['layer'] for z in Z}):
        qs = [Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in Z if z['layer'] == lay and z['net'] in gnd]
        if qs and unary_union(qs).intersection(U).area >= 0.9 * U.area: layers.append(lay)

    layers = [l for l in layers if not any(z['layer'] == l and z['net'] not in gnd for z in Z)]
    return layers, [z for z in Z if z['layer'] in layers and z['net'] in gnd]


def _drc_prev(pcb, routes, out, info, RG, pinned, KO):
    zj = out + '.zones.json'; json.dump(info['gnd_zones'], open(zj, 'w'))
    via = json.load(open(routes)).get('options', {}).get('via', info['via'])
    cmd = [KO.KPY, os.path.join(KO.R, 'kicad/import_routes.py'), pcb, routes, zj, out,
           'cu_layers=%d' % info['cu_layers'], 'via=' + via, 'zclr=%g' % RG['router'].get('clr', 0.2)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.returncode: return dict(faults='Layer import: ' + r.stderr[-300:])
    js = out + '.drc.json'
    r = subprocess.run([KO.KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, out], capture_output=True, text=True, timeout=300)
    if not os.path.exists(js): return dict(faults='Layer DRC: ' + r.stderr[-300:])
    return KO.drc_evaluate(json.load(open(js)), pinned)
