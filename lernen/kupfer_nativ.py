""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import resource
import time

ROOT = Path(__file__).resolve().parents[1]
KPY = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'


def run(cmd, log, cpu_s=45):
    from gemeinsam import plaetze as PL
    result = PL.lauf(cmd, cpu_s=cpu_s, wand_s=10*cpu_s,
                     platz_wand_s=10*cpu_s, log=str(log), cwd=ROOT)
    Path(str(log) + '.lauf.json').write_text(json.dumps(result, indent=2))
    if result['zustand'] != 'ok':
        raise RuntimeError('Native Operation fehlgeschlagen: ' + str(result) + '\n' + Path(log).read_text()[-1000:])


def native_items(R):
    ""
    import math
    from gemeinsam.bahnitems import native_arc_representable, validate
    out=copy.deepcopy(R)
    for t in out['tracks']:
        items=[]
        for it in t.get('items',[]):
            if it[0]!='A' or native_arc_representable(it):
                items.append(it);continue
            a,m,b=it[1:4]
            length=math.dist(a,b)
            dev=abs((m[0]-a[0])*(b[1]-a[1])-(m[1]-a[1])*(b[0]-a[0]))/max(length,1e-30)
            if dev>1e-6 or math.dist(a,m)+math.dist(m,b)>length+2e-6:
                raise ValueError('Nicht darstellbarer Bogen mit realem Kupferunterschied')
            for x,y in ((a,m),(m,b)):
                if math.dist(x,y)>1e-12:items.append(['L',x,y,it[-1]])
        if 'items' in t:
            t['items']=items
            validate(t,error=float((R.get('organisch_nachlauf') or {}).get('sehnenfehler_mm',.00025)))
    return out


def regelvertrag(B, R, regeln, project=None, dru=''):
    ""
    from lernen.kupferbalken import _regeln
    p = copy.deepcopy(project or {})


    p.setdefault('meta', {'filename': 'board.kicad_pro', 'version': 3})
    rules = p.setdefault('board', {}).setdefault('design_settings', {}).setdefault('rules', {})
    aliases = {'min_breite':'min_track_width', 'min_abstand':'min_clearance', 'rand':'min_copper_edge_clearance',
               'via_min':'min_via_diameter', 'bohr_min':'min_through_hole_diameter', 'ring_min':'min_via_annular_width',
               'loch_loch':'min_hole_to_hole', 'loch_abstand':'min_hole_clearance'}


    for key in aliases.values(): rules.setdefault(key, 0.0)
    values = {aliases[k]:v for k,v in (B.get('design') or {}).items() if k in aliases and v is not None}
    values.update({k:v for k,v in (regeln or {}).get('roh', {}).items() if k.startswith('min_') and isinstance(v, (int,float))})
    clr, edge, hole = _regeln(R, regeln)
    from gemeinsam.zonen import aktiv
    if aktiv(R):

        op=R.get('optionen') or {}; rg=(regeln or {}).get('router',regeln or {})
        clr=max(float(q.get(k) or 0) for q,k in ((R,'clr'),(op,'clr'),(op,'drc'),(rg,'clr'),(rg,'drc'),((regeln or {}).get('roh',{}),'min_clearance')))
    else:


        op=R.get('optionen') or {}; rg=(regeln or {}).get('router',regeln or {})
        harte=[float(q[k]) for q,k in ((op,'drc'),(rg,'drc'),((regeln or {}).get('roh',{}),'min_clearance')) if q.get(k) is not None]
        if harte: clr=max(harte)
    values.update(min_clearance=max(clr, values.get('min_clearance', 0)),
                  min_copper_edge_clearance=max(edge, values.get('min_copper_edge_clearance', 0)),
                  min_hole_clearance=max(hole, values.get('min_hole_clearance', 0)))
    for k,v in values.items(): rules[k] = max(float(rules.get(k) or 0), v)
    ns = p.setdefault('net_settings', {})
    classes = ns.setdefault('classes', [])
    fields = {'clr':'clearance', 'w':'track_width', 'via_d':'via_diameter', 'via_b':'via_drill'}
    for name, vals in sorted((B.get('netzklassen') or {}).items()):
        c = next((c for c in classes if c.get('name') == name), None)
        if c is None:
            c = {'name':name}; classes.append(c)
        for k,v in vals.items():
            if k in fields and v is not None: c[fields[k]] = max(float(c.get(fields[k]) or 0), v)
    if not any(c.get('name') == 'Default' for c in classes): classes.append({'name':'Default', 'clearance':clr, 'track_width':0, 'via_diameter':0, 'via_drill':0})
    patterns = ns.setdefault('netclass_patterns', [])
    for net, name in sorted((B.get('netclass') or {}).items()):
        if not any(q.get('pattern') == net for q in patterns): patterns.append({'netclass':name, 'pattern':net})
    if not dru.strip(): dru = '(version 1)\n'
    minima = {'clearance':rules['min_clearance'], 'edge_clearance':rules['min_copper_edge_clearance'],
              'hole_clearance':rules['min_hole_clearance']}
    dru = _router_minima(dru, minima)
    if True:
        pad_min=max(float((q or {}).get('drcpad') or 0) for q in (R.get('optionen'), (regeln or {}).get('router')))
        if pad_min>clr:
            import re
            own=list(re.finditer(r'\(rule "Stromform Router [^"]+" \(constraint [a-z_]+ \(min [^)]+\)\)\)',dru))
            pos=own[-1].end() if own else re.search(r'\(version\s+1\)',dru).end()
            floor="\n(rule \"R36 Pad-Abstand\" (condition \"A.Type == 'Pad' || B.Type == 'Pad'\") (constraint clearance (min %.9gmm)))\n" % pad_min
            dru=dru[:pos]+floor+dru[pos:]
    return p, dru


def _router_minima(dru, minima):
    ""
    import re
    tokens = list(re.finditer(r'"(?:[^"\\]|\\.)*"|\#[^\n]*|[()]|[^\s()]+', dru))
    stack = []; nodes = []
    for token in tokens:
        value = token.group()
        if value.startswith('#'): continue
        if value == '(':
            node = {'start':token.start(), 'parts':[]}
            if stack: stack[-1]['parts'].append(node)
            stack.append(node)
        elif value == ')':
            if not stack: raise ValueError('Ungueltige native Regelklammer')
            node = stack.pop(); node['end'] = token.end(); nodes.append(node)
        elif stack: stack[-1]['parts'].append(token)
    if stack: raise ValueError('Unvollstaendige native Regeln')
    def text(token): return token.group().strip('"')
    changes = []
    for node in nodes:
        parts = node['parts']
        if len(parts)<2 or isinstance(parts[0],dict) or text(parts[0])!='constraint': continue
        kind = text(parts[1]); floor = minima.get(kind)
        if floor is None: continue
        minimum = next((x for x in parts[2:] if isinstance(x,dict) and x['parts'] and text(x['parts'][0])=='min'),None)
        if minimum is None:
            changes.append((node['end']-1,node['end']-1,' (min %.9gmm)' % floor));continue
        vv = minimum['parts'][1:]
        if not vv or any(isinstance(v,dict) for v in vv): raise ValueError('Nicht interpretierbares Regelminimum')
        literal = ' '.join(text(v) for v in vv)
        match = re.fullmatch(r'([0-9]+(?:\.[0-9]*)?|\.[0-9]+)\s*(mm|mil|in|um)?',literal)
        if not match: raise ValueError('Nicht interpretierbares Regelminimum: '+literal)
        value = float(match[1])*{None:1,'mm':1,'mil':.0254,'in':25.4,'um':.001}[match[2]]
        if value < floor:
            changes.append((vv[0].start(),vv[-1].end(),'%.9gmm' % floor))
    for start,end,value in sorted(changes,reverse=True): dru=dru[:start]+value+dru[end:]
    version = re.search(r'\(version\s+1\)',dru)
    if not version: raise ValueError('Native Regelversion 1 fehlt')
    floors = ''.join('\n(rule "Stromform Router %s" (constraint %s (min %.9gmm)))\n' % (k,k,v)
                     for k,v in minima.items())
    return dru[:version.end()]+floors+dru[version.end():]


def _identisch(B, actual):
    ""
    from collections import Counter
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    def pads(b):
        return Counter((p['ref'], str(p['num']), p['net'], round(p['x'],4), round(p['y'],4), tuple(sorted(p['layers'])))
                       for p in b['pads'] if not str(p['ref']).startswith(('NTBAR_',))
                       and (p.get('layers') or p.get('drill') or p.get('net') or str(p.get('num') or '')))
    if pads(B) != pads(actual) or int(B['lagen']) != int(actual['lagen']):
        raise ValueError('Native Basis passt nicht zu den deklarierten Padinstanzen/Lagen')
    from lernen.kupferbalken import pad_polygon
    for p in B['pads']:
        if str(p['ref']).startswith('NTBAR_'): continue
        candidates = [q for q in actual['pads'] if q['ref']==p['ref'] and str(q['num'])==str(p['num'])
                      and q['net']==p['net'] and abs(q['x']-p['x'])<1e-4 and abs(q['y']-p['y'])<1e-4]
        for layer, polygons in p.get('polys',{}).items():
            expected = unary_union([pad_polygon(B,p,q) for q in polygons])
            if not any(expected.hausdorff_distance(unary_union([Polygon(q) for q in candidate.get('polys',{}).get(layer,[])])) <= .006
                       for candidate in candidates):
                raise ValueError('Native Padform weicht vom Boardvertrag ab: '+p['ref']+':'+str(p['num']))
    def outline(b): return unary_union([Polygon(q) for q in b['outline']])
    if outline(B).symmetric_difference(outline(actual)).area > 1e-4:
        raise ValueError('Native Basis passt nicht zum deklarierten Umriss')


def bibliotheksgeometrie(folder, B, R=None, regeln=None):
    ""
    if not str(B.get('src', '')).startswith('aufgabe seed='):
        return B, {}
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    bj, pcb, export = [folder / q for q in ('board.json', 'basis.kicad_pcb', 'native.json')]
    bj.write_text(json.dumps(B))
    run([KPY, str(ROOT/'kicad/strom_board.py'), str(bj), str(pcb)], folder/'aufbau.log')
    run([KPY, str(ROOT/'kicad/export_board.py'), str(pcb), str(export)], folder/'export.log')
    actual = json.loads(export.read_text())


    project, dru = regelvertrag(B, dict(R or {}, stromzonen_modell='r36-dc-v1'), regeln)
    pcb.with_suffix('.kicad_pro').write_text(json.dumps(project))
    pcb.with_suffix('.kicad_dru').write_text(dru)
    new = copy.deepcopy(B)
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    changes = []
    for p in new['pads']:
        candidates = [q for q in actual['pads'] if q['ref']==p['ref'] and str(q['num'])==str(p['num'])
                      and q['net']==p['net'] and set(q['layers'])==set(p['layers'])
                      and abs(q['x']-p['x'])<1e-4 and abs(q['y']-p['y'])<1e-4]
        if len(candidates) != 1:
            raise ValueError('Bibliothekspad nicht eindeutig bei gleicher Position: '+p['ref']+':'+str(p['num']))
        q = candidates[0]
        distance = max((unary_union([Polygon(x) for x in ps]).hausdorff_distance(
            unary_union([Polygon(x) for x in q['polys'][l]])) for l, ps in p['polys'].items()), default=0)
        changes.append(dict(ref=p['ref'], num=p['num'], konturabweichung_mm=distance))
        p.update(polys=q['polys'], bbox=q['bbox'], drill=q['drill'])
    new['src'] = str(pcb)
    new['aufgabe_ursprung'] = B['src']


    new['keepouts'] = [k for k in B.get('keepouts', []) if not k.get('nativ')] + [dict(k,nativ=True) for k in actual.get('keepouts', [])]
    _identisch(new, actual)
    return new, dict(quelle='native KiCad-Bibliothek bei unveraenderten Padpositionen',
                     max_konturabweichung_mm=max((q['konturabweichung_mm'] for q in changes), default=0), pads=changes)


def pruefen(folder, B, vorher, nachher, regeln=None, cpu_s=120):
    t0 = time.process_time()
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    child0 = usage.ru_utime + usage.ru_stime
    from gemeinsam.zonen import aktiv
    child_ceiling = 120 if aktiv(vorher) or aktiv(nachher) else 45
    def native_run(cmd, log):
        usage = resource.getrusage(resource.RUSAGE_CHILDREN)
        used = time.process_time()-t0+usage.ru_utime+usage.ru_stime-child0
        left = cpu_s-used
        if left < 1: raise RuntimeError('Native Gesamt-CPU-Budget erschoepft')
        run(cmd, log, cpu_s=max(1,int(min(child_ceiling,left))))
    from gemeinsam.zonen import vereinheitlichen
    vorher, nachher = vereinheitlichen(vorher), vereinheitlichen(nachher)
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    base = folder / 'base.kicad_pcb'
    src = Path(str(B.get('src') or ''))
    if src.is_file() and src.resolve().is_relative_to(folder.resolve()):
        raise ValueError('Native Quelle darf nicht im Ausgabeordner liegen')
    for ext in ('.kicad_pro','.kicad_dru'):
        base.with_suffix(ext).unlink(missing_ok=True)
    if src.is_file() and src.suffix == '.kicad_pcb':
        shutil.copyfile(src, base)
        for ext in ('.kicad_pro', '.kicad_dru'):
            if src.with_suffix(ext).exists(): shutil.copyfile(src.with_suffix(ext), base.with_suffix(ext))
    else:


        if not B.get('fps'): raise ValueError('Keine native Quelle und keine rekonstruierbaren Footprints')
        bj = folder / 'board.json'; bj.write_text(json.dumps(B))
        native_run([KPY, str(ROOT/'kicad/strom_board.py'), str(bj), str(base)], folder/'basis.log')

    rule_src = Path(str((regeln or {}).get('quelle') or ''))
    pp = base.with_suffix('.kicad_pro')
    dp = base.with_suffix('.kicad_dru')
    if not (src.is_file() and src.suffix == '.kicad_pcb') and rule_src.suffix in ('.kicad_pcb','.kicad_pro'):
        if rule_src.with_suffix('.kicad_pro').exists(): shutil.copyfile(rule_src.with_suffix('.kicad_pro'), pp)
        if rule_src.with_suffix('.kicad_dru').exists(): shutil.copyfile(rule_src.with_suffix('.kicad_dru'), dp)
    secured = ((src.is_file() and src.suffix == '.kicad_pcb' and src.with_suffix('.kicad_pro').exists()) or
               (rule_src.suffix in ('.kicad_pcb','.kicad_pro') and rule_src.with_suffix('.kicad_pro').exists()))
    project = json.loads(pp.read_text()) if secured and pp.exists() else {}
    merged = dict(vorher, optionen=dict(vorher.get('optionen') or {}))
    for k in ('clr','drc','drcpad','edge','h2h'):
        merged['optionen'][k] = max(float((q.get('optionen') or {}).get(k, 0)) for q in (vorher,nachher))
    project, dru = regelvertrag(B, merged, regeln, project, dp.read_text() if dp.exists() else '')
    pp.write_text(json.dumps(project)); dp.write_text(dru)
    zb = folder/'base_board.json'
    native_run([KPY, str(ROOT/'kicad/export_board.py'), str(base), str(zb)], folder/'basis_export.log')
    _identisch(B, json.loads(zb.read_text()))
    from lernen.kupferbalken import lagen, via_size
    results = {}; filled = {}
    for tag, route in [('vorher', vorher), ('vorschlag', nachher)]:
        native_route = native_items(route)
        vd, drill = via_size(route)
        for v in native_route.get('vias', []):
            if set(v.get('layers') or lagen(B)) != set(lagen(B)):
                raise ValueError('Native Uebernahme unterstuetzt nur deklarierte Durchgangsvias')
            v.update(d=v.get('d',v.get('diameter',vd)),drill=v.get('drill',drill))
        native_route['native_vias'] = True
        rj = folder/(tag+'.json'); rj.write_text(json.dumps(native_route))

        import uuid
        zones = [{**z, 'prio':z.get('prio',1)} for z in route.get('zones', [])]
        for zi,z in enumerate(zones):
            z.setdefault('zone_id',str(uuid.uuid5(uuid.NAMESPACE_URL, 'autorouter:'+str(zi)+':'+json.dumps(z,sort_keys=True))))

        native_route['zones'] = zones
        rj.write_text(json.dumps(native_route))
        zj = folder/(tag+'_zones.json'); zj.write_text(json.dumps(zones))
        pcb = folder/(tag+'.kicad_pcb')
        via = str((route.get('optionen') or {}).get('via','0.45/0.20'))
        native_run([KPY,str(ROOT/'kicad/import_routes.py'),str(base),str(rj),str(zj),str(pcb),
             'lagen='+str(B['lagen']), 'via='+via], folder/(tag+'_import.log'))
        drc = folder/(tag+'_drc.json')
        native_run([KC,'pcb','drc','--format','json','--severity-error','--all-track-errors','-o',str(drc),str(pcb)],folder/(tag+'_drc.log'))
        data = json.loads(drc.read_text())
        results[tag] = dict(fehler=len(data['violations']),offen=len(data.get('unconnected_items',[])),
                            gueltig=not data['violations'] and not data.get('unconnected_items',[]),
                            typen=__import__('collections').Counter(v['type'] for v in data['violations']))
        export = folder/(tag+'_filled.json')
        native_run([KPY,str(ROOT/'kicad/routes_export.py'),str(pcb),str(export),'voll=1'],folder/(tag+'_export.log'))
        native = json.loads(export.read_text())


        source_zones = {z['zone_id']:z for z in zones}
        for z in native['zones']:
            source = source_zones[z['zone_id']]
            if source['net'] != z['net'] or z['layer'] not in (source.get('native_layers') or [source['layer']]):
                raise ValueError('Native Zonenherkunft nicht eindeutig')

            for key in ('kupferbalken', 'strompfad', 'track', 'polygon_dc', 'r36', 'quelle', 'zonen_quelle', 'kern_id'):
                if key in source: z[key] = source[key]
            z['full'] = z['pad_connection'] == 2
            z.update(native_fuellung=True, modell=False)
        filled[tag] = dict(native_route, zones=native['zones'], zonen=[],
                           conflicts={'native:%d'%i:q['type'] for i,q in enumerate(data['violations'])},
                           failed=['native:%d'%i for i in range(len(data.get('unconnected_items',[])))], kupferbalken={'modell':'native Neufuellung'},
                           native_drc=results[tag], drc=dict(version='drc-v1', fehler=results[tag]['fehler'],
                               offen=results[tag]['offen'], routing=results[tag]['fehler'], unverbunden=results[tag]['offen']))
    info = dict(**results, native_vorschlag_gueltig=results['vorschlag']['gueltig'],
                nicht_mehr_fehler=results['vorschlag']['fehler']<=results['vorher']['fehler'],
                nicht_mehr_offen=results['vorschlag']['offen']<=results['vorher']['offen'],
                regel_sha256=hashlib.sha256(pp.read_bytes()+dp.read_bytes()).hexdigest())
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    info['cpu_s'] = time.process_time()-t0+usage.ru_utime+usage.ru_stime-child0
    (folder/'vergleich.json').write_text(json.dumps(info,indent=1))
    return filled['vorher'], filled['vorschlag'], info
