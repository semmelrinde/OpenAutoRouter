"""Read design rules from a KiCad project (.kicad_pro, .kicad_dru) into rules.json for placer and router."""
import json, os, re, sys, time, shutil, math

ROOT = os.path.expanduser('~/KiCad/10.0/projects')
SKIP = re.compile(r'(/\.history/|/backup|/archive|/validation|/verifier|-backups/|/_)', re.I)

KICAD_SPEC = dict(min_clearance=0.0, min_track_width=0.0, min_copper_edge_clearance=0.5, min_hole_clearance=0.25,
                     min_hole_to_hole=0.25, min_through_hole_diameter=0.3, min_via_diameter=0.5, min_via_annular_width=0.1)
KICAD_NET_CLASS = dict(clearance=0.2, track_width=0.2, via_diameter=0.6, via_drill=0.3)



SIGW_BY_LAYERS = {2: 0.15, 4: 0.15, 6: 0.10, 8: 0.10}


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def latest_board():
    best = None
    for d, dirs, files in os.walk(ROOT):
        dirs[:] = sorted(x for x in dirs if not x.startswith('.') and not SKIP.search(os.path.join(d, x) + '/'))
        for f in sorted(files):
            if not f.endswith('.kicad_pcb') or f.startswith(('~', '_')): continue
            p = os.path.join(d, f)
            if SKIP.search(p) or os.path.islink(p): continue
            pro = p[:-len('.kicad_pcb')] + '.kicad_pro'
            if not os.path.exists(pro): continue
            t = os.path.getmtime(p)
            if best is None or t > best[0]: best = (t, p)
    return best[1] if best else None


def own_design_rules(pcb):
    base = pcb[:-len('.kicad_pcb')] if pcb.endswith('.kicad_pcb') else pcb[:-len('.kicad_pro')]
    if not os.path.exists(base + '.kicad_pro'): return False, 'no .kicad_pro'
    if dru_rules(base + '.kicad_dru'): return True, '.kicad_dru with %d rules' % len(dru_rules(base + '.kicad_dru'))
    try:
        pro = json.load(open(base + '.kicad_pro'))
    except (OSError, ValueError):
        return False, '.kicad_pro not readable'
    ru = ((pro.get('board') or {}).get('design_settings') or {}).get('rules') or {}
    different = sorted(k for k, v in KICAD_SPEC.items() if k in ru and abs(float(ru[k]) - v) > 1e-9)
    dflt = {c.get('name'): c for c in (pro.get('net_settings') or {}).get('classes', [])}.get('Default', {})
    different += sorted('Default.' + k for k, v in KICAD_NET_CLASS.items() if dflt.get(k) is not None and abs(float(dflt[k]) - v) > 1e-9)
    if different: return True, 'deviating from the KiCad default: ' + ', '.join(different)
    return False, 'KiCad defaults only (no .kicad_dru)'


def template_write(template_pcb, goal_pcb):
    goal = os.path.abspath(goal_pcb)
    if goal.startswith(os.path.expanduser('~/KiCad') + os.sep):
        raise SystemExit('design_rules.py: never write to a KiCad project under ~/KiCad (%s); working copies only' % goal)
    vb, zb = template_pcb[:-len('.kicad_pcb')], goal[:-len('.kicad_pcb')]
    V = json.load(open(vb + '.kicad_pro'))
    Z = json.load(open(zb + '.kicad_pro')) if os.path.exists(zb + '.kicad_pro') else {}
    vds = V['board']['design_settings']; zds = Z.setdefault('board', {}).setdefault('design_settings', {})
    zds['rules'] = dict(zds.get('rules') or {}, **vds.get('rules', {}))
    for k in ('via_dimensions', 'track_widths'):
        if k in vds: zds[k] = vds[k]
    vd = {c.get('name'): c for c in (V.get('net_settings') or {}).get('classes', [])}.get('Default', {})
    zk = Z.setdefault('net_settings', {}).setdefault('classes', [])
    zd = next((c for c in zk if c.get('name') == 'Default'), None)
    if zd is None: zd = {'name': 'Default'}; zk.insert(0, zd)
    for k in ('clearance', 'track_width', 'via_diameter', 'via_drill', 'microvia_diameter', 'microvia_drill'):
        if k in vd: zd[k] = vd[k]
    json.dump(Z, open(zb + '.kicad_pro', 'w'), indent=2)
    if os.path.exists(vb + '.kicad_dru'): shutil.copyfile(vb + '.kicad_dru', zb + '.kicad_dru')
    return zb + '.kicad_pro'


def sexp(text):


    toks = re.findall(r'"(?:[^"\\]|\\.)*"|#[^\n]*|\(|\)|[^\s()"#]+', text)
    stack = [[]]
    for t in toks:
        if t.startswith('#'): continue
        if t == '(': stack.append([])
        elif t == ')':
            if len(stack) == 1: raise ValueError('Unbalanced S-expression')
            x = stack.pop(); stack[-1].append(x)
        else: stack[-1].append(t[1:-1] if t.startswith('"') else t)
    if len(stack) != 1: raise ValueError('Unbalanced S-expression')
    return stack[0]


def mmval(s):
    m = re.fullmatch(r'([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)\s*(mm|mil|in)?', s)
    if not m: return None
    v = float(m.group(1))
    return v * (0.0254 if m.group(2) == 'mil' else 25.4 if m.group(2) == 'in' else 1)

def dru_rules(path):
    if not os.path.exists(path): return []
    out = []
    for r in sexp(open(path).read()):
        if not isinstance(r, list) or not r or r[0] != 'rule': continue
        d = dict(name=r[1], cond='', layer=None, constraints={}, constraints_expression={})
        for x in r[2:]:
            if not isinstance(x, list) or not x: continue
            if x[0] == 'condition': d['cond'] = x[1]
            elif x[0] == 'severity': d['severity'] = x[1]
            elif x[0] == 'layer': d['layer'] = x[1]
            elif x[0] == 'severity': d['severity'] = x[1]
            elif x[0] == 'constraint':
                c = {}; raw = {}
                for y in x[2:]:
                    if isinstance(y, list) and len(y) >= 2:
                        c[y[0]] = mmval(y[1]); raw[y[0]] = ' '.join(y[1:])
                d['constraints'][x[1]] = c
                d['constraints_expression'][x[1]] = raw
        out.append(d)
    return out


def dru_global(path):
    values = {}; conditional = []
    for rule in dru_rules(path):
        if rule['cond'] or rule['layer']:
            conditional.append(rule['name']); continue
        for kind, constraint in rule['constraints'].items():
            if constraint.get('min') is not None: values[kind] = constraint['min']
    return values, conditional


def border_layer(layer):


    non_cu={'F.Courtyard','B.Courtyard','F.CrtYd','B.CrtYd','F.Fab','B.Fab',
            'F.SilkS','B.SilkS','F.Silkscreen','B.Silkscreen','F.Mask','B.Mask',
            'F.Paste','B.Paste','F.Adhes','B.Adhes','F.Adhesive','B.Adhesive',
            'Dwgs.User','Cmts.User','Eco1.User','Eco2.User','Margin'}
    return layer not in non_cu


def border_stroke_width(pcb, rules):
    if not any(r.get('severity')!='ignore' and border_layer(r.get('layer')) and 'physical_clearance' in r['constraints'] for r in rules): return None
    with open(pcb,encoding='utf-8') as f: todo=sexp(f.read())
    largest=0.; found=False
    shapes={'gr_line','gr_arc','gr_rect','gr_poly','gr_circle','gr_curve',
            'fp_line','fp_arc','fp_rect','fp_poly','fp_circle','fp_curve'}
    while todo:
        node=todo.pop()
        if not isinstance(node,list) or not node:continue
        children={n[0]:n[1:] for n in node[1:] if isinstance(n,list) and n}
        if children.get('layer',[None])[0]=='Edge.Cuts':
            if node[0] not in shapes or children.get('fill',['none'])[0]!='none':
                raise ValueError('Physical edge rule: unmodeled Edge.Cuts graphic '+str(node[0]))
            width=children.get('width',[None])[0]
            for item in children.get('stroke',[]):
                if isinstance(item,list) and item and item[0]=='width':width=item[1]
            width=mmval(str(width))
            if width is None or not math.isfinite(width) or width<0:
                raise ValueError('Physical edge rule: no assigned Edge.Cuts line width')
            largest=max(largest,width);found=True
        todo.extend(n for n in node[1:] if isinstance(n,list))
    if not found:raise ValueError('Physical edge rule without Edge.Cuts geometry')
    return largest


def edge_rules(board_min, rules, edge_width_mm=None):
    minimum = float(board_min)
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError('Invalid board copper edge spacing')
    records = []
    for rule in rules:
        layer = rule.get('layer')
        if not border_layer(layer): continue
        for kind in ('edge_clearance', 'physical_clearance'):
            c = rule['constraints'].get(kind)
            if c is None: continue
            value = c.get('min')
            if rule.get('severity')=='ignore':
                if value is not None and not math.isfinite(value):raise ValueError('Invalid edge minimum: '+rule['name'])
                records.append(dict(name=rule['name'],condition=rule['cond'],layer=layer,
                                    constraint=kind,min_mm=value,edge_min_mm=None,severity='ignore'))
                continue
            if value is None:
                raise ValueError('Edge rule without evaluable minimum: ' + rule['name'])
            if not math.isfinite(value):
                raise ValueError('Invalid edge minimum: ' + rule['name'])
            bound=value
            if kind=='physical_clearance':
                if edge_width_mm is None or not math.isfinite(edge_width_mm) or edge_width_mm<0:
                    raise ValueError('Physical edge rule needs an assigned Edge.Cuts line width')
                bound+=edge_width_mm/2
            records.append(dict(name=rule['name'], condition=rule['cond'], layer=layer,
                                constraint=kind, min_mm=value, edge_min_mm=bound, severity=rule.get('severity', 'error')))

            minimum = max(minimum, bound)
    return dict(version='edge_rules-hull-v2', mode='conservative_hull_limit',
                board_min_mm=float(board_min), edge_mm=minimum, design_rules=records,
                physical_edge_width_mm=edge_width_mm,
                conditional=any(r['condition'] or r['layer'] for r in records))


def layers_off_pcb(pcb):
    try:
        head = open(pcb, encoding='utf-8', errors='ignore').read(200000)
    except OSError:
        return None
    m = re.search(r'\(layers(.*?)\n\t\)', head, re.S)
    if not m: return None
    return len(re.findall(r'"(F|B|In\d+)\.Cu"', m.group(1))) or None


def read(source='latest', cu_layers=None, structure=False):
    pcb = latest_board() if source in (None, '', 'latest') else source
    if pcb.endswith('.kicad_pro'): pcb = pcb[:-len('.kicad_pro')] + '.kicad_pcb'
    base = pcb[:-len('.kicad_pcb')]
    pro = json.load(open(base + '.kicad_pro'))
    ds = pro['board']['design_settings']; ru = ds['rules']
    nc = {c['name']: c for c in pro.get('net_settings', {}).get('classes', [])}
    dflt = nc.get('Default', {})
    rules = dru_rules(base + '.kicad_dru')
    global_dru, conditional_dru = dru_global(base + '.kicad_dru')
    border = edge_rules(ru.get('min_copper_edge_clearance', 0.5), rules,border_stroke_width(pcb,rules))
    raw_data = dict(min_clearance=ru.get('min_clearance', 0.2), min_track_width=ru.get('min_track_width', 0.2),
               min_copper_edge_clearance=ru.get('min_copper_edge_clearance', 0.5), min_hole_clearance=ru.get('min_hole_clearance', 0.25),
               min_hole_to_hole=ru.get('min_hole_to_hole', 0.25), min_through_hole_diameter=ru.get('min_through_hole_diameter', 0.3),
               min_via_diameter=ru.get('min_via_diameter', 0.5), min_via_annular_width=ru.get('min_via_annular_width', 0.1),
               netclass_clearance=dflt.get('clearance'), netclass_track=dflt.get('track_width'),
               netclass_via=[dflt.get('via_diameter'), dflt.get('via_drill')],
               via_dimensions=[v for v in ds.get('via_dimensions', []) if v.get('diameter')])
    pad_track = pad_pad_smd = None; via_min_by_drill = []
    for r in rules:
        c = r['cond'].replace(' ', '')
        cl = r['constraints'].get('clearance', {}).get('min')
        if cl is not None and not r['layer']:
            if not c:

                pad_track = pad_pad_smd = cl
            if c == "A.Type=='Pad'&&B.Type=='Track'": pad_track = cl
            if c == "A.Pad_Type=='SMD'&&B.Pad_Type=='SMD'": pad_pad_smd = cl
        vd = r['constraints'].get('via_diameter', {}).get('min')
        m = re.search(r"A\.Hole_Size_X<([0-9.]+)mm", c)
        if vd is not None and not r['layer'] and "A.Type=='Via'" in c and m:
            via_min_by_drill.append((float(m.group(1)), vd))
    raw_data.update(pad_track=pad_track, pad_pad_smd=pad_pad_smd, via_min_by_drill=via_min_by_drill)
    for kind, key in (('clearance', 'min_clearance'), ('track_width', 'min_track_width'),
                      ('edge_clearance', 'min_copper_edge_clearance'), ('hole_clearance', 'min_hole_clearance'),
                      ('hole_to_hole', 'min_hole_to_hole'), ('hole_size', 'min_through_hole_diameter'),
                      ('via_diameter', 'min_via_diameter'), ('annular_width', 'min_via_annular_width')):
        if kind in global_dru: raw_data[key] = max(raw_data[key], global_dru[kind])
    raw_data.update(dru_global=global_dru, dru_conditional=conditional_dru)
    lq = layers_off_pcb(pcb)
    L = int(cu_layers or lq or 4)
    drc = max(raw_data['min_clearance'], raw_data['netclass_clearance'] or 0)

    vd, vdr = raw_data['netclass_via'] if all(raw_data['netclass_via']) else (raw_data['via_dimensions'][0]['diameter'], raw_data['via_dimensions'][0]['drill']) \
        if raw_data['via_dimensions'] else (0.45, 0.2)
    vdr = max(vdr, raw_data['min_through_hole_diameter'])


    via_min = ru.get('min_via_diameter', 0.5)
    for r in rules:
        dmin = r['constraints'].get('via_diameter', {}).get('min')
        c = r['cond'].replace(' ', '')
        if dmin is None or r['layer']: continue
        m = re.fullmatch(r"A.Type=='Via'&&A.Hole_Size_X<([0-9.]+)mm", c)
        if not c or c == "A.Type=='Via'" or (m and vdr < float(m.group(1))):
            via_min = dmin
    vd = max(vd, ru.get('min_via_diameter', 0.5), via_min, vdr + 2 * raw_data['min_via_annular_width'],
             vdr + 2 * max(0.0, raw_data['min_hole_clearance'] - drc))
    sigw = max(SIGW_BY_LAYERS.get(L, 0.15 if L <= 4 else 0.10), raw_data['min_track_width'])
    def via_mass(v):

        return '%.2f' % v if v == round(v, 2) else repr(v)
    router = dict(cu_layers=L, drc=drc, drcpad=pad_track if pad_track is not None else drc, clr=drc + 0.005,
                  edge=border['edge_mm'], h2h=raw_data['min_hole_to_hole'], via=via_mass(vd) + '/' + via_mass(vdr),
                  sigw=sigw, sigmin=sigw, vip='1' if L >= 6 else '0',

                  track_min=raw_data['min_track_width'], hole_clr=raw_data['min_hole_clearance'], edge_exact='1',

                  angle45='1')


    placer = dict(pad_spacing=pad_pad_smd if pad_pad_smd is not None else 0.15,
                  pad_spacing_drc=pad_pad_smd if pad_pad_smd is not None else max(drc + 0.005, 0.15),
                  border=max(border['edge_mm'], 0.2))
    args = ' '.join('%s=%s' % (k, v) for k, v in router.items())
    result = dict(source=pcb, source_datum=time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(pcb))),
                layers_source=lq, raw_data=raw_data, router=router, placer=placer, args=args, edge_rules=border,
                hints=['Via-in-pad (POFV) is free of surcharge at JLC from 6 layers (vip=1).' if L >= 6 else
                          'Via-in-pad costs a surcharge at 4 layers (vip=0).',
                          'Signal width per user constraint %.2f mm (all signals equal).' % sigw])

    if border['conditional']:
        result['hints'].append('Conditional edge rules: conservative envelope limit %.6g mm for all nets/layers/edge pieces; no exact condition evaluation.' % border['edge_mm'])

    if structure:
        from analysis.structure import typical
        result['structure'] = typical(L)
    from analysis.spacings import contract_spec


    result['spacings'] = contract_spec(design_rules={'raw_data': dict(raw_data, min_clearance=ru.get('min_clearance', 0.2))},
                                 project=pro, dru=rules)
    return result


def choose(source='latest', fallback='0', template=False, cu_layers=None):
    if source in (None, '', 'latest'):
        R = read('latest', cu_layers); R.update(source_kind='latest', circuit_board=None, own_design_rules=None); return R
    pcb = source[:-len('.kicad_pro')] + '.kicad_pcb' if source.endswith('.kicad_pro') else source
    ok, reason = own_design_rules(pcb)
    if ok or fallback in ('0', '', None):
        R = read(pcb, cu_layers); R.update(source_kind='circuit_board' if ok else 'given', circuit_board=pcb, own_design_rules=reason); return R
    vl = latest_board() if fallback == 'latest' else fallback
    if not vl: raise SystemExit('design_rules.py: %s has no own rules (%s) and no template found' % (pcb, reason))
    if template:
        template_write(vl, pcb)
    R = read(vl, cu_layers or layers_off_pcb(pcb))
    R.update(source_kind='template', circuit_board=pcb, own_design_rules=reason, template_written=bool(template))
    R['hints'].insert(0, 'Board without own rules (%s): template %s%s.' % (
        reason, vl, ', written to the working copy' if template else ', DRC of the board checks with its own values'))
    return R


def hint_layer_stack(B, R, K):
    from analysis.areas import metric
    E = metric(B, R, K)
    status = ' K19 rule gate: ' + ('Option on.' if E['gate_active'] else 'Diagnosis, uncalibrated heuristic off.')
    if E['contradictions']:
        status += ' Contradiction declaration/final fill: ' + '; '.join('%s: %s → %s' %
                    (e['layer_name'], '|'.join(e['declared']) or '-', '|'.join(e['filled']) or '-') for e in E['contradictions']) + '.'
    if E['power_board']: return E, 'K19 exception (heuristic): ' + E['exception_reason'] + status
    return E, ('K19 (%s): GND layers %s; potential layers %s; missing GND %d; signal layers without direct GND neighbor %s; analog nets unshielded %s.' %
               (E['proof'], ','.join(E['gnd_layers']) or '-', ','.join(E['potential_layers']) or '-', E['gnd_missing'], ','.join(E['without_baseline']) or '-', ','.join(E['analog_unshielded']) or '-')) + status


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'structure':
        B, K = json.load(open(sys.argv[2])), json.load(open(sys.argv[3]))
        kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
        routes = json.load(open(kw['routes'])) if kw.get('routes') else {}
        if 'layer_stack_heuristic' in kw:
            K = dict(K, specs=dict(K.get('specs') or {}, layer_stack_heuristic=kw['layer_stack_heuristic'] == '1'))
        if 'gnd_layers' in kw: routes = dict(routes, options=dict(routes.get('options') or {}, gnd_layers=kw['gnd_layers']))
        E, text = hint_layer_stack(B, routes, K); print(text)
        if kw.get('out'): json.dump(E, open(kw['out'], 'w'), indent=1)
        raise SystemExit(0)
    pos = [a for a in sys.argv[1:] if '=' not in a]
    kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    R = choose(pos[0] if pos else 'latest', kw.get('fallback', '0'), kw.get('template') == '1', kw.get('cu_layers'))
    if 'out' in kw: json.dump(R, open(kw['out'], 'w'), indent=1)
    print('Source  ', R['source'], '(%s, %s layers; %s%s)' % (R['source_datum'], R['layers_source'], R['source_kind'],
          ': ' + R['own_design_rules'] if R.get('own_design_rules') else ''))
    print('Router  ', R['args'])
    print('Placer  ', R['placer'])
    for h in R['hints']: print('Notice ', h)
