""
import json, os, re, sys, time, shutil, math

ROOT = os.path.expanduser('~/KiCad/10.0/projects')
SKIP = re.compile(r'(/\.history/|/backup|/archive|/archiv|/validation|/verifier|-backups/|/_|/arbeit/)', re.I)

KICAD_VORGABE = dict(min_clearance=0.0, min_track_width=0.0, min_copper_edge_clearance=0.5, min_hole_clearance=0.25,
                     min_hole_to_hole=0.25, min_through_hole_diameter=0.3, min_via_diameter=0.5, min_via_annular_width=0.1)
KICAD_KLASSE = dict(clearance=0.2, track_width=0.2, via_diameter=0.6, via_drill=0.3)



SIGW_BY_LAGEN = {2: 0.15, 4: 0.15, 6: 0.10, 8: 0.10}


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def neuestes_board():
    ""
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


def eigene_regeln(pcb):
    ""
    base = pcb[:-len('.kicad_pcb')] if pcb.endswith('.kicad_pcb') else pcb[:-len('.kicad_pro')]
    if not os.path.exists(base + '.kicad_pro'): return False, 'keine .kicad_pro'
    if dru_rules(base + '.kicad_dru'): return True, '.kicad_dru mit %d Regeln' % len(dru_rules(base + '.kicad_dru'))
    try:
        pro = json.load(open(base + '.kicad_pro'))
    except (OSError, ValueError):
        return False, '.kicad_pro nicht lesbar'
    ru = ((pro.get('board') or {}).get('design_settings') or {}).get('rules') or {}
    anders = sorted(k for k, v in KICAD_VORGABE.items() if k in ru and abs(float(ru[k]) - v) > 1e-9)
    dflt = {c.get('name'): c for c in (pro.get('net_settings') or {}).get('classes', [])}.get('Default', {})
    anders += sorted('Default.' + k for k, v in KICAD_KLASSE.items() if dflt.get(k) is not None and abs(float(dflt[k]) - v) > 1e-9)
    if anders: return True, 'abweichend von der KiCad-Vorgabe: ' + ', '.join(anders)
    return False, 'nur KiCad-Vorgaben (keine .kicad_dru)'


def vorlage_schreiben(vorlage_pcb, ziel_pcb):
    ""
    ziel = os.path.abspath(ziel_pcb)
    if ziel.startswith(os.path.expanduser('~/KiCad') + os.sep):
        raise SystemExit('regeln.py: schreibe nie in ein KiCad-Projekt unter ~/KiCad (%s); nur Arbeitskopien' % ziel)
    vb, zb = vorlage_pcb[:-len('.kicad_pcb')], ziel[:-len('.kicad_pcb')]
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
    ""


    toks = re.findall(r'"(?:[^"\\]|\\.)*"|#[^\n]*|\(|\)|[^\s()"#]+', text)
    stack = [[]]
    for t in toks:
        if t.startswith('#'): continue
        if t == '(': stack.append([])
        elif t == ')':
            if len(stack) == 1: raise ValueError('Unbalancierter S-Ausdruck')
            x = stack.pop(); stack[-1].append(x)
        else: stack[-1].append(t[1:-1] if t.startswith('"') else t)
    if len(stack) != 1: raise ValueError('Unbalancierter S-Ausdruck')
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
        d = dict(name=r[1], cond='', layer=None, constraints={}, constraints_ausdruck={})
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
                d['constraints_ausdruck'][x[1]] = raw
        out.append(d)
    return out


def dru_global(path):
    ""
    values = {}; conditional = []
    for rule in dru_rules(path):
        if rule['cond'] or rule['layer']:
            conditional.append(rule['name']); continue
        for kind, constraint in rule['constraints'].items():
            if constraint.get('min') is not None: values[kind] = constraint['min']
    return values, conditional


def rand_lage(layer):


    non_cu={'F.Courtyard','B.Courtyard','F.CrtYd','B.CrtYd','F.Fab','B.Fab',
            'F.SilkS','B.SilkS','F.Silkscreen','B.Silkscreen','F.Mask','B.Mask',
            'F.Paste','B.Paste','F.Adhes','B.Adhes','F.Adhesive','B.Adhesive',
            'Dwgs.User','Cmts.User','Eco1.User','Eco2.User','Margin'}
    return layer not in non_cu


def rand_strichbreite(pcb, rules):
    ""
    if not any(r.get('severity')!='ignore' and rand_lage(r.get('layer')) and 'physical_clearance' in r['constraints'] for r in rules): return None
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
                raise ValueError('Physische Randregel: nicht modellierte Edge.Cuts-Grafik '+str(node[0]))
            width=children.get('width',[None])[0]
            for item in children.get('stroke',[]):
                if isinstance(item,list) and item and item[0]=='width':width=item[1]
            width=mmval(str(width))
            if width is None or not math.isfinite(width) or width<0:
                raise ValueError('Physische Randregel: keine belegte Edge.Cuts-Strichbreite')
            largest=max(largest,width);found=True
        todo.extend(n for n in node[1:] if isinstance(n,list))
    if not found:raise ValueError('Physische Randregel ohne Edge.Cuts-Geometrie')
    return largest


def randregeln(board_min, rules, edge_width_mm=None):
    ""
    minimum = float(board_min)
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError('Ungueltiger Board-Kupfer-Randabstand')
    records = []
    for rule in rules:
        layer = rule.get('layer')
        if not rand_lage(layer): continue
        for kind in ('edge_clearance', 'physical_clearance'):
            c = rule['constraints'].get(kind)
            if c is None: continue
            value = c.get('min')
            if rule.get('severity')=='ignore':
                if value is not None and not math.isfinite(value):raise ValueError('Ungueltiges Randminimum: '+rule['name'])
                records.append(dict(name=rule['name'],condition=rule['cond'],layer=layer,
                                    constraint=kind,min_mm=value,edge_min_mm=None,severity='ignore'))
                continue
            if value is None:
                raise ValueError('Randregel ohne auswertbares Minimum: ' + rule['name'])
            if not math.isfinite(value):
                raise ValueError('Ungueltiges Randminimum: ' + rule['name'])
            bound=value
            if kind=='physical_clearance':
                if edge_width_mm is None or not math.isfinite(edge_width_mm) or edge_width_mm<0:
                    raise ValueError('Physische Randregel braucht belegte Edge.Cuts-Strichbreite')
                bound+=edge_width_mm/2
            records.append(dict(name=rule['name'], condition=rule['cond'], layer=layer,
                                constraint=kind, min_mm=value, edge_min_mm=bound, severity=rule.get('severity', 'error')))

            minimum = max(minimum, bound)
    return dict(version='randregeln-huelle-v2', modus='konservative_huellgrenze',
                board_min_mm=float(board_min), edge_mm=minimum, regeln=records,
                physical_edge_width_mm=edge_width_mm,
                bedingt=any(r['condition'] or r['layer'] for r in records))


def lagen_aus_pcb(pcb):
    try:
        head = open(pcb, encoding='utf-8', errors='ignore').read(200000)
    except OSError:
        return None
    m = re.search(r'\(layers(.*?)\n\t\)', head, re.S)
    if not m: return None
    return len(re.findall(r'"(F|B|In\d+)\.Cu"', m.group(1))) or None


def lese(quelle='neuestes', lagen=None, aufbau=False):
    pcb = neuestes_board() if quelle in (None, '', 'neuestes') else quelle
    if pcb.endswith('.kicad_pro'): pcb = pcb[:-len('.kicad_pro')] + '.kicad_pcb'
    base = pcb[:-len('.kicad_pcb')]
    pro = json.load(open(base + '.kicad_pro'))
    ds = pro['board']['design_settings']; ru = ds['rules']
    nc = {c['name']: c for c in pro.get('net_settings', {}).get('classes', [])}
    dflt = nc.get('Default', {})
    rules = dru_rules(base + '.kicad_dru')
    global_dru, conditional_dru = dru_global(base + '.kicad_dru')
    rand = randregeln(ru.get('min_copper_edge_clearance', 0.5), rules,rand_strichbreite(pcb,rules))
    roh = dict(min_clearance=ru.get('min_clearance', 0.2), min_track_width=ru.get('min_track_width', 0.2),
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
    roh.update(pad_track=pad_track, pad_pad_smd=pad_pad_smd, via_min_by_drill=via_min_by_drill)
    for kind, key in (('clearance', 'min_clearance'), ('track_width', 'min_track_width'),
                      ('edge_clearance', 'min_copper_edge_clearance'), ('hole_clearance', 'min_hole_clearance'),
                      ('hole_to_hole', 'min_hole_to_hole'), ('hole_size', 'min_through_hole_diameter'),
                      ('via_diameter', 'min_via_diameter'), ('annular_width', 'min_via_annular_width')):
        if kind in global_dru: roh[key] = max(roh[key], global_dru[kind])
    roh.update(dru_global=global_dru, dru_bedingt=conditional_dru)
    lq = lagen_aus_pcb(pcb)
    L = int(lagen or lq or 4)
    drc = max(roh['min_clearance'], roh['netclass_clearance'] or 0)

    vd, vdr = roh['netclass_via'] if all(roh['netclass_via']) else (roh['via_dimensions'][0]['diameter'], roh['via_dimensions'][0]['drill']) \
        if roh['via_dimensions'] else (0.45, 0.2)
    vdr = max(vdr, roh['min_through_hole_diameter'])


    via_min = ru.get('min_via_diameter', 0.5)
    for r in rules:
        dmin = r['constraints'].get('via_diameter', {}).get('min')
        c = r['cond'].replace(' ', '')
        if dmin is None or r['layer']: continue
        m = re.fullmatch(r"A.Type=='Via'&&A.Hole_Size_X<([0-9.]+)mm", c)
        if not c or c == "A.Type=='Via'" or (m and vdr < float(m.group(1))):
            via_min = dmin
    vd = max(vd, ru.get('min_via_diameter', 0.5), via_min, vdr + 2 * roh['min_via_annular_width'],
             vdr + 2 * max(0.0, roh['min_hole_clearance'] - drc))
    sigw = max(SIGW_BY_LAGEN.get(L, 0.15 if L <= 4 else 0.10), roh['min_track_width'])
    def via_mass(v):

        return '%.2f' % v if v == round(v, 2) else repr(v)
    router = dict(lagen=L, drc=drc, drcpad=pad_track if pad_track is not None else drc, clr=drc + 0.005,
                  edge=rand['edge_mm'], h2h=roh['min_hole_to_hole'], via=via_mass(vd) + '/' + via_mass(vdr),
                  sigw=sigw, sigmin=sigw, vip='1' if L >= 6 else '0',

                  bahn_min=roh['min_track_width'], loch_clr=roh['min_hole_clearance'], edge_exakt='1',

                  winkel45='1')


    placer = dict(padabst=pad_pad_smd if pad_pad_smd is not None else 0.15,
                  padabst_drc=pad_pad_smd if pad_pad_smd is not None else max(drc + 0.005, 0.15),
                  rand=max(rand['edge_mm'], 0.2))
    args = ' '.join('%s=%s' % (k, v) for k, v in router.items())
    result = dict(quelle=pcb, quelle_datum=time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(pcb))),
                lagen_quelle=lq, roh=roh, router=router, placer=placer, args=args, randregeln=rand,
                hinweise=['Via-in-Pad (POFV) ist bei JLC ab 6 Lagen ohne Aufpreis (vip=1).' if L >= 6 else
                          'Via-in-Pad kostet bei 4 Lagen Aufpreis (vip=0).',
                          'Signalbreite nach Nutzervorgabe %.2f mm (alle Signale gleich).' % sigw])

    if rand['bedingt']:
        result['hinweise'].append('Bedingte Randregeln: konservative Huellgrenze %.6g mm fuer alle Netze/Lagen/Randstuecke; keine exakte Bedingungsauswertung.' % rand['edge_mm'])

    if aufbau:
        from analyse.aufbau import typisch
        result['aufbau'] = typisch(L)
    from analyse.abstaende import vertrag


    result['abstaende'] = vertrag(regeln={'roh': dict(roh, min_clearance=ru.get('min_clearance', 0.2))},
                                 projekt=pro, dru=rules)
    return result


def waehle(quelle='neuestes', rueckfall='0', vorlage=False, lagen=None):
    ""
    if quelle in (None, '', 'neuestes'):
        R = lese('neuestes', lagen); R.update(quelle_art='neuestes', platine=None, eigene_regeln=None); return R
    pcb = quelle[:-len('.kicad_pro')] + '.kicad_pcb' if quelle.endswith('.kicad_pro') else quelle
    ok, grund = eigene_regeln(pcb)
    if ok or rueckfall in ('0', '', None):
        R = lese(pcb, lagen); R.update(quelle_art='platine' if ok else 'angegeben', platine=pcb, eigene_regeln=grund); return R
    vl = neuestes_board() if rueckfall == 'neuestes' else rueckfall
    if not vl: raise SystemExit('regeln.py: %s hat keine eigenen Regeln (%s) und keine Vorlage gefunden' % (pcb, grund))
    if vorlage:
        vorlage_schreiben(vl, pcb)
    R = lese(vl, lagen or lagen_aus_pcb(pcb))
    R.update(quelle_art='vorlage', platine=pcb, eigene_regeln=grund, vorlage_geschrieben=bool(vorlage))
    R['hinweise'].insert(0, 'Platine ohne eigene Regeln (%s): Vorlage %s%s.' % (
        grund, vl, ', in die Arbeitskopie geschrieben' if vorlage else ', DRC der Platine prueft mit ihren eigenen Werten'))
    return R


def hinweis_lagenaufbau(B, R, K):
    from analyse.flaechen import kennzahl
    E = kennzahl(B, R, K)
    status = ' K19-Regeltor: ' + ('Option an.' if E['tor_aktiv'] else 'Diagnose, ungeeichte Heuristik aus.')
    if E['widersprueche']:
        status += ' Widerspruch Deklaration/Endfüllung: ' + '; '.join('%s: %s → %s' %
                    (e['lage'], '|'.join(e['deklariert']) or '-', '|'.join(e['gefuellt']) or '-') for e in E['widersprueche']) + '.'
    if E['leistungsboard']: return E, 'K19 Ausnahme (Heuristik): ' + E['ausnahme_grund'] + status
    return E, ('K19 (%s): GND-Lagen %s; Potenziallagen %s; fehlende GND %d; Signallagen ohne direkten GND-Nachbarn %s; analoge Netze ungeschirmt %s.' %
               (E['nachweis'], ','.join(E['gndlagen']) or '-', ','.join(E['potenziallagen']) or '-', E['gnd_fehlen'], ','.join(E['ohne_bezug']) or '-', ','.join(E['analog_ungeschirmt']) or '-')) + status


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'aufbau':
        B, K = json.load(open(sys.argv[2])), json.load(open(sys.argv[3]))
        kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
        routes = json.load(open(kw['routes'])) if kw.get('routes') else {}
        if 'lagenaufbau_heuristik' in kw:
            K = dict(K, vorgaben=dict(K.get('vorgaben') or {}, lagenaufbau_heuristik=kw['lagenaufbau_heuristik'] == '1'))
        if 'gndlagen' in kw: routes = dict(routes, optionen=dict(routes.get('optionen') or {}, gndlagen=kw['gndlagen']))
        E, text = hinweis_lagenaufbau(B, routes, K); print(text)
        if kw.get('out'): json.dump(E, open(kw['out'], 'w'), indent=1)
        raise SystemExit(0)
    pos = [a for a in sys.argv[1:] if '=' not in a]
    kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    R = waehle(pos[0] if pos else 'neuestes', kw.get('rueckfall', '0'), kw.get('vorlage') == '1', kw.get('lagen'))
    if 'out' in kw: json.dump(R, open(kw['out'], 'w'), indent=1)
    print('Quelle  ', R['quelle'], '(%s, %s Lagen; %s%s)' % (R['quelle_datum'], R['lagen_quelle'], R['quelle_art'],
          ': ' + R['eigene_regeln'] if R.get('eigene_regeln') else ''))
    print('Router  ', R['args'])
    print('Placer  ', R['placer'])
    for h in R['hinweise']: print('Hinweis ', h)
