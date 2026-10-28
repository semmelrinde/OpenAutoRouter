""
import json, math, heapq, sys, time, os, copy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.prozessschutz import bootstrap


_bootstrap_options = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
try: _bootstrap_portfolio = int(_bootstrap_options.get('versuche', '0'))
except ValueError: _bootstrap_portfolio = 0
bootstrap(heavy=_bootstrap_portfolio <= 0)
from pathlib import Path
import numpy as np
from scipy import ndimage
try:
    from edt_exact import distance_transform_edt as _edt
except ImportError:

    from scipy.ndimage import distance_transform_edt as _edt
from PIL import Image, ImageDraw
from shapely.geometry import Polygon, Point

def rand_polygon(board):
    ""
    import shapely
    holes = board.get('outline_holes') or []
    if len(holes) > len(board['outline']):
        raise ValueError('outline_holes ohne zugeordneten Aussenring')
    polygons = [Polygon(q, holes[i] if i < len(holes) else [])
                for i, q in enumerate(board['outline'])]
    if not polygons or any(p.is_empty or not p.is_valid for p in polygons):
        raise ValueError('Ungueltige Edge.Cuts-Konturen; nativen Umriss pruefen')
    result = shapely.union_all(polygons)
    return result


t_start = time.time()
args = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)


MEISTER_BELEG = None
if 'klassen' in args:
    from lernen.meister import anwenden as _meister_anwenden, pruefe_cli as _meister_pruefe_cli
    MEISTER_BELEG = _meister_anwenden(args, json.loads(Path(sys.argv[1]).read_text()), 'router')
    _meister_pruefe_cli(args)



if args.get('kritik', '0') not in ('0', '') or any(k in args for k in ('kritik_min', 'kritik_max', 'kritik_bis')):
    raise ValueError('kritik/kritik_min/kritik_max/kritik_bis wurden entfernt; siehe wissen/codex_optionen_aufraeumen.md')
_ARGS_EXPLIZIT = set(args)
STD_AN = 'klassen' in args and args.get('std', '1') == '1'
if STD_AN:
    for _k, _v in (('pinzugang', 'exakt'), ('pinzugang_rest', '1'), ('pinzugang_minimax', '1'),
                   ('kurzschluss_schutz', '1'), ('klassenabstand', '1')):
        args.setdefault(_k, _v)
    def _native_basis_da():
        try: _r = json.loads(Path(args['regeln']).read_text()) if args.get('regeln') else {}
        except (OSError, ValueError): _r = {}
        _b = Path(args.get('nativ_basis') or _r.get('platine') or '')
        return _b.suffix == '.kicad_pcb' and all(_b.with_suffix(x).is_file() for x in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'))
    if _native_basis_da():
        args.setdefault('skew', '1')
        if args.get('budget') == 'arbeit':
            args.setdefault('versuche', '3')
    elif 'skew' not in args or 'versuche' not in args:
        print('Standard: skew/versuche aus (keine native KiCad-Basis)', flush=True)


if args.get('budget', 'cpu') not in ('0', 'cpu', 'arbeit'):
    raise ValueError('budget muss 0, cpu oder arbeit sein')
BUDGET_ARBEIT = args.get('budget') == 'arbeit'
ARBEIT_MESSEN = BUDGET_ARBEIT or args.get('arbeitsmessung') == '1'


ARBEIT_MAX = int(args.get('arbeit_max', '59147'))
if ARBEIT_MAX <= 0: raise ValueError('arbeit_max muss eine positive ganze Zahl sein')
ARBEIT_SEED = int(args.get('seed', '0')) if BUDGET_ARBEIT else 0
if not 0 <= ARBEIT_SEED < 2**32: raise ValueError('seed muss zwischen 0 und 2**32-1 liegen')
if BUDGET_ARBEIT:

    for _name in ('loop', 'agent_loop', 'weiter', 'restkonflikte', 'rest_reparatur',
                  'lerndaten', 'fruehabbruch', 'lernbudget'):
        if args.get(_name, '0') not in ('0', ''):
            raise ValueError('budget=arbeit unterstuetzt %s noch nicht' % _name)
_ARBEIT = dict(einheit=0, astar=0, pruefung=0)
_ARBEIT_MARKEN = []


def arbeit_schritt(art):
    arbeit_notbremse()
    if ARBEIT_MESSEN: _ARBEIT[art] += 1

def arbeit_notbremse():
    if BUDGET_ARBEIT and globals().get('CPU_MAX', 0) > 0 and time.process_time() >= CPU_MAX:


        raise SystemExit('budget=arbeit: CPU-Notbremse; kein reproduzierbares Messergebnis')


def arbeit_gesamt():
    return sum(_ARBEIT.values())


def budget_uhr():
    return arbeit_gesamt() if BUDGET_ARBEIT else time.process_time()

def arbeit_marke(phase):
    if ARBEIT_MESSEN:
        _ARBEIT_MARKEN.append(dict(phase=phase, arbeit=arbeit_gesamt(), zaehler=dict(_ARBEIT),
                                  cpu_s=time.process_time()))


def arbeit_beleg():
    return dict(version='routingauftraege-v1', modus='arbeit' if BUDGET_ARBEIT else 'cpu_messung',
                maximum=ARBEIT_MAX, verbraucht=arbeit_gesamt(), zaehler=dict(_ARBEIT),
                marken=list(_ARBEIT_MARKEN), notbremse=False)


def budget_rng(salt):


    return np.random.RandomState((int(salt) + 1000003 * ARBEIT_SEED) % 2**32)



def native_routen_pruefen(board, routes, optionen, folder, export=False):
    import shutil
    from gemeinsam import plaetze as PL
    from kicad.python_aufruf import python_befehl
    root = Path(__file__).resolve().parents[1]
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    rules = json.loads(Path(optionen['regeln']).read_text()) if optionen.get('regeln') else {}
    base = Path(optionen.get('nativ_basis') or rules.get('platine') or '')
    if not base.is_file() or base.suffix != '.kicad_pcb':
        raise ValueError('Native Abnahme braucht nativ_basis= oder regeln.platine (.kicad_pcb)')
    for suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        src = base.with_suffix(suffix)
        if not src.is_file(): raise ValueError('Native Eingabe fehlt: ' + str(src))
        shutil.copy2(src, folder / ('base' + suffix))
    rp = folder / 'routes.json'; rp.write_text(json.dumps(routes))
    zones = json.loads(Path(optionen['nativ_zonen']).read_text()) if optionen.get('nativ_zonen') else routes.get('zones', board.get('zones', []))
    (folder/'zones.json').write_text(json.dumps(zones))
    runs = []
    def call(tag, cmd):
        r = PL.lauf(list(map(str, cmd)), cpu_s=900, log=str(folder/(tag+'.log')),
                    cwd=root, env=dict(os.environ, PYTHONHASHSEED='0'))
        runs.append(dict(tag=tag, argv=list(map(str,cmd)), **r))
        (folder/'laeufe.json').write_text(json.dumps(runs, indent=2))
        if r['zustand'] != 'ok': raise RuntimeError('Native Abnahme '+tag+': '+str(r))
    pcb = folder/'board.kicad_pcb'
    call('import', python_befehl(root/'kicad/import_routes.py', folder/'base.kicad_pcb', rp,
         folder/'zones.json', pcb, 'projekt=1', 'lagen='+str(board.get('lagen',4)),
         'via='+optionen.get('via', (rules.get('router') or {}).get('via','0.45/0.20'))))
    cli = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
    call('drc', [cli,'pcb','drc','--format','json','--severity-all','-o',folder/'drc.json',pcb])
    d = json.loads((folder/'drc.json').read_text())
    if not all(k in d for k in ('violations','unconnected_items','kicad_version')):
        raise ValueError('Unvollstaendiger nativer DRC-Bericht')
    if export:
        call('export', python_befehl(root/'kicad/routes_export.py',pcb,folder/'native.routes.json','voll=1'))


        (folder/'eingang.board.json').write_text(json.dumps(board))
        call('effektiv', python_befehl(root/'codex/v12/native_effektive_pads.py',pcb,
                                      folder/'eingang.board.json',folder/'native.board.json'))
    return d


def native_skew_fehler(drc):
    ""
    import re
    out = []
    for v in drc['violations']:
        if v['type'] != 'skew_out_of_range' or v['severity'] != 'error': continue
        text = v['description']
        m = re.search(r"rule '(.+)' max skew ([0-9.]+) mm; actual (-?[0-9.]+) mm; target net length ([0-9.]+) mm \(from (.+)\); actual ([0-9.]+) mm", text)
        item = re.search(r'\[([^]]+)\]', v['items'][0]['description'])
        if not m or not item: raise ValueError('Unbekannte native Skew-Messung: '+text)
        name,tol,skew,target,partner,length = m.groups()
        if float(skew) >= 0: raise ValueError('Skew-Nachlauf kann nur kuerzere Adern verlaengern')
        out.append(dict(regel=name, net=item[1], partner=partner, toleranz_mm=float(tol),
                        delta_mm=-float(skew), vorher_mm=float(length), ziel_mm=float(target)))
    return sorted(out, key=lambda q:(q['regel'],q['net']))


def mehrfachversuche(board_path, out, optionen, count):
    ""
    from gemeinsam import plaetze as PL
    from analyse.bewertung import bewerte
    if not optionen.get('klassen'): raise ValueError('versuche=N braucht klassen= fuer Note v3')
    if optionen.get('budget','arbeit')!='arbeit': raise ValueError('versuche=N braucht budget=arbeit')
    seed=int(optionen.get('seed','0'))
    if seed<0 or seed+count>2**32: raise ValueError('versuche: Seedbereich ausserhalb uint32')
    folder=Path(out).with_suffix('.versuche');folder.mkdir(parents=True,exist_ok=False)
    board=json.loads(Path(board_path).read_text());classes=json.loads(Path(optionen['klassen']).read_text())
    entries=[]
    for offset in range(count):
        current_seed=seed+offset;dest=folder/('seed_%d'%current_seed);dest.mkdir()


        child={k:v for k,v in optionen.items() if k!='versuche' and (k in _ARGS_EXPLIZIT or not STD_AN)}
        child['versuche']='0'
        child.update(seed=str(current_seed),budget='arbeit')
        for name in ('pre','png','etiketten','verlauf'):
            if name in child: child[name]=str(dest/name)
        argv=[sys.executable,__file__,str(board_path),str(dest/'routes.json')]+[k+'='+str(v) for k,v in child.items()]
        run=PL.lauf(argv,cpu_s=float(optionen.get('versuche_cpu_notbremse','12000')),log=str(dest/'route.log'),
                    env=dict(os.environ,PYTHONHASHSEED='0'))
        (dest/'lauf.json').write_text(json.dumps(dict(argv=argv,**run),indent=2))
        if run['zustand']!='ok':raise RuntimeError('versuche: unvollstaendiger Seed '+str(current_seed)+': '+str(run))
        route=json.loads((dest/'routes.json').read_text())


        with PL.platz():
            d=native_routen_pruefen(board,route,child,dest/'nativ',export=True)
            nr=json.loads((dest/'nativ/native.routes.json').read_text())
            nr.update({k:route[k] for k in ('optionen','gndlagen','flaechennetze') if k in route})
            evaluation=bewerte(json.loads((dest/'nativ/native.board.json').read_text()),nr,classes,v3=True)
            (dest/'bewertung.json').write_text(json.dumps(evaluation))
        errors=sum(v['severity']=='error' for v in d['violations']);opens=len(d['unconnected_items'])
        valid=errors==0 and opens==0 and evaluation['gueltig']
        note=evaluation.get('note_v3')
        if note is None or not math.isfinite(note):raise ValueError('versuche: Note v3 fehlt')
        entry=dict(seed=current_seed,gueltig=valid,fehler=errors,offene_paare=opens,note_v3=note,
                   modell_gueltig=evaluation['gueltig'],modell_unverbunden=evaluation['unverbunden'],
                   rang=[not valid,opens,note,offset],route=str(dest/'routes.json'),lauf=run)
        entries.append(entry)
        (folder/'zwischenstand.json').write_text(json.dumps(entries,indent=2))
    winner=min(entries,key=lambda q:q['rang'])
    result=json.loads(Path(winner['route']).read_text())
    result['mehrfachversuche']=dict(version='seed-portfolio-v1',anzahl=count,start_seed=seed,
         gewaehlt=winner['seed'],versuche=entries,vertrag='gueltig nativ+bewertung, offene Paare, Note v3, Seedfolge',
         cpu_router_s=sum(q['lauf']['cpu_s'] for q in entries))
    Path(out).write_text(json.dumps(result))
    print('Mehrfachversuche: %d, gewaehlt Seed %d, gueltig %s, offen %d, Note %.3f'%
          (count,winner['seed'],winner['gueltig'],winner['offene_paare'],winner['note_v3']),flush=True)


if args.get('skew', '0') not in ('0', '1'):
    raise ValueError('skew muss 0 oder 1 sein')
if 'versuche' in args:
    _versuche = int(args['versuche'])
    if _versuche < 0: raise ValueError('versuche muss eine nichtnegative ganze Zahl sein')
    if _versuche:
        mehrfachversuche(sys.argv[1], sys.argv[2], args, _versuche)
        sys.exit(0)


_AGENT_SAVED = None
if args.get('agent_loop', '0') != '0' or args.get('weiter'):
    from codex.agent_loop.zustand import vorbereiten as _agent_vorbereiten, quellen as _agent_quellen, sha as _agent_sha
    _AGENT_ARGV = dict(args)
    _AGENT_QUELLEN = _agent_quellen(__file__, sys.argv, args)
    _AGENT_BOARD_SHA = _agent_sha(sys.argv[1])
    _AGENT_SAVED = _agent_vorbereiten(__file__, sys.argv, args)



if args.get('restkonflikte') == '1': args.setdefault('kurzschluss_schutz', '1')
if args.get('loop') == '1' or args.get('agent_loop') == '1':
    args.setdefault('kurzschluss_schutz', '1')
    args.setdefault('konfliktort', 'exakt')
if args.get('rest_reparatur') == '1' and 'start' not in args:
    raise ValueError('rest_reparatur=1 braucht start= mit gespeichertem Kupfer')
D = json.load(open(sys.argv[1]))
_REST_START_SOURCE = None
if 'start' in args:
    _REST_START_SOURCE = json.load(open(args['start']))
    if args.get('rest_reparatur') == '1':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from nachbearbeitung.restkonflikte import reparatur_sperre
        _rest_block = reparatur_sperre(_REST_START_SOURCE)
        if _rest_block:
            _REST_START_SOURCE['restkonflikte'] = dict(uebernommen=0, versuche=[], cpu_s=0,
                grund=_rest_block)
            if args.get('kurzschluss_schutz', '0') == '1':


                import tempfile
                from nachbearbeitung.kurzschluss_schutz import datei_sichern, R44Fehler
                _r44_bp = sys.argv[1]
                with tempfile.TemporaryDirectory(prefix='autorouter_r45_sperre_') as _r44_td:
                    if _REST_START_SOURCE.get('rest_board'):
                        from nachbearbeitung.restkonflikte import effektives_board
                        _r44_bp = os.path.join(_r44_td, 'board.json')
                        Path(_r44_bp).write_text(json.dumps(effektives_board(D, _REST_START_SOURCE)))
                    try: datei_sichern(_r44_bp, args['start'], sys.argv[2], daten=_REST_START_SOURCE)
                    except R44Fehler as _r44_exc:
                        print(_r44_exc, file=sys.stderr); sys.exit(1)
            else:
                Path(sys.argv[2]).write_text(json.dumps(_REST_START_SOURCE))
            sys.exit(0)
    if _REST_START_SOURCE.get('rest_board') and not args.get('neu'):
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from nachbearbeitung.restkonflikte import effektives_board
        D = effektives_board(D, _REST_START_SOURCE)
_REST_BOARD_SOURCE = copy.deepcopy(D) if (_REST_START_SOURCE or {}).get('rest_board') else None
if _AGENT_SAVED is not None: D = _AGENT_SAVED['board']
ZONENPLAN = None
if args.get('zonenplan'):
    from pathlib import Path as _ZPPath
    sys.path.insert(0, str(_ZPPath(__file__).resolve().parents[1]))
    from lernen.zonenplaner import pruefe_eingaben as _zp_pruefe
    ZONENPLAN = json.load(open(args['zonenplan']))
    _zp_pruefe(ZONENPLAN, D,
               json.load(open(args['klassen'])) if args.get('klassen') else None,
               json.load(open(args['regeln'])) if args.get('regeln') else None, args)
OUT = sys.argv[2]


_FRUEH_AN = args.get('fruehabbruch', '0') in ('1', 'q1')
_LERN_AN = args.get('lerndaten', '0') == '1' or _FRUEH_AN
if _FRUEH_AN:
    args.setdefault('etiketten', os.path.splitext(OUT)[0] + '_etiketten.npz')
    args.setdefault('kurzschluss_schutz', '1')
_LERN_SNAPSHOT = None
_LERN_LABELS = None
_LERN_SIGNAL = False


def lern_atomic(path, payload):
    import tempfile
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)), suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(payload); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def lern_npz_atomic(path, data):
    import tempfile
    dest = os.path.abspath(path); os.makedirs(os.path.dirname(dest), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
    try:
        np.savez_compressed(tmp, **data); os.replace(tmp, dest)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def lern_sigxcpu(signum, frame):
    global _LERN_SIGNAL
    import signal, resource
    _LERN_SIGNAL = True


    soft, hard = resource.getrlimit(resource.RLIMIT_CPU)
    reserve = max(soft + 30, int(time.process_time()) + 30)
    reserve = reserve if hard == resource.RLIM_INFINITY else min(reserve, hard)
    resource.setrlimit(resource.RLIMIT_CPU, (reserve, reserve))
    signal.signal(signal.SIGXCPU, signal.SIG_IGN)
    if _LERN_SNAPSHOT is None:
        data = dict(tracks=[], vias=[], failed=sorted({p['net'] for p in D['pads'] if p.get('net')}),
                    conflicts={'#initialisierung': 1}, optionen=args,
                    unter_wmin=[], lernstand=dict(phase='initialisierung', paare=[], zensiert=True))


        plan = globals().get('ZONENPLAN')
        if plan:
            if args.get('kurzschluss_schutz', '0') == '1':
                data.update(zonenplan_id=plan['id'], zonen=[], zonenplan_ausgabe='ungeprueft_zurueckgehalten')
            else:
                data.update(zonenplan_id=plan['id'], zonen=plan['kern'], native_vias=True,
                    vias=[v for f in plan['via_kandidaten'] if f['angenommen'] for v in f['vias']])
    else:
        data = json.loads(_LERN_SNAPSHOT)
    if args.get('kurzschluss_schutz', '0') == '1' and data.get('failed'):


        names = data.get('kurzschluss_einheiten', {})
        if 'ausgabe_luftlinien' in globals():
            data['luftlinien'] = ausgabe_luftlinien(D, {names.get(n, n) for n in data['failed']})
    data['abschluss'] = 'SIGXCPU'
    data['lernstand']['zensiert'] = True
    lern_atomic(OUT, json.dumps(data))
    if _LERN_LABELS is not None:
        lern_npz_atomic(args['etiketten'], _LERN_LABELS)
    print('Rechenzeit-Budget: SIGXCPU, letzter sicherer Stand gespeichert', flush=True)
    sys.exit(0)


if _LERN_AN:
    import signal
    signal.signal(signal.SIGXCPU, lern_sigxcpu)
SPERR_N = 0
_LOOP_SOURCE_PAD_COUNT = len(D['pads'])
_VIA_KEEP = []


def _lochfrei(g):
    ""
    from shapely.geometry import LineString as _Ls, Polygon as _Pg2
    from shapely.ops import split as _split
    out, st = [], [g]
    while st:
        q = st.pop()
        if not q.interiors: out.append(q); continue
        c = _Pg2(q.interiors[0]).representative_point(); b = q.bounds
        st.extend(sorted((h for h in _split(q, _Ls([(c.x, b[1] - 1), (c.x, b[3] + 1)])).geoms if h.geom_type == 'Polygon' and h.area > 1e-9),
                         key=lambda h: h.bounds))
    return out


REGELN_AUFBAU = json.load(open(args['regeln'])).get('aufbau') if 'regeln' in args else None
if 'regeln' in args:
    for _k, _v in json.load(open(args['regeln']))['router'].items(): args.setdefault(_k, str(_v))

if args.get('sperren', '1') == '1' and D.get('keepouts'):






    from shapely.geometry import Polygon as _Pg, Point as _Pt
    from shapely.ops import unary_union as _uu
    _brd = rand_polygon(D)
    _nl = int(D.get('lagen') or args.get('lagen', '2'))
    _alle = ['F'] + ['In%d' % i for i in range(1, _nl - 1)] + ['B']
    _clr0 = float(args.get('clr', '0.13'))
    for _ki, _k in enumerate(list(D['keepouts'])):
        _lo = _k.get('loecher') or []
        _g = _uu([_Pg(q, [h for h in (_lo[i] if i < len(_lo) else []) if len(h) >= 3]).buffer(0)
                  for i, q in enumerate(_k.get('polys', [])) if len(q) >= 3]).intersection(_brd)
        if not (_k.get('verbote') or {}).get('bahnen', False):
            if (_k.get('verbote') or {}).get('vias', False) and args.get('sperren_vias', '1' if 'klassen' in args else '0') == '1' and not _g.is_empty:
                _VIA_KEEP.append(_g)
            continue
        _lag = [L_ for L_ in (_k.get('lagen') or _alle) if L_ in _alle]
        for _L in _lag:
            _eig = [_Pg(q).buffer(_clr0) for p_ in D['pads'] if _k.get('ref') and p_['ref'] == _k['ref']
                    and _L in p_['layers'] for q in p_['polys'].get(_L, [])]
            _gl = _g.difference(_uu(_eig)) if _eig else _g
            _ps = [g_ for g_ in getattr(_gl, 'geoms', [_gl]) if g_.geom_type == 'Polygon' and g_.area > 1e-3]
            _ps = [h_ for g_ in _ps for h_ in (_lochfrei(g_) if g_.interiors else [g_]) if h_.area > 1e-6]
            if not _ps: continue
            _q = [[[round(x, 5), round(y, 5)] for x, y in g_.exterior.coords] for g_ in _ps]
            _bb = _uu(_ps).bounds; _c = _uu(_ps).representative_point()
            D['pads'].append(dict(ref='SPERR_%d_%s' % (_ki, _L), num='1', net='#SPERR_%d' % _ki, layers=[_L], polys={_L: _q},
                                  x=_c.x, y=_c.y, bbox=list(_bb), drill=0.0, fp_side='F', pinfunc='', pintype='', maske={}))
            SPERR_N += 1
    if SPERR_N: print('Sperrflaechen: %d als Hindernis' % SPERR_N, flush=True)
    if _VIA_KEEP: print('Sperrflaechen nur fuer Vias: %d' % len(_VIA_KEEP), flush=True)
if args.get('stil_je_netzart') == '1': args['stil'] = 'je_netzart'
if 'stil' in args:
    args['winkel45'] = '1' if args['stil'] in ('45', 'winkel45', 'je_netzart') else '0'
KL = json.load(open(args['klassen'])) if 'klassen' in args else None
if args.get('sw_kompakt', '0') == '1':
    if KL is None: raise ValueError('sw_kompakt=1 braucht klassen=')
    from analyse.schaltknoten import korrigiere as _sw_korrigiere
    _sw_korrigiere(D, KL)
PINZUGANG = args.get('pinzugang', '0') == 'exakt'
if args.get('pinzugang', '0') not in ('0', 'exakt'):
    raise ValueError('pinzugang muss 0 oder exakt sein')
if PINZUGANG and KL is None:
    raise ValueError('pinzugang=exakt braucht klassen= und Platinenregeln')
PZ_REST = args.get('pinzugang_rest', '0') == '1'
PZ_MINI = args.get('pinzugang_minimax', '0') == '1'
if (PZ_REST or PZ_MINI) and (not PINZUGANG or args.get('kurzschluss_schutz') != '1'):
    raise ValueError('pinzugang_rest/minimax brauchen pinzugang=exakt und kurzschluss_schutz=1')
PZ_BREITEN = {}
PZ_LOKAL = {}
PZ_RAND_NM = 1000
PZ_RADIUS_NM = 1600000


def _pz_active(net):
    return bool(globals().get('PINZUGANG', False) and net in PZ_LOKAL)


def _pz_near(net,x,y):
    if not _pz_active(net):return False
    x,y=_pz_nm(x),_pz_nm(y)
    for p in D['pads']:
        if id(p) not in PZ_LOKAL[net]:continue
        a,b,c,d=map(_pz_nm,p['bbox'])
        if a-PZ_RADIUS_NM<=x<=c+PZ_RADIUS_NM and b-PZ_RADIUS_NM<=y<=d+PZ_RADIUS_NM:return True
    return False


def _pz_nm(value):
    from decimal import Decimal, ROUND_HALF_EVEN
    return int((Decimal(str(value)) * 1000000).to_integral_value(rounding=ROUND_HALF_EVEN))


def _pz_mm(nm):

    value = int(nm) / 1000000
    if int(value * 1000000) != nm:
        value = math.nextafter(value, math.inf if nm > 0 else -math.inf)
    assert int(value * 1000000) == nm
    return value


def _pz_snap(point):
    return [_pz_mm(_pz_nm(v)) for v in point]



def _pz_breiten(D, KL, regeln):
    ""
    import re
    roh = regeln.get('roh') or {}
    minimum = max(float((D.get('design') or {}).get('min_breite') or 0),
                  float(roh.get('min_track_width') or 0))
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError('pinzugang: keine belegte gueltige Platinen-Mindestbreite')
    if minimum == 0 and not (D.get('netzklassen') or roh.get('netclass_track')):
        raise ValueError('pinzugang: keine belegte positive Platinen-Mindestbreite/Netzklasse')

    source = regeln.get('platine') or (D.get('src') if str(D.get('src','')).endswith('.kicad_pcb') else None)
    if not source and regeln.get('quelle'):

        raise ValueError('pinzugang: Regelquelle ist ein anderes Board, DRU nicht gelesen')
    rules = []
    if source:
        base = Path(source).with_suffix('')
        pro = base.with_suffix('.kicad_pro'); dru = base.with_suffix('.kicad_dru')
        if pro.exists():
            actual = json.loads(pro.read_text())['board']['design_settings']['rules']['min_track_width']
            minimum = max(minimum, float(actual))
        if dru.exists():
            from analyse.regeln import dru_rules
            rules = dru_rules(str(dru))
        elif roh.get('dru_bedingt') or regeln.get('dru_bedingt'):
            raise ValueError('pinzugang: bedingte DRU-Regeln ohne Quelldatei')
    elif roh.get('dru_bedingt') or regeln.get('dru_bedingt'):
        raise ValueError('pinzugang: bedingte DRU-Regeln ohne Quelle')
    from gemeinsam.dru_bedingung import passt as dru_passt
    netze = {p['net'] for p in D['pads']}
    out = {}
    for net, k in sorted(KL['netze'].items()):
        if k.get('art') not in ('signal', 'analog', 'schnell'): continue

        if k.get('z_ohm') or k.get('z_diff_ohm') or k.get('paar'):
            continue
        nc = (D.get('netclass') or {}).get(net, 'Default')
        nominal = float(((D.get('netzklassen') or {}).get(nc) or {}).get('w') or roh.get('netclass_track') or 0)
        w = minimum if minimum > 0 else nominal
        if not math.isfinite(w) or w <= 0:raise ValueError('pinzugang: keine positive Regelbreite fuer '+net)
        maximum = float('inf'); names = []; dru_min = minimum
        for r in rules:
            c = r['constraints'].get('track_width')
            if not c: continue

            v = None if r.get('layer') else dru_passt(r['cond'], net, nc, None, netze)
            if v is False: continue
            if c.get('min') is None:
                raise ValueError('pinzugang: Breitenregel ohne auswertbares Minimum: '+r['name'])
            if v is None:
                dru_min = max(dru_min, float(c['min'])); w = max(w, dru_min); names.append(r['name']); continue

            dru_min = max(minimum, float(c['min']))
            w = dru_min
            if w <= 0:w=nominal
            maximum = float(c['max']) if c.get('max') is not None else float('inf'); names = [r['name']]
            if c.get('opt') is not None: w = max(w, float(c['opt']))
        if not math.isfinite(w) or not 0 < w <= maximum:
            raise ValueError('pinzugang: widerspruechliche Breitenregeln fuer '+net)
        out[net] = dict(w=w, min_platine=minimum, netclass=nc, regeln=names,
                        quelle=source or 'board.design.min_breite')
        if names:out[net]['dru_min']=dru_min
    return out


if PINZUGANG:
    try:
        PZ_BREITEN = _pz_breiten(D, KL, json.load(open(args['regeln'])) if 'regeln' in args else {})
    except ValueError as _e:
        if 'pinzugang' in _ARGS_EXPLIZIT or not str(_e).startswith('pinzugang'): raise

        print('Pinzugang exakt (Standard) aus: %s' % _e, flush=True)
        PINZUGANG = PZ_REST = PZ_MINI = False; PZ_BREITEN = {}
        args['pinzugang'] = '0'; args['pinzugang_rest'] = '0'; args['pinzugang_minimax'] = '0'
if PINZUGANG:
    print('Pinzugang exakt: %d einheitliche Signalbreiten aus Platinenregeln' % len(PZ_BREITEN), flush=True)
DRCL = KL is not None and args.get('drc_modell', '0') == '1'
if DRCL: D['pads'].extend(D.get('footprint_copper') or [])
if KL and args.get('mask_grafik', '0') == '1':
    D['pads'].extend(D.get('mask_graphics') or [])
GNDS = set(KL['gnd']) if KL and KL.get('gnd') else {'/GND'}
GND_HAUPT = KL['gnd_haupt'] if KL and KL.get('gnd_haupt') else '/GND'


STROM_ZUERST = args.get('strom_zuerst', '0') == '1'
STROM_J = float(args.get('strom_J', '40')) if STROM_ZUERST else 40.0
STROM_UMWEG = float(args.get('strom_umweg', '1')) if STROM_ZUERST else 1.0
STROM_POT = {}
if STROM_ZUERST:
    if KL is None: raise ValueError('strom_zuerst braucht klassen= und einen belegten Flussbaum')
    if not math.isfinite(STROM_J) or STROM_J <= 0 or not math.isfinite(STROM_UMWEG) or STROM_UMWEG < 0:
        raise ValueError('strom_J muss positiv, strom_umweg endlich und nichtnegativ sein')
    _sp_names = {p['net'] for p in D['pads']}
    _sp_ground = []
    for _spec in args.get('gndlagen', 'In1').split(','):
        _layer, _, _names = _spec.partition('=')
        _nn = {n if n in _sp_names else '/' + n if '/' + n in _sp_names else n.lstrip('/') for n in _names.split('|') if n}
        if _nn and not _nn <= GNDS:
            if _nn & GNDS: raise ValueError('strom_zuerst: gemischte GND/Potenziallage nicht unterstuetzt')
            STROM_POT[_layer] = sorted(_nn)
        elif _spec: _sp_ground.append(_spec)

    args['gndlagen'] = ','.join(_sp_ground)

RES = 0.05
BAHN_MIN = float(args.get('bahn_min', '0'))
LOCH_CLR = float(args.get('loch_clr', '0'))
CLR = float(args.get('clr', '0.13'))
QM = 0.036
CLR_EFF = CLR + QM
EDGE = float(args.get('edge', '0.30')) + 0.036
VIA_D, VIA_DR = [float(v) for v in args.get('via', '0.45/0.20').split('/')]
DRCC = float(args.get('drc', '0.127'))
DRCP = float(args.get('drcpad', args.get('drc', '0.127')))

KLASSENABSTAND = args.get('klassenabstand', '0') == '1'
if args.get('klassenabstand', '0') not in ('0', '1'):
    raise ValueError('klassenabstand muss 0 oder 1 sein')
KA = None
KA_VERTRAG = None
KA_LEGACY_DECKT = False
try:
    if KLASSENABSTAND:
        if KL is None: raise ValueError('klassenabstand=1 braucht klassen=')
        if args.get('loop','0') not in ('0','') or args.get('agent_loop','0') not in ('0',''):
            raise ValueError('klassenabstand: Placement-Transaktionen noch nicht abgedeckt')
        from analyse.abstaende import vertrag as _ka_vertrag, Abstand as _Abstand
        _ka_rg = json.load(open(args['regeln'])) if args.get('regeln') else {}
        if not _ka_rg.get('abstaende') and KL.get('abstaende'):
            _ka_rg = dict(_ka_rg, abstaende=KL['abstaende'])
        _ka_c = _ka_vertrag(D, _ka_rg)
        if not (D.get('netzklassen') or _ka_c['klassen'].get('Default') is not None):
            raise ValueError('klassenabstand: kein belegter Netzklassen-/Boardvertrag')

        if not _ka_rg.get('abstaende') and not KL.get('abstaende') and ((_ka_rg.get('roh') or {}).get('dru_bedingt') or _ka_rg.get('dru_bedingt')):
            raise ValueError('klassenabstand: DRU-Vertrag fehlt; regeln/klassen neu erzeugen')
        KA = _Abstand(_ka_c)
        for _ka_net in sorted({p['net'] for p in D['pads'] if p.get('net')}):
            KA.paar(_ka_net,'')
        print('Netzpaarabstaende: %d Klassen, %d DRU-Regeln, max %.6g mm' %
              (len(_ka_c['klassen']), len(_ka_c['dru']), KA.maximum), flush=True)
        KA_VERTRAG = _ka_c
        _ka_L = int(args.get('lagen','4'))
        KA_LEGACY_DECKT = KA.legacy_deckt({p['net'] for p in D['pads'] if p.get('net')},
                             ['F']+['In%d'%i for i in range(1,_ka_L-1)]+['B'],DRCC,DRCP)
        if KA_LEGACY_DECKT:
            print('Netzpaarvertrag vollstaendig vom bisherigen Abstand gedeckt; identische Altkarten',flush=True)
            KA = None


except ValueError as _e:
    if 'klassenabstand' in _ARGS_EXPLIZIT: raise

    print('Klassenabstand (Standard) aus: %s' % _e, flush=True)
    KLASSENABSTAND = False; KA = None; KA_VERTRAG = None; KA_LEGACY_DECKT = False; args['klassenabstand'] = '0'
def _ka_clr(net, other, li=0, typ='Track', typ2='Track', pad='', pad2=''):
    if KA is None: return DRCP if 'Pad' in (typ,typ2) else DRCC


    if 'MaskKeepout' in (typ,typ2) or 'HoleKeepout' in (typ,typ2):return 0.
    if any(str(n).startswith(('#SPERR','#MASK','#NTBAR')) for n in (net,other)):
        return DRCP if 'Pad' in (typ,typ2) else DRCC

    net = '' if str(net).startswith('#') else net
    other = '' if str(other).startswith('#') else other
    return KA.paar(net, other, LAYERS[li], typ, typ2, pad, pad2)[0]


def _ka_margin(net, other, li=0, typ='Track', typ2='Track', pad='', pad2=''):
    if KA is None: return _pz_clearance(net,other,'Pad' in (typ,typ2))
    return _ka_clr(net,other,li,typ,typ2,pad,pad2) + (PZ_RAND_NM/1e6 if _pz_active(net) or _pz_active(other) else .004)
VIP = args.get('vip') == '1'
BGA_LOCH = KL is not None and int(args.get('lagen', '4')) >= 6 and args.get('bga_via_exakt', '0') == '1'
if KL is not None and int(args.get('lagen', '4')) >= 6 and args.get('bga_via_min', '0') == '1':


    _bd = D.get('design') or {}
    _dr = float(_bd.get('bohr_min') or VIA_DR)
    _dia = max(float(_bd.get('via_min') or VIA_D), _dr + 2 * float(_bd.get('ring_min') or 0.1),
               0.0 if BGA_LOCH else _dr + 2 * max(0.0, LOCH_CLR - min(DRCP, DRCC)))
    VIA_D, VIA_DR = round(_dia, 4), round(_dr, 4)
    args['via'] = '%g/%g' % (VIA_D, VIA_DR)
    print('BGA kleine Vias (Option, ganze Platine):', args['via'], flush=True)
VIA_R = VIA_D / 2


VIA_KEEP_R = max(VIA_R, VIA_DR / 2 + LOCH_CLR - min(DRCP, DRCC)) if BGA_LOCH else VIA_R
VIA_OCC_R = VIA_R if BGA_LOCH and args.get('bga_belegung', '') == 'real' else VIA_KEEP_R
H2H = float(args.get('h2h', '0.25'))
HOLE_C = H2H + VIA_DR + 0.02
NLAG = int(args.get('lagen', '4'))
BGA_ESCAPE = KL is not None and NLAG >= 4 and args.get('bga_escape', '0') == '1'
BGA_FANOUT = KL is not None and ((NLAG >= 6 and args.get('bga_fanout', '0') == '1') or BGA_ESCAPE)
BGA_PLAN = BGA_FANOUT and (args.get('bga_plan', '0') == '1' or BGA_ESCAPE)
if BGA_ESCAPE: args.setdefault('bga_budget', '1')
BGA_ENG = BGA_PLAN and args.get('bga_gassen', '0') == '1'
PAD_EXTRA = 0.000001 if BGA_ENG else 0.006

POTENZIAL_PLAN = None
LAGENAUFBAU = False
if KL is not None:
    from pathlib import Path as _FlPath
    sys.path.insert(0, str(_FlPath(__file__).resolve().parents[1]))
    from analyse.flaechen import aktiv as _fl_aktiv, plan as _fl_plan
    LAGENAUFBAU = _fl_aktiv(dict(optionen=args), KL)
    if LAGENAUFBAU and not (ZONENPLAN and ZONENPLAN.get('audit',{}).get('etappe') == 3):
        _fl_map, _fl_fallback = _fl_plan(dict(D, lagen=NLAG), dict(optionen=args), KL)
        if _fl_fallback:
            POTENZIAL_PLAN = dict(angefragt=args.get('gndlagen', 'In1'), bahnbaum=_fl_fallback)
            args['gndlagen'] = ','.join(L + '=' + '|'.join(nn) for L, nn in _fl_map.items())
            print('Potenzialplan: Bahnbaum statt getrennter Fläche:', _fl_fallback, flush=True)
GNDL = [q.split('=')[0] for q in args.get('gndlagen', 'In1').split(',') if q]


PLANE_SIGNALS = set()
if PINZUGANG and D.get('plane_domains'):
    PLANE_SIGNALS = set(json.load(open(D['plane_domains'])).get('signal_layers', []))
    if PLANE_SIGNALS - set(GNDL): raise ValueError('signal_layers braucht zugeordnete Flaechenlagen')
STIL_JE_NETZART = args.get('stil') == 'je_netzart'
from gemeinsam.netzstil import organisch as _netz_organisch
OKT = args.get('winkel45') == '1'
LAYERS = ['F'] + ['In%d' % k for k in range(1, NLAG - 1) if 'In%d' % k not in GNDL or 'In%d' % k in PLANE_SIGNALS] + ['B']




_alle_netze = {p['net'] for p in D['pads'] if p['net']}
def _netzname(n):
    return n if n in _alle_netze else ('/' + n if '/' + n in _alle_netze else (n.lstrip('/') if n.lstrip('/') in _alle_netze else n))
_FL_EXPLIZIT = '=' in args.get('gndlagen', 'In1')
if LAGENAUFBAU and _FL_EXPLIZIT:
    from pathlib import Path as _FlPath
    sys.path.insert(0, str(_FlPath(__file__).resolve().parents[1]))
    from analyse.flaechen import zuordnung as _fl_zuordnung
    FL_ZUORDNUNG = _fl_zuordnung(dict(D, lagen=NLAG), dict(optionen=args), dict(gnd=sorted(GNDS)))
else:

    FL_ZUORDNUNG = {q.split('=')[0]: sorted({_netzname(n) for n in q.split('=', 1)[1].split('|') if n}) if '=' in q else ([] if _FL_EXPLIZIT else sorted(GNDS))
                    for q in args.get('gndlagen', 'In1').split(',') if q and q.split('=')[0] in ['F'] + ['In%d' % i for i in range(1, NLAG - 1)] + ['B']}
FLNETZE = {n for nn in FL_ZUORDNUNG.values() for n in nn}
PLANE_MIXED_NETS = {n for L in PLANE_SIGNALS for n in FL_ZUORDNUNG.get(L,[])}
if not LAGENAUFBAU:
    _fl = [n for q in args.get('gndlagen', 'In1').split(',') if '=' in q for n in q.split('=', 1)[1].split('|') if n]
    FLNETZE = {_netzname(n) for n in _fl} if _fl else (set(GNDS) if GNDL else set())
LI = {L: i for i, L in enumerate(LAYERS)}
NL = len(LAYERS)
STROM_T = {}
if STROM_ZUERST:
    _st_all = ['F'] + ['In%d' % i for i in range(1, NLAG - 1)] + ['B']
    if set(STROM_POT) - set(LAYERS): raise ValueError('strom_zuerst: ungueltige Potenziallage')
    _st_cu = (REGELN_AUFBAU or D.get('aufbau') or {}).get('kupfer_mm')
    if _st_cu is not None and (len(_st_cu) != NLAG or not all(math.isfinite(float(t)) and float(t) > 0 for t in _st_cu)):
        raise ValueError('strom_zuerst: ungueltige Kupferdicken')
    STROM_T = dict(zip(_st_all, map(float, _st_cu))) if _st_cu is not None else {L: .035 if L in ('F', 'B') else .0175 for L in _st_all}


xs = [p[0] for o in D['outline'] for p in o]; ys = [p[1] for o in D['outline'] for p in o]
X0, Y0 = min(xs) - 0.3, min(ys) - 0.3
NX = int(math.ceil((max(xs) + 0.3 - X0) / RES)); NY = int(math.ceil((max(ys) + 0.3 - Y0) / RES))
NN = NX * NY
print('Raster', NX, 'x', NY, '=', NN, 'Zellen je Lage')



VIA_DICHTE = args.get('via_dichte', '0') == '1'
VD_NACH = args.get('via_dichte_nach', '0') == '1' and KL is not None
VD_ZAEHLER = VIA_DICHTE or VD_NACH
VD_FENSTER = float(args.get('via_dichte_fenster', '2.0'))
VD_SCHWELLE = float(args.get('via_dichte_schwelle', '1.25'))
VD_MM = float(args.get('via_dichte_mm', '0.5'))
VD_STROM = float(args.get('via_dichte_strom', '0.5'))
if VD_ZAEHLER and (not all(math.isfinite(v) for v in (VD_FENSTER, VD_SCHWELLE, VD_MM, VD_STROM)) or
                   min(VD_FENSTER, VD_SCHWELLE) <= 0 or min(VD_MM, VD_STROM) < 0):
    raise ValueError('Ungueltige via_dichte-Optionen')
_VD_ZAHL = np.zeros((NY, NX), dtype=np.int32) if VD_ZAEHLER else None
_VD_VIAS = {}
_VD_VERSION = [0]
_VD_CACHE = [None]


def to_px(x, y): return ((x - X0) / RES - 0.5, (y - Y0) / RES - 0.5)
def cell_xy(ix, iy): return (X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES)


def raster_polys(polys, buf=0.0):
    img = Image.new('1', (NX, NY), 0); dr = ImageDraw.Draw(img)
    for pts in polys:
        P = Polygon(pts)
        if buf: P = P.buffer(buf, join_style=2)
        geoms = getattr(P, 'geoms', [P])
        for g in geoms:
            if g.is_empty: continue
            dr.polygon([to_px(x, y) for x, y in g.exterior.coords], fill=1, outline=1)
    return np.array(img, dtype=bool)


inside = raster_polys(D['outline'])
if D.get('outline_holes'):
    for _hs in D['outline_holes']:
        for _h in _hs: inside &= ~raster_polys([_h])
edge_d = _edt(inside) * RES
if args.get('edge_exakt') == '1' or any(D.get('outline_holes') or []):


    import shapely as _edge_shapely
    _edge_board = rand_polygon(D)
    _edge_x = X0 + (np.arange(NX) + .5) * RES
    for _y0 in range(0, NY, 64):
        _yy = Y0 + (np.arange(_y0, min(NY, _y0 + 64)) + .5) * RES
        _gx, _gy = np.meshgrid(_edge_x, _yy)
        _points = _edge_shapely.points(_gx.ravel(), _gy.ravel())
        _within = _edge_shapely.covers(_edge_board, _points)
        _dist = _edge_shapely.distance(_edge_board.boundary, _points)
        edge_d[_y0:_y0 + len(_yy)] = np.where(_within, _dist, 0.).reshape(_gx.shape)
        inside[_y0:_y0 + len(_yy)] = _within.reshape(_gx.shape)
nets = [''] + sorted({p['net'] for p in D['pads']})
NID = {n: i for i, n in enumerate(nets)}
occ = np.zeros((NL, NY, NX), dtype=np.int16)
padany = np.zeros((NL, NY, NX), dtype=bool)
drills = np.zeros((NY, NX), dtype=bool)
pad_cells = []
for k, p in enumerate(D['pads']):
    p.setdefault('cells', {}); p.setdefault('bcells', {})
    for L in p['layers']:
        if L not in LI: continue
        li = LI[L]
        m_buf = raster_polys(p['polys'][L], QM)
        m_core = raster_polys(p['polys'][L], -0.05)
        if not m_core.any():
            cx, cy = to_px(p['x'], p['y'])
            if 0 <= int(round(cy)) < NY and 0 <= int(round(cx)) < NX: m_core[int(round(cy)), int(round(cx))] = True
        nid = NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else -1
        occ[li][m_buf] = np.where(occ[li][m_buf] == 0, nid if nid > 0 else 32000, occ[li][m_buf])
        padany[li] |= m_buf
        p.setdefault('cells', {})[L] = np.flatnonzero(m_core)
        p.setdefault('bcells', {})[L] = np.flatnonzero(m_buf)
def pad_huelle(p, L, abstand, modell):
    ""
    poly = shapely.union_all([Polygon(q) for q in p['polys'][L]])
    mk = (p.get('maske') or {}).get(L, 0.0)
    if modell and mk > abstand:
        poly = poly.buffer(mk - abstand + 0.01, join_style=2)
    if modell and p.get('drill', 0) > 0:
        hole = shapely.union_all([Polygon(q) for q in p['hole_polys']]) if p.get('hole_polys') else Point(p['x'], p['y']).buffer(p['drill'] / 2, 32)
        hole = hole.buffer(max(0.0, LOCH_CLR - abstand), 32)
        poly = shapely.union_all([poly, hole])
    elif LOCH_CLR > abstand and p.get('drill', 0) > 0:
        hole = Point(p['x'], p['y']).buffer(p['drill'] / 2 + LOCH_CLR - abstand, 32)
        poly = shapely.union_all([poly, hole])
    if not modell and mk > abstand:
        poly = poly.buffer(mk - abstand + 0.01, join_style=2)
    return poly



import shapely


PAD_LOCH_GEOMS = []
PAD_LOCH_KREISE = []
PAD_LOCH_ABST = None






PAD_LOCH_RESERVE = 0.004
if KL is not None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from gemeinsam.bohrungen import pad_bohrung
    for p in D['pads']:
        try:
            hole = pad_bohrung(p)
        except (ValueError, TypeError, IndexError) as exc:
            raise ValueError('Pad %s:%s: %s' % (p['ref'],p['num'],exc)) from exc
        if hole is None: continue
        PAD_LOCH_GEOMS.append(hole['geo'])
        PAD_LOCH_KREISE.append((hole['x'],hole['y'],hole['drill']/2) if hole['kreis'] else None)
    if PAD_LOCH_GEOMS:
        PAD_LOCH_ABST = np.full((NY,NX),9.0,dtype=np.float32)
        margin = VIA_DR/2+H2H+.05
        for hole, circle in zip(PAD_LOCH_GEOMS,PAD_LOCH_KREISE):
            x0,y0,x1,y1 = hole.bounds
            ix0=max(0,int((x0-margin-X0)/RES));ix1=min(NX,int((x1+margin-X0)/RES)+1)
            iy0=max(0,int((y0-margin-Y0)/RES));iy1=min(NY,int((y1+margin-Y0)/RES)+1)
            if ix1<=ix0 or iy1<=iy0: continue
            gx,gy=np.meshgrid(X0+(np.arange(ix0,ix1)+.5)*RES,Y0+(np.arange(iy0,iy1)+.5)*RES)
            distances=(np.maximum(0.,np.hypot(gx-circle[0],gy-circle[1])-circle[2])
                       if circle is not None else shapely.distance(hole,shapely.points(gx,gy))).astype(np.float32)
            PAD_LOCH_ABST[iy0:iy1,ix0:ix1]=np.minimum(PAD_LOCH_ABST[iy0:iy1,ix0:ix1],distances)
PD1 = np.full((NL, NY, NX), 9.0, dtype=np.float32); PN1 = np.zeros((NL, NY, NX), dtype=np.int32); PD2 = np.full((NL, NY, NX), 9.0, dtype=np.float32)
GXc = X0 + (np.arange(NX) + 0.5) * RES; GYc = Y0 + (np.arange(NY) + 0.5) * RES
RW = 1.6
for k, p in enumerate(D['pads']):
    pn = NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else 30000 + k
    for L in p['layers']:
        if L not in LI: continue
        li = LI[L]
        poly = pad_huelle(p, L, CLR, DRCL)
        mk = (p.get('maske') or {}).get(L, 0.0)
        bx = p['bbox']
        if mk > CLR or LOCH_CLR > CLR:
            g = poly.bounds; bx = [min(bx[0], g[0]), min(bx[1], g[1]), max(bx[2], g[2]), max(bx[3], g[3])]
        ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
        iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
        GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
        d = shapely.distance(poly, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape).astype(np.float32)
        d1 = PD1[li, iya:iyb, ixa:ixb]; n1 = PN1[li, iya:iyb, ixa:ixb]; d2 = PD2[li, iya:iyb, ixa:ixb]
        same = n1 == pn
        d1[same] = np.minimum(d1[same], d[same])
        newbest = (~same) & (d < d1)
        d2[newbest] = d1[newbest]; d1[newbest] = d[newbest]; n1[newbest] = pn
        other = (~same) & (~newbest) & (d < d2)
        d2[other] = d[other]

VSP = np.full((NY, NX), 9.0, dtype=np.float32)
for vs in D.get('via_sperre', []):
    poly = shapely.union_all([Polygon(pts) for pts in vs['polys']]); bx = poly.bounds
    ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
    iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    d = shapely.distance(poly, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape).astype(np.float32)
    VSP[iya:iyb, ixa:ixb] = np.minimum(VSP[iya:iyb, ixa:ixb], d)
    for p in D['pads']:
        if p['ref'] == vs['ref']:
            for L, cells in p['cells'].items(): VSP.flat[cells] = 9.0

VIA_VERBOT = None
if _VIA_KEEP:
    VIA_VERBOT = np.zeros((NY, NX), dtype=bool)
    for _g in _VIA_KEEP:
        bx = _g.bounds
        ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
        iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
        GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
        d = shapely.distance(_g, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape)
        VIA_VERBOT[iya:iyb, ixa:ixb] |= d < VIA_R + 0.01
occ[:] = 0

under_esp = np.zeros((NY, NX), dtype=bool)
for c in D['courtyards']:
    if ((KL and c['ref'] in KL.get('teuer_unter', []))) and c['side'] == 'F':
        under_esp |= raster_polys(c['polys'])
padd = [PD1[0]] + [np.full((NY, NX), 9.0, dtype=np.float32) for _ in range(NL - 2)] + [PD1[NL - 1]]


BREIT = {0: (0.6, 0.3), 1: (0.35, 0.2), 2: (1.0, 0.3), 3: (0.8, 0.3), 4: (0.5, 0.2), 5: (0.4, 0.15), 6: (0.35, 0.15)}
SENS = ('FB_', 'VSENSE', 'SENSE', 'NTC', 'VREF')




DRU_MIN = {}
if KL is not None and args.get('dru_breite', '1') == '1':
    _dq = None
    try: _rg = json.load(open(args['regeln'])) if 'regeln' in args else {}
    except (OSError, ValueError): _rg = {}
    if _rg.get('platine'): _dq = Path(_rg['platine']).with_suffix('.kicad_dru')
    elif str(D.get('src', '')).endswith('.kicad_pcb'): _dq = Path(D['src']).with_suffix('.kicad_dru')
    else: _dq = Path(sys.argv[1]).with_suffix('.kicad_dru')
    if _dq is not None and _dq.exists():
        from analyse.regeln import dru_rules as _dru_rules
        from gemeinsam.dru_bedingung import mindestbreite as _dru_mb
        _dr = [r for r in _dru_rules(str(_dq)) if (r['constraints'].get('track_width') or {}).get('min') is not None]
        _dn = {p['net'] for p in D['pads']}
        _dl = ['F.Cu'] + ['In%d.Cu' % k for k in range(1, NLAG - 1)] + ['B.Cu']
        for _n in sorted(KL['netze']):
            _m, _nm = _dru_mb(_dr, _n, (D.get('netclass') or {}).get(_n, 'Default'), _dl, _dn, BAHN_MIN)
            if _nm and max(_m.values()) > BAHN_MIN: DRU_MIN[_n] = max(_m.values())
        if DRU_MIN: print('DRU-Mindestbreiten (E95): %d Netze, %s' % (len(DRU_MIN), sorted(set(DRU_MIN.values()))), flush=True)

def cls(net):
    C = _cls(net)
    if KL:
        _art = (KL['netze'].get(net) or {}).get('art', 'signal')
        _via = float(args.get('kosten_via_' + _art, '1'))
        _innen = float(args.get('kosten_innen_' + _art, '1'))
        if _via != 1: C = dict(C, via=C['via'] * _via)
        if _innen != 1:
            C = dict(C, inner=C['inner'] * _innen)
            if C.get('lf'):
                C['lf'] = tuple(v * (_innen if 0 < li < NL-1 else 1.) for li, v in enumerate(C['lf']))
    if 'histw_sig' in args:
        C = dict(C, histw=float(args['histw_sig']) if C['prio'] >= 8 else float(args.get('histw_pw', '0.5')))
    if args.get('breit') == '1' and KL is None and C['prio'] in BREIT:
        C = dict(C, w=BREIT[C['prio']][0], wmin=BREIT[C['prio']][1])
    if C['prio'] == 2 and 'vin_wmin' in args:
        C = dict(C, wmin=float(args['vin_wmin']), w=max(C['w'], float(args.get('vin_w', '2.0'))), pen=0.3)
    if args.get('schmal') == '1':
        C = dict(C, wmin=min(C['wmin'], {0: 0.25, 1: 0.15, 2: 0.2, 3: 0.2, 4: 0.15, 5: 0.12, 6: 0.12}.get(C['prio'], C['wmin'])))
    if KL and args.get('strom_soll', '0') == '1':
        from gemeinsam.strombahn import sollbreite
        k = KL['netze'].get(net, {})
        if k.get('art') in ('strom', 'versorgung', 'sw'):
            C = dict(C, w=sollbreite(k))
    if _pz_active(net) and net in PZ_BREITEN:
        C = dict(C, w=PZ_BREITEN[net]['w'], wmin=PZ_BREITEN[net]['w'])
    if net in DRU_MIN:
        m = DRU_MIN[net]
        C = dict(C, w=max(C['w'], m), wmin=max(C['wmin'], m), dru_min=m)
    return C












SCHNELL_AB = int(args.get('schnell_ab', '3'))
REIHENFOLGE = int(args.get('reihenfolge', '1')) if KL else 1
SCHNELL_LAGE = float(args.get('schnell_lage', '0.5')) if KL else 0.0
SCHNELL_INNEN = float(args.get('schnell_innen', '0')) if KL else 0.0
SCHNELL_VIA = float(args.get('schnell_via', '1')) if KL else 1.0
GEWICHTET = int(args.get('gewichtet', '0')) if KL else 0
STOER_FLANKE = args.get('stoer_flanke', '0') == '1' and KL is not None
STOERLISTE = None
if STOER_FLANKE:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from analyse.stoerer import lesen as _stoerer_lesen
    STOERLISTE = _stoerer_lesen(KL)


def ist_schnell(net):
    ""
    k = KL['netze'].get(net) if KL else None
    if STOER_FLANKE and k:
        tr = (STOERLISTE['netze'].get(net) or {}).get('tr_ns')
        if k.get('art') not in ('signal', 'schnell'): return False
        if tr is not None and 0 < tr <= 5.0: return True
        lk = k.get('l_krit_mm')
        if lk is None or float(lk) <= 0:
            if tr is None or tr <= 0: return False
            lk = 1000 * tr / (6 * 6.0)
        ps = [(p['x'], p['y']) for p in D['pads'] if p['net'] == net]
        span = max((math.dist(a, b) for a in ps for b in ps), default=0.0)
        return span >= float(lk)
    return bool(k) and k.get('art') == 'schnell' and k.get('rang', 0) >= SCHNELL_AB


def _lage_bezug():
    ""
    stapel = ['F'] + ['In%d' % k for k in range(1, NLAG - 1)] + ['B']
    art = {L: (2 if (not _FL_EXPLIZIT or set(nn) & GNDS) else 1)
           for L, nn in FL_ZUORDNUNG.items()}
    out = []
    for L in LAYERS:
        i = stapel.index(L)
        out.append(max([art.get(stapel[j], 0) for j in (i - 1, i + 1) if 0 <= j < len(stapel)] or [0]))
    return out


LAGE_BEZUG = _lage_bezug()
_LF = {}







_DIEL_JLC = {2: [1.5], 4: [0.0994, 1.265, 0.0994], 6: [0.0994, 0.55, 0.109, 0.55, 0.0994]}


def _dielektrika(n):
    vg = ((KL or {}).get('vorgaben') or {}).get('aufbau') or {}
    if vg.get('dielektrika') and len(vg['dielektrika']) == n - 1: return [float(d) for d in vg['dielektrika']]
    if n in _DIEL_JLC: return list(_DIEL_JLC[n])
    k = n - 1; pp = [i for i in range(k) if i % 2 == 0]; kern = [i for i in range(k) if i % 2 == 1]
    rest = 1.6 - 2 * 0.035 - (n - 2) * 0.0175 - 0.0994 * len(pp)
    return [0.0994 if i % 2 == 0 else max(0.1, rest / max(1, len(kern))) for i in range(k)]


def _aufbau():
    stapel = ['F'] + ['In%d' % k for k in range(1, NLAG - 1)] + ['B']
    z = [0.0]
    for d in _dielektrika(NLAG): z.append(z[-1] + d)
    ebene = {}
    for L, nn in FL_ZUORDNUNG.items():
        ns = set(nn); gn = sorted(ns & GNDS)
        ebene[L] = (gn[0] if len(gn) == 1 and len(ns) == 1 else None) if _FL_EXPLIZIT else (GND_HAUPT if GND_HAUPT in FLNETZE or not FLNETZE else sorted(FLNETZE)[0])
    out = []
    for L in LAYERS:
        i = stapel.index(L)
        ob = [j for j in range(i) if stapel[j] in ebene]; un = [j for j in range(i + 1, len(stapel)) if stapel[j] in ebene]
        h1 = z[i] - z[ob[-1]] if ob else None; h2 = z[un[0]] - z[i] if un else None
        if h1 is None and h2 is None: ref = h = H = None
        else:
            ref = stapel[ob[-1]] if (h2 is None or (h1 is not None and h1 <= h2)) else stapel[un[0]]
            h = min(x for x in (h1, h2) if x is not None)
            H = h1 * h2 / (h1 + h2) if (h1 is not None and h2 is not None) else h
        out.append(dict(L=L, i=i, z=z[i], ref=ref, ref_netz=ebene.get(ref) if ref else None, h=h, H=H))
    return out, stapel


AUFBAU, STAPEL = _aufbau()


def lf_schnell(inner):
    ""
    if inner not in _LF:
        alt = [1.0] + [inner] * (NL - 2) + [1.0]
        lf = list(alt)
        Hs = [a['H'] for a in AUFBAU if a['H']]
        if SCHNELL_LAGE > 0 and Hs:
            Hb = min(Hs)
            for li, a in enumerate(AUFBAU):
                q = 1.0 if not a['H'] else 1.0 - (Hb / a['H']) ** 2
                if a['H'] and a['ref_netz'] not in GNDS: q = max(q, 0.5)
                lf[li] *= 1.0 + SCHNELL_LAGE * q
        best = max(LAGE_BEZUG) if LAGE_BEZUG else 0
        if SCHNELL_INNEN > 0 and best > 0 and any(LAGE_BEZUG[li] == best for li in range(1, NL - 1)):
            for li in (0, NL - 1): lf[li] *= 1.0 + SCHNELL_INNEN
        _LF[inner] = None if lf == alt else tuple(lf)
    return _LF[inner]







NACHBARLAGE = args.get('nachbarlage', '1') == '1' and KL is not None


def nachbar_abstand(li, dist):
    ""
    if not NACHBARLAGE: return []
    a = AUFBAU[li]; out = []
    if not a['H']: return out
    for lj, b in enumerate(AUFBAU):
        if lj == li or abs(b['i'] - a['i']) != 1 or not b['H']: continue
        dz = abs(b['z'] - a['z']); s2 = dist * dist * b['H'] / a['H'] - dz * dz
        if s2 > 0: out.append((lj, min(math.sqrt(s2), 2.0 * dist)))
    return out








IMPEDANZ = args.get('impedanz', '0') == '1' and KL is not None
SI_FELDER = KL is not None and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('laengengruppe') for k in KL['netze'].values())
                              or bool((KL.get('vorgaben') or {}).get('laengengruppen')))
PAAR = args.get('paar', '0') == '1' and KL is not None
PAAR_SUCHRAUM = args.get('paar_suchraum', '0') == '1'
if PAAR_SUCHRAUM and not PAAR: raise ValueError('paar_suchraum=1 braucht paar=1 und klassen=')
PAAR_PRESET = args.get('paar_preset', 'bisher')
if PAAR_PRESET not in ('bisher', 'locker'): raise ValueError('paar_preset=bisher|locker')
PAAR_LAGE = float(args.get('paar_lage', '0.0' if PAAR_PRESET == 'locker' else '3.0')); PAAR_VIA = float(args.get('paar_via', '1.0' if PAAR_PRESET == 'locker' else '3.0')); PAAR_SPALT = float(args.get('paar_spalt', '0.2'))
PAAR_ZUG = float(args.get('paar_zug', '1.0')); PAAR_STOER = float(args.get('paar_stoer', '1.0' if PAAR_PRESET == 'locker' else '3.0')); PAAR_FREI = float(args.get('paar_frei', '0.3'))
PAAR_KANAL = float(args.get('paar_kanal', '1.0'))
PAAR_NETZE = {}
if PAAR:
    _ne = {p['net'] for p in D['pads'] if p['net']}
    for _n, _k in sorted(KL['netze'].items()):
        if _k.get('paar') and _k['paar'] in KL['netze'] and _n in _ne and _k['paar'] in _ne: PAAR_NETZE[_n] = _k['paar']


def paar_schnell(net):
    return net in PAAR_NETZE and KL['netze'][net].get('paar_art', KL['netze'][net].get('art')) == 'schnell'


def paar_lf(inner):
    ""
    gut = [li for li, a in enumerate(AUFBAU) if li in (0, NL - 1) and a['ref_netz'] in GNDS]
    if not gut: return None
    return tuple((1.0 if li in gut else 1.0 + PAAR_LAGE) * (1.0 if li in (0, NL - 1) else inner) for li in range(NL))
def paar_frei_mm(net):
    ""
    hs = [a['h'] for li, a in enumerate(AUFBAU) if li in (0, NL - 1) and a['ref_netz'] in GNDS and a['h']]
    return max(3 * min(hs), CLR + 0.05) if hs else PAAR_FREI


IMP_TAB = {}
if IMPEDANZ:
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analyse.impedanz_tabelle import tabelle as _z_tabelle
    from analyse.aufbau import bezugslagen as _z_refs
    _za = ((KL.get('vorgaben') or {}).get('aufbau') or KL.get('aufbau') or REGELN_AUFBAU)
    _zr = _z_refs(dict(optionen={'gndlagen': args.get('gndlagen', 'In1')}), GNDS, _alle_netze)
    for _zn, _zk in sorted(KL['netze'].items()):
        _zd = bool(_zk.get('z_diff_ohm') and _zk.get('paar'))
        _ziel = _zk.get('z_diff_ohm') if _zd else _zk.get('z_ohm')
        if _ziel:
            IMP_TAB[_zn] = _z_tabelle(_za, NLAG, _zr, float(_ziel), _zd, max(BAHN_MIN, float(args.get('sigmin', '0.09'))),
                                     max(DRCC + (.006 if PAAR_SUCHRAUM and _zd else 0.), float(args.get('impedanz_spalt', '0.13'))))
    print('Impedanz (Option): %d Netze, %d Lagenwerte' % (len(IMP_TAB), sum(map(len, IMP_TAB.values()))), flush=True)


def z_breite(net, li, fallback):
    return IMP_TAB.get(net, {}).get(LAYERS[li], {}).get('w', fallback)


def z_spalt(net, li):
    return IMP_TAB.get(net, {}).get(LAYERS[li], {}).get('spalt', PAAR_SPALT)

SI = args.get('si', '0') == '1' and KL is not None
SI_VIA = float(args.get('si_via', '1.5'))
SI_STICH = args.get('si_stich', '1') == '1' and SI


def _cls(net):
    if KL is not None:
        k = KL['netze'].get(net)
        if k:
            C = dict(w=k['w'], wmin=k['wmin'], pen=k['pen'], via=k['via'], inner=k['inner'], prio=k['prio'])
            if globals().get('KA') is not None: C['abstand_vertrag'] = KA.c['version']
            if (SCHNELL_LAGE > 0 or SCHNELL_INNEN > 0) and ist_schnell(net):
                _lf = lf_schnell(k['inner'])
                if _lf is not None: C['lf'] = _lf
            if SI and k.get('art') == 'schnell' and k.get('rang', 0) >= 5:
                C['via'] *= 1.0 + SI_VIA
            if net in PAAR_NETZE:
                _pl = paar_lf(k['inner'])
                if _pl is not None: C['lf'] = tuple(a * b for a, b in zip(C['lf'], _pl)) if C.get('lf') else _pl
                C['via'] = C['via'] * PAAR_VIA
            if net in IMP_TAB and IMP_TAB[net]:
                C['zw'] = tuple(z_breite(net, li, C['w']) for li in range(NL))
                C['w'] = max(C['zw']); C['wmin'] = min(C['zw'])

                C['lf'] = tuple((C.get('lf') or [1.0] * NL)[li] *
                                (min(2.0, 1 + 0.25 * max(0.0, min(IMP_TAB[net][LAYERS[li]]['profil']['h'],
                                 IMP_TAB[net][LAYERS[li]]['profil'].get('oben', float('inf'))) / .1 - 1))
                                 if LAYERS[li] in IMP_TAB[net] else 8.0) for li in range(NL))
            return C
        sw_ = float(args.get('sigw', KL.get('sigw', 0.15)))
        return dict(w=sw_, wmin=sw_, pen=0.0, via=0.9, inner=0.9, prio=8)
    n = net.split('/')[-1]
    return dict(w=float(args.get('sigw', '0.15')), wmin=float(args.get('sigmin', '0.12')), pen=0.0, via=0.9, inner=0.9, prio=8)


def loop_pad_patch(B):
    ""
    global D, PD1, PD2, PN1, padany, padd
    old = D['pads']
    if len(old) != len(B['pads']): raise ValueError('Padanzahl geaendert')
    clean = lambda p: {k:v for k,v in p.items() if k not in ('cells','bcells')}
    changed = [k for k,p in enumerate(B['pads']) if clean(old[k]) != clean(p)]
    refs = {old[k]['ref'] for k in changed}
    if not changed or len(refs) != 1 or len(changed) > 2: raise ValueError('kein lokaler SMD-Zweipoler')
    if any(p.get('drill') for k in changed for p in (old[k],B['pads'][k])): raise ValueError('bewegte Bohrung')
    if any(v['ref'] in refs for v in B.get('via_sperre',[])): raise ValueError('bewegte Net-Tie-Sperre')
    if any(v.get('ref') in refs for v in B.get('keepouts',[])): raise ValueError('bewegtes Teil beeinflusst Keepout-Geometrie; loop_karten=voll')
    if GASSEN or any(r in (KL.get('teuer_unter') or []) for r in refs): raise ValueError('gekoppelte Sonderkarte braucht loop_karten=voll')

    D = dict(B, pads=[p if k in changed else old[k] for k,p in enumerate(B['pads'])])
    boxes = []
    def window(p, L):
        poly = pad_huelle(p,L,CLR,DRCL); bx=p['bbox']
        if (p.get('maske') or {}).get(L,0.) > CLR or LOCH_CLR > CLR:
            g=poly.bounds;bx=[min(bx[0],g[0]),min(bx[1],g[1]),max(bx[2],g[2]),max(bx[3],g[3])]
        return (max(0,int((bx[0]-RW-X0)/RES)),min(NX,int((bx[2]+RW-X0)/RES)+1),
                max(0,int((bx[1]-RW-Y0)/RES)),min(NY,int((bx[3]+RW-Y0)/RES)+1),poly)
    for k in changed:
        for p in (old[k],D['pads'][k]):
            for L in p['layers']:
                if L in LI: boxes.append(window(p,L)[:4])
    xa=min(b[0] for b in boxes);xb=max(b[1] for b in boxes);ya=min(b[2] for b in boxes);yb=max(b[3] for b in boxes)
    PD1=PD1.copy();PD2=PD2.copy();PN1=PN1.copy();padany=padany.copy()
    PD1[:,ya:yb,xa:xb]=9.;PD2[:,ya:yb,xa:xb]=9.;PN1[:,ya:yb,xa:xb]=0;padany[:,ya:yb,xa:xb]=False
    for k in changed:
        p=D['pads'][k];p['cells']={};p['bcells']={}
        for L in p['layers']:
            if L not in LI: continue
            core=raster_polys(p['polys'][L],-.05)
            if not core.any():
                cx,cy=to_px(p['x'],p['y'])
                if 0<=int(round(cy))<NY and 0<=int(round(cx))<NX:core[int(round(cy)),int(round(cx))]=True
            p['cells'][L]=np.flatnonzero(core)
            p['bcells'][L]=np.flatnonzero(raster_polys(p['polys'][L],QM))
    for k,p in enumerate(D['pads']):
        pn=NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else 30000+k
        for L in p['layers']:
            if L not in LI: continue
            li=LI[L];cells=p['bcells'][L];cy,cx=cells//NX,cells%NX
            mask=(cx>=xa)&(cx<xb)&(cy>=ya)&(cy<yb);padany[li,cy[mask],cx[mask]]=True

            bx=p['bbox'];extra=max((p.get('maske') or {}).get(L,0.),LOCH_CLR)+.02


            if (p.get('maske') or {}).get(L,0.)<=CLR and LOCH_CLR<=CLR:
                if bx[2]+RW+extra<X0+xa*RES or bx[0]-RW-extra>X0+xb*RES or bx[3]+RW+extra<Y0+ya*RES or bx[1]-RW-extra>Y0+yb*RES: continue
            x0,x1,y0,y1,poly=window(p,L);x0=max(xa,x0);x1=min(xb,x1);y0=max(ya,y0);y1=min(yb,y1)
            if x1<=x0 or y1<=y0:continue
            gx,gy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);d=shapely.distance(poly,shapely.points(gx.ravel(),gy.ravel())).reshape(gx.shape).astype(np.float32)
            d1=PD1[li,y0:y1,x0:x1];n1=PN1[li,y0:y1,x0:x1];d2=PD2[li,y0:y1,x0:x1]
            same=n1==pn;d1[same]=np.minimum(d1[same],d[same]);newbest=(~same)&(d<d1)
            d2[newbest]=d1[newbest];d1[newbest]=d[newbest];n1[newbest]=pn
            other=(~same)&(~newbest)&(d<d2);d2[other]=d[other]
    padd=[PD1[0]]+padd[1:-1]+[PD1[NL-1]]
    return changed,(xa,xb,ya,yb)
