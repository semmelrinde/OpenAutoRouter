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
from shapely.geometry import LineString as _LS, Point as _PT
import shapely

_ZP_GEO = {}; _ZP_CACHE = {}; _ZP_VIA_MASK = {}; _ZP_VIAS = []; _ZP_DRILL = None
_ZP_POT_NETZE = set()
if ZONENPLAN:
    from lernen.stromzonen import _geo as _zp_geo
    from shapely.ops import unary_union as _zp_union
    if ZONENPLAN.get('audit', {}).get('etappe') == 3:
        for _label in ZONENPLAN['referenzschutz']['gndlagen']:
            _layer, *_names = _label.split('=', 1)
            if _names and _layer in ZONENPLAN.get('potenziallagen', []):
                _ZP_POT_NETZE.update(n for n in _names[0].split('|')
                                    if ZONENPLAN['netze'].get(n, {}).get('gewaehlt'))
    for _z in ZONENPLAN['kern']:
        _ZP_GEO.setdefault((_z['net'], _z['layer']), []).append(_zp_geo(_z))
    _ZP_VIAS = [v for f in ZONENPLAN['via_kandidaten'] if f['angenommen'] for v in f['vias']]
    for _v in _ZP_VIAS:
        for _L in _v['layers']:
            _ZP_GEO.setdefault((_v['net'], _L), []).append(_PT(_v['x'], _v['y']).buffer(_v['d']/2))
    _ZP_GEO = {key:_zp_union(gs) for key,gs in _ZP_GEO.items()}
    if _ZP_VIAS:
        _zx, _zy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        _ZP_DRILL = np.full((NY,NX), 9.0, dtype=np.float32)
        for _v in _ZP_VIAS:
            _ZP_DRILL = np.minimum(_ZP_DRILL, np.hypot(_zx-_v['x'],_zy-_v['y'])-_v['drill']/2)


def _zp_karten(net):
    if net not in _ZP_CACHE:
        if len(_ZP_CACHE)>=4: del _ZP_CACHE[next(iter(_ZP_CACHE))]
        xx,yy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        hard=[]; soft=[]
        for L in LAYERS:
            g = _zp_union([g for (n,l),g in _ZP_GEO.items() if n!=net and l==L])
            mask = shapely.intersects_xy(g.buffer(QM),xx,yy)
            hard.append(((_edt(~mask)*RES-2*QM) if mask.any() else np.full((NY,NX),9.0)).astype(np.float32))
            growth = _zp_union([_zp_geo(z) for z in ZONENPLAN['wachstum'] if z['net']!=net and z['layer']==L])
            soft.append(np.where(shapely.intersects_xy(growth,xx,yy),2.,1.).astype(np.float32))
        _ZP_CACHE[net] = (hard,np.concatenate([s.ravel() for s in soft]))
    return _ZP_CACHE[net]


def _zp_hart(net, ch, li, ys=slice(None), xs=slice(None)):
    if not ZONENPLAN: return ch
    spec=ZONENPLAN['netze'].get(net,{})
    if spec.get('lokal_ohne_lagenwechsel') and LAYERS[li]!=spec.get('lokale_lage'):
        return np.full(ch.shape,-1.,dtype=np.float32)
    return np.minimum(ch,2*(_zp_karten(net)[0][li][ys,xs]-CLR-.005))


def _zp_viah(net, v, ys=slice(None), xs=slice(None)):
    if not ZONENPLAN: return v
    if ZONENPLAN['netze'].get(net,{}).get('lokal_ohne_lagenwechsel'):
        return np.zeros(v.shape,dtype=bool)
    if ZONENPLAN.get('korridore') or net in _ZP_POT_NETZE:
        if net not in _ZP_VIA_MASK:
            xx,yy=np.meshgrid(X0+np.arange(NX)*RES,Y0+np.arange(NY)*RES)
            legal=np.ones((NY,NX),dtype=bool)
            for corridor in ZONENPLAN['korridore']:
                if corridor['net']==net:continue
                area=_ZP_GEO[(corridor['net'],corridor['layer'])]
                forbidden=shapely.intersects_xy(area.buffer(VIA_D/2+CLR+QM),xx,yy)
                allowed=np.zeros((NY,NX),dtype=bool)
                for slot in ZONENPLAN['via_fenster']:
                    if slot['net']==corridor['net'] and slot['layer']==corridor['layer'] and VIA_D<=slot['max_via_d']+1e-9:
                        allowed |= np.hypot(xx-slot['x'],yy-slot['y'])<=slot['radius_mm']
                legal &= ~forbidden | allowed
            if net in _ZP_POT_NETZE:
                area=_zp_union([_zp_geo(z) for z in ZONENPLAN['kern']
                                if z['net']==net and z.get('korridor')
                                and z['layer'] in ZONENPLAN['potenziallagen']])
                legal &= shapely.intersects_xy(area.buffer(-VIA_D/2-QM-.002),xx,yy)
            if len(_ZP_VIA_MASK)>=4:del _ZP_VIA_MASK[next(iter(_ZP_VIA_MASK))]
            _ZP_VIA_MASK[net]=legal
        v = v & _ZP_VIA_MASK[net][ys,xs]
    if _ZP_DRILL is not None:
        v = v & (_ZP_DRILL[ys,xs] >= VIA_DR/2+H2H+.004)
    return v

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NB = None
if args.get('numba', '1') == '1':
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
        from router.astar_nb import Suche as _Suche
        from geometry_nb import line_cap_min_kernel as _linecap_nb
        NB = _Suche(NL, NN, RES)
    except ImportError:
        NB = None
CLR_PAD = DRCP + 0.012
use = np.zeros(NL * NN, dtype=np.int16)
drill_use = np.zeros(NN, dtype=np.int16)
hist = np.zeros(NL * NN, dtype=np.float32)
if args.get('hist_start'):
    from lernen.selbstimitation import laden as _hist_laden
    hist[:] = _hist_laden(args['hist_start'], args.get('hist_context', ''), LAYERS, [X0, Y0], NX, NY, RES)
    print('Selbstimitation: Konfliktkarte geladen, max hist', float(hist.max()))
U = {}
STROM_AUDIT = {}
_SZ_VERSION = [0]; _SZ_RESERVE = [None]; _SZ_BODEN = {}


def strom_modell(net):
    if not STROM_ZUERST: return None
    k = KL['netze'].get(net) or {}; f = k.get('fluss') or {}
    if k.get('art') not in ('strom', 'versorgung', 'sw'): return None
    if f.get('einheit') != 'A' or not f.get('quelle') or not f.get('herkunft'): return None
    ls = f.get('last') or {}
    if not ls or not all(math.isfinite(float(v)) and float(v) >= 0 for v in ls.values()): return None
    if sum(map(float, ls.values())) <= 0: return None
    return f


def strom_plan(net, pads, stamm, KT):
    ""
    fl = strom_modell(net)
    if not fl: return None
    pk = {i: ('PAD', p['ref'], p['num']) for i, p in enumerate(pads)}
    names = {'%s:%s' % (p['ref'], p['num']): pk[i] for i, p in enumerate(pads)}
    if set(fl['quelle']) - set(names) or set(fl['last']) - set(names):
        raise ValueError('strom_zuerst: Flusspad fehlt bei ' + net)
    quelle = set(fl['quelle'])
    if any(float(fl['last'].get(q, 0)) > 0 for q in quelle):
        raise ValueError('strom_zuerst: Quelle zugleich Last bei ' + net)


    geplant = set(stamm) | {i for c in KT or [] for i in c['pins'] + c['caps']}
    verlangt = set(fl['last']) | quelle
    if verlangt - {'%s:%s' % (pads[i]['ref'], pads[i]['num']) for i in geplant}:
        raise ValueError('strom_zuerst: Last/Quelle ausserhalb des Strombaums ' + net)
    starts = [i for i, p in enumerate(pads) if '%s:%s' % (p['ref'], p['num']) in quelle]
    first = starts[0]
    if first not in stamm:
        first = next(c['caps'][-1] for c in KT or [] if first in c['pins'] + c['caps'])
    order = []; parent = {}; tree = [first]; rest = [i for i in stamm if i != first]
    while rest:
        _, j, a = min((math.dist((pads[i]['x'], pads[i]['y']), (pads[a]['x'], pads[a]['y'])), i, a) for i in rest for a in tree)
        parent[j] = a; order.append(j); tree.append(j); rest.remove(j)
    adj = {q: [] for q in pk.values()}; edges = []; keys = {}
    def edge(a, b, key):
        ti = len(edges); kid = ('s', ti, 0, 0., 1.)
        adj[pk[a]].append((pk[b], kid)); adj[pk[b]].append((pk[a], kid))
        edges.append(dict(layer='F', pts=[[pads[a]['x'], pads[a]['y']], [pads[b]['x'], pads[b]['y']]], w=[1.]))
        keys[key] = ti
    for j in order: edge(parent[j], j, ('stamm', j))
    for c in KT or []:
        cs, ps = c['caps'], c['pins']
        edge(cs[0], ps[0], ('kette', c['nr'], 0, cs[0]))
        for j in ps[1:]: edge(ps[0], j, ('kette', c['nr'], 0, j))
        for s, (a, b) in enumerate(zip(cs[:-1], cs[1:])): edge(b, a, ('kette', c['nr'], s + 1, b))
    from gemeinsam.fluss import analyse as _sz_fluss
    erreicht = {names[q] for q in quelle}; offen = list(erreicht)
    while offen:
        for q, _ in adj[offen.pop()]:
            if q not in erreicht: erreicht.add(q); offen.append(q)
    if any(names[q] not in erreicht for q in verlangt):
        raise ValueError('strom_zuerst: unerreichbare Last/Quelle im Strombaum ' + net)
    info = _sz_fluss(edges, [], pads, fl, graph=dict(adj=adj, padk=names, lagen=['F']))
    if info is None or any(pk[i] not in erreicht for i in geplant):
        raise ValueError('strom_zuerst: unvollstaendiger geplanter Flussbaum ' + net)
    amp = {key: info['gesamt'] * info['anteil'].get((ti, 0), 0.) for key, ti in keys.items()}
    STROM_AUDIT[net] = dict(strom_A=info['gesamt'], geplante_aeste=len(edges), quelle=sorted(quelle),
                           J=STROM_J, kupfer_mm=STROM_T, umweg=STROM_UMWEG)
    return first, order, parent, amp


def strom_klasse(C, I, a, b):
    widths = tuple(max(BAHN_MIN, C.get('dru_min', 0.), min(C['wmin'], HALS_W), I / (STROM_J * STROM_T[L])) for L in LAYERS)
    return dict(C, w=max(widths), wmin=max(widths), strom_w=widths, strom_I=I,
                strom_luft=(a['x'], a['y'], b['x'], b['y']), histw=float(args.get('histw_pw', '.3')),
                via=C['via'] * max(2, math.ceil(4 * I)))


def strom_segmentbreite(C, kapazitaet):


    return max(BAHN_MIN, min(HALS_W,C['w']), min(C['w'],kapazitaet))
def strom_boden(net, C):
    key = (net, C['strom_w'])
    if key not in _SZ_BODEN:
        while len(_SZ_BODEN) >= 4: del _SZ_BODEN[next(iter(_SZ_BODEN))]
        wf = hals_feld_karte(net, C, max(C['strom_w']))
        base = np.repeat(np.asarray(C['strom_w'], np.float32), NN)
        if wf is not None: base = np.minimum(base, wf)
        _SZ_BODEN[key] = base
    return _SZ_BODEN[key]


def strom_reservieren(net, capH, viaH):
    ""
    if not STROM_ZUERST: return capH, viaH
    pot = [li for li, L in enumerate(LAYERS) if L in STROM_POT and net not in STROM_POT[L]]
    reserve = strom_modell(net) is None and net not in GNDS and not paar_schnell(net)
    if not reserve and not pot: return capH, viaH


    hard = _Karten(); hard.flach = np.concatenate([c.ravel() for c in capH])
    hard.extend(hard.flach[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL))
    vh = viaH.copy()
    for li in pot: hard[li][:] = -1.
    if reserve:
        if _SZ_RESERVE[0] is None or _SZ_RESERVE[0][0] != _SZ_VERSION[0]:
            mask = np.zeros(NL * NN, bool); drills = np.zeros(NN, bool)
            for r in U.values():
                if strom_modell(r['net']): mask[r['cells']] = True; drills[r['dcells']] = True
            ds = [((_edt(~mask[li*NN:(li+1)*NN].reshape(NY,NX))*RES - QM - CLR - .005)*2).astype(np.float32)
                  if mask[li*NN:(li+1)*NN].any() else np.full((NY,NX),1e6,np.float32) for li in range(NL)]
            vd = (_edt(~drills.reshape(NY, NX))*RES - QM) if drills.any() else np.full((NY,NX), 1e6)
            _SZ_RESERVE[0] = (_SZ_VERSION[0], ds, vd)
        _, ds, vd = _SZ_RESERVE[0]
        for c, d in zip(hard, ds): np.minimum(c, d, out=c)
        vh &= vd >= HOLE_C
        for d in ds: vh &= d / 2 >= VIA_KEEP_R
    return hard, vh


def strom_feld_offsets(n):
    pitch = max(.5, HOLE_C, VIA_D + max(DRCC, DRCP) + .02)
    step = math.ceil(pitch / RES); r = math.ceil(math.sqrt(n))
    return sorted(((x * step, y * step) for y in range(-r, r + 1) for x in range(-r, r + 1)),
                  key=lambda q: (q[0]**2 + q[1]**2, q[1], q[0]))[:n]


def strom_stegbreite(C, n, L):
    return max(BAHN_MIN, VIA_D, C.get('strom_I', 0.) / n / (STROM_J * STROM_T[L]))


def strom_feld_maske(viaH, n, C, capH):
    ""
    offsets = strom_feld_offsets(n); legal = viaH.copy()
    for x,y in C.get('strom_vias', ()):
        ix,iy=int((x-X0)/RES),int((y-Y0)/RES); rr=math.ceil(HOLE_C/RES)+1
        x0,x1=max(0,ix-rr),min(NX,ix+rr+1);y0,y1=max(0,iy-rr),min(NY,iy+rr+1)
        gx,gy=np.meshgrid(GXc[x0:x1],GYc[y0:y1])
        legal[y0:y1,x0:x1] &= np.hypot(gx-x,gy-y) >= HOLE_C
    good = legal.copy()


    r = max(math.hypot(x, y) for x, y in offsets)
    for li,c in enumerate(capH):

        if LAYERS[li] in STROM_POT and NET_CUR[0] not in STROM_POT[LAYERS[li]]: continue
        good &= c >= 2 * r * RES + strom_stegbreite(C, n, LAYERS[li])
    for dx, dy in offsets:
        shifted = np.zeros_like(viaH)
        y0,y1=max(0,-dy),min(NY,NY-dy); x0,x1=max(0,-dx),min(NX,NX-dx)
        shifted[y0:y1,x0:x1] = legal[y0+dy:y1+dy,x0+dx:x1+dx]
        good &= shifted
    return good


def strom_feld_setzen(res, vs, C):
    stege, extra = [], []; n = max(2, math.ceil(4 * C['strom_I']-1e-8), C.get('strom_n',0))
    for v, (a, b) in zip(vs, [(a//NN,b//NN) for a,b in zip(res[:-1],res[1:]) if a//NN != b//NN]):
        v['n'] = n; v['strom_A'] = C['strom_I']; v['strom_lagen'] = [LAYERS[a], LAYERS[b]]
        for dx, dy in strom_feld_offsets(n)[1:]:
            x,y = round(v['x'] + dx*RES,4),round(v['y'] + dy*RES,4)
            extra.append(dict(x=x,y=y,gr=1,zu=[v['x'],v['y']],strom_A=C['strom_I']))
            for li in (a,b):

                p=(v['x'],v['y']); q=(x,y); mid=(p[0]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[0]-p[0]),
                                                p[1]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[1]-p[1]))
                pts=[list(p)]+([list(mid)] if mid!=p and mid!=q else [])+[list(q)]
                w=strom_stegbreite(C,n,LAYERS[li])
                stege.append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),steg=True))
    return stege, extra


def strom_felder_frei(res, vs, C, capH=None):
    ts, extra = strom_feld_setzen(res, copy.deepcopy(vs), C)
    points = [(v['x'],v['y']) for v in vs+extra] + list(C.get('strom_vias',()))
    if not all(math.dist(a,b) >= HOLE_C - 1e-6 for i,a in enumerate(points) for b in points[i+1:]): return False


    return capH is None or all(line_cap_min(capH[LI[t['layer']]], a, b) >= w
                              for t in ts for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']))

def strom_ist(tracks, vias, pads, fl):
    ""
    from gemeinsam.fluss import netzgraph, padgeo
    from shapely.geometry import Point, LineString
    from shapely.ops import nearest_points
    from shapely import STRtree
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve
    from collections import defaultdict
    tol = 1e-7
    def nk(L, p): return (L, round(p[0]/tol)*tol, round(p[1]/tol)*tol)
    segs = []
    for ti,t in enumerate(tracks):
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            if math.dist(a,b)>tol: segs.append((ti,si,t['layer'],LineString([a,b]),w))
    cuts = [{0.,s[3].length} for s in segs]; links = []; via_links = []
    bylayer = defaultdict(list)
    for i,s in enumerate(segs): bylayer[s[2]].append(i)
    for L,ids in sorted(bylayer.items()):
        tree = STRtree([segs[i][3].buffer(segs[i][4]/2+tol) for i in ids])
        for ii,i in enumerate(ids):
            a=segs[i][3]; aw=segs[i][4]
            for jj in sorted(tree.query(a.buffer(aw/2+tol))):
                j=ids[int(jj)]
                if j<=i: continue
                b=segs[j][3]
                if a.distance(b)> (aw+segs[j][4])/2+tol: continue
                inter=a.intersection(b)
                if inter.is_empty: pairs=[nearest_points(a,b)]
                elif inter.geom_type=='Point': pairs=[(inter,inter)]
                else:


                    gs=list(inter.geoms) if hasattr(inter,'geoms') else [inter]
                    pairs=[(Point(p),Point(p)) for g in gs for p in (list(g.coords) if hasattr(g,'coords') else [])]
                for p,q in pairs:
                    pa,pb=a.project(p),b.project(q); cuts[i].add(pa);cuts[j].add(pb)
                    links.append((nk(L,a.interpolate(pa).coords[0]),nk(L,b.interpolate(pb).coords[0])))
            for vi,v in enumerate(vias):
                p=Point(v['x'],v['y'])
                if a.distance(p)<=aw/2+float(v.get('d',VIA_D))/2+tol:
                    d=a.project(p); cuts[i].add(d)
                    via_links.append((nk(L,a.interpolate(d).coords[0]),nk(L,(v['x'],v['y']))))
            for p in pads:
                geo=padgeo(p).get(L)
                if geo is not None and a.distance(geo)<=aw/2+tol:
                    q,_=nearest_points(a,geo);cuts[i].add(a.project(q))
    split=[]; orig=[]
    for i,(ti,si,L,line,w) in enumerate(segs):
        ds=sorted(cuts[i])
        for a,b in zip(ds[:-1],ds[1:]):
            if b-a<=tol:continue
            split.append(dict(layer=L,pts=[list(line.interpolate(a).coords[0]),list(line.interpolate(b).coords[0])],w=[w]))
            orig.append((ti,si))
    g=netzgraph(split,[],pads,tol=tol);adj=g['adj'];padk=g['padk']
    for vi,v in enumerate(vias):
        ks=[nk(L,(v['x'],v['y'])) for L in LAYERS]
        for a,b in zip(ks[:-1],ks[1:]):
            adj[a].append((b,('v',vi)));adj[b].append((a,('v',vi)))
        for p in pads:
            for L,geo in padgeo(p).items():
                if L in LAYERS and geo.distance(Point(v['x'],v['y']))<=float(v.get('d',VIA_D))/2+tol:
                    links.append((('PAD',p['ref'],p['num']),nk(L,(v['x'],v['y']))))
    for a,b in links+via_links:
        adj[a].append((b,None));adj[b].append((a,None))

    parent={q:q for q in adj}
    def root(q):
        while parent[q]!=q: parent[q]=parent[parent[q]];q=parent[q]
        return q
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None: parent[root(b)]=root(a)
    sources={root(padk[q]) for q in fl['quelle'] if q in padk and padk[q] in parent}
    resistors={}; members=defaultdict(list)
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None or a>b:continue
            u,v=root(a),root(b)
            if u==v:continue
            if kid[0]=='s':
                t=split[kid[1]]
                resistance=1.72e-5*math.dist(t['pts'][0],t['pts'][1])*(kid[4]-kid[3])/(t['w'][0]*STROM_T[t['layer']])
                tag=('s',*orig[kid[1]])
            else:
                resistance=.0024/max(1,NL-1); tag=('v',kid[1],a[0],b[0])
            if resistance<=1e-15:continue
            key=('s',a,b) if kid[0]=='s' else ('v',kid[1],a,b)


            old=resistors.get(key)
            resistors[key]=(u,v,max(old[2] if old else 0.,1/resistance));members[key].append(tag)
    edges=defaultdict(float)
    for u,v,conductance in resistors.values():edges[tuple(sorted((u,v)))]+=conductance
    conn=defaultdict(list)
    for u,v in edges:conn[u].append(v);conn[v].append(u)
    reached=set(sources);todo=list(sources)
    while todo:
        for v in conn[todo.pop()]:
            if v not in reached:reached.add(v);todo.append(v)
    missing=sorted(q for q in set(fl['last'])|set(fl['quelle']) if q not in padk or padk[q] not in parent or root(padk[q]) not in reached)
    if missing:return dict(fehlend=missing,seg={},via={},gesamt=sum(map(float,fl['last'].values())))
    nodes=sorted(reached-sources);idx={q:i for i,q in enumerate(nodes)};rhs=np.zeros(len(nodes));row=[];col=[];val=[]
    for q,I in fl['last'].items():
        k=root(padk[q])
        if k in idx:rhs[idx[k]]+=float(I)
    for (u,v),conductance in edges.items():
        if u not in reached:continue
        for a,b in ((u,v),(v,u)):
            if a in idx:
                row.append(idx[a]);col.append(idx[a]);val.append(conductance)
                if b in idx:row.append(idx[a]);col.append(idx[b]);val.append(-conductance)
    voltage=spsolve(coo_matrix((val,(row,col)),shape=(len(nodes),len(nodes))).tocsr(),rhs) if nodes else np.zeros(0)
    if not np.all(np.isfinite(voltage)):raise ValueError('strom_zuerst: singulaerer Endgraph')
    amps={};viamps={}
    for key,(u,v,conductance) in resistors.items():
        if u not in reached:continue
        I=abs((voltage[idx[u]] if u in idx else 0.)-(voltage[idx[v]] if v in idx else 0.))*conductance
        for tag in members[key]:
            if tag[0]=='s':amps[tag[1:]]=max(amps.get(tag[1:],0.),I)
            else:viamps[tag[1:]]=max(viamps.get(tag[1:],0.),I)
    return dict(fehlend=[],seg=amps,via=viamps,gesamt=sum(map(float,fl['last'].values())))


def strom_korrigieren(u, C):
    ""
    r=U[u];net=r['net'];fl=strom_modell(net)
    info=strom_ist(r['tracks'],r['vias'],[D['pads'][i] for i in bynet[net]],fl)
    if info['fehlend']:return info
    ziel={(ti,si):max(BAHN_MIN,DRU_MIN.get(net,0.),min(HALS_W,C['wmin']),I/(STROM_J*STROM_T[r['tracks'][ti]['layer']]))
          for (ti,si),I in info['seg'].items()}
    FLUSS_W[u]=ziel


    points=[p for t in r['tracks'] for p in t['pts']]+[[v['x'],v['y']] for v in r['vias']]
    xs=[p[0] for p in points];ys=[p[1] for p in points]
    limit=max([C.get('w',0.)]+list(ziel.values())+[w for t in r['tracks'] for w in t['w']])
    capH,*_=karten(net,(min(xs)-2.,min(ys)-2.,max(xs)+2.,max(ys)+2.),limit)
    sb=_stoer_breite(net)
    X=_kidx_bauen(u);tr=copy.deepcopy(r['tracks']);vs=copy.deepcopy(r['vias'])
    kz=_kz_vorbereiten(r,net) if KETTE and any('kz' in t for t in tr) else None
    ab=_abg_vorbereiten(r,net) if ABG_AN and any(t.get('abg') for t in tr) else None
    widths=[list(t['w']) for t in tr]
    for ti,t in enumerate(tr):
        li=LI[t['layer']]
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            target=max(w,ziel.get((ti,si),w));limit=_r32_freiraum(X,u,li,a,b,net,target)
            limit=min(limit,line_cap_min(capH[li],a,b),2*pad_room(li,a,b,net))
            if kz is not None:limit=min(limit,_kz_breite(kz,r,widths,ti,a,b,li))
            if ab is not None:limit=min(limit,_abg_breite(ab,a,b,li))
            for dk,sp in sb:
                limit=min(limit,2*(line_cap_min(dk[li],a,b)-sp))
                for lj,s in nachbar_abstand(li,sp):limit=min(limit,2*(line_cap_min(dk[lj],a,b)-s))
            t['w'][si]=max(w,min(target,limit))
        widths[ti]=t['w']


    for vi,v in enumerate(vs):
        v['strom_ist_A']=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.])
        if not v.get('n'):continue
        ids={j for j,w in enumerate(vs) if j==vi or w.get('zu')==[v['x'],v['y']]}
        sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        I=max(sums.values(),default=0.);v['strom_A']=I
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*I-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
        if v['n']<=len(ids):continue
        layers=v.get('strom_lagen')
        if not layers:continue
        stege,extras=strom_feld_setzen([LI[layers[0]]*NN,LI[layers[1]]*NN],[dict(x=v['x'],y=v['y'])],dict(strom_I=I,strom_n=v['n']))
        template=next((t for t in tr if t.get('steg') and t.get('kette')==v.get('kette')), {})
        for t in stege:t.update({k:template[k] for k in ('kette','kp') if k in template})
        new=[w for w in extras if all(math.dist((w['x'],w['y']),(q['x'],q['y']))>.02 for q in vs)]
        if not new:continue
        if any(math.dist((w['x'],w['y']),(q['x'],q['y']))<HOLE_C-1e-6 for w in new for q in vs):continue
        good=True
        for w in new:
            ix,iy=int((w['x']-X0)/RES),int((w['y']-Y0)/RES);q=(w['x'],w['y'])
            good &= 0<=ix<NX and 0<=iy<NY
            if not good:break
            good &= via_hart(net,np.array([iy*NX+ix]))
            good &= all(_r32_freiraum(X,u,li,q,q,net,VIA_D)>=VIA_D-1e-9 for li in range(NL))
            good &= all(math.dist(q,(v2['x'],v2['y']))>=HOLE_C-1e-6
                        for uu,rr in U.items() if uu!=u for v2 in rr['vias'])
        if not good:continue
        for t in stege:
            li=LI[t['layer']]
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                good &= _r32_freiraum(X,u,li,a,b,net,w)>=w-1e-9 and 2*pad_room(li,a,b,net)>=w-1e-9
                good &= line_cap_min(capH[li],a,b)>=w-1e-9
                if kz is not None:
                    rr=dict(r,tracks=tr+[t]);ww=widths+[t['w']]
                    good &= _kz_breite(kz,rr,ww,len(tr),a,b,li)>=w-1e-9
                if ab is not None:good &= _abg_breite(ab,a,b,li)>=w-1e-9
        if good:tr+=stege;vs+=new
    del_unit(u);add_unit(u,net,tr,vs)
    return info


def strom_abschluss(u, C):
    ""
    for _ in range(8):
        before=copy.deepcopy((U[u]['tracks'],U[u]['vias']))
        info=strom_korrigieren(u,C)
        if info['fehlend'] or before==(U[u]['tracks'],U[u]['vias']):break
    r=U[u];info=strom_ist(r['tracks'],r['vias'],[D['pads'][i] for i in bynet[r['net']]],strom_modell(r['net']))
    deficit=[]
    for (ti,si),I in info['seg'].items():
        t=r['tracks'][ti];target=I/(STROM_J*STROM_T[t['layer']])
        if target>t['w'][si]+1e-6:deficit.append(dict(bahn=ti,segment=si,strom_A=I,ist_mm=t['w'][si],soll_mm=target))
    via_deficit=[]
    for vi,v in enumerate(r['vias']):
        I=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.]);v['strom_ist_A']=I
        if I>.25+1e-6:via_deficit.append(dict(via=vi,strom_A=I,soll_A=.25))


    for vi,v in enumerate(r['vias']):
        if not v.get('n'):continue
        ids={j for j,w in enumerate(r['vias']) if j==vi or w.get('zu')==[v['x'],v['y']]};sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        v['strom_A']=max(sums.values(),default=0.)
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*v['strom_A']-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
    audit=STROM_AUDIT.setdefault(r['net'],{})
    audit['endpruefung']=dict(modell='Widerstandsgraph/Kirchhoff, ideale Kontakte',strom_A=info['gesamt'],
                             fehlende_pads=info['fehlend'],breitendefizite=deficit,viadefizite=via_deficit,
                             viafeld_fehl=vg_fehl(u))
    return not (info['fehlend'] or deficit or via_deficit or vg_fehl(u))
IMP_REF_MASK = {}; IMP_REF_GEOM = {}
if IMP_TAB and D.get('plane_domains'):
    from analyse.aufbau import bezugsflaechen as _z_flaechen
    _z_polys = _z_flaechen(D, GNDS, dict(optionen={'gndlagen': args.get('gndlagen', 'In1')}), _alle_netze)
    _z_x, _z_y = np.meshgrid(GXc, GYc)
    for _net, _tab in sorted(IMP_TAB.items()):
        for _layer, _q in sorted(_tab.items()):
            _key = (_net, _layer)
            _mask = np.ones((NY, NX), dtype=bool)
            _geom = None
            for _ref in _q['profil']['ref']:
                _p = _z_polys.get(_ref, Polygon())
                _geom = _p if _geom is None else _geom.intersection(_p)
            IMP_REF_GEOM[_key] = _geom

            _mask &= shapely.contains_xy(_geom.buffer(-_q['w']/2 - RES), _z_x, _z_y)
            IMP_REF_MASK[_key] = _mask
    del _z_x, _z_y


def z_ref_frei(C, li, a, b, net=None):
    ""
    if not C.get('zw'): return True
    g = IMP_REF_GEOM.get((NET_CUR[0] if net is None else net, LAYERS[li]))
    return g is None or g.covers(_LS([a, b]).buffer(C['zw'][li]/2))

PLANE_MASK = {}; PLANE_AREA = {}; PLANE_INFO = None
PLANE_COPPER = {}; PLANE_TRACE = {}; PLANE_LAYER = {}; PLANE_REF = {}
PLANE_FEEDERS = False
if (BGA_PLAN or PINZUGANG) and D.get('plane_domains'):
    PLANE_LAYER = {}; PLANE_COPPER = {}


    import hashlib as _plane_hash
    _raw = open(D['plane_domains'], 'rb').read(); _domains = json.loads(_raw)
    PLANE_FEEDERS = PINZUGANG and bool(_domains.get('group_feeders',False))
    if PINZUGANG and _domains.get('outline_targets') and not _domains.get('reference_domains'):
        raise ValueError('outline_targets braucht native reference_domains')
    PLANE_REF={}
    if PINZUGANG:
        for _z in _domains.get('reference_domains',_domains['zones']):
            PLANE_REF.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        PLANE_REF={key:shapely.union_all(gs) for key,gs in PLANE_REF.items()}
    _plane_nets = {}
    if PINZUGANG and not BGA_PLAN:

        _plane_nets = {n: [] for n in sorted(FLNETZE)}
    for _z in _domains['zones']:
        if PINZUGANG:PLANE_LAYER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z.get('layer') in globals().get('PLANE_SIGNALS', set()):
            PLANE_COPPER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z['net'] in FLNETZE:
            if PINZUGANG and not BGA_PLAN and _z.get('layer') not in [L for L,nn in FL_ZUORDNUNG.items() if _z['net'] in nn]: continue
            _plane_nets.setdefault(_z['net'], []).append(Polygon(_z['pts'], _z.get('holes', [])).buffer(0))
    for _n, _gs in sorted(_plane_nets.items()):
        _area = shapely.union_all(_gs).buffer(-VIA_R - 0.01)
        if PINZUGANG and _domains.get('outline_targets') and _n in globals().get('PLANE_MIXED_NETS',()) and _n not in GNDS:

            _original=[g for (n,L),g in PLANE_REF.items() if n==_n and L in PLANE_SIGNALS and n in FL_ZUORDNUNG.get(L,[])]
            _area=shapely.union_all(_original).buffer(-VIA_R-.01) if _original else Polygon()
        PLANE_AREA[_n] = _area
        _mask = np.zeros((NY, NX), dtype=bool)
        if not _area.is_empty:
            _x0,_y0,_x1,_y1 = _area.bounds
            _ix0,_ix1 = max(0,int((_x0-X0)/RES)),min(NX,int((_x1-X0)/RES)+2)
            _iy0,_iy1 = max(0,int((_y0-Y0)/RES)),min(NY,int((_y1-Y0)/RES)+2)
            _xx,_yy = np.meshgrid(GXc[_ix0:_ix1], GYc[_iy0:_iy1])
            _mask[_iy0:_iy1,_ix0:_ix1] = shapely.contains_xy(_area,_xx,_yy)
        PLANE_MASK[_n] = _mask
    PLANE_INFO = dict(sha256=_plane_hash.sha256(_raw).hexdigest(),netze=sorted(PLANE_AREA),lagen=_domains.get('gndlagen'))
    PLANE_COPPER = {key:shapely.union_all(gs) for key,gs in PLANE_COPPER.items()}
    PLANE_LAYER = {key:shapely.union_all(gs) for key,gs in PLANE_LAYER.items()}


def plane_trace_maps(net, capH, rng=None):
    ""
    if not PLANE_COPPER: return
    ix0,ix1,iy0,iy1 = rng if rng is not None else (0,NX,0,NY)
    ys,xs=slice(iy0,iy1),slice(ix0,ix1)
    for (n,L),g in PLANE_COPPER.items():
        if n==net:continue
        key=(n,L)
        if key not in PLANE_TRACE:
            xx,yy=np.meshgrid(GXc,GYc)
            blocked=shapely.intersects_xy(g,xx,yy)
            PLANE_TRACE[key]=(_edt(~blocked)*RES-QM-CLR-.005).astype(np.float32)*2
            PLANE_TRACE[key][blocked]=-1.
        np.minimum(capH[LI[L]][ys,xs],PLANE_TRACE[key][ys,xs],out=capH[LI[L]][ys,xs])


def plane_feeder_targets(net, p, li, viaH, viaBad, rng):
    ""
    x0,x1,y0,y1=rng
    mask=viaH[y0:y1,x0:x1] & ~viaBad[y0:y1,x0:x1] & PLANE_MASK[net][y0:y1,x0:x1]
    yy,xx=np.nonzero(mask);xx=xx+x0;yy=yy+y0
    if not len(xx):return set()
    d=(GXc[xx]-p['x'])**2+(GYc[yy]-p['y'])**2
    order=np.lexsort((xx,yy,d))[:32]
    return {li*NN+int(yy[i])*NX+int(xx[i]) for i in order}
PF = [float(args.get('pf0', '0.6'))]
HART_FREMD = [False]
QUELLFREI = [args.get('quellfrei', '0') == '1']
NET_CUR = ['']


def okt_netz(net=None):

    return not _netz_organisch(NET_CUR[0] if net is None else net, KL) if STIL_JE_NETZART else OKT

STOER_FAK = [1.0]
VIA_MASKE = [None]
VIA_FAK = [1.0]
VIAF = VIA_FAK
ASTAR_COST = [None]
ASTAR_HINT = [None]
_LOOP_DEADLINE = None


def _loop_cpu_check():
    arbeit_notbremse()
    if _LOOP_DEADLINE is not None and budget_uhr() >= _LOOP_DEADLINE:
        raise TimeoutError('Loop-CPU-Budget in Suche')

TAPS = args.get('abgriffe', '1' if KL and KL.get('abgriffe') else '0') == '1'



ABG_KL = TAPS and bool(KL) and 'abgriffe' in KL and args.get('abgriffe_quelle', 'klassen') == 'klassen'
FENSTER = float(args.get('fenster', '0'))
KONFLIKTORT_EXAKT = args.get('konfliktort', '') == 'exakt'
HALS = float(args.get('hals', '0')); HALS_CACHE = {}
_HALS_MAX = max(4, int(float(args.get('hals_mb', '600')) * 1e6 / (NL * NN * 4)))


HFAC = float(args.get('astar', '1'))


KNICK = float(args.get('knick', '0'))
if not math.isfinite(KNICK) or KNICK < 0: raise ValueError('knick muss endlich und >= 0 sein')


KNICK_STROM = float(args.get('knick_strom', '1'))
if not math.isfinite(KNICK_STROM) or KNICK_STROM < 0: raise ValueError('knick_strom muss endlich und >= 0 sein')
VIA_ZU = [None]




VIA_STEG = float(args.get('via_steg', '0.03'))
HALS_W = float(args.get('hals_w', args.get('sigw', '0.15') if KL else '0.25'))

HALS_FELD = args.get('hals_feld', '1' if KL else '0') == '1'
DICHT = []
if HALS_FELD:
    from scipy.spatial import cKDTree as _KD
    _je = {}
    for _p in D['pads']: _je.setdefault(_p['ref'], []).append(_p)
    for _r in sorted(_je):
        _P = _je[_r]
        if len(_P) < 16: continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _pit = float(np.median(_KD(_xy).query(_xy, k=2)[0][:, 1]))
        if _pit > 0.85: continue
        _lg = sorted({LI[L] for q in _P for L in q['layers'] if L in LI})
        DICHT.append((_lg, max(0, int((_xy[:, 0].min() - _pit - X0) / RES)), min(NX, int((_xy[:, 0].max() + _pit - X0) / RES) + 1),
                      max(0, int((_xy[:, 1].min() - _pit - Y0) / RES)), min(NY, int((_xy[:, 1].max() + _pit - Y0) / RES) + 1),
                      {q['net'] for q in _P if q['net']}))
    if DICHT: print('dichte Felder (BGA, feines Raster):', len(DICHT))
bynet_g = {}
for _k, _p in enumerate(D['pads']):
    if _p['net']: bynet_g.setdefault(_p['net'], []).append(_k)
_LEER = np.zeros(0, dtype=np.float32)
_MULT = np.empty(NL * NN, dtype=np.float32)


class _Karten(list):
    ""
    flach = None





NAH = args.get('nahkarte', '1') == '1'
NAH_W = float(args.get('nah_w', '0.6'))




FLAECHE = float(args.get('flaeche', '0'))
FL_LUECKE = float(args.get('flaeche_luecke', '0.6'))
FL_WMAX = float(args.get('flaeche_wmax', '0.3'))



FL_PREIS = float(args.get('flaeche_preis', '0'))
if FLAECHE > 0 or FL_PREIS > 0: NAH_W = max(NAH_W, 2 * (FL_LUECKE + FL_WMAX / 2 - CLR - RES))
D_NAH = max(NAH_W, VIA_D) / 2 + CLR + 2 * RES; PCN = int(math.ceil(D_NAH / RES)) + 2
CAP_NAH = np.float32(2 * (D_NAH - CLR))
D_BOHR = HOLE_C + 2 * RES; PCB = int(math.ceil(D_BOHR / RES)) + 2
SN = {}
UOWN = {}
NETU = {}
DDN = np.full(NN, D_BOHR, dtype=np.float32)
SN_OFFEN = {}
_TMP = np.zeros(NL * NN, dtype=bool)
_NVER = {}
_GEO = {}
_STAMM = {}
_STAMM_V = {}
if NAH:
    SN[''] = np.full(NL * NN, CAP_NAH, dtype=np.float32)
    for _g in sorted(FLNETZE):
        SN[_g] = np.full(NL * NN, CAP_NAH, dtype=np.float32); UOWN[_g] = np.zeros(NL * NN, dtype=np.int16); SN_OFFEN[_g] = []
FIXD = [None, None]
FIXNETS = set()
OFFS = [(1, 0, RES), (-1, 0, RES), (0, 1, RES), (0, -1, RES),
        (1, 1, RES * 1.4142), (1, -1, RES * 1.4142), (-1, 1, RES * 1.4142), (-1, -1, RES * 1.4142)]
GXc = X0 + (np.arange(NX) + 0.5) * RES; GYc = Y0 + (np.arange(NY) + 0.5) * RES


def seg_cells(li, a, b, w):
    (x0, y0), (x1, y1) = a, b
    r = w / 2 + QM
    ixa = max(0, int((min(x0, x1) - r - X0) / RES) - 1); ixb = min(NX, int((max(x0, x1) + r - X0) / RES) + 2)
    iya = max(0, int((min(y0, y1) - r - Y0) / RES) - 1); iyb = min(NY, int((max(y0, y1) + r - Y0) / RES) + 2)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    dx, dy = x1 - x0, y1 - y0; ll = dx * dx + dy * dy
    t = np.clip(((GX - x0) * dx + (GY - y0) * dy) / ll, 0, 1) if ll > 0 else 0
    m = np.hypot(GX - (x0 + t * dx), GY - (y0 + t * dy)) <= r
    yy, xx = np.nonzero(m)
    return li * NN + (yy + iya) * NX + (xx + ixa)


def drill_cells(x, y):
    r = VIA_DR / 2
    ix, iy = int((x - X0) / RES), int((y - Y0) / RES); rr = int(r / RES) + 1
    out = []
    for jy in range(iy - rr, iy + rr + 1):
        for jx in range(ix - rr, ix + rr + 1):
            if 0 <= jx < NX and 0 <= jy < NY and math.hypot(GXc[jx] - x, GYc[jy] - y) <= r + QM: out.append(jy * NX + jx)
    return np.array(out, dtype=np.int64)


def _vd_slice(x, y):
    ""
    r = VD_FENSTER / 2
    ax = max(0, int(math.ceil((x - r - X0) / RES - 0.5 - 1e-9)))
    bx = min(NX, int(math.floor((x + r - X0) / RES - 0.5 + 1e-9)) + 1)
    ay = max(0, int(math.ceil((y - r - Y0) / RES - 0.5 - 1e-9)))
    by = min(NY, int(math.floor((y + r - Y0) / RES - 0.5 + 1e-9)) + 1)
    return slice(ay, by), slice(ax, bx)


def _vd_update(net, vias, delta):
    for v in vias:
        key = (round(v['x'], 6), round(v['y'], 6), v.get('net') or net)
        vorher = _VD_VIAS.get(key, 0); nachher = vorher + delta
        if nachher < 0: raise RuntimeError('Via-Dichte-Referenzzaehler negativ')
        if bool(vorher) != bool(nachher):
            _VD_ZAHL[_vd_slice(v['x'], v['y'])] += 1 if nachher else -1
            _VD_VERSION[0] += 1
        if nachher: _VD_VIAS[key] = nachher
        else: _VD_VIAS.pop(key, None)


def _vd_fenster_max(z):
    ""
    n = 2 * int(math.floor(VD_FENSTER / (2 * RES) + 1e-9)) + 1
    return ndimage.maximum_filter1d(ndimage.maximum_filter1d(z, n, axis=0, mode='constant', cval=0),
                                    n, axis=1, mode='constant', cval=0)


def _vd_kosten(lokal=()):
    ""
    c = _VD_CACHE[0]
    if not lokal and c is not None and c[0] == _VD_VERSION[0]: return c[1]
    z = _VD_ZAHL.astype(np.float32)
    seen = set()
    for v in lokal:
        key = (round(v['x'], 6), round(v['y'], 6))
        vorhanden = (*key, v.get('net') or NET_CUR[0]) in _VD_VIAS
        if key not in seen and not vorhanden:
            z[_vd_slice(*key)] += 1; seen.add(key)


    z = np.square(np.maximum(np.float32(0), _vd_fenster_max(z) + np.float32(1 - VD_SCHWELLE * VD_FENSTER ** 2)))
    if not lokal: _VD_CACHE[0] = (_VD_VERSION[0], z)
    return z


def add_unit(u, net, tracks, vias):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    if globals().get('PINZUGANG', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if strom_modell(net): _SZ_VERSION[0] += 1
    vv = []
    for v in vias:
        if all(math.hypot(v['x'] - w['x'], v['y'] - w['y']) > 0.02 for w in vv): vv.append(v)
    vias = vv
    cl = []
    for t in tracks:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): cl.append(seg_cells(li, a, b, w))
    dl = []
    for v in vias:


        for li in range(NL): cl.append(seg_cells(li, (v['x'], v['y']), (v['x'], v['y']), 2 * VIA_OCC_R))
        dl.append(drill_cells(v['x'], v['y']))
    cells = np.unique(np.concatenate(cl)) if cl else np.zeros(0, dtype=np.int64)
    dcells = np.unique(np.concatenate(dl)) if dl else np.zeros(0, dtype=np.int64)
    use[cells] += 1; drill_use[dcells] += 1
    _STAMM.clear(); _STAMM_V.clear()
    bb = _teilboxen(cl, dl) if NAH else None
    U[u] = dict(net=net, tracks=tracks, vias=vias, cells=cells, dcells=dcells, bb=bb)
    if VD_ZAEHLER: _vd_update(net, vias, 1)
    _NVER[net] = _NVER.get(net, 0) + 1; _GEO.pop(u, None)
    if NAH:
        NETU[net] = NETU.get(net, 0) + 1
        if net in UOWN: UOWN[net][cells] += 1
        _nah_update(bb, net, cells)


def del_unit(u):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    r = U.pop(u, None)
    _STAMM.clear(); _STAMM_V.clear()
    if r is None: return
    if globals().get('PINZUGANG', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if strom_modell(r['net']): _SZ_VERSION[0] += 1
    if VD_ZAEHLER: _vd_update(r['net'], r['vias'], -1)
    if VIA_ANKER > 0: VIA_ALT[u] = [(v['x'], v['y']) for v in r['vias'] if not v.get('rv') and not v.get('gr')]
    use[r['cells']] -= 1; drill_use[r['dcells']] -= 1
    _NVER[r['net']] = _NVER.get(r['net'], 0) + 1; _GEO.pop(u, None)
    if NAH:
        NETU[r['net']] -= 1
        if r['net'] in UOWN: UOWN[r['net']][r['cells']] -= 1
        _nah_update(r['bb'], r['net'])


def _verschmelzen(B, rand):
    ""
    B = sorted(set(B)); fl = lambda b: (b[1] - b[0] + 2 * rand) * (b[3] - b[2] + 2 * rand) + 6000
    i = 0
    while i < len(B):
        j = i + 1; neu = False
        while j < len(B):
            a, b = B[i], B[j]; v = (min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3]))
            if fl(v) <= fl(a) + fl(b): B[i] = v; del B[j]; neu = True
            else: j += 1
        if not neu: i += 1
    return B


def _teilboxen(cl, dl):
    ""
    je = {}
    for a in cl:
        if not len(a): continue
        li = int(a[0]) // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        je.setdefault(li, []).append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    bo = []
    for a in dl:
        if not len(a): continue
        iy = a // NX; ix = a - iy * NX
        bo.append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    return dict(kupfer=[(li, b) for li in sorted(je) for b in _verschmelzen(je[li], 2 * PCN)], bohr=_verschmelzen(bo, 2 * PCB))


def _nah_neu(S, key, li, y0, y1, x0, x1):
    ""
    ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
    wy0, wy1, wx0, wx1 = max(0, ay0 - PCN), min(NY, ay1 + PCN), max(0, ax0 - PCN), min(NX, ax1 + PCN)
    sl = slice(li * NN, (li + 1) * NN)
    ul = use[sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]
    f = (ul > 0) if key == '' else ((ul - UOWN[key][sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]) > 0)
    ziel = S[sl].reshape(NY, NX)
    if f.any():
        d = (_edt(~f) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
        ziel[ay0:ay1, ax0:ax1] = np.minimum(2 * (d - CLR), CAP_NAH)
    else:
        ziel[ay0:ay1, ax0:ax1] = CAP_NAH


def _nah_update(bb, net, cells=None):
    ""
    S = SN['']
    if cells is not None: _TMP[cells] = True
    for li, (y0, y1, x0, x1) in bb['kupfer']:
        if cells is None: _nah_neu(S, '', li, y0, y1, x0, x1)
        else:
            ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
            sl = slice(li * NN, (li + 1) * NN)
            d = (_edt(~_TMP[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]) * RES).astype(np.float32)
            z = S[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]
            np.minimum(z, np.minimum(2 * (d - CLR), CAP_NAH), out=z)
    if cells is not None: _TMP[cells] = False
    for key, off in SN_OFFEN.items():
        if key == net or off is None: continue
        off.extend((li, b) for li, b in bb['kupfer'])
        if len(off) > 400: SN_OFFEN[key] = None
    D2 = DDN.reshape(NY, NX); du = drill_use.reshape(NY, NX)
    for y0, y1, x0, x1 in bb['bohr']:
        ay0, ay1, ax0, ax1 = max(0, y0 - PCB), min(NY, y1 + PCB), max(0, x0 - PCB), min(NX, x1 + PCB)
        wy0, wy1, wx0, wx1 = max(0, ay0 - PCB), min(NY, ay1 + PCB), max(0, ax0 - PCB), min(NX, ax1 + PCB)
        fd = du[wy0:wy1, wx0:wx1] > 0
        if fd.any():
            dd = (_edt(~fd) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
            D2[ay0:ay1, ax0:ax1] = np.minimum(dd, D_BOHR)
        else:
            D2[ay0:ay1, ax0:ax1] = D_BOHR


def _nah_flush(key):
    ""
    off = SN_OFFEN[key]
    if off is None:
        S = SN[key]
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            f = ((use[sl] - UOWN[key][sl]) > 0).reshape(NY, NX)
            if f.any(): S[sl] = np.minimum(2 * ((_edt(~f) * RES).astype(np.float32) - CLR), CAP_NAH).ravel()
            else: S[sl] = CAP_NAH
    else:
        je = {}
        for li, b in off: je.setdefault(li, []).append(b)
        for li in sorted(je):
            for b in _verschmelzen(je[li], 2 * PCN): _nah_neu(SN[key], key, li, *b)
    SN_OFFEN[key] = []


def own_counts(net):
    own = np.zeros(NL * NN, dtype=np.int16); od = np.zeros(NN, dtype=np.int16)
    for r in U.values():
        if r['net'] == net:
            own[r['cells']] += 1; od[r['dcells']] += 1
    return own, od


def _box_idx(box):
    ""
    if box is None: return (0, NX, 0, NY)
    ix0, ix1 = max(0, int((box[0] - X0) / RES) - 1), min(NX, int((box[2] - X0) / RES) + 2)
    iy0, iy1 = max(0, int((box[1] - Y0) / RES) - 1), min(NY, int((box[3] - Y0) / RES) + 2)
    if ix1 <= ix0 or iy1 <= iy0: return (0, NX, 0, NY)
    return (ix0, ix1, iy0, iy1)




_KA_META = {}
_KA_HARD = {}
_KA_CELLS = {}


def _ka_kind(g, pad=False):
    return _KA_META.get(id(g), ('Pad' if pad else 'Track', ''))


def _ka_pad_kind(p):
    if p.get('graphic') or p.get('board_copper') or p.get('ref','').startswith('CU_'):
        return ('Graphic','')
    return ('Pad',p.get('dru_pad_type') or ('NPTH, mechanical' if p.get('pad_type')=='npth' else 'Through-hole' if p.get('drill') else 'SMD'))


def _ka_pad_keeps(p,L):
    ""
    mk=(p.get('maske') or {}).get(L,0.)
    if mk>0:
        g=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        yield g.buffer(mk+.01,join_style=2),'MaskKeepout'
    if p.get('drill',0)>0 and LOCH_CLR>0:
        g=(shapely.union_all([Polygon(q) for q in p['hole_polys']])
           if DRCL and p.get('hole_polys') else _PT(p['x'],p['y']).buffer(p['drill']/2,32))
        yield g.buffer(LOCH_CLR,32),'HoleKeepout'


def _ka_geo_limit(net, other, li, g=None, g2=None, pad=False):
    if globals().get('KA') is None: return _pz_clearance(net,other,pad)
    t,p = _ka_kind(g) if g is not None else ('Track','')
    t2,p2 = _ka_kind(g2,pad)
    return _ka_margin(net,other,li,t,t2,p,p2)
def _ka_obstacles(net, li, fixed=True, ohne=None, pads=True):
    if pads:
        for n,g in _PADG[li]:
            if n != net and not g.is_empty:
                typ,pad = _ka_kind(g,True)
                yield n,g,typ,pad
        for n,g,typ in globals().get('_KA_PAD_KEEP',{}).get(li,[]):
            if n!=net and not g.is_empty:yield n,g,typ,''
    for u,r in U.items():
        if u == ohne or (fixed and not u.startswith('FIX#')): continue
        for l,g,_,n in _unit_geo(u,r):
            if l==li and n!=net and not g.is_empty:
                typ,pad = _ka_kind(g)
                yield n,g,typ,pad
    for source in (_ZP_GEO, PLANE_COPPER):
        for (n,L),g in source.items():
            if n!=net and L==LAYERS[li] and not g.is_empty:yield n,g,'Zone',''


def _ka_field(net, li, points, obstacles, typ='Track', margin=.012):
    groups={}
    for n,g,t,p in obstacles:
        c=_ka_clr(net,n,li,typ,t,'',p)+margin
        groups.setdefault(c,[]).append(g)
    out=np.full(len(points),9.,dtype=np.float64)
    for c,gs in sorted(groups.items()):

        d=_pz_distance(shapely.STRtree(gs),points)
        out=np.minimum(out,d-c)
    return out


def _ka_raster(net,li,rng,obstacles,typ='Track',margin=.012,raw=False):


    x0,x1,y0,y1=rng
    cap=max(1.,cls(net)['w']*max(1.,BREITE_INNEN)/2+.5,VIA_KEEP_R+.5)
    out=np.full((y1-y0,x1-x0),cap,dtype=np.float32)
    for n,g,t,p in obstacles:
        if g.is_empty:continue
        c=0. if raw else _ka_clr(net,n,li,typ,t,'',p)+margin
        bx=g.bounds;r=cap+c+RES
        xa=max(x0,int(math.floor((bx[0]-r-X0)/RES)));xb=min(x1,int(math.ceil((bx[2]+r-X0)/RES)))
        ya=max(y0,int(math.floor((bx[1]-r-Y0)/RES)));yb=min(y1,int(math.ceil((bx[3]+r-Y0)/RES)))
        if xa>=xb or ya>=yb:continue
        xx,yy=np.meshgrid(GXc[xa:xb],GYc[ya:yb])
        d=shapely.distance(shapely.points(xx,yy),g)-c
        v=out[ya-y0:yb-y0,xa-x0:xb-x0]
        np.minimum(v,d,out=v)
    return out


def _ka_hard_maps(net,rng=None):
    rng=tuple(rng) if GROSS and rng is not None else _VOLL
    key=(net,rng,_PZ_VERSION[0],tuple(sorted(PZ_LOKAL.get(net,()))))
    if key in _KA_HARD:return _KA_HARD[key]
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    margin=PZ_RAND_NM/1e6 if _pz_active(net) else .012
    capH=_Karten();capH.flach=np.full(NL*NN,-1.,dtype=np.float32)
    via=(edge_d[ys,xs]-EDGE>=VIA_R)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
    if VIA_VERBOT is not None:via &= ~VIA_VERBOT[ys,xs]
    fx0,fx1=_fix_karte(net)
    if fx1 is not None:via &= fx1[ys,xs]-QM>=HOLE_C
    for li in range(NL):
        obs=list(_ka_obstacles(net,li))
        track=_ka_raster(net,li,rng,obs,margin=margin)
        vroom=_ka_raster(net,li,rng,obs,'Via',margin)
        layer=capH.flach[li*NN:(li+1)*NN].reshape(NY,NX)
        layer[ys,xs]=2*np.minimum(track,edge_d[ys,xs]-EDGE)
        capH.append(layer)
        via &= vroom>=VIA_R
        if LOCH_CLR>0:
            raw=_ka_raster(net,li,rng,[o for o in obs if o[2] not in ('MaskKeepout','HoleKeepout')],raw=True)
            via &= raw>=VIA_DR/2+LOCH_CLR+margin
        if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_STEG
    viaH=np.zeros((NY,NX),dtype=bool);viaH[ys,xs]=via
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
    viaH[ys,xs]=_zp_viah(net,viaH[ys,xs],ys,xs)
    if PAD_LOCH_ABST is not None:viaH[ys,xs] &= _pad_loch_frei(PAD_LOCH_ABST[ys,xs])
    if VP:_vp_ein(net,viaH,rng)

    allowed=viaH.copy()
    if EPZ:_ep_ein(net,viaH,capH,rng)
    viaH &= allowed
    plane_trace_maps(net,capH,rng)
    while len(_KA_HARD)>=_HM_MAX:_KA_HARD.pop(next(iter(_KA_HARD)))
    _KA_HARD[key]=(capH,viaH)
    return capH,viaH


def _ka_raw_distance(points,geoms):
    return _pz_distance(shapely.STRtree(geoms),points)


def _ka_cells(u,r):
    if u in _KA_CELLS:return _KA_CELLS[u]
    groups={}
    for t in r['tracks']:
        li=LI[t['layer']];n=t.get('net') or r['net']
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            groups.setdefault((li,n,'Track'),[]).append(seg_cells(li,a,b,w)-li*NN)
    for v in r['vias']:
        n=v.get('net') or r['net']
        for L in v.get('layers',LAYERS):
            if L not in LI:continue
            li=LI[L]
            groups.setdefault((li,n,'Via'),[]).append(seg_cells(li,(v['x'],v['y']),(v['x'],v['y']),float(v.get('d',VIA_D)))-li*NN)
    result={k:np.unique(np.concatenate(v)) for k,v in groups.items()}
    _KA_CELLS[u]=result
    return result


def _ka_soft_maps(net,box=None):



    rng=_box_idx(box);x0,x1,y0,y1=rng
    pad=int(math.ceil((max(KA.maximum,CLR)+1.5)/RES))+2
    jx0,jx1=max(0,x0-pad),min(NX,x1+pad);jy0,jy1=max(0,y0-pad),min(NY,y1+pad)
    cap=[]
    for li in range(NL):
        groups={}
        for u,r in U.items():
            for (l,n,t),cells in _ka_cells(u,r).items():
                if l!=li or n==net:continue
                c=max(_ka_clr(net,n,li,'Track',t),_ka_clr(net,n,li,'Via',t))+.005
                mask=groups.setdefault(c,np.zeros((jy1-jy0,jx1-jx0),dtype=bool))
                yy=cells//NX;xx=cells%NX
                inside=(yy>=jy0)&(yy<jy1)&(xx>=jx0)&(xx<jx1)
                mask[yy[inside]-jy0,xx[inside]-jx0]=True
        room=np.full((jy1-jy0,jx1-jx0),9.,dtype=np.float32)
        for c,mask in sorted(groups.items()):
            if mask.any():room=np.minimum(room,_edt(~mask)*RES-c)
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[y0:y1,x0:x1]=2*room[y0-jy0:y1-jy0,x0-jx0:x1-jx0];cap.append(a)
    fd=(drill_use>0).reshape(NY,NX)
    dd=(_edt(~fd)*RES).astype(np.float32) if fd.any() else np.full(fd.shape,9.,dtype=np.float32)
    return cap,dd,rng


def _ka_room(net,li,a,b,fixed=True,ohne=None):
    seg=_LS([a,b]) if a!=b else _PT(a)
    margin=PZ_RAND_NM/1e6 if _pz_active(net) else PAD_EXTRA
    room=min((seg.distance(g)-_ka_clr(net,n,li,'Track',t,'',p)-margin
              for n,g,t,p in _ka_obstacles(net,li,fixed,ohne)),default=9.)
    board=globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg):return -1.
        room=min(room,seg.distance(board.boundary)-float(args.get('edge','.30'))-margin)
    return room

def _ka_via_ok(net,x,y,fixed=True,ohne=None):
    pt=_PT(x,y);margin=PZ_RAND_NM/1e6
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+margin):return False
    if any(pt.distance(g)<VIA_R+margin for g in _VIA_KEEP):return False
    for li in range(NL):
        for n,g,t,p in _ka_obstacles(net,li,fixed,ohne):
            limit=VIA_R if t in ('MaskKeepout','HoleKeepout') else max(VIA_R+_ka_clr(net,n,li,'Via',t,'',p),VIA_DR/2+LOCH_CLR)
            if pt.distance(g)<limit+margin:return False
        if li in (0,NL-1) and not VIP:
            if any(n==net and pt.distance(g)<VIA_R+VIA_STEG for n,g in _PADG[li]):return False

    return _ka_legacy_via_ok(net,x,y,fixed,ohne)


def soft_maps(net, box=None, wmax=None):
    ""
    if globals().get('KA') is not None: return _ka_soft_maps(net,box)
    if NAH and wmax is not None and wmax <= NAH_W:
        key = '' if NETU.get(net, 0) == 0 else (net if net in SN else None)
        if key is not None:
            if key and (SN_OFFEN[key] is None or SN_OFFEN[key]): _nah_flush(key)
            capS = _Karten(SN[key][li * NN:(li + 1) * NN].reshape(NY, NX) for li in range(NL)); capS.flach = SN[key]
            return capS, DDN.reshape(NY, NX), _box_idx(None)
    own = None if (NAH and NETU.get(net, 0) == 0) else own_counts(net)[0]
    ix0, ix1, iy0, iy1 = _box_idx(box)
    P = int(1.5 / RES) + 2
    jx0, jx1, jy0, jy1 = max(0, ix0 - P), min(NX, ix1 + P), max(0, iy0 - P), min(NY, iy1 + P)
    teil = box is not None and (jx0, jx1, jy0, jy1) != (0, NX, 0, NY)
    capS = []
    for li in range(NL):
        f = use[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]
        f = (f > 0) if own is None else ((f - own[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]) > 0)
        if f.any(): d = (_edt(~f) * RES).astype(np.float32)
        else: d = np.full(f.shape, 9.0, dtype=np.float32)
        if not teil: capS.append(2 * (d - CLR)); continue
        full = np.full((NY, NX), -1.0, dtype=np.float32)
        full[iy0:iy1, ix0:ix1] = 2 * (d[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0] - CLR)
        capS.append(full)
    fd = (drill_use > 0).reshape(NY, NX)[jy0:jy1, jx0:jx1]
    dd = (_edt(~fd) * RES).astype(np.float32) if fd.any() else np.full(fd.shape, 9.0, dtype=np.float32)
    if teil:
        full = np.zeros((NY, NX), dtype=np.float32); full[iy0:iy1, ix0:ix1] = dd[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0]; dd = full
    return capS, dd, _box_idx(box)


def _im_fenster(win, rng):
    ""
    ixa, ixb = max(1, int((win[0] - X0) / RES)), min(NX - 2, int((win[2] - X0) / RES))
    iya, iyb = max(1, int((win[1] - Y0) / RES)), min(NY - 2, int((win[3] - Y0) / RES))
    return rng[0] <= ixa and ixb < rng[1] and rng[2] <= iya and iyb < rng[3]


ERREICH = args.get('erreichbar', '1') == '1'
_HM = {}
_HM_MAX = max(4, int(float(args.get('hm_mb', '1500')) * 1e6 / (NL * NN * 4 + NN)))


GROSS = args.get('gross', 'auto') == '1' or (args.get('gross', 'auto') == 'auto' and len(nets) > _HM_MAX)
_VOLL = (0, NX, 0, NY)
if GROSS:
    _B_CAPH = np.empty(NL * NN, dtype=np.float32); _B_CAPC = np.empty(NL * NN, dtype=np.float32)
    _B_VIAH = np.zeros(NN, dtype=bool); _B_VBAD = np.zeros(NN, dtype=bool)
    print('grosse Platine: Karten je Einheit nur im Fenster (%d Netze, Platz fuer %d volle Karten)' % (len(nets), _HM_MAX))


_FIX_KARTEN = {}


def _fix_karte(net):
    ""
    if FIXD[0] is None or net not in FIXNETS: return FIXD
    if net not in _FIX_KARTEN:
        fo = np.zeros(NL * NN, dtype=bool)
        for u, r in U.items():
            if u.startswith('FIX#') and r['net'] != net: fo[r['cells']] = True
        _FIX_KARTEN[net] = [[(_edt(~fo[i * NN:(i + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for i in range(NL)], FIXD[1]]
    return _FIX_KARTEN[net]


def _pad_loch_frei(abst):
    ""
    return abst >= VIA_DR/2+H2H+PAD_LOCH_RESERVE




_PZ_FIXED = {}; _PZ_HARD = {}; _PZ_DYNAMIC = {}; _PZ_UNITS = {}; _PZ_VERSION = [0,0]
_PZ_BUDGET_ACTIVE = [False]
_PZ_PARTIAL = [None]
_PZ_BASE = [False]


class _PZBudget(RuntimeError):
    pass


def _pz_check():
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if _PZ_BUDGET_ACTIVE[0] and zeit_knapp(.95):raise _PZBudget('Pinzugang-Nachlauf: CPU-Budget')


def _pz_index(net, fixed=True, ohne=None):
    from shapely.strtree import STRtree
    key=(net,_PZ_VERSION[0 if fixed else 1],ohne)
    cache=_PZ_FIXED if fixed else _PZ_DYNAMIC
    if key in cache:return cache[key]
    units = [(u,r) for u,r in U.items() if u != ohne and (not fixed or u.startswith('FIX#'))]
    layers = [[] for _ in LAYERS]; holes = []
    for u,r in units:
        _pz_check()
        if u not in _PZ_UNITS:
            copper=[]; drills=[]
            for t in r['tracks']:
                if t['layer'] not in LI:continue
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    center=_LS([a,b]) if a!=b else _PT(a)
                    copper.append((r['net'],LI[t['layer']],center.buffer(w/2/math.cos(math.pi/256),64)))
            for v in r['vias']:
                radius=float(v.get('d',VIA_D))/2;drill=float(v.get('drill',VIA_DR))/2
                point=_PT(v['x'],v['y']);n=v.get('net') or r['net']
                drills.append((point,drill,n,radius))
                geom=point.buffer(max(radius,drill+LOCH_CLR-DRCC)/math.cos(math.pi/256),64)
                for L in v.get('layers',LAYERS):
                    if L in LI:copper.append((n,LI[L],geom))
            _PZ_UNITS[u]=(copper,drills)
        copper,drills=_PZ_UNITS[u];holes.extend(drills)
        for n,li,g in copper:
            if n!=net:layers[li].append(g)
    trees = [STRtree(g) for g in layers]
    hole_tree=STRtree([pt.buffer(drill/math.cos(math.pi/256),64) for pt,drill,_,_ in holes])
    result = (layers, trees, holes,hole_tree)
    if len(cache)>=4:cache.pop(next(iter(cache)))
    cache[key]=result
    return result


def _pz_distance(tree, points):
    d = np.full(len(points),9.,dtype=np.float64)
    if len(tree.geometries):
        for offset in range(0,len(points),16384):
            _pz_check()
            ij, ds = tree.query_nearest(points[offset:offset+16384], return_distance=True, all_matches=False)
            d[offset+ij[0]] = ds
    return d


def _pz_hard_maps(net, rng):
    ""
    rng = tuple(rng) if GROSS and rng is not None else _VOLL
    key = (net,rng,_PZ_VERSION[0])
    if key in _PZ_HARD: return _PZ_HARD[key]
    _PZ_BASE[0] = True
    try: base, viaH = hard_maps(net,rng)
    finally: _PZ_BASE[0] = False
    capH = _Karten(); capH.flach = np.concatenate([a.ravel().copy() for a in base])
    capH.extend([capH.flach[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    viaH = viaH.copy()
    _, trees, holes, hole_tree = _pz_index(net)
    margin = PZ_RAND_NM / 1e6
    for p in D['pads']:
        if id(p) not in PZ_LOKAL[net]: continue
        bx=p['bbox'];r=PZ_RADIUS_NM/1e6
        x0=max(rng[0],int((bx[0]-r-X0)/RES));x1=min(rng[1],int((bx[2]+r-X0)/RES)+1)
        y0=max(rng[2],int((bx[1]-r-Y0)/RES));y1=min(rng[3],int((bx[3]+r-Y0)/RES)+1)
        if x0>=x1 or y0>=y1:continue
        ys=slice(y0,y1);xs=slice(x0,x1)
        xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
        via=(edge_d[ys,xs]-EDGE>=VIA_R+margin)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
        if VIA_VERBOT is not None:via &= ~VIA_VERBOT[ys,xs]
        via &= _pz_distance(hole_tree,pts).reshape(xx.shape)>=VIA_DR/2+H2H+margin
        for li in range(NL):
            dp=np.where(PN1[li][ys,xs]==NID[net],PD2[li][ys,xs],PD1[li][ys,xs])
            fixed=_pz_distance(trees[li],pts).reshape(xx.shape)
            ch=np.minimum(2*(dp-DRCP-margin),2*(edge_d[ys,xs]-EDGE-margin))
            ch=np.minimum(ch,2*(fixed-DRCC-margin))
            capH[li][ys,xs]=_zp_hart(net,ch,li,ys,xs)
            via &= (dp>=max(VIA_R+DRCP,VIA_DR/2+LOCH_CLR)+margin)&(fixed>=VIA_R+DRCC+margin)
            if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_STEG
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):via &= PLANE_MASK[net][ys,xs]
        via=_zp_viah(net,via,ys,xs)
        if PAD_LOCH_ABST is not None:via &= _pad_loch_frei(PAD_LOCH_ABST[ys,xs])
        viaH[ys,xs]=via
    if VP:_vp_ein(net,viaH,rng)
    plane_trace_maps(net,capH,rng)
    while len(_PZ_HARD)>=max(1,min(3,_HM_MAX)):_PZ_HARD.pop(next(iter(_PZ_HARD)))
    _PZ_HARD[key]=(capH,viaH)
    return capH,viaH


def _pz_room(net,li,a,b,fixed=True,ohne=None):
    if globals().get('KA') is not None: return _ka_room(net,li,a,b,fixed,ohne)
    seg=_LS([a,b]) if a!=b else _PT(a)
    _, trees, _, _ = _pz_index(net,fixed,ohne)
    best=9.
    tree=trees[li]
    if len(tree.geometries):
        j=tree.nearest(seg)
        best=min(best,seg.distance(tree.geometries[j])-DRCC-(PZ_RAND_NM/1e6))

    board = globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg): return -1.
        best=min(best,seg.distance(board.boundary)-float(args.get('edge','.30'))-(PZ_RAND_NM/1e6))
    return best


def _ka_legacy_via_ok(net,x,y,fixed=True,ohne=None):
    pt=_PT(x,y); _,trees,holes,hole_tree=_pz_index(net,fixed,ohne)
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+(PZ_RAND_NM/1e6)):return False
    if any(pt.distance(g)<VIA_R+(PZ_RAND_NM/1e6) for g in _VIA_KEEP):return False
    for li in range(NL):
        for j in _PADT[li].query(pt.buffer(VIA_R+max(DRCP,LOCH_CLR)+.01)):
            n,g=_PADG[li][int(j)];d=pt.distance(g)
            if globals().get('KA') is None and n!=net and d<max(VIA_R+DRCP,VIA_DR/2+LOCH_CLR)+(PZ_RAND_NM/1e6):return False
            if n==net and li in (0,NL-1) and not VIP and d<VIA_R+VIA_STEG:return False
        for j in ([] if globals().get('KA') is not None else trees[li].query(pt.buffer(max(VIA_R+DRCC,VIA_DR/2+LOCH_CLR)+.01))):
            if pt.distance(trees[li].geometries[j])<max(VIA_R+DRCC,VIA_DR/2+LOCH_CLR)+(PZ_RAND_NM/1e6):return False
        for (n,L),g in _ZP_GEO.items():
            if globals().get('KA') is None and n!=net and L==LAYERS[li] and pt.distance(g)<VIA_R+CLR+.005:return False
    if any(pt.distance(g)<VIA_DR/2+H2H+(PZ_RAND_NM/1e6) for g in PAD_LOCH_GEOMS):return False
    for j in hole_tree.query(pt.buffer(VIA_DR/2+H2H+(PZ_RAND_NM/1e6))):
        p,drill,_,_=holes[int(j)]
        if pt.distance(p)<VIA_DR/2+drill+H2H+(PZ_RAND_NM/1e6):return False
    for vs in D.get('via_sperre',[]):
        g=shapely.union_all([Polygon(q) for q in vs['polys']])
        own=any(p['ref']==vs['ref'] and p['net']==net and any(Polygon(q).covers(pt) for pp in p['polys'].values() for q in pp) for p in D['pads'])
        if not own and pt.distance(g)<VIA_DR/2+H2H+(PZ_RAND_NM/1e6):return False
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
        ix=int((x-X0)/RES);iy=int((y-Y0)/RES)
        if not (0<=ix<NX and 0<=iy<NY and PLANE_MASK[net][iy,ix]):return False
        if PINZUGANG and not PLANE_AREA[net].covers(pt):return False
    return True


def _pz_via_ok(net,x,y,fixed=True,ohne=None):
    if globals().get('KA') is not None: return _ka_via_ok(net,x,y,fixed,ohne)
    return _ka_legacy_via_ok(net,x,y,fixed,ohne)


def _pz_path_points(path,net):
    pts=[cell_xy((n%NN)%NX,(n%NN)//NX) for n in path]
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if a//NN!=b//NN:
            p=_vp_xy(net,a%NN) if VP else None
            if p:pts[i]=pts[i+1]=p
    return [_pz_snap(p) for p in pts]


def _pz_grid_geometry(path,net,w):
    ""
    points=_pz_path_points(path,net);tr=[];vi=[];run=[points[0]];li=path[0]//NN
    def finish():
        if len(run)>1:

            ps=[run[0]]
            for j,p in enumerate(run[1:]):
                if len(ps)>1 and abs((ps[-1][0]-ps[-2][0])*(p[1]-ps[-1][1])-(ps[-1][1]-ps[-2][1])*(p[0]-ps[-1][0]))<1e-12:ps[-1]=p
                else:ps.append(p)
            tr.append(dict(layer=LAYERS[li],pts=[list(p) for p in ps],w=[w]*(len(ps)-1)))
    for i,n in enumerate(path[1:],1):
        if n//NN!=li:
            finish();vi.append(dict(x=points[i][0],y=points[i][1]));li=n//NN;run=[points[i]]
        else:run.append(points[i])
    finish();return tr,vi


def _pz_edge_retry(res,src,tgt,bx,capH,capS,viaH,viaBad,C,win,extra):
    ""
    net=NET_CUR[0];w=PZ_BREITEN[net]['w'];protected=set(src)|set(tgt)
    ch=_Karten();ch.flach=np.concatenate([a.ravel() for a in capH])
    ch.extend([ch.flach[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    vh=viaH.copy()
    for trial in range(5):
        _pz_check()
        if not res:return None
        tr,vi=_pz_grid_geometry(res,net,w)
        if _pz_geometry_ok(net,tr,vi,not HART_FREMD[0]):
            PZ_EDGE_STAT['angenommen']+=1
            return res,tr,vi
        if trial==4 or ('zeit_knapp' in globals() and zeit_knapp(.95)):break
        points=_pz_path_points(res,net);blocked=False
        for i,(a,b) in enumerate(zip(res[:-1],res[1:])):
            p,q=points[i],points[i+1]
            if a//NN!=b//NN:
                if not _pz_via_ok(net,*p,not HART_FREMD[0]):vh.flat[a%NN]=False;blocked=True;break
            elif min(pad_room(a//NN,p,q,net),_pz_room(net,a//NN,p,q,not HART_FREMD[0]))<w/2-1e-9:
                candidates=[n for n in (b,a) if n not in protected]
                if candidates:
                    n=min(candidates,key=lambda n:float(ch.flach[n]));ch.flach[n]=-1.;blocked=True;break
        if not blocked:

            for i,v in enumerate(vi):
                if any(math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_RAND_NM/1e6) for q in vi[:i]):
                    c=int((v['y']-Y0)/RES)*NX+int((v['x']-X0)/RES);vh.flat[c]=False;blocked=True;break
        if not blocked:break
        PZ_EDGE_STAT['wiederholungen']+=1
        res=astar_pf(src,tgt,bx,ch,capS,vh,viaBad,dict(C,w=w,wmin=w),win,extra=extra)
    PZ_EDGE_STAT['verworfen']+=1;return None


PZ_EDGE_STAT = dict(angenommen=0,wiederholungen=0,verworfen=0)


def _pz_geometry_ok(net,tracks,vias,fixed=True,ohne=None):


    if globals().get('PINZUGANG', False) or globals().get('KA') is not None:
        for t in tracks:
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
        for v in vias:
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
    minimum=(globals().get('PZ_BREITEN',{}).get(net) or {}).get('dru_min')
    if minimum is not None and any(w<minimum-1e-9 for t in tracks for w in t['w']):return False
    for t in tracks:
        li=LI[t['layer']]
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            if min(pad_room(li,a,b,net),_pz_room(net,li,a,b,fixed,ohne))<w/2-1e-9:return False
    for i,v in enumerate(vias):
        if any(.02<math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_RAND_NM/1e6) for q in vias[:i]):return False
        if not _pz_via_ok(v.get('net') or net,v['x'],v['y'],fixed,ohne):return False
    return True


def hard_maps(net, rng=None):
    ""
    if globals().get('KA') is not None: return _ka_hard_maps(net,rng)
    if _pz_active(net) and not _PZ_BASE[0]: return _pz_hard_maps(net, rng)
    if net in _HM:
        r = _HM.pop(net); _HM[net] = r; return r
    nid = NID[net]
    fx0, fx1 = _fix_karte(net)
    if GROSS and rng is not None and tuple(rng) != _VOLL:
        ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
        capH = _Karten(); capH.flach = _B_CAPH
        v = (edge_d[ys, xs] - EDGE >= VIA_R) & (VSP[ys, xs] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_VERBOT is not None: v &= ~VIA_VERBOT[ys, xs]
        if fx1 is not None: v &= fx1[ys, xs] - QM >= HOLE_C
        for li in range(NL):
            dp = np.where(PN1[li][ys, xs] == nid, PD2[li][ys, xs], PD1[li][ys, xs])
            ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d[ys, xs] - EDGE))
            if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li][ys, xs] - QM - CLR - 0.005))
            ch = _zp_hart(net,ch,li,ys,xs)
            ch = ch.astype(np.float32)
            lage = _B_CAPH[li * NN:(li + 1) * NN].reshape(NY, NX); lage[ys, xs] = ch; capH.append(lage)
            v &= ch / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and not VIP: v &= padd[li][ys, xs] >= VIA_R + VIA_STEG
        viaH = _B_VIAH.reshape(NY, NX); viaH[ys, xs] = v
        if VP: _vp_ein(net, viaH, rng)
        if EPZ: _ep_ein(net, viaH, capH, rng)
        if GS_N: _gassen_ein(net, capH, rng)
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
        if ZONENPLAN: viaH[ys,xs] = _zp_viah(net,viaH[ys,xs],ys,xs)
        if PAD_LOCH_ABST is not None: viaH[ys,xs] &= _pad_loch_frei(PAD_LOCH_ABST[ys,xs])
        plane_trace_maps(net,capH,rng)
        return capH, viaH
    capH = _Karten(); capH.flach = np.empty(NL * NN, dtype=np.float32)
    for li in range(NL):
        dp = np.where(PN1[li] == nid, PD2[li], PD1[li])
        ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d - EDGE))
        if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li] - QM - CLR - 0.005))
        ch = _zp_hart(net,ch,li)
        capH.flach[li * NN:(li + 1) * NN] = ch.astype(np.float32).ravel()
        capH.append(capH.flach[li * NN:(li + 1) * NN].reshape(NY, NX))
    viaH = (edge_d - EDGE >= VIA_R) & (VSP >= H2H + VIA_DR / 2 + 0.01)
    if VIA_VERBOT is not None: viaH &= ~VIA_VERBOT
    if fx1 is not None: viaH &= fx1 - QM >= HOLE_C
    for li in range(NL):
        viaH &= capH[li] / 2 >= VIA_KEEP_R
        if li in (0, NL - 1) and not VIP: viaH &= padd[li] >= VIA_R + VIA_STEG
    if VP: _vp_ein(net, viaH, None)
    if EPZ: _ep_ein(net, viaH, capH, None)
    if GS_N: _gassen_ein(net, capH, None)
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH &= PLANE_MASK[net]
    viaH = _zp_viah(net,viaH)
    if PAD_LOCH_ABST is not None: viaH &= _pad_loch_frei(PAD_LOCH_ABST)
    plane_trace_maps(net,capH)
    while len(_HM) >= _HM_MAX: del _HM[next(iter(_HM))]
    _HM[net] = (capH, viaH)
    return capH, viaH


def _pz_soft_maps(net,rng):
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
    _,trees,holes,hole_tree=_pz_index(net,False)
    cap=[]
    for tree in trees:
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[ys,xs]=2*(_pz_distance(tree,pts).reshape(xx.shape)-DRCC-1e-6)
        cap.append(a)
    dd=np.full((NY,NX),9.,dtype=np.float32)

    dd[ys,xs]=_pz_distance(hole_tree,pts).reshape(xx.shape)+VIA_DR/2+.02-1e-6
    return cap,dd


def karten(net, box, wmax):
    ""
    capS, dd, rng = soft_maps(net, box, wmax)
    if GROSS and box is not None and tuple(rng) == _VOLL: rng = _box_idx(box)
    capH, viaH = hard_maps(net, rng)
    if globals().get('KA') is None and _pz_active(net) and (FIXD[0] is not None or HART_FREMD[0]):
        if all(u.startswith('FIX#') or r['net']==net for u,r in U.items()):
            capS=capH


            dd=np.full((NY,NX),9.,dtype=np.float32)
        else:capS,dd=_pz_soft_maps(net,rng)
    capH, viaH = strom_reservieren(net, capH, viaH)
    if not GROSS or tuple(rng) == _VOLL:
        viaBad = (dd < HOLE_C)
        for li in range(NL): viaBad |= capS[li] / 2 < VIA_R
        capC = [np.minimum(capH[i], capS[i]) for i in range(NL)]
        capCf = np.concatenate([c.ravel() for c in capC])
        if HART_FREMD[0]:
            capH = _Karten(); capH.flach = capCf
            capH.extend([capCf[i * NN:(i + 1) * NN].reshape(NY, NX) for i in range(NL)])
            viaH = viaH & ~viaBad
        return capH, viaH, capS, dd, rng, viaBad, capC, capCf, viaBad.ravel()
    ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
    v = dd[ys, xs] < HOLE_C
    capC = _Karten(); capC.flach = _B_CAPC
    for li in range(NL):
        v |= capS[li][ys, xs] / 2 < VIA_R
        lage = _B_CAPC[li * NN:(li + 1) * NN].reshape(NY, NX); lage[ys, xs] = np.minimum(capH[li][ys, xs], capS[li][ys, xs]); capC.append(lage)
    viaBad = _B_VBAD.reshape(NY, NX); viaBad[ys, xs] = v
    return (capC if HART_FREMD[0] else capH), (viaH & ~viaBad if HART_FREMD[0] else viaH), capS, dd, rng, viaBad, capC, _B_CAPC, _B_VBAD


def karten_groesser(net, box, win, wmax):
    ""
    if not GROSS: return karten(net, None, None), None
    b2 = (min(box[0], win[0]) - 1.0, min(box[1], win[1]) - 1.0, max(box[2], win[2]) + 1.0, max(box[3], win[3]) + 1.0)
    return karten(net, b2, wmax), b2


_ERR = {}
_ERR_MAX = max(2, int(float(args.get('err_mb', '600')) * 1e6 / (NL * NN * 4)))
UNERREICHBAR = {}
_ERR_NETZE = set()


def _komponenten(net, wmin):
    ""
    key = (net, wmin)
    if key in _ERR:
        K = _ERR.pop(key); _ERR[key] = K; return K
    capH, viaH = hard_maps(net)
    from scipy.ndimage import label as _label
    acht = np.ones((3, 3), dtype=bool)
    K = np.zeros(NL * NN, dtype=np.int32); n_ges = 0
    for li in range(NL):
        lab, n = _label((capH[li] >= np.float64(wmin)) | viaH, structure=acht)
        lab = lab.astype(np.int32); lab[lab > 0] += n_ges; n_ges += n
        K[li * NN:(li + 1) * NN] = lab.ravel()
    eltern = list(range(n_ges + 1))
    def wurzel(a):
        while eltern[a] != a:
            eltern[a] = eltern[eltern[a]]; a = eltern[a]
        return a
    vidx = np.flatnonzero(viaH.ravel())
    if len(vidx):
        k0 = K[vidx]
        for li in range(1, NL):
            kl = K[li * NN + vidx]
            neu_ = np.ones(len(vidx), dtype=bool); neu_[1:] = (k0[1:] != k0[:-1]) | (kl[1:] != kl[:-1])
            for a, b in np.unique(np.stack([k0[neu_], kl[neu_]], axis=1), axis=0).tolist():
                ra, rb = wurzel(a), wurzel(b)
                if ra != rb: eltern[max(ra, rb)] = min(ra, rb)
    K = np.array([wurzel(i) for i in range(n_ges + 1)], dtype=np.int32)[K]
    while len(_ERR) >= _ERR_MAX: del _ERR[next(iter(_ERR))]
    _ERR[key] = K
    return K


def _unerreichbar(K, src, tgt):
    ""
    def umfeld(a):
        li = a // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        out = [a]
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                jy = iy + dy; jx = ix + dx; m = (jy >= 0) & (jy < NY) & (jx >= 0) & (jx < NX)
                out.append((li * NN + jy * NX + jx)[m])
        return np.concatenate(out)
    s = umfeld(np.asarray(src, dtype=np.int64)); t = np.fromiter(tgt, dtype=np.int64, count=len(tgt))
    if np.intersect1d(s, t).size: return False
    ks = np.unique(K[s]); kt = np.unique(K[umfeld(t)])
    return not np.intersect1d(ks[ks > 0], kt[kt > 0]).size


def hals_aktiv(net, C, wmin, stich=False):
    ""
    if wmin <= max(HALS_W, DRU_MIN.get(net, 0.)) or net not in bynet_g: return False
    feld = HALS_FELD and any(net in f[5] for f in DICHT)
    if C.get('prio', 8) < 8: return HALS > 0 or feld
    return (stich or BGA_ENG) and feld


def hals_feld_karte(net, C, wmin, stich=False):
    if not hals_aktiv(net, C, wmin, stich): return None
    if (net, wmin) in HALS_CACHE:
        WMINF = HALS_CACHE.pop((net, wmin)); HALS_CACHE[(net, wmin)] = WMINF; return WMINF

    while len(HALS_CACHE) >= _HALS_MAX: del HALS_CACHE[next(iter(HALS_CACHE))]
    WMINF = np.full(NL * NN, wmin, dtype=np.float32); HALS_CACHE[(net, wmin)] = WMINF
    hw = max(HALS_W, DRU_MIN.get(net, 0.))
    if HALS > 0 and C.get('prio', 8) < 8:
        m = np.zeros(NL * NN, dtype=bool)
        for k in bynet_g[net]:
            for L, cells in D['pads'][k]['cells'].items(): m[LI[L] * NN + cells] = True
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any():
                near = _edt(~m[sl].reshape(NY, NX)) * RES <= HALS
                WMINF[sl][near.ravel()] = hw
    if HALS_FELD:
        for lg, ix0, ix1, iy0, iy1, netze in DICHT:
            if net not in netze: continue
            for li in lg: WMINF[li * NN:(li + 1) * NN].reshape(NY, NX)[iy0:iy1, ix0:ix1] = hw
    return WMINF



def _knick_h(n, nn, nx, x0, y0, res, tx0, ty0, tx1, ty1, knick=0.):


    idx = n % nn
    x = x0 + (idx % nx + .5) * res; y = y0 + (idx // nx + .5) * res
    dx = max(tx0-x, 0., x-tx1); dy = max(ty0-y, 0., y-ty1)
    return max(dx, dy) + (1.4142 - 1.) * min(dx, dy)

def knick_kern(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win,
               NX, NN, NL, X0, Y0, RES, g, par, gst, cst, tst, run, ofx, ofy, ofs, wminf, hfac, info, viaz, vzk,
               knick, vzu, vpr, vank, vtbox):
    ixa, ixb, iya, iyb = win[0], win[1], win[2], win[3]
    tx0, ty0, tx1, ty1 = tbox[0], tbox[1], tbox[2], tbox[3]
    hfac = min(1.0, hfac) * (1.4142 / math.sqrt(2.0))
    for t in tgt: tst[t] = run
    kap = max(65536, 2 * src.shape[0] + 64)
    hf = np.empty(kap, dtype=np.float64); hn = np.empty(kap, dtype=np.int64); hs = 0
    for s0 in src:
        s = s0 * 9 + 8
        gst[s] = run; g[s] = 0.0; par[s] = -1
        f0 = hfac * _knick_h(s // 9, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, knick)
        hs = _hpush(hf, hn, hs, f0, np.int64(s))
    while hs > 0:
        n = hn[0]
        hs = _hpop(hf, hn, hs)
        if hs + NL + 12 > kap:
            kap *= 2
            hf2 = np.empty(kap, dtype=np.float64); hn2 = np.empty(kap, dtype=np.int64)
            hf2[:hs] = hf[:hs]; hn2[:hs] = hn[:hs]; hf = hf2; hn = hn2
        if n < 0:
            n = -n - 1
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        if cst[n] == run: continue
        if info.shape[0] > 1: info[1] += 1
        node = n // 9; direction = n % 9


        dominated = False
        for d in range(9):
            other = node * 9 + d
            if other != n and gst[other] == run and (g[other] + (0.0 if d == 8 else knick) < g[n]):
                dominated = True; break
        if dominated:
            if info.shape[0] > 2: info[2] += 1
            continue
        cst[n] = run
        idx = node % NN
        li = node // NN
        if via_target:
            if tst[node] == run:
                L = 0; q = n
                while q != -1: L += 1; q = par[q]
                out = np.empty(L, dtype=np.int64); q = n; k = L - 1
                while q != -1: out[k] = q // 9; k -= 1; q = par[q]
                return out
            if viaf[idx] and (li == 0 or li == NL - 1):
                idx0 = node % NN
                xx = X0 + (idx0 % NX + 0.5) * RES; yy = Y0 + (idx0 // NX + 0.5) * RES
                dx0 = max(vtbox[0]-xx, 0.0, xx-vtbox[2]); dy0 = max(vtbox[1]-yy, 0.0, yy-vtbox[3])
                hv = math.sqrt(dx0*dx0+dy0*dy0)

                hs = _hpush(hf, hn, hs, g[n] + 0.3 + 2.0 * hv + (3.0 * PFv if vbad[idx] else 0.0)
                            + (vzk * np.float64(viaz[idx]) if viaz.shape[0] > 0 else 0.0), np.int64(-n - 1))
        elif tst[node] == run:
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        iy = idx // NX; ix = idx - iy * NX
        gn = g[n]

        for k in range(8):
            jx = ix + ofx[k]; jy = iy + ofy[k]
            if jx < ixa or jx > ixb or jy < iya or jy > iyb:
                info[0] += 1
                continue
            nextnode = li * NN + jy * NX + jx
            m = nextnode * 9 + k
            wm = wmin if wminf.shape[0] == 0 else wminf[nextnode]
            if capHf[nextnode] < wm and tst[nextnode] != run: continue
            ng = gn + ofs[k] * np.float64(mult[nextnode])
            if direction != 8 and direction != k: ng += knick
            if gst[m] != run or ng < g[m]:
                gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                hm = hfac * _knick_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, knick)
                hs = _hpush(hf, hn, hs, ng + hm, np.int64(m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PFv * vbad[idx]) + 0.2 * np.float64(hist[node])
            if vank.shape[0] > 0: vc = vc + np.float64(vank[idx])
            for lj in range(NL):
                if lj == li: continue
                nextnode = lj * NN + idx
                m = nextnode * 9 + 8
                vcl = vc
                if vzu.shape[0] > 0: vcl = vc + vpr[li * NL + lj] * np.float64(vzu[idx])
                ng = gn + vcl
                if gst[m] != run or ng < g[m]:
                    gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                    hs = _hpush(hf, hn, hs, ng + hfac * _knick_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, knick), np.int64(m))
    return np.empty(0, dtype=np.int64)



_KNICK_SUCHE = None
_KNICK_CPU = 0.; _KNICK_CALLS = 0; _KNICK_POP = 0; _KNICK_DOM = 0
if NB is not None:
    from router.astar_nb import _hpush, _hpop
    import numba as _knick_numba
    _knick_h = _knick_numba.njit(cache=True, nogil=True)(_knick_h)
    knick_kern = _knick_numba.njit(cache=True, nogil=True)(knick_kern)

def knick_suche(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                 win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                 knick=0., vzu=None, vpr=None, vank=None):
    global _KNICK_SUCHE, _KNICK_CPU, _KNICK_CALLS, _KNICK_POP, _KNICK_DOM
    if _KNICK_SUCHE is None:
        _KNICK_SUCHE = _Suche(nl, nn, res, richtung=True)
        _KNICK_SUCHE.info = np.zeros(3, np.int64)
    search = _KNICK_SUCHE
    xa, xb, ya, yb = win
    vtbox = tuple(tbox)
    hfac = min(float(hfac), max(0., float(mult.reshape(nl, nn//nx, nx)[:,ya:yb+1,xa:xb+1].min())))
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tbox = (min(tbox[0], min(x for x,y in points)), min(tbox[1], min(y for x,y in points)),
                max(tbox[2], max(x for x,y in points)), max(tbox[3], max(y for x,y in points)))
    search.run += 1; search.info[:] = 0
    if search.run >= 2**31-2:
        search.gst[:] = 0; search.cst[:] = 0; search.tst[:] = 0; search.run = 1
    from router.astar_nb import OFX, OFY
    start = time.process_time()
    path = knick_kern(np.asarray(src, np.int64), np.asarray(sorted(tgt), np.int64), np.asarray(tbox, np.float64),
        cap, mult, via, vbad, hist, float(wmin), float(vc), float(pf), bool(via_target), np.asarray(win,np.int64),
        nx,nn,nl,float(x0),float(y0),float(res),search.g,search.par,search.gst,search.cst,search.tst,np.int32(search.run),
        OFX,OFY,search.ofs,np.asarray(wminf,np.float32),float(hfac),search.info,
        _LEER if viaz is None else viaz,float(vzk),float(knick),_LEER if vzu is None else vzu,
        np.zeros(1,np.float64) if vpr is None else vpr,_LEER if vank is None else vank,np.asarray(vtbox,np.float64))
    _KNICK_CPU += time.process_time()-start; _KNICK_CALLS += 1
    _KNICK_POP += int(search.info[1]); _KNICK_DOM += int(search.info[2])
    NB.info[0] = search.info[0]
    return path.tolist() if len(path) else None

def astar_pf(src, tgt_set, tgt_box, capH, capS, viaH, viaBad, C, window, via_target=False, extra=None, viaz=None, vzk=0.0):
    arbeit_schritt('astar')
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    x0, y0, x1, y1 = window
    ixa, ixb = max(1, int((x0 - X0) / RES)), min(NX - 2, int((x1 - X0) / RES))
    iya, iyb = max(1, int((y0 - Y0) / RES)), min(NY - 2, int((y1 - Y0) / RES))
    w, wmin = C['w'], C['wmin']

    art = (KL.get('netze', {}).get(NET_CUR[0], {}).get('art') if KL is not None else None)
    turn = KNICK * (KNICK_STROM if art in ('strom','versorgung','sw','gnd') else 1.)
    WMINF = hals_feld_karte(NET_CUR[0], C, wmin, via_target)
    if C.get('strom_w'): WMINF = strom_boden(NET_CUR[0], C)
    if C.get('zw'):
        WMINF = np.repeat(np.asarray(C['zw'], dtype=np.float32), NN)
    capHf = capH.flach if getattr(capH, 'flach', None) is not None else np.concatenate([c.ravel() for c in capH])
    if HART_FREMD[0] or (STROM_ZUERST and not strom_modell(NET_CUR[0])):


        def frei(n): return capHf[n] >= (wmin if WMINF is None else WMINF[n])
        src = [n for n in src if frei(n)]
        tgt_set = {n for n in tgt_set if frei(n)}
        if not src or (not via_target and not tgt_set): return None
    elif QUELLFREI[0]:
        def frei(n):
            wm = wmin if WMINF is None else WMINF[n]
            return min(capHf[n], capS[n // NN].ravel()[n % NN]) >= wm
        freie_src = [n for n in src if frei(n)]
        freie_tgt = {n for n in tgt_set if frei(n)}

        if freie_src: src = freie_src
        if freie_tgt: tgt_set = freie_tgt
    hw = C.get('histw', 1.0)


    mult = _MULT
    _key = (id(capH), id(capS), id(extra), w, wmin, C['inner'], C['pen'], hw, PF[0], C.get('lf'), C.get('zw'), C.get('strom_w'), C.get('strom_luft'))
    _c = ASTAR_COST[0]
    if not (_c is not None and _c[0] == _key and _c[1][0] <= iya and iyb <= _c[1][1] and _c[1][2] <= ixa and ixb <= _c[1][3]):
        fy0, fy1, fx0, fx1 = iya, iyb, ixa, ixb
        for _f in (ASTAR_HINT[0], _c[1] if _c is not None and _c[0] == _key else None):
            if _f is not None: fy0, fy1, fx0, fx1 = min(fy0, _f[0]), max(fy1, _f[1]), min(fx0, _f[2]), max(fx1, _f[3])
        lf = C.get('lf') or ([1.0] + [C['inner']] * (NL - 2) + [1.0])
        ys = slice(fy0, fy1 + 1); xs = slice(fx0, fx1 + 1)
        for li in range(NL):
            if C.get('zw'): w = wmin = C['zw'][li]
            if C.get('strom_w'): w = wmin = C['strom_w'][li]
            sl = slice(li * NN, (li + 1) * NN)
            c = np.minimum(capH[li][ys, xs], capS[li][ys, xs])
            m = (1.0 + C['pen'] * np.clip((w - c) / w, 0, 1)) * lf[li]


            _eng = float(args.get('kosten_eng', '0')) if KL is not None else 0.
            if _eng:
                m *= 1. + _eng * np.clip(1. - c / (2*w + .2), 0., 1.)
            if C.get('zw') and IMP_REF_MASK:
                _zp = IMP_TAB.get(NET_CUR[0], {}).get(LAYERS[li])
                if _zp:
                    _mask = IMP_REF_MASK[NET_CUR[0], LAYERS[li]][ys, xs]
                    m = np.where(_mask, m, m * 8.0)
            if FL_PREIS > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_LUECKE), m * (1.0 + FL_PREIS * (e - CLR)), m)
            elif FLAECHE > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_LUECKE), m * (1.0 + FLAECHE), m)
            if li == 0: m = np.where(under_esp[ys, xs], m * 1.8, m)
            m = (m * (1.0 + PF[0] * (c < wmin))).astype(np.float32)
            m *= (1.0 + hw * hist[sl].reshape(NY, NX)[ys, xs])
            if C.get('strom_luft') and STROM_UMWEG:
                ax,ay,bx,by = C['strom_luft']; gx,gy = np.meshgrid(GXc[xs],GYc[ys])
                luft = math.hypot(bx-ax,by-ay)
                mehr = np.maximum(0.,np.hypot(gx-ax,gy-ay)+np.hypot(gx-bx,gy-by)-luft)
                m *= (1. + STROM_UMWEG * mehr / max(1.,luft)).astype(np.float32)
            if extra is not None: m *= extra[sl].reshape(NY, NX)[ys, xs]
            mult[sl].reshape(NY, NX)[ys, xs] = m

        ASTAR_COST[0] = (_key, (fy0, fy1, fx0, fx1), capH, capS, extra)
    viaf = viaH.ravel(); vbad = viaBad.ravel()
    if C.get('strom_w'):
        viaf = strom_feld_maske(viaH, max(2, math.ceil(4*C['strom_I'])), C, capH).ravel()
    if VIA_MASKE[0] is not None: viaf = viaf & VIA_MASKE[0]
    VC = C['via'] * VIA_FAK[0]
    _vz = VIA_ZU[0] if not via_target else None
    vzu, vpr, vank = _vz if _vz is not None else (None, None, None)

    bounded_loop = globals().get('_LOOP_DEADLINE') is not None and not BUDGET_ARBEIT
    if NB is not None and not bounded_loop and turn > 0:
        return knick_suche(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
            (ixa,ixb,iya,iyb),NX,NN,NL,X0,Y0,RES,_LEER if WMINF is None else WMINF,HFAC,viaz,vzk,turn,vzu,vpr,vank)
    if NB is not None and not bounded_loop:
        return NB(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                  (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _LEER if WMINF is None else WMINF, HFAC, viaz, vzk,
                  turn, vzu, vpr, vank)
    if turn > 0:

        return astar_richtung_python(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                                (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _LEER if WMINF is None else WMINF,
                                HFAC, viaz, vzk, turn, vzu, vpr, vank)
    tx0, ty0, tx1, ty1 = tgt_box
    def h(n):
        idx = n % NN; ix = idx % NX; iy = idx // NX
        x, y = X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES
        dx = max(tx0 - x, 0, x - tx1); dy = max(ty0 - y, 0, y - ty1)
        return math.sqrt(dx * dx + dy * dy) if turn > 0.0 else math.hypot(dx, dy)
    g = {}; parent = {}; openh = []
    for n in src:
        g[n] = 0.0; parent[n] = -1; heapq.heappush(openh, (0.0 if via_target else HFAC * h(n), n))
    closed = set()
    loop_poll = 0
    while openh:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        f, n = heapq.heappop(openh)
        if n < 0:
            n = -n - 1; path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        if n in closed: continue
        closed.add(n)
        idx = n % NN
        if via_target:
            if n in tgt_set:
                path = []
                while n != -1: path.append(n); n = parent[n]
                return path[::-1]
            if viaf[idx] and n // NN in (0, NL - 1):
                heapq.heappush(openh, (g[n] + 0.3 + 2.0 * h(n) + (3.0 * PF[0] if vbad[idx] else 0.0)
                                       + (vzk * float(viaz[idx]) if viaz is not None else 0.0), -n - 1))
        elif n in tgt_set:
            path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        li = n // NN; iy, ix = divmod(idx, NX)
        gn = g[n]
        pdx = 0; pdy = 0; hat_richtung = False
        if turn > 0.0 and parent[n] >= 0 and parent[n] // NN == li:
            pq = parent[n] % NN
            pdx = ix - pq % NX; pdy = iy - pq // NX; hat_richtung = True
        for dx, dy, st in OFFS:
            jx, jy = ix + dx, iy + dy
            if jx < ixa or jx > ixb or jy < iya or jy > iyb: continue
            m = li * NN + jy * NX + jx
            if capHf[m] < (wmin if WMINF is None else WMINF[m]) and m not in tgt_set: continue
            ng = gn + st * float(mult[m])
            if hat_richtung and (dx != pdx or dy != pdy): ng += turn
            if ng < g.get(m, 1e18):
                g[m] = ng; parent[m] = n
                heapq.heappush(openh, (ng + (0.0 if via_target else HFAC * h(m)), m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PF[0] * vbad[idx]) + 0.2 * float(hist[n])
            if vank is not None: vc = vc + float(vank[idx])
            for lj in range(NL):
                if lj == li: continue
                m = lj * NN + idx
                vcl = vc
                if vzu is not None: vcl = vc + float(vpr[li * NL + lj]) * float(vzu[idx])
                ng = gn + vcl
                if ng < g.get(m, 1e18):
                    g[m] = ng; parent[m] = n
                    heapq.heappush(openh, (ng + HFAC * h(m), m))
    return None


def astar_richtung_python(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                          win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                          knick=0., vzu=None, vpr=None, vank=None):
    ""
    ny = nn // nx
    xa, xb, ya, yb = win
    tx0, ty0, tx1, ty1 = tbox
    vx0, vy0, vx1, vy1 = tbox
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tx0 = min(tx0, min(x for x, y in points)); tx1 = max(tx1, max(x for x, y in points))
        ty0 = min(ty0, min(y for x, y in points)); ty1 = max(ty1, max(y for x, y in points))
    hfac = min(1., float(hfac), max(0., float(mult.reshape(nl, ny, nx)[:, ya:yb+1, xa:xb+1].min()))) * (1.4142 / math.sqrt(2.))
    def h(node):
        idx = node % nn
        x, y = x0+(idx % nx+.5)*res, y0+(idx // nx+.5)*res
        dx, dy = max(tx0-x, 0., x-tx1), max(ty0-y, 0., y-ty1)
        return max(dx,dy)+(1.4142-1.)*min(dx,dy)
    directions = ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))
    g, parent, closed, queue = {}, {}, set(), []
    for node in src:
        s = node*9+8
        g[s] = 0.; parent[s] = -1
        heapq.heappush(queue, (hfac*h(node), s))
    def relax(s, cost, previous, heuristic):
        if cost < g.get(s, float('inf')):
            g[s] = cost; parent[s] = previous; closed.discard(s)
            heapq.heappush(queue, (cost+heuristic, s))
    def path(s):
        result = []
        while s != -1:
            result.append(s//9); s = parent[s]
        return result[::-1]
    bounded_loop = globals().get('_LOOP_DEADLINE') is not None
    loop_poll = 0
    while queue:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        _, s = heapq.heappop(queue)
        if s < 0: return path(-s-1)
        if s in closed: continue
        node, direction = divmod(s, 9)
        if any(other != s and g.get(other, float('inf')) + (0. if d == 8 else knick) < g[s]
               for d in range(9) for other in (node*9+d,)): continue
        closed.add(s)
        layer, idx = divmod(node, nn)
        if node in tgt: return path(s)
        if via_target and via[idx] and layer in (0, nl-1):
            extra = vzk*float(viaz[idx]) if viaz is not None and len(viaz) else 0.
            heapq.heappush(queue, (g[s]+.3+2.*math.sqrt(max(vx0-(x0+(idx % nx+.5)*res),0.,(x0+(idx % nx+.5)*res)-vx1)**2+
                max(vy0-(y0+(idx // nx+.5)*res),0.,(y0+(idx // nx+.5)*res)-vy1)**2)+(3.*pf if vbad[idx] else 0.)+extra, -s-1))
        y, x = divmod(idx, nx)
        for k, (dx, dy) in enumerate(directions):
            xx, yy = x+dx, y+dy
            if not (xa <= xx <= xb and ya <= yy <= yb): continue
            target = layer*nn+yy*nx+xx
            wm = wmin if not len(wminf) else wminf[target]
            if cap[target] < wm and target not in tgt: continue
            cost = g[s]+(res*1.4142 if k >= 4 else res)*float(mult[target])
            if direction != 8 and direction != k: cost += knick
            relax(target*9+k, cost, s, hfac*h(target))
        if via[idx] and not via_target:
            extra = float(vank[idx]) if vank is not None and len(vank) else 0.
            cost = vc*(1.+3.*pf*vbad[idx])+.2*float(hist[node])+extra
            for ll in range(nl):
                if ll == layer: continue
                target = ll*nn+idx
                change = cost
                if vzu is not None and len(vzu): change += vpr[layer*nl+ll]*float(vzu[idx])
                relax(target*9+8, g[s]+change, s, hfac*h(target))
    return None



_PADG = {li: [] for li in range(NL)}
_KA_PAD_KEEP = {li: [] for li in range(NL)}
_PADXY = {li: [] for li in range(NL)}
for _p in D['pads']:
    for _L in _p['layers']:
        if _L in LI:
            _PADXY[LI[_L]].append((_p['x'], _p['y']) if not _p.get('drill') else None)
            _PADG[LI[_L]].append((_p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p),
                                  shapely.union_all([Polygon(q) for q in _p['polys'][_L]])))
            if globals().get('KA') is not None:
                _KA_META[id(_PADG[LI[_L]][-1][1])] = _ka_pad_kind(_p)
                for _g,_kind in _ka_pad_keeps(_p,_L):
                    _KA_META[id(_g)]=(_kind,'')
                    _KA_PAD_KEEP[LI[_L]].append((_PADG[LI[_L]][-1][0],_g,_kind))
_PADT = {li: shapely.STRtree([g for _, g in _PADG[li]]) for li in range(NL)}





VPL = args.get('via_plaetze', '1' if KL else '0') == '1'
VP = {}
if VPL:
    from scipy.spatial import cKDTree as _KD2
    _jr = {}
    for _p in D['pads']:
        if not _p.get('drill') and any(L in LI for L in _p['layers']): _jr.setdefault(_p['ref'], []).append(_p)
    _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
    for _r in sorted(_jr):
        _P = _jr[_r]
        if len(_P) < 16 or any(max(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) > 1.3 * min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) for q in _P): continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _d, _i = _KD2(_xy).query(_xy, k=2); _pit = float(np.median(_d[:, 1]))
        if not 0.3 <= _pit <= 1.05: continue
        _v = _xy[_i[:, 1]] - _xy; _w = np.arctan2(_v[:, 1], _v[:, 0])
        _th = float(np.angle(np.mean(np.exp(4j * _w)))) / 4
        _e1 = (math.cos(_th) * _pit / 2, math.sin(_th) * _pit / 2); _e2 = (-_e1[1], _e1[0])
        _orte = sorted({(round(x + a * _e1[0] + b * _e2[0], 3), round(y + a * _e1[1] + b * _e2[1], 3)) for x, y in _xy.tolist() for a in (-1, 1) for b in (-1, 1)})
        for _x, _y in _orte:
            _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
            if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
            _pt = shapely.Point(_x, _y); _kr = _pt.buffer(_rn); _nah = []
            for _li in range(NL):
                for _j in _PADT[_li].query(_kr):
                    _n2, _g2 = _PADG[_li][int(_j)]; _dd = float(_pt.distance(_g2))
                    if _dd < _rn: _nah.append((_n2, round(_dd, 4), _li in (0, NL - 1)))
            VP[_iy * NX + _ix] = (_x, _y, tuple(sorted(_nah)))
    if VP: print('exakte Via-Plaetze im Kugelraster:', len(VP), flush=True)



BGA = {}
if BGA_FANOUT:
    from scipy.spatial import cKDTree as _BKD
    _bf = {f['ref']: f for f in D.get('fps', []) if 'BGA' in (f.get('fp', '') + f.get('lib', '')).upper()}
    for _ref in sorted(_bf):
        _bp = [p for p in D['pads'] if p['ref'] == _ref and not p.get('drill') and len(p['layers']) == 1 and p['layers'][0] in LI]
        if len(_bp) < 16: continue
        _xy = np.array([(p['x'], p['y']) for p in _bp]); _bd, _bi = _BKD(_xy).query(_xy, k=2)
        _pitch = float(np.median(_bd[:, 1]))
        if not 0.3 <= _pitch <= 1.05: continue
        _v = _xy[_bi[:, 1]] - _xy; _th = float(np.angle(np.mean(np.exp(4j * np.arctan2(_v[:, 1], _v[:, 0]))))) / 4
        _rot = np.array([[math.cos(_th), -math.sin(_th)], [math.sin(_th), math.cos(_th)]])
        _local = _xy @ _rot; _origin = _local.min(axis=0)
        _ij = np.rint((_local - _origin) / _pitch).astype(int)
        if np.max(np.abs(_local - _origin - _ij * _pitch)) > 0.02: continue
        _mx, _my = _ij.max(axis=0); _mid = (_local.min(axis=0) + _local.max(axis=0)) / 2
        for _p, (_i, _j), _lp in zip(_bp, _ij.tolist(), _local):
            _ring = min(_i, _j, int(_mx) - _i, int(_my) - _j)
            _sx, _sy = (1 if _lp[0] >= _mid[0] else -1), (1 if _lp[1] >= _mid[1] else -1)
            _signs = [(_sx, _sy), (_sx, -_sy), (-_sx, _sy), (-_sx, -_sy)]
            _dog = [tuple(np.round(np.array([_p['x'], _p['y']]) + (np.array([a, b]) * _pitch / 2) @ _rot.T, 5)) for a, b in _signs]
            _vip = [(_p['x'], _p['y'])] if VIP else []
            _pts = (_vip if _pitch <= 0.5 + 1e-6 else _dog + _vip)
            BGA[id(_p)] = dict(ref=_ref, pad=_p['num'], ring=_ring, pitch=round(_pitch, 5), layer=_p['layers'][0], pts=[])
            for _x, _y in _pts:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                _c = _iy * NX + _ix
                if _c in VP and math.dist((_x, _y), VP[_c][:2]) > 1e-5: continue
                _pt = shapely.Point(_x, _y); _nah = []; _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
                for _li in range(NL):
                    for _k in _PADT[_li].query(_pt.buffer(_rn)):
                        _n2, _g2 = _PADG[_li][int(_k)]; _dist = float(_pt.distance(_g2))
                        if _dist < _rn: _nah.append((_n2, round(_dist, 4), _li in (0, NL - 1)))
                VP[_c] = (_x, _y, tuple(sorted(_nah)))
                BGA[id(_p)]['pts'].append(_c)
    print('BGA-Fanout (Option): %d Balls, %d innere ab Ring 2' % (len(BGA), sum(v['ring'] >= 2 for v in BGA.values())), flush=True)
_VP_C = np.array(sorted(VP), dtype=np.int64); _VP_IY, _VP_IX = _VP_C // NX, _VP_C % NX







GASSEN = (args.get('gassen', '0') == '1' or BGA_ENG) and KL is not None
GS_PT = {}
GS_N = 0
if GASSEN:
    from shapely.ops import nearest_points as _np_pts
    _wg = float(args.get('gassen_w', '%g' % cls('/__signal__')['w'])); _rg = DRCP + PAD_EXTRA
    _gmin = _wg + 2 * _rg; _gmax = _wg + 2 * CLR_PAD + 2 * QM
    _nid = lambda n_: NID[n_] if n_ in NID else -2
    for _li in range(NL):
        _G = _PADG[_li]; _T = _PADT[_li]; _best = {}
        for _i, (_n1, _g1) in enumerate(_G):
            for _j in sorted(int(q) for q in _T.query(_g1.buffer(_gmax))):
                if _j <= _i: continue
                _n2, _g2 = _G[_j]
                if _n1 == _n2: continue
                _d = float(_g1.distance(_g2))
                if not (_gmin <= _d < _gmax): continue
                _pa, _pb = _np_pts(_g1, _g2); _ux, _uy = (_pb.x - _pa.x) / _d, (_pb.y - _pa.y) / _d
                _mx, _my = (_pa.x + _pb.x) / 2, (_pa.y + _pb.y) / 2; _vx, _vy = -_uy, _ux
                _b1, _b2 = _g1.bounds, _g2.bounds
                _L = max(_b1[2] - _b1[0], _b1[3] - _b1[1], _b2[2] - _b2[0], _b2[3] - _b2[1]) / 2 + 0.3
                _k = int(_L / (RES / 4))
                for _t in range(-_k, _k + 1):
                    _x, _y = _mx + _vx * _t * RES / 4, _my + _vy * _t * RES / 4
                    _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                    if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                    _c = _iy * NX + _ix
                    if 2 * (float(PD1[_li].ravel()[_c]) - CLR_PAD) >= _wg: continue
                    _pt = shapely.Point(_x, _y)
                    _dm = min(float(_G[int(q)][1].distance(_pt)) for q in _T.query(_pt.buffer(_gmax)))
                    _cap = 2 * (_dm - _rg)
                    if _cap < _wg - 1e-9: continue
                    _o = (abs(_x - cell_xy(_ix, _iy)[0]) + abs(_y - cell_xy(_ix, _iy)[1]), -_cap)
                    if _c not in _best or _o < _best[_c][0]: _best[_c] = (_o, _x, _y, _cap, _nid(_n1), _nid(_n2))
        for _c, (_o, _x, _y, _cap, _a, _b) in sorted(_best.items()): GS_PT[_li * NN + _c] = (round(_x, 4), round(_y, 4), _a, _b, _cap)
    GS_N = len(GS_PT)
    _GS_I = np.array(sorted(GS_PT), dtype=np.int64)
    _GS_CAP = np.array([GS_PT[c][4] for c in _GS_I.tolist()], dtype=np.float32)
    _GS_A = np.array([GS_PT[c][2] for c in _GS_I.tolist()], dtype=np.int64); _GS_B = np.array([GS_PT[c][3] for c in _GS_I.tolist()], dtype=np.int64)
    print('enge Gassen (R15, Offene 29): %d Zellen auf der exakten Mittellinie begehbar (Bahn %.3f mm)' % (GS_N, _wg), flush=True)


def _gassen_ein(net, capH, rng):
    ""
    nid = NID[net]; m = (_GS_A != nid) & (_GS_B != nid)
    idx = _GS_I[m]; cap = _GS_CAP[m]; ic = idx % NN; li = idx // NN
    if rng is not None:
        ix, iy = ic % NX, ic // NX; w = (ix >= rng[0]) & (ix < rng[1]) & (iy >= rng[2]) & (iy < rng[3]); idx, cap, ic, li = idx[w], cap[w], ic[w], li[w]
    if not len(idx): return
    cap = np.minimum(cap, 2 * (edge_d.ravel()[ic] - EDGE)).astype(np.float32)
    if FIXD[0] is not None and net not in FIXNETS:
        cap = np.minimum(cap, np.array([2 * (FIXD[0][l].ravel()[c] - QM - CLR - 0.005) for l, c in zip(li.tolist(), ic.tolist())], dtype=np.float32))
    capH.flach[idx] = np.maximum(capH.flach[idx], cap)

VIP_EP = (not VIP) and args.get('vip_ep', '1' if KL else '0') == '1'
EPZ = {}



EP_VIAS = bool(FLNETZE) and args.get('ep_vias', '1' if KL else '0') == '1'
EP_ABST = max(float(args.get('ep_abstand', '%g' % (VIA_D + 0.3))), HOLE_C)
EP_ABST_K12 = max(EP_ABST, VIA_D + 2 * CLR + 0.2)
EPM = {}
EPG = []
_ep = {}; _epk = {}
if VIP_EP or EP_VIAS:
    _jr3 = {}; _jk3 = {}
    for _k, _p in enumerate(D['pads']):
        if not _p.get('drill'): _jr3.setdefault(_p['ref'], []).append(_p); _jk3[id(_p)] = _k
    for _r in sorted(_jr3):
        _P = _jr3[_r]
        if len(_P) < 8: continue
        for _p in _P:
            _lg = [L for L in _p['layers'] if L in LI and LI[L] in (0, NL - 1)]; _b = _p['bbox']
            if not _p['net'] or _p['net'].startswith('unconnected') or len(_lg) != 1 or min(_b[2] - _b[0], _b[3] - _b[1]) < 1.0: continue
            _xs = [q['x'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            _ys = [q['y'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            if len(_xs) < 6 or not (min(_xs) < _p['x'] < max(_xs) and min(_ys) < _p['y'] < max(_ys)): continue
            _ep.setdefault((_r, _p['net'], LI[_lg[0]]), []).extend(Polygon(q) for q in _p['polys'][_lg[0]] if len(q) >= 3)
            _epk.setdefault((_r, _p['net'], LI[_lg[0]], _p['num']), []).append(_jk3[id(_p)])
if VIP_EP:
    for (_r, _n, _li), _pl in sorted(_ep.items()):
        _g = shapely.union_all(_pl).buffer(-VIA_R)
        if _g.is_empty: continue
        _b = _g.bounds
        _ixa, _ixb = max(1, int((_b[0] - X0) / RES)), min(NX - 1, int((_b[2] - X0) / RES) + 1)
        _iya, _iyb = max(1, int((_b[1] - Y0) / RES)), min(NY - 1, int((_b[3] - Y0) / RES) + 1)
        if _ixb <= _ixa or _iyb <= _iya: continue
        _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
        _in = shapely.contains_xy(_g, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
        if _in.any(): EPZ.setdefault(_n, []).append((_li, _GY.ravel()[_in].astype(np.int64), _GX.ravel()[_in].astype(np.int64)))
    if EPZ: print('Thermal-Pads mit erlaubtem Via im Pad:', sum(len(v) for v in EPZ.values()), flush=True)


def _ep_achse(W):
    ""
    return max(1, int((W - VIA_D + 1e-9) / EP_ABST) + 1) if W >= VIA_D else 1


def _ep_teilung(W, n):
    ""
    return 0.0 if n < 2 else min((W - VIA_D) / (n - 1), max(W / n, EP_ABST_K12))


if EP_VIAS:
    for (_r, _n, _li, _num), _ks in sorted(_epk.items()):
        if _n not in FLNETZE: continue
        _L = LAYERS[_li]
        _gf = [shapely.union_all([Polygon(q) for q in D['pads'][_k]['polys'][_L] if len(q) >= 3]) for _k in _ks]
        _g = shapely.union_all(_gf); _A = float(_g.area)
        _soll = 4 if _A < 9.0 else (9 if _A < 25.0 else 16)
        _je = -(-_soll // len(_ks))
        EPG.append(dict(ref=_r, num=_num, netz=_n, flaeche=round(_A, 2), soll=_soll, felder=len(_ks), poly=_g.buffer(0.02), pads=list(_ks)))
        if _je < 2: continue
        for _k, _gk in zip(_ks, _gf):
            _b = _gk.bounds; _W, _H = _b[2] - _b[0], _b[3] - _b[1]; _cx, _cy = (_b[0] + _b[2]) / 2, (_b[1] + _b[3]) / 2
            _best = None
            for _nx in range(1, _ep_achse(_W) + 1):
                for _ny in range(1, _ep_achse(_H) + 1):
                    _q = (max(0, _je - _nx * _ny), max(0, _nx * _ny - _je), abs(_ep_teilung(_W, _nx) - _ep_teilung(_H, _ny)), _nx, _ny)
                    if _best is None or _q < _best: _best = _q
            _nx, _ny = _best[3], _best[4]; _px, _py = _ep_teilung(_W, _nx), _ep_teilung(_H, _ny)
            _gi = _gk.buffer(-VIA_R + 0.002)
            if _gi.is_empty: continue
            _orte = []
            for _j in range(_ny):
                for _i in range(_nx):
                    _ix, _iy = int((_cx + (_i - (_nx - 1) / 2) * _px - X0) / RES), int((_cy + (_j - (_ny - 1) / 2) * _py - Y0) / RES)
                    if 0 < _ix < NX - 1 and 0 < _iy < NY - 1 and _iy * NX + _ix not in _orte: _orte.append(_iy * NX + _ix)
            _bi = _gi.bounds
            _ixa, _ixb = max(1, int((_bi[0] - X0) / RES)), min(NX - 1, int((_bi[2] - X0) / RES) + 1)
            _iya, _iyb = max(1, int((_bi[1] - Y0) / RES)), min(NY - 1, int((_bi[3] - Y0) / RES) + 1)
            if _ixb <= _ixa or _iyb <= _iya: continue
            _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
            _in = shapely.contains_xy(_gi, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
            EPM[_k] = dict(soll=_nx * _ny, wunsch=_je, orte=_orte, zellen=(_GY.ravel()[_in] * NX + _GX.ravel()[_in]).astype(np.int64),
                           teilung=(round(_px, 3), round(_py, 3)))
    if EPG: print('Massepads unter Chips (R16):', ', '.join('%s:%s %.1f mm2 Soll %d' % (e['ref'], e['num'], e['flaeche'], e['soll']) for e in EPG), flush=True)



EP_RES = {}
for _k, _E in sorted(EPM.items()):
    _m = EP_RES.setdefault(D['pads'][_k]['net'], np.zeros((NY, NX), dtype=bool)); _rr = int(EP_ABST / RES) + 1
    for _c in _E['orte']:
        _iy, _ix = divmod(_c, NX)
        _ya, _yb, _xa, _xb = max(0, _iy - _rr), min(NY, _iy + _rr + 1), max(0, _ix - _rr), min(NX, _ix + _rr + 1)
        _GX, _GY = np.meshgrid(np.arange(_xa, _xb), np.arange(_ya, _yb))
        _m[_ya:_yb, _xa:_xb] |= np.hypot(_GX - _ix, _GY - _iy) * RES < EP_ABST - 1e-6


def via_hart(net, cells):
    ""
    if not len(cells): return False
    if not GROSS: return bool(hard_maps(net)[1].ravel()[cells].any())
    iy, ix = cells // NX, cells % NX
    rng = (max(0, int(ix.min()) - 2), min(NX, int(ix.max()) + 3), max(0, int(iy.min()) - 2), min(NY, int(iy.max()) + 3))
    return bool(hard_maps(net, rng)[1].ravel()[cells].any())


def ep_setzen(k, viaH, vbadf, rng):
    ""
    E = EPM[k]; viaf = viaH.ravel(); Z = E['zellen']
    def gilt(c): return rng[0] < c % NX < rng[1] - 1 and rng[2] < c // NX < rng[3] - 1
    zx = X0 + (Z % NX + 0.5) * RES; zy = Y0 + (Z // NX + 0.5) * RES
    frei = viaf[Z] & ~vbadf[Z] & (Z % NX > rng[0]) & (Z % NX < rng[1] - 1) & (Z // NX > rng[2]) & (Z // NX < rng[3] - 1) if len(Z) else None
    def versuch(ab):
        gesetzt = []; offen = []
        def weit(c):
            x, y = cell_xy(c % NX, c // NX)
            return all(math.hypot(x - a, y - b) >= EP_ABST - 1e-6 for a, b in gesetzt)
        for c in E['orte']:
            if gilt(c) and viaf[c] and not vbadf[c] and weit(c): gesetzt.append(cell_xy(c % NX, c // NX))
            else: offen.append(c)
        if offen and frei is not None:
            rest = []
            for c in offen:
                x, y = cell_xy(c % NX, c // NX); m = frei.copy()
                for a, b in gesetzt: m &= np.hypot(zx - a, zy - b) >= ab - 1e-6
                if not m.any(): rest.append(c); continue
                d = np.hypot(zx - x, zy - y); d[~m] = 1e9; j = int(np.argmin(d))
                gesetzt.append((float(zx[j]), float(zy[j])))
            offen = rest
        if VG_WEICH[0]:
            for c in offen:
                if gilt(c) and viaf[c] and weit(c): gesetzt.append(cell_xy(c % NX, c // NX))
        return gesetzt
    gesetzt = versuch(EP_ABST)
    if EP_ABST_K12 > EP_ABST + 1e-9:
        g2 = versuch(EP_ABST_K12)
        if len(g2) >= len(gesetzt): gesetzt = g2
    return [dict(x=round(x, 4), y=round(y, 4)) for x, y in gesetzt]







VAP = VIP and bool(FLNETZE) and args.get('via_am_pad', '1' if KL else '0') == '1'
VAP_K = 3.0 * float(args.get('via_am_pad_mm', '1.5'))
VAP_DRITTEL = float(args.get('via_am_pad_drittel', '0.25'))
VAP_WF = float(args.get('via_am_pad_wichtig', '2.0'))
VAP_WICHTIG = set()
_VZ = np.ones(NN, dtype=np.float32) if VAP else None
_VAPZ = {}
if VAP and KL:
    _wr = {r for r, t in KL.get('teile', {}).items() if t.get('entkopplung')}
    _wr |= {p['ref'] for p in D['pads'] if (KL['netze'].get(p['net']) or {}).get('art') == 'sw'}
    VAP_WICHTIG = {k for k, p in enumerate(D['pads']) if p['net'] in FLNETZE and p['ref'] in _wr}
if VAP: print('Via am Pad (R9): ein Via neben dem Pad kostet %.1f (wie %.1f mm Stich), %d von %d Pads der Flaechennetze zaehlen %gfach (Entkopplung, Schaltknoten)' % (
    VAP_K, VAP_K / 3.0, len(VAP_WICHTIG), sum(len(bynet_g.get(n, ())) for n in FLNETZE), VAP_WF), flush=True)


def vap_anteil(g, xs, ys):
    ""
    if not len(xs): return np.zeros(0)
    kr = shapely.buffer(shapely.points(xs, ys), VIA_R, quad_segs=8)
    return shapely.area(shapely.intersection(kr, g)) / min(float(shapely.area(kr[0])), float(g.area))


def _vap_zellen(k):
    ""
    r = _VAPZ.get(k)
    if r is None:
        p = D['pads'][k]; L = next(x for x in p['layers'] if x in LI)
        g = shapely.union_all([Polygon(q) for q in p['polys'][L] if len(q) >= 3]); b = g.bounds
        ixa, ixb = max(0, int((b[0] - VIA_R - X0) / RES)), min(NX, int((b[2] + VIA_R - X0) / RES) + 1)
        iya, iyb = max(0, int((b[1] - VIA_R - Y0) / RES)), min(NY, int((b[3] + VIA_R - Y0) / RES) + 1)
        GX, GY = np.meshgrid(np.arange(ixa, ixb), np.arange(iya, iyb)); gx = GX.ravel(); gy = GY.ravel()
        x = X0 + (gx + 0.5) * RES; y = Y0 + (gy + 0.5) * RES
        nah = shapely.distance(g, shapely.points(x, y)) < VIA_R
        a = vap_anteil(g, x[nah], y[nah]); m = a >= 1 / 3 - 0.01
        r = _VAPZ[k] = ((gy[nah][m] * NX + gx[nah][m]).astype(np.int64), np.where(a[m] >= 0.49, 0.0, VAP_DRITTEL).astype(np.float32))
    return r


def _ep_ein(net, viaH, capH, rng):
    ""
    for li_e, cy_, cx_ in EPZ.get(net, ()):
        m = ~viaH[cy_, cx_]
        if rng is not None: m &= (cx_ >= rng[0]) & (cx_ < rng[1]) & (cy_ >= rng[2]) & (cy_ < rng[3])
        if not m.any(): continue
        cy2, cx2 = cy_[m], cx_[m]
        ok = (edge_d[cy2, cx2] - EDGE >= VIA_R) & (VSP[cy2, cx2] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_VERBOT is not None: ok &= ~VIA_VERBOT[cy2, cx2]
        if FIXD[1] is not None and net not in FIXNETS: ok &= FIXD[1][cy2, cx2] - QM >= HOLE_C
        for li in range(NL):
            ok &= capH[li][cy2, cx2] / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and li != li_e: ok &= padd[li][cy2, cx2] >= VIA_R + VIA_STEG
        viaH[cy2[ok], cx2[ok]] = True


def _vp_ok(net, nah):
    ""
    for n2, d, aussen in nah:
        if n2 != net:
            if d < VIA_R + DRCP + 0.006: return False
            if BGA_LOCH and d < VIA_DR / 2 + LOCH_CLR + 0.000001: return False
        elif aussen and not VIP and d < VIA_R + VIA_STEG: return False
    return True
def _vp_xy(net, c):
    ""
    if globals().get('KA') is not None:
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _ka_via_ok(net,v[0],v[1]) else None
    if PINZUGANG and net in PLANE_AREA and net not in globals().get('PLANE_MIXED_NETS',()) and c in VP and not PLANE_AREA[net].covers(_PT(*VP[c][:2])): return None
    if _pz_active(net) and c in VP and _pz_near(net,*VP[c][:2]):
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _pz_via_ok(net,v[0],v[1]) else None
    if FIXD[0] is not None: return None
    v = VP.get(c)
    return (v[0], v[1]) if v is not None and _vp_ok(net, v[2]) else None


def _vp_ein(net, viaH, rng):
    ""
    if globals().get('KA') is not None:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            if _ka_via_ok(net,*VP[c][:2]):viaH[iy,ix]=True
        return
    if _pz_active(net) and not _PZ_BASE[0]:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            v=VP[c]
            if not _pz_near(net,v[0],v[1]):continue
            if _pz_via_ok(net,v[0],v[1]): viaH[iy,ix]=True
        return
    if not len(_VP_C) or FIXD[0] is not None: return
    m = ~viaH[_VP_IY, _VP_IX] & (edge_d[_VP_IY, _VP_IX] - EDGE >= VIA_R) & (VSP[_VP_IY, _VP_IX] >= H2H + VIA_DR / 2 + 0.01)
    if VIA_VERBOT is not None: m &= ~VIA_VERBOT[_VP_IY, _VP_IX]
    if rng is not None: m &= (_VP_IX >= rng[0]) & (_VP_IX < rng[1]) & (_VP_IY >= rng[2]) & (_VP_IY < rng[3])
    if FIXD[1] is not None and net not in FIXNETS: m &= FIXD[1][_VP_IY, _VP_IX] - QM >= HOLE_C
    for c in _VP_C[m].tolist():
        if PINZUGANG and net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
            if not PLANE_MASK[net][c // NX, c % NX] or not PLANE_AREA[net].covers(_PT(*VP[c][:2])): continue
        if _vp_ok(net, VP[c][2]): viaH[c // NX, c % NX] = True


from functools import lru_cache as _lru_cache
PAD_ROOM_CACHE = args.get('pad_room_cache', '1') == '1'


def pad_room(li, a, b, net):

    a, b = tuple(a), tuple(b)
    return _pad_room_cached(li, a, b, net) if PAD_ROOM_CACHE else _pad_room_exact(li, a, b, net)


def _pad_room_exact(li, a, b, net):
    ""
    if globals().get('KA') is not None: return _ka_room(net,li,a,b)
    seg = _LS([a, b]) if (abs(a[0] - b[0]) > 1e-9 or abs(a[1] - b[1]) > 1e-9) else _PT(a)
    best = 9.0
    for j in _PADT[li].query(seg.buffer(1.0)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == net: continue
        best = min(best, seg.distance(g2))
    room = best - (DRCP + PAD_EXTRA)
    if _pz_active(net): room = min(best - DRCP - PZ_RAND_NM / 1e6, _pz_room(net,li,a,b))
    if ZONENPLAN:
        room = min(room, min((seg.distance(g)-CLR-.005 for (n,L),g in _ZP_GEO.items() if n!=net and L==LAYERS[li]), default=9.0))
    if PLANE_COPPER:
        room=min(room,min((seg.distance(g)-CLR-.005 for (n,L),g in PLANE_COPPER.items() if n!=net and L==LAYERS[li]),default=9.))
    return room


_pad_room_cached = _lru_cache(maxsize=100000)(_pad_room_exact)


def _pad_room_lower_loop(d1, d2, pn, nid, ax, ay, bx, by, x0, y0, res, nx, ny, count):
    best = 9.0
    for k in range(count):
        t = k / (count - 1)
        ix = int(math.floor((ax + t * (bx - ax) - x0) / res))
        iy = int(math.floor((ay + t * (by - ay) - y0) / res))
        if ix < 0 or ix >= nx or iy < 0 or iy >= ny: return -999.0
        d = d2[iy, ix] if pn[iy, ix] == nid else d1[iy, ix]
        if d < best: best = d
    return best


_pad_room_lower_nb = None
if NB is not None:
    from numba import njit as _pad_njit
    _pad_room_lower_nb = _pad_njit(cache=True)(_pad_room_lower_loop)


@_lru_cache(maxsize=100000)
def _pad_room_lower(li, a, b, net):
    L = math.dist(a, b); count = max(2, int(L / (RES * 0.5)) + 1)
    f = _pad_room_lower_nb if _pad_room_lower_nb is not None else _pad_room_lower_loop
    dist = f(PD1[li], PD2[li], PN1[li], NID.get(net, -1), a[0], a[1], b[0], b[1], X0, Y0, RES, NX, NY, count)
    return min(float(dist), RW - 2 * RES) - RES / math.sqrt(2) - L / (2 * (count - 1)) - 1e-5 - DRCP - PAD_EXTRA


def pad_room_min(li, a, b, net, bedarf):
    ""
    if globals().get('KA') is None and PAD_ROOM_CACHE and not ZONENPLAN and not _pz_active(net):
        if _pad_room_lower(li, tuple(a), tuple(b), net) >= bedarf: return bedarf
    return min(bedarf, pad_room(li, a, b, net))


def line_cap_min(capL, a, b):
    (x0, y0), (x1, y1) = a, b
    L = math.hypot(x1 - x0, y1 - y0); n = max(2, int(L / (RES * 0.5)) + 1)
    if NB is not None:
        return float(_linecap_nb(capL, x0, y0, x1, y1, n, X0, Y0, RES))
    t = np.linspace(0, 1, n)
    ix = ((x0 + t * (x1 - x0) - X0) / RES).astype(int); iy = ((y0 + t * (y1 - y0) - Y0) / RES).astype(int)
    return float(capL[iy, ix].min())


def okt(a, b):
    ""
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6






HYPO = OKT and args.get('hypo', '1') != '0'



HYPO_SPAET = HYPO and args.get('hypo', 'ende') == 'ende' and ('start' not in args or args.get('hypo') == 'ende')
_HYPO_AN = [HYPO and not HYPO_SPAET]


def _ecken(a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; m = min(abs(dx), abs(dy))
    if m < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9: return ()
    sx, sy = (1.0 if dx > 0 else -1.0), (1.0 if dy > 0 else -1.0)
    return ((b[0] - sx * m, b[1] - sy * m), (a[0] + sx * m, a[1] + sy * m))










PADMITTE = args.get('padmitte', '1' if KL else '0') == '1'
PADMITTE_MAX = float(args.get('padmitte_max', '4.0'))


def _padmitte(li, pt):
    ""
    q = _PT(pt)
    for j in _PADT[li].query(q.buffer(0.03)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.distance(q) <= 0.03:
            c = _PADXY[li][int(j)]
            if c is not None and g2.contains(_PT(c)): return (c, g2)
    return None


def _padanschluss(P, li, capL, C):
    ""
    r = _padmitte(li, P[-1])
    if r is None: return P
    c, g = r; e = P[-1]; d = math.dist(e, c)
    if d < 1e-6 or d > PADMITTE_MAX: return P
    req = C['wmin']
    def frei(a, b): return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= req / 2 and (not IMP_REF_MASK or z_ref_frei(C, li, a, b))
    def im_pad(a, b): return g.buffer(0.03).contains(_LS([a, b]))
    def wege(a, b):
        if not okt_netz() or okt(a, b): yield []
        if okt_netz():
            for ck in reversed(_ecken(a, b)): yield [ck]
    if len(P) >= 3:
        q, a = P[-3], P[-2]; d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        det = -d1[0] * d2[1] + d1[1] * d2[0]
        if abs(det) > 1e-9:
            t = (-(c[0] - q[0]) * d2[1] + (c[1] - q[1]) * d2[0]) / det
            a2 = (q[0] + t * d1[0], q[1] + t * d1[1])
            if t > 1e-6 and math.dist(a2, a) <= 0.25 and frei(q, a2) and frei(a2, c): return P[:-2] + [a2, c]
    if len(P) >= 2:
        a = P[-2]
        for zw in wege(a, c):
            pk = [a] + zw + [c]
            if all(frei(x, y) for x, y in zip(pk[:-1], pk[1:])): return P[:-1] + zw + [c]
    for zw in wege(e, c):
        pk = [e] + zw + [c]
        if all(im_pad(x, y) and frei(x, y) for x, y in zip(pk[:-1], pk[1:])): return P + zw + [c]
    return P












R27_STAMM = PADMITTE and args.get('r27_stamm', '0') == '1'
ABZW_EXAKT = args.get('abzweig', '0') == '1' or (R27_STAMM and args.get('abzweig_exakt', '1') == '1')
VERSATZ = PADMITTE and args.get('versatz', '1') == '1'
STOSS_STAT = dict(abzweig_exakt=0, abzweig_geschoben=0, abzweig_ecke=0, abzweig_schief=0, abzweig_offen=0, via_start=0, via_start_neben=0,
                  via_neben_bahn=0, versatz=0, versatz_ende=0)
def _schnitt(p, d1, q, d2):
    ""
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-9 * max(1e-9, math.hypot(*d1) * math.hypot(*d2)): return None
    t = ((q[0] - p[0]) * d2[1] - (q[1] - p[1]) * d2[0]) / det
    return (p[0] + t * d1[0], p[1] + t * d1[1])


def _auf_anker(P, A, frei):
    ""
    if math.dist(P[0], A) < 1e-7: return [A] + P[1:]
    if not okt_netz(): return [A] + P[1:] if frei(A, P[1]) else None
    if len(P) < 3: return None
    d1 = (P[1][0] - P[0][0], P[1][1] - P[0][1]); d2 = (P[2][0] - P[1][0], P[2][1] - P[1][1])
    a2 = _schnitt(A, d1, P[1], d2)
    if a2 is None or math.dist(a2, P[1]) > 0.25: return None
    if (a2[0] - A[0]) * d1[0] + (a2[1] - A[1]) * d1[1] <= 1e-9 or (P[2][0] - a2[0]) * d2[0] + (P[2][1] - a2[1]) * d2[1] <= 1e-9: return None
    return [A, a2] + P[2:] if frei(A, a2) and frei(a2, P[2]) else None


def _stamm_punkt(t, i, X):
    ""
    t['pts'] = list(t['pts'][:i + 1]) + [[X[0], X[1]]] + list(t['pts'][i + 1:])
    t['w'] = list(t['w'][:i + 1]) + [t['w'][i]] + list(t['w'][i + 1:])


def _abzweig(P, li, node, capL, C):
    ""
    S = P[0]
    if len(P) < 2: return P
    v = _STAMM_V.get(node); st = [t for t in _STAMM.get(node, ()) if LI[t['layer']] == li]
    if v is None and not st: return P
    c0 = min(C['wmin'], line_cap_min(capL, S, P[1])); r0 = min(C['wmin'] / 2, pad_room(li, S, P[1], NET_CUR[0]))



    def frei(a, b): return pad_room(li, a, b, NET_CUR[0]) >= r0 - 1e-9 and (not okt_netz() or line_cap_min(capL, a, b) >= c0 - 1e-6)
    if v is not None:
        if math.dist(S, v) < 1e-7: return P
        Q = _auf_anker(P, v, frei); STOSS_STAT['via_start' if Q else 'via_start_neben'] += 1
        return Q or P
    kand = []; ecke = []
    d1 = (P[1][0] - S[0], P[1][1] - S[1])
    for t in st:
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); L_ = (a[0] + u * dx, a[1] + u * dy); d = math.dist(S, L_)
            if d > 0.045: continue
            ecke.append((d, 1, len(ecke), i, L_, t))
            for j_, e_ in ((i, a), (i + 1, b)):
                if math.dist(S, e_) <= 0.08: ecke.append((math.dist(S, e_), 0, len(ecke), j_, (e_[0], e_[1]), t))
            if d < 1e-7: kand.append((0.0, len(kand), t, i, S)); continue
            X = _schnitt(S, d1, a, (dx, dy))
            if X is None: continue
            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
            if ux < -1e-9 or ux > 1 + 1e-9 or math.dist(X, S) > 0.08: continue
            if (P[1][0] - X[0]) * d1[0] + (P[1][1] - X[1]) * d1[1] <= 1e-9: continue
            kand.append((math.dist(X, S), len(kand), t, i, X))
    for d, _, t, i, X in sorted(kand, key=lambda k_: k_[:2]):
        a, b = t['pts'][i], t['pts'][i + 1]
        if math.dist(X, a) < 2e-4: X = (a[0], a[1]); neu = False
        elif math.dist(X, b) < 2e-4: X = (b[0], b[1]); neu = False
        else: X = (round(X[0], 4), round(X[1], 4)); neu = True


        if neu: _stamm_punkt(t, i, X)
        STOSS_STAT['abzweig_exakt' if d <= 1e-7 else 'abzweig_geschoben'] += 1
        return [X] + P[1:]
    for d, lot, _, i, A, t in sorted(ecke, key=lambda k_: k_[:3]):
        A = (round(A[0], 4), round(A[1], 4))
        Q = _auf_anker(P, A, frei); art = 'abzweig_ecke'
        if Q is None and frei(A, P[1]): Q = [A] + P[1:]; art = 'abzweig_schief'
        if Q is None: continue
        if lot and math.dist(A, t['pts'][i]) > 2e-4 and math.dist(A, t['pts'][i + 1]) > 2e-4: _stamm_punkt(t, i, A)
        STOSS_STAT[art] += 1
        return Q
    if kand or ecke: STOSS_STAT['abzweig_offen'] += 1
    return P


def _via_auf_stamm(li, node, S):
    ""
    if node in _STAMM_V: return
    best = None
    for t in _STAMM.get(node, ()):
        if LI[t['layer']] != li: continue
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); d = math.hypot(S[0] - a[0] - u * dx, S[1] - a[1] - u * dy)
            if best is None or d < best[0]: best = (d, t, i)
    if best is None: return
    d, t, i = best
    if d > 1e-7: STOSS_STAT['via_neben_bahn'] += 1; return
    X = (round(S[0], 4), round(S[1], 4))
    if math.dist(X, t['pts'][i]) > 2e-4 and math.dist(X, t['pts'][i + 1]) > 2e-4: _stamm_punkt(t, i, X)


def stoesse_reparieren():
    ""
    n = 0; je = {}; alt = {}
    for u, r in U.items():
        if u.startswith(('FIX#', 'HEFT#')): continue
        V = [(v['x'], v['y']) for v in r['vias'] if (v.get('net') or r['net']) == r['net']]; ch = False
        alt[u] = [(t, list(t['pts']), list(t['w'])) for t in r['tracks']]
        for t in r['tracks']:
            if len(t['pts']) < 2: continue
            li = LI[t['layer']]
            for ende in (0, -1):
                E = t['pts'][ende]; N = t['pts'][1] if ende == 0 else t['pts'][-2]
                if any(math.dist(E, v) < 1e-6 for v in V): continue
                _pe = _PT(E)
                if any(_PADG[li][int(j)][0] == r['net'] and _PADG[li][int(j)][1].covers(_pe) for j in _PADT[li].query(_pe)): continue
                def richtungstreu(X):
                    if not okt_netz(r['net']): return True
                    for t3 in r['tracks']:
                        if t3['layer'] != t['layer'] or len(t3['pts']) < 2: continue
                        for e3, n3 in ((0, 1), (-1, -2)):
                            if math.dist(t3['pts'][e3], E) >= 1e-6: continue
                            q = t3['pts'][n3]; dx, dy = E[0] - q[0], E[1] - q[1]
                            vx, vy = X[0] - q[0], X[1] - q[1]
                            if not okt(q, X) or abs(dx * vy - dy * vx) > 1e-7 * max(1.0, math.hypot(dx, dy)) or dx * vx + dy * vy <= 0:
                                return False
                    return True
                best = None; exakt = False
                for t2 in r['tracks']:
                    if t2 is t or t2['layer'] != t['layer']: continue
                    P2 = t2['pts']
                    for i in range(len(P2) - 1):
                        a, b = P2[i], P2[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
                        if l2 < 1e-12: continue
                        uu = max(0.0, min(1.0, ((E[0] - a[0]) * dx + (E[1] - a[1]) * dy) / l2))
                        d = math.hypot(E[0] - a[0] - uu * dx, E[1] - a[1] - uu * dy)
                        if d <= 1e-7: exakt = True; break
                        if d > 0.045: continue
                        X = _schnitt(E, (N[0] - E[0], N[1] - E[1]), a, (dx, dy))
                        if X is not None:
                            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
                            if -1e-9 <= ux <= 1 + 1e-9 and math.dist(X, E) <= 0.08 and (N[0] - X[0]) * (N[0] - E[0]) + (N[1] - X[1]) * (N[1] - E[1]) > 1e-9:
                                k_ = (math.dist(X, E), 0, t2, i, X)
                                if richtungstreu(X) and (best is None or k_[:2] < best[:2]): best = k_
                                continue
                        for e_ in (a, b):
                            if math.dist(E, e_) <= 0.08:
                                k_ = (math.dist(E, e_), 1, t2, i, (e_[0], e_[1]))
                                if richtungstreu(e_) and (best is None or k_[:2] < best[:2]): best = k_
                    if exakt: break
                if exakt or best is None: continue
                _, _, t2, i, X = best
                a, b = t2['pts'][i], t2['pts'][i + 1]
                if math.dist(X, a) < 2e-4: X = (a[0], a[1])
                elif math.dist(X, b) < 2e-4: X = (b[0], b[1])
                else: X = (round(X[0], 4), round(X[1], 4))
                if not richtungstreu(X): continue
                if math.dist(X, a) >= 2e-4 and math.dist(X, b) >= 2e-4: _stamm_punkt(t2, i, X)
                for t3 in r['tracks']:
                    if t3['layer'] != t['layer']: continue
                    for e3 in (0, -1):
                        if math.dist(t3['pts'][e3], E) < 1e-6:
                            if e3 == 0: t3['pts'] = [[X[0], X[1]]] + list(t3['pts'][1:])
                            else: t3['pts'] = list(t3['pts'][:-1]) + [[X[0], X[1]]]
                n += 1; ch = True; je[u] = je.get(u, 0) + 1
        if ch: _GEO.pop(u, None)
    if je:
        nach = exact_conflicts()
        for u in sorted(je):
            if u not in nach: continue
            for t, p_, w_ in alt[u]: t['pts'] = p_; t['w'] = w_
            _GEO.pop(u, None); n -= je[u]; STOSS_STAT['stoss_zurueck'] = STOSS_STAT.get('stoss_zurueck', 0) + je[u]
    return n


def kollinear_zusammen():
    ""
    kn = {}
    for u, r in U.items():
        s = kn.setdefault(r['net'], set())
        for t in r['tracks']:
            for q in (t['pts'][0], t['pts'][-1]): s.add((t['layer'], round(q[0], 4), round(q[1], 4)))
        for v in r['vias']:
            if (v.get('net') or r['net']) != r['net']: continue
            for L in LAYERS: s.add((L, round(v['x'], 4), round(v['y'], 4)))
    n = 0
    for u, r in U.items():
        if u.startswith(('FIX#', 'HEFT#')): continue
        s = kn[r['net']]; ch = False
        for t in r['tracks']:
            P, W = t['pts'], t['w']
            if len(P) < 3: continue
            np_, nw = [P[0]], []
            for i in range(1, len(P)):
                if nw and abs(nw[-1] - W[i - 1]) < 1e-6 and len(np_) >= 2 and (t['layer'], round(np_[-1][0], 4), round(np_[-1][1], 4)) not in s:
                    a, b, c = np_[-2], np_[-1], P[i]; dx, dy = c[0] - a[0], c[1] - a[1]; l_ = math.hypot(dx, dy)
                    if l_ > 1e-9 and abs((b[0] - a[0]) * dy - (b[1] - a[1]) * dx) / l_ <= 2e-4 and (b[0] - a[0]) * dx + (b[1] - a[1]) * dy > 0 \
                            and (c[0] - b[0]) * dx + (c[1] - b[1]) * dy > 0:
                        np_[-1] = P[i]; n += 1; ch = True; continue
                np_.append(P[i]); nw.append(W[i - 1])
            if len(np_) != len(P): t['pts'] = np_; t['w'] = nw
        if ch: _GEO.pop(u, None)
    return n


MINI = 0.1


def _versatz_weg(P, li, capL, C):
    ""
    if len(P) < 4: return P
    def cap(a, b): return line_cap_min(capL, a, b)
    def frei(a, b, req): return cap(a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= C['wmin'] / 2 and (not IMP_REF_MASK or z_ref_frei(C, li, a, b))
    def mitte(Q, i):
        ""
        if i < 1 or i + 3 > len(Q) - 1: return None
        u = (Q[i][0] - Q[i - 1][0], Q[i][1] - Q[i - 1][1]); v = (Q[i + 2][0] - Q[i + 1][0], Q[i + 2][1] - Q[i + 1][1])
        lu, lv = math.hypot(*u), math.hypot(*v)
        if lu < 1e-9 or lv < 1e-9 or abs(u[0] * v[1] - u[1] * v[0]) > 1e-6 * lu * lv or u[0] * v[0] + u[1] * v[1] <= 0: return None
        w = (Q[i + 3][0] - Q[i + 2][0], Q[i + 3][1] - Q[i + 2][1])
        X = _schnitt(Q[i - 1], u, Q[i + 2], w)
        if X is None or math.dist(X, Q[i + 2]) > 0.2: return None
        if (X[0] - Q[i - 1][0]) * u[0] + (X[1] - Q[i - 1][1]) * u[1] <= 1e-9 or (Q[i + 3][0] - X[0]) * w[0] + (Q[i + 3][1] - X[1]) * w[1] <= 1e-9: return None
        req = min([C['wmin']] + [cap(Q[j], Q[j + 1]) for j in range(i - 1, i + 3)])
        return Q[:i] + [X] + Q[i + 3:] if frei(Q[i - 1], X, req) and frei(X, Q[i + 3], req) else None
    def ende(Q):
        ""
        if len(Q) < 4: return None
        q, a, e, c = Q[-4], Q[-3], Q[-2], Q[-1]
        d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        a2 = _schnitt(q, d1, c, d2)
        if a2 is None or math.dist(a2, a) > 0.2: return None
        if (a2[0] - q[0]) * d1[0] + (a2[1] - q[1]) * d1[1] <= 1e-9 or (c[0] - a2[0]) * d2[0] + (c[1] - a2[1]) * d2[1] <= 1e-9: return None
        req = min(C['wmin'], cap(q, a), cap(a, e), cap(e, c))
        return Q[:-3] + [a2, c] if frei(q, a2, req) and frei(a2, c, req) else None
    for _ in range(40):
        neu = None
        for i in range(len(P) - 1):
            if not (1e-9 < math.dist(P[i], P[i + 1]) < MINI + 1e-9): continue
            neu = mitte(P, i)
            if neu is None:
                neu = mitte(P[::-1], len(P) - 2 - i); neu = neu[::-1] if neu else None
            if neu is not None: STOSS_STAT['versatz'] += 1; break
            if i == len(P) - 2: neu = ende(P)
            elif i == 0: neu = ende(P[::-1]); neu = neu[::-1] if neu else None
            if neu is not None: STOSS_STAT['versatz_ende'] += 1; break
        if neu is None: break
        P = neu
    return P


def _im_eigenen_pad(li, pt):
    ""
    q = _PT(pt)
    for j in _PADT[li].query(q):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.contains(q) and g2.boundary.distance(q) >= 0.02: return True
    return False


def _aufpreis(li, a, b, ex):
    ""
    L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1); t = np.linspace(0, 1, n)
    ix = ((a[0] + t * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + t * (b[1] - a[1]) - Y0) / RES).astype(int)
    f = np.ones(n, dtype=np.float32) if ex is None else ex[li * NN + iy * NX + ix].astype(np.float32)
    if li == 0: f = np.where(under_esp[iy, ix], f * 1.8, f)
    if IMP_REF_MASK:
        _m = IMP_REF_MASK.get((NET_CUR[0], LAYERS[li]))
        if _m is not None: f = np.where(_m[iy, ix], f, f * 8.0)
    return float((f - 1.0).sum()) * L / n


def path_to_geometry(path, capC, C):
    runs, cur = [], [path[0]]
    for n in path[1:]:
        if n // NN != cur[-1] // NN: runs.append(cur); cur = [n]
        else: cur.append(n)
    runs.append(cur)
    out_tr, out_v = [], []
    for k, run in enumerate(runs):
        li = run[0] // NN
        if C.get('zw'): C = dict(C, w=C['zw'][li], wmin=C['zw'][li])
        if C.get('strom_w'): C = dict(C, w=C['strom_w'][li], wmin=C['strom_w'][li])
        pts = [cell_xy((n % NN) % NX, (n % NN) // NX) for n in run]
        if GS_N:
            _nc = NID.get(NET_CUR[0], -3)
            for _q, _nq in enumerate(run):
                _g = GS_PT.get(_nq)
                if _g is not None and _g[2] != _nc and _g[3] != _nc: pts[_q] = (_g[0], _g[1])
        if VP:
            if k > 0:
                _s = _vp_xy(NET_CUR[0], run[0] % NN)
                if _s: pts[0] = _s
            if k < len(runs) - 1:
                _s = _vp_xy(NET_CUR[0], run[-1] % NN)
                if _s: pts[-1] = _s
        capL = capC[li]
        pc = [float(capL.ravel()[n % NN]) for n in run]


        _r41_pre = None
        if R41_PARAM is not None and C.get('extra') is not None:
            _r41_pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]):
                _r41_pre.append(_r41_pre[-1] + _aufpreis(li, _a, _b, C['extra']))
        def _r41_shortcut(i, j):
            return _r41_pre is None or _aufpreis(li, pts[i], pts[j], C['extra']) <= _r41_pre[j] - _r41_pre[i] + RES
        if ABZW_EXAKT and k == 0 and len(run) == 1 and len(runs) > 1: _via_auf_stamm(li, run[0], pts[0])
        if _HYPO_AN[0] and okt_netz():
            _ex = C.get('extra'); _pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]): _pre.append(_pre[-1] + _aufpreis(li, _a, _b, _ex))
            def _frei(a, b, req):
                return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room_min(li, a, b, NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2 and (not IMP_REF_MASK or z_ref_frei(C, li, a, b))
            def _verbinde(i, j):
                ""
                if j == i + 1: return []
                a, b = pts[i], pts[j]; mn = min(pc[i:j + 1]); req = max(min(C['wmin'], mn), min(C['w'], mn))
                if okt(a, b): return [] if _frei(a, b, req) and _r41_shortcut(i, j) else None
                best = None
                for c in _ecken(a, b):
                    if _frei(a, c, req) and _frei(c, b, req):
                        kk = _aufpreis(li, a, c, _ex) + _aufpreis(li, c, b, _ex)
                        q = min(line_cap_min(capL, a, c), line_cap_min(capL, c, b))
                        if best is None or kk < best[0] - 1e-9 or (abs(kk - best[0]) <= 1e-9 and q > best[1] + 1e-9): best = (kk, q, c)
                if best is None or best[0] > _pre[j] - _pre[i] + 0.05: return None
                return [best[2]]
            P, i, n_ = [pts[0]], 0, len(pts)
            while i < n_ - 1:
                j, zw = n_ - 1, None
                zw = _verbinde(i, j)
                if zw is None:
                    lo, hi, zw = i + 1, j, []
                    while hi - lo > 1:
                        mid = (lo + hi) // 2; r = _verbinde(i, mid)
                        if r is None: hi = mid
                        else: lo, zw = mid, r
                    j = lo
                P.extend(zw); P.append(pts[j]); i = j


            if k == 0 and len(P) >= 3 and math.dist(P[0], P[1]) < 0.1 + 1e-9 and _im_eigenen_pad(li, P[1]): P = P[1:]
            if k == len(runs) - 1 and len(P) >= 3 and math.dist(P[-1], P[-2]) < 0.1 + 1e-9 and _im_eigenen_pad(li, P[-2]): P = P[:-1]
        else:
            simp, i = [0], 0
            while i < len(pts) - 1:
                j_best = i + 1
                for j in range(len(pts) - 1, i + 1, -1):
                    if okt_netz() and not okt(pts[i], pts[j]): continue
                    req = max(min(C['wmin'], min(pc[i:j + 1])), min(C['w'], min(pc[i:j + 1])))
                    if QUELLFREI[0]: req = max(req, max(BAHN_MIN, min(C['wmin'], HALS_W)))
                    if line_cap_min(capL, pts[i], pts[j]) >= req - 1e-6 and (j == i + 1 or pad_room_min(li, pts[i], pts[j], NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2) and (j == i + 1 or not IMP_REF_MASK or z_ref_frei(C, li, pts[i], pts[j])):
                        if j > i + 1 and not _r41_shortcut(i, j): continue
                        j_best = j; break
                simp.append(j_best); i = j_best
            P = [pts[i] for i in simp]
        if PADMITTE and C.get('anker', True):
            if k == len(runs) - 1: P = _padanschluss(P, li, capL, C)
            if k == 0: P = _padanschluss(P[::-1], li, capL, C)[::-1]
        if ABZW_EXAKT and k == 0 and P[0] == pts[0]: P = _abzweig(P, li, run[0], capL, C)
        if VERSATZ and (_HYPO_AN[0] or not okt_netz()): P = _versatz_weg(P, li, capL, C)
        seg_pts, seg_w = [P[0]], []
        for a, b in zip(P[:-1], P[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); m = max(1, int(math.ceil(L / 0.8)))
            for q in range(m):
                pa = (a[0] + (b[0] - a[0]) * q / m, a[1] + (b[1] - a[1]) * q / m)
                pb = (a[0] + (b[0] - a[0]) * (q + 1) / m, a[1] + (b[1] - a[1]) * (q + 1) / m)
                wv = min(C['w'], line_cap_min(capL, pa, pb), 2 * pad_room_min(li, pa, pb, NET_CUR[0], C['w'] / 2))
                _bo = min(C['wmin'], 0.15) if C.get('prio', 8) < 8 else min(C['wmin'], 0.1)
                if C.get('bodenkarte') is not None:
                    _bk = float(C['bodenkarte'][li * NN + int(((pa[1] + pb[1]) / 2 - Y0) / RES) * NX + int(((pa[0] + pb[0]) / 2 - X0) / RES)])
                    if _bk < _bo: _bo = _bk
                wv = max(BAHN_MIN, _bo, math.floor(wv * 40) / 40)
                if C.get('strom_w'): wv = strom_segmentbreite(C,min(line_cap_min(capL,pa,pb), 2*pad_room_min(li,pa,pb,NET_CUR[0],C['w']/2)))
                if C.get('zw'): wv = C['zw'][li]
                if _pz_active(NET_CUR[0]) and NET_CUR[0] in PZ_BREITEN:wv=PZ_BREITEN[NET_CUR[0]]['w']
                seg_pts.append(pb); seg_w.append(wv)
        mp, mw = [seg_pts[0]], []
        for i2, wv in enumerate(seg_w):
            pb = seg_pts[i2 + 1]
            if mw and abs(mw[-1] - wv) < 1e-6 and len(mp) >= 2:
                a, b = mp[-2], mp[-1]
                if abs((b[0] - a[0]) * (pb[1] - b[1]) - (b[1] - a[1]) * (pb[0] - b[0])) < 1e-6: mp[-1] = pb; continue
            mp.append(pb); mw.append(wv)
        if len(mp) >= 2:
            out_tr.append(dict(layer=LAYERS[li], pts=[[round(x, 4), round(y, 4)] for x, y in mp], w=mw))
        if k < len(runs) - 1:
            vx, vy = pts[-1]; out_v.append(dict(x=round(vx, 4), y=round(vy, 4)))
    return out_tr, out_v


SW_ABST = float(args.get('sw_abst', '0.8' if KL else '0'))
_SWN = [n for n in nets if n.split('/')[-1].startswith('SW_')]
TRENN = float(args.get('trenn', '0.5'))
if KL:
    AGG_SW = {n for n, k in KL['netze'].items() if k['art'] == 'sw'}
    AGG_FAST = {n for n, k in KL['netze'].items() if k['art'] == 'schnell' and k['rang'] >= 3}
    VICT_SW = {n for n, k in KL['netze'].items() if k.get('empfindlich_sw')}
    VICT_A = {n for n, k in KL['netze'].items() if k['art'] == 'analog' and k['rang'] >= 2}
    if STOER_FLANKE:
        AGG_SW = set(STOERLISTE['sw'] + STOERLISTE['strom'])
        AGG_FAST = set(STOERLISTE['digital'])
        VICT_SW = VICT_A = set(STOERLISTE['opfer'])
_SPULEN_M = None
if STOER_FLANKE:
    _SPULEN_M = np.zeros(NL * NN, dtype=bool)
    for c in D.get('courtyards', []):
        if c['ref'] in STOERLISTE['spulen'] and c.get('side') in LI:
            li = LI[c['side']]; _SPULEN_M[li * NN:(li + 1) * NN] |= raster_polys(c['polys']).ravel()


def _flanken_gruppen():
    ""
    out = {}
    for n in sorted(AGG_FAST):
        f = STOERLISTE['netze'][n]['abstand_faktor']
        out.setdefault(f, set()).add(n)
    return sorted(out.items())


def _trenn_mm(net):
    return TRENN * STOERLISTE['netze'][net]['abstand_faktor'] if STOER_FLANKE and net in AGG_FAST else TRENN


def _maske(nets):
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'] in nets:
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'] in nets: m[r['cells']] = True
    return m


def _nahe(m, dist, fac):
    ex = np.ones(NL * NN, dtype=np.float32)
    if NACHBARLAGE:
        dl = {}
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): dl[li] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        for li in range(NL):
            nah = dl[li] < dist if li in dl else None
            for lj, s in nachbar_abstand(li, dist):
                if lj in dl: nah = (dl[lj] < s) if nah is None else (nah | (dl[lj] < s))
            if nah is not None: ex[li * NN:(li + 1) * NN] = np.where(nah, fac, 1.0)
        return ex
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < dist, fac, 1.0)
    return ex


_NAHE_C = {}


R41_SPALT = float(args.get('r41_spalt', '0'))
if not math.isfinite(R41_SPALT) or R41_SPALT < 0: raise ValueError('r41_spalt muss endlich und >= 0 sein')
R41_FAST, R41_NON_DC, R41_PARAM = set(), set(), None
if R41_SPALT and KL is not None and NLAG >= 4:
    from analyse.r41 import netze as _r41_netze, parameter as _r41_parameter
    R41_FAST, R41_NON_DC = _r41_netze(KL)
    R41_PARAM = _r41_parameter(KL, dict(args, via='%g/%g' % (VIA_D, VIA_DR)))
    print('R41 Via-Platz (Option):', R41_PARAM, 'Preis', R41_SPALT, flush=True)
_R41_C = {}


def r41_extra(net):
    ""
    if R41_PARAM is None or net not in R41_NON_DC: return None
    nets = (R41_NON_DC if net in R41_FAST else R41_FAST) - {net}
    if net in PAAR_NETZE: nets = nets - {PAAR_NETZE[net]}
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets))
    key = (net, sig)
    if key in _R41_C: return _R41_C[key]

    if len(_R41_C) >= 4: _R41_C.pop(next(iter(_R41_C)))
    ex = np.ones(NL * NN, dtype=np.float32)
    geometries = [[] for _ in range(NL)]
    for unit in U.values():
        if unit['net'] not in nets: continue
        for t in unit['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if a != b: geometries[li].append(_LS([a,b]).buffer(w/2))
    from shapely.ops import unary_union as _r41_union
    for li, gg in enumerate(geometries):
        if not gg: continue
        w = z_breite(net, li, cls(net)['w']) if net in IMP_TAB else cls(net)['w']


        band = _r41_union([g.buffer(w/2 + R41_PARAM['via_platz']).difference(
            g.buffer(w/2 + R41_PARAM['spalt_ab'])) for g in gg])

        x0,y0,x1,y1 = band.bounds if not band.is_empty else (0,0,0,0)
        xa,xb = max(0,int((x0-X0)/RES)),min(NX,int((x1-X0)/RES)+2)
        ya,yb = max(0,int((y0-Y0)/RES)),min(NY,int((y1-Y0)/RES)+2)
        if xa >= xb or ya >= yb: continue
        xx,yy = np.meshgrid(GXc[xa:xb], GYc[ya:yb])
        mask = shapely.contains_xy(band,xx,yy)
        ex[li*NN:(li+1)*NN].reshape(NY,NX)[ya:yb,xa:xb][mask] = 1.0 + R41_SPALT
    _R41_C[key] = ex
    return ex


KUPFER_VORHALT = max(0.0, float(args.get('kupfer_vorhalt', '0'))) if KL else 0.0
_KUPFER_VORHALT_C = {}


def kupfer_vorhalt_extra(net):
    ""
    if not KUPFER_VORHALT:
        return None
    ex = np.ones(NL * NN, dtype=np.float32)
    for n, k in sorted(KL['netze'].items()):
        if n == net or k.get('art') not in ('strom', 'versorgung', 'sw'):
            continue
        ver = _NVER.get(n, 0)
        cached = _KUPFER_VORHALT_C.get(n)
        if cached is None or cached[0] != ver:
            groups = {}
            for r in U.values():
                if r['net'] != n:
                    continue
                for t in r['tracks']:
                    if t.get('sig') or t.get('kelvin'):
                        continue
                    li = LI[t['layer']]
                    goal = cls(n)['w'] * f_innen(n, li)
                    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                        if goal <= w + 1e-6:
                            continue
                        band = math.ceil(((goal - w) / 2 + CLR) / RES) * RES
                        groups.setdefault((li, band), []).append(seg_cells(li, a, b, w))
            maps = []
            for (li, band), cells in sorted(groups.items()):
                idx = np.unique(np.concatenate(cells)) - li * NN
                if not len(idx):
                    continue
                xx, yy = idx % NX, idx // NX
                pad = math.ceil(band / RES) + 2
                xa, xb = max(0, int(xx.min()) - pad), min(NX, int(xx.max()) + pad + 1)
                ya, yb = max(0, int(yy.min()) - pad), min(NY, int(yy.max()) + pad + 1)
                mask = np.zeros((yb - ya, xb - xa), dtype=bool)
                mask[yy - ya, xx - xa] = True
                d = _edt(~mask) * RES
                cost = (1 + KUPFER_VORHALT * np.clip(1 - d / band, 0, 1)).astype(np.float32)
                maps.append((li, xa, xb, ya, yb, cost))
            cached = (ver, maps)
            _KUPFER_VORHALT_C[n] = cached
        for li, xa, xb, ya, yb, cost in cached[1]:
            target = ex[li * NN:(li + 1) * NN].reshape(NY, NX)[ya:yb, xa:xb]
            np.maximum(target, cost, out=target)
    return ex


def _nahe_c(name, nets, dist, fac, koerper=False):
    ""
    key = (name, frozenset(nets), dist, fac, koerper) if STOER_FLANKE else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if STOER_FLANKE else sum(_NVER.get(n, 0) for n in nets)
    c = _NAHE_C.get(key)
    if c is None or c[0] != sig:
        m = _maske(nets)
        if koerper and _SPULEN_M is not None: m |= _SPULEN_M
        c = (sig, _nahe(m, dist, fac)); _NAHE_C[key] = c
    return c[1]

def si_abstand(opfer, stoerer):


    from analyse.signalintegritaet import vorgabe, zahl
    budget = zahl(vorgabe(KL, opfer, 'stoerbudget_mV'), 1.0, 1e-9) / 1000
    swing = zahl(vorgabe(KL, stoerer, 'hub_V'), 12.0 if stoerer in AGG_SW else 3.3)
    hs = [a['H'] for a in AUFBAU if a['H']]
    h = max(hs) if hs else 1.5
    return min(float(args.get('si_abstand_max', '3.0')), h * math.sqrt(max(0.0, swing / budget - 1)))


def si_stoer_extra(net, ex):
    if not SI: return ex
    opfer = {n for n, k in KL['netze'].items() if k.get('art') in ('analog', 'kelvin') or k.get('kelvin')}
    agg = AGG_FAST | AGG_SW
    if net in opfer:
        gruppen = {}
        for ag in sorted(agg):
            d = round(si_abstand(net, ag), 3)
            gruppen.setdefault(d, set()).add(ag)
    elif net in agg:
        gruppen = {}
        for v in sorted(opfer):
            d = round(si_abstand(v, net), 3)
            gruppen.setdefault(d, set()).add(v)
    else: return ex
    for d, ns in sorted(gruppen.items()):
        if d <= 0: continue
        e = _nahe_c(('si', tuple(sorted(ns)), d), ns, d, 4.0)
        ex = e if ex is None else np.maximum(ex, e)
    return ex


def stoer_extra(net):
    ""
    if KL is None: return sw_extra(net)
    ex = None
    if net in VICT_SW and SW_ABST > 0: ex = _nahe_c('sw', AGG_SW - {net} if STOER_FLANKE else AGG_SW, SW_ABST, 6.0, koerper=STOER_FLANKE)
    if net in VICT_A and TRENN > 0:
        if STOER_FLANKE:
            e2 = np.ones(NL * NN, dtype=np.float32)
            for f, ns in _flanken_gruppen(): e2 = np.maximum(e2, _nahe_c(('flanke', f), ns - {net}, TRENN * f, 3.0))
        else: e2 = _nahe_c('schnell', AGG_FAST, TRENN, 3.0)
        ex = e2 if ex is None else ex * e2
    if net in AGG_FAST and TRENN > 0:
        e2 = _nahe_c(('analog', _trenn_mm(net)), VICT_A - {net} if STOER_FLANKE else VICT_A, _trenn_mm(net), 3.0); ex = e2 if ex is None else ex * e2
    if PAAR_NETZE and PAAR_STOER > 1 and net not in PAAR_NETZE:
        e2 = _nahe_c('paar', set(PAAR_NETZE), paar_frei_mm(net), PAAR_STOER); ex = e2 if ex is None else ex * e2
    if KUPFER_VORHALT:
        e2 = kupfer_vorhalt_extra(net)
        ex = e2 if ex is None else ex * e2
    ex = si_stoer_extra(net, ex)
    if ZONENPLAN:
        zex = _zp_karten(net)[1]
        ex = zex if ex is None else ex * zex
    if R41_PARAM is not None:
        e2 = r41_extra(net)
        if e2 is not None: ex = e2 if ex is None else ex * e2
    if ex is not None and STOER_FAK[0] != 1.0:
        ex = np.where(ex > 1.0, ex * np.float32(STOER_FAK[0]), ex).astype(ex.dtype)
    return ex



def paar_einheit(u):
    return PAAR_SUCHRAUM and paar_schnell(u)


def _paar_toleranzen(k):
    ""
    if not PAAR_SUCHRAUM: return k
    nets=k['netze'];seen=set()
    explicit=(k.get('vorgaben') or {}).get('laengengruppen') or {}
    for n,x in nets.items():
        p=x.get('paar')
        if not p or p not in nets or n in seen or not paar_schnell(n):continue
        seen.update((n,p));name='paar:'+'|'.join(sorted((n,p)))
        ts=[nets[q].get('paar_toleranz_mm',None if nets[q].get('laengengruppe') else nets[q].get('toleranz_mm')) for q in (n,p)]
        ts=[float(t) for t in ts if t is not None]
        tol=explicit.get(name,{}).get('toleranz_mm')
        tol=float(tol) if tol is not None else min(ts) if ts else .15
        if not math.isfinite(tol) or tol<=0:raise ValueError('Paar-Skew-Toleranz muss endlich und positiv sein')
        for q in (n,p):nets[q]['paar_toleranz_mm']=tol
    return k


def _paar_fest_pruefen(u):
    ""
    fixed = [n for n in (u, PAAR_NETZE[u]) if 'FIX#' + n in FIXED]
    if fixed:
        raise ValueError('paar_suchraum=1: feste Paarleiter %s; beide Partner frei routen oder das Paar vollstaendig mit fix= vorlegen' % ', '.join(sorted(fixed)))


def paar_netze(reihe):
    ""
    out = set(reihe)
    out.update(PAAR_NETZE[u] for u in reihe if paar_einheit(u))
    return out


def paar_reihe(reihe):
    ""
    if not PAAR_SUCHRAUM: return list(reihe)
    out = []; seen = set()
    for u in reihe:
        if paar_einheit(u): _paar_fest_pruefen(u)
        key = min(u, PAAR_NETZE[u]) if paar_einheit(u) else u
        if key not in seen: seen.add(key); out.append(key)
    return sorted(out, key=lambda u: (0 if paar_einheit(u) else 1, out.index(u)))


def _paar_masse(a, b, li):
    wa, wb = z_breite(a, li, cls(a)['w']), z_breite(b, li, cls(b)['w'])
    gap = max(z_spalt(a, li), z_spalt(b, li))
    if not all(math.isfinite(v) and v > 0 for v in (wa, wb, gap)):
        raise ValueError('Paargeometrie muss endlich und positiv sein')
    return wa, wb, gap, (wa + wb) / 2 + gap, wa + wb + gap


def _paar_padgruppen(a, b):
    ""
    from scipy.optimize import linear_sum_assignment
    aa = [D['pads'][k] for k in bynet.get(a, [])]
    bb = [D['pads'][k] for k in bynet.get(b, [])]
    groups = []
    for ref in sorted({p['ref'] for p in aa + bb}):
        ap = sorted([p for p in aa if p['ref'] == ref], key=lambda p: str(p['num']))
        bp = sorted([p for p in bb if p['ref'] == ref], key=lambda p: str(p['num']))
        if len(ap) != len(bp): return None
        if not ap: continue
        costs = np.array([[math.dist((p['x'],p['y']),(q['x'],q['y'])) for q in bp] for p in ap])
        ii,jj = linear_sum_assignment(costs)
        for i,j in zip(ii,jj):
            p,q = ap[i],bp[j]
            layers = sorted(set(p['layers']) & set(q['layers']) & set(LAYERS), key=LI.get)
            if not layers: return None
            groups.append((p,q,layers))
    return groups if len(groups) >= 2 else None


def _paar_zp_ok(net, li, geom):
    ""
    if not ZONENPLAN: return True
    spec = ZONENPLAN['netze'].get(net, {})
    if spec.get('lokal_ohne_lagenwechsel') and LAYERS[li] != spec.get('lokale_lage'): return False
    return all(geom.distance(g) >= (_ka_clr(net,n,li,'Track','Zone')+.005 if globals().get('KA') is not None else CLR+.005) - 1e-9 for (n, L), g in _ZP_GEO.items() if n != net and L == LAYERS[li])


def _paar_zp_via_ok(net, x, y):
    ""
    if not ZONENPLAN: return True
    ix, iy = math.floor((x - X0) / RES), math.floor((y - Y0) / RES)
    if not (0 <= ix < NX and 0 <= iy < NY): return False
    return bool(_zp_viah(net, np.ones((1, 1), dtype=bool), slice(iy, iy + 1), slice(ix, ix + 1))[0, 0])


def _paar_hart(a, b):
    ""
    maps = []; soft = []
    cnt = use.copy()
    for r in U.values():
        if r['net'] in (a,b): cnt[r['cells']] -= 1
    for li in range(NL):
        heights = [IMP_TAB.get(n,{}).get(LAYERS[li],{}).get('profil',{}).get('h') for n in (a,b)]
        heights += [AUFBAU[li]['h'] if AUFBAU[li]['ref_netz'] in GNDS else None]
        outside = max(KA.maximum if globals().get('KA') is not None else CLR, 3*max(h for h in heights if h) if any(heights) else PAAR_FREI)
        mask = np.zeros((NY,NX), dtype=bool)
        for p in D['pads']:
            if p['net'] not in (a,b) and LAYERS[li] in p['cells']:
                mask.ravel()[p['cells'][LAYERS[li]]] = True
        for u,r in U.items():
            if u.startswith(('FIX#','HEFT#')) and r['net'] not in (a,b):
                cells = r['cells']; cells = cells[cells//NN == li] % NN
                mask.ravel()[cells] = True
        dist = _edt(~mask)*RES if mask.any() else np.full((NY,NX), 1e6)
        h = np.minimum(2*(dist-outside-QM), 2*(edge_d-EDGE))

        for net in (a,b):
            ref = IMP_REF_MASK.get((net,LAYERS[li]))
            if ref is not None:
                rd = _edt(ref)*RES
                h = np.minimum(h,2*(rd-QM))

        if ZONENPLAN: h = _zp_hart(b, _zp_hart(a, h, li), li)
        maps.append(h.astype(np.float32))
        occupied = cnt[li*NN:(li+1)*NN].reshape(NY,NX)>0
        d = _edt(~occupied)*RES if occupied.any() else np.full((NY,NX),1e6)
        soft.append((2*(d-outside-QM)).astype(np.float32))
    if HART_FREMD[0]: maps = [np.minimum(h,s) for h,s in zip(maps,soft)]
    return maps, soft


def _paar_linie_ok(net, li, pts, w, other=None):
    if globals().get('KA') is not None:
        if any(_ka_room(net,li,a,b,True)<w/2 for a,b in zip(pts[:-1],pts[1:])):return False
        if other is not None and _LS(pts).buffer(w/2).distance(other)<_ka_margin(net,PAAR_NETZE.get(net,net),li):return False
    line = _LS(pts)
    if not line.is_simple: return False
    copper = line.buffer(w/2)
    board = rand_polygon(D).buffer(-float(args.get('edge','0.3')))
    if not board.covers(copper): return False
    for nn,g in _PADG[li]:
        if nn != net and copper.distance(g) < (_ka_geo_limit(net,nn,li,g2=g,pad=True) if globals().get('KA') is not None else DRC_PADC)-1e-6: return False
    for u,r in U.items():
        if r['net'] == net: continue
        if not u.startswith(('FIX#','HEFT#')) and not HART_FREMD[0]: continue
        for t in r['tracks']:
            if LI[t['layer']] != li: continue
            for x,y,ww in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                if copper.distance(_LS([x,y]).buffer(ww/2)) < (_ka_margin(net,r['net'],li) if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
        for v in r['vias']:
            if copper.distance(Point(v['x'],v['y']).buffer(VIA_R)) < (_ka_margin(net,v.get('net') or r['net'],li,typ2='Via') if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
    if net in IMP_TAB and any(not z_ref_frei(cls(net),li,x,y,net) for x,y in zip(pts[:-1],pts[1:])): return False
    if globals().get('KA') is None and other is not None and copper.distance(other) < DRC_CLR-1e-6: return False
    if not _paar_zp_ok(net, li, copper): return False
    return True


def _paar_pad_flucht(net,pad,end,li,w,direction,other=None):
    ""
    if zeit_knapp():return None
    sx,sy=pad['x'],pad['y']
    xa=max(1,int((min(sx,end[0])-.8-X0)/RES));xb=min(NX-2,int((max(sx,end[0])+.8-X0)/RES))
    ya=max(1,int((min(sy,end[1])-.8-Y0)/RES));yb=min(NY-2,int((max(sy,end[1])+.8-Y0)/RES))
    nx,ny=xb-xa+1,yb-ya+1
    if nx<2 or ny<2:return None
    cap=hard_maps(net)[0][li][ya:yb+1,xa:xb+1].copy()
    yy,xx=np.indices((ny,nx));px=X0+(xx+xa+.5)*RES;py=Y0+(yy+ya+.5)*RES
    foreign=shapely.union_all([g for nn,g in _PADG[li] if nn!=net])
    if not foreign.is_empty:
        exact=shapely.distance(shapely.points(px,py),foreign)
        cap=np.minimum(cap,2*(exact-DRC_PADC-QM))
    if other is not None and not other.is_empty:
        distance=shapely.distance(shapely.points(px,py),other)
        cap=np.minimum(cap,2*(distance-DRC_CLR-.005))
    ix,iy=int((sx-X0)/RES)-xa,int((sy-Y0)/RES)-ya
    tx,ty=int((end[0]-X0)/RES)-xa,int((end[1]-Y0)/RES)-ya
    if not (0<=ix<nx and 0<=iy<ny and 0<=tx<nx and 0<=ty<ny):return None
    dirs=np.array(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)),np.int64)
    goals=np.zeros(nx*ny,np.int64)
    for j,(dx,dy) in enumerate(dirs):
        if (dx*direction[0]+dy*direction[1])/math.hypot(dx,dy)>=math.sqrt(.5)-1e-7:goals[ty*nx+tx]|=1<<j
    rest=(RES*np.hypot(xx-tx,yy-ty)).ravel()
    path,count=_PAAR_KERN(np.full(8,iy*nx+ix,np.int64),np.arange(8,dtype=np.int64),np.zeros(8),goals,
        cap.reshape(1,-1),np.ones((1,nx*ny)),np.zeros((8,nx*ny),np.bool_),np.zeros(nx*ny,np.bool_),np.array([w]),dirs,
        nx,ny,RES,0,0,nx-1,ny-1,0,np.ones(8,np.int64),12.,15000,rest,np.ones((1,8,nx*ny),np.bool_))
    if not len(path):return None
    raster=[(X0+(xa+n%nx+.5)*RES,Y0+(ya+n//nx+.5)*RES) for n in path]
    own=shapely.union_all([Polygon(poly) for poly in pad.get('polys',{}).get(LAYERS[li],[])])
    while raster and not own.is_empty and own.covers(Point(*raster[0])):raster.pop(0)
    pts=[(sx,sy)]+raster+[end]
    pts=_okt_zug(pts)

    out=[pts[0]]
    for j in range(1,len(pts)-1):
        a,b,c=pts[j-1:j+2]
        if abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))>1e-8:out.append(b)
    out.append(pts[-1]);out=[list(p) for j,p in enumerate(out) if j==0 or math.dist(p,out[j-1])>1e-8]
    if len(out)<2:return None
    if not _paar_linie_ok(net,li,out,w,other):return None
    return out


def _paar_anschluss(p, q, li, center, direction, side, a, b, copper=None):
    ""
    wa,wb,gap,d,total = _paar_masse(a,b,li)
    norm = (-direction[1],direction[0])
    foreign=[None,None] if copper is None else [copper[1][li],copper[0][li]]
    ends = [(center[0]+side*(wb+gap)/2*norm[0],center[1]+side*(wb+gap)/2*norm[1]),
            (center[0]-side*(wa+gap)/2*norm[0],center[1]-side*(wa+gap)/2*norm[1])]
    def paths(pad,end):
        start = (pad['x'],pad['y'])
        choices = [[start,end]] if okt(start,end) else [[start,k,end] for k in _ecken(start,end)]

        choices=[pp for pp in choices if math.dist(pp[-2],pp[-1])<1e-8 or (pp[-1][0]-pp[-2][0])*direction[0]+(pp[-1][1]-pp[-2][1])*direction[1]>=-1e-8]
        return [[list(x) for i,x in enumerate(pp) if i == 0 or math.dist(x,pp[i-1])>1e-8] for pp in choices]
    for pa in paths(p,ends[0]):
        if len(pa)<2: pa=[pa[0],pa[0]]
        if not _paar_linie_ok(a,li,pa,wa,foreign[0]): continue
        for pb in paths(q,ends[1]):
            if len(pb)<2: pb=[pb[0],pb[0]]
            if _paar_linie_ok(b,li,pb,wb,shapely.union_all([_LS(pa).buffer(wa/2)]+([foreign[1]] if foreign[1] is not None else []))):
                return [dict(layer=LAYERS[li],pts=pa,w=[wa]*(len(pa)-1)),
                        dict(layer=LAYERS[li],pts=pb,w=[wb]*(len(pb)-1))]


    for first in (0,1):
        pads=(p,q);nets=(a,b);widths=(wa,wb);second=1-first
        paths=[None,None]
        paths[first]=_paar_pad_flucht(nets[first],pads[first],ends[first],li,widths[first],direction,foreign[first])
        if paths[first] is None:continue
        paths[second]=_paar_pad_flucht(nets[second],pads[second],ends[second],li,widths[second],direction,shapely.union_all([_LS(paths[first]).buffer(widths[first]/2)]+([foreign[second]] if foreign[second] is not None else [])))
        if paths[second] is not None:
            return [dict(layer=LAYERS[li],pts=pp,w=[widths[k]]*(len(pp)-1)) for k,pp in enumerate(paths)]
    return None


def _paar_ports(group, a, b, hard, side, copper=None):
    p,q,layers = group
    middle = ((p['x']+q['x'])/2,(p['y']+q['y'])/2)
    ports = {}
    for layer in layers:
        li = LI[layer]; _,_,_,_,width = _paar_masse(a,b,li)
        for dx,dy,_ in OFFS:
            length = math.hypot(dx,dy); direction = (dx/length,dy/length)

            for step in (0.,.2,.4,.6,.8,1.0,1.4,2.0,2.6,3.2,4.0):
                c = (middle[0]+step*direction[0],middle[1]+step*direction[1])
                ix,iy = int((c[0]-X0)/RES),int((c[1]-Y0)/RES)
                if not (1<=ix<NX-1 and 1<=iy<NY-1) or hard[li][iy,ix]<width: continue
                center = cell_xy(ix,iy)
                links = _paar_anschluss(p,q,li,center,direction,side,a,b,copper)
                if links is not None:
                    node = li*NN+iy*NX+ix
                    cost = sum(math.dist(x,y) for t in links for x,y in zip(t['pts'][:-1],t['pts'][1:]))
                    ports.setdefault(node,[]).append((cost,links,direction))
    if not ports: return ports
    shortest = min(v[0] for vv in ports.values() for v in vv)
    return {n:[v for v in vv if v[0]<=shortest+.3] for n,vv in ports.items() if any(v[0]<=shortest+.3 for v in vv)}



def _paar_zustaende(a,b,side,tracks,vias):
    ""
    yy,xx=np.indices((NY,NX));px=(X0+(xx+.5)*RES).ravel();py=(Y0+(yy+.5)*RES).ravel()
    copper=[[None]*NL for _ in range(2)];allowed=np.ones((NL,8,NN),np.bool_)
    for k,net in enumerate((a,b)):
        for li in range(NL):
            gg=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            gg += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg += [g for nn,g in _PADG[li] if nn==net]
            copper[k][li]=shapely.union_all(gg)
    for li in range(NL):
        wa,wb,gap,_,_=_paar_masse(a,b,li)
        for k,sign,offset,w in ((0,side,(wb+gap)/2,wa),(1,-side,(wa+gap)/2,wb)):
            other=copper[1-k][li]
            if other.is_empty:continue


            near=np.flatnonzero(shapely.contains_xy(other.buffer(offset+w/2+DRC_CLR+.01),px,py))
            for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
                norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy))
                positions=shapely.points(px[near]+sign*offset*norm[0],py[near]+sign*offset*norm[1])
                allowed[li,j,near] &= shapely.distance(positions,other)>=w/2+DRC_CLR-1e-7
    return allowed,copper


def _paar_baum_frei(tracks,vias):
    for li in range(NL):
        gg=[]
        for k in range(2):
            shapes=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            shapes += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg.append(shapely.union_all(shapes))
        if gg[0].distance(gg[1])<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:return False
        for k in range(2):
            for v in vias[k]:
                if Point(v['x'], v['y']).distance(gg[1-k]) < VIA_DR/2 + LOCH_CLR - 1e-6:
                    return False
    allvias = [(k, v) for k in range(2) for v in vias[k]]
    for i, (k, v) in enumerate(allvias):
        for qk, q in allvias[i+1:]:
            dist=math.dist((v['x'],v['y']),(q['x'],q['y']))
            if k == qk and dist <= 1e-8: continue
            if dist < VIA_DR+H2H+.012: return False
    return True


def _paar_via_mask(a,b,hard,side=1,copper=None):
    ""
    distance=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+LOCH_CLR+.012,max(_paar_masse(a,b,li)[3] for li in range(NL)))
    yy,xx=np.indices((NY,NX));out=np.zeros((8,NN),np.bool_)

    zp=[_zp_viah(n,np.ones((NY,NX),np.bool_)) for n in (a,b)] if ZONENPLAN else None
    for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
        norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy));ok=np.ones((NY,NX),np.bool_)
        for sign in (-1,1):
            ix=np.rint(xx+sign*distance/2/RES*norm[0]).astype(int);iy=np.rint(yy+sign*distance/2/RES*norm[1]).astype(int)
            inside=(ix>=0)&(ix<NX)&(iy>=0)&(iy<NY);ix=np.clip(ix,0,NX-1);iy=np.clip(iy,0,NY-1)
            ok &= inside & (VSP[iy,ix]>=H2H+VIA_DR/2+.01)
            if VIA_VERBOT is not None:ok &= ~VIA_VERBOT[iy,ix]
            for h in hard:ok &= h[iy,ix]>=2*VIA_KEEP_R
            k=0 if sign==side else 1
            if zp is not None:ok &= zp[k][iy,ix]
            if copper is not None:
                px=X0+(xx+.5)*RES;py=Y0+(yy+.5)*RES
                for li in range(NL):
                    other=copper[1-k][li]
                    if other.is_empty:continue
                    near=shapely.contains_xy(other.buffer(distance/2+VIA_R+max(DRC_CLR,DRC_PADC)+.01),px,py)&ok
                    rows,cols=np.nonzero(near)
                    points=shapely.points(px[rows,cols]+sign*distance/2*norm[0],py[rows,cols]+sign*distance/2*norm[1])
                    ok[rows,cols] &= shapely.distance(points,other)>=VIA_R+max(DRC_CLR,DRC_PADC)
        out[j]=ok.ravel()
    return out


def _paar_restkosten(tgt,hard,widths,mult,viah,nx,ny,res,xmin,ymin,xmax,ymax,via_cost):
    ""
    nl=hard.shape[0];nn=nx*ny
    out=np.full(nl*nn,np.inf);queue=[(0.,np.int64(0))];queue.pop()
    for n in range(len(tgt)):
        if tgt[n]:out[n]=0.;heapq.heappush(queue,(0.,np.int64(n)))
    dxs=(1,1,0,-1,-1,-1,0,1);dys=(0,1,1,1,0,-1,-1,-1)
    while queue:
        cost,raw=heapq.heappop(queue);n=int(raw)
        if cost>out[n]+1e-10:continue
        li=n//nn;idx=n%nn;iy=idx//nx;ix=idx%nx
        for j in range(8):
            x=ix+dxs[j];y=iy+dys[j]
            if x<xmin or x>xmax or y<ymin or y>ymax:continue
            ii=y*nx+x;m=li*nn+ii
            if hard[li,ii]<widths[li]:continue
            val=cost+res*math.hypot(dxs[j],dys[j])*mult[li,idx]
            if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))


        if np.any(viah[:,idx]):
            for lj in range(nl):
                if lj==li or hard[lj,idx]<widths[lj]:continue
                m=lj*nn+idx;val=cost+via_cost
                if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))
    return out


def _paar_suche_kern(src_nodes, src_dirs, src_cost, tgt_dirs, hard, mult, viah, viabad, widths,
                      offsets, nx, ny, res, xmin, ymin, xmax, ymax, leadcap, via_steps, via_cost, maxexpand, rest, allowed, require_via=False):
    ""
    layers=hard.shape[0];nn=nx*ny;wx=xmax-xmin+1;wy=ymax-ymin+1;wn=wx*wy
    base_stride=8*(leadcap+1);stride=base_stride*(2 if require_via else 1)


    g={np.int64(-1):np.float64(np.inf)}
    parent={np.int64(-1):np.int64(-2)};vcenter={np.int64(-1):np.int64(-1)}
    closed={np.int64(-1):np.int64(0)}
    queue=[(0.0,np.int64(0))];queue.pop()
    if not np.any(tgt_dirs):return np.empty(0,np.int64),0
    for i in range(len(src_nodes)):
        n=src_nodes[i];li=n//nn;iy=n%nn//nx;ix=n%nn%nx
        if ix<xmin or ix>xmax or iy<ymin or iy>ymax:continue
        if not allowed[li,src_dirs[i],n%nn]:continue
        local=li*wn+(iy-ymin)*wx+ix-xmin
        state=local*stride+src_dirs[i]*(leadcap+1)
        if state not in g or src_cost[i]<g[state]:
            g[state]=src_cost[i];parent[state]=-1
            heapq.heappush(queue,(src_cost[i]+rest[n],state))
    expanded=0;goal=-1
    while queue:
        _,rawstate=heapq.heappop(queue);state=int(rawstate)
        if state in closed:continue
        closed[state]=1;expanded+=1
        if expanded>maxexpand:break
        local=state//stride;phase=(state%stride)//base_stride;direction=(state%base_stride)//(leadcap+1);lead=state%(leadcap+1)
        li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin;n=li*nn+iy*nx+ix
        if (not require_via or phase==1) and lead>=leadcap and tgt_dirs[n] & (1<<direction):goal=state;break
        for turn in (-1,0,1):
            if turn and lead<leadcap:continue
            nd=(direction+turn)%8;dx=int(offsets[nd,0]);dy=int(offsets[nd,1]);jx,jy=ix+dx,iy+dy
            if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
            nnidx=jy*nx+jx
            if hard[li,nnidx]<widths[li] or not allowed[li,nd,n%nn] or not allowed[li,nd,nnidx]:continue
            nl=min(leadcap,lead+1) if turn==0 else 0
            dest=(li*wn+(jy-ymin)*wx+jx-xmin)*stride+phase*base_stride+nd*(leadcap+1)+nl
            cost=g[state]+res*math.hypot(dx,dy)*mult[li,nnidx]+(.025 if turn else 0.)
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=-1
            heapq.heappush(queue,(cost+rest[li*nn+nnidx],dest))
        if lead<leadcap:continue
        dx=int(offsets[direction,0]);dy=int(offsets[direction,1]);vs=int(via_steps[direction])
        cx,cy=ix+vs*dx,iy+vs*dy;jx,jy=ix+2*vs*dx,iy+2*vs*dy
        if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
        ci=cy*nx+cx;end=jy*nx+jx
        if not viah[direction,ci]:continue
        for lj in range(layers):
            if lj==li:continue
            free=True;cost=g[state]+via_cost*(1.+3.*viabad[ci])
            for step in range(1,2*vs+1):
                ii=(iy+step*dy)*nx+ix+step*dx;ll=li if step<=vs else lj
                if hard[ll,ii]<widths[ll] or not allowed[ll,direction,ii]:free=False;break
                cost+=res*math.hypot(dx,dy)*mult[ll,ii]
            if not free:continue
            dest=(lj*wn+(jy-ymin)*wx+jx-xmin)*stride+(base_stride if require_via else 0)+direction*(leadcap+1)+leadcap
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=ci
            heapq.heappush(queue,(cost+rest[lj*nn+end],dest))
    if goal<0:return np.empty(0,np.int64),expanded
    reverse=[];state=goal
    while state>=0:
        local=state//stride;li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin
        reverse.append(li*nn+iy*nx+ix)
        prev=parent[state]
        if state in vcenter and vcenter[state]>=0 and prev>=0:
            pli=prev//stride//wn
            reverse.append(li*nn+vcenter[state]);reverse.append(pli*nn+vcenter[state])
        state=prev
    return np.asarray(reverse[::-1],dtype=np.int64),expanded


_PAAR_REST = _paar_restkosten
_PAAR_KERN = _paar_suche_kern
if NB is not None:
    from numba import njit as _paar_njit
    _PAAR_REST = _paar_njit(cache=True)(_paar_restkosten)
    _PAAR_KERN = _paar_njit(cache=True)(_paar_suche_kern)


def _paar_suche(src,tgt,hard,soft,viaH,viaBad,C):
    directions=np.array([(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)],dtype=np.int64)
    def heading(d):return min(range(8),key=lambda i: abs(d[0]-directions[i,0]/math.hypot(*directions[i]))+abs(d[1]-directions[i,1]/math.hypot(*directions[i])))
    entries=[(n,v) for n in sorted(src) for v in src[n]]
    sn=np.array([n for n,v in entries],dtype=np.int64);sd=np.array([heading(v[2]) for n,v in entries],dtype=np.int64)
    sc=np.array([v[0]/2 for n,v in entries],dtype=np.float64)
    td=np.zeros(NL*NN,dtype=np.int64)
    for n,vv in tgt.items():
        for v in vv: td[n] |= 1<<((heading(v[2])+4)%8)
    h=np.asarray([v.ravel() for v in hard]);mult=[]
    for li in range(NL):
        c=np.minimum(hard[li],soft[li]);width=C['zw'][li]
        m=(1.+PF[0]*(c<width))*(1.+hist[li*NN:(li+1)*NN].reshape(NY,NX))*C['lf'][li]
        if C.get('paar_baum') is not None: m=np.where(C['paar_baum'][li*NN:(li+1)*NN].reshape(NY,NX),m*.2,m)
        mult.append(m.ravel())
    mult=np.asarray(mult,dtype=np.float64)
    step=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+LOCH_CLR+.012,max(_paar_masse(NET_CUR[0],PAAR_NETZE[NET_CUR[0]],li)[3] for li in range(NL)),.6)
    vs=np.array([math.ceil((step+RES)/RES/math.hypot(*v)) for v in directions],dtype=np.int64)
    lead=max(2,math.ceil(max(C['zw'])*math.tan(math.pi/8)/RES)+1)
    xs=[n%NN%NX for n in list(src)+list(tgt)];ys=[n%NN//NX for n in list(src)+list(tgt)]
    for margin in (3.,8.,20.):
        if zeit_knapp():return None
        pad=math.ceil(margin/RES)
        xa,ya,xb,yb=max(1,min(xs)-pad),max(1,min(ys)-pad),min(NX-2,max(xs)+pad),min(NY-2,max(ys)+pad)
        states=NL*(xb-xa+1)*(yb-ya+1)*8*(lead+1)*(2 if C.get('paar_via_erzwungen',False) else 1)
        state_limit=int(args.get('paar_zustaende_max','0'))
        if state_limit>0 and states>state_limit:continue
        rest=_PAAR_REST(td,h,np.array(C['zw']),mult,viaH,NX,NY,RES,xa,ya,xb,yb,C['via'])
        if zeit_knapp():return None
        if not any(math.isfinite(rest[n]) for n in sn):continue
        path,count=_PAAR_KERN(sn,sd,sc,td,h,mult,viaH,viaBad.ravel(),np.array(C['zw']),directions,NX,NY,RES,
            xa,ya,xb,yb,lead,vs,C['via'],int(args.get('paar_expansionen','1000000')),rest,C['paar_zulaessig'],C.get('paar_via_erzwungen',False))
        if len(path):return [int(n) for n in path]
    return None


def _paar_center(path, hard, widths):
    ""
    runs = []; cur = [path[0]]
    for n in path[1:]:
        if n//NN != cur[-1]//NN: runs.append(cur); cur=[n]
        else: cur.append(n)
    runs.append(cur); result=[]
    for run in runs:
        li=run[0]//NN
        pts=[cell_xy(n%NN%NX,n%NN//NX) for n in run]


        out=[pts[0]]
        for j in range(1,len(pts)-1):
            x,y,z=pts[j-1:j+2]
            if abs((y[0]-x[0])*(z[1]-y[1])-(y[1]-x[1])*(z[0]-y[0]))>1e-8 or (y[0]-x[0])*(z[0]-y[0])+(y[1]-x[1])*(z[1]-y[1])<=0:
                out.append(y)
        out.append(pts[-1])
        result.append((li,out))
    return result


def _paar_via_muster(a,b,li,lj,center,direction,side):
    ""
    wa,wb,gap0,d0,_=_paar_masse(a,b,li); w1,w2,gap1,d1,_=_paar_masse(a,b,lj)
    distance=max(d0,d1,VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+LOCH_CLR+.012)
    normal=(-direction[1],direction[0]); reach=max(distance, .6)
    pair=[]
    for net,sign,w0,wnew,offset0,offset1 in ((a,side,wa,w1,(wb+gap0)/2,(w2+gap1)/2),(b,-side,wb,w2,(wa+gap0)/2,(w1+gap1)/2)):
        v=(center[0]+sign*distance/2*normal[0],center[1]+sign*distance/2*normal[1])
        if not _paar_vias_erlaubt([dict(x=v[0],y=v[1])]): return None
        if not _paar_zp_via_ok(net,*v) or not all(_paar_zp_ok(net,ll,Point(*v).buffer(VIA_R)) for ll in range(NL)): return None
        board=rand_polygon(D).buffer(-float(args.get('edge','0.3')))
        if not board.covers(Point(*v).buffer(VIA_R)): return None
        for ll in range(NL):
            for nn,g in _PADG[ll]:
                if nn!=net and Point(*v).distance(g)<max(VIA_R+DRC_PADC,VIA_DR/2+LOCH_CLR)+.006: return None
        start=(center[0]-reach*direction[0]+sign*offset0*normal[0],center[1]-reach*direction[1]+sign*offset0*normal[1])
        end=(center[0]+reach*direction[0]+sign*offset1*normal[0],center[1]+reach*direction[1]+sign*offset1*normal[1])
        pp=_okt_zug([start,v]); qq=_okt_zug([v,end])
        if not _paar_linie_ok(net,li,pp,w0) or not _paar_linie_ok(net,lj,qq,wnew): return None
        pair.append(dict(via=dict(x=v[0],y=v[1]),vor=pp,nach=qq,reach=reach))
    for ll,key in ((li,'vor'),(lj,'nach')):
        w0,w1=_paar_masse(a,b,ll)[:2]
        g0=_LS(pair[0][key]).buffer(w0/2);g1=_LS(pair[1][key]).buffer(w1/2)
        if g0.distance(g1)<DRC_CLR-1e-6: return None
        for i,g in enumerate((g0,g1)):
            v=pair[1-i]['via']
            if g.distance(Point(v['x'],v['y']).buffer(VIA_R))<DRC_CLR-1e-6: return None
    return pair


_PAAR_SPLIT_GRUND = [None]


def _paar_split(runs, a, b, side, start, end, copper=None):
    ""
    out=[[],[]]; vias=[[],[]]
    offsets=[]
    _PAAR_SPLIT_GRUND[0]='Kanal-Aufteilung'
    for li,pts in runs:
        if len(pts)<2: _PAAR_SPLIT_GRUND[0]='Zu kurze Lagenstrecke'; return None
        wa,wb,gap,d,_=_paar_masse(a,b,li)
        lane=[]
        for sign,offset in ((side,(wb+gap)/2),(-side,(wa+gap)/2)):
            line=_LS(pts).offset_curve(sign*offset,join_style=2,mitre_limit=3)



            if line.geom_type=='MultiLineString': line=shapely.line_merge(line,directed=True)
            if line.geom_type!='LineString' or line.is_empty or not line.is_simple:
                _PAAR_SPLIT_GRUND[0]='Offset unzulaessig: '+str((line.geom_type,li,pts));return None
            lane.append([list(p) for p in line.coords])
        offsets.append(lane)
    for i in range(len(runs)-1):
        li,pp=runs[i]; lj,qq=runs[i+1]
        dx,dy=pp[-1][0]-pp[-2][0],pp[-1][1]-pp[-2][1]; ll=math.hypot(dx,dy)
        direction=(dx/ll,dy/ll)

        if math.dist(qq[0],pp[-1])>1e-6: return None
        if (qq[1][0]-qq[0][0])*direction[0]+(qq[1][1]-qq[0][1])*direction[1] < math.dist(qq[0],qq[1])-.00001: return None
        pattern=_paar_via_muster(a,b,li,lj,pp[-1],direction,side)
        if pattern is None or math.dist(pp[-2],pp[-1])<=pattern[0]['reach'] or math.dist(qq[0],qq[1])<=pattern[0]['reach']:
            _PAAR_SPLIT_GRUND[0]='Via-Muster oder Gerade davor/danach: '+str((li,lj,pp,qq));return None
        for k in range(2):
            offsets[i][k]=offsets[i][k][:-1]+[list(p) for p in pattern[k]['vor']]
            offsets[i+1][k]=[list(p) for p in pattern[k]['nach']]+offsets[i+1][k][1:]
            vias[k].append(pattern[k]['via'])
    for i,(li,_) in enumerate(runs):
        for k,net in enumerate((a,b)):
            pts=offsets[i][k]
            if i==0: pts=start[k]['pts'][:-1]+pts
            if i==len(runs)-1: pts=pts+end[k]['pts'][-2::-1]
            pts=[p for j,p in enumerate(pts) if j==0 or math.dist(p,pts[j-1])>1e-8]
            w=_paar_masse(a,b,li)[k]
            if len(pts)<2 or not all(okt(x,y) for x,y in zip(pts[:-1],pts[1:])) or not _paar_linie_ok(net,li,pts,w,None if copper is None else copper[1-k][li]):
                _PAAR_SPLIT_GRUND[0]='Leiter-Aufteilung unzulaessig: '+str((net,li,pts));return None
            out[k].append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),paar_suchraum=True))
        ga=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[0] if LI[t['layer']]==li])
        gb=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[1] if LI[t['layer']]==li])
        if ga.distance(gb)<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:
            _PAAR_SPLIT_GRUND[0]='Paarabstand bei Aufteilung';return None
    return out,vias


def _paar_baum_geometrie(tracks,vias):
    ""
    result=[[],[]];vv=[[],[]]
    for k in range(2):
        for layer in LAYERS:
            tt=[t for t in tracks[k] if t['layer']==layer]
            if not tt:continue
            w=tt[0]['w'][0]
            assert all(all(abs(x-w)<1e-9 for x in t['w']) for t in tt)
            geom=shapely.line_merge(shapely.union_all([_LS(t['pts']) for t in tt]),directed=True)
            lines=[geom] if geom.geom_type=='LineString' else list(geom.geoms)
            for ln in lines:
                pts=[list(p) for p in ln.coords]
                if len(pts)>1:result[k].append(dict(layer=layer,pts=pts,w=[w]*(len(pts)-1),paar_suchraum=True))
        seen=set()
        for v in vias[k]:
            key=(round(v['x'],8),round(v['y'],8))
            if key not in seen:seen.add(key);vv[k].append(v)
    return result,vv


_PAAR_SUCH_INFO = {}

def _paar_planar_sperren(C, path, src, tgt):
    ""
    candidates = [i for i in range(1, len(path)-1) if path[i] not in src and path[i] not in tgt
                  and not C['paar_baum'][path[i]]]
    if not candidates: return False
    def rank(i):
        x,y,z = path[i-1:i+2]
        dx,dy = y%NN%NX-x%NN%NX, y%NN//NX-x%NN//NX
        ex,ey = z%NN%NX-y%NN%NX, z%NN//NX-y%NN//NX
        return (0 if dx*ey-dy*ex else 1, abs(i-(len(path)-1)/2), i)
    n = path[min(candidates, key=rank)]
    C['paar_zulaessig'][n//NN, :, n%NN] = False
    return True


def _paar_kante(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,source,target):
    ""
    allowed = C['paar_zulaessig']; details = []; rejected = False
    forced = C.get('paar_via_erzwungen', False)
    try:
        for with_vias in (False, True):
            C['paar_zulaessig'] = allowed.copy() if not with_vias else allowed
            attempts = 1 if with_vias else max(1, int(args.get('paar_planar_versuche', '3')))
            if not with_vias and not ({n//NN for n in src}&{n//NN for n in tgt}): continue
            for attempt in range(attempts):
                if zeit_knapp(): return None, None, details
                ASTAR_COST[0] = None
                C['paar_via_erzwungen'] = with_vias and rejected
                path = _paar_suche(src,tgt,hard,soft,viaH if with_vias else np.zeros_like(viaH),viaBad,C)
                if path is None or len(path)<2:
                    details.append('Kein Pfad '+str((with_vias,attempt,len(src),len(tgt))))
                    break
                runs = _paar_center(path,hard,widths)
                l0,p0 = runs[0]; l1,p1 = runs[-1]; found = None
                if len(p0)>=2 and len(p1)>=2:
                    def direct(x,y):
                        d=math.dist(x,y); return ((y[0]-x[0])/d,(y[1]-x[1])/d)
                    start = _paar_anschluss(*source[:2],l0,p0[0],direct(p0[0],p0[1]),side,a,b,copper)
                    end = _paar_anschluss(*target[:2],l1,p1[-1],direct(p1[-1],p1[-2]),-side,a,b,copper)
                    if start is not None and end is not None:
                        found = _paar_split(runs,a,b,side,start,end,copper)
                        if found is not None and not _paar_baum_frei([alltr[k]+found[0][k] for k in range(2)], [allvs[k]+found[1][k] for k in range(2)]):
                            found = None; _PAAR_SPLIT_GRUND[0] = 'Paarast verletzt bisherigen Stamm'
                    else: _PAAR_SPLIT_GRUND[0] = 'Padport-Richtung'
                else: _PAAR_SPLIT_GRUND[0] = 'Zu kurze Lagenstrecke'
                if found is not None:
                    if not with_vias and attempt: C['paar_planar_ersatz'] = C.get('paar_planar_ersatz', 0)+1
                    return found, path, details
                details.append(('Via' if with_vias else 'Planar')+' '+str(attempt)+': '+str(_PAAR_SPLIT_GRUND[0]))
                if with_vias: break
                rejected = True
                C['paar_planar_verworfen'] = C.get('paar_planar_verworfen', 0)+1
                if attempt+1<attempts and not _paar_planar_sperren(C,path,src,tgt): break
        return None, None, details
    finally:
        C['paar_zulaessig'] = allowed
        C['paar_via_erzwungen'] = forced


def route_paar(u):
    ""
    _paar_fest_pruefen(u)
    a,b=sorted((u,PAAR_NETZE[u])); old={n:U.get(n) for n in (a,b)}
    anchors=copy.deepcopy(VIA_ALT); success=False
    for n in (a,b): del_unit(n)
    try:
        groups=_paar_padgruppen(a,b)
        if groups is None: _PAAR_SUCH_INFO[a]=dict(grund='Padpaar-Zuordnung unbekannt'); return False
        hard,soft=_paar_hart(a,b); widths=tuple(_paar_masse(a,b,li)[4] for li in range(NL))

        C=dict(cls(a),w=max(widths),wmin=min(widths),zw=widths,via=max(12.,2*cls(a)['via']*VIA_FAK[0]),inner=1.,pen=3.)
        C['lf']=paar_lf(1.) or tuple(1. if li in (0,NL-1) else 4. for li in range(NL))
        NET_CUR[0]=a; ASTAR_COST[0]=None; ASTAR_HINT[0]=None; VIA_ZU[0]=None

        viaH=_paar_via_mask(a,b,hard)
        viaBad=np.logical_or.reduce([v<VIA_D for v in soft])


        def root_key(i):
            p,q,_=groups[i]; ref=p['ref']
            driver=all(x.get('pintype') in ('output','tri_state') for x in (p,q))
            chip=ref.startswith(('U','IC')) and sum(g[0]['ref']==ref for g in groups)==1 and all(x.get('pintype') in ('output','tri_state','bidirectional') for x in (p,q))
            return (0 if driver else 1 if chip else 2,ref,str(p['num']))
        root=min(range(len(groups)),key=root_key)
        def middle(g):return ((g[0]['x']+g[1]['x'])/2,(g[0]['y']+g[1]['y'])/2)
        edges=[(root,j) for j in sorted((j for j in range(len(groups)) if j!=root),key=lambda j:(math.dist(middle(groups[root]),middle(groups[j])),j))]
        C['paar_baum']=np.zeros(NL*NN,dtype=np.bool_)
        alltr=[[],[]]; allvs=[[],[]]
        for i,j in edges:
            found=None;details=[]
            dx,dy=middle(groups[j])[0]-middle(groups[i])[0],middle(groups[j])[1]-middle(groups[i])[1]
            p=groups[i][0];mid=middle(groups[i])
            preferred=1 if dx*(p['y']-mid[1])-dy*(p['x']-mid[0])>=0 else -1
            for side in (preferred,-preferred):
                if zeit_knapp(): return False
                C['paar_zulaessig'],copper=_paar_zustaende(a,b,side,alltr,allvs)
                viaH=_paar_via_mask(a,b,hard,side,copper)
                src=_paar_ports(groups[i],a,b,hard,side,copper); tgt=_paar_ports(groups[j],a,b,hard,-side,copper)
                if not src or not tgt: details.append('Keine Padports '+str((len(src),len(tgt))));continue
                found,path,attempts = _paar_kante(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,groups[i],groups[j])
                details += attempts
                if found is not None: C['paar_baum'][path] = True
                if found is not None: break
            if found is None:
                _PAAR_SUCH_INFO[a]=dict(grund='Kein gemeinsamer Kanal mit legaler Pad-/Via-Aufteilung',kanten=len(edges),details=details);return False
            for k in range(2): alltr[k]+=found[0][k]; allvs[k]+=found[1][k]
        if zeit_knapp(): return False
        alltr,allvs=_paar_baum_geometrie(alltr,allvs)
        if not _paar_baum_frei(alltr,allvs):
            _PAAR_SUCH_INFO[a]=dict(grund='Paar-Aeste/Vias verletzen Abstand');return False
        for k,n in enumerate((a,b)): add_unit(n,n,alltr[k],allvs[k])

        if RUECK>0 and (allvs[0] or allvs[1]) and set(RV_NETZ.values()).issubset(NID):
            RK=rv_karte(a,(min(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,min(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,max(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2,max(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2),C['via'])
            if RK is not None:
                extra=rv_setzen(a,alltr[0]+alltr[1],allvs[0]+allvs[1],RK)
                del_unit(a);add_unit(a,a,alltr[0],allvs[0]+extra)
        if not zeit_knapp(): laengen_abgleichen(nur_paare=True,nur_netze={a,b})
        success=not zeit_knapp()
        _PAAR_SUCH_INFO[a]=dict(grund='Gemeinsamer Kanal' if success else 'CPU-Budget',kanten=len(edges),baum_anfang=groups[root][0]['ref'],vias=[len(v) for v in allvs],planar_verworfen=C.get('paar_planar_verworfen',0),planar_ersatz=C.get('paar_planar_ersatz',0))
        return success
    finally:
        if not success:
            for n in (a,b): del_unit(n)
            for n,r in old.items():
                if r is not None: add_unit(n,r['net'],r['tracks'],r['vias'])
                else: add_unit(n,n,[],[])
            VIA_ALT.clear();VIA_ALT.update(anchors)
        _KIDX[0]=None;ASTAR_COST[0]=None


def _paar_distanzkosten(d, soll, staerke):
    ""
    kanal = np.abs(d - soll) <= .75 * RES
    return np.where(kanal, PAAR_KANAL, 1.0 + staerke * np.minimum(np.abs(d-soll)/max(soll,RES), 3.0)).astype(np.float32)


def paar_extra(net, C, capS, ex):
    ""
    fr = paar_frei_mm(net); e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    pn = PAAR_NETZE[net]
    cnt = use.copy()
    if pn in U: cnt[U[pn]['cells']] -= 1
    for li in range(NL) if PAAR_STOER > 1 else ():
        sl = slice(li * NN, (li + 1) * NN); f = cnt[sl] > 0
        if not f.any(): continue
        d = (_edt(~f.reshape(NY, NX)) * RES).ravel()
        e[sl] = np.where(d - C['w'] / 2 < fr, e[sl] * np.float32(PAAR_STOER), e[sl])
    if pn in U and U[pn]['tracks']:
        staerke = PAAR_ZUG if paar_schnell(net) else .25 * PAAR_ZUG
        m = np.zeros(NL * NN, dtype=bool)
        for t in U[pn]['tracks']:
            li = LI[t['layer']]
            for a, b in zip(t['pts'][:-1], t['pts'][1:]): m[seg_cells(li, a, b, 0.0)] = True
        soll = max(C['w'], (KL['netze'].get(pn) or {}).get('w', C['w'])) / 2 + C['w'] / 2 + PAAR_SPALT
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if not m[sl].any():
                e[sl] *= np.float32(1.0 + 3*staerke)
                continue
            if net in IMP_TAB:
                soll = z_breite(net, li, C['w']) / 2 + z_breite(pn, li, C['w']) / 2 + z_spalt(net, li)
            d = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
            e[sl] *= _paar_distanzkosten(d,soll,staerke)
    return e


def _okt_zug(pts):
    ""
    out = [pts[0]]
    for q in pts[1:]:
        c = _ecken(out[-1], q)
        if c: out.append(c[1])
        out.append(q)
    return out


def _paar_weggruppen(net):
    from analyse.laengenimpedanz import laengen, laengengruppen
    k = _paar_toleranzen(copy.deepcopy(KL))
    groups = laengengruppen(k)
    selected = {g:q for g,q in groups.items() if net in q.get('netze', [])}
    k['vorgaben'] = dict(k.get('vorgaben') or {},laengengruppen=selected)
    for n, entry in k['netze'].items():
        entry.pop('laengengruppe',None)
        if n in (net,PAAR_NETZE[net]) and paar_schnell(n): entry['z_diff_ohm'] = entry.get('z_diff_ohm') or 1
        else: entry.pop('z_diff_ohm',None)
    tracks = [dict(t,net=r['net']) for r in U.values() for t in r['tracks']]
    vias = [dict(v,net=v.get('net',r['net'])) for r in U.values() for v in r['vias']]
    return laengen(D,dict(tracks=tracks,vias=vias),k)


def _paar_vias_erlaubt(vias):
    ""
    for v in vias:
        ix, iy = math.floor((v['x']-X0)/RES), math.floor((v['y']-Y0)/RES)
        if not (0 <= ix < NX and 0 <= iy < NY): return False
        if VSP[iy, ix] < H2H + VIA_DR/2 + .01: return False
        if VIA_VERBOT is not None and VIA_VERBOT[iy, ix]: return False
    return True


def _paar_kandidat(net, tracks, vias, pruefen, tore=True):
    ""
    if tore and (zeit_knapp() or not _paar_vias_erlaubt(vias)): return False
    old = U[net]; accepted = False
    anchor_present = net in VIA_ALT; anchor = copy.deepcopy(VIA_ALT.get(net))
    del_unit(net)
    try:
        add_unit(net, old['net'], tracks, vias)
        accepted = (not tore or not zeit_knapp()) and einheit_konfliktfrei(net) and pruefen() and (not tore or not zeit_knapp())
        if accepted: kidx_geaendert(net)
        return accepted
    finally:
        if not accepted:
            del_unit(net); add_unit(net, old['net'], old['tracks'], old['vias']); kidx_zurueck(net)
            if tore:
                if anchor_present: VIA_ALT[net] = anchor
                else: VIA_ALT.pop(net, None)


def _paar_verschieben(a, b):
    ""
    from analyse.laengenimpedanz import weglaengen
    from shapely.geometry import LineString, Point
    if zeit_knapp(): return False
    rp, old = U[a], U[b]
    if any(v.get('rv') or v.get('gr') for r in (rp,old) for v in r['vias']): return False
    if not rp['tracks']: return False
    base = max(rp['tracks'], key=lambda t: sum(math.dist(p, q) for p, q in zip(t['pts'][:-1], t['pts'][1:])))
    dx, dy = base['pts'][-1][0]-base['pts'][0][0], base['pts'][-1][1]-base['pts'][0][1]

    directions = [(0, 1), (1, 0)] if abs(dx) >= abs(dy) else [(1, 0), (0, 1)]
    if dx and dy: directions.insert(0, (-math.copysign(1/math.sqrt(2), dy), math.copysign(1/math.sqrt(2), dx)))
    w = cls(b)['w']
    board = rand_polygon(D).buffer(-float(args.get('edge', '0.3')))
    before = next((p for p in paare_bericht(nur={a,b},alt_tor=not PAAR_SUCHRAUM) if p['netze'] == sorted((a, b))), None)
    if before is None: return False
    before_groups = _paar_weggruppen(b)
    for nx, ny in directions:
        if zeit_knapp(): return False
        for side in (1, -1):
            if zeit_knapp(): return False
            shifts = {LI[t['layer']]: (side*(_paar_masse(a,b,LI[t['layer']])[3] if PAAR_SUCHRAUM else (cls(a)['w']+w)/2+PAAR_SPALT)*nx,
                                      side*(_paar_masse(a,b,LI[t['layer']])[3] if PAAR_SUCHRAUM else (cls(a)['w']+w)/2+PAAR_SPALT)*ny) for t in rp['tracks']}
            if rp['vias'] and len(set(shifts.values())) > 1: continue
            shift = next(iter(shifts.values()))
            tracks = [dict(layer=t['layer'], pts=[[round(p[0]+shifts[LI[t['layer']]][0],4),round(p[1]+shifts[LI[t['layer']]][1],4)] for p in t['pts']],
                           w=[z_breite(b, LI[t['layer']], w)]*len(t['w'])) for t in rp['tracks']]
            vias = [dict(x=round(v['x']+shift[0],4),y=round(v['y']+shift[1],4)) for v in rp['vias']]
            valid = True
            for ki in bynet[b]:
                if zeit_knapp(): return False
                pad = D['pads'][ki]
                lines = [(LineString(t['pts']), t) for t in tracks if t['layer'] in pad['layers']]
                if not lines: valid = False; break
                point = Point(pad['x'], pad['y']); line, t = min(lines, key=lambda v: v[0].distance(point))
                q = line.interpolate(line.project(point)); pts = _okt_zug([(pad['x'],pad['y']),(q.x,q.y)])
                pts = [p for i,p in enumerate(pts) if i == 0 or math.dist(p,pts[i-1]) > 1e-6]
                if len(pts)>1: tracks.append(dict(layer=t['layer'],pts=[list(p) for p in pts],w=[w]*(len(pts)-1)))
            if not valid or any(not board.covers(LineString(t['pts']).buffer(max(t['w'])/2)) for t in tracks): continue
            if any(not board.covers(Point(v['x'],v['y']).buffer(VIA_R)) for v in vias): continue
            if IMP_REF_MASK and any(not z_ref_frei(cls(b), LI[t['layer']], p, q, b) for t in tracks for p,q in zip(t['pts'][:-1],t['pts'][1:])): continue
            if zeit_knapp(): return False
            def pruefen():
                paths = weglaengen(D,dict(tracks=[dict(t,net=b) for t in U[b]['tracks']],
                    vias=[dict(v,net=b) for v in U[b]['vias']]),b,REGELN_AUFBAU,alle_pads=True)
                after = next(p for p in paare_bericht(nur={a,b},alt_tor=not PAAR_SUCHRAUM) if p['netze'] == sorted((a,b)))
                good = bool(paths) and all(p['laenge_mm'] is not None for p in paths)
                good = good and after['gekoppelt'] > before['gekoppelt'] + .01 and after['schief_mm'] <= before['schief_mm'] + (1e-6 if PAAR_SUCHRAUM else .15)
                good = good and _empfaenger_nicht_schlechter(before_groups,_paar_weggruppen(b),b)
                return good
            if _paar_kandidat(b, tracks, vias, pruefen): return True
    return False


def paar_koppeln(nur=None):
    ""
    from shapely.geometry import LineString as _L2
    n_ok = 0; _KIDX[0] = None
    for P in sorted(PAAR_NETZE):
        if zeit_knapp(): break
        N = PAAR_NETZE[P]
        if P > N or P not in U or N not in U or paar_einheit(P): continue
        if nur is not None and P not in nur: continue
        if any(v.get('rv') or v.get('gr') for q in (P,N) for v in U[q]['vias']): continue
        if any(n_sv(U[q]) or len(U[q]['tracks']) != 1 or len(bynet.get(q, ())) != 2 for q in (P,N)):
            if _paar_verschieben(P,N) or _paar_verschieben(N,P): n_ok += 1
            continue
        for a_, b_ in ((P, N), (N, P)):
            if zeit_knapp(): break
            rp, rn = U[a_], U[b_]
            if n_sv(rp) or len(rp['tracks']) != 1 or len(bynet.get(a_, ())) != 2 or len(bynet.get(b_, ())) != 2: continue
            tp = rp['tracks'][0]; li = LI[tp['layer']]; Pp = [tuple(q) for q in tp['pts']]
            if len(Pp) < 2: continue
            pn = [D['pads'][k] for k in bynet[b_]]
            if not all(tp['layer'] in q['layers'] for q in pn): continue
            Cn = cls(b_); w = z_breite(b_, li, Cn['w'])
            soll = (_paar_masse(a_,b_,li)[3] if PAAR_SUCHRAUM else
                    ((z_breite(a_, li, cls(a_)['w'])/2 + w/2 + z_spalt(b_, li)) if b_ in IMP_TAB else
                     max(Cn['w'], cls(a_)['w'])/2 + w/2 + PAAR_SPALT))
            pa = min(pn, key=lambda q: math.dist((q['x'], q['y']), Pp[0])); pb = [q for q in pn if q is not pa][0]
            dx, dy = Pp[1][0] - Pp[0][0], Pp[1][1] - Pp[0][1]
            seite = 1.0 if (dx * (pa['y'] - Pp[0][1]) - dy * (pa['x'] - Pp[0][0])) > 0 else -1.0
            try: o = _L2(Pp).offset_curve(seite * soll, join_style=2, mitre_limit=3.0)
            except Exception: continue
            if o.geom_type != 'LineString' or o.is_empty: continue
            oq = [(round(x, 4), round(y, 4)) for x, y in o.coords]
            if math.dist(oq[0], (pa['x'], pa['y'])) > math.dist(oq[-1], (pa['x'], pa['y'])): oq = oq[::-1]
            zug = [(pa['x'], pa['y'])] + oq + [(pb['x'], pb['y'])]
            if okt_netz(b_): zug = _okt_zug(zug)
            zug = [q for i, q in enumerate(zug) if i == 0 or math.dist(q, zug[i - 1]) > 1e-6]
            neu_t = [dict(net=b_, layer=tp['layer'], pts=[[x, y] for x, y in zug], w=[w] * (len(zug) - 1))]
            if IMP_REF_MASK and not all(z_ref_frei(Cn, li, a, b, b_) for a, b in zip(zug[:-1], zug[1:])): continue
            Lp = sum(math.dist(x, y) for x, y in zip(Pp[:-1], Pp[1:])); Ln = sum(math.dist(x, y) for x, y in zip(zug[:-1], zug[1:]))
            L0 = sum(math.dist(x, y) for t in rn['tracks'] for x, y in zip(t['pts'][:-1], t['pts'][1:]))


            if abs(Ln - Lp) > abs(L0 - Lp) + (1e-6 if PAAR_SUCHRAUM else .15) and n_sv(rn) == 0: continue
            if not _L2(zug).is_simple: continue
            board = rand_polygon(D).buffer(-float(args.get('edge', '0.3')))
            if not board.covers(_L2(zug).buffer(w/2)): continue
            before_groups = _paar_weggruppen(b_)
            if _paar_kandidat(b_, neu_t, [], lambda: _empfaenger_nicht_schlechter(before_groups,_paar_weggruppen(b_),b_)):
                n_ok += 1; break
    _KIDX[0] = None
    return n_ok


def paare_bericht(empfaenger=False, nur=None, alt_tor=False):
    ""
    out = []
    for n in sorted(PAAR_NETZE):
        pn = PAAR_NETZE[n]
        if n > pn: continue
        if nur is not None and n not in nur: continue
        art = KL['netze'][n].get('paar_art', KL['netze'][n].get('art'))
        if n not in U or pn not in U:
            out.append(dict(netze=[n,pn], art=art, laenge=[None,None], schief_mm=None,
                vias=[n_sv(U[q]) if q in U else None for q in (n,pn)], gekoppelt=0.0, unvollstaendig=True))
            continue
        L = {q: sum(math.dist(a,b) for t in U[q]['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) for q in (n,pn)}
        soll = cls(n)['w']/2 + cls(pn)['w']/2 + PAAR_SPALT
        fractions = []; abstaende = {}
        for q, partner in ((n,pn),(pn,n)):
            gek = 0.0
            for t in U[q]['tracks']:
                li = LI[t['layer']]
                distance = (_paar_masse(q,partner,li)[3] if PAAR_SUCHRAUM else
                            ((z_breite(q,li,cls(q)['w'])/2 + z_breite(partner,li,cls(partner)['w'])/2 + z_spalt(q,li)) if q in IMP_TAB else soll))
                abstaende[t['layer']] = round(distance,4)
                for a,b in zip(t['pts'][:-1],t['pts'][1:]):
                    length = math.dist(a,b)
                    if length < 1e-8: continue
                    k = max(1, int(math.ceil(length/.05)))
                    parallel = [(c,d) for t2 in U[partner]['tracks'] if t2['layer'] == t['layer'] for c,d in zip(t2['pts'][:-1],t2['pts'][1:])
                                if math.dist(c,d)>1e-8 and abs((b[0]-a[0])*(d[0]-c[0])+(b[1]-a[1])*(d[1]-c[1]))/(length*math.dist(c,d)) > .995]
                    for i in range(k):
                        pt=(a[0]+(b[0]-a[0])*(i+.5)/k,a[1]+(b[1]-a[1])*(i+.5)/k)

                        dm=min((_seg_abst(pt,c,d) for c,d in parallel if alt_tor or
                                0 <= (pt[0]-c[0])*(d[0]-c[0])+(pt[1]-c[1])*(d[1]-c[1]) <= (d[0]-c[0])**2+(d[1]-c[1])**2),default=math.inf)
                        if abs(dm-distance)<=.05: gek += length/k
            fractions.append(gek/max(L[q],1e-9))
        out.append(dict(netze=[n,pn], art=art, laenge=[round(L[n],3),round(L[pn],3)],
            schief_mm=round(abs(L[n]-L[pn]),3), vias=[n_sv(U[n]),n_sv(U[pn])],
            gekoppelt=round(min(fractions),3), gekoppelt_je_netz=[round(f,3) for f in fractions],
            abstand_soll=round(soll,3), abstand_je_lage=abstaende))
    if empfaenger:
        from analyse.laengenimpedanz import laengen
        k = _paar_toleranzen(copy.deepcopy(KL))
        for n in PAAR_NETZE:
            if paar_schnell(n): k['netze'][n]['z_diff_ohm'] = k['netze'][n].get('z_diff_ohm') or 1
        tr,vi = ausgabe_geometrie(); groups=laengen(D,dict(tracks=tr,vias=vi),k)
        for p in out:
            g=groups.get('paar:'+'|'.join(p['netze']))
            p.update(skew_wege_mm=g['fehler_mm'] if g else None, toleranz_mm=g['toleranz_mm'] if g else None,
                     skew_in_toleranz=bool(g and g['vollstaendig'] and g['ueber_mm']<=1e-6),
                     empfaenger=g['empfaenger'] if g else {})
    return out


def _empfaenger_nicht_schlechter(vorher, nachher, net):
    ""
    from analyse.laengenimpedanz import empfaenger_schutz
    return empfaenger_schutz(vorher, nachher, {net}, streng=PAAR_SUCHRAUM)


def laengen_abgleichen(nur_paare=False, nur_netze=None):
    ""
    from analyse.laengenimpedanz import laengen
    from shapely.geometry import LineString, Point
    board = rand_polygon(D).buffer(-float(args.get('edge', '0.3')))
    aufbau = (KL.get('vorgaben') or {}).get('aufbau') or KL.get('aufbau') or REGELN_AUFBAU
    def routes():
        tr, vi = ausgabe_geometrie()
        return dict(tracks=tr, vias=vi)
    length_kl = _paar_toleranzen(copy.deepcopy(KL))
    if nur_paare:
        length_kl['vorgaben'] = dict(length_kl.get('vorgaben') or {}, laengengruppen={})
        for k in length_kl['netze'].values(): k.pop('laengengruppe', None); k.pop('z_diff_ohm', None)


    for n in PAAR_NETZE:
        if paar_schnell(n): length_kl['netze'][n]['z_diff_ohm'] = length_kl['netze'][n].get('z_diff_ohm') or 1
    groups = laengen(D, routes(), length_kl)
    lengths = {n: v for g in groups.values() for n, v in g['netze'].items()}
    targets = {n: v for n, v in lengths.items() if v is not None}

    for _ in range(len(groups) + 1):
        changed = False
        for g in groups.values():
            if not g['vollstaendig']: continue
            target = max(targets[n] for n in g['netze'])
            for n in g['netze']:
                if targets[n] < target: targets[n] = target; changed = True
        if not changed: break
    accepted = []; tried = 0; _KIDX[0] = None
    for net in sorted(targets):
        if nur_netze is not None and net not in nur_netze: continue
        if net not in U or net.startswith(('FIX#', 'HEFT#', 'GND#')): continue


        full_groups = _paar_weggruppen(net) if nur_paare and net in PAAR_NETZE else None
        tol = min((g['toleranz_mm'] for g in groups.values() if net in g['netze']), default=0.15)
        for _ in range(int(args.get('laengen_zuege', '12'))):
            if zeit_knapp(0.95) or targets[net] - lengths[net] <= tol * 0.8: break
            old = U[net]; delta = targets[net] - lengths[net]; success = False
            segments = sorted([(math.dist(a, b), ti, si, a, b, w) for ti, t in enumerate(old['tracks'])
                               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))], reverse=True)
            for L, ti, si, a, b, w in segments:
                if not okt(a, b): continue
                layer = old['tracks'][ti]['layer']; c = max(3 * w, 0.15); top = 4 * w + 0.02
                span = 4 * c + top; lead = (L - span) / 2
                compact = net in PAAR_NETZE and (PAAR_SUCHRAUM or delta <= 0.6)
                if compact:



                    if PAAR_SUCHRAUM: delta=max(0.,targets[net]-lengths[net]-.75*tol)
                    height = delta / (2 * (math.sqrt(2) - 1)); span = 2*height; lead = (L-span)/2
                if lead < 4 * w + 0.02: continue
                e = ((b[0] - a[0]) / L, (b[1] - a[1]) / L); normal = (-e[1], e[0])

                for fraction in (1.0, 0.5, 0.25, 0.125):
                    add = delta * fraction
                    height = add / (2 * (math.sqrt(2) - 1)) if compact else add / 2 + 2 * c * (2 - math.sqrt(2))
                    if not compact and height < 2 * c + 0.02: continue
                    if compact:
                        span = 2*height; lead = (L-span)/2
                        if lead < 4*w+.02: continue
                    if PAAR_SUCHRAUM and net in PAAR_NETZE:
                        receivers=[]
                        for g in groups.values():
                            if net not in g['netze'] or not g['vollstaendig'] or g.get('fehler_mm',float('inf'))<=g['toleranz_mm']: continue
                            for path in g.get('wege',{}).get(net,[]):
                                ref,num=(path.get('empfaenger') or ':').rsplit(':',1)
                                receivers += [(p['x'],p['y']) for p in D.get('pads',[]) if p['net']==net and p['ref']==ref and str(p['num'])==num]
                        local_span=2*height if compact else span
                        available=L-local_span
                        margin=4*w+.02
                        if available>=2*margin and receivers:
                            da=min(math.dist(a,p) for p in receivers);db=min(math.dist(b,p) for p in receivers)
                            lead=margin if da<db else available-margin

                    positions=[lead]
                    if compact and PAAR_SUCHRAUM:
                        available=L-span
                        direction=1. if lead<available/2 else -1.
                        margin=4*w+.02
                        positions += [min(available-margin,max(margin,lead+direction*d)) for d in (.5,1.)]
                        positions += [available/2]
                    for lead in dict.fromkeys(positions):

                        foreign = [LineString([pa, pb]).buffer(ww/2) for r2 in U.values() if r2['net'] != net for tt in r2['tracks'] if tt['layer'] == layer
                                   for pa, pb, ww in zip(tt['pts'][:-1], tt['pts'][1:], tt['w'])]
                        foreign += [g for nn, g in _PADG[LI[layer]] if nn != net]
                        def freiraum(side):
                            mid = (Point(a[0]+(lead+(height if compact else span/2))*e[0] + side*height*normal[0], a[1]+(lead+(height if compact else span/2))*e[1] + side*height*normal[1])
                                   if PAAR_SUCHRAUM else Point((a[0]+b[0])/2 + side*height*normal[0], (a[1]+b[1])/2 + side*height*normal[1]))
                            return min((mid.distance(g) for g in foreign), default=1e9)
                        for side in sorted((1, -1), key=lambda s: (-freiraum(s), -s)):
                            if zeit_knapp(0.95): break
                            tried += 1
                            q = [(0, 0), (lead, 0), (lead + c, c), (lead + c, height - c),
                                 (lead + 2*c, height), (lead + 2*c + top, height),
                                 (lead + 3*c + top, height - c), (lead + 3*c + top, c),
                                 (lead + span, 0), (L, 0)]
                            if compact:
                                if not PAAR_SUCHRAUM: lead = (L - 2*height)/2
                                q = [(0,0),(lead,0),(lead+height,height),(lead+2*height,0),(L,0)]
                            pts = [[round(a[0] + x*e[0] + side*y*normal[0], 4),
                                    round(a[1] + x*e[1] + side*y*normal[1], 4)] for x, y in q]
                            pts[0], pts[-1] = list(a), list(b)
                            line = LineString(pts); copper = line.buffer(w/2, cap_style=1)
                            if not line.is_simple or not board.covers(copper): continue
                            if IMP_REF_MASK and not all(z_ref_frei(cls(net), LI[layer], a, b, net) for a, b in zip(pts[:-1], pts[1:])): continue

                            pocket = LineString(pts[1:-1])
                            own_units = [(u2, r2) for u2, r2 in U.items() if r2['net'] == net]
                            own = [LineString([pa, pb]).buffer(ww/2) for u2, r2 in own_units for tj, t in enumerate(r2['tracks']) if t['layer'] == layer
                                   for sj, (pa, pb, ww) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if (u2, tj, sj) != (net, ti, si)]
                            own += [Point(v['x'], v['y']).buffer(VIA_R) for _, r2 in own_units for v in r2['vias']]
                            own += [g for n, g in _PADG[LI[layer]] if n == net]
                            if any(pocket.distance(g) < 3*w + w/2 - 1e-5 for g in own): continue
                            new = copy.deepcopy(old['tracks']); t = new[ti]
                            t['pts'] = t['pts'][:si] + pts + t['pts'][si+2:]
                            t['w'] = t['w'][:si] + [w]*(len(pts)-1) + t['w'][si+1:]
                            result = {}
                            def pruefen():
                                after = laengen(D, routes(), length_kl)
                                observed = [g['netze'][net] for g in after.values() if net in g['netze']]
                                length = max(observed) if observed and all(v is not None for v in observed) else None
                                good = length is not None and lengths[net] + 1e-4 < length <= targets[net] + 0.001
                                good = good and _empfaenger_nicht_schlechter(groups, after, net)
                                after_full = _paar_weggruppen(net) if good and full_groups is not None else None
                                if full_groups is not None:
                                    good = good and _empfaenger_nicht_schlechter(full_groups, after_full, net)
                                result.update(after=after, length=length, full=after_full)
                                return good
                            if _paar_kandidat(net, new, old['vias'], pruefen, tore=PAAR_SUCHRAUM):
                                length = result['length']
                                accepted.append(dict(net=net, layer=layer, vorher_mm=lengths[net], nachher_mm=length, w=w))
                                if full_groups is not None: full_groups = result['full']
                                lengths[net] = length; groups = result['after']; success = True; break
                        if success: break
                    if success: break
                if success: break
            if not success: break
    _KIDX[0] = None
    return dict(zuege=accepted, versuche=tried, gruppen=laengen(D, routes(), length_kl))


def sw_extra(net):
    ""
    if SW_ABST <= 0 or not any(k in net for k in SENS): return None
    sig = sum(_NVER.get(n, 0) for n in _SWN)
    c = _NAHE_C.get('sw_alt')
    if c is not None and c[0] == sig: return c[1]
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'].split('/')[-1].startswith('SW_'):
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'].split('/')[-1].startswith('SW_'): m[r['cells']] = True
    ex = np.ones(NL * NN, dtype=np.float32)
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < SW_ABST, 6.0, 1.0)
    _NAHE_C['sw_alt'] = (sig, ex)
    return ex


def geo_nodes(trs, vs):
    ""
    out = set()
    for t in trs:
        li = LI[t['layer']]
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.25)) + 1)
            for k in range(n):
                x = a[0] + (b[0] - a[0]) * k / (n - 1); y = a[1] + (b[1] - a[1]) * k / (n - 1)
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if math.hypot(X0 + (ix + 0.5) * RES - x, Y0 + (iy + 0.5) * RES - y) < 0.03:
                    out.add(li * NN + iy * NX + ix)
                    if ABZW_EXAKT:
                        _l = _STAMM.setdefault(li * NN + iy * NX + ix, [])
                        if not any(_t is t for _t in _l): _l.append(t)
    for v in vs:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        for li in range(NL):
            out.add(li * NN + iy * NX + ix)
            if ABZW_EXAKT: _STAMM_V[li * NN + iy * NX + ix] = (v['x'], v['y'])
    return sorted(out)







VG = args.get('viagruppe', '1' if KL else '0') == '1'
VG_ABST = max(float(args.get('vg_abstand', '0.7')), HOLE_C)
VG_IVIA = float(args.get('vg_strom03', '1.0')) * VIA_DR / 0.3
STROM_VIA_PREIS = args.get('strom_via_preis', '0') == '1' and KL is not None
VG_RES = float(args.get('vg_reserve', '1.3')); VG_MIN = float(args.get('vg_mindest', '0.5')); VG_MAX = int(args.get('vg_max', '0' if STROM_VIA_PREIS else '4'))
VG_REL = args.get('vg_relativ', '0') == '1'
VG_WEICH = [True]


def vg_anzahl(net, pads, tree, j, mit=()):
    ""
    k = KL['netze'].get(net) or {}; fl = k.get('fluss') or {}
    if k.get('art') not in ('strom', 'versorgung', 'sw') or not (fl.get('einheit') == 'A' or VG_REL): return 1
    last = fl.get('last') or {}; quelle = set(fl.get('quelle') or [])
    def nm(i): return '%s:%s' % (pads[i]['ref'], pads[i]['num'])
    lt = sum(float(last.get(nm(i), 0.0)) for i in tree if nm(i) not in quelle)
    lj = float(last.get(nm(j), 0.0))
    if mit: lj += sum(float(last.get(nm(i), 0.0)) for i in mit if nm(i) not in quelle)
    if nm(j) in quelle or any(nm(i) in quelle for i in mit): strom = lt
    elif any(nm(i) in quelle for i in tree): strom = lj
    else: strom = min(lj, lt)
    if strom < VG_MIN: return 1
    soll = max(2, int(math.ceil(VG_RES * strom / VG_IVIA - 1e-9)))
    return min(VG_MAX, soll) if VG_MAX > 0 else soll


def vg_setzen(net, res, trs, vs, n_soll, capH, viaH, capC, vbadf, Cb, schon, rng):
    ""
    viaf = viaH.ravel(); w_st = max(BAHN_MIN, min(Cb['wmin'], VIA_D))
    wechsel = [(a // NN, b // NN) for a, b in zip(res[:-1], res[1:]) if a // NN != b // NN]
    eigene = [(v['x'], v['y']) for v in schon] + [(v['x'], v['y']) for v in vs]
    stege, neu_v = [], []
    for v, (la, lb) in zip(vs, wechsel):
        v['n'] = n_soll; vx, vy = v['x'], v['y']
        kand = []
        for t in trs:
            li = LI[t['layer']]
            if li not in (la, lb): continue
            for pts in (t['pts'], t['pts'][::-1]):
                if abs(pts[0][0] - vx) > 1e-6 or abs(pts[0][1] - vy) > 1e-6: continue
                weg = 0.0; k = 1
                for a, b in zip(pts[:-1], pts[1:]):
                    L = math.hypot(b[0] - a[0], b[1] - a[1])
                    while k < n_soll and L > 0 and weg + L >= k * VG_ABST - 1e-9:
                        f = (k * VG_ABST - weg) / L
                        kand.append((k, li, a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))); k += 1
                    weg += L
                    if k >= n_soll: break
        ringe = [(10, VG_ABST), (11, 1.5 * VG_ABST)]
        if STROM_VIA_PREIS:
            ringe += [(12 + k, (2 + k) * VG_ABST) for k in range(max(1, int(math.ceil(math.sqrt(n_soll))))) ]
        for ring, r in ringe:
            nq = 8 if ring < 12 else max(8, int(2 * math.pi * r / VG_ABST))
            for q in range(nq):
                kand.append((ring, -1, vx + r * math.cos(q * 2 * math.pi / nq), vy + r * math.sin(q * 2 * math.pi / nq)))
        fehlt = n_soll - 1
        for weich in ((False, True) if VG_WEICH[0] else (False,)):
            karte = capH if weich else capC
            for rang, lt, x, y in sorted(kand):
                if fehlt <= 0: break
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if not (rng[0] < ix < rng[1] - 1 and rng[2] < iy < rng[3] - 1): continue
                c = iy * NX + ix; x, y = cell_xy(ix, iy)
                if not viaf[c] or (not weich and vbadf[c]): continue
                if any(math.hypot(x - ex, y - ey) < VG_ABST - 0.06 for ex, ey in eigene): continue
                st = []
                for lo in (la, lb):
                    if lo == lt: continue
                    if line_cap_min(karte[lo], (x, y), (vx, vy)) < w_st - 1e-6 or pad_room(lo, (x, y), (vx, vy), net) < w_st / 2: st = None; break
                    st.append(dict(layer=LAYERS[lo], pts=[[round(x, 4), round(y, 4)], [vx, vy]], w=[w_st], steg=True))
                if st is None: continue
                stege += st; neu_v.append(dict(x=round(x, 4), y=round(y, 4), gr=1, zu=[vx, vy])); eigene.append((x, y)); fehlt -= 1
    return stege, neu_v








RUECK = float(args.get('rueck', '1.27')) if KL is not None else 0.0
RUECK_R3 = float(args.get('rueck_r3', '2.5'))
RUECK_DA = float(args.get('rueck_da', '0.25')); RUECK_FEHLT = float(args.get('rueck_fehlt', '1.0'))
RV_PAAR = np.zeros(NL * NL, dtype=np.float64)
RV_NETZ = {}
if RUECK > 0:



    for _li, _a in enumerate(AUFBAU):
        for _lj, _b in enumerate(AUFBAU):
            if _li == _lj or not _a['ref'] or not _b['ref'] or _a['ref'] == _b['ref']: continue
            _gn = sorted({x for x in (_a['ref_netz'], _b['ref_netz']) if x in GNDS})
            if not _gn and GND_HAUPT in FLNETZE: _gn = [GND_HAUPT]
            if len(_gn) == 1: RV_PAAR[_li * NL + _lj] = 1.0; RV_NETZ[(_li, _lj)] = _gn[0]
    if not RV_NETZ: RUECK = 0.0
    else: print('Rueck-Vias (Q2): %d Lagenpaare mit verschiedener Masse-Bezugsflaeche, Radius %.2f mm (Rang <= 3: %.2f mm)' % (
        len(RV_NETZ) // 2, RUECK, RUECK_R3), flush=True)
RV_STAT = dict(gesetzt=0, schon=0, kein_platz=0, nachlauf_neu=0, nachlauf_weg=0)


def rv_radius(net):
    ""
    if RUECK <= 0: return 0.0
    k = KL['netze'].get(net) or {}
    if k.get('art') != 'schnell': return 0.0
    return RUECK if k.get('rang', 0) >= 4 else RUECK_R3


def _rv_gnd(netz):
    ""
    xy = [(v['x'], v['y']) for r in U.values() for v in r['vias'] if (v.get('net') or r['net']) == netz]
    xy += [(D['pads'][k]['x'], D['pads'][k]['y']) for k in bynet_g.get(netz, ()) if D['pads'][k].get('drill')]
    return np.array(xy, dtype=np.float64).reshape(-1, 2)


def rv_karte(net, box, VC):
    ""
    R = rv_radius(net)
    if R <= 0: return None
    if PLANE_LAYER:
        local={g:_rv_karte_netz(g,box,VC,R) for g in sorted(GNDS & PLANE_AREA.keys())}
        if not local:return None

        vzu=np.full(NN,np.float32(RUECK_FEHLT*VC),dtype=np.float32)
        for g,rk in local.items():
            mask=PLANE_MASK[g].ravel();vzu[mask]=np.minimum(vzu[mask],rk['vzu'][mask])
        return dict(lokal=local,vzu=vzu,R=R)
    netze = sorted(set(RV_NETZ.values()))
    if len(netze) != 1: return None
    g = netze[0]
    return _rv_karte_netz(g,box,VC,R)


def _rv_karte_netz(g, box, VC, R):
    bxR = (box[0] - R, box[1] - R, box[2] + R, box[3] + R)
    ix0, ix1, iy0, iy1 = _box_idx(bxR)
    ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    rng = (ix0, ix1, iy0, iy1)
    _, vH = hard_maps(g, rng if GROSS else None)
    frei = np.zeros((NY, NX), dtype=bool); f = np.array(vH[ys, xs], dtype=bool, copy=True)
    capS, dd, _r2 = soft_maps(g, bxR, VIA_D)
    f &= dd[ys, xs] >= HOLE_C
    for li in range(NL): f &= capS[li][ys, xs] / 2 >= VIA_R
    if PLANE_LAYER:
        xx,yy=np.meshgrid(GXc[ix0:ix1],GYc[iy0:iy1])
        for L,nn in FL_ZUORDNUNG.items():
            if g in nn:
                area=PLANE_LAYER.get((g,L),Polygon()).buffer(-VIA_R-.01)
                f &= shapely.contains_xy(area,xx,yy)
    frei[ys, xs] = f
    vzu = np.full(NN, np.float32(RUECK_FEHLT * VC), dtype=np.float32); V = vzu.reshape(NY, NX)
    if f.any(): V[ys, xs] = np.where(_edt(~f) * RES <= R, np.float32(RUECK_DA * VC), V[ys, xs])
    gv = _rv_gnd(g)
    if len(gv):
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        jx = ((gv[:, 0] - X0) / RES).astype(int) - ix0; jy = ((gv[:, 1] - Y0) / RES).astype(int) - iy0
        ok = (jx >= 0) & (jx < ix1 - ix0) & (jy >= 0) & (jy < iy1 - iy0)
        m[jy[ok], jx[ok]] = True
        if m.any(): V[ys, xs] = np.where(_edt(~m) * RES <= R, np.float32(0.0), V[ys, xs])
    return dict(netz=g, frei=frei, vzu=vzu, R=R)


def _seg_abst(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _via_lagen(net, v, trs):
    ""
    gen = set()
    for t in trs:
        if t.get('net', net) != net: continue
        for e in (t['pts'][0], t['pts'][-1]):
            if abs(e[0] - v['x']) < VIA_R + 0.05 and abs(e[1] - v['y']) < VIA_R + 0.05: gen.add(LI[t['layer']])
    for k in bynet_g.get(net, ()):
        p = D['pads'][k]; bb = p['bbox']
        if bb[0] - 0.01 <= v['x'] <= bb[2] + 0.01 and bb[1] - 0.01 <= v['y'] <= bb[3] + 0.01: gen |= {LI[L] for L in p['layers'] if L in LI}
    return gen


def rv_bedarf(net, v, trs):
    ""
    gen = sorted(_via_lagen(net, v, trs))
    for i, a in enumerate(gen):
        for b in gen[i + 1:]:
            if RV_PAAR[a * NL + b] > 0:
                if PLANE_LAYER:
                    refs=[AUFBAU[i]['ref'] for i in (a,b)]
                    mass=[L for L in refs if set(FL_ZUORDNUNG.get(L,[])) & GNDS]
                    pt=_PT(v['x'],v['y'])
                    domains=globals().get('PLANE_REF',PLANE_LAYER)
                    candidates=[g for g in sorted(GNDS) if mass and all(domains.get((g,L),Polygon()).covers(pt) for L in mass)]
                    return candidates[0] if len(candidates)==1 else None
                return RV_NETZ[(a, b)]
    return None


def rv_setzen(net, trs, vs, RK, eigen=None):
    ""
    if 'lokal' in RK:
        neu=[]
        for g,rk in sorted(RK['lokal'].items()):
            subset=[v for v in vs if rv_bedarf(net,v,trs)==g]
            neu+=rv_setzen(net,trs,subset,rk,eigen=(vs if eigen is None else eigen)+neu)
        return neu
    R = RK['R']; g = RK['netz']; frei = RK['frei']; neu = []
    gv = _rv_gnd(g)
    eig_v = [(v['x'], v['y']) for v in (vs if eigen is None else eigen)]
    r0 = VIA_D + DRCC + 0.02
    ringe = [r0] + [r for r in (0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0, 2.25, 2.5) if r0 < r < R - 0.02] + [R - 0.02]
    for v in vs:
        if v.get('rv') or v.get('gr'): continue
        if rv_bedarf(net, v, trs) != g: continue
        alle = [tuple(q) for q in gv.tolist()] + [(w['x'], w['y']) for w in neu]
        if any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in alle): RV_STAT['schon'] += 1; continue
        ok = False
        for r in ringe:
            for q in range(16):
                ix = int((v['x'] + r * math.cos(q * math.pi / 8) - X0) / RES); iy = int((v['y'] + r * math.sin(q * math.pi / 8) - Y0) / RES)
                if not (0 <= ix < NX and 0 <= iy < NY) or not frei[iy, ix]: continue
                x, y = cell_xy(ix, iy)
                if math.hypot(x - v['x'], y - v['y']) > R: continue
                if any(math.hypot(x - a, y - b) < max(VIA_D + DRC_CLR, DRC_HOLE) + 0.01 for a, b in eig_v + [(w['x'], w['y']) for w in neu]): continue
                if any(_seg_abst((x, y), a, b) < VIA_R + w_ / 2 + DRC_CLR + 0.01 for t in trs for a, b, w_ in zip(t['pts'][:-1], t['pts'][1:], t['w'])): continue
                neu.append(dict(x=round(x, 4), y=round(y, 4), net=g, rv=True)); ok = True; break
            if ok: break
        RV_STAT['gesetzt' if ok else 'kein_platz'] += 1
    return neu


def rv_nachlauf():
    ""
    if RUECK <= 0: return 0, 0
    neu_n = weg_n = 0; _KIDX[0] = None
    for u in sorted(U):
        if u.startswith(('GND#', 'FIX#', 'HEFT#')): continue
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or not r['vias']: continue
        sv = [v for v in r['vias'] if not v.get('rv')]; rv = [v for v in r['vias'] if v.get('rv')]
        bedarf = [(v, rv_bedarf(net, v, r['tracks'])) for v in sv]; bedarf = [(v, g) for v, g in bedarf if g]
        if not bedarf and not rv: continue
        legacy_g = bedarf[0][1] if bedarf else rv[0]['net']
        fremd_netze={}
        for g in sorted({g for _,g in bedarf}|{v['net'] for v in rv}):
            source_g=g if PLANE_LAYER else legacy_g
            fremd_netze[g]=[(v['x'],v['y']) for w,r2 in U.items() if w!=u for v in r2['vias'] if (v.get('net') or r2['net'])==source_g]
            fremd_netze[g]+=[(D['pads'][k]['x'],D['pads'][k]['y']) for k in bynet_g.get(source_g,()) if D['pads'][k].get('drill')]
        behalten = []
        for w in rv:
            noetig = any((not PLANE_LAYER or w['net']==g) and math.hypot(v['x'] - w['x'], v['y'] - w['y']) <= R and not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in fremd_netze[g])
                         for v, g in bedarf)
            if noetig: behalten.append(w)
            else: weg_n += 1
        offen = [v for v,g in bedarf if not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a,b in fremd_netze[g]+[(w['x'],w['y']) for w in behalten if not PLANE_LAYER or w['net']==g])]
        if len(behalten) != len(rv):
            del_unit(u); add_unit(u, net, r['tracks'], sv + behalten); r = U[u]; kidx_geaendert(u)
        if offen:
            pads = [D['pads'][k] for k in bynet[net]]
            bx = (min(p['bbox'][0] for p in pads) - 3.0, min(p['bbox'][1] for p in pads) - 3.0, max(p['bbox'][2] for p in pads) + 3.0, max(p['bbox'][3] for p in pads) + 3.0)
            bx = (min([bx[0]] + [v['x'] - 1 for v in offen]), min([bx[1]] + [v['y'] - 1 for v in offen]), max([bx[2]] + [v['x'] + 1 for v in offen]), max([bx[3]] + [v['y'] + 1 for v in offen]))
            import copy as _cp
            save = _cp.deepcopy(U[u]); del_unit(u)
            RK = rv_karte(net, bx, 0.0)
            zus = rv_setzen(net, save['tracks'], offen, RK, eigen=save['vias']) if RK else []
            add_unit(u, net, save['tracks'], save['vias'] + zus)
            if zus and not einheit_konfliktfrei(u):
                del_unit(u); add_unit(u, net, save['tracks'], save['vias']); zus = []
            kidx_geaendert(u); neu_n += len(zus)
    RV_STAT['nachlauf_neu'] += neu_n; RV_STAT['nachlauf_weg'] += weg_n; _KIDX[0] = None
    return neu_n, weg_n








VIA_ANKER = float(args.get('via_anker', '0')) if KL is not None else 0.0
VIA_ALT = {}
VIA_ANKER_AN = [False]
VIA_VORRANG = args.get('via_vorrang', '0') == '1' and KL is not None
PAIRS_ROH = []



K12 = float(args.get('k12', '0')) if (KL is not None and FLNETZE) else 0.0
K12_ABST = float(args.get('k12_abstand', '%g' % (VIA_D + 2 * 0.15 + 0.15)))


def via_zuschlag(u, net, C, box):
    ""
    k = (KL['netze'].get(net) or {}) if KL else {}
    anker = VIA_ANKER > 0 and VIA_ANKER_AN[0] and k.get('art') in ('strom', 'versorgung', 'sw') and VIA_ALT.get(u)
    k12 = K12 > 0 and DICHT and net not in FLNETZE
    if not (anker or k12): return None
    V = None
    ix0, ix1, iy0, iy1 = _box_idx(box); ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    if anker:
        V = np.full((NY, NX), np.float32(VIA_ANKER), dtype=np.float32)
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        for x, y in VIA_ALT[u]:
            jx, jy = int((x - X0) / RES) - ix0, int((y - Y0) / RES) - iy0
            if 0 <= jx < ix1 - ix0 and 0 <= jy < iy1 - iy0: m[jy, jx] = True
        if m.any():
            d = _edt(~m) * RES
            V[ys, xs] = (VIA_ANKER * np.clip((d - 1.0) / 4.0, 0.0, 1.0)).astype(np.float32)
    if k12:
        vs = [(v['x'], v['y']) for w, r in U.items() for v in r['vias'] if (v.get('net') or r['net']) not in FLNETZE]
        for lg, dx0, dx1, dy0, dy1, _nz in DICHT:
            ax0, ax1, ay0, ay1 = dx0, dx1, dy0, dy1
            if ax1 <= ax0 or ay1 <= ay0: continue
            pz = int(K12_ABST / RES) + 2
            m2 = np.zeros((ay1 - ay0 + 2 * pz, ax1 - ax0 + 2 * pz), dtype=bool)
            for x, y in vs:
                jx, jy = int((x - X0) / RES) - ax0 + pz, int((y - Y0) / RES) - ay0 + pz
                if 0 <= jx < m2.shape[1] and 0 <= jy < m2.shape[0]: m2[jy, jx] = True
            if not m2.any(): continue
            d = (_edt(~m2) * RES)[pz:-pz, pz:-pz]
            if V is None: V = np.zeros((NY, NX), dtype=np.float32)
            V[ay0:ay1, ax0:ax1] += np.where(d < K12_ABST, np.float32(K12 * C['via'] * VIA_FAK[0]), np.float32(0.0))
    return None if V is None else V.ravel()


def vorrang_filtern(conf):
    ""
    def stufe1(w): return not w.startswith(('GND#', 'FIX#', 'HEFT#')) and w in U and (KL['netze'].get(U[w]['net']) or {}).get('art') in ('signal', 'analog')
    neu = {}
    def note(w, x, y, li):
        if w == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES); lst = neu.setdefault(w, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    n = 0
    for li, x, y, w1, w2 in PAIRS_ROH:
        for a, b in ((w1, w2), (w2, w1)):
            if a.startswith('GND#') and stufe1(b) and li >= 0: break
        else: a = None
        lis = range(NL) if li < 0 else (li,)
        for l_ in lis:
            if a is None: note(w1, x, y, l_); note(w2, x, y, l_)
            else: note(b, x, y, l_); n += 1
    return neu, n


def rv_bericht():
    ""
    if RUECK <= 0: return None
    n = ok = eigen = 0; fehlt = []
    for u in sorted(U):
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or u.startswith(('GND#', 'FIX#', 'HEFT#')): continue
        for v in r['vias']:
            if v.get('rv'): continue
            g = rv_bedarf(net, v, r['tracks'])
            if not g: continue
            n += 1; gv = _rv_gnd(g)
            d = float(np.min(np.hypot(gv[:, 0] - v['x'], gv[:, 1] - v['y']))) if len(gv) else 99.0
            if d <= R + 1e-6:
                ok += 1; eigen += any(w.get('rv') and math.hypot(w['x'] - v['x'], w['y'] - v['y']) <= R + 1e-6 for w in r['vias'])
            else: fehlt.append(dict(netz=net, x=v['x'], y=v['y'], abstand=round(d, 2)))
    return dict(wechsel=n, mit_rueck=ok, eigenes_rueck_via=eigen, ohne=fehlt, stat=dict(RV_STAT))







FLUCHT = args.get('flucht', '1' if KL else '0') == '1'
_FL = {}


def _pz_ports(net,p,w,capH,rng):
    ""
    if p.get('drill') or max(p['bbox'][2]-p['bbox'][0],p['bbox'][3]-p['bbox'][1])>1.2:return ()
    out={};a=(p['x'],p['y'])
    for L in p['layers']:
        if L not in LI:continue
        li=LI[L]
        own=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        if not own.covers(_PT(a)):continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            seen=set()
            for step in range(1,33):
                d=step*.025
                ix=int((a[0]+dx*d-X0)/RES);iy=int((a[1]+dy*d-Y0)/RES)
                if (ix,iy) in seen:continue
                seen.add((ix,iy))
                if not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
                if capH[li][iy,ix]<w:continue
                b=cell_xy(ix,iy)
                if own.covers(_PT(b)):continue
                paths=[[a,b]] if okt(a,b) else [[a,c,b] for c in _ecken(a,b)]
                good=[]
                for pts in paths:
                    pts=[_pz_snap(p) for p in pts]
                    if all(pad_room(li,x,y,net)>=w/2 and (not HART_FREMD[0] or _pz_room(net,li,x,y,False)>=w/2) for x,y in zip(pts[:-1],pts[1:])):
                        good.append(pts)
                if good:
                    pts=min(good,key=lambda ps:sum(math.dist(x,y) for x,y in zip(ps[:-1],ps[1:])))
                    c=li*NN+iy*NX+ix
                    if c not in out:out[c]=(c,L,[list(pt) for pt in pts])
                    break
    return tuple(out[c] for c in sorted(out))


def flucht(net, p, wk, capH, rng):
    if _pz_active(net) and id(p) in PZ_LOKAL[net]:return _pz_ports(net,p,wk,capH,rng)
    if not FLUCHT or p.get('drill'): return ()
    key = (id(p), wk)
    if key in _FL: return _FL[key]
    lag = [x for x in p['layers'] if x in LI]
    bx = p['bbox']; bw, bh = bx[2] - bx[0], bx[3] - bx[1]
    if len(lag) != 1 or min(bw, bh) > 0.45 or max(bw, bh) < 1.3 * min(bw, bh): _FL[key] = (); return ()
    L = lag[0]; li = LI[L]; cap = capH[li]; cx, cy = (bx[0] + bx[2]) / 2, (bx[1] + bx[3]) / 2
    out = []; merken = True
    for sgn in (-1.0, 1.0):
        if bw < bh: ex, ey, dx, dy = cx, (bx[3] if sgn > 0 else bx[1]), 0.0, sgn
        else: ex, ey, dx, dy = (bx[2] if sgn > 0 else bx[0]), cy, sgn, 0.0
        zellen = []; offen = False
        for q in range(1, 13):
            ix, iy = int(math.floor((ex + dx * q * RES - X0) / RES)), int(math.floor((ey + dy * q * RES - Y0) / RES))


            if not (0 <= ix < NX and 0 <= iy < NY): break
            if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): offen = True; break
            zellen.append((q * RES, ix, iy, bool(cap[iy, ix] >= wk)))
        if len(zellen) >= 2 and zellen[0][3] and zellen[1][3]: continue
        if offen and len(zellen) < 2: merken = False; continue
        for d, ix, iy, ok in zellen:
            if not ok: continue
            zx, zy = cell_xy(ix, iy); da = max(0.0, d - 0.15); A = (ex + dx * da, ey + dy * da)
            if pad_room(li, (cx, cy), A, net) >= wk / 2 and pad_room(li, A, (zx, zy), net) >= wk / 2:
                out.append((li * NN + iy * NX + ix, L, [[round(cx, 4), round(cy, 4)], [round(A[0], 4), round(A[1], 4)], [round(zx, 4), round(zy, 4)]]))
            break
        else:
            if offen: merken = False
    if merken: _FL[key] = tuple(out)
    return tuple(out)

def bga_eigene_lochkarte(viaH, vias):
    ""
    if not vias: return viaH
    out = viaH.copy()
    for x, y in sorted({(float(v['x']), float(v['y'])) for v in vias}):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rad = int(math.ceil(HOLE_C / RES)) + 2
        x1, x2 = max(0, ix - rad), min(NX, ix + rad + 1)
        y1, y2 = max(0, iy - rad), min(NY, iy + rad + 1)
        if x1 >= x2 or y1 >= y2: continue
        yy, xx = np.mgrid[y1:y2, x1:x2]
        px = X0 + (xx + .5) * RES; py = Y0 + (yy + .5) * RES
        for vy, vx in zip(*np.nonzero(out[y1:y2, x1:x2])):
            c = (vy + y1) * NX + vx + x1
            if c in VP: px[vy, vx], py[vy, vx] = VP[c][:2]
        d2 = (px - x) ** 2 + (py - y) ** 2
        out[y1:y2, x1:x2] &= (d2 >= HOLE_C ** 2) | (d2 <= 1e-12)
    return out


def bga_flucht(net, p, wk, capH, viaH, vbadf, rng, flaeche=False, soft=None, innen_w=None):
    ""
    b = BGA.get(id(p))
    ring_min = int(args.get('bga_ring_min', '2' if BGA_ESCAPE else '0')) if BGA_PLAN else 2
    if b is None: return None
    if BGA_ESCAPE and b['ring'] == 1 and 'bga_ring_min' not in args:


        bb = p['bbox']
        if b['pitch'] - max(bb[2]-bb[0],bb[3]-bb[1]) - 2*DRCP < wk - 1e-9:
            ring_min = 1


    supply_escape = BGA_ESCAPE and p.get('pintype') == 'power_in'
    if b['ring'] < ring_min and not flaeche and not supply_escape: return None
    inner = [i for i, L in enumerate(LAYERS) if L.startswith('In')]
    if not inner and not flaeche: return None
    if inner:
        q = max(0, b['ring'] - 2) % len(inner)
        inner = inner[q:] + inner[:q]
    candidates = b['pts']
    _vd = _vd_kosten().ravel() if VIA_DICHTE else None
    if BGA_PLAN:


        def preis(c):
            iy, ix = divmod(c, NX)



            via_enge = (sum(max(0.0, VIA_D - float(soft[il][iy, ix])) / VIA_D for il in range(NL))
                        if BGA_ESCAPE and soft is not None else 0.0)
            return ((float(vbadf[c]) + via_enge) * PF[0] + sum(float(hist[il * NN + c]) for il in range(NL)) +
                    (VD_MM * float(_vd[c]) if _vd is not None else 0.0),
                    math.dist((p['x'], p['y']), VP[c][:2]), c)
        candidates = sorted((c for c in candidates if rng[0] <= c % NX < rng[1] and rng[2] <= c // NX < rng[3]), key=preis)
    if VIA_DICHTE and not BGA_PLAN and _vd.any():
        candidates = sorted(candidates, key=lambda c: (VD_MM * float(_vd[c]) + float(vbadf[c]) * PF[0],
                                                       math.dist((p['x'], p['y']), VP[c][:2]), c))
    best = None; bestkey = None
    def stichpreis(tracks):



        cells = {}
        for tr in tracks:
            il = LI[tr['layer']]
            for a_, b_, width in zip(tr['pts'][:-1], tr['pts'][1:], tr['w']):
                count = max(2, int(math.ceil(math.dist(a_, b_) / RES)) + 1)
                for x_, y_ in np.linspace(a_, b_, count):
                    ix_, iy_ = int((x_ - X0) / RES), int((y_ - Y0) / RES)
                    if not (0 <= ix_ < NX and 0 <= iy_ < len(capH[il])): return math.inf
                    key = (il, iy_ * NX + ix_)
                    cells[key] = max(cells.get(key, 0.0), width)
        cost = 0.0
        for (il, cell), width in sorted(cells.items()):
            iy_, ix_ = divmod(cell, NX)
            if soft is not None:
                cost += PF[0] * max(0.0, width - float(soft[il][iy_, ix_])) / max(width, 1e-6)
            cost += float(hist[il * NN + cell])
        return cost

    for c in candidates:
        iy, ix = divmod(c, NX)
        if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): continue
        if not viaH[iy, ix] or (vbadf[c] and not BGA_PLAN) or not _vp_ok(net, VP[c][2]): continue
        x, y = VP[c][:2]; a = (p['x'], p['y']); v = (x, y)
        if net in PLANE_AREA and not PLANE_AREA[net].covers(_PT(x,y)): continue
        pts = _okt_zug([a, v]) if okt_netz(net) else [a, v]
        pts = [q for i, q in enumerate(pts) if i == 0 or math.dist(q, pts[i - 1]) > 1e-7]
        li = LI[b['layer']]
        if any(pad_room(li, a_, b_, net) < wk / 2 for a_, b_ in zip(pts[:-1], pts[1:])): continue
        trs = [dict(layer=b['layer'], pts=[list(q) for q in pts], w=[wk]*(len(pts)-1))] if len(pts) > 1 else []
        vs = [dict(x=x, y=y)]
        if flaeche:
            result = (None, trs, vs)
            if not BGA_ESCAPE: return result
            score = (preis(c)[0] if BGA_PLAN else 0.0) + stichpreis(trs)
            key = (score, math.dist(a, v), c)
            if math.isfinite(score) and (bestkey is None or key < bestkey): bestkey, best = key, result
            continue
        z = cell_xy(ix, iy)
        order = inner
        if BGA_PLAN and soft is not None:
            order = sorted(inner, key=lambda il: (PF[0] * max(0.0, wk - float(soft[il][iy, ix])) / max(wk, 1e-6) + float(hist[il * NN + c]), inner.index(il)))
        for il in order:




            iw = wk if innen_w is None else max(wk, innen_w)
            if capH[il][iy, ix] < iw: continue
            ip = _okt_zug([v, z]) if okt_netz(net) else [v, z]
            ip = [q for i, q in enumerate(ip) if i == 0 or math.dist(q, ip[i - 1]) > 1e-7]
            if any(pad_room(il, a_, b_, net) < iw/2 for a_, b_ in zip(ip[:-1], ip[1:])): continue
            it = [dict(layer=LAYERS[il], pts=[list(q) for q in ip], w=[iw]*(len(ip)-1))] if len(ip) > 1 else []
            result = (il*NN+c, trs+it, vs)
            if not BGA_ESCAPE: return result
            score = (preis(c)[0] if BGA_PLAN else 0.0) + stichpreis(trs + it)
            key = (score, math.dist(a, v), c, inner.index(il))
            if math.isfinite(score) and (bestkey is None or key < bestkey): bestkey, best = key, result
    return best














KETTE = KL is not None and args.get('kette', '1' if NLAG >= 4 else '0') == '1'
KETTE_VIA = float(args.get('kette_via', '10'))
KETTE_ABST = float(args.get('kette_abstand', '0.15'))
KETTE_MAX = float(args.get('kette_max', '6.0'))
KETTE_UMWEG = float(args.get('kette_umweg', '2.0')); KETTE_UMWEG_MM = float(args.get('kette_umweg_mm', '3.0'))
KETTE_PIN = float(args.get('kette_pin', '4.0'))
KETTEN = {}
KF_NACH = {}
KF_VOR = {}
_KZ_AUS = set()
KETTE_STAT = dict(glied=0, ohne_zone=0, zurueck=0, umweg=0, rest_frei=0)
KETTE_REST = KETTE and args.get('kette_rest', '1') == '1'
KETTE_REST_ANT = float(args.get('kette_rest_anteil', '0.2')); KETTE_REST_MM = float(args.get('kette_rest_mm', '5.0'))


def _luft_min(src, tgt):
    ""
    s = np.unique(np.asarray(src, dtype=np.int64) % NN); t = np.unique(np.fromiter(tgt, dtype=np.int64, count=len(tgt)) % NN)
    sx, sy = (s % NX) * RES, (s // NX) * RES; tx, ty = (t % NX) * RES, (t // NX) * RES
    best = np.inf
    for i in range(0, len(s), 512):
        d2 = (sx[i:i + 512, None] - tx[None, :]) ** 2 + (sy[i:i + 512, None] - ty[None, :]) ** 2
        best = min(best, float(d2.min()))
    return math.sqrt(best) * (1.4142 / math.sqrt(2.0))


def _plen(res):
    ""
    L = 0.0
    for a, b in zip(res[:-1], res[1:]):
        if a // NN != b // NN: L += 0.6; continue
        ia, ib = a % NN, b % NN
        L += RES * (1.4142 if (ia % NX != ib % NX and ia // NX != ib // NX) else 1.0)
    return L


def _pad_abst(p, q):
    a, b = p['bbox'], q['bbox']
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0.0), max(a[1] - b[3], b[1] - a[3], 0.0))


def _ketten_bauen():
    je = {}
    for r in sorted(KL.get('teile', {})):
        t = KL['teile'][r]; st = t.get('stuetzt') or {}
        if not str(t.get('funktion', '')).startswith('entkopplung_') or st.get('ueber_spule') or not t.get('C'): continue
        n, chip, pin = st.get('netz'), st.get('ref'), st.get('pin')
        if not n or chip is None or pin is None or n in GNDS or n not in bynet_g: continue
        ck = [k for k in bynet_g[n] if D['pads'][k]['ref'] == r]
        if len(ck) != 1 or not any(L in LI for L in D['pads'][ck[0]]['layers']): continue


        gruppe = tuple(sorted(str(x) for x in (st.get('pins') or [pin]))) if st.get('p3_gruppe') else ()
        if gruppe:


            instanzen = {}
            for k in bynet_g[n]:
                p = D['pads'][k]
                if p['ref'] == chip and str(p['num']) in gruppe:
                    instanzen.setdefault(str(p['num']), []).append(k)
            kandidaten = [ks[0] for ks in instanzen.values() if len(ks) == 1
                          and any(L in LI for L in D['pads'][ks[0]]['layers'])]
            if not kandidaten: continue
            pk = [min(kandidaten, key=lambda k: (_pad_abst(D['pads'][ck[0]], D['pads'][k]), k))]
        else:
            pk = [k for k in bynet_g[n] if D['pads'][k]['ref'] == chip and str(D['pads'][k]['num']) == str(pin)]
            if len(pk) != 1 or not any(L in LI for L in D['pads'][pk[0]]['layers']): continue
        if _pad_abst(D['pads'][ck[0]], D['pads'][pk[0]]) > KETTE_MAX: continue
        g = je.setdefault((chip, n, gruppe), dict(ketten={}, pins=set()))
        if gruppe:
            g.setdefault('anker', []).append((float(t['C']), r, pk[0]))
            pk = [gruppe]
        g['ketten'].setdefault(pk[0], []).append((float(t['C']), r, ck[0]))
        g['pins'].update(str(x) for x in (st.get('pins') or [pin]))
    for (chip, n, gruppe), g in sorted(je.items()):
        if gruppe:
            anker = min(g['anker'])[2]
            if not any(L in LI for L in D['pads'][anker]['layers']): continue
            ks = [dict(pins=[anker], caps=[c for _, _, c in sorted(cs)]) for cs in g['ketten'].values()]
        else:
            ks = [dict(pins=[pk], caps=[c for _, _, c in sorted(cs)]) for pk, cs in sorted(g['ketten'].items())]
        belegt = {k for K_ in ks for k in K_['pins'] + K_['caps']}
        for k in bynet_g[n]:
            p = D['pads'][k]
            if p['ref'] != chip or k in belegt or str(p['num']) not in g['pins'] or not any(L in LI for L in p['layers']): continue
            if gruppe and sum(D['pads'][j]['ref'] == chip and str(D['pads'][j]['num']) == str(p['num']) for j in bynet_g[n]) != 1: continue
            d, i = min((_pad_abst(p, D['pads'][K_['caps'][0]]), i) for i, K_ in enumerate(ks))
            if d <= KETTE_PIN: ks[i]['pins'].append(k)
        KETTEN.setdefault(n, []).extend(ks)
    for n in sorted(KETTEN):
        if n not in FLNETZE: continue
        for K_ in KETTEN[n]:
            for a, b in [(k, K_['caps'][0]) for k in K_['pins']] + list(zip(K_['caps'][:-1], K_['caps'][1:])):
                KF_NACH[a] = b; KF_VOR.setdefault(b, []).append(a)
    if KETTEN: print('Versorgungsketten (R22): %d Ketten in %d Netzen, davon %d Netze mit Flaeche' % (
        sum(len(v) for v in KETTEN.values()), len(KETTEN), sum(1 for n in KETTEN if n in FLNETZE)), flush=True)

if KETTE: _ketten_bauen()

def _rechteck_zellen(bx, r):
    ""
    ixa = max(0, int((bx[0] - r - X0) / RES)); ixb = min(NX, int((bx[2] + r - X0) / RES) + 1)
    iya = max(0, int((bx[1] - r - Y0) / RES)); iyb = min(NY, int((bx[3] + r - Y0) / RES) + 1)
    if ixb <= ixa or iyb <= iya: return np.zeros(0, dtype=np.int64)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    m = (np.maximum(bx[0] - GX, GX - bx[2]) <= r) & (np.maximum(bx[1] - GY, GY - bx[3]) <= r)
    yy, xx = np.nonzero(m)
    return (yy + iya) * NX + (xx + ixa)

def _kz(innen, spuren, vias, frei, wmin, abst=None):
    ""
    if abst is None: abst = KETTE_ABST
    rb = wmin / 2 + abst; rv = VIA_R + abst; zb, zv = [], []
    for p in innen:
        for L in p['layers']:
            if L in LI: zb.append(LI[L] * NN + _rechteck_zellen(p['bbox'], rb))
        zv.append(_rechteck_zellen(p['bbox'], rv))
    for t in spuren:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            zb.append(seg_cells(li, a, b, w + 2 * rb)); zv.append(seg_cells(0, a, b, w + 2 * rv))
    for v in vias:
        q = (v['x'], v['y'])
        for li in range(NL): zb.append(seg_cells(li, q, q, VIA_D + 2 * rb))
        zv.append(seg_cells(0, q, q, VIA_D + 2 * rv))
    zb = np.concatenate(zb) if zb else np.zeros(0, dtype=np.int64)
    zv = np.concatenate(zv) if zv else np.zeros(0, dtype=np.int64)
    for p in frei:
        zb = zb[~np.isin(zb % NN, _rechteck_zellen(p['bbox'], rb + 0.05))]
        zv = zv[~np.isin(zv, _rechteck_zellen(p['bbox'], 0.0))]
    return zb, zv


def _kz_karten(capH, viaH, zb, zv):
    ""
    c2 = _Karten(); c2.flach = capH.flach.copy(); c2.flach[zb] = -1.0
    for li in range(NL): c2.append(c2.flach[li * NN:(li + 1) * NN].reshape(NY, NX))
    v2 = np.array(viaH, dtype=bool, copy=True); v2.reshape(-1)[zv] = False
    return c2, v2


def _kz_vorbereiten(r, net):
    ""
    innen = set()
    for t in r['tracks']:
        if 'kz' in t: innen.update(t['kz'][1:])
    eigen = {0: {k for k in bynet_g[net] if k not in innen}}
    for t in r['tracks']:
        if t.get('kette'): eigen.setdefault(t['kette'], set()).update(t.get('kp') or ())
    geo = {k: {LI[L]: shapely.union_all([Polygon(q) for q in D['pads'][k]['polys'][L]]) for L in D['pads'][k]['layers'] if L in LI} for k in bynet_g[net]}
    G = {}
    for g in sorted(eigen):
        own = eigen[g]; e = {}; pf = {}; fr = {}
        for li in range(NL):
            ow = [geo[k][li] for k in sorted(own) if li in geo[k]]
            e[li] = shapely.union_all(ow) if ow else None
            pf[li] = e[li].buffer(KETTE_ABST + 0.1) if ow else None
            fr[li] = [geo[k][li] for k in bynet_g[net] if k not in own and li in geo[k]]
        G[g] = dict(eigen=e, puffer=pf, fremd=fr)
    xs = [q[0] for t in r['tracks'] if t.get('kette') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in innen for i in (0, 2)]
    ys = [q[1] for t in r['tracks'] if t.get('kette') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in innen for i in (1, 3)]
    box = (min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5) if xs else (0.0, 0.0, 0.0, 0.0)
    return dict(G=G, box=box)


def _kz_breite(kz, r, breiten, ti, a, b, li):
    ""
    g = r['tracks'][ti].get('kette') or 0; bx = kz['box']
    if g == 0 and (max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]): return 9.0
    G = kz['G'][g]; seg = _LS([a, b]) if a != b else _PT(a)
    if G['eigen'][li] is not None: seg = seg.difference(G['eigen'][li])
    if seg.is_empty: return 9.0
    d = 9.0
    for q in G['fremd'][li]: d = min(d, seg.distance(q))
    pf = G['puffer'][li]
    for tj, t2 in enumerate(r['tracks']):
        if (t2.get('kette') or 0) == g or LI[t2['layer']] != li: continue
        for a2, b2, w2 in zip(t2['pts'][:-1], t2['pts'][1:], breiten[tj]):
            if min(a2[0], b2[0]) - 2.0 > max(a[0], b[0]) or max(a2[0], b2[0]) + 2.0 < min(a[0], b[0]) or \
               min(a2[1], b2[1]) - 2.0 > max(a[1], b[1]) or max(a2[1], b2[1]) + 2.0 < min(a[1], b[1]): continue
            s2 = _LS([a2, b2]) if a2 != b2 else _PT(a2)
            if pf is not None: s2 = s2.difference(pf)
            if not s2.is_empty: d = min(d, seg.distance(s2) - w2 / 2)
    for v in r['vias']:
        if (v.get('kette') or 0) == g: continue
        q = _PT(v['x'], v['y'])
        if pf is None or not pf.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - KETTE_ABST)


def _kf_zone(k, p, wmin):
    ""
    vor, st = [], [k]
    while st:
        for a in KF_VOR.get(st.pop(), ()): vor.append(a); st.append(a)
    if not vor: return None
    sp = [t for a in vor if 'GND#%d' % a in U for t in U['GND#%d' % a]['tracks']]
    vv = [v for a in vor if 'GND#%d' % a in U for v in U['GND#%d' % a]['vias']]
    return _kz([D['pads'][a] for a in vor], sp, vv, [p], wmin)


def _kf_glied(u, k, p, net, C, src, fst, wk, karten_):
    ""
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = karten_
    q = D['pads'][KF_NACH[k]]; bx, qb = p['bbox'], q['bbox']
    Cb = dict(C, via=3.0 * KETTE_VIA if set(p['layers']) & set(q['layers']) else 0.0)
    win = (min(bx[0], qb[0]) - 3.0, min(bx[1], qb[1]) - 3.0, max(bx[2], qb[2]) + 3.0, max(bx[3], qb[3]) + 3.0)
    if not _im_fenster(win, rng): return False
    tgt = set()
    for L, cells in q['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
    fzt = {c: (l_, pts) for c, l_, pts in flucht(net, q, wk, capH, rng)}; tgt.update(fzt)
    src = sorted({c for c in src if c not in tgt})
    if not src or not tgt: return False
    res = None; zn = _kf_zone(k, p, Cb['wmin'])
    if zn is not None:
        cH, vH = _kz_karten(capH, viaH, *zn)
        res = astar_pf(src, tgt, qb, cH, capS, vH, viaBad, Cb, win)
        if not res: KETTE_STAT['ohne_zone'] += 1
    if not res: res = astar_pf(src, tgt, qb, capH, capS, viaH, viaBad, Cb, win)
    if not res: return False
    trs, vs = path_to_geometry(res, capC, dict(Cb, w=Cb['wmin'])) if len(res) > 1 else ([], [])
    if sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > KETTE_UMWEG * _pad_abst(p, q) + KETTE_UMWEG_MM:
        KETTE_STAT['umweg'] += 1; return None
    for c_, f_ in ((res[0], fst), (res[-1], fzt)):
        if c_ in f_: trs.append(dict(layer=f_[c_][0], pts=f_[c_][1], w=[wk, wk]))
    for t in trs: t['kette'] = True
    add_unit(u, net, trs, vs); KETTE_STAT['glied'] += 1
    return True


def n_sv(r):
    ""
    return sum(1 for v in r['vias'] if not v.get('rv') and not v.get('gr'))
def vg_fehl(u):
    ""
    vs = U[u]['vias']
    return sum(max(0, v['n'] - 1 - sum(1 for w in vs if w.get('zu') == [v['x'], v['y']])) for v in vs if v.get('n'))

















ABG_AN = TAPS and args.get('abgriff_abstand', '1' if KL else '0') == '1'
ABG_CLR = float(args.get('abgriff_clr', '%g' % CLR))
ABG_BAUM = TAPS and args.get('abgriff_baum', '1' if KL else '0') == '1'
ABG_LAGE = float(args.get('abgriff_lage', '0')) if TAPS else 0.0


ABG_VIA = float(args.get('abgriff_via', '1')) if TAPS else 1.0
ABG_LAGE_D = float(args.get('abgriff_lage_d', '0.8'))
ABZW = args.get('abzweig', '0') == '1' or R27_STAMM
ABZW_D = float(args.get('abzweig_d', '0.6')); ABZW_L = float(args.get('abzweig_l', '1.0')); ABZW_GRAD = float(args.get('abzweig_grad', '60'))
ABZW_FAK = float(args.get('abzweig_fak', '3.0')); ABZW_MEHR = float(args.get('abzweig_mehr', '1.5')); STERN_R = float(args.get('stern_r', '0.3'))
TOPO_STAT = dict(abgriffe=0, abgriff_ohne_zone=0, abzweig_geprueft=0, abzweig_mangel=0, abzweig_besser=0, stern_gesperrt=0)


def _arm_richtung(pts, i, schritt, p, r=0.15):
    ""
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += schritt
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0


def _arme_bei(spuren, li, p):
    ""
    roh = []
    for t in spuren:
        if LI[t['layer']] != li: continue
        P = t['pts']
        for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
            if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            f = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
            pf = (a[0] + f * dx, a[1] + f * dy)
            if math.dist(p, pf) > 0.036: continue
            roh += [_arm_richtung(P, k + 1, 1, pf), _arm_richtung(P, k, -1, pf)]
    arme = []
    for w in sorted(x for x in roh if x is not None):
        if not arme or w - arme[-1] > 3.0: arme.append(w)
    if len(arme) > 1 and arme[0] + 360.0 - arme[-1] <= 3.0: arme.pop()
    return arme


def _parallel_mm(trs, spuren):
    ""
    ges = 0.0
    for t in trs:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.dist(a, b)
            if L < 1e-6: continue
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            for t2 in spuren:
                if t2['layer'] != t['layer']: continue
                for c, d in zip(t2['pts'][:-1], t2['pts'][1:]):
                    L2 = math.dist(c, d)
                    if L2 < 1e-6 or abs(ux * (d[1] - c[1]) - uy * (d[0] - c[0])) / L2 > 0.2588: continue
                    sc = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy; sd = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
                    lo, hi = max(0.0, min(sc, sd)), min(L, max(sc, sd))
                    if hi - lo <= 1e-6 or abs(sd - sc) < 1e-9: continue
                    nc = -(c[0] - a[0]) * uy + (c[1] - a[1]) * ux; nd = -(d[0] - a[0]) * uy + (d[1] - a[1]) * ux
                    e0 = nc + (nd - nc) * (lo - sc) / (sd - sc); e1 = nc + (nd - nc) * (hi - sc) / (sd - sc)
                    if max(abs(e0), abs(e1)) < ABZW_D and min(abs(e0), abs(e1)) > 0.02: ges += hi - lo
    return ges


def _abzw_maengel(trs, spuren, li, p, am_pad):
    ""
    arme = [] if am_pad else _arme_bei(spuren, li, p)
    wn = None
    for t in trs:
        if LI[t['layer']] == li and math.dist(t['pts'][0], p) < 0.08: wn = _arm_richtung(t['pts'], 1, 1, p); break
    wk = min((abs((wn - a + 180.0) % 360.0 - 180.0) for a in arme), default=180.0) if wn is not None else 180.0
    par = _parallel_mm(trs, spuren)
    return (len(arme) >= 3, par if par > ABZW_L else 0.0, wk < ABZW_GRAD - (1e-6 if R27_STAMM else 1.0))


def _streifen(ex, spuren, breit, fak):
    ""
    m = np.zeros(NL * NN, dtype=bool)
    for t in spuren:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): m[seg_cells(li, a, b, w + 2 * breit)] = True
    e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    e[m] *= np.float32(fak)
    return e


def _abg_vorbereiten(r, net):
    ""
    spur = {li: [] for li in range(NL)}; enden = []
    for t in r['tracks']:
        if not t.get('abg'): continue
        enden += [t['pts'][0], t['pts'][-1]]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if a != b: spur[LI[t['layer']]].append((_LS([a, b]), w / 2))
    vv = [_PT(v['x'], v['y']) for v in r['vias'] if v.get('abg')]
    frei = {}
    for li in range(NL):
        g = [g2 for n2, g2 in _PADG[li] if n2 == net and any(g2.distance(_PT(e)) <= 0.05 for e in enden)]
        frei[li] = shapely.union_all(g).buffer(ABG_CLR + 0.1) if g else None
    xs = [e[0] for e in enden] + [q.x for q in vv]; ys = [e[1] for e in enden] + [q.y for q in vv]
    for li in range(NL):
        for s2, _ in spur[li]: b_ = s2.bounds; xs += [b_[0], b_[2]]; ys += [b_[1], b_[3]]
    return dict(spur=spur, vias=vv, frei=frei, box=(min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5))


def _abg_breite(ab, a, b, li):
    ""
    bx = ab['box']
    if max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]: return 9.0
    seg = _LS([a, b]) if a != b else _PT(a); fr = ab['frei'][li]
    if fr is not None: seg = seg.difference(fr)
    if seg.is_empty: return 9.0
    d = 9.0
    for s2, h2 in ab['spur'][li]:
        if fr is not None: s2 = s2.difference(fr)
        if not s2.is_empty: d = min(d, seg.distance(s2) - h2)
    for q in ab['vias']:
        if fr is None or not fr.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - ABG_CLR)


def route_unit(u):
    arbeit_schritt('einheit')
    ok = _route_unit_impl(u)
    net = U.get(u,{}).get('net',u)
    if not ok and PINZUGANG and not _pz_active(net) and net in PZ_BREITEN and not u.startswith(('GND#','FIX#')):
        C=cls(net);capH,viaH=hard_maps(net)
        blocked=[]
        for k in bynet[net]:
            p=D['pads'][k]
            cells=[(LI[L],cs) for L,cs in p['cells'].items() if L in LI]
            free=any(np.any(capH[li].ravel()[cs]>=C['wmin']) for li,cs in cells)



            cut = '%s:%s' % (p['ref'],p['num']) in UNERREICHBAR.get(net,[])
            if cut or (not free and not flucht(net,p,C['wmin'],capH,_VOLL)):blocked.append(id(p))
        if blocked:
            PZ_LOKAL[net]=set(blocked);_pad_room_cached.cache_clear()
            for key in list(_ERR):
                if key[0]==net:_ERR.pop(key)
            _ERR_NETZE.difference_update(key for key in list(_ERR_NETZE) if key[0]==net)
            print('Pinzugang lokal:',net,'abgeschnittene Pads',len(blocked),flush=True)
            del_unit(u)
            ok=_route_unit_impl(u)
    return ok


_PLANE_GROUP_ACTIVE = [None]


def route_plane_group(net):
    ""
    old=_PLANE_GROUP_ACTIVE[0];_PLANE_GROUP_ACTIVE[0]=net
    tr=[];vs=[];complete=True;del_unit(net)
    try:
        for k in PLANE_GROUPS[net]:
            child='GND#%d'%k
            try:
                ok=_route_unit_impl(child);r=U.get(child)
                if r:tr.extend(r['tracks']);vs.extend(r['vias'])
                complete &= bool(ok)
            finally:del_unit(child)
            del_unit(net);add_unit(net,net,tr,vs)
            vs=list(U[net]['vias'])
        return complete
    finally:_PLANE_GROUP_ACTIVE[0]=old


def _route_unit_impl(u):
    ""
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if paar_einheit(u): return route_paar(u)
    if u in PLANE_GROUPS: return route_plane_group(u)
    ASTAR_COST[0] = None; ASTAR_HINT[0] = None; VIA_ZU[0] = None
    NET_CUR[0] =D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
    if u.startswith('GND#'):
        k = int(u[4:]); p = D['pads'][k]; net = p['net']
        _direct_plane = any(net in FL_ZUORDNUNG.get(L,[]) for L in PLANE_SIGNALS)
        if _direct_plane and (net not in PLANE_AREA or PLANE_AREA[net].is_empty):return False
        C = dict(w=max(0.35, BAHN_MIN), wmin=max(0.15, BAHN_MIN), pen=0.5, via=0.0, inner=99.0)
        bx = p['bbox']
        box = (bx[0] - 6.1, bx[1] - 6.1, bx[2] + 6.1, bx[3] + 6.1)
        capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = karten(net, box, C['w'])
        L = next(x for x in p['layers'] if x in LI)
        src = (LI[L] * NN + p['cells'][L]).tolist()
        _wk = min(C['wmin'], HALS_W) if hals_aktiv(net, C, C['wmin'], True) else C['wmin']
        _fst = {c: (l_, pts) for c, l_, pts in flucht(net, p, _wk, capH, rng)}
        src += [c for c in sorted(_fst) if c not in src]
        if KETTE and not _direct_plane and k in KF_NACH and (net, k) not in _KZ_AUS:
            _kg = _kf_glied(u, k, p, net, C, src, _fst, _wk, (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf))
            if _kg: return True
            if _kg is False: _KZ_AUS.add((net, k))
            KETTE_STAT['zurueck'] += 1
        if BGA_FANOUT:
            _bf = bga_flucht(net, p, _wk, capH, viaH, vbadf, rng, flaeche=True, soft=capS)
            if _bf: add_unit(u, net, _bf[1], _bf[2]); return True
        gnd_tgt = set()
        _plane_distance = PLANE_AREA[net].distance(_PT(p['x'],p['y'])) if _direct_plane and net in PLANE_AREA else 0.
        if _plane_distance > 6.:



            for owner,r in U.items():
                if _PLANE_GROUP_ACTIVE[0]==net and owner!=net:continue
                if r['net']!=net or not any(PLANE_AREA[net].covers(_PT(v['x'],v['y'])) for v in r['vias']):continue
                for t in r['tracks']:
                    for q in t['pts']:
                        ix,iy=int((q[0]-X0)/RES),int((q[1]-Y0)/RES)
                        if 0<=ix<NX and 0<=iy<NY:gnd_tgt.add(LI[t['layer']]*NN+iy*NX+ix)
        if k in EPM:
            _ev = ep_setzen(k, viaH, vbadf, rng)
            if _ev: add_unit(u, net, [], _ev); return True
        _epr = EP_RES.get(net) if k not in EPM else None
        if _epr is not None: viaH = viaH & ~_epr
        _vz = None; _vk = 0.0
        if VAP and not _direct_plane:
            _vzc, _vzw = _vap_zellen(k); _VZ[_vzc] = _vzw; _vz = _VZ
            _vk = VAP_K * (VAP_WF if k in VAP_WICHTIG else 1.0)
        if VIA_DICHTE:
            _dv = _vd_kosten().ravel() * np.float32(VD_MM)
            if _dv.any():


                _vz = _dv if _vz is None else _vk * _vz.astype(np.float64) + _dv
                _vk = 1.0

        _zs = _kf_zone(k, p, C['wmin']) if KETTE and not _direct_plane and k in KF_VOR and (net, k, 2) not in _KZ_AUS else None
        res=None
        for _zn in ([_zs] if _zs is not None else []) + [None]:
            _windows = (1.8,3.5,6.0) if _plane_distance <= 6. else (max(6.,_plane_distance+2.),)
            for ml in _windows:
                win = (bx[0] - ml, bx[1] - ml, bx[2] + ml, bx[3] + ml)
                if not _im_fenster(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = karten_groesser(net, box, win, C['w'])
                    if _b2 is not None: box = _b2
                    if _epr is not None: viaH = viaH & ~_epr
                _cH, _vH = (capH, viaH) if _zn is None else _kz_karten(capH, viaH, *_zn)
                if _direct_plane and _plane_distance<=6.:
                    _vH=_vH & PLANE_MASK[net]
                targets=gnd_tgt;target_box=(p['x'],p['y'],p['x'],p['y']);via_target=True
                if _plane_distance>6.:
                    targets=gnd_tgt | plane_feeder_targets(net,p,LI[L],_vH,viaBad,rng)
                    if not targets:continue
                    xy=[cell_xy((c%NN)%NX,(c%NN)//NX) for c in sorted(targets)]
                    target_box=(min(q[0] for q in xy),min(q[1] for q in xy),max(q[0] for q in xy),max(q[1] for q in xy))
                    via_target=False
                res = astar_pf(src, targets, target_box, _cH, capS, _vH, viaBad, C, win, via_target=via_target, viaz=_vz, vzk=_vk)
                if res:
                    konf = any(capCf[q] < C['wmin'] for q in res) or (res[-1] not in gnd_tgt and bool(vbadf[res[-1] % NN]))
                    if not konf or ml >= 6.0: break
                    if NB is not None and NB.info[0] == 0: break
            if res or _zn is None: break
            _KZ_AUS.add((net, k, 2)); KETTE_STAT['ohne_zone'] += 1
        if VAP and not _direct_plane and _vz is not None: _VZ[_vzc] = 1.0
        if not res: return False
        trs, _mid_vias = path_to_geometry(res, capC, dict(C, w=C['wmin'])) if len(res) > 1 else ([], [])
        if res[0] in _fst: trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        n = res[-1]; vx, vy = cell_xy((n % NN) % NX, (n % NN) // NX)
        _s = _vp_xy(net, n % NN) if VP and n not in gnd_tgt else None
        if _s and _direct_plane and not PLANE_AREA[net].covers(_PT(*_s)):_s=None
        if _s:
            vx, vy = _s
            for t in trs:
                if LI[t['layer']] == n // NN and math.hypot(t['pts'][-1][0] - vx, t['pts'][-1][1] - vy) < 0.05: t['pts'][-1] = [round(vx, 4), round(vy, 4)]
        add_unit(u, net, trs, _mid_vias+([] if n in gnd_tgt else [dict(x=round(vx, 4), y=round(vy, 4))]))
        return True
    net = u; C = cls(net)
    pads = [D['pads'][k] for k in bynet[net]]
    pbx = (min(p['bbox'][0] for p in pads), min(p['bbox'][1] for p in pads), max(p['bbox'][2] for p in pads), max(p['bbox'][3] for p in pads))
    box = (pbx[0] - 8.5, pbx[1] - 8.5, pbx[2] + 8.5, pbx[3] + 8.5)
    _m0 = FENSTER if FENSTER else 3.0
    ASTAR_HINT[0] = (max(1, int((pbx[1] - _m0 - Y0) / RES)), min(NY - 2, int((pbx[3] + _m0 - Y0) / RES)),
                     max(1, int((pbx[0] - _m0 - X0) / RES)), min(NX - 2, int((pbx[2] + _m0 - X0) / RES)))
    Cs = cls('/__signal__'); _wmax = max(C['w'], Cs['w'])
    _sz_fl = strom_modell(net)
    if _sz_fl: _wmax = max(_wmax, sum(map(float,_sz_fl['last'].values())) / (STROM_J * min(STROM_T.values())))
    RK = rv_karte(net, box, C['via'] * VIA_FAK[0]) if RUECK > 0 else None
    _vank = via_zuschlag(u, net, C, box) if (VIA_ANKER > 0 or K12 > 0) else None
    _vank_alt = _vank
    if VIA_DICHTE:
        _dv = _vd_kosten().ravel() * np.float32(VD_MM)
        if _dv.any(): _vank = _dv if _vank is None else _vank + _dv
    if RK is not None or _vank is not None: VIA_ZU[0] = (RK['vzu'] if RK else None, RV_PAAR if RK else None, _vank)
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = karten(net, box, _wmax)

    k = net.split('/')[-1]; taps = []; kel = {}
    if ABG_KL:
        _ak = {'%s:%s' % (p['ref'], p['num']): i for i, p in enumerate(pads)}
        _ae = [e for e in KL['abgriffe'].get(net, []) if e['pad'] in _ak]
        taps = sorted(_ak[e['pad']] for e in _ae)
        for e in _ae:
            if e.get('start') in _ak: kel[_ak[e['pad']]] = [_ak[e['start']]]

    power = [i for i in range(len(pads)) if i not in taps]
    EXTRA = stoer_extra(net)
    if VIA_DICHTE and KL and (KL['netze'].get(net) or {}).get('art') in ('strom', 'versorgung') and VD_STROM > 0:


        _dc = np.square(np.maximum(np.float32(0), _vd_fenster_max(_VD_ZAHL.astype(np.float32)) - np.float32(VD_SCHWELLE * VD_FENSTER ** 2)))
        if _dc.any():
            _de = np.tile(1.0 + np.float32(VD_STROM) * _dc.ravel(), NL)
            EXTRA = _de if EXTRA is None else EXTRA * _de
    if net in PAAR_NETZE: EXTRA = paar_extra(net, C, capS, EXTRA)
    _abg_sp, _abg_vi, _abg_kn, _abg_pads = [], [], [], []
    _stern = set(); _pn_set = set()

    KT = None; _innen = set(); stamm = power
    if KETTE and net in KETTEN and not taps:
        _lok = {k_: i for i, k_ in enumerate(bynet[net])}
        KT = [dict(nr=_n, pins=[_lok[k_] for k_ in K_['pins']], caps=[_lok[k_] for k_ in K_['caps']], pin_ok=[], ende=-1, spuren=[], vias=[], kn=[], stop=False)
              for _n, K_ in enumerate(KETTEN[net])]
        _innen = {i for K_ in KT for i in K_['pins'] + K_['caps'][:-1]}
        stamm = [i for i in power if i not in _innen]
    BF = {}; BF_NODES = {}; _bf_done = set()
    _bga_own_holes = BGA_PLAN and (args.get('bga_eigenvia', '0') == '1' or BGA_ESCAPE)
    if BGA_FANOUT and (KT is None or BGA_PLAN) and not taps:
        _bf_viaH = viaH
        _bw = min(C['wmin'], HALS_W) if hals_aktiv(net, C, C['wmin']) else C['wmin']
        for i in sorted(power, key=lambda i: (BGA.get(id(pads[i]), {}).get('ring', -1), pads[i]['ref'], pads[i]['num'])):
            _bf = bga_flucht(net, pads[i], _bw, capH, _bf_viaH, vbadf, rng, soft=capS,
                             innen_w=C['wmin'] if args.get('bga_innen_breite', '0') == '1' or BGA_ESCAPE else None)
            if _bf:
                BF[i] = _bf; _old = BF_NODES.setdefault(_bf[0], ([], []))
                _old[0].extend(_bf[1]); _old[1].extend(_bf[2])


                if _bga_own_holes: _bf_viaH = bga_eigene_lochkarte(_bf_viaH, _bf[2])
    _bga_reserved_vias = [v for bf in BF.values() for v in bf[2]] if _bga_own_holes else []
    if _sz_fl and not stamm:
        STROM_AUDIT[net]=dict(abgelehnt='strom_zuerst: Quelle/Last nur als Abgriffe, kein Stromstamm ' + net)
        add_unit(u,net,[],[]);return False
    cx = sum(pads[i]['x'] for i in stamm) / len(stamm); cy = sum(pads[i]['y'] for i in stamm) / len(stamm)
    first = min(stamm, key=lambda i: math.hypot(pads[i]['x'] - cx, pads[i]['y'] - cy))
    FL = KL['netze'].get(net, {}).get('fluss') if KL else None
    if FL and FL.get('quelle') and args.get('fluss_start') == '1':
        qs = [i for i in power if '%s:%s' % (pads[i]['ref'], pads[i]['num']) in FL['quelle']]
        if qs: first = qs[0]
        if first in _innen: first = next(K_['caps'][-1] for K_ in KT if first in K_['pins'] + K_['caps'])
    try: SP = strom_plan(net, pads, stamm, KT)
    except ValueError as e:


        STROM_AUDIT[net]=dict(abgelehnt=str(e))
        add_unit(u,net,[],[]);return False
    if SP: first = SP[0]
    tree = [first]; done_xy = [(pads[first]['x'], pads[first]['y'])]
    rest = [i for i in stamm if i != first]
    all_tr, all_v, pathnodes, ok = [], [], [], True
    if globals().get('PZ_REST') and HART_FREMD[0] and globals().get('_LOOP_DEADLINE') is not None:
        _PZ_PARTIAL[0] = (u, net, all_tr, all_v)
    _stiche = set()
    order = []
    while rest:
        j = min(rest, key=lambda i: min(math.hypot(pads[i]['x'] - x, pads[i]['y'] - y) for x, y in done_xy))
        rest.remove(j); order.append(j); done_xy.append((pads[j]['x'], pads[j]['y']))
    if SP: order = SP[1]
    order += taps
    done_xy = [(pads[first]['x'], pads[first]['y'])]
    jobs = [(j, None) for j in order]
    _kzz = None; _kzr = None; _kzk = None; tree_vg = tree
    if KT is not None:
        _gl = []
        for K_ in KT:
            _gl.append((K_['caps'][0], (K_, 0, [K_['pins'][0]])))
            _gl += [(q_, (K_, 0, None)) for q_ in K_['pins'][1:]]
            _gl += [(b_, (K_, s_ + 1, [a_])) for s_, (a_, b_) in enumerate(zip(K_['caps'][:-1], K_['caps'][1:]))]
        jobs = _gl + jobs; tree_vg = list(tree)
    _hint0 = ASTAR_HINT[0]
    for j, gl in jobs:
        if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
        if VIA_DICHTE and all_v:
            _dv = _vd_kosten(all_v).ravel() * np.float32(VD_MM)
            _vank = _dv if _vank_alt is None else _vank_alt + _dv
            VIA_ZU[0] = (RK['vzu'] if RK else None, RV_PAAR if RK else None, _vank)
        p = pads[j]; is_tap = j in taps; Cb = (dict(Cs, via=Cs['via'] * ABG_VIA) if ABG_VIA != 1.0 else Cs) if is_tap else C
        _sz_parent = SP[2].get(j, first) if SP else None
        if SP and not is_tap:
            _sz_key = ('stamm',j) if gl is None else ('kette',gl[0]['nr'],gl[1],j)
            _sz_I = SP[3].get(_sz_key, sum(map(float,_sz_fl['last'].values())))
            Cb = strom_klasse(Cb,_sz_I,pads[_sz_parent] if gl is None else pads[(gl[2] or gl[0]['pins'])[0]],p)
            Cb['strom_vias'] = tuple((v['x'],v['y']) for v in all_v)
        ASTAR_HINT[0] = _hint0 if gl is None else None
        _zone = None
        _abg_z = False; _ex = EXTRA
        if gl is not None:
            K_, _stufe, _qp = gl
            if K_['stop'] or (_stufe > 0 and K_['ende'] != _stufe - 1): continue
            if _qp is None: _qp = K_['pin_ok'] + K_['caps'][:1]
            src = list(K_['kn']) if gl[2] is None else []
            for i in _qp:
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if any(set(pads[i]['layers']) & set(p['layers']) for i in _qp) and not (BGA_PLAN and (j in BF or any(i in BF for i in _qp))):
                Cb = dict(Cb if SP else C, via=Cb['via'] * KETTE_VIA)
            if (net, bynet[net][j]) not in _KZ_AUS:
                _eig = (set(K_['pins']) | {K_['caps'][0]}) if _stufe == 0 else (set(_qp) | {j})
                _zone = _kz([pads[i] for i in power if i not in _eig], K_['spuren'] if _stufe > 0 else [], K_['vias'] if _stufe > 0 else [],
                            [pads[i] for i in sorted(_eig)], Cb['wmin'])
        elif j in kel:
            src = list(_abg_kn) if ABG_BAUM else []
            for i in kel[j] + (_abg_pads if ABG_BAUM else []):
                for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            _strom = [t for t in all_tr if not t.get('abg')]
            if ABG_AN:
                _kg = set(kel[j]) | {t_ for t_ in kel if kel[t_] == kel[j]}
                _zone = _kz([pads[i] for i in range(len(pads)) if i not in _kg], _strom, [v for v in all_v if not v.get('abg')],
                            [pads[i] for i in sorted(_kg)], Cb['wmin'], ABG_CLR + QM)
                _abg_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
            if ABG_LAGE > 0 and _strom:
                _ex = _streifen(EXTRA, _strom, ABG_LAGE_D, 1.0 + 2.0 * Cb['via'] * VIA_FAK[0] / ABG_LAGE)
        else:
            _si_kette = SI_STICH and ist_schnell(net) and gl is None and not is_tap
            _sz_sources = [_sz_parent] if SP and not is_tap and _sz_parent in tree else ([] if SP and not is_tap else None)
            src = [] if _si_kette or _sz_sources is not None else ([q for q in pathnodes if q not in _stern] if _stern else list(pathnodes))
            for i in (_sz_sources if _sz_sources is not None else (tree[-1:] if _si_kette else tree)):
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if KT is not None and not is_tap:
                if _kzz is None:
                    _zi = [pads[i] for K_ in KT for i in K_['pin_ok'] + K_['caps'][:max(K_['ende'], 0)]]
                    _zf = [pads[K_['caps'][K_['ende']]] for K_ in KT if K_['ende'] >= 0]
                    _kzz = _kz(_zi, [t for K_ in KT for t in K_['spuren']], [v for K_ in KT for v in K_['vias']], _zf, C['wmin'])
                    _kzr = _kz(_zi, [], [], _zf, C['wmin'])
                    tree_vg += [i for K_ in KT if K_['ende'] >= 0 and K_['caps'][K_['ende']] == first for i in K_['pin_ok'] + K_['caps'][:K_['ende']]]
                if len(_kzz[0]) and (net, bynet[net][j]) not in _KZ_AUS: _zone = _kzz
                elif len(_kzr[0]) and (net, bynet[net][j], 1) not in _KZ_AUS: _zone = _kzr
            if ABG_AN and _abg_sp and _zone is None:
                _zone = _kz([], _abg_sp, _abg_vi, [pads[i] for i in sorted({i_ for t_ in kel for i_ in kel[t_]} | set(_abg_pads))], Cb['wmin'], ABG_CLR + QM)
                _abg_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
        tgt = set()
        if j in BF: tgt.add(BF[j][0])
        else:
            for L, cells in p['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
        if j in BF and BF[j][0] in src:


            if BGA_PLAN and gl is None:
                _c = BF[j][0]
                for _i in tree + [j]:
                    if _i in BF and BF[_i][0] == _c and _i not in _bf_done:
                        _bf_done.add(_i); all_tr += BF[_i][1]; all_v += BF[_i][2]
                _stiche.add(_c)
                tree.append(j); continue
            if not BGA_PLAN:
                all_tr += BF[j][1]; all_v += BF[j][2]; tree.append(j); continue
        if gl is None and not is_tap and j not in kel:
            tcells = set()
            for i in tree:
                for L, cells in pads[i]['cells'].items(): tcells.update((LI[L] * NN + cells).tolist())
            if tgt & tcells: tree.append(j); continue
        _wk = min(Cb['wmin'], HALS_W) if hals_aktiv(net, Cb, Cb['wmin']) else Cb['wmin']
        _fst, _fzt = {}, {}
        if FLUCHT:
            for i in (_qp if gl is not None else (kel[j] if j in kel else (_sz_sources if _sz_sources is not None else (tree[-1:] if SI_STICH and ist_schnell(net) and not is_tap else tree)))):
                for c, l_, pts in flucht(net, pads[i], _wk, capH, rng): _fst[c] = (l_, pts)
            for c, l_, pts in flucht(net, p, _wk, capH, rng): _fzt[c] = (l_, pts)
            _gem = sorted(c for c in _fzt if c in _fst)
            if _gem:


                _c = _gem[0]; trs = [dict(layer=_fzt[_c][0], pts=_fzt[_c][1], w=[_wk] * (len(_fzt[_c][1])-1))]
                if _c not in _stiche: trs.append(dict(layer=_fst[_c][0], pts=_fst[_c][1], w=[_wk] * (len(_fst[_c][1])-1)))
                _stiche.add(_c)
                if is_tap:
                    for t in trs: t['sig'] = True
                    if j in kel and (ABG_AN or ABG_BAUM):
                        for t in trs: t['abg'] = True
                        _abg_sp += trs; _abg_pads.append(j)
                else: _kn = geo_nodes(trs, []); pathnodes.extend(_kn); _pn_set.update(_kn)
                all_tr += trs; tree.append(j); continue
            src.extend(_fst); tgt.update(_fzt)
        _bf_shared = sorted(set(src) & tgt) if BGA_PLAN and gl is not None and j in BF else []
        src = sorted(set(src)) if _bf_shared else sorted({q for q in src if q not in tgt})
        if j in kel:
            src_ok = [q for q in src if capCf[q] >= Cb['wmin']]
            if src_ok: src = src_ok
        if STROM_VIA_PREIS and VG and not is_tap and not SP:
            _mit_vg = [i for K_ in (KT or []) if K_['ende'] >= 0 and K_['caps'][K_['ende']] == j for i in K_['pin_ok'] + K_['caps'][:K_['ende']]]
            _via_n = vg_anzahl(net, pads, tree_vg, j, _mit_vg)
            if _via_n > 1: Cb = dict(Cb, via=Cb['via'] * _via_n)
        bx = p['bbox']; res = None
        if not src or not tgt: ok = False; continue
        sxy = [cell_xy((q % NN) % NX, (q % NN) // NX) for q in src]
        if len(src) > 4000:
            o = sorted(range(len(src)), key=lambda q: math.hypot(sxy[q][0] - p['x'], sxy[q][1] - p['y']))[:4000]
            src_s = [src[q] for q in o]
        else: src_s = src


        _hart = ERREICH and (net, _wk) in _ERR_NETZE and _unerreichbar(_komponenten(net, _wk), src_s, tgt)
        _raender = () if _hart else ((FENSTER, 40.0) if FENSTER else (3.0, 8.0, 40.0))
        _zl = [] if _zone is None else ([_zone, _kzr] if _zone is _kzz and len(_kzr[0]) and (net, bynet[net][j], 1) not in _KZ_AUS else [_zone])
        for _zone in _zl + [None]:
            _mz = _zone is not None
            _cH, _vH, _cC = capH, viaH, capC
            if _mz:
                if _zone is not _kzz: _cH, _vH = _kz_karten(capH, viaH, *_zone)
                else:
                    if _kzk is None or _kzk[0] is not capH: _kzk = (capH,) + _kz_karten(capH, viaH, *_zone)
                    _cH, _vH = _kzk[1], _kzk[2]
                _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
            for _mi, margin in enumerate(_raender):
                win = (min(min(x for x, _ in sxy), bx[0]) - margin, min(min(y for _, y in sxy), bx[1]) - margin,
                       max(max(x for x, _ in sxy), bx[2]) + margin, max(max(y for _, y in sxy), bx[3]) + margin)
                if not _im_fenster(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = karten_groesser(net, box, win, _wmax)
                    if _b2 is not None: box = _b2
                    _cH, _vH, _cC = capH, viaH, capC
                    if _mz: _cH, _vH = _kz_karten(capH, viaH, *_zone); _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
                if _bga_own_holes:



                    viaH = bga_eigene_lochkarte(viaH, _bga_reserved_vias + all_v)
                    _vH = bga_eigene_lochkarte(_vH, _bga_reserved_vias + all_v)
                res = astar_pf(src_s, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex)


                if res is None and ERREICH and _mi == len(_raender) - 2 and _unerreichbar(_komponenten(net, _wk), src_s, tgt):
                    _hart = True; _ERR_NETZE.add((net, _wk)); break
                if res and FENSTER: break
                if res:
                    konf = any(capCf[q] < Cb['wmin'] for q in res) or any(
                        a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(res[:-1], res[1:]))
                    if not konf or margin >= 40.0: break
                    if NB is not None and NB.info[0] == 0: break
            if _mz and res and gl is not None and Cb is not C and any(a // NN != b2 // NN for a, b2 in zip(res[:-1], res[1:])): res = None
            if res and _mz and KETTE_REST and gl is None and not is_tap and KT is not None and (_zone is _kzz or _zone is _kzr):




                _lb = _luft_min(src_s, tgt)
                res_f = astar_pf(src_s, tgt, bx, capH, capS, viaH, viaBad, Cb, win, extra=_ex) if _plen(res) > _lb + max(KETTE_REST_ANT * _lb, KETTE_REST_MM) else None
                if res_f and _plen(res) > _plen(res_f) + max(KETTE_REST_ANT * _plen(res_f), KETTE_REST_MM):
                    res = res_f; _mz = False; _cC = capC; KETTE_STAT['rest_frei'] += 1
                    _KZ_AUS.add((net, bynet[net][j])); _KZ_AUS.add((net, bynet[net][j], 1))
            if res or _hart or not _mz: break
            if _abg_z: TOPO_STAT['abgriff_ohne_zone'] += 1; continue
            _KZ_AUS.add((net, bynet[net][j], 1) if _zone is _kzr else (net, bynet[net][j])); KETTE_STAT['ohne_zone'] += 1
        trs = vs = None
        if res:
            if _mz:
                for q_ in res: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
            trs, vs = path_to_geometry(res, _cC, dict(Cb, w=Cb['wmin'], bodenkarte=hals_feld_karte(net, Cb, Cb['wmin']), extra=_ex, anker=j not in kel))
            if _pz_active(net) and net in PZ_BREITEN and not taps and gl is None and not BF and not _pz_geometry_ok(net,trs,vs,not HART_FREMD[0]):
                repair=_pz_edge_retry(res,src_s,tgt,bx,_cH,capS,_vH,viaBad,Cb,win,_ex)
                if repair:res,trs,vs=repair
                else:res=None
            if SP and not is_tap and vs and not strom_felder_frei(res,vs,Cb,_cH): res = None
            if gl is not None and sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > \
               KETTE_UMWEG * min(_pad_abst(pads[i], p) for i in _qp) + KETTE_UMWEG_MM:
                res = None; KETTE_STAT['umweg'] += 1
        if not res and gl is not None:
            K_ = gl[0]; KETTE_STAT['zurueck'] += 1
            if gl[2] is None: jobs.append((j, None)); continue
            K_['stop'] = True
            _frei = [i for i in K_['pins'] + K_['caps'][:-1] if i not in K_['pin_ok'] and i not in K_['caps'][:max(K_['ende'], 0)]]
            jobs.extend((i, None) for i in _frei); continue
        if not res:
            if _hart:
                _pk = '%s:%s' % (p['ref'], p['num'])
                if _pk not in UNERREICHBAR.setdefault(net, []): UNERREICHBAR[net].append(_pk)
            ok = False; continue
        _sp0 = (_abg_sp if j in kel else all_tr) if ABZW and gl is None and (j not in kel or ABG_BAUM) else None
        if _sp0 and not SP:
            TOPO_STAT['abzweig_geprueft'] += 1; _kset = set(_abg_kn) if j in kel else _pn_set
            _p0 = cell_xy((res[0] % NN) % NX, (res[0] % NN) // NX); _l0 = res[0] // NN
            _m0 = _abzw_maengel(trs, _sp0, _l0, _p0, res[0] not in _kset)
            if any(_m0):
                TOPO_STAT['abzweig_mangel'] += 1
                _ex2 = _streifen(_ex, _sp0, ABZW_D, ABZW_FAK); _s2 = src_s
                if _m0[0] or _m0[2]:
                    _r = int(STERN_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX)
                    _weg = {_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r}
                    _s2 = [q for q in src_s if q not in _weg or q not in _kset]
                res2 = astar_pf(_s2, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex2) if _s2 else None
                if res2:



                    _alt_stamm = [(t_, t_['pts'], t_['w']) for t_ in _sp0] if R27_STAMM else []
                    if _mz:
                        for q_ in res2: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
                    trs2, vs2 = path_to_geometry(res2, _cC, dict(Cb, w=Cb['wmin'], bodenkarte=hals_feld_karte(net, Cb, Cb['wmin']), extra=_ex2, anker=j not in kel))
                    _p2 = cell_xy((res2[0] % NN) % NX, (res2[0] % NN) // NX)
                    _m2 = _abzw_maengel(trs2, _sp0, res2[0] // NN, _p2, res2[0] not in _kset)
                    _lg = lambda T_: sum(math.dist(a_, b_) for t in T_ for a_, b_ in zip(t['pts'][:-1], t['pts'][1:]))
                    _kf = lambda R_: any(capCf[q] < Cb['wmin'] for q in R_) or any(a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(R_[:-1], R_[1:]))


                    _topo_besser = (all(a_ <= b_ for a_, b_ in zip(_m2, _m0)) and _m2 != _m0) if R27_STAMM else _m2 < _m0
                    if _topo_besser and len(vs2) <= len(vs) and (not _kf(res2) or _kf(res)) and _lg(trs2) <= ABZW_MEHR * _lg(trs) + 1.0:
                        res, trs, vs = res2, trs2, vs2; _p0 = _p2; _l0 = res[0] // NN; TOPO_STAT['abzweig_besser'] += 1
                    else:
                        for t_, p_, w_ in _alt_stamm: t_['pts'], t_['w'] = p_, w_
            if res[0] in _kset and len(_arme_bei(_sp0, _l0, _p0)) >= 2:
                _r = int(STERN_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX); TOPO_STAT['stern_gesperrt'] += 1
                _stern.update(_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r)


        if res[0] in _fst and res[0] not in _stiche:
            _stiche.add(res[0]); trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        if res[-1] in _fzt:
            _stiche.add(res[-1]); trs.append(dict(layer=_fzt[res[-1]][0], pts=_fzt[res[-1]][1], w=[_wk] * (len(_fzt[res[-1]][1])-1)))
        if BF_NODES:
            if BGA_PLAN:


                for _i in (_qp if gl is not None else tree) + [j]:
                    if _i in BF and BF[_i][0] in (res[0], res[-1]) and _i not in _bf_done:
                        _bf_done.add(_i); trs += BF[_i][1]; vs += BF[_i][2]
            else:
                for _c in (res[0], res[-1]):
                    if _c in BF_NODES and _c not in _stiche:
                        _stiche.add(_c); trs += BF_NODES[_c][0]; vs += BF_NODES[_c][1]
        if is_tap:
            for t in trs: t['sig'] = True
            if j in kel and (ABG_AN or ABG_BAUM):
                for t in trs: t['abg'] = True
                for v in vs: v['abg'] = True
                _kn = geo_nodes(trs, vs); _abg_sp += trs; _abg_vi += vs; _abg_pads.append(j); TOPO_STAT['abgriffe'] += 1
                _abg_kn.extend(q for q in _kn if q not in _stern)
        if gl is not None:
            if SP and vs:
                _sg,_sv = strom_feld_setzen(res,vs,Cb); trs += _sg; vs += _sv
            K_ = gl[0]; KETTE_STAT['glied'] += 1
            _g = 1 + 16 * K_['nr'] + gl[1]
            _kp = [bynet[net][i] for i in (K_['pins'] + K_['caps'][:1] if gl[1] == 0 else _qp + [j])]
            for t in trs: t['kette'] = _g; t['kp'] = _kp
            for v in vs: v['kette'] = _g
            K_['spuren'] += trs; K_['vias'] += vs; all_tr += trs; all_v += vs
            if gl[1] == 0: K_['kn'].extend(geo_nodes(trs, vs))
            if gl[2] is None: K_['pin_ok'].append(j)
            elif gl[1] == 0: K_['pin_ok'].append(gl[2][0]); K_['ende'] = 0
            else: K_['ende'] = gl[1]
            continue
        _vg = ([], [])
        if SP and vs and not is_tap and j not in kel:
            _vg = strom_feld_setzen(res,vs,Cb)
        elif VG and KL is not None and vs and not is_tap and j not in kel:
            _mit = [i for K_ in KT if K_['ende'] >= 0 and K_['caps'][K_['ende']] == j for i in K_['pin_ok'] + K_['caps'][:K_['ende']]] if KT is not None else []
            _n = vg_anzahl(net, pads, tree_vg, j, _mit)
            if _n > 1: _vg = vg_setzen(net, res, trs, vs, _n, capH, viaH, capC, vbadf, Cb, all_v, rng)
        all_tr += trs + _vg[0]; all_v += vs + _vg[1]
        if not is_tap: _kn = geo_nodes(trs, vs); pathnodes.extend(_kn); _pn_set.update(_kn)
        tree.append(j)
        if KT is not None:
            tree_vg.append(j); tree_vg += [i for K_ in KT if K_['ende'] >= 0 and K_['caps'][K_['ende']] == j for i in K_['pin_ok'] + K_['caps'][:K_['ende']]]
    if KT is not None:
        for K_ in KT:
            if K_['ende'] >= 0 and K_['spuren']:
                K_['spuren'][0]['kz'] = [bynet[net][K_['caps'][K_['ende']]]] + [bynet[net][i] for i in K_['pin_ok'] + K_['caps'][:K_['ende']]]
    if RK is not None and all_v: all_v = all_v + rv_setzen(net, all_tr, all_v, RK)
    if _pz_active(net) and not _pz_geometry_ok(net,all_tr,all_v):
        all_tr,all_v,ok=[],[],False
    add_unit(u, net, [dict(net=net, **t) for t in all_tr], all_v)
    return ok


def unit_conflicts(u, capS, dd):
    ""
    r = U[u]; bad = []
    for t in r['tracks']:
        li = LI[t['layer']]; capL = capS[li]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1)
            tt = np.linspace(0, 1, n)
            ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
            m = capL[iy, ix] < w - 0.005
            if m.any(): bad.extend((li * NN + iy[m] * NX + ix[m]).tolist())
    for v in r['vias']:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        if dd[iy, ix] < HOLE_C or any(capS[li][iy, ix] / 2 < VIA_R - 0.003 for li in range(NL)):
            bad.extend([li * NN + iy * NX + ix for li in range(NL)])
    return bad


from shapely.geometry import LineString as _LS, Point as _PT
from shapely.strtree import STRtree as _STR
DRC_CLR = DRCC + 0.004
DRC_HOLE = H2H + VIA_DR + 0.004
DRC_PADC = DRCP + (PAD_EXTRA if BGA_ENG else 0.004)


def _pz_clearance(net,other,pad=False):
    if globals().get('KA') is not None:
        return _ka_margin(net,other,typ2='Pad' if pad else 'Track')
    if _pz_active(net) or _pz_active(other):
        return (DRCP if pad else DRCC) + PZ_RAND_NM / 1e6
    return DRC_PADC if pad else DRC_CLR


PAIRS = []
_DMAX = max(DRC_CLR, DRC_PADC)
if globals().get('KA') is not None: _DMAX = max(_DMAX,KA.maximum+.012)
_PADB = {li: [(pn, pg, pg.buffer(_DMAX)) for pn, pg in _PADG[li]] for li in range(NL)}
if DRCL:


    _PADB = {li: [] for li in range(NL)}
    for _p in D['pads']:
        _pn = _p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p)
        for _L in _p['layers']:
            if _L not in LI: continue
            _g = (shapely.union_all([Polygon(q) for q in _p['polys'][_L]])
                  if globals().get('KA') is not None else pad_huelle(_p, _L, DRCP, True))
            if globals().get('KA') is not None: _KA_META[id(_g)] = _ka_pad_kind(_p)
            _PADB[LI[_L]].append((_pn, _g, _g.buffer(_DMAX)))
if globals().get('KA') is not None:
    for _li,_keeps in _KA_PAD_KEEP.items():
        _PADB[_li].extend((_pn,_g,_g.buffer(_DMAX)) for _pn,_g,_kind in _keeps)


def _unit_geo(u, r):
    ""
    g = _GEO.get(u)
    if g is None:
        g = []; n0 = r['net']
        for t in r['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                radius=w/2/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOKAL',{}) else w/2
                k = _LS([a, b]).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOKAL',{}) else 8) if a != b else _PT(a).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOKAL',{}) else 8)
                g.append((li, k, k.buffer(_DMAX), n0))
                if globals().get('KA') is not None: _KA_META[id(k)] = ('Track','')
        for v in r['vias']:
            k = _PT(v['x'], v['y']).buffer(VIA_R/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOKAL',{}) else VIA_R, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOKAL',{}) else 8); kb = k.buffer(_DMAX); nv = v.get('net') or n0
            for li in range(NL):
                if globals().get('KA') is not None and LAYERS[li] not in v.get('layers',LAYERS):continue
                g.append((li, k, kb, nv))
            if globals().get('KA') is not None: _KA_META[id(k)] = ('Via','')
        _GEO[u] = g
    return g


FIX_PAIRS = []


def _fixpaar(a, b):


    return a.startswith('FIX#') and b.startswith('FIX#') and (globals().get('args') or {}).get('rest_pruefung') != '1'


def _konfliktpunkte(a, b, abstand):
    ""
    if abstand == 0:
        c = shapely.point_on_surface(shapely.intersection(a, b))
        return c, c
    line = shapely.shortest_line(a, b)
    return shapely.get_point(line, 0), shapely.get_point(line, -1)


def _pad_bohrkonflikte(vias, note, melden=True):
    ""
    if not PAD_LOCH_GEOMS or not vias: return
    holes=np.array(PAD_LOCH_GEOMS,dtype=object)
    points=np.array([_PT(x,y) for _,_,x,y in vias],dtype=object)
    a,b=_STR(holes).query(shapely.envelope(shapely.buffer(points,VIA_DR/2+H2H+.0011)))
    distance=shapely.distance(points[a],holes[b])-VIA_DR/2
    for k,j in enumerate(b.tolist()):
        circle=PAD_LOCH_KREISE[j]
        if circle is not None:
            _,_,x,y=vias[int(a[k])]
            distance[k]=math.hypot(x-circle[0],y-circle[1])-circle[2]-VIA_DR/2
    for i,j,d in zip(a.tolist(),b.tolist(),distance.tolist()):
        if d >= H2H-1e-9: continue
        u,_,x,y=vias[i]
        for li in range(NL): note(u,x,y,li)
        if melden:
            PAIRS.append(('*',round(d,4),u,'PAD',round(x,2),round(y,2)))
            if VIA_VORRANG: PAIRS_ROH.append((-1,x,y,u,'PAD'))


def _im_fenster(bounds, F):
    return bounds[0] <= F[2] and bounds[2] >= F[0] and bounds[1] <= F[3] and bounds[3] >= F[1]
def exact_conflicts(ohne=None, fenster=None):
    ""
    PAIRS.clear(); PAIRS_ROH.clear(); FIX_PAIRS.clear()
    geoms = {li: [] for li in range(NL)}
    drills_l = []
    for u, r in U.items():
        if ohne is not None and u in ohne: continue
        for li, k, kb, ne in _unit_geo(u, r):
            if fenster is None or _im_fenster(k.bounds, fenster): geoms[li].append((u, ne, k, kb))
        for v in r['vias']:


            if fenster is not None and not _im_fenster((v['x'], v['y'], v['x'], v['y']), fenster): continue
            nv = (v.get('net') or r['net']) if BGA_LOCH or bool(globals().get('PZ_LOKAL')) else r['net']
            drills_l.append((u, nv, v['x'], v['y']))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]:
            if fenster is None or _im_fenster(pg.bounds, fenster): geoms[li].append(('PAD', pn, pg, pb))
    conf = {}
    def note(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rr = 3
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - rr), min(NY, iy + rr + 1)):
            for jx in range(max(0, ix - rr), min(NX, ix + rr + 1)):
                lst.append(li * NN + jy * NX + jx)
    for li in range(NL):
        G = geoms[li]
        if not G: continue
        ga = np.array([g for _, _, g, _ in G], dtype=object)
        tree = _STR(ga)
        netz = np.array([n for _, n, _, _ in G], dtype=object)
        if (BGA_LOCH or bool(globals().get('PZ_LOKAL')) or globals().get('KA') is not None) and LOCH_CLR > 0 and drills_l:


            hp = np.array([_PT(x, y) for _, _, x, y in drills_l], dtype=object)
            hi, hj = tree.query(shapely.buffer(hp, VIA_DR / 2 + LOCH_CLR + 0.00001))
            hn = np.array([n for _, n, _, _ in drills_l], dtype=object)
            hd = shapely.distance(hp[hi], ga[hj]) - VIA_DR / 2
            hm = (hn[hi] != netz[hj]) & (hd < LOCH_CLR - 0.000001)
            for i, j, dij in zip(hi[hm].tolist(), hj[hm].tolist(), hd[hm].tolist()):
                u, _, x, y = drills_l[i]; u2 = G[j][0]
                if _fixpaar(u, u2): continue
                c2 = shapely.get_point(shapely.shortest_line(hp[i], ga[j]), -1)



                note(u, x, y, li); note(u2, c2.x, c2.y, li)
                cx, cy = (x+c2.x)/2, (y+c2.y)/2
                PAIRS.append((LAYERS[li], round(dij, 4), u, u2, round(cx, 2), round(cy, 2)))
                if VIA_VORRANG: PAIRS_ROH.append((li, cx, cy, u, u2))
        qi, qj = tree.query(np.array([gb for _, _, _, gb in G], dtype=object))
        m = qj > qi
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        netz = np.array([n for _, n, _, _ in G], dtype=object); pad = np.array([u == 'PAD' for u, _, _, _ in G])
        m = (netz[qi] != netz[qj]) & ~(pad[qi] & pad[qj])
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        dist = shapely.distance(ga[qi], ga[qj])
        limits = np.where(pad[qi] | pad[qj], DRC_PADC, DRC_CLR)
        if globals().get('PZ_LOKAL'):
            limits = np.array([_pz_clearance(netz[i],netz[j],bool(pad[i] or pad[j])) for i,j in zip(qi,qj)])
        if globals().get('KA') is not None:
            limits = np.array([_ka_geo_limit(netz[i],netz[j],li,ga[i],ga[j],bool(pad[j])) for i,j in zip(qi,qj)])
        m = dist < limits
        for i, j, dij in zip(qi[m].tolist(), qj[m].tolist(), dist[m].tolist()):
            u, net, g, _ = G[i]; u2, net2, g2, _ = G[j]
            if _fixpaar(u, u2):
                FIX_PAIRS.append((LAYERS[li], round(dij, 4), u, u2)); continue
            if BGA_PLAN or KONFLIKTORT_EXAKT or globals().get('_REST_LOKAL_KONFLIKT', False):


                c, c2 = _konfliktpunkte(g, g2, dij)
                note(u, c.x, c.y, li); note(u2, c2.x, c2.y, li)
                c = _PT((c.x+c2.x)/2,(c.y+c2.y)/2)
            else:
                c = g.centroid if dij == 0 else g.boundary.interpolate(g.boundary.project(g2.centroid))
                note(u, c.x, c.y, li); note(u2, c.x, c.y, li)
            PAIRS.append((LAYERS[li], round(dij, 3), u, u2, round(c.x, 2), round(c.y, 2)))
            if VIA_VORRANG: PAIRS_ROH.append((li, c.x, c.y, u, u2))
    _pad_bohrkonflikte(drills_l,note)
    if len(drills_l) > 1:
        from scipy.spatial import cKDTree
        P = np.array([(x, y) for _, _, x, y in drills_l])
        _vhclear = max(DRC_HOLE, VIA_DR / 2 + VIA_R + LOCH_CLR) if BGA_LOCH or bool(globals().get('PZ_LOKAL')) else DRC_HOLE
        for i, j in sorted(cKDTree(P).query_pairs(_vhclear * (1 + 1e-9) + 1e-9)):
            u, n1, x1, y1 = drills_l[i]; u2, n2, x2, y2 = drills_l[j]
            if math.hypot(x1 - x2, y1 - y2) < DRC_HOLE or ((BGA_LOCH or bool(globals().get('PZ_LOKAL'))) and n1 != n2 and math.hypot(x1 - x2, y1 - y2) < VIA_DR / 2 + VIA_R + LOCH_CLR - 0.000001):
                if _fixpaar(u, u2):
                    FIX_PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 4), u, u2)); continue
                for li in range(NL): note(u, x1, y1, li); note(u2, x2, y2, li)
                if VIA_VORRANG: PAIRS_ROH.append((-1, x1, y1, u, 'PAD')); PAIRS_ROH.append((-1, x2, y2, u2, 'PAD'))


                PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 3), u, u2, round((x1 + x2) / 2, 2), round((y1 + y2) / 2, 2)))
    return conf


def _schmal_roh():
    ""
    out = []; Cs = cls('/__signal__')
    for u, r in U.items():
        if u.startswith(('FIX#', 'HEFT#')): continue
        stich = u.startswith('GND#') or u in PLANE_GROUPS
        C = dict(wmin=max(0.15, BAHN_MIN)) if stich else cls(r['net'])
        WF = hals_feld_karte(r['net'], C, C['wmin'], stich)
        for t in r['tracks']:
            if t.get('steg'): continue
            soll = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= soll - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                erl = WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < soll
                out.append((u, r['net'], li, a, b, w, soll, bool(erl)))
    return out
def unter_wmin():
    ""
    out = []; vor = None
    for u, net, li, a, b, w, soll, erl in _schmal_roh():
        L = math.dist(a, b)
        if vor is not None and vor[0] == (u, li, erl, soll) and vor[1] == tuple(a):
            e = out[-1]; e['_p'].append((a, b)); e['laenge'] += L; e['breite'] = min(e['breite'], w)
        else:
            out.append(dict(netz=net, lage=LAYERS[li], x=0.0, y=0.0, breite=w, soll=soll, laenge=L, erlaubt=erl, _p=[(a, b)]))
        vor = ((u, li, erl, soll), tuple(b))
    for e in out:
        halb = e['laenge'] / 2; p = e.pop('_p')
        for a, b in p:
            L = math.dist(a, b)
            if halb <= L or (a, b) == p[-1]:
                f = min(1.0, halb / L) if L > 0 else 0.0
                e['x'] = round(a[0] + f * (b[0] - a[0]), 3); e['y'] = round(a[1] + f * (b[1] - a[1]), 3); break
            halb -= L
        e['laenge'] = round(e['laenge'], 3)
    return out


WMIN_VOR = args.get('wmin_vorrang', '1') == '1'


def _vorrang(net):
    ""
    if not KL: return 1
    return 2 if KL['netze'].get(net, {}).get('art') in ('strom', 'versorgung', 'sw', 'gnd') else 1


def eng_konflikte():
    ""
    roh = [s for s in _schmal_roh() if not s[7]]
    if WMIN_VOR and KL:
        roh = [s for s in roh if not s[0].startswith('GND#') and _vorrang(s[1]) >= 2]
    conf = {}
    if not roh: return conf
    def note(u, x, y, li):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    for li in sorted({s[2] for s in roh}):
        G = [(u, ne, k) for u, r in U.items() for l2, k, _kb, ne in _unit_geo(u, r) if l2 == li]
        ga = np.array([g for _, _, g in G], dtype=object); tree = _STR(ga)
        for u, net, _li, a, b, w, soll, _e in roh:
            if _li != li: continue
            k = _LS([a, b]).buffer(soll / 2, 8) if a != b else _PT(a).buffer(soll / 2, 8)
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            gegner = sorted({G[j][0] for j in tree.query(k.buffer(DRC_CLR)).tolist() if G[j][1] != net and k.distance(G[j][2]) < DRC_CLR})



            weich = [u2 for u2 in gegner if _vorrang(U[u2]['net']) < _vorrang(net)] if WMIN_VOR else []
            if not weich: note(u, mx, my, li)
            for u2 in (weich or gegner): note(u2, mx, my, li)
            for u2 in gegner or ['PAD']: PAIRS.append((LAYERS[li], round(w - soll, 3), u, u2, round(mx, 2), round(my, 2)))
    return conf


_KIDX = [None]


def _kidx_bauen(ohne):
    ""
    lagen = {}
    je = {li: [] for li in range(NL)}
    for u, r in U.items():
        if u == ohne: continue
        for li, k, kb, ne in _unit_geo(u, r): je[li].append((u, ne, k, False))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: je[li].append(('PAD', pn, pg, True))
        if je[li]:
            ga = np.array([g for _, _, g, _ in je[li]], dtype=object)
            lagen[li] = (_STR(ga), [o for o, _, _, _ in je[li]], [n for _, n, _, _ in je[li]], ga, [q for _, _, _, q in je[li]])
    return dict(lagen=lagen, alt=set(), neu={}, fehlt={ohne})


def kidx_geaendert(u):
    ""
    X = _KIDX[0]
    if X is not None: X['alt'].add(u); X['neu'][u] = _unit_geo(u, U[u]); X['fehlt'].discard(u)


def kidx_zurueck(u):
    ""
    X = _KIDX[0]
    if X is not None and u in X['fehlt']: kidx_geaendert(u)


def einheit_konfliktfrei(u):
    ""
    if BGA_LOCH or globals().get('PZ_LOKAL') or globals().get('KA') is not None: return u not in exact_conflicts()
    X = _KIDX[0]
    if X is None or len(X['alt']) > 48: X = _KIDX[0] = _kidx_bauen(u)
    r = U[u]; net = r['net']; geo = _unit_geo(u, r)
    for li in range(NL):
        mine = [(k, kb, ne) for l2, k, kb, ne in geo if l2 == li]
        if not mine: continue
        T = X['lagen'].get(li)
        if T is not None:
            tree, own, netz, ga, pad = T
            qi, qj = tree.query(np.array([kb for _, kb, _ in mine], dtype=object))
            for i, j in zip(qi.tolist(), qj.tolist()):
                if own[j] == u or own[j] in X['alt'] or netz[j] == mine[i][2] or _fixpaar(u, own[j]): continue
                if mine[i][0].distance(ga[j]) < (_ka_geo_limit(net,netz[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)): return False
        for u2, g2 in X['neu'].items():
            if u2 == u or u2 not in U or _fixpaar(u, u2): continue
            for l2, k2, _, n2 in g2:
                if l2 != li: continue
                for k, _, ne in mine:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
        if any(ne != net for _, _, ne in mine):
            for i, (k, _, ne) in enumerate(mine):
                for k2, _, n2 in mine[i + 1:]:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
    if r['vias']:
        pad_konflikt = []
        _pad_bohrkonflikte([(u, net, v['x'], v['y']) for v in r['vias']],
                          lambda *q: pad_konflikt.append(q), melden=False)
        if pad_konflikt: return False
        alle = [(v['x'], v['y'], u2) for u2, r2 in U.items() for v in r2['vias']]
        for i, v in enumerate(r['vias']):
            for x, y, u2 in alle:
                if (u2 == u and x == v['x'] and y == v['y']) or _fixpaar(u, u2): continue
                if math.hypot(v['x'] - x, v['y'] - y) < DRC_HOLE: return False
    return True


def gruppen_index(gruppe):
    ""
    g = set(gruppe); je = {li: [] for li in range(NL)}; bohr = []
    for u, r in U.items():
        if u in g: continue
        for li, k, kb, ne in _unit_geo(u, r): je[li].append((u, ne, k))
        for v in r['vias']: bohr.append((v['x'], v['y'], u))
    lagen = {}
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: je[li].append(('PAD', pn, pg))
        if je[li]:
            ga = np.array([q for _, _, q in je[li]], dtype=object)
            lagen[li] = (_STR(ga), [o for o, _, _ in je[li]], [n for _, n, _ in je[li]], ga)
    return dict(lagen=lagen, bohr=bohr, bxy=np.array([(x, y) for x, y, _ in bohr], dtype=np.float64).reshape(-1, 2))


def konflikte_gruppe(gruppe, X, ohne=None):
    ""
    if BGA_LOCH or globals().get('PZ_LOKAL') or globals().get('KA') is not None:


        conf = exact_conflicts(ohne)
        g = set(gruppe)
        mit = {u for pair in PAIRS if pair[2] in g or pair[3] in g for u in pair[2:4]}
        return {u: cs for u, cs in conf.items() if u in mit}
    conf = {}
    def note(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)):
                lst.append(li * NN + jy * NX + jx)
    mit = [u for u in gruppe if u in U]
    for li in range(NL):
        mine = [(u, ne, k, kb) for u in mit for l2, k, kb, ne in _unit_geo(u, U[u]) if l2 == li]
        if not mine: continue
        suche = np.array([kb for _, _, _, kb in mine], dtype=object)
        T = X['lagen'].get(li)
        if T is not None:
            tree, own, netz, ga = T
            qi, qj = tree.query(suche)
            for i, j in zip(qi.tolist(), qj.tolist()):
                u, net, k, _ = mine[i]
                if netz[j] == net or _fixpaar(u, own[j]) or (ohne is not None and own[j] in ohne): continue
                d = k.distance(ga[j])
                if d < (DRC_PADC if own[j] == 'PAD' else DRC_CLR):
                    if KONFLIKTORT_EXAKT:
                        c, c2 = _konfliktpunkte(k, ga[j], d)
                        note(u, c.x, c.y, li); note(own[j], c2.x, c2.y, li)
                    else:
                        c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(ga[j].centroid))
                        note(u, c.x, c.y, li); note(own[j], c.x, c.y, li)
        gm = np.array([k for _, _, k, _ in mine], dtype=object)
        qi, qj = _STR(gm).query(suche)
        for i, j in zip(qi.tolist(), qj.tolist()):
            if j <= i or mine[i][1] == mine[j][1] or _fixpaar(mine[i][0], mine[j][0]): continue
            d = mine[i][2].distance(mine[j][2])
            if d < DRC_CLR:
                k = mine[i][2]
                if KONFLIKTORT_EXAKT:
                    c, c2 = _konfliktpunkte(k, mine[j][2], d)
                    note(mine[i][0], c.x, c.y, li); note(mine[j][0], c2.x, c2.y, li)
                else:
                    c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(mine[j][2].centroid))
                    note(mine[i][0], c.x, c.y, li); note(mine[j][0], c.x, c.y, li)
    eig = [(v['x'], v['y'], u) for u in mit for v in U[u]['vias']]
    _pad_bohrkonflikte([(u,None,x,y) for x,y,u in eig],note,melden=False)
    for i, (x, y, u) in enumerate(eig):
        if len(X['bxy']):
            for j in np.flatnonzero(np.hypot(X['bxy'][:, 0] - x, X['bxy'][:, 1] - y) < DRC_HOLE).tolist():
                if _fixpaar(u, X['bohr'][j][2]) or (ohne is not None and X['bohr'][j][2] in ohne): continue
                for li in range(NL): note(u, x, y, li); note(X['bohr'][j][2], X['bohr'][j][0], X['bohr'][j][1], li)
        for x2, y2, u2 in eig[i + 1:]:
            if math.hypot(x - x2, y - y2) < DRC_HOLE:
                if _fixpaar(u, u2): continue
                for li in range(NL): note(u, x, y, li); note(u2, x2, y2, li)
    return conf


FLUSS_W = {}






BREIT_STOER = args.get('breit_stoer', '1' if KL else '0') == '1'
R32 = args.get('r32', '0') == '1' and KL is not None
_ABST_C = {}


def _abst_c(name, nets, koerper=False):
    ""
    key = (name, frozenset(nets), koerper) if STOER_FLANKE else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if STOER_FLANKE else sum(_NVER.get(n, 0) for n in nets)
    c = _ABST_C.get(key)
    if c is None or c[0] != sig:
        m = _maske(nets)
        if koerper and _SPULEN_M is not None: m |= _SPULEN_M
        d = np.full(NL * NN, 99.0, dtype=np.float32)
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): d[sl] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        c = (sig, d.reshape(NL, NY, NX)); _ABST_C[key] = c
    return c[1]


def _stoer_breite(net):
    ""
    if not (BREIT_STOER and KL): return []
    paare = []
    if net in AGG_SW and SW_ABST > 0: paare.append(('b_vsw', VICT_SW, SW_ABST))
    if net in AGG_FAST and TRENN > 0: paare.append(('b_va', VICT_A, _trenn_mm(net)))
    if net in VICT_SW and SW_ABST > 0: paare.append(('b_asw', AGG_SW, SW_ABST))
    if net in VICT_A and TRENN > 0:
        if STOER_FLANKE:
            for f, ns in _flanken_gruppen(): paare.append((('b_af', f), ns, TRENN * f))
        else: paare.append(('b_af', AGG_FAST, TRENN))
    if SI:
        opfer = {n for n, k in KL['netze'].items() if k.get('art') in ('analog', 'kelvin') or k.get('kelvin')}
        for other in sorted((AGG_SW | AGG_FAST) if net in opfer else opfer if net in AGG_SW | AGG_FAST else []):
            d = si_abstand(net, other) if net in opfer else si_abstand(other, net)
            if d > 0: paare.append((('b_si', other), {other}, d))
    return [(_abst_c(n_, frozenset(m_ - {net}), koerper=STOER_FLANKE and n_ == 'b_asw'), sp) for n_, m_, sp in paare if m_ - {net} or (STOER_FLANKE and n_ == 'b_asw')]





BREITE_INNEN = float(args.get('breite_innen', '2.3')) if KL is not None else 0.0


def f_innen(net, li):
    ""
    if BREITE_INNEN <= 0 or li in (0, NL - 1): return 1.0
    k = KL['netze'].get(net) or {}
    if k.get('art') not in ('strom', 'versorgung', 'sw'): return 1.0
    if args.get('strom_soll') == '1':
        from gemeinsam.strombahn import sollbreite
        return sollbreite(k, innen=True) / sollbreite(k)
    if k.get('w_innen') and k.get('w'): return max(1.0, float(k['w_innen']) / float(k['w']))
    return BREITE_INNEN


def _r32_freiraum(X, u, li, a, b, net, limit):
    ""
    T = X['lagen'].get(li)
    if T is None or limit <= 0: return limit
    seg = _LS([a, b]) if a != b else _PT(a)
    tree, own, netz, ga, pad = T
    for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-9)):
        j = int(j)
        if netz[j] == net or _fixpaar(u, own[j]): continue
        limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,netz[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR))))
    return limit


def _widen_unit_alt(u, C):
    if paar_einheit(u) or U[u]['net'] in IMP_TAB: return
    if strom_modell(U[u]['net']):
        strom_korrigieren(u,C);return
    ""
    r = U[u]; net = r['net']
    _sb = _stoer_breite(net)
    del_unit(u)
    einheitlich = R32 and (KL['netze'].get(net) or {}).get('art') in ('signal', 'schnell', 'analog')
    if all(t.get('sig') for t in r['tracks']) and not einheitlich:
        add_unit(u, net, list(r['tracks']), r['vias']); return
    rx = _kidx_bauen(u) if einheitlich else None
    xs = [q[0] for t in r['tracks'] for q in t['pts']] or [0.0]; ys = [q[1] for t in r['tracks'] for q in t['pts']] or [0.0]
    ziel = FLUSS_W.get(u, {})
    _fmax = max([f_innen(net, LI[t['layer']]) for t in r['tracks']] + [1.0])
    capH, _, capS, dd, _, _, capC, _, _ = karten(net, (min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0), max([C['w']] + list(ziel.values())) * _fmax)
    new_tr = list(r['tracks']); reihe = range(len(new_tr)); kz = None
    if KETTE and any('kz' in t for t in new_tr):
        kz = _kz_vorbereiten(r, net); breiten = [list(t['w']) for t in new_tr]
        reihe = sorted(reihe, key=lambda ti: (not new_tr[ti].get('kette'), new_tr[ti].get('kette') or 0, ti))
    ab = _abg_vorbereiten(r, net) if ABG_AN and any(t.get('abg') for t in new_tr) else None
    for ti in reihe:
        t = new_tr[ti]
        if t.get('sig') and not einheitlich: continue
        li = LI[t['layer']]; pts = t['pts']; ws = []
        for si, (a, b, w0) in enumerate(zip(pts[:-1], pts[1:], t['w'])):
            wv = min(ziel.get((ti, si), C['w']) * f_innen(net, li), line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net))
            if kz is not None: wv = min(wv, _kz_breite(kz, r, breiten, ti, a, b, li))
            if ab is not None: wv = min(wv, _abg_breite(ab, a, b, li))
            for _dk, _sp in _sb:
                wv = min(wv, 2 * (line_cap_min(_dk[li], a, b) - _sp))
                for _lj, _s in nachbar_abstand(li, _sp):
                    wv = min(wv, 2 * (line_cap_min(_dk[_lj], a, b) - _s))
            if 'w_step' in C: wv = min(wv, w0 + C['w_step'])
            if einheitlich:
                ws.append(_r32_freiraum(rx, u, li, a, b, net, min(C['w'], wv)))
            else: ws.append(max(w0, math.floor(wv * 40) / 40))
        new_tr[ti] = dict(t, w=ws)
        if kz is not None: breiten[ti] = ws
    if einheitlich:

        widths = [w for t in new_tr for w in t['w']]
        if widths:


            minimum = BAHN_MIN if BAHN_MIN > 0 else C['wmin']
            target = max(minimum, C['w']) if min(widths) >= max(minimum, C['w']) - 1e-9 else minimum
            new_tr = [dict(t, w=[target] * len(t['w'])) for t in new_tr]
    add_unit(u, net, new_tr, r['vias'])


BREIT_DICHT = args.get('breit_dicht', '0') == '1'


def _breit_dicht_cap(X, u, li, a, b, net, limit):
    ""
    seg = _LS([a, b]) if a != b else _PT(a)
    T = X['lagen'].get(li)
    if T is not None:
        tree, own, netz, ga, pad = T

        for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-7).envelope):
            j = int(j)
            if own[j] == u or own[j] in X['alt'] or netz[j] == net or _fixpaar(u, own[j]): continue
            limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,netz[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)) - 1e-7))
    for other, geo in X['neu'].items():
        if other == u or other not in U or _fixpaar(u, other): continue
        for l2, g, _, n2 in geo:
            if l2 == li and n2 != net:
                limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))

    for l2, g, _, n2 in _unit_geo(u, U[u]):
        if l2 == li and n2 != net:
            limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))
    if (BGA_LOCH or bool(globals().get('PZ_LOKAL')) or globals().get('KA') is not None) and LOCH_CLR > 0:
        for other, r in U.items():
            if _fixpaar(u, other): continue
            for v in r['vias']:
                if (v.get('net') or r['net']) != net:
                    limit = min(limit, 2 * (seg.distance(_PT(v['x'], v['y'])) - VIA_DR / 2 - LOCH_CLR - 1e-7))
    return limit
def widen_unit(u, C):
    if not BREIT_DICHT:
        return _widen_unit_alt(u, C)
    old = copy.deepcopy(U[u])
    X = _KIDX[0]
    if X is None or len(X['alt']) > 48:
        X = _KIDX[0] = _kidx_bauen(u)
    _widen_unit_alt(u, C)
    r = U[u]


    shape = lambda q: ([(t['layer'], t['pts']) for t in q['tracks']],
                       [(v['x'], v['y'], v.get('net')) for v in q['vias']])
    if shape(r) != shape(old):
        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias'])
        kidx_geaendert(u)
        return
    tr = []
    for t, t0 in zip(r['tracks'], old['tracks']):
        ws = []
        for a, b, w, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w'], t0['w']):
            if w > w0:
                cap = _breit_dicht_cap(X, u, LI[t['layer']], a, b, r['net'], w)

                w = max(w0, min(w, math.floor(max(0., cap) * 40) / 40))
            ws.append(w)
        tr.append(dict(t, w=ws))
    del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    kidx_geaendert(u)









RUHIG = args.get('breite_ruhig', '1' if KL else '0') == '1'
RUHIG_R = float(args.get('ruhig_r', '0.03')); RUHIG_MM = float(args.get('ruhig_mm', '0.35')); RUHIG_KURZ = float(args.get('ruhig_kurz', '1.0'))
FLUSS_A = {}
if RUHIG:
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from gemeinsam.breitenprofil import beruhigen as _bp_ruhig, zaehlen as _bp_zaehl


def breite_beruhigen(u, C):
    if paar_einheit(u) or U[u]['net'] in IMP_TAB: return (0, 0, 0)
    ""
    r = U[u]; ziel = FLUSS_W.get(u); ant = FLUSS_A.get(u); vor = nach = n = 0; new_tr = []
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('steg') or len(t['w']) < 2: new_tr.append(t); continue
        l = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(t['pts'][:-1], t['pts'][1:])]
        _fi = f_innen(r['net'], LI[t['layer']])
        z = [ziel.get((ti, si), C['w']) * _fi for si in range(len(l))] if ziel else None
        a_ = max(ant.get((ti, si), 1.0) for si in range(len(l))) if ant else 1.0
        ws = _bp_ruhig(l, t['w'], ziel=z, boden=C['wmin'], lam=RUHIG_MM, kurz=RUHIG_KURZ, r_max=RUHIG_R / max(a_, 0.2))
        vor += _bp_zaehl(l, t['w'])[0]; nach += _bp_zaehl(l, ws)[0]
        if ws != list(t['w']): n += 1; t = dict(t, w=ws)
        new_tr.append(t)
    if n: del_unit(u); add_unit(u, r['net'], new_tr, r['vias'])
    else: U[u] = U.pop(u)
    return vor, nach, n
def skew_abgleichen(board, routes, optionen, zielordner):
    ""
    import collections, hashlib, re
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    folder = Path(zielordner); folder.mkdir(parents=True, exist_ok=True)
    def sha(v): return hashlib.sha256(json.dumps(v,sort_keys=True).encode()).hexdigest()
    def signature(d):
        return collections.Counter((v['type'],v['description'],json.dumps([(i['description'],i['pos']) for i in v['items']],sort_keys=True))
            for v in d['violations'] if v['severity']=='error' and v['type']!='skew_out_of_range')
    def opens(d):
        return collections.Counter(re.search(r'\[([^]]+)\]',v['items'][0]['description'])[1] for v in d['unconnected_items'])
    def allowed(old,new):
        return not(signature(new)-signature(old)) and not(opens(new)-opens(old))
    current = copy.deepcopy(routes)
    drc = native_routen_pruefen(board,current,optionen,folder/'anfang')
    native_folder=folder/'anfang'
    report = dict(version='nativer-skew-v1', zuege=[], versucht_nativ=0, vor=native_skew_fehler(drc),
                  eingang_sha=sha(routes), nativ_vor=dict(fehler=sum(v['severity']=='error' for v in drc['violations']),offen=len(drc['unconnected_items'])))
    boundary=rand_polygon(board).buffer(-float(optionen.get('edge',.3)))
    clearance=max(float(optionen.get('drc',.09)),float(optionen.get('drcpad',.09)))+.002
    vd=float(optionen.get('via','0.45/0.20').split('/')[0])
    maxmoves=int(optionen.get('skew_zuege','24'));maxnative=int(optionen.get('skew_kandidaten','80'))
    for iteration in range(maxmoves):
        errors=native_skew_fehler(drc)
        if not errors: break
        moved=False
        for error in errors:
            net=error['net'];partner=error['partner'];delta=error['delta_mm']-.35*error['toleranz_mm']

            foreign={}; own={}; partners={}
            for t in current['tracks']:
                L=t['layer']
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    if math.dist(a,b)<1e-7:continue
                    g=LineString([a,b]).buffer(w/2)
                    if t['net']!=net:foreign.setdefault(L,[]).append(g)
                    if t['net']==partner:partners.setdefault(L,[]).append(g)
            for v in current['vias']:
                for L in v.get('layers') or ['F']+['In%d'%i for i in range(1,board.get('lagen',4)-1)]+['B']:
                    (foreign if v['net']!=net else own).setdefault(L,[]).append(Point(v['x'],v['y']).buffer(v.get('d',vd)/2))
            for p in board['pads']:
                for L,polys in p.get('polys',{}).items():
                    (foreign if p['net']!=net else own).setdefault(L,[]).extend(Polygon(q) for q in polys)
            obstacles={L:unary_union(gg) for L,gg in foreign.items()}


            spans=[]
            for ti,t in enumerate(current['tracks']):
                if t['net']!=net or t.get('items'):continue
                pts=t['pts'];ws=t['w']
                for i in range(len(ws)):
                    for j in range(i+1,len(pts)):
                        widths=ws[i:j]
                        if max(widths)-min(widths)>1e-7:break
                        a,b=pts[i],pts[j];L=math.dist(a,b)
                        if L<.25:continue
                        dx,dy=abs(a[0]-b[0]),abs(a[1]-b[1])
                        oldlen=sum(math.dist(x,y) for x,y in zip(pts[i:j],pts[i+1:j+1]))

                        if min(dx,dy,abs(dx-dy))<=2e-5:
                            spans.append((-L,ti,i,j,oldlen,(tuple(a),tuple(b)),0))
                        else:



                            diag=min(dx,dy);sx=1 if b[0]>a[0] else -1;sy=1 if b[1]>a[1] else -1
                            for mid in ((a[0]+sx*diag,a[1]+sy*diag),(b[0]-sx*diag,b[1]-sy*diag)):
                                chain=(tuple(a),mid,tuple(b))
                                for ki in range(2):
                                    length=math.dist(chain[ki],chain[ki+1])
                                    if length>=.25:spans.append((-length,ti,i,j,oldlen,chain,ki))
            spans.sort(key=lambda q:(len(q[5])>2,q))
            candidates=[]
            for _,ti,i,j,oldlen,chain,ki in spans:
                t=current['tracks'][ti];a,b=chain[ki],chain[ki+1];w=t['w'][i];L=math.dist(a,b);layer=t['layer']
                baselen=sum(math.dist(x,y) for x,y in zip(chain,chain[1:]))
                e=((b[0]-a[0])/L,(b[1]-a[1])/L);normal=(-e[1],e[0])
                rest=[g for g in own.get(layer,[]) if g.distance(Point(a))>w and g.distance(Point(b))>w]
                for tj,tt in enumerate(current['tracks']):
                    if tt['net']!=net or tt['layer']!=layer:continue
                    rest.extend(LineString([pa,pb]).buffer(ww/2) for si,(pa,pb,ww) in enumerate(zip(tt['pts'][:-1],tt['pts'][1:],tt['w'])) if tj!=ti or not max(0,i-1)<=si<min(len(tt['w']),j+1))
                selfgeo=unary_union(rest)
                for fraction in (1.,.5,.25,.125,.0625):
                    added=delta*fraction
                    if added < .01:continue
                    extra=added+oldlen-baselen
                    if extra<=.005:continue
                    c=max(w/2,.04);pitch=max(2*w+clearance,w+float(optionen.get('paar_spalt',.2)));top=pitch-2*c
                    from gemeinsam.maeander import kuppen_formen as _ma_formen, kuppe_bahn as _ma_bahn
                    for fi_,(n_,s_,r_,A_,F_) in enumerate(_ma_formen(extra,L,w,clearance) if optionen.get('skew_form','rund')=='rund' else []):
                        leadspace=L-F_

                        for lead in [x for x in dict.fromkeys([leadspace/2,.05,leadspace-.05,leadspace*.25,leadspace*.75]) if x>=.05-1e-9 and leadspace-x>=.05-1e-9]:
                            for side in (1,-1):
                                points,sp_items=_ma_bahn(chain,ki,n_,s_,r_,A_,lead,side,w)
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple or not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue
                                pocket=LineString(points[1:-1]).buffer(w/2)

                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,0,fi_,coupling,j-i,ti,i,j,side,lead,'kuppe',chain,ki)
                                candidates.append((rank,ti,i,j,points,added,w,layer,'kuppe',sp_items))
                    for shape in ('trombone','dreieck'):
                        height=(extra+4*c*(2-math.sqrt(2)))/2 if shape=='trombone' else extra/(2*(math.sqrt(2)-1))
                        span=4*c+top if shape=='trombone' else 2*height
                        if shape=='trombone' and height<2*c:continue
                        leadspace=L-span
                        if leadspace<.04:continue
                        for lead in dict.fromkeys([leadspace/2,.02,leadspace-.02,leadspace*.25,leadspace*.75]):
                            for side in (1,-1):
                                if shape=='trombone':
                                    q=[(0,0),(lead,0),(lead+c,c),(lead+c,height-c),(lead+2*c,height),(lead+2*c+top,height),(lead+3*c+top,height-c),(lead+3*c+top,c),(lead+span,0),(L,0)]
                                else:q=[(0,0),(lead,0),(lead+height,height),(lead+span,0),(L,0)]
                                points=[[round(a[0]+x*e[0]+side*y*normal[0],6),round(a[1]+x*e[1]+side*y*normal[1],6)] for x,y in q]
                                points[0]=list(a);points[-1]=list(b)
                                points=[list(p) for p in chain[:ki]]+points+[list(p) for p in chain[ki+2:]]
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple:continue
                                if not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue

                                pocket=LineString(points[1:-1]).buffer(w/2)
                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,2*w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,1,height,coupling,j-i,ti,i,j,side,lead,shape,chain,ki)
                                candidates.append((rank,ti,i,j,points,added,w,layer,shape,None))
                if len(candidates)>100:break
            candidates.sort(key=lambda q:q[0])
            seen=set()
            for _,ti,i,j,points,added,w,layer,shape,sp_items in candidates:
                key=(ti,i,j,tuple(map(tuple,points)))
                if key in seen:continue
                seen.add(key)
                if report['versucht_nativ']>=maxnative:break
                candidate=copy.deepcopy(current);t=candidate['tracks'][ti]
                if sp_items is not None:
                    vor=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][:i],t['pts'][1:i+1],t['w'][:i]) if math.dist(p,q)>1e-9]
                    nach=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][j:],t['pts'][j+1:],t['w'][j:]) if math.dist(p,q)>1e-9]
                    t['items']=vor+sp_items+nach
                t['pts']=t['pts'][:i]+points+t['pts'][j+1:]
                t['w']=t['w'][:i]+[w]*(len(points)-1)+t['w'][j:]
                for name in ('nm_pts','nm_w'):t.pop(name,None)
                if sp_items is not None:
                    from gemeinsam.bahnitems import validate as _bi_validate
                    try:_bi_validate(t)
                    except ValueError:continue
                report['versucht_nativ']+=1
                nd=native_routen_pruefen(board,candidate,optionen,folder/('kandidat_%03d'%report['versucht_nativ']))
                ne=native_skew_fehler(nd)
                oldscore=sum(max(0,q['delta_mm']-q['toleranz_mm']) for q in errors)
                newscore=sum(max(0,q['delta_mm']-q['toleranz_mm']) for q in ne)

                oldmap={(q['regel'],q['net']):q['delta_mm'] for q in errors}
                safe=all(q['delta_mm']<=oldmap.get((q['regel'],q['net']),q['toleranz_mm'])+1e-6 for q in ne)
                if allowed(drc,nd) and safe and newscore<oldscore-.005:
                    report['zuege'].append(dict(netz=net,regel=error['regel'],layer=layer,form=shape,geometrisch_plus_mm=added,
                       vorher_skew_mm=error['delta_mm'],rest=[q for q in ne if q['regel']==error['regel']],beleg=str(folder/('kandidat_%03d'%report['versucht_nativ']))))
                    current=candidate;drc=nd;moved=True;native_folder=folder/('kandidat_%03d'%report['versucht_nativ'])
                    print('Skew: %s +%.4f mm, Rest %s'%(net,added,[q['delta_mm'] for q in ne if q['regel']==error['regel']]),flush=True)
                    break
            if moved:break
        if not moved:break
    report.update(nach=native_skew_fehler(drc),ausgang_sha=sha(current),
                  native_ordner=str(native_folder),
                  drc_sha256=hashlib.sha256((native_folder/'drc.json').read_bytes()).hexdigest() if (native_folder/'drc.json').exists() else None,
                  pcb_sha256=hashlib.sha256((native_folder/'board.kicad_pcb').read_bytes()).hexdigest() if (native_folder/'board.kicad_pcb').exists() else None,
                  nativ_nach=dict(fehler=sum(v['severity']=='error' for v in drc['violations']),offen=len(drc['unconnected_items'])))
    current['skew_abgleich']=report
    (folder/'bericht.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return current


class _Etiketten:
    def __init__(self):
        self.cpu = 0.0
        self.an = bool(args.get('etiketten'))
        self.runden = []; self.konflikte = []; self.signaturen = {}; self.aenderung = {}
        self.namen = sorted(n for n in bynet if not n.startswith('#'))
        self.nid = {n: i for i, n in enumerate(self.namen)}
        self.nx = (NX + 4) // 5; self.ny = (NY + 4) // 5
        if self.an:
            self.hmax = np.zeros((NL, self.ny, self.nx), np.float32)
            self.kmap = np.zeros_like(self.hmax, dtype=np.uint32)
            self.karten = []

    def grob(self, a, mittel=False):
        a = np.asarray(a).reshape(NL, NY, NX)
        p = np.pad(a, ((0, 0), (0, self.ny * 5 - NY), (0, self.nx * 5 - NX)))
        p = p.reshape(NL, self.ny, 5, self.nx, 5)
        return p.mean((2, 4)) if mittel else p.max((2, 4))

    def zellen(self, c):
        c = np.asarray(c, dtype=np.int64)
        c = c[(c >= 0) & (c < NL * NN)]
        return (c // NN) * self.nx * self.ny + ((c % NN) // NX // 5) * self.nx + (c % NX) // 5

    def netz(self, u):
        if u in U: return U[u]['net']
        if u.startswith('GND#'): return D['pads'][int(u[4:])]['net']
        return u[4:] if u.startswith('FIX#') else u

    def konfliktkopie(self, conf):
        if not self.an: return None
        t = time.process_time()
        try: return {u: list(c) for u, c in conf.items()}
        except Exception as e:
            self.an = False; print('Etiketten: Konfliktkopie ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def ergaenzen(self, conf, eng):
        if not self.an: return
        t = time.process_time()
        try:
            for u, c in eng.items(): conf.setdefault(u, []).extend(c)
        except Exception as e:
            self.an = False; print('Etiketten: Konfliktergaenzung ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def beobachten(self, conf, durchgang, phase='verhandlung', geometrie=True):
        if not self.an or phase == 'erstaufbau': return
        t = time.process_time()
        try:
            counts = np.zeros(len(self.namen), np.uint32); cells = []
            for u, c in conf.items():
                n = self.netz(u)
                if n in self.nid: counts[self.nid[n]] += len(c)
                if len(c): cells.append(self.zellen(c))
            if cells:
                self.kmap.ravel()[np.unique(np.concatenate(cells))] += 1
            np.maximum(self.hmax, self.grob(hist), out=self.hmax)
            self.runden.append((durchgang, phase)); self.konflikte.append(counts)
            if geometrie:
                by = {}
                for u in sorted(U):
                    r = U[u]; by.setdefault(r['net'], []).append((r['tracks'], r['vias']))
                for n, g in by.items():
                    sig = json.dumps(g, sort_keys=True)
                    if self.signaturen.get(n) != sig:
                        self.signaturen[n] = sig; self.aenderung[n] = durchgang
            if args.get('etiketten_runden') == '1': self.karten.append(self.grob(hist).astype(np.float32))
        except Exception as e:
            self.an = False; print('Etiketten: Beobachtung ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def ausgabebelegung(self, tracks, vias, zonen):
        ""
        from nachbearbeitung.kurzschluss_schutz import ausgabebelegung
        return ausgabebelegung(dict(tracks=tracks, vias=vias, zonen=list(zonen), optionen=args),
                               LAYERS, np.array([X0,Y0]), (self.ny,self.nx))

    def schreiben(self, tracks, vias, nur_cache=False, fehlend=(), zonen=()):
        global _LERN_LABELS
        if not self.an: return
        t = time.process_time()
        try:
            import os, tempfile
            nr = len(self.namen); a = np.zeros((nr, 7), np.float32)
            failed = {self.netz(u) for u in set(fail) | set(fehlend)}
            need = {self.netz(u) for u in units}
            if globals().get('_LERN_AN'): failed |= {self.netz(u) for u in set(units) - set(U) - set(FIXED)}
            vorhanden = {r['net'] for r in U.values()}
            _r45_label = (globals().get('_sicher') or {}) if not nur_cache else {}
            _r45_board = _r45_label.get('rest_board')
            if _r45_board: vorhanden = {q['net'] for q in tracks+vias}
            lm = np.zeros((nr, NL), bool); vmap = np.zeros_like(self.kmap)
            for n, i in self.nid.items():
                ps = ([p for p in _r45_board['pads'] if p['net'] == n] if _r45_board else [D['pads'][k] for k in bynet[n]])
                hp = max(p['x'] for p in ps) - min(p['x'] for p in ps) + max(p['y'] for p in ps) - min(p['y'] for p in ps)
                a[i, 0] = n not in failed and (n not in need or n in vorhanden)
                a[i, 2] = hp; a[i, 6] = self.aenderung.get(n, 0)
            for tr in tracks:
                i = self.nid.get(tr['net'])
                if i is None: continue
                a[i, 1] += sum(math.dist(x, y) for x, y in zip(tr['pts'][:-1], tr['pts'][1:]))
                lm[i, LI[tr['layer']]] = True
            for v in vias:
                i = self.nid.get(v['net'])
                if i is not None: a[i, 4] += 1; lm[i, :] = True
                x = int((v['x'] - X0) / .25); y = int((v['y'] - Y0) / .25)
                if 0 <= x < self.nx and 0 <= y < self.ny: vmap[:, y, x] += 1
            a[:, 3] = np.divide(a[:, 1], a[:, 2], out=np.zeros(nr, np.float32), where=a[:, 2] > 0)
            a[:, 5] = lm.sum(1)
            search_occ = self.grob(use > 0, mittel=True).astype(np.float16)
            protected = args.get('kurzschluss_schutz', '0') == '1'
            occ = self.ausgabebelegung(tracks, vias, zonen) if protected else search_occ
            data = dict(hist_context=np.asarray(args.get('hist_context', '')), etikettenversion=np.asarray(2 if protected else 1, np.int16), beobachterversion=np.asarray(4 if protected else 3, np.int16), origin_mm=np.array([X0, Y0]), raster_mm=np.asarray(.25),
                        lagen=np.asarray(LAYERS), history_max=self.hmax, history_final=self.grob(hist),
                        belegung=occ, belegung_suche=search_occ, belegung_vertrag=np.asarray('ausgabekupfer_zellkontakt' if protected else 'suchbelegung_anteil'),
                        konflikt_runden=self.kmap, vias_raster=vmap,
                        netze=np.asarray(self.namen), netz_spalten=np.asarray(['geroutet', 'laenge_mm', 'hpwl_mm', 'umweg_hpwl', 'vias', 'lagen_n', 'letzte_aenderung']),
                        netz_werte=a, netz_lagen=lm, netz_konflikte=np.asarray(self.konflikte, np.uint32).reshape(-1, nr),
                        durchgaenge=np.asarray([r[0] for r in self.runden], np.int32), phasen=np.asarray([r[1] for r in self.runden]))
            if not protected:
                del data['belegung_suche']; del data['belegung_vertrag']
            if _r45_board:
                data['r45_placement_json'] = np.asarray(json.dumps(_r45_label['rest_placement'], sort_keys=True))
                data['r45_zielpad_json'] = np.asarray(json.dumps(_r45_board['pads'], sort_keys=True))
            if self.karten: data['history_runden'] = np.asarray(self.karten)
            if nur_cache:
                _LERN_LABELS = {k: v.copy() for k, v in data.items()}
                return
            dest = os.path.abspath(args['etiketten']); os.makedirs(os.path.dirname(dest), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
            try:
                np.savez_compressed(tmp, **data); os.replace(tmp, dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            print('Etiketten: %s (%d Netze, %d Beobachtungen, %.3f CPU-s)' % (dest, nr, len(self.runden), self.cpu + time.process_time() - t), flush=True)
        except Exception as e: print('Etiketten: Schreiben ausgelassen:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t


_ET = None



bynet = {}
for k, p in enumerate(D['pads']):
    if p['net'] and not p['net'].startswith('unconnected'):
        bynet.setdefault(p['net'], []).append(k)
def span(n):
    pads = [D['pads'][k] for k in bynet[n]]
    return max(p['x'] for p in pads) - min(p['x'] for p in pads) + max(p['y'] for p in pads) - min(p['y'] for p in pads)


def lern_reihenwert(n, value):
    ""
    if KL is not None and n in bynet:
        art=(KL['netze'].get(n) or {}).get('art','signal')
        factor=float(args.get('kosten_rang_'+art,'1'))
        if factor!=1.:return value-(factor-1.)*math.log1p(span(n))/5.
    return value


def lern_startspan(n, power=False):
    value=span(n)
    if KL is not None:
        art=(KL['netze'].get(n) or {}).get('art','signal')
        factor=float(args.get('kosten_rang_'+art,'1'))
        if factor!=1.:return value*(4*factor-3 if power else 3-2*factor)
    return value
NETTIES = set(KL.get('net_ties', [])) if KL else set()
def grp(n):
    if n in PLANE_GROUPS:return (4.8,0)
    if paar_schnell(n):
        k = KL['netze'][n]
        return (-2, -k.get('paar_rang', k.get('rang', 0)), min(n, PAAR_NETZE[n]), n)
    if strom_modell(n): return (-1.5, -sum(map(float,strom_modell(n)['last'].values())), n)
    if n.startswith('GND#'):
        r_ = D['pads'][int(n[4:])]['ref']
        if r_.startswith('NT_') or r_ in NETTIES: return (-1, 0) if KL else (0, 0)
        if int(n[4:]) in VAP_WICHTIG: return (4.8, 0)
        return (5, 0) if KL else (4, 0)
    if KL is not None and n in KL['netze']:
        k = KL['netze'][n]; g = k.get('gruppe', 4)
        _rang_f = float(args.get('kosten_rang_' + k.get('art', 'signal'), '1'))
        rang = k.get('rang', 0) * float(args.get('kosten_rang_' + k.get('art', 'signal'), '1'))
        if g == 2: return (2 - .1 * (_rang_f-1), -lern_startspan(n,power=True))
        if REIHENFOLGE >= 2:
            if g == 1 and k.get('rang', 0) < SCHNELL_AB: g = 3.5
            elif g == 4 and k.get('rang', 0) <= 0: g = 4.5
        return (g - .1 * (_rang_f-1), -rang, lern_startspan(n))
    pr = cls(n)['prio']
    if pr <= 1: return (0, span(n))
    if pr in (2, 3, 5): return (1, -span(n))
    if pr < 8: return (2, span(n))
    return (3, span(n))
units = [n for n in bynet if n not in GNDS and n not in FLNETZE and len(bynet[n]) > 1 and not n.startswith('#')]
if args.get('nur') == 'leistung':
    units = [n for n in units if cls(n)['prio'] < 8]
esp61 = 0
OHNE = {}
for spec in args.get('gnd_ohne_via', '').split(';'):
    if ':' in spec: r_, nums = spec.split(':'); OHNE[r_] = set(nums.split(','))
for _g in sorted(FLNETZE):
    for k in bynet.get(_g, []):
        if D['pads'][k]['num'] in OHNE.get(D['pads'][k]['ref'], ()): continue
        if any(L in GNDL for L in D['pads'][k]['layers']): continue
        units.append('GND#%d' % k)

units += [g for g in sorted(GNDS - FLNETZE) if len(bynet.get(g, [])) > 1]
PLANE_GROUPS = {}
if PLANE_FEEDERS:
    for _n in sorted(PLANE_MIXED_NETS):
        _far=[k for k in bynet.get(_n,[]) if PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y']))>6.]
        if not _far:continue
        PLANE_GROUPS[_n]=sorted(_far,key=lambda k:(PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y'])),k))
        _remove={'GND#%d'%k for k in _far};units=[u for u in units if u not in _remove];units.append(_n)
FIXGND = {'FIX#' + g for g in FLNETZE}
FIXED = []
if 'fix' in args:
    FX = json.load(open(args['fix']))
    if PINZUGANG and any(abs(float(v.get('d',VIA_D))-VIA_D)>1e-9 or abs(float(v.get('drill',VIA_DR))-VIA_DR)>1e-9 for v in FX['vias']):
        raise ValueError('pinzugang: fix= mit gemischten Via-Massen im globalen Routermodell nicht unterstuetzt')
    byn = {}
    for t in FX['tracks']: byn.setdefault(t['net'], ([], []))[0].append(dict(layer=t['layer'], pts=t['pts'], w=t['w']))
    for v in FX['vias']: byn.setdefault(v['net'], ([], []))[1].append(dict(x=v['x'], y=v['y']))
    for n, (tr, vs) in byn.items():

        add_unit('FIX#' + n, n, tr, vs); FIXED.append('FIX#' + n)
    fixnets = set(byn)
    if args.get('fixhart', '1') == '1':
        fo = np.zeros(NL * NN, dtype=bool); fdr = np.zeros(NN, dtype=bool)
        for u in FIXED:
            fo[U[u]['cells']] = True; fdr[U[u]['dcells']] = True
        FIXD[0] = [(_edt(~fo[li * NN:(li + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for li in range(NL)]
        FIXD[1] = (_edt(~fdr.reshape(NY, NX)) * RES).astype(np.float32)
        FIXNETS.update(fixnets); _HM.clear(); _ERR.clear(); _FIX_KARTEN.clear()
        print('feste Kupfer als harte Hindernisse')


    def fix_pad_verbunden(k):
        p = D['pads'][k]; net = p['net']; tr, vs = byn.get(net, ([], []))
        for L in p['layers']:
            g = shapely.union_all([Polygon(q) for q in p['polys'].get(L, [])])
            kupfer = [g] + [_LS((a, b)).buffer(w / 2) for t in tr if t['layer'] == L for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            seen = {0}; offen = [0]
            while offen:
                j = offen.pop()
                if any(kupfer[j].distance(Point(v['x'], v['y'])) <= VIA_R + 1e-7 for v in vs): return True
                for q, h in enumerate(kupfer):
                    if q not in seen and kupfer[j].distance(h) <= 1e-7: seen.add(q); offen.append(q)
        return False
    units = [u for u in units if (not fix_pad_verbunden(int(u[4:])) if u.startswith('GND#') else u not in fixnets)]
    print('fest:', len(FIXED), 'Netze aus', args['fix'])
units.sort(key=grp)
if BGA_FANOUT:

    def bga_ring(u):
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        return min((BGA[id(D['pads'][k])]['ring'] for k in ks if id(D['pads'][k]) in BGA), default=-1)
    units.sort(key=lambda u: (grp(u)[:2], bga_ring(u), grp(u)[2:], u))
if PAAR_NETZE:
    _positions = {u:i for i,u in enumerate(units)}
    units.sort(key=lambda u: (grp(u)[0], -(KL['netze'].get(u) or {}).get('paar_rang',0),
                             min(u,PAAR_NETZE[u]) if u in PAAR_NETZE else '', u if u in PAAR_NETZE else _positions[u]))
print('Einheiten', len(units), '(Netze', sum(1 for u in units if not u.startswith('GND#')), ')')
if KL is not None and (REIHENFOLGE >= 2 or SCHNELL_LAGE > 0 or SCHNELL_INNEN > 0 or SCHNELL_VIA != 1 or GEWICHTET):
    _sn = [u for u in units if ist_schnell(u)]; _lfs = sorted({cls(u).get('lf') for u in _sn if cls(u).get('lf')})
    print('Regeln R1/R14/R21: reihenfolge=%d, schnell_ab=%d (%d schnelle, %d mittelschnelle Netze), schnell_via=%g, gewichtet=%d; Lagen %s' % (
        REIHENFOLGE, SCHNELL_AB, len(_sn), sum(1 for u in units if u in KL['netze'] and KL['netze'][u].get('art') == 'schnell' and not ist_schnell(u)),
        SCHNELL_VIA, GEWICHTET, ', '.join('%s %s' % (L, {2: 'neben Masse', 1: 'neben Flaeche', 0: 'ohne Flaeche'}[LAGE_BEZUG[i]]) for i, L in enumerate(LAYERS))) +
        ('; Kostenfaktor schneller Netze: %s' % ' / '.join(', '.join('%s x%.2f' % (L, f) for L, f in zip(LAYERS, lf)) for lf in _lfs) if _lfs else '; Lagenwahl ohne Wirkung (alle Lagen gleich gut oder schnell_lage=0)'), flush=True)
MAXIT = int(args.get('iter', '40')); HINC = float(args.get('hinc', '0.4'))
PFMAX = float(args['pfmax']) if 'pfmax' in args else None
if PFMAX is not None and (not math.isfinite(PFMAX) or PFMAX <= 0): raise ValueError('pfmax muss endlich und > 0 sein')
if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
t0 = time.time()
fail = set()



CPU_MAX = float(args.get('cpu_max', '0'))
if _LERN_AN:
    args.setdefault('verlauf', os.path.join(os.path.dirname(OUT), 'verlauf.jsonl'))
_VERLAUF = open(args['verlauf'], 'w') if args.get('verlauf') else None
_VERLAUF_CPU = [time.process_time()]
_LERN_ROWS = []
_LERN_STOP = None
_LERN_BASIS = CPU_MAX
_LERN_MAX = float(args.get('lern_cpu_max', str(2 * CPU_MAX))) if args.get('lernbudget') == 'adaptiv' else CPU_MAX
_LERN_POLICY = None
_FRUEH_MODELL = None
_FRUEH_INFO = None
if _FRUEH_AN:
    from lernen import fruehabbruch as _FRUEH
    try:
        _FRUEH.generator_board_pruefen(D)
        _FRUEH_KONTEXT = _FRUEH.kontext_pruefen(
            json.loads(args['fruehabbruch_kontext']) if args.get('fruehabbruch_kontext')
            else _FRUEH.kontext_board(D, NL))
        _FRUEH_MODELL = _FRUEH.modell_laden(_FRUEH.modellpfad(args))
    except (OSError, ValueError, KeyError, TypeError) as _e:
        print('Fruehabbruch gesperrt: Modell-/Kontextfehler %s' % repr(_e)[:180], flush=True)
if _LERN_AN and args.get('lernpolicy'):
    _LERN_POLICY = json.load(open(args['lernpolicy']))
    if _LERN_POLICY.get('version') != 1 or _LERN_POLICY.get('status') != 'experimentell':
        raise ValueError('lernpolicy: version=1, status=experimentell erforderlich')


def ausgabe_geometrie():
    tracks = []; vias = []
    for u, r in U.items():
        if args.get('rest_reparatur') == '1':
            tracks.extend(dict(t,net=r['net'],**({'strom_steg': True} if t.get('steg') else {})) for t in r['tracks'])
            vias.extend(dict(v,net=r['net'],**({'strom_via': True} if v.get('gr') else {})) for v in r['vias'])
            continue
        for t in r['tracks']: tracks.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w'], sig=bool(t.get('sig')), **({'strom_steg': True} if t.get('steg') else {}), **({'kette': True} if t.get('kette') else {})))
        for v in r['vias']: vias.append(dict(net=r['net'], x=v['x'], y=v['y'], strom_via=True) if v.get('gr') else
                                        (dict(net=v['net'], x=v['x'], y=v['y'], rueck_via=True) if v.get('rv') else dict(net=r['net'], x=v['x'], y=v['y'])))
    if args.get('kurzschluss_schutz', '0') == '1':
        for q, u in zip(tracks, [u for u,r in U.items() for t in r['tracks']]): q['einheit'] = u if u.startswith('GND#') else q['net']
        for q, u in zip(vias, [u for u,r in U.items() for v in r['vias']]): q['einheit'] = u if u.startswith('GND#') else q['net']
    for _t, _q in zip(tracks, [t for r in U.values() for t in r['tracks']]):
        if _q.get('abg'): _t['kelvin'] = True
    if globals().get('ZONENPLAN'): vias += _ZP_VIAS
    for t in tracks:
        if _pz_active(t['net']):
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
            t['nm_pts']=[[_pz_nm(v) for v in p] for p in t['pts']]
            t['nm_w']=[_pz_nm(w) for w in t['w']]
    for v in vias:
        if _pz_active(v['net']):
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
            v['nm_xy']=[_pz_nm(v['x']),_pz_nm(v['y'])]
    return tracks, vias


def ausgabe_kontakte(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.kurzschluss_schutz import ausgabe_kontakte as impl
    return impl(*pos, **kw)


def ausgabe_auswahl(*pos, **kw):
    from nachbearbeitung.kurzschluss_schutz import ausgabe_auswahl as impl
    return impl(*pos, **kw)


def ausgabe_luftlinien(*pos, **kw):
    from nachbearbeitung.kurzschluss_schutz import ausgabe_luftlinien as impl
    return impl(*pos, **kw)


def ausgabe_sicher(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.kurzschluss_schutz import ausgabe_sicher as impl
    return impl(*pos, **kw)



def ausgabe_daten(tracks, vias, conf, fehlend=None, checkpoint=False, roh=False, **extra):
    ""
    data = dict(tracks=tracks, vias=vias, failed=sorted(fail if fehlend is None else fehlend),
        conflicts=conf, res=RES, clr=CLR, optionen=args,
        unerreichbar={n:v for n,v in sorted(UNERREICHBAR.items())},
        flaechennetze=sorted(FLNETZE), gndlagen=GNDL, unter_wmin=unter_wmin())
    if ARBEIT_MESSEN: data['arbeitsbudget'] = arbeit_beleg()
    if KA_VERTRAG is not None:
        data['abstaende'] = KA_VERTRAG
        data['abstand_legacy_deckt'] = KA_LEGACY_DECKT
    if PINZUGANG:
        data['pinzugang_exakt']=dict(breiten=PZ_BREITEN,minimax=globals().get('PZ_MINIMAX'),kanten=PZ_EDGE_STAT,lokal={n:len(ps) for n,ps in sorted(PZ_LOKAL.items())},rand_nm=PZ_RAND_NM)
    if globals().get('BREIT_DICHT_BERICHT') is not None: data['breit_dicht'] = BREIT_DICHT_BERICHT
    if globals().get('STROM_ZUERST'):
        data['strom_zuerst'] = dict(netze=STROM_AUDIT, potenziallagen=STROM_POT, J=STROM_J,
            ohne_modell=sorted(n for n,k in KL['netze'].items() if k.get('art') in ('strom','versorgung','sw') and not strom_modell(n)))
    if globals().get('ZONENPLAN'): data.update(zonenplan_id=ZONENPLAN['id'], zonen=ZONENPLAN['kern'], native_vias=True)
    if PLANE_INFO: data['plane_domains'] = PLANE_INFO
    if PLANE_GROUPS:data['plane_groups']={n:['%s:%s'%(D['pads'][k]['ref'],D['pads'][k]['num']) for k in ks] for n,ks in sorted(PLANE_GROUPS.items())}
    if POTENZIAL_PLAN: data['potenzial_plan'] = POTENZIAL_PLAN
    if IMP_TAB: data['impedanz_tabelle'] = IMP_TAB
    if FIX_PAIRS: data['fix_abstaende'] = FIX_PAIRS
    if args.get('fix'):
        data['fix_netze'] = sorted({r['net'] for u, r in U.items() if u.startswith('FIX#')})
    gruppen = []
    for r in U.values():
        for v in r['vias']:
            if v.get('n'):
                gruppen.append(dict(netz=r['net'], x=v['x'], y=v['y'], soll=v['n'],
                    ist=1 + sum(1 for w in r['vias'] if w.get('zu') == [v['x'], v['y']])))
    if gruppen: data['via_gruppen'] = gruppen
    for key, name, enabled in (
        ('r19_kurz', 'R19_KURZ_STAT', globals().get('R19_KURZ')),
        ('r19_form', 'R19_FORM_STAT', globals().get('R19_FORM')),
        ('laengenabgleich', 'LAENGEN_BERICHT', globals().get('LAENGEN_BERICHT') is not None),
        ('massepad_vias', 'MPV', True), ('via_neben_pad', 'VAPB', True),
        ('rueck_vias', 'RVB', True), ('paare', 'PAB', True),
        ('vias_sparen', 'VS_LOG', args.get('vias_sparen') == '1'),
        ('via_feinschub', 'FEINSCHUB_LOG', args.get('via_feinschub') == '1'),
        ('via_dichte_nach', 'VD_NACH_LOG', globals().get('VD_NACH')),
        ('bga_fanout', 'BGB', globals().get('BGA_FANOUT'))):
        if enabled and name in globals() and (globals()[name] or key in ('laengenabgleich', 'vias_sparen', 'via_feinschub', 'bga_fanout')):
            data[key] = globals()[name]
    if globals().get('ST') and ST.get('rest_board'):
        if args.get('neu'):
            from nachbearbeitung.restkonflikte import placement_marker
            data.update(placement_marker(ST['rest_board'],copy.deepcopy(_REST_BOARD_SOURCE)))
        else:
            for _field in ('rest_board','rest_quelle','rest_moves','rest_placement','rest_native_sources'):
                if ST.get(_field) is not None: data[_field] = ST[_field]
    if args.get('restkonflikte') == '1' and globals().get('ST'):
        for _field in ('zones','zonen','r41_fuellung_lagen'):
            if ST.get(_field): data[_field] = ST[_field]
    if globals().get('REST_BERICHT') is not None: data['restkonflikte'] = REST_BERICHT
    if globals().get('_RL') is not None:
        data['routing_loop'] = _RL.summary()
        if _RL.board(D) != _RL.initial_board:
            data.update(_RL.rest.placement_marker(_RL.initial_board, _RL.board(D)))
    data.update(extra)
    if globals().get('_FRUEH_INFO') is not None: data['fruehabbruch'] = _FRUEH_INFO




    if roh or args.get('kurzschluss_schutz', '0') != '1': return data
    return r44_ausgabe(data, checkpoint=checkpoint)
def r44_ausgabe(data, checkpoint=False, board=None, paare=None, einheiten_extra=None):
    ""
    board = D if board is None else board
    einheiten = {u:r['net'] for u,r in U.items()}
    for u, n in (einheiten_extra or {}).items(): einheiten.setdefault(u, n)
    ausgabenetze = {q['net'] for k in ('tracks', 'vias', 'zonen', 'zones') for q in data.get(k, [])}
    wichtig = {n: max(1, 20 - int(cls(n).get('prio', 10))) for n in ausgabenetze}
    wichtig.update({u:wichtig.get(n, 1) for u,n in einheiten.items() if u.startswith('GND#')})
    result = ausgabe_sicher(board, data, einheiten, wichtig, list(PAIRS) if paare is None else list(paare), checkpoint=checkpoint)
    if checkpoint:
        result = dict(result)
        result['kurzschluss_einheiten'] = {u: (board['pads'][int(u[4:])]['net'] if u.startswith('GND#') else einheiten.get(u, u[4:] if u.startswith('FIX#') else u)) for u in result['failed']}
    return result


def rest_abschluss(result):
    ""
    paare = result.pop('rest_check_paare', None)
    if args.get('kurzschluss_schutz', '0') != '1': return result
    if result.get('restkonflikte') is not None:
        result['restkonflikte']['vor_r44'] = dict(konflikte=len(result['conflicts']), offen=len(result['failed']))
    board = result.get('rest_board') or D

    extra = {}
    for u in list(result['conflicts']) + [q for p in (paare or ()) for q in p[2:4]]:
        if u in extra or u == 'PAD': continue
        if u.startswith('FIX#'): extra[u] = u[4:]
        elif u.startswith('GND#'): extra[u] = board['pads'][int(u[4:])]['net']
        else: extra[u] = u
    return r44_ausgabe(result, board=board, paare=paare, einheiten_extra=extra)


_LERN_CPU = 0.0
_LERN_TIEFE = 0


def beobachter_cpu():
    return _LERN_CPU + (_ET.cpu if _ET else 0.0)


def lern_beobachter(fn):
    ""
    def messen(*pos, **kw):
        global _LERN_CPU, _LERN_TIEFE
        if not _LERN_AN and _VERLAUF is None: return fn(*pos, **kw)
        aussen = _LERN_TIEFE == 0
        start = time.process_time(); etik = _ET.cpu if _ET else 0.0
        _LERN_TIEFE += 1
        try: return fn(*pos, **kw)
        finally:
            _LERN_TIEFE -= 1
            if aussen:
                _LERN_CPU += max(0.0, time.process_time() - start - ((_ET.cpu if _ET else 0.0) - etik))
    return messen


@lern_beobachter
def lern_snapshot(runde, conf, phase):
    global _LERN_SNAPSHOT


    tracks, vias = ausgabe_geometrie()
    data = ausgabe_daten(tracks, vias, {u: len(c) for u,c in conf.items()}, checkpoint=True,
        fehlend=set(fail) | (set(units) - set(U) - set(FIXED)),
        lernstand=dict(runde=runde, phase=phase, cpu_s=time.process_time(), paare=list(PAIRS), zensiert=True),
        lernbudget=dict(basis_cpu_s=_LERN_BASIS, aktuell_cpu_s=CPU_MAX, max_cpu_s=_LERN_MAX))


    import signal
    oldmask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGXCPU})
    try:
        if _ET:
            _ET.beobachten(conf, runde, phase)
            _ET.schreiben(data['tracks'], data['vias'], nur_cache=True, fehlend=data['failed'], zonen=data.get('zonen', []) + data.get('zones', []))
        _LERN_SNAPSHOT = json.dumps(data)
    finally: signal.pthread_sigmask(signal.SIG_SETMASK, oldmask)


def lern_entscheidung():
    global CPU_MAX, _LERN_STOP, _FRUEH_INFO
    if not _LERN_AN: return
    if globals().get('_FRUEH_MODELL') is not None and args.get('lernbudget') != 'adaptiv':
        _info = _FRUEH.entscheidung(_LERN_ROWS, _FRUEH_KONTEXT, CPU_MAX, _FRUEH_MODELL)
        if _info:
            _FRUEH_INFO = _info
            _LERN_STOP = 'fruehabbruch'
            print('Fruehabbruch: Runde %d, %d Konflikte, Score %.6f >= %.6f, %.1f CPU-s' %
                  (_info['runde'], _info['konflikte'], _info['score'], _info['schwelle'], _info['cpu_s']), flush=True)
    if _LERN_POLICY or args.get('lernbudget') == 'adaptiv':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from lernen.lerneffekt import entscheidung, vielversprechend
        if _LERN_POLICY and entscheidung(_LERN_ROWS, len(D['pads']), _LERN_POLICY):
            _LERN_STOP = 'fruehabbruch'
        elif CPU_MAX > 0 and time.process_time() >= .65 * CPU_MAX and CPU_MAX < _LERN_MAX and vielversprechend(_LERN_ROWS):
            CPU_MAX = min(_LERN_MAX, max(CPU_MAX * 1.5, time.process_time() / .6))
            print('Lernbudget: sinkende Konflikte, verlaengert auf %.1f CPU-s' % CPU_MAX, flush=True)


@lern_beobachter
def verhandlung_messen(runde, conf, phase='verhandlung'):
    if _VERLAUF is None: return
    cpu = time.process_time()
    cells = np.unique(np.concatenate([np.asarray(c, dtype=np.int64) for c in conf.values()])) if conf else np.empty(0, dtype=np.int64)
    h = hist[cells]
    row = dict(runde=runde, phase=phase, cpu_s=cpu, cpu_runde_s=cpu - _VERLAUF_CPU[0], pf=float(PF[0]),
               konflikte=len(conf), offen=len(fail), paare=list(PAIRS),
               hist_max=float(hist.max()), hist_konflikt_max=float(h.max()) if len(h) else 0.0,
               hist_konflikt_mittel=float(h.mean()) if len(h) else 0.0,
               hist_konflikt_null=float(np.mean(h == 0)) if len(h) else 0.0,
               netze=sorted(conf), vias=sum(len(r['vias']) for r in U.values()))
    _VERLAUF.write(json.dumps(row) + '\n'); _VERLAUF.flush(); _VERLAUF_CPU[0] = time.process_time()
    if _LERN_AN:
        _LERN_ROWS.append(row)
        lern_snapshot(runde, conf, phase)
        if phase == 'verhandlung': lern_entscheidung()

if args.get('etiketten'):
    _etik_c0 = time.process_time()
    try:
        _ET = _Etiketten(); _ET.cpu += time.process_time() - _etik_c0
    except Exception as _e: print('Etiketten: Initialisierung ausgelassen:', repr(_e)[:180], flush=True)


@lern_beobachter
def erstaufbau_checkpoint():

    verhandlung_messen(0, exact_conflicts(), 'erstaufbau')


def such_cpu():


    return time.process_time() + (getattr(globals().get('_AL'), 'cpu_offset', 0.))


def budget_meldung():
    if BUDGET_ARBEIT:
        return 'Arbeitsbudget (arbeit_max=%d, verbraucht=%d Auftraege)' % (ARBEIT_MAX, arbeit_gesamt())
    return 'Rechenzeit-Budget (cpu_max=%.0f s)' % CPU_MAX

def zeit_knapp(anteil=0.8):
    if BUDGET_ARBEIT:

        arbeit_notbremse()
        return arbeit_gesamt() >= anteil * ARBEIT_MAX
    if _LERN_AN and _LERN_STOP: return True
    if _LERN_AN and (args.get('lernbudget') == 'adaptiv' or _LERN_POLICY):
        return CPU_MAX > 0 and time.process_time() > anteil * CPU_MAX


    return CPU_MAX > 0 and (such_cpu() - beobachter_cpu()) > anteil * CPU_MAX


todo = None
if _LERN_AN: lern_snapshot(0, {}, 'vor_routing')
if _AGENT_SAVED is not None:
    pass
elif 'start' in args:

    ST = _REST_START_SOURCE if _REST_START_SOURCE is not None else json.load(open(args['start']))
    byn = {}
    for t in ST['tracks']: byn.setdefault(t['net'], ([], []))[0].append(t)
    for v in ST['vias']:
        _sv = dict(v) if args.get('rest_reparatur') == '1' else dict(x=v['x'], y=v['y'])
        if v.get('strom_via'): _sv['gr'] = 1
        byn.setdefault(v['net'], ([], []))[1].append(_sv)


    NEU = {r for r in args.get('neu', '').split(',') if r}
    neu_nets = {D['pads'][k]['net'] for k in range(len(D['pads'])) if D['pads'][k]['ref'] in NEU} - FLNETZE - {''}
    neu_nets |= {n for n in args.get('neu_netze', '').split(',') if n}
    neu_nets = {n for n in neu_nets if n in bynet and len(bynet[n]) > 1}
    zone = [(p['x'], p['y']) for p in D['pads'] if p['ref'] in NEU]
    gnd_neu = [k for k, p in enumerate(D['pads']) if p['net'] in FLNETZE and
               (p['ref'] in NEU or any(math.hypot(p['x'] - a, p['y'] - b_) < 1.5 for a, b_ in zone))]


    fail |= {n for n in (ST.get('failed') or []) if n in units and n not in neu_nets} - {'GND#%d' % k for k in gnd_neu}
    def in_pad(x, y, ks):
        for k in ks:
            bb = D['pads'][k]['bbox']
            if bb[0] - 0.05 <= x <= bb[2] + 0.05 and bb[1] - 0.05 <= y <= bb[3] + 0.05: return True
        return False
    for n, (tr, vs) in byn.items():
        if n in neu_nets: continue
        if n in FLNETZE:
            weg_v = set()
            tr_keep = []
            for t in tr:
                ends = [tuple(t['pts'][0]), tuple(t['pts'][-1])]
                if any(in_pad(x, y, gnd_neu) for x, y in ends):
                    for x, y in ends: weg_v.add((round(x, 3), round(y, 3)))
                    continue
                tr_keep.append(t)
            vs_keep = [v for v in vs if (round(v['x'], 3), round(v['y'], 3)) not in weg_v and not in_pad(v['x'], v['y'], gnd_neu)]
            add_unit('FIX#' + n, n, [dict(t) if args.get('rest_reparatur') == '1' else dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr_keep], vs_keep); FIXED.append('FIX#' + n)
            continue
        C = cls(n); Cs = cls('/__signal__')


        _wk = lambda t: (Cs if t.get('sig') else C)['wmin']
        tr2 = [dict(t) if args.get('rest_reparatur') == '1' else dict(net=n, layer=t['layer'], pts=t['pts'], w=[(wi if args.get('rest_pruefung') == '1' else min(wi, _wk(t))) for wi in t['w']], sig=bool(t.get('sig')), **({'steg': True} if t.get('strom_steg') else {})) for t in tr]
        add_unit(n, n, tr2, vs)
        for _t2, _t in zip(tr2, tr):
            if _t.get('kelvin'): _t2['abg'] = True
    if args.get('rest_check') == '1':
        _REST_LOKAL_KONFLIKT = True
        _rc = exact_conflicts(); _rt, _rv = ausgabe_geometrie()
        json.dump(ausgabe_daten(_rt, _rv, {u: len(v) for u,v in _rc.items()}, roh=True, rest_check_paare=list(PAIRS)), open(OUT, 'w'))
        sys.exit(0)
    if args.get('rest_reparatur') == '1':
        from nachbearbeitung.restkonflikte import reparatur_ausgabe
        reparatur_ausgabe(globals())
        sys.exit(0)
    if NEU or neu_nets:

        neu_units = sorted(neu_nets, key=lambda n: (grp(n), n)) + ['GND#%d' % k for k in gnd_neu]
        for u in paar_reihe(neu_units):
            if route_unit(u): fail.difference_update(paar_netze([u]))
            else:
                if paar_einheit(u): fail.update((u,PAAR_NETZE[u]))
                else: fail.add(u)
        if args.get('rest_teil_lokal') == '1':


            _REST_LOKAL_KONFLIKT = True
            _rc=exact_conflicts();_rt,_rv=ausgabe_geometrie()
            json.dump(ausgabe_daten(_rt,_rv,{u:len(v) for u,v in _rc.items()},roh=True,rest_check_paare=list(PAIRS)),open(OUT,'w'))
            sys.exit(0)
        units = [u for u in units if not u.startswith('GND#')] + ['GND#%d' % k for k in gnd_neu]
        args['alle'] = '0'
        print('Start aus', args['start'], ': neu verlegt', len(neu_units), 'Einheiten fuer', sorted(NEU), flush=True)
    else:
        units = [u for u in units if not u.startswith('GND#')]
        MAXIT = (MAXIT if args.get('loop') == '1' or args.get('agent_loop') == '1' else (2 if PAAR_SUCHRAUM else 1)); conf = exact_conflicts(); todo = (set(conf) | fail | ({u for u in units if paar_einheit(u)} if PAAR_SUCHRAUM else set())) - set(FIXED)
        print('Start aus', args['start'], ':', len(U), 'Einheiten geladen', flush=True)
else:
    _lern_cp_cpu = time.process_time()
    _reihe = paar_reihe(units)
    for _i, u in enumerate(_reihe):
        if (_LERN_AN and args.get('lernbudget') == 'adaptiv' and CPU_MAX > 0
                and CPU_MAX < _LERN_MAX and _i >= max(1, len(_reihe) // 2)
                and time.process_time() >= .65 * CPU_MAX):



            CPU_MAX = min(_LERN_MAX, max(CPU_MAX, time.process_time() * len(_reihe) / _i / .65))
            print('Lernbudget: Erstaufbau-Fortschritt, %.1f CPU-s' % CPU_MAX, flush=True)
        if zeit_knapp():
            rest = paar_netze(_reihe[_i:]); fail.update(rest); print('%s im ersten Durchgang verbraucht: %d Einheiten nicht verlegt' % (budget_meldung(), len(rest)), flush=True); break
        if not route_unit(u):
            if paar_einheit(u): fail.update((u,PAAR_NETZE[u]))
            else: fail.add(u)
        if u in PAAR_NETZE and u > PAAR_NETZE[u] and u not in fail and PAAR_NETZE[u] not in fail:
            paar_koppeln(nur={u, PAAR_NETZE[u]})
        if _LERN_AN and time.process_time() - _lern_cp_cpu >= 10:
            erstaufbau_checkpoint()
            _lern_cp_cpu = time.process_time()
print('Iteration 1: %.0fs, unvollstaendig %d' % (time.time() - t0, len(fail)))
class _RoutingLoop:
    VERSION = 'routing-loop-v4-local-cpu'

    def __init__(self, G):
        import hashlib
        from nachbearbeitung import restkonflikte as rest
        from analyse.bewertung import bewerte
        self.G, self.rest, self.bewerte = G, rest, bewerte
        self.initial_board = self.board(G['D'])
        a = G['args']
        G['_LOOP_START_CPU'] = time.process_time()
        code=Path(G['__file__']).resolve();root=code.parents[1]
        self.quellen={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in
                      [('router',code),('klassen_code',root/'analyse/klassen.py')]+
                      [(name,Path(a[name])) for name in ('klassen','regeln') if a.get(name)]}
        self.quellen['board']=hashlib.sha256(json.dumps(self.initial_board,sort_keys=True).encode()).hexdigest()
        self.fraction = self.number(a, 'loop_anteil', .10, 0., .5)
        self.explicit_budget = 'loop_cpu' in a
        self.limit = self.number(a, 'loop_cpu', self.budget(G, self.fraction), 0., 86400.)
        self.move_cpu = self.number(a, 'loop_zug_cpu', 5., .01, 300.)
        self.large = len(G['units']) >= 400
        self.threshold = int(self.number(a, 'loop_schwelle', 80, 1, 1000000))
        if a.get('loop_karten','lokal') not in ('lokal','voll'):
            raise ValueError('loop_karten=lokal|voll')
        if a.get('loop_suche','begrenzt') not in ('begrenzt','numba'):
            raise ValueError('loop_suche=begrenzt|numba')
        self.counts = []
        self.prefiltered = {}; self.rejected = {}
        self.k = int(self.number(a, 'loop_k', 4, 1, 100))
        self.n = int(self.number(a, 'loop_n', 4, 1, 100))
        self.cap = int(self.number(a, 'loop_kand', 24, 0, 256))
        self.radius = self.number(a, 'loop_radius', .8, .2, 3.)
        self.used = 0.; self.previous = None; self.stale = 0; self.cells = {}
        self.seen = set(); self.rounds = 0; self.accepted = {}; self.trials = 0
        self.accepted_moves = []
        self.disabled = None
        self.log = Path(a.get('loop_log', G['OUT'].replace('.json', '') + '_loop.jsonl'))
        self.exchange = Path(a.get('loop_zustand', G['OUT'].replace('.json', '') + '_engstellen'))
        if not G['KL']: self.disabled = 'loop braucht klassen='
        source = dict(G.get('ST') or {}, optionen=a)
        self.disabled = self.disabled or rest.reparatur_sperre(source)
        if any(source.get(k) for k in ('zones','zonen','r41_fuellung_lagen')):
            self.disabled = 'native Flaechen brauchen abgestimmten Kupferzustand'
        if G.get('PAAR_NETZE') or G.get('ZONENPLAN'):
            self.disabled = 'Paarnetze/Zonenplan brauchen abgestimmten Gruppenzug'
        self.input_path = a.get('loop_zuege')
        self.external = []
        if self.input_path:
            payload = json.loads(Path(self.input_path).read_text())
            if not isinstance(payload, list) or len(payload) > 256:
                raise ValueError('loop_zuege: JSON-Liste mit hoechstens 256 Zuegen erwartet')
            self.external = [self.validate_move(m) for m in payload]
        self.emit(dict(ereignis='start', standard_aus=True, grund=self.disabled,
                       budget_cpu_s=self.limit, argv=list(sys.argv)))

    @staticmethod
    def budget(G, fraction):


        total = G.get('CPU_MAX', 0)
        if total > 0: return min(86400., fraction * total)
        return min(86400., max(20., .05 * len(G['units']),
                              fraction * max(0., time.process_time() - G.get('_LOOP_START_CPU', time.process_time()))))

    @staticmethod
    def board(B):
        result = copy.deepcopy({k:v for k,v in B.items() if k != 'pads'})
        result['pads'] = [copy.deepcopy({k:v for k,v in p.items() if k not in ('cells','bcells')}) for p in B['pads']]
        return result

    @staticmethod
    def number(a, key, default, lo, hi):
        v = float(a.get(key, default))
        if not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('%s muss zwischen %s und %s liegen' % (key, lo, hi))
        return v

    @staticmethod
    def validate_move(m):
        if isinstance(m,dict) and m.get('art') in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import pruefe
            return pruefe(m)
        if not isinstance(m, dict) or m.get('art') not in ('via', 'teil', 'lage', 'umlegen'):
            raise ValueError('Unbekannte Loop-Zugart')
        fields = dict(via={'art','einheit','via','ziel','zustand'},
                      teil={'art','ref','delta','zustand'},
                      lage={'art','einheit','bahn','segment','lage','zustand'},
                      umlegen={'art','einheit','zustand'})
        if set(m) != fields[m['art']]: raise ValueError('Fehlende/unbekannte Zugfelder')
        if not isinstance(m['zustand'], str) or len(m['zustand']) != 64 or any(x not in '0123456789abcdef' for x in m['zustand']):
            raise ValueError('Zug braucht SHA-256 des Engstellenzustands')
        if m['art'] == 'teil':
            v = m['delta']
            if not isinstance(m['ref'],str) or not isinstance(v,list) or len(v)!=3:
                raise ValueError('Teilzug braucht ref und delta=[dx,dy,drehung]')
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in v): raise ValueError('Ungueltiges delta')
            if math.hypot(*v[:2]) > .500001 or v[2] not in (0,90,180,270): raise ValueError('Teilzug ausserhalb 0,5 mm/90 Grad')
        else:
            if not isinstance(m['einheit'],str): raise ValueError('einheit fehlt')
            for key in ('via','bahn','segment'):
                if key in m and (type(m[key]) is not int or m[key]<0): raise ValueError('Ungueltiger Index')
            if m['art']=='via' and (not isinstance(m['ziel'],list) or len(m['ziel'])!=2 or
                    any(type(x) not in (int,float) or not math.isfinite(x) for x in m['ziel'])): raise ValueError('Ungueltiges Viaziel')
            if m['art']=='lage' and not isinstance(m['lage'],str): raise ValueError('Ungueltige Lage')
        return copy.deepcopy(m)

    def emit(self, row):
        def finite(v):
            if isinstance(v,float) and not math.isfinite(v):return None
            if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
            if isinstance(v,(list,tuple)):return [finite(x) for x in v]
            return v
        row = dict(version=self.VERSION, quellen=self.quellen, **row)
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open('a') as f:
            f.write(json.dumps(finite(row), sort_keys=True, allow_nan=False) + '\n'); f.flush()

    def summary(self):
        return dict(version=self.VERSION, cpu_s=round(self.used,5), versuche=self.trials,
                    uebernommen=dict(self.accepted), grund=self.disabled,
                    abgelehnt=dict(self.rejected), vorgefiltert=dict(self.prefiltered),
                    budget_cpu_s=self.limit, zug_budget_cpu_s=self.move_cpu,
                    protokoll=str(self.log), engstellen=str(self.exchange),quellen=self.quellen)

    def out(self):
        G=self.G; c=G['exact_conflicts'](); ts,vs=G['ausgabe_geometrie']()
        return G['ausgabe_daten'](ts,vs,{u:len(v) for u,v in c.items()},roh=True), list(G['PAIRS'])

    def digest(self):
        import hashlib
        G=self.G
        io={'loop_zuege','loop_log','loop_zustand','etiketten','png','verlauf','klassen','regeln','start','fix','weiter','zwischenstand','agent_halt','agent_runde','agent_pakete','agent_zuege'}
        state=dict(board=self.board(G['D']), einheiten={u:{k:r[k] for k in ('net','tracks','vias')} for u,r in sorted(G['U'].items())}, failed=sorted(G['fail']),
                   quellen=self.quellen,lagen=G['LAYERS'],res=G['RES'],fest=sorted(G['FIXED']),optionen={k:v for k,v in G['args'].items() if k not in io})
        if G['args'].get('agent_loop') == '1':
            state['verhandlung']=dict(hist_sha256=hashlib.sha256(G['hist'].tobytes()).hexdigest(),
                praesenz=G['PF'],einheiten_reihenfolge=list(G['U']),
                vorgaben=G.get('_AGENT_VORGABEN',[]),prioritaet=G.get('_AGENT_PRIORITAET',[]))
        return hashlib.sha256(json.dumps(state,sort_keys=True,allow_nan=False).encode()).hexdigest()

    def clusters(self, pairs):


        from scipy.spatial import cKDTree
        if not pairs: return []
        xy=np.asarray([(p[4],p[5]) for p in pairs]);tree=cKDTree(xy)
        balls=tree.query_ball_point(xy,self.radius);remaining=set(range(len(pairs)));groups={}



        order=sorted(remaining,key=lambda i:(-len(balls[i]),pairs[i][4],pairs[i][5],pairs[i][0],pairs[i][2],pairs[i][3],pairs[i][1]))
        for i in order:
            if i not in remaining:continue
            members=sorted(set(balls[i])&remaining,key=lambda j:tuple(pairs[j]))
            remaining.difference_update(members);groups[i]=[pairs[j] for j in members]
        G=self.G; result=[]
        for ps in groups.values():
            points=sorted({(p[4],p[5]) for p in ps}); us=sorted({u for p in ps for u in p[2:4] if u in G['U']})
            xs,ys=zip(*points); bounds=[min(xs)-self.radius,min(ys)-self.radius,max(xs)+self.radius,max(ys)+self.radius]
            pads=[i for i,p in enumerate(G['D']['pads']) if any(math.dist((p['x'],p['y']),q)<=self.radius for q in points)]
            vias=[dict(einheit=u,via=i,netz=v.get('net') or r['net'],x=v['x'],y=v['y'],beteiligt=u in us,
                       gruppenschutz=any(v.get(k) for k in ('n','zu','gr','rv','strom_via','rueck_via')))
                  for u,r in sorted(G['U'].items()) for i,v in enumerate(r.get('vias',[]))
                  if any(math.dist((v['x'],v['y']),q)<=self.radius for q in points)]
            history=[]
            for p in ps:
                ix=int((p[4]-G['X0'])/G['RES']);iy=int((p[5]-G['Y0'])/G['RES'])
                if 0<=ix<G['NX'] and 0<=iy<G['NY']:
                    history.append(float(G['hist'].reshape(G['NL'],G['NY'],G['NX'])[:,iy,ix].max()))
            result.append(dict(paare=ps, punkte=points, einheiten=us, bbox=bounds,
                               netze=sorted({G['U'][u]['net'] for u in us}), pads=pads,
                               bauteile=sorted({G['D']['pads'][i]['ref'] for i in pads}),vias=vias,
                               historie=max(history,default=0)))
        return sorted(result,key=lambda c:(-len(c['paare']),-c['historie'],c['bbox']))

    def movable(self,u,via=False):
        G=self.G
        if u not in G['U'] or u.startswith('HEFT#'): return False
        if u in G['FIXED'] or u.startswith('FIX#'):

            return via and u.startswith('FIX#') and 'start' in G['args'] and not G['args'].get('fix') and G['U'][u]['net'] in G['FLNETZE']
        return True

    def neighbours(self,c):
        G=self.G
        if 'nachbarn' in c:
            return [u for u in c['nachbarn'] if u in G['U'] and self.movable(u)]
        return [u for u,r in G['U'].items() if self.movable(u) and G['_nah_an'](r,c['punkte'],self.radius)]

    def part_units(self,ref):
        G=self.G
        nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}-G['FLNETZE']
        touched={u for u,r in G['U'].items() if r['net'] in nets and self.movable(u)}
        if G.get('_AL') is not None:
            touched.update(u for u in G['units'] if u in nets and u not in G['FIXED'])


        touched.update('GND#%d'%i for i,p in enumerate(G['D']['pads'])
                       if p['ref']==ref and p['net'] in G['FLNETZE']
                       and 'GND#%d'%i in G['units'] and self.movable('GND#%d'%i))
        return touched

    def part_moves(self,c):
        ""
        import re
        from gemeinsam import wanderweite as WW
        G=self.G;B=G['D'];W=WW.tabelle(G['KL']);fps={f['ref']:f for f in B['fps']}
        raw=B.get('platzierungsgruppen') or []
        groups=raw.values() if isinstance(raw,dict) else raw
        grouped={r for g in groups if isinstance(g,dict) for r in g.get('refs',[])}
        if not c['punkte']:return
        for ref in sorted(fps,key=lambda r:(min((math.dist((fps[r]['x'],fps[r]['y']),p) for p in c['punkte']),default=math.inf),r)):
            f=fps[ref];t=G['KL']['teile'].get(ref,{})
            if (not re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp',''))
                    or ref in grouped or f.get('locked') or t.get('fest') or t.get('rolle')!='klein'
                    or ref not in W or W[ref]['max']<=0):continue
            nearest=min(c['punkte'],key=lambda p:math.dist((f['x'],f['y']),p))
            ax,ay=f['x']-nearest[0],f['y']-nearest[1]
            if math.hypot(ax,ay)>1.5:continue
            direction=(0,1 if ay>=0 else -1) if abs(ay)>=abs(ax) else (1 if ax>=0 else -1,0)
            preferred=[(round(direction[0]*d,5),round(direction[1]*d,5),0) for d in (.1,.2,.3,.4,.5)]
            rotations=[(0,0,th) for th in (90,180,270)] if W[ref]['drehen'] else []
            origin=(B.get('wander_basis') or {}).get(ref) or f
            if isinstance(origin,(list,tuple)):origin=dict(x=origin[0],y=origin[1])
            for dx,dy,th in preferred+rotations+[(x,y,0) for x,y in self.rest.offsets(.5) if (x,y,0) not in preferred]:
                if math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[ref]['max']+1e-8:continue
                yield dict(ref=ref,moves={ref:(dx,dy,th)})

    def external_local(self,m,c):
        G=self.G
        if m['art'] in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import lokal
            return lokal(G,m,c,self.radius)
        if m['art']=='teil':
            f=next((f for f in G['D']['fps'] if f['ref']==m['ref']),None)
            return f is not None and any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['punkte'])
        r=G['U'].get(m['einheit'])
        if r is None:return False
        try:
            if m['art']=='via':
                v=r['vias'][m['via']]
                return any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['punkte'])
            if m['art']=='lage':
                from shapely.geometry import LineString
                t=r['tracks'][m['bahn']];j=m['segment'];line=LineString(t['pts'][j:j+2])
                return any(line.distance(Point(p))<=self.radius for p in c['punkte'])
        except (IndexError,ValueError):return False
        return G['_nah_an'](r,c['punkte'],self.radius)

    def part_board(self,m,c):
        ""
        import re
        from gemeinsam import wanderweite as WW
        from gemeinsam.teile import lade_teile,platine,Belegung
        from gemeinsam.verschieben import bewegen
        from nachbearbeitung.zugregeln import sperre
        G=self.G;B=self.board(G['D']);r=m['ref'];dx,dy,th=m['delta']
        fps={f['ref']:f for f in B['fps']};f=fps.get(r);t=G['KL']['teile'].get(r,{})
        groups=B.get('platzierungsgruppen') or []
        if isinstance(groups,dict):groups=groups.values()
        grouped={ref for group in groups if isinstance(group,dict) for ref in group.get('refs',[])}
        W=WW.tabelle(G['KL'])
        chip=bool(G.get('_AL') is not None and f and re.search(r'(?:^|[_:])(?:SOT|SOIC)[-_]',f.get('fp',''),re.I))
        small=bool(f and re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp','')))
        if f is None or not (small or chip) or r in grouped or f.get('locked') or t.get('fest') or t.get('rolle') not in (('klein','chip') if chip else ('klein',)) or r not in W:
            raise ValueError('Teil ist kein freies C/R oder freigegebenes SOT/SOIC')
        if chip and not any(q['ref']==r for q in B.get('courtyards',[])):raise ValueError('SOT/SOIC braucht exportierten Courtyard')
        if not any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['punkte']):raise ValueError('Teil nicht lokal')
        if math.hypot(dx,dy)>.500001 or th not in (0,90,180,270) or (th and not (W[r]['drehen'] or chip)):raise ValueError('Teilzug ausserhalb Bewegungsvertrag')
        origin=(B.get('wander_basis') or {}).get(r) or next(q for q in self.initial_board['fps'] if q['ref']==r)
        if isinstance(origin,(tuple,list)):origin=dict(x=origin[0],y=origin[1])
        if W[r]['max']<=0 or math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[r]['max']+1e-8:raise ValueError('Wanderweite ueberschritten')
        B1,_=bewegen(B,{r:(dx,dy,th)});T=lade_teile(B1,.15);fps={f['ref']:f for f in B1['fps']};bel=Belegung()
        for ref in T:
            q=fps[ref]
            if ref!=r:bel.add(ref,T[ref].geo(q['x'],q['y'],0,q['side']))
        f=fps[r];geo=T[r].geo(f['x'],f['y'],0,f['side']);_,_,inside,_=platine(B,.2)
        if bel.kollision(geo) or not all(inside.contains(g) for g in geo.values()) or sperre(B1,geo,(r,)):raise ValueError('Courtyard/Padabstand/Rand/Keepout verletzt')
        return B1

    def via_move(self,owner,index,target):
        ""
        G=self.G;r=copy.deepcopy(G['U'][owner]);v=r['vias'][index];q=[v['x'],v['y']]
        for t in r['tracks']:
            ps=t['pts'];ws=t['w']
            if len(ps)<2:continue
            if math.dist(ps[0],q)<1e-6 and math.dist(q,ps[1])>.55:
                d=math.dist(q,ps[1]);a=[round(q[k]+.55/d*(ps[1][k]-q[k]),5) for k in (0,1)]
                t['pts']=[ps[0],a]+ps[1:];t['w']=[ws[0]]+ws
            elif math.dist(ps[-1],q)<1e-6 and math.dist(q,ps[-2])>.55:
                d=math.dist(q,ps[-2]);a=[round(q[k]+.55/d*(ps[-2][k]-q[k]),5) for k in (0,1)]
                t['pts']=ps[:-1]+[a,ps[-1]];t['w']=ws+[ws[-1]]
        return self.rest.via_zug(G['D'],r,index,target,G['VIA_D'],G['VIP'],owner.startswith('GND#') and int(owner[4:]) in G['EPM'],G['cls']('/__signal__')['wmin'])

    def candidates(self,c,digest):
        G=self.G; limit=max(1,self.cap//4); count=0


        vv=[]
        for u,r in sorted(G['U'].items()):
            if not self.movable(u,True): continue
            for i,v in enumerate(r['vias']):
                d=min(math.dist((v['x'],v['y']),p) for p in c['punkte'])
                if d<=self.radius: vv.append((u not in c['einheiten'],d,u,i))
        for _,_,u,i in sorted(vv):
            v=G['U'][u]['vias'][i]; targets=[]
            for step in (.1,.2,.3,.4,.5):
                for dx,dy in ((0,step),(0,-step),(step,0),(-step,0),(step,step),(-step,step),(step,-step),(-step,-step)):
                    if math.hypot(dx,dy)<=.500001:
                        x,y=v['x']+dx,v['y']+dy
                        targets.append((round(x,5),round(y,5)))
                        x=G['X0']+round((x-G['X0'])/G['RES'])*G['RES']; y=G['Y0']+round((y-G['Y0'])/G['RES'])*G['RES']
                        if .099999<=math.dist((v['x'],v['y']),(x,y))<=.500001: targets.append((round(x,5),round(y,5)))
            targets=sorted(set(targets),key=lambda p:(max(0., G['VIA_D']/2 + G['DRC_CLR'] - min(math.dist(p,q) for q in c['punkte'])),math.dist(p,(v['x'],v['y'])),p))
            for target in targets[:limit]:
                yield dict(art='via',einheit=u,via=i,ziel=list(target),zustand=digest)
                count+=1
                if count>=limit: break
            if count>=limit: break
        for i,p in enumerate(self.part_moves(c)):
            if i>=limit: break
            yield dict(art='teil',ref=p['ref'],delta=list(p['moves'][p['ref']]),zustand=digest)
        count=0
        for u in c['einheiten']:
            if not self.movable(u): continue
            for i,t in enumerate(G['U'][u]['tracks']):
                if any(t.get(k) for k in ('steg','strom_steg','kelvin','abg','rv')): continue
                for j,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                    from shapely.geometry import LineString
                    if not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['punkte']): continue
                    for layer in G['LAYERS']:
                        if layer==t['layer']: continue
                        yield dict(art='lage',einheit=u,bahn=i,segment=j,lage=layer,zustand=digest)
                        count+=1
                        if count>=limit: break
                    if count>=limit: break
                if count>=limit: break
            if count>=limit: break
        neighbours=sorted((u for u in G['U'] if self.movable(u) and G['_nah_an'](G['U'][u],c['punkte'],self.radius)),key=lambda u:(u in c['einheiten'],u))
        for u in neighbours[:limit]: yield dict(art='umlegen',einheit=u,zustand=digest)

    @staticmethod
    def pair_key(p): return (p[0],*sorted(p[2:4]),round(p[4],2),round(p[5],2))

    def pair_gate(self,p0,p1,c):
        from collections import Counter
        b=c['bbox']
        inside=lambda p:b[0]<=p[4]<=b[2] and b[1]<=p[5]<=b[3]
        before=[p for p in p0 if inside(p)]; after=[p for p in p1 if inside(p)]
        if len(after)>=len(before): return 'Paarzahl im Cluster sinkt nicht'
        old=Counter(self.pair_key(p) for p in p0 if not inside(p))
        new=Counter(self.pair_key(p) for p in p1 if not inside(p))
        if new-old: return 'neuer Konflikt ausserhalb des Clusters'

        distances={};after_distances={}
        for p in p0:
            if not inside(p):distances.setdefault(self.pair_key(p),[]).append(p[1])
        for p in p1:
            if not inside(p):after_distances.setdefault(self.pair_key(p),[]).append(p[1])
        for key,ds in after_distances.items():
            if any(n<o-1e-6 for o,n in zip(sorted(distances[key]),sorted(ds))):return 'Konflikt ausserhalb wird schwerer'
        return None

    def evaluate_gate(self,B0,R0,E0,R1,p0,p1,c):
        G=self.G; K=G['KL']; B1=G['D']
        reason=self.pair_gate(p0,p1,c)
        if reason: return reason,None
        if not set(R1['failed'])<=set(R0['failed']): return 'neue offene Einheit',None
        for check in (self.rest.harte_vias(B1,R0,R1),self.rest.harte_segmente(B1,R0,R1,diagnose=True),self.rest.teil_freiraum(B0,B1,R1)):
            if not check['ok']: return check['tor'],None
        from collections import Counter
        old_vias=Counter((v['net'],v['x'],v['y']) for v in R0['vias'])
        for v in R1['vias']:
            key=(v['net'],v['x'],v['y'])
            if old_vias[key]:old_vias[key]-=1;continue
            ix=int((v['x']-G['X0'])/G['RES']);iy=int((v['y']-G['Y0'])/G['RES'])
            if not (0<=ix<G['NX'] and 0<=iy<G['NY']) or not G['via_hart'](v['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
                return 'Via verletzt statische Rasterkarte (inklusive VIP)',None
        E1=self.bewerte(B1,R1,K,v3=True,minimax=False)
        if E1['offen']>E0['offen'] or not set(E1['unverbunden'])<=set(E0['unverbunden']): return 'Verbindungen schlechter',E1
        from analyse import regelkennzahlen as RK
        from nachbearbeitung.zugregeln import regelwerte, TOPO
        rg=RK.annahme(E0['regeln'],E1['regeln'])
        if not rg['nicht_schlechter']: return rg['grund'],E1
        if not math.isfinite(E1['note_v3']) or E1['note_v3']>E0['note_v3']+1e-6: return 'Note v3 schlechter',E1
        a=regelwerte(B0,R0,K);b=regelwerte(B1,R1,K)
        if any(b['topologie'][k]>a['topologie'][k]+1e-6 for k in TOPO) or b['kopplung']>a['kopplung']+1e-6: return 'Topologie/Kopplung schlechter',E1
        protected=set(K['gnd'])|{n for n,k in K['netze'].items() if k.get('art') in ('strom','versorgung','sw')}
        a=Counter(v['net'] for v in R0['vias'] if v['net'] in protected);b=Counter(v['net'] for v in R1['vias'])
        if a-b: return 'geschuetzte Via-Zahl sinkt',E1
        a=self.rest.massepad_zahlen(B0,R0,K);b=self.rest.massepad_zahlen(B1,R1,K)
        if any(b.get(p,0)<n for p,n in a.items()): return 'R16 Massepad-Via fehlt',E1


        from analyse.bewertung import strompfade,strom_verlust_gleich
        s0=strompfade(B0,R0,K,E0,dc=None);s1=strompfade(B1,R1,K,E1,dc=None)
        p0w,p1w=strom_verlust_gleich(E0,E1,B0,R0,B1,R1,K)
        if (s1.get('umweg_mm') or 0)-(s0.get('umweg_mm') or 0)>1. and p1w>p0w*1.02+1e-9: return 'Strompfade schlechter',E1
        return None,E1

    def static_preflight(self,R0,R1):
        ""
        v=self.rest.harte_vias(self.G['D'],R0,R1)
        if not v['ok'] and v['tor']!='Via-Bohrung-Bohrung':return v['tor']
        s=self.rest.harte_segmente(self.G['D'],R0,R1,diagnose=True)
        return None if s['ok'] else s['tor']

    def prefilter(self,m,c):
        ""
        G=self.G
        if m['art']=='teil':
            projected=self.part_units(m['ref'])|set(self.neighbours(c))
            limit=min(int(G['args'].get('loop_gruppe','24')),int(G['args'].get('loop_neu_max','4' if self.large else '24')))
            return 'Teilgruppe schon vor Placement-Pruefung zu gross' if len(projected)>limit else None
        if m['art'] != 'via': return None
        u=m['einheit']
        if not self.movable(u,True): return 'festes/fehlendes Via'
        if m['via'] >= len(G['U'][u]['vias']): return 'fehlendes Via'
        x,y=m['ziel'];v=G['U'][u]['vias'][m['via']]
        if not .099999<=math.hypot(x-v['x'],y-v['y'])<=.500001: return 'Viaweg ausserhalb Vertrag'
        ix=int((x-G['X0'])/G['RES']);iy=int((y-G['Y0'])/G['RES'])
        if not (0<=ix<G['NX'] and 0<=iy<G['NY']): return 'Viaziel ausserhalb Raster'

        if not G['via_hart'](G['U'][u]['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
            return 'Viaziel statisch gesperrt'
        if self.via_move(u,m['via'],m['ziel']) is None: return 'Via-Anbindung/Gruppenschutz'
        return None

    def rebuild(self,B):
        if self.G.get('_AL') is not None:
            changed=[i for i,(a,b) in enumerate(zip(self.board(self.G['D'])['pads'],B['pads'])) if a!=b]
            if len(changed)>2:return self.rebuild_full(B)
        if self.G['args'].get('loop_karten','lokal') != 'voll':
            return self.rebuild_local(B)
        return self.rebuild_full(B)

    def rebuild_local(self,B):
        G=self.G;original=G.copy()
        changed,window=G['loop_pad_patch'](B)


        changed=set(changed);idx={li:0 for li in range(G['NL'])}
        for key in ('_PADG','_PADXY','_PADB'):G[key]={li:list(rows) for li,rows in G[key].items()}
        for k,p in enumerate(G['D']['pads']):
            for L in p['layers']:
                if L not in G['LI']:continue
                li=G['LI'][L];j=idx[li];idx[li]+=1
                if k not in changed:continue
                pg=G['shapely'].union_all([G['Polygon'](q) for q in p['polys'][L]])
                pn=p['net'] if p['net'] and not p['net'].startswith('unconnected') else '#%d'%id(p)
                G['_PADG'][li][j]=(pn,pg);G['_PADXY'][li][j]=(p['x'],p['y'])
                if G['DRCL']:pg=G['pad_huelle'](p,L,G['DRCP'],True)
                G['_PADB'][li][j]=(pn,pg,pg.buffer(G['_DMAX']))
        G['_PADT']={li:G['shapely'].STRtree([g for _,g in rows]) for li,rows in G['_PADG'].items()}

        rn=G['VIA_KEEP_R']+max(G['DRCP'],.03)+.03
        VP=G['VP'].copy()
        for cell,(x,y,near) in VP.items():
            if not any(p['bbox'][0]-rn<=x<=p['bbox'][2]+rn and p['bbox'][1]-rn<=y<=p['bbox'][3]+rn
                       for k in changed for p in (original['D']['pads'][k],G['D']['pads'][k])):continue
            pt=Point(x,y);near=[]
            for li in range(G['NL']):
                for j in G['_PADT'][li].query(pt.buffer(rn)):
                    n,g=G['_PADG'][li][int(j)];d=float(pt.distance(g))
                    if d<rn:near.append((n,round(d,4),li in (0,G['NL']-1)))
            VP[cell]=(x,y,tuple(sorted(near)))
        G['VP']=VP
        for key in ('_HM','_ERR','HALS_CACHE','_VAPZ','_NAHE_C','_ABST_C'):
            if key in G and isinstance(G[key],dict):G[key]={}
        G['_pad_room_cached'].cache_clear()
        G['KETTEN']={};G['KF_NACH']={};G['KF_VOR']={}
        if G['KETTE']:G['_ketten_bauen']()
        G['_KIDX']=[None];G['ASTAR_COST']=[None]
        self.last_window=list(window)
        return original

    def rebuild_full(self,B):



        import tempfile,contextlib,io
        G=self.G; original=G.copy(); argv=list(sys.argv)
        code=Path(G['__file__']).read_text().split('# Dichte Etiketten sind ein passiver Beobachter;',1)[0]


        code=code.replace('\nbootstrap(heavy=True)\n','\n# bestehender Prozessschutz bleibt erhalten\n',1)
        with tempfile.TemporaryDirectory(prefix='routing_loop_') as td:
            payload=self.board(B)


            payload['pads']=payload['pads'][:original['_LOOP_SOURCE_PAD_COUNT']]
            path=Path(td)/'board.json';path.write_text(json.dumps(payload))
            opt={k:v for k,v in G['args'].items() if k not in ('start','neu','neu_netze','hist_start','hist_context','agent_loop','weiter','agent_zuege')}
            try:
                sys.argv=[argv[0],str(path),original['OUT']]+[k+'='+str(v) for k,v in opt.items()]
                with contextlib.redirect_stdout(io.StringIO()): exec(compile(code,G['__file__'],'exec'),G)
                for u,r in original['U'].items(): G['add_unit'](u,r['net'],copy.deepcopy(r['tracks']),copy.deepcopy(r['vias']))

                for key in ('args','t_start','PF','VIA_ALT','VG_WEICH','VIA_ANKER_AN','QUELLFREI','HART_FREMD','_REST_BOARD_SOURCE','_REST_START_SOURCE'):
                    if key in original: G[key]=copy.deepcopy(original[key])
                G['hist'][:]=original['hist']; G['fail']=set(original['fail']); G['_KIDX'][0]=None
                for key in original:
                    if key.startswith('_AGENT_'):G[key]=original[key]
                if G.get('_AL') is not None:G['_AL'].install_router()
            finally: sys.argv=argv
        return original
    def apply(self,m,c):
        G=self.G; art=m['art']; owner=m.get('einheit'); touched=[]
        if art in ('fenster_lage','kugel_via','prioritaet'):
            from codex.agent_loop.zuege import anwenden
            return anwenden(self,m,c)
        if art=='teil':
            marker=self.rest.placement_marker(self.initial_board,self.part_board(m,c))
            self.rebuild(marker['rest_board']); G=self.G
            ref=m['ref']; nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}
            touched=list(self.part_units(ref))


            for u,r in list(G['U'].items()):
                if not u.startswith('FIX#') or r['net'] not in nets: continue
                if G['_nah_an'](r,c['punkte'],self.radius): raise ValueError('anonymer Flaechenstich braucht gemeinsamen Zug')
        elif art=='via':
            if not self.movable(owner,True): raise ValueError('festes/fehlendes Via')
            r=G['U'][owner];v=r['vias'][m['via']]
            if not .099999<=math.dist((v['x'],v['y']),m['ziel'])<=.500001: raise ValueError('Viaweg ausserhalb 0,1 bis 0,5 mm')
            if not any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['punkte']): raise ValueError('Via nicht lokal')
            r1=self.via_move(owner,m['via'],m['ziel'])
            if r1 is None: raise ValueError('Via-Anbindung/Gruppenschutz')
            G['del_unit'](owner);G['add_unit'](owner,r1['net'],r1['tracks'],r1['vias'])
        elif art=='lage':
            if not self.movable(owner) or m['lage'] not in G['LAYERS']: raise ValueError('gesperrte Einheit/Lage')
            r=copy.deepcopy(G['U'][owner]);i=m['bahn'];j=m['segment'];t=r['tracks'][i]
            if any(t.get(k) for k in ('steg','strom_steg','kelvin','abg','rv')): raise ValueError('geschuetzte Bahn')
            a,b=t['pts'][j:j+2];w=t['w'][j]
            from shapely.geometry import LineString
            if t['layer']==m['lage'] or not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['punkte']): raise ValueError('Lagenwechsel nicht lokal')
            if math.dist(a,b)<G['DRC_HOLE']: raise ValueError('Via-Paar zu eng')
            replacement=[]
            if j: replacement.append(dict(t,pts=t['pts'][:j+1],w=t['w'][:j]))
            replacement.append(dict(t,layer=m['lage'],pts=[a,b],w=[w]))
            if j+1<len(t['w']): replacement.append(dict(t,pts=t['pts'][j+1:],w=t['w'][j+1:]))
            r['tracks'][i:i+1]=replacement
            for x,y in (a,b):
                if not any(math.dist((v['x'],v['y']),(x,y))<1e-6 for v in r['vias']):r['vias'].append(dict(x=x,y=y))
            G['del_unit'](owner);G['add_unit'](owner,r['net'],r['tracks'],r['vias'])
        else:
            if not self.movable(owner) or not G['_nah_an'](G['U'][owner],c['punkte'],self.radius): raise ValueError('Blockierer nicht beweglich/lokal')
            touched=[owner]

        touched+=[u for u in self.neighbours(c) if u!=owner]
        touched=sorted(set(touched),key=lambda u:(u!=m.get('einheit'),G['grp'](u),u))
        if len(touched)>int(G['args'].get('loop_gruppe','24')): raise ValueError('lokale Gruppe zu gross')
        return touched

    def trial(self,m,c,R0,p0,E0,deadline):
        start=time.process_time()
        G=self.G; original=G.copy(); B0=G['D']; order=list(G['U'])


        saved=G['U'].copy()
        hs=G['hist'].copy();vs=copy.deepcopy(G['VIA_ALT']);fails=set(G['fail'])
        flags={k:copy.deepcopy(G[k]) for k in ('PF','VG_WEICH','HART_FREMD','QUELLFREI','VIA_ANKER_AN')}
        semantic={k:copy.deepcopy(G[k]) for k in ('_KZ_AUS','_ERR_NETZE','UNERREICHBAR','STOSS_STAT','KETTE_STAT','RV_STAT','TOPO_STAT','STROM_AUDIT','_AGENT_VORGABEN','_AGENT_PRIORITAET') if k in G}
        ok=False;E1=None;R1=None;p1=None;reason=None;phases={'sichern':time.process_time()-start}
        old_deadline=G.get('_LOOP_DEADLINE')
        stamp=[time.process_time()]
        def phase(name):
            now=time.process_time();phases[name]=now-stamp[0];stamp[0]=now
        try:
            touched=self.apply(m,c)
            phase('anwenden')
            if G['args'].get('loop_suche','begrenzt')=='begrenzt':
                G['_LOOP_DEADLINE']=deadline-min(.5,max(0.,deadline-start)*.2)
            G['VG_WEICH'][0]=False;G['HART_FREMD'][0]=True
            R1,p1=self.out()
            phase('konflikte')
            if m['art'] in ('via','lage','fenster_lage','kugel_via'):
                blocked=self.static_preflight(R0,R1)
                if blocked:raise ValueError('statische Vorpruefung: '+blocked)
            phase('statisch')
            pair_reason=self.pair_gate(p0,p1,c)
            if m['art'] in ('via','lage','fenster_lage','kugel_via') and pair_reason and 'ausserhalb' in pair_reason:
                raise ValueError(pair_reason)

            if m['art'] in ('teil','prioritaet') or pair_reason is not None:
                if len(touched)>int(G['args'].get('loop_neu_max','4' if self.large else '24')):
                    raise ValueError('zu viele Einheiten fuer billigen Neuverlegungszug')
                for u in touched:G['del_unit'](u)
                for u in touched:
                    if time.process_time()>=deadline or G['zeit_knapp'](): raise TimeoutError('Loop-CPU-Budget')
                    if G['route_unit'](u):G['fail'].discard(u)
                    else:G['fail'].add(u)
                R1,p1=self.out()
            phase('neuverlegen')
            if time.process_time()>=deadline or G['zeit_knapp'](): raise TimeoutError('Loop-CPU-Budget')
            reason,E1=self.evaluate_gate(B0,R0,E0,R1,p0,p1,c)
            phase('tore')
            ok=reason is None
            if time.process_time()>=deadline:ok=False;reason='Loop-CPU-Budget nach Pruefung'
        except Exception as e:
            reason=type(e).__name__+': '+str(e)[:220]
        finally:
            phase('arbeit_bis_abbruch_oder_abschluss')
            G['_LOOP_DEADLINE']=old_deadline
            if not ok:
                for u in list(G['U']):
                    if G['U'][u] is not saved.get(u):G['del_unit'](u)
                for u in order:
                    if u not in G['U']:
                        r=saved[u];G['add_unit'](u,r['net'],r['tracks'],r['vias'])
                if G['D'] is not B0:


                    dynamic={k:G[k] for k in ('U','use','drill_use','SN','SN_OFFEN','UOWN','NETU','DDN','_NVER','_GEO')}
                    G.clear();G.update(original);G.update(dynamic)
                G['fail'].clear();G['fail'].update(fails)
                G['VIA_ALT'].clear();G['VIA_ALT'].update(vs)
                for k,v in semantic.items():G[k]=v
            G['hist'][:]=hs
            for k,v in flags.items():G[k][:]=v
            G['_KIDX'][0]=None

            current=G['U'].copy();G['U'].clear()
            for u in order:
                if u in current:G['U'][u]=current.pop(u)
            for u in sorted(current):G['U'][u]=current[u]
            G['exact_conflicts']()
            G['_LOOP_DEADLINE']=old_deadline
            phase('ruecknahme_oder_abschluss')
        self.trials+=1
        if not ok:self.rejected[m['art']]=self.rejected.get(m['art'],0)+1
        self.emit(dict(ereignis='zug',runde=self.rounds,engstelle=c,zug=m,angenommen=ok,
                       grund=reason or 'Cluster verbessert, volle Tore gehalten',cpu_s=round(time.process_time()-start,6),
                       cpu_phasen_s=phases, zug_budget_cpu_s=max(0.,deadline-start),
                       paare_vorher=p0,paare_nachher=p1,
                       vorher={k:E0.get(k) for k in ('gueltig','note_v3','konflikte','offen','regeln')},
                       nachher=None if E1 is None else {k:E1.get(k) for k in ('gueltig','note_v3','konflikte','offen','regeln')}))
        if ok:
            self.accepted[m['art']]=self.accepted.get(m['art'],0)+1
            self.accepted_moves.append(copy.deepcopy(m))
        return ok

    def export(self,c,digest,quick=False):
        d=self.exchange/('%04d_%s'%(self.rounds,digest[:12]));d.mkdir(parents=True,exist_ok=True)
        G=self.G;b=c['bbox'];margin=.5;b=[b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin]
        scale=min(160,1000/max(b[2]-b[0],b[3]-b[1]));size=(max(1,int((b[2]-b[0])*scale)),max(1,int((b[3]-b[1])*scale)))
        im=Image.new('RGB',size,'white');draw=ImageDraw.Draw(im)
        def p(q):return ((q[0]-b[0])*scale,(q[1]-b[1])*scale)
        for u,r in sorted(G['U'].items()):
            for t in r['tracks']:draw.line([p(q) for q in t['pts']],fill=('#315e8e' if u in c['einheiten'] else '#b3bdc7'),width=max(1,int(min(t['w'],default=.1)*scale)))
            for v in r['vias']:
                x,y=p((v['x'],v['y']));rr=G['VIA_D']/2*scale;draw.ellipse((x-rr,y-rr,x+rr,y+rr),outline='#245c40',width=2)
        for i in c['pads']:
            for qs in G['D']['pads'][i]['polys'].values():
                for q in qs:draw.polygon([p(v) for v in q],outline='#9b7300')
        for q in c['punkte']:
            x,y=p(q);draw.ellipse((x-6,y-6,x+6,y+6),outline='red',width=3)
        im.save(d/'konflikte.png')


        places={}; previous=G['HART_FREMD'][0];G['HART_FREMD'][0]=True
        try:
            ix0=max(0,int((b[0]-G['X0'])/G['RES']));ix1=min(G['NX'],int((b[2]-G['X0'])/G['RES']))
            iy0=max(0,int((b[1]-G['Y0'])/G['RES']));iy1=min(G['NY'],int((b[3]-G['Y0'])/G['RES']))
            step=max(1,int(.2/G['RES']));cells=np.asarray([iy*G['NX']+ix for iy in range(iy0,iy1,step) for ix in range(ix0,ix1,step)],dtype=np.int64)
            for net in (() if quick else c['netze']):

                allowed=G['hard_maps'](net)[1].ravel()
                probe=[k for k in cells[:256] if allowed[k]]
                points=[(G['X0']+(int(k)%G['NX']+.5)*G['RES'],G['Y0']+(int(k)//G['NX']+.5)*G['RES']) for k in probe]
                foreign=[geo for u,r in G['U'].items() for li,geo,_,ne in G['_unit_geo'](u,r) if ne!=net]
                holes=[Point(v['x'],v['y']) for r in G['U'].values() for v in r['vias']]
                copper=G['shapely'].union_all(foreign);drills=G['shapely'].union_all(holes)
                places[net]=[[round(x,5),round(y,5)] for x,y in points if (copper.is_empty or Point(x,y).distance(copper)>=G['VIA_D']/2+G['DRC_CLR']) and (drills.is_empty or Point(x,y).distance(drills)>=G['DRC_HOLE'])]
        finally:G['HART_FREMD'][0]=previous
        clean=self.board(G['D'])
        state=dict(version=self.VERSION,zustand=digest,engstelle=c,lagen=G['LAYERS'],
                   raster=dict(res=G['RES'],ursprung=[G['X0'],G['Y0']]),freie_via_rasterpunkte=places,
                   freie_via_rasterpunkte_geprueft=not quick,
                   bauteile=[f for f in G['D']['fps'] if f['ref'] in c['bauteile']],pads=[clean['pads'][i] for i in c['pads']],
                   bisherige_zuege=copy.deepcopy(self.accepted_moves),
                   einheiten={u:{k:G['U'][u][k] for k in ('net','tracks','vias')} for u in sorted(set(c['einheiten'])|set(c.get('nachbarn',[]))|{v['einheit'] for v in c.get('vias',[])})},
                   zugformat='Liste: art via/teil/lage/umlegen; zustand bindet exakt diesen Snapshot')
        (d/'zustand.json').write_text(json.dumps(state,sort_keys=True,indent=1))
        try:
            with (d/'zuege.json').open('x') as f:
                f.write(json.dumps(self.accepted_moves,sort_keys=True,indent=1)+'\n')
        except FileExistsError:
            pass
        self.emit(dict(ereignis='ungeloest',runde=self.rounds,zustand=digest,ordner=str(d)))

    def trigger(self,pairs):
        count=len(pairs);self.stale=self.stale+1 if count==self.previous else 1;self.previous=count
        self.counts=(self.counts+[count])[-self.k:]
        cells={(p[0],round(p[4]/self.G['RES']),round(p[5]/self.G['RES'])) for p in pairs}
        self.cells={k:self.cells.get(k,0)+1 for k in cells}


        if self.large and (count>self.threshold or len(self.counts)<self.k or min(self.counts)<self.counts[0]):return False
        return bool(count and (self.stale>=self.k or max(self.cells.values(),default=0)>=self.n))

    def tick(self,round_number,pairs=None):
        if not self.explicit_budget:self.limit=self.budget(self.G,self.fraction)
        if self.disabled or self.used>=self.limit or self.trials>=self.cap or self.G['zeit_knapp']():return False
        t=time.process_time();G=self.G;self.rounds=round_number;phases={};triggered=False
        def timed(name,start):phases[name]=phases.get(name,0.)+time.process_time()-start
        try:
            start=time.process_time()
            if pairs is None:
                G['exact_conflicts']();pairs=list(G['PAIRS'])
            p0=pairs
            if not self.trigger(p0):return False
            triggered=True;timed('beobachten',start)
            deadline=t+max(0,self.limit-self.used)
            clusters=[];skipped=None;start=time.process_time()
            for c in self.clusters(p0):
                neighbours=[u for u,r in G['U'].items() if self.movable(u) and G['_nah_an'](r,c['punkte'],self.radius)]
                c['nachbarn']=sorted(neighbours)
                if len(neighbours)>int(G['args'].get('loop_gruppe','24'))+1:
                    self.emit(dict(ereignis='cluster_ausgelassen',runde=round_number,grund='lokale Gruppe schon vor Zug zu gross',gruppe_n=len(neighbours),engstelle=c));skipped=skipped or c;continue
                clusters.append(c)
                if len(clusters)>=3:break
            timed('cluster',start)
            if not clusters:
                if skipped:self.export(skipped,self.digest(),quick=True)
                return False
            start=time.process_time();R0,p0=self.out();timed('ausgabe',start)
            start=time.process_time();E0=self.bewerte(G['D'],R0,G['KL'],v3=True,minimax=False);timed('ausgangsbewertung',start)
            if not math.isfinite(E0.get('note_v3',math.inf)):
                self.disabled='unvollstaendige Ausgangsbewertung';self.emit(dict(ereignis='gesperrt',grund=self.disabled));return False
            start=time.process_time();digest=self.digest();timed('zustand_hash',start)
            for c in clusters:
                candidates=[m for m in self.external if m['zustand']==digest] if self.input_path else self.candidates(c,digest)
                iterator=iter(candidates)
                while time.process_time()<deadline and self.trials<self.cap:


                    start=time.process_time()
                    try:m=next(iterator)
                    except StopIteration:
                        timed('vorschlagsbau',start);break
                    timed('vorschlagsbau',start)
                    if time.process_time()>=deadline:break
                    if self.input_path and not self.external_local(m,c):continue
                    key=json.dumps(m,sort_keys=True)
                    if key in self.seen:continue
                    self.seen.add(key)
                    if m['zustand']!=digest:
                        self.emit(dict(ereignis='abgelehnt',grund='veralteter Zustand',zug=m));continue
                    start=time.process_time();blocked=self.prefilter(m,c);timed('vorfilter',start)
                    if blocked:
                        self.prefiltered[m['art']]=self.prefiltered.get(m['art'],0)+1
                        self.emit(dict(ereignis='vorgefiltert',zug=m,grund=blocked));continue
                    start=time.process_time();ok=self.trial(m,c,R0,p0,E0,min(deadline,time.process_time()+self.move_cpu));timed('zuege',start)
                    if ok:
                        self.stale=0;self.cells.clear();return True
                start=time.process_time();self.export(c,digest,quick=time.process_time()>=deadline or G['zeit_knapp']());timed('schnittstelle',start)
            for m in self.external:
                if json.dumps(m,sort_keys=True) not in self.seen:
                    self.emit(dict(ereignis='wartend',grund='Zustand noch nicht erreicht oder kein lokales Ziel in geprueften Clustern/Budget',zug=m))
            return False
        finally:
            elapsed=time.process_time()-t;self.used+=elapsed
            self.emit(dict(ereignis='loop_runde',runde=round_number,ausgeloest=triggered,cpu_s=elapsed,cpu_phasen_s=phases,budget_cpu_s=self.limit))


_RL = _RoutingLoop(globals()) if args.get('loop') == '1' or args.get('agent_loop') == '1' else None





ENDSPIEL = args.get('endspiel', '1' if KL else '0') == '1'



WMIN_KONF = args.get('wmin_konf', '1' if KL else '0') == '1'
WMIN_BIS = int(args.get('wmin_konf_bis', str(2 * MAXIT // 3)))
ALLE_NAH = float(args.get('alle_nah', '0'))
KONVERGENZ_LOKAL = args.get('konvergenz', '') == 'lokal'
_lokal_best = [len(units), 0]
END_MAX = int(args.get('endspiel_max', '8'))
_es = dict(n=0, vor=None, stufe=0, ab=0, lief=False, cpu=0.0, aus=False)
END_ZEIT = float(args.get('endspiel_zeit', '1.0'))


def lokale_suchmaske(paare, offene, radius):
    ""
    m = np.zeros((NY, NX), dtype=bool)
    def region(x1, y1, x2, y2, rand):
        xa = max(0, min(NX, int(math.floor((x1 - rand - X0) / RES))))
        xb = max(0, min(NX, int(math.ceil((x2 + rand - X0) / RES)) + 1))
        ya = max(0, min(NY, int(math.floor((y1 - rand - Y0) / RES))))
        yb = max(0, min(NY, int(math.ceil((y2 + rand - Y0) / RES)) + 1))
        m[ya:yb, xa:xb] = True
    for (_l, _d, _a, _b, x, y) in paare: region(x, y, x, y, radius)
    for u in sorted(offene):
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        boxes = [D['pads'][k]['bbox'] for k in ks]
        if not boxes: continue

        region(min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes),
               6.1 + radius if u.startswith('GND#') else (FENSTER or 3.0) + radius)
    return m


def _nah_an(r, stellen, R):
    for v in r['vias']:
        if any(math.hypot(v['x'] - x, v['y'] - y) <= R for x, y in stellen): return True
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            for x, y in stellen:
                s = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
                if math.hypot(a[0] + s * dx - x, a[1] + s * dy - y) <= R: return True
    return False


def endspiel(conf, paare, stufe, saat, reparatur=False):
    import copy
    R = (1.0, 2.0, 3.5, 5.0)[stufe % 4]
    stellen = [(x, y) for (_l, _d, _a, _b, x, y) in paare]
    kern = sorted(u for u in conf if u in U and not u.startswith(('FIX#', 'HEFT#')))
    gruppe = list(kern)
    for w, r in U.items():
        if w in kern or w.startswith(('FIX#', 'HEFT#')): continue
        if _nah_an(r, stellen, R): gruppe.append(w)
    if PAAR_NETZE: gruppe = sorted(set(gruppe) | {PAAR_NETZE[w] for w in gruppe if w in PAAR_NETZE and PAAR_NETZE[w] in U and PAAR_NETZE[w] not in FIXED})
    if len(gruppe) == len(kern) or len(gruppe) > int(args.get('endspiel_gruppe', '40')):
        print('   Endspiel: Umkreis %.1f mm, Gruppe %d bei %d Konfliktnetzen (%s)' % (R, len(gruppe), len(kern), 'niemand im Weg' if len(gruppe) == len(kern) else 'zu gross'), flush=True)
        return False
    save = {w: copy.deepcopy(U[w]) for w in gruppe}; hsave = hist.copy(); PFs = PF[0]; weich = VG_WEICH[0]; VG_WEICH[0] = False
    quellfrei_vor = QUELLFREI[0]


    X = gruppen_index(gruppe)
    frei = False


    for versuch in range(int(args.get('endspiel_versuche', '3'))):
        _end_used = _es.get('arbeit', 0) if BUDGET_ARBEIT else _es['cpu']
        _end_total = arbeit_gesamt() if BUDGET_ARBEIT else such_cpu() - beobachter_cpu()
        if _end_used > max(30.0, END_ZEIT * (_end_total - _end_used)) or zeit_knapp():
            if BUDGET_ARBEIT: print('   Endspiel: Arbeitsanteil verbraucht (%d Auftraege)' % _end_used, flush=True)
            else: print('   Endspiel: Zeitanteil verbraucht (%.0f s)' % _es['cpu'], flush=True)
            _es['aus'] = True; break
        rng_ = budget_rng(1000 + 10 * saat + versuch); _tc = time.process_time()
        _end_work = arbeit_gesamt()
        _etik_cpu_vor = beobachter_cpu()
        QUELLFREI[0] = reparatur or quellfrei_vor
        HART_FREMD[0] = reparatur and versuch == int(args.get('endspiel_versuche', '3')) - 1
        for w in gruppe:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if versuch == 0: order = kern + sorted((w for w in gruppe if w not in kern), key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else (grp(w)[0], rng_.rand()))
        else: order = sorted(gruppe, key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else (grp(w)[0] if versuch % 2 else 0, rng_.rand()))
        if PAAR_NETZE:
            positions = {w:i for i,w in enumerate(order)}
            order.sort(key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('paar_rang',0), min(w,PAAR_NETZE[w]) if w in PAAR_NETZE else '', w if w in PAAR_NETZE else positions[w]))
        okk = True; verlauf = []; _ab = False
        for w in paar_reihe(order):
            if zeit_knapp(): _ab = True; break
            if not route_unit(w): okk = False
        for _k in range(int(args.get('endspiel_iter', '14'))):
            if not okk or _ab: break
            cfg = konflikte_gruppe(gruppe, X); verlauf.append(len(cfg))
            if _ET: _ET.beobachten(cfg, it if 'it' in globals() else 1, 'endspiel', geometrie=False)
            if args.get('endspiel_pruef') == '1':
                _v = exact_conflicts(); assert set(_v) == set(cfg), ('Endspiel-Pruefung', sorted(set(_v) ^ set(cfg)))
            if not cfg: frei = True; break
            if len(verlauf) >= 5 and min(verlauf[-4:]) >= verlauf[-5]: break
            for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
            PF[0] *= 1.5
            if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
            for w in paar_reihe(sorted((w for w in order if w in cfg), key=lambda w: (grp(w)[0], -(KL['netze'].get(w) or {}).get('rang', 0), rng_.rand()) if KL and args.get('rang_alle') == '1' else rng_.rand())):
                if zeit_knapp(): _ab = True; break
                if not paar_einheit(w): del_unit(w)
                if not route_unit(w): okk = False
        print('   Endspiel: Umkreis %.1f mm, Gruppe %d (davon GND-Stiche %d), Anlauf %d, Konflikte je Runde %s%s' % (
            R, len(gruppe), sum(1 for w in gruppe if w.startswith('GND#')), versuch + 1, verlauf, '' if okk else ', unvollstaendig'), flush=True)
        HART_FREMD[0] = False
        QUELLFREI[0] = quellfrei_vor
        _es['cpu'] += time.process_time() - _tc - (beobachter_cpu() - _etik_cpu_vor)
        if BUDGET_ARBEIT: _es['arbeit'] = _es.get('arbeit', 0) + arbeit_gesamt() - _end_work
        if _ab:
            print('   Endspiel: Rechenzeit-Budget verbraucht, Anlauf abgebrochen', flush=True); _es['aus'] = True; break
        if frei: break
    hist[:] = hsave; PF[0] = PFs; VG_WEICH[0] = weich
    if frei: return True
    for w in gruppe:
        if w in U: del_unit(w)
    for w in gruppe: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    return False









ZUG = args.get('zug', '0') == '1'
ZUG_MAX = int(args.get('zug_max', '80'))
ZUG_STELLEN = int(args.get('zug_stellen', '10'))
ZUG_GRUPPE = int(args.get('zug_gruppe', '25'))
ZUG_ANTEIL = float(args.get('zug_anteil', '0.25'))
ZUG_ITER = int(args.get('zug_iter', '10'))



ZUG_STRAT = (dict(R=2.0, reihe='kern', warm=True), dict(R=3.5, reihe='kern', warm=True), dict(R=1.0, reihe='kern'),
             dict(R=2.0, reihe='kern'), dict(R=2.0, reihe='zufall'), dict(R=3.5, reihe='kern'),
             dict(R=2.0, reihe='kern', reparatur=True), dict(R=5.0, reihe='kern'))
ZUG_WARM_PF = float(args.get('zug_warm_pf', '50'))
_zs = dict(n=0, ok=0, cpu=0.0, arbeit=0, ruhig=0, best=None, log=None)

ZUG_POLITIK = json.load(open(args['zug_politik']))['tabelle'] if args.get('zug_politik') else None
def _zug_pads2(x, y):
    return sum(1 for p in D['pads'] if abs(p['x'] - x) <= 2.0 and abs(p['y'] - y) <= 2.0)

def _zug_reihe(stelle):
    if not ZUG_POLITIK: return list(enumerate(ZUG_STRAT))
    k = len(stelle['kern']); p = _zug_pads2(stelle['x'], stelle['y'])
    lage = '%s|k%d|p%d' % (stelle['typ'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)
    tab = ZUG_POLITIK.get(lage) or ZUG_POLITIK.get(stelle['typ']) or {}
    def wert(js):
        s = js[1]; e = tab.get('%s|%s|%d|%d' % (s['R'], s['reihe'], int(bool(s.get('reparatur'))), int(bool(s.get('warm')))))
        return (-(e['wert'] if e else -1.0), js[0])
    return sorted(enumerate(ZUG_STRAT), key=wert)


def _zug_log(row):
    if _zs['log'] is None:
        _zs['log'] = open(args.get('zug_log', OUT.replace('.json', '') + '_zuege.jsonl'), 'w')
    _zs['log'].write(json.dumps(row, sort_keys=True) + '\n'); _zs['log'].flush()


def _zug_stellen(paare, offene):
    ""
    st = []
    for (_l, _d, a, b, x, y) in sorted(paare, key=lambda p: (round(p[4], 3), round(p[5], 3), p[2], p[3])):
        kern = {w for w in (a, b) if w in U and not w.startswith(('FIX#', 'HEFT#'))}
        if not kern: continue
        for s in st:
            if s['typ'] == 'konflikt' and math.hypot(s['x'] - x, s['y'] - y) <= 0.5:
                s['kern'] |= kern; break
        else: st.append(dict(typ='konflikt', x=x, y=y, kern=kern))
    for u in sorted(offene):
        if u in FIXED or u.startswith(('FIX#', 'HEFT#')): continue
        ks = [int(u[4:])] if u.startswith('GND#') else bynet.get(u, [])
        seen = []
        for k in ks:
            p = D['pads'][k]
            if any(math.hypot(p['x'] - x, p['y'] - y) <= 1.0 for x, y in seen): continue
            seen.append((p['x'], p['y']))
            st.append(dict(typ='offen', x=p['x'], y=p['y'], kern={u}))
            if len(seen) >= 4: break
    return st


def _zug_abstand(r, p):
    x, y = p; d = float('inf')
    for v in r['vias']: d = min(d, math.hypot(v['x'] - x, v['y'] - y))
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            s_ = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
            d = min(d, math.hypot(a[0] + s_ * dx - x, a[1] + s_ * dy - y))
    return d


ZUG_LOKAL = args.get('zug_lokal', '1') == '1'
ZUG_PRUEF = args.get('zug_pruef', '0') == '1'


_ZUG_RAND = _DMAX + max(DRC_HOLE, VIA_DR / 2 + VIA_R + LOCH_CLR, VIA_DR / 2 + LOCH_CLR, VIA_DR / 2 + H2H) + VIA_R + 0.05


def konflikte_lokal(gruppe):
    ""
    g = set(gruppe); b = None
    for w in sorted(g):
        if w not in U: continue
        for _li, k, _kb, _ne in _unit_geo(w, U[w]):
            x0, y0, x1, y1 = k.bounds
            b = [x0, y0, x1, y1] if b is None else [min(b[0], x0), min(b[1], y0), max(b[2], x1), max(b[3], y1)]
    if b is None: return {}
    F = (b[0] - _ZUG_RAND, b[1] - _ZUG_RAND, b[2] + _ZUG_RAND, b[3] + _ZUG_RAND)
    sv = (PAIRS[:], PAIRS_ROH[:], FIX_PAIRS[:])
    def auswahl(conf):
        paare = [p for p in PAIRS if p[2] in g or p[3] in g]
        mit = {u for p in paare for u in p[2:4]}
        return {u: cs for u, cs in conf.items() if u in mit}, sorted(paare)
    res, paare = auswahl(exact_conflicts(fenster=F))
    if ZUG_PRUEF:
        resv, paarev = auswahl(exact_conflicts())
        gleich = (sorted(res) == sorted(resv) and paare == paarev
                  and all(sorted(res[u]) == sorted(resv[u]) for u in res if u in g))
        _zs['pruef'] = _zs.get('pruef', 0) + 1
        if not gleich:
            raise AssertionError('Zug-Fensterpruefung weicht ab: %s / %s' % (sorted(set(res) ^ set(resv)), sorted(set(paare) ^ set(paarev))[:6]))
    PAIRS[:], PAIRS_ROH[:], FIX_PAIRS[:] = sv
    return res


def _zug_konf(gruppe, X):
    return konflikte_lokal(gruppe) if ZUG_LOKAL else konflikte_gruppe(gruppe, X)


def _zug_wert(gruppe, X):
    cfg = _zug_konf(gruppe, X)
    return len(cfg) + 2 * sum(1 for w in gruppe if w in fail), cfg


def zug_versuch(stelle, strat, saat):
    ""
    import copy
    R = strat['R']; xy = [(stelle['x'], stelle['y'])]
    kern = sorted(stelle['kern'])
    nah = [w for w, r in sorted(U.items()) if w not in stelle['kern'] and not w.startswith(('FIX#', 'HEFT#'))
           and w not in FIXED and _nah_an(r, xy, R)]
    gekappt = len(kern) + len(nah) > ZUG_GRUPPE
    if gekappt:
        nah = sorted(nah, key=lambda w: (_zug_abstand(U[w], xy[0]), w))[:max(0, ZUG_GRUPPE - len(kern))]
    gruppe = list(kern) + nah
    if PAAR_NETZE: gruppe = sorted(set(gruppe) | {PAAR_NETZE[w] for w in gruppe if w in PAAR_NETZE and PAAR_NETZE[w] in units and PAAR_NETZE[w] not in FIXED})
    zeile = dict(zug=_zs['n'], typ=stelle['typ'], x=round(stelle['x'], 3), y=round(stelle['y'], 3), R=R, reihe=strat['reihe'], warm=bool(strat.get('warm')),
                 reparatur=bool(strat.get('reparatur')), kern=len(kern), gruppe=len(gruppe),
                 gnd=sum(1 for w in gruppe if w.startswith('GND#')), paar=sum(1 for w in gruppe if w in PAAR_NETZE),
                 arten=sorted({(KL['netze'].get(w) or {}).get('art', 'signal') for w in kern}) if KL else [],
                 pads=sum(1 for p in D['pads'] if abs(p['x'] - stelle['x']) <= R and abs(p['y'] - stelle['y']) <= R),
                 pads2=_zug_pads2(stelle['x'], stelle['y']), politik=bool(ZUG_POLITIK),
                 konflikte_ges=len(PAIRS), offen_ges=len(fail), lagen=NL, gekappt=gekappt)
    if len(gruppe) > ZUG_GRUPPE:
        zeile.update(ergebnis='gruppe_zu_gross'); _zug_log(zeile); return False
    _tc = time.process_time(); _ta = arbeit_gesamt() if BUDGET_ARBEIT else 0
    X = None if ZUG_LOKAL else gruppen_index(gruppe)
    vor, _ = _zug_wert(gruppe, X)
    save = {w: copy.deepcopy(U[w]) for w in gruppe if w in U}; fail_vor = {w for w in gruppe if w in fail}
    hsave = hist.copy(); PFs = PF[0]; weich = VG_WEICH[0]; VG_WEICH[0] = False; qf = QUELLFREI[0]
    QUELLFREI[0] = bool(strat.get('reparatur')) or qf; HART_FREMD[0] = False
    rng_ = budget_rng(5000 + saat)
    andere = [w for w in gruppe if w not in stelle['kern']]
    if strat.get('warm'):
        for w in kern:
            if w in U: del_unit(w)
        PF[0] = min(PFs, ZUG_WARM_PF)
        order = list(kern)
    else:
        for w in gruppe:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if strat['reihe'] == 'kern': order = kern + sorted(andere, key=lambda w: (grp(w)[0], rng_.rand()))
        else: order = sorted(gruppe, key=lambda w: rng_.rand())
    ok = {}; verlauf = []
    for w in paar_reihe(order): ok[w] = route_unit(w)
    for _k in range(ZUG_ITER):
        cfg = _zug_konf(gruppe, X); verlauf.append(len(cfg))
        if not cfg or zeit_knapp(0.95): break
        if len(verlauf) >= 4 and min(verlauf[-3:]) >= verlauf[-4]: break
        if strat.get('warm') and len(verlauf) >= 3 and min(verlauf[-2:]) >= min(verlauf[:-2]): break
        for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
        PF[0] *= 1.5
        if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
        for w in paar_reihe(sorted((w for w in gruppe if w in cfg or not ok.get(w, True)), key=lambda w: rng_.rand())):
            if not paar_einheit(w): del_unit(w)
            ok[w] = route_unit(w)
    for w in gruppe:
        if ok.get(w, w in U): fail.discard(w)
        else: fail.add(w)
    nach, _ = _zug_wert(gruppe, X)
    hist[:] = hsave; PF[0] = PFs; VG_WEICH[0] = weich; QUELLFREI[0] = qf
    angenommen = nach < vor
    if not angenommen:
        for w in gruppe:
            if w in U: del_unit(w)
        for w, r in sorted(save.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
        for w in gruppe:
            if w in fail_vor: fail.add(w)
            else: fail.discard(w)
    cpu = time.process_time() - _tc; _zs['cpu'] += cpu
    if BUDGET_ARBEIT: _zs['arbeit'] += arbeit_gesamt() - _ta
    zeile.update(ergebnis='angenommen' if angenommen else 'verworfen', vorher=vor, nachher=nach, verlauf=verlauf, cpu_s=round(cpu, 3))
    _zug_log(zeile)
    return angenommen


def zug_merken():
    ""
    import copy
    wert = len(exact_conflicts()) + len(fail)
    if _zs.get('stand') is None or wert < _zs['stand'][0]:
        _zs['stand'] = (wert, {w: copy.deepcopy(r) for w, r in U.items() if not w.startswith(('FIX#', 'HEFT#'))}, set(fail))


def zug_bester():
    ""
    global conf
    if _zs.get('stand') is None: return False
    jetzt = len(exact_conflicts()) + len(fail)
    if _zs['stand'][0] >= jetzt: return False
    _, alt, alt_fail = _zs['stand']
    for w in sorted(set(U) | set(alt)):
        if w.startswith(('FIX#', 'HEFT#')): continue
        if w in U: del_unit(w)
    for w, r in sorted(alt.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
    fail.clear(); fail.update(alt_fail)
    conf = exact_conflicts()
    print('   Zugschleife: besten Stand zurueckgeholt (%d statt %d Konflikte+offen)' % (_zs['stand'][0], jetzt), flush=True)
    return True


def zug_budget_aus():
    if _zs['n'] >= ZUG_MAX or zeit_knapp(0.95): return True
    if BUDGET_ARBEIT: return _zs['arbeit'] >= ZUG_ANTEIL * ARBEIT_MAX
    return CPU_MAX > 0 and _zs['cpu'] >= ZUG_ANTEIL * CPU_MAX


def zugschleife(phase, saat):
    ""
    global conf
    conf = exact_conflicts()
    if not (0 < len(conf) + len(fail) <= ZUG_STELLEN): return 0
    gut = 0
    for i, st in enumerate(_zug_stellen(list(PAIRS), set(fail))):
        if not (st['kern'] & (set(conf) | fail)): continue
        for j, strat in _zug_reihe(st):
            if zug_budget_aus(): break
            tabu = (round(st['x'], 1), round(st['y'], 1), j)
            if _zs.setdefault('tabu', {}).get(tabu, 0) >= 2: continue
            _zs['tabu'][tabu] = _zs['tabu'].get(tabu, 0) + 1
            _zs['n'] += 1
            if zug_versuch(st, strat, 100 * saat + 10 * i + j):
                gut += 1; _zs['ok'] += 1; conf = exact_conflicts(); break
    print('   Zugschleife (%s): %d Zuege gesamt, %d uebernommen in diesem Durchgang, jetzt %d Konflikte, %d offen (%.0f CPU-s)' % (
        phase, _zs['n'], gut, len(conf), len(fail), _zs['cpu']), flush=True)
    return gut


if MAXIT < 2 and todo is None:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANKER_AN[0] = True
_AL = None
if args.get('agent_loop') == '1':
    from codex.agent_loop.zustand import AgentLoop
    _AL = AgentLoop(globals(), _AGENT_SAVED)
for it in range(_AGENT_SAVED['runde'] if _AGENT_SAVED else 2, MAXIT + 1):
    VG_WEICH[0] = it <= int(args.get('vg_iter', '10'))

    conf = exact_conflicts(); _es['lief'] = True
    if (_AL.before_round(it, list(PAIRS)) if _AL else (_RL and _RL.tick(it - 1, list(PAIRS)))):
        print('   Routing-Loop: lokalen Zug uebernommen', flush=True)
        conf = exact_conflicts()
    verhandlung_messen(it - 1, conf)
    _etik_conf = _ET.konfliktkopie(conf) if _ET else None
    if VIA_VORRANG and it <= WMIN_BIS:
        conf, _nvr = vorrang_filtern(conf)
        if _nvr: print('   Via-Vorrang (R25): %d Stellen, an denen nur das Signal weicht' % _nvr, flush=True)
    if WMIN_KONF and it <= WMIN_BIS:
        _eng = eng_konflikte()
        for _u, _c in _eng.items(): conf.setdefault(_u, []).extend(_c)
        if _ET: _ET.ergaenzen(_etik_conf, _eng)
        if _eng: print('   zu schmal (wmin_konf): %d Einheiten beteiligt' % len(_eng), flush=True)
    if _ET and not _LERN_AN: _ET.beobachten(_etik_conf, it - 1)
    todo = (set(conf) | fail) - set(FIXED)
    print('Iteration %d: Konflikte %d, unvollstaendig %d, PF %.2f (%.0fs)' % (it, len(conf), len(fail), PF[0], time.time() - t0), flush=True)
    arbeit_marke('runde_%d' % it)
    if it % 4 == 0:
        import collections
        print('   Paare', len(PAIRS), collections.Counter(tuple(sorted((x[2].split('#')[0][:12], x[3].split('#')[0][:12]))) for x in PAIRS).most_common(12), flush=True)
        print('   Beispiele', sorted(PAIRS, key=lambda x: x[1])[:6], flush=True)
    if not todo: break
    if zeit_knapp():
        print('%s zu 80 %% verbraucht: Verhandlung endet mit %d Konflikten' % (budget_meldung(), len(conf)), flush=True); break
    if ZUG and it >= 8:
        _zsum = len(conf) + len(fail)
        if _zs['best'] is None or _zsum < _zs['best']: _zs['best'] = _zsum; _zs['ruhig'] = 0
        else: _zs['ruhig'] += 1
        if _zs['ruhig'] >= 3 and 0 < _zsum <= ZUG_STELLEN and not zug_budget_aus():
            _zs['ruhig'] = 0; _zgut = 0
            for _zr in range(4):
                if not zugschleife('Verhandlung', 10 * it + _zr): break
                _zgut += 1; zug_merken()
                if not conf and not fail: break
            if _zgut:
                _zs['best'] = len(conf) + len(fail); continue
    if ENDSPIEL and not fail and len(conf) <= END_MAX and it >= 6:
        if _es.get('ok_vor') is not None:
            if set(conf) == _es['ok_vor']:
                _es['schein'] = _es.get('schein', 0) + 1; _es['stufe'] += 1; _es['ab'] = it + 3
                print('   Endspiel: Scheinerfolg %d (derselbe Konflikt wieder da)' % _es['schein'], flush=True)
                if _es['schein'] >= 2: _es['aus'] = True; print('   Endspiel: aus fuer diesen Lauf', flush=True)
            _es['ok_vor'] = None
        _es['n'] = _es['n'] + 1 if set(conf) == _es['vor'] else 1; _es['vor'] = set(conf)
        if _es['n'] >= 4 and it >= _es['ab'] and not _es['aus']:
            _ok = endspiel(conf, list(PAIRS), _es['stufe'], it)
            print('   Endspiel (Anlaufrunde %d): %s' % (_es['stufe'] + 1, 'konfliktfrei' if _ok else 'ohne Erfolg, alter Stand'), flush=True)
            if _ok: _es['ok_vor'] = set(conf); continue
            _es['stufe'] += 1; _es['ab'] = it + 3
    else:
        _es['n'] = 0; _es['vor'] = None
    for u, b in conf.items(): hist[np.array(b, dtype=np.int64)] += HINC
    PF[0] *= float(args.get('pfmul', '1.5'))
    if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
    fail_alt = fail; fail = set(); _dran = set()
    alle = args.get('alle', '1') == '1' and len(conf) > int(args.get('ziel_ab', '0'))
    nah = ALLE_NAH
    if KONVERGENZ_LOKAL:
        if len(conf) < _lokal_best[0]: _lokal_best[:] = [len(conf), 0]
        else: _lokal_best[1] += 1
        alle = it % 4 == 0
        nah = ALLE_NAH or min(6.0, 1.0 + _lokal_best[1] / 4)
    rng = budget_rng(it)
    def gkey(u):
        if PAAR_NETZE:


            p = PAAR_NETZE.get(u)
            if p: return (grp(u)[0], -KL['netze'][u].get('paar_rang', 0), min(u, p), u)
            rang = (KL['netze'].get(u) or {}).get('rang', 0) if args.get('rang_alle', '0') == '1' else 0
            if KL:
                rang *= float(args.get('kosten_rang_' + (KL['netze'].get(u) or {}).get('art', 'signal'), '1'))
            return (grp(u)[0], -rang, '', lern_reihenwert(u,rng.rand()))
        if KL and args.get('rang_alle', '0') == '1':
            _nk = KL['netze'].get(u) or {}
            return (grp(u)[0], -_nk.get('rang', 0) * float(args.get('kosten_rang_' + _nk.get('art', 'signal'), '1')), lern_reihenwert(u,rng.rand()))
        return (grp(u)[0], lern_reihenwert(u,rng.rand()))


    if alle and nah > 0:


        if KONVERGENZ_LOKAL:
            _m = lokale_suchmaske(PAIRS, fail_alt, nah)
        else:
            _m = np.zeros((NY, NX), dtype=bool); _rr = int(nah / RES)
            for (_l, _d, _a, _b, _x, _y) in PAIRS:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                _m[max(0, _iy - _rr):_iy + _rr + 1, max(0, _ix - _rr):_ix + _rr + 1] = True
        _mf = _m.ravel()
        reihe = [u for u in units if u in todo or (u in U and u not in FIXED and len(U[u]['cells']) and bool(_mf[U[u]['cells'] % NN].any()))]
        print('   Umkreis %.1f mm: %d von %d Einheiten neu' % (nah, len(reihe), len(units)), flush=True)
    else:
        reihe = units if alle else sorted(todo)
    if PAAR_NETZE:
        reihe = sorted(set(reihe) | {PAAR_NETZE[u] for u in reihe if u in PAAR_NETZE and PAAR_NETZE[u] in units})
    _verhandlungsreihe = paar_reihe(sorted(reihe, key=gkey))
    if _AL:
        from codex.agent_loop.zuege import reihenfolge as _agent_reihenfolge
        _verhandlungsreihe = _agent_reihenfolge(_verhandlungsreihe, _AGENT_PRIORITAET)
    for u in _verhandlungsreihe:
        if zeit_knapp():
            fail |= fail_alt - _dran; break
        if paar_einheit(u):
            _dran.update((u,PAAR_NETZE[u]))
            if route_unit(u): fail.discard(u); fail.discard(PAAR_NETZE[u])
            else: fail.update((u,PAAR_NETZE[u]))
        else:
            del_unit(u); _dran.add(u)
            if not route_unit(u): fail.add(u)
        if u in PAAR_NETZE and u > PAAR_NETZE[u] and u not in fail and PAAR_NETZE[u] not in fail:
            paar_koppeln(nur={u, PAAR_NETZE[u]})
if ENDSPIEL and _es['lief'] and not fail:
    conf = exact_conflicts()
    for _st in range(_es['stufe'], _es['stufe'] + 4):
        if not (0 < len(conf) <= END_MAX) or _es['aus']: break
        _ok = endspiel(conf, list(PAIRS), _st, 100 + _st)
        print('Endspiel nach dem letzten Durchgang (Anlaufrunde %d): %s' % (_st + 1, 'konfliktfrei' if _ok else 'ohne Erfolg'), flush=True)
        conf = exact_conflicts()
    todo = (set(conf) | fail) - set(FIXED)
elif _es['lief']:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
if ENDSPIEL and KL and args.get('endspiel_reparatur', '1') == '1' and not fail and not zeit_knapp():



    conf = exact_conflicts()
    if 0 < len(conf) <= END_MAX:
        _es['aus'] = False; _es_alt_cpu = _es['cpu']; _es['cpu'] = 0.0
        for _st in range(4):
            if zeit_knapp(): break

            _ok = endspiel(conf, list(PAIRS), _st, 24 + _st, reparatur=True)
            print('Endspiel-Reparatur: Umkreis %.1f mm, %s' % ((1.0, 2.0, 3.5, 5.0)[_st], 'konfliktfrei' if _ok else 'alter Stand'), flush=True)
            conf = exact_conflicts()
            if not conf or _es['aus']: break
        _es['cpu'] += _es_alt_cpu
        todo = (set(conf) | fail) - set(FIXED)
if ZUG and _es['lief']:
    zug_bester()
    for _zr in range(4):
        if zug_budget_aus() or not zugschleife('Ende %d' % (_zr + 1), 900 + _zr): break
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANKER_AN[0] = False
VG_WEICH[0] = False

_cf_checkpoint = exact_conflicts(); PAIRS_LAST = list(PAIRS)
arbeit_marke('vor_nachlauf')
verhandlung_messen(it if 'it' in globals() else 1, _cf_checkpoint, 'vor_nachlauf')
if _ET and not _LERN_AN: _ET.beobachten(_cf_checkpoint, it if 'it' in globals() else 1)
json.dump(PAIRS_LAST, open(OUT.replace('.json', '_konflikte_vor_verbreitern.json'), 'w'))
if ((BGA_PLAN and args.get('bga_budget', '0') == '1') or args.get('budget_vor_nachlauf', '0') == '1' or globals().get('_FRUEH_INFO') is not None) and zeit_knapp() and (_cf_checkpoint or fail):

    _ct, _cv = ausgabe_geometrie()
    _cc = {u:len(cs) for u,cs in _cf_checkpoint.items()}
    _sicher = ausgabe_daten(_ct, _cv, _cc,
        abschluss=(_LERN_STOP or 'cpu_budget') if _LERN_AN else 'budget_vor_nachlauf',
        **({'lernstand': json.loads(_LERN_SNAPSHOT)['lernstand'], 'lernbudget':json.loads(_LERN_SNAPSHOT)['lernbudget']} if _LERN_AN else {}))
    json.dump(_sicher, open(OUT,'w'))
    json.dump(PAIRS_LAST,open(OUT.replace('.json','_konflikte.json'),'w'))
    print('Rechenzeit-Budget: ungueltiger Stand gespeichert, Breiten-/Formnachlauf ausgelassen',flush=True)
    print('Ende: Konflikte',_cc,'unvollstaendig',sorted(fail),flush=True)
    print('Bahnen',len(_ct),'Vias',len(_cv),'Zeit %.0fs' % (time.time()-t_start),flush=True)
    if _ET: _ET.schreiben(_sicher['tracks'], _sicher['vias'], fehlend=_sicher['failed'], zonen=_sicher.get('zonen', []) + _sicher.get('zones', []))
    sys.exit(0)
import copy
def ulen(u):
    r = U[u]
    if VG and any(v.get('gr') for v in r['vias']):
        return sum(math.dist(a, c) for t in r['tracks'] if not t.get('steg') for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
    return sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
def grp_key(u): return grp(u)[0]


GEW_ART = None
if GEWICHTET:
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
        from gemeinsam.wichtigkeit import gewichte as _gewichte, tausch_besser as _tausch_besser
        GEW_ART = _gewichte()
    except Exception as _e:
        print('gewichtet=%d: Gewichte aus analyse/bewertung.py nicht ladbar (%r), es bleibt bei der nackten Laenge' % (GEWICHTET, _e), flush=True)
        GEWICHTET = 0
def u_gew(u):
    ""
    if GEW_ART is None or u not in U: return 1.0
    return GEW_ART.get(KL['netze'].get(U[u]['net'], {}).get('art', 'signal'), 1.0)
def u_vias(u):
    r = U[u]
    return n_sv(r)
def ulen_gew(u):
    ""
    nv = u_vias(u)
    return u_gew(u) * (ulen(u) - 0.6 * nv) + 0.6 * nv
def strom_widerstand(u, erreichbar=False):
    ""
    r = U[u]; net = r['net']; C = cls(net)
    k = (KL['netze'].get(net) or {}) if KL else {}
    from gemeinsam.fluss import analyse as fa, zielbreite as fz
    info = fa(r['tracks'], r['vias'], [D['pads'][i] for i in bynet_g.get(net, [])], k.get('fluss') or {})
    anteil = info['anteil'] if info and info['gesamt'] > 0 else None
    cap = None
    if erreichbar and r['tracks']:
        xy = [q for t in r['tracks'] for q in t['pts']]
        box = (min(q[0] for q in xy) - 1, min(q[1] for q in xy) - 1, max(q[0] for q in xy) + 1, max(q[1] for q in xy) + 1)
        cap = karten(net, box, C['w'] * max(f_innen(net, LI[t['layer']]) for t in r['tracks']))[6]
    out = 0.0
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('steg'): continue
        li = LI[t['layer']]; dick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            f = anteil.get((ti, si), 0.0) if anteil is not None else 1.0
            if cap is not None:
                ziel = fz(f, C['wmin'], C['w'], float(args.get('fluss_exp', '0.5'))) * f_innen(net, li)
                w = max(w, math.floor(min(ziel, line_cap_min(cap[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
            out += f * f * 1.72e-5 * math.dist(a, b) / (w * dick)
    out += 0.0024 * sum(1 for v in r['vias'] if not v.get('gr') and not v.get('rv'))
    return out


def routing_mass(u):
    if KL and args.get('strom_kurz') == '1' and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw'):

        return strom_widerstand(u, erreichbar=True) * cls(U[u]['net'])['w'] * 0.035 / 1.72e-5
    return ulen(u)


def routing_mass_gew(u):


    if KL and args.get('strom_kurz') == '1' and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw'):
        return u_gew(u) * routing_mass(u)
    return ulen_gew(u)


def kurz_key(u): return -ulen(u) * (u_gew(u) if GEWICHTET >= 2 else 1.0)


R19_KURZ = args.get('r19_kurz', '0') == '1' and KL is not None
R19_KURZ_STAT = dict(versuche=0, mehr_vias_verworfen=0, zusatz_vias_verworfen=0)
def r19_kurz_ok(alt, neu):
    return n_sv(neu) <= n_sv(alt)

if args.get('kurz') == '1' and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3
    _KIDX[0] = None
    for rnd in range(int(args.get('kurz_runden', '2'))):
        better = 0; gain = 0.0
        for u in sorted([u for u in units if not u.startswith('GND#') and u in U and not paar_einheit(u)], key=kurz_key):
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); L0 = routing_mass(u); _f0 = vg_fehl(u) if VG else 0
            del_unit(u)
            okk = route_unit(u)
            if okk and routing_mass(u) < L0 - 0.05 and (not VG or vg_fehl(u) <= _f0):
                if R19_KURZ and (KL['netze'].get(old['net']) or {}).get('art') in ('signal', 'analog', 'schnell'):
                    R19_KURZ_STAT['versuche'] += 1
                    if not r19_kurz_ok(old, U[u]):
                        R19_KURZ_STAT['mehr_vias_verworfen'] += 1
                        R19_KURZ_STAT['zusatz_vias_verworfen'] += n_sv(U[u]) - n_sv(old)
                        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u); continue
                if einheit_konfliktfrei(u): better += 1; gain += L0 - routing_mass(u); kidx_geaendert(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
        print('Kuerzen Runde %d: %d Netze kuerzer, zusammen %.1f mm (%.0fs)' % (rnd + 1, better, gain, time.time() - t2), flush=True)
        if not better: break
    PF[0] = PFs
    if R19_KURZ: print('R19 Kuerzen:', R19_KURZ_STAT, flush=True)



KURZ_VIA = float(args.get('kurz_via', '8' if KL and 'start' not in args else '0'))
if KURZ_VIA > 0 and KL is not None and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _KIDX[0] = None; VIA_FAK[0] = KURZ_VIA
    _mm = float(args.get('kurz_via_mm', '3.0')); _gesp = 0; _mehr = 0.0
    def _via_kandidat(u):
        if paar_einheit(u) or u.startswith(('GND#', 'FIX#', 'HEFT#')) or u not in U or not U[u]['vias']: return False
        if KL['netze'].get(U[u]['net'], {}).get('art') in ('strom', 'versorgung', 'sw', 'gnd'): return False
        return not any(v.get('n') or v.get('gr') for v in U[u]['vias'])



    def _via_gew(u): return SCHNELL_VIA if SCHNELL_VIA != 1 and ist_schnell(U[u]['net']) else 1.0
    def _via_key(u):
        if GEWICHTET >= 2: return (-n_sv(U[u]) * _via_gew(u) * u_gew(u), u)
        return (-n_sv(U[u]) * _via_gew(u), u)
    for rnd in range(int(args.get('kurz_via_runden', '2'))):
        _n = 0
        for u in sorted([u for u in units if _via_kandidat(u)], key=_via_key):
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); _v0 = n_sv(old); L0 = ulen(u) - 0.6 * _v0; _vg_ = _via_gew(u)
            del_unit(u)
            okk = route_unit(u)
            _v1 = n_sv(U[u]); L1 = ulen(u) - 0.6 * _v1
            _erl = _mm * (2.0 if KL['netze'].get(old['net'], {}).get('art', 'signal') == 'signal' else 1.0) * _vg_
            if okk and _v1 < _v0 and L1 <= L0 + _erl * (_v0 - _v1) and einheit_konfliktfrei(u):
                _n += 1; _gesp += _v0 - _v1; _mehr += L1 - L0; kidx_geaendert(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
        if not _n: break
    VIA_FAK[0] = 1.0; PF[0] = PFs
    print('Via-Runde (Via-Preis x%g): %d Vias gespart, Bahnen dafuer %.1f mm laenger (%.0fs)' % (KURZ_VIA, _gesp, _mehr, time.time() - t2), flush=True)





if (VAP or EPM) and args.get('via_am_pad_raeumen', '0' if 'start' in args else '1') == '1' and not todo and not zeit_knapp():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _nr = 0; _nv = 0; _mehr = 0.0; _nep = 0; _gew = 0.0
    _umw = float(args.get('via_am_pad_umweg', '2.0')); _gmin = float(args.get('via_am_pad_gewinn', '0.6'))
    def _guete(u):
        r = U[u]; _p = D['pads'][int(u[4:])]; _v = r['vias'][0]; _c, _w = _vap_zellen(int(u[4:]))
        _j = np.flatnonzero(_c == int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES))
        return (sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:]))
                + 2.0 * math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']) + VAP_K * (float(_w[_j[0]]) if len(_j) else 1.0))
    for u in [u for u in units if u.startswith('GND#') and u in U and U[u]['vias']]:
        if zeit_knapp(0.9): break
        _k = int(u[4:]); _n0 = len(U[u]['vias'])
        if _k in EPM:
            if _n0 >= EPM[_k]['soll']: continue
        elif not VAP: continue
        else: _g0 = _guete(u)
        save = {u: copy.deepcopy(U[u])}
        del_unit(u)
        if _k in EPM:
            VG_WEICH[0] = True; okk = route_unit(u); VG_WEICH[0] = False
            okk = okk and len(U[u]['vias']) > _n0; _g1 = 0.0; _g0 = 0.0
        else:
            PF[0] = 0.01; okk = route_unit(u); PF[0] = 1e3
            if LAGENAUFBAU: okk = okk and bool(U[u]['vias'])
            if okk: _g1 = _guete(u); okk = _g1 < _g0 - _gmin
        if okk:
            _nv += 1; _pp = D['pads'][_k]
            _cf = konflikte_gruppe([u], gruppen_index([u])); im_weg = sorted(w for w in _cf if w != u)
            if not _cf:
                _nr += 1; _gew += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
            okk = 0 < len(im_weg) <= 4 and not any(w.startswith(('GND#', 'FIX#', 'HEFT#')) for w in im_weg)
            _grund = 'uebernommen' if okk else ('fremdes Pad im Weg' if not im_weg else 'andere Stiche, feste Einheiten oder mehr als vier im Weg')
            if okk:
                if PAAR_SUCHRAUM: im_weg=sorted(set(im_weg)|{PAAR_NETZE[w] for w in im_weg if paar_einheit(w)})
                L0 = sum(ulen(w) - 0.6 * n_sv(U[w]) for w in im_weg); _v0 = sum(n_sv(U[w]) for w in im_weg)
                _f0 = sum(vg_fehl(w) for w in im_weg) if VG else 0
                for w in im_weg: save[w]=copy.deepcopy(U[w])
                for w in paar_reihe(im_weg):
                    if not paar_einheit(w): del_unit(w)
                    if not route_unit(w): okk = False
                gruppe = [u] + im_weg
                if not okk: _grund = 'kein Weg fuer die Einheiten im Weg'
                else:
                    L1 = sum(ulen(w) - 0.6 * n_sv(U[w]) for w in im_weg); _dv = sum(n_sv(U[w]) for w in im_weg) - _v0
                    if L1 > L0 + _umw * (VAP_WF if (_k in VAP_WICHTIG or _k in EPM) else 1.0): okk = False; _grund = 'Umweg %.1f mm' % (L1 - L0)
                    elif _dv > 0: okk = False; _grund = '%d Vias mehr' % _dv
                    elif VG and sum(vg_fehl(w) for w in im_weg) > _f0: okk = False; _grund = 'Via-Gruppe verliert ein Via'
                    elif konflikte_gruppe(gruppe, gruppen_index(gruppe)): okk = False; _grund = 'Konflikt bleibt'
            print('   %s:%s %s, im Weg %s: %s' % (_pp['ref'], _pp['num'], 'Massepad %d -> %d Vias' % (_n0, len(U[u]['vias'])) if _k in EPM else 'Stichkosten %.1f -> %.1f' % (_g0, _g1),
                                               im_weg[:5], _grund), flush=True)
            if okk:
                _nr += 1; _mehr += L1 - L0; _gew += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
        for w in save:
            if w in U: del_unit(w)
        for w in save: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    PF[0] = PFs; _KIDX[0] = None
    print('Platz am Pad geraeumt: %d von %d Stichen und Massepads mit besserem Platz verbessert (Stichkosten zusammen -%.1f), %d Massepad-Vias mehr, andere Bahnen dafuer %.1f mm laenger (%.0fs)' % (
        _nr, _nv, _gew, _nep, _mehr, time.time() - t2), flush=True)

def net_mst(n):
    pts = [(D['pads'][k]['x'], D['pads'][k]['y']) for k in bynet[n]]
    inT = [pts[0]]; rest = pts[1:]; L = 0
    while rest:
        d, i = min((min(math.dist(r, t) for t in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L
def unit_box(u):
    xs, ys = [], []
    for t in U[u]['tracks']:
        for q in t['pts']: xs.append(q[0]); ys.append(q[1])
    for v in U[u]['vias']: xs.append(v['x']); ys.append(v['y'])
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None
if args.get('umleg') == '1' and not todo and not zeit_knapp():
    t3 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _ugw = [0, 0]
    for rnd in range(int(args.get('umleg_runden', '3'))):
        cand = []
        for u in units:
            if paar_einheit(u) or u.startswith('GND#') or u not in U: continue
            m = net_mst(u); L = ulen(u)
            if L - m > float(args.get('umleg_min', '1.5')) and L / max(m, 0.1) > 1.4: cand.append(((L - m) * (u_gew(u) if GEWICHTET else 1.0), u))
        cand.sort(reverse=True); done = 0
        for exc, u in cand:
            if zeit_knapp(0.9): break
            pads = [D['pads'][k] for k in bynet[u]]
            bx = (min(p['x'] for p in pads) - 0.6, min(p['y'] for p in pads) - 0.6, max(p['x'] for p in pads) + 0.6, max(p['y'] for p in pads) + 0.6)
            gruppe = [u]
            for w in list(U):
                if w == u or w.startswith(('FIX#', 'HEFT#')): continue
                b2 = unit_box(w)
                if b2 and not (b2[0] > bx[2] or b2[2] < bx[0] or b2[1] > bx[3] or b2[3] < bx[1]): gruppe.append(w)
            if PAAR_SUCHRAUM:
                gruppe=sorted(set(gruppe)|{PAAR_NETZE[w] for w in gruppe if paar_einheit(w)})
            if len(gruppe) > int(args.get('umleg_max', '40')): continue
            save = {w: copy.deepcopy(U[w]) for w in gruppe}
            _ux = gruppen_index(gruppe) if GEWICHTET else None
            L0 = sum(routing_mass(w) for w in gruppe)
            if GEWICHTET:
                _g0 = {w: (u_gew(w), routing_mass(w), routing_mass_gew(w)) for w in gruppe}
            for w in gruppe: del_unit(w)
            okk = True; hsave = hist.copy(); PF[0] = 1.0
            order = [u] + sorted((w for w in gruppe if w != u), key=grp_key)
            for w in paar_reihe(order):
                if not route_unit(w): okk = False
            for k_ in range(int(args.get('umleg_iter', '12'))):
                cfg = konflikte_gruppe(gruppe, _ux) if _ux is not None else exact_conflicts()
                if args.get('umleg_pruef') == '1': assert set(cfg) == set(exact_conflicts()), 'Umleg-Gruppenpruefung'
                bad = [w for w in cfg if w in gruppe]
                if not bad or not okk: break
                for w, cells in cfg.items():
                    if w in gruppe: hist[np.array(cells, dtype=np.int64)] += 1.0
                PF[0] *= 1.4
                for w in paar_reihe([w for w in order if w in bad]):
                    if not paar_einheit(w): del_unit(w)
                    if not route_unit(w): okk = False
            PF[0] = 1e3; hist[:] = hsave
            L1 = sum(routing_mass(w) for w in gruppe if w in U)
            _besser = L1 < L0 - 0.3
            if GEWICHTET and okk:



                _besser, _gw0, _gw1, _umgekehrt = _tausch_besser([(_g0[w][0], _g0[w][1], routing_mass(w), _g0[w][2], routing_mass_gew(w)) for w in gruppe])
                _ugw[0] += 1
                if _besser != (L1 < L0 - 0.3):
                    _ugw[1] += 1
                    print('   umleg %-24s gewichtet %s: nackt %.1f -> %.1f mm, gewichtet %.1f -> %.1f%s' % (
                        u, 'angenommen' if _besser else 'abgelehnt', L0, L1, _gw0, _gw1, ', ein wichtigeres Netz wuerde laenger' if _umgekehrt else ''), flush=True)
            if okk and _besser and not (konflikte_gruppe(gruppe, _ux) if _ux is not None else exact_conflicts()):
                done += 1; print('   umgelegt %-24s Gruppe %2d: %.1f -> %.1f mm' % (u, len(gruppe), L0, L1), flush=True)
            else:
                for w in gruppe:
                    if w in U: del_unit(w)
                for w in gruppe: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
        print('Umlegen Runde %d: %d Gruppen verbessert (%.0fs)' % (rnd + 1, done, time.time() - t3), flush=True)
        if not done: break
    PF[0] = PFs
    if GEWICHTET: print('Umlegen gewichtet (R21): %d vollstaendige Versuche, %d davon anders entschieden als nach nackter Laenge' % tuple(_ugw), flush=True)







EINZELTEST = args.get('einzeltest', '0') == '1'
ET_ORT = args.get('et_ort', 'nach')
def einzeltest():
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analyse.kopplung import kopplung as _kopplung, mengen as _kmengen
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _KIDX[0] = None
    _ET_O, _ET_S = _kmengen(KL)
    _ET_GEW = GEW_ART
    if _ET_GEW is None:
        try:
            from gemeinsam.wichtigkeit import gewichte as _gw_et
            _ET_GEW = _gw_et()
        except Exception: _ET_GEW = {}
    _p_via = float(args.get('et_via', args.get('kurz_via_mm', '3.0'))); _p_schmal = float(args.get('et_schmal', '2.0'))
    _p_kopp = float(args.get('et_kopp', '1.0')); _et_min = float(args.get('et_min', '0.1'))
    _et_vmehr = int(args.get('et_vias_mehr', '0')); _et_var = int(args.get('et_varianten', '2')); _et_stoer = float(args.get('et_stoer', '2.0'))
    _HART = ('strom', 'versorgung', 'sw')
    def _et_art(u): return KL['netze'].get(U[u]['net'], {}).get('art', 'signal')
    def _et_schmal(u):
        ""
        r = U[u]; C = cls(r['net']); Cs = cls('/__signal__'); s = 0.0
        WF = hals_feld_karte(r['net'], C, C['wmin'], False)
        for t in r['tracks']:
            if t.get('steg'): continue
            soll = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= soll - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                if WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < soll: continue
                s += math.dist(a, b)
        return s
    def _et_tr(w): return [dict(net=U[w]['net'], layer=t['layer'], pts=t['pts'], w=t['w'], kelvin=bool(t.get('abg'))) for t in U[w]['tracks']]
    def _et_kopp(u):
        ""
        net = U[u]['net']; k = 0.0
        if net in _ET_O or any(t.get('abg') for t in U[u]['tracks']):
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] in _ET_S for x in _et_tr(w)])
            k += _kopplung(R, KL, opfer={net}, stoerer=_ET_S)['summe']
        if net in _ET_S:
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] != net and (U[w]['net'] in _ET_O or any(t.get('abg') for t in U[w]['tracks'])) for x in _et_tr(w)])
            k += _kopplung(R, KL, stoerer={net: _ET_S[net]})['summe']
        return k
    def _et_mass(u):
        nv = u_vias(u); Lb = ulen(u) - 0.6 * nv; g = _ET_GEW.get(_et_art(u), 1.0)
        pv = _p_via * (2.0 if ist_schnell(U[u]['net']) else 1.0); ps = _p_schmal * (2.5 if _et_art(u) in _HART else 1.0)
        Ls = _et_schmal(u); K = _et_kopp(u)
        return dict(note=g * Lb + pv * nv + ps * Ls + _p_kopp * K, L=Lb, v=nv, s=Ls, k=K, f=vg_fehl(u) if VG else 0)
    def _et_ok(u, m0, m1):
        art = _et_art(u); hart = art in _HART
        if m1['note'] >= m0['note'] - _et_min: return False
        if m1['L'] > m0['L'] * 1.10 + 1.0: return False
        if m1['v'] > m0['v'] + (0 if hart or ist_schnell(U[u]['net']) else _et_vmehr): return False
        if m1['s'] > m0['s'] + (1e-6 if hart else 0.3): return False
        if m1['k'] > m0['k'] + 1e-6 or m1['f'] > m0['f']: return False
        return True
    _et_units = sorted([u for u in units if u in U and not paar_einheit(u) and not u.startswith(('GND#', 'FIX#', 'HEFT#')) and U[u]['tracks']
                        and not any(v.get('gr') or v.get('n') for v in U[u]['vias'])], key=lambda u: (grp(u), u))
    _et_n = 0; _et_var2 = 0; _et_d = dict(note=0.0, L=0.0, v=0, s=0.0, k=0.0); _et_det = []
    for u in _et_units:
        if zeit_knapp(0.9): break
        old = copy.deepcopy(U[u]); m0 = _et_mass(u); best = None
        varianten = [1.0] + ([_et_stoer] if _et_var >= 2 and stoer_extra(U[u]['net']) is not None else [])
        del_unit(u)
        for vi, fak in enumerate(varianten):
            STOER_FAK[0] = fak; okk = route_unit(u); STOER_FAK[0] = 1.0
            if okk:
                m1 = _et_mass(u)
                if _et_ok(u, m0, m1) and (best is None or m1['note'] < best[0]['note']) and einheit_konfliktfrei(u):
                    best = (m1, copy.deepcopy(U[u]), vi)
            del_unit(u)
        if best is not None:
            m1, nu, vi = best; add_unit(u, nu['net'], nu['tracks'], nu['vias']); kidx_geaendert(u)
            _et_n += 1; _et_var2 += vi
            for k_ in _et_d: _et_d[k_] += m1[k_] - m0[k_]
            _et_det.append('%s %.1f->%.1f (L %+.1f, V %+d, schmal %+.2f, Kopplung %+.2f%s)' % (
                u.split('/')[-1], m0['note'], m1['note'], m1['L'] - m0['L'], m1['v'] - m0['v'], m1['s'] - m0['s'], m1['k'] - m0['k'], ', Stoerkosten x%g' % _et_stoer if vi else ''))
        else:
            add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u)
    PF[0] = PFs; _KIDX[0] = None
    print('Einzeltest: %d von %d Einheiten besser (%d mit doppelten Stoerkosten); Mass %+.1f, Bahn %+.1f mm, Vias %+d, unter Mindestbreite %+.2f mm, Kopplung %+.2f (%.0fs)' % (
        _et_n, len(_et_units), _et_var2, _et_d['note'], _et_d['L'], _et_d['v'], _et_d['s'], _et_d['k'], time.time() - t2), flush=True)
    for _z in _et_det: print('   ' + _z, flush=True)
if EINZELTEST and ET_ORT == 'vor' and KL is not None and not todo and not zeit_knapp(): einzeltest()
def vs_kennzahlen(tracks, vias):
    ""
    L = 0.0; Lo = 0.0; knicke = 0; kurz_ = 0; seg = 0
    for t in tracks:
        rv = None; fl = t['net'] in FLNETZE
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b); L += d; seg += 1
            if not fl: Lo += d
            if d < 0.1: kurz_ += 1
            if d < 1e-9: continue
            r = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
            if rv is not None and rv[0] * r[0] + rv[1] * r[1] < 0.99985: knicke += 1
            rv = r
    art = {}
    if KL is not None:
        for v in vias:
            a_ = 'gnd' if v['net'] in GNDS else (KL['netze'].get(v['net']) or {}).get('art', 'signal'); art[a_] = art.get(a_, 0) + 1
    return dict(vias=len(vias), vias_ohne_flaeche=sum(1 for v in vias if v['net'] not in FLNETZE),
                strom_vias=sum(1 for v in vias if v.get('strom_via') or v.get('gr')), bahnen=len(tracks), segmente=seg,
                knicke=knicke, seg_unter_0_1=kurz_, laenge=round(L, 2), laenge_ohne_flaeche=round(Lo, 2),
                **({'vias_je_art': dict(sorted(art.items()))} if KL is not None else {}))


def kennzahlen_jetzt():
    return vs_kennzahlen([dict(net=r['net'], pts=t['pts']) for r in U.values() for t in r['tracks']],
                      [dict(v, net=r['net']) for r in U.values() for v in r['vias']])




def lokal_anschluss(pts, A, B, capL, li, net, w):
    q = [tuple(x) for x in pts]; q[0] = tuple(A); q[-1] = tuple(B)
    out = [q[0]]
    for a, b in zip(q[:-1], q[1:]):
        if math.dist(a, b) < 1e-8: continue
        def frei(x, y): return line_cap_min(capL, x, y) >= w - 1e-6 and pad_room(li, x, y, net) >= w / 2 - 1e-6
        if not okt_netz(net) or okt(a, b):
            if not frei(a, b): return None
            out.append(b)
        else:
            c = next((c for c in _ecken(a, b) if frei(a, c) and frei(c, b)), None)
            if c is None: return None
            out.extend((c, b))
    return out if len(out) >= 2 else None

def ast_verschieben(w):
    ""
    old = copy.deepcopy(U[w]); net = old['net']; C = cls(net)
    if paar_einheit(w) or w.startswith(('FIX#', 'HEFT#', 'GND#')) or (KL and (KL['netze'].get(net) or {}).get('quarz')): return False
    for ti, t in enumerate(old['tracks']):
        if t.get('abg') or t.get('steg') or len(t['pts']) < 3: continue
        A, B = t['pts'][0], t['pts'][-1]; li = LI[t['layer']]; ln = _LS(t['pts'])
        innen = ln.difference(_PT(A).buffer(0.03).union(_PT(B).buffer(0.03)))
        rest = [q for j, q in enumerate(old['tracks']) if j != ti]
        kontakte = [(_LS((a, b)).buffer(max(t['w']) / 2).intersection(_LS((c, d)).buffer(ww / 2)))
                    for a, b in zip(t['pts'][:-1], t['pts'][1:]) for q in rest if q['layer'] == t['layer'] for c, d, ww in zip(q['pts'][:-1], q['pts'][1:], q['w'])]
        rand = max([max(t['w'])] + [max(q['w']) for q in rest]) + 0.03
        enden = _PT(A).buffer(rand).union(_PT(B).buffer(rand))
        if any(not g.difference(enden).is_empty for g in kontakte): continue
        if any(innen.distance(_PT(v['x'], v['y'])) < VIA_D / 2 for v in old['vias']): continue
        box = (min(q[0] for q in t['pts']) - 1, min(q[1] for q in t['pts']) - 1, max(q[0] for q in t['pts']) + 1, max(q[1] for q in t['pts']) + 1)
        del_unit(w); add_unit(w, net, rest, old['vias']); NET_CUR[0] = net
        ch, vh, cs, dd, rng, vb, cc, cf, vbf = karten(net, box, C['w'])
        def node(q): return li * NN + int((q[1] - Y0) / RES) * NX + int((q[0] - X0) / RES)
        src, tgt = node(A), node(B)
        if ch[li].ravel()[src % NN] < min(t['w']) or ch[li].ravel()[tgt % NN] < min(t['w']): path = None
        else: path = astar_pf([src], {tgt}, (B[0], B[1], B[0], B[1]), cc, cs, np.zeros_like(vh), vb, C, box)
        Q = None
        if path:
            tr, vi = path_to_geometry(path, cc, C)
            if len(tr) == 1 and not vi:
                Q = lokal_anschluss(tr[0]['pts'], A, B, cc[li], li, net, min(t['w']))
        del_unit(w)
        if Q is not None:
            nt = dict(t, pts=Q, w=[min(t['w'])] * (len(Q) - 1)); neu = list(old['tracks']); neu[ti] = nt
            add_unit(w, net, neu, old['vias'])
            if einheit_konfliktfrei(w): return True
            del_unit(w)
        add_unit(w, net, old['tracks'], old['vias'])
    return False




























VS_LOG = []
if args.get('vias_sparen') == '1' and not todo and not zeit_knapp():
    t4 = time.time(); _vs_c0 = time.process_time()
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    try: from analyse.bewertung import GEW as VS_GEW
    except Exception: VS_GEW = {'sw': 4.0, 'strom': 2.5, 'versorgung': 1.0, 'schnell': 3.0, 'analog': 2.0, 'signal': 1.0}
    VS_MM = float(args.get('vs_mm', '3.0'))
    VS_VAR = [(float(x.split(':')[0]), float(x.split(':')[1]) if ':' in x else VS_MM) for x in args.get('vs_var', '1000/3').split('/') if x]
    VS_HART = float(args.get('vs_hart', '1000')); VS_GR = int(args.get('vs_gruppe', '6')); VS_IT = int(args.get('vs_iter', '4'))
    VS_MIN = float(args.get('vs_min', '1.0')); VS_TOL = float(args.get('vs_tol', '0.1'))
    VS_EPS = float(args.get('vs_gewinn', '0.5'))
    VS_KNICK = float(args.get('vs_knick', '0'))
    VS_SCHUTZ = args.get('vs_schutz', '1') == '1'
    VS_STROM = args.get('vs_strom', '0') == '1'
    VS_BREITE = args.get('vs_breite', '1') == '1'
    VS_RUECK = float(args.get('vs_rueck', '1.0'))
    VS_AMPAD = float(args.get('vs_ampad', '0.5'))
    VS_REIN = args.get('vs_rein', '0') == '1'
    VS_PADS = int(args.get('vs_pads', '6'))
    VS_GEDULD = int(args.get('vs_geduld', '1'))
    VS_NACHBAR_VIAS = args.get('vs_nachbar_vias', '1') == '1'
    VS_HOEHER = args.get('vs_hoeher', '1') == '1'
    _vs_p = sum(len(bynet[u]) for u in units if not u.startswith('GND#')) + sum(1 for u in units if u.startswith('GND#'))
    VS_ARBEIT = float(args.get('vs_arbeit', '6')) * _vs_p; _vs_arb = [0, 0.0, 0.0, 0.0]
    VS_BUDGET = max(float(args.get('vs_zeit_min', '30')), float(args.get('vs_zeit', '1.0')) * _vs_c0)
    VS_PRUEF = args.get('vs_pruef') == '1'
    _vs_alt = {w: (r['net'], r['tracks'], r['vias']) for w, r in U.items()}
    hsave = hist.copy(); hist[:] = 0; PFs = PF[0]; _KIDX[0] = None
    kz0 = kennzahlen_jetzt()
    def vs_vias(w): return sum(1 for v in U[w]['vias'] if not v.get('gr') and not v.get('rv'))
    def u_len(w): return sum(math.dist(a, c) for t in U[w]['tracks'] if not t.get('steg') for a, c in zip(t['pts'][:-1], t['pts'][1:]))
    def u_fehl(w): return vg_fehl(w) if VG else 0
    def u_knicke(w):
        n_ = 0
        for t in U[w]['tracks']:
            rv = None
            for a, b in zip(t['pts'][:-1], t['pts'][1:]):
                d = math.dist(a, b)
                if d < 1e-9: continue
                r_ = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
                if rv is not None and rv[0] * r_[0] + rv[1] * r_[1] < 0.99985: n_ += 1
                rv = r_
        return n_
    def vs_art(w):
        if w.startswith('GND#') or w in GNDS: return 'gnd'
        if KL is not None: return (KL['netze'].get(w) or {}).get('art', 'signal')
        return 'strom' if cls(w)['prio'] < 8 else 'signal'
    def vs_schutz(w): return paar_einheit(w) or VS_SCHUTZ and vs_art(w) in ('gnd', 'strom', 'versorgung', 'sw')
    def vs_strom(w): return vs_art(w) in ('gnd', 'strom', 'versorgung', 'sw')
    def vs_ist_strom(w): return VS_BREITE and not w.startswith('GND#') and vs_strom(w)
    def vs_padabst(w):
        bbs = [D['pads'][int(w[4:])]['bbox']] if w.startswith('GND#') else [D['pads'][k]['bbox'] for k in bynet_g.get(U[w]['net'], [])]
        return sorted(min(math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3])) for b in bbs)
                      for v in U[w]['vias'] if not v.get('gr') and not v.get('rv')) if bbs else []
    def vs_pad_ok(a_, n_): return not a_ or not n_ or (max(n_) <= max(a_) + 0.05 and sum(n_) / len(n_) <= sum(a_) / len(a_) + 0.05)
    def vs_leff(net, tracks):
        ""
        C = cls(net); soll = C['w']; s_ = 0.0
        xs_ = [q[0] for t in tracks for q in t['pts']] or [0.0]; ys_ = [q[1] for t in tracks for q in t['pts']] or [0.0]
        _cH, _, _cS, _dd, _, _, capC, _, _ = karten(net, (min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0), soll)
        for t in tracks:
            if t.get('steg'): continue
            li = LI[t['layer']]; dick = 1.0 if li in (0, NL - 1) else 2.0
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                d = math.dist(a, b)
                if d <= 0: continue
                if t.get('sig'): s_ += d; continue
                wv = max(w0, math.floor(min(soll, line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
                s_ += d * dick * soll / wv
        return s_
    def u_mass(w):
        if not vs_ist_strom(w): return u_len(w)
        s_ = (U[w]['net'], U[w]['tracks'], U[w]['vias']); del_unit(w); v = vs_leff(s_[0], s_[1]); add_unit(w, *s_)
        return v
    _vs_m = {}
    def u_mass0(w):
        if w not in _vs_m: _vs_m[w] = u_mass(w)
        return _vs_m[w]
    def u_gew(w): return VS_GEW.get(vs_art(w), 1.0)
    def u_vgew(w): return VS_MM * (2.0 if vs_art(w) == 'schnell' else 1.0)
    def vs_unten(n):
        gem = None
        for k in bynet[n]:
            s_ = {L for L in D['pads'][k]['layers'] if L in LI}; gem = s_ if gem is None else gem & s_
        return 0 if gem else 1
    def vs_potential(u):
        return u_gew(u) * (u_mass0(u) + u_vgew(u) * vs_vias(u) - net_mst(u) - u_vgew(u) * vs_unten(u))
    def vs_viaf(w, mm):
        if vs_schutz(w): return 1.0
        return max(1.0, mm * u_vgew(w) / VS_MM / max(cls(w)['via'], 0.05))
    def vs_pad_d(net, v):
        return min((math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3]))
                    for b in (D['pads'][k]['bbox'] for k in bynet_g.get(net, []))), default=9.0)
    def vs_maske(net, vias):
        ""
        m = np.zeros((NY, NX), dtype=bool)
        for v in vias:
            r_ = 0.0 if vs_pad_d(net, v) <= VS_AMPAD else VS_RUECK; rr = int(r_ / RES) + 1
            ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
            y0, y1, x0, x1 = max(0, iy - rr), min(NY, iy + rr + 1), max(0, ix - rr), min(NX, ix + rr + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            m[y0:y1, x0:x1] |= (xx - ix) ** 2 + (yy - iy) ** 2 <= (r_ / RES) ** 2
        return m.ravel()
    def vs_pad_vias_da(net, alt_v, neu_v):
        return all(any(abs(v['x'] - q['x']) < 0.011 and abs(v['y'] - q['y']) < 0.011 for q in neu_v) for v in alt_v if vs_pad_d(net, v) <= VS_AMPAD)
    def vs_versatz(alt_tr, neu_tr):
        m = 0.0
        for t in neu_tr:
            alte = [_LS(a['pts']) for a in alt_tr if a['layer'] == t['layer'] and len(a['pts']) >= 2]
            if not alte: return None
            for q in t['pts']: m = max(m, min(l_.distance(_PT(q[0], q[1])) for l_ in alte))
        return round(m, 2)
    _vs_eig = {}
    def vs_hart(net, tracks, vias):
        ""
        xs_ = [q[0] for t in tracks for q in t['pts']] + [v['x'] for v in vias]; ys_ = [q[1] for t in tracks for q in t['pts']] + [v['y'] for v in vias]
        if not xs_: return 0.0
        capH, viaH = hard_maps(net, _box_idx((min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0)) if GROSS else None)
        s_ = 0.0
        if net not in _vs_eig:
            _vs_eig[net] = {}
            for k in bynet_g.get(net, []):
                for L_, c in D['pads'][k]['bcells'].items(): _vs_eig[net].setdefault(LI[L_], set()).update(c.tolist())
        for t in tracks:
            li = LI[t['layer']]; capL = capH[li]; eig = _vs_eig[net].get(li, ())
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                L = math.dist(a, b)
                if L <= 0: continue
                tt = np.linspace(0, 1, max(2, int(L / (RES * 0.5)) + 1))
                ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
                eng = capL[iy, ix] < w0 - 0.005
                if eng.any(): s_ += L * sum(1 for q in (iy[eng] * NX + ix[eng]).tolist() if q not in eig) / len(tt)
        for v in vias:
            if not viaH[int((v['y'] - Y0) / RES), int((v['x'] - X0) / RES)]: s_ += VIA_D
        return s_
    def vs_fest(w, gu=None):
        if w.startswith(('FIX#', 'HEFT#')): return True
        if gu is None: return False
        if w.startswith('GND#'): return True
        if KL is not None and (KL['netze'].get(w) or {}).get('quarz'): return True
        if len(bynet[w]) > VS_PADS and not (args.get('vs_aeste') == '1' and not vs_schutz(w)): return True
        return gu is not None and not VS_HOEHER and u_gew(w) > gu + 1e-9
    def vs_route(w, wer):
        _t = time.process_time(); _vs_arb[0] += 1 if w.startswith('GND#') else len(bynet[w])
        ok_ = (ast_verschieben(w) if wer == 2 and w in U and args.get('vs_aeste') == '1' and not vs_schutz(w) else False)
        if not ok_ and not (wer == 2 and args.get('vs_aeste') == '1' and len(bynet[w]) > VS_PADS):
            if w in U: del_unit(w)
            ok_ = route_unit(w)
        _vs_arb[wer] += time.process_time() - _t
        return ok_
    def vs_versuch(u, pfw, mm, X):
        ""
        G = [u]; alt = {u: (U[u]['net'], U[u]['tracks'], U[u]['vias'])}; masken = {}
        wert = {u: (vs_vias(u), u_len(u), u_fehl(u), u_mass0(u), vs_padabst(u) if vs_schutz(u) else None, u_knicke(u) if VS_KNICK else 0)}; gu = u_gew(u)
        def k0(w): return u_gew(w) * (wert[w][3] + u_vgew(w) * wert[w][0] + VS_KNICK * wert[w][5])
        def k1(w): return u_gew(w) * (u_mass(w) + u_vgew(w) * vs_vias(w) + (VS_KNICK * u_knicke(w) if VS_KNICK else 0.0))
        hc = []; frei = False; verlauf = []; grund = 'kein eigener Gewinn'
        if VS_RUECK > 0 and vs_schutz(u): masken[u] = vs_maske(alt[u][0], alt[u][2])
        del_unit(u); PF[0] = pfw; VIAF[0] = vs_viaf(u, mm); VIA_MASKE[0] = masken.get(u)
        okk = vs_route(u, 1); VIA_MASKE[0] = None
        if okk and k1(u) < k0(u) - VS_EPS:
            grund = 'Konflikte bleiben'
            for runde in range(VS_IT + 1):
                _t = time.process_time(); cfg = konflikte_gruppe(G, X, set(G)); verlauf.append(len(cfg)); _vs_arb[3] += time.process_time() - _t
                if VS_PRUEF:
                    _v = exact_conflicts(); assert set(_v) == set(cfg), ('vias_sparen-Pruefung', sorted(set(_v) ^ set(cfg)))
                if not cfg: frei = True; break
                if runde == VS_IT: break
                neu = [w for w in sorted(cfg) if w not in alt and not vs_fest(w, gu)]
                if len(G) + len(neu) > VS_GR: grund = 'Gruppe zu gross'; break
                for w in neu:
                    alt[w] = (U[w]['net'], U[w]['tracks'], U[w]['vias']); wert[w] = (vs_vias(w), u_len(w), u_fehl(w), (_vs_m[w] if w in _vs_m else u_mass(w)) if vs_ist_strom(w) else u_len(w), vs_padabst(w) if vs_schutz(w) else None, u_knicke(w) if VS_KNICK else 0); G.append(w)
                andere = sorted((w for w in G[1:] if w in cfg), key=lambda w: vs_art(w) == 'schnell')
                if runde >= VS_GEDULD or not andere:
                    for w, cells in cfg.items():
                        c = np.array(cells, dtype=np.int64); hist[c] += 1.0; hc.append(c)
                PF[0] = VS_HART
                for w in andere:
                    if VS_RUECK > 0 and vs_schutz(w):
                        if w not in masken: masken[w] = vs_maske(alt[w][0], alt[w][2])
                        VIA_MASKE[0] = masken[w]
                    if args.get('vs_aeste') != '1' or vs_schutz(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm)
                    if not vs_route(w, 2): okk = False
                    VIA_MASKE[0] = None
                if u in cfg and (runde >= VS_GEDULD or not andere):
                    PF[0] = VS_HART if andere else pfw
                    del_unit(u); VIAF[0] = vs_viaf(u, mm); VIA_MASKE[0] = masken.get(u)
                    if not vs_route(u, 1): okk = False
                    VIA_MASKE[0] = None
                    if okk and not k1(u) < k0(u) - VS_EPS: grund = 'kein eigener Gewinn'; break
                if not okk: grund = 'unvollstaendig'; break
                if sum(vs_vias(w) for w in G) > sum(wert[w][0] for w in G): grund = 'Vias der Gruppe steigen'; break
        elif not okk: grund = 'unvollstaendig'
        def hart_schlecht(): return [w for w in G if vs_hart(U[w]['net'], U[w]['tracks'], U[w]['vias']) > vs_hart(*alt[w]) + 0.05]
        if frei and okk:



            neu_z = list(G) if VS_REIN else hart_schlecht()
            if neu_z:
                PF[0] = VS_HART
                for w in neu_z:
                    if VS_RUECK > 0 and vs_schutz(w) and w not in masken: masken[w] = vs_maske(alt[w][0], alt[w][2])
                    if args.get('vs_aeste') != '1' or vs_schutz(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm); VIA_MASKE[0] = masken.get(w)
                    if not vs_route(w, 1 if w == u else 2): okk = False
                    VIA_MASKE[0] = None
                if not okk: grund = 'unvollstaendig'
                else:
                    cfg = konflikte_gruppe(G, X, set(G)); verlauf.append(len(cfg))
                    if cfg: frei = False; grund = 'Konflikte nach dem Reinzeichnen'
                    elif hart_schlecht(): frei = False; grund = 'Bahn laege in gesperrtem Bereich (Loetstopp, Kante, Bohrung)'
        erg = None
        if frei and okk:
            V0 = sum(wert[w][0] for w in G); V1 = sum(vs_vias(w) for w in G)
            L0 = sum(wert[w][1] for w in G); L1 = sum(u_len(w) for w in G)
            M1 = {w: u_mass(w) for w in G}
            K1 = {w: u_gew(w) * (M1[w] + u_vgew(w) * vs_vias(w) + (VS_KNICK * u_knicke(w) if VS_KNICK else 0.0)) for w in G}
            bilanz = sum(K1[w] - k0(w) for w in G)
            if V1 > V0: grund = 'Vias der Gruppe steigen (%d -> %d)' % (V0, V1)
            elif sum(u_fehl(w) for w in G) > sum(wert[w][2] for w in G): grund = 'Via-Gruppe unvollstaendig'
            elif any(vs_schutz(w) and vs_vias(w) < wert[w][0] for w in G): grund = 'geschuetztes Netz verloere ein Via'
            elif any(vs_schutz(w) and not (vs_pad_ok(wert[w][4], vs_padabst(w)) and vs_pad_vias_da(alt[w][0], alt[w][2], U[w]['vias'])) for w in G):
                grund = 'Via eines geschuetzten Netzes wandert vom Pad weg'
            elif any(vs_schutz(w) and M1[w] > 1.05 * wert[w][3] + VS_TOL for w in G[1:]): grund = 'Stromnetz bekaeme mehr Widerstand'
            elif any(u_gew(w) > gu + 1e-9 and (vs_vias(w) > wert[w][0] or K1[w] > k0(w) + u_gew(w) * VS_TOL) for w in G[1:]):
                grund = 'wichtigeres Netz wuerde schlechter'
            elif not VS_NACHBAR_VIAS and any(vs_vias(w) > wert[w][0] for w in G[1:]): grund = 'Nachbar bekaeme mehr Vias'
            elif not bilanz < -VS_EPS: grund = 'Bilanz %+.1f' % bilanz
            else:
                gesch = [w for w in G[1:] if U[w]['tracks'] != alt[w][1] or U[w]['vias'] != alt[w][2]]
                neu_v = {(v['x'], v['y']) for v in U[u]['vias']}
                erg = dict(netz=u, pf=pfw, via_mm=mm, gruppe=G[1:], geschoben=gesch, vias=[V0, V1], laenge=[round(L0, 2), round(L1, 2)],
                           bilanz=round(bilanz, 2), verhaeltnis=round(-bilanz / (1 + sum(u_gew(w) for w in gesch)), 3), runden=verlauf,
                           weg=[[v['x'], v['y']] for v in alt[u][2] if not v.get('gr') and not v.get('rv') and (v['x'], v['y']) not in neu_v][:6],
                           je={w: [wert[w][0], vs_vias(w), round(wert[w][1], 2), round(u_len(w), 2), vs_versatz(alt[w][1], U[w]['tracks'])] for w in G},
                           _neu={w: (U[w]['net'], U[w]['tracks'], U[w]['vias']) for w in G}, _alt=alt)
        for c in hc: hist[c] = 0.0
        for w in G:
            if w in U: del_unit(w)
        for w in G: add_unit(w, *alt[w])
        return erg, len(G), verlauf, grund
    _vs_fehler = False
    try:
        for w in [w for w in U if vs_ist_strom(w) and not w.startswith(('FIX#', 'HEFT#'))]: u_mass0(w)
        X = gruppen_index([]); _vs_aus = False; _vs_boxen = None
        for rnd in range(int(args.get('vs_runden', '2'))):
            cand = [(grp(u)[0], -vs_potential(u), u) for u in units if u in U and not u.startswith('GND#') and not vs_fest(u)
                    and (not vs_schutz(u) or (VS_STROM and vs_art(u) != 'gnd'))]
            cand = [u for _g, p_, u in sorted(cand) if -p_ >= VS_MIN]
            if _vs_boxen is not None:
                def _nahe_box(u):
                    b = unit_box(u); pb = [D['pads'][k]['bbox'] for k in bynet[u]]
                    b = (min([q[0] for q in pb] + ([b[0]] if b else [])), min([q[1] for q in pb] + ([b[1]] if b else [])),
                         max([q[2] for q in pb] + ([b[2]] if b else [])), max([q[3] for q in pb] + ([b[3]] if b else [])))
                    return any(not (b[0] > q[2] + 1.5 or b[2] < q[0] - 1.5 or b[1] > q[3] + 1.5 or b[3] < q[1] - 1.5) for q in _vs_boxen)
                cand = [u for u in cand if _nahe_box(u)]
            boxen = []; n_ok = 0; dv = 0; dl = 0.0; db = 0.0; n_vers = 0
            for u in cand:
                if _vs_arb[0] > VS_ARBEIT: _vs_aus = 'Aufwand verbraucht'; break
                if (not BUDGET_ARBEIT and time.process_time() - _vs_c0 > VS_BUDGET) or zeit_knapp(0.9): _vs_aus = 'Zeitanteil verbraucht (Ergebnis haengt damit von der Rechenzeit ab)'; break
                pot = vs_potential(u); beste = None
                for pfw, mm in VS_VAR:
                    erg, ng, verlauf, grund = vs_versuch(u, pfw, mm, X); n_vers += 1
                    if args.get('vs_log') == '1' and erg is None and (ng > 1 or grund != 'kein eigener Gewinn'):
                        print('   kein Gewinn   %-24s PF %g, Gruppe %d, Konflikte je Runde %s: %s' % (u, pfw, ng, verlauf, grund), flush=True)
                    if erg is None: continue
                    if beste is None or erg['verhaeltnis'] > beste['verhaeltnis']: beste = erg
                    if -erg['bilanz'] >= 0.9 * pot: break
                if beste is None: continue
                neu_ = beste.pop('_neu'); alt_ = beste.pop('_alt'); _nb = len(boxen)
                for w in neu_: del_unit(w)
                for w in neu_: add_unit(w, *neu_[w])
                beste['runde'] = rnd + 1; beste['potential'] = round(pot, 2); VS_LOG.append(beste)
                n_ok += 1; dv += beste['vias'][0] - beste['vias'][1]; dl += beste['laenge'][1] - beste['laenge'][0]; db += beste['bilanz']
                for w in neu_:
                    ta = {(t['layer'], tuple(map(tuple, t['pts']))) for t in alt_[w][1]}; tn = {(t['layer'], tuple(map(tuple, t['pts']))) for t in neu_[w][1]}
                    for _l, pts in sorted(ta ^ tn): boxen.append((min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts)))
                    va = {(v['x'], v['y']) for v in alt_[w][2]}; vn = {(v['x'], v['y']) for v in neu_[w][2]}
                    for x_, y_ in sorted(va ^ vn): boxen.append((x_, y_, x_, y_))
                for w in neu_: _vs_m.pop(w, None)
                for w in [w for w in list(_vs_m) if w not in neu_]:
                    b = unit_box(w)
                    if b and any(not (b[0] > q[2] + 2.0 or b[2] < q[0] - 2.0 or b[1] > q[3] + 2.0 or b[3] < q[1] - 2.0) for q in boxen[_nb:]): _vs_m.pop(w)
                for w in [w for w in U if vs_ist_strom(w) and not w.startswith(('FIX#', 'HEFT#'))]: u_mass0(w)
                X = gruppen_index([])
                print('   %-13s %-24s Vias %d -> %d, Laenge %.1f -> %.1f mm, Bilanz %+.1f (Potential %.1f), PF %g, geschoben %s, Via weg bei %s' % (
                    'Via gespart' if beste['vias'][1] < beste['vias'][0] else 'kuerzer', u, beste['vias'][0], beste['vias'][1], beste['laenge'][0],
                    beste['laenge'][1], beste['bilanz'], pot, beste['pf'],
                    ['%s (%s mm)' % (w, beste['je'][w][4]) for w in beste['geschoben']], beste['weg']), flush=True)
            print('Vias sparen Runde %d: %d Kandidaten, %d Versuche, %d uebernommen, %d Vias weniger, Laenge %+.1f mm, Bilanz %+.1f (%.0fs)%s' % (
                rnd + 1, len(cand), n_vers, n_ok, dv, dl, db, time.time() - t4, ', ' + _vs_aus if _vs_aus else ''), flush=True)
            if not n_ok or _vs_aus: break
            _vs_boxen = boxen
    except Exception:
        if VS_PRUEF: raise
        import traceback
        traceback.print_exc(file=sys.stdout); _vs_fehler = True
    VIAF[0] = 1.0; VIA_MASKE[0] = None; PF[0] = PFs; hist[:] = hsave; _KIDX[0] = None
    if _vs_fehler or exact_conflicts():
        print('Vias sparen: %s -> alles zurueck auf den Stand davor' % ('Fehler im Nachlauf (siehe oben)' if _vs_fehler else 'Schlusspruefung findet Konflikte %s' % sorted(set((a, b) for (_l, _d, a, b, _x, _y) in PAIRS))[:6]), flush=True)
        for w in list(U): del_unit(w)
        for w, s_ in _vs_alt.items(): add_unit(w, *s_)
        VS_LOG.clear()



    for w in _vs_alt:
        if w in U: U[w] = U.pop(w)
    kz1 = kennzahlen_jetzt()
    print('Vias sparen: Vias %d -> %d (ohne Flaechennetze %d -> %d), Knicke %d -> %d, Laenge %.1f -> %.1f mm, CPU %.0f s (Kandidat %.0f, Partner %.0f, Pruefung %.0f), Aufwand %d von %d Anschluessen' % (
        kz0['vias'], kz1['vias'], kz0['vias_ohne_flaeche'], kz1['vias_ohne_flaeche'], kz0['knicke'], kz1['knicke'],
        kz0['laenge'], kz1['laenge'], time.process_time() - _vs_c0, _vs_arb[1], _vs_arb[2], _vs_arb[3], _vs_arb[0], VS_ARBEIT), flush=True)



R19_FORM = args.get('r19_form', '0') == '1' and KL is not None
R19_FORM_STAT = dict(geprueft=0, verworfen=0, beispiele=[])
def r19_form_ok(alt, neu):
    if not R19_FORM or (KL['netze'].get(alt['net']) or {}).get('art') not in ('signal', 'analog', 'schnell'): return True
    from analyse.r19 import form_ok, formmasse
    R19_FORM_STAT['geprueft'] += 1
    if form_ok(alt, neu): return True
    R19_FORM_STAT['verworfen'] += 1
    if len(R19_FORM_STAT['beispiele']) < 12:
        R19_FORM_STAT['beispiele'].append(dict(net=alt['net'], alt=formmasse(alt), neu=formmasse(neu)))
    return False
if HYPO_SPAET and not todo and not zeit_knapp():
    _HYPO_AN[0] = True; t2 = time.time(); PFs = PF[0]; PF[0] = 1e3; hist[:] = 0; _KIDX[0] = None; _nu = 0
    _htol = float(args.get('hypo_tol', '0.01'))
    _rest = sorted([u for u in units if u in U and not paar_einheit(u) and not (u.startswith('GND#') and int(u[4:]) in EPM)], key=kurz_key); _warum = {}
    for _rd in range(2):
        _bleibt = []
        for u in _rest:
            if zeit_knapp(0.9): break
            old = copy.deepcopy(U[u]); L0 = ulen(u); _v0 = n_sv(U[u]); _f0 = vg_fehl(u) if VG else 0
            del_unit(u)
            okk = route_unit(u)
            if okk and ulen(u) <= L0 * (1 + _htol) + 0.2 and n_sv(U[u]) <= _v0 and (not VG or vg_fehl(u) <= _f0) and einheit_konfliktfrei(u) and r19_form_ok(old, U[u]):
                _nu += 1; kidx_geaendert(u); continue
            _warum[u] = 'unvollstaendig' if not okk else ('laenger %.1f statt %.1f' % (ulen(u), L0) if ulen(u) > L0 * (1 + _htol) + 0.2 else
                         ('Vias %d statt %d' % (n_sv(U[u]), _v0) if n_sv(U[u]) > _v0 else 'Via-Gruppe oder Konflikt'))
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); kidx_zurueck(u); _bleibt.append(u)
        _rest = _bleibt
        if not _rest: break
    PF[0] = PFs
    print('Umformen: %d Einheiten neu geformt, %d in alter Form (%.0fs)' % (_nu, len(_rest), time.time() - t2), {u: _warum[u] for u in _rest if not u.startswith('GND#')}, flush=True)
if EINZELTEST and ET_ORT != 'vor' and KL is not None and not todo and not zeit_knapp(): einzeltest()
if PAAR_NETZE and not todo and not zeit_knapp():
    print('Differenzpaare (R29) gekoppelt: %d' % paar_koppeln(), flush=True)
if RUECK > 0 and not todo and not zeit_knapp():
    _rvn = rv_nachlauf(); print('Rueck-Vias (Q2) nach der Verhandlung: %d nachgetragen, %d ueberfluessige entfernt' % _rvn, flush=True)
if KL is not None and args.get('fluss', '1') == '1':
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from gemeinsam.fluss import analyse as _fl_analyse, zielbreite as _fl_ziel
    _nf = 0
    for u, r in list(U.items()):
        if u.startswith(('GND#', 'FIX#', 'HEFT#')): continue
        k = KL['netze'].get(r['net'], {})
        if k.get('art') not in (('strom', 'versorgung', 'sw') if args.get('strom_fix') == '1' else ('strom', 'versorgung')) or not k.get('fluss'): continue
        info = _fl_analyse([t for t in r['tracks']], r['vias'], [D['pads'][i] for i in bynet[r['net']]], k['fluss'], endpunkte=args.get('strom_fix') == '1')
        if not info: continue
        C_ = cls(r['net'])
        FLUSS_W[u] = {sid: _fl_ziel(a_, C_['wmin'], C_['w'], float(args.get('fluss_exp', '0.5'))) for sid, a_ in info['anteil'].items()}
        if strom_modell(r['net']):
            FLUSS_W[u] = {sid: max(BAHN_MIN,min(HALS_W,C_['wmin']),info['gesamt'] * a_ / (STROM_J * STROM_T[r['tracks'][sid[0]]['layer']])) for sid,a_ in info['anteil'].items()}
        FLUSS_A[u] = info['anteil']
        _nf += 1
    print('Flussprinzip: Zielbreiten fuer %d Stromnetze' % _nf, flush=True)





FEINSCHUB_LOG = []
def via_feinschub():
    from analyse.bewertung import bewerte as _vf_note, unverbunden as _vf_unverbunden
    from analyse.topologie import topologie as _vf_topo
    from analyse.kopplung import kopplung as _vf_kopp
    def snapshot():
        tracks, vias = ausgabe_geometrie()
        return dict(tracks=tracks, vias=vias, conflicts={}, failed=[], optionen=args,
                    gndlagen=GNDL, flaechennetze=sorted(FLNETZE))
    def tore(R):
        e = _vf_note(D, R, KL, v3=True); tp = _vf_topo(R, D)
        gueltig = e['diagnose'] and e['konflikte'] == 0 and e['offen'] == 0 and not _vf_unverbunden(D, R, set(KL['gnd']) | FLNETZE)
        return gueltig, e['note_v3'], _vf_kopp(R, KL)['summe'], tuple(tp[k] - tp.get(k + '_pad', 0) for k in ('stern', 'spitz')), tp['parallel_rest_mm'], tp['kelvin_nah_mm']
    def besser(a, b): return b[0] and all(y <= x + 1e-7 for x, y in zip((a[1], a[2], *a[3], a[4], a[5]), (b[1], b[2], *b[3], b[4], b[5])))
    start = time.process_time(); versuche = 0; reihe = list(U); _KIDX[0] = None; verschoben = set()
    power = [u for u in U if not u.startswith(('FIX#', 'HEFT#', 'GND#')) and (KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung')]
    for pu in power:
        if zeit_knapp() or (not BUDGET_ARBEIT and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
        C = cls(U[pu]['net'])
        eng = [(LI[t['layer']], _LS((a, b)), max(C['wmin'], FLUSS_W.get(pu, {}).get((ti, si), C['w'])))
               for ti, t in enumerate(U[pu]['tracks']) if not t.get('sig') and not t.get('steg')
               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if w < C['w'] - 1e-6 and math.dist(a, b) > 0.3]
        for u in list(U):
            r = U[u]; k = KL['netze'].get(r['net']) or {}
            if u.startswith(('FIX#', 'HEFT#', 'GND#')) or r['net'] in GNDS or k.get('quarz') or k.get('art') in ('strom', 'versorgung', 'sw', 'schnell'): continue
            for vi, v in enumerate(list(r['vias'])):
                if (u, vi) in verschoben or v.get('gr') or v.get('rv') or v.get('n'): continue
                A = (v['x'], v['y'])
                if any(_PT(A).distance(_LS(poly + [poly[0]])) < 0.5 or _PT(A).within(Polygon(poly))
                       for pi in bynet_g.get(r['net'], []) for polys in D['pads'][pi].get('polys', {}).values() for poly in polys): continue
                near = [(li, ln, goal) for li, ln, goal in eng if ln.distance(_PT(A)) < goal / 2 + VIA_D / 2 + CLR + 0.3]
                if not near: continue
                inc = [(ti, e) for ti, t in enumerate(r['tracks']) for e in (0, -1) if math.dist(t['pts'][e], A) < 1e-6]
                if len(inc) < 2 or any(_LS((a, b)).distance(_PT(A)) < VIA_D / 2 + w / 2
                                      for tj, t in enumerate(r['tracks']) for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))
                                      if ((tj, 0) not in inc and (tj, -1) not in inc) or ((tj, 0) in inc and si > 0 and (tj, -1) not in inc) or ((tj, -1) in inc and si < len(t['w']) - 1 and (tj, 0) not in inc)): continue
                if any(r['tracks'][ti].get('abg') or r['tracks'][ti].get('steg') for ti, e in inc): continue
                old = copy.deepcopy(r); pwold = copy.deepcopy(U[pu]); before = tore(snapshot())
                for ti, e in inc:
                    q = old['tracks'][ti]['pts'][1 if e == 0 else -2]; d = math.dist(A, q)
                    for step in (0.05, 0.1, 0.2, 0.3):
                        if step >= d - 0.05 or versuche >= int(args.get('vf_versuche', '60')) or zeit_knapp() or (not BUDGET_ARBEIT and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
                        versuche += 1; B = (A[0] + (q[0] - A[0]) * step / d, A[1] + (q[1] - A[1]) * step / d)
                        del_unit(u); net = old['net']; Cn = cls(net); ch, vh, cs, dd, rng, vb, cc, cf, vbf = karten(net, (B[0]-2,B[1]-2,B[0]+2,B[1]+2), Cn['w'])
                        ix, iy = int((B[0]-X0)/RES), int((B[1]-Y0)/RES); neu = copy.deepcopy(old); ok = 0 <= ix < NX and 0 <= iy < NY and vh[iy, ix] and not vb[iy, ix]
                        for tj, end in inc:
                            t = neu['tracks'][tj]; P = t['pts'] if end == 0 else t['pts'][::-1]; W = t['w'] if end == 0 else t['w'][::-1]
                            Q = lokal_anschluss([B, P[1]], B, P[1], cc[LI[t['layer']]], LI[t['layer']], net, W[0]) if ok else None
                            if Q is None: ok = False; break
                            Q += P[2:]; ww = [W[0]] * (len(Q) - len(P) + 1) + W[1:]
                            t['pts'], t['w'] = (Q, ww) if end == 0 else (Q[::-1], ww[::-1])
                        if ok:
                            neu['vias'][vi] = dict(v, x=B[0], y=B[1]); add_unit(u, net, neu['tracks'], neu['vias']); _KIDX[0] = None; ok = einheit_konfliktfrei(u)
                            if ok:
                                widen_unit(pu, C); _KIDX[0] = None; after = tore(snapshot()); ok = einheit_konfliktfrei(pu) and besser(before, after) and strom_widerstand(pu) < strom_widerstand_alt(pwold, pu) - 1e-9
                        if ok:
                            verschoben.add((u, vi)); FEINSCHUB_LOG.append(dict(netz=u, strom=pu, alt=A, neu=B, versatz_mm=step)); kidx_geaendert(u); kidx_geaendert(pu); break
                        if u in U: del_unit(u)
                        add_unit(u, old['net'], old['tracks'], old['vias'])
                        del_unit(pu); add_unit(pu, pwold['net'], pwold['tracks'], pwold['vias']); _KIDX[0] = None
                    if FEINSCHUB_LOG and FEINSCHUB_LOG[-1]['alt'] == A: break
                r = U[u]
    for w in reihe: U[w] = U.pop(w)
    _KIDX[0] = None
    print('Via-Feinschub: %d uebernommen, %d Versuche, CPU %.1f s' % (len(FEINSCHUB_LOG), versuche, time.process_time()-start), flush=True)

def via_dichte_nachlauf():
    ""
    global VIA_DICHTE
    from analyse.via_dichte import messen as dichte, kurz, schutznetze, via_geschuetzt
    from analyse.bewertung import bewerte
    from analyse.regelkennzahlen import annahme as regeltor
    from analyse.vertrag import vergleich as vertragstor
    from analyse.minimax import annahme as minimaxtor
    from analyse.wackel import messen as wackel
    start = time.process_time(); reihe = list(U); kosten = VIA_DICHTE; pf = PF[0]
    def knapp():
        return zeit_knapp(0.95) or (not BUDGET_ARBEIT and time.process_time()-start >= float(args.get('via_dichte_nach_cpu', '45')))
    def snapshot():
        tr, vi = ausgabe_geometrie()
        return ausgabe_daten(tr, vi, {}, fehlend=[])
    def qualitaet(R):
        return bewerte(D, R, KL, v3=True, **{k: str(args.get(k, '0')).lower() in ('1', 'true')
                       for k in ('si_note', 'si_messen', 'laengentor')}), wackel(D, R, gnd=KL.get('gnd') or [])
    try:
        if knapp():
            VD_NACH_LOG.update(grund='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        base = snapshot(); vd0 = dichte(D, base, fenster=VD_FENSTER, schwelle=VD_SCHWELLE)
        if not vd0['fenster_ueber'] and vd0['spitze_vias_mm2'] <= VD_SCHWELLE:
            VD_NACH_LOG.update(grund='unter_schwelle', cpu_s=round(time.process_time()-start, 3)); return
        if knapp():
            VD_NACH_LOG.update(grund='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        e0, w0 = qualitaet(base)
        protected = schutznetze(KL, base, IMP_TAB)
        if not e0['gueltig']:
            VD_NACH_LOG.update(grund='bewertung_ungueltig', cpu_s=round(time.process_time()-start, 3)); return
    except Exception as exc:
        VD_NACH_LOG.update(grund='diagnosefehler', fehler=[repr(exc)[:160]], cpu_s=round(time.process_time()-start, 3)); return
    cand = []
    for u in sorted(U):
        r = U[u]
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#', 'GND#')): continue
        if not r['vias'] or any(via_geschuetzt(dict(v, net=r['net']), KL, protected) for v in r['vias']): continue
        score = sum(float(_vd_kosten()[int((v['y']-Y0)/RES), int((v['x']-X0)/RES)])
                    for v in r['vias'] if 0 <= int((v['x']-X0)/RES) < NX and 0 <= int((v['y']-Y0)/RES) < NY)
        if score: cand.append((-score, u))
    try:
        VIA_DICHTE = True; PF[0] = 1e3; _KIDX[0] = None
        for _, u in sorted(cand)[:int(args.get('via_dichte_nach_max', '8'))]:
            if knapp(): break
            old = copy.deepcopy(U[u]); saved_fail = set(fail)
            saved_unreachable = copy.deepcopy(UNERREICHBAR)
            VD_NACH_LOG['versuche'] += 1
            accepted = False
            try:
                del_unit(u)
                if route_unit(u) and not knapp() and einheit_konfliktfrei(u) and not exact_conflicts():
                    neu = snapshot(); q = kurz(neu, KL, VD_FENSTER, VD_SCHWELLE)
                    if q['spitze_vias_mm2'] <= vd0['spitze_vias_mm2'] and q['vias'] <= vd0['vias']:
                        if knapp(): continue
                        vd1 = dichte(D, neu, fenster=VD_FENSTER, schwelle=VD_SCHWELLE)
                        if (vd1['ueberlast_integral_mm2'] < vd0['ueberlast_integral_mm2'] - 1e-7
                                and vd1['fenster_ueber'] <= vd0['fenster_ueber']):
                            if knapp(): continue
                            e1, w1 = qualitaet(neu)
                            accepted = e1['gueltig'] and regeltor(e0.get('regeln'), e1.get('regeln'))['nicht_schlechter']
                            if accepted and any(e.get('tor') or (e.get('regeln') or {}).get('messversion') for e in (e0, e1)):
                                accepted = vertragstor(e0, e1)['nicht_schlechter']
                            if accepted and any(e.get('bewertungsziel') == 'minimax' for e in (e0, e1)):
                                accepted = minimaxtor(e0, e1)['nicht_schlechter']
                            accepted = accepted and w1['zuege'] <= w0['zuege']


                            accepted = accepted and not knapp()
                            if accepted:
                                VD_NACH_LOG['uebernommen'] += 1
                                VD_NACH_LOG['zuege'].append(dict(netz=u, integral=[vd0['ueberlast_integral_mm2'], vd1['ueberlast_integral_mm2']]))
                                vd0, e0, w0 = vd1, e1, w1
            except Exception as exc:
                VD_NACH_LOG.setdefault('fehler', []).append(repr(exc)[:160])
            finally:
                if not accepted:
                    if u in U: del_unit(u)
                    add_unit(u, old['net'], old['tracks'], old['vias'])
                    fail.clear(); fail.update(saved_fail)
                    UNERREICHBAR.clear(); UNERREICHBAR.update(saved_unreachable)
                _KIDX[0] = None
    finally:
        VIA_DICHTE = kosten; PF[0] = pf
        for u in reihe: U[u] = U.pop(u)
        _KIDX[0] = None
        VD_NACH_LOG['cpu_s'] = round(time.process_time()-start, 3)
    print('Via-Dichte-Nachlauf:', VD_NACH_LOG, flush=True)


def strom_widerstand_alt(r, u):
    now = U[u]; U[u] = r
    try: return strom_widerstand(u)
    finally: U[u] = now

t1 = time.time()
PRE = {u: copy.deepcopy(r) for u, r in U.items()}
BREIT_DICHT_BERICHT = None
if BREIT_DICHT:
    _KIDX[0] = None
    _bd_vor = exact_conflicts()
    _bd_vor_n = len(PAIRS)

if 'pre' in args:
    _t, _v = [], []
    for u, r in U.items():
        for t in r['tracks']: _t.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w']))
        for v in r['vias']: _v.append(dict(net=v.get('net') or r['net'], x=v['x'], y=v['y']))
    if args.get('kurzschluss_schutz', '0') == '1': _t, _v = ausgabe_geometrie()
    json.dump(ausgabe_daten(_t, _v, {}) if args.get('kurzschluss_schutz', '0') == '1' else dict(tracks=_t, vias=_v), open(args['pre'], 'w')); print('vor Verbreitern gespeichert:', args['pre'])
RHO = 1.72e-5
def t_cu(L): return 0.035 if L in ('F', 'B') else 0.0175
def chan_R(c):
    R = 0.0
    for u, r in U.items():
        k = r['net'].split('/')[-1]
        if k in ('K_' + c, 'S_' + c, 'M_' + c):
            for t in r['tracks']:
                if t.get('sig'): continue
                for a, b2, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): R += RHO * math.dist(a, b2) / (w * t_cu(t['layer']))
            R += 0.0024 * len(r['vias'])
    return R
CH = sorted({r['net'].split('/')[-1][2:] for r in U.values() if r['net'].split('/')[-1][:2] in ('K_', 'S_', 'M_')})
WMAX = {0: 0.8, 1: 0.35, 2: 1.5, 3: 1.2, 4: 1.0, 5: 0.5, 6: 0.35} if args.get('breiter') == '1' else {}
def ucls(u):
    if u.startswith('GND#'): return dict(w=max(0.35, BAHN_MIN), wmin=max(0.15, BAHN_MIN))
    if u in PLANE_GROUPS:return dict(w=max(0.35,BAHN_MIN),wmin=max(0.15,BAHN_MIN),prio=6.5)
    C = cls(u[4:] if u.startswith('FIX#') else u)
    return dict(C, w=max(C['w'], WMAX.get(C['prio'], 0)))
if args.get('fair') in ('1', 'alle'):

    STEP = 0.05
    pw = [u for u in U if not u.startswith(('GND#', 'FIX#', 'HEFT#')) and u not in FIXGND and u not in PLANE_GROUPS and
          ((KL['netze'].get(U[u]['net']) or {}).get('art') in ('strom', 'versorgung', 'sw') if KL and (args.get('fair_all') == '1' or args.get('fair') == 'alle') else ucls(u)['prio'] < 8)]
    satt = set()
    for rnd in range(60):
        Rc = {c: chan_R(c) for c in CH}
        Rn = {u: strom_widerstand(u) for u in pw} if KL and (args.get('fair_all') == '1' or args.get('fair') == 'alle') else None
        def key(u):
            if Rn is not None: return (0, -Rn[u], ucls(u)['prio'])
            k = U[u]['net'].split('/')[-1]
            return (0, -Rc[k[2:]]) if k[:2] in ('K_', 'S_', 'M_') else (1, ucls(u)['prio'])
        ch = 0
        if zeit_knapp(0.95): break
        for u in sorted(pw, key=key):
            if u in satt:
                U[u] = U.pop(u); continue
            C = ucls(u); before = [list(t['w']) for t in U[u]['tracks']]
            widen_unit(u, dict(C, w_step=STEP))
            if [list(t['w']) for t in U[u]['tracks']] != before: ch += 1
            else: satt.add(u)
        if not ch: break
    print('Fair verbreitert in %d Runden' % (rnd + 1), {c: round(chan_R(c) * 1000, 1) for c in CH}, 'mOhm', flush=True)
    rest = [u for u in U if u not in pw and not u.startswith(('FIX#', 'HEFT#')) and u not in FIXGND]
else:
    rest = [u for u in U if not u.startswith(('FIX#', 'HEFT#')) and u not in FIXGND]
for u in sorted(rest, key=lambda u: (ucls(u)['prio'] if not u.startswith('GND#') else 6.5)):
    if zeit_knapp(0.95): break
    widen_unit(u, ucls(u))
print('Verbreitert in %.0fs' % (time.time() - t1))

for rnd in range(0 if BREIT_DICHT else 6):
    cf = exact_conflicts()
    if not cf: break
    print('   nach Verbreitern Konflikte', len(PAIRS), sorted(PAIRS, key=lambda x: x[1])[:4], flush=True)
    _zuletzt = {}
    for _pi, (lay, dist, ua, ub, x, y) in enumerate(PAIRS):
        if lay == '*': continue
        for _ui, u in enumerate((ua, ub)):
            if paar_einheit(u) or u not in U or u not in PRE or u.startswith(('FIX#', 'HEFT#')): continue
            r = U[u]; pre = PRE[u]; rad = 0.6 + 0.4 * rnd
            if KL and args.get('strom_fix') == '1':
                from gemeinsam.strombahn import korrigiere
                li = LI[lay]
                hind = [(g, DRC_CLR) for other, rr in U.items() if rr['net'] != r['net'] and not _fixpaar(u, other)
                        for l2, g, _, _ in _unit_geo(other, rr) if l2 == li]
                hind += [(g, DRC_PADC) for pn, g, _ in _PADB[li] if pn != r['net']]
                if korrigiere(r['tracks'], pre['tracks'], lay, hind): _zuletzt[u] = (_pi, _ui)
                continue
            chg = False
            for t, t0 in zip(r['tracks'], pre['tracks']):
                if t['layer'] != lay: continue
                ws = list(t['w'])
                for k, (a, b2) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if _LS([a, b2]).distance(_PT(x, y)) < rad and ws[k] > t0['w'][k]:
                        ws[k] = t0['w'][k] if rnd < 4 else min(t0['w'][k], ws[k])
                        chg = True
                t['w'] = ws
            if chg or rnd >= 4: _zuletzt[u] = (_pi, _ui)
    for u in sorted(_zuletzt, key=_zuletzt.get):
        r = U[u]
        tr = r['tracks'] if rnd < 5 or (KL and args.get('strom_fix') == '1') else PRE[u]['tracks']
        del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    if BGA_PLAN and not _zuletzt:


        break
if BREIT_DICHT:
    _bd_nach = exact_conflicts()
    _bd_unter = []
    for u, r in U.items():
        if u.startswith(('FIX#', 'HEFT#', 'GND#')): continue
        if KL is None or (KL['netze'].get(r['net']) or {}).get('art') not in ('strom', 'versorgung', 'sw'): continue
        for ti, t in enumerate(r['tracks']):
            if t.get('sig'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                soll = FLUSS_W.get(u, {}).get((ti, si), ucls(u)['w'])
                if not strom_modell(r['net']): soll *= f_innen(r['net'], LI[t['layer']])
                if w + 1e-7 < soll:
                    _bd_unter.append(dict(netz=r['net'], layer=t['layer'], pts=[a,b], breite=w,
                                          soll=soll, laenge=math.dist(a,b)))
    BREIT_DICHT_BERICHT = dict(phase='direkt_nach_verbreitern',
        konfliktpaare_vor=_bd_vor_n, konfliktpaare_nach=len(PAIRS),
        konflikteinheiten_vor=len(_bd_vor), konflikteinheiten_nach=len(_bd_nach),
        unter_zielbreite=_bd_unter)
    print('Dicht verbreitert: Paare %d -> %d, Einheiten %d -> %d; %d Stromstuecke unter Zielbreite' % (
        _bd_vor_n, len(PAIRS), len(_bd_vor), len(_bd_nach), len(_bd_unter)), flush=True)
if R32:

    for u in list(U):
        r = U[u]
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#')) or r['net'] in IMP_TAB or (KL['netze'].get(r['net']) or {}).get('art') not in ('signal', 'schnell', 'analog'): continue
        widths = [w for t in r['tracks'] for w in t['w']]
        ziel = max(BAHN_MIN, min(widths)) if widths else BAHN_MIN
        if widths and any(abs(w - ziel) > 1e-9 for w in widths):

            tr = [dict(t, w=[ziel] * len(t['w'])) for t in r['tracks']]
            del_unit(u); add_unit(u, r['net'], tr, r['vias'])

if RUHIG:
    _bw = [0, 0, 0]
    for u in list(U):
        if paar_einheit(u) or u.startswith(('FIX#', 'HEFT#')): U[u] = U.pop(u); continue
        _bw = [a + b for a, b in zip(_bw, breite_beruhigen(u, ucls(u)))]
    print('Breitenprofil beruhigt (ruhig_r=%g): %d Bahnen schmaler, Breitenwechsel %d -> %d' % (RUHIG_R, _bw[2], _bw[0], _bw[1]), flush=True)

if PADMITTE:
    _ns = stoesse_reparieren() if args.get('stoss_fix', '1') == '1' else 0
    if _ns: print('Stoesse (R24): %d Bahnenden am Schluss exakt auf die Mittellinie der eigenen Bahn gesetzt' % _ns, flush=True)
    _nk = kollinear_zusammen() if args.get('kollinear', '1') == '1' else 0
    print('Stoesse (R24): %d kollineare Stuetzpunkte zusammengefasst; beim Verlegen (alle Durchgaenge)' % _nk, STOSS_STAT, flush=True)


if args.get('via_feinschub') == '1' and KL and not fail and not exact_conflicts() and not zeit_knapp(): via_feinschub()
LAENGEN_BERICHT = None
if (args.get('laengen', '0') == '1' and SI_FELDER or PAAR_NETZE) and (PAAR_SUCHRAUM or not fail and not exact_conflicts()) and not zeit_knapp(0.95):
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if PAAR_NETZE: paar_koppeln()
    LAENGEN_BERICHT = laengen_abgleichen(nur_paare=args.get('laengen', '0') != '1')
    print('Laengenabgleich (Option): %d Maeander, %d Versuche' % (len(LAENGEN_BERICHT['zuege']), LAENGEN_BERICHT['versuche']), flush=True)

VD_NACH_LOG = dict(versuche=0, uebernommen=0, zuege=[], cpu_s=0.0)
if VD_NACH and not fail and not exact_conflicts() and not zeit_knapp(): via_dichte_nachlauf()

if STROM_ZUERST:


    for u,r in list(U.items()):
        if strom_modell(r['net']) and not u.startswith(('FIX#','HEFT#')) and not strom_abschluss(u,ucls(u)): fail.add(u)

REST_BERICHT = None
if args.get('restkonflikte', '0') == '1':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nachbearbeitung.restkonflikte import loese_vias
    REST_BERICHT = loese_vias(globals())
    print('R45: %d Via-Zuege angenommen, %.2f CPU-s' % (REST_BERICHT['uebernommen'], REST_BERICHT['cpu_s']), flush=True)


def pinzugang_regelwerte(net):
    ""
    from analyse.kopplung import kopplung, mengen
    from analyse.topologie import topologie
    tracks, vias = ausgabe_geometrie()
    own = [t for t in tracks if t['net'] == net]
    opfer, stoerer = mengen(KL)
    k = 0.
    if net in opfer or any(t.get('kelvin') for t in own):
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and t['net'] in stoerer])
        k += kopplung(r, KL, opfer={net}, stoerer=stoerer)['summe']
    if net in stoerer:
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and (t['net'] in opfer or t.get('kelvin'))])
        k += kopplung(r, KL, opfer=opfer - {net}, stoerer={net: stoerer[net]})['summe']
    r = dict(tracks=own, vias=[v for v in vias if v['net'] == net], clr=DRCC)
    t = topologie(r, dict(D, pads=[p for p in D['pads'] if p['net'] == net]))
    keys = ('parallel_rest_mm', 'stern', 'spitz', 'kelvin_nah_mm', 'abgriff_via_im_pad')
    return dict(kopplung=k, **{key:t[key] for key in keys},
                stern_frei=t['stern']-t['stern_pad'], spitz_frei=t['spitz']-t['spitz_pad'])


def pinzugang_regelannahme(vor, nach):
    ""
    return all(math.isfinite(nach[k]) and nach[k] <= v + 1e-6 for k,v in vor.items())

def pinzugang_nachlauf():
    ""
    global _LOOP_DEADLINE
    report = dict(reparatur=[], versuche=[], uebernommen=0, cpu_s=0., budget_s=0.)
    cpu_start = time.process_time()
    start = budget_uhr()
    budget = max(0., start / 9.)
    if BUDGET_ARBEIT: budget = min(budget, max(0., .90 * ARBEIT_MAX - arbeit_gesamt()))
    elif CPU_MAX > 0: budget = min(budget, max(0., .90 * CPU_MAX - such_cpu()))
    report['budget_arbeit' if BUDGET_ARBEIT else 'budget_s'] = budget

    if budget < .1: return report
    old_deadline = _LOOP_DEADLINE
    deadline = start + budget
    old_hist, old_pf, old_hart = hist.copy(), PF[0], HART_FREMD[0]
    old_active = _PZ_BUDGET_ACTIVE[0]
    try:
        hist[:] = 0; PF[0] = 1e3; HART_FREMD[0] = True
        _PZ_BUDGET_ACTIVE[0] = True; _LOOP_DEADLINE = deadline

        removed = []
        if PZ_REST:
            _loop_cpu_check()
            conflicts = exact_conflicts()
            tr, vi = ausgabe_geometrie()
            safe = r44_ausgabe(dict(tracks=tr, vias=vi, failed=sorted(fail), conflicts=conflicts))
            removed = (safe.get('kurzschluss_schutz') or {}).get('einheiten_entfernt', [])


            removed = [u for u in removed if u in U and u not in FIXED and not u.startswith(('FIX#','HEFT#'))]
            for u in removed:
                del_unit(u); fail.add(u)
            _KIDX[0] = None


            order = sorted(removed,key=lambda u:(-len(bynet.get(u,[])),u))
            if order:
                order = order[:1] + sorted(order[1:],key=lambda u:(u.startswith('GND#'),len(bynet.get(u,[])),cls(U[u]['net'] if u in U else u).get('prio',10),u))
            repair_end = start + budget * .55
            for index, u in enumerate(order):
                if budget_uhr() >= repair_end: break
                _LOOP_DEADLINE = min(repair_end,budget_uhr()+budget*(.20 if index==0 else .07))
                entry = dict(einheit=u, angenommen=False)
                local_before = {n:set(ps) for n,ps in PZ_LOKAL.items()}
                _PZ_PARTIAL[0] = None
                regeln0 = None
                try:
                    net = D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
                    regeln0 = pinzugang_regelwerte(net)
                    ok = route_unit(u); candidate = U.get(u)
                    valid = bool(candidate and (candidate['tracks'] or candidate['vias']))
                    valid = valid and _pz_geometry_ok(candidate['net'], candidate['tracks'], candidate['vias'], False, u)
                    _KIDX[0] = None
                    valid = valid and einheit_konfliktfrei(u)
                    if valid:
                        regeln1 = pinzugang_regelwerte(net)
                        entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                        valid = pinzugang_regelannahme(regeln0, regeln1)
                        if not valid: entry['grund'] = 'Topologie/Kopplung schlechter'
                    _loop_cpu_check()
                    if valid:
                        entry.update(angenommen=True, vollstaendig=bool(ok))
                        if ok: fail.discard(u)
                except (TimeoutError, _PZBudget):
                    entry['grund'] = 'CPU-Budget'


                    partial = _PZ_PARTIAL[0]
                    _LOOP_DEADLINE = repair_end
                    try:
                        _loop_cpu_check()
                        if partial and partial[0] == u and partial[2]:
                            add_unit(u,partial[1],partial[2],partial[3])
                            candidate = U[u]; _KIDX[0] = None
                            valid = _pz_geometry_ok(candidate['net'],candidate['tracks'],candidate['vias'],False,u)
                            valid = valid and einheit_konfliktfrei(u)
                            if valid and regeln0 is not None:
                                regeln1 = pinzugang_regelwerte(net)
                                entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                                valid = pinzugang_regelannahme(regeln0, regeln1)
                                if not valid: entry['grund'] = 'Topologie/Kopplung schlechter'
                            else: valid = False
                            _loop_cpu_check()
                            if valid: entry.update(angenommen=True,vollstaendig=False,teilbaum=True)
                    except Exception as error:
                        entry['pruefung'] = repr(error)[:180]
                except Exception as error:
                    entry['grund'] = 'Kandidat verworfen: ' + repr(error)[:180]
                finally:
                    _PZ_PARTIAL[0] = None
                    if not entry['angenommen']:
                        del_unit(u)
                        PZ_LOKAL.clear(); PZ_LOKAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    report['reparatur'].append(entry); _KIDX[0] = None
                if budget_uhr() >= repair_end: break
        _LOOP_DEADLINE = deadline


        if PZ_MINI and (removed or PZ_LOKAL):
            eligible = [u for u in units if u in U and u not in FIXED and u not in fail
                        and not u.startswith(('GND#','FIX#','HEFT#')) and not paar_einheit(u)
                        and u not in PAAR_NETZE and U[u]['net'] in PZ_BREITEN and net_mst(U[u]['net']) > .000001]
            for u in sorted(eligible, key=lambda u: (-(ulen(u)-.6*n_sv(U[u]))/net_mst(U[u]['net']), u))[:5]:
                _loop_cpu_check()
                old = copy.deepcopy(U[u]); l0 = sum(math.dist(a,b) for t in old['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:]))
                entry = dict(netz=old['net'], vor_mm=l0, q_vor=l0/net_mst(old['net']), angenommen=False)
                local_before = {n:set(ps) for n,ps in PZ_LOKAL.items()}
                try:
                    regeln0 = pinzugang_regelwerte(old['net'])
                    del_unit(u)
                    ok = route_unit(u); candidate = U.get(u)
                    l1 = sum(math.dist(a,b) for t in candidate['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) if candidate else float('inf')
                    entry['nach_mm'] = l1 if math.isfinite(l1) else None
                    ok = ok and l1 < l0-.01 and n_sv(candidate) <= n_sv(old)
                    ok = ok and min((w for t in candidate['tracks'] for w in t['w']),default=BAHN_MIN) >= min((w for t in old['tracks'] for w in t['w']),default=BAHN_MIN)-1e-9
                    ok = ok and _pz_geometry_ok(old['net'],candidate['tracks'],candidate['vias'],False,u)
                    _KIDX[0] = None
                    ok = ok and einheit_konfliktfrei(u)
                    if ok:
                        from analyse.bewertung import unverbunden as _pz_unverbunden
                        b = dict(D,pads=[p for p in D['pads'] if p['net']==old['net']])
                        r = dict(tracks=[dict(t,net=old['net']) for t in candidate['tracks']],
                                 vias=[dict(v,net=v.get('net') or old['net'],d=VIA_D,drill=VIA_DR,layers=LAYERS) for v in candidate['vias']])
                        ok = not _pz_unverbunden(b,r,set())
                    if ok:
                        regeln1 = pinzugang_regelwerte(old['net'])
                        entry.update(regeln_vor=regeln0, regeln_nach=regeln1)
                        ok = pinzugang_regelannahme(regeln0, regeln1)
                        if not ok: entry['grund'] = 'Topologie/Kopplung schlechter'
                    _loop_cpu_check()
                    if ok: entry['angenommen']=True; report['uebernommen']+=1
                except (TimeoutError, _PZBudget):
                    entry['grund']='CPU-Budget'
                except Exception as error:
                    entry['grund']='Kandidat verworfen: ' + repr(error)[:180]
                finally:
                    if not entry['angenommen']:
                        del_unit(u); add_unit(u,old['net'],old['tracks'],old['vias'])
                        PZ_LOKAL.clear(); PZ_LOKAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    _KIDX[0]=None; report['versuche'].append(entry)
                if entry.get('grund')=='CPU-Budget': break
    except (TimeoutError, _PZBudget):
        report['abbruch']='CPU-Budget'
    finally:
        hist[:]=old_hist; PF[0]=old_pf; HART_FREMD[0]=old_hart
        _PZ_BUDGET_ACTIVE[0]=old_active; _LOOP_DEADLINE=old_deadline
        report['cpu_s']=time.process_time()-cpu_start
        if BUDGET_ARBEIT: report['arbeit'] = arbeit_gesamt()-start
        report['suchkern']='atomare Suchauftraege' if BUDGET_ARBEIT else 'unterbrechbarer Python-Kern'
    return report


PZ_MINIMAX = dict(versuche=[], uebernommen=0)
arbeit_marke('vor_pinzugang')
if PZ_REST or PZ_MINI:
    PZ_MINIMAX = pinzugang_nachlauf()
    print('Pinzugang Minimax:', PZ_MINIMAX, flush=True)


if PZ_LOKAL:
    for u,r in list(U.items()):
        if u in FIXED or u.startswith(('FIX#','HEFT#')) or not _pz_active(r['net']):continue
        if not _pz_geometry_ok(r['net'],r['tracks'],r['vias'],False,u):
            del_unit(u);add_unit(u,r['net'],[],[]);fail.add(u)

_cf_final = exact_conflicts()
if _ET and not _LERN_AN: _ET.beobachten(_cf_final, MAXIT + 1, 'nachlauf')
if _LERN_AN: verhandlung_messen(MAXIT + 1, _cf_final, 'abschluss')
conf = {u: len(v) for u, v in _cf_final.items()}
print('Ende: Konflikte', conf, 'unvollstaendig', sorted(fail))
json.dump(PAIRS, open(OUT.replace('.json', '_konflikte.json'), 'w'))
tracks, vias = ausgabe_geometrie()
if PAAR_SUCHRAUM: print('Gemeinsamer Paar-Suchraum:', _PAAR_SUCH_INFO, flush=True)
if ABG_AN or ABG_BAUM or ABZW: print('Topologie (R27, Summe ueber alle Durchgaenge):', TOPO_STAT, flush=True)

VGB = []
for u, r in U.items():
    for v in r['vias']:
        if v.get('n'):
            ist = 1 + sum(1 for w in r['vias'] if w.get('zu') == [v['x'], v['y']])
            VGB.append(dict(netz=r['net'], x=v['x'], y=v['y'], soll=v['n'], ist=ist))
RVB = rv_bericht()
PAB = paare_bericht(empfaenger=True) if PAAR_NETZE else []
if PAB: print('Differenzpaare (R29): %s' % ['%s/%s %s/%s mm, schief %s, Vias %s, gekoppelt %.0f %%' % (
    b['netze'][0].split('/')[-1], b['netze'][1].split('/')[-1], b['laenge'][0], b['laenge'][1], b['schief_mm'], b['vias'], 100 * b['gekoppelt']) for b in PAB], flush=True)
if RVB: print('Rueck-Vias (Q2): %d Lagenwechsel schneller Netze mit anderer Masse-Bezugsflaeche, %d mit Masse-Via in Reichweite (%d eigenes Rueck-Via), %d ohne %s; %s' % (
    RVB['wechsel'], RVB['mit_rueck'], RVB['eigenes_rueck_via'], len(RVB['ohne']), [(b['netz'], b['abstand']) for b in RVB['ohne'][:6]], RVB['stat']), flush=True)
if VGB: print('Via-Gruppen (K10): %d Lagenwechsel mit Strom, %d Zusatz-Vias, an %d Stellen zu wenig Platz' % (
    len(VGB), sum(1 for r in U.values() for v in r['vias'] if v.get('gr')), sum(1 for b in VGB if b['ist'] < b['soll'])), flush=True)
if KETTE and KETTEN:
    _kb = [t for r in U.values() for t in r['tracks'] if t.get('kette')]
    print('Versorgungsketten (R22): %d Bahnen als Glieder, %d Pads ohne Kette bzw. ohne Sperrzone (kein Weg)' % (len(_kb), len(_KZ_AUS)), sorted(_KZ_AUS), KETTE_STAT, flush=True)

UNTER = unter_wmin()
if UNTER:
    _uf = [e for e in UNTER if not e['erlaubt']]
    print('Unter Mindestbreite: %d Stuecke (%.1f mm) in %d Netzen%s; dazu %d im Hals oder dichten Feld erlaubt (unter_wmin in der Ausgabe)' % (
        len(_uf), sum(e['laenge'] for e in _uf), len({e['netz'] for e in _uf}),
        ', am schmalsten %.3f statt %.3f mm (%s)' % min((e['breite'] - e['soll'], e['breite'], e['soll'], e['netz']) for e in _uf)[1:] if _uf else '',
        len(UNTER) - len(_uf)), flush=True)
if UNERREICHBAR: print('Unerreichbar (harte Hindernisse, Pad hat keinen Ausgang):', {n: v for n, v in sorted(UNERREICHBAR.items())})

MPV = []
for _e in EPG:
    _ist = sum(1 for r in U.values() if r['net'] == _e['netz'] for v in r['vias'] if _e['poly'].contains(_PT(v['x'], v['y'])))
    _ges = False
    if any(_k in EPM for _k in _e['pads']):
        _ges = not any(via_hart(_e['netz'], EPM[_k]['zellen']) for _k in _e['pads'] if _k in EPM)
    MPV.append(dict(teil=_e['ref'], pad=_e['num'], netz=_e['netz'], flaeche=_e['flaeche'], felder=_e['felder'], soll=_e['soll'], ist=_ist,
                    **({'grund': 'gesperrt'} if _ges else {})))
if MPV: print('Massepad-Vias (R16): %d Massepads, Soll %d Vias, Ist %d, an %d Pads weniger als Soll %s' % (
    len(MPV), sum(m['soll'] for m in MPV), sum(m['ist'] for m in MPV), sum(1 for m in MPV if m['ist'] < m['soll']),
    ['%s:%s %d/%d%s' % (m['teil'], m['pad'], m['ist'], m['soll'], ' (gesperrt: kein erlaubter Platz im Pad)' if m.get('grund') else '') for m in MPV]), flush=True)


VAPB = []
if VAP:
    _zahl = dict(im_pad=0, drittel=0, daneben=0)
    for u, r in U.items():
        if not u.startswith('GND#') or int(u[4:]) in EPM or not r['vias']: continue
        _k = int(u[4:]); _c, _w = _vap_zellen(_k); _v = r['vias'][0]
        _vc = int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES); _j = np.flatnonzero(_c == _vc)
        if len(_j): _zahl['im_pad' if _w[_j[0]] == 0.0 else 'drittel'] += 1; continue
        _zahl['daneben'] += 1; _p = D['pads'][_k]
        _hart = via_hart(r['net'], _c)
        VAPB.append(dict(teil=_p['ref'], pad=_p['num'], netz=r['net'], abstand=round(math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']), 3),
                         grund='belegt' if _hart else 'gesperrt', wichtig=_k in VAP_WICHTIG))
    print('Via am Pad (R9): %d Stiche mit Via ab der Haelfte im Pad, %d ab einem Drittel, %d daneben (davon %d gesperrt, %d belegt) %s' % (
        _zahl['im_pad'], _zahl['drittel'], _zahl['daneben'], sum(1 for b in VAPB if b['grund'] == 'gesperrt'),
        sum(1 for b in VAPB if b['grund'] == 'belegt'), ['%s:%s %s' % (b['teil'], b['pad'], b['grund']) for b in VAPB]), flush=True)
BGB = []
if BGA_FANOUT:
    for p in D['pads']:
        b = BGA.get(id(p))
        if b is None or not p['net'] or p['net'].startswith('unconnected'): continue

        near = [v for v in vias if v['net'] == p['net'] and math.dist((v['x'], v['y']), (p['x'], p['y'])) <= b['pitch'] * 0.72]
        BGB.append(dict(ref=b['ref'], pad=b['pad'], ring=b['ring'], pitch=b['pitch'],
                        via_nah=bool(near), via_im_pad=any(math.dist((v['x'], v['y']), (p['x'], p['y'])) < 0.02 for v in near)))


_rest_final = ausgabe_daten(tracks, vias, conf, roh=True,
    **({'paar_suchraum_info': _PAAR_SUCH_INFO} if PAAR_SUCHRAUM else {}),
    **({'abschluss': 'fruehabbruch' if _FRUEH_INFO is not None else ('cpu_budget' if zeit_knapp() else 'fertig'),
        'lernstand': json.loads(_LERN_SNAPSHOT)['lernstand'], 'lernbudget':json.loads(_LERN_SNAPSHOT)['lernbudget']} if _LERN_AN else {}))
if args.get('restkonflikte') == '1' and args.get('rest_teile', '1') == '1' and REST_BERICHT and KL and args.get('klassen'):
    import tempfile
    from nachbearbeitung.restkonflikte import teile_nachlauf, reparatur_sperre
    _rest_budget = max(0, float(args.get('rest_cpu', '30')) - REST_BERICHT['cpu_s'])
    if CPU_MAX > 0: _rest_budget = min(_rest_budget, max(0, CPU_MAX-such_cpu()))
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter_r45_final_') as _rest_tmp:
            _rp = Path(_rest_tmp) / 'routes.json'
            _bp = Path(_rest_tmp) / 'board.json'
            _rp.write_text(json.dumps(_rest_final)); _bp.write_text(Path(sys.argv[1]).read_text())
            _rest_block = reparatur_sperre(globals().get('ST') or {})
            if _rest_block:

                _rest_budget = 0
            _rp.with_name('routes_konflikte.json').write_text(json.dumps(PAIRS))
            _rest_teile = teile_nachlauf(str(_bp), args['klassen'], str(_rp), _rest_budget)
            if _rest_block:
                _rest_teile['grund'] = _rest_block
            _rest_final = json.loads(_rp.read_text()); _rest_final['rest_teile'] = _rest_teile
        print('R45 Teile: %d Zuege, %.2f CPU-s' % (_rest_teile['uebernommen'], _rest_teile['cpu_s']), flush=True)
    except Exception as _rest_error:
        _rest_final['rest_teile'] = dict(uebernommen=0, grund='Pruefung fehlgeschlagen: '+repr(_rest_error)[:180])
_sicher = rest_abschluss(_rest_final)
if args.get('skew', '0') == '1':
    _sicher = skew_abgleichen(D, _sicher, args, Path(OUT).with_suffix('.skew'))

arbeit_notbremse()
if ARBEIT_MESSEN:
    arbeit_marke('fertig')
    _sicher['arbeitsbudget'] = arbeit_beleg()
if MEISTER_BELEG is not None: _sicher['meister'] = MEISTER_BELEG
json.dump(_sicher, open(OUT, 'w'))
if _sicher.get('rest_board'):
    Path(OUT).with_name(Path(OUT).stem+'_rest_board.json').write_text(json.dumps(_sicher['rest_board']))
    Path(OUT).with_name(Path(OUT).stem+'_rest_placement.json').write_text(json.dumps(_sicher['rest_placement']))
print('Bahnen', len(tracks), 'Vias', len(vias), 'Zeit %.0fs' % (time.time() - t_start))
if _ET: _ET.schreiben(_sicher['tracks'], _sicher['vias'], fehlend=_sicher['failed'], zonen=_sicher.get('zonen', []) + _sicher.get('zones', []))
if _sicher.get('kurzschluss_schutz'):
    print('R44: Ausgabe %d Bahnen, %d Vias, offen %s' % (len(_sicher['tracks']), len(_sicher['vias']), _sicher['kurzschluss_schutz']['netze_entfernt']), flush=True)
if 'png' in args:
    _png_ausgabe = args.get('kurzschluss_schutz', '0') == '1' or args.get('skew', '0') == '1'
    for li, L in enumerate(LAYERS):
        img = np.zeros((NY, NX, 3), dtype=np.uint8); img[inside] = (235, 235, 230)
        if li in (0, NL - 1): img[PD1[li] <= 0.0] = (150, 150, 150)
        rng = np.random.RandomState(1); cols = rng.randint(40, 220, size=(len(nets) + 1, 3)); [cols.__setitem__(NID[_g], (60, 60, 60)) for _g in GNDS if _g in NID]
        if not _png_ausgabe:
            for u, r in U.items():
                c = r['cells'][(r['cells'] >= li * NN) & (r['cells'] < (li + 1) * NN)] - li * NN
                img.reshape(-1, 3)[c] = cols[NID[r['net']]]
        im = Image.fromarray(img).resize((NX * 2, NY * 2), Image.NEAREST)
        dr = ImageDraw.Draw(im)
        xy = lambda p: ((p[0]-X0)/RES*2, (p[1]-Y0)/RES*2)
        for t in (_sicher['tracks'] if _png_ausgabe else []):
            if t['layer'] != L: continue
            c = tuple(int(q) for q in cols[NID[t['net']]])
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']): dr.line([xy(a),xy(b)], fill=c, width=max(1,round(w/RES*2)))
        for v in (_sicher['vias'] if _png_ausgabe else []):
            x,y=xy([v['x'],v['y']]); r=VIA_R/RES*2
            dr.ellipse([x-r,y-r,x+r,y+r],fill=(40,40,40))
        for e in _sicher.get('luftlinien', []): dr.line([xy(p) for p in e['pts']],fill=(230,120,20),width=1)
        if not _png_ausgabe:
            for (lay, dist, ua, ub, x, y) in PAIRS_LAST:
                if lay not in (L, '*'): continue
                cx, cy = (x - X0) / RES * 2, (y - Y0) / RES * 2
                dr.ellipse([cx - 14, cy - 14, cx + 14, cy + 14], outline=(255, 0, 0), width=3)
        im.save(args['png'] + '_' + L + '.png')

if _sicher.get('rest_board') and 'png' in args and KL:
    from analyse.bewertung import bild
    bild(_sicher['rest_board'], _sicher, KL, args['png'])
if KNICK > 0 and NB is not None:
    print('Knick-Suche: %d Aufrufe, %d Entnahmen, %d dominiert, %.6f CPU-s' %
          (_KNICK_CALLS, _KNICK_POP, _KNICK_DOM, _KNICK_CPU), flush=True)
