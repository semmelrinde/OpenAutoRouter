""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import time
import zipfile

import numpy as np

VERSION = 'agent-zwischenstand-v1'
FELDER = '''U use drill_use hist SN SN_OFFEN UOWN NETU DDN _NVER
PF HART_FREMD QUELLFREI NET_CUR STOER_FAK VIA_FAK _HYPO_AN
VG_WEICH VIA_ALT VIA_ANKER_AN fail units FIXED FIXD FIXNETS
_KZ_AUS _ERR_NETZE UNERREICHBAR STOSS_STAT KETTE_STAT RV_STAT TOPO_STAT
STROM_AUDIT _SZ_VERSION _SZ_BODEN KRITIK_N _PAAR_SUCH_INFO
_PAAR_SPLIT_GRUND MPV VAPB RVB PAB BGB VD_NACH_LOG
_VD_ZAHL _VD_VIAS _VD_VERSION _es _lokal_best todo MAXIT CPU_MAX
KETTEN KF_NACH KF_VOR PAIRS PAIRS_ROH FIX_PAIRS
_AGENT_VORGABEN _AGENT_PRIORITAET
_VZ WMINF _STAMM _STAMM_V
'''.split()
IO = {'weiter', 'zwischenstand', 'agent_halt', 'agent_runde', 'agent_pakete',
      'agent_zuege', 'loop_zuege', 'loop_log', 'loop_zustand', 'verlauf', 'png'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def schreiben(path, value):
    ""
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, suffix='.tmp')
    os.close(fd)
    try:
        with zipfile.ZipFile(tmp, 'w', compression=zipfile.ZIP_DEFLATED) as z:
            arrays = [0]
            def enc(v):
                if isinstance(v, np.ndarray):
                    name = 'a%d.npy' % arrays[0]; arrays[0] += 1
                    b = io.BytesIO(); np.save(b, v, allow_pickle=False)
                    z.writestr(name, b.getvalue()); return ['array', name]
                if isinstance(v, np.generic): return enc(v.item())
                if isinstance(v, dict): return ['dict', [[enc(k), enc(x)] for k, x in v.items()]]
                if isinstance(v, (set, frozenset)): return ['set', [enc(x) for x in sorted(v, key=repr)]]
                if isinstance(v, tuple): return ['tuple', [enc(x) for x in v]]
                if isinstance(v, list): return ['list', [enc(x) for x in v]]
                if isinstance(v, float): return ['float', v.hex()]
                if v is None or type(v) in (bool, str, int): return ['atom', v]
                raise TypeError('Nicht serialisierbarer Verhandlungszustand: %s' % type(v))
            z.writestr('zustand.json', json.dumps(enc(value), allow_nan=False))
        with open(tmp, 'rb') as f: os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def lesen(path):
    with zipfile.ZipFile(path) as z:
        def dec(v):
            tag, x = v
            if tag == 'array': return np.load(io.BytesIO(z.read(x)), allow_pickle=False)
            if tag == 'dict': return {dec(k): dec(t) for k, t in x}
            if tag == 'set': return set(map(dec, x))
            if tag == 'tuple': return tuple(map(dec, x))
            if tag == 'list': return list(map(dec, x))
            if tag == 'float': return float.fromhex(x)
            if tag == 'atom': return x
            raise ValueError('Unbekannter Zwischenstand-Typ')
        result = dec(json.loads(z.read('zustand.json')))
    if result.get('version') != VERSION: raise ValueError('Unbekannte Zwischenstandversion')
    return result


def quellen(router, argv, args):
    root = Path(router).resolve().parents[1]
    paths = {'router': router, 'klassen_code': root / 'analyse/klassen.py',
             'zustand_code': Path(__file__), 'paket_code': Path(__file__).with_name('paket.py'),
             'zuege_code': Path(__file__).with_name('zuege.py')}
    paths.update({k: args[k] for k in ('klassen', 'regeln', 'start', 'fix') if args.get(k)})
    return {k: sha(p) for k, p in paths.items()}


def vorbereiten(router, argv, args):
    ""
    if args.get('agent_loop', '0') not in ('0', '1'): raise ValueError('agent_loop=0|1')
    if not args.get('weiter'): return None
    if args.get('agent_loop') != '1': raise ValueError('weiter braucht agent_loop=1')
    saved = lesen(args['weiter'])
    if sha(argv[1]) != saved['board_sha']: raise ValueError('weiter: anderes Eingangsboard')
    if quellen(router, argv, args) != saved['quellen']: raise ValueError('weiter: Quellen/Eingaben geaendert')
    semantic = lambda a: {k: v for k, v in a.items() if k not in IO}
    if semantic(args) != semantic(saved['argv_optionen']):
        raise ValueError('weiter: Suchoptionen geaendert')
    return saved


class AgentLoop:
    def __init__(self, G, saved):
        from .paket import exportieren
        self.G = G; self.saved = saved; self.exportieren = exportieren
        a = G['args']; self.rl = G['_RL']; self.resume_pending = saved is not None
        if not G['KL']: raise ValueError('agent_loop braucht klassen=')
        if self.rl.disabled: raise ValueError('agent_loop gesperrt: ' + self.rl.disabled)
        if G['_LERN_AN'] or G['_ET'] is not None:
            raise ValueError('agent_loop: Lern-/Etikettenlauf nicht fortsetzbar')
        if a.get('agent_halt', '1') not in ('0', '1'): raise ValueError('agent_halt=0|1')
        self.round = int(a.get('agent_runde', '0'))
        if self.round < 0: raise ValueError('agent_runde muss >=0 sein')
        self.path = Path(a.get('zwischenstand', G['OUT'] + '.zwischenstand.zip'))
        self.packages = Path(a.get('agent_pakete', G['OUT'] + '_pakete'))
        self.cpu_offset = 0.
        G['_AGENT_VORGABEN'] = []; G['_AGENT_PRIORITAET'] = []
        if saved:
            for k, v in saved['dynamisch'].items(): G[k] = v
            G['VIAF'] = G['VIA_FAK']
            G['_FL'] = {(id(G['D']['pads'][i]),w):v for i,w,v in saved.get('pad_flucht',[])}
            for k, v in saved['loop'].items(): setattr(self.rl, k, v)
            self.rl.initial_board = saved['initial_board']
            self.cpu_offset = saved['cpu_s'] - time.process_time()
            print('Agent: exakt weiter vor Runde %d (%d Einheiten)' % (saved['runde'], len(G['U'])), flush=True)
        self.install_router()

    def install_router(self):
        from .zuege import nachfuehren
        G = self.G; base = G['route_unit']
        def route(u):
            ok = base(u)
            if ok:
                try: nachfuehren(G, u)
                except ValueError:
                    G['del_unit'](u); return False
            return ok
        G['route_unit'] = route

    def before_round(self, it, pairs):
        G = self.G
        if self.resume_pending:
            self.resume_pending = False
            return self.eingriffe()
        triggered = self.rl.trigger(pairs)
        triggered = (triggered and len(pairs) <= self.rl.threshold and
                     len(self.rl.counts) >= self.rl.k and min(self.rl.counts) >= self.rl.counts[0])
        if not (pairs and (it == self.round or (not self.round and triggered))): return False
        self.rl.rounds = it - 1
        checkpoint_cpu = time.process_time()
        board = self.rl.board(G['D']); board['pads'] = board['pads'][:G['_LOOP_SOURCE_PAD_COUNT']]
        ts, vs = G['ausgabe_geometrie']()
        raw = G['ausgabe_daten'](ts, vs, {u:len(v) for u,v in G['exact_conflicts']().items()}, roh=True)
        pad_ids={id(p):i for i,p in enumerate(G['D']['pads'])}
        value = dict(version=VERSION, runde=it, board=board,
                     rohrouten=raw,
                     pad_flucht=[[pad_ids[pid],w,v] for (pid,w),v in G['_FL'].items() if pid in pad_ids],
                     board_sha=G['_AGENT_BOARD_SHA'], argv_optionen=G['_AGENT_ARGV'],
                     quellen=G['_AGENT_QUELLEN'], cpu_s=checkpoint_cpu + self.cpu_offset,
                     initial_board=self.rl.initial_board,
                     dynamisch={k: G[k] for k in FELDER if k in G},
                     loop={k: v for k, v in vars(self.rl).items() if k not in
                           ('G', 'rest', 'bewerte', 'log', 'exchange', 'external', 'input_path')})
        schreiben(self.path, value)
        package_dirs = []
        for i, c in enumerate(self.rl.clusters(pairs)[:3]):
            c['nachbarn'] = sorted(self.rl.neighbours(c))
            d = self.packages / ('%04d_%02d' % (it - 1, i))
            self.exportieren(self, c, d)
            package_dirs.append(str(d))
        receipt = dict(version=VERSION, zustand='wartet', runde=it,
                       zwischenstand=str(self.path), sha256=sha(self.path),
                       pakete=package_dirs, cpu_s=time.process_time() + self.cpu_offset)
        Path(G['OUT']).write_text(json.dumps(receipt, indent=1) + '\n')
        print('Agent: Plateau, Zwischenstand %s, %d Pakete' % (self.path, len(package_dirs)), flush=True)
        self.cpu_offset -= time.process_time() - checkpoint_cpu
        if G['args'].get('agent_halt', '1') == '1': raise SystemExit(0)
        return False

    def eingriffe(self):
        ""
        path = self.G['args'].get('agent_zuege') or self.G['args'].get('loop_zuege')
        if not path: return False
        payload = json.loads(Path(path).read_text())
        if not isinstance(payload, list) or len(payload) > 256: raise ValueError('agent_zuege: Liste, hoechstens 256')
        rl = self.rl; G = self.G; changed = False
        for raw in payload:
            start = time.process_time()
            try:
                m = rl.validate_move(raw)
                if m['zustand'] != rl.digest(): raise ValueError('veralteter Zustand')
                G['exact_conflicts']()
                c = next((c for c in rl.clusters(list(G['PAIRS'])) if rl.external_local(m, c)), None)
                if c is None: raise ValueError('Zug passt zu keiner lokalen Engstelle')
                c['nachbarn'] = sorted(rl.neighbours(c))
                blocked = rl.prefilter(m, c)
                if blocked: raise ValueError(blocked)
                R0, p0 = rl.out(); E0 = rl.bewerte(G['D'], R0, G['KL'], v3=True, minimax=False)
                changed = self.versuch(m,c,R0,p0,E0,start) or changed
            except (ValueError, KeyError, IndexError, TypeError) as exc:
                rl.emit(dict(ereignis='abgelehnt', zug=raw, grund=str(exc), angenommen=False, cpu_s=time.process_time()-start))
        if changed:
            rl.stale = 0; rl.counts = []; rl.cells = {}; rl.previous = None
        return changed

    def versuch(self,m,c,R0,p0,E0,start):
        ""
        G=self.G;rl=self.rl;t=time.process_time()
        saved=copy.deepcopy({k:G[k] for k in FELDER if k in G})
        flucht=copy.deepcopy(G['_FL']);board=G['D'];cpu_save=time.process_time()-t
        ok=False;cpu_restore=0.
        try:
            ok=rl.trial(m,c,R0,p0,E0,start+rl.move_cpu)
            return ok
        finally:
            t=time.process_time()
            if not ok:
                G.update(saved);G['VIAF']=G['VIA_FAK'];G['_FL']=flucht
            else:


                G['_FL']={}
            for k in ('_GEO','_NAHE_C','_ABST_C','_R41_C','_KUPFER_VORHALT_C'):
                if k in G:G[k]={}
            if G.get('_pad_room_cached') is not None:G['_pad_room_cached'].cache_clear()
            G['_SZ_RESERVE']=[None];G['_VD_CACHE']=[None]
            G['_KIDX']=[None];G['ASTAR_COST']=[None];G['ASTAR_HINT']=[None];G['VIA_ZU']=[None]
            cpu_restore=time.process_time()-t
            row=json.loads(rl.log.read_text().splitlines()[-1])
            row.pop('version',None);row.pop('quellen',None)
            row.update(ereignis='agent_zug',cpu_s=time.process_time()-start,
                       cpu_zustand_sichern_s=cpu_save,cpu_zustand_ruecknahme_s=cpu_restore)
            rl.emit(row)
