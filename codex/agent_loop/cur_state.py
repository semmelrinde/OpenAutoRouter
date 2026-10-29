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

VERSION = 'agent-interim_state-v1'
FIELDS = '''U use drill_use hist SN SN_OPEN UOWN NETU DDN _NVER
PF HARD_FOREIGN SOURCE_FREE NET_CUR DISTURB_FACTOR VIA_FACTOR _HYPO_ON
CONS_SOFT VIA_PREV VIA_ANCHOR_ON fail units FIXED FIXD FIXNETS
_METRIC_OFF _ERR_NETS UNREACHABLE JOINT_STAT CHAIN_STAT RV_STAT TOPO_STAT
CURRENT_AUDIT _SZ_VERSION _SZ_FLOOR CRITIQUE_N _PAIR_SEARCH_INFO
_PAIR_SPLIT_REASON MPV VAPB RVB PAB BGB VD_AFTER_LOG
_VD_COUNT _VD_VIAS _VD_VERSION _es _local_best todo MAXIT CPU_MAX
CHAINS KF_AFTER KF_BEFORE PAIRS PAIRS_RAW FIX_PAIRS
_AGENT_SPECS _AGENT_PRIORITY
_VZ WMINF _TRUNK _TRUNK_V
'''.split()
IO = {'further', 'interim_state', 'agent_halt', 'agent_round', 'agent_packages',
      'agent_moves', 'loop_moves', 'loop_log', 'loop_state', 'history', 'png'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_out(path, value):
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
                raise TypeError('Non-serializable negotiation state: %s' % type(v))
            z.writestr('cur_state.json', json.dumps(enc(value), allow_nan=False))
        with open(tmp, 'rb') as f: os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def read_in(path):
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
            raise ValueError('Unknown intermediate state type')
        result = dec(json.loads(z.read('cur_state.json')))
    if result.get('version') != VERSION: raise ValueError('Unknown intermediate state version')
    return result


def source_list(router, argv, args):
    root = Path(router).resolve().parents[1]
    paths = {'router': router, 'netclasses_code': root / 'analysis/netclasses.py',
             'state_code': Path(__file__), 'package_part_code': Path(__file__).with_name('package_part.py'),
             'moves_code': Path(__file__).with_name('moves.py')}
    paths.update({k: args[k] for k in ('netclasses', 'design_rules', 'start', 'fix') if args.get(k)})
    return {k: sha(p) for k, p in paths.items()}


def prepare(router, argv, args):
    if args.get('agent_loop', '0') not in ('0', '1'): raise ValueError('agent_loop=0|1')
    if not args.get('further'): return None
    if args.get('agent_loop') != '1': raise ValueError('further needs agent_loop=1')
    saved = read_in(args['further'])
    if sha(argv[1]) != saved['board_sha']: raise ValueError('further: different input board')
    if source_list(router, argv, args) != saved['source_list']: raise ValueError('further: sources/inputs changed')
    semantic = lambda a: {k: v for k, v in a.items() if k not in IO}
    if semantic(args) != semantic(saved['argv_options']):
        raise ValueError('further: search options changed')
    return saved


class AgentLoop:
    def __init__(self, G, saved):
        from .package_part import export
        self.G = G; self.saved = saved; self.export = export
        a = G['args']; self.rl = G['_RL']; self.resume_pending = saved is not None
        if not G['NC']: raise ValueError('agent_loop needs netclasses=')
        if self.rl.disabled: raise ValueError('agent_loop blocked: ' + self.rl.disabled)
        if G['_LEARN_ON'] or G['_ET'] is not None:
            raise ValueError('agent_loop: learning/label run cannot be resumed')
        if a.get('agent_halt', '1') not in ('0', '1'): raise ValueError('agent_halt=0|1')
        self.round = int(a.get('agent_round', '0'))
        if self.round < 0: raise ValueError('agent_round must be >=0')
        self.path = Path(a.get('interim_state', G['OUT'] + '.interim_state.zip'))
        self.packages = Path(a.get('agent_packages', G['OUT'] + '_packages'))
        self.cpu_offset = 0.
        G['_AGENT_SPECS'] = []; G['_AGENT_PRIORITY'] = []
        if saved:
            for k, v in saved['dynamic'].items(): G[k] = v
            G['VIAF'] = G['VIA_FACTOR']
            G['_FL'] = {(id(G['D']['pads'][i]),w):v for i,w,v in saved.get('pad_escape',[])}
            for k, v in saved['loop'].items(): setattr(self.rl, k, v)
            self.rl.initial_board = saved['initial_board']
            self.cpu_offset = saved['cpu_s'] - time.process_time()
            print('Agent: continue exactly before round %d (%d units)' % (saved['round_no'], len(G['U'])), flush=True)
        self.install_router()

    def install_router(self):
        from .moves import update
        G = self.G; base = G['route_unit']
        def route(u):
            ok = base(u)
            if ok:
                try: update(G, u)
                except ValueError:
                    G['del_unit'](u); return False
            return ok
        G['route_unit'] = route

    def before_round(self, it, pairs):
        G = self.G
        if self.resume_pending:
            self.resume_pending = False
            return self.interventions()
        triggered = self.rl.trigger(pairs)
        triggered = (triggered and len(pairs) <= self.rl.threshold and
                     len(self.rl.counts) >= self.rl.k and min(self.rl.counts) >= self.rl.counts[0])
        if not (pairs and (it == self.round or (not self.round and triggered))): return False
        self.rl.rounds = it - 1
        checkpoint_cpu = time.process_time()
        board = self.rl.board(G['D']); board['pads'] = board['pads'][:G['_LOOP_SOURCE_PAD_COUNT']]
        ts, vs = G['export_geometry']()
        raw = G['export_data'](ts, vs, {u:len(v) for u,v in G['exact_conflicts']().items()}, raw_data=True)
        pad_ids={id(p):i for i,p in enumerate(G['D']['pads'])}
        value = dict(version=VERSION, round_no=it, board=board,
                     raw_routes=raw,
                     pad_escape=[[pad_ids[pid],w,v] for (pid,w),v in G['_FL'].items() if pid in pad_ids],
                     board_sha=G['_AGENT_BOARD_SHA'], argv_options=G['_AGENT_ARGV'],
                     source_list=G['_AGENT_SOURCES'], cpu_s=checkpoint_cpu + self.cpu_offset,
                     initial_board=self.rl.initial_board,
                     dynamic={k: G[k] for k in FIELDS if k in G},
                     loop={k: v for k, v in vars(self.rl).items() if k not in
                           ('G', 'rest', 'rate', 'log', 'exchange', 'external', 'input_path')})
        write_out(self.path, value)
        package_dirs = []
        for i, c in enumerate(self.rl.clusters(pairs)[:3]):
            c['neighbors'] = sorted(self.rl.neighbours(c))
            d = self.packages / ('%04d_%02d' % (it - 1, i))
            self.export(self, c, d)
            package_dirs.append(str(d))
        receipt = dict(version=VERSION, cur_state='waits', round_no=it,
                       interim_state=str(self.path), sha256=sha(self.path),
                       packages=package_dirs, cpu_s=time.process_time() + self.cpu_offset)
        Path(G['OUT']).write_text(json.dumps(receipt, indent=1) + '\n')
        print('Agent: plateau, intermediate state %s, %d packages' % (self.path, len(package_dirs)), flush=True)
        self.cpu_offset -= time.process_time() - checkpoint_cpu
        if G['args'].get('agent_halt', '1') == '1': raise SystemExit(0)
        return False

    def interventions(self):
        path = self.G['args'].get('agent_moves') or self.G['args'].get('loop_moves')
        if not path: return False
        payload = json.loads(Path(path).read_text())
        if not isinstance(payload, list) or len(payload) > 256: raise ValueError('agent_moves: list, at most 256')
        rl = self.rl; G = self.G; changed = False
        for raw in payload:
            start = time.process_time()
            try:
                m = rl.validate_move(raw)
                if m['cur_state'] != rl.digest(): raise ValueError('outdated state')
                G['exact_conflicts']()
                c = next((c for c in rl.clusters(list(G['PAIRS'])) if rl.external_local(m, c)), None)
                if c is None: raise ValueError('Move matches no local bottleneck')
                c['neighbors'] = sorted(rl.neighbours(c))
                blocked = rl.prefilter(m, c)
                if blocked: raise ValueError(blocked)
                R0, p0 = rl.out(); E0 = rl.rate(G['D'], R0, G['NC'], v3=True, minimax=False)
                changed = self.attempt(m,c,R0,p0,E0,start) or changed
            except (ValueError, KeyError, IndexError, TypeError) as exc:
                rl.emit(dict(event='declined', move=raw, reason=str(exc), taken=False, cpu_s=time.process_time()-start))
        if changed:
            rl.stale = 0; rl.counts = []; rl.cells = {}; rl.previous = None
        return changed

    def attempt(self,m,c,R0,p0,E0,start):
        G=self.G;rl=self.rl;t=time.process_time()
        saved=copy.deepcopy({k:G[k] for k in FIELDS if k in G})
        escape=copy.deepcopy(G['_FL']);board=G['D'];cpu_save=time.process_time()-t
        ok=False;cpu_restore=0.
        try:
            ok=rl.trial(m,c,R0,p0,E0,start+rl.move_cpu)
            return ok
        finally:
            t=time.process_time()
            if not ok:
                G.update(saved);G['VIAF']=G['VIA_FACTOR'];G['_FL']=escape
            else:


                G['_FL']={}
            for k in ('_GEO','_NEARBY_C','_DIST_C','_R41_C','_COPPER_RESERVE_C'):
                if k in G:G[k]={}
            if G.get('_pad_room_cached') is not None:G['_pad_room_cached'].cache_clear()
            G['_SZ_RESERVE']=[None];G['_VD_CACHE']=[None]
            G['_CAND_IDX']=[None];G['ASTAR_COST']=[None];G['ASTAR_HINT']=[None];G['VIA_TOO']=[None]
            cpu_restore=time.process_time()-t
            row=json.loads(rl.log.read_text().splitlines()[-1])
            row.pop('version',None);row.pop('source_list',None)
            row.update(event='agent_move',cpu_s=time.process_time()-start,
                       cpu_state_save_s=cpu_save,cpu_state_rollback_s=cpu_restore)
            rl.emit(row)
