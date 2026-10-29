import atexit
import contextlib
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
if __name__ == '__main__':
    sys.path.insert(0, str(ROOT))
THREADS = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'VECLIB_MAXIMUM_THREADS', 'NUMEXPR_NUM_THREADS', 'NUMBA_NUM_THREADS',
           'BLIS_NUM_THREADS', 'OMP_THREAD_LIMIT')
LOCAL = threading.local()
INSTALLED = False
ACTIVE_FDS = set()
ORIGINAL_POPEN = subprocess.Popen
COORDINATORS = {'process_guard.py', 'slots.py', 'runner.py', 'check.py', 'corpus.py', 'four_layer.py',
                'selfplay.py', 'learning_curve.py', 'endurance_run.py', 'improve.py',
                'integration.py', 'test_learning_time.py', 'zones_workbench.py',
                'collect.py', 'moves.py', 'native_probe.py', 'scheduler_probe.py',
                'test_learning_time_improvement.py', 'improvement_verify.py',
                'scheduler_suite.py', 'test_learning_time_round3.py', 'round3_verify.py', 'native_gate.py',
                'test_learning_time_round4.py', 'round4_verify.py', 'tidy_up.py', 'test_learning_time_claude.py'}
LIGHT = {'git', 'ps', 'sysctl', 'uptime', 'uname', 'which', 'sw_supply', 'tar'}

WRAPPERS = {'nice': {'-n'}, 'caffeinate': {'-t', '-w'}, 'nohup': set(), 'time': set(), 'arch': set(), 'env': {'-u', '-S', '-P'}}
PY_OPTS_ARG = {'-W', '-X', '-Q'}
WARNED = set()


def strict():
    return os.environ.get('AUTOROUTER_START_GUARD_STRICT') == '1'


def violation(message):
    if strict():
        raise RuntimeError(message)
    if message not in WARNED:
        WARNED.add(message)
        print('Startup guard warning (pid %d): %s; runs as before without its own slot' % (os.getpid(), message),
              file=sys.stderr, flush=True)


def physical_limit():
    return max(1, (os.cpu_count() or 4) - 4)


def limit():

    physical = physical_limit()
    return min(physical, max(1, int(os.environ.get('AUTOROUTER_SLOTS', physical))))


def directory():
    return Path(os.environ.get('AUTOROUTER_SLOTS_FOLDER', '/tmp/autorouter_slots'))


def environment(env=None):
    e = dict(os.environ if env is None else env)
    e.update({k: '1' for k in THREADS})
    e['OMP_MAX_ACTIVE_LEVELS'] = '1'
    e['AUTOROUTER_START_GUARD'] = str(ROOT)
    hook = str(ROOT / 'common/start_guard')
    e['PYTHONPATH'] = os.pathsep.join([hook] + [p for p in e.get('PYTHONPATH', '').split(os.pathsep) if p and p != hook])

    e.pop('AUTOROUTER_TICKET', None)
    e.pop('AUTOROUTER_COORDINATOR', None)
    return e


class Lease:
    def __init__(self, slots, fds, borrowed=False):
        self.slots, self.fds, self.borrowed = slots, fds, borrowed

    def close(self):
        if not self.borrowed:
            for fd in self.fds:


                os.close(fd)
                ACTIVE_FDS.discard(fd)
            self.fds = []

    def child_env(self, env):
        e = environment(env)
        e['AUTOROUTER_TICKET'] = json.dumps(list(zip(self.slots, self.fds)))
        return e


def inherited():
    raw = os.environ.pop('AUTOROUTER_TICKET', None)
    if not raw:
        return None
    pairs = json.loads(raw)
    if len(pairs) != 1:
        raise RuntimeError('A compute process needs exactly one slot')
    slot, fd = pairs[0]


    if type(slot) is not int or not 0 <= slot < physical_limit() or type(fd) is not int or fd < 0:
        raise RuntimeError('Outdated/wrong compute ticket')
    path = directory() / ('slot_%02d.lock' % slot)
    st = os.stat(path)
    fs = os.fstat(fd)
    if (st.st_dev, st.st_ino) != (fs.st_dev, fs.st_ino):
        raise RuntimeError('Outdated/wrong compute ticket')

    probe = os.open(path, os.O_RDWR)
    try:
        try:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:



            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('Compute ticket without lock ownership on the FD') from error
            return Lease([slot], [fd])
        raise RuntimeError('Compute ticket is not locked')
    finally:
        os.close(probe)

def check_deadline(deadline):
    if deadline is not None and time.monotonic() >= deadline:
        raise TimeoutError('Infrastructure emergency brake during compute slot allocation')


def reserve(n=1, elastic=False, deadline=None):
    if getattr(LOCAL, 'prebootstrap', False):
        raise RuntimeError('Heavy start during multiprocessing import; delegate after worker bootstrap')
    n = min(max(1, n), limit())
    d = directory()
    d.mkdir(parents=True, exist_ok=True)
    while True:
        check_deadline(deadline)
        fds, slots = [], []
        with open(d / 'assignment.lock', 'a') as mutex:

            try:
                fcntl.flock(mutex, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                mutex.close()
                time.sleep(0.02)
                continue
            for slot in range(limit()):
                fd = os.open(d / ('slot_%02d.lock' % slot), os.O_CREAT | os.O_RDWR, 0o600)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    os.close(fd)
                    continue
                fds.append(fd)
                slots.append(slot)
                if len(fds) == n:
                    break
            if len(fds) == n or (elastic and fds):
                for fd in fds:
                    os.ftruncate(fd, 0)
                    os.pwrite(fd, ('%d %s startup guard\n' % (os.getpid(), time.strftime('%H:%M:%S'))).encode(), 0)
                ACTIVE_FDS.update(fds)
                return Lease(slots, fds)
            for fd in fds:
                os.close(fd)
        time.sleep(0.1)


def current():
    return getattr(LOCAL, 'lease', None)


def ancestor_slot():
    owners = {}
    d = directory()
    for slot in range(physical_limit()):
        try:
            fd = os.open(d / ('slot_%02d.lock' % slot), os.O_RDONLY)
        except OSError:
            continue
        try:
            fields = os.pread(fd, 100, 0).decode(errors='replace').split()
            if not fields or not fields[0].isdigit():
                continue
            try:
                fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
                fcntl.flock(fd, fcntl.LOCK_UN)
            except BlockingIOError:
                owners.setdefault(int(fields[0]), slot)
        finally:
            os.close(fd)
    if not owners:
        return None
    pid = os.getppid()
    if pid in owners:
        return pid, owners[pid]
    p = ORIGINAL_POPEN(['ps', '-axo', 'pid=,ppid='], stdout=subprocess.PIPE, text=True)
    text, _ = p.communicate()
    parents = {}
    for line in text.splitlines():
        try:
            a, b = map(int, line.split())
            parents[a] = b
        except ValueError:
            continue
    seen = set()
    while pid > 1 and pid not in seen:
        if pid in owners:
            return pid, owners[pid]
        seen.add(pid)
        pid = parents.get(pid, 0)
    return None


def limit_loaded():
    if 'numpy' in sys.modules or 'scipy' in sys.modules:
        from threadpoolctl import threadpool_limits
        threadpool_limits(limits=1)
    if 'numba' in sys.modules:
        sys.modules['numba'].set_num_threads(1)


def ensure_lease():
    lease = current()
    if not lease:
        return
    try:
        for fd in lease.fds:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lease.close()
        replacement = reserve()
        lease.slots, lease.fds = replacement.slots, replacement.fds
    for fd in lease.fds:
        os.ftruncate(fd, 0)
        os.pwrite(fd, ('%d %s startup guard\n' % (os.getpid(), time.strftime('%H:%M:%S'))).encode(), 0)


@contextlib.contextmanager
def suspended():
    old = current()
    if old:


        for fd in old.fds:
            fcntl.flock(fd, fcntl.LOCK_UN)
        old.close()
        LOCAL.lease = None
    try:
        yield
    finally:
        if old:
            new = reserve()
            old.slots, old.fds = new.slots, new.fds
            LOCAL.lease = old


@contextlib.contextmanager
def seat(deadline=None):
    limit_loaded()
    old = current()
    if old:
        yield old
        return
    from common import slots as PL
    while PL.tight():
        check_deadline(deadline)
        time.sleep(1)
    lease = reserve(deadline=deadline)
    LOCAL.lease = lease
    try:
        yield lease
    finally:
        LOCAL.lease = None
        lease.close()


def program(cmd):
    a = [str(x) for x in cmd]
    i = 0
    while i < len(a) and Path(a[i]).name in WRAPPERS:
        wrapper = Path(a[i]).name
        i += 1
        while i < len(a) and (a[i].startswith('-') or (wrapper == 'env' and '=' in a[i])):
            i += 2 if a[i] in WRAPPERS[wrapper] else 1
    if i >= len(a):
        return None, None
    name = Path(a[i]).name
    if name in ('sh', 'bash', 'zsh') and a[i + 1:i + 2] == ['-c'] and len(a) > i + 3 and 'exec "$@"' in a[i + 2]:
        return program(a[i + 4:])
    if 'python' not in name.lower():
        return name, None
    i += 1
    while i < len(a) and a[i].startswith('-') and a[i] not in ('-c', '-m', '-'):
        i += 2 if a[i] in PY_OPTS_ARG else 1
    if i >= len(a) or a[i] in ('-c', '-'):
        return name, None
    if a[i] == '-m':
        if i + 1 >= len(a):
            return name, None
        def py(module):
            parts = module.split('.')
            return (parts[1] if parts[0] == 'tests' and len(parts) > 1 else parts[-1]) + '.py'
        if a[i + 1] in ('unittest', 'pytest'):
            tests = [py(x) for x in a[i + 2:] if not x.startswith('-')]
            return name, next((t for t in tests if t in COORDINATORS), tests[0] if tests else py(a[i + 1]))
        return name, py(a[i + 1])
    return name, Path(a[i]).name


def is_codex(cmd):
    return program(cmd)[0] == 'codex'


def coordinator(cmd):
    name, script = program(cmd)
    if name == 'codex':
        return True
    return (script or name) in COORDINATORS


def light(cmd):
    return isinstance(cmd, (list, tuple)) and cmd and Path(str(cmd[0])).name in LIGHT


def command_bridge(cmd):
    cmd = list(cmd)
    for i, arg in enumerate(cmd[:-1]):
        if Path(str(arg)).name == 'kicad-python-wrapper' and str(cmd[i + 1]).endswith('.py'):
            cmd.insert(i + 1, str(ROOT / 'common/native_start.py'))
            break
    return cmd


def coordinator_task(fn):
    fn._autorouter_coordinator = True
    return fn


def install(heavy=False):
    global INSTALLED
    if INSTALLED:
        return
    INSTALLED = True
    os.environ.update(environment())
    limit_loaded()
    if heavy and not current():
        holder = ancestor_slot()
        if holder:
            print('Startup guard: slot %d shared from ancestor %d (no ticket, old procedure)' % (holder[1], holder[0]),
                  file=sys.stderr, flush=True)
        else:
            from common import slots as PL
            while PL.tight():
                time.sleep(1)
            LOCAL.lease = reserve()
            atexit.register(LOCAL.lease.close)

    class GuardedPopen(ORIGINAL_POPEN):
        def __init__(self, args, *pos, **kw):
            if light(args):
                super().__init__(args, *pos, **kw)
                return
            if pos or kw.get('preexec_fn') or kw.get('shell') or isinstance(args, (str, bytes)):
                violation('Shell/preexec/positional Popen options not safe; use argv')
                kw['env'] = environment(kw.get('env'))
                super().__init__(args, *pos, **kw)
                return
            args = command_bridge(args)
            e = environment(kw.get('env'))
            if Path(str(args[0])).name in ('sh', 'bash', 'zsh') and any(str(a) in ('-c', '-lc', '-ic') for a in args[1:]):
                violation('Shell code can hide parallel runs; use script/argv and coordinator')
                kw['env'] = e
                super().__init__(args, **kw)
                return
            lease = current()
            sync = getattr(LOCAL, 'sync', False)
            coord = coordinator(args)
            if lease and not sync:

                violation('Asynchronous start from heavy process: start as coordinate=1')
                kw['env'] = e
                super().__init__(args, **kw)
                return
            owned = None
            if not coord:
                owned = None if lease and sync else reserve()
                ticket = owned or lease
                e = ticket.child_env(e)
                kw['pass_fds'] = tuple(set(kw.get('pass_fds', ())) | set(ticket.fds))
            elif not is_codex(args):
                e['AUTOROUTER_COORDINATOR'] = '1'
            kw['env'] = e
            try:
                super().__init__(args, **kw)
            except BaseException:
                if owned:
                    owned.close()
                raise
            if owned:


                def reap():
                    while self.poll() is None:
                        time.sleep(0.05)
                    owned.close()
                threading.Thread(target=reap, daemon=True).start()

    subprocess.Popen = GuardedPopen
    for name in ('run', 'call', 'check_output'):
        original = getattr(subprocess, name)
        def synchronous(*args, _original=original, **kw):
            old = getattr(LOCAL, 'sync', False)
            LOCAL.sync = True
            try:
                cmd = args[0] if args else kw.get('args', [])
                ctx = suspended() if isinstance(cmd, (list, tuple)) and coordinator(cmd) else contextlib.nullcontext()
                with ctx:
                    return _original(*args, **kw)
            finally:
                LOCAL.sync = old
                ensure_lease()
        setattr(subprocess, name, synchronous)
    _pools()
    raw_fork = os.fork
    def guarded_fork():
        if not (getattr(LOCAL, 'mp_start', False) or getattr(LOCAL, 'forkserver', False)):
            violation('Raw fork without slot: use multiprocessing.Process')
        return raw_fork()
    os.fork = guarded_fork
    def unsupported(original):
        def raw(*args, **kw):
            violation('Raw process start without slot: use subprocess with argv')
            return original(*args, **kw)
        return raw
    os.system = unsupported(os.system)
    os.posix_spawn = unsupported(os.posix_spawn)
    if hasattr(os, 'posix_spawnp'):
        os.posix_spawnp = unsupported(os.posix_spawnp)
    from concurrent.futures import ThreadPoolExecutor
    submit = ThreadPoolExecutor.submit
    def guarded_submit(self, fn, /, *args, **kw):
        if current():
            violation('Thread pool from heavy process: start as coordinate=1')
            return submit(self, fn, *args, **kw)
        def work():
            ctx = contextlib.nullcontext() if getattr(fn, '_autorouter_coordinator', False) else seat()
            with ctx:
                return fn(*args, **kw)
        return submit(self, work)
    ThreadPoolExecutor.submit = guarded_submit


class Ticket:
    def __init__(self, fd, slot):
        self.fd, self.slot = fd, slot

    def __reduce__(self):


        from multiprocessing.reduction import DupFd
        return rebuild_ticket, (DupFd(self.fd), self.slot)


def rebuild_ticket(fd, slot):
    return Ticket(fd.detach(), slot)


def _adopt(ticket):
    LOCAL.prebootstrap = False
    LOCAL.forkserver = False
    fd, slot = ticket.fd, ticket.slot
    for inherited_fd in list(ACTIVE_FDS):
        if inherited_fd != fd:
            os.close(inherited_fd)
    ACTIVE_FDS.clear()
    lease = Lease([slot], [fd])
    ACTIVE_FDS.add(fd)
    LOCAL.lease = lease
    atexit.register(lease.close)


def _pools():
    import multiprocessing.process as mp
    import multiprocessing.pool as pools
    from concurrent.futures import ProcessPoolExecutor
    start = mp.BaseProcess.start
    boot = mp.BaseProcess._bootstrap
    def guarded_boot(self, *args, **kw):
        ticket = getattr(self, '_autorouter_ticket', None)
        if ticket is None:
            LOCAL.prebootstrap = False
            LOCAL.forkserver = False
            LOCAL.lease = None
        else:
            _adopt(ticket)

        limit_loaded()
        return boot(self, *args, **kw)
    mp.BaseProcess._bootstrap = guarded_boot
    def guarded_start(self):
        self._check_closed()
        limit_loaded()
        if self._popen is not None:
            return start(self)
        if current():
            violation('Multiprocessing from heavy process: start as coordinate=1')
            self._autorouter_ticket = None
            old = getattr(LOCAL, 'mp_start', False)
            LOCAL.mp_start = True
            try:
                return start(self)
            finally:
                LOCAL.mp_start = old
        group = getattr(LOCAL, 'pool_group', None)
        lease = group.pop(0) if group else reserve()
        self._autorouter_ticket = Ticket(lease.fds[0], lease.slots[0])
        old = getattr(LOCAL, 'mp_start', False)
        LOCAL.mp_start = True
        try:
            start(self)
        except BaseException:
            lease.close()
            raise
        finally:
            LOCAL.mp_start = old

        lease.close()
    mp.BaseProcess.start = guarded_start

    @contextlib.contextmanager
    def group(n):
        if current():
            violation('Pool from heavy process: start as coordinate=1')
            yield n
            return


        lease = reserve(n, elastic=True)
        seats = [Lease([s], [fd]) for s, fd in zip(lease.slots, lease.fds)]
        LOCAL.pool_group = seats
        try:
            yield len(seats)
        finally:
            for s in seats:
                s.close()
            LOCAL.pool_group = None
    init = pools.Pool.__init__
    def pool_init(self, processes=None, *args, **kw):
        if processes is not None and processes < 1:
            return init(self, processes, *args, **kw)
        with group(processes or limit()) as n:
            init(self, n, *args, **kw)
    pools.Pool.__init__ = pool_init
    launch = ProcessPoolExecutor._launch_processes
    def executor_launch(self):
        remaining = self._max_workers - len(self._processes)
        if not remaining:
            return
        with group(remaining) as n:
            self._max_workers = len(self._processes) + n
            launch(self)
    ProcessPoolExecutor._launch_processes = executor_launch
    init_ex = ProcessPoolExecutor.__init__
    def executor_init(self, max_workers=None, *args, **kw):

        init_ex(self, min(max_workers if max_workers is not None else limit(), limit()), *args, **kw)
        self._safe_to_dynamically_spawn_children = False
    ProcessPoolExecutor.__init__ = executor_init

def bootstrap(heavy=None):
    raw = os.environ.get('AUTOROUTER_TICKET')
    got = inherited() if raw else None
    if got:
        LOCAL.lease = got
        atexit.register(got.close)
    if heavy is None:
        original_args = getattr(sys, 'orig_argv', [])
        if not hasattr(sys, 'orig_argv') and sys.argv[0] == '-c':


            p = ORIGINAL_POPEN(['ps', '-ww', '-p', str(os.getpid()), '-o', 'args='], stdout=subprocess.PIPE, text=True)
            args, _ = p.communicate()
            original_args = [args]
        kind = None
        for arg in original_args:
            match = re.search(r'(?:^|\s-c\s+)from multiprocessing\.(resource_tracker|spawn|forkserver) import (?:spawn_)?main;', str(arg))
            if match:
                kind = match.group(1)
                break
        mp_boot = kind is not None
        LOCAL.forkserver = kind == 'forkserver'
        LOCAL.prebootstrap = kind in ('spawn', 'forkserver')
        heavy = not (os.environ.get('AUTOROUTER_COORDINATOR') == '1' or
                     Path(sys.argv[0]).name in COORDINATORS or
                     mp_boot or '--multiprocessing-fork' in sys.argv)
    install(heavy=heavy and got is None)
def main():
    a = sys.argv[1:]
    if a == ['audit']:
        report = audit()
        print(json.dumps(report, ensure_ascii=False))
        return 1 if report['unsecured'] or report['load_1min'] > 20 else 0
    sep = a.index('--')
    cmd = a[sep + 1:]
    env = environment()
    if 'coordinate=1' in a[:sep] or coordinator(cmd):
        if not is_codex(cmd):
            env['AUTOROUTER_COORDINATOR'] = '1'
        return ORIGINAL_POPEN(cmd, env=env).wait()
    with seat() as lease:
        return ORIGINAL_POPEN(cmd, env=lease.child_env(env), pass_fds=lease.fds).wait()


def project_inventory(procs, roots, cwd_of):
    roots = [Path(p).resolve() for p in roots]
    def inside(path):
        return any(path == root or root in path.parents for root in roots)
    seeds = {}
    for pid, (_, _, _, comm, args) in procs.items():
        try:
            argv = shlex.split(args)
        except ValueError:
            continue
        if not argv or 'python' not in Path(argv[0]).name.lower():
            continue
        i = 1
        while i < len(argv) and argv[i] in ('-u', '-B', '-S', '-s', '-E', '-I'):
            i += 1
        if i >= len(argv) or argv[i] == '-c':
            continue
        module = argv[i] == '-m' and i + 1 < len(argv)
        script = Path(argv[i+1].replace('.', '/') + '.py') if module else Path(argv[i])
        cwd = None
        if not script.is_absolute():
            cwd = cwd_of(pid)
            if cwd is None:
                continue
            script = Path(cwd)/script
        script = script.resolve()
        if inside(script):
            seeds[pid] = dict(pid=pid, script=str(script), argv=argv, cwd=str(cwd) if cwd else None)
    members = set(seeds)
    changed = True
    while changed:
        changed = False
        for pid, (parent, *_) in procs.items():
            if parent in members and pid not in members:
                members.add(pid); changed = True
    return members, seeds


def project_roots():
    p = ORIGINAL_POPEN(['git', 'worktree', 'list', '--porcelain'], cwd=ROOT, stdout=subprocess.PIPE, text=True)
    text, _ = p.communicate()
    if p.returncode:
        raise RuntimeError('Project working copies not readable')
    roots = [line[9:] for line in text.splitlines() if line.startswith('worktree ')]
    cache = Path.home()/'Library/Caches/Autorouter'

    roots += [str(cache/'selfplay_code'), str(cache/'learning_curve')]
    return roots


def process_cwd(pid):
    p = ORIGINAL_POPEN(['/usr/sbin/lsof', '-a', '-p', str(pid), '-d', 'cwd', '-Fn'],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    text, _ = p.communicate()
    return next((line[1:] for line in text.splitlines() if line.startswith('n/')), None)


def audit():
    from common import slots as PL
    owners = {int(who.split()[0]) for _, busy, who in PL.status() if busy and who.split() and who.split()[0].isdigit()}
    r = ORIGINAL_POPEN(['ps', '-axww', '-o', 'pid=,ppid=,stat=,%cpu=,comm=,args='], stdout=subprocess.PIPE, text=True)
    text, _ = r.communicate()
    if r.returncode:
        raise RuntimeError('Process inventory not readable')
    procs = {}
    for line in text.splitlines():
        parts = line.split(None, 5)
        if len(parts) != 6:
            continue
        try:
            pid, ppid, state, cpu, comm, args = parts
            procs[int(pid)] = (int(ppid), state, float(cpu), comm, args)
        except ValueError:
            continue
    roots = project_roots()
    members, seeds = project_inventory(procs, roots, process_cwd)
    active = []
    for pid, (parent, state, cpu, comm, args) in procs.items():
        executable = Path(args.split()[0]).name if args.split() else Path(comm).name
        if pid not in members or cpu < 1 or not ('python' in executable.lower() or 'kicad' in executable.lower()):
            continue
        ancestors, p = set(), pid
        while p in procs and p not in ancestors:
            ancestors.add(p); p = procs[p][0]
        active.append(dict(pid=pid, name=executable, cpu=cpu, runnable=state.startswith('R'),
                           slot_ancestor=bool(ancestors & owners)))
    services = [dict(seed, start=process_start(pid), role=Path(seed['script']).name)
                for pid, seed in seeds.items() if Path(seed['script']).name in
                ('runner.py', 'selfplay.py', 'learning_curve.py', 'endurance_run.py')]
    return dict(version='start_guard-audit-v2', load_1min=os.getloadavg()[0], load_system=os.getloadavg(), global_limit=limit(),
                active_processes=len(active), unsecured=sum(not p['slot_ancestor'] for p in active),
                processes=active, project_roots=roots, services=services,
                cpu_percent_project=sum(p['cpu'] for p in active),
                hint='Project path/ancestor/CPU sample; system load still includes foreign projects')


def process_start(pid):
    p = ORIGINAL_POPEN(['ps', '-p', str(pid), '-o', 'lstart='], stdout=subprocess.PIPE, text=True)
    text, _ = p.communicate()
    return text.strip()


if __name__ == '__main__':
    sys.exit(main())
