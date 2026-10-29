import contextlib, fcntl, os, signal, subprocess, sys, threading, time
if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

FOLDER = os.environ.get('AUTOROUTER_SLOTS_FOLDER', '/tmp/autorouter_slots')
CPU_CLOCK_S = 1.0


def count():
    from common.process_guard import limit
    return limit()


_N = []


def _n():
    if not _N: _N.append(count())
    return _N[0]


def _file(k): return os.path.join(FOLDER, 'slot_%02d.lock' % k)


MEMORY_MIN = int(os.environ.get('AUTOROUTER_MEMORY_MIN', '15'))
PLATE_MIN_GB = float(os.environ.get('AUTOROUTER_PLATE_MIN_GB', '20'))


def tight():
    try:
        vacant = int(subprocess.run(['sysctl', '-n', 'core_set.memorystatus_level'], capture_output=True, text=True, timeout=5).stdout.strip())
        if vacant < MEMORY_MIN: return True
    except Exception:
        pass
    try:
        st = os.statvfs('/')
        if st.f_bavail * st.f_frsize / 1e9 < PLATE_MIN_GB: return True
    except Exception:
        pass
    return False


@contextlib.contextmanager
def slot(wall_s=None):
    from common.process_guard import seat
    t0 = time.monotonic()
    with seat(deadline=t0 + wall_s if wall_s is not None else None) as lease:
        yield lease.slots[0], time.monotonic() - t0


def status():
    os.makedirs(FOLDER, exist_ok=True); out = []
    for k in range(_n()):
        fd = os.open(_file(k), os.O_CREAT | os.O_RDWR, 0o666)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB); fcntl.flock(fd, fcntl.LOCK_UN); out.append((k, False, ''))
        except OSError:
            out.append((k, True, os.pread(fd, 100, 0).decode(errors='replace').strip()))
        os.close(fd)
    return out


def job(cmd, cpu_s=None, wall_s=None, log=None, env=None, cwd=None, with_slot=True, slot_wall_s=None, protection_root=None, terminal=False,
         coordinate=False):
    cmd = [str(c) for c in cmd]
    from common.process_guard import coordinator, suspended, command_bridge
    coord = coordinate or coordinator(cmd)
    if cpu_s:
        import math
        cmd = ['/bin/sh', '-c', 'ulimit -S -t %d; exec "$@"' % math.ceil(cpu_s), 'sh'] + cmd
        if wall_s is None: wall_s = 10 * cpu_s
    cmd = command_bridge(cmd)

    ctx = contextlib.nullcontext((None, 0.0)) if coord else slot(wall_s=slot_wall_s)
    suspension = suspended() if coord else contextlib.nullcontext()
    t0 = time.monotonic()
    try:
        with suspension:
            return _run(cmd, cpu_s, wall_s, log, env, cwd, ctx, coord, protection_root, terminal)
    except TimeoutError:

        waited = round(time.monotonic() - t0, 1)
        return dict(rc=124, cur_state='wall_limit', phase='slot_allocation', cpu_s=0.0,
                    wall_s=waited, wait_s=waited, load=round(os.getloadavg()[0], 1), slot=None)


def _run(cmd, cpu_s, wall_s, log, env, cwd, ctx, coord, protection_root, terminal=False):
    with ctx as (nr, wait):
        t0 = time.time(); load = os.getloadavg()[0]
        out = open(log, 'w') if log else (None if terminal else subprocess.DEVNULL)
        try:
            from common.process_guard import current, environment, ORIGINAL_POPEN
            lease = current()
            child_env = lease.child_env(env) if lease else environment(env)
            if protection_root is not None:
                from pathlib import Path
                root = Path(protection_root).resolve()
                child_env['AUTOROUTER_START_GUARD'] = str(root)
                child_env['PYTHONPATH'] = str(root/'common/start_guard') + os.pathsep + child_env.get('PYTHONPATH', '')
            from common.process_guard import is_codex
            if coord and not is_codex(cmd):
                child_env['AUTOROUTER_COORDINATOR'] = '1'
            from common.cpu_tree import CpuTree
            tree = CpuTree(ORIGINAL_POPEN) if cpu_s else None
            p = ORIGINAL_POPEN(cmd, stdout=out, stderr=subprocess.STDOUT, env=child_env, cwd=cwd,
                               pass_fds=tuple(lease.fds) if lease else (), start_new_session=True)
            wall = [False]
            budget = [False]
            monitor_errors = []
            done = threading.Event()
            def cpu_watch():
                try:
                    while not done.is_set():
                        if tree.sample(p.pid) >= cpu_s:
                            budget[0] = True
                            tree.kill(p.pid)
                            return
                        done.wait(CPU_CLOCK_S)
                except ProcessLookupError:
                    pass
                except BaseException as error:
                    monitor_errors.append(repr(error))
                    tree.kill(p.pid)
            watcher = threading.Thread(target=cpu_watch, daemon=True) if tree else None
            if watcher: watcher.start()
            def brake():
                wall[0] = True
                if tree: tree.kill(p.pid)
                else:
                    try: os.killpg(p.pid, signal.SIGKILL)
                    except OSError: pass
            tm = threading.Timer(wall_s, brake) if wall_s else None
            if tm: tm.daemon = True; tm.start()
            _, st, ru = os.wait4(p.pid, 0)
            done.set()
            if watcher: watcher.join()
            if tm: tm.cancel()
            p.returncode = os.waitstatus_to_exitcode(st)
            if p.returncode < 0 and tree:
                tree.kill(p.pid)
            from common.process_guard import ensure_lease
            ensure_lease()
        finally:
            if log: out.close()
        rc = p.returncode; cpu = max(ru.ru_utime + ru.ru_stime, tree.maximum if tree else 0.0)
        if monitor_errors: cur_state = 'infrastructure'
        elif budget[0]: cur_state = 'time_limit'
        elif wall[0]: cur_state = 'wall_limit'
        elif cpu_s and (rc == -signal.SIGXCPU or (rc == -signal.SIGKILL and cpu >= cpu_s - 1)): cur_state = 'time_limit'
        else: cur_state = 'ok' if rc == 0 else 'faults'
        return dict(rc=rc, cur_state=cur_state, cpu_s=round(cpu, 1), wall_s=round(time.time() - t0, 1), wait_s=round(wait, 1),
                    load=round(load, 1), slot=nr, cpu_monitor_error=monitor_errors,
                    cpu_measurement='libproc-ProcessTree+wait4' if tree else 'wait4')


def main():
    a = sys.argv[1:]
    if a[:1] == ['status']:
        S = status()
        print('%d of %d slots occupied (%s), load %.1f' % (sum(b for _, b, _ in S), len(S), FOLDER, os.getloadavg()[0]))
        for k, b, who in S:
            if b: print('  Slot %2d: %s' % (k, who))
        return 0
    if '--' not in a:
        print(__doc__); return 2
    kw = dict(x.split('=', 1) for x in a[:a.index('--')] if '=' in x); cmd = a[a.index('--') + 1:]

    r = job(cmd, cpu_s=float(kw['cpu']) if 'cpu' in kw else None, terminal=True, coordinate=kw.get('coordinate') == '1')
    if 'cpu' in kw:
        import json
        print('PL budget ' + json.dumps(r), file=sys.stderr, flush=True)
    if r['cur_state'] in ('time_limit', 'wall_limit'):
        return 124
    if r['cur_state'] == 'infrastructure':
        return 78
    rc = r['rc']
    return rc if rc >= 0 else 128 - rc


if __name__ == '__main__':
    sys.exit(main())
