""
import contextlib, fcntl, os, signal, subprocess, sys, threading, time
if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ORDNER = os.environ.get('AUTOROUTER_PLAETZE_ORDNER', '/tmp/autorouter_plaetze')
CPU_TAKT_S = 1.0


def anzahl():
    ""
    from gemeinsam.prozessschutz import limit
    return limit()


_N = []


def _n():
    if not _N: _N.append(anzahl())
    return _N[0]


def _datei(k): return os.path.join(ORDNER, 'platz_%02d.lock' % k)


SPEICHER_MIN = int(os.environ.get('AUTOROUTER_SPEICHER_MIN', '15'))
PLATTE_MIN_GB = float(os.environ.get('AUTOROUTER_PLATTE_MIN_GB', '20'))


def knapp():
    ""
    try:
        frei = int(subprocess.run(['sysctl', '-n', 'kern.memorystatus_level'], capture_output=True, text=True, timeout=5).stdout.strip())
        if frei < SPEICHER_MIN: return True
    except Exception:
        pass
    try:
        st = os.statvfs('/')
        if st.f_bavail * st.f_frsize / 1e9 < PLATTE_MIN_GB: return True
    except Exception:
        pass
    return False


@contextlib.contextmanager
def platz(wand_s=None):
    ""
    from gemeinsam.prozessschutz import seat
    t0 = time.monotonic()
    with seat(deadline=t0 + wand_s if wand_s is not None else None) as lease:
        yield lease.slots[0], time.monotonic() - t0


def status():
    ""
    os.makedirs(ORDNER, exist_ok=True); out = []
    for k in range(_n()):
        fd = os.open(_datei(k), os.O_CREAT | os.O_RDWR, 0o666)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB); fcntl.flock(fd, fcntl.LOCK_UN); out.append((k, False, ''))
        except OSError:
            out.append((k, True, os.pread(fd, 100, 0).decode(errors='replace').strip()))
        os.close(fd)
    return out


def lauf(cmd, cpu_s=None, wand_s=None, log=None, env=None, cwd=None, mit_platz=True, platz_wand_s=None, schutz_root=None, terminal=False,
         koord=False):
    ""
    cmd = [str(c) for c in cmd]
    from gemeinsam.prozessschutz import coordinator, suspended, command_bridge
    coord = koord or coordinator(cmd)
    if cpu_s:
        import math
        cmd = ['/bin/sh', '-c', 'ulimit -S -t %d; exec "$@"' % math.ceil(cpu_s), 'sh'] + cmd
        if wand_s is None: wand_s = 10 * cpu_s
    cmd = command_bridge(cmd)

    ctx = contextlib.nullcontext((None, 0.0)) if coord else platz(wand_s=platz_wand_s)
    suspension = suspended() if coord else contextlib.nullcontext()
    t0 = time.monotonic()
    try:
        with suspension:
            return _lauf(cmd, cpu_s, wand_s, log, env, cwd, ctx, coord, schutz_root, terminal)
    except TimeoutError:

        waited = round(time.monotonic() - t0, 1)
        return dict(rc=124, zustand='wandlimit', phase='platzvergabe', cpu_s=0.0,
                    wand_s=waited, warte_s=waited, last=round(os.getloadavg()[0], 1), platz=None)


def _lauf(cmd, cpu_s, wand_s, log, env, cwd, ctx, coord, schutz_root, terminal=False):
    with ctx as (nr, warte):
        t0 = time.time(); last = os.getloadavg()[0]
        out = open(log, 'w') if log else (None if terminal else subprocess.DEVNULL)
        try:
            from gemeinsam.prozessschutz import current, environment, ORIGINAL_POPEN
            lease = current()
            child_env = lease.child_env(env) if lease else environment(env)
            if schutz_root is not None:
                from pathlib import Path
                root = Path(schutz_root).resolve()
                child_env['AUTOROUTER_STARTSCHUTZ'] = str(root)
                child_env['PYTHONPATH'] = str(root/'gemeinsam/startschutz') + os.pathsep + child_env.get('PYTHONPATH', '')
            from gemeinsam.prozessschutz import is_codex
            if coord and not is_codex(cmd):
                child_env['AUTOROUTER_KOORDINATOR'] = '1'
            from gemeinsam.cpu_baum import CpuTree
            tree = CpuTree(ORIGINAL_POPEN) if cpu_s else None
            p = ORIGINAL_POPEN(cmd, stdout=out, stderr=subprocess.STDOUT, env=child_env, cwd=cwd,
                               pass_fds=tuple(lease.fds) if lease else (), start_new_session=True)
            wand = [False]
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
                        done.wait(CPU_TAKT_S)
                except ProcessLookupError:
                    pass
                except BaseException as error:
                    monitor_errors.append(repr(error))
                    tree.kill(p.pid)
            watcher = threading.Thread(target=cpu_watch, daemon=True) if tree else None
            if watcher: watcher.start()
            def bremse():
                wand[0] = True
                if tree: tree.kill(p.pid)
                else:
                    try: os.killpg(p.pid, signal.SIGKILL)
                    except OSError: pass
            tm = threading.Timer(wand_s, bremse) if wand_s else None
            if tm: tm.daemon = True; tm.start()
            _, st, ru = os.wait4(p.pid, 0)
            done.set()
            if watcher: watcher.join()
            if tm: tm.cancel()
            p.returncode = os.waitstatus_to_exitcode(st)
            if p.returncode < 0 and tree:
                tree.kill(p.pid)
            from gemeinsam.prozessschutz import ensure_lease
            ensure_lease()
        finally:
            if log: out.close()
        rc = p.returncode; cpu = max(ru.ru_utime + ru.ru_stime, tree.maximum if tree else 0.0)
        if monitor_errors: zustand = 'infrastruktur'
        elif budget[0]: zustand = 'zeitlimit'
        elif wand[0]: zustand = 'wandlimit'
        elif cpu_s and (rc == -signal.SIGXCPU or (rc == -signal.SIGKILL and cpu >= cpu_s - 1)): zustand = 'zeitlimit'
        else: zustand = 'ok' if rc == 0 else 'fehler'
        return dict(rc=rc, zustand=zustand, cpu_s=round(cpu, 1), wand_s=round(time.time() - t0, 1), warte_s=round(warte, 1),
                    last=round(last, 1), platz=nr, cpu_monitor_fehler=monitor_errors,
                    cpu_messung='libproc-Prozessbaum+wait4' if tree else 'wait4')


def main():
    a = sys.argv[1:]
    if a[:1] == ['status']:
        S = status()
        print('%d von %d Plaetzen belegt (%s), Last %.1f' % (sum(b for _, b, _ in S), len(S), ORDNER, os.getloadavg()[0]))
        for k, b, wer in S:
            if b: print('  Platz %2d: %s' % (k, wer))
        return 0
    if '--' not in a:
        print(__doc__); return 2
    kw = dict(x.split('=', 1) for x in a[:a.index('--')] if '=' in x); cmd = a[a.index('--') + 1:]

    r = lauf(cmd, cpu_s=float(kw['cpu']) if 'cpu' in kw else None, terminal=True, koord=kw.get('koord') == '1')
    if 'cpu' in kw:
        import json
        print('PL-Budget ' + json.dumps(r), file=sys.stderr, flush=True)
    if r['zustand'] in ('zeitlimit', 'wandlimit'):
        return 124
    if r['zustand'] == 'infrastruktur':
        return 78
    rc = r['rc']
    return rc if rc >= 0 else 128 - rc


if __name__ == '__main__':
    sys.exit(main())
