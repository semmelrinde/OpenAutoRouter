""
import ctypes
import sys
import os
import signal


class Usage(ctypes.Structure):
    _fields_ = [('uuid', ctypes.c_uint8 * 16)] + [
        (name, ctypes.c_uint64) for name in
        ('user', 'system', 'idle', 'interrupt', 'pageins', 'wired', 'resident',
         'footprint', 'start', 'exit', 'child_user', 'child_system', 'child_idle',
         'child_interrupt', 'child_pageins', 'child_elapsed', 'read', 'write')]


class CpuTree:
    def __init__(self, popen):
        if sys.platform != 'darwin':
            raise RuntimeError('Aggregierte CPU-Ueberwachung braucht hier macOS libproc')
        self.popen = popen
        self.lib = ctypes.CDLL('/usr/lib/libproc.dylib')
        self.lib.proc_pid_rusage.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        self.lib.proc_pid_rusage.restype = ctypes.c_int
        class Timebase(ctypes.Structure):
            _fields_ = [('numer', ctypes.c_uint32), ('denom', ctypes.c_uint32)]
        base = Timebase()
        if ctypes.CDLL('/usr/lib/libSystem.B.dylib').mach_timebase_info(ctypes.byref(base)) or not base.denom:
            raise RuntimeError('Mach-CPU-Zeitbasis unbekannt')
        self.seconds_per_tick = base.numer / base.denom / 1e9
        self.nodes = {}
        self.maximum = 0.0

    def libproc_inventar(self):
        ""
        import struct
        n = self.lib.proc_listallpids(None, 0)
        buf = (ctypes.c_int * (max(n, 0) + 256))()
        n = self.lib.proc_listallpids(buf, ctypes.sizeof(buf))
        if n <= 0: raise RuntimeError('CPU-Prozessinventar nicht lesbar')
        info = ctypes.create_string_buffer(136); rows = []
        for pid in sorted(buf[i] for i in range(n) if buf[i] > 0):
            if self.lib.proc_pidinfo(pid, 3, 0, info, 136) == 136:
                _, _, _, p, pp = struct.unpack_from('<5I', info.raw, 0); pg, = struct.unpack_from('<I', info.raw, 100)
                rows.append((p, pp, pg))
        return rows

    def sample(self, root):
        try:
            p = self.popen(['ps', '-axo', 'pid=,ppid=,pgid='], stdout=-1, text=True)
            output, _ = p.communicate()
            if p.returncode:
                raise RuntimeError('CPU-Prozessinventar nicht lesbar')
            rows = [tuple(map(int, line.split())) for line in output.splitlines()]
        except PermissionError:

            rows = self.libproc_inventar()
        descendants = {root} | set(self.nodes)
        changed = True
        while changed:
            changed = False
            for pid, parent, group in rows:
                if pid not in descendants and (parent in descendants or group == root):
                    descendants.add(pid); changed = True


        parents = {pid: self.nodes.get(pid, {}).get('parent', parent) for pid, parent, _ in rows}
        def depth(pid):
            seen = set()
            while pid in parents and pid not in seen:
                seen.add(pid)
                pid = parents[pid]
            return len(seen)
        for node in self.nodes.values():
            node['live'] = False
        for pid, parent, group in sorted(rows, key=lambda row: depth(row[0])):
            if pid not in descendants:
                continue
            u = Usage()
            if self.lib.proc_pid_rusage(pid, 2, ctypes.byref(u)):
                continue
            old = self.nodes.get(pid)
            if old and old['start'] != u.start:
                raise RuntimeError('PID-Wechsel innerhalb eines CPU-Budgets')
            self.nodes[pid] = dict(parent=old['parent'] if old else parent, start=u.start,
                own=(u.user + u.system)*self.seconds_per_tick, reaped=(u.child_user + u.child_system)*self.seconds_per_tick,


                live=True)
        children = {}
        for pid, node in self.nodes.items():
            children.setdefault(node['parent'], []).append(pid)
        def total(pid):
            node = self.nodes[pid]
            parts = [total(c) for c in children.get(pid, [])]
            retired = node['own'] + max(node['reaped'], sum(c for c, _ in parts))
            live = sum(c for _, c in parts)


            return (0, retired + live) if node['live'] else (retired, live)
        if root in self.nodes:
            self.maximum = max(self.maximum, sum(total(root)))
        return self.maximum
    def kill(self, root):



        live = []
        for pid, node in list(self.nodes.items())[::-1]:
            u = Usage()
            if self.lib.proc_pid_rusage(pid, 2, ctypes.byref(u)) == 0 and u.start == node['start'] and not u.exit:
                live.append(pid)
        for pid in live:
            try:
                if os.getpgid(pid) == root:
                    try: os.killpg(root, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError): pass
                    break
            except ProcessLookupError:
                pass
        for pid in live:
            try: os.kill(pid, signal.SIGKILL)
            except ProcessLookupError: pass
