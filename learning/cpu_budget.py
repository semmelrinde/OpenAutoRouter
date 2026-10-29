from contextlib import contextmanager
import math
import time


class Budget:
    def __init__(self, cpu_s, clock=None):
        self.limit = float(cpu_s)
        if not math.isfinite(self.limit) or self.limit <= 0:
            raise ValueError('CPU budget must be positive and finite')
        self.clock = clock or time.thread_time
        self.start = self.clock()
        self.children = 0.0
        self.diagnose_owned = 0.0

    @property
    def used(self):
        return self.children + max(0.0, self.clock() - self.start - self.diagnose_owned)

    @property
    def rest(self):
        return max(0.0, self.limit - self.used)

    def add(self, cpu_s):
        self.children += float(cpu_s)

    @contextmanager
    def separate(self):
        start = self.clock()
        try:
            yield
        finally:
            self.diagnose_owned += max(0.0, self.clock()-start)

    def logbook(self, z):
        cpu = self.used
        z['cpu_total'] = cpu
        z['cpu_budget'] = dict(nominal=self.limit, children=self.children, owned=max(0.0, cpu-self.children),
                               rest=max(0.0, self.limit-cpu), overflow=max(0.0, cpu-self.limit))
        if cpu >= self.limit:
            z.update(cpu_time_limit=True, is_valid=False, cur_state='time_limit')
            if 'teacher_status' in z:
                z.update(teacher_status='unknown', teacher_checking=dict(status='unknown', reason='CPU budget exhausted'))
        return z
