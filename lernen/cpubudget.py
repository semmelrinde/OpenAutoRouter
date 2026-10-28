""
from contextlib import contextmanager
import math
import time


class Budget:
    def __init__(self, cpu_s, clock=None):
        self.limit = float(cpu_s)
        if not math.isfinite(self.limit) or self.limit <= 0:
            raise ValueError('CPU-Budget muss positiv und endlich sein')
        self.clock = clock or time.thread_time
        self.start = self.clock()
        self.kinder = 0.0
        self.diagnose_eigen = 0.0

    @property
    def verwendet(self):
        return self.kinder + max(0.0, self.clock() - self.start - self.diagnose_eigen)

    @property
    def rest(self):
        return max(0.0, self.limit - self.verwendet)

    def add(self, cpu_s):
        self.kinder += float(cpu_s)

    @contextmanager
    def separat(self):
        ""
        start = self.clock()
        try:
            yield
        finally:
            self.diagnose_eigen += max(0.0, self.clock()-start)

    def protokoll(self, z):
        cpu = self.verwendet
        z['cpu_total'] = cpu
        z['cpu_budget'] = dict(soll=self.limit, kinder=self.kinder, eigen=max(0.0, cpu-self.kinder),
                               rest=max(0.0, self.limit-cpu), ueberlauf=max(0.0, cpu-self.limit))
        if cpu >= self.limit:
            z.update(cpu_zeitlimit=True, gueltig=False, zustand='zeitlimit')
            if 'lehrer_status' in z:
                z.update(lehrer_status='unbekannt', lehrer_pruefung=dict(status='unbekannt', grund='CPU-Budget erschoepft'))
        return z
