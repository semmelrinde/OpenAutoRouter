""
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import time


def aktiv(value):
    if str(value) not in ('0', '1'): raise ValueError('lehrplan_tempo erwartet 0|1')
    return str(value) == '1'


def stufe(zeilen, fallback, rnd, version, achse=None):
    ""
    from lernen.lerneffekt import _grenz_ergebnis
    tasks = {}
    for z in zeilen[-3000:]:
        if str(z.get('version', '')).split('+')[0] != str(version): continue
        if z.get('nicht_geroutet') or z.get('generatorfehler') or z.get('lehrer_status') or z.get('rezept'): continue
        if z.get('stufe', 0) < 0 or _grenz_ergebnis(z) is None: continue
        if achse:
            art = 'bga' if any('BGA' in b.upper() for b in z.get('bloecke', [])) else 'qfn'
            if z.get('lagen') != achse['lagen'] or art != achse['gehaeuse']: continue
        s = z.get('stufe_aufgabe')
        if not isinstance(s, (int, float)) or not math.isfinite(s): continue
        key = (z.get('aufgabe'), s)
        tasks[key] = tasks.get(key, False) or z['gueltig']
    gruppen = {}
    for (_, s), ok in tasks.items(): gruppen.setdefault(s, []).append(ok)
    quote = {s: sum(g)/len(g) for s, g in gruppen.items() if len(g) >= 8}
    grenze = [s for s, q in sorted(quote.items()) if .2 <= q <= .8]
    if not grenze: return fallback

    alle = sorted(set(quote) | {fallback})
    return rnd.choice(grenze if rnd.random() < .8 else alle)


def armstrafe(buch, key, value):
    ""
    arme = buch.get('verschwendete_arme') or {}
    if not isinstance(arme, dict): return 0.
    row = arme.get('%s=%s' % (key, value), {})
    if not isinstance(row, dict): return 0.
    n, q = row.get('n'), row.get('ausfall')
    if not isinstance(n, int) or n < 20: return 0.
    if not isinstance(q, (int, float)) or not math.isfinite(q) or not .5 < q <= 1: return 0.
    return .25 * (q - .5) / .5


def fehlerbuch(pfad):
    try:
        b = json.loads(Path(pfad).read_text())
        return b if isinstance(b, dict) else {}
    except (OSError, ValueError): return {}




ROUTER = {'pfmul', 'router', 'form', 'fruehabbruch', 'fruehabbruch_modell'}
DATEIEN = ('placement.json', 'board.json', 'klassen.json')


def schluessel(ad, A, cfg, zeit, it, hashes):
    from lernen import meister as ME
    if cfg.get('meister', '1') != '0': return None
    c = {k: v for k, v in cfg.items() if k not in ROUTER | {'lehrplan_tempo', 'tempo_richter_modell'}}
    if 'kosten' in c: c['kosten'] = {k: v for k, v in c['kosten'].items() if k not in ME.ROUTER}
    inputs = {}
    for name in ('board.json', 'regeln.json', 'vorgaben.json', 'klassen.json'):
        p = Path(ad, name)
        inputs[name] = hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
    raw = dict(cfg=c, aufgabe=A, zeit=zeit, it=it, code=hashes, inputs=inputs)
    return hashlib.sha256(json.dumps(raw, sort_keys=True, allow_nan=False).encode()).hexdigest()


class Vorbereitung:
    ""
    def __init__(self): self.pool = {}; self.seeds = {}

    def seed(self, cfg):

        from lernen import meister as ME
        if not cfg.get('kosten') or cfg.get('meister') != '0': return
        key = {k: v for k, v in cfg.items() if k not in ROUTER | {'seed', 'kosten'}}
        key['kosten'] = {k: v for k, v in cfg['kosten'].items() if k not in ME.ROUTER}
        s = json.dumps(key, sort_keys=True)
        cfg['seed'] = self.seeds.setdefault(s, cfg['seed'])

    def holen(self, key, ziel, cfg, k):
        if key is None or key not in self.pool: return False
        quelle, z, hashes = self.pool[key]

        if not all((quelle/n).is_file() and hashlib.sha256((quelle/n).read_bytes()).hexdigest() == hashes[n] for n in DATEIEN): return False
        ziel = Path(ziel)
        if ziel == quelle or ziel.exists(): return False
        ziel.mkdir(parents=True)
        for n in DATEIEN: shutil.copyfile(quelle/n, ziel/n)
        z = copy.deepcopy(z)
        z.update(cfg=copy.deepcopy(cfg), kandidat=k, cpu_total=0., cpu_aufruf_s=0., s=0.,
                 tempo_cache=dict(quelle=str(quelle), cpu_vorbereitung_s=z.get('cpu_total')))


        (ziel/'vorbereitet.json').write_text(json.dumps(z))
        return True

    def merken(self, key, quelle, z):
        quelle = Path(quelle)
        if key is None or z.get('fehler') or z.get('code_geaendert') or not z.get('merkmale'): return
        if not all((quelle/n).is_file() for n in (*DATEIEN, 'vorbereitet.json')): return
        self.pool[key] = (quelle, copy.deepcopy(z), {n: hashlib.sha256((quelle/n).read_bytes()).hexdigest() for n in DATEIEN})


def richter_schatten(z, modell):
    from lernen.richter import Dienst
    dienst = Dienst(daten='', modell=modell, bilder=False)
    probe = {k: z[k] for k in ('merkmale', 'lagen', 'klasse') if k in z}
    dienst.anwenden(probe)
    return dict(art='schatten', ausschliessen=False, grund='keine Vorrouting-Kalibrierung',
                note=probe.get('note_ki'), status=probe.get('note_ki_status'), modell=probe.get('note_ki_modell'))


def versuch(cache, ad, A, cfg, k, zeit, it, **opts):
    from lernen import selbstspiel as SS
    if not aktiv(cfg.get('lehrplan_tempo', '0')) or opts.get('vorbereitet'):
        return SS.versuch(ad, A, cfg, k, zeit, it, **opts)
    cfg = copy.deepcopy(cfg)
    from lernen import meister as ME
    kosten = ME.parameter(cfg.get('kosten', {}))
    if 'form' in kosten: cfg['form'] = kosten['form']
    for source, target in (('placer_stil', 'placer'), ('router_stil', 'router')):
        if source in kosten and isinstance(cfg.get(target, 'alt'), str): cfg[target] = kosten[source]
    tc = time.thread_time()
    key = schluessel(ad, A, cfg, zeit, it, SS.code_hashes())
    c = Path(ad)/('k%d' % k)
    hit = cache.holen(key, c, cfg, k)
    overhead = time.thread_time() - tc
    prep = SS.versuch(ad, A, cfg, k, zeit, it, nur_platzieren=True, vorbereitet=hit)
    tc = time.thread_time()
    if not hit: cache.merken(key, c, prep)
    overhead += time.thread_time() - tc
    if opts.get('nur_platzieren') or not (c/'vorbereitet.json').exists():
        prep['cpu_aufruf_s'] = prep.get('cpu_aufruf_s', 0) + overhead
        prep['cpu_total'] = prep.get('cpu_total', 0) + overhead
        return prep
    z = SS.versuch(ad, A, cfg, k, zeit, it, vorbereitet=True, **opts)
    z['cpu_aufruf_s'] += prep.get('cpu_aufruf_s', 0) + overhead
    z['cpu_total'] = z['cpu_aufruf_s']
    return z
