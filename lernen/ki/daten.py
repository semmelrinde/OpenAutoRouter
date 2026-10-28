""
import contextlib
import datetime
import fcntl
import functools
import hashlib
import json
import math
import os
import re
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
from lernen.ki.merkmale_raster import merkmale_raster, ersatzmasse

VERSION = 2
BEOBACHTERVERSION = 3
LESEVERTRAG = 'beobachter-v3-suchbelegung-v3'
ZIEL_HASH_VERSION = 5
ROOT = Path(__file__).resolve().parents[2]
STANDARD = Path.home() / 'Library/Caches/Autorouter/datensatz'
X_SPALTEN = ('dc', 'j', 'rueck', 'xt_opfer', 'xt_stoer', 'laenge', 'umweg', 'vias',
             'c_last', 'l_last', 'sw_flaeche', 'hot_L', 'hot_A', 'masse',
             'auge_v', 'auge_h', 'auge_skew', 'stoer', 'leistung')


def ordner():
    return Path(os.environ.get('AUTOROUTER_DATENSATZ', str(STANDARD))).expanduser()


def board_hash(B):
    return hashlib.sha256(json.dumps(B, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


@functools.lru_cache(maxsize=64)
def _pruefhash(path, stamp, size):
    b = Path(path).read_bytes()
    return hashlib.sha256(b).hexdigest(), board_hash(json.loads(b))


def ausgeschlossen(path, B=None):
    ""
    p = Path(path); raw = hashlib.sha256(p.read_bytes()).hexdigest()
    canon = board_hash(B if B is not None else json.loads(p.read_bytes()))
    from lernen.lernkurve import ORDNER, STUFEN, SEEDS
    hashes = set()
    for st in STUFEN:
        for seed in SEEDS:
            ref = Path(ORDNER) / 'aufgaben' / ('s%d_%d' % (st, seed)) / 'board.json'
            stat = ref.stat()
            hashes.update(_pruefhash(str(ref), stat.st_mtime_ns, stat.st_size))
    return raw in hashes or canon in hashes


def _designname(f):

    f = re.sub(r'^\d+_', '', str(f)).lower().removesuffix('_eng')
    seed = re.match(r'erf_\d+', f)
    if seed: return seed.group()
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]

def familie(z):
    if z.get('designfamilie'): return _designname(z['designfamilie'])
    if z.get('aufgabe') is not None: return 'erf_%06d' % z['aufgabe']
    return _designname(z.get('familie') or z['board'])


@functools.lru_cache(maxsize=1)
def testfamilien():
    ""
    p = Path(__file__).with_name('familien_v1.json')
    data = json.loads(p.read_text())

    ids = data['testfamilien']
    return frozenset(ids)


def split_familie(f):
    f = _designname(f)
    if _designname(f) in {_designname(t) for t in testfamilien()}: return 'test'
    h = int(hashlib.sha256(('datenfabrik-split-v1:' + f).encode()).hexdigest()[:8], 16) % 10
    return 'test' if h == 0 else 'val' if h == 1 else 'train'


@contextlib.contextmanager
def sperre(root, exklusiv=True):
    root.mkdir(parents=True, exist_ok=True)
    with open(root / '.index.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX if exklusiv else fcntl.LOCK_SH)
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)


@functools.lru_cache(maxsize=8)
def _code(stamps):
    h = hashlib.sha256()
    for p, _, _ in stamps:
        h.update(str(Path(p).relative_to(ROOT)).encode()); h.update(Path(p).read_bytes())
    return h.hexdigest()


def code_hash():
    paths = sorted(p for d in ('lernen', 'router', 'platzierung', 'analyse', 'gemeinsam', 'kicad', 'nachbearbeitung')
                   for p in (ROOT / d).rglob('*.py') if '__pycache__' not in str(p))
    return _code(tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths))


def _halb(data):
    ""
    data = dict(data)
    for k in ('eingang', 'router_history_max', 'router_history_final', 'router_history_runden'):
        if k not in data: continue
        a = np.asarray(data[k], np.float32)
        mx = float(np.max(np.abs(a))) if a.size else 0.
        if not math.isfinite(mx): raise ValueError('nicht endliche Rasterkarte: ' + k)
        scale = max(1., 2. ** math.ceil(math.log2(max(1., mx / 60000.))))
        data[k] = (a / scale).astype(np.float16); data[k + '_skala'] = np.asarray(scale, np.float32)
    return data


def _index_anhaengen(root, row):
    ""
    path=root/'index.jsonl'
    with open(path,'a+b') as index:
        index.seek(0,2);size=index.tell()
        if size:
            index.seek(size-1)
            if index.read(1)!=b'\n':
                index.seek(max(0,size-65536));start=index.tell();tail=index.read()
                last=tail.rfind(b'\n');cut=start+last+1 if last>=0 else 0
                rest=tail[last+1:]
                try: json.loads(rest);complete=True
                except (ValueError,UnicodeError): complete=False
                if complete:
                    index.write(b'\n')
                else:
                    with open(root/'index_abgebrochen.jsonl','ab') as bad: bad.write(rest+b'\n')
                    index.truncate(cut)
        index.seek(0,2);start=index.tell()
        try:
            index.write((json.dumps(row,sort_keys=True)+'\n').encode())
            index.flush();os.fsync(index.fileno())
        except Exception:
            index.truncate(start)
            raise
        return start


def _quellbeobachter(data):
    observer = int(data.get('router_beobachterversion', 1))


    return int(data.get('router_quell_beobachterversion', 1 if observer == 4 else observer))


def zielvertrag(data):
    return dict(etikettenversion=VERSION, ziel_hash_version=ZIEL_HASH_VERSION,
                router_belegung_vertrag=str(data.get('router_belegung_vertrag', 'historisch_ohne_vertrag')),
                router_quell_beobachterversion=_quellbeobachter(data),
                router_beobachterversion=int(data.get('router_beobachterversion', 1)),
                router_etikettenversion=int(data.get('router_etikettenversion', 1)))


def ziel_hash(data):
    ""
    keys = ('router_history_max', 'router_history_final', 'router_history_runden',
            'router_belegung', 'router_belegung_suche', 'router_konflikt_runden', 'router_vias_raster',
            'router_netz_werte', 'router_netz_lagen', 'router_netz_konflikte', 'router_phasen',
            'physik_x', 'physik_maske', 'global_ziel', 'global_maske',
            'netz_verbunden', 'netz_verbunden_maske',
            'teil_konflikte', 'teil_konflikte_maske')

    contract = zielvertrag(data)
    zh = hashlib.sha256(json.dumps(contract, sort_keys=True).encode())

    keys = set(keys) | {k + '_maske' for k in keys}
    for key in sorted(k for k in keys if k in data):
        a = np.asarray(data[key]); zh.update(key.encode()); zh.update(str((a.dtype.str, a.shape)).encode()); zh.update(a.tobytes())
    return zh.hexdigest()


def schreiben(data, meta, root=None, vor_veroeffentlichen=None):
    ""
    root = Path(root or ordner()); meta = dict(meta, etikettenversion=VERSION)
    ziel_daten = data
    meta.update(zielvertrag(ziel_daten))
    meta['ziel_sha256'] = ziel_hash(ziel_daten)
    f = _designname(meta['designfamilie']); meta['designfamilie'] = f; meta['split'] = split_familie(f)
    if meta['herkunft'] not in ('selbstspiel', 'dauerlauf', 'pruefstand'): raise ValueError('Herkunft')
    day = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%d')
    meta.update(tag_utc=day, id=uuid.uuid4().hex)
    rel = Path('shards') / day / (meta['id'] + '.npz'); dest = root / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    data = _halb(data); data['meta_json'] = np.asarray(json.dumps(meta, sort_keys=True))
    fd, tmp = tempfile.mkstemp(dir=dest.parent, suffix='.npz'); os.close(fd)
    try:
        np.savez_compressed(tmp, **data)
        with open(tmp, 'rb') as file: os.fsync(file.fileno())
        with sperre(root):
            manifest = root / 'split_v1.json'
            expected = dict(version=1, regel='sha256(datenfabrik-split-v1:familie)%10: 0=test, 1=val, sonst=train',
                            testfamilien=sorted(testfamilien()))
            if manifest.exists():
                if json.loads(manifest.read_text()) != expected: raise ValueError('Familienmanifest geaendert; neue Datensatzversion erforderlich')
            else:
                manifest_tmp = root / ('.split-' + meta['id'])
                manifest_tmp.write_text(json.dumps(expected, indent=1)); os.replace(manifest_tmp, manifest)
            while True:
                checksum = hashlib.sha256(Path(tmp).read_bytes()).hexdigest()
                row = dict(meta, shard=str(rel), bytes=Path(tmp).stat().st_size, sha256=checksum,
                           etiketten_n=sum(int(data[k].size) for k in ('router_history_max', 'router_belegung',
                                  'router_konflikt_runden', 'router_vias_raster', 'router_netz_werte', 'router_netz_konflikte') if k in data)
                                  + int(np.count_nonzero(data.get('physik_maske', []))) + int(np.count_nonzero(data.get('teil_konflikte_maske', []))))
                geaendert = vor_veroeffentlichen is not None and vor_veroeffentlichen(data, meta)
                if not geaendert:
                    os.replace(tmp, dest)
                    try:
                        index_start = _index_anhaengen(root, row)
                    except Exception:
                        dest.unlink(missing_ok=True); raise



                    geaendert = vor_veroeffentlichen is not None and vor_veroeffentlichen(data, meta)
                    if not geaendert: break
                    with open(root / 'index.jsonl', 'r+b') as index:
                        index.truncate(index_start); index.flush(); os.fsync(index.fileno())
                    dest.unlink(missing_ok=True)
                meta['ziel_sha256'] = ziel_hash(ziel_daten)
                data['meta_json'] = np.asarray(json.dumps(meta, sort_keys=True))
                np.savez_compressed(tmp, **data)
                with open(tmp, 'rb') as file: os.fsync(file.fileno())
        return row
    finally:
        Path(tmp).unlink(missing_ok=True)


def physik(data, z, metrics):
    names = list(data['netz_namen']); x = np.full((len(names), len(X_SPALTEN)), np.nan, np.float32)
    nodes = (metrics or {}).get('netze') or {}
    for i, n in enumerate(names):
        for j, key in enumerate(X_SPALTEN):
            v = (nodes.get(n) or {}).get('x', {}).get(key)
            if isinstance(v, (float, int)) and math.isfinite(v): x[i, j] = v
    E = z.get('ergebnis') or {}; rule = z.get('regel') or {}
    routed = 'router_netz_werte' in data
    unv = (metrics or {}).get('unverbunden')
    from analyse.strom_engstellen import lernwerte
    r39 = lernwerte(rule.get('strom_engstellen'))
    r39_complete = bool(r39) and (r39.get('abdeckung') or {}).get('vollstaendig', True) and (r39.get('abdeckung') or {}).get('komponenten') != 0
    plane_nets = set((metrics or {}).get('flaechennetze') or [])
    data['netz_verbunden'] = np.asarray([n not in (unv or []) for n in names], bool)
    data['netz_verbunden_maske'] = np.asarray([unv is not None and n not in plane_nets and bool(n) and not n.startswith('unconnected') for n in names], bool)
    from lernen.lerneffekt import _grenz_ergebnis
    known = _grenz_ergebnis(z) is not None
    from lernen.fruehabbruch import abgebrochen
    data.update(physik_x=x, physik_maske=np.isfinite(x), physik_spalten=np.asarray(X_SPALTEN),
                global_ziel=np.asarray([np.nan if abgebrochen(z) else float(bool(z.get('gueltig'))), z.get('note_v3') if z.get('note_v3') is not None else np.nan], np.float32),
                global_maske=np.asarray([routed and known,
                                         routed and known and z.get('note_v3') is not None], bool),
                physik_meta_json=np.asarray(json.dumps(dict(minimax_status=rule.get('minimax_status'),
                     minimax_gueltig=rule.get('minimax_gueltig'), minimax_vollstaendig=rule.get('minimax_vollstaendig'),
                     minimax_rang=rule.get('minimax_rang'), sicherheit=rule.get('sicherheit'),
                     note_v3_version=rule.get('version'), note_v3_voll=(metrics or {}).get('note_v3_voll'),
                     native_drc=E.get('drc'),
                     topologie=rule.get('topologie'), r27=rule.get('r27'), kopplung=rule.get('kopplung'),
                     **({'k20': rule['k20']} if 'k20' in rule else
                        ({'k20': rule['regeln']['k20']} if 'k20' in (rule.get('regeln') or {}) else {})),
                     strom_engstellen=r39,
                     r39_abdeckung=rule.get('r39_abdeckung'),
                     strom_engstellen_mm2=rule.get('strom_engstellen_mm2') if r39_complete else None,
                     strom_begrenzt=z.get('v3_strom_begrenzt') or (metrics or {}).get('strom_begrenzt'),
                     unverbunden=unv,
                     modellannahmen='analyse/metriken; keine Hardwaremessung',
                     form=z.get('form')), sort_keys=True)))


def aufnehmen(c, z, quellboard, herkunft='selbstspiel', metrics=None, eingang=None, board=None, klassen=None, etiketten=None, drc_path=None, cpu_budget=None):
    ""
    t0 = time.thread_time(); c = Path(c); r44_cpu = 0.0
    try:
        if ausgeschlossen(quellboard): return dict(zustand='ausgeschlossen_lernkurve')
        bj = Path(board) if board else c / 'board.json'
        if not bj.exists(): bj = Path(quellboard)
        B = json.loads(bj.read_text()); kj = Path(klassen) if klassen else c / 'klassen.json'
        if not kj.exists(): kj = Path(quellboard).parent / 'klassen.json'
        K = json.loads(kj.read_text()) if kj.exists() else {}
        if ausgeschlossen(bj, B): return dict(zustand='ausgeschlossen_lernkurve')
        gnd = z.get('gndlagen') or []
        data = dict(eingang or merkmale_raster(B, K, gndlagen=gnd))
        ep = Path(etiketten) if etiketten else c / 'etiketten.npz'
        if etiketten is None and not ep.exists(): ep = c / 'routes_etiketten.npz'
        if ep.exists():
            rp = c / 'routes.json'
            if rp.exists():
                from nachbearbeitung.kurzschluss_schutz import etiketten_sichern, schutz_an
                rr = json.loads(rp.read_text())

                if schutz_an(rr.get('optionen')):
                    start = time.thread_time()
                    try: etiketten_sichern(B, rr, rp, ep)
                    finally: r44_cpu += time.thread_time()-start
        if ep.exists():
            with np.load(ep, allow_pickle=False) as r:
                if not np.allclose(r['origin_mm'], data['origin_mm']) or r['belegung'].shape[-2:] != data['eingang'].shape[-2:]:
                    raise ValueError('Eingang/Zielraster verschieden')
                for key in r.files: data['router_' + key] = r[key]

            grid = data['router_konflikt_runden']; ls = list(data['router_lagen']); origin = data['origin_mm']
            near = np.zeros(len(data['teil_namen']), np.uint32)
            from lernen.ki.merkmale_raster import geometrie, mal
            courts = {q['ref']: q for q in B.get('courtyards', [])}
            for i, (x, y, _, side, *_) in enumerate(data['teil_werte']):
                lay = str(data['lagen'][int(side)])
                if lay not in ls: continue
                cy_geo = courts.get(str(data['teil_namen'][i]))
                if cy_geo:
                    m = mal(geometrie(cy_geo['polys']).buffer(1), origin, grid.shape[-2:])
                    near[i] = grid[ls.index(lay)][m].sum()
                    continue
                cx, cy = np.floor((np.array([x, y]) - origin) / .25).astype(int)
                yy, xx = np.mgrid[max(0, cy - 4):min(grid.shape[1], cy + 5), max(0, cx - 4):min(grid.shape[2], cx + 5)]
                if xx.size:
                    m = (xx - cx) ** 2 + (yy - cy) ** 2 <= 16
                    near[i] = grid[ls.index(lay), yy[m], xx[m]].sum()
            data['teil_konflikte'] = near
            data['teil_konflikte_maske'] = np.ones(len(near), bool)
        else:
            data['teil_konflikte'] = np.zeros(len(data['teil_namen']), np.uint32)
            data['teil_konflikte_maske'] = np.zeros(len(data['teil_namen']), bool)
        if cpu_budget is not None:
            from lernen.lerneffekt import ergaenzen
            cpu_budget.protokoll(z)
            ergaenzen(z, c)
        physik(data, z, metrics)
        if z.get('lerndaten'):
            data['lerndaten_json'] = np.asarray(json.dumps(z['lerndaten'], sort_keys=True))
            data['board_json'] = np.asarray(json.dumps(B, sort_keys=True))
            data['klassen_json'] = np.asarray(json.dumps(K, sort_keys=True))
            for name in ('routes', 'placement'):
                raw = c / (name + '.json')
                if raw.exists(): data[name + '_json'] = np.asarray(raw.read_text())


            if z['lerndaten'].get('zensiert'):
                data['global_maske'][:] = False
        drc = Path(drc_path) if drc_path else c / 'fertig.kicad_pcb.drc.json'
        if drc.exists(): data['native_drc_json'] = np.asarray(drc.read_text())
        rg_path = c / 'regeln.json'
        if rg_path.exists(): data['regeln_json'] = np.asarray(rg_path.read_text())
        meta = dict(aufgabe=z.get('aufgabe', z.get('board')), designfamilie=familie(z), seed=z.get('cfg', {}).get('seed'),
                    kandidat=z.get('kandidat'), code_sha256=z.get('code_sha256') or code_hash(),
                    router_sha256=hashlib.sha256((ROOT / 'router/route_pf.py').read_bytes()).hexdigest(),
                    klassen_sha256=hashlib.sha256((ROOT / 'analyse/klassen.py').read_bytes()).hexdigest(),
                    herkunft=herkunft, board_sha256=hashlib.sha256(bj.read_bytes()).hexdigest(),
                    quellboard_sha256=hashlib.sha256(Path(quellboard).read_bytes()).hexdigest(),
                    cfg=z.get('cfg'), cpu_s=sum((z.get(p) or {}).get('cpu_s', 0) or 0 for p in ('placer', 'router')),
                    phase='routing' if ep.exists() else 'platzierung', zustand=z.get('zustand'),
                    code_geaendert=z.get('code_geaendert', False), ersatzmasse=z.get('ersatzmasse'),
                    vergleichsgruppe=z.get('vergleichsgruppe'), paar_mit=z.get('paar_mit'),
                    sammler_cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)
        def budget_abschluss(data, meta):


            vorher = bool(z.get('cpu_zeitlimit'))
            cpu_budget.protokoll(z)
            if vorher == bool(z.get('cpu_zeitlimit')): return False
            ergaenzen(z, c)
            from lernen.fruehabbruch import abgebrochen
            data['global_ziel'][0] = np.nan if abgebrochen(z) else 0
            data['global_maske'][:] = False
            data['lerndaten_json'] = np.asarray(json.dumps(z['lerndaten'], sort_keys=True))
            meta['zustand'] = z['zustand']
            return True
        row = schreiben(data, meta, **({'vor_veroeffentlichen': budget_abschluss} if cpu_budget is not None else {}))
        return dict(zustand='ok', shard=row['shard'], bytes=row['bytes'], etiketten_n=row['etiketten_n'], ziel_sha256=row.get('ziel_sha256'), ziel_hash_version=row.get('ziel_hash_version', ZIEL_HASH_VERSION), router_beobachterversion=row.get('router_beobachterversion'),
                    cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)
    except Exception as e:
        return dict(zustand='fehler', fehler=repr(e)[:250], cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)


def index_lesen(root):
    ""
    root = Path(root)
    with sperre(root, exklusiv=False):
        raw = (root / 'index.jsonl').read_bytes()
    rows = []
    for line in raw.splitlines(keepends=True):
        try: row = json.loads(line)
        except ValueError:
            if not line.endswith(b'\n'): break
            raise
        rows.append(row)
    return rows


def _altziele(data, row, observer):
    ""
    phases = data.get('router_phasen')
    prep = phases is None or bool(np.any(phases == 'erstaufbau'))
    if phases is not None:
        keep = phases != 'erstaufbau'
        for key in ('router_phasen', 'router_durchgaenge', 'router_netz_konflikte', 'router_history_runden'):
            if key in data:
                if len(data[key]) != len(keep): raise ValueError('Beobachterrunden ungleich')
                data[key] = data[key][keep]
    if prep:
        if 'router_history_runden' in data and len(data['router_history_runden']):
            data['router_history_max'] = data['router_history_runden'].max(axis=0)
        elif 'router_history_max' in data:
            data['router_history_max_maske'] = np.zeros_like(data['router_history_max'], bool)
        for key in ('router_konflikt_runden', 'teil_konflikte'):
            if key in data: data[key + '_maske'] = np.zeros_like(data[key], bool)
    if 'router_netz_werte' in data:
        mask = np.asarray(data.get('router_netz_werte_maske', np.isfinite(data['router_netz_werte'])), bool).copy()
        if observer < 2 or prep: mask[:, 6] = False
        data['router_netz_werte_maske'] = mask
    data['router_quell_beobachterversion'] = np.asarray(observer)
    data['router_beobachterversion'] = np.asarray(BEOBACHTERVERSION)
    data['lesevertrag'] = np.asarray(LESEVERTRAG)


def _globale_masken(data, row):
    ""
    if 'global_maske' not in data: return
    from lernen.lerneffekt import _grenz_ergebnis
    z = dict(row)
    for key in ('lerndaten_json', 'physik_meta_json'):
        if key in data:
            info = json.loads(str(data[key]))
            if key == 'lerndaten_json':
                z['lerndaten'] = info
                if info.get('fehler'): z['fehler'] = info['fehler']
                z.update(info.get('rechenlaeufe') or {})
            else: z['form'] = info.get('form') or {}
    targets = data.get('global_ziel')
    if targets is not None:
        from lernen.fruehabbruch import abgebrochen
        if abgebrochen(z):
            targets[0] = np.nan
        data['global_maske'] = np.asarray(data['global_maske'], bool) & np.isfinite(targets)
        if targets[0] in (0, 1): z['gueltig'] = bool(targets[0])
    if _grenz_ergebnis(z) is None: data['global_maske'] = np.zeros_like(data['global_maske'], bool)
    if (z.get('lerndaten') or {}).get('zensiert') and 'router_netz_werte' in data:
        mask = np.asarray(data.get('router_netz_werte_maske', np.isfinite(data['router_netz_werte'])), bool).copy()
        mask[:, 0] = False
        data['router_netz_werte_maske'] = mask


def _r44_suchbelegung(data):
    ""
    data['router_belegung_ausgabe'] = data['router_belegung']
    alt = data.pop('router_belegung_maske', None)
    if alt is not None: data['router_belegung_ausgabe_maske'] = alt
    data['router_belegung'] = data['router_belegung_suche']
    if 'router_belegung_suche_maske' in data: data['router_belegung_maske'] = data['router_belegung_suche_maske']
    elif alt is not None: data['router_belegung_maske'] = np.asarray(alt, bool).copy()
    data['router_belegung_vertrag'] = np.asarray('suchbelegung_anteil')


class RasterDataset:
    ""
    def __init__(self, root=None, split='train', version=None, phase='routing', pruefe_hash=False, beobachterversion=BEOBACHTERVERSION, rows=None):
        self.root = Path(root or ordner()); self.rows = []; self.pruefe_hash = pruefe_hash
        self.beobachterversion = beobachterversion


        versions = {1, VERSION} if version is None else {version}
        cohorts = {}
        if split not in ('train', 'val', 'test', 'alle'): raise ValueError('split')
        if rows is None: rows = index_lesen(self.root)
        for row in rows:
            if row['split'] != split_familie(row['designfamilie']): raise ValueError('Familien-Leck im Index')
            observer = row.get('router_beobachterversion') if row['phase'] == 'routing' else None
            selected = (not row.get('code_geaendert') and row['etikettenversion'] in versions
                        and (not phase or row['phase'] == phase)
                        and (split == 'alle' or row['split'] == split))
            if selected and row['phase'] == 'routing':
                if beobachterversion == BEOBACHTERVERSION:
                    selected = observer in (None, 1, 2, 4, BEOBACHTERVERSION)
                else:
                    if observer is None:
                        with np.load(self.root / row['shard'], allow_pickle=False) as shard:
                            observer = int(shard.get('router_beobachterversion', 1))
                    selected = observer == beobachterversion
            key = (row['etikettenversion'], observer, row['phase'], row['split'], bool(row.get('code_geaendert')))
            counts = cohorts.setdefault(key, [0, 0]); counts[0] += 1; counts[1] += int(selected)
            if selected: self.rows.append(row)
        self.kohorten = [dict(etikettenversion=k[0], router_beobachterversion=k[1], phase=k[2],
                             split=k[3], code_geaendert=k[4], gesamt=n[0], ausgewaehlt=n[1])
                         for k, n in sorted(cohorts.items(), key=lambda item: str(item[0]))]

    def __len__(self): return len(self.rows)

    def __getitem__(self, i):
        import torch
        row = self.rows[i]; path = self.root / row['shard']
        if self.pruefe_hash and hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']: raise ValueError('Shard-Hash')
        with np.load(path, allow_pickle=False) as z: data = {k: z[k] for k in z.files}
        for k in list(data):
            if k.endswith('_skala'):
                target = k.removesuffix('_skala'); data[target] = data[target].astype(np.float32) * data[k]
        observer = int(data.get('router_beobachterversion', 1))
        if row['phase'] == 'routing':
            if row.get('router_beobachterversion', observer) != observer:
                raise ValueError('Beobachtervertrag zwischen Index und Shard abweichend')
            if self.beobachterversion == BEOBACHTERVERSION and observer in (1, 2):
                _altziele(data, row, observer)
            elif observer == 4 and self.beobachterversion == BEOBACHTERVERSION:
                if str(data.get('router_belegung_vertrag', '')) != 'ausgabekupfer_zellkontakt' or 'router_belegung_suche' not in data:
                    raise ValueError('R44-Belegungssemantik fehlt')
                _r44_suchbelegung(data)
                source = _quellbeobachter(data)
                if source in (1, 2): _altziele(data, row, source)
                data['router_quell_beobachterversion'] = np.asarray(source)
                data['router_beobachterversion'] = np.asarray(BEOBACHTERVERSION)
                data['lesevertrag'] = np.asarray(LESEVERTRAG)
            elif observer != self.beobachterversion:
                raise ValueError('Unbekannter Beobachtervertrag')
        _globale_masken(data, row)

        if 'router_history_max' in data:
            ls=list(data['lagen']); rs=list(data['router_lagen']); sh=data['eingang'].shape[-2:]
            maps=('router_history_max','router_belegung','router_konflikt_runden','router_vias_raster')
            target=np.full((len(ls),len(maps),*sh),np.nan,np.float32)
            mask=np.zeros_like(target,dtype=bool)
            for j,l in enumerate(rs):
                if l not in ls: continue
                i=ls.index(l)
                for k,key in enumerate(maps): target[i,k]=data[key][j]
                for k,key in enumerate(maps):
                    mask[i,k]=(data['eingang'][i,0]>0) & np.isfinite(data[key][j])
                    if key+'_maske' in data: mask[i,k] &= data[key+'_maske'][j]
            data['ziel_raster']=target;data['ziel_raster_maske']=mask
            data['ziel_raster_spalten']=np.asarray(['history_max','belegung','konflikt_runden','vias'])
            names_n=list(data['netz_namen']); routed_names=list(data['router_netze'])
            target_n=np.full((len(names_n),data['router_netz_werte'].shape[1]),np.nan,np.float32)
            mask_n=np.zeros_like(target_n,dtype=bool)
            for j,n in enumerate(routed_names):
                if n in names_n:
                    i=names_n.index(n);target_n[i]=data['router_netz_werte'][j];mask_n[i]=np.isfinite(target_n[i])
                    if 'router_netz_werte_maske' in data: mask_n[i] &= data['router_netz_werte_maske'][j]
                    if observer < 2:
                        mask_n[i,6]=False
            data['ziel_netz']=target_n;data['ziel_netz_maske']=mask_n
        tensors = {}; names = {}
        for k, v in data.items():
            if v.dtype.kind in 'bifuc':
                a = v.astype(np.int64) if v.dtype.kind == 'u' else v
                tensors[k] = torch.from_numpy(a.copy())
            else: names[k] = v.tolist()
        return dict(tensoren=tensors, namen=names, meta=row)

    @staticmethod
    def collate(samples): return samples



Dataset = RasterDataset


def importieren(c, z, quellboard):
    ""
    return aufnehmen(c, z, quellboard, herkunft='pruefstand')


def route_kontext(o, c):
    ""
    try:
        c = Path(c); rg = json.loads((Path(o) / 'regeln.json').read_text())
        rg['router']['etiketten'] = str((c / 'etiketten.npz').resolve())
        (c / 'regeln.json').write_text(json.dumps(rg))
        return str(c)
    except Exception as e:
        print('Etiketten: Regelkopie ausgelassen:', repr(e)[:180], flush=True)
        return o


def kandidaten(c, z, quellboard, n=3, automatisch=True, cpu_budget=None):
    ""
    import random
    from gemeinsam.verschieben import bewegen
    from lernen.ki.merkmale_raster import geometrie
    c = Path(c); t0 = time.thread_time(); rows = []
    try:
        if n <= 1 or ausgeschlossen(quellboard): return dict(kandidaten=0, cpu_s=0.)
        B = json.loads((c / 'board.json').read_text()); K = json.loads((c / 'klassen.json').read_text())
        rg = json.loads((c / 'regeln.json').read_text()) if (c / 'regeln.json').exists() else {}
        base_cpu = sum((z.get(s) or {}).get('cpu_s', 0) or 0 for s in ('placer', 'router'))
        budget = max(0., base_cpu * .09 - (z.get('datensatz') or {}).get('cpu_s', 0))

        group = '%s:%s:%s' % (familie(z), hashlib.sha256(Path(quellboard).read_bytes()).hexdigest(), z.get('kandidat'))
        rnd = random.Random((z.get('cfg') or {}).get('seed', 0) + 104729)
        fixed = {f['ref'] for f in B['fps'] if f.get('locked') or (K.get('teile') or {}).get(f['ref'], {}).get('fest')}
        refs = {f['ref'] for f in B['fps']}
        groups = []
        covered = set(); blocked = set()
        for owner, isl in sorted((K.get('inseln') or {}).items()):
            g = ({owner} | set(isl.get('mitglieder') or [])) & refs
            covered |= g
            if g & fixed: blocked |= g
            elif g: groups.append(sorted(g))
        groups = [g for g in groups if not set(g) & blocked]
        groups += [[r] for r in sorted(refs - covered - fixed)]
        if not groups: return dict(kandidaten=0, cpu_s=time.thread_time() - t0)
        estimate = .5
        for k in range(1, n):
            if cpu_budget is not None and cpu_budget.rest <= 0: break
            if automatisch and time.thread_time() - t0 + estimate > budget: break
            tc = time.thread_time(); g = rnd.choice(groups)
            dx, dy = rnd.choice(((-.5, 0), (.5, 0), (0, -.5), (0, .5), (-.25, -.25), (.25, .25)))
            BB, _ = bewegen(B, {r: (dx, dy, 0) for r in g})
            m, cheap = ersatzmasse(BB, K, rg, z.get('gndlagen'))
            outline = geometrie(BB['outline'])
            cheap['teile_aussen'] = sum(1 for cy in BB.get('courtyards', []) if geometrie(cy['polys']).difference(outline).area > 1e-8)

            tmp = c / ('etiketten_kandidat_%d' % k); tmp.mkdir(exist_ok=True)
            (tmp / 'board.json').write_text(json.dumps(BB)); (tmp / 'klassen.json').write_text(json.dumps(K))
            zz = dict(z, kandidat=str(z.get('kandidat')) + ':ersatz%d' % k, vergleichsgruppe=group,
                      paar_mit=z.get('kandidat'), ersatzmasse=cheap, gueltig=None, note_v3=None,
                      ergebnis=None, regel=None, zustand='nur_ersatzmasse')
            row = aufnehmen(tmp, zz, quellboard, metrics=None, eingang=m)
            rows.append(row)
            import shutil
            shutil.rmtree(tmp)
            estimate = max(.01, (time.thread_time() - tc) * 1.2)
        return dict(kandidaten=len(rows), cpu_s=time.thread_time() - t0, vergleichsgruppe=group,
                    budget_cpu_s=budget if automatisch else None, shards=rows)
    except Exception as e:
        return dict(kandidaten=len(rows), cpu_s=time.thread_time() - t0, fehler=repr(e)[:200])


def _native_gueltig(E, drc):

    from analyse.korpus import gueltig_voll
    if drc is None: return False
    try: return gueltig_voll(dict(E, drc=drc), True)
    except (KeyError, TypeError): return False


def pruefstand_aufnehmen(o, res):
    ""
    from analyse.bewertung import bewerte
    o = Path(o); rows = []
    for route in sorted([o / 'unser.json', *o.glob('auto*.json')]):
        if not (route.stem == 'unser' or route.stem == 'auto' or route.stem.startswith('auto_k')): continue
        ep = route.with_name(route.stem + '_etiketten.npz')
        if not route.exists() or not ep.exists(): continue
        tag = route.stem.removeprefix('auto')
        bj = o / ('board.json' if route.stem == 'unser' else 'auto_board%s.json' % tag)
        kj = o / ('klassen.json' if route.stem == 'unser' else 'auto_klassen%s.json' % tag)
        if not bj.exists() or not kj.exists(): continue
        try:
            B = json.loads(bj.read_text()); K = json.loads(kj.read_text()); rr = json.loads(route.read_text())

            z0 = res.get('unser') if route.stem == 'unser' else next((e for e in res.get('_etiketten_auto', []) if e.get('kandidat') == (tag or 'k0')), None)
            if z0 and z0.get('repariert'):
                ep = o / '_kein_rohziel.npz'
                if route.stem.startswith('auto'):
                    rep_board = o / ('rep%s' % tag) / 'board_nach.json'
                    if rep_board.exists(): bj = rep_board
                    else: continue
            if bj != o / ('board.json' if route.stem == 'unser' else 'auto_board%s.json' % tag): B = json.loads(bj.read_text())
            E = bewerte(B, dict(rr, **({'drc': z0['drc']} if z0 and z0.get('drc') else {})), K, v3=True)
            z = dict(board=res['board'], designfamilie=_designname(res['board']), kandidat=route.stem,
                     cfg=dict(seed=res.get('_etiketten_seed', 1)), gueltig=_native_gueltig(E, (z0 or {}).get('drc')), note_v3=E.get('note_v3_netze'),
                     regel={**(E.get('regeln') or {}), 'minimax_status': E.get('minimax_status')},
                     ergebnis=dict(drc=(z0 or {}).get('drc')), gndlagen=rr.get('gndlagen', []), zustand='ok')
            metrics = dict(E.get('metriken') or {}, note_v3_voll=E.get('note_v3'), unverbunden=E['unverbunden'], flaechennetze=rr.get('flaechennetze', []))
            drc_path = o / ('unser.kicad_pcb.drc.json' if route.stem == 'unser' else 'auto_fertig%s.kicad_pcb.drc.json' % tag)
            rows.append(aufnehmen(o, z, o / 'board.json', 'pruefstand', metrics, board=bj, klassen=kj, etiketten=ep, drc_path=drc_path))
        except Exception as e: rows.append(dict(zustand='fehler', fehler=repr(e)[:200]))
    return rows
