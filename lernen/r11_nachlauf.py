""
import copy
from contextlib import contextmanager
import fcntl
import json
import math
import os
import resource
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def vorschlag(B, R, K, net, wachsen):
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from lernen import stromzonen as SZ
    from gemeinsam.fluss import analyse
    from gemeinsam.zonen import vereinheitlichen
    from analyse.kupfermodell import einstellungen
    R = vereinheitlichen(R)
    gnd = set(K.get('gnd') or [])
    clr, edge = einstellungen(K)
    outline = unary_union([Polygon(q) for q in B['outline']]).buffer(-edge)
    tracks = [t for t in R['tracks'] if t['net'] == net]
    flow = analyse(tracks, [v for v in R['vias'] if v['net'] == net],
                   [p for p in B['pads'] if p['net'] == net], K['netze'][net].get('fluss') or {})
    if not flow: return None
    bare = dict(R, zones=[z for z in R.get('zones', []) if z['net'] not in gnd])
    additions = {}
    for L in SZ._lagen(B):
        if L in SZ._gndlagen(R, None): continue
        seed = []
        for ti, t in enumerate(tracks):
            if t['layer'] != L or t.get('sig') or t.get('kelvin') or t.get('abg'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                share = flow['anteil'].get((ti, si), 0)
                if share <= 0 or a == b: continue

                seed.append(LineString([a, b]).buffer(w/2 + wachsen*math.sqrt(share), quad_segs=16))
        if not seed: continue
        foreign = unary_union([g for n, g in SZ._teile(B, bare, L) if n != net])
        free = outline.difference(foreign.buffer(clr+.005)).difference(SZ.sperrflaeche(B, L, 'flaechen'))
        free = free.difference(SZ.sperrflaeche(B, L, 'bahnen'))
        g = unary_union(seed).intersection(free)
        if not g.is_empty: additions[L] = g
    if not additions: return None
    out = copy.deepcopy(R)


    out['zones'] = [z for z in out.get('zones', [])
                    if not (z.get('quelle') == 'r11_korridor' and z['net'] == net
                            and z['layer'] in additions)]

    prio = 1 + max((z.get('prio', 0) for z in out.get('zones', [])), default=0)
    for L, g in additions.items():
        for p in SZ._polys(g):
            out.setdefault('zones', []).append(dict(net=net, layer=L, quelle='r11_korridor',
                pts=p['aussen'], holes=p['innen'], filled=True, full=True, r36=True,
                prio=prio, min_thickness=.025, local_clearance=None))
    out['stromzonen_modell'] = 'r36-dc-v1'
    return out


def regeltor(a, b):
    from analyse.regelkennzahlen import torfelder
    keys = set(torfelder(a['regeln'])) | set(torfelder(b['regeln']))
    missing, worse = [], []
    for key in sorted(keys):
        x, y = a['regeln'].get(key), b['regeln'].get(key)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y)):
            missing.append(key)
        elif y > x + 1e-8: worse.append(key)
    return dict(bestanden=isinstance(a.get('gueltig'), bool) and b.get('gueltig') is True and not missing and not worse,
                fehlt=missing, schlechter=worse)


def ausfuehren(bj, kj, rj, regeln, folder, cpu_s=240):
    def verbraucht():
        child = resource.getrusage(resource.RUSAGE_CHILDREN)
        return time.process_time() + child.ru_utime + child.ru_stime
    cpu_start = verbraucht()
    from analyse.bewertung import bewerte
    from analyse.r11 import messen
    from gemeinsam.zonen import boardgeometrie
    from lernen.kupfer_nativ import bibliotheksgeometrie, pruefen
    from lernen.stromzonen_nachlauf import physikvergleich
    from analyse.kupfermodell import masse_korridore
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj, kj, rj)]
    B = boardgeometrie(B, R)




    R['stromzonen_modell'] = 'r36-dc-v1'
    B0 = B
    e = bewerte(B, R, K, v3=True)
    diag = messen(K, e['metriken'], B)
    info = dict(angewendet=False, vorher=diag, versuche=[], grund='kein vollständiges Quellen-/Lastmodell')
    if diag['status'] != 'gemessen': return R, info
    info['grund'] = 'kein bestandener IR-/Regelkandidat'
    RG = json.loads(Path(regeln).read_text()) if regeln and Path(regeln).exists() else None



    B, padinfo = bibliotheksgeometrie(Path(folder)/'bibliothek', B, R, RG)
    info['padquelle'] = padinfo
    ranked = sorted(diag['netze'], key=lambda n: (-diag['netze'][n]['ziel_x'], -diag['netze'][n]['abfall_mV'], n))
    letzter_aufwand = 0
    for net in ranked[:2]:
        if diag['netze'][net]['strom_A'] <= 0: continue
        for growth in (.15, .35):
            rest = cpu_s - (verbraucht()-cpu_start)
            if rest < max(5, 1.2*letzter_aufwand):
                info['budget_erschoepft'] = True
                break
            versuch_start = verbraucht()
            candidate = vorschlag(B, R, K, net, growth)
            if candidate is None: continue
            path = Path(folder)/('%02d' % len(info['versuche']))
            path.mkdir(parents=True, exist_ok=True)
            try:
                a, b, native = pruefen(path, B, R, candidate, RG, cpu_s=min(120, int(rest)))
                a['native_padgeometrie'] = B['pads']
                b['native_padgeometrie'] = B['pads']
                ea, eb = [bewerte(B, r, K, v3=True) for r in (a, b)]
                physics = physikvergleich(B, a, B, b, K, ea['ir'], eb['ir'])
                gate = regeltor(ea, eb)
                original_gate = regeltor(e, eb)
                original_physics = physikvergleich(B0, R, B, b, K, e['ir'], eb['ir'])
                after = messen(K, eb['metriken'], B)
                ground = masse_korridore(B, a, b, K, RG)
                improve = (after['status'] == 'gemessen' and after['dc_x_max'] <= diag['dc_x_max']+1e-8
                           and eb['note_v3_netze'] < ea['note_v3_netze']-1e-8)
                good = (native['vorschlag']['gueltig'] and physics['bestanden'] and gate['bestanden']
                        and original_gate['bestanden'] and original_physics['bestanden']
                        and ground['bestanden'] and improve
                        and eb['note_v3_netze'] < e['note_v3_netze']-1e-8)
                row = dict(netz=net, wachsen_mm=growth, nativ=native, physik=physics,
                           regeltor=gate, eingabetor=original_gate, eingabephysik=original_physics,
                           masse=ground, vorher=messen(K, ea['metriken'], B), nachher=after,
                           note_v3_vorher=ea['note_v3_netze'], note_v3_nachher=eb['note_v3_netze'], angenommen=good)
                if good:
                    R, e, diag = b, eb, after
                    B0 = B
                    info.update(angewendet=True, grund='IR-/Regeltor bestanden')
            except Exception as exc:
                row = dict(netz=net, wachsen_mm=growth, angenommen=False, fehler=str(exc)[:500])
            info['versuche'].append(row)
            letzter_aufwand = verbraucht()-versuch_start
    info['nachher'] = diag
    return R, info


@contextmanager
def dateisperre(rj):
    ""
    with Path(str(Path(rj).resolve())+'.r11.lock').open('a+b') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def anwenden(bj, kj, rj, regeln=None, cpu_s=240):
    from gemeinsam import plaetze as PL
    rj = str(Path(rj).resolve())
    with dateisperre(rj):
        original = Path(rj).read_bytes()
    parent = Path(str(Path(rj).with_suffix(''))+'_r11')
    parent.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='lauf_', dir=parent))
    source, target, result = [folder/n for n in ('eingabe.json','kandidat.json','info.json')]
    source.write_bytes(original)
    target.unlink(missing_ok=True)
    result.unlink(missing_ok=True)
    cmd = [str(ROOT/'.venv/bin/python'), '-m', 'lernen.r11_nachlauf', bj, kj,
           str(source), regeln or '', str(target), str(result), str(cpu_s)]
    run = PL.lauf(cmd, cpu_s=cpu_s, cwd=str(ROOT), log=str(folder/'worker.log'),
                  env=dict(os.environ, PYTHONHASHSEED='0'))
    info = json.loads(result.read_text()) if run['zustand'] == 'ok' and result.exists() else dict(angewendet=False, grund=run['zustand'])
    info['rechenplatz'] = run
    info['arbeitsordner'] = str(folder)
    if info['angewendet']:
        with dateisperre(rj):
            if Path(rj).read_bytes() != original:
                info.update(angewendet=False, grund='Eingabe während Nachlauf geändert')
            else:
                with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                    tmp = f.name
                    f.write(target.read_bytes())
                try: os.replace(tmp, rj)
                finally:
                    if os.path.exists(tmp): os.unlink(tmp)
    result.write_text(json.dumps(info, indent=1))
    return info


if __name__ == '__main__':
    bj, kj, rj, rg, target, result, budget = sys.argv[1:]
    out, info = ausfuehren(bj, kj, rj, rg, Path(result).parent, cpu_s=float(budget))
    Path(target).write_text(json.dumps(out))
    Path(result).write_text(json.dumps(info, indent=1))
