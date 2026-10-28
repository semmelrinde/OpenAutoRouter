""
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def export_zonen(R):
    from collections import defaultdict
    import uuid
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
    from gemeinsam.zonen import vereinheitlichen
    r=copy.deepcopy(vereinheitlichen(R))
    groups=defaultdict(list);kept=[]
    for z in r.get('zones',[]):
        if z.get('quelle')=='R36-Vollkupfermodell':groups[z['net'],z['layer']].append(z)
        else:kept.append(z)
    for (net,layer),zones in sorted(groups.items()):
        parameters=('full','min_thickness','local_clearance','prio')
        if len({tuple(z.get(k) for k in parameters) for z in zones})!=1:
            raise ValueError('R39: unterschiedliche Zonenparameter bei '+net+'/'+layer)
        g=unary_union([Polygon(z['pts'],z.get('holes',[])).buffer(0) for z in zones])
        contours=[]
        for poly in sorted(getattr(g,'geoms',[g]),key=lambda p:(p.bounds,p.area)):
            if poly.is_empty or poly.geom_type!='Polygon':continue
            poly=orient(poly,sign=1)
            contours.append(dict(pts=list(poly.exterior.coords),holes=[list(i.coords) for i in poly.interiors]))
        meta={k:v for k,v in zones[0].items() if k not in ('pts','holes','native_outlines','zone_id')}
        meta.update(native_layers=[layer],native_outlines=contours,
                    zone_id=str(uuid.uuid5(uuid.NAMESPACE_URL,'autorouter:r39-export:'+net+':'+layer)))
        kept.extend(dict(meta,**p) for p in contours)
    r['zones']=kept
    return r


def unbekannttor(B, vorher, nachher, messung):
    ""
    from lernen.kupferbalken import kupfer
    from shapely.geometry import box, Polygon
    a, b = kupfer(B, vorher), kupfer(B, nachher)
    for row in messung.get('nicht_messbar', []):
        key = row['netz'], row['lage']
        region = box(*row['bounds']).buffer(1e-6)
        if a.get(key, Polygon()).symmetric_difference(b.get(key, Polygon())).intersection(region).area > 1e-8:
            return False
    return True


def exportgleich(B, vorher, nachher):
    ""
    from lernen.kupferbalken import kupfer
    from gemeinsam.zonen import vereinheitlichen
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    representation = {'zones', 'zonen'}
    canonical = vereinheitlichen(vorher)
    if ({k:v for k,v in canonical.items() if k not in representation} !=
            {k:v for k,v in nachher.items() if k not in representation}): return False


    groups = {}
    for z in nachher.get('zones', []):
        if z.get('quelle') == 'R36-Vollkupfermodell':
            groups.setdefault((z['net'], z['layer']), []).append(z)
    for (_, layer), zones in groups.items():
        filled = unary_union([Polygon(z['pts'], z.get('holes', [])) for z in zones])
        for z in zones:
            if z.get('native_layers') != [layer] or not z.get('native_outlines'): return False
            outlines = unary_union([Polygon(q['pts'], q.get('holes', [])) for q in z['native_outlines']])
            if filled.symmetric_difference(outlines).area > 1e-8: return False
    a, b = kupfer(B, vorher), kupfer(B, nachher)
    return all(a.get(key, Polygon()).symmetric_difference(b.get(key, Polygon())).area <= 1e-8
               for key in set(a) | set(b))


def pfadtor(a, b):
    ""
    return (a['version'] == b['version'] and a['parameter'] == b['parameter']
            and a['abdeckung'] == b['abdeckung']
            and a['nicht_messbar'] == b['nicht_messbar']
            and b['unnoetig_mm2'] < a['unnoetig_mm2'] - .001)


def regeltor(a, b):
    from analyse.regelkennzahlen import annahme, torfelder
    gate = annahme(a, b)

    return gate['nicht_schlechter'] and all(
        isinstance(a.get(k), (int, float)) and isinstance(b.get(k), (int, float))
        and math.isfinite(a[k]) and math.isfinite(b[k]) and b[k] <= a[k] + 1e-9
        for k in set(torfelder(a)) | set(torfelder(b)))


def vorschlag(B, R, K, netz, gndlagen):
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    from lernen import stromzonen as SZ
    candidate = copy.deepcopy(R)


    candidate['zonen'] = [z for z in candidate.get('zonen', []) if z.get('quelle') != 'gnd_fuellung']
    candidate['vias'] = [v for v in candidate.get('vias', []) if v.get('quelle') != 'gnd_fuellung']
    from gemeinsam.zonen import vereinheitlichen
    candidate = copy.deepcopy(vereinheitlichen(candidate))
    extra = []
    outline = unary_union([Polygon(q).buffer(0) for q in B['outline']]).buffer(-.25, SZ.QS)
    clr = max(.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or .1))
    _, drill = SZ._via_masse(R)
    hole = float((R.get('optionen') or {}).get('h2h') or .2)
    for z in candidate.get('zones', []):
        if not z.get('r36') or z['net'] != netz: continue
        L = z['layer']
        copper = SZ._teile(B, candidate, L)
        own = unary_union([g for n, g in copper if n == netz])
        foreign = unary_union([g for n, g in copper if n != netz])
        holes = [Point(p['x'], p['y']).buffer(float(p['drill'])/2, SZ.QS)
                 for p in B['pads'] if p.get('drill') and p['net'] != netz]
        holes += [Point(v['x'], v['y']).buffer(float(v.get('drill') or drill)/2, SZ.QS)
                  for v in candidate['vias'] if v['net'] != netz and L in (v.get('layers') or SZ._lagen(B))]
        free = outline.difference(foreign.buffer(clr+.003, SZ.QS))
        free = free.difference(unary_union(holes).buffer(hole+.003, SZ.QS))
        free = free.difference(SZ.sperrflaeche(B, L, 'flaechen'))
        old = Polygon(z['pts'], z.get('holes', [])).buffer(0)
        g = SZ.r39_korridor(B, candidate, K, L, netz, old, own, free)
        delta = g.difference(old)
        if not delta.is_empty:
            extra.append(dict(net=netz, layer=L, quelle='r39_korridor',
                polys=SZ._polys(delta), flaeche_mm2=round(delta.area, 4)))
    candidate['zonen'] += extra
    if any(z.get('quelle') == 'gnd_fuellung' for z in R.get('zonen', [])):
        gz, gv = SZ.gnd_fuellung(B, candidate, K, gndlagen)
        candidate['zonen'] += gz
        candidate['vias'] += gv
    return candidate


def reparieren(B, R, K, gndlagen=None, versuche=3):
    from shapely.geometry import Polygon
    from analyse.strom_engstellen import engstellen
    from lernen.form import gueltig
    from lernen.sammeln import regelmessung
    from lernen.stromzonen import pruefen
    from lernen.kupferbalken import kupfer
    from analyse.kupfermodell import ir_werte
    from lernen.stromzonen_nachlauf import physiktor
    gl = gndlagen if gndlagen is not None else R.get('gndlagen') or []
    baseline = copy.deepcopy(R)

    baseline['optionen'] = dict(baseline.get('optionen') or {}, r39_diagnose='1')
    E = engstellen(B, baseline, K, gl)
    info = dict(angewendet=False, native_abnahme=False, messversion=E['version'], vorher=E, versuche=[])
    valid, _ = gueltig(B, K, None, gl, routes=baseline)
    if not valid:
        info['grund'] = 'Ungültiges Routing: kein zusätzliches Stromkupfer'
        info['nachher'] = E
        return baseline, info
    scores = {}
    for u in E['unnoetig']: scores[u['netz']] = scores.get(u['netz'], 0) + u['flaeche_mm2']
    nets = sorted(scores, key=lambda n: (-scores[n], n))[:versuche]
    accepted = baseline
    m0 = regelmessung(B, accepted, K, gl)['regeln'] if nets else None
    ir0 = ir_werte(B, accepted, K) if nets else None
    for n in nets:
        candidate = vorschlag(B, accepted, K, n, gl)
        before_cu, after_cu = kupfer(B, accepted), kupfer(B, candidate)

        additive = all(g.difference(after_cu.get(key, Polygon())).area <= 1e-8
                       for key, g in before_cu.items() if key[0] not in (K.get('gnd') or ['GND']))
        e1 = engstellen(B, candidate, K, gl)
        net_area = lambda e: sum(u['flaeche_mm2'] for u in e['unnoetig'] if u['netz'] == n)
        early = dict(unbekannt=unbekannttor(B, accepted, candidate, E), additiv=additive, r39=pfadtor(E, e1), zielnetz=net_area(e1)<net_area(E)-.001,
                     abstand=not pruefen(B,candidate,K,gl))
        if not all(early.values()):
            info['versuche'].append(dict(netz=n,tore=dict(early,gueltig=None,regeln=None,physik=None),
                angenommen=False,vorher_mm2=E['unnoetig_mm2'],nachher_mm2=e1['unnoetig_mm2'],
                vorher_note=m0.get('note_v3'),nachher_note=None,grund='Frühes Geometrie-/R39-Tor'))
            continue
        valid1, _ = gueltig(B, K, None, gl, routes=candidate)
        m1 = regelmessung(B, candidate, K, gl)['regeln']
        ir1 = ir_werte(B, candidate, K)
        gates = dict(unbekannt=early['unbekannt'], gueltig=valid1, additiv=additive, abstand=not pruefen(B, candidate, K, gl),
                     regeln=regeltor(m0, m1), r39=pfadtor(E, e1),
                     zielnetz=net_area(e1) < net_area(E)-.001,
                     physik=physiktor(ir0, ir1)['bestanden'])
        take = all(gates.values())
        info['versuche'].append(dict(netz=n, tore=gates, angenommen=take,
            vorher_mm2=E['unnoetig_mm2'], nachher_mm2=e1['unnoetig_mm2'],
            vorher_note=m0.get('note_v3'), nachher_note=m1.get('note_v3')))
        if take:
            accepted, E, m0, ir0 = candidate, e1, m1, ir1
            info['angewendet'] = True
    if info['angewendet']:
        pre_export = accepted
        accepted = export_zonen(pre_export)
        info['export_kupfergleich'] = exportgleich(B, pre_export, accepted)
        final_e = engstellen(B, accepted, K, gl)
        if not (info['export_kupfergleich'] and pfadtor(info['vorher'], final_e)
                and final_e['unnoetig_mm2'] <= E['unnoetig_mm2']+.001):
            info.update(angewendet=False, export_tor=False)
            accepted = baseline; E = info['vorher']
        else:
            E = final_e; info['export_tor'] = True
            accepted.pop('stromzonen_abnahme', None)
            accepted.pop('native_abnahme', None)
    info.update(nachher=E, grund='Gesicherter geometrischer Nachlauf' if info['angewendet'] else 'Kein Kandidat besteht alle Tore')
    return accepted, info


def anwenden(bj, kj, rj, gndlagen=None, cpu_s=240):
    from gemeinsam import plaetze as PL
    from lernen.r11_nachlauf import dateisperre
    rj = str(Path(rj).resolve())
    with dateisperre(rj):
        original = Path(rj).read_bytes()
    parent = Path(str(Path(rj).with_suffix(''))+'_r39')
    parent.mkdir(parents=True, exist_ok=True)
    artifact = Path(tempfile.mkdtemp(prefix='lauf_', dir=parent))
    source = artifact/'eingabe.json'; source.write_bytes(original)
    result, route = artifact/'abnahme.json', artifact/'routes.json'
    result.unlink(missing_ok=True); route.unlink(missing_ok=True)
    run = PL.lauf(['caffeinate', '-i', '-s', ROOT/'.venv/bin/python', __file__, bj, kj, source,
                   result, route, json.dumps(gndlagen)], cpu_s=cpu_s, log=str(artifact/'worker.log'), cwd=str(ROOT))
    info = json.loads(result.read_text()) if run['zustand'] == 'ok' and result.exists() else dict(angewendet=False, grund=run['zustand'])
    info['rechenplatz'] = run
    info['arbeitsordner'] = str(artifact)


    with dateisperre(rj):
        if Path(rj).read_bytes() != original:
            info.update(angewendet=False, grund='Route während R39-Prüfung geändert')
        elif route.exists() and run['zustand'] == 'ok':
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                tmp = f.name; f.write(route.read_bytes())
            try: os.replace(tmp, rj)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
    result.write_text(json.dumps(info, indent=1))
    return info


if __name__ == '__main__':
    bj, kj, rj, result, route, gl = sys.argv[1:]
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj, kj, rj)]
    try:
        output, info = reparieren(B, R, K, json.loads(gl))
        Path(route).write_text(json.dumps(output))
    except Exception as e:
        info = dict(angewendet=False, grund=repr(e))
    Path(result).write_text(json.dumps(info, indent=1))
