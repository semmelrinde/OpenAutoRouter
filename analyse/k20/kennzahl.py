""
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

VERSION = 'k20-kupfer-v4'


def _padgruppen(pads, abstand=1.5):
    ""
    parents = list(range(len(pads)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for i, (_, a) in enumerate(pads):
        for j in range(i):
            if a.distance(pads[j][1]) <= abstand:
                parents[root(i)] = root(j)
    groups = defaultdict(list)
    for i, (ident, _) in enumerate(pads):
        groups[root(i)].append(i)
    return [g for g in groups.values() if len(g) >= 2]


def messen(B, R, K):
    from gemeinsam.zonen import boardgeometrie
    B = boardgeometrie(B, R)
    native, model = defaultdict(list), defaultdict(list)
    np, mp = defaultdict(list), defaultdict(list)
    for z in R.get('zones', []):
        if not z.get('filled'): continue
        key = z['net'], z['layer']


        is_native = z.get('native_fuellung') or (z.get('modell') is not True and
            (R.get('kupferbalken') or {}).get('modell') == 'native Neufuellung')
        modeled = not is_native and (z.get('modell') or z.get('quelle') == 'R36-Vollkupfermodell')
        target, priorities = (model, mp) if modeled else (native, np)
        target[key].append(Polygon(z['pts'], z.get('holes') or []).buffer(0))
        priorities[key].append(z.get('prio'))
    for z in R.get('zonen', []):
        key = z['net'], z['layer']
        model[key].extend(Polygon(p['aussen'], p.get('innen') or []).buffer(0) for p in z.get('polys', []))
        mp[key].append(z.get('prio', -1 if z.get('quelle') == 'gnd_fuellung' else 1))
    native_ok = bool(native and (R.get('stromzonen_abnahme') or {}).get('angewendet'))
    n = _messen(B, R, K, native, np, 'native_abgenommen' if native_ok else
                'filled_export' if native else 'keine_zonen', native_ok)
    m = _messen(B, R, K, model, mp, 'modell' if model else 'keine_zonen', False)


    out = dict(n if native else m)
    out.update(nativ=n, modell=m)
    return out
def _messen(B, R, K, fills, priorities, source, native_ok):
    from lernen.stromzonen import _lagen, STROMARTEN
    power = {n for n, v in K.get('netze', {}).items()
             if v.get('art') in STROMARTEN and not v.get('kelvin') and n not in K.get('gnd', [])}
    from lernen.stromzonen_topo import laststatus
    status = {n: laststatus(K, n, B) for n in power}
    ohne_vertrag = sorted(n for n in power if status[n] == 'unbekannt')
    lastlos = sorted(n for n in power if status[n] == 'lastlos')
    gnd = set(K.get('gnd') or ['GND'])
    fills = {key: unary_union(parts) for key, parts in fills.items()}
    rows = []
    for L in _lagen(B):
        for n in sorted(power):
            pads = [(dict(instanz=i, ref=p['ref'], num=str(p['num']), x=p['x'], y=p['y']),
                     unary_union([Polygon(q).buffer(0) for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]))
                    for i, p in enumerate(B['pads']) if p['net'] == n and (p.get('polys') or {}).get(L)]
            pad = [g for _, g in pads]
            paths = [LineString(t['pts']) for t in R.get('tracks', [])
                     if t['net'] == n and t['layer'] == L and len(t['pts']) >= 2]
            if not pad and not paths: continue
            zone = fills.get((n, L), Polygon())
            pieces = [p for p in getattr(zone, 'geoms', [zone]) if p.geom_type == 'Polygon' and p.area >= .01]


            path = unary_union(paths).difference(unary_union(pad).buffer(.02))
            missing = path.difference(zone.buffer(.008)).length
            groups = _padgruppen(pads)
            bad_groups = 0
            for group in groups:
                members = []
                for i in group:
                    pd = B['pads'][pads[i][0]['instanz']]
                    drill = float(pd.get('drill') or 0.0)



                    members.append(pad[i].difference(Point(pd['x'],pd['y']).buffer(drill/2,64)) if drill else pad[i])
                if not any(all(part.buffer(.008).covers(p) for p in members) for part in pieces):
                    bad_groups += 1
            rows.append(dict(netz=n, lage=L, korridor_mm=round(path.length, 4),
                             ohne_zone_mm=round(missing, 4), padgruppen=len(groups),
                             padgruppen_mitglieder=[[pads[i][0] for i in group] for group in groups],
                             padgruppen_ohne_gemeinsame_zone=bad_groups, stuecke=len(pieces),
                             zonen_mm2=round(zone.area, 4)))
    ground_layers = sorted({L for (n, L), f in fills.items() if n in gnd and not f.is_empty})
    bad_priority = []
    for (n, L), vals in sorted(priorities.items()):
        if n not in power: continue
        ground = [v for (gn, layer), vs in priorities.items() if gn in gnd and layer == L for v in vs]
        if ground and (any(v is None for v in vals + ground) or min(vals) <= max(ground)):
            bad_priority.append(dict(netz=n, lage=L))
    length = sum(r['korridor_mm'] for r in rows)
    absent = sum(r['ohne_zone_mm'] for r in rows)
    from analyse.kupfermodell import umriss
    board = umriss(B)
    outer_power = sum(unary_union([f for (n, layer), f in fills.items()
                                  if n in power and layer == L]).intersection(board).area for L in ('F', 'B'))
    return dict(version=VERSION, quelle=source, nativ_abgenommen=native_ok, je=rows,
                netze_ohne_lastvertrag=ohne_vertrag, netze_ohne_lastvertrag_n=len(ohne_vertrag),
                netze_dc_lastlos=lastlos, netze_dc_lastlos_n=len(lastlos),
                korridor_mm=round(length, 4), ohne_zone_mm=round(absent, 4),
                ohne_zone_anteil=round(absent/length, 6) if length > 1e-9 else None,
                padgruppen=sum(r['padgruppen'] for r in rows),
                padgruppen_ohne_gemeinsame_zone=sum(r['padgruppen_ohne_gemeinsame_zone'] for r in rows),
                gnd_zonenlagen=ground_layers, gnd_zonenlagen_fehlend=sorted(set(_lagen(B))-set(ground_layers)),
                gnd_zonenlagen_fehlend_n=len(set(_lagen(B))-set(ground_layers)),
                prioritaet_fehler=bad_priority, prioritaet_fehler_n=len(bad_priority),
                platinenflaeche_mm2=round(board.area, 4), versorgung_aussen_mm2=round(outer_power,4),
                versorgung_aussen_anteil=round(outer_power/(2*board.area),6) if board.area else None,
                fuellstuecke=sum(r['stuecke'] for r in rows),
                hinweis='Diagnose; keine Gewichtung, keine erfundene GND-Fuellung, keine DRC-/IR-Abnahme.')


def diagnose(out, B, R, K):
    try:
        out['k20'] = messen(B, R, K)
    except Exception as exc:
        out['k20'] = None
        out.setdefault('diagnose_fehler', {})['k20'] = repr(exc)[:200]
