""
import math


def _zahl(x, positiv=False):
    return (isinstance(x, (int, float)) and not isinstance(x, bool)
            and math.isfinite(x) and (x > 0 if positiv else x >= 0))


def messen(K, M, B=None, R=None, *, kette_messen=None):
    ""
    from analyse.minimax import lastfehler
    pads = {f"{p['ref']}:{p.get('num')}": p['net'] for p in (B or {}).get('pads', [])}
    rows = {}
    missing = []
    reasons = {}
    idle = []
    for n, k in sorted(K.get('netze', {}).items()):
        if k.get('art') not in ('strom', 'versorgung', 'sw'): continue
        m = M.get('netze', {}).get(n, {}).get('m', {})
        flow = k.get('fluss') or {}
        if not isinstance(flow, dict):


            reasons[n] = 'Flusstyp ungueltig'
            missing.append(n)
            continue
        current = m.get('I_A')
        sources, loads = flow.get('quelle'), flow.get('last')
        reason = lastfehler(k)
        if flow.get('einheit') != 'A' or flow.get('einheit_quelle') not in ('vorgabe', 'schaetzung'):
            reason = 'Ampere-Last mit Herkunft fehlt'
        elif not isinstance(sources, list) or not sources or not isinstance(loads, dict) or not loads:
            reason = 'Quelle/Lastanschluesse fehlen'
        elif B is None or any(not isinstance(p, str) or pads.get(p) != n for p in sources + list(loads)):
            reason = 'Quelle/Last nicht auf diesem Netz belegt'
        elif set(sources) & set(loads):


            reason = 'Pad zugleich Quelle und Last'
        if reason:
            reasons[n] = reason
            missing.append(n)
            continue

        tolerance = 5e-5 + (len(loads)+1)*5e-8+1e-12

        origin = flow['einheit_quelle']
        if flow.get('lernbegrenzung'):
            origin += ' (lernbegrenzt)'
        elif origin == 'schaetzung':
            origin = k.get('strom_A_quelle')
        if flow['gesamt'] == 0:

            drop, j = m.get('dV_mV'), m.get('J_x')
            if ((current is not None and (not _zahl(current) or current > tolerance))
                    or (m.get('I_quelle') is not None and m.get('I_quelle') != origin)
                    or (drop is not None and (not _zahl(drop) or drop > 1e-6))
                    or (j is not None and (not _zahl(j) or j > 1e-6))):
                reasons[n] = 'Messwerte widersprechen der Nulllast'
                missing.append(n)
                continue
            idle.append(n)
            continue
        if (not _zahl(current) or abs(current-flow['gesamt']) > tolerance
                or not isinstance(origin, str) or not origin or origin == '1A'
                or m.get('I_quelle') != origin):
            reasons[n] = 'Messstrom/Herkunft passt nicht zum Lastvertrag'
            missing.append(n)
            continue
        budget, drop = m.get('s_dc_mV'), m.get('dV_mV')
        j = m.get('J_x')
        if not (_zahl(budget, positiv=True) and _zahl(drop) and _zahl(current)
                and _zahl(j) and _zahl(drop / budget)):
            missing.append(n)
            continue
        rows[n] = dict(dc_x=drop / budget, abfall_mV=drop, budget_mV=budget,
                       gruppe=m.get('gruppe'), ziel_x=drop/budget,
                       J_x=j, strom_A=current, modell=m.get('dV_quelle'),
                       lastquelle=flow.get('einheit_quelle') or m.get('I_quelle'))
    groups = {}
    for name, q in sorted(M.get('gruppen', {}).items()):
        members = q.get('netze') or []
        budget = q.get('mV')
        if (not _zahl(budget, positiv=True) or not isinstance(members, list) or not members
                or any(not isinstance(n, str) for n in members) or len(set(members)) != len(members)
                or any(n not in rows and n not in idle for n in members)):
            missing.append('gruppe:'+name)
            continue


        value = sum(rows[n]['abfall_mV'] for n in members if n in rows) / budget
        if not _zahl(value):
            missing.append('gruppe:'+name)
            continue
        groups[name] = dict(dc_x=value, netze=members, budget_mV=budget,
                            lastlose_netze=[n for n in members if n in idle])
        for n in members:
            if n in rows: rows[n]['ziel_x'] = value
    for n, row in rows.items():
        name = row['gruppe']
        if name and (not isinstance(name, str) or name not in groups or n not in groups[name]['netze']):
            missing.append(n+':budgetgruppe:'+str(name))
    criteria = [q['dc_x'] for q in rows.values() if not q['gruppe']] + [q['dc_x'] for q in groups.values()]
    maximum = max(criteria, default=None)
    chain = dict(status='nicht_gemessen', wirkungsgrad=None, glied_max=None)
    if kette_messen is None:
        option = ((R or {}).get('optionen') or {}).get('r11_kette',
                    (K.get('vorgaben') or {}).get('r11_kette', False))
        kette_messen = option is True or option == 1 or option == '1'
    if (kette_messen and rows and B is not None and R is not None
            and 'J_max_A_mm2' in (M.get('kupfer_ir') or {})):



        try:
            from analyse.strom_kette import kette
            ck = dict(K, netze={n: K['netze'][n] for n in rows})
            result = kette(B, R, ck, ir=M['kupfer_ir'])
            measured = set(result.get('netze') or {})
            absent = sorted(set(rows) - measured)
            status = 'unvollstaendig' if absent and measured else 'nicht_gemessen' if not measured else 'gemessen'
            chain = dict(result, status=status, fehlende_netze=absent)
        except Exception as exc:
            chain = dict(chain, status='messfehler', fehler=repr(exc)[:200])
    return dict(version='r11-v2', status='unvollstaendig' if missing else 'gemessen' if rows else 'lastlos' if idle else 'kein_strommodell',
                netze_gemessen=len(rows), fehlende_modelle=len(missing),
                modell_vollstaendig=bool(rows) and not missing,
                netze=rows, gruppen=groups, fehlende_netze=missing, modell_fehler=reasons,
                lastlose_netze=idle, lastlos_n=len(idle), kette=chain, dc_x_max=maximum,
                netze_ueber_budget=sum(x > 1 for x in criteria) if criteria else None,
                J_x_max=max((q['J_x'] for q in rows.values() if q['J_x'] is not None), default=None))
