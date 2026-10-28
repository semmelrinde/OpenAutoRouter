""


def annahme(B, old, new, K):
    from analyse.bewertung import bewerte
    from analyse.regelkennzahlen import annahme as regeltor
    from analyse.k20.kennzahl import messen
    before, after = bewerte(B, old, K, v3=True), bewerte(B, new, K, v3=True)
    gate = regeltor(before['regeln'], after['regeln'])
    a, b = messen(B, old, K), messen(B, new, K)
    fields = ('ohne_zone_mm', 'padgruppen_ohne_gemeinsame_zone')
    worse = [k for k in fields if b[k] > a[k] + 1e-4]
    gain = [k for k in fields if b[k] < a[k] - 1e-4]
    for name in ('gnd_zonenlagen_fehlend', 'prioritaet_fehler'):
        def identities(values):
            return {tuple(sorted(v.items())) if isinstance(v,dict) else v for v in values}
        old_ids,new_ids=identities(a[name]),identities(b[name])
        if new_ids-old_ids: worse.append(name)
        if old_ids-new_ids: gain.append(name)



    aa,bb=a['versorgung_aussen_anteil'],b['versorgung_aussen_anteil']
    if aa is None or bb is None: worse.append('Aussenflaeche unbekannt')
    elif bb > max(.1, aa) + 1e-6: worse.append('versorgung_aussen_anteil')
    elif aa > .1 and bb < aa - 1e-6: gain.append('versorgung_aussen_anteil')
    valid = bool(after['gueltig'])
    return dict(angenommen=valid and gate['nicht_schlechter'] and not worse and bool(gain),
                gueltig_vorher=before['gueltig'], gueltig_nachher=valid,
                regeltor=gate, schlechter=worse, besser=gain, vorher=a, nachher=b)


def anwenden(bj, kj, rj, regeln=None, gndlagen=None, cpu_s=720, zonenplan=None):
    from lernen.stromzonen import anwenden as zonen
    if cpu_s < 1:
        return dict(angewendet=False, grund='zeitlimit',
                    rechenplatz=dict(cpu_s=0.0, zustand='nicht_gestartet'))
    if zonenplan == 'lokal':
        return dict(angewendet=False, grund='K20 mit lokalem Zonenplan gesperrt',
                    rechenplatz=dict(cpu_s=0.0, zustand='nicht_gestartet'))
    try:
        return zonen(bj,kj,rj,gndlagen,regeln=regeln,cpu_s=cpu_s,k20=True)
    except Exception as exc:
        return dict(angewendet=False, grund='K20-Pruefung fehlgeschlagen', fehler=repr(exc))


def abnahme_info(standard, option):
    ""
    if (option or {}).get('angewendet'):
        return dict(option, native_abnahme=True, abschluss='k20')
    return standard
