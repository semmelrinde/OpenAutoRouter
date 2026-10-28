""
import json, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)


def schluessel_faust(m):
    ""
    return m['mst'] * (1 + 0.02 * m['kreuzungen']) * (1 + max(0.0, m['rudy_max'] - 1))


def rangfolge(B, K, RG, placements, auswahl='faust'):
    from lernen.auswahl import board_aus_placement
    from lernen.merkmale import merkmale
    ks = [dict(pfad=p, merkmale=merkmale(board_aus_placement(B, json.load(open(p))), K, RG)) for p in placements]
    if auswahl == 'modell':
        import joblib, numpy as np
        from lernen import modell as LM
        mod = joblib.load(os.path.join(R, 'lernen/modell/auswahl.joblib'))
        P, C = LM.vorhersage(mod, [dict(k['merkmale'], via_f=1.0) for k in ks]); med = float(np.median(C))
        E = P * C + (1 - P) * 2 * med
        for k, e in zip(ks, E): k['schluessel'] = float(e)
    else:
        for k in ks: k['schluessel'] = schluessel_faust(k['merkmale'])
    return sorted(ks, key=lambda k: (k['schluessel'], k['pfad']))


def zone_opt():
    ""
    return {k: os.environ[k] for k in ('ZONE_EDGE', 'ZONE_GAP', 'ZONE_FAIR', 'ZONE_VORRANG', 'ZONE_GND_SIGNAL', 'GND_ALLE',
                                       'FILL_NETZ', 'ZUNGE', 'ZCLR') if k in os.environ}


def drc_zahlen(pfad):
    ""
    try:
        d = json.load(open(pfad))
        v = d['violations']; offen = d['unconnected_items']
        if not isinstance(v, list) or not isinstance(offen, list) or any(not isinstance(x, dict) for x in offen):
            raise ValueError('ungueltiges DRC-Format')
        if any(not isinstance(x, dict) or x.get('severity') not in ('error', 'warning', 'info') or not isinstance(x.get('type'), str) for x in v):
            raise ValueError('ungueltiger DRC-Eintrag')
    except (OSError, ValueError, KeyError, TypeError):
        return 99999, 99999, 0
    return (sum(1 for x in v if x.get('severity') == 'error'), len(offen),
            sum(1 for x in v if x.get('type') in ('track_dangling', 'via_dangling')))


def lade(ordner, routes='routes_form.json', drc='drc_fertig.json', ir=True):
    from analyse.bewertung import bewerte
    rp = os.path.join(ordner, routes)
    if not os.path.exists(rp): rp = os.path.join(ordner, 'routes.json')
    B = json.load(open(os.path.join(ordner, 'board.json'))); K = json.load(open(os.path.join(ordner, 'klassen.json')))
    Rt = json.load(open(rp))


    Rt = dict(Rt, tracks=[{k: x for k, x in t.items() if k != 'items'} for t in Rt['tracks']])
    E = bewerte(B, Rt, K, ir=ir, zonen=zone_opt() if ir else None, v3=True)
    from lernen.richter_paar import messsignatur, komponenten, komponentenbeleg
    from lernen.paar_vertrag import boardkennung
    E.update(board_kennung=boardkennung(B), paar_messversion=messsignatur(), v3_komponenten=komponenten(E.get('metriken') or {}), v3_komponenten_beleg=komponentenbeleg(E.get('metriken') or {}))
    fe, of, lo = drc_zahlen(os.path.join(ordner, drc))
    return dict(ordner=ordner, routes=rp, B=B, K=K, R=Rt, E=E, drc_fehler=fe, drc_offen=of, drc_lose=lo,
                gueltig_voll=bool(E['gueltig']) and fe == 0 and of == 0)


def tor_schluessel(v):
    ""
    return (not v['gueltig_voll'], v['drc_fehler'] + v['drc_offen'], tuple(v['E']['rang'][:3]))
def waehle(vs, note='v3', paar=None):
    ""
    from analyse.bewertung import vergleich
    vs = sorted(vs, key=lambda v: (tor_schluessel(v), v['ordner']))
    oben = [v for v in vs if tor_schluessel(v) == tor_schluessel(vs[0])]
    best = oben[0]; prot = []
    for v in oben[1:]:
        vg = vergleich(best['E'], v['E'], best['B'], best['R'], v['B'], v['R'], best['K'], note=note, K1=v['K'])
        prot.append(dict(gegen=best['ordner'], kandidat=v['ordner'], besser=vg['besser'], grund=vg['grund'], delta=vg.get('delta')))
        if vg['besser']: best = v

    if paar is None:
        from lernen.richter_paar import Dienst
        paar = Dienst()
    def row(v):
        return dict(v['E'], gueltig=v['gueltig_voll'], regel=v['E'].get('regeln') or {})
    for v in oben:
        if v is best: continue
        decision = paar.vergleich(row(best), row(v))
        if decision['paar']:
            prot.append(dict(gegen=best['ordner'], kandidat=v['ordner'], **decision))
            if decision['besser']: best = v
    return best, prot


def zeile(v):
    E = v['E']; sp = E.get('strompfade') or {}; ir = E.get('ir') or {}
    mx = max((ir.get('max_abfall_mV') or {}).values() or [0.0])
    return '%-28s %s DRC %d/%d (lose %d) | Note v3 %8.1f | v2 %s | Vias %4d | Laenge %7.1f mm | Verlust %s mW | groesster Abfall %.1f mV' % (
        os.path.basename(v['ordner'].rstrip('/')), 'GUELTIG  ' if v['gueltig_voll'] else 'ungueltig', v['drc_fehler'], v['drc_offen'],
        v['drc_lose'], E.get('note_v3') or 0.0, '%.1f' % E['note_v2'] if E.get('note_v2') is not None else '-', E['vias'],
        E['laenge_gesamt'], '%.1f' % sp['verlust_mW'] if sp.get('verlust_mW') is not None else '-', mx)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    if not a: sys.exit(__doc__)
    if a[0] == 'rangfolge':
        B, K, RG = (json.load(open(p)) for p in a[1:4])
        for i, k in enumerate(rangfolge(B, K, RG, a[4:], kw.get('auswahl', 'faust')), 1):
            m = k['merkmale']
            print('RANG %d %s  Schluessel %.1f  Luftlinie %.0f  Kreuzungen %d' % (i, k['pfad'], k['schluessel'], m['mst'], m['kreuzungen']))
    elif a[0] == 'waehle':
        vs = [lade(o, kw.get('routes', 'routes_form.json'), kw.get('drc', 'drc_fertig.json'), kw.get('ir', '1') == '1') for o in a[2:]]
        for v in vs: print(zeile(v))
        from lernen.richter_paar import Dienst
        pair = Dienst(modell=kw['paar_modell']) if kw.get('paar_modell') else None
        best, prot = waehle(vs, kw.get('note', 'v3'), pair)
        for p in prot: print('  Vergleich %s gegen %s: %s (%s)' % (os.path.basename(p['kandidat']), os.path.basename(p['gegen']),
                                                                   'besser' if p['besser'] else 'nicht besser', p['grund']))
        json.dump(dict(gewaehlt=best['ordner'], tor='gueltig (Bewertung + KiCad-DRC), dann Note v3 (bewertung.vergleich)',
                       varianten=[dict(ordner=v['ordner'], routes=v['routes'], gueltig=v['gueltig_voll'], drc=[v['drc_fehler'], v['drc_offen'], v['drc_lose']],
                                       rang=v['E']['rang'], note_v3=v['E'].get('note_v3'), note_v2=v['E'].get('note_v2'), note=v['E']['note'],
                                       vias=v['E']['vias'], laenge=v['E']['laenge_gesamt'], strompfade=v['E'].get('strompfade'))
                                  for v in vs], vergleiche=prot), open(a[1], 'w'), indent=1)
        print('GEWAEHLT', best['ordner'])
    else:
        sys.exit(__doc__)
