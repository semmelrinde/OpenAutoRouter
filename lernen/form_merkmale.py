""
import math
import re

VERSION = 'gutachter-form-v3'
ZONEN = ('keile_45', 'keile_80', 'kerben', 'spitze_ecken', 'nebenstreifen',
         'eigennetz_schlitze', 'via_bisse', 'z2_aussen_pct', 'z4_splitter',
         'z2_ohne_pad_pct', 'z4_je_netz_lage', 'stuecke')
FELDER = ZONEN + ('zonen_vorhanden', 'kette_eta', 'kette_eta_median', 'kette_glied_max',
                   'r39_unnoetig_mm2', 'breitenwechsel', 'r18_knick_ueberschuss',
                   'r18_signal_umweg_mm', 'k20_ohne_zone_anteil',
                   'k20_padgruppen_ohne_zone', 'k20_versorgung_aussen_anteil',
                   'r40_spitze_vias_mm2', 'r40_fenster_ueber',
                   'k17_freie_kerben_mm2', 'k17_schmale_zusatzflaeche_mm2')


def zahl(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0


def fehlgeschlagen(data):
    return (not isinstance(data, dict) or 'fehler' in data or
            data.get('vollstaendig') is False or bool(data.get('nicht_messbar')) or
            isinstance(data.get('abdeckung'), dict) and data['abdeckung'].get('vollstaendig') is False)


def gruppenfehler(quellen, *gruppen):
    ""
    for src in quellen:
        if not isinstance(src, dict): continue
        if (src.get('vollstaendig') is False or src.get('nicht_messbar') or
                isinstance(src.get('abdeckung'), dict) and src['abdeckung'].get('vollstaendig') is False):
            return True
        for key in ('fehler', 'diagnose_fehler'):
            if key in src:
                if not isinstance(src[key], dict): return True
                if any(g in src[key] for g in gruppen): return True
        for g in gruppen:
            if g+'_fehler' in src: return True
            obj = src.get(g)
            if isinstance(obj, dict) and ('fehler' in obj or gruppenfehler([obj])): return True
    return False


def kette_ok(data):
    ""
    return (isinstance(data, dict) and 'fehler' not in data and
            not gruppenfehler([data]) and isinstance(data.get('ausgelassen'), dict)
            and not data['ausgelassen'])


def r39_ok(quellen):
    if gruppenfehler(quellen, 'r39', 'strom_engstellen'): return False
    coverage = []
    for src in quellen:
        if not isinstance(src, dict): continue
        if 'r39_abdeckung' in src: coverage.append(src['r39_abdeckung'])
        eng = src.get('strom_engstellen')
        if isinstance(eng, dict) and 'abdeckung' in eng: coverage.append(eng['abdeckung'])
    return bool(coverage) and all(isinstance(c, dict) and 'fehler' not in c and
                                c.get('vollstaendig') is True and zahl(c.get('komponenten'))
                                and c['komponenten'] > 0 for c in coverage)


def werte(z):
    ""
    m = z.get('form_messung')
    if not isinstance(m, dict) or m.get('version') != VERSION or m.get('vollstaendig') is not True or 'fehler' in m:
        return {}
    if any(not isinstance(m.get(k), str) or not re.fullmatch('[0-9a-f]{64}', m[k])
           for k in ('geometrie_sha256', 'auswerter_sha256')):
        return {}
    if z.get('geometrie_sha256') is not None and z['geometrie_sha256'] != m['geometrie_sha256']:
        return {}
    v, coverage = m.get('werte'), m.get('abdeckung')
    if not isinstance(v, dict) or not isinstance(coverage, dict):
        return {}
    chain=m.get('kette_abdeckung') or {}
    chain_ok=chain.get('vollstaendig') is True and kette_ok(chain)
    return {k: float(v[k]) for k in FELDER if coverage.get(k) is True and zahl(v.get(k))
            and (not k.startswith('kette_') or chain_ok)}


def aus_messung(w, r, geometrie_sha256, auswerter_sha256):
    ""
    v = {}
    r_ok = not gruppenfehler([r])
    w_ok = not fehlgeschlagen(w)
    if w_ok and zahl(w.get('stuecke')):
        v.update(stuecke=w['stuecke'], zonen_vorhanden=int(w['stuecke'] > 0))
        if w['stuecke'] > 0:
            for k in ZONEN:
                if k not in ('nebenstreifen', 'eigennetz_schlitze', 'via_bisse') or not fehlgeschlagen(w.get('geometrie') or {}):
                    if zahl(w.get(k)): v[k] = w[k]
    chain_coverage = w.get('kette_abdeckung') or {}
    if w_ok and chain_coverage.get('vollstaendig') is True and kette_ok(chain_coverage):
        for k in ('kette_eta', 'kette_eta_median', 'kette_glied_max'):
            if zahl(w.get(k)): v[k] = w[k]
    eng = r.get('strom_engstellen') or {}
    if r39_ok([r]):
        if zahl(eng.get('unnoetig_mm2')): v['r39_unnoetig_mm2'] = eng['unnoetig_mm2']
    if not gruppenfehler([r], 'kennzahlen', 'breitenwechsel') and zahl(r.get('breitenwechsel')): v['breitenwechsel'] = r['breitenwechsel']
    r18 = (r.get('r18') or {}) if not gruppenfehler([r], 'r18') else {}
    for key, group, field in [('r18_knick_ueberschuss', 'alle', 'knick_ueberschuss'),
                              ('r18_signal_umweg_mm', 'signal', 'umweg_octi_mm')]:
        data = r18.get(group) or {}
        x = data.get(field) if not fehlgeschlagen(data) else None
        if zahl(x): v[key] = x
    k20 = (r.get('k20') or {}) if not gruppenfehler([r], 'k20') else {}
    for key, field in [('k20_ohne_zone_anteil', 'ohne_zone_anteil'),
                       ('k20_padgruppen_ohne_zone', 'padgruppen_ohne_gemeinsame_zone'),
                       ('k20_versorgung_aussen_anteil', 'versorgung_aussen_anteil')]:
        if zahl(k20.get(field)): v[key] = k20[field]
    for key, field in [('r40_spitze_vias_mm2', 'via_dichte_spitze'), ('r40_fenster_ueber', 'via_dichte_fenster_ueber')]:
        if not gruppenfehler([r], 'via_dichte', 'r40') and zahl(r.get(field)): v[key] = r[field]
    k17 = r.get('k17') or {}
    if (not gruppenfehler([r], 'k17') and k17.get('version') == 'k17-freie-kerben-v2'
            and k17.get('status') == 'gemessen' and k17.get('vollstaendig') is True):
        for field in ('freie_kerben_mm2', 'schmale_zusatzflaeche_mm2'):
            if zahl(k17.get(field)): v['k17_'+field] = k17[field]
    return dict(version=VERSION, geometrie_sha256=geometrie_sha256, auswerter_sha256=auswerter_sha256,
                vollstaendig=w_ok and r_ok,
                kette_abdeckung=dict(chain_coverage),
                werte=v, abdeckung={k: k in v for k in FELDER
                                    if not k.startswith('k17_') or k in v})
