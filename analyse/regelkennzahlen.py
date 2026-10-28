""
import json, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VERSION = 'regeln-v1'
SI_VERSIONS = ('v3.2', 'v3.2+r35-si2-option')
TEILE_V3 = dict(verlust=('dc', 'j', 'leistung'), uebersprechen=('xt_opfer', 'xt_stoer', 'masse'), rueckweg=('rueck',),
                schaltknoten=('sw_flaeche', 'l_last', 'hot_L', 'hot_A'), signal=('laenge', 'c_last', 'vias', 'umweg'))
TEILE_V3_SI = dict(TEILE_V3, uebersprechen=TEILE_V3['uebersprechen'] + ('stoer',), auge=('auge_v', 'auge_h', 'auge_skew'))
TOR = ('note_v3', 'verlust', 'uebersprechen', 'rueckweg', 'topologie', 'kopplung', 'regelpruefer')
TOR_SI = TOR + ('auge', 'auge_vertikal_verlust', 'auge_horizontal_verlust', 'stoerbudget', 'stoerbudget_max')
NEBEN = ('schaltknoten', 'signal', 'knicke', 'mini', 'vias_unnoetig', 'parallel_mm', 'via_dichte_spitze', 'via_dichte_fenster_ueber', 'strom_engstellen_mm2')

TOL_ABS = dict(auge=0.01, auge_vertikal_verlust=0.01, auge_horizontal_verlust=0.01, stoerbudget=0.01, stoerbudget_max=0.01,
               note_v3=0.5, verlust=0.5, uebersprechen=0.5, rueckweg=0.5, schaltknoten=0.5, signal=0.5, topologie=1.0,
               regelpruefer=1.0, knicke=2.0, mini=2.0, kopplung=0.5, vias_unnoetig=1.0, parallel_mm=1.0)


def _gnd_lagen(gl):
    ""
    if gl is None: return None
    if isinstance(gl, str): gl = [x for x in gl.split(',') if x]
    return [str(x).split('=')[0] for x in gl]


def v3_teile(M):
    ""
    teile = TEILE_V3_SI if M.get('version') in SI_VERSIONS else TEILE_V3
    out = {k: 0.0 for k in teile}; rest = 0.0
    wo = {g: k for k, gs in teile.items() for g in gs}
    for v in M['netze'].values():
        W = v.get('gewicht') or 1.0
        for g, t in (v.get('teile') or {}).items():
            if g in wo: out[wo[g]] += W * t
            else: rest += W * t
    out['verlust'] += sum(v for k, v in (M.get('board_teile') or {}).items() if k != 'lagenaufbau')
    rest += (M.get('board_teile') or {}).get('lagenaufbau', 0)
    if M.get('si_note_aktiv'): out['signal'] += M['signalintegritaet']['note_zusatz']
    if abs(rest) > 1e-9: out['sonst'] = rest
    return {k: round(v, 3) for k, v in out.items()}


def _regelpruefer(E, mit_flaeche):
    ""
    ent = E['entkopplung']['gruppen']
    z = dict(entkopplung_abstand=sum(g['ueber_soll'] for n, g in ent.items() if n != 'ausgang'),
             entkopplung_lagenwechsel=(ent.get('klein') or {}).get('mit_lagenwechsel', 0),
             kette=sum(v for u, v in E['kette']['urteile'].items() if u in ('via_vor_c', 'abzweig')),
             quarz=E['quarz']['ueber_soll'], mindestbreite=E['mindestbreite']['netze'],
             vias_unnoetig=E['vias_unnoetig']['vias'], ketten=E['kette']['anzahl'],
             entkopplung_teile=sum(g['anzahl'] for n, g in ent.items() if n != 'ausgang'))
    mv = E['masse_vias']['gruppen']
    z['masse_stich'] = sum(g['stich_ueber_soll'] for n, g in mv.items() if n in ('entkopplung', 'chip')) if mit_flaeche else 0
    z['masse_ohne_via'] = sum(g['ohne_via'] for n, g in mv.items() if n in ('entkopplung', 'chip')) if mit_flaeche else 0
    z['massepad_fehlen'] = sum(p['fehlen'] for p in E['massepad']['pads']) if mit_flaeche else 0
    z['summe'] = sum(z[k] for k in ('entkopplung_abstand', 'entkopplung_lagenwechsel', 'kette', 'quarz', 'mindestbreite',
                                    'masse_stich', 'masse_ohne_via', 'massepad_fehlen'))
    return z


def _r36_diagnose(out, B, R, K):
    try:
        from analyse.r36 import messen
        out['r36'] = messen(B, R, K)
    except Exception as e:
        out['r36'] = dict(version='r36-form-v3', status='messfehler')
        out['diagnose_fehler'] = dict(out.get('diagnose_fehler') or {}, r36=repr(e)[:200])


def _r18_diagnose(out, B, R, K):
    ""
    try:
        from analyse.r18 import messen
        out['r18'] = messen(B, R, K)
    except Exception as e:
        out.setdefault('diagnose_fehler', {})['r18'] = repr(e)[:200]


def _r41_diagnose(out, B, R, K):
    ""
    flag = (R.get('optionen') or {}).get('r41_messen', (K.get('vorgaben') or {}).get('r41_messen', False))
    if not (flag is True or flag == '1'): return
    try:
        from analyse.r41 import messen as r41_messen
        out['r41'] = r41_messen(B, R, K)
        out['r41_fehler_mm'] = out['r41']['fehler_mm']
    except Exception as e:
        out['r41_fehler_mm'] = None
        out.setdefault('diagnose_fehler', {})['r41'] = repr(e)[:200]


def r19_messen(B, R, K):
    ""
    from analyse.wackel import messen as wackel, ketten
    from analyse.r19 import ausreisser, knickexzess45
    signale = {n for n, k in K['netze'].items() if k.get('art') in ('signal', 'analog', 'schnell')}
    rs = dict(R, tracks=[t for t in R.get('tracks', []) if t['net'] in signale],
              vias=[v for v in R.get('vias', []) if v['net'] in signale and not v.get('strom_via')
                    and not v.get('rueck_via') and not v.get('rv')])
    chains = ketten(rs)
    return dict(wackel(B, rs, gnd=K.get('gnd') or [], chains=chains), version='r19-signal-v2', ausreisser=ausreisser(B, rs, chains=chains),
                **knickexzess45(rs, chains=chains),
                hinweis='Umweg gegen Pad-MST ist Diagnose, kein Beweis einer freien Alternative; Zonen nicht gezaehlt.')


def messen(B, R, K, gndlagen=None, flaechennetze=None, metriken_result=None, minimax=None, si_note=False, si_messen=False, topologie_version=None, r27_diagnose=True, r44_result=None):
    ""


    from analyse.r44 import messen as r44_messen
    r44 = r44_result if r44_result is not None else r44_messen(B, R)
    if (K.get('vorgaben') or {}).get('physik_version') == 'v3.3-vertrag-option':
        from analyse.regelkennzahlen_physik import messen as physik
        out = physik(B, R, K, gndlagen=gndlagen, flaechennetze=flaechennetze,
                      metriken_result=metriken_result, minimax=minimax, si_note=si_note,
                      si_messen=si_messen, topologie_version=topologie_version)
        out['r44'] = r44
        _r41_diagnose(out, B, R, K)
        _r40_diagnose(out, R, K)
        _via_diagnose(out, B, R)
        _r36_diagnose(out, B, R, K)
        _r24_diagnose(out, B, R, K)
        _r18_diagnose(out, B, R, K)


        try:
            out['r19'] = r19_messen(B, R, K)
        except Exception as e:
            out['diagnose_fehler'] = dict(out.get('diagnose_fehler') or {}, r19=repr(e)[:200])
        _p26(B, K, out)
        _k20_diagnose(out, B, R, K)
        _k17_diagnose(out, B, R, K)
        return out
    t0 = time.time(); c0 = time.process_time()
    R = dict(R)
    if gndlagen is not None:
        R['gndlagen'] = list(gndlagen) if not isinstance(gndlagen, str) else [x for x in gndlagen.split(',') if x]
        from analyse.flaechen import aktiv
        if aktiv(R, K): R['optionen'] = dict(R.get('optionen') or {}, gndlagen=','.join(R['gndlagen']))
    if flaechennetze is not None: R['flaechennetze'] = sorted(flaechennetze)
    R.setdefault('conflicts', {}); R.setdefault('failed', [])
    from analyse.flaechen import aktiv, kennzahl
    la_aktiv = aktiv(R, K)
    la = kennzahl(B, R, K) if la_aktiv else None
    lagen = la['gnd_kupferlagen'] if la_aktiv else (_gnd_lagen(R.get('gndlagen')) or [])
    out = dict(version=VERSION, gndlagen=lagen, flaechennetze=sorted(R.get('flaechennetze') or []), r44=r44)
    _r18_diagnose(out, B, R, K)
    fehler = {}
    _p26(B, K, out)
    try:
        from analyse.metriken import metriken
        M = metriken_result if metriken_result is not None else metriken(B, R, K, gndlagen=gndlagen if la_aktiv else R.get('gndlagen', lagen), minimax=minimax, si_note=si_note, si_messen=si_messen)
        if 'minimax_version' in M:
            out.update({k: M[k] for k in ('minimax', 'minimax_version', 'minimax_profil', 'minimax_rang', 'minimax_summe',
                                          'minimax_vollstaendig', 'minimax_quellen', 'engpass', 'sicherheit')})
            out['minimax_fehlend'] = M.get('minimax_fehlend')
        if 'signalintegritaet' in M and bool(M.get('si_note_aktiv')) != bool(si_note):
            raise ValueError('metriken_result: abweichende si_note-Option')
        T = v3_teile(M)
        from analyse.r11 import messen as r11_messen
        out['r11'] = r11_messen(K, M, B, R)
        out.update(note_v3_version=M.get('version'), note_v3=round(M['note_v3'], 3), verlust_mW=M.get('verlust_mW'), hot_loops=len(M.get('hot_loop') or []),
                   masse_max_uV=(M.get('masse') or {}).get('max_uV'), **T)
        if 'signalintegritaet' in M:
            out.update(signalintegritaet=M['signalintegritaet'], si_note_aktiv=M['si_note_aktiv'], note_v3_basis=M['note_v3_basis'])
        if M.get('version') in SI_VERSIONS:
            from analyse.signalintegritaet import kurz
            out.update(version='regeln-v2-si', note_v3_version=M['version'], **kurz(M))
            out['stoerbudget'] = round(sum(v['x'] for v in (M.get('analog_stoer') or {}).values()), 6)
            out['stoerbudget_max'] = max((v['x'] for v in (M.get('analog_stoer') or {}).values()), default=0.0)
            out['auge_vertikal_verlust'] = 100 - out['augen_vertikal_pct'] if out['augen_vertikal_pct'] is not None else 0.0
            out['auge_horizontal_verlust'] = 100 - out['augen_horizontal_pct'] if out['augen_horizontal_pct'] is not None else 0.0

        out['v3_netze'] = [[n, round(v['beitrag'], 3), v['klasse']] for n, v in sorted(M['netze'].items(), key=lambda kv: (-kv[1]['beitrag'], kv[0]))[:5]]
    except Exception as e:
        fehler['note_v3'] = repr(e)[:200]
    try:
        from analyse.kennzahlen import kennzahlen
        k = kennzahlen(R)
        out.update(knicke=k['knicke'], mini=k['mini'], anteil_45=k['anteil_45'], segmente=k['segmente'], fast_stoesse=k['fast_stoesse'],
                   breitenwechsel=k['breitenwechsel'], vias=k['vias'], signal_vias=k['signal_vias'], laenge=k['laenge'])
    except Exception as e:
        fehler['kennzahlen'] = repr(e)[:200]
    try:
        out['r19'] = r19_messen(B, R, K)
    except Exception as e:

        out['diagnose_fehler'] = dict(out.get('diagnose_fehler') or {}, r19=repr(e)[:200])
    try:
        from analyse.topologie import topologie
        tp = topologie(R, B, **({'version': topologie_version} if topologie_version is not None else {}))
        if 'version' in tp: out['topologie_version'] = tp['version']
        top = dict(stern_frei=tp['stern'] - tp['stern_pad'], spitz_frei=tp['spitz'] - tp['spitz_pad'], stern_pad=tp['stern_pad'],
                   spitz_pad=tp['spitz_pad'], knoten=tp['knoten'], parallel_rest_mm=tp['parallel_rest_mm'],
                   parallel_kette_mm=tp['parallel_kette_mm'], parallel_abgriff_mm=tp['parallel_abgriff_mm'], abgriff_nah=tp['abgriff_nah'],
                   abgriff_neben_strom_mm=tp['abgriff_neben_strom_mm'], abgriff_via_im_pad=tp['abgriff_via_im_pad'])
        out['topologie_teile'] = top
        if 'parallel_r27_mm' in tp: top['parallel_r27_mm'] = tp['parallel_r27_mm']
        out['topologie'] = round(top['stern_frei'] + top['spitz_frei'] + top['abgriff_nah'] + top.get('parallel_r27_mm', top['parallel_rest_mm']), 1)
        out['parallel_mm'] = round(top['parallel_rest_mm'] + top['parallel_kette_mm'] + top['parallel_abgriff_mm'], 1)
    except Exception as e:
        fehler['topologie'] = repr(e)[:200]


    if r27_diagnose:
        try:
            from analyse.topologie_stamm import messen as stamm_messen
            out['r27'] = stamm_messen(R, B)
        except Exception as e:
            out['diagnose_fehler'] = dict(out.get('diagnose_fehler') or {}, r27=repr(e)[:200])
    try:
        from analyse.kopplung import kopplung
        out['kopplung'] = kopplung(R, K)['summe']
    except Exception as e:
        fehler['kopplung'] = repr(e)[:200]
    try:
        from analyse.regelpruefung import pruefe
        from gemeinsam.zonen import aktiv
        if R.get('kupferbalken') and not aktiv(R):
            from analyse.kupfermodell import ersatzrouten
            R = ersatzrouten(B, R, K)
        regelbericht = pruefe(B, K, R)
        rp = _regelpruefer(regelbericht, bool(lagen))
        if 'schleifen' in regelbericht['entkopplung']:
            out['entkopplung_schleifen'] = regelbericht['entkopplung']['schleifen']
        if 'schleifen_fehler' in regelbericht['entkopplung']:
            out.setdefault('diagnose_fehler', {})['p3'] = regelbericht['entkopplung']['schleifen_fehler']
        if la_aktiv:
            rp['lagenaufbau'] = la['summe'] if la['tor_aktiv'] else 0
            rp['summe'] += rp['lagenaufbau']
            out['lagenaufbau'] = la
        out['regelpruefer_teile'] = rp; out['regelpruefer'] = rp['summe']; out['vias_unnoetig'] = rp['vias_unnoetig']
    except Exception as e:
        fehler['regelpruefer'] = repr(e)[:200]
    r39_messen(B, R, K, gndlagen, out, fehler)
    _r41_diagnose(out, B, R, K)
    _r40_diagnose(out, R, K)
    _via_diagnose(out, B, R)
    _r36_diagnose(out, B, R, K)
    _r24_diagnose(out, B, R, K)
    _k20_diagnose(out, B, R, K)
    _k17_diagnose(out, B, R, K)
    if fehler: out['fehler'] = fehler
    out['zeit_s'] = round(time.time() - t0, 2); out['cpu_s'] = round(time.process_time() - c0, 2)
    if 'M' in locals() and 'note_v3' not in fehler:
        from lernen.paar_vertrag import messsignatur, boardkennung
        out.update(paar_messversion=messsignatur(), board_kennung=boardkennung(B))
    return out


def _k17_diagnose(out, B, R, K):
    flag = (R.get('optionen') or {}).get('k17_messen',
            (K.get('vorgaben') or {}).get('k17_messen', R.get('r36_k17', False)))
    if str(flag).lower() not in ('1', 'true'):
        return
    try:
        from analyse.k17 import messen
        out['k17'] = messen(B, R, K)
    except Exception as e:
        out['diagnose_fehler'] = dict(out.get('diagnose_fehler') or {}, k17=repr(e)[:200])


def _k20_diagnose(out, B, R, K):
    flag = (R.get('optionen') or {}).get('k20_messen', (K.get('vorgaben') or {}).get('k20_messen', False))
    if str(flag).lower() in ('1', 'true'):
        from analyse.k20.kennzahl import diagnose as k20_diagnose
        k20_diagnose(out, B, R, K)


def r39_messen(B, R, K, gndlagen, out, fehler):
    ""
    out['r39_tor_aktiv'] = str((R.get('optionen') or {}).get('r39_tor', '0')) == '1'
    opt = R.get('optionen') or {}
    if not out['r39_tor_aktiv'] and str(opt.get('r39_diagnose', opt.get('r39', '0'))) != '1': return
    try:
        from analyse.strom_engstellen import engstellen
        r39 = engstellen(B, R, K, gndlagen)
        out.update(strom_engstellen=r39, strom_engstellen_mm2=r39['unnoetig_mm2'] if not r39['nicht_messbar'] and r39['abdeckung']['komponenten'] else None,
                   strom_engstellen_n=r39['unnoetig_n'] if not r39['nicht_messbar'] and r39['abdeckung']['komponenten'] else None,
                   r39_version=r39['version'], r39_parameter=r39['parameter'], r39_abdeckung=r39['abdeckung'])
        if r39['nicht_messbar']:
            out['r39_fehler'] = 'Zonenkomponenten oder Seitenaeste ohne geroutete Mittellinie nicht vollstaendig messbar'
            if out['r39_tor_aktiv']: fehler['strom_engstellen'] = out['r39_fehler']
    except Exception as e:
        out['r39_abdeckung'] = dict(vollstaendig=False, unbekannt=1, anteil=0.0)
        out['r39_fehler'] = repr(e)[:200]
        if out['r39_tor_aktiv']: fehler['strom_engstellen'] = out['r39_fehler']
def _r40_diagnose(out, R, K):
    ""
    try:
        from analyse.via_dichte import kurz
        out['r40'] = kurz(R, K)
    except Exception as e:
        out['r40'] = dict(version='r40-kurz-v1', status='messfehler')
        out.setdefault('diagnose_fehler', {})['r40'] = repr(e)[:200]


def _via_diagnose(out, B, R):
    ""
    opt = R.get('optionen') or {}
    flag = opt.get('via_dichte_diagnose', opt.get('via_dichte', '0'))
    if str(flag).lower() not in ('1', 'true'): return
    try:
        from analyse.via_dichte import messen as via_messen, SCHWELLE, FENSTER_MM
        vd = via_messen(B, R, schwelle=float(opt.get('via_dichte_schwelle', SCHWELLE)),
                       fenster=float(opt.get('via_dichte_fenster', FENSTER_MM)))
        out.update(via_dichte=vd, via_dichte_spitze=vd['spitze_vias_mm2'], via_dichte_fenster_ueber=vd['fenster_ueber'])
    except Exception as e:
        out.setdefault('diagnose_fehler', {})['via_dichte'] = repr(e)[:200]


def _r24_diagnose(out, B, R, K):
    ""
    try:
        from nachbearbeitung.r24 import messen
        suchen = str((R.get('optionen') or {}).get('r24_diagnose', '0')).lower() in ('1', 'true')
        info = messen(B, R, K, suchen=suchen)
        info.pop('stellen', None)
        out['r24'] = info
    except Exception as e:
        out.setdefault('diagnose_fehler', {})['r24'] = repr(e)[:200]


def _p26(B, K, out):
    ""
    if not B.get('fps'): return
    try:
        from analyse.dichte import diagnose
        out['p26'] = dict(diagnose(B, K, K.get('regeln')), tor_aktiv=False)
        from analyse.p26_gruppenwege import messen
        out['p26']['gruppenwege'] = messen(B, K)
    except Exception as e:
        out['p26'] = dict(tor_aktiv=False, fehler=repr(e)[:200])


def wert(m, k):
    ""
    if not m: return None
    v = m.get(k)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def r27_formtor(B, vorher, nachher, K, gndlagen=None):
    ""
    measurements = [messen(B, r, K, gndlagen=gndlagen) for r in (vorher, nachher)]
    a, b = measurements
    keys = tuple(dict.fromkeys(torfelder(a) + torfelder(b)))
    missing = [k for k in keys if wert(a, k) is None or wert(b, k) is None]
    worse = [k for k in keys if k not in missing and wert(b, k) > wert(a, k) + 1e-8]
    r27 = []
    for m in measurements:
        r = m.get('r27') or {}
        complete = all(wert(r, k) is not None and wert(r, k) >= 0 for k in (
                            'stern', 'stern_pad', 'spitz', 'spitz_pad', 'parallel_r27_mm'))
        complete = complete and r['stern_pad'] <= r['stern'] and r['spitz_pad'] <= r['spitz']
        r27.append(dict(stern_frei=r['stern']-r['stern_pad'], spitz_frei=r['spitz']-r['spitz_pad'],
                        parallel_r27_mm=r['parallel_r27_mm']) if complete else {})
    for k in ('stern_frei', 'spitz_frei', 'parallel_r27_mm'):
        if any(wert(m, k) is None for m in r27): missing.append('r27.'+k)
        elif r27[1][k] > r27[0][k] + 1e-8: worse.append('r27.'+k)
    versions = [m.get('r27', {}).get('version') for m in measurements]
    if versions[0] != versions[1] or not versions[0]: missing.append('r27.version')
    if any(m.get('fehler') for m in measurements): missing.append('bewertungsfehler')
    return dict(version='r27-formtor-v1', angenommen=not missing and not worse,
                schlechter=worse, fehlt=missing, r27_version=versions[0],
                vorher=dict(regeln={k: a.get(k) for k in keys}, r27=r27[0]),
                nachher=dict(regeln={k: b.get(k) for k in keys}, r27=r27[1]))


def torfelder(m):
    ""
    m = m or {}
    keys = TOR_SI if m.get('note_v3_version', m.get('version')) in SI_VERSIONS else TOR
    return keys + ('strom_engstellen_mm2',) if m.get('r39_tor_aktiv') else keys


def summen(ms, keys=None):
    ""
    ms = list(ms)
    if keys is None: keys = tuple(dict.fromkeys(TOR + tuple(k for m in ms for k in torfelder(m)))) + NEBEN
    out = {}
    for k in keys:
        w = [wert(m, k) for m in ms]; w = [x for x in w if x is not None]
        out[k] = round(sum(w), 3) if w else None
    return out


TOL_REL = 0.05



NI_GRENZEN = {k: dict(summe=0.0, einzeln=0.0) for k in TOR_SI + ('strom_engstellen_mm2',)}
ALPHA = 0.10
GEO = ('note_v3', 'verlust', 'uebersprechen', 'rueckweg', 'schaltknoten', 'signal', 'kopplung')


def schlechter(a, b, k, tol_rel=TOL_REL):
    ""
    if a is None or b is None: return 0
    t = max(tol_rel * abs(a), TOL_ABS.get(k, 0.5))
    return 1 if b > a + t else (-1 if b < a - t else 0)


def p_mindestens(w, n):
    ""
    if n <= 0 or w <= 0: return 1.0
    return sum(math.comb(n, i) for i in range(w, n + 1)) / 2.0 ** n


def vergleich(A, Bv, tol_rel=TOL_REL, keys=None, alpha=ALPHA):
    ""
    namen = sorted(n for n in set(A) & set(Bv) if A[n] and Bv[n])
    alle = sorted(set(A) | set(Bv))
    tore = {n: tuple(dict.fromkeys(torfelder(A.get(n)) + torfelder(Bv.get(n)))) for n in alle}
    if keys is None: keys = tuple(dict.fromkeys(TOR + tuple(k for n in alle for k in tore[n]))) + NEBEN
    tor = set(k for fs in tore.values() for k in fs)
    fehlend = [dict(board=n, groesse=k, seite=seite) for seite, daten in (('bezug', A), ('neu', Bv))
               for n in alle for k in keys if k in tore[n] and wert(daten.get(n), k) is None]
    fehlend += [dict(board=n, groesse='Bewertungsvertrag', seite='beide') for n in namen
                if torfelder(A[n]) != torfelder(Bv[n])]
    fehlend += [dict(board=n, groesse='Topologieversion', seite='beide',
                     bezug=A[n].get('topologie_version', 'topologie-v1'),
                     neu=Bv[n].get('topologie_version', 'topologie-v1')) for n in namen
                if A[n].get('topologie_version', 'topologie-v1') != Bv[n].get('topologie_version', 'topologie-v1')]
    fehlend += [dict(board=n, groesse='R39-Messversion', seite='beide',
                     bezug=A[n].get('r39_version'), neu=Bv[n].get('r39_version')) for n in namen
                if 'strom_engstellen_mm2' in tore[n] and (not A[n].get('r39_version') or
                                                        A[n].get('r39_version') != Bv[n].get('r39_version'))]
    fehlend += [dict(board=n, groesse='R39-Messparameter',
                     bezug=A[n].get('r39_parameter'), neu=Bv[n].get('r39_parameter')) for n in namen
                if 'strom_engstellen_mm2' in tore[n] and (not A[n].get('r39_parameter') or
                     A[n].get('r39_parameter') != Bv[n].get('r39_parameter'))]
    out = dict(boards=namen, je={}, tol_rel=tol_rel, alpha=alpha, vollstaendig=bool(namen) and not fehlend,
               fehlend=fehlend)
    for k in keys:
        pa = [(wert(A[n], k), wert(Bv[n], k)) for n in namen if k not in tor or k in tore[n]]
        pa = [(x, y) for x, y in pa if x is not None and y is not None]
        if not pa: out['je'][k] = None; continue
        sa, sb = sum(x for x, _ in pa), sum(y for _, y in pa)
        je = [schlechter(x, y, k, tol_rel) for x, y in pa]
        w, g = sum(1 for s in je if s > 0), sum(1 for s in je if s < 0)
        v = dict(n=len(pa), bezug=round(sa, 3), neu=round(sb, 3), delta=round(sb - sa, 3), rel=round((sb - sa) / abs(sa), 4) if abs(sa) > 1e-9 else None,
                 besser=g, schlechter=w, gleich=len(pa) - w - g, p_schlechter=round(p_mindestens(w, w + g), 4), p_besser=round(p_mindestens(g, w + g), 4),
                 tor=k in tor)
        if k in GEO:
            z = TOL_ABS.get(k, 0.5)
            q = [math.log((y + z) / (x + z)) for x, y in pa if x + z > 0 and y + z > 0]
            v['geo'] = round(math.exp(sum(q) / len(q)), 4) if q else 1.0
            richtung = 1 if v['geo'] > 1 else (-1 if v['geo'] < 1 else 0)
        else:
            richtung = 1 if sb > sa else (-1 if sb < sa else 0)
        v['urteil'] = 1 if (v['p_schlechter'] < alpha and w > g and richtung > 0) else (-1 if (v['p_besser'] < alpha and g > w and richtung < 0) else 0)
        v['statistik_urteil'] = v['urteil']
        if k in tor:
            gr = NI_GRENZEN[k]
            delta_max = max(y - x for x, y in pa)
            ni = sb - sa <= gr['summe'] and delta_max <= gr['einzeln']
            v.update(nicht_schlechter=ni, grenzen=dict(gr), einzel_max=delta_max)

            v['urteil'] = 1 if not ni else (-1 if sb < sa else 0)
        out['je'][k] = v
    return out


def annahme(a, b):
    ""
    v = vergleich({'board': a}, {'board': b})
    schlecht = [k for k, t in v['je'].items() if t and t['tor'] and t['urteil'] > 0]
    gut = [k for k, t in v['je'].items() if t and t['tor'] and t['urteil'] < 0]
    return dict(besser=v['vollstaendig'] and not schlecht and bool(gut),
                nicht_schlechter=v['vollstaendig'] and not schlecht,
                grund='Regelkennzahlen fehlen' if not v['vollstaendig'] else
                      ('Regelregression: ' + ', '.join(schlecht) if schlecht else ('Regeln besser' if gut else 'Regeln gleich')),
                vergleich=v)




def kupfer(C, S=None, clr=0.15, tol=0.03, rand=0.5):
    ""
    from shapely.geometry import Polygon, LineString, Point
    from shapely.ops import unary_union
    def poly(p): return Polygon(p['aussen'], p['loecher']).buffer(0)
    board = unary_union([poly(p) for p in C['outline']]); inner = board.buffer(-rand)
    lagen = sorted({t['layer'] for t in C.get('tracks', [])} | {z['layer'] for z in C.get('zones', [])} |
                   {L for p in C.get('pads', []) for L in (p.get('polys') or {})}, key=lambda L: (L != 'F', L == 'B', L))
    out = {}
    for L in lagen:
        cu = []
        for t in C.get('tracks', []):
            if t['layer'] == L: cu.append((LineString(t['pts']) if len(t['pts']) > 1 else Point(t['pts'][0])).buffer(t['w'] / 2))
        for v in C.get('vias', []): cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2))
        for p in C.get('pads', []):
            cu += [poly(q) for q in (p.get('polys') or {}).get(L, [])]
        stuecke = {}
        for z in C.get('zones', []):
            if z['layer'] != L: continue
            ps = [poly(q) for q in z['polys']]; cu += ps
            stuecke.setdefault(z.get('net') or '', []).extend(ps)
        for f in C.get('formen', []):
            if f['layer'] == L: cu += [poly(q) for q in f['polys']]
        kupfer_ = unary_union(cu).intersection(board)
        sperr = unary_union([Polygon(s['pts']).buffer(0) for s in (S or []) if L in s['lagen'] and s.get('fuellen')]) if S else None
        weiss = inner.difference(kupfer_.buffer(clr + tol))
        if sperr is not None and not sperr.is_empty: weiss = weiss.difference(sperr.buffer(clr))
        teile = [g for g in getattr(weiss, 'geoms', [weiss]) if g.area > 0.002]
        zs = {n: len([g for x in ps for g in getattr(unary_union([x]), 'geoms', [x]) if g.area > 1e-4]) for n, ps in sorted(stuecke.items())}
        out[L] = dict(kupfer_anteil=round(kupfer_.area / board.area, 4) if board.area else None, weiss_stellen=len(teile),
                      weiss_mm2=round(sum(g.area for g in teile), 2), zonen_stuecke=zs)
    return dict(lagen=out, kupfer_anteil=round(sum(v['kupfer_anteil'] or 0 for v in out.values()) / max(1, len(out)), 4),
                weiss_stellen=sum(v['weiss_stellen'] for v in out.values()), weiss_mm2=round(sum(v['weiss_mm2'] for v in out.values()), 2))


def text(m):
    ""
    if not m: return '-'
    f = lambda k, fm='%.1f': ('-' if m.get(k) is None else fm % m[k])
    return 'v3 %s (Verlust %s, Uebersprechen %s, Rueckweg %s, Schaltknoten %s, Signal %s) | Topologie %s | Regelpruefer %s | Knicke %s | Kopplung %s' % (
        f('note_v3'), f('verlust'), f('uebersprechen'), f('rueckweg'), f('schaltknoten'), f('signal'), f('topologie'),
        f('regelpruefer', '%d'), f('knicke', '%d'), f('kopplung'))


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    if a[:1] == ['kupfer']:
        C = json.load(open(a[1])); S = json.load(open(a[2])) if len(a) > 2 else None
        print(json.dumps(kupfer(C, S, float(kw.get('clr', '0.15')), float(kw.get('tol', '0.03')), float(kw.get('rand', '0.5'))), indent=1))
        sys.exit(0)
    if len(a) != 3: sys.exit(__doc__)
    B, R, K = (json.load(open(x)) for x in a)
    for name in ('r41_messen', 'k20_messen', 'k17_messen'):
        if name in kw:
            R = dict(R, optionen=dict(R.get('optionen') or {}, **{name: kw[name]}))
    vd_opt = {k: v for k, v in kw.items() if k in ('via_dichte_diagnose', 'via_dichte_fenster', 'via_dichte_schwelle', 'r24_diagnose')}
    if vd_opt: R = dict(R, optionen=dict(R.get('optionen') or {}, **vd_opt))
    if 'lagenaufbau' in kw:
        K = dict(K, vorgaben=dict(K.get('vorgaben') or {}, lagenaufbau=kw['lagenaufbau'] == '1'))
    if 'lagenaufbau_heuristik' in kw:
        K = dict(K, vorgaben=dict(K.get('vorgaben') or {}, lagenaufbau_heuristik=kw['lagenaufbau_heuristik'] == '1'))
    fl = kw['flaechennetze'].split(',') if 'flaechennetze' in kw else None
    m = messen(B, R, K, gndlagen=kw.get('gndlagen'), flaechennetze=fl, si_note=kw.get('si_note') == '1', si_messen=kw.get('si_messen') == '1', topologie_version=kw.get('topologie_version'))
    print(text(m)); print(json.dumps(m, indent=1))
