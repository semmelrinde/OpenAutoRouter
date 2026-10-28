""
import json, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VERSION = 'regeln-v3-vertrag-option'
SI_VERSIONS = ('v3.2', 'v3.2+r35-si2-option')
TEILE_V3 = dict(verlust=('dc', 'j', 'j_via', 'entk_L', 'leistung'), uebersprechen=('xt_opfer', 'xt_stoer', 'masse'), rueckweg=('rueck',),
                schaltknoten=('sw_flaeche', 'l_last', 'hot_L', 'hot_A'), signal=('laenge', 'c_last', 'vias', 'umweg'))
TEILE_V3_SI = dict(TEILE_V3, uebersprechen=TEILE_V3['uebersprechen'] + ('stoer',), auge=('auge_v', 'auge_h', 'auge_skew'))
TOR = ('note_v3', 'verlust', 'uebersprechen', 'rueckweg', 'topologie', 'kopplung', 'regelpruefer')
TOR_SI = TOR + ('auge', 'auge_vertikal_verlust', 'auge_horizontal_verlust', 'stoerbudget', 'stoerbudget_max')
NEBEN = ('schaltknoten', 'signal', 'knicke', 'mini', 'vias_unnoetig', 'parallel_mm', 'strom_engstellen_mm2')

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
    out['verlust'] += sum((M.get('board_teile') or {}).values())
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


def messen(B, R, K, gndlagen=None, flaechennetze=None, metriken_result=None, minimax=None, si_note=False, si_messen=False, topologie_version=None, geometrie=None):
    ""
    t0 = time.time(); c0 = time.process_time()
    R = dict(R)
    if gndlagen is not None: R['gndlagen'] = list(gndlagen) if not isinstance(gndlagen, str) else [x for x in gndlagen.split(',') if x]
    if flaechennetze is not None: R['flaechennetze'] = sorted(flaechennetze)
    R_geometrie=dict(R)
    R.setdefault('conflicts', {}); R.setdefault('failed', [])
    lagen = _gnd_lagen(R.get('gndlagen')) or []
    out = dict(version=VERSION, gndlagen=lagen, flaechennetze=sorted(R.get('flaechennetze') or []))
    fehler = {}
    try:
        from analyse.metriken_physik import metriken
        M = metriken_result if metriken_result is not None else metriken(B, R, K, gndlagen=R.get('gndlagen', lagen), minimax=minimax, si_note=si_note, si_messen=si_messen)
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
        from analyse.topologie import topologie
        tp = topologie(R, B, **({'version': topologie_version} if topologie_version is not None else {}))
        if 'version' in tp: out['topologie_version'] = tp['version']
        top = dict(stern_frei=tp['stern'] - tp['stern_pad'], spitz_frei=tp['spitz'] - tp['spitz_pad'], stern_pad=tp['stern_pad'],
                   spitz_pad=tp['spitz_pad'], knoten=tp['knoten'], parallel_rest_mm=tp['parallel_rest_mm'],
                   parallel_kette_mm=tp['parallel_kette_mm'], parallel_abgriff_mm=tp['parallel_abgriff_mm'], abgriff_nah=tp['abgriff_nah'],
                   abgriff_neben_strom_mm=tp['abgriff_neben_strom_mm'], abgriff_via_im_pad=tp['abgriff_via_im_pad'])
        out['topologie_teile'] = top
        out['topologie'] = round(top['stern_frei'] + top['spitz_frei'] + top['abgriff_nah'] + top['parallel_rest_mm'], 1)
        out['parallel_mm'] = round(top['parallel_rest_mm'] + top['parallel_kette_mm'] + top['parallel_abgriff_mm'], 1)
    except Exception as e:
        fehler['topologie'] = repr(e)[:200]
    try:
        from analyse.kopplung import kopplung
        out['kopplung'] = kopplung(R, K)['summe']
    except Exception as e:
        fehler['kopplung'] = repr(e)[:200]
    try:
        from analyse.regelpruefung import pruefe
        regelbericht = pruefe(B, K, R)
        rp = _regelpruefer(regelbericht, bool(lagen))
        if 'schleifen' in regelbericht['entkopplung']:
            out['entkopplung_schleifen'] = regelbericht['entkopplung']['schleifen']
        if 'schleifen_fehler' in regelbericht['entkopplung']:
            out.setdefault('diagnose_fehler', {})['p3'] = regelbericht['entkopplung']['schleifen_fehler']
        if 'M' in locals():
            rp['summe']-=rp['entkopplung_abstand']+rp['entkopplung_lagenwechsel']
            loops={}
            for h in M.get('entkopplung') or []:
                key=(h['chip'],h['netz'],h['pin'])
                loops[key]=min(loops.get(key,float('inf')),h['L_nH'])
            soll=float((K.get('vorgaben') or {}).get('entkopplung_L_nH') or 2.)
            rp['entkopplung_physik']=sum(max(0.,L/soll-1.) for L in loops.values())
            rp['summe']+=rp['entkopplung_physik']
        out['regelpruefer_teile'] = rp; out['regelpruefer'] = rp['summe']; out['vias_unnoetig'] = rp['vias_unnoetig']
    except Exception as e:
        fehler['regelpruefer'] = repr(e)[:200]
    from analyse.regelkennzahlen import r39_messen
    r39_messen(B, R, K, gndlagen, out, fehler)
    if fehler: out['fehler'] = fehler
    from analyse import vertrag as VT
    if geometrie is None:
        from analyse.bewertung import bewerte
        try:geometrie=bewerte(B,R_geometrie,K,v3=False,minimax=False)
        except Exception as e:
            geometrie=dict(gueltig=None);fehler['geometrie']=repr(e)[:200];out['fehler']=fehler
    if 'M' in locals():
        out['note_v3_version']=M.get('version')
        out['messversion']=M.get('messversion')
        out['tor']=VT.zustand(dict(geometrie,note_v3_version=M.get('version')),out,VT.pflichtnachweise(M,K))
    else:out['tor']=VT.zustand(geometrie,out)
    if 'M' in locals() and 'note_v3' not in fehler:
        from lernen.paar_vertrag import messsignatur, boardkennung
        out.update(paar_messversion=messsignatur(), board_kennung=boardkennung(B))
    out['zeit_s'] = round(time.time() - t0, 2); out['cpu_s'] = round(time.process_time() - c0, 2)
    return out


def wert(m, k):
    ""
    if not m: return None
    v = m.get(k)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


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
    from analyse import vertrag as VT
    def e(r):
        r=r or {}
        return dict(gueltig=(r.get('tor') or {}).get('gueltig', True), regeln=r,
                    nachweise=(r.get('tor') or {}).get('nachweise') or {}, note_v3_version=(r.get('tor') or {}).get('note_version') or r.get('note_v3_version') or r.get('version'))
    t=VT.vergleich(e(a),e(b))
    t['vergleich']=vergleich({'board':a},{'board':b})
    if any((m or {}).get('r39_tor_aktiv') for m in (a,b)) and not t['vergleich']['vollstaendig']:
        t.update(besser=False,nicht_schlechter=False,grund='R39-Messvertrag fehlt oder verschieden')
    return t





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
    fl = kw['flaechennetze'].split(',') if 'flaechennetze' in kw else None
    m = messen(B, R, K, gndlagen=kw.get('gndlagen'), flaechennetze=fl, si_note=kw.get('si_note') == '1', si_messen=kw.get('si_messen') == '1', topologie_version=kw.get('topologie_version'))
    print(text(m)); print(json.dumps(m, indent=1))
