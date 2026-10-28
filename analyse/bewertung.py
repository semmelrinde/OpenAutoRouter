""
import json, sys, math, collections, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.fluss import analyse as fluss_analyse, verengung as fluss_verengung, zielbreite as fluss_zielbreite, padgeo, via_radius, NetzgraphCache
from shapely.geometry import Point

GEW = {'sw': 4.0, 'strom': 2.5, 'versorgung': 1.0, 'schnell': 3.0, 'analog': 2.0, 'signal': 1.0}


def mst(pts):
    if len(pts) < 2: return 0.0
    inT = [pts[0]]; rest = list(pts[1:]); L = 0.0
    while rest:
        d, i = min((min(math.dist(r, q) for q in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L


def gnd_flaechen(R, GND, K=None):
    ""
    from analyse.flaechen import aktiv
    if not aktiv(R, K or {}):
        gedeckt = set(R.get('flaechennetze') or []) & GND
        raw = (R.get('optionen') or {}).get('gndlagen') or []
        labels = (raw.split(',') if isinstance(raw, str) else list(raw))
        gl = R.get('gndlagen') or []
        labels += gl.split(',') if isinstance(gl, str) else list(gl)
        for label in labels:
            if not label: continue
            if '=' in str(label):
                for name in str(label).split('=', 1)[1].split('|'):
                    gedeckt.update(n for n in GND if n == name or n == '/' + name)
            elif len(GND) == 1: gedeckt.update(GND)
            else: gedeckt.update(n for n in GND if n.split('/')[-1] == 'GND')
        return gedeckt
    from analyse.flaechen import zuordnung
    raw = (R.get('optionen') or {}).get('gndlagen')
    if raw is None: raw = R.get('gndlagen') or []
    labels = raw.split(',') if isinstance(raw, str) else list(raw)
    layer_names = [str(x).split('=', 1)[0] for x in labels]
    L = max([int(x[2:]) + 2 for x in layer_names if x.startswith('In') and x[2:].isdigit()] + [2])
    B = dict(lagen=L, pads=[dict(net=n) for n in sorted(GND)])
    mapping = zuordnung(B, R, dict(gnd=sorted(GND)))
    gedeckt = ({n for nn in mapping.values() for n in nn} if labels else set(R.get('flaechennetze') or [])) & GND
    return gedeckt


def kupfer_fingerabdruck(R):
    ""
    import hashlib
    return hashlib.sha256(json.dumps({k: R.get(k) for k in ('tracks', 'vias', 'conflicts', 'failed')}, sort_keys=True).encode()).hexdigest()


def native_flaechen(R, K):
    ""
    proof = R.get('native_flaechen')
    d = R.get('drc')
    if not isinstance(proof, dict) or proof.get('version') != 'native-flaechen-v1': return set()
    if not isinstance(d, dict) or d.get('version') != 'korpus-drc-v1' or 'fehler' in d: return set()
    keys = ('alle', 'platz', 'routing', 'unverbunden', 'zonen_kupfer')
    if not all(type(d.get(k)) is int and d[k] >= 0 for k in keys): return set()
    if any(d[k] for k in ('routing', 'unverbunden', 'zonen_kupfer')): return set()
    if d.get('routes_sha256') != kupfer_fingerabdruck(R): return set()
    if not isinstance(proof.get('netze'), list): return set()
    return {n for n in proof['netze'] if (K.get('netze', {}).get(n) or {}).get('art') in ('gnd', 'strom', 'versorgung')}


def unverbunden(B, R, GND, fluss_cache=None, K=None):
    ""
    from gemeinsam.zonen import vereinheitlichen
    R = vereinheitlichen(R)
    from analyse.flaechen import aktiv, kupfer, zuordnung, verbunden
    KK = K or dict(gnd=sorted(GND), lagen=B.get('lagen') or int((R.get('optionen') or {}).get('lagen', 2)))
    FR = kupfer(B, R, KK) if aktiv(R, KK) else R
    fn = ({n for nn in zuordnung(B, R, KK).values() for n in nn} |
          {z['net'] for z in FR.get('zones', [])}) if aktiv(R, KK) else set()
    if any(z.get('filled') for z in FR.get('zones', [])):
        from analyse.kupfermodell import unverbunden_polygone
        return unverbunden_polygone(B, FR, set(GND) - fn)
    pads = collections.defaultdict(list); tr = collections.defaultdict(list); vi = collections.defaultdict(list)
    for p in B['pads']:
        if p['net'] and (p['net'] not in GND or p['net'] in fn): pads[p['net']].append(p)
    for t in R.get('tracks', []): tr[t['net']].append(t)
    for v in R.get('vias', []): vi[v['net']].append(v)
    out = []
    for n, P in sorted(pads.items()):
        keys = sorted({'%s:%s' % (p['ref'], p['num']) for p in P})
        if len(keys) < 2: continue
        if n in fn:
            if not verbunden(B, FR, KK, n): out.append(n)
            continue
        if tr[n]:
            info = fluss_analyse(tr[n], vi[n], P, {'quelle': [keys[0]], 'last': {}},
                                 graph=fluss_cache.graph(tr[n], vi[n], P, endpunkte=False) if fluss_cache is not None else None)
            if info is None or info['pads_erreicht'] < info['pads']: out.append(n)
            continue

        teil = {k: k for k in keys}
        def wurzel(k):
            while teil[k] != k: k = teil[k]
            return k
        G = {id(p): padgeo(p) for p in P}
        for v in vi[n]:
            ks = sorted({'%s:%s' % (p['ref'], p['num']) for p in P
                         if min(G[id(p)][L].distance(Point(v['x'], v['y'])) for L in p['layers']) <= via_radius(v) + 0.03})
            for k in ks[1:]: teil[wurzel(k)] = wurzel(ks[0])
        for i, p in enumerate(P):
            for q in P[i + 1:]:
                gem = set(p['layers']) & set(q['layers'])
                if any(G[id(p)][L].distance(G[id(q)][L]) <= 1e-6 for L in gem):
                    teil[wurzel('%s:%s' % (p['ref'], p['num']))] = wurzel('%s:%s' % (q['ref'], q['num']))
        if len({wurzel(k) for k in keys}) > 1: out.append(n)
    return out


GETRENNT = 'Von Quelle getrennte Verbraucher: '


def ir_block(B, R, K, zonen=None):
    ""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'codex', 'ir_drop'))
        from solver import ir_kennzahlen
        def loese(KK):
            if zonen is None: return ir_kennzahlen(B, R, KK)
            from codex.zonen_ir.integration import zonen_ir_kennzahlen
            return zonen_ir_kennzahlen(B, R, KK, options=zonen)
        I = loese(K); ohne = {}
        for n, v in sorted(I['netze'].items()):
            f = str(v.get('error') or '')
            if v.get('status') == 'error' and f.startswith(GETRENNT): ohne[n] = sorted(x.strip() for x in f[len(GETRENNT):].split(','))
        if ohne:
            import copy
            K2 = dict(K); K2['netze'] = dict(K['netze'])
            for n, weg in ohne.items():
                k = K2['netze'][n] = copy.deepcopy(K['netze'][n]); fl = k.get('fluss') or {}
                for s in weg:
                    (fl.get('last') or {}).pop(s, None); (fl.get('herkunft') or {}).pop(s, None)
            I2 = loese(K2)
            for n in ohne:
                if (I2['netze'].get(n) or {}).get('status') == 'solved': I['netze'][n] = I2['netze'][n]
            rest = sum(1 for v in I['netze'].values() if v.get('status') == 'error')
            I['anzahl_fehler'] = rest
            if not rest: I['status'] = 'solved'
    except Exception as e:
        return dict(status='fehler', fehler=repr(e)[:200], verlust_mW=None)
    ok = {n: v for n, v in I['netze'].items() if v.get('status') == 'solved'}
    return dict(status=I.get('status'), verlust_mW=1000 * sum(float(v['power_W']) for v in ok.values()),
                verlust_je_netz_mW={n: 1000 * float(v['power_W']) for n, v in sorted(ok.items())},
                max_abfall_mV={n: 1000 * float(v['max_drop_V']) for n, v in sorted(ok.items())},
                strom_je_netz_A={n: float(v.get('total_current_A') or v.get('source_current_A') or 0.0) for n, v in sorted(ok.items())},
                strommodus_je_netz={n: v.get('strommodus') for n, v in sorted(ok.items())},
                fehler_netze=sorted(n for n, v in I['netze'].items() if v.get('status') == 'error'), ohne_last=ohne,
                strommodus=sorted({v.get('strommodus') for v in ok.values()}), fehler=I.get('anzahl_fehler'), zonen_modell=I.get('zonen_modell'))


MU0 = 1.2566
RHO = 1.72e-5


def sw_impedanz(B, R, K, n):
    ""
    VG = K.get('vorgaben') or {}; fsw_vg = VG.get('fsw') or {}
    chips = sorted({p['ref'] for p in B['pads'] if p['net'] == n and K['teile'].get(p['ref'], {}).get('rolle') == 'chip'})
    f = next((float(fsw_vg[c]) for c in chips if c in fsw_vg), None); f_q = 'vorgabe' if f else 'annahme_1MHz'
    f = f or 1e6
    h = 0.1; Rs = 0.0; Ls = 0.0
    for t in R['tracks']:
        if t['net'] != n: continue
        dick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            l = math.dist(a, b); Rs += RHO * l / (w * dick); Ls += MU0 * h / (w + 2 * h) * l
    nv = sum(1 for v in R['vias'] if v['net'] == n); Rs += 0.0015 * nv; Ls += 0.3 * nv
    Z = math.hypot(Rs, 2 * math.pi * f * Ls * 1e-9)
    fl = (K['netze'].get(n) or {}).get('fluss') or {}
    I = fl.get('gesamt') if fl.get('einheit') == 'A' else 1.0
    return dict(R_mOhm=1000 * Rs, L_nH=Ls, f_Hz=f, f_quelle=f_q, Z_mOhm=1000 * Z, I_A=I,
                verlust_ac_mW=1000 * I * I * Rs, abfall_ac_mV=1000 * I * Z)


def budgets(K):
    ""
    import re
    VG = K.get('vorgaben') or {}; je, gr = {}, {}
    for n in K['netze']:
        for rx, v in (VG.get('budget_mV') or {}).items():
            if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): je[n] = float(v); break
    for name, g in (VG.get('kanal_budget_mV') or {}).items():
        nn = [n for n in g.get('netze', []) if n in K['netze']]
        if nn: gr[name] = (nn, float(g['mV']))
    return je, gr


def budget_ueber(I, K):
    ""
    je, gr = budgets(K)
    if not je and not gr: return {}
    d = I.get('max_abfall_mV') or {}
    out = {n: round(d[n] - b, 3) for n, b in je.items() if n in d and d[n] > b}
    for name, (nn, b) in gr.items():
        s = sum(d.get(n, 0.0) for n in nn)
        if s > b: out['kanal:' + name] = round(s - b, 3)
    return out


def dc_werte(E, B, R, K, fluss_cache=None):
    ""
    I = E.get('ir')
    if not I or I.get('verlust_mW') is None: return None
    out = {n: (float(I['max_abfall_mV'][n]), float(I['verlust_je_netz_mW'][n]), 'ir') for n in I['max_abfall_mV']}
    fehlt = [n for n in I.get('fehler_netze') or [] if n not in out]
    if fehlt:
        from analyse.metriken import gleichstrom_schaetzung
        for n, (dv, p) in gleichstrom_schaetzung(B, R, K, fehlt, wie_loeser=True, fluss_cache=fluss_cache).items(): out[n] = (dv, p, 'schaetzung')
    return out


def note_v2(E, B, R, K, dc=None):
    ""
    I = E.get('ir')
    if dc is None: dc = dc_werte(E, B, R, K)
    if dc is None or not I: return None, None
    N = K['netze']; GND = set(K['gnd'])
    strom = {n for n, k in N.items() if k.get('art') in ('strom', 'versorgung', 'sw')}
    sw_netze = {n for n, k in N.items() if k.get('art') == 'sw'}
    laenge = sum(GEW.get(N.get(n, {}).get('art', 'signal'), 1.0) * l for n, l in E['_L'].items() if n not in strom and n not in GND)
    vias = sum(1 for v in R['vias'] if v['net'] not in strom and v['net'] not in GND)
    S = laenge + 0.6 * vias + 50 * E['konflikte'] + 200 * E['offen']
    dc_P = sum(v[1] for n, v in sorted(dc.items()) if n not in sw_netze)
    dc_D = sum(v[0] for n, v in sorted(dc.items()) if n not in sw_netze)
    sw = {n: sw_impedanz(B, R, K, n) for n in sorted(sw_netze) if (N[n].get('fluss') or {}).get('gesamt', 0) > 0}
    teile = dict(S=S, verlust=2 * dc_P, abfall=dc_D, sw_verlust=2 * sum(z['verlust_ac_mW'] for z in sw.values()),
                 sw_abfall=sum(z['abfall_ac_mV'] for z in sw.values()))
    ueber = budget_ueber(dict(max_abfall_mV={n: v[0] for n, v in dc.items()}), K)
    if ueber: teile['budget'] = 20 * sum(ueber.values())
    ersatz = sorted(n for n, v in dc.items() if v[2] != 'ir')
    return sum(teile.values()), dict(teile=teile, sw=sw, budget_ueber=ueber, ersatz=ersatz, dc={n: [round(v[0], 3), round(v[1], 3), v[2]] for n, v in sorted(dc.items())})


def bewerte(B, R, K, ir=False, zonen=None, v3=False, minimax=None, laengentor=False, si_note=False, si_messen=False, fluss_cache=None):
    from analyse.standard import ziel as standardziel
    ziel = standardziel(K.get('vorgaben'))
    if minimax is not None: ziel = 'minimax' if minimax else 'v3'
    if ziel not in ('v3', 'minimax'): raise ValueError('bewertung muss v3 oder minimax sein')
    from gemeinsam.zonen import aktiv, vereinheitlichen, boardgeometrie
    B = boardgeometrie(B, R)
    from analyse.r44 import messen as r44_messen, bestanden as r44_bestanden
    r44 = r44_messen(B, R)
    r36 = aktiv(R)
    if any(z.get('r41') for z in R.get('zonen', [])):
        R = dict(R, r41_fuellung_stat=dict(geschnitten_mm2=sum((z.get('r41') or {}).get('geschnitten_mm2', 0) for z in R['zonen'])))
    R = vereinheitlichen(R)
    v3 = v3 or ziel == 'minimax'
    endpunkte = bool(R.get('strom_form') or str((R.get('optionen') or {}).get('strom_fix', '0')) == '1')
    if fluss_cache is None or (endpunkte and not fluss_cache.endpunkte):
        fluss_cache = NetzgraphCache(endpunkte=endpunkte)
    infos = {}
    N = K['netze']; GND = set(K['gnd'])
    pads = collections.defaultdict(list)
    for p in B['pads']:
        if p['net']: pads[p['net']].append((p['x'], p['y']))
    L = collections.defaultdict(float); defizit = 0.0
    pads_n = collections.defaultdict(list)
    for p in B['pads']: pads_n[p['net']].append(p)
    soll = {}
    for n, k in N.items():
        if k.get('art') not in ('strom', 'versorgung') or not k.get('fluss'): continue
        tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
        vi = [v for v in R['vias'] if v['net'] == n]
        info = fluss_analyse(tr, vi, pads_n[n], k['fluss'], graph=fluss_cache.graph(tr, vi, pads_n[n]))
        infos[n] = info
        if info:
            for ti, t in enumerate(tr):
                for si in range(len(t['w'])): soll[(id(t), si)] = fluss_zielbreite(info['anteil'].get((ti, si), 1.0), k['wmin'], k['w'])


    from gemeinsam.laenge import aussen
    AU = aussen(R, B); L_ges = 0.0
    for ti, t in enumerate(R['tracks']):
        k = N.get(t['net'], {})
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            d = AU[ti][si]; L[t['net']] += d; L_ges += math.dist(a, b)
            if k.get('art') in ('strom', 'versorgung', 'sw') and not t.get('sig'): defizit += max(0.0, soll.get((id(t), si), k['w']) - w) * d
    art_L = collections.defaultdict(float); gew = 0.0; umwege = []
    for n, l in L.items():
        a = 'gnd' if n in GND else N.get(n, {}).get('art', 'signal'); art_L[a] += l
        if n in GND: continue
        gew += GEW.get(a, 1.0) * l
        m = mst(pads[n])
        if l > m + 1.0: umwege.append((round(l - m, 2), n, round(l, 1), round(m, 1)))
    umwege.sort(reverse=True)
    diagnose = 'conflicts' in R and 'failed' in R
    konf = len(R.get('conflicts') or {}); offen = len(R.get('failed') or [])
    unv = unverbunden(B, R, gnd_flaechen(R, GND, K) | native_flaechen(R, K), fluss_cache=fluss_cache, K=K)
    vias = len(R['vias'])
    veng = 0.0; veng_stellen = []
    for n, k in N.items():
        if k.get('art') not in ('strom', 'versorgung') or not k.get('fluss'): continue
        tr = [t for t in R['tracks'] if t['net'] == n and not t.get('sig')]
        info = infos.get(n)
        v, st = fluss_verengung(tr, info)
        veng += v; veng_stellen += [(x[0], n, x[3], x[4], x[5]) for x in st[:3]]
    veng_stellen.sort(reverse=True)
    summanden = dict(laenge=gew, vias=0.6 * vias, defizit=10 * defizit, verengung=10 * veng, konflikte=50 * konf, offen=200 * offen)
    note = sum(summanden.values())
    rg = K.get('regeln') or {}; rg = rg.get('router', rg)
    from analyse.metriken import review_aktiv
    minimum = max(float(rg.get('bahn_min') or 0),
                  float((R.get('optionen') or {}).get('bahn_min') or 0)) if review_aktiv(K) else 0.0
    unter = [dict(netz=t['net'], lage=t['layer'], bahn=ti, segment=si, breite=w, minimum=minimum)
             for ti, t in enumerate(R['tracks']) for si, w in enumerate(t['w']) if w < minimum - 1e-9]
    eingang_fehler = (R.get('kurzschluss_schutz') or {}).get('eingang_pad_kontakte') or []
    gueltig = diagnose and konf == 0 and offen == 0 and not unv and not unter and not eingang_fehler and r44_bestanden(r44)
    E_ir = ir_block(B, R, K, zonen=zonen) if ir else None
    if (R.get('kupferbalken') or r36) and (v3 or ir):
        from analyse.kupfermodell import ir_werte
        E_ir = ir_werte(B, R, K)
        if r36 and E_ir['fehler_netze']:
            E_ir = ir_werte(B, R, K, res=.05)

        betroffen = {z['net'] for z in R.get('zones', []) if z.get('kupferbalken') or z.get('r36')}
        if betroffen.intersection(E_ir['fehler_netze']) or (r36 and E_ir['fehler_netze']):
            gueltig = False
    E = dict(ir=E_ir, _L=dict(L), note=round(note, 1), note_raw=note, summanden=summanden, bewertung_version='note-v1', gueltig=gueltig, rang=[0 if gueltig else 1, offen + len(unv), konf, round(note, 1)],
                r44=r44, eingang_kupfer_fehler=eingang_fehler, konflikte=konf, offen=offen, unverbunden=unv, diagnose=diagnose, vias=vias, gewichtet=round(gew, 1),
                laenge={a: round(v, 1) for a, v in sorted(art_L.items())}, laenge_gesamt=round(L_ges, 1), laenge_im_pad=round(L_ges - sum(L.values()), 1), defizit=round(defizit, 2),
                verengung=round(veng, 2), verengungen=veng_stellen[:10],
                umweg_summe=round(sum(u[0] for u in umwege), 1), umwege=umwege[:15])
    if unter: E['mindestbreitenverletzungen'] = unter
    dc = dc_werte(E, B, R, K, fluss_cache=fluss_cache) if ir else None
    v2, v2d = note_v2(E, B, R, K, dc=dc) if ir else (None, None)
    E.update(note_v2=round(v2, 1) if v2 is not None else None, note_v2_raw=v2, note_v2_teile=v2d,
             note_v2_vollstaendig=(not v2d['ersatz']) if v2d else None)
    E['strompfade'] = strompfade(B, R, K, E, dc=dc)
    if v3:
        from analyse.metriken import metriken
        M = metriken(B, R, K, ir=E_ir if (E_ir and (E_ir.get('max_abfall_mV') or R.get('kupferbalken'))) else None, AU=AU, fluss_cache=fluss_cache, minimax=ziel == 'minimax', si_note=si_note, si_messen=si_messen or laengentor)
        E.update(note_v3=round(M['note_v3'] + 50 * konf + 200 * offen, 1), note_v3_netze=M['note_v3'], metriken=M,
                 note_v3_version=M.get('version'))
        if ziel == 'minimax':
            E.update({k: M[k] for k in ('minimax', 'minimax_version', 'minimax_profil', 'minimax_rang', 'minimax_summe',
                                          'minimax_vollstaendig', 'minimax_boardvertrag', 'minimax_quellen', 'engpass', 'sicherheit')})
            E['bewertungsziel'] = ziel
            p0, _ = strom_verlust_gleich(E, E, B, R, B, R, K)
            E['minimax_strompfade'] = dict(umweg_mm=E['strompfade']['umweg_mm'], verlust_mW=p0)
            from analyse import minimax as MM
            drc = R.get('drc')
            if 'drc' in R:
                schemas = {'drc-v1': ('fehler', 'offen', 'routing', 'unverbunden'),
                           'korpus-drc-v1': ('alle', 'platz', 'routing', 'unverbunden')}
                keys = schemas.get(drc.get('version'), ()) if isinstance(drc, dict) and isinstance(drc.get('version'), str) else ()
                bekannt = bool(keys) and all(
                    isinstance(drc.get(k), int) and not isinstance(drc[k], bool) and drc[k] >= 0 for k in keys)
                if not bekannt:
                    E['sicherheit']['fehlend'].append(dict(netz='Board', kriterium='DRC', grund='vollstaendiges versioniertes DRC-Schema fehlt'))
                    E['minimax_vollstaendig'] = False

                if isinstance(drc, dict) and any(isinstance(drc.get(k), int) and not isinstance(drc[k], bool) and drc[k] > 0 for k in ('fehler', 'offen', 'routing', 'unverbunden', 'alle', 'platz')):
                    E['sicherheit']['fehler'].append(dict(netz='Board', kriterium='DRC', grund='externe DRC nicht fehlerfrei'))

            gnd_offen = [n for n in unv if n in GND]
            if gnd_offen:
                E['sicherheit']['fehler'].extend(dict(netz=n, kriterium='verbunden', grund='GND ohne eigene Flaeche unverbunden') for n in gnd_offen)
                E['sicherheit']['sicher'] = False
            MM.sicherheitsstatus(E['sicherheit'])
            E['minimax_status'] = ('ungueltig' if not E['gueltig'] or E['sicherheit']['fehler'] else
                                   'unbekannt' if not E['minimax_vollstaendig'] else 'gueltig')
            E['minimax_gueltig'] = None if E['minimax_status'] == 'unbekannt' else E['minimax_status'] == 'gueltig'
            if ziel == 'minimax':
                E['bewertung_version'] = E['minimax_version']
                E['gueltig_routing'] = E['gueltig']


        from analyse import regelkennzahlen as RK
        E['regeln'] = RK.messen(B, R, K, metriken_result=M, si_note=si_note, r44_result=r44)
        from analyse.flaechen import aktiv
        if aktiv(R, K) and E['regeln'].get('fehler'):
            E.update(gueltig=False, bewertung_fehler=E['regeln']['fehler'])
            E['rang'][0] = 1
        if (K.get('vorgaben') or {}).get('physik_version') == 'v3.3-vertrag-option':
            from analyse import vertrag as VT
            E['nachweise'] = VT.pflichtnachweise(M, K)
            E['geometrisch_gueltig'] = gueltig
            E['tor'] = VT.zustand(E)
            E['vollstaendig_gueltig'] = E['tor']['vollstaendig_gueltig']
            E['regeln']['tor'] = E['tor']
    if (si_messen or si_note or laengentor) and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('laengengruppe') for k in N.values()) or (K.get('vorgaben') or {}).get('laengengruppen')):
        from analyse.laengenimpedanz import messen as _si_messen
        E['signalintegritaet'] = M['signalintegritaet'] if v3 else _si_messen(B, R, K)
        E['si_note_aktiv'] = bool(v3 and si_note)
        if laengentor and not E['signalintegritaet']['laengentor']:
            E['gueltig'] = False
            E['rang'][0] = 1
            E['regelverstoesse_laenge'] = [g for g, q in E['signalintegritaet']['laengengruppen'].items()
                                          if not q['vollstaendig'] or q['ueber_mm'] > 1e-6]


    if 'minimax_status' in E and not E['gueltig']:
        E['minimax_status'] = 'ungueltig'; E['minimax_gueltig'] = False
    return E


def strompfade(B, R, K, E, schaetzen=False, dc=False):
    ""
    N = K['netze']; GND = set(K['gnd'])
    netze = sorted(n for n, k in N.items() if k.get('art') in ('strom', 'versorgung', 'sw') and n not in GND)
    pads = collections.defaultdict(list)
    for p in B['pads']:
        if p['net']: pads[p['net']].append((p['x'], p['y']))
    L = E['_L']; um = {}
    for n in netze:
        if n in L: um[n] = round(max(0.0, L[n] - mst(pads[n])), 3)
    if dc is False: dc = dc_werte(E, B, R, K)
    P = {n: v[1] for n, v in dc.items()} if dc is not None else {}
    fehlt = [n for n in netze if n not in P and n in L]
    if fehlt and (dc is not None or schaetzen):
        from analyse.metriken import gleichstrom_schaetzung
        for n, (_dv, p) in gleichstrom_schaetzung(B, R, K, fehlt, wie_loeser=True).items(): P[n] = p
    voll = dc is not None or schaetzen
    return dict(umweg_mm=round(sum(um.values()), 2), verlust_mW=round(sum(P.get(n, 0.0) for n in netze if n in L), 3) if voll else None,
                quelle='ir' if dc is not None else ('baum' if schaetzen else None),
                je_netz={n: [um.get(n, 0.0), round(P[n], 3) if n in P else None] for n in netze if n in L})
def vergleich(E0, E1, B0, R0, B1, R1, K, note='v2', tol_umweg_mm=1.0, tol_verlust=0.02, K1=None):
    ""
    if note == 'v3' and (E0.get('bewertungsziel') == 'minimax' or E1.get('bewertungsziel') == 'minimax'): note = 'minimax'
    out = dict(note=note)
    if note in ('v3', 'minimax'):
        key = 'note_v3_version' if note == 'v3' else 'minimax_version'
        if not E0.get(key) or E0.get(key) != E1.get(key):
            return dict(note=note, delta=None, besser=False, grund='verschiedene/fehlende Bewertungsversionen')
    if note == 'minimax':
        from analyse import minimax as MM
        return dict(note=note, **MM.annahme(E0, E1, tol_umweg_mm, tol_verlust))
    if any(e.get('tor') or (e.get('regeln') or {}).get('messversion') for e in (E0,E1)):
        from analyse.vertrag import vergleich as vertrag_vergleich
        c=vertrag_vergleich(E0,E1)
        if not c['nicht_schlechter']:return dict(note=note,delta=None,besser=False,grund=c['grund'])
    r0, r1 = tuple(E0['rang'][:3]), tuple(E1['rang'][:3])
    if r0 != r1:
        out.update(delta=None, besser=r1 < r0, grund='harte Tore %s -> %s' % (list(r0), list(r1))); return out
    if note == 'v2':
        d0, d1 = dc_werte(E0, B0, R0, K), dc_werte(E1, B1, R1, K)
        if d0 is None or d1 is None:
            out.update(note='v1', delta=E1['note_raw'] - E0['note_raw'], hinweis='IR fehlt auf einer Seite: note-v1 auf beiden Seiten')
        else:
            from analyse.metriken import gleichstrom_schaetzung
            beide = {n for n in set(d0) | set(d1) if d0.get(n, (0, 0, ''))[2] == 'ir' and d1.get(n, (0, 0, ''))[2] == 'ir'}
            rest = sorted((set(d0) | set(d1)) - beide)
            s0 = gleichstrom_schaetzung(B0, R0, K, rest, wie_loeser=True) if rest else {}
            s1 = gleichstrom_schaetzung(B1, R1, K, rest, wie_loeser=True) if rest else {}
            c0 = {n: d0[n] for n in beide}; c1 = {n: d1[n] for n in beide}
            for n in rest:
                if n in s0: c0[n] = s0[n] + ('schaetzung',)
                if n in s1: c1[n] = s1[n] + ('schaetzung',)
            v0, _ = note_v2(E0, B0, R0, K, dc=c0); v1, _ = note_v2(E1, B1, R1, K, dc=c1)
            out.update(delta=v1 - v0, gleich_gerechnet=rest)
    elif note == 'v3':
        if E0.get('note_v3') is None or E1.get('note_v3') is None: raise ValueError('note_v3 fehlt: bewerte(..., v3=True)')
        out['delta'] = E1['note_v3'] - E0['note_v3']
    else:
        out['delta'] = E1['note_raw'] - E0['note_raw']
    s0 = E0.get('strompfade') or strompfade(B0, R0, K, E0, dc=None); s1 = E1.get('strompfade') or strompfade(B1, R1, K, E1, dc=None)
    du = (s1.get('umweg_mm') or 0.0) - (s0.get('umweg_mm') or 0.0)
    p0, p1 = strom_verlust_gleich(E0, E1, B0, R0, B1, R1, K)
    dp = p1 - p0
    schlechter = du > tol_umweg_mm and dp > tol_verlust * max(1e-9, p0)
    out.update(verlust_strom_vorher_mW=round(p0, 3))
    out.update(umweg_strom_mm=round(du, 2), verlust_strom_mW=round(dp, 3), strompfade_schlechter=schlechter)
    from analyse import regelkennzahlen as RK
    vg = RK.annahme(E0.get('regeln') or RK.messen(B0, R0, K),
                    E1.get('regeln') or RK.messen(B1, R1, K1 if K1 is not None else K))
    out['regeltor'] = vg


    out['besser'] = out['delta'] < -1e-6 and vg['nicht_schlechter'] and not schlechter
    out['grund'] = 'Strompfade: Umweg %+.1f mm, Verlust %+.1f mW' % (du, dp) if (schlechter and out['delta'] < 0) else ('Note %+.1f' % out['delta'])
    if not vg['nicht_schlechter']: out['grund'] = vg['grund']
    return out


def strom_verlust_gleich(E0, E1, B0, R0, B1, R1, K):
    ""
    from analyse.metriken import gleichstrom_schaetzung
    N = K['netze']; GND = set(K['gnd'])
    netze = sorted(n for n, k in N.items() if k.get('art') in ('strom', 'versorgung', 'sw') and n not in GND)
    d0, d1 = dc_werte(E0, B0, R0, K) or {}, dc_werte(E1, B1, R1, K) or {}
    beide = {n for n in netze if (d0.get(n) or (0, 0, ''))[2] == 'ir' and (d1.get(n) or (0, 0, ''))[2] == 'ir'}
    rest = [n for n in netze if n not in beide]
    s0 = gleichstrom_schaetzung(B0, R0, K, rest, wie_loeser=True) if rest else {}
    s1 = gleichstrom_schaetzung(B1, R1, K, rest, wie_loeser=True) if rest else {}
    P0 = sum(d0[n][1] for n in sorted(beide)) + sum(s0[n][1] for n in rest if n in s0)
    P1 = sum(d1[n][1] for n in sorted(beide)) + sum(s1[n][1] for n in rest if n in s1)
    return P0, P1


def besser(E1, E2):
    ""
    if E1.get('bewertungsziel') == 'minimax' or E2.get('bewertungsziel') == 'minimax': return besser_v3(E1, E2)
    return tuple(E1['rang']) < tuple(E2['rang'])


def besser_v3(E1, E2):
    ""
    if any(e.get('tor') or (e.get('regeln') or {}).get('messversion') for e in (E1,E2)):
        from analyse.vertrag import vergleich
        return vergleich(E2,E1)['besser']
    if E1.get('bewertungsziel') == 'minimax' or E2.get('bewertungsziel') == 'minimax':
        from analyse import minimax as MM
        return MM.annahme(E2, E1)['besser']
    if not E1.get('note_v3_version') or E1.get('note_v3_version') != E2.get('note_v3_version'): return False
    a, b = tuple(E1['rang'][:3]), tuple(E2['rang'][:3])
    if a != b: return a < b
    return E1['note_v3'] < E2['note_v3']


def bild(B, R, K, prefix, E=None, top=None):
    from PIL import Image, ImageDraw, ImageFont
    N = K['netze']; GND = set(K['gnd'])
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0 = min(xs) - 1, min(ys) - 1; S = 40.0 if max(xs) - min(xs) < 60 else 12.0
    W, H = int((max(xs) - x0 + 1) * S), int((max(ys) - y0 + 1) * S)
    tr = lambda x, y: ((x - x0) * S, (y - y0) * S)
    FAR = {'sw': (200, 30, 30), 'strom': (220, 70, 70), 'versorgung': (230, 120, 120), 'schnell': (240, 140, 0),
           'analog': (30, 150, 60), 'signal': (60, 100, 220)}
    try: font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 13)
    except OSError: font = None
    um = {u[1] for u in (top or [])}
    lagen = sorted({t['layer'] for t in R['tracks']}, key=lambda L: (L != 'F', L == 'B', L))
    for Lg in lagen:
        im = Image.new('RGB', (W, H), 'white'); d = ImageDraw.Draw(im)
        for o in B['outline']: d.polygon([tr(*p) for p in o], outline=(0, 0, 0))
        for p in B['pads']:
            if Lg in p['polys']:
                for q in p['polys'][Lg]:
                    if len(q) >= 3: d.polygon([tr(*c) for c in q], fill=(200, 200, 200))
        for t in R['tracks']:
            if t['layer'] != Lg or t['net'] in GND: continue
            col = FAR.get(N.get(t['net'], {}).get('art', 'signal'), (90, 90, 90))
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if t['net'] in um: d.line([tr(*a), tr(*b)], fill=(160, 40, 200), width=max(3, int(w * S) + 6))
                d.line([tr(*a), tr(*b)], fill=col, width=max(1, int(w * S)))
        for v in R['vias']:
            cx, cy = tr(v['x'], v['y']); d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], outline=(0, 0, 0))
        if E:
            for r, mv in (E.get('verschiebungen') or {}).items():
                ps = [(p['x'], p['y']) for p in B['pads'] if p['ref'] == r]
                if not ps: continue
                cx = sum(p[0] for p in ps) / len(ps); cy = sum(p[1] for p in ps) / len(ps)
                a = tr(cx, cy); b = tr(cx + mv['dx'] * 6, cy + mv['dy'] * 6)
                d.line([a, b], fill=(0, 0, 0), width=3); d.ellipse([b[0] - 4, b[1] - 4, b[0] + 4, b[1] + 4], fill=(0, 0, 0))
                if font: d.text((b[0] + 5, b[1] - 6), r, fill=(0, 0, 0), font=font)
        if font: d.text((8, 6), 'Lage %s  rot Strom, orange schnell, gruen analog, blau Signal; violett: Umwege; Pfeile: Vorschlaege (x6)' % Lg,
                        fill=(0, 0, 0), font=font)
        im.save('%s_%s.png' % (prefix, Lg))
    return ['%s_%s.png' % (prefix, L) for L in lagen]


if __name__ == '__main__':
    B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3]))
    kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    if 'lagenaufbau' in kw:
        K = dict(K, vorgaben=dict(K.get('vorgaben') or {}, lagenaufbau=kw['lagenaufbau'] == '1'))
    V3 = kw.get('v3', '1') == '1'
    MM = kw.get('minimax') == '1' or kw.get('note') == 'minimax'
    E1 = bewerte(B, R, K, ir=kw.get('ir') == '1', v3=V3, minimax=True if MM else None, laengentor=kw.get('laengentor') == '1', si_note=kw.get('si_note') == '1', si_messen=kw.get('si_messen') == '1')
    print('%s | ' % ('GUELTIG' if E1['gueltig'] else 'UNGUELTIG (Note nicht mit gueltigen vergleichen)'), end='')
    if E1['unverbunden']: print('unverbunden: %s | ' % ', '.join(n.split('/')[-1] for n in E1['unverbunden'][:6]), end='')
    if not E1['diagnose']: print('Diagnosefelder fehlen | ', end='')
    print('Note %.1f | Konflikte %d | offen %d | Vias %d | gewichtet %.1f | Defizit Strom %.2f mm2 | Verengung %.2f mm2 | Umwege %.1f mm' % (
        E1['note'], E1['konflikte'], E1['offen'], E1['vias'], E1['gewichtet'], E1['defizit'], E1['verengung'], E1['umweg_summe']))
    for v in E1['verengungen'][:4]: print('  Flussverengung %-10s %.2f mm statt %.2f mm bei %s (%.2f mm2)' % (v[1].split('/')[-1], v[2], v[3], v[4], v[0]))
    if E1.get('note_v2') is not None:
        t = E1['note_v2_teile']['teile']
        print('note-v2 %.1f (Signal S %.1f, Verlust %.1f, Abfall %.1f, Schaltknoten Verlust %.1f, Schaltknoten Abfall %.1f; note-v1 %.1f)' % (
            E1['note_v2'], t['S'], t['verlust'], t['abfall'], t['sw_verlust'], t['sw_abfall'], E1['note']))
        for n, z in E1['note_v2_teile']['sw'].items():
            print('  Schaltknoten %-14s R %.1f mOhm, L %.2f nH, f %.2f MHz (%s), |Z| %.1f mOhm bei %.2f A' % (
                n.split('/')[-1], z['R_mOhm'], z['L_nH'], z['f_Hz'] / 1e6, z['f_quelle'], z['Z_mOhm'], z['I_A']))
    if E1.get('ir') and E1['ir'].get('verlust_mW') is not None:
        mx = sorted(E1['ir']['max_abfall_mV'].items(), key=lambda kv: -kv[1])[:3]
        print('Spannungsabfall (%s): Verluste %.1f mW, groesster Abfall %s' % (','.join(str(m) for m in E1['ir'].get('strommodus') or []) or '?', E1['ir']['verlust_mW'],
              ', '.join('%s %.1f mV' % (n.split('/')[-1], v) for n, v in mx)))
    if E1.get('note_v3') is not None:
        from analyse.metriken import tabelle
        print('note-v3 %.1f (Netze %.1f, %.2f s CPU); schlechteste Netze:' % (E1['note_v3'], E1['note_v3_netze'], E1['metriken'].get('cpu_s', E1['metriken']['zeit_s'])))
        print(tabelle(E1['metriken'], 10))
    if E1.get('minimax') is not None:
        e = E1['engpass'] or {}
        print('Minimax %.6g (%s); Engpass: %s / %s, x=%.3g; Summe v3.1 %.2f; sicher %s' %
              (E1['minimax'], E1['minimax_profil'], e.get('netz', '-'), e.get('kriterium', '-'), e.get('x', 0),
               E1['minimax_summe'], E1['sicherheit']['sicher']))
    print('Laenge je Art:', E1['laenge'])
    for u in E1['umwege'][:6]: print('  Umweg %-26s +%.1f mm (%.1f statt %.1f)' % (u[1].split('/')[-1], u[0], u[2], u[3]))
    if E1.get('strompfade'):
        sp = E1['strompfade']
        print('Strompfade: Umweg %.1f mm, Verlust %s (%s)' % (sp['umweg_mm'], '%.1f mW' % sp['verlust_mW'] if sp['verlust_mW'] is not None else '-', sp['quelle']))
    if 'vergleich' in kw:
        R2 = json.load(open(kw['vergleich']))
        E2 = bewerte(B, R2, K, ir=kw.get('ir') == '1', v3=V3, minimax=True if MM else None, laengentor=kw.get('laengentor') == '1', si_note=kw.get('si_note') == '1', si_messen=kw.get('si_messen') == '1')
        print('Vergleich (harte Tore zuerst): %s' % ('besser' if besser(E1, E2) else ('gleich' if E1['rang'] == E2['rang'] else 'schlechter')))
        for nt in (['v3'] if V3 else []) + (['v2'] if kw.get('ir') == '1' else []):
            vg = vergleich(E2, E1, B, R2, B, R, K, note=nt)
            print('Vergleich %s (gleich gerechnet): %s, %s%s' % (vg['note'], 'besser' if vg['besser'] else 'nicht besser', vg['grund'],
                  '; Strompfade Umweg %+.1f mm, Verlust %+.1f mW' % (vg['umweg_strom_mm'], vg['verlust_strom_mW']) if 'umweg_strom_mm' in vg else ''))
        print('Vergleich: Note %.1f -> %.1f (%+.1f), Vias %d -> %d, Defizit %.2f -> %.2f, Verengung %.2f -> %.2f, Umwege %.1f -> %.1f, Konflikte %d -> %d' % (
            E2['note'], E1['note'], E1['note'] - E2['note'], E2['vias'], E1['vias'], E2['defizit'], E1['defizit'],
            E2['verengung'], E1['verengung'], E2['umweg_summe'], E1['umweg_summe'], E2['konflikte'], E1['konflikte']))
    if 'json' in kw: json.dump(E1, open(kw['json'], 'w'), indent=1)
    if 'bild' in kw:
        E = json.load(open(kw['engstellen'])) if 'engstellen' in kw else None
        print('Bilder:', ' '.join(bild(B, R, K, kw['bild'], E, E1['umwege'][:8])))
