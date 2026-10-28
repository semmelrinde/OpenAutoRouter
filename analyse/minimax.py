""
import collections
import hashlib
import heapq
import json
import math
import re

VERSION = 'minimax-v4.1-kupfervertrag'


def _zahl(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x) and x >= 0


def lastvertrag(n, K, I, quelle, ir=None):
    ""
    k = K['netze'][n]; ir = ir or {}
    v = dict(I_A=I, I_modus=quelle, strom_A=k.get('strom_A'), strom_A_quelle=k.get('strom_A_quelle'),
             fluss=k.get('fluss'), art=k.get('art'),
             ir_strom_A=(ir.get('strom_je_netz_A') or {}).get(n),
             ir_modus=(ir.get('strommodus_je_netz') or {}).get(n),
             ir_ohne_last=(ir.get('ohne_last') or {}).get(n), ir_zonen_modell=ir.get('zonen_modell'))
    return hashlib.sha256(json.dumps(v, sort_keys=True, allow_nan=False).encode()).hexdigest()


def lastfehler(k):
    ""
    fl = k.get('fluss') or {}
    last = fl.get('last') or {}
    if not isinstance(last, dict) or any(not _zahl(q) for q in last.values()): return 'Lastverteilung ungueltig'
    if fl.get('einheit') == 'A':
        i = fl.get('gesamt')
        if not _zahl(i): return 'fluss.gesamt in A fehlt/ungueltig'
        if i == 0 and (any(q > 0 for q in last.values()) or (k.get('strom_A') or 0) > 0):
            return 'Nulllast widerspricht positiver Last'


        if last and abs(sum(last.values()) - i) > (len(last) + 1) * 5e-8 + 1e-12:
            return 'Lastsumme passt nicht zu gesamt'
    elif not _zahl(k.get('strom_A')): return 'strom_A fehlt/ungueltig (Ersatz 1A)'
    return None

def prioritaeten(vg, netze):
    raw = vg.get('prioritaet_netze') or {}
    if isinstance(raw, list): raw = {n: 2.0 for n in raw}
    if not isinstance(raw, dict): raise ValueError('prioritaet_netze muss Liste oder Gewichte sein')
    if any(not _zahl(q) or q < 1 for q in raw.values()): raise ValueError('Netzprioritaeten muessen endlich und >= 1 sein')
    return {n: float(raw.get(n, 1)) for n in sorted(netze)}


def _vorgabe(v, n):
    for rx, x in sorted((v or {}).items()):
        if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, n.split('/')[-1]): return float(x)
    return None


def _pfadlaenge(tracks, vias, pads, A):
    ""
    from gemeinsam.fluss import netzgraph
    if len(pads) != 2: return None
    g = netzgraph(tracks, vias, pads, 0.03)
    s, z = [('PAD', p['ref'], p['num']) for p in pads]
    best = {s: 0.0}; h = [(0.0, s)]
    while h:
        d, k = heapq.heappop(h)
        if k == z: return d
        if d > best[k]: continue
        for q, kid in g['adj'].get(k, []):
            l = 0.0
            if kid and kid[0] == 's':
                t = tracks[kid[1]]; l = math.dist(t['pts'][kid[2]], t['pts'][kid[2] + 1]) * (kid[4] - kid[3])
            elif kid and kid[0] == 'v': l = abs(A['lagen'][k[0]]['z'] - A['lagen'][q[0]]['z'])
            nd = d + l
            if nd < best.get(q, math.inf): best[q] = nd; heapq.heappush(h, (nd, q))
    return None


def _formlinie(t):
    ""
    if len(t['pts']) < 2: return t
    pts = [t['pts'][0], t['pts'][1]]; ws = [t['w'][0]]
    for q, w in zip(t['pts'][2:], t['w'][1:]):
        a, b = pts[-2:]; u = (b[0]-a[0], b[1]-a[1]); v = (q[0]-b[0], q[1]-b[1])
        if abs(w-ws[-1]) < 1e-12 and abs(u[0]*v[1]-u[1]*v[0]) < 1e-10 and u[0]*v[0]+u[1]*v[1] > 0:
            pts[-1] = q
        else: pts.append(q); ws.append(w)
    return dict(t, pts=pts, w=ws)


def _formketten(tracks):
    ""
    segs = []; adj = collections.defaultdict(list)
    def key(t, p): return (t['net'], t['layer'], round(p[0], 9), round(p[1], 9))
    for t in tracks:
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) < 1e-12: continue
            i = len(segs); segs.append((t, a, b, w))
            adj[key(t, a)].append(i); adj[key(t, b)].append(i)
    used = set(); out = []
    def follow(i, a):
        t, x, y, w = segs[i]; b = y if key(t, x) == key(t, a) else x
        used.add(i); pts = [a, b]; ws = [w]
        while len(adj[key(t, b)]) == 2:
            j = next((j for j in adj[key(t, b)] if j not in used), None)
            if j is None: break
            tj, xj, yj, wj = segs[j]
            if abs(wj - ws[-1]) > 1e-12: break
            q = yj if key(tj, xj) == key(t, b) else xj
            used.add(j); pts.append(q); ws.append(wj); b = q

        b = a
        while len(adj[key(t, b)]) == 2:
            j = next((j for j in adj[key(t, b)] if j not in used), None)
            if j is None: break
            tj, xj, yj, wj = segs[j]
            if abs(wj - ws[0]) > 1e-12: break
            q = yj if key(tj, xj) == key(t, b) else xj
            used.add(j); pts.insert(0, q); ws.insert(0, wj); b = q
        return _formlinie(dict(net=t['net'], layer=t['layer'], pts=pts, w=ws))

    for i, (t, a, b, w) in enumerate(segs):
        if i in used: continue
        if len(adj[key(t, a)]) != 2: out.append(follow(i, a))
        elif len(adj[key(t, b)]) != 2: out.append(follow(i, b))
    for i, (t, a, b, w) in enumerate(segs):
        if i not in used: out.append(follow(i, a))
    return out

def messen_zusatz(M, B, R, K, G, ir=None):
    ""
    from analyse.kennzahlen import kennzahlen
    from analyse.metriken import aufbau, leitung, breite_ipc, strombaum, stromdichte, VIA_HUELSE, T_INNEN
    cfg = G['minimax']; S = cfg['schwellen']; VG = K.get('vorgaben') or {}
    A = aufbau(len(M['aufbau']['stapel']), M['aufbau']['ebenen'],
               diel=M['aufbau']['dielektrika'], er=float((VG.get('aufbau') or {}).get('er', 4.1)))
    tracks = collections.defaultdict(list); vias = collections.defaultdict(list); pads = collections.defaultdict(list)
    for t in R['tracks']: tracks[t['net']].append(t)
    for v in R['vias']: vias[v['net']].append(v)
    for p in B['pads']: pads[p['net']].append(p)
    fehlend = []; gruppen = collections.defaultdict(list)
    from analyse.strom_kette import kette
    from gemeinsam.zonen import vereinheitlichen
    chain = kette(B, vereinheitlichen(R), K, ir=ir)
    M['strom_kette'] = chain
    M['minimax_prioritaeten'] = prioritaeten(VG, M['netze'])
    budgetvertrag = {}
    drill = float(str((R.get('optionen') or {}).get('via') or (K.get('regeln') or {}).get('via') or '0.45/0.20').split('/')[-1])
    for n, v in sorted(M['netze'].items()):
        m = v['m']; nk = K['netze'].get(n) or {}; x = {}
        lf = lastfehler(nk) if m.get('traegt') else None
        if lf: fehlend.append(n + ': ' + lf)
        if nk.get('art') in ('strom', 'versorgung') and m.get('traegt') and not lf and m['sicherheit_roh']['I_A'] > 0:
            q = chain['netze'].get(n)
            if q:
                m['strom_kette'] = q
                roh = q['roh']
                eta, glied = roh['wirkungsgrad'], roh['glied']
                if _zahl(eta) and eta > 0: x['kette_eta'] = 1 / eta / S['kette_ineffizienz']
                else: fehlend.append(n + ': Kettenwirkungsgrad fehlt')
                if _zahl(glied): x['kette_glied'] = glied / S['kette_glied']
                else: fehlend.append(n + ': Kettenstromdichte fehlt')


                area = _vorgabe(VG.get('kupfer_budget_mm2'), n)
                if area is None:
                    dv = m['sicherheit_roh'].get('s_dc_mV')
                    l = roh['l_eff_mm']; i = roh['strom_A']
                    area = max(nk.get('wmin', .1) * l, (1.72e-5 / .035) * i * l*l * 1000 / dv) if dv else None
                if _zahl(area) and area > 0: x['kette_kupfer'] = roh['kupfer_mm2'] / area / S['kette_kupfer']
                else: fehlend.append(n + ': Kupferbudget fehlt')
            else: fehlend.append(n + ': Stromkette nicht geloest (Last/Quelle/Kupfer)')
        budgetvertrag[n] = dict(wmin_mm=nk.get('wmin', .1), abfall_mV=m.get('sicherheit_roh', {}).get('s_dc_mV'),
             verlust_mW=_vorgabe(VG.get('verlust_budget_mW'), n),
             kupfer_mm2=_vorgabe(VG.get('kupfer_budget_mm2'), n),
             z_ohm=nk.get('z_ohm'), z_diff_ohm=nk.get('z_diff_ohm'),
             z_toleranz_rel=nk.get('z_toleranz_rel', S['z_toleranz_rel']),
             laengengruppe=nk.get('laengengruppe'), toleranz_mm=nk.get('toleranz_mm'))
        if not A['ebenen'] and (v['klasse'].startswith('schnell') or v['klasse'] == 'analog'):
            fehlend.append(n + ': Rueckweg/Johnson ohne Bezugsflaeche nicht gemessen')
        if m.get('traegt') and (m.get('sicherheit_roh', {}).get('I_A', m.get('I_A')) or 0) > 0:

            tl = _formketten([t for t in tracks[n] if not t.get('sig') and t['layer'] in A['lagen']])
            baum = strombaum(tl, vias[n], pads[n], nk.get('fluss'), A, drill,
                             m['sicherheit_roh']['I_A']) if tl else None
            j = stromdichte(baum['seg'], A, n, pads[n])[0] if baum else None
            m['sicherheit_roh']['J_x'] = j
            if j is not None: x['j'] = j
            sb = _vorgabe(VG.get('verlust_budget_mW'), n)
            if sb is None: sb = m['I_A'] * (m.get('s_dc_mV') or 0)
            if not sb or sb <= 0: fehlend.append(n + ': Verlustbudget fehlt')
            elif m.get('P_mW') is not None: x['verlust'] = m['P_mW'] / sb
            else: fehlend.append(n + ': Verlustmessung fehlt')

            vi = m.get('via_I_A')
            if vias[n]:
                stroeme = vi if vi is not None else [m['sicherheit_roh']['I_A']] * len(vias[n])
                barrel_w = math.pi * (drill + VIA_HUELSE) * VIA_HUELSE / T_INNEN
                m['J_via_x'] = max((breite_ipc(i, True) / barrel_w for i in stroeme), default=0.0)
            else: m['J_via_x'] = 0.0
        if tracks[n]:

            logisch = []; boegen = 0
            for t in tracks[n]:
                if 'items' not in t: logisch.append(_formlinie(t)); continue
                for it in t['items']:
                    if it[0] == 'A': boegen += 1
                    elif it[0] == 'L': logisch.append(dict(net=n, layer=t['layer'], pts=[it[1], it[2]], w=[it[3]]))
            f = kennzahlen(dict(tracks=_formketten(logisch), vias=vias[n]))
            f['segmente'] += boegen
            if boegen:


                fehlend.append(n + ': native Bogengeometrie fuer Form/Anschluesse nicht vollstaendig gemessen')
            x['form_mini'] = f['mini'] / max(1, f['segmente']) / S['mini_anteil']
            x['form_knicke'] = f['knicke'] / max(1.0, m['laenge_mm']) / S['knicke_pro_mm']
            x['form_stoesse'] = (f['fast_stoesse'] + f['vias_fast']) / S['fast_stoesse']
        z = nk.get('z_ohm')
        if z:
            tol = float(nk.get('z_toleranz_rel') or S['z_toleranz_rel'])
            zz = [leitung(A, t['layer'], w)[0] for t in tracks[n] if t['layer'] in A['lagen'] for w in t['w']]
            if zz and all(q > 0 for q in zz): x['impedanz'] = max(abs(q / z - 1) / tol for q in zz)
            else: fehlend.append(n + ': Impedanz ohne Bezug/Leitung')
        if nk.get('z_diff_ohm'): fehlend.append(n + ': differentielle Impedanz nicht gemessen')
        gr = nk.get('laengengruppe')
        if gr: gruppen[gr].append((n, nk.get('toleranz_mm')))
        m['minimax_x'] = x
    for gr, gg in sorted((VG.get('laengengruppen') or {}).items()):
        gruppen[gr] = [(n, gg.get('toleranz_mm')) for n in gg['netze'] if n in M['netze']]
    for gr, nn in sorted(gruppen.items()):
        ll = {n: _pfadlaenge(tracks[n], vias[n], pads[n], A) for n, _ in nn}
        if len(nn) < 2 or any(l is None for l in ll.values()):
            fehlend.append(gr + ': Laengenabgleich braucht mindestens zwei Zweipunktnetze'); continue
        delta = max(ll.values()) - min(ll.values())
        for n, tol in nn:
            if not tol or tol <= 0: fehlend.append(n + ': Laengentoleranz fehlt'); continue
            M['netze'][n]['m']['minimax_x']['laengenfehler'] = delta / tol
    M['minimax_boardvertrag'] = dict(netze=budgetvertrag, laengengruppen=VG.get('laengengruppen'),
                                        kanal_budget_mV={g:v.get('mV') for g,v in M.get('gruppen', {}).items()}, prioritaeten=M['minimax_prioritaeten'])
    M['minimax_normvertrag'] = normvertrag(G)
    M['minimax_fehlend'] = fehlend
    return M


def sicherheit(M):
    ""
    fehler = []; fehlend = []
    def pruefen(n, key, wert, grenze):
        if not _zahl(wert) or not _zahl(grenze) or grenze <= 0:
            fehlend.append(dict(netz=n, kriterium=key, grund='Messung/Grenze fehlt oder ungueltig'))
        elif wert > grenze + 1e-9:
            fehler.append(dict(netz=n, kriterium=key, wert=wert, grenze=grenze))
    for n, v in sorted(M['netze'].items()):
        m = v['m']; roh = m.get('sicherheit_roh') or m
        if not m.get('traegt'): continue
        strom = roh.get('I_A', m.get('I_A'))
        if not _zahl(strom):
            fehlend.append(dict(netz=n, kriterium='strom', grund='I_A fehlt oder ungueltig')); continue
        if strom == 0: continue
        for key, wert, grenze in [('strom_waerme', roh.get('J_x'), 1.0), ('via_waerme', m.get('J_via_x'), 1.0),
                                   ('abfall', roh.get('dV_mV'), roh.get('s_dc_mV'))]:
            pruefen(n, key, wert, grenze)
    for gr, v in sorted((M.get('gruppen') or {}).items()):

        pruefen('kanal:' + gr, 'abfall', v.get('summe_roh_mV', v.get('summe_mV')), v.get('mV'))
    return sicherheitsstatus(dict(fehler=fehler, fehlend=fehlend,
                                  modell='IPC-2221/10K, Flussbaum, Via-Kupferquerschnitt'))


def sicherheitsstatus(s):
    s['status'] = 'unsicher' if s['fehler'] else ('unbekannt' if s.get('fehlend') else 'sicher')
    s['sicher'] = s['status'] == 'sicher'
    return s

def bewerten(M, G, profil='ausgewogen'):
    from analyse.metriken import netz_x
    if M.get('minimax_normvertrag') and M['minimax_normvertrag'] != normvertrag(G):
        raise ValueError('Minimax-Schwellen/Anstiegszeiten geaendert: neu messen')
    if profil not in G['profile']: raise ValueError('Unbekanntes Profil: ' + str(profil))
    w = G['profile'][profil]['gewichte']; teile = []
    for n, v in sorted(M['netze'].items()):
        x = netz_x(v['m'], G['klassen'].get(v['klasse']) or {}, v['klasse'] == 'sw')
        if not v['klasse'].startswith('schnell'): x.pop('vias', None)

        m = v['m']
        if m.get('traegt') and m.get('s_dc_mV'): x['dc'] = m['dV_mV'] / m['s_dc_mV']
        x.update(m.get('minimax_x') or {})
        for k, q in sorted(x.items()):
            if not _zahl(q): raise ValueError('Ungueltige Minimax-Messung: %s %s' % (n, k))
            gewicht = float(w.get(k, 1.0)) * (M.get('minimax_prioritaeten') or {}).get(n, 1)
            if not math.isfinite(gewicht) or gewicht <= 0: raise ValueError('Minimax-Gewicht muss positiv sein: ' + k)
            teile.append(dict(netz=n, kriterium=k, x=q, gewicht=gewicht, wert=gewicht * max(0.0, q - 1.0)))
    for gr, v in sorted((M.get('gruppen') or {}).items()):
        if v.get('mV') and v['mV'] > 0:
            x = v['summe_mV'] / v['mV']; g = float(w.get('dc', 1.0)) * max(((M.get('minimax_prioritaeten') or {}).get(n, 1) for n in v.get('netze', [])), default=1)
            teile.append(dict(netz='kanal:' + gr, kriterium='dc', x=x, gewicht=g, wert=g * max(0.0, x - 1)))
    teile.sort(key=lambda t: (-t['wert'], t['netz'], t['kriterium']))

    version = versionsvertrag(G, profil, M.get('version'), M.get('minimax_boardvertrag'))
    sicher = sicherheit(M)
    M.update(minimax=max((t['wert'] for t in teile), default=0.0), minimax_version=version,
             minimax_profil=profil, minimax_rang=[t['wert'] for t in teile], minimax_teile=teile,
             engpass=max(teile, key=lambda t: (t['wert'], t['gewicht'] * t['x'])) if teile else None, minimax_summe=M['note_v3'], sicherheit=sicher,
             minimax_vollstaendig=not M.get('minimax_fehlend') and not sicher['fehlend'],
             minimax_quellen={n: dict(last=v['m'].get('lastvertrag'), messung=v['m'].get('dV_quelle'))
                             for n, v in sorted(M['netze'].items()) if v['m'].get('traegt')})
    return M

def normvertrag(G):
    norm = dict(schwellen=G['minimax']['schwellen'],
                klassen={k: dict(s=v.get('s'), tr_ns=v.get('tr_ns')) for k, v in G['klassen'].items()})
    return hashlib.sha256(json.dumps(norm, sort_keys=True, allow_nan=False).encode()).hexdigest()


def versionsvertrag(G, profil, messversion='v3.1', boardvertrag=None):
    vertrag = dict(version=VERSION, board=boardvertrag, messversion=messversion, config=G['minimax'], gewichte=G['profile'][profil]['gewichte'], klassen=G['klassen'], summe={k: G.get(k) for k in ('knick', 'knick_je', 'w_rang', 'gruppen')})
    digest = hashlib.sha256(json.dumps(vertrag, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
    return VERSION + ':' + digest


def aktuell(e):
    from analyse.metriken import lade_gewichte
    p = e.get('minimax_profil')
    G = lade_gewichte(si=e.get('note_v3_version', e.get('version')) == 'v3.2')
    return p in G.get('profile', {}) and e.get('minimax_version') == versionsvertrag(G, p, G.get('version', 'v3.1'), e.get('minimax_boardvertrag'))


def rang(E):
    ""
    v = list(E['minimax_rang'])
    while v and v[-1] == 0: v.pop()
    return tuple(v), E['minimax_summe']


def vollstaendig(e):
    ""
    rr = e.get('minimax_rang')
    return bool(e.get('minimax_vollstaendig') and isinstance(rr, list) and
                all(_zahl(v) for v in rr) and _zahl(e.get('minimax_summe')) and
                _zahl(e.get('minimax')) and isinstance(e.get('minimax_quellen'), dict) and
                all(isinstance(v, dict) and isinstance(v.get('last'), str) and re.fullmatch('[0-9a-f]{64}', v['last'])
                    and isinstance(v.get('messung'), str) for v in e['minimax_quellen'].values()))


def vergleich(a, b):
    if not a.get('minimax_version') or a.get('minimax_version') != b.get('minimax_version') or a.get('minimax_profil') != b.get('minimax_profil'):
        return dict(besser=False, delta=None, grund='verschiedene/fehlende Minimax-Versionen oder Profile')
    if 'minimax_quellen' not in a or 'minimax_quellen' not in b:
        return dict(besser=False, delta=None, grund='Minimax-Strommodelle/IR-Quellen fehlen')
    if a.get('minimax_quellen') != b.get('minimax_quellen'):
        return dict(besser=False, delta=None, grund='verschiedene Minimax-Strommodelle/IR-Quellen')
    if not vollstaendig(a) or not vollstaendig(b):
        return dict(besser=False, delta=None, grund='Minimax-Messgroessen fehlen')

    ar, br = a['minimax_rang'], b['minimax_rang']
    for i in range(max(len(ar), len(br))):
        x, y = (ar[i] if i < len(ar) else 0), (br[i] if i < len(br) else 0)
        if x != y: return dict(besser=y < x, delta=y-x, rangstelle=i, grund='Minimax Rangstelle %d: %g -> %g' % (i+1, x, y))
    d = b['minimax_summe'] - a['minimax_summe']
    return dict(besser=d < 0, delta=d, rangstelle=None, grund='Minimax gleich, Summe %+.3f' % d)


def annahme(a, b, tol_umweg_mm=1.0, tol_verlust=0.02):
    ""
    from analyse import regelkennzahlen as RK
    c = vergleich(a, b)
    out = dict(c, nicht_schlechter=False)
    if c['delta'] is None: return out
    if not (aktuell(a) and aktuell(b)):
        return dict(out, besser=False, grund='veraltete Minimax-Version')
    ga = a.get('minimax_gueltig', (a.get('sicherheit') or {}).get('sicher'))
    gb = b.get('minimax_gueltig', (b.get('sicherheit') or {}).get('sicher'))
    if not gb:
        return dict(out, besser=False, grund='Sicherheitstor nicht bestanden')
    tor = RK.annahme(a.get('regeln') or a.get('regeltor'), b.get('regeln') or b.get('regeltor'))
    out['regeltor'] = tor
    if not tor['nicht_schlechter']: return dict(out, besser=False, grund=tor['grund'])
    sa, sb = a.get('minimax_strompfade') or {}, b.get('minimax_strompfade') or {}
    if not all(_zahl(s.get(k)) for s in (sa, sb) for k in ('umweg_mm', 'verlust_mW')):
        return dict(out, besser=False, grund='Minimax-Strompfadschutz fehlt')
    du = sb['umweg_mm'] - sa['umweg_mm']; dp = sb['verlust_mW'] - sa['verlust_mW']
    schlechter = du > tol_umweg_mm and dp > tol_verlust * max(1e-9, sa['verlust_mW'])
    out.update(umweg_strom_mm=du, verlust_strom_mW=dp, strompfade_schlechter=schlechter)
    if schlechter: return dict(out, besser=False, grund='Strompfade verschlechtert')
    out['nicht_schlechter'] = not ga or c['delta'] <= 0
    out['besser'] = not ga or c['besser']
    return out


def aus_zeile(z):
    r = z.get('regel') or {}
    return r if 'minimax_version' in r else z


def belohnung(z):
    ""
    e = aus_zeile(z)
    if not aktuell(e) or not vollstaendig(e): return None
    if z.get('gueltig') is None or e.get('minimax_status') == 'unbekannt': return None
    if not z.get('gueltig') or not e.get('sicherheit', {}).get('sicher'): return 2.0
    v = e.get('minimax')
    return v / (1.0 + v) if _zahl(v) else None
