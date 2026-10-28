""
import json, os, shutil, sys, time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VPY = os.path.join(R, '.venv', 'bin', 'python')


def gueltig(B, K, rj, gndlagen, fluss_cache=None, routes=None, minimax=None):
    ""
    sys.path.insert(0, R)
    from analyse import korpus as KO
    RR = KO.normalisierte_route(B, K, routes if routes is not None else json.load(open(rj)), gndlagen) if routes is not None or (rj and os.path.exists(rj)) else None
    E = KO.bewerte_wer(B, K, rj, gndlagen=gndlagen, fluss_cache=fluss_cache, routes=RR) if RR is not None else None
    from analyse.metriken import review_aktiv
    if E and gndlagen and not review_aktiv(K):
        from analyse.bewertung import bewerte, unverbunden
        E['unverbunden'] = len(unverbunden(B, RR, set(K.get('gnd') or []), fluss_cache=fluss_cache, K=K))
        E['gueltig'] = bool(bewerte(B, RR, K, minimax=False, fluss_cache=fluss_cache)['gueltig']) and E['unverbunden'] == 0 and E['konflikte'] == 0 and E['offen'] == 0
    from analyse.standard import ziel
    if E and ziel(K.get('vorgaben'), explizit=('minimax' if minimax else 'v3') if minimax is not None else None) == 'minimax':
        from analyse.bewertung import bewerte
        EE = bewerte(B, RR if review_aktiv(K) else dict(RR, gndlagen=list(gndlagen)), K, v3=True, minimax=True, fluss_cache=fluss_cache)
        E['gueltig_routing'] = E['gueltig']
        E['gueltig'] = E['gueltig'] and EE['minimax_gueltig']
        E['minimax_status'] = EE['minimax_status']
        E['sicherheit'] = EE['sicherheit']
    if E and (K.get('vorgaben') or {}).get('physik_version') == 'v3.3-vertrag-option':
        from analyse.bewertung import bewerte
        EE = bewerte(B, dict(RR, gndlagen=list(gndlagen or [])), K, v3=True, fluss_cache=fluss_cache)
        from analyse.vertrag import zustand
        E.update(regeln=EE['regeln'], nachweise=EE['nachweise'], messversion=EE['tor'].get('messversion'), note_v3_version=EE['note_v3_version'])
        E['tor'] = zustand(E)
        E['vollstaendig_gueltig'] = E['tor']['vollstaendig_gueltig']
    return E['gueltig'] if E else False, E


def empfaengertor_an(K):
    ""
    return bool((K.get('vorgaben') or {}).get('empfaengertor')) or os.environ.get('EMPFAENGERTOR') == '1'


def formen(bj, kj, rj, regeln, gndlagen, stil='45r', log=None, zeit=120, strom_form=False, kupfer=False, minimax=None, ergebnis=None, fluss_cache=None, board=None, klassen=None, r27_formtor=False, r18=False, r18_abschluss=None):
    ""
    if r18:
        if stil != '45r': raise ValueError('r18=1 erfordert form=45r')
        start = time.thread_time()
        info = formen(bj, kj, rj, regeln, gndlagen, stil, log, zeit, strom_form,
                      kupfer, minimax, ergebnis, fluss_cache, board, klassen, r27_formtor)

        rest = max(0., zeit - info.get('cpu_s', 0.) - (time.thread_time()-start))
        gate = dict(angewendet=False, cpu_s=0., grund='Basisform nicht abgeschlossen')
        if info.get('basis_abgeschlossen') and info.get('glatt_sha256'):
            if rest < 2:
                gate['grund'] = 'R18: CPU-Budget erschoepft'
            else:
                from lernen.r18_form import anwenden

                try:
                    gate = anwenden(bj, kj, rj, regeln, gndlagen, cpu_s=rest/2,
                        roh_sha256=info['roh_sha256'], glatt_sha256=info['glatt_sha256'],
                        abschluss=r18_abschluss, minimax=minimax)
                except Exception as exc:


                    gate = dict(angewendet=False, abgeschlossen=False, cpu_s=0.,
                                grund='R18-Fehler: ' + repr(exc)[:300])
        info['r18'] = gate; info['cpu_s'] = info.get('cpu_s', 0.) + gate['cpu_s']
        if (gate.get('angewendet') or gate.get('abgeschlossen')) and ergebnis is not None:
            B = board if board is not None else json.load(open(bj))
            K = klassen if klassen is not None else json.load(open(kj))
            RR = json.load(open(rj)); g,e = gueltig(B,K,rj,gndlagen,routes=RR,minimax=minimax)
            ergebnis.update(gueltig=g,ergebnis=e,routes=RR)
        return info
    if stil not in (None, '', 'roh', '45r', 'organisch', 'ideal', 'je_netzart'):
        raise ValueError('unbekannter Formstil: '+str(stil))
    form_start = time.thread_time()
    def strom_abschluss(info):
        rest = max(0., zeit - info.get('cpu_s', 0.) - (time.thread_time()-form_start))
        if rest < 1:
            info['strom_form'] = dict(angewendet=False, grund='Stromform: CPU-Budget erschoepft')
        else:
            info['strom_form'] = strom_nachlauf(bj, kj, rj, regeln, minimax=minimax, cpu_s=rest)
            info['cpu_s'] = info.get('cpu_s', 0.) + info['strom_form'].get('rechenplatz', {}).get('cpu_s', 0.)
        return info
    if stil in (None, '', 'roh') or not os.path.exists(rj):
        info = dict(stil=stil, angewendet=False, grund='aus')
        if (strom_form or kupfer) and os.path.exists(rj):
            strom_abschluss(info)
            info['angewendet'] = info['strom_form']['angewendet']
            info['grund'] = info['strom_form'].get('grund', '')
        return info
    roh = rj[:-5] + '_roh.json'; aus = rj[:-5] + '_form.json'
    from pathlib import Path
    import hashlib
    glatt = rj[:-5] + '_glatt.json'
    for q in (roh, aus, glatt): Path(q).unlink(missing_ok=True)
    shutil.copyfile(rj, roh)
    roh_sha = hashlib.sha256(Path(roh).read_bytes()).hexdigest()
    glatt_sha = None

    opt = ['stil=ideal', 'topologie_version=topologie-v4-symmetrisch'] if stil == 'ideal' else (['stil=je_netzart'] if stil == 'je_netzart' else ['ziehen=0', 'rmax=0.5'] if stil == '45r' else ['rmax=3'])
    ein = roh
    if stil in ('45r', 'je_netzart'):
        try:
            from nachbearbeitung.glaetten45 import glaetten
            Bg, Rg = json.load(open(bj)), json.load(open(roh))
            sperren = None
            if stil == 'je_netzart':
                from gemeinsam.netzstil import organisch
                Kg = klassen if klassen is not None else json.load(open(kj))
                sperren = {t['net'] for t in Rg['tracks'] if organisch(t['net'], Kg)}
            Rg['glaetten45'] = glaetten(Bg, Rg, sperren=sperren)
            Rg['r18_roh_sha256'] = roh_sha
            ein = glatt; Path(ein).write_text(json.dumps(Rg))
            glatt_sha = hashlib.sha256(Path(ein).read_bytes()).hexdigest()
        except Exception as e:
            ein = roh; glatt_sha = None
            Path(glatt).unlink(missing_ok=True)
            if log: Path(log + '.glaetten').write_text(repr(e))
    cmd = [VPY, os.path.join(R, 'nachbearbeitung/pulltight_routes.py'), bj, ein, aus, 'regeln=' + regeln, 'klassen=' + kj] + opt
    try:
        from gemeinsam import plaetze as PL
        r = PL.lauf(cmd, cpu_s=zeit, log=log, cwd=R, env=dict(os.environ, PYTHONHASHSEED='0'))
    except OSError:
        return dict(stil=stil, angewendet=False, grund='zeit')
    if r['zustand'] != 'ok' or not os.path.exists(aus): return dict(stil=stil, angewendet=False, grund=r['zustand'], cpu_s=r['cpu_s'])
    B = board if board is not None else json.load(open(bj))
    K = klassen if klassen is not None else json.load(open(kj))
    R0, R1 = json.load(open(roh)), json.load(open(aus))
    try:
        from nachbearbeitung.glaetten45 import via_mitte
        R1['via_mitte'] = via_mitte(R1); json.dump(R1, open(aus, 'w'))
    except Exception as e:
        R1['via_mitte'] = dict(fehler=repr(e)[:160]); json.dump(R1, open(aus, 'w'))
    g0, e0 = gueltig(B, K, roh, gndlagen, fluss_cache=fluss_cache, routes=R0, minimax=minimax); g1, e1 = gueltig(B, K, aus, gndlagen, fluss_cache=fluss_cache, routes=R1, minimax=minimax)
    try: st = json.load(open(aus)).get('organisch_nachlauf', {}).get('statistik', {})
    except ValueError: st = {}
    from analyse.vertrag import vergleich
    an = bool(e0 and e1 and vergleich(e0,e1)['nicht_schlechter']) if (e0 or {}).get('tor') or (e1 or {}).get('tor') else g1 or not g0

    info = json.load(open(aus)).get('organisch_nachlauf', {})
    if stil == 'ideal' and not info.get('regeltor', {}).get('angenommen', False):
        an = False



    length_ok = True
    if empfaengertor_an(K):
        from analyse.laengenimpedanz import laengen, empfaenger_schutz, empfaenger_klassen
        length_k = empfaenger_klassen(K)
        length_ok = empfaenger_schutz(laengen(B, R0, length_k), laengen(B, R1, length_k))
    an = an and length_ok
    r27_gate = None
    if r27_formtor and an:
        gate_cpu = time.thread_time()
        try:
            from lernen.r27_form import kandidat
            repaired, r27_gate = kandidat(B, R0, R1, json.load(open(regeln)) if regeln else {})
            if r27_gate['vorgeschlagen']:
                from analyse.regelkennzahlen import r27_formtor as pruefe_r27
                r27_gate['regeltor'] = pruefe_r27(B, R1, repaired, K, gndlagen)
                g2, e2 = gueltig(B, K, None, gndlagen, fluss_cache=fluss_cache, routes=repaired, minimax=minimax)
                r27_gate['gueltig'] = g2
                receiver_ok = True
                if empfaengertor_an(K):
                    receiver_ok = empfaenger_schutz(laengen(B, R1, length_k), laengen(B, repaired, length_k))
                r27_gate['empfaengertor'] = receiver_ok
                contract = vergleich(e1, e2) if (e1 or {}).get('tor') or (e2 or {}).get('tor') else dict(nicht_schlechter=g2 or not g1)
                r27_gate['vergleich'] = contract
                r27_gate['angewendet'] = bool(contract['nicht_schlechter'] and receiver_ok and r27_gate['regeltor']['angenommen'])
        except Exception as exc:
            r27_gate = dict(version='r27-form-v2', vorgeschlagen=False, angewendet=False,
                            grund='R27-Messfehler', fehler=repr(exc)[:200])
        if r27_gate.get('angewendet'):
            json.dump(repaired, open(aus, 'w'))
            R1, e1, g1 = repaired, e2, g2
        r27_gate['cpu_s'] = time.thread_time()-gate_cpu
        r['cpu_s'] += r27_gate['cpu_s']
    if an: shutil.copyfile(aus, rj)
    if ergebnis is not None: ergebnis.update(gueltig=g1 if an else g0, ergebnis=e1 if an else e0, routes=R1 if an else R0)
    info = dict(stil=stil, angewendet=an, basis_abgeschlossen=True, roh_sha256=roh_sha, glatt_sha256=glatt_sha, gueltig_roh=g0, gueltig_form=g1, boegen=st.get('boegen'), geschuetzt=st.get('geschuetzt'),
                grund='' if an else ('Empfaengertor' if not length_ok else 'Regeltor' if stil == 'ideal' else 'Form waere ungueltig'), cpu_s=r['cpu_s'],
                empfaengertor=length_ok,
                form_nativ=info.get('form_nativ'), regeltor=info.get('regeltor'))
    if r27_gate is not None: info['r27_formtor'] = r27_gate
    if strom_form or kupfer: strom_abschluss(info)
    return info


def strom_nachlauf(bj, kj, rj, regeln, minimax=None, cpu_s=180):
    ""
    import tempfile
    from pathlib import Path
    from gemeinsam import plaetze as PL
    with tempfile.TemporaryDirectory(prefix='autorouter-form-worker-') as folder:
        result = Path(folder) / 'info.json'
        original = Path(rj).read_bytes()
        candidate = Path(folder) / 'routes.json'
        candidate.write_bytes(original)
        cmd = [VPY, '-m', 'lernen.form', bj, kj, str(candidate), regeln or '', str(result),
               'auto' if minimax is None else str(int(minimax))]
        run = PL.lauf(cmd, cpu_s=cpu_s, wand_s=10*cpu_s, cwd=R,
                      log=str(Path(folder)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['zustand'] == 'ok' and result.exists() else dict(
            angewendet=False, grund='Kupfer-Nachlauf: ' + run['zustand'])
        info['rechenplatz'] = run
        if info['angewendet']:
            if Path(rj).read_bytes() != original:
                info.update(angewendet=False, grund='Route waehrend Nachlauf geaendert')
                return info
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.strom_form_', delete=False) as f:
                tmp = f.name
                f.write(candidate.read_bytes())
            try:
                Path(rj[:-5] + '_vor_kupfer.json').write_bytes(original)
                os.replace(tmp, rj)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        return info


def _strom_nachlauf(bj, kj, rj, regeln, minimax=None):
    ""
    import time
    import math
    from lernen.kupferbalken import erweitern, pruefe_kanten
    from analyse.bewertung import bewerte
    from analyse.regelkennzahlen import torfelder, annahme
    from pathlib import Path
    B, K, Rr = (json.loads(Path(q).read_text()) for q in (bj, kj, rj))
    RG = json.loads(Path(regeln).read_text()) if regeln else None



    prepared = dict(Rr, tracks=[])
    for t in Rr['tracks']:
        items = t.get('items')
        same = (items and len(items) == len(t['w']) and len(t['pts']) == len(items) + 1 and
                all(it[0] == 'L' and it[1] == t['pts'][j] and it[2] == t['pts'][j + 1] and
                    it[3] == t['w'][j] for j, it in enumerate(items)))
        if same and K['netze'].get(t['net'], {}).get('art') in ('strom', 'versorgung', 'sw') and \
                not any(t.get(q) for q in ('sig', 'kelvin', 'fix', 'strom_steg')):
            t = {q: v for q, v in t.items() if q != 'items'}
        prepared['tracks'].append(t)
    out, info = erweitern(B, prepared, K, RG, cpu_s=8.0)
    if not info['angewendet']:
        return info
    t0 = time.process_time()
    import resource
    child = resource.getrusage(resource.RUSAGE_CHILDREN)
    child0 = child.ru_utime + child.ru_stime
    def auswertung_cpu():
        child = resource.getrusage(resource.RUSAGE_CHILDREN)
        return time.process_time()-t0+child.ru_utime+child.ru_stime-child0
    from analyse.kupfermodell import masse_korridore


    from lernen.kupfer_nativ import pruefen
    import tempfile
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter-stromform-') as native_dir:
            baseline, out, native_info = pruefen(native_dir, B, Rr, out, RG)
        info['nativ'] = native_info
    except Exception as exc:
        info.update(angewendet=False, grund='Native Pruefung fehlgeschlagen: ' + str(exc),
                    bewertung_cpu_s=auswertung_cpu())
        return info
    if not native_info['vorschlag']['gueltig']:
        info.update(angewendet=False, grund='Native DRC/Neufuellung sperrt Uebernahme',
                    bewertung_cpu_s=auswertung_cpu())
        return info
    info['massekorridore'] = masse_korridore(B, baseline, out, K, RG)
    audit = pruefe_kanten(B, out, K, voll=True)
    info['breitenpruefung'] = audit
    if audit['stufen'] or audit['harte_ecken'] or audit.get('pads_nicht_umschlossen',0) or audit.get('fehlende_stromflaechen',0) or not info['massekorridore']['bestanden']:
        info.update(angewendet=False, grund='Natives Kantentor/Massekorridor nicht bestanden',
                    bewertung_cpu_s=auswertung_cpu())
        return info
    if minimax is None:
        from analyse.standard import ziel
        minimax = ziel(K.get('vorgaben')) == 'minimax'
    e0, e1 = bewerte(B, baseline, K, v3=True, minimax=minimax), bewerte(B, out, K, v3=True, minimax=minimax)
    keys = tuple(dict.fromkeys(torfelder(e0['regeln']) + torfelder(e1['regeln'])))
    missing = [q for q in keys if not all(isinstance(e['regeln'].get(q), (int, float)) and
               math.isfinite(e['regeln'][q]) for e in (e0, e1))]
    worse = [q for q in keys if q not in missing and e1['regeln'][q] > e0['regeln'][q] + 1e-8]
    contract = annahme(e0['regeln'], e1['regeln'])
    if minimax:
        from analyse.minimax import annahme as minimax_annahme
        contract = minimax_annahme(e0, e1)
    info.update(bewertungsziel='minimax' if minimax else 'v3', annahmetor=contract)
    good = contract['nicht_schlechter'] and e1['gueltig'] and not missing and not worse and not e0['ir']['fehler_netze'] and not e1['ir']['fehler_netze'] and audit['stufen'] == 0 and audit['harte_ecken'] == 0
    info.update(angewendet=good, tor_schlechter=worse, tor_fehlt=missing, gueltig_vorher=e0['gueltig'], gueltig_nachher=e1['gueltig'],
                note_v3_vorher=e0['note_v3_netze'], note_v3_nachher=e1['note_v3_netze'], bewertung_cpu_s=auswertung_cpu(),
                ir_fehler_vorher=e0['ir']['fehler_netze'], ir_fehler_nachher=e1['ir']['fehler_netze'])
    if good:
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=Path(rj).parent, prefix='.strom_form_', delete=False) as f:
            tmp = f.name
            json.dump(out, f, ensure_ascii=False)
        try:
            shutil.copyfile(rj, rj[:-5] + '_vor_kupfer.json')
            os.replace(tmp, rj)
        finally:
            if os.path.exists(tmp): os.unlink(tmp)
    else:
        info['grund'] = 'Tor nicht bestanden: keine Uebernahme'
    return info


balken_nachlauf = strom_nachlauf


def abschliessen(bj, kj, rj, regeln, gndlagen, cfg, mode, r36, cpu=None):
    ""
    route_gndlagen = gndlagen
    z = {}
    if cpu is not None and cpu.rest < 1:
        raise TimeoutError('Abschluss: CPU-Budget erschoepft')
    try:
        from lernen.zonenplaner import abschluss
        _rv = json.load(open(rj)) if os.path.exists(rj) else {}
        if _rv.get('conflicts') or _rv.get('failed'):


            z['stromzonen'] = dict(angewendet=False, aus=True, native_abnahme=False, grund='router_ungueltig')
        elif r36 == '0' and mode != 'lokal':
            z['stromzonen'] = dict(angewendet=False,aus=True,native_abnahme=False)
        else:
            z['stromzonen'] = abschluss(bj, kj, rj, regeln,
                route_gndlagen, modus='lokal' if mode == 'lokal' else r36, cpu_s=720 if cpu is None else max(1,int(cpu.rest)))
            if cpu: cpu.add(z['stromzonen'].get('rechenplatz',{}).get('cpu_s',0))
    except Exception as e: z['stromzonen'] = dict(angewendet=False,native_abnahme=False,fehler=repr(e)[:160])
    if cfg.get('k20_zonen'):
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Abschluss: CPU-Budget erschoepft')
        from lernen.k20_nachlauf import anwenden as k20_anwenden
        z['k20_zonen'] = k20_anwenden(bj, kj, rj,
                                    regeln, route_gndlagen,
                                    cpu_s=720 if cpu is None else int(cpu.rest), zonenplan=mode)
        if cpu: cpu.add(z['k20_zonen'].get('rechenplatz', {}).get('cpu_s', 0))
        if z['k20_zonen'].get('angewendet'):
            from lernen.k20_nachlauf import abnahme_info
            z['stromzonen_standard'] = z.get('stromzonen')
            z['stromzonen'] = abnahme_info(z.get('stromzonen'), z['k20_zonen'])
    if cfg.get('r39_reparatur'):
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Abschluss: CPU-Budget erschoepft')
        from lernen.r39_nachlauf import anwenden as r39_anwenden
        z['r39_reparatur'] = r39_anwenden(bj, kj, rj, route_gndlagen,
                                         cpu_s=240 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r39_reparatur']['rechenplatz']['cpu_s'])
        if z['r39_reparatur'].get('angewendet'):
            z['stromzonen_vor_r39'] = z.get('stromzonen')
            z['stromzonen'] = dict(angewendet=True, native_abnahme=False, ersatz='geometrisch',
                                   grund='R39-Geometrie ersetzt native Abnahme')
    if cfg.get('r11_ir'):
        if cpu is not None and cpu.rest < 1:
            z['r11_ir'] = dict(angewendet=False, grund='zeitlimit',
                               rechenplatz=dict(cpu_s=0.0, zustand='nicht_gestartet'))
            raise TimeoutError('Abschluss: CPU-Budget erschoepft')
        from lernen.r11_nachlauf import anwenden as r11_anwenden
        z['r11_ir'] = r11_anwenden(bj, kj, rj,
                                  regeln,
                                  cpu_s=240 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r11_ir']['rechenplatz']['cpu_s'])
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Abschluss: CPU-Budget erschoepft')
    if cfg.get('r24'):
        from lernen.r24_nachlauf import anwenden as r24_anwenden
        if cpu is not None and cpu.rest < 1:
            raise TimeoutError('Abschluss: CPU-Budget erschoepft')
        z['r24'] = r24_anwenden(bj, kj, rj,
                               regeln,
                               cpu_s=180 if cpu is None else max(1, int(cpu.rest)))
        if cpu: cpu.add(z['r24']['rechenplatz']['cpu_s'])
    return z


if __name__ == '__main__':
    bj, kj, rj, rg, result, mode = sys.argv[1:]
    info = _strom_nachlauf(bj, kj, rj, rg or None, minimax=None if mode == 'auto' else mode == '1')
    from pathlib import Path
    Path(result).write_text(json.dumps(info, ensure_ascii=False))
