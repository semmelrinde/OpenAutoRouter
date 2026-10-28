""
import copy, hashlib, json, math, os, random, re, shutil, sys, threading, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
from gemeinsam.prozessschutz import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from lernen import aufgaben as AU
from lernen import lehrplan_tempo as TT
from lernen import placer_lockern as PLK
from lernen import fruehabbruch as FR
from lernen import meister as ME
from analyse import minimax as MM
from analyse.standard import ziel as standardziel
from gemeinsam import plaetze as PL
from analyse import vertrag as VT
VPY = os.path.join(R, '.venv', 'bin', 'python')
LOCK = threading.Lock()
PAAR_DIENSTE = {}
PAAR_LOCK = threading.Lock()


def lernkosten_fruehabbruch(cfg, rnd):
    ""
    if cfg.get('budget') != 'arbeit' and 'fruehabbruch' not in cfg and rnd.random() < .8:
        cfg['fruehabbruch']='q1'
    return cfg


def paardienst(modell=None):
    from lernen.richter_paar import Dienst, MODELL
    key = str(modell or MODELL)
    with PAAR_LOCK:
        if key not in PAAR_DIENSTE: PAAR_DIENSTE[key] = Dienst(modell=key)
        return PAAR_DIENSTE[key]
QUAL_TOLERANZ = 1.15

ARME = dict(kreuz=[1, 2, 4], trenn=[2, 3], via_kosten=[0.3, 0.6, 1.0], fein=['1', '0'], pfmul=[1.25, 1.4], form=['45r', 'organisch'],
            placer=['alt', 'insel'], router=['alt', 'konvergenz'])


PLACER_VARIANTEN = {'alt': [], 'insel': ['insel_physik=1', 'pin_nachlegal=1', 'bga_gegen=1', 'bedien_neu_frei=1']}


ROUTER_VARIANTEN = {'alt': [], 'konvergenz': ['konfliktort=exakt', 'quellfrei=1', 'konvergenz=lokal', 'pfmax=1000', 'kurz_via=8'],
                   'r27': ['r27_stamm=1']}


def klasse(A, ziel='v3'):
    ""
    k = ('leistung_' if A.get('leistungsboard') else '') + 'L%d_%s_s%d' % (A['lagen'], 'bga' if any('BGA' in b for b in A['bloecke']) else 'qfn', int(round(A.get('stufe', 0) / 2.0) * 2))

    return k + '|profil:' + A.get('profil', 'ausgewogen') if ziel == 'minimax' else k


def strom(x):
    ""
    sx = (x.get('regel') or {}).get('strom_x_max')
    if not isinstance(sx, (int, float)) or isinstance(sx,bool) or not math.isfinite(sx): return 'unbekannt', None
    return ('verletzt' if sx > 1.0 else 'erfuellt'), float(sx)


STROM_RANG = dict(erfuellt=0, unbekannt=1, verletzt=2)


def belohnungen(zeilen):
    ""


    minima={}
    for x in zeilen:
        if not x.get('gueltig'):continue
        r=x.get('regel') or {};sig=r.get('messversion') or (x.get('tor') or {}).get('messversion')
        if not sig:continue
        key=(x.get('aufgabe'),round(x.get('faktor') or 0,3),sig)
        vals=minima.setdefault(key,{})
        for k,v in VT.von_zeile(x)['regeln'].items():
            if v is not None:vals[k]=min(vals.get(k,float('inf')),v)
    best = {}
    for x in zeilen:
        if x.get('lehrer_status', 'erfuellt') == 'erfuellt' and x.get('gueltig') and isinstance(x.get('note_v3'), (int, float)) and strom(x)[0] == 'erfuellt':
            k = (x.get('aufgabe'), round(x.get('faktor') or 0, 3)); best[k] = min(best.get(k, 9e99), x['note_v3'])
    out = []
    for x in zeilen:
        if x.get('lehrer_status', 'erfuellt') != 'erfuellt' or not x.get('gueltig'): out.append(2.0); continue
        st, sx = strom(x)
        if st == 'unbekannt': out.append(1.5); continue
        if st == 'verletzt': out.append(round(1.5 + 1e-4 + 0.4899 * min(1.0, math.log(sx) / math.log(50.0)), 4)); continue
        b = best.get((x.get('aufgabe'), round(x.get('faktor') or 0, 3))); n = x.get('note_v3')
        r=x.get('regel') or {};sig=r.get('messversion') or (x.get('tor') or {}).get('messversion')
        if sig:
            vals=VT.von_zeile(x)['regeln'];ref=minima.get((x.get('aufgabe'),round(x.get('faktor') or 0,3),sig),{})
            if any(v is None for v in vals.values()):out.append(1.5);continue
            ratios=[max(0.,v-ref.get(k,v))/max(abs(ref.get(k,v)),1e-9) for k,v in vals.items()]
            excess=sum(q/(1+q) for q in ratios)/max(1,len(ratios))
            out.append(1.+.49*excess);continue
        out.append(1.0 + 0.49 * min(1.0, n / b - 1.0) if b and isinstance(n, (int, float)) and b > 0 else 1.25)
    from lernen.richter_paar import qualitaets_belohnung
    return [paardienst(x.get('paar_modell_pfad')).belohnung(x, qualitaets_belohnung(x, b) if x.get('paar_qualitaet') else b)
            if x.get('paar_aktiv') else b for x, b in zip(zeilen, out)]


def bewertungsziel(z):
    return (z.get('regel') or {}).get('bewertungsziel') or 'v3'


def belohnung(z):
    ""
    if z.get('lehrer_status', 'erfuellt') != 'erfuellt': return 2.0
    if bewertungsziel(z) != 'minimax': return belohnungen([z])[0]
    b = MM.belohnung(z)
    return paardienst(z.get('paar_modell_pfad')).belohnung(z, b) if z.get('paar_aktiv') else b


def besser_als(z, bester):
    decision = None
    if bester is not None and z.get('paar_aktiv') and z.get('flaeche_mm2') == bester.get('flaeche_mm2'):
        judge = paardienst(z.get('paar_modell_pfad'))
        try:
            with judge.lock:
                judge.laden(); released = judge.anwendbar(z) and judge.anwendbar(bester.get('zeile') or bester)
        except (OSError, ValueError, KeyError): released = False
        if released:
            decision = judge.vergleich(bester.get('zeile') or bester, z)
            if not decision['nicht_schlechter']: return False
    if _besser_als_basis(z, bester): return True
    if bester is not None and z.get('paar_aktiv') and z.get('flaeche_mm2') == bester.get('flaeche_mm2'):
        before = bester.get('zeile') or bester
        if bewertungsziel(z) == 'minimax' and not MM.annahme(before.get('regel') or {}, z.get('regel') or {})['nicht_schlechter']:
            return False
        return bool(decision and decision['paar'] and decision['besser'])
    return False


def _besser_als_basis(z, bester):
    if not z.get('gueltig'): return False
    if z.get('tor') or (z.get('regel') or {}).get('tor'):
        current=VT.von_zeile(z)
        if current['elektrisch']=='verletzt' or any(v is None for v in current['regeln'].values()):return False
        if bester is not None:
            before=VT.von_zeile(bester.get('zeile') or bester)
            if not VT.vergleich(before,current)['nicht_schlechter']:return False
    if bewertungsziel(z) != 'minimax':


        n, nb = z.get('note_v3'), (bester or {}).get('note_v3')
        if bester is None: return True
        if not isinstance(n, (int, float)): return False


        (sz, xz), (sb, xb) = strom(z), strom(bester)
        if STROM_RANG[sz] != STROM_RANG[sb]: return STROM_RANG[sz] < STROM_RANG[sb]
        if sz == 'verletzt':
            if xz > xb * 1.1: return False
            if xz < xb / 1.1 and z['flaeche_mm2'] <= bester['flaeche_mm2'] + 1e-6: return True
        qual_ok = not isinstance(nb, (int, float)) or n <= nb * QUAL_TOLERANZ
        return (z['flaeche_mm2'] < bester['flaeche_mm2'] - 1e-6 and qual_ok) or (
            z['flaeche_mm2'] <= bester['flaeche_mm2'] + 1e-6 and (not isinstance(nb, (int, float)) or n < nb))
    b = z.get('regel') or {}
    if not MM.aktuell(b) or not MM.vollstaendig(b) or not b.get('sicherheit', {}).get('sicher'): return False
    if bester is None: return True
    a = bester.get('regel') or {}
    if bewertungsziel(bester) != 'minimax': return False
    c = MM.annahme(a, b)
    if not c['nicht_schlechter']: return False
    return c['besser'] if c['delta'] else z['flaeche_mm2'] < bester['flaeche_mm2']


def klassenpassend(x, kl):
    k = x.get('klasse', '')
    if k == kl: return True

    if re.search(r'_s\d+(?:$|\|)', k): return False
    base = k.split('|profil:')[0]
    for p in ('ausgewogen', 'low_power', 'high_end_si', 'leistung'):
        if base.endswith('_' + p): base = base[:-len(p)-1]; break
    st = int(round(x.get('stufe_aufgabe', 0) / 2.0) * 2)
    neu = base + '_s%d' % st
    if bewertungsziel(x) == 'minimax': neu += '|profil:' + (x.get('regel') or {}).get('minimax_profil', 'ausgewogen')
    return neu == kl


def arm(x, k):
    ""
    v = (x.get('cfg') or {}).get(k)
    if k == 'bga' and v is None: return 'aus'
    return 'alt' if k in ('placer', 'router') and v is None else v


BGA_VARIANTEN = {'aus': [], 'fanout': ['bga_fanout=1'],
                 'plan': ['bga_fanout=1', 'bga_plan=1'],
                 'gassen': ['bga_fanout=1', 'bga_plan=1', 'bga_gassen=1'],
                 'eigenvia': ['bga_fanout=1', 'bga_plan=1', 'bga_eigenvia=1'],
                 'sicher': ['bga_fanout=1', 'bga_plan=1', 'bga_eigenvia=1', 'bga_innen_breite=1']}
def form_arme_lesen(wert):
    ""
    if wert is None: return None
    if wert == '0': return list(ARME['form'])
    if not isinstance(wert, str): raise ValueError('form_arme muss eine kommaseparierte Zeichenkette sein')
    arme = [x.strip() for x in wert.split(',')]
    if any(x not in ('45r', 'organisch', 'je_netzart') for x in arme) or len(set(arme)) != len(arme):
        raise ValueError('form_arme: eindeutige Werte aus 45r,organisch,je_netzart erwartet')
    return arme


def wahl(zeilen, kl, rnd, ziel='v3', bga_achse=False, stil_je_netzart=False, form_arme=None, tempo_fehlerbuch=None, zug_arm=False):
    ""
    zeilen = [x for x in zeilen if not FR.abgebrochen(x) and not x.get('generatorfehler') and not x.get('nicht_geroutet') and bewertungsziel(x) == ziel and
              x.get('rezept', {}).get('art') != 'kalibrierung' and x.get('zustand') != 'code_geaendert']
    if ziel == 'minimax':
        profil = kl.split('|profil:')[-1]
        zeilen = [x for x in zeilen if (x.get('regel') or {}).get('minimax_profil') == profil and belohnung(x) is not None]
    zeilen = [x for x in zeilen if str(x.get('version', '')).split('+')[0] == AU.VERSION]
    zeilen = zeilen[-3000:]
    rew = [belohnung(x) for x in zeilen] if ziel == 'minimax' else belohnungen(zeilen)
    cfg = {}
    arme = dict(ARME)
    formen = form_arme_lesen(form_arme)
    if formen is not None: arme['form'] = formen
    elif stil_je_netzart: arme['form'] = [*arme['form'], 'je_netzart']
    if bga_achse and re.search(r'L[68]_bga_', kl):
        arme['bga'] = list(BGA_VARIANTEN)
    if zug_arm: arme['zug'] = ['0', '1']
    for k, werte in sorted(arme.items()):
        best, bw = None, 9e99
        for w in werte:
            r = [b for x, b in zip(zeilen, rew) if b is not None and arm(x, k) == w and klassenpassend(x, kl)] or \
                [b for x, b in zip(zeilen, rew) if b is not None and arm(x, k) == w
                 and (k != 'bga' or re.search(r'L[68]_bga_', x.get('klasse', '')))]
            if not r: m, s = 1.5, 0.5
            else:
                m = sum(r) / len(r); s = max(0.1, (sum((q - m) ** 2 for q in r) / max(1, len(r) - 1)) ** 0.5) / math.sqrt(len(r))
            v = rnd.gauss(m, s)
            if tempo_fehlerbuch is not None: v += TT.armstrafe(tempo_fehlerbuch, k, w)
            if v < bw: best, bw = w, v
        cfg[k] = best
    cfg['seed'] = rnd.randint(1, 10 ** 6)
    return cfg


KURZSCHLUSS = {}
ZONENPLAN = dict(modus='aus', r36='geometrisch')

def lauf(cmd, cpu, log):
    r = PL.lauf(cmd, cpu_s=cpu, log=log, env=dict(os.environ, PYTHONHASHSEED='0'), cwd=R)
    return r['zustand'], r['cpu_s']


def code_hashes():
    from lernen.sammeln import code_stand
    return dict(code_sha256=code_stand(),
                router_sha256=hashlib.sha256((Path(R) / 'router/route_pf.py').read_bytes()).hexdigest(),
                klassen_sha256=hashlib.sha256((Path(R) / 'analyse/klassen.py').read_bytes()).hexdigest())


def code_pruefen(z):
    jetzt = code_hashes()
    if any(z.get(k) != v for k, v in jetzt.items()):
        z.update(code_geaendert=True, code_nachher=jetzt, gueltig=False, zustand='code_geaendert')
        return False
    return True



_HAUPT = Path(os.environ.get('AUTOROUTER_HAUPT') or Path(__file__).resolve().parents[1])
ZUG_MODELL = _HAUPT / 'lernen/modell/zugwahl.json'
ZUG_DATEN = _HAUPT / 'lernen/daten/zuege_endphase.jsonl'


def zuege_sammeln(c, A, cfg):
    ""
    import fcntl, glob
    zeilen = []
    try: kl = klasse(A)
    except Exception: kl = None
    for f in sorted(glob.glob(os.path.join(c, '**', '*_zuege.jsonl'), recursive=True)):
        for l in open(f):
            z = json.loads(l); z.update(aufgabe=A.get('seed'), klasse=kl, datei=os.path.relpath(f, c))
            zeilen.append(json.dumps(z, sort_keys=True))
    if not zeilen: return
    with open(ZUG_DATEN, 'a') as out:
        fcntl.flock(out, fcntl.LOCK_EX); out.write('\n'.join(zeilen) + '\n'); fcntl.flock(out, fcntl.LOCK_UN)


def _versuch(ad, A, cfg, k, zeit, it, nur_platzieren=False, vorbereitet=False, cpu=None, protokoll=None):
    ""
    from gemeinsam.verschieben import bewegen
    from analyse import korpus as KO
    from lernen import sammeln as S
    cfg = copy.deepcopy(cfg)
    mk = ME.klasse(A); meister_doc = None
    if cfg.get('meister', '1') != '0' and str(cfg.get('std', '1')) != '0' and 'kosten' not in cfg:
        meister_doc = ME.lesen(cfg.get('meister_cfg'))
        _, entry = ME.auswahl(meister_doc, mk, exakt=True)
        if entry:
            cfg['kosten'] = entry['parameter']; cfg['meister_generation'] = entry['generation']
    kosten = ME.parameter(cfg.get('kosten', {}))
    if 'form' in kosten:cfg['form']=kosten['form']
    if 'placer_stil' in kosten and isinstance(cfg.get('placer','alt'),str):cfg['placer']=kosten['placer_stil']
    if 'router_stil' in kosten and isinstance(cfg.get('router','alt'),str):cfg['router']=kosten['router_stil']


    def meister_optionen(tool):
        keys = ME.PLACER if tool == 'placer' else ME.ROUTER
        source = ['meister_cfg=' + os.path.join(c, 'meister_cfg.json')] if meister_doc and kosten else ['meister=0']
        return source + (['std='+str(cfg['std'])] if 'std' in cfg else []) + ['meister_klasse=' + mk] + [key+'='+value for key,value in sorted(ME.werkzeugoptionen(kosten,tool).items())]
    c = os.path.join(ad, 'k%d' % k); os.makedirs(c, exist_ok=True); t0 = time.time()
    if meister_doc and kosten:
        Path(c, 'meister_cfg.json').write_text(json.dumps(meister_doc))
    z = dict(aufgabe=A['seed'], kandidat=k, klasse=klasse(A, A.get('bewertungsziel', 'v3')), lagen=A['lagen'], faktor=A['faktor'], flaeche_mm2=A['flaeche_mm2'],
             teile=A['teile'], pads=A['pads'], netze=A['netze'], cfg=cfg, version=A.get('version', AU.VERSION), zeit=time.strftime('%Y-%m-%d %H:%M:%S'),
             stufe_aufgabe=A.get('stufe', 0), bloecke=A['bloecke'], laengengruppen=A.get('laengengruppen'))
    if protokoll is not None:
        protokoll.update(z); z = protokoll
    if PLK.aktiv(cfg.get('placer_lockern')): z.update(PLK.etikett(A))
    z['_cpu_kind_s'] = 0.0
    budget = int(zeit * min(5.0, max(1.0, A['pads'] / 300.0)))
    if cfg.get('budget') == 'arbeit': budget = int(cfg.get('arbeit_cpu_notbremse', 60000))
    if cfg.get('lernbudget') == 'adaptiv': budget = int(zeit)
    def rechenlauf(cmd, limit, log):
        if cpu is None:
            st, verwendet = lauf(cmd, limit, log)
            z['_cpu_kind_s'] += verwendet
            return st, verwendet
        if cpu.rest < 1: return 'zeitlimit', 0.0
        st, verwendet = lauf(cmd, min(limit, int(cpu.rest)), log)
        cpu.add(verwendet)
        z['_cpu_kind_s'] += verwendet
        return st, verwendet
    def klassifizieren(bj, kj, name):
        if cpu is None: return KO.klassen_lauf(ad, bj, kj, os.path.join(ad, 'vorgaben.json'))
        cmd = [VPY, os.path.join(R, 'analyse/klassen.py'), bj, kj,
               'regeln=' + os.path.join(ad, 'regeln.json'), 'vorgaben=' + os.path.join(ad, 'vorgaben.json')]
        st, cp = rechenlauf(cmd, 300, os.path.join(c, name + '.log'))
        z[name] = dict(zustand=st, cpu_s=cp)
        if st != 'ok': return None
        return json.load(open(kj))
    if not vorbereitet:
        z.update(code_hashes(), format='v2', gndlagen=list(A['gndlagen']))
        if A.get('lagenaufbau'):
            z.update(format='v3', lagenaufbau=True, leistungsboard=A.get('leistungsboard', False))
        if cpu:
            from lernen.lehrer import kontext
            z['kontext'] = kontext(cpu.limit, it, z['code_sha256'])
            if klassifizieren(os.path.join(ad, 'board.json'), os.path.join(ad, 'klassen.json'), 'klassen_eingang') is None:
                z.update(gueltig=False, zustand='klassen_eingang_' + z['klassen_eingang']['zustand']); return z
        pcmd = [VPY, os.path.join(R, 'platzierung/placer.py'), os.path.join(ad, 'board.json'), os.path.join(ad, 'klassen.json'),
                os.path.join(c, 'placement.json'), 'regeln=' + os.path.join(ad, 'regeln.json'), 'modus=neu', 'seiten=F', 'drehen_regel=0',
                'seed=%d' % cfg['seed'], 'kreuz=%g' % cfg['kreuz'], 'trenn=%g' % cfg['trenn'], 'via_kosten=%g' % cfg['via_kosten'], 'fein=' + cfg['fein']]
        pv = cfg.get('placer') or []
        pcmd += list(PLACER_VARIANTEN[pv] if isinstance(pv, str) else pv)
        pcmd += [f'{key}={cfg[key]}' for key in ('dicht', 'dicht_innen', 'p35') if key in cfg]
        pcmd += meister_optionen('placer')
        if cfg.get('kupferreserve'): pcmd += ['kupferreserve=1']
        zp, cp = rechenlauf(pcmd, budget, os.path.join(c, 'placer.log'))
        z['placer_versuche'] = [dict(seed=cfg['seed'], zustand=zp, cpu_s=cp)]


        def _kol():
            try: return ((json.load(open(os.path.join(c, 'placement.json'))).get('kennzahlen') or {}).get('nachher') or {}).get('kollisionen')
            except (OSError, ValueError): return None
        z['placer_wdh'] = 0


        for w_ in range(0 if PLK.aktiv(cfg.get('placer_lockern')) else 2):
            if zp != 'ok' or _kol() == 0: break
            z['placer_wdh'] += 1
            pcmd2 = [x if not x.startswith('seed=') else 'seed=%d' % (cfg['seed'] + 7919 * (w_ + 1)) for x in pcmd]
            zp, cp2 = rechenlauf(pcmd2, budget, os.path.join(c, 'placer_wdh%d.log' % (w_ + 1))); cp += cp2
            z['placer_versuche'].append(dict(seed=cfg['seed'] + 7919 * (w_ + 1), zustand=zp, cpu_s=cp2))
        z['placer'] = dict(zustand=zp, cpu_s=cp)
        if zp != 'ok' or not os.path.exists(os.path.join(c, 'placement.json')):
            z.update(gueltig=None if cfg.get('budget') == 'arbeit' else False, zustand='placer_' + zp, s=round(time.time() - t0, 1)); return z
        kol = _kol()
        if kol is not None and kol > 0 and str(cfg.get('legal_lp', '0')).lower() in ('1', 'true'):



            cmd = [VPY, os.path.join(R, 'platzierung/legal_lp.py'),
                   os.path.join(ad, 'board.json'), os.path.join(ad, 'klassen.json'),
                   os.path.join(ad, 'regeln.json'), os.path.join(c, 'placement.json')]
            if cfg.get('budget') == 'arbeit': cmd += [os.path.join(c, 'placement.json'), 'budget=arbeit']
            placement_datei = Path(c) / 'placement.json'
            original = placement_datei.read_bytes()
            zl, cl = rechenlauf(cmd, budget if cfg.get('budget') == 'arbeit' else 60, os.path.join(c, 'legal_lp.log'))
            z['legal_lp'] = dict(zustand=zl, cpu_s=cl)
            z['placer']['cpu_s'] += cl
            if zl != 'ok': placement_datei.write_bytes(original)
            if zl != 'ok' and cfg.get('budget') == 'arbeit':
                z.update(gueltig=None, zustand='legal_lp_unvollstaendig'); return z
            kol = _kol()
        if kol != 0:
            try: z['placer_kennzahlen'] = (json.load(open(os.path.join(c, 'placement.json'))).get('kennzahlen') or {}).get('nachher')
            except (OSError, ValueError): pass
            z.update(gueltig=False, zustand='placer_kollision', s=round(time.time() - t0, 1),
                     ungueltig_grund='Platzierung: %s' % ('%d Kollisionen' % kol if kol is not None else 'Kollisionsmessung fehlt'))
            return z
        B = json.load(open(os.path.join(ad, 'board.json'))); P = json.load(open(os.path.join(c, 'placement.json')))['parts']
        fps = {f['ref']: f for f in B['fps']}
        moves = {r: (p['x'] - fps[r]['x'], p['y'] - fps[r]['y'], p.get('drehung', 0.0)) for r, p in P.items() if r in fps}
        BA, _ = bewegen(B, moves, seiten={r: p['side'] for r, p in P.items() if r in fps})
        from platzierung.p3_gruppen import board_mit_gruppen
        BA = board_mit_gruppen(BA, json.load(open(os.path.join(c, 'placement.json'))))
        reserve = json.load(open(os.path.join(c, 'placement.json'))).get('kupferreserve')
        if reserve: BA['kupferreserve'] = reserve
        bj = os.path.join(c, 'board.json'); json.dump(BA, open(bj, 'w'))
        KA = klassifizieren(bj, os.path.join(c, 'klassen.json'), 'klassen_placement')
        if KA is None:
            z.update(gueltig=False, zustand='klassen_placement_' + z['klassen_placement']['zustand']); return z
        try:
            from lernen.merkmale import merkmale
            z['merkmale'] = merkmale(BA, KA, json.load(open(os.path.join(ad, 'regeln.json'))), gndlagen=A['gndlagen'])
            z['placer_kennzahlen'] = (json.load(open(os.path.join(c, 'placement.json'))).get('kennzahlen') or {}).get('nachher')
        except Exception as e: z['merkmale_fehler'] = repr(e)[:160]
        if TT.aktiv(cfg.get('lehrplan_tempo', '0')):
            z['tempo_richter'] = TT.richter_schatten(z, cfg.get('tempo_richter_modell', os.path.join(R, 'lernen/modell/richter')))
        z['budget'] = dict(cpu_s=budget, iter=it)
        z['s'] = round(time.time() - t0, 1)
        if not code_pruefen(z): return z
        if cpu: cpu.protokoll(z)
        json.dump({key: value for key, value in z.items() if key != '_cpu_kind_s'},
                  open(os.path.join(c, 'vorbereitet.json'), 'w'))
    else:
        geladen = json.load(open(os.path.join(c, 'vorbereitet.json')))
        if protokoll is not None:
            protokoll.update(geladen); z = protokoll
        else: z = geladen
        from lernen.lerneffekt import cpu_s
        vorbereitung = cpu_s(geladen)
        if vorbereitung is None:
            z.update(cpu_total=None, cpu_vorbereitung_unbekannt=True,
                     gueltig=False, zustand='fehler',
                     fehler='vorbereitete CPU unbekannt'); return z
        z['_cpu_kind_s'] = 0.0
        z['_cpu_vorbereitung_s'] = vorbereitung
        if cpu:
            from lernen.lehrer import kontext
            cpu.add(vorbereitung)
            if z.get('kontext') != kontext(cpu.limit, it, z.get('code_sha256')):
                z.update(gueltig=False, zustand='fehler', fehler='vorbereiteter CPU-Kontext geaendert'); return z
        t0 -= z.get('s', 0)
        bj = os.path.join(c, 'board.json')
        BA = json.load(open(bj)); KA = json.load(open(os.path.join(c, 'klassen.json')))
        if z.get('budget') != dict(cpu_s=budget, iter=it) or z.get('cfg') != cfg:
            z.update(gueltig=False, zustand='fehler', fehler='vorbereiteter Routerkontext geaendert'); return z
    if not code_pruefen(z): return z
    if nur_platzieren: return z
    rj = os.path.join(c, 'routes.json')
    mode = cfg.get('zonenplan', ZONENPLAN['modus'])
    if mode not in ('aus', 'diagnose', 'lokal'): raise ValueError('zonenplan=aus|diagnose|lokal')
    r36 = cfg.get('r36', ZONENPLAN['r36'])
    if r36 not in ('legacy','nativ','geometrisch','0'): raise ValueError('r36=legacy|nativ|geometrisch|0')
    plan_extra = []
    if mode != 'aus':
        from lernen.zonenplaner import vorbereiten as zp_vorbereiten
        from contextlib import nullcontext

        with cpu.separat() if cpu and mode == 'diagnose' else nullcontext():
            z['zonenplan'] = zp_vorbereiten(bj, os.path.join(c, 'klassen.json'), os.path.join(ad, 'regeln.json'), mode,
                                          A['gndlagen'], cpu_s=120 if cpu is None or mode == 'diagnose' else max(1,int(cpu.rest)))
        if cpu and mode == 'lokal': cpu.add(z['zonenplan']['rechenplatz']['cpu_s'])
        if mode == 'lokal':
            if not z['zonenplan']['angewendet']:
                z.update(gueltig=False,zustand='zonenplan_fehler'); return z
            plan_extra = ['zonenplan='+z['zonenplan']['datei']]
    rcmd = [VPY, os.path.join(R, 'router/route_pf.py'), bj, rj, 'regeln=' + os.path.join(ad, 'regeln.json'), 'klassen=' + os.path.join(c, 'klassen.json'),
            'gndlagen=' + ','.join(A['gndlagen']), 'iter=%d' % it, 'pf0=1', 'pfmul=%g' % cfg['pfmul'], 'hinc=1.0', 'ziel_ab=20', 'kurz=1', 'fair=1',
            'stil=' + (cfg['form'] if cfg.get('form') in ('organisch', 'je_netzart') else '45'),
            'cpu_max=%d' % budget]
    arbeitsmessung = cfg.get('budget') == 'arbeit'
    if arbeitsmessung:
        if str(cfg.get('lerndaten', '1')) != '0': raise ValueError('Arbeitsmessung braucht lerndaten=0')
        if cfg.get('lernbudget') or FR.aktiv(cfg.get('fruehabbruch')):
            raise ValueError('Arbeitsmessung erlaubt keine CPU-gesteuerte Lernpolicy')
        notbremse = int(cfg.get('arbeit_cpu_notbremse', 60000))
        rcmd = [x for x in rcmd if not x.startswith('cpu_max=')]
        rcmd += ['budget=arbeit', 'arbeit_max=%d' % int(cfg.get('arbeit_max', 59147)),
                 'seed=%d' % cfg['seed'], 'cpu_max=%d' % notbremse]
    if str(cfg.get('lerndaten', '1')) == '1':
        rcmd += ['lerndaten=1', 'etiketten=' + os.path.join(c, 'etiketten.npz')]
    route_limit = (notbremse if arbeitsmessung else budget) + 60
    if cfg.get('lernbudget') == 'adaptiv':
        cap = int(cfg.get('lern_cpu_max', 2 * budget))
        rcmd += ['lernbudget=adaptiv', 'lern_cpu_max=%d' % cap]
        route_limit = cap + 60
    if 'budget_vor_nachlauf' in cfg: rcmd += ['budget_vor_nachlauf=' + str(cfg['budget_vor_nachlauf'])]
    if cfg.get('lernpolicy'): rcmd += ['lernpolicy=' + str(cfg['lernpolicy'])]
    if FR.aktiv(cfg.get('fruehabbruch')) and cpu is not None and cfg.get('lernbudget') != 'adaptiv':


        rcmd = [x for x in rcmd if not x.startswith('cpu_max=')] + ['cpu_max=%d' % max(1,int(min(budget,cpu.rest)))]
    if FR.aktiv(cfg.get('fruehabbruch')):
        from lernen.fruehabbruch import optionen, kontext_aufgabe
        rcmd += optionen(cfg, kontext_aufgabe(A))
    rv = cfg.get('router') or 'alt'
    rcmd += (ROUTER_VARIANTEN.get(rv, []) if isinstance(rv, str) else list(rv)) + plan_extra
    rcmd += BGA_VARIANTEN[cfg.get('bga', 'aus')]
    native_options = dict(x.split('=', 1) for x in rcmd[3:] if '=' in x)
    if arbeitsmessung and native_options.get('std', '1') == '1' and 'klassen' in native_options:

        native_options.setdefault('skew', '1'); native_options.setdefault('versuche', '3')
    if (arbeitsmessung and int(native_options.get('versuche', '0')) > 0
            and 'versuche_cpu_notbremse' not in native_options):



        rcmd += ['versuche_cpu_notbremse=%d' % (notbremse + 60)]
    if (native_options.get('skew') == '1' or int(native_options.get('versuche', '0')) > 0) and not native_options.get('nativ_basis'):

        from lernen.kupfer_nativ import regelvertrag
        from kicad.python_aufruf import python_befehl
        native = Path(c, 'native_basis'); native.mkdir(exist_ok=True)
        base = native/'base.kicad_pcb'
        native_cmd = (python_befehl(Path(R, 'codex/generator_echt/nativ.py'), bj, base, ad)
                      if str(A.get('version', '')).split('+')[0] == '11' else
                      python_befehl(Path(R, 'kicad/strom_board.py'), bj, base))
        nb, nc = rechenlauf(native_cmd, 300, str(native/'aufbau.log'))
        z['native_basis'] = dict(zustand=nb, cpu_s=nc)
        if nb != 'ok':
            z.update(gueltig=None, zustand='native_basis_unvollstaendig'); return z
        if str(A.get('version', '')).split('+')[0] != '11':
            project, dru = regelvertrag(BA, dict(optionen=native_options), json.load(open(Path(ad, 'regeln.json'))))
            base.with_suffix('.kicad_pro').write_text(json.dumps(project))
            base.with_suffix('.kicad_dru').write_text(dru)
        zones = (json.loads(Path(ad, 'zones.json').read_text()) if str(A.get('version', '')).split('+')[0] == '11' else
                 [dict(net='GND', layer=layer, pts=poly, full=True, prio=0) for layer in A['gndlagen'] for poly in BA['outline']])
        (native/'zones.json').write_text(json.dumps(zones))
        rcmd += ['nativ_basis='+str(base), 'nativ_zonen='+str(native/'zones.json')]
    if arbeitsmessung:
        route_limit *= max(1, int(native_options.get('versuche', '0')))
    rcmd += [f'{key}={cfg[key]}' for key in ('hist_start', 'hist_context') if cfg.get(key)]
    rcmd += [f'{key}={cfg[key]}' for key in ('restkonflikte', 'rest_cpu', 'rest_teile', 'rest_kand') if key in cfg]
    from nachbearbeitung.kurzschluss_schutz import schutz_an, schutz_option
    rcmd += schutz_option(schutz_an(rcmd, KURZSCHLUSS, cfg))
    rcmd += meister_optionen('router')
    if str(cfg.get('zug', '0')) == '1':

        rcmd += ['zug=1'] + (['zug_politik=' + str(ZUG_MODELL)] if ZUG_MODELL.exists() else [])
    zr, cr = rechenlauf(rcmd, route_limit, os.path.join(c, 'route.log'))
    if str(cfg.get('zug', '0')) == '1': zuege_sammeln(c, A, cfg)
    z['router'] = dict(zustand=zr, cpu_s=cr)
    if arbeitsmessung and zr != 'ok':
        z.update(gueltig=None, zustand='arbeitsmessung_unvollstaendig',
                 ungueltig_grund='Routerfehler/CPU-Notbremse ist kein reproduzierbares Qualitaetsergebnis')
        return z
    if not code_pruefen(z): return z
    if not os.path.exists(rj):
        z.update(gueltig=False, zustand=zr if zr != 'ok' else 'router_ohne_ausgabe',
                 ungueltig_grund='Router ohne Teilergebnis'); return z
    if FR.aktiv(cfg.get('fruehabbruch')) and json.load(open(rj)).get('abschluss') == 'fruehabbruch':

        from nachbearbeitung.kurzschluss_schutz import kandidat_sichern
        grund = kandidat_sichern(bj, rj)
        z.update(gueltig=None, zustand='r44_ungeprueft' if grund else 'fruehabbruch',
                 ungueltig_grund=grund or 'Vorhersage: kein Erfolg im CPU-Budget (zensiert)')
        return z
    if os.path.exists(rj):
        from gemeinsam.zonen import boardgeometrie
        _rest_r = json.load(open(rj))
        if _rest_r.get('rest_board'):
            BA = boardgeometrie(BA, _rest_r)
            json.dump(BA, open(bj, 'w'))
    if cfg.get('zonen_richter_diagnose'):
        rr = json.load(open(rj)); rr['optionen'] = dict(rr.get('optionen') or {}, zonen_richter_diagnose='1')
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('zonen_richter') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['optionen'] = dict(rr.get('optionen') or {}, zonen_richter='1')
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_glatt') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_glatt'] = True
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_korridor') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_korridor'] = True
        Path(rj).write_text(json.dumps(rr))
    if cfg.get('r36_padgruppen') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_padgruppen'] = True
        Path(rj).write_text(json.dumps(rr))
    if str(cfg.get('r36_k17', '0')).lower() in ('1', 'true') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['r36_k17'] = True
        Path(rj).write_text(json.dumps(rr))
    if str(cfg.get('k17_messen', '0')).lower() in ('1', 'true') and os.path.exists(rj):
        rr = json.load(open(rj)); rr['optionen'] = dict(rr.get('optionen') or {}, k17_messen='1')
        Path(rj).write_text(json.dumps(rr))
    if not code_pruefen(z): return z
    if cfg.get('r39') and os.path.exists(rj):
        RR = json.load(open(rj)); RR['optionen'] = dict(RR.get('optionen') or {}, r39='1')
        json.dump(RR, open(rj, 'w'))
    route_gndlagen = A['gndlagen']
    if 'strom_zuerst=1' in rcmd and os.path.exists(rj):


        route_gndlagen = json.load(open(rj))['optionen']['gndlagen'].split(',')
        z['gndlagen'] = route_gndlagen
    from lernen import form as FO
    try:
        if cpu is not None and cpu.rest < 1:
            z['form'] = dict(angewendet=False, grund='zeitlimit', cpu_s=0.0)
        else:
            z['form'] = FO.formen(bj, os.path.join(c, 'klassen.json'), rj, os.path.join(ad, 'regeln.json'), route_gndlagen, cfg.get('form', '45r'),
                                  log=os.path.join(c, 'form.log'), zeit=(notbremse if arbeitsmessung else 120) if cpu is None else max(1, int(cpu.rest)), strom_form=bool(cfg.get('strom_form', False)), minimax=True if A.get('bewertungsziel') == 'minimax' else None,
                                  r27_formtor=str(cfg.get('r27_formtor', '0')).lower() in ('1', 'true'),
                                  r18=str(cfg.get('r18', '0')).lower() in ('1', 'true'),
                                  r18_abschluss=dict(cfg=cfg, mode=mode, r36=r36))
            if cpu: cpu.add(z['form'].get('cpu_s', 0.0))
            if arbeitsmessung and z['form'].get('grund') in ('zeit', 'zeitlimit', 'wandlimit', 'fehler'):
                z.update(gueltig=None, zustand='form_unvollstaendig'); return z
        if cpu and cpu.rest < 1:
            z.update(gueltig=False, zustand='zeitlimit'); return z
        r18_info = z.get('form', {}).get('r18') or {}
        if r18_info.get('abgeschlossen'):
            z.update(r18_info['abschluss'])
        else:
            try:
                z.update(FO.abschliessen(bj, os.path.join(c, 'klassen.json'), rj,
                    os.path.join(ad, 'regeln.json'), route_gndlagen, cfg, mode, r36, cpu))
            except TimeoutError:
                z.update(gueltig=False, zustand='zeitlimit'); return z
        if cfg.get('r19_nachlauf'):
            from lernen.r19_nachlauf import anwenden as r19_anwenden
            z['r19_nachlauf'] = r19_anwenden(bj, os.path.join(c, 'klassen.json'), rj,
                                            os.path.join(ad, 'regeln.json'), route_gndlagen,
                                            cpu_s=180 if cpu is None else max(1, int(cpu.rest)))
            if cpu: cpu.add(z['r19_nachlauf']['rechenplatz']['cpu_s'])
        from nachbearbeitung.kurzschluss_schutz import kandidat_sichern_messung
        r44_grund, z['r44'] = kandidat_sichern_messung(bj, rj)

        if r44_grund:
            z.update(gueltig=False, zustand='r44_ungeprueft', ungueltig_grund=r44_grund[:300], s=round(time.time() - t0, 1))
            code_pruefen(z)
            return z
        from gemeinsam.fluss import NetzgraphCache
        _graph_cache = NetzgraphCache(); _routes = json.load(open(rj))
        g, E = FO.gueltig(BA, KA, rj, route_gndlagen, fluss_cache=_graph_cache, routes=_routes)
    except Exception as e:
        from analyse.flaechen import aktiv
        RR = json.load(open(rj)) if os.path.exists(rj) else {}
        if not (aktiv(RR, KA) or A.get('lagenaufbau')): raise
        z.update(gueltig=False, zustand='bewertungsfehler', regel_fehler=repr(e)[:160],
                 ungueltig_grund='Flächenbewertung unvollständig', s=round(time.time() - t0, 1))
        code_pruefen(z)
        return z
    from lernen.zonenplaner import abnahmetor
    gueltig_routing = E.get('gueltig_routing', g) if E else g
    pflicht = 'nativ' if mode == 'lokal' or r36 == 'nativ' else ('legacy' if r36 in ('legacy', 'geometrisch') else False)
    gate = abnahmetor(dict(gueltig=g), z.get('stromzonen'), pflicht)
    g = gate['gueltig']
    if gate.get('ungueltig_grund'): z['ungueltig_grund'] = gate['ungueltig_grund']
    kol = (z.get('placer_kennzahlen') or {}).get('kollisionen') or 0
    if kol: g = gueltig_routing = False; z['ungueltig_grund'] = 'Platzierung: %d Kollisionen' % kol
    z['ergebnis'] = E; z['gueltig'] = g
    if A.get('bewertungsziel') == 'minimax': z['profil'] = A['profil']
    if E:
        RR = {}
        try:
            if cpu and cpu.rest < 1:
                z.update(gueltig=False, zustand='zeitlimit'); return z
            RR = _routes
            z['regel'] = S.regelmessung(BA, RR, KA, route_gndlagen, dicht=True, fluss_cache=_graph_cache, kette=bool(cfg.get('paar_qualitaet'))); z['note_v3'] = z['regel']['note_v3']


            if 'tor' in z['regel']: z['tor'] = z['regel']['tor']
            from analyse.flaechen import aktiv
            if (aktiv(RR, KA) or A.get('lagenaufbau')) and (z['regel'].get('fehler') or (z['regel'].get('regeln') or {}).get('fehler')):
                raise ValueError('Aktives Flächenmodell: Bewertungsfehler')
            if bewertungsziel(z) == 'minimax':
                z['gueltig_routing'] = gueltig_routing
                z['gueltig'] = z['gueltig'] and z['regel']['minimax_gueltig']
            z['bewertung_version'] = z['regel']['version']
        except Exception as e:
            z['regel_fehler'] = repr(e)[:160]
            from analyse.flaechen import aktiv
            if aktiv(RR, KA) or A.get('lagenaufbau'):
                z.update(gueltig=False, ungueltig_grund='Flächenbewertung unvollständig')
        from lernen.zonen_richter import diagnose
        zonen_rating = diagnose(BA, RR)
        z.update(zonen_richter_note=zonen_rating['zonen_richter_note'], zonen_richter=zonen_rating)
        z['kennzahlen'] = dict(vias=E.get('vias'), konflikte=E.get('konflikte'), offen=E.get('offen'), laenge=E.get('laenge_mm'),
                               knicke=E.get('knicke'), anteil_45=E.get('anteil_45'))
    z['zustand'] = 'ok' if zr == 'ok' else zr; z['s'] = round(time.time() - t0, 1)
    abnahmetor(z, z.get('stromzonen'), pflicht)
    if cpu and cpu.verwendet > cpu.limit:z.update(gueltig=False,zustand='zeitlimit',ungueltig_grund='Gesamtbudget ueberschritten')
    code_pruefen(z)
    return z


def versuch(ad, A, cfg, k, zeit, it, nur_platzieren=False, vorbereitet=False, gesamt_cpu=None, cpu_budget=None, kandidaten=3, kandidaten_auto=True):
    ""
    from lernen.cpubudget import Budget
    cpu = cpu_budget or (Budget(gesamt_cpu) if gesamt_cpu is not None else None)
    tc = time.thread_time(); z = {}
    try:
        z = _versuch(ad, A, cfg, k, zeit, it, nur_platzieren, vorbereitet, cpu, z)
        if not nur_platzieren and z.get('zustand') == 'ok' and z.get('cfg', {}).get('kosten'):
            from lernen.meister_tor import nativ as meister_nativ
            try:
                meister_nativ(Path(ad), A, z, cpu_budget=cpu)
                code_pruefen(z)
            except Exception as exc:
                z.update(gueltig=None,zustand='native_lernmessung_unvollstaendig',fehler=repr(exc))
    except Exception as e:
        z.update(gueltig=None if cfg.get('budget') == 'arbeit' else False, zustand='fehler', fehler=repr(e)[:300])
    from lernen.lerneffekt import ergaenzen
    c = os.path.join(ad, 'k%d' % k)
    def vorbereitung_speichern():
        p = Path(c) / 'vorbereitet.json'
        if nur_platzieren and not vorbereitet and p.exists():


            cached = json.loads(p.read_text())
            for key in ('cpu_total', 'cpu_aufruf_s', 'cpu_messung', 'cpu_budget', 'cpu_zeitlimit'):
                if key in z: cached[key] = z[key]
            p.write_text(json.dumps({key: value for key, value in cached.items()
                                     if not key.startswith('_cpu_')}))
    if cpu: z = cpu.protokoll(z)
    ergaenzen(z, c)
    if z.get('cfg', {}).get('kosten'):
        z['kosten_belohnung']=dict(version=ME.BELOHNUNG_VERSION,rang=ME.belohnung(z))
    if cpu:
        z.pop('_cpu_kind_s', None)
        z['cpu_messung'] = 'teilweise'
        z.setdefault('s', 0.0)
    if cfg.get('bga_diagnose') in (True, '1') and not nur_platzieren and not z.get('gueltig') and any('BGA' in b for b in A['bloecke']):
        try:
            from lernen.bga_diagnose import aus_dateien
            bp = Path(c, 'board.json')
            z['bga_diagnose'] = aus_dateien(json.loads(bp.read_text()), Path(c, 'etiketten.npz')) if bp.exists() else dict(zustand='unbekannt', grund='Platzierung fehlt')
        except Exception as e:
            z['bga_diagnose'] = dict(zustand='unbekannt', grund=repr(e)[:200])
    if str(cfg.get('datensatz', '1')) != '0':
        datensatz_aufnehmen(ad, A, z, k, nur_platzieren, cpu, kandidaten, kandidaten_auto)
    if cpu:


        if z.get('datensatz', {}).get('zustand') != 'ok':
            vorher = bool(z.get('cpu_zeitlimit'))
            cpu.protokoll(z)
            if vorher != bool(z.get('cpu_zeitlimit')):
                ergaenzen(z, c)
                cpu.protokoll(z)
        z['cpu_aufruf_s'] = z['cpu_total'] - z.pop('_cpu_vorbereitung_s', 0.0)
        if z.get('cpu_vorbereitung_unbekannt'): z['cpu_total'] = None
        vorbereitung_speichern()
        if FR.abgebrochen(z): z['gueltig'] = None
        return z


    own = time.thread_time() - tc
    children = z.pop('_cpu_kind_s', 0.0)
    children += (z.get('form') or {}).get('cpu_s', 0) or 0

    completed = ((z.get('form') or {}).get('r18') or {}).get('abgeschlossen')
    extra = ('zonenplan',) if completed else ('zonenplan', 'stromzonen', 'r11_ir', 'r24')
    children += sum((z.get(q) or {}).get('rechenplatz', {}).get('cpu_s', 0) or 0 for q in extra)
    z['cpu_aufruf_s'] = children + own
    z['cpu_total'] = z['cpu_aufruf_s'] + z.pop('_cpu_vorbereitung_s', 0.0)
    if z.get('cpu_vorbereitung_unbekannt'): z['cpu_total'] = None
    z['cpu_messung'] = 'teilweise'
    z.setdefault('s', 0.0)
    vorbereitung_speichern()
    return z


def datensatz_aufnehmen(ad, A, z, k, nur_platzieren=False, cpu=None, kandidaten=3, kandidaten_auto=True):
    ""
    _tc_dicht = time.thread_time()
    try:
        from lernen.ki import daten as D
        z.setdefault('designfamilie', 'erf_%06d' % A['seed'])
        z.setdefault('gndlagen', list(A['gndlagen']))
        metrics = (z.get('regel') or {}).pop('_physik', None)
        c = os.path.join(ad, 'k%d' % k)

        if os.path.exists(os.path.join(c, 'board.json')):
            from lernen.ki.merkmale_raster import ersatzmasse
            b = json.load(open(os.path.join(c, 'board.json'))); kk = json.load(open(os.path.join(c, 'klassen.json')))
            m, cheap = ersatzmasse(b, kk, gndlagen=z.get('gndlagen', A['gndlagen']))
            z['ersatzmasse'] = cheap
        else: m = None
        z['vergleichsgruppe'] = '%s:%s:%s' % (z['designfamilie'], hashlib.sha256(Path(ad, 'board.json').read_bytes()).hexdigest(), k)
        if cpu and not nur_platzieren and cpu.rest > 0:
            z['etiketten_kandidaten'] = D.kandidaten(c, z, os.path.join(ad, 'board.json'), kandidaten, kandidaten_auto, cpu_budget=cpu)
        z['datensatz'] = D.aufnehmen(c, z, os.path.join(ad, 'board.json'), metrics=metrics, eingang=m, cpu_budget=cpu)
        z['datensatz']['cpu_s'] = time.thread_time() - _tc_dicht
        if not nur_platzieren and cpu is None:
            z['etiketten_kandidaten'] = D.kandidaten(c, z, os.path.join(ad, 'board.json'), kandidaten, kandidaten_auto)
    except Exception as e: z['datensatz'] = dict(zustand='fehler', fehler=repr(e)[:200])
    return z


def stufe_lesen(kw):
    p = kw['daten'][:-6] + '_stufe.json' if kw['daten'].endswith('.jsonl') else kw['daten'] + '_stufe.json'
    try: return p, json.load(open(p))
    except (OSError, ValueError): return p, dict(stufe=float(kw.get('stufe', '0')), fenster=[], verlauf=[])


def stufe_melden(kw, geschafft, seed, generatorfehler=False):
    ""
    with LOCK:
        p, S = stufe_lesen(kw)
        if generatorfehler: S['generatorfehler'] = S.get('generatorfehler', 0) + 1
        S['fenster'] = (S.get('fenster') or [])[-7:] + [bool(geschafft)]
        if len(S['fenster']) >= 8:
            n = sum(S['fenster']); alt = S['stufe']
            if n >= 6: S['stufe'] = round(alt + 0.5, 2)
            elif n <= 2: S['stufe'] = round(max(0.0, alt - 0.5), 2)
            if S['stufe'] != alt:
                S['fenster'] = []; S.setdefault('verlauf', []).append(dict(zeit=time.strftime('%Y-%m-%d %H:%M'), aufgabe=seed, stufe=S['stufe'], von=alt))
                print('Stufe %.1f -> %.1f (%d von 8 geschafft)' % (alt, S['stufe'], n), flush=True)
        json.dump(S, open(p, 'w'), indent=1)


def schwierigkeit(A):
    bga = int(any('BGA' in b for b in A['bloecke'])); w = int(any(b.startswith(('Buck', 'Boost', 'Mini')) for b in A['bloecke']))
    opv = int(any('OPV' in b for b in A['bloecke'])); mcu = int(any(b.startswith('Prozessor') for b in A['bloecke']))
    return dict(punkte=3 * w + 2 * mcu + 4 * bga + opv + 3 + (A['lagen'] - 2) / 2 + A.get('stufe', 0), wandler=w, mcu=mcu, bga=bga, opv=opv,
                pads=A['pads'], lagen=A['lagen'], stufe=A.get('stufe', 0), erfunden=True)


def aufgabe_erzeugen(ad, seed, ziel=None, **kwargs):
    explizit = ziel
    ziel = standardziel(explizit=ziel, kontext='selbstspiel')
    a = AU.erzeugen(ad, seed, **kwargs)
    if ziel == 'minimax':
        a['profil'] = 'ausgewogen'
        a['bewertungsziel'] = ziel
        p = os.path.join(ad, 'vorgaben.json'); vg = json.load(open(p))
        vg.update(bewertung=ziel, profil=a['profil']); json.dump(vg, open(p, 'w'), indent=1)
        Path(ad, 'aufgabe.json').write_text(json.dumps(a, indent=1, ensure_ascii=False))
    elif explizit == 'v3' and standardziel() == 'minimax':

        a['bewertungsziel'] = 'v3'
        p = os.path.join(ad, 'vorgaben.json'); vg = json.load(open(p)); vg['bewertung'] = 'v3'
        json.dump(vg, open(p, 'w'), indent=1)
        json.dump(a, open(os.path.join(ad, 'aufgabe.json'), 'w'), indent=1, ensure_ascii=False)
    return a


def grenze_faktor(zeilen, n=40, aufschlag=1.10, f_min=1.10, f_max=1.55):
    ""
    best, reihe, gesehen = {}, [], set()
    for z in zeilen:
        a, f = z.get('aufgabe'), z.get('faktor')
        if a is None or not isinstance(f, (int, float)) or z.get('zustand') == 'code_geaendert': continue
        if a not in gesehen: gesehen.add(a); reihe.append(a)
        if z.get('gueltig'): best[a] = min(best.get(a, 9.0), f)
    werte = [best[a] for a in reihe if a in best][-n:]
    if len(werte) < 8: return None
    werte.sort(); med = werte[len(werte) // 2]
    return round(min(f_max, max(f_min, med * aufschlag)), 3)


def aufgabe_spielen(out, seed, kw, zeilen, rnd):
    ""
    optionen_pruefen(kw)
    KURZSCHLUSS.clear(); KURZSCHLUSS.update({k: kw[k] for k in ('kurzschluss_schutz',) if k in kw})
    if kw.get('rennbahn', '0') == '1':
        from lernen.rennbahn import spielen
        return spielen(out, seed, kw, zeilen, rnd)
    ZONENPLAN.update(modus=kw.get('zonenplan','aus'),r36=kw.get('r36','geometrisch'))
    if kw.get('lehrer', '0') == '1': return lehrer_spielen(out, seed, kw, zeilen, rnd)
    versuche = int(kw.get('versuche', '3')); vd = float(kw.get('verdichten', '0.93')); zeit = int(kw.get('zeit', '300'))
    it = int(kw.get('iter', '40')); gal = None if kw.get('galerie', '1') == '0' else os.path.expanduser('~/Library/Caches/Autorouter/galerie')
    ad = os.path.join(out, 'aufgabe_%06d' % seed)
    tempo = TT.aktiv(kw.get('lehrplan_tempo', '0'))
    version = kw.get('generatorversion') or ('11' if kw.get('kosten_lernen', '1') != '0' and seed % 5 == 0 else '10')
    achse = None
    if kw.get('bga_achse', '0') == '1':
        from lernen import bga_achsen as AX
        if tempo:
            achse = AX.reservieren(kw, seed, stufe_wahl=lambda fallback, axis: TT.stufe(zeilen, fallback, rnd, version, axis))
        else: achse = AX.reservieren(kw, seed)
    st = achse['stufe'] if achse else stufe_lesen(kw)[1]['stufe']
    overload_stufe = st
    tempo_cache = TT.Vorbereitung()
    tempo_buch = TT.fehlerbuch(kw.get('tempo_fehlerbuch', os.path.join(R, 'lernen/daten/fehlerbuch.json'))) if tempo else None
    if tempo and not achse:
        st = TT.stufe(zeilen, st, rnd, version, achse)

    def tempo_versuch(ad, A, cfg, kid, zeit, it, **opts):
        if not tempo: return versuch(ad, A, cfg, kid, zeit, it, **opts)
        return TT.versuch(tempo_cache, ad, A, cfg, kid, zeit, it, **opts)
    ziel = standardziel(explizit=kw.get('ziel'), kontext='selbstspiel')
    if ziel not in ('v3', 'minimax'): raise ValueError('ziel muss v3 oder minimax sein')
    neu = []; k = 0; bester = None; stufe = 0
    cpu_aufgabe = []
    locker_cfg = None
    def kosten(z):
        cpu_aufgabe.append(z.get('cpu_aufruf_s', z.get('cpu_total')))
    def achse_abschliessen(abgebrochen=False):
        if not achse: return
        from lernen.lerneffekt import _grenz_ergebnis
        geschafft = any(_grenz_ergebnis(z) is True and z['faktor'] <= faktor0 + 1e-9 for z in neu)
        unbekannt = not geschafft and (abgebrochen or any(
            not z.get('generatorfehler') and not z.get('nicht_geroutet') and _grenz_ergebnis(z) is None for z in neu))
        AX.melden(kw, seed, None if unbekannt else geschafft,
                  cpu_s=sum(cpu_aufgabe) if cpu_aufgabe and all(isinstance(c, (int, float)) and math.isfinite(c) and c >= 0 for c in cpu_aufgabe) and not any(z.get('generatorfehler') for z in neu) else None,
                  gueltige=sum(z.get('gueltig') is True for z in neu),
                  generatorfehler=bool(neu and neu[-1].get('generatorfehler')), abgebrochen=abgebrochen)
    def erzeugen(**kwargs):
        try:
            if kw.get('kosten_lernen', '1') != '0':
                kwargs.setdefault('generatorversion', kw.get('generatorversion') or ('11' if seed % 5 == 0 else '10'))
            a = aufgabe_erzeugen(ad, seed, ziel=ziel,
                                **(dict(gehaeuse=achse['gehaeuse'], lagen=achse['lagen']) if achse else {}), anker_legal=kw.get('anker_legal', '0') == '1', potenzial=kw.get('potenzial') == '1', leistungsanteil=float(kw.get('leistungsanteil', '0')), **kwargs)
            if achse:
                a['lern_achse'] = achse
                json.dump(a, open(os.path.join(ad, 'aufgabe.json'), 'w'), indent=1, ensure_ascii=False)
            return a
        except AU.GeneratorFehler as e:



            z = dict(aufgabe=seed, kandidat=None, designfamilie='erf_%06d' % seed,
                     klasse='generator', cfg={}, version=AU.VERSION + '+anker-legal1',
                     generatorfehler=True, zustand='generator_fehler', gueltig=False,
                     ungueltig_grund=str(e), note_v3=None, wert=2.0, stufe=stufe,
                     stufe_aufgabe=st, faktor=kwargs.get('faktor'), nicht_geroutet=True,
                     regel=dict(bewertungsziel=ziel), zeit=time.strftime('%Y-%m-%d %H:%M:%S'))
            if achse: z.update(lern_achse=dict(achse), lagen=achse['lagen'], bloecke=[])
            with LOCK:
                with open(kw['daten'], 'a') as f: f.write(json.dumps(z) + '\n')
            neu.append(z)
            print('Aufgabe %d Generatorfehler: %s' % (seed, e), flush=True)
            return None


    grenzenzeilen = zeilen
    if achse:
        grenzenzeilen = [z for z in zeilen if z.get('lagen') == achse['lagen'] and AX.art(z) == achse['gehaeuse']]
    fg = grenze_faktor(grenzenzeilen) if kw.get('start_grenze', '1') == '1' else None
    A = erzeugen(stufe=st, **({'faktor': fg} if fg else {}))
    if A is None:
        AX.melden(kw, seed, False, generatorfehler=True) if achse else stufe_melden(kw, False, seed, generatorfehler=True)
        if kw.get('behalten') != '1': shutil.rmtree(ad, ignore_errors=True)
        return neu
    if kw.get('lerngrenze') == '1':
        from lernen.lerneffekt import grenze_faktor as lern_grenze
        from lernen.ki.daten import familie, split_familie
        training = [z for z in zeilen if split_familie(familie(z)) == 'train']
        target = lern_grenze(training, klasse(A, ziel), A['faktor'])
        if target != A['faktor']:
            A = erzeugen(stufe=st, faktor=target)
            if A is None:
                if achse: AX.melden(kw, seed, False, generatorfehler=True)
                return neu
    faktor = A['faktor']; faktor0 = faktor
    from lernen.wertmodell import Dienst
    auswahl = kw.get('auswahl', 'alle')
    if auswahl not in ('alle', 'faust', 'wert', 'wert_auto', 'routbarkeit'): raise ValueError('auswahl=alle|faust|wert|wert_auto|routbarkeit')
    wd = kw.get('_wert_dienst') or Dienst([kw['daten']], kw.get('wert_modell', os.path.join(R, 'lernen/modell/wert')),
                neu=int(kw.get('wert_neu', '100')), cpu_s=int(kw.get('wert_cpu', '300')), routen=int(kw.get('wert_routen', '4')))
    if auswahl == 'routbarkeit':
        from lernen.ki.routbarkeit_dienst import Dienst as RoutDienst
        wd = RoutDienst(kw.get('routbarkeit_modell'), routen=int(kw.get('wert_routen', '4')), seed=seed)
    while True:
        gueltig_hier = False; unbekannt_hier = False; lockerung_hier = False
        queue = []; auswahl_info = None
        nroute = versuche if auswahl == 'alle' else (max(1, int(kw.get('wert_kandidaten', '16'))) if kw.get('wert_shadow', '0') == '1' else min(max(1, int(kw.get('wert_kandidaten', '16'))), wd.routen + int(kw.get('wert_erkunden', '1'))))
        for v in range(nroute):
            if os.path.exists(kw['stop']):
                achse_abschliessen(abgebrochen=True)
                return neu
            from analyse import korpus as KO
            KO.klassen_lauf(ad, os.path.join(ad, 'board.json'), os.path.join(ad, 'klassen.json'), os.path.join(ad, 'vorgaben.json'))
            def lerncfg(cfg):
                if kw.get('kosten_lernen', '1') != '0':
                    _, me = ME.auswahl(ME.lesen(kw.get('meister_cfg')), ME.klasse(A), exakt=True)
                    start=me['parameter'] if me else dict(ME.DEFAULT,form=cfg.get('form','45r'),
                          placer_stil=cfg.get('placer','alt'),router_stil=cfg.get('router','alt'))
                    cfg['kosten'] = ME.vorschlag(zeilen + neu, ME.klasse(A), rnd, start)
                    cfg['meister'] = '0'

                    lernkosten_fruehabbruch(cfg,rnd)
                if tempo:
                    cfg['lehrplan_tempo'] = '1'
                    if 'tempo_richter_modell' in kw: cfg['tempo_richter_modell'] = kw['tempo_richter_modell']
                    tempo_cache.seed(cfg)
                if achse or kw.get('bga_diagnose') == '1': cfg['bga_diagnose'] = True
                if PLK.aktiv(kw.get('placer_lockern')): cfg['placer_lockern'] = '1'
                for name in ('lernbudget', 'lern_cpu_max', 'lernpolicy', 'budget_vor_nachlauf', 'lerndaten', 'fruehabbruch', 'fruehabbruch_modell'):
                    if name in kw: cfg[name] = kw[name]
                if kw.get('paar_qualitaet', '0') == '1': cfg['paar_qualitaet'] = True
                return cfg
            if auswahl in ('faust', 'wert', 'wert_auto', 'routbarkeit'):
                if not queue:
                    if v > 0: break
                    pool = []
                    for _ in range(max(1, int(kw.get('wert_kandidaten', '16')))):
                        if os.path.exists(kw['stop']):
                            achse_abschliessen(abgebrochen=True)
                            return neu
                        if locker_cfg is not None:
                            cfg = dict(locker_cfg); locker_cfg = None
                        else:
                            with LOCK: cfg = wahl(zeilen + neu, klasse(A, ziel), rnd, ziel=ziel, bga_achse=bool(achse), stil_je_netzart=kw.get("stil_je_netzart") == "1", form_arme=kw.get("form_arme"), tempo_fehlerbuch=tempo_buch, zug_arm=kw.get("zug") == "1")
                        lerncfg(cfg)
                        if kw.get('r36_glatt', '0') == '1': cfg['r36_glatt'] = True
                        if kw.get('zonen_richter', '0') == '1': cfg['zonen_richter'] = True
                        if kw.get('zonen_richter_diagnose', '0') == '1': cfg['zonen_richter_diagnose'] = True
                        if kw.get('r36_korridor', '0') == '1': cfg['r36_korridor'] = True
                        if kw.get('r36_padgruppen', '0') == '1': cfg['r36_padgruppen'] = True
                        if kw.get('r36_k17', '0') == '1': cfg['r36_k17'] = True
                        if kw.get('k17_messen', '0') == '1': cfg['k17_messen'] = True
                        if kw.get('strom_form', '0') == '1': cfg['strom_form'] = True
                        if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
                        if kw.get('r19_nachlauf', '0') == '1': cfg['r19_nachlauf'] = True
                        if kw.get('k20_zonen', '0') == '1': cfg['k20_zonen'] = True
                        if kw.get('form'): cfg['form'] = kw['form']
                        if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['form'] = '45r'
                        if kw.get('legal_lp', '0') == '1': cfg['legal_lp'] = True
                        if kw.get('r39', '0') == '1': cfg['r39'] = True
                        if kw.get('r24', '0') == '1': cfg['r24'] = True
                        if kw.get('r39_reparatur', '0') == '1': cfg['r39_reparatur'] = True
                        cfg.update({key:kw[key] for key in ('restkonflikte','rest_cpu','rest_teile','rest_kand') if key in kw})
                        zp = tempo_versuch(ad, A, cfg, k, zeit, it, nur_platzieren=True)
                        kosten(zp)
                        if zp.get('merkmale') and not zp.get('code_geaendert') and not zp.get('fehler'):
                            pool.append(zp)
                        else:
                            zp.setdefault('gueltig', False)
                            zp['board'] = 'erf_%06d_f%.2f' % (seed, faktor)
                            zp['wert_vorbereitung_fehlgeschlagen'] = True
                            with LOCK:
                                with open(kw['daten'], 'a') as f: f.write(json.dumps(zp) + '\n')
                            neu.append(zp)
                        k += 1
                        if PLK.anfordern(zp):
                            lockerung_hier = True; locker_cfg = dict(cfg)
                            break
                    if auswahl == 'routbarkeit':
                        for zp in pool:
                            zp['routbarkeit_eingang'] = os.path.join(ad, 'k%d' % zp['kandidat'])
                            zp['routbarkeit_regeln'] = os.path.join(ad, 'regeln.json')
                    chosen, auswahl_info = wd.auswahl(pool, auswahl, erkunden=int(kw.get('wert_erkunden', '1')))
                    auswahl_info['shadow'] = kw.get('wert_shadow', '0') == '1'
                    if auswahl_info['shadow']: chosen = list(range(len(pool)))
                    auswahl_info['geroutet'] = chosen
                    ledger = dict(aufgabe=seed, faktor=faktor, stufe=stufe, pool=pool, auswahl=auswahl_info)
                    with LOCK:
                        with open(kw.get('wert_auswahl_daten', os.path.join(out, 'wert_auswahl.jsonl')), 'a') as f: f.write(json.dumps(ledger) + '\n')
                    queue = [pool[i] for i in chosen]
                    for i, zp in enumerate(pool):
                        if i in chosen: continue


                        rejected = dict(zp, gueltig=None, zustand='auswahl_verworfen',
                            nicht_geroutet=True, designfamilie='erf_%06d' % seed,
                            board='erf_%06d_f%.2f' % (seed, faktor), stufe=stufe,
                            cpu_total=zp.get('cpu_aufruf_s', zp.get('cpu_total')))
                        rejected.pop('merkmale', None)
                        if achse: rejected['lern_achse'] = dict(achse)
                        with LOCK:
                            with open(kw['daten'], 'a') as f: f.write(json.dumps(rejected) + '\n')
                        neu.append(rejected)
                    if kw.get('behalten') != '1':
                        for i, zp in enumerate(pool):
                            if i not in chosen: shutil.rmtree(os.path.join(ad, 'k%d' % zp['kandidat']), ignore_errors=True)
                    if not queue: break
                zp = queue.pop(0); cfg = zp['cfg']; kid = zp['kandidat']
                if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['form'] = '45r'
                if kw.get('r27_formtor', '0') == '1' or kw.get('r18', '0') == '1':
                    if kw.get('r27_formtor', '0') == '1': cfg['r27_formtor'] = True
                    p = Path(ad)/('k%d' % kid)/'vorbereitet.json'
                    prepared = json.loads(p.read_text()); prepared['cfg'] = cfg
                    p.write_text(json.dumps(prepared))
                z = tempo_versuch(ad, A, cfg, kid, zeit, it, vorbereitet=True, kandidaten=int(kw.get('kandidaten', '3')), kandidaten_auto='kandidaten' not in kw)
                kosten(z)
                z['wert_auswahl'] = auswahl_info
            else:
                if locker_cfg is not None:
                    cfg = dict(locker_cfg); locker_cfg = None
                else:
                    with LOCK: cfg = wahl(zeilen + neu, klasse(A, ziel), rnd, ziel=ziel, bga_achse=bool(achse), stil_je_netzart=kw.get("stil_je_netzart") == "1", form_arme=kw.get("form_arme"), tempo_fehlerbuch=tempo_buch, zug_arm=kw.get("zug") == "1")
                lerncfg(cfg)
                if kw.get('r36_glatt', '0') == '1': cfg['r36_glatt'] = True
                if kw.get('zonen_richter', '0') == '1': cfg['zonen_richter'] = True
                if kw.get('zonen_richter_diagnose', '0') == '1': cfg['zonen_richter_diagnose'] = True
                if kw.get('r36_korridor', '0') == '1': cfg['r36_korridor'] = True
                if kw.get('r36_padgruppen', '0') == '1': cfg['r36_padgruppen'] = True
                if kw.get('r36_k17', '0') == '1': cfg['r36_k17'] = True
                if kw.get('k17_messen', '0') == '1': cfg['k17_messen'] = True
                if kw.get('strom_form', '0') == '1': cfg['strom_form'] = True
                if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
                if kw.get('r19_nachlauf', '0') == '1': cfg['r19_nachlauf'] = True
                if kw.get('k20_zonen', '0') == '1': cfg['k20_zonen'] = True
                if kw.get('form'): cfg['form'] = kw['form']
                if kw.get('legal_lp', '0') == '1': cfg['legal_lp'] = True
                if kw.get('r39', '0') == '1': cfg['r39'] = True
                if kw.get('r24', '0') == '1': cfg['r24'] = True
                cfg.update({key:kw[key] for key in ('restkonflikte','rest_cpu','rest_teile','rest_kand') if key in kw})
                if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['form'] = '45r'
                if kw.get('r27_formtor', '0') == '1': cfg['r27_formtor'] = True
                if kw.get('r39_reparatur', '0') == '1': cfg['r39_reparatur'] = True
                kid = k; k += 1
                z = tempo_versuch(ad, A, cfg, kid, zeit, it, kandidaten=int(kw.get('kandidaten', '3')), kandidaten_auto='kandidaten' not in kw)
                kosten(z)
            if kw.get('anker_legal', '0') == '1': z['version'] = A['version']
            if achse: z['lern_achse'] = dict(achse)
            z['stufe'] = stufe; z['designfamilie'] = 'erf_%06d' % seed
            z['board'] = 'erf_%06d_f%.2f' % (seed, faktor)
            z['paar_aktiv'] = kw.get('richter_paar', '1') == '1'
            z['paar_qualitaet'] = kw.get('paar_qualitaet', '0') == '1'
            if kw.get('paar_modell'): z['paar_modell_pfad'] = kw['paar_modell']
            if z['paar_aktiv']:
                pd = paardienst(z.get('paar_modell_pfad')); z['paar_modell_pfad'] = str(pd.modell); pd.anwenden(z)
                if bester is not None and bester.get('faktor') == z.get('faktor'):
                    from lernen.richter_paar import referenzzeile
                    z['paar_referenz'] = referenzzeile(bester.get('zeile') or bester)
            if besser_als(dict(z, flaeche_mm2=A['flaeche_mm2']), bester):
                bester = dict(z, flaeche_mm2=A['flaeche_mm2'], kandidat=kid, zeile=copy.deepcopy(z))
            if ziel == 'minimax': z['belohnung'] = belohnung(z)
            z['wert'] = 2.0 if not z['gueltig'] else round(A['flaeche_mm2'] / (bester['flaeche_mm2'] if bester else A['flaeche_mm2']), 4)
            if ziel == 'minimax' and z['gueltig'] is None: z['wert'] = None
            z['board'] = 'erf_%06d_f%.2f' % (seed, faktor); z['schwierigkeit'] = schwierigkeit(A)
            if gal:
                try:
                    from lernen import galerie as G
                    with LOCK: G.aufnehmen(os.path.join(ad, 'k%d' % kid), dict(z, dauer=dict(quelle='selbstspiel %s' % cfg.get('form', '')), wert=None), gal)
                except Exception as e: print('  Galerie:', repr(e)[:120], flush=True)
            with LOCK:
                with open(kw['daten'], 'a') as f: f.write(json.dumps(z) + '\n')
            neu.append(z)
            if kw.get('wert_trainieren', '0') == '1': wd.pruefen()
            print('  Aufgabe %d k%d %s Faktor %.2f %5.0f mm2 %s Note %s  %4.0fs  %s' % (seed, kid, z['klasse'], faktor, A['flaeche_mm2'],
                  'unbekannt' if z['gueltig'] is None else 'gueltig ' if z['gueltig'] else 'ungueltig', z.get('note_v3'), z['s'],
                  '' if z['gueltig'] else 'konfl %s offen %s' % ((z.get('ergebnis') or {}).get('konflikte'), (z.get('ergebnis') or {}).get('offen'))), flush=True)
            if not kw.get('behalten') == '1' and not z['gueltig']: shutil.rmtree(os.path.join(ad, 'k%d' % kid), ignore_errors=True)
            if PLK.anfordern(z):
                lockerung_hier = True; locker_cfg = dict(cfg)
                break
            if z['gueltig'] is None: unbekannt_hier = True
            if z['gueltig']:
                if gueltig_hier and auswahl == 'alle': break
                gueltig_hier = True
        if lockerung_hier:


            if bester is None and stufe <= -2: break
            neuer_faktor = PLK.naechster_faktor(A, neu)
            if neuer_faktor is None: break
            naechste = erzeugen(faktor=neuer_faktor, stufe=st)
            if naechste is None: break
            A = PLK.uebernehmen(A, naechste); faktor = A['faktor']; stufe -= 1
            Path(ad, 'aufgabe.json').write_text(json.dumps(A, indent=1, ensure_ascii=False))
            continue
        if PLK.aktiv(kw.get('placer_lockern')) and A.get('gelockert') and gueltig_hier: break
        if not gueltig_hier:
            if unbekannt_hier: break
            if bester is None and stufe > -2:
                faktor = round(faktor * 1.12, 3); stufe -= 1
                naechste = erzeugen(faktor=faktor, stufe=st)
                if naechste is None: break
                if PLK.aktiv(kw.get('placer_lockern')) and A.get('gelockert'):
                    naechste = PLK.uebernehmen(A, naechste)
                A = naechste; continue
            break
        faktor = round(faktor * vd, 3); stufe = max(stufe, 0) + 1
        if faktor < 1.02: break
        naechste = erzeugen(faktor=faktor, stufe=st)
        if naechste is None: break
        A = naechste
    geschafft = any(z['gueltig'] and z['faktor'] <= faktor0 + 1e-9 for z in neu)
    if achse:
        achse_abschliessen()
    elif neu and neu[-1].get('generatorfehler') and (not tempo or st == overload_stufe): stufe_melden(kw, geschafft, seed, generatorfehler=True)
    elif (not tempo or st == overload_stufe) and (any(z.get('gueltig') is True for z in neu) or not any(z.get('gueltig') is None for z in neu if not z.get('nicht_geroutet'))):
        stufe_melden(kw, geschafft, seed)
    print('Aufgabe %d fertig: %s' % (seed, 'kleinste gueltige %.0f mm2 (Faktor %.2f)' % (bester['flaeche_mm2'], bester['flaeche_mm2'] / A['courtyard_mm2'])
                                     if bester else 'nie gueltig'), flush=True)
    if kw.get('behalten') != '1' and not bester: shutil.rmtree(ad, ignore_errors=True)
    return neu


def lehrer_spielen(out, seed, kw, zeilen, rnd):
    ""
    from lernen.lehrer import Lehrer, kontext, zufallsrezept
    zeit = int(kw.get('zeit', '300')); it = int(kw.get('iter', '40'))
    budget = float(kw.get('lehrer_cpu', str(zeit)))
    from lernen.cpubudget import Budget
    cpu = Budget(budget)
    kalibrierung = kw.get('lehrer_kalibrierung', '0') == '1'
    with LOCK:
        if '_lehrer' not in kw:
            kw['_lehrer'] = Lehrer(kw.get('lehrer_ordner', os.path.join(out, 'lehrer')), kontext(budget, it), seed)
        lehrer = kw['_lehrer']
        rezept = (dict(zufallsrezept(rnd, art=('funk', 'analog')[seed % 2], seed=seed), art='kalibrierung')
                  if kalibrierung else lehrer.vorschlag())
    ad = os.path.join(out, 'lehrer_%06d' % seed)
    args = {k: rezept[k] for k in ('seed', 'bloecke', 'stufe', 'achsen', 'familie')}
    try:
        A = aufgabe_erzeugen(ad, ziel=standardziel(explizit=kw.get('ziel'), kontext='selbstspiel'), **args)
    except ValueError as e:

        lehrer.verwerfen(rezept, str(e))
        print('Lehrer verwirft Vorschlag:', str(e), flush=True); return []
    neu = []
    for k in range(int(kw.get('versuche', '3'))):
        if os.path.exists(kw['stop']): break
        if k: cpu = Budget(budget)
        ziel = standardziel(explizit=kw.get('ziel'), kontext='selbstspiel')
        with LOCK: cfg = wahl(zeilen if kalibrierung else zeilen + neu, klasse(A, ziel), rnd, ziel=ziel, stil_je_netzart=kw.get("stil_je_netzart") == "1", form_arme=kw.get("form_arme"), zug_arm=kw.get("zug") == "1")
        if kw.get('zonen_richter', '0') == '1': cfg['zonen_richter'] = True
        if kw.get('zonen_richter_diagnose', '0') == '1': cfg['zonen_richter_diagnose'] = True
        if kw.get('r11_ir', '0') == '1': cfg['r11_ir'] = True
        if kw.get('r19_nachlauf', '0') == '1': cfg['r19_nachlauf'] = True
        if kw.get('k20_zonen', '0') == '1': cfg['k20_zonen'] = True
        if kw.get('form'): cfg['form'] = kw['form']
        if kw.get('r18', '0') == '1': cfg['r18'] = True; cfg['form'] = '45r'
        if kw.get('r39', '0') == '1': cfg['r39'] = True
        if kw.get('r24', '0') == '1': cfg['r24'] = True
        for name in ('fruehabbruch', 'fruehabbruch_modell'):
            if name in kw: cfg[name] = kw[name]
        if kw.get('r39_reparatur', '0') == '1': cfg['r39_reparatur'] = True
        cfg.update({key:kw[key] for key in ('restkonflikte','rest_cpu','rest_teile','rest_kand') if key in kw})
        z = versuch(ad, A, cfg, k, zeit, it, cpu_budget=cpu)
        z['regel'] = dict(z.get('regel') or {}, bewertungsziel=ziel)
        if ziel == 'minimax': z['regel']['minimax_profil'] = A['profil']
        z.update(designfamilie=A['familie'], abstammung=A['abstammung'], achsen=A['achsen'])
        melden = lehrer.kalibrieren if kalibrierung else lehrer.melden
        z = melden(rezept, ad, A, z, os.path.join(ad, 'k%d' % k), cpu_budget=cpu)
        daten = os.path.join(out, 'lehrer_kalibrierung.jsonl') if kalibrierung else kw['daten']
        with LOCK:
            with open(daten, 'a') as f: f.write(json.dumps(z) + '\n')
        neu.append(z)
    return neu


def optionen_pruefen(kw):
    form_arme_lesen(kw.get("form_arme"))
    if TT.aktiv(kw.get('lehrplan_tempo', '0')) and (kw.get('lehrer') == '1' or kw.get('rennbahn') == '1'):
        raise ValueError('lehrplan_tempo=1 braucht normales Generator-Selbstspiel')
    from lernen.zonen_richter_auswahl import an as richter_an
    richter_an({},kw.get('zonen_richter_diagnose',0))
    if richter_an({},kw.get('zonen_richter',0)) and (kw.get('r36','geometrisch')!='nativ' or kw.get('zonenplan','aus')!='aus' or kw.get('k20_zonen','0')=='1'):
        raise ValueError('zonen_richter=1 erfordert r36=nativ, zonenplan=aus, k20_zonen=0')
    if PLK.aktiv(kw.get('placer_lockern')) and (kw.get('lehrer') == '1' or kw.get('rennbahn') == '1'):
        raise ValueError('placer_lockern=1 braucht normales Generator-Selbstspiel')
    if kw.get('bga_achse') == '1' and (kw.get('lehrer') == '1' or float(kw.get('leistungsanteil', '0'))):
        raise ValueError('bga_achse=1: normales Selbstspiel ohne Lehrer/Leistungsboards')
    if kw.get('auswahl')=='routbarkeit' and kw.get('wert_trainieren','0')=='1':
        raise ValueError('Routbarkeit: routbarkeit_trainieren=1 statt wert_trainieren')


def eingefroren_dauer(out, args, kw, takt=300, behalten=3):
    ""
    import glob, subprocess
    basis = os.path.expanduser('~/Library/Caches/Autorouter/selbstspiel_code'); os.makedirs(out, exist_ok=True)
    stop = kw.get('stop', os.path.join(R, 'lernen/STOP')); daten = os.path.abspath(kw.get('daten', os.path.join(R, 'lernen/daten/selbstspiel.jsonl')))
    rest = [a for a in args if a.split('=', 1)[0] not in ('eingefroren', 'stop', 'daten', 'seed0', 'wert_modell')]
    kinder = []

    def kopf():
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True).stdout.strip()

    def starten(sha):
        code = os.path.join(basis, sha[:12])
        if not os.path.exists(os.path.join(code, 'lernen', 'selbstspiel.py')):
            os.makedirs(code, exist_ok=True)
            archive = subprocess.check_output(['git', '-C', R, 'archive', sha])
            subprocess.run(['tar', '-x', '-C', code], input=archive, check=True)
            if not os.path.exists(os.path.join(code, '.venv')): os.symlink(os.path.join(R, '.venv'), os.path.join(code, '.venv'))
        nrn = [int(d.split('_')[1]) for d in os.listdir(out) if d.startswith('aufgabe_') and d.split('_')[1].isdigit()] if os.path.exists(out) else []
        zs = [json.loads(l).get('aufgabe') or 0 for l in open(daten)] if os.path.exists(daten) else []
        seed0 = max(nrn + zs + [0]) + int(kw.get('parallel', '4')) + 2
        gstop = os.path.join(code, 'STOP_generation')
        if os.path.exists(gstop): os.remove(gstop)
        cmd = [os.path.join(R, '.venv', 'bin', 'python'), os.path.join(code, 'lernen', 'selbstspiel.py'), out] + rest + [
            'daten=' + daten, 'stop=' + gstop, 'seed0=%d' % seed0, 'wert_modell=' + os.path.join(R, 'lernen/modell/wert')]
        log = open(os.path.join(out, 'generation_%s.log' % sha[:12]), 'a')
        p = subprocess.Popen(cmd, cwd=code, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONHASHSEED='0',
                             AUTOROUTER_MEISTER=kw.get('meister_cfg') or str(ME.datei())))
        print('%s Generation %s gestartet (pid %d, ab Aufgabe %d)' % (time.strftime('%d.%m. %H:%M'), sha[:12], p.pid, seed0), flush=True)
        kinder.append((p, gstop, code))

    aktuell = kopf(); starten(aktuell); zuletzt_aufgeraeumt = 0.0
    while True:
        if time.time() - zuletzt_aufgeraeumt > 3600:
            subprocess.Popen(['nice', '-n', '15', os.path.join(R, '.venv', 'bin', 'python'), os.path.join(R, 'gemeinsam', 'aufraeumen.py')],
                             cwd=R, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); zuletzt_aufgeraeumt = time.time()
            if kw.get('zug', '0') == '1' and ZUG_DATEN.exists():
                subprocess.Popen(['nice', '-n', '15', os.path.join(R, '.venv', 'bin', 'python'), os.path.join(R, 'lernen', 'zugwahl.py'),
                                  str(ZUG_DATEN), str(ZUG_MODELL)], cwd=R, stdout=open(str(ZUG_MODELL) + '.log', 'a'),
                                 stderr=subprocess.STDOUT)
        t0 = time.time()
        while time.time() - t0 < takt and not os.path.exists(stop) and kinder[-1][0].poll() is None: time.sleep(10)
        if os.path.exists(stop):
            for p, gs, _ in kinder: open(gs, 'w').close()
            for p, _, _ in kinder: p.wait()
            print('Selbstspiel (eingefroren) beendet.', flush=True); return
        neu = kopf()
        if kinder[-1][0].poll() is not None and neu == aktuell and kw.get('aufgaben'):
            for p, _, _ in kinder: p.wait()
            print('Selbstspiel (eingefroren) fertig.', flush=True); return
        if neu != aktuell or kinder[-1][0].poll() is not None:
            open(kinder[-1][1], 'w').close()
            time.sleep(2); aktuell = neu; starten(neu)
        kinder[:] = [k for k in kinder[:-1] if k[0].poll() is None] + kinder[-1:]
        lebend = {k[2] for k in kinder}
        alt = sorted((d for d in glob.glob(os.path.join(basis, '*')) if d not in lebend), key=os.path.getmtime)[:-behalten]
        for d in alt: shutil.rmtree(d, ignore_errors=True)


def main():
    out = sys.argv[1]; kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    form_arme_lesen(kw.get('form_arme'))
    if kw.get('eingefroren') == '1': return eingefroren_dauer(out, sys.argv[2:], kw)
    optionen_pruefen(kw)
    kw.setdefault('daten', os.path.join(R, 'lernen/daten/selbstspiel.jsonl')); kw.setdefault('stop', os.path.join(R, 'lernen/STOP'))
    if kw.get('lehrer', '0') == '1' and 'daten' not in dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a):
        kw['daten'] = os.path.join(out, 'lehrer_versuche.jsonl')
    os.makedirs(out, exist_ok=True); os.makedirs(os.path.dirname(kw['daten']), exist_ok=True)
    zeilen = [json.loads(l) for l in open(kw['daten'])] if os.path.exists(kw['daten']) else []
    seed0 = int(kw.get('seed0', str(max([z['aufgabe'] for z in zeilen] or [0]) + 1)))
    if kw.get('bga_achse') == '1' and 'seed0' not in kw:
        from lernen import bga_achsen as AX
        with AX.zustand(kw) as state:
            seed0 = max(seed0, max([int(s) for s in state['aufgaben']] + [state.get('archiv_bis', -1)]) + 1)
    n = int(kw.get('aufgaben', '0')); par = int(kw.get('parallel', '4'))
    print('Selbstspiel: ab Aufgabe %d, %s Aufgaben, %d parallel, %d alte Zeilen' % (seed0, n or 'endlos', par, len(zeilen)), flush=True)
    from lernen.wertmodell import Dienst
    kw['_wert_dienst'] = Dienst([kw['daten']], kw.get('wert_modell', os.path.join(R, 'lernen/modell/wert')),
                                neu=int(kw.get('wert_neu', '100')), cpu_s=int(kw.get('wert_cpu', '300')), routen=int(kw.get('wert_routen', '4')))
    zaehler = [seed0]

    def naechste():
        with LOCK:
            s = zaehler[0]; zaehler[0] += 1; return s

    def arbeiter(_):
        while not os.path.exists(kw['stop']):
            s = naechste()
            if n and s >= seed0 + n: return
            try:
                neu = aufgabe_spielen(out, s, kw, zeilen, random.Random(s))
                with LOCK:
                    zeilen.extend(z for z in neu if z.get('rezept', {}).get('art') != 'kalibrierung')
            except Exception as e: print('Aufgabe %d Fehler: %r' % (s, e), flush=True)
    with ThreadPoolExecutor(par) as ex: list(ex.map(arbeiter, range(par)))
    print('Selbstspiel beendet.', flush=True)


if __name__ == '__main__':
    main()
