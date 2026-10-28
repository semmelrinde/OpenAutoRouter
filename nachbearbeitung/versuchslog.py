""
import copy, hashlib, json, math, os, platform, sys, uuid
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

SCHEMA = 'versuch.v1'
NOTE_KEY = 'note'
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CODE = ['router/route_pf.py', 'router/astar_nb.py', 'analyse/klassen.py', 'analyse/bewertung.py', 'nachbearbeitung/nachbessern.py',
        'nachbearbeitung/zugregeln.py', 'analyse/engstellen.py', 'analyse/metriken.py', 'analyse/metriken_gewichte.json',
        'analyse/minimax.py', 'analyse/strom_kette.py', 'analyse/kupfermodell.py', 'analyse/regelkennzahlen.py',
        'analyse/standard.py', 'analyse/standard.json', 'analyse/topologie.py', 'analyse/kopplung.py', 'gemeinsam/wanderweite.py',
        'nachbearbeitung/versuchslog.py', 'gemeinsam/verschieben.py', 'gemeinsam/fluss.py', 'gemeinsam/teile.py', 'gemeinsam/versetzen.py',
        'codex/ir_drop/solver.py', 'codex/zonen_ir/fuellung.py', 'codex/zonen_ir/umrisse.py',
        'codex/zonen_ir/solver_fast.py', 'codex/zonen_ir/integration.py']

CODE = sorted(set(CODE + ['lernen/kupferbalken.py', 'analyse/standard.json'] +
    [os.path.relpath(os.path.join(root, f), R)
     for folder in ('analyse', 'gemeinsam', 'codex/zonen_ir', 'codex/ir_drop')
     for root, _, files in os.walk(os.path.join(R, folder)) for f in files if f.endswith('.py')]))


def sha_datei(path):
    try:
        with open(path, 'rb') as f: return hashlib.sha256(f.read()).hexdigest()
    except OSError: return None


def sha_obj(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def code_hashes():
    return {c: sha_datei(os.path.join(R, c)) for c in CODE}


def umgebung():
    import numpy, scipy, shapely
    return dict(python=sys.version.split()[0], numpy=numpy.__version__, scipy=scipy.__version__, shapely=shapely.__version__,
                plattform=platform.platform(), cpu=platform.machine(), pythonhashseed=os.environ.get('PYTHONHASHSEED'))


def _endlich(x):
    return x if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else None


def score(E):
    ""
    if E is None: return None
    return copy.deepcopy(dict(note=_endlich(E.get(NOTE_KEY)), note_key=NOTE_KEY, note_v3=_endlich(E.get('note_v3')), note_v3_version=E.get('note_v3_version'),
                minimax=E.get('minimax'), minimax_version=E.get('minimax_version'), minimax_profil=E.get('minimax_profil'),
                minimax_rang=E.get('minimax_rang'), minimax_summe=E.get('minimax_summe'), engpass=E.get('engpass'),
                minimax_quellen=E.get('minimax_quellen'), minimax_strompfade=E.get('minimax_strompfade'),
                regeln=E.get('regeln'), regeltor=E.get('regeltor'), bewertungsziel=E.get('bewertungsziel'),
                minimax_gueltig=E.get('minimax_gueltig'), minimax_status=E.get('minimax_status'),
                minimax_boardvertrag=E.get('minimax_boardvertrag'),
                minimax_vollstaendig=E.get('minimax_vollstaendig'), sicherheit=E.get('sicherheit'),
                zug_regeln=E.get('zug_regeln'), note_v1=E.get('note'), note_v2=_endlich(E.get('note_v2_raw')),
                note_raw=_endlich(E.get('note_raw')), gueltig=E.get('gueltig'), offen=E.get('offen'),
                konflikte=E.get('konflikte'), unverbunden=len(E['unverbunden']) if E.get('unverbunden') is not None else None,
                vias=E.get('vias'), umweg_mm=E.get('umweg_summe'), defizit_mm2=E.get('defizit'), verengung_mm2=E.get('verengung'),
                summanden={k: _endlich(v) for k, v in (E.get('summanden') or {}).items()} or None,
                bewertung_version=E.get('bewertung_version'),
                ir_verlust_mW=_endlich((E.get('ir') or {}).get('verlust_mW')), ir_strommodus=(E.get('ir') or {}).get('strommodus'),
                ir_zonen_modell=(E.get('ir') or {}).get('zonen_modell')))


def delta(a, b):
    ""
    if a is None or b is None: return None
    def d(x, y): return None if x is None or y is None else y - x
    out = {k: d(a.get(k), b.get(k)) for k in ('note', 'note_v1', 'note_v2', 'note_v3', 'note_raw', 'offen', 'konflikte', 'vias', 'umweg_mm', 'defizit_mm2', 'verengung_mm2', 'ir_verlust_mW')}
    sa, sb = a.get('summanden') or {}, b.get('summanden') or {}
    out['summanden'] = {k: d(sa.get(k), sb.get(k)) for k in sorted(set(sa) | set(sb))} or None
    return out


def label(vor, nach, status, mindest):
    ""
    if status != 'evaluated' or vor is None or nach is None or vor.get('note') is None or nach.get('note') is None:
        return dict(policy='annahmeregel-v1-' + NOTE_KEY, success=None, gain_if_valid=None, relation=None)
    ok = bool(nach['gueltig']) and (not vor['gueltig'] or nach['note'] < vor['note'] - mindest)
    gain = (vor['note'] - nach['note']) if (vor['gueltig'] and nach['gueltig']) else None
    rv = (0 if vor['gueltig'] else 1, (vor['offen'] or 0) + (vor['unverbunden'] or 0), vor['konflikte'] or 0, vor['note'])
    rn = (0 if nach['gueltig'] else 1, (nach['offen'] or 0) + (nach['unverbunden'] or 0), nach['konflikte'] or 0, nach['note'])
    return dict(policy='annahmeregel-v1-' + NOTE_KEY, success=ok, gain_if_valid=gain, relation='besser' if rn < rv else ('gleich' if rn == rv else 'schlechter'))


def _outline(B):
    try: return unary_union([Polygon(o) for o in B['outline'] if len(o) >= 3]).buffer(0)
    except Exception: return None


def lokale_belegung(B, refs, halb=(2.0, 5.0)):
    ""
    fps = {f['ref']: f for f in B['fps']}; brett = _outline(B)
    cy = {}
    for c in B.get('courtyards', []):
        for q in c.get('polys', []):
            if len(q) >= 3: cy.setdefault((c['side'], c['ref']), []).append(Polygon(q).buffer(0))
    out = {}
    for r in refs:
        f = fps.get(r)
        if not f: continue
        je = {}
        for h in halb:
            fen = box(f['x'] - h, f['y'] - h, f['x'] + h, f['y'] + h)
            if brett is not None: fen = fen.intersection(brett)
            if fen.area <= 1e-9: je['%g' % h] = None; continue
            werte = {}
            for side in ('F', 'B'):
                fremd = [g for (s, q), gs in cy.items() if s == side and q not in refs for g in gs if g.intersects(fen)]
                werte[side] = round(unary_union(fremd).intersection(fen).area / fen.area, 4) if fremd else 0.0
            nachbarn = sum(1 for q in fps.values() if q['ref'] not in refs and abs(q['x'] - f['x']) <= h and abs(q['y'] - f['y']) <= h)
            je['%g' % h] = dict(courtyard_anteil=werte, nachbarn=nachbarn, fenster_mm2=round(fen.area, 3))
        out[r] = je
    return dict(version='local-v1-mech', teile=out)


def umfeld(B, Rt, refs, halb=(1.5, 4.0)):
    ""
    if Rt is None: return None
    fps = {f['ref']: f for f in B['fps']}
    netze = {p['net'] for p in B['pads'] if p['ref'] in refs and p['net']}
    konf = Rt.get('conflicts') or {}
    knetze = {str(u).split('#')[0].split('|')[0] for u in (konf if isinstance(konf, (dict, list)) else [])}
    segs = [(t['net'], (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, math.dist(a, b)) for t in Rt.get('tracks', []) for a, b in zip(t['pts'], t['pts'][1:])]
    vias = [(v['net'], v['x'], v['y']) for v in Rt.get('vias', [])]
    out = {}
    for r in refs:
        f = fps.get(r)
        if not f: continue
        je = {}
        for h in halb:
            inb = lambda x, y: abs(x - f['x']) <= h and abs(y - f['y']) <= h
            je['%g' % h] = dict(kupfer_mm=round(sum(L for n, x, y, L in segs if inb(x, y)), 3),
                                kupfer_fremd_mm=round(sum(L for n, x, y, L in segs if inb(x, y) and n not in netze), 3),
                                vias=sum(1 for n, x, y in vias if inb(x, y)))
        dk = [math.hypot(x - f['x'], y - f['y']) for n, x, y, L in segs if n in knetze] + [math.hypot(x - f['x'], y - f['y']) for n, x, y in vias if n in knetze]
        out[r] = dict(fenster=je, konflikt_netze_am_teil=len(netze & knetze), konflikt_abstand_mm=round(min(dk), 3) if dk else None)
    return dict(version='umfeld-v1', konflikte_gesamt=len(konf), teile=out)


def vorher(B, K, k, E0, Rt=None, ww=None):
    ""
    fps = {f['ref']: f for f in B['fps']}; NETZ = K['netze']; TEILE = K['teile']; GND = set(K['gnd'])
    refs = list(k['refs'])
    pads = {r: [p for p in B['pads'] if p['ref'] == r] for r in refs}
    teile = []
    for r in refs:
        f = fps[r]; t = TEILE.get(r, {})
        teile.append(dict(ref=r, rolle=t.get('rolle'), funktion=t.get('funktion'), entkopplung=t.get('entkopplung'), fest=t.get('fest'), insel=t.get('insel'),
                          fp=f.get('fp'), wert=f.get('value'), n_pads=len({p['num'] for p in pads[r]}), seite=f['side'],
                          pose=dict(x=f['x'], y=f['y'], rot=f.get('rot', 0.0))))
    netze = {}
    for r in refs:
        for p in pads[r]:
            n = p['net']
            if not n: continue
            netze.setdefault(n, dict(grund='part_pad', art='gnd' if n in GND else NETZ.get(n, {}).get('art'),
                                     rang=NETZ.get(n, {}).get('rang'), w=NETZ.get(n, {}).get('w'), wmin=NETZ.get(n, {}).get('wmin'),
                                     gnd=n in GND, pads=sum(1 for q in B['pads'] if q['net'] == n),
                                     fluss_einheit=(NETZ.get(n, {}).get('fluss') or {}).get('einheit')))
    ziel = k.get('netz') or k.get('ziel_netz')
    if ziel and ziel not in netze:
        n = ziel
        netze[n] = dict(grund='explicit_reroute', art=NETZ.get(n, {}).get('art'), rang=NETZ.get(n, {}).get('rang'),
                        w=NETZ.get(n, {}).get('w'), wmin=NETZ.get(n, {}).get('wmin'), gnd=n in GND,
                        pads=sum(1 for q in B['pads'] if q['net'] == n), fluss_einheit=(NETZ.get(n, {}).get('fluss') or {}).get('einheit'))
    grund = k.get('quelle') or 'unbekannt'
    if grund == 'engstelle' and 'onflikt' in k.get('grund', ''): grund = 'konflikt'
    return dict(fokus=refs[0] if refs else None, teile=teile,
                move=dict(type=k['art'], members={r: dict(dx_mm=v[0], dy_mm=v[1], rotation_deg=v[2]) for r, v in k['refs'].items()},
                          sides=k.get('seiten'), route_moves=k.get('route_moves'), target_net=k.get('ziel_netz')),
                reason=dict(type=grund, text=k.get('grund'), trigger_net_name=k.get('netz'), gewinn_schaetzung=k.get('gewinn')),
                local_occupancy=lokale_belegung(B, refs), affected_nets=[dict(name=n, **v) for n, v in sorted(netze.items())],
                score=score(E0), preis=_endlich(k.get('preis')), umfeld=umfeld(B, Rt, refs),
                wanderweite={r: dict(schluessel=w.get('schluessel'), grund=_endlich(w.get('grund')), max=_endlich(w.get('max')),
                                     ganz=w.get('max') == math.inf, wichtigkeit=_endlich(w.get('wichtigkeit')), drehen=w.get('drehen'))
                             for r, w in sorted((ww or {}).items())} or None)


class Versuchslog:
    def __init__(self, pfad, meta):
        self.f = open(pfad, 'a'); self.meta = meta

    def schreibe(self, rec):
        self.f.write(json.dumps(rec, ensure_ascii=False, allow_nan=False) + '\n'); self.f.flush()

    def runde_fertig(self):
        try: os.fsync(self.f.fileno())
        except OSError: pass

    def versuch(self, *, runde, index, decision_id, phase, parent, original, k, before, state, nach_E, status, fehler,
                wall_s, route_s, router_opt, accepted, mindest, kandidatenboard_sha, ergebnis_sha, policy=None, model_id=None,
                comparison=None, control_id=None, control_comparison=None, attempt_id=None):
        vor_s = before['score']; nach_s = score(nach_E)
        lab = label(vor_s, nach_s, status, mindest)
        if comparison is not None:
            lab.update(policy='annahmeregel-v3-gleich-regeln-preis-null', success=bool(comparison['besser']),
                       relation='besser' if comparison['besser'] else 'nicht_besser')
            if phase != 'control':
                lab['success'] = bool(comparison['besser'] and control_comparison['besser']) if control_comparison else None
            lab['gain_vs_control'] = -control_comparison['delta'] if control_comparison and control_comparison.get('delta') is not None else None
            lab['gain_vs_basis'] = -comparison['delta'] if comparison.get('delta') is not None else None
            if phase != 'control': lab['gain_if_valid'] = lab['gain_vs_control']
        if phase == 'control': lab['success'] = None
        rec = dict(schema_version=SCHEMA, record_origin='native',
                   ids=dict(attempt_id=attempt_id or str(uuid.uuid4()), run_id=self.meta['run_id'], board_id=self.meta['board_id'],
                            board_family_id=self.meta['familie'], round=runde, candidate_index=index, decision_id=decision_id,
                            parent_attempt_id=parent, original_attempt_id=original),
                   provenance=dict(snapshots=dict(state, kandidatenboard=kandidatenboard_sha, ergebnisrouten=ergebnis_sha),
                                   historical_code_sha256=self.meta['code'], router_seed=None, pythonhashseed='0',
                                   router_options=router_opt, environment=self.meta['umgebung'], started_at_utc=self.meta['start_utc']),
                   decision=dict(phase=phase, mindest=mindest, selection_policy=policy or 'heuristik-gewinn-pro-aufwand-v1', selection_probability=None, model_id=model_id),
                   before=before,
                   result=dict(status=status, fehler=fehler, score=nach_s, delta=delta(vor_s, nach_s),
                               wall_s=round(wall_s, 3) if wall_s is not None else None, route_s=round(route_s, 3) if route_s is not None else None,
                               comparison=comparison, control_attempt_id=control_id, control_comparison=control_comparison),
                   label=lab, selection=dict(accepted=accepted),
                   quality=dict(fehlend=['local_occupancy.kupfer_je_lage', 'pending_journal'], feature_version='nachbessern-v3'))
        self.schreibe(rec)
        return rec['ids']['attempt_id']
