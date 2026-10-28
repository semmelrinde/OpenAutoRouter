""
import json, math, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DIMENSIONEN = ('kreuz', 'trenn', 'via_kosten', 'via_f', 'stil')
PLACER_DIM = ('kreuz', 'trenn', 'via_kosten')





VOREINSTELLUNG = dict(
    pruefstand=dict(kreuz=0.0, trenn=3.0, via_kosten=0.6, via_f=1.0, stil='standard'),
    ablauf=dict(kreuz=2.0, trenn=3.0, via_kosten=0.6, via_f=1.0, stil='standard'))
PFAD = 'lernen/modell/lenkung.json'
N_MIN = 10
Z = 1.645
ABSTAND = 0.02
SIGMA, N0, PRIOR0 = 0.5, 1.0, 1.2

def klasse(mm, lagen=2):
    ""
    mm = mm or {}
    d = mm.get('pads_je_cm2') or 0.0; n = mm.get('n_teile') or 0
    dk = 'd0' if d < 1.5 else 'd1' if d < 3.0 else 'd2'
    tk = 't0' if n < 25 else 't1' if n < 60 else 't2'
    return 'L%d_%s_%s' % (int(mm.get('lagen') or lagen or 2), dk, tk)


def _arm(a):
    ""
    try: return '%g' % float(a)
    except (TypeError, ValueError): return str(a)


def _wert(dim, a, vorbild):
    ""
    return float(a) if isinstance(vorbild, (int, float)) else str(a)


def pfad_aufloesen(pfad):
    if not pfad: pfad = PFAD
    if os.path.isabs(pfad) or os.path.exists(pfad): return os.path.abspath(pfad)
    return os.path.join(R, pfad)


def lenkung_laden(pfad=PFAD):
    p = pfad_aufloesen(pfad)
    try:
        with open(p) as f: return json.load(f), p
    except Exception: return None, p


def posterior(L, k, dim, a):
    ""
    st = L.get('statistik') or {}
    ng, sg = st.get('*|%s|%s' % (dim, _arm(a)), [0, 0.0])
    prior = sg / ng if ng else PRIOR0
    n, s = st.get('%s|%s|%s' % (k, dim, _arm(a)), [0, 0.0])
    return (s + N0 * prior) / (n + N0), SIGMA / math.sqrt(n + N0), int(n)


def einstellungen(merkmale, lagen=2, pfad=PFAD, n_min=N_MIN, vorgabe='pruefstand', fest=None, z=Z, abstand=ABSTAND):
    ""
    V = dict(VOREINSTELLUNG[vorgabe] if isinstance(vorgabe, str) else vorgabe)
    fest = {d: v for d, v in (fest or {}).items() if d in DIMENSIONEN and v not in (None, '')}
    k = klasse(merkmale, lagen)
    L, p = lenkung_laden(pfad)
    out = dict(klasse=k, werte={}, herkunft={}, grund={}, lenkung=p, lenkung_zeit=(L or {}).get('zeit'), n_min=n_min)
    beste = ((L or {}).get('beste') or {}).get(k) or {}
    for d in DIMENSIONEN:
        v0 = V[d]
        if d in fest:
            out['werte'][d] = _wert(d, fest[d], v0); out['herkunft'][d] = 'ausdruecklich'; out['grund'][d] = 'ausdruecklich gesetzt'; continue
        out['werte'][d] = v0; out['herkunft'][d] = 'voreinstellung'
        if L is None:
            out['grund'][d] = 'lenkung.json fehlt'; continue
        if d not in beste:
            out['grund'][d] = 'keine Daten fuer Klasse %s' % k; continue
        if L.get('format') == 'lenkung.v3':


            ab = beste[d][1]
            c = ((L.get('kontraste') or {}).get(k, {}).get(d, {}).get('%s|%s' % (_arm(ab), _arm(v0))))
            out['grund'][d] = 'entzerrt, aber keine Tor-Freigabe fuer den Ablauf'
            if not L.get('freigegeben') or not c or not c.get('ci'): continue
            from lernen.sammeln import code_stand
            if L.get('code') != code_stand():
                out['grund'][d] = 'anderer Code-Stand'; continue
            if c['n'] < n_min or c['n_bezug'] < n_min or c['ci'][1] > -abstand:
                out['grund'][d] = 'Kontrast nicht sicher vorn oder zu wenig Daten'; continue
            out['werte'][d] = _wert(d, ab, v0); out['herkunft'][d] = 'gelernt'
            out['grund'][d] = 'entzerrter Kontrast %.3f, 95 %% %s' % (c['diff'], c['ci'])
            continue
        mb, ab, nb = beste[d][0], beste[d][1], int(beste[d][2])
        if _arm(ab) == _arm(v0):
            out['grund'][d] = 'Voreinstellung %s ist der beste Arm (%.3f, n %d)' % (_arm(v0), mb, nb); continue
        if nb < n_min:
            out['grund'][d] = 'bester Arm %s (%.3f) hat nur n %d < %d' % (_arm(ab), mb, nb, n_min); continue
        mb2, sb, nb2 = posterior(L, k, d, ab)
        m0, s0, n0 = posterior(L, k, d, v0)
        diff = m0 - mb2; noetig = max(abstand, z * math.hypot(sb, s0))
        txt = '%s %.3f (n %d) gegen Voreinstellung %s %.3f (n %d): Vorsprung %.3f, noetig %.3f' % (_arm(ab), mb2, nb2, _arm(v0), m0, n0, diff, noetig)
        if diff >= noetig:
            out['werte'][d] = _wert(d, ab, v0); out['herkunft'][d] = 'gelernt'; out['grund'][d] = txt
        else:
            out['grund'][d] = 'nicht klar vorn: ' + txt
    return out


def placer_optionen(E):
    ""
    return ['%s=%g' % (d, E['werte'][d]) for d in PLACER_DIM if E['herkunft'][d] == 'gelernt']


def kurz(E):
    ""
    return '%s: ' % E['klasse'] + ' '.join('%s=%s (%s)' % (d, _arm(E['werte'][d]), E['herkunft'][d]) for d in DIMENSIONEN)


def main():
    pos = [a for a in sys.argv[1:] if '=' not in a]; kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    if len(pos) < 2:
        print(__doc__); sys.exit(2)
    sys.path.insert(0, R)
    from lernen.merkmale import merkmale
    B = json.load(open(pos[0])); K = json.load(open(pos[1])); RG = json.load(open(pos[2])) if len(pos) > 2 else None
    lagen = int(kw.get('lagen') or B.get('lagen') or 2)
    mm = merkmale(B, K, RG)
    if kw.get('lagen'): mm = dict(mm, lagen=lagen)
    E = einstellungen(mm, lagen, pfad=kw.get('lenkung', PFAD), n_min=int(kw.get('n_min', N_MIN)), vorgabe=kw.get('vorgabe', 'pruefstand'))
    fmt = kw.get('format', 'text')
    if fmt == 'json':
        print(json.dumps(E, indent=1)); return
    if fmt == 'sh':
        print('E_KLASSE=%s' % E['klasse'])
        for d in DIMENSIONEN: print('E_%s=%s' % (d.upper(), _arm(E['werte'][d])))
        for d in DIMENSIONEN: print('E_HERKUNFT_%s=%s' % (d.upper(), E['herkunft'][d]))
        for d in DIMENSIONEN: print('# %s: %s (%s)' % (d, E['herkunft'][d], E['grund'][d]), file=sys.stderr)
        return
    print('Klasse %s, lenkung.json vom %s' % (E['klasse'], E['lenkung_zeit']))
    for d in DIMENSIONEN: print('  %-10s %-9s %-14s %s' % (d, _arm(E['werte'][d]), E['herkunft'][d], E['grund'][d]))


if __name__ == '__main__':
    main()
