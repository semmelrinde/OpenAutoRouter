""
import math, sys, json

GANZ = math.inf



TABELLE = {


    'chip':                 (0.0, 2.0, 100.0, False, 'P22 Hauptteil: wandert nur wenig, teuer (Nutzer 06.10. 19:45; Preis aus Codex P3)'),
    'stecker':              (0.0, 0.0, 0.0, False, 'P22/P16 Mechanik, fest'),
    'mechanik':             (0.0, 0.0, 0.0, False, 'P22/P16 Mechanik, fest'),

    'lose':                 (0.0, GANZ, 0.5, True, 'lose in erzeugter Aufgabe (klassen.lose_stecker), frei platzieren'),
    'quarz':                (0.0, 0.0, 0.0, False, 'P20(3) Quarz bleibt an seinem Chip'),

    ('entkopplung_klein', 'leistung'): (0.3, 0.6, 80.0, True, 'P8/P20(2) heisse Schleife des Schaltreglers'),
    ('entkopplung_klein', 'digital'):  (0.5, 1.0, 50.0, True, 'P19/P20(2) 100 nF am schnellen Chip'),
    ('entkopplung_klein', 'gemischt'): (0.5, 1.0, 50.0, True, 'P19/P20(2) 100 nF am schnellen Chip'),
    ('entkopplung_klein', 'analog'):   (1.5, 2.5, 15.0, True, 'P19 Nutzer 06.10.: OPV-Versorgungs-C 1-2 mm, mehr nur bei viel besserem Routing'),
    'entkopplung_klein':               (1.0, 2.0, 25.0, True, 'P19/P20(2) kleine Entkopplung, Chip mittelschnell'),
    ('entkopplung_mittel', 'leistung'): (0.5, 1.0, 40.0, True, 'P8 Ein-/Ausgangs-C des Schaltreglers'),
    'entkopplung_mittel':              (2.0, 3.0, 8.0, True, 'P20(5) mittlere C (~1 uF)'),
    ('entkopplung_gross', 'leistung'): (0.5, 1.5, 25.0, True, 'P8/P14 Ein-/Ausgangs-C des Schaltreglers (Schleife)'),
    'entkopplung_gross':               (3.0, 5.0, 4.0, True, 'P19/P20(6) grosse C (10 uF+) darf rücken'),
    'bootstrap':            (0.3, 0.5, 60.0, True, 'P8 Bootstrap-C am Schaltknoten, Schleife klein'),
    'spule_sw':             (0.3, 0.5, 60.0, False, 'P8/P20 Drossel am Schaltknoten; nicht drehen (Wicklungsanfang)'),
    'quarz_last':           (0.3, 0.5, 60.0, True, 'P20(3) Lastkondensatoren, Bahnen extrem kurz (ST AN2867)'),
    'serien_r':             (0.5, 1.0, 30.0, True, 'P20(4) Serienwiderstand auf schneller Leitung'),

    'rueckkopplung':        (1.0, 2.0, 10.0, True, 'FB-Knoten empfindlich, Teiler nah am Pin'),
    'rueckkopplung_c':      (1.0, 2.0, 10.0, True, 'FB-/Kompensations-C nah am Pin'),
    'einstell_r':           (1.0, 2.0, 10.0, True, 'RT/ISET-Knoten empfindlich'),
    'shunt':                (1.0, 2.0, 10.0, True, 'K6/R13 Messwiderstand im Strompfad (IR-Schutz prueft zusaetzlich)'),

    'teiler':               (3.0, GANZ, 1.0, True, 'P20 langsamer Teiler, darf wandern'),
    'rc_glied':             (3.0, GANZ, 1.0, True, 'P20 "22 nF im RC-Glied kann egal sein"'),
    'sonstig':              (2.0, GANZ, 2.0, True, 'unbekannte Funktion: mittelweit, maessiger Preis'),
    'pullup':               (GANZ, GANZ, 0.0, True, 'P20(7)/P19 Nutzer 06.10.: Pull-up irgendwo, Hauptsache Platz'),
    'pulldown':             (GANZ, GANZ, 0.0, True, 'P20(7)/P19 Nutzer 06.10.: Pull-down irgendwo, Hauptsache Platz'),
}
OHNE_FUNKTION = (2.0, GANZ, 2.0, True, 'klassen.json ohne funktion: wie sonstig')


def _inselart(t, inseln):
    s = t.get('stuetzt') or {}
    I = inseln.get(s.get('ref')) or inseln.get(t.get('insel')) or {}
    return I.get('art')


def weite(t, inseln):
    ""
    f = t.get('funktion') or t.get('rolle')
    if t.get('fest') or t.get('locked') or t.get('rolle') != 'klein':
        key = t.get('rolle') if t.get('rolle') in TABELLE and not t.get('fest') and not t.get('locked') else 'fest'
        if t.get('lose') and not t.get('fest') and not t.get('locked'): key = 'lose'
        g = TABELLE.get(key) or (0.0, 0.0, 0.0, False, 'fest (vorgaben.json, Stecker, Bedienteil oder kein kleines Teil; P22)')
    else:
        a = _inselart(t, inseln)
        key = (f, a) if (f, a) in TABELLE else f
        g = TABELLE.get(key) or OHNE_FUNKTION
        if key not in TABELLE: key = 'ohne_funktion'
    grund, mx, w, dreh, regel = g
    return dict(grund=grund, max=mx, wichtigkeit=w, drehen=dreh, schluessel=key if isinstance(key, str) else '%s@%s' % key, regel=regel)


def tabelle(K):
    ""
    I = K.get('inseln') or {}
    return {r: weite(t, I) for r, t in sorted(K['teile'].items())}


def ueber(w, weg):
    ""
    return max(0.0, weg - w['grund']) if w['grund'] != GANZ else 0.0


def preis(w, weg_alt, weg_neu):
    ""
    return w['wichtigkeit'] * (ueber(w, weg_neu) - ueber(w, weg_alt))


def im_rahmen(w, weg_neu, eps=1e-6):
    return w['max'] > 0 and weg_neu <= w['max'] + eps


def beweglich(w):
    return w['max'] > 0


if __name__ == '__main__':
    K = json.load(open(sys.argv[1]))
    for r, w in tabelle(K).items():
        f = lambda x: 'ganz' if x == GANZ else '%.1f' % x
        print('%-10s %-30s grund %-5s max %-5s %5.1f/mm %s  %s' % (r, w['schluessel'], f(w['grund']), f(w['max']), w['wichtigkeit'],
                                                                 '' if w['drehen'] else 'nicht drehen', w['regel']))
