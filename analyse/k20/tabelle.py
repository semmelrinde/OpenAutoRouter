""
import json, os, statistics
S = os.path.dirname(os.path.abspath(__file__))
C = os.path.expanduser('~/Library/Caches/Autorouter/selbstspiel')
BOARDS = [('Vorbild', 'vorbild', 'KiCad-Füllung', ['In1', 'In6']),
          ('a164_k4', 'test/a164_k4', 'nativ', None), ('a166_k0', 'test/a166_k0', 'nativ ¹', None), ('a159_k4', 'test/a159_k4', 'nativ', None),
          ('a163_k2', 'test/a163_k2', 'Modell', None), ('a161_k3', 'test/a161_k3', 'Modell', None), ('a160_k7', 'test/a160_k7', 'Modell', None),
          ('d_1077', 'test/d_1077', 'Dauer, Modell ²', []), ('d_0645', 'test/d_0645', 'Dauer, Modell ²', [])]
STREU = ['a164_k0', 'a164_k1', 'a164_k3', 'a164_k4']


def gndlagen(name):
    if name.startswith('a'):
        a, k = name[1:].split('_')
        return json.load(open(os.path.join(C, 'aufgabe_000%s' % a, k, 'routes.json'))).get('gndlagen') or []
    return []


def lade(d):
    E = json.load(open(os.path.join(S, d, 'k20.json')))
    F = json.load(open(os.path.join(S, d, 'k20_form.json'))) if os.path.exists(os.path.join(S, d, 'k20_form.json')) else None
    return E, F


def p(x, n=0): return ('%.' + str(n) + 'f %%') % (100 * x)
def f(x, n=1): return ('%.' + str(n) + 'f') % x


zeilen = []
def werte(name, d, art, gl):
    E, F = lade(d); J = E['je_lage']; G = E['gesamt']; A = E['aussen']; I = E['innen']
    gl = gl if gl is not None else gndlagen(name)
    lagen = E['lagen']
    fl = lambda Ls: sum(J[L]['flaeche_mm2'] for L in Ls)
    gw = lambda Ls, k: sum(J[L][k] * J[L]['flaeche_mm2'] for L in Ls) / max(1e-9, fl(Ls))
    auss = [L for L in lagen if L in ('F', 'B')]; ebene = [L for L in lagen if L in gl]; rest = [L for L in lagen if L not in auss and L not in gl]
    nL = len(lagen); flL = fl(lagen)
    ins = [J[L]['gnd_inseln'] for L in lagen]
    v = {}
    v['Quelle'] = art
    v['Lagen'] = str(nL)
    v['Platinenfläche'] = '%.1f cm²' % (E['flaeche_mm2'] / 100)
    v['GND-Anteil alle Lagen'] = p(G['gnd_anteil'])
    v['GND-Anteil F/B'] = ' / '.join(p(J[L]['gnd_anteil']) for L in auss)
    v['GND-Anteil GND-Ebenen'] = (' / '.join(p(J[L]['gnd_anteil']) for L in ebene)) if ebene else '–'
    v['GND-Anteil übrige Innenlagen'] = (' / '.join(p(J[L]['gnd_anteil']) for L in rest)) if rest else '–'
    v['Versorgung Innenlagen (Anteil Fläche)'] = p(gw([L for L in lagen if L not in ('F', 'B')], 'vers_anteil'), 1)
    v['GND-Inseln ≥ 1 mm² je Lage (Mittel / max)'] = '%.1f / %d' % (sum(ins) / nL, max(ins))
    v['GND-Inseln je Lage und 10 cm²'] = f(sum(ins) / nL / (flL / nL / 1000), 1)
    v['GND-Splitter < 1 mm² mit Via/Pad · ohne'] = '%d · %d' % (G['gnd_splitter_mit'], G['gnd_splitter_ohne'])
    v['kleinste GND-Insel (Anteil GND-Fläche der Lage)'] = '%.3f %%' % (100 * G['gnd_kleinste_insel_anteil'])
    v['Weiss K17 (Abstand 0,15) gesamt'] = '%.1f mm² (%d Stellen)' % (G['weiss15_mm2'], G['weiss15_n'])
    v['Weiss K17 je 10 cm² Lagenfläche'] = '%.2f mm²' % (G['weiss15_mm2'] / flL * 1000)
    v['Weiss K17 mit eigenem Zonenabstand'] = '%.1f mm² (%.2f je 10 cm²; Abstand %.2f)' % (G['weisseigen_mm2'], G['weisseigen_mm2'] / flL * 1000, E['clr_eigen'])
    v['Versorgungs-Netz×Lage mit Zone'] = str(G['vers_netz_lage'])
    v['Versorgungszonen-Stücke (je Netz×Lage)'] = '%d (%.1f)' % (G['vers_stuecke'], G['vers_stuecke'] / max(1, G['vers_netz_lage'])) if G['vers_netz_lage'] else '–'
    v['Netz×Lage einteilig (Füllung · mit eigenem Kupfer)'] = ('%s · %s' % (p(G['vers_einteilig'] / G['vers_netz_lage']), p(G['vers_einteilig_mit_eigen'] / G['vers_netz_lage']))) if G['vers_netz_lage'] else '–'
    v['Versorgungs-Splitter < 0,5 mm²'] = str(G['vers_splitter']) if G['vers_netz_lage'] else '–'
    if F:
        a = F['aussen']; i = F['innen']
        v['Außenzonen Versorgung: Stück Median / p90 / max'] = '%.1f / %.0f / %.0f mm²' % (a['median_mm2'], a['p90_mm2'], a['max_mm2'])
        v['Außenzonen: Anteil an F+B-Fläche'] = p(a['anteil_lagenflaeche'], 1)
        v['Außenzonen: Breite 2A/U Median'] = '%.2f mm' % a['breite_median']
        v['Außenzonen: Pads je Stück (Median) · Stücke ohne Pad'] = '%g · %d von %d' % (a['pads_median'], a['ohne_pad'], a['stuecke'])
        v['Füllrand Versorgung außen auf 0/45/90°'] = p(a['kante45_vers'])
        v['Innen: Versorgungsflächen > 50 mm² (größte)'] = '%d (%s)' % (sum(1 for g in i['groesste'] if g[2] > 50) if i['stuecke'] else 0, ', '.join('%s %s %.0f mm²' % (g[0].split('/')[-1], g[1], g[2]) for g in i['groesste'][:2] if g[2] > 50) or '–')
    else:
        for k in ('Außenzonen Versorgung: Stück Median / p90 / max', 'Außenzonen: Anteil an F+B-Fläche', 'Außenzonen: Breite 2A/U Median',
                  'Außenzonen: Pads je Stück (Median) · Stücke ohne Pad', 'Füllrand Versorgung außen auf 0/45/90°', 'Innen: Versorgungsflächen > 50 mm² (größte)'): v[k] = '–'
    v['Versorgung als Zone (außen · innen)'] = ('%s · %s' % (p(A['zone_anteil']), p(I['zone_anteil']) if I['zone_mm2'] + I['bahn_mm2'] > 0 else '–')) if A['zone_mm2'] + A['bahn_mm2'] > 0 else '– (keine Zonen)'
    v['Lochanteil Versorgungsflächen (außen · innen)'] = ('%s · %s' % (p(A['lochanteil'], 1), p(I['lochanteil'], 1))) if G['vers_netz_lage'] else '–'
    v['spitze Kupferecken < 90° je 100 mm Zonenrand (alle · Versorgung)'] = '%.2f · %s' % (G['spitz_cu_je100'], f(G['vers_spitz_cu_je100'], 2) if G['vers_rand_mm'] else '–')
    v['spitze weiße Ecken (> 270°) je 100 mm'] = '%.2f' % G['spitz_weiss_je100']
    v['Ecken mit Radius ≥ 0,1 mm (alle · Versorgung)'] = '%s · %s' % (p(G['rund_anteil']), p(G['vers_rund_anteil']) if G['vers_ecken'] else '–')
    R = E['r37']
    v['R37 Bahnanschlüsse in Via-Mitte'] = '%s (%d von %d; %d Vias mit Anschluss am Ring)' % (p(R['anteil'], 1), R['mitte'], R['anschluesse'], R['vias_schief'])
    return v


alle = [(n, werte(n, d, a, g)) for n, d, a, g in BOARDS]
keys = list(alle[0][1].keys())
print('| Kennzahl | ' + ' | '.join(n for n, _ in alle) + ' |')
print('|---|' + '---|' * len(alle))
for k in keys:
    print('| ' + k + ' | ' + ' | '.join(v.get(k, '') for _, v in alle) + ' |')
print()
print('Streuung der 4 nativen Kandidaten von Aufgabe 164 (gleiches Board):')
st = [(n, werte(n, 'test/' + n, 'nativ', None)) for n in STREU]
for k in ('GND-Anteil alle Lagen', 'GND-Anteil F/B', 'Weiss K17 (Abstand 0,15) gesamt', 'Außenzonen Versorgung: Stück Median / p90 / max', 'Außenzonen: Anteil an F+B-Fläche',
          'spitze Kupferecken < 90° je 100 mm Zonenrand (alle · Versorgung)', 'Ecken mit Radius ≥ 0,1 mm (alle · Versorgung)', 'Füllrand Versorgung außen auf 0/45/90°', 'R37 Bahnanschlüsse in Via-Mitte'):
    print('| ' + k + ' | ' + ' | '.join(v[k] for _, v in st) + ' |')
