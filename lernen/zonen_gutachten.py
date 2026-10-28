""
import glob, gzip, hashlib, json, math, os, random, subprocess, sys, time
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
GAL = os.path.expanduser('~/Library/Caches/Autorouter/galerie')
AUS = os.path.join(R, 'lernen', 'zonen_gutachten')
DATEN = os.path.join(R, 'lernen', 'daten', 'zonen_urteile.jsonl')
VERSION = 'zonen-gutachten-v1'

FEHLER = {
    'kanten_unnoetig': 'mehr Ecken/Knicke im Umriss als nötig, Treppen, Zacken',
    'rundungen_unnoetig': 'Ausbuchtungen/Kreisbögen, die keinem Zweck dienen (z. B. Via- oder Bahnkappen-Beulen)',
    'kerben': 'Einschnitte oder Schlitze in den Rand',
    'finger_stummel': 'Ausläufer, Finger, Stummel, die keinen Strom führen',
    'breitensprung': 'Sprünge dick/dünn entlang des Strompfads statt gleichmässiger Breite',
    'engstelle': 'unnötige Engstelle/Hals im Strompfad',
    'zu_duenn': 'insgesamt zu schmal für den Strom, nicht durchgehend dick',
    'kupfer_verschwendet': 'Fläche, die keinen Strom führt (zu breit, Klecks, Flächen abseits des Pfads)',
    'mehr_kupfer_moeglich': 'freier Platz daneben bleibt ungenutzt, wo mehr Kupfer den Pfad verbessern würde',
    'kein_padbezug': 'Zone umschliesst die Pads nicht sauber, Pads hängen nur über Bahnen dran',
    'diagonalband': 'langes schräges Band statt kompakter Form',
    'klecks': 'unförmiger Klecks statt klarer Korridor',
    'splitter_insel': 'abgetrennte Stücke, Inseln, Splitter',
    'spitze_winkel': 'spitze Ecken oder spitze weisse Keile zu fremdem Kupfer',
}

AUFTRAG = """Du bist Gutachter für Kupferzonen (Stromzonen) auf Leiterplatten. Jedes Bild zeigt EINE Stromzone (ein Netz auf einer Lage)
als Ausschnitt: hellrot mit dunklem Rand = die Zone; dunkelrot = Bahnen und Pads desselben Netzes; grau = fremdes Kupfer
(andere Netze, darf nicht berührt werden, weiss dazwischen ist der Isolationsabstand); schwarz = Vias; dünne Linie = Platinenrand.
Massstab: der weisse Balken unten links ist 1 mm lang.

Vorbild ist ein guter menschlicher Layouter (Nutzer: Testboard): Stromzonen sind kompakte, glatte Korridore zwischen
Quelle und Lasten, durchgehend etwa gleich breit, umschliessen die Pads, nutzen freien Platz sinnvoll, ohne Zacken, Kerben,
Finger, Kleckse oder unnötige Rundungen; Weiss nur als Isolationsabstand; Kanten gern gerade (auch schräg), Ecken leicht gerundet.

Hinter jedem Bildpfad steht der Gesamtstrom des Netzes und seine Zonenfläche auf allen Lagen. Ein Netz verteilt seinen Strom auf
mehrere Lagen: Beurteile Breite und „zu_duenn“/„engstelle“ nach dem Anteil, den diese Lage trägt (kleine Fläche hier, grosse auf
anderen Lagen = diese Lage führt nur einen Teil des Stroms), nicht nach dem Gesamtstrom allein.

Beurteile jede Zone im Detail:
- note: 1 (sehr schlecht) bis 10 (wie ein sehr guter Mensch)
- fehler: Liste aus genau diesen Schlüsseln (nur die zutreffenden): {katalog}
- schwere: je Fehler 1 (leicht) bis 3 (stark), als Objekt {{"kerben": 2, ...}}
- besser: 1–2 Sätze, was konkret besser gemacht werden sollte (z. B. „Band 1,5 mm breit gerade von C3 zu U2, Kappe am Via weg“)
Antworte NUR mit JSON: {{"zonen":[{{"bild":"<dateiname>","note":4,"fehler":["kerben"],"schwere":{{"kerben":2}},"besser":"…"}}]}}
Bilder:
"""


def _geo(z):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    return unary_union([Polygon(p['aussen'], p.get('innen') or []).buffer(0) for p in z['polys']])


def _bahn(t):
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    return unary_union([LineString([a, b]).buffer(w / 2, 8) for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])
                        if abs(a[0] - b[0]) > 1e-6 or abs(a[1] - b[1]) > 1e-6])


def masse(zg):
    ""
    import shapely
    teile = [p for p in getattr(zg, 'geoms', [zg]) if p.geom_type == 'Polygon']
    A = zg.area; P = zg.length
    ecken = sum(len(p.exterior.coords) - 1 + sum(len(i.coords) - 1 for i in p.interiors) for p in teile)
    hull = zg.convex_hull.area or 1e-9
    breiten = []
    for r in (0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0):
        e = zg.buffer(-r, 8)
        breiten.append(round(e.area / A, 4) if A else 0)
        if e.is_empty: break
    return dict(flaeche_mm2=round(A, 3), umfang_mm=round(P, 3), ecken=ecken, teile=len(teile),
                kompakt=round(4 * math.pi * A / P ** 2, 4) if P else 0, konvex=round(A / hull, 4),
                loecher=sum(len(p.interiors) for p in teile), erosion=breiten)


def zeichnen(B, Rt, zone, ziel, px_mm=40, rand=2.0, max_px=1100):
    from PIL import Image, ImageDraw
    from shapely.geometry import Point
    zg = _geo(zone); L = zone['layer']; n = zone['net']
    x0, y0, x1, y1 = zg.bounds; x0 -= rand; y0 -= rand; x1 += rand; y1 += rand
    S = min(px_mm, max_px / max(x1 - x0, y1 - y0)); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    im = Image.new('RGB', (W, H), (250, 250, 247)); dr = ImageDraw.Draw(im, 'RGBA')
    tr = lambda p: ((p[0] - x0) * S, (p[1] - y0) * S)
    def poly(g, fill, outline=None, width=1):
        for p in getattr(g, 'geoms', [g]):
            if p.geom_type != 'Polygon' or p.is_empty: continue
            dr.polygon([tr(q) for q in p.exterior.coords], fill=fill, outline=outline)
            for i in p.interiors: dr.polygon([tr(q) for q in i.coords], fill=(250, 250, 247))
            if outline: dr.line([tr(q) for q in p.exterior.coords], fill=outline, width=width)
    for o in B['outline']:
        if len(o) >= 3: dr.line([tr(q) for q in o] + [tr(o[0])], fill=(90, 90, 90), width=2)
    for z in Rt.get('zonen', []):
        if z['layer'] == L and z['net'] != n and z.get('polys'): poly(_geo(z), (185, 185, 185, 255))
    for t in Rt.get('tracks', []):
        if t['layer'] == L and t['net'] != n and len(t['pts']) > 1: poly(_bahn(t), (160, 160, 160, 255))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3 and p['net'] != n: dr.polygon([tr(v) for v in q], fill=(150, 150, 150, 255))
    poly(zg, (240, 120, 110, 255), outline=(150, 30, 25, 255), width=max(1, int(S / 25)))
    for t in Rt.get('tracks', []):
        if t['layer'] == L and t['net'] == n and len(t['pts']) > 1: poly(_bahn(t), (200, 40, 35, 200))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3 and p['net'] == n: dr.polygon([tr(v) for v in q], fill=(140, 20, 20, 255))
    for v in Rt.get('vias', []):
        x, y = tr((v['x'], v['y'])); r = v.get('diameter', 0.45) / 2 * S
        dr.ellipse([x - r, y - r, x + r, y + r], fill=(20, 20, 20, 230))
    dr.rectangle([10, H - 14, 10 + S, H - 10], fill=(0, 0, 0))
    im.save(ziel, optimize=True)
    return zg


def zeile(w):
    ""
    L = w.get('lagen_mm2') or {}
    if not L: return w['bild_pfad']
    teile = ', '.join('%s %.0f mm²%s' % (k, v, ' (dieses Bild)' if k == w['layer'] else '') for k, v in sorted(L.items()))
    return '%s  — Netz %s, %s A gesamt; Zone je Lage: %s' % (w['bild_pfad'], w['net'], ('%.2f' % w['strom_a']) if w.get('strom_a') is not None else '?', teile)


def vorbereiten(n=40, je_board=3, seed=None, gal=GAL, frisch=40, aus=None, gesehen=None):
    from lernen.galerie import zeilen
    rnd = random.Random(seed if seed is not None else int(time.time()) // 3600)
    Z = [z for z in zeilen(os.path.join(gal, 'galerie.jsonl')) if z.get('gueltig') and z.get('geometrie')
         and os.path.exists(os.path.join(gal, z['geometrie']))][-frisch:]
    rnd.shuffle(Z)
    zeit = time.strftime('%Y%m%d_%H%M%S'); ziel = os.path.join(aus or AUS, zeit); os.makedirs(ziel + '_bilder', exist_ok=True)
    W = []
    for z in Z:
        if len(W) >= n: break
        raw = open(os.path.join(gal, z['geometrie']), 'rb').read(); g = json.loads(gzip.decompress(raw))
        B, Rt = g['board'], g['routes']
        zs = [q for q in Rt.get('zonen', []) if q.get('quelle') == 'stromzonen' and q.get('polys')]
        raw_sha=hashlib.sha256(raw).hexdigest()
        zs = [q for q in zs if _geo(q).area >= 1.0 and (raw_sha,q['net'],q['layer']) not in (gesehen or set())]
        for q in rnd.sample(zs, min(je_board, len(zs))):
            if len(W) >= n: break
            name = 'zone_%04d.png' % len(W)
            zg = zeichnen(B, Rt, q, os.path.join(ziel + '_bilder', name))
            W.append(dict(bild=name, bild_pfad=os.path.join(ziel + '_bilder', name), galerie_bild=z['bild'],
                          geometrie=z['geometrie'], geometrie_sha256=raw_sha, board=z.get('board'),
                          familie=((g.get('klassen') or {}).get('vorgaben') or {}).get('familie') or z.get('familie'),
                          quelle_meta=z.get('quelle_meta'),
                          lagen=z.get('lagen'), net=q['net'], layer=q['layer'], strom_a=q.get('strom_a'),
                          modell=q.get('modell'), masse=masse(zg), version=VERSION,
                          lagen_mm2={p['layer']: round(_geo(p).area, 1) for p in Rt.get('zonen', [])
                                     if p.get('quelle') == 'stromzonen' and p.get('net') == q['net'] and p.get('polys')}))
    Path(ziel + '_stichprobe.json').write_text(json.dumps(dict(zeit=zeit, stichprobe=W), indent=1))
    katalog = ', '.join('%s (%s)' % kv for kv in FEHLER.items())
    Path(ziel + '_auftrag.md').write_text(AUFTRAG.format(katalog=katalog) + '\n'.join(zeile(w) for w in W)
                                          + '\nSchreibe die JSON-Antwort nach ' + ziel + '_antwort.json\n')
    return ziel


def auswerten(ziel, richter='claude'):
    W = {w['bild']: w for w in json.load(open(ziel + '_stichprobe.json'))['stichprobe']}
    p = ziel + ('_codex_antwort.json' if richter == 'codex' else '_antwort.json')
    txt = open(p).read(); A = json.loads(txt[txt.index('{'):txt.rindex('}') + 1])
    zeilen = []
    for x in A.get('zonen', []):
        w = W.get(x.get('bild'))
        g = x.get('note')
        if not w or isinstance(g, bool) or not isinstance(g, (int, float)) or not 1 <= g <= 10: continue
        fehler = [f for f in (x.get('fehler') or []) if f in FEHLER]
        schwere = {f: int(v) for f, v in (x.get('schwere') or {}).items() if f in FEHLER and isinstance(v, (int, float))}
        zeilen.append(dict(w, note=float(g), fehler=fehler, schwere=schwere, besser=x.get('besser'),
                           richter=A.get('richter') or richter, gutachten=os.path.basename(ziel), zeit=time.strftime('%Y-%m-%d %H:%M')))
    os.makedirs(os.path.dirname(DATEN), exist_ok=True)
    with open(DATEN, 'a') as f:
        for z in zeilen: f.write(json.dumps(z, ensure_ascii=False) + '\n')
    import statistics, collections
    C = collections.Counter(f for z in zeilen for f in z['fehler'])
    print('%d Zonen benotet (%s), mittlere Note %.1f; häufigste Fehler: %s' % (
        len(zeilen), richter, statistics.mean(z['note'] for z in zeilen) if zeilen else 0, ', '.join('%s %d' % kv for kv in C.most_common(6))))
    return zeilen


def codex(ziel, aufwand='high', modell='gpt-6.1-sol'):
    W = json.load(open(ziel + '_stichprobe.json'))['stichprobe']
    prompt = open(ziel + '_auftrag.md').read().replace(ziel + '_antwort.json', ziel + '_codex_antwort.json')
    cmd = ['codex', 'exec', '--sandbox', 'read-only', '-m', modell, '-c', 'model_reasoning_effort=%s' % aufwand,
           '-o', ziel + '_codex_antwort.txt']
    for w in W: cmd += ['-i', w['bild_pfad']]
    cmd += ['--', '-']
    r = subprocess.run(cmd, cwd=R, input=prompt.replace('Schreibe die JSON-Antwort nach', 'Gib die JSON-Antwort aus (wird gespeichert als)'),
                       capture_output=True, text=True, timeout=5400)
    Path(ziel + '_codex.log').write_text('rc=%s\n%s' % (r.returncode, r.stderr[-4000:]))
    if r.returncode != 0: print('Codex fehlgeschlagen, siehe', ziel + '_codex.log'); return 1
    txt = open(ziel + '_codex_antwort.txt').read()
    Path(ziel + '_codex_antwort.json').write_text(txt[txt.index('{'):txt.rindex('}') + 1])
    auswerten(ziel, 'codex'); return 0


def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); pos = [x for x in a if '=' not in x]
    if not pos: print(__doc__); return 0
    if pos[0] == 'vorbereiten':
        print(vorbereiten(int(kw.get('n', '40')), int(kw.get('je_board', '3')), int(kw['seed']) if 'seed' in kw else None,
                          frisch=int(kw.get('frisch', '40')))); return 0
    if pos[0] == 'auswerten': auswerten(pos[1], kw.get('richter', 'claude')); return 0
    if pos[0] == 'codex': return codex(pos[1], kw.get('aufwand', 'high'), kw.get('modell', 'gpt-6.1-sol'))
    print(__doc__); return 1


if __name__ == '__main__':
    sys.exit(main())
