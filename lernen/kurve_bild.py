""
import json
import math
from pathlib import Path
import sys
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont

R = Path(__file__).resolve().parents[1]
W, H = 1200, 840
COLORS = ('#c0392b', '#2471a3', '#1e8449', '#7d3c98', '#b9770e')


def lesen(path):
    path = Path(path)
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def serie(z):
    return json.dumps([vertrag(z),z.get('optionen', {})], sort_keys=True)



def vertrag(z):
    return tuple(json.dumps(z.get(k), sort_keys=True) for k in
                 ('generatorversion', 'aufgaben_sha', 'bewertungsziel', 'messvertrag'))


def verbindbar(a, b):

    for z in (a, b):
        c = z.get('messvertrag') or {}
        if c.get('budget') == 'cpu' and not (c.get('zeit') and c.get('cpu_skalierung')):
            return False
    return vertrag(a) == vertrag(b)

def zeichnen(erzeugt, echt, ziel):
    im = Image.new('RGB', (W, H), 'white')
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    def text(xy, s, fill='black'):
        d.text(xy, s, fill=fill, font=font)
    for col, (title, rows) in enumerate((('Erzeugte Aufgaben', erzeugt), ('Echte Boards · native DRC', echt))):
        left, right = col*600+65, col*600+575
        text((left, 16), title)
        if not rows:
            text((left, 72), 'Noch keine Messung')
            continue
        rows = sorted(rows, key=lambda z: z['zeit'])
        times = [datetime.strptime(z['zeit'], '%Y-%m-%d %H:%M').timestamp() for z in rows]
        x = lambda t: left+(t-min(times))/max(60, max(times)-min(times))*(right-left)
        groups = {}
        for z, t in zip(rows, times):
            groups.setdefault(serie(z), []).append((z, t))
        n = max(z['aufgaben'] if isinstance(z.get('aufgaben'), int) else len(z.get('aufgaben') or []) for z in rows) or 1
        y = lambda v: 310-v/n*240
        for k in sorted({0, n//2, n}):
            d.line([(left, y(k)), (right, y(k))], fill='#dddddd')
            text((left-35, y(k)-8), str(k))
        last = rows[-1]
        text((left, 43), 'Gültig: %s/%s · unbekannt: %s' % (last.get('gueltig'), last['aufgaben'], last.get('unbekannt', 0)))
        cv = [z.get('cpu_je_gueltig') for z in rows if z.get('cpu_je_gueltig') and z['cpu_je_gueltig'] > 0]
        if col == 1:
            text((left, 355), 'Native Restfehler: offene Paare (blau), DRC-Fehler (rot)')
            rest = [(z, t, [r.get('offene_paare') for r in z.get('ergebnisse', [])],
                     [r.get('drc_fehler') for r in z.get('ergebnisse', [])]) for z, t in zip(rows, times)]
            valid_rest = [(z, t, sum(op), sum(dr)) for z, t, op, dr in rest
                          if op and dr and all(v is not None for v in op+dr)]
            if valid_rest:
                maximum = max(1, max(max(op, dr) for z, t, op, dr in valid_rest))
                yr = lambda v: 550-v/maximum*150
                for v in (0, maximum):
                    d.line([(left, yr(v)), (right, yr(v))], fill='#dddddd')
                    text((left-45, yr(v)-8), str(v))
                for index, color in ((2, '#2471a3'), (3, '#c0392b')):
                    previous = {}
                    for item in valid_rest:
                        z, t = item[:2]
                        pt = (x(t), yr(item[index]))
                        key = serie(z)
                        if key in previous:
                            d.line([previous[key], pt], fill=color, width=2)
                        d.ellipse([pt[0]-3, pt[1]-3, pt[0]+3, pt[1]+3], fill=color)
                        previous[key] = pt
                text((left, 570), 'Zuletzt %d offene Paare / %d DRC-Fehler (Summe)' % valid_rest[-1][2:])
            else:
                text((left, 420), 'Native Restfehler noch unbekannt')
        else:
            text((left, 400), 'Gültigkeit bleibt die Qualitätszahl.')
            text((left, 430), 'Andere Eingaben/Optionen bilden getrennte Serien.')
        text((left, 625), 'CPU-s je gültigem Board · Nebenzahl (log)')
        if cv:
            lo, hi = math.log10(min(cv)), math.log10(max(cv))
            yc = lambda v: 750-(math.log10(v)-lo)/max(1, hi-lo)*90
            for v in sorted({min(cv), max(cv)}):
                d.line([(left, yc(v)), (right, yc(v))], fill='#dddddd')
                text((left-55, yc(v)-8), '%.0f' % v)
        else:
            text((left, 680), 'Keine gültigen CPU-Bezugswerte')
        for i, data in enumerate(groups.values()):
            color = COLORS[i % len(COLORS)]
            prev = prev_cpu = None
            for z, t in data:
                pt = (x(t), y(z.get('gueltig') or 0))
                if prev and verbindbar(prev[1],z):
                    d.line([prev[0], (pt[0], prev[0][1]), pt], fill=color, width=2)
                d.ellipse([pt[0]-3, pt[1]-3, pt[0]+3, pt[1]+3], fill=color)
                prev = (pt,z)
                c = z.get('cpu_je_gueltig')
                if c and c > 0:
                    cp = (x(t), yc(c))
                    if prev_cpu and verbindbar(prev_cpu[1],z):
                        d.line([prev_cpu[0], cp], fill=color, width=2)
                    d.ellipse([cp[0]-3, cp[1]-3, cp[0]+3, cp[1]+3], fill=color)
                    prev_cpu = (cp,z)
                else:
                    prev_cpu = None
        text((left, 785), rows[0]['zeit']+' → '+rows[-1]['zeit'])
        text((left, 812), '%d getrennte Eingabe-/Optionsserien' % len(groups))
    ziel = Path(ziel).expanduser()
    ziel.parent.mkdir(parents=True, exist_ok=True)
    im.save(ziel)
    return ziel


def main():
    args = sys.argv[1:]
    kw = dict(a.split('=', 1) for a in args if '=' in a)
    ziel = next((a for a in args if '=' not in a), '~/Library/Caches/Autorouter/berichte/lernkurve.png')
    print(zeichnen(lesen(kw.get('daten', R/'lernen/daten/lernkurve.jsonl')),
                   lesen(kw.get('echt', R/'lernen/daten/echtkurve.jsonl')), ziel))


if __name__ == '__main__':
    main()
