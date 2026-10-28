""
import json, os, re, sys
from PIL import Image, ImageDraw, ImageFont
from shapely.geometry import Polygon
from shapely.ops import unary_union

FARBE = dict(grund=(238, 238, 234), traeger=(38, 52, 44), gnd=(70, 125, 215), vers=(225, 85, 50), sig=(235, 200, 70), bohr=(10, 10, 10),
             weiss=(255, 0, 230), spitz=(255, 255, 255))
_cache = {}


def lade(d):
    if d not in _cache:
        C = json.load(open(os.path.join(d, 'cu.json'))); Z = json.load(open(os.path.join(d, 'zusatz.json'))); E = json.load(open(os.path.join(d, 'k20.json')))
        _cache[d] = (C, Z, E)
    return _cache[d]


def bbox(Z):
    xs = [p[0] for o, h in Z['outline'] for p in o]; ys = [p[1] for o, h in Z['outline'] for p in o]
    return min(xs), min(ys), max(xs), max(ys)


def lage_bild(d, L, px_mm=8.0, ss=2, titel=None, box=None, weiss=True, spitz=True):
    C, Z, E = lade(d)
    x0, y0, x1, y1 = box or bbox(Z)
    x0 -= 0.5; y0 -= 0.5; x1 += 0.5; y1 += 0.5
    S = px_mm * ss; W, H = int((x1 - x0) * S) + 1, int((y1 - y0) * S) + 1
    tr = lambda pts: [((x - x0) * S, (y - y0) * S) for x, y in pts]
    gnd, vers = set(E['gnd']), set(E['vers'])
    im = Image.new('RGB', (W, H), FARBE['grund'])
    def maske(polys_listen):
        m = Image.new('L', (W, H), 0); dr = ImageDraw.Draw(m)
        alle = [pp for pl in polys_listen for pp in pl]
        alle.sort(key=lambda q: -abs(Polygon(q[0]).area) if len(q[0]) >= 3 else 0)
        for o, hs in alle:
            if len(o) < 3: continue
            dr.polygon(tr(o), fill=255)
            for h in hs:
                if len(h) >= 3: dr.polygon(tr(h), fill=0)
        return m
    im.paste(FARBE['traeger'], (0, 0), maske([Z['outline']]))
    ks = Z.get('keepouts') or []
    if not ks and os.path.exists(os.path.join(d, 'board.json')):
        BJ = json.load(open(os.path.join(d, 'board.json')))
        ks = [dict(layers=k.get('lagen') or [L], kein_kupfer=bool((k.get('verbote') or {}).get('flaechen')), polys=[[q, []] for q in k.get('polys', [])]) for k in BJ.get('keepouts', [])]
    for k in ks:
        if k.get('kein_kupfer') and L in k['layers']:
            m = maske([k['polys']]); im.paste((60, 66, 62), (0, 0), m)
    for art, farbe in (('gnd', FARBE['gnd']), ('vers', FARBE['vers']), ('sig', FARBE['sig'])):
        pl = [jl[L] for n, jl in C['nets'].items() if L in jl and (('gnd' if n in gnd else 'vers' if n in vers else 'sig') == art)]
        if pl: im.paste(farbe, (0, 0), maske(pl))
    dr = ImageDraw.Draw(im)
    for b in Z.get('bohrungen', []):
        r = b['d'] / 2 * S; cx, cy = (b['x'] - x0) * S, (b['y'] - y0) * S
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=FARBE['bohr'])
    for v in Z['vias']:
        if v.get('layers') and L not in v['layers']: continue
        r = v['drill'] / 2 * S; cx, cy = (v['x'] - x0) * S, (v['y'] - y0) * S
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=FARBE['bohr'])
    J = E['je_lage'][L]
    if weiss:
        for x, y, a in J.get('weiss_xy', []):
            r = max(0.25, (a / 3.14) ** 0.5) * S; cx, cy = (x - x0) * S, (y - y0) * S
            dr.ellipse([cx - r, cy - r, cx + r, cy + r], outline=FARBE['weiss'], width=max(2, int(S * 0.12)))
    if spitz:
        for kl in ('ecken_gnd', 'ecken_vers'):
            for x, y in J[kl].get('spitz_cu_xy', []):
                r = 0.18 * S; cx, cy = (x - x0) * S, (y - y0) * S
                dr.rectangle([cx - r, cy - r, cx + r, cy + r], outline=FARBE['spitz'], width=max(1, int(S * 0.06)))
    im = im.resize((W // ss, H // ss), Image.LANCZOS)
    if titel:
        d2 = ImageDraw.Draw(im)
        try: f = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', max(12, int(px_mm * 1.6)))
        except Exception: f = ImageFont.load_default()
        d2.rectangle([0, 0, im.width, int(px_mm * 2.4)], fill=(255, 255, 255))
        d2.text((4, 2), titel, fill=(0, 0, 0), font=f)
    return im


if __name__ == '__main__':
    d, L, out = sys.argv[1:4]; px = float(sys.argv[4]) if len(sys.argv) > 4 else 8
    lage_bild(d, L, px, titel='%s %s' % (os.path.basename(d), L)).save(out)
