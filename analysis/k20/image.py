import json, os, re, sys
from PIL import Image, ImageDraw, ImageFont
from shapely.geometry import Polygon
from shapely.ops import unary_union

COLOR = dict(reason=(238, 238, 234), carrier=(38, 52, 44), gnd=(70, 125, 215), sup=(225, 85, 50), sig=(235, 200, 70), bore=(10, 10, 10),
             white=(255, 0, 230), sharp=(255, 255, 255))
_cache = {}


def load(d):
    if d not in _cache:
        C = json.load(open(os.path.join(d, 'cu.json'))); Z = json.load(open(os.path.join(d, 'extra.json'))); E = json.load(open(os.path.join(d, 'k20.json')))
        _cache[d] = (C, Z, E)
    return _cache[d]


def bbox(Z):
    xs = [p[0] for o, h in Z['outline'] for p in o]; ys = [p[1] for o, h in Z['outline'] for p in o]
    return min(xs), min(ys), max(xs), max(ys)


def layer_image(d, L, px_mm=8.0, ss=2, title=None, box=None, white=True, sharp=True):
    C, Z, E = load(d)
    x0, y0, x1, y1 = box or bbox(Z)
    x0 -= 0.5; y0 -= 0.5; x1 += 0.5; y1 += 0.5
    S = px_mm * ss; W, H = int((x1 - x0) * S) + 1, int((y1 - y0) * S) + 1
    tr = lambda pts: [((x - x0) * S, (y - y0) * S) for x, y in pts]
    gnd, sup = set(E['gnd']), set(E['sup'])
    in_ = Image.new('RGB', (W, H), COLOR['reason'])
    def mask(polys_lists):
        m = Image.new('L', (W, H), 0); dr = ImageDraw.Draw(m)
        all_items = [pp for pl in polys_lists for pp in pl]
        all_items.sort(key=lambda q: -abs(Polygon(q[0]).area) if len(q[0]) >= 3 else 0)
        for o, hs in all_items:
            if len(o) < 3: continue
            dr.polygon(tr(o), fill=255)
            for h in hs:
                if len(h) >= 3: dr.polygon(tr(h), fill=0)
        return m
    in_.paste(COLOR['carrier'], (0, 0), mask([Z['outline']]))
    ks = Z.get('keepouts') or []
    if not ks and os.path.exists(os.path.join(d, 'board.json')):
        BJ = json.load(open(os.path.join(d, 'board.json')))
        ks = [dict(layers=k.get('cu_layers') or [L], no_copper=bool((k.get('prohibitions') or {}).get('areas')), polys=[[q, []] for q in k.get('polys', [])]) for k in BJ.get('keepouts', [])]
    for k in ks:
        if k.get('no_copper') and L in k['layers']:
            m = mask([k['polys']]); in_.paste((60, 66, 62), (0, 0), m)
    for category, color in (('gnd', COLOR['gnd']), ('sup', COLOR['sup']), ('sig', COLOR['sig'])):
        pl = [jl[L] for n, jl in C['nets'].items() if L in jl and (('gnd' if n in gnd else 'sup' if n in sup else 'sig') == category)]
        if pl: in_.paste(color, (0, 0), mask(pl))
    dr = ImageDraw.Draw(in_)
    for b in Z.get('drills', []):
        r = b['d'] / 2 * S; cx, cy = (b['x'] - x0) * S, (b['y'] - y0) * S
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COLOR['bore'])
    for v in Z['vias']:
        if v.get('layers') and L not in v['layers']: continue
        r = v['drill'] / 2 * S; cx, cy = (v['x'] - x0) * S, (v['y'] - y0) * S
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COLOR['bore'])
    J = E['per_layer'][L]
    if white:
        for x, y, a in J.get('white_xy', []):
            r = max(0.25, (a / 3.14) ** 0.5) * S; cx, cy = (x - x0) * S, (y - y0) * S
            dr.ellipse([cx - r, cy - r, cx + r, cy + r], outline=COLOR['white'], width=max(2, int(S * 0.12)))
    if sharp:
        for nc in ('corners_gnd', 'corners_supply'):
            for x, y in J[nc].get('sharp_cu_xy', []):
                r = 0.18 * S; cx, cy = (x - x0) * S, (y - y0) * S
                dr.rectangle([cx - r, cy - r, cx + r, cy + r], outline=COLOR['sharp'], width=max(1, int(S * 0.06)))
    in_ = in_.resize((W // ss, H // ss), Image.LANCZOS)
    if title:
        d2 = ImageDraw.Draw(in_)
        try: f = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', max(12, int(px_mm * 1.6)))
        except Exception: f = ImageFont.load_default()
        d2.rectangle([0, 0, in_.width, int(px_mm * 2.4)], fill=(255, 255, 255))
        d2.text((4, 2), title, fill=(0, 0, 0), font=f)
    return in_


if __name__ == '__main__':
    d, L, out = sys.argv[1:4]; px = float(sys.argv[4]) if len(sys.argv) > 4 else 8
    layer_image(d, L, px, title='%s %s' % (os.path.basename(d), L)).save(out)
