import glob, gzip, hashlib, json, math, os, random, subprocess, sys, time
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
GAL = os.path.expanduser('~/Library/Caches/Autorouter/gallery')
OFF = os.path.join(R, 'learning', 'zones_appraisal')
DATA = os.path.join(R, 'learning', 'data', 'zones_verdicts.jsonl')
VERSION = 'zone_list-appraisal-v1'

ERROR = {
    'edges_unnecessary': 'more corners/bends in the outline than necessary, stairs, spikes',
    'roundings_unnecessary': 'Bulges/circular arcs that serve no purpose (e.g. via or track cap bumps)',
    'notches': 'Notches or slits into the edge',
    'finger_stub': 'Spurs, fingers, stubs that carry no current',
    'width_jump': 'Jumps thick/thin along the current path instead of uniform width',
    'bottleneck': 'unnecessary bottleneck/neck in the current path',
    'too_thin': 'overall too narrow for the current, not uniformly thick',
    'copper_wasted': 'Area that carries no current (too wide, blob, areas away from the path)',
    'more_copper_possible': 'free room next to it stays unused where more copper would improve the path',
    'no_pad_reference': 'Zone does not enclose the pads cleanly, pads are attached only via tracks',
    'diagonal_band': 'long diagonal band instead of a compact shape',
    'blob': 'shapeless blob instead of a clear corridor',
    'splitter_island': 'detached pieces, islands, slivers',
    'sharp_angle': 'sharp corners or sharp white wedges towards foreign copper',
}

JOB = """You are a reviewer for copper zones (current zones) on circuit boards. Each image shows ONE current zone (one net on one layer)
as a detail: light red with dark outline = the zone; dark red = tracks and pads of the same net; grey = foreign copper
(other nets, must not be touched, white in between is the isolation spacing); black = vias; thin line = board edge.
Scale: the white bar at the bottom left is 1 mm long.

The model is a good human layouter (user: testboard): current zones are compact, smooth corridors between
source and loads, of roughly equal width throughout, enclosing the pads, using free space sensibly, without spikes, notches,
fingers, blobs or unnecessary rounding; white only as isolation spacing; sides preferably straight (also diagonal), corners slightly rounded.

Behind each image path stands the total current of the net and its zone area on all layers. A net distributes its current over
several layers: judge width and "too_thin"/"bottleneck" by the share this layer carries (small area here, large on
other layers = this layer carries only part of the current), not by the total current alone.

Judge each zone in detail:
- rating: 1 (very bad) to 10 (like a very good human)
- faults: list of exactly these keys (only those that apply): {catalog}
- heavy: per error 1 (slight) to 3 (strong), as an object {{"notches": 2, ...}}
- better: 1-2 sentences on what concretely should be done better (e.g. "1.5 mm wide straight band from C3 to U2, remove cap at the via")
Answer ONLY with JSON: {{"zone_list":[{{"image":"<filename>","rating":4,"faults":["notches"],"heavy":{{"notches":2}},"better":"…"}}]}}
Images:
"""


def _geo(z):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    return unary_union([Polygon(p['outer'], p.get('inner') or []).buffer(0) for p in z['polys']])


def _track(t):
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    return unary_union([LineString([a, b]).buffer(w / 2, 8) for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])
                        if abs(a[0] - b[0]) > 1e-6 or abs(a[1] - b[1]) > 1e-6])


def ground(zg):
    import shapely
    part_list = [p for p in getattr(zg, 'geoms', [zg]) if p.geom_type == 'Polygon']
    A = zg.area; P = zg.length
    corners = sum(len(p.exterior.coords) - 1 + sum(len(i.coords) - 1 for i in p.interiors) for p in part_list)
    hull = zg.convex_hull.area or 1e-9
    width_list = []
    for r in (0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0):
        e = zg.buffer(-r, 8)
        width_list.append(round(e.area / A, 4) if A else 0)
        if e.is_empty: break
    return dict(area_mm2=round(A, 3), perimeter_mm=round(P, 3), corners=corners, part_list=len(part_list),
                compact=round(4 * math.pi * A / P ** 2, 4) if P else 0, convex=round(A / hull, 4),
                hole_list=sum(len(p.interiors) for p in part_list), erosion=width_list)


def draw(B, Rt, zone, goal, px_mm=40, border=2.0, max_px=1100):
    from PIL import Image, ImageDraw
    from shapely.geometry import Point
    zg = _geo(zone); L = zone['layer']; n = zone['net']
    x0, y0, x1, y1 = zg.bounds; x0 -= border; y0 -= border; x1 += border; y1 += border
    S = min(px_mm, max_px / max(x1 - x0, y1 - y0)); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    in_ = Image.new('RGB', (W, H), (250, 250, 247)); dr = ImageDraw.Draw(in_, 'RGBA')
    tr = lambda p: ((p[0] - x0) * S, (p[1] - y0) * S)
    def poly(g, fill, outline=None, width=1):
        for p in getattr(g, 'geoms', [g]):
            if p.geom_type != 'Polygon' or p.is_empty: continue
            dr.polygon([tr(q) for q in p.exterior.coords], fill=fill, outline=outline)
            for i in p.interiors: dr.polygon([tr(q) for q in i.coords], fill=(250, 250, 247))
            if outline: dr.line([tr(q) for q in p.exterior.coords], fill=outline, width=width)
    for o in B['outline']:
        if len(o) >= 3: dr.line([tr(q) for q in o] + [tr(o[0])], fill=(90, 90, 90), width=2)
    for z in Rt.get('zone_list', []):
        if z['layer'] == L and z['net'] != n and z.get('polys'): poly(_geo(z), (185, 185, 185, 255))
    for t in Rt.get('tracks', []):
        if t['layer'] == L and t['net'] != n and len(t['pts']) > 1: poly(_track(t), (160, 160, 160, 255))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3 and p['net'] != n: dr.polygon([tr(v) for v in q], fill=(150, 150, 150, 255))
    poly(zg, (240, 120, 110, 255), outline=(150, 30, 25, 255), width=max(1, int(S / 25)))
    for t in Rt.get('tracks', []):
        if t['layer'] == L and t['net'] == n and len(t['pts']) > 1: poly(_track(t), (200, 40, 35, 200))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3 and p['net'] == n: dr.polygon([tr(v) for v in q], fill=(140, 20, 20, 255))
    for v in Rt.get('vias', []):
        x, y = tr((v['x'], v['y'])); r = v.get('diameter', 0.45) / 2 * S
        dr.ellipse([x - r, y - r, x + r, y + r], fill=(20, 20, 20, 230))
    dr.rectangle([10, H - 14, 10 + S, H - 10], fill=(0, 0, 0))
    in_.save(goal, optimize=True)
    return zg


def row(w):
    L = w.get('layers_mm2') or {}
    if not L: return w['image_path']
    part_list = ', '.join('%s %.0f mm²%s' % (k, v, ' (this image)' if k == w['layer'] else '') for k, v in sorted(L.items()))
    return '%s  — net %s, %s A total; zone per layer: %s' % (w['image_path'], w['net'], ('%.2f' % w['amps_a']) if w.get('amps_a') is not None else '?', part_list)


def prepare(n=40, per_board=3, seed=None, gal=GAL, fresh=40, off=None, seen=None):
    from learning.gallery import lines
    rnd = random.Random(seed if seed is not None else int(time.time()) // 3600)
    Z = [z for z in lines(os.path.join(gal, 'gallery.jsonl')) if z.get('is_valid') and z.get('geometry')
         and os.path.exists(os.path.join(gal, z['geometry']))][-fresh:]
    rnd.shuffle(Z)
    time_s = time.strftime('%Y%m%d_%H%M%S'); goal = os.path.join(off or OFF, time_s); os.makedirs(goal + '_images', exist_ok=True)
    W = []
    for z in Z:
        if len(W) >= n: break
        raw = open(os.path.join(gal, z['geometry']), 'rb').read(); g = json.loads(gzip.decompress(raw))
        B, Rt = g['board'], g['routes']
        zs = [q for q in Rt.get('zone_list', []) if q.get('source') == 'current_zones' and q.get('polys')]
        raw_sha=hashlib.sha256(raw).hexdigest()
        zs = [q for q in zs if _geo(q).area >= 1.0 and (raw_sha,q['net'],q['layer']) not in (seen or set())]
        for q in rnd.sample(zs, min(per_board, len(zs))):
            if len(W) >= n: break
            name = 'zone_%04d.png' % len(W)
            zg = draw(B, Rt, q, os.path.join(goal + '_images', name))
            W.append(dict(image=name, image_path=os.path.join(goal + '_images', name), gallery_image=z['image'],
                          geometry=z['geometry'], geometry_sha256=raw_sha, board=z.get('board'),
                          family=((g.get('netclasses') or {}).get('specs') or {}).get('family') or z.get('family'),
                          source_meta=z.get('source_meta'),
                          cu_layers=z.get('cu_layers'), net=q['net'], layer=q['layer'], amps_a=q.get('amps_a'),
                          model=q.get('model'), ground=ground(zg), version=VERSION,
                          layers_mm2={p['layer']: round(_geo(p).area, 1) for p in Rt.get('zone_list', [])
                                     if p.get('source') == 'current_zones' and p.get('net') == q['net'] and p.get('polys')}))
    Path(goal + '_sample.json').write_text(json.dumps(dict(time_s=time_s, sample=W), indent=1))
    catalog = ', '.join('%s (%s)' % kv for kv in ERROR.items())
    Path(goal + '_job.md').write_text(JOB.format(catalog=catalog) + '\n'.join(row(w) for w in W)
                                          + '\nWrite the JSON answer to ' + goal + '_answer.json\n')
    return goal


def evaluate(goal, judge='claude'):
    W = {w['image']: w for w in json.load(open(goal + '_sample.json'))['sample']}
    p = goal + ('_codex_answer.json' if judge == 'codex' else '_answer.json')
    txt = open(p).read(); A = json.loads(txt[txt.index('{'):txt.rindex('}') + 1])
    lines = []
    for x in A.get('zone_list', []):
        w = W.get(x.get('image'))
        g = x.get('rating')
        if not w or isinstance(g, bool) or not isinstance(g, (int, float)) or not 1 <= g <= 10: continue
        faults = [f for f in (x.get('faults') or []) if f in ERROR]
        heavy = {f: int(v) for f, v in (x.get('heavy') or {}).items() if f in ERROR and isinstance(v, (int, float))}
        lines.append(dict(w, rating=float(g), faults=faults, heavy=heavy, better=x.get('better'),
                           judge=A.get('judge') or judge, appraisal=os.path.basename(goal), time_s=time.strftime('%Y-%m-%d %H:%M')))
    os.makedirs(os.path.dirname(DATA), exist_ok=True)
    with open(DATA, 'a') as f:
        for z in lines: f.write(json.dumps(z, ensure_ascii=False) + '\n')
    import statistics, collections
    C = collections.Counter(f for z in lines for f in z['faults'])
    print('%d zones scored (%s), mean score %.1f; most common errors: %s' % (
        len(lines), judge, statistics.mean(z['rating'] for z in lines) if lines else 0, ', '.join('%s %d' % kv for kv in C.most_common(6))))
    return lines


def codex(goal, effort='high', model='gpt-6.1-sol'):
    W = json.load(open(goal + '_sample.json'))['sample']
    prompt = open(goal + '_job.md').read().replace(goal + '_answer.json', goal + '_codex_answer.json')
    cmd = ['codex', 'exec', '--sandbox', 'read-only', '-m', model, '-c', 'model_reasoning_effort=%s' % effort,
           '-o', goal + '_codex_answer.txt']
    for w in W: cmd += ['-i', w['image_path']]
    cmd += ['--', '-']
    r = subprocess.run(cmd, cwd=R, input=prompt.replace('Write the JSON answer to', 'Print the JSON answer (it is saved as)'),
                       capture_output=True, text=True, timeout=5400)
    Path(goal + '_codex.log').write_text('rc=%s\n%s' % (r.returncode, r.stderr[-4000:]))
    if r.returncode != 0: print('Codex failed, see', goal + '_codex.log'); return 1
    txt = open(goal + '_codex_answer.txt').read()
    Path(goal + '_codex_answer.json').write_text(txt[txt.index('{'):txt.rindex('}') + 1])
    evaluate(goal, 'codex'); return 0


def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); pos = [x for x in a if '=' not in x]
    if not pos: print(__doc__); return 0
    if pos[0] == 'prepare':
        print(prepare(int(kw.get('n', '40')), int(kw.get('per_board', '3')), int(kw['seed']) if 'seed' in kw else None,
                          fresh=int(kw.get('fresh', '40')))); return 0
    if pos[0] == 'evaluate': evaluate(pos[1], kw.get('judge', 'claude')); return 0
    if pos[0] == 'codex': return codex(pos[1], kw.get('effort', 'high'), kw.get('model', 'gpt-6.1-sol'))
    print(__doc__); return 1


if __name__ == '__main__':
    sys.exit(main())
