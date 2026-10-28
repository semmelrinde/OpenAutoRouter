""
import pcbnew, sys, json, math
src, dj, out = sys.argv[1:4]
opt = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
b = pcbnew.LoadBoard(src)
SILK = (pcbnew.F_SilkS, pcbnew.B_SilkS)
NUR_REF = ('${REFERENCE}', '${VALUE}', 'REF**', '%R', '%V')
n_txt = 0
if opt.get('texte', '1') == '1':
    for fp in b.GetFootprints():
        for f in fp.GetFields():
            if f.GetLayer() in SILK and f.IsVisible(): f.SetVisible(False); n_txt += 1
        for g in fp.GraphicalItems():
            if g.GetClass() == 'PCB_TEXT' and g.GetLayer() in SILK and g.IsVisible() and g.GetText().strip() in NUR_REF:
                g.SetVisible(False); n_txt += 1
todo = []
d = json.load(open(dj)) if dj != '-' else {'violations': []}
for v in d['violations']:
    if v['type'] not in ('silk_edge_clearance', 'silk_overlap', 'silk_over_copper'): continue
    for it in v['items']:
        desc = it['description']
        if 'Silkscreen' not in desc or ' of ' not in desc: continue
        ref = desc.split(' of ')[1].split(' on ')[0]
        todo.append((ref, it['pos']['x'], it['pos']['y'], desc.split(' of ')[0]))
n = 0; rem = []
for ref, x, y, kind in todo:
    fp = b.FindFootprintByReference(ref)
    if fp is None: continue
    best = None
    for g in [x for x in fp.GraphicalItems()]:
        if g.GetLayer() not in SILK or g.GetClass() == 'PCB_TEXT': continue
        c = g.GetBoundingBox().GetCenter(); dd = math.dist((pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)), (x, y))
        bb = g.GetBoundingBox()
        inside = pcbnew.ToMM(bb.GetX()) - 0.05 <= x <= pcbnew.ToMM(bb.GetX() + bb.GetWidth()) + 0.05 and pcbnew.ToMM(bb.GetY()) - 0.05 <= y <= pcbnew.ToMM(bb.GetY() + bb.GetHeight()) + 0.05
        if inside and (best is None or dd < best[0]): best = (dd, g)
    if best: rem.append((fp, best[1])); n += 1
seen = set()
for fp, g in rem:
    if id(g) in seen: continue
    seen.add(id(g)); fp.Remove(g)
b.Save(out); print('Silk-Texte versteckt:', n_txt, '| Silk-Elemente entfernt:', n)
