import json, math, re, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kicad.kicad_protection import activate
activate()
import pcbnew

src, drc_json, out, suggestion = sys.argv[1:5]
args = dict(a.split('=', 1) for a in sys.argv[5:] if '=' in a)
off = {tuple(x) for x in json.loads(Path(args['exclusion']).read_text())} if args.get('exclusion') else set()
b = pcbnew.LoadBoard(src)
if b is None: raise ValueError('Board not readable: ' + src)
drc = json.loads(Path(drc_json).read_text())
V = lambda x, y: pcbnew.VECTOR2I(int(round(x)), int(round(y)))
pads = {p.m_Uuid.AsString(): p for fp in b.GetFootprints() for p in fp.Pads()}
zone_list = {z.m_Uuid.AsString(): z for z in b.Zones() if not z.GetIsRuleArea()}


def findings():
    out_ = []
    for v in drc.get('violations', []):
        if v.get('type') != 'starved_thermal' or v.get('severity') != 'error' or v.get('excluded'): continue
        d = v.get('description', '')
        if re.search(r'island|Island', d): continue
        m = re.search(r'\((?:layer|Layer) ([^;)]+)', d)
        p = next((pads[i['uuid']] for i in v['items'] if i.get('uuid') in pads), None)
        z = next((zone_list[i['uuid']] for i in v['items'] if i.get('uuid') in zone_list), None)
        if not (m and p and z): continue
        lid = b.GetLayerID(m.group(1).strip())
        if lid < 0: continue
        nominal = re.search(r'(?:count|Steganzahl) (\d+)', d); actual = re.search(r'(?:actual|tatsächlich) (\d+)', d)
        missing = max(1, int(nominal.group(1)) - int(actual.group(1))) if nominal and actual else 1
        out_.append((p, z, lid, missing))
    return out_


def exit_point(p, ux, uy):
    c = p.GetPosition(); lo, hi = 0.0, float(max(p.GetSize().x, p.GetSize().y))
    for _ in range(40):
        m = (lo + hi) / 2
        if p.HitTest(V(c.x + ux * m, c.y + uy * m)): lo = m
        else: hi = m
    return hi


def foreign(lid, net, box):
    items = [t for t in b.GetTracks() if t.GetNetCode() != net and t.IsOnLayer(lid)]
    items += [q for q in pads.values() if q.GetNetCode() != net and q.IsOnLayer(lid) and q.FlashLayer(lid)]
    items += [z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() != net and z.IsOnLayer(lid)]
    return [i for i in items if i.GetBoundingBox().Intersects(box)]


assigned = []
for p, z, lid, missing in findings():
    c = p.GetPosition(); net = p.GetNetCode()
    gap = z.GetThermalReliefGap(); w = max(z.GetThermalReliefSpokeWidth(), z.GetMinThickness())
    w = min(w, int(min(p.GetSize().x, p.GetSize().y)))
    fill = z.GetFilledPolysList(lid)
    cand = []
    for k in range(8):
        a = math.radians(45 * k); ux, uy = math.cos(a), math.sin(a)
        if (p.m_Uuid.AsString(), b.GetLayerName(lid), k) in off: continue
        r = exit_point(p, ux, uy) + gap + w
        e = V(c.x + ux * r, c.y + uy * r)
        if not fill.Contains(e): continue
        t = pcbnew.PCB_TRACK(b); t.SetStart(c); t.SetEnd(e); t.SetWidth(w); t.SetLayer(lid); t.SetNet(p.GetNet())
        box = t.GetBoundingBox(); box.Inflate(pcbnew.FromMM(1))
        sh = t.GetEffectiveShape(lid); ok = True; vacant = 1e18
        for i in foreign(lid, net, box):
            clr = max(t.GetOwnClearance(lid), i.GetOwnClearance(lid))
            if i.Type() == pcbnew.PCB_ZONE_T:
                if i.GetFilledPolysList(lid).Collide(sh, clr) if hasattr(i.GetFilledPolysList(lid), 'Collide') else False: ok = False; break
                continue
            if sh.Collide(i.GetEffectiveShape(lid), clr): ok = False; break
            for d_ in (2 * clr, 3 * clr, 4 * clr):
                if sh.Collide(i.GetEffectiveShape(lid), d_): vacant = min(vacant, d_); break
        if ok: cand.append((-vacant, k, t, e))
    for _, k, t, e in sorted(cand, key=lambda q: (q[0], q[1]))[:missing]:
        b.Add(t)
        assigned.append(dict(pad=p.m_Uuid.AsString(), ref=p.GetParentFootprint().GetReference(), number=p.GetNumber(),
                            net_name=p.GetNetname(), layer_name=b.GetLayerName(lid), direction=k,
                            from_=[pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)], until=[pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)], width=pcbnew.ToMM(w)))
Path(suggestion).write_text(json.dumps(assigned, indent=1) + '\n')
if assigned: b.Save(out)
print('Spoke pieces placed:', len(assigned))
