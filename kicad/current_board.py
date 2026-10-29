import json
import math
from pathlib import Path
import sys

import pcbnew
import wx

APP = wx.GetApp() or wx.App(False)
_log_goal = wx.LogStderr(); wx.Log.SetActiveTarget(_log_goal)
B = json.loads(Path(sys.argv[1]).read_text())
board = pcbnew.BOARD()
board.SetCopperLayerCount(int(B['cu_layers']))
MM = pcbnew.FromMM


def vec(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


net = {}
for n in sorted({p['net'] for p in B['pads']} - {''}):
    net[n] = pcbnew.NETINFO_ITEM(board, n)
    board.Add(net[n])
worst = 0.0
root = Path('/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints')
for f in B['fps']:
    fp = pcbnew.FootprintLoad(str(root / (f['lib'] + '.pretty')), f['fp'])
    if fp is None:
        raise ValueError('Footprint missing: ' + f['ref'])
    board.Add(fp)
    fp.SetReference(f['ref'])
    fp.SetValue(f.get('value', ''))
    if f['side'] == 'B':
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)


    fp.SetOrientationDegrees(f.get('rot', 0))
    fp.SetPosition(vec(f['x'], f['y']))
    targets = [p for p in B['pads'] if p['ref'] == f['ref']]
    def deviation():
        ds = []
        for pad in fp.Pads():
            same = [t for t in targets if str(t['num']) == pad.GetNumber()]
            if same:
                xy = (pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y))
                ds.append(min(math.dist(xy, (t['x'], t['y'])) for t in same))
        return max(ds, default=0)


    if deviation() > .005:
        fits = []
        for deg in sorted({(sign*f.get('rot',0)+j*90)%360 for sign in (-1,1) for j in range(4)}):
            fp.SetOrientationDegrees(deg)
            fits.append((deviation(),deg))
        distance, deg = min(fits)
        fp.SetOrientationDegrees(deg)
        print('Native pose from pad targets:',f['ref'],f.get('rot',0),'->',deg,'Deviation',distance,flush=True)
    for p in fp.Pads():
        xy = (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y))
        same = [t for t in targets if str(t['num']) == p.GetNumber()]
        t = min(same, key=lambda t: math.dist(xy, (t['x'], t['y']))) if same else None
        if t:
            if t['net']:
                p.SetNet(net[t['net']])
            worst = max(worst, math.dist((pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)), (t['x'], t['y'])))


    for z in fp.Zones():
        if not z.GetIsRuleArea() or not z.GetDoNotAllowPads(): continue
        for p in fp.Pads():
            if not z.Outline().Contains(p.GetPosition()): continue
            copper = [l for l in z.GetLayerSet().Seq() if pcbnew.IsCopperLayer(l) and p.IsOnLayer(l)]
            if not copper: continue
            cut = pcbnew.SHAPE_POLY_SET()
            p.TransformShapeToPolygon(cut, copper[0], MM(.001), MM(.0001), pcbnew.ERROR_OUTSIDE)
            z.Outline().BooleanSubtract(cut)
            print('Own pad landing spot within the rule range:', f['ref'], p.GetNumber(), flush=True)
for outline in B['outline']:
    for a, c in zip(outline, outline[1:] + outline[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(vec(*a))
        s.SetEnd(vec(*c))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.05))
        board.Add(s)
if worst > 0.005:
    raise ValueError('Pad target deviation: %g mm' % worst)
pcbnew.SaveBoard(sys.argv[2], board)
print('Pad deviation max:', worst, 'mm; footprints:', len(B['fps']))
