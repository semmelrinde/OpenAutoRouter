""
import json, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kicad.kicad_schutz import aktivieren
aktivieren()
import pcbnew, wx
app = wx.GetApp() or wx.App(False)
_log_ziel = wx.LogStderr(); wx.Log.SetActiveTarget(_log_ziel)
b = pcbnew.LoadBoard(sys.argv[1])
items = (list(b.GetTracks()) + [z for z in b.Zones() if not z.GetIsRuleArea()]
         + [q for q in b.GetDrawings() if pcbnew.IsCopperLayer(q.GetLayer())])
ids = {str(q.m_Uuid.AsString()): q for q in items}
if sys.argv[3] == 'inventar':
    print(json.dumps(sorted(ids)))
else:
    requested = sorted(ids) if sys.argv[3] == 'alle' else json.load(open(sys.argv[3]))
    if any(u not in ids for u in requested): raise ValueError('R44: nicht entfernbares Kupfer')
    for u in requested: b.Delete(ids[u])
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(sys.argv[2], b)
