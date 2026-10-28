""
import math


def native_platzierung(board, R, pcbnew):
    ""
    if not R.get('rest_board'): return
    target={f['ref']:f for f in R['rest_board']['fps']}
    fps={f.GetReference():f for f in board.GetFootprints()}
    for r,source in R['rest_quelle'].items():
        f=fps[r]; dest=target[r]; pos=f.GetPosition()
        actual=dict(x=pcbnew.ToMM(pos.x),y=pcbnew.ToMM(pos.y),rot=f.GetOrientationDegrees()%360,side='B' if f.IsFlipped() else 'F')
        def same(q):
            return actual['side']==q['side'] and math.dist((actual['x'],actual['y']),(q['x'],q['y']))<.00001 and abs((actual['rot']-q['rot']+180)%360-180)<.0001
        if not any(same(q) for q in ((R.get('rest_native_sources') or {}).get(r) or [source])+[dest]): raise ValueError('R45 native Quellpose stimmt nicht: '+r)
        if actual['side']!=dest['side']: f.Flip(f.GetPosition(),pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        f.SetOrientationDegrees(dest['rot']); f.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(dest['x']),pcbnew.FromMM(dest['y'])))
        wanted=[p for p in R['rest_board']['pads'] if p['ref']==r]
        remaining=list(f.Pads())
        for q in wanted:
            matches=[p for p in remaining if p.GetNumber()==q['num'] and p.GetNetname()==q['net'] and math.dist((pcbnew.ToMM(p.GetPosition().x),pcbnew.ToMM(p.GetPosition().y)),(q['x'],q['y']))<.00001]
            if not matches: raise ValueError('R45 native Zielpads stimmen nicht: '+r)
            remaining.remove(matches[0])
        if remaining: raise ValueError('R45 native Padanzahl stimmt nicht: '+r)


if __name__ == '__main__':
    import json
    import sys
    import pcbnew
    import wx
    from projekt_kopie import regeln_sichern, regeln_kopieren
    app=wx.GetApp() or wx.App(False)
    _log_ziel = wx.LogStderr(); wx.Log.SetActiveTarget(_log_ziel)
    src,routes,out=sys.argv[1:4]
    rules=regeln_sichern(src)
    b=pcbnew.LoadBoard(src)
    native_platzierung(b,json.load(open(routes)),pcbnew)
    pcbnew.SaveBoard(out,b)
    regeln_kopieren(rules,out)
