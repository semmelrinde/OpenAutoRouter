import runpy,sys
import wx
app=wx.GetApp() or wx.App(False)
_log_goal = wx.LogStderr(); wx.Log.SetActiveTarget(_log_goal)
script=sys.argv[1];sys.argv=sys.argv[1:]
runpy.run_path(script,run_name='__main__')
