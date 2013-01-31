import wx
import wx.lib.newevent

(UpdatePoseEvent, EVT_UPDATE_POSE) = wx.lib.newevent.NewEvent()
(UpdateVectorCommand, EVT_UPDATE_VECTOR) = wx.lib.newevent.NewEvent()