import wx
import math
import harlie_gui.gui_events

class FromCRIOPanel(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)

        self.l1 = wx.StaticText(self, wx.ID_ANY, "X Coordinate")
        self.t1 = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))

        self.l2 = wx.StaticText(self, wx.ID_ANY, "Y Coordinate")
        self.t2 = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))

        self.l3 = wx.StaticText(self, wx.ID_ANY, "HeadingRads")
        self.t3 = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))

	
        self.l4 = wx.StaticText(self, wx.ID_ANY, "HeadingDegs")
        self.t4 = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
        
	space = 6
        sizer = wx.FlexGridSizer(cols=3, hgap=space, vgap=space)
        sizer.AddMany([ self.l1, self.t1, (0,0),
                        self.l2, self.t2, (0,0),
                        self.l3, self.t3, (0,0),
                        self.l4, self.t4, (0,0),
                        ])

        self.SetSizer(sizer)

        self.Bind(harlie_gui.gui_events.EVT_UPDATE_POSE, self.UpdatePose)

    def UpdatePose(self, event):
        self.t1.SetValue(str(event.x))
        self.t2.SetValue(str(event.y))
        self.t3.SetValue(str(event.heading))
        self.t4.SetValue(str(math.degrees(event.heading)))
