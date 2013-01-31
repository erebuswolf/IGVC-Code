import wx
import math
import harlie_gui.gui_events
#just a constant for spacing in layouts
spacing = 6

class WayPanel(wx.Panel):
	def __init__(self, parent, crio_interface):
        	wx.Panel.__init__(self, parent=parent, id=wx.ID_ANY)

		self.parent = parent
		self.CrioInterface = crio_interface
        	self.l1 = wx.StaticText(self, wx.ID_ANY, "X Coordinate")
        	self.xCoord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.xCoord.SetValue("0.00")
        
		self.l2 = wx.StaticText(self, wx.ID_ANY, "Y Coordinate")
		self.yCoord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.yCoord.SetValue("0.00")
	
		self.l3 = wx.StaticText(self, wx.ID_ANY, "Speed")
		self.speed = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.speed.SetValue("0.00")
		
		self.button = wx.Button(self, id=-1, label='Send', size=(75, 28))
		self.button.Bind(wx.EVT_BUTTON, self.buttonClick)

		spacing = 6
        	sizer = wx.FlexGridSizer(cols=3, hgap=spacing, vgap=spacing)
        	sizer.AddMany([ self.l1, self.xCoord, (0,0),
				self.l2, self.yCoord, (0,0),
				self.l3, self.speed, (0,0),
				self.button,
				])

	        self.SetSizer(sizer)

	def buttonClick(self,event):
		self.CrioInterface.send_waypoint_command(float(self.xCoord.GetValue()), float(self.yCoord.GetValue()), 0, float(self.speed.GetValue()))


class VectorPanel(wx.Panel):
	def __init__(self, parent, crio_interface):
        	wx.Panel.__init__(self, parent=parent, id=wx.ID_ANY)

		self.parent = parent
		self.CrioInterface = crio_interface

		self.labelx = wx.StaticText(self, wx.ID_ANY, "X")
		self.labely = wx.StaticText(self, wx.ID_ANY, "Y")
		self.labelstart = wx.StaticText(self, wx.ID_ANY, "Start")
		self.labelend = wx.StaticText(self, wx.ID_ANY, "End")

        	self.x1Coord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.x1Coord.SetValue("0.00")
        	self.x2Coord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.x2Coord.SetValue("0.00")

        	self.y1Coord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.y1Coord.SetValue("0.00")
        	self.y2Coord = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
		self.y2Coord.SetValue("0.00")
		
		self.button = wx.Button(self, id=-1, label='Send', size=(75, 28))
		self.button.Bind(wx.EVT_BUTTON, self.buttonClick)

        	sizer = wx.FlexGridSizer(cols=3, hgap=spacing, vgap=spacing)
        	sizer.AddMany([ (0,0), self.labelx, self.labely,
				self.labelstart, self.x1Coord, self.y1Coord,
				self.labelend, self.x2Coord, self.y2Coord,
				self.button,
				])

	        self.SetSizer(sizer)
	def buttonClick(self,event):
			foo = 4
		#self.CrioInterface.send_vector_command(float(self.x1Coord.GetValue()), float(self.y1Coord.GetValue()), float(self.x2Coord.GetValue()), float(self.y2Coord.GetValue()), 0.5)

class SysInfoPanel(wx.Panel):
	def __init__(self, parent, crio_interface):
        	wx.Panel.__init__(self, parent=parent, id=wx.ID_ANY)

		self.parent = parent
		self.CrioInterface = crio_interface
		self.labelVoltage = wx.StaticText(self, wx.ID_ANY, "Battery Voltage")
        	self.batVoltage = wx.TextCtrl(self, wx.ID_ANY, size=(125, -1))
        	
		sizer = wx.FlexGridSizer(cols=2, hgap=spacing, vgap=spacing)
		sizer.AddMany([ self.labelVoltage, self.batVoltage ])
		self.SetSizer(sizer)

