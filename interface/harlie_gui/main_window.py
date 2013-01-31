import harlie_gui.joystick
__author__="Eric Perko (exp63)/Morgan McClure (mtm28)"
"""Important notes: when adding widgets to the MainWindow class,
Make SURE you don't set their parent as "self" or the window will become blank.
Parents must be "panel"
"""
import wx

try:
    from agw import flatnotebook as fnb
except ImportError: # if it's not there locally, try the wxPython lib.
    import wx.lib.agw.flatnotebook as fnb

import crio_interface.from_crio
import crio_interface.to_crio

import harlie_gui.gui_events
import harlie_gui.from_crio
import harlie_gui.to_crio

weAreOnMac = False
if wx.Platform == "__WXMAC__":
	weAreOnMac = True

class MainNotebook(fnb.FlatNotebook):
	def __init__(self, parent, crioInt):
		fnb.FlatNotebook.__init__(self, parent, wx.ID_ANY)
		self.toCRIO = crioInt
		self.wayPage = harlie_gui.to_crio.WayPanel(self, self.toCRIO)
		self.vectPage = harlie_gui.to_crio.VectorPanel(self, self.toCRIO)
		self.infoPage = harlie_gui.to_crio.SysInfoPanel(self, self.toCRIO)
		self.AddPage(self.wayPage, "Waypoint")
		self.AddPage(self.vectPage, "Vector")
		self.AddPage(self.infoPage, "Sys Info")
		#self.AddPage(PSOPanel, "PSO")
		#if not weAreOnMac:
		#	self.joyPanel = harlie_gui.joystick.JoystickDemoPanel(self, None)
		#	self.AddPage(joyPanel, "Joystick")
		

class MainWindow(wx.Frame):
	"""Main window for Jinx GUI"""
	def __init__(self, *args, **kwargs):
		"""docstring for __init__"""
		wx.Frame.__init__(self, *args, **kwargs)
		
		self.panel = wx.Panel(self)
		self.toCRIO = crio_interface.to_crio.ToCRIO(wxPanel = self.panel)
		self.notebook = MainNotebook(self.panel, self.toCRIO)
		
		self.mode = wx.CheckBox (self.panel, wx.ID_ANY, 'Use AngularForward steering (uncheck for harlie)' )
		self.reflexHalt = wx.Button(self, id=-1, label='STOP ROBOT', size=(150, 28))
		self.reflexHalt.Bind(wx.EVT_BUTTON, self.sendReflexHalt)
		#Set up the PSO Panel and bind it to a CRIO interface
		self.PSOPanel = harlie_gui.from_crio.FromCRIOPanel(self.panel)
		self.fromCRIO = crio_interface.from_crio.FromCRIO(wxPanel = self.PSOPanel)
		
		#Building up our layout structure
		vLayout = wx.BoxSizer(wx.VERTICAL)
		vLayout.Add(self.reflexHalt)
		vLayout.Add(self.mode)
		vLayout.Add(self.PSOPanel)
		hLayout = wx.BoxSizer(wx.HORIZONTAL)
		hLayout.Add(vLayout)
		hLayout.Add(self.notebook, 1, wx.ALL|wx.EXPAND, 5)
	
		#Layout assignment and window resizing
		self.panel.SetSizer(hLayout)
		self.panel.Fit()
		self.Fit()
		self.Layout()

		#Constants to be used for menu events
		UI_ID_Exit = 1000
		UI_ID_Reboot_cRIO = 1001
			
		#add menu
		menu = wx.Menu()
		menu.Append(UI_ID_Exit, "&Exit" ,"Exit this application")

		menu2 = wx.Menu()
		menu2.Append(UI_ID_Reboot_cRIO, "&Reboot", "Reboot the cRIO")

		#create the menu class
		menuBar = wx.MenuBar()
		menuBar.Append(menu, "&Jinx-GUI")
		menuBar.Append(menu2, "&cRIO")

		#add the menu to our window
		self.SetMenuBar(menuBar)

		wx.EVT_MENU(self, UI_ID_Reboot_cRIO, self.sendRebootCRIO)
		wx.EVT_MENU(self, UI_ID_Exit, self.OnUIExit)
		self.Bind(wx.EVT_CLOSE, self.OnExit)
		self.Bind(harlie_gui.gui_events.EVT_UPDATE_VECTOR, self.sendVectorCommand)
	
	def sendReflexHalt(self, event):
		self.toCRIO.send_stop_command(1)

#	def sendSteeringMode(self, mode):
#		self.toCRIO.send_steering_switch()

	def sendRebootCRIO(self, event):
		self.toCRIO.send_reboot_command()
		dlg = wx.MessageDialog(self,
				message='Reboot command sent',
				caption='Rebooting',
				style=wx.OK|wx.ICON_INFORMATION
				)
		dlg.ShowModal()
		dlg.Destroy()

	def sendVectorCommand(self, event):
		if self.mode.GetValue:
			self.toCRIO.send_heading_speed_command(event.angle_change, event.speed)
		else:
			cur_heading = self.fromCRIO.heading
			self.toCRIO.send_heading_speed_command(cur_heading + event.angle_change, event.speed)

	def sendWaypointCommand(self, x, y):
		self.toCRIO.send_waypoint_command(x,y,0,0.5)
    
    	def OnUIExit(self, event):
		self.Close()
		event.Skip()

	def OnExit(self, event):
		self.fromCRIO.cleanup()
		event.Skip()


