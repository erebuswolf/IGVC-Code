#!/usr/bin/python
__author__="Eric Perko (exp63)"

import sys

#if not hasattr(sys, 'frozen'): #This is here so that if we bundle the python, it won't get screwy
#import wxversion
#wxversion.select('2.8.10') #Require at wxPython version 2.8
import wx

from harlie_logging import logging_utils
import harlie_gui.main_window

class HarlieMain(wx.App):
    def __init__(self, *args, **kwargs):
        wx.App.__init__(self, *args, **kwargs)
    
    def OnInit(self):
        self.h_main_frame = harlie_gui.main_window.MainWindow(None, wx.ID_ANY, 'Harlie GUI')
        self.h_main_frame.Show()
        self.SetTopWindow(self.h_main_frame)
        
        return True
        
if __name__ == "__main__":
    app = HarlieMain(redirect=True, filename="gui_output.log")
    app.MainLoop()
    
