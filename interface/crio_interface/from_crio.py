__author__="Eric Perko (exp63)"

import socket
import threading
import logging

from harlie_logging import logging_utils
import settings
import packets

if settings.USE_WX:
    import wx
    import harlie_gui.gui_events

class FromCRIO:
    def __init__(self, wxPanel = None):
        self.wxPanel = wxPanel
        self.closeConnections = False
        self.x = 0
        self.y = 0
        self.heading = 0
        self.x_var = 0
        self.y_var = 0
        self.heading_var = 0
        self.voltage = 0
        self.has_data = threading.Event()

        #Setup the logger stuff
        self.logger = logging.getLogger("FromCRIO")
        self.logger.setLevel(settings.logger['level'])
        self.logger.addHandler(logging_utils.handler)
        
        ## UDP listening socket
        self.incomingUDP_PSO = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.incomingUDP_PSO.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.incomingUDP_PSO.bind(('', settings.ports["PSO"])) # host is '' to bind to all addresses/interfaces

        # Start the listening threads
        ## UDP listener thread
        # runs handleIncomingUDP
        # @see handleIncomingUDP
        self.UDP_listener = threading.Thread(target = self.run)
        self.UDP_listener.start()
        

    def cleanup(self):
        self.logger.debug("Starting UDP cleanup")
        self.closeConnections = True
        dummyUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dummyUDP.sendto("CLOSE",("127.0.0.1", settings.ports["PSO"]))
        dummyUDP.close()

        self.incomingUDP_PSO.close()
        self.logger.debug("UDP Listener closed")
        
    def run(self):
        self.logger.debug("UDP Listener Started")
        while True:
            data, addr = self.incomingUDP_PSO.recvfrom(1024)

            # Do not continue if we are closing connections
            if self.closeConnections:
                break

            type = packets.read_packet_type(data)
            if type == packets.DIAGNOSTICS_t:
                self.status = packets.read_diagnostics_packet(data)[0]
                self.logger.debug("Received status. Was: %d" % (self.status, ))
            if type == packets.SYSINFO_t:
                self.voltage = packets.read_sysinfo_packet(data)[0]
                self.logger.debug("Received SysInfo packet, voltage = " % (self.voltage))
            elif type == packets.POSE_t:
                self.x, self.y, self.heading, self.x_var, self.y_var, self.heading_var = packets.read_pose_packet(data)
                self.has_data.set()
                self.logger.debug('X: %f Y: %f Heading: %f' % (self.x, self.y, self.heading))
                self.OnUpdate()


    def OnUpdate(self):
        if settings.USE_WX:
            evt = harlie_gui.gui_events.UpdatePoseEvent(x = self.x, y = self.y, heading = self.heading)
            wx.PostEvent(self.wxPanel, evt)

if __name__ == "__main__":
    settings.USE_WX = False
    f = FromCRIO()
            
