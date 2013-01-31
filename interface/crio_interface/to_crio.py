__author__="Eric Perko (exp63)"

import threading
import socket
import logging

import settings
from harlie_logging import logging_utils
import packets

if settings.USE_WX:
    import wx
    import harlie_gui.gui_events

class ToCRIO:
    def __init__(self, wxPanel = None):
        """Documentation"""
        self.wxPanel = wxPanel
        self.logger = logging.getLogger("ToCRIO")
        self.logger.setLevel(settings.logger['level'])
        self.logger.addHandler(logging_utils.handler)
        self.outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (settings.crio_ip, settings.ports["COMMAND"])

    def send_vector_command(self, x1, y1, x2, y2, speed):
        self.outgoingUDP.sendto(packets.make_vector_packet(x1, y1, x2, y2, speed), self.address)
        self.logger.debug("Sent vector to address %s. (%f %f), (%f, %f)" % (x1, y1, x2, y2))
    
    def send_heading_speed_command(self, heading, speed):
        self.outgoingUDP.sendto(packets.make_heading_speed_packet(heading, speed), self.address)
        self.logger.debug("Sent heading speed to address %s. Heading: %f Speed: %f" % (str(self.address), heading, speed))

    def send_waypoint_command(self, x, y, heading, speed):
        self.outgoingUDP.sendto(packets.make_waypoint_packet(x, y, heading, speed), self.address)
        self.logger.debug("Sent waypoint driver to address %s. Heading: %f Speed %f X: %f Y: %f" % (str(self.address), heading, speed, x, y))

    def send_reboot_command(self):
        self.outgoingUDP.sendto(packets.make_reboot_packet(), self.address)
        self.logger.debug("Sent reboot command to address %s." % str(self.address))

    def send_start_command(self):
        self.outgoingUDP.sendto(packets.make_start_packet(), self.address)
        self.logger.debug("Sent start command to address %s." % str(self.address))

    def send_stop_command(self, force):
        self.outgoingUDP.sendto(packets.make_stop_packet(force), self.address)
        self.logger.debug("Sent stop command to address %s." % str(self.address))

    def send_estop_command(self):
        self.outgoingUDP.sendto(packets.make_estop_packet(), self.address)
        self.logger.debug("Sent estop command to address %s." % str(self.address))

    def send_querystatus_command(self, verbosity):
        self.outgoingUDP.sendto(packets.make_querystatus_packet(verbosity), self.address)
        self.logger.debug("Send querystatus command to address %s." % str(self.address))
        
    
        

