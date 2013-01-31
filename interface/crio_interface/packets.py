import struct

REBOOTCOMMAND_t = 0
HEADINGSPEEDCOMMAND_t = 1
WAYPOINTCOMMAND_t = 2
STARTCOMMAND_t = 3
STOPCOMMAND_t = 4
QUERYSYSTEMSTATUS_t = 5
ESTOP_t = 6
DIAGNOSTICS_t = 7
STEERINGSWITCH_t = 10
VECTORCOMMAND_t = 11
ANGULARFORWARDCOMMAND_t = 12
POSE_t = 32
ACK_t = 33
SYSINFO_t = 33


def make_vector_packet(x1, y1, x2, y2, speed):
	string = struct.pack(">Ifffff", VECTORCOMMAND_t, x1, y1, x2, y2, speed)
	return string

def make_heading_speed_packet(heading, speed):
	string = struct.pack(">Iff", HEADINGSPEEDCOMMAND_t, heading, speed)
	return string

def make_waypoint_packet(x, y, heading, speed):
	string = struct.pack(">Iffff", WAYPOINTCOMMAND_t, x, y, heading, speed)
	return string

def make_reboot_packet():
	string = struct.pack(">I", REBOOTCOMMAND_t)
	return string

def make_start_packet():
	string = struct.pack(">I", STARTCOMMAND_t)
	return string

def make_stop_packet(force):
	string = struct.pack(">II", STOPCOMMAND_t, force)
	return string

def make_querystatus_packet(verbosity):
	string = struct.pack(">bb", QUERYSYSTEMSTATUS_t, verbosity)
	return string

def make_estop_packet():
	string = struct.pack(">I", ESTOP_t)
	return string

def read_packet_type(packet):
	type = struct.unpack(">I", packet[0:4])[0]
	return type

def read_pose_packet(packet):
	ret_tuple = struct.unpack(">Iffffff", packet)[1:]
	return ret_tuple

def read_info_packet(packet):
	ret_tuple = struct.unpack(">If", packet)[1:]
	return ret_tuple

def read_diagnostics_packet(packet):
	ret_tuple = struct.unpack(">bbbxi", packet)[1:]
	return ret_tuple
