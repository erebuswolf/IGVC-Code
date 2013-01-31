import math
import time

import crio_interface.from_crio
import crio_interface.to_crio

def make_square():
    f = crio_interface.from_crio.FromCRIO()
    s = crio_interface.to_crio.ToCRIO()
    s.send_heading_speed_command(0.0, 0.3)
    time.sleep(3)
    s.send_heading_speed_command(math.pi/2, 0.0)
    time.sleep(3)
    s.send_heading_speed_command(math.pi/2, 0.3)
    time.sleep(3)
    s.send_heading_speed_command(math.pi, 0.0)
    time.sleep(3)
    s.send_heading_speed_command(math.pi, 0.3)
    time.sleep(3)
    s.send_heading_speed_command((math.pi * 3)/2, 0.0)
    time.sleep(3)
    s.send_heading_speed_command((math.pi * 3)/2, 0.3)
    time.sleep(3)
    s.send_heading_speed_command((math.pi * 2), 0.0)
    time.sleep(3)
    f.cleanup()

if __name__ == "__main__":
    print "Starting a square..."
    make_square()
    print "Square complete"
