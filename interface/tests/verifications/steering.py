import math
import unittest
from time import sleep

import tests.multitest_helpers

from crio_interface.to_crio import ToCRIO
from crio_interface.from_crio import FromCRIO

def suite():
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RotateInPlaceVerification)
    return suite
    
def get_test_angles():
    angle = 0
    angles = []
    for i in xrange(0, 5):
        angles.append(angle + i * math.pi / 2.0)
    return angles
#    return [0]

def distance_from_origin(x, y):
    return math.sqrt(x*x + y*y)

def min_angle(angle):
    angles = [math.fabs(angle), math.fabs(angle + math.pi*2), math.fabs(angle - math.pi*2)]
    return min(angles)

class RotateInPlaceVerification(unittest.TestCase):
    def setUp(self):
        self.sender = ToCRIO()
        self.sender.send_reboot_command()
        sleep(5)
#        self.sender.send_stop_command()
#        self.sender.send_start_command()
        self.receiver = FromCRIO()
        self.receiver.has_data.wait(180)

    def tearDown(self):
#        self.sender.send_stop_command()
        self.sender.send_heading_speed_command(self.receiver.heading, 0)
        self.sender = None
        self.receiver.cleanup()
        self.receiver = None

    multitest_steering_values = get_test_angles()
    def multitest_steering(self, angle):
        condition = self.receiver.has_data.isSet()
        self.assertTrue(condition, "Timed out waiting for the receiver to receive a packet")
        self.sender.send_heading_speed_command(angle, 0.0)
        sleep(4.8)
        self.x = self.receiver.x
        self.y = self.receiver.y
        self.heading = self.receiver.heading
        self.dist_err = distance_from_origin(self.x, self.y)
        self.assertTrue(self.dist_err < 0.5, "Not within 0.5 meters of origin after turn to %f. Distance Error was %f" % (angle, self.dist_err))
        self.error = angle - self.heading
        self.min_error = min_angle(self.error)
        self.assertTrue(math.degrees(self.min_error) < 5.0, "Heading error not less than 5 degrees after turn to %f. Error was %f" % (angle, math.degrees(self.min_error)))
        
tests.multitest_helpers.add_test_cases(RotateInPlaceVerification)
