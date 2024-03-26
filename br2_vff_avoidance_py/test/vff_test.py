import unittest
import numpy as np
from math import pi, atan2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from br2_vff_avoidance_py.avoidance_vff_main import AvoidanceNode  # Adjust the import according to your package structure

class AvoidanceNodeTest(AvoidanceNode):

    def get_vff_test(self, scan):
        return self.get_vff(scan)

    def get_debug_vff_test(self, vff_vectors):
        return self.get_debug_vff(vff_vectors)

def get_scan_test(angle_min, angle_max, angle_increment, ranges):
    scan = LaserScan()
    scan.header.stamp = Node.get_clock().now().to_msg()  # Adjust as necessary for accurate timestamp
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.ranges = ranges
    return scan

class TestVFF(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = AvoidanceNodeTest()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_vff(self):
        scans = [
            # Tuple format: (angle_min, angle_max, angle_increment, ranges)
            (-pi, pi, 2.0 * pi / 16.0, [float('inf')] * 16),
            (-pi, pi, 2.0 * pi / 16.0, [0.0] * 16),
            (-pi, pi, 2.0 * pi / 16.0, [5.0] * 16 + [0.3]),
            # Add the rest of your test cases here following the same pattern
        ]

        for i, (amin, amax, ainc, ranges) in enumerate(scans):
            scan = get_scan_test(amin, amax, ainc, ranges)
            vff_result = self.node.get_vff_test(scan)
            # Add your assertions here based on your test expectations
            # For example:
            # self.assertEqual(vff_result.attractive, [1.0, 0.0])
            # This is just an example, adjust your test logic to match the expected behavior

    # Add more test methods as necessary

if __name__ == '__main__':
    unittest.main()
