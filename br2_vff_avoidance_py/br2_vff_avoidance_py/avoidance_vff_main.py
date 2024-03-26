import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from math import atan2, sqrt, pi

class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('avoidance_vff')
        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)
        self.vff_debug_pub = self.create_publisher(MarkerArray, 'vff_debug', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'input_scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.control_cycle)
        self.last_scan = None

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_cycle(self):
        if self.last_scan is None:
            return

        now = self.get_clock().now()
        last_scan_time = rclpy.time.Time.from_msg(self.last_scan.header.stamp)
        if (now - last_scan_time).nanoseconds > 1e9:
            return

        vff = self.get_vff(self.last_scan)

        angle = atan2(vff['result'][1], vff['result'][0])
        module = sqrt(vff['result'][0]**2 + vff['result'][1]**2)

        vel = Twist()
        vel.linear.x = max(min(module, 0.3), 0.0)
        vel.angular.z = max(min(angle, 0.5), -0.5)

        self.vel_pub.publish(vel)

        if self.vff_debug_pub.get_subscription_count() > 0:
            self.vff_debug_pub.publish(self.get_debug_vff(vff))

    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 1.0
        vff = {'attractive': [OBSTACLE_DISTANCE, 0.0], 'repulsive': [0.0, 0.0], 'result': [0.0, 0.0]}
        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]

        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + scan.angle_increment * min_idx
            oposite_angle = angle + pi
            complementary_dist = OBSTACLE_DISTANCE - distance_min
            vff['repulsive'] = [np.cos(oposite_angle) * complementary_dist, np.sin(oposite_angle) * complementary_dist]

        vff['result'] = [sum(x) for x in zip(vff['repulsive'], vff['attractive'])]
        return vff

    def get_debug_vff(self, vff_vectors):
        colors = {'RED': [1.0, 0.0, 0.0, 1.0], 'GREEN': [0.0, 1.0, 0.0, 1.0], 'BLUE': [0.0, 0.0, 1.0, 1.0]}
        marker_array = MarkerArray()
        for key, color in colors.items():
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.1
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

            start, end = [0.0, 0.0, 0.0], [vff_vectors[key.lower()][0], vff_vectors[key.lower()][1], 0.0]
            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

            marker_array.markers.append(marker)
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    avoidance_node = AvoidanceNode()
    rclpy.spin(avoidance_node)
    avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
