# Import necessary ROS2 and message packages for robot operation
from geometry_msgs.msg import Twist  # For publishing velocity commands
import rclpy  # ROS2 Python library
from rclpy.duration import Duration  # For time-based operations
from rclpy.node import Node  # Base class for ROS2 nodes
from rclpy.qos import qos_profile_sensor_data  # Quality of service profile for sensor data
from rclpy.time import Time  # ROS2 Time management
from sensor_msgs.msg import LaserScan  # For subscribing to laser scan messages

class BumpGoNode(Node):
    """
    Bump and Go Node Class
    This node handles obstacle avoidance for a robot by subscribing to laser scan data
    and publishing velocity commands based on the presence of obstacles.
    """

    def __init__(self):
        # Initialize the node with the name 'bump_go'
        super().__init__('bump_go')

        # Define robot states for readability
        self.FORWARD = 0  # Moving forward
        self.BACK = 1  # Moving backward
        self.TURN = 2  # Turning (default direction is not specified)
        self.STOP = 3  # Stopped
        self.state = self.FORWARD  # Initial state is set to FORWARD
        self.state_ts = self.get_clock().now()  # Timestamp of the last state change

        # Timing constants for behaviors
        self.TURNING_TIME = 2.0  # Time to turn before attempting to move forward again
        self.BACKING_TIME = 2.0  # Time to move back when an obstacle is detected
        self.SCAN_TIMEOUT = 1.0  # Timeout for considering scan data stale

        # Speed constants for linear and angular movement
        self.SPEED_LINEAR = 0.3  # Linear speed
        self.SPEED_ANGULAR = 0.3  # Angular speed
        self.OBSTACLE_DISTANCE = 1.0  # Distance to consider an obstacle too close

        # the angles for the left, right, and front diagonal sensors
        self.LEFT_DIAGONAL_ANGLE = 90  # Left diagonal sensor at 45 degrees
        self.RIGHT_DIAGONAL_ANGLE = -90  # Right diagonal sensor at -45 degrees
        self.FRONT_DIAGONAL_ANGLE = 0  # Front diagonal sensor at 0 degrees

        # the angle increment per sensor reading
        self.ANGLE_INCREMENT = 10  # Angle increment per reading is 1 degree

        # Calculate the index of each diagonal sensor based on angles
        self.LEFT_DIAGONAL_POS = round(self.LEFT_DIAGONAL_ANGLE / self.ANGLE_INCREMENT)
        self.RIGHT_DIAGONAL_POS = round(self.RIGHT_DIAGONAL_ANGLE / self.ANGLE_INCREMENT)
        self.FRONT_DIAGONAL_POS = round(self.FRONT_DIAGONAL_ANGLE / self.ANGLE_INCREMENT)

        # Last received scan data
        self.last_scan = None

        # Subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'input_scan',
            self.scan_callback,
            qos_profile_sensor_data)

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)

        # Timer for the control cycle
        self.timer = self.create_timer(0.05, self.control_cycle)

    def scan_callback(self, msg):
        """
        Callback for processing laser scan data.
        Saves the latest scan message for use in the control cycle.
        """
        self.last_scan = msg

    def control_cycle(self):
        """
        Main control cycle.
        This function is called periodically and implements the logic for moving the robot
        based on its current state and the sensor data.
        """
        if self.last_scan is None:
            return  # Do nothing if no scan data is available

        out_vel = Twist()  # Velocity command to be published

        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR  # Move forward

            if self.check_forward_2_stop():
                self.go_state(self.STOP)
            if self.check_forward_2_back():
                self.go_state(self.BACK)
            if self.check_diagonal_obstacles():
                self.go_state(self.TURN)

        elif self.state == self.BACK:
            out_vel.linear.x = -self.SPEED_LINEAR  # Move backward

            if self.check_back_2_turn():
                self.go_state(self.TURN)

        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR  # Turn

            if self.check_turn_2_forward():
                self.go_state(self.FORWARD)

        elif self.state == self.STOP:
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)

        # Example of logging state and velocity
        self.get_logger().info(f'Current state: {self.state}')
        # After setting out_vel based on the state
        self.get_logger().info(f'Publishing velocity: linear={out_vel.linear.x}, angular={out_vel.angular.z}')


        self.vel_pub.publish(out_vel)  # Publish the velocity command


    def go_state(self, new_state):
        """
        Transition to a new state.
        Sets the robot's current state to the new state and updates the timestamp.
        """
        self.state = new_state
        self.state_ts = self.get_clock().now()

    # The following methods implement checks for state transitions
    # based on the robot's current state and the sensor data.

    def check_forward_2_back(self):
        """Check if the robot should move back due to an obstacle in front."""
        pos = round(len(self.last_scan.ranges) / 2)
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        """Check if the robot should stop due to stale scan data."""
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed > Duration(seconds=self.SCAN_TIMEOUT)

    def check_stop_2_forward(self):
        """Check if the robot can move forward again after stopping."""
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)

    def check_back_2_turn(self):
        """Check if the robot should turn after moving back."""
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.BACKING_TIME)

    def check_turn_2_forward(self):
        """Check if the robot can move forward again after turning."""
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)
    
    def check_diagonal_obstacles(self):
        """Check if there are obstacles diagonally in front, left, and right."""
        if (self.last_scan.ranges[self.LEFT_DIAGONAL_POS] < self.OBSTACLE_DISTANCE and
            self.last_scan.ranges[self.RIGHT_DIAGONAL_POS] < self.OBSTACLE_DISTANCE and
            self.last_scan.ranges[self.FRONT_DIAGONAL_POS] < self.OBSTACLE_DISTANCE):
            return True
        else:
            return False
    
def main(args=None):
    """Main function to initialize the node and start ROS2 processing."""
    rclpy.init(args=args)  # Initialize ROS2
    bump_go_node = BumpGoNode()  # Create the BumpGoNode
    rclpy.spin(bump_go_node)  # Enter the ROS2 event loop
    bump_go_node.destroy_node()  # Cleanup the node
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()
