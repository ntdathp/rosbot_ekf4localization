import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class SquarePathNode(Node):

    def __init__(self):
        super().__init__('squarepathnode')
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.pose = None
        self.state = 0
        self.side_length = 0.5
        self.turn_angle = math.pi / 2
        self.turn_tolerance = 0.01
        self.distance_tolerance = 0.01
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def timer_callback(self):
        if self.pose is None:
            return

        twist = Twist()

        if self.state % 2 == 0:  # Moving straight
            if self.start_x is None and self.start_y is None:
                self.start_x = self.pose.position.x
                self.start_y = self.pose.position.y

            distance = math.sqrt((self.pose.position.x - self.start_x) ** 2 +
                                 (self.pose.position.y - self.start_y) ** 2)
            if distance < self.side_length - self.distance_tolerance:
                twist.linear.x = 0.2  # Adjust this speed as needed
            else:
                twist.linear.x = 0.0
                self.start_x = None
                self.start_y = None
                self.state += 1

        else:  # Turning
            if self.start_theta is None:
                self.start_theta = self.get_yaw_from_quaternion(self.pose.orientation)

            current_theta = self.get_yaw_from_quaternion(self.pose.orientation)
            angle_diff = self.normalize_angle(self.start_theta + self.turn_angle - current_theta)
            if abs(angle_diff) > self.turn_tolerance:
                twist.angular.z = 0.2 if angle_diff > 0 else -0.2  # Adjust this speed as needed
            else:
                twist.angular.z = 0.0
                self.start_theta = None
                self.state += 1

        if self.state == 8:
            self.state = 0

        self.cmd_vel_publisher.publish(twist)

    def get_yaw_from_quaternion(self, quaternion):
        q_x = quaternion.x
        q_y = quaternion.y
        q_z = quaternion.z
        q_w = quaternion.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    square_path_node = SquarePathNode()
    rclpy.spin(square_path_node)
    square_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
