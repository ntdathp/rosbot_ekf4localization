import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class CirclePathClosedLoop(Node):
    def __init__(self):
        super().__init__('circle_path_closed_loop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        self.target_radius = 1.0  # Bán kính của hình tròn (m)
        self.speed = 0.2  # Tốc độ tuyến tính (m/s)
        self.kp = 1.0  # Hằng số điều khiển tỉ lệ cho vòng kín
        self.current_pose = None

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw
        }
        self.control_loop()

    def control_loop(self):
        if self.current_pose is None:
            return

        # Tính toán vị trí mong muốn trên quỹ đạo hình tròn
        time_now = self.get_clock().now().seconds_nanoseconds()[0]
        desired_x = self.target_radius * math.cos(self.speed * time_now / self.target_radius)
        desired_y = self.target_radius * math.sin(self.speed * time_now / self.target_radius)

        # Tính toán sai số
        error_x = desired_x - self.current_pose['x']
        error_y = desired_y - self.current_pose['y']
        error_distance = math.sqrt(error_x**2 + error_y**2)

        # Tính góc mong muốn
        desired_theta = math.atan2(error_y, error_x)

        # Tính toán góc quay cần thiết
        angle_diff = desired_theta - self.current_pose['theta']
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Điều khiển vòng kín
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.kp * angle_diff

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_path_closed_loop_node = CirclePathClosedLoop()
    rclpy.spin(circle_path_closed_loop_node)
    circle_path_closed_loop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
