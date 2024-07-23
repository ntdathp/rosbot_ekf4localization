import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CirclePath(Node):
    def __init__(self):
        super().__init__('circle_path_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0
        self.radius = 1.0  # Bán kính của hình tròn (m)
        self.speed = 0.2   # Tốc độ tuyến tính (m/s)
        self.angular_speed = self.speed / self.radius  # Tốc độ góc (rad/s)
        self.total_angle = 2 * math.pi  # 360 độ

    def timer_callback(self):
        msg = Twist()
        if self.angle < self.total_angle:
            msg.linear.x = self.speed
            msg.angular.z = self.angular_speed
            self.angle += self.angular_speed * 0.1
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.destroy_timer(self.timer)
            self.get_logger().info('Finished the circular path.')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_path_node = CirclePath()
    rclpy.spin(circle_path_node)
    circle_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
