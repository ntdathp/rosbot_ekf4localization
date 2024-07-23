import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquarePath(Node):
    def __init__(self):
        super().__init__('square_path_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.state = 0
        self.edge_length = 2.0  # Độ dài cạnh của hình vuông (m)
        self.speed = 0.2  # Tốc độ tuyến tính (m/s)
        self.turn_speed = 0.5  # Tốc độ góc để quay 90 độ (rad/s)
        self.edge_duration = self.edge_length / self.speed  # Thời gian đi hết một cạnh
        self.turn_duration = (3.14159 / 2) / self.turn_speed  # Thời gian quay 90 độ
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        msg = Twist()

        if self.state < 4:
            if elapsed_time < (self.edge_duration + self.turn_duration) * (self.state + 1):
                if elapsed_time % (self.edge_duration + self.turn_duration) < self.edge_duration:
                    msg.linear.x = self.speed
                    msg.angular.z = 0.0
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = self.turn_speed
            else:
                self.state += 1
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.destroy_timer(self.timer)
            self.get_logger().info('Finished the square path.')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    square_path_node = SquarePath()
    rclpy.spin(square_path_node)
    square_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
