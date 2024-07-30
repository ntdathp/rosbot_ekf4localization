import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class FrameIdChanger(Node):
    def __init__(self):
        super().__init__('frame_id_changer')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',  # Thay đổi 'input_topic' bằng tên topic đầu vào
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu', 10)  # Thay đổi 'output_topic' bằng tên topic đầu ra
        self.new_frame_id = 'base_link'  # Thay đổi tên frame_id mới ở đây

    def listener_callback(self, msg):
        new_msg = Imu()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = self.new_frame_id
        new_msg.orientation = msg.orientation
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    frame_id_changer = FrameIdChanger()
    rclpy.spin(frame_id_changer)
    frame_id_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
