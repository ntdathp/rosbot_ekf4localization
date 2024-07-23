import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryToPath(Node):
    def __init__(self):
        super().__init__('odometry_to_path_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    odometry_to_path_node = OdometryToPath()
    rclpy.spin(odometry_to_path_node)
    odometry_to_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
