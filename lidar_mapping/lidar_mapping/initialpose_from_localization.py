import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_from_localization')
        self.sub = self.create_subscription(
            PoseStamped, 'odom/localization_pose', self.callback, 10)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 1)
        self.done = False

    def callback(self, msg):
        if self.done:
            return

        initialpose = PoseWithCovarianceStamped()
        initialpose.header.stamp = self.get_clock().now().to_msg()
        initialpose.header.frame_id = 'map'
        initialpose.pose.pose = msg.pose
        self.pub.publish(initialpose)
        self.get_logger().info('Initial pose publicado en /initialpose')
        self.done = True
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()