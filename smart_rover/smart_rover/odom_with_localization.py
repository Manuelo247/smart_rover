import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclToPose(Node):
    def __init__(self):
        super().__init__('amcl_to_pose_publisher')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)
        self.pub = self.create_publisher(PoseStamped, 'odom/localization_pose', 10)

    def amcl_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose  # Ignoramos la covarianza

        self.pub.publish(pose_stamped)
        self.get_logger().debug("Publicada pose en /odom/localization_pose")

def main(args=None):
    rclpy.init(args=args)
    node = AmclToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()