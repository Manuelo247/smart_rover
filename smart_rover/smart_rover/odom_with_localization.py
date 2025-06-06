import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Time
import tf2_geometry_msgs


class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/odom/localization_pose', 10)

    def odom_callback(self, msg):
        zero_time = Time(sec=0, nanosec=0)
        try:
            if self.tf_buffer.can_transform('odom', 'base_footprint', zero_time):
                transform = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint', zero_time,
                    timeout=rclpy.duration.Duration(seconds=1.0))

                # Pose del robot en odom
                pose_odom = PoseStamped()
                pose_odom.header.stamp = self.get_clock().now().to_msg()
                pose_odom.header.frame_id = 'odom'
                pose_odom.pose.position = Point(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    z=transform.transform.translation.z
                )
                pose_odom.pose.orientation = transform.transform.rotation

                # Transformar a frame 'map'
                pose_map = self.tf_buffer.transform(
                    pose_odom, 'map',
                    timeout=rclpy.duration.Duration(seconds=1.0))
                
                self.current_pose_pub.publish(pose_map)
            else:
                self.get_logger().warn("TF aún no conectado: odom -> base_footprint")
        except Exception as e:
            self.get_logger().warn(f'Error al transformar pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
