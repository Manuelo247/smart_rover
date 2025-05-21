import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class StaticMapToOdomPublisher(Node):
    def __init__(self):
        super().__init__('static_map_to_odom_publisher')
        self.br = TransformBroadcaster(self)
        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Transformación de map a odom (ajusta según necesites)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Solo rotación en yaw (ej. 0 radianes)
        yaw = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapToOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

