import numpy as np
if not hasattr(np, 'float'):
    np.float = float


import math
import tf_transformations

## Librerias de ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist as TwistMsg, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomSimulator(Node):
    def __init__(self):
        super().__init__('odom_simulator')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(f"Simulacion de odometria inicializada")   

        self.v = 0.0
        self.w = 0.0

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Limitar dt para evitar saltos grandes
        if dt > 0.2:
            dt = 0.2

        # Integrar movimiento
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        # Normalizar theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Crear mensaje de odometría
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist = TwistMsg(
            linear=Vector3(x=self.v, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.w)
        )

        self.odom_pub.publish(odom)
        
        # Publicar la transformación odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()