import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

# Importa tu mensaje personalizado
# from rover_interfaces.msg import TwistArray  # Comentado: solo para spline

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Suscripción directa al path planeado
        self.path_sub = self.create_subscription(Path, '/planning/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.twistarray_sub = self.create_subscription(TwistArray, '/spline/cmd_vels', self.twistarray_callback, 10)  # Comentado: solo para spline
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose/local', 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel/desired', 10)
        self.path = []
        # self.cmd_vels = []  # Comentado: solo para spline
        self.current_idx = 0
        self.threshold = 0.15  # distancia para cambiar al siguiente punto
        self.current_pose = None

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"Nuevo path recibido con {len(self.path)} puntos.")

    # def twistarray_callback(self, msg):
    #     self.cmd_vels = msg.twists
    #     self.get_logger().info(f"TwistArray recibido con {len(self.cmd_vels)} velocidades.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.follow_path()

    def follow_path(self):
        if not self.path or self.current_pose is None or self.current_idx >= len(self.path):
            return
        target = self.path[self.current_idx].pose
        dx = target.position.x - self.current_pose.position.x
        dy = target.position.y - self.current_pose.position.y
        dist = np.hypot(dx, dy)
        if dist < self.threshold:
            self.current_idx += 1
            if self.current_idx >= len(self.path):
                self.get_logger().info("Ruta completada.")
                return
            target = self.path[self.current_idx].pose
        # Publicar el siguiente objetivo
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose = target
        self.goal_pub.publish(goal_msg)

        # Ya no publica cmd_vels de spline
        # if self.cmd_vels and self.current_idx < len(self.cmd_vels):
        #     self.cmd_vel_pub.publish(self.cmd_vels[self.current_idx])

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()