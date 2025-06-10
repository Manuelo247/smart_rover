import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Suscripción directa al path planeado
        self.path_sub = self.create_subscription(Path, 'planning/planned_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'odom/localization_pose', self.pose_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose/local', 10)
        self.path = []
        self.current_idx = 0
        self.threshold = 0.15  # distancia para cambiar al siguiente punto
        self.current_pose = None

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"Nuevo path recibido con {len(self.path)} puntos.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
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

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()