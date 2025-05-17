import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

# IMPORTA TU MENSAJE PERSONALIZADO
from s4_custom_interface.msg import TwistArray

def cubic_spline_coeffs(p0, v0, p1, v1, tf):
    T = np.array([
        [1,    0,      0,        0],
        [0,    1,      0,        0],
        [1,   tf,   tf**2,   tf**3],
        [0,    1,   2*tf,  3*tf**2]
    ])
    X = np.array([p0, v0, p1, v1])
    a = np.linalg.solve(T, X)
    return a

def eval_cubic_spline(a, t):
    pos = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    return pos

class SplinePathPublisher(Node):
    def __init__(self):
        super().__init__('spline_path_publisher')
        self.path_pub = self.create_publisher(Path, '/spline/planned_path', 10)
        self.cmd_vels_pub = self.create_publisher(TwistArray, '/spline/cmd_vels', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.planned_path_sub = self.create_subscription(Path, '/planning/planned_path', self.planned_path_callback, 10)
        self.timer = self.create_timer(0.2, self.publish_path)
        self.current_pose = None

        self.get_logger().info("Nodo spline_path_publisher iniciado y esperando path en /planning/planned_path.")

        self.tf = 0.2
        self.path_points = []
        self.path_received = False

        self.current_segment = 0
        self.total_segments = 0
        self.segment_published = False

        self._last_info_time = self.get_clock().now()

    def info_throttled(self, msg):
        """Publica info solo si ha pasado al menos 0.5s desde el último info."""
        now = self.get_clock().now()
        if (now - self._last_info_time).nanoseconds * 1e-9 > 0.5:
            self.get_logger().info(msg)
            self._last_info_time = now

    def planned_path_callback(self, msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.total_segments = len(self.path_points) - 1
        self.current_segment = 0
        self.segment_published = False
        self.path_received = True
        self.info_throttled(f"Path recibido con {len(self.path_points)} puntos.")

        # Calcula velocidades deseadas para suavidad
        self.vx_points = [0]
        self.vy_points = [0]
        for i in range(1, len(self.path_points)-1):
            vx = 0.5 * ((self.path_points[i+1][0] - self.path_points[i-1][0]) / self.tf)
            vy = 0.5 * ((self.path_points[i+1][1] - self.path_points[i-1][1]) / self.tf)
            self.vx_points.append(vx)
            self.vy_points.append(vy)
        self.vx_points.append(0)
        self.vy_points.append(0)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def publish_path(self):
        if not self.path_received or self.total_segments < 1:
            return

        if self.current_segment >= self.total_segments:
            self.info_throttled("Todos los segmentos ya fueron publicados.")
            return

        # Si ya publiqué el segmento, espero a que el robot llegue al destino
        if self.segment_published:
            if self.current_pose is None:
                return
            x_goal, y_goal = self.path_points[self.current_segment + 1]
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            dist = np.hypot(x_goal - x, y_goal - y)
            if dist < 0.15:  # Tolerancia de llegada
                self.info_throttled(f"Llegó al punto {self.current_segment+1}, avanzando al siguiente segmento.")
                self.current_segment += 1
                self.segment_published = False
            return

        # Publicar el segmento actual
        x0, y0 = self.path_points[self.current_segment]
        x1, y1 = self.path_points[self.current_segment + 1]
        v0x = self.vx_points[self.current_segment]
        v1x = self.vx_points[self.current_segment + 1]
        v0y = self.vy_points[self.current_segment]
        v1y = self.vy_points[self.current_segment + 1]

        ax = cubic_spline_coeffs(x0, v0x, x1, v1x, self.tf)
        ay = cubic_spline_coeffs(y0, v0y, y1, v1y, self.tf)
        t_segment = np.linspace(0, self.tf, 20)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for t in t_segment:
            px = eval_cubic_spline(ax, t)
            py = eval_cubic_spline(ay, t)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.info_throttled(f"Segmento {self.current_segment+1} publicado con {len(t_segment)} puntos.")
        self.segment_published = True

        # Calcular y publicar cmd_vels como array
        cmd_vels = []
        for idx in range(1, len(t_segment)):
            dt = t_segment[idx] - t_segment[idx-1]
            if dt == 0:
                continue
            dx = eval_cubic_spline(ax, t_segment[idx]) - eval_cubic_spline(ax, t_segment[idx-1])
            dy = eval_cubic_spline(ay, t_segment[idx]) - eval_cubic_spline(ay, t_segment[idx-1])
            vx = dx / dt
            vy = dy / dt
            v = np.hypot(vx, vy)
            theta = np.arctan2(dy, dx)
            if idx > 1:
                prev_theta = np.arctan2(
                    eval_cubic_spline(ay, t_segment[idx-1]) - eval_cubic_spline(ay, t_segment[idx-2]),
                    eval_cubic_spline(ax, t_segment[idx-1]) - eval_cubic_spline(ax, t_segment[idx-2])
                )
                w = (theta - prev_theta) / dt
            else:
                w = 0.0
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            cmd_vels.append(twist)

        cmd_vels_msg = TwistArray()
        cmd_vels_msg.twists = cmd_vels
        self.cmd_vels_pub.publish(cmd_vels_msg)
        self.info_throttled(f"Publicado array de cmd_vels con {len(cmd_vels)} elementos.")

def main(args=None):
    rclpy.init(args=args)
    node = SplinePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()