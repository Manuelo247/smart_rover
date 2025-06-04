import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # FOV y distancia de seguridad por lado
        self.declare_parameter('fov_front', 60.0)
        self.declare_parameter('fov_back', 60.0)
        self.declare_parameter('fov_left', 60.0)
        self.declare_parameter('fov_right', 60.0)

        self.declare_parameter('safety_dist_front', 0.2)
        self.declare_parameter('safety_dist_back', 0.16)
        self.declare_parameter('safety_dist_left', 0.16)
        self.declare_parameter('safety_dist_right', 0.16)

        # Duración de evasión por lado (valores diferentes)
        self.declare_parameter('evasion_duration_front', 1.2)
        self.declare_parameter('evasion_duration_back', 1.2)
        self.declare_parameter('evasion_duration_left', 1.8)
        self.declare_parameter('evasion_duration_right', 1.8)

        self.declare_parameter('safety_wait_time', 2.0)
        self.declare_parameter('evasion_speed', 0.2)
        self.declare_parameter('active_directions', [])
        #        self.declare_parameter('active_directions', ['front', 'back', 'right', 'left'])

        self.obstacle_detected = False
        self.pause_timer = None
        self.move_timer = None
        self.post_move_timer = None
        self.last_distances = {}

        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription_control = self.create_subscription(Twist, '/control', self.control_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def control_callback(self, msg):
        if not self.obstacle_detected:
            self.cmd_vel_pub.publish(msg)
        else:
            self.get_logger().info('Obstáculo presente. Ignorando comando de /control.')

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) * 180.0 / np.pi
        angles = (angles + 360) % 360

        distances = {}
        for side, center_angle in {'front': 0.0, 'back': 180.0, 'left': 90.0, 'right': 270.0}.items():
            fov = self.get_parameter(f'fov_{side}').get_parameter_value().double_value
            lower = (center_angle - fov / 2.0) % 360
            upper = (center_angle + fov / 2.0) % 360
            if lower < upper:
                indices = np.where((angles >= lower) & (angles <= upper))[0]
            else:
                indices = np.where((angles >= lower) | (angles <= upper))[0]

            valid = ranges[indices]
            valid = valid[(valid > 0.0) & np.isfinite(valid)]  # ignora ceros e inf
            distances[side] = np.nanmin(valid) if valid.size > 0 else float('inf')

        self.last_distances = distances

        self.get_logger().info(
            f"Front: {distances['front']:.2f} m | Back: {distances['back']:.2f} m | "
            f"Left: {distances['left']:.2f} m | Right: {distances['right']:.2f} m"
        )

        safety_dist = {s: self.get_parameter(f'safety_dist_{s}').get_parameter_value().double_value for s in distances}
        active_directions = self.get_parameter('active_directions').get_parameter_value().string_array_value

        if any(distances[d] < safety_dist[d] for d in active_directions if d in distances):
            if not self.obstacle_detected:
                self.get_logger().warn('¡Obstáculo detectado! Deteniendo el robot.')
                self.stop_robot()
                self.obstacle_detected = True
                wait_time = self.get_parameter('safety_wait_time').get_parameter_value().double_value
                self.pause_timer = self.create_timer(wait_time, self.after_pause_check)
        else:
            if self.obstacle_detected and all(t is None for t in [self.pause_timer, self.move_timer, self.post_move_timer]):
                self.get_logger().info('Obstáculo despejado. Reanudando movimiento.')
                self.obstacle_detected = False

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def after_pause_check(self):
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None

        distances = self.last_distances
        safety_dist = {s: self.get_parameter(f'safety_dist_{s}').get_parameter_value().double_value for s in distances}
        active_directions = self.get_parameter('active_directions').get_parameter_value().string_array_value

        if any(distances[d] < safety_dist[d] for d in active_directions if d in distances):
            direction = self.choose_direction(distances, safety_dist, active_directions)
            if direction:
                self.get_logger().warn(f'Moviendo hacia {direction} para evitar obstáculo.')
                self.move_in_direction(direction)
                evasion_time = self.get_parameter(f'evasion_duration_{direction}').get_parameter_value().double_value
                self.move_timer = self.create_timer(evasion_time, self.after_move)
            else:
                self.get_logger().info('Ninguna dirección libre. Reevaluando...')
                wait_time = self.get_parameter('safety_wait_time').get_parameter_value().double_value
                self.pause_timer = self.create_timer(wait_time, self.after_pause_check)
        else:
            self.get_logger().info('Obstáculo despejado. Reanudando movimiento.')
            self.obstacle_detected = False

    def after_move(self):
        if self.move_timer:
            self.move_timer.cancel()
            self.move_timer = None
        self.stop_robot()
        wait_time = self.get_parameter('safety_wait_time').get_parameter_value().double_value
        self.get_logger().info(f'Esperando {wait_time} s después del movimiento...')
        self.post_move_timer = self.create_timer(wait_time, self.after_post_move)

    def after_post_move(self):
        if self.post_move_timer:
            self.post_move_timer.cancel()
            self.post_move_timer = None
        self.after_pause_check()

    def choose_direction(self, distances, safety_dist, active_directions):
        priority = ['front', 'back', 'right', 'left']
        for d in priority:
            if d in active_directions and distances.get(d, 0.0) > safety_dist[d]:
                return d
        return None

    def move_in_direction(self, direction):
        speed = self.get_parameter('evasion_speed').get_parameter_value().double_value
        msg = Twist()
        if direction == 'front':
            msg.linear.x = speed
        elif direction == 'back':
            msg.linear.x = -speed
        elif direction == 'right':
            msg.linear.y = -speed
        elif direction == 'left':
            msg.linear.y = speed
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
