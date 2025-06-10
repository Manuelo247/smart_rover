#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.srv import GetTwoPoses
from s7_robot_network_interface.msg import RobotStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.get_logger().info('Pose Publisher node has been started')

        # --- Service client setup ---
        self.client = self.create_client(GetTwoPoses, '/get_two_poses')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for service. Exiting.')
                return
            self.get_logger().info('Waiting for service to become available...')
        self.req = GetTwoPoses.Request()

        # --- Robot ID ---
        self.robot_id = 2

        # --- Robot state ---
        self.x = 0.3
        self.y = 3.56
        self.yaw = 0.0
        self.stage = 0

        # --- Fetch initial poses ---
        self.get_new_poses()

        # --- Publishers ---
        self.robot_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'goal_pose/global', 10)
        self.target_yaw_pub = self.create_publisher(Float32, 'goal_pose/yaw', 10)

        # --- Subscriber to amcl_pose ---
        self.amcl_pose = None
        self.amcl_pose_received = False
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # --- Subscriber to distance_to_goal ---
        self.distance_to_goal = None
        self.distance_to_goal_sub = self.create_subscription(
            Float32,
            'distance_to_goal',
            self.distance_to_goal_callback,
            10
        )

        # --- Subscriber to planned_path ---
        self.path_received = False
        self.planned_path_sub = self.create_subscription(
            Path,
            '/planning/planned_path',
            self.planned_path_callback,
            10
        )

        # --- Timer for periodic updates (~30 Hz) ---
        self.timer = self.create_timer(0.033, self.publish)

        # Banderas para publicación única
        self.pickup_goal_sent = False
        self.pickup_yaw_sent = False
        self.delivery_goal_sent = False
        self.delivery_yaw_sent = False

        self.last_stage_time = self.get_clock().now()

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
        if not self.amcl_pose_received:
            self.get_logger().info('Primer mensaje de amcl_pose recibido, iniciando lógica principal.')
        self.amcl_pose_received = True

    def distance_to_goal_callback(self, msg):
        self.distance_to_goal = msg.data

    def planned_path_callback(self, msg):
        self.path_received = True

    def get_new_poses(self):
        self.req.robot_id = self.robot_id
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        resp = future.result()
        if resp is None:
            self.get_logger().error('Failed to call service get_two_poses')
            return
        self.pickup_pose = resp.pickup_pose
        self.delivery_pose = resp.delivery_pose
        self.pickup_yaw = self.pickup_pose.theta
        self.delivery_yaw = self.delivery_pose.theta
        self.get_logger().info(
            f'New pickup_pose: ({self.pickup_pose.x:.2f}, {self.pickup_pose.y:.2f}, {self.pickup_pose.theta:.2f} rad)' 
        )
        self.get_logger().info(
            f'New delivery_pose: ({self.delivery_pose.x:.2f}, {self.delivery_pose.y:.2f}, {self.delivery_pose.theta:.2f} rad)'  
        )

    def update_pose(self):
        if self.amcl_pose is not None:
            self.x = self.amcl_pose.pose.pose.position.x
            self.y = self.amcl_pose.pose.pose.position.y
            q = self.amcl_pose.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
        return
    
    def publish_target_position(self, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        self.target_pose_pub.publish(pose_msg)

    def publish_target_orientation(self, yaw):
        yaw_msg = Float32()
        yaw_msg.data = float(yaw)
        self.target_yaw_pub.publish(yaw_msg)

    def set_target_position(self):
        if self.stage == 0:
            self.publish_target_position(self.pickup_pose.x, self.pickup_pose.y)
            print(f"Publicando posicion objetivo de pickup: {self.pickup_pose.x}, {self.pickup_pose.y}")
        elif self.stage == 1:
            self.publish_target_position(self.delivery_pose.x, self.delivery_pose.y)
            print(f"Publicando posicion objetivo de delivery: {self.delivery_pose.x}, {self.delivery_pose.y}")
        return

    def set_target_orientation(self):
        if self.stage == 1:
            self.publish_target_orientation(self.pickup_yaw)
        elif self.stage == 4:
            self.publish_target_orientation(self.delivery_yaw)
        return

    def position_achieved(self):
        if self.distance_to_goal is not None and self.distance_to_goal < 0.10:
            print(self.distance_to_goal)
            return True
        else:
            return False
        
    def orientation_achieved(self):
        # Aquí deberías poner la lógica real de orientación
        # Por ahora, siempre retorna False para evitar avanzar de estado
        return True

    def can_advance_stage(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_stage_time).nanoseconds / 1e9
        return elapsed >= 2.0

    def publish(self):
        if not self.amcl_pose_received:
            return

        # --- State machine logic ---
        if self.stage == 0:
            if not self.pickup_goal_sent:
                self.publish_target_position(self.pickup_pose.x, self.pickup_pose.y)
                print(f"Publicando posicion objetivo de pickup: {self.pickup_pose.x}, {self.pickup_pose.y}")
                self.pickup_goal_sent = True
                self.path_received = False  # Espera nuevo path
                self.last_stage_time = self.get_clock().now()
            # Solo avanza si recibió path y pasaron 2 segundos
            if self.path_received and self.can_advance_stage():
                self.stage = 1
                self.pickup_yaw_sent = False
                self.last_stage_time = self.get_clock().now()

        elif self.stage == 1:  # ALCANZAR PICKUP
            self.update_pose()
            if self.position_achieved() and self.can_advance_stage():
                self.get_logger().info('Posicion pickup alcanzada')
                self.stage = 2
                self.last_stage_time = self.get_clock().now()

        elif self.stage == 2:  # ORIENTARSE AL PICKUP
            self.update_pose()
            if not self.pickup_yaw_sent:
                self.publish_target_orientation(self.pickup_yaw)
                self.pickup_yaw_sent = True
            if self.orientation_achieved() and self.can_advance_stage():
                self.get_logger().info('Orientacion pickup alcanzada')
                self.stage = 3
                self.delivery_goal_sent = False
                self.last_stage_time = self.get_clock().now()

        elif self.stage == 3:
            self.stage = 4
            self.delivery_yaw_sent = False
            self.last_stage_time = self.get_clock().now()

        elif self.stage == 4:  # ALCANZAR DELIVERY (espera path)
            if not self.delivery_goal_sent:
                self.publish_target_position(self.delivery_pose.x, self.delivery_pose.y)
                print(f"Publicando posicion objetivo de delivery: {self.delivery_pose.x}, {self.delivery_pose.y}")
                self.delivery_goal_sent = True
                self.path_received = False  # Espera nuevo path
                self.last_stage_time = self.get_clock().now()
            # Solo avanza si recibió path y pasaron 2 segundos
            if self.path_received and self.can_advance_stage():
                self.stage = 5
                self.last_stage_time = self.get_clock().now()

        elif self.stage == 5:  # ORIENTARSE AL DELIVERY
            self.update_pose()
            if not self.delivery_yaw_sent:
                self.publish_target_orientation(self.delivery_yaw)
                self.delivery_yaw_sent = True
            if self.position_achieved() and self.can_advance_stage():
                self.get_logger().info('Orientacion delivery alcanzada')
                self.stage = 6
                self.last_stage_time = self.get_clock().now()

        elif self.stage == 6:  # NUEVA PETICION Y VUELTA A EMPEZAR
            if self.can_advance_stage():
                self.get_logger().info('Completed delivery. Requesting new goals...')
                self.get_new_poses()
                self.stage = 0
                self.pickup_goal_sent = False
                self.pickup_yaw_sent = False
                self.delivery_goal_sent = False
                self.delivery_yaw_sent = False
                self.last_stage_time = self.get_clock().now()

        # --- Publish the current Robot Status ---
        robot_msg = RobotStatus()
        robot_msg.timestamp = self.get_clock().now().to_msg()
        robot_msg.robot_id = self.robot_id
        robot_msg.task_stage = self.stage
        robot_msg.pose.x = self.x
        robot_msg.pose.y = self.y
        robot_msg.pose.theta = self.yaw
        robot_msg.align_yaw = True
        self.robot_pub.publish(robot_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()