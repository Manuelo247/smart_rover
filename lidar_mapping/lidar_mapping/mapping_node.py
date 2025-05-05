import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np

from gridmap import OccupancyGridMap
from sensor_model import inverse_sensor_model
from utils import bresenham

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 400)

        resolution = self.get_parameter("resolution").get_parameter_value().double_value
        width = self.get_parameter("width").get_parameter_value().integer_value
        height = self.get_parameter("height").get_parameter_value().integer_value

        self.map = OccupancyGridMap(width, height, resolution)
        self.pose = (0.0, 0.0, 0.0)

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = (pos.x, pos.y, yaw)

    def scan_callback(self, msg):
        x, y, theta = self.pose
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                end_x = x + r * np.cos(theta + angle)
                end_y = y + r * np.sin(theta + angle)

                free_cells = bresenham(*self.map.world_to_map(x, y), *self.map.world_to_map(end_x, end_y))
                for cx, cy in free_cells[:-1]:
                    wx = (cx - self.map.width // 2) * self.map.resolution
                    wy = (cy - self.map.height // 2) * self.map.resolution
                    self.map.update_cell(wx, wy, inverse_sensor_model(False))

                self.map.update_cell(end_x, end_y, inverse_sensor_model(True))
            angle += msg.angle_increment

