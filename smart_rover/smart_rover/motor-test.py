import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorTestPublisher(Node):
    def __init__(self):
        super().__init__('motor_test_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 1.0  # Cambia cada 1 segundo
        self.timer = self.create_timer(self.timer_period, self.publish_cmd)
        self.high_speed = True

    def publish_cmd(self):
        twist = Twist()
        if self.high_speed:
            twist.linear.x = 0.4  # Velocidad alta
            twist.angular.z = 0.8
            self.get_logger().info("Publicando velocidad ALTA")
        else:
            twist.linear.x = -0.4  # Velocidad baja
            twist.angular.z = 0.2
            self.get_logger().info("Publicando velocidad BAJA")
        self.publisher.publish(twist)
        self.high_speed = not self.high_speed  # Alternar velocidad

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()