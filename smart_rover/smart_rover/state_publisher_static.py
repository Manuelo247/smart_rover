# static_joint_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StaticJointPublisher(Node):
    def __init__(self):
        super().__init__('static_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish)

    def publish(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'left_back_joint',
            'left_front_joint',
            'right_back_joint',
            'right_front_joint'
        ]
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
