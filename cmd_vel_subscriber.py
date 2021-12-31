from msg_interface_example.msg import UniformCircularVel
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

_eps = 1e-7

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.cwdirection = True
        self.radius = 1.0
        self.lin_vel = 0.0

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.lin_z = 0.0
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0

        self.lin_vec = None
        self.ang_vec = None
        self.twist = None

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.cmd_vel_subscriber = self.create_subscription(
            UniformCircularVel,
            'uniform_circular_velocity',
            self.get_uniform_circular_vel,
            QOS_RKL10V)

        self.twist_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_twist_msg)

    def get_uniform_circular_vel(self, msg):
        self.cwdirection = msg.cwdirection
        self.radius = msg.radius
        self.lin_vel = msg.lin_vel
        self.get_logger().info('Subscribed msgs!')
        self.get_logger().info(f'\tClockWise Direction: {msg.cwdirection}')
        self.get_logger().info(f'\tRadius: {msg.radius}')
        self.get_logger().info(f'\tLinear Velocity: {msg.lin_vel}')

    def publish_twist_msg(self):
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()
        self.twist = Twist()

        self.lin_x = self.lin_vel if self.cwdirection == True else -(self.lin_vel)
        self.ang_z = (self.lin_x / (self.radius + _eps)) if self.cwdirection == True else -(self.lin_x / (self.radius + _eps))

        self.lin_vec.x, self.lin_vec.y, self.lin_vec.z = self.lin_x, self.lin_y, self.lin_z
        self.ang_vec.x, self.ang_vec.y, self.ang_vec.z = self.ang_x, self.ang_y, self.ang_z
        self.twist.linear = self.lin_vec
        self.twist.angular = self.ang_vec

        self.twist_publisher.publish(self.twist)
        self.get_logger().info(f'Published Linear Velocity: {self.lin_x}')
        self.get_logger().info(f'Published Angular Velocity: {self.ang_z}')

def main(args=None):
    rclpy.init(args=args)
    try:
        vel_transformer = CmdVelSubscriber()
        try:
            rclpy.spin(vel_transformer)
        except KeyboardInterrupt:
            vel_transformer.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            vel_transformer.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
