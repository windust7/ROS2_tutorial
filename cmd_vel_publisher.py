import argparse
import sys

from msg_interface_example.msg import UniformCircularVel

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class CmdVelPublisher(Node):
    def __init__(self, cwdirection, radius, lin_vel):
        super().__init__('cmd_vel_publisher')
        self.cwdirection = cwdirection
        self.radius = radius
        self.lin_vel = lin_vel

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.cmd_vel_publisher = self.create_publisher(
            UniformCircularVel,
            'uniform_circular_velocity',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        msg = UniformCircularVel()
        msg.cwdirection = self.cwdirection
        msg.radius = self.radius
        msg.lin_vel = self.lin_vel
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Published ClockWise Direction: {msg.cwdirection}')
        self.get_logger().info(f'Published Radius: {msg.radius}')
        self.get_logger().info(f'Published Linear Velocity: {msg.lin_vel}')

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-r',
        '--radius',
        type=float,
        default=1.0,
        help='radius of circle trajectory'
    )
    parser.add_argument(
        '-v',
        '--velocity',
        type=float,
        default=2.0,
        help='linear velocity of circle trajectory'
    )
    parser.add_argument(
        '-c',
        '--cwdirection',
        type=int,
        default=1,
        help='0 for counterclockwise direction, 1 for clockwisedirection'
    )
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable'
    )
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    try:
        cwdirection = False if args.cwdirection == 0 else True
        vel_generator = CmdVelPublisher(cwdirection=cwdirection, 
                                        radius=args.radius, 
                                        lin_vel=args.velocity)
        try:
            rclpy.spin(vel_generator)
        except KeyboardInterrupt:
            vel_generator.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            vel_generator.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
