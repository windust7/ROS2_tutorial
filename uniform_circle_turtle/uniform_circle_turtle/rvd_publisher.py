import argparse
import sys

from rvd_msg_example.msg import RVD

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class CmdVelPublisher(Node):
    def __init__(self, ccwdirection, radius, lin_vel):
        super().__init__('rvd_publisher')
        self.ccwdirection = ccwdirection
        self.radius = radius
        self.lin_vel = lin_vel

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.rvd_publisher = self.create_publisher(
            RVD,
            'uniform_circular_velocity',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_rvd)

    def publish_rvd(self):
        msg = RVD()
        msg.ccwdirection = self.ccwdirection
        msg.radius = self.radius
        msg.lin_vel = self.lin_vel
        self.rvd_publisher.publish(msg)
        self.get_logger().info(f'Published CounterClockWise Direction: {msg.ccwdirection}')
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
        '--ccwdirection',
        type=int,
        default=1,
        help='0 for clockwise direction, 1 for counterclockwise direction'
    )
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable'
    )
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    try:
        ccwdirection = False if args.ccwdirection == 0 else True
        vel_generator = CmdVelPublisher(ccwdirection=ccwdirection, 
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
