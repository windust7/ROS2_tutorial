from turtlesim.srv import Spawn

import rclpy
from rclpy.node import Node

import argparse
import sys

class AgentSpawner(Node):
    def __init__(self):
        super().__init__('agent_spawner')

        self.spawn_agent_service_client = self.create_client(
            Spawn,
            '/spawn'
        )

        while not self.spawn_agent_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The service not available.')

    def send_request(self, x, y, theta):
        service_request = Spawn.Request()
        service_request.x = x
        service_request.y = y
        service_request.theta = theta
        service_request.name = 'agent'
        futures = self.spawn_agent_service_client.call_async(service_request)
        return futures

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-x',
        '--init_x',
        type=float,
        default=1.0,
        help='initial x position of agent'
    )
    parser.add_argument(
        '-y',
        '--init_y',
        type=float,
        default=1.0,
        help='initial y position of agent'
    )
    parser.add_argument(
        '-t',
        '--theta',
        type=float,
        default=0.0,
        help='initial theta position of agent(rad)'
    )
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable'
    )
    args = parser.parse_args()

    rclpy.init(args=args.argv)

    spawner = AgentSpawner()
    future = spawner.send_request(args.init_x, args.init_y, args.theta)

    try:
        while rclpy.ok():
            rclpy.spin_once(spawner)
            if future.done():
                try:
                    service_response = future.result()
                except Exception as e:  # noqa: B902
                    spawner.get_logger().warn('Service call failed: {}'.format(str(e)))

    except KeyboardInterrupt:
        spawner.get_logger().info('Keyboard Interrupt (SIGINT)')

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()