from turtlesim.srv import Spawn

import rclpy
from rclpy.node import Node

import math

class AgentSpawner(Node):
    def __init__(self):
        super().__init__('agent_spawner')

        self.spawn_agent_service_client = self.create_client(
            Spawn,
            '/spawn'
        )

        while not self.spawn_agent_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The service not available.')

    def send_request(self):
        service_request = Spawn.Request()
        service_request.x = 1.0
        service_request.y = 1.0
        service_request.theta = math.pi / 8.0
        service_request.name = 'agent'
        futures = self.spawn_agent_service_client.call_async(service_request)
        return futures

def main(args=None):
    rclpy.init(args=args)
    spawner = AgentSpawner()
    future = spawner.send_request()

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