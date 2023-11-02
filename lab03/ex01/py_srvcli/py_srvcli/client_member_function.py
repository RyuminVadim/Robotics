import sys

from tutorial_interfaces.srv import AddThreeName
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeName, 'add_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeName.Request()

    def send_request(self, a, b, c):
        self.req.first_name = a
        self.req.name = b
        self.req.last_name = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(str(sys.argv[1]), str(sys.argv[2]),str(sys.argv[3]))
    minimal_client.get_logger().info(
        'Result of add_name: for %s + %s + %s = %s' %
        (str(sys.argv[1]), str(sys.argv[2]),str(sys.argv[3]), response.full_name))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
