import sys

from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, 'lock')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1])==1)
    minimal_client.get_logger().info(
            'input%d' %sys.argv[1])
    if (response.success):
        minimal_client.get_logger().info(
            'Result of request: True')
    else:
        minimal_client.get_logger().info(
            'Result of request: error')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
