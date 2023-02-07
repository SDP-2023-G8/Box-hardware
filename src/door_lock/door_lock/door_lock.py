#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool

import magnet
from time import time, sleep


class LockService(Node):

    def __init__(self):
        super().__init__('lock_service')
        self.srv = self.create_service(SetBool, 'lock', self.callback)

    def callback(self, request, response):
        if (request.data):
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.data))
        else:
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.data))

        return True


def main():
    rclpy.init()
    lock_service = LockService()
    rclpy.spin(lock_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
