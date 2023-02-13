#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import AccelStamped

import time
import board
import digitalio
import busio
import adafruit_lis3dh

class Pub_accelerometer(Node):
    def __init__(self):
        super().__init__('accelerometer_publisher')
        self.publisher_ = self.create_publisher(AccelStamped, '/accelerometer/acc', 10)
        timer_period = 0.01  # in seconds, 10ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        i2c = busio.I2C(board.SCL, board.SDA)                # Remove this line if using SPI
        int1 = digitalio.DigitalInOut(board.D24)             # Remove this line if using SPI
        lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)
        x, y, z = lis3dh.acceleration

        msg.accel.linear.x += 'acc x: %d\n' % x
        msg.accel.linear.y += 'acc y: %d\n' % y
        msg.accel.linear.z += 'acc z: %d\n' % z

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main():
    rclpy.init()

    pub_accelerometer = Pub_accelerometer()

    rclpy.spin(pub_accelerometer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_accelerometer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()