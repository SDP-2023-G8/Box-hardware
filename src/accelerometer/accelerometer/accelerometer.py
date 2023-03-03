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

    ACC_PUB_TOPIC = '~/acc'

    def __init__(self):
        super().__init__('accelerometer')
        self.i2c = busio.I2C(board.SCL, board.SDA) 
        self.int1 = digitalio.DigitalInOut(board.D24)
        self.publisher_ = self.create_publisher(AccelStamped, self.ACC_PUB_TOPIC, 10)
        timer_period = 0.05     # 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            "Accelererometer publisher topic: {0}. Publishing frequency: {1}Hz.".format(self.publisher_.topic_name, 1e9 / self.timer.timer_period_ns))

    def timer_callback(self):
        if self.publisher_.get_subscription_count() == 0:
            return
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        lis3dh = adafruit_lis3dh.LIS3DH_I2C(self.i2c, int1=self.int1)
        x, y, z = lis3dh.acceleration

        msg.accel.linear.x = x
        msg.accel.linear.y = y
        msg.accel.linear.z = z

        self.publisher_.publish(msg)
        self.get_logger().debug('Publishing: "%s"' % msg)


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