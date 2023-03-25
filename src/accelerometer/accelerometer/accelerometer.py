#!/usr/bin/env python
import math
import os

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

    # Motion detection
    THRESHOLD = 0.5
    # At least N_ABOVE_THRESHOLD samples above threshold among last WINDOWS_SIZE samples
    WIND0W_SIZE = 4
    N_ABOVE_THRESHOLD = 2
    G = 9.81
    

    def is_moving(self):
        above_th_count = 0
        for acc in self.past_samples:
            if abs(self.G - acc) > self.THRESHOLD:
                above_th_count += 1
            if above_th_count >= self.N_ABOVE_THRESHOLD:
                return True
        return False

    def __init__(self):
        super().__init__('accelerometer')
        self.past_samples = [self.G for _ in range(self.WIND0W_SIZE)]
        self.alarm_playing = False
        self.i2c = busio.I2C(board.SCL, board.SDA) 
        self.int1 = digitalio.DigitalInOut(board.D24)
        self.publisher_ = self.create_publisher(AccelStamped, self.ACC_PUB_TOPIC, 10)
        self.timer_period = 0.05     # 20Hz
        self.acc_timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(
            "Accelererometer publisher topic: {0}. Publishing frequency: {1}Hz.".format(self.publisher_.topic_name, 1e9 / self.acc_timer_.timer_period_ns))

    def timer_callback(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        lis3dh = adafruit_lis3dh.LIS3DH_I2C(self.i2c, int1=self.int1)
        x, y, z = lis3dh.acceleration

        msg.accel.linear.x = x
        msg.accel.linear.y = y
        msg.accel.linear.z = z

        acc_value = math.sqrt(x*x + y*y + z*z)
        self.past_samples.pop(0)
        self.past_samples.append(acc_value)
        if not self.alarm_playing and self.is_moving():
            self.get_logger().info('Triggering the alarm')
            alarm_playing = True
            self.destroy_timer(self.acc_timer_)
            self.play_timer_ = self.create_timer(0, lambda: self.play_alarm_callback())

        self.publisher_.publish(msg)

        if self.publisher_.get_subscription_count() > 0:
            self.get_logger().debug('Publishing: "%s"' % msg)

    def play_alarm_callback(self):
        self.destroy_timer(self.play_timer_)
        os.system('aplay ./examples/speaker/sounds/alarm.wav -D default:CARD=UACDemoV10')
        self.alarm_playing = False
        self.acc_timer_ = self.create_timer(self.timer_period, self.timer_callback)

def main():
    rclpy.init()

    pub_accelerometer = Pub_accelerometer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pub_accelerometer);
    executor.spin()

if __name__ == '__main__':
    main()