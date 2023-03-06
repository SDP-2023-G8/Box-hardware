from std_srvs.srv  import SetBool
import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from time import sleep 
PIN = 14
import time


class led(Node):

    def __init__(self):
        super().__init__('led')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN,GPIO.OUT)
        self.srv = self.create_service(SetBool, '~/led', self.callback)

    def callback(self, request, response):
        if (request.data == True):
            self.get_logger().info('led on')
            GPIO.output(PIN,GPIO.HIGH)

        else:
            self.get_logger().info('led off')
            GPIO.output(PIN,GPIO.LOW)
        response.success = True 
        return response

def main():
    rclpy.init()

    _led = led()

    rclpy.spin(_led)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
