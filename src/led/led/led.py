from std_srvs.srv  import SetBool
import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from time import sleep 
import time


class led(Node):

    def __init__(self):
        super().__init__('led')
        self.PIN = self.declare_parameter("PIN",14)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN.value,GPIO.OUT)
        self.srv = self.create_service(SetBool, '~/led', self.callback)
        self.get_logger().info('[Node] Led Running')

    def callback(self, request, response):
        if (request.data == True):
            self.get_logger().info('led on')
            GPIO.output(self.PIN.value,GPIO.HIGH)

        else:
            self.get_logger().info('led off')

            GPIO.output(self.PIN.value,GPIO.LOW)
        response.success = True 
        return response

def main():
    rclpy.init()

    _led = led()

    rclpy.spin(_led)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
