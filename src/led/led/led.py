from led_interfaces.srv import Led
import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from time import sleep 
import time


class led(Node):

    def __init__(self):
        super().__init__('led')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.srv = self.create_service(Led, '~/led', self.callback)
        self.get_logger().info('[Node] Led Running')

    def callback(self, request, response):
        try:
            gpio_nr = request.gpio
            GPIO.setup(gpio_nr, GPIO.OUT)
            if request.led_state:
                self.get_logger().info('led {0} on'.format(gpio_nr))
                GPIO.output(gpio_nr, GPIO.HIGH)
            else:
                self.get_logger().info('led {0} off'.format(gpio_nr))
                GPIO.output(gpio_nr, GPIO.LOW)
            response.success = True
        except:
            response.success = False
        return response

def main():
    rclpy.init()

    _led = led()

    rclpy.spin(_led)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
