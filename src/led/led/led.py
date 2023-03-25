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
        GPIO.setup(self.PIN.value,GPIO.OUT)
        self.srv = self.create_service(Led, '~/led', self.callback)
        self.get_logger().info('[Node] Led Running')

    def callback(self, request, response):
        gpio_nr = request.gpio
        if request.led_state:
            self.get_logger().info('led on')
            GPIO.output(gpio_nr, GPIO.HIGH)
        else:
            self.get_logger().info('led off')
            GPIO.output(gpio_nr, GPIO.LOW)
        response.success = True 
        return response

def main():
    rclpy.init()

    _led = led()

    rclpy.spin(_led)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
