from std_srvs.srv  import SetBool
import os
import rclpy
from rclpy.node import Node


class SpeakerService(Node):

    def __init__(self):
        super().__init__('speaker')
        self.srv = self.create_service(SetBool, '~/speaker', self.callback)

    def callback(self, request, response):
        if (request.data == True):
            self.get_logger().info('[ Sound ] door open')
            os.system('aplay ./examples/speaker/sounds/door_open.wav -D default:CARD=UACDemoV10')
        else:
            self.get_logger().info('[ Sound ] alarm')
            os.system('aplay ./examples/speaker/sounds/alarm.wav -D default:CARD=UACDemoV10')
        response.success = True 
        return response


def main():
    rclpy.init()

    speaker_service = SpeakerService()

    rclpy.spin(speaker_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
