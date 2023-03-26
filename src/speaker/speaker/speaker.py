from std_srvs.srv import SetBool
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SpeakerService(Node):

    def __init__(self):
        super().__init__('speaker')
        self.srv = self.create_service(SetBool, '~/speaker', self.callback)

    def callback(self, request, response):
        sound_path = None
        if (request.data == True):
            self.get_logger().info('[ Sound ] door open')
            sound_path = os.path.join(get_package_share_directory('speaker'), 'door_open.wav')
        else:
            self.get_logger().info('[ Sound ] alarm')
            sound_path = os.path.join(get_package_share_directory('speaker'), 'alarm.wav')

        os.system('aplay {0} -D default:CARD=UACDemoV10'.format(sound_path))
        response.success = True 
        return response


def main():
    rclpy.init()

    speaker_service = SpeakerService()

    rclpy.spin(speaker_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
