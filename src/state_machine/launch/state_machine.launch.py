from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='state_machine',
            executable='state_machine',
            name='state_machine',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/qr_msg', '/qr_code_node/qr_decoded'),
                ('/lock_srv', '/lock_service/lock'),
                ('/led_srv', '/led/led'),
                ('/speaker_srv', '/speaker/speaker')
            ],
            arguments=[
                '--ros-args', '--log-level', 'info'
            ],
        )
    ])