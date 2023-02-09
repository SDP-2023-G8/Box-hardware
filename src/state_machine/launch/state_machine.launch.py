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
                ('/lock_srv', '/door_lock/lock')
            ],
            arguments=[
                '--ros-args', '--log-level', 'info'
            ],
        )
    ])