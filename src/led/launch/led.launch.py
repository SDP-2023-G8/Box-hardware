from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='led',
            executable='led',
            name='led',
            output='screen',
            emulate_tty=True,
        )
    ])
