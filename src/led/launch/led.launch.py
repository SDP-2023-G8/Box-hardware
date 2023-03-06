from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = {
    'PIN': 14
}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='led',
            executable='led',
            name='led',
            output='screen',
            emulate_tty=True,
            parameters=[
                PARAMS
                ]
        )
    ])
