from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = {
    'display': False,
    'cam_fps': 24
}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_scanner_pipeline',
            executable='qr_scanner',
            name='qr_code_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                PARAMS
                ]
        )
    ])
