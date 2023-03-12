import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

def generate_launch_description():
    qr_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('qr_scanner_pipeline'), 'qr_scanner.launch.py'
        )]),
    )
    
    door_lock_node = Node(
        name='lock_service',
        package='door_lock',
        executable='door_lock',
        output='both',
    )

    state_machine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('state_machine'), 'state_machine.launch.py'
        )]),
    )

    accelerometer_node = Node(
        name='accelerometer',
        package='accelerometer',
        executable='accelerometer',
        output='both'
    )

    led = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('led'), 'led.launch.py'
        )]),
    )

    speaker_node = Node(
        name='speaker',
        package='speaker',
        executable='speaker',
        output='both'
    )

    return LaunchDescription([
        qr_pipeline, 
        door_lock_node, 
        state_machine,
        accelerometer_node,
        led,
        speaker_node
    ])
