from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam_info_publisher',
            executable='cam_info_publisher',
            name='camera_info_pub',
            output='screen'
        )
    ])
