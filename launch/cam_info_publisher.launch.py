from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the specific YAML file
    config_file_path = '/home/adwait/workspace/ros2_packages/cam_info_publisher/config/8380/18481924.yaml'

    # Describe the launch file
    # Parameter declaration to be loaded while launching the node
    # The path to the YAML file is passed as a parameter to the node
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config', default_value=config_file_path,
            description='Path to the camera configuration file'
        ),

        Node(
            package='cam_info_publisher',                # The ROS 2 package
            executable='cam_info_publisher',             # The compiled executable name
            name='cam_info_publisher',                   # Node name (can be the same as executable)
            output='screen',                             # Output to the terminal
            parameters=[config_file_path],               # Load the YAML file as parameters
        ),
    ])
