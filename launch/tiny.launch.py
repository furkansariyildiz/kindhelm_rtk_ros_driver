import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kindhelm_rtk_ros_package',
            executable='tiny',
            name='kindhelm_tiny_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('kindhelm_rtk_ros_package'),
                'config', 'config.yaml')]
        )
    ])