from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization',
            executable='test_frame_node',
            name='test_frame_node',
            output="screen"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz'
        ),
    ])