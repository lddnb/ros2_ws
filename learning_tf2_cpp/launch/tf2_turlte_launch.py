from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_cpp',
            executable='learning_tf2_cpp',
            name='learning_tf2'
        ),
        Node(
            package='learning_tf2_cpp',
            executable='sendtf',
            name='sendturtle1',
            parameters=[{
                'turtle_name': 'turtle1'
            }]
        ),
        Node(
            package='learning_tf2_cpp',
            executable='sendtf',
            name='sendturtle2',
            parameters=[{
                'turtle_name': 'turtle2'
            }]
        ),
    ])