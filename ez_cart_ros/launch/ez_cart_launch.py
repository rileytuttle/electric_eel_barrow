from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='ez_cart',
            executable='talker',
        ),
        Node(
            package='robot_controller',
            executable='robot',
        ),
        Node(
            package='image_tools',
            executable='cam2image',
        )
    ])
