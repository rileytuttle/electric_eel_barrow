from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
        ),
        Node(
            package='ez_cart',
            executable='talker',
        ),
        Node(
            package='robot_controller',
            executable='robot',
        )
    ])
