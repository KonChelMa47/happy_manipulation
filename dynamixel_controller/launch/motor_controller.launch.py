from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_controller',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=['../../config/mimi_specification.yaml']
        )
    ])