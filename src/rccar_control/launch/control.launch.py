"""
Control Launch File
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Joystick node (joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[
            {'autorepeat_rate': 50.0},
            {'deadzone': 0.05},
        ]
    )
    
    # Gamepad controller node
    gamepad_node = Node(
        package='rccar_control',
        executable='gamepad_node.py',
        output='screen',
    )
    
    return LaunchDescription([
        joy_node,
        gamepad_node,
    ])
