"""
Hardware Bringup Launch File
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rccar_control_pkg = FindPackageShare('rccar_control')
    
    # Control launch (gamepad + joy)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [rccar_control_pkg, 'launch', 'control.launch.py'])),
    )
    
    # TODO: Add hardware driver nodes here (motor controller, sensors, etc.)
    
    return LaunchDescription([
        control_launch,
    ])
