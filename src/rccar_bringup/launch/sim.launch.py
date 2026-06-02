"""
Simulation Bringup Launch File (Gazebo + Control)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rccar_gazebo_pkg = FindPackageShare('rccar_gazebo')
    rccar_control_pkg = FindPackageShare('rccar_control')
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [rccar_gazebo_pkg, 'launch', 'gazebo.launch.py'])),
    )
    
    # Control launch (gamepad + joy)
    #control_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution(
    #            [rccar_control_pkg, 'launch', 'control.launch.py'])),
    #)
    
    return LaunchDescription([
        gazebo_launch,
        #control_launch,
    ])
