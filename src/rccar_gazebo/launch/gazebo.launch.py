"""
Gazebo Simulation Launch File
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 1. ⚠️ 아래 ParameterValue 임포트 라인을 추가했습니다.
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package names
    rccar_description_pkg = FindPackageShare('rccar_description')
    rccar_gazebo_pkg = FindPackageShare('rccar_gazebo')
    
    # Get URDF file
    urdf_file = PathJoinSubstitution(
        [rccar_description_pkg, 'urdf', 'rccar_sim.urdf.xacro'])
    
    # Get world file
    world_file = PathJoinSubstitution(
        [rccar_gazebo_pkg, 'worlds', 'bookstore.world'])

    # Add our package models to Gazebo model path
    gazebo_model_path = PathJoinSubstitution([rccar_gazebo_pkg, 'models'])
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', gazebo_model_path]
    )
    robot_description_content = Command(
        ['xacro ', urdf_file])
    
    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file')
    
    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )
    
    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])),
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            # 2. ⚠️ xacro 데이터를 ParameterValue(..., value_type=str)로 감싸주었습니다.
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # Gazebo spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rccar',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        set_gazebo_model_path,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
    ])