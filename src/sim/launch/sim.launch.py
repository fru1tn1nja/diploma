#!/usr/bin/env python3
"""
Запускает Gazebo-11 с пустым миром, спавнит модель WAM-V с Fuel,
поднимает ros2_control и включает контроллер дифф-привода («thruster_controller»).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1. Gazebo 11 — gzserver + gzclient
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    world = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros_share, '/launch/gzserver.launch.py']),
        launch_arguments={'world': world}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros_share, '/launch/gzclient.launch.py'])
    )

    # 2. Спавним лодку WAM-V из Fuel (загружается автоматически)
    spawn_wamv = Node(
    package='gazebo_ros', executable='spawn_entity.py',
    arguments=['-entity', 'wamv', '-database', 'WAM-V'],
    output='screen')

    # 3. Подключаем контроллеры ros2_control
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # thruster_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['thruster_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_wamv,
        joint_state_broadcaster,
        # thruster_controller,
    ])
