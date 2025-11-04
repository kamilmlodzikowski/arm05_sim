#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np


def generate_launch_description():
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
    )
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
    )
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    arm05_share_dir = get_package_share_directory('arm05_sim')

    models_path = os.path.join(arm05_share_dir, 'models')
    existing_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_gazebo_path:
        gazebo_model_path = models_path + os.pathsep + existing_gazebo_path
    else:
        gazebo_model_path = models_path

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    aruco_seed = LaunchConfiguration('aruco_seed', default='')

    aruco_seed_launch_arg = DeclareLaunchArgument(
        'aruco_seed',
        default_value=str(np.random.randint(0, 9999999)),
        description="Seed for aruco spawner"
    )

    world = os.path.join(arm05_share_dir, 'worlds', 'arm_house.world')
    map_file = os.path.join(arm05_share_dir, 'map', 'map.yaml')
    params_file = os.path.join(arm05_share_dir, 'param', 'waffle.yaml')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    spawn_aruco_cubes = Node(
        package='arm05_sim',
        executable='spawn_aruco.py',
        namespace='spawn_cubes',
        output='screen',
        arguments=['--seed', aruco_seed]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))
    ld.add_action(aruco_seed_launch_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_aruco_cubes)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(navigation_cmd)
    

    return ld
