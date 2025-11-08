#!/usr/bin/env python3
"""
Complete Navigation Launch File for nav_neupan
This launch file sequentially starts:
1. Gazebo simulation with house environment
2. Autonomous navigation system
3. Neupan ROS2 navigation node
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directories
    robot_simulation_dir = FindPackageShare('robot_simulation').find('robot_simulation')
    neupan_ros2_dir = FindPackageShare('neupan_ros2')
    
    # Build absolute path to map file
    default_map_path = os.path.join(robot_simulation_dir, 'maps', 'house_map.yaml')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time if true')
    
    declare_map_file_path_cmd = DeclareLaunchArgument(
        name='map_file_path',
        default_value=default_map_path,
        description='Path to the map file')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file_path = LaunchConfiguration('map_file_path')
    
    # Start house simulation with Gazebo
    house_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_simulation_dir, 'launch', 'house_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'True'  # Disable RViz from house_sim to avoid conflicts
        }.items()
    )
    
    # Log message before starting navigation
    log_before_nav = LogInfo(msg='Waiting for Gazebo and robot to initialize...')
    
    # Delay autonomous_navigation launch to allow simulation to fully start
    # Increased delay to ensure robot is spawned and TF is published
    autonomous_navigation_launch = TimerAction(
        period=8.0,  # Wait 8 seconds for Gazebo to fully initialize
        actions=[
            LogInfo(msg='Starting autonomous navigation system...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_simulation_dir, 'launch', 'autonomous_navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map_file_path': map_file_path
                }.items()
            )
        ]
    )
    
    # Delay neupan launch to allow navigation stack to initialize
    neupan_launch = TimerAction(
        period=15.0,  # Wait 15 seconds for navigation stack to fully initialize
        actions=[
            LogInfo(msg='Starting Neupan navigation node...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    neupan_ros2_dir,
                    '/launch/limo_diff_launch.py'
                ]),
                launch_arguments={}.items()
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_path_cmd)
    
    # Add actions in sequence with logging
    ld.add_action(log_before_nav)
    ld.add_action(house_sim_launch)
    ld.add_action(autonomous_navigation_launch)
    ld.add_action(neupan_launch)
    
    return ld
