#!/usr/bin/env python3
"""
ROS 2 Launch File for Google Cartographer 2D SLAM (MS200 LiDAR + BNO08X IMU)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default='/home/admin/cartographer')
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')

    return LaunchDescription([
        # 1. Static TF: base_link -> laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # 2. Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 3. MS200 LiDAR Node
        Node(
            package='cartographer_pi5',
            executable='ms200_laser_node.py',
            name='ms200_laser_node',
            output='screen'
        ),

        # 4. BNO08X IMU Node
        Node(
            package='cartographer_pi5',
            executable='bno08x_imu_node.py',
            name='bno08x_imu_node',
            output='screen'
        ),

        # 5. Google Cartographer 2D SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu')
            ]
        ),

        # 6. Cartographer Occupancy Grid Node (Publishes 2D Map on /map)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec
            ]
        )
    ])
