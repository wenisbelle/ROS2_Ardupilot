#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ROS <-> Gazebo bridge of ONE drone (runs in the ROS domain of the drone):
#   /clock                                  sim time at `clock_rate` Hz (clock_relay; every domain needs its own)
#   /<name>/camera/image_raw, camera_info   camera (lazy: rendered only when subscribed)
#   /<name>/ground_truth/odometry           true pose / velocity (simulation only)
# The flight data (IMU, GPS, attitude...) does NOT go through this bridge:
# Gazebo -> ArduPilot SITL (JSON) -> MAVLink -> MAVROS, exactly as on the real drone.
def bridge(context):
    name = LaunchConfiguration('name').perform(context)
    camera = LaunchConfiguration('camera').perform(context).lower() in ('true', '1', 'yes')

    entries = [
        {'ros_topic_name': f'/{name}/ground_truth/odometry', 'gz_topic_name': f'/{name}/ground_truth/odometry',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
         'direction': 'GZ_TO_ROS', 'lazy': True},
    ]
    if camera:
        entries += [
            {'ros_topic_name': f'/{name}/camera/image_raw', 'gz_topic_name': f'/{name}/camera/image',
             'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image',
             'direction': 'GZ_TO_ROS', 'lazy': True},
            {'ros_topic_name': f'/{name}/camera/camera_info', 'gz_topic_name': f'/{name}/camera/camera_info',
             'ros_type_name': 'sensor_msgs/msg/CameraInfo', 'gz_type_name': 'gz.msgs.CameraInfo',
             'direction': 'GZ_TO_ROS', 'lazy': True},
        ]

    out_dir = '/tmp/ardupilot_drones'
    os.makedirs(out_dir, exist_ok=True)
    config = os.path.join(out_dir, f'{name}_gz_bridge.yaml')
    with open(config, 'w') as f:
        yaml.safe_dump(entries, f, sort_keys=False)

    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  namespace=name, output='screen',
                  parameters=[{'config_file': config, 'use_sim_time': True}])]
    # Gazebo publishes its clock at 1 kHz (physics step); relayed at a lower rate
    # (1 kHz /clock costs ~1 CPU core per drone with MAVROS + Python nodes)
    if LaunchConfiguration('clock').perform(context).lower() in ('true', '1', 'yes'):
        nodes.append(Node(package='robot_gazebo', executable='clock_relay', name='clock_relay',
                          namespace=name, output='screen',
                          parameters=[{'rate': float(LaunchConfiguration('clock_rate').perform(context))}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='drone1'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('clock', default_value='true', description='Publish /clock in this domain'),
        DeclareLaunchArgument('clock_rate', default_value='100.0', description='/clock rate [Hz]'),
        OpaqueFunction(function=bridge),
    ])
