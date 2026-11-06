#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def truthy(value):
    return str(value).lower() in ('true', '1', 'yes', 'on')


# Ground control station (GCS) side of the swarm, in its own ROS domain (0):
#   - Zenoh router + bridge: the "base station" of the radio network. The drone
#     bridges connect to it (and to each other through gossip).
#   - swarm_monitor: markers / TF of every drone found (/<name>/state)
#   - RViz (swarm view) and, optionally, QGroundControl
#
#   ros2 launch drone_bringup gcs.launch.py [rviz:=false] [qgc:=true]
def launch_gcs(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    pkg = get_package_share_directory('drone_bringup')
    use_sim_time = truthy(get('use_sim_time'))

    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', get('domain_id'))]
    if truthy(get('zenoh')):
        actions.append(ExecuteProcess(
            cmd=['zenoh-bridge-ros2dds', 'router', '-c', os.path.join(pkg, 'config', 'zenoh', 'swarm.json5'),
                 '-d', get('domain_id'), '-l', get('zenoh_listen'), '--no-multicast-scouting'],
            name='zenoh_gcs', output='screen', additional_env={'RUST_LOG': get('zenoh_log')}))
    if truthy(get('monitor')):
        actions.append(Node(package='swarm_control', executable='swarm_monitor', output='screen',
                            parameters=[{'use_sim_time': use_sim_time}]))
    if truthy(get('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz_swarm', output='log',
                            arguments=['-d', os.path.join(pkg, 'rviz', 'swarm.rviz')],
                            parameters=[{'use_sim_time': use_sim_time}]))
    if truthy(get('qgc')):
        actions.append(ExecuteProcess(cmd=['qgroundcontrol'], name='qgroundcontrol', output='log'))
    return [GroupAction(scoped=True, actions=actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='0', description='ROS domain of the GCS'),
        DeclareLaunchArgument('zenoh', default_value='true'),
        DeclareLaunchArgument('zenoh_listen', default_value='tcp/0.0.0.0:7447',
                              description='Endpoint where the drone bridges connect'),
        DeclareLaunchArgument('zenoh_log', default_value='warn'),
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('qgc', default_value='false', description='Also open QGroundControl'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='A real GCS has no simulation clock'),
        OpaqueFunction(function=launch_gcs),
    ])
