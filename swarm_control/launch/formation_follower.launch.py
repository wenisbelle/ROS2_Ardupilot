#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Starts formation_follower ON a follower drone: namespace /<name>, ROS domain of
# that drone (default = number in the name, as drone_bringup/drone.launch.py).
#   ros2 launch swarm_control formation_follower.launch.py name:=drone2 leader:=drone1 offset:="[-4.0, 4.0, 0.0]"
def follower(context):
    name = LaunchConfiguration('name').perform(context)
    domain = LaunchConfiguration('domain_id').perform(context)
    if not domain:
        digits = ''.join(c for c in name if c.isdigit())
        domain = digits or '0'
    offset = [float(v) for v in LaunchConfiguration('offset').perform(context).strip('[] ').split(',')]
    return [GroupAction(scoped=True, actions=[
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        Node(package='swarm_control', executable='formation_follower', namespace=name, output='screen',
             parameters=[{'leader': LaunchConfiguration('leader').perform(context),
                          'offset': offset,
                          'rotate_with_leader': LaunchConfiguration('rotate_with_leader').perform(context) == 'true',
                          'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}]),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='drone2', description='Follower drone'),
        DeclareLaunchArgument('leader', default_value='drone1'),
        DeclareLaunchArgument('offset', default_value='[-4.0, 4.0, 0.0]',
                              description='[back/forward, left/right, up] m in the leader frame'),
        DeclareLaunchArgument('rotate_with_leader', default_value='true'),
        DeclareLaunchArgument('domain_id', default_value='', description='Default: number in the name'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=follower),
    ])
