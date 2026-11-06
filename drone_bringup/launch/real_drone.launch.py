#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def truthy(value):
    return str(value).lower() in ('true', '1', 'yes', 'on')


# REAL drone, on its companion computer (Raspberry Pi, Jetson...): the same
# MAVROS + drone_commander + Zenoh bridge as in simulation, without Gazebo / SITL.
# Your high level nodes (vision, AI, swarm) are started on top of it, unchanged.
#
#   ros2 launch drone_bringup real_drone.launch.py id:=1 fcu_url:=/dev/ttyACM0:921600 \
#        gcs_ip:=192.168.1.10 origin_lat:=-22.9068 origin_lon:=-43.1729
#
# It also works against a SITL (fcu_url:=tcp://127.0.0.1:5760) to test the
# exact real launch on the bench.
def launch_real(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    drone_id = int(get('id'))
    name = get('name') or f'drone{drone_id}'
    domain = get('domain_id') or str(drone_id)
    pkg_fc = get_package_share_directory('flight_controller')
    pkg_bringup = get_package_share_directory('drone_bringup')
    pkg_control = get_package_share_directory('drone_control')

    origin = {}
    if get('origin_lat') and get('origin_lon'):
        origin = {'origin_lat': float(get('origin_lat')), 'origin_lon': float(get('origin_lon')),
                  'origin_alt': float(get('origin_alt'))}

    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_fc, 'launch', 'mavros.launch.py')),
            launch_arguments={'name': name, 'fcu_url': get('fcu_url'), 'gcs_url': get('gcs_url'),
                              'tgt_system': get('sysid') or str(drone_id), 'use_sim_time': 'false',
                              'respawn': 'true'}.items()),
        Node(package='drone_control', executable='drone_commander', namespace=name, output='screen',
             parameters=[os.path.join(pkg_control, 'config', 'drone_commander.yaml'),
                         dict(name=name, id=drone_id, use_sim_time=False, **origin)]),
    ]
    if truthy(get('zenoh')):
        cmd = ['zenoh-bridge-ros2dds', get('zenoh_mode'), '-c', os.path.join(pkg_bringup, 'config', 'zenoh', 'swarm.json5'),
               '-d', domain, '-l', get('zenoh_listen'), '--no-multicast-scouting']
        if get('gcs_ip'):
            cmd += ['-e', f"tcp/{get('gcs_ip')}:7447"]
        for peer in [p for p in get('zenoh_peers').split(',') if p.strip()]:
            cmd += ['-e', peer.strip()]
        actions.append(ExecuteProcess(cmd=cmd, name=f'zenoh_{name}', output='screen',
                                      additional_env={'RUST_LOG': 'warn'}))

    return [LogInfo(msg=f'[{name}] REAL drone | fcu {get("fcu_url")} | ROS_DOMAIN_ID {domain}'),
            GroupAction(scoped=True, actions=actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('id', default_value='1', description='Drone id (name drone<id>, ROS domain)'),
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('sysid', default_value='', description='MAV_SYSID of the FC (default id)'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600',
                              description='Serial port:baud of the FC, or udp:// / tcp:// (mavlink-router, SITL)'),
        DeclareLaunchArgument('gcs_url', default_value='', description='Optional MAVLink forward to a GCS'),
        DeclareLaunchArgument('domain_id', default_value=''),
        DeclareLaunchArgument('origin_lat', default_value='', description='Swarm frame origin (same on all drones)'),
        DeclareLaunchArgument('origin_lon', default_value=''),
        DeclareLaunchArgument('origin_alt', default_value='0.0'),
        DeclareLaunchArgument('zenoh', default_value='true'),
        DeclareLaunchArgument('zenoh_mode', default_value='peer', choices=['peer', 'client']),
        DeclareLaunchArgument('gcs_ip', default_value='', description='IP of the GCS Zenoh router'),
        DeclareLaunchArgument('zenoh_peers', default_value='',
                              description='Extra endpoints, comma separated (tcp/192.168.1.12:7447,...)'),
        DeclareLaunchArgument('zenoh_listen', default_value='tcp/0.0.0.0:7447'),
        OpaqueFunction(function=launch_real),
    ])
