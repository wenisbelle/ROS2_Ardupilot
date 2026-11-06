#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# MAVROS for ONE drone: MAVLink <-> ROS 2 bridge with the ArduPilot flight controller.
# The same launch is used in simulation (fcu_url tcp://127.0.0.1:5760+10*i, SITL)
# and on the real drone (fcu_url /dev/ttyACM0:921600 or udp://..., companion computer).
#
# Namespace /<name> (MAVROS topics /<name>/mavros/...). Frames are prefixed with the drone name so that
# several drones can be displayed together:
#   <name>/odom       local EKF frame (ENU, origin = where the EKF got its GPS origin)
#   <name>/base_link  body (FLU)
def launch_mavros(context):
    name = LaunchConfiguration('name').perform(context)
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    gcs_url = LaunchConfiguration('gcs_url').perform(context)
    tgt_system = int(LaunchConfiguration('tgt_system').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    respawn = LaunchConfiguration('respawn').perform(context).lower() in ('true', '1', 'yes')

    pkg = get_package_share_directory('flight_controller')
    pluginlists = os.path.join(pkg, 'config', 'mavros', 'apm_pluginlists.yaml')
    # ArduPilot defaults shipped with MAVROS (time sync, plugin options...)
    apm_config = os.path.join(get_package_share_directory('mavros'), 'launch', 'apm_config.yaml')

    odom = f'{name}/odom'
    base_link = f'{name}/base_link'
    override = {
        '/**/sys': {'ros__parameters': {'heartbeat_mav_type': 'ONBOARD_CONTROLLER'}},
        '/**/global_position': {'ros__parameters': {
            'frame_id': odom, 'child_frame_id': base_link, 'tf.send': False,
            'tf.frame_id': odom, 'tf.child_frame_id': base_link}},
        '/**/local_position': {'ros__parameters': {
            'frame_id': odom, 'tf.send': False, 'tf.frame_id': odom, 'tf.child_frame_id': base_link}},
        '/**/imu': {'ros__parameters': {'frame_id': base_link}},
        '/**/setpoint_position': {'ros__parameters': {'tf.frame_id': odom}},
        '/**/setpoint_attitude': {'ros__parameters': {'tf.frame_id': odom}},
        '/**/guided_target': {'ros__parameters': {'tf.frame_id': odom}},
        '/**/distance_sensor': {'ros__parameters': {'config': (
            'rangefinder_pub:\n'
            '  id: 0\n'
            f'  frame_id: "{name}/rangefinder"\n'
            '  field_of_view: 0.0\n'
            '  send_tf: false\n'
            '  sensor_position: {x: 0.0, y: 0.0, z: -0.1}\n')}},
    }
    out_dir = '/tmp/ardupilot_drones'
    os.makedirs(out_dir, exist_ok=True)
    override_file = os.path.join(out_dir, f'{name}_mavros.yaml')
    with open(override_file, 'w') as f:
        yaml.safe_dump(override, f, sort_keys=False)

    # MAVROS 2 topics are private to its node "mavros": namespace /<name> -> /<name>/mavros/...
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        namespace=name,
        output='screen',
        respawn=respawn,
        respawn_delay=3.0,
        parameters=[
            pluginlists,
            apm_config,
            override_file,
            {
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': tgt_system,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                # MAVROS identifies itself as the onboard computer of the same vehicle
                'system_id': tgt_system,
                'component_id': 191,
                'use_sim_time': use_sim_time,
            },
        ],
    )
    return [LogInfo(msg=f'[mavros {name}] fcu_url {fcu_url} | sysid {tgt_system} | topics /{name}/mavros/...'), mavros]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='drone1', description='Drone name (namespace)'),
        DeclareLaunchArgument('fcu_url', default_value='tcp://127.0.0.1:5760',
                              description='SITL: tcp://127.0.0.1:5760+10*i | real: /dev/ttyACM0:921600'),
        DeclareLaunchArgument('gcs_url', default_value='',
                              description='Optional MAVLink forward to a GCS, e.g. udp://@192.168.1.10:14550'),
        DeclareLaunchArgument('tgt_system', default_value='1', description='MAV_SYSID of the flight controller'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('respawn', default_value='false'),
        OpaqueFunction(function=launch_mavros),
    ])
