#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_gazebo.world_info import read_world, remove_model


def truthy(value):
    return str(value).lower() in ('true', '1', 'yes', 'on')


# ONE simulated drone, as it would be in reality: its own "computer" (ROS domain),
# its flight controller (ArduPilot SITL) and its radio link (Zenoh bridge).
#
#   Gazebo model (robot_gazebo/spawn_drone)  <-- JSON / UDP 9002+10*i -->  ArduPilot SITL (flight_controller/sitl)
#   ArduPilot SITL  <-- MAVLink tcp 5760+10*i -->  MAVROS (/<name>/mavros)
#   ArduPilot SITL  --- MAVLink udp 14550 ------>  QGroundControl
#   MAVROS  <-->  drone_commander (/<name>/cmd/*, /<name>/state, services)
#   ROS domain <id>  <-- zenoh-bridge-ros2dds -->  other drones / GCS
#
#   ros2 launch drone_bringup drone.launch.py id:=1
#   ros2 launch drone_bringup drone.launch.py id:=2 frame:=hexa world:=farm
def launch_drone(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    drone_id = int(get('id'))
    if drone_id < 1:
        raise RuntimeError('id must be >= 1 (MAVLink system id)')
    name = get('name') or f'drone{drone_id}'
    instance = int(get('instance')) if get('instance') else drone_id - 1
    domain = int(get('domain_id')) if get('domain_id') else drone_id
    frame = get('frame')
    world = get('world')
    x = float(get('x')) if get('x') else 3.0 * (drone_id - 1)
    y = float(get('y')) if get('y') else 0.0
    yaw = float(get('yaw'))
    camera = truthy(get('camera'))
    use_zenoh = truthy(get('zenoh'))

    winfo = read_world(world)
    home = f"{winfo['latitude']:.8f},{winfo['longitude']:.8f},{winfo['elevation']:.2f},0"

    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_fc = get_package_share_directory('flight_controller')
    pkg_bringup = get_package_share_directory('drone_bringup')
    pkg_control = get_package_share_directory('drone_control')
    with open(os.path.join(get_package_share_directory('robot_description'), 'config', 'frames.yaml')) as f:
        frame_cfg = yaml.safe_load(f)[frame]
    cam_pitch = frame_cfg['camera_pitch'] if get('camera_pitch') == 'default' else float(get('camera_pitch'))

    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))]

    # ---------------- Gazebo model ----------------
    if truthy(get('spawn')):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_robot_gazebo, 'launch', 'spawn_drone.launch.py')),
            launch_arguments={'name': name, 'frame': frame, 'instance': str(instance),
                              'camera': str(camera).lower(), 'camera_pitch': get('camera_pitch'),
                              'world': world, 'x': str(x), 'y': str(y), 'yaw': str(yaw)}.items()))

    # ---------------- ArduPilot SITL (flight controller) ----------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_fc, 'launch', 'sitl.launch.py')),
        launch_arguments={'name': name, 'instance': str(instance), 'sysid': str(drone_id), 'frame': frame,
                          'profile': get('profile'), 'home': home, 'extra_params': get('extra_params'),
                          'speedup': get('speedup'), 'wipe': get('wipe'), 'gcs_udp': get('gcs_udp')}.items()))

    # ---------------- Gazebo <-> ROS (clock, camera, ground truth) ----------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_gazebo, 'launch', 'drone_bridge.launch.py')),
        launch_arguments={'name': name, 'camera': str(camera).lower(), 'clock': get('clock'),
                          'clock_rate': get('clock_rate')}.items()))

    if camera:
        actions += [
            Node(package='tf2_ros', executable='static_transform_publisher', name='camera_tf', namespace=name,
                 arguments=['--x', str(frame_cfg['camera_x']), '--z', str(frame_cfg['camera_z']),
                            '--pitch', str(cam_pitch),
                            '--frame-id', f'{name}/base_link', '--child-frame-id', f'{name}/camera_link'],
                 parameters=[{'use_sim_time': True}]),
            Node(package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf', namespace=name,
                 arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                            '--frame-id', f'{name}/camera_link', '--child-frame-id', f'{name}/camera_optical_frame'],
                 parameters=[{'use_sim_time': True}]),
        ]

    # ---------------- MAVROS + drone_commander (after SITL opened its ports) ----------------
    late = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_fc, 'launch', 'mavros.launch.py')),
            launch_arguments={'name': name, 'fcu_url': f'tcp://127.0.0.1:{5760 + 10 * instance}',
                              'tgt_system': str(drone_id), 'use_sim_time': 'true', 'respawn': 'true'}.items()),
        Node(package='drone_control', executable='drone_commander', namespace=name, output='screen',
             parameters=[os.path.join(pkg_control, 'config', 'drone_commander.yaml'),
                         {'name': name, 'id': drone_id,
                          'origin_lat': winfo['latitude'], 'origin_lon': winfo['longitude'],
                          'origin_alt': winfo['elevation'], 'use_sim_time': True}]),
    ]

    # ---------------- Zenoh bridge (the radio link of this drone) ----------------
    if use_zenoh:
        listen = get('zenoh_listen') or f'tcp/127.0.0.1:{7447 + drone_id}'
        late.append(ExecuteProcess(
            cmd=['zenoh-bridge-ros2dds', get('zenoh_mode'),
                 '-c', os.path.join(pkg_bringup, 'config', 'zenoh', 'swarm.json5'),
                 '-d', str(domain), '-e', get('zenoh_connect'), '-l', listen,
                 '--no-multicast-scouting'],
            name=f'zenoh_{name}', output='screen', additional_env={'RUST_LOG': get('zenoh_log')}))

    # The timer fires after the scoped group below has restored the environment:
    # the delayed actions need their own scope with the domain of the drone.
    actions.append(TimerAction(period=3.0, actions=[GroupAction(
        scoped=True, actions=[SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))] + late)]))

    # Ctrl-C in the terminal of this drone also removes its model from Gazebo
    if truthy(get('spawn')):
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _ctx: [LogInfo(msg=f'[{name}] removed from Gazebo')]
            if remove_model(winfo['name'], name) else [])])))

    info = (f'[{name}] id {drone_id} | {frame} | ROS_DOMAIN_ID {domain} | SITL instance {instance} | '
            f'spawn ({x:.1f}, {y:.1f}) in {winfo["name"]} | zenoh {"on" if use_zenoh else "off"}')
    return [LogInfo(msg=info), GroupAction(scoped=True, actions=actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('id', default_value='1',
                              description='Drone id >= 1: MAV_SYSID, ROS domain, name drone<id>, spawn x = 3*(id-1)'),
        DeclareLaunchArgument('name', default_value='', description='Default drone<id>'),
        DeclareLaunchArgument('frame', default_value='x500v2', choices=['x500v2', 'hexa']),
        DeclareLaunchArgument('world', default_value='runway',
                              description='World running in Gazebo (GPS origin): runway | farm | village | disaster | path'),
        DeclareLaunchArgument('x', default_value='', description='Spawn x [m] (default 3*(id-1), pads of the worlds)'),
        DeclareLaunchArgument('y', default_value='', description='Spawn y [m] (default 0)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad] ENU (0 = East)'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_pitch', default_value='default', description='rad, down positive'),
        DeclareLaunchArgument('profile', default_value='ideal', choices=['ideal', 'realistic'],
                              description='ArduPilot sensor profile'),
        DeclareLaunchArgument('extra_params', default_value='', description='Extra .parm files (comma separated)'),
        DeclareLaunchArgument('speedup', default_value='1'),
        DeclareLaunchArgument('clock', default_value='true',
                              description='Publish /clock in the domain (only ONE drone per domain needs it)'),
        DeclareLaunchArgument('clock_rate', default_value='100.0',
                              description='/clock rate in the drone domain [Hz] (Gazebo: 1000)'),
        DeclareLaunchArgument('wipe', default_value='true', description='Reset ArduPilot parameters at start'),
        DeclareLaunchArgument('gcs_udp', default_value='127.0.0.1:14550', description='QGroundControl address'),
        DeclareLaunchArgument('instance', default_value='', description='SITL instance (default id-1)'),
        DeclareLaunchArgument('domain_id', default_value='', description='ROS_DOMAIN_ID (default id)'),
        DeclareLaunchArgument('spawn', default_value='true', description='Spawn the model in Gazebo'),
        DeclareLaunchArgument('zenoh', default_value='true', description='Start the Zenoh bridge of this drone'),
        DeclareLaunchArgument('zenoh_mode', default_value='peer', choices=['peer', 'client']),
        DeclareLaunchArgument('zenoh_connect', default_value='tcp/127.0.0.1:7447',
                              description='Zenoh router (GCS) endpoint'),
        DeclareLaunchArgument('zenoh_listen', default_value='', description='Default tcp/127.0.0.1:<7447+id>'),
        DeclareLaunchArgument('zenoh_log', default_value='warn', description='RUST_LOG of the bridge'),
        OpaqueFunction(function=launch_drone),
    ])
