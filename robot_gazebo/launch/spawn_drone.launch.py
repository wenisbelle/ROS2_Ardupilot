#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_gazebo.world_info import read_world, remove_model


# Generates the drone SDF (robot_description/sdf/drone.sdf.xacro) for one
# instance and spawns it in the running Gazebo world.
# The instance number sets the UDP port of the ArduPilotPlugin (9002 + 10*instance),
# which must match the SITL instance started for this drone.
def spawn(context):
    name = LaunchConfiguration('name').perform(context)
    frame = LaunchConfiguration('frame').perform(context)
    instance = LaunchConfiguration('instance').perform(context)
    camera = LaunchConfiguration('camera').perform(context)
    camera_pitch = LaunchConfiguration('camera_pitch').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_description = get_package_share_directory('robot_description')
    with open(os.path.join(pkg_description, 'config', 'frames.yaml')) as f:
        frames = yaml.safe_load(f)
    if frame not in frames:
        raise RuntimeError(f'Unknown frame "{frame}". Available: {list(frames)}')
    if not z:
        # standing on the ground (+ 3 cm: landing pads are 2 cm thick)
        z = str(frames[frame]['spawn_z'] + 0.03)

    doc = xacro.process_file(
        os.path.join(pkg_description, 'sdf', 'drone.sdf.xacro'),
        mappings={'frame': frame, 'name': name, 'instance': instance,
                  'camera': camera, 'camera_pitch': camera_pitch})
    out_dir = '/tmp/ardupilot_drones'
    os.makedirs(out_dir, exist_ok=True)
    sdf_file = os.path.join(out_dir, f'{name}.sdf')
    with open(sdf_file, 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

    args = ['-file', sdf_file, '-name', name, '-x', x, '-y', y, '-z', z, '-Y', yaw]
    if world:
        world_name = read_world(world)['name']
        args = ['-world', world_name] + args
        # a model left by a previous run (crash, kill) would make the creation fail
        if remove_model(world_name, name):
            print(f'[spawn {name}] removed the old "{name}" model from the world')

    return [
        LogInfo(msg=f'[spawn {name}] {frame} at ({x}, {y}, {z}) yaw {yaw} | SDF {sdf_file}'),
        Node(package='ros_gz_sim', executable='create', name=f'spawn_{name}',
             output='screen', arguments=args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='drone1', description='Model name (= ROS namespace)'),
        DeclareLaunchArgument('frame', default_value='x500v2', choices=['x500v2', 'hexa']),
        DeclareLaunchArgument('instance', default_value='0', description='ArduPilot SITL instance'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_pitch', default_value='default', description='rad, down positive'),
        DeclareLaunchArgument('world', default_value='',
                              description='World name/path (only to target the Gazebo world by name)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='', description='Empty = standing on the ground'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='rad, ENU (0 = nose to East)'),
        OpaqueFunction(function=spawn),
    ])
