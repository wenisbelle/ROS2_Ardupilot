#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def truthy(value):
    return str(value).lower() in ('true', '1', 'yes', 'on')


# Everything in one command: Gazebo world + GCS (Zenoh router, monitor, RViz)
# + the drones described in a YAML file (config/swarm.yaml), started one after
# the other (`stagger` seconds) so that Gazebo / SITL are not overloaded.
#
#   ros2 launch drone_bringup swarm.launch.py
#   ros2 launch drone_bringup swarm.launch.py config:=/path/my_swarm.yaml world:=farm gui:=false
#
# To develop, it is usually better to start the parts in separate terminals
# (world, gcs, one drone.launch.py per drone): see the README.
def isolated(include):
    """Included launch files share the launch configurations of the parent: without a scope
    the arguments of one include (domain_id of the GCS, profile of drone1...) would leak into
    the next ones. forwarding=False: the include only sees the arguments given to it."""
    return GroupAction(scoped=True, forwarding=False, actions=[include])


def launch_swarm(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    pkg = get_package_share_directory('drone_bringup')
    config = get('config') or os.path.join(pkg, 'config', 'swarm.yaml')
    with open(config) as f:
        cfg = yaml.safe_load(f)
    world = get('world') or cfg.get('world', 'runway')
    defaults = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in (cfg.get('defaults') or {}).items()}
    drones = cfg.get('drones') or []
    stagger = float(get('stagger'))

    actions = [LogInfo(msg=f'[swarm] {config}: {len(drones)} drones in {world}')]
    if truthy(get('start_world')):
        actions.append(isolated(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'world.launch.py')),
            launch_arguments={'world': world, 'gui': get('gui')}.items())))
    if truthy(get('start_gcs')):
        actions.append(isolated(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gcs.launch.py')),
            launch_arguments={'rviz': get('rviz'), 'qgc': get('qgc')}.items())))

    first_delay = 8.0 if truthy(get('start_world')) else 1.0
    for k, d in enumerate(drones):
        if 'id' not in d:
            raise RuntimeError(f'swarm config {config}: every drone needs an "id" ({d})')
        args = dict(defaults)
        args.update({key: str(v).lower() if isinstance(v, bool) else str(v) for key, v in d.items()})
        args['world'] = world
        actions.append(TimerAction(period=first_delay + k * stagger, actions=[isolated(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'drone.launch.py')),
            launch_arguments=args.items()))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='Swarm YAML (default config/swarm.yaml)'),
        DeclareLaunchArgument('world', default_value='', description='Overrides the world of the YAML'),
        DeclareLaunchArgument('start_world', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('start_gcs', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('qgc', default_value='false'),
        DeclareLaunchArgument('stagger', default_value='4.0', description='Seconds between drones'),
        OpaqueFunction(function=launch_swarm),
    ])
