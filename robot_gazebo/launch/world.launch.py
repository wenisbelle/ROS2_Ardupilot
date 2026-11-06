#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def world_path(world):
    """'runway' -> <robot_gazebo>/worlds/runway.sdf ; a path is returned as it is."""
    if world.endswith('.sdf') or os.path.sep in world:
        return os.path.abspath(os.path.expanduser(world))
    return os.path.join(get_package_share_directory('robot_gazebo'), 'worlds', f'{world}.sdf')


def launch_gazebo(context):
    world = world_path(LaunchConfiguration('world').perform(context))
    if not os.path.isfile(world):
        raise RuntimeError(f'World not found: {world}')

    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    extra = LaunchConfiguration('gz_extra_args').perform(context)
    verbosity = LaunchConfiguration('verbosity').perform(context)

    # -r: start running. Without GUI: server only (-s) with headless rendering (cameras still work)
    gz_args = f'{extra} -r -v {verbosity} {world}'
    if not gui:
        gz_args = f'-s --headless-rendering {gz_args}'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )
    # The WindEffects system only acts after a first wind message: send the initial
    # wind (default calm), which also turns on the body drag of the drones
    wind = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'robot_gazebo', 'set_wind',
             '--speed', LaunchConfiguration('wind_speed').perform(context),
             '--from', LaunchConfiguration('wind_from').perform(context)],
        name='initial_wind', output='screen')])
    return [gazebo, wind]


# Starts Gazebo Sim (Harmonic) with one of the outdoor worlds. The drones are
# added later, each one by its own launch (drone_bringup drone.launch.py), so
# they can be started / stopped in different terminals.
def generate_launch_description():

    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')

    # Models of this package (model://landing_pad, ...) and installed packages
    # (model://robot_description/...). Must be set BEFORE starting Gazebo
    set_models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_robot_gazebo, 'models'))
    set_share_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_robot_gazebo))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='runway',
                              description='runway | farm | village | disaster, or the path of a .sdf world'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo server only (headless rendering)'),
        DeclareLaunchArgument('gz_extra_args', default_value='', description='Extra Gazebo arguments'),
        DeclareLaunchArgument('verbosity', default_value='3', description='Gazebo verbosity 0..4'),
        DeclareLaunchArgument('wind_speed', default_value='0.0', description='Initial mean wind [m/s]'),
        DeclareLaunchArgument('wind_from', default_value='0.0',
                              description='Initial wind direction: FROM, deg clockwise from North'),
        set_models_path,
        set_share_path,
        OpaqueFunction(function=launch_gazebo),
    ])
