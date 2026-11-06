# -*- coding: utf-8 -*-
"""Helpers shared by the launch files: locate a world and read its GPS origin.

The <spherical_coordinates> of the world is the single source of truth for:
  - the ArduPilot SITL home of every drone (--home lat,lon,alt,heading)
  - the origin of the swarm "map" frame (ENU) used by drone_control
"""
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def world_path(world):
    """'runway' -> <robot_gazebo>/worlds/runway.sdf ; a path is returned as an absolute path."""
    if world.endswith('.sdf') or os.path.sep in world:
        return os.path.abspath(os.path.expanduser(world))
    return os.path.join(get_package_share_directory('robot_gazebo'), 'worlds', f'{world}.sdf')


def read_world(world):
    """Returns {name, path, latitude, longitude, elevation, heading} of a world SDF."""
    path = world_path(world)
    if not os.path.isfile(path):
        raise RuntimeError(f'World not found: {path}')
    root = ET.parse(path).getroot()
    w = root.find('world')
    if w is None:
        raise RuntimeError(f'No <world> in {path}')
    info = {'name': w.get('name'), 'path': path,
            'latitude': 0.0, 'longitude': 0.0, 'elevation': 0.0, 'heading': 0.0}
    sc = w.find('spherical_coordinates')
    if sc is not None:
        for key, tag in (('latitude', 'latitude_deg'), ('longitude', 'longitude_deg'),
                         ('elevation', 'elevation'), ('heading', 'heading_deg')):
            el = sc.find(tag)
            if el is not None and el.text:
                info[key] = float(el.text)
    return info


def sitl_home(world):
    """ArduPilot SITL --home string of a world: 'lat,lon,alt,heading'."""
    i = read_world(world)
    return f"{i['latitude']:.8f},{i['longitude']:.8f},{i['elevation']:.2f},{i['heading']:.1f}"


def remove_model(world_name, model, timeout_ms=2000):
    """Removes a model from the running Gazebo world (no error if it does not exist).
    Returns True if Gazebo removed it."""
    try:
        out = subprocess.run(
            ['gz', 'service', '-s', f'/world/{world_name}/remove', '--reqtype', 'gz.msgs.Entity',
             '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms),
             '--req', f'name: "{model}" type: MODEL'],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 3.0)
        return 'data: true' in out.stdout
    except (OSError, subprocess.SubprocessError):
        return False
