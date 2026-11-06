# -*- coding: utf-8 -*-
"""Geodesy and rotation helpers (no external dependencies).

Swarm frame "map": ENU (x East, y North, z Up) at the swarm origin (lat0, lon0).

ap_latlon_to_en / ap_en_to_latlon use EXACTLY the flat-earth model of ArduPilot
(AP_Common/Location.cpp: LATLON_TO_M, longitude scaled by cos(mean latitude)).
The local frame of every ArduPilot EKF is built with this model, and SITL also
uses it to turn the Gazebo position into GPS: with the same model the map frame
is a pure translation of each drone's local frame (and, in simulation, equal to
the Gazebo world frame). Exact WGS84 ENU differs by ~0.5 % (0.25 m at 50 m).

geodetic_to_enu / enu_to_geodetic: exact WGS84 (through ECEF), for other uses
(e.g. geolocating camera detections far away).
"""
import math

# ArduPilot AP_Math/definitions.h: LATLON_TO_M = 0.011131884502145034 m per 1e-7 deg
AP_DEG_TO_M = 0.011131884502145034 * 1.0e7


def _ap_longitude_scale(lat_deg):
    return max(math.cos(math.radians(lat_deg)), 0.01)


def ap_latlon_to_en(lat, lon, lat0, lon0):
    """(east, north) [m] of (lat, lon) from (lat0, lon0) with the ArduPilot model."""
    dlon = (lon - lon0 + 180.0) % 360.0 - 180.0
    north = (lat - lat0) * AP_DEG_TO_M
    east = dlon * AP_DEG_TO_M * _ap_longitude_scale((lat + lat0) / 2.0)
    return east, north


def ap_en_to_latlon(east, north, lat0, lon0):
    """Inverse of ap_latlon_to_en (ArduPilot Location::offset_latlng)."""
    dlat = north / AP_DEG_TO_M
    dlon = east / AP_DEG_TO_M / _ap_longitude_scale(lat0 + dlat / 2.0)
    return lat0 + dlat, (lon0 + dlon + 180.0) % 360.0 - 180.0


_A = 6378137.0                  # WGS84 semi-major axis [m]
_F = 1.0 / 298.257223563        # WGS84 flattening
_E2 = _F * (2.0 - _F)           # first eccentricity squared
_B = _A * (1.0 - _F)            # semi-minor axis


def geodetic_to_ecef(lat, lon, alt):
    lat, lon = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    x = (n + alt) * math.cos(lat) * math.cos(lon)
    y = (n + alt) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _E2) + alt) * math.sin(lat)
    return x, y, z


def ecef_to_geodetic(x, y, z):
    """Bowring's method (sub-millimetre for terrestrial altitudes)."""
    ep2 = (_A ** 2 - _B ** 2) / _B ** 2
    p = math.hypot(x, y)
    th = math.atan2(z * _A, p * _B)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2 * _B * math.sin(th) ** 3, p - _E2 * _A * math.cos(th) ** 3)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    x, y, z = geodetic_to_ecef(lat, lon, alt)
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    return e, n, u


def enu_to_geodetic(e, n, u, lat0, lon0, alt0):
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    la, lo = math.radians(lat0), math.radians(lon0)
    dx = -math.sin(lo) * e - math.sin(la) * math.cos(lo) * n + math.cos(la) * math.cos(lo) * u
    dy = math.cos(lo) * e - math.sin(la) * math.sin(lo) * n + math.cos(la) * math.sin(lo) * u
    dz = math.cos(la) * n + math.sin(la) * u
    return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rotation about z) of a quaternion, rad."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """(x, y, z, w) of a pure yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))
